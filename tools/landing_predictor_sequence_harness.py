"""Audit one V1 landing against its capture, guidance trace, and KSP state log.

This is an observational equivalence gate. Unknown controller or terrain inputs
remain unknown; a passing publication-lifecycle audit cannot make them valid.
"""

import argparse
import bisect
import json
import math
import re
import tempfile
from collections import Counter, defaultdict
from pathlib import Path

from landing_predictor_capture_reader import read_capture
from landing_predictor_lifecycle_harness import audit_lifecycle, TERMINAL_PHASES


STATE_PREFIX = "[MechJebLandingTrace] state step="
STATE_FIELD = re.compile(r"([A-Za-z]+)=([^ ]+)")
OBSERVED_FIELDS = ("ut", "warp", "alt", "surfaceSpeed", "verticalSpeed",
                   "horizontalSpeed", "throttle", "thrustAccel", "maxAccel",
                   "attitudeError", "predictionVersion")


def read_one_session(path, session=None, allow_incomplete=False):
    """Validate one append-only capture session without hiding its own bad rows."""
    if not allow_incomplete:
        return read_capture(path), session
    if session is None:
        with Path(path).open(encoding="utf-8-sig") as stream:
            for line in stream:
                try:
                    item = json.loads(line)
                except json.JSONDecodeError:
                    continue
                session = item.get("captureSession", session)
    if not session:
        raise ValueError("capture contains no complete session record")
    selected = []
    inside = False
    with Path(path).open(encoding="utf-8-sig") as stream:
        for line in stream:
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                if inside:
                    selected.append(line)
                continue
            if item.get("captureSession") == session:
                inside = True
                selected.append(line)
            elif inside:
                break
    if not selected:
        raise ValueError(f"capture has no session {session}")
    with tempfile.TemporaryDirectory() as directory:
        selected_path = Path(directory) / "selected-session.jsonl"
        selected_path.write_text("".join(selected), encoding="utf-8")
        return read_capture(selected_path, allow_incomplete=True), session


def read_trace(path, start_ut, end_ut):
    states = []
    terrain = []
    malformed = []
    with Path(path).open(encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                malformed.append(line_number)
                continue
            ut = item.get("ut", item.get("processUT"))
            if not isinstance(ut, (int, float)) or not start_ut <= ut <= end_ut:
                continue
            item["lineNumber"] = line_number
            if item.get("recordType") in ("guidance_state", "guidance_transition"):
                states.append(item)
            elif item.get("recordType") == "predictor_result" and item.get("terrainProfileApplied"):
                terrain.append(item)
    states.sort(key=lambda state: state["ut"])
    return states, terrain, malformed


def read_ksp_states(path, start_ut, end_ut):
    states = []
    with Path(path).open(encoding="utf-8", errors="replace") as stream:
        for line_number, line in enumerate(stream, 1):
            if STATE_PREFIX not in line:
                continue
            fields = dict(STATE_FIELD.findall(line.split(STATE_PREFIX, 1)[1]))
            try:
                state = {name: float(fields[name]) for name in OBSERVED_FIELDS}
            except (KeyError, ValueError):
                continue
            if start_ut <= state["ut"] <= end_ut:
                state["phase"] = line.split(STATE_PREFIX, 1)[1].split(" ", 1)[0]
                state["lineNumber"] = line_number
                states.append(state)
    states.sort(key=lambda state: state["ut"])
    return states


def nearest_state(states, ut):
    if not states:
        return None
    times = [state["ut"] for state in states]
    index = bisect.bisect_left(times, ut)
    options = states[max(0, index - 1):min(len(states), index + 1)]
    return min(options, key=lambda state: abs(state["ut"] - ut))


def terminal_observation(states):
    samples = [state for state in states
               if state.get("phase") == "KillHorizontalVelocity" and
               state.get("recordType") == "guidance_state"]
    def summary(state):
        if state is None:
            return None
        velocity, up, forward = (state.get(name) for name in
                                 ("surfaceVelocity", "up", "forward"))
        if not all(isinstance(vector, list) and len(vector) == 3
                   for vector in (velocity, up, forward)):
            return None
        vertical = sum(a * b for a, b in zip(velocity, up))
        horizontal = [v - vertical * u for v, u in zip(velocity, up)]
        forward_up = sum(a * b for a, b in zip(forward, up))
        forward_horizontal = [f - forward_up * u for f, u in zip(forward, up)]
        hspeed = math.sqrt(sum(value * value for value in horizontal))
        fmagnitude = math.sqrt(sum(value * value for value in forward_horizontal))
        alignment = (sum(a * b for a, b in zip(horizontal, forward_horizontal)) /
                     (hspeed * fmagnitude)) if hspeed * fmagnitude > 0 else None
        gravity = state.get("localGravity")
        # The corrected V1 hover policy tilts thrust 0.2 laterally. Even with
        # instantaneous attitude, its lateral acceleration is about 0.2 g.
        ideal_stop_time = hspeed / (0.2 * gravity) if isinstance(
            gravity, (int, float)) and gravity > 0 else None
        ideal_stop_distance = hspeed * ideal_stop_time / 2 if ideal_stop_time else None
        return {"ut": state["ut"], "terrainClearance": state.get("altitudeTrue"),
                "horizontalSpeed": hspeed, "verticalSpeed": vertical,
                "forwardHorizontalAlignment": alignment,
                "commandedThrottle": state.get("commandedThrottle"),
                "idealHorizontalStopTime": ideal_stop_time,
                "idealHorizontalStopDistance": ideal_stop_distance}
    if not samples:
        return None
    initial_error = samples[0].get("attitudeErrorDegrees")
    settled = next((state for state in samples
                    if isinstance(state.get("attitudeErrorDegrees"), (int, float)) and
                    state["attitudeErrorDegrees"] < 5), None)
    return {"first": summary(samples[0]), "last": summary(samples[-1]),
            "samples": len(samples), "initialAttitudeError": initial_error,
            "attitudeSettlingSeconds": (settled["ut"] - samples[0]["ut"])
            if settled and isinstance(initial_error, (int, float)) and
            initial_error >= 5 else None}


def audit_sequence(document, guidance_states, ksp_states, terrain_results,
                   session=None, malformed_trace_lines=()):
    submissions = [case for case in document["cases"] if case["submission"] and
                   case["submission"].get("kind") == "target_aware_transaction"]
    if not submissions:
        raise ValueError("capture has no target-aware submissions")
    if session is None:
        session = submissions[-1]["captureSession"]
    cases = {case["submissionId"]: case for case in document["cases"]
             if case["captureSession"] == session}
    events = [event for event in document["events"]
              if event["captureSession"] == session]
    phases = defaultdict(lambda: {"traceSamples": 0, "transitionSamples": 0,
                                  "kspSamples": 0,
                                  "maximumThrottle": None, "maximumWarp": None,
                                  "minimumAttitudeError": None,
                                  "correctionPulseObservations": 0})
    issues = []
    unknowns = []
    publications = []
    failures = Counter()
    failures_after_commit = Counter()
    active = None
    for event in events:
        kind = event["recordType"]
        if kind == "submission" and event.get("kind") == "target_aware_transaction" and \
                event.get("phase") in TERMINAL_PHASES and not (
                    event.get("phase") == "DecelerationBurn" and
                    event.get("modelProvenance") in (
                        "v1_live_braking_forecast", "v1_no_further_correction_baseline")):
            issues.append(f"submission {event['submissionId']}: terrain candidate search in {event['phase']}")
        if kind == "selection_decision" and str(event.get("decision", "")).startswith("target_aware_failed:"):
            reason = event["decision"].split(":", 1)[1]
            failures[reason] += 1
            if active is not None:
                failures_after_commit[reason] += 1
        if kind != "published":
            continue
        case = cases.get(event["submissionId"])
        submission = case["submission"] if case else None
        if submission is None:
            issues.append(f"version {event['resultVersion']}: publication has no submission")
            continue
        model = submission.get("kind")
        if model != "target_aware_transaction":
            issues.append(f"version {event['resultVersion']}: incompatible {model} publication")
            continue
        worker = case.get("worker") or {}
        resolved = case.get("resolved") or {}
        brake_ut = worker.get("controllerBrakeReferenceUT", worker.get("virtualBrakeUT"))
        if brake_ut is None:
            brake_ut = worker.get("virtualBrakeUT")
        trajectory_start = worker.get("trajectoryStart")
        brake_position = worker.get("controllerBrakeReferencePosition") or trajectory_start
        publication = {
            "submissionId": event["submissionId"], "version": event["resultVersion"],
            "inputUT": submission["inputUT"], "publishedUT": event["processUT"],
            "inputPhase": submission.get("phase"), "brakeUT": brake_ut,
            "predictedBrakeASL": (brake_position[2] - submission["bodyRadius"]
                                  if isinstance(brake_position, list) else None),
            "trajectoryStartUT": trajectory_start[3] if isinstance(trajectory_start, list) else None,
            "predictedEndpoint": worker.get("simulatorEnd"),
            "predictedEndpointTerrainASL": resolved.get("resolvedEndASL"),
            "modelProvenance": submission.get("modelProvenance"),
            "forecastKind": worker.get("forecastKind"),
        }
        publications.append(publication)
        active = publication
        controller_model = publication["modelProvenance"] in (
            "v1_controller_equivalent", "v1_controller_policy_forecast",
            "v1_live_braking_forecast", "v1_no_further_correction_baseline",
            "v1_safe_brake_timing_no_further_correction")
        if not controller_model:
            issues.append(f"version {active['version']}: active result lacks V1 controller-equivalent provenance")
        elif (brake_ut is None and publication["forecastKind"] == "ImpactForecast"):
            pass  # Terrain contact can precede V1's nominal speed trigger.
        elif brake_ut is None or worker.get("controllerBrakeReferenceUT") is None or \
                worker.get("controllerBrakeReferencePosition") is None or \
                publication["trajectoryStartUT"] is None or \
                publication["trajectoryStartUT"] > brake_ut:
            issues.append(f"version {active['version']}: controller brake reference or preceding path missing")

    active_lineage = {(item["version"], item["inputUT"]) for item in publications}
    linked_trace = [state for state in guidance_states
                    if (state.get("predictionVersion"),
                        state.get("predictionInputUT")) in active_lineage]
    first_active_line = min((state["lineNumber"] for state in linked_trace), default=None)
    active_trace = [state for state in guidance_states
                    if first_active_line is not None and
                    state["lineNumber"] >= first_active_line]
    trace_only = {(state.get("predictionVersion"), state.get("predictionInputUT"))
                  for state in active_trace if state.get("predictionVersion") and
                  (state.get("predictionVersion"),
                   state.get("predictionInputUT")) not in active_lineage}
    if trace_only:
        unknowns.append(f"trace-only result lineage absent from capture: {sorted(trace_only)}")
    if first_active_line is not None:
        terrain_results = [item for item in terrain_results
                           if item["lineNumber"] >= first_active_line]
        relevant_malformed = [line for line in malformed_trace_lines
                              if line >= first_active_line]
        if relevant_malformed:
            unknowns.append(f"malformed active-session trace rows: {relevant_malformed}")
    for state in active_trace:
        publication = next((item for item in publications if
                            item["version"] == state.get("predictionVersion") and
                            item["inputUT"] == state.get("predictionInputUT")), None)
        if publication and publication["forecastKind"] == "ImpactForecast" and \
                state["phase"] in ("CoastToDeceleration", "DecelerationBurn"):
            issues.append(f"version {publication['version']}: impact forecast authorized {state['phase']}")
        if publication and publication["modelProvenance"] == \
                "v1_no_further_correction_baseline" and state["phase"] in (
                    "CourseCorrection", "CoastToDeceleration", "DecelerationBurn") and \
                state["ut"] - publication["inputUT"] > 10:
            issues.append(f"version {publication['version']}: stale baseline used in {state['phase']}")
        phase = phases[state["phase"]]
        if state.get("recordType") == "guidance_transition":
            phase["transitionSamples"] += 1
            continue
        phase["traceSamples"] += 1
        phase["maximumThrottle"] = max(phase["maximumThrottle"] or 0,
                                       state.get("commandedThrottle") or 0)
        phase["maximumWarp"] = max(phase["maximumWarp"] or 0,
                                   state.get("warpRate") or 0)
        if "pulseDv=" in (state.get("controllerDetail") or ""):
            phase["correctionPulseObservations"] += 1
    for state in ksp_states:
        phase = phases[state["phase"]]
        phase["kspSamples"] += 1
        angle = state["attitudeError"]
        old = phase["minimumAttitudeError"]
        phase["minimumAttitudeError"] = angle if old is None else min(old, angle)

    for publication in publications:
        brake_ut = publication["brakeUT"]
        if not isinstance(brake_ut, (int, float)) or not math.isfinite(brake_ut):
            if publication["forecastKind"] != "ImpactForecast":
                unknowns.append(f"version {publication['version']}: brake time absent")
            continue
        nearest = nearest_state(ksp_states, brake_ut)
        if nearest and abs(nearest["ut"] - brake_ut) < 10:
            publication["actualAtBrakeUT"] = {
                "ut": nearest["ut"], "altitudeASL": nearest["alt"],
                "throttle": nearest["throttle"], "attitudeError": nearest["attitudeError"],
            }
            publication["brakeAltitudeDifference"] = (
                publication["predictedBrakeASL"] - nearest["alt"]
                if publication["predictedBrakeASL"] is not None else None)
            if publication["modelProvenance"] in (
                    "v1_controller_policy_forecast", "v1_no_further_correction_baseline") and \
                    publication["brakeAltitudeDifference"] is not None and \
                    abs(publication["brakeAltitudeDifference"]) > 200:
                issues.append(f"version {publication['version']}: V1 brake position diverged by over 200 m")
        else:
            unknowns.append(f"version {publication['version']}: no vessel state near brake UT")
        expected_burn_start = brake_ut - 5 if publication["modelProvenance"] in (
            "v1_controller_policy_forecast", "v1_controller_equivalent",
            "v1_no_further_correction_baseline") else brake_ut
        early = [state for state in ksp_states if state["phase"] == "DecelerationBurn" and
                 publication["inputUT"] <= state["ut"] < expected_burn_start - 0.2 and
                 state["throttle"] > 0]
        if early:
            first = early[0]
            publication["firstPreBrakeThrottle"] = {
                "ut": first["ut"], "secondsBeforePredictedBrake": brake_ut - first["ut"],
                "throttle": first["throttle"], "altitudeASL": first["alt"],
            }
            label = "forecast burn start" if publication["modelProvenance"] in (
                "v1_controller_policy_forecast", "v1_controller_equivalent",
                "v1_no_further_correction_baseline") else "forecast brake UT"
            issues.append(f"version {publication['version']}: V1 commanded throttle before {label}")
        changing_burn = [state for state in ksp_states if state["phase"] == "DeorbitBurn" and
                         state["ut"] > publication["inputUT"] and state["throttle"] > 0]
        if changing_burn:
            first = changing_burn[0]
            publication["postSnapshotDeorbitBurnUT"] = first["ut"]
            issues.append(f"version {publication['version']}: active snapshot precedes continuing deorbit burn")
        ages = [state["ut"] - publication["inputUT"] for state in active_trace
                if state.get("predictionVersion") == publication["version"] and
                state.get("predictionInputUT") == publication["inputUT"]]
        publication["maximumObservedInputAgeSeconds"] = max(ages) if ages else None
        if not ages:
            unknowns.append(f"version {publication['version']}: no active guidance observations")

    if not active_trace:
        unknowns.append("no guidance-state samples linked to a published result")
    if not ksp_states:
        unknowns.append("no KSP controller-state samples")
    if not any(state["phase"] == "KillHorizontalVelocity" for state in ksp_states):
        unknowns.append("no observed KillHorizontalVelocity controller state")
    if not any(state["phase"] == "FinalDescent" for state in ksp_states):
        unknowns.append("no observed FinalDescent controller state")
    if not any(state["phase"] == "CourseCorrection" and
               state.get("recordType") == "guidance_state" for state in active_trace):
        unknowns.append("no sampled CourseCorrection pulse/settling state")
    local_terrain = [item.get("resolvedEndpoint", [None] * 4)[3] for item in terrain_results
                     if item.get("phase") in TERMINAL_PHASES and
                     isinstance(item.get("resolvedEndpoint"), list)]
    local_terrain = [value for value in local_terrain if isinstance(value, (int, float))]
    if local_terrain:
        for publication in publications:
            endpoint_terrain = publication["predictedEndpointTerrainASL"]
            if isinstance(endpoint_terrain, (int, float)):
                # Airless V1 hands off at predicted endpoint terrain + 205 m.
                publication["v1TerminalHandoffASL"] = endpoint_terrain + 205
                if min(local_terrain) > publication["v1TerminalHandoffASL"]:
                    issues.append(f"version {publication['version']}: observed local terrain exceeds predicted V1 terminal handoff altitude")
    lifecycle = audit_lifecycle({"cases": list(cases.values()), "events": events})
    issues.extend(lifecycle["issues"])
    return {
        "session": session, "passed": not issues and not unknowns,
        "publications": publications, "phases": dict(phases),
        "failedTransactions": dict(failures),
        "failedRefreshesAfterCommit": dict(failures_after_commit),
        "terminalObservation": terminal_observation(active_trace),
        "terminalTerrainASLRange": [min(local_terrain), max(local_terrain)] if local_terrain else None,
        "issues": list(dict.fromkeys(issues)), "unknowns": unknowns,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("trace", type=Path)
    parser.add_argument("ksp_log", type=Path)
    parser.add_argument("--session")
    parser.add_argument("--allow-incomplete", action="store_true",
                        help="audit a truncated capture while reporting its gaps")
    args = parser.parse_args()
    document, selected_session = read_one_session(
        args.capture, args.session, args.allow_incomplete)
    submissions = [case for case in document["cases"] if case["submission"] and
                   case["submission"].get("kind") == "target_aware_transaction"]
    session = selected_session or submissions[-1]["captureSession"]
    events = [event for event in document["events"] if event["captureSession"] == session]
    epochs = [event.get("processUT", event.get("captureEpochUT")) for event in events]
    epochs = [value for value in epochs if isinstance(value, (int, float))]
    if not epochs:
        parser.error("selected session has no UT observations")
    start_ut = min(epochs)
    # Capture stops when terminal search stops. Follow the published lineage
    # through the trace so KillHorizontalVelocity and FinalDescent remain in
    # the whole-sequence audit even after the last predictor submission.
    guidance_states, terrain, malformed = read_trace(args.trace, start_ut, float("inf"))
    published_lineage = {(event["resultVersion"],
                          next((case["submission"]["inputUT"] for case in document["cases"]
                                if case["captureSession"] == session and
                                case["submissionId"] == event["submissionId"] and
                                case["submission"]), None))
                         for event in events if event["recordType"] == "published"}
    linked_lines = [state["lineNumber"] for state in guidance_states
                    if (state.get("predictionVersion"),
                        state.get("predictionInputUT")) in published_lineage]
    first_line = min(linked_lines, default=None)
    ordered = sorted((state for state in guidance_states
                      if first_line is not None and state["lineNumber"] >= first_line),
                     key=lambda state: state["lineNumber"])
    sequence = []
    previous_ut = None
    for state in ordered:
        if previous_ut is not None and state["ut"] < previous_ut - 60:
            break  # A later game session reset UT.
        sequence.append(state)
        previous_ut = state["ut"]
    end_ut = max((state["ut"] for state in sequence), default=max(epochs)) + 1
    last_line = sequence[-1]["lineNumber"] if sequence else first_line
    guidance_states = sequence
    terrain = [item for item in terrain if
               first_line is not None and first_line <= item["lineNumber"] <= last_line]
    malformed = [line for line in malformed if first_line is not None and
                 first_line <= line <= last_line + 1]
    ksp_states = read_ksp_states(args.ksp_log, start_ut, end_ut)
    report = audit_sequence(document, guidance_states, ksp_states, terrain, session,
                            malformed)
    session_gaps = [gap for gap in document["gaps"]
                    if session in gap or gap.startswith("line ")]
    if session_gaps:
        report["unknowns"].extend(session_gaps)
        report["passed"] = False
    print(json.dumps(report, indent=2, allow_nan=False))
    if not report["passed"]:
        parser.exit(1)


if __name__ == "__main__":
    main()
