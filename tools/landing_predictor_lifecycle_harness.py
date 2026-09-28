"""Audit target-aware V1 publication ownership in a deterministic capture.

Usage: python tools/landing_predictor_lifecycle_harness.py capture.jsonl

This checks the emitted production transaction/slot events. The captured Mun
baseline predates the new model, so it supplies numerical evidence but cannot
itself satisfy the target-aware lifecycle gates.
"""

import argparse
import json
import math
from pathlib import Path

from landing_predictor_capture_reader import CaptureError, read_capture


END_ACTIONS = {"landing_stopped", "landing_module_disabled",
               "predictor_disabled", "switched_to_untargeted"}
INVALIDATION_ACTIONS = {"target_aware_target_changed",
                        "target_aware_target_terrain_changed"}
TERMINAL_PHASES = {"DecelerationBurn", "KillHorizontalVelocity", "FinalDescent"}


def audit_lifecycle(document, require_active=False):
    """Return witnessed transitions and violations, preserving append order."""
    cases = {(case["captureSession"], case["submissionId"]): case
             for case in document["cases"]}
    active_sessions = {case["captureSession"] for case in document["cases"]
                       if case["submission"] and
                       case["submission"].get("kind") == "target_aware_transaction"}
    issues = []
    transitions = []
    phase_observations = []
    active_by_session = {}
    targeted_by_session = {}
    last_version_by_session = {}
    last_sequence_by_session = {}
    abandoned = set()
    active_publications = 0
    for event in document["events"]:
        session = event["captureSession"]
        kind = event["recordType"]
        line = event["lineNumber"]
        if kind == "lifecycle":
            action = event.get("action")
            if action == "target_reset":
                targeted_by_session[session] = session in active_sessions
                active_by_session[session] = None
                last_sequence_by_session.pop(session, None)
            elif action in END_ACTIONS:
                targeted_by_session[session] = False
                active_by_session[session] = None
                last_sequence_by_session.pop(session, None)
            elif action in INVALIDATION_ACTIONS:
                active_by_session[session] = None
                transitions.append({"line": line, "action": action,
                                    "activeSubmissionId": None})
            continue
        if kind == "submission" and event.get("kind") == "target_aware_transaction":
            targeted_by_session[session] = True
            if event.get("phase") in TERMINAL_PHASES and not (
                    event.get("phase") == "DecelerationBurn" and
                    event.get("modelProvenance") == "v1_live_braking_forecast"):
                issues.append(f"line {line}: target-aware planning submitted during {event['phase']}")
            case = cases.get((session, event["submissionId"]))
            refresh = next((item for item in (case.get("decisions", []) if case else [])
                            if item.get("decision") == "target_aware_refresh_started"), None)
            expected_active = active_by_session.get(session) or 0
            if refresh is None or refresh.get("comparedSubmissionId") != expected_active:
                issues.append(f"line {line}: refresh did not retain active target-aware result")
        if kind == "submission" and targeted_by_session.get(session):
            phase_observations.append({"line": line, "phase": event.get("phase"),
                                       "activeSubmissionId": active_by_session.get(session)})
        if kind == "submission_abandoned":
            abandoned.add((session, event["submissionId"]))
        if kind != "published":
            continue
        submission_id = event["submissionId"]
        case = cases.get((session, submission_id))
        submission = case["submission"] if case else None
        model = submission.get("kind") if submission else None
        if (session, submission_id) in abandoned:
            issues.append(f"line {line}: abandoned transaction was published")
        version = event["resultVersion"]
        previous_version = last_version_by_session.get(session)
        if previous_version is not None and version != previous_version + 1:
            issues.append(f"line {line}: result version jumped from {previous_version} to {version}")
        last_version_by_session[session] = version
        if targeted_by_session.get(session) and model != "target_aware_transaction":
            issues.append(f"line {line}: {model or 'unknown'} result published in target-aware mode")
        if model != "target_aware_transaction":
            continue
        if event.get("phase") in TERMINAL_PHASES and not (
                event.get("phase") == "DecelerationBurn" and
                submission.get("modelProvenance") == "v1_live_braking_forecast"):
            issues.append(f"line {line}: target-aware planning published during {event['phase']}")
        if not targeted_by_session.get(session):
            issues.append(f"line {line}: target-aware result published after landing stopped")
        active_publications += 1
        validation = case.get("validation") if case else None
        worker = case.get("worker") if case else None
        resolved = case.get("resolved") if case else None
        decisions = case.get("decisions", []) if case else []
        accepted = any(item.get("decision") == "target_aware_atomic_accept"
                       for item in decisions)
        if (not validation or validation.get("stage") != "Complete" or
                not validation.get("terrainResolved") or
                not validation.get("clearPath") or
                not validation.get("terminalNecessaryBoundPasses") or
                not worker or worker.get("recordType") != "worker_result" or
                not worker.get("complete") or not resolved or not accepted):
            issues.append(f"line {line}: target-aware publication lacks complete validation")
            continue
        if (validation.get("generation") != submission.get("generation") or
                validation.get("generation") != event.get("currentGeneration") or
                resolved.get("currentGeneration") != event.get("currentGeneration")):
            issues.append(f"line {line}: target-aware generation changed before publication")
        tolerance = max(200, submission["bodyRadius"] * 0.0005)
        age = event["processUT"] - submission["captureEpochUT"]
        if not (math.isfinite(age) and 0 <= age <= 10):
            issues.append(f"line {line}: target-aware snapshot exceeded provisional age gate")
        if validation.get("brakeTimeBracketed", True):
            if (abs(validation["signedDownrangeError"]) >= tolerance or
                    validation["timingDistanceEstimate"] >= tolerance or
                    validation["timingInterval"] <= 0):
                issues.append(f"line {line}: signed refinement did not meet V1 distance gate")
        elif not validation.get("directForecast", False) and not math.isfinite(
                validation["signedDownrangeError"]):
            issues.append(f"line {line}: unbracketed miss is not recorded")
        if (validation["terrainQueryCount"] > 1024 or
                validation["minimumSampledClearance"] < 0 or
                validation["handoffClearance"] < 0):
            issues.append(f"line {line}: terrain clearance or query cap failed")
        sequence = validation["sequence"]
        previous_sequence = last_sequence_by_session.get(session)
        if previous_sequence is not None and sequence <= previous_sequence:
            issues.append(f"line {line}: target-aware sequence did not advance")
        last_sequence_by_session[session] = sequence
        old_id = active_by_session.get(session)
        acceptance = next(item for item in decisions
                          if item.get("decision") == "target_aware_atomic_accept")
        if acceptance.get("comparedSubmissionId") != (old_id or 0):
            issues.append(f"line {line}: atomic acceptance compared the wrong predecessor")
        active_by_session[session] = submission_id
        transitions.append({"line": line, "action": "commit",
                            "phase": event.get("phase"),
                            "activeSubmissionId": submission_id,
                            "resultVersion": version,
                            "sequence": sequence})
    if require_active and active_publications == 0:
        issues.append("capture has no completed target-aware publication")
    return {"activePublications": active_publications,
            "transitions": transitions, "phaseObservations": phase_observations,
            "issues": issues,
            "passed": not issues}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--require-active", action="store_true")
    args = parser.parse_args()
    try:
        capture = read_capture(args.capture)
        report = audit_lifecycle(capture, args.require_active)
    except CaptureError as exc:
        parser.exit(2, f"lifecycle harness: {exc}\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        parser.exit(1)


if __name__ == "__main__":
    main()
