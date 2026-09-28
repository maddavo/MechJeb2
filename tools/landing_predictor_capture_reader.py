"""Validate V1 passive predictor capture and export a versioned replay input.

Usage: python tools/landing_predictor_capture_reader.py LandingGuidanceV1.capture.jsonl --output mun-replay.json
The export is input data for a future offline lifecycle harness; this reader does
not simulate, select, or alter a landing prediction.
"""

import argparse
import json
import math
import sys
from pathlib import Path


SCHEMA_VERSION = 1
RECORD_TYPES = {
    "submission", "worker_result", "worker_exception", "resolved_result",
    "published", "capture_error", "submission_abandoned", "discarded", "lifecycle",
    "selection_decision", "target_aware_validation", "target_aware_worker_stage",
}
INPUT_NUMBERS = (
    "inputUT", "captureEpochUT", "bodyRadius", "bodyMu", "bodyGeeASL",
    "rotationPeriod", "mass", "thrustAvailable", "thrustMinimum",
    "throttleFixedLimit", "limitedMaxThrustAcceleration",
    "maxEngineResponseTime", "targetLatitude", "targetLongitude",
    "targetTerrainASL", "targetTerrainQueryElapsedMs", "decelEndASL",
    "probableLandingSiteASL", "maxThrustAcceleration",
    "parachuteMultiplier", "dt", "minDt", "maxOrbits",
)
INPUT_VECTORS = ("angularVelocity", "bodyAxis0", "bodyAxis90", "bodyAxisNorth",
                 "positionBCI", "velocityBCI")
CONTROL_NUMBERS = ("currentThrustAcceleration", "maxThrustAccelerationAtSubmission",
                   "localGravity", "controllerDeltaT", "altitudeASL", "speedSurface",
                   "speedSurfaceHorizontal", "speedVertical",
                   "vesselAngularSpeed", "commandedThrottle", "landingTouchdownSpeed")
CONTROL_VECTORS = ("surfaceVelocity", "orbitalVelocity", "forward", "up",
                   "gravityForce")


class CaptureError(ValueError):
    pass


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _reject_constant(value):
    raise CaptureError(f"non-JSON numeric constant: {value}")


def _validate_submission(record, line_number):
    for name in INPUT_NUMBERS:
        if name not in record or not _finite(record[name]):
            raise CaptureError(f"line {line_number}: missing or non-finite submission field {name}")
    for name in INPUT_VECTORS:
        vector = record.get(name)
        if not isinstance(vector, list) or len(vector) != 3 or not all(map(_finite, vector)):
            raise CaptureError(f"line {line_number}: invalid three-component vector {name}")
    if any(name in record for name in CONTROL_NUMBERS + CONTROL_VECTORS):
        for name in CONTROL_NUMBERS:
            if not _finite(record.get(name)):
                raise CaptureError(f"line {line_number}: missing or invalid control field {name}")
        for name in CONTROL_VECTORS:
            vector = record.get(name)
            if not isinstance(vector, list) or len(vector) != 3 or \
                    not all(map(_finite, vector)):
                raise CaptureError(f"line {line_number}: missing or invalid control vector {name}")
        angle = record.get("attitudeErrorDegrees")
        if angle is not None and not _finite(angle):
            raise CaptureError(f"line {line_number}: invalid attitude error")
        for name in ("autoWarpEnabled", "rcsAdjustmentEnabled"):
            if not isinstance(record.get(name), bool):
                raise CaptureError(f"line {line_number}: missing or invalid control flag {name}")
        if type(record.get("activePredictionVersion")) is not int:
            raise CaptureError(f"line {line_number}: invalid active prediction version")
        active_ut = record.get("activePredictionInputUT")
        if active_ut is not None and not _finite(active_ut):
            raise CaptureError(f"line {line_number}: invalid active prediction input UT")
    if not record.get("body") or not record.get("kind"):
        raise CaptureError(f"line {line_number}: missing body or submission kind")
    provenance = record.get("modelProvenance")
    if provenance is not None and (not isinstance(provenance, str) or not provenance):
        raise CaptureError(f"line {line_number}: invalid model provenance")
    if provenance in ("v1_controller_policy_forecast", "v1_live_braking_forecast"):
        for name in ("controllerInitialMass", "controllerMaximumThrust",
                     "controllerMinimumThrust", "controllerMaximumMassFlow",
                     "controllerMinimumMassFlow", "controllerPolicyTerrainRadius",
                     "controllerPolicyGravity", "controllerPolicyThrust",
                     "controllerMinimumCommandThrottle", "controllerMaximumCommandThrottle",
                     "controllerThrottleSmoothingSeconds", "controllerInitialAppliedThrottle",
                     "livePolicyTerrainRadius", "livePolicyGravity",
                     "livePolicyThrust"):
            if not _finite(record.get(name)):
                raise CaptureError(f"line {line_number}: incomplete V1 control model field {name}")
    if "phase" not in record or type(record.get("wallTimestamp")) is not int or \
            type(record.get("wallTimestampFrequency")) is not int:
        raise CaptureError(f"line {line_number}: missing phase or wall-clock fields")
    if "forcedBrakingStartUT" not in record or "noSkipToFreefall" not in record:
        raise CaptureError(f"line {line_number}: missing predictor settings")
    if record["kind"] == "forced_candidate" and not _finite(record["forcedBrakingStartUT"]):
        raise CaptureError(f"line {line_number}: forced candidate needs a finite brake UT")
    if record["kind"] == "target_aware_transaction" and (
            not _finite(record.get("minimumTerrainASL")) or
            not _finite(record.get("maximumTerrainASL")) or
            record["minimumTerrainASL"] > record["maximumTerrainASL"]):
        raise CaptureError(f"line {line_number}: target-aware terrain bounds missing")
    if record["forcedBrakingStartUT"] is not None and not _finite(record["forcedBrakingStartUT"]):
        raise CaptureError(f"line {line_number}: invalid forced brake UT")
    if not isinstance(record["noSkipToFreefall"], bool):
        raise CaptureError(f"line {line_number}: noSkipToFreefall must be boolean")


def _validate_result(record, line_number):
    if record["recordType"] == "worker_result":
        brake_reference = record.get("controllerBrakeReferenceUT")
        if brake_reference is not None and not _finite(brake_reference):
            raise CaptureError(f"line {line_number}: invalid controller brake reference UT")
        if brake_reference is not None:
            position = record.get("controllerBrakeReferencePosition")
            if not isinstance(position, list) or len(position) != 4 or \
                    not all(map(_finite, position)):
                raise CaptureError(f"line {line_number}: missing controller brake reference position")
        if not isinstance(record.get("complete"), bool) or not record.get("outcome"):
            raise CaptureError(f"line {line_number}: missing worker outcome or completeness")
        if type(record.get("wallTimestamp")) is not int or not _finite(record.get("simulationElapsedMs")):
            raise CaptureError(f"line {line_number}: missing worker timing")
        if record["complete"]:
            for name in ("start", "simulatorEnd", "simulatorEndVelocity", "trajectoryStart"):
                vector = record.get(name)
                if not isinstance(vector, list) or len(vector) != 4 or not all(map(_finite, vector)):
                    raise CaptureError(f"line {line_number}: complete result has invalid {name}")
            for name in ("inputUT", "endUT", "virtualBrakeUT", "endSurfaceSpeed", "virtualDeltaV"):
                if not _finite(record.get(name)):
                    raise CaptureError(f"line {line_number}: complete result has invalid {name}")
    elif record["recordType"] == "resolved_result":
        if not _finite(record.get("processUT")) or not record.get("disposition"):
            raise CaptureError(f"line {line_number}: invalid resolution time or disposition")
        if "phase" not in record or type(record.get("currentGeneration")) is not int or \
                type(record.get("wallTimestamp")) is not int:
            raise CaptureError(f"line {line_number}: missing resolution phase or lineage")
        count = record.get("terrainQueryCount")
        samples = record.get("terrainSamples")
        if type(count) is not int or count < 0:
            raise CaptureError(f"line {line_number}: invalid terrain query count")
        if count:
            if not isinstance(samples, list) or len(samples) != count:
                raise CaptureError(f"line {line_number}: missing terrain samples")
            if any(not isinstance(sample, list) or len(sample) != 4 or
                   not all(map(_finite, sample)) for sample in samples):
                raise CaptureError(f"line {line_number}: invalid terrain sample")
        if record.get("terrainContactConfirmed"):
            if not record.get("terrainProfileApplied") or record.get("terrainContactIndex", -1) < 0:
                raise CaptureError(f"line {line_number}: inconsistent terrain contact")


def _validate_target_aware(record, line_number):
    if type(record.get("generation")) is not int or type(record.get("sequence")) is not int:
        raise CaptureError(f"line {line_number}: missing target-aware lineage")
    if not _finite(record.get("processUT")) or not record.get("stage"):
        raise CaptureError(f"line {line_number}: missing target-aware stage or process UT")
    for name in ("terrainResolved", "clearPath", "terminalNecessaryBoundPasses"):
        if not isinstance(record.get(name), bool):
            raise CaptureError(f"line {line_number}: invalid target-aware {name}")
    for name in ("terrainQueryCount",):
        if type(record.get(name)) is not int or record[name] < 0:
            raise CaptureError(f"line {line_number}: invalid target-aware {name}")
    for name in ("terrainQueryElapsedMs", "workerElapsedMs"):
        if not _finite(record.get(name)) or record[name] < 0:
            raise CaptureError(f"line {line_number}: invalid target-aware {name}")
    if "selectedPolicyTerrainASL" in record and not _finite(record["selectedPolicyTerrainASL"]):
        raise CaptureError(f"line {line_number}: invalid policy terrain")
    if "policyEscalations" in record and (type(record["policyEscalations"]) is not int or
                                          record["policyEscalations"] < 0):
        raise CaptureError(f"line {line_number}: invalid policy escalation count")
    if record["stage"] == "Complete":
        for name in ("ballisticContactUT", "virtualBrakeUT",
                     "minimumSampledClearance", "handoffClearance", "localTerrainASL",
                     "endVerticalSpeed", "endSurfaceSpeed", "transitionVerticalSpeed",
                     "transitionSurfaceSpeed", "optimisticStoppingDistance"):
            if not _finite(record.get(name)):
                raise CaptureError(f"line {line_number}: complete target-aware result lacks {name}")
        direct = record.get("directForecast", False)
        bracketed = record.get("brakeTimeBracketed", True)
        if not isinstance(direct, bool) or not isinstance(bracketed, bool):
            raise CaptureError(f"line {line_number}: invalid forecast mode")
        if not direct:
            for name in ("signedDownrangeError", "crossrangeError"):
                if not _finite(record.get(name)):
                    raise CaptureError(f"line {line_number}: target forecast lacks {name}")
        if bracketed:
            for name in ("timingInterval", "timingDistanceEstimate"):
                if not _finite(record.get(name)):
                    raise CaptureError(f"line {line_number}: refined forecast lacks {name}")
        if not (record["terrainResolved"] and record["clearPath"] and
                record["terminalNecessaryBoundPasses"]):
            raise CaptureError(f"line {line_number}: complete target-aware result is not validated")


def read_capture(path, allow_incomplete=False):
    """Return a replay-input document; never silently repair malformed capture."""
    cases = {}
    capture_errors = []
    lifecycle = []
    events = []
    record_count = 0
    truncated_tail = []
    with Path(path).open(encoding="utf-8-sig") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                record = json.loads(line, parse_constant=_reject_constant)
            except (ValueError, TypeError) as exc:
                # KSP can exit while its final append is in progress. Preserve
                # every complete record and report that final fragment as a
                # gap; malformed records in the middle still fail loudly.
                if allow_incomplete and not line.endswith("\n") and not stream.read(1):
                    truncated_tail.append(f"line {line_number}: truncated final record")
                    break
                raise CaptureError(f"line {line_number}: invalid JSON: {exc}") from exc
            if not isinstance(record, dict) or record.get("schemaVersion") != SCHEMA_VERSION:
                raise CaptureError(f"line {line_number}: unsupported capture schema")
            session = record.get("captureSession")
            kind = record.get("recordType")
            if not isinstance(session, str) or not session or kind not in RECORD_TYPES:
                raise CaptureError(f"line {line_number}: missing session or invalid record type")
            record_count += 1
            events.append({"lineNumber": line_number, **record})
            if kind == "capture_error":
                capture_errors.append({"line": line_number, **record})
                continue
            if kind == "lifecycle":
                lifecycle.append(record)
                continue
            submission_id = record.get("submissionId")
            if type(submission_id) is not int or submission_id <= 0:
                raise CaptureError(f"line {line_number}: invalid submission ID")
            key = (session, submission_id)
            case = cases.setdefault(key, {"captureSession": session, "submissionId": submission_id,
                                          "submissionLine": None,
                                          "submission": None, "worker": None, "resolved": None,
                                          "published": None, "validation": None,
                                          "discarded": [], "decisions": [], "workerStages": []})
            if kind == "discarded":
                case["discarded"].append(record)
                continue
            if kind == "selection_decision":
                case["decisions"].append(record)
                continue
            if kind == "target_aware_worker_stage":
                if record.get("stage") not in ("Ballistic", "Coarse", "Refinement",
                                               "PolicyRevalidation", "PolicyEscalation",
                                               "DirectForecast") or \
                        not _finite(record.get("processUT")) or \
                        not _finite(record.get("elapsedMs")) or record["elapsedMs"] < 0 or \
                        type(record.get("outputCount")) is not int or record["outputCount"] < 0:
                    raise CaptureError(f"line {line_number}: invalid target-aware worker stage")
                case["workerStages"].append(record)
                continue
            slot = {"submission": "submission", "worker_result": "worker",
                    "worker_exception": "worker", "submission_abandoned": "worker",
                    "resolved_result": "resolved", "target_aware_validation": "validation",
                    "published": "published"}[kind]
            if case[slot] is not None:
                raise CaptureError(f"line {line_number}: duplicate {slot} for {session}/{submission_id}")
            if kind == "submission":
                _validate_submission(record, line_number)
                case["submissionLine"] = line_number
            elif kind in ("worker_result", "resolved_result"):
                _validate_result(record, line_number)
            elif kind == "target_aware_validation":
                _validate_target_aware(record, line_number)
            elif kind == "published" and ("phase" not in record or
                                           type(record.get("currentGeneration")) is not int or
                                           type(record.get("resultVersion")) is not int or
                                           not _finite(record.get("processUT"))):
                raise CaptureError(f"line {line_number}: invalid publication lineage")
            case[slot] = record

    # File order preserves append-only sessions even when KSP UT resets.
    ordered = sorted(cases.values(), key=lambda case: (
        case["submissionLine"] if case["submissionLine"] is not None else float("inf"),
        case["captureSession"], case["submissionId"]))
    if not ordered:
        raise CaptureError("capture contains no predictor submissions")
    gaps = []
    for case in ordered:
        key = f'{case["captureSession"]}/{case["submissionId"]}'
        if case["submission"] is None:
            gaps.append(f"{key}: result without submission")
        if case["worker"] is None:
            gaps.append(f"{key}: submission without worker completion")
        if case["submission"] is not None and \
                case["submission"].get("kind") == "target_aware_transaction" and \
                case["validation"] is None:
            gaps.append(f"{key}: target-aware transaction without validation")
        if case["published"] is not None and case["resolved"] is None:
            gaps.append(f"{key}: publication without flight-thread resolution")
        if case["published"] is not None and case["worker"] is not None and \
                case["worker"]["recordType"] != "worker_result":
            gaps.append(f"{key}: publication without simulator result")
        if case["submission"] is not None and case["worker"] is not None and \
                case["worker"]["recordType"] == "worker_result" and \
                case["worker"].get("inputUT") != case["submission"]["inputUT"]:
            gaps.append(f"{key}: worker input UT differs from submitted UT")
    if capture_errors:
        gaps.extend(f'line {item["line"]}: capture error at {item.get("stage")}'
                    for item in capture_errors)
    gaps.extend(truncated_tail)
    if gaps and not allow_incomplete:
        raise CaptureError("incomplete capture:\n  " + "\n  ".join(gaps[:30]))

    return {
        "format": "mechjeb-v1-predictor-replay-input",
        "formatVersion": 1,
        "source": str(path),
        "recordCount": record_count,
        "caseCount": len(ordered),
        "gaps": gaps,
        "captureErrors": capture_errors,
        "lifecycle": lifecycle,
        "events": events,
        "cases": ordered,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path, help="LandingGuidanceV1.capture.jsonl")
    parser.add_argument("--output", type=Path, help="write versioned replay input JSON")
    parser.add_argument("--allow-incomplete", action="store_true",
                        help="inspect a truncated capture; exported gaps remain explicit")
    args = parser.parse_args(argv)
    try:
        document = read_capture(args.capture, args.allow_incomplete)
    except (CaptureError, OSError) as exc:
        parser.exit(2, f"capture reader: {exc}\n")
    if args.output:
        args.output.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f'{document["recordCount"]} records, {document["caseCount"]} submissions, '
          f'{len(document["gaps"])} gaps, '
          f'{sum(case["published"] is not None for case in document["cases"])} publications')
    return 0


if __name__ == "__main__":
    sys.exit(main())
