"""Regression cases for the Mun whole-sequence observation gate."""

import unittest

from landing_predictor_sequence_harness import audit_sequence, terminal_observation


def captured_case():
    submission = {"recordType": "submission", "captureSession": "flight",
                  "submissionId": 1, "kind": "target_aware_transaction",
                  "generation": 1, "captureEpochUT": 100, "inputUT": 100,
                  "bodyRadius": 200000, "phase": "DeorbitBurn"}
    worker = {"recordType": "worker_result", "complete": True,
              "virtualBrakeUT": 150, "trajectoryStart": [0, 0, 213000, 150],
              "simulatorEnd": [0, 20, 200500, 200]}
    return {"captureSession": "flight", "submissionId": 1,
            "submission": submission, "worker": worker,
            "resolved": {"recordType": "resolved_result", "currentGeneration": 1},
            "validation": {"stage": "Complete", "generation": 1,
                           "terrainResolved": True, "clearPath": True,
                           "terminalNecessaryBoundPasses": True,
                           "signedDownrangeError": 0, "timingDistanceEstimate": 0,
                           "timingInterval": 0.2, "terrainQueryCount": 2,
                           "minimumSampledClearance": 1, "handoffClearance": 20,
                           "sequence": 1},
            "decisions": [{"decision": "target_aware_refresh_started",
                           "comparedSubmissionId": 0},
                          {"decision": "target_aware_atomic_accept",
                           "comparedSubmissionId": 0}]}


def controller_state(ut, phase, throttle=0, altitude=5000):
    return {"ut": ut, "phase": phase, "throttle": throttle, "alt": altitude,
            "attitudeError": 0, "lineNumber": 1}


class SequenceHarnessTests(unittest.TestCase):
    def test_terminal_observation_reports_ideal_horizontal_stop_lower_bound(self):
        report = terminal_observation([{
            "recordType": "guidance_state", "phase": "KillHorizontalVelocity",
            "ut": 1, "surfaceVelocity": [100, 0, -10], "up": [0, 0, 1],
            "forward": [-1, 0, 0], "altitudeTrue": 200,
            "localGravity": 1.6, "commandedThrottle": 0.2,
        }])
        self.assertAlmostEqual(312.5, report["last"]["idealHorizontalStopTime"])
        self.assertAlmostEqual(15625, report["last"]["idealHorizontalStopDistance"])
        self.assertIsNone(report["attitudeSettlingSeconds"])

    def test_terminal_observation_measures_actual_attitude_settling(self):
        common = {"recordType": "guidance_state", "phase": "KillHorizontalVelocity",
                  "surfaceVelocity": [100, 0, -10], "up": [0, 0, 1],
                  "forward": [-1, 0, 0], "altitudeTrue": 200,
                  "localGravity": 1.6, "commandedThrottle": 0.2}
        report = terminal_observation([
            {**common, "ut": 1, "attitudeErrorDegrees": 62},
            {**common, "ut": 6, "attitudeErrorDegrees": 4},
        ])
        self.assertEqual(5, report["attitudeSettlingSeconds"])

    def document(self):
        case = captured_case()
        return {"cases": [case], "events": [
            {"recordType": "lifecycle", "captureSession": "flight", "lineNumber": 1,
             "action": "target_reset"},
            {**case["submission"], "lineNumber": 2},
            {"recordType": "published", "captureSession": "flight", "lineNumber": 3,
             "submissionId": 1, "resultVersion": 24, "currentGeneration": 1,
             "processUT": 101, "phase": "DeorbitBurn"},
        ]}

    def trace(self):
        return [{"ut": 101, "phase": "DeorbitBurn", "lineNumber": 10,
                 "predictionVersion": 24, "predictionInputUT": 100,
                 "commandedThrottle": 0, "warpRate": 1},
                {"ut": 201, "phase": "DecelerationBurn", "lineNumber": 11,
                 "predictionVersion": 24, "predictionInputUT": 100,
                 "commandedThrottle": 0.5, "warpRate": 1},
                # A different flight reused version 24; its age must be excluded.
                {"ut": 999, "phase": "DecelerationBurn", "lineNumber": 2,
                 "predictionVersion": 24, "predictionInputUT": 99,
                 "commandedThrottle": 1, "warpRate": 1}]

    def test_continuing_burn_and_early_throttle_fail_with_correct_lineage(self):
        states = [controller_state(102, "DeorbitBurn", 1),
                  controller_state(146, "DecelerationBurn", 1),
                  controller_state(150, "DecelerationBurn", 1)]
        report = audit_sequence(self.document(), self.trace(), states, [], "flight")
        self.assertFalse(report["passed"])
        self.assertEqual(101, report["publications"][0]["maximumObservedInputAgeSeconds"])
        self.assertEqual(8000, report["publications"][0]["brakeAltitudeDifference"])
        self.assertTrue(any("continuing deorbit burn" in issue for issue in report["issues"]))
        self.assertTrue(any("before forecast brake UT" in issue for issue in report["issues"]))

    def test_terminal_planning_fails_without_publishing(self):
        document = self.document()
        document["events"].append({"recordType": "submission", "captureSession": "flight",
                                   "lineNumber": 4, "submissionId": 2,
                                   "kind": "target_aware_transaction",
                                   "phase": "FinalDescent"})
        report = audit_sequence(document, self.trace(), [], [], "flight")
        self.assertTrue(any("terrain candidate search in FinalDescent" in issue
                            for issue in report["issues"]))

    def test_controller_model_refresh_tracks_all_v1_phases(self):
        document = self.document()
        case = document["cases"][0]
        case["submission"].update(phase="CourseCorrection",
                                  modelProvenance="v1_controller_policy_forecast")
        document["events"][1].update(phase="CourseCorrection",
                                      modelProvenance="v1_controller_policy_forecast")
        document["events"][2]["phase"] = "CourseCorrection"
        case["worker"].update(controllerBrakeReferenceUT=150,
                              controllerBrakeReferencePosition=[0, 0, 205000, 150],
                              trajectoryStart=[0, 0, 210000, 100])
        case["resolved"]["resolvedEndASL"] = 500
        case["validation"].update(brakeTimeBracketed=True, directForecast=False)
        trace = [{"recordType": "guidance_state", "ut": ut, "phase": phase,
                  "lineNumber": index + 10, "predictionVersion": 24,
                  "predictionInputUT": 100, "commandedThrottle": throttle,
                  "warpRate": 1, "controllerDetail": ""}
                 for index, (ut, phase, throttle) in enumerate((
                     (101, "CourseCorrection", 0), (145, "DecelerationBurn", 1),
                     (150, "DecelerationBurn", 1),
                     (200, "KillHorizontalVelocity", 0.5),
                     (205, "FinalDescent", 0.3)))]
        actual = [controller_state(101, "CourseCorrection"),
                  controller_state(144, "DecelerationBurn"),
                  controller_state(145, "DecelerationBurn", 1),
                  controller_state(150, "DecelerationBurn", 1),
                  controller_state(200, "KillHorizontalVelocity", .5),
                  controller_state(205, "FinalDescent", .3)]
        report = audit_sequence(document, trace, actual, [], "flight")
        self.assertTrue(report["passed"], report["issues"] + report["unknowns"])

    def test_impact_without_brake_is_not_a_landing_phase_prediction(self):
        document = self.document()
        case = document["cases"][0]
        case["submission"]["modelProvenance"] = "v1_no_further_correction_baseline"
        document["events"][1]["modelProvenance"] = "v1_no_further_correction_baseline"
        case["worker"].update(outcome="IMPACT", forecastKind="ImpactForecast",
                              virtualBrakeUT=None, trajectoryStart=[0, 0, 210000, 100])
        case["validation"].update(directForecast=True, brakeTimeBracketed=False,
                                  forecastKind="ImpactForecast", firstContactUT=200,
                                  minimumSampledClearance=-1, handoffClearance=0,
                                  terminalNecessaryBoundPasses=False)
        guidance = [{"recordType": "guidance_state", "ut": 102,
                     "phase": "CourseCorrection", "lineNumber": 10,
                     "predictionVersion": 24, "predictionInputUT": 100,
                     "commandedThrottle": 0, "warpRate": 1}]
        report = audit_sequence(document, guidance, [], [], "flight")
        self.assertFalse(any("brake time absent" in gap for gap in report["unknowns"]))
        self.assertFalse(any("impact forecast authorized" in issue for issue in report["issues"]))
        guidance.append({**guidance[0], "ut": 103, "phase": "DecelerationBurn",
                         "lineNumber": 11})
        report = audit_sequence(document, guidance, [], [], "flight")
        self.assertTrue(any("impact forecast authorized DecelerationBurn" in issue
                            for issue in report["issues"]))

    def test_approved_safe_timing_model_still_requires_brake_path_and_reference(self):
        document = self.document()
        case = document["cases"][0]
        provenance = "v1_safe_brake_timing_no_further_correction"
        case["submission"]["modelProvenance"] = provenance
        document["events"][1]["modelProvenance"] = provenance
        case["worker"].update(controllerBrakeReferenceUT=150,
                              controllerBrakeReferencePosition=[0, 0, 205000, 150],
                              trajectoryStart=[0, 0, 210000, 100])
        report = audit_sequence(document, self.trace(), [], [], "flight")
        self.assertFalse(any("provenance" in issue for issue in report["issues"]))
        case["worker"]["controllerBrakeReferenceUT"] = None
        report = audit_sequence(document, self.trace(), [], [], "flight")
        self.assertTrue(any("brake reference or preceding path missing" in issue
                            for issue in report["issues"]))


if __name__ == "__main__":
    unittest.main()
