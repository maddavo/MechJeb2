"""Event-order gates for target-aware capture publication and invalidation."""

import unittest

from landing_predictor_lifecycle_harness import audit_lifecycle


SESSION = "mun-replay"


def event(line, record_type, submission_id=None, **fields):
    item = {"lineNumber": line, "captureSession": SESSION,
            "recordType": record_type, **fields}
    if submission_id is not None:
        item["submissionId"] = submission_id
    return item


def case(submission_id, predecessor, sequence, complete=True):
    return {"captureSession": SESSION, "submissionId": submission_id,
            "submission": {"kind": "target_aware_transaction", "generation": 1,
                           "bodyRadius": 200000,
                           "captureEpochUT": 100 + sequence * 5},
            "validation": {"stage": "Complete" if complete else "Failed",
                           "generation": 1,
                           "terrainResolved": complete, "clearPath": complete,
                           "terminalNecessaryBoundPasses": complete,
                           "signedDownrangeError": 36.0,
                           "timingDistanceEstimate": 90.0,
                           "timingInterval": 0.2,
                           "terrainQueryCount": 818,
                           "minimumSampledClearance": 2.0,
                           "handoffClearance": 40.0,
                           "sequence": sequence},
            "worker": {"recordType": "worker_result", "complete": complete},
            "resolved": {"recordType": "resolved_result",
                         "currentGeneration": 1} if complete else None,
            "decisions": ([{"decision": "target_aware_refresh_started",
                            "comparedSubmissionId": predecessor}] +
                          ([{"decision": "target_aware_atomic_accept",
                             "comparedSubmissionId": predecessor}] if complete else []))}


class LifecycleHarnessTests(unittest.TestCase):
    def document(self):
        cases = [case(1, 0, 1), case(3, 1, 2), case(4, 3, 3, False),
                 case(5, 0, 4)]
        cases.append({"captureSession": SESSION, "submissionId": 2,
                      "submission": {"kind": "ordinary"}})
        events = [event(1, "lifecycle", action="target_reset"),
                  event(2, "submission", 1, kind="target_aware_transaction",
                        phase="DeorbitBurn"),
                  event(3, "published", 1, resultVersion=10, currentGeneration=1,
                        processUT=106,
                        phase="DeorbitBurn"),
                  event(4, "submission", 2, kind="ordinary", phase="CourseCorrection"),
                  event(5, "submission", 3, kind="target_aware_transaction",
                        phase="CourseCorrection"),
                  event(6, "published", 3, resultVersion=11, currentGeneration=1,
                        processUT=111,
                        phase="CourseCorrection"),
                  event(7, "submission", 4, kind="target_aware_transaction",
                        phase="CoastToDeceleration"),
                  event(8, "submission_abandoned", 4),
                  event(9, "target_aware_worker_stage", 4, stage="Refinement"),
                  event(10, "lifecycle", action="target_aware_target_terrain_changed"),
                  event(11, "submission", 5, kind="target_aware_transaction",
                        phase="CoastToDeceleration"),
                  event(12, "published", 5, resultVersion=12, currentGeneration=1,
                        processUT=121,
                        phase="CoastToDeceleration"),
                  event(13, "lifecycle", action="landing_stopped"),
                  event(14, "target_aware_worker_stage", 4, stage="Refinement")]
        return {"cases": cases, "events": events}

    def test_refresh_survives_phase_change_and_late_failure(self):
        report = audit_lifecycle(self.document(), require_active=True)
        self.assertTrue(report["passed"], report["issues"])
        self.assertEqual(3, report["activePublications"])
        self.assertEqual([1, 3, None, 5],
                         [item["activeSubmissionId"] for item in report["transitions"]])
        self.assertEqual([None, 1, 1, 3, None],
                         [item["activeSubmissionId"] for item in report["phaseObservations"]])

    def test_ordinary_overwrite_and_incomplete_publication_fail(self):
        document = self.document()
        document["events"].insert(4, event(4.5, "published", 2,
                                           resultVersion=11, phase="CourseCorrection"))
        report = audit_lifecycle(document)
        self.assertFalse(report["passed"])
        self.assertTrue(any("ordinary result published" in issue for issue in report["issues"]))

        document = self.document()
        document["events"].insert(9, event(9.5, "published", 4,
                                           resultVersion=12, phase="DecelerationBurn"))
        report = audit_lifecycle(document)
        self.assertFalse(report["passed"])
        self.assertTrue(any("lacks complete validation" in issue for issue in report["issues"]))

        document = self.document()
        document["events"].append(event(15, "published", 4, resultVersion=13,
                                        phase="FinalDescent"))
        report = audit_lifecycle(document)
        self.assertTrue(any("abandoned transaction was published" in issue
                            for issue in report["issues"]))
        self.assertTrue(any("after landing stopped" in issue for issue in report["issues"]))

        document = self.document()
        document["events"][5]["currentGeneration"] = 2
        report = audit_lifecycle(document)
        self.assertTrue(any("generation changed" in issue for issue in report["issues"]))

    def test_terminal_phase_never_starts_or_publishes_search(self):
        for phase in ("DecelerationBurn", "KillHorizontalVelocity", "FinalDescent"):
            with self.subTest(phase=phase):
                document = self.document()
                document["events"][10]["phase"] = phase
                document["events"][11]["phase"] = phase
                report = audit_lifecycle(document)
                self.assertTrue(any(f"planning submitted during {phase}" in issue
                                    for issue in report["issues"]))
                self.assertTrue(any(f"planning published during {phase}" in issue
                                    for issue in report["issues"]))

    def test_live_braking_forecast_refreshes_without_terminal_search(self):
        direct = case(1, 0, 1)
        direct["submission"]["modelProvenance"] = "v1_live_braking_forecast"
        direct["validation"].update(directForecast=True, brakeTimeBracketed=False,
                                    signedDownrangeError=None,
                                    timingInterval=None,
                                    timingDistanceEstimate=None)
        document = {"cases": [direct], "events": [
            event(1, "lifecycle", action="target_reset"),
            event(2, "submission", 1, kind="target_aware_transaction",
                  modelProvenance="v1_live_braking_forecast",
                  phase="DecelerationBurn"),
            event(3, "published", 1, resultVersion=10,
                  currentGeneration=1, processUT=106,
                  phase="DecelerationBurn"),
        ]}
        report = audit_lifecycle(document, require_active=True)
        self.assertTrue(report["passed"], report["issues"])

    def test_baseline_impact_replaces_only_with_complete_contact_lineage(self):
        first = case(1, 0, 1)
        first["submission"]["modelProvenance"] = "v1_no_further_correction_baseline"
        first["validation"].update(directForecast=True, brakeTimeBracketed=False,
            forecastKind="ImpactForecast", signedDownrangeError=None,
            minimumSampledClearance=-1, handoffClearance=0,
            terminalNecessaryBoundPasses=False, firstContactUT=120)
        first["worker"].update(outcome="IMPACT", forecastKind="ImpactForecast")
        document = {"cases": [first], "events": [
            event(1, "lifecycle", action="target_reset"),
            event(2, "submission", 1, kind="target_aware_transaction",
                  modelProvenance="v1_no_further_correction_baseline",
                  phase="CourseCorrection"),
            event(3, "published", 1, resultVersion=10, currentGeneration=1,
                  processUT=106, phase="CourseCorrection"),
        ]}
        report = audit_lifecycle(document, require_active=True)
        self.assertTrue(report["passed"], report["issues"])
        first["worker"]["outcome"] = "LANDED"
        report = audit_lifecycle(document, require_active=True)
        self.assertFalse(report["passed"])


if __name__ == "__main__":
    unittest.main()
