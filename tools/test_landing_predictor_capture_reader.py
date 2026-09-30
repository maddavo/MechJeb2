"""Lifecycle and schema checks for the passive predictor capture reader."""

import json
import tempfile
import unittest
from pathlib import Path

import landing_predictor_capture_reader as reader


BASE = {"schemaVersion": 1, "captureSession": "session-a", "submissionId": 1}


def submission(**changes):
    record = dict(
        BASE, recordType="submission", kind="ordinary", generation=4,
        inputUT=100.0, captureEpochUT=100.0, phase="CourseCorrection",
        body="Mun", bodyRadius=200000.0, bodyMu=65138398000.0,
        bodyGeeASL=0.166, rotationPeriod=138984.0,
        angularVelocity=[0.0, 0.0, 0.1], bodyAxis0=[1, 0, 0],
        bodyAxis90=[0, 1, 0], bodyAxisNorth=[0, 0, 1],
        positionBCI=[210000, 0, 0], velocityBCI=[0, 550, 0],
        mass=10.0, thrustAvailable=100.0, thrustMinimum=0.0,
        throttleFixedLimit=1.0, limitedMaxThrustAcceleration=10.0,
        maxEngineResponseTime=1.0, targetLatitude=0.0,
        targetLongitude=10.0, targetTerrainASL=150.0,
        targetTerrainQueryElapsedMs=0.1, policy="SafeDescentSpeedPolicy",
        decelEndASL=350.0, probableLandingSiteASL=0.0,
        maxThrustAcceleration=10.0, parachuteMultiplier=3.0,
        dt=0.2, minDt=0.02, maxOrbits=1.0, noSkipToFreefall=False,
        forcedBrakingStartUT=None, wallTimestamp=100,
        wallTimestampFrequency=1000,
    )
    record.update(changes)
    return record


def worker(**changes):
    record = dict(
        BASE, recordType="worker_result", outcome="LANDED", complete=True,
        inputUT=100.0, endUT=300.0, virtualBrakeUT=100.0,
        start=[0, 0, 210000, 100], simulatorEnd=[0, 20, 200000, 300],
        simulatorEndVelocity=[0, 0, 0, 300], trajectoryStart=[0, 0, 210000, 100],
        endSurfaceSpeed=5.0, virtualDeltaV=300.0, wallTimestamp=200,
        simulationElapsedMs=40.0,
    )
    record.update(changes)
    return record


def resolved(**changes):
    record = dict(
        BASE, recordType="resolved_result", processUT=101.0,
        currentGeneration=4, wallTimestamp=300, phase="CourseCorrection",
        disposition="ordinary_processed", terrainQueryCount=1,
        terrainSamples=[[0, 20, 0, 150]], terrainContactConfirmed=True,
        terrainProfileApplied=True, terrainContactIndex=5,
    )
    record.update(changes)
    return record


class CaptureReaderTests(unittest.TestCase):
    def test_map_observation_does_not_create_a_prediction_or_submission(self):
        map_event = dict(BASE, recordType="map_draw", submissionId=0,
                         processUT=101, resultVersion=7, mapEnabled=True,
                         cameraTrajectory=False, predictorEnabled=True,
                         vesselLanded=False, outcome=None, markerRequested=False,
                         reason="no_committed_result")
        document = self.read([submission(), worker(), map_event])
        self.assertEqual(document["caseCount"], 1)
        self.assertEqual(document["events"][-1]["reason"], "no_committed_result")
        self.assertIsNone(document["cases"][0]["published"])
        with self.assertRaisesRegex(reader.CaptureError, "map draw markerRequested"):
            self.read([submission(), worker(), {**map_event, "markerRequested": 1}])

    def test_cache_samples_accounting_and_terminal_worker_are_replayable(self):
        validation = dict(BASE, recordType="target_aware_validation", processUT=101,
                          generation=4, sequence=1, stage="Failed", terrainResolved=False,
                          clearPath=False, terminalNecessaryBoundPasses=False,
                          terrainQueryCount=2, terrainSampleCount=4, terrainCacheHits=2,
                          terrainCacheSamples=[[0, 23, 492], [0, 24, 600]],
                          terrainQueryElapsedMs=0.1, workerElapsedMs=10)
        stage = dict(BASE, recordType="target_aware_worker_stage", processUT=101,
                     stage="Terminal", elapsedMs=5, outputCount=20, exceptionType=None)
        document = self.read([submission(), worker(), stage, validation])
        self.assertEqual(document["cases"][0]["validation"]["terrainCacheSamples"],
                         validation["terrainCacheSamples"])
        with self.assertRaisesRegex(reader.CaptureError, "cache accounting"):
            self.read([submission(), worker(), {**validation, "terrainCacheHits": 4}])
        with self.assertRaisesRegex(reader.CaptureError, "duplicate cached terrain"):
            self.read([submission(), worker(), {**validation, "terrainCacheSamples":
                       [[0, 23, 492], [0, 23, 493]]}])

    def test_terminal_contact_with_unknown_total_delta_v_preserves_the_gap(self):
        contact = worker(forecastKind="LandableForecast", virtualDeltaV=None,
                         endVerticalSpeed=-0.31, endHorizontalSpeed=0.0003,
                         endTerrainClearance=0)
        doc = self.read([submission(), contact])
        self.assertIsNone(doc["cases"][0]["worker"]["virtualDeltaV"])
        with self.assertRaisesRegex(reader.CaptureError, "virtualDeltaV"):
            self.read([submission(), {**contact, "forecastKind": None}])

    def test_spatial_resolution_metadata_is_preserved_and_validated(self):
        validation = dict(BASE, recordType="target_aware_validation", processUT=101,
                          generation=4, sequence=1, stage="Failed", terrainResolved=False,
                          clearPath=False, terminalNecessaryBoundPasses=False,
                          terrainQueryCount=0, terrainQueryElapsedMs=0, workerElapsedMs=1,
                          terrainCacheModel="demand_quantized_pqs",
                          coarseTerrainResolutionMetres=25, fineTerrainResolutionMetres=1,
                          terrainMaximumQueryOffsetMetres=12)
        doc = self.read([submission(), worker(), validation])
        self.assertEqual(doc["cases"][0]["validation"]["fineTerrainResolutionMetres"], 1)
        with self.assertRaisesRegex(reader.CaptureError, "invalid terrain query offset"):
            self.read([submission(), worker(), {**validation, "terrainMaximumQueryOffsetMetres": 30}])
        with self.assertRaisesRegex(reader.CaptureError, "invalid fineTerrainResolutionMetres"):
            self.read([submission(), worker(), {**validation, "fineTerrainResolutionMetres": 0}])

    def read(self, records, allow_incomplete=False):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.jsonl"
            path.write_text("".join(json.dumps(record) + "\n" for record in records),
                            encoding="utf-8")
            return reader.read_capture(path, allow_incomplete)

    def test_pairs_out_of_order_completion_and_publication_with_lifecycle(self):
        published = dict(BASE, recordType="published", processUT=101.0,
                         currentGeneration=4, phase="CourseCorrection", resultVersion=7)
        discarded = dict(BASE, recordType="discarded", reason="published_result_replaced")
        lifecycle = {"schemaVersion": 1, "captureSession": "session-a",
                     "recordType": "lifecycle", "action": "target_reset", "processUT": 102.0}
        result = self.read([worker(), published, resolved(), discarded, lifecycle, submission()])
        self.assertEqual(result["caseCount"], 1)
        self.assertEqual(result["gaps"], [])
        self.assertEqual(result["cases"][0]["published"]["resultVersion"], 7)
        self.assertEqual(result["cases"][0]["discarded"][0]["reason"],
                         "published_result_replaced")
        self.assertEqual(result["lifecycle"][0]["action"], "target_reset")

    def test_missing_worker_requires_explicit_incomplete_mode(self):
        with self.assertRaisesRegex(reader.CaptureError, "submission without worker"):
            self.read([submission()])
        self.assertIn("submission without worker", self.read([submission()], True)["gaps"][0])

    def test_truncated_final_append_is_an_explicit_gap(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.jsonl"
            path.write_text(json.dumps(submission()) + "\n" +
                            json.dumps(worker()) + "\n" + '{"recordType":"sub',
                            encoding="utf-8")
            with self.assertRaisesRegex(reader.CaptureError, "invalid JSON"):
                reader.read_capture(path)
            document = reader.read_capture(path, allow_incomplete=True)
            self.assertEqual(document["caseCount"], 1)
            self.assertIn("truncated final record", " ".join(document["gaps"]))

    def test_keeps_append_order_when_a_later_session_resets_ut(self):
        later_submission = submission(captureSession="session-b", inputUT=10.0,
                                      captureEpochUT=10.0)
        later_worker = worker(captureSession="session-b", inputUT=10.0)
        result = self.read([submission(), worker(), later_submission, later_worker])
        self.assertEqual([case["captureSession"] for case in result["cases"]],
                         ["session-a", "session-b"])

    def test_rejects_nonfinite_input_and_missing_terrain_samples(self):
        with self.assertRaisesRegex(reader.CaptureError, "targetTerrainASL"):
            self.read([submission(targetTerrainASL=None), worker()])
        with self.assertRaisesRegex(reader.CaptureError, "missing terrain samples"):
            self.read([submission(), worker(), resolved(terrainSamples=[])])

    def test_preserves_existing_incomplete_v1_publication_as_evidence(self):
        no_reentry = worker(outcome="NO_REENTRY", complete=False, endUT=None,
                            virtualBrakeUT=None, start=None, simulatorEnd=None,
                            simulatorEndVelocity=None, trajectoryStart=None,
                            endSurfaceSpeed=None, virtualDeltaV=None)
        no_contact = resolved(terrainQueryCount=0, terrainSamples=None,
                              terrainContactConfirmed=False, terrainProfileApplied=False,
                              terrainContactIndex=-1)
        published = dict(BASE, recordType="published", processUT=101.0,
                         currentGeneration=4, phase="CourseCorrection", resultVersion=8)
        result = self.read([submission(), no_reentry, no_contact, published])
        self.assertFalse(result["cases"][0]["worker"]["complete"])
        self.assertEqual(result["gaps"], [])

    def test_target_aware_validation_and_atomic_publication(self):
        active = submission(kind="target_aware_transaction", minimumTerrainASL=-1000,
                            maximumTerrainASL=10000)
        validation = dict(BASE, recordType="target_aware_validation", processUT=101.0,
                          generation=4, sequence=1, stage="Complete", failure=None,
                          ballisticContactUT=350.0, virtualBrakeUT=260.0,
                          signedDownrangeError=40.0, crossrangeError=165.0,
                          timingInterval=0.2, timingDistanceEstimate=80.0,
                          terrainResolved=True, clearPath=True,
                          minimumSampledClearance=41.0, handoffClearance=42.0,
                          localTerrainASL=450.0, endVerticalSpeed=-45.0,
                          endSurfaceSpeed=60.0, transitionVerticalSpeed=-47.0,
                          transitionSurfaceSpeed=70.0,
                          optimisticStoppingDistance=175.0,
                          terminalNecessaryBoundPasses=True,
                          terrainQueryCount=300, terrainQueryElapsedMs=2.5,
                          workerElapsedMs=20.0,
                          selectedPolicyTerrainASL=840.0, policyEscalations=1)
        publication = dict(BASE, recordType="published", processUT=101.0,
                           currentGeneration=4, phase="CourseCorrection", resultVersion=7)
        stages = [dict(BASE, recordType="target_aware_worker_stage", stage=stage,
                       processUT=100.1 + index * 0.1, elapsedMs=2.0,
                       outputCount=count, exceptionType=None)
                  for index, (stage, count) in enumerate((("Ballistic", 300),
                                                          ("Coarse", 9),
                                                          ("Refinement", 8),
                                                          ("PolicyEscalation", 1)))]
        result = self.read([active, *stages, validation, worker(),
                            resolved(terrainQueryCount=0, terrainSamples=None,
                                     terrainContactConfirmed=False,
                                     terrainProfileApplied=False), publication])
        self.assertEqual(result["gaps"], [])
        self.assertEqual(result["cases"][0]["validation"]["handoffClearance"], 42.0)
        self.assertEqual([event["stage"] for event in result["cases"][0]["workerStages"]],
                         ["Ballistic", "Coarse", "Refinement", "PolicyEscalation"])
        with self.assertRaisesRegex(reader.CaptureError, "without validation"):
            self.read([active, worker(), resolved(terrainQueryCount=0,
                                                terrainSamples=None,
                                                terrainContactConfirmed=False,
                                                terrainProfileApplied=False), publication])

    def test_optional_controller_lineage_requires_a_complete_brake_reference(self):
        active = submission(modelProvenance="v1_controller_equivalent")
        result = worker(controllerBrakeReferenceUT=150.0,
                        controllerBrakeReferencePosition=[0, 10, 205000, 150])
        self.assertEqual(self.read([active, result])["gaps"], [])

    def test_baseline_impact_without_a_brake_trigger_is_complete(self):
        active = submission(kind="target_aware_transaction", minimumTerrainASL=-1000,
                            maximumTerrainASL=10000)
        contact = worker(outcome="IMPACT", forecastKind="ImpactForecast",
                         virtualBrakeUT=None, virtualDeltaV=None,
                         endVerticalSpeed=-58,
                         endHorizontalSpeed=400, endTerrainClearance=0)
        validation = dict(BASE, recordType="target_aware_validation",
                          processUT=101, generation=4, sequence=1, stage="Complete",
                          directForecast=True, forecastKind="ImpactForecast",
                          firstContactUT=300, virtualBrakeUT=None,
                          terrainResolved=True, clearPath=True,
                          terminalNecessaryBoundPasses=False,
                          minimumSampledClearance=-1, handoffClearance=0,
                          localTerrainASL=150, endVerticalSpeed=-58,
                          endSurfaceSpeed=404, terrainQueryCount=12,
                          terrainQueryElapsedMs=0.1, workerElapsedMs=20)
        result = self.read([active, contact, validation])
        self.assertEqual(result["gaps"], [])
        self.assertEqual(result["cases"][0]["worker"]["forecastKind"], "ImpactForecast")
        self.assertIsNone(result["cases"][0]["worker"]["virtualDeltaV"])
        with self.assertRaisesRegex(reader.CaptureError, "impact lacks contact state"):
            self.read([active, worker(**{**contact, "endHorizontalSpeed": None}),
                       validation])
        with self.assertRaisesRegex(reader.CaptureError, "controller brake reference position"):
            self.read([active, worker(controllerBrakeReferenceUT=150.0)])
        with self.assertRaisesRegex(reader.CaptureError, "model provenance"):
            self.read([submission(modelProvenance=5), worker()])

    def test_optional_controller_snapshot_is_atomic(self):
        fields = {name: 1.0 for name in reader.CONTROL_NUMBERS}
        fields.update({name: [1.0, 0.0, 0.0] for name in reader.CONTROL_VECTORS})
        fields.update(attitudeErrorDegrees=None, autoWarpEnabled=True,
                      rcsAdjustmentEnabled=False, activePredictionVersion=5,
                      activePredictionInputUT=99.0)
        self.assertEqual(self.read([submission(**fields), worker()])["gaps"], [])
        del fields["forward"]
        with self.assertRaisesRegex(reader.CaptureError, "control vector forward"):
            self.read([submission(**fields), worker()])

    def test_live_controller_model_requires_complete_inputs(self):
        control = {name: 1.0 for name in (
            "controllerInitialMass", "controllerMaximumThrust",
            "controllerMinimumThrust", "controllerMaximumMassFlow",
            "controllerMinimumMassFlow", "controllerPolicyTerrainRadius",
            "controllerPolicyGravity", "controllerPolicyThrust",
            "controllerMinimumCommandThrottle", "controllerMaximumCommandThrottle",
            "controllerThrottleSmoothingSeconds", "controllerInitialAppliedThrottle",
            "livePolicyTerrainRadius", "livePolicyGravity",
            "livePolicyThrust")}
        active = submission(kind="target_aware_transaction",
                            modelProvenance="v1_live_braking_forecast",
                            minimumTerrainASL=-1000,
                            maximumTerrainASL=10000, **control)
        self.assertEqual(self.read([active, worker()], True)["cases"][0]["submission"][
            "controllerMaximumMassFlow"], 1.0)
        del active["controllerMaximumMassFlow"]
        with self.assertRaisesRegex(reader.CaptureError, "controllerMaximumMassFlow"):
            self.read([active, worker()])


if __name__ == "__main__":
    unittest.main()
