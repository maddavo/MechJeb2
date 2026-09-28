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
                          workerElapsedMs=20.0)
        publication = dict(BASE, recordType="published", processUT=101.0,
                           currentGeneration=4, phase="CourseCorrection", resultVersion=7)
        stages = [dict(BASE, recordType="target_aware_worker_stage", stage=stage,
                       processUT=100.1 + index * 0.1, elapsedMs=2.0,
                       outputCount=count, exceptionType=None)
                  for index, (stage, count) in enumerate((("Ballistic", 300),
                                                          ("Coarse", 9),
                                                          ("Refinement", 8)))]
        result = self.read([active, *stages, validation, worker(),
                            resolved(terrainQueryCount=0, terrainSamples=None,
                                     terrainContactConfirmed=False,
                                     terrainProfileApplied=False), publication])
        self.assertEqual(result["gaps"], [])
        self.assertEqual(result["cases"][0]["validation"]["handoffClearance"], 42.0)
        self.assertEqual([event["stage"] for event in result["cases"][0]["workerStages"]],
                         ["Ballistic", "Coarse", "Refinement"])
        with self.assertRaisesRegex(reader.CaptureError, "without validation"):
            self.read([active, worker(), resolved(terrainQueryCount=0,
                                                terrainSamples=None,
                                                terrainContactConfirmed=False,
                                                terrainProfileApplied=False), publication])


if __name__ == "__main__":
    unittest.main()
