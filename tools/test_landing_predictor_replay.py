"""Numerical parity and terrain-state checks against the captured Mun run."""

import json
import math
import unittest
from pathlib import Path

from landing_predictor_replay import (refine_spherical_brake_time, replay_forced_candidate,
                                      sampled_terrain_clearance)


FIXTURE = Path(__file__).parent / "fixtures" / "mun_forced_candidates_2026-09-28.json"


class MunReplayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads(FIXTURE.read_text(encoding="utf-8"))

    def test_all_nine_forced_candidates_match_captured_simulator(self):
        self.assertEqual(9, len(self.fixture["cases"]))
        for case in self.fixture["cases"]:
            with self.subTest(submission_id=case["submissionId"]):
                replay = replay_forced_candidate(case["submission"])
                worker = case["worker"]
                actual, expected = replay["end"], worker["simulatorEnd"]
                horizontal = math.hypot(math.radians(actual[0] - expected[0]),
                                        math.radians(actual[1] - expected[1]) *
                                        math.cos(math.radians(expected[0]))) * expected[2]
                self.assertLess(horizontal, 0.001)
                self.assertLess(abs(actual[2] - expected[2]), 0.001)
                self.assertLess(abs(actual[3] - expected[3]), 1e-6)
                self.assertLess(abs(replay["endSurfaceSpeed"] - worker["endSurfaceSpeed"]), 1e-4)
                self.assertEqual(worker["steps"], replay["steps"])

    def test_candidate_177_is_above_terrain_without_sampled_early_contact(self):
        case = next(item for item in self.fixture["cases"] if item["submissionId"] == 177)
        terrain = sampled_terrain_clearance(case["resolved"], case["worker"])
        self.assertEqual("sampled_clear_to_handoff", terrain["status"])
        self.assertEqual(11, terrain["sampleCount"])
        self.assertAlmostEqual(41.78476681932807, terrain["handoffClearance"], places=6)
        self.assertFalse(case["resolved"]["terrainContactConfirmed"])

    def test_unqueried_candidates_remain_terrain_unresolved(self):
        case = next(item for item in self.fixture["cases"] if item["submissionId"] == 178)
        self.assertEqual("terrain_unresolved",
                         sampled_terrain_clearance(case["resolved"], case["worker"])["status"])

    def test_signed_refinement_is_numerically_near_target_but_terrain_unknown(self):
        # 200 m is the existing V1 CourseCorrection completion distance on Mun.
        result = refine_spherical_brake_time(self.fixture["cases"], 200)
        self.assertEqual("spherical_refined_terrain_unresolved", result["status"])
        self.assertLess(abs(result["signedDownrangeError"]), 200)
        self.assertLess(max(map(abs, result["bracketDownrangeErrors"])), 200)
        self.assertLess(result["timingDistanceEstimate"], 200)
        self.assertGreater(abs(result["crossrangeError"]), 0)


if __name__ == "__main__":
    unittest.main()
