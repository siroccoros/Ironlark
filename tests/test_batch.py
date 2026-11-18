"""Evaluator escape requires passage through the actual gate below wall height."""
import json
import tempfile
import unittest
from pathlib import Path

from tools.evaluate_batch import escape_index, judge, minimum_clearance
from tools.generate_maze import detour_obstacle, generate


class BatchEvaluationTests(unittest.TestCase):
    def test_spawned_wall_closes_an_open_passage_with_a_detour(self):
        truth = generate(7)
        obstacle = detour_obstacle(truth)
        a, b = obstacle["passage"]
        self.assertIn([a, b], truth["passages"])
        self.assertIsNotNone(obstacle["shortest_detour_m"])
        pitch = truth["cell_pitch_m"]
        start = truth["start"]["cell"]
        self.assertAlmostEqual(obstacle["centre"][0], ((a[1] + b[1]) / 2 - start[1]) * pitch)
        self.assertAlmostEqual(obstacle["centre"][1], ((a[0] + b[0]) / 2 - start[0]) * pitch)

    def test_exit_gate_and_wall_height(self):
        truth = {"exit": {"side": "east", "opening": [[1, -1.25], [1, 1.25]]},
                 "bounds": {"min": [-1, -1], "max": [1, 1]},
                 "parameters": {"wall_height_m": 3}}
        samples = [{"position": [1.5, 2, 1.5]},
                   {"position": [1.5, 0, 3.5]},
                   {"position": [1.5, 0, 1.5]}]
        self.assertEqual(escape_index(samples, truth), 2)

    def test_new_obstacle_only_counts_after_its_spawn(self):
        obstacle = {"centre": [0, 0, 1.5], "size": [1, 1, 3]}
        samples = [{"time": 4, "position": [0, 0, 1.5]},
                   {"time": 6, "position": [0.75, 0, 1.5]}]
        self.assertEqual(minimum_clearance(samples, [], obstacle, spawned_at=5), 0.25)

    def test_failed_run_keeps_its_reason_failsafe_and_odometry_lag(self):
        truth = generate(7)
        with tempfile.TemporaryDirectory() as directory:
            run = Path(directory)
            (run / "manifest.json").write_text(json.dumps({"world": "maze", "maze_seed": 7, "gps_denied": True}))
            (run / "result.json").write_text(json.dumps({
                "passed": False, "error": "Exploration horizontal speed exceeded safety limit",
                "events": [{"state": "EXPLORING"}, {"state": "FAILSAFE_LANDED"}]}))
            (run / "truth.json").write_text(json.dumps(truth))
            (run / "ground_truth.jsonl").write_text("".join(
                json.dumps({"time": t, "position": [0, 0, 1.5]}) + "\n" for t in (0, 1)))
            (run / "odometry.jsonl").write_text("".join(
                json.dumps({"time": t, "delivery_lag_s": lag, "received_wall_s": wall}) + "\n"
                for t, lag, wall in ((0, None, 10.0), (1, 0.02, 10.1), (2, 0.9, 13.1))))
            row = judge(run)
        self.assertFalse(row["escaped"])
        self.assertTrue(row["failsafe_landed"])
        self.assertEqual(row["error"], "Exploration horizontal speed exceeded safety limit")
        self.assertEqual(row["max_odometry_delivery_lag_s"], 0.9)
        self.assertEqual(row["max_scan_arrival_gap_s"], 3.0)
        self.assertIsNone(row["ate_rmse_m"])


if __name__ == "__main__":
    unittest.main()
