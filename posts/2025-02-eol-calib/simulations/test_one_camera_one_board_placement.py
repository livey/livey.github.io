"""Regression checks for the article's centred-placement explanation.

From the repository root:
    python -m unittest discover -s content/posts/2025-02-EOL-calib/simulations \
        -p 'test_one_camera_one_board_placement.py'
"""

import csv
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

import one_camera_one_board_placement as placement


class CentredPlacementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.off_centre, _, cls.off_result = placement.optimize()
        cls.centred, cls.history, cls.result = placement.optimize_centred(cls.off_centre)

    def test_centred_variables_and_start(self):
        self.assertTrue(self.result.success)
        np.testing.assert_array_equal(self.history[:, :2], 0.0)
        np.testing.assert_allclose(self.history[0, 2:], self.off_centre[2:])
        np.testing.assert_array_equal(self.history[-1], self.centred)
        np.testing.assert_allclose(placement.project(self.centred[None, :3])[0],
                                   placement.PRINCIPAL)
        placement.validate_feasibility(self.centred)
        placement.validate_jacobian(self.centred)

    def test_article_values_and_tradeoff(self):
        centred = placement.metrics(self.centred)
        off_centre = placement.metrics(self.off_centre)
        self.assertAlmostEqual(centred["score"], 0.48927244, delta=2e-5)
        self.assertAlmostEqual(off_centre["score"], 0.42765855, delta=2e-5)
        self.assertAlmostEqual(centred["predicted_translation_rms_mm"], 1.0744774, delta=2e-4)
        self.assertAlmostEqual(centred["predicted_rotation_rms_deg"], 0.06656518, delta=2e-5)
        self.assertLess(centred["predicted_translation_rms_mm"],
                        off_centre["predicted_translation_rms_mm"])
        self.assertGreater(centred["predicted_rotation_rms_deg"],
                           off_centre["predicted_rotation_rms_deg"])
        reduction = 100 * (1 - off_centre["score"] / centred["score"])
        self.assertAlmostEqual(reduction, 12.59296, delta=0.01)

    def test_mirrors_reflect_geometry_not_just_the_centre(self):
        original, _, _ = placement.board_geometry(self.off_centre)
        grid = original.reshape(5, 7, 3)
        mirrored = placement.mirrored_poses(self.off_centre)
        cases = [("Original", False, False), ("Reflect x", True, False),
                 ("Reflect y", False, True), ("Reflect x and y", True, True)]
        for label, flip_x, flip_y in cases:
            with self.subTest(placement=label):
                expected = grid[::-1 if flip_y else 1, ::-1 if flip_x else 1]
                expected = expected.reshape(-1, 3) * [-1 if flip_x else 1,
                                                    -1 if flip_y else 1, 1]
                actual, _, _ = placement.board_geometry(mirrored[label])
                np.testing.assert_allclose(actual, expected, atol=1e-12, rtol=1e-12)
        rows = placement.mirror_symmetry_checks(self.off_centre)
        self.assertEqual(len(rows), 4)
        self.assertTrue(all(row["feasible"] and row["symmetry_check_passed"] for row in rows))

    def test_feasibility_checks_bounds_and_nonfinite_values(self):
        too_far = np.array([0.0, 0.0, 3.01, 0.0, 0.0, 0.0])
        self.assertGreaterEqual(np.min(placement.constraints(too_far)), 0)
        with self.assertRaises(ValueError):
            placement.validate_feasibility(too_far)
        invalid = self.off_centre.copy()
        invalid[0] = np.nan
        with self.assertRaises(ValueError):
            placement.validate_feasibility(invalid)

    def test_exports_are_complete_and_need_no_noise_trials(self):
        reference = self.off_centre.copy()
        with tempfile.TemporaryDirectory(prefix="ocob-centred-test-") as directory:
            output = Path(directory)
            with patch.object(placement, "monte_carlo", side_effect=AssertionError("Unexpected noise trials")):
                report = placement.reproduce_centred_explanation(reference, self.off_result, output)
            np.testing.assert_array_equal(reference, self.off_centre)
            with (output / "ocob_centred_reproducibility.json").open() as handle:
                saved = json.load(handle)
            self.assertEqual(saved, report)
            self.assertTrue(saved["centred_optimizer_success"])
            self.assertTrue(saved["off_centre_optimizer_success"])
            self.assertEqual(saved["centred_options"], placement.CENTRED_OPTIONS)
            self.assertEqual(saved["versions"]["numpy"], np.__version__)
            for name, count in [("ocob_centred_comparison.csv", 2),
                                ("ocob_mirror_symmetry_checks.csv", 4)]:
                with (output / name).open() as handle:
                    self.assertEqual(len(list(csv.DictReader(handle))), count)
            with (output / "ocob_centred_optimization_history.csv").open() as handle:
                history = list(csv.DictReader(handle))
            self.assertGreater(len(history), 1)
            self.assertTrue(all(float(row["x_m"]) == float(row["y_m"]) == 0.0
                                for row in history))
            self.assertEqual(float(history[-1]["score"]), saved["comparison"][0]["score"])


if __name__ == "__main__":
    unittest.main()
