import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from day3.hoop_view import load, table

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


class HoopViewLoaderTest(unittest.TestCase):
    def load_data(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hoops.json"
            path.write_text(json.dumps(data))
            return load(path)

    def test_course_example(self):
        hoops, bounds, ends = load(EXAMPLES / "course.shuttle.synthetic.json")
        self.assertEqual([h["name"] for h in hoops], ["forward", "return"])
        self.assertEqual(bounds[1].tolist(), [2.5, 1.0, 1.5])
        self.assertIn("start", ends)
        self.assertIn("same", table(hoops))

    def test_aliases_and_unnormalised_normal(self):
        hoops, bounds, _ = self.load_data({"gates": [
            {"id": 7, "center": {"x": 1, "y": 2, "z": 0.8}, "direction": [2, 0, 0], "diameter": 1.0}],
            "arena": {"min": [0, 0, 0], "max": [7, 3, 2]}})
        self.assertEqual(hoops[0]["name"], "7")
        self.assertEqual(hoops[0]["normal"], [1.0, 0.0, 0.0])
        self.assertAlmostEqual(hoops[0]["radius_m"], 0.5)
        self.assertEqual(bounds[1].tolist(), [7, 3, 2])

    def test_rim_points_fit(self):
        angles = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        rim = [[3.0, 0.4 * np.cos(t), 1 + 0.4 * np.sin(t)] for t in angles]
        hoops, _, _ = self.load_data([{"edges": rim, "normal": [1, 0, 0]}])
        self.assertTrue(np.allclose(hoops[0]["centre"], [3, 0, 1]))
        self.assertAlmostEqual(hoops[0]["radius_m"], 0.4)

    def test_missing_field_is_named(self):
        with self.assertRaisesRegex(ValueError, "missing normal"):
            self.load_data([{"centre": [0, 0, 1], "radius_m": 0.4}])


if __name__ == "__main__":
    unittest.main()
