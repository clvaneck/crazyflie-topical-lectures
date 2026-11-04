"""Numbered notebook boundaries and the shared Day 1 radio-label convention."""

import json
from pathlib import Path
import unittest

from .notebook_tools import drone_uri
import day1.crazyflie as crazyflie


class LayoutTests(unittest.TestCase):
    def test_sticker_labels_match_day_one(self):
        for label, suffix in ((0, '00'), (2, '02'), ('A', '0A'), ('E7', 'E7')):
            self.assertEqual(drone_uri(label), crazyflie.drone_uri(label))
            self.assertTrue(drone_uri(label).endswith('E7E7E7E7' + suffix))
        self.assertIs(drone_uri, crazyflie.drone_uri)
        self.assertEqual(drone_uri('A', channel=60), 'radio://0/60/2M/E7E7E7E70A')
        for label in (None, -1, 256, 'unknown'):
            with self.subTest(label=label), self.assertRaises(crazyflie.DroneError):
                drone_uri(label)

    def test_numbered_notebooks_keep_the_coupled_filter_work_together(self):
        folder = Path(__file__).parent
        groups = {
            '01_tracking.ipynb': ('day2-tracking-setup', 'day2-intro', 'day2-position-velocity-demo'),
            '02_calibration.ipynb': ('day2-calibration-setup', 'day2-e2'),
            '03_height_velocity.ipynb': ('day2-setup', 'day2-e3', 'day2-e4', 'day2-e5', 'day2-e6', 'day2-e7'),
        }
        for name, expected in groups.items():
            notebook = json.loads((folder/name).read_text())
            cells = {cell['id']: ''.join(cell['source']) for cell in notebook['cells']}
            self.assertTrue(set(expected).issubset(cells))
            setup = cells[expected[0]]
            self.assertIn('"data" / run_id', setup)
            for optional in ('experiments.py', 'kalman.filter', 'extra_notebook_tools'):
                self.assertNotIn(optional, setup)
            if name == '01_tracking.ipynb':
                headings = [''.join(cell['source']).lstrip() for cell in notebook['cells']
                            if cell['cell_type'] == 'markdown']
                self.assertTrue(any(text.startswith('### E1A: particle tracking') for text in headings))
        extra = json.loads((folder/'extra_six_state.ipynb').read_text())
        self.assertIn('day2-six-state-model', {cell['id'] for cell in extra['cells']})


if __name__ == '__main__':
    unittest.main()
