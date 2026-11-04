"""The visible notebook calculations drive results while helpers retain checks."""

import ast
from contextlib import redirect_stdout
from copy import deepcopy
import io
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import Mock

import matplotlib.pyplot as plt
import numpy as np

from race_common.geometry import plane_basis
from trajectory_tools.course import construct_course, read_json
from trajectory_tools.preview import plot_trajectory
from .notebook_tools import clearance_budget, plan_course, plan_race, checked_plan
from .uncertainty import run


def cells(filename):
    notebook = json.loads(Path(__file__).with_name(filename).read_text())
    return {cell["id"]: "".join(cell["source"]) for cell in notebook["cells"]}


def notebook_function(source, name):
    definition = next(node for node in ast.parse(source).body
                      if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = {"np": np}
    exec(compile(ast.Module(body=[definition], type_ignores=[]), "<notebook>", "exec"), namespace)
    return namespace[name]


class VisibleCalculationTests(unittest.TestCase):
    def test_visible_turning_velocity_calculation_matches_the_planned_reference(self):
        notebook = cells("02_trajectory.ipynb")
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(output):
            setup = notebook["day3-setup"].replace('root / "code/day3/data" / run_id',
                                                  f'Path({temporary!r}) / run_id')
            namespace = dict(__name__="__main__")
            exec(setup, namespace)
            namespace["display"] = lambda _: None
            exec(notebook["day3-template"], namespace)
            source = notebook["day3-surveys"].replace('conservative_mode = True', 'conservative_mode = False')
            source = source.replace('turning_waypoints = {}',
                                    'turning_waypoints = {1: [{"position": [1.7, .4, .7], "velocity": [0, -.1, 0]}]}')
            exec(source, namespace)
            exec(notebook["day3-baseline"], namespace)
            row = namespace["plan_summary"]["turning_details"][0]
            np.testing.assert_allclose(namespace["requested_velocity"], [0, -.1, 0])
            np.testing.assert_allclose(namespace["resulting_velocity"], row["planned_velocity_m_s"], atol=1e-12)
            self.assertGreater(namespace["baseline_trajectory"].time_scale, 1.)
        self.assertIn("Turning velocity [m/s], requested / resulting:", output.getvalue())

    def test_visible_crossing_choices_drive_geometry_and_failed_edits_clear_the_course(self):
        notebook = cells("02_trajectory.ipynb")
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            setup = notebook["day3-setup"].replace('root / "code/day3/data" / run_id',
                                                  f'Path({temporary!r}) / run_id')
            namespace = dict(__name__="__main__")
            exec(setup, namespace)
            namespace["display"] = lambda _: None
            exec(notebook["day3-template"], namespace)
            source = notebook["day3-surveys"].replace('gate_speeds = {}',
                                                      'gate_speeds = {"forward": .4, "return": .25}')
            source = source.replace('conservative_mode = True', 'conservative_mode = False')
            source = source.replace('crossing_offsets = {}', 'crossing_offsets = {"forward": [0, .05, 0]}')
            source = source.replace('turning_waypoints = {}', 'turning_waypoints = {1: [[1.6, .4, .7]]}')
            exec(source, namespace)
            exec(notebook["day3-gate-geometry"], namespace)
            np.testing.assert_allclose(namespace["crossing_point"], [1, .05, .7])
            self.assertAlmostEqual(namespace["radial_space"], .25)
            exec(notebook["day3-baseline"], namespace)
            details = namespace["baseline_plan"].summary()["crossing_details"]
            self.assertEqual([row["requested_speed_m_s"] for row in details], [.4, .25, .4, .25])
            with self.assertRaisesRegex(ValueError, "aperture clearance"):
                exec(source.replace('[0, .05, 0]', '[0, .31, 0]'), namespace)
            self.assertIsNone(namespace["course"])
            self.assertIsNone(namespace["baseline_plan"])
            self.assertIsNone(namespace["candidate_plan"])

    def test_visible_margin_matches_projection_for_correlated_and_rotated_covariance(self):
        calculate = notebook_function(cells("02_trajectory.ipynb")["day3-baseline"], "our_gate_margin")
        covariance = np.array([[.0004, .0001, 0], [.0001, .0009, .0002], [0, .0002, .0016]])
        for normal in ([1, 0, 0], [0, 0, 1], [1, 2, 3]):
            with self.subTest(normal=normal):
                basis = plane_basis(normal)
                sigma, margin = calculate(basis, covariance, .01, .03, 2.5)
                expected = np.sqrt(np.linalg.eigvalsh(basis.T @ covariance @ basis).max())
                self.assertAlmostEqual(sigma, expected)
                self.assertAlmostEqual(margin, .01 + .03 + 2.5 * expected)

    def test_edited_margin_drives_the_plan_and_failed_edits_clear_it(self):
        root = Path(__file__).resolve().parents[2]
        course = construct_course(read_json(root / "examples/course.shuttle.synthetic.json"), laps=2)
        source = cells("02_trajectory.ipynb")["day3-baseline"]
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            namespace = dict(np=np, deepcopy=deepcopy, course=course, course_provenance={},
                             output=Path(temporary), clearance_budget=clearance_budget,
                             plan_course=plan_course, plan_race=plan_race, checked_plan=checked_plan,
                             gate_speeds={}, crossing_offsets={}, turning_waypoints={},
                             conservative_mode=False, target_speed_m_s=.3,
                             plot_trajectory=plot_trajectory, plt=plt, display=lambda _: None)
            exec(source, namespace)
            baseline_margin = namespace["baseline_plan"].course["limits"]["clearance_margin_m"]
            exec(source.replace("k * plane_std_m", "k * plane_std_m + 0.02"), namespace)
            plan = namespace["baseline_plan"]
            chosen = plan.course["limits"]["clearance_margin_m"]
            self.assertGreater(chosen, baseline_margin)
            self.assertEqual(chosen, namespace["budget"]["required_margin_m"])
            self.assertEqual(plan.provenance["clearance_budget"], namespace["budget"])
            with self.assertRaisesRegex(ValueError, "Gate margin"):
                exec(source.replace("k * plane_std_m", "0.0"), namespace)
            self.assertIsNone(namespace["baseline_plan"])
            self.assertIsNone(namespace["candidate_plan"])

    def test_margin_helper_rejects_nonfinite_or_understated_results(self):
        course = {"hoops": [dict(centre=[0, 0, 1], normal=[1, 0, 0], radius_m=.5, name="gate")]}
        for result in ((np.nan, .2), (.02, np.inf), (0, .2), (.02, 0), (-.1, .2), (.1, .2)):
            with self.subTest(result=result), self.assertRaisesRegex(ValueError, "Gate margin"):
                clearance_budget(course, np.eye(3) * .0004, survey_allowance_m=.01,
                                 tracking_allowance_m=.03, sigma_multiplier=2.5,
                                 margin_calculation=lambda *_: result)

    def test_visible_tracking_errors_have_correct_sign_norm_and_unmatched_mask(self):
        source = cells("02_trajectory.ipynb")["day3-recording"]
        targets = np.array([[0, 0, 0], [1, 2, 3], [np.nan] * 3, [4, 5, 6]])
        positions = np.array([[3, 4, 0], [0, 4, 1], [999, 999, 999], [4, 5, 6]])
        comparison = SimpleNamespace(positions=positions, targets=targets)
        analyze = Mock(return_value=(comparison, [], {}))
        namespace = dict(np=np, Path=Path, output=Path("unused-plots"), analyze_recording=analyze)
        source = source.replace("telemetry_csv = None", 'telemetry_csv = "attempt/race.csv"')
        source = source.replace("flown_course_json = None", 'flown_course_json = "attempt/race.course.json"')
        with redirect_stdout(io.StringIO()):
            exec(source, namespace)
        np.testing.assert_allclose(namespace["errors_m"][[0, 1, 3]], [[3, 4, 0], [-1, 2, -2], [0, 0, 0]])
        np.testing.assert_array_equal(namespace["matched"], [True, True, False, True])
        np.testing.assert_allclose(namespace["distances_m"], [5, 3, 0])
        self.assertAlmostEqual(namespace["rms_error_m"], np.sqrt(34 / 3))
        self.assertEqual(namespace["maximum_error_m"], 5)
        analyze.assert_called_once_with(Path("attempt/race.csv"), Path("attempt/race.course.json"), phase="race")

    def test_visible_speed_policy_stops_ramps_and_saturates_at_cruise(self):
        policy = notebook_function(cells("04_extra_uncertainty.ipynb")["uncertainty-policy"], "our_speed_policy")
        for clearance, expected in ((-.01, 0), (0, 0), (.075, .4), (.15, .8), (.3, .8)):
            with self.subTest(clearance=clearance):
                self.assertAlmostEqual(policy(clearance, .8), expected)

    def test_speed_policy_changes_recommendations_without_changing_filter_or_motion(self):
        policy = notebook_function(cells("04_extra_uncertainty.ipynb")["uncertainty-policy"], "our_speed_policy")
        baseline = run(7, "dropout", speed_policy=policy)
        slower = run(7, "dropout", speed_policy=lambda clearance, nominal: .5 * policy(clearance, nominal))
        self.assertTrue(np.any(baseline["speed"] > 0))
        self.assertTrue(np.any(baseline["clearance"] <= 0))
        np.testing.assert_array_equal(baseline["speed"][baseline["clearance"] <= 0], 0)
        np.testing.assert_allclose(slower["speed"], baseline["speed"] * .5)
        for key in baseline:
            if key != "speed":
                np.testing.assert_array_equal(baseline[key], slower[key], err_msg=key)

    def test_speed_helper_rejects_invalid_or_unsafe_recommendations(self):
        for speed in (np.nan, np.inf, -.1, 2.0, .2):
            with self.subTest(speed=speed), self.assertRaisesRegex(ValueError, "Speed policy"):
                run(7, "dropout", speed_policy=lambda *_: speed)


if __name__ == "__main__":
    unittest.main()
