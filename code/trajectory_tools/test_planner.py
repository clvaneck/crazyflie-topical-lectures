"""Offline construction, continuous reference bounds and package CLI regressions."""

from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
from scipy.interpolate import BPoly

from .course import construct_course, crossing_region, gate_outline, read_json, validate_course
from .planner import Trajectory, _derivative_bound, build_trajectory, validate_crossings


ROOT = Path(__file__).resolve().parents[2]


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.template = read_json(ROOT / "examples/course.synthetic.json")
        self.gate = deepcopy(self.template["hoops"][0])
        self.gate["name"] = "gate-1"

    def test_synthetic_course_endpoints_crossings_and_continuity(self):
        course = construct_course(self.template)
        trajectory = build_trajectory(course)
        spline = trajectory.spline
        np.testing.assert_allclose(spline(0), course["start"], atol=1e-12)
        np.testing.assert_allclose(spline(trajectory.duration_s), course["finish"])
        np.testing.assert_allclose(spline([0, trajectory.duration_s], 1), 0, atol=1e-12)
        np.testing.assert_allclose(spline([0, trajectory.duration_s], 2), 0, atol=1e-12)
        for time in spline.x[1:-1]:
            for derivative in (0, 1, 2):
                np.testing.assert_allclose(spline(time - 1e-8, derivative), spline(time + 1e-8, derivative), atol=1e-7)
        for gate, time in zip(course["hoops"], trajectory.crossing_times_s):
            np.testing.assert_allclose(spline(time), gate["centre"], atol=1e-12)
            self.assertGreater(spline(time, 1) @ gate["normal"], 0)

    def test_subdivided_derivative_hulls_retain_continuous_bounds(self):
        random = np.random.default_rng(19)
        for _ in range(20):
            spline = BPoly(random.normal(size=(6, 4, 3)), [0, 1, 2.5, 4, 6])
            times = np.linspace(0, 6, 4001)
            for order in (1, 2):
                coarse = _derivative_bound(spline, order, 0)
                refined = _derivative_bound(spline, order, 2)
                sampled = float(np.linalg.norm(spline(times, order), axis=1).max())
                self.assertLessEqual(refined, coarse + 1e-10)
                self.assertLessEqual(sampled, refined + 1e-10)

    def test_repeated_forward_return_laps_keep_source_and_disable_flight(self):
        self.template["enabled_for_flight"] = True
        self.template["finish"] = self.template["start"]
        original = deepcopy(self.template)
        course = construct_course(self.template, [self.gate], ["gate-1:+", "gate-1:-"], laps=2)
        self.assertEqual(self.template, original)
        self.assertFalse(course["enabled_for_flight"])
        self.assertEqual(len(course["hoops"]), 4)
        self.assertEqual(len({gate["name"] for gate in course["hoops"]}), 4)
        self.assertEqual(course["hoops"][1]["normal"], [-1, 0, 0])
        trajectory = build_trajectory(course)
        self.assertTrue(np.all(np.diff(trajectory.crossing_times_s) > 0))

    def test_horizontal_rectangle_and_conservative_aperture(self):
        gate = {"name": "horizontal", "centre": [0, 0, 0.7], "normal": [0, 0, 1],
                "radius_m": 0.4, "shape": "rectangle", "width_m": 1.0, "height_m": 0.8,
                "width_axis": [1, 0, 0], "height_axis": [0, 1, 0]}
        self.template.update(start=[0, 0, 0.1], finish=[0, 0, 1.4])
        course = construct_course(self.template, [gate], ["horizontal:+"])
        build_trajectory(course)
        outline = gate_outline(course["hoops"][0])
        np.testing.assert_allclose(outline[:, 2], 0.7)
        self.assertAlmostEqual(np.ptp(outline[:, 0]), 1.0)
        gate["radius_m"] = 0.5
        with self.assertRaisesRegex(ValueError, "inscribed"):
            gate_outline(gate)

    def test_speed_acceleration_tilt_and_position_bounds(self):
        self.template["limits"]["maximum_tilt_deg"] = 2
        self.template["limits"]["maximum_acceleration_m_s2"] = 0.04
        trajectory = build_trajectory(self.template)
        _, p, v, a = trajectory.sample(200)
        limits = self.template["limits"]
        self.assertLessEqual(np.max(np.linalg.norm(v, axis=1)), trajectory.speed_bound_m_s + 1e-10)
        self.assertLessEqual(trajectory.speed_bound_m_s, limits["maximum_speed_m_s"])
        self.assertLessEqual(np.max(np.linalg.norm(a, axis=1)), trajectory.acceleration_bound_m_s2 + 1e-10)
        self.assertLessEqual(trajectory.acceleration_bound_m_s2, limits["maximum_acceleration_m_s2"])
        tilt = np.rad2deg(np.arctan2(np.linalg.norm(a[:, :2], axis=1), 9.81 + a[:, 2]))
        self.assertLessEqual(float(np.max(tilt)), limits["maximum_tilt_deg"])
        self.assertTrue(np.all(p >= limits["bounds_min"]))
        self.assertTrue(np.all(p <= limits["bounds_max"]))

    def test_bad_geometry_limits_and_unknown_crossings_are_rejected(self):
        for key, value in (("target_speed_m_s", float("nan")), ("maximum_tilt_deg", 90), ("pre_post_distance_m", -1)):
            course = deepcopy(self.template)
            course["limits"][key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                validate_course(course)
        with self.assertRaisesRegex(ValueError, "unknown gate"):
            construct_course(self.template, [self.gate], ["missing:+"])
        with self.assertRaisesRegex(ValueError, "duplicate survey"):
            construct_course(self.template, [self.gate, self.gate], ["gate-1:+"])
        self.template["limits"]["pre_post_distance_m"] = 4
        with self.assertRaisesRegex(ValueError, "bounds"):
            build_trajectory(self.template)

    def test_unscheduled_reverse_crossing_is_rejected(self):
        self.template["finish"] = [0.1, 0, 0.7]
        with self.assertRaisesRegex(ValueError, "unexpected intersection"):
            build_trajectory(self.template)

    def test_parallel_planes_on_opposite_sides_are_both_checked(self):
        self.template["limits"]["bounds_min"][0] = -3
        self.template["limits"]["bounds_max"][0] = 3
        self.template["finish"] = [0.1, 0, 0.7]
        first = dict(self.gate, name="positive", centre=[1, 0, 0.7])
        second = dict(self.gate, name="negative", centre=[-1, 0, 0.7], normal=[-1, 0, 0])
        course = construct_course(self.template, [first, second], ["positive:+", "positive:-", "negative:+"])
        # Returning to the finish crosses the negative plane for a second,
        # unlisted time. Its distance from the origin equals the other plane's.
        with self.assertRaisesRegex(ValueError, "unexpected intersection"):
            build_trajectory(course)

    def test_sample_count_is_bounded(self):
        trajectory = build_trajectory(self.template)
        with self.assertRaisesRegex(ValueError, "two million"):
            trajectory.sample(1_000_000)

    def plane_test_trajectory(self, points, times=(0.5,)):
        spline = BPoly.from_derivatives(np.arange(len(points)), [[point] for point in points])
        return Trajectory(spline, times, 0, 0)

    def test_finite_circle_bypass_is_allowed_but_frame_margin_is_not(self):
        for bypass_y, allowed in ((2.0, True), (0.85, True), (0.65, False), (0.2, False)):
            trajectory = self.plane_test_trajectory([[0, 0, .7], [2, 0, .7],
                                                     [2, bypass_y, .7], [0, bypass_y, .7]])
            with self.subTest(bypass_y=bypass_y):
                if allowed:
                    validate_crossings(self.template, trajectory)
                else:
                    with self.assertRaises(ValueError):
                        validate_crossings(self.template, trajectory)

    def test_generated_reference_can_return_around_the_gate(self):
        self.template["finish"] = [0, 3, .7]
        self.template["limits"]["bounds_max"] = [4, 5, 1.5]
        trajectory = build_trajectory(self.template)
        self.assertEqual(len(trajectory.crossing_times_s), 1)
        np.testing.assert_allclose(trajectory.spline(trajectory.duration_s), [0, 3, .7])

    def test_explicit_frame_padding_and_invalid_values(self):
        trajectory = self.plane_test_trajectory([[0, 0, .7], [2, 0, .7], [2, .85, .7], [0, .85, .7]])
        validate_crossings(self.template, trajectory)
        self.template["hoops"][0]["frame_margin_m"] = .4
        with self.assertRaisesRegex(ValueError, "frame safety margin"):
            validate_crossings(self.template, trajectory)
        for margin in (-1, float("nan"), float("inf")):
            with self.subTest(margin=margin), self.assertRaisesRegex(ValueError, "frame_margin_m"):
                construct_course(self.template, frame_margin_m=margin)
        course = construct_course(self.template, frame_margin_m=0.15)
        self.assertEqual(course["hoops"][0]["frame_margin_m"], .15)

    def test_rectangle_bypass_uses_axes_and_full_opening_not_inscribed_circle(self):
        gate = dict(self.gate, shape="rectangle", width_m=2, height_m=1,
                    width_axis=[0, 1, 0], height_axis=[0, 0, 1])
        # The wide opening outside its inscribed circle is still a passage.
        self.assertEqual(crossing_region(gate, [1, .7, .7], .2), "opening")
        self.assertEqual(crossing_region(gate, [1, 1.2, .7], .2), "frame")
        self.assertEqual(crossing_region(gate, [1, 1.4, .7], .2), "outside")
        self.assertEqual(crossing_region(gate, [1, 0, 1.4], .2), "frame")
        self.assertEqual(crossing_region(gate, [1, 0, 1.6], .2), "outside")

    def test_distinct_coplanar_gates_are_not_conflated(self):
        second = dict(self.gate, name="second", centre=[1, 2, .7], normal=[-1, 0, 0])
        self.template["hoops"] = [self.gate, second]
        trajectory = self.plane_test_trajectory([[0, 0, .7], [2, 0, .7], [2, 2, .7], [0, 2, .7]], times=(.5, 2.5))
        validate_crossings(self.template, trajectory)

    def test_coplanar_travel_outside_gate_is_allowed(self):
        trajectory = self.plane_test_trajectory([[0, 0, .7], [2, 0, .7], [2, 2, .7], [1, 2, .7], [1, 3, .7]])
        validate_crossings(self.template, trajectory)
        unsafe = self.plane_test_trajectory([[0, 0, .7], [2, 0, .7], [2, .6, .7], [1, .6, .7], [1, .7, .7]])
        with self.assertRaisesRegex(ValueError, "finite footprint"):
            validate_crossings(self.template, unsafe)

    def test_horizontal_gate_bypass_uses_gate_frame(self):
        rotation = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]])
        self.gate["centre"] = (rotation @ np.asarray(self.gate["centre"])).tolist()
        self.gate["normal"] = [0, 0, 1]
        self.template["hoops"] = [self.gate]
        points = np.array([[0, 0, .7], [2, 0, .7], [2, 2, .7], [0, 2, .7]]) @ rotation.T
        validate_crossings(self.template, self.plane_test_trajectory(points))

    def test_six_decimal_survey_normal_is_renormalized(self):
        self.gate["normal"] = [0.707107, 0.707107, 0]
        normal = np.asarray(self.gate["normal"])
        self.template["start"] = (np.asarray(self.gate["centre"]) - 0.8 * normal).tolist()
        self.template["finish"] = (np.asarray(self.gate["centre"]) + 0.8 * normal).tolist()
        course = construct_course(self.template, [self.gate], ["gate-1:+"])
        build_trajectory(course)

    def test_bom_cli_outputs_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            source = directory / "template.json"
            source.write_text(json.dumps(self.template), encoding="utf-8-sig")
            destination = directory / "plan"
            command = [sys.executable, str(ROOT / "scripts/plan_trajectory.py"), "--course", str(source), "--output", str(destination)]
            environment = dict(os.environ, MPLBACKEND="Agg")
            result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual({file.name for file in destination.iterdir()}, {"course.json", "trajectory.csv", "summary.json", "preview.png"})
            self.assertFalse(read_json(destination / "course.json")["enabled_for_flight"])
            self.assertGreater((destination / "preview.png").stat().st_size, 10000)
            result = subprocess.run(command, env=environment, capture_output=True, text=True, timeout=90)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("already exists", result.stderr)

            # Exercise the actual survey CLI, not only the Python construction API.
            self.template["finish"] = self.template["start"]
            source.write_text(json.dumps(self.template), encoding="utf-8-sig")
            gate = dict(self.gate, shape="rectangle", width_m=1.2, height_m=1.0,
                        width_axis=[0, 1, 0], height_axis=[0, 0, 1])
            survey = directory / "gate.json"
            survey.write_text(json.dumps(gate), encoding="utf-8-sig")
            measured = directory / "measured"
            result = subprocess.run(
                [*command[:-1], str(measured), "--gate-survey", str(survey),
                 "--crossing", "gate-1:+", "--crossing", "gate-1:-", "--laps", "2"],
                env=environment, capture_output=True, text=True, timeout=90)
            self.assertEqual(result.returncode, 0, result.stderr)
            exported = read_json(measured / "course.json")
            self.assertEqual(len(exported["hoops"]), 4)
            self.assertEqual(exported["hoops"][0]["width_m"], 1.2)
            self.assertFalse(exported["enabled_for_flight"])


if __name__ == "__main__":
    unittest.main()
