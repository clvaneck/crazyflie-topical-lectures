"""Course/trajectory pairing stays valid across notebook edits and failures."""

from pathlib import Path
from copy import deepcopy
import csv
import json
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from trajectory_tools.course import construct_course, read_json
from .notebook_tools import (
    checked_plan, plan_course, plan_race, load_plan, save_plan, clearance_budget, save_example_replay,
)


class PlanTests(unittest.TestCase):
    def course(self):
        root = Path(__file__).resolve().parents[2]
        return construct_course(read_json(root / "examples/course.shuttle.synthetic.json"), laps=2)

    def test_plan_owns_a_copy_and_returns_a_matched_disabled_course(self):
        source = self.course()
        plan = plan_course(source)
        source["limits"]["target_speed_m_s"] = .45
        course, trajectory = checked_plan(plan)
        self.assertIs(trajectory, plan.trajectory)
        self.assertEqual(course["limits"]["target_speed_m_s"], .3)
        self.assertIs(course["enabled_for_flight"], False)
        self.assertEqual(len(course["hoops"]), len(trajectory.crossing_times_s))

    def test_notebook_api_builds_surveyed_crossings_and_retains_shared_limits(self):
        template = self.course()
        gates = [dict(template["hoops"][0], name="A")]
        original = deepcopy(template)
        original_gates = deepcopy(gates)
        plan = plan_race(template, gates=gates, crossing_order=["A:+", "A:-"], laps=2,
                         speed=.4, clearance=.1, approach_distance=.35, conservative=False)
        course, _ = checked_plan(plan)
        self.assertEqual(template, original)
        self.assertEqual(gates, original_gates)
        self.assertEqual([gate["name"] for gate in course["hoops"]],
                         ["lap-1-1-A", "lap-1-2-A", "lap-2-1-A", "lap-2-2-A"])
        self.assertEqual([gate["normal"][0] for gate in course["hoops"]], [1, -1, 1, -1])
        expected = dict(original["limits"], target_speed_m_s=.4, clearance_margin_m=.1,
                        pre_post_distance_m=.35)
        self.assertEqual(course["limits"], expected)
        self.assertFalse(course["enabled_for_flight"])

    def test_notebook_api_checks_motion_limits_and_does_not_relabel_a_prepared_course(self):
        course = self.course()
        self.assertEqual(plan_race(course, conservative=False).course,
                         dict(course, trajectory_version=1, planning_profile="custom", stop_at_approaches=False))
        for choices in (dict(clearance=.01), dict(speed=.6), dict(approach_distance=1.6),
                        dict(laps=True), dict(laps=0), dict(speed=float("nan"))):
            with self.subTest(choices=choices), self.assertRaises(ValueError):
                plan_race(course, **choices)

    def test_notebook_api_plots_and_summarizes_only_a_checked_plan(self):
        import matplotlib.pyplot as plt
        plan = plan_race(self.course(), provenance={"source_surveys": ["gate.json"]})
        figure = plan.plot()
        self.assertTrue(figure.axes)
        plt.close(figure)
        summary = plan.summary()
        self.assertEqual(summary["crossings"], [gate["name"] for gate in plan.course["hoops"]])
        self.assertEqual(summary["planned_duration_s"], plan.trajectory.duration_s)
        summary["limits"]["maximum_speed_m_s"] = 99
        self.assertEqual(plan.course["limits"]["maximum_speed_m_s"], .3)
        plan.course["start"][0] += .1
        for action in (plan.plot, plan.summary):
            with self.assertRaisesRegex(ValueError, "changed since validation"):
                action()

    def test_minimal_gate_inputs_produce_a_slow_centred_stopped_course(self):
        setup = self.course()
        setup["gate_defaults"] = {"radius_m": .5}
        original = deepcopy(setup)
        gate = dict(name="A", centre=[1, 0, .7], normal=[1, 0, 0])
        plan = plan_race(setup, gates=[gate], crossing_order=["A:+", "A:-"], laps=2)
        limits = plan.course["limits"]
        self.assertEqual(setup, original)
        self.assertEqual(plan.summary()["planning_profile"], "conservative")
        self.assertEqual(limits["target_speed_m_s"], .2)
        self.assertLessEqual(plan.trajectory.speed_bound_m_s, .3)
        self.assertLessEqual(plan.trajectory.acceleration_bound_m_s2, .5)
        self.assertLess(plan.trajectory.duration_s, 45.)
        self.assertEqual(limits["maximum_tilt_deg"], 10.)
        self.assertGreaterEqual(limits["clearance_margin_m"], .1)
        self.assertTrue(plan.course["stop_at_approaches"])
        self.assertFalse(plan.course["enabled_for_flight"])
        for key in ("bounds_min", "bounds_max", "drone_radius_m", "maximum_tracking_error_m", "telemetry_timeout_s"):
            self.assertEqual(limits[key], original["limits"][key])
        self.assertEqual(plan.course["start"], setup["start"])
        self.assertEqual(plan.course["finish"], setup["finish"])
        knots = plan.trajectory.spline(plan.trajectory.spline.x)
        for crossing, time in zip(plan.course["hoops"], plan.trajectory.crossing_times_s):
            np.testing.assert_allclose(plan.trajectory.spline(time), crossing["centre"])
            for sign in (-1, 1):
                approach = np.asarray(crossing["centre"]) + sign * limits["pre_post_distance_m"] * np.asarray(crossing["normal"])
                index = int(np.argmin(np.linalg.norm(knots - approach, axis=1)))
                np.testing.assert_allclose(knots[index], approach)
                np.testing.assert_allclose(plan.trajectory.spline(plan.trajectory.spline.x[index], 1), 0, atol=1e-12)

    def test_minimal_reverse_crossing_and_horizontal_gate_route_around_the_frame(self):
        setup = self.course()
        gate = dict(setup["hoops"][0], name="A")
        for normal, order in (([1, 0, 0], ["A:-"]), ([0, 0, 1], ["A:+"])):
            with self.subTest(normal=normal):
                plan = plan_race(setup, gates=[dict(gate, normal=normal)], crossing_order=order)
                self.assertTrue(plan.course["turning_waypoints"])
                self.assertEqual(len(plan.trajectory.crossing_times_s), 1)
                self.assertLessEqual(plan.trajectory.speed_bound_m_s, .3)
                self.assertLessEqual(plan.trajectory.acceleration_bound_m_s2, .5)
                if normal == [1, 0, 0]:
                    np.testing.assert_allclose(plan.trajectory.spline(plan.trajectory.spline.x)[:, 2], .7)
                for stop in plan.course["turning_waypoints"]:
                    for point in stop["positions_m"]:
                        self.assertTrue(np.all(np.asarray(point) >= setup["limits"]["bounds_min"]))
                        self.assertTrue(np.all(np.asarray(point) <= setup["limits"]["bounds_max"]))

    def test_three_gates_need_only_geometry_and_order_on_the_shared_setup(self):
        setup = self.course()
        setup["start"] = setup["finish"] = [0, -1.5, .7]
        setup["limits"].update(bounds_min=[-.5, -2.5, -.2], bounds_max=[4.5, 3, 3.5])
        gates = [dict(name="A", centre=[1, -1.5, .7], normal=[1, 0, 0], radius_m=.5),
                 dict(name="B", centre=[3, 1.5, 1], normal=[-1, 0, 0], radius_m=.5),
                 dict(name="C", centre=[1, 1.5, 1.7], normal=[0, 0, 1], radius_m=.5)]
        plan = plan_race(setup, gates=gates, crossing_order=["A:+", "B:+", "C:+"])
        self.assertEqual(len(plan.trajectory.crossing_times_s), 3)
        self.assertTrue(plan.course["turning_waypoints"])
        for gate, time in zip(gates, plan.trajectory.crossing_times_s):
            np.testing.assert_allclose(plan.trajectory.spline(time), gate["centre"])

    def test_conservative_routing_fails_closed_and_never_expands_the_site(self):
        setup = self.course()
        setup["limits"].update(bounds_min=[-.5, -.6, .1], bounds_max=[2.5, .6, 1.3])
        original = deepcopy(setup)
        gate = dict(setup["hoops"][0], name="A")
        with self.assertRaisesRegex(ValueError, "No conservative.*route"):
            plan_race(setup, gates=[gate], crossing_order=["A:-"])
        self.assertEqual(setup, original)
        with self.assertRaisesRegex(ValueError, "endpoint.*gate envelope"):
            plan_race(self.course(), turning_waypoints={0: [[1, 0, .7]]})
        for depth in (0, -.1, float("nan")):
            setup = self.course()
            setup["frame_depth_m"] = depth
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                plan_race(setup)

    def test_conservative_defaults_preserve_tighter_limits_and_require_explicit_custom_mode(self):
        setup = self.course()
        setup["limits"].update(target_speed_m_s=.08, maximum_speed_m_s=.1,
                                maximum_acceleration_m_s2=.2, maximum_tilt_deg=5,
                                clearance_margin_m=.15, takeoff_duration_s=4.)
        plan = plan_race(setup)
        self.assertEqual(plan.course["limits"], setup["limits"])
        self.assertLessEqual(plan.trajectory.speed_bound_m_s, .1)
        with self.assertRaisesRegex(ValueError, "target speed exceeds"):
            plan_race(self.course(), speed=.4)
        custom = plan_race(self.course(), speed=.4, conservative=False)
        self.assertEqual(custom.summary()["planning_profile"], "custom")
        self.assertFalse(custom.course["stop_at_approaches"])
        self.assertEqual(custom.course["limits"]["maximum_speed_m_s"], .5)

    def test_pose_only_gates_require_explicit_shared_aperture_dimensions(self):
        setup = self.course()
        gate = dict(name="A", centre=[1, 0, .7], normal=[1, 0, 0])
        with self.assertRaisesRegex(ValueError, "aperture dimensions"):
            plan_race(setup, gates=[gate], crossing_order=["A:+"])
        setup["gate_defaults"] = {"radius_m": .5}
        plan_race(setup, gates=[gate], crossing_order=["A:+", "A:-"])
        with self.assertRaisesRegex(ValueError, "insufficient aperture"):
            plan_race(setup, gates=[dict(gate, radius_m=.2)], crossing_order=["A:+"])
        setup["gate_defaults"]["centre"] = [0, 0, 0]
        with self.assertRaisesRegex(ValueError, "dimensions only"):
            plan_race(setup, gates=[gate], crossing_order=["A:+"])

    def test_new_geometry_does_not_inherit_old_course_detours(self):
        setup = self.course()
        setup["turning_waypoints"] = [dict(after_crossing=0, positions_m=[[9, 9, 9]])]
        gate = dict(name="A", centre=[1, 0, .7], normal=[1, 0, 0], radius_m=.5)
        plan = plan_race(setup, gates=[gate], crossing_order=["A:+", "A:-"])
        self.assertFalse(plan.course.get("turning_waypoints"))
        self.assertEqual(setup["turning_waypoints"][0]["positions_m"], [[9, 9, 9]])

    def test_crossing_choices_keep_physical_geometry_and_apply_to_laps_or_one_passage(self):
        template = self.course()
        gate = dict(template["hoops"][0], name="A")
        source = deepcopy(gate)
        plan = plan_race(template, gates=[gate], crossing_order=["A:+", "A:-"], laps=2,
                         gate_speeds={"A": .4, "lap-1-2-A": .2},
                         crossing_offsets={"A": [0, .05, 0], "lap-1-2-A": [0, -.05, 0]},
                         turning_waypoints={1: [[1.6, .4, .7], [1.6, .25, .7]]}, conservative=False)
        self.assertEqual(gate, source)
        details = plan.summary()["crossing_details"]
        self.assertEqual([row["requested_speed_m_s"] for row in details], [.4, .2, .4, .4])
        for index, (entry, time) in enumerate(zip(plan.course["hoops"], plan.trajectory.crossing_times_s)):
            self.assertEqual(entry["centre"], source["centre"])
            self.assertEqual(entry["radius_m"], source["radius_m"])
            expected = np.asarray(source["centre"]) + [0, -.05 if index == 1 else .05, 0]
            np.testing.assert_allclose(plan.trajectory.spline(time), expected, atol=1e-12)
            self.assertAlmostEqual(details[index]["planned_speed_m_s"],
                                   entry["speed_m_s"] / plan.trajectory.time_scale)
        self.assertEqual(plan.course["trajectory_version"], 1)
        self.assertFalse(plan.course["enabled_for_flight"])
        points = plan.trajectory.spline(plan.trajectory.spline.x)
        for stop in ([1.6, .4, .7], [1.6, .25, .7]):
            index = int(np.argmin(np.linalg.norm(points - stop, axis=1)))
            time = plan.trajectory.spline.x[index]
            np.testing.assert_allclose(points[index], stop, atol=1e-12)
            np.testing.assert_allclose(plan.trajectory.spline(time, 1), 0, atol=1e-12)
            np.testing.assert_allclose(plan.trajectory.spline(time, 2), 0, atol=1e-12)
        for time in plan.trajectory.spline.x[1:-1]:
            for derivative in (0, 1, 2):
                np.testing.assert_allclose(plan.trajectory.spline(time - 1e-8, derivative),
                                           plan.trajectory.spline(time + 1e-8, derivative), atol=1e-7)

    def test_turning_velocity_is_attained_after_time_scaling_with_c2_joins(self):
        waypoints = {1: [{"position": [1.7, .4, .7], "velocity": [.05, -.1, 0]},
                         [1.7, .2, .7]]}
        original = deepcopy(waypoints)
        plan = plan_race(self.course(), conservative=False, turning_waypoints=waypoints)
        self.assertEqual(waypoints, original)
        self.assertGreater(plan.trajectory.time_scale, 1.)
        import matplotlib.pyplot as plt
        figure = plan.plot()
        labels = figure.axes[0].get_legend_handles_labels()[1]
        self.assertIn("Fly-through waypoints", labels)
        self.assertIn("Stop-and-turn waypoints", labels)
        plt.close(figure)
        rows = plan.summary()["turning_details"]
        self.assertEqual(len(rows), 2)
        for index, row in enumerate(rows):
            requested = [.05, -.1, 0] if index == 0 else [0, 0, 0]
            self.assertEqual(row["after_crossing"], 1)
            self.assertEqual(row["waypoint_index"], index + 1)
            self.assertEqual(row["requested_velocity_m_s"], requested)
            np.testing.assert_allclose(row["planned_velocity_m_s"],
                                       np.asarray(requested) / plan.trajectory.time_scale, atol=1e-12)
            np.testing.assert_allclose(plan.trajectory.spline(row["time_s"]), row["position_m"], atol=1e-12)
            np.testing.assert_allclose(plan.trajectory.spline(row["time_s"], 2), 0, atol=1e-12)
        for time in plan.trajectory.spline.x[1:-1]:
            for derivative in (0, 1, 2):
                np.testing.assert_allclose(plan.trajectory.spline(time - 1e-8, derivative),
                                           plan.trajectory.spline(time + 1e-8, derivative), atol=1e-7)
        # Input edits cannot alter the already validated position/velocity targets.
        waypoints[1][0]["velocity"][0] = 99
        self.assertEqual(plan.summary()["turning_details"][0]["requested_velocity_m_s"], [.05, -.1, 0])
        with tempfile.TemporaryDirectory() as temporary:
            summary = save_plan(plan, temporary)
            self.assertEqual(summary["turning_details"], rows)
            loaded = load_plan(Path(temporary) / "course.json")
            self.assertEqual(loaded.summary()["turning_details"], rows)
            np.testing.assert_array_equal(loaded.trajectory.spline.c, plan.trajectory.spline.c)

    def test_omitted_and_zero_turning_velocities_preserve_the_stopped_reference(self):
        reference = plan_race(self.course(), turning_waypoints={1: [[1.6, .4, .7]]})
        for waypoint in ({"position": [1.6, .4, .7]},
                         {"position": [1.6, .4, .7], "velocity": [0, 0, 0]}):
            plan = plan_race(self.course(), turning_waypoints={1: [waypoint]})
            np.testing.assert_array_equal(plan.trajectory.spline.c, reference.trajectory.spline.c)
            np.testing.assert_array_equal(plan.trajectory.spline.x, reference.trajectory.spline.x)
            row = plan.summary()["turning_details"][0]
            self.assertEqual(row["requested_velocity_m_s"], [0, 0, 0])
            np.testing.assert_allclose(row["planned_velocity_m_s"], 0, atol=1e-12)

    def test_bad_waypoint_velocity_data_and_nonzero_conservative_targets_are_rejected(self):
        for waypoint in ({"velocity": [0, .1, 0]}, {"position": [1.7, .4, .7], "acceleration": [0, 0, 0]},
                         {"position": [1.7, .4, .7], "velocity": [0, .1]},
                         {"position": [1.7, .4, .7], "velocity": [0, float("nan"), 0]},
                         {"position": [1.7, .4, .7], "velocity": [.4, .4, 0]}):
            with self.subTest(waypoint=waypoint), self.assertRaises(ValueError):
                plan_race(self.course(), conservative=False, turning_waypoints={1: [waypoint]})
        with self.assertRaisesRegex(ValueError, "conservative=False"):
            plan_race(self.course(), turning_waypoints={1: [dict(position=[1.7, .4, .7], velocity=[0, -.1, 0])]})

    def test_waypoint_velocities_cannot_override_endpoint_rest_or_merged_knots(self):
        for slot, point in ((0, [0, 0, .7]), (1, [1.4, 0, .7]), (4, [0, 0, .7])):
            with self.subTest(slot=slot), self.assertRaisesRegex(ValueError, "Coincident control points"):
                plan_race(self.course(), conservative=False,
                          turning_waypoints={slot: [dict(position=point, velocity=[.1, 0, 0])]})

    def test_turning_velocities_retain_whole_curve_bounds_and_crossing_checks(self):
        for point, velocity, error in (([1.7, .99, .7], [0, .4, 0], "control hull"),
                                       ([1.01, 0, .7], [.5, 0, 0], "unexpected intersection")):
            with self.subTest(point=point), self.assertRaisesRegex(ValueError, error):
                plan_race(self.course(), conservative=False,
                          turning_waypoints={1: [dict(position=point, velocity=velocity)]})

    def test_json_waypoint_velocity_count_is_checked_and_editing_it_invalidates_a_plan(self):
        plan = plan_race(self.course(), conservative=False,
                         turning_waypoints={1: [dict(position=[1.7, .4, .7], velocity=[0, -.1, 0])]})
        course = deepcopy(plan.course)
        course["turning_waypoints"][0]["velocities_m_s"] = []
        with self.assertRaisesRegex(ValueError, "one XYZ target velocity"):
            plan_course(course)
        plan.course["turning_waypoints"][0]["velocities_m_s"][0][0] = .05
        with self.assertRaisesRegex(ValueError, "changed since validation"):
            checked_plan(plan)

    def test_crossing_choices_reject_unknown_names_unsafe_offsets_and_bad_speeds(self):
        template = self.course()
        name = template["hoops"][0]["gate_id"]
        cases = [dict(gate_speeds={"missing": .3}), dict(crossing_offsets={"missing": [0, 0, 0]}),
                 dict(crossing_offsets={name: [.01, 0, 0]}), dict(crossing_offsets={name: [0, .31, 0]}),
                 dict(crossing_offsets={name: [0, float("nan"), 0]})]
        cases.extend(dict(gate_speeds={name: value}) for value in (0, -.1, .6, float("nan"), True))
        for choices in cases:
            with self.subTest(choices=choices), self.assertRaises(ValueError):
                plan_race(template, **choices)

    def test_turning_waypoints_remain_subject_to_volume_frame_and_crossing_checks(self):
        template = self.course()
        for choices in ({-1: [[0, 0, .7]]}, {99: [[0, 0, .7]]}, {True: [[0, 0, .7]]},
                        {1: []}, {1: [[0, 0]]}, {1: [[0, float("nan"), .7]]},
                        {1: [[9, 0, .7]]}, {1: [[1, .5, .7]]}, {1: [[.5, 0, .7]]}):
            with self.subTest(choices=choices), self.assertRaises(ValueError):
                plan_race(template, turning_waypoints=choices, conservative=False)

    def test_empty_choices_clear_overrides_and_new_choices_survive_save_reload(self):
        plan = plan_race(self.course(), conservative=False, gate_speeds={"forward": .4},
                         crossing_offsets={"forward": [0, .05, 0]},
                         turning_waypoints={0: [[-.1, .1, .7]], 4: [[-.1, -.1, .7]]})
        cleared = plan_race(plan.course, conservative=False, gate_speeds={}, crossing_offsets={}, turning_waypoints={})
        self.assertFalse(any("speed_m_s" in gate or "crossing_offset_m" in gate for gate in cleared.course["hoops"]))
        self.assertEqual(cleared.course["turning_waypoints"], [])
        with tempfile.TemporaryDirectory() as temporary:
            summary = save_plan(plan, temporary)
            loaded = load_plan(Path(temporary) / "course.json")
            self.assertEqual(loaded.course, plan.course)
            self.assertEqual(summary["crossing_details"], plan.summary()["crossing_details"])
            np.testing.assert_allclose(loaded.trajectory.spline.c, plan.trajectory.spline.c)

    def test_notebook_api_reloads_saved_course_before_planning_the_flight(self):
        plan = plan_race(self.course())
        with tempfile.TemporaryDirectory() as temporary:
            save_plan(plan, temporary)
            path = Path(temporary) / "course.json"
            loaded = load_plan(path)
            self.assertEqual(loaded.course, plan.course)
            self.assertEqual(loaded.provenance, plan.provenance)
            np.testing.assert_allclose(loaded.trajectory.spline.c, plan.trajectory.spline.c)
            path.write_text(json.dumps(dict(plan.course, enabled_for_flight=True)))
            with self.assertRaisesRegex(ValueError, "summary does not match"):
                load_plan(path)
            path.write_text("invalid JSON")
            with self.assertRaises(ValueError):
                load_plan(path)
            # Previously loaded state is an independent, still-valid snapshot.
            checked_plan(loaded)

    def test_relocated_plan_preserves_evidence_and_recomputes_diagnostics(self):
        provenance = dict(source_surveys=["survey-A.json"], source_template="setup.json",
                          clearance_budget={"required_margin_m": .1, "covariance": [[.01]]},
                          crossing_order=["A:+", "A:-"], laps=2)
        plan = plan_race(self.course(), provenance=provenance)
        with tempfile.TemporaryDirectory() as temporary:
            original = Path(temporary) / "selected"
            save_plan(plan, original)
            moved = Path(temporary) / "flight-laptop"
            original.rename(moved)
            summary_path = moved / "summary.json"
            summary = json.loads(summary_path.read_text())
            summary["planned_duration_s"] = -999
            summary["crossing_details"] = []
            summary_path.write_text(json.dumps(summary), encoding="utf-8-sig")
            course_path = moved / "course.json"
            course_path.write_text(json.dumps(plan.course, sort_keys=True), encoding="utf-8-sig")
            loaded = load_plan(course_path)
            self.assertEqual(loaded.provenance, plan.provenance)
            for index in range(2):
                exported = Path(temporary) / f"handover-{index}"
                result = save_plan(loaded, exported)
                self.assertEqual(result["provenance"], plan.provenance)
                for key, value in plan.provenance.items():
                    self.assertEqual(result[key], value)
                self.assertGreater(result["planned_duration_s"], 0)
                self.assertTrue(result["crossing_details"])
                loaded = load_plan(exported / "course.json")
                self.assertEqual(loaded.provenance, plan.provenance)

    def test_reload_requires_a_matching_provenance_summary(self):
        plan = plan_race(self.course())
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            summary = save_plan(plan, directory)
            path = directory / "course.json"
            companion = directory / "summary.json"
            companion.unlink()
            with self.assertRaisesRegex(ValueError, "Keep course.json and summary.json together"):
                load_plan(path)
            for invalid in ([], {}, dict(summary, provenance=[]),
                            dict(summary, course_sha256="wrong")):
                with self.subTest(summary=invalid):
                    companion.write_text(json.dumps(invalid))
                    with self.assertRaises(ValueError):
                        load_plan(path)
            companion.write_text("invalid JSON")
            with self.assertRaises(ValueError):
                load_plan(path)
            companion.write_text(json.dumps(summary))
            changed = deepcopy(plan.course)
            changed["yaw_deg"] += 1
            path.write_text(json.dumps(changed))
            with self.assertRaisesRegex(ValueError, "summary does not match"):
                load_plan(path)

    def test_missing_or_modified_plan_cannot_be_exported(self):
        with self.assertRaisesRegex(ValueError, "No valid selected plan"):
            checked_plan(None)
        with self.assertRaisesRegex(ValueError, "No valid course inputs"):
            plan_course(None)
        plan = plan_course(self.course())
        plan.course["limits"]["target_speed_m_s"] = .4
        with self.assertRaisesRegex(ValueError, "changed since validation"):
            checked_plan(plan)
        plan = plan_course(self.course())
        plan.trajectory.spline.c[0, 0, 0] += .1
        with self.assertRaisesRegex(ValueError, "changed since validation"):
            checked_plan(plan)

    def test_checked_course_can_be_changed_without_mutating_baseline(self):
        source = self.course()
        source["limits"]["pre_post_distance_m"] = .35
        baseline = plan_course(source)
        candidate, _ = checked_plan(baseline)
        candidate["limits"]["target_speed_m_s"] = .4
        candidate_plan = plan_course(candidate)
        restored, _ = checked_plan(candidate_plan)
        restored["limits"]["target_speed_m_s"] = source["limits"]["target_speed_m_s"]
        self.assertEqual(restored, source)
        self.assertEqual(checked_plan(baseline)[0], source)

    def test_invalid_approach_does_not_produce_a_plan(self):
        course = self.course()
        course["limits"]["pre_post_distance_m"] = 1.6
        with self.assertRaisesRegex(ValueError, "control hull"):
            plan_course(course)

    def test_provenance_is_snapshotted_and_inherited_without_shared_mutable_state(self):
        provenance = {"source_surveys": ["gate.json"], "crossing_order": ["gate:+"], "laps": 2}
        baseline = plan_course(self.course(), provenance=provenance)
        provenance["source_surveys"].append("unused.json")
        provenance["laps"] = 99
        candidate, _ = checked_plan(baseline)
        candidate["limits"]["target_speed_m_s"] = .4
        candidate_plan = plan_course(candidate, provenance=baseline.provenance)
        detached = candidate_plan.provenance
        detached["source_surveys"].clear()
        detached["laps"] = 10
        expected = {"source_surveys": ["gate.json"], "crossing_order": ["gate:+"], "laps": 2}
        self.assertEqual(baseline.provenance, expected)
        self.assertEqual(candidate_plan.provenance, expected)

    def test_saved_samples_and_provenance_belong_to_the_selected_plan(self):
        provenance = {"source_surveys": ["gate.json"], "laps": 2}
        plan = plan_course(self.course(), provenance=provenance)
        provenance["laps"] = 99
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "run"
            summary = save_plan(plan, directory)
            course = json.loads((directory / "course.json").read_text())
            self.assertEqual(course, plan.course)
            self.assertIs(course["enabled_for_flight"], False)
            self.assertEqual(summary["laps"], 2)
            self.assertEqual(summary, json.loads((directory / "summary.json").read_text()))
            samples = np.genfromtxt(directory / "trajectory.csv", delimiter=",", names=True)
            for name, expected in zip(("x_m", "y_m", "z_m"),
                                      plan.trajectory.spline(samples["time_s"]).T):
                np.testing.assert_allclose(samples[name], expected)
            self.assertEqual(len(summary["crossings"]), len(course["hoops"]))

    def test_invalid_plan_does_not_write_or_replace_export_files(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary) / "run"
            with self.assertRaises(ValueError):
                save_plan(None, directory)
            self.assertFalse(directory.exists())
            plan = plan_course(self.course())
            save_plan(plan, directory)
            before = {path.name: path.read_bytes() for path in directory.iterdir()}
            plan.course["enabled_for_flight"] = True
            with self.assertRaises(ValueError):
                save_plan(plan, directory)
            self.assertEqual(before, {path.name: path.read_bytes() for path in directory.iterdir()})

    def test_clearance_budget_projects_state_covariance_and_preserves_course(self):
        course = self.course()
        original = deepcopy(course)
        covariance = np.diag([0.01, 0.04, 0.09])
        budget = clearance_budget(course, covariance, survey_allowance_m=.01,
                                  tracking_allowance_m=.03, sigma_multiplier=2.5)
        # The X-normal gate sees only Y/Z uncertainty; the largest sigma is .3 m.
        self.assertAlmostEqual(budget['gates'][0]['plane_std_m'], .3)
        self.assertAlmostEqual(budget['required_margin_m'], .01 + .03 + 2.5 * .3)
        self.assertEqual(course, original)
        course['hoops'][0]['normal'] = [0, 0, 1]
        horizontal = clearance_budget(course, covariance, survey_allowance_m=.01,
                                      tracking_allowance_m=.03, sigma_multiplier=2.5)
        self.assertAlmostEqual(horizontal['gates'][0]['plane_std_m'], .2)

    def test_clearance_budget_rejects_invalid_covariance_and_allowances(self):
        for covariance in (np.eye(2), np.diag([1, -1, 1]), np.eye(3) * np.nan,
                           [[1, .1, 0], [0, 1, 0], [0, 0, 1]]):
            with self.subTest(covariance=covariance), self.assertRaises(ValueError):
                clearance_budget(self.course(), covariance, survey_allowance_m=.01,
                                 tracking_allowance_m=.03)
        for keyword, value in (('survey_allowance_m', -.01), ('tracking_allowance_m', np.nan),
                               ('sigma_multiplier', 0)):
            arguments = dict(survey_allowance_m=.01, tracking_allowance_m=.03)
            arguments[keyword] = value
            with self.subTest(keyword=keyword), self.assertRaises(ValueError):
                clearance_budget(self.course(), np.eye(3), **arguments)

    def test_example_replay_uses_the_validated_plan_and_declares_its_lag(self):
        plan = plan_course(self.course(), provenance={'source_surveys': ['gate.json']})
        with tempfile.TemporaryDirectory() as directory:
            telemetry, course_path = save_example_replay(plan, directory, lag_s=.12)
            self.assertEqual(json.loads(course_path.read_text()), plan.course)
            with telemetry.open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            commands = [row for row in rows if row['event'] == 'command']
            positions = [row for row in rows if row['event'] == 'position']
            times = np.array([float(row['host_monotonic_s']) for row in positions])
            measured = np.array([[float(row[f'{axis}_m']) for axis in 'xyz'] for row in positions])
            np.testing.assert_allclose(measured, plan.trajectory.spline(np.maximum(times - .12, 0)))
            np.testing.assert_allclose(
                [[float(row[f'command_{axis}_m']) for axis in 'xyz'] for row in commands],
                plan.trajectory.spline(times))
            self.assertTrue(all(row['phase'] == 'race' for row in commands))
            source = json.loads((Path(directory) / 'source.json').read_text())
            self.assertEqual(source['lag_s'], .12)
            self.assertIn('synthetic', source['source'])
            self.assertEqual(source['plan_provenance'], plan.provenance)
            self.assertIs(json.loads(course_path.read_text())['enabled_for_flight'], False)

    def test_invalid_example_replay_cannot_replace_existing_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'example'
            plan = plan_course(self.course())
            for lag in (-.1, np.nan):
                with self.subTest(lag=lag), self.assertRaises(ValueError):
                    save_example_replay(plan, path, lag_s=lag)
                self.assertFalse(path.exists())
            save_example_replay(plan, path)
            before = {p.name: p.read_bytes() for p in path.iterdir()}
            plan.course['enabled_for_flight'] = True
            with self.assertRaises(ValueError):
                save_example_replay(plan, path)
            self.assertEqual(before, {p.name: p.read_bytes() for p in path.iterdir()})

    def test_cli_preserves_a_directory_created_after_its_initial_check(self):
        from .plan import main

        root = Path(__file__).resolve().parents[2]
        with tempfile.TemporaryDirectory() as temporary:
            destination = Path(temporary) / "run"

            def another_run_claims_directory(*args):
                destination.mkdir()
                (destination / "keep.txt").write_text("earlier results")

            argv = ["day3", "--course", str(root / "examples/course.shuttle.synthetic.json"),
                    "--output", str(destination)]
            with patch("sys.argv", argv), patch("sys.stderr"), patch(
                    "day3.plan.plot_trajectory", side_effect=another_run_claims_directory):
                with self.assertRaises(SystemExit) as error:
                    main()
            self.assertEqual(error.exception.code, 1)
            self.assertEqual([path.name for path in destination.iterdir()], ["keep.txt"])
            self.assertEqual((destination / "keep.txt").read_text(), "earlier results")


if __name__ == "__main__":
    unittest.main()
