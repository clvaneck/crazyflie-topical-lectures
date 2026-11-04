"""Keep a notebook's course and computed trajectory together for export."""

from copy import deepcopy
import csv
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path

import numpy as np

from trajectory_tools.planner import Trajectory, build_trajectory
from trajectory_tools.course import configure_crossings, construct_course, read_json, turning_velocities
from trajectory_tools.conservative import route_transitions, slow_defaults
from race_common.hoop import Hoop


def clearance_budget(course, position_covariance, *, survey_allowance_m,
                     tracking_allowance_m, sigma_multiplier=2.5, margin_calculation=None):
    """Project Day 2's position covariance into each gate's opening plane.

    Add survey and control-tracking allowances as conservative bounds. Their
    sum is a planning allowance, not a calibrated collision probability.
    A notebook calculation receives (U, C, survey, tracking, k) and returns
    (plane standard deviation, margin); the shared calculation bounds it below.
    """
    if not isinstance(course, dict):
        raise ValueError("Select valid course inputs before budgeting clearance.")
    covariance = np.asarray(position_covariance, dtype=float)
    if (covariance.shape != (3, 3) or not np.isfinite(covariance).all()
            or not np.allclose(covariance, covariance.T, rtol=0, atol=1e-12)):
        raise ValueError("Position covariance must be a finite symmetric 3x3 matrix in m^2.")
    if np.linalg.eigvalsh(covariance).min() < -1e-12:
        raise ValueError("Position covariance must be positive semidefinite.")
    allowances = np.asarray([survey_allowance_m, tracking_allowance_m, sigma_multiplier])
    if not np.isfinite(allowances).all() or (allowances < 0).any() or sigma_multiplier == 0:
        raise ValueError("Allowances must be nonnegative and the sigma multiplier positive.")
    gates = []
    for entry in course['hoops']:
        gate = Hoop(entry['centre'], entry['normal'], entry['radius_m'])
        plane_covariance = gate.plane_covariance(covariance)
        minimum_sigma = float(np.sqrt(max(0.0, np.linalg.eigvalsh(plane_covariance).max())))
        minimum_margin = float(survey_allowance_m + tracking_allowance_m + sigma_multiplier * minimum_sigma)
        if margin_calculation is None:
            sigma, margin = minimum_sigma, minimum_margin
        else:
            sigma, margin = margin_calculation(gate.plane_basis(), covariance.copy(),
                                               survey_allowance_m, tracking_allowance_m, sigma_multiplier)
            sigma, margin = float(sigma), float(margin)
            required_margin = max(minimum_margin,
                                  survey_allowance_m + tracking_allowance_m + sigma_multiplier * sigma)
            if (not np.isfinite([sigma, margin]).all() or sigma < 0 or margin < 0
                    or sigma < minimum_sigma - 1e-12 or margin < required_margin - 1e-12):
                raise ValueError("Gate margin calculation must retain the projected uncertainty and allowances.")
        gates.append({'name': entry['name'], 'plane_std_m': sigma, 'required_margin_m': margin})
    if not gates:
        raise ValueError("A clearance budget needs at least one gate.")
    return {'position_covariance_m2': covariance.tolist(),
            'survey_allowance_m': float(survey_allowance_m),
            'tracking_allowance_m': float(tracking_allowance_m),
            'sigma_multiplier': float(sigma_multiplier),
            'required_margin_m': max(gate['required_margin_m'] for gate in gates),
            'gates': gates}


def _trajectory_signature(trajectory):
    digest = hashlib.sha256()
    digest.update(trajectory.spline.x.tobytes())
    digest.update(trajectory.spline.c.tobytes())
    digest.update(repr((trajectory.crossing_times_s, trajectory.speed_bound_m_s,
                        trajectory.acceleration_bound_m_s2, trajectory.time_scale,
                        trajectory.turning_times_s)).encode("ascii"))
    return digest.hexdigest()


@dataclass(frozen=True)
class CoursePlan:
    course: dict
    trajectory: Trajectory
    course_snapshot: str
    trajectory_signature: str
    provenance_snapshot: str

    @property
    def provenance(self):
        """Return a fresh copy so later notebook edits cannot change provenance."""
        return json.loads(self.provenance_snapshot)

    def plot(self):
        """Preview the checked path and physical gates without opening a radio."""
        from trajectory_tools.preview import plot_trajectory

        course, trajectory = checked_plan(self)
        return plot_trajectory(course, trajectory)

    def summary(self):
        course, trajectory = checked_plan(self)
        return dict(course_name=course["course_name"], start=course["start"], finish=course["finish"],
                    limits=course["limits"], planned_duration_s=trajectory.duration_s,
                    speed_bound_m_s=trajectory.speed_bound_m_s,
                    acceleration_bound_m_s2=trajectory.acceleration_bound_m_s2,
                    crossings=[gate["name"] for gate in course["hoops"]],
                    crossing_details=crossing_details(course, trajectory),
                    turning_waypoints=course.get("turning_waypoints", []),
                    turning_details=turning_details(course, trajectory),
                    time_scale=trajectory.time_scale,
                    planning_profile=course.get("planning_profile", "custom"),
                    stop_at_approaches=course.get("stop_at_approaches", False),
                    frame_depth_m=course.get("frame_depth_m"),
                    enabled_for_flight=False)

    def configure_race(self, *, baseline, controller=None, kalman=None,
                       factor_range=None, range_checked=False):
        """Bind the checked trajectory to one explicit, bounded tuning choice."""
        from .tuning import race_tuning

        checked_plan(self)
        tuning = race_tuning(baseline, controller=controller, kalman=kalman,
                             factor_range=factor_range, range_checked=range_checked)
        return RaceConfiguration(self, json.dumps(tuning, sort_keys=True))

    def export_race_pair(self, *, output, **choices):
        """Convenience route for another team's notebook using the same checks."""
        return self.configure_race(**choices).export(output)


@dataclass(frozen=True)
class RaceConfiguration:
    plan: CoursePlan
    tuning_snapshot: str

    def summary(self):
        from .tuning import tuning_changes

        tuning = json.loads(self.tuning_snapshot)
        return dict(self.plan.summary(), drone_uri=tuning["uri"],
                    mode=tuning["mode"], parameter_changes=tuning_changes(tuning))

    def export(self, directory):
        """Export two runner inputs and their trajectory/provenance evidence."""
        from .tuning import export_race_pair

        course, _ = checked_plan(self.plan)
        tuning = json.loads(self.tuning_snapshot)
        paths = export_race_pair(course, tuning, directory, summary=_export_summary(self.plan))
        for submission in paths.values():
            save_plan(self.plan, submission["course"].parent)
        return paths


def plan_race(template, *, gates=(), crossing_order=(), laps=1, speed=None,
              clearance=None, approach_distance=None, gate_speeds=None,
              crossing_offsets=None, turning_waypoints=None, conservative=True, provenance=None):
    """Build a race from notebook choices while retaining shared template limits.

    Gates are survey dictionaries; crossing names use 'gate:+' or 'gate:-'.
    Clearance is an extra allowance outside the drone envelope, in metres.
    Gate speeds use m/s; crossing offsets are XYZ metres in the gate plane.
    Turning keys count completed crossings (0 means before the first).
    Each waypoint is XYZ metres or {position: XYZ, velocity: XYZ m/s}; velocity
    defaults to zero. Nonzero turning velocities require conservative=False.
    Conservative defaults tighten motion limits and route stopped transitions
    around modelled gates. The common setup supplies bounds, endpoints and size.
    """
    if isinstance(laps, bool) or not isinstance(laps, int) or laps < 1:
        raise ValueError("laps must be a positive integer")
    if type(conservative) is not bool:
        raise ValueError("conservative must be True or False")
    if gates:
        defaults = template.get("gate_defaults", {})
        geometry_fields = {"radius_m", "shape", "width_m", "height_m", "width_axis", "height_axis",
                           "frame_margin_m", "frame_depth_m"}
        if not isinstance(defaults, dict) or set(defaults) - geometry_fields:
            raise ValueError("Shared gate_defaults may contain aperture and frame dimensions only")
        gates = [dict(deepcopy(defaults), **deepcopy(gate)) for gate in gates]
        if any("radius_m" not in gate for gate in gates):
            raise ValueError("Supply gate aperture dimensions in the survey or shared gate_defaults")
    shared_margin = template["limits"]["clearance_margin_m"]
    if conservative:
        template = slow_defaults(template)
    else:
        template = deepcopy(template)
        template["planning_profile"] = "custom"
        template["stop_at_approaches"] = False
        template["trajectory_version"] = 1
    if gates and turning_waypoints is None:
        # A new survey/order starts a new route; old course-specific detours
        # are retained only when the caller explicitly supplies them.
        template.pop("turning_waypoints", None)
    if gates or crossing_order or laps != 1:
        course = construct_course(template, gates, crossing_order, laps)
    else:
        course = deepcopy(template)
        course["enabled_for_flight"] = False
    for name, value in (("target_speed_m_s", speed), ("clearance_margin_m", clearance),
                        ("pre_post_distance_m", approach_distance)):
        if value is not None:
            # Notebook choices may increase the shared clearance, never reduce it.
            if name == "clearance_margin_m" and value < shared_margin:
                raise ValueError("Clearance must retain the shared template minimum")
            if name == "clearance_margin_m" and conservative:
                course["limits"][name] = max(value, template["limits"][name])
            else:
                course["limits"][name] = value
    course = configure_crossings(course, gate_speeds=gate_speeds, crossing_offsets=crossing_offsets,
                                 turning_waypoints=turning_waypoints)
    if conservative:
        course = route_transitions(course)
    return plan_course(course, provenance=provenance)


def crossing_details(course, trajectory):
    """Expose requested crossing choices and the speed after common time scaling."""
    return [dict(name=gate["name"], gate_id=gate.get("gate_id", gate["name"]),
                 crossing_position_m=trajectory.spline(time).tolist(),
                 requested_speed_m_s=gate.get("speed_m_s", course["limits"]["target_speed_m_s"]),
                 planned_speed_m_s=float(np.linalg.norm(trajectory.spline(time, 1))))
            for gate, time in zip(course["hoops"], trajectory.crossing_times_s)]


def turning_details(course, trajectory):
    """Compare requested world velocities with the time-scaled reference targets."""
    details = []
    expected = sum(len(turn["positions_m"]) for turn in course.get("turning_waypoints", []))
    if len(trajectory.turning_times_s) != expected:
        raise ValueError("Turning waypoint times do not match the course; replan before export")
    times = iter(trajectory.turning_times_s)
    for turn in sorted(course.get("turning_waypoints", []), key=lambda item: item["after_crossing"]):
        for index, (point, velocity) in enumerate(zip(turn["positions_m"], turning_velocities(turn)), start=1):
            time = next(times)
            details.append(dict(after_crossing=turn["after_crossing"], waypoint_index=index,
                                position_m=list(point), time_s=time,
                                requested_velocity_m_s=list(velocity),
                                planned_velocity_m_s=trajectory.spline(time, 1).tolist()))
    return details


def load_plan(course_path):
    """Reload a matched course/summary pair and recompute its reference."""
    path = Path(course_path)
    course = read_json(path)
    summary_path = path.with_name("summary.json")
    try:
        summary = read_json(summary_path)
    except FileNotFoundError as error:
        raise ValueError("Keep course.json and summary.json together; re-export the selected plan if its summary is missing") from error
    validate_plan_summary(course, summary)
    # Only the input evidence is restored. Timings, bounds and waypoint
    # diagnostics are recomputed from this course, never trusted from a file.
    plan = plan_course(course, provenance=summary["provenance"])
    checked_plan(plan)
    return plan


def validate_plan_summary(course, summary):
    """Require input evidence bound to this exact planned course."""
    if not isinstance(summary, dict) or not isinstance(summary.get("provenance"), dict):
        raise ValueError("Plan summary needs a provenance snapshot; re-export the selected plan")
    if summary.get("course_sha256") != _course_digest(course):
        raise ValueError("Plan summary does not match the course; re-export the selected plan")


def _course_digest(course):
    """Bind evidence to the course content, independent of JSON layout or BOM."""
    content = json.dumps(course, sort_keys=True, allow_nan=False).encode("utf-8")
    return hashlib.sha256(content).hexdigest()


def plan_course(course, *, provenance=None):
    """Return a matched pair only after the planner accepts a private input copy."""
    if not isinstance(course, dict):
        raise ValueError("No valid course inputs. Correct them and rerun course construction before planning.")
    course = deepcopy(course)
    trajectory = build_trajectory(course)
    return CoursePlan(course, trajectory, json.dumps(course, sort_keys=True),
                      _trajectory_signature(trajectory),
                      json.dumps(provenance if provenance is not None else {}, sort_keys=True))


def checked_plan(plan):
    """Reject missing or edited plans before any export files are written."""
    if not isinstance(plan, CoursePlan):
        raise ValueError("No valid selected plan. Correct the inputs and rerun planning before export.")
    if (json.dumps(plan.course, sort_keys=True) != plan.course_snapshot
            or _trajectory_signature(plan.trajectory) != plan.trajectory_signature):
        raise ValueError("Plan changed since validation. Rerun planning before export.")
    if plan.course.get("enabled_for_flight") is not False:
        raise ValueError("Notebook exports must keep enabled_for_flight=false.")
    return deepcopy(plan.course), plan.trajectory


def _export_summary(plan):
    """Build the same validated evidence for file and in-memory exports."""
    course, trajectory = checked_plan(plan)
    provenance = plan.provenance
    return {
        **provenance,
        "provenance": provenance,
        "course_sha256": _course_digest(course),
        "planned_duration_s": trajectory.duration_s,
        "speed_bound_m_s": trajectory.speed_bound_m_s,
        "acceleration_bound_m_s2": trajectory.acceleration_bound_m_s2,
        "crossings": [{"name": gate["name"], "time_s": time}
                      for gate, time in zip(course["hoops"], trajectory.crossing_times_s)],
        "crossing_details": crossing_details(course, trajectory),
        "turning_details": turning_details(course, trajectory),
        "time_scale": trajectory.time_scale,
        "note": "Planned reference only; no flight commands or physical qualification.",
    }


def save_plan(plan, directory):
    """Write the checked course, reference samples and provenance together.

    The notebook owns the run directory. Re-exporting there replaces the three
    plan files; use a new run label to retain a separate comparison.
    """
    course, trajectory = checked_plan(plan)
    times, positions, velocities, accelerations = trajectory.sample(
        course["limits"]["setpoint_rate_hz"])
    summary = _export_summary(plan)
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "course.json").write_text(json.dumps(course, indent=2) + "\n", encoding="utf-8")
    np.savetxt(directory / "trajectory.csv",
               np.column_stack((times, positions, velocities, accelerations)),
               delimiter=",", comments="",
               header="time_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,ax_m_s2,ay_m_s2,az_m_s2")
    (directory / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def save_example_replay(plan, directory, lag_s=0.12):
    """Save a labelled delayed-reference example for the offline R7 analysis.

    The position stream is exactly p_reference(t - lag), clipped at the start.
    This exercises timing and plotting; it is not a simulated drone/controller.
    Like save_plan, this replaces generated files in the notebook-owned directory.
    """
    course, trajectory = checked_plan(plan)
    if not np.isfinite(lag_s) or lag_s < 0:
        raise ValueError("Example lag must be finite and nonnegative.")
    times, positions, _, _ = trajectory.sample(course['limits']['setpoint_rate_hz'])
    delayed_positions = trajectory.spline(np.maximum(times - lag_s, 0.0))
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    course_path = directory / 'course.json'
    course_path.write_text(json.dumps(course, indent=2) + '\n', encoding='utf-8')
    telemetry_path = directory / 'telemetry.synthetic.csv'
    fields = ['event', 'host_monotonic_s', 'phase', 'x_m', 'y_m', 'z_m',
              'command_x_m', 'command_y_m', 'command_z_m']
    with telemetry_path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for time, target, position in zip(times, positions, delayed_positions):
            writer.writerow({'event': 'command', 'host_monotonic_s': time, 'phase': 'race',
                             'command_x_m': target[0], 'command_y_m': target[1],
                             'command_z_m': target[2]})
            writer.writerow({'event': 'position', 'host_monotonic_s': time,
                             'x_m': position[0], 'y_m': position[1], 'z_m': position[2]})
    source = {'source': 'synthetic delayed-reference example', 'lag_s': float(lag_s),
              'plan_provenance': plan.provenance,
              'note': 'Prescribed positions; no drone dynamics, estimator or flight-time prediction.'}
    (directory / 'source.json').write_text(json.dumps(source, indent=2) + '\n', encoding='utf-8')
    return telemetry_path, course_path
