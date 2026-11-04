"""C2-continuous geometric reference with conservative continuous-time bounds."""

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import BPoly, PPoly

from race_common.geometry import unit_vector

from .course import (crossing_region, frame_margin, gate_basis,
                     opening_half_size, turning_velocities, validate_course)


@dataclass(frozen=True)
class Trajectory:
    spline: BPoly
    crossing_times_s: tuple[float, ...]
    speed_bound_m_s: float
    acceleration_bound_m_s2: float
    time_scale: float = 1.0
    turning_times_s: tuple[float, ...] = ()

    @property
    def duration_s(self):
        return float(self.spline.x[-1])

    def sample(self, rate_hz):
        count = int(np.ceil(self.duration_s * rate_hz)) + 1
        if count > 2_000_000:
            raise ValueError("trajectory would exceed two million samples; reduce rate or duration")
        times = np.linspace(0, self.duration_s, count)
        return times, self.spline(times), self.spline(times, 1), self.spline(times, 2)


def build_trajectory(course):
    validate_course(course)
    if course.get("planning_profile") == "conservative":
        from .conservative import validate_transition_route

        validate_transition_route(course)
    limits = course["limits"]
    speed = limits["target_speed_m_s"]
    offset = limits["pre_post_distance_m"]
    points = [np.asarray(course["start"], dtype=float)]
    velocities = [np.zeros(3)]
    point_speeds = [speed]
    crossing_knots = []
    turning_knots, turning_targets = [], []
    turns = {turn["after_crossing"]: turn for turn in course.get("turning_waypoints", [])}

    def append(point, velocity, requested_speed):
        # A shared exit/entry waypoint can occur on a forward/return course.
        # Stop there rather than assigning two contradictory tangents.
        if np.linalg.norm(point - points[-1]) < 1e-8:
            velocities[-1] = np.zeros(3)
            point_speeds[-1] = min(point_speeds[-1], requested_speed)
        else:
            points.append(point)
            velocities.append(velocity)
            point_speeds.append(requested_speed)

    def append_turns(slot):
        if slot not in turns:
            return
        turn = turns[slot]
        for point, velocity in zip(turn["positions_m"], turning_velocities(turn)):
            target = np.asarray(velocity, dtype=float)
            append(np.asarray(point, dtype=float), target, speed)
            turning_knots.append(len(points) - 1)
            turning_targets.append(target)

    append_turns(0)
    for index, gate in enumerate(course["hoops"], start=1):
        centre = np.asarray(gate["centre"], dtype=float) + np.asarray(gate.get("crossing_offset_m", [0, 0, 0]))
        normal = unit_vector(gate["normal"])
        gate_speed = gate.get("speed_m_s", speed)
        approach_velocity = np.zeros(3) if course.get("stop_at_approaches", False) else gate_speed * normal
        append(centre - offset * normal, approach_velocity, gate_speed)
        append(centre, gate_speed * normal, gate_speed)
        crossing_knots.append(len(points) - 1)
        append(centre + offset * normal, approach_velocity, gate_speed)
        append_turns(index)
    append(np.asarray(course["finish"], dtype=float), np.zeros(3), speed)
    velocities[0] = np.zeros(3)
    velocities[-1] = np.zeros(3)
    for knot, target in zip(turning_knots, turning_targets):
        if not np.array_equal(velocities[knot], target):
            raise ValueError("Coincident control points conflict with the turning target velocity; use distinct points or zero velocity")
    points = np.asarray(points)
    velocities = np.asarray(velocities)
    segment_speeds = np.minimum(point_speeds[:-1], point_speeds[1:])
    durations = np.maximum(np.linalg.norm(np.diff(points, axis=0), axis=1) / segment_speeds, 0.2)

    # Quintic Bernstein coefficients impose position/velocity and zero
    # acceleration at every knot. Thus position, velocity and acceleration are
    # continuous, but jerk need not be continuous at joins.
    coefficients = []
    for index, duration in enumerate(durations):
        p, q = points[index:index + 2]
        v, w = velocities[index:index + 2] * duration
        coefficients.append([p, p + v / 5, p + 2 * v / 5,
                             q - 2 * w / 5, q - w / 5, q])
    spline = BPoly(np.asarray(coefficients).transpose(1, 0, 2),
                   np.concatenate(([0.0], np.cumsum(durations))))
    lower = np.asarray(limits["bounds_min"])
    upper = np.asarray(limits["bounds_max"])
    # A Bernstein polynomial stays in the convex hull of its coefficients.
    # This checks the whole curve, not just the points used to draw it.
    if np.any(spline.c < lower - 1e-10) or np.any(spline.c > upper + 1e-10):
        raise ValueError("trajectory control hull leaves bounds; adjust positions, waypoint velocities or approach distance")
    # Subdivision tightens the same continuous convex-hull bound. In stopped
    # default routes, a single hull otherwise makes short flights very slow.
    refinements = 2 if course.get("planning_profile") == "conservative" else 0
    speed_bound = _derivative_bound(spline, 1, refinements)
    acceleration_bound = _derivative_bound(spline, 2, refinements)
    # The acceleration-norm bound also conservatively guarantees positive
    # vertical thrust and a nominal tilt below the requested angle.
    tilt = np.deg2rad(limits["maximum_tilt_deg"])
    tilt_acceleration = 9.81 * np.tan(tilt) / (1 + np.tan(tilt))
    acceleration_limit = min(limits["maximum_acceleration_m_s2"], tilt_acceleration)
    scale = max(1.0, speed_bound / limits["maximum_speed_m_s"],
                np.sqrt(acceleration_bound / acceleration_limit)) * 1.001
    spline = BPoly(spline.c, spline.x * scale)
    crossing_times = tuple(float(spline.x[index]) for index in crossing_knots)
    trajectory = Trajectory(spline, crossing_times, speed_bound / scale,
                            acceleration_bound / scale**2, scale,
                            tuple(float(spline.x[index]) for index in turning_knots))
    validate_crossings(course, trajectory)
    return trajectory


def _split_bernstein(coefficients):
    """de Casteljau halves represent exactly the same curve on each half interval."""
    levels = [coefficients]
    while len(levels[-1]) > 1:
        levels.append((levels[-1][:-1] + levels[-1][1:]) / 2)
    left = np.asarray([level[0] for level in levels])
    right = np.asarray([level[-1] for level in reversed(levels)])
    return left, right


def _derivative_bound(spline, order, refinements):
    segments = list(np.moveaxis(spline.derivative(order).c, 1, 0))
    for _ in range(refinements):
        halves = []
        for coefficients in segments:
            halves.extend(_split_bernstein(coefficients))
        segments = halves
    return max(float(np.linalg.norm(coefficients, axis=1).max()) for coefficients in segments)


def _coplanar_curve_is_outside(coefficients, gate, clearance, depth=0):
    """Prove a planar segment avoids the finite footprint using convex hulls.

    Subdivision avoids rejecting a curve merely because its original bounding
    box overlaps the gate. An unresolved near-contact remains a rejection.
    """
    local = (coefficients - gate["centre"]) @ gate_basis(gate)
    lower, upper = local.min(axis=0), local.max(axis=0)
    outer = opening_half_size(gate) + frame_margin(gate) + clearance
    if gate.get("shape", "circle") == "rectangle":
        if np.any(lower > outer) or np.any(upper < -outer):
            return True
    else:
        closest = np.maximum(lower, np.minimum(0, upper))
        if np.linalg.norm(closest) > outer[0]:
            return True
    if depth >= 12:
        return False
    left, right = _split_bernstein(coefficients)
    return (_coplanar_curve_is_outside(left, gate, clearance, depth + 1)
            and _coplanar_curve_is_outside(right, gate, clearance, depth + 1))


def validate_crossings(course, trajectory):
    """Ignore plane extensions, but reject frame contact and unlisted passages."""
    spline = trajectory.spline
    limits = course["limits"]
    clearance = limits["drone_radius_m"] + limits["clearance_margin_m"]
    for gate in course["hoops"]:
        centre = np.asarray(gate["centre"])
        normal = unit_vector(gate["normal"])
        coefficients = (spline.c - centre) @ normal
        for index in np.flatnonzero(np.all(np.abs(coefficients) < 1e-10, axis=0)):
            if not _coplanar_curve_is_outside(spline.c[:, index], gate, clearance):
                raise ValueError(f"trajectory runs along gate plane inside finite footprint of {gate['name']}")
        plane = PPoly.from_bernstein_basis(BPoly(coefficients, spline.x))
        roots = plane.roots(extrapolate=False)
        roots = sorted(root for root in roots if np.isfinite(root))
        unique = []
        for root in roots:
            region = crossing_region(gate, spline(root), clearance)
            if region == "frame":
                raise ValueError(f"trajectory intersects frame safety margin of {gate['name']}")
            if region == "opening" and (not unique or abs(root - unique[-1]) > 1e-6):
                unique.append(root)
        expected = []
        for other, time in zip(course["hoops"], trajectory.crossing_times_s):
            other_normal = unit_vector(other["normal"])
            # Distinct gates may share a plane: match the physical centre, not
            # just its plane equation. Repeated/reversed events share a centre.
            if abs(abs(other_normal @ normal) - 1) < 1e-6 and np.linalg.norm(np.asarray(other["centre"]) - centre) < 1e-6:
                expected.append(time)
        if len(unique) != len(expected) or not np.allclose(unique, expected, atol=1e-5, rtol=0):
            raise ValueError(f"unexpected intersection through opening of {gate['name']}; revise the crossing order or waypoints")
    for gate, time in zip(course["hoops"], trajectory.crossing_times_s):
        if np.dot(spline(time, 1), gate["normal"]) <= 0:
            raise ValueError(f"wrong crossing direction at {gate['name']}")
