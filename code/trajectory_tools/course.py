"""Read the shared course format and assemble ordered surveyed crossings."""

from copy import deepcopy
import json
from pathlib import Path

import numpy as np

from race_common.geometry import plane_basis

DEFAULT_FRAME_MARGIN_M = 0.10
DEFAULT_FRAME_DEPTH_M = 0.10
CONSERVATIVE_CRUISE_M_S = 0.20
CONSERVATIVE_MAXIMUMS = (("maximum_speed_m_s", .30), ("maximum_acceleration_m_s2", .50),
                         ("maximum_tilt_deg", 10.))
CONSERVATIVE_MARGIN_M = .10
CONSERVATIVE_PHASE_DURATION_S = 3.


def frame_margin(gate):
    """Extra physical extent beyond the measured inner opening, on each side."""
    margin = float(gate.get("frame_margin_m", DEFAULT_FRAME_MARGIN_M))
    if not np.isfinite(margin) or margin < 0:
        raise ValueError("frame_margin_m must be finite and non-negative")
    return margin


def gate_basis(gate):
    if gate.get("shape", "circle") == "rectangle":
        return np.column_stack((gate["width_axis"], gate["height_axis"]))
    return plane_basis(gate["normal"])


def opening_half_size(gate):
    if gate.get("shape", "circle") == "rectangle":
        return np.array([gate["width_m"], gate["height_m"]]) / 2
    return np.array([gate["radius_m"], gate["radius_m"]])


def crossing_region(gate, point, clearance):
    """Classify a drone-centre plane intersection, including its envelope."""
    local = (np.asarray(point) - gate["centre"]) @ gate_basis(gate)
    opening = opening_half_size(gate)
    outside = opening + frame_margin(gate) + clearance
    if gate.get("shape", "circle") == "rectangle":
        if np.any(np.abs(local) > outside):
            return "outside"
        if np.all(np.abs(local) < opening - clearance):
            return "opening"
    else:
        radius = np.linalg.norm(local)
        if radius > outside[0]:
            return "outside"
        if radius < opening[0] - clearance:
            return "opening"
    return "frame"


def read_json(path):
    # PowerShell's UTF-8 output may include a byte-order mark.
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def vector(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must contain three finite numbers")
    return result


def positive(value, name):
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return result


def turning_velocities(turn):
    """World XYZ m/s, with a stationary target for every omitted velocity."""
    return turn.get("velocities_m_s", [[0., 0., 0.] for _ in turn["positions_m"]])


def gate_outline(gate):
    """Draw the physical opening, retaining the rectangle's measured axes."""
    centre = vector(gate["centre"], "gate centre")
    normal = vector(gate["normal"], "gate normal")
    if not np.isclose(np.linalg.norm(normal), 1.0, atol=1e-3):
        raise ValueError("gate normal must be a unit vector")
    positive(gate["radius_m"], "radius_m")
    shape = gate.get("shape", "circle")
    if shape == "rectangle":
        width = positive(gate["width_m"], "width_m")
        height = positive(gate["height_m"], "height_m")
        u = vector(gate["width_axis"], "width_axis")
        v = vector(gate["height_axis"], "height_axis")
        frame = np.column_stack((u, v, normal))
        if not np.allclose(frame.T @ frame, np.eye(3), atol=1e-3):
            raise ValueError("rectangle axes must be orthogonal unit vectors in its plane")
        if gate["radius_m"] > min(width, height) / 2 + 1e-6:
            raise ValueError("rectangle radius exceeds its inscribed circle")
        corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1], [-1, -1]])
        return centre + corners[:, :1] * width / 2 * u + corners[:, 1:] * height / 2 * v
    if shape != "circle":
        raise ValueError(f"unknown gate shape: {shape}")
    angles = np.linspace(0, 2 * np.pi, 101)
    basis = plane_basis(normal)
    return centre + gate["radius_m"] * (
        np.cos(angles)[:, None] * basis[:, 0] + np.sin(angles)[:, None] * basis[:, 1])


def validate_course(course):
    """Check the documented interface, without enabling or approving flight."""
    for key in ("course_name", "uri_environment_variable"):
        if not isinstance(course[key], str) or not course[key].strip():
            raise ValueError(f"{key} must be a nonempty string")
    if not isinstance(course["enabled_for_flight"], bool):
        raise ValueError("enabled_for_flight must be a JSON boolean")
    if not np.isfinite(float(course["yaw_deg"])):
        raise ValueError("yaw_deg must be finite")
    limits = course["limits"]
    for key in ("setpoint_rate_hz", "target_speed_m_s", "maximum_speed_m_s",
                "maximum_acceleration_m_s2", "maximum_tilt_deg", "maximum_tracking_error_m",
                "telemetry_timeout_s", "drone_radius_m", "clearance_margin_m",
                "pre_post_distance_m", "takeoff_duration_s", "landing_duration_s"):
        positive(limits[key], key)
    if limits["target_speed_m_s"] > limits["maximum_speed_m_s"]:
        raise ValueError("target speed exceeds maximum speed")
    if limits["maximum_tilt_deg"] >= 90:
        raise ValueError("maximum_tilt_deg must be below 90 degrees")
    lower = vector(limits["bounds_min"], "bounds_min")
    upper = vector(limits["bounds_max"], "bounds_max")
    if np.any(lower >= upper):
        raise ValueError("every lower bound must be below its upper bound")
    gates = course["hoops"]
    if not gates or len({gate["name"] for gate in gates}) != len(gates):
        raise ValueError("at least one crossing and unique crossing names are required")
    points = [vector(course[key], key) for key in ("start", "finish")]
    version = course.get("trajectory_version")
    has_choices = ("turning_waypoints" in course or "stop_at_approaches" in course or
                   any("speed_m_s" in gate or "crossing_offset_m" in gate for gate in gates))
    if "trajectory_version" in course and (type(version) is not int or version != 1):
        raise ValueError("Unsupported trajectory_version; use version 1")
    if has_choices and version != 1:
        raise ValueError("Per-crossing choices require trajectory_version=1")
    if "stop_at_approaches" in course and type(course["stop_at_approaches"]) is not bool:
        raise ValueError("stop_at_approaches must be a boolean")
    profile = course.get("planning_profile", "custom")
    if profile not in ("conservative", "custom"):
        raise ValueError("planning_profile must be conservative or custom")
    if profile == "conservative":
        if course.get("stop_at_approaches") is not True:
            raise ValueError("Conservative profiles must stop at gate approaches")
        for name, ceiling in CONSERVATIVE_MAXIMUMS:
            if limits[name] > ceiling:
                raise ValueError(f"Conservative {name} exceeds {ceiling:g}")
        if limits["clearance_margin_m"] < CONSERVATIVE_MARGIN_M:
            raise ValueError("Conservative aperture margin must be at least 0.10 m")
        if min(limits["takeoff_duration_s"], limits["landing_duration_s"]) < CONSERVATIVE_PHASE_DURATION_S:
            raise ValueError("Conservative takeoff and landing durations must be at least 3 s")
    if "frame_depth_m" in course:
        positive(course["frame_depth_m"], "frame_depth_m")
    for gate in gates:
        if not isinstance(gate["name"], str) or not gate["name"].strip():
            raise ValueError("crossing names must be nonempty strings")
        gate_outline(gate)
        frame_margin(gate)
        if "frame_depth_m" in gate:
            positive(gate["frame_depth_m"], "frame_depth_m")
        if gate["radius_m"] <= limits["drone_radius_m"] + limits["clearance_margin_m"]:
            raise ValueError(f"gate {gate['name']} has insufficient aperture clearance")
        points.append(vector(gate["centre"], "gate centre"))
        if "speed_m_s" in gate:
            speed = positive(gate["speed_m_s"], "gate speed_m_s")
            if isinstance(gate["speed_m_s"], bool) or speed > limits["maximum_speed_m_s"]:
                raise ValueError("Gate speed must stay within the shared maximum speed")
        if "crossing_offset_m" in gate:
            offset = vector(gate["crossing_offset_m"], "crossing_offset_m")
            normal = vector(gate["normal"], "gate normal")
            if abs(offset @ normal) > 1e-6:
                raise ValueError("Crossing offset must lie in the gate plane")
            required = limits["drone_radius_m"] + limits["clearance_margin_m"]
            if np.linalg.norm(offset) + required >= gate["radius_m"]:
                raise ValueError("Crossing offset consumes the aperture clearance")
            points.append(vector(gate["centre"], "gate centre") + offset)
    slots = set()
    turns = course.get("turning_waypoints", [])
    if not isinstance(turns, list):
        raise ValueError("turning_waypoints must be a list of after_crossing/positions_m records")
    for turn in turns:
        if (not isinstance(turn, dict) or not {"after_crossing", "positions_m"}.issubset(turn)
                or set(turn) - {"after_crossing", "positions_m", "velocities_m_s"}):
            raise ValueError("Each turning record needs after_crossing, positions_m and optional velocities_m_s")
        slot = turn["after_crossing"]
        if type(slot) is not int or not 0 <= slot <= len(gates) or slot in slots:
            raise ValueError("Turning slot must be unique and between 0 and the crossing count")
        slots.add(slot)
        if not isinstance(turn["positions_m"], list) or not turn["positions_m"]:
            raise ValueError("Supply at least one XYZ turning position per slot")
        points.extend(vector(point, "turning waypoint") for point in turn["positions_m"])
        velocities = turning_velocities(turn)
        if not isinstance(velocities, list) or len(velocities) != len(turn["positions_m"]):
            raise ValueError("Supply one XYZ target velocity per turning position")
        for value in velocities:
            velocity = vector(value, "turning target velocity")
            if np.linalg.norm(velocity) > limits["maximum_speed_m_s"] + 1e-12:
                raise ValueError("Turning target velocity exceeds the shared maximum speed")
            if profile == "conservative" and np.any(velocity != 0):
                raise ValueError("Fly-through turning velocities require conservative=False; default waypoints stop")
    if np.any(np.asarray(points) < lower) or np.any(np.asarray(points) > upper):
        raise ValueError("start, finish or gate centre lies outside the flight volume")


def configure_crossings(course, *, gate_speeds=None, crossing_offsets=None, turning_waypoints=None):
    """Apply notebook choices to a private copy, retaining physical gate geometry.

    A physical gate ID applies to every passage; an exact crossing name overrides
    that choice. Turning slot 0 precedes the first crossing, slot N follows it.
    """
    course = deepcopy(course)
    course["trajectory_version"] = 1
    names = {gate["name"] for gate in course["hoops"]}
    names.update(gate.get("gate_id", gate["name"]) for gate in course["hoops"])
    for field, choices in (("speed_m_s", gate_speeds), ("crossing_offset_m", crossing_offsets)):
        if choices is None:
            continue
        if not isinstance(choices, dict) or set(choices) - names:
            raise ValueError(f"{field}: use physical gate IDs or exact crossing names from the summary")
        for gate in course["hoops"]:
            gate.pop(field, None)
            key = gate["name"] if gate["name"] in choices else gate.get("gate_id", gate["name"])
            if key in choices:
                value = choices[key]
                gate[field] = vector(value, field).tolist() if field == "crossing_offset_m" else value
    if turning_waypoints is not None:
        if not isinstance(turning_waypoints, dict):
            raise ValueError("Turning waypoints use {after_crossing: [XYZ, ...]}")
        turns = []
        for slot, waypoints in turning_waypoints.items():
            if type(slot) is not int:
                raise ValueError("Turning slots are integer crossing counts, starting at 0")
            if not isinstance(waypoints, (list, tuple, np.ndarray)):
                raise ValueError("Turning positions must be a list of XYZ vectors")
            positions, velocities = [], []
            has_velocity = False
            for waypoint in waypoints:
                if isinstance(waypoint, dict):
                    if "position" not in waypoint or set(waypoint) - {"position", "velocity"}:
                        raise ValueError("A turning waypoint needs position and optional velocity")
                    position = waypoint["position"]
                    velocity = waypoint.get("velocity", [0, 0, 0])
                    has_velocity = has_velocity or "velocity" in waypoint
                else:
                    position, velocity = waypoint, [0, 0, 0]
                positions.append(vector(position, "turning waypoint").tolist())
                velocities.append(vector(velocity, "turning target velocity").tolist())
            record = dict(after_crossing=slot, positions_m=positions)
            if has_velocity:
                record["velocities_m_s"] = velocities
            turns.append(record)
        course["turning_waypoints"] = sorted(turns, key=lambda turn: turn["after_crossing"])
    validate_course(course)
    return course


def construct_course(template, surveys=(), crossings=(), laps=1, frame_margin_m=None):
    """Repeat a crossing sequence; 'name:-' reverses the surveyed normal."""
    if isinstance(laps, bool) or not isinstance(laps, int) or laps < 1:
        raise ValueError("laps must be a positive integer")
    course = deepcopy(template)
    if surveys:
        gates = {}
        for gate in surveys:
            gate_outline(gate)
            if gate["name"] in gates:
                raise ValueError(f"duplicate survey name: {gate['name']}")
            gates[gate["name"]] = gate
        if not crossings:
            raise ValueError("supply --crossing NAME:+ or NAME:- for surveyed gates")
        sequence = []
        for specification in crossings:
            name, separator, sign = specification.rpartition(":")
            if not separator or sign not in ("+", "-") or name not in gates:
                raise ValueError(f"unknown gate or invalid crossing: {specification}")
            # Only runner/plotting geometry goes into the course. Original survey
            # samples and provenance stay in the unmodified measurement file.
            fields = ("name", "centre", "normal", "radius_m", "shape", "width_m",
                      "height_m", "width_axis", "height_axis", "frame_margin_m", "frame_depth_m")
            gate = {key: deepcopy(gates[name][key]) for key in fields if key in gates[name]}
            gate["normal"] = (vector(gate["normal"], "normal") * (1 if sign == "+" else -1)).tolist()
            sequence.append(gate)
    else:
        if crossings:
            raise ValueError("--crossing requires --gate-survey")
        sequence = course["hoops"]
    course["hoops"] = []
    for lap in range(1, laps + 1):
        for index, gate in enumerate(sequence, start=1):
            event = deepcopy(gate)
            event["gate_id"] = gate.get("gate_id", gate["name"])
            event["name"] = f"lap-{lap}-{index}-{gate['name']}"
            event["frame_margin_m"] = frame_margin(gate) if frame_margin_m is None else frame_margin_m
            course["hoops"].append(event)
    course["enabled_for_flight"] = False
    validate_course(course)
    return course
