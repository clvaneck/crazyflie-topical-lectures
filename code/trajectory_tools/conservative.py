"""Slow defaults and bounded routing between explicitly scheduled crossings."""

from copy import deepcopy
from itertools import product

import numpy as np
from scipy.sparse.csgraph import shortest_path

from race_common.geometry import unit_vector
from .course import (
    CONSERVATIVE_CRUISE_M_S, CONSERVATIVE_MARGIN_M, CONSERVATIVE_MAXIMUMS,
    CONSERVATIVE_PHASE_DURATION_S, DEFAULT_FRAME_DEPTH_M,
    frame_margin, gate_basis, opening_half_size, validate_course,
)


def slow_defaults(template):
    """Tighten motion limits, retaining the site's bounds and airframe envelope."""
    course = deepcopy(template)
    limits = course["limits"]
    limits["target_speed_m_s"] = min(limits["target_speed_m_s"], CONSERVATIVE_CRUISE_M_S)
    for name, ceiling in CONSERVATIVE_MAXIMUMS:
        limits[name] = min(limits[name], ceiling)
    limits["clearance_margin_m"] = max(limits["clearance_margin_m"], CONSERVATIVE_MARGIN_M)
    for name in ("takeoff_duration_s", "landing_duration_s"):
        limits[name] = max(limits[name], CONSERVATIVE_PHASE_DURATION_S)
    course["planning_profile"] = "conservative"
    course["trajectory_version"] = 1
    course.setdefault("frame_depth_m", DEFAULT_FRAME_DEPTH_M)
    course["stop_at_approaches"] = True
    course["enabled_for_flight"] = False
    return course


def _gate_boxes(course):
    """Treat openings as blocked during transitions: only scheduled passes use them.

    The routing envelope includes the drone, margin and frame. The shared setup
    supplies frame depth; 0.10 m is an explicit planning assumption to check.
    """
    clearance = course["limits"]["drone_radius_m"] + course["limits"]["clearance_margin_m"]
    boxes = []
    for gate in course["hoops"]:
        normal = unit_vector(gate["normal"])
        width_axis = gate_basis(gate)[:, 0]
        width_axis = unit_vector(width_axis - normal * (normal @ width_axis))
        basis = np.column_stack((width_axis, np.cross(normal, width_axis), normal))
        depth = float(gate.get("frame_depth_m", course.get("frame_depth_m", DEFAULT_FRAME_DEPTH_M)))
        if not np.isfinite(depth) or depth <= 0:
            raise ValueError("frame_depth_m must be finite and positive")
        half_size = np.r_[opening_half_size(gate) + frame_margin(gate) + clearance,
                          depth / 2 + clearance]
        box = (np.asarray(gate["centre"], dtype=float), basis, half_size)
        # Repeated passages through one physical opening share one obstacle.
        if not any(np.array_equal(box[0], old[0]) and np.array_equal(box[2], old[2])
                   and np.allclose(np.abs(np.diag(box[1].T @ old[1])), 1., atol=1e-12, rtol=0)
                   for old in boxes):
            boxes.append(box)
    return boxes


def _clear_segment(start, finish, boxes):
    """Slab intersection in each gate's orthonormal frame, including tangencies."""
    for centre, basis, half_size in boxes:
        position = (start - centre) @ basis
        direction = (finish - start) @ basis
        low, high = 0., 1.
        for coordinate in range(3):
            if abs(direction[coordinate]) < 1e-12:
                if abs(position[coordinate]) > half_size[coordinate]:
                    break
            else:
                endpoints = ((-half_size[coordinate] - position[coordinate]) / direction[coordinate],
                             (half_size[coordinate] - position[coordinate]) / direction[coordinate])
                low = max(low, min(endpoints))
                high = min(high, max(endpoints))
                if low > high:
                    break
        else:
            return False
    return True


def route_transitions(course):
    """Insert stop waypoints around finite gate envelopes, or reject the layout.

    A visibility graph over envelope corners/edge midpoints gives a deterministic
    conservative route. It covers modelled gates, not unmeasured room obstacles.
    """
    course = deepcopy(course)
    validate_course(course)
    boxes = _gate_boxes(course)
    lower = np.asarray(course["limits"]["bounds_min"])
    upper = np.asarray(course["limits"]["bounds_max"])
    distance = course["limits"]["pre_post_distance_m"]
    user_turns = {turn["after_crossing"]: turn["positions_m"]
                  for turn in course.get("turning_waypoints", [])}
    transitions = []
    previous = np.asarray(course["start"], dtype=float)
    for index, gate in enumerate(course["hoops"]):
        crossing = np.asarray(gate["centre"]) + np.asarray(gate.get("crossing_offset_m", [0, 0, 0]))
        normal = unit_vector(gate["normal"])
        entry, exit_point = crossing - distance * normal, crossing + distance * normal
        other_boxes = [box for box in boxes if not (
            np.allclose(box[0], gate["centre"]) and abs(box[1][:, 2] @ normal) > 1 - 1e-6)]
        if not _clear_segment(entry, exit_point, other_boxes):
            raise ValueError("Conservative crossing corridor overlaps another gate envelope; revise the layout")
        transitions.append([previous, *map(np.asarray, user_turns.get(index, [])), entry])
        previous = exit_point
    transitions.append([previous, *map(np.asarray, user_turns.get(len(course["hoops"]), [])),
                        np.asarray(course["finish"], dtype=float)])
    for points in transitions:
        for point in points:
            if np.any(point < lower) or np.any(point > upper) or not _clear_segment(point, point, boxes):
                raise ValueError("Conservative transition endpoint is outside bounds or inside a gate envelope; check setup/approach distance")
    pairs = [(start, finish) for points in transitions for start, finish in zip(points, points[1:])]
    # Automatic detours stay within the altitude range already requested by
    # endpoints, gate approaches and explicit waypoints; no surprise dives/climbs.
    altitudes = [point[2] for points in transitions for point in points]
    minimum_altitude, maximum_altitude = min(altitudes), max(altitudes)
    if all(_clear_segment(start, finish, boxes) for start, finish in pairs):
        return course

    nodes = []
    node_indexes = {}

    def node_index(point):
        key = tuple(float(value) for value in point)
        if key not in node_indexes:
            node_indexes[key] = len(nodes)
            nodes.append(np.asarray(point, dtype=float))
        return node_indexes[key]

    for points in transitions:
        for point in points:
            node_index(point)
    for centre, basis, half_size in boxes:
        # Five centimetres beyond the already inflated box avoid edge contact.
        for signs in product((-1, 0, 1), repeat=3):
            if signs == (0, 0, 0):
                continue
            point = centre + basis @ (np.asarray(signs) * (half_size + .05))
            if (np.all(point >= lower) and np.all(point <= upper)
                    and minimum_altitude - 1e-10 <= point[2] <= maximum_altitude + 1e-10
                    and _clear_segment(point, point, boxes)):
                node_index(point)
    graph = np.zeros((len(nodes), len(nodes)))
    for index, start in enumerate(nodes):
        for other in range(index + 1, len(nodes)):
            if _clear_segment(start, nodes[other], boxes):
                graph[index, other] = graph[other, index] = np.linalg.norm(nodes[other] - start)
    distances, predecessors = shortest_path(graph, directed=False, return_predecessors=True)
    routed = []
    for slot, points in enumerate(transitions):
        stops = []
        for start, finish in zip(points, points[1:]):
            source, target = node_index(start), node_index(finish)
            if not np.isfinite(distances[source, target]):
                raise ValueError("No conservative gate-bypass route within the shared volume; revise the layout or supply checked waypoints")
            path = [target]
            while path[-1] != source:
                path.append(int(predecessors[source, path[-1]]))
            stops.extend(nodes[index].tolist() for index in reversed(path[:-1]))
        # Each route includes its final approach/finish; that point is already
        # a trajectory knot. Interior user stops and automatic detours remain.
        if stops[:-1]:
            routed.append(dict(after_crossing=slot, positions_m=stops[:-1]))
    course["turning_waypoints"] = routed
    return course


def validate_transition_route(course):
    """Require the exported stops to already contain a clear conservative route."""
    checked = route_transitions(course)
    if checked.get("turning_waypoints", []) != course.get("turning_waypoints", []):
        raise ValueError("Conservative route needs additional bypass stops; replan before export or flight")
