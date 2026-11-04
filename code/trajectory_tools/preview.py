"""Matplotlib geometry and kinematics preview; does not simulate flown motion."""

import numpy as np

from .course import frame_margin, gate_outline, turning_velocities


def plot_trajectory(course, trajectory):
    import matplotlib.pyplot as plt

    times, positions, velocities, accelerations = trajectory.sample(course["limits"]["setpoint_rate_hz"])
    figure = plt.figure(figsize=(16, 10), layout="constrained")
    figure.get_layout_engine().set(wspace=0.15)
    axes = [figure.add_subplot(2, 3, 1, projection="3d")]
    axes += [figure.add_subplot(2, 3, index) for index in (2, 3, 4)]
    pairs = ((0, 1), (0, 2), (1, 2))

    def draw(points, label, **style):
        axes[0].plot(*points.T, label=label, **style)
        for axis, (a, b) in zip(axes[1:], pairs):
            axis.plot(points[:, a], points[:, b], label=label, **style)

    draw(positions, "Planned reference", color="C0")
    crossings = trajectory.spline(trajectory.crossing_times_s)
    draw(crossings, "Chosen crossings", marker="x", linestyle="", color="C4")
    stops, fly_through = [], []
    for turn in course.get("turning_waypoints", []):
        for point, velocity in zip(turn["positions_m"], turning_velocities(turn)):
            if np.any(np.asarray(velocity) != 0):
                fly_through.append(point)
            else:
                stops.append(point)
    if stops:
        draw(np.asarray(stops), "Stop-and-turn waypoints", marker="s", linestyle="", color="C5")
    if fly_through:
        draw(np.asarray(fly_through), "Fly-through waypoints", marker="D", linestyle="", color="C6")
    for index, gate in enumerate(course["hoops"], start=1):
        draw(gate_outline(gate), "_nolegend_", color="0.4", linewidth=1)
        padding = (frame_margin(gate) + course["limits"]["drone_radius_m"]
                   + course["limits"]["clearance_margin_m"])
        envelope = dict(gate)
        if gate.get("shape", "circle") == "rectangle":
            envelope["width_m"] += 2 * padding
            envelope["height_m"] += 2 * padding
        else:
            envelope["radius_m"] += padding
        draw(gate_outline(envelope), "Outer bypass clearance" if index == 1 else "_nolegend_",
             color="C1", linestyle="--", linewidth=1)
        centre = np.asarray(gate["centre"])
        direction = np.asarray(gate["normal"]) * 0.25
        axes[0].quiver(*centre, *direction, color="C3")
        # Repeated crossings share geometry; list their names/times in the
        # summary rather than placing overlapping labels on the same opening.
        if not any(np.allclose(centre, earlier["centre"]) for earlier in course["hoops"][:index - 1]):
            axes[0].text(*centre, f" Gate at crossing {index}", fontsize=8)
    draw(np.asarray([course["start"]]), "Start", marker="o", color="C2")
    draw(np.asarray([course["finish"]]), "Finish", marker="x", color="C3")
    axes[0].legend(loc="upper left", fontsize=8)
    lower = np.asarray(course["limits"]["bounds_min"])
    upper = np.asarray(course["limits"]["bounds_max"])
    for coordinate, label in enumerate(("X [m]", "Y [m]", "Z [m]")):
        (axes[0].set_xlim, axes[0].set_ylim, axes[0].set_zlim)[coordinate](lower[coordinate], upper[coordinate])
        (axes[0].set_xlabel, axes[0].set_ylabel, axes[0].set_zlabel)[coordinate](label)
    axes[0].set_box_aspect(upper - lower)
    for axis, (a, b) in zip(axes[1:], pairs):
        axis.set(xlabel=f"{'XYZ'[a]} [m]", ylabel=f"{'XYZ'[b]} [m]",
                 xlim=(lower[a], upper[a]), ylim=(lower[b], upper[b]))
        axis.set_aspect("equal", adjustable="box")
        axis.grid(True)
    for panel, values, label, bound in (
        (5, velocities, "Speed [m/s]", course["limits"]["maximum_speed_m_s"]),
        (6, accelerations, "Acceleration [m/s2]", course["limits"]["maximum_acceleration_m_s2"]),
    ):
        axis = figure.add_subplot(2, 3, panel)
        axis.plot(times, np.linalg.norm(values, axis=1))
        axis.axhline(bound, color="C3", linestyle="--")
        axis.set(xlabel="Reference time [s]", ylabel=label)
        axis.grid(True)
    figure.suptitle(f"{course['course_name']} - planned path, not simulated flight")
    return figure
