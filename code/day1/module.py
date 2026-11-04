"""Drone simulation module for Nikhef topical lectures.

This module provides a 2D drone simulation with physics, visualization, and
noise generation for wind effects.

Classes:
    Drone: Physics simulation of a 2D drone with motors and state tracking.
    Plotter: Real-time visualization with matplotlib animation.
    Perlin: Perlin noise generator for realistic wind simulation.

Authors: Andreas Freise, Bas Swinkels
Date: 30.10.2026
"""

import numpy as np
import time
import random
from types import SimpleNamespace
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import matplotlib.transforms as trafo
from matplotlib.patches import Polygon
import warnings


# Shared simulation step. Used as the default deltaT in both Drone and
# PIDController so they cannot silently disagree on the integration
# interval.
DELTA_T = 1.0 / 60.0


class Drone:
    # Crazyflie 2.1 reference parameters (SI units).
    # Source: MATEC Web of Conferences 417, 04007 (2025), Table 1.
    # The 2D model projects the X-frame quadcopter onto a single plane, so
    # each of our two motors stands in for the two real motors on that side
    # of the airframe. The max thrust per *2D* motor is therefore twice the
    # per-real-motor max thrust derived from the paper's thrust coefficient
    # k_t and the PWM-to-angular-velocity mapping.
    CF_M           = 0.032            # kg, drone mass
    CF_IXX         = 16.57e-6         # kg*m^2, roll inertia (2D slice; the paper lists J_yy separately at 16.55)
    CF_L_ARM       = 0.0325           # m, perpendicular distance from CG to motor pair
    CF_G           = 9.8              # m/s^2
    CF_F_MAX_MOTOR = 0.296            # N, max thrust per 2D motor (= 2 * 0.148)

    def __init__(self, test=False, name=None, wind=False,
                 flight_range=(-3.0, -3.0, 3.0, 3.0),
                 thrust_factor=1.0,
                 drag_t=4.0, drag_r=5.0):
        """Initialize a drone simulation object.

        Physical parameters are taken from the Crazyflie 2.1 reference values
        above; SI units throughout (m, kg, s, N, rad).

        Args:
            test: If True, uses a 'perfect' drone for initial testing only.
            name: Student name used to set personalized drone parameters.
            wind: If True, adds Perlin noise-based wind disturbances.
            flight_range: Boundary limits ``(min_y, min_z, max_y, max_z)`` in
                metres. Default is a 6x6 m box (-3 m to +3 m on each axis).
            thrust_factor: Multiplier on the Crazyflie's per-motor max thrust.
                ``1.0`` matches the real platform (thrust-to-weight ~ 1.9);
                raise it (e.g. 2.0 or 3.0) for a snappier, less realistic
                simulator. Affects only ``V_left_amp`` / ``V_right_amp``,
                not the V signal range itself.
            drag_t: Linear (translational) damping rate with units of
                1/s. Enters ``dx`` directly as a deceleration:
                ``y_ddot += -drag_t * dy`` (and similarly for z), i.e. the
                damping per unit mass rather than a Newtonian drag force.
                Default ``4.0`` gives a forward-flight terminal velocity
                of roughly 3 m/s for a Crazyflie at full tilt, matching
                the published spec. Set to 0 to remove drag entirely (the
                model behaves like a frictionless point mass).
            drag_r: Angular damping rate with units of 1/s. Enters ``dx``
                directly as an angular deceleration:
                ``phi_ddot += -drag_r * dphi``. Default ``5.0`` limits the
                roll rate to a few turns per second at full motor
                differential, which is plausible for a small drone.
        """
        self.test = test
        self.wind = wind
        self.name = name

        self.deltaT = DELTA_T   # s, simulation step

        # Physical body parameters (SI)
        self.M     = self.CF_M
        self.Ixx   = self.CF_IXX
        self.L_arm = self.CF_L_ARM
        self.g     = self.CF_G

        # Motor model: F_motor = V * amp + offset, with V in [0, 1] (throttle).
        # amp is the per-motor max thrust in newtons; thrust_factor scales it.
        self.F = 0.0
        self.tau = 0.0
        self.F_left = 0.0
        self.F_right = 0.0
        self.V_left_amp  = self.CF_F_MAX_MOTOR * thrust_factor
        self.V_right_amp = self.CF_F_MAX_MOTOR * thrust_factor
        self.V_left_offset  = 0.0   # N, small per-student bias
        self.V_right_offset = 0.0
        # Single-direction motors: V is clipped to [0, 1].
        self.V_left_clip_m = 0.0
        self.V_left_clip_p = 1.0
        self.V_right_clip_m = 0.0
        self.V_right_clip_p = 1.0
        self.range = flight_range
        # Drag coefficients (1/s). See class docstring for definitions.
        self.drag_t = drag_t
        self.drag_r = drag_r
        self.perlin = Perlin()

        if self.test:
            warnings.warn("Running drone in test mode. Don't use this for system identification", stacklevel=2)
        else:
            if not self.name:
                warnings.warn("You need to provide your full name in the call to `drone()' for the project",stacklevel=2)
            else:
                # per-student variation; magnitudes scaled to SI units:
                #   force offsets ~ 2% of hover thrust per motor (M*g/2 ~ 0.16 N),
                #   upper-clip variation ~ 5% of nominal throttle.
                key = name_to_int(self.name)
                self.V_left_offset  += bits_to_float(key,  8, 11, 0.003)
                self.V_right_offset += bits_to_float(key, 12, 15, 0.003)
                self.V_left_clip_p  += bits_to_float(key, 20, 23, 0.05)
                self.V_right_clip_p += bits_to_float(key, 28, 31, 0.05)

        self.reset()

    def reset(self):
        """Reset drone state to initial conditions.

        Resets time, forces, position, velocity, and target tracking to their
        initial values. Wind offsets are randomized for varied simulation runs.
        """
        self.stop = False
        self.t = 0

        self.a_wind_z = 0
        self.a_wind_y = 0
        self.perlin_offset1 = random.random()*1000.0
        self.perlin_offset2 = random.random()*1000.0

        # state vector for odeint: [y, z, phi, dy, dz, dphi]. Allocated once
        # and written in place by update() so that the ``pos`` / ``vel``
        # views below stay valid for the lifetime of the Drone.
        if not hasattr(self, 'solve_state'):
            self.solve_state = np.zeros(6)
        else:
            self.solve_state[:] = 0.0

        # Stable views into solve_state.
        self.pos = self.solve_state[0:3]
        self.vel = self.solve_state[3:6]
        # current forces: [F, tau, F_left, F_right]
        self.forces = np.zeros(4)

        # variables for targets
        self.targets = None
        self.target_idx = 0
        self.num_targets = 0


    def set_targets(self, _targets):
        """Set a list of targets for the drone to reach.

        Args:
            _targets: NumPy array of shape (N, 3) where each row contains
                (y, z, radius) for N targets.

        Raises:
            ValueError: If targets is not a NumPy array of shape (N, 3).
        """
        if not isinstance(_targets, np.ndarray):
            raise ValueError("targets must be a NumPy array of shape (N, 3)")
        if _targets.ndim != 2 or _targets.shape[1] != 3:
            raise ValueError("targets must be a NumPy array of shape (N, 3)")
        self.targets = _targets
        self.target_idx = 0
        self.num_targets = self.targets.shape[0]

    def dx(self, t, x, F, tau):
        """Compute state derivatives for ODE integration.

        State update function for use with scipy.integrate.odeint with
        tfirst=True. Adds wind disturbance accelerations from
        ``self.a_wind_y`` and ``self.a_wind_z`` (both in m/s^2; they
        enter the EOM as accelerations, not as Newtonian forces).

        Args:
            t: Current time (unused but required by odeint).
            x: State vector [y, z, phi, y', z', phi'] with positions and velocities.
            F: Total thrust along the body z-axis (rotated into the world
                frame inside this function via ``cos(phi)`` / ``-sin(phi)``).
            tau: Angular torque from differential motor thrust.

        Returns:
            State derivatives [y', z', phi', y'', z'', phi''].

        References:
            https://www.youtube.com/watch?v=lAVYDUeqdW4&t=187s
            https://www.youtube.com/watch?v=dWwhLP0Iwvg
        """

        F_over_M = F / self.M

        # Linear and angular damping, proportional to velocity. drag_t and
        # drag_r have units 1/s and enter as decelerations (already
        # per-unit-mass; not Newtonian drag forces). Without these the
        # drone reaches unrealistic top speeds in a small room: the
        # original Crazyflie spec settles around 3 m/s in forward flight,
        # which corresponds to ``drag_t`` of order 4 1/s.
        return np.array([x[3], x[4], x[5],
            -np.sin(x[2]) * F_over_M + self.a_wind_y - self.drag_t * x[3],
             np.cos(x[2]) * F_over_M - self.g + self.a_wind_z - self.drag_t * x[4],
             tau / self.Ixx                          - self.drag_r * x[5],
        ])

    def set_V(self, V_left, V_right):
        """Set motor throttle commands and compute resulting forces.

        Applies throttle clipping, amplitude scaling, and offset adjustments
        based on drone-specific parameters, then computes total force and torque.
        The ``V_`` prefix is kept for backward compatibility with the older
        voltage-based motor model; values are now throttle fractions in
        ``[0, 1]``.

        Args:
            V_left: Throttle command for left motor (clipped to ``[0, 1]``).
            V_right: Throttle command for right motor (clipped to ``[0, 1]``).
        """
        V_left  = np.clip(V_left,  self.V_left_clip_m,  self.V_left_clip_p)
        V_right = np.clip(V_right, self.V_right_clip_m, self.V_right_clip_p)

        # Single-direction motors cannot produce negative thrust, so we clamp
        # at zero after the per-student offset is applied. Otherwise a name
        # with a strongly negative offset would let the motors "pull" down
        # at idle, which is unphysical.
        self.F_left  = max(0.0, V_left  * self.V_left_amp  + self.V_left_offset)
        self.F_right = max(0.0, V_right * self.V_right_amp + self.V_right_offset)

        self.F = self.F_right + self.F_left
        self.tau = (self.F_right - self.F_left) * self.L_arm

    def set_state(self):
        """Apply boundary constraints and update observable state.

        Clips position to flight range boundaries, normalizes angle to [-pi, pi],
        and checks whether the next target has been reached. The next-target
        check uses Euclidean distance so it agrees with the red circle drawn
        in the plotter.
        """
        # Push the drone a small fraction of the range width *inward* when it
        # has crossed a boundary. The fraction-of-width form works regardless
        # of whether the range straddles zero (e.g. ``flight_range=(1, 1, 5, 5)``
        # for a corner-of-the-room demo).
        nudge_y = 0.001 * (self.range[2] - self.range[0])
        nudge_z = 0.001 * (self.range[3] - self.range[1])
        for axis, vel_idx, low, high, nudge in (
            (0, 3, self.range[0], self.range[2], nudge_y),
            (1, 4, self.range[1], self.range[3], nudge_z),
        ):
            if self.solve_state[axis] < low:
                self.solve_state[axis] = low + nudge
                self.solve_state[vel_idx] = 0.0
            elif self.solve_state[axis] > high:
                self.solve_state[axis] = high - nudge
                self.solve_state[vel_idx] = 0.0

        # Keep phi in [-pi, +pi].
        self.solve_state[2] = (self.solve_state[2] + np.pi) % (2 * np.pi) - np.pi

        # ``pos`` and ``vel`` are views; ``forces`` is filled in place
        # to keep all references stable across update() calls.
        self.forces[0] = self.F
        self.forces[1] = self.tau
        self.forces[2] = self.F_left
        self.forces[3] = self.F_right

        # Next-target check: Euclidean distance, matching the red circle drawn
        # by ``Plotter`` and ``render_replay``.
        if self.targets is not None and self.target_idx < self.num_targets:
            tgt = self.targets[self.target_idx]
            dy = self.pos[0] - tgt[0]
            dz = self.pos[1] - tgt[1]
            if dy * dy + dz * dz < tgt[2] * tgt[2]:
                self.target_idx += 1

    def update(self):
        """Advance simulation by one time step.

        Updates wind forces (if enabled), integrates equations of motion,
        applies state constraints, and checks target completion.

        Returns:
            SimpleNamespace with named attributes: t, y, z, phi, dy, dz, dphi,
            F, tau, F_left, F_right, target_idx.
        """
        self.t += self.deltaT
        if self.wind:
            self.a_wind_z = 0.1 * self.perlin.sum(self.t+self.perlin_offset1, 0.2, 1, 1.05, 1.3)
            self.a_wind_y = 2   * self.perlin.sum(self.t+self.perlin_offset2, 0.2, 1, 1.05, 1.3)

        # In-place write keeps ``pos`` / ``vel`` views valid.
        self.solve_state[:] = odeint(
            self.dx, self.solve_state, [0, self.deltaT],
            tfirst=True, args=(self.F, self.tau),
            atol=1e-6, rtol=1e-6,
        )[1]
        self.set_state()
        return SimpleNamespace(
            t=self.t,
            y=self.pos[0], z=self.pos[1], phi=self.pos[2],
            dy=self.vel[0], dz=self.vel[1], dphi=self.vel[2],
            F=self.forces[0], tau=self.forces[1],
            F_left=self.forces[2], F_right=self.forces[3],
            target_idx=self.target_idx
        )

    # Visual width of the rendered drone in metres. Independent of the
    # physical L_arm used by the dynamics: a real Crazyflie at ~5 cm would
    # be invisible in the default 6 m field, so we exaggerate the icon
    # for legibility.
    DISPLAY_WIDTH = 0.5

    def shape(self):
        """Create drone shape as a matplotlib Polygon.

        The returned polygon is `DISPLAY_WIDTH` metres across. It is a
        visual icon only; the dynamics use the smaller physical `L_arm`
        for torque calculations.
        """
        # right part of drone, in the original "cm-scale" coordinates 0..30
        right_part = np.array([
            [3, 3],[3, 1],[19, 1],[19, 3],[10, 2],[10, 5],[19, 4],[21, 4],
            [30, 5],[30, 2],[21, 3],[21, 1],[22, 1],[22, -3],[18, -3],
            [18, -1],[3, -1],[3, -3],])
        # the design above spans x = -30..30 (60 units); rescale so the
        # rendered drone is DISPLAY_WIDTH metres across.
        right_part = right_part * (self.DISPLAY_WIDTH / 60.0)
        left_part = np.c_[-right_part[::-1, 0], right_part[::-1, 1]]
        drone_shape = np.concatenate((left_part, right_part))
        return Polygon(drone_shape, closed=True, zorder=2, facecolor="gray")

def state_to_array(state):
    """Convert SimpleNamespace state to numpy array for storage."""
    return np.array([state.t, state.y, state.z, state.phi,
                     state.dy, state.dz, state.dphi,
                     state.F, state.tau, state.F_left, state.F_right,
                     state.target_idx])


class PIDController:
    """Discrete PID controller with a real integrator and anti-windup clamping.

    Sign convention matches the original notebook controllers: ``err`` is
    ``value - setpoint`` (positive when above target). The sign of each gain
    determines how the actuation responds. For an altitude controller, ``Kp``
    and ``Kd`` are typically negative (positive error means "above target", so
    we want less thrust); for a horizontal-position controller they are
    typically positive (the output is fed forward as a desired tilt).

    Output:

        fb = Kp * err  +  Kd * value_dot  +  Ki * integral_of_err_over_time

    The integrator state is clamped to ``[-int_clamp, +int_clamp]`` so a
    sustained actuator saturation cannot let it grow without bound.
    """

    def __init__(self, Kp, Ki, Kd, deltaT=DELTA_T, int_clamp=10):
        """Build a controller with fixed gains and a fixed time step.

        Args:
            Kp: Proportional gain.
            Ki: Integral gain (per unit of accumulated error*time).
            Kd: Derivative gain (applied directly to ``value_dot``).
            deltaT: Simulation step in seconds. Must match the drone's
                ``deltaT`` so the integrator accumulates over the right
                interval.
            int_clamp: Hard cap on the magnitude of the integrator state.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.deltaT = deltaT
        self.int_clamp = int_clamp
        self.reset()

    def reset(self):
        """Zero the integrator state. Call between independent simulation runs."""
        self.integral = 0.0

    def __call__(self, value, value_dot, setpoint, setpoint_dot=0.0):
        """Compute one PID step and return the feedback signal.

        Args:
            value: Current measured value (e.g. position, angle).
            value_dot: Current measured derivative (e.g. velocity).
            setpoint: Desired value.
            setpoint_dot: Desired derivative (default 0). Non-zero for
                trajectory tracking, where the setpoint itself is moving
                and we want to penalise *velocity error*, not raw
                velocity.

        Returns:
            ``Kp*err + Kd*err_dot + Ki*integral`` with
            ``err = value - setpoint`` and ``err_dot = value_dot - setpoint_dot``.
        """
        err = value - setpoint
        err_dot = value_dot - setpoint_dot
        self.integral = float(np.clip(
            self.integral + err * self.deltaT,
            -self.int_clamp, self.int_clamp,
        ))
        return self.Kp * err + self.Kd * err_dot + self.Ki * self.integral


# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
class Plotter:
    """Real-time drone visualization with matplotlib animation.

    Provides a two-panel display: a full-field view showing the drone's
    position within the flight range, and a zoomed view for detailed
    orientation. Supports target visualization and FPS monitoring.
    """

    def __init__(self, _drone, target_fps=None, text_update_rate=10):
        """Initialize the plotter with a drone instance.

        Args:
            _drone: Drone instance to visualize.
            target_fps: Target frames per second for animation (None for unlimited).
            text_update_rate: Hz at which the numeric overlay (F / tau /
                position / velocity) refreshes. The drone graphic itself
                still updates every render frame; only the text strings
                are throttled. Set to ``None`` or a very large value to
                refresh every frame. Default 10 Hz.
        """
        self.target_fps = target_fps
        self.text_update_rate = text_update_rate
        self._text_update_interval = (
            1.0 / text_update_rate if text_update_rate else 0.0
        )
        self._last_text_update = 0.0

        # shapes for plotting
        self.drone = _drone
        self.drone1 = self.drone.shape()
        self.drone2 = self.drone.shape()

        self.num_targets = 0

        # init figure and axes
        self.fig = plt.figure(figsize=(4.5,8))
        heights = [1, 1]
        self.gs1 = self.fig.add_gridspec(nrows=2, ncols=1,height_ratios=heights)

        # full field drone view
        self.ax1 = self.fig.add_subplot(self.gs1[0,0])
        # zoomed-in drone view
        self.ax2 = self.fig.add_subplot(self.gs1[1,0])

        # Text positions are in axes-relative coordinates (0..1), so they do
        # not need re-tuning when the data range changes (SI vs old units).
        self.text0 = self.ax1.text(0.55, 0.02, "", transform=self.ax1.transAxes)  # user string
        self.text1 = self.ax1.text(0.02, 0.02, "", transform=self.ax1.transAxes,
                                   family='monospace')  # fps
        self.text2 = self.ax2.text(0.02, 0.02, "", transform=self.ax2.transAxes,
                                   family='monospace')  # F, tau
        self.text3 = self.ax2.text(0.02, 0.09, "", transform=self.ax2.transAxes,
                                   family='monospace')  # forces left/right
        self.text4 = self.ax2.text(0.02, 0.16, "", transform=self.ax2.transAxes,
                                   family='monospace')  # position
        self.text5 = self.ax2.text(0.02, 0.23, "", transform=self.ax2.transAxes,
                                   family='monospace')  # velocity

        # Full-field axis covers the drone's flight range; the zoomed
        # axis is sized so the rendered drone icon fills about half the
        # panel.
        rng = self.drone.range
        self.ax1.axis([rng[0], rng[2], rng[1], rng[3]])
        zoom_half = self.drone.DISPLAY_WIDTH
        self.ax2.axis([-zoom_half, zoom_half, -zoom_half, zoom_half])

        self.ax1.set_aspect('equal')
        self.ax2.set_aspect('equal')

        self.gs1.tight_layout(self.fig)
        self.fig.canvas.draw()

        # Store background for blitting
        self.ax1_bg = self.fig.canvas.copy_from_bbox(self.ax1.bbox)
        self.ax2_bg = self.fig.canvas.copy_from_bbox(self.ax2.bbox)

        # Re-capture the cached backgrounds whenever the figure is resized
        # so blits remain consistent with the current axes bbox.
        self._needs_redraw = False
        self._cid_resize = self.fig.canvas.mpl_connect(
            'resize_event', self._on_resize
        )

        self.reset()

        plt.show(block=False)

    def reset(self):
        """Reset display state and reinitialize target patches.

        Clears existing patches, resets text strings and frame counters,
        and recreates target visualizations from the drone's target list.
        """
        self.str1 = ""
        self.str2 = ""
        self.str3 = ""
        self.str4 = ""
        self.str5 = ""
        self.t=0
        self.frames = 0
        self.last_frame = time.perf_counter()
        self._last_text_update = 0.0

        for p in reversed(self.ax1.patches):
            p.remove()

        self.target_idx = 0
        if self.drone.targets is not None:
            self.num_targets = self.drone.num_targets
            self.targets = [None] * self.num_targets
            self.target_patches = [None] * self.num_targets

            for i in range(self.num_targets):
                t = self.drone.targets[i,:]
                self.targets[i] = plt.Circle((t[0], t[1]), t[2], lw=1, color='red', alpha=0.5, fill=False)
                self.target_patches[i] = self.ax1.add_patch(self.targets[i])

        self.patch1 = self.ax1.add_patch(self.drone1)
        self.patch2 = self.ax2.add_patch(self.drone2)

    def _on_resize(self, event):
        """Mark the blitting cache as stale after a resize.

        The actual re-capture happens at the top of update_display, so the
        cost is paid once per render iteration rather than once per
        resize_event (which Qt can fire many times during a drag).
        """
        self._needs_redraw = True

    def close(self):
        """Close the figure and release resources."""
        if hasattr(self, '_cid_resize'):
            self.fig.canvas.mpl_disconnect(self._cid_resize)
        plt.close(self.fig)

    def start_animation(self, physics_step, render_step, stop_condition,
                        max_catch_up_steps=5):
        """Run a fixed-timestep physics loop with an independent render rate.

        Physics ticks at ``self.drone.deltaT`` of wall-clock time so the drone
        always evolves at real-time speed. Rendering happens at ``target_fps``,
        which may be smaller than the physics rate on slow hardware. Sleep
        targets are deadline-based, so short overshoots in either loop cancel
        out instead of accumulating drift.

        Args:
            physics_step: Callable advancing the simulation by one drone step
                (drone.update plus any per-tick controller bookkeeping).
            render_step: Callable that redraws the figure (no return value used).
            stop_condition: Callable returning True when the loop should exit.
            max_catch_up_steps: After a long stall, do at most this many catch-up
                physics steps in a single outer iteration. Remaining backlog is
                dropped so the UI does not freeze trying to catch up.
        """
        physics_dt = self.drone.deltaT
        render_dt = 1.0 / self.target_fps if self.target_fps else physics_dt

        accumulator = 0.0
        last_wall = time.perf_counter()
        next_render = last_wall + render_dt

        while not stop_condition():
            now = time.perf_counter()
            accumulator += now - last_wall
            last_wall = now

            steps = 0
            while accumulator >= physics_dt and steps < max_catch_up_steps:
                physics_step()
                accumulator -= physics_dt
                steps += 1
            if accumulator >= physics_dt:
                accumulator = 0.0  # hit the catch-up cap, drop the backlog

            now = time.perf_counter()
            if now >= next_render:
                render_step()
                next_render += render_dt
                if next_render < now:
                    next_render = now + render_dt

            now = time.perf_counter()
            sleep_for = min(physics_dt - accumulator, next_render - now)
            if sleep_for > 0:
                time.sleep(sleep_for)

    def update_display(self, user_str=""):
        """Update display with current drone state.

        Args:
            user_str: Optional text to display in the plot.
        """
        if self._needs_redraw:
            self.fig.canvas.draw()
            self.ax1_bg = self.fig.canvas.copy_from_bbox(self.ax1.bbox)
            self.ax2_bg = self.fig.canvas.copy_from_bbox(self.ax2.bbox)
            self._needs_redraw = False

        [y, z, phi] = self.drone.pos
        [dy, dz, dphi] = self.drone.vel
        [F, tau, F_left, F_right] = self.drone.forces
        self.frames += 1

        # Throttle the numeric overlay refresh: the drone graphic is still
        # redrawn every frame, only the F / tau / position / velocity text
        # is recomputed at text_update_rate Hz. Field widths are sized to
        # the realistic SI range of each quantity so the row does not jump
        # as values cross digit boundaries. With monospace text this
        # eliminates the visible twitching.
        now = time.perf_counter()
        if now - self._last_text_update >= self._text_update_interval:
            self.str2 = f"F={F: 6.3f}N, tau={tau: 8.4f}Nm"
            self.str3 = f"F_left={F_left: 6.3f}N, F_right={F_right: 6.3f}N"
            self.str4 = f"y={y: 6.2f}m, z={z: 6.2f}m, phi={np.rad2deg(phi): 6.1f}deg"
            self.str5 = f"y'={dy: 6.2f}, z'={dz: 6.2f}, phi'={np.rad2deg(dphi): 6.1f}"
            self.text2.set_text(self.str2)
            self.text3.set_text(self.str3)
            self.text4.set_text(self.str4)
            self.text5.set_text(self.str5)
            self._last_text_update = now

        # draw drones (see https://stackoverflow.com/a/4891658)
        tr1 = trafo.Affine2D().rotate(phi).translate(y, z)
        tr1 = tr1 + self.ax1.transData

        tr2 = trafo.Affine2D().rotate(phi)
        tr2 = tr2 + self.ax2.transData

        self.fps_printer()

        self.text0.set_text(user_str)
        self.text1.set_text(self.str1)
        # text2..text5 are set inside the throttled block above; do not
        # overwrite them here on every frame.
        self.drone1.set_transform(tr1)
        self.drone2.set_transform(tr2)

        # check if target has been reached
        if self.drone.target_idx > self.target_idx:
            self.target_patches[self.target_idx].set_fill(True)
            self.target_idx += 1

        # Restore backgrounds for blitting
        self.fig.canvas.restore_region(self.ax1_bg)
        self.fig.canvas.restore_region(self.ax2_bg)

        # Draw all artists
        self.ax1.draw_artist(self.ax1.patch)
        self.ax1.draw_artist(self.text0)
        self.ax1.draw_artist(self.text1)
        self.ax1.draw_artist(self.drone1)
        for i in range(self.num_targets):
            self.ax1.draw_artist(self.target_patches[i])

        self.ax2.draw_artist(self.ax2.patch)
        self.ax2.draw_artist(self.text2)
        self.ax2.draw_artist(self.text3)
        self.ax2.draw_artist(self.text4)
        self.ax2.draw_artist(self.text5)
        self.ax2.draw_artist(self.drone2)

        # Blit and flush
        self.fig.canvas.blit(self.ax1.bbox)
        self.fig.canvas.blit(self.ax2.bbox)
        self.fig.canvas.flush_events()

    def fps_printer(self):
        """Update FPS display string every 10 frames."""
        if not self.frames % 10:
            t = time.perf_counter()
            self.str1 = f"fps: {10/(t-self.last_frame+1e-6):3.0f}"
            self.last_frame = t

# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
# --------------------------------------------------------------------------------
class Perlin:
    """Perlin noise generator for 1D coherent noise.

    Generates smooth, continuous pseudo-random noise using the Perlin noise
    algorithm. Used primarily for simulating natural phenomena like wind.
    """

    def __init__(self):
        """Initialize the Perlin noise generator with permutation table."""
        self.hash = [151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233,  7, 225,
                     140, 36, 103, 30, 69, 142,  8, 99, 37, 240, 21, 10, 23, 190,  6, 148,
                     247, 120, 234, 75,  0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32,
                     57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175,
                     74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122,
                     60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54,
                     65, 25, 63, 161,  1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169,
                     200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186,  3, 64,
                     52, 217, 226, 250, 124, 123,  5, 202, 38, 147, 118, 126, 255, 82, 85, 212,
                     207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213,
                     119, 248, 152,  2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172,  9,
                     129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104,
                     218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241,
                     81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157,
                     184, 84, 204, 176, 115, 121, 50, 45, 127,  4, 150, 254, 138, 236, 205, 93,
                     222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180,
                     151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233,  7, 225,
                     140, 36, 103, 30, 69, 142,  8, 99, 37, 240, 21, 10, 23, 190,  6, 148,
                     247, 120, 234, 75,  0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32,
                     57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175,
                     74, 165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122,
                     60, 211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54,
                     65, 25, 63, 161,  1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169,
                     200, 196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186,  3, 64,
                     52, 217, 226, 250, 124, 123,  5, 202, 38, 147, 118, 126, 255, 82, 85, 212,
                     207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213,
                     119, 248, 152,  2, 44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172,  9,
                     129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104,
                     218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241,
                     81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157,
                     184, 84, 204, 176, 115, 121, 50, 45, 127,  4, 150, 254, 138, 236, 205, 93,
                     222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180]
        self.hashMask = 255
        self.gradients1D = [1.0, -1.0]
        self.gradientsMask1D = 1

    def lerp(self, A, B, factor):
        """Linear interpolation between two values.

        Args:
            A: Start value.
            B: End value.
            factor: Interpolation factor in [0, 1].

        Returns:
            Interpolated value between A and B.
        """
        return (1.0 - factor) * A + factor * B

    def smooth(self, t):
        """Apply smoothstep function for smoother interpolation.

        Uses 6t^5 - 15t^4 + 10t^3 (Ken Perlin's improved smoothstep).

        Args:
            t: Input value, typically in [0, 1].

        Returns:
            Smoothed value with zero first and second derivatives at 0 and 1.
        """
        return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)

    def perlin_1d(self, point, frequency):
        """Generate 1D Perlin noise at a given point.

        Args:
            point: Position to sample noise at.
            frequency: Frequency multiplier for the noise.

        Returns:
            Noise value in range [-1, 1].
        """
        point *= frequency
        i0 = int(np.floor(point))
        t0 = float(point - i0)
        t1 = t0 - 1.0
        i0 &= self.hashMask
        i1 = i0 + 1

        g0 = self.gradients1D[self.hash[i0] & self.gradientsMask1D]
        g1 = self.gradients1D[self.hash[i1] & self.gradientsMask1D]

        v0 = g0 * t0
        v1 = g1 * t1

        t = self.smooth(t0)
        return self.lerp(v0, v1, t) * 2.0

    def sum(self, point, frequency, octaves, lacunarity, persistence):
        """Generate fractal Brownian motion by summing octaves of Perlin noise.

        Args:
            point: Position to sample noise at.
            frequency: Base frequency for the first octave.
            octaves: Number of noise layers to sum.
            lacunarity: Frequency multiplier between octaves (typically ~2).
            persistence: Amplitude multiplier between octaves (typically <1).

        Returns:
            Summed noise value, normalized by total amplitude.
        """
        total = self.perlin_1d(point, frequency)
        amplitude = 1.0
        amplitude_range = 1.0
        for o in np.arange(octaves):
            frequency *= lacunarity
            amplitude *= persistence
            amplitude_range += amplitude
            total += self.perlin_1d(point, frequency) * amplitude
        return total / amplitude_range


def hash32(ID):
    """Compute a 32-bit hash of an integer.

    Args:
        ID: Input integer to hash.

    Returns:
        Hashed integer with pseudorandom distribution.

    References:
        http://burtleburtle.net/bob/hash/integer.html
    """
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        a = np.uint32(ID)
        a -= (a << np.uint32(6))
        a ^= (a >> np.uint32(17))
        a -= (a << np.uint32(9))
        a ^= (a << np.uint32(4))
        a -= (a << np.uint32(3))
        a ^= (a << np.uint32(10))
        a ^= (a >> np.uint32(15))
    return a


def limited_hash(key, startbit, endbit):
    """Extract a range of bits from an integer.

    Args:
        key: Input integer.
        startbit: Starting bit position (0-indexed from right).
        endbit: Ending bit position (inclusive).

    Returns:
        Integer formed by bits from startbit to endbit.

    Example:
        limited_hash(93, 2, 5) -> 7
    """
    newkey = key >> startbit
    newkey = newkey & (np.power(2, endbit - startbit + 1) - 1)
    return newkey


def name_to_int(s):
    """Convert a name string to a deterministic integer hash.

    Args:
        s: Input string (typically a student name).

    Returns:
        32-bit hash derived from the first 13 characters.
    """
    s2=0
    idx = 0
    for c in s[0:13]:
        if c == ' ':
            s2 +=  5**idx
        else:
            s2 += abs(ord(c)-95)*5**idx
        idx += 1
    while s2>2**32:
        s2 = s2/1.5
    return hash32(int(s2))


def bits_to_float(key, bit1, bit2, factor):
    """Convert a bit range from a key to a scaled float.

    Args:
        key: Input integer containing the bits.
        bit1: Starting bit position.
        bit2: Ending bit position.
        factor: Scale factor for the output.

    Returns:
        Float value in range [-factor, +factor].
    """
    bits = limited_hash(key, bit1, bit2)
    fl = (bits - 7.5)/7.5 * factor
    return fl


def accel_to_phi_F(a_y, a_z, M=Drone.CF_M, g=Drone.CF_G):
    """Invert the drone equations of motion.

    The 2D drone's equations of motion (no wind) are::

        y_ddot = -sin(phi) * F / M
        z_ddot =  cos(phi) * F / M  -  g

    Given a desired ``(a_y, a_z)`` this function returns the unique
    ``(phi, F)`` that produces it:

        F   = M * sqrt(a_y**2 + (a_z + g)**2)
        phi = atan2(-a_y, a_z + g)

    The gravity term ``+g`` inside ``F`` provides the hover thrust
    automatically; no separate ``V_hover/cos(phi)`` feedforward is needed.

    Args:
        a_y: Desired horizontal acceleration.
        a_z: Desired vertical acceleration.
        M: Drone mass. Defaults to the Crazyflie reference ``Drone.CF_M``;
            pass ``drone.M`` explicitly if you ever override it.
        g: Gravitational acceleration. Defaults to the Crazyflie reference
            ``Drone.CF_G``.

    Returns:
        ``(phi_des, F_des)``: commanded tilt angle in radians and the
        total thrust to apply.
    """
    F = M * np.sqrt(a_y ** 2 + (a_z + g) ** 2)
    phi = np.arctan2(-a_y, a_z + g)
    return phi, F


def render_replay(drone, results, fps=30, size=500, max_frames=600):
    """Render a recorded drone trajectory as an inline SVG with a JS player.

    The notebook output is a single ``<svg>`` element carrying the drone
    polygon and the target rings in world coordinates, plus a short
    JavaScript player that rewrites the drone's ``transform`` attribute
    frame by frame. Per-frame state ``(t, y, z, phi, target_idx)`` is
    embedded next to the markup as a JSON blob, so the player never has
    to call back into Python.

    Compared with the older PNG-frame replay (one base64 PNG per frame,
    embedded in matplotlib's jshtml player), this produces a roughly
    150x smaller notebook output for the same race, and it does not need
    a Qt or matplotlib animation backend.

    Args:
        drone: Drone instance the results were produced with. Used for the
            drone shape, the world bounds, and the target positions only;
            its dynamic state is not read.
        results: (N, 14) array of recorded drone state, as produced by
            ``state_to_array`` inside the simulation loop.
        fps: playback frame rate (the JS player's setInterval rate).
        size: SVG width and height in pixels.
        max_frames: hard cap on the number of frames embedded. The results
            array is subsampled uniformly to fit. Lowering this shrinks
            the embedded payload at the cost of playback smoothness;
            when the cap kicks in the video plays faster than real time.

    Returns:
        IPython.display.HTML object embedding the SVG and its JS player.
        In a notebook the returned value renders inline.
    """
    import json
    import uuid
    from IPython.display import HTML

    n = len(results)
    if n == 0:
        raise ValueError("results array is empty")
    target_n = min(max_frames, n)
    idx = np.linspace(0, n - 1, target_n).astype(int)

    times = [float(v) for v in results[idx, 0]]
    ys    = [float(v) for v in results[idx, 1]]
    zs    = [float(v) for v in results[idx, 2]]
    phis  = [float(v) for v in results[idx, 3]]
    tgts  = [int(v)   for v in results[idx, 11]]

    y_min, z_min, y_max, z_max = drone.range
    # Runs with a clamped flight_range (e.g. [0, 0, 0, 0] for the jig
    # tests) would give a zero-size view: widen such axes to the default
    # +-3 m for the display only.
    if y_max - y_min < 1e-9:
        y_min, y_max = -3.0, 3.0
    if z_max - z_min < 1e-9:
        z_min, z_max = -3.0, 3.0
    w_y = y_max - y_min
    w_z = z_max - z_min

    # Drone polygon vertices in world units.
    poly_pts = drone.shape().get_xy()
    drone_polygon = " ".join(f"{p[0]:.4f},{p[1]:.4f}" for p in poly_pts)

    # Targets as static SVG circles; the JS player toggles their fill
    # as the drone passes through them.
    target_svg = ""
    if drone.targets is not None:
        for t in drone.targets:
            target_svg += (
                f'<circle cx="{t[0]}" cy="{t[1]}" r="{t[2]}" '
                f'stroke="red" stroke-width="0.02" fill="none" '
                f'opacity="0.6" class="tgt"/>'
            )

    # Per-replay unique id so multiple replays can coexist in one notebook.
    uid = uuid.uuid4().hex[:8]

    # Geometry lives inside the SVG; the time readout sits in an HTML
    # overlay so CSS handles its font size in pixels (avoiding the
    # SVG-% / viewBox-unit ambiguity).
    html = f"""
<div style="display:inline-block; font-family:sans-serif;">
  <div style="position:relative; width:{size}px; height:{size}px;">
    <svg viewBox="{y_min} {-z_max} {w_y} {w_z}"
         width="{size}" height="{size}"
         style="border:1px solid #ccc; background:#fff; display:block;">
      <g transform="scale(1, -1)">
        {target_svg}
        <g id="drone-{uid}">
          <polygon points="{drone_polygon}" fill="gray"/>
        </g>
      </g>
    </svg>
    <div id="time-{uid}"
         style="position:absolute; top:6px; left:8px;
                font-family:monospace; font-size:13px;
                color:#000; background:rgba(255,255,255,0.7);
                padding:2px 6px; border-radius:3px;">
      t = 0.00 s
    </div>
  </div>
  <div style="margin-top:6px; display:flex; align-items:center; gap:6px;">
    <button id="play-{uid}"  style="width:60px;">Play</button>
    <button id="reset-{uid}" style="width:60px;">Reset</button>
    <input  id="scrub-{uid}" type="range" min="0" max="{target_n - 1}"
            value="0" style="flex:1;"/>
  </div>
  <script>
  (function() {{
    const F = {{
      t:   {json.dumps(times)},
      y:   {json.dumps(ys)},
      z:   {json.dumps(zs)},
      phi: {json.dumps(phis)},
      tg:  {json.dumps(tgts)}
    }};
    const N   = {target_n};
    const FPS = {fps};
    const drone   = document.getElementById("drone-{uid}");
    const timeT   = document.getElementById("time-{uid}");
    const playBtn = document.getElementById("play-{uid}");
    const resetBtn= document.getElementById("reset-{uid}");
    const scrub   = document.getElementById("scrub-{uid}");
    const svg     = drone.closest("svg");
    const tgts    = svg.querySelectorAll(".tgt");
    let frame = 0;
    let playing = false;
    let timer = null;

    function render(i) {{
      const deg = F.phi[i] * 180 / Math.PI;
      drone.setAttribute("transform",
        "translate(" + F.y[i] + "," + F.z[i] + ") rotate(" + deg + ")");
      timeT.textContent = "t = " + F.t[i].toFixed(2) + " s";
      const reached = F.tg[i];
      tgts.forEach((t, j) => {{
        t.setAttribute("fill", j < reached ? "red" : "none");
      }});
      scrub.value = i;
    }}
    function stop() {{
      playing = false;
      playBtn.textContent = "Play";
      if (timer !== null) {{ clearInterval(timer); timer = null; }}
    }}
    function play() {{
      playing = true;
      playBtn.textContent = "Pause";
      timer = setInterval(() => {{
        frame++;
        if (frame >= N) {{ stop(); return; }}
        render(frame);
      }}, 1000 / FPS);
    }}
    playBtn.onclick = () => {{ playing ? stop() : play(); }};
    resetBtn.onclick = () => {{ stop(); frame = 0; render(0); }};
    scrub.oninput = (e) => {{ stop(); frame = parseInt(e.target.value); render(frame); }};
    render(0);
  }})();
  </script>
</div>
"""
    return HTML(html)
