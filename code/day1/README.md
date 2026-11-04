# Day 1: feedback control, from the simulator to the real drone

Andreas Freise, 01.11.2026

Hands-on material for the first day of the topical lectures. In the morning
you learn the basics of feedback control by flying a simulated drone in
Jupyter notebooks. In the afternoon the same position controller flies a real
Crazyflie, by radio, from the same notebook environment.

The notebooks are meant to be worked through in order. Each one is a task
sheet: read the text, run the cells, change the code where the text asks you
to. Keep `module.py` and `crazyflie.py` in the same directory as the
notebooks.

## Schedule

The day in [Indico event 8196](https://indico.nikhef.nl/event/8196/),
3 November 2026, and the notebooks of each block. The times are a
guideline; groups advance at their own pace.

* 10:00 to 10:15. Introduction: goals of the three days.
* 10:15 to 10:45. Software check, see below.
* 10:45 to 11:30. Lecture: feedback, stability, PID controllers.
* 11:30 to 12:30. Simulation, block 1, see below.
* 12:30 to 13:30. Lunch.
* 13:30 to 14:30. Simulation, block 2, see below.
* 14:30 to 14:45. Introduction to the Crazyflie hardware.
* 14:45 to 16:00. The real drone, see below.
* 16:00 to 16:15. Break.
* 16:15 to 17:00. The real drone, continued.

The figure and the table of each group are shown on one of the following
days, or at the end of the afternoon by groups that are done early.

## The software check

Everyone runs `00_jupyter_example` top to bottom and the first cells of
`2Dsim_01_keyboard_flying`, which opens a separate plot window and needs the
Qt backend of the full installation. One person per group connects to the
group's drone with the first cells of `drone_01_connect`: the drones in
range, a blink, the status.

## The simulator

Block 1: `2Dsim_01_keyboard_flying` (10 minutes), flying without a controller
is hard; it opens a separate plot window and needs the Qt backend of the full
installation. `2Dsim_02_system_identification`: measure the motor offset and
the hover voltage of your drone; the sections marked *optional* can be
skipped.

Block 2: `2Dsim_03_basic_control`: the tilt loop, the vertical loop with and
without feedforward, the integrator, then the cascade for the horizontal
position; the last section is optional. `2Dsim_04_velocity_loop`: a velocity
loop between position and tilt, with a speed limit in physical units. It is
the bridge to the afternoon: its position loop produces a feedback signal, a
velocity, and that is exactly what the real drone accepts by radio. Groups
that finish early: `2Dsim_05_race`, the race flown by a preprogrammed
sequence of setpoints, with an inline video, or
`extra_2Dsim_01_race_interactive`, the same race flown with the keyboard (Qt
backend).

## The real drone

Each group has one drone, four people and a laptop each with all notebooks,
and the flying area is large enough for all groups to fly at the same time.
One laptop is connected to the drone and runs the drone notebooks; one
person watches the drone and the abort key during flights; the other two
work on their own laptops with the analysis notebooks. The group decides who
does what, and swaps between runs.

The drone notebooks, in order:

1. `drone_01_connect`: the drones in range, yours by the label on its
   sticker, a blink, the status, the height sensor. Done at the desk, as in
   the software check of the morning.
2. `drone_02_firmware_flight`: the first flights, under the drone's own
   controllers: a path from Python, forward and back, then a square; saved
   like all flights.
3. `drone_03_handtest`: motors off. The code of our position loop, the
   measurement protocol by hand for the axes and the noise of the estimate,
   then the sign of the gains from the feedback of the code while the drone
   is moved by hand.
4. `drone_04_identification`: one flight, a velocity setpoint step. The
   response time `tau` and the dead time of the velocity loop, and the noise
   bounds from the hover: the parameters of the drone, recorded.
5. `drone_05_tuning`: one gain per flight, a prediction before each flight,
   the row in the table after it. A gain too small on purpose, the gain the
   model suggests, then `Kp` up until the drone overshoots, `Kd` against the
   overshoot, `Ki` against the offset, the same gains twice. One run takes
   about 20 s in the air and three minutes with setup and saving.

The analysis notebooks run alongside, on the laptops without the radio.
`drone_analysis_01_flight_plan` runs a model of the drone, from the
parameters recorded in part 4 or from typical values before that, and gives
the pilot the flight plan and the prediction before each flight.
`drone_analysis_02_summary_figure` takes a copy of the `data/` folder of the
drone laptop, whenever there are flights to show, and makes the table and
the figure of the group. The reference recordings under `data/reference/` serve
while the group has no files of its own.

Flying rules: the marked area only, one person at the keyboard with the cursor
at the abort window (Enter lands the drone), one person watching, nobody within
arm's length of a running drone, a full battery. The loop lands by itself
when the drone leaves a 1 m box around its start, when the radio link or
the position estimate fails, or when the battery sags. Change the battery
when the status shows less than 3.5 V at rest; spare batteries and the
charger are with the instructors.

## Files

* `00_jupyter_example`: Jupyter primer. Skip if you have used notebooks before.
* `2Dsim_01_keyboard_flying`: first flight, keyboard control of the raw motor voltages.
* `2Dsim_02_system_identification`: measure your simulated drone (motor offset, hover voltage, clipping limits).
* `2Dsim_03_basic_control`: the first feedback loops. Tilt PD, then a flat cascade for altitude and lateral position.
* `2Dsim_04_velocity_loop`: a velocity loop between the position controller and the attitude loop, with the speed limit at the right place.
* `2Dsim_05_race`: the race, flown by a preprogrammed sequence of setpoints. The end of the simulator part.
* `drone_01_connect`: the drones in range, yours by its label, connect, status, a blink of its LEDs, the height sensor.
* `drone_02_firmware_flight`: the first flights, under the drone's own controllers: a path from Python, forward and back, a square, saved like every flight.
* `drone_03_handtest`: the code of our position loop, shown and then run for the first time with the motors off: the direction of the axes, the noise and the drift of the position estimate from a guided protocol by hand, then the sign of the gains from the feedback of the code while the drone is moved by hand.
* `drone_04_identification`: the parameters of your drone from one flight under the firmware: the velocity setpoint step, the response time and the dead time read by eye and by a fit, the noise of the hover and its bounds on the gains, the model's step responses for each gain; the results recorded for part 5.
* `drone_05_tuning`: our own position loop, tuned one gain per flight: the prediction before each flight from the formulas and the model, a first flight with a gain too small on purpose, the flights with `Kp`, `Kd` and `Ki`, the record and the table of the runs.
* `drone_analysis_01_flight_plan`: the model of the drone, written out, for the members of the group not at the drone laptop: the flight plan before the tuning, the prediction for each flight during it, model flights saved like real ones; no drone needed.
* `drone_analysis_02_summary_figure`: all saved flights side by side, real, firmware and model: the table, the step responses over each other, the paths, the hold against the hover of the identification flight, our loop against the firmware flight, the figure for the summary.

Extras, not part of the day, for groups that finish early and for later:

* `extra_2Dsim_01_race_interactive`: the same race as `2Dsim_05_race`, flown with the keyboard (Qt backend).
* `extra_2Dsim_02_model_based_control`: the race with an algebraic inversion of the dynamics instead of the cascade.
* `extra_2Dsim_03_trajectories`: smooth trajectories through the waypoints with feedforward, on top of the inversion.
* `extra_drone_analysis_flight_data`: what the five files of a flight hold beyond the step response: height against the range sensor, roll and pitch, battery and motors, the flow sensor, the path in the plane with time, feedback against velocity; the figure for the summary as the task.

Supporting files:

* `module.py`: the simulator (`Drone`, `Plotter`, `PIDController`, `accel_to_phi_F`, `render_replay`).
* `crazyflie.py`: the radio link, logging, the two flight loops, saving and loading of runs. Also a terminal tool: `python crazyflie.py find | status | flow | recover | reboot | charge`, and for the instructors `config`, `relabel` and `channel`.
* `drone_parameters.md`: where the simulator's parameters come from.
* `data/reference/`: recorded sensor identification, hand test and two step flights, for the analysis without a drone.
* `testing/`: the tests, run from this folder with `python testing/test_module.py` and `python testing/test_crazyflie.py`. The second one runs the flight loops against a fake drone and needs no hardware; it is also the check that cflib and the loops work in your environment. `python testing/test_tuning.py` checks the tuned controller gains of `2Dsim_03` and `2Dsim_04` against the original ones and writes comparison plots to `testing/tuning_plots/`.

## Software

The simulation notebooks need numpy, scipy and matplotlib, as pinned in the
course requirements. `2Dsim_01_keyboard_flying` and `extra_2Dsim_01_race_interactive` open a separate animated window and
need the Qt backend, which the full installation provides through cfclient;
all other notebooks plot inline. `2Dsim_02` to `2Dsim_05` show every run as an
inline video (`module.render_replay`). The flight notebooks need cflib, also part of
the full installation. The radio address of your drone follows from the label
on its sticker, entered at the top of each flight notebook; `drone_01_connect`
lists the drones in range and checks yours.

## Sharing a Lighthouse setup

Use one drone with a working Lighthouse setup as the source for the fleet.
With the drones on the ground and other radio clients disconnected, run from
`code/day1`:

```bash
python crazyflie.py lighthouse-export --label 0 --out lighthouse-fleet.yaml
python crazyflie.py lighthouse --label 1 --yaml lighthouse-fleet.yaml
```

Replace `0` with the calibrated source drone's label and `1` with a target label.
Repeat the upload for each target. Export reads stored base-station geometry,
factory calibration and Lighthouse type, validates them, and saves a complete
cfclient-compatible configuration. Export and upload leave controller/estimator
selections unchanged. Missing calibration or a read failure prevents export;
existing output files are protected against overwriting.

The complete file supplies the same coordinate system and station corrections
to every drone. Use it for the same physical base stations, channel assignments
and unchanged layout. After export or upload, power-cycle the affected drone
before preflight; the helper leaves its firmware motor stop engaged. Check
positioning on every drone before flight. Export copies the source's stored
solution; the geometry-fit diagnostics described below belong to session-based
calculation, rather than being measured during a memory export.

### Geometry sessions

The `lighthouse --label LABEL --yaml FILE` command also accepts both cfclient
export formats:

- A configuration export supplies calculated geometry and base-station calibration.
- A geometry-session export supplies the recorded measurements. The helper runs
  cflib's geometry solver, using cfclient's convention that the first X-axis
  sample is one metre from the origin. It preserves the connected drone's
  base-station calibration and system type. Use a session from the same physical
  base stations and channel assignments. If their calibration is missing, keep
  motors stopped, place the drone in view of the base stations and let cfclient
  receive the calibration data before retrying.

To calculate a session once and reuse its geometry across drones, prepare it
offline, then upload the prepared file by label:

```bash
python crazyflie.py lighthouse-prepare --yaml session.yaml --out geometry.yaml
python crazyflie.py lighthouse --label 2 --yaml geometry.yaml
```

The prepared YAML stores base-station positions and rotations, the source
session's SHA-256, input-pose counts, solver convergence status, and mean,
standard deviation and maximum beam disagreement for the fit and independent
verification samples. The report is also printed when calculating or loading
the geometry. Missing verification data is reported explicitly. Uploading a
prepared file does not rerun the solver and preserves onboard calibration.
Recompute after moving the base stations or changing the coordinate frame.
Preparation refuses to overwrite an existing file.

Geometry calculations require valid origin, X-axis and XY-plane samples, and at most
0.02 m maximum beam disagreement in the fit and any verification samples. The
helper reports solver non-convergence as a warning when those checks pass.
Beam agreement checks internal consistency, not absolute positioning accuracy.

The helper verifies the firmware motor stop, detects the target's supported
memory slots, persists the configuration and checks read-back before resetting
the estimator. Geometry for omitted base stations is invalidated; configuration
exports also replace stored calibration entries. On success, power-cycle
the drone and perform the normal preflight checks; the motor stop stays engaged
until then. If the upload fails, keep the drone grounded and retry the complete
export after reconnecting. `--timeout 60` allows a longer upload wait.

From Python, the same upload is available as
`connect(drone_uri(2), lighthouse_config='lighthouse.yaml')`.

## Drone model

The simulated drone is a 2D slice (y, z, phi) of a Crazyflie 2.1, with
parameters matched to the high-fidelity Simulink model in Richardson and
Pretorius, MATEC Web of Conferences 417, 04007 (2025). See
`drone_parameters.md` for the full parameter list and where each value comes
from. The real drone uses its optical flow deck and height sensor for the
position estimate; the horizontal position loop is the only loop that runs on
the laptop, the velocity and attitude loops run in the firmware.
