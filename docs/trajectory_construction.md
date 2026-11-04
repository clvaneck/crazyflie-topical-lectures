# Construct and preview a race trajectory

The [Day 3 trajectory notebook](../code/day3/02_trajectory.ipynb) contains the
working instructions. This reference explains the Python choices, optional
command-line route and planning checks. **Stretch goal: continuous and limited
jerk** is an optional development project with additional implementation and
validation. Planning uses the Python simulation environment.
Commands below run from the project root in bash or PowerShell after activating
the environment described in the [setup guide](software_setup.md).

## Try it without hardware

```bash
python scripts/plan_trajectory.py --course examples/course.synthetic.json --output plans/first --show
python scripts/plan_trajectory.py --course examples/course.shuttle.synthetic.json --laps 2 --output plans/two-laps --show
```

The second example crosses one opening forward and back on each of two laps:
four crossings in total. These are synthetic examples, not measured flight courses.
Omit `--show` on a machine without a display. The saved preview still works.
Choose a new output directory for each comparison; existing results are not overwritten.

Each output directory contains:

- `course.json`: runner-format geometry, ordered uniquely named crossings, and
  **flight disabled**, even when the input file had flight enabled.
- `preview.png`: planned reference in 3D and XY/XZ/YZ projections, opening
  outlines, crossing arrows, start/finish and speed/acceleration histories.
- `trajectory.csv`: `time_s`, XYZ position in metres, velocity in m/s and
  acceleration in m/s2. All vectors use the Lighthouse world frame.
- `summary.json`: planned duration, conservative speed/acceleration bounds,
  each crossing's reference time, and input file paths.

For notebook exports, keep `course.json` and `summary.json` together when moving
a plan to the flight laptop. The summary preserves survey references and the
clearance budget, bound to the course content. Reloading recomputes trajectory
results; a missing or mismatched summary requires re-exporting the selected plan.
The flight notebook checks the summary again before launch and archives it as
`race.summary.json`, alongside the exact course and tuning. It preserves the
planned-course hash and adds `flown_course_sha256` and `tuning_sha256` for the
archived file bytes (`tuning_sha256` is null for an untuned flight). Enabling
flight changes only `enabled_for_flight`; other edits require a new planner
export and dry run. The archive is retained on abort.
Direct `day3.tuning.export_race_pair(course_path, tuning, directory)` exports
also validate and copy the companion summary into both submissions. For an
in-memory course, supply its matching `summary=` explicitly; the notebook-facing
API supplies this evidence automatically.

## Use measured gates

1. Collect and independently check gate JSONs using the [survey tool](hoop_survey_challenge.md).
2. Copy a synthetic course to a team template and edit its name, airborne start
   and finish, and agreed bounds/limits. Choose these for the actual takeoff
   location and qualified room volume. Keep the survey files alongside it.
3. Order the surveyed gates with repeated `--crossing NAME:+` or `NAME:-` options.
   `NAME` is the survey's `name` field; `+` follows its normal and `-` reverses it.
   `--laps` repeats the whole ordered crossing sequence.

For a survey named `gate-1`, forward and return crossings on two laps:

```bash
python scripts/plan_trajectory.py --course team-template.json --gate-survey survey/gate-1.json --crossing gate-1:+ --crossing gate-1:- --laps 2 --name team-two-laps --output plans/measured --show
```

Repeat `--gate-survey` for several openings and `--crossing` for their intended
order. Without survey options, the planner uses the template's existing `hoops`
sequence. Every generated crossing receives a lap/event prefix for identification.
For an out-and-back course, put the finish on the start side of the gate; otherwise
reaching it may require an additional unlisted crossing. Reversing direction at a
shared approach/exit point brings the reference velocity to zero there.

Rectangle outlines retain measured width, height and axes, including horizontal
gates. Planning uses the conservative inscribed-circle aperture expected by the
[course interface](course_interface.md). Pointer/jig offsets are not corrected
automatically: apply and record the correction described in the survey guide first.
The tool reads its geometry from JSON files. Edit gate
geometry or endpoints in JSON and rerun to inspect a different layout.

## Notebook choices

With the common setup loaded, gate poses and crossing order are enough:

```python
plan = plan_race(shared_setup, gates=surveys, crossing_order=["A:+", "B:+", "A:-"])
display(plan.summary())
display(plan.plot())
```

The shared setup supplies checked airborne start/finish positions, the flight
volume and full drone envelope. Aperture dimensions come from the surveys or
explicit `gate_defaults` in that setup, for example `{"radius_m": 0.5}` for
known 1 m circular openings. The planner never guesses room bounds or aperture size.

**Default conservative profile:** requested cruise at most **0.20 m/s**, peak
speed **0.30 m/s**, acceleration **0.50 m/s2**, nominal tilt **10 degrees** and
at least **0.10 m** additional aperture margin. Tighter shared limits are kept.
Entry/exit and turning points are at rest; centre crossings remain moving.
Subdividing the Bernstein control hulls tightens the continuous motion bounds,
avoiding unnecessarily long flights while retaining the same limits.
Takeoff/landing durations are at least three seconds. Frame depth defaults to
**0.10 m**; specify the measured `frame_depth_m` globally or per gate and check
projecting supports against `frame_margin_m`.

The planner inserts stop waypoints to bypass gates during transitions, treating
the whole opening/frame envelope as blocked except for the scheduled crossing.
Automatic detours stay within the altitude range already required by the
endpoints, gate approaches and explicit waypoints.
It rejects an obstructed route instead of widening the flight volume or reducing
clearance. Preview the resulting path and check floors, supports, visibility
and tracking in the existing low-speed qualification; geometric planning alone
does not establish safe physical flight. The exported course keeps flight disabled.

For a custom trajectory, select `conservative=False` explicitly and choose the
additional Python values below. Shared limits and the normal path checks still apply:

```python
plan = plan_race(
    template, gates=surveys, crossing_order=["A:+", "A:-"], laps=2, conservative=False,
    speed=0.3, clearance=0.10, approach_distance=0.35,
    gate_speeds={"A": 0.4, "lap-1-2-A": 0.25},
    crossing_offsets={"A": [0, 0.05, 0]},
    turning_waypoints={1: [{"position": [1.7, 0.4, 0.7], "velocity": [0, -0.1, 0]}]},
)
display(plan.summary())
display(plan.plot())
```

These numbers illustrate the API; use measured positions and the shared limits.
Gate IDs apply to all passages; an exact crossing name overrides an individual
passage. Speeds are m/s. Offsets are XYZ world-frame metres within the gate plane,
so reversing the crossing direction retains the same physical offset. The
physical gate centre and radius remain unchanged. Turning keys count completed
crossings across the whole race: 0 is before the first, 1 after the first, and
so on. A plain XYZ waypoint, or a dictionary containing only `position`, gives
a stop. In custom mode, an optional `velocity` specifies a world-frame XYZ
target in m/s. Mix stops and fly-through points in the same list. Each target
must lie within the shared speed limit; nonzero targets require distinct knots.
The planner sets zero acceleration at every join and retains C2 continuity.

`plan.summary()["turning_details"]` and the exported summary record requested
and resulting reference velocities with waypoint times. The result equals the
requested vector divided by the global time-stretch factor; its direction is
preserved. Check the complete new curve, since changing a velocity also changes
the path between waypoints. Bounds, frame clearance, crossing order, speed,
acceleration and tilt checks remain active. Conservative mode retains stopped
waypoints and automatic bypasses; choose `conservative=False` for fly-throughs.

Empty mappings retain centre crossings and the global speed. Conservative mode
adds any required bypass stops; custom mode uses only the selected waypoints.
The API checks offsets against the conservative circular aperture and
retains the shared margins. Every connecting curve still undergoes the same
continuous bounds, finite-frame and crossing-order checks. The summary shows
the planned speed after global time stretching, alongside each requested speed.

## Model and checks

Each gate adds entry, crossing and exit points along its normal, through the
chosen in-plane crossing point. Additional turning waypoints are visited in order.
Piecewise [quintic polynomials](glossary.md#trajectory-continuity) join them with continuous position, velocity and
acceleration; acceleration is zero at each join. Endpoints and explicit turning
waypoints are at rest. Crossing velocity follows the requested normal.
[Jerk](glossary.md#jerk-and-snap) can jump at joins;
the optional extension below explores higher-order continuity.

The implementation uses Bernstein polynomials. Their control-point convex hulls
bound the entire reference and its derivatives between plotted samples as well.
The planner stretches time uniformly until the conservative speed, acceleration
and nominal tilt bounds are met. Consequently `target_speed_m_s` sets the initial
timing, which can be stretched to satisfy those bounds. The resulting duration
can be conservative. Bounds apply to the reference position;
cage/swept-volume [clearance](glossary.md#clearance-and-gate-normal)
to walls and supports still needs a separate check.

The planner rejects insufficient circular aperture margin and out-of-box control
hulls. Each gate occupies a **finite footprint** in its plane. At each plane
intersection:

- Passage through the clear opening must be a listed crossing, in order and in
  the specified direction.
- The opening's boundary/frame exclusion band cannot be crossed.
- Passing outside the finite gate footprint plus clearance is allowed and does
  not count as a gate passage. Separate openings can share a plane.

Each gate can specify `frame_margin_m`: physical extent beyond the measured
inner opening, in metres **per side**. The default is **0.10 m**, an assumption
to review against the actual frame and supports, not a measured dimension or a
guarantee of safety. It extends a circle radially or a rectangle on all four
sides. Use enough padding to enclose projecting supports; irregular obstacles
not covered by that footprint still need separate checks. For example:

```json
"frame_margin_m": 0.15
```

Keep this field in each course/survey gate, or use `--frame-margin 0.15` to
override all gates when planning. The chosen/default value is written into the
exported course for every crossing. Bypass clearance additionally includes
`drone_radius_m + clearance_margin_m`. Thus a 0.50 m opening radius, 0.15 m frame
padding, 0.12 m drone radius and 0.08 m clearance require a bypass radius **greater
than 0.85 m**. The orange dashed outline marks that outer bypass boundary; the
grey outline marks the opening. For a rectangle the outer envelope is a
conservative expanded box in the gate's measured axes.

Segments lying along a gate plane are allowed only when their entire curve can
be shown to stay outside the finite padded footprint. Ambiguous contact fails
conservatively. These are plane-intersection checks, not a complete swept-volume
collision model: gate thickness, nearby parallel passes, raised floors, supports
outside the padded footprint, Lighthouse visibility, tracking error, sensor
uncertainty and actuator response still need qualification.

## Limitations of the baseline

The baseline makes the following modelling and planning choices:

| Current choice | Consequence |
| --- | --- |
| Quintic segments with zero acceleration at every join | Position, velocity and acceleration are continuous (C2), but jerk can jump. The reference may demand abrupt changes in thrust rate and attitude rate. |
| Chosen gate offsets, speeds and waypoint velocities | The team chooses the racing line. Zero-velocity points and shared reversals stop; custom fly-through points constrain the tangent. |
| Uniform time stretching using convex-hull bounds | Simple, conservative timing; no local speed optimization or explicit jerk limit. |
| Geometric reference and nominal tilt bound | No closed-loop tracking prediction, motor/thrust or body-rate limits, battery model, or race-time distribution. |

The finite-footprint and flight-interface limitations above still apply.
Smoothing the reference may help tracking, but does not by itself establish a
faster or safer physical flight.

## Stretch goal: continuous and limited jerk

If the baseline is working and time remains, explore this offline; it is **not
required for the race** and does not need an extra report. Keep the supplied
planner and qualified flight fallback unchanged. There is no extra scheduled
slot: stop before this displaces qualification or crosses the configuration freeze.

Jerk is the third derivative of position:

\[
j(t)=\frac{d^3r(t)}{dt^3}.
\]

The aim is **continuous, bounded jerk**, allowing acceleration to change smoothly
throughout the flight. The planner specifies this smoothness explicitly; the
controller then tracks the resulting reference. Actuator lag can also smooth
motion, accompanied by tracking error. The current position-setpoint interface
sends position and yaw; using the planner's velocity/acceleration arrays as
feed-forward commands requires a compatible flight interface.

One approachable experiment:

1. Inspect `build_trajectory` in `code/trajectory_tools/planner.py`. Plot jerk
   within each segment, and evaluate its left/right limits at every join.
   Ordinary sampled plots or finite differences alone can miss a jump.
2. Replace the quintic construction with degree-seven (septic) segments. Each
   axis then has eight coefficients, enough to specify position, velocity,
   acceleration **and jerk at both ends**. Share those derivatives at every join.
   Setting acceleration and jerk to zero at each knot gives a simple first
   version. Allowing nonzero shared derivatives offers further timing choices.
3. SciPy's existing `BPoly.from_derivatives` can build this reference: provide
   `[position, velocity, acceleration, jerk]` at each knot, in physical-time units.
   Retain the approach/centre/exit geometry and crossing directions initially,
   so the comparison changes smoothness rather than everything at once.
4. Recheck endpoints, left/right continuity through the third derivative,
   whole-path bounds, crossing order, finite frame clearance, speed,
   acceleration and nominal tilt. Add an explicit jerk bound in m/s3 and a test
   that checks it over the whole curve, for example using derivative control hulls.
   Stretching duration by a factor `s` reduces jerk by `s**3` for a fixed curve.
5. Compare the baseline and extension under the same course and existing limits:
   plot acceleration/jerk, record join jumps, peak jerk and planned duration.
   Do not claim timing gain merely by relaxing a limit. If a suitable closed-loop
   simulator is available, also compare tracking and saturation; otherwise leave
   that benefit as an untested hypothesis. Link the comparison in the existing notes.

A further extension is a joint minimum-snap fit, minimizing
\(\int \|d^4r/dt^4\|^2\,dt\) with waypoint and derivative-continuity constraints,
rather than setting all knot accelerations/jerks to zero. Neither a septic fit
nor minimum-snap optimization automatically respects actuator limits or supplies
a qualified flight adapter. Any hardware use needs the same separate review as
other changes to the racing reference.

## From preview to flight

The preview shows **the planned geometric reference**. Its duration covers the
reference between airborne endpoints. Predicting flown motion and race-time
spread requires a closed-loop simulation; actual timing comes from flight
measurements.
Keep actual geometry, source surveys and any corrections with the saved output;
the summary's file paths are not a substitute for retaining those files.

Notebook API exports carry `trajectory_version: 1`. The common runner uses the
shared planner for these files, including crossing choices and turning points,
and confirms this during its dry run. The flight notebook rejects a runner
without that confirmation. Uniform command ticks can stretch the duration by
less than one period. Compare the runner's dry-run plot and limits before the
low-speed qualification; a preview alone does not establish flight readiness.

For the practical, saving this preview and checking it together is enough;
no additional report or planner implementation is expected. Reuse the approved
fallback when time or hardware prevents qualifying a candidate.

Offline tests, from `code`:

```bash
python -m unittest trajectory_tools.test_planner
```
