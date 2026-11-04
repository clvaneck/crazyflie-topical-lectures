# Measure the gates with the supplied survey tool

Use [01_survey.ipynb](../code/day3/01_survey.ipynb) for R2: record an outline,
fit a circle or rectangle, inspect it and save the gate JSON for the race planner.
It reuses the labelled connection and motor-free acquisition conventions from
Days 1 and 2. The exercise is to collect a useful measurement, check the geometry
and understand what could bias it.

Allow 40 minutes for a shared survey of the course, with 20 minutes reserved for
retries or hardware problems. Teams can use the same survey files and share
collection/checking roles; a separate full survey by every team is not required.
The number of gates and available survey stations determine what fits. If it
takes longer, use checked fallback geometry or reduce the course for everyone,
not the safety checks. Agree the final shared course before qualification.

## Assignment

1. Open the survey notebook, confirm the Lighthouse frame and connect by sticker
   `LABEL`. Put the drone on a stable surface, clear of propellers, and close
   cfclient. Wait for **STOP VERIFIED** before handling: the helper engages
   `stabilizer.stop=1` and checks the firmware lock and four zero motor outputs.
2. Use a pointer/jig referenced to the drone's estimated position. Account for
   any offset: pressing a deck or cage against the rim can shift the fitted gate.
   The script does not correct that offset. With fixed attitude, a known constant
   world-frame tip offset is added to the fitted centre when building the course;
   keep the original JSON and record the correction. Changing attitude makes this
   a sample-by-sample correction, outside this exercise. Avoid covering receivers.
3. Select live capture in the notebook. After stop verification, the default
   five-second preparation interval precedes **TRACE STARTED**. Trace the complete
   inner opening over 30 seconds; recording ends automatically. One team member
   operates the laptop while another moves the drone at steady attitude with
   receivers visible. Trace all four edges/corners for a rectangle. Select the
   [normal](glossary.md#clearance-and-gate-normal) sign for the
   intended crossing direction; reversing it does not move the physical plane.
4. Check the centre, normal and aperture against independent manual measurements
   or a second pass not used in the fit. Examine the printed residuals, but do not
   treat a small residual as proof of absolute accuracy. Repeat poor captures.
5. Keep the JSON and check results. Use the geometry in the course configuration
   and plot the planned path for the team's flight checks.

## Optional command-line alternative

The original `scripts/measure_hoop.py` remains available with the same fitter and
JSON format. From an activated project-root terminal, pass the URI returned by
the notebook's **Connect** cell:

```text
python scripts/measure_hoop.py --uri YOUR_DISCOVERED_URI --shape circle --name hoop-1 --output survey/hoop-1.json
```

Use `--shape circle` for a hoop or `--shape rectangle` for a rectangular gate;
change the name and output file for each gate.

Existing files are protected unless `--force`
is supplied. Default sample spacing is 0.02 m; `--sample-spacing 0.01` retains a
denser trace. The CLI asks for `STOP AND SURVEY`, verifies the motor stop, then
starts at `Command [Enter=record, q=quit]:` when you press **Enter on the laptop**.
Press Enter again after tracing, then confirm `Fit this trace?`. Move slowly
and watch the physical drone. The script logs position and sends no arming or flight commands;
it sets only the firmware stop parameter before acquisition and leaves it engaged.
Keep clear of propellers while handling. Afterwards, place the drone down and
power-cycle it before the normal preflight checks for flight. Missing stop/status
support prevents surveying; use checked shared geometry.

During each CLI trace, connection loss, logging errors, invalid positions and long
sample gaps reject the acquisition, even if enough earlier points exist to fit
a gate. The freshness limit is 0.5 s, or three requested log periods if longer.
On a warning, stop moving and press Enter to finish the trace. The tool saves
partial observations and their failure reason under `<output-stem>-failed-traces/`
beside the requested output file; these files contain no fitted gate geometry.
An interrupted recording is saved there too. Reconnect and record a fresh trace,
or use checked shared geometry. The firmware stop remains engaged.

The notebook saves `positions.csv` and `recording.json` per capture,
checks Lighthouse and motor status throughout, and rejects incomplete/changed
recordings before fitting. A failed capture retains its observations and reason.

## Constraints and interface

The [course format and flight-tool guide](course_interface.md) describes the full course file.
A survey JSON describes one physical gate. The team builds the course, checks
the shared dry-run output and completes its preflight checks before flying.

- All positions are metres in the same right-handed Lighthouse frame as the
  flight estimator, with Z upward. Record what physical point is being measured.
- Circle output contains `name`, `centre`, unit `normal`, `radius_m`, and a
  `survey` object with sampled `points_m`, sample count and plane/radial residuals.
- Rectangle output adds `shape`, `width_axis`, `height_axis`, `width_m`, `height_m`
  and outline residuals. Its `radius_m` is half the smaller dimension: the circular
  runner's conservative inscribed aperture. Keep the full rectangle output for
  plotting; width or diagonal must not be used as the radius.
- Copy the runner's gate fields into the course's `hoops` list, assigning each
  required crossing a unique name and normal direction. Preserve a separate map
  from repeated crossings to the physical survey file. Keep
  `enabled_for_flight=false` until qualification.
- Budget separately for drone size, gate thickness/supports, raised floor,
  survey/estimator/tracking error and safety margin. Fit residuals alone are not
  that error budget. A horizontal hoop needs its measured orientation too.
- A moved gate or changed Lighthouse frame invalidates the survey; recheck it
  before flight. Never use unqualified autonomous flight to locate an obstacle.

## Deliverables

Keep the survey JSONs, their independent check, the course configuration and one
geometry/path plot. The JSON already stores points and fit residuals; there is
no separate survey report or new reconstruction code to submit. Note the shared
data source and any pointer offset or unresolved error in the team notes.

**Acceptance check:** explain the centre, normal sign, aperture and main
uncertainty, and show how you checked them. A second team member cross-checks
the physical setup and margins before a powered crossing. Developing your own
survey/fitter is an optional extension for further exploration.

## Time limit and fallback

If collection or validation overruns, keep the attempted measurements and switch
to an already checked shared survey or instructor fallback. Label it as supplied
geometry and retain its source and validation record. Use the same agreed course for all
timed competitors. If no physical geometry is qualified, work with recordings or
simulation instead. Never relax clearance checks to meet the timetable.
