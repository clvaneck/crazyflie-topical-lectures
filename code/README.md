# Course code

The [day guides](../docs/README.md) and worksheets describe the exercises;
the [glossary](../docs/glossary.md) explains their terminology.
For a guided route, open the [Day 1 primer](day1/00_jupyter_example.ipynb),
[Day 2](day2/01_tracking.ipynb) or [Day 3](day3/02_trajectory.ipynb) notebook in
`jupyter lab` from the project root, using **Crazyflie lectures (.venv)**.
Navigate into `code/` to open a notebook; `docs/` remains available for its links.
The [Day 1 notebook guide](day1/README.md) lists the simulation and drone notebooks
in teaching order, with optional extensions marked separately.
Day 2 continues with [02 calibration](day2/02_calibration.ipynb) and
[03 height and velocity](day2/03_height_velocity.ipynb); optional studies are in
[extra six-state estimation](day2/extra_six_state.ipynb).
Day 3's [survey notebook](day3/01_survey.ipynb) records and fits gate geometry
for the [trajectory notebook](day3/02_trajectory.ipynb), with shared-data and simulated fallbacks.
After activating the native environment, run from the course project root
(the cloned folder containing `code/`, `scripts/` and the requirements files):

```bash
cd code
python day1/testing/test_module.py
python -m day2.simulate
python -m unittest kalman.test_filter
```

| Exercise commands | Reusable code |
| --- | --- |
| [day1](day1/README.md): controller experiments, hand checks and flight analysis | Adjacent `module.py` simulator, `crazyflie.py` radio helpers and reference recordings. |
| [day2](day2/README.md): timestamped accelerometer/ToF fusion, editable height-velocity expressions and optional six-state studies | [kalman](kalman/README.md): six-state filter, callbacks, recording validation and baseline simulation. |
| [day3](day3/README.md): motor-free gate survey, course construction and race analysis | [race_common](race_common/README.md): gate geometry/policies; `trajectory_tools`: course construction. |

Day 1 notebooks use their adjacent helpers; Days 2 and 3 use the shared filter,
control and geometry modules. `andreas` provides additional controller experiments
and a guided notebook.
`trajectory_tools` supplies [offline course construction and preview](../docs/trajectory_construction.md),
without radio access or flight execution.
Hardware execution uses a separately supplied,
tested flight program and the team's documented preflight checks.
Use [the flight notebook](day3/03_flight.ipynb) to check and launch that program.

The [Day 2 guide](day2/README.md) documents APIs, assumptions, and
simulation limitations. The [post-flight guide](../docs/post_flight_analysis.md)
describes telemetry fields, alignment and optional command-line plotting.
