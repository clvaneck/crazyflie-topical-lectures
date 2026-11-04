# Day 3: race

Start `jupyter lab` from the project root, using **Crazyflie lectures (.venv)**.
In the file browser, open [01_survey.ipynb](01_survey.ipynb) to record and fit
gates, then [02_trajectory.ipynb](02_trajectory.ipynb) to plan and analyse the race.
Use [03_flight.ipynb](03_flight.ipynb) on the flight laptop for the tested
runner's dry run and flight launch, with explicit readiness and course confirmation.

Use Day 1 tracking and Day 2 covariance for margins. Submit baseline and optimized
course/tuning configurations for the common runner; compare one bounded PID-gain
change at fixed course/speed. Kalman tuning is optional. Saved examples work offline.
For surveying, set `LABEL` to your drone's sticker label and confirm the blinking
drone. Capture runs in the notebook after verified firmware stop, with a timed
preparation interval and trace. Saved recordings and simulated outlines work offline.
In the flight notebook, power-cycle after identification and repeat preflight.

## Optional command-line route

From the project root, the same planner can save a disabled course, reference
CSV, summary and preview:

```text
python scripts/plan_trajectory.py --course examples/course.shuttle.synthetic.json --laps 2 --output plans/example
```

Use `--help` for measured surveys and crossing order.
The [planner reference](../../docs/trajectory_construction.md) gives mathematics,
limitations and optional extensions.

The optional [uncertainty notebook](04_extra_uncertainty.ipynb) has an editable
speed policy with fixed simulated motion.
