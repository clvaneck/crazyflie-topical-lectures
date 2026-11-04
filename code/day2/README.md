# Day 2: timestamped sensor fusion

Run `jupyter lab` from the project root and open these in the file browser,
using the Day 1 environment and kernel:

1. [01_tracking.ipynb](01_tracking.ipynb): lecture demonstration and E1.
2. [02_calibration.ipynb](02_calibration.ipynb): E2 stationary IMU calibration,
   onboard KF noise-parameter upload and readback, and a second recording.
3. [03_height_velocity.ipynb](03_height_velocity.ipynb): E3-E7 height/velocity
   filtering with live/saved acceleration and ToF data, or simulation.

Start with each notebook's setup cell. Set `run_id` for the session; results go to
local `data/<run_id>/`. Comparisons get fresh attempt folders; capture cells have
explicit overwrite options. For live capture, enter the
sticker `LABEL` as on Day 1 and confirm the blinking drone. The helper finds
the channel and engages the firmware stop. The numbered
notebooks explain the motor-stop checks, data choices and completion questions.
The notebooks are the working instructions; the [filter reference](../../docs/kalman_filter_practical.md) adds conventions and optional studies.

E2 reads the current `kalman.pNAcc_xy`, `kalman.pNAcc_z`,
`kalman.mNGyro_rollpitch` and `kalman.mNGyro_yaw` values, estimates noise amplitudes
from stationary data in SI units, then uploads them while grounded. Writes require
acknowledgement and readback. These updates last for the powered session.
Compare the second recording using the uploaded noise values and the first
recording's bias estimates. For the flight comparison, follow the notebook's
power-cycle, re-upload and Day 1 preflight sequence.

In E3, complete four mathematical expressions. E4 explores chi-square rejection;
E7 provides a clearance-to-speed policy to edit.
`vertical.py` supplies timestamps, stationary calibration, replay, diagnostics and
recording validation; `vertical_live.py` captures accelerometer/ToF data with
motor-stop verification and a wall-clock bound. `notebook_tools.py` supplies
capture, plots and file handling. `tracking.py` is the E1 reference implementation.
The shared `notebook_radio` uses Day 1's radio helpers. Connection preserves
controller and estimator selection. Each
comparison follows the Day 1 sequence: predict, change one choice, measure,
compare and keep the result with its settings.

From `code/`, the corresponding tests and optional CLI capture are below.
Replace `<discovered-uri>` with the `uri` returned by the notebook's Connect cell.

```text
python -m unittest day2.test_vertical day2.test_tracking day2.test_layout
python -m day2.vertical_live --uri "<discovered-uri>" --output day2/data/vertical-training --prepare-for-handling
```

Follow the [recording protocol](../../docs/sensor_recording_contract.md#vertical-sensor-recordings-for-e3-e7)
before handling. Recorded accelerometer/ToF samples are distinct from onboard
position estimates. Failed or incomplete selections remain errors; simulation
is an explicit choice. E6 validates frozen calibration/settings on independent
input; E7 uses covariance
from the resulting runs.

## Additional six-state experiments

Open [extra_six_state.ipynb](extra_six_state.ipynb) for the optional model and
recording inspection. Its `extra_notebook_tools.py` helpers load the separate
six-state backend. The [six-state reference](six_state_reference.md) documents
CLI commands, model assumptions, output columns and diagnostics.
