# Day 2: recording sensor data and replaying it

E2 uses stationary IMU logs for calibration. E3-E7 use timestamped accelerometer and downward ToF readings for a two-state height/velocity experiment. Keep observations, source units, log-time conventions, reference conditions and any existing processing with each recording. One training capture serves E3-E5; E6 uses different input with frozen calibration and Q/V. If live acquisition stalls, a checked recording or matching simulation lets you continue; spend at most five minutes troubleshooting.

All hand-held acquisition uses the [firmware-stop preparation](#prepare-the-drone-for-hand-held-recording). It sends no flight commands.

## Vertical sensor recordings for E3-E7

Use **Connect** and the bounded capture cell in
[03_height_velocity.ipynb](../code/day2/03_height_velocity.ipynb). Select your
sticker `LABEL` there; capture reuses the returned URI. The underlying
`day2.vertical_live` worker sets and verifies `stabilizer.stop=1` and monitors
supervisor lock and zero motor outputs. Wait for STOP VERIFIED, keep level and
stationary for five seconds, then wait for Calibration check passed before gentle
up/down motion over the same fixed flat floor. Hold still again by 15 seconds;
capture ends at 20 seconds. The stop remains engaged. Put the drone down and
power-cycle before flight/preflight checks.

The notebook stops the capture worker after the requested duration
plus 30 seconds (50 seconds by default), including connection and parameter
download. A timeout or interruption preserves partial files for diagnosis;
incomplete recordings cannot be selected for analysis. Put the drone down and
power-cycle before flight, then choose a checked recording or simulation to continue.

For the initial rest, use a stable support with an opening beneath the sensor, 5-20 cm above the floor. This keeps the floor range valid before picking up the stopped drone.

Live vertical captures retain the full IMU and downward ToF sensor record: gyro
channels in degrees per second, acceleration channels in g, and `range.zrange`
in mm. The vertical model uses acceleration and range; gyros remain in the raw
file for reuse. No onboard state estimate or quaternion is needed. The raw CSV
header is:

```text
time_s,gyro.x,gyro.y,gyro.z,acc.x,acc.y,acc.z,range.zrange
```

New captures use `vertical-sensors-v2`; the loader also accepts existing
`vertical-sensors-v1` files with the five-column vertical-only header. The
metadata schema for new captures includes:

```json
{
  "format": "vertical-sensors-v2",
  "source": "IMU and downward ToF sensor logs",
  "status": "complete",
  "columns": ["time_s", "gyro.x", "gyro.y", "gyro.z", "acc.x", "acc.y", "acc.z", "range.zrange"],
  "channels": ["gyro.x", "gyro.y", "gyro.z", "acc.x", "acc.y", "acc.z", "range.zrange"],
  "vertical_analysis_channels": ["acc.x", "acc.y", "acc.z", "range.zrange"],
  "raw_sha256": "SHA-256 of the exact raw.csv bytes",
  "timing": "Unwrapped firmware log times in seconds; sensor delays not compensated.",
  "known_rest_intervals_s": []
}
```

The helper also records URI, channels, model assumptions and the firmware-stop check. Add observed rest intervals; the motion prompts alone are not evidence that the drone stayed still. Preserve partial captures; failed/incomplete status and hash mismatches are rejected. UTF-8 BOM inputs are accepted.

Measurement updates use each recorded timestamp after propagation over its measured dt. Acceleration approximates the preceding interval. IMU and range share one log block; its timestamp is a logging-time proxy rather than proof of synchronous sensor acquisition. The loader rejects duplicate/reversed times and gaps above 0.30 s.

Log variables use their firmware TOC types, not assumed types. For the live
sensor block this is six 4-byte IMU floats plus the 2-byte `uint16_t` range,
exactly 26 bytes.

Subtract the initial stationary Z specific-force mean, including gravity, then multiply by 9.80665. Fit it before filtering, and retain it unchanged on validation input. The first five seconds must be consistent with level rest and a stable range. The teaching model accepts positive floor ranges up to 4 m, including measurements below 0.02 m; zero, negative, non-finite, and above-4 m returns are invalid. These are experiment checks, not manufacturer performance specifications. Invalid later ranges skip that update. Off-axis acceleration and upside-down motion fail the level/gentle-motion checks. Subtle tilt, sensor latency and calibration uncertainty remain model limitations; do not hide them by changing Q/V.

### Vertical analysis outputs

Each notebook comparison saves `comparison.png`, timestamped run CSVs and
`settings.json` in a fresh directory. CSV fields include height/velocity,
C_zz/C_zv/C_vv, predicted residual and variance, chi-square, prospective range
gain, acceptance and stationary-update flag. Settings retain calibration,
input hashes and model assumptions. Real recordings support consistency checks;
truth-based RMSE and coverage are available with simulation.

## Prepare the drone for hand-held recording

For notebook capture or a survey command printed by the notebook:

1. Put the drone on a stable surface, with hands and loose objects clear of its
   propellers. Close cfclient and any other drone-control scripts. Leave the
   drone on the surface until verification completes.
2. Use the notebook's **Connect** cell with your sticker `LABEL`, and confirm
   the blinking drone. Enable the capture switch for the chosen recording;
   the tool reuses the selected address. For surveying, run the printed command
   and confirm `STOP AND SURVEY`.
3. The tool sets `stabilizer.stop=1`, then requires a parameter readback of `1`,
   a fresh supervisor status showing locked and neither armed nor able to fly,
   and four zero motor outputs. Verification has a five-second timeout after
   connection/parameter loading. If it fails or channels are missing, leave the
   drone down and use a checked recording or simulation.
4. Wait for **STOP VERIFIED** in the notebook/terminal output before picking
   up the drone. Use secure handling that prevents contact with propellers.
   Keep the same powered session throughout capture. The Day 2 helper
   also checks zero motor outputs continuously during acquisition.

`stabilizer.stop=1` requests the firmware's emergency stop. On the lecture
firmware this latches the supervisor's locked state until reboot; setting the
parameter back to `0` is not the recovery procedure. After finishing the
motor-free work, place the drone down and power-cycle it before returning to
the normal preflight checks for flight. Repeat preparation if you restart it
before another recording. There is no need to save this parameter persistently.
See Bitcraze's [stop parameter implementation](https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/supervisor.c)
and [emergency-stop behaviour](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/supervisor/#emergency-stops).

**For the optional cfclient logging route:** first complete **Connect** in the
Day 2 notebook. Its connection closes with the firmware stop engaged. Without
rebooting, select that same discovered URI in cfclient and leave the stop engaged.
Check four zero motor outputs before recording. Power-cycle on the surface and
repeat preflight checks before returning to flight.

## Additional sensor and six-state recording studies

The sections below support E2 inventory/reference checks and optional multi-sensor studies. Their prepared position/acceleration format is separate from the core vertical sensor recordings.

## Before recording

Teams can share checked recordings across the exercises. For the
[E2 core](sensor_bias_calibration_exercise.md#e2-core-45-minutes), provide a
60-second stationary IMU calibration recording and a separate 60-second
validation recording, including gyro and acceleration with their reference
conditions, units and existing corrections. The core vertical experiment checks ToF variability in its own initial rest interval. The sensor-specific studies beyond this pair
are optional; they do not add acquisition or analysis requirements to E2.
If a checked pair is unavailable, E2C's `use_simulated_imu = True` creates
`E2-simulated/calibration.csv` and `validation.csv` inside the notebook run folder.
Their independent noise permits the same held-out analysis, labelled as simulation;
the actual drone's calibration remains outstanding.
Before E2, check its sample and metadata together: channels/firmware, world/body
frames, geometry, reference conditions and calibration/held-out split. Teams can
build the channel inventory from cfclient and share the work. Missing sensors or
references remain labelled limitations, not extra instructor data-production tasks.
Before an optional six-state study, check the prepared files below with the supplied helper once and agree
which channels they cover. Do this before spending the acquisition slot. Never
substitute fused position for a raw Lighthouse observation or simulator truth.

The [synthetic CSV](../examples/sensor-events.synthetic.csv) demonstrates the
normalized row format only; it is not a calibration, motion, or validation dataset.

## cfclient channel inventory and logging blocks

Use the [Day 1 logging procedure](day_1_baseline_checks.md#b3-create-and-save-cfclient-logs)
to create these **additional**, separate blocks. Check every name/type in
**[Log TOC](glossary.md#telemetry-and-log-toc)**
for the actual firmware. The suggested periods specify logging frequency;
sensor-acquisition timing is recorded separately. Enable only the blocks needed for
the current experiment; retain original CSVs and configuration definitions.

| Block and suggested period | Common channels | Meaning and conversion for normalized replay |
| --- | --- | --- |
| [IMU](glossary.md#imu), 20 ms | `acc.x`, `acc.y`, `acc.z`, `gyro.x`, `gyro.y`, `gyro.z` | Six floats, 24 bytes. [Specific force](glossary.md#specific-force) in body axes: g to m/s2 using 9.80665. Gyro: degrees/s to rad/s. These channels may already include onboard bias compensation/filtering. |
| Orientation, 20 ms | `stateEstimate.qx`, `.qy`, `.qz`, `.qw` | Four floats, 16 bytes; unit [quaternion](glossary.md#quaternion-and-so3) ordered x,y,z,w. Derived onboard attitude, sharing inputs with other onboard estimates. Verify body-to-world convention against the hardware/frame note. |
| Position cross-check, 50 ms | `stateEstimate.x`, `.y`, `.z` | Three floats, 12 bytes; world metres, already fused. Store as `position_onboard`, never raw Lighthouse or truth. |
| Range, 50 ms | `range.zrange` | Downward sensor range, commonly mm; convert to metres. Record invalid-return rules from the driver/TOC and reference surface. Do not assume every number is valid or vertical height. |
| Pressure, 50 ms | `baro.pressure`, `baro.temp`, `baro.asl` where available | Pressure commonly hPa, temperature degrees C, and pressure-derived altitude metres. Convert pressure to Pa if storing the normalized `pressure` event; label altitude as derived. |
| Flow, 20 ms initially | `motion.deltaX`, `motion.deltaY`, `motion.squal`, `motion.motion`, optionally `motion.shutter` | Pixel/image increments and quality/status. Velocity conversion requires timing and geometry; see below. Preserve original values. Use stored types or verify converted block size stays below 26 bytes. |

Slash/dot shorthand in the table means separate complete variable names.
If a channel is missing, label it unavailable and choose a documented
supported subset or supplied dataset. Do not invent a log variable, silently fill
missing values with zero, or claim an absent sensor was calibrated.

**Flow timing caveat:** the normal log samples the latest flow registers. Logging
slower than the driver can skip increments; repeated snapshots are not new
measurements. The interval between log packets is not necessarily the integration
interval of that increment. The firmware's axis swap/sign convention and angular
motion also matter. Unless the supplied dataset/adapter preserves true increment
duration and source-channel details, use these logs for stationary offset/quality
studies, not quantitative velocity fusion. Flow fusion in a six-state study is optional when
those timing and conversion requirements cannot be met; the simulated flow
experiment remains available. See the
[Flow measurement model](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/sensor-to-control/state_estimators/#measurement-models).

**Lighthouse caveat:** `stateEstimate.*` is not a raw sweep-angle record. Raw
angle acquisition requires a specifically supported capture path and calibration/
geometry metadata, supplied by the instructor if used. The six-state replay may use
onboard position as an explicitly **[cascaded](glossary.md#cascaded-estimation)** measurement, with shared-IMU
correlations discussed; it is not independent raw Lighthouse/IMU fusion.

## Normalized event CSV and metadata

For the optional adapter extension, teams retain original cfclient files and implement a converter to
this interchange format (or an equivalent documented format with an explicit map):

```text
time_s,sensor,x,y,z,w,flow_dt_s,valid
```

One row is one new observation at that channel's measurement/log time. `valid`
is `1` or `0`; unused components are empty. Preserve rejected/invalid rows for
diagnostics, but do not feed invalid/non-finite data into the filter. Supported
event meanings are:

| `sensor` | Components and frame |
| --- | --- |
| `specific_force_body` | x,y,z in m/s2; gravity still present as specific force at rest. |
| `angular_rate_body` | x,y,z in rad/s, body frame. |
| `attitude_quat_onboard` | x,y,z,w unit quaternion, documented body-to-world convention; fused/derived. |
| `position_onboard` | x,y,z in world metres; fused/derived. Independent external position, if available, uses a separate documented event type. |
| `flow_pixels` | x,y image increments; `flow_dt_s` required for quantitative fusion, with source axes/quality in metadata or retained original columns. |
| `range` | x = sensor-to-surface distance in metres; not world Z. |
| `pressure` | x = pressure in Pa; optional for this starter, which has no barometer-update callback. |

A companion `recording.json` records dataset ID, source files/hashes, firmware,
decks, sensor availability, geometry/frame definitions, exact source-channel to
event mapping, units/conversions, timestamp kind and units per block, expected
rates, known [latency](glossary.md#latency-jitter-and-dropout)/integration times, invalid rules, existing onboard processing,
and any offboard corrections. Include reference-condition intervals, external
survey/reference uncertainties, calibration/development/held-out split and IDs.
Unknown timing/calibration facts are explicitly unknown, not estimated silently.
Keep raw observations, normalized data and calibration output separate.

Standard cfclient CSV `Timestamp` is the firmware log timestamp in milliseconds,
not host callback time and not necessarily the physical sensor-acquisition instant.
It wraps after 2^24 ms; unwrap within a continuous connection and split restarts
into new sessions. Convert to seconds relative to a common documented session
origin. Do not independently zero each block and destroy their offsets. Preserve
duplicates/gaps in raw files and define treatment in the converter. A reconnect
does not automatically prove a continuous clock. Independently timestamped external
reference data need a documented clock alignment and its uncertainty.

Prefer true acquisition times when available. If only firmware log times exist,
label them **logging-time proxies**, document unknown sensor age and limitations,
and use sufficiently slow motions or supplied acquisition-timestamped recordings.
Do not assert exact acquisition-time or delay compensation from these CSVs alone.

## Live six-state session and replay fallback

### Live capture using the supplied helper

The [six-state examples](../code/day2/six_state_reference.md)
use the capture helper in the Day 2 notebooks with the URI already returned by
**Connect**.
The helper reads three separate 50 ms log blocks: acceleration plus onboard
position (six floats, 24 bytes), [quaternion](glossary.md#quaternion-and-so3)
(16 bytes), and motor outputs
(16 bytes). No gyro, range, flow or raw Lighthouse-angle fusion is claimed.
Close cfclient; only one client should use the radio connection.

The optional `day2.extra_notebook_tools.capture_live()` wrapper runs this six-state
logger from a notebook. It accepts 1-120 seconds of data and allows 30 additional
seconds for connection and saving. Set `ready_for_motor_stop=True` after the
surface, propeller-clearance and radio-client checks above. Only completed,
validated recordings are returned. Interrupting the kernel stops the child;
partial files remain available for diagnosis. Choose a fresh output directory
and select any fallback explicitly.

`raw.csv` preserves received values, original firmware timestamps and host receive
times. The helper unwraps 24-bit firmware log milliseconds and buffers arrivals
for 150 ms to reduce callback reordering. It skips duplicates/late events with
a recorded count; it does not claim arbitrary-delay tolerance. Each accepted
acceleration/position packet uses the latest causal quaternion and zero-motor
snapshot, each at most 150 ms old. Stale dependencies or a sample gap over
300 ms after startup stop capture. Total telemetry loss also stops capture.

Body specific force in g, optionally minus a justified additional body-axis bias,
is rotated with the onboard body-to-world quaternion (x,y,z,w), multiplied by
9.80665 and combined with world gravity `[0,0,-9.80665]`. Quaternion norms are
checked and normalized. Current acceleration is a held-input approximation for
the preceding interval; packet timestamps are logging-time proxies, not exact
independent sensor times. No hidden interpolation or sensor-delay compensation
is performed. Verify stationary acceleration at level and tilt before accepting
data. Saved `prepared.csv` and `recording.json` use the replay format below;
both configurations therefore reproduce the live numerical results offline.
Metadata includes input hashes, channels, conversion/correction, skipped-event
count and completion/failure status. Partial data survive ordinary errors or
Ctrl+C; an abrupt process/power loss may leave incomplete metadata.

The default extra accelerometer correction is zero. E2 values are not silently
loaded; `--acc-bias-g` requires a separately justified correction in g. Do not
subtract existing onboard compensation twice. The only parameter written by
capture is `stabilizer.stop=1`, which remains engaged on success, error or
interruption. Capture never selects an estimator/controller, arms or sends
flight setpoints. Keep clear of propellers even after lock verification.

### Prepared-data fallback

For a six-state replay, select `data/e6-prepared.csv` and `data/e6-recording.json`
relative to the package root, plus original data or stable hashed locations and
a preparation/calibration map. Include labelled rest and slow motion (and a
position gap if available), using real E2/instructor data. Preparation happens
before the slot: frame/gravity conversion, clock normalization/alignment and any
resampling are **not** participant core tasks. Test the helper on those exact
files and retain known-good outputs. A 30-second rest/slow-motion recording can
serve several six-state comparisons. Label its position source explicitly.

The helper's CSV columns are:

```text
time_s,ax_world_m_s2,ay_world_m_s2,az_world_m_s2,px_m,py_m,pz_m
```

Each row supplies world-frame acceleration, gravity already removed, for the
interval **ending** at `time_s`, and an optional position observation at that
time. The first row establishes time; no propagation precedes it. Times must be
finite and strictly increasing in one documented clock. All three position fields
are present together or all blank; blanks mean no update, not zero position.
All other numeric inputs must be finite. Invalid/reordered/wrapped original data
require documented preparation, not silent acceptance. This restricted format
is intentionally different from the asynchronous raw-event format.

Six-state capture, experiment comparisons and replay reject sample gaps above 0.30 s before
propagation. Choose a continuous checked recording or the simulated fallback
when a gap fails this check; an acceleration snapshot cannot describe a long outage.

For six-state comparisons, the first available position initializes the estimate
once, with zero initial velocity and the configured prior variance. Leading
rows without position are omitted from analysis; their count and initialization
time are recorded in metadata. The original file and time origin are preserved.
At least one position observation is needed to anchor these comparisons.
Plots, replay outputs and intervention metadata retain the file's `time_s` values.
Experiment windows use elapsed time from the file's first sample, recorded as
`analysis_time_origin_s`: the 7-10 s dropout and 12 s outlier occur at file times
107-110 s and 112 s, respectively, when the first timestamp is 100 s.

Metadata retains dataset identity, source hashes, channel/calibration map,
frame/time/shared-information caveats, preparation/resampling method and labelled
rest/motion/gap intervals. It additionally contains this object (within a JSON document):

```json
{
  "prepared_replay": {
    "acceleration_frame": "world",
    "gravity_removed": true,
    "acceleration_interval": "preceding",
    "position_source": "onboard"
  }
}
```

`position_source` is `onboard` for fused positions or `external` for a documented
independent observation, not automatically noise-free truth. Onboard attitude/
position makes the replay cascaded. The helper uses the existing six-state filter;
it neither calibrates raw sensors nor fuses raw flow/range/Lighthouse angles.
Its uncertainty bands are model predictions, not validated accuracy claims.
Keep both replay outputs and their interpretation for this optional comparison.

For an installation/format smoke test only, run from an activated environment:

```text
cd code
python -m day2.replay ../examples/replay.synthetic.csv --metadata ../examples/replay.synthetic.json --output ../E6/format-check
cd ..
```

These tiny synthetic values check the file format. For a complete controlled
motion comparison without hardware or shared files, run from the package root:

```text
cd code
python -m day2.experiments --study single --export-prepared --output ../E6/cooked
python -m day2.replay ../E6/cooked/prepared.csv --metadata ../E6/cooked/recording.json --output ../E6/cooked-baseline
python -m day2.replay ../E6/cooked/prepared.csv --metadata ../E6/cooked/recording.json --position-std 0.12 --output ../E6/cooked-comparison
cd ..
```

The exported metadata identifies these data as synthetic. This fallback supports
the filtering concepts, with real-data calibration/validation left outstanding.
It does not turn simulated results into evidence about the team's hardware.

## Minimum six-state replay and callback mapping (optional adapter extension)

A supported raw-input adapter recording contains body specific force, compatible onboard
orientation and onboard position over rest and slow hand motion, plus metadata.
Gyro is required for the E2 IMU calibration study; range/flow/pressure studies use
their actual installed sensors or supplied data. A six-state adapter may omit unsupported flow,
raw Lighthouse or barometer fusion and must identify that limitation.

| Pipeline entry | Adapter responsibility |
| --- | --- |
| `on_imu(time_s, acceleration_world)` | Apply justified calibration, rotate specific force with compatible orientation, then add world gravity `[0,0,-9.80665]` to obtain translational acceleration. Do not subtract body Z alone while tilted. |
| `on_position(position_m)` | Supply explicitly labelled world position; using `position_onboard` makes this cascaded estimation. |
| `on_flow_velocity(velocity_xy_m_s)` | Only after a justified pixel-duration/height/attitude/gyro conversion to world velocity. Otherwise omit this update. |
| `on_range(height_m)` | Only after converting range to world Z using known surface elevation, sensor offset and attitude assumptions; otherwise omit or extend the model. |
| `on_stationary()` | Conditional [ZUPT](glossary.md#zupt) from validated stationarity logic; never inject continuously during motion. |

`pipeline.py` supplies the update callbacks. The adapter schedules them: before
each update, propagate to its timestamp with the appropriate held process input;
do not double-propagate when the next IMU sample arrives. Define an ordering for
equal timestamps and buffer/reject late observations explicitly. The adapter
extension tests should cover duplicate/out-of-order times, missing blocks, invalid returns,
clock wrap and rest-to-motion transitions without sending any motor commands.

**Additional extension deliverables:** converter/replay command, original and normalized data,
`recording.json`, calibration reference, callback mapping/timing tests, state and
sensor-specific predicted residual plots, and a statement of raw/derived dependencies
and unidentifiable quantities. Replay never qualifies a hardware controller.
