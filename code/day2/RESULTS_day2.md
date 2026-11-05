# Day 2 results: Kalman filtering with drone 4

Team branch `caspar-tuning`, drone label **4** (`radio://0/30/2M/E7E7E7E704`), 4 November 2026.
All figures and recordings referenced here are copied into [`results/`](results/) (the working folder
`code/day2/data/` is gitignored). Notebooks are committed with their outputs.

| Exercise | Notebook / file | Data | Status |
| --- | --- | --- | --- |
| E1A particle tracking | [`e1a_tracking.py`](e1a_tracking.py) | simulation | done, incl. both extras |
| E2B position reveals velocity | [`01_tracking.ipynb`](01_tracking.ipynb) | demo | explained, Q not yet added in the notebook |
| E2 IMU calibration + flight | [`02_calibration.ipynb`](02_calibration.ipynb) | live, drone 4 | done, incl. flight |
| E3 height filter | [`03_height_velocity.ipynb`](03_height_velocity.ipynb) | live hand recording | done |
| E4 outlier gate (+ bias state extra) | same | live recording + injected faults | done |
| E5A range dropout | same | live recording | done |
| E5B rest constraint | same | simulation | done |
| E6 frozen settings on fresh data | same | second live recording | done |
| E7 covariance to speed rule | same | E3/E5A covariance | done |

**Day 2 in one sentence:** day 1 was *steering* (how hard to react to an error: PID gains), day 2 is
*seeing* (where am I, and how sure: Kalman filter). Day 3 needs both: gains to follow the course,
covariance to size the margin around each gate.

---

## E1A: Kalman track fit in a parallel-plane detector

Script: [`e1a_tracking.py`](e1a_tracking.py) (`.venv/bin/python code/day2/e1a_tracking.py`, about 1 s).
10 planes at z = 1…10 m, hit resolution σ = 0.01 m, slope kick q = 0.02 per plane (after the measurement),
start x₀, t₀ ~ N(0, 0.1), fit prior diag(1, 1). Vectorised over tracks: the covariance does not depend on
the data, so one C serves the whole ensemble.

**Check against the course reference** (`day2/tracking.py`): covariance at the last plane identical
(max difference 1.4·10⁻²⁰).

| 1000 tracks, σ = 0.01, q = 0.02 | pull mean | pull std | resolution: measured / filter |
| --- | --- | --- | --- |
| x | −0.004 ± 0.031 | 0.992 ± 0.022 | 9.30 mm / 9.38 mm |
| slope | +0.011 ± 0.032 | 1.009 ± 0.023 | 0.0105 / 0.0104 |

- Pulls consistent with N(0,1): the filter's covariance is honest. Error correlation x/slope: measured
  +0.700, from C +0.707.
- Predicted residuals r/√R ~ N(0,1) on every plane. Plane 1 has a huge residual because it measures the
  1 m wide prior.
- **Q ≫ σ:** the track forgets old hits. Position → σ (10 mm, last hit only), slope → √2·σ/d ≈ 0.014
  (last two hits). **Q → 0:** straight-line least squares, slope σ = 0.0011.
- Only 35.6 % of tracks stay inside the 1 m active height. All planes are used as a control, as the reference does.
- **Extra 1, prediction bands:** green bands widen between planes and collapse at each hit
  ([`track.png`](results/E1A/track.png), [`track_large_q.png`](results/E1A/track_large_q.png)).
- **Extra 2, binary pixel readout** (p = 1 cm, V = p²/12): hit residuals uniform with RMS 0.289·p
  (the exercise text says "√12", but it is p/√12). Pull widths stay ≈ 1, but the shape is **not** Gaussian:
  - at q = 0.02 the x pull is flat (excess kurtosis −1.17), because the last hit dominates
  - at q ≈ 0 it is too peaked (+0.7), because the rounding errors along a straight track are correlated

Figures: [`pulls_resolution_residuals.png`](results/E1A/pulls_resolution_residuals.png),
[`resolution_scan.png`](results/E1A/resolution_scan.png), [`pixel_readout.png`](results/E1A/pixel_readout.png),
numbers in [`summary.json`](results/E1A/summary.json).

## E2B: position measurements reveal velocity (01_tracking)

Velocity noise 0.3 m/s per step acts on velocity only, so **Q = [[0, 0], [0, 0.3²]] = [[0, 0], [0, 0.09]]**,
added in the prediction: `C_predicted = F @ demo_C @ F.T + Q`. The velocity band then stays wide and the
estimate follows the measurements more nervously, because the filter knows the speed can change, so old
measurements count less. *Not yet entered in the notebook cell.*

## E2: stationary IMU calibration and a flight with the calibrated values

Run 13:00 (folder `data/e2-20261104-130040`), drone flat on the floor, 60 s at 50 Hz (3003 samples).

| Sensor | mean (bias) | RMS noise | onboard Kalman value | ratio |
| --- | --- | --- | --- | --- |
| acc.x | −0.012 m/s² | 0.011 m/s² | pNAcc_xy = 0.5 | ~45× smaller |
| acc.y | +0.039 m/s² | 0.012 m/s² | pNAcc_xy = 0.5 | ~45× smaller |
| acc.z | 9.799 (g = 9.807, −0.008) | 0.012 m/s² | pNAcc_z = 1.0 | ~80× smaller |
| gyro.x | +0.0017 rad/s (0.1°/s) | 0.0020 rad/s | mNGyro_rollpitch = 0.1 | ~40× smaller |
| gyro.y | −0.0011 rad/s | 0.0028 rad/s | mNGyro_rollpitch = 0.1 | ~40× smaller |
| gyro.z | ≈ 0 | 0.0021 rad/s | mNGyro_yaw = 0.1 | ~50× smaller |

- Uploaded and read back: pNAcc_xy 0.0113, pNAcc_z 0.0121, mNGyro_rollpitch 0.0024, mNGyro_yaw 0.0021.
- Second recording (30 s) with these values: pulls match N(0,1) on all six channels
  ([`pulls_after_calibration.png`](results/E2/pulls_after_calibration.png)).
- The biases are small. acc.y 0.04 m/s² corresponds to about a 0.2° tilt (floor not perfectly level).

**Flight with the calibrated values** (13:04, 0.5 m forward, hold 2 s, back; completed, no abort):
[`flight_calibrated.png`](results/E2/flight_calibrated.png), raw logs in [`results/E2/flight/`](results/E2/flight/).

| | E2 flight (calibrated) | reference: helix flight day 1 (defaults) |
| --- | --- | --- |
| height estimate vs ToF | 3 cm off | 0.8 cm |
| forward move | overshoot 22 cm (0.81 m instead of 0.61), pitch spikes ±6° | max tracking error 12 cm |
| sideways | estimate drifts ~0.9 m at ~10 cm/s (target 0) | max 17 cm |

**Interpretation:** the stationary values are right for a drone *at rest*, but too small for flight.
- Motor vibration makes the accelerometer far noisier in the air.
- `pNAcc` is *process* noise: every unmodelled push (vibration, air, thrust differences), not only sensor noise.
- With values 45× too small, the filter trusts its prediction and corrects too slowly from the flow deck,
  which gives overshoot and drift.

So a stationary calibration gives a good *measurement* noise, but not a usable *process* noise for flight.
Power-cycling restores the defaults.

## E3: height and vertical speed from accelerometer + ToF

Recording `capture-131904` (13:19): drone on a support 8 cm above the floor, 5 s rest, lifted to 14 cm and
back around 6.5–8 s, then still. ToF noise at rest 1.7 mm. Model x = (z, v_z), F = [[1, Δt], [0, 1]],
B = (½Δt², Δt), Q = B σ_a² Bᵀ, H = (1, 0).

**The four formulas** (cell `our_predict` / `our_update`; the built-in check passes):
`x⁻ = F x + B a`, `C⁻ = F C Fᵀ + Q`, `K = C⁻ Hᵀ R⁻¹`, `x⁺ = x⁻ + K r`.

| choice | pull width (≈ 1 wanted) |
| --- | --- |
| **baseline: measured V = 1.7 mm, σ_a = 0.1 m/s²** (chosen) | **1.01** |
| V-small (V × 0.1) | 2.85, overconfident |
| V-large (V × 10) | 0.38, too cautious |
| σ_a = 0.08 / 0.3 / 1.0 | 1.03 / 0.91 / 0.79 |

- Accelerometer integration alone drifts to **73 cm** while the drone sat at 8 cm.
- Differentiating the ToF gives a speed that is noisy by ±0.1 m/s.
- The KF follows the height exactly and gives a smooth speed (+5 cm/s up, back to 0).
- **One update:** a +1 cm residual gives K = (0.99, 1.17 /s), so the height moves +0.99 cm and the speed
  +1.2 cm/s, through the cross-covariance C_vz. The height σ drops from 2.1 cm to 0.17 cm.

Figure: [`e3_comparison.png`](results/E3/e3_comparison.png).

## E4: outlier rejection with a χ² gate

Threshold `chi2.ppf(1 − α, 1)` = 6.63 for α = 0.01. A fake +15 cm reading is injected at t = 12 s.

| | height at 12.00 s | speed |
| --- | --- | --- |
| ungated | 14.4 cm (jumps 6 cm) | +34 cm/s, then overshoots to −9 cm/s |
| gated | 8.0 cm (outlier ignored) | ≈ 0 |

| α | good readings rejected (of 301) |
| --- | --- |
| 0.001 | 2 |
| **0.01** | **8** (≈ 3 expected: tails slightly heavier than Gaussian) |
| 0.05 | 36 |
| 0.2 | 271: runaway, the filter stops listening to the sensor |

Figures: [`e4_comparison.png`](results/E4/e4_comparison.png) (with fault),
[`e4-clean_comparison.png`](results/E4/e4-clean_comparison.png).

### E4 extra: accelerometer bias as a third state

State (z, v_z, b), F = [[1, Δt, −½Δt²], [0, 1, −Δt], [0, 0, 1]], bias random walk 0.01 m/s²/√s.
Test: range sensor off 12–15 s; height error at the end of the gap relative to the full-data filter.

| case | 2-state + gate | 2-state, no gate | 3-state with bias |
| --- | --- | --- | --- |
| recording as is | −2.7 cm (10 rejected) | −2.7 cm | −7.3 cm |
| **+0.05 m/s² offset the rest calibration missed** | **+190 cm** (193 rejected, runaway) | +22 cm | **−7.1 cm** (bias estimate 0.059) |

The bias state costs a little when the 5 s rest calibration was already good, but it prevents the drift and
gate runaway when the offset changes afterwards (temperature, vibration). That is why the onboard EKF
estimates sensor biases. Figure: [`e4_bias_state.png`](results/E4/e4_bias_state.png).

## E5A: range dropout 8–11 s (live recording)

| t | height σ | estimate minus full-data estimate |
| --- | --- | --- |
| before | 0.11 cm | 0 |
| 8.5 s | 0.78 cm | +1.7 cm |
| 9.5 s | 2.9 cm | +4.8 cm |
| 10.95 s (end) | **7.3 cm** | **+10.5 cm** |
| 11.05 s (range back) | 0.17 cm | 0 |

- On accelerometer only, the uncertainty grows quickly (roughly with t², through the speed uncertainty).
  The error stays within about 1.5σ, so the filter is honest about being unsure.
- The first returning reading has gain 1.00 (height) and 0.46 /s (speed): one reading snaps it back.

Figure: [`e5-dropout_comparison.png`](results/E5/e5-dropout_comparison.png).

## E5B: zero-velocity ("I am standing still") update (simulation)

H = (0, 1), m = 0, σ = 0.02 m/s. Known rest 16–19 s (true rest), false rest 8–9 s (true speed 3–5 cm/s).
Notebook fix: E5B now uses the simulation's own range noise (5.3 mm). Our recording's 1.7 mm made the filter
overconfident on simulated data (pull width 2.8), which blurred the effect.

| run | speed error in the window | speed σ claimed | worst error/σ |
| --- | --- | --- | --- |
| no constraint, 16–19 s | 0.85 cm/s | 1.23 | 1.6 |
| **known rest**, 16–19 s | **0.43 cm/s** | 0.89 | 1.0 |
| no constraint, 8–9 s | 1.06 cm/s | 1.23 | 1.9 |
| **false rest**, 8–9 s | **2.90 cm/s** | 0.90 | **4.0** |

Whole-run velocity RMSE: 0.94 / 0.87 / 1.21 cm/s (none / known / false); 95 % coverage 1.00 / 1.00 / 0.93.
**Interpretation:** a true "standing still" halves the speed error for free. A false one makes the filter
*confidently wrong* (4σ). Use it only when you are sure, e.g. on the ground before take-off.
Figure: [`e5-rest_comparison.png`](results/E5/e5-rest_comparison.png).

## E6: frozen settings on a fresh recording

Validation recording `validation-capture-01`: a second hand recording, lifted 8 → 15 cm at 12–14 s and held
at ~13 cm. Same frozen Q/V and training calibration.

| | value |
| --- | --- |
| pull width | **1.02** |
| pull mean | 0.06 |
| rejected | 0 |
| ToF noise of the new recording (own calibration) | 1.44 mm (frozen V: 1.68 mm) |

- **Prediction checked:** "pull width ≈ 1 on new data". Confirmed.
- Accelerometer integration alone again drifts (to −7 cm).
- **Limitation:** both recordings are by hand with motors off. In flight, vibration makes V and Q larger (see E2).
- **Verdict:** the settings transfer to new data from the same drone; flight still needs its own check.

"Selecting data" means choosing the validation set: a fresh live recording (chosen; the strongest), someone
else's recording, or the simulation (code check only). It must differ from the training data; the notebook
rejects identical input hashes.

Figure: [`e6_comparison.png`](results/E6/e6_comparison.png).

## E7: from covariance to a gate decision

Clearance = radius 0.50 − radial offset 0.03 − drone 0.10 − tracking 0.05 − 2.5·σ_worst-in-plane.
Horizontal σ assumed 0.04 m (this filter only estimates height). Height variance taken at the end of the E5A gap.

**Speed rule** (replaces the fixed 0.8 m/s):
- clearance ≤ 0 → 0
- otherwise min(clearance / (2·lag), √(2·a·d), 0.8), with lag **0.50 s, measured for drone 4** (5 Nov:
  1/posCtlPid.xKp, firmware xKp 2.0, `code/day1/HANDOVER.md`; first written with an assumed 0.25 s),
  braking a = 1 m/s², approach d = 0.5 m

The lag term spends at most half the clearance on following error; the braking term means it can still stop in the run-up.

| situation | gate | height σ | clearance | speed, lag 0.50 s | (was, lag 0.25 s) |
| --- | --- | --- | --- | --- | --- |
| range available | vertical opening | 0.11 cm | 0.220 m | 0.22 m/s | 0.44 m/s |
| range available | horizontal opening | 0.11 cm | 0.220 m | 0.22 m/s | 0.44 m/s |
| **3 s range gap** | **vertical opening** | 7.4 cm | **0.134 m** | **0.13 m/s** | 0.27 m/s |
| 3 s range gap | horizontal opening | 7.4 cm | 0.220 m | 0.22 m/s | 0.44 m/s |

With the measured lag every speed halves; the braking (1.0 m/s) and 0.8 m/s caps do not bind. The notebook
constant is updated; its E7 cell output still shows the 0.25 s run (re-running would mean re-running the
notebook, whose E6 cells fly live).

- The **horizontal opening is unaffected**: for it, height is the normal direction, and uncertainty along the
  normal does not reduce in-plane clearance.
- Chain: missing measurement → uncertainty grows → less clearance → lower speed. At σ_z ≈ 13 cm the
  clearance reaches 0 and the rule stops.
- This is the same margin formula the day 3 planner uses (`survey + tracking + k·σ_plane`).

---

## Changes made to the course material

- **`02_calibration.ipynb`**
  - `LABEL = 4`; a fresh `run_id` per session (`e2-<date>-<time>`)
  - Connect uses the shared parameter cache `code/day2/cache` (see "Problems" below)
  - before the flight, the drone is restarted over the radio to clear the motor stop, checked
    (`stabilizer.stop = 0`), and the calibrated values are uploaded again
  - the flight cell asks for `yes` first
  - outputs are from the 13:00 run: the analysis cells were recomputed from the saved recordings, and the drone
    cells contain the logged output of that run
- **`03_height_velocity.ipynb`**
  - `LABEL = 4`, shared cache, reuses the E3 recording, E6 records live
  - the four E3 formulas; `chosen_name = "baseline"` with reasoning
  - α reasoning for E4; the E4 bias-state extra cell
  - E5B uses the simulation's own range noise
  - E7 speed rule
- **`code/day1/crazyflie.py`**
  - The macOS abort window crashed on every call: `new_manager` needs an int figure number. It then fell back
    to the input box, which in VS Code answers by itself and aborted flights right after take-off. Fixed.
  - The input-box fallback now ignores an answer within 1 s of opening (at most 3 retries).
  - Day 1 tests: 28/28 pass.
- **New:** `e1a_tracking.py`, this file, `results/`, `cache/` (parameter TOC of the drone firmware).

## Problems we ran into (and the fix)

| symptom | cause | fix |
| --- | --- | --- |
| drone "locked" after E2 recordings | intentional `stabilizer.stop = 1` safety lock during handling | radio restart (`crazyflie.reboot`), now automatic before the flight |
| flight aborted right after take-off, "deck not delivering", motors never spun | abort-window crash, then the VS Code input box returned instantly, which counted as an abort | `crazyflie.py` fixes above |
| Connect timed out after 30 s | a fresh empty cache per run folder, so the full ~900-entry parameter/log list was downloaded over a busy radio | shared cache `code/day2/cache` |
| Connect timed out once | notebook and terminal used the radio at the same time | one radio client at a time; close other kernels |
| "Too many packets lost" in a recording | radio interference (drones 0 and 5 of other groups nearby) | repeat the recording |
| E3 recording rejected ("Motion violates the level, gentle-motion model") | jolts > 0.15 g when picking up/setting down, and the drone resting 11° tilted | lift the support instead of the drone, hold still in the air at the end |

## Reproduce

```bash
.venv/bin/python code/day2/e1a_tracking.py                 # E1A, figures to code/day2/data/e1a/
cd code/day1 && ../../.venv/bin/python testing/test_crazyflie.py   # 28/28
```

The E3–E7 analysis runs offline from `results/recordings/` by pointing `captured_directory` (E3) and
`validation_directory` (E6) at copies of those recordings, laid out as `raw.csv` + `recording.json` folders.

## Open

- E2B: enter Q in the `01_tracking.ipynb` demo cell (answer above).
- Optional `extra_six_state.ipynb` not done.
- Done 5 Nov: the E7 lag is now the measured 0.50 s of drone 4 (speeds halved, see E7).
- For day 3: flight-level process noise has not been measured. Keep the firmware defaults rather than the
  stationary E2 values.
