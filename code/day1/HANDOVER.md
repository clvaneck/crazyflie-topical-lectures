# Handover: drone 4 calibration and tuning (state 5 Nov 2026, ~09:55)

For the next agent who takes over from here. Read this file first, then the docstring at the top of
`drone_pipeline.py`. The visual version of this state is `drone_roadmap.html`, also published (public) at
https://claude.ai/artifact/S186mYQyL8f8hYWN44vBrY.

## Rules (non-negotiable)

- **Never fly without Niels's explicit go for that specific flight.** One go per flight. Before asking, say
  what the flight does: arm, take-off height, path or step, fence, how to abort. Ground commands need no go:
  `preflight`, `plan`, `table`, `key_test`, `abort_test`, `tune ... model=True`, `crazyflie.py status / flow / params`.
- **Only drone label 4, `radio://0/30/2M/E7E7E7E704`.** Other groups' drones are in radio range: never connect
  to them. Another group once connected to our address and flew/crashed our drone. If the drone does something
  nobody commanded, abort and check.
- Every flight: pilot clicked into the abort window (Enter or Esc lands), a second person watching, nobody
  within arm's length, 1.5 m clear around it.
- Commit or push only when Niels asks. In this repo (team fork `clvaneck/crazyflie-topical-lectures`) Niels
  has no push rights: push to the `fork` remote (`NielsterLinden/crazyflie-topical-lectures`) and open a PR
  into `caspar-tuning`, as PR #1 and this branch did.

## Setup

```bash
cd code/day1
PY=../../.venv/bin/python        # the course venv: Python 3.12, built by `bash scripts/setup-unix.sh`
# on Niels's laptop this clone has no venv; use the GitLab course clone next to it:
PY="../../../crazyflie-topical-lectures/.venv/bin/python"
$PY testing/test_drone_pipeline.py   # dry run on the course's fake drone, ~40 s: "all checks passed"
```

Until now the pipeline ran in Niels's GitLab course clone (`crazyflie-topical-lectures/code/day1`). This
branch copies the code and all of today's data into the team repo. Use this copy from now on, so the data stays
in one place.

## Where we are

Calibration of drone 4 (flow deck, over carpet), in `data/drone_4_params.json`:
tau 0.13 s, dead time 0.18 s, noise bounds |Kp| < 2.1, |Kd| < 2.9; hover noise x 10 mm, y 24 mm.

| Run (`data/flight_4_<run>_*.csv`) | What | Result |
|---|---|---|
| 1533, 1535 `firmware_fwd` | firmware path flights | 1533 flew at 95 deg yaw and went sideways: the yaw lesson below |
| 1537, 1546 `helix` | `extra_drone_helix` notebook runs | not part of this pipeline |
| 1520 `firmware_hold` | hover over the plain floor | **invalid**: drifted ~4 m, the flow estimate saw 15 cm |
| 1540 `firmware_hold` | hover over the carpet | near perfect: std x 9.6 mm, y 2.7 mm |
| 1543 `identify` | two 0.2 m/s velocity steps | the calibration above |
| 1545 `kp0.7` | `tune -0.7` | rise 3.40 s (model 2.88), no overshoot, mean error last 2 s 6.7 cm |
| 1546 `kp1.4` | `tune -1.4` | rise 2.34 s (model 1.86), overshoot 2.6 cm, mean error 2.5 cm, 37 % at the 0.3 m/s clamp |
| 0936 `kp2` (5 Nov) | `tune -2.0` | rise 2.28 s, overshoot 4.7 cm, max error last 2 s 10.7 cm, 46 % at the clamp |
| 0937 `kp2_kd-0.3` | `tune -2.0 -0.3` | rise 2.12 s, overshoot 3.5 cm, max error last 2 s 6.7 cm |
| 0939 `kp2_kd-0.3_ki-0.1` | `tune -2.0 -0.3 -0.1` | rise 1.98 s, overshoot 7.2 cm, ends 2.1 cm past: worse |
| 0941 `kp2_kd-0.3` | repeat | rise 2.00 s (model 1.86), overshoot 4.0 cm, max error last 2 s 4.5 cm: confirms 0937 |
| 0943 `keyboard` | `keyboard speed=0.1` | flown forward to the 1.5 m fence, which landed it; coasted 0.2 m, touched down at x 1.71 m |
| 0947 `keyboard` | `keyboard speed=0.2` | full 60 s, to x 0.6 m, y 1.2 m, height 0.49–0.59 m, yaw ±17°. Estimate after touchdown runs off (17 m): ignore |

The model (`plan`) ranks Kp −2.0 best (rise 1.78 s, overshoot 1.7 cm). −4 and −8 exceed the noise bound.
Measured runs are about 0.5 s slower than the model, partly because of the V_MAX = 0.3 m/s clamp.

**Best gains, saved with `set_best`: Kp −2.0, Kd −0.3, Ki 0.** Kd +0.3 was not flown (model: 5.7 cm overshoot);
Kd −0.6 is overdamped in the model. The card under the drone fixed the ground drift on the carpet (preflight OK every time).

`params`: 335 parameters, none stored on the drone, so posCtlPid.xKp is the default 2.0 (`data/params_4.csv`).
`handover`: following lag 1/xKp = 0.50 s, against the 0.25 s E7 assumed: 15 cm behind at 0.3 m/s, 25 cm at 0.5 m/s.
Race factors 0.9–1.1 give 0.56–0.45 s.

Yaw drifts in flight: 0 → +15° in 0936 (median 7.8°), y drifted 20 cm during an x step. Under the 10° warning in the
other runs, but it matters for the race (body-frame setpoints).

Battery: ~3.67 V after today's six flights. **Recharge before the race** (needs ≥ 4.10 V at rest).

## Next steps, in order

Steps 1–8 of the old list (recharge, card, tune series, `set_best`, `params`, `handover`, first keyboard flight)
are done, see above. Left:

1. Done: Day 2 E7 uses the measured 0.50 s lag (`code/day2/RESULTS_day2.md`, notebook constant): speeds halve
   to 0.22 m/s, 0.13 m/s in a 3 s range gap. The notebook's E7 output cell is still from the 0.25 s run.
2. Keyboard flown at 0.1 and 0.2 m/s (cap 0.3). Each flight needs a go. Off the card, preflight refuses on the
   carpet: put it back on the card. It coasts ~0.2 m past the fence while landing.
3. Day 3 race:
   - The runner is not in the repo: its path, Python and directory come from the run card on the shared flight
     laptop. The repo has `code/day3/flight.py` (the launcher) and the bounded tuning file.
   - Gate the race on `drone_pipeline.py preflight vbat_min=4.10`.
   - Check the floor texture in the race area first (see below).

## Pitfalls that cost a flight today

- **Floor texture.** The flow deck needs texture. Over the plain floor the drone drifted 4 m while the estimate,
  and so the 1.5 m fence, saw almost nothing. Before flying on any new floor, run
  `$PY crazyflie.py flow --label 4`, hold the drone ~0.5 m up and move it around. Surface quality should be
  about 100; the carpet gives 110–130.
- **Carpet pile on the ground** blinds the height sensor at rest: preflight then fails "on the ground". Put a
  card under the drone.
- **Supervisor lock after a crash.** `preflight recover=True` and a power cycle did not clear it.
  `$PY crazyflie.py reboot --label 4` (a soft reboot over the radio) did. Right after the reboot, "is armed:
  yes" is normal, and preflight accepts it.
- **USB attached.** While charging, the voltage reads high. Preflight requires power state "in use".
- **Yaw.** `crazyflie.fly()` turns world-frame errors into body-frame velocity setpoints, which is only right
  while the drone faces +x. Preflight refuses |yaw| > 10 deg after the estimator reset.
  - Open question: the yaw estimate swung −20..+15 deg in the carpet hover, although the drone looked steady.
    Watch `stabilizer.yaw` in the next runs. `_check_heading` warns after a flight if the median exceeds 10 deg.
- **Preflight parses the text of `crazyflie.status()`** as printed in release 0.1.11. If a course update
  changes the wording, preflight fails closed; then update the strings in `preflight()`.
- **Keyboard mode monkeypatches** `crazyflie.start_abort` and `crazyflie.MotionCommander` during the flight,
  and restores them afterwards. Re-run the dry run after any update to `crazyflie.py`.
- The abort window is Qt (PyQt6 via matplotlib) and works under Wayland. Caspar's `crazyflie.py` adds a native
  macOS window. The dry run passes against Caspar's version.

## Files on this branch

- `drone_pipeline.py`: drone_02/04/05 as CLI functions, with preflight, keyboard mode and handover.
- `testing/test_drone_pipeline.py`: the dry run on the fake drone, covering every function including the keyboard.
- `data/drone_4_params.json`: the calibration. `set_best` and `handover` add to it.
- `data/flight_4_15*_*.csv`: today's runs.
- `data/plots/<run>.png`: one plot per run.
- `drone_roadmap.html`: the roadmap page.
  - To update the public artifact, use `edit_version` with `shared_live="public"` and
    `artifact_title="Drone 4 Flight Roadmap"`. Otherwise, edit the HTML only.
