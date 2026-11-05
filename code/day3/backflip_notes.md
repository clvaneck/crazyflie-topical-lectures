# Backflip (pitch flip) for race 2: decision note

Written 2026-11-05. Firmware facts checked in crazyflie-firmware `master` and cflib 0.1.34 (our venv).
Nothing here has been flown yet. Firmware on the drone must be recent enough for `send_setpoint_manual` (generic TYPE_MANUAL); check it first.

## Recommendation: no-go by default, go only if all gates below hold
- The way this is normally done is an open-loop, phased maneuver: climb, spin at a high body rate with low thrust, recover (Lupashin 2010/2011, Mellinger 2012). Stock firmware supports this through rate mode, so no firmware changes are needed.
- Main risk: the first attempt is the real one, and it comes right after a race on a drained battery. Brushed motors lose thrust as the voltage drops, so the drone has less margin and falls further. Bitcraze forum: flipped CFs "tend to hit the floor" and the CF is not built for high-speed crashes.
- Go only if: there is a spare drone or props, the floor is padded, the ceiling is at least 2.3 m, a rehearsal with the script has been done in sim or dry, and the race result is already in.
- Teammate's "PID with a 3rd derivative": **no**. A flip runs at actuator saturation and follows a feedforward plan, so feedback gains are not what limits it. The onboard rate loop already closes on the gyro. A D³ term on a noisy 27 g IMU only amplifies noise; the firmware already low-passes plain D. The valid idea behind it: jerk maps to body rate (differential flatness, Mellinger & Kumar minimum-snap). But that idea means jerk/snap *feedforward* from a planned trajectory into the geometric (Mellinger) controller. It is not a PID term, and it is not feasible in 1 h.

## Phase plan (PID controller 1, Kalman 2, setpoints streamed at 100 Hz, no gaps)
Measure hover thrust T_h first: log `stabilizer.thrust` for 2 s in hover (expected ~38-45k for about 32 g with decks).
0. **Arm**: position-hold at (x0, y0, z0 = 1.7 m), centred, nothing within 1 m. Check the gates (below). Set `motion.disable = 1` (flow is garbage when inverted).
1. **Climb** 0.20 s: angle mode, level, thrust ~1.6 T_h (cap 60000). Expected vz ≈ +1.2-1.5 m/s.
2. **Rotate**: rate mode, pitch rate 800 dps (keep it ≤ 1000; the gyro saturates at 2000 dps), thrust ~0.3 T_h. The rate loop needs some base thrust to have differential authority, so do not send 0. Duration open-loop ≈ 330°/800 dps ≈ 0.40-0.45 s including spin-up. Tune only the duration.
3. **Recover** 0.35 s: angle mode, roll = pitch = 0, thrust ~1.5 T_h. The onboard attitude PID levels the last ±30-60°, so timing errors of ±40 ms are tolerated. Do not switch near 180° or 270° (Euler-angle errors are meaningless there).
4. **Re-engage**: `motion.disable = 0`. Send `send_position_setpoint` at the *current* estimate first, then ramp back to (x0, y0, z0).
Expected height loss is ~0.5-1.0 m below the start of rotation, or ~0.3-0.6 m below z0 after the climb (ballistic: 1.5·0.45 − ½g·0.45² ≈ −0.3 m, then stopping from ~3 m/s at ~0.8 g net).
Backflip vs frontflip sign: cflib negates pitch in both `send_setpoint` and `send_setpoint_manual`. Check the sign with a 10° angle-mode step in hover beforehand.

## cflib calls / params
- `cf.param.set_value('stabilizer.controller', '1')` (PID; race code already asserts 1 and estimator 2).
- Rate setpoint without param juggling: `cf.commander.send_setpoint_manual(roll, pitch_dps, yawrate, thrust_pct, rate=True)`. Use `rate=False` for the level phases. thrust_pct maps to 10001 + pct·500.
- Legacy alternative: `flightmode.stabModePitch = 0` (0 = rate, 1 = angle), then `send_setpoint(0, pitch_dps, 0, thrust)`. Restore to 1 afterwards. Thrust needs one 0-thrust packet to unlock.
- Back to position: `send_position_setpoint(x, y, z, yaw)`. If the high-level commander is in use, call `send_notify_setpoint_stop()` first.
- Logs: `stateEstimate.x/y/z`, `gyro.y`, `pm.vbat`, `supervisor.info`, `kalman.varPX/PY/PZ`.
- `send_zdistance_setpoint` is not useful here: it holds height with angle inputs.

## Safety gates (all must hold, else land normally)
- z0 ≥ 1.5 m (target 1.7 m), clear ceiling ≥ z0 + 0.6 m, ≥ 1 m from walls/hoops, padding under the flip.
- `pm.vbat` ≥ 3.8 V in hover (fresh battery preferred: do the flip early in the race-2 slot, or swap the battery).
- Kalman healthy: `kalman.varPX/PY/PZ` small, Lighthouse tracking, hover drift < 5 cm.
- Abort: Ctrl-C/kill key → `send_stop_setpoint()` (motors off). In the script, any exception → stop. If z_est < 0.5 m in phase 4, stop.
- One explicit go-ahead from Niels for the flight (team rule).

## Supervisor / tumble and estimator
- `supervisor.c isTumbledCheck`: tumbled if acc.z < 0.5 g for > 1000 ms, or acc.z < −0.2 g for > 100 ms. On the trigger, the supervisor cuts the motors (needs crash recovery/reboot). Key point: in flight the accelerometer measures *thrust specific force*, not gravity, so acc.z ≈ thrust/m ≥ 0 even when inverted. The upside-down branch therefore does not fire in a fast flip. The low-thrust phase *does* start the 1 s tilt timer, so keep phase 2 under ~0.6 s. `crashDetectGs` defaults to 0 (off). Recommendation: leave `supervisor.tmblChckEn = 1`. It is what kills the motors after a crash. Only set it to 0 if a test shows a false trigger.
- Commander watchdog: a gap > 500 ms in setpoints makes the drone level out; > 2 s stops the motors. Stream continuously.
- Estimator through 360°: attitude comes from gyro integration (quaternion, no singularity), which is fine below 2000 dps. ToF z is gated automatically (`mm_tof.c`: only if R[2][2] > 0.1). Flow has **no** orientation gate (`mm_flow.c`), hence `motion.disable = 1`. Lighthouse sensors face down when inverted, so they get no sweeps for ~0.3 s; this is harmless. Kalman resets itself if |v| > 10 m/s or |p| > 100 m (`kalman_supervisor.c`).

## What can go wrong
Too little climb or battery → hits floor. Over-rotation (timing too long) → recover tilted and shoot sideways. Under-rotation → the attitude PID flips back the long way. Yaw drift during the spin. Flow left enabled → Kalman jump and a position-controller lunge after recovery. Radio latency (~10-20 ms) means the timing must run in one tight loop, not via callbacks.

## Sources
- Firmware: https://github.com/bitcraze/crazyflie-firmware/blob/master/src/modules/src/supervisor.c , `src/platform/interface/platform_defaults.h` (tumble thresholds), `src/modules/src/crtp_commander_rpyt.c` (flightmode.stabMode*), `src/modules/src/crtp_commander_generic.c` (manualDecoder), `src/modules/src/kalman_core/mm_flow.c`, `mm_tof.c`, `src/modules/src/kalman_supervisor.c`, `src/hal/src/sensors_bmi088_bmp3xx.c` (2000 dps / 24 g), `src/deck/drivers/src/flowdeck_v1v2.c` (motion.disable)
- cflib: https://github.com/bitcraze/crazyflie-lib-python/blob/master/cflib/crazyflie/commander.py
- Forum: https://forum.bitcraze.io/viewtopic.php?p=3326 (first CF flip, rate mode), https://forum.bitcraze.io/viewtopic.php?p=15669 (rate mode → flip → re-enable angle mode), https://forum.bitcraze.io/viewtopic.php?p=7024 (CF2 rate mode)
- Lupashin, Schöllig, Sherback, D'Andrea, "A simple learning strategy for high-speed quadrocopter multi-flips", ICRA 2010
- Lupashin & D'Andrea, "Adaptive open-loop aerobatic maneuvers for quadrocopters", IFAC World Congress 2011
- Mellinger, Michael, Kumar, "Trajectory generation and control for precise aggressive maneuvers with quadrotors", IJRR 2012; Mellinger & Kumar, "Minimum snap trajectory generation and control for quadrotors", ICRA 2011
