"""Day 1 drone experiments as functions: drone_02 (firmware flights), drone_04 (identification), drone_05 (tuning).

The notebook cells for drone LABEL, callable from a terminal. Run from code/day1 with the course venv:

    PY=../../.venv/bin/python
    $PY drone_pipeline.py preflight             # ground gate of every flight: battery, deck, lock, estimates
    $PY drone_pipeline.py abort_test            # once per session: press Enter in the window, it must report the abort
    $PY drone_pipeline.py firmware_flight fwd   # drone_02, FLIES: fwd | square | square_turns | hold
    $PY drone_pipeline.py identify              # drone_04, FLIES: velocity step -> tau, dead time, noise bounds
    $PY drone_pipeline.py plan                  # model numbers for the gain series (drone_analysis_01), no drone
    $PY drone_pipeline.py tune -0.1             # drone_05, FLIES: Kp [Kd Ki] label=.. note=..; model=True: model drone
    $PY drone_pipeline.py table                 # every saved tuning flight with its step numbers
    $PY drone_pipeline.py set_best -1.0 0 0 why=...
    $PY drone_pipeline.py handover              # numbers for Day 2 E7 (lag) and Day 3 R3 (posCtlPid.xKp factor)
    $PY drone_pipeline.py key_test              # no drone: the keyboard window, prints what the keys would send
    $PY drone_pipeline.py keyboard speed=0.1    # FLIES with arrows / WASD in the abort window, Enter lands

Arguments are Python literals, key=value for keywords. FLIES means the real drone takes off: pilot at the abort
window (Enter lands), a second person watching, nobody within arm's length. Every flight runs preflight() first.
Results: data/flight_<drone>_<HHMM>_<label>_*.csv per run, data/drone_<LABEL>_params.json (calibration, best
gains, handover), data/plots/<run>.png. Dry run against the course's fake drone: python testing/test_drone_pipeline.py
"""
import ast
import contextlib
import csv
import io
import json
import os
import re
import signal
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use('Agg')                     # plots go to files; the abort window brings its own Qt
import matplotlib.pyplot as plt
import numpy as np

import crazyflie
import module

HERE = Path(__file__).resolve().parent
DATA = HERE / 'data'
LABEL = 4
URI = crazyflie.drone_uri(LABEL)          # radio://0/30/2M/E7E7E7E704; never another group's drone
KP_SIGN = -1        # module.PIDController: error = value - setpoint, so position gains are negative (drone_03)
STEP, V_MAX, DT = 0.5, 0.3, 0.02          # m, m/s, s: as in drone_05
VBAT_MIN = 3.5      # V at rest for Day 1 flights; the race asks 4.10 V (docs/day_1_baseline_checks.md)
# fly() turns world-frame errors into body-frame velocity setpoints, right only while the drone faces +x. With
# Lighthouse the estimate's yaw is absolute (flight 4_1533 flew at 95 deg and moved sideways), so place it facing +x.
YAW_MAX = 10.0      # deg


# ---------------------------------------------------------------------------
# On the ground
# ---------------------------------------------------------------------------

def preflight(vbat_min=VBAT_MIN, recover=False, yaw_max=YAW_MAX):
    """crazyflie.status() as a gate: raises DroneError unless the drone is fit to fly; returns the volts at rest.

    Checks battery >= vbat_min at rest, flow deck attached, supervisor can arm and is not locked, height
    estimate agrees with the range sensor, position estimate still on the ground, and the heading: |yaw| <=
    yaw_max, see YAW_MAX. recover=True first clears a supervisor lock (after a crash; upright and still).
    """
    out = io.StringIO()
    try:
        with contextlib.redirect_stdout(out), crazyflie.connect(URI) as scf:
            if recover:
                crazyflie.recover(scf)
            crazyflie.status(scf)
            yaw = _read_yaw(scf)
    finally:
        print(out.getvalue(), end='')
    text = out.getvalue()
    print('  heading      : yaw %s deg (limit %g)' % ('--' if yaw is None else '%+.0f' % yaw, yaw_max))
    volts = re.search(r'(\d+\.\d+) V at rest', text)
    # shortcut: matches the wording of status() in release 0.1.11; changed wording fails closed, then update these
    failed = [what for what, ok in (
        ('battery >= %.2f V at rest' % vbat_min, volts is not None and float(volts.group(1)) >= vbat_min),
        ('on battery: unplug the USB cable', 'battery      : in use' in text),   # charging/charged = cable attached
        ('flow deck attached', 'flow deck    : attached' in text),
        ('supervisor can arm or is armed', 'can be armed : yes' in text or 'is armed     : yes' in text),
        ('not locked', 'is locked    : no' in text),
        ('height estimate agrees with the sensor', ', agree' in text),
        ('position estimate still on the ground', 'on the ground, ok' in text),
        ('facing +x: turn the drone until |yaw| <= %g deg' % yaw_max, yaw is not None and abs(yaw) <= yaw_max))
        if not ok]
    if failed:
        raise crazyflie.DroneError('preflight failed, not flying: ' + '; '.join(failed))
    print('PREFLIGHT OK')
    return float(volts.group(1))


def _read_yaw(scf, seconds=1.0):
    """Median stabilizer.yaw over `seconds` on the ground after the estimator reset fly() does; None without samples.

    With Lighthouse the reset does not zero the yaw (flight 4_1533: 90 deg right after it), so this is the
    heading the flight will see.
    """
    crazyflie.reset_estimator(scf, settle=1.0)
    got = []
    conf = crazyflie.LogConfig(name='yaw', period_in_ms=100)
    conf.add_variable('stabilizer.yaw', 'float')
    conf.data_received_cb.add_callback(lambda _ts, data, _conf: got.append(data['stabilizer.yaw']))
    scf.cf.log.add_config(conf)
    conf.start()
    time.sleep(seconds)
    conf.stop()
    return float(np.median(got)) if got else None


def _check_heading(run):
    """After a flight: warn when the drone did not face +x, the run then does not measure the loop it meant to."""
    yaw = run.get('state', {}).get('stabilizer.yaw')
    if yaw is not None and len(yaw) and abs(np.median(yaw)) > YAW_MAX:
        print('WARNING: median yaw %+.0f deg in flight: body-frame setpoints went sideways in the room; '
              'discard this run, face the drone along +x and fly again' % np.median(yaw))


def lh_check(seconds=8.0, yaml=None):
    """No flight: are the Lighthouse base stations seen and fused? Optional `yaml` uploads a setup first
    (as `crazyflie.py lighthouse`). Prints status, active stations, position and yaw; returns True when usable."""
    names = [('lighthouse.status', 'uint8_t'), ('lighthouse.bsActive', 'uint16_t'), ('stateEstimate.x', 'float'),
             ('stateEstimate.y', 'float'), ('stateEstimate.z', 'float'), ('stabilizer.yaw', 'float')]
    with crazyflie.connect(URI, lighthouse_config=yaml) as scf:
        names = [(n, t) for n, t in names if crazyflie._toc_has(scf, n)]
        if not any(n == 'lighthouse.status' for n, _ in names):
            print('no lighthouse log variables: the Lighthouse deck is not detected')
            return False
        crazyflie.reset_estimator(scf, settle=2.0)
        rows = []
        conf = crazyflie.LogConfig(name='lh', period_in_ms=100)
        for n, t in names:
            conf.add_variable(n, t)
        conf.data_received_cb.add_callback(lambda _ts, data, _c: rows.append(data))
        scf.cf.log.add_config(conf)
        conf.start()
        try:
            for _ in range(int(seconds)):
                time.sleep(1.0)
                if rows:
                    r = rows[-1]
                    print('status %d (2 = ok)  stations active %s  x %+.2f y %+.2f z %+.2f m  yaw %+.0f deg' % (
                        r['lighthouse.status'], bin(r.get('lighthouse.bsActive', 0)).count('1'),
                        r['stateEstimate.x'], r['stateEstimate.y'], r['stateEstimate.z'], r['stabilizer.yaw']))
        finally:
            conf.stop()
    if not rows:
        print('no samples arrived: reconnect and rerun')
        return False
    last = rows[len(rows) // 2:]
    n_bs = min(bin(r.get('lighthouse.bsActive', 0)).count('1') for r in last)
    spread = max(float(np.ptp([r['stateEstimate.%s' % a] for r in last])) for a in 'xyz')
    ok = all(r['lighthouse.status'] == 2 for r in last) and n_bs >= 2 and spread < 0.02
    print('LIGHTHOUSE %s: status 2 throughout %s, >= 2 stations %s (min %d), position still within 2 cm %s (%.3f m)'
          % ('OK' if ok else 'NOT OK', all(r['lighthouse.status'] == 2 for r in last), n_bs >= 2, n_bs,
             spread < 0.02, spread))
    return ok


def abort_test():
    """Once per session: a 10 s dummy loop with the flights' abort window; press Enter in it, it must report the abort."""
    crazyflie.bench_test_abort()


def params():
    """The calibration of this drone (calibrate()), plus best gains and handover once set."""
    path = DATA / ('drone_%s_params.json' % LABEL)
    if not path.exists():
        raise crazyflie.DroneError('no %s yet: fly identify() first, or calibrate(<run name>)' % path.name)
    return json.loads(path.read_text())


def _update_params(**values):
    path = DATA / ('drone_%s_params.json' % LABEL)
    p = json.loads(path.read_text()) if path.exists() else {}
    p.update(values)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(p, indent=2) + '\n')
    print('saved', path)
    return p


# ---------------------------------------------------------------------------
# drone_02: under the firmware's own controllers
# ---------------------------------------------------------------------------

PATHS = {'fwd': [('forward', 0.5), ('hold', 2.0), ('back', 0.5)],
         'square': [('forward', 0.5), ('left', 0.5), ('back', 0.5), ('right', 0.5)],
         'square_turns': [('forward', 0.5), ('turn', 90)] * 4,
         'hold': [('hold', 8.0)]}


def firmware_flight(path='fwd', speed=0.3, label=None):
    """drone_02, FLIES: a path of PATHS under the firmware's controllers; saved, then firmware_numbers() and plot()."""
    preflight()
    run = crazyflie.fly_path(PATHS[path], uri=URI, speed=speed)
    name = crazyflie.save(run, label or 'firmware_%s' % path, data_dir=str(DATA))
    firmware_numbers(name)
    plot(name)
    return name


def firmware_numbers(name, data_dir=None):
    """drone_02 analysis: hover spread before the path, largest and final distance from the planned path."""
    run = crazyflie.load_run(name, str(data_dir or DATA))
    _check_heading(run)
    pid = run['pid']
    if not len(pid['t_s']):
        print('%s: no loop records, the flight ended before the path' % name)
        return None
    hold, moving = pid['step'] < 0.5, pid['step'] >= 0.5
    off = np.hypot(pid['x'] - pid['target_x'], pid['y'] - pid['target_y'])
    n = {'hover_std_x_mm': 1000 * float(np.std(pid['x'][hold])), 'hover_std_y_mm': 1000 * float(np.std(pid['y'][hold])),
         'max_path_error_cm': 100 * float(off[moving].max()) if moving.any() else None,
         'end_error_cm': 100 * float(off[-1])}
    print('%s: hover std x %.1f mm, y %.1f mm; largest distance from the plan %s cm, at the end %.1f cm'
          % (name, n['hover_std_x_mm'], n['hover_std_y_mm'],
             '--' if n['max_path_error_cm'] is None else '%.1f' % n['max_path_error_cm'], n['end_error_cm']))
    return n


# ---------------------------------------------------------------------------
# drone_04: the parameters of the drone
# ---------------------------------------------------------------------------

def identify(step=0.2, step_s=1.5):
    """drone_04, FLIES: hover, velocity setpoint `step` m/s along x for `step_s` s, land; then calibrate()."""
    preflight()
    run = crazyflie.identify_response(step=step, step_s=step_s, uri=URI)
    name = crazyflie.save(run, 'identify', data_dir=str(DATA))
    calibrate(name)
    return name


def calibrate(name, data_dir=None):
    """drone_04 analysis: tau and dead time of the firmware velocity loop, hover noise, gain bounds -> params().

    Prints Milestone 3: the fit follows the data, the dead time is below 0.3 s, the velocity does not overshoot.
    """
    run = crazyflie.load_run(name, str(data_dir or DATA))
    _check_heading(run)
    fit = crazyflie.fit_tau(run)
    noise = crazyflie.hover_noise(run)
    limits = crazyflie.gain_limits(noise)
    v_set = abs(fit['v_set'])
    rms = float(np.sqrt(np.mean((fit['v_meas'] - fit['v_fit']) ** 2)))
    peak = float(np.max(fit['v_meas'] * np.sign(fit['v_set'])) / v_set)
    print('hover noise: x %.1f mm, y %.1f mm, vx %.0f mm/s, vy %.0f mm/s (%d records)'
          % (1000 * noise['std_x_m'], 1000 * noise['std_y_m'], 1000 * noise['std_vx_mps'], 1000 * noise['std_vy_mps'],
             noise['n']))
    print('noise bounds: |Kp| < %.1f 1/s, |Kd| < %.1f' % (limits['Kp_max'], limits['Kd_max']))
    # the 0.3 s is drone_04's; 20 % rms and 110 % peak are our reading of "the fit followed" and "first order"
    for ok, text in ((rms < 0.2 * v_set, 'fit follows the data (rms %.0f mm/s, below 20 %% of the step)' % (1000 * rms)),
                     (fit['delay_s'] < 0.3, 'dead time %.2f s, below 0.3 s' % fit['delay_s']),
                     (peak < 1.1, 'velocity peak %.0f %% of the setpoint: no overshoot, first order holds' % (100 * peak))):
        print('  %s  %s' % ('PASS' if ok else 'FAIL', text))
    p = _update_params(source=name, tau_s=fit['tau_s'], delay_s=fit['delay_s'], fit_rms_mps=rms, velocity_peak=peak,
                       kp_max=limits['Kp_max'], kd_max=limits['Kd_max'], **noise)
    plot(name, data_dir, fit=fit)
    return p


# ---------------------------------------------------------------------------
# drone_05: our position loop, one gain per flight
# ---------------------------------------------------------------------------

def make_controller(Kp, Kd, Ki):
    """drone_05's controller: one module.PIDController per axis, feedback (a velocity) clipped to V_MAX."""
    x_ctrl = module.PIDController(Kp=Kp, Ki=Ki, Kd=Kd, deltaT=DT)
    y_ctrl = module.PIDController(Kp=Kp, Ki=Ki, Kd=Kd, deltaT=DT)

    def controller(x, y, vx, vy, x_target, y_target):
        return (float(np.clip(x_ctrl(x, vx, x_target), -V_MAX, V_MAX)),
                float(np.clip(y_ctrl(y, vy, y_target), -V_MAX, V_MAX)))

    def reset():
        x_ctrl.reset()
        y_ctrl.reset()

    return controller, reset


def predict(Kp, Kd=0.0, Ki=0.0, step=STEP):
    """The model's numbers for these gains with this drone's tau and dead time, printed; refuses Kp >= 0."""
    p = params()
    return crazyflie.predict_step(Kp, Kd, Ki, p['tau_s'], p['delay_s'], step=step, v_max=V_MAX,
                                  limits={'Kp_max': p['kp_max'], 'Kd_max': p['kd_max']})


def plan(step=STEP):
    """drone_analysis_01: model numbers for drone_05's gain series, and the first real gain to fly."""
    p = params()
    series = ([(KP_SIGN * k, 0.0, 0.0) for k in (0.1, 0.5, 1.0, 2.0, 4.0, 8.0)]
              + [(KP_SIGN * 4.0, KP_SIGN * kd, 0.0) for kd in (0.3, 0.6, -0.3)]
              + [(KP_SIGN * 1.0, 0.0, KP_SIGN * ki) for ki in (0.1, 0.3)])
    print('model: tau %.2f s, dead time %.2f s, step %.2f m, v_max %.2f m/s' % (p['tau_s'], p['delay_s'], step, V_MAX))
    print('%6s %6s %6s %9s %9s %10s %9s %7s %7s' % ('Kp', 'Kd', 'Ki', 'rise [s]', 'over [m]', 'err2s [m]',
                                                   'mean [m]', 'limit', 'noise'))
    for Kp, Kd, Ki in series:
        n = crazyflie.model_numbers(Kp, Kd, p['tau_s'], delay=p['delay_s'], step=step, v_max=V_MAX, Ki=Ki)
        noisy = abs(Kp) > p['kp_max'] or abs(Kd) > p['kd_max']
        print('%+6.2f %+6.2f %+6.2f %9s %9.3f %10.3f %+9.3f %6.0f%% %7s'
              % (Kp, Kd, Ki, '--' if n['rise_s'] is None else '%.2f' % n['rise_s'], n['overshoot_m'],
                 n['settle_error_m'], n['mean_error_m'], 100 * n['clamp_fraction'], 'OVER' if noisy else 'ok'))
    # drone_05: aim at a rise time of 2-3 s; the smallest gain that gets there is the cautious first real gain
    for k in np.arange(0.1, 4.001, 0.05):
        n = crazyflie.model_numbers(KP_SIGN * k, 0.0, p['tau_s'], delay=p['delay_s'], step=step, v_max=V_MAX)
        if n['rise_s'] is not None and n['rise_s'] <= 3.0:
            print('first real gain (smallest |Kp| with model rise <= 3 s): Kp = %+.2f, rise %.2f s, overshoot %.3f m'
                  % (KP_SIGN * k, n['rise_s'], n['overshoot_m']))
            return KP_SIGN * float(round(k, 2))
    return None


def tune(Kp, Kd=0.0, Ki=0.0, label=None, note='', axes='x', step=STEP, model=False):
    """drone_05, FLIES (model=True: the model drone with this drone's tau and dead time, no radio).

    One flight with our PID position loop: handover, hold 2 s, the target steps by `step` along `axes`, 4 s,
    land. Prints the prediction first, saves the run with gains and prediction, prints measured against
    predicted, writes a plot. The first flight, |Kp| <= 0.2, is the sign check: it must move towards the target.
    """
    p = params()
    pred = predict(Kp, Kd, Ki, step)
    label = label or ('kp%g' % abs(Kp) + ('_kd%+g' % Kd if Kd else '') + ('_ki%+g' % Ki if Ki else ''))
    info = dict(Kp=Kp, Kd=Kd, Ki=Ki, tau=p['tau_s'], delay=p['delay_s'], note=note,
                predicted_rise_s=pred['rise_s'], predicted_overshoot_m=pred['overshoot_m'])
    controller, reset = make_controller(Kp, Kd, Ki)
    kw = dict(axes=axes, step=step, v_max=V_MAX, reset=reset, info=info)
    if model:
        run = crazyflie.fly_model(controller, tau=p['tau_s'], delay=p['delay_s'], **kw)
    else:
        preflight()
        run = crazyflie.fly(controller, uri=URI, **kw)
    name = crazyflie.save(run, label, data_dir=str(DATA))
    flown = crazyflie.load_run(name, str(DATA))
    _check_heading(flown)
    meas = crazyflie.step_numbers(flown, v_max=V_MAX, quiet=True).get(axes[0])
    if meas:
        show = lambda v: '--' if v is None else '%.3f' % v
        print('%-26s %10s %10s' % ('', 'predicted', 'measured'))
        for key, text in (('rise_s', 'rise to 90 % [s]'), ('overshoot_m', 'overshoot [m]'),
                          ('settle_error_m', 'max error, last 2 s [m]'), ('mean_error_m', 'mean error, last 2 s [m]')):
            print('%-26s %10s %10s' % (text, show(pred[key]), show(meas[key])))
        if abs(meas['mean_error_m']) > abs(step):
            print('WARNING: the drone ended further from the target than the step: is the sign of the gains wrong?')
    plot(name)
    return name


def table(drone=LABEL, data_dir=None):
    """drone_05: one line per saved tuning flight (runs flown with gains), with its step numbers."""
    runs = [r for r in crazyflie.load_runs(str(data_dir or DATA), drone=drone) if 'Kp' in r['info'] and r['t_step'] is not None]
    return crazyflie.run_table(runs)


def set_best(Kp, Kd=0.0, Ki=0.0, why=''):
    """drone_05's last cell: the gains you would fly again, kept with the calibration."""
    return _update_params(best=dict(Kp=Kp, Kd=Kd, Ki=Ki, why=why))


# ---------------------------------------------------------------------------
# What Day 2 and Day 3 take from this
# ---------------------------------------------------------------------------

def handover(fw_kp=None, factors=(0.9, 1.0, 1.1), speeds=(0.3, 0.5), fw_v_max=1.0):
    """Day 2 E7's lag and Day 3 R3's gain factor from the calibration and the saved flights; stored in params().

    Day 3's common runner streams position targets to the firmware position PID, whose posCtlPid.xKp/yKp play
    the role of our |Kp| (m/s per m of error); R3 allows one change by a factor within 0.9-1.1. Following a
    target that moves at v, a P loop lags by v / Kp in steady state: lag = 1 / Kp seconds, the number E7 assumed
    as 0.25 s. The step numbers use the measured tau and dead time; onboard there is no radio dead time, so
    their overshoot is an upper estimate. fw_v_max is the firmware's xVelMax (1 m/s default), not our V_MAX.
    """
    p = params()
    source = 'assumed firmware default; read it: python crazyflie.py params --label %s' % LABEL
    dump = DATA / ('params_%s.csv' % LABEL)
    if fw_kp is not None:
        source = 'argument'
    elif dump.exists():
        with dump.open(newline='') as f:
            fw_kp = float(next(r['value'] for r in csv.DictReader(f) if r['name'] == 'posCtlPid.xKp'))
        source = str(dump)
    else:
        fw_kp = 2.0
    print('velocity loop of drone %s: tau %.2f s, dead time %.2f s (%s); with velocity feed-forward the lag '
          'would be about tau + dead time = %.2f s' % (LABEL, p['tau_s'], p['delay_s'], p['source'],
                                                        p['tau_s'] + p['delay_s']))
    print('firmware posCtlPid.xKp = %g (%s)' % (fw_kp, source))
    print('%7s %7s %8s %s %9s %9s' % ('factor', 'xKp', 'lag [s]', ''.join('%15s' % ('err @%.1f m/s' % v) for v in speeds),
                                      'rise [s]', 'over [m]'))
    rows = []
    for factor in factors:
        kp = factor * fw_kp
        n = crazyflie.model_numbers(-kp, 0.0, p['tau_s'], delay=p['delay_s'], step=STEP, v_max=fw_v_max)
        rows.append(dict(factor=factor, xKp=kp, lag_s=1 / kp, rise_s=n['rise_s'], overshoot_m=n['overshoot_m'],
                         following_error_m={str(v): v / kp for v in speeds}))
        print('%7.2f %7.2f %8.3f %s %9s %9.3f' % (factor, kp, 1 / kp, ''.join('%13.3f m' % (v / kp) for v in speeds),
                                                  '--' if n['rise_s'] is None else '%.2f' % n['rise_s'],
                                                  n['overshoot_m']))
    names = [n for n in crazyflie.saved_runs(str(DATA), drone=LABEL)
             if 'moves' in crazyflie.load_run(n, str(DATA))['info']]          # fly_path runs carry their moves
    firmware = {n: firmware_numbers(n) for n in names}
    hover = [max(v['hover_std_x_mm'], v['hover_std_y_mm']) for v in firmware.values() if v]
    if hover:
        print('Day 1 tracking under the firmware: hover spread up to %.1f mm over %d flights' % (max(hover), len(hover)))
    if 'best' in p:
        print('best Day 1 gains: %s' % p['best'])
    return _update_params(handover=dict(fw_kp=fw_kp, fw_kp_source=source, rows=rows, firmware_flights=firmware))


# Keyboard flying: fly_path() with its checks and logging, the velocity setpoint taken from the keys held.
KEYS = {'up': (1, 0, 0, 0), 'down': (-1, 0, 0, 0), 'left': (0, 1, 0, 0), 'right': (0, -1, 0, 0),   # vx vy vz yaw
        'w': (0, 0, 1, 0), 's': (0, 0, -1, 0), 'a': (0, 0, 0, 1), 'd': (0, 0, 0, -1)}
_QT_KEYS = {0x01000013: 'up', 0x01000015: 'down', 0x01000012: 'left', 0x01000014: 'right',
            0x57: 'w', 0x53: 's', 0x41: 'a', 0x44: 'd'}                # Qt key codes, the same in every binding
held = set()                                                              # names of KEYS held now


def key_setpoint(keys, speed, z, dt, z_range=(0.3, 1.0), v_z=0.2, yaw_rate=30.0):
    """(vx, vy, vz, yaw rate, new z) for the held keys: body frame, |v_xy| <= speed, vz stops at the height range."""
    s = [sum(KEYS[k][i] for k in keys) for i in range(4)]
    n = max(1.0, float(np.hypot(s[0], s[1])))                             # a diagonal is not faster
    vz = v_z * s[2] if z_range[0] <= z + v_z * s[2] * dt <= z_range[1] else 0.0
    return speed * s[0] / n, speed * s[1] / n, vz, yaw_rate * s[3], z + vz * dt


def _key_filter():
    """Track arrows and WASD in every window of the app; losing the focus releases all keys, so the drone stops."""
    from matplotlib.backends.qt_compat import QtCore, QtWidgets
    T = QtCore.QEvent.Type

    class Keys(QtCore.QObject):
        def eventFilter(self, _obj, event):
            if event.type() in (T.KeyPress, T.KeyRelease) and event.key() in _QT_KEYS:
                if not event.isAutoRepeat():
                    (held.add if event.type() == T.KeyPress else held.discard)(_QT_KEYS[event.key()])
                return True                                               # arrows must not move the focus
            if event.type() in (T.WindowDeactivate, T.ApplicationDeactivate):
                held.clear()
            return False

    keys = Keys()
    QtWidgets.QApplication.instance().installEventFilter(keys)
    return keys


def key_test(seconds=15.0):
    """No drone: the keyboard window on this screen, printing the setpoint the held keys would send; Enter ends."""
    abort = crazyflie.start_abort('key test: arrows, W/S, A/D; Enter ends')
    keys, shown, t_end = _key_filter(), None, time.monotonic() + seconds   # keys: keep the filter alive
    while time.monotonic() < t_end and not abort.stop.is_set():
        sp = key_setpoint(held, 0.2, 0.5, 0.0)[:4]
        if sp != shown:
            shown = sp
            print('%-22s vx %+.2f  vy %+.2f  vz %+.2f m/s  yaw %+3.0f deg/s' % (sorted(held), *sp))
        abort.pump()
        time.sleep(0.02)
    abort.close()
    held.clear()


def keyboard(seconds=60.0, speed=0.2, z=0.5, fence=1.5, label='keyboard'):
    """FLIES with the keyboard: click into the abort window, then arrows = forward/back/left/right at `speed` m/s,
    W/S = up/down 0.2 m/s within 0.3-1.0 m, A/D = turn left/right 30 deg/s, no key = stop; Enter lands, and the
    flight lands by itself after `seconds`. Arrows are in the frame of the drone. All checks of fly_path():
    preflight, fence around the start, stale data, battery, frozen height sensor, estimator variance. The logged
    setpoint columns stay 0 (the keys bypass fly_path's plan); the measured position is what to look at."""
    if not 0 < speed <= V_MAX:
        raise crazyflie.DroneError('speed %g m/s: keep 0 < speed <= %g' % (speed, V_MAX))
    preflight()
    held.clear()
    st = {'mc': None, 'z': z, 't': None, 'keys': None}
    start_abort, motion_commander = crazyflie.start_abort, crazyflie.MotionCommander

    def tick():                                    # every tick, so fly_path's zero setpoint between segments never sticks
        if st['mc'] is None:
            return
        now = time.monotonic()
        *sp, st['z'] = key_setpoint(held, speed, st['z'], now - (st['t'] or now))
        st['t'] = now
        st['mc'].start_linear_motion(*sp)

    def keyed_abort(*args, **kw):
        abort = start_abort(*args, **kw)
        if abort.kind == 'window':
            st['keys'] = _key_filter()
        pump = abort.pump
        def keyed_pump(text=None):
            if text is None:                       # the loop's tick; stage messages, e.g. 'landing', move nothing
                tick()
            pump(text)
        abort.pump = keyed_pump
        return abort

    class KeyedMotionCommander(motion_commander):
        def __enter__(self):
            st['mc'] = super().__enter__()
            return st['mc']

        def down(self, *args, **kw):               # land_gently(): the keys stop moving the drone
            st['mc'] = None
            return super().down(*args, **kw)

        def land(self, *args, **kw):
            st['mc'] = None
            return super().land(*args, **kw)

    print('keyboard flight: click into the abort window, arrows move, W/S height, A/D turn, Enter lands')
    crazyflie.start_abort, crazyflie.MotionCommander = keyed_abort, KeyedMotionCommander
    try:
        run = crazyflie.fly_path([('hold', seconds)], uri=URI, z=z, speed=speed, fence=fence,
                                 info={'keyboard_speed': speed})
    finally:
        crazyflie.start_abort, crazyflie.MotionCommander = start_abort, motion_commander
        held.clear()
    name = crazyflie.save(run, label, data_dir=str(DATA))
    _check_heading(crazyflie.load_run(name, str(DATA)))
    plot(name)
    return name


def plot(name, data_dir=None, fit=None):
    """Position and velocity of a saved run against time, to data/plots/<name>.png; a fit_tau() result overlays."""
    r = crazyflie.load_run(name, str(data_dir or DATA))
    pid = r['pid']
    t = pid['t_s']
    if not len(t):
        return None
    fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    for a in 'xy':
        line, = ax[0].plot(t, pid[a] - pid['target_%s' % a][0], label='%s measured' % a)
        ax[0].plot(t, pid['target_%s' % a] - pid['target_%s' % a][0], '--', color=line.get_color(), lw=0.8,
                   label='%s target' % a)
        line, = ax[1].plot(t, pid['fb_%s' % a], label='%s setpoint (feedback)' % a)
        ax[1].plot(t, pid['v%s' % a], color=line.get_color(), alpha=0.5, label='v%s measured' % a)
    if fit is not None:
        i0 = int(np.argmax(np.abs(pid['fb_x']) > 1e-6))       # where velocity_step() starts the fit
        ax[1].plot(t[i0] + fit['t'], pid['vx'][i0] + fit['v_fit'], 'k--',
                   label='fit: tau %.2f s, dead time %.2f s' % (fit['tau_s'], fit['delay_s']))
    if r['t_step'] is not None:
        for a in ax:
            a.axvline(r['t_step'], color='0.6', lw=0.8)
    ax[0].set_title(name + ('  ABORTED: %s' % r['info']['aborted'] if r['info'].get('aborted') else ''), fontsize=9)
    ax[0].set_ylabel('position from start [m]')
    ax[1].set_ylabel('velocity [m/s]')
    ax[1].set_xlabel('t [s]')
    for a in ax:
        a.grid()
        a.legend(fontsize=7, loc='upper left')
    out = DATA / 'plots' / ('%s.png' % name)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print('plot', out)
    return str(out)


def _arg(text):
    try:
        return ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return text


if __name__ == '__main__':
    os.chdir(HERE)                                              # connect() keeps its parameter cache in ./cache
    signal.signal(signal.SIGTERM, signal.default_int_handler)   # a killed run lands like an interrupt, not a drop
    if len(sys.argv) < 2 or sys.argv[1].startswith('_') or not callable(globals().get(sys.argv[1])):
        sys.exit(__doc__)
    argv = sys.argv[2:]
    result = globals()[sys.argv[1]](*[_arg(a) for a in argv if '=' not in a],
                                    **{k: _arg(v) for k, v in (a.split('=', 1) for a in argv if '=' in a)})
    if result is not None:
        print(result)
