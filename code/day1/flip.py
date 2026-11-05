"""Backflip for drone 4: model first, then rehearsal steps, then the flip. Each real flight needs Niels's go.

    PY flip.py plan                       # no radio: predicted heights for the default flip
    PY flip.py plan climb_s=0.3 rate=1000  # other settings
    PY flip.py fly step=hover             # FLIES: hover at z0, measures hover thrust, lands
    PY flip.py fly step=hop               # FLIES: climb burst + recover, no spin (tests thrust and recovery)
    PY flip.py fly step=flip              # FLIES: the backflip

Method (code/day3/backflip_notes.md): open-loop phases on the legacy roll/pitch/yaw-rate/thrust commander,
pitch in rate mode (flightmode.stabModePitch = 0) only for the spin, flow deck off (motion.disable = 1) while
tilted, then position hold at the current estimate. Thrust levels are set relative to the hover thrust measured
in the same flight, so battery and deck weight are included. Hover took 71-76 % PWM on 5 Nov: little headroom.
"""
import sys
import time

import numpy as np

import crazyflie
import drone_pipeline as dp

G = 9.81
PWM_MAX = 60000          # cap below 65535: keeps some differential authority for the attitude loop


def force(pwm):
    """Thrust per motor [N] for a CF2 brushed motor (Bitcraze system identification, Förster 2015)."""
    pwm = np.asarray(pwm, float)
    return 2.130295e-11 * pwm ** 2 + 1.032633e-6 * pwm + 5.484560e-4


def pwm_for(ratio, hover_pwm):
    """PWM giving `ratio` times the hover thrust, capped at PWM_MAX."""
    f = ratio * force(hover_pwm)
    a, b, c = 2.130295e-11, 1.032633e-6, 5.484560e-4 - f
    return int(min(PWM_MAX, max(0.0, (-b + np.sqrt(b * b - 4 * a * c)) / (2 * a))))


def simulate(z0=1.5, climb_s=0.40, rate=1000.0, spin_ratio=0.3, recover_ratio=None, hover_pwm=47000,
             spin_deg=330.0, tau_rate=0.03, tau_angle=0.08, dt=0.001):
    """Vertical/forward point model of the flip. Returns phases, thrusts and the predicted heights."""
    max_ratio = float(force(PWM_MAX) / force(hover_pwm))
    k_climb = max_ratio
    k_rec = max_ratio if recover_ratio is None else min(recover_ratio, max_ratio)
    t_spin = spin_deg / rate + tau_rate               # first-order rate response lags by about tau_rate
    z, vz, x, vx, th, w, t = z0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    z_max, z_min, phase_end = z0, z0, {}
    for phase, dur in (('climb', climb_s), ('spin', t_spin), ('recover', 3.0)):
        t_end = t + dur
        while t < t_end:
            if phase == 'climb':
                k, w = k_climb, 0.0
            elif phase == 'spin':
                k = spin_ratio
                w += (np.radians(rate) - w) * dt / tau_rate
            else:
                k = k_rec
                w = (np.radians(360.0) - th) / tau_angle      # angle loop pulls the last part level
            th += w * dt
            vz += (k * G * np.cos(th) - G) * dt
            vx += (k * G * np.sin(th)) * dt
            z += vz * dt
            x += vx * dt
            z_max, z_min, t = max(z_max, z), min(z_min, z), t + dt
            if phase == 'recover' and vz >= 0 and abs(np.degrees(th) - 360) < 5:
                break
        phase_end[phase] = dict(t=round(t, 3), z=round(z, 3), vz=round(vz, 2), deg=round(np.degrees(th), 0))
    return dict(z0=z0, z_min=round(z_min, 3), z_max=round(z_max, 3), drift_x=round(x, 2), t_spin=round(t_spin, 3),
                max_ratio=round(max_ratio, 2), climb_pwm=pwm_for(k_climb, hover_pwm),
                spin_pwm=pwm_for(spin_ratio, hover_pwm), recover_pwm=pwm_for(k_rec, hover_pwm), phases=phase_end)


def plan(**kw):
    """No radio: the default flip and a few alternatives, with the predicted lowest and highest point."""
    print('%-34s %7s %7s %7s %7s' % ('setting', 'z min', 'z max', 'x drift', 't spin'))
    base = dict(kw)
    for label, extra in (('this setting', {}), ('climb 0.2 s', dict(climb_s=0.2)), ('climb 0.4 s', dict(climb_s=0.4)),
                         ('rate 700 dps', dict(rate=700.0)), ('rate 1000 dps', dict(rate=1000.0)),
                         ('hover 50000 pwm (tired battery)', dict(hover_pwm=50000))):
        r = simulate(**dict(base, **extra))
        print('%-34s %6.2fm %6.2fm %6.2fm %6.2fs' % (label, r['z_min'], r['z_max'], r['drift_x'], r['t_spin']))
    r = simulate(**base)
    print('this setting: max thrust %.2f x hover; climb %d, spin %d, recover %d PWM; phases %s'
          % (r['max_ratio'], r['climb_pwm'], r['spin_pwm'], r['recover_pwm'], r['phases']))
    print('model only: point mass, Bitcraze thrust map, no drag, no battery sag. Add 0.3 m margin below z min.')
    return None


# ---------------------------------------------------------------------------
# Flight
# ---------------------------------------------------------------------------

def _log(scf, rows):
    conf = crazyflie.LogConfig(name='flip', period_in_ms=10)
    for n, t in (('stateEstimate.x', 'float'), ('stateEstimate.y', 'float'), ('stateEstimate.z', 'float'),
                 ('stateEstimate.vz', 'float'), ('stabilizer.pitch', 'float'), ('stabilizer.thrust', 'float')):
        conf.add_variable(n, t)
    conf.data_received_cb.add_callback(lambda ts, d, _c: rows.append(dict(d, t=time.monotonic())))
    scf.cf.log.add_config(conf)
    conf.start()
    return conf


def _stream(cf, seconds, send, abort, dt=0.01):
    """Send a setpoint every 10 ms for `seconds`; stops early when abort is set. Returns False on abort."""
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        send()
        abort.pump()
        if abort.stop.is_set():
            return False
        time.sleep(dt)
    return True


def _hold(cf, x, y, z, seconds, abort):
    return _stream(cf, seconds, lambda: cf.commander.send_position_setpoint(x, y, z, 0.0), abort)


def _land(cf, x, y, z, abort_ignored=None):
    for zi in np.linspace(z, 0.05, max(2, int(z / 0.3 * 100))):      # 0.3 m/s down
        cf.commander.send_position_setpoint(x, y, float(zi), 0.0)
        time.sleep(0.01)
    # the drone lags ~0.5 s: hover at 5 cm for 1 s, then cut (~7 cm drop). Holding z=0 on carpet kept
    # near-hover thrust on the floor, it tipped to 18 deg and the tumble lock fired (rehearsal 1032)
    for _ in range(100):
        cf.commander.send_position_setpoint(x, y, 0.05, 0.0)
        time.sleep(0.01)
    cf.commander.send_stop_setpoint()


def fly(step='hover', z0=1.5, climb_s=0.40, rate=1000.0, spin_ratio=0.3, sign=1, fence=1.0, vbat_min=4.0):
    """FLIES. step: 'hover' (measure hover thrust), 'hop' (climb burst + recover, no spin), 'flip'.
    sign flips the pitch direction: cflib negates pitch, check with the hop log which sign is a backflip."""
    if step not in ('hover', 'hop', 'flip'):
        raise crazyflie.DroneError("step must be 'hover', 'hop' or 'flip'")
    if not 1.2 <= z0 <= 2.2:
        raise crazyflie.DroneError('z0 %.2f m: keep 1.2 <= z0 <= 2.2' % z0)
    dp.preflight(vbat_min=vbat_min)
    rows, abort = [], crazyflie.start_abort('flip %s: Enter lands' % step)
    with crazyflie.connect(dp.URI) as scf:
        cf, param = scf.cf, scf.cf.param
        crazyflie.reset_estimator(scf, settle=1.5)
        conf = _log(scf, rows)
        time.sleep(0.3)
        x0, y0 = rows[-1]['stateEstimate.x'], rows[-1]['stateEstimate.y']
        cf.commander.send_setpoint(0, 0, 0, 0)                     # unlock the rpyt commander on the ground
        landed = False
        try:
            for zi in np.linspace(0.1, z0, int(z0 / 0.4 * 100)):   # take-off at 0.4 m/s
                cf.commander.send_position_setpoint(x0, y0, float(zi), 0.0)
                time.sleep(0.01)
            if not _hold(cf, x0, y0, z0, 3.0, abort):
                return _finish(cf, conf, rows, x0, y0, step)
            hover = [r for r in rows if time.monotonic() - r['t'] < 1.5]
            hover_pwm = float(np.median([r['stabilizer.thrust'] for r in hover]))
            z_now = float(np.median([r['stateEstimate.z'] for r in hover]))
            r = simulate(z0=z_now, climb_s=climb_s, rate=rate, spin_ratio=spin_ratio,
                         hover_pwm=hover_pwm)
            print('hover thrust %.0f PWM (%.0f %%), z %.2f m; a flip from here: model z min %.2f, z max %.2f'
                  % (hover_pwm, hover_pwm / 655.35, z_now, r['z_min'], r['z_max']))
            far = np.hypot(rows[-1]['stateEstimate.x'] - x0, rows[-1]['stateEstimate.y'] - y0)
            gates = {'height within 20 cm of z0': abs(z_now - z0) < 0.20, 'within 15 cm of start': far < 0.15,
                     'hover thrust < 52000 PWM': hover_pwm < 52000,
                     'model z min >= 0.5 m': step == 'hover' or r['z_min'] >= 0.5}
            if not all(gates.values()) or step == 'hover':
                if not all(gates.values()):
                    print('GATE FAILED, landing:', [k for k, v in gates.items() if not v])
                return _finish(cf, conf, rows, x0, y0, step)
            param.set_value('motion.disable', 1)
            if step == 'flip':
                param.set_value('flightmode.stabModePitch', 0)     # pitch is now a rate in rpyt packets
            time.sleep(0.15)
            send = cf.commander.send_setpoint
            _stream(cf, climb_s, lambda: send(0, 0, 0, r['climb_pwm']), abort)
            if step == 'flip':
                _stream(cf, r['t_spin'], lambda: send(0, sign * rate, 0, r['spin_pwm']), abort)
                param.set_value('flightmode.stabModePitch', 1)     # back to angle; pitch 0 = level
                _stream(cf, 0.35, lambda: send(0, 0, 0, r['recover_pwm']), abort)   # hop: no fall to stop
            param.set_value('motion.disable', 0)
            e = rows[-1]
            _hold(cf, e['stateEstimate.x'], e['stateEstimate.y'], max(0.5, e['stateEstimate.z']), 0.5, abort)
            if np.hypot(e['stateEstimate.x'] - x0, e['stateEstimate.y'] - y0) < fence:
                _hold(cf, x0, y0, z0, 2.0, abort)
            return _finish(cf, conf, rows, rows[-1]['stateEstimate.x'], rows[-1]['stateEstimate.y'], step)
        except BaseException:
            param.set_value('flightmode.stabModePitch', 1)
            param.set_value('motion.disable', 0)
            e = rows[-1] if rows else {'stateEstimate.x': 0, 'stateEstimate.y': 0, 'stateEstimate.z': 0.3}
            _land(cf, e['stateEstimate.x'], e['stateEstimate.y'], max(0.1, e['stateEstimate.z']))
            raise
        finally:
            abort.close()


def _finish(cf, conf, rows, x, y, step):
    _land(cf, x, y, max(0.1, rows[-1]['stateEstimate.z']))
    conf.stop()
    path = dp.DATA / ('flip_4_%s_%s.csv' % (time.strftime('%H%M'), step))
    keys = list(rows[0])
    np.savetxt(path, [[r[k] for k in keys] for r in rows], delimiter=',', header=','.join(keys), comments='')
    z = [r['stateEstimate.z'] for r in rows]
    p = [r['stabilizer.pitch'] for r in rows]
    print('saved %s: z %.2f..%.2f m, pitch %+.0f..%+.0f deg' % (path.name, min(z), max(z), min(p), max(p)))
    return path.name


if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] not in ('plan', 'fly'):
        sys.exit(__doc__)
    kw = {k: dp._arg(v) for k, v in (a.split('=', 1) for a in sys.argv[2:])}
    print(globals()[sys.argv[1]](**kw))
