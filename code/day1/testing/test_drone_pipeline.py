"""Dry run of drone_pipeline.py on the fake drone of test_crazyflie.py: every flight function, no radio.

From code/day1:  ../../.venv/bin/python testing/test_drone_pipeline.py   (about 40 s, the fake drone flies in real time)
"""
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE)]
import test_crazyflie as fake        # noqa: E402  installs the fake cflib before crazyflie is imported
import crazyflie                     # noqa: E402
import drone_pipeline as d           # noqa: E402


def main():
    crazyflie.ABORT_WINDOW = False   # no window pops up; without stdin the input box never aborts
    d.URI = fake.TEST_URI
    fake_drone = crazyflie.drone_label(fake.TEST_URI)
    with tempfile.TemporaryDirectory() as tmp:
        d.DATA = Path(tmp)
        fake._reset_world()
        assert d.preflight() == 3.9

        name = d.firmware_flight('fwd', speed=0.5)
        assert d.firmware_numbers(name, d.DATA)['end_error_cm'] < 20

        d.identify(step=0.2, step_s=1.2)
        p = d.params()
        assert 0.15 < p['tau_s'] < 0.5 and p['delay_s'] < 0.15, p     # the fake drone: tau 0.3 s, no dead time

        assert d.plan() is not None
        name = d.tune(-1.0, label='kp1')
        assert crazyflie.step_numbers(crazyflie.load_run(name, str(d.DATA)), quiet=True)['x']['settled']
        d.tune(-1.0, label='kp1', model=True)                         # the model drone, saved as model_*
        assert len(d.table(drone=fake_drone)) == 1
        try:
            d.tune(+1.0)                                              # the wrong sign never flies
            raise AssertionError('tune accepted Kp > 0')
        except crazyflie.DroneError:
            pass

        d.set_best(-1.0, why='settles')
        h = d.handover(fw_kp=2.0)['handover']
        assert [round(r['lag_s'], 3) for r in h['rows']] == [0.556, 0.5, 0.455], h['rows']
        assert d.params()['best']['Kp'] == -1.0 and 'tau_s' in d.params()
        assert len(list((d.DATA / 'plots').glob('*.png'))) == 4

        fake._reset_world(vbat=3.4)
        try:
            d.preflight()
            raise AssertionError('preflight passed a flat battery')
        except crazyflie.DroneError as exc:
            assert 'battery' in str(exc), exc
        fake._reset_world()
        try:
            d.preflight(vbat_min=4.10)                                # the race limit
            raise AssertionError('preflight passed 3.9 V against the race limit')
        except crazyflie.DroneError:
            pass
        read_yaw, d._read_yaw = d._read_yaw, lambda scf: 95.0          # the heading of flight 4_1533
        try:
            d.preflight()
            raise AssertionError('preflight passed a drone facing 95 deg')
        except crazyflie.DroneError as exc:
            assert 'facing +x' in str(exc), exc
        finally:
            d._read_yaw = read_yaw

        check_keyboard()
    print('\ndrone_pipeline dry run: all checks passed')


def check_keyboard():
    vx, vy, vz, yaw, z = d.key_setpoint({'up', 'left'}, 0.2, 0.5, 0.02)
    assert abs((vx ** 2 + vy ** 2) ** 0.5 - 0.2) < 1e-9 and vx > 0 and vy > 0, (vx, vy)   # diagonal not faster
    assert d.key_setpoint({'up', 'down', 'a'}, 0.2, 0.5, 0.02)[:4] == (0.0, 0.0, 0.0, 30.0)
    assert d.key_setpoint({'w'}, 0.2, 1.0, 0.02)[2] == 0.0                               # height ceiling
    assert d.key_setpoint({'s'}, 0.2, 0.3, 0.02)[2] == 0.0                               # height floor

    from matplotlib.backends.qt_compat import QtCore, QtGui, QtWidgets                  # offscreen Qt, no display
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(['test'])
    widget, keys = QtWidgets.QWidget(), d._key_filter()
    T, up = QtCore.QEvent.Type, 0x01000013

    def send(kind, key=up, repeat=False):
        app.sendEvent(widget, QtGui.QKeyEvent(kind, key, QtCore.Qt.KeyboardModifier.NoModifier, '', repeat))
    send(T.KeyPress)
    assert d.held == {'up'}, d.held
    send(T.KeyRelease, repeat=True)                                                      # auto repeat: still held
    assert d.held == {'up'}, d.held
    send(T.KeyRelease)
    assert d.held == set(), d.held
    send(T.KeyPress)
    app.sendEvent(widget, QtCore.QEvent(T.WindowDeactivate))                             # focus lost: all released
    assert d.held == set(), d.held
    app.removeEventFilter(keys)

    # the fake drone with the up arrow held: it must fly forward and the fence must land it
    fake._reset_world()
    flying = threading.Event()
    flying.set()

    def hold_up():
        while flying.is_set():
            d.held.add('up')
            time.sleep(0.05)
    threading.Thread(target=hold_up, daemon=True).start()
    try:
        name = d.keyboard(seconds=10.0, speed=0.3, fence=0.5)
    finally:
        flying.clear()
    pid = crazyflie.load_run(name, str(d.DATA))['pid']
    moved = pid['x'].max() - pid['x'][0]
    assert 0.4 < moved < 0.8 and pid['t_s'][-1] < 6.0, (moved, pid['t_s'][-1])        # stopped by the 0.5 m fence
    assert crazyflie.start_abort.__name__ == 'start_abort'                               # patches undone


if __name__ == '__main__':
    main()
