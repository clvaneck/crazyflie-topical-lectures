"""Offline: flip.fly() against a fake drone; checks the order of mode switches and setpoints. No radio."""
import contextlib, sys, threading, types
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import crazyflie, drone_pipeline as dp, flip

calls = []
class Cmd:
    def __getattr__(self, name):
        return lambda *a: calls.append((name,) + a)
class Param:
    def set_value(self, k, v): calls.append(('param', k, v))
class Log:
    def add_config(self, conf): conf.cb = conf.cbs
class Conf:
    def __init__(self, **kw): self.cbs = []; self.data_received_cb = types.SimpleNamespace(add_callback=self.cbs.append)
    def add_variable(self, *a): pass
    def start(self):
        def feed():
            import time
            while not getattr(self, 'stopped', False):
                for cb in self.cbs:
                    cb(0, {'stateEstimate.x': 0.0, 'stateEstimate.y': 0.0, 'stateEstimate.z': 1.5, 'stateEstimate.vz': 0.0,
                           'stabilizer.pitch': 0.0, 'stabilizer.thrust': 47000.0}, self)
                time.sleep(0.01)
        threading.Thread(target=feed, daemon=True).start()
    def stop(self): self.stopped = True
scf = types.SimpleNamespace(cf=types.SimpleNamespace(commander=Cmd(), param=Param(), log=Log()))
crazyflie.connect = lambda uri: contextlib.nullcontext(scf)
crazyflie.LogConfig = Conf
crazyflie.reset_estimator = lambda scf, settle=0: None
crazyflie.start_abort = lambda text: types.SimpleNamespace(stop=threading.Event(), pump=lambda *a: None, close=lambda: None)
dp.preflight = lambda **kw: 4.1
dp.DATA = Path('/tmp/claude-1000') ; dp.DATA.mkdir(exist_ok=True)

flip.fly(step='flip')
modes = [c for c in calls if c[0] == 'param']
assert modes == [('param', 'motion.disable', 1), ('param', 'flightmode.stabModePitch', 0),
                 ('param', 'flightmode.stabModePitch', 1), ('param', 'motion.disable', 0)], modes
rpyt = [c for c in calls if c[0] == 'send_setpoint']
assert rpyt[0] == ('send_setpoint', 0, 0, 0, 0)                 # unlock on the ground first
spin = [c for c in rpyt if c[2] != 0]
assert spin and all(c[2] == 1000.0 for c in spin) and all(c[4] < 25000 for c in spin)
i0 = calls.index(('param', 'flightmode.stabModePitch', 0)); i1 = calls.index(('param', 'flightmode.stabModePitch', 1))
assert all(i0 < calls.index(c) < i1 for c in spin[:1]), 'spin only while in rate mode'
assert calls[-1] == ('send_stop_setpoint',)
calls.clear(); flip.fly(step='hover')
assert not [c for c in calls if c[0] == 'param'] and calls[-1] == ('send_stop_setpoint',)
print('flip dry run: all checks passed')

import rehearse, numpy as np, tempfile, os
tr = os.path.join(tempfile.mkdtemp(dir='/tmp/claude-1000'), 'trajectory.csv')
tt = np.linspace(0, 3, 31)
np.savetxt(tr, np.c_[tt, 0.3 * tt, 0 * tt, 0.8 + 0 * tt, 0 * tt + 0.3, 0 * tt, 0 * tt, 0 * tt, 0 * tt, 0 * tt],
           delimiter=',', header='time_s,x_m,y_m,z_m,vx_m_s,vy_m_s,vz_m_s,ax_m_s2,ay_m_s2,az_m_s2', comments='')
calls.clear(); rehearse.fly(tr, room=2.0)
pos = [c for c in calls if c[0] == 'send_position_setpoint']
assert max(c[1] for c in pos) > 0.8 and calls[-1] == ('send_stop_setpoint',)
print('rehearsal dry run: all checks passed')
