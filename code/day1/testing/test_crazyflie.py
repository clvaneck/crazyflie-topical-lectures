"""Tests for crazyflie.py without a drone.

Run from the Day 1 folder: python testing/test_crazyflie.py     (tiny inline runner, as in test_module.py)
Or:                        pytest testing/test_crazyflie.py

A fake cflib is installed into sys.modules before crazyflie is imported, so the
tests never touch a radio even when the real cflib is installed. The fake drone
is a point mass whose velocity follows the velocity setpoint with a first order
lag of 0.3 s, which is roughly what the onboard velocity loop does. The tests
check the loop logic: timing, clamping, the step, the abort conditions, saving,
loading and the step numbers.
"""
import math
import os
import shutil
import sys
import tempfile
import threading
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # the Day 1 folder
sys.path.insert(0, HERE)


# ---------------------------------------------------------------------------
# A fake cflib: just enough of the API that crazyflie.py uses
# ---------------------------------------------------------------------------

class FakeWorld:
    """Point mass in x, y. Velocity follows the setpoint with a first order lag.

    While `hand` is set, the position is dictated by that function of time
    instead (the drone is being moved by hand, as in the hand test).
    """

    def __init__(self):
        self.x = self.y = self.vx = self.vy = 0.0
        self.v_set = (0.0, 0.0)
        self.tau = 0.3
        self.vbat = 3.9
        self.var = 0.001
        self.hand = None
        self.flow_alive = True        # False: a deck whose optical flow sensor sees nothing
        self._flow_ref = (0.0, 0.0)   # position at the last flow reading: the sensor reports motion since then
        self.t = 0.0
        self.lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def _run(self):
        last = time.monotonic()
        while not self._stop.is_set():
            now = time.monotonic()
            dt = min(0.05, now - last)   # real elapsed time, so the plant runs in real time
            last = now
            with self.lock:
                self.t += dt
                if self.hand is not None:
                    x, y = self.hand(self.t)
                    self.vx, self.vy = (x - self.x) / dt, (y - self.y) / dt
                    self.x, self.y = x, y
                else:
                    self.vx += (self.v_set[0] - self.vx) * dt / self.tau
                    self.vy += (self.v_set[1] - self.vy) * dt / self.tau
                    self.x += self.vx * dt
                    self.y += self.vy * dt
                    if not self.flow_alive:
                        self.x += 0.1 * dt    # a blind deck: the estimate runs away on the ground
            time.sleep(0.005)

    def sample(self, names):
        with self.lock:
            dx, dy = 0.0, 0.0
            if any(n.startswith('motion.') for n in names):     # accumulated motion since the last flow reading
                dx, dy = self.x - self._flow_ref[0], self.y - self._flow_ref[1]
                self._flow_ref = (self.x, self.y)
            values = {'stateEstimate.x': self.x, 'stateEstimate.y': self.y,
                      'stateEstimate.vx': self.vx, 'stateEstimate.vy': self.vy,
                      'kalman.varPX': self.var, 'kalman.varPY': self.var,
                      'pm.vbat': self.vbat, 'pm.state': 0, 'supervisor.info': 1,
                      'stateEstimate.z': 0.5,
                      'range.zrange': (500 + int(self.t * 50) % 3) if self.flow_alive else 5,   # mm, jitters when alive
                      'motion.deltaX': int(round(500 * dx)) if self.flow_alive else 0,
                      'motion.deltaY': int(round(500 * dy)) if self.flow_alive else 0,
                      'motion.squal': 90 if self.flow_alive else 0}
            return int(self.t * 1000), {n: values.get(n, 0.0) for n in names}


WORLD = FakeWorld()


class _Callbacks:
    def __init__(self):
        self.cbs = []

    def add_callback(self, cb):
        self.cbs.append(cb)


class FakeLogConfig:
    def __init__(self, name, period_in_ms):
        self.name = name
        self.period = period_in_ms / 1000.0
        self.names = []
        self.data_received_cb = _Callbacks()
        self._stop = threading.Event()

    def add_variable(self, name, ctype):
        self.names.append(name)

    def start(self):
        self._stop.clear()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop.is_set():
            ts, data = WORLD.sample(self.names)
            for cb in self.data_received_cb.cbs:
                cb(ts, data, self)
            time.sleep(self.period)

    def stop(self):
        self._stop.set()

    def delete(self):
        pass


class _Toc:
    def get_element_by_complete_name(self, name):
        return name


class _Log:
    toc = _Toc()

    def add_config(self, conf):
        pass


class _Param:
    toc = _Toc()

    def __init__(self):
        self.values = {'deck.bcFlow2': '1'}
        self.history = []

    def set_value(self, name, value):
        self.values[name] = value
        self.history.append((name, value))

    def get_value(self, name, timeout=None):
        return self.values.get(name)

    def add_update_callback(self, group, name, cb):
        pass

    def request_param_update(self, name):
        pass


class _Commander:
    def send_stop_setpoint(self):
        WORLD.v_set = (0.0, 0.0)


class _Supervisor:
    armed = False

    def send_arming_request(self, flag):
        self.armed = flag

    def send_crash_recovery_request(self):
        pass


class FakeConfigBlock:
    """The EEPROM block with the radio configuration, as cflib's I2CElement presents it."""
    def __init__(self):
        self.elements = {'version': 1, 'radio_channel': 80, 'radio_speed': 2,
                         'pitch_trim': 0.0, 'roll_trim': 0.0, 'radio_address': 0xE7E7E7E7E7}
        self.written = None

    def update(self, cb):
        cb(self)

    def write_data(self, cb):
        self.written = dict(self.elements)
        cb(self, 0)


class _Mem:
    def __init__(self):
        self.block = FakeConfigBlock()

    def get_mems(self, mem_type):
        return [self.block]


class FakeMemoryElement:
    TYPE_I2C = 0


class FakeCrazyflie:
    last = None                      # the most recently created instance, for checks after connect()

    def __init__(self, rw_cache=None):
        FakeCrazyflie.last = self
        self.mem = _Mem()
        self.param = _Param()
        self.log = _Log()
        self.commander = _Commander()
        self.supervisor = _Supervisor()


class FakeSyncCrazyflie:
    def __init__(self, uri, cf=None):
        self.cf = cf
        if cf is not None:
            cf.link_uri = uri

    def __enter__(self):
        WORLD.start()
        return self

    def __exit__(self, *exc):
        WORLD.stop()
        return False


class FakeMotionCommander:
    calls = []

    def __init__(self, scf, default_height=0.3):
        self.height = default_height

    def __enter__(self):
        FakeMotionCommander.calls.append(('takeoff', self.height))
        return self

    def __exit__(self, *exc):
        FakeMotionCommander.calls.append(('exit',))
        return False

    def start_linear_motion(self, vx, vy, vz, rate_yaw=0.0):
        WORLD.v_set = (vx, vy)
        if vz or rate_yaw:
            FakeMotionCommander.calls.append(('linear', vx, vy, vz, rate_yaw))

    def down(self, distance_m, velocity=0.2):
        FakeMotionCommander.calls.append(('down', distance_m))

    def land(self, velocity=0.2):
        WORLD.v_set = (0.0, 0.0)
        FakeMotionCommander.calls.append(('land', velocity))

    def start_turn_left(self, rate=72):
        WORLD.v_set = (0.0, 0.0)
        FakeMotionCommander.calls.append(('turn_left', rate))

    def start_turn_right(self, rate=72):
        WORLD.v_set = (0.0, 0.0)
        FakeMotionCommander.calls.append(('turn_right', rate))

    def stop(self):
        WORLD.v_set = (0.0, 0.0)


class FakeSharedRadio:
    """The dongle: a ping is acknowledged when the address is in DRONES_IN_RANGE on the set channel."""
    channel = None

    def set_channel(self, channel):
        self.channel = channel

    def set_data_rate(self, dr):
        self.dr = dr

    def set_address(self, address):
        self.address = int.from_bytes(bytes(address), 'big')

    def send_packet(self, data):
        ack = types.SimpleNamespace(ack=DRONES_IN_RANGE.get(self.address) == self.channel, data=())
        return ack

    def scan_channels(self, start, stop, packet):
        """The firmware scan of the dongle: the channels on which the set address answers."""
        ch = DRONES_IN_RANGE.get(self.address)
        return (ch,) if ch is not None and start <= ch <= stop else ()

    def close(self):
        pass


class FakeRadioManager:
    @staticmethod
    def open(devid):
        return FakeSharedRadio()


class FakeCrazyradio:
    DR_250KPS, DR_1MPS, DR_2MPS = 0, 1, 2


def _install_fake_cflib():
    cflib = types.ModuleType('cflib')
    crtp = types.ModuleType('cflib.crtp')
    radiodriver = types.ModuleType('cflib.crtp.radiodriver')
    radiodriver.RadioManager = FakeRadioManager
    crtp.radiodriver = radiodriver
    drivers = types.ModuleType('cflib.drivers')
    crazyradio = types.ModuleType('cflib.drivers.crazyradio')
    crazyradio.Crazyradio = FakeCrazyradio
    drivers.crazyradio = crazyradio
    cflib.drivers = drivers
    crtp.init_drivers = lambda: None
    crtp.scan_interfaces = lambda address=None: (
        [['radio://0/%d/2M/%010X' % (DRONES_IN_RANGE[address], address), '']] if address in DRONES_IN_RANGE else [])
    crazyflie = types.ModuleType('cflib.crazyflie')
    crazyflie.Crazyflie = FakeCrazyflie
    sync = types.ModuleType('cflib.crazyflie.syncCrazyflie')
    sync.SyncCrazyflie = FakeSyncCrazyflie
    log = types.ModuleType('cflib.crazyflie.log')
    log.LogConfig = FakeLogConfig
    mem = types.ModuleType('cflib.crazyflie.mem')
    mem.MemoryElement = FakeMemoryElement
    crazyflie.mem = mem
    positioning = types.ModuleType('cflib.positioning')
    motion = types.ModuleType('cflib.positioning.motion_commander')
    motion.MotionCommander = FakeMotionCommander
    cflib.crtp, cflib.crazyflie, cflib.positioning = crtp, crazyflie, positioning
    crazyflie.syncCrazyflie, crazyflie.log = sync, log
    positioning.motion_commander = motion
    for name, mod in [('cflib', cflib), ('cflib.crtp', crtp), ('cflib.crazyflie', crazyflie),
                      ('cflib.crtp.radiodriver', radiodriver), ('cflib.drivers', drivers),
                      ('cflib.crazyflie.mem', mem),
                      ('cflib.drivers.crazyradio', crazyradio),
                      ('cflib.crazyflie.syncCrazyflie', sync), ('cflib.crazyflie.log', log),
                      ('cflib.positioning', positioning),
                      ('cflib.positioning.motion_commander', motion)]:
        sys.modules[name] = mod


# What the fake radio hears, address: channel. The test drone with the factory address on channel 80,
# label 7 on the channel of its label (51), label 3 not yet moved, still on channel 80.
DRONES_IN_RANGE = {0xE7E7E7E7E7: 80, 0xE7E7E7E707: 51, 0xE7E7E7E703: 80}
TEST_URI = 'radio://0/80/2M/E7E7E7E7E7'
_install_fake_cflib()

import crazyflie   # noqa: E402  (after the fake cflib is in place)
import module      # noqa: E402


def _reset_world(**kw):
    global WORLD
    WORLD.stop()
    WORLD = FakeWorld()
    for k, v in kw.items():
        setattr(WORLD, k, v)
    FakeMotionCommander.calls = []


def make_controller(kp=-1.2, kd=-0.6, ki=0.0, v_max=0.3, dt=0.02):
    """The controller the notebooks write: one module.PIDController per axis."""
    x_ctrl = module.PIDController(Kp=kp, Ki=ki, Kd=kd, deltaT=dt)
    y_ctrl = module.PIDController(Kp=kp, Ki=ki, Kd=kd, deltaT=dt)

    def controller(x, y, vx, vy, x_target, y_target):
        vx_fb = float(np.clip(x_ctrl(x, vx, x_target), -v_max, v_max))
        vy_fb = float(np.clip(y_ctrl(y, vy, y_target), -v_max, v_max))
        return vx_fb, vy_fb

    def reset():
        x_ctrl.reset()
        y_ctrl.reset()

    return controller, reset


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def _raises(exc, fn, *args, **kw):
    try:
        fn(*args, **kw)
    except exc:
        return True
    assert False, 'expected %s' % exc.__name__


def test_resolve_uri_needs_radio_address():
    assert _raises(crazyflie.DroneError, crazyflie.resolve_uri, None)
    assert _raises(crazyflie.DroneError, crazyflie.resolve_uri, 'usb://0')
    assert crazyflie.resolve_uri('radio://0/80/2M/E7E7E7E701') == 'radio://0/80/2M/E7E7E7E701'


def test_drone_channel_follows_the_label():
    assert crazyflie.drone_channel(1) == 9
    assert crazyflie.drone_channel(2) == 16
    assert crazyflie.drone_channel('A') == 72
    assert crazyflie.drone_channel('B') == 79          # the highest label of the course, inside the band
    assert crazyflie.drone_channel('E7') == 80         # the factory address keeps the factory channel
    for bad in ('C', 15, -1, None, 'G'):
        assert _raises(crazyflie.DroneError, crazyflie.drone_channel, bad)


def test_drone_uri_from_label_address_or_uri():
    assert crazyflie.drone_uri(2) == 'radio://0/16/2M/E7E7E7E702'
    assert crazyflie.drone_uri('2') == 'radio://0/16/2M/E7E7E7E702'
    assert crazyflie.drone_uri('A') == 'radio://0/72/2M/E7E7E7E70A'
    assert crazyflie.drone_uri('b') == 'radio://0/79/2M/E7E7E7E70B'
    assert crazyflie.drone_uri('E7') == 'radio://0/80/2M/E7E7E7E7E7'
    assert crazyflie.drone_uri(2, channel=80) == 'radio://0/80/2M/E7E7E7E702'      # not moved yet
    assert crazyflie.drone_uri('f', channel=80) == 'radio://0/80/2M/E7E7E7E70F'    # outside the course scheme
    assert _raises(crazyflie.DroneError, crazyflie.drone_uri, 'f')
    for bad in (None, -1, 256, 'G', 'seven', '', 'E7E7E7E7E7', 'radio://0/80/2M/E7E7E7E7E7'):
        assert _raises(crazyflie.DroneError, crazyflie.drone_uri, bad)
    assert crazyflie.drone_label('radio://0/16/2M/E7E7E7E702') == '2'


def test_zero_label_uses_channel_two():
    for label in (0, '0', '00'):
        assert crazyflie.drone_channel(label) == 2
        assert crazyflie.drone_uri(label) == 'radio://0/2/2M/E7E7E7E700'
        assert crazyflie.drone_uri(label, channel=80) == 'radio://0/80/2M/E7E7E7E700'
    assert crazyflie.drone_label('radio://0/2/2M/E7E7E7E700') == '0'


def test_zero_label_is_found_by_default_and_explicit_discovery():
    address = 0xE7E7E7E700
    expected = 'radio://0/2/2M/E7E7E7E700'
    DRONES_IN_RANGE[address] = 2
    try:
        assert crazyflie.find_drones()[0] == expected
        assert crazyflie.find_drones(factory=False)[0] == expected
        assert crazyflie.find_drones(channel=2, factory=False) == {0: expected}
        for label in (0, '0', '00'):
            assert crazyflie.find_drones(label) == {0: expected}
            assert crazyflie.find_drone(label) == expected
    finally:
        del DRONES_IN_RANGE[address]


def test_find_drones_scans_all_channels():
    # label 7 on the channel of its label, label 3 still on the factory channel: both are found
    assert crazyflie.find_drones(factory=False) == {3: 'radio://0/80/2M/E7E7E7E703', 7: 'radio://0/51/2M/E7E7E7E707'}
    assert crazyflie.find_drones(labels=range(4, 7), factory=False) == {}
    assert crazyflie.find_drones() == {0xE7: 'radio://0/80/2M/E7E7E7E7E7', 3: 'radio://0/80/2M/E7E7E7E703',
                                       7: 'radio://0/51/2M/E7E7E7E707'}
    # one channel only: a plain ping of every label on that channel
    assert crazyflie.find_drones(channel=80, factory=False) == {3: 'radio://0/80/2M/E7E7E7E703'}
    assert crazyflie.find_drones(channel=51, factory=False) == {7: 'radio://0/51/2M/E7E7E7E707'}
    # one label, as a number or a string: that drone alone, the factory address only when asked for
    assert crazyflie.find_drones('7') == {7: 'radio://0/51/2M/E7E7E7E707'}
    assert crazyflie.find_drones(3) == {3: 'radio://0/80/2M/E7E7E7E703'}
    assert crazyflie.find_drones('E7') == {0xE7: 'radio://0/80/2M/E7E7E7E7E7'}
    assert crazyflie.find_drones(5) == {}
    assert _raises(crazyflie.DroneError, crazyflie.find_drones, 'G')


def test_connect_refuses_a_silent_address_in_one_line():
    assert crazyflie.ping(TEST_URI) is True
    assert crazyflie.ping('radio://0/80/2M/E7E7E7E708') is False
    assert crazyflie.ping('radio://0/80/2M/E7E7E7E707') is False      # label 7 is on its own channel
    assert crazyflie.ping('radio://0/51/2M/E7E7E7E707') is True
    try:
        with crazyflie.connect('radio://0/80/2M/E7E7E7E708'):
            assert False, 'expected DroneError'
    except crazyflie.DroneError as exc:
        assert 'No drone answers at radio://0/80/2M/E7E7E7E708' in str(exc)
        lines = exc._render_traceback_()           # what a notebook shows: one line
        assert len(lines) == 1 and 'No drone answers' in lines[0] and 'DroneError' in lines[0]
    assert _raises(crazyflie.DroneError, crazyflie.ping, 'radio://0/80/2M/E7E7')


def test_set_label_writes_the_configuration_block():
    _reset_world()
    with crazyflie.connect(TEST_URI) as scf:
        assert crazyflie.config_uri(crazyflie.read_config(scf)) == 'radio://0/80/2M/E7E7E7E7E7'
        assert crazyflie.set_label(scf, 2) == 'radio://0/16/2M/E7E7E7E702'
        written = scf.cf.mem.block.written
    assert written['radio_address'] == 0xE7E7E7E702
    assert written['radio_channel'] == 16 and written['radio_speed'] == 2 and written['version'] == 1
    assert written['pitch_trim'] == 0.0 and written['roll_trim'] == 0.0


def test_set_label_zero_writes_the_configuration_block():
    _reset_world()
    with crazyflie.connect(TEST_URI) as scf:
        assert crazyflie.set_label(scf, 0) == 'radio://0/2/2M/E7E7E7E700'
        written = scf.cf.mem.block.written
    assert written['radio_address'] == 0xE7E7E7E700
    assert written['radio_channel'] == 2 and written['radio_speed'] == 2


def test_set_channel_finds_the_drone_on_the_factory_channel_and_moves_it():
    _reset_world()
    FakeCrazyflie.last = None
    # label 3 answers on channel 80: its block is written with channel 23 and the same address
    assert crazyflie.set_channel(3) == 'radio://0/23/2M/E7E7E7E703'
    written = FakeCrazyflie.last.mem.block.written
    assert written['radio_channel'] == 23 and written['radio_address'] == 0xE7E7E7E703
    # label 7 already answers on its channel: nothing is written
    FakeCrazyflie.last = None
    assert crazyflie.set_channel(7) == 'radio://0/51/2M/E7E7E7E707'
    assert FakeCrazyflie.last is None
    # label 5 answers nowhere
    assert _raises(crazyflie.DroneError, crazyflie.set_channel, 5)


def _scripted_hand(distance=0.3, sign=1, speed=0.6):
    """The hand of a student who follows the printed protocol, moving at `speed` m/s; sign -1 mirrors both moves."""
    d = sign * distance
    positions = {'still': (0, 0), 'lift': (0, 0), 'forward': (d, 0), 'left': (d, d), 'down': (d, d)}
    state = {'pos': [0.0, 0.0], 't': None}

    def hand(t):
        goal = positions.get(crazyflie.CURRENT_PHASE['phase'], (0, 0))
        if state['t'] is None:
            state['t'] = t
        room = speed * max(0.0, t - state['t'])      # how far the hand gets in this step
        state['t'] = t
        for i in (0, 1):
            state['pos'][i] += max(-room, min(room, goal[i] - state['pos'][i]))
        return tuple(state['pos'])
    return hand


def test_identify_sensors_follows_the_protocol():
    phase_s = 0.8
    _reset_world(hand=_scripted_hand())
    tmp = tempfile.mkdtemp()
    phases = [(name, phase_s, text) for name, _s, text in crazyflie.SENSOR_ID_PHASES]
    # by default nothing is written; here both files are asked for
    samples, r = crazyflie.identify_sensors(uri=TEST_URI, phases=phases,
                                            result=os.path.join(tmp, 'drone_id.csv'),
                                            out=os.path.join(tmp, 'sensor_id_E7.csv'))
    assert sorted(os.listdir(tmp)) == ['drone_id.csv', 'sensor_id_E7.csv'], os.listdir(tmp)
    raw = crazyflie.load_sensor_id(os.path.join(tmp, 'sensor_id_E7.csv'))
    assert list(raw['phase']) == list(samples['phase']) and len(raw['x']) == len(samples['x'])
    assert set(samples['phase']) >= {'still', 'lift', 'forward', 'left', 'down'}
    assert set(r) == {'uri', 'time', 'rate_hz', 'still_s', 'std_x_m', 'std_y_m', 'std_vx_mps',
                      'std_vy_mps', 'drift_x_m', 'drift_y_m', 'x_sign', 'y_sign',
                      'flow_counts_max', 'squal_median', 'zrange_lift_m', 'z_lift_m'}
    assert abs(r['zrange_lift_m'] - 0.5) < 0.01 and abs(r['z_lift_m'] - 0.5) < 0.01
    assert r['flow_counts_max'] > 0 and r['squal_median'] == 90, (r['flow_counts_max'], r['squal_median'])
    assert set(raw) == set(crazyflie.SENSOR_ID_COLUMNS) and abs(raw['zrange_m'][0] - 0.5) < 0.005
    assert r['x_sign'] == 1 and r['y_sign'] == 1
    assert r['std_x_m'] < 0.01 and r['std_vx_mps'] < 0.05 and abs(r['drift_x_m']) < 0.01
    loaded = crazyflie.load_drone_id(os.path.join(tmp, 'drone_id.csv'))
    assert loaded == r
    with open(os.path.join(tmp, 'drone_id.csv')) as f:
        assert f.readline().strip() == 'name,value,comment'
    assert crazyflie.drone_label('radio://0/80/2M/E7E7E7E70A') == 'A' and crazyflie.drone_label(TEST_URI) == 'E7'
    limits = crazyflie.gain_limits(r)
    assert limits['Kp_max'] > 0 and limits['Kd_max'] > 0
    crazyflie.print_drone_id(r)
    assert _raises(crazyflie.DroneError, crazyflie.load_drone_id, os.path.join(tmp, 'missing.csv'))
    shutil.rmtree(tmp)
    _reset_world(hand=_scripted_hand(sign=-1))          # a drone whose estimate counts the other way
    _s, flipped = crazyflie.identify_sensors(uri=TEST_URI, phases=phases, result=None, out=False)
    assert flipped['x_sign'] == -1 and flipped['y_sign'] == -1
    _reset_world(hand=_scripted_hand(), flow_alive=False)   # a deck whose flow sensor is blind
    tmp = tempfile.mkdtemp()
    _s, blind = crazyflie.identify_sensors(uri=TEST_URI, phases=phases,
                                           result=os.path.join(tmp, 'drone_id.csv'), out=False)
    assert blind['flow_counts_max'] == 0 and blind['squal_median'] == 0 and blind['zrange_lift_m'] < 0.05
    # the check before arming: estimate 0.5 m against a sensor stuck at 5 mm
    assert _raises(crazyflie.DroneError, crazyflie.fly, make_controller()[0], uri=TEST_URI, drone_id=None,
                   stop_event=threading.Event())
    _reset_world()
    assert _raises(crazyflie.DroneError, crazyflie.fly, make_controller()[0], uri=TEST_URI,
                   drone_id=os.path.join(tmp, 'drone_id.csv'), stop_event=threading.Event())
    shutil.rmtree(tmp)


def test_fly_checks_the_sensor_identification_when_given():
    _reset_world()
    controller, reset = make_controller()
    assert _raises(crazyflie.DroneError, crazyflie.fly, controller, uri=TEST_URI,
                   drone_id=os.path.join(tempfile.gettempdir(), 'no_such_drone_id.csv'))
    tmp = tempfile.mkdtemp()
    path = os.path.join(tmp, 'drone_id.csv')
    crazyflie.save_drone_id({'x_sign': 1, 'y_sign': 1}, path)
    run = crazyflie.fly(controller, uri=TEST_URI, drone_id=path, hold_s=0.3, step_s=0.3,
                        stop_event=threading.Event())
    assert run['aborted'] is None
    shutil.rmtree(tmp)


def test_fly_model_follows_the_design_and_saves_like_a_flight():
    controller, reset = make_controller()                      # Kp -1.2, Kd -0.6
    run = crazyflie.fly_model(controller, reset=reset, tau=0.3, drone_id=None, noise_scale=0.0, quiet=True)
    assert run['aborted'] is None and run['drone'] == 'model'
    rows = np.asarray(run['rows'], dtype=float)
    assert abs(rows[-1, 3] - rows[-1, 7]) < 0.03, rows[-1, 3]        # settled at the stepped target
    t_m, x_m, _, _ = crazyflie.model_step_response(-1.2, -0.6, 0.3)   # the same model without noise
    # fly_model logs absolute time; model_step_response uses time since the step.
    step_time = rows[rows[:, 2] > 0.5, 0][0]
    i, j = np.searchsorted(rows[:, 0], step_time + 2.0), np.searchsorted(t_m, 2.0)
    assert abs((rows[i, 3] - rows[0, 3]) - x_m[j]) < 0.02, (rows[i, 3], x_m[j])
    tmp = tempfile.mkdtemp()
    try:
        name = crazyflie.save(run, 'test', data_dir=tmp)
        assert name.startswith('model_') and name.endswith('_test'), name
        loaded, = crazyflie.load_runs(tmp)
        assert loaded['drone'] == 'model' and loaded['run_label'] == 'test' and loaded['axes'] == 'x'
        assert loaded['info']['tau'] == 0.3 and loaded['info']['delay'] == 0 and loaded['info']['drone'] == 'model'
        assert crazyflie.load_runs(tmp, drone='model')[0]['label'] == name
        numbers = crazyflie.step_numbers(loaded, quiet=True)
        assert numbers['x']['settled'] and numbers['x']['overshoot_m'] < 0.05
        hot, _ = make_controller(kp=-8.0, kd=-0.5)                 # a fast loop, without and with dead time
        quick = crazyflie.fly_model(hot, tau=0.3, delay=0.0, drone_id=None, noise_scale=0.0, quiet=True)
        hot, _ = make_controller(kp=-8.0, kd=-0.5)
        run2 = crazyflie.fly_model(hot, tau=0.3, delay=0.3, drone_id=None, noise_scale=0.0, quiet=True)
        name2 = crazyflie.save(run2, 'hot', data_dir=tmp)
        hot_numbers = crazyflie.step_numbers(crazyflie.load_run(name2, tmp), quiet=True)
        quick_over = np.max(np.asarray(quick['rows'], dtype=float)[:, 3]) - 0.5
        assert run2['aborted'] or hot_numbers['x']['overshoot_m'] > quick_over + 0.03, (quick_over, hot_numbers)
        assert crazyflie.saved_runs(tmp, match='hot') == [name2]
        early = dict(run, rows=[], aborted='stopped by abort')       # a flight ended before the handover
        name3 = crazyflie.save(early, 'early', data_dir=tmp)
        loaded3 = crazyflie.load_run(name3, tmp)
        assert loaded3['t_step'] is None and loaded3['axes'] == ''
    finally:
        shutil.rmtree(tmp)
    noisy = crazyflie.fly_model(controller, reset=reset, tau=0.3, drone_id=None, noise_scale=5.0, seed=1, quiet=True)
    assert np.std(np.asarray(noisy['rows'], dtype=float)[:, 9]) > np.std(rows[:, 9])   # noisier feedback


def test_fly_path_under_the_firmware_logs_like_a_flight():
    _reset_world()
    run = crazyflie.fly_path([('forward', 0.5), ('hold', 0.3), ('left', 0.3), ('turn', 90), ('back', 0.2)],
                             uri=TEST_URI, speed=0.5, hold_s=0.3, hover_s=0.2, stop_event=threading.Event())
    assert run['aborted'] is None and run['drone'] == 'E7' and run['path'][0] == ('forward', 0.5)
    rows = np.asarray(run['rows'], dtype=float)
    assert rows[0, 2] == 0 and rows[-1, 2] == 1                       # hover first, then the path
    first_move = rows[rows[:, 2] > 0.5][0, 0]
    after_forward = rows[(rows[:, 0] > first_move + 1.0) & (rows[:, 0] < first_move + 1.3)]
    assert abs(after_forward[:, 7].mean() - 0.5) < 0.02, after_forward[:, 7].mean()   # planned x after 0.5 m forward
    assert after_forward[:, 3].mean() > 0.3, after_forward[:, 3].mean()               # the fake drone followed
    assert ('turn_left', 45.0) in FakeMotionCommander.calls and ('land', 0.2) in FakeMotionCommander.calls
    tmp = tempfile.mkdtemp()
    try:
        name = crazyflie.save(run, 'firmware_test', data_dir=tmp)
        loaded = crazyflie.load_run(name, tmp)
        assert loaded['axes'] == 'xy' and loaded['t_step'] is not None and loaded['run_label'] == 'firmware_test'
    finally:
        shutil.rmtree(tmp)
    _reset_world()
    stop = threading.Event()
    threading.Timer(4.5, stop.set).start()
    run = crazyflie.fly_path([('forward', 2.0)], uri=TEST_URI, speed=0.2, hover_s=0.2, stop_event=stop)
    assert run['aborted'] == 'stopped by abort', run['aborted']
    assert _raises(crazyflie.DroneError, crazyflie.fly_path, [('sideways', 1.0)], uri=TEST_URI)


def test_fly_path_helix():
    _reset_world()
    FakeMotionCommander.calls.clear()
    run = crazyflie.fly_path([('helix', 0.2, 0.3, 1.0)], uri=TEST_URI, speed=0.4, hold_s=0.3, hover_s=0.2,
                             stop_event=threading.Event())
    assert run['aborted'] is None, run['aborted']
    linear = [c for c in FakeMotionCommander.calls if c[0] == 'linear']
    assert len(linear) == 1
    _, vx, vy, vz, rate_yaw = linear[0]
    duration = 2 * math.pi * 0.2 / 0.4
    assert abs(vx - 0.4) < 1e-9 and vy == 0.0 and abs(vz - 0.3 / duration) < 1e-9
    assert abs(rate_yaw - math.degrees(0.4 / 0.2)) < 1e-9          # left, a circle of 0.2 m
    rows = np.asarray(run['rows'], dtype=float)
    planned = rows[rows[:, 2] > 0.5][:, 7:9] - rows[0, 7:9]
    assert abs(planned[:, 1].max() - 0.4) < 0.03, planned[:, 1].max()   # the circle reaches 2 r to the left
    assert np.hypot(*planned[-1]) < 0.03                                 # and closes after one turn
    down = [c for c in FakeMotionCommander.calls if c[0] == 'down']
    assert abs(down[-1][1] - (0.5 + 0.3 - 0.1)) < 0.03, down               # landing starts from the climbed height
    assert run['info']['moves'] == 'helix 0.2 0.3 1'
    assert _raises(crazyflie.DroneError, crazyflie.fly_path, [('helix', 0.2, 0.3)], uri=TEST_URI)
    assert _raises(crazyflie.DroneError, crazyflie.fly_path, [('helix', 0.2, 2.0, 1)], uri=TEST_URI)
    assert _raises(crazyflie.DroneError, crazyflie.fly_path, [('down', 0.4)], uri=TEST_URI)


def test_identify_response_and_the_fit():
    _reset_world()
    run = crazyflie.identify_response(step=0.2, step_s=1.2, settle_s=0.4, uri=TEST_URI,
                                      stop_event=threading.Event(), drone_id=None)
    assert run['aborted'] is None and run['identify']['step'] == 0.2 and run['info']['v_step'] == 0.2
    fit = crazyflie.fit_tau(run)
    assert 0.15 < fit['tau_s'] < 0.5, fit['tau_s']          # the fake drone has tau = 0.3 s
    assert fit['delay_s'] < 0.15
    assert abs(fit['v_set'] - 0.2) < 1e-9
    vs = crazyflie.velocity_step(run)
    assert abs(vs['v_set'] - 0.2) < 1e-9 and len(vs['t']) == len(fit['t']) and vs['t'][0] == 0.0
    # the hover before the step gives the noise bounds of part 4, no file of part 3 needed
    noise = crazyflie.hover_noise(run)
    assert noise['n'] >= 10 and all(noise[k] >= 0.0 for k in ('std_x_m', 'std_y_m', 'std_vx_mps', 'std_vy_mps'))
    limits = crazyflie.gain_limits(noise)
    assert limits['Kp_max'] > 0 and limits['Kd_max'] > 0
    # the prediction in numbers
    pred = crazyflie.model_numbers(-1.2, -0.6, 0.3)
    assert pred['settled'] and pred['rise_s'] < 3.5
    slow = crazyflie.model_numbers(-0.13, 0.0, 0.13)             # omega0 = 1 rad/s on a fast velocity loop
    assert slow['rise_s'] is None or slow['rise_s'] > 3.0
    told = crazyflie.predict_step(-0.5, 0.0, 0.0, 0.13, 0.2, limits={'Kp_max': 25.0, 'Kd_max': 3.0})
    assert told == crazyflie.model_numbers(-0.5, 0.0, 0.13, delay=0.2)
    assert told['rise_s'] is None          # the default four-second step ends before reaching 90%
    longer = crazyflie.model_numbers(-0.5, 0.0, 0.13, delay=0.2, step_s=8.0)
    assert 3.5 < longer['rise_s'] < 4.5
    assert crazyflie.predict_step(-4.0, 0.3, -0.2, 0.13, 0.2, quiet=True)['overshoot_m'] > told['overshoot_m']
    late = crazyflie.model_numbers(-3.0, 0.0, 0.13, delay=0.3)   # dead time adds overshoot
    assert late['overshoot_m'] > crazyflie.model_numbers(-3.0, 0.0, 0.13)['overshoot_m']
    # the design formulas invert each other
    Kp, Kd = crazyflie.design_gains(0.3, omega0=2.0, zeta=0.7)
    omega0, zeta = crazyflie.closed_loop_parameters(Kp, Kd, 0.3)
    assert abs(omega0 - 2.0) < 1e-9 and abs(zeta - 0.7) < 1e-9
    omega0, zeta = crazyflie.closed_loop_parameters(-1.2, -0.6, 0.3)
    assert abs(omega0 - 2.0) < 1e-9 and abs(zeta - 1.3333) < 1e-3
    assert _raises(crazyflie.DroneError, crazyflie.closed_loop_parameters, 1.2, 0, 0.3)
    # the model settles at the step with the September gains
    t, x, v, fb = crazyflie.model_step_response(-1.2, -0.6, 0.3, step_s=8.0)
    assert abs(x[-1] - 0.5) < 0.01 and np.abs(fb).max() <= 0.3 + 1e-9
    assert x[t < 0].max() == 0.0


def test_find_drone_by_label_or_factory_address():
    assert crazyflie.find_drone(7) == 'radio://0/51/2M/E7E7E7E707'   # the search scans all channels
    assert _raises(crazyflie.DroneError, crazyflie.find_drone, 8)
    assert crazyflie.find_drone(address='E7E7E7E7E7') == 'radio://0/80/2M/E7E7E7E7E7'


def test_identify_blinks_and_returns_the_leds():
    import io
    import contextlib
    _reset_world()
    with crazyflie.connect(TEST_URI) as scf:
        assert crazyflie.identify(scf, blinks=2, period=0.02) is True
        sets = [v for name, v in scf.cf.param.history if name == 'led.bitmask']
        assert sets == [0xBF, 0x80, 0xBF, 0x80, 0]
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            crazyflie.status(scf, timeout=2.0)
            crazyflie.watch_height(scf, duration=0.3, period=0.1)
        text = out.getvalue()
    assert 'Drone status of ' + TEST_URI in text
    assert 'height sensor' in text


def test_clamp_and_finite():
    assert crazyflie._clip(2.0, 0.3) == 0.3
    assert crazyflie._clip(-2.0, 0.3) == -0.3
    fb = crazyflie._call_controller(lambda *a: (5.0, -5.0), 0, 0, 0, 0, (0, 0), 0.3)
    assert fb == (0.3, -0.3)
    try:
        crazyflie._call_controller(lambda *a: (float('nan'), 0.0), 0, 0, 0, 0, (0, 0), 0.3)
        assert False, 'expected ValueError'
    except ValueError:
        pass


def test_handtest_logs_and_opposes_displacement():
    tmp = tempfile.mkdtemp()
    try:
        # the hand moves the drone back and forth along x, 0.2 m amplitude;
        # with Kd = 0 the feedback is exactly Kp times the displacement
        _reset_world(hand=lambda t: (0.2 * np.sin(4.0 * t), 0.0))
        controller, reset = make_controller(kd=0.0)
        out = os.path.join(tmp, 'handtest.csv')
        d = crazyflie.handtest(controller, uri=TEST_URI, duration=2.0, out=out, period_ms=20)
        assert os.path.exists(out)
        assert len(d['x']) > 50, len(d['x'])
        slope = np.polyfit(d['x'], d['fb_x'], 1)[0]
        assert abs(slope + 1.2) < 0.1, slope
        assert np.all(np.abs(d['fb_x']) <= 0.3 + 1e-9)
        assert not FakeMotionCommander.calls          # never took off
    finally:
        shutil.rmtree(tmp)


def test_handtest_reports_controller_error():
    _reset_world()

    def bad(*args):
        raise KeyError('oops')
    try:
        crazyflie.handtest(bad, uri=TEST_URI, duration=1.0, out=os.path.join(tempfile.gettempdir(), 'x.csv'))
        assert False, 'expected the controller error to be raised'
    except KeyError:
        pass


def test_fly_step_and_save_roundtrip():
    tmp = tempfile.mkdtemp()
    try:
        _reset_world()
        controller, reset = make_controller()
        stop = threading.Event()   # external abort event, so no input() listener
        run = crazyflie.fly(controller, uri=TEST_URI, drone_id=None, axes='x', step=0.5, hold_s=0.5, step_s=6.0,
                            loop_dt=0.02, reset=reset, stop_event=stop, info=dict(Kp=-1.2, Kd=-0.6, Ki=0.0))
        assert run['aborted'] is None
        rows = np.array(run['rows'])
        assert len(rows) > 100, len(rows)
        # fixed 50 Hz schedule
        period = np.diff(rows[:, 0]).mean()
        assert abs(period - 0.02) < 0.004, period
        # the target steps by 0.5 m on x only, y stays at the start
        assert abs(np.ptp(rows[:, 7]) - 0.5) < 1e-9, np.ptp(rows[:, 7])
        assert np.ptp(rows[:, 8]) == 0.0, np.ptp(rows[:, 8])
        assert np.all(rows[:, 10] == 0.0)             # y not acted on
        assert np.all(np.abs(rows[:, 9]) <= 0.3 + 1e-9), np.abs(rows[:, 9]).max()
        # the fake drone ends close to the stepped target
        assert abs(rows[-1, 3] - rows[-1, 7]) < 0.05, rows[-1, 3]
        # take-off, controlled descent and landing happened in that order
        kinds = [c[0] for c in FakeMotionCommander.calls]
        assert kinds[:1] == ['takeoff'] and 'down' in kinds and 'land' in kinds, kinds

        name = crazyflie.save(run, 'test', data_dir=tmp)
        assert name.startswith('E7_') and name.endswith('_test'), name
        numbers = crazyflie.step_numbers(crazyflie.load_run(name, data_dir=tmp), quiet=True)
        for group in ('pid', 'state', 'sensor', 'power', 'land', 'info'):
            assert os.path.exists(os.path.join(tmp, 'flight_%s_%s.csv' % (name, group)))
        assert numbers['x']['settled'], numbers['x']
        # 0.45 m at 0.3 m/s plus the 0.3 s lag of the fake plant: about 3 s
        assert numbers['x']['rise_s'] is not None and numbers['x']['rise_s'] < 4.5, numbers['x']
        assert crazyflie.saved_runs(tmp) == [name] and crazyflie.saved_runs(tmp, match='_test') == [name]
        assert crazyflie.saved_runs(tmp, match='identify') == []
        loaded = crazyflie.load_run(name, data_dir=tmp)
        assert loaded['axes'] == 'x'
        # the settings of the run come back with it, numbers as numbers
        info = loaded['info']
        assert info['Kp'] == -1.2 and info['Kd'] == -0.6 and info['Ki'] == 0 and info['v_max'] == 0.3
        assert info['drone'] == 'E7' and info['run_label'] == 'test' and info['axes'] == 'x' and info['aborted'] == ''
        assert crazyflie.saved_runs(tmp, drone='E7') == [name] and crazyflie.saved_runs(tmp, drone='model') == []
        table = crazyflie.run_table([loaded], quiet=True)
        assert len(table) == 1 and table[0]['Kp'] == -1.2 and table[0]['run'] == name and table[0]['settled']
        crazyflie.run_table([loaded])                 # prints one line per axis
        assert abs(loaded['t_step'] - 0.5) < 0.05
        assert len(loaded['pid']['x']) == len(rows)
        again = crazyflie.step_numbers(loaded, quiet=True)
        assert again['x']['settled'] == numbers['x']['settled']
    finally:
        shutil.rmtree(tmp)


def test_fly_aborts_on_fence():
    _reset_world(hand=lambda t: (0.6 * max(0.0, t - 6.0), 0.0))   # still on the ground, then dragged away at 0.6 m/s
    controller, reset = make_controller()
    run = crazyflie.fly(controller, uri=TEST_URI, drone_id=None, axes='x', hold_s=5.0, step_s=1.0, stop_event=threading.Event())
    assert run['aborted'] and run['aborted'].startswith('fence'), run['aborted']
    assert ('land', 0.2) in FakeMotionCommander.calls


def test_fly_refuses_takeoff_when_stop_event_is_already_set():
    _reset_world()
    controller, reset = make_controller()
    stop = threading.Event()
    stop.set()
    try:
        crazyflie.fly(controller, uri=TEST_URI, drone_id=None, stop_event=stop)
        assert False, 'expected abort before take-off'
    except crazyflie.DroneError as exc:
        assert 'before the take-off' in str(exc), str(exc)
    assert FakeMotionCommander.calls == [], FakeMotionCommander.calls


def test_fly_aborts_on_stop_event():
    _reset_world()
    controller, reset = make_controller()
    stop = threading.Event()

    def stop_during_feedback(*args):
        stop.set()                   # request the abort after take-off, without a wall-clock race
        return controller(*args)

    run = crazyflie.fly(stop_during_feedback, uri=TEST_URI, drone_id=None, axes='xy',
                        hold_s=5.0, step_s=1.0, stop_event=stop)
    assert run['aborted'] == 'stopped by abort', run['aborted']
    kinds = [call[0] for call in FakeMotionCommander.calls]
    assert kinds[0] == 'takeoff' and 'land' in kinds, kinds


def test_fly_aborts_on_low_battery_and_bad_variance():
    _reset_world(vbat=3.0)
    controller, reset = make_controller()
    run = crazyflie.fly(controller, uri=TEST_URI, drone_id=None, hold_s=2.0, step_s=1.0, stop_event=threading.Event())
    assert run['aborted'] and run['aborted'].startswith('battery'), run['aborted']
    _reset_world(var=0.5)
    run = crazyflie.fly(controller, uri=TEST_URI, drone_id=None, hold_s=2.0, step_s=1.0, stop_event=threading.Event())
    assert run['aborted'] and 'variance' in run['aborted'], run['aborted']


def test_fly_aborts_when_controller_fails():
    _reset_world()

    def bad(x, y, vx, vy, xt, yt):
        return float('inf'), 0.0
    run = crazyflie.fly(bad, uri=TEST_URI, drone_id=None, hold_s=2.0, step_s=1.0, stop_event=threading.Event())
    assert run['aborted'] and run['aborted'].startswith('controller raised'), run['aborted']


def test_reference_runs_load():
    ref = os.path.join(HERE, 'data', 'reference')
    for label in ('reference_x', 'reference_xy'):
        run = crazyflie.load_run(label, data_dir=ref)
        numbers = crazyflie.step_numbers(run, quiet=True)
        assert run['t_step'] is not None
        assert set(run['axes']) == set(label.split('_')[1])
        for a in run['axes']:
            assert numbers[a]['rise_s'] is not None
            assert numbers[a]['settled'], (label, a, numbers[a])
    d = crazyflie.load_csv(os.path.join(ref, 'handtest_reference.csv'))
    assert np.polyfit(d['x'], d['fb_x'], 1)[0] < 0


# ---------------------------------------------------------------------------
# Tiny runner
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    tests = [(name, obj) for name, obj in sorted(globals().items())
             if name.startswith('test_') and callable(obj)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print('ok   ', name)
        except Exception as exc:
            failed += 1
            print('FAIL ', name, '->', type(exc).__name__, exc)
    print('\n%d / %d passed' % (len(tests) - failed, len(tests)))
    sys.exit(1 if failed else 0)
