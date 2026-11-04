"""Test the tuned 2Dsim_02-04 gains for Caspar's drone against the original gains.

Gains are read from the notebooks themselves, then every controller is flown in its
notebook scenario on unseen wind seeds with 0, 1 and 2 frames of actuation delay.
"""
import json, re, sys, random
from pathlib import Path
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
DAY1 = Path(__file__).resolve().parents[1]   # code/day1
sys.path.insert(0, str(DAY1)); import module
OUT = DAY1 / 'testing' / 'tuning_plots'
COLORS = ["#3f90da", "#ffa90e", "#bd1f01", "#94a4a2", "#832db6"]
plt.rcParams.update({"figure.facecolor": "white", "axes.facecolor": "white",
                     "savefig.facecolor": "white", "legend.frameon": False})
SEEDS = range(10, 20); DELAYS = (0, 1, 2); NAME = "Caspar"; T_HOVER_TRUE = 0.53311

# ---------------- 1. read gains from the notebooks ----------------
def src(nb, i): return ''.join(json.load(open(DAY1 / nb))['cells'][i]['source'])
def pid(s, var):
    m = re.search(rf'{var}\s*=\s*module\.PIDController\(Kp=([-\d.]+),\s*Ki=([-\d.]+),\s*Kd=([-\d.]+)', s)
    return tuple(float(x) for x in m.groups())
def pd(s, fn):
    body = s.split(f'def {fn}')[1]
    return float(re.search(r'p = ([-\d.]+)', body).group(1)), float(re.search(r'd = ([-\d.]+)', body).group(1))
N3, N4 = '2Dsim_03_basic_control.ipynb', '2Dsim_04_velocity_loop.ipynb'
c3 = src(N3, 3)
NEW = dict(
    off=float(re.search(r'V_left_offset = ([-\d.]+)', c3).group(1)),
    hov=float(re.search(r'V_hover = ([-\d.]+)', c3).group(1)),
    phi=pd(src(N3, 11), 'phi_damped_feedback'), zpd=pd(src(N3, 15), 'z_damped_feedback'),
    zpid=pid(src(N3, 25), 'z_ctrl'),
    casc=(pid(src(N3, 29), 'z_ctrl'), pid(src(N3, 29), 'y_ctrl'), pid(src(N3, 29), 'phi_ctrl')),
    ff=(pid(src(N3, 34), 'z_ctrl'), pid(src(N3, 34), 'y_ctrl'), pid(src(N3, 34), 'phi_ctrl')),
    vel=tuple(pid(src(N4, 5), v) for v in ('y_pos_ctrl', 'z_pos_ctrl', 'y_vel_ctrl', 'z_vel_ctrl', 'phi_ctrl')))
OLD = dict(off=-0.0027, hov=0.5297, phi=(0.1, 0.02), zpd=(-1.0, 0.5), zpid=(-0.3, -0.05, -0.3),
    casc=((-0.1, -0.005, -0.1), (0.2, 0.01, 0.2), (0.1, 0, 0.02)),
    ff=((-2.0, -0.1, -1.0), (0.5, 0.02, 0.3), (0.1, 0, 0.02)),
    vel=((-2.0, 0, 0), (-2.0, 0, 0), (0.5, 0.02, 0), (-3.0, -0.3, 0), (0.2, 0, 0.04)))
# The OLD set flies with the *correct* calibration too, so only the gains differ.
OLD['off'], OLD['hov'] = NEW['off'], NEW['hov']

# ---------------- 2. notebook scenarios ----------------
PID = module.PIDController
def mixer(drone, VL, VR, hov, off):
    VL, VR = VL + hov + off, VR + hov
    fz, fp = (VL + VR) / 2, (VL - VR) / 2
    fz = min(max(fz, 0), 1); h = min(fz, 1 - fz); fp = min(max(fp, -h), h)
    drone.set_V(fz + fp, fz - fp)

def fly(kind, G, seed, delay):
    random.seed(seed); off, hov = G['off'], G['hov']
    cfg = dict(phi=([0,0,0,0], 0, 0, True, 300), zpd=((0,-3,0,3), 0, 0, True, 1000),
               zpid=((0,-3,0,3), 4, 5, True, 3000), casc=((-3,-3,3,3), 4, 5, True, 1800),
               ff=((-3,-3,3,3), 4, 5, True, 1800), vel=((-5,-3,5,3), 4, 5, True, 600),
               vel0=((-5,-3,5,3), 0, 5, False, 480))[kind]
    d = module.Drone(name=NAME, wind=cfg[3], flight_range=cfg[0], drag_t=cfg[1], drag_r=cfg[2])
    out = np.zeros((cfg[4], 5)); q = []; VL = 0
    if kind in ('casc', 'ff'):
        zc, yc, pc = (PID(*g) for g in G[kind]); clip = 0.1*np.pi if kind == 'casc' else np.deg2rad(15)
    if kind.startswith('vel'):
        yp, zp, yv, zv, pc = (PID(*g) for g in G['vel']); tmax = np.deg2rad(20)
    if kind == 'zpid': zc = PID(*G['zpid'], int_clamp=20)
    for i in range(cfg[4]):
        s = d.update(); out[i] = [s.t, s.y, s.z, s.phi, VL]
        h = hov
        if kind == 'phi':
            p, dd = G['phi']; fp = (s.phi - np.deg2rad(45))*p + dd*s.dphi; fz = 0
        elif kind == 'zpd':
            p, dd = G['phi']; fp = s.phi*p + dd*s.dphi
            p, dd = G['zpd']; fz = (s.z - 1)*p - dd*s.dz
        elif kind == 'zpid':
            p, dd = G['phi']; fp = s.phi*p + dd*s.dphi; fz = zc(s.z, s.dz, 1.5); h = 0
        elif kind in ('casc', 'ff'):
            fz = zc(s.z, s.dz, 1.0); fp = pc(s.phi, s.dphi, np.clip(yc(s.y, s.dy, 1.0), -clip, clip))
            if kind == 'ff': fz += hov/np.cos(s.phi); h = 0
        else:
            dyd = float(np.clip(yp(s.y, s.dy, 2.5), -1, 1)); dzd = float(np.clip(zp(s.z, s.dz, 0), -1, 1))
            fp = pc(s.phi, s.dphi, float(np.clip(yv(s.dy, 0, dyd), -tmax, tmax)))
            fz = zv(s.dz, 0, dzd) + hov/np.cos(s.phi); h = 0
        VL = fz + fp; q.append((fz + fp, fz - fp, h))
        if len(q) > delay:
            a, b, hh = q.pop(0); mixer(d, a, b, hh, off)
    return out

def score(o, col, x0, x_init):
    t, x = o[:, 0], o[:, col]; span = abs(x0 - x_init); e = x - x0
    os_ = max(0, np.max(np.sign(x0 - x_init)*e))/span*100
    bad = np.where(np.abs(e) > 0.05*span)[0]
    ts = np.inf if len(bad) and bad[-1] == len(t)-1 else (t[bad[-1]+1] if len(bad) else t[0])
    chatter = np.mean(np.abs(np.diff(o[:, 4])))
    return os_, ts, chatter

# ---------------- 3. run, judge, plot ----------------
TESTS = [  # (key, kind, [(col, setpoint, start, axis label)], max overshoot %)
    ('phi',  'phi',  [(3, np.deg2rad(45), 0, r'$\phi$ [rad]')], 10, '03: tilt PD, 45° step'),
    ('zpd',  'zpd',  [(2, 1.0, 0, 'z [m]')], 10, '03: z PD, step to 1 m'),
    ('zpid', 'zpid', [(2, 1.5, 0, 'z [m]')], 10, '03: z PID without hover feedforward'),
    ('casc', 'casc', [(1, 1.0, 0, 'y [m]'), (2, 1.0, 0, 'z [m]')], 10, '03: basic cascade'),
    ('ff',   'ff',   [(1, 1.0, 0, 'y [m]'), (2, 1.0, 0, 'z [m]')], 10, '03: cascade + tilt feedforward'),
    ('vel',  'vel',  [(1, 2.5, 0, 'y [m]')], 10, '04: velocity cascade, wind + drag'),
    ('vel0', 'vel0', [(1, 2.5, 0, 'y [m]')], 10, '04: velocity cascade, no wind/drag'),
]
CHATTER_MAX = 0.01
allpass = True
print(f"{'test':42s} {'gains':4s} {'delay':5s} {'worst OS%':>9s} {'worst ts[s]':>11s} {'chatter':>8s}  verdict")
for key, kind, axes, os_max, label in TESTS:
    fig, axs = plt.subplots(len(axes), 1, figsize=(8, 3.2*len(axes)+0.6), sharex=True, squeeze=False)
    for gname, G, color in (('old', OLD, COLORS[3]), ('new', NEW, COLORS[0])):
        for dl in DELAYS:
            runs = [fly(kind, G, sd, dl) for sd in (SEEDS if kind != 'vel0' else [0])]
            worst = [max(score(o, c, x0, xi)[k] for o in runs for c, x0, xi, _ in axes) for k in range(3)]
            ok = worst[0] <= os_max and np.isfinite(worst[1]) and worst[2] <= CHATTER_MAX
            if gname == 'new': allpass &= ok
            print(f"{label:42s} {gname:4s} {dl:5d} {worst[0]:9.1f} {worst[1]:11.2f} {worst[2]:8.4f}  "
                  f"{'PASS' if ok else 'FAIL'}")
            for j, (c, x0, xi, yl) in enumerate(axes):
                ax = axs[j, 0]
                for o in runs[:3]:
                    ax.plot(o[:, 0], o[:, c], color=color, lw=1.4 if dl == 0 else 1.0,
                            ls=('-', '--', ':')[dl], alpha=0.85)
    for j, (c, x0, xi, yl) in enumerate(axes):
        ax = axs[j, 0]; ax.axhline(x0, color='k', lw=0.8, ls='--'); ax.set_ylabel(yl); ax.grid(alpha=0.3)
    from matplotlib.lines import Line2D
    axs[0, 0].legend(handles=[Line2D([], [], color=COLORS[3], label='old gains'),
                              Line2D([], [], color=COLORS[0], label='new gains'),
                              Line2D([], [], color='k', ls='-', label='no delay'),
                              Line2D([], [], color='k', ls='--', label='1 frame delay'),
                              Line2D([], [], color='k', ls=':', label='2 frames delay'),
                              Line2D([], [], color='k', ls='--', lw=0.8, label='setpoint')],
                     loc='lower right', ncol=2, fontsize=9)
    axs[0, 0].text(0.02, 0.95, f"{label}\nname = {NAME}, 3 wind seeds per line set",
                   transform=axs[0, 0].transAxes, va='top', fontsize=10)
    axs[-1, 0].set_xlabel('t [s]'); fig.tight_layout()
    fig.savefig(OUT / f'{key}_old_vs_new.png', dpi=150); plt.close(fig)

# calibration check against the true values hidden in the name hash
d = module.Drone(name=NAME); amp = d.V_left_amp
true_off = (d.V_right_offset - d.V_left_offset)/amp; true_hov = (d.M*d.g - 2*d.V_right_offset)/(2*amp)
cal_ok = abs(NEW['off'] - true_off) < 1e-4 and abs(NEW['hov'] - true_hov) < 1e-3
print(f"\ncalibration: V_left_offset {NEW['off']} (true {true_off:.5f}), V_hover {NEW['hov']} (true {true_hov:.4f})"
      f"  {'PASS' if cal_ok else 'FAIL'}")
print('\nALL NEW-GAIN TESTS PASS' if (allpass and cal_ok) else '\nSOME NEW-GAIN TESTS FAIL')
print(f'plots in {OUT}')
