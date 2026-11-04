"""Timestamped, sensor-level height/velocity experiments; no radio imports."""

from dataclasses import asdict, dataclass, replace
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


GRAVITY = 9.80665
RAW_COLUMNS = ("time_s", "acc_x_g", "acc_y_g", "acc_z_g", "range_mm")
LIVE_SENSOR_COLUMNS = ("gyro.x", "gyro.y", "gyro.z", "acc.x", "acc.y", "acc.z", "range.zrange")
LIVE_COLUMNS = ("time_s", *LIVE_SENSOR_COLUMNS)
VERTICAL_SENSOR_COLUMNS = ("acc.x", "acc.y", "acc.z", "range.zrange")
MAX_GAP_S = 0.30


@dataclass(frozen=True)
class Calibration:
    acceleration_at_rest_g: float
    range_std_m: float
    end_s: float = 5.0


@dataclass(frozen=True)
class Settings:
    acceleration_std_m_s2: float = 0.10
    range_std_m: float = 0.005
    initial_height_std_m: float = 0.02
    initial_velocity_std_m_s: float = 0.10
    stationary_velocity_std_m_s: float = 0.02


@dataclass
class Data:
    time_s: np.ndarray
    acceleration_m_s2: np.ndarray
    height_m: np.ndarray
    initial_height_m: float
    calibration: Calibration
    metadata: dict
    truth: np.ndarray | None = None


def timestamps(values):
    time = np.asarray(values, dtype=float)
    if time.ndim != 1 or len(time) < 2 or not np.all(np.isfinite(time)):
        raise ValueError("Need at least two finite timestamps.")
    intervals = np.diff(time)
    tolerance = np.maximum(1e-9, .5 * (np.spacing(np.abs(time[:-1])) + np.spacing(np.abs(time[1:]))))
    if np.any(intervals <= 0) or np.any(intervals > MAX_GAP_S + tolerance):
        raise ValueError("Timestamps must increase, with gaps no larger than 0.30 s.")
    return time


def synthetic(seed=0, duration_s=20.0, *, calibration=None, settings=None):
    """A level drone: five seconds at rest, rise, lower, then rest again."""
    if not np.isfinite(duration_s) or duration_s < 18:
        raise ValueError("The complete synthetic experiment needs at least 18 seconds.")
    rng = np.random.default_rng(seed)
    acceleration_at_rest_g = calibration.acceleration_at_rest_g if calibration else 1 + .04 / GRAVITY
    range_std_m = settings.range_std_m if settings else (calibration.range_std_m if calibration else .005)
    acceleration_std = settings.acceleration_std_m_s2 if settings else .06
    time = np.arange(0, duration_s + 0.001, 0.05)
    height = np.full(len(time), 0.30)
    velocity = np.zeros(len(time))
    for start, direction in ((5, 1), (10, -1)):
        mask = (time >= start) & (time < start + 5)
        u = (time[mask] - start) / 5
        height[mask] = (0.30 if direction == 1 else 0.45) + direction * 0.15 * (10*u**3 - 15*u**4 + 6*u**5)
        velocity[mask] = direction * 0.15 / 5 * (30*u**2 - 60*u**3 + 30*u**4)
    # Current input represents the interval ending at the sample timestamp.
    acceleration = np.r_[0, np.diff(velocity) / np.diff(time)]
    raw = np.column_stack((time, rng.normal(0, .001, len(time)),
                           rng.normal(0, .001, len(time)),
                           acceleration_at_rest_g + (acceleration + rng.normal(0, acceleration_std, len(time))) / GRAVITY,
                           (height + rng.normal(0, range_std_m, len(time))) * 1000))
    metadata = {"source": "simulation", "seed": seed, "status": "complete",
                "sensor_assumptions": {"acceleration_at_rest_g": acceleration_at_rest_g,
                                       "acceleration_std_m_s2": acceleration_std,
                                       "range_std_m": range_std_m},
                "known_rest_intervals_s": [[0, 5], [15, duration_s]],
                "timing": "Simulated measurement times; acceleration approximates the preceding interval."}
    return raw, metadata, np.column_stack((height, velocity))


def check_rest(raw, end_s=5.0, *, range_noise_std_m=0.):
    """Check the initial reference conditions, including on held-out recordings."""
    time = timestamps(raw[:, 0])
    if time[-1] < end_s or time[0] > end_s - 4.5:
        raise ValueError("Record five initial stationary seconds for calibration.")
    rest = raw[time < end_s]
    force, ranges = rest[:, 1:4], rest[:, 4] / 1000
    if (len(rest) < 30 or not np.all(np.isfinite(rest))
            or np.any(ranges <= 0) or np.any(ranges > 4.0)):
        raise ValueError("Calibration needs finite acceleration and positive floor ranges no greater than 4 m.")
    if (abs(np.linalg.norm(force.mean(axis=0)) - 1) > .08
            or np.linalg.norm(force[:, :2].mean(axis=0)) > .10
            or force[:, 2].mean() < .9 or force[:, 2].std() > .03
            or ranges.std() > max(.01, 2 * range_noise_std_m)):
        raise ValueError("Initial data are inconsistent with a stationary, level drone over a fixed floor.")
    return force, ranges


def calibrate(raw, end_s=5.0):
    """Estimate the stationary Z specific-force offset and range precision."""
    force, ranges = check_rest(raw, end_s)
    # A constant, quantized range record is not evidence for zero measurement variance.
    return Calibration(float(force[:, 2].mean()), max(float(ranges.std(ddof=1)), .001), end_s)


def prepare(raw, metadata, *, calibration=None, truth=None):
    raw = np.asarray(raw, dtype=float)
    if raw.ndim != 2 or raw.shape[1] != len(RAW_COLUMNS):
        raise ValueError("Raw columns must be time_s, acc_x_g, acc_y_g, acc_z_g, range_mm.")
    time = timestamps(raw[:, 0])
    if not np.all(np.isfinite(raw[:, :4])):
        raise ValueError("Acceleration must be finite at every timestamp.")
    # Without attitude estimation, the experiment deliberately assumes level motion.
    if np.any(np.linalg.norm(raw[:, 1:3], axis=1) > .15) or np.any(raw[:, 3] < .7):
        raise ValueError("Motion violates the level, gentle-motion model; use a checked recording.")
    if calibration is None:
        calibration = calibrate(raw)
    else:
        # Known simulated rest can include the deliberately enlarged V from E3.
        # Recorded data retain the original stationary-geometry check.
        range_noise = (metadata.get("sensor_assumptions", {}).get("range_std_m", 0.)
                       if truth is not None and metadata.get("source") == "simulation" else 0.)
        check_rest(raw, calibration.end_s, range_noise_std_m=range_noise)
    if (not np.isfinite(calibration.acceleration_at_rest_g)
            or not np.isfinite(calibration.range_std_m) or calibration.range_std_m <= 0):
        raise ValueError("Calibration values must be finite, with positive range precision.")
    # Calibration is available before filtering begins; never use later samples to fit it.
    mask = time >= calibration.end_s
    if mask.sum() < 2:
        raise ValueError("Need motion samples after calibration.")
    ranges = raw[:, 4] / 1000
    ranges[(ranges <= 0) | (ranges > 4) | ~np.isfinite(ranges)] = np.nan
    initial = ranges[mask][0]
    if not np.isfinite(initial):
        raise ValueError("The first filtering sample needs a valid range for initialization.")
    source = dict(metadata, input_sha256=hashlib.sha256(raw.tobytes()).hexdigest())
    return Data(time[mask], (raw[mask, 3] - calibration.acceleration_at_rest_g) * GRAVITY,
                ranges[mask], float(initial), calibration, source,
                None if truth is None else np.asarray(truth)[mask])


def model(dt):
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError("Use a finite positive sample interval.")
    return np.array([[1., dt], [0., 1.]]), np.array([.5 * dt**2, dt])


def predict(x, C, acceleration, dt, acceleration_std):
    F, B = model(dt)
    Q = np.outer(B, B) * acceleration_std**2
    return F @ x + B * acceleration, F @ C @ F.T + Q


def update(x, C, height, variance):
    H = np.array([1., 0.])
    residual = height - H @ x
    R = float(H @ C @ H + variance)
    K = C @ H / R
    corrected = x + K * residual
    # Joseph form keeps the supplied covariance bookkeeping numerically robust.
    A = np.eye(2) - np.outer(K, H)
    return corrected, A @ C @ A.T + np.outer(K, K) * variance


def check_steps(predict_step, update_step):
    """Check the team's four expressions before processing a recording."""
    for dt, variance in ((.05, .000025), (.2, .04)):
        x, C = np.array([.3, .2]), np.array([[.04, .01], [.01, .09]])
        expected = predict(x, C, .1, dt, .08)
        actual = predict_step(x.copy(), C.copy(), .1, dt, .08)
        for result, reference in zip(actual, expected):
            np.testing.assert_allclose(result, reference, err_msg="Check the state/covariance prediction.")
        expected = update(*expected, .35, variance)
        actual = update_step(*actual, .35, variance)
        for result, reference in zip(actual, expected):
            np.testing.assert_allclose(result, reference, err_msg="Check gain, state correction and covariance.")
    print("Prediction and update checks passed.")


def accept_range(residual, R, threshold):
    """Accept a scalar residual within the chosen chi-square threshold."""
    return threshold is None or residual**2 / R <= threshold


def zero_velocity_update(x, C, V):
    """Fuse a known-rest velocity measurement with finite variance V."""
    H = np.array([0., 1.])
    residual = -H @ x
    R = H @ C @ H + V
    K = C @ H / R
    A = np.eye(2) - np.outer(K, H)
    return x + K * residual, A @ C @ A.T + np.outer(K, K) * V


def run(data, settings=None, *, predict_step=predict, update_step=update,
        gate_step=accept_range, stationary_step=zero_velocity_update,
        residual_gate=None, alpha=.01, range_gap=None, stationary_intervals=()):
    if data is None:
        raise ValueError("Select data successfully before running an experiment.")
    settings = settings or Settings(range_std_m=data.calibration.range_std_m)
    if any(not np.isfinite(value) or value <= 0 for value in asdict(settings).values()):
        raise ValueError("All configured standard deviations must be finite and positive.")
    if not np.isfinite(alpha) or not 0 < alpha < 1:
        raise ValueError("False-rejection probability must be between zero and one.")
    threshold = residual_gate(1, alpha) if residual_gate is not None else None
    if threshold is not None and (not np.isfinite(threshold) or threshold <= 0):
        raise ValueError("Residual threshold must be finite and positive, or None.")
    time = timestamps(data.time_s)
    if range_gap is not None:
        start, stop = range_gap
        valid = np.isfinite(data.height_m)
        if (not np.isfinite([start, stop]).all() or start >= stop
                or time[-1] < stop + 2 or not np.any(valid & (time < start))
                or not np.any(valid & (time >= start) & (time <= stop))
                or not np.any(valid & (time > stop))):
            raise ValueError("Range-gap comparison needs observations before, during and after the gap, plus two recovery seconds.")
    x = np.array([data.initial_height_m, 0.])
    C = np.diag([settings.initial_height_std_m**2, settings.initial_velocity_std_m_s**2])
    states, covariances, records = [], [], []
    for index, stamp in enumerate(time):
        residual, R, chi2 = np.nan, np.nan, np.nan
        K, accepted, constrained = np.zeros(2), False, False
        if index:
            x, C = predict_step(x, C, data.acceleration_m_s2[index], stamp - time[index-1],
                                settings.acceleration_std_m_s2)
            available = np.isfinite(data.height_m[index])
            in_gap = range_gap is not None and range_gap[0] <= stamp <= range_gap[1]
            if available and not in_gap:
                residual = float(data.height_m[index] - x[0])
                R = float(C[0, 0] + settings.range_std_m**2)
                K = C[:, 0] / R  # Prospective gain, retained even when the gate rejects the sample.
                chi2 = residual**2 / R
                accepted = gate_step(residual, R, threshold)
                if accepted:
                    x, C = update_step(x, C, data.height_m[index], settings.range_std_m**2)
            if any(start <= stamp <= stop for start, stop in stationary_intervals):
                V = settings.stationary_velocity_std_m_s**2
                x, C = stationary_step(x, C, V)
                constrained = True
        if (np.shape(x) != (2,) or np.shape(C) != (2, 2)
                or not np.all(np.isfinite(x)) or not np.all(np.isfinite(C))):
            raise ValueError("Prediction/update must return a finite two-state vector and 2x2 covariance.")
        states.append(x.copy())
        covariances.append(C.copy())
        records.append((residual, R, chi2, accepted, *K, constrained))
    diagnostics = np.asarray(records)
    return {"time_s": time, "state": np.asarray(states), "covariance": np.asarray(covariances),
            "predicted_residual": diagnostics[:, 0], "predicted_residual_variance": diagnostics[:, 1],
            "chi2": diagnostics[:, 2], "accepted": diagnostics[:, 3].astype(bool),
            "gain": diagnostics[:, 4:6], "stationary_update": diagnostics[:, 6].astype(bool),
            "settings": asdict(settings), "range_gap_s": range_gap,
            "stationary_intervals_s": list(stationary_intervals), "gate_threshold": threshold,
            "false_rejection_probability": alpha,
            "input_sha256": data_fingerprint(data)}


def data_fingerprint(data):
    return hashlib.sha256(np.column_stack((data.time_s, data.acceleration_m_s2, data.height_m)).tobytes()).hexdigest()


def summary(data, result):
    mask = result['time_s'] >= result['time_s'][0] + 2
    pull = result['predicted_residual'] / np.sqrt(result['predicted_residual_variance'])
    valid = mask & np.isfinite(pull)
    report = {"pre_gate_pull_mean": float(pull[valid].mean()) if valid.any() else None,
              "pre_gate_pull_std": float(pull[valid].std()) if valid.any() else None,
              "rejected": int(np.sum(np.isfinite(pull) & ~result['accepted']))}
    if data.truth is not None:
        error = result['state'][mask] - data.truth[mask]
        sigma = np.sqrt(np.diagonal(result['covariance'][mask], axis1=1, axis2=2))
        report['rmse_height_velocity'] = np.sqrt(np.mean(error**2, axis=0)).tolist()
        report['marginal_95_percent_coverage'] = np.mean(np.abs(error) <= 1.96 * sigma, axis=0).tolist()
    return report


def with_outlier(data, time_s=12., offset_m=.15):
    indices = np.flatnonzero((data.time_s >= time_s) & np.isfinite(data.height_m))
    if not len(indices) or indices[0] == 0:
        raise ValueError("Need a later valid range observation at or after the requested fault time.")
    heights = data.height_m.copy()
    heights[indices[0]] += offset_m
    return replace(data, height_m=heights, metadata=dict(data.metadata,
                   analysis_change={"outlier_time_s": float(data.time_s[indices[0]]), "offset_m": offset_m}))


def load_raw(directory):
    directory = Path(directory)
    metadata = json.loads((directory / "recording.json").read_text(encoding="utf-8-sig"))
    recording_format = metadata.get("format")
    if recording_format == "vertical-sensors-v1":
        columns = RAW_COLUMNS
        value_columns = RAW_COLUMNS
    elif recording_format == "vertical-sensors-v2":
        columns = LIVE_COLUMNS
        value_columns = ("time_s", *VERTICAL_SENSOR_COLUMNS)
        if (metadata.get("channels") != list(LIVE_SENSOR_COLUMNS)
                or metadata.get("vertical_analysis_channels") != list(VERTICAL_SENSOR_COLUMNS)):
            raise ValueError("Live vertical recording has unexpected sensor or analysis channels.")
    else:
        raise ValueError("Choose a complete sensor-level vertical recording with declared columns.")
    if metadata.get("status") != "complete" or metadata.get("columns") != list(columns):
        raise ValueError("Choose a complete sensor-level vertical recording with declared columns.")
    content = (directory / "raw.csv").read_bytes()
    if hashlib.sha256(content).hexdigest() != metadata.get("raw_sha256"):
        raise ValueError("Raw recording hash does not match metadata.")
    with (directory / "raw.csv").open(encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames != list(columns):
            raise ValueError("Unexpected vertical recording columns.")
        rows = [[float(row[name]) if row[name] else np.nan for name in value_columns]
                for row in reader]
    return np.asarray(rows), metadata


def select(directory=None, *, seed=0, calibration=None, settings=None):
    if directory is None:
        raw, metadata, truth = synthetic(seed, calibration=calibration, settings=settings)
    else:
        raw, metadata = load_raw(directory)
        truth = None
    data = prepare(raw, metadata, calibration=calibration, truth=truth)
    if data.time_s[-1] < 18:
        raise ValueError("Use a complete 20-second recording for E3-E7.")
    return data


def save_comparison(data, runs, output, *, notes="", plot_pull_distribution=False):
    """Keep timestamped diagnostics, frozen settings, provenance and compact plots."""
    import matplotlib.pyplot as plt

    if not runs or any(result['input_sha256'] != data_fingerprint(data) for result in runs.values()):
        raise ValueError("Run results must match the selected timestamped data.")
    pull_distribution_labels = ("baseline", "V-small", "V-large", "Q-candidate")
    if plot_pull_distribution and not all(label in runs for label in pull_distribution_labels):
        raise ValueError("Pull-distribution plots need baseline, V-small, V-large and Q-candidate runs.")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    np.savetxt(output / 'inputs.csv', np.column_stack((data.time_s, data.acceleration_m_s2, data.height_m)),
               delimiter=',', header='time_s,vertical_acceleration_m_s2,range_height_m', comments='')
    if plot_pull_distribution:
        figure, axes = plt.subplots(4, 1, figsize=(10, 10))
        axes[1].sharex(axes[0])
        axes[2].sharex(axes[0])
    else:
        figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(data.time_s, data.height_m, '.', color='.65', label="ToF measurement")
    integrated_height, integrated_velocity = [data.initial_height_m], [0.]
    for index, dt in enumerate(np.diff(data.time_s), start=1):
        integrated_height.append(integrated_height[-1] + dt * integrated_velocity[-1]
                                 + .5 * dt**2 * data.acceleration_m_s2[index])
        integrated_velocity.append(integrated_velocity[-1] + dt * data.acceleration_m_s2[index])
    axes[0].plot(data.time_s, integrated_height, '--', color='C0', label="acceleration integration")
    axes[1].plot(data.time_s, integrated_velocity, '--', color='C0', label="acceleration integration")
    axes[1].plot(data.time_s[1:], np.diff(data.height_m)/np.diff(data.time_s),
                 color='.75', label="range differences")
    saved_settings = {}
    # Keep each filter's colour identical across state, uncertainty and residual panels.
    for color_index, (label, result) in enumerate(runs.items(), start=1):
        color = f'C{color_index % 10}'
        if Path(label).name != label or label in ('.', '..') or '\\' in label:
            raise ValueError("Run labels must be simple filenames.")
        with (output / f"{label}.csv").open('x', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(["time_s", "height_m", "velocity_m_s", "C_zz", "C_zv", "C_vv",
                             "predicted_residual_m", "predicted_residual_variance_m2", "chi2",
                             "accepted", "K_height", "K_velocity", "stationary_update"])
            for i, stamp in enumerate(result['time_s']):
                C = result['covariance'][i]
                writer.writerow([stamp, *result['state'][i], C[0, 0], C[0, 1], C[1, 1],
                                 result['predicted_residual'][i], result['predicted_residual_variance'][i],
                                 result['chi2'][i], int(result['accepted'][i]), *result['gain'][i],
                                 int(result['stationary_update'][i])])
        for component in (0, 1):
            mean = result['state'][:, component]
            sigma = np.sqrt(result['covariance'][:, component, component])
            axes[component].plot(result['time_s'], mean, color=color, label=label)
            axes[component].fill_between(result['time_s'], mean-1.96*sigma, mean+1.96*sigma,
                                        color=color, alpha=.12)
        pull = result['predicted_residual'] / np.sqrt(result['predicted_residual_variance'])
        axes[2].plot(result['time_s'], pull, '.', color=color, label=label)
        if plot_pull_distribution and label in pull_distribution_labels:
            finite_pull = pull[np.isfinite(pull)]
            pull_rms = np.sqrt(np.mean(finite_pull**2))
            axes[3].hist(finite_pull, bins=np.linspace(-5, 5, 81), density=True,
                         histtype='step', linewidth=1.6, color=color,
                         label=f"{label} (RMS={pull_rms:.2f})")
        saved_settings[label] = {"filter": result['settings'], "range_gap_s": result['range_gap_s'],
                                 "stationary_intervals_s": result['stationary_intervals_s'],
                                 "gate_threshold": result['gate_threshold'],
                                 "false_rejection_probability": result['false_rejection_probability'],
                                 "summary": summary(data, result)}
    if data.truth is not None:
        for component in (0, 1):
            axes[component].plot(data.time_s, data.truth[:, component], 'k:', label="simulation truth")
    labels = ["Height [m]", "Vertical velocity [m/s]", "Predicted residual / sigma"]
    if plot_pull_distribution:
        labels.append("Density")
    for axis, ylabel in zip(axes, labels):
        axis.set_ylabel(ylabel)
        axis.grid(True)
        axis.legend(fontsize='small', ncol=2)
    if plot_pull_distribution:
        normal_x = np.linspace(-5, 5, 401)
        normal_density = np.exp(-0.5 * normal_x**2) / np.sqrt(2 * np.pi)
        axes[3].plot(normal_x, normal_density, 'k--', linewidth=1.4, label="Normal N(0, 1)")
        axes[3].set_xlim(-5, 5)
        axes[3].set_xlabel("Normalized predicted residual, r / sqrt(R)")
        axes[2].set_xlabel("Time since first firmware log sample [s] (simulated time for fallback)")
    else:
        axes[-1].set_xlabel("Time since first firmware log sample [s] (simulated time for fallback)")
    figure.tight_layout()
    figure.savefig(output / "comparison.png", dpi=120)
    plt.close(figure)
    provenance = {"settings": saved_settings, "calibration": asdict(data.calibration),
                  "source": data.metadata, "notes": notes,
                  "processed_input_sha256": data_fingerprint(data),
                  "limitations": "Bands are model uncertainty, not independent accuracy evidence. Level drone and fixed floor assumed."}
    (output / "settings.json").write_text(json.dumps(provenance, indent=2, allow_nan=False) + '\n')
    return output / "comparison.png"
