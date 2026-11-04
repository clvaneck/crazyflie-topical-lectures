"""Motor-free gate surveying for the Day 3 notebook and optional CLI."""

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import sys
import time

import numpy as np

from notebook_radio import motor_stopped_connection, run_worker
from race_common.hoop_fit import CircleFit3D, RectangleFit3D, fit_circle_3d, fit_rectangle_3d
from race_common.motor_stop import ACTIVE_BITS, LOCKED_BIT, MOTOR_CHANNELS


MAX_GAP_S = 0.5
PERIOD_MS = 50
POSE_CHANNELS = ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z", "lighthouse.status")
STATUS_CHANNELS = ("supervisor.info", *MOTOR_CHANNELS)


def _packet_values(stream, data):
    channels = POSE_CHANNELS if stream == "pose" else STATUS_CHANNELS
    values = np.array([float(data[name]) for name in channels])
    if not np.isfinite(values).all():
        raise ValueError("non-finite survey telemetry")
    if stream == "pose" and values[3] != 2:
        raise ValueError("Lighthouse stopped supplying measurements; trace rejected")
    if stream == "motors":
        info = int(values[0])
        if (values[0] != info or not info & LOCKED_BIT or info & ACTIVE_BITS or np.any(values[1:] != 0)):
            raise ValueError("firmware lock or zero motor outputs lost; stop handling the drone")
    return values


def _write_metadata(directory, metadata):
    (directory / "recording.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")


def capture_outline(uri, directory, *, ready_for_motor_stop=False, duration_s=30, preparation_s=5):
    """Bound connection and acquisition in a child, preserving failed recordings."""
    if not ready_for_motor_stop or not isinstance(uri, str) or not uri.startswith("radio://"):
        raise ValueError("Connect first, place the drone on a stable surface and confirm readiness for motor stop.")
    if not math.isfinite(duration_s) or not 5 <= duration_s <= 120:
        raise ValueError("Trace duration must be between 5 and 120 seconds.")
    if not math.isfinite(preparation_s) or not 1 <= preparation_s <= 15:
        raise ValueError("Preparation time must be between 1 and 15 seconds.")
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    _write_metadata(directory, {"status": "starting", "source": "motor-free Lighthouse survey"})
    command = [sys.executable, "-u", "-m", "day3.survey", "--uri", uri,
               "--output", str(directory), "--duration", str(duration_s),
               "--preparation", str(preparation_s)]
    try:
        result = run_worker(command, cwd=Path(__file__).resolve().parents[1], env=dict(os.environ),
                            timeout_s=duration_s + preparation_s + 30)
        if result.returncode:
            raise RuntimeError("Survey capture failed; partial observations were retained.\n" + result.stdout)
        read_capture(directory)
    except BaseException as error:
        metadata = json.loads((directory / "recording.json").read_text())
        metadata.update(status="failed", reason=str(error) or type(error).__name__)
        _write_metadata(directory, metadata)
        raise
    return directory


def _collect(uri, directory, duration_s, preparation_s):
    # Share the typed log-block helpers used by Day 2; hardware imports stay here.
    from day2.notebook_tools import add_log_config

    events = queue.Queue(maxsize=1000)
    errors = queue.Queue()
    metadata = {"status": "recording", "source": "motor-free Lighthouse survey", "uri": uri,
                "duration_s": duration_s, "preparation_s": preparation_s,
                "position_source": "onboard stateEstimate in the configured Lighthouse frame",
                "period_ms": PERIOD_MS, "max_gap_s": MAX_GAP_S}
    raw = directory / "positions.csv"
    rows = 0
    configs = []

    def receive(stream):
        def callback(timestamp, data, _config):
            try:
                # Surface safety errors immediately, including the final packet
                # that could otherwise sit behind valid events at trace end.
                _packet_values(stream, data)
                events.put_nowait((stream, timestamp, dict(data), time.monotonic()))
            except queue.Full:
                errors.put("survey telemetry queue overflow")
            except (KeyError, TypeError, ValueError) as error:
                errors.put(str(error))
        return callback

    try:
        with raw.open("x", newline="", encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerow(("time_s", "host_monotonic_s", "firmware_timestamp_ms", "x_m", "y_m", "z_m"))
            file.flush()
            with motor_stopped_connection(uri, directory / ".cflib-cache") as (connection, stop_check):
                cf = connection.cf
                metadata["motor_stop_check"] = stop_check
                _write_metadata(directory, metadata)
                print("STOP VERIFIED: firmware locked, four motor outputs zero. You may handle the drone.", flush=True)
                cf.connection_lost.add_callback(lambda _uri, message: errors.put("connection lost: " + str(message)))
                started = time.monotonic()
                latest = {}
                previous_stamp = {}
                preparation_started = trace_started = None
                try:
                    for stream, channels in (("pose", POSE_CHANNELS), ("motors", STATUS_CHANNELS)):
                        config = add_log_config(cf, "Survey_" + stream, channels, PERIOD_MS,
                                                receive(stream), lambda _config, message: errors.put(str(message)))
                        configs.append(config)
                        config.start()
                    while True:
                        now = time.monotonic()
                        if not errors.empty():
                            raise RuntimeError(errors.get_nowait())
                        try:
                            stream, stamp, data, arrival = events.get(timeout=0.02)
                        except queue.Empty:
                            stream = None
                        if stream is not None:
                            values = _packet_values(stream, data)
                            if stream in latest and arrival - latest[stream] > MAX_GAP_S:
                                raise ValueError("survey telemetry gap exceeds 0.5 s")
                            # Log clocks wrap at 24 bits. Duplicate/reversed frames
                            # cannot make a stale position trace look continuous.
                            if stream in previous_stamp:
                                dt = (int(stamp) - previous_stamp[stream]) % (1 << 24)
                                if not 0 < dt <= MAX_GAP_S * 1000:
                                    raise ValueError("duplicate, reversed or gapped firmware timestamps")
                            previous_stamp[stream] = int(stamp)
                            latest[stream] = arrival
                        if any(now - arrival > MAX_GAP_S for arrival in latest.values()):
                            raise TimeoutError("survey telemetry stopped; trace rejected")
                        if len(latest) < 2:
                            if now - started > 5:
                                raise TimeoutError("no complete pose and motor telemetry within 5 s")
                            continue
                        if preparation_started is None:
                            preparation_started = now
                            print(f"Position the pointer on the opening. TRACE STARTS IN {preparation_s:g} SECONDS.", flush=True)
                        if trace_started is None and now - preparation_started >= preparation_s:
                            trace_started = now
                            print(f"TRACE STARTED: follow the complete outline over {duration_s:g} seconds.", flush=True)
                        if trace_started is not None:
                            if stream == "pose" and trace_started <= arrival <= trace_started + duration_s:
                                writer.writerow((arrival - trace_started, arrival, stamp, *values[:3]))
                                file.flush()
                                rows += 1
                            if now - trace_started >= duration_s:
                                if rows < 2:
                                    raise ValueError("fewer than two outline samples")
                                break
                finally:
                    for config in configs:
                        for cleanup in (config.stop, config.delete):
                            try:
                                cleanup()
                            except Exception:
                                pass  # Preserve the acquisition error on link loss.
        if not errors.empty():
            raise RuntimeError(errors.get_nowait())
        metadata["status"] = "complete"
        print("TRACE FINISHED. Place the drone down; firmware stop remains engaged.", flush=True)
    except BaseException as error:
        metadata.update(status="failed", reason=str(error) or type(error).__name__)
        raise
    finally:
        metadata["samples"] = rows
        if raw.is_file():
            metadata["positions_sha256"] = hashlib.sha256(raw.read_bytes()).hexdigest()
        _write_metadata(directory, metadata)


def read_capture(directory):
    """Accept complete, hash-checked captures; failed attempts stay diagnostic only."""
    directory = Path(directory)
    metadata = json.loads((directory / "recording.json").read_text(encoding="utf-8-sig"))
    raw = directory / "positions.csv"
    if metadata.get("status") != "complete" or metadata.get("motor_stop_check", {}).get("value") != 1:
        raise ValueError("Select a complete survey with verified firmware stop; this attempt is not fit-ready.")
    if hashlib.sha256(raw.read_bytes()).hexdigest() != metadata.get("positions_sha256"):
        raise ValueError("Survey CSV changed since capture.")
    with raw.open(newline="", encoding="utf-8-sig") as file:
        records = list(csv.DictReader(file))
    values = np.array([[float(row[name]) for name in ("time_s", "x_m", "y_m", "z_m")]
                       for row in records])
    if values.ndim != 2 or len(values) < 2 or not np.isfinite(values).all():
        raise ValueError("Survey needs finite timestamped positions.")
    times = values[:, 0]
    gaps = np.diff(np.r_[0, times, metadata["duration_s"]])
    if np.any(np.diff(times) <= 0) or np.any(gaps < 0) or gaps.max() > MAX_GAP_S + 1e-6:
        raise ValueError("Survey trace is incomplete or has non-monotonic timestamps.")
    return values[:, 1:], {**metadata, "recording_directory": str(directory.resolve())}


def example_outline(shape="circle", seed=0):
    """Labelled simulated outline for notebook execution without hardware."""
    rng = np.random.default_rng(seed)
    if shape == "circle":
        angle = np.linspace(0, 2 * np.pi, 240)
        points = np.column_stack((np.zeros_like(angle), .5 * np.cos(angle), .8 + .5 * np.sin(angle)))
    elif shape == "rectangle":
        corners = np.array([[0, -.6, .3], [0, .6, .3], [0, .6, 1.3], [0, -.6, 1.3]])
        points = np.vstack([start + np.linspace(0, 1, 60)[:, None] * (end - start)
                            for start, end in zip(corners, np.roll(corners, -1, axis=0))])
    else:
        raise ValueError("Choose circle or rectangle.")
    points += rng.normal(0, .002, points.shape)
    return points, {"source": "SIMULATED outline", "shape": shape, "seed": seed}


def spatially_subsample(points, spacing_m):
    """Reduce dwell-time weighting by retaining spatially separated samples."""
    points = np.asarray(points, dtype=float)
    if not math.isfinite(spacing_m) or spacing_m <= 0:
        raise ValueError("Sample spacing must be finite and positive.")
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError("Provide finite XYZ points.")
    retained = []
    for point in points:
        if not retained or np.linalg.norm(point - retained[-1]) >= spacing_m:
            retained.append(point)
    return np.asarray(retained).reshape(-1, 3)


def gate_record(name, fit, points, source=None):
    """The same one-gate JSON schema is consumed by the CLI and course planner."""
    def rounded(value):
        return np.round(np.asarray(value, dtype=float), 6).tolist()
    result = {"name": name, "centre": rounded(fit.centre), "normal": rounded(fit.normal)}
    survey = {"point_count": len(points), "points_m": rounded(points),
              "plane_rms_m": round(fit.plane_rms_m, 6), "plane_max_m": round(fit.plane_max_m, 6)}
    if isinstance(fit, CircleFit3D):
        result["radius_m"] = round(fit.radius_m, 6)
        survey.update(radial_rms_m=round(fit.radial_rms_m, 6), radial_max_m=round(fit.radial_max_m, 6))
    else:
        result.update(shape="rectangle", width_axis=rounded(fit.width_axis), height_axis=rounded(fit.height_axis),
                      width_m=round(fit.width_m, 6), height_m=round(fit.height_m, 6),
                      radius_m=round(min(fit.width_m, fit.height_m) / 2, 6))
        survey.update(outline_rms_m=round(fit.outline_rms_m, 6), outline_max_m=round(fit.outline_max_m, 6))
    if source is not None:
        survey["source"] = source
    result["survey"] = survey
    return result


def write_gate(path, name, fit, points, force=False, source=None):
    path = Path(path)
    record = gate_record(name, fit, points, source)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w" if force else "x", encoding="utf-8") as file:
        json.dump(record, file, indent=2, allow_nan=False)
        file.write("\n")
    return path


@dataclass(frozen=True)
class FittedSurvey:
    fit: CircleFit3D | RectangleFit3D
    points: np.ndarray
    source: dict
    snapshot: str


def fit_survey(points, shape, normal_towards, source, spacing_m=.02):
    points = spatially_subsample(points, spacing_m)
    minimum = 6 if shape == "circle" else 12
    if len(points) < minimum:
        raise ValueError(f"Need at least {minimum} retained points; trace the full outline.")
    if shape not in ("circle", "rectangle"):
        raise ValueError("Choose circle or rectangle.")
    fit = (fit_circle_3d if shape == "circle" else fit_rectangle_3d)(points, normal_towards)
    source = json.loads(json.dumps(source))
    snapshot = json.dumps(gate_record("", fit, points, source), sort_keys=True)
    return FittedSurvey(fit, points.copy(), source, snapshot)


def save_survey(survey, path, name):
    if not isinstance(survey, FittedSurvey):
        raise ValueError("Fit the selected data before saving a gate.")
    if json.dumps(gate_record("", survey.fit, survey.points, survey.source), sort_keys=True) != survey.snapshot:
        raise ValueError("Survey changed since fitting; select the data and refit before exporting.")
    return write_gate(path, name, survey.fit, survey.points, source=survey.source)


def plot_survey(survey):
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator

    fit, points = survey.fit, survey.points
    figure = plt.figure(figsize=(11, 4.5), layout="constrained")
    axis = figure.add_subplot(121, projection="3d")
    axis.scatter(*points.T, s=8, label="Recorded outline")
    if isinstance(fit, CircleFit3D):
        seed_axis = np.eye(3)[np.argmin(np.abs(fit.normal))]
        u = np.cross(fit.normal, seed_axis)
        u /= np.linalg.norm(u)
        v = np.cross(fit.normal, u)
        angle = np.linspace(0, 2 * np.pi, 150)
        outline = fit.centre + fit.radius_m * (np.cos(angle)[:, None] * u + np.sin(angle)[:, None] * v)
    else:
        outline = np.array([fit.centre + x * fit.width_m / 2 * fit.width_axis
                            + y * fit.height_m / 2 * fit.height_axis
                            for x, y in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))])
    axis.plot(*outline.T, label="Fit")
    axis.quiver(*fit.centre, *fit.normal, length=.3, color="red")
    axis.set(xlabel="X [m]", ylabel="Y [m]", zlabel="Z [m]")
    # A nearly planar trace needs room on its normal axis too: collapsing that
    # axis makes tick labels overlap and visually distorts the normal arrow.
    extent = max(.1, np.abs(np.vstack((points, outline)) - fit.centre).max(), .3) * 1.15
    for index, limits in enumerate((axis.set_xlim, axis.set_ylim, axis.set_zlim)):
        limits(fit.centre[index] - extent, fit.centre[index] + extent)
    axis.set_box_aspect((1, 1, 1))
    for coordinate in (axis.xaxis, axis.yaxis, axis.zaxis):
        coordinate.set_major_locator(MaxNLocator(3))
    axis.legend()
    residual_axis = figure.add_subplot(122)
    plane_residuals = (points - fit.centre) @ fit.normal
    residual_axis.plot(plane_residuals * 1000, ".", label="Plane residual")
    residual_axis.set(xlabel="Retained sample", ylabel="Plane residual [mm]")
    residual_axis.grid(True)
    figure.suptitle(survey.source["source"])
    return figure


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--preparation", type=float, required=True)
    args = parser.parse_args()
    _collect(args.uri, args.output, args.duration, args.preparation)
