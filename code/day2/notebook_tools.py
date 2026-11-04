"""Notebook capture and experiment helpers."""

from contextlib import contextmanager
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from typing import Sequence

import numpy as np

from day1.crazyflie import drone_uri
from notebook_radio import motor_stopped_connection as stopped_connection
from notebook_radio import run_worker as _run_capture


@contextmanager
def _capture_receipt(destination, details):
    """Only mark a capture complete after logging and cleanup have succeeded."""
    receipt = Path(str(destination) + ".capture.json")
    if destination.exists() or receipt.exists():
        raise FileExistsError(f"Choose a new capture filename: {destination}")
    details = dict(details, status="recording", rows=0)
    with receipt.open("x", encoding="utf-8") as stream:
        json.dump(details, stream, indent=2)
    try:
        yield details
    except BaseException as error:
        details.update(status="interrupted" if isinstance(error, KeyboardInterrupt) else "failed",
                       reason=str(error))
        raise
    finally:
        if destination.is_file():
            details["sha256"] = hashlib.sha256(destination.read_bytes()).hexdigest()
        receipt.write_text(json.dumps(details, indent=2) + "\n", encoding="utf-8")


def completed_capture(path):
    """Auto-select only a complete, unchanged recording made by capture()."""
    path = Path(path)
    receipt = Path(str(path) + ".capture.json")
    if not path.is_file() or not receipt.is_file():
        return None
    details = json.loads(receipt.read_text(encoding="utf-8"))
    if details.get("status") != "complete" or details.get("rows", 0) < 2:
        return None
    if details.get("sha256") != hashlib.sha256(path.read_bytes()).hexdigest():
        raise ValueError(f"Capture changed after completion: {path}")
    return path


def read_imu(path):
    """Read a checked cfclient or Python IMU CSV, retaining body units."""
    columns = tuple(f"{sensor}.{axis}" for sensor in ("acc", "gyro") for axis in "xyz")
    with Path(path).open(encoding="utf-8-sig", newline="") as stream:
        rows = csv.DictReader(stream)
        values = np.array([[float(row[name]) for name in columns] for row in rows])
    if values.ndim != 2 or values.shape[0] < 2 or not np.all(np.isfinite(values)):
        raise ValueError("Need at least two finite six-channel IMU samples.")
    return values


def simulated_imu_pair(directory):
    """Create two independent stationary examples; never represent them as drone data."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    paths = []
    columns = [f"{sensor}.{axis}" for sensor in ("acc", "gyro") for axis in "xyz"]
    for name, seed in (("calibration", 42), ("validation", 43)):
        path = directory / f"{name}.csv"
        if path.exists():
            if completed_capture(path) is None:
                raise ValueError(f"Incomplete simulated example: {path}; choose a fresh output directory.")
            details = json.loads(Path(str(path) + ".capture.json").read_text())
            if details.get("source") != "simulation" or details.get("seed") != seed:
                raise ValueError(f"Expected a labelled simulated example: {path}")
        else:
            rng = np.random.default_rng(seed)
            # Fixed sensor offsets; independent noise in the held-out recording.
            values = rng.normal([.01, -.02, 1., .3, -.2, .5],
                                [.002, .002, .002, .08, .08, .08], size=(3000, 6))
            with _capture_receipt(path, {"source": "simulation", "seed": seed}) as receipt:
                with path.open("x", newline="", encoding="utf-8") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(columns)
                    writer.writerows(values)
                receipt.update(status="complete", rows=len(values))
        paths.append(path)
    return tuple(paths)


def plot_imu_calibration(values, names, standardized_calibration, standardized_validation):
    """Plot already calculated quantities; keep the calibration math in the notebook."""
    import matplotlib.pyplot as plt

    for view in ("samples", "standardized comparison"):
        figure, axes = plt.subplots(3, 2, figsize=(12, 8))
        for index, (axis, name) in enumerate(zip(axes.flat, names)):
            unit = "g" if name.startswith("acc.") else "deg/s"
            if view == "samples":
                axis.plot(values[:, index], linewidth=0.7)
                axis.set(xlabel="sample index", ylabel=unit, title=name)
            else:
                calibration = standardized_calibration[:, index]
                if np.all(np.isnan(calibration)):
                    axis.text(.5, .5, "Zero calibration RMS; normalization undefined",
                              transform=axis.transAxes, ha="center", va="center", wrap=True)
                else:
                    validation = (None if standardized_validation is None
                                  else standardized_validation[:, index])
                    combined = calibration if validation is None else np.concatenate((calibration, validation))
                    bins = np.histogram_bin_edges(combined, bins=35)
                    axis.hist(calibration, bins=bins, density=True, histtype="step", linewidth=1.5,
                              label="Calibration: self-standardized")
                    if validation is not None:
                        axis.hist(validation, bins=bins, density=True, histtype="step", linewidth=1.5,
                                  label="Held-out: frozen calibration")
                    reference = np.linspace(min(-4, combined.min()), max(4, combined.max()), 300)
                    axis.plot(reference, np.exp(-.5 * reference**2) / np.sqrt(2 * np.pi),
                              "k--", label="N(0, 1) reference")
                    axis.legend(fontsize="small")
                axis.set(xlabel="(sample - calibration mean) / calibration RMS", ylabel="density", title=name)
            axis.grid(True, alpha=.3)
        figure.suptitle(f"Stationary IMU calibration: {view}")
        figure.tight_layout()
        plt.show()
        plt.close(figure)


def display_table(rows):
    """Render a small comparison without adding a dataframe dependency."""
    from html import escape
    from IPython.display import HTML, display

    if not rows:
        print("No measurements available for this comparison.")
        return
    columns = list(rows[0])
    header = "".join(f"<th>{escape(str(name))}</th>" for name in columns)
    body = "".join("<tr>" + "".join(f"<td>{escape(str(row.get(name)))}</td>"
                                    for name in columns) + "</tr>" for row in rows)
    display(HTML(f"<table><thead><tr>{header}</tr></thead><tbody>{body}</tbody></table>"))


@contextmanager
def motor_stopped_connection(uri: str, cache_directory: Path):
    """Connect to a Crazyflie and verify its firmware motor stop before yielding."""
    with stopped_connection(uri, cache_directory) as (connection, stop_check):
        yield connection.cf, stop_check


def add_log_config(cf, name: str, channels: Sequence[str], period_ms: int,
                   data_callback, error_callback):
    """Validate TOC channels and register a typed log block with callbacks."""
    from cflib.crazyflie.log import LogConfig

    elements = log_channel_elements(cf, channels)
    if not isinstance(period_ms, int) or isinstance(period_ms, bool) or period_ms <= 0:
        raise ValueError("Logging period must be a positive integer number of milliseconds.")
    config = LogConfig(name=name, period_in_ms=period_ms)
    for channel, element in elements.items():
        config.add_variable(channel, element.ctype)
    config.data_received_cb.add_callback(data_callback)
    config.error_cb.add_callback(error_callback)
    cf.log.add_config(config)
    return config


def log_channel_elements(cf, channels: Sequence[str]):
    """Return TOC elements for unique channel names or report all missing names."""
    if not channels or isinstance(channels, str) or len(set(channels)) != len(channels):
        raise ValueError("Provide a non-empty list of unique log channel names.")
    elements = {
        channel: cf.log.toc.get_element_by_complete_name(channel)
        for channel in channels
    }
    missing = [channel for channel, element in elements.items() if element is None]
    if missing:
        raise ValueError("Firmware lacks required log channels: " + ", ".join(missing))
    return elements


def capture(uri, output, channels, *, duration_s=60, period_ms=20, overwrite=False):
    """Save selected log channels with elapsed seconds from the first firmware sample."""
    if not isinstance(uri, str) or not uri.startswith("radio://"):
        raise ValueError("Provide the assigned Crazyradio URI.")
    if not channels or isinstance(channels, str) or len(set(channels)) != len(channels):
        raise ValueError("Provide a non-empty list of unique log channel names.")
    if not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("Capture duration must be finite and positive.")
    if not isinstance(period_ms, int) or isinstance(period_ms, bool) or period_ms <= 0:
        raise ValueError("Logging period must be a positive integer number of milliseconds.")
    if not isinstance(overwrite, bool):
        raise ValueError("overwrite must be True or False.")

    destination = Path(output)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if overwrite:
        receipt_path = Path(str(destination) + ".capture.json")
        existing_paths = (destination, receipt_path)
        if any(path.is_dir() for path in existing_paths):
            raise IsADirectoryError("Capture output and receipt paths must be files.")
        for path in existing_paths:
            if path.exists() or path.is_symlink():
                path.unlink()
    changed = threading.Event()
    lock = threading.Lock()
    timestamp_modulus = 1 << 24
    state = {
        "first_sample": None, "last_sample": None, "rows": 0, "error": None,
        "previous_timestamp_ms": None, "elapsed_ms": 0,
    }

    details = dict(uri=uri, channels=list(channels), duration_s=duration_s,
                   period_ms=period_ms, timestamp_unit="seconds",
                   timestamp_origin="first firmware sample")
    with _capture_receipt(destination, details) as receipt, \
            destination.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("time_s", *channels))
        stream.flush()

        def receive(timestamp, data, _config):
            with lock:
                if state["error"] is not None:
                    return
                try:
                    values = [float(data[channel]) for channel in channels]
                    if not all(math.isfinite(value) for value in values):
                        raise ValueError("log packet contains a non-finite value")
                    timestamp_ms = int(timestamp)
                    if not 0 <= timestamp_ms < timestamp_modulus:
                        raise ValueError("firmware timestamp is outside its 24-bit range")
                    previous_timestamp_ms = state["previous_timestamp_ms"]
                    if previous_timestamp_ms is not None:
                        delta_ms = (timestamp_ms - previous_timestamp_ms) % timestamp_modulus
                        if delta_ms == 0 or delta_ms >= timestamp_modulus // 2:
                            raise ValueError("firmware timestamps are duplicated or out of order")
                        state["elapsed_ms"] += delta_ms
                    state["previous_timestamp_ms"] = timestamp_ms
                    now = time.monotonic()
                    if state["first_sample"] is None:
                        state["first_sample"] = now
                    writer.writerow((state["elapsed_ms"] / 1000, *values))
                    stream.flush()
                    state["last_sample"] = now
                    state["rows"] += 1
                    receipt["rows"] = state["rows"]
                except (KeyError, TypeError, ValueError, OverflowError) as error:
                    state["error"] = f"Invalid telemetry: {error}"
                changed.set()

        def log_error(_config, message):
            with lock:
                state["error"] = f"Logging failed: {message}"
                changed.set()

        cache_directory = destination.parent / ".cflib-cache"
        with motor_stopped_connection(uri, cache_directory) as (cf, _stop_check):
            receipt["motor_stop_check"] = _stop_check
            print("STOP VERIFIED: firmware lock set and all four motor outputs zero.")
            cf.connection_lost.add_callback(
                lambda _uri, message: log_error(None, f"connection lost: {message}")
            )
            config = add_log_config(
                cf, "Day2_capture", channels, period_ms, receive, log_error
            )
            started = False
            try:
                config.start()
                started = True
                wait_started = time.monotonic()
                while True:
                    now = time.monotonic()
                    with lock:
                        error = state["error"]
                        first_sample = state["first_sample"]
                        last_sample = state["last_sample"]
                        row_count = state["rows"]
                    if error is not None:
                        raise RuntimeError(error)
                    if first_sample is None:
                        if now - wait_started > 10:
                            raise TimeoutError("No telemetry received within 10 seconds.")
                    else:
                        if last_sample is not None and now - last_sample > 1:
                            raise TimeoutError(
                                "Telemetry stopped; partial CSV was retained."
                            )
                        if now - first_sample >= duration_s:
                            if row_count < 2:
                                raise RuntimeError(
                                    "Fewer than two samples were recorded."
                                )
                            break
                    changed.wait(0.05)
                    changed.clear()
            finally:
                try:
                    if started:
                        config.stop()
                finally:
                    config.delete()
                    stream.flush()

        receipt["status"] = "complete"

    return destination, state["rows"]


def capture_vertical(output, uri, *, ready_for_motor_stop=False, duration_s=20,
                     overwrite=False):
    """Acquire sensor-level height data; optionally replace this capture directory."""
    if not ready_for_motor_stop or not isinstance(uri, str) or not uri.strip():
        raise ValueError("Set the assigned URI and confirm a stable surface, hands clear and other radio clients closed.")
    if not math.isfinite(duration_s) or not 18 <= duration_s <= 30:
        raise ValueError("Vertical capture duration must be between 18 and 30 seconds.")
    if not isinstance(overwrite, bool):
        raise ValueError("overwrite must be True or False.")
    output = Path(output).expanduser().absolute()
    if output.is_symlink():
        raise ValueError(f"Refusing to overwrite a symbolic link: {output}")
    output = output.resolve()
    protected_paths = (Path(output.anchor), Path.home().resolve(), Path.cwd().resolve())
    if any(path == output or output in path.parents for path in protected_paths):
        raise ValueError(f"Refusing to overwrite a root, home, or workspace directory: {output}")
    if output.exists():
        if not output.is_dir():
            raise FileExistsError(f"Capture output exists and is not a directory: {output}")
        if not overwrite:
            raise FileExistsError(f"Capture directory already exists; set overwrite=True to replace it: {output}")
        shutil.rmtree(output)
    # This notebook already owns the timeout; avoid a nested supervisor/worker.
    command = [sys.executable, '-u', '-m', 'day2.vertical_live', '--uri', uri.strip(),
               '--output', str(output), '--duration', str(duration_s),
               '--prepare-for-handling', '--capture-worker']
    print("Keep the drone on the surface until STOP VERIFIED, then follow the five-second rest check and motion prompts.", flush=True)
    try:
        result = _run_capture(command, cwd=Path(__file__).resolve().parents[1],
                              env=dict(os.environ, MPLBACKEND='Agg'), timeout_s=duration_s + 30)
    except subprocess.TimeoutExpired as error:
        raise TimeoutError("Capture exceeded its bound; select a checked sensor recording or simulation.") from error
    if result.returncode:
        raise RuntimeError(f"Capture failed; inspect {output}, then explicitly select a fallback.\n{result.stdout}")
    from .vertical import select
    select(output)  # Validate before returning a usable selection to the notebook.
    return output


def next_attempt(parent):
    """Choose a new attempt path; the experiment writer creates that directory.

    Notebook cells call this on every execution, so changing one assumption does
    not require a new capture or a kernel restart. Writers still refuse to
    overwrite an existing directory, including partially failed experiments.
    """
    parent = Path(parent)
    parent.mkdir(parents=True, exist_ok=True)
    number = 1
    while (parent / f"attempt-{number:03d}").exists():
        number += 1
    result = parent / f"attempt-{number:03d}"
    print("Saving this attempt to:", result)
    return result


def calc_3sigmarms(x, max_iter=50, tol=1e-6):
    """
    '3-sigma RMS': find (mu, sigma) such that the RMS of the data about mu,
    computed inside the window |x - mu| <= 3*sigma, equals sigma.

    Returns (mu, sigma), where mu is the centre of the window (the mean of
    the points inside it, once converged).
    """
    x = np.asarray(x, dtype=float)
    x = x[np.isfinite(x)]
    n = len(x)
    if n < 3:
        raise ValueError("need at least 3 values")

    mu = np.median(x)                      # robust starting point
    sigma = np.sqrt(np.mean((x - mu) ** 2))

    for _ in range(max_iter):
        # "symmetrize": only the distance to the centre matters
        d = np.sort(np.abs(x - mu))
        k = np.arange(1, n + 1)
        rms = np.sqrt(np.cumsum(d ** 2) / k)   # RMS of the k closest points
        g = 3 * rms - d                        # >0: window edge still "inside" 3*rms

        # as in the ROOT code, require more than half of the data in the window
        kmin = n // 2 + 1
        below = np.nonzero(g[kmin - 1:] <= 0)[0]

        if below.size == 0:
            # never self-consistent: fall back to the plain RMS
            new_sigma = rms[-1]
        else:
            kk = kmin + below[0]               # first k (1-based) with 3*rms <= edge
            prev = rms[kk - 2]                 # rms with kk-1 points in the window
            if 3 * prev < d[kk - 1]:
                new_sigma = prev               # exact solution: 3*rms_(k-1) lies in [d_(k-1), d_k)
            else:
                new_sigma = d[kk - 1] / 3.0    # rms jumps across the edge: take the crossing point

        # recentre on the mean of the points inside the window
        inside = np.abs(x - mu) <= 3 * new_sigma
        new_mu = x[inside].mean()

        converged = abs(new_mu - mu) < tol * new_sigma and abs(new_sigma - sigma) < tol * new_sigma
        mu, sigma = new_mu, new_sigma
        if converged:
            break

    return mu, sigma
