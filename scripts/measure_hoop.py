#!/usr/bin/env python3
"""Survey a gate outline in the Lighthouse frame with a hand-moved Crazyflie."""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time
import uuid
from pathlib import Path

import numpy as np
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from cflib.crazyflie.log import LogConfig


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "code"))

from race_common.hoop_fit import (  # noqa: E402
    CircleFit3D,
    RectangleFit3D,
    fit_circle_3d,
    fit_rectangle_3d,
)
from day3.survey import spatially_subsample, write_gate  # noqa: E402


class PositionSampler:
    """Collect timestamped position estimates from the cflib logging thread."""

    def __init__(self, max_gap_s: float = 0.5) -> None:
        self._samples: list[tuple[float, np.ndarray]] = []
        self._lock = threading.Lock()
        self.first_sample = threading.Event()
        self.max_gap_s = max_gap_s
        self._last_sample: float | None = None
        self._trace_start: float | None = None
        self._error: str | None = None

    def fail(self, message: str) -> None:
        with self._lock:
            first_error = self._error is None
            if first_error:
                self._error = message
        if first_error:
            print(f"\nSurvey telemetry failed: {message}. Press Enter to end the trace; it will not be fitted.", flush=True)

    def callback(self, _timestamp: int, data: dict, _log_config: LogConfig) -> None:
        try:
            position = np.array([data[f"stateEstimate.{axis}"] for axis in "xyz"], dtype=float)
            if not np.all(np.isfinite(position)):
                raise ValueError("non-finite position")
        except (KeyError, TypeError, ValueError) as error:
            self.fail(f"invalid position packet: {error}")
            return
        now = time.monotonic()
        with self._lock:
            gap = (now - self._last_sample) if self._last_sample is not None else 0
            gap_in_trace = self._trace_start is not None and gap > self.max_gap_s
            self._samples.append((now, position))
            self._last_sample = now
        if gap_in_trace:
            self.fail(f"position gap {gap:.3f} s exceeds {self.max_gap_s:g} s")
        self.first_sample.set()

    def begin_trace(self) -> float:
        now = time.monotonic()
        with self._lock:
            if self._error:
                raise RuntimeError(self._error)
            if self._last_sample is None or now - self._last_sample > self.max_gap_s:
                raise RuntimeError("No fresh position data; leave the drone on the surface and retry the connection.")
            self._trace_start = now
        return now

    def monitor(self, finished: threading.Event) -> None:
        while not finished.wait(0.1):
            with self._lock:
                stale = self._last_sample is None or time.monotonic() - self._last_sample > self.max_gap_s
            if stale:
                self.fail(f"position updates stopped for more than {self.max_gap_s:g} s")
                return

    def finish_trace(self, start: float, end: float) -> str | None:
        with self._lock:
            self._trace_start = None
            stamps = [stamp for stamp, _ in self._samples if start <= stamp <= end]
            error = self._error
        # Check the whole interval as well as the watchdog: a resumed stream or
        # a last-packet tail gap must not turn an interrupted trace into a fit.
        gaps = np.diff([start, *stamps, end])
        if error:
            return error
        if not stamps or np.max(gaps) > self.max_gap_s:
            return f"incomplete position stream; maximum allowed gap is {self.max_gap_s:g} s"
        return None

    def save_failed_trace(self, directory: Path, start: float, end: float,
                          shape: str, reason: str) -> Path:
        with self._lock:
            samples = [{"host_monotonic_s": stamp, "position_m": position.tolist()}
                       for stamp, position in self._samples if start <= stamp <= end]
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"failed-{uuid.uuid4().hex}.json"
        record = {"status": "failed", "reason": reason, "shape": shape,
                  "position_source": "onboard stateEstimate", "max_gap_s": self.max_gap_s,
                  "start_host_monotonic_s": start, "end_host_monotonic_s": end,
                  "samples": samples}
        with path.open("x", encoding="utf-8") as stream:
            json.dump(record, stream, indent=2, allow_nan=False)
            stream.write("\n")
        return path

    def samples_since(self, start_time: float, end_time: float = float("inf")) -> np.ndarray:
        with self._lock:
            return np.asarray([
                position for timestamp, position in self._samples
                if start_time <= timestamp <= end_time
            ])


def _rounded(values: np.ndarray, digits: int = 6) -> list[float]:
    return [round(float(value), digits) for value in values]


def _print_fit(fit: CircleFit3D) -> None:
    print("\nCircle fit:")
    print(f"  centre       : {_rounded(fit.centre)} m")
    print(f"  normal       : {_rounded(fit.normal)}")
    print(f"  radius       : {fit.radius_m:.6f} m")
    print(f"  plane RMS/max: {fit.plane_rms_m:.6f} / {fit.plane_max_m:.6f} m")
    print(f"  radial RMS/max: {fit.radial_rms_m:.6f} / {fit.radial_max_m:.6f} m")


def _print_rectangle_fit(fit: RectangleFit3D) -> None:
    print("\nRectangle fit:")
    print(f"  centre       : {_rounded(fit.centre)} m")
    print(f"  normal       : {_rounded(fit.normal)}")
    print(f"  width axis   : {_rounded(fit.width_axis)}")
    print(f"  height axis  : {_rounded(fit.height_axis)}")
    print(f"  width x height: {fit.width_m:.6f} x {fit.height_m:.6f} m")
    print(f"  plane RMS/max: {fit.plane_rms_m:.6f} / {fit.plane_max_m:.6f} m")
    print(f"  outline RMS/max: {fit.outline_rms_m:.6f} / {fit.outline_max_m:.6f} m")


def _write_result(
    path: Path,
    hoop_name: str,
    fit: CircleFit3D,
    points: list[np.ndarray],
    force: bool,
) -> None:
    write_gate(path, hoop_name, fit, points, force)


def _write_rectangle_result(
    path: Path,
    gate_name: str,
    fit: RectangleFit3D,
    points: list[np.ndarray],
    force: bool,
) -> None:
    write_gate(path, gate_name, fit, points, force)


def _interactive_survey(
    sampler: PositionSampler,
    shape: str,
    minimum_spacing_m: float,
    failed_directory: Path,
) -> list[np.ndarray]:
    outline = "hoop rim" if shape == "circle" else "rectangular gate outline"
    print(f"Place the centre-referenced pointer on the {outline}.")
    print("Press Enter to start recording, move once around the full outline, then")
    print("press Enter again to stop. Firmware stop remains engaged until reboot.")
    while True:
        command = input("Command [Enter=record, q=quit]: ").strip().lower()
        if command == "q":
            return []
        if command:
            print("Unknown command. Press Enter to record or q to quit.")
            continue

        start_time = sampler.begin_trace()
        finished = threading.Event()
        watchdog = threading.Thread(target=sampler.monitor, args=(finished,), daemon=True)
        watchdog.start()
        interrupted = None
        try:
            input("Recording... press Enter after completing the outline: ")
        except (KeyboardInterrupt, EOFError) as error:
            interrupted = error
            sampler.fail("operator interrupted recording")
        finally:
            end_time = time.monotonic()
            finished.set()
            watchdog.join()
        failure = sampler.finish_trace(start_time, end_time)
        if failure:
            path = sampler.save_failed_trace(failed_directory, start_time, end_time, shape, failure)
            print(f"Rejected trace; partial observations saved to {path}")
            if interrupted is not None:
                raise interrupted
            raise RuntimeError("Survey acquisition failed; reconnect for a new trace or use checked shared geometry.")
        raw_points = sampler.samples_since(start_time, end_time)
        points = _spatially_subsample(raw_points, minimum_spacing_m)
        print(
            f"Captured {len(raw_points)} position samples; retained {len(points)} "
            "spatially separated points."
        )
        minimum_points = 6 if shape == "circle" else 12
        if len(points) < minimum_points:
            print(
                f"Need at least {minimum_points} retained points. Move more slowly "
                "around the complete outline or reduce --sample-spacing."
            )
            continue

        accept = input("Fit this trace? [Y/n]: ").strip().lower()
        if accept in ("", "y", "yes"):
            return [point for point in points]


def _spatially_subsample(points: np.ndarray, minimum_spacing_m: float) -> np.ndarray:
    """Keep samples separated in space, reducing dwell-time weighting."""
    if len(points) == 0:
        return np.empty((0, 3))
    return spatially_subsample(points, minimum_spacing_m)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--uri",
        default=os.getenv("CF_URI"),
        help="radio address supplied by the notebook's Connect cell",
    )
    parser.add_argument("--name", default="hoop-1", help="Hoop name in race configuration.")
    parser.add_argument(
        "--shape",
        choices=("circle", "rectangle"),
        default="circle",
        help="Geometry to survey.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("hoop-measurement.json"),
        help="Output JSON path.",
    )
    parser.add_argument("--force", action="store_true", help="Replace an existing output file.")
    parser.add_argument("--log-period-ms", type=int, default=50)
    parser.add_argument(
        "--sample-spacing",
        type=float,
        default=0.02,
        help="Minimum distance in metres between retained outline samples.",
    )
    parser.add_argument(
        "--normal-towards",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="Orient the fitted normal towards this Lighthouse-frame direction.",
    )
    args = parser.parse_args()
    if not args.uri:
        parser.error("use the survey command printed by the notebook after Connect")
    if args.log_period_ms <= 0 or args.sample_spacing <= 0.0:
        parser.error("log period and sample spacing must be positive")
    return args


def main() -> int:
    args = parse_args()
    # Keep --help and offline geometry tests usable without the radio libraries.
    from cflib.crazyflie.log import LogConfig
    from notebook_radio import motor_stopped_connection

    sampler = PositionSampler(max_gap_s=max(0.5, 3 * args.log_period_ms / 1000))
    log_config = LogConfig(name="Hoop survey", period_in_ms=args.log_period_ms)
    for variable in ("stateEstimate.x", "stateEstimate.y", "stateEstimate.z"):
        log_config.add_variable(variable, "float")

    print("Place the drone on a stable surface and keep clear of propellers.")
    if input("Type STOP AND SURVEY to engage the firmware stop and connect: ").strip() != "STOP AND SURVEY":
        print("Survey cancelled; no connection made.")
        return 1
    with motor_stopped_connection(args.uri, Path('./cache')) as (scf, _stop_check):
        print("STOP VERIFIED: firmware locked, four motor outputs zero. You may handle the drone.")
        print("Leave the stop engaged; place the drone down and power-cycle before flight/preflight checks.")
        scf.cf.log.add_config(log_config)
        log_config.data_received_cb.add_callback(sampler.callback)
        log_config.error_cb.add_callback(lambda _config, message: sampler.fail("logging error: " + str(message)))
        scf.cf.connection_lost.add_callback(lambda _uri, message: sampler.fail("connection lost: " + str(message)))
        log_config.start()
        try:
            if not sampler.first_sample.wait(timeout=5.0):
                raise RuntimeError("no position data received within five seconds")
            failed_directory = args.output.parent / f"{args.output.stem}-failed-traces"
            points = _interactive_survey(sampler, args.shape, args.sample_spacing, failed_directory)
        finally:
            for cleanup in (log_config.stop, log_config.delete):
                try:
                    cleanup()
                except Exception:
                    pass  # Link loss must not mask the saved acquisition failure.

    if not points:
        print("No measurement written.")
        return 1

    if args.shape == "circle":
        fit = fit_circle_3d(np.asarray(points), args.normal_towards)
        _print_fit(fit)
    else:
        fit = fit_rectangle_3d(np.asarray(points), args.normal_towards)
        _print_rectangle_fit(fit)
    if args.normal_towards is None:
        flip = input(
            "Flip the normal to select the opposite crossing direction? [y/N] "
        )
        if flip.lower() == "y":
            if isinstance(fit, CircleFit3D):
                fit = CircleFit3D(
                    centre=fit.centre,
                    normal=-fit.normal,
                    radius_m=fit.radius_m,
                    plane_rms_m=fit.plane_rms_m,
                    radial_rms_m=fit.radial_rms_m,
                    plane_max_m=fit.plane_max_m,
                    radial_max_m=fit.radial_max_m,
                )
            else:
                fit = RectangleFit3D(
                    centre=fit.centre,
                    normal=-fit.normal,
                    width_axis=fit.width_axis,
                    height_axis=-fit.height_axis,
                    width_m=fit.width_m,
                    height_m=fit.height_m,
                    plane_rms_m=fit.plane_rms_m,
                    outline_rms_m=fit.outline_rms_m,
                    plane_max_m=fit.plane_max_m,
                    outline_max_m=fit.outline_max_m,
                )
            print(f"  selected normal: {_rounded(fit.normal)}")

    if isinstance(fit, CircleFit3D):
        _write_result(args.output, args.name, fit, points, args.force)
    else:
        _write_rectangle_result(args.output, args.name, fit, points, args.force)
    print(f"Wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
