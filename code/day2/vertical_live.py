"""Bounded, motor-stopped accelerometer/ToF capture for the vertical experiment."""

import argparse
from collections import deque
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import subprocess
import sys
import time

from race_common.motor_stop import ACTIVE_BITS, LOCKED_BIT, MOTOR_CHANNELS
from notebook_radio import motor_stopped_connection
from .vertical import (LIVE_COLUMNS, LIVE_SENSOR_COLUMNS,
                       VERTICAL_SENSOR_COLUMNS, calibrate, load_raw)


SENSOR_CHANNELS = LIVE_SENSOR_COLUMNS
STATUS_CHANNELS = ('supervisor.info', *MOTOR_CHANNELS)


def check_status(data):
    values = [float(data[name]) for name in STATUS_CHANNELS]
    if (not all(math.isfinite(value) for value in values) or not values[0].is_integer()
            or not 0 <= values[0] <= 65535):
        raise ValueError("Invalid motor-stop status telemetry.")
    info = int(values[0])
    if not info & LOCKED_BIT or info & ACTIVE_BITS or any(value != 0 for value in values[1:]):
        raise ValueError("Firmware stop is not locked with zero motor outputs; stop handling the drone.")


def collect(output, uri, duration_s=20):
    if not isinstance(uri, str) or not uri.strip() or not math.isfinite(duration_s) or not 18 <= duration_s <= 30:
        raise ValueError("Provide the assigned URI and an 18-30 second duration.")
    # Offline work and notebook import require no USB/radio libraries.
    import numpy as np
    from cflib.crazyflie.log import LogConfig

    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    metadata = {"format": "vertical-sensors-v2", "source": "IMU and downward ToF sensor logs", "uri": uri,
                "units": {"time_s": "seconds", "gyro.x/gyro.y/gyro.z": "degrees per second",
                          "acc.x/acc.y/acc.z": "g", "range.zrange": "millimetres"},
                "columns": list(LIVE_COLUMNS), "status": "recording", "channels": list(SENSOR_CHANNELS),
                "vertical_analysis_channels": list(VERTICAL_SENSOR_COLUMNS),
                "timing": "24-bit firmware log milliseconds, unwrapped relative to first sensor packet. Log times approximate sensor times; no delay compensation. Acceleration approximates the preceding interval.",
                "model": "Level drone, gentle vertical motion, fixed flat floor; no onboard stateEstimate channels.",
                "known_rest_intervals_s": []}
    (output / 'recording.json').write_text(json.dumps(metadata, indent=2) + '\n')
    rows, configs = [], []
    events = queue.Queue(maxsize=1000)
    errors = queue.Queue()
    origin = previous_ms = None
    status_stamps = deque(maxlen=10)
    deadline = time.monotonic() + duration_s + 15
    calibrated = False
    final_rest_prompted = False

    def receive(stream):
        def callback(stamp, data, _config):
            try:
                events.put_nowait((stream, stamp, dict(data)))
            except queue.Full:
                errors.put("Telemetry queue overflow.")
        return callback

    try:
        with (output / 'raw.csv').open('x', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(LIVE_COLUMNS)
            with motor_stopped_connection(uri, output / '.cflib-cache') as (connection, stop_check):
                cf = connection.cf
                metadata['motor_stop_check'] = stop_check
                missing = [name for name in SENSOR_CHANNELS
                           if cf.log.toc.get_element_by_complete_name(name) is None]
                if missing:
                    raise ValueError("Required sensor channels missing: " + ', '.join(missing))
                cf.connection_lost.add_callback(lambda _uri, reason: errors.put(str(reason)))
                print("STOP VERIFIED. Keep the drone stationary and level for five seconds over a fixed flat floor.", flush=True)
                try:
                    # Separate packets keep both logs within the firmware payload limit.
                    for name, channels in (('status', STATUS_CHANNELS), ('sensors', SENSOR_CHANNELS)):
                        elements = {
                            channel: cf.log.toc.get_element_by_complete_name(channel)
                            for channel in channels
                        }
                        missing = [channel for channel, element in elements.items()
                                   if element is None]
                        if missing:
                            raise ValueError("Required log channels missing: " + ', '.join(missing))
                        log = LogConfig(name='Vertical_' + name, period_in_ms=50)
                        for channel, element in elements.items():
                            log.add_variable(channel, element.ctype)
                        cf.log.add_config(log)
                        configs.append(log)
                        log.data_received_cb.add_callback(receive(name))
                        log.error_cb.add_callback(lambda _log, message: errors.put(str(message)))
                        log.start()
                    while True:
                        if time.monotonic() >= deadline:
                            raise TimeoutError("Capture exceeded its wall-clock limit; use the fallback.")
                        if not errors.empty():
                            raise ValueError(errors.get_nowait())
                        try:
                            kind, stamp, data = events.get(timeout=1)
                        except queue.Empty as cause:
                            raise TimeoutError("Sensor/status telemetry stopped.") from cause
                        if kind == 'status':
                            check_status(data)
                            status_stamps.append(int(stamp) % (1 << 24))
                            continue
                        if not status_stamps:
                            continue
                        # Arrival order may differ across blocks: use a causal status packet.
                        age_ms = min((int(stamp) - status) % (1 << 24) for status in status_stamps)
                        if age_ms > 150 and previous_ms is None:
                            continue
                        if age_ms > 150 or int(cf.param.get_value('stabilizer.stop')) != 1:
                            raise ValueError("Motor-stop status is stale or stop was cleared; recording stopped.")
                        raw_ms = int(stamp) % (1 << 24)
                        if previous_ms is None:
                            unwrapped_ms = origin = raw_ms
                        else:
                            delta = (raw_ms - previous_ms % (1 << 24) + (1 << 23)) % (1 << 24) - (1 << 23)
                            if delta <= 0 or delta > 300:
                                raise ValueError("Duplicate, reversed or excessive-gap sensor timestamps.")
                            unwrapped_ms = previous_ms + delta
                        previous_ms = unwrapped_ms
                        t = (unwrapped_ms - origin) / 1000
                        capture_row = [t, *(float(data[name]) for name in SENSOR_CHANNELS)]
                        if not all(math.isfinite(value) for value in capture_row):
                            raise ValueError("Non-finite sensor measurement.")
                        rows.append([t, *(float(data[name]) for name in VERTICAL_SENSOR_COLUMNS)])
                        writer.writerow(capture_row)
                        stream.flush()
                        if t >= 5 and not calibrated:
                            calibration = calibrate(np.asarray(rows))
                            metadata['calibration_check'] = vars(calibration)
                            calibrated = True
                            print("Calibration check passed. Move gently UP, then DOWN, keeping level and the ToF beam on the same floor. Hold still again by t=15 s.", flush=True)
                        if t >= 15 and not final_rest_prompted:
                            print("t=15 s: hold the drone stationary and level until capture ends; note whether you did so.", flush=True)
                            final_rest_prompted = True
                        if t >= duration_s:
                            break
                finally:
                    for log in configs:
                        for action in (log.stop, log.delete):
                            try:
                                action()
                            except Exception:
                                pass  # Never clear firmware stop, including after radio loss.
        metadata['status'] = 'complete'
    except BaseException as error:
        metadata.update(status='failed', reason=str(error) or type(error).__name__)
        raise
    finally:
        raw_path = output / 'raw.csv'
        if raw_path.exists():
            metadata['raw_sha256'] = hashlib.sha256(raw_path.read_bytes()).hexdigest()
        (output / 'recording.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print("Firmware stop remains engaged. Place the drone down and power-cycle before flight/preflight checks.", flush=True)
    load_raw(output)
    return output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uri', default=os.getenv('CF_URI'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--prepare-for-handling', action='store_true')
    parser.add_argument('--capture-worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if not args.uri or not args.prepare_for_handling:
        parser.error('Use the notebook capture cell, or pass its discovered --uri; place the drone on a stable surface, clear of propellers, close other radio clients, then confirm --prepare-for-handling.')
    if not math.isfinite(args.duration) or not 18 <= args.duration <= 30:
        parser.error('Capture duration must be between 18 and 30 seconds.')
    try:
        if args.capture_worker:
            collect(args.output, args.uri, args.duration)
        else:
            from .notebook_tools import _run_capture

            # Radio connection and parameter waits can block inside cflib. Bound
            # the entire worker lifetime, including startup and cleanup.
            command = [sys.executable, '-u', '-m', 'day2.vertical_live',
                       '--uri', args.uri, '--output', str(args.output.resolve()),
                       '--duration', str(args.duration), '--prepare-for-handling', '--capture-worker']
            print(f'Overall capture limit: {args.duration + 30:g} s, including connection and parameters. '
                  'Leave the drone on the surface until STOP VERIFIED.', flush=True)
            result = _run_capture(command, cwd=Path(__file__).resolve().parents[1],
                                  env=dict(os.environ), timeout_s=args.duration + 30)
            if result.returncode:
                parser.exit(1, 'Capture worker failed. Preserve partial files and select a checked recording or simulation.\n')
    except subprocess.TimeoutExpired:
        parser.exit(1, f'Capture exceeded {args.duration + 30:g} s; worker stopped. '
                    'Put the drone down and power-cycle before flight. Preserve partial files '
                    'and select a checked recording or simulation.\n')
    except (Exception, KeyboardInterrupt) as error:
        parser.exit(1, f'Capture stopped: {error}. Preserve partial files and select a checked recording or simulation.\n')
    if not args.capture_worker:
        print(f'Saved raw sensor recording in {args.output}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
