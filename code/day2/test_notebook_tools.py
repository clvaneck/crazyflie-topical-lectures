"""Notebook comparisons reuse the same filter and preserve participant decisions."""

import csv
import io
import json
from contextlib import redirect_stdout
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import ModuleType, SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import numpy as np

from .experiment_data import generate_data
from .experiments import ExperimentCase, export_prepared, run_experiment
from .notebook_tools import (_capture_receipt, _run_capture, add_log_config, capture,
                             completed_capture, next_attempt, read_imu)
from .extra_notebook_tools import capture_live, compare, select_data
from kalman.filter import FilterConfig


IMU_CHANNELS = (
    "acc.x", "acc.y", "acc.z",
    "gyro.x", "gyro.y", "gyro.z",
)


class Callbacks:
    def add_callback(self, callback):
        self.callback = callback


class RawCaptureTests(unittest.TestCase):
    def test_saves_elapsed_seconds_across_firmware_timestamp_wrap(self):
        timestamp_modulus = 1 << 24
        packets = [
            (timestamp_modulus - 10, dict(zip(IMU_CHANNELS, (1, 2, 3, 4, 5, 6)))),
            (10, dict(zip(IMU_CHANNELS, (7, 8, 9, 10, 11, 12)))),
        ]

        class FakeLogConfig:
            def __init__(self, name, period_in_ms):
                self.name = name
                self.period_in_ms = period_in_ms
                self.variables = []
                self.data_received_cb = Callbacks()
                self.error_cb = Callbacks()
                self.start = Mock(side_effect=lambda: [
                    self.data_received_cb.callback(stamp, data, self)
                    for stamp, data in packets
                ])
                self.stop = Mock()
                self.delete = Mock()

            def add_variable(self, name, variable_type):
                self.variables.append((name, variable_type))

        config_holder = {}

        def make_config(name, period_in_ms):
            config_holder["config"] = FakeLogConfig(name, period_in_ms)
            return config_holder["config"]

        cf = SimpleNamespace(
            log=SimpleNamespace(
                toc=SimpleNamespace(
                    get_element_by_complete_name=lambda _name: SimpleNamespace(ctype="float")
                ),
                add_config=Mock(),
            ),
            connection_lost=Callbacks(),
        )
        connection = MagicMock()
        connection.cf = cf
        cflib = ModuleType("cflib")
        cflib.crtp = ModuleType("cflib.crtp")
        cflib.crtp.init_drivers = Mock()
        crazyflie = ModuleType("cflib.crazyflie")
        crazyflie.Crazyflie = Mock()
        log_module = ModuleType("cflib.crazyflie.log")
        log_module.LogConfig = make_config
        sync_module = ModuleType("cflib.crazyflie.syncCrazyflie")
        sync_module.SyncCrazyflie = Mock(return_value=connection)
        modules = {
            module.__name__: module
            for module in (cflib, cflib.crtp, crazyflie, log_module, sync_module)
        }

        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "imu.csv"
            with patch.dict(sys.modules, modules), patch(
                "notebook_radio.engage_motor_stop", return_value={"verified": True}
            ), patch("notebook_radio.crazyflie.init"), patch(
                "day2.notebook_tools.time.monotonic",
                side_effect=[0, 0, 0, 1, 0, 0, 0, 1],
            ):
                path, count = capture(
                    "radio://0/80/2M/TEST", destination, IMU_CHANNELS, duration_s=.01
                )
                destination.write_text("stale data")
                with self.assertRaises(FileExistsError):
                    capture(
                        "radio://0/80/2M/TEST", destination, IMU_CHANNELS,
                        duration_s=.01,
                    )
                path, count = capture(
                    "radio://0/80/2M/TEST", destination, IMU_CHANNELS,
                    duration_s=.01, overwrite=True,
                )

            self.assertEqual(path, destination)
            self.assertEqual(count, 2)
            self.assertEqual(completed_capture(destination), destination)
            receipt = json.loads(Path(str(destination) + ".capture.json").read_text())
            self.assertEqual(receipt["motor_stop_check"], {"verified": True})
            self.assertEqual(receipt["status"], "complete")
            self.assertEqual(receipt["rows"], 2)
            self.assertEqual(receipt["timestamp_unit"], "seconds")
            self.assertEqual(receipt["timestamp_origin"], "first firmware sample")
            with destination.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(list(rows[0]), ["time_s", *IMU_CHANNELS])
            self.assertEqual(rows[0]["time_s"], "0.0")
            self.assertEqual(rows[1]["time_s"], "0.02")
            self.assertEqual(rows[1]["gyro.z"], "12.0")
            self.assertEqual(config_holder["config"].period_in_ms, 20)
            self.assertEqual(
                config_holder["config"].variables,
                [(name, "float") for name in IMU_CHANNELS],
            )
            self.assertEqual(crazyflie.Crazyflie.call_count, 2)
            for call in crazyflie.Crazyflie.call_args_list:
                self.assertEqual(
                    call.kwargs["rw_cache"],
                    str(destination.parent / ".cflib-cache"),
                )

    def test_partial_interrupted_and_changed_captures_are_not_auto_selected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "imu.csv"
            path.write_text("unchecked cfclient data")
            self.assertIsNone(completed_capture(path))
            path.unlink()
            for error, status in ((RuntimeError("radio lost"), "failed"),
                                  (KeyboardInterrupt(), "interrupted")):
                with self.subTest(status=status):
                    with self.assertRaises(type(error)):
                        with _capture_receipt(path, {}) as receipt:
                            path.write_text("partial data")
                            receipt["rows"] = 20
                            raise error
                    self.assertIsNone(completed_capture(path))
                    receipt_path = Path(str(path) + ".capture.json")
                    self.assertEqual(json.loads(receipt_path.read_text())["status"], status)
                    with self.assertRaises(FileExistsError):
                        with _capture_receipt(path, {}):
                            pass
                    path.unlink()
                    receipt_path.unlink()
            with _capture_receipt(path, {}) as receipt:
                path.write_text("complete data")
                receipt.update(status="complete", rows=2)
            self.assertEqual(completed_capture(path), path)
            path.write_text("changed data")
            with self.assertRaisesRegex(ValueError, "changed after completion"):
                completed_capture(path)

    def test_imu_csv_accepts_bom_and_rejects_nonfinite_or_short_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "imu.csv"
            header = ",".join(IMU_CHANNELS) + "\n"
            path.write_text(header + "0,0,1,1,2,3\n" * 2, encoding="utf-8-sig")
            np.testing.assert_array_equal(read_imu(path), [[0, 0, 1, 1, 2, 3]] * 2)
            for rows in ("", "0,0,1,1,2,3\n", "0,0,1,nan,2,3\n" * 2):
                path.write_text(header + rows)
                with self.assertRaisesRegex(ValueError, "two finite"):
                    read_imu(path)

    def test_log_config_uses_requested_period_and_toc_types(self):
        class FakeLogConfig:
            def __init__(self, name, period_in_ms):
                self.name = name
                self.period_in_ms = period_in_ms
                self.variables = []
                self.data_received_cb = Callbacks()
                self.error_cb = Callbacks()

            def add_variable(self, name, variable_type):
                self.variables.append((name, variable_type))

        made = {}

        def make_config(name, period_in_ms):
            made["config"] = FakeLogConfig(name, period_in_ms)
            return made["config"]

        log_module = ModuleType("cflib.crazyflie.log")
        log_module.LogConfig = make_config
        with patch.dict(sys.modules, {"cflib.crazyflie.log": log_module}):
            cf = SimpleNamespace(
                log=SimpleNamespace(
                    toc=SimpleNamespace(
                        get_element_by_complete_name=lambda name:
                        SimpleNamespace(ctype="uint16_t" if name == "range.zrange" else "float")
                    ),
                    add_config=Mock(),
                )
            )
            config = add_log_config(
                cf, "test", ("acc.x", "range.zrange"), 35, Mock(), Mock()
            )
        self.assertIs(config, made["config"])
        self.assertEqual(config.period_in_ms, 35)
        self.assertEqual(config.variables, [("acc.x", "float"), ("range.zrange", "uint16_t")])

    def test_capture_rejects_bad_channels_and_periods(self):
        for channels in ((), ("acc.x", "acc.x"), "acc.x"):
            with self.subTest(channels=channels):
                with self.assertRaisesRegex(ValueError, "channel names"):
                    capture("radio://0/80/2M/TEST", "unused.csv", channels)
        for period in (0, -20, 20.0, True):
            with self.subTest(period=period):
                with self.assertRaisesRegex(ValueError, "positive integer"):
                    capture("radio://0/80/2M/TEST", "unused.csv", IMU_CHANNELS,
                            period_ms=period)


class NotebookToolsTests(unittest.TestCase):
    def test_unselected_data_writes_no_comparison(self):
        cases = [ExperimentCase("baseline", FilterConfig(), "nominal")]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "comparison"
            with self.assertRaisesRegex(ValueError, "data-selection cell"):
                compare(None, cases, output)
            self.assertFalse(output.exists())

    def test_invalid_dropout_comparison_writes_no_baseline_or_plot(self):
        data = generate_data(0, "nominal", duration_s=5)
        cases = [ExperimentCase("complete", FilterConfig(), "nominal"),
                 ExperimentCase("gap", FilterConfig(), "dropout", dropout_s=3)]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "comparison"
            with self.assertRaisesRegex(ValueError, "synthetic fallback"):
                compare(data, cases, output)
            self.assertFalse(output.exists())

    def test_repeat_attempts_preserve_results_and_allow_retry_after_policy_error(self):
        data = generate_data(0, "nominal", duration_s=1)
        case = ExperimentCase("baseline", FilterConfig(), "nominal", participant_gate=True)
        with tempfile.TemporaryDirectory() as directory:
            parent = Path(directory) / "E4"
            first = next_attempt(parent)
            compare(data, [case], first, predicted_residual_policy=lambda dimension, alpha: None)
            original = (first / "baseline/updates.csv").read_bytes()
            second = next_attempt(parent)
            compare(data, [case], second, predicted_residual_policy=lambda dimension, alpha: 1.0)
            self.assertNotEqual(first, second)
            self.assertEqual((first / "baseline/updates.csv").read_bytes(), original)
            failed = next_attempt(parent)
            with self.assertRaises(ValueError):
                compare(data, [case], failed, predicted_residual_policy=lambda dimension, alpha: -1)
            retry = next_attempt(parent)
            self.assertNotEqual(failed, retry)
            compare(data, [case], retry, predicted_residual_policy=lambda dimension, alpha: None)
            self.assertEqual(first.name, "attempt-001")
            self.assertEqual(retry.name, "attempt-004")

    def test_capture_checks_before_starting_radio_process(self):
        with tempfile.TemporaryDirectory() as directory, patch("day2.extra_notebook_tools._run_capture") as run:
            output = Path(directory) / "capture"
            for kwargs in ({}, {"ready_for_motor_stop": True, "duration_s": 121},
                           {"ready_for_motor_stop": True, "duration_s": float("nan")},
                           {"ready_for_motor_stop": True, "position_std": -1}):
                with self.assertRaises(ValueError):
                    capture_live(output, "radio://0/80/2M/E7E7E7E7E7", **kwargs)
            with self.assertRaises(ValueError):
                capture_live(output, "", ready_for_motor_stop=True)
            with self.assertRaises(FileExistsError):
                capture_live(directory, "test://no-radio", ready_for_motor_stop=True)
            run.assert_not_called()
            self.assertFalse(output.exists())

    def test_capture_is_bounded_and_failed_inputs_are_not_selected(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "failed"
            for error in (subprocess.TimeoutExpired(["logger"], 31), KeyboardInterrupt()):
                with patch("day2.extra_notebook_tools._run_capture", side_effect=error) as run:
                    with self.assertRaises(TimeoutError if isinstance(error, subprocess.TimeoutExpired)
                                           else KeyboardInterrupt):
                        capture_live(output, "test://no-radio", duration_s=1, ready_for_motor_stop=True)
                    self.assertEqual(run.call_args.kwargs["timeout_s"], 31)
            with patch("day2.extra_notebook_tools._run_capture", return_value=
                       subprocess.CompletedProcess([], 1, "", "motor outputs are nonzero")):
                with self.assertRaisesRegex(RuntimeError, "motor outputs are nonzero"):
                    capture_live(output, "test://no-radio", ready_for_motor_stop=True)

    def test_capture_returns_only_complete_validated_recordings_without_propeller_confirmation(self):
        from .live import CHANNELS, LiveSession

        status = "complete"

        def logger(command, **kwargs):
            self.assertEqual(command[1:4], ["-u", "-m", "day2.live"])
            self.assertIn("--prepare-for-handling", command)
            self.assertIn("--no-show", command)
            self.assertEqual(kwargs["env"]["MPLBACKEND"], "Agg")
            self.assertEqual(kwargs["timeout_s"], 90)
            session = LiveSession(command[command.index("--output") + 1],
                                  [FilterConfig(), FilterConfig()], [0, 0, 0], "test://no-radio")
            for timestamp in (100, 150, 200):
                for stream, values in (("motors", [0, 0, 0, 0]),
                                       ("attitude", [0, 0, 0, 1]),
                                       ("imu_position", [0, 0, 1, 0, 0, 1])):
                    session.enqueue(stream, timestamp, dict(zip(CHANNELS[stream], values)), 0)
            session.drain(1, force=True)
            session.finish(status)
            return subprocess.CompletedProcess(command, 0, "Saved test recording\n", "")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            with patch("day2.extra_notebook_tools._run_capture", side_effect=logger):
                recording, metadata = capture_live(output, "test://no-radio", ready_for_motor_stop=True)
            self.assertEqual(recording, output.resolve() / "prepared.csv")
            self.assertEqual(json.loads(metadata.read_text())["prepared_samples"], 3)
            data = select_data(recording, metadata)
            self.assertFalse(data.has_truth)
            # Success exit alone is insufficient if the logger says interrupted.
            status = "interrupted"
            with patch("day2.extra_notebook_tools._run_capture", side_effect=logger):
                with self.assertRaisesRegex(RuntimeError, "did not complete"):
                    capture_live(Path(directory) / "interrupted", "test://no-radio", ready_for_motor_stop=True)

    def test_progress_is_relayed_and_timeout_kills_child(self):
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(io.StringIO()) as output:
            command = [sys.executable, "-u", "-c", "print('STOP VERIFIED', flush=True)"]
            result = _run_capture(command, cwd=directory, env=None, timeout_s=5)
            self.assertEqual(result.returncode, 0)
            self.assertIn("STOP VERIFIED", output.getvalue())
            child = subprocess.Popen
            processes = []

            def start(*args, **kwargs):
                process = child(*args, **kwargs)
                processes.append(process)
                return process

            with patch("day2.notebook_tools.subprocess.Popen", side_effect=start):
                with self.assertRaises(subprocess.TimeoutExpired):
                    _run_capture([sys.executable, "-c", "import time; time.sleep(10)"],
                                 cwd=directory, env=None, timeout_s=0.1)
            self.assertIsNotNone(processes[0].poll())

    def test_selected_recordings_do_not_silently_become_simulation(self):
        with self.assertRaises(ValueError):
            select_data("missing.csv", None)
        with self.assertRaises(OSError):
            select_data("missing.csv", "missing.json")
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            case = ExperimentCase("source", FilterConfig(), "nominal")
            export_prepared(generate_data(0, "nominal"), case, path, 0)
            data = select_data(path / "prepared.csv", path / "recording.json")
            self.assertFalse(data.has_truth)
            self.assertTrue(data.metadata["synthetic"])
            self.assertTrue(np.all(np.isnan(data.measured_flow)))

    def test_notebook_gate_changes_results_without_modifying_global_policies(self):
        from . import exercise_policies

        original = exercise_policies.predicted_residual_threshold
        data = generate_data(0, "outlier")
        case = ExperimentCase("gate", FilterConfig(), "outlier", participant_gate=True)
        baseline = run_experiment(data, case)
        dimensions = set()

        def threshold(dimension, alpha):
            dimensions.add(dimension)
            return 1.0

        changed = run_experiment(data, case, predicted_residual_policy=threshold)
        self.assertEqual(dimensions, {1, 2, 3})
        self.assertTrue(all(event["accepted"] for event in baseline["events"]))
        self.assertTrue(any(not event["accepted"] for event in changed["events"]))
        self.assertIs(exercise_policies.predicted_residual_threshold, original)

    def test_notebook_stationary_decision_is_used(self):
        data = generate_data(0, "stationary", duration_s=2.0)
        case = ExperimentCase("zupt", FilterConfig(), "stationary", participant_stationary=True)
        no_updates = run_experiment(data, case)
        # Deliberately permissive for this wiring test, not an exercise solution.
        changed = run_experiment(data, case, stationary_policy=lambda window: True)
        self.assertFalse(any(event["sensor"] == "zero_velocity" for event in no_updates["events"]))
        self.assertTrue(any(event["sensor"] == "zero_velocity" for event in changed["events"]))

    def test_saved_results_are_labelled_and_existing_outputs_are_preserved(self):
        data = select_data(seed=0)
        case = ExperimentCase("baseline", FilterConfig(), "nominal")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "comparison"
            summary = compare(data, [case], output, notes="Prediction: compare the bands.")
            self.assertTrue(summary[0]["has_truth"])
            settings = json.loads((output / "settings.json").read_text())
            self.assertTrue(settings["source"]["synthetic"])
            self.assertTrue((output / "baseline/states.csv").is_file())
            self.assertTrue((output / "baseline/updates.csv").is_file())
            self.assertTrue((output / "comparison.png").is_file())
            with self.assertRaises(FileExistsError):
                compare(data, [case], output)


if __name__ == "__main__":
    unittest.main()
