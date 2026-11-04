"""Offline tests for notebook flight handoffs; no radio or motors are used."""

from contextlib import redirect_stdout
from hashlib import sha256
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from . import flight
from .notebook_tools import _course_digest


class FlightNotebookTests(unittest.TestCase):
    def test_notebook_filenames_sort_in_working_order(self):
        directory = Path(__file__).parent
        self.assertEqual(sorted(path.name for path in directory.glob("*.ipynb")), [
            "01_survey.ipynb", "02_trajectory.ipynb", "03_flight.ipynb",
            "04_extra_uncertainty.ipynb",
        ])
        notebook = json.loads((directory / "02_trajectory.ipynb").read_text())
        self.assertEqual("".join(notebook["cells"][0]["source"]).splitlines()[0],
                         "# Plan the trajectory and analyse the flight")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.course = self.root / "course.json"
        self.course.write_text(json.dumps(dict(course_name="test-course", enabled_for_flight=True)))
        self.save_summary(self.course)
        self.runner = self.root / "runner.py"
        self.runner.write_text('''import sys
if "--fly" in sys.argv:
    answer = input("confirm: ")
    assert answer == "test-course"
    print("mock landing completed")
else:
    print("mock dry run")
''')

    def save_summary(self, course):
        planned = json.loads(course.read_text(encoding="utf-8-sig"))
        planned["enabled_for_flight"] = False
        summary = dict(course_sha256=_course_digest(planned),
                       provenance=dict(source_surveys=["gate.json"],
                                       clearance_budget={"required_margin_m": .1}))
        course.with_name("summary.json").write_text(json.dumps(summary), encoding="utf-8-sig")
        return summary

    def check(self):
        return flight.dry_run(self.runner, self.course, sys.executable,
                              self.root, self.root / "dry.txt")

    def test_dry_run_and_explicit_flight_keep_runner_confirmation(self):
        with redirect_stdout(io.StringIO()):
            checked = self.check()
            flight.fly(checked, "radio://0/9/2M/E7E7E7E701", self.root / "telemetry.csv",
                       self.root / "flight.txt", ready=True, confirm=lambda _: "test-course")
        self.assertIn("mock dry run", (self.root / "dry.txt").read_text())
        self.assertIn("mock landing completed", (self.root / "flight.txt").read_text())

    def test_venv_interpreter_path_is_preserved(self):
        python = Path(sys.executable).absolute()
        with redirect_stdout(io.StringIO()):
            checked = self.check()
        self.assertEqual(checked.python, python)
        # In a normal virtual environment prefix differs from base_prefix.
        self.runner.write_text("import sys; print(sys.prefix, flush=True)\n")
        with redirect_stdout(io.StringIO()):
            flight.dry_run(self.runner, self.course, python, self.root, self.root / "prefix.txt")
        self.assertEqual((self.root / "prefix.txt").read_text().strip(), sys.prefix)

    def test_readiness_confirmation_and_changed_inputs_prevent_launch(self):
        with redirect_stdout(io.StringIO()):
            checked = self.check()
        arguments = (checked, "radio://0/9/2M/E7E7E7E701", self.root / "t.csv", self.root / "f.txt")
        with patch.object(flight, "_run") as launch:
            with self.assertRaisesRegex(ValueError, "ready=True"):
                flight.fly(*arguments)
            with self.assertRaisesRegex(ValueError, "did not match"):
                flight.fly(*arguments, ready=True, confirm=lambda _: "wrong")
            self.course.write_text(self.course.read_text() + "\n")
            with self.assertRaisesRegex(ValueError, "changed"):
                flight.fly(*arguments, ready=True, confirm=lambda _: "test-course")
            launch.assert_not_called()

    def test_disabled_course_and_existing_files_are_rejected(self):
        self.course.write_text(json.dumps(dict(course_name="test-course", enabled_for_flight=False)))
        with redirect_stdout(io.StringIO()):
            checked = self.check()
        with self.assertRaisesRegex(ValueError, "enabled_for_flight"):
            flight.fly(checked, "radio://0/9/2M/E7E7E7E701", self.root / "t.csv",
                       self.root / "f.txt", ready=True)
        with self.assertRaises(FileExistsError):
            self.check()

    def test_notebook_repeats_dry_runs_after_enabling_and_after_failure(self):
        notebook = json.loads(Path(__file__).with_name("03_flight.ipynb").read_text())
        code = next("".join(cell["source"]) for cell in notebook["cells"]
                    if cell["id"] == "flight-tool")
        values = dict(runner=self.runner, course_json=self.course,
                      runner_python=Path(sys.executable), runner_directory=self.root)
        for name, value in values.items():
            code = code.replace(f"{name} = None", f"{name} = Path({str(value)!r})")
        code = code.replace("run_dry_check = False", "run_dry_check = True")
        scope = dict(Path=Path, output=self.root, new_attempt=flight.new_attempt, dry_run=flight.dry_run,
                     tuning_spec=None)
        self.course.write_text(json.dumps(dict(course_name="test-course", enabled_for_flight=False)))
        with redirect_stdout(io.StringIO()):
            exec(code, scope)
            first_report = (scope["check_directory"] / "console.txt")
            original_report = first_report.read_bytes()
            self.course.write_text(json.dumps(dict(course_name="test-course", enabled_for_flight=True)))
            exec(code, scope)
            second_report = scope["check_directory"] / "console.txt"
            self.assertNotEqual(first_report, second_report)
            working_runner = self.runner.read_text()
            self.runner.write_text("print('failed check', flush=True)\nraise SystemExit(1)\n")
            with self.assertRaises(RuntimeError):
                exec(code, scope)
            self.assertIsNone(scope["checked_run"])
            failed_report = scope["check_directory"] / "console.txt"
            self.assertIn("failed check", failed_report.read_text())
            self.runner.write_text(working_runner)
            exec(code, scope)
        self.assertIsInstance(scope["checked_run"], flight.CheckedRun)
        self.assertEqual(first_report.read_bytes(), original_report)
        self.assertEqual(len(list((self.root / "checks").glob("attempt-*/console.txt"))), 4)

    def test_example_cells_generate_matching_evidence_before_radio_free_dry_run(self):
        notebook = json.loads(Path(__file__).with_name("03_flight.ipynb").read_text())
        cells = {cell["id"]: "".join(cell["source"]) for cell in notebook["cells"]}
        ids = [cell["id"] for cell in notebook["cells"]]
        self.assertLess(ids.index("flight-example"), ids.index("flight-connect"))
        prepare = cells["flight-example"].replace("prepare_example = False", "prepare_example = True")
        check = cells["flight-tool"].replace("course_json = None", "course_json = example_course_json")
        check = check.replace("run_dry_check = False", "run_dry_check = True")
        paths = dict(runner=self.runner, runner_python=Path(sys.executable), runner_directory=self.root)
        for name, path in paths.items():
            check = check.replace(f"{name} = None", f"{name} = Path({str(path)!r})")
        scope = dict(Path=Path, root=Path(__file__).resolve().parents[2], output=self.root,
                     new_attempt=flight.new_attempt, dry_run=flight.dry_run,
                     tuning_spec=object(), checked_run=object())
        previous = None
        for _ in range(2):
            with redirect_stdout(io.StringIO()):
                exec(prepare, scope)
                exec(check, scope)
            course = scope["example_course_json"]
            summary = json.loads(course.with_name("summary.json").read_text())
            self.assertFalse(json.loads(course.read_text())["enabled_for_flight"])
            self.assertEqual(summary["course_sha256"], _course_digest(json.loads(course.read_text())))
            self.assertEqual(summary["provenance"]["geometry_status"], "synthetic")
            self.assertEqual(scope["checked_run"].summary_bytes, course.with_name("summary.json").read_bytes())
            self.assertIsNone(scope["tuning_spec"])
            if previous is not None:
                self.assertNotEqual(course, previous)
                self.assertTrue(previous.is_file())
            previous = course
        scope["root"] = self.root / "missing-project"
        with self.assertRaises(FileNotFoundError):
            exec(prepare, scope)
        self.assertIsNone(scope["example_course_json"])
        self.assertIsNone(scope["checked_run"])

    def test_actual_runner_uses_archived_bytes_and_replanning_does_not_replace_them(self):
        self.runner.write_text('''import json, sys
from pathlib import Path
if "--fly" in sys.argv:
    assert input("confirm: ") == "test-course"
    course = Path(sys.argv[sys.argv.index("--config") + 1])
    telemetry = Path(sys.argv[sys.argv.index("--log") + 1])
    telemetry.write_text(json.dumps({"course_path": str(course), "course": course.read_text()}))
''')
        # Preserve exact bytes, including a Windows BOM and insignificant spaces.
        original = b'\xef\xbb\xbf{ "course_name": "test-course", "enabled_for_flight": true }\n'
        self.course.write_bytes(original)
        with redirect_stdout(io.StringIO()):
            checked = self.check()
            attempt = flight.new_attempt(self.root / "attempts")
            telemetry = attempt / "race.csv"
            archived = flight.fly(checked, "radio://0/9/2M/E7E7E7E701", telemetry,
                                  attempt / "console.txt", ready=True, confirm=lambda _: "test-course")
        evidence = json.loads(telemetry.read_text())
        self.assertEqual(Path(evidence["course_path"]), archived)
        self.assertEqual(archived.read_bytes(), original)
        summary_path = telemetry.with_suffix(".summary.json")
        summary = json.loads(summary_path.read_text())
        self.assertEqual(summary["provenance"], json.loads(checked.summary_bytes.decode("utf-8-sig"))["provenance"])
        self.assertEqual(summary["flown_course_sha256"], sha256(original).hexdigest())
        self.assertIsNone(summary["tuning_sha256"])
        summary_bytes = summary_path.read_bytes()
        self.course.write_text('{"course_name": "replanned"}')
        self.course.with_name("summary.json").write_text("changed planning evidence")
        self.assertEqual(archived.read_bytes(), original)
        self.assertEqual(summary_path.read_bytes(), summary_bytes)
        with patch.object(flight, "_run") as launch:
            with self.assertRaises(FileExistsError):
                flight.fly(checked, "radio://0/9/2M/E7E7E7E701", telemetry,
                           attempt / "console.txt", ready=True)
            launch.assert_not_called()

    def test_abort_keeps_the_archived_course_and_reports_its_path(self):
        with redirect_stdout(io.StringIO()):
            checked = self.check()
        expected = self.course.read_bytes()
        output = io.StringIO()
        for error in (RuntimeError("aborted"), KeyboardInterrupt()):
            with self.subTest(error=type(error).__name__):
                attempt = flight.new_attempt(self.root / "attempts")
                telemetry = attempt / "race.csv"
                archive = telemetry.with_suffix(".course.json")

                def interrupted(arguments, *_args):
                    self.assertEqual(arguments[arguments.index("--config") + 1], str(archive))
                    raise error

                with patch.object(flight, "_run", side_effect=interrupted), redirect_stdout(output):
                    with self.assertRaises(type(error)):
                        flight.fly(checked, "radio://0/9/2M/E7E7E7E701", telemetry,
                                   attempt / "console.txt", ready=True, confirm=lambda _: "test-course")
                self.assertEqual(archive.read_bytes(), expected)
                summary_path = telemetry.with_suffix(".summary.json")
                summary = json.loads(summary_path.read_text())
                self.assertEqual(summary["flown_course_sha256"], sha256(expected).hexdigest())
                self.assertEqual(summary["provenance"], json.loads(checked.summary_bytes.decode("utf-8-sig"))["provenance"])
                self.assertIn(str(archive), output.getvalue())
                self.assertIn(str(summary_path), output.getvalue())

    def test_summary_is_required_and_bound_before_dry_run(self):
        path = self.course.with_name("summary.json")
        original = path.read_bytes()
        for content in (None, b"invalid JSON", b"{}", b"[]"):
            with self.subTest(content=content), patch.object(flight, "_run") as launch:
                if content is None:
                    path.unlink()
                else:
                    path.write_bytes(content)
                with self.assertRaises(ValueError):
                    self.check()
                launch.assert_not_called()
        path.write_bytes(original)
        course = json.loads(self.course.read_text())
        course["course_name"] = "another-course"
        self.course.write_text(json.dumps(course))
        with patch.object(flight, "_run") as launch:
            with self.assertRaisesRegex(ValueError, "summary does not match"):
                self.check()
            launch.assert_not_called()

    def test_summary_changes_during_dry_run_or_confirmation_block_launch(self):
        path = self.course.with_name("summary.json")
        original = path.read_bytes()

        def change_summary(*_args, **_kwargs):
            summary = json.loads(original.decode("utf-8-sig"))
            summary["provenance"]["source_surveys"] = ["changed.json"]
            path.write_text(json.dumps(summary))
            return "test-course"

        with patch.object(flight, "_run", side_effect=change_summary):
            with self.assertRaisesRegex(ValueError, "summary changed during"):
                self.check()
        path.write_bytes(original)
        with redirect_stdout(io.StringIO()):
            checked = self.check()
        with patch.object(flight, "_run") as launch:
            with self.assertRaisesRegex(ValueError, "summary changed since"):
                flight.fly(checked, "radio://0/9/2M/E7E7E7E701", self.root / "race.csv",
                           self.root / "flight.txt", ready=True, confirm=change_summary)
            launch.assert_not_called()
        self.assertFalse((self.root / "race.course.json").exists())
        self.assertFalse((self.root / "race.summary.json").exists())

    def test_existing_summary_archive_is_never_overwritten(self):
        with redirect_stdout(io.StringIO()):
            checked = self.check()
        path = self.root / "race.summary.json"
        path.write_bytes(b"earlier attempt")
        with patch.object(flight, "_run") as launch:
            with self.assertRaises(FileExistsError):
                flight.fly(checked, "radio://0/9/2M/E7E7E7E701", self.root / "race.csv",
                           self.root / "flight.txt", ready=True)
            launch.assert_not_called()
        self.assertEqual(path.read_bytes(), b"earlier attempt")

    def test_course_changed_during_confirmation_is_not_archived_or_flown(self):
        with redirect_stdout(io.StringIO()):
            checked = self.check()

        def confirm(_prompt):
            self.course.write_text(self.course.read_text() + "\n")
            return "test-course"

        with patch.object(flight, "_run") as launch:
            with self.assertRaisesRegex(ValueError, "changed"):
                flight.fly(checked, "radio://0/9/2M/E7E7E7E701", self.root / "race.csv",
                           self.root / "flight.txt", ready=True, confirm=confirm)
            launch.assert_not_called()
        self.assertFalse((self.root / "race.course.json").exists())

    def test_notebook_places_power_cycle_after_identification_and_retains_uri(self):
        notebook = json.loads(Path(__file__).with_name("03_flight.ipynb").read_text())
        cells = {cell["id"]: "".join(cell["source"]) for cell in notebook["cells"]}
        ids = [cell["id"] for cell in notebook["cells"]]
        self.assertLess(ids.index("flight-connect"), ids.index("flight-launch-guide"))
        self.assertNotIn("power-cycle", cells["flight-connect-guide"])
        for phrase in ("**After Connect**", "fully off, then on", "without rerunning Connect",
                       "preflight", "separate emergency-stop operator"):
            self.assertIn(phrase, cells["flight-launch-guide"])
        self.assertNotIn("check_drone(", cells["flight-launch"])
        self.assertNotIn("uri =", cells["flight-launch"])

    def test_notebook_uses_new_flight_attempts_and_exposes_archive_even_on_abort(self):
        notebook = json.loads(Path(__file__).with_name("03_flight.ipynb").read_text())
        code = next("".join(cell["source"]) for cell in notebook["cells"]
                    if cell["id"] == "flight-launch")
        code = code.replace("ready_to_fly = False", "ready_to_fly = True", 1)
        with redirect_stdout(io.StringIO()):
            checked = self.check()
        uri = "radio://0/9/2M/E7E7E7E701"
        paths = []

        def abort(checked, selected_uri, telemetry, console, **kwargs):
            self.assertEqual(selected_uri, uri)
            paths.append(telemetry)
            raise KeyboardInterrupt

        scope = dict(output=self.root, new_attempt=flight.new_attempt, checked_run=checked,
                     uri=uri, fly=abort, tuning_spec=None)
        for _ in range(2):
            with redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
                exec(code, scope)
            self.assertFalse(scope["ready_to_fly"])
            self.assertEqual(scope["flown_course_json"], scope["telemetry"].with_suffix(".course.json"))
        self.assertNotEqual(*paths)

    def test_interrupt_requests_runner_cleanup_and_waits_for_it(self):
        self.runner.write_text('''import signal, time
from pathlib import Path
print("worker ready", flush=True)
try:
    time.sleep(20)
except KeyboardInterrupt:
    time.sleep(.1)
    Path("landed.txt").write_text("landed")
    print("mock controlled landing completed", flush=True)
''')
        original_get = flight.queue.Queue.get
        interrupted = False
        processes = []
        real_popen = flight.subprocess.Popen

        def start(*args, **kwargs):
            process = real_popen(*args, **kwargs)
            process.terminate = Mock(wraps=process.terminate)
            process.kill = Mock(wraps=process.kill)
            processes.append(process)
            return process

        def interrupt_after_ready(lines, *args, **kwargs):
            nonlocal interrupted
            if interrupted:
                return original_get(lines, *args, **kwargs)
            line = original_get(lines, *args, **kwargs)
            if line and "worker ready" in line:
                interrupted = True
                raise KeyboardInterrupt
            return line

        with patch.object(flight.queue.Queue, "get", interrupt_after_ready), \
                patch.object(flight.subprocess, "Popen", start), redirect_stdout(io.StringIO()):
            with self.assertRaises(KeyboardInterrupt):
                flight._run([sys.executable, "-u", "-c", flight._BOOTSTRAP, str(self.runner)],
                            self.root, self.root / "interrupted.txt")
        self.assertEqual((self.root / "landed.txt").read_text(), "landed")
        self.assertIn("landing completed", (self.root / "interrupted.txt").read_text())
        self.assertEqual(len(processes), 1)
        processes[0].terminate.assert_not_called()
        processes[0].kill.assert_not_called()

    def test_dry_run_deadline_covers_open_or_closed_output_and_bounds_cleanup(self):
        real_popen = flight.subprocess.Popen
        processes = []

        def start(*args, **kwargs):
            process = real_popen(*args, **kwargs)
            process.kill = Mock(wraps=process.kill)
            processes.append(process)
            return process

        for close_output in (False, True):
            with self.subTest(close_output=close_output):
                worker = "import os, signal, time\n"
                if os.name != "nt":
                    worker += "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
                worker += "print('dry-run worker ready', flush=True)\n"
                if close_output:
                    worker += "os.close(1)\nos.close(2)\n"
                worker += "time.sleep(20)\n"
                before = time.monotonic()
                with patch.object(flight.subprocess, "Popen", start), redirect_stdout(io.StringIO()):
                    with self.assertRaisesRegex(TimeoutError, "Dry run exceeded"):
                        flight._run([sys.executable, "-u", "-c", worker], self.root,
                                    self.root / f"timeout-{close_output}.txt", timeout_s=.3)
                self.assertLess(time.monotonic() - before, 3)
                self.assertIsNotNone(processes[-1].poll())
                if os.name != "nt":
                    processes[-1].kill.assert_called_once()

    def test_planning_notebook_is_radio_free_and_uses_current_attempt_paths(self):
        notebook = json.loads(Path(__file__).with_name("02_trajectory.ipynb").read_text())
        code = "\n".join("".join(cell["source"]) for cell in notebook["cells"]
                         if cell["cell_type"] == "code")
        for obsolete in ("check_drone", "LABEL", "day3-survey-command", "--fly"):
            self.assertNotIn(obsolete, code)
        cells = {cell["id"]: "".join(cell["source"]) for cell in notebook["cells"]}
        self.assertIn("attempt folder printed by the flight notebook", cells["day3-r7"])
        self.assertIn("archive path", cells["day3-recording"])

    def test_notebook_defaults_do_not_launch_tools_or_connect(self):
        path = Path(__file__).with_name("03_flight.ipynb")
        notebook = json.loads(path.read_text())
        namespace = dict(__name__="__main__")
        with patch.object(flight, "dry_run") as dry, patch.object(flight, "fly") as fly:
            with patch("notebook_radio.check_drone") as connect, redirect_stdout(io.StringIO()):
                for cell in notebook["cells"]:
                    if cell["cell_type"] == "code":
                        code = "".join(cell["source"])
                        if cell["id"] == "flight-setup":
                            code = code.replace('root / "code/day3/data" / run_id', repr(str(self.root / "attempt")))
                            code = code.replace('output.mkdir(', 'output = Path(output)\noutput.mkdir(')
                        exec(code, namespace)
                dry.assert_not_called()
                fly.assert_not_called()
                connect.assert_not_called()

    def test_versioned_course_requires_shared_trajectory_support_before_flight(self):
        course = self.root / "versioned.json"
        course.write_text(json.dumps(dict(course_name="test", enabled_for_flight=False, trajectory_version=1)))
        self.save_summary(course)
        self.runner.write_text("print('old runner')\n")
        with redirect_stdout(io.StringIO()), self.assertRaisesRegex(ValueError, "trajectory support"):
            flight.dry_run(self.runner, course, sys.executable, self.root, self.root / "old-protocol.txt")
        self.runner.write_text("print('TRAJECTORY PROTOCOL 1: shared planner')\n")
        with redirect_stdout(io.StringIO()):
            checked = flight.dry_run(self.runner, course, sys.executable, self.root, self.root / "new-protocol.txt")
        self.assertEqual(checked.course_name, "test")

    def test_native_recording_analysis_has_inline_figures_and_metrics(self):
        from race_common.flight_analysis import analyze_recording
        import matplotlib.pyplot as plt
        telemetry = self.root / "recording.csv"
        telemetry.write_text(
            "event,host_monotonic_s,x_m,y_m,z_m,command_x_m,command_y_m,command_z_m,phase\n"
            "command,1,,,,0,0,0.7,race\n"
            "position,1.1,0.01,0,0.7,,,,\n"
            "command,1.2,,,,0.1,0,0.7,race\n"
            "position,1.3,0.11,0,0.7,,,,\n")
        self.course.write_text(json.dumps(dict(course_name="test", hoops=[],
                                               start=[0, 0, .7], finish=[.1, 0, .7])))
        comparison, figures, metrics = analyze_recording(telemetry, self.course, phase="race")
        self.assertEqual(metrics["matched_samples"], 2)
        self.assertAlmostEqual(metrics["rms_error_m"], .01)
        self.assertEqual(len(figures), 2)
        for figure in figures:
            self.assertTrue(figure.axes)
            plt.close(figure)

    def test_notebooks_use_native_analysis_and_optional_simulation(self):
        directory = Path(__file__).parent
        race = json.loads((directory / "02_trajectory.ipynb").read_text())
        cells = {cell["id"]: "".join(cell["source"]) for cell in race["cells"]}
        self.assertIn("analyze_recording(", cells["day3-recording"])
        self.assertNotIn("subprocess", cells["day3-recording"])
        scope = {"__name__": "__main__"}
        optional = json.loads((directory / "04_extra_uncertainty.ipynb").read_text())
        for cell in optional["cells"]:
            if cell["cell_type"] == "code":
                with redirect_stdout(io.StringIO()):
                    exec("".join(cell["source"]), scope)
        self.assertTrue(scope["figure"].axes)


if __name__ == "__main__":
    unittest.main()
