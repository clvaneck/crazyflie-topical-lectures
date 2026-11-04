"""Notebook front end for a separately supplied, qualified flight runner."""

from dataclasses import dataclass
from hashlib import sha256
from pathlib import Path
import os
import json
import math
from itertools import count
import queue
import signal
import subprocess
import threading
import time

from race_common.tuning import validate_tuning
from .notebook_tools import validate_plan_summary


@dataclass(frozen=True)
class CheckedRun:
    runner: Path
    course: Path
    python: Path
    directory: Path
    digests: tuple[str, ...]
    course_name: str
    summary_bytes: bytes
    tuning_json: str | None = None


def _digest(path):
    return sha256(path.read_bytes()).hexdigest()


def _summary_snapshot(course):
    """Validate planning evidence, permitting only the flight-enable flag change."""
    path = course.with_name("summary.json")
    if not path.is_file():
        raise ValueError("Keep course.json and summary.json together; re-export the selected plan if its summary is missing")
    content = path.read_bytes()
    summary = json.loads(content.decode("utf-8-sig"))
    planned_course = json.loads(course.read_text(encoding="utf-8-sig"))
    planned_course["enabled_for_flight"] = False
    validate_plan_summary(planned_course, summary)
    return content


def new_attempt(directory):
    """Create a numbered folder without replacing any earlier check or flight."""
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    for number in count(1):
        attempt = directory / f"attempt-{number:03d}"
        try:
            attempt.mkdir()
        except FileExistsError:
            continue
        return attempt


def _files(runner, course, python, directory):
    # A venv's python can be a symlink; executing the resolved system binary
    # would discard the selected environment and its installed libraries.
    paths = (Path(runner).expanduser().resolve(), Path(course).expanduser().resolve(),
             Path(python).expanduser().absolute())
    for path in paths:
        if not path.is_file():
            raise ValueError(f"File does not exist: {path}")
    directory = Path(directory).expanduser().resolve()
    if not directory.is_dir():
        raise ValueError(f"Runner directory does not exist: {directory}")
    return *paths, directory


def _run(arguments, directory, journal, confirmation=None, timeout_s=None):
    """Relay output, bounding dry runs and preserving flight abort landing."""
    journal = Path(journal)
    journal.parent.mkdir(parents=True, exist_ok=True)
    with journal.open("x", encoding="utf-8") as log:
        options = {}
        if os.name == "nt":
            options["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP
        else:
            options["start_new_session"] = True
        process = subprocess.Popen(
            arguments, cwd=directory, stdin=subprocess.PIPE,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, **options,
        )
        lines = queue.Queue()

        def read_output():
            try:
                for line in process.stdout:
                    lines.put(line)
            finally:
                process.stdout.close()
                lines.put(None)

        reader = threading.Thread(target=read_output, daemon=True)
        reader.start()
        interrupted = False
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        try:
            if confirmation is not None:
                process.stdin.write(confirmation + "\n")
                process.stdin.flush()
            process.stdin.close()
            finished = False
            # EOF and process exit are separate events: a worker can close its
            # output and keep running, so the deadline covers both.
            while not finished or process.poll() is None:
                try:
                    if deadline is not None and time.monotonic() >= deadline:
                        raise TimeoutError(f"Dry run exceeded {timeout_s:g} seconds; see {journal}")
                    line = lines.get(timeout=0.1)
                    if line is None:
                        finished = True
                    else:
                        print(line, end="", flush=True)
                        log.write(line)
                        log.flush()
                except queue.Empty:
                    continue
                except KeyboardInterrupt:
                    if deadline is not None:
                        raise
                    if not interrupted:
                        interrupted = True
                        print("Stop requested; waiting for the runner's controlled landing.")
                        if process.poll() is None:
                            process.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
            status = process.wait()
        finally:
            if deadline is not None:
                # Radio-free checks can be stopped forcibly after their deadline.
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=0.5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=0.5)
                reader.join(timeout=0.5)
            else:
                # Flight cleanup belongs to the runner's controlled-landing path.
                while process.poll() is None:
                    try:
                        if not interrupted:
                            interrupted = True
                            process.send_signal(signal.CTRL_BREAK_EVENT if os.name == "nt" else signal.SIGINT)
                        process.wait(timeout=0.2)
                    except (KeyboardInterrupt, subprocess.TimeoutExpired):
                        continue
                reader.join()
            if not process.stdin.closed:
                process.stdin.close()
    if interrupted:
        raise KeyboardInterrupt("Runner stopped; inspect its landing report and telemetry.")
    if status:
        raise RuntimeError(f"Flight tool exited with status {status}; see {journal}")


# The child uses its own interpreter and imports, keeping qualification independent
# of modules already imported into the notebook kernel.
_BOOTSTRAP = """
import os, runpy, signal, sys
from pathlib import Path
runner = Path(sys.argv[1]).resolve()
sys.argv = sys.argv[1:]
sys.path.insert(0, str(runner.parent))
if hasattr(signal, 'SIGBREAK'):
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGBREAK, interrupt)
runpy.run_path(str(runner), run_name='__main__')
"""


def dry_run(runner, course, python, directory, journal, *, tuning=None):
    """Validate the exact course with the flight tool, without opening a radio."""
    runner, course, python, directory = _files(runner, course, python, directory)
    before = tuple(_digest(path) for path in (runner, course, python))
    summary_bytes = _summary_snapshot(course)
    name = json.loads(course.read_text(encoding="utf-8-sig"))["course_name"]
    tuning_json = None if tuning is None else json.dumps(validate_tuning(tuning), sort_keys=True)
    arguments = [str(python), "-u", "-c", _BOOTSTRAP, str(runner), "--config", str(course), "--dry-run"]
    if tuning_json is not None:
        path = Path(journal).with_suffix(".tuning.json")
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("x", encoding="utf-8") as stream:
            stream.write(tuning_json)
        arguments.extend(["--tuning", str(path)])
    _run(arguments, directory, journal, timeout_s=60)
    if tuning_json is not None and "TUNING PROTOCOL 1:" not in Path(journal).read_text(encoding="utf-8"):
        raise ValueError("Flight tool did not confirm tuning support; use an updated tested runner or untuned fallback")
    course_data = json.loads(course.read_text(encoding="utf-8-sig"))
    if "trajectory_version" in course_data:
        version = course_data["trajectory_version"]
        if (type(version) is not int or version != 1
                or "TRAJECTORY PROTOCOL 1:" not in Path(journal).read_text(encoding="utf-8")):
            raise ValueError("Flight tool did not confirm trajectory support; use the updated common runner")
    if before != tuple(_digest(path) for path in (runner, course, python)):
        raise ValueError("Runner, course or interpreter changed during the dry run.")
    if summary_bytes != _summary_snapshot(course):
        raise ValueError("Plan summary changed during the dry run; repeat the dry run.")
    return CheckedRun(runner, course, python, directory, before, name, summary_bytes, tuning_json)


def fly(checked, uri, telemetry, journal, *, ready=False, confirm=input, tuning=None):
    """Archive and fly the checked course; retain the snapshot even on abort."""
    if not ready:
        raise ValueError("Complete the flight checklist and explicitly set ready=True.")
    if not isinstance(checked, CheckedRun):
        raise ValueError("Run the flight tool's dry run successfully first.")
    if not isinstance(uri, str) or not uri.startswith("radio://"):
        raise ValueError("Connect and confirm the selected drone first.")
    tuning_json = None if tuning is None else json.dumps(validate_tuning(tuning, uri), sort_keys=True)
    if tuning_json != checked.tuning_json:
        raise ValueError("Tuning changed since the dry run; repeat the dry run")
    telemetry = Path(telemetry).resolve()
    journal = Path(journal).resolve()
    archived_course = telemetry.with_suffix(".course.json")
    archived_summary = telemetry.with_suffix(".summary.json")
    tuning_path = telemetry.with_suffix(".tuning.json")
    receipt_path = telemetry.with_suffix(".parameters.json")
    for path in (telemetry, journal, archived_course, archived_summary):
        if path.exists():
            raise FileExistsError(f"Choose a new attempt; file already exists: {path}")
    if tuning_json is not None and (tuning_path.exists() or receipt_path.exists()):
        raise FileExistsError("Tuning evidence already exists; choose a new attempt")
    course_bytes = checked.course.read_bytes()
    if not json.loads(course_bytes.decode("utf-8-sig")).get("enabled_for_flight", False):
        raise ValueError("The checked course must have enabled_for_flight=true; edit it and repeat the dry run.")
    answer = confirm(f"Type the course name '{checked.course_name}' to arm flight: ")
    if answer != checked.course_name:
        raise ValueError("Confirmation did not match; no connection opened.")
    if checked.digests != tuple(_digest(path) for path in (checked.runner, checked.course, checked.python)):
        raise ValueError("Runner, course or interpreter changed; repeat the dry run.")
    if sha256(course_bytes).hexdigest() != checked.digests[1]:
        raise ValueError("Course changed while preparing the attempt; repeat the dry run.")
    if checked.summary_bytes != _summary_snapshot(checked.course):
        raise ValueError("Plan summary changed since the dry run; repeat the dry run.")
    summary = json.loads(checked.summary_bytes.decode("utf-8-sig"))
    # Keep the planned-course binding and add exact attempt-file hashes. The
    # enabled flight copy differs from the planner export only in that flag.
    summary["flown_course_sha256"] = sha256(course_bytes).hexdigest()
    summary["tuning_sha256"] = (None if tuning_json is None
                                else sha256(tuning_json.encode("utf-8")).hexdigest())
    telemetry.parent.mkdir(parents=True, exist_ok=True)
    with archived_summary.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, allow_nan=False)
        stream.write("\n")
    with archived_course.open("xb") as stream:
        stream.write(course_bytes)
    print("Exact flown course:", archived_course, flush=True)
    print("Planning provenance:", archived_summary, flush=True)
    print("Recording:", telemetry, flush=True)
    arguments = [str(checked.python), "-u", "-c", _BOOTSTRAP, str(checked.runner),
                 "--config", str(archived_course), "--fly", "--uri", uri, "--log", str(telemetry)]
    if tuning_json is not None:
        with tuning_path.open("x", encoding="utf-8") as stream:
            stream.write(tuning_json)
        arguments.extend(["--tuning", str(tuning_path)])
        print("Parameter audit and restoration:", receipt_path, flush=True)
    try:
        _run(arguments, checked.directory, journal, answer)
    finally:
        if tuning_json is not None:
            _verify_restoration(receipt_path, json.loads(tuning_json))
    return archived_course


def _verify_restoration(path, tuning):
    try:
        receipt = json.loads(path.read_text(encoding="utf-8-sig"))
        if receipt["tuning"] != tuning or receipt["status"] not in ("restored", "unchanged"):
            raise ValueError("Restoration did not complete")
        for name in receipt["attempted"]:
            original = tuning["baseline"][name]
            restored = float(receipt["restored"][name])
            if not math.isfinite(restored) or abs(restored - original) > max(1e-8, abs(original) * 1e-5):
                raise ValueError("Restored value does not match baseline")
    except (OSError, KeyError, TypeError, ValueError) as error:
        raise RuntimeError(f"Baseline restoration unverified; keep the drone grounded and inspect {path}") from error
