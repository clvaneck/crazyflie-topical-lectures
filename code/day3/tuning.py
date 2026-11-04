"""Notebook baseline capture and tuning-file construction; no flight commands."""

import argparse
import json
import os
from pathlib import Path
import sys

from notebook_radio import motor_stopped_connection, run_worker
from race_common.tuning import (
    PARAMETERS, PARAMETER_GROUPS, parameter_value, selected_parameters, validate_tuning,
)
from .notebook_tools import validate_plan_summary


def export_race_pair(course_path, optimized_tuning, directory, *, summary=None):
    """Package one course with baseline and tuned profiles for the common runner.

    A file input needs its matching summary.json alongside it; an in-memory
    course needs an explicit summary. Geometry validation remains with the
    runner's dry run. Exporting preserves the flight-enable flag and opens no radio.
    """
    optimized = validate_tuning(optimized_tuning)
    if optimized["mode"] == "baseline" or optimized["factor"] == 1:
        raise ValueError("Choose a bounded parameter change for the optimized submission")
    baseline = dict(optimized, mode="baseline", factor=1., factor_range=None, range_checked=False)
    baseline.pop("parameter_group", None)
    baseline = validate_tuning(baseline)
    if isinstance(course_path, dict):
        course_bytes = (json.dumps(course_path, indent=2, allow_nan=False) + "\n").encode("utf-8")
        if summary is None:
            raise ValueError("An in-memory course needs its validated provenance summary")
        summary_bytes = (json.dumps(summary, indent=2, allow_nan=False) + "\n").encode("utf-8")
    else:
        if summary is not None:
            raise ValueError("A course file uses its companion summary.json")
        path = Path(course_path)
        course_bytes = path.read_bytes()
        try:
            summary_bytes = path.with_name("summary.json").read_bytes()
        except FileNotFoundError as error:
            raise ValueError("Keep course.json and summary.json together; re-export the selected plan if its summary is missing") from error
    course = json.loads(course_bytes.decode("utf-8-sig"))
    if not isinstance(course, dict) or not isinstance(course.get("course_name"), str):
        raise ValueError("Choose the exported trajectory course JSON")
    # The planner exports flight disabled; enabling it before qualification is
    # the sole course edit that may retain the original planning evidence.
    planned_course = dict(course, enabled_for_flight=False)
    validate_plan_summary(planned_course, json.loads(summary_bytes.decode("utf-8-sig")))
    # Reuse identical course bytes so only the declared parameter change varies.
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=False)
    submissions = {}
    for name, tuning in (("baseline", baseline), ("optimized", optimized)):
        folder = directory / name
        folder.mkdir()
        course_target = folder / "course.json"
        course_target.write_bytes(course_bytes)
        (folder / "summary.json").write_bytes(summary_bytes)
        tuning_target = folder / "tuning.json"
        tuning_target.write_text(json.dumps(tuning, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        submissions[name] = dict(course=course_target, tuning=tuning_target)
    return submissions


def race_tuning(baseline, *, controller=None, kalman=None, factor_range=None, range_checked=False):
    """Translate one named notebook choice to the runner's bounded profile."""
    choices = [(mode, values) for mode, values in (("controller", controller), ("kalman", kalman))
               if values is not None]
    if len(choices) != 1:
        raise ValueError("Choose one controller or Kalman parameter group per comparison")
    mode, values = choices[0]
    if not isinstance(values, dict) or len(values) != 1:
        raise ValueError("Specify one parameter group and its multiplicative factor")
    group, factor = next(iter(values.items()))
    if group not in PARAMETER_GROUPS[mode]:
        raise ValueError(f"Unknown {mode} group: {group}; choose from {', '.join(PARAMETER_GROUPS[mode])}")
    tuning = make_tuning(baseline, mode=mode, factor=factor, factor_range=factor_range,
                         range_checked=range_checked, parameter_group=group)
    if factor == 1:
        raise ValueError("Choose a factor different from 1 for the optimized submission")
    return tuning


def tuning_changes(tuning):
    """Show firmware names, original values and requested values before export."""
    tuning = validate_tuning(tuning)
    return [dict(parameter=name, baseline=tuning["baseline"][name],
                 optimized=tuning["baseline"][name] * tuning["factor"], factor=tuning["factor"])
            for name in selected_parameters(tuning)]


def capture_baseline(uri, directory):
    """Capture on the grounded, stopped drone; leave firmware stop engaged."""
    if not isinstance(uri, str) or not uri.startswith("radio://"):
        raise ValueError("Identify the grounded drone first")
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    if (directory / "baseline.json").exists():
        raise FileExistsError("Choose a new baseline capture folder")
    result = run_worker([sys.executable, "-u", "-m", "day3.tuning", "--uri", uri,
                         "--output", str(directory / "baseline.json")],
                        cwd=Path(__file__).resolve().parents[1], env=dict(os.environ), timeout_s=30)
    if result.returncode:
        raise RuntimeError("Baseline capture failed; use the checked fallback.\n" + result.stdout)
    validate_tuning(make_tuning(directory / "baseline.json"), uri)
    return directory / "baseline.json"


def make_tuning(baseline_path, *, mode="baseline", factor=1., factor_range=None, range_checked=False,
                parameter_group=None):
    baseline = json.loads(Path(baseline_path).read_text(encoding="utf-8-sig"))
    if baseline.get("motor_stop_check", {}).get("value") != 1:
        raise ValueError("Use a baseline captured with verified firmware stop")
    tuning = dict(version=1, uri=baseline["uri"], baseline=baseline["parameters"],
                  mode=mode, factor=factor, factor_range=factor_range, range_checked=range_checked)
    if parameter_group is not None:
        tuning["parameter_group"] = parameter_group
    return validate_tuning(tuning)


def _capture(uri, path):
    with motor_stopped_connection(uri, path.parent / ".cflib-cache") as (connection, stop_check):
        cf = connection.cf
        for name, expected in (("stabilizer.controller", 1), ("stabilizer.estimator", 2)):
            if parameter_value(cf, name) != expected:
                raise ValueError("Prepare and check PID controller 1 / Kalman estimator 2 before capturing a baseline")
        parameters = {name: parameter_value(cf, name) for name in PARAMETERS
                      if cf.param.toc.get_element_by_complete_name(name) is not None}
        # Validate required gains and values before marking the capture complete.
        validate_tuning(dict(version=1, uri=uri, baseline=parameters, mode="baseline",
                             factor=1., factor_range=None, range_checked=False))
        record = dict(uri=uri, parameters=parameters, motor_stop_check=stop_check)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(record, stream, indent=2)
    print("Baseline captured. Firmware stop remains engaged; power-cycle before flight preflight.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uri", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    _capture(args.uri, args.output)
