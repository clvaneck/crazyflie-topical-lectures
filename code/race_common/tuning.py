"""Bounded, temporary onboard tuning with fresh readback and an audit trail."""

from contextlib import contextmanager
import json
import math
from pathlib import Path
import queue
import time


CONTROLLER_PARAMETERS = ("posCtlPid.xKp", "posCtlPid.yKp")
KALMAN_PARAMETERS = ("kalman.pNAcc_xy",)
MODES = {"controller": CONTROLLER_PARAMETERS, "kalman": KALMAN_PARAMETERS, "baseline": ()}
# Paired XY gains retain the baseline's axis ratio. One group changes per trial,
# so differences can be attributed to a particular loop or noise contribution.
PARAMETER_GROUPS = {
    "controller": {
        "position_p_xy": CONTROLLER_PARAMETERS,
        "position_i_xy": ("posCtlPid.xKi", "posCtlPid.yKi"),
        "position_d_xy": ("posCtlPid.xKd", "posCtlPid.yKd"),
        "velocity_p_xy": ("velCtlPid.vxKp", "velCtlPid.vyKp"),
        "velocity_i_xy": ("velCtlPid.vxKi", "velCtlPid.vyKi"),
        "velocity_d_xy": ("velCtlPid.vxKd", "velCtlPid.vyKd"),
    },
    "kalman": {
        "acceleration_xy": KALMAN_PARAMETERS,
        "acceleration_z": ("kalman.pNAcc_z",),
        "velocity": ("kalman.pNVel",),
        "position": ("kalman.pNPos",),
    },
}
DEFAULT_GROUPS = {"controller": "position_p_xy", "kalman": "acceleration_xy"}
PARAMETERS = tuple(dict.fromkeys(name for groups in PARAMETER_GROUPS.values()
                                for names in groups.values() for name in names))
# Software ceilings limit the experiment; they do not establish a safe range
# for a particular airframe. Teams also supply their agreed, narrower interval.
FACTOR_BOUNDS = {"controller": (.9, 1.1), "kalman": (.5, 2.)}


def wait_for_parameters(cf, timeout_s=5):
    deadline = time.monotonic() + timeout_s
    while not cf.param.is_updated:
        if time.monotonic() >= deadline:
            raise TimeoutError("Parameter download incomplete; no tuning applied.")
        time.sleep(.02)


def parameter_value(cf, name, *, expected=None, timeout_s=2):
    """Read a fresh value, or write and await acknowledgement of the target."""
    if cf.param.toc.get_element_by_complete_name(name) is None:
        raise ValueError(f"Firmware does not expose {name}")
    group, key = name.split(".", 1)
    values = queue.Queue()

    def receive(_name, value):
        values.put(value)

    cf.param.add_update_callback(group=group, name=key, cb=receive)
    deadline = time.monotonic() + timeout_s
    try:
        if expected is not None:
            cf.param.set_value(name, str(expected))
        cf.param.request_param_update(name)
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"No verified acknowledgement for {name}")
            try:
                value = float(values.get(timeout=remaining))
            except queue.Empty as error:
                raise TimeoutError(f"No verified acknowledgement for {name}") from error
            if not math.isfinite(value):
                raise ValueError(f"Non-finite parameter {name}")
            if expected is None or math.isclose(value, expected, rel_tol=1e-5, abs_tol=1e-8):
                return value
    finally:
        cf.param.remove_update_callback(group=group, name=key, cb=receive)


def validate_tuning(data, uri=None):
    """Accept one allowlisted family, tied to a measured per-drone baseline."""
    data = json.loads(json.dumps(data, allow_nan=False))
    required = {"version", "uri", "baseline", "mode", "factor", "factor_range", "range_checked"}
    if not required.issubset(data) or set(data) - required - {"parameter_group"}:
        raise ValueError("Unexpected or missing tuning fields")
    if data["version"] != 1 or not isinstance(data["uri"], str) or not data["uri"].startswith("radio://"):
        raise ValueError("Tuning needs a version-1 baseline and explicit radio URI")
    if uri is not None and data["uri"] != uri:
        raise ValueError("Tuning baseline belongs to a different drone URI")
    baseline = data["baseline"]
    if (not isinstance(baseline, dict) or not set(CONTROLLER_PARAMETERS).issubset(baseline)
            or not set(baseline).issubset(PARAMETERS)):
        raise ValueError("Baseline must contain the horizontal PID gains and only allowlisted parameters")
    for name, value in baseline.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError("Baseline parameters must be finite and nonnegative")
        if name in CONTROLLER_PARAMETERS and value == 0:
            raise ValueError("Core position P gains must be positive")
    mode = data["mode"]
    if mode not in MODES:
        raise ValueError("Choose baseline, controller or kalman tuning")
    factor = data["factor"]
    if isinstance(factor, bool) or not isinstance(factor, (int, float)) or not math.isfinite(factor):
        raise ValueError("Tuning factor must be finite")
    if mode == "baseline":
        if "parameter_group" in data:
            raise ValueError("Choose a tuning mode before selecting a parameter group")
        if factor != 1 or data["factor_range"] is not None or data["range_checked"] is not False:
            raise ValueError("Baseline mode uses factor 1, no range and range_checked=False")
    else:
        interval = data["factor_range"]
        if (data["range_checked"] is not True or not isinstance(interval, list) or len(interval) != 2
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in interval)):
            raise ValueError("Agree and explicitly check the two-value tuning interval first")
        low, high = interval
        floor, ceiling = FACTOR_BOUNDS[mode]
        if not floor <= low <= 1 <= high <= ceiling or not low <= factor <= high:
            raise ValueError(f"{mode} factors must stay within the agreed interval and software bounds {floor:g}-{ceiling:g}")
        group = data.get("parameter_group", DEFAULT_GROUPS[mode])
        if not isinstance(group, str) or group not in PARAMETER_GROUPS[mode]:
            raise ValueError("Choose a parameter group belonging to the selected tuning mode")
        names = PARAMETER_GROUPS[mode][group]
        if not set(names).issubset(baseline):
            raise ValueError("The captured firmware does not support this tuning family")
        if any(baseline[name] == 0 for name in names):
            raise ValueError("A zero baseline cannot be scaled; select a group with positive baseline values")
    return data


def selected_parameters(tuning):
    if tuning["mode"] == "baseline":
        return ()
    mode = tuning["mode"]
    return PARAMETER_GROUPS[mode][tuning.get("parameter_group", DEFAULT_GROUPS[mode])]


def load_tuning(path, uri=None):
    return validate_tuning(json.loads(Path(path).read_text(encoding="utf-8-sig")), uri)


def _save(path, record):
    Path(path).write_text(json.dumps(record, indent=2, allow_nan=False) + "\n", encoding="utf-8")


@contextmanager
def temporary_tuning(cf, tuning, uri, receipt_path):
    """Apply before arming; restore every attempted write before link cleanup."""
    tuning = validate_tuning(tuning, uri)
    receipt_path = Path(receipt_path)
    receipt_path.parent.mkdir(parents=True, exist_ok=True)
    with receipt_path.open("x", encoding="utf-8") as stream:
        json.dump({"status": "checking", "tuning": tuning}, stream)
    record = {"status": "checking", "tuning": tuning, "original": {}, "applied": {}, "restored": {}}
    attempted = []
    record["attempted"] = attempted
    try:
        wait_for_parameters(cf)
        for name, expected in (("stabilizer.controller", 1), ("stabilizer.estimator", 2)):
            actual = parameter_value(cf, name)
            if actual != expected:
                raise ValueError("Tuning requires active PID controller 1 and Kalman estimator 2")
        # Check every baseline value before any write, including the untuned family.
        for name, expected in tuning["baseline"].items():
            actual = parameter_value(cf, name)
            record["original"][name] = actual
            if not math.isclose(actual, expected, rel_tol=1e-5, abs_tol=1e-8):
                raise ValueError(f"{name} differs from captured baseline; restore it before another attempt")
        record["status"] = "applying"
        _save(receipt_path, record)
        for name in selected_parameters(tuning):
            attempted.append(name)  # Restore even if the write's acknowledgement fails.
            target = record["original"][name] * tuning["factor"]
            record["applied"][name] = parameter_value(cf, name, expected=target)
        record["status"] = "active"
        _save(receipt_path, record)
        yield record
    except BaseException as error:
        record["error"] = str(error) or type(error).__name__
        raise
    finally:
        failures = {}
        for name in reversed(attempted):
            try:
                record["restored"][name] = parameter_value(cf, name, expected=record["original"][name])
            except BaseException as error:
                failures[name] = str(error) or type(error).__name__
        record["status"] = "restore_failed" if failures else ("restored" if attempted else "unchanged")
        record["restore_errors"] = failures
        _save(receipt_path, record)
        if failures:
            raise RuntimeError("Baseline restoration was not verified; keep the drone grounded. "
                               f"Restore the recorded values or power-cycle and verify them; see {receipt_path}")
