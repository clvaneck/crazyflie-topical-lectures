"""E7: project estimator uncertainty into gate clearance and speed decisions."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from kalman.simulation import SCENARIOS, run as run_filter
from race_common.hoop import Hoop
from race_common.strategy import RaceLimits, recommended_speed
from flight_control.trajectory import reference_state


def make_hoops() -> list[Hoop]:
    hoops = []
    for crossing_time in (5.0, 9.0, 14.0):
        reference = reference_state(crossing_time)
        hoops.append(Hoop(reference.position, reference.velocity, radius_m=0.45))
    return hoops


def run(seed: int = 7, scenario: str = "combined", *, speed_policy=None) -> dict[str, np.ndarray]:
    """Simulate estimation and evaluate a speed policy without altering motion.

    Optional speed_policy(clearance_m, nominal_speed_m_s) returns a finite speed
    in [0, maximum_speed_m_s], with zero required for non-positive clearance.
    """
    records = run_filter(seed, scenario)
    hoops = make_hoops()
    limits = RaceLimits()
    speeds, clearances = [], []
    for position, covariance in zip(records["estimate"], records["position_covariance"]):
        hoop = min(hoops, key=lambda candidate: abs(candidate.signed_plane_distance(position)))
        speed, clearance = recommended_speed(hoop, position, covariance, limits)
        if speed_policy is not None:
            speed = float(speed_policy(float(clearance), limits.nominal_speed_m_s))
            if (not np.isfinite(speed) or speed < 0 or speed > limits.maximum_speed_m_s
                    or (clearance <= 0 and speed != 0)):
                raise ValueError("Speed policy must return a finite speed within limits and stop at non-positive clearance.")
        speeds.append(speed)
        clearances.append(clearance)
    return dict(records, speed=np.asarray(speeds), clearance=np.asarray(clearances))


def plot(records: dict[str, np.ndarray], scenario: str = "combined",
         output: Path | None = None, show: bool = True):
    time_s = records["time"]
    truth_position = records["truth"]
    estimate = records["estimate"]
    error = estimate - truth_position
    position_std = records["position_std"]

    figure, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes[0, 0].plot(truth_position[:, 0], truth_position[:, 1], label="truth")
    axes[0, 0].plot(estimate[:, 0], estimate[:, 1], label="estimate")
    axes[0, 0].set(xlabel="x [m]", ylabel="y [m]", title="Trajectory")
    axes[0, 0].legend()

    for axis, label in enumerate("xyz"):
        axes[0, 1].plot(time_s, error[:, axis], label=f"{label} error")
        axes[0, 1].plot(time_s, 2.0 * position_std[:, axis], linestyle="--", alpha=0.6)
        axes[0, 1].plot(time_s, -2.0 * position_std[:, axis], linestyle="--", alpha=0.6)
    if scenario in ("dropout", "combined"):
        axes[0, 1].axvspan(7.0, 10.0, color="0.9", label="position dropout")
    if scenario in ("outlier", "combined"):
        axes[0, 1].axvline(12.0, color="red", linestyle=":", label="position outlier")
    axes[0, 1].set(xlabel="time [s]", ylabel="position error [m]", title="Error and +/-2 sigma")
    axes[0, 1].legend(ncols=2, fontsize="small")

    axes[1, 0].plot(time_s, records["chi2"])
    axes[1, 0].set(xlabel="time [s]", ylabel="chi-square", title="Maximum chi-square across sensors")
    axes[1, 0].set_yscale("symlog", linthresh=1.0)

    axes[1, 1].plot(time_s, records["speed"], label="recommended speed")
    axes[1, 1].plot(time_s, records["clearance"], label="predicted clearance")
    axes[1, 1].axhline(0.0, color="black", linewidth=0.8)
    axes[1, 1].set(xlabel="time [s]", title="Current speed policy")
    axes[1, 1].legend()

    figure.suptitle(f"Gate uncertainty: {scenario}")
    figure.tight_layout()
    if output is not None:
        output.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(output)
    if show:
        plt.show()
    plt.close(figure)
    return figure


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=SCENARIOS, default="combined")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--output", type=Path, help="save the comparison figure")
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args(argv)
    if args.seed < 0:
        parser.error("seed must be nonnegative")
    plot(run(args.seed, args.scenario), args.scenario, args.output, not args.no_show)


if __name__ == "__main__":
    main()
