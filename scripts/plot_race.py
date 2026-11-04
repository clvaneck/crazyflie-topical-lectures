#!/usr/bin/env python3
"""Compare recorded race setpoints and onboard position estimates offline."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))



from race_common.flight_analysis import (
    RaceLog, Comparison, load_telemetry, select_phase, compare,
    gate_outline, break_gaps, plot_comparison,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("telemetry", type=Path, help="Race telemetry CSV")
    parser.add_argument("--course", type=Path, required=True, help="Course JSON used for the flight")
    parser.add_argument("--phase", default="auto", help="Command phase, all, or auto (default: race, then trajectory)")
    parser.add_argument("--max-gap", type=float, default=0.25, help="Maximum command age/line gap in seconds")
    parser.add_argument("--gate-survey", type=Path, action="append", default=[], help="Optional gate survey JSON; repeat for multiple gates")
    parser.add_argument("--output", type=Path, help="Save PREFIX-trajectory.png and PREFIX-errors.png")
    parser.add_argument("--no-show", action="store_true", help="Save without opening plot windows (requires --output)")
    args = parser.parse_args()
    if args.no_show and args.output is None:
        parser.error("--no-show requires --output")
    try:
        if args.no_show:
            import matplotlib
            matplotlib.use("Agg")
        # Accept JSON saved by Windows editors with or without a UTF-8 BOM.
        course = json.loads(args.course.read_text(encoding="utf-8-sig"))
        surveys = [json.loads(path.read_text(encoding="utf-8-sig")) for path in args.gate_survey]
        log = load_telemetry(args.telemetry)
        phase = select_phase(log, args.phase)
        comparison = compare(log, phase, args.max_gap)
        valid = np.all(np.isfinite(comparison.errors), axis=1)
        if not np.any(valid):
            raise ValueError("No positions have a sufficiently recent command; check timestamps or max-gap")
        planned = None
        figures = plot_comparison(comparison, course, surveys, planned, args.max_gap)
        distances = np.linalg.norm(comparison.errors[valid], axis=1)
        print(f"Compared {valid.sum()}/{len(valid)} position samples in phase {phase}")
        print(f"3D RMS error: {np.sqrt(np.mean(distances**2)):.3f} m; maximum: {distances.max():.3f} m")
        print("Host receive/send alignment includes transport delay; onboard position is not ground truth.")
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            for figure, suffix in zip(figures, ("trajectory", "errors")):
                path = args.output.parent / f"{args.output.name}-{suffix}.png"
                figure.savefig(path, dpi=150)
                print(f"Saved {path}")
        import matplotlib.pyplot as plt
        if not args.no_show:
            plt.show()
        for figure in figures:
            plt.close(figure)
        return 0
    except (OSError, ValueError, KeyError) as error:
        print(f"Race plot failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
