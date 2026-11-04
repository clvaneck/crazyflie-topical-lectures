#!/usr/bin/env python3
"""Construct and preview an offline course; never connects to a drone."""

import argparse
from pathlib import Path
from trajectory_tools.course import construct_course, read_json
from trajectory_tools.preview import plot_trajectory
from .notebook_tools import checked_plan, plan_course, save_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--course", type=Path, required=True, help="Course or template JSON with endpoints and agreed limits")
    parser.add_argument("--gate-survey", action="append", type=Path, default=[], help="Gate survey JSON from the notebook or CLI; repeat for multiple gates")
    parser.add_argument("--crossing", action="append", default=[], help="Survey name followed by :+ or :-; repeat in crossing order")
    parser.add_argument("--laps", type=int, default=1)
    parser.add_argument("--frame-margin", type=float, help="Physical frame/support padding beyond each opening, metres per side; overrides every gate")
    parser.add_argument("--name", help="Name for the constructed course")
    parser.add_argument("--output", type=Path, required=True, help="New output directory; existing directories are never overwritten")
    parser.add_argument("--show", action="store_true", help="Open the interactive Matplotlib preview as well as saving PNG")
    args = parser.parse_args()
    try:
        if args.output.exists():
            raise ValueError("output already exists; choose a new directory")
        course = construct_course(read_json(args.course), [read_json(path) for path in args.gate_survey],
                                  args.crossing, args.laps, frame_margin_m=args.frame_margin)
        if args.name:
            course["course_name"] = args.name
        plan = plan_course(course, provenance={
            "source_course": str(args.course),
            "source_surveys": [str(path) for path in args.gate_survey],
        })
        course, trajectory = checked_plan(plan)
        figure = plot_trajectory(course, trajectory)
        args.output.mkdir(parents=True)
        save_plan(plan, args.output)
        figure.savefig(args.output / "preview.png", dpi=160)
        print(f"Planned duration {trajectory.duration_s:.2f} s (excludes takeoff/landing); {len(course['hoops'])} crossings")
        print(f"Wrote disabled course, trajectory CSV, summary and preview to {args.output}")
        if course.get("trajectory_version") == 1:
            print("Shared trajectory version 1: verify support in the common runner's dry run before flight.")
        else:
            print("Preview only: the qualified runner may construct a different path. Check its own dry run before flight.")
        import matplotlib.pyplot as plt
        if args.show:
            plt.show()
        plt.close(figure)
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f"Planning failed: {error}\n")


if __name__ == "__main__":
    main()
