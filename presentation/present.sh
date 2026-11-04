#!/usr/bin/env bash
# Present the talk full screen. Right arrow = next, left = back, q = quit.
cd "$(dirname "$0")" && exec .venv/bin/manim-slides present --full-screen Title Reaper GamePlan TB2 Progress1 Bomber Progress2 End "$@"
