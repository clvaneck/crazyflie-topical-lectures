# Race 1: from organisers' JSON to launch

Goal: pass every hoop in time. All steps below are offline until step 6. Run from the repo root.
`PY` = the course venv python (`.../crazyflie-topical-lectures/.venv/bin/python`).

## Pipeline in one line
organisers' JSON -> `hoop_view.py` (load, table, picture, `--to-course` into our template)
-> plan (`plan_race` conservative, or `scripts/plan_trajectory.py`) -> `plans/race1/{course,summary}.json,
trajectory.csv, preview.png` -> `03_flight.ipynb` dry run with the run-card runner -> enable -> fly.
Speeds come only from the course `limits` (target/maximum speed, acceleration, tilt) and clearance from
`drone_radius_m + clearance_margin_m` (+ `frame_margin_m` 0.10 per gate). The planner does NOT use the
0.8 m/s `RaceLimits.nominal_speed_m_s` (only notebook 04) nor our E7 rule.

## Commands
1. Look at the file: `$PY code/day3/hoop_view.py race1.json --png race1.png`
   If it fails, the message names the missing field and the keys present; add an alias in
   `_hoop()` / `load()` (docstring lists accepted fields) or hand-edit the JSON. Rim/edge points
   are fitted to a circle (normal sign is then a guess: check the arrow).
2. Make our template `race1-template.json` (copy `examples/course.synthetic.json`), edit:
   `course_name`, `start`/`finish` (airborne, 0.4+ m before hoop 1 / after the last),
   `limits.bounds_min/max` = the real ~7 m arena minus nets/walls, `drone_radius_m` (cage), `yaw_deg: 0`.
3. Merge: `$PY code/day3/hoop_view.py race1.json --to-course race1-template.json race1-course.json`
4. Plan (conservative profile: stops at each approach, max 0.3 m/s, trajectory_version 1 so the
   runner flies exactly this path):
   ```
   cd code && $PY -c "import sys; from day3.notebook_tools import plan_race, save_plan; from trajectory_tools.course import read_json
   p = plan_race(read_json(sys.argv[1]), provenance={'source_course': sys.argv[1]}); save_plan(p, sys.argv[2]); p.plot().savefig(sys.argv[2] + '/preview.png', dpi=120)
   print(round(p.summary()['planned_duration_s'], 1), 's')" ../race1-course.json ../plans/race1 && cd ..
   ```
   Fallback (no trajectory_version, runner may build its own path):
   `$PY scripts/plan_trajectory.py --course race1-course.json --output plans/race1-cli`
5. View plan: `$PY code/day3/hoop_view.py plans/race1/course.json --plan plans/race1 --png plans/race1/hoops.png`
   and open `plans/race1/preview.png` (speed/acceleration panels).
6. Export: `03_flight.ipynb` "Export the two race submissions" (`planned_course = plans/race1/course.json`,
   `export_submissions=True`) writes `baseline/` and `optimized/` (course, tuning, summary, trajectory).
7. Submission (release 0.1.12 layout; team repo is still 0.1.11, only the course clone's 03_flight changed):
   `submissions/team-<X>/v01/README.md` (team name, LABEL 4, short description), `v01/baseline/` and
   `v01/optimized/`, each with at least `course.json, tuning.json, summary.json`. Copy the WHOLE exported
   profile folders (keeps the sampled trajectory and matched files together). Flight stays disabled.
   Transfer `v01/` to the flight laptop via the shared folder or USB. Updates go in `v02/`; never change
   a submitted version. Declare versions before the freeze deadline.
8. Local flight, `03_flight.ipynb`: copy the profile folder to a separate working folder; set
   `enabled_for_flight: true` in that copy only. Setup -> `LABEL` -> run-card runner paths ->
   `submission_to_check = <copy>`, `run_dry_check=True` (must print TRAJECTORY PROTOCOL 1) ->
   power-cycle, preflight -> fly (type course name). Ask Niels before each real flight.

## Check by eye (before step 4 and again after step 5)
- Hoop count and order match the organisers' numbering; numbers in the picture follow the race order.
- Every orange arrow points in the direction we fly through it; `n.dir_next` in the table is
  positive unless the course turns back. A negative value = likely flipped normal.
- Centres/radii plausible (metres, not mm; hoop heights 0.5-1.5 m; radius matches the real hoop).
- All hoops, start, finish and the blue path inside the red bounds, clear of walls/nets by >= 0.3 m.
- Path passes through the middle of each hoop, no swing near another hoop's frame.
- Lighthouse frame origin/axes = the one the organisers used (ask!). Drone faces +x at takeoff.

## Open risks
- Lag: position-loop lag 0.50 s. Along-track error = lag * v: 0.12 m at 0.24 m/s (fake 7 m plan peak),
  0.25 m at 0.5 m/s, close to the runner abort `maximum_tracking_error_m` 0.3. Keep max speed <= 0.3.
- Our E7 rule says 0.22 m/s (range) / 0.13 m/s (range gap). Conservative plan: cruise 0.20, peak 0.30,
  crossings ~0.16 m/s: consistent. On a curve the cross-track error grows; we cross only at stops
  in the conservative profile, so it should be fine.
- Time: a fake 3-hoop 6 m course plans to ~41 s + 6 s takeoff/landing. Unknown race time limit:
  if too slow, plan with `plan_race(..., conservative=False, speed=0.25)` and re-check.
- Template bounds (-0.5..2.5 m) are the synthetic example; planning fails with "control hull leaves
  bounds" until step 2 sets the real arena.
- `clearance_margin_m` 0.08 (+0.10 conservative) + drone 0.12: hoop radius must exceed ~0.22 m + tracking 5 cm.
- Runner is NOT in this repo (run card); our plan is only checked by its dry run.
- Yaw: flow-deck drift was +/-17 deg; with Lighthouse yaw is absolute, so align the drone with +x.
