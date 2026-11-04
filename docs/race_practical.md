# Race flight checklist and extensions

The [trajectory notebook](../code/day3/02_trajectory.ipynb) contains the working instructions
for planning and analysis, with gate acquisition in the
[survey notebook](../code/day3/01_survey.ipynb) and dry runs/flight in the
[flight notebook](../code/day3/03_flight.ipynb).
Use this checklist during physical operations.
The [programme](day_3_race.md#schedule) gives flight slots and deadlines.
The [glossary](glossary.md) explains unfamiliar terms.

## R1: agree rules, limits and roles

Agree gate order/directions, laps, start/finish, timing, penalties and physical
crossing rules. Record the shared flight volume, speed, acceleration, bank and
clearance limits. Name a pilot, separate emergency-stop operator, telemetry
monitor and recorder. Everyone can call a stop; agree readiness within the group.

## R2: measure and check the gates

Follow the notebook's survey instructions and the
[survey reference](hoop_survey_challenge.md) for fitting and offset corrections.
Wait for **STOP VERIFIED** before handling. Independently check centre, normal,
opening dimensions, frame and full drone clearance, including supports.
A moved gate or changed Lighthouse frame needs revalidation.
No survey report or new fitting code is required.

## R5: qualify and freeze the race configuration

Before the low-speed check and each serious attempt, cross-check within the group:

- Assigned URI, firmware/decks, stable estimation, frame/units, timing,
  telemetry, command interface and tested emergency-stop path.
- [Flight-ready mass and controller assumptions](practical_workflow.md#hardware-preflight-mass-and-controller-assumptions),
  battery, cage/decks/payload, parameter settings and readback.
- Checked survey, crossing order/directions, full planned path, physical
  gate/support/floor clearance, Lighthouse visibility and agreed limits.
- Exact code, course/settings, active profile, operators, stop conditions
  and any unresolved validation issue.
- Captured tuning baseline for this URI, agreed factor interval and fresh readbacks;
  PID controller 1 / Kalman estimator 2 for the supplied tuning workflow.

After identification in the flight notebook, power-cycle the grounded drone to
clear firmware stop, then repeat preflight. Keep the discovered URI without
rerunning Connect. Repeat preflight after further handling or rebooting.
Keep onboard attitude stabilization active.
Make one short low-speed test in the Lighthouse area and check tracking,
crossings and landing before racing. Do not tune unstable gains in flight.
For the core comparison, change only the paired horizontal position P gains,
with course and speed fixed. The changed profile's first flight is a low-speed
test; stop on oscillation or poor tracking. Kalman process-noise tuning is optional.

Record the check/cross-check and go/no-go decision. A material change in software,
geometry, frame, limits, battery/decks/cage/payload needs renewed checks.
A failed safety check means a checked fallback or no flight; the same physical
checks apply to supplied configurations. Keep a tested route back to baseline.

## Freeze points and fallback

Use the deadlines in the [Day 3 schedule](day_3_race.md#schedule):
estimator development stops 90 minutes before racing, controller development
60 minutes before, and all algorithm changes 30 minutes before.
If the race start changes, agree revised deadlines at the briefing.
Record settings before lunch if a deadline falls during lunch.
Declare and dry-run profiles before these deadlines. Qualify and compare them
in allocated flight slots afterward. Select only declared settings.

Label shared fallback geometry/telemetry and retain failed attempts and their
causes. Recheck before restarting after an abort. Stop racing in time for the
scheduled analysis block.

## Attempt record

Apply the [race battery limits](day_1_baseline_checks.md#battery-checks-for-every-flight)
before each flight. The flight notebook creates a separate folder for each
attempt, with its archived course, telemetry and console log, including failed
starts, plus the tuning specification and parameter/restoration audit.
Unverified restoration means no further flight until baseline values are verified.
Keep the attempt ID, exact configuration, operators,
preflight outcome, observed crossings, time/penalties and abort reason.
An observer or agreed video/timing method establishes physical crossing validity;
the onboard crossing monitor uses the position estimate.

## R7: compare planned, estimated, and physical performance

Use the notebook for the path/error plots and the
[plotting reference](post_flight_analysis.md) for CLI options and telemetry
alignment. Match telemetry to the archived course from the same attempt folder. Host-time alignment
includes transport delay, and onboard position is not independent truth.

Distinguish planned duration, estimated tracking, observed crossings and timed
performance. A single attempt does not establish repeatability or a rare-failure
rate. Keep competing survey, estimation, control, visibility and range/flow
explanations when the data cannot distinguish them.

## Team notes and closing discussion

Keep at most one page in the existing team notes: margin/speed choice, one plot,
outcome and next change. Link the inputs, plans, preflight record and attempt log.
The [notes template](templates/race_report.md) is optional. Presentation slots
are in the [programme](day_3_race.md); the notebook contains the R7 discussion prompts.

## Stretch exercises

Declare extensions before the relevant freeze; qualify and compare them in
allocated flight slots:

| Extension | Task | Additional evidence and completion check |
| --- | --- | --- |
| PID/KF tuning | Flight notebook's stretch selector. | Baseline comparison and verified restoration. |
| Jerk-continuous reference | Follow the [optional planner extension](trajectory_construction.md#stretch-goal-continuous-and-limited-jerk): replace C2 quintics with a C3 reference and an explicit jerk bound. | Left/right derivative tests, acceleration/jerk plots and planned duration under matched limits; no tracking claim without closed-loop evidence. Retain the supplied planner and qualified fallback. |
| Racing line | Optimize off-centre crossing points or approach geometry. | Same course and limits; continuous-path clearance and dynamics, repeat-run benefit, and a retained central-crossing fallback. |
| Adaptive speed | Combine uncertainty, curvature, and braking constraints. | Closed-loop nominal/dropout trials and non-positive-clearance behaviour; no lap-time claims from a speed recommendation that does not affect motion. |
| Survey uncertainty | Propagate geometry errors into clearance decisions. | Document shared frame/calibration errors and correlations; compare predictions with independent or simulated perturbed geometry. |
| Banked turns | Use bank/attitude constraints in planning and clearance. | Swept-volume/thrust checks and attitude-aware simulation before a team-checked low-speed physical test. |
| Joint control/estimation | Compare an advanced controller/observer with the simpler choice. | Matched limits, complete trial ensembles, tracking/saturation/effort and completion-rate evidence, and measured computation/timing cost. |

These extensions are optional, not prerequisites or guarantees of the best
physical lap. Qualification takes priority over additional algorithm complexity.

**Optional integrated-race extension.** Only if a course-driven closed-loop harness
was completed and tested earlier, compare nominal, dropout, outlier and lag/
disturbance cases using development seeds 0-4 and held-out seeds 20-39. Check ordered
directional aperture crossings and abort transitions, retain every trajectory,
and report valid completions, times, clearance and tracking/saturation metrics.
Building that harness is additional work, not part of the 30-minute core slot.
The starter's `recommended_speed` does not alter prescribed truth motion; do not
claim faster laps unless the speed policy actually changes the simulated motion.

**Optional batch analysis.**

If automating analysis, an optional `runs.csv` can use:
`run_id,run_type,config_id,profile,course_id,survey_source,
preflight_passed,official_valid,completed_laps,required_laps,valid_crossings,
required_crossings,official_time_s,penalty,telemetry_path,observer_record,
tracking_rms_m,tracking_max_m,min_estimated_clearance_m,abort_reason`.
Define penalty units/rules in R1. Use empty fields for unavailable/inapplicable
quantities, not zero. Simulation results remain separate, with seeds/scenarios
and truth-based versus estimated metrics labelled.
