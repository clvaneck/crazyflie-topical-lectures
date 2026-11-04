# Crazyflie Topical Lectures

The [course project](https://gitlab.nikhef.nl/graven/crazyflie-topical-lectures/)
is the starting point for the lectures. This README links the installation
guide, the three daily programmes, exercises and supporting material.

**Topic:** We use hands-on drone experiments to study feedback control and
Kalman filtering. Along the way, we connect these ideas to particle-physics
tracking and gravitational-wave instrumentation.

The exercises are starting points for investigating models and comparing ideas.
We work in groups of four or five per drone, sharing measurements, cross-checks,
piloting and recording. Before each experimental stage, the group cross-checks
the configuration and results so far, agrees on the next experiment and confirms
that its checks and operating limits are understood. Before flight, the group
also identifies the pilot and emergency-stop operator and agrees whether the
drone and flight area are ready.

**General information on the drone:** The [hardware and sensor guide](docs/drone_hardware.md)
describes the drone, its sensors, and the Lighthouse and Flow decks.
The [practical glossary](docs/glossary.md) explains terminology used in the guides.

## Prerequisites

No prior drone experience is assumed. Bring a laptop and follow the
[installation guide](docs/software_setup.md) before the lectures. It covers
GitLab access, installing Git and Python, downloading the course, creating the
Python environment and checking the software. Start there even if you have
not installed Git or downloaded the files yet. We check the hardware connections
together during Day 1.

The
[Git guide](docs/course_git.md) explains how to keep your team's work on a branch,
merge course updates and share improvements through a fork and merge request.
The native Python environment supports cfclient, simulations and notebooks. Docker is an optional
way to run simulations, rather than a prerequisite for the course.

## Three-day programme

### Day 1: feedback control

A lecture introduces control theory, feedback, stability and PID controllers.
We use position and velocity [PID](docs/glossary.md#pd-and-pid) experiments to see
how a controller follows a target, responds to disturbances and behaves near its
limits. The afternoon connects those models to first hover and position-loop
experiments, with preflight checks and recorded-data analysis alongside the
shared flight slots.

**Detailed programme and exercises:** [Day 1 programme](docs/day_1_control.md).

### Day 2: Kalman filtering

A lecture introduces the theory, history and applications of Kalman filters.
We then investigate sensor calibration and state estimation, comparing uncertainty,
outliers and missing measurements in simulation and live motor-free experiments.

**Detailed programme and exercises:** [Day 2 programme](docs/day_2_kalman.md).

### Day 3: race

The race brings these ideas together. We measure the gates, plan a trajectory
through them and compare how controller and estimator choices affect speed,
clearance and repeatability. Before timed runs, teams check their configuration
in low-speed tests. A tested conservative configuration provides a fallback when
an experimental choice is not ready for flight.

**Detailed programme and exercises:** [Day 3 programme](docs/day_3_race.md).

The [documentation guide](docs/README.md) links experiments, optional notes prompts,
and theory notes. The programmes give the timetable; the notebooks contain the
working instructions and completion checks.
The longer derivations, detector analogies, external courses and stretch goals
are optional background or further exploration. Flight and handling checks apply
whenever hardware is used.
The [code guide](code/README.md) describes the starter and
offline tests. A shared notebook or notes file is enough: at most one page
per day plus links to outputs, rather than separate exercise reports.
Having the software installed beforehand leaves more time for experiments;
software verification follows the general
introduction. Each team is responsible for its checks, configuration and decision
to fly, with a clear shared flight area and a separate emergency-stop operator.
The [shared practical conventions](docs/practical_workflow.md#responsibility-for-the-experiment)
describe these responsibilities and when to stop.

## About this release

Release **0.1.11** contains exercises, theory notes, starter code, setup
scripts, and offline trajectory analysis. Organizers provide any tested
flight fallback and recordings separately. The starter does not implement a
complete flight runner. Package versions are pinned for reproducible experiments.
`RELEASE.json` records this release's version, source revision, and file checksums.
