# Documentation guide

Before the lectures, follow the [installation guide](software_setup.md).
The [hardware overview](drone_hardware.md) introduces the drone and its sensors;
the [glossary](glossary.md) is available as a reference.

## For the scheduled exercises

The programmes give schedules and learning goals. The notebooks are the working
instructions.

| Day | Programme | Working material |
| --- | --- | --- |
| 1 | [Feedback control](day_1_control.md) | [Notebook guide](../code/day1/README.md) |
| 2 | [Kalman filtering](day_2_kalman.md) | [Numbered notebooks](../code/day2/README.md) |
| 3 | [Race](day_3_race.md) | [Survey](../code/day3/01_survey.ipynb), [planning/analysis](../code/day3/02_trajectory.ipynb), [flight](../code/day3/03_flight.ipynb) |

Keep one shared team notebook or notes file, at most one page per day plus
links to saved outputs. The [shared conventions](practical_workflow.md) cover
team responsibilities and repeatable comparisons. Record where your
data came from and complete the handling and flight checks, including with
shared hardware or fallback data.

## Practical reference

Look up these details as needed during the notebooks:

- [Hardware baseline checks](day_1_baseline_checks.md), including battery checks,
  and the [race flight checklist](race_practical.md).
- [Sensor calibration](sensor_bias_calibration_exercise.md) and
  [recording formats and acquisition](sensor_recording_contract.md).
- [Gate survey](hoop_survey_challenge.md), [course files and flight interface](course_interface.md)
  and [post-flight plots](post_flight_analysis.md).
- [Git updates and sharing](course_git.md) and [code layout and tests](../code/README.md).

## Further reading: theory and experimental context

The following are optional background or extensions:

- [Additional controller comparisons](controller_practical.md) and
  [CLI tools](controller_cli_examples.md).
- [Filter conventions and extensions](kalman_filter_practical.md) and
  [six-state experiments](../code/day2/six_state_reference.md).
- [Planner bounds, limitations and jerk-continuity extension](trajectory_construction.md).
- [Rotations and quaternions](rotations_and_quaternions.md).
- [Quadrotor equations of motion](quadrotor_equations_of_motion.md).
- [Detector-system analogies](detector_system_analogies.md),
  [collider-tracking Kalman analogies](collider_tracking_kalman_analogies.md)
  and [gravitational-wave feedback analogies](gravitational_wave_control_analogies.md).
- Optional notes prompts for [Day 1](templates/controller_report.md),
  [Day 2](templates/kalman_filter_report.md) and [Day 3](templates/race_report.md).

### Further courses and worked examples

- [Brian Douglas: What Is PID Control?](https://www.youtube.com/watch?v=wkfEZmsQqiA):
  an introduction to feedback and PID; see also the
  [Understanding PID Control playlist](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y).
- [ETH Zurich: Quadrotors, Control and Estimation](https://www.dfall.ethz.ch/pands.php):
  Crazyflie lecture notes and practical sheets progressing from modelling and
  simulation to PID/LQR control and flight. Its software uses MATLAB/Simulink
  and C++; our exercises keep the supplied Python environment.
- [Roger Labbe: Kalman and Bayesian Filters in Python](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python):
  executable explanations and exercises; chapters 4, 6 and 8 are particularly
  useful for scalar filtering, multivariate filtering and filter design.
- [MIT: Underactuated Robotics](https://underactuated.mit.edu/):
  lecture notes and videos; the quadrotor sections connect planar dynamics,
  tilt and thrust to feedback control. LQR and trajectory optimization offer
  further reading for the stretch goals.
- [NASA NESC: Fundamentals of Kalman Filtering and Estimation](https://nescacademy.nasa.gov/video/7287e7815cc049bdbaf4dc04830107341d):
  Chris D'Souza's seminar and slides connect estimation and least squares to
  practical aerospace applications.

These are optional references; the supplied environment and exercises contain
the material needed for the three days.

