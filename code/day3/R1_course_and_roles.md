# R1: course, frame, rules and roles

Prepared from `examples/course.shuttle.synthetic.json` (simulated, `enabled_for_flight: false`).
Replace the template values with the agreed physical values at the 09:00 briefing.

## Frame

- Right-handed Lighthouse frame, `[x, y, z]` in metres, Z up.
- The course is a shuttle: start and finish at `[0, 0, 0.7]`. One physical gate at `[1, 0, 0.7]` (radius 0.5 m),
  crossed `forward` (normal +x) and then `return` (normal -x). With `laps = 2` that is 4 crossings.
- Entry and exit points are `C ∓ 0.4·n`, at x = 0.6 m and x = 1.4 m.
- Start and finish lie inside the bounds `[-0.5, -1.0, -0.2]` to `[2.5, 1.0, 1.5]`.

## First-crossing prediction

Moving from the start (x = 0) towards the gate (x = 1) means v ≈ +x, and n = +x, so **v·n > 0 (sign +)**.
I checked this against the planned trajectory, sampled at each crossing:

| Crossing | t [s] | v [m/s] | v·n |
| --- | --- | --- | --- |
| lap-1-1-forward | 6.01 | +0.167 x | + |
| lap-1-2-return | 10.81 | -0.167 x | + |
| lap-2-1-forward | 15.62 | +0.167 x | + |
| lap-2-2-return | 20.42 | -0.167 x | + |

Every crossing goes along its own normal, so the "return" gate is just the same hoop with n flipped.
The crossing speed is 0.167 m/s, not the target of 0.3 m/s, because the shuttle stops at each shared exit/entry
waypoint to turn around. The planned duration is 26.4 s.

## Limits (template, to be agreed)

| Limit | Template | Agreed |
| --- | --- | --- |
| target / max speed | 0.3 / 0.5 m/s | |
| max acceleration | 1.0 m/s² | |
| max tilt | 20° | |
| drone radius (cage) | 0.12 m | measure |
| clearance margin (shared floor) | 0.08 m | |
| radial space left | 0.5 − 0.12 − 0.08 = 0.30 m | |
| pre/post distance | 0.4 m | |
| max tracking error (abort) | 0.3 m | |
| bounds (incl. netting, floor, supports) | see above | measure |

## Rules (agree in the group)

- Gate order and directions:
- Laps:
- Start/finish definition and timing method (observer / video):
- What counts as a valid physical crossing:
- Penalties (units):

## Roles

| Role | Name |
| --- | --- |
| Pilot (runs the flight notebook) | |
| Emergency-stop operator (separate person!) | |
| Telemetry monitor | |
| Recorder (attempt log) | |

Anyone can call a stop. Readiness is agreed in the group before each flight.
