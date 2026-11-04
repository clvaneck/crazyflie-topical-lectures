# Post-flight trajectory analysis

Use the recording-analysis cells in [the trajectory notebook](../code/day3/02_trajectory.ipynb)
to select telemetry and the exact flown course, inspect the plots inline and save
them with the attempt. The labelled example is available when recordings are missing.

## Optional command-line reference

After activating the native environment, run from the project folder containing
`code/`, `scripts/` and the requirements files:
The following single-line command works in Bash and PowerShell.

```bash
python scripts/plot_race.py path/to/run.csv --course path/to/run.course.json --phase race --output plots/run --no-show
```

The command writes `plots/run-trajectory.png` (3D plus XY/XZ/YZ projections) and
`plots/run-errors.png` (signed X/Y/Z and total position errors). Omit `--no-show`
for interactive windows. `--phase` defaults to selecting `race`, or `trajectory`
when that is the recorded phase; `--phase all` includes takeoff and landing.

The plotter is offline: it never connects to or commands a drone. Geometry and
logs come from your team's checked course and flight, or supplied recordings.

## Inputs and interpretation

The course JSON has `course_name`, `start`, `finish`, and a `hoops` list. Each
gate needs `name`, `centre`, unit `normal`, and `radius_m`, in world metres.
Rectangle outlines can additionally specify `shape: rectangle`, `width_m`,
`height_m`, and orthogonal `width_axis` / `height_axis` vectors. Gate surveying
uses the [supplied survey tool and geometry checks](hoop_survey_challenge.md).

## Survey overlays

To compare the course with separately surveyed outlines, repeat `--gate-survey`
once per gate. This single-line command works in Bash and PowerShell:

```bash
python scripts/plot_race.py path/to/run.csv --course path/to/run.course.json --gate-survey survey/gate-1.json --gate-survey survey/gate-2.json --phase race --output plots/run-survey --no-show
```

Each survey file contains **one JSON object describing one gate**.
If your team's `gates.json` groups several gates, export each gate into its own
file with the fields below. Extra metadata is ignored. UTF-8 files with or without
a byte-order mark (BOM) are accepted for surveys, courses, and telemetry CSV.

| Field | Meaning |
| --- | --- |
| `name` | Optional label in the plot. |
| `centre` | Three finite coordinates `[x, y, z]`, in metres in the flight's Lighthouse world frame. |
| `normal` | Unit vector perpendicular to the gate plane, in that same frame. |
| `shape` | `circle` (also the default if omitted) or `rectangle`. |
| `radius_m` | Positive opening radius; required for circles only. |
| `width_m`, `height_m` | Positive opening dimensions; required for rectangles only. |
| `width_axis`, `height_axis` | Unit vectors along the rectangle's width and height, in world coordinates. Required for rectangles; mutually perpendicular and perpendicular to `normal`. |

Example circle file (illustrative coordinates, not a surveyed gate):

```json
{
  "name": "circle-1",
  "shape": "circle",
  "centre": [1.0, 0.0, 0.8],
  "normal": [1.0, 0.0, 0.0],
  "radius_m": 0.5
}
```

Example rectangle file:

```json
{
  "name": "rectangle-1",
  "shape": "rectangle",
  "centre": [1.0, 0.0, 0.8],
  "normal": [1.0, 0.0, 0.0],
  "width_m": 1.0,
  "height_m": 0.6,
  "width_axis": [0.0, 1.0, 0.0],
  "height_axis": [0.0, 0.0, 1.0]
}
```

Survey overlays draw the measured geometry for comparison with the course.
Course changes require the team's flight checks to be repeated. Use the same
coordinate frame and opening definition for survey and course comparisons.

## Telemetry alignment

CSV rows use `event` and `host_monotonic_s`. Position rows (`event=position`)
have `x_m`, `y_m`, `z_m`; command rows (`event=command`) have `command_x_m`,
`command_y_m`, `command_z_m`, and `phase`. Other event rows are ignored.

Each estimated position is compared with the last preceding sent command.
Commands older than `--max-gap` (default 0.25 s) are left unmatched, and long
gaps break drawn lines. Estimated position minus sent command includes transport
delay; it is not an independently measured physical tracking error.

The plotter compares recorded commands, preserving their actual
host send timing. Logs
without command events need an explicitly documented alternative analysis;
planned duration alone is not a reliable synchronization reference.
