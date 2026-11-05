#!/usr/bin/env python3
"""Load hoop locations from a JSON file, print a table and draw top/side views.

Offline only; never connects to a drone.

    python code/day3/hoop_view.py course.json [--plan plans/x] [--png out.png]
    python code/day3/hoop_view.py organisers.json --to-course examples/course.synthetic.json our-course.json

Accepted input (the loader is tolerant; adapt `_hoop` below if the file differs):
  * a course file {"hoops": [...], "limits": {"bounds_min", "bounds_max"}, "start", "finish"},
    a dict with "gates" instead of "hoops", a bare list of hoops, or one survey dict.
  * per hoop:  name | id | gate_id | label            (default hoop-N)
               centre | center | position | location  ([x,y,z] or {"x","y","z"})
               normal | direction | axis              (crossing direction, unit or not)
               radius_m | radius | diameter_m/2 | diameter/2 | min(width_m, height_m)/2
               points_m | points | edges | rim | corners (rim samples; also under "survey"):
                 used to fit centre/normal/radius when those are missing.
  * optional bounds: limits.bounds_min/max, bounds_min/max, or arena {"min", "max"}.
Lengths in metres, Lighthouse frame, Z up. With --plan, trajectory.csv from a planner
output directory is overlaid.
"""

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from race_common.geometry import plane_basis, unit_vector  # noqa: E402
from race_common.hoop_fit import fit_circle_3d  # noqa: E402


def _pick(record, *keys):
    for key in keys:
        if key in record and record[key] is not None:
            return record[key]
    return None


def _xyz(value, what):
    if isinstance(value, dict):
        value = [value[k] for k in ("x", "y", "z")]
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"{what} must be three finite numbers, got {value!r}")
    return result


def _hoop(record, index):
    name = str(_pick(record, "name", "id", "gate_id", "label") or f"hoop-{index}")
    points = _pick(record, "points_m", "points", "edges", "rim", "corners")
    if points is None and isinstance(record.get("survey"), dict):
        points = _pick(record["survey"], "points_m", "points")
    fit = None
    if points is not None:
        fit = fit_circle_3d(np.array([_xyz(p, f"{name} rim point") for p in points]))
    centre = _pick(record, "centre", "center", "position", "location")
    normal = _pick(record, "normal", "direction", "axis")
    radius = _pick(record, "radius_m", "radius")
    if radius is None and _pick(record, "diameter_m", "diameter") is not None:
        radius = float(_pick(record, "diameter_m", "diameter")) / 2
    if radius is None and "width_m" in record and "height_m" in record:
        radius = min(float(record["width_m"]), float(record["height_m"])) / 2  # inscribed circle
    if fit is not None:
        centre = fit.centre if centre is None else centre
        radius = fit.radius_m if radius is None else radius
        if normal is None:
            normal = fit.normal
            print(f"WARNING {name}: normal fitted from rim points; its SIGN is a guess, check the arrow")
    missing = [k for k, v in (("centre", centre), ("normal", normal), ("radius", radius)) if v is None]
    if missing:
        raise ValueError(f"{name}: missing {', '.join(missing)}; keys present: {sorted(record)}")
    hoop = dict(name=name, centre=_xyz(centre, f"{name} centre").tolist(),
                normal=unit_vector(_xyz(normal, f"{name} normal")).tolist(), radius_m=float(radius))
    if not 0 < hoop["radius_m"] < 3:
        raise ValueError(f"{name}: radius {hoop['radius_m']} m is implausible (units?)")
    return hoop


def load(path):
    """Return (hoops, bounds or None, {start/finish: xyz}) from a tolerant JSON read."""
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if isinstance(data, list):
        records, data = data, {}
    elif "hoops" in data or "gates" in data:
        records = _pick(data, "hoops", "gates")
    else:
        records = [data]
    hoops = [_hoop(record, index) for index, record in enumerate(records, start=1)]
    limits = data.get("limits", {})
    low = _pick(limits, "bounds_min") or _pick(data, "bounds_min") or (data.get("arena") or {}).get("min")
    high = _pick(limits, "bounds_max") or _pick(data, "bounds_max") or (data.get("arena") or {}).get("max")
    bounds = None if low is None or high is None else (_xyz(low, "bounds_min"), _xyz(high, "bounds_max"))
    ends = {key: _xyz(data[key], key) for key in ("start", "finish") if key in data}
    if max(abs(c) for hoop in hoops for c in hoop["centre"]) > 20:
        print("WARNING: a centre is more than 20 m from the origin; are the units millimetres?")
    return hoops, bounds, ends


def table(hoops):
    lines = [f"{'#':>2} {'name':<22} {'centre x,y,z [m]':<22} {'r [m]':>5}  {'normal':<18}"
             f" {'to next [m]':>11} {'n.dir_next':>10}"]
    for index, hoop in enumerate(hoops):
        centre, normal = np.array(hoop["centre"]), np.array(hoop["normal"])
        distance = alignment = ""
        if index + 1 < len(hoops):
            step = np.array(hoops[index + 1]["centre"]) - centre
            distance = f"{np.linalg.norm(step):.2f}"
            # Negative: this normal points away from the next hoop (possible sign error).
            alignment = f"{normal @ step / np.linalg.norm(step):+.2f}" if np.linalg.norm(step) > 1e-6 else "same"
        lines.append(f"{index + 1:>2} {hoop['name'][:22]:<22} {'%.2f,%.2f,%.2f' % tuple(centre):<22}"
                     f" {hoop['radius_m']:>5.2f}  {'%+.2f,%+.2f,%+.2f' % tuple(normal):<18}"
                     f" {distance:>11} {alignment:>10}")
    return "\n".join(lines)


def plot(hoops, bounds, ends, path_xyz=None, title=""):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(2, 1, figsize=(12, 9), layout="constrained")
    angles = np.linspace(0, 2 * np.pi, 73)
    for axis, (a, b), label in zip(axes, ((0, 1), (0, 2)), ("Top view (x-y)", "Side view (x-z)")):
        if bounds is not None:
            low, high = bounds
            axis.add_patch(plt.Rectangle((low[a], low[b]), high[a] - low[a], high[b] - low[b],
                                         fill=False, color="C3", linestyle="--", label="bounds"))
        if path_xyz is not None:
            axis.plot(path_xyz[:, a], path_xyz[:, b], color="C0", linewidth=1, label="planned path")
        for index, hoop in enumerate(hoops, start=1):
            centre, normal = np.array(hoop["centre"]), np.array(hoop["normal"])
            basis = plane_basis(normal)
            rim = centre + hoop["radius_m"] * (np.cos(angles)[:, None] * basis[:, 0]
                                               + np.sin(angles)[:, None] * basis[:, 1])
            axis.plot(rim[:, a], rim[:, b], color="0.3")
            axis.annotate("", xy=(centre[a] + 0.4 * normal[a], centre[b] + 0.4 * normal[b]),
                          xytext=(centre[a], centre[b]), arrowprops=dict(arrowstyle="->", color="C1"))
            axis.text(centre[a], centre[b], f" {index}", fontsize=11, color="C1", weight="bold")
        for key, marker in (("start", "o"), ("finish", "x")):
            if key in ends:
                axis.plot(ends[key][a], ends[key][b], marker, color="C2", markersize=9, label=key)
        axis.set(title=label, xlabel=f"{'xyz'[a]} [m]", ylabel=f"{'xyz'[b]} [m]")
        axis.set_aspect("equal", adjustable="datalim")
        axis.grid(True)
        axis.legend(loc="upper right", fontsize=8)
    figure.suptitle(f"{title}  (orange arrow = normal = crossing direction, 0.4 m)")
    return figure


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("course", type=Path, help="Course, survey or organisers' hoop JSON")
    parser.add_argument("--plan", type=Path, help="Planner output directory with trajectory.csv")
    parser.add_argument("--png", type=Path, help="Output PNG (default: next to the input, *.hoops.png)")
    parser.add_argument("--to-course", nargs=2, type=Path, metavar=("TEMPLATE", "OUT"),
                        help="Write OUT: TEMPLATE course with its hoops replaced by the loaded ones")
    args = parser.parse_args()
    try:
        hoops, bounds, ends = load(args.course)
    except (ValueError, KeyError, TypeError, OSError, json.JSONDecodeError) as error:
        parser.exit(1, f"Could not load hoops: {error}\n")
    print(table(hoops))
    if bounds is not None:
        outside = [h["name"] for h in hoops if np.any(np.array(h["centre"]) < bounds[0])
                   or np.any(np.array(h["centre"]) > bounds[1])]
        print(f"bounds {bounds[0].tolist()} .. {bounds[1].tolist()}"
              + (f"  OUTSIDE: {outside}" if outside else "  (all centres inside)"))
    else:
        print("no bounds in this file")
    path_xyz = None
    if args.plan:
        with (args.plan / "trajectory.csv").open() as stream:
            rows = list(csv.DictReader(stream))
        path_xyz = np.array([[float(r["x_m"]), float(r["y_m"]), float(r["z_m"])] for r in rows])
    png = args.png or args.course.with_suffix(".hoops.png")
    plot(hoops, bounds, ends, path_xyz, title=args.course.name).savefig(png, dpi=130)
    print(f"Wrote {png}")
    if args.to_course:
        template_path, out = args.to_course
        if out.exists():
            parser.exit(1, f"{out} exists; choose a new name\n")
        course = json.loads(template_path.read_text(encoding="utf-8-sig"))
        course["hoops"], course["enabled_for_flight"] = hoops, False
        out.write_text(json.dumps(course, indent=2) + "\n", encoding="utf-8")
        print(f"Wrote {out}: template limits/start/finish/bounds kept, CHECK them for the real arena")


if __name__ == "__main__":
    main()
