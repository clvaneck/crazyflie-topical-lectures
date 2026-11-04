"""Freeze the two real traces the talk shows into data/*.npz. Run once: .venv/bin/python prep_data.py"""

import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DAY2 = HERE.parent / "code" / "day2"
sys.path.insert(0, str(DAY2))
import vertical  # noqa: E402  (the course's own filter, branch caspar-tuning)

OUT = HERE / "data"
OUT.mkdir(exist_ok=True)

# Slide 6: E2 flight at 13:04 with the stationary noise values.
flight = DAY2 / "results" / "E2" / "flight" / "flight_4_1304_firmware_fwd"
state = np.genfromtxt(f"{flight}_state.csv", delimiter=",", names=True)
pid = np.genfromtxt(f"{flight}_pid.csv", delimiter=",", names=True)
t0 = state["t_ms"][0]
np.savez(OUT / "flight.npz",
         t=(state["t_ms"] - t0) / 1000, x=state["stateEstimatex"], y=state["stateEstimatey"],
         target_t=(pid["t_ms"] - t0) / 1000, target_x=pid["target_x"], target_y=pid["target_y"])

# Slide 8: E5A, range sensor removed 8-11 s, same settings as RESULTS_day2.md (sigma_a 0.1, measured V).
rec = DAY2 / "results" / "recordings" / "e3_training_capture_131904"
with tempfile.TemporaryDirectory() as tmp:
    shutil.copy(f"{rec}.csv", Path(tmp) / "raw.csv")
    shutil.copy(f"{rec}.json", Path(tmp) / "recording.json")
    raw, meta = vertical.load_raw(tmp)
data = vertical.prepare(raw, meta)
gap = vertical.run(data, range_gap=(8.0, 11.0))
sigma = np.sqrt(gap["covariance"][:, 0, 0])
np.savez(OUT / "dropout.npz", t=data.time_s, tof=data.height_m,
         z=gap["state"][:, 0], sigma=sigma)

# Check against RESULTS_day2.md E5A: sigma 0.11 cm before, 7.3 cm at 10.95 s.
t = data.time_s
print("sigma before gap [cm]:", round(100 * sigma[t < 8][-1], 2))
print("sigma at end of gap [cm]:", round(100 * sigma[t <= 11][-1], 2))
