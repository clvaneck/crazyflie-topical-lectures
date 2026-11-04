# Operation Hoop Storm: race briefing (4 Nov 2026)

Five-minute talk on our Day 2 progress, built with manim-slides (+ [manim-hep](https://github.com/NielsterLinden/manim-hep)).
Two points: the stationary IMU calibration made flight worse (race on firmware defaults), and losing
the range sensor shrinks the hoop clearance, so the speed rule slows us down.

## Watch

`mp4/01_Title.mp4` to `mp4/08_End.mp4`, one clip per section.

## Present (click-through, rotors keep spinning during pauses)

```bash
python3.12 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/manim render -qh --fps 30 talk.py Title Reaper GamePlan TB2 Progress1 Bomber Progress2 End
./present.sh        # right arrow = next, q = quit
```

`manim-slides render` rejects `--fps`, so render with plain `manim`; it still writes `slides/` for the presenter.
Axis numbers are plain text on purpose: manim's LaTeX numbers need TeX packages not every laptop has.

## Files

- `talk.py`: all scenes; the words on screen are constants at the top
- `prep_data.py`: freezes the two real traces from `code/day2/results/` into `data/*.npz`
- `storyline.html`: the first plan (since changed: military fly-bys, no conclusion slide)
- `fonts/`: Courier Prime (OFL) and Special Elite (Apache 2.0); `assets/nikhef.svg`: Nikhef logo
