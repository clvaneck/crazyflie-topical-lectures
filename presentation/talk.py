"""Operation Hoop Storm: race-strategy briefing, team drone 4 (Topical Lectures, Nikhef, 4 Nov 2026).

Render:  .venv/bin/manim render -qh --fps 30 talk.py $SCENES
Present: ./present.sh        (right arrow = next)
with SCENES="Title Reaper GamePlan TB2 Progress1 Bomber Progress2 End".
Traces come from data/*.npz, written by prep_data.py from the team repo.
Every pause is a 1 s loop, so the logo's rotors keep spinning while you talk.
"""

from pathlib import Path

import manimpango
import numpy as np
from manim import (BOLD, DL, DOWN, DR, LEFT, PI, RIGHT, UL, UP, UR, AddTextLetterByLetter, Axes, Circle, Create,
                   DashedVMobject, Dot, Ellipse, FadeIn, FadeOut, GrowFromEdge, Line, Polygon, Rectangle,
                   SVGMobject, Text, ValueTracker, VGroup, VMobject, Write, always_redraw, config, linear,
                   rate_functions, smooth)
from manim_slides import Slide
from manim_hep import use_theme

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
for ttf in sorted((HERE / "fonts").glob("*.ttf")):
    manimpango.register_font(str(ttf))
TYPE, STAMP = "Courier Prime", "Special Elite"  # typewriter body, inked headers and stamps

C = use_theme("generic").palette  # data colours (Okabe-Ito) from manim-hep
PAPER, INK, RED, CANVAS, NIKHEF = "#F3EFE4", "#23262B", "#A4161A", "#E4DBBF", "#E6223D"
config.background_color = PAPER

# ---- Words on screen: edit these, then re-render. ----
BANNER = "OPERATION HOOP STORM"
CREW = "Caspar · Niels · Maria · Mattia · Siang"
BRIEF = "Race briefing · Topical Lectures · Nikhef · 04 NOV 2026"
MARKING = "TOP SECRET // HOOP STORM // EYES ONLY"
REF = "REF. HS-2026-1104"
GAME_PLAN = [  # (line, words hidden under black bars)
    ("Cruise 0.44 m/s, brake to 0.27 m/s when blind", ["0.44 m/s", "0.27 m/s", "when blind"]),
    ("Fly on firmware noise values, not ours", ["firmware noise values", "not ours"]),
    ("χ² gate α = 0.01 drops bad range readings", ["α = 0.01", "bad range readings"]),
    ("Abort if tracking error > 0.3 m", ["0.3 m"]),
    ("Secret weapon: duct tape", ["duct tape"]),
]

# ---- Page layout ----
LOGO_H = 1.25
RULE_Y = 2.25
SPIN = 3 * PI  # rad/s; one 1 s hold loop turns a two-blade prop by 3 pi, so the loop is seamless


def txt(s, size=28, color=INK, weight="NORMAL", font=TYPE):
    return Text(s, font_size=size, color=color, weight=weight, font=font, disable_ligatures=True)


def fit(m, width):
    return m.scale_to_fit_width(min(m.width, width))


def paperwork():
    """Classification markings, reference number and the double rule under the header row."""
    top = txt(MARKING, 18, RED, BOLD).move_to([0, 3.78, 0])
    ref = txt(REF, 16, C.muted).set_y(3.78).to_edge(RIGHT, buff=0.45)
    rules = VGroup(Line([-6.85, RULE_Y, 0], [6.85, RULE_Y, 0]).set_stroke(INK, 2),
                   Line([-6.85, RULE_Y - 0.07, 0], [6.85, RULE_Y - 0.07, 0]).set_stroke(INK, 0.8))
    return VGroup(top, top.copy().set_y(-3.78), ref, rules)


def copter(height=LOGO_H):
    """The Nikhef vertex mark (no lettering) as a quadcopter: rotors on its four arm tips."""
    svg = SVGMobject(str(HERE / "assets" / "nikhef.svg"))
    mark = max((m for m in svg.family_members_with_points() if m.get_fill_opacity() > 0), key=lambda m: m.height)
    radius = 38 * mark.width / 138.5  # 38 svg units; the mark is 138.5 svg units wide
    rotors = VGroup()
    for corner, sign in [(UL, 1), (UR, -1), (DL, -1), (DR, 1)]:  # neighbours counter-rotate
        c = mark.get_corner(corner)
        disc = Circle(radius=radius).move_to(c).set_stroke(NIKHEF, 1.5, opacity=0.7).set_fill(NIKHEF, 0.08)
        blade = Line(c + LEFT * radius, c + RIGHT * radius).set_stroke(INK, 3)
        blade.add_updater(lambda m, dt, sign=sign: m.rotate(sign * SPIN * dt))
        rotors.add(VGroup(disc, blade))
    return VGroup(mark, rotors).scale_to_fit_height(height)


LOGO_W = copter().width
LOGO_AT = np.array([-6.85 + LOGO_W / 2, 2.95, 0])


def glide(scene, mob, to, run_time, rate_func=smooth, width=None):
    """Move (and resize) with an updater, so the rotors keep spinning on the way."""
    start, w0, a = mob.get_center().copy(), mob.width, ValueTracker(0)

    def follow(m):
        t = a.get_value()
        if width:
            m.scale_to_fit_width(w0 + (width - w0) * t)
        m.move_to(start + (np.asarray(to) - start) * t)

    mob.add_updater(follow)
    scene.play(a.animate.set_value(1), run_time=run_time, rate_func=rate_func)
    mob.remove_updater(follow)


# ---- Aircraft, top view: nose right, wingtips top and bottom. Schematic silhouettes. ----
def poly(points):
    return Polygon(*[np.array([x, y, 0.0]) for x, y in points], fill_color=INK, fill_opacity=1, stroke_width=0)


def mirrored(half):  # half outline nose -> tail centre on the +y side, closed by mirroring
    return poly(half + [(x, -y) for x, y in reversed(half[1:-1])])


def both(points):
    return [poly(points), poly([(x, -y) for x, y in points])]


def b2(span):
    return VGroup(mirrored([(1.05, 0), (-0.30, 2.6), (-0.85, 1.80), (-0.42, 1.30),
                            (-0.82, 0.85), (-0.48, 0.45), (-0.80, 0)])).scale_to_fit_height(span)


def reaper(span):
    return VGroup(mirrored([(1.2, 0), (1.1, 0.11), (0.7, 0.13), (-1.0, 0.07), (-1.1, 0)]),
                  *both([(0.25, 0.1), (0.13, 2.0), (-0.02, 2.0), (-0.12, 0.1)]),
                  *both([(-0.8, 0.05), (-1.05, 0.6), (-1.15, 0.6), (-1.0, 0.05)]),
                  poly([(-1.11, -0.3), (-1.11, 0.3), (-1.15, 0.3), (-1.15, -0.3)])).scale_to_fit_height(span)


def tb2(span):
    return VGroup(mirrored([(0.75, 0), (0.68, 0.09), (0.4, 0.1), (-0.75, 0.05), (-0.85, 0)]),
                  *both([(0.12, 0.08), (0.06, 1.5), (-0.06, 1.5), (-0.12, 0.08)]),
                  *both([(-0.55, 0.04), (-0.78, 0.42), (-0.86, 0.42), (-0.72, 0.04)]),
                  poly([(-0.86, -0.22), (-0.86, 0.22), (-0.89, 0.22), (-0.89, -0.22)])).scale_to_fit_height(span)



def crazyflie(width, color=INK):
    arms = VGroup(Line(UL, DR), Line(UR, DL)).set_stroke(color, 6)
    rotors = VGroup(*[Circle(radius=0.38).move_to(p).set_stroke(color, 3) for p in (UL, UR, DL, DR)])
    body = Rectangle(width=0.45, height=0.45).set_fill(color, 1).set_stroke(width=0)
    return VGroup(arms, rotors, body).scale_to_fit_width(width)


def banner(s, emblem=0.0):
    """Canvas banner; `emblem` leaves room on the left for the logo to ride along."""
    label = txt(s, 38, INK, font=STAMP)
    L, H = label.width + 0.9 + emblem, max(label.height + 0.7, 1.45 if emblem else 0)
    xs = np.linspace(0, L, 60)
    wave = 0.1 * np.sin(3 * np.pi * xs / L)
    top = [np.array([x - L / 2, H / 2 + w, 0]) for x, w in zip(xs, wave)]
    bottom = [np.array([x - L / 2, -H / 2 + w, 0]) for x, w in zip(xs[::-1], wave[::-1])]
    cloth = Polygon(*top, *bottom, fill_color=CANVAS, fill_opacity=1, stroke_color=INK, stroke_width=2)
    return VGroup(cloth, label.move_to(cloth).shift(RIGHT * emblem / 2))


def towing(flag, rtl=False):
    plane = b2(2.4)
    if rtl:
        plane.rotate(PI)
        flag.next_to(plane, RIGHT, buff=1.0)
        rope = Line(flag.get_left(), plane.get_right())
    else:
        flag.next_to(plane, LEFT, buff=1.0)
        rope = Line(flag.get_right(), plane.get_left())
    return VGroup(flag, rope.set_stroke(C.muted, 2), plane)


def slow_middle(t):  # monotonic; the banner lingers mid-screen
    return t + 0.12 * np.sin(2 * PI * t)


def fly(scene, mob, run_time, rtl=False, y=(-0.6, 0.6), rate_func=linear):
    half = config.frame_width / 2 + mob.width / 2 + 0.3
    sign = -1 if rtl else 1
    mob.move_to([-sign * half, y[0], 0])
    scene.add(mob)
    scene.play(mob.animate.move_to([sign * half, y[1], 0]), run_time=run_time, rate_func=rate_func)
    scene.remove(mob)


class Page(Slide):
    """Every section: report paperwork, the logo top left, an optional header centred on the logo."""

    def page(self, title=None, logo=True):
        self.next_slide(auto_next=True)
        self.keep = [paperwork()]
        if logo:
            self.logo = copter().move_to(LOGO_AT)
            self.keep.append(self.logo)
        self.add(*self.keep)
        if title:
            head = fit(txt(title, 34, INK, font=STAMP), 11.0)
            head.next_to(self.logo, RIGHT, buff=0.35).set_y(self.logo.get_center()[1])
            self.play(AddTextLetterByLetter(head, time_per_char=0.03))

    def hold(self, notes=""):
        """Pause here: a seamless 1 s loop (spinning rotors) until the presenter clicks."""
        self.next_slide(loop=True, notes=notes)
        self.wait(1.0)
        self.next_slide(auto_next=True)

    def clear_page(self):
        gone = [m for m in self.mobjects if m not in self.keep]
        for m in gone:
            m.clear_updaters()
        if gone:
            self.play(FadeOut(*gone), run_time=0.5)


# ---- Sections ----
class Title(Page):
    def construct(self):
        self.page(logo=False)
        flag = banner(BANNER, emblem=LOGO_W + 0.3)
        tow = towing(flag)
        bot = copter().move_to(flag[0].get_left() + RIGHT * (0.45 + LOGO_W / 2))
        convoy = VGroup(tow, bot)
        offset = convoy.get_center() - flag[0].get_center()
        convoy.move_to([-(config.frame_width / 2 + convoy.width / 2 + 0.3), 0.7, 0] + offset * [0, 1, 0])
        self.add(convoy)
        glide(self, convoy, np.array([0, 0.7, 0]) + offset, 3.5, rate_functions.ease_out_cubic)

        crew = VGroup(txt("OPERATIVES", 20, RED, BOLD), txt(CREW, 32, INK, BOLD)).arrange(DOWN, buff=0.18)
        crew.move_to([0, -1.35, 0])
        brief = txt(BRIEF, 20, C.muted).move_to([0, -2.6, 0])
        self.play(FadeIn(crew[0]))
        self.play(AddTextLetterByLetter(crew[1], time_per_char=0.04))
        self.play(FadeIn(brief))
        glide(self, bot, LOGO_AT, 1.6, width=LOGO_W)  # the logo lifts off the banner and takes its post
        self.keep.append(bot)
        self.hold("We are Operation Hoop Storm: Caspar, Niels, Maria, Mattia and Siang.")

        self.play(tow.animate.shift(RIGHT * (config.frame_width + 2)), FadeOut(crew, brief),
                  run_time=2.2, rate_func=rate_functions.ease_in_cubic)
        self.remove(tow)


class _FlyBy(Page):
    BUILD, SPAN, RTL = None, 3.0, False

    def construct(self):
        self.page()
        plane = type(self).BUILD(self.SPAN)
        if self.RTL:
            plane.rotate(PI)
        fly(self, plane, 2.6, rtl=self.RTL)


class Reaper(_FlyBy):
    BUILD, SPAN, RTL = staticmethod(reaper), 3.6, True


class TB2(_FlyBy):
    BUILD, SPAN, RTL = staticmethod(tb2), 3.0, False


class Bomber(_FlyBy):
    BUILD, SPAN, RTL = staticmethod(b2), 3.0, True


class GamePlan(Page):
    def construct(self):
        self.page("GAME PLAN · DAY 3")
        lines = VGroup(*[txt("• " + s, 28) for s, _ in GAME_PLAN]).arrange(DOWN, aligned_edge=LEFT, buff=0.42)
        fit(lines, 12.4).move_to([0, -0.45, 0])
        for line, (s, secrets) in zip(lines, GAME_PLAN):
            full = "• " + s
            bars = VGroup()
            for secret in secrets:
                i = full.index(secret)  # Text keeps one submobject per character, spaces included
                part = line[i:i + len(secret)]
                bars.add(Rectangle(width=part.width + 0.16, height=line.height + 0.12, stroke_width=0,
                                   fill_color="#0A0A0B", fill_opacity=1)
                         .move_to([part.get_center()[0], line.get_center()[1], 0]))
            self.play(AddTextLetterByLetter(line, time_per_char=0.02))
            self.play(*[GrowFromEdge(b, LEFT) for b in bars], run_time=0.25)
        word = txt("REDACTED", 80, RED, font=STAMP)
        stamp = VGroup(word, Rectangle(width=word.width + 0.5, height=word.height + 0.4).set_stroke(RED, 8))
        stamp.rotate(12 * PI / 180).move_to([3.6, -2.55, 0])
        self.play(FadeIn(stamp, scale=1.8), run_time=0.35)
        self.hold("Our game plan for tomorrow. Unfortunately our lawyers made some edits.")
        self.clear_page()


class Progress1(Page):
    def construct(self):
        f = np.load(DATA / "flight.npz")
        self.page("PROGRESS 1 · THE CALIBRATION THAT BACKFIRED")

        # Beat 1: measured noise against the firmware value (linear scale, so the gap is visible).
        base, scale = -2.7, 3.4 / 0.5
        bars = VGroup()
        for x, value, name in [(-3.6, 0.5, "firmware default"), (0.4, 0.011, "measured at rest")]:
            bar = Rectangle(width=1.4, height=value * scale, stroke_width=0, fill_opacity=1,
                            fill_color=C.muted if value > 0.1 else C.theory)
            bar.move_to([x, base + value * scale / 2, 0])
            bars.add(VGroup(bar, txt(f"{value} m/s²", 26).next_to(bar, UP, buff=0.15),
                            txt(name, 24).move_to([x, base - 0.35, 0])))
        caption = txt("Accelerometer noise, drone 60 s still on the floor", 22, C.muted).move_to([0, 1.75, 0])
        ratio = txt("45× smaller", 44, RED, BOLD).move_to([4.2, -1.7, 0])
        self.play(FadeIn(caption))
        for b in bars:
            self.play(GrowFromEdge(b[0], DOWN), FadeIn(b[1], b[2]), run_time=0.8)
        self.play(Write(ratio))
        self.hold("We put the drone on the floor for 60 s and measured the accelerometer noise: "
                  "0.011 m/s², 45 times smaller than what the firmware assumes.")

        axes = Axes(x_range=[0, 21, 5], y_range=[-1.25, 1.0, 0.25], x_length=6.6, y_length=4.4,
                    axis_config={"color": INK, "include_tip": False})
        axes.to_edge(LEFT, buff=0.9).set_y(-0.8)
        labels = VGroup(axes.get_axis_labels(x_label=txt("t [s]", 20), y_label=txt("position [m]", 20)),
                        ticks(axes, [0, 5, 10, 15, 20], [-1, -0.5, 0, 0.5, 1]))

        def trace(t, y, color, width=4):
            return axes.plot_line_graph(t, y, add_vertex_dots=False, line_color=color, stroke_width=width)["line_graph"]

        targets = VGroup(*[DashedVMobject(trace(f["target_t"], f[k], INK, 2), num_dashes=45)
                           for k in ("target_x", "target_y")])
        est_x, est_y = trace(f["t"], f["x"], C.theory), trace(f["t"], f["y"], C.highlight)
        legend = VGroup(txt("forward (x)", 20, C.theory), txt("sideways (y)", 20, C.highlight),
                        txt("dashed: target", 20, C.muted)).arrange(DOWN, aligned_edge=LEFT, buff=0.08)
        legend.move_to(axes.c2p(0.6, 0.8), aligned_edge=UL)
        self.play(FadeOut(bars, caption, ratio), Create(axes), FadeIn(labels, legend))
        self.play(Create(targets), run_time=0.8)
        self.play(Create(est_x), Create(est_y), run_time=4, rate_func=linear)
        i = int(np.argmax(f["x"]))
        peak = Dot(axes.c2p(f["t"][i], f["x"][i]), color=C.theory)
        peak_label = txt("0.81 m, target 0.61 m", 20, C.theory).next_to(peak, RIGHT, buff=0.2)
        j = int(np.argmin(np.abs(f["t"] - 17.5)))
        drift = Dot(axes.c2p(f["t"][j], f["y"][j]), color=C.highlight)
        drift_label = txt("drifts ≈ 0.9 m sideways", 20, C.highlight).move_to(axes.c2p(11.5, -1.08))
        self.play(FadeIn(peak, peak_label), FadeIn(drift, drift_label))
        self.hold("So we uploaded those values and flew 0.5 m forward and back. "
                  "It overshot by 20 cm and drifted almost a metre sideways.")

        lesson = VGroup(txt("REST ≠ FLIGHT", 34, INK, font=STAMP),
                        txt("At rest: only sensor noise.", 24),
                        txt("In flight: vibration, air,", 24),
                        txt("thrust → far more process", 24),
                        txt("noise.", 24),
                        txt("Tomorrow: firmware defaults.", 24, RED, BOLD)).arrange(DOWN, aligned_edge=LEFT, buff=0.22)
        fit(lesson, 4.6).to_edge(RIGHT, buff=0.4).set_y(-0.6)
        for line in lesson:
            self.play(FadeIn(line, shift=RIGHT * 0.2), run_time=0.4)
        self.hold("At rest the filter only sees sensor noise. In flight, vibration and air add "
                  "process noise. So tomorrow we race on the firmware defaults.")
        self.clear_page()


def ticks(axes, xs, ys):  # plain-text tick labels: manim's numbers need LaTeX packages this machine lacks
    return VGroup(*[txt(f"{v:g}", 18).next_to(axes.x_axis.n2p(v), DOWN, buff=0.12) for v in xs],
                  *[txt(f"{v:g}", 18).next_to(axes.y_axis.n2p(v), LEFT, buff=0.12) for v in ys])


class Progress2(Page):
    def construct(self):
        d = np.load(DATA / "dropout.npz")
        t, tof, z, sigma = d["t"], d["tof"], d["z"], d["sigma"]
        self.page("PROGRESS 2 · BLIND MEANS SLOW")

        # Beat 1: the real E5A recording, range sensor removed from 8 to 11 s.
        axes = Axes(x_range=[5, 20, 5], y_range=[0, 30, 10], x_length=7.2, y_length=4.2,
                    axis_config={"color": INK, "include_tip": False})
        axes.to_edge(LEFT, buff=0.9).set_y(-0.85)
        labels = VGroup(axes.get_axis_labels(x_label=txt("t [s]", 20), y_label=txt("height [cm]", 20)),
                        ticks(axes, [5, 10, 15, 20], [0, 10, 20, 30]))
        gap = Polygon(axes.c2p(8, 0), axes.c2p(11, 0), axes.c2p(11, 30), axes.c2p(8, 30),
                      stroke_width=0, fill_color=C.muted, fill_opacity=0.2)
        gap_label = txt("range\nsensor\noff", 16, C.muted).move_to(axes.c2p(9.5, 4.5))
        key = txt("dots: range sensor · line and band: filter ± σ", 18, C.muted).next_to(axes, UP, buff=0.3).align_to(axes, RIGHT)
        seen = ~((t >= 8) & (t <= 11)) & np.isfinite(tof)
        dots = VGroup(*[Dot(axes.c2p(a, 100 * b), radius=0.025, color=C.muted).set_opacity(0)
                        for a, b in zip(t[seen], tof[seen])])
        now = ValueTracker(5.0)
        dots.add_updater(lambda g: [dot.set_opacity(1.0 if a <= now.get_value() else 0)
                                    for dot, a in zip(g, t[seen])])

        def band():
            m = t <= now.get_value()
            if m.sum() < 2:
                return VMobject()
            up = [axes.c2p(a, 100 * (b + s)) for a, b, s in zip(t[m], z[m], sigma[m])]
            low = [axes.c2p(a, 100 * (b - s)) for a, b, s in zip(t[m], z[m], sigma[m])]
            return Polygon(*up, *low[::-1], stroke_width=0, fill_color=C.theory, fill_opacity=0.28)

        def estimate():
            m = t <= now.get_value()
            line = VMobject().set_stroke(C.theory, 4)
            if m.sum() >= 2:
                line.set_points_as_corners([axes.c2p(a, 100 * b) for a, b in zip(t[m], z[m])])
            return line

        readout = always_redraw(lambda: txt(f"σ = {100 * np.interp(now.get_value(), t, sigma):.2f} cm", 28,
                                            C.theory, BOLD).move_to(axes.c2p(15.5, 26)))
        self.play(Create(axes), FadeIn(labels, gap, gap_label, key))
        self.add(dots, always_redraw(band), always_redraw(estimate), readout)
        self.play(now.animate.set_value(11.0), run_time=4.5, rate_func=linear)
        self.wait(0.8)
        self.play(now.animate.set_value(20.0), run_time=2.5, rate_func=linear)
        self.hold("We switched the range sensor off for 3 s. The uncertainty grows from 1 mm to 7 cm, "
                  "the estimate wanders 10 cm, and one reading snaps it back.")

        k, hoop_at, gauge_at = 2.5, np.array([4.4, 0.3, 0]), np.array([4.4, -2.75, 0])
        sz = ValueTracker(float(sigma[t < 8][-1]))

        def clearance():  # E7: radius 0.50 - offset 0.03 - drone 0.10 - tracking 0.05 - 2.5 sigma_worst
            return 0.32 - 2.5 * max(0.04, sz.get_value())

        def speed():  # E7: min(clearance / (2 lag), sqrt(2 a d), 0.8), lag 0.25 s, a = 1 m/s2, d = 0.5 m
            return max(0.0, min(clearance() / 0.5, 1.0, 0.8))

        ring = Circle(radius=0.5 * k).set_stroke(INK, 10).move_to(hoop_at)
        safe = DashedVMobject(Circle(radius=0.32 * k), num_dashes=40).set_stroke(C.muted, 2).move_to(hoop_at)
        hoop_note = txt("hoop r = 0.5 m\ndashed: room after drone + margins", 16, C.muted).next_to(ring, UP, buff=0.12)
        drone = crazyflie(0.2 * k).move_to(hoop_at)
        blob = always_redraw(lambda: Ellipse(width=5 * 0.04 * k, height=max(5 * sz.get_value() * k, 0.03),
                                             stroke_color=C.highlight, stroke_width=3, fill_color=C.highlight,
                                             fill_opacity=0.25).move_to(hoop_at))
        clear_text = always_redraw(lambda: txt(f"clearance {clearance():.2f} m", 24).next_to(ring, DOWN, buff=0.15))
        dial = VGroup(Line(gauge_at + LEFT * 0.85, gauge_at + RIGHT * 0.85).set_stroke(C.muted, 1),
                      txt("0", 16, C.muted).move_to(gauge_at + LEFT * 1.05),
                      txt("0.8", 16, C.muted).move_to(gauge_at + RIGHT * 1.15))
        arc = Circle(radius=0.85).set_stroke(INK, 5).move_to(gauge_at)
        arc = VMobject().set_points(arc.get_points()[: len(arc.get_points()) // 2]).set_stroke(INK, 5)
        needle = always_redraw(lambda: Line(gauge_at, gauge_at + 0.75 * np.array(
            [np.cos(PI * (1 - speed() / 0.8)), np.sin(PI * (1 - speed() / 0.8)), 0])).set_stroke(RED, 6))
        speed_text = always_redraw(lambda: txt(f"{speed():.2f} m/s", 28, INK, BOLD).next_to(gauge_at, DOWN, buff=0.12))
        rule = txt("v = min(clearance / 2·lag, √(2·a·d), 0.8 m/s) · lag 0.25 s and horizontal σ 0.04 m assumed",
                   14, C.muted).set_y(-3.45).to_edge(LEFT, buff=0.5)
        self.play(Create(ring), Create(safe), FadeIn(hoop_note, drone, blob, clear_text),
                  Create(arc), FadeIn(dial, needle, speed_text, Dot(gauge_at, color=INK), rule))
        self.wait(0.5)
        self.play(sz.animate.set_value(float(sigma[t <= 11][-1])), run_time=3)
        self.hold("The uncertainty eats the hoop. Less clearance means slower: 0.44 down to 0.27 m/s.")

        self.clear_page()


class End(Page):
    def construct(self):
        self.page()
        self.next_slide(loop=True)
        fly(self, towing(banner("THANK YOU · SEE YOU TOMORROW!"), rtl=True), 7, rtl=True, y=(0.0, 0.0),
            rate_func=slow_middle)
