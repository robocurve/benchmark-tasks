"""Get an insulin pen ready for a 12-unit dose: shield off, the 2-unit safety shot with the needle up, then 12 units set by the dose window, the pen back in its tray.

A fluid-filled training pen of the KwikPen kind (0 to 60 units in steps of 1) lies in its tray with a 4 mm pen needle screwed on and its outer needle shield on. One hand
holds the pen; the other pulls the shield off, dials 2 units, pushes the dose knob home with the needle pointing up and holds it there 5 s (the IFU's safety shot), dials
12 units by the window, turning back if it overshoots, and the pen goes back in its tray.

Sim: the pen is one free mechanism. Its frame is the body, cartridge holder, needle hub and needle (the hub and needle are visual; the scorer reads the needle's place). The
dose knob turns (knob, about the pen's axis) and screws out (knob_out, along it): a stated coupling in code turns the knob's angle into units, with a click at every unit
(the knob's drive holds the nearest unit with at most CLICK_T), and moves the knob out RISE a unit. Pushing the knob back from below (a fingertip under its end face) moves it
in against PUSH_N (a constant 9.2 N over the stroke) and counts the units back down, as the pen delivers them; pushed out with the needle up and the shield off,
the 2 units show as a drop at the needle tip. The number sleeve turns and screws out with the knob, so the window shows the unit count as the
real pen does: even numbers printed, odd numbers as lines. The outer shield is a free body held on the hub in code (its pose written each step, its weight carried) until a
hand pinching it has drawn it 3 mm off along the pen's axis; then it is free.

    python demokit.py task.py OUT --mode full                    the full-score reference, filmed
    python demokit.py task.py OUT --no-record --mode noshot,short_hold,over,shieldon,shieldloose,holdend --hold 2.5   failure checks: 0.2, 0.2, 0.6, 0.0, 0.0, 0.6
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = ("Get the insulin pen ready for a 12-unit dose: take off the needle shield and put it in its tray; do the 2-unit safety shot: needle up, push the "
               "dose knob in until it stops and hold it there 5 s so a drop shows at the needle tip; then set 12 units by the dose window and leave the pen in its tray.")
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 2 s hold, so score() and success() agree
HOLD_STEPS = 40  # 2 s at 20 Hz

# -- the pen, in its own frame: axis along +z, needle up (+z), knob end down, dose window facing -y; origin at the body's middle ----------
UNIT_DEG = 18.0  # the knob turns this far a unit (20 units a turn; to measure)
RISE = 0.00033  # and screws out this far (m)
MAX_U = 60
SHOT_U, DOSE_U = 2, 12
CLICK_T = 0.04  # N m: the click's hold
STOP_T = 0.4  # N m: the stops at 0 and 60 units
PUSH_N = 9.2  # N: the knob's resistance to a push, over the whole stroke: a standard KwikPen's plateau injection force at 6 U/s, 9.17 N (max 11.95 N; van der Burg 2011)
PUSH_PRE = 0.004  # m: the drive's preload while a push is on (5000 N/m x 4 mm = 20 N, above the 9.2 N cap)
R_KNOB, KNOB0, KNOB1 = 0.009, -0.080, -0.066  # the dose knob: radius, its end face and its top at 0 units
R_BODY, BODY1 = 0.0085, 0.015
R_SLEEVE = 0.0072
Z_WIN, WIN_H, WIN_HALF = -0.053, 0.008, 24.0  # the window's middle, height (m) and half-width (degrees about the axis)
R_HOLDER, HOLDER1 = 0.0068, 0.064
HUB0, HUB1, R_HUB = 0.064, 0.076, 0.0055
NEEDLE1 = 0.080  # the needle's tip (4 mm out of the hub)
SH0, SH1, R_SH = 0.066, 0.086, 0.006  # the outer shield: from its open end to its closed end
PEN_MASS, SHIELD_MASS = 0.020, 0.002  # the shield is 0.5 g; 2 g keeps it steady (stated)
Q_TRAY = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # xyzw: a quarter turn about x lays the pen along y, needle toward -y, window down

PEN_TRAY = np.array([0.24, 0.12])  # the pen tray's middle
SHIELD_TRAY = np.array([0.30, -0.11])
AXIS_Z = 0.003 + R_KNOB + 0.0005  # the pen's axis above the table, lying in its tray
PEN0 = np.array([PEN_TRAY[0], PEN_TRAY[1], TOP + AXIS_Z])

LOOKS = {"body": m.Look(color=m.rgb(0.42, 0.44, 0.47), roughness=0.45), "knob": m.Look(color=m.rgb(0.20, 0.21, 0.23), roughness=0.5),
         "band": m.Look(color=m.rgb(0.55, 0.12, 0.20), roughness=0.5), "sleeve": m.Look(color=m.rgb(0.93, 0.93, 0.91), roughness=0.6),
         "ink": m.Look(color=m.rgb(0.03, 0.03, 0.03), roughness=0.7), "holder": m.Look(glass=(0.92, 0.95, 0.97)),
         "liquid": m.Look(color=m.rgb(0.85, 0.88, 0.90), roughness=0.1, opacity=0.35), "drop": m.Look(color=m.rgb(0.75, 0.88, 1.0), roughness=0.05, metallic=0.2), "rubber": m.Look(color=m.rgb(0.25, 0.25, 0.27), roughness=0.8),
         "cap": m.Look(color=m.rgb(0.75, 0.76, 0.78), metallic=0.8, roughness=0.35), "hub": m.Look(color=m.rgb(0.94, 0.94, 0.94), roughness=0.5),
         "steel": m.Look(color=m.rgb(0.8, 0.8, 0.82), metallic=1.0, roughness=0.25), "shield": m.Look(color=m.rgb(0.95, 0.95, 0.95), roughness=0.55),
         "tray": m.Look(color=m.rgb(0.80, 0.83, 0.86), roughness=0.6), "tray2": m.Look(color=m.rgb(0.62, 0.78, 0.86), roughness=0.6)}


def _cyl(r, z0, z1, n=32):
    return m.cylinder(r, z1 - z0, (0.0, 0.0, (z0 + z1) / 2), axis="z", segments=n)


def _on_sleeve(u, v, su, sv, r, ang0, z0, depth=0.0002):
    """A flat mark on the sleeve: its middle `u` along the turn (m, toward +angle) and `v` along the axis from (ang0 degrees, z0), su x sv in size."""
    b = m.box((su, depth, sv), (0.0, -(r + depth / 2), z0 + v))  # made facing -y (angle -90), then turned to its place
    return b.turned(ang0 + 90.0 + math.degrees(u / r), "z")


_SEG = {"a": (0.0, 0.5, 1), "d": (0.0, -0.5, 1), "g": (0.0, 0.0, 1), "f": (-0.5, 0.25, 0), "b": (0.5, 0.25, 0), "e": (-0.5, -0.25, 0), "c": (0.5, -0.25, 0)}
_DIGITS = {"0": "abcdef", "1": "bc", "2": "abged", "3": "abgcd", "4": "fgbc", "5": "afgcd", "6": "afgedc", "7": "abc", "8": "abcdefg", "9": "abcfgd"}


def _number_marks(n: int, ang0: float, z0: float, h=0.0036, w=0.0021, s=0.00045, gap=0.0006):
    """Unit n as the real pen shows it: even numbers in digits h tall, odd ones as a short line."""
    if n % 2:
        return [_on_sleeve(-0.0012, 0.0, 0.0022, s, R_SLEEVE, ang0, z0)]
    text = str(n)
    width = len(text) * w + (len(text) - 1) * gap
    out = []
    for k, ch in enumerate(text):
        uc = -width / 2 + w / 2 + k * (w + gap)
        for seg in _DIGITS[ch]:
            x, y, horiz = _SEG[seg]
            su, sv = (w, s) if horiz else (s, h / 2)
            out.append(_on_sleeve(uc + x * (w - s), y * (h - s), su, sv, R_SLEEVE, ang0, z0))
    return out


def _window_band():
    """The body's shell round the window: boxes in a ring, leaving the window open toward -y."""
    out, n = [], 40
    for k in range(n):
        a0 = -90.0 + WIN_HALF + k * (360.0 - 2 * WIN_HALF) / n
        a1 = a0 + (360.0 - 2 * WIN_HALF) / n
        mid = math.radians((a0 + a1) / 2)
        chord = 2 * R_BODY * math.sin(math.radians(a1 - a0) / 2) + 0.0003
        b = m.box((chord, 0.0012, WIN_H), (0.0, -(R_BODY - 0.0006), Z_WIN))
        out.append(b.turned(math.degrees(mid) + 90.0, "z"))
    return m.join(*out)


def _pen() -> m.Part:
    lo, hi = Z_WIN - WIN_H / 2, Z_WIN + WIN_H / 2
    frame = [("body_lo", _cyl(R_BODY, KNOB1, lo), "body"), ("window_band", _window_band(), "body"), ("body_hi", _cyl(R_BODY, hi, BODY1 - 0.012), "body"),
             ("band", _cyl(R_BODY + 0.0002, BODY1 - 0.012, BODY1), "band"), ("holder", _cyl(R_HOLDER, BODY1, HOLDER1 - 0.004), "holder"),
             ("tip", _cyl(0.0045, HOLDER1 - 0.004, HUB0), "holder"), ("cartridge", _cyl(0.0048, BODY1 + 0.002, HOLDER1 - 0.006, 24), "liquid"),
             ("plunger", _cyl(0.0049, BODY1 + 0.002, BODY1 + 0.008, 24), "rubber"), ("crimp", _cyl(0.0042, HOLDER1 - 0.006, HOLDER1 - 0.002, 24), "cap"),
             ("hub", _cyl(R_HUB, HUB0, HUB1, 24), "hub"), ("needle", _cyl(0.0003, HUB1, NEEDLE1, 8), "steel"),
             ("drop", m.sphere(0.0016, (0.0, 0.0, NEEDLE1 + 0.0012)), "drop", True)]  # the drop the safety shot pushes out, shown once it flows
    colliders = [m.col_hull("body_c", _cyl(R_BODY, KNOB1 + 0.0005, BODY1, 24)), m.col_hull("holder_c", _cyl(R_HOLDER, BODY1, HUB0 - 0.001, 24))]
    marks = []
    for n in range(MAX_U + 1):
        marks += _number_marks(n, -90.0 - n * UNIT_DEG, Z_WIN + n * RISE)
    knob_parts = [("knob", _cyl(R_KNOB, KNOB0, KNOB1), "knob")]
    sleeve_parts = [("sleeve", _cyl(R_SLEEVE, KNOB1, Z_WIN + MAX_U * RISE + 0.006), "sleeve"), ("numbers", m.join(*marks), "ink")]
    out = m.slider("knob_out", "Z", -MAX_U * RISE - 0.002, 0.0005, [], [], mass=0.002, spring=5000.0, damping=5.0, friction=0.0, effort=PUSH_N)
    knob = m.hinge("knob", "Z", (0.0, 0.0, 0.0), -math.inf, math.inf, knob_parts, [m.col_hull("knob_c", _cyl(R_KNOB, KNOB0, KNOB1, 24)), m.col_hull("sleeve_c", _cyl(R_SLEEVE, KNOB1, KNOB1 + 0.02, 24))],
                   mass=0.003, spring=0.3, damping=0.0005, friction=0.0, effort=CLICK_T, parent="knob_out", inertia=(2e-7, 2e-7, 1.2e-7))
    # the number sleeve: screws out with the knob and turns to show the count; while the knob is pushed in it turns back, so the window counts down to 0
    # (on the real pen the button does not turn as it goes in; the sleeve inside does)
    sleeve = m.hinge("sleeve", "Z", (0.0, 0.0, 0.0), -math.inf, math.inf, sleeve_parts, [], mass=0.001, spring=0.5, damping=0.001, friction=0.0, effort=0.2,
                     parent="knob_out", inertia=(1e-7, 1e-7, 6e-8))
    return m.mechanism("pen", tuple(PEN0), [out, knob, sleeve], LOOKS, frame_parts=frame, frame_colliders=colliders, free_mass=PEN_MASS, friction=(0.9, 0.8), start_quat=Q_TRAY,
                       frame_inertia=(PEN_MASS * 0.16 ** 2 / 12, PEN_MASS * 0.16 ** 2 / 12, PEN_MASS * R_BODY ** 2 / 2))


def _shield() -> m.Part:
    parts = [("shell", _cyl(R_SH, SH0, SH1 - 0.001, 24), "shield"), ("end", _cyl(R_SH - 0.0005, SH1 - 0.001, SH1, 24), "shield")]
    ribs = [m.box((0.0008, 0.0016, SH1 - SH0 - 0.004), (0.0, -(R_SH + 0.0006), (SH0 + SH1) / 2)).turned(a, "z") for a in range(0, 360, 45)]
    parts.append(("ribs", m.join(*ribs), "shield"))
    pos = PEN0 + m.quat_matrix(Q_TRAY) @ np.array([0.0, 0.0, 0.0])
    return m.free("shield", tuple(pos), parts, [m.col_hull("shield_c", _cyl(R_SH, SH0, SH1, 16))], LOOKS, mass=SHIELD_MASS, friction=(0.9, 0.8), start_quat=Q_TRAY)


def _trays() -> m.Part:
    parts, cols = [], []

    def box(name, size, center, look):
        parts.append((name, m.box(size, center), look))
        cols.append(m.col_box(f"{name}_c", size, center))

    x, y = PEN_TRAY
    box("pen_floor", (0.05, 0.21, 0.003), (x, y, 0.0015), "tray")
    for k, s in enumerate((-1, 1)):
        box(f"pen_rail{k}", (0.004, 0.21, 0.003), (x + s * 0.022, y, 0.0045), "tray")  # low rails: the open fingers pass over them
    x, y = SHIELD_TRAY
    box("shield_floor", (0.07, 0.07, 0.004), (x, y, 0.002), "tray2")
    for k, (dx, dy, sx, sy) in enumerate(((0.032, 0.0, 0.006, 0.07), (-0.032, 0.0, 0.006, 0.07), (0.0, 0.032, 0.07, 0.006), (0.0, -0.032, 0.07, 0.006))):
        box(f"shield_wall{k}", (sx, sy, 0.016), (x + dx, y + dy, 0.008), "tray2")
    return m.fixed("trays", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.8, 0.7))


OBJECTS = [_pen(), _shield(), _trays()]


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


class Pen:
    """The dose knob's coupling, the shield's hold, and the scorer's trial-long record."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.steps, self.prev, self.cum, self.offset = 0, None, 0.0, 0.0
        self.units, self.peak = 0, 0
        self.latched, self.pinch_at = True, None
        self.pushing, self.push_from, self.push_up_ok, self.push_out = False, 0, True, 0.0
        self.home_s, self.shot_ok, self.wasted, self.touched, self.dropped = 0.0, False, False, False, False
        self.hold, self.last, self.full, self.drop = 0, {}, False, False
        if env is not None:
            try:
                m.set_visible(f"{m.ENV_PATH}/pen/frame/visual/drop", False)
            except Exception:
                pass

    # -- the knob ----------------------------------------------------------------------------------------
    def _knob(self, env, s, tips, axis_up):
        import torch

        asset = env.unwrapped.scene["pen"]
        a = s.joint("knob")
        if self.prev is not None:
            self.cum += _wrap(a - self.prev)
        self.prev = a
        out = -s.joint("knob_out")  # how far the knob stands out (m)
        # a fingertip under the knob's end face, within its radius: a push from below
        local = s.to_local(tips)
        under = bool(np.any((np.hypot(local[:, 0], local[:, 1]) < R_KNOB + 0.003) & (local[:, 2] < KNOB0 - out + 0.012) & (local[:, 2] > KNOB0 - out - 0.006)))
        dial = int(round((self.cum - self.offset) / math.radians(UNIT_DEG)))
        if under and out < (self.units - 0.6) * RISE:  # pushed in: the pen delivers, the count comes down with the knob
            if not self.pushing:
                self.pushing, self.push_from, self.push_up_ok, self.push_out = True, self.units, True, self.units * RISE
            self.units = max(0, int(math.ceil(out / RISE - 0.4)))
            self.offset = self.cum - self.units * math.radians(UNIT_DEG)
            if axis_up < 0.0:
                self.wasted = True  # a push with the needle below horizontal
            if axis_up < math.cos(math.radians(30.0)):
                self.push_up_ok = False
        elif not under:
            self.units = int(np.clip(dial, 0, MAX_U))
            if dial != self.units:
                self.offset = self.cum - self.units * math.radians(UNIT_DEG)  # turned past a stop: the stop holds
            if self.pushing:
                self.pushing = False
        self.peak = max(self.peak, self.units)
        # the click: the knob's drive holds the nearest unit, from its present angle
        jk = asset.find_joints("knob_joint")[0][0]
        jo = asset.find_joints("knob_out_joint")[0][0]
        want = self.offset + self.units * math.radians(UNIT_DEG)
        stop = (dial < 0 or dial > MAX_U) and not under
        asset.write_joint_effort_limit_to_sim(torch.tensor([[STOP_T if stop else CLICK_T]], device=asset.device), joint_ids=[jk])
        asset.set_joint_position_target(torch.tensor([[a + (want - self.cum)]], device=asset.device), joint_ids=[jk])
        # out: held at its units; while a push is on, held where the push began, so the push meets PUSH_N all the way in
        # while a push is on, the drive's rest sits PUSH_PRE further out than where the push began: its spring is past its cap from the first moment, so the
        # push meets PUSH_N, the pen's force, over the whole stroke
        asset.set_joint_position_target(torch.tensor([[-(self.push_out + PUSH_PRE if self.pushing else self.units * RISE)]], device=asset.device), joint_ids=[jo])
        js = asset.find_joints("sleeve_joint")[0][0]  # the window shows the count: the sleeve at the count's angle
        asset.set_joint_position_target(torch.tensor([[self.units * math.radians(UNIT_DEG)]], device=asset.device), joint_ids=[js])
        m._TOUCHED.add(("pen", "knob"))
        m._TOUCHED.add(("pen", "knob_out"))
        return out, under

    # -- the shield ------------------------------------------------------------------------------------------
    def _shield(self, env, s, tips):
        import torch

        if not self.latched:
            return
        asset = env.unwrapped.scene["shield"]
        origin = env.unwrapped.scene.env_origins[0]
        pinched, mid = False, None
        for hand in (tips[:12], tips[12:]):  # either hand: one finger's six points, then the other's
            opening = float(np.linalg.norm(hand[:6].mean(axis=0) - hand[6:].mean(axis=0))) / 2 - 0.0045
            hm = s.to_local(hand.mean(axis=0))[0]
            if np.hypot(hm[0], hm[1]) < 0.02 and SH0 - 0.005 < hm[2] < SH1 + 0.005 and R_SH - 0.003 < opening < R_SH + 0.0015:  # closed on it
                pinched, mid = True, hm
                break
        if pinched:
            zmid = float(mid[2])
            self.pinch_at = zmid if self.pinch_at is None else self.pinch_at
            if zmid - self.pinch_at > 0.003:  # drawn 3 mm off along the axis: it comes away
                self.latched = False
                m.push(env, "shield", (0.0, 0.0, 0.0))
                return
        else:
            self.pinch_at = None
        pose = torch.tensor([[*(s.pos + np.asarray(origin.cpu())), *s.quat_xyzw]], dtype=torch.float32, device=asset.device)
        asset.write_root_pose_to_sim(pose)
        vel = np.r_[s.lin_vel, s.ang_vel]
        asset.write_root_velocity_to_sim(torch.tensor([vel.tolist()], dtype=torch.float32, device=asset.device))
        m.push(env, "shield", (0.0, 0.0, SHIELD_MASS * 9.81))  # carried with the pen: its weight held

    def step(self, env):
        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        s = m.state(env, "pen")
        tips = m.fingertips(env)
        R = s.rotation()
        axis_up = float(R[2, 2])  # the needle's direction against straight up
        out, under = self._knob(env, s, tips, axis_up)
        self._shield(env, s, tips)
        tip = s.to_world(np.array([0.0, 0.0, NEEDLE1]))[0]
        shc = m.state(env, "shield").to_world(np.array([0.0, 0.0, (SH0 + SH1) / 2]))[0]  # the shield's middle (its origin is the pen's, as authored)
        shield_clear = (not self.latched) and float(np.linalg.norm(shc - tip)) > 0.03  # off the needle, not just loosened
        if self.pushing or self.home_s > 0.0:  # needle up through the whole push and the hold, every step
            if axis_up < 0.0:
                self.wasted = True
            if axis_up < math.cos(math.radians(30.0)):
                self.push_up_ok = False
        # the safety shot: dialled to exactly 2, pushed to 0 with the needle within 30 degrees of up, held home 5 s
        home = out < 0.0002 and under and self.units == 0
        if home and self.push_from == SHOT_U and self.push_up_ok and not self.wasted and shield_clear:  # the shield off the needle first
            if not self.drop:  # the 2 units pushed out through the needle, pointing up: a drop shows at its tip
                self.drop = True
                try:
                    m.set_visible(f"{m.ENV_PATH}/pen/frame/visual/drop", True)
                except Exception:
                    pass
            self.home_s += dt
            if self.home_s >= 5.0 and self.drop:
                self.shot_ok = True  # the drop shown at the needle, the knob held home 5 s
        elif not home:
            self.home_s = 0.0
        # the needle: touching a finger, or the table (its tip within 4 mm of it), once the shield is off
        tip = s.to_world(np.array([0.0, 0.0, NEEDLE1]))[0]
        base = s.to_world(np.array([0.0, 0.0, HUB1]))[0]
        if not self.latched:
            seg = np.linspace(base, tip, 5)
            near = float(np.min(np.linalg.norm(tips[:, None, :] - seg[None, :, :], axis=2)))
            if near < 0.002 or tip[2] - TOP < 0.004:
                self.touched = True
        # the pen dropped: on the table outside its tray, or below it
        in_tray = abs(s.pos[0] - PEN_TRAY[0]) < 0.03 and abs(s.pos[1] - PEN_TRAY[1]) < 0.06 and s.pos[2] - TOP < 0.03
        if (s.pos[2] - TOP < 0.03 and not in_tray and self.steps > 10) or s.pos[2] < TOP - 0.05:
            self.dropped = True
        shield_in = (not self.latched) and abs(shc[0] - SHIELD_TRAY[0]) < 0.03 and abs(shc[1] - SHIELD_TRAY[1]) < 0.03 and shc[2] - TOP < 0.03
        lying = abs(axis_up) < math.sin(math.radians(15.0))
        along = s.to_world(np.array([[0.0, 0.0, z] for z in np.linspace(KNOB0, NEEDLE1, 9)]))  # points along the whole pen
        clear = bool(np.min(np.linalg.norm(tips[:, None, :] - along[None, :, :], axis=2)) > 0.05)  # every fingertip of both hands 5 cm from it
        self.last = {"t": round(self.steps * dt, 2), "z_mm": round((s.pos[2] - TOP) * 1000, 1), "units": self.units, "peak": self.peak, "out_mm": round(out * 1000, 2), "under": under, "axis_up": round(axis_up, 3),
                     "latched": self.latched, "shield_in": bool(shield_in), "home_s": round(self.home_s, 2), "drop": self.drop, "shot_ok": self.shot_ok, "wasted": self.wasted,
                     "touched": self.touched, "dropped": self.dropped, "in_tray": bool(in_tray and lying), "clear": clear}
        self.hold = self.hold + 1 if _tier(self.last) == 1.0 else 0
        self.full = self.hold >= HOLD_STEPS
        self.last["hold"] = self.hold
        step_file = os.environ.get("TASK_STEP_FILE")
        if step_file:  # the simulated time, for the Dream-1 runner's session watch
            with open(step_file, "w") as f:
                f.write(f"{self.steps * dt:.2f}")
        if self.steps % 5 == 0:
            print("[pen] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last}), flush=True)


def _tier(r) -> float:
    if not r or r["dropped"]:
        return 0.0
    total = 0.2 * r["shield_in"] + 0.4 * r["shot_ok"] + 0.4 * (r["shot_ok"] and r["units"] == DOSE_U and r["in_tray"] and r["clear"])
    if r["wasted"]:
        total = min(total, 0.2)
    if r["touched"]:
        total = min(total, 0.5)
    return round(total, 3)


PEN = Pen()
m.on_step(PEN.step)
m.on_reset(PEN.reset)


def score(objects) -> float:
    if PEN.full:
        return 1.0
    t = _tier(PEN.last)
    return 0.6 if t >= 1.0 else t  # every part met, not yet held 2 s


def success(objects) -> bool:
    return PEN.full


# -- the scripted demo ----------------------------------------------------------------------------

def demo_script(ctx):
    env = ctx.emb._env
    L, R = ctx.left, ctx.right
    pen = lambda: m.state(env, "pen")  # noqa: E731
    g_pick = d.down(-90.0)  # fingers closing along x across the body
    g_hold = d.tilted(90.0, -90.0)  # the same grip pitched a quarter turn about x: fingers level toward -y, the needle up
    g_act = d.tilted(90.0, 90.0)  # the right hand: fingers level toward +y, closing along x
    if ctx.mode == "scene":
        yield from L.wait(1.0)
        print("[pen] END " + json.dumps({"mode": ctx.mode, **PEN.last}), flush=True)
        return

    # the left hand picks the pen at its middle and stands it needle up
    c = pen().pos
    yield from L.move(c + np.array([0.0, 0.0, 0.08]), grip=g_pick, finger=d.FINGER_OPEN, t=2.0, label="over the pen")
    yield from L.move(c + np.array([0.0, 0.0, 0.0095]), t=1.0, label="down round the pen")
    yield from L.close(2 * R_BODY, force=40.0, label="pinch the pen")
    yield from L.move(L.at() + np.array([0.0, 0.0, 0.15]), t=1.5, label="lift the pen")
    yield from L.turn(-90.0, axis=(1.0, 0.0, 0.0), t=1.5, label="needle up")
    held = pen().pos - L.at()

    def pen_to(goal, t, label):
        yield from L.move(np.asarray(goal) - held, grip=g_hold, t=t, label=label)
        yield from L.servo(np.asarray(goal), tol=0.001, timeout=1.5, read=lambda: pen().pos, label=label + " (servo)")

    yield from pen_to((0.26, 0.0, TOP + 0.155), 1.5, "the pen up for the shield")
    # the right hand pinches the shield and draws it off along the axis, then drops it in its tray
    if ctx.mode != "shieldon":  # shieldon, a failure check: the safety shot with the shield still on: nothing flows out, no drop
        sh = pen().to_world(np.array([0.0, 0.0, (SH0 + SH1) / 2]))[0]  # the shield's middle
        yield from R.move(sh + np.array([0.0, -0.06, 0.0]), grip=g_act, finger=d.FINGER_OPEN, t=2.0, label="beside the shield")
        yield from R.move(sh + np.array([0.0, -0.002, 0.0]), t=1.0, label="round the shield")
        yield from R.close(2 * R_SH, force=20.0, label="pinch the shield")
        if ctx.mode == "shieldloose":  # shieldloose, a failure check: the shield drawn 4 mm, let go, left on the needle
            yield from R.move(R.at() + np.array([0.0, 0.0, 0.004]), t=0.6, label="draw the shield 4 mm")
            yield from R.open(label="let go of the shield, still on the needle")
            yield from R.move(R.at() + np.array([0.0, -0.06, 0.0]), t=1.0, label="away from the shield")
        else:
            yield from R.move(R.at() + np.array([0.0, 0.0, 0.025]), t=1.0, label="pull the shield off")
            yield from R.move(R.at() + np.array([0.0, -0.05, 0.02]), t=1.0, label="away from the needle")
            yield from R.move(np.array([SHIELD_TRAY[0], SHIELD_TRAY[1] - 0.01, TOP + 0.10]), t=1.5, label="over the shield tray")
            yield from R.open(label="drop the shield")

    def knob_mid():
        return pen().to_world(np.array([0.0, 0.0, (KNOB0 + KNOB1) / 2 - PEN.last.get("out_mm", 0.0) / 1000.0]))[0]

    def dial(target, label):
        """Turn the knob in strokes of up to 50 degrees, reading the window between them; back if it overshoots."""
        for _ in range(12):
            u = PEN.units
            if u == target:
                return
            n = target - u
            deg = float(np.clip(n * UNIT_DEG + math.copysign(6.0, n), -50.0, 50.0))  # the click takes the nearest unit: a little over
            phi0 = 40.0 if deg > 0 else 90.0
            g0 = d.tilted(90.0, phi0)
            k = knob_mid()
            ap = g0.rot[:, 2]
            yield from R.move(k - 0.04 * ap, grip=g0, finger=d.FINGER_OPEN, t=0.7, label=f"{label}: beside the knob")
            yield from R.move(k + 0.002 * ap, t=0.4, label=f"{label}: round the knob")
            yield from R.close(2 * R_KNOB, force=40.0, t=0.3, label=f"{label}: pinch the knob")
            yield from R.turn(deg, axis=(0.0, 0.0, 1.0), t=0.7, label=f"{label}: turn the knob")
            yield from R.open(t=0.25, label=f"{label}: let go of the knob")
            yield from R.move(R.at() - 0.04 * R.approach_axis(), t=0.4, label=f"{label}: back from the knob")
            print(f"[pen] {label}: window reads {PEN.units}", flush=True)

    yield from pen_to((0.26, 0.0, TOP + 0.225), 1.5, "the pen up for the knob")
    def shot(hold_s):
        """The safety shot: closed fingers under the knob's end face, pushed up until the knob is home, held there."""
        end = pen().to_world(np.array([0.0, 0.0, KNOB0 - PEN.last["out_mm"] / 1000.0]))[0]
        under = end + np.array([0.0, -0.006, -0.018 - 0.004])  # the closed fingers' upper edge (18 mm above the TCP) 4 mm under the end face
        yield from R.move(under + np.array([0.0, -0.05, 0.0]), grip=g_act, finger=d.FINGER_CLOSED, t=1.0, label="below the knob")
        yield from R.move(under, t=0.8, label="under the knob")
        knob_end = lambda: pen().to_world(np.array([0.0, 0.0, KNOB0 - PEN.last["out_mm"] / 1000.0]))[0]  # noqa: E731
        home = lambda s: pen().to_world(np.array([0.0, 0.0, KNOB0 + 0.0003]))[0]  # noqa: E731  the knob home, and a little on (it stops there)
        goal = pen().pos.copy()  # the left hand holds the pen here against the push: the arm is soft and yields otherwise
        yield from ctx.together(L.servo(goal, tol=0.0005, timeout=3.0, gain=0.5, read=lambda: pen().pos, label="hold the pen against the push"),
                                R.servo(home, tol=0.0003, timeout=3.0, gain=0.5, read=knob_end, label="push the knob home"))
        yield from ctx.together(L.servo(goal, tol=0.0005, timeout=hold_s, gain=0.5, read=lambda: pen().pos, label="hold the pen against the push"),
                                R.servo(home, tol=0.0003, timeout=hold_s, gain=0.5, read=knob_end, label="hold the knob home"))
        print(f"[pen] safety shot: {json.dumps(PEN.last)}", flush=True)
        R.trim = np.zeros(3)  # the push's servo shift (it pressed in to meet 9.2 N) must not carry into the dialling
        yield from R.move(R.at() + np.array([0.0, -0.05, -0.02]), t=0.8, label="away from the knob")
        L.trim = np.zeros(3)  # nor the left's shift, holding the pen against the push
        yield from L.servo(goal, tol=0.001, timeout=2.0, gain=0.5, read=lambda: pen().pos, label="the pen back where it was before the push")

    yield from dial(SHOT_U, "dial 2")
    if ctx.mode != "noshot":  # noshot, a failure check: straight on to 12
        yield from shot(2.0 if ctx.mode == "short_hold" else 5.5)  # short_hold, a failure check: held 2 s, not 5
    yield from dial(DOSE_U + (1 if ctx.mode == "over" else 0), "dial 12")  # over, a failure check: left at 13
    # the right hand out of the way; the left lays the pen back in its tray
    yield from R.move(np.array([0.30, -0.20, TOP + 0.20]), grip=d.down(0.0), t=1.5, label="right clear")
    yield from pen_to((PEN_TRAY[0], PEN_TRAY[1], TOP + 0.16), 1.5, "the pen over its tray")
    yield from L.turn(90.0, axis=(1.0, 0.0, 0.0), t=1.5, label="lay the pen down")
    c = pen().pos
    yield from L.move(L.at() + np.array([PEN0[0] - c[0], PEN0[1] - c[1], PEN0[2] + 0.002 - c[2]]), t=1.5, label="into the tray")
    yield from L.open(label="let go of the pen")
    if ctx.mode == "holdend":  # holdend, a failure check: the left hand moves straight to the pen's knob end, 6.7 cm from its middle, and pinches it
        kend = pen().to_world(np.array([0.0, 0.0, (KNOB0 + KNOB1) / 2]))[0]
        yield from L.move(kend + np.array([0.0, 0.0, 0.03]), t=0.8, label="over the knob end")
        yield from L.move(kend + np.array([0.0, 0.0, 0.002]), t=0.8, label="fingers either side of the knob end")
        yield from L.close(2 * R_KNOB, force=10.0, label="pinch the knob end")
    else:
        yield from L.move(L.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="left clear")
    yield from L.wait(2.5)
    print("[pen] END " + json.dumps({"mode": ctx.mode, **PEN.last}), flush=True)
