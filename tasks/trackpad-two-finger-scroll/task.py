"""Scroll a document with a two-finger slide on a Magic Trackpad until the red line rests in the marked band on the monitor, without clicking.

An Apple Magic Trackpad (USB-C, MXK93AM/A: 160.0 x 114.9 mm, a wedge 4.9 mm thick at the front and 10.9 mm at the back, 230 g) lies on the table in
front of the right arm, its thick edge away from the arms. A 24-inch monitor behind it shows a document: grey text lines, one red line, and a pale
blue band at mid-screen. The red line starts 105 mm below the band's centre.
The trackpad reads the robot the way macOS does: a touch is a conductive nib on a finger (links tip_left and tip_right) pressing the glass's
top with at least 0.05 N (PhysX contact report); every link of either arm counts toward the click force, and only tip_* links count as touches. With exactly two touches the page moves with their mean position along the pad's depth, 1.5 mm of
page per mm of travel ("natural" direction: fingers slide away, the page moves up), and after the touches lift it coasts on: it keeps its speed of
the last 0.1 s and slows with a 0.35 s time constant. One touch that travels moves the pointer, not the page. A total press over 0.8 N is a click (env SC_CLICK sets it).

    python demokit.py task.py OUT --no-record --mode full,none,one_tip,flick,press,rigid,brush --render 320x176
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
"""

from __future__ import annotations

import dataclasses
import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Scroll the document with a two-finger slide on the trackpad until the red line rests inside the blue band on the screen. Do not click."
SUCCESS_HOLD_S = 0.0  # the scorer reads the end state
SCORE_OVER = "end"
TOP = d.TABLE_TOP


def _e(k, v):
    return float(os.environ.get(k, v))


# -- the scene: Magic Trackpad (USB-C) and a monitor, one fixed part ---------------------------------------------------------------------------
# Part frame: x away from the arms, y to the left, z up; origin on the table top at the trackpad's centre.
PAD_W, PAD_D = 0.160, 0.1149  # Apple 121932: width 160.0 mm, depth 114.9 mm
PAD_H0, PAD_H1 = 0.0049, 0.0109  # height 4.9 mm at the front edge, 10.9 mm at the back
SLOPE = math.atan2(PAD_H1 - PAD_H0, PAD_D)  # the glass rises 3.0 degrees away from the user
PX, PY = _e("SC_PX", 0.33), _e("SC_PY", -0.16)  # the trackpad's centre on the table


def glass_z(x_local):
    """The glass's height over the table at depth x_local (part frame, metres)."""
    return PAD_H0 + (x_local + PAD_D / 2) / PAD_D * (PAD_H1 - PAD_H0)


# The monitor: a 24-inch 16:9 panel (active area 531.4 x 298.9 mm), facing the arms, on a stand.
MON_X, MON_Y = 0.30, 0.15  # its screen's centre, relative to the trackpad's centre
SCR_W, SCR_H = 0.5314, 0.2989
SCR_Z0 = 0.11  # the active area's bottom edge over the table
BEZ = 0.01
GAIN = 1.5  # page mm per mm of two-finger travel
TAU = 0.35  # the coast's time constant (s)
BAND_Z = SCR_H / 2  # the band's centre, from the active area's bottom (m)
BAND_HALF = 0.012  # the band: 24 mm tall on the screen (16 mm of finger travel)
RED0 = BAND_Z - 0.105  # the red line starts 105 mm below the band's centre: 70 mm of finger travel
TOUCH_N = 0.05  # a touch presses at least this hard (N)
CLICK_N = _e("SC_CLICK", 0.8)  # a total press over this clicks (env SC_CLICK; Apple publishes no force; the scorer passes for 0.5 to 3 N, the press window being 0.12 to 0.4 N)
PAD_K = _e("SC_K", 600.0)  # contact stiffness (N/m): the silicone stylus nibs' give, 0.6 N/mm (an estimate; the trial measures it, a nib pressed on a kitchen scale)
PAD_C = _e("SC_C", 10.0)  # its damping (N s/m)
MERGE = 0.012  # two contacts closer than this read as one touch, as on a capacitive pad
DRAG_VOID = 0.005  # one touch that travels this far in one stroke moves the pointer
REST_S = 0.5  # the page is at rest when it has moved under 1 mm over this long
MOVED = 0.010  # the first tier: a two-finger gesture has scrolled the page 10 mm toward the band
NORMAL_COS = 0.9  # a contact presses the glass's top when its normal is within about 25 degrees of the glass's normal

LOOKS = {"pad": m.Look(m.rgb(0.93, 0.93, 0.92), roughness=0.25), "edge": m.Look(m.rgb(0.78, 0.79, 0.80), roughness=0.3, metallic=0.8),
         "foot": m.Look(m.rgb(0.2, 0.2, 0.2), roughness=0.8), "mon": m.Look(m.rgb(0.05, 0.05, 0.06), roughness=0.4),
         "page": m.Look(m.rgb(0.97, 0.97, 0.97), roughness=0.6, emissive=m.rgb(0.75, 0.75, 0.75)),
         "text": m.Look(m.rgb(0.35, 0.35, 0.38), roughness=0.6, emissive=m.rgb(0.12, 0.12, 0.13)),
         "red": m.Look(m.rgb(0.9, 0.05, 0.05), roughness=0.5, emissive=m.rgb(0.6, 0.02, 0.02)),
         "band": m.Look(m.rgb(0.62, 0.8, 0.98), roughness=0.6, emissive=m.rgb(0.3, 0.42, 0.55))}


def _monitor_parts():
    sx = MON_X  # the screen's plane (facing -x)
    zc = SCR_Z0 + SCR_H / 2
    return [("mon_body", m.box((0.03, SCR_W + 2 * BEZ, SCR_H + 2 * BEZ + 0.012), (sx + 0.015, MON_Y, zc - 0.006)), "mon"),
            ("mon_neck", m.box((0.03, 0.06, SCR_Z0), (sx + 0.06, MON_Y, SCR_Z0 / 2)), "mon"),
            ("mon_foot", m.box((0.18, 0.24, 0.01), (sx + 0.06, MON_Y, 0.005)), "mon"),
            ("page", m.box((0.0004, SCR_W * 0.62, SCR_H), (sx - 0.0002, MON_Y, zc)), "page"),
            ("band", m.box((0.0004, SCR_W * 0.62, 2 * BAND_HALF), (sx - 0.0006, MON_Y, SCR_Z0 + BAND_Z)), "band")]


def _page_lines():
    """Grey text lines and the red line, on the page's slider; z from the active area's bottom. Lines span the screen at the start and at the
    end (shifted up by up to 0.14 m), so none leaves it."""
    parts = []
    sx = MON_X - 0.0010
    w = SCR_W * 0.5
    for i, z in enumerate(np.arange(0.012, SCR_H - 0.15, 0.009)):
        if abs(z - RED0) < 0.006:
            continue
        ww = w * (0.55 if i % 7 == 6 else 1.0)
        parts.append((f"t{i}", m.box((0.0004, ww, 0.003), (sx, MON_Y + (w - ww) / 2, SCR_Z0 + z)), "text"))
    parts.append(("redline", m.box((0.0004, w, 0.004), (sx - 0.0002, MON_Y, SCR_Z0 + RED0)), "red"))
    return parts


def _desk():
    xb, xf = PAD_D / 2, -PAD_D / 2
    side = [(xf, 0.0), (xb, 0.0), (xb, PAD_H1), (xf, PAD_H0)]  # the wedge's side outline (x, z)
    wedge = m.prism(side, -PAD_W / 2, PAD_W / 2).turned(90, "x")  # the outline drawn in x-z, extruded across the pad's width (y)
    frame = [("pad", wedge, "pad"),
             ("pad_edge", m.box((0.002, PAD_W, PAD_H1 - 0.002), (xb + 0.001, 0.0, PAD_H1 / 2)), "edge"),
             ("foot_f", m.box((0.01, PAD_W - 0.02, 0.0008), (xf + 0.012, 0.0, 0.0004)), "foot")] + _monitor_parts()
    cols = [m.col_hull("pad_c", wedge)]
    doc = m.slider("doc", "Z", 0.0, 0.2, _page_lines(), (), mass=0.002, spring=400.0, damping=4.0, friction=0.0, effort=20.0, armature=1e-5)
    part = m.mechanism("desk", (PX, PY, TOP), [doc], LOOKS, frame_parts=frame, frame_colliders=cols, friction=(0.3, 0.25), solver_iters=32)
    orig = part.build  # PhysX contact reports on the trackpad, so that it can read the robot's touches; a compliant contact for the soft finger pads

    def build():
        u = orig().replace('"PhysicsMassAPI"', '"PhysicsMassAPI", "PhysxContactReportAPI"').replace('"PhysicsMaterialAPI"', '"PhysicsMaterialAPI", "PhysxMaterialAPI"')
        i = u.index("float physics:dynamicFriction")
        j = u.index("\n", i)
        return u[:j] + f"\n        float physxMaterial:compliantContactStiffness = {PAD_K}\n        float physxMaterial:compliantContactDamping = {PAD_C}" + u[j:]
    return dataclasses.replace(part, build=build)


OBJECTS = [_desk()]


# -- touch: robot contacts on the glass, one touch per robot link ----------------------------------------------------------------------------
_VIEW: dict = {}
_COND: list = []  # indices of the robot links that carry a conductive nib


def _view(env):
    base = env.unwrapped
    key = id(base)
    if key not in _VIEW:
        paths = []
        for side in ("left", "right"):
            paths += [str(p) for p in base.scene[f"{side}_robot"].root_view.link_paths[0]]
        _COND.clear()
        _COND.extend(i for i, p in enumerate(paths) if p.rsplit("/", 1)[-1].startswith("tip_"))  # the nibs sit on the two finger links (tip_left, tip_right) only
        print("[sc] robot links", [p.rsplit("/", 1)[-1] for p in paths], "conductive", _COND, flush=True)
        desk = base.scene["desk"]
        frame = [str(p) for n, p in zip(desk.body_names, desk.root_view.link_paths[0]) if n == "frame"][0]
        _VIEW[key] = (base.sim.physics_sim_view.create_rigid_contact_view([frame], filter_patterns=[paths], max_contact_data_count=4096), len(paths))
    return _VIEW[key]


def _np(x):
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x.detach().cpu().numpy(), float)


def touches(env):
    """Every robot link pressing the glass's top now: [((x, y) in the part frame, metres), force N, link index, carries a nib]."""
    view, n = _view(env)
    base = env.unwrapped
    f, pts, nrm, _sep, cnt, start = view.get_contact_data(float(base.sim.get_physics_dt()))
    f, pts, nrm = _np(f).reshape(-1), _np(pts).reshape(-1, 3), _np(nrm).reshape(-1, 3)
    g = np.array([-math.sin(SLOPE), 0.0, math.cos(SLOPE)])  # the glass's normal
    cnt, start = _np(cnt).reshape(-1).astype(int), _np(start).reshape(-1).astype(int)
    origin = m._np(base.scene.env_origins[0])
    out = []
    for li, (c, s) in enumerate(zip(cnt, start)):  # one (sensor, filter) pair per robot link
        if c <= 0:
            continue
        p = pts[s:s + c] - origin - np.array([PX, PY, TOP])
        w = np.abs(f[s:s + c])
        on = (np.abs(p[:, 0]) <= PAD_D / 2) & (np.abs(p[:, 1]) <= PAD_W / 2) & (np.abs(nrm[s:s + c] @ g) >= NORMAL_COS)
        if not on.any() or w[on].sum() <= 0:
            continue
        out.append(((w[on, None] * p[on, :2]).sum(0) / w[on].sum(), float(w[on].sum()), li, li in _COND))
    return out


class Trial:
    def reset(self, env=None):
        self.page = 0.0  # how far the page has moved up (m): the app's state
        self.v = 0.0  # its speed (m/s), while it coasts
        self.hist = []  # (t, page) while two touches are down
        self.t = 0.0
        self.prev = None  # the mean of the two touches' depth last step
        self.one_prev = None
        self.drag = 0.0  # how far the current single touch has travelled in this stroke
        self.pointer = False  # a single touch travelled over DRAG_VOID in one stroke: the pointer moved
        self.recent = []  # (t, page) over the last REST_S seconds
        self.f_max = 0.0
        self.n = 0
        self.click = False
        self.last = {}

    def step(self, env):
        dt = float(env.unwrapped.step_dt) if hasattr(env.unwrapped, "step_dt") else 0.05
        self.t += dt
        t = touches(env)
        tt = [p for p, f, _, cond in t if cond and f >= TOUCH_N]  # only a nib registers on the capacitive glass
        if len(tt) == 2 and np.linalg.norm(tt[0] - tt[1]) < MERGE:
            tt = [0.5 * (tt[0] + tt[1])]  # fingertips closer than a fingertip's width read as one touch
        f_tot = sum(f for _, f, _, _ in t)  # the Force Touch sensors read every press
        if os.environ.get("SC_DEBUG"):
            ft = m.fingertips(env)[12:] - np.array([PX, PY, TOP])
            self.dbg = {"links": [(li, round(f, 2)) for _, f, li, _ in t], "tips_z": [round(float(z - glass_z(x)) * 1000, 1) for x, z in ft[::2, [0, 2]]],
                        "tips_y": [round(float(y) * 1000, 1) for y in ft[::2, 1]]}
        self.f_max = max(self.f_max, f_tot)
        if f_tot > CLICK_N:
            self.click = True
        self.n = len(tt)
        if self.n == 2:
            x = 0.5 * (tt[0][0] + tt[1][0])
            if self.prev is not None:
                self.page += GAIN * (x - self.prev)
            self.prev = x
            self.v = 0.0
            self.hist = [h for h in self.hist if h[0] > self.t - 0.1] + [(self.t, self.page)]
        else:
            if self.prev is not None and len(self.hist) >= 2:  # the fingers just lifted: the page coasts on at its last speed
                self.v = (self.hist[-1][1] - self.hist[0][1]) / max(self.hist[-1][0] - self.hist[0][0], 1e-6)
            self.prev, self.hist = None, []
            self.page += self.v * dt
            self.v *= math.exp(-dt / TAU)
            if abs(self.v) < 1e-4:
                self.v = 0.0
        if self.n == 1:
            if self.one_prev is not None:
                self.drag += float(np.linalg.norm(tt[0] - self.one_prev))
                self.pointer = self.pointer or self.drag > DRAG_VOID
            self.one_prev = tt[0]
        else:
            self.one_prev, self.drag = None, 0.0
        self.recent = [h for h in self.recent if h[0] > self.t - REST_S] + [(self.t, self.page)]
        m.set_target(env, "desk", "doc", float(np.clip(self.page, 0.0, 0.2)))
        self.last = {"n": self.n, "tips": [[round(float(v) * 1000, 1) for v in p] for p in tt], "f": round(f_tot, 3), "f_max": round(self.f_max, 3),
                     "page": round(self.page * 1000, 1), "v": round(self.v * 1000, 1), "red_to_band": round((RED0 + self.page - BAND_Z) * 1000, 1),
                     "drag": round(self.drag * 1000, 1), "click": self.click, "pointer": self.pointer, "score": self.tier(), **getattr(self, "dbg", {})}

    def at_rest(self):
        """The page has not coasted and has moved under 1 mm in the last REST_S seconds."""
        pg = [p for _, p in self.recent]
        return self.v == 0.0 and self.t >= REST_S and max(pg) - min(pg) < 0.001

    def red_off(self):
        """The red line's height from the band's centre on the screen (m)."""
        return RED0 + self.page - BAND_Z

    def tier(self) -> float:
        if self.click or self.pointer:
            return 0.0
        if abs(self.red_off()) <= BAND_HALF and self.at_rest():
            return 1.0
        if self.page >= 0.5 * (BAND_Z - RED0):
            return 0.67
        if self.page >= MOVED:
            return 0.33
        return 0.0


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.tier()


def success(objects) -> bool:
    return TRIAL.tier() >= 1.0


# -- the demo ------------------------------------------------------------------------------------------------------------------------------
OPEN = _e("SC_OPEN", -0.010)  # each finger's opening: the jaw 20 mm open, the two fingertips side by side across the pad
X0 = -0.035  # the fingers' start depth on the pad (part frame)
F_LO, F_HI = 0.12, 0.4  # the press kept between these (N, both fingers): under the lowest click force of the range run, 0.5 N
STEP = 0.004  # slide in 4 mm steps
STOP_EARLY = _e("SC_STOP", 0.010)  # stop the slide with the red line this far below the band centre (m of page)
STEP_T = _e("SC_STEP_T", 0.1)


def demo_script(ctx):
    """Modes: full. none: the arms wait. one_tip: the jaw closed, so one finger pair makes one touch, slid the same way. flick: the slide made fast,
    the fingers lifted while moving. press: the fingers pressed down 6 mm into the glass, then slid. rigid: the full script, run with SC_K=100000
    (the nibs' give taken away: bare rigid fingers on rigid glass). brush: the nibs skim the glass under the touch force (0.05 N) along the same slide."""
    mode = ctx.mode
    if mode not in ("full", "none", "one_tip", "flick", "press", "rigid", "brush"):
        raise ValueError(f"unknown mode {mode!r}")
    R = ctx.right
    if mode == "none":
        yield from R.wait(3.0)
        return
    c = np.array([PX, PY, TOP])
    u = np.array([math.cos(SLOPE), 0.0, math.sin(SLOPE)])  # along the glass, away from the user
    fing = 0.0 if mode == "one_tip" else OPEN

    def on_glass(x, dz):
        return c + np.array([x, 0.0, glass_z(x) + dz])

    yield from R.move(on_glass(X0, 0.06), grip=d.down(0.0), finger=fing, label="over the trackpad")
    yield from R.move(on_glass(X0, 0.022), t=0.8, label="down")
    print("[sc] above", json.dumps(TRIAL.last), flush=True)
    p = R.at().copy()
    for k in range(240):  # down slowly, 0.15 mm at a time, until the glass reports the fingers (the first contact)
        if mode == "brush" and TRIAL.last.get("f", 0.0) > 0.0:
            break
        if TRIAL.last.get("n", 0) >= (1 if mode == "one_tip" else 2) and TRIAL.last.get("f", 0.0) >= F_LO:
            break
        p = p - np.array([0.0, 0.0, 0.00015 if k < 60 else 0.00003])  # slow for the last millimetre: the fingers lag the command
        yield from R.move(p, t=0.1, label="descend")
    print("[sc] down", k, json.dumps(TRIAL.last), flush=True)
    if mode == "press":
        p = p - np.array([0.0, 0.0, 0.006])
        yield from R.move(p, t=0.5, label="presses hard")
    if mode in ("full", "rigid"):  # settle the press into the window before sliding: the fingers lag the command
        ok = 0
        for k in range(50):
            f = TRIAL.last.get("f", 0.0)
            ok = ok + 1 if (F_LO <= f <= F_HI and TRIAL.last.get("n", 0) >= 2) else 0
            if ok >= 5:  # five steady reads in a row: the fingers have stopped sinking
                break
            p = p + np.array([0.0, 0.0, float(np.clip((f - 0.5 * (F_LO + F_HI)) * 0.00015, -0.0002, 0.0002))])
            yield from R.move(p, t=0.1, label="settle")
    travel = 0.0
    for k in range(40):  # slide along the glass in short steps, reading the screen and the press
        off = TRIAL.red_off()
        if mode in ("full", "rigid") and off >= -STOP_EARLY:  # the fingers lag the command by about 8 mm of page: stop short
            break
        if mode not in ("full", "rigid") and travel >= 0.075:
            break
        f = TRIAL.last.get("f", 0.0)
        n = TRIAL.last.get("n", 0)
        dz = 0.0
        if mode in ("full", "rigid"):
            if f <= 0.0 or n < 2:
                dz = -0.00015  # bear down a little
            else:
                dz = float(np.clip((f - 0.5 * (F_LO + F_HI)) * 0.0003, -0.0003, 0.0003))  # a proportional hold on the press (0.3 mm per N of error)
        if mode == "brush":
            dz = 0.0001 if f >= 0.8 * TOUCH_N else (-0.0001 if f <= 0.0 else 0.0)  # keep skimming, under the touch force
        step = STEP * (3.0 if mode == "flick" else min(1.0, (k + 1) / 4))  # ramp the speed up over the first steps
        p = p + u * step + np.array([0.0, 0.0, dz])
        travel += step
        yield from R.move(p, t=STEP_T, label="slide")
    print("[sc] slid", round(travel * 1000, 1), json.dumps(TRIAL.last), flush=True)
    if mode in ("full", "rigid"):
        yield from R.wait(0.8)  # stop, then lift: no coast
    yield from R.move(R.at() + [0.0, 0.0, 0.05], t=0.3 if mode == "flick" else 0.8, label="lift")
    yield from R.wait(2.5)
    print("[sc] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
