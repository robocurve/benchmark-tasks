"""Refit a Dell PowerEdge R720's six-fan cooling assembly: carry it by its two levers, lower it level onto its two hidden guide pins, and cam both levers down.

A section of an R720 chassis (87.3 mm high, 444 mm wide, cover off) stands on the table: the drive cage in front, the empty fan bay behind it, the processor
heat sinks behind that. The cooling-fan assembly (six fans, 420 x 60 x 70 mm, 1.0 kg, assumed) stands on a foam mat in front, both blue levers up. The
robot takes one lever in each hand, lifts it over the drive cage, lowers it between the bay walls (2 mm to spare at each end) so that the two slots in its
base take the two guide pins hidden under it, lets go, and pushes each lever down by its free end: each lever's two cams bear on two brackets and press
its end the last 3.8 mm down, so that its three fan plugs enter their sockets. Each plug resists with 3 N (assumed): that is 9 N an end, more than the assembly's weight.

Sim (stated, all in code): the assembly is a free mechanism, its frame the grey body, each lever a hinge about x; a lever's closing stop is 2 degrees past
over-centre, so a closed lever stays closed. Each lever carries two cams, real colliders (8 mm discs 19 mm off the hinge, one either side of the lever), that
meet two brackets' undersides (one off the drive cage's back, one on a post) from 49 degrees on and press its end 3.8 mm down, the lift peaking at 86 degrees.
The plugs and sockets are real pins and square holes, so the end only goes down if its plugs are over their sockets. Each end rests on two pads, preloaded
springs (6 N a pad at rest, 10 N at the seat; estimate) that stand for the plugs' resistance. Nothing is applied by code. The demo reads the assembly's pose and the lever angles.

    python demokit.py task.py OUT
    python demokit.py task.py OUT --no-record --mode none,drop,drop15,drop5,drop3,drop5close,one,nocam,press     lazy routes
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
from yam_arena import ik  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Refit the cooling-fan assembly in the server and lock it in."
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0  # the tracker counts its own 1 s hold
HOLD_STEPS = 20
CM = 0.01

SHIFT = float(os.environ.get("FAN_SHIFT", "0.04"))  # x shift of the whole layout, metres
XB = 0.402 + SHIFT  # the fan bay's middle (and the guide pins' and plug sockets' x)
XM = 0.15 + SHIFT  # the mat the assembly starts on
# the assembly: 420 wide (y) x 60 deep (x) x 70 tall; its local origin is its middle
AW, AD, AH = 0.420, 0.060, 0.070
ZH = AH / 2  # local z of its bottom is -ZH
MASS = 1.0
M_LEV = 0.04
M_FRAME = MASS - 2 * M_LEV
# heights above the table
FLOOR = 0.010  # chassis floor top
ZS = 0.016  # seated: the assembly's base bottom, on four standoffs
REST = ZS + 0.004  # on its pins with the plugs resting on their sockets, unpressed
PLUG = 0.0055  # plug length below the base
SOCK = ZS - PLUG + 0.004  # socket top: a plug is 4 mm in when seated
PIN_Y, PIN_TOP, PIN_R = 0.160, FLOOR + 0.012, 0.0025  # the two guide pins: 5 mm across, 12 mm tall
SLOT = 0.0030  # slot half-width: 0.5 mm a side
SOCK_H = 0.0022  # a socket hole's half-width
PLUG_YS = [(-2.5 + k) * 7.0 * CM for k in range(6)]  # six plugs, 70 mm apart
# levers
YP = 0.175  # lever hinge's |y|
XL = -0.001  # lever's x (local): over the assembly's middle, so that it hangs level by its levers
Z0 = 0.030  # hinge height above the base bottom: the levers lie in grooves in the frame when closed
LEV_W, LEV_T, LEV_L = 0.020, 0.018, 0.075
E_CAM, R_CAM = 0.019, 0.004
C_CAM = R_CAM + E_CAM - 0.0038  # the bracket's underside above the hinge at rest: the cam presses an end 3.8 mm, and the last 0.2 mm is the pads' give
XE = 0.0175  # the rear cam and its bracket: x (local)
XEF = -0.016  # the front cam and its bracket: the two cams straddle the lever, so that the press goes down through the end's middle and the frame stays level
NOTCH = (0.165, 0.190)  # |y| of the slot in the frame's front base and wall that the front bracket passes through as the assembly comes down
BETA = math.radians(4.0)  # the cam's lobe leads the lever by 4 degrees: its lift peaks at 86 degrees and falls off 0.1 mm to the closed stop at 92 (over-centre)
LATCH_DEG = 88.0
STOP_DEG = 92.0
PLUG_F = 9.0  # N an end, nominal (3 plugs of 3 N)
PAD_K = float(os.environ.get("FAN_PADK", "1000"))  # N/m a pad (estimate)
PAD_PRE = float(os.environ.get("FAN_PADPRE", "0.006"))  # the pads' springs are set 6 mm past their stops: 6 N a pad, 12 N an end at rest, 20 N at the seat (the plugs' 3 N each is 9 N an end; 6 to 15 for 2 to 5 N a plug), so the weight (4.5 N an end) leaves it at its rest height
PLUG_TRAVEL = 0.0041

LOOKS = {"steel": m.Look(color=m.rgb(0.62, 0.64, 0.67), metallic=0.9, roughness=0.4), "dark": m.Look(color=m.rgb(0.10, 0.10, 0.11), roughness=0.5),
         "grey": m.Look(color=m.rgb(0.30, 0.31, 0.33), roughness=0.55), "blue": m.Look(color=m.rgb(0.10, 0.28, 0.85), roughness=0.45),
         "brass": m.Look(color=m.rgb(0.78, 0.62, 0.30), metallic=0.85, roughness=0.3), "foam": m.Look(color=m.rgb(0.20, 0.22, 0.26), roughness=0.9),
         "green": m.Look(color=m.rgb(0.05, 0.30, 0.12), roughness=0.5), "gold": m.Look(color=m.rgb(0.80, 0.65, 0.20), metallic=0.9, roughness=0.3)}


def _box(parts, cols, name, x0, x1, y0, y1, z0, z1, look, collide=True):
    size, c = (x1 - x0, y1 - y0, z1 - z0), ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    parts.append((name, m.box(size, c), look))
    if collide:
        cols.append(m.col_box(f"{name}_c", size, c))


def _chassis() -> m.Part:
    """World x, y, and z above the table. A section of the chassis from its front face (x 0.19 here) to just behind the heat sinks."""
    parts, cols = [], []
    xf, xr = 0.19 + SHIFT, XB + 0.072
    _box(parts, cols, "floor", xf, xr, -0.222, 0.222, 0.0, FLOOR, "steel")
    for k, sy in enumerate((-1, 1)):
        a, b = (0.212, 0.222) if sy > 0 else (-0.222, -0.212)
        _box(parts, cols, f"wall{k}", xf, xr, a, b, 0.0, 0.0873, "steel")
    _box(parts, cols, "cage", xf, XB - 0.032, -0.212, 0.212, FLOOR, 0.0873, "grey")  # the drive cage under its closed top
    _box(parts, cols, "sinks", XB + 0.034, XB + 0.070, -0.212, 0.212, FLOOR, 0.040, "grey")  # the heat sinks: they stop the assembly sliding back
    _box(parts, cols, "rear", XB + 0.070, XB + 0.072, -0.222, 0.222, 0.0, 0.0873, "steel")
    # the six fan sockets in one rail: each a 4.4 mm square hole (plugs are 4 mm across, 0.2 mm a side)
    edges = [-0.200] + [v for y in PLUG_YS for v in (y - SOCK_H, y + SOCK_H)] + [0.200]
    for k in range(7):
        _box(parts, cols, f"rail{k}", XB - 0.006, XB + 0.006, edges[2 * k], edges[2 * k + 1], FLOOR, SOCK, "green")
    for k, y in enumerate(PLUG_YS):
        _box(parts, cols, f"sockf{k}", XB - 0.006, XB - SOCK_H, y - SOCK_H, y + SOCK_H, FLOOR, SOCK, "green")
        _box(parts, cols, f"sockb{k}", XB + SOCK_H, XB + 0.006, y - SOCK_H, y + SOCK_H, FLOOR, SOCK, "green")
    links = []
    for k, (sx, sy) in enumerate(((-0.018, -1), (-0.018, 1), (0.006, -1), (0.006, 1))):  # four spring-loaded pads under the base: the six plugs' hold
        w = 0.012 if sx < 0 else 0.006
        py = 0.150 if sx < 0 else 0.190
        size, c = (w, 0.012, REST - FLOOR), (XB + sx, sy * py, (REST + FLOOR) / 2)
        pad = m.box(size, c)
        links.append(m.slider(f"pad{k}", "Z", -(REST - ZS), 0.0, [(f"pad{k}", pad, "steel")], [m.col_box(f"pad{k}_c", size, c)], at=c, mass=0.05, spring=PAD_K, friction=0.05, damping=2.0))
    for k, sy in enumerate((-1, 1)):  # the guide pins, chamfered at the tip
        base = m.cylinder(PIN_R, 0.009, (XB, sy * PIN_Y, FLOOR + 0.0045), segments=12)
        tip = m.cylinder(0.001, 0.003, (XB, sy * PIN_Y, FLOOR + 0.0105), segments=12)
        both = m.join(base, tip)
        parts.append((f"pin{k}", both, "brass"))
        cols.append(m.col_hull(f"pin{k}_c", both))
    zr = REST + Z0  # the hinge's height with the assembly at its rest height
    zb = zr + C_CAM  # the underside of each cam bracket
    for k, sy in enumerate((-1, 1)):  # a cam bracket on each wall: a post from the floor beside the cam, and a plate over it
        y0, y1 = YP - 0.0075, YP + 0.0125
        ya, yb = (y0, y1) if sy > 0 else (-y1, -y0)
        _box(parts, cols, f"post{k}", XB + 0.022, XB + 0.025, ya, yb, FLOOR, zb + 0.008, "gold")
        _box(parts, cols, f"plate{k}", XB + 0.014, XB + 0.025, ya, yb, zb, zb + 0.008, "gold")
        _box(parts, cols, f"fplate{k}", XB - 0.033, XB - 0.0125, ya, yb, zb, zb + 0.008, "gold")  # the front bracket, from the drive cage's back over the front cam
    return m.mechanism("chassis", (0.0, 0.0, TOP), links, LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.4, 0.35))


def _mat() -> m.Part:
    parts, cols = [], []
    _box(parts, cols, "mat", XM - 0.045, XM + 0.045, -0.24, 0.24, 0.0, 0.010, "foam")
    return m.fixed("mat", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.8, 0.7))


def L(x, y, z):
    """A point given by height above the assembly's base bottom, as its local coordinates."""
    return (x, y, z - ZH)


def _assembly() -> m.Part:
    parts, cols = [], []

    def box(name, x0, x1, y0, y1, z0, z1, look="grey"):
        _box(parts, cols, name, x0, x1, y0, y1, z0 - ZH, z1 - ZH, look)

    h = AW / 2
    # base plate (0 to 15 mm) with a 6 mm slot for each guide pin, and the notch for each cam bracket; the body above it
    n0, n1 = NOTCH
    box("plateA0", -0.030, -SLOT, -n0, n0, 0.0, 0.015)
    for k, sy in enumerate((-1, 1)):
        a, b = (n0, n1) if sy > 0 else (-n1, -n0)
        box(f"plateN{k}", -0.010, -SLOT, a, b, 0.0, 0.015)  # beside the front bracket's slot
        a, b = (n1, h) if sy > 0 else (-h, -n1)
        box(f"plateO{k}", -0.030, -SLOT, a, b, 0.0, 0.015)
    for k, (a, b) in enumerate(((-h, -PIN_Y - SLOT), (-PIN_Y + SLOT, PIN_Y - SLOT), (PIN_Y + SLOT, h))):
        box(f"plateM{k}", -SLOT, SLOT, a, b, 0.0, 0.015)
    box("plateA1", SLOT, 0.010, -h, h, 0.0, 0.015)
    # the body above it, with a groove at each end for the closed lever (x -22 to 10 mm, |y| over 90 mm), and the notches (x 10 to 28 mm, |y| over 160 mm) for the cam brackets
    box("bodyW", -0.030, -0.022, -n0, n0, 0.015, AH)
    for k, sy in enumerate((-1, 1)):
        a, b = (n0, n1) if sy > 0 else (-n1, -n0)
        box(f"bodyWs{k}", -0.030, -0.022, a, b, 0.064, AH)  # the wall over the slot
        a, b = (n1, h) if sy > 0 else (-h, -n1)
        box(f"bodyWo{k}", -0.030, -0.022, a, b, 0.015, AH)
    box("bodyM", -0.022, 0.010, -0.090, 0.090, 0.015, AH)
    box("bodyB", 0.010, 0.028, -0.160, 0.160, 0.0, AH)
    box("bodyC", 0.028, 0.030, -h, h, 0.0, AH)
    for k, y in enumerate(PLUG_YS):  # the six fan plugs
        plug = m.join(m.box((0.004, 0.004, 0.0025), (0.0, y, -ZH - 0.00125)), m.box((0.002, 0.002, 0.003), (0.0, y, -ZH - 0.0025 - 0.0015)))  # a plug with a chamfered tip
        parts.append((f"plug{k}", plug, "gold"))
        cols.append(m.col_hull(f"plug{k}_c", plug))
    for k in range(6):  # the fans: dark hubs on the top, in a grey frame
        parts.append((f"fan{k}", m.cylinder(0.026, 0.002, (0.0, PLUG_YS[k], AH - ZH + 0.001)), "dark"))
    links = []
    for k, sy in enumerate((-1, 1)):
        side = "L" if sy > 0 else "R"
        slab_c = L(XL, sy * YP, Z0 + LEV_L / 2)
        slab = m.box((LEV_W, LEV_T, LEV_L), slab_c)
        ear = m.cylinder(R_CAM, 0.006, L(XE, sy * (YP + E_CAM * math.cos(BETA)), Z0 + E_CAM * math.sin(BETA)), axis="x", segments=16)
        fear = m.cylinder(R_CAM, 0.006, L(XEF, sy * (YP + E_CAM * math.cos(BETA)), Z0 + E_CAM * math.sin(BETA)), axis="x", segments=16)
        lo, hi = (0.0, STOP_DEG) if sy > 0 else (-STOP_DEG, 0.0)
        links.append(m.hinge(f"lev{side}", "X", L(XL, sy * YP, Z0), lo, hi, [(f"slab{side}", slab, "blue"), (f"ear{side}", ear, "blue"), (f"fear{side}", fear, "blue")],
                             [m.col_box(f"slab{side}_c", (LEV_W, LEV_T, LEV_L), slab_c), m.col_hull(f"ear{side}_c", ear), m.col_hull(f"fear{side}_c", fear)],
                             mass=M_LEV, spring=0.0, damping=0.05, friction=0.01, effort=5.0))
    pos = (XM, 0.0, TOP + 0.010 + ZH + 0.0003)
    ixx, iyy, izz = M_FRAME * (AW ** 2 + AH ** 2) / 12, M_FRAME * (AD ** 2 + AH ** 2) / 12, M_FRAME * (AD ** 2 + AW ** 2) / 12
    return m.mechanism("fan", pos, links, LOOKS, frame_parts=parts, frame_colliders=cols, free_mass=M_FRAME, friction=(0.5, 0.45), frame_inertia=(ixx, iyy, izz))


OBJECTS = [_chassis(), _mat(), _assembly()]


# -- the scorer --------------------------------------------------------------------------------


def _wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def measure(st) -> dict:
    """From the assembly's pose and its two lever angles."""
    R = st.rotation()
    ends, lat, on_pin = {}, {}, True
    for name, sy in (("L", 1), ("R", -1)):
        p = st.pos + R @ np.array([0.0, sy * 0.190, -ZH])
        h = float(p[2] - TOP - ZS)
        ys = [y for y in PLUG_YS if (y > 0) == (sy > 0)]
        e = 0.0
        for y in ys:
            q = st.pos + R @ np.array([0.0, y, -ZH - PLUG])
            e = max(e, float(math.hypot(q[0] - XB, q[1] - y)))
        ends[name], lat[name] = h, e
        pin = np.array([XB, sy * PIN_Y, TOP + PIN_TOP - 0.0005])  # the pin's top
        lp = (pin - st.pos) @ R
        if not (abs(lp[0]) < SLOT - PIN_R + 0.001 and abs(lp[1] - sy * PIN_Y) < SLOT - PIN_R + 0.001 and lp[2] > -ZH):
            on_pin = False
    roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    pitch = math.degrees(math.asin(-float(np.clip(R[2, 0], -1, 1))))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    angs = {"L": st.deg("levL"), "R": -st.deg("levR")}
    return {"h_mm": {k: round(v * 1000, 2) for k, v in ends.items()}, "lat_mm": {k: round(v * 1000, 2) for k, v in lat.items()}, "on_pins": bool(on_pin),
            "roll": round(roll, 2), "pitch": round(pitch, 2), "yaw": round(yaw, 2), "lever_deg": {k: round(v, 1) for k, v in angs.items()}}


def _tier(r) -> float:
    if not r or not r["on_pins"]:
        return 0.0
    h = r["h_mm"]
    if max(h.values()) > 5.0:  # on its pins, both ends at most 1 mm over its rest height (4 mm over its seat): one end pressed lifts the other about 0.5 mm
        return 0.0
    mated = {k: bool(h[k] <= 0.5 and r["lat_mm"][k] <= 0.5 and r["lever_deg"][k] >= LATCH_DEG) for k in "LR"}
    level = abs(r["roll"]) <= 0.5 and abs(r["pitch"]) <= 0.5
    if mated["L"] and mated["R"] and level and r["clear"] and r["still"]:
        return 1.0
    if mated["L"] or mated["R"]:
        return 0.5
    return 0.25


class Tracker:
    def __init__(self):
        self.reset()

    def reset(self, env=None):
        if env is not None and PAD_PRE > 0:
            for k in range(4):
                m.set_target(env, "chassis", f"pad{k}", PAD_PRE)  # the pads' springs, preloaded against their stops
        self.steps, self.last, self.hold, self.dropped, self.prev = 0, {}, 0, False, None
        self.peak_free = 0.0
        self.h_touch, self.h_rel, self.pend, self.was_touch, self.why = None, 0.0, 0, False, ""
        self.tips = None
        self.poses = []

    def step(self, env):
        self.steps += 1
        dt = float(env.unwrapped.step_dt)
        st = m.state(env, "fan")
        r = measure(st)
        R = st.rotation()
        tips = m.fingertips(env)
        lp = (tips - st.pos) @ R
        half = np.array([AD / 2, AW / 2, (Z0 + LEV_L) / 2])  # the body and the levers when open
        cen = np.array([0.0, 0.0, (Z0 + LEV_L) / 2 - ZH])
        out = np.maximum(np.abs(lp - cen) - half, 0.0)
        clear = bool(np.min(np.linalg.norm(out, axis=1)) > 0.03)
        # still: moved under 0.25 mm and turned under 0.1 degrees in the last 0.5 s, read from its pose (the solver's velocity of a body clamped
        # between its cams and its pads reads about 16 mm/s while its pose holds to 0.01 mm, so the pose is the honest test)
        self.poses.append((np.array(st.pos, dtype=float), R.copy()))
        self.poses = self.poses[-11:]
        p0, R0 = self.poses[0]
        turn = math.degrees(math.acos(float(np.clip((np.trace(R0.T @ R) - 1.0) / 2.0, -1.0, 1.0))))
        moved = float(np.linalg.norm(np.asarray(st.pos, dtype=float) - p0))
        r["clear"], r["still"] = clear, bool(len(self.poses) == 11 and moved < 0.00025 and turn < 0.1)
        r["moved_mm"] = round(moved * 1000, 3)
        hmin = min(r["h_mm"].values()) / 1000.0
        hmax = max(r["h_mm"].values()) / 1000.0
        # touching: a finger point within 3 mm of the body box or of a lever's slab (at the lever's own angle), not nearness to the open levers' bounding box
        dmin = float(np.min(np.linalg.norm(np.maximum(np.abs(lp) - np.array([AD / 2, AW / 2, ZH]), 0.0), axis=1)))
        for sy, key in ((1, "L"), (-1, "R")):
            th = math.radians(r["lever_deg"][key])
            dv = lp - np.array([XL, sy * YP, Z0 - ZH])
            v, n = np.array([0.0, -sy * math.sin(th), math.cos(th)]), np.array([0.0, sy * math.cos(th), math.sin(th)])
            q = np.stack([dv[:, 0], dv @ n, dv @ v - LEV_L / 2], axis=1)
            dmin = min(dmin, float(np.min(np.linalg.norm(np.maximum(np.abs(q) - np.array([LEV_W / 2, LEV_T / 2, LEV_L / 2]), 0.0), axis=1))))
        touch = dmin < 0.003
        if touch:
            self.h_touch, self.pend, self.was_touch = hmax, 0, True  # its height while a finger is on it
        else:
            if self.was_touch:
                self.h_rel, self.pend, self.was_touch = self.h_touch, 20, False  # a release: watch the next 1 s
            if self.pend > 0:
                self.pend -= 1
                if self.h_rel - hmax > 0.002:
                    self.dropped = True  # let go more than 2 mm above where it rests (the demo lets go within 1.2 mm)
                    self.why = "release"
        if clear and hmin > 0.020:
            self.dropped = True  # let go with its lower end more than 2 cm above its seat
            self.why = self.why or "20mm"
        if float(st.lin_vel[2]) < -0.3:
            self.dropped = True
            self.why = self.why or "vel"
        z = float(st.pos[2])
        if self.prev is not None and (self.prev - z) / dt > 0.3:
            self.dropped = True
            self.why = self.why or "fall"  # its pose fell faster than 0.3 m/s in one step: free fall from about 5 mm up or more (the demo lowers it at under 0.1 m/s)
        self.prev = z
        if st.pos[2] < TOP + 0.005 or abs(st.pos[0] - XB) > 0.12 and st.pos[2] < TOP + 0.08:
            pass
        if os.environ.get("FAN_DEBUG") and self.steps % 10 == 0:
            lt = ((tips[:12].mean(axis=0) - st.pos) @ R) * 1000
            print("[fan-dbg] left fingertips in the assembly's frame (mm): " + str(np.round(lt, 1).tolist()), flush=True)
        self.tips = lp  # the 24 finger points in the assembly's frame (diagnostics)
        self.last = r
        self.hold = self.hold + 1 if (_tier(r) == 1.0 and not self.dropped) else 0
        r["hold"] = self.hold
        if self.steps % 10 == 0:
            print("[fan] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), "t": round(self.steps * dt, 2), **{k: v for k, v in r.items()}, "drop": self.dropped, "why": self.why, "dmin_mm": round(dmin * 1000, 1)}), flush=True)


TRACK = Tracker()
m.on_step(TRACK.step)
m.on_reset(TRACK.reset)


def score(objects) -> float:
    if TRACK.dropped:
        return 0.0
    if TRACK.hold >= HOLD_STEPS:
        return 1.0
    t = _tier(TRACK.last)
    return min(t, 0.5)


def success(objects) -> bool:
    return TRACK.hold >= HOLD_STEPS and not TRACK.dropped


# -- the scripted demo --------------------------------------------------------------------------

PUSH_H = float(os.environ.get("FAN_PUSH_H", "0.06"))  # where on the lever the fingers push
LEAD0 = float(os.environ.get("FAN_LEAD", "12"))  # degrees the command leads the lever
PUSH_O = float(os.environ.get("FAN_PUSH_O", "0.014"))  # the fingertips' centre outside the lever's face
GRIP_H = float(os.environ.get("FAN_GRIP_H", "0.060"))  # the TCP's height above the hinge on the lever
CARRY_DZ = float(os.environ.get("FAN_CARRY", "0.075"))  # the base bottom's height over its rest when carried over the drive cage... added to the cage top


def _hand_goal(side, st, deg=0.0):
    """Where the TCP is, held at the lever's grip point at lever angle `deg` (closing)."""
    sy = 1 if side == "left" else -1
    pivot = st.to_world(np.array(L(XL - 0.008, sy * YP, Z0)))[0]  # the fingertips on the lever's middle
    v = st.rotation() @ np.array([0.0, -sy * math.sin(math.radians(deg)), math.cos(math.radians(deg))])
    return pivot + GRIP_H * v


def demo_script(ctx):
    mode = ctx.mode
    arms = [("left", 1), ("right", -1)]
    if mode == "none":  # the arms wait (a lazy route)
        yield from ctx.left.wait(5.0)
        return
    A = {a: getattr(ctx, a) for a, _ in arms}
    st = ctx.state("fan")
    grip = d.tilted(float(os.environ.get("FAN_TILT", "60")), 0.0)  # fingers pointing forward and down, closing across the lever's thickness (y)
    goals = {a: _hand_goal(a, st) for a, _ in arms}
    above = {a: goals[a] - np.array([0.04, 0.0, 0.0]) for a, _ in arms}  # in front of the levers, nearer the robot
    yield from ctx.together(*[A[a].move(above[a], grip=grip, finger=d.FINGER_OPEN, t=2.5, label="in front of the levers") for a, _ in arms])
    yield from ctx.together(*[A[a].move(goals[a], t=1.2, label="in on the levers") for a, _ in arms])
    yield from ctx.together(*[A[a].servo(goals[a], timeout=1.0, label="settle") for a, _ in arms])
    yield from ctx.together(*[A[a].close(LEV_T, force=35.0, label="grip the levers") for a, _ in arms])
    # lift and carry
    z_carry = TOP + 0.120  # the base bottom: over the cage (87 mm) with room for the wrists' droop under the weight

    def bottom():
        return ctx.state("fan").pos[2] - ZH

    def rise_to(z, t=1.0):
        for _ in range(4):
            dz = z - bottom()
            if abs(dz) < 0.004:
                break
            yield from ctx.together(*[A[a].move(A[a].at() + np.array([0.0, 0.0, dz]), t=max(t, 0.5), label="raise to carry height") for a, _ in arms])
            yield from ctx.left.wait(0.3)

    yield from rise_to(z_carry, 1.5)
    dx = XB - ctx.state("fan").pos[0]
    for k in range(4):
        yield from ctx.together(*[A[a].move(A[a].at() + np.array([dx / 4, 0.0, 0.0]), t=0.9, label="carry over the cage") for a, _ in arms])
        yield from rise_to(z_carry, 0.6)
    yield from ctx.left.wait(0.5)

    def pose():
        s = ctx.state("fan")
        R = s.rotation()
        return s, math.degrees(math.atan2(R[2, 1], R[2, 2])), math.degrees(math.atan2(R[1, 0], R[0, 0]))

    def pitch_now():
        R = ctx.state("fan").rotation()
        return math.degrees(math.asin(-float(np.clip(R[2, 0], -1, 1))))

    def align(tol=0.0008):
        """Shift both hands together (and turn them about the assembly's middle) until its x, y and yaw are on the bay's."""
        for _ in range(0 if os.environ.get("FAN_DRY") else 6):
            s, roll, yaw = pose()
            ex, ey = XB - s.pos[0], -s.pos[1]
            pit = pitch_now()
            if abs(ex) < tol and abs(ey) < tol and abs(yaw) < 0.3 and abs(roll) < 0.3 and abs(pit) < 0.3:
                break
            gens = []
            for a, sy in arms:
                tcp = A[a].at()
                rad = np.array([tcp[0] - s.pos[0], tcp[1] - s.pos[1]])
                dyaw = -math.radians(yaw)
                turn = np.array([-dyaw * rad[1], dyaw * rad[0], 0.0])
                lvl = np.array([0.0, 0.0, -sy * math.radians(roll) * YP])  # the end that is high comes down, the low one up
                gens.append(A[a].move(tcp + np.array([ex, ey, 0.0]) + turn + lvl, t=0.8, label="align on the bay"))
            yield from ctx.together(*gens)
            s, roll, yaw = pose()
            pit = pitch_now()
            if abs(pit) >= 0.3:  # the wrists let it droop under its weight: take the pitch out about its middle
                yield from ctx.together(*[A[a].arc(s.pos, np.array([0.0, 1.0, 0.0]), -pit, t=0.8, label="take out the pitch") for a, _ in arms])
            yield from ctx.left.wait(0.3)

    yield from align()
    if mode in ("drop", "drop15", "drop5", "drop3", "drop5close"):
        # let go 3.4 cm over the pin tops (drop), or 15 / 5 / 3 mm over its rest height: it falls in; drop5close then closes both levers (lazy routes)
        s, _, _ = pose()
        dz = (TOP + ZS + {"drop": 0.040, "drop15": 0.004 + 0.015, "drop5": 0.004 + 0.005, "drop3": 0.004 + 0.003, "drop5close": 0.004 + 0.005}[mode]) - (s.pos[2] - ZH)
        yield from ctx.together(*[A[a].move(A[a].at() + np.array([0.0, 0.0, dz]), t=2.0, label="down to 4 cm") for a, _ in arms])
        yield from ctx.together(*[A[a].open(label="let go") for a, _ in arms])
        yield from ctx.together(*[A[a].move(A[a].at() + np.array([0.0, 0.0, 0.06]), t=1.0, label="clear") for a, _ in arms])
        yield from ctx.left.wait(2.5)
        if mode != "drop5close":
            return
    # lower: fast to 25 mm over the pins' tops, then slowly, correcting as it goes
    s, _, _ = pose()
    dz = (TOP + PIN_TOP + 0.020) - (s.pos[2] - ZH)
    if mode != "drop5close":
        yield from ctx.together(*[A[a].move(A[a].at() + np.array([0.0, 0.0, dz]), t=2.5, label="lower to the pins") for a, _ in arms])
        yield from align()
    for k in range(0 if (os.environ.get("FAN_DRY") or mode == "drop5close") else 80):
        s, roll, yaw = pose()
        h = (s.pos[2] - ZH) - (TOP + REST)
        if h < 0.0012:
            break  # on the pads: stop, so that the hands do not push it through them
        step = -min(0.0015, max(0.0003, h * 0.3))
        if h < 0.012 and (abs(XB - s.pos[0]) > 0.0012 or abs(s.pos[1]) > 0.0012):
            step = 0.002  # off the pins: back up a little and try again
        ex, ey = XB - s.pos[0], -s.pos[1]
        yield from ctx.together(*[A[a].move(A[a].at() + np.array([0.8 * ex, 0.8 * ey, step - sy * math.radians(roll) * YP * 0.5]), t=0.5, label="down onto the pins") for a, sy in arms])
    # on the pads: let go of the levers at once, so that the hands cannot push it down through them
    yield from ctx.together(*[A[a].open(label="let go of the levers") for a, _ in arms])
    yield from ctx.left.wait(0.8)
    if mode == "nocam":
        yield from ctx.together(*[A[a].open(label="let go") for a, _ in arms])
        yield from ctx.together(*[A[a].move(A[a].at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear") for a, _ in arms])
        yield from ctx.left.wait(2.0)
        return
    if mode == "press":  # push it down by the levers' tops, without closing them (a lazy route)
        yield from ctx.together(*[A[a].move(A[a].at() + np.array([0.0, 0.0, -0.01]), t=1.0, label="push down") for a, _ in arms])
        yield from ctx.together(*[A[a].open(label="let go") for a, _ in arms])
        yield from ctx.together(*[A[a].move(A[a].at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="clear") for a, _ in arms])
        yield from ctx.left.wait(2.0)
        return
    # let go of the levers, and push each down with closed fingers on its outer face, round its hinge, in stages read from its angle
    yield from ctx.together(*[A[a].move(A[a].at() + np.array([-0.03, 0.0, 0.07]), t=1.2, label="up and back, clear of the levers") for a, _ in arms])
    act = [("left", 1)] + ([("right", -1)] if mode != "one" else [])
    PT = float(os.environ.get("FAN_PUSH_TILT", "0"))
    YAW = bool(int(os.environ.get("FAN_PUSH_YAW", "1")))  # fingers closing along x: the closed pair is 18 mm across x and fits the lever's 32 mm groove; across y (yaw 0) the pads are 36 mm along x and land on the groove's edges
    PX = float(os.environ.get("FAN_PUSH_X", "-0.006" if YAW else "0.0"))  # the fingertips' x on the lever (local): the groove's middle
    push_grip = {sy: (d.down(-90.0 * sy) if YAW else d.tilted(PT, 0.0)) for _, sy in arms}
    pax = np.array([0.0, 0.0, -1.0]) if YAW else np.array([math.sin(math.radians(PT)), 0.0, -math.cos(math.radians(PT))])
    HALF_Y = 0.018 if YAW else 0.009  # the closed fingers' half-extent across the lever (world y)

    def contact(sy, th):
        s_ = ctx.state("fan")
        piv = s_.to_world(np.array(L(XL + PX, sy * YP, Z0)))[0]
        c, sn = math.cos(math.radians(th)), math.sin(math.radians(th))
        v = s_.rotation() @ np.array([0.0, -sy * sn, c])
        n = s_.rotation() @ np.array([0.0, sy * c, sn])
        off = PUSH_O if not YAW else LEV_T / 2 + HALF_Y * abs(c) - 0.002  # the fingers' near side 2 mm into the lever's face
        return piv + PUSH_H * v + off * n - 0.015 * pax

    try:
        sys.path.append(str(pathlib.Path(__file__).resolve().parent.parent.parent / "kit"))
        import taskspace as ts
    except ImportError:
        ts = None

    def diag(tag):
        """The push's state: lever angles, the fingers against the frame, the arm's command against its joints, the cam's resisting moment (estimate)."""
        s_ = ctx.state("fan")
        r_ = measure(s_)
        out = {"tag": tag}
        tp = TRACK.tips
        for a, sy in act:
            th = r_["lever_deg"]["L" if sy > 0 else "R"]
            q_obs = ctx.observed(a)[:6]
            q_cmd = A[a].q
            tcp_obs, tcp_cmd = A[a].tcp(), ik.tcp_pose(q_cmd, A[a].base)[0]
            lag = tcp_cmd - tcp_obs
            f = None
            if ts is not None:
                try:
                    f = np.linalg.solve(ts.compliance(q_obs, A[a].base) + 1e-9 * np.eye(3), lag)
                except Exception:
                    f = None
            pts = tp[:12] if (tp is not None and a == "left") else (tp[12:] if tp is not None else None)
            h = r_["h_mm"]["L" if sy > 0 else "R"] / 1000.0
            comp = max(0.0, (REST - ZS) - h)
            f_cam = max(0.0, 2 * PAD_K * (comp + PAD_PRE) - M_FRAME * 9.81 / 2) if comp > 0.0002 else 0.0
            t_cam = f_cam * E_CAM * math.cos(BETA + math.radians(th))
            arm_ = PUSH_H
            o = {"deg": th, "end_mm": round(h * 1000, 2), "q_err_deg": np.round(np.degrees(q_cmd - q_obs), 1).tolist(), "tcp_lag_mm": np.round(lag * 1000, 1).tolist(),
                 "push_N_est": None if f is None else np.round(f, 1).tolist(), "cam_Nm_est": round(t_cam, 3),
                 "push_Nm_est": None if f is None else round(float(sy * np.cross(s_.rotation() @ np.array([0.0, -sy * math.sin(math.radians(th)), math.cos(math.radians(th))]) * arm_, f) @ (s_.rotation() @ np.array([1.0, 0.0, 0.0]))), 3)}
            if pts is not None:
                o["tips_x_mm"] = [round(float(pts[:, 0].min()) * 1000, 1), round(float(pts[:, 0].max()) * 1000, 1)]
                o["tips_y_mm"] = [round(float(pts[:, 1].min()) * 1000, 1), round(float(pts[:, 1].max()) * 1000, 1)]
                o["tips_zlow_mm_above_top"] = round((float(pts[:, 2].min()) - (AH - ZH)) * 1000, 1)
                o["tips_zlow_mm_above_hinge"] = round((float(pts[:, 2].min()) - (Z0 - ZH)) * 1000, 1)
            o["tcp_z_mm"] = round(float(tcp_obs[2] - TOP) * 1000, 1)
            o["tcp_y_mm"] = round(float(tcp_obs[1]) * 1000, 1)
            out[a] = o
        out["pitch"], out["roll"], out["lat"] = r_["pitch"], r_["roll"], r_["lat_mm"]
        print("[fan-push] " + json.dumps(out), flush=True)

    yield from ctx.together(*[A[a].move(contact(sy, 0.0) + np.array([0.0, sy * 0.02, 0.05]), grip=push_grip[sy], finger=d.FINGER_CLOSED, t=2.0, label="over the lever's outer side") for a, sy in act])
    yield from ctx.together(*[A[a].move(contact(sy, 2.0), t=1.0, label="down beside the lever") for a, sy in act])
    # push each lever on from its own angle, the command leading it by a few degrees (the arm is soft: the lead is the push), until it is latched
    def ang(sy):
        return abs(ctx.state("fan").deg("lev" + ("L" if sy > 0 else "R")))

    for k in range(40):
        if all(ang(sy) >= LATCH_DEG + 1.0 for _, sy in act):
            break
        lead = LEAD0 + 1.0 * k
        yield from ctx.together(*[A[a].move(contact(sy, min(ang(sy) + lead, 112.0)), t=0.4, label=f"push the lever on ({ang(sy):.0f} degrees)") for a, sy in act])
        diag(k)
    yield from ctx.left.wait(0.3)
    yield from ctx.together(*[A[a].move(A[a].tcp() + np.array([-0.02, 0.0, 0.12]), t=1.5, label="clear") for a, _ in arms])  # from where the hand is: the push left the command below the lever
    yield from ctx.left.wait(1.5)
    print("[fan] END " + json.dumps({"mode": mode, **TRACK.last}), flush=True)
