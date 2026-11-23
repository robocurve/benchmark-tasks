"""Secure a new car battery with its hold-down: hook both J-bolts into the tray's slots, fit the bar over them and push
the nuts down onto it.

The table has an engine bay cut into it: a well 21 cm deep with a BCI Group 35 battery (230 x 175 x 225 mm) standing in
a steel tray at its bottom, its top 4 cm above the table, its length along x. The tray's floor sits on a bracket 19.5 mm
above the well's floor; halfway along each long side of the battery the floor has a hook slot (28 mm along x, 10 mm
across), 2.5 mm outside the battery, with the tray's 40 mm side wall 6 mm beyond it. Two new 12 in (305 mm) steel
J-bolts (1/4 in rod) stand hook down in holders at the bay's front corners, one each side, hook tips towards +x; the
hold-down bar (228 x 25.4 x 4 mm steel, a 10 x 20 mm slot 96 mm either side of its middle) lies on the table to the
right of the bay, along x, and two 1/4-20 steel flange nuts lie flange down on the table beside the bay, one each side.

Mechanics fitting a new battery hook each J-bolt into its slot in the tray, lay the bar across the battery's top over
both bolts, then run a nut down each bolt onto the bar (Dorman's hold-down kit instructions).

What makes it hard: each J-bolt is held by its top, a hand's length above the battery, and its hook has to go down
the gap beside the battery into a slot 21 cm below, out of sight, then slide along under the tray's floor to the
slot's far end; the bolt sways and turns in the hand, and its hook has room of a few mm either way. Let go, a bolt
is held only by its hook and leans a few degrees. The bar then has to go down over both leaning bolt tops at once
without knocking either loose, and each nut has to follow its leaning bolt down onto the bar.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_room, on_step  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import dims as d  # noqa: E402
import touch  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Secure the battery with its hold-down: hook each J-bolt into its slot in the battery tray, lay the bar "
               "across the battery's top with both bolts through its slots, then push a nut down each bolt onto the bar.")

TOP = TABLE_TOP[SCENE]
SIDES = {"l": 1.0, "r": -1.0}  # the left bolt's slot is at +y, the right one's at -y


def Z(h: float) -> float:
    """World height of a height above the table top."""
    return TOP + h


# -- the engine bay: a hole in the table and a well under it -----------------------------------------------------
@on_room
def _cut_table(room, scene):
    """The station table with the bay's hole cut out of it: four slabs round the hole."""
    from yam_arena import station

    cx, cy, cz = station.TABLE_CENTER
    sx, sy, sz = station.TABLE_SIZE
    x0, x1, y0, y1 = cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2
    (hx0, hx1), (hy0, hy1) = d.HOLE_X, d.HOLE_Y
    slabs = {"table": ((x0, hx0), (y0, y1)), "table_far": ((hx1, x1), (y0, y1)),
             "table_left": ((hx0, hx1), (hy1, y1)), "table_right": ((hx0, hx1), (y0, hy0))}
    out = [o for o in room if o.name != "table"]
    for name, ((a, b), (c, e)) in slabs.items():
        kw = {"cls": scene._Table} if name == "table" else {}
        out.append(scene._box(name, (b - a, e - c, sz), ((a + b) / 2, (c + e) / 2, cz), scene.STATION_TABLE_COLOR, **kw))
    return out


def box(name, lo, hi, color):
    """A static part, a box spanning lo..hi (x, y, height above the table)."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = (lo + hi) / 2
    return Shape(name, pos=(c[0], c[1], Z(c[2])), size=tuple(hi - lo), color=color)


BAY = (0.20, 0.21, 0.22)  # painted steel, dark
TRAY = (0.12, 0.12, 0.13)  # black-coated steel
CASE = (0.10, 0.10, 0.11)  # black polypropylene battery case
LID = (0.55, 0.56, 0.58)
LEAD = (0.48, 0.48, 0.50)
HOLDER_C = (0.85, 0.45, 0.10)  # orange plastic

(HX0, HX1), (HY0, HY1) = d.HOLE_X, d.HOLE_Y
TABLE_UNDER = -0.06  # the table top's slab is 6 cm thick
_t = d.WELL_WALL_T
WELL = [
    box("well_floor", (HX0 - _t, HY0 - _t, d.FLOOR - 0.010), (HX1 + _t, HY1 + _t, d.FLOOR), BAY),
    box("well_back", (HX0 - _t, HY0 - _t, d.FLOOR), (HX0, HY1 + _t, TABLE_UNDER), BAY),
    box("well_front", (HX1, HY0 - _t, d.FLOOR), (HX1 + _t, HY1 + _t, TABLE_UNDER), BAY),
    box("well_left", (HX0, HY1, d.FLOOR), (HX1, HY1 + _t, TABLE_UNDER), BAY),
    box("well_right", (HX0, HY0 - _t, d.FLOOR), (HX1, HY0, TABLE_UNDER), BAY),
]

# -- the tray, the bracket under it and the battery --------------------------------------------------------------
(PX0, PX1), (SX0, SX1) = d.PLATE_X, d.SLOT_X
_T, _H = d.TRAY_WALL_T, d.TRAY_WALL_H
_wall_top = d.PLATE_TOP + _H
TRAY_PARTS = [
    box("bracket", (d.BAT_X - d.BAT_L / 2 + 0.005, -d.BRACKET_Y, d.FLOOR), (d.BAT_X + d.BAT_L / 2 - 0.005, d.BRACKET_Y,
                                                                           d.PLATE_BOTTOM), BAY),
    box("tray_floor", (PX0, -d.SLOT_IN, d.PLATE_BOTTOM), (PX1, d.SLOT_IN, d.PLATE_TOP), TRAY),
    box("tray_end_back", (PX0 - _T, -d.WALL_IN - _T, d.PLATE_BOTTOM), (PX0, d.WALL_IN + _T, _wall_top), TRAY),
    box("tray_end_front", (PX1, -d.WALL_IN - _T, d.PLATE_BOTTOM), (PX1 + _T, d.WALL_IN + _T, _wall_top), TRAY),
]
for _k, _s in SIDES.items():
    _yin, _yout = sorted((_s * d.SLOT_IN, _s * d.SLOT_OUT))
    _w0, _w1 = sorted((_s * d.WALL_IN, _s * (d.WALL_IN + _T)))
    _l0, _l1 = sorted((_s * d.SLOT_OUT, _s * d.WALL_IN))
    if d.TRAY_LEDGE > 0:
        TRAY_PARTS.append(box(f"tray_ledge_{_k}", (PX0, _l0, d.PLATE_BOTTOM), (PX1, _l1, d.PLATE_TOP), TRAY))
    TRAY_PARTS += [
        box(f"tray_strip_{_k}_back", (PX0, _yin, d.PLATE_BOTTOM), (SX0, _yout, d.PLATE_TOP), TRAY),
        box(f"tray_strip_{_k}_front", (SX1, _yin, d.PLATE_BOTTOM), (PX1, _yout, d.PLATE_TOP), TRAY),
        box(f"tray_wall_{_k}", (PX0, _w0, d.PLATE_BOTTOM), (PX1, _w1, _wall_top), TRAY),
    ]

POST_R, POST_H = 0.0085, 0.010  # SAE taper posts (about 17.5 mm across), cylindrical part
POSTS_X, POSTS_Y = 0.085, -0.055  # the two posts near the battery's front (-y) side, one at each end
BATTERY = [
    box("battery", (d.BAT_X - d.BAT_L / 2, d.BAT_Y - d.BAT_W / 2, d.PLATE_TOP),
        (d.BAT_X + d.BAT_L / 2, d.BAT_Y + d.BAT_W / 2, d.BAT_TOP), CASE),
    box("battery_label", (d.BAT_X - 0.070, -0.030, d.BAT_TOP), (d.BAT_X + 0.070, 0.050, d.BAT_TOP + 0.0005), LID),
    *(Shape(f"post_{n}", kind="capsule", radius=POST_R, height=POST_H, axis="Z",
            pos=(d.BAT_X + s * POSTS_X, POSTS_Y, Z(d.BAT_TOP + POST_H / 2)), color=LEAD)
      for n, s in (("neg", -1), ("pos", 1))),
]

# -- the holders and the bolts standing in them ------------------------------------------------------------------
HX, HY, HZ = d.HOLDER
SLX, SLY = d.HOLDER_SLOT
HOLDERS = []
for _k, _s in SIDES.items():
    _cx, _cy = d.HOLDER_X, _s * d.HOLDER_Y
    _ex, _ey = (HX - SLX) / 2, (HY - SLY) / 2
    HOLDERS += [
        box(f"holder_{_k}_back", (_cx - HX / 2, _cy - SLY / 2, d.FLOOR), (_cx - SLX / 2, _cy + SLY / 2, d.FLOOR + HZ),
            HOLDER_C),
        box(f"holder_{_k}_front", (_cx + SLX / 2, _cy - SLY / 2, d.FLOOR), (_cx + HX / 2, _cy + SLY / 2, d.FLOOR + HZ),
            HOLDER_C),
        box(f"holder_{_k}_in", (_cx - HX / 2, _cy - HY / 2, d.FLOOR), (_cx + HX / 2, _cy - SLY / 2, d.FLOOR + HZ), HOLDER_C),
        box(f"holder_{_k}_out", (_cx - HX / 2, _cy + SLY / 2, d.FLOOR), (_cx + HX / 2, _cy + HY / 2, d.FLOOR + HZ),
            HOLDER_C),
    ]

# a bolt's origin: on its shank's axis at the hook's underside; the hook's middle stands over the holder's slot
BOLT_START_X = d.HOLDER_X - d.J_TIP_X / 2
BOLTS = {k: UsdBody(f"jbolt_{k}", usd="assets/jbolt.usda", pos=(BOLT_START_X, s * d.HOLDER_Y, Z(d.FLOOR) + 0.0003),
                    size=(0.02, 0.007, d.J_LEN), mass=d.J_MASS, material=False, torsional_patch=0.003)
         for k, s in SIDES.items()}

# the bar lies flat on the table to the right of the bay, along x
BAR_START = (d.BAT_X, -0.250, Z(d.BAR_T / 2) + 0.0003)
BAR = UsdBody("bar", usd="assets/bar.usda", pos=BAR_START, size=(d.BAR_LEN, d.BAR_W, d.BAR_T), mass=d.BAR_MASS,
              material=False, torsional_patch=0.008)

# the two 1/4-20 flange nuts lie flange down on the table beside the bay, one each side
NUTS = {k: UsdBody(f"nut_{k}", usd="assets/nut.usda", pos=(d.NUT_X, s * d.NUT_Y, Z(0.0) + 0.0003),
                   size=(d.NUT_FLANGE_D, d.NUT_FLANGE_D, d.NUT_H), mass=d.NUT_MASS, material=False,
                   torsional_patch=0.006)
        for k, s in SIDES.items()}

OBJECTS = [*WELL, *TRAY_PARTS, *BATTERY, *HOLDERS, *BOLTS.values(), BAR, *NUTS.values()]

# -- scoring -----------------------------------------------------------------------------------------------------
SEAT_TOL = 3.0 * d.MM  # the bar's underside within this of the battery's top...
LEVEL = math.radians(5.0)  # ... and within this of level
NUT_DOWN = 2.0 * d.MM  # a nut is down when its flange is within this of the bar's top...
NUT_LEVEL = math.radians(10.0)  # ... square to its bolt within this
_STATE: dict = {"free": True, "t": 0.0, "last": -1.0}


def on_axis(b, z: float):
    """The point at world height z on a bolt's shank axis (None if the bolt lies within 60 degrees of flat)."""
    a = b.rotation()[:, 2]
    if a[2] < 0.5:
        return None
    o = np.asarray(b.pos, float)
    return o + (z - o[2]) / a[2] * a


def tip_top(b) -> np.ndarray:
    """World position of the top of a bolt's hook tip."""
    return np.asarray(b.pos, float) + b.rotation() @ np.array([d.J_TIP_X, 0.0, d.J_TIP_H])


def hooked(b, side: float) -> bool:
    """The bolt's shank passes through the slot on its side and its hook's tip is under the tray's floor, past the
    slot's edge: lifted, the bolt would stop against the tray."""
    s = on_axis(b, Z(d.PLATE_TOP - d.PLATE_T / 2))
    if s is None:
        return False
    sy = side * s[1]
    if not (SX0 <= s[0] <= SX1 and d.SLOT_IN <= sy <= d.SLOT_OUT):
        return False
    q = tip_top(b)
    qy = side * q[1]
    if q[2] >= Z(d.PLATE_BOTTOM) or not (PX0 <= q[0] <= PX1 and 0.0 <= qy <= d.WALL_IN + d.TRAY_WALL_T):
        return False
    # the tip, ROD_R round, clear of the slot's opening: under solid floor
    dx = max(SX0 - q[0], 0.0, q[0] - SX1)
    dy = max(d.SLOT_IN - qy, 0.0, qy - d.SLOT_OUT)
    return math.hypot(dx, dy) > d.ROD_R


def through_slots(bar, bolts) -> bool:
    """Each bolt's shank passes through a different one of the bar's two slots."""
    r = bar.rotation()
    slots = set()
    for b in bolts:
        p = on_axis(b, float(bar.pos[2]))
        if p is None:
            return False
        local = r.T @ (p - np.asarray(bar.pos, float))
        for n, s in enumerate((1.0, -1.0)):
            if abs(local[0] - s * d.BAR_HOLE_Y) < d.BAR_SLOT / 2 and abs(local[1]) < d.BAR_HOLE / 2:
                slots.add(n)
                break
        else:
            return False
    return len(slots) == len(bolts)


def seated(bar) -> bool:
    """The bar lies on the battery's top, level."""
    up = bar.rotation()[:, 2]
    bottom = float(bar.pos[2]) - d.BAR_T / 2
    return up[2] > math.cos(LEVEL) and abs(bottom - Z(d.BAT_TOP)) < SEAT_TOL


def nut_down(nut, bolt, bar) -> bool:
    """The nut is on the bolt (the bolt's axis through its bore) and pushed down onto the bar: its flange within
    NUT_DOWN of the bar's top, square to the bolt."""
    up = nut.rotation()[:, 2]
    if up[2] < math.cos(NUT_LEVEL):
        return False
    o = np.asarray(nut.pos, float)
    p = on_axis(bolt, float(o[2]))
    if p is None or math.hypot(p[0] - o[0], p[1] - o[1]) > d.NUT_BORE / 2:
        return False
    return abs(float(o[2]) - (float(bar.pos[2]) + d.BAR_T / 2)) < NUT_DOWN


def score(objects) -> float:
    """0: neither J-bolt hooked into its slot.
    0.2: one J-bolt hooked.
    0.4: both hooked; the bar not on the battery's top with both bolts through its slots.
    0.6: both hooked and the bar on the battery's top, level, both bolts through its slots; not both nuts down.
    0.8: so, and a nut pushed down each bolt onto the bar, but the robot still touching a bolt, the bar or a nut.
    1.0: so, and the robot off them."""
    n = sum(hooked(objects[f"jbolt_{k}"], s) for k, s in SIDES.items())
    if n == 0:
        return 0.0
    if n == 1:
        return 0.2
    bar = objects["bar"]
    bolts = [objects["jbolt_l"], objects["jbolt_r"]]
    if not (seated(bar) and through_slots(bar, bolts)):
        return 0.4
    nuts = [objects.get("nut_l"), objects.get("nut_r")]
    # each bolt needs a nut down on it (either nut on either bolt)
    if any(n is None for n in nuts) or not (
            (nut_down(nuts[0], bolts[0], bar) and nut_down(nuts[1], bolts[1], bar))
            or (nut_down(nuts[0], bolts[1], bar) and nut_down(nuts[1], bolts[0], bar))):
        return 0.6
    return 1.0 if _STATE["free"] else 0.8


def success(objects) -> bool:
    return score(objects) == 1.0


@on_reset
def _reset(environment) -> None:
    touch.reset()
    _STATE.update(free=True, t=0.0, last=-1.0)


@on_step
def _watch(environment) -> None:
    """Released: no robot link touches a bolt, the bar or a nut (PhysX contacts, touch.py)."""
    _STATE["free"] = not any(touch.touching(environment, ["jbolt_l", "jbolt_r", "bar", "nut_l", "nut_r"]).values())
    _STATE["t"] += environment.unwrapped.step_dt
    if os.environ.get("BATTERY_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
        _STATE["last"] = _STATE["t"]
        from yam_arena.scene import object_states

        s = object_states(environment, ["jbolt_l", "jbolt_r", "bar", "nut_l", "nut_r"])
        parts = []
        for k, side in SIDES.items():
            b = s[f"jbolt_{k}"]
            o = (np.asarray(b.pos) - np.array([d.BAT_X, 0.0, TOP])) * 1000
            lean = math.degrees(math.acos(max(-1.0, min(1.0, b.rotation()[2, 2]))))
            turn = math.degrees(math.atan2(b.rotation()[1, 0], b.rotation()[0, 0]))
            parts.append(f"{k}: {o[0]:+.1f} {o[1]:+.1f} {o[2]:+.1f} mm lean {lean:.1f} turn {turn:+.1f} "
                         f"hooked={hooked(b, side)}")
        bar = s["bar"]
        bo = (np.asarray(bar.pos) - np.array([d.BAT_X, 0.0, TOP])) * 1000
        nuts = []
        for k in SIDES:
            nn = (np.asarray(s[f"nut_{k}"].pos) - np.array([d.BAT_X, 0.0, TOP])) * 1000
            down = nut_down(s[f"nut_{k}"], s[f"jbolt_{k}"], bar)
            nuts.append(f"nut_{k} {nn[0]:+.1f} {nn[1]:+.1f} {nn[2]:+.1f}{' down' if down else ''}")
        print(f"LOG t={_STATE['t']:.1f} {'  '.join(parts)}  bar {bo[0]:+.1f} {bo[1]:+.1f} {bo[2]:+.1f} mm "
              f"seated={seated(bar)}  {'  '.join(nuts)}  free={_STATE['free']}  score={score(s)}", flush=True)


# -- two places in a slot the checks below use: a hook straight down through it, and slid on under the floor ----
DROP_U = (d.SLOT_LEN - d.J_TIP_X) / 2  # the shank this far into the slot from its back (-x) end: the whole hook,
# 18.7 mm across, goes straight down through the 28 mm slot with 4.65 mm to spare either end
HOOK_U = d.SLOT_LEN - d.ROD_R - 0.0005  # then on until the shank is at the slot's front end: the tip is under the
# floor, and let go, the bolt can't lean forwards (the slot's end) or back (the tip meets the floor's underside)


def self_test(ObjectState=None):
    """The scoring rules on hand-built states."""
    from scenekit import BodyState

    def qz(yaw):
        return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])

    def bolt(side, x, hook_z=d.FLOOR, y=None, yaw=0.0, lean_y=0.0):
        """A bolt with its shank at (x, y) at the tray floor's height and its hook's underside at hook_z; lean_y tips
        its top towards +y (rad)."""
        y = side * d.SLOT_YC if y is None else y
        q = qz(yaw)
        if lean_y:
            qx = np.array([-math.sin(lean_y / 2), 0.0, 0.0, math.cos(lean_y / 2)])  # about -x: top towards +y
            # compose qz * qx (xyzw)
            x1, y1, z1, w1 = q
            x2, y2, z2, w2 = qx
            q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                          w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        b0 = BodyState(pos=np.zeros(3), quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.zeros(3),
                       high=np.zeros(3))
        a = b0.rotation()[:, 2]
        z_plate = Z(d.PLATE_TOP - d.PLATE_T / 2)
        pos = np.array([x, y, z_plate]) - a * (z_plate - Z(hook_z)) / a[2]
        return BodyState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)

    def bar(x=SX0 + HOOK_U, y=0.0, h=d.BAT_TOP + d.BAR_T / 2, yaw=math.pi / 2):
        pos = np.array([x, y, Z(h)])
        return BodyState(pos=pos, quat_xyzw=qz(yaw), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)

    def nut(x, y, h, tilt=0.0):
        q = np.array([math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)])
        pos = np.array([x, y, Z(h)])
        return BodyState(pos=pos, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos)

    hook_x, drop_x = SX0 + HOOK_U, SX0 + DROP_U
    off = bar(*BAR_START[:2], h=d.BAR_T / 2, yaw=0.0)
    start = {k: bolt(s, BOLT_START_X, y=s * d.HOLDER_Y) for k, s in SIDES.items()}
    hk = {k: bolt(s, hook_x) for k, s in SIDES.items()}
    nut_table = {f"nut_{k}": nut(d.NUT_X, s * d.NUT_Y, 0.0) for k, s in SIDES.items()}
    on_bar = d.BAT_TOP + d.BAR_T
    nut_on = {f"nut_{k}": nut(hook_x, s * d.SLOT_YC, on_bar) for k, s in SIDES.items()}
    full = {"jbolt_l": hk["l"], "jbolt_r": hk["r"], "bar": bar()}
    cases = {
        "as set up": ({"jbolt_l": start["l"], "jbolt_r": start["r"], "bar": off}, True, 0.0),
        "left hook down through its slot, not slid": ({"jbolt_l": bolt(1, drop_x), "jbolt_r": start["r"], "bar": off},
                                                      True, 0.0),
        "left hooked": ({"jbolt_l": hk["l"], "jbolt_r": start["r"], "bar": off}, True, 0.2),
        "both hooked": ({"jbolt_l": hk["l"], "jbolt_r": hk["r"], "bar": off}, True, 0.4),
        "both hooked, lifted 1 mm (tip still under the floor)": (
            {"jbolt_l": bolt(1, hook_x, d.FLOOR + 0.001), "jbolt_r": hk["r"], "bar": off}, True, 0.4),
        "both hooked, one leaning out 3 degrees": (
            {"jbolt_l": bolt(1, hook_x, lean_y=math.radians(3)), "jbolt_r": hk["r"], "bar": off}, True, 0.4),
        "bar on, both through, still held": ({"jbolt_l": hk["l"], "jbolt_r": hk["r"], "bar": bar()}, False, 0.6),
        "bar on, both through, released": ({"jbolt_l": hk["l"], "jbolt_r": hk["r"], "bar": bar()}, True, 0.6),
        "bar on, slots swapped end for end": ({"jbolt_l": hk["l"], "jbolt_r": hk["r"], "bar": bar(yaw=-math.pi / 2), **nut_on},
                                              True, 1.0),
        "bar on, 6 mm off along x (bolts beside its slots)": (
            {"jbolt_l": hk["l"], "jbolt_r": hk["r"], "bar": bar(x=hook_x + 0.006)}, True, 0.4),
        "bar resting on the bolt tops": ({"jbolt_l": hk["l"], "jbolt_r": hk["r"],
                                          "bar": bar(h=d.BOLT_TOP + d.BAR_T / 2)}, True, 0.4),
        "bar on, one bolt slipped back (hook under the slot)": (
            {"jbolt_l": bolt(1, drop_x), "jbolt_r": hk["r"], "bar": bar(x=hook_x)}, True, 0.2),
        "a bolt hooked but turned half round (tip to -x)": (
            {"jbolt_l": bolt(1, SX1 - (hook_x - SX0), yaw=math.pi), "jbolt_r": hk["r"], "bar": off}, True, 0.4),
        "a bolt lying flat": ({"jbolt_l": BodyState(pos=np.array([0.3, 0.2, Z(0.003)]), quat_xyzw=np.array(
            [0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
            low=np.zeros(3), high=np.zeros(3)), "jbolt_r": hk["r"], "bar": off}, True, 0.2),
        "bar on, both nuts down, still held": ({**full, **nut_on}, False, 0.8),
        "bar on, both nuts down, released": ({**full, **nut_on}, True, 1.0),
        "bar on, nuts swapped side for side, down": ({**full, "nut_l": nut_on["nut_r"], "nut_r": nut_on["nut_l"]},
                                                     True, 1.0),
        "bar on, one nut down, the other on the table": ({**full, "nut_l": nut_on["nut_l"],
                                                          "nut_r": nut_table["nut_r"]}, True, 0.6),
        "bar on, both nuts on their bolts 10 mm above the bar": (
            {**full, **{k: nut(hook_x, (1 if k == "nut_l" else -1) * d.SLOT_YC, on_bar + 0.010) for k in nut_on}},
            True, 0.6),
        "bar on, a nut beside its bolt on the bar": ({**full, "nut_l": nut(hook_x + 0.015, d.SLOT_YC, on_bar),
                                                      "nut_r": nut_on["nut_r"]}, True, 0.6),
        "bar on, a nut tipped 30 degrees": ({**full, "nut_l": nut(hook_x, d.SLOT_YC, on_bar, math.radians(30)),
                                             "nut_r": nut_on["nut_r"]}, True, 0.6),
        "nuts down but one bolt unhooked": ({"jbolt_l": bolt(1, drop_x), "jbolt_r": hk["r"], "bar": bar(),
                                             **nut_on}, True, 0.2),
        "a bolt pulled out up through its slot": (
            {"jbolt_l": bolt(1, hook_x, d.PLATE_TOP + 0.005), "jbolt_r": hk["r"], "bar": off}, True, 0.2),
    }
    for name, (objs, free, want) in cases.items():
        objs = {**nut_table, **objs}
        _STATE["free"] = free
        got = score(objs)
        assert abs(got - want) < 1e-9, (name, got, want)
    _STATE["free"] = True
    # sizes against each other
    hook_w = d.J_TIP_X + d.ROD_D
    assert hook_w < d.SLOT_LEN and d.ROD_D < d.SLOT_W and d.ROD_D < d.BAR_HOLE
    assert hook_w < d.HOLDER_SLOT[0] and d.ROD_D < d.HOLDER_SLOT[1]
    assert d.J_TIP_H < d.BRACKET_H  # a hook resting on the well's floor keeps its tip under the tray's floor
    assert d.BOLT_TOP > d.BAT_TOP + d.BAR_T + 0.020  # ... and its top 2 cm or more above the bar
    assert d.SLOT_IN > d.BAT_W / 2 and d.BRACKET_Y < d.SLOT_IN - d.ROD_D  # the slot outside the battery and the bracket
    assert DROP_U >= d.ROD_R and DROP_U + hook_w - d.ROD_R <= d.SLOT_LEN  # down through the slot
    assert HOOK_U + d.ROD_R <= d.SLOT_LEN and HOOK_U + d.J_TIP_X - d.ROD_R > d.SLOT_LEN  # hooked
    assert d.ROD_D < d.NUT_BORE and d.NUT_FLANGE_D > d.BAR_HOLE  # the nut goes on the bolt; it cannot pass the slot
    assert d.BOLT_TOP > d.BAT_TOP + d.BAR_T + d.NUT_H + 0.010  # the bolt reaches above a nut down on the bar
    return len(cases) + 9


if __name__ == "__main__":
    # Offline check, no GPU: the scoring rules on hand-built states and the sizes against each other.
    print(f"self-test: {self_test()} cases pass")
