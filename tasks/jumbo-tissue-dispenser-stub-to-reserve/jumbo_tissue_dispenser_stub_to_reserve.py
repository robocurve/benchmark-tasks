"""A twin jumbo-roll toilet-tissue dispenser hangs on a vertical board at the back of the bench, in its normal wall
orientation. Its two spindles stick out horizontally from the back plate, side by side: the primary (left) still
carries an almost used-up stub; the reserve (right) is empty. Each spindle is a split hub that turns on a bearing,
so a loaded roll spins as tissue is pulled: a fixed upper half that a core hangs on, and a sprung lower half that
presses the core up against it. The front cover is hinged along the top edge and held open, swung up, by a stay. Its
front panel has a tissue outlet window over each roll along the bottom; a red changeover shutter slides along the
outside of the panel and covers one window, so tissue can be drawn from one roll at a time. As found, it covers the
reserve window and the primary (stub) is the one dispensing. A new 230 mm jumbo roll lies on its carton on the bench
to the left, a waste bin in front of it.

The job: the stub goes onto the reserve spindle and the new roll onto the primary, the cover is closed until its
latch clicks, and the changeover shutter is slid across so only the stub's window is open and the stub is used up
first. Each hub only lets a core on or off while its release button on the bottom lip is held down (the sprung half
draws in), so one hand holds the button while the other slides the core along the spindle. Once its stay is
released the cover closes under its own weight against a soft-close damper; it may also be closed by hand, but
slammed shut (over 6 rad/s near closed) it fails.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory. The demo
uses a few grips with the fingers pointing horizontally (toward the board); this file adds that option to the stock
scripted waypoints in memory (``_Ang`` below).

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/jumbo-tissue-dispenser-stub-to-reserve/jumbo_tissue_dispenser_stub_to_reserve.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 2400 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

import yam_arena.demo as _ydemo  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WHITE = (0.92, 0.92, 0.90)
BOARD = (0.70, 0.66, 0.58)
SMOKE = (0.30, 0.32, 0.36)  # smoked cover
GREY = (0.55, 0.56, 0.58)
BLUE = (0.15, 0.35, 0.75)
RED = (0.85, 0.15, 0.12)
PAPER = (0.96, 0.95, 0.90)
BIN = (0.20, 0.22, 0.24)

# -- housing: a 300 (tall) x 490 (wide) mm back plate on the board, 30 mm rim ---------------------------------------
XW = 0.400  # front face of the back plate (the board is behind it)
ZB, ZT = TOP + 0.004, TOP + 0.268  # housing bottom and top
YH = 0.245
RIM_X = XW - 0.030  # front edge of the rim: the cover's hinge line is here, along the top
WALL = 0.004
CD = 0.074  # cover depth

# -- rolls: 75 mm cores, 90 mm long; the spindles stick out along -x -----------------------------------------------
R_I = 0.0375
ROLL_RO, STUB_RO, ROLL_H = 0.115, 0.0415, 0.090
Z_AX = 0.890  # spindle axes
PRIM_Y, RES_Y = 0.120, -0.120
HUBS = {"p": PRIM_Y, "r": RES_Y}
Q_AX = tuple(mk.mat_to_quat(mk.axis_angle((0, 1, 0), -math.pi / 2)))  # a core's local z (bottom to top) along -x


def _ring(ro, h, color_out, n=12):
    """A 12-sided ring (core + paper) round a 75 mm core hole, frame at the bottom centre, axis along local z."""
    t = ro - R_I
    w = 2 * ro * math.tan(math.pi / n)
    out = []
    for i in range(n):
        a = 2 * math.pi * i / n
        rc = (ro + R_I) / 2
        out.append(mk.box((t, w, h), at=(rc * math.cos(a), rc * math.sin(a), h / 2), yaw=a, color=color_out))
    return out


kit = mk.Kit()

# hubs. Each turns on a free bearing about its spindle axis: a flat bearing boss on the back plate, and the hub on a
# revolute joint round it. The hub is two lugs 29 mm long: the fixed upper lug (radially 22.5 to 37.5 mm above the
# axis, so a 75 mm core hangs centred on it) and the sprung lower lug (32.5 to 43.5 mm below the axis at rest, 4.8 cm
# travel upward), which blocks a core unless drawn in and then presses a seated core up onto the fixed lug.
LUG_X = XW - 0.001 - 0.0145  # lug centre (x): from 1 mm to 30 mm in front of the plate
LUG_FRONT = XW - 0.030
JAW_TRAVEL, JAW_OPEN_AT = 0.048, 0.036
TAB_TRAVEL, TAB_PRESSED = 0.006, 0.004
HUB_SPIN_DAMP, HUB_BRAKE = 0.01, 0.03  # bearing drag (N*m*s/rad) and roll brake (N*m): a 2 N tissue pull on a
# full roll is 0.23 N*m, so it turns; the hub does not swing round under its own weight (0.012 N*m at most)
LIP_Z = TOP + 0.016  # top of the bottom lip that carries the release buttons
TAB_X = 0.150  # 25 cm forward of the plate: the cover's bottom front corner sweeps down to x 0.20 just above the lip
# the two hub release buttons on the bottom lip: the primary's right of centre, the reserve's under the right end
# (bases: z of the base, height)
TABS = {"p": (TAB_X, -0.055), "r": (TAB_X, -0.225)}
TAB_BASE = {"p": (LIP_Z, 0.010), "r": (LIP_Z, 0.010)}
for k, hy in HUBS.items():
    kit.part(f"spindle_{k}", (XW, hy, Z_AX), [mk.box((0.0008, 0.020, 0.020), at=(-0.0004, 0, 0))], mass=None,
             color=SMOKE)
    kit.part(f"hub_fix_{k}", (XW, hy, Z_AX), [mk.box((0.029, 0.024, 0.015), at=(LUG_X - XW, 0, 0.030))], mass=0.040,
             color=GREY)
    kit.hinge(f"hub_{k}_spin", f"spindle_{k}", f"hub_fix_{k}", anchor=(XW, hy, Z_AX), axis=(-1, 0, 0),
              lower_deg=-math.inf, upper_deg=math.inf, damping=HUB_SPIN_DAMP,
              friction=HUB_BRAKE)
    kit.part(f"jaw_{k}", (XW, hy, Z_AX), [mk.box((0.029, 0.024, 0.011), at=(LUG_X - XW, 0, -0.038))], mass=0.020,
             color=GREY)
    kit.slider(f"jaw_{k}_slide", f"hub_fix_{k}", f"jaw_{k}", axis=(0, 0, 1), lower=0.0, upper=JAW_TRAVEL,
               start=0.006 if k == "p" else 0.0, stiffness=500.0, rest=-0.004, damping=8.0)
    tx, ty = TABS[k]
    bz, bh = TAB_BASE[k]
    kit.part(f"tab_base_{k}", (tx, ty, bz), [mk.box((0.036, 0.036, bh), at=(0, 0, bh / 2))], mass=None,
             color=GREY)
    kit.part(f"tab_{k}", (tx, ty, bz + bh), [mk.box((0.032, 0.032, 0.012), at=(0, 0, 0.006))], mass=0.020,
             color=BLUE)
    kit.slider(f"tab_{k}_push", f"tab_base_{k}", f"tab_{k}", axis=(0, 0, -1), lower=0.0, upper=TAB_TRAVEL,
               stiffness=400.0, rest=-0.002, damping=2.0)

# the stub on the primary hub (held there until its button is pressed) and the new roll standing on the bench
STUB0 = (XW, PRIM_Y, Z_AX)
# the new roll lies on its side on its 72 mm carton, axis along x, front end toward the robot
CARTON = (0.305, 0.420, 0.072)
NEW_AX = TOP + CARTON[2] + ROLL_RO + 0.001  # resting on one of its 12 flats
NEW_FRONT = 0.260
NEW0 = (NEW_FRONT + ROLL_H, CARTON[1], NEW_AX)
kit.part("stub", STUB0, _ring(STUB_RO, ROLL_H, PAPER), mass=0.15, color=PAPER, quat=Q_AX)
kit.bond("stub_hold", "hub_fix_p", "stub", anchor=(LUG_X, PRIM_Y, Z_AX))
kit.part("roll", NEW0, _ring(ROLL_RO, ROLL_H, PAPER), mass=0.90, color=PAPER, quat=Q_AX)

# cover: a 490 x 300 mm shell 74 mm deep, hinged along the top front edge of the rim. Opening swings its free
# (bottom) edge out and up; it is held at 170 deg (swung up, just short of upright) by a stay until the stay's button is pressed. Its weight, a light
# closing spring and a rotary damper then lower it onto its latch.
HINGE = (RIM_X, 0.0, ZT)
CH = ZT - ZB
OPEN_DEG = 170.0
OUT_Z = (0.030, 0.085)  # outlet windows from 3 to 8.5 cm above the cover's bottom edge
OUT_Y = ((-0.200, -0.040), (0.040, 0.200))  # reserve (right) window, primary (left) window
_px = -(CD - 0.002)  # front panel centre, cover frame
_wz0, _wz1 = -CH + OUT_Z[0], -CH + OUT_Z[1]
_cover = [mk.box((0.004, 2 * YH, CH - OUT_Z[1]), at=(_px, 0, -(CH - OUT_Z[1]) / 2)),  # panel above the windows
          mk.box((0.004, 2 * YH, OUT_Z[0]), at=(_px, 0, -CH + OUT_Z[0] / 2)),  # strip below them
          *[mk.box((0.004, b - a, _wz1 - _wz0), at=(_px, (a + b) / 2, (_wz0 + _wz1) / 2))
            for a, b in ((-YH, OUT_Y[0][0]), (OUT_Y[0][1], OUT_Y[1][0]), (OUT_Y[1][1], YH))],
          mk.box((CD - 0.005, 2 * YH, WALL), at=(-0.001 - (CD - 0.005) / 2, 0, -WALL / 2)),  # top
          mk.box((CD - 0.005, 2 * YH, WALL), at=(-0.001 - (CD - 0.005) / 2, 0, -CH + WALL / 2)),  # bottom
          mk.box((CD - 0.005, WALL, CH), at=(-0.001 - (CD - 0.005) / 2, YH - WALL / 2, -CH / 2)),
          mk.box((CD - 0.005, WALL, CH), at=(-0.001 - (CD - 0.005) / 2, -YH + WALL / 2, -CH / 2)),
          ]
kit.part("cover", HINGE, _cover, mass=0.50, color=SMOKE)
kit.hinge("cover_hinge", None, "cover", anchor=HINGE, axis=(0, 1, 0), lower_deg=0.0, upper_deg=OPEN_DEG,
          start_deg=OPEN_DEG, stiffness=0.2, rest_deg=-10.0, damping=3.0)
kit.catch("cover_latch", "cover", kit.parts["cover"].ref)

# changeover shutter: a plate with a knob, sliding along the outside of the front panel. Detents over the reserve
# window (0, as found: the primary feeds) and 24 cm toward +y over the primary window (the reserve feeds).
SHUT_TRAVEL = OUT_Y[1][0] - OUT_Y[0][0]
KNOB_L = 0.060  # the shutter's thumb tab sticks out 6 cm from the panel, so a hand can get beside it
_sz = (_wz0 + _wz1) / 2
SHUT_W = OUT_Y[0][1] - OUT_Y[0][0] + 0.010
kit.part("shutter", HINGE, [mk.box((0.003, SHUT_W, OUT_Z[1] - OUT_Z[0] + 0.008), at=(-CD - 0.0025, -0.120, _sz)),
                            mk.box((KNOB_L, 0.012, 0.020), at=(-CD - 0.004 - KNOB_L / 2, -0.120, _sz))],
         mass=0.030, color=RED)
kit.slider("shutter_slide", "cover", "shutter", axis=(0, 1, 0), lower=0.0, upper=SHUT_TRAVEL, stiffness=60.0,
           rest=-0.003, damping=1.0)
KNOB0 = (RIM_X - CD - 0.004 - KNOB_L / 2, -0.120, ZT + _sz)  # knob centre with the cover shut, shutter as found


def _cover_T(q_deg):
    """The cover's world transform with the hinge at q_deg."""
    R = np.eye(4)
    R[:3, :3] = mk.axis_angle((0, 1, 0), math.radians(q_deg))
    T0, T1 = np.eye(4), np.eye(4)
    T0[:3, 3], T1[:3, 3] = -np.asarray(HINGE), np.asarray(HINGE)
    return T1 @ R @ T0 @ kit.parts["cover"].ref


kit.catch("cover_stay", "cover", _cover_T(OPEN_DEG))
# stay release button: on the bottom lip, between the two hub release buttons
STAY = (TAB_X, 0.0)
STAY_H = 0.010
kit.part("stay_base", (*STAY, LIP_Z), [mk.box((0.018, 0.018, STAY_H), at=(0, 0, STAY_H / 2))], mass=None,
         color=GREY)
kit.part("stay_btn", (*STAY, LIP_Z + STAY_H), [mk.box((0.016, 0.016, 0.012), at=(0, 0, 0.006))], mass=0.020,
         color=BLUE)
kit.slider("stay_push", "stay_base", "stay_btn", axis=(0, 0, -1), lower=0.0, upper=TAB_TRAVEL, stiffness=400.0,
           rest=-0.002)

BIN_XY, BIN_W, BIN_H = (0.090, 0.220), 0.110, 0.050  # clear of the arc the cover's bottom front corner sweeps


def _bin():
    x, y = BIN_XY
    w, t, h = BIN_W, 0.006, BIN_H
    return [Box("bin_a", pos=(x + (w - t) / 2, y, TOP + h / 2), size=(t, w, h), color=BIN),
            Box("bin_b", pos=(x - (w - t) / 2, y, TOP + h / 2), size=(t, w, h), color=BIN),
            Box("bin_c", pos=(x, y + (w - t) / 2, TOP + h / 2), size=(w - 2 * t, t, h), color=BIN),
            Box("bin_d", pos=(x, y - (w - t) / 2, TOP + h / 2), size=(w - 2 * t, t, h), color=BIN)]


_ZM, _RZ = (ZB + ZT) / 2, ZT - ZB
FIXTURES = [
    Box("wall_board", pos=(XW + 0.016, 0, (TOP + ZT) / 2), size=(0.020, 0.56, ZT - TOP), color=BOARD),
    Box("back_plate", pos=(XW + 0.003, 0, _ZM), size=(0.006, 2 * YH, _RZ), color=WHITE),
    Box("rim_top", pos=((RIM_X + XW) / 2, 0, ZT - WALL / 2), size=(XW - RIM_X, 2 * YH, WALL), color=WHITE),
    Box("rim_bottom", pos=((RIM_X + XW) / 2, 0, ZB + WALL / 2), size=(XW - RIM_X, 2 * YH, WALL), color=WHITE),
    Box("rim_left", pos=((RIM_X + XW) / 2, YH - WALL / 2, _ZM), size=(XW - RIM_X, WALL, _RZ - 2 * WALL), color=WHITE),
    Box("rim_right", pos=((RIM_X + XW) / 2, -YH + WALL / 2, _ZM), size=(XW - RIM_X, WALL, _RZ - 2 * WALL),
        color=WHITE),
    # the bottom lip in front of the housing that carries the two release buttons
    Box("lip", pos=(TAB_X, -0.075, (TOP + LIP_Z) / 2), size=(0.038, 0.330, LIP_Z - TOP), color=WHITE),
    Box("carton", pos=(CARTON[0], CARTON[1], TOP + CARTON[2] / 2), size=(0.110, 0.200, CARTON[2]), color=(0.62, 0.50, 0.36)),
    *_bin(),
]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- rules ------------------------------------------------------------------------------------------------------
# a slam: the cover still closing at over 6 rad/s within 15 deg of shut, i.e. its bottom edge (30 cm from the hinge)
# meets the rim at over 1.8 m/s. Its weight, spring and damper alone close it at under 1 rad/s; a hand closing it,
# even briskly (1 to 3 rad/s), is far below this; reaching 6 rad/s against the 3 N*m*s/rad damper takes a blow of
# 60 N or more at the bottom edge.
SLAM_RAD_S, SLAM_ZONE_DEG = 6.0, 15.0
DT = 0.05  # one control step at 20 Hz


def _hub_rule(objects):
    for k in HUBS:
        pressed = kit.q(objects, f"tab_{k}_push") >= TAB_PRESSED
        if mk.MEMORY.get(f"pressed_{k}") != pressed:
            mk.MEMORY[f"pressed_{k}"] = pressed
            kit.drive(f"jaw_{k}_slide", JAW_TRAVEL + 0.004 if pressed else -0.004)
        if pressed and k == "p" and "stub_hold" not in kit._released:
            kit.release("stub_hold")


def _shutter_rule(objects):
    det = SHUT_TRAVEL + 0.003 if kit.q(objects, "shutter_slide") >= SHUT_TRAVEL / 2 else -0.003
    if mk.MEMORY.get("shut_det") != det:
        mk.MEMORY["shut_det"] = det
        kit.drive("shutter_slide", det)


def _disengage(name):
    """Open catch ``name`` now (sim); every catch is disarmed again at reset."""
    path = mk._SIM["catch_paths"].get((kit.id, name))
    if path and mk._SIM["stage"] is not None:
        from pxr import UsdPhysics

        UsdPhysics.Joint(mk._SIM["stage"].GetPrimAtPath(path)).GetJointEnabledAttr().Set(False)


def _cover_local(p, q_deg):
    """A world point in the cover's frame (hinge at origin; front panel inner face at x = -(CD - 0.004))."""
    R = mk.axis_angle((0, 1, 0), math.radians(q_deg))
    return R.T @ (np.asarray(p, float) - np.asarray(HINGE))


def _cover_rule(objects):
    m = mk.MEMORY
    m["n"] = m.get("n", 0) + 1
    q = kit.q_deg(objects, "cover_hinge")
    if m["n"] == 1:
        kit.engage("cover_stay")
    if not m.get("stay_off") and kit.q(objects, "stay_push") >= TAB_PRESSED:
        _disengage("cover_stay")
        m["stay_off"] = True
    if m.get("latched"):
        return
    prev = m.get("q_prev")
    m["q_prev"] = q
    w = math.radians(prev - q) / DT if prev is not None else 0.0  # closing speed, rad/s
    if m["n"] > 10 and q < SLAM_ZONE_DEG and w > SLAM_RAD_S:
        m["slammed"] = True  # the bottom edge hits the rim at over 1.8 m/s: a slam, not a hand closing it
    if q < 1.5:
        kit.engage("cover_latch")
        m["latched"] = True
        return
    if q < 45.0:  # cover down on a core that is not on its hub: forced shut over a mis-hung roll
        for n in ("stub", "roll"):
            if seated(objects, n, "p") or seated(objects, n, "r"):
                continue
            st = objects[n]
            lo, hi = np.asarray(st.low), np.asarray(st.high)
            for c in [(a, b, d) for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for d in (lo[2], hi[2])]:
                lc = _cover_local(c, q)
                if lc[0] < -(CD - 0.004) + 0.001 and -CH < lc[2] < 0.0 and abs(lc[1]) < YH and lc[0] > -CD - 0.03:
                    m["crushed"] = True


def _latch_rule(objects):
    for n in ("stub", "roll"):
        c = kit.point(objects, n, (0, 0, ROLL_H / 2))
        if float(c[2]) < TOP - 0.05:
            mk.MEMORY["floor"] = True
    c = kit.point(objects, "stub", (0, 0, ROLL_H / 2))
    if abs(float(c[0]) - BIN_XY[0]) < BIN_W / 2 and abs(float(c[1]) - BIN_XY[1]) < BIN_W / 2 \
            and float(c[2]) < TOP + BIN_H:
        mk.MEMORY["trashed"] = True


def _axis_off(objects, part, hy) -> float:
    """Distance (m) from the core's axis, at its back end, to the spindle axis, across the spindle."""
    b = kit.point(objects, part, (0, 0, 0))
    return math.hypot(float(b[1]) - hy, float(b[2]) - Z_AX)


def _push_rule(objects):
    """Remember each hub's sprung half drawn in (its button held) while its new core was on that spindle."""
    for k, n in (("r", "stub"), ("p", "roll")):
        b = kit.point(objects, n, (0, 0, 0))
        if (kit.q(objects, f"jaw_{k}_slide") >= JAW_OPEN_AT and _axis_off(objects, n, HUBS[k]) < 0.040
                and float(b[0]) > XW - 0.040):
            mk.MEMORY[f"pushed_{k}"] = True


def _debug_rule(objects):
    if os.environ.get("JUMBO_DEBUG") and mk.MEMORY.get("n", 0) % 20 == 0:
        f = lambda v: "(" + ",".join(f"{float(c):.4f}" for c in v) + ")"
        print(f"[jumbo] n={mk.MEMORY.get('n')} stub={f(kit.point(objects, 'stub', (0, 0, 0)))} "
              f"roll={f(kit.point(objects, 'roll', (0, 0, 0)))} cover={kit.q_deg(objects, 'cover_hinge'):.1f} "
              f"shut={kit.q(objects, 'shutter_slide'):.3f} jaw_p={kit.q(objects, 'jaw_p_slide'):.4f} "
              f"jaw_r={kit.q(objects, 'jaw_r_slide'):.4f} hubs={kit.q_deg(objects, 'hub_p_spin'):.1f},"
              f"{kit.q_deg(objects, 'hub_r_spin'):.1f} tcps={[f(t) for t in mk.tcps()]} "
              f"mem={ {k: v for k, v in mk.MEMORY.items() if k not in ('n', 'q_prev')} } "
              f"score={score(objects)}", flush=True)


kit.on_step(_hub_rule)
kit.on_step(_push_rule)
kit.on_step(_shutter_rule)
kit.on_step(_cover_rule)
kit.on_step(_latch_rule)
kit.on_step(_debug_rule)


SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------
_TEST_TCPS = None
# a core all the way on: its back end within 3 mm of the plate face, its axis within 12 mm of the spindle axis (any
# further off, its wall would sit on a lug) and within 3 deg of the spindle direction
SEAT_AXIAL, SEAT_LAT, SEAT_DEG = 0.003, 0.012, 3.0
CENTRED_TOL = 0.006  # a core hung on the fixed lug and pressed up by the sprung one is centred to within 6 mm
JAW_CONTACT = 0.003  # a sprung half pressing on a core stops at least this far in; with no core it rests at 0
SHUT_OK = SHUT_TRAVEL - 0.010  # the shutter over the primary window, the reserve window clear


def _tilt(q) -> float:
    z = mk.quat_to_mat(q)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, -float(z[0])))))


def seated(objects, part, hub) -> bool:
    b = kit.point(objects, part, (0, 0, 0))
    _, q = kit.pose(objects, part)
    return (abs(float(b[0]) - XW) <= SEAT_AXIAL and _axis_off(objects, part, HUBS[hub]) <= SEAT_LAT
            and _tilt(q) <= SEAT_DEG)


def engaged(objects, hub) -> bool:
    """The sprung half was drawn in (button held) while the core went on, and now presses on the core."""
    q = kit.q(objects, f"jaw_{hub}_slide")
    return bool(mk.MEMORY.get(f"pushed_{hub}")) and JAW_CONTACT <= q < JAW_OPEN_AT


def centred(objects, part, hub) -> bool:
    return _axis_off(objects, part, HUBS[hub]) <= CENTRED_TOL


def cover_locked(objects) -> bool:
    return bool(mk.MEMORY.get("latched")) and kit.q_deg(objects, "cover_hinge") < 1.0


def changed_over(objects) -> bool:
    """The shutter at its second detent: over the primary (new roll) window, the reserve (stub) window open."""
    return kit.q(objects, "shutter_slide") >= SHUT_OK


def _released(st, margin=0.020) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def settled(objects) -> bool:
    return (all(_released(objects[n]) for n in ("stub", "roll", "tab_p", "tab_r", "stay_btn", "shutter"))
            and all(objects[n].resting(max_speed=0.01, max_spin=0.25) for n in ("stub", "roll", "cover")))


def failed(objects) -> bool:
    m = mk.MEMORY
    return bool(m.get("slammed") or m.get("floor") or m.get("trashed") or m.get("crushed"))


def inverted(objects) -> bool:
    return seated(objects, "roll", "r") and seated(objects, "stub", "p")


def discarded(objects) -> bool:
    """The stub is not in the dispenser (on the bench, in the bin, in a hand outside the housing)."""
    c = kit.point(objects, "stub", (0, 0, ROLL_H / 2))
    return not (RIM_X - CD < float(c[0]) < XW and -YH < float(c[1]) < YH and ZB < float(c[2]) < ZT)


def score(objects) -> float:
    """0 if failed(), inverted() or the stub is discarded. Otherwise: stub seated, centred and engaged on the reserve
    hub 0.25; only with that: new roll seated and engaged on the primary hub 0.25; with both: cover shut and latched
    0.25; with that: shutter changed over to the stub's window 0.15; then grippers off and everything still 0.10."""
    if failed(objects) or inverted(objects) or discarded(objects):
        return 0.0
    if not (seated(objects, "stub", "r") and centred(objects, "stub", "r") and engaged(objects, "r")):
        return 0.0
    s = 0.25
    if seated(objects, "roll", "p") and engaged(objects, "p"):
        s += 0.25
        if cover_locked(objects):
            s += 0.25
            if changed_over(objects):
                s += 0.15 + 0.10 * settled(objects)
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    a = kit.point(objects, "stub", (0, 0, 0))
    b = kit.point(objects, "roll", (0, 0, 0))
    return float(math.dist([float(v) for v in a], [XW, RES_Y, Z_AX]) + math.dist([float(v) for v in b],
                                                                                   [XW, PRIM_Y, Z_AX]))


# -- the scripted demo ------------------------------------------------------------------------------------------
class _Ang(float):
    """A waypoint yaw that also carries a pitch: -pi/2 turns the straight-down grip to point the fingers along +x."""

    def __new__(cls, yaw=0.0, pitch=0.0):
        o = super().__new__(cls, yaw)
        o.pitch = pitch
        return o


_stock_grip = getattr(_ydemo.grip_rotation, "_stock", _ydemo.grip_rotation)


def _grip_rotation(yaw: float = 0.0) -> np.ndarray:
    r = _stock_grip(float(yaw))
    p = getattr(yaw, "pitch", 0.0)
    return r if not p else mk.axis_angle((0, 1, 0), p) @ r


_grip_rotation._stock = _stock_grip
_ydemo.grip_rotation = _grip_rotation

TIP = 0.013
PITCH = -math.pi / 2
F_IN, F_GRIP = -0.020, -0.0469  # fingers inside a core; opened hard (fully open command) against its wall
GRIP_DZ = ROLL_H - 0.050 + TIP  # down grip: TCP above a core's bottom, pad tips 50 mm inside
SAG = 0.050  # the 0.9 kg roll held at one end: arm sag and the roll's tilt put its back end 2 to 5 cm below the
# TCP target, so the carry aims this much higher (the core then clears the fixed lug and its own weight takes it down)
RAISE = 0.026  # a core goes on with its axis this far above the spindle's, clear of both lugs
X_IN = XW - ROLL_H + 0.050 - TIP  # horizontal grip on a seated core: pad tips 50 mm inside its front end
X_OFF = X_IN - 0.070  # the core's back end 7 cm off the plate, clear of the lugs
TAB_Z = {k: bz + bh + 0.012 - TAB_TRAVEL - 0.006 + TIP for k, (bz, bh) in TAB_BASE.items()}  # onto the stop, 6 mm on
UP = 0.980
BENCH_STUB = (0.100, 0.110)  # bench_stub variant: the stub stood on the bench, left there
YC = math.pi / 2  # core grips: fingers open across x (down) or up and down (pointing at the board)
PARK = {"left": (0.150, 0.380, 0.880), "right": (0.150, -0.380, 0.880)}


class _Seq:
    def __init__(self, yc=YC):
        self.w, self.t, self.yc = [], 0.0, yc
        self.last = None

    def go(self, d, pos, f, label, yaw=0.0, pitch=0.0):
        self.w.append(Waypoint(d, tuple(float(v) for v in pos), f, label, _Ang(yaw, pitch)))
        self.t += d
        self.last = (tuple(pos), f, yaw, pitch)
        return self

    def h(self, d, pos, f, label):  # fingers pointing at the board, opening up and down
        return self.go(d, pos, f, label, self.yc, PITCH)

    def c(self, d, pos, f, label):  # straight down, fingers opening along x (so a core cannot swing about them)
        return self.go(d, pos, f, label, self.yc)

    def until(self, t, label="wait"):
        pos, f, yaw, pitch = self.last
        return self.go(max(0.1, t - self.t), pos, f, label, yaw, pitch)


def _press_tab(s, k, hold_until):
    tx, ty = TABS[k]
    yaw = YC if k == "r" else 0.0  # the hand's long side along x, clear of the core going onto the reserve spindle
    s.go(2.0, (tx, ty, TAB_Z[k] + 0.08), 0.0, f"over the {k} release button", yaw)
    z = TAB_Z[k] - (0.006 if yaw else 0.0)  # turned, the hand presses 6 mm further to reach the same travel
    s.go(1.0, (tx, ty, z), 0.0, "hold the button down", yaw)
    s.until(hold_until, "hold it while the core goes along the spindle")
    s.go(0.8, (tx, ty, TAB_Z[k] + 0.08), 0.0, "let the button up", yaw)


def _close_cover(s):
    """Press the cover stay's release button; the cover swings down onto its latch."""
    bx, by = STAY
    top = LIP_Z + STAY_H + 0.012 - TAB_TRAVEL - 0.006
    s.go(2.0, (bx, by, top + 0.050 + TIP), 0.0, "over the cover stay's button")
    s.go(1.0, (bx, by, top + TIP), 0.0, "press it")
    s.go(0.6, (bx, by, top + TIP), 0.0, "hold")
    s.go(1.0, (bx, by, top + 0.050 + TIP), 0.0, "let it up")
    s.go(1.5, PARK["right"], 0.0, "clear of the cover")
    s.go(12.0, PARK["right"], 0.0, "wait for the latch")


SH_PITCH = -0.6  # fingers tipped 34 deg toward the board, so the wrist stays clear of the cover's top front edge


def _change_over(s):
    """Closed fingers down beside the shutter's thumb tab (its -y side, near its outer end), tipped toward the board
    and with the hand's long side along x, then along the panel past half way; the detent takes it the rest."""
    kx, ky, kz = KNOB0
    tx, tz = kx - KNOB_L / 2 + 0.015, kz - 0.006  # finger tips: 15 mm in from the tab's end, 6 mm below its middle
    x, z = tx - TIP * math.sin(-SH_PITCH), tz + TIP * math.cos(SH_PITCH)
    s.go(2.0, (x - 0.02, ky - 0.060, z + 0.06), 0.0, "over the left of the shutter tab", YC, SH_PITCH)
    s.go(1.0, (x, ky - 0.060, z), 0.0, "down beside it", YC, SH_PITCH)
    s.go(3.0, (x, ky + 0.150, z), 0.0, "slide the shutter across", YC, SH_PITCH)
    s.go(1.0, (x - 0.02, ky + 0.150, z + 0.06), 0.0, "up", YC, SH_PITCH)
    s.go(2.0, PARK["right"], 0.0, "clear")
    s.go(2.0, PARK["right"], 0.0, "wait")


def _plans(variant=""):
    L, R = _Seq(-YC), _Seq(YC)  # each wrist turned the way it reaches the bench middle
    if variant == "close_only":
        L.go(2.0, PARK["left"], 0.0, "park")
        R.go(1.0, PARK["right"], 0.0, "start")
        _close_cover(R)
        return {"left": L.w, "right": R.w}
    # L: fingers into the stub's core from the front, opened; R holds the primary button; L draws the stub off
    L.h(2.5, (X_OFF - 0.03, PRIM_Y, Z_AX + 0.04), F_IN, "in front of the stub")
    L.h(2.0, (X_OFF, PRIM_Y, Z_AX), F_IN, "level with its core")
    L.h(2.0, (X_IN, PRIM_Y, Z_AX), F_IN, "fingers into the core")
    L.h(1.0, (X_IN, PRIM_Y, Z_AX), F_GRIP, "open against the core")
    L.h(0.5, (X_IN, PRIM_Y, Z_AX), F_GRIP, "settle")
    t_tab = L.t
    R.go(1.0, PARK["right"], 0.0, "start")
    R.until(t_tab - 3.0)
    L.until(t_tab + 1.0, "wait for the hub to be released")
    L.h(2.0, (X_OFF, PRIM_Y, Z_AX + 0.003), F_GRIP, "draw the stub off the spindle")
    t_off = L.t
    _press_tab(R, "p", t_off + 0.3)
    if variant == "trash_stub":
        L.c(3.0, (BIN_XY[0], BIN_XY[1], TOP + GRIP_DZ + 0.08), F_GRIP, "over the bin, stub upright")
        L.c(0.8, (BIN_XY[0], BIN_XY[1], TOP + GRIP_DZ + 0.08), F_IN, "drop it in")
        L.go(1.0, PARK["left"], F_IN, "park")
    elif variant == "bench_stub":
        bx, by = BENCH_STUB
        L.c(3.0, (bx, by, TOP + GRIP_DZ + 0.04), F_GRIP, "over the bench, stub upright")
        L.c(1.0, (bx, by, TOP + GRIP_DZ + 0.001), F_GRIP, "down")
        L.c(0.8, (bx, by, TOP + GRIP_DZ + 0.001), F_IN, "let go")
        L.c(1.0, (bx, by, 0.90), F_IN, "up")
    else:
        # L: straight across to the reserve spindle while R holds the reserve button (offset_seat: without it)
        L.h(3.0, (X_OFF - 0.02, RES_Y, Z_AX + RAISE), F_GRIP, "across to the reserve spindle")
        L.h(1.0, (X_OFF, RES_Y, Z_AX + RAISE), F_GRIP, "line it up")
        t_res = L.t
        L.h(3.0, (X_IN + 0.003, RES_Y, Z_AX + RAISE), F_GRIP, "slide it along the spindle")
        L.h(0.5, (X_IN + 0.003, RES_Y, Z_AX + RAISE), F_GRIP, "settle")
        L.h(0.8, (X_IN + 0.003, RES_Y, Z_AX + RAISE), F_IN, "let go")
        t_r_off = L.t
        L.h(1.0, (X_IN + 0.003, RES_Y, Z_AX + 0.004), F_IN, "down with it onto the hub")
        L.until(t_r_off + 1.6)
        L.h(1.5, (X_OFF - 0.03, RES_Y, Z_AX), F_IN, "fingers out")
        R.until(t_res - 4.0)
        if variant != "offset_seat":
            _press_tab(R, "r", t_r_off + 1.2)
        else:
            R.until(t_r_off + 3.0, "wait, the reserve button not touched")
    # L: the new roll onto the primary while R holds the primary button (loose_roll: without the button)
    nx = NEW_FRONT + 0.050 - TIP  # pad tips 50 mm inside its front end
    ny, nz = NEW0[1], NEW_AX
    L.h(2.0, (0.240, 0.220, nz + 0.04), F_IN, "toward the new roll")
    L.h(2.0, (nx - 0.07, ny, nz + 0.02), F_IN, "in front of the new roll")
    L.h(1.5, (nx - 0.07, ny, nz), F_IN, "level with its core")
    L.h(2.0, (nx, ny, nz), F_IN, "fingers into its core")
    L.h(1.0, (nx, ny, nz), F_GRIP, "open against the core")
    L.h(0.5, (nx, ny, nz), F_GRIP, "settle")
    L.h(2.0, (nx, ny, nz + 0.040), F_GRIP, "lift it off the carton")
    L.h(2.0, (X_OFF - 0.01, ny, nz + 0.045), F_GRIP, "draw it back a little")
    # across at arm's length: nearer the base the arm folds up and its elbow runs into the roll
    # level until clear of the carton (lowering on the way drags the roll over its edge), then down
    L.h(5.0, (X_OFF - 0.01, PRIM_Y + 0.03, nz + 0.045), F_GRIP, "across to the primary spindle")
    L.h(1.5, (X_OFF - 0.01, PRIM_Y + 0.03, Z_AX + RAISE + SAG), F_GRIP, "down level with it")
    L.h(2.0, (X_OFF, PRIM_Y, Z_AX + RAISE + SAG), F_GRIP, "line it up with the primary spindle")
    t_line = L.t
    L.until(t_line + 3.5, "wait for the primary hub to be released")
    t_prim = L.t
    L.h(3.0, (X_IN + 0.003, PRIM_Y, Z_AX + RAISE + SAG), F_GRIP, "slide it along the spindle")
    L.h(0.5, (X_IN + 0.003, PRIM_Y, Z_AX + RAISE + SAG), F_GRIP, "settle")
    L.h(0.8, (X_IN + 0.003, PRIM_Y, Z_AX + RAISE + SAG), F_IN, "let go")
    t_l_off = L.t
    L.h(1.0, (X_IN + 0.003, PRIM_Y, Z_AX + 0.004), F_IN, "down with it onto the hub")
    L.until(t_l_off + 1.6)
    L.h(1.5, (X_OFF - 0.03, PRIM_Y, Z_AX), F_IN, "fingers out")
    # the heavy roll can hang up a little short of the plate: closed fingers push its front face home
    push_z = Z_AX + (R_I + ROLL_RO) / 2
    L.h(1.0, (X_OFF - 0.03, PRIM_Y, push_z), 0.0, "up to the roll's front face")
    L.h(2.0, (XW - ROLL_H - TIP + 0.006, PRIM_Y, push_z), 0.0, "push it home against the plate")
    L.h(0.5, (XW - ROLL_H - TIP + 0.006, PRIM_Y, push_z), 0.0, "hold")
    t_pushed = L.t
    L.h(1.0, (X_OFF - 0.03, PRIM_Y, push_z), 0.0, "back off")
    L.go(2.0, PARK["left"], 0.0, "park")
    if variant != "loose_roll":
        R.until(t_line)  # the right hand keeps clear until the roll is lined up in front of the spindle
        _press_tab(R, "p", t_pushed + 0.3)
    else:
        R.until(t_l_off)
    R.until(L.t)
    _close_cover(R)
    if variant != "shutter_left":
        _change_over(R)
    return {"left": L.w, "right": R.w}


def demo():
    # DEMO_VARIANT: close_only | trash_stub | bench_stub | offset_seat | loose_roll | shutter_left (none score 1.0)
    return _plans(os.environ.get("DEMO_VARIANT", ""))


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    I = np.array([0, 0, 0, 1.0])
    QA = np.asarray(Q_AX)

    def at(hub, dx=0.0, dz=0.0, quat=QA):
        return (np.array([XW + dx, HUBS[hub], Z_AX + dz]), quat)

    def st(q=None, **moved):
        return kit.fake_states(q=q, moved=moved)

    J = {"jaw_r_slide": 0.006, "jaw_p_slide": 0.006}  # sprung halves pressing on the seated cores
    sh = {"shutter_slide": SHUT_TRAVEL}
    closed = {"cover_hinge": 0.0, **J, **sh}
    done = st(closed, stub=at("r"), roll=at("p"))
    P = {"pushed_r": True, "pushed_p": True}
    L = {"latched": True, **P}
    tilt = mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.radians(5)) @ mk.quat_to_mat(QA))
    cases = {
        "start": (st(), [], {}, 0.0),
        "start, both buttons pressed earlier": (st(), [], P, 0.0),
        "closed untouched": (st({"cover_hinge": 0.0}), [], L, 0.0),
        "closed untouched, shutter changed over": (st({"cover_hinge": 0.0, **sh}), [], L, 0.0),
        "stub on reserve": (st(J, stub=at("r")), [], P, 0.25),
        "stub on reserve, button never held": (st(J, stub=at("r")), [], {"pushed_p": True}, 0.0),
        "stub hung on the sprung lug, 9 mm high": (st({"jaw_r_slide": 0.0}, stub=at("r", dz=0.009)), [], P, 0.0),
        "stub 1 cm short of the plate": (st(J, stub=at("r", dx=-0.010)), [], P, 0.0),
        "both seated, cover open": (st(J, stub=at("r"), roll=at("p")), [], P, 0.5),
        "roll seated, its button never held": (st(closed, stub=at("r"), roll=at("p")), [],
                                               {**L, "pushed_p": False}, 0.25),
        "both seated, latched, shutter as found": (st({**closed, "shutter_slide": 0.0}, stub=at("r"),
                                                      roll=at("p")), [], L, 0.75),
        "both seated, latched, shutter half way": (st({**closed, "shutter_slide": SHUT_TRAVEL / 2}, stub=at("r"),
                                                      roll=at("p")), [], L, 0.75),
        "both seated, latched, changed over, held": (done, [done["shutter"].pos], L, 0.9),
        "everything done": (done, [], L, 1.0),
        "roll 5 mm off the plate": (st(closed, stub=at("r"), roll=at("p", dx=-0.005)), [], L, 0.25),
        "roll tilted 5 deg": (st(closed, stub=at("r"), roll=(at("p")[0], tilt)), [], L, 0.25),
        "roll hub still drawn in": (st({**closed, "jaw_p_slide": 0.040}, stub=at("r"), roll=at("p")), [], L, 0.25),
        "inverted": (st(closed, stub=at("p"), roll=at("r")), [], L, 0.0),
        "slammed cover": (done, [], {**L, "slammed": True}, 0.0),
        "stub in the bin": (done, [], {**L, "trashed": True}, 0.0),
        "roll seated, stub on the bench": (st(closed, stub=(np.array([0.08, 0.25, TOP]), I), roll=at("p")), [], L,
                                           0.0),
        "roll seated, stub left on the primary": (st(closed, roll=at("p")), [], L, 0.0),
        "cover forced on a loose roll": (done, [], {**L, "crushed": True}, 0.0),
        "roll on the floor": (done, [], {**L, "floor": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        assert seated(st(), "stub", "p") and centred(st(), "stub", "p")
        _TEST_TCPS = []

        def close_at(rad_s, roll=None):  # the cover swung shut at a steady rad_s
            mk.new_episode()
            mk.MEMORY.update({**P, "n": 20})
            q = 60.0
            rl = roll if roll is not None else at("p")
            while q > 0.0:
                _cover_rule(st({**J, "cover_hinge": math.radians(q)}, stub=at("r"), roll=rl))
                q -= math.degrees(rad_s * DT)
            _cover_rule(st({**closed}, stub=at("r"), roll=rl))
            return bool(mk.MEMORY.get("slammed")), bool(mk.MEMORY.get("latched")), bool(mk.MEMORY.get("crushed"))

        for w in (0.3, 1.0, 3.0, 5.0):  # weight and damper alone, slow, brisk and fast hand closes: none fails
            assert close_at(w) == (False, True, False), (w, close_at(w))
        assert close_at(8.0)[0], "slam"
        # a roll stuck 3 cm out of its seat (not on its hub) under a closing cover is crushed
        assert close_at(1.0, roll=at("p", dx=-0.030))[2], "crush"
        # the hubs turn: a seated roll turned 90 deg about the spindle with its hub still scores the same
        mk.new_episode()
        mk.MEMORY.update(L)
        turn = mk.mat_to_quat(mk.axis_angle((-1, 0, 0), math.radians(90)) @ mk.quat_to_mat(QA))
        spun = st({**closed, "hub_p_spin": math.radians(90)}, stub=at("r"), roll=(at("p")[0], turn))
        assert abs(score(spun) - 1.0) < 1e-9, score(spun)
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for v in ("", "close_only", "trash_stub", "bench_stub", "offset_seat", "loose_roll", "shutter_left"):
        os.environ["DEMO_VARIANT"] = v
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (v, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (v, arm)
            print(f"{v or 'demo'} {arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / "
                  f"{plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT")
