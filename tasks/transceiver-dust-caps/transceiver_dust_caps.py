"""On the station table, in front of the left arm, a switch's port row lies flat: a steel faceplate with four
transceiver cages in a row along y, opening upward. Cages 1, 3 and 4 hold working optics; the optic in cage 2 has
failed (a red link tile lies in front of it, green tiles in front of the others). Nearer the arms stand three open
trays: a white spares tray holding the new optic with its dust cap on its bore end, a blue cap tray holding a cage
dust plug, and a black return tray. Fibre hygiene sets the order: an empty cage is plugged as soon as its optic comes
out and unplugged only when the new optic goes in, and the new optic keeps its own cap until it is seated. Both rules
are checked every step; breaking either caps the score at 0.25. Scored from part poses.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/transceiver-dust-caps/transceiver_dust_caps.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
OPTIC = (0.022, 0.024, 0.060)  # a transceiver standing on its electrical end, bore end up: 22 x 24 mm, 60 mm tall
OPTIC_KG = 0.040
CAP = (0.030, 0.016, 0.018)  # the optic's dust cap, sitting over its bore end: 30 mm along x, 16 mm across, 18 tall
CAP_KG = 0.008
PLUG = (0.022, 0.024, 0.040)  # the cage dust plug with its pull tab, drawn as one block that fills the cage mouth
PLUG_KG = 0.010

PLATE = (0.060, 0.260, 0.003)
PLATE_TOP = TOP + PLATE[2]
CAGE_X = 0.340
CAGES = 4
CAGE_PITCH = 0.060
CAGE_Y = {k: 0.250 + (CAGES + 1 - 2 * k) * CAGE_PITCH / 2 for k in range(1, CAGES + 1)}  # cage 1 at the left
FAILED = 2
CAGE_CLEAR = 0.003  # air round an optic in its cage
CAGE_T, CAGE_H = 0.004, 0.012
TILE = (0.020, 0.030, 0.003)
TILE_X = CAGE_X - 0.050

TRAY_T, TRAY_H = 0.006, 0.010
SPARE_TRAY = ((0.170, 0.330), (0.070, 0.070))  # (centre, inner size)
CAP_TRAY = ((0.170, 0.240), (0.140, 0.070))
RETURN_TRAY = ((0.170, 0.150), (0.080, 0.070))
PLUG_SPOT = (0.135, 0.240)  # where the plug starts and is parked again
CAP_SPOT = (0.195, 0.240)  # where the demo parks the optic's cap

STEEL = (0.70, 0.71, 0.73)
OPTIC_C = (0.80, 0.80, 0.82)
CAP_C = (0.10, 0.10, 0.11)
PLUG_C = (0.12, 0.30, 0.70)
GREEN = (0.15, 0.70, 0.25)
RED = (0.85, 0.12, 0.10)
WHITE = (0.92, 0.92, 0.92)
BLUE = (0.30, 0.50, 0.85)
BLACK = (0.06, 0.06, 0.06)


def _frame(name, centre, inner, color, height=TRAY_H, base=TOP, t=TRAY_T):
    (cx, cy), (ix, iy) = centre, inner
    z = base + height / 2
    return [
        Box(f"{name}_wall_ypos", pos=(cx, cy + iy / 2 + t / 2, z), size=(ix + 2 * t, t, height), color=color),
        Box(f"{name}_wall_yneg", pos=(cx, cy - iy / 2 - t / 2, z), size=(ix + 2 * t, t, height), color=color),
        Box(f"{name}_wall_near", pos=(cx - ix / 2 - t / 2, cy, z), size=(t, iy, height), color=color),
        Box(f"{name}_wall_far", pos=(cx + ix / 2 + t / 2, cy, z), size=(t, iy, height), color=color),
    ]


CAGE_INNER = (OPTIC[0] + 2 * CAGE_CLEAR, OPTIC[1] + 2 * CAGE_CLEAR)
WORKING = [Box(f"optic_{k}", pos=(CAGE_X, CAGE_Y[k], PLATE_TOP + OPTIC[2] / 2), size=OPTIC, mass=OPTIC_KG,
               color=OPTIC_C) for k in range(1, CAGES + 1) if k != FAILED]
FAILED_OPTIC = Box("failed_optic", pos=(CAGE_X, CAGE_Y[FAILED], PLATE_TOP + OPTIC[2] / 2), size=OPTIC, mass=OPTIC_KG,
                   color=OPTIC_C)
NEW_OPTIC = Box("new_optic", pos=(*SPARE_TRAY[0], TOP + OPTIC[2] / 2), size=OPTIC, mass=OPTIC_KG, color=OPTIC_C)
NEW_CAP = Box("new_optic_cap", pos=(*SPARE_TRAY[0], TOP + OPTIC[2] + CAP[2] / 2), size=CAP, mass=CAP_KG, color=CAP_C)
CAGE_PLUG = Box("cage_plug", pos=(*PLUG_SPOT, TOP + PLUG[2] / 2), size=PLUG, mass=PLUG_KG, color=PLUG_C)
FIXTURES = [
    Box("faceplate", pos=(CAGE_X, 0.250, TOP + PLATE[2] / 2), size=PLATE, color=STEEL),
    *(p for k in range(1, CAGES + 1) for p in _frame(f"cage_{k}", (CAGE_X, CAGE_Y[k]), CAGE_INNER, STEEL, CAGE_H,
                                                      PLATE_TOP, CAGE_T)),
    *(Box(f"link_tile_{k}", pos=(TILE_X, CAGE_Y[k], TOP + TILE[2] / 2), size=TILE, color=RED if k == FAILED else GREEN)
      for k in range(1, CAGES + 1)),
    *_frame("spares_tray", *SPARE_TRAY, WHITE),
    *_frame("cap_tray", *CAP_TRAY, BLUE),
    *_frame("return_tray", *RETURN_TRAY, BLACK),
]
FREE = [*WORKING, FAILED_OPTIC, NEW_OPTIC, NEW_CAP, CAGE_PLUG]
OBJECTS = [*FREE, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_XY, SEAT_Z, SEAT_DEG = 0.0035, 0.002, 6.0  # seated in cage 2 (the cage allows 3 mm)
KEEP_MOVE, KEEP_DEG = 0.004, 5.0  # a working optic counts as untouched within 4 mm and 5 degrees
LIFTED = 0.010  # the new optic has left its tray once its centre is 1 cm from its start
ON_XY, ON_Z = 0.010, 0.005  # the cap counts as on the new optic: centred within 1 cm, resting on its top within 5 mm


def _yaw_tilt(quat_xyzw):
    """(yaw about the vertical, tilt of the up axis from the vertical), degrees."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    yaw = math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return yaw, tilt


def _square(yaw_deg) -> float:
    return abs((yaw_deg + 90.0) % 180.0 - 90.0)


def _in_cage(st, still=True) -> bool:
    """A part standing in cage 2: centred within 3.5 mm, bottom within 2 mm of the faceplate, upright and square."""
    yaw, tilt = _yaw_tilt(st.quat_xyzw)
    return bool(
        abs(float(st.pos[0]) - CAGE_X) <= SEAT_XY
        and abs(float(st.pos[1]) - CAGE_Y[FAILED]) <= SEAT_XY
        and abs(float(st.low[2]) - PLATE_TOP) <= SEAT_Z
        and _square(yaw) <= SEAT_DEG
        and tilt <= SEAT_DEG
        and (st.resting() or not still)
    )


def _in_tray(st, tray) -> bool:
    (cx, cy), (ix, iy) = tray
    return bool(
        abs(float(st.pos[0]) - cx) < ix / 2
        and abs(float(st.pos[1]) - cy) < iy / 2
        and TOP - 0.005 < float(st.low[2]) < TOP + 0.015
        and st.resting()
    )


def _cap_on(objects) -> bool:
    c, o = objects[NEW_CAP.name], objects[NEW_OPTIC.name]
    return bool(math.hypot(float(c.pos[0]) - float(o.pos[0]), float(c.pos[1]) - float(o.pos[1])) <= ON_XY
                and abs(float(c.low[2]) - float(o.high[2])) <= ON_Z)


def _untouched(objects) -> bool:
    for o in WORKING:
        st = objects[o.name]
        yaw, tilt = _yaw_tilt(st.quat_xyzw)
        if math.dist([float(v) for v in st.pos], o.pos) > KEEP_MOVE or _square(yaw) > KEEP_DEG or tilt > KEEP_DEG:
            return False
    return True


# The trial's history. The task file only sees object states, so it resets whenever the scene is back at its start.
_HISTORY = {"plugged": False, "violated": False}


def _at_start(objects) -> bool:
    return all(math.dist([float(v) for v in objects[o.name].pos], o.pos) < 0.003 for o in FREE)


def _update(objects) -> None:
    if _at_start(objects):
        _HISTORY.update(plugged=False, violated=False)
        return
    new = objects[NEW_OPTIC.name]
    cage_empty_of_failed = not _in_cage(objects[FAILED_OPTIC.name], still=False)
    if cage_empty_of_failed and _in_cage(objects[CAGE_PLUG.name]):
        _HISTORY["plugged"] = True
    left_tray = math.dist([float(v) for v in new.pos], NEW_OPTIC.pos) > LIFTED
    if left_tray and not _HISTORY["plugged"]:
        _HISTORY["violated"] = True  # the new optic came out of its tray before the open cage had been plugged
    if not _cap_on(objects) and not _in_cage(new, still=False):
        _HISTORY["violated"] = True  # the new optic uncapped before it was seated


def score(objects) -> float:
    """0.25 for the failed optic at rest in the return tray; 0.25 once cage 2 has been plugged after the failed optic
    came out; 0.25 for the new optic seated in cage 2; 0.25 for its cap and the cage plug both at rest in the cap tray
    while it is seated. Either hygiene rule ever broken (the new optic out of its tray before cage 2 was plugged, or
    its cap off before it is seated), or a working optic moved more than 4 mm or 5 degrees, caps the score at 0.25."""
    _update(objects)
    seated = _in_cage(objects[NEW_OPTIC.name])
    s = 0.25 * _in_tray(objects[FAILED_OPTIC.name], RETURN_TRAY) + 0.25 * _HISTORY["plugged"] + 0.25 * seated
    s += 0.25 * (seated and _in_tray(objects[NEW_CAP.name], CAP_TRAY) and _in_tray(objects[CAGE_PLUG.name], CAP_TRAY))
    if _HISTORY["violated"] or not _untouched(objects):
        s = min(s, 0.25)
    return float(s)


def success(objects) -> bool:
    """Failed optic returned, cage plugged in time, new optic seated and uncapped only once seated, plug and cap in
    the cap tray, working optics untouched; holds for 1 s."""
    return score(objects) >= 1.0 - 1e-9


def violated() -> bool:
    """Whether this trial has broken a hygiene rule (for the log)."""
    return _HISTORY["violated"]


def distance(objects) -> float:
    """Metres to go: the failed optic to the return tray, the new optic to cage 2, its cap to the cap tray."""
    f, n, c = (objects[o.name] for o in (FAILED_OPTIC, NEW_OPTIC, NEW_CAP))
    d = math.hypot(float(f.pos[0]) - RETURN_TRAY[0][0], float(f.pos[1]) - RETURN_TRAY[0][1])
    d += math.dist((float(n.pos[0]), float(n.pos[1]), float(n.low[2])), (CAGE_X, CAGE_Y[FAILED], PLATE_TOP))
    d += math.hypot(float(c.pos[0]) - CAP_SPOT[0], float(c.pos[1]) - CAP_SPOT[1])
    return float(d)


# -- the scripted demo ------------------------------------------------------------------------------------------

ACROSS, ALONG = 0.0, math.pi / 2  # close along y (optics and plug, 24 mm) or along x (the cap, 30 mm)
TIP = 0.013
TRAVEL = TOP + 0.170
SQUEEZE = 0.008
X_TRIM = 0.0015  # a carried light part lands about 1.5 mm short in x (batch notes): trim the cage set-downs


def _move(pick, place, label, width, depth, yaw, trim=0.0, open_to=None):
    """Top-down pick and place: open to the part's width plus 16 mm (the pads clear it by 8 mm, the neighbours by
    18 mm or more), straight down, settle, close, straight up, carry high, straight down, release to the same opening.
    ``depth`` is how far below the part's top the pad tips go; ``pick``/``place`` are (x, y, bottom, height)."""
    (px, py, pb, ph), (qx, qy, qb, qh) = pick, place
    qx += trim
    gz = pb + ph - depth + TIP
    qz = qb + ph - depth + TIP + 0.0015  # set down 1.5 mm above the surface
    wide = open_to if open_to is not None else -(width + 0.016) / 2
    grip = -(width - SQUEEZE) / 2
    return [
        Waypoint(1.8, (px, py, TRAVEL), wide, f"{label}: over", yaw),
        Waypoint(1.8, (px, py, gz + 0.020), wide, f"{label}: descend", yaw),
        Waypoint(1.0, (px, py, gz), wide, f"{label}: down slowly", yaw),
        Waypoint(0.5, (px, py, gz), wide, f"{label}: settle", yaw),
        Waypoint(1.2, (px, py, gz), grip, f"{label}: close", yaw),
        Waypoint(1.5, (px, py, gz + 0.040), grip, f"{label}: lift straight up", yaw),
        Waypoint(1.2, (px, py, TRAVEL), grip, f"{label}: lift", yaw),
        Waypoint(2.2, (qx, qy, TRAVEL), grip, f"{label}: carry", yaw),
        Waypoint(0.6, (qx, qy, TRAVEL), grip, f"{label}: settle over", yaw),
        Waypoint(1.8, (qx, qy, qz + 0.020), grip, f"{label}: lower", yaw),
        Waypoint(1.2, (qx, qy, qz), grip, f"{label}: set in slowly", yaw),
        Waypoint(0.5, (qx, qy, qz), grip, f"{label}: settle", yaw),
        Waypoint(1.2, (qx, qy, qz), wide, f"{label}: release", yaw),
        Waypoint(1.4, (qx, qy, TRAVEL), wide, f"{label}: retreat", yaw),
    ]


def demo():
    """Left arm only: the failed optic out of cage 2 into the return tray; at once the cage plug into cage 2; then
    the plug back out to the cap tray and the capped new optic straight into cage 2 (its cap riding on top, the pads
    below the cap); last the cap lifted off the seated optic into the cap tray. 89.5 s."""
    cage = (CAGE_X, CAGE_Y[FAILED], PLATE_TOP)
    oh, ph = OPTIC[2], PLUG[2]
    plan = _move((*cage, oh), (*RETURN_TRAY[0], TOP, oh), "failed optic out", OPTIC[1], 0.025, ACROSS)
    plan += _move((*PLUG_SPOT, TOP, ph), (*cage, ph), "plug the open cage", PLUG[1], 0.016, ACROSS, trim=X_TRIM)
    plan += _move((*cage, ph), (*PLUG_SPOT, TOP, ph), "unplug the cage", PLUG[1], 0.016, ACROSS)
    plan += _move((*SPARE_TRAY[0], TOP, oh), (*cage, oh), "capped new optic in", OPTIC[1], 0.032, ACROSS, trim=X_TRIM)
    plan += _move((*cage[:2], PLATE_TOP + oh, CAP[2]), (*CAP_SPOT, TOP, CAP[2]), "uncap the seated optic", CAP[0],
                  0.009, ALONG)
    return {"left": plan}


def self_test(ObjectState=None):
    """The scoring rules on hand-built sequences of states, so the history is exercised too."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def box(size, x, y, bottom, yaw_deg=0.0, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
        yaw, tilt = math.radians(yaw_deg), math.radians(tilt_deg)
        x1, y1, z1, w1 = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)
        x2, y2, z2, w2 = math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)
        q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                      w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
        half = np.array(size) / 2
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def at(o, **kw):
        return box(o.size, o.pos[0], o.pos[1], o.pos[2] - o.size[2] / 2, **kw)

    start = {o.name: at(o) for o in FREE}
    cx, cy = CAGE_X, CAGE_Y[FAILED]
    f_lift = {"failed_optic": box(OPTIC, cx, cy, PLATE_TOP + 0.05, vel=(0, 0, 0.05))}
    f_out = {"failed_optic": box(OPTIC, *RETURN_TRAY[0], TOP)}
    plug_in = {"cage_plug": box(PLUG, cx + 0.001, cy, PLATE_TOP)}
    plug_back = {"cage_plug": box(PLUG, *PLUG_SPOT, TOP)}
    sx, sy = SPARE_TRAY[0]
    new_up = {"new_optic": box(OPTIC, sx, sy, TOP + 0.06), "new_optic_cap": box(CAP, sx, sy, TOP + 0.12)}
    new_in = {"new_optic": box(OPTIC, cx - 0.002, cy, PLATE_TOP), "new_optic_cap": box(CAP, cx - 0.002, cy, PLATE_TOP + OPTIC[2])}
    cap_off = {"new_optic_cap": box(CAP, *CAP_SPOT, TOP)}
    a = start | f_out
    b = a | plug_in
    c = a | plug_back
    d = c | new_in
    e = d | cap_off

    def run(*frames):
        for f in frames[:-1]:
            score(f)
        return score(frames[-1]), success(frames[-1])

    cases = {
        "start": ((start,), 0.0, False),
        "failed optic lifted": ((start, start | f_lift), 0.0, False),
        "failed optic returned": ((start, a), 0.25, False),
        "cage plugged": ((start, a, b), 0.5, False),
        "plug parked again, cage open": ((start, a, b, c), 0.5, False),
        "new optic seated, still capped": ((start, a, b, c, c | new_up, d), 0.75, False),
        "done in order": ((start, a, b, c, c | new_up, d, e), 1.0, True),
        "never plugged: straight swap": ((start, a, a | new_up, d, e), 0.25, False),
        "new optic taken out before the failed one is pulled": ((start, start | new_up, b, c, d, e), 0.25, False),
        "cap taken off in the spares tray first": ((start, a, b, c, c | cap_off, c | cap_off | {"new_optic": new_in["new_optic"]}), 0.25, False),
        "cap taken off while held over the cage": ((start, a, b, c, c | new_up, c | {"new_optic": box(OPTIC, cx, cy, PLATE_TOP + 0.04)} | cap_off, e), 0.25, False),
        "plug pushed in while the failed optic was still in": ((start, start | {"cage_plug": box(PLUG, cx, cy, PLATE_TOP + OPTIC[2])}, a, a | new_up, d, e), 0.25, False),
        "done but working optic 3 knocked 5 mm": ((start, a, b, c, d, e | {"optic_3": box(OPTIC, cx - 0.005, CAGE_Y[3], PLATE_TOP)}), 0.25, False),
        "done but the plug left in the cap tray lying down": ((start, a, b, c, d, e | {"cage_plug": box(PLUG, *PLUG_SPOT, TOP, 0, 90)}), 1.0, True),
        "done but the cap dropped on the faceplate": ((start, a, b, c, d, d | {"new_optic_cap": box(CAP, 0.30, 0.20, PLATE_TOP)}), 0.75, False),
        "new optic seated 5 mm off": ((start, a, b, c, c | {"new_optic": box(OPTIC, cx - 0.005, cy, PLATE_TOP), "new_optic_cap": box(CAP, cx - 0.005, cy, PLATE_TOP + OPTIC[2])}), 0.5, False),
        "a new trial after a violation starts clean": ((start, a | new_up, start, a, b, c, d, e), 1.0, True),
    }
    for name, (frames, want, want_success) in cases.items():
        got, got_success = run(*frames)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    score(start)
    assert not violated()
    assert distance(e) < 0.005 and distance(start) > 0.2
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
