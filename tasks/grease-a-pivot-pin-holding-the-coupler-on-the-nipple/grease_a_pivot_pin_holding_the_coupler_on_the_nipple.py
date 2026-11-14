"""On the station table stands the pivot of a loader arm: a yellow steel lug on a base plate with a steel pin through
it, and on the lug's top a straight grease nipple (DIN 71412 form A: M10x1 taper thread, 11 mm hex, 6.5 mm ball
head). To the left stands a grease pail with a lever pump on its lid; its black hose runs over the table to a jawless
push-type coupler standing in a steel holder cup.

Task: grease the pivot pin through its nipple from the pail pump without leaking grease at the coupler or the nipple,
then leave the coupler back in its holder cup and both arms back where they started.

The coupler's mouth is a 7.2 mm bore with a 45-degree lead-in that goes right over the 6.5 mm ball head until a
nitrile seal ring 5 mm up the bore sits on the ball's top. Nothing latches it: while the lever is pressed the grease
pressure pushes it back off the ball, so it seals only while it is pressed on squarely. A lever stroke made through a
sealed seat is a shot of grease into the pin's bushing; a stroke made while the coupler is not sealed on the nipple is
a leak (latched for the whole trial).

Run it:  inspect-robots run --embodiment yam_arena -E task=grease_a_pivot_pin_holding_the_coupler_on_the_nipple.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, ball, hinge,  # noqa: E402
                     on_reset, on_step, plate, rotation, segment)
from yam_arena.constants import FINGER_CLOSED, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the loader pivot: a lug with a pin through it and a grease nipple on top ------------------------------------------
MX, MY = 0.30, -0.08
LUG = (0.05, 0.04, 0.05)
BASE_T = 0.010
PIN_Z = 0.030  # the pin's axis above the table
BOSS_TOP = TOP + BASE_T + LUG[2] + 0.006  # the boss's top face, where the nipple is screwed in
# the grease nipple, DIN 71412 form A (straight): M10x1 taper thread (in the boss), 11 mm hex, 6.5 mm ball head on a
# 4 mm neck, its top 10.5 mm above the boss (16 mm long, 5.5 mm of it thread)
HEX_AF, HEX_H = 0.011, 0.0035
NECK_R = 0.002
BALL_R = 0.00325
BALL_Z = BOSS_TOP + 0.00725  # the ball head's centre
NIPPLE_TOP = BALL_Z + BALL_R

# -- the grease pail with its lever pump ---------------------------------------------------------------------------
PX, PY = 0.37, 0.20
PAIL_R, PAIL_H = 0.065, 0.085
PIVOT = np.array([PX - 0.015, PY, TOP + 0.120])  # the lever's pivot on the pump head
LEVER_L = 0.125  # from the pivot to the middle of the lever's grip pad
LEVER_UP = 15.0  # degrees the lever stands up from level at rest
PAD = (0.030, 0.028, 0.016)
STROKE_DOWN, STROKE_UP = 18.0, 10.0  # degrees pressed down: a stroke is down past 18, then back up past 10
PRESSURE_DEG = 6.0  # pressed past this, the pump is pushing grease: the line is under pressure
STROKES = 4  # sealed strokes that fill the pin's passage and bushing and purge fresh grease at its ends
OUTLET = (PX, PY - PAIL_R - 0.006, TOP + 0.070)

# -- the push-type coupler (origin: the middle of its mouth face, z up its axis) and its holder cup -----------------
HX, HY = 0.24, 0.0
CUP_R, CUP_H, CUP_FLOOR = 0.023, 0.016, 0.004  # the holder's outer radius, rim height and floor
SEAT_R, FUNNEL_Z, FUNNEL_R = 0.011, 0.007, 0.020  # the 22 mm seat the nose stands in, and its 45-degree funnel
IN_HOLDER_OFF, IN_HOLDER_UP, IN_HOLDER_TILT = 0.006, 0.006, 35.0  # the mouth down in the seat, the coupler upright
NOSE_R, NOSE_H = 0.009, 0.012  # the hardened steel nose, 18 mm across
BORE_R = 0.0036  # the mouth: a 7.2 mm bore over the 6.5 mm ball head
CHAMFER_R, CHAMFER_D = 0.0052, 0.0016  # its 45-degree lead-in, 10.4 mm across at the face
SEAL_Z, SEAL_T, SEAL_HOLE_R = 0.005, 0.002, 0.0023  # the nitrile seal ring 5 mm up the bore, its hole 4.6 mm across
BODY_AF, BODY_Z1 = 0.013, 0.038  # the 13 mm hex body above the nose, where it is held
SWIVEL_R, SWIVEL_Z1 = 0.006, 0.048  # the brass swivel on top
STUB_Z, STUB_L, STUB_R = 0.043, 0.030, 0.005  # its side outlet, where the hose comes in
COUPLER_KG = 0.07
SEAT_DEPTH = math.sqrt(BALL_R ** 2 - SEAL_HOLE_R ** 2)  # the ball's centre this far under the seal when seated
BALL_IN_MOUTH = SEAL_Z - SEAT_DEPTH  # the ball's centre this far up from the mouth face when seated (2.7 mm)
MOUTH_SEATED = BALL_Z - BALL_IN_MOUTH  # the mouth face's height when seated (1.05 mm over the hex)
SEAT_OFF, SEAT_LIFT, SEAT_SINK, SEAT_TILT = 0.0015, 0.0005, 0.001, 8.0  # the sealed-seat tolerances
PUSH_N = 15.0  # while the line is under pressure, the grease pushes a coupler on the ball back off with this force
PUSH_OFF, PUSH_LIFT, PUSH_TILT = 0.002, 0.001, 20.0  # ... while its seal is this close on the ball

# -- the hose: five segments on ball joints from the pump's outlet to the coupler's side outlet ----------------------
HOSE = [OUTLET, (PX, 0.120, TOP + 0.007), (0.40, 0.060, TOP + 0.007), (0.38, 0.0, TOP + 0.007),
        (0.31, 0.0, TOP + 0.007), (HX + STUB_L, HY, TOP + CUP_FLOOR + STUB_Z)]

YELLOW, STEEL, DARK, RED, BRASS, BLACK, GREASE, ZINC = (0.93, 0.68, 0.10), (0.72, 0.74, 0.77), (0.20, 0.21, 0.23), \
    (0.72, 0.12, 0.10), (0.80, 0.66, 0.30), (0.06, 0.06, 0.06), (0.62, 0.45, 0.12), (0.62, 0.64, 0.66)
GREASE_SPOTS = [(MX + 0.006, MY + 0.0235, TOP + PIN_Z + 0.017), (MX + 0.006, MY - 0.0235, TOP + PIN_Z + 0.017),
                (MX - 0.010, MY + 0.0235, TOP + PIN_Z + 0.015), (MX - 0.010, MY - 0.0235, TOP + PIN_Z + 0.015)]
PARKED = [(0.55, -0.40 + 0.03 * k, TOP - 0.12) for k in range(5)]  # under the table until squeezed out
LEAK_NEAR = 0.06  # a leak with the coupler's mouth this close to the nipple shows at the nipple

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _hexagon(af: float, z0: float, z1: float, color, collide: bool = True) -> list:
    """A hexagonal prism (``af`` across flats, flats facing +-y) from ``z0`` to ``z1`` on the body's axis."""
    side = af / math.sqrt(3.0)
    return [BoxShape((0.0, 0.0, (z0 + z1) / 2), (side, af, z1 - z0), color, yaw=k * math.pi / 3, collide=collide)
            for k in range(3)]


def _pivot() -> Body:
    """The loader arm's pivot: a base plate, a yellow lug, a steel pin through it (washers, cotter pins) with the
    arm's two side plates on it, a boss on the lug's top and the grease nipple screwed into it. Origin on the table
    under the lug. Greasy steel: slick (low friction)."""
    shapes = [BoxShape((0.0, 0.0, BASE_T / 2), (0.12, 0.10, BASE_T), DARK),
              BoxShape((0.0, 0.0, BASE_T + LUG[2] / 2), LUG, YELLOW),
              CylShape((0.0, 0.0, PIN_Z), 0.010, 0.100, STEEL, axis="Y")]
    for s in (-1.0, 1.0):
        shapes += [CylShape((0.0, s * 0.024, PIN_Z), 0.016, 0.004, ZINC, axis="Y", collide=False),  # washers
                   BoxShape((0.0, s * 0.031, PIN_Z), (0.042, 0.008, 0.034), YELLOW, collide=False),  # side plates
                   BoxShape((-0.035, s * 0.031, PIN_Z + 0.004), (0.035, 0.008, 0.026), YELLOW, collide=False),
                   BoxShape((0.0, s * 0.047, PIN_Z), (0.003, 0.003, 0.022), STEEL, collide=False)]  # cotter pins
    b = BOSS_TOP - TOP
    shapes += [CylShape((0.0, 0.0, b - 0.003), 0.012, 0.006, STEEL)]  # the boss
    shapes += _hexagon(HEX_AF, b, b + HEX_H, BRASS)  # the nipple's hex (yellow-zinc-plated steel)
    shapes += [CylShape((0.0, 0.0, b + HEX_H + 0.00125), NECK_R, 0.0025, BRASS),  # its neck
               SphereShape((0.0, 0.0, BALL_Z - TOP), BALL_R, BRASS)]  # its ball head
    return Body.build("pivot", pos=(MX, MY, TOP), mass=5.0, shapes=shapes, kinematic=True, color=YELLOW,
                      friction=(0.15, 0.12), slick=True)


def _pump() -> Body:
    """The grease pail (red, with a yellow band) and its pump on the lid: a column, the head with the lever's
    bracket and the hose outlet on the pail's right side. Origin on the table under the pail's middle."""
    hx = PIVOT[0] - PX
    shapes = [CylShape((0.0, 0.0, PAIL_H / 2), PAIL_R, PAIL_H, RED),
              CylShape((0.0, 0.0, PAIL_H * 0.55), PAIL_R + 0.001, 0.022, YELLOW, collide=False),
              CylShape((0.0, 0.0, PAIL_H + 0.003), PAIL_R + 0.002, 0.006, DARK),
              CylShape((0.0, 0.0, (PAIL_H + 0.006 + 0.108) / 2), 0.011, 0.108 - PAIL_H - 0.006, STEEL),
              BoxShape((hx + 0.008, 0.0, 0.112), (0.040, 0.030, 0.022), DARK),  # the pump head
              BoxShape((hx, 0.0, 0.120), (0.014, 0.034, 0.012), DARK, collide=False),  # the lever's bracket
              CylShape((hx, 0.0, 0.120), 0.004, 0.040, STEEL, axis="Y", collide=False),  # its pivot pin
              CylShape((0.0, -PAIL_R - 0.003, OUTLET[2] - TOP), 0.008, 0.012, BRASS, axis="Y", collide=False)]
    return Body.build("pump", pos=(PX, PY, TOP), mass=6.0, shapes=shapes, kinematic=True, color=RED)


def _lever() -> Body:
    """The pump's lever: a steel bar from the pivot toward the robot, up LEVER_UP, with a red rubber grip pad at its
    end. Origin at the pivot."""
    a = math.radians(LEVER_UP)
    end = np.array([-LEVER_L * math.cos(a), 0.0, LEVER_L * math.sin(a)])
    shapes = [segment((0.0, 0.0, 0.0), tuple((end * 1.06).tolist()), 0.010, STEEL),
              BoxShape(tuple(end.tolist()), PAD, RED)]
    return Body.build("lever", pos=tuple(PIVOT.tolist()), mass=0.12, shapes=shapes, color=STEEL,
                      com=tuple((end * 0.55).tolist()), friction=(1.0, 0.9), ignore=("pump",))


def _holder() -> Body:
    """A steel holster for the coupler on a foot plate: a 22 mm seat 3 mm deep that the nose stands in, under a
    45-degree funnel 40 mm across at its rim, 16 mm above the table. Slick steel, so a coupler set down on the funnel
    slides into the seat. The seat wall, the funnel and the outer wall are 12-sided rings."""
    n = 12
    tan = math.tan(math.pi / n)
    shapes = [CylShape((0.0, 0.0, CUP_FLOOR / 2), CUP_R, CUP_FLOOR, STEEL),
              BoxShape((-0.035, 0.0, 0.0015), (0.03, 0.05, 0.003), DARK, collide=False)]
    for k in range(n):
        a = k * 2 * math.pi / n
        c, s = math.cos(a), math.sin(a)
        r = (SEAT_R + CUP_R) / 2
        shapes.append(BoxShape((r * c, r * s, (CUP_FLOOR + FUNNEL_Z) / 2),
                               (CUP_R - SEAT_R, 2 * CUP_R * tan + 0.0005, FUNNEL_Z - CUP_FLOOR), STEEL, yaw=a))
        t = 0.002
        out = np.array([c, s, -1.0]) / math.sqrt(2.0)
        p1 = np.array([SEAT_R * c, SEAT_R * s, FUNNEL_Z]) + out * t / 2
        p2 = np.array([FUNNEL_R * c, FUNNEL_R * s, CUP_H]) + out * t / 2
        shapes.append(plate(tuple(p1.tolist()), tuple(p2.tolist()), 2 * (FUNNEL_R + t) * tan + 0.001, t,
                            (c, s, -1.0), STEEL))
        r = CUP_R - 0.00075
        shapes.append(BoxShape((r * c, r * s, (FUNNEL_Z + CUP_H) / 2), (0.0015, 2 * CUP_R * tan + 0.0005,
                                                                         CUP_H - FUNNEL_Z), STEEL, yaw=a))
    return Body.build("holder", pos=(HX, HY, TOP), mass=1.0, shapes=shapes, kinematic=True, color=STEEL,
                      friction=(0.15, 0.12), slick=True)


def _hose(i: int) -> Body:
    a, b = np.asarray(HOSE[i], float), np.asarray(HOSE[i + 1], float)
    shapes = [segment((0.0, 0.0, 0.0), tuple((b - a).tolist()), 0.012, BLACK)]
    ign = ("pump", "lever") if i == 0 else ("pump", "lever", f"hose{i - 1}")
    return Body.build(f"hose{i}", pos=tuple(a.tolist()), mass=0.012, shapes=shapes, color=BLACK,
                      damping=(0.6, 0.6), friction=(0.8, 0.7), ignore=ign)


def _coupler() -> Body:
    """A jawless push-type grease coupler. Its hardened nose (18 mm across, 12 mm tall) has a 7.2 mm mouth bore with
    a 45-degree lead-in, and 5 mm up the bore a nitrile seal ring with a 4.6 mm hole whose edge sits on the ball
    head's top; above the nose a black-oxide 13 mm hex body (where it is held) and a brass swivel with the hose's side
    outlet. The nose's walls, lead-in and seal ring are 12-sided rings of boxes. Origin: the middle of its mouth."""
    n = 12
    tan = math.tan(math.pi / n)
    shapes = []
    for k in range(n):
        a = k * 2 * math.pi / n
        c, s = math.cos(a), math.sin(a)
        r = (BORE_R + NOSE_R) / 2  # the bore's wall, from the lead-in up to the nose's top
        shapes.append(BoxShape((r * c, r * s, (CHAMFER_D + NOSE_H) / 2),
                               (NOSE_R - BORE_R, 2 * NOSE_R * tan + 0.0005, NOSE_H - CHAMFER_D), None, yaw=a))
        r = (CHAMFER_R + 0.0001 + NOSE_R) / 2  # the face ring round the lead-in
        shapes.append(BoxShape((r * c, r * s, CHAMFER_D / 2),
                               (NOSE_R - CHAMFER_R - 0.0001, 2 * NOSE_R * tan + 0.0005, CHAMFER_D), None, yaw=a))
        t = 0.0015  # the lead-in: a cone wall from the face's inner edge to the bore
        out = np.array([c, s, 1.0]) / math.sqrt(2.0)
        p1 = np.array([CHAMFER_R * c, CHAMFER_R * s, 0.0]) + out * t / 2
        p2 = np.array([BORE_R * c, BORE_R * s, CHAMFER_D]) + out * t / 2
        shapes.append(plate(tuple(p1.tolist()), tuple(p2.tolist()), 2 * CHAMFER_R * tan + 0.0006, t,
                            (c, s, 1.0), None))
        r = (SEAL_HOLE_R + BORE_R + 0.0001) / 2  # the seal ring
        shapes.append(BoxShape((r * c, r * s, SEAL_Z + SEAL_T / 2),
                               (BORE_R + 0.0001 - SEAL_HOLE_R, 2 * (BORE_R + 0.0001) * tan, SEAL_T), None, yaw=a))
    shapes += [CylShape((0.0, 0.0, (SEAL_Z + SEAL_T + NOSE_H) / 2), BORE_R + 0.0002, NOSE_H - SEAL_Z - SEAL_T, None),
               CylShape((0.0, 0.0, NOSE_H / 2), NOSE_R + 0.0003, NOSE_H, ZINC, collide=False)]  # the nose, drawn
    shapes += _hexagon(BODY_AF, NOSE_H, BODY_Z1, DARK)
    shapes += [CylShape((0.0, 0.0, (BODY_Z1 + SWIVEL_Z1) / 2), SWIVEL_R, SWIVEL_Z1 - BODY_Z1, BRASS),
               CylShape((STUB_L / 2, 0.0, STUB_Z), STUB_R, STUB_L, BRASS, axis="X")]
    return Body.build("coupler", pos=(HX, HY, TOP + CUP_FLOOR), mass=COUPLER_KG, shapes=shapes, color=ZINC,
                      com=(0.0, 0.0, 0.020), friction=(1.2, 1.1), ignore=("hose4",))


def _grease(k: int) -> Body:
    """A dab of grease (drawn only), parked under the table until it appears: the first four squeeze out of the pin's
    bushing, the fifth shows a leak."""
    shapes = [SphereShape((0.0, 0.0, 0.0), 0.0060, GREASE, collide=False),
              SphereShape((0.006, 0.0, -0.002), 0.0050, GREASE, collide=False),
              SphereShape((-0.005, 0.0, -0.003), 0.0045, GREASE, collide=False)]
    return Body.build(f"grease{k}", pos=PARKED[k], mass=0.01, shapes=shapes, kinematic=True, color=GREASE)


OBJECTS = [_pivot(), _pump(), _lever(), _holder()] + [_hose(i) for i in range(5)] + [_coupler()] + \
    [_grease(k) for k in range(5)]
MOVING = ["lever", "coupler"] + [f"hose{i}" for i in range(5)]
hinge("pump_lever", "pump", "lever", anchor=tuple(PIVOT.tolist()), axis="Y", limits_deg=(-26.0, 8.0), spring=0.02,
      damping=0.001, target_deg=4.0)
ball("hose_out", "pump", "hose0", anchor=HOSE[0], swing_damping=0.002, twist_damping=0.002)
for _i in range(1, 5):
    ball(f"hose_j{_i}", f"hose{_i - 1}", f"hose{_i}", anchor=HOSE[_i], swing_damping=0.002, twist_damping=0.002)
ball("hose_in", "hose4", "coupler", anchor=HOSE[5], swing_damping=0.002, twist_damping=0.002)


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"down": False, "strokes": 0, "shots": 0, "leak": False, "seated_ever": False, "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("GREASE_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _set_force(asset, f_body) -> None:
    import torch

    t = torch.tensor(np.asarray(f_body, float), dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), is_global=False)


def _place(base, origin, name, pos) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)


def _coupler_state(cp, cr) -> dict:
    """Where the coupler is against the nipple's ball head, measured in the coupler's own frame: ``off`` how far the
    ball's centre is from its axis, ``lift`` how far its seal is off the ball's top (negative: pressed into it)."""
    q = cr.T @ (np.array([MX, MY, BALL_Z]) - cp)
    off = math.hypot(q[0], q[1])
    lift = BALL_IN_MOUTH - q[2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(cr[2, 2])))))
    seated = off <= SEAT_OFF and -SEAT_SINK <= lift <= SEAT_LIFT and tilt <= SEAT_TILT
    on_ball = off <= PUSH_OFF and lift <= PUSH_LIFT and tilt <= PUSH_TILT
    in_holder = (math.hypot(cp[0] - HX, cp[1] - HY) <= IN_HOLDER_OFF and cp[2] <= TOP + CUP_FLOOR + IN_HOLDER_UP
                 and tilt <= IN_HOLDER_TILT)
    return {"off": off, "lift": lift, "tilt": tilt, "seated": seated, "on_ball": on_ball, "in_holder": in_holder}


def _show_leak(base, origin, cp) -> None:
    """Show the leaked grease: on the nipple's boss when the coupler is at the nipple, else under its mouth."""
    if math.hypot(cp[0] - MX, cp[1] - MY) <= LEAK_NEAR:
        spot = (MX + 0.010, MY, BOSS_TOP + 0.003)
    elif math.hypot(cp[0] - HX, cp[1] - HY) <= CUP_R:
        spot = (cp[0], cp[1], TOP + CUP_FLOOR + 0.003)
    else:
        spot = (cp[0], cp[1], TOP + 0.003)
    _place(base, origin, "grease4", spot)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    cp, cr = _pose(base, origin, "coupler")
    _, lr = _pose(base, origin, "lever")
    c = _coupler_state(cp, cr)
    pressed = -math.degrees(math.atan2(-lr[2, 0], lr[0, 0]))  # degrees the lever is pressed down from rest
    pressure = pressed >= PRESSURE_DEG
    # while the pump pushes grease, its pressure on the seal pushes a coupler on the ball back off along its axis
    _set_force(base.scene["coupler"], (0.0, 0.0, PUSH_N) if pressure and c["on_ball"] else (0.0, 0.0, 0.0))
    if pressure and not c["seated"] and not _EP["leak"]:  # grease squirts out of an unsealed coupler: a leak
        _EP["leak"] = True
        _show_leak(base, origin, cp)
    if not _EP["down"] and pressed >= STROKE_DOWN:
        _EP["down"] = True
        if c["seated"] and not _EP["leak"]:  # a full stroke through a sealed seat squeezes grease into the bushing
            _EP["shots"] += 1
            if _EP["shots"] <= len(GREASE_SPOTS):
                _place(base, origin, f"grease{_EP['shots'] - 1}", GREASE_SPOTS[_EP["shots"] - 1])
    elif _EP["down"] and pressed <= STROKE_UP:
        _EP["down"] = False
        _EP["strokes"] += 1
    _EP["seated_ever"] = _EP["seated_ever"] or c["seated"]
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = dict(c, pressed=pressed)
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0 or (pressure and c["off"] < 0.01):
            tl, trc = _EP["tcps"]
            print(f"[grease] t={_DEBUG['steps'] / 20:5.2f}s lever={pressed:5.1f}deg strokes={_EP['strokes']} "
                  f"shots={_EP['shots']} leak={_EP['leak']} coupler={np.round(cp * 1000, 1).tolist()} "
                  f"off={c['off'] * 1000:5.2f}mm lift={c['lift'] * 1000:6.2f}mm tilt={c['tilt']:5.1f} "
                  f"seated={c['seated']} on_ball={c['on_ball']} in_holder={c['in_holder']} "
                  f"leftTCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"rightTCP={np.round(trc * 1000, 1).tolist()} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0 with the pin greased (at least four full strokes of the pump's lever made through a sealed seat, and no
    leak all trial), the coupler back in its holder cup and both arms back where they started; 0.5 with the pin
    greased at some point and no leak; 0.25 with the coupler sealed on the nipple at some point; else 0.0. A leak
    (the lever pressed past PRESSURE_DEG at any moment while the coupler is not sealed on the nipple) is latched and
    removes the greasing credit."""
    if _EP["shots"] >= STROKES and not _EP["leak"]:
        if _EP["s"].get("in_holder") and _arms_home():
            return 1.0
        return 0.5
    return 0.25 if _EP["seated_ever"] else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with the coupler, the lever and the hose still: the trial ends there."""
    return score(objects) >= 1.0 and all(objects[name].resting() for name in MOVING)


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand comes down round the coupler's hex body in its holder, takes it, lifts it out, carries it over the
# nipple, lowers it onto the ball head and presses it on (aiming PRESS below the seat, so the soft arm leans about
# 20 N on it, more than the 15 N the grease pressure pushes back with), and holds it there. The left hand comes down on
# the lever's grip pad, fingers closed, and pumps the lever five full strokes, following its swing, then rises and goes
# back to where it started. Once the lever is back up, the right pulls the coupler off, carries it back over the
# holder, steadies it just above it, lowers it in, lets go and goes back to where it started.

GRIP = grip_rotation(0.0)  # straight down, fingers closing along y (across the hex body's flats)
GRIP_TCP = 0.027  # the TCP this far above the coupler's mouth when holding it by its hex body
PART_OPEN = -0.020
AIM = (-0.002, 0.0)  # aim the held coupler's mouth this far off the nipple's axis (measured grip offset)
PRESS = 0.047  # aim this far below the seat: the soft arm (~420 N/m) then presses the coupler on with ~20 N
PUMPS = 5
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), label, finger)


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), "back to where it started", finger)


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _pad_top(pressed: float) -> np.ndarray:
    """The top of the lever's grip pad with the lever pressed ``pressed`` degrees down from rest."""
    a = math.radians(LEVER_UP - pressed)
    return PIVOT + np.array([-LEVER_L * math.cos(a), 0.0, LEVER_L * math.sin(a) + PAD[2] / 2])


def _right(release_at: float) -> tuple[list[Waypoint], float]:
    """The right hand; returns its waypoints and when the coupler is pressed on."""
    take = np.array([HX, HY, TOP + CUP_FLOOR + GRIP_TCP])
    out = _lead_in("right", take + [0.0, 0.0, 0.05], GRIP, "over the coupler in its holder", PART_OPEN)
    carry_z = NIPPLE_TOP + 0.030 + GRIP_TCP
    over = np.array([MX + AIM[0], MY + AIM[1], carry_z])
    on = np.array([MX + AIM[0], MY + AIM[1], MOUTH_SEATED + GRIP_TCP + 0.002])
    press = on - [0.0, 0.0, PRESS + 0.002]
    rim = np.array([HX, HY, TOP + CUP_H + 0.012 + GRIP_TCP])  # the coupler's mouth 12 mm over the holder's rim
    out += [_w(1.0, take, PART_OPEN, "down round its hex body", GRIP),
            _w(0.6, take, FINGER_CLOSED, "take it", GRIP),
            _w(1.0, [HX, HY, carry_z], FINGER_CLOSED, "lift it out of the holder", GRIP),
            _w(1.6, over, FINGER_CLOSED, "carry it over the grease nipple", GRIP),
            _w(1.4, on, FINGER_CLOSED, "lower it over the ball head", GRIP),
            _w(0.8, press, FINGER_CLOSED, "press it on", GRIP)]
    seated_at = sum(w.duration for w in out)
    out += [_w(max(release_at - seated_at, 0.5), press, FINGER_CLOSED, "hold it on while the other hand pumps", GRIP),
            _w(0.6, on, FINGER_CLOSED, "ease off", GRIP),
            _w(0.8, over, FINGER_CLOSED, "pull it off the nipple", GRIP),
            _w(1.6, [HX, HY, carry_z], FINGER_CLOSED, "carry it back over the holder", GRIP),
            _w(1.0, rim, FINGER_CLOSED, "down to just above the holder", GRIP),
            _w(0.5, rim, FINGER_CLOSED, "steady it there", GRIP),
            _w(1.0, take + [0.0, 0.0, 0.004], FINGER_CLOSED, "lower it into the holder", GRIP),
            _w(0.5, take + [0.0, 0.0, 0.004], PART_OPEN, "let go", GRIP),
            _w(0.8, take + [0.0, 0.0, 0.05], PART_OPEN, "rise", GRIP)]
    out += _go_home("right", take + [0.0, 0.0, 0.05], GRIP, PART_OPEN)
    return out, seated_at


def _left(start: float, total: float) -> tuple[list[Waypoint], float]:
    """The left hand; returns its waypoints and when it is done pumping."""
    over = _pad_top(0.0) + [0.0, 0.0, 0.03]
    out = _lead_in("left", over, GRIP, "over the pump lever's grip pad", FINGER_CLOSED)
    out.append(_w(max(start - LEAD_S, 0.3), over, FINGER_CLOSED, "wait for the coupler to be held on", GRIP))
    out.append(_w(0.8, _pad_top(0.0) - [0.0, 0.0, 0.002], FINGER_CLOSED, "down onto the pad", GRIP))
    for _ in range(PUMPS):
        for k in range(1, 5):
            out.append(_w(0.25, _pad_top(25.0 * k / 4) - [0.0, 0.0, 0.004], FINGER_CLOSED,
                          "press the lever down a full stroke", GRIP))
        for k in range(3, -2, -1):
            out.append(_w(0.3, _pad_top(25.0 * k / 4) + [0.0, 0.0, 0.003], FINGER_CLOSED,
                          "let it come back up", GRIP))
    out.append(_w(0.8, over, FINGER_CLOSED, "rise off the pad", GRIP))
    done = sum(w.duration for w in out)
    out += _go_home("left", over, GRIP, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s with both arms home", out[-1].yaw))
    return out, done


def demo():
    _, seated_at = _right(0.0)
    _, pumped_at = _left(seated_at + 0.5, 0.0)
    right, _ = _right(pumped_at + 0.5)
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, PART_OPEN, "wait at the start", right[-1].yaw))
    left, _ = _left(seated_at + 0.5, total)
    return {"left": left, "right": right}
