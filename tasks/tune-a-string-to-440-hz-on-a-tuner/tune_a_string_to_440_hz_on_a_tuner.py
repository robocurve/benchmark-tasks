"""On the station table lies a one-string zither: a long box with a string stretched from a bridge to a tuning pin, a key
fitted on the pin, and a clip-on tuner on its head. The string is flat: it sounds 412 Hz.

Task: tune it to 440 Hz. The tuner shows the pitch only when the string is plucked; one hand turns the key clockwise
(each degree raises the pitch 0.2 Hz) while the other plucks the string over and over, watching the tuner's needle,
and the key hand lets go once the needle reads 440 (within 3 Hz) and the tuner's green light is on. As tuners do it,
the final reading must be taken while the key hand is still on the key.

Run it:  inspect-robots run --embodiment yam_arena -E task=tune_a_string_to_440_hz_on_a_tuner.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge, on_reset,  # noqa: E402
                     on_step, rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the zither (fixed): a box along y; the string from a bridge (left) to the tuning pin (right) ------------------------
SX = 0.28  # the string's line
BOX_X, BOX_Y, BOX_H = (0.24, 0.32), (-0.21, 0.13), 0.030
STRING_Z = TOP + BOX_H + 0.010  # the string, over 1 cm bridges
BRIDGE_Y, PIN_Y = 0.10, -0.17
PLUCK_Y = (-0.10, 0.06)  # where along the string a pluck sounds it

# -- the tuning pin and its key: turns about z on a stiff hinge (it stays where it is turned) ------------------------
KEY_W, KEY_Z = 0.016, (BOX_H + 0.015, BOX_H + 0.040)  # a square key head, its bottom and top above the table
F0, HZ_PER_DEG = 412.0, 0.2  # the pitch with the pin as it starts; clockwise (from above) raises it
PIN_DRAG = 0.002  # N m per degree/s: the pin turns stiffly and stays where it is left
TARGET, IN_TUNE, CLOSE = 440.0, 3.0, 10.0

# -- the tuner: a dial on a clip at the zither's right end, facing the arms; its needle and green light are moved by
# the task file ------------------------------------------------------------------------------------------------------
DIAL_AT = np.array([BOX_X[0] - 0.012, -0.195, TOP + BOX_H + 0.045])  # the needle's pivot, on the dial's face
DIAL_R = 0.025
DEG_PER_HZ = 3.0  # the needle swings 3 degrees per Hz off 440, up to 60
LIGHT_AT = DIAL_AT + [0.0, 0.0, DIAL_R + 0.008]
HIDDEN = np.array([0.30, 0.0, TOP - 0.03])

WOOD, DARK, STEEL, BRASS, WHITE, RED, GREEN = (0.62, 0.40, 0.22), (0.12, 0.12, 0.13), (0.80, 0.80, 0.82), \
    (0.78, 0.64, 0.30), (0.95, 0.95, 0.93), (0.85, 0.12, 0.10), (0.10, 0.85, 0.20)

# -- scoring ----------------------------------------------------------------------------------------------------------
PLUCK_R = 0.016  # a TCP this close to the string (across it and above it): its fingers touch it, a pluck ...
REARM = 0.03  # ... once it has been at least this far from the string since its last pluck
ON_KEY = 0.025  # a TCP this close to the key's head holds it
LET_GO = 0.03
READ_MATCH = 1.5  # Hz: the last reading still matches the string (letting go of the key nudges it a little)
SUCCESS_HOLD_S = 1.0


def _zither() -> Body:
    cy = (BOX_Y[0] + BOX_Y[1]) / 2
    shapes = [BoxShape(((BOX_X[0] + BOX_X[1]) / 2, cy, BOX_H / 2), (BOX_X[1] - BOX_X[0], BOX_Y[1] - BOX_Y[0], BOX_H),
                       WOOD),
              CylShape((SX, cy + 0.02, BOX_H + 0.0005), 0.018, 0.001, DARK, collide=False),  # the sound hole
              BoxShape((SX, BRIDGE_Y, BOX_H + 0.005), (0.03, 0.006, 0.010), WHITE),
              BoxShape((SX, PIN_Y + 0.025, BOX_H + 0.005), (0.03, 0.006, 0.010), WHITE),
              CylShape((SX, (BRIDGE_Y + PIN_Y) / 2, STRING_Z - TOP), 0.0007, BRIDGE_Y - PIN_Y, STEEL, collide=False,
                       axis="Y"),  # the string
              # the tuner: a clip and a dial facing the arms, with marks at 440 and +-3 Hz
              BoxShape((DIAL_AT[0] + 0.012, DIAL_AT[1], (BOX_H + DIAL_AT[2] - TOP) / 2), (0.010, 0.012,
                                                                                         DIAL_AT[2] - TOP - BOX_H), DARK),
              CylShape((DIAL_AT[0] + 0.004, DIAL_AT[1], DIAL_AT[2] - TOP), DIAL_R + 0.004, 0.006, DARK, axis="X"),
              CylShape((DIAL_AT[0] + 0.0005, DIAL_AT[1], DIAL_AT[2] - TOP), DIAL_R, 0.001, WHITE, collide=False,
                       axis="X")]
    for deg, color, ln in ((0.0, GREEN, 0.008), (DEG_PER_HZ * IN_TUNE, DARK, 0.005), (-DEG_PER_HZ * IN_TUNE, DARK, 0.005)):
        a = math.radians(deg)
        r = DIAL_R - ln / 2 - 0.001
        shapes.append(BoxShape((DIAL_AT[0] - 0.0002, DIAL_AT[1] - r * math.sin(a), DIAL_AT[2] - TOP + r * math.cos(a)),
                               (0.0006, 0.0012, ln), color, collide=False,
                               quat_xyzw=(math.sin(a / 2), 0.0, 0.0, math.cos(a / 2))))
    return Body.build("zither", pos=(0.0, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.6, 0.5),
                      color=WOOD)


def _key() -> Body:
    """The tuning pin with its key on it; origin on the pin's axis on the zither's top."""
    shapes = [CylShape((0.0, 0.0, 0.0075), 0.004, 0.015, STEEL),
              BoxShape((0.0, 0.0, (KEY_Z[0] + KEY_Z[1]) / 2 - BOX_H), (KEY_W, KEY_W, KEY_Z[1] - KEY_Z[0]), BRASS),
              BoxShape((0.0, KEY_W / 2 - 0.002, KEY_Z[1] - BOX_H + 0.0003), (0.003, 0.004, 0.0006), DARK,
                       collide=False)]  # a mark: shows how far it has turned
    return Body.build("key", pos=(SX, PIN_Y, TOP + BOX_H), mass=0.03, shapes=shapes, friction=(1.2, 1.1), color=BRASS)


def _needle() -> Body:
    """Origin at its pivot; it points up (toward +z) at 440."""
    return Body.build("needle", pos=tuple(HIDDEN.tolist()), mass=0.001, kinematic=True, color=RED,
                      shapes=[BoxShape((-0.0005, 0.0, DIAL_R * 0.45), (0.0008, 0.0015, DIAL_R * 0.9), RED,
                                       collide=False),
                              CylShape((-0.0005, 0.0, 0.0), 0.002, 0.001, DARK, collide=False, axis="X")])


def _light() -> Body:
    return Body.build("tuned_light", pos=tuple(HIDDEN.tolist()), mass=0.001, kinematic=True, color=GREEN,
                      shapes=[SphereShape((0.0, 0.0, 0.0), 0.004, GREEN, collide=False)])


OBJECTS = [_zither(), _key(), _needle(), _light()]
hinge("pin", "zither", "key", anchor=(SX, PIN_Y, TOP + BOX_H), axis="Z", damping=PIN_DRAG)


# -- followed after every control step: the pitch, the plucks, the tuner -----------------------------------------------

def _fresh() -> dict:
    return {"plucks": 0, "armed": [True, True], "reading": None, "read_held": False, "tcps": None, "s": {},
            "shown": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("TUNE_DEBUG")), "steps": 0}


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


def _place(base, origin, name, at, quat_xyzw=(0.0, 0.0, 0.0, 1.0)) -> None:
    import torch

    asset = base.scene[name]
    p = np.asarray(at, float) + origin
    asset.write_root_pose_to_sim(torch.tensor([[*p.tolist(), *quat_xyzw]], dtype=torch.float32, device=asset.device))


def _string_dist(p) -> float | None:
    """How far a point is from the string (None: not over the part of it a pluck sounds)."""
    if not PLUCK_Y[0] <= p[1] <= PLUCK_Y[1]:
        return None
    return math.hypot(float(p[0] - SX), float(p[2] - STRING_Z))


def _show(base, origin, reading) -> None:
    dev = max(-60.0, min(60.0, DEG_PER_HZ * (reading - TARGET)))
    a = math.radians(dev)
    _place(base, origin, "needle", DIAL_AT, (math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)))  # sharp: toward the right
    _place(base, origin, "tuned_light", LIGHT_AT if abs(reading - TARGET) <= IN_TUNE else HIDDEN)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    k = as_numpy(base.scene["key"].data.root_link_pose_w)[0]
    k_pos, k_rot = k[:3] - origin, rotation(k[3:7])
    turned = -math.degrees(math.atan2(float(k_rot[1, 0]), float(k_rot[0, 0])))  # clockwise from above
    if _EP.get("last_yaw") is not None:  # unwrap past half a turn
        d = (turned - _EP["last_yaw"] + 180.0) % 360.0 - 180.0
        _EP["turn"] = _EP.get("turn", 0.0) + d
    else:
        _EP["turn"] = turned
    _EP["last_yaw"] = turned
    pitch = F0 + HZ_PER_DEG * _EP["turn"]
    _EP["tcps"] = _tcps(base, origin)
    head = k_pos + np.array([0.0, 0.0, (KEY_Z[0] + KEY_Z[1]) / 2 - BOX_H])
    on_key = [float(np.linalg.norm(p - head)) < ON_KEY for p in _EP["tcps"]]
    for i, p in enumerate(_EP["tcps"]):
        d = _string_dist(p)
        if d is None or d >= REARM:
            _EP["armed"][i] = True
        elif d < PLUCK_R and _EP["armed"][i]:  # a pluck: the tuner reads the pitch
            _EP["armed"][i] = False
            _EP["plucks"] += 1
            _EP["reading"] = pitch
            _EP["read_held"] = on_key[1 - i]
            _show(base, origin, pitch)
    clear = all(float(np.linalg.norm(p - head)) >= LET_GO and (_string_dist(p) is None or _string_dist(p) >= LET_GO)
                for p in _EP["tcps"])
    _EP["s"] = {"pitch": pitch, "clear": clear}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            tl, tr = _EP["tcps"]
            print(f"[tune] t={_DEBUG['steps'] / 20:5.1f}s turned={_EP['turn']:6.1f}deg pitch={pitch:6.1f}Hz "
                  f"plucks={_EP['plucks']} reading={None if _EP['reading'] is None else round(_EP['reading'], 1)} "
                  f"read_held={_EP['read_held']} on_key={on_key} clear={clear} "
                  f"leftTCP-string={None if _string_dist(tl) is None else round(_string_dist(tl) * 1000)}mm "
                  f"rightTCP-key={np.round((tr - head) * 1000).astype(int).tolist()} score={_points(_status())}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    st = _EP["s"]
    r = _EP["reading"]
    return {"pitch": st.get("pitch", F0), "clear": bool(st.get("clear")), "reading": r, "read_held": _EP["read_held"]}


def _points(s) -> float:
    in_tune = abs(s["pitch"] - TARGET) <= IN_TUNE
    read_ok = (s["reading"] is not None and abs(s["reading"] - TARGET) <= IN_TUNE
               and abs(s["reading"] - s["pitch"]) <= READ_MATCH)
    if in_tune and read_ok and s["read_held"] and s["clear"]:
        return 1.0
    if in_tune:
        return 0.5
    return 0.25 if abs(s["pitch"] - TARGET) <= CLOSE else 0.0


def score(objects) -> float:
    """1.0 with the string at 440 Hz (within 3 Hz), the tuner's last reading (taken by plucking the string while the
    other hand held the key) showing it, and both hands let go; 0.5 with the string at 440 Hz (within 3 Hz) otherwise;
    0.25 within 10 Hz; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the key still: the trial ends once it has held 1 s."""
    return score(objects) >= 1.0 and objects["key"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes down beside the string, fingers closed, and plucks it (sweeping across it), reading 412 Hz. The
# right hand comes straight down on the key, fingers closing across it, and turns it clockwise in two strokes (the
# second while the left plucks again), stopping at 440 Hz; holding the key, it waits for the left's last pluck, which
# reads 440 and lights the tuner. Then both hands rise.


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


KEY_GRIP = _rz(math.pi / 4) @ grip_rotation(0.0)  # straight down, across the key's flats at 45 degrees
PLUCK_GRIP = grip_rotation(0.0)  # straight down, fingers closed
KEY_IN_HAND = 0.008  # the right TCP this far below the key's top
TURNS = (90.0, 62.0)  # degrees, in two strokes (the second slips some in the fingers: about 140 in all)
PLUCK_AT = 0.03  # along the string (y)
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


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _pluck(out: list, label: str) -> None:
    """One pluck: from beside the string (toward the arms), across it at its height, to beside it (far side), and up."""
    z = STRING_Z + 0.006  # the closed fingertips below the string's height as they cross it
    near, far = np.array([SX - 0.03, PLUCK_AT, z]), np.array([SX + 0.03, PLUCK_AT, z])
    out += [_w(0.6, near + [0.0, 0.0, 0.03], FINGER_CLOSED, label, PLUCK_GRIP),
            _w(0.5, near, FINGER_CLOSED, label, PLUCK_GRIP),
            _w(1.0, far, FINGER_CLOSED, label, PLUCK_GRIP),
            _w(0.5, far + [0.0, 0.0, 0.03], FINGER_CLOSED, label, PLUCK_GRIP)]


def _left(times: list[float], total: float) -> list[Waypoint]:
    start = np.array([SX - 0.03, PLUCK_AT, STRING_Z + 0.04])
    out = _lead_in("left", start, PLUCK_GRIP, "over the string, beside it", FINGER_CLOSED)
    for k, t in enumerate(times):
        now = sum(w.duration for w in out)
        out.append(_w(max(t - now, 0.3), start, FINGER_CLOSED, "wait, beside the string", PLUCK_GRIP))
        _pluck(out, f"pluck the string ({k + 1}): the tuner reads it")
        now = sum(w.duration for w in out)
        out.append(_w(0.6, start, FINGER_CLOSED, "back beside it", PLUCK_GRIP))
    now = sum(w.duration for w in out)
    out.append(_w(max(total - now, 0.5), start + [0.0, 0.03, 0.03], FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", PLUCK_GRIP))
    return out


def _right() -> tuple[list[Waypoint], list[float]]:
    head = np.array([SX, PIN_Y, TOP + KEY_Z[1] - KEY_IN_HAND])
    out = _lead_in("right", head + [0.0, 0.0, 0.04], KEY_GRIP, "over the key", FINGER_OPEN)
    out += [_w(1.0, head, FINGER_OPEN, "down round it", KEY_GRIP),
            _w(0.6, head, FINGER_CLOSED, "grip it", KEY_GRIP)]
    plucks = [sum(w.duration for w in out)]  # a first pluck before turning: 412 Hz
    out.append(_w(2.6, head, FINGER_CLOSED, "hold it while the string is plucked", KEY_GRIP))
    for k, deg in enumerate(TURNS):
        for j in range(1, 5):
            out.append(_w(0.6, head, FINGER_CLOSED, f"turn it clockwise ({k + 1})", _rz(-math.radians(deg) * j / 4) @ KEY_GRIP))
        if k == 0:
            out += [_w(0.4, head, FINGER_OPEN, "let go", _rz(-math.radians(deg)) @ KEY_GRIP),
                    *[_w(0.25, head, FINGER_OPEN, "turn back, open", _rz(-math.radians(deg) * (1 - j / 4)) @ KEY_GRIP)
                      for j in range(1, 5)],
                    _w(0.5, head, FINGER_CLOSED, "take it again", KEY_GRIP)]
    end = _rz(-math.radians(TURNS[1])) @ KEY_GRIP
    plucks.append(sum(w.duration for w in out))  # the last pluck while the key is held: 440 Hz
    out += [_w(2.8, head, FINGER_CLOSED, "hold it while the string is plucked: the tuner reads 440", end),
            _w(0.4, head, FINGER_OPEN, "let go", end),
            _w(1.0, head + [0.0, -0.03, 0.04], FINGER_OPEN, "rise", end)]
    return out, plucks


def demo():
    right, plucks = _right()
    total = sum(w.duration for w in right) + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_OPEN, "hold still", right[-1].yaw))
    return {"left": _left(plucks, total), "right": right}
