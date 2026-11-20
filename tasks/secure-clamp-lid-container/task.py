"""Seat a rigid food-storage lid and close all four latches."""
from __future__ import annotations

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from plan import OPEN, TABLE, TIP, Track, waypoints, check_tracks
from runtime import (
    asset,
    bodies,
    contacts,
    install_writer,
    register,
    reset_contacts,
    tick,
)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 1600
INSTRUCTION = "Put the lid on the glass leftovers container. Press down on the lid while closing all four tabs, then let go."
BOWL = (0.237, -0.012, TABLE)
SEAT = (BOWL[0], BOWL[1], TABLE + 0.066)
START = (0.206, 0.180, TABLE + 0.027)
GRIP_Z = 0.017
OPEN_ANGLE = math.radians(-57)
CLOSED_AT = math.radians(-7)
RIM_EDGE = 0.074
RIM_UNDERSIDE = TABLE + 0.0588
INNER_CATCH_TOP = (-0.008, -0.0102)
FLAPS = (
    (0.079, 0.0, 0.0),
    (0.0, 0.079, math.pi / 2),
    (-0.079, 0.0, math.pi),
    (0.0, -0.079, 3 * math.pi / 2),
)
state = {}
_points = 0.0
_hold = 0.0
_drive = None


def tensor(value):
    return getattr(value, "torch", value)


class Latches:
    def __init__(self, unit):
        self.unit = unit
        self.ids = [unit.joint_names.index(f"latch{i}") for i in range(4)]
        install_writer(unit, self.step)

    def step(self):
        import torch

        q = tensor(self.unit.data.joint_pos)[:, self.ids]
        goal = torch.where(q > OPEN_ANGLE * 0.43, 0.0, OPEN_ANGLE)
        speed = tensor(self.unit.data.joint_vel)[:, self.ids]
        effort = (0.011 * (goal - q) - 0.00025 * speed).clamp(-0.010, 0.010)
        self.unit.set_joint_effort_target(effort, joint_ids=self.ids)


def catch_geometry(pose, angles):
    rotation = pose.rotation()
    for (x, y, yaw), angle in zip(FLAPS, angles):
        outward = np.array((math.cos(yaw), math.sin(yaw), 0.0))
        cx, cz = INNER_CATCH_TOP
        radial = cx * math.cos(angle) + cz * math.sin(angle)
        height = -cx * math.sin(angle) + cz * math.cos(angle)
        local = np.array((x, y, height)) + radial * outward
        tip = pose.pos + rotation @ local
        overlap = RIM_EDGE - (tip - BOWL) @ outward
        under = RIM_UNDERSIDE - tip[2]
        yield float(overlap), float(under)


def evaluate(pose, angles, free):
    d = pose.pos - np.array(SEAT)
    seated = bool(
        np.linalg.norm(d[:2]) < 0.004
        and -0.003 < d[2] < 0.004
        and pose.rotation()[2, 2] > 0.994
        and pose.rotation()[0, 0] > 0.986
    )
    closed = (
        sum(
            a >= CLOSED_AT and overlap > 0.0 and 0.0 <= under < 0.008
            for a, (overlap, under) in zip(angles, catch_geometry(pose, angles))
        )
        if seated else 0
    )
    return seated, closed, closed == 4 and free and pose.resting(0.014, 0.16)


def score(objects):
    return _points


def success(objects):
    return _points == 1.0


def telemetry():
    return dict(state, hold=round(_hold, 2))


def setup():
    from scenekit import UsdBody

    items = [
        UsdBody(
            "lid_rest",
            usd="assets/lid_rest.usda",
            pos=(*START[:2], TABLE),
            size=(0.112, 0.112, 0.025),
            mass=None,
        ),
        UsdBody(
            "container",
            usd="assets/container.usda",
            pos=BOWL,
            size=(0.15, 0.118, 0.055),
            mass=None,
        ),
        UsdBody(
            "lid",
            usd="assets/lid.usda",
            pos=START,
            size=(0.20, 0.168, 0.054),
            articulation=True,
            joint_pos=tuple((f"latch{i}", OPEN_ANGLE) for i in range(4)),
            joint_damping=0.0,
            joint_effort_limit=0.04,
            friction=(0.9, 0.7),
        ),
    ]
    names = ["lid", *(f"lid/flap{i}" for i in range(4))]

    def reset(env):
        global _points, _hold, _drive
        _points, _hold = 0.0, 0.0
        state.clear()
        reset_contacts()
        unit = asset(env, "lid")
        if _drive is None or _drive.unit is not unit:
            _drive = Latches(unit)
        position = tensor(unit.data.joint_pos)[:, _drive.ids].clone()
        position.fill_(OPEN_ANGLE)
        unit.write_joint_state_to_sim_index(
            position=position, velocity=position.new_zeros(position.shape),
            joint_ids=_drive.ids,
        )

    def update(env):
        global _points, _hold
        hit = contacts(env, names)
        lid = bodies(env, ["lid"])["lid"]
        angles = [lid.joints[f"latch{i}"] for i in range(4)]
        seated, count, ready = evaluate(lid, angles, not any(hit.values()))
        _hold = _hold + tick(env) if ready else 0.0
        _points = 1.0 if _hold >= 1.0 - 1e-8 else 0.25 + 0.15 * count if seated else 0.0
        state.update(
            seat_error_mm=np.round((lid.pos - SEAT) * 1000, 1).tolist(),
            lid_free=not any(hit.values()),
            seated=seated,
            latched=count,
            angles=[round(math.degrees(a), 1) for a in angles],
            catch_overlap_and_drop_mm=[
                [round(1000 * overlap, 1), round(1000 * under, 1)]
                for overlap, under in catch_geometry(lid, angles)
            ],
        )

    return register(items, reset, update)


def tracks():
    left, right = Track(), Track()
    hover = TABLE + 0.145

    def press(track, index):
        x, y, yaw = FLAPS[index]
        ux, uy = math.cos(yaw), math.sin(yaw)
        x, y = SEAT[0] + x, SEAT[1] + y
        pose_yaw = -math.pi / 2 if index == 2 else 0.0

        def at(offset, z):
            return x + offset * ux, y + offset * uy, z

        track.go(at(0.007, hover), 0.0, f"above latch {index+1}", 1.6, pose_yaw)
        track.go(
            at(0.006, SEAT[2] + 0.014 + TIP),
            0.0,
            "lower onto latch face",
            1.35,
            pose_yaw,
        )
        track.go(
            at(0.009, SEAT[2] - 0.0033 + TIP),
            0.0,
            "press tab past over-center",
            1.65,
            pose_yaw,
        )
        track.go(
            at(0.016, SEAT[2] + 0.016 + TIP),
            0.0,
            "withdraw above tab before lateral travel",
            1.3,
            pose_yaw,
        )
        track.go(at(0.021, hover), 0.0, "lift clear of outer tab", 1.05, pose_yaw)

    left_idle = (0.157, 0.193, hover)
    right_idle = (0.170, -0.192, hover)
    a = (*START[:2], START[2] + GRIP_Z + TIP - 0.002)
    b = (*SEAT[:2], SEAT[2] + GRIP_Z + TIP - 0.002)
    left.go((*a[:2], hover), OPEN, "above the loose lid", 1.5)
    left.go(a, OPEN, "lower around lid pickup loop", 1.3)
    left.go(a, -0.006, "grasp the loose lid", 0.9)
    left.go((*a[:2], hover), -0.006, "lift the lid clear", 1.4)
    left.go((*b[:2], hover), -0.006, "align lid over the glass rim", 1.8)
    left.go(b, -0.006, "seat the lid and gasket", 2.0)
    left.go(b, -0.022, "release the pickup loop", 1.0)
    left.go((*b[:2], hover), -0.022, "lift off the loop before moving", 1.35)

    left_brace = (SEAT[0] - 0.049, SEAT[1] + 0.044, SEAT[2] + TIP + 0.0013)
    right_brace = (SEAT[0] + 0.054, SEAT[1] - 0.054, SEAT[2] + TIP - 0.0025)

    def brace(track, point):
        track.go((*point[:2], hover), 0.0, "above far edge of lid", 1.45)
        track.go(point, 0.0, "hold lid down at far edge", 1.45)

    def unbrace(track, point, idle):
        track.go((*point[:2], hover), 0.0, "raise bracing fingers", 1.2)
        track.go(idle, OPEN, "park away from the other arm", 1.5)

    right.go(right_idle, OPEN, "park during lid placement", 1.65)
    brace(left, left_brace)
    right.wait_until(left.duration + 0.20)
    for index in (3, 0):
        press(right, index)
    right.go(right_idle, OPEN, "clear first pair of latches", 1.5)
    left.wait_until(right.duration)
    unbrace(left, left_brace, left_idle)
    right.wait_until(left.duration)
    brace(right, right_brace)
    left.wait_until(right.duration + 0.20)
    for index in (1, 2):
        press(left, index)
    left.go(left_idle, OPEN, "clear last pair of latches", 1.5)
    right.wait_until(left.duration)
    unbrace(right, right_brace, right_idle)
    left.wait_until(right.duration)
    return {"left": left, "right": right}


def demo():
    t = tracks()
    return waypoints(t["left"], t["right"])


if __name__ == "__main__":
    from types import SimpleNamespace

    p = SimpleNamespace(
        pos=np.array(SEAT), rotation=lambda: np.eye(3), resting=lambda *args: True
    )
    assert evaluate(p, [0.0, 0.0, 0.0, OPEN_ANGLE], True) == (True, 3, False)
    assert evaluate(p, [0.0] * 4, True) == (True, 4, True)
    check_tracks(tracks(), MAX_STEPS, ik="--ik" in sys.argv)
    print("Rigid lid retention and waypoint checks passed.")
else:
    OBJECTS = setup()
