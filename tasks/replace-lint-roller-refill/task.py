"""Two-arm replacement on the IKEA BÄSTIS-style pull-off lint roller."""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from plan import OPEN, TABLE, TIP, Track, check_tracks, steered
from runtime import bodies, register, reset_contacts, tick

SCENE = "station"
SCORE_OVER = "end"
MAX_STEPS = 1350
SUCCESS_HOLD_S = 1.0
INSTRUCTION = "Hold the lint roller by its handle. Remove the used roll, fit the fresh refill, and lay the reassembled roller back in its rest."
BASE = (0.275, 0.029, TABLE + 0.030)
NEW = (0.362, -0.148, TABLE + 0.030)
DISCARD = (0.172, -0.154, TABLE)
ROLLER_LENGTH = 0.099
HANDLE_LENGTH = 0.120
PIN_LENGTH = 0.092
HOLD_OFFSET = 0.075
ROLL_OFFSET = -0.077
WORK_Z = BASE[2] + 0.024
AXIS = np.array((0.0, -1.0, 0.0))
EXCHANGE_YAW = math.radians(32)
WORK_AXIS = np.array((math.sin(EXCHANGE_YAW), -math.cos(EXCHANGE_YAW), 0.0))
WORK_CENTER = np.array((BASE[0] - 0.061, BASE[1] + HOLD_OFFSET, WORK_Z)) + HOLD_OFFSET * WORK_AXIS


@dataclass
class Trial:
    separated: bool = False
    inserted: bool = False
    joint_lift: bool = False
    hold: float = 0.0
    score: float = 0.0

    def step(
        self,
        old_axial,
        discarded,
        ready,
        supported,
        removed_by_pair,
        seated_by_pair,
        assembled_lift,
        parked,
        dt,
    ):
        self.separated |= old_axial > PIN_LENGTH + 0.004 and removed_by_pair
        self.inserted |= self.separated and discarded and ready and seated_by_pair
        self.joint_lift |= self.inserted and ready and assembled_lift
        self.hold = (
            self.hold + dt
            if self.joint_lift and ready and discarded and parked
            else 0.0
        )
        self.score = (
            1.0
            if self.hold >= 1.0 - 1e-8
            else 0.75
            if self.joint_lift and ready
            else 0.5
            if discarded and ready and self.inserted
            else 0.25
            if discarded and self.separated and supported
            else 0.0
        )


_trial = Trial()
state = {}
poses = {}
bias = np.zeros(3)
handle_axis = AXIS.copy()


def steer_lint(side, label, point, rotation, finger):
    if not poses:
        return point, rotation, finger
    shift = np.zeros(3)
    if side == "left" and label in {
        "lower around the polypropylene handle",
        "hold the handle",
    }:
        shift = poses["roller_handle"] - np.array(BASE)
    elif side == "right":
        if label in {"approach used roll", "grip used refill"}:
            shift = poses["spent"] - WORK_CENTER
        elif label in {"approach fresh roll", "grasp fresh refill"}:
            shift = poses["fresh"] - np.array(NEW)
        elif label in {
            "align the bore at spindle height",
            "start new refill on tapered nose",
            "press refill against the shoulder",
            "release fresh roll without disturbing the fit",
        }:
            handle, fresh = poses["roller_handle"], poses["fresh"]
            if state.get("roll_right") and label in {
                "start new refill on tapered nose",
                "press refill against the shoulder",
            }:
                delta = handle - fresh
                radial = delta - (delta @ handle_axis) * handle_axis
                if np.linalg.norm(radial) < 0.025:
                    correction = np.where(np.abs(radial) > 0.0015, radial, 0.0)
                    bias[:] = np.clip(
                        bias + np.clip(correction * 0.08, -0.00045, 0.00045),
                        -0.009,
                        0.009,
                    )
            shift = bias.copy()
    return np.asarray(point) + np.clip(shift, -0.012, 0.012), rotation, finger


def score(objects):
    return _trial.score


def success(objects):
    return _trial.score == 1.0


def telemetry():
    return dict(
        state,
        paired_removal=_trial.separated,
        paired_insertion=_trial.inserted,
        retention_lift=_trial.joint_lift,
        hold_s=round(_trial.hold, 2),
    )


def setup():
    from scenekit import UsdBody
    import touch

    parts = [
        UsdBody(
            "roller_handle",
            usd="assets/roller_handle.usda",
            pos=BASE,
            size=(0.044, 0.219, 0.044),
            mass=0.062,
            friction=(0.63, 0.47),
            torsional_patch=0.009,
        ),
        UsdBody(
            "spent",
            usd="assets/spent.usda",
            pos=BASE,
            size=(0.044, 0.099, 0.044),
            mass=0.026,
            friction=(0.88, 0.62),
            torsional_patch=0.010,
        ),
        UsdBody(
            "fresh",
            usd="assets/fresh.usda",
            pos=NEW,
            size=(0.048, 0.099, 0.048),
            mass=0.046,
            friction=(0.9, 0.66),
            torsional_patch=0.010,
        ),
        UsdBody(
            "rest",
            usd="assets/rest.usda",
            pos=(*BASE[:2], TABLE),
            size=(0.071, 0.349, 0.053),
            mass=None,
        ),
        UsdBody(
            "refill_rest",
            usd="assets/refill_rest.usda",
            pos=(*NEW[:2], TABLE),
            size=(0.071, 0.105, 0.03),
            mass=None,
        ),
        UsdBody(
            "discard",
            usd="assets/discard.usda",
            pos=DISCARD,
            size=(0.091, 0.126, 0.016),
            mass=None,
        ),
    ]

    settled_at = None

    def reset(env):
        global _trial
        nonlocal settled_at
        _trial = Trial()
        settled_at = None
        state.clear()
        poses.clear()
        bias[:] = 0.0
        reset_contacts()

    measure_clearance = os.environ.get("FAILSAFE_ARM_CONTACTS") == "1"

    def update(env):
        nonlocal settled_at
        names = ["roller_handle", "spent", "fresh"]
        posing = bodies(env, names)
        handle, old, fresh = (posing[name] for name in names)
        poses.update((name, posing[name].pos.copy()) for name in names)
        contacts = touch.touching_by_arm(env, names)
        if measure_clearance:
            clearance = touch.interarm_contacts(env)
            if clearance is not None:
                distance, left_link, right_link = clearance
                smallest = state.get("closest_arm_contact")
                if smallest is None or 1000 * distance < smallest[0]:
                    state["closest_arm_contact"] = (1000 * distance, left_link, right_link)
        left, right = contacts["left"], contacts["right"]
        supported = left["roller_handle"] and not right["roller_handle"]
        holding_old, holding_new = right["spent"], right["fresh"]
        holding_new_any = left["fresh"] or holding_new
        axis = handle.rotation() @ AXIS
        handle_axis[:] = axis
        old_axial = float(np.dot(old.pos - handle.pos, axis))
        delta = fresh.pos - handle.pos
        travel = float(np.dot(delta, axis))
        coaxial = (fresh.rotation() @ AXIS) @ axis
        ready = (
            abs(travel) < 0.004
            and np.linalg.norm(delta - travel * axis) < 0.0035
            and coaxial > math.cos(math.radians(7.0))
        )
        target_old = np.array(DISCARD) + (0.0, 0.048, 0.026)
        discarded = (
            np.linalg.norm(old.pos[:2] - target_old[:2]) < 0.028
            and TABLE + 0.014 < old.pos[2] < TABLE + 0.037
            and not (left["spent"] or holding_old)
            and old.resting(0.019, 0.22)
        )
        assembled_lift = (
            supported and not holding_new_any and handle.pos[2] > WORK_Z + 0.015
        )
        parked = np.linalg.norm(handle.pos - np.array(BASE)) < 0.012 and not (
            left["roller_handle"] or right["roller_handle"] or holding_new_any
        )
        static = False
        if not parked:
            settled_at = None
        else:
            pose = np.stack((handle.pos, fresh.pos))
            axis_pair = np.stack((axis, fresh.rotation() @ AXIS))
            if settled_at is None:
                settled_at = (pose, axis_pair)
            else:
                reference, directions = settled_at
                static = np.linalg.norm(
                    pose - reference, axis=1
                ).max() < 0.002 and np.min(
                    np.sum(axis_pair * directions, axis=1)
                ) > math.cos(
                    math.radians(3)
                )
                if not static:
                    settled_at = (pose, axis_pair)
        _trial.step(
            old_axial,
            discarded,
            ready,
            supported,
            supported and holding_old,
            supported and holding_new,
            assembled_lift,
            parked and static,
            tick(env),
        )
        state.update(
            handle_left=supported,
            roll_right=holding_new,
            spent_right=holding_old,
            handle_z_mm=round(1000 * (handle.pos[2] - TABLE), 1),
            spent_z_mm=round(1000 * (old.pos[2] - TABLE), 1),
            axial_mm=round(1000 * travel, 1),
            spent_axial_mm=round(1000 * old_axial, 1),
            seated=bool(ready),
            discarded=bool(discarded),
            parked=bool(parked and static),
            radial_mm=round(1000 * np.linalg.norm(delta - travel * axis), 1),
            angle_deg=round(math.degrees(math.acos(np.clip(coaxial, -1.0, 1.0))), 1),
        )

    return register(parts, reset, update)


def tracks():
    left, right = Track(), Track()
    rotation = math.pi / 2
    handle_yaw = -math.pi / 2
    hover = TABLE + 0.139
    handle = np.array(BASE) + (0.0, HOLD_OFFSET, TIP - 0.002)
    h_lift = (*handle[:2], WORK_Z + TIP - 0.002)
    h_work = WORK_CENTER - HOLD_OFFSET * WORK_AXIS + (0.0, 0.0, TIP - 0.002)
    work_yaw = rotation + EXCHANGE_YAW
    support_yaw = handle_yaw + EXCHANGE_YAW

    def sleeve(travel, z):
        center = WORK_CENTER + (travel - ROLL_OFFSET) * WORK_AXIS
        return (*center[:2], z)
    left.go((*handle[:2], hover), OPEN, "above BÄSTIS handle", 1.7, handle_yaw)
    left.go(handle, OPEN, "lower around the polypropylene handle", 1.5, handle_yaw)
    left.go(handle, -0.004, "hold the handle", 0.9, handle_yaw)
    left.go(h_lift, -0.006, "lift assembled roller clear of rest", 1.45, handle_yaw)
    left.go(h_work, -0.006, "turn roller into the exchange position", 1.55, support_yaw)
    right.go((0.226, -0.232, hover), OPEN, "wait clear of the roller", 1.8, rotation)
    right.wait_until(left.duration + 0.35)

    right_park = (0.226, -0.232, hover)
    roll_z = WORK_Z + TIP - 0.003
    roll = sleeve(0.0, roll_z)
    away = sleeve(PIN_LENGTH + 0.015, roll_z)
    right.go((*away[:2], hover), OPEN, "approach used roll from the right lane", 1.5, work_yaw)
    right.go((*roll[:2], hover), OPEN, "above used paper roll", 1.4, work_yaw)
    right.go(roll, OPEN, "approach used roll", 1.45, work_yaw)
    right.go(roll, -0.006, "grip used refill", 0.85, work_yaw)
    left.wait_until(right.duration)
    partial_pull = sleeve(0.065, roll_z)
    handle_back = h_work - 0.050 * WORK_AXIS
    left.go(handle_back, -0.006, "counter the axial pull", 2.8, support_yaw)
    right.go(partial_pull, -0.004, "pull refill axially off the spindle", 2.8, work_yaw)
    discard_pose = (
        DISCARD[0],
        DISCARD[1] + 0.048 + ROLL_OFFSET,
        TABLE + 0.027 + TIP - 0.003,
    )
    right.go((*partial_pull[:2], hover), -0.004, "lift used refill clear", 1.55, work_yaw)
    right.go((*discard_pose[:2], hover), -0.004, "move to discard tray", 1.75, rotation)
    right.go(discard_pose, -0.004, "lower used refill", 1.4, rotation)
    right.go(discard_pose, OPEN, "release used refill", 0.85, rotation)
    right.go((*discard_pose[:2], hover), OPEN, "withdraw from discard", 1.25, rotation)

    right.go(right_park, OPEN, "park clear before crossing to fresh refill", 1.4, rotation)
    left.wait_until(right.duration)
    left.go(h_work, -0.006, "restore the handle exchange pose", 1.4, support_yaw)
    right.wait_until(left.duration)

    fresh = (NEW[0], NEW[1] + ROLL_OFFSET, NEW[2] + TIP - 0.003)
    right.go((*fresh[:2], hover), OPEN, "above fresh refill", 1.45, rotation)
    right.go(fresh, OPEN, "approach fresh roll", 1.35, rotation)
    right.go(fresh, -0.017, "grasp fresh refill", 0.85, rotation)
    right.go((*fresh[:2], hover), -0.017, "lift fresh refill", 1.5, rotation)
    right.go((*away[:2], hover), -0.017, "align fresh bore to spindle", 1.55, work_yaw)
    feed_z = roll_z + 0.0031
    right.go((*away[:2], feed_z), -0.017, "align the bore at spindle height", 1.45, work_yaw)
    right.go(sleeve(0.051, feed_z), -0.017, "start new refill on tapered nose", 1.7, work_yaw)
    right.go(sleeve(-0.015, feed_z), -0.017, "press refill against the shoulder", 2.1, work_yaw)
    right.go(sleeve(-0.015, feed_z), -0.031, "release fresh roll without disturbing the fit", 1.0, work_yaw)
    right.go(sleeve(0.065, feed_z), -0.031, "retreat along the spindle axis", 1.35, work_yaw)
    right.go(sleeve(0.065, hover), -0.031, "lift clear of fitted roll", 1.3, work_yaw)
    right.go((0.248, -0.238, hover), OPEN, "park right arm", 1.45, rotation)
    left.wait_until(right.duration)
    left.go(
        (h_work[0], h_work[1], h_work[2] + 0.043),
        -0.007,
        "verify refill retains during a handle-only lift",
        1.3,
        support_yaw,
    )
    left.go((*h_lift[:2], h_work[2] + 0.043), -0.007, "turn over the rest", 1.8, handle_yaw)
    left.go(handle, -0.007, "return assembled roller to the rest", 1.8, handle_yaw)
    left.go(handle, OPEN, "release assembled lint roller", 0.85, handle_yaw)
    left.go((*handle[:2], hover), OPEN, "clear the reassembled tool", 1.4, handle_yaw)
    right.wait_until(left.duration)
    return {"left": left, "right": right}


def demo():
    paths = tracks()
    return steered(paths["left"], paths["right"], steer_lint)


if __name__ == "__main__":
    test = Trial()
    test.step(0.11, False, False, True, True, False, False, False, 0.05)
    test.step(0.15, True, True, True, False, True, False, False, 0.05)
    assert test.score == 0.5
    test.step(0.15, True, True, True, False, False, True, False, 0.05)
    test.step(0.15, True, True, False, False, False, False, True, 1.0)
    assert test.score == 1.0
    check_tracks(tracks(), MAX_STEPS, ik="--ik" in sys.argv)
    print("BÄSTIS dual-contact and retention-lift scoring passed.")
else:
    OBJECTS = setup()
