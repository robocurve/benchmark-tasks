"""Guide a spring-return projection screen into its case using its pull tab."""
from __future__ import annotations

import sys
from collections import deque
from math import cos, sin, sqrt
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from scenekit import Shape, UsdBody, arm_joints, on_reset, on_step
import tiltkit

SCENE = "station"
INSTRUCTION = "Put the projector screen away. Guide it back into the case."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

X, Y, Z = .498, -.08, 1.160
START = .275
UNLOCK = .328
MAX_RETURN_SPEED = .22
HALFWAY = (.110, .170)
STOW_TOL = .012
TAB_X, TAB_Z = X - .016, Z - .028
SLIDE = "panel_slide"
STATE = {"unlocked": False, "snap": False, "still": False, "clear": False}
RECENT = deque(maxlen=5)

SCREEN = UsdBody(
    "screen", usd="assets/screen.usda", pos=(X, Y, Z), size=(.08, 1.95, .45),
    rot_xyzw=(0., 0., -sqrt(.5), sqrt(.5)), articulation=True, fix_root=True,
    joint_pos=((SLIDE, -START),), joint_damping=10., material=False,
)
OBJECTS = [
    SCREEN,
    # The opaque header masks the stored part of the rigid screen graphic.
    Shape("header", pos=(X-.011, Y, Z+.296), size=(.022, 1.96, .346),
          color=(.18, .20, .23), collide=False),
    Shape("mount_left", pos=(X+.055, Y+.955, .62), size=(.08, .055, 1.24),
          color=(.20, .23, .27)),
    Shape("mount_right", pos=(X+.055, Y-.955, .62), size=(.08, .055, 1.24),
          color=(.20, .23, .27)),
]


def screen_status(extension, return_rate, unlocked, snap):
    """Latch release and overspeed are determined by the contacted mechanism."""
    unlocked = unlocked or extension >= UNLOCK
    snap = snap or (unlocked and return_rate > MAX_RETURN_SPEED)
    return unlocked, snap


def reel_force(extension, unlocked):
    # Reel spring has a positive preload. The ratchet holds its initial stop
    # against that spring and offers no resistance to a downward release pull.
    force = 6.1 + 8.*extension
    if not unlocked:
        # The catch cancels the rewind force at its stop. Blend the reaction
        # over 8 mm so the 20 Hz effort updates cannot excite a hard impact.
        engagement = max(0., min(1., 1. - (extension - START)/.008))
        force = (1. - engagement)*force - 180.*max(0., START - extension)
    return force


def apply_spring(asset, extension):
    force = reel_force(extension, STATE["unlocked"])
    targets = asset.data.joint_pos
    targets = getattr(targets, "torch", targets)
    asset.set_joint_effort_target(targets.new_full(targets.shape, force))


@on_reset
def reset(environment):
    STATE.update(unlocked=False, snap=False, still=True, clear=False)
    RECENT.clear()
    asset = environment.unwrapped.scene["screen"]
    template = asset.data.joint_pos
    template = getattr(template, "torch", template)
    asset.write_joint_position_to_sim_index(position=template.new_full(template.shape, -START))
    asset.write_joint_velocity_to_sim_index(velocity=template.new_zeros(template.shape))
    apply_spring(asset, START)


@on_step
def track(environment):
    asset = environment.unwrapped.scene["screen"]
    position = asset.data.joint_pos
    extension = -float(getattr(position, "torch", position)[0, 0])
    RECENT.append(extension)
    measured_return = (RECENT[-3] - extension)/.1 if len(RECENT) >= 3 else 0.
    STATE["unlocked"], STATE["snap"] = screen_status(
        extension, measured_return, STATE["unlocked"], STATE["snap"])
    STATE["still"] = len(RECENT) == 5 and max(RECENT) - min(RECENT) <= .005
    STATE["clear"] = extension <= STOW_TOL and arms_clear(environment)
    apply_spring(asset, extension)


def arms_clear(environment):
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    for side in station.BASES:
        joints = arm_joints(environment, side, ARM_JOINTS)
        tip, _ = ik.tcp_pose(joints, station.BASES[side])
        if tip[0] > TAB_X - .05:
            return False
    return True


def outcome(extension, unlocked, snap, still, clear):
    if not unlocked or snap or not still:
        return 0.
    if extension <= STOW_TOL:
        return float(clear)
    return .5 if HALFWAY[0] <= extension <= HALFWAY[1] else 0.


def score(objects):
    return outcome(-objects["screen"].joints[SLIDE], **STATE)


def success(objects):
    return score(objects) == 1.


def demo():
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    tiltkit.install()
    pose = tiltkit.grip(approach=(cos(.20), 0, -sin(.20)), closing=(0, -1, 0))
    loaded_pose = tiltkit.Tilt(pose.rot, load=(-.84, (0., 0., 0.)))

    def move(seconds, x, extension, aperture, label):
        orientation = loaded_pose if label in ("release catch", "maintain grip", "guide screen into housing") else pose
        return Waypoint(seconds, (x, Y, TAB_Z-extension-.009), aperture, label, orientation)

    return {"right": [
        move(3., TAB_X-.10, START, FINGER_OPEN, "approach tab"),
        move(2., TAB_X+.008, START, FINGER_OPEN, "insert fingers"),
        move(1., TAB_X+.008, START, FINGER_CLOSED, "pinch tab"),
        move(2., TAB_X+.008, UNLOCK+.022, FINGER_CLOSED, "release catch"),
        move(.75, TAB_X+.008, UNLOCK+.022, FINGER_CLOSED, "maintain grip"),
        move(9., TAB_X+.008, .002, FINGER_CLOSED, "guide screen into housing"),
        move(1.5, TAB_X+.008, .002, FINGER_OPEN, "let go at housing"),
        move(2., TAB_X-.10, .002, FINGER_OPEN, "retreat"),
    ]}


def self_test():
    assert outcome(0., True, False, True, True) == 1.
    assert outcome(0., True, False, True, False) == 0.
    assert outcome(.14, True, False, True, False) == .5
    assert outcome(0., True, True, True, True) == 0.
    assert outcome(0., False, False, True, True) == 0.
    assert outcome(.14, True, False, False, False) == 0.
    assert reel_force(START, False) == 0.
    assert reel_force(START-.010, False) < 0.
    assert reel_force(UNLOCK, False) == reel_force(UNLOCK, True)
    assert screen_status(START, .30, False, False) == (False, False)
    assert screen_status(UNLOCK, .06, False, False) == (True, False)
    assert screen_status(.20, .30, True, False) == (True, True)
    assert screen_status(0., 0., True, True) == (True, True)


if __name__ == "__main__":
    from yam_arena.demo import ArmPlan
    from yam_arena.station import BASES

    self_test()
    print("Scorer and reel checks passed")
    for side, waypoints in demo().items():
        plan = ArmPlan(waypoints, BASES[side])
        for i in range(1, round(plan.duration*20)+1):
            plan.action7(i/20)
        print(side, "IK error mm / degrees:", plan.worst_ik_mm, plan.worst_ik_deg)
        assert plan.worst_ik_mm < 2. and plan.worst_ik_deg < 2.
