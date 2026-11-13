"""Measure fixture-relative pose, stable test outcomes, and repair milestones."""

from dataclasses import dataclass, field
from itertools import product
from math import atan2, cos, degrees, radians, sin

import numpy as np

import task

ALL_FREE = (*task.GUIDES, "lock_pin", "test_part")
ORIGINAL, MATCHING, WRONG = task.GUIDES


@dataclass
class Pose:
    pos: np.ndarray
    rot: np.ndarray
    linear: float = 0.0
    angular: float = 0.0

    @property
    def still(self):
        return self.linear <= .004 and self.angular <= radians(6)

    @property
    def upright(self):
        return self.rot[2, 2] >= cos(radians(3))


def yaw_error(rotation, yaw):
    current = atan2(rotation[1, 0], rotation[0, 0]) - yaw
    return abs(degrees(atan2(sin(current), cos(current))))


def near_xy(pose, target, tolerance):
    return bool(np.max(np.abs(pose.pos[:2] - target)) <= tolerance)


def offset(pose):
    return pose.pos - np.array([*task.FIX, task.TOP + task.FLOOR])


def slid_home(guide):
    """Slid under the lips to within 2.5 mm of the stop, square and flat."""
    dx, dy, dz = offset(guide)
    return bool(-.0025 <= dx <= .001 and abs(dy) <= .002 and abs(dz) <= .001
            and guide.upright and yaw_error(guide.rot, 0) <= 3)


def pin_home(pin):
    """Collar down on the guide tab, shank through the tab into the base."""
    dx, dy = pin.pos[:2] - np.asarray(task.PIN_HOLE)
    return bool(abs(dx) <= .003 and abs(dy) <= .002 and abs(pin.pos[2] - task.TOP - task.TAB_TOP) <= .001
            and pin.rot[2, 2] >= cos(radians(5)))


def installed(poses):
    """The single guide that is home and locked by the pin, if any."""
    home = [name for name in task.GUIDES if slid_home(poses[name])]
    return home[0] if len(home) == 1 and pin_home(poses["lock_pin"]) else None


def retained(guide, pin):
    """Still in the slot with the pin engaged: small settling is not a removal."""
    dx, dy, dz = offset(guide)
    return bool(abs(dx) <= .004 and abs(dy) <= .003 and abs(dz) <= .002
            and near_xy(pin, task.PIN_HOLE, .0035) and pin.pos[2] - task.TOP - task.TAB_TOP <= .004)


def entered(guide):
    """A guide put into the fixture slot, anywhere from the drop-in position to the stop."""
    dx, dy, dz = offset(guide)
    return bool(-task.ENTRY - .004 <= dx <= .002 and abs(dy) <= .003 and abs(dz) <= .002)


def test_reading(part, guide, hands_clear):
    """Do not infer a valid rejection from failure to descend."""
    if not (hands_clear and part.still and guide.still and part.upright):
        return None
    if not near_xy(part, guide.pos[:2], .0032):
        return None
    for direction, yaw in (("aligned", 0.), ("opposed", np.pi)):
        if yaw_error(part.rot, yaw) <= 5:
            break
    else:
        return None
    height = part.pos[2] - (task.TOP + task.SEAT)
    if abs(height) <= .001:
        return direction, "seated"
    if abs(height - task.BLOCK) <= .001:
        return direction, "blocked"
    return None


_CORNERS = {}
for _name, _parts in task.PARTS.items():
    if _name in ALL_FREE:
        _CORNERS[_name] = np.concatenate([
            np.asarray(p.vertices) if p.vertices else (
                np.asarray(p.pos) + np.asarray(list(product((-.5, .5), repeat=3))) * p.size
            )
            for p in _parts if p.collision
        ])


def bounds(name, pose):
    points = _CORNERS[name] @ pose.rot.T + pose.pos
    return np.min(points, axis=0), np.max(points, axis=0)


def in_reject(pose):
    lo, hi = bounds(ORIGINAL, pose)
    low = np.asarray(task.REJECT) - (.071, .041)
    high = np.asarray(task.REJECT) + (.071, .041)
    return bool(pose.still and np.all(lo[:2] >= low) and np.all(hi[:2] <= high)
                and abs(lo[2] - (task.TOP + .006)) <= .0015)


def in_cradle(pose):
    return bool(pose.upright and pose.still and near_xy(pose, task.CRADLE, .005)
            and abs(pose.pos[2] - (task.TOP + .006)) <= .0015)


@dataclass
class Repair:
    hold_steps: int
    fault_proven: bool = False
    replaced: bool = False
    wrong_tried: bool = False
    current: str | None = None
    verified: set = field(default_factory=set)
    proof_guide: str | None = None
    previous: tuple | None = None
    count: int = 0
    finish_count: int = 0
    invalid: bool = False

    def update(self, poses, clear):
        self.invalid |= any(p.pos[2] < task.TOP - .055 for p in poses.values())
        if self.invalid:
            return 0.0
        # Choosing by part number: putting the 428 spare in the slot ends progress.
        self.wrong_tried |= entered(poses[WRONG])
        part, pin = poses["test_part"], poses["lock_pin"]
        mounted = installed(poses)
        if self.proof_guide is not None and (
                not retained(poses[self.proof_guide], pin) or mounted not in (None, self.proof_guide)):
            self.verified.clear()
            self.proof_guide = None
        if mounted != self.current:
            self.current = mounted
            self.previous = None
            self.count = self.finish_count = 0

        quarantined = in_reject(poses[ORIGINAL])
        if (self.fault_proven and not self.wrong_tried and quarantined and mounted == MATCHING
                and poses[MATCHING].still and pin.still):
            self.replaced = True
        ready = self.replaced and not self.wrong_tried and quarantined and mounted == MATCHING
        reading = test_reading(part, poses[mounted], clear["fixture"]) if mounted else None
        if reading is None:
            self.count = 0
        else:
            self.count = self.count + 1 if reading == self.previous else 1
        self.previous = reading

        if self.count >= self.hold_steps:
            if mounted == ORIGINAL and reading == ("opposed", "seated"):
                self.fault_proven = True
            elif ready:
                if reading in {("aligned", "blocked"), ("opposed", "seated")}:
                    self.verified.clear()
                    self.proof_guide = None
                elif reading in {("aligned", "seated"), ("opposed", "blocked")}:
                    self.proof_guide = mounted
                    self.verified.add(reading)

        full_proof = ready and len(self.verified) == 2
        finished = (full_proof and in_cradle(part) and all(clear.values())
                    and poses[mounted].still and pin.still)
        self.finish_count = self.finish_count + 1 if finished else 0
        if self.wrong_tried:
            return .25 * self.fault_proven
        return .25 * (self.fault_proven + self.replaced + full_proof
                      + (self.finish_count >= self.hold_steps))
