"""Measure fixture-relative pose, stable test outcomes, and repair milestones."""

from dataclasses import dataclass, field
from itertools import product
from math import atan2, cos, degrees, radians, sin

import numpy as np

import task

ALL_FREE = (*task.GUIDES, "test_part")


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


def installed(poses):
    seated = []
    for name in task.GUIDES:
        g = poses[name]
        if (near_xy(g, task.FIX, .0035) and abs(g.pos[2] - task.TOP - task.FLOOR) <= .001
                and g.upright and min(yaw_error(g.rot, 0), yaw_error(g.rot, np.pi)) <= 4):
            seated.append(name)
    return seated[0] if len(seated) == 1 else None


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
    lo, hi = bounds("original_guide", pose)
    low = np.asarray(task.REJECT) - (.063, .0375)
    high = np.asarray(task.REJECT) + (.063, .0375)
    return bool(pose.still and np.all(lo[:2] >= low) and np.all(hi[:2] <= high)
                and abs(lo[2] - (task.TOP + .006)) <= .0015)


def in_cradle(pose):
    return (pose.upright and pose.still and near_xy(pose, task.CRADLE, .005)
            and abs(pose.pos[2] - (task.TOP + .006)) <= .0015)


@dataclass
class Repair:
    hold_steps: int
    fault_proven: bool = False
    replaced: bool = False
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
        part = poses["test_part"]
        mounted = installed(poses)
        if self.proof_guide is not None:
            tested = poses[self.proof_guide]
            # A guide may rock inside its 12 mm nest during withdrawal.
            # Clear proof when it leaves the nest, rather than treating every
            # short violation of the seated-reading tolerance as a removal.
            retained = (near_xy(tested, task.FIX, .006)
                        and abs(tested.pos[2] - task.TOP - task.FLOOR) <= .007
                        and tested.rot[2, 2] >= cos(radians(14))
                        and min(yaw_error(tested.rot, 0), yaw_error(tested.rot, np.pi)) <= 12)
            if not retained or mounted not in (None, self.proof_guide):
                self.verified.clear()
                self.proof_guide = None
        if mounted != self.current:
            self.current = mounted
            self.previous = None
            self.count = self.finish_count = 0

        quarantined = in_reject(poses["original_guide"])
        if self.fault_proven and quarantined and mounted in task.GUIDES[1:] and poses[mounted].still:
            self.replaced = True
        ready_for_verification = self.replaced and quarantined and mounted in task.GUIDES[1:]
        reading = test_reading(part, poses[mounted], clear["fixture"]) if mounted else None
        if reading is None:
            self.count = 0
        else:
            self.count = self.count + 1 if reading == self.previous else 1
        self.previous = reading

        if self.count >= self.hold_steps:
            if mounted == task.GUIDES[0] and reading == ("opposed", "seated"):
                self.fault_proven = True
            elif ready_for_verification:
                if reading in {("aligned", "blocked"), ("opposed", "seated")}:
                    self.verified.clear()
                    self.proof_guide = None
                elif reading in {("aligned", "seated"), ("opposed", "blocked")}:
                    self.proof_guide = mounted
                    self.verified.add(reading)

        full_proof = ready_for_verification and len(self.verified) == 2
        finished = full_proof and in_cradle(part) and all(clear.values()) and poses[mounted].still
        self.finish_count = self.finish_count + 1 if finished else 0
        return .25 * (self.fault_proven + self.replaced + full_proof
                      + (self.finish_count >= self.hold_steps))
