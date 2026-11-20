"""Set and lock the height of a mounted silicon cross-section below a fixed gauge."""
from dataclasses import dataclass
from math import degrees
from bench import TABLE, TIP, OPEN, HOVER, Track, bodies, contacts, rig, register_trial, proof, waypoints
from runtime import asset, install_writer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 720
INSTRUCTION = "Raise the mounted chip to just below the height gauge. Hold the pillar steady while tightening the locking nut, then release both hands."
BASE = (0.284, 0.0, TABLE)
GAUGE = 0.050
INITIAL_TOP = 0.04125
TRAVEL = 0.008
NUT_ANGLE = 0.29
STATE = {}


@dataclass
class Trial:
    locked_height: float | None = None
    hold: float = 0.0
    value: float = 0.0

    def step(self, q, angle, supported, turning, free, dt):
        gap = GAUGE - INITIAL_TOP - q
        in_band = 0.0005 <= gap <= 0.001
        if angle < NUT_ANGLE:
            self.locked_height = None
        elif self.locked_height is None and in_band and supported and turning:
            self.locked_height = q
        locked = self.locked_height is not None
        steady = locked and abs(q - self.locked_height) <= 0.00025
        self.hold = self.hold + dt if steady and in_band and free else 0.0
        self.value = 1.0 if self.hold >= 1 - 1e-8 else 0.65 if steady and in_band else 0.35 if in_band else 0.0


_trial = Trial()


def score(objects):
    return _trial.value


def success(objects):
    return _trial.value == 1.0


def telemetry():
    return dict(STATE, locked=_trial.locked_height is not None, released_s=round(_trial.hold, 2))


def setup():
    fixture = rig(BASE, (0.16, 0.18, 0.063), {"height": 0.0, "nut": 0.0}, 0.35, 0.0, 12.0)

    def sample(environment):
        unit = asset(environment, "rig")
        if not getattr(unit, "_sem_brake", False):
            height = [unit.joint_names.index("height")]
            nut = [unit.joint_names.index("nut")]

            def brake():
                q = unit.data.joint_pos.torch[:, height]
                velocity = unit.data.joint_vel.torch[:, height]
                target = _trial.locked_height
                if target is None:
                    unit.set_joint_effort_target(q * 0, joint_ids=height)
                    unit.set_joint_effort_target(q * 0, joint_ids=nut)
                else:
                    unit.set_joint_effort_target(((target - q) * 6500 - 13 * velocity).clamp(-12, 12), joint_ids=height)
                    angle = unit.data.joint_pos.torch[:, nut]
                    unit.set_joint_effort_target(((0.43 - angle) * 0.12).clamp(-0.04, 0.04), joint_ids=nut)

            install_writer(unit, brake)
            unit._sem_brake = True
        q = bodies(environment, ["rig"])["rig"].joints
        hit = contacts(environment, ["rig/pillar", "rig/locknut"])
        gap = GAUGE - INITIAL_TOP - q["height"]
        STATE.update(gap_mm=round(gap * 1000, 3), nut_deg=round(degrees(q["nut"]), 1),
                     pillar_touched=hit["rig/pillar"], nut_touched=hit["rig/locknut"])
        return q["height"], q["nut"], hit["rig/pillar"], hit["rig/locknut"], not any(hit.values())

    return register_trial([fixture], Trial, STATE, sample, globals())


def tracks():
    left, right = Track(), Track()
    grip = (BASE[0], BASE[1] + 0.103, TABLE + 0.039 + TIP)
    raised = (grip[0], grip[1], grip[2] + 0.010)
    left.go((*grip[:2], HOVER), OPEN, "above pillar steadying grip")
    left.go(grip, OPEN, "approach pillar steadying grip")
    left.go(grip, 0.0, "grasp pillar support")
    left.go(raised, 0.0, "raise specimen to gauge", 1.6)
    start = (BASE[0] - 0.027, BASE[1] - 0.104, TABLE + 0.031 + TIP)
    end = (BASE[0] + 0.050, BASE[1] - 0.095, start[2])
    right.go((*start[:2], HOVER), OPEN, "park over nut wrench")
    right.wait_until(left.duration + 0.3)
    right.go(start, 0.0, "lower beside nut wrench handle")
    right.go(end, 0.0, "turn nut against held pillar", 2.5)
    right.go((end[0] - 0.015, end[1] - 0.015, end[2]), 0.0, "back away from nut", 1.0)
    right.go((*end[:2], HOVER), OPEN, "clear nut")
    left.wait_until(right.duration)
    left.go(raised, OPEN, "release locked pillar", 0.8)
    left.go((*raised[:2], HOVER), OPEN, "clear pillar")
    right.wait_until(left.duration)
    return {"left": left, "right": right}


def demo():
    path = tracks()
    return waypoints(path["left"], path["right"])


if __name__ == "__main__":
    import sys

    trial = Trial()
    trial.step(0.0, 0.0, False, False, True, 2)
    assert trial.value == 0
    trial.step(TRAVEL, 0, True, False, False, 0.05)
    assert trial.value == 0.35
    trial.step(TRAVEL, 0.4, False, True, False, 0.05)
    assert trial.value == 0.35
    trial.step(TRAVEL, 0.4, True, True, False, 0.05)
    assert trial.value == 0.65
    trial.step(TRAVEL, 0.4, False, False, True, 1)
    assert trial.value == 1.0
    proof(tracks(), MAX_STEPS, sys.argv)
else:
    OBJECTS = setup()
