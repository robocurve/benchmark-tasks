"""Top-down waypoint helpers for the lab's yam_scripted policy."""
from dataclasses import dataclass
from math import ceil

TABLE = 0.7475
TIP = 0.013
OPEN = -0.04695


@dataclass(frozen=True)
class Step:
    duration: float
    pos: tuple
    finger: float
    label: str
    yaw: float = 0.0


class Track(list):
    def go(self, xyz, finger, label, seconds=1.4, yaw=0.0):
        self.append(Step(seconds, tuple(xyz), finger, label, yaw))

    @property
    def duration(self):
        return sum(s.duration for s in self)

    def wait_until(self, time):
        remaining = time - self.duration
        if remaining > 1e-6:
            step = self[-1]
            self.go(step.pos, step.finger, "hold", remaining, step.yaw)


def waypoints(left, right):
    from yam_arena.demo import Waypoint

    return {name: [Waypoint(s.duration, s.pos, s.finger, s.label, s.yaw) for s in track]
            for name, track in (("left", left), ("right", right))}


def check_tracks(tracks, max_steps, *, ik=False):
    """Pure layout and optional CPU IK check using the installed Arena kinematics."""
    for name, track in tracks.items():
        assert all(s.duration > 0 and OPEN <= s.finger <= 0 for s in track), name
        assert all(s.pos[0] > 0 and TABLE < s.pos[2] <= TABLE + .190 for s in track), name
        assert ceil(sum(s.duration for s in track) * 20) + 60 <= max_steps, name
    if ik:
        from yam_arena.demo import ArmPlan, Waypoint
        from yam_arena.constants import LEFT_BASE_POS, RIGHT_BASE_POS

        for name, base in (("left", LEFT_BASE_POS), ("right", RIGHT_BASE_POS)):
            track = tracks[name]
            calibrated = (base[0], .305 if name == "left" else -.305, base[2])
            arm = ArmPlan([Waypoint(s.duration, s.pos, s.finger, s.label, s.yaw) for s in track],
                          calibrated)
            for step in range(ceil(arm.duration * 20) + 1):
                arm.action7(step / 20)
            print(f"{name}: {arm.duration:.1f}s, max IK {arm.worst_ik_mm:.2f} mm / "
                  f"{arm.worst_ik_deg:.2f} deg")
            assert arm.worst_ik_mm < 5 and arm.worst_ik_deg < 8, name


class SteeredLabel(str):
    """A waypoint phase with bounded feedback from the scripted demo only."""

    def __new__(cls, text, steer):
        label = super().__new__(cls, text)
        label.steer = steer
        return label


def steered(left, right, steer_left, steer_right):
    from dataclasses import replace
    from yam_arena.demo import ArmPlan

    if not getattr(ArmPlan.target, "_failsafe_steered", False):
        original = ArmPlan.target

        def target(self, t):
            point, rotation, finger, label = original(self, t)
            if isinstance(label, SteeredLabel):
                point, rotation, finger = label.steer(str(label), point, rotation, finger)
            return point, rotation, finger, label

        target._failsafe_steered = True
        ArmPlan.target = target

    paths = waypoints(left, right)
    for side, steer in (("left", steer_left), ("right", steer_right)):
        paths[side] = [replace(w, label=SteeredLabel(w.label, steer)) for w in paths[side]]
    return paths
