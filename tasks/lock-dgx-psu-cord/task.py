"""The face-up PSU bench fixture has one loose black connector in a shallow cradle. Its gray locking collar rotates independently of the plug body."""
from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from plan import OPEN, TABLE, TIP, Track, check_tracks, steered

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
MAX_STEPS = 1200
INSTRUCTION = (
    "On the de-energized DGX power-supply fixture, seat the loose system-end cord with its gray ring "
    "unlocked, then turn the ring to locked. Release both hands."
)

FACE_Z = TABLE + 0.036
FACE = np.array([0.300, 0.010, FACE_Z])
NOSE = 0.018
NOSE_WIDTH = 0.028
NOSE_HEIGHT = 0.020
CRADLE = (0.222, -0.168, TABLE)
CRADLE_FLOOR = 0.005
CRADLE_HEIGHT = 0.020
CRADLE_CLEARANCE = 0.001
START = (*CRADLE[:2], TABLE + CRADLE_FLOOR + NOSE + 0.0003)
RING_RADIUS = 0.0185
CABLE_LINKS = 5
CABLE_DENSITY = 0.23  # kg/m
CABLE_RADIUS = 0.005
CABLE_STIFFNESS = 0.027
CABLE_DAMPING = 0.003
RING_DAMPING = 0.0014
SEATED_FRICTION_N = 3.0
LOCK_ANGLE = math.radians(36)
LOCK_AT = math.radians(31)
UNLOCK_AT = math.radians(8)
SEAT_GAP = 0.0025
NEAR_XY = 0.006
YAW_OK = math.radians(9)
STILL_SEC = 1.0


@dataclass(frozen=True)
class Reading:
    dx: float
    dy: float
    gap: float
    yaw: float
    tilt: float
    ring: float
    free: bool
    still: bool

    @property
    def keyed(self):
        return (
            abs(self.dx) <= 0.0035
            and abs(self.dy) <= 0.0035
            and abs(self.yaw) <= YAW_OK
            and self.tilt <= YAW_OK
        )

    @property
    def seated(self):
        return self.keyed and -0.002 <= self.gap <= SEAT_GAP


@dataclass
class Trial:
    entered: bool = False
    seated_unlocked: bool = False
    lock_confirmed: bool = False
    invalid: bool = False
    hold: float = 0.0

    def step(self, read, dt):
        near = math.hypot(read.dx, read.dy) <= NEAR_XY
        insertion = read.keyed and -0.004 <= read.gap <= NOSE - 0.003
        if near and read.gap < -0.004:
            self.invalid = True
        if insertion and not self.entered and read.ring > UNLOCK_AT:
            self.invalid = True
        self.entered = insertion
        if read.seated and read.ring <= UNLOCK_AT:
            self.seated_unlocked = True
        if read.seated and self.seated_unlocked and read.ring >= LOCK_AT:
            self.lock_confirmed = True
        if self.lock_confirmed and read.ring >= LOCK_AT and not read.seated:
            if read.gap > 0.006 or not near:
                self.invalid = True
        if not self.entered:
            self.seated_unlocked = False
        if read.ring < LOCK_AT:
            self.lock_confirmed = False
        passed = (
            self.lock_confirmed
            and read.seated
            and read.free
            and read.still
            and not self.invalid
        )
        self.hold = self.hold + dt if passed else 0.0

    def score(self, read):
        if self.invalid:
            return 0.0
        if self.hold >= STILL_SEC - 1e-8:
            return 1.0
        if read.seated and self.seated_unlocked:
            return 0.60
        if self.entered and read.keyed and read.gap <= NOSE + 0.002:
            return 0.25
        return 0.0


_trial = Trial()
_last = Reading(1, 1, 1, 0, 0, 0, True, True)
_drive = None
_trim = np.zeros(3)
_yaw_trim = 0.0
_tilt_trim = np.zeros(2)
_plug_up = np.array((0.0, 0.0, 1.0))
_handoff_ready = False


def telemetry():
    return {
        "mouth_error_mm": np.round(np.array((_last.dx, _last.dy)) * 1000, 1).tolist(),
        "gap_mm": round(_last.gap * 1000, 1),
        "insertion_depth_mm": round((NOSE - _last.gap) * 1000, 1),
        "tilt_deg": round(math.degrees(_last.tilt), 1),
        "yaw_deg": round(math.degrees(_last.yaw), 1),
        "ring_deg": round(math.degrees(_last.ring), 1),
        "lead_mps": round(cable_speed(), 3),
        "seated": _last.seated,
        "ready_to_release": _handoff_ready,
        "guided_xy_mm": np.round(_trim[:2] * 1000, 1).tolist(),
        "free": _last.free,
        "invalid": _trial.invalid,
        "hold_s": round(_trial.hold, 2),
    }


def steer_seating(label, point, rotation, finger):
    global _yaw_trim, _handoff_ready
    if _drive is None:
        return point, rotation, finger
    if label in (
        "find keyed lead-in",
        "press connector squarely home",
        "check fully seated",
    ):
        # Correct measured plug pose rather than assuming it stayed centered in the jaws.
        if abs(_last.dx) < 0.040 and abs(_last.dy) < 0.040 and not _last.free:
            error = np.array((_last.dx, _last.dy))
            _trim[:2] = np.clip(
                _trim[:2] - np.clip(error * 0.16, -0.0010, 0.0010), -0.024, 0.024
            )
            _yaw_trim = float(
                np.clip(
                    _yaw_trim - np.clip(0.10 * _last.yaw, -0.024, 0.024), -0.26, 0.26
                )
            )
            up_x, up_y, _ = _plug_up
            _tilt_trim[:] = np.clip(
                _tilt_trim + 0.025 * np.array((up_y, -up_x)), -0.12, 0.12
            )
            if label == "check fully seated" or (
                label == "press connector squarely home" and abs(_last.gap) < 0.012
            ):
                _trim[2] = np.clip(
                    _trim[2] - np.clip((_last.gap - 0.0001) * 0.10, -0.0006, 0.0006),
                    -0.012,
                    0.006,
                )
            _handoff_ready = _last.seated and _last.ring <= UNLOCK_AT
    elif label == "open jaws clear of backshell":
        # A missed insertion never causes the carrying gripper to drop the cord.
        if not _handoff_ready:
            finger = -0.008
    elif label in (
        "raise open pads vertically",
        "back off toward cable-free side",
        "park clear of ring arm",
    ):
        if not _handoff_ready:
            finger = -0.008

    seating = {
        "find keyed lead-in",
        "press connector squarely home",
        "check fully seated",
        "open jaws clear of backshell",
        "raise open pads vertically",
    }
    if label in seating:
        c, s = math.cos(_yaw_trim), math.sin(_yaw_trim)
        rotation = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1))) @ rotation
        tilt = np.linalg.norm(_tilt_trim)
        if tilt > 1e-6:
            from yam_arena.urdf import axis_angle_matrix

            rotation = axis_angle_matrix(np.array((*_tilt_trim, 0.0)) / tilt, tilt) @ rotation
        point = np.asarray(point) + _trim
    return point, rotation, finger


def read_plug(plug, *, free):
    rot = plug.rotation()
    x, y, z = plug.pos - FACE
    return Reading(
        float(x),
        float(y),
        float(z),
        math.atan2(rot[1, 0], rot[0, 0]),
        math.acos(float(np.clip(rot[2, 2], -1, 1))),
        plug.joints["lock_ring"],
        free,
        np.linalg.norm(plug.lin_vel) < 0.012 and np.linalg.norm(plug.ang_vel) < 0.12,
    )


def score(objects):
    return _trial.score(_last)


def success(objects):
    return score(objects) == 1.0


def outcome(objects):
    if _trial.invalid:
        return "locked_on_entry_or_forced"
    return {
        0.0: "unmated",
        0.25: "entered",
        0.60: "seated",
        1.0: "locked_and_released",
    }[score(objects)]


class RingPhysics:
    """Approximate the ring detent and seat retention using forces on simulated bodies."""

    def __init__(self, environment):
        self.environment = environment
        self.asset = environment.unwrapped.scene["cord"]
        self.ring = self.asset.joint_names.index("lock_ring")
        self.body = self.asset.body_names.index("plug")
        self.leads = [
            self.asset.body_names.index(f"lead{i}") for i in range(CABLE_LINKS)
        ]
        write = self.asset.write_data_to_sim

        def write_with_detent():
            self.update()
            write()

        self.asset.write_data_to_sim = write_with_detent

    def update(self):
        import torch

        angle = tensor(self.asset.data.joint_pos)[:, self.ring]
        target = torch.where(angle > LOCK_ANGLE / 2, LOCK_ANGLE, 0.0)
        effort = torch.clamp(0.045 * (target - angle), -0.018, 0.018)
        self.asset.set_joint_effort_target(effort[:, None], joint_ids=[self.ring])
        base = self.environment.unwrapped
        pos = tensor(self.asset.data.body_pos_w)[:, self.body] - tensor(
            base.scene.env_origins
        )
        face = torch.as_tensor(FACE, dtype=pos.dtype, device=pos.device)
        delta = pos - face
        lateral = torch.linalg.vector_norm(delta[:, :2], dim=1)
        inside = (lateral < 0.004) & (delta[:, 2] > -0.003) & (delta[:, 2] < 0.004)
        # Socket spring contacts resist extraction while the nose is still mated.
        # Limit the resistance to upward motion so this does not pull in a loose plug.
        v_z = tensor(self.asset.data.body_vel_w)[:, self.body, 2]
        resisting_lift = (260 * v_z).clamp(0.0, SEATED_FRICTION_N)
        locked = (angle >= LOCK_AT) & _trial.seated_unlocked
        force = torch.zeros_like(pos)
        force[:, 2] = -torch.where(inside, resisting_lift + locked * 2.5, 0.0)
        self.asset.permanent_wrench_composer.set_forces_and_torques_index(
            forces=force[:, None, :], body_ids=[self.body], is_global=True
        )


def tensor(value):
    return getattr(value, "torch", value)


def cable_speed():
    if _drive is None:
        return 0.0
    speed = tensor(_drive.asset.data.body_vel_w)[0, _drive.leads, :3]
    return float(speed.norm(dim=-1).max().item())


def _register_runtime():
    from scenekit import UsdBody, on_reset, on_step
    import touch

    bay = UsdBody(
        "bay",
        usd="assets/dgx_psu_bay.usda",
        pos=(FACE[0], FACE[1], TABLE),
        size=(0.222, 0.154, 0.09),
        mass=None,
        friction=(0.55, 0.4),
    )
    cradle = UsdBody(
        "cradle",
        usd="assets/dgx_plug_cradle.usda",
        pos=CRADLE,
        size=(0.052, 0.044, CRADLE_HEIGHT),
        mass=None,
        friction=(0.55, 0.4),
    )
    cord = UsdBody(
        "cord",
        usd="assets/dgx_locking_cord.usda",
        pos=START,
        size=(0.28, 0.044, 0.092),
        articulation=True,
        joint_pos=(
            ("lock_ring", 0.0),
            *((f"bend{i}", 0.0) for i in range(CABLE_LINKS)),
        ),
        solver_iters=32,
        joint_stiffness={"lock_ring": 0.0, "bend.*": CABLE_STIFFNESS},
        joint_damping={"lock_ring": RING_DAMPING, "bend.*": CABLE_DAMPING},
        joint_effort_limit=0.025,
        friction=(0.86, 0.68),
    )

    @on_reset
    def reset(environment):
        global _trial, _last, _drive, _yaw_trim, _handoff_ready
        _trial = Trial()
        _last = Reading(1, 1, 1, 0, 0, 0, True, True)
        _trim[:] = 0.0
        _yaw_trim = 0.0
        _tilt_trim[:] = 0.0
        _plug_up[:] = (0.0, 0.0, 1.0)
        _handoff_ready = False
        touch.reset()
        if _drive is None or _drive.asset is not environment.unwrapped.scene["cord"]:
            _drive = RingPhysics(environment)

    @on_step
    def monitor(environment):
        global _last
        from yam_arena.scene import object_states

        names = ["cord", "cord/ring", *(f"cord/lead{i}" for i in range(CABLE_LINKS))]
        contacts = touch.touching(environment, names)
        plug = object_states(environment, ["cord"])["cord"]
        _plug_up[:] = plug.rotation()[:, 2]
        _last = read_plug(plug, free=not any(contacts.values()))
        _trial.step(_last, float(environment.unwrapped.step_dt))

    return [bay, cradle, cord]


def tracks():
    left, right = Track(), Track()
    grip = -0.008
    ring_grip = -0.016
    plug_offset = 0.029 + TIP
    # Contact the upper rim so the fingertips pass above the retainer cleats.
    ring_offset = 0.0155 + TIP
    carry_z = TABLE + 0.125
    right.go((START[0], START[1], carry_z), OPEN, "above plug")
    right.go((START[0], START[1], START[2] + plug_offset), OPEN, "grip approach")
    right.go(right[-1].pos, grip, "grip molded plug body", 0.8)
    right.go(
        (START[0], START[1], START[2] + plug_offset + 0.038),
        grip,
        "lift straight out of cradle",
        2.2,
    )
    right.go((START[0], START[1], carry_z), grip, "raise cord clear of bench", 1.4)
    right.go((FACE[0], FACE[1], carry_z), grip, "align keyed inlet", 3.3)
    right.go(
        (FACE[0], FACE[1], FACE_Z + plug_offset + 0.014),
        grip,
        "find keyed lead-in",
        2.5,
    )
    right.go(
        (FACE[0], FACE[1], FACE_Z + plug_offset - 0.008),
        grip,
        "press connector squarely home",
        2.2,
    )
    right.go(right[-1].pos, grip, "check fully seated", 2.0)
    right.go(right[-1].pos, OPEN, "open jaws clear of backshell", 1.05)
    lifted = (FACE[0], FACE[1], FACE_Z + plug_offset + 0.025)
    right.go(lifted, OPEN, "raise open pads vertically", 1.65)
    away = (FACE[0] + 0.032, FACE[1], carry_z)
    right.go(away, OPEN, "back off toward cable-free side", 1.55)
    right.go((0.26, -0.21, carry_z), OPEN, "park clear of ring arm", 1.8)
    left.go((0.23, 0.17, carry_z), OPEN, "wait clear")
    left.wait_until(right.duration)
    left.go((FACE[0], FACE[1], carry_z), OPEN, "above locking ring", 2.0)
    left.go(
        (FACE[0], FACE[1], FACE_Z + ring_offset),
        -0.025,
        "lower to collar between cable and adjacent ports",
        1.9,
    )
    left.go(left[-1].pos, ring_grip, "grip collar", 0.9)
    left.go(
        left[-1].pos, ring_grip, "turn ring to its lock stop", 2.2, LOCK_ANGLE + 0.06
    )
    left.go(left[-1].pos, -0.026, "release the lock ring", 0.9, LOCK_ANGLE + 0.06)
    left.go((FACE[0], FACE[1], carry_z), -0.026, "lift clear", 1.5, LOCK_ANGLE + 0.06)
    left.go((0.23, 0.17, carry_z), OPEN, "retreat")
    right.wait_until(left.duration)
    return {"left": left, "right": right}


def demo():
    script = tracks()
    return steered(script["left"], script["right"], steer_seating)


def self_test(run_ik=False):
    def sample(z, angle=0, free=True, still=True, x=0, tilt=0):
        return Reading(x, 0, z, 0, tilt, angle, free, still)

    trial = Trial()
    start, enter, home, lock = (
        sample(0.03),
        sample(0.010),
        sample(0.001),
        sample(0.001, LOCK_ANGLE),
    )
    for r, expected in ((start, 0), (enter, 0.25), (home, 0.60), (lock, 0.60)):
        trial.step(r, 0.05)
        assert trial.score(r) == expected
    trial.step(sample(0.001, LOCK_ANGLE, free=False), 0.5)
    assert trial.hold == 0
    trial.step(lock, 1.0)
    assert trial.score(lock) == 1
    trial.step(sample(0.009, LOCK_ANGLE), 0.05)
    assert trial.score(lock) == 0
    early = Trial()
    early.step(sample(0.012, LOCK_ANGLE), 0.05)
    assert early.invalid
    bypass = Trial()
    bypass.step(sample(0.001, LOCK_ANGLE, x=0.020), 1)
    assert bypass.score(lock) == 0
    reinsert = Trial()
    reinsert.step(enter, 0.05)
    reinsert.step(start, 0.05)
    reinsert.step(sample(0.011, LOCK_ANGLE), 0.05)
    assert reinsert.invalid
    forced = Trial()
    forced.step(sample(-0.006), 0.05)
    assert forced.invalid
    check_tracks(tracks(), MAX_STEPS, ik=run_ik)
    print("DGX scoring, trajectories and run limit passed.")


if __name__ == "__main__":
    self_test("--ik" in sys.argv)
else:
    OBJECTS = _register_runtime()
