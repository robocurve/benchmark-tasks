"""Cold rear-facing DGX access prototype with a simulated seat witness and lock indicator."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from plan import OPEN, TABLE, TIP, Track, check_tracks, steered

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
spec = json.loads(Path(__file__).with_name("task.json").read_text())
INSTRUCTION = spec["instruction"]
MAX_STEPS = int(spec["run_limit"].split()[0].replace(",", ""))

FACE_Z = TABLE + 0.095
FACE = np.array([0.235, 0.045, FACE_Z])
# Connector-local +Z points out of the network rack, toward the operator.
FRAME = np.array(((1., 0., 0.), (0., 0., -1.), (0., 1., 0.)))
CASE = os.environ.get("FAILSAFE_CASE", "demo")
CONTROL = CASE in {"partial-turned", "unlocked", "backout"}
NOSE = 0.018
NOSE_WIDTH = 0.028
NOSE_HEIGHT = 0.020
START = tuple(FACE + FRAME @ np.array((0., 0., .050)))
CRADLE = (FACE[0], START[1] - .037, TABLE)
RING_RADIUS = 0.0185
RING_DAMPING = 0.006
SEATED_FRICTION_N = 3.0
LOCK_ANGLE = math.radians(36)
# Prototype switch tolerance about the actual authored detent.
LOCK_WINDOW = math.radians(1.5)
UNLOCK_WINDOW = math.radians(1.5)
SEAT_STOP = -0.0006  # Authored socket floor minus nose depth.
WITNESS_TOLERANCE = 0.0015  # Simulated witness alignment resolution.


def lock_indicator(angle):
    return abs(angle - LOCK_ANGLE) <= LOCK_WINDOW


def open_indicator(angle):
    return abs(angle) <= UNLOCK_WINDOW
NEAR_XY = 0.006
YAW_OK = math.radians(9)
STILL_SEC = 1.0
CONTROL_GAP = .007 if CASE == "partial-turned" else SEAT_STOP
CONTROL_START = tuple(FACE + FRAME @ (0., 0., CONTROL_GAP))


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
        return self.keyed and abs(self.gap - SEAT_STOP) <= WITNESS_TOLERANCE


@dataclass
class Trial:
    entered: bool = False
    seated_unlocked: bool = False
    lock_confirmed: bool = False
    invalid: bool = False
    hold: float = 0.0
    hold_reference: Reading | None = None

    def step(self, read, dt):
        insertion = read.keyed and -0.004 <= read.gap <= NOSE - 0.003
        locked = lock_indicator(read.ring)
        self.invalid |= math.hypot(read.dx, read.dy) <= NEAR_XY and read.gap < -0.004
        # Two neighboring plug envelopes start at +/- 79 mm from the target.
        # Test intersection against the target back shell and rear cabling zone.
        neighbor_overlap = abs(abs(read.dx) - .079) < .034
        self.invalid |= neighbor_overlap and abs(read.dy) < .025 and -.019 < read.gap < .070
        self.invalid |= insertion and not self.entered and not open_indicator(read.ring)
        self.invalid |= self.lock_confirmed and not read.seated
        self.invalid |= insertion and not read.seated and (
            locked or (self.seated_unlocked and not open_indicator(read.ring))
        )
        self.entered = insertion
        if not insertion:
            self.seated_unlocked = False
        elif read.seated and open_indicator(read.ring):
            self.seated_unlocked = True
        self.lock_confirmed = read.seated and self.seated_unlocked and locked
        passed = self.lock_confirmed and read.free and not self.invalid
        if not passed:
            self.hold = 0.0
            self.hold_reference = None
            return
        anchor = self.hold_reference or read
        drift = math.dist((read.dx, read.dy, read.gap), (anchor.dx, anchor.dy, anchor.gap))
        turn = max(abs(math.remainder(read.yaw - anchor.yaw, math.tau)),
                   abs(read.tilt - anchor.tilt), abs(read.ring - anchor.ring))
        # Stability is witnessed over the full interval with pose measurements,
        # avoiding high-frequency contact-solver noise in instantaneous velocity.
        if drift > .0008 or turn > math.radians(2):
            self.hold_reference = read
            self.hold = 0.0
            return
        self.hold_reference = anchor
        self.hold += dt

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
_pull_time = 0.0
_elapsed = 0.0
_trim = np.zeros(3)
_ring_touched = False


def telemetry():
    return {
        "mouth_error_mm": np.round(np.array((_last.dx, _last.dy)) * 1000, 2).tolist(),
        "gap_mm": round(_last.gap * 1000, 2),
        "tilt_deg": round(math.degrees(_last.tilt), 2),
        "yaw_deg": round(math.degrees(_last.yaw), 2),
        "ring_deg": round(math.degrees(_last.ring), 2),
        "visible_lock": lock_indicator(_last.ring),
        "ring_touched": _ring_touched,
        "seat_witness": _last.seated,
        "free": _last.free,
        "invalid": _trial.invalid,
        "hold_s": round(_trial.hold, 2),
        "backout_load_active": _pull_time > .2,
    }


def steer_seating(label, point, rotation, finger):
    if label in {"align with the rear inlet", "seat and hold", "release backshell"}:
        if not _last.free and abs(_last.dx) < .04 and abs(_last.dy) < .04:
            correction = np.array((_last.dx, 0.0, _last.dy))
            _trim[:] = np.clip(_trim - np.clip(correction * .08, -.0004, .0004), -.008, .008)
        point = point + _trim
    return point, rotation, finger


def read_plug(plug, *, free):
    rot = FRAME.T @ plug.rotation() @ FRAME
    x, y, z = FRAME.T @ (plug.pos - FACE)
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
        write = self.asset.write_data_to_sim

        def write_with_detent():
            self.update()
            write()

        self.asset.write_data_to_sim = write_with_detent

    def update(self):
        import torch

        angle = tensor(self.asset.data.joint_pos)[:, self.ring]
        target = torch.where(
            (angle > LOCK_ANGLE / 2) & _trial.seated_unlocked, LOCK_ANGLE, 0.0
        )
        if CASE == "partial-turned" or (CASE == "backout" and _elapsed >= .6):
            target = torch.full_like(angle, LOCK_ANGLE)
        speed = tensor(self.asset.data.joint_vel)[:, self.ring]
        effort = (.080 * (target - angle) - .002 * speed).clamp(-.025, .025)
        self.asset.set_joint_effort_target(effort[:, None], joint_ids=[self.ring])
        base = self.environment.unwrapped
        pos = tensor(self.asset.data.body_pos_w)[:, self.body] - tensor(
            base.scene.env_origins
        )
        face = torch.as_tensor(FACE, dtype=pos.dtype, device=pos.device)
        frame = torch.as_tensor(FRAME, dtype=pos.dtype, device=pos.device)
        delta = (pos - face) @ frame
        lateral = torch.linalg.vector_norm(delta[:, :2], dim=1)
        inside = (lateral < 0.004) & (delta[:, 2] > -0.003) & (delta[:, 2] < 0.004)
        # The socket contacts resist sliding after mating. The prototype lock
        # biases the plug toward the physical stop only while the witness is home.
        velocity = tensor(self.asset.data.body_vel_w)[:, self.body, :3] @ frame
        axial = (240 * velocity[:, 2]).clamp(-SEATED_FRICTION_N, SEATED_FRICTION_N)
        locked = ((angle - LOCK_ANGLE).abs() <= LOCK_WINDOW) & _trial.seated_unlocked
        force = torch.zeros_like(pos)
        force[:, 2] = -torch.where(inside, axial + locked * 2.5, 0.0)
        if CASE == "backout" and _pull_time > .2:
            # A bounded fixture pull moves the witness about 9 mm rearward.
            force[:, 2] += (500 * (.009 - delta[:, 2]) - 16 * velocity[:, 2]).clamp(-4., 5.)
        force = force @ frame.T
        self.asset.permanent_wrench_composer.set_forces_and_torques_index(
            forces=force[:, None, :], body_ids=[self.body], is_global=True
        )


def tensor(value):
    return getattr(value, "torch", value)


def _register_runtime():
    from scenekit import UsdBody, on_reset, on_step
    import touch

    bay = UsdBody(
        "bay",
        usd="assets/dgx_psu_bay.usda",
        pos=tuple(FACE - FRAME @ np.array((0., 0., .036))),
        size=(0.222, 0.154, 0.09),
        mass=None,
        friction=(0.55, 0.4),
    )
    cradle = UsdBody(
        "cradle",
        usd="assets/dgx_plug_cradle.usda",
        pos=CRADLE,
        size=(0.046, 0.070, 0.080),
        mass=None,
        friction=(0.55, 0.4),
    )
    cord = UsdBody(
        "cord",
        usd="assets/dgx_locking_cord.usda",
        pos=CONTROL_START if CONTROL else START,
        size=(0.28, 0.044, 0.092),
        articulation=True,
        joint_pos=(("lock_ring", LOCK_ANGLE if CASE == "partial-turned" else 0.0),),
        solver_iters=32,
        joint_stiffness=0.0,
        joint_damping=RING_DAMPING,
        joint_effort_limit=0.025,
        friction=(0.86, 0.68),
    )

    @on_reset
    def reset(environment):
        global _trial, _last, _drive, _pull_time, _elapsed
        _trial = Trial()
        _last = Reading(1, 1, 1, 0, 0, 0, True, True)
        _trim[:] = 0.0
        _pull_time = 0.0
        _elapsed = 0.0
        touch.reset()
        if _drive is None or _drive.asset is not environment.unwrapped.scene["cord"]:
            _drive = RingPhysics(environment)

    @on_step
    def monitor(environment):
        global _last, _pull_time, _elapsed, _ring_touched
        from yam_arena.scene import object_states

        names = ["cord", "cord/ring"]
        contacts = touch.touching(environment, names)
        _ring_touched = contacts["cord/ring"]
        plug = object_states(environment, ["cord"])["cord"]
        _last = read_plug(plug, free=not any(contacts.values()))
        dt = float(environment.unwrapped.step_dt)
        _elapsed += dt
        _trial.step(_last, dt)
        if CASE == "backout" and (
            _pull_time > 0 or (_trial.lock_confirmed and _last.free)
        ):
            _pull_time += dt

    return [bay, cradle, cord]


def tracks():
    left, right = Track(), Track()
    x, y, z = FACE
    hover = TABLE + .164
    if CONTROL:
        # Preset physical scorer controls. Both arms remain clear of the inlet.
        left.go((.22, .145, hover), OPEN, "observe deliberate failure", 4.0)
        right.go((.22, -.145, hover), OPEN, "observe deliberate failure", 4.0)
        return {"left": left, "right": right}
    grip_z = z - .006 + TIP
    start = (x, START[1] - .066, grip_z)
    gap = -.017
    home = (x, y - .049 - gap, grip_z)
    right.go((*start[:2], hover), 0., "above cord")
    right.go(start, 0., "approach backshell from rear")
    right.go((x, home[1] - .021, grip_z), 0., "align with the rear inlet", 2.7)
    right.go(home, 0., "seat and hold", 3.0)
    right.go((x, home[1] - .045, grip_z), 0., "withdraw pusher straight rearward", 1.8)
    right.go((x, home[1] - .045, hover), 0., "raise after clearing the cord")
    # The off-axis pad turns the ring directly under a downward YAM action.
    lever_y = y - .062
    right.go((x - .050, home[1] - .045, hover), 0., "offset toward rotary pad")
    right.go((x - .050, lever_y, hover), 0., "above rotary pad")
    right.go((x - .050, lever_y, z + .037 + TIP), 0., "approach rotary pad")
    for theta in np.linspace(0., LOCK_ANGLE, 6):
        px = -.050 * math.cos(theta) - .025 * math.sin(theta)
        pz = -.050 * math.sin(theta) + .025 * math.cos(theta)
        right.go((x + px, lever_y, z + pz + TIP + .002), 0., "press the rotary pad", .6)
    right.go((x + px, lever_y, hover), 0., "clear collar vertically")
    right.go((.22, -.145, hover), OPEN, "park cord arm")
    left.go((.22, .145, hover), OPEN, "remain clear of rear panel")
    left.wait_until(right.duration)
    return {"left": left, "right": right}


def demo():
    script = tracks()
    return steered(script["left"], script["right"], steer_seating)


def self_test(run_ik=False):
    def sample(gap, angle=0, free=True):
        return Reading(0, 0, gap, 0, 0, angle, free, True)

    home = sample(SEAT_STOP)
    locked = sample(SEAT_STOP, LOCK_ANGLE)
    trial = Trial()
    trial.step(home, 1)
    assert trial.score(home) == .60
    trial.step(locked, 1)
    assert trial.score(locked) == 1
    backout = sample(.007, LOCK_ANGLE)
    trial.step(backout, .05)
    assert trial.score(backout) == 0
    partial = Trial()
    partial.step(sample(.007), .05)
    partial.step(backout, 1)
    assert partial.score(backout) == 0
    insertion_locked = Trial()
    insertion_locked.step(sample(.010, LOCK_ANGLE), 1.)
    assert insertion_locked.invalid
    incomplete = Trial()
    incomplete.step(home, .05)
    midway = sample(SEAT_STOP, LOCK_ANGLE / 2)
    incomplete.step(midway, 2.)
    assert incomplete.score(midway) == .60
    unreleased = Trial()
    unreleased.step(home, .05)
    unreleased.step(sample(SEAT_STOP, LOCK_ANGLE, free=False), 2.)
    unreleased.step(locked, .25)
    assert unreleased.score(locked) < 1.
    drift = Trial()
    drift.step(home, .05)
    drift.step(locked, .55)
    displaced = Reading(.001, 0., SEAT_STOP, 0., 0., LOCK_ANGLE, True, True)
    drift.step(displaced, .05)
    drift.step(displaced, .50)
    assert drift.score(displaced) == .60
    neighbor = Trial()
    neighbor.step(Reading(.079, 0., .025, 0., 0., 0., True, True), .05)
    assert neighbor.invalid
    check_tracks(tracks(), MAX_STEPS, ik=run_ik)
    print("Rear-facing seat witness, lock indicator, backout and paths passed.")


if __name__ == "__main__":
    self_test("--ik" in sys.argv)
else:
    OBJECTS = _register_runtime()
