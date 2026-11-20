"""Loosen two captive screws and stow the removed gripper from a de-energized YAM wrist."""
from __future__ import annotations

from dataclasses import dataclass, field
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
REMOVAL_ONLY = True
spec = json.loads(Path(__file__).with_name("task.json").read_text())
INSTRUCTION = spec["instruction"]
MAX_STEPS = int(spec["run_limit"].split()[0].replace(",", ""))

MOUNT = np.array([0.248, 0.008, TABLE + 0.054])
REPLACEMENT = np.array([0.205, 0.231, TABLE + 0.056])
REMOVED = np.array([0.385, 0.188, TABLE + 0.056])
FASTENERS = ((-0.027, -0.0155, 0.036), (-0.027, 0.0155, 0.036))
DRIVERS = {
    "loosen_driver": ((0.195, -0.192, TABLE + 0.009), 1),
    "tighten_driver": ((0.290, -0.197, TABLE + 0.009), -1),
}
ACTIVE_DRIVERS = ("loosen_driver",) if REMOVAL_ONLY else tuple(DRIVERS)
MODULES = ("old_gripper",) if REMOVAL_ONLY else ("old_gripper", "new_gripper")
TOOL_GRIP = 0.058
MODULE_OFFSET = np.array((0.042, 0.0, 0.021 + TIP))
HEAD_DIAMETER = 0.0055
BIT_ACROSS_FLATS = 0.0025
GRIPPER_MASS = 0.553219 + 2 * 0.0710042
BOLT_DAMPING = 0.00005
THREAD_TURNS_TO_CLEAR = 2
REQUIRED_TURN = math.radians(800)  # Captive stop includes 80 degrees of clearance.
DRIVER_CAPACITY = 1.30
BREAKAWAY = float(os.environ.get("YAM_BREAKAWAY_NM", ".50"))


def drive_torque(angle, omega, engaged, breakaway=BREAKAWAY, capacity=DRIVER_CAPACITY):
    """Reduced hex coupling with static preload and Coulomb running resistance."""
    if not engaged or angle >= REQUIRED_TURN - RELEASE_STOP_SLOP:
        return 0.0, 0.0
    resistance = breakaway if angle < math.radians(5) else .20 * breakaway
    output = min(capacity, resistance + max(0.0, .35 * (3.6 - omega)))
    # Resist initial breakaway as well as running rotation, without reversing
    # an unloaded joint when the motor cannot overcome static friction.
    return output, max(0.0, output - resistance)
TIGHT_AT = math.radians(670)
RELEASE_STOP_SLOP = math.radians(1)
CASE = os.environ.get("FAILSAFE_CASE", "demo")
THREAD_PITCH = 0.0005
MAX_RISE = REQUIRED_TURN * THREAD_PITCH / math.tau
ENGAGE_RADIUS = 0.0010
SPINDLE_INDEXING_TORQUE = 0.004
FULL_HOLD = 1.0


@dataclass(frozen=True)
class GripperReading:
    xyz: tuple
    face_error: float
    free: bool
    still: bool
    tilt_axis: tuple = (0.0, 0.0, 0.0)

    def distance(self, target):
        return math.dist(self.xyz, target)

    def aligned_at_mount(self):
        dx, dy, dz = (a - b for a, b in zip(self.xyz, MOUNT))
        return (
            abs(dx) <= 0.003
            and abs(dy) <= 0.004
            and abs(dz) <= 0.004
            and self.face_error <= math.radians(7)
        )

    def stowed(self):
        return (
            self.distance(REMOVED) <= 0.013
            and self.face_error <= math.radians(12)
            and self.free
            and self.still
        )


@dataclass(frozen=True)
class DriverReading:
    tip: tuple
    yaw: float
    tilt: float
    held: bool
    heads: tuple = (0.0, 0.0)
    lifts: tuple = (0.0, 0.0)
    compression: float = 0.0
    up: tuple = (0.0, 0.0, 1.0)

    def socket(self):
        if not self.held or self.tilt > math.radians(9):
            return None
        for i, at in enumerate(FASTENERS):
            x, y, z = MOUNT + at
            depth = z + self.lifts[i] - self.tip[2]
            if (
                math.hypot(self.tip[0] - x, self.tip[1] - y) <= ENGAGE_RADIUS
                and 0.00075 < depth < 0.00415
            ):
                return i
        return None


def wrapped_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


@dataclass
class Trial:
    loose: list = field(default_factory=lambda: [0.0] * len(FASTENERS))
    tight: list = field(default_factory=lambda: [0.0] * len(FASTENERS))
    opened: list = field(default_factory=lambda: [False] * len(FASTENERS))
    prior: tuple | None = None
    invalid: bool = False
    separated: bool = False
    hold: float = 0.0

    @property
    def old_released(self):
        return all(self.opened)

    @property
    def secured(self):
        return sum(x >= TIGHT_AT for x in self.tight)

    def step(self, old, new, driver, dt):
        # The authored captive mechanism disengages at its mechanical end stop.
        # Driver motion alone cannot open a fastener. Require actual head motion
        # under engagement and revoke release if the joint moves off its stop.
        for i, angle in enumerate(driver.heads):
            self.opened[i] = (
                angle >= REQUIRED_TURN - RELEASE_STOP_SLOP and self.loose[i] >= THREAD_TURNS_TO_CLEAR * math.tau
            )
        pulled = old.xyz[0] - MOUNT[0]
        if not self.old_released and old.distance(MOUNT) > 0.010:
            self.invalid = True
        if not self.separated and pulled > 0.006:
            self.invalid |= old.free
            self.separated = pulled >= 0.026 and not old.free
        if min(old.xyz[2], new.xyz[2]) < TABLE - 0.02:
            self.invalid = True
        if self.secured and new.distance(MOUNT) > 0.010:
            self.invalid = True
        socket = driver.socket()
        mode = (
            "remove"
            if old.aligned_at_mount()
            else "refit"
            if self.old_released and old.stowed() and new.aligned_at_mount()
            else None
        )
        engaged = (mode, socket) if mode is not None and socket is not None else None
        if engaged is not None and self.prior is not None and engaged == self.prior[:2]:
            actual = wrapped_delta(driver.heads[socket], self.prior[3])
            move = max(0.0, actual) if mode == "remove" else min(0.0, actual)
            # Count the measured screw motion only while the bit is seated.
            # Returning below the mechanical stop also revokes the release.
            if abs(move) < math.radians(23):
                turns = self.loose if mode == "remove" else self.tight
                direction = 1 if mode == "remove" else -1
                turns[socket] = min(
                    REQUIRED_TURN, max(0.0, turns[socket] + direction * move)
                )
        self.prior = (
            (*engaged, driver.yaw, driver.heads[socket])
            if engaged is not None
            else None
        )
        if REMOVAL_ONLY:
            done = (
                self.old_released and old.stowed() and not driver.held
                and self.separated
                and not self.invalid
            )
            self.hold = self.hold + dt if done else 0.0
            return
        done = (
            self.old_released
            and old.stowed()
            and new.aligned_at_mount()
            and new.free
            and not driver.held
            and new.still
            and self.secured == len(FASTENERS)
            and max(driver.lifts) < 0.00025
            and not self.invalid
        )
        self.hold = self.hold + dt if done else 0.0

    def score(self, old, new):
        if self.invalid:
            return 0.0
        if self.hold >= FULL_HOLD - 1e-8:
            return 1.0
        if REMOVAL_ONLY:
            return 0.5 if self.old_released else 0.0
        if not (self.old_released and old.stowed()):
            return 0.0
        if not new.aligned_at_mount():
            return 0.25
        return 0.75 if self.secured else 0.50


_trial = Trial()
_old = GripperReading(tuple(MOUNT), 0, True, True)
_new = GripperReading(tuple(REPLACEMENT), 0, True, True)
_driver = DriverReading((0.0, 0.0, TABLE), 0, 0, False)
_physics = None
_bit_trim = np.zeros(3)
_tool_lean = np.zeros(2)
_dock_trim = np.zeros(3)
_dock_lean = np.zeros(2)
_placement_trim = np.zeros(3)
_placement_lean = np.zeros(3)
_socket_index = None


def steer_driver(label, point, rotation, finger):
    global _socket_index
    if _physics is None:
        return point, rotation, finger
    if label.startswith("return powered driver"):
        _dock_trim[:] = 0.0
        _dock_lean[:] = 0.0
    if label.startswith(("align driver over stand", "lower into stand", "release driver")):
        stand, _ = DRIVERS[label.rsplit(" ", 1)[1]]
        if _driver.held and not label.startswith("release driver"):
            error = np.array(stand) - _driver.tip
            _dock_trim[:2] = np.clip(
                _dock_trim[:2] + np.clip(0.11 * error[:2], -0.0009, 0.0009), -0.038, 0.038
            )
            up_x, up_y, _ = _driver.up
            _dock_lean[:] = np.clip(
                _dock_lean + 0.025 * np.array((up_y, -up_x)), -0.17, 0.17
            )
        return point + _dock_trim, rotate_wrist(rotation, (*_dock_lean, 0.0)), finger
    if label.startswith("above collar screw"):
        index = int(label.rsplit(" ", 1)[1]) - 1
        if index != _socket_index:
            _socket_index = index
            _bit_trim[:] = 0.0
            _tool_lean[:] = 0.0
    contact_phase = label.startswith(
        ("center quill", "feed bit", "load spring", "hold driver")
    )
    retract_phase = label.startswith(("unload the quill", "withdraw bit"))
    if _socket_index is not None and (contact_phase or retract_phase):
        nearby = MOUNT + FASTENERS[_socket_index]
        error = nearby[:2] - np.array(_driver.tip[:2])
        if contact_phase and _driver.held and np.linalg.norm(error) < 0.035:
            correction = np.where(abs(error) > 0.00016, error, 0.0)
            _bit_trim[:2] = np.clip(
                _bit_trim[:2] + np.clip(0.11 * correction, -0.00065, 0.00065), -0.030, 0.030
            )
            up_x, up_y, _ = _driver.up
            _tool_lean[:] = np.clip(
                _tool_lean + 0.025 * np.array((up_y, -up_x)), -0.15, 0.15
            )
            if label.startswith(("load spring", "hold driver")) and np.linalg.norm(error) < 0.002:
                preload_error = _driver.compression - 0.0007
                _bit_trim[2] = np.clip(
                    _bit_trim[2] + np.clip(0.18 * preload_error, -0.00035, 0.00065),
                    -0.002, 0.012,
                )
        point = np.asarray(point) + _bit_trim
        rotation = rotate_wrist(rotation, (*_tool_lean, 0.0))
    return point, rotation, finger


def rotate_wrist(rotation, correction):
    from yam_arena.urdf import axis_angle_matrix

    angle = np.linalg.norm(correction)
    if angle > 1e-6:
        return axis_angle_matrix(np.asarray(correction) / angle, angle) @ rotation
    return rotation


def steer_module(label, point, rotation, finger):
    if label not in {
        "align the replacement flange", "slide flange home into saddle",
        "settle flange in the saddle", "release module into saddle",
    }:
        return point, rotation, finger
    if not _new.free and label != "release module into saddle":
        error = point - MODULE_OFFSET - _new.xyz
        if np.linalg.norm(error) < 0.035:
            _placement_trim[:] = np.clip(
                _placement_trim + np.clip(0.10 * error, -0.0006, 0.0006), -0.020, 0.020
            )
            _placement_lean[:] = np.clip(
                _placement_lean - 0.025 * np.asarray(_new.tilt_axis), -0.15, 0.15
            )
    return point + _placement_trim, rotate_wrist(rotation, _placement_lean), finger


def read_gripper(body, free):
    rotation = body.rotation()
    angle = math.acos(float(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))
    return GripperReading(
        tuple(body.pos),
        angle,
        free,
        np.linalg.norm(body.lin_vel) < 0.012 and np.linalg.norm(body.ang_vel) < 0.12,
        tuple(0.5 * np.array((rotation[2, 1] - rotation[1, 2],
                              rotation[0, 2] - rotation[2, 0],
                              rotation[1, 0] - rotation[0, 1]))),
    )


def tensor(value):
    return getattr(value, "torch", value)


def score(objects):
    return _trial.score(_old, _new)


def success(objects):
    return score(objects) == 1.0


def outcome(objects):
    if _trial.invalid:
        return "forced_or_dropped"
    if REMOVAL_ONLY:
        return {0.0: "fasteners_engaged", 0.5: "unfastened", 1.0: "removed_and_released"}[score(objects)]
    return {
        0.0: "incomplete_removal",
        0.25: "old_stowed",
        0.50: "new_aligned",
        0.75: "new_fastened_not_released",
        1.0: "replaced_and_released",
    }[score(objects)]


def telemetry():
    driver = _driver
    centers = MOUNT + np.array(FASTENERS)
    i = int(np.argmin(np.linalg.norm(centers[:, :2] - driver.tip[:2], axis=1)))
    diff = np.array(driver.tip) - centers[i]
    diff[2] -= driver.lifts[i]
    tools = {}
    spindle_rates = {}
    spindle_commands = {}
    if _physics is not None:
        for name, (unit, _) in _physics.drivers.items():
            vel = tensor(unit.data.root_vel_w)[0, :3]
            tools[name] = round(float(vel.norm().item()), 3)
            spin = unit.joint_names.index("spin")
            spindle_rates[name] = round(float(tensor(unit.data.joint_vel)[0, spin]), 2)
            command = _physics.motor_commands.get(name)
            if command is not None:
                spindle_commands[name] = round(float(command[0, 0]) * 1000, 1)
    return {
        "held": driver.held,
        "near": i + 1,
        "bit_mm": np.round(diff * 1000, 2).tolist(),
        "quill_mm": round(driver.compression * 1000, 2),
        "driver_tilt_deg": round(math.degrees(driver.tilt), 1),
        "socket": driver.socket(),
        "hex_transmitted_nm": _physics.transmitted_nm if _physics else 0.,
        "hex_resisted_nm": _physics.resisted_nm if _physics else 0.,
        "fasteners_released": _trial.opened,
        "supported_separation": _trial.separated,
        "module_contact": not _old.free,
        "heads_deg": np.round(np.rad2deg(driver.heads), 1).tolist(),
        "loosened_deg": np.round(np.rad2deg(_trial.loose), 1).tolist(),
        "tightened_deg": np.round(np.rad2deg(_trial.tight), 1).tolist(),
        "replacement_error_mm": np.round((np.array(_new.xyz) - MOUNT) * 1000, 1).tolist(),
        "replacement_angle_deg": round(math.degrees(_new.face_error), 1),
        "rise_mm": np.round(np.array(driver.lifts) * 1000, 2).tolist(),
        "driver_speed_mps": tools,
        "spindle_rad_s": spindle_rates,
        "spindle_command_mnm": spindle_commands,
        "steer_mm": np.round(_bit_trim * 1000, 2).tolist(),
        "invalid": _trial.invalid,
        "hold_s": round(_trial.hold, 2),
    }


class WristClamp:
    """Fastener retention by compliance at the real mating state, never a pose write on either module."""

    def __init__(self, environment):
        self.environment = environment
        scene = environment.unwrapped.scene
        self.modules = [scene[n] for n in MODULES]
        self.fixture = scene["wrist"]
        self.bolts = [
            self.fixture.joint_names.index(f"bolt_{i}") for i in range(len(FASTENERS))
        ]
        self.lifts = [
            self.fixture.joint_names.index(f"rise_{i}") for i in range(len(FASTENERS))
        ]
        self.fixture.write_joint_velocity_limit_to_sim_index(limits=5.0, joint_ids=self.bolts)
        bolts = tensor(self.fixture.data.joint_pos)[:, self.bolts]
        self.engaged_socket = None
        self.transmitted_nm = 0.0
        self.resisted_nm = 0.0
        self.drivers = {}
        self.feed_joints = {}
        self.motor_commands = {}
        self.held = dict.fromkeys(ACTIVE_DRIVERS, False)
        for index, module in enumerate(self.modules):
            write = module.write_data_to_sim

            def write_with_clamp(index=index, write=write):
                self.update_module(index)
                write()

            module.write_data_to_sim = write_with_clamp
        fixture_write = self.fixture.write_data_to_sim

        def write_threads():
            angles = tensor(self.fixture.data.joint_pos)[:, self.bolts]
            goal = (angles * (THREAD_PITCH / math.tau)).clamp(0.0, MAX_RISE)
            height = tensor(self.fixture.data.joint_pos)[:, self.lifts]
            speed = tensor(self.fixture.data.joint_vel)[:, self.lifts]
            # The screw and slider weigh 12.5 g together. The axial force models
            # the thread advancing by one pitch per actual turn of the head.
            force = (1200.0 * (goal - height) - 1.4 * speed + 0.0125 * 9.81).clamp(-1.6, 1.6)
            self.fixture.set_joint_effort_target(force, joint_ids=self.lifts)
            bolt_load = angles.new_zeros(angles.shape)
            index = self.engaged_socket
            self.transmitted_nm = self.resisted_nm = 0.
            if index is not None:
                # Apply the net torque to the actual head joint. This explicit
                # coupling is used in place of resolving high hex contact stress
                # at rigid-mesh corners.
                omega = tensor(self.fixture.data.joint_vel)[0, self.bolts[index]]
                output, net = drive_torque(float(angles[0, index]), float(omega), True)
                self.transmitted_nm, self.resisted_nm = output, output - net
                bolt_load[0, index] = net
            self.fixture.set_joint_effort_target(bolt_load, joint_ids=self.bolts)
            fixture_write()

        self.fixture.write_data_to_sim = write_threads
        for name in ACTIVE_DRIVERS:
            _, direction = DRIVERS[name]
            unit = scene[name]
            self.drivers[name] = unit, unit.body_names.index("spindle")
            spin = unit.joint_names.index("spin")
            unit.write_joint_velocity_limit_to_sim_index(limits=5.0, joint_ids=[spin])
            self.feed_joints[name] = unit.joint_names.index("feed")
            tool_write = unit.write_data_to_sim

            def write_motor(
                name=name, spin=spin, direction=direction, tool_write=tool_write
            ):
                self.update_motor(name, spin, direction)
                tool_write()

            unit.write_data_to_sim = write_motor

    def update_module(self, index):
        import torch

        module = self.modules[index]
        is_clamped = not _trial.old_released if index == 0 else _trial.secured > 0
        root = tensor(module.data.root_link_pose_w)
        velocity = tensor(module.data.root_vel_w)
        pos = root[:, :3] - tensor(self.environment.unwrapped.scene.env_origins)
        zero = torch.zeros_like(pos)
        force, torque = zero, zero
        if is_clamped:
            goal = torch.as_tensor(MOUNT, dtype=pos.dtype, device=pos.device)
            error = goal - pos
            force = torch.clamp(3200 * error - 32 * velocity[:, :3], -38, 38)
            force[:, 2] += GRIPPER_MASS * 9.81
            quat = root[:, 3:7]  # XYZW in the lab's Isaac Lab build.
            axis = -2 * quat[:, :3] * torch.where(quat[:, 3:] < 0, -1.0, 1.0)
            torque = torch.clamp(1.5 * axis - 0.1 * velocity[:, 3:6], -1.0, 1.0)
        module.permanent_wrench_composer.set_forces_and_torques_index(
            forces=force[:, None, :],
            torques=torque[:, None, :],
            body_ids=[0],
            is_global=True,
        )

    def spindle_pose(self, name):
        unit, index = self.drivers[name]
        origins = tensor(self.environment.unwrapped.scene.env_origins)
        return (
            tensor(unit.data.body_pos_w)[:, index] - origins,
            tensor(unit.data.body_quat_w)[:, index],
        )

    def update_motor(self, name, spin, direction):
        import torch

        unit, _ = self.drivers[name]
        pos, quat = self.spindle_pose(name)
        torque = torch.zeros_like(pos[:, :1])
        if self.held[name]:
            sockets = torch.as_tensor(
                MOUNT + np.array(FASTENERS), dtype=pos.dtype, device=pos.device
            )
            depth = (
                sockets[:, 2]
                + tensor(self.fixture.data.joint_pos)[:, self.lifts]
                - pos[:, 2:3]
            )
            radial = torch.linalg.vector_norm(pos[:, None, :2] - sockets[:, :2], dim=-1)
            upright = 2 * (quat[:, :2] ** 2).sum(dim=1, keepdim=True) < 1 - math.cos(
                math.radians(8)
            )
            compression = tensor(unit.data.joint_pos)[:, self.feed_joints[name], None]
            pressed = ((radial < 0.0015) & (depth > -0.00070) & (depth < 0.0040)).any(
                dim=1, keepdim=True
            )
            seated_bit = ((radial < 0.0010) & (depth > 0.0012) & (depth < 0.0040)).any(
                dim=1, keepdim=True
            )
            # A seated bit can close the modeled switch before compressing the spring.
            torque = torch.where(
                pressed & upright & ((compression > 0.0006) | seated_bit),
                SPINDLE_INDEXING_TORQUE * direction,
                torque,
            )
        if CASE == "free-spin" and self.held[name]:
            # Deliberately power the raised tool during the no-engagement control.
            torque.fill_(SPINDLE_INDEXING_TORQUE * direction)
        self.motor_commands[name] = torque
        unit.set_joint_effort_target(torque, joint_ids=[spin])

    def read_driver(self, contacts):
        self.held = {name: contacts[name] for name in ACTIVE_DRIVERS}
        heads = tensor(self.fixture.data.joint_pos)[0, self.bolts].tolist()
        lifts = tensor(self.fixture.data.joint_pos)[0, self.lifts].tolist()
        name = next((k for k in ACTIVE_DRIVERS if self.held[k]), ACTIVE_DRIVERS[0])
        pos, quat = self.spindle_pose(name)
        x, y, z, w = quat[0].tolist()
        yaw = math.atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z))
        tilt = math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
        unit, _ = self.drivers[name]
        compression = float(
            tensor(unit.data.joint_pos)[0, self.feed_joints[name]].item()
        )
        return DriverReading(
            tuple(pos[0].tolist()),
            yaw,
            tilt,
            self.held[name],
            tuple(heads),
            tuple(lifts),
            compression,
            (2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)),
        )


def _register_runtime():
    from scenekit import UsdBody, on_reset, on_step
    import touch

    wrist = UsdBody(
        "wrist",
        usd="assets/yam_wrist_fixture.usda",
        pos=tuple(MOUNT),
        size=(0.19, 0.11, 0.095),
        articulation=True,
        material=False,
        solver_iters=32,
        joint_damping={"bolt_.*": BOLT_DAMPING, "rise_.*": 0.0},
        joint_effort_limit=18.0,
        joint_pos=tuple(
            (f"{kind}_{i}", 0.0)
            for i in range(len(FASTENERS))
            for kind in ("bolt", "rise")
        ),
    )
    modules = [
        UsdBody(
            name,
            usd="assets/linear_4310_module.usda",
            pos=tuple(pos),
            size=(0.158, 0.068, 0.104),
            mass=GRIPPER_MASS,
            friction=(1.1, 0.9),
            torsional_patch=0.014,
            solver_iters=32,
        )
        for name, pos in zip(MODULES, (MOUNT, REPLACEMENT))
    ]
    tools = [
        UsdBody(
            name,
            usd=f"assets/{name}.usda",
            pos=pose,
            size=(0.029, 0.029, 0.147),
            articulation=True,
            joint_pos=tuple((j, 0.0) for j in ("spin", "feed")),
            joint_damping={"spin": 0.00075, "feed": 0.24},
            joint_stiffness={"spin": 0.0, "feed": 450.0},
            joint_effort_limit={"spin": 0.033, "feed": 3.0},
            friction=(0.36, 0.26),
            solver_iters=48,
        )
        for name, (pose, _) in ((name, DRIVERS[name]) for name in ACTIVE_DRIVERS)
    ]
    cradles = [
        UsdBody(
            name,
            usd="assets/module_cradle.usda",
            pos=(p[0], p[1], TABLE),
            size=(0.18, 0.12, 0.012),
            mass=None,
        )
        for name, p in zip(("old_cradle", "new_cradle"), (REMOVED, REPLACEMENT)[:len(MODULES)])
    ]
    stands = [
        UsdBody(
            f"{name}_stand",
            usd="assets/driver_stand.usda",
            pos=(*pose[:2], TABLE),
            size=(0.050, 0.050, 0.056),
            mass=None,
        )
        for name, (pose, _) in ((name, DRIVERS[name]) for name in ACTIVE_DRIVERS)
    ]

    @on_reset
    def reset(environment):
        global _trial, _old, _new, _driver, _physics, _socket_index
        _trial = Trial()
        _old = GripperReading(tuple(MOUNT), 0, True, True)
        _new = GripperReading(tuple(REPLACEMENT), 0, True, True)
        _driver = DriverReading((0.0, 0.0, TABLE), 0, 0, False)
        _bit_trim[:] = 0.0
        _tool_lean[:] = 0.0
        _dock_trim[:] = 0.0
        _dock_lean[:] = 0.0
        _placement_trim[:] = 0.0
        _placement_lean[:] = 0.0
        _socket_index = None
        touch.reset()
        if (
            _physics is None
            or _physics.fixture is not environment.unwrapped.scene["wrist"]
        ):
            _physics = WristClamp(environment)
        _physics.held = dict.fromkeys(ACTIVE_DRIVERS, False)
        _physics.engaged_socket = None

    @on_step
    def monitor(environment):
        global _old, _new, _driver
        from yam_arena.scene import object_states

        contact = touch.touching(environment, [*MODULES, *ACTIVE_DRIVERS])
        states = object_states(environment, MODULES)
        _old = read_gripper(states["old_gripper"], not contact["old_gripper"])
        if not REMOVAL_ONLY:
            _new = read_gripper(states["new_gripper"], not contact["new_gripper"])
        _driver = _physics.read_driver(contact)
        _physics.engaged_socket = _driver.socket() if CASE != "free-spin" else None
        previous = tuple(_trial.opened)
        _trial.step(_old, _new, _driver, float(environment.unwrapped.step_dt))
        if previous != tuple(_trial.opened):
            from isaaclab.sim.utils.stage import get_current_stage
            from pxr import Gf, UsdGeom

            stage = get_current_stage()
            for index, released in enumerate(_trial.opened):
                matches = [
                    prim for prim in stage.Traverse()
                    if prim.GetName() == f"ReleaseWitness{index}"
                ]
                for prim in matches:
                    color = (0.06, 0.8, 0.16) if released else (0.85, 0.08, 0.035)
                    UsdGeom.Gprim(prim).GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

    return [wrist, *modules, *tools, *cradles, *stands]


def tracks():
    left, right = Track(), Track()
    module_grip, tool_grip = -0.024, -0.0035
    carry = TABLE + 0.131
    drive_hover = TABLE + 0.185
    driver_park = (0.183, -0.265, TABLE + 0.136)
    left_clear = (0.194, 0.188, carry)
    hand_above_bit = TOOL_GRIP + TIP - 0.003

    def over(xyz):
        return (xyz[0], xyz[1], carry)

    def drive(name):
        datum, direction = DRIVERS[name]
        hand = (*datum[:2], datum[2] + hand_above_bit)
        above_stand = (*hand[:2], drive_hover)
        right.go(above_stand, OPEN, "above powered driver", 1.65)
        right.go(hand, OPEN, "lower onto parked driver casing", 1.6)
        right.go(hand, tool_grip, "grip driver casing", 1.2)
        right.go(
            (hand[0], hand[1], hand[2] + 0.030),
            tool_grip,
            "lift clear of casing dock",
            1.7,
        )
        right.go(above_stand, tool_grip, "confirm bit clears the stand", 0.8)
        verb = "loosen" if direction > 0 else "tighten"
        for i, offset in enumerate(FASTENERS):
            if CASE == "one-engaged" and i == 1:
                continue
            x, y, z = MOUNT + offset
            top = (x, y, drive_hover)
            seat = (x, y, z + hand_above_bit + 0.0080)
            right.go(top, tool_grip, f"above collar screw {i+1}", 1.55)
            if CASE == "free-spin":
                right.go(top, tool_grip, "spin above the socket without engagement", 7.0)
                continue
            right.go(
                (x, y, z + hand_above_bit + 0.023),
                tool_grip,
                "center quill in pilot sleeve",
                2.8,
            )
            right.go(
                (x, y, z + hand_above_bit + 0.017),
                tool_grip,
                "feed bit into guide",
                1.05,
            )
            right.go(seat, tool_grip, "load spring quill in hex recess", 2.0)
            right.go(seat, tool_grip, f"hold driver to {verb} screw {i+1}", 7.0)
            right.go(
                (x, y, z + hand_above_bit + 0.022),
                tool_grip,
                "unload the quill straight up",
                1.15,
            )
            right.go(top, tool_grip, "withdraw bit before leaving screw", 1.6)
        right.go(above_stand, tool_grip, "return powered driver", 1.55)
        right.go(above_stand, tool_grip, f"align driver over stand {name}", 1.7)
        right.go(hand, tool_grip, f"lower into stand {name}", 1.7)
        right.go(hand, OPEN, f"release driver {name}", 0.85)
        right.go(above_stand, OPEN, "clear driver stand", 1.25)
        right.go(driver_park, OPEN, "park driver arm clear", 1.35)

    left.go(left_clear, OPEN, "park module arm clear")
    drive("loosen_driver")
    left.wait_until(right.duration)

    old = MOUNT + MODULE_OFFSET
    old_out = old + (0.035, 0, 0)
    old_drop = REMOVED + MODULE_OFFSET
    new = REPLACEMENT + MODULE_OFFSET
    left.go(over(old), OPEN, "above the released module", 1.65)
    left.go(old, OPEN, "lower onto module housing")
    left.go(old, module_grip, "grip loose module", 1.0)
    left.go(old_out, module_grip, "slide module clear of the wrist", 1.9)
    left.go(over(old_out), module_grip, "lift the removed module")
    left.go(over(old_drop), module_grip, "carry to the empty cradle", 1.7)
    left.go(old_drop, module_grip, "lower removed unit")
    left.go(old_drop, OPEN, "release removed unit", 0.9)
    left.go(over(old_drop), OPEN, "clear cradle")
    if REMOVAL_ONLY:
        left.go(left_clear, OPEN, "park the module arm", 1.5)
        right.wait_until(left.duration)
        return {"left": left, "right": right}
    left.go(over(new), OPEN, "above replacement", 1.7)
    left.go(new, OPEN, "around replacement housing")
    left.go(new, module_grip, "grasp replacement", 0.95)
    left.go(over(new), module_grip, "lift replacement")
    left.go(over(old_out), module_grip, "move to open side of wrist", 1.7)
    left.go(old_out, module_grip, "align the replacement flange")
    left.go(old, module_grip, "slide flange home into saddle", 1.9)
    left.go(old, module_grip, "settle flange in the saddle", 1.7)
    left.go(old, OPEN, "release module into saddle", 0.95)
    left.go(over(old), OPEN, "lift off the seated module")
    # Push the upper housing along the saddle after the gripping fingers clear its sides.
    front = (MOUNT[0] + 0.096, MOUNT[1], MOUNT[2] + TIP + 0.028)
    flush = (MOUNT[0] + 0.0675, front[1], front[2])
    left.go(over(front), 0.0, "above the front of the replacement housing", 1.4)
    left.go(front, 0.0, "lower above the module jaws", 1.4)
    left.go(flush, 0.0, "push the replacement flange flush", 2.0)
    left.go(front, 0.0, "back straight out of the saddle", 1.25)
    left.go(over(front), OPEN, "clear the replacement housing", 1.2)
    left.go(left_clear, OPEN, "park module arm before tightening", 1.5)

    right.wait_until(left.duration)
    drive("tighten_driver")
    left.wait_until(right.duration)
    return {"left": left, "right": right}


def demo():
    t = tracks()
    return steered(t["left"], t["right"], steer_module, steer_driver)


def self_test(run_ik=False):
    for assumed in (.25, .50, 1.0):
        applied, net = drive_torque(0., 0., True, assumed)
        assert applied > assumed and net > 0.
        applied, net = drive_torque(.2, 3., True, assumed)
        assert math.isclose(applied - net, assumed * .20)
        assert drive_torque(0., 0., True, assumed, assumed * .8)[1] == 0.
        assert drive_torque(0., 0., False, assumed) == (0., 0.)
    t = Trial()
    installed = GripperReading(tuple(MOUNT), 0, True, True)
    old_stowed = GripperReading(tuple(REMOVED), 0, True, True)
    new_start = GripperReading(tuple(REPLACEMENT), 0, True, True)
    assert t.score(installed, new_start) == 0

    def sweep(index, sign, old, new, *, held=True, high=False, slips=False):
        center = MOUNT + FASTENERS[index]
        heads = [REQUIRED_TURN if opened else 0.0 for opened in t.opened]
        initial = 0 if sign > 0 else REQUIRED_TURN
        for k in range(121):
            heads[index] = (
                initial
                if slips
                else min(REQUIRED_TURN, max(0.0, initial + sign * k * 0.125))
            )
            lifts = [min(MAX_RISE, angle * THREAD_PITCH / math.tau) for angle in heads]
            tip = tuple(center + (0, 0, lifts[index] + (0.01 if high else -0.0022)))
            t.step(
                old,
                new,
                DriverReading(
                    tip, sign * k * 0.125, 0, held, tuple(heads), tuple(lifts)
                ),
                0.05,
            )
        t.step(
            old,
            new,
            DriverReading(
                tuple(center + (0, 0, 0.01)), 0, 0, False, tuple(heads), tuple(lifts)
            ),
            0.05,
        )

    sweep(0, 1, installed, new_start, high=True)
    assert t.loose == [0, 0]
    sweep(0, 1, installed, new_start, held=False)
    sweep(0, 1, installed, new_start, slips=True)
    assert t.loose == [0, 0]
    sweep(0, 1, installed, new_start)
    assert not t.old_released and t.opened[0]
    sweep(1, 1, installed, new_start)
    if REMOVAL_ONLY:
        assert t.old_released and t.score(old_stowed, new_start) == 0.5
        clear = DriverReading((0, 0, 1), 0, 0, False, (REQUIRED_TURN,) * 2, (MAX_RISE,) * 2)
        t.step(GripperReading(tuple(MOUNT + (0.028, 0, 0)), 0, False, True),
               new_start, clear, 0.05)
        t.step(old_stowed, new_start, clear, 1.0)
        assert t.score(old_stowed, new_start) == 1.0
        unsupported = Trial(loose=[REQUIRED_TURN] * 2)
        unsupported.step(GripperReading(tuple(MOUNT + (.028, 0, 0)), 0, True, True),
                         new_start, clear, .05)
        assert unsupported.invalid
        dropped = Trial(loose=[REQUIRED_TURN] * 2, separated=True)
        dropped.step(GripperReading((REMOVED[0], REMOVED[1], TABLE - .03), 0, True, True),
                     new_start, clear, .05)
        assert dropped.invalid
        held = Trial(loose=[REQUIRED_TURN] * 2, separated=True)
        held.step(GripperReading(tuple(REMOVED), 0, False, True), new_start, clear, 2.)
        assert held.score(old_stowed, new_start) == .5
        check_tracks(tracks(), MAX_STEPS, ik=run_ik)
        print("YAM module removal and released-stow checks passed.")
        return
    assert t.old_released and t.score(old_stowed, new_start) == 0.25
    assert t.score(old_stowed, installed) == 0.50
    sweep(0, -1, old_stowed, installed)
    assert t.score(old_stowed, installed) == 0.75
    sweep(1, -1, old_stowed, installed)
    away = DriverReading((0, 0, 1), 0, 0, False)
    held_new = GripperReading(tuple(MOUNT), 0, False, True)
    t.step(old_stowed, held_new, away, 1.0)
    assert t.score(old_stowed, held_new) == 0.75
    t.step(old_stowed, installed, away, 1.0)
    assert t.score(old_stowed, installed) == 1
    t.step(old_stowed, new_start, away, 0.05)
    assert t.invalid and t.score(old_stowed, new_start) == 0
    forced = Trial()
    forced.step(old_stowed, new_start, away, 0.05)
    assert forced.invalid
    check_tracks(tracks(), MAX_STEPS, ik=run_ik)
    print("YAM screw gate, scoring and waypoint checks passed.")


if __name__ == "__main__":
    self_test("--ik" in sys.argv)
else:
    OBJECTS = _register_runtime()
