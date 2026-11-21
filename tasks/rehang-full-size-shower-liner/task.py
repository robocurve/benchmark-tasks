"""Rehang two middle eyelets of a full-width, gravity-loaded PEVA shower liner."""
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path

import numpy as np

from geometry import BASES, BORE, CENTERS, COUNT, LOWER_RETURN, MASS, ROWS, STRIP, TARGETS, TIP_HEIGHT, X, Z
from runtime import asset, install_writer, register, scalar_array, tick, wrench
from scenekit import UsdBody
from yam_arena.constants import TCP_OFFSET
from shower_bay import ShowerPlan, ShowerWaypoint
from yam_arena.urdf import quat_xyzw_to_matrix
from touch import contact_links, reset as reset_touch

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
SPEC = json.loads(Path(__file__).with_name("task.json").read_text())
MAX_STEPS = 900
INSTRUCTION = SPEC["instruction"]
MODE = os.getenv("SHOWER_CONTROL", "demo")
ORIGINALS = tuple(i for i in range(COUNT) if i not in TARGETS)
STRIPS = tuple(f"strip_{i:02}" for i in range(COUNT))
HOOKS = tuple(f"hook_{i:02}" for i in range(COUNT))
PLANTED = np.array([(X, y, Z) for y in CENTERS])
HOOK_POSITIONS = PLANTED.copy()
if MODE == "wrong-hook":
    # The negative fixture keeps the wrong return within the liner's reach.
    HOOK_POSITIONS[6, 1] = CENTERS[5] + .0245
INITIAL = PLANTED + (.006, 0., .008)
INITIAL[list(TARGETS)] = PLANTED[list(TARGETS)] + (-.052, 0., -.057)
if MODE == "wrong-hook":
    INITIAL[5] = HOOK_POSITIONS[6] + (.006, 0., .008)
OPEN = -.04695
PINCH = -.001
GRASP_OFFSET = np.array((-.028, 0., .011))


@dataclass
class Trial:
    elapsed: float = 0.
    hold: float = 0.
    repaired: int = 0
    invalid: bool = False
    loss: np.ndarray = field(default_factory=lambda: np.zeros(COUNT))
    reference: np.ndarray | None = None
    overload: float = 0.
    faults: set[str] = field(default_factory=set)

    def step(self, seats, support, nearest, eyes, clear, strain, dt, fixture_load=0.):
        self.elapsed += dt
        correct = seats & (support > .012)
        self.repaired = int(correct[list(TARGETS)].sum())
        lost = ~seats[list(ORIGINALS)]
        self.loss[list(ORIGINALS)] = np.where(lost, self.loss[list(ORIGINALS)] + dt, 0.)
        wrong = (nearest != np.arange(COUNT)) & (nearest >= 0)
        self.overload = self.overload + dt if fixture_load > 40. else 0.
        faults = {
            "wrong hook": wrong.any(),
            "excess strain": strain > .17,
            "liner dropped": np.min(eyes[:, 2]) < 1.70,
            "fixture overload": self.overload > .15,
            "original grommet displaced": self.elapsed > .6 and max(self.loss) > .33,
        }
        self.faults.update(fault for fault, detected in faults.items() if detected)
        self.invalid = bool(self.faults)
        if not (correct.all() and clear and not self.invalid):
            self.hold = 0.
            self.reference = None
        else:
            if self.reference is None or np.max(np.linalg.norm(eyes - self.reference, axis=1)) > .005:
                self.hold = 0.
                self.reference = eyes.copy()
            self.hold += dt

    @property
    def value(self):
        if self.invalid:
            return 0.
        return 1. if self.hold >= 1. - 1e-8 else .35 * self.repaired


_trial = Trial()
_state = {}
_physics = None


def telemetry():
    return dict(_state, hold=round(_trial.hold, 2), invalid=_trial.invalid, faults=sorted(_trial.faults))


def score(objects):
    return _trial.value


def success(objects):
    return _trial.value == 1.


class Membrane:
    """Eight fold joints per strip, with equal-and-opposite elastic seam loads."""

    def __init__(self, environment):
        import torch

        self.scene = environment.unwrapped.scene
        self.strips = [asset(environment, key) for key in STRIPS]
        self.ids = [tuple(s.body_names.index(n) for n in ("eye", *(f"sheet{j}" for j in range(ROWS))))
                    for s in self.strips]
        self.forces = None
        self.torques = None
        self.drag = torch.tensor((.035, .012, .010), device=self.scene.device)
        self.contact_view = None
        self.rest = torch.tensor([STRIP] * (ROWS + 1), device=self.scene.device)
        self.stiffness = torch.tensor([90.] + [39.] * ROWS, device=self.scene.device)
        for i, unit in enumerate(self.strips):
            def apply(index=i, current=unit):
                if index == 0:
                    self.calculate()
                order = torch.as_tensor(self.ids[index], device=self.scene.device)
                wrench(current, self.forces[index].unsqueeze(0),
                       self.torques[index].unsqueeze(0), ids=order)
            install_writer(unit, apply)

    def calculate(self):
        import torch

        position = torch.stack([s.data.body_pos_w.torch[0, ids, :] for s, ids in zip(self.strips, self.ids)])
        velocity = torch.stack([s.data.body_lin_vel_w.torch[0, ids, :] for s, ids in zip(self.strips, self.ids)])
        delta = position[1:] - position[:-1]
        span = torch.linalg.vector_norm(delta, dim=-1).clamp_min(.005)
        axis = delta / span.unsqueeze(-1)
        extension = span - self.rest
        rate = ((velocity[1:] - velocity[:-1]) * axis).sum(dim=-1)
        # A thin sheet buckles in compression. Allow slack, retaining modest damping.
        tension = (self.stiffness * extension + .32 * rate).clamp(0., 9.)
        edge = axis * tension.unsqueeze(-1)
        # Drag and angular damping dissipate motion without supporting any static load.
        angular_velocity = torch.stack([s.data.body_ang_vel_w.torch[0, ids, :]
                                        for s, ids in zip(self.strips, self.ids)])
        self.forces = -(velocity * self.drag).clamp(-.15, .15)
        self.torques = -(angular_velocity * .00012).clamp(-.015, .015)
        self.forces[:-1] += edge
        self.forces[1:] -= edge
        self.stretch = (span / self.rest - 1.).amax()

    def measurements(self):
        poses = np.stack([scalar_array(unit.data.body_pos_w)[0, indices, :]
                          for unit, indices in zip(self.strips, self.ids)])
        origin = scalar_array(self.scene.env_origins)[0]
        return poses - origin

    def eye_rotations(self):
        return np.stack([quat_xyzw_to_matrix(scalar_array(unit.data.body_link_pose_w)[0, ids[0], 3:])
                         for unit, ids in zip(self.strips, self.ids)])

    def hook_forces(self, dt):
        if self.contact_view is None:
            from isaaclab.sim.utils.stage import get_current_stage
            from isaaclab_physx.physics import PhysxManager

            stage = get_current_stage()
            def body_path(key, name):
                root = self.scene[key].root_view.prim_paths[0]
                prim = stage.GetPrimAtPath(root)
                if prim.GetName() == name:
                    return str(prim.GetPath())
                for candidate in prim.GetChildren():
                    if candidate.GetName() == name:
                        return str(candidate.GetPath())
                raise ValueError(f"Cannot locate {name} below {root}")
            eyes = [body_path(k, "eye") for k in STRIPS]
            hooks = [body_path(k, "hook") for k in HOOKS]
            self.contact_view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
                eyes, filter_patterns=[hooks] * COUNT,
                max_contact_data_count=256 * COUNT,
            )
        matrix = scalar_array(self.contact_view.get_contact_force_matrix(dt=dt)).reshape(COUNT, COUNT, 3)
        return matrix


def engagement(eyes, normals, hook_positions=PLANTED):
    """Test physical intersection between the eye plane and the curved lower return."""
    return_centerline = np.array([(x, 0., z) for x, z in LOWER_RETURN])
    return_centerline[0, 2] -= .004
    end_points = hook_positions[None, :, None, :] + return_centerline[None, None, :, :]
    relative = end_points - eyes[:, None, None, :]
    projection = (relative * normals[:, None, None, :]).sum(axis=-1)
    first, last = projection[:, :, :-1], projection[:, :, 1:]
    span = last - first
    fraction = -first / np.where(np.abs(span) > 1e-9, span, 1.)
    crossing = (fraction >= 0.) & (fraction <= 1.) & (np.abs(span) > 1e-9)
    locations = relative[:, :, :-1] + fraction[:, :, :, None] * np.diff(relative, axis=2)
    radial_distance = np.linalg.norm(locations, axis=-1)
    return (crossing & (radial_distance <= BORE)).any(axis=-1)


def initialize_environment(environment):
    global _physics
    _physics = Membrane(environment)


def release_environment():
    global _physics
    reset_touch()
    _physics = None


def setup():
    objects = [UsdBody("room_shower", usd="assets/shower.usda", pos=(0.,0.,0.),
                       size=(.65,2.,2.17), mass=None)]
    for i in range(COUNT):
        objects += [
            UsdBody(HOOKS[i], usd="assets/hook.usda", pos=tuple(HOOK_POSITIONS[i]),
                    size=(.08,.02,.11), mass=None, friction=(.48,.40)),
            UsdBody(STRIPS[i], usd="assets/strip.usda", pos=tuple(INITIAL[i]),
                    size=(.09,.144,1.829), mass=MASS/COUNT, articulation=True,
                    joint_pos=tuple((f"bend{j}",0.) for j in range(ROWS)),
                    joint_stiffness=.006, joint_damping=.014, joint_effort_limit=.25,
                    friction=(.75,.58), solver_iters=24),
        ]

    def reset(environment):
        global _trial, _physics
        _trial = Trial()
        _state.clear()
        reset_touch()
        if _physics is None or _physics.scene is not environment.unwrapped.scene:
            _physics = Membrane(environment)
        _physics.contact_view = None

    def update(environment):
        dt = tick(environment)
        all_panels = _physics.measurements()
        eyes = all_panels[:, 0]
        matrix = _physics.hook_forces(environment.unwrapped.physics_dt)
        force = np.linalg.norm(matrix, axis=-1)
        normals = _physics.eye_rotations()[:, :, 2]
        retained = engagement(eyes, normals, HOOK_POSITIONS)
        retained_force = force * retained
        nearest = np.where(retained_force.max(axis=1) > .012, retained_force.argmax(axis=1), -1)
        seats = retained.diagonal()
        captive_near_seat = eyes[:, 2] < Z + .0095
        support = force.diagonal() * captive_near_seat
        fixture_load = np.linalg.norm(matrix.sum(axis=(0, 1)))
        scene = environment.unwrapped.scene
        origin = scalar_array(scene.env_origins)[0]
        tcp = {}
        for arm in BASES:
            robot = scene[f"{arm}_robot"]
            pose = scalar_array(robot.data.body_link_pose_w)[0, robot.body_names.index("gripper")]
            tcp[arm] = pose[:3] - origin + quat_xyzw_to_matrix(pose[3:]) @ TCP_OFFSET
        fingers = list(tcp.values())
        held = contact_links(environment, [f"{key}/eye" for key in STRIPS])
        cloud = all_panels.reshape(-1, 3)
        clear = not any(held.values()) and min(np.linalg.norm(cloud - x, axis=1).min() for x in fingers) > .08
        strain = max(0., float(getattr(_physics, "stretch", 0.)))
        _trial.step(seats, support, nearest, eyes, clear, strain, dt, fixture_load)
        _state.update(support=support.round(3).tolist(), eyes=eyes.round(4).tolist(),
                      normals=normals.round(3).tolist(), fixture_load=round(float(fixture_load), 2),
                      tcp={arm: pos.round(4).tolist() for arm, pos in tcp.items()},
                      touched={key: ["/".join(link.split("/")[-2:]) for link in value]
                               for key, value in held.items() if value},
                      confirmed=(seats & (support > .012)).tolist(), nearest=nearest.tolist(),
                      threaded=seats.tolist(),
                      strain=round(strain, 4), clear=clear, lowest_panel=round(float(cloud[:,2].min()), 3))

    return register(objects, reset, update)


def tracks():
    settled = _physics.measurements()[:, 0] if _physics is not None else INITIAL
    rotations = _physics.eye_rotations() if _physics is not None else np.repeat(np.eye(3)[None], COUNT, axis=0)
    paths = {}
    for arm, i, park in (("right", 5, -.29), ("left", 6, .29)):
        if MODE == "original-hook-displacement" and arm == "right":
            i = 4
        y = CENTERS[i]
        start = settled[i] + rotations[i] @ np.array((-.036, 0., 0.)) + (0., 0., -.010)
        high = np.array((X, y, Z + TIP_HEIGHT + .032)) + GRASP_OFFSET
        low = PLANTED[i] + GRASP_OFFSET + (0., 0., .001)
        if arm == "right":
            high += (-.005, -.005, 0.)
            low += (-.006, -.006, 0.)
        else:
            high += (.010, .007, .019)
            low += (.007, .008, 0.)
        if MODE in ("settle", "wrong-hook") or (MODE == "original-hook-displacement" and arm == "left"):
            paths[arm] = [ShowerWaypoint(4.,(.17,park,2.04),OPEN,"settle without contact")]
            continue
        park_pos = (.17,park,2.045)
        clear = start + (0,0,.085)
        if arm == "left":
            start += (.002, -.001, .004)
            clear += (.002, -.001, .004)
        lifted = np.array((start[0] - .025, y, Z + .015))
        front_high = np.array((.302, high[1], high[2]))
        first = i == 5
        one_eye_control = MODE == "partial"
        if one_eye_control and not first:
            # This hand supports the loose hem without bringing the second
            # grommet past a cap. Leave it loose when the hand withdraws.
            front_high = high = low = lifted
        pieces = [
            (1.7, clear + (-.033,0.,0.), -.022, "above and in front of loose hem"),
            (1.4, start + (-.032,0.,0.), -.022, "descend in front of the hook"),
            (.9, start, -.022, "approach eye border at grasp height"),
            (1.0, start, PINCH, "pinch reinforced hem"),
            (1.6, lifted, PINCH, "support hanging liner"),
        ]
        if MODE == "original-hook-displacement":
            pieces += [
                (1.8, start + (0., 0., .055), PINCH, "lift original grommet off its return"),
                (1.2, start + (-.055, 0., .055), PINCH, "take original grommet clear"),
                (1., start + (-.055, 0., .055), OPEN, "release displaced original grommet"),
            ]
            low = start + (-.055, 0., .055)
        elif MODE == "release-before-attachment":
            pieces += [(1., lifted, OPEN, "release before reaching the hook")]
            low = lifted
        else:
            pieces += [
                (1.8, front_high if first else lifted, PINCH, "raise first eye forward of the hook"),
                (1.6, high if first else lifted, PINCH, "align first eye above its return"),
                (2.0, low if first else lifted, PINCH, "seat first eye"),
                (1.8, low if first else front_high, PINCH, "support first and raise second"),
                (1.6, low if first else high, PINCH, "align second eye above its return"),
                (2.0, low, PINCH, "seat second eye"),
                (1.2, low, -.021 if one_eye_control else OPEN, "gradually release both hands"),
            ]
        if one_eye_control:
            pieces += [(1., low + (-.052, 0., 0.), -.021, "withdraw before lifting clear")]
        pieces += [
            (1.2, low + (-.045,0,.065), OPEN, "clear the hooks"),
            (1.7, park_pos, OPEN, "park"),
        ]
        paths[arm] = [ShowerWaypoint(seconds, tuple(target), finger, label,
                                     tilt=math.radians(-35 if step < len(pieces) - 2 else -15))
                      for step, (seconds, target, finger, label) in enumerate(pieces)]
    return paths

demo = tracks


def verify():
    for arm, path in tracks().items():
        plan = ShowerPlan(path, BASES[arm])
        for step in range(math.ceil(plan.duration * 20)):
            plan.action7(step / 20)
        print(arm, "max IK residual", round(plan.worst_ik_mm, 2), "mm",
              round(plan.worst_ik_deg, 2), "degrees")
        assert plan.worst_ik_mm < 6 and plan.worst_ik_deg < 8
    up = np.tile((0., 0., 1.), (COUNT, 1))
    assert engagement(PLANTED, up).diagonal().all()
    assert not engagement(PLANTED + (-.014, 0., 0.), up).diagonal().any()
    assert not engagement(PLANTED + (0., 0., TIP_HEIGHT), up).diagonal().any()
    t = Trial()
    state = np.ones(COUNT, dtype=bool)
    force = np.ones(COUNT)
    t.step(state, force, np.arange(COUNT), PLANTED, False, 0., 1.)
    assert t.value == .70
    t.step(state, force, np.arange(COUNT), PLANTED, True, 0., 1.)
    assert t.value == 1.
    force[5] = 0.
    t.step(state, force, np.arange(COUNT), PLANTED, True, 0., .05)
    assert t.value == .35
    force[4] = 0.
    t.step(state, force, np.arange(COUNT), PLANTED, True, 0., .5)
    assert not t.invalid, "An unloaded but captured original is still threaded."
    displaced = state.copy()
    displaced[4] = False
    t.step(displaced, force, np.arange(COUNT), PLANTED, True, 0., .5)
    assert t.value == 0. and t.invalid
    wrong = np.arange(COUNT)
    wrong[5] = 4
    t = Trial()
    t.step(state, np.ones(COUNT), wrong, PLANTED, True, 0., 1.)
    assert t.value == 0. and t.invalid
    t = Trial()
    t.step(state, np.ones(COUNT), np.arange(COUNT), PLANTED, True, 0., .20, fixture_load=41.)
    assert t.invalid and t.value == 0. and t.faults == {"fixture overload"}
    t = Trial()
    t.step(state, np.ones(COUNT), np.arange(COUNT), PLANTED, True, .18, .05)
    assert t.invalid and t.value == 0. and t.faults == {"excess strain"}
    print("Threading, support, wrong-hook, original-loss, hands-clear, strain and fixture regressions passed")


if __name__ == "__main__":
    verify()
else:
    OBJECTS = setup()
