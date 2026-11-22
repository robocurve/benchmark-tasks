"""Launch a flexible topped pizza from a stock peel onto a cold oven stone."""

from dataclasses import dataclass
from math import cos, pi, sin, sqrt
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from scenekit import KINDS, Shape, UsdBody, _np, on_reset, on_step
import tiltkit
from yam_arena.task_file import Box

SCENE = "station"
INSTRUCTION = "Launch the pizza from the peel onto the oven stone."
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.
STONE = np.array((.642, 0., .8425))
STONE_HALF = .165
PREP_TOP = .845
START_X = .285
DOUGH_X = .331
TOPPINGS = ((-.058, -.058), (.058, -.058), (-.058, .058), (.058, .058))
ENTRY = {"supported": False}


@dataclass(frozen=True)
class Dough(Box):
    """Food-grade deformable crust, registered with Arena's scene builder."""

    friction: float = .28


def build_dough(spec):
    import isaaclab.sim as sim
    from isaaclab_physx.sim.schemas import PhysxCollisionCfg, PhysxDeformableBodyPropertiesCfg
    from isaaclab_physx.sim.spawners.materials import PhysxDeformableBodyMaterialCfg
    from isaaclab_arena.assets.deformable_object import DeformableObject
    from isaaclab_arena.utils.pose import Pose

    return DeformableObject(
        name=spec.name,
        initial_pose=Pose(position_xyz=spec.pos),
        spawner_cfg=sim.UsdFileCfg(
            usd_path=str(HERE / "assets/crust.usdc"),
            deformable_props=PhysxDeformableBodyPropertiesCfg(linear_damping=.02, mass=spec.mass),
            collision_props=[PhysxCollisionCfg(rest_offset=0., contact_offset=.001)],
            physics_material=PhysxDeformableBodyMaterialCfg(
                density=580., youngs_modulus=4e4, poissons_ratio=.35, dynamic_friction=spec.friction),
        ),
    )


def dough_state(environment, spec):
    base = environment.unwrapped
    asset = base.scene[spec.name]
    nodes = _np(asset.data.nodal_pos_w)[0] - _np(base.scene.env_origins)[0]
    velocities = _np(asset.data.nodal_vel_w)[0]
    # Use the maximum free-node speed so a moving edge precludes a settled score.
    return nodes, float(np.max(np.linalg.norm(velocities, axis=1)))


KINDS[Dough] = (build_dough, dough_state)

OBJECTS = [
    UsdBody("oven", usd="assets/oven.usdc", pos=(.611, 0., .7475),
            size=(.45, .42, .30), rot_xyzw=(0., 0., -sqrt(.5), sqrt(.5)), material=False),
    UsdBody("peel", usd="assets/peel.usdc", pos=(START_X, 0., .8515), size=(.305, .505, .012),
            rot_xyzw=(0., 0., sqrt(.5), sqrt(.5)), mass=.717, material=False, torsional_patch=.010),
    Dough("crust", pos=(DOUGH_X, 0., .8645), size=(.240, .240, .011),
          mass=.210, friction=.09),
    *[UsdBody(f"topping_{i}", usd="assets/topping.usdc", size=(.028, .028, .0025),
              pos=(DOUGH_X+x, y, .8725), mass=.0025)
      for i, (x, y) in enumerate(TOPPINGS)],
    Shape("prep_worktop", pos=(.200, 0., .79625), size=(.500, .460, .0975),
          color=(.44, .29, .17)),
]


@on_reset
def reset_entry(environment):
    ENTRY["supported"] = False


@on_step
def track_entry(environment):
    """Register an entry through the mouth with the pizza supported by the peel."""
    if ENTRY["supported"]:
        return
    from yam_arena.scene import object_states

    objects = object_states(environment, ["peel", "crust"])
    peel = objects["peel"]
    nodes, _ = objects["crust"]
    center = (nodes.min(axis=0) + nodes.max(axis=0))/2
    across, along, height = peel.rotation().T @ (center - peel.pos)
    ENTRY["supported"] = bool(.49 <= center[0] <= .58 and abs(center[1]) <= .13
                              and center[2] >= STONE[2] and abs(across) <= .10
                              and -.145 <= along <= .12 and .003 <= height <= .045)


def outcome(vertices, speed, peel_front, toppings, supported_entry):
    extent_min, extent_max = np.min(vertices, axis=0), np.max(vertices, axis=0)
    center = (extent_min + extent_max) / 2
    on_stone = (supported_entry and np.all(np.abs(vertices[:, :2]-STONE[:2]) <= STONE_HALF+.003)
                and STONE[2]-.004 <= extent_min[2] <= STONE[2]+.005
                and extent_max[2] <= STONE[2]+.028
                and np.all(extent_max[:2] - extent_min[:2] >= .208)
                and speed <= .028 and peel_front <= .462)
    if not on_stone:
        return 0.
    if np.linalg.norm(center[:2]-STONE[:2]) > .045:
        return .5
    for position in toppings:
        radial = np.linalg.norm(position[:2] - vertices[:, :2], axis=1)
        if (np.linalg.norm(position[:2]-center[:2]) > .103
                or position[2] < STONE[2]+.003
                or position[2] > extent_max[2]+.006 or np.min(radial) > .030):
            return .5
    return 1.


def score(objects):
    peel = objects["peel"]
    corners = np.array([(x, y, 0.) for x in (-.153, .153) for y in (-.161, .345)])
    peel_front = float(np.max((corners @ peel.rotation().T + peel.pos)[:, 0]))
    vertices, speed = objects["crust"]
    return outcome(vertices, speed, peel_front,
                   [objects[f"topping_{i}"].pos for i in range(len(TOPPINGS))], ENTRY["supported"])


def success(objects):
    return score(objects) == 1.


def demo(positive=True):
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    from yam_arena.demo import Waypoint

    tiltkit.install()
    depth = .604 if positive else .495
    pitch, level = .012, .854
    tipped_z = level + .161*sin(pitch)
    path = (
        (3., START_X, .925, 0., False, "approach the stock handle"),
        (2., START_X, PREP_TOP, 0., False, "lower to the handle"),
        (1., START_X, PREP_TOP, 0., True, "pinch the handle"),
        (1.2, START_X, level, 0., True, "level the blade"),
        (2.25, depth, level, 0., True, "insert the topped pizza"),
        (.75, depth, tipped_z, pitch, True, "set the blade nose"),
        (.43, .270, level, .025, True, "withdraw the peel"),
        (1.6, .270, level, .025, True, "hold the peel clear"),
    )
    goals = {}
    for side, offset, roll in (("left", .184, -.45), ("right", .302, .45)):
        grip = tiltkit.rotation((.48, roll, -1.), (0., -1., 0.))
        goals[side] = []
        for seconds, x, height, pitch, closed, label in path:
            rot = np.array([[cos(pitch), 0., sin(pitch)],
                            [0., 1., 0.], [-sin(pitch), 0., cos(pitch)]])
            tcp = np.array((x, 0., height)) + rot @ (-offset, 0., .005)
            goals[side].append(Waypoint(seconds, tuple(tcp), FINGER_CLOSED if closed else FINGER_OPEN,
                                        label, tiltkit.Tilt(rot @ grip)))
    return goals


def self_test():
    samples = np.linspace(0., 2*pi, 64, endpoint=False)
    vertices = STONE + np.array([.12*np.cos(samples), .12*np.sin(samples), .002+samples*0]).T
    toppings = [STONE + (x, y, .007) for x, y in TOPPINGS]
    vertices = np.vstack([vertices, [STONE + (x, y, .005) for x, y in TOPPINGS]])
    assert outcome(vertices, 0., .40, toppings, True) == 1.
    assert outcome(vertices, 0., .40, [STONE + (.15, 0., .002)], True) == .5
    assert outcome(vertices + (.04, .04, 0.), 0., .40, toppings, True) == .5
    assert outcome(vertices + (.05, .05, 0.), 0., .40, toppings, True) == 0.
    assert outcome(vertices, .10, .40, toppings, True) == 0.
    assert outcome(vertices, 0., .50, toppings, True) == 0.
    assert outcome(vertices, 0., .40, toppings, False) == 0.


if __name__ == "__main__":
    self_test()
    from yam_arena.constants import SHOULDER, WRIST
    from yam_arena.demo import ArmPlan
    from yam_arena.station import BASES

    print("Pizza scorer checks passed.")
    for side, waypoints in demo().items():
        plan = ArmPlan(waypoints, BASES[side])
        joints = np.array([plan.action7(i/20)[0][:6]
                           for i in range(1, round(20*plan.duration)+1)])
        velocities = 20*np.max(np.abs(np.diff(joints, axis=0)), axis=0)
        print(f"{side}: {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f}° "
              f"peak joint speeds {velocities.round(2)} rad/s")
        assert plan.worst_ik_mm < 2. and plan.worst_ik_deg < 2.
        assert np.all(velocities[:3] < SHOULDER["velocity"])
        assert np.all(velocities[3:] < WRIST["velocity"])
