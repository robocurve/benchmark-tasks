"""Hook a real load onto a portable scale, wait for its hold reading, and lower it."""

from __future__ import annotations

import math
import os
from collections import deque
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, allow_matrix_grips, as_numpy, on_reset, on_step, rotation, segment
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS
from yam_arena.demo import Waypoint, grip_rotation
from yam_arena.task_file import TABLE_TOP

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
allow_matrix_grips()

SCALE = np.array([0.354, -0.142, TOP + 0.203])
BAG = np.array([0.302, 0.057, TOP + 0.004])
HINGE = np.array([0.0, 0.0, 0.112])
PEG = SCALE + [0.0, -0.020, 0.073]
WEIGHT = 0.642
SPRING = 980.0
HOOK_BOWL = np.array([0.0, 0.0, -0.111])
STILL_REQUIRED = 43
CONTACT_TOLERANCE = 0.10
MIN_UPWARD_HOOK_FORCE = 3.0
BLACK = (0.055, 0.058, 0.064)
SILVER = (0.57, 0.61, 0.66)
SCREEN = (0.054, 0.073, 0.057)
BAG_BLUE = (0.19, 0.35, 0.52)
BEIGE = (0.71, 0.60, 0.46)


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rim(radius, z):
    pts = [(radius*math.sin(a), 0.0, z + radius*math.cos(a))
           for a in np.linspace(0, 2*math.pi, 13)]
    return [segment(a, b, 0.0045, SILVER) for a, b in zip(pts, pts[1:])]


hook_path = [(-0.003, -0.044), (-0.003, -0.086), (-0.003, -0.103),
             (0.002, -0.109), (0.011, -0.110), (0.019, -0.104), (0.019, -0.093)]
hook_shapes = [segment((0, *a), (0, *b), 0.0055, SILVER)
               for a, b in zip(hook_path, hook_path[1:])]

OBJECTS = [
    Body.build("peg_stand", (PEG[0], PEG[1], TOP), 0.8, [
        BoxShape((0, -0.056, 0.008), (0.092, 0.101, 0.016), BEIGE),
        BoxShape((0, -0.067, 0.137), (0.025, 0.022, 0.276), BEIGE),
        CylShape((0, -0.016, PEG[2]-TOP), 0.004, 0.086, SILVER, axis="Y"),
    ], fixed=True, color=BEIGE, friction=(0.45, 0.35), contact_offset=0.00045),
    Linkage.build("scale", tuple(SCALE), 0.102, [
        BoxShape((0, 0, 0.0), (0.040, 0.024, 0.095), BLACK),
        BoxShape((-0.0205, 0, 0.026), (0.002, 0.021, 0.022), SCREEN),
        BoxShape((-0.021, 0.006, 0.005), (0.004, 0.007, 0.006), SILVER),
        BoxShape((-0.021, -0.006, 0.005), (0.004, 0.007, 0.006), SILVER),
        segment((0, 0, 0.046), (0, 0, 0.054), 0.007, SILVER),
        *_rim(0.015, 0.065),
    ], [
        Link("loadcell", tuple(hook_shapes), 0.013, tuple(SCALE + [0, 0, -0.043]), axis="Z", slide=True,
             limits=(-0.018, 0.002), stiffness=SPRING, spring_damping=12.0, report_contacts=True),
    ], floating=True, color=BLACK, friction=(0.93, 0.70), contact_offset=0.00035, solver_iterations=40),
    Linkage.build("bag", tuple(BAG), WEIGHT - 0.022, [
        BoxShape((0, 0, 0.049), (0.232, 0.181, 0.095), BAG_BLUE),
        BoxShape((0, 0, 0.100), (0.213, 0.166, 0.016), BAG_BLUE),
        BoxShape((0, -0.092, 0.082), (0.207, 0.006, 0.006), BLACK),
        BoxShape((0, 0, 0.018), (0.195, 0.185, 0.012), BLACK),
        *[BoxShape((x, 0, 0.109), (0.016, 0.022, 0.016), BLACK) for x in (-0.060, 0.060)],
    ], [
        Link("handle", (
            segment((-0.060, 0, 0.113), (-0.058, 0, 0.153), 0.008, BLACK),
            segment((0.060, 0, 0.113), (0.058, 0, 0.153), 0.008, BLACK),
            segment((-0.058, 0, 0.153), (0.058, 0, 0.153), 0.011, BLACK),
        ), 0.022, tuple(BAG+HINGE), axis="X", limits=(-91, 88),
             friction=0.013, damping=0.022, init=-70.0, report_contacts=True),
    ], floating=True, color=BAG_BLUE, friction=(0.94, 0.77), contact_offset=0.00045,
       solver_iterations=40, report_contacts=True),
]


def _vertices(shapes):
    signs = np.array([(x, y, z) for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    corners = []
    for shape in shapes:
        local = signs * np.asarray(shape.size) / 2
        corners.extend(local @ rotation(shape.quat()).T + shape.centre)
    return np.asarray(corners)


BAG_VERTICES = _vertices(OBJECTS[2].shapes)
HANDLE_VERTICES = _vertices(OBJECTS[2].links[0].shapes)

_state = {}


@on_reset
def _reset():
    _state.clear()
    _state.update(arms={}, indices={}, stable=deque(maxlen=STILL_REQUIRED), reading=None,
                  loaded=False, measured=False, wrong=False, score=0.0, step=0, display=None,
                  contact_view=None)


_reset()


def _arm_positions(base):
    result = {}
    for side, root in station.BASES.items():
        arm = base.scene[f"{side}_robot"]
        if side not in _state["arms"]:
            _state["arms"][side] = arm.find_joints(ARM_JOINTS, preserve_order=True)[0]
        q = as_numpy(arm.data.joint_pos)[0][_state["arms"][side]]
        result[side], _ = ik.tcp_pose(q, root)
    return result


def _support_forces(base):
    """Measure upward hook support and the largest competing bag or hook contact in N."""
    from isaaclab_physx.physics import PhysxManager
    from yam_arena.robot import asset_link_paths

    if _state["contact_view"] is None:
        root = base.scene.env_prim_paths[0]
        hook = f"{root}/scale/loadcell"
        bag = f"{root}/bag/base"
        handle = f"{root}/bag/handle"
        external = [f"{root}/scale/base", f"{root}/peg_stand/body"]
        external.extend(f"{root}/{arm}{link}" for arm in ("LeftArm", "RightArm")
                        for link in asset_link_paths().values())
        filters = [hook, handle, bag, *external]
        sim = PhysxManager.get_physics_sim_view()
        _state["contact_view"] = sim.create_rigid_contact_view(
            [handle, bag, hook], filter_patterns=[filters] * 3,
            max_contact_data_count=24 * len(filters),
        )
    view, dt = _state["contact_view"], float(base.physics_dt)
    forces = view.get_contact_force_matrix(dt=dt).numpy()
    net = view.get_net_contact_forces(dt=dt).numpy()
    handle_forces, bag_forces, hook_forces = forces
    unfiltered = np.linalg.norm(net - forces.sum(axis=1), axis=-1).max()
    foreign = max(np.linalg.norm(forces[:, 3:], axis=-1).max(),
                  np.linalg.norm(bag_forces[0]), np.linalg.norm(hook_forces[2]), unfiltered)
    return float(handle_forces[0, 2]), float(foreign)


def _draw_reading(base, value):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom

    state = (0.0 if value is None else value, value is not None)
    if _state["display"] == state:
        return
    _state["display"] = state
    stage = get_current_stage()
    prefix = base.scene.env_prim_paths[0] + "/scale/base/Digits"
    if stage.GetPrimAtPath(prefix):
        stage.RemovePrim(prefix)
    UsdGeom.Xform.Define(stage, prefix)
    segments = (
        (0, 0.0044, 0.0034, 0.0007),
        (-0.0018, 0.0023, 0.00072, 0.0032),
        (-0.0018, -0.0023, 0.00072, 0.0032),
        (0, -0.0044, 0.0034, 0.0007),
        (+0.0018, -0.0023, 0.00072, 0.0032),
        (+0.0018, 0.0023, 0.00072, 0.0032),
        (0, 0, 0.0034, 0.0007),
    )
    numbers = ("012345", "12", "01643", "01236", "5126", "56230", "562340", "012", "0123456", "012356")
    ink = (0.56, 0.85, 0.44)

    def pixel(name, y, z, width, height, color):
        block = UsdGeom.Cube.Define(stage, prefix + "/" + name)
        block.CreateSizeAttr(1)
        block.AddTranslateOp().Set(Gf.Vec3d(-0.022, y, z))
        block.AddScaleOp().Set(Gf.Vec3f(0.0002, width, height))
        block.CreateDisplayColorAttr([Gf.Vec3f(*color)])

    for i, d in enumerate(f"{state[0]:.2f}".replace(".", "")):
        for t in numbers[int(d)]:
            y, z, width, height = segments[int(t)]
            pixel(f"d{i}_{t}", 0.0062-0.0063*i+y, 0.025+z, width, height, ink)
    pixel("decimal", 0.0032, 0.0206, 0.0009, 0.0009, ink)
    if state[1]:
        pixel("hold", -0.0075, 0.0335, 0.0040, 0.0020, (0.05, 0.30, 0.07))


@on_step
def _step(env):
    base = getattr(env, "unwrapped", env)
    scale, bag = base.scene["scale"], base.scene["bag"]
    if not _state["indices"]:
        _state["indices"] = {"loadcell": scale.joint_names.index("loadcell_turn"),
                             "handle": bag.joint_names.index("handle_turn")}
    handle_idx, cell_idx = _state["indices"]["handle"], _state["indices"]["loadcell"]
    spose = as_numpy(scale.data.root_link_pose_w)[0]
    bpose = as_numpy(bag.data.root_link_pose_w)[0]
    bvel = as_numpy(bag.data.root_link_vel_w)[0]
    bdir = rotation(bpose[3:7])
    spos, bpos = spose[:3], bpose[:3]
    sq = float(as_numpy(scale.data.joint_pos)[0, cell_idx])
    bq = float(as_numpy(bag.data.joint_pos)[0, handle_idx])
    hinge_rotation = _rx(bq)
    handle = bpos + bdir @ (HINGE + hinge_rotation @ [0.0, 0.0, 0.041])
    handle_corners = (HANDLE_VERTICES - HINGE) @ hinge_rotation.T + HINGE
    bbottom = bpos[2] + min((BAG_VERTICES @ bdir[2]).min(), (handle_corners @ bdir[2]).min())
    mass = max(0.0, -sq * SPRING / 9.81 - 0.013)
    wrists = _arm_positions(base)
    grip_clear = all(min(np.linalg.norm(wrist - handle), np.linalg.norm(wrist - bpos - [0, 0, 0.08])) > 0.073
                     for wrist in wrists.values())
    hook_up, foreign_contact = _support_forces(base)
    hooked = (bbottom - TOP >= 0.025 and hook_up >= MIN_UPWARD_HOOK_FORCE
              and foreign_contact <= CONTACT_TOLERANCE)
    _state["loaded"] |= hooked
    gentle = (np.linalg.norm(bvel[:3]) <= 0.034 and np.linalg.norm(bvel[3:]) <= 0.35)
    scale_held = rotation(spose[3:7])[2, 2] >= math.cos(math.radians(12))
    if mass >= 0.20 and scale_held and _state["reading"] is None:
        _state["stable"].append((mass, spos.copy(), hooked))
        if len(_state["stable"]) == STILL_REQUIRED:
            readings, positions, valid_support = zip(*_state["stable"])
            housing_travel = np.linalg.norm(np.ptp(positions, axis=0))
            if max(readings) - min(readings) <= 0.050 and housing_travel <= 0.008:
                reading = round(float(np.median(readings)), 2)
                _state["reading"] = reading
                _state["measured"] = all(valid_support) and gentle and abs(reading - WEIGHT) <= 0.085
    else:
        _state["stable"].clear()
    if _state["loaded"] and bbottom - TOP > 0.009 and bvel[2] < -0.40:
        _state["wrong"] = True
    score = 0.25 if _state["loaded"] else 0.0
    if _state["measured"]:
        score = 0.6
        peg_supported = (np.linalg.norm(spos-SCALE) <= 0.033 and scale_held
                         and all(np.linalg.norm(hand-spos) >= 0.068 for hand in wrists.values()))
        if bbottom - TOP <= 0.006 and gentle and grip_clear and mass < 0.06 and peg_supported:
            score = 1.0
    _state["score"] = score if not _state["wrong"] else 0.0
    _draw_reading(base, _state["reading"])
    _state["step"] += 1
    if os.getenv("BAGSCALE_DEBUG") == "1" and _state["step"] % 5 == 0:
        print(f"BAGSCALE {_state['step']:03} mass={mass:.3f} reading={_state['reading']} "
              f"handle={math.degrees(bq):.1f} bottom={bbottom-TOP:.3f} steady={len(_state['stable'])} "
              f"hook_up={hook_up:.2f}N competing={foreign_contact:.2f}N supported={hooked} "
              f"bag_still={gentle} "
              f"clear={grip_clear} score={_state['score']:.2f} "
              f"handle_pos={np.round(handle-[0,0,TOP],3)} hook={np.round(spos+rotation(spose[3:7])@HOOK_BOWL-[0,0,TOP],3)} scale={np.round(spos-[0,0,TOP],3)} bag={np.round(bpos-[0,0,TOP],3)} "
              f"L={np.round(wrists['left']-[0,0,TOP],3)} R={np.round(wrists['right']-[0,0,TOP],3)}", flush=True)


def score(objects):
    return min(_state["score"], 0.6)


def success(objects):
    return _state["score"] == 1.0


def demo():
    right_rotation = _ry(-0.90) @ grip_rotation()
    grip_center = SCALE + [-0.003, 0.001, -0.010]
    closed, opened = -0.001, -0.035
    left_x = 0.045

    def hand(dt, angle, height, grip, text):
        contact = BAG + HINGE + _rx(math.radians(angle)) @ [left_x, 0.0, 0.041] + [0, 0, height]
        return Waypoint(dt, tuple(contact), grip, text)

    def r(dt, xyz, grip, text):
        return Waypoint(dt, tuple(xyz), grip, text, right_rotation)

    target_x = BAG[0]-0.006
    bag_y = BAG[1]
    lower = TOP + 0.253
    lift = TOP + (0.308 if os.getenv("BAGSCALE_LOW_LIFT") == "1" else 0.328)
    entry = TOP + 0.219
    right = [
        Waypoint(1.6, tuple(grip_center + [-0.107,0.003,0.044]), opened, "move to the side of the peg"),
        r(0.5, grip_center + [-0.093,0.004,0.035], opened, "turn toward the scale body"),
        r(1.20, grip_center, opened, "approach the scale's handgrip"),
        r(1.15, grip_center, closed, "grasp the scale"),
        r(1.30, grip_center + [0,0.083,0.002], closed, "slide the hanging loop off the peg"),
        r(1.3, (target_x, bag_y-0.039, lower), closed, "bring the metal hook beside the carry handle"),
        r(0.65, (target_x, bag_y-0.039, entry), closed, "lower the tip of the hook below the handle"),
        r(0.65, (target_x, bag_y-0.004, entry), closed, "slide the hook underneath"),
        r(0.8, (target_x, bag_y-0.004, lower+0.008), closed, "capture the handle in the hook"),
    ]
    if os.getenv("BAGSCALE_UNSTEADY") == "1":
        right += [r(0.8, (target_x, bag_y-0.004, lift), closed, "start lifting"),
                  r(0.75, (target_x, bag_y-0.004, lower), closed, "lower without waiting for a stable reading")]
    else:
        right += [
            r(1.6, (target_x, bag_y-0.004, lift), closed, "lift the bag gently"),
            r(3.6, (target_x, bag_y-0.004, lift), closed, "hold until the weight locks"),
            r(1.6, (target_x, bag_y-0.004, lower-0.020), closed, "gently lower the bag onto the bench"),
        ]
    right += [
        r(1.0, (target_x, bag_y-0.080, lower-0.028), closed, "unhook beside the bench"),
        r(1.65, grip_center + [0,0.057,0.021], closed, "bring the loop level with the peg"),
        r(1.40, grip_center + [0,0,0.018], closed, "slide the hanging loop onto the peg"),
        r(0.72, grip_center + [0,0,0.005], closed, "lower the ring onto the peg"),
        r(0.55, grip_center + [0,0,0.005], opened, "let the ring support the scale"),
        r(1.45, grip_center + [-0.092,0.0,0.035], opened, "withdraw"),
        r(1.00, grip_center + [-0.092,0.0,0.035], opened, "leave the lock reading visible"),
    ]
    left = [
        hand(2.3, -70.0, 0.061, -0.026, "approach the carry handle"),
        hand(1.1, -70.0, 0.002, -0.026, "arrive at the folded handle"),
        hand(0.6, -70.0, 0.002, -0.006, "pinch its free end"),
        hand(1.55, 0, 0.003, -0.006, "bring the handle upright"),
        hand(1.0, 0, 0.003, -0.006, "bring the carry handle to rest upright"),
        hand(0.47, 0, 0.003, -0.026, "let go of the handle before bringing in the hook"),
        Waypoint(1.6, (0.306,0.197,TOP+0.275), -0.026, "withdraw to leave the bag freely hanging"),
    ]
    return {"left": left, "right": right}
