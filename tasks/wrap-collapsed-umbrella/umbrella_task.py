"""RoboCurve prototype for the YAM station in Isaac Lab-Arena."""

from dataclasses import replace
import os
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import bodykit
from bodykit import Body, BoxShape, CylShape, as_numpy, on_reset, on_step, rotation
from geometry import (
    AXIS, CLOSED_RADIUS, COVER_LENGTH, FINGER_CLEARANCE, HANDLE_Y, LOOSE_RADIUS,
    PAD_LENGTH, TAB_LENGTH, TAB_PINCH, Outcome, PLEAT_ANGLES, PLEAT_HALF_WIDTH, Pad, SETTLE_STEPS, STRAP_COUNT, STRAP_SEGMENT,
    STRAP_THICK, STRAP_WIDTH, TABLE_Z, TIP_Y, open_strap, pad_mates, radial,
    engaged_cells, HOOK_FACE, mating_points, CELL_AREA, press_samples, score_outcome, strap_winding, strip_frame,
    FASTENED_AREA, set_down,
)
from joints import Connection, fastener_broken, recruit, row_group_intact, register, set_fastener
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.ik import matrix_to_quat_xyzw

bodykit.allow_matrix_grips()
SCENE = "station"
SUCCESS_HOLD_S = 0.05  # the scorer has already required one second of continuous release
SCORE_OVER = "end"
NAVY = (0.13, 0.23, 0.41)
WEBBING = (0.64, 0.43, 0.17)
DARK = (0.075, 0.09, 0.12)
STEEL = (0.50, 0.51, 0.55)


def _quaternion(frame):
    return tuple(map(float, matrix_to_quat_xyzw(frame)))


CORE = Body.build("umbrella", AXIS, 0.125, (
    CylShape((0, 0, 0), 0.012, 0.221, STEEL, axis="Y"),
    BoxShape((0, HANDLE_Y, 0), (0.032, 0.064, 0.024), DARK),
    BoxShape((0, HANDLE_Y, 0), (0.024, 0.064, 0.032), DARK),
    *(CylShape((x, HANDLE_Y, z), 0.004, 0.064, DARK, axis="Y")
      for x in (-0.012, 0.012) for z in (-0.012, 0.012)),
    CylShape((0, TIP_Y, 0), 0.010, 0.026, DARK, axis="Y"),
), friction=(1.15, 0.95), solver_iterations=32)


def _pleat(index, angle):
    direction = radial(angle)
    frame = strip_frame(direction)
    pos = AXIS + (LOOSE_RADIUS - 0.005) * direction
    # Overlapping pleat proxies preserve a continuous barrier to the ribbon
    # while allowing cloth layers to pass over each other as they gather.
    ignore = ("umbrella", *(f"pleat_{other}" for other in range(index)))
    colour = NAVY if index % 2 == 0 else (0.16, 0.29, 0.48)
    # Taper the stack of folded fabric towards its ferrule. The same shapes
    # are used for rendering and contacts; the strap crosses the full-width middle.
    panels = (
        BoxShape((-0.006, -0.084, 0), (0.008, 0.022, 0.015), colour),
        BoxShape((-0.003, -0.063, 0), (0.009, 0.020, 0.021), colour),
        BoxShape((0, 0.021, 0), (0.010, COVER_LENGTH - 0.042, 2 * PLEAT_HALF_WIDTH), colour),
    )
    pleat = Body.build(f"pleat_{index}", pos, 0.0025, panels, start_xyzw=_quaternion(frame), ignore=ignore, contact_offset=0.001,
        friction=(0.38, 0.28), solver_iterations=32)
    register(Connection("spring_fold", CORE, pleat, pos, frame, 0.024, 42.0 if index == 0 else 21.0))
    return pleat


PLEATS = tuple(_pleat(i, a) for i, a in enumerate(PLEAT_ANGLES))
START_STRAP = open_strap()


# Centres of the two sewn fastener layers in the ribbon body frames.
BASE_LAYER = np.array([-0.006, 0.0, STRAP_THICK])
TIP_LAYER = np.array([0.0, 0.0, -STRAP_THICK])

def _make_strap():
    links = []
    for i, (a, b) in enumerate(zip(START_STRAP, START_STRAP[1:])):
        frame = strip_frame(b - a)
        name = "strap_tip" if i == STRAP_COUNT - 1 else f"strap_{i:02d}"
        # Small bevel gaps permit adjacent links to articulate. The stock strap has no added handle.
        strap_color = (0.17, 0.27, 0.43) if i in (0, STRAP_COUNT - 1) else WEBBING
        friction = (1.15, 0.90) if i >= STRAP_COUNT - 2 else (0.58, 0.40)
        layers = []
        if i < 4 or i == STRAP_COUNT - 1:
            centre = TIP_LAYER if i == STRAP_COUNT-1 else BASE_LAYER
            layers.append(BoxShape(tuple(centre), (PAD_LENGTH, STRAP_WIDTH, STRAP_THICK), DARK))
        link = Body.build(name, (a + b) / 2, 0.0025, (
            *layers,
            BoxShape((0, 0, 0), (STRAP_SEGMENT - 0.0006, STRAP_WIDTH, STRAP_THICK), strap_color),
        ), start_xyzw=_quaternion(frame), contact_offset=0.0006, friction=friction,
            solver_iterations=32, damping=(0.12, 0.14))
        register(Connection("seam" if i == 0 else "bend", links[-1] if links else PLEATS[0], link, a, frame))
        links.append(link)
    return tuple(links)


STRAP = _make_strap()
# A separate articulated extension can fold over the hook without levering the
# sewn panel away from its receiver during gripper withdrawal.
def _tail_links():
    direction = START_STRAP[-1]-START_STRAP[-2]
    frame = strip_frame(direction)
    direction = frame[:,0]
    anchor = START_STRAP[-1].copy()
    parent = STRAP[-1]
    extra = []
    for index in range(2):
        length = TAB_LENGTH/2
        body = Body.build(f"tail_fabric_{index}",anchor+length/2*direction,.0006,
            (BoxShape((0.,0.,0.),(length-.0006,STRAP_WIDTH,STRAP_THICK),WEBBING),
             BoxShape(tuple(TIP_LAYER),(PAD_LENGTH,STRAP_WIDTH,STRAP_THICK),DARK)),
            start_xyzw=_quaternion(frame),friction=(.58,.40),contact_offset=.0006,
            solver_iterations=32,damping=(.12,.14))
        register(Connection("bend",parent,body,anchor,frame))
        extra.append(body)
        parent = body
        anchor = anchor+length*direction
    return tuple(extra)
EXTENSION = _tail_links()
HOOKS = (STRAP[-1],*EXTENSION)
RECEIVERS = STRAP[:4]
PAIRS = tuple((base,tip) for tip in HOOKS for base in RECEIVERS)
# A flat 4 mm tabletop skin provides a rigid-body target for the force sensor.
# The station table itself is a static triangle mesh.
TABLE_SURFACE = Body.build("landing_surface", (AXIS[0], 0., TABLE_Z + 0.002), 1.0,
    (BoxShape((0., 0., 0.), (0.50, 0.55, 0.004), (0.45, 0.45, 0.44)),),
    kinematic=True, contact_offset=0.0006)
OBJECTS = [CORE, *PLEATS, *STRAP, *EXTENSION, TABLE_SURFACE]
MOVING = (CORE, *PLEATS, *STRAP, *EXTENSION)
STRAP_BODIES = tuple(range(1 + len(PLEATS), len(MOVING)))  # indices of the strap links in MOVING
NEGATIVE = os.getenv("UMBRELLA_NEGATIVE", "")  # scripted failure routes for the scorer check; read by the demo only
trial = None
# What the scripted demonstrator may read: simulator poses, contact forces and the fastener
# model's own engagement state. It is refreshed after each control and never holds a score.
sensed = {}
demo_memory = {}  # the demonstrator's own notes between its two arms
CONTROLLER_KEYS = ("handle", "acquired", "placement_started", "carry_requested", "core_frame", "centre",
                   "landed", "handle_load", "landing_q", "table_load", "receiver", "normal_pressure",
                   "handle_turn", "tail", "fingers", "free_patch", "rock_contact", "steps", "jaw_force",
                   "tacked", "fastened", "patch_outward")


def _pad(state):
    return Pad(np.asarray(state.pos), rotation(state.quat_xyzw))


def _fastener_pads(states, index=0):
    pads = []
    for spec, offset in zip(PAIRS[index],(BASE_LAYER,TIP_LAYER)):
        ribbon = _pad(states[spec.name])
        pads.append(Pad(ribbon.pos + ribbon.rotation @ offset, ribbon.rotation))
    return pads

def _finger_samples(env):
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    points = []
    for name in ("left_robot", "right_robot"):
        robot = base.scene[name]
        link = list(robot.body_names).index("gripper")
        pose = as_numpy(robot.data.body_link_pose_w[0, link])
        frame = rotation(pose[3:7])
        positions = as_numpy(robot.data.joint_pos[0])
        joint_names = list(robot.joint_names)
        for sign, joint in ((1, "joint7"), (-1, "joint8")):
            gap = abs(positions[joint_names.index(joint)])
            for along in (-0.020, 0.0, PAD_TIP_PAST_TCP):
                offset = np.asarray(TCP_OFFSET) + [sign * gap, 0, along]
                points.append(pose[:3] + frame @ offset - origin)
    return np.asarray(points)


def _patch_distance(fingers, pad):
    local = (fingers - pad.pos) @ pad.rotation
    half_size = 0.5 * np.array([STRAP_SEGMENT, STRAP_WIDTH, STRAP_THICK])
    outside = np.maximum(np.abs(local) - half_size, 0.0)
    return np.linalg.norm(outside, axis=1).min()


def _measure(states, fingers, fastened):
    axis = states[CORE.name]
    inverse = rotation(axis.quat_xyzw).T
    local = lambda world: (world - axis.pos) @ inverse.T
    # Check each pleat, not just the flap under the strap. The width term
    # includes the outside corners of each collision box.
    pleats = np.asarray([local(states[spec.name].pos) for spec in PLEATS])
    r = np.linalg.norm(pleats[:, [0, 2]], axis=1)
    radius = float(np.max(np.hypot(r + 0.005, PLEAT_HALF_WIDTH)))
    strip_points = np.asarray([local(states[spec.name].pos) for spec in (*STRAP,*EXTENSION)])
    base, tip = _fastener_pads(states)
    first = states[STRAP[0].name]
    last = states[EXTENSION[-1].name]
    sewn_edge = local(first.pos - 0.5 * STRAP_SEGMENT * rotation(first.quat_xyzw)[:, 0])
    free_edge = local(last.pos + 0.25 * TAB_LENGTH * rotation(last.quat_xyzw)[:, 0])
    winding = strap_winding(np.vstack([sewn_edge, strip_points, free_edge]), np.zeros(3))
    still = all(states[spec.name].resting(max_speed=0.022, max_spin=1.0) for spec in MOVING)
    held = False
    clear = False
    if fingers is not None:
        left = local(fingers[:6]).reshape(2, 3, 3)
        handle = left - [0, HANDLE_Y, 0]
        radii = np.linalg.norm(handle[:, :, [0, 2]], axis=2)
        grip = (np.abs(handle[:, :, 1]) <= 0.026) & (radii <= 0.022)
        opposed = handle[0, 1, [0, 2]] @ handle[1, 1, [0, 2]] < 0
        distances = []
        bottom = np.inf
        for spec in MOVING:
            body = states[spec.name]
            basis = rotation(body.quat_xyzw)
            low, high = spec.local_bounds()
            corners = np.where(BOX_CORNERS, high, low)
            bottom = min(bottom, float((corners @ basis[2] + body.pos[2]).min()))
            points = (fingers[6:] - body.pos) @ basis
            distances.append(np.linalg.norm(points - np.clip(points, low, high), axis=1).min())
        held = bool(grip.any(axis=1).all() and opposed and bottom > TABLE_Z + 0.020)
        clear = min(distances) >= FINGER_CLEARANCE
    return Outcome(radius, winding, pad_mates(base, tip), fastened, clear, still, held)


BOX_CORNERS = np.indices((2, 2, 2)).reshape(3, -1).T.astype(bool)


def _grip_sensors(env):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Usd, UsdPhysics
    scene = env.unwrapped.scene
    stage = get_current_stage()
    contacts = []
    grips = []
    everything = [f"/World/envs/env_0/{spec.name}/body" for spec in MOVING]
    for arm, names in (("left_robot", ("umbrella",)), ("right_robot", ("strap_11", "strap_tip"))):
        robot = scene[arm]
        root = robot.root_physx_view.prim_paths[0]
        tips = sorted(str(prim.GetPath()) for prim in Usd.PrimRange(stage.GetPrimAtPath(root))
                      if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in ("tip_left","tip_right"))
        if len(tips) != 2:
            raise RuntimeError(f"Expected two gripper pads under {root}, found {tips}")
        bodies = [f"/World/envs/env_0/{name}/body" for name in names]
        contacts.append(robot._physics_sim_view.create_rigid_contact_view(
            bodies, filter_patterns=[tips for _ in bodies], max_contact_data_count=256))
        # Either gripper's two pads against every part of the umbrella: holding, pressing and release
        # are judged the same way whichever hand does them.
        grips.append(robot._physics_sim_view.create_rigid_contact_view(
            everything, filter_patterns=[tips for _ in everything]))
    base = "/World/envs/env_0"
    hook_paths = [f"{base}/{spec.name}/body" for spec in HOOKS]
    receiver_paths = [f"{base}/{spec.name}/body" for spec in RECEIVERS]
    patch = robot._physics_sim_view.create_rigid_contact_view(
        hook_paths,filter_patterns=[receiver_paths for _ in HOOKS],max_contact_data_count=512)
    tables = [f"{base}/{TABLE_SURFACE.name}/body"]
    bodies = [f"{base}/{spec.name}/body" for spec in (CORE,*PLEATS)]
    support = robot._physics_sim_view.create_rigid_contact_view(
        bodies, filter_patterns=[tables for _ in bodies])
    return contacts, patch, support, grips


def _observe_physics_motion(env):
    scene = env.unwrapped.scene
    if getattr(scene, "_umbrella_observes_motion", False):
        return
    origin = as_numpy(scene.env_origins[0])
    sensors, patch, support, grips = _grip_sensors(env)
    original = scene.update
    def update(dt):
        original(dt)
        # Integrate both opposing pad loads over the 12 physics substeps.
        # The right pad may bear on either the ribbon or its sewn tip.
        if trial is not None:
            table_force = support.get_contact_force_matrix(float(dt))
            table_force = as_numpy(table_force) if hasattr(table_force,"detach") else table_force.numpy()
            trial["support_impulse"] = trial.get("support_impulse",0.) + dt*abs(table_force[...,2].sum())
            grip_load = []
            for view in grips:
                native = view.get_contact_force_matrix(float(dt))
                force = native.numpy() if not hasattr(native, "detach") else as_numpy(native)
                grip_load.append(np.linalg.norm(force, axis=-1).reshape(len(MOVING), 2))
            grip_load = np.asarray(grip_load)  # arm, umbrella body, pad
            trial["grip_impulse"] = trial.get("grip_impulse", np.zeros((2,2))) + dt*grip_load.sum(axis=1)
            trial["strap_grip_impulse"] = (trial.get("strap_grip_impulse", np.zeros((2,2)))
                                           + dt*grip_load[:, STRAP_BODIES].sum(axis=1))
            loads = []
            for side, view in enumerate(sensors):
                if side == 0:
                    data = view.get_contact_data(float(dt))
                    def array(value):
                        return value.numpy() if not hasattr(value, "detach") else as_numpy(value)
                    magnitudes, points, _, _, counts, starts = map(array, data)
                    pose = as_numpy(scene["umbrella"].data.root_link_pose_w[0])
                    local = (points.reshape(-1,3)-pose[:3]) @ rotation(pose[3:7])
                    # Only count contact on the exterior of the 64 mm long handle.
                    handle = ((np.abs(local[:,1]-HANDLE_Y) <= 0.033)
                              & (np.hypot(local[:,0],local[:,2]) >= 0.0125))
                    pad_loads = []
                    for count, start in zip(counts.ravel(), starts.ravel()):
                        indices = slice(int(start), int(start+count))
                        pad_loads.append(np.abs(magnitudes.ravel()[indices][handle[indices]]).sum())
                    loads.append(pad_loads)
                    continue
                native = view.get_contact_force_matrix(float(dt))
                force = native.numpy() if not hasattr(native, "detach") else as_numpy(native)
                loads.append(np.linalg.norm(force, axis=-1).reshape(-1,2).sum(axis=0))
            trial["pad_impulse"] = trial.get("pad_impulse", np.zeros((2,2))) + dt*np.asarray(loads)
            trial["pad_span"] = trial.get("pad_span", 0.) + dt
            if "receiver" in trial:
                magnitudes, points, normals, _, counts, starts = map(array,patch.get_contact_data(float(dt)))
                force = np.zeros(len(PAIRS))
                def live_pad(spec,offset):
                    pose = as_numpy(scene[spec.name].data.root_link_pose_w[0])
                    basis = rotation(pose[3:7])
                    return Pad(pose[:3]-origin+basis@offset,basis)
                for k,(loop,hook) in enumerate(PAIRS):
                    start, count = int(starts.ravel()[k]), int(counts.ravel()[k])
                    if not count:
                        continue
                    receiver = live_pad(loop,BASE_LAYER)
                    tip = live_pad(hook,TIP_LAYER)
                    sl = slice(start,start+count)
                    hit = points.reshape(-1,3)[sl]-origin
                    nr = normals.reshape(-1,3)[sl] @ receiver.rotation[:,2]
                    bp = (hit-receiver.pos) @ receiver.rotation
                    tp = (hit-tip.pos) @ tip.rotation
                    on_faces = ((abs(bp[:,0])<=PAD_LENGTH/2) & (abs(tp[:,0])<=PAD_LENGTH/2)
                               & (abs(bp[:,1])<=STRAP_WIDTH/2) & (abs(tp[:,1])<=STRAP_WIDTH/2)
                               & (abs(bp[:,2]-STRAP_THICK/2)<=0.0015)
                               & (abs(tp[:,2]+STRAP_THICK/2)<=0.0015) & (abs(nr)>0.85))
                    loads = np.abs(magnitudes.ravel()[sl]*nr)*on_faces
                    if k == trial["receiver_index"] and loads.max() > 0.30:
                        best = int(loads.argmax())
                        trial["rock_contact"] = (trial["steps"],hit[best].copy())
                    force[k] = np.abs(magnitudes.ravel()[sl]*nr)[on_faces].sum()
                trial["patch_impulse"] = trial.get("patch_impulse", np.zeros(len(PAIRS))) + force*dt
        old = (trial or {}).get("outcome")
        if old is None or not (old.fingers_clear and old.latched):
            trial.pop("substep_prev", None)
            return
        poses = np.stack([as_numpy(scene[body.name].data.root_com_pose_w[0]) for body in MOVING])
        poses = poses.astype(float)
        previous = trial.get("substep_prev")
        trial["substep_prev"] = poses
        if previous is None:
            return
        speed = np.linalg.norm(poses[:, :3] - previous[:, :3], axis=1) / dt
        q, p = poses[:, 3:], previous[:, 3:]
        q = q / np.linalg.norm(q, axis=1)[:, None]
        p = p / np.linalg.norm(p, axis=1)[:, None]
        p = np.where((q*p).sum(axis=1)[:, None] < 0, -p, p)
        spin = 4 * np.arctan2(np.linalg.norm(q-p, axis=1), np.linalg.norm(q+p, axis=1)) / dt
        trial["substep_v"] = max(trial.get("substep_v", 0), float(speed.max()))
        trial["substep_w"] = max(trial.get("substep_w", 0), float(spin.max()))
        trial["substep_n"] = trial.get("substep_n", 0) + 1
        trial["observed_duration"] = trial.get("observed_duration", 0.0) + dt
    scene.update = update
    scene._umbrella_observes_motion = True


@on_reset
def reset_trial():
    global trial
    set_fastener()
    trial = dict(fastened=False, mating_steps=0, released_steps=0, steps=0, outcome=None,
                 roll=0.0, previous_angle=0.0, dropped=False)
    sensed.clear()
    demo_memory.clear()


@on_step
def update_trial(env):
    _observe_physics_motion(env)
    n = trial.pop("substep_n", 0)
    span = trial.pop("observed_duration", 0.0)
    peak_v = trial.pop("substep_v", 0.0)
    peak_w = trial.pop("substep_w", 0.0)
    complete = n == env.unwrapped.cfg.decimation and np.isclose(span, env.unwrapped.step_dt)
    observed_still = complete and peak_v < 0.022 and peak_w < 1.0
    from yam_arena.scene import object_states

    states = object_states(env, [spec.name for spec in MOVING])
    fingers = _finger_samples(env)
    outcome = _measure(states, fingers, trial["fastened"])
    core = states[CORE.name]
    basis = rotation(core.quat_xyzw)
    pairs = [_fastener_pads(states,k) for k in range(len(PAIRS))]
    index = trial.get("receiver_index",0)
    if not trial["fastened"] and not trial.get("tacked"):
        loads = trial.get("patch_impulse",np.zeros(len(PAIRS)))/max(trial.get("pad_span",0.),1.e-9)
        def mismatch(k):
            receiver,tip=pairs[k]
            return np.linalg.norm(tip.pos-receiver.pos)+.018*(1.-receiver.rotation[:,2]@tip.rotation[:,2])
        loaded = [k for k,(receiver,tip) in enumerate(pairs) if loads[k]>=.15 and len(mating_points(receiver,tip))]
        best = min(loaded or range(len(PAIRS)),key=mismatch)
        if best != index and (loaded or mismatch(best)+.002<mismatch(index)):
            index=best
            trial["mating_steps"]=trial["tack_steps"]=0
    base,tail=pairs[index]
    trial["receiver_index"]=index
    outcome=replace(outcome,mated=pad_mates(base,tail))
    grip = _pad(states[STRAP[-2].name])
    trial.update(core_frame=basis.copy(), handle=core.pos + basis @ [0, HANDLE_Y + 0.002, 0], tail=grip,
                 receiver=base, free_patch=tail, centre=core.pos.copy(), fingers=fingers)
    jaws = (fingers[6:] - grip.pos) @ grip.rotation
    inside = np.all(np.abs(jaws.reshape(2, 3, 3)) <= [0.011, 0.013, 0.0045], axis=2)
    guided = bool(inside.any(axis=1).all() and jaws[1, 2] * jaws[4, 2] <= 0)
    span = max(trial.pop("pad_span", 0.), 1e-9)
    forces = trial.pop("pad_impulse", np.zeros((2,2))) / span
    # Mean load of each gripper pad (arm x pad) on any part of the umbrella, and on the strap alone.
    grip_all = trial.pop("grip_impulse", np.zeros((2,2))) / span
    grip_strap = trial.pop("strap_grip_impulse", np.zeros((2,2))) / span
    pinched = bool((grip_all > 0.1).all(axis=1).any())
    strap_pressed = bool((grip_strap > 0.1).any() or (grip_strap > 0.04).all(axis=1).any())
    pair_loads = trial.pop("patch_impulse", np.zeros(len(PAIRS))) / span
    pressure_load = pair_loads[index]
    trial["normal_pressure"] = pressure_load
    trial["jaw_force"] = forces[1].copy()
    trial["handle_load"] = min(forces[0])
    # Pads must actually squeeze the named body, in the intended grip region.
    # Express positions in the shaft frame to distinguish the handle from the canopy.
    airborne = all(states[spec.name].pos[2] + (np.where(BOX_CORNERS,*reversed(spec.local_bounds())) @ rotation(states[spec.name].quat_xyzw)[2]).min() > TABLE_Z+0.020 for spec in MOVING)
    # Held: both pads of either gripper bear at least 0.1 N on the umbrella.
    outcome = replace(outcome, handle_held=pinched)
    left_holding = bool(airborne and min(forces[0]) > 0.1)  # the scripted demo's handle hand, for its own turn count
    guided = bool(min(forces[1])>0.04 and np.linalg.norm(fingers[[7,10]].mean(axis=0)-grip.pos)<0.024)
    previous = trial.get("basis", basis)
    relative = previous.T @ basis
    twist = np.arctan2(relative[0, 2] - relative[2, 0], relative[0, 0] + relative[2, 2])
    if not trial["steps"]:
        trial["started_clear"] = _patch_distance(fingers[6:], grip) > 0.025
    # Controller observation for the scripted demonstrator only: how far its handle hand has turned
    # the shaft while its other hand guides the strap. Neither the score nor the fastener reads it.
    if "completed_turn" in trial:
        trial["handle_turn"] = trial["completed_turn"]
    elif not trial["fastened"] and left_holding and guided and trial["started_clear"]:
        trial.setdefault("acquired", trial["steps"])
        trial["net_turn"] = trial.get("net_turn", 0.0) - twist
        trial["handle_turn"] = max(trial.get("handle_turn",0.), trial["net_turn"])
    elif (not trial["fastened"] and left_holding and trial.get("handle_turn",0.) >= 2*np.pi
          and max(forces[1]) > 0.1):
        pass
    elif not trial["fastened"]:
        trial["net_turn"] = trial["handle_turn"] = 0.0
        trial.pop("acquired", None)
    if trial.get("handle_turn",0.) >= 2*np.pi:
        trial.setdefault("completed_turn",trial["handle_turn"])
    trial["basis"] = basis.copy()
    trial["guided"] = guided
    if outcome.mated and not trial.get("saw_mating"):
        print(f"[fastener] opposed patches overlapped at step {trial['steps']}", flush=True)
        trial["saw_mating"] = True
    if not trial["fastened"]:
        # A fingertip of either gripper at the strap end or its hook patch.
        touching = min(_patch_distance(fingers, grip), _patch_distance(fingers, tail)) <= 0.006
        # A loaded narrow row can adhere first. It carries only the measured
        # row's shear and peel capacity while the arm rolls the wider face down.
        sites = mating_points(base,tail)
        if trial.get("cell_panel") != index:
            trial["cell_panel"] = index
            trial["hook_cells"] = {}
        verified_touch = touching and pressure_load >= .15 and strap_pressed
        trial["hook_cells"] = engaged_cells(trial.get("hook_cells",{}) if trial.get("tacked") else {},base,tail,
                                            sites if verified_touch else [])
        retained = len(trial["hook_cells"])*CELL_AREA
        delta = base.rotation.T @ (tail.pos-base.pos)
        retained_contact = (retained>=.000048 and abs(delta[0])<=PAD_LENGTH-.008
                            and abs(delta[1])<=STRAP_WIDTH*.4
                            and base.rotation[:,2]@tail.rotation[:,2]>.90)
        if trial.get("tacked") and retained_contact:
            outcome = replace(outcome,mated=True)
        if trial.get("tacked"):
            raw = trial["tack_points"]
            gap = np.linalg.norm((base.pos + raw[0] @ base.rotation.T) -
                                 (tail.pos + raw[1] @ tail.rotation.T),axis=1)
            if fastener_broken() or gap.min()>0.003:
                set_fastener()
                trial["tacked"] = False
                trial["tack_steps"] = 0
        if not trial.get("tacked") and not outcome.mated:
            dx,dy,_ = base.rotation.T @ (tail.pos-base.pos)
            touching_row = (len(sites)*CELL_AREA >= 0.000012 and abs(dx)<PAD_LENGTH-0.008
                            and abs(dy)<STRAP_WIDTH*0.4)
            trial["tack_steps"] = press_samples(trial.get("tack_steps",0),mated=touching_row,
                held=strap_pressed,touching=touching,normal_load=pressure_load)
            if trial["tack_steps"]>=3:
                set_fastener(_pad(states[PAIRS[index][0].name]),_pad(states[PAIRS[index][1].name]),base,tail,index,sites)
                trial["tack_points"] = ((sites-base.pos)@base.rotation,(sites-tail.pos)@tail.rotation)
                trial["tacked"] = True
                print("[partial row]",trial["steps"],"area_mm2",len(sites)*CELL_AREA*1e6,flush=True)
        # Engagement needs only a loaded press of overlapping, aligned patches by a gripper for three
        # controls, however the strap was wound and whichever hand presses.
        trial["mating_steps"] = press_samples(trial["mating_steps"], mated=outcome.mated,
                                              held=strap_pressed, touching=touching, normal_load=pressure_load)
        if trial["mating_steps"] >= 3:
            indices = list(trial["hook_cells"])
            world = (tail.pos + HOOK_FACE[indices]@tail.rotation.T
                     if indices else mating_points(base,tail))
            set_fastener(_pad(states[PAIRS[index][0].name]), _pad(states[PAIRS[index][1].name]), base, tail, index, world, seated=True)
            print(f"[fastener] engaged at step {trial['steps']}", flush=True)
            trial["fastened"] = True
            trial["retained_area"] = len(world)*CELL_AREA
            trial["tacked"] = False
            trial["patch_outward"] = base.rotation[:, 2].copy()
            trial["release_joints"] = {}
            for left, name in ((True, "left_robot"), (False, "right_robot")):
                robot = env.unwrapped.scene[name]
                idx = [list(robot.joint_names).index(f"joint{i}") for i in range(1, 7)]
                trial["release_joints"][left] = as_numpy(robot.data.joint_pos[0])[idx].copy()
            trial["receiver_hooks"] = (world - base.pos) @ base.rotation
            trial["tip_hooks"] = (world - tail.pos) @ tail.rotation
            trial["groups"]={(index,0):(trial["receiver_hooks"],trial["tip_hooks"])}
            trial["covered"]={index:set(indices)}
            trial["row_samples"]={}
            print(f"[fastener] seated area {len(world)*CELL_AREA*1e6:.1f} mm2 pair {index}",flush=True)
            trial["peel_steps"] = 0
            trial.setdefault("release_at", trial["steps"])
            trial["mating_steps"] = 0
    else:
        groups=trial["groups"]
        for k,(receiver,hook) in enumerate(pairs):
            covered=trial["covered"].get(k,set())
            first=4 if (k,0) in groups else 0
            if (k,first) in groups or pair_loads[k]<.15:
                trial["row_samples"].pop(k,None)
                continue
            sites=mating_points(receiver,hook)
            candidates=engaged_cells({},receiver,hook,sites)
            prev=trial["row_samples"].get(k,{})
            counts={i:prev.get(i,0)+1 for i in candidates if i not in covered}
            trial["row_samples"][k]=counts
            ids=[i for i,c in counts.items() if c>=3]
            if len(ids)*CELL_AREA < .000012:
                continue
            world=hook.pos+HOOK_FACE[ids]@hook.rotation.T
            recruit(_pad(states[PAIRS[k][0].name]),_pad(states[PAIRS[k][1].name]),receiver,world,k,first)
            groups[k,first]=((world-receiver.pos)@receiver.rotation,(world-hook.pos)@hook.rotation)
            trial["covered"].setdefault(k,set()).update(ids)
            print(f"[fastener] recruited {len(ids)*CELL_AREA*1e6:.1f} mm2 pair {k} at {trial['steps']}",flush=True)
        retained_area=0.
        for (k,first),(anchors,sites) in groups.items():
            if not row_group_intact(k,first):
                continue
            receiver,hook=pairs[k]
            differences=(hook.pos+sites@hook.rotation.T-receiver.pos-anchors@receiver.rotation.T)@receiver.rotation
            attached=(differences[:,2]<=.0017)&(np.linalg.norm(differences[:,:2],axis=1)<=.003)
            if receiver.rotation[:,2]@hook.rotation[:,2]>.90:
                retained_area+=np.count_nonzero(attached)*CELL_AREA
        trial["peel_steps"] = 0 if retained_area>0 else trial["peel_steps"]+1
        trial["retained_area"]=retained_area
        outcome=replace(outcome,mated=bool(retained_area>=FASTENED_AREA))
        if fastener_broken() or trial["peel_steps"]>=2:
            print(f"[fastener] released at step {trial['steps']} retained_mm2={retained_area*1e6:.1f}",flush=True)
            set_fastener()
            trial["fastened"]=False
    outcome = replace(outcome, latched=trial["fastened"])
    if trial["steps"] == 640:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import PhysxSchema
        stage = get_current_stage()
        for spec in MOVING:
            prim = stage.GetPrimAtPath(f"/World/envs/env_0/{spec.name}/body")
            api = PhysxSchema.PhysxRigidBodyAPI(prim)
            # Limit positional penetration correction for the millimetre-scale
            # strips. This is a solver bias limit, not a body velocity limit.
            api.CreateMaxDepenetrationVelocityAttr(0.060)
            api.CreateSolverPositionIterationCountAttr(96)
            api.CreateSolverVelocityIterationCountAttr(2)
        trial["convergence_check"] = True
    table_load = trial.pop("support_impulse",0.) / span
    bottom = min(states[spec.name].low[2] for spec in (CORE,*PLEATS))
    supported = table_load > 0.6 and abs(bottom-TABLE_Z) <= 0.018
    trial["bottom"] = bottom
    trial["table_load"] = table_load
    if pinched:
        # Whether the umbrella already rested on the table while a gripper still held it.
        trial["rested_when_released"] = bool(supported)
    placed = set_down(supported, trial.get("rested_when_released", False), pinched)
    trial["dropped"] = bool(supported and not pinched and not trial.get("rested_when_released", False))
    if supported and trial.get("placement_started") and "landing_q" not in trial:
        trial["landing_q"] = {}
        for left,name in ((True,"left_robot"),(False,"right_robot")):
            robot=env.unwrapped.scene[name]
            jids=[list(robot.joint_names).index(f"joint{i}") for i in range(1,7)]
            trial["landing_q"][left]=as_numpy(robot.data.joint_pos[0])[jids].copy()

    # Controller phases for the scripted demonstrator (fastener state, table load and pad loads only).
    if trial["fastened"] and trial.get("retained_area", 0.) >= FASTENED_AREA and min(forces[0]) > 0.1:
        trial["placement_started"] = True
    if trial.get("placement_started") or (trial["steps"] >= 1340 and forces.max() > 0.1):
        trial["carry_requested"] = True
    if supported and trial.get("carry_requested") and forces.max() > 0.1:
        trial["landed"] = True
    left_clear = True
    for spec in MOVING:
        state = states[spec.name]
        points = (fingers[:6]-state.pos) @ rotation(state.quat_xyzw)
        low,high = spec.local_bounds()
        if np.linalg.norm(points-np.clip(points,low,high),axis=1).min() < FINGER_CLEARANCE:
            left_clear = False
            break
    # Set down: resting on the table now, and already resting on it when the last gripper let go.
    outcome = replace(outcome,table_supported=bool(supported),set_down=placed,
                      both_clear=bool(left_clear and outcome.fingers_clear and grip_all.max()<0.10))
    native_still = outcome.still
    outcome = replace(outcome, still=bool(observed_still))
    ready_for_release = score_outcome(outcome, SETTLE_STEPS) == 1.0
    if ready_for_release:
        trial["hold_v"] = max(trial.get("hold_v", 0.0), peak_v)
        trial["hold_w"] = max(trial.get("hold_w", 0.0), peak_w)
        if trial["released_steps"] == SETTLE_STEPS - 1:
            print(f"[verified hold] samples={SETTLE_STEPS*n} bodies={len(MOVING)} "
                  f"v_max={trial['hold_v']:.7f} w_max={trial['hold_w']:.7f} "
                  f"native_still={native_still}", flush=True)
    else:
        trial["hold_v"] = trial["hold_w"] = 0.0
    trial["released_steps"] = trial["released_steps"] + 1 if ready_for_release else 0
    trial["outcome"] = outcome
    trial["steps"] += 1
    if os.getenv("UMBRELLA_TRACE") and trial["steps"] % 20 == 0:
        handle_pos = trial["handle"]
        patch_offset = np.round(1000*(base.rotation.T @ (tail.pos-base.pos))).astype(int).tolist()
        print(f"[handheld] {trial['steps']} handle_z={trial['handle'][2]-TABLE_Z:.3f} "
              f"forces={forces.round(1).tolist()} press_N={pressure_load:.2f} held={outcome.handle_held} guided={guided} turn={np.degrees(trial.get('handle_turn',0.)):.1f} "
              f"winding={np.degrees(outcome.winding):.1f} radius={1000*outcome.pleat_radius:.1f} "
              f"latched={outcome.latched} clear={outcome.fingers_clear} table_N={table_load:.2f} both_clear={outcome.both_clear} set_down={outcome.set_down} dropped={trial['dropped']} score={score_outcome(outcome, trial['released_steps']):.2f} "
              f"patch_mm={patch_offset} alignment={np.dot(base.rotation[:,2],tail.rotation[:,2]):.2f} left_mm={np.round(1000*(fingers[[1,4]].mean(axis=0)-handle_pos)).astype(int).tolist()} "
              f"right_mm={np.round(1000*(fingers[[7,10]].mean(axis=0)-grip.pos)).astype(int).tolist()}", flush=True)
    sensed.clear()
    sensed.update({key: trial[key] for key in CONTROLLER_KEYS if key in trial})


def score(objects):
    if trial is None or trial["outcome"] is None:
        return score_outcome(_measure(objects, None, False), 0)
    return score_outcome(trial["outcome"], trial["released_steps"])


def success(objects):
    return score(objects) == 1.0


def _w(seconds, pos, finger, label, frame):
    return Waypoint(seconds, tuple(map(float, pos)), finger, label, frame)


def _grip_frame(close_axis, approach):
    tool_axis = np.asarray(approach, dtype=float)
    tool_axis -= close_axis * np.dot(tool_axis, close_axis)
    tool_axis /= np.linalg.norm(tool_axis)
    return np.column_stack((close_axis, np.cross(tool_axis, close_axis), tool_axis))


def _tip_grip(angle, approach):
    return _grip_frame(-radial(angle), approach)


# Both hands act through joint commands and contact. Poses below are in the table frame.
def _phase(t, start, duration):
    return ik.smoothstep(float(np.clip((t - start) / duration, 0, 1)))


def _held_action(plan, t):
    left = plan.base_pos[1] > 0
    if not hasattr(plan, "_handoff"):
        plan.q = (INITIAL_GRASP_Q if left else APPROACH_Q).copy()
        plan._handoff = True
    if not sensed or "handle" not in sensed:
        return np.r_[plan.q, 0.0 if left else APPROACH_OPEN], "hold handle; open strap hand"
    if NEGATIVE == "none":
        return np.r_[plan.q, 0.0 if left else APPROACH_OPEN], "negative: hold the handle and leave the strap"
    acquired = sensed.get("acquired")
    if not left and (NEGATIVE == "unfastened" and sensed["handle_turn"] >= 2*np.pi or hasattr(plan, "_skip")):
        # Negative route: let go of the wound strap without pressing the fastener.
        if not hasattr(plan, "_skip"):
            tcp, frame = ik.tcp_pose(plan.q, plan.base_pos)
            plan._skip = (t, tcp.copy(), frame.copy(), getattr(plan, "_last_opening", 0.))
        start, anchor, frame, opening = plan._skip
        finger = opening + (1. - opening)*_phase(t, start, 1.0)
        target = anchor - 0.100*_phase(t, start + 0.6, 1.8)*frame[:, 2]
        q, _, _ = ik.solve(target, matrix_to_quat_xyzw(frame), plan.q, base_pos=plan.base_pos, rot_weight=0.18, iters=48)
        plan.q += np.clip(q - plan.q, -0.085, 0.085)
        return np.r_[plan.q, finger], "negative: let go of the strap without pressing it"
    if left and NEGATIVE == "drop" and sensed.get("placement_started"):
        # Negative route: open the handle hand in mid-air once the strap is fastened.
        if not hasattr(plan, "_drop"):
            plan._drop = t
        # Wait until the strap hand has opened and backed away, then open the handle hand in mid-air.
        return np.r_[plan.q, _phase(t, plan._drop + 3.0, 0.5)], "negative: drop the fastened umbrella"
    if (sensed.get("placement_started") or sensed.get("carry_requested")) and not (NEGATIVE == "drop" and not left):
        if not hasattr(plan,"_carry"):
            tcp, frame = ik.tcp_pose(plan.q,plan.base_pos)
            basis = sensed["core_frame"]
            centre = sensed["centre"]
            plan._carry = (t,plan.q.copy(),basis.T@(tcp-centre),basis.T@frame)
            plan._carry_opening = getattr(plan,"_last_opening",0.)
            plan._placement_z = tcp[2]
            if left:
                _, neutral = ik.tcp_pose(ROLL_PATH[36],plan.base_pos)
                hover = np.array([0.255,0.130,TABLE_Z+0.085])
                goal,_,_ = ik.solve(hover,matrix_to_quat_xyzw(neutral),ROLL_PATH[36],
                                    base_pos=plan.base_pos,rot_weight=.14,iters=180)
                plan._landing_pose = goal,neutral,hover
        start,original,offset,orientation = plan._carry
        finger = plan._carry_opening
        if not sensed.get("landed"):
            if left:
                goal,frame,target = plan._landing_pose
                phase = _phase(t,start,4.0)
                if phase < 1.:
                    plan.q = (1.-phase)*original+phase*goal
                    return np.r_[plan.q,0.], "bring the handle down over the table"
                target[2] = max(TABLE_Z+0.016,target[2]-0.0016)
            else:
                # Follow the measured core during the carry, including its rotation.
                basis = sensed["core_frame"]
                target = sensed["centre"]+basis@offset
                frame = basis@orientation
                if sensed["handle_load"] < 0.1:
                    plan._placement_z = max(TABLE_Z+.012,min(plan._placement_z,target[2])-.0025)
                    target[2] = plan._placement_z
        else:
            if NEGATIVE == "held":
                return np.r_[plan.q, finger], "negative: keep hold after setting the umbrella down"
            if not hasattr(plan,"_on_table"):
                # Freeze the measured wrist, removing the downward command that
                # accumulated while the compliant handle contacts loaded up.
                feedback=sensed.get("landing_q",{}).get(left,plan.q)
                pos,frame = ik.tcp_pose(feedback,plan.base_pos)
                plan.q=feedback.copy()
                plan._on_table = (t,pos.copy(),frame.copy())
                plan._relief_z=0.
            landed,target,frame = plan._on_table
            target=target.copy()
            elapsed=t-landed
            if left:
                if elapsed<1.:
                    plan._relief_z+=np.clip((sensed["table_load"]-1.)*.00008,-.00015,.0005)
                    plan._relief_z=np.clip(plan._relief_z,-.001,.005)
                target[2]+=plan._relief_z
                finger=_phase(t,landed,.75)
                target-=.065*_phase(t,landed+.8,1.2)*frame[:,2]
                target[2]+=.05*_phase(t,landed+1.35,1.1)
            else:
                normal=sensed["receiver"].rotation[:,2]
                if not hasattr(plan,"_seating_normal"):
                    plan._seating_normal=normal.copy()
                    plan._seating_travel=0.
                if elapsed<2.:
                    # While the handle hand clears, maintain modest normal
                    # pressure on the wrap without dragging the loose edge.
                    plan._seating_travel+=np.clip((.85-sensed["normal_pressure"])*.00012,-.00012,.00016)
                    plan._seating_travel=np.clip(plan._seating_travel,-.001,.004)
                target-=plan._seating_travel*plan._seating_normal
                finger += (1.-finger)*_phase(elapsed,2.0,1.)
                outward=frame[:,0].copy()
                if outward@plan._seating_normal<0: outward*=-1
                target += (finger-plan._carry_opening)*abs(FINGER_OPEN)*outward
                target-=.09*_phase(elapsed,2.5,1.6)*frame[:,2]
        q,_,_ = ik.solve(target,matrix_to_quat_xyzw(frame),plan.q,
                          base_pos=plan.base_pos,rot_weight=.14,iters=48)
        plan.q += np.clip(q-plan.q,-.065,.065)
        return np.r_[plan.q,finger], "lower together and release on measured tabletop support"
    if left:
        if not hasattr(plan, "_roll_start") and acquired is None:
            return np.r_[INITIAL_GRASP_Q,0.0], "support the handle while the right hand reaches"
        if not hasattr(plan, "_roll_start"):
            plan._roll_start = t
        setup = _phase(t,plan._roll_start,2.5)
        if setup < 1.:
            plan.q = (1-setup)*INITIAL_GRASP_Q + setup*ROLL_PATH[0]
            return np.r_[plan.q,0.0], "maintain the pinch while centring the wrist"
        if sensed["handle_turn"] >= 2*np.pi:
            plan._turn_done = True
        if acquired is not None and not getattr(plan, "_turn_done", False):
            angle = getattr(plan,"_angle",0.)
            speed = 0.70 - 0.35*_phase(angle,225.,75.)
            plan._angle = min(410.,angle + speed)
        angle = getattr(plan,"_angle",0.)
        demo_memory["wrist_target"] = angle
        segment = min(int(angle // 5), len(ROLL_PATH)-2)
        blend = angle/5-segment
        plan.q = (1-blend)*ROLL_PATH[segment] + blend*ROLL_PATH[segment+1]
        return np.r_[plan.q,0.0], "squeeze the handle and roll the shoulder, forearm and wrist together"
    else:
        strip = sensed["tail"]
        tcp = sensed["fingers"][[7,10]].mean(axis=0)
        frame = PINCH_FRAME.copy()
        target = strip.pos - (1-_phase(t,0.3,2.2))*0.060*frame[:,2]
        if acquired is None:
            # Compensate steady TCP tracking error while approaching the swinging ribbon.
            error = target - tcp
            correction = getattr(plan, "_correction", np.zeros(3))
            plan._correction = np.clip(correction + 0.09*error,-0.020,0.020)
            target = target + plan._correction
        if acquired is None and getattr(plan, "_closed", False) and np.linalg.norm(tcp-strip.pos)>0.022:
            plan._closed = False
            plan._correction = np.zeros(3)
        if not hasattr(plan, "_closed"):
            plan._closed = False
        if t > 2.3 and np.linalg.norm(tcp-strip.pos) < 0.009:
            plan._closed = True
        finger = 0.0 if plan._closed else APPROACH_OPEN
        if acquired is not None:
            if not hasattr(plan, "_anchor"):
                plan._anchor = tcp.copy() - sensed["centre"]
            # Keep the tension trajectory continuous even when the sensor loses contact.
            fraction = np.clip(demo_memory.get("wrist_target",0.) / 360., 0., 1.)
            direction = plan._anchor.copy()
            direction[1] = 0.
            radius = np.linalg.norm(direction)
            # The guiding hand travels with the shaft during the second half so
            # the sewn strap never has to stretch by a second circumference.
            angle = demo_memory.get("wrist_target",0.)
            follow = -np.deg2rad(110)*_phase(angle,90.,250.)
            polar = np.arctan2(direction[0],direction[2]) + follow
            target_radius = max(0.032,radius*(1.-0.80*fraction))
            target = sensed["centre"] + target_radius*radial(polar)
            target[1] = strip.pos[1]
            normal = PINCH_FRAME[:,0]
            theta = np.arctan2(normal[0],normal[2]) + follow
            frame = _grip_frame(radial(theta),(0.,1.,0.))
            if sensed["handle_turn"] >= 2*np.pi and not hasattr(plan,"_press_target"):
                plan._press_target = target.copy()
                plan._press_angle = np.arctan2(frame[0,0],frame[2,0])
                plan._press_start = t
            if hasattr(plan,"_press_target"):
                receiver, tip = sensed["receiver"], sensed["free_patch"]
                delta = receiver.rotation.T @ (tip.pos-receiver.pos)
                centred = abs(delta[0])<0.006 and abs(delta[1])<0.005
                gap = STRAP_THICK-0.0005 if centred else max(delta[2],0.005)
                error = receiver.rotation @ (np.array([0.,0.,gap])-delta)
                gain = 0.050*_phase(t,plan._press_start+0.3,1.)
                plan._press_target += np.clip(gain*error,-0.00045,0.00045)
                target = plan._press_target.copy()
                phase = _phase(t,plan._press_start+0.8,2.0)
                target = target-0.002*phase*frame[:,2]
                lean = np.deg2rad(-9.)*phase if centred else 0.
                normal = np.cos(lean)*receiver.rotation[:,2]+np.sin(lean)*receiver.rotation[:,0]
                actual = tip.rotation[:,2]
                finger = 0.04*phase if centred else finger
                yaw_error = np.arctan2(np.cross(actual,normal)[1],np.dot(actual,normal))
                plan._press_angle += np.clip(1.1*gain*yaw_error,-0.007,0.007)
                frame = _grip_frame(radial(plan._press_angle),(0.,1.,0.))
                # Roll the flap down about its loaded edge without sweeping the
                # edge away. The total wrist correction is fixed at first touch.
                contact = sensed.get("rock_contact")
                if contact and sensed["steps"]-contact[0]<=1 and not hasattr(plan,"_rock"):
                    from scipy.spatial.transform import Rotation
                    tip_n,base_n = tip.rotation[:,2],receiver.rotation[:,2]
                    axis = np.cross(tip_n,base_n)
                    angle = np.arctan2(np.linalg.norm(axis),np.dot(tip_n,base_n))
                    if np.linalg.norm(axis)>1e-5:
                        rotation_vector = axis/np.linalg.norm(axis)*min(angle,np.deg2rad(25.))
                        pivot = contact[1]
                        plan._rock = (t, receiver.rotation.T@(pivot-receiver.pos),
                                      target-pivot,frame.copy(),rotation_vector)
                if hasattr(plan,"_rock"):
                    from scipy.spatial.transform import Rotation
                    start, local_pivot, lever, start_frame, rotation_vector = plan._rock
                    pivot = receiver.pos+receiver.rotation@local_pivot
                    pitch = Rotation.from_rotvec(rotation_vector*_phase(t,start,2.5)).as_matrix()
                    frame = pitch@start_frame
                    target = pivot+pitch@lever-0.0015*_phase(t,start+1.,1.5)*receiver.rotation[:,2]

        if sensed["handle_turn"] >= 2*np.pi and not sensed.get("fastened"):
            # Keep enough pinch to tension the webbing, then let it conform as
            # the outer jaw presses. Feedback uses the opposing fingertip loads.
            if not hasattr(plan,"_grip_opening"):
                plan._grip_opening = getattr(plan,"_last_opening",0.)
            measured = min(sensed["jaw_force"])
            target_load = 1.4 if sensed.get("tacked") else 3.0
            adjustment = np.clip((measured-target_load)*0.000014,-0.00012,0.00018)
            plan._grip_opening = np.clip(plan._grip_opening+adjustment/abs(FINGER_OPEN),0.,.65)
            finger = float(plan._grip_opening)
        if sensed.get("fastened"):
            if not hasattr(plan, "_release"):
                # Keep the inner jaw at its contact radius as both jaws open.
                # Opening in place moves that jaw into the closed canopy.
                outward = frame[:, 0].copy()
                if np.dot(outward, sensed["patch_outward"]) < 0:
                    outward *= -1
                plan._release = t, target.copy(), frame.copy(), finger, outward
            start, anchor, frame, opening, outward = plan._release
            finger = opening + (1.0 - opening) * _phase(t, start + 0.2, 1.0)
            offset = (finger - opening) * abs(FINGER_OPEN)
            target = anchor + offset * outward
            target -= 0.100 * _phase(t, start + 0.6, 1.8) * frame[:, 2]
        label = "guide the webbing, press the patch and withdraw"
    q, _, _ = ik.solve(target, matrix_to_quat_xyzw(frame), plan.q, base_pos=plan.base_pos,
                       rot_weight=0.18, iters=48)
    plan.q += np.clip(q - plan.q, -0.085, 0.085)
    plan._last_opening = finger
    return np.r_[plan.q, finger], label


HANDLE_TARGET = AXIS + [0, HANDLE_Y + 0.002, 0]
ROLL_PATH = np.asarray(json.loads(Path(__file__).with_name("handle-roll.json").read_text()))
INITIAL_GRASP_Q,_,_ = ik.solve(HANDLE_TARGET,matrix_to_quat_xyzw(_tip_grip(np.deg2rad(270), (0.,-1.,0.))),
    np.array([0.,1.55,.96,-.9,1.57,-1.52]),base_pos=station.BASES["left"],rot_weight=.10,iters=180)
PINCH_TARGET = (START_STRAP[-3] + START_STRAP[-2]) / 2
PINCH_FRAME = _grip_frame(np.array([1.,0.,0.]), (0.,0.985,-0.174))
STRAP_GRASP_Q, strap_error, _ = ik.solve(
    PINCH_TARGET, matrix_to_quat_xyzw(PINCH_FRAME), np.array([0.47, 2.08, 0.68, 1.03, -1.07, -0.33]),
    base_pos=station.BASES["right"], rot_weight=0.10, iters=200)
if strap_error > 0.004:
    raise ValueError(f"Initial strap grasp residual {strap_error:.4f} m")
APPROACH_OPEN = 0.014 / abs(FINGER_OPEN)
APPROACH_Q, approach_error, _ = ik.solve(PINCH_TARGET - 0.060*PINCH_FRAME[:,2], matrix_to_quat_xyzw(PINCH_FRAME), STRAP_GRASP_Q, base_pos=station.BASES["right"], rot_weight=0.10, iters=160)
bodykit.initial_grasps = {"left": (INITIAL_GRASP_Q, 0.016), "right": (APPROACH_Q, 0.014)}
ArmPlan.action7 = _held_action


def demo():
    return {side: [_w(89.9, AXIS, FINGER_OPEN, "handheld umbrella wrap", np.eye(3))]
            for side in ("left", "right")}
