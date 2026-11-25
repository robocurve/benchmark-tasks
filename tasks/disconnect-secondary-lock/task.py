"""Separate a connector after physically moving both of its interlocking locks."""

import json
from itertools import chain
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from secondary_lock_checks import ARM_NAMES, SIGNS, Dwell, Unlock, clear, evaluate, vertices, world
from secondary_lock_contacts import Contacts
from secondary_lock_motion import (
    ALIGNED_DRAW_YAW, PAD_CONTROL_LIFT, PAD_DRAW, PAD_HOLD, PAD_LIFT, PRESS_HEADING, PRESS_ROLL, PRESS_YAW, SETDOWN_YAW,
    WITHDRAW_YAW, track_pad,
)
from secondary_lock_physics import Lead, Part, Slider, env, on_reset, on_step, part_state
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint
from yam_arena.task_file import Box, TABLE_TOP
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = .05
INSTRUCTION = ("Slide the red secondary lock toward the wires. Hold down the rear latch while pulling the "
               "plug straight out of the mounted cap. Set the plug down with its leads attached.")
TOP = TABLE_TOP[SCENE]
X = .310
PAD_HEIGHT = .004
START = (X, 0., TOP + PAD_HEIGHT + .011225)
SERVICE_YAW = np.pi
PLUG_TARGET = np.array([X, .062, START[2]])
PLUG_GRIP = np.array([0., .032, .008])
BLACK, CAP_BLACK, COVER = (.18, .19, .21), (.23, .24, .25), (.29, .31, .32)
RED, SEAL, GREY = (.70, .07, .07), (.24, .51, .37), (.55, .57, .57)
WIRE_COLORS = ((.59, .12, .10), (.70, .46, .08), (.11, .50, .35), (.18, .35, .57))


def box(x0, x1, y0, y1, z0, z1, color):
    return (((x0 + x1)/2000, (y0 + y1)/2000, (z0 + z1)/2000),
            ((x1 - x0)/1000, (y1 - y0)/1000, (z1 - z0)/1000), color)


def lead(points):
    return Lead(tuple(tuple(value / 1000 for value in point) for point in points), WIRE_COLORS)


def detent_cam():
    # One continuous convex undersurface avoids internal face contacts at the crest.
    section = ((-20.95, 9.92), (-19.30, 8.75), (-17.55, 9.92), (-17.55, 11.58), (-20.95, 11.58))
    return np.array([(x, y, z) for x in (-3.7, 3.7) for y, z in reversed(section)]) / 1000, RED


# Local +y goes from the plug nose into the cap. The coupled datum is the cap mouth.
SOCKET_SHOULDERS = tuple(box(x0, x1, 7., 9., 9.7, 11.225, CAP_BLACK)
                         for x0, x1 in ((-5.3, -1.9), (1.9, 5.3)))
SOCKET = Part("socket", pos=START, size=(.02460, .04755, .02245), mass=.014, color=CAP_BLACK, friction=.64,
              yaw=SERVICE_YAW,
              mounted=True,
              boxes=(box(-12.30, 12.30, 0., 38., -8.95, -8.15, CAP_BLACK),
                     box(-12.30, -9.42, 0., 38., -8.15, 8.7, CAP_BLACK),
                     box(9.42, 12.30, 0., 38., -8.15, 8.7, CAP_BLACK),
                     box(-12.30, 12.30, 0., 38., 7.85, 8.7, CAP_BLACK),
                     box(-9.42, 9.42, 16., 38., -8.15, 7.85, CAP_BLACK),
                     box(-12.0, 12.0, 38., 47.55, -11.225, 8.85, COVER),
                     box(-6.9, -4.9, 8.2, 35.3, -11.225, -8.95, CAP_BLACK),
                     box(4.9, 6.9, 8.2, 35.3, -11.225, -8.95, CAP_BLACK),
                     box(-8.0, -6.6, 16.5, 19.1, 8.7, 10.75, CAP_BLACK),
                     box(6.6, 8.0, 16.5, 19.1, 8.7, 10.75, CAP_BLACK),
                     *SOCKET_SHOULDERS),
              lead=lead(((0., 47.55, 0.), (0., 71., -4.), (7., 91., -9.8), (27., 106., -10.1),
                         (45., 124., -10.1), (32., 140., -10.1), (0., 154., -10.1))))
LATCH = Slider("latch", rest=(0., -.004, .0108), travel=np.radians(12.), rate=.064, damping=.005, mass=.00048,
               axis="X", negative=False, angular=True, friction=.72,
               boxes=(box(-4.9, -2.05, 9.55, 11.30, 9.05, 11.25, BLACK),
                      box(2.05, 4.9, 9.55, 11.30, 9.05, 11.25, BLACK),
                      box(-4.9, 4.9, -9.0, 11.3, 11.0, 12.55, BLACK),
                      box(-5.2, 5.2, -14.40, -8.85, 10.65, 13.25, BLACK)))
SECONDARY = Slider("safety", rest=(0., -.017, .010), travel=.0032, rate=0., damping=.025, mass=.00030,
                   axis="Y", friction=.62,
                   boxes=(box(-3.3, 3.3, -17.27, -11.75, 9.30, 10.42, RED),
                          box(-4.2, 4.2, -22.70, -17.15, 10.35, 11.58, RED),
                          box(-4.2, 4.2, -22.4, -20.5, 11.58, 13.65, RED),
                          box(-3.8, 3.8, -20.1, -19.3, 11.58, 12.25, RED)),
                   wedges=(detent_cam(),))
DETENT = Slider("detent", rest=(0., -.02075, .009), travel=.0017, rate=1500., damping=.25, mass=.00007,
                axis="Z", friction=.20,
                boxes=(box(-1.5, 1.5, -21.05, -20.45, 8.15, 9.9, BLACK),))
PLUG = Part("plug", pos=START, size=(.02533, .03670, .0268), mass=.01165, color=BLACK, friction=.69,
            yaw=SERVICE_YAW,
            boxes=(box(-11.7, 11.7, -22.70, -16.5, -11.225, 8.60, COVER),
                   box(-12.665, 12.665, -16.5, -.12, -11.225, 8.6, BLACK),
                   box(-8.75, 8.75, -.12, 13.25, -7.35, 7.2, BLACK),
                   box(-8.75, 8.75, 13.25, 14., -7.35, 7.2, RED),
                   box(-8.9, 8.9, .0, .9, -7.60, 7.45, SEAL),
                   box(-6.95, -5.8, -6.80, .45, 8.6, 15.575, BLACK),
                   box(5.8, 6.95, -6.80, .45, 8.6, 15.575, BLACK),
                   box(-6.40, -4.45, -22.5, -15.35, 8.6, 11.55, COVER),
                   box(4.45, 6.40, -22.5, -15.35, 8.6, 11.55, COVER)),
            sliders=(LATCH, SECONDARY, DETENT),
            lead=lead(((0., -22.7, 0.), (0., -46., -4.), (6., -66., -8.8), (30., -84., -10.1),
                       (48., -106., -10.1), (31., -131., -10.1), (0., -145., -10.1))))


def rest(name, y, length):
    return Box(name, pos=(X, y, TOP + PAD_HEIGHT / 2), size=(.061, length, PAD_HEIGHT), color=GREY)


PADS = [rest("service_rest", .0115, .023), rest("plug_rest", .067, .072)]
MOUNT_Y = -.02175
MOUNT = [
    Box("chassis_bracket", pos=(X, MOUNT_Y, TOP + .0016), size=(.036, .038, .0032), color=(.47, .49, .50)),
    Box("mounting_clip_base", pos=(X, MOUNT_Y, TOP + .0036), size=(.01765, .0236, .0008), color=CAP_BLACK),
    Box("mounting_clip_tongue", pos=(X, MOUNT_Y, TOP + .0051), size=(.0095, .0205, .0022), color=CAP_BLACK),
]
CLIPS = [Box(f"loom_clip_{side}_{leg}", pos=(X + offset, y, TOP + .004), size=(.004, .010, .008), color=CAP_BLACK)
         for side, y in (("plug", .145), ("cap", -.154))
         for leg, offset in (("a", -.007), ("b", .007))]
PICK_YAW = np.radians(30.)
PICK = Part("pick", pos=(.215, -.144, TOP + .0115), size=(.093, .013, .016), mass=.025, yaw=float(PICK_YAW),
            color=(.49, .28, .09), friction=.74,
            boxes=(box(-28., 27., -6.5, 6.5, -6.5, 6.5, (.49, .28, .09)),
                   box(26., 63., -1.15, 1.15, -1.15, 1.15, (.46, .47, .48)),
                   box(62., 65., -.80, .80, -9.5, 1.15, (.67, .68, .69))))
PICK_REST = Box("tool_rest", pos=(.215, -.144, TOP + .0025), size=(.072, .044, .005), yaw=float(PICK_YAW), color=GREY)
OBJECTS = [SOCKET, PLUG, PICK, PICK_REST, *PADS, *MOUNT, *CLIPS]
PARTS = (PLUG, SOCKET)
SHAPES = tuple(vertices(part) for part in PARTS)
STEP, PREVIOUS_STEP, SCORE = 0, -1, 0.
CONTACTS = None
DWELL = Dwell()
LOCK = Unlock()


@on_reset
def reset(environment):
    global STEP, PREVIOUS_STEP, SCORE, CONTACTS, LOCK
    from isaaclab_physx.physics.physx_manager import PhysxManager

    STEP, PREVIOUS_STEP, SCORE = 0, -1, 0.
    LOCK = Unlock()
    DWELL.poses.clear()
    CONTACTS = Contacts(("plug", "socket", "safety", "latch", "pick"), os.environ.get("INTERLOCK_CONTACTS"))
    if not getattr(PhysxManager.step, "_interlock_contacts", False):
        native_step = PhysxManager.step.__func__

        def step(cls):
            native_step(cls)
            CONTACTS.poll(cls.get_physics_dt())

        step._interlock_contacts = True
        PhysxManager.step = classmethod(step)


@on_step
def tick(environment):
    global STEP
    STEP += 1


def jaw_states():
    base = env().unwrapped
    for arm in ARM_NAMES:
        robot = base.scene[f"{arm}_robot"]
        ids, _ = robot.find_joints([*ARM_JOINTS, *FINGER_JOINTS], preserve_order=True)
        q = robot.data.joint_pos.torch[0, ids].detach().cpu().numpy()
        center, rotation = ik.tcp_pose(q[:6], station.BASES[arm])
        yield center, rotation, q[6:]


def jaw_bounds(center, rotation, finger_pos):
    fingers = center + (SIGNS * [max(abs(finger_pos)) + .013, .039, .033] + [0., 0., -.016]) @ rotation.T
    return fingers.min(0), fingers.max(0)


def stage_shape(specs, shift=(0., 0., 0.)):
    return np.concatenate([np.array(center) + shift + SIGNS * np.array(size) / 2
                           for center, size, _ in specs])


def latch_points(specs, angle):
    pivot = np.array(LATCH.rest)
    rotation = axis_angle_matrix(np.array([1., 0., 0.]), angle)
    return (stage_shape(specs) - pivot) @ rotation.T + pivot


def rear_pad_loads(plug, angle):
    pivot = np.array(LATCH.rest)
    inverse = plug.rotation() @ axis_angle_matrix(np.array([1., 0., 0.]), angle)
    loads = np.zeros(5)
    for source, force, point, normal in chain(CONTACTS.surface_contacts("latch"), CONTACTS.pick_pad_contacts()):
        rest_point = (point - plug.to_world(pivot)) @ inverse + pivot
        vertical = abs(float((normal @ inverse)[2]))
        if (abs(rest_point[0]) <= .0058 and -.015 <= rest_point[1] <= -.0082
                and abs(rest_point[2] - .01325) <= .0012 and vertical >= .65):
            loads[source] += force * vertical
    return loads.tolist()


def cpa_contacts(plug):
    rotation = plug.rotation()
    samples = [(str(finger), force, point, normal)
               for finger, force, point, normal in CONTACTS.surface_contacts("safety")]
    samples.extend(("pick", force, point, normal) for _, force, point, normal in CONTACTS.pick_contacts())
    samples.extend(CONTACTS.internal_cpa_contacts())
    return [dict(contact=source, load_n=force, point_m=plug.to_local(point).tolist(),
                 normal=(normal @ rotation).tolist()) for source, force, point, normal in samples]


def shell_contacts(plug):
    rotation = plug.rotation()
    return [dict(finger=finger, load_n=force, point_m=plug.to_local(point).tolist(), normal=(normal @ rotation).tolist())
            for finger, force, point, normal in CONTACTS.surface_contacts("plug") if force >= .25]


def tool_contacts(plug):
    rotation = plug.rotation()
    return [dict(part=part, load_n=force, point_m=plug.to_local(point).tolist(), normal=(normal @ rotation).tolist())
            for part in CONTACTS.pick_surfaces for _, force, point, normal in CONTACTS.pick_contacts(part)
            if force >= .1]


def score(objects):
    global PREVIOUS_STEP, SCORE
    if PREVIOUS_STEP == STEP:
        return SCORE
    plug, socket = (objects[part.name] for part in PARTS)
    if not np.isfinite(np.r_[plug.pos, socket.pos, plug.quat_xyzw, socket.quat_xyzw]).all():
        raise RuntimeError("Connector physics produced a nonfinite housing pose")
    relative = (plug.pos - socket.pos) @ socket.rotation()
    withdrawal = float(-relative[1])
    travel, angle = -plug.sliders["safety"], plug.sliders["latch"]
    latch_shape = latch_points(LATCH.boxes, angle)
    pad_center = np.array([0., -.013, .01325])
    pivot = np.array(LATCH.rest)
    press_normal = axis_angle_matrix(np.array([1., 0., 0.]), angle)
    depression = float((pad_center - ((pad_center - pivot) @ press_normal.T + pivot))[2])
    hooks = latch_points(LATCH.boxes[:2], angle)
    tip_height = float(hooks[:, 2].min())
    hooks_in_cap = (world(plug, hooks) - socket.pos) @ socket.rotation()
    hook_bottom_cap = hooks_in_cap[:, 2].reshape(2, len(SIGNS)).min(1)
    alignment = (socket.rotation().T @ plug.rotation())[1, 1]
    aligned = alignment > .995 and np.linalg.norm(relative[[0, 2]]) < .003
    pad_load = rear_pad_loads(plug, angle)
    slider_open, tooth_clear = LOCK.update(travel, depression, withdrawal, aligned, np.all(hook_bottom_cap >= .0112),
                                          max(pad_load) >= .20)
    physical_shapes = [
        np.concatenate((SHAPES[0], latch_shape,
                        stage_shape(SECONDARY.boxes, [0., -travel, 0.]))), SHAPES[1],
    ]
    shapes = [world(state, shape) for state, shape in zip((plug, socket), physical_shapes)]
    plug_front = np.max((shapes[0] - socket.pos) @ socket.rotation(), axis=0)[1]
    separated = plug_front < -.030
    on_pads = (np.linalg.norm((plug.pos - PLUG_TARGET)[:2]) < .021
               and abs(shapes[0][:, 2].min() - TOP - PAD_HEIGHT) < .002
               and np.linalg.norm(socket.pos - START) < .001)
    settled = all(np.linalg.norm(state.lin_vel) < .015 and np.linalg.norm(state.ang_vel) < .35
                  for state in (plug, socket))
    LOCK.invalid |= any(shape[:, 2].min() < TOP - .045 for shape in shapes)
    hands = list(jaw_states())
    hands_clear = clear([jaw_bounds(*hand) for hand in hands], shapes)
    dwell = DWELL.update(separated and on_pads and settled and hands_clear, (plug, socket))
    SCORE = evaluate(slider_open, tooth_clear, withdrawal, LOCK.escaped, separated, on_pads, dwell, LOCK.invalid)
    PREVIOUS_STEP = STEP
    path = os.environ.get("INTERLOCK_DIAGNOSTICS")
    if path:
        row = dict(step=STEP, score=SCORE, withdrawal_m=withdrawal, safety_m=travel, latch_m=depression,
                   latch_angle_deg=float(np.degrees(angle)), latch_tip_m=tip_height,
                   hook_bottom_cap_m=hook_bottom_cap.tolist(),
                   detent_m=-plug.sliders["detent"],
                   plug_front_m=float(plug_front), aligned=bool(aligned), cross_error_m=float(np.linalg.norm(relative[[0, 2]])),
                   escaped=bool(LOCK.escaped), staged=bool(on_pads),
                   dwell=dwell, hands_clear=hands_clear, invalid=bool(LOCK.invalid),
                   position={name: objects[name].pos.tolist() for name in ("plug", "socket", "pick") if name in objects},
                   rotation={name: objects[name].rotation().tolist() for name in ("plug", "socket", "pick") if name in objects},
                   clip_error_m={"plug": plug.clip_error, "socket": socket.clip_error},
                   fingers_n=CONTACTS.loads.tolist(), rear_pad_n=pad_load,
                   cpa_contacts=cpa_contacts(plug), shell_contacts=shell_contacts(plug),
                   tool_contacts=tool_contacts(plug),
                   hook_socket_contacts=[dict(load_n=force, point_m=socket.to_local(point).tolist(),
                                              normal=(normal @ socket.rotation()).tolist())
                                         for _, force, point, normal in CONTACTS.hook_contacts()],
                   housing_socket_contacts=[dict(load_n=force, point_m=socket.to_local(point).tolist(),
                                                 normal=(normal @ socket.rotation()).tolist())
                                            for _, force, point, normal in CONTACTS.housing_contacts()],
                   cpa_force_n=(CONTACTS.vectors[:, CONTACTS.names.index("safety")] @ plug.rotation()).tolist(),
                   jaw={arm: dict(tcp=center.tolist(), rotation=rot.tolist(), slides=q.tolist())
                        for arm, (center, rot, q) in zip(ARM_NAMES, hands)},
                   arm_contacts=CONTACTS.count)
        with open(path, "a") as stream:
            stream.write(json.dumps(row) + "\n")
    return SCORE


def success(objects):
    return score(objects) == 1.


def xyz(values):
    return tuple(float(value) for value in values)


def demo():
    mode = os.environ.get("INTERLOCK_DEMO", "demo")
    if mode == "idle":
        return {}
    left, right = [], []
    opened, shut = FINGER_OPEN, FINGER_CLOSED
    start = np.array(START)
    high = np.array([0., 0., .065])
    hold = start + PLUG_GRIP
    left_park = hold + [0., .092, .064]
    tool_grip = np.array(PICK.pos) + [0., 0., .010]
    ready = tool_grip + high

    def both(seconds, lpos, rpos, lfingers, rfingers, label, yaw=PICK_YAW, left_yaw=WITHDRAW_YAW):
        left.append(Waypoint(seconds, xyz(lpos), lfingers, label, left_yaw))
        right.append(Waypoint(seconds, xyz(rpos), rfingers, label, yaw))

    both(2., left_park, ready, opened, opened, "approach the mounted connector and service pick")
    both(1.2, left_park, tool_grip, opened, opened, "lower the right jaws over the service pick")
    both(1., left_park, tool_grip, opened, shut, "grasp the service pick handle")
    both(1., left_park, ready, opened, shut, "lift the pick")
    rise = np.array([0., 0., .035])
    if mode != "locked":
        tip_offset = .0635 * np.array([np.cos(PICK_YAW), np.sin(PICK_YAW), 0.])
        rail = start + [-.0005, .0181, .0315] - tip_offset
        rail_rear = rail + [0., .010, -.0020]
        both(1.4, left_park, rail + rise, opened, shut, "bring the narrow pick over the native slider")
        both(1.3, left_park, rail, opened, shut, "lower the pick blade in front of the ridge")
        both(.95, left_park, rail + [0., .0055, -.0005], opened, shut, "engage the front face of the rear ridge")
        both(1.5, left_park, rail_rear, opened, shut, "slide the CPA rearward by its own molded ridge")
        both(.7, left_park, rail_rear, opened, shut, "let the CPA settle against its rear stop")
        both(1.1, left_park, rail_rear + rise, opened, shut, "raise the pick clear of the retracted slider")

    press_park = start + [-.020, -.103, .067]
    tool_park = start + [-.064, -.123, .092]
    both(1.15, left_park, press_park, opened, shut, "move the pick to a clear orientation pose")
    both(1.2, left_park, press_park, opened, shut, "turn the pick along the withdrawal path", PRESS_HEADING)
    both(1.2, left_park, press_park, opened, shut, "aim the pick shaft toward the native rear pad", PRESS_YAW)

    side_open = -.025
    plug_grip = -.002
    both(1.2, hold + [0., .041, .029], press_park, side_open, shut, "align the left jaws with the plug shell", PRESS_YAW)
    both(1.35, hold, press_park, side_open, shut, "approach the plug shell", PRESS_YAW)
    both(1.1, hold, press_park, plug_grip, shut, "hold the plug shell without touching the leads", PRESS_YAW)
    seated = hold + [0., -.0026, 0.]
    both(1.25, seated, press_park, plug_grip, shut, "seat the plug to unload the primary latch", PRESS_YAW)

    tool_rotation = (axis_angle_matrix(np.array([0., 0., 1.]), PRESS_HEADING)
                     @ axis_angle_matrix(np.array([0., 1., 0.]), PRESS_ROLL))
    toe_lever = tool_rotation @ np.array([.0642, -.00015, -.01925])
    pad = start + [-.0034, .0141, .0065] - toe_lever
    pad_above = pad + [0., 0., .019]
    both(1.5, seated, pad_above, plug_grip, shut, "align the narrow pick over the native rear latch pad", PRESS_YAW)
    pressed = pad_above if mode == "unpressed" else pad
    both(2.1, seated, pressed, plug_grip, shut, "depress the native rear latch pad", PRESS_YAW)
    both(2.3, seated, pressed, plug_grip, shut, PAD_HOLD, PRESS_YAW)
    first_pull = np.array([-.0006, .022, -.0008])
    tool_follow = np.array([0., .022, 0. if mode == "unpressed" else -.0007])
    track_pad(lambda: part_state(env(), PLUG).pos if env() is not None else None,
              pad[1], pressed[2] + tool_follow[2])
    both(3.25, hold + first_pull, pressed + tool_follow, plug_grip, shut, PAD_DRAW, PRESS_YAW, ALIGNED_DRAW_YAW)
    both(1.05, hold + first_pull, pressed + tool_follow, plug_grip, shut, PAD_DRAW, PRESS_YAW, ALIGNED_DRAW_YAW)
    if mode in ("locked", "unpressed"):
        place = hold + first_pull
        release_yaw = ALIGNED_DRAW_YAW
        both(1.15, place, pad_above + tool_follow, plug_grip, shut, PAD_CONTROL_LIFT, PRESS_YAW, release_yaw)
    else:
        far = np.array([-.0006, .064, -.001])
        both(.8, hold + first_pull, pad_above + tool_follow, plug_grip, shut, PAD_LIFT, PRESS_YAW, ALIGNED_DRAW_YAW)
        both(2.2, hold + far, tool_park, plug_grip, shut, "withdraw the plug completely", PRESS_YAW, ALIGNED_DRAW_YAW)
        levelled = start + [.0005, .087, .038]
        place = levelled + [0., 0., -.013]
        release_yaw = SETDOWN_YAW
        both(1.55, levelled, tool_park, plug_grip, shut, "level the unplugged housing over its rest", PRESS_YAW, release_yaw)
        both(1.25, place, tool_park, plug_grip, shut, "set the underside flat on the release rest", PRESS_YAW, release_yaw)
        both(.85, place, tool_park, plug_grip, shut, "let the supported plug settle", PRESS_YAW, release_yaw)
    both(.9, place, tool_park, side_open, shut, "release the plug on its support", PRESS_YAW, release_yaw)
    both(1.6, place + [0., .022, .039], tool_park, side_open, shut, "retreat along the left approach", PRESS_YAW, release_yaw)
    both(1.5, left_park, tool_park, opened, shut, "move the left hand out of the service area", PRESS_YAW)
    both(1.2, left_park, tool_park, opened, shut, "level the pick above the worktop", PRESS_HEADING)
    both(1.2, left_park, ready, opened, shut, "bring the pick back to its rest")
    both(1.1, left_park, tool_grip, opened, shut, "replace the pick on the rest")
    both(.9, left_park, tool_grip, opened, opened, "release the pick")
    both(1.3, left_park, ready, opened, opened, "retreat clear of the mounted connector and loose plug")
    return {"left": left, "right": right}
