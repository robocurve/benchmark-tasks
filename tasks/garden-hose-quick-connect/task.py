"""On the station table, in front of the arms: a garden tap on a standpipe post, a brass quick-connect coupler on its
spout with the mouth toward the robot, and a green hose clamped to the table off to the robot's right, its brass plug
resting on the table.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import Shape, UsdBody, before_reset, on_reset, on_step  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Connect the hose to the tap: pull back the coupler's collar, push the hose's brass plug fully in, "
               "let the collar spring forward to lock it, then let go of everything.")

TOP = g.TOP
POST_GREY = (0.55, 0.56, 0.58)


def _f(v):
    return tuple(float(x) for x in v)


def _yaw(a: float):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


PIN_JOINTS = tuple(f"pin{i}" for i in range(len(g.PIN_ANGLES)))
HOSE_JOINTS = tuple(f"j{i}" for i in range(g.HOSE_LINKS + 1))

# -- objects --------------------------------------------------------------------------------------------------------
AX = (math.cos(g.BORE_YAW), math.sin(g.BORE_YAW), 0.0)        # the bore's direction, into the tap
COUPLER = UsdBody("coupler", usd="assets/coupler.usda", pos=(g.MOUTH_X, g.MOUTH_Y, TOP + g.AXIS_Z),
                  rot_xyzw=_yaw(g.BORE_YAW),
                  size=(g.BODY_LEN, 2 * g.COLLAR_R_OUT, 2 * g.COLLAR_R_OUT), articulation=True, fix_root=True,
                  joint_pos=(("collar", 0.0), *((p, 0.0) for p in PIN_JOINTS)),
                  drives=(("collar", g.COLLAR_K, 4.0), ("pin.*", g.PIN_K, 0.5)),
                  friction=(0.9, 0.8))
HOSE_REST = tuple((j, 0.0) for j in HOSE_JOINTS)     # the curve is built into the hose (its joints' zero)
HOSE = UsdBody("hose", usd="assets/hose.usda", pos=(g.ANCHOR_X, g.ANCHOR_Y, TOP + g.HOSE_R + 0.0005),
               rot_xyzw=_yaw(g.ANCHOR_HEADING), size=(0.05, 0.05, 0.03), articulation=True, fix_root=True,
               joint_pos=HOSE_REST,
               drives=(("j.*", g.HOSE_K, g.HOSE_DAMPING),), friction=(1.2, 1.0))
_post_h = g.AXIS_Z + 0.06
POST = Shape("tap_post", pos=_f((g.MOUTH_X + AX[0] * g.POST_BACK, g.MOUTH_Y + AX[1] * g.POST_BACK, TOP + _post_h / 2)),
             size=_f((*g.POST_SIZE, _post_h)), rot_xyzw=_f(_yaw(g.BORE_YAW)), color=POST_GREY)
OBJECTS = [COUPLER, HOSE, POST]

# Finger pads, from each finger link's origin (at the finger's base) in the gripper link's frame: the pad's tip and a
# point 3 cm back along it (yam_arena's URDF).
PADS = {"tip_left": [(-0.0445, 0.024, 0.0903), (-0.0445, 0.024, 0.0603)],
        "tip_right": [(0.0445, -0.024, 0.0903), (0.0445, -0.024, 0.0603)]}

# -- monitor: the lock ----------------------------------------------------------------------------------------------
_STATE: dict = {}


def _np(x):
    return getattr(x, "torch", x).detach().cpu().numpy()


def _rotation(xyzw):
    import numpy as np

    x, y, z, w = xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _limits(environment, values: dict) -> None:
    import torch

    asset = environment.unwrapped.scene["coupler"]
    for joint, lim in values.items():
        if _STATE["limits"].get(joint) == lim:
            continue
        _STATE["limits"][joint] = lim
        ids = asset.find_joints([joint])[0]
        asset.write_joint_position_limit_to_sim_index(
            limits=torch.tensor([[list(lim)]], dtype=torch.float32, device=asset.device), joint_ids=ids,
            warn_limit_violation=False)


def plug_pose(environment):
    """The plug's frame (its flange face, x toward its nose) and speed, station frame."""
    import numpy as np

    base = environment.unwrapped
    asset = base.scene["hose"]
    origin = _np(base.scene.env_origins[0])
    i = list(asset.body_names).index("plug")
    pose = _np(asset.data.body_link_pose_w)[0, i]
    vel = _np(asset.data.body_link_vel_w)[0, i]
    return pose[:3] - origin, _rotation(pose[3:7]), float(np.linalg.norm(vel[:3]))


def pad_points(environment) -> list:
    import numpy as np

    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    points = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        pose = _np(robot.data.body_link_pose_w)[0]
        names = list(robot.body_names)
        R = _rotation(pose[names.index("gripper"), 3:7])
        for link, offsets in PADS.items():
            at = pose[names.index(link), :3] - origin
            points += [at + R @ np.asarray(o) for o in offsets]
    return points


def _segment_distance(p, a, b) -> float:
    import numpy as np

    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = min(1.0, max(0.0, float((p - a) @ ab / (ab @ ab))))
    return float(np.linalg.norm(p - (a + t * ab)))


def _clearance(environment, plug_p, plug_R) -> float:
    """The smallest distance from any finger pad to the coupler and collar (a 36 mm rod from the collar's front to the
    tap) or to the plug (a 24 mm rod from its boot to its nose)."""
    import numpy as np

    mouth, ax = np.array([g.MOUTH_X, g.MOUTH_Y, TOP + g.AXIS_Z]), np.array(AX)
    c0, c1 = mouth + ax * g.COLLAR_X0, mouth + ax * g.BODY_LEN
    back = g.FLANGE_LEN + g.PLUG_BODY_LEN + g.BOOT_LEN
    p0, p1 = plug_p - plug_R[:, 0] * back, plug_p + plug_R[:, 0] * g.SHAFT_LEN
    return min(min(_segment_distance(q, c0, c1) - g.COLLAR_R_OUT, _segment_distance(q, p0, p1) - g.FLANGE_R)
               for q in pad_points(environment))


def _reset_values() -> None:
    _STATE.update(limits={}, collar=0.0, pins=(0.0,) * len(PIN_JOINTS), flange=-1.0, nose=-1.0, entered=False,
                  seated=False, locked=False, clearance=1.0, plug_speed=0.0, max_collar=0.0, history=[], drift=1.0)


@on_reset
def _reset_state(environment=None) -> None:
    from scenekit import set_joint_targets

    _reset_values()
    if environment is not None:
        set_joint_targets(environment, "coupler", {"collar": -g.COLLAR_PRELOAD, **{p: 0.0 for p in PIN_JOINTS}})
        set_joint_targets(environment, "hose", dict(HOSE_REST))       # the hose keeps the curve it was laid in
        _limits(environment, {"collar": (0.0, g.COLLAR_TRAVEL), **{p: (0.0, 1e-4) for p in PIN_JOINTS}})


@before_reset
def _restore(environment) -> None:
    from scenekit import write_joint_positions

    _STATE["limits"] = {}
    _limits(environment, {"collar": (0.0, g.COLLAR_TRAVEL), **{p: (0.0, g.PIN_TRAVEL) for p in PIN_JOINTS}})
    write_joint_positions(environment, "coupler", {"collar": 0.0, **{p: 0.0 for p in PIN_JOINTS}})
    write_joint_positions(environment, "hose", dict(HOSE_REST))


_reset_values()


@on_step
def _monitor(environment) -> None:
    """After every control step, the ball lock as joint limits: with the collar forward its land holds the pins down;
    pulled back past RELEASE its relief lets them rise (the plug's shaft pushes them out, their springs push them back);
    a raised pin holds the collar back at RELEASE; once all pins are down the collar springs forward and holds them.
    Then: how far the plug is in, whether it is locked, and whether the hands have let go."""
    import numpy as np

    from yam_arena.scene import object_states

    joints = object_states(environment, ["coupler"])["coupler"].joints
    c = float(joints["collar"])
    pins = tuple(float(joints[p]) for p in PIN_JOINTS)
    raised = any(p > g.PIN_DOWN for p in pins)
    if c >= g.RELEASE - 1e-4:
        _limits(environment, {p: (0.0, g.PIN_TRAVEL) for p in PIN_JOINTS})
    else:
        _limits(environment, {p: (0.0, 1e-4) for p in PIN_JOINTS})
    _limits(environment, {"collar": (g.RELEASE if (raised and c >= g.RELEASE - 1e-4) else 0.0, g.COLLAR_TRAVEL)})

    plug_p, plug_R, speed = plug_pose(environment)
    mouth, ax = np.array([g.MOUTH_X, g.MOUTH_Y, TOP + g.AXIS_Z]), np.array(AX)
    off = plug_p - mouth
    flange = float(off @ ax)                                            # the flange face, in along the bore (<= 0)
    cos_tilt = float(plug_R[:, 0] @ ax)
    on_axis = float(np.linalg.norm(off - flange * ax)) <= g.BORE_R and cos_tilt > math.cos(math.radians(10))
    # depth into the bore counts only with the plug on the bore's axis (not alongside the coupler)
    nose = flange + g.SHAFT_LEN * cos_tilt if on_axis else -1.0
    seated = on_axis and flange >= -g.SEATED
    _STATE.update(collar=c, pins=pins, flange=flange, nose=nose, plug_speed=speed, seated=seated,
                  max_collar=max(_STATE["max_collar"], c))
    hist = _STATE["history"]                                            # the plug's last second, for stillness
    hist.append(plug_p.copy())
    del hist[:-20]
    _STATE["drift"] = max(float(np.linalg.norm(q - plug_p)) for q in hist) if len(hist) == 20 else 1.0
    _STATE["entered"] = _STATE["entered"] or nose >= g.ENTERED
    _STATE["locked"] = bool(seated and c <= g.LOCKED_COLLAR and not raised)
    _STATE["clearance"] = _clearance(environment, plug_p, plug_R)


def success(objects) -> bool:
    """Locked (seated, collar forward, balls down in the groove), every finger pad 3 cm clear and the plug still (moved
    under 3 mm over the last second); held 1 s."""
    return bool(_STATE["locked"] and _STATE["clearance"] >= g.LET_GO and _STATE["drift"] < g.STILL)


def score(objects) -> float:
    """Partial credit at the end (success scores 1.0): 0.75 locked but still held; 0.5 the plug's nose at least 15 mm (past the balls)
    into the bore but not locked; 0.25 it got that far at some point but is out at the end; 0 otherwise."""
    if success(objects):
        return 1.0
    if _STATE["locked"]:
        return 0.75
    if _STATE["nose"] >= g.ENTERED:
        return 0.5
    return 0.25 if _STATE["entered"] else 0.0
