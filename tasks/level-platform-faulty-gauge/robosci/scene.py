"""Base scene: two i2RT YAM arms on a table with the ABC-kit cameras.

World frame = the left arm's base frame (i2RT convention). Arms face +x; the
right arm sits at y = -0.61. The table top is z = 0. The workspace centre, where
the overhead camera's optical axis meets the table, is (0.384, -0.305, 0).

Action layout (14-D, matches Robocurve's ``yam_arms`` contract):
    [left j1..j6, left gripper, right j1..j6, right gripper]
Joints are radians. Gripper is the finger slide in metres per finger:
0 = closed, GRIPPER_OPEN = fully open (9.4 cm between pads).
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

ASSETS = Path(__file__).parent / "assets"
STATION_XML = ASSETS / "yam" / "station" / "yam_station" / "yam_station.xml"

SIDES = ("left", "right")
ARM_JOINTS = {s: [f"{s}_joint{i}" for i in range(1, 7)] for s in SIDES}
GRIPPER_OPEN = 0.0469
WORK_CENTER = np.array([0.384, -0.305, 0.0])
TABLE_TOP = 0.0
CONTROL_HZ = 10.0
TIMESTEP = 0.002

# Home pose: gripper pointing down about 22 cm above the table near each base,
# clear of the overhead camera's view of the workspace.
HOME_ARM = {
    "left": np.array([-0.245, 1.4, 1.571, -1.692, 0.0, 1.325]),
    "right": np.array([0.244, 1.399, 1.571, -1.692, 0.0, 1.815]),
}

# The i2RT optical frames are ROS style (+z forward, +y down). MuJoCo cameras
# look along -z with +y up, which is the ROS frame turned 180 deg about x.
_ROS_TO_MJ = [0.0, 1.0, 0.0, 0.0]
CAMERAS = {"top": "top_camera", "left_wrist": "left_camera", "right_wrist": "right_camera"}
# D405: 87 x 58 deg; MuJoCo fovy is vertical.
D405_FOVY = 58.0


def _arm_bodies(spec: mujoco.MjSpec) -> list[mujoco.MjsBody]:
    return [b for b in spec.bodies if b.name.startswith(("left_", "right_")) and "base" not in b.name]


# Real YAM look: white links, black joints and base, dark grey gripper (the CAD export
# gives every part an arbitrary colour). Purely visual.
_WHITE, _BLACK, _DARK = [0.93, 0.93, 0.92, 1], [0.08, 0.08, 0.09, 1], [0.22, 0.22, 0.24, 1]
_MESH_RGBA = {"base": _BLACK, "link1": _BLACK, "link2": _WHITE, "link3": _WHITE, "link4": _BLACK,
              "link5": _BLACK, "gripper": _DARK, "tip_left": _DARK, "tip_right": _DARK,
              "d405_wrist_linear_4310": _BLACK, "d405_top": _BLACK}


def _restyle_arms(spec: mujoco.MjSpec) -> None:
    for g in spec.geoms:
        if g.type == mujoco.mjtGeom.mjGEOM_MESH and g.meshname in _MESH_RGBA:
            g.rgba = _MESH_RGBA[g.meshname]


def base_spec() -> mujoco.MjSpec:
    """Return an editable spec of the arms, table, lights, cameras and actuators."""
    spec = mujoco.MjSpec.from_file(str(STATION_XML))
    spec.option.timestep = TIMESTEP
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.noslip_iterations = 3
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720
    spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
    spec.visual.headlight.diffuse = [0.35, 0.35, 0.35]
    spec.visual.headlight.specular = [0.05, 0.05, 0.05]
    spec.visual.quality.shadowsize = 4096
    spec.visual.quality.offsamples = 8  # anti-aliasing for offscreen renders

    # Gravity-compensated arm links so position control tracks precisely,
    # as the real YAM controller does.
    for b in _arm_bodies(spec):
        b.gravcomp = 1.0
    for j in spec.joints:
        j.armature = 0.01
        j.damping = [1.0 if j.type == mujoco.mjtJoint.mjJNT_HINGE else 5.0, 0, 0]

    # Robot meshes are visual only. Their convex hulls nearly close the finger gap,
    # so each finger collides through a box pad fitted to the real pad facet
    # (2.8 x 1.8 cm, measured from the tip mesh; pads 2 * slide apart).
    for g in spec.geoms:
        g.contype = 0
        g.conaffinity = 0
    for s in SIDES:
        for tip, pos in ((f"{s}_tip_left", [0.02409, -0.04777, -0.08047]),
                         (f"{s}_tip_right", [-0.0239, 0.04785, -0.08047])):
            spec.body(tip).add_geom(name=f"{tip}_pad", type=mujoco.mjtGeom.mjGEOM_BOX,
                                    size=[0.014, 0.0033, 0.009], pos=pos, quat=[0, 1, 0, 0],
                                    friction=[1.5, 0.06, 0.002], condim=4, priority=1,
                                    rgba=[0.2, 0.2, 0.2, 0], mass=0.005, group=3)

    world = spec.worldbody
    _restyle_arms(spec)
    # Look only: light grey gradient sky, plain matte table, pale floor, soft light.
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
                     rgb1=[0.86, 0.87, 0.89], rgb2=[0.62, 0.64, 0.67], width=512, height=512)
    spec.add_material(name="table", rgba=[0.90, 0.84, 0.71, 1], reflectance=0.0, specular=0.05, shininess=0.1)
    spec.add_material(name="floor", rgba=[0.80, 0.81, 0.83, 0], specular=0.0)  # invisible: sky shows
    world.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[3, 3, 0.05],
                   pos=[0, 0, -0.75], material="floor", contype=1, conaffinity=1)
    world.add_geom(name="table", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.45, 0.65, 0.375],
                   pos=[0.25, -0.305, -0.375], material="table", friction=[0.8, 0.01, 0.001],
                   contype=1, conaffinity=1)
    world.add_light(name="key", pos=[0.6, -0.305, 1.8], dir=[-0.2, 0, -1], diffuse=[0.45, 0.45, 0.45],
                    specular=[0.05, 0.05, 0.05], castshadow=True)
    world.add_light(name="fill", pos=[1.2, 0.4, 1.0], dir=[-1, -0.5, -0.6], diffuse=[0.22, 0.22, 0.22],
                    specular=[0, 0, 0], castshadow=False)

    for site in spec.sites:
        site.group = 4  # hide tool-centre markers from all cameras

    for cam_name, body_name in CAMERAS.items():
        spec.body(body_name).add_camera(name=cam_name, quat=_ROS_TO_MJ, fovy=D405_FOVY)
    world.add_camera(name="overview", pos=[1.25, -0.305, 0.75], xyaxes=[0, 1, 0, -0.5, 0, 0.87], fovy=45)
    world.add_camera(name="side", pos=[0.40, 0.75, 0.55], xyaxes=[-1, 0, 0, 0, -0.55, 0.83], fovy=45)
    # Presentation view for screenshots and videos (not a robot camera): behind and above the arms.
    world.add_camera(name="hero", pos=[-0.55, -0.305, 1.0], xyaxes=[0, -1, 0, 0.726, 0, 0.688], fovy=44)

    for s in SIDES:
        for jn in ARM_JOINTS[s]:
            spec.add_actuator(name=f"{jn}_act", target=jn,
                              trntype=mujoco.mjtTrn.mjTRN_JOINT).set_to_position(kp=400.0, kv=20.0)
        act = spec.add_actuator(name=f"{s}_gripper_act", target=f"{s}_joint7",
                                trntype=mujoco.mjtTrn.mjTRN_JOINT)
        act.set_to_position(kp=600.0, kv=10.0)
        act.forcerange = [-25.0, 25.0]
        act.forcelimited = mujoco.mjtLimited.mjLIMITED_TRUE
        act.ctrlrange = [0.0, GRIPPER_OPEN]
        act.ctrllimited = mujoco.mjtLimited.mjLIMITED_TRUE
    return spec


def home_ctrl() -> np.ndarray:
    """14-D action holding both arms at home with grippers open."""
    return np.concatenate([np.concatenate([HOME_ARM[s], [GRIPPER_OPEN]]) for s in SIDES])


def gripper_slide_from_action(open_amount: float) -> float:
    """Map gripper opening in metres to the joint7 slide target (equal by design)."""
    return float(np.clip(open_amount, 0.0, GRIPPER_OPEN))


def set_home(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Teleport both arms to home and set matching controls."""
    for s in SIDES:
        for jn, q in zip(ARM_JOINTS[s], HOME_ARM[s], strict=True):
            data.qpos[model.joint(jn).qposadr[0]] = q
        for k in (7, 8):
            data.qpos[model.joint(f"{s}_joint{k}").qposadr[0]] = GRIPPER_OPEN
    apply_action(model, data, home_ctrl())
    mujoco.mj_forward(model, data)


def apply_action(model: mujoco.MjModel, data: mujoco.MjData, action: np.ndarray) -> None:
    """Write a 14-D action into actuator controls (gripper given as opening)."""
    a = np.asarray(action, dtype=float)
    for k, s in enumerate(SIDES):
        seg = a[7 * k: 7 * k + 7]
        for jn, q in zip(ARM_JOINTS[s], seg[:6], strict=True):
            data.ctrl[model.actuator(f"{jn}_act").id] = q
        data.ctrl[model.actuator(f"{s}_gripper_act").id] = gripper_slide_from_action(seg[6])


def current_action(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    """Read the measured 14-D joint state (gripper as opening in metres)."""
    out = []
    for s in SIDES:
        out += [data.qpos[model.joint(jn).qposadr[0]] for jn in ARM_JOINTS[s]]
        out.append(data.qpos[model.joint(f"{s}_joint7").qposadr[0]])
    return np.array(out)
