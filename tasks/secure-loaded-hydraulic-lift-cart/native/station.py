"""Attach both YAM arms at the verified original positions, facing the lift."""

import math, numpy as np
from native.fixture import BASES

EYE = (-2.05, -0.60, 1.13)
TARGET = (-0.62, 0.04, 0.66)


def aim(pos, target, up=(0.0, 0.0, 1.0)):
    from scipy.spatial.transform import Rotation

    back = np.subtract(pos, target)
    back /= np.linalg.norm(back)
    right = np.cross(up, back)
    right /= np.linalg.norm(right)
    return tuple(
        Rotation.from_matrix(
            np.column_stack((right, np.cross(back, right), back))
        ).as_quat()
    )


def wrist_pose(pos, quat):
    from scipy.spatial.transform import Rotation

    r = Rotation.from_quat(quat)
    p = tuple(np.asarray(pos) + r.apply((0, 0, 0.065)))
    return p, aim(p, (0, 0, 0.15), r.apply((0, 1, 0)))


def camera_mount():
    from scipy.spatial.transform import Rotation
    from yam_arena.station import _segment

    quat = aim(EYE, TARGET)
    back = Rotation.from_quat(quat).apply((0.0, 0.0, 1.0))
    housing = np.array(EYE) + back * 0.031
    mast_top = np.array([*(np.array(EYE)[:2] + back[:2] * 0.18), housing[2]])
    foot = (mast_top[0], mast_top[1], 0.012)
    mast = _segment((*foot[:2], 0.024), mast_top)
    arm = _segment(mast_top, housing)
    return dict(
        stand_foot=(foot, (0.0, 0.0, 0.0, 1.0), (0.38, 0.38, 0.024)),
        stand_mast=mast,
        stand_arm=arm,
        camera_housing=(housing, quat, (0.05, 0.042, 0.022)),
    )


def configure(cfg):
    parts = camera_mount()
    for name, (pos, rotation, dims) in parts.items():
        part = getattr(cfg, name)
        part.init_state.pos = tuple(map(float, pos))
        part.init_state.rot = tuple(map(float, rotation))
        if isinstance(dims, tuple):
            part.spawn.size = dims
        else:
            part.spawn.height = dims
    layout = {
        "camera_mount_positions_m": {
            n: list(map(float, v[0])) for n, v in parts.items()
        }
    }
    # The calibrated station ground sits 12.5 mm below the cart design datum.
    floor = cfg.floor
    height = float(floor.spawn.size[2])
    x, y, old_center = floor.init_state.pos
    if abs(old_center + height / 2 + 0.0125) > 0.003:
        raise ValueError("Unexpected station floor")
    floor.init_state.pos = (x, y, -height / 2)
    for side, p in BASES.items():
        state = getattr(cfg, side + "_robot").init_state
        state.pos = p
        state.rot = (0.0, 0.0, 1.0, 0.0)
        cam = getattr(cfg, side + "_cam").offset
        if cam.convention != "opengl":
            raise ValueError("Unexpected wrist convention")
        cam.pos, cam.rot = wrist_pose(cam.pos, cam.rot)
    cfg.top_cam.offset.pos = EYE
    cfg.top_cam.offset.rot = aim(EYE, TARGET)
    cfg.top_cam.offset.convention = "opengl"
    return layout


def readback():
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    stage = omni.usd.get_context().get_stage()

    def pose(n):
        prim = stage.GetPrimAtPath("/World/envs/env_0/" + n)
        if not prim or not prim.IsValid():
            raise ValueError("Missing live prim " + n)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )

    bases = {}
    for s, p in BASES.items():
        m = pose(s.title() + "Arm")
        bases[s] = list(m.ExtractTranslation())
        if (
            math.dist(bases[s], p) > 0.004
            or math.dist(m.TransformDir(Gf.Vec3d(1, 0, 0)), (-1, 0, 0)) > 0.015
        ):
            raise RuntimeError("Base/IK mismatch on " + s)
    camera = pose("TopCam")
    f = np.array(camera.TransformDir(Gf.Vec3d(0, 0, -1)))
    d = np.subtract(TARGET, EYE)
    if (
        math.dist(camera.ExtractTranslation(), EYE) > 0.01
        or np.dot(f, d) / np.linalg.norm(d) < 0.99
    ):
        raise RuntimeError("Policy top camera pose mismatch")
    for name, (expected, _, _) in camera_mount().items():
        if math.dist(pose(name).ExtractTranslation(), expected) > 0.006:
            raise RuntimeError("Camera mount readback differs: " + name)
    wrists = {}
    for s in BASES:
        cameras = [
            p
            for p in stage.Traverse()
            if p.GetName() == s + "_wrist_cam" and p.IsA(UsdGeom.Camera)
        ]
        if len(cameras) != 1:
            raise RuntimeError("Missing wrist camera " + s)
        camera_xf = UsdGeom.Xformable(cameras[0]).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        parent_xf = UsdGeom.Xformable(
            cameras[0].GetParent()
        ).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        eye = np.array(camera_xf.ExtractTranslation())
        target = np.array(parent_xf.Transform(Gf.Vec3d(0.0, 0.0, 0.15)))
        direction = np.array(camera_xf.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0)))
        alignment = float(
            np.dot(direction, target - eye) / np.linalg.norm(target - eye)
        )
        if alignment < 0.99:
            raise RuntimeError("Wrist camera misses pads: " + s)
        wrists[s] = dict(eye=eye.tolist(), alignment=alignment)
    if abs(pose("floor").ExtractTranslation()[2] + 0.01) > 0.003:
        raise RuntimeError("Station floor height mismatch")
    return dict(
        bases=bases,
        wrists=wrists,
        top_eye=list(camera.ExtractTranslation()),
        top_forward=f.tolist(),
    )
