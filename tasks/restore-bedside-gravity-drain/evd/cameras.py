import numpy as np
from scipy.spatial.transform import Rotation

EYE = (-0.070, -0.040, 1.310)
TARGET = (0.435, 0.053, 1.035)


def aim(eye, target, up=(0, 0, 1)):
    z = np.subtract(eye, target)
    z = z / np.linalg.norm(z)
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    return tuple(
        Rotation.from_matrix(np.column_stack([x, np.cross(z, x), z])).as_quat()
    )


def wrist(side, pos, quat):
    rot = Rotation.from_quat(quat)
    # Optical centres are expressed in the gripper link frame. The fixed left
    # bracket puts the lens outboard of the adjacent brake hand on every layout.
    if side == "left":
        p = np.array([0.0, 0.085, 0.070])
    elif side == "right":
        p = np.asarray(pos) + rot.apply([0, 0, 0.045])
    else:
        raise ValueError(f"Unknown wrist side: {side}")
    return tuple(p), aim(p, (0, 0, 0.146), rot.apply([0, 1, 0]))


def configure(cfg):
    from yam_arena.station import _segment

    quat = aim(EYE, TARGET)
    back = Rotation.from_quat(quat).apply([0, 0, 1])
    housing = np.asarray(EYE) + back * 0.033
    top = np.array([-0.345, housing[1] - 0.13, housing[2]])
    bottom = (top[0], top[1], -0.0005)
    parts = {
        "stand_foot": (bottom, (0, 0, 0, 1), (0.22, 0.22, 0.024)),
        "stand_mast": _segment((*bottom[:2], 0.0115), top),
        "stand_arm": _segment(top, housing),
        "camera_housing": (housing, quat, (0.050, 0.042, 0.022)),
    }
    for n, (pos, rot, dim) in parts.items():
        obj = getattr(cfg, n)
        obj.init_state.pos = tuple(map(float, pos))
        obj.init_state.rot = tuple(map(float, rot))
        if isinstance(dim, tuple):
            obj.spawn.size = dim
        else:
            obj.spawn.height = dim
    cfg.top_cam.offset.pos = EYE
    cfg.top_cam.offset.rot = quat
    cfg.top_cam.offset.convention = "opengl"
    for s in ("left", "right"):
        cam = getattr(cfg, s + "_cam").offset
        if cam.convention != "opengl":
            raise RuntimeError("Unexpected wrist convention")
        cam.pos, cam.rot = wrist(s, cam.pos, cam.rot)
