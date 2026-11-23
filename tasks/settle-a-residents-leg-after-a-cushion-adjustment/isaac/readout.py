"""Read actual native body poses with the same geometric witness model used in development."""

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation as R
from task.station import make_model
from task.witness import Witness

ORIGIN = np.array([0.185, -0.305, 0.725])
LINKS = (
    "base",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "gripper",
    "tip_left",
    "tip_right",
)
BODIES = ("frame", "upper_leg", "shin", "cushion", *(f"foam{i}" for i in range(6)))


class Readout:
    def __init__(self, settings):
        self.m = make_model(
            dual=True, **{k: v for k, v in settings.items() if k != "delay"}
        )
        self.d = mj.MjData(self.m)
        self.reader = Witness(self.m)
        self.names = []
        for i in self.m.geom_bodyid:
            name = self.m.body(i).name
            self.names.append(
                "frame"
                if name == "world"
                else "right/" + name
                if name in LINKS
                else "left/" + name[6:]
                if name.startswith("other_")
                else name
            )
        self.rot = R.from_quat(self.m.geom_quat[:, [1, 2, 3, 0]]).as_matrix()

    def sample(self, raw):
        p = raw["poses"]
        m, d = self.m, self.d
        needed = set(BODIES) | {s + "/" + n for s in ("left", "right") for n in LINKS}
        if set(p) != needed:
            raise ValueError("Incomplete station transforms")
        arr = np.array([p[name] for name in self.names])
        if (
            arr.shape != (m.ngeom, 7)
            or not np.isfinite(arr).all()
            or np.max(abs(np.linalg.norm(arr[:, 3:], axis=1) - 1)) > 0.002
        ):
            raise ValueError("Invalid station pose")
        axes = R.from_quat(arr[:, 3:]).as_matrix()
        d.geom_xpos[:] = arr[:, :3] - ORIGIN + np.einsum("ijk,ik->ij", axes, m.geom_pos)
        d.geom_xmat[:] = (axes @ self.rot).reshape(-1, 9)
        base = np.asarray(p["cushion"])
        local = R.from_quat(base[3:]).inv()
        for i in range(6):
            rel = local.apply(np.asarray(p[f"foam{i}"])[:3] - base[:3])
            d.joint(f"compression{i}").qpos[0] = rel[2] - 0.038
        w = self.reader.sample(
            m, d, raw["heel_n"], raw["assist_nm"], arms=("arm_", "other_")
        )
        # For the scripted controller only. Learned policies see station cameras and proprioception.
        heel = m.geom("heel").id
        mat = d.geom_xmat[heel].reshape(3, 3)
        bottom = 1000 * (
            d.geom_xpos[heel, 2] - np.linalg.norm(mat[2] * m.geom_size[heel])
        )
        pose = p["right/gripper"]
        tcp = np.asarray(pose[:3]) - ORIGIN + R.from_quat(pose[3:]).apply([0, 0, 0.120])
        return w, dict(
            heel_gap_mm=bottom,
            tcp=tcp.tolist(),
            settled_support=any(w["calf_rows"]) and abs(raw["knee_rate"]) < 0.08,
        )
