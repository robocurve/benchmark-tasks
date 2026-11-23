"""Camera and joint inputs for a new policy. Scorer channels stay in the evaluator."""

import mujoco as mj
import numpy as np


def camera(lookat, distance, azimuth, elevation):
    cam = mj.MjvCamera()
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation
    return cam


def action_vector(model, value):
    vec = np.asarray(value, dtype=np.float64)
    if vec.shape != (8,) or not np.isfinite(vec).all():
        raise ValueError("Return eight finite absolute joint targets")
    lo, hi = model.actuator_ctrlrange.T
    if (vec < lo - 1e-6).any() or (vec > hi + 1e-6).any():
        raise ValueError("Joint target outside YAM control limits")
    return np.clip(vec, lo, hi)


class Cameras:
    def __init__(self, model):
        self.renderer = mj.Renderer(model, height=240, width=384)
        self.views = {
            "bedside": camera([0.37, 0.04, 0.10], 1.10, -42, -16),
            "heel_side": camera([0.42, -0.045, 0.08], 0.69, 5, -12),
        }

    def observe(self, data, instruction):
        frames = {}
        for name, cam in self.views.items():
            self.renderer.update_scene(data, cam)
            frames[name] = self.renderer.render().copy()
        return {
            "rgb": frames,
            "joint_position": data.qpos[:8].copy(),
            "instruction": instruction,
        }

    def close(self):
        self.renderer.close()
