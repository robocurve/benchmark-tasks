"""Renderer for instrument-free RGB observations and the review video."""

import mujoco as mj
import numpy as np

TEA = (0.68, 0.28, 0.047, 0.95)


def overlay_geom(scene, kind, size, pos, color):
    if scene.ngeom >= scene.maxgeom:
        raise RuntimeError("Visualization capacity exhausted")
    g = scene.geoms[scene.ngeom]
    mj.mjv_initGeom(
        g,
        kind,
        np.array(size),
        np.array(pos),
        np.eye(3).flatten(),
        np.array(color, dtype="f"),
    )
    scene.ngeom += 1


class Camera:
    def __init__(self, env, width, height):
        self.env = env
        self.renderer = mj.Renderer(env.m, height=height, width=width)
        # MuJoCo's default visibility mask hides groups 3 and 4: those are the human and furniture.
        self.opt = mj.MjvOption()
        self.opt.geomgroup[:] = 1

    def frame(self, name):
        e = self.env
        r = self.renderer
        r.update_scene(e.d, camera=name, scene_option=self.opt)
        sc = r.scene
        mug = e.d.body("receiving_hand").xpos
        h = 0.078 * e.cup / 260
        if h > 0.0003:
            overlay_geom(
                sc,
                mj.mjtGeom.mjGEOM_CYLINDER,
                [0.0325, h / 2, 0],
                mug + [0, 0, 0.006 + h / 2],
                TEA,
            )
        # Cosmetic liquid surface in the server, displayed upright only: at steep tilt the stream is the witness.
        if e.d.xmat[e.pot].reshape(3, 3)[2, 2] > 0.96 and e.remaining > 1:
            h = 0.084 * e.remaining / 300
            overlay_geom(
                sc,
                mj.mjtGeom.mjGEOM_CYLINDER,
                [0.032, h / 2, 0],
                e.d.xpos[e.pot] + [0, 0, 0.005 + h / 2],
                TEA,
            )
        # Every emitted drop is visible in the evaluator's camera model. Its
        # radius represents its water volume, independently of any outcome label.
        for p, _, w in e.packets:
            radius = (3 * w * 1e-6 / (4 * np.pi)) ** (1 / 3)
            overlay_geom(sc, mj.mjtGeom.mjGEOM_SPHERE, [radius, 0, 0], p, TEA)
        if e.last_skin_point is not None:
            # Attached blot follows the hand after wetting. Its presence conveys wetness, not dose.
            overlay_geom(
                sc,
                mj.mjtGeom.mjGEOM_SPHERE,
                [0.0045, 0, 0],
                e.d.xpos[e.last_skin_point[0]]
                + e.d.xmat[e.last_skin_point[0]].reshape(3, 3) @ e.last_skin_point[1],
                (0.46, 0.16, 0.025, 1.0),
            )
        return r.render().copy()

    def close(self):
        self.renderer.close()
