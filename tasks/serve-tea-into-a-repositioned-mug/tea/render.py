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
        for p, v, w in e.packets:
            if w > 0.0008:
                overlay_geom(
                    sc,
                    mj.mjtGeom.mjGEOM_SPHERE,
                    [min(0.0028, 0.0009 + 0.0019 * (w / 0.17) ** (1 / 3)), 0, 0],
                    p,
                    TEA,
                )
        # A spring pin on the cuff exposes its red head after any witness trips.
        base = mug + [0.104, 0.005, 0.066]
        overlay_geom(sc, mj.mjtGeom.mjGEOM_CYLINDER, [0.010, 0.008, 0], base,
                     (0.15, 0.19, 0.20, 1))
        if e.flag_source is not None:
            overlay_geom(sc, mj.mjtGeom.mjGEOM_CYLINDER, [0.004, 0.024, 0],
                         base + [0, 0, 0.030], (0.84, 0.08, 0.04, 1))
            overlay_geom(sc, mj.mjtGeom.mjGEOM_BOX, [0.018, 0.005, 0.013],
                         base + [0, 0, 0.060], (1, 0.035, 0.025, 1))
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
