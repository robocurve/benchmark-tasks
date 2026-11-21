"""hhdeco: visual-only scene dressing added to the stage at the first control step (no colliders, no mass, so it can
never touch the robot or change physics or scoring). Three kinds:

- static pieces (box / cylinder / sphere) drawn once, in the env's frame;
- cables: a chain of thin cylinders along a cubic Bezier between two points, redrawn every control step (one end may
  follow a loose body, e.g. a torch's hose stub), lying on the bench where the curve would go below it;
- switches: pieces shown only while a flag is true (e.g. the arc's glow while the arc is on, the tack bead after it).

Everything lives under /World/hh_deco. Any failure here is printed once and ignored: it only removes dressing.
"""

from __future__ import annotations

import math
import sys

import numpy as np

ROOT = "/World/hh_deco"


def _qz_to(v):
    """wxyz quaternion turning +z onto unit vector v."""
    v = np.asarray(v, float)
    v = v / (np.linalg.norm(v) + 1e-12)
    d = float(v[2])
    if d < -0.999999:
        return (0.0, 1.0, 0.0, 0.0)
    c = np.cross([0.0, 0.0, 1.0], v)
    q = np.array([1.0 + d, *c])
    q /= np.linalg.norm(q)
    return tuple(float(x) for x in q)


def bezier(p0, p1, p2, p3, n):
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return ((1 - t) ** 3) * p0 + 3 * ((1 - t) ** 2) * t * p1 + 3 * (1 - t) * t * t * p2 + (t ** 3) * p3


class Deco:
    def __init__(self, floor_z):
        self.floor_z = float(floor_z)
        self.static = []  # (name, kind, dims, xyz, quat_wxyz, look)
        self.looks = {}  # name -> (color, rough, metallic, emissive)
        self.cables = []  # (name, radius, look, ends_fn, segments)
        self.switches = []  # (name, flag_fn)
        self._ready = self._failed = False
        self._stage = self._origin = None
        self._seg = {}
        self._vis = {}

    # -- authoring -------------------------------------------------------------------------------------------------
    def look(self, name, color, rough=0.5, metallic=0.0, emissive=None):
        self.looks[name] = (tuple(color), rough, metallic, emissive)
        return name

    def piece(self, name, kind, dims, xyz, look, quat_wxyz=(1.0, 0.0, 0.0, 0.0), show_if=None):
        self.static.append((name, kind, tuple(dims), tuple(float(v) for v in xyz), tuple(quat_wxyz), look))
        if show_if is not None:
            self.switches.append((name, show_if))

    def cable(self, name, radius, look, ends_fn, segments=16):
        """ends_fn(env, origin) -> (p0, p1, p2, p3) Bezier control points in the env's frame."""
        self.cables.append((name, float(radius), look, ends_fn, int(segments)))

    # -- run time --------------------------------------------------------------------------------------------------
    def step(self, env, flags):
        if self._failed or env is None:
            return
        try:
            if not self._ready:
                self._build(env)
            self._update(env, flags)
        except Exception as e:  # dressing only: never break a trial
            self._failed = True
            print(f"[hhdeco] disabled: {type(e).__name__}: {e}", file=sys.stderr)

    def _build(self, env):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils import bind_visual_material
        from pxr import Gf, UsdGeom

        from yam_arena.station import linear_rgb

        base = env.unwrapped
        stage = base.sim.stage
        self._stage = stage
        self._origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        if not stage.GetPrimAtPath(ROOT).IsValid():
            UsdGeom.Xform.Define(stage, ROOT)
        for name, (color, rough, metal, emis) in self.looks.items():
            path = f"{ROOT}/Looks/{name}"
            if stage.GetPrimAtPath(path).IsValid():
                continue
            kw = dict(diffuse_color=linear_rgb(color), roughness=rough, metallic=metal)
            if emis is not None:
                kw["emissive_color"] = tuple(float(v) for v in emis)
            cfg = sim_utils.PreviewSurfaceCfg(**kw)
            cfg.func(path, cfg)
        o = self._origin
        for name, kind, dims, xyz, quat, look in self.static:
            path = f"{ROOT}/{name}"
            if stage.GetPrimAtPath(path).IsValid():
                continue
            if kind == "box":
                g = UsdGeom.Cube.Define(stage, path)
                g.CreateSizeAttr(1.0)
            elif kind == "cyl":
                g = UsdGeom.Cylinder.Define(stage, path)
                g.CreateRadiusAttr(1.0)
                g.CreateHeightAttr(1.0)
                g.CreateAxisAttr("Z")
            else:
                g = UsdGeom.Sphere.Define(stage, path)
                g.CreateRadiusAttr(1.0)
            g.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(np.asarray(xyz) + o)))
            g.AddOrientOp().Set(Gf.Quatf(*quat))
            if kind == "box":
                scale = dims
            elif kind == "cyl":
                scale = (dims[0], dims[0], dims[1])
            else:
                scale = dims if len(dims) == 3 else (dims[0],) * 3
            g.AddScaleOp().Set(Gf.Vec3f(*[float(v) for v in scale]))
            bind_visual_material(path, f"{ROOT}/Looks/{look}", stage=stage)
            self._vis[name] = UsdGeom.Imageable(g.GetPrim())
        for name, r, look, _fn, n in self.cables:
            segs = []
            for i in range(n):
                path = f"{ROOT}/{name}/s{i:02d}"
                g = UsdGeom.Cylinder.Define(stage, path)
                g.CreateRadiusAttr(1.0)
                g.CreateHeightAttr(1.0)
                g.CreateAxisAttr("Z")
                ops = (g.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble), g.AddOrientOp(), g.AddScaleOp())
                bind_visual_material(path, f"{ROOT}/Looks/{look}", stage=stage)
                segs.append(ops)
            self._seg[name] = segs
        self._ready = True
        print(f"[hhdeco] built {len(self.static)} pieces, {len(self.cables)} cables", file=sys.stderr)

    def _update(self, env, flags):
        from pxr import Gf

        o = self._origin
        for name, r, _look, fn, n in self.cables:
            pts = bezier(*[np.asarray(p, float) for p in fn(env, o)], n)
            pts[1:-1, 2] = np.maximum(pts[1:-1, 2], self.floor_z + r)
            for (tr, ori, sc), a, b in zip(self._seg[name], pts[:-1], pts[1:]):
                d = b - a
                L = float(np.linalg.norm(d)) + 1e-6
                tr.Set(Gf.Vec3d(*((a + b) / 2 + o)))
                ori.Set(Gf.Quatf(*_qz_to(d)))
                sc.Set(Gf.Vec3f(r, r, L + 1.2 * r))  # overlap the joints a little
        for name, fn in self.switches:
            on = bool(fn(flags))
            if self._vis.get(name) is not None and on != getattr(self, f"_on_{name}", None):
                (self._vis[name].MakeVisible if on else self._vis[name].MakeInvisible)()
                setattr(self, f"_on_{name}", on)


def body_pose(env, name, origin):
    """A rigid object's frame origin (env frame) and rotation matrix, read from the simulator."""
    obj = env.unwrapped.scene[name]
    p = obj.data.root_pos_w[0].detach().cpu().numpy().astype(float) - origin
    w, x, y, z = (float(v) for v in obj.data.root_quat_w[0].detach().cpu().numpy())
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return p, R


assert math
