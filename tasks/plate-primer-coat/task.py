"""Fabrication shop: "Give the steel plate a coat of primer."

A bare steel plate, a tray of red-oxide primer and a block brush with flexible bristle tufts. Nothing
names the brush or the tray: the brush has to be dipped before it puts anything on the plate, a
freshly dipped brush drips until the excess is wiped off on the tray's rim, and the coat goes on
where the loaded bristle tips touch the plate.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

import simkit  # noqa: E402
import paint as P  # noqa: E402
from scoring import TrialScorer  # noqa: E402

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP as PAD_TIP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "plate-primer-coat"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
MAX_STEPS = 1200
OBJECT_NAMES = ("brush", *P.TUFT_NAMES, *P.DROP_NAMES)

DOWN = simkit.patch_demo()


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    top_down_overview(embodiment)

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    specs = (P.plate_object(), P.tray_object(), P.brush_object(), *(P.tuft_object(n) for n in P.TUFT_NAMES),
             P.splat_object(), *P.drop_objects())
    objects += [yscene.task_object(s) for s in specs]
    cfg = IsaacLabArenaEnvironment(name="yam_plate_primer_coat", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


class Dynamics:
    """The primer, by volume, from measured poses only (a disclosed stand-in for wet paint).

    Bristle tips sunk in the tray's primer load the brush to ``PICKUP_ML``. The bristles hold
    ``HOLD_ML``; held up out of the tray, the excess runs down to the lowest tuft tip and falls as
    drops of ``DROP_ML`` (small rigid spheres released there), faster the more there is. Drawing the
    tips across the tray's far rim scrapes the excess back into the tray. Every tuft tip touching the
    plate's top while the brush is loaded wets the 10 mm patches under it, each taking ``ML_PER_TILE``.
    A wet patch is shown and stays wet. A drop that comes to rest is drawn as a flat splat; one off the
    plate and tray counts as a drip on the bench. Loaded tips flat on the bench count as smearing it."""

    def __init__(self, env):
        import omni.usd
        self.base = env.unwrapped
        stage = omni.usd.get_context().get_stage()
        self.tiles = [[None] * len(P.COAT_LAYERS) for _ in range(P.TILES)]
        self.splats = [None] * P.DROPS
        self.wet = {}
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            for k in range(P.TILES):
                for layer in range(len(P.COAT_LAYERS)):
                    if path.endswith(f"/plate/piece{P.tile_piece(k, layer)}"):
                        self.tiles[k][layer] = prim
            for k in range(P.DROPS):
                if path.endswith(f"/splats/piece{k}"):
                    self.splats[k] = prim
            for n in P.TUFT_NAMES:
                if path.endswith(f"/{n}/piece0"):
                    self.wet[n] = prim
        if any(t is None for row in self.tiles for t in row) or any(t is None for t in self.splats) \
                or len(self.wet) != len(P.TUFT_NAMES):
            raise RuntimeError("primer tiles, splats or wet bristles not found on the stage")

    def _show(self, prim, on):
        from pxr import UsdGeom
        img = UsdGeom.Imageable(prim)
        img.MakeVisible() if on else img.MakeInvisible()

    def _place_splat(self, k, xyz):
        from pxr import Gf, UsdGeom
        local = Gf.Vec3d(*(float(v) for v in np.asarray(xyz) - np.array([0.05, -0.60, P.TOP + 0.001])))
        ops = [op for op in UsdGeom.Xformable(self.splats[k]).GetOrderedXformOps()
               if op.GetOpType() == UsdGeom.XformOp.TypeTranslate]
        if not ops:
            raise RuntimeError("splat has no translate op")
        attr = ops[0].GetAttr()
        attr.Set(Gf.Vec3f(*local) if attr.GetTypeName() == "float3" else local)
        self._show(self.splats[k], True)

    def _write(self, name, pos):
        import torch
        obj = self.base.scene[name]
        origin = self.base.scene.env_origins[0].cpu().numpy()
        pose = torch.tensor([[*(np.asarray(pos) + origin), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=self.base.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=torch.float32, device=self.base.device))

    def reset(self):
        self.free = list(range(P.DROPS))
        self.falling = {}
        self.drips = 0
        self.smear_steps = 0
        self.splatted = 0
        for i, n in enumerate(P.DROP_NAMES):
            self._write(n, P.parking(i))
        for prim in self.splats:
            self._show(prim, False)
        self.load = 0.0
        self.run_off = 0.0
        self.dips = 0
        self.dipped = False
        self.film = np.zeros(P.TILES)
        self.tip = 0.0
        self.shown_layer = [None] * P.TILES
        for k in range(P.TILES):
            self._layer(k)
        self.shown_wet = None
        self._wet(False)

    def _layer(self, k):
        """Draw patch k at the thickest coat layer its film has reached (none below the first)."""
        layer = max([i for i, (frm, _) in enumerate(P.COAT_LAYERS) if self.film[k] >= frm], default=-1)
        if self.shown_layer[k] != layer:
            for i, prim in enumerate(self.tiles[k]):
                self._show(prim, i == layer)
            self.shown_layer[k] = layer

    def _wet(self, on):
        if self.shown_wet != on:
            for prim in self.wet.values():
                self._show(prim, on)
            self.shown_wet = on

    def before_step(self, step, action):
        pass

    def _drops(self, step, pose, tips_b, objects, root_poses):
        tips, _, _ = P.bristle_tips(pose)
        excess = self.load - P.HOLD_ML
        if excess > 0 and not P.in_primer(pose):
            self.run_off += excess * P.RUN_OFF_PER_S / HZ
            if P.on_rim(pose):
                self.load = P.HOLD_ML               # scraped back into the tray
                self.run_off = 0.0
            elif self.run_off >= P.DROP_ML:
                self.run_off -= P.DROP_ML
                self.load -= P.DROP_ML
                if not P.over_tray(tips) and self.free:
                    low = min(tips_b, key=lambda t: t[2])
                    k = self.free.pop(0)
                    self._write(P.DROP_NAMES[k], low - np.array([0.0, 0.0, P.DROP_R + 0.001]))
                    self.falling[k] = step
        for k, t0 in list(self.falling.items()):
            n = P.DROP_NAMES[k]
            p = np.asarray(root_poses[n], float)[:3]
            if step - t0 > 2 and float(np.linalg.norm(objects[n].lin_vel)) < 0.03:
                del self.falling[k]
                surface = P.PLATE_TOP if P.on_plate(p) else P.TOP
                self._place_splat(k, (p[0], p[1], surface + 0.0004))
                self._write(n, P.parking(k))           # the drop has spread out flat
                self.splatted += 1
                if not (P.on_plate(p) or P.over_tray(p)):
                    self.drips += 1

    def after_step(self, step, objects, root_poses, robot):
        pose = np.asarray(root_poses["brush"], float)
        tips_b = [P.tuft_tip(root_poses[n])[0] for n in P.TUFT_NAMES if n.endswith("b")]
        if P.in_primer(pose):
            if not self.dipped:
                self.dips += 1
            self.dipped = True
            self.load = P.PICKUP_ML
            self.tip = P.TIP_CAP_ML
            self.run_off = 0.0
        else:
            self.dipped = False
        self._drops(step, pose, tips_b, objects, root_poses)
        if self.load > 0.05 and any(P.tip_on_bench(t) for t in tips_b):
            self.smear_steps += 1                 # loaded bristle tips dragged on the bench
        # the tips fill from the body of the brush, then lay what they carry on the patches they touch
        self.tip = min(P.TIP_CAP_ML, self.load, self.tip + P.TIP_FEED_ML_S / HZ)
        touched = sorted({k for tip in tips_b for k in P.tiles_touched(tip)})
        if touched and self.tip > 1e-6:
            strength = self.tip / P.TIP_CAP_ML
            for k in touched:
                lay = min(1.0 - self.film[k], P.LAY_PER_STEP * strength)
                ml = min(lay * P.ML_PER_TILE, self.tip)
                if ml <= 0:
                    continue
                self.film[k] += ml / P.ML_PER_TILE
                self.tip -= ml
                self.load -= ml
                self._layer(k)
        self._wet(self.load > 0.05)
        bend = [P.tuft_tip(root_poses[n])[1] for n in P.TUFT_NAMES if n.endswith("b")]
        coated = [k for k in range(P.TILES) if self.film[k] >= P.FULL_COAT]
        return {"painted": coated, "coverage": len(coated) / P.TILES,
                "thin_patches": int(sum(1 for f in self.film if 0.15 <= f < P.FULL_COAT)),
                "load_ml": round(float(self.load), 3), "tip_ml": round(float(self.tip), 3),
                "dips": int(self.dips), "drips": int(self.drips), "drops_landed": int(self.splatted),
                "bench_smear_steps": int(self.smear_steps), "max_bristle_bend_deg": round(float(max(bend)), 1)}


def make_dynamics(env):
    return Dynamics(env)


OVERVIEW_EYE = (0.29, 0.08, 1.58)      # straight above the bench, image up = away from the arms


def top_down_overview(embodiment):
    """Point the recording-only overview camera straight down on the bench (the model never sees it)."""
    from yam_arena import ik
    cam = embodiment.camera_config.overview_cam
    forward, left, up = np.array([0.0, 0.0, -1.0]), np.array([0.0, 1.0, 0.0]), np.array([1.0, 0.0, 0.0])
    cam.offset.pos = tuple(float(v) for v in OVERVIEW_EYE)
    cam.offset.rot = tuple(float(v) for v in ik.matrix_to_quat_xyzw(np.column_stack([forward, left, up])))


def make_scorer():
    return TrialScorer()


# -- demo ----------------------------------------------------------------------------------

HANDLE_FINGER = -(2 * P.HANDLE_R - 0.013) / 2      # squeeze: the brush with its tufts weighs ~0.1 kg
FAR_X = P.PLATE_XY[0] + P.PLATE[0] / 2 - P.BRISTLE_BLOCK[0] / 2 + 0.001   # set down just inside the far edge: pressed bristles hanging over it would curl onto the bench
NEAR_X = P.PLATE_XY[0] - P.PLATE[0] / 2 + P.BRISTLE_BLOCK[0] / 2 - 0.004  # last patches covered
STROKE_Z = P.PLATE_TOP - 0.002          # tips pressed 2 mm: the tufts bend back and drag flat
DIP_Z = P.TOP + P.TRAY_WALL + 0.003


def _hand(seconds, tips_xyz, finger, label):
    """Down grip on the handle, jaws across it, given where the bristle tips' centre should be."""
    x, y, z = tips_xyz
    tip = (x + P.GRIP_X, y, z + P.HANDLE_Z - 0.004)
    return Waypoint(seconds, (tip[0], tip[1], tip[2] + PAD_TIP), finger, label, DOWN)


def demo():
    bx, by = P.BRUSH_XY
    tx, ty = P.TRAY_XY
    f = HANDLE_FINGER
    left = [
        Waypoint(2.0, (0.22, 0.22, P.TOP + 0.18), FINGER_OPEN, "look at the plate", DOWN),
        _hand(2.0, (bx, by, P.TOP + 0.06), FINGER_OPEN, "over the brush"),
        _hand(1.5, (bx, by, P.TOP), FINGER_OPEN, "down round the handle"),
        _hand(1.0, (bx, by, P.TOP), f, "take the brush"),
        _hand(1.5, (bx, by, P.TOP + 0.06), f, "pick it up"),
    ]
    left += [
        _hand(1.8, (tx, ty, P.TOP + 0.05), f, "over the primer"),
        _hand(1.2, (tx, ty, DIP_Z), f, "dip the brush"),
        _hand(0.8, (tx, ty, DIP_Z), f, "let it soak up"),
        _hand(1.0, (tx, ty, P.TOP + 0.05), f, "lift it out"),
        _hand(1.0, (P.WIPE_WALL_X - 0.012, ty, P.RIM_TOP + 0.012), f, "over the far rim"),
        _hand(0.8, (P.WIPE_WALL_X - 0.004, ty, P.RIM_TOP - 0.001), f, "tips on the rim"),
        _hand(1.0, (P.WIPE_WALL_X + 0.006, ty, P.RIM_TOP - 0.001), f, "wipe the excess off on it"),
        _hand(0.6, (P.WIPE_WALL_X + 0.006, ty, P.RIM_TOP + 0.03), f, "lift off the rim"),
    ]
    for k, dy in enumerate((0.004, -0.004)):
        left += [
            _hand(1.8 if k == 0 else 1.2, (FAR_X, dy, STROKE_Z + 0.03), f, "over the far edge of the plate"),
            _hand(1.0, (FAR_X, dy, STROKE_Z), f, "bristles down, pressed so they bend"),
            _hand(3.5, (NEAR_X, dy, STROKE_Z), f, "draw the brush along the plate"),
            _hand(0.8, (NEAR_X, dy, STROKE_Z + 0.03), f, "lift"),
        ]
    left += [
        _hand(2.2, (tx, ty, P.TOP + 0.05), f, "back over the tray"),
        _hand(1.5, (tx, ty, P.TOP + P.TRAY_WALL + 0.002), f, "rest the wet brush in the tray"),
        _hand(1.0, (tx, ty, P.TOP + P.TRAY_WALL + 0.002), FINGER_OPEN, "let go"),
        _hand(1.5, (tx, ty, P.TOP + 0.08), FINGER_OPEN, "clear"),
        _hand(2.0, (tx, ty, P.TOP + 0.08), FINGER_OPEN, "stand clear"),
    ]
    return {"left": left}
