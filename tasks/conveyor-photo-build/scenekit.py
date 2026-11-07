"""Turns the task's ``belt`` box into a running conveyor, puts the photo on the ``photo_card`` box and draws the
round posts round.
Imports Isaac only after the simulator starts.

The belt is a kinematic slab whose surface carries what rests on it toward -y at ``SPEED``
(PhysX surface velocity). PhysX applies surface velocity through contact modification, which runs
only in the CPU pipeline, so the task needs ``-E device=cpu`` (the cameras still render on the GPU).

The card is a fixed board leaning back ``CARD_TILT``; ``photo.png`` is mapped onto its front face as a
textured quad (UsdPreviewSurface), visual only. The round posts are drawn as cylinders over hidden box colliders
(see ``_round_look``).

It wraps ``yam_arena.scene.task_object`` when that module loads, without modifying yam_arena, so the
task runs with the standard ``yam_arena`` embodiment, ``yam_scripted`` policy and ``task_score`` scorer.
"""
from __future__ import annotations

import importlib.abc
import importlib.machinery
import os
from pathlib import Path
import sys
from functools import wraps

from layout import CARD_SIZE, CYLINDERS, PARTS, PHOTO_FILE, SPEED, card_quat_xyzw

BELT = 'belt'
CARD = 'photo_card'
PHOTO = Path(__file__).resolve().parent / PHOTO_FILE
_installed = False
_task_path = None
_belt_prims: list[str] = []


def _put_photo(prim) -> None:
    """A quad 0.2 mm in front of the card's front (-x) face, the photo mapped right way up as seen from in front."""
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    stage = prim.GetStage()
    root = prim.GetPath()
    x = -(CARD_SIZE[0] / 2 + 0.0002)
    w, h = CARD_SIZE[1] / 2, CARD_SIZE[2] / 2
    quad = UsdGeom.Mesh.Define(stage, root.AppendChild('photo'))
    # Seen from in front (from -x, looking toward +x) the viewer's right is -y: u runs from +y to -y.
    quad.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(x, w, -h), Gf.Vec3f(x, -w, -h), Gf.Vec3f(x, -w, h), Gf.Vec3f(x, w, h)]))
    quad.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    quad.CreateFaceVertexIndicesAttr(Vt.IntArray([0, 1, 2, 3]))
    quad.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(-1, 0, 0)] * 4))
    quad.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    quad.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(quad).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(1, 0), Gf.Vec2f(1, 1), Gf.Vec2f(0, 1)]))

    material = UsdShade.Material.Define(stage, root.AppendChild('PhotoLook'))
    surface = UsdShade.Shader.Define(stage, material.GetPath().AppendChild('Surface'))
    surface.CreateIdAttr('UsdPreviewSurface')
    surface.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.7)
    surface.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.0)
    reader = UsdShade.Shader.Define(stage, material.GetPath().AppendChild('St'))
    reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
    texture = UsdShade.Shader.Define(stage, material.GetPath().AppendChild('Photo'))
    texture.CreateIdAttr('UsdUVTexture')
    texture.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(str(PHOTO))
    texture.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
    texture.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
    rgb = texture.CreateOutput('rgb', Sdf.ValueTypeNames.Float3)
    surface.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(quad.GetPrim()).Bind(material)


def _make_belt(prim) -> None:
    from pxr import Gf, PhysxSchema, UsdPhysics

    body = UsdPhysics.RigidBodyAPI.Apply(prim)
    body.CreateKinematicEnabledAttr().Set(True)
    surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
    surface.CreateSurfaceVelocityEnabledAttr().Set(True)
    surface.CreateSurfaceVelocityLocalSpaceAttr().Set(False)
    surface.CreateSurfaceVelocityAttr().Set(Gf.Vec3f(0.0, -float(SPEED), 0.0))
    _belt_prims.append(str(prim.GetPath()))


def _round_look(prim, size) -> None:
    """A round post: its box stays the collider (hidden), and a cylinder of the same width and height, with the
    box's colour, is drawn in its place. PhysX's convex cylinders rocked and tipped on the running belt; upright,
    a 40 mm square prism stands, slides, is gripped across 40 mm and carries a beam as the round post does."""
    from pxr import Usd, UsdGeom, UsdShade

    stage = prim.GetStage()
    box = next(p for p in Usd.PrimRange(prim) if p.IsA(UsdGeom.Gprim))
    material, _ = UsdShade.MaterialBindingAPI(box).ComputeBoundMaterial()
    UsdGeom.Imageable(box).MakeInvisible()
    look = UsdGeom.Cylinder.Define(stage, prim.GetPath().AppendChild('round_look'))
    look.CreateAxisAttr('Z')
    look.CreateRadiusAttr(size[0] / 2)
    look.CreateHeightAttr(size[2])
    if material:
        UsdShade.MaterialBindingAPI.Apply(look.GetPrim()).Bind(material)


def _patch_scene(module) -> None:
    original = module.task_object

    @wraps(original)
    def task_object(spec):
        obj = original(spec)
        if spec.name not in (BELT, CARD, *CYLINDERS):
            return obj
        from isaaclab.sim.spawners.shapes import spawn_cuboid
        from isaaclab.sim.utils import clone
        from isaaclab_arena.utils.pose import Pose

        cfg = obj.spawner_cfg
        studio = os.environ.get('CONVEYOR_STUDIO') == '1'  # taking the photo: the card stays blank
        if spec.name == BELT:
            dress = _make_belt
        elif spec.name == CARD:
            dress = (lambda prim: None) if studio else _put_photo
        else:
            size = PARTS[spec.name][0]
            dress = lambda prim: _round_look(prim, size)  # noqa: E731
        pose = obj.initial_pose
        if spec.name == CARD:  # task files can only turn a box about the vertical; the card leans back
            pose = Pose(position_xyz=tuple(spec.pos), rotation_xyzw=card_quat_xyzw())

        @clone
        def spawn(path, config, *args, **kwargs):
            prim = spawn_cuboid.__wrapped__(path, config, *args, **kwargs)
            dress(prim)
            return prim

        cfg.func = spawn
        # Object caches its spawner config when built; rebuild it so the part spawns this way on every build.
        return module.Object(name=spec.name, object_type=obj.object_type, spawner_cfg=cfg, initial_pose=pose)

    module.task_object = task_object


class _SceneFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != 'yam_arena.scene':
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is None or spec.loader is None:
            return None
        loader = spec.loader

        class Loader(importlib.abc.Loader):
            def create_module(self, module_spec):
                return loader.create_module(module_spec)

            def exec_module(self, module):
                loader.exec_module(module)
                _patch_scene(module)

        spec.loader = Loader()
        return spec


def install(task_path) -> None:
    """Patch yam_arena once per process; refuse GPU physics, where the belt would not run."""
    global _installed, _task_path
    _task_path = task_path
    if _installed:
        return
    _installed = True
    if 'yam_arena.scene' in sys.modules:
        _patch_scene(sys.modules['yam_arena.scene'])
    else:
        sys.meta_path.insert(0, _SceneFinder())
    try:
        from yam_arena.inspect_embodiment import YamArenaEmbodiment
    except ImportError:  # offline checks without yam_arena's embodiment
        return
    # The task file loads inside the embodiment's __init__, so check the device when a trial starts.
    original_reset = YamArenaEmbodiment.reset

    @wraps(original_reset)
    def reset(emb, scene, *, seed=None):
        if emb.task is not None and emb.task.path == _task_path:
            if not str(emb.device).startswith('cpu'):
                raise ValueError('The conveyor needs CPU physics (PhysX surface velocity): add -E device=cpu.')
            if emb.object_jitter != 0:
                raise ValueError('The parts start in a fixed line on the belt: add -E object_jitter=0.')
        return original_reset(emb, scene, seed=seed)

    YamArenaEmbodiment.reset = reset
