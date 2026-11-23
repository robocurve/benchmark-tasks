"""Shared helpers: z-up metre stages, invisible box colliders, merged pc-anatomy visual meshes."""

import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

# Millimetre-scale clearances need a thin contact shell; PhysX's defaults are sized for larger objects.
CONTACT_OFFSET = 0.0002
REST_OFFSET = 0.0

# The tasks that get the DIMM and board models, each with the slot whose latches it drives (0..3: A1, A2, B1, B2;
# "pcie": the x16 slot's). make_dimm.py and make_board.py write the models into every one of them. The three memory
# module tasks are closed and keep the models they were tested with, so they are no longer listed.
TASKS = {"install-graphics-card": "pcie", "replace-graphics-card": "pcie-nobutton"}
DIMM_TASKS = [name for name, live in TASKS.items() if not live.startswith("pcie")]
REPO = Path(__file__).resolve().parents[1]


def task_assets(name):
    return REPO / "tasks" / name / "assets"


def new_stage(path):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    return stage


def physics_scene(stage, path):
    """A physics scene stepping at 240 Hz, as yam_arena does."""
    prim = UsdPhysics.Scene.Define(stage, path).GetPrim()
    prim.AddAppliedSchema("PhysxSceneAPI")
    prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(240)


def collider_box(stage, path, lo, hi, color=None):
    """A box collider spanning corners lo..hi (metres); invisible unless given a display colour."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2)))
    cube.AddScaleOp().Set(Gf.Vec3f(*(hi - lo)))
    if color is None:
        cube.MakeInvisible()
    else:
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    prim = cube.GetPrim()
    prim.AddAppliedSchema("PhysxCollisionAPI")  # PhysxSchema isn't importable outside Isaac Sim
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(CONTACT_OFFSET)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(REST_OFFSET)


def collider_box_rotated(stage, path, center, size, rot_deg, axis="y"):
    """An invisible box collider of `size`, centred at `center`, turned rot_deg about `axis` ("x" or "y")."""
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(*center))
    (cube.AddRotateXOp() if axis == "x" else cube.AddRotateYOp()).Set(rot_deg)
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.MakeInvisible()
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(CONTACT_OFFSET)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(REST_OFFSET)


def ramp(stage, path, a, b, outward, length, centre, axis, thickness):
    """An invisible plate whose inner face runs from a to b (2D points) in the plane across `axis`
    ("y": points are (x, z), the plate runs `length` along y about `centre`; "x": points are (y, z),
    along x). `outward` (2D) is the side the plate's body lies on; `thickness` is its depth."""
    a, b, outward = np.asarray(a, float), np.asarray(b, float), np.asarray(outward, float)
    u = (b - a) / np.linalg.norm(b - a)
    n = np.array([u[1], -u[0]])
    n = n if n @ outward > 0 else -n
    mid = (a + b) / 2 + n * thickness / 2
    span = float(np.linalg.norm(b - a))
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    if axis == "y":
        cube.AddTranslateOp().Set(Gf.Vec3d(mid[0], centre, mid[1]))
        cube.AddRotateYOp().Set(float(np.degrees(np.arctan2(u[0], u[1]))))
        cube.AddScaleOp().Set(Gf.Vec3f(thickness, length, span))
    else:
        cube.AddTranslateOp().Set(Gf.Vec3d(centre, mid[0], mid[1]))
        cube.AddRotateXOp().Set(float(np.degrees(np.arctan2(-u[0], u[1]))))
        cube.AddScaleOp().Set(Gf.Vec3f(length, thickness, span))
    cube.MakeInvisible()
    prim = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(CONTACT_OFFSET)
    prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(REST_OFFSET)


def load_parts(path, transform):
    """Read a pc_anatomy_export.ts JSON; transform maps an (N, 3) array of points."""
    parts = json.loads(open(path).read())
    for p in parts:
        p["pts"] = transform(np.array(p["points"]).reshape(-1, 3))
    return parts


def add_visual(stage, scope, parts, textured_color):
    """One mesh per (concept, colour): pc-anatomy's canvas textures become textured_color."""
    UsdGeom.Scope.Define(stage, scope)
    merged = defaultdict(lambda: ([], []))
    for p in parts:
        color = textured_color if p["textured"] else p["color"]
        pts, idx = merged[(p["concept"], tuple(round(c, 3) for c in color))]
        idx.append(np.array(p["indices"]) + sum(len(a) for a in pts))
        pts.append(p["pts"])
    for n, ((concept, color), (pts, idx)) in enumerate(sorted(merged.items())):
        pts, idx = np.concatenate(pts), np.concatenate(idx)
        mesh = UsdGeom.Mesh.Define(stage, f"{scope}/{concept}_{n}")
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * (len(idx) // 3)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx.tolist()))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])


def physics_material(stage, path, prim, static=0.6, dynamic=0.5, compliance=None):
    """Bind a friction material to prim. compliance=(stiffness, damping) makes contacts springy;
    PhysX only honours it on a dynamic body's material, and scales it with the body's mass."""
    mat = UsdShade.Material.Define(stage, path)
    pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    pm.CreateStaticFrictionAttr(static)
    pm.CreateDynamicFrictionAttr(dynamic)
    pm.CreateRestitutionAttr(0.0)
    if compliance:
        m = mat.GetPrim()
        m.AddAppliedSchema("PhysxMaterialAPI")
        m.CreateAttribute("physxMaterial:compliantContactStiffness", Sdf.ValueTypeNames.Float).Set(compliance[0])
        m.CreateAttribute("physxMaterial:compliantContactDamping", Sdf.ValueTypeNames.Float).Set(compliance[1])
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
