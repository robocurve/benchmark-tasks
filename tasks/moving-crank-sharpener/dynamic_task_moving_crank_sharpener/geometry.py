"""Compound rigid geometry. Dimensions and offsets are metres in the body frame.

These parts become ordinary colliders before physics starts; no assembly rule
moves or attaches an object during execution.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Solid:
    kind: str
    size: tuple[float, ...]
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    axis: str = "Z"

    def __post_init__(self):
        dimensions = {"box": 3, "cylinder": 2, "sphere": 1, "capsule_x": 2}
        if self.kind not in dimensions or len(self.size) != dimensions[self.kind]:
            raise ValueError("solid kind and dimensions disagree")
        if any(x <= 0 or not math.isfinite(x) for x in self.size):
            raise ValueError("solid dimensions must be positive and finite")
        if len(self.offset) != 3 or not all(math.isfinite(x) for x in (*self.offset, self.yaw)):
            raise ValueError("solid pose must be finite")
        if self.axis not in ("X", "Y", "Z"):
            raise ValueError("solid axis must be X, Y or Z")


def square_ring(outer: float, hole: float, thickness: float):
    """A four-bar washer with a genuine square through-hole."""
    if not 0 < hole < outer or not thickness > 0:
        raise ValueError("ring needs positive thickness and 0 < hole < outer")
    width = (outer-hole)/2
    shift = (outer+hole)/4
    return (Solid("box", (width, outer, thickness), (-shift, 0, 0)),
            Solid("box", (width, outer, thickness), (shift, 0, 0)),
            Solid("box", (hole, width, thickness), (0, -shift, 0)),
            Solid("box", (hole, width, thickness), (0, shift, 0)))


def author_compound(prim, parts, cfg):
    """Replace the placeholder mesh with primitive solid colliders on one body.

    Call only from the spawn hook, after SimulationApp starts. The original
    rigid body, mass, materials and initial transform remain in force.
    """
    from pxr import Gf, UsdGeom, UsdPhysics
    import isaaclab.sim as sim_utils

    if not parts:
        raise ValueError("a compound body needs at least one solid")
    stage = prim.GetStage()
    root = str(prim.GetPath())
    stage.RemovePrim(root+"/geometry/mesh")
    for i, part in enumerate(parts):
        path = f"{root}/geometry/solid_{i}"
        if part.kind == "box":
            geom = UsdGeom.Cube.Define(stage, path)
            geom.CreateSizeAttr(1.0)
            scale = part.size
        elif part.kind == "cylinder":
            geom = UsdGeom.Cylinder.Define(stage, path)
            geom.CreateRadiusAttr(part.size[0])
            geom.CreateHeightAttr(part.size[1])
            geom.CreateAxisAttr(part.axis)
            scale = None
        elif part.kind == "capsule_x":
            geom = UsdGeom.Capsule.Define(stage, path)
            geom.CreateRadiusAttr(part.size[0])
            geom.CreateHeightAttr(part.size[1])
            geom.CreateAxisAttr("X")
            scale = None
        else:
            geom = UsdGeom.Sphere.Define(stage, path)
            geom.CreateRadiusAttr(part.size[0])
            scale = None
        geom.AddTranslateOp().Set(Gf.Vec3d(*part.offset))
        if part.yaw:
            geom.AddRotateZOp().Set(math.degrees(part.yaw))
        if scale:
            geom.AddScaleOp().Set(Gf.Vec3f(*scale))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim()).CreateCollisionEnabledAttr(True)
        if cfg.visual_material is not None:
            mat = cfg.visual_material_path
            sim_utils.bind_visual_material(path, mat if mat.startswith("/") else root+"/geometry/"+mat, stage=stage)
        if cfg.physics_material is not None:
            mat = cfg.physics_material_path
            sim_utils.bind_physics_material(path, mat if mat.startswith("/") else root+"/geometry/"+mat, stage=stage)
