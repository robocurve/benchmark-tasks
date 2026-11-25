"""Compound rigid geometry. Dimensions and offsets are metres in the body frame.

These parts become ordinary colliders before physics starts; no assembly rule
moves or attaches an object during execution.
"""

from dataclasses import dataclass
import math


def contact_patch_radii(value):
    """Validate finite torsional contact patches; values are radii in metres."""
    if len(value)!=2:
        raise ValueError('contact patch needs a radius and minimum radius')
    radius,minimum=map(float,value)
    if not (math.isfinite(radius) and math.isfinite(minimum) and 0<=minimum<=radius):
        raise ValueError('contact patch radii must be finite and 0 <= minimum <= radius')
    return radius,minimum


@dataclass(frozen=True)
class Solid:
    kind: str
    size: tuple[float, ...]
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    axis: str = "Z"
    roll: float = 0.0
    pitch: float = 0.0
    color: tuple[float, float, float] | None = None
    opacity: float | None = None

    def __post_init__(self):
        dimensions = {"box": 3, "cylinder": 2, "sphere": 1, "capsule_x": 2, "tapered_panel": 5}
        if self.kind not in dimensions or len(self.size) != dimensions[self.kind]:
            raise ValueError("solid kind and dimensions disagree")
        if any(x <= 0 or not math.isfinite(x) for x in self.size):
            raise ValueError("solid dimensions must be positive and finite")
        if self.kind == "tapered_panel" and self.size[4] >= math.pi:
            raise ValueError("a tapered panel must have an angular span below pi")
        if len(self.offset) != 3 or not all(math.isfinite(x) for x in (*self.offset, self.yaw, self.roll, self.pitch)):
            raise ValueError("solid pose must be finite")
        if self.axis not in ("X", "Y", "Z"):
            raise ValueError("solid axis must be X, Y or Z")
        if self.color is not None and (len(self.color)!=3 or any(not math.isfinite(v) or not 0<=v<=1 for v in self.color)):
            raise ValueError("solid color must contain three finite values in [0,1]")
        if self.opacity is not None and (self.color is None or not 0 <= self.opacity <= 1):
            raise ValueError("opacity requires a color and a value in [0,1]")


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


def tapered_panel_mesh(size):
    """Return one convex shell sector with its taper along the X axis."""
    inner_start, inner_end, thickness, length, span = size
    cosine, sine = math.cos(span/2), math.sin(span/2)
    points = [(x, radius*cosine, sign*radius*sine)
              for x, inner in ((-length/2, inner_start), (length/2, inner_end))
              for radius in (inner, inner+thickness)
              for sign in (-1, 1)]
    faces = ((0,1,3,2), (4,6,7,5), (0,4,5,1), (2,3,7,6), (0,2,6,4), (1,5,7,3))
    return points, faces


def author_compound(prim, parts, cfg, *, visual_only=False):
    """Replace the placeholder mesh with primitive solid colliders on one body.

    Call only from the spawn hook, after SimulationApp starts. The original
    rigid body, mass, materials and initial transform remain in force.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade
    import isaaclab.sim as sim_utils

    if not parts:
        raise ValueError("a compound body needs at least one solid")
    stage = prim.GetStage()
    root = str(prim.GetPath())
    if not visual_only:
        stage.RemovePrim(root+"/geometry/mesh")
    for i, part in enumerate(parts):
        prefix = 'visual' if visual_only else 'solid'
        path = f"{root}/geometry/{prefix}_{i}"
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
        elif part.kind == "tapered_panel":
            points, faces = tapered_panel_mesh(part.size)
            geom = UsdGeom.Mesh.Define(stage, path)
            geom.CreatePointsAttr([Gf.Vec3f(*point) for point in points])
            bounds = tuple(zip(*points))
            geom.CreateExtentAttr([Gf.Vec3f(*map(min,bounds)), Gf.Vec3f(*map(max,bounds))])
            geom.CreateFaceVertexCountsAttr([len(face) for face in faces])
            geom.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
            geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            if not visual_only:
                UsdPhysics.MeshCollisionAPI.Apply(geom.GetPrim()).CreateApproximationAttr("convexHull")
            scale = None
        else:
            geom = UsdGeom.Sphere.Define(stage, path)
            geom.CreateRadiusAttr(part.size[0])
            scale = None
        geom.AddTranslateOp().Set(Gf.Vec3d(*part.offset))
        if part.roll or part.pitch:
            geom.AddRotateXYZOp().Set(Gf.Vec3f(*(math.degrees(v) for v in (part.roll,part.pitch,part.yaw))))
        elif part.yaw:
            geom.AddRotateZOp().Set(math.degrees(part.yaw))
        if scale:
            geom.AddScaleOp().Set(Gf.Vec3f(*scale))
        if not visual_only:
            UsdPhysics.CollisionAPI.Apply(geom.GetPrim()).CreateCollisionEnabledAttr(True)
        if cfg.visual_material is not None:
            mat = cfg.visual_material_path
            sim_utils.bind_visual_material(path, mat if mat.startswith("/") else root+"/geometry/"+mat, stage=stage)
        if not visual_only and cfg.physics_material is not None:
            mat = cfg.physics_material_path
            sim_utils.bind_physics_material(path, mat if mat.startswith("/") else root+"/geometry/"+mat, stage=stage)
        if part.color is not None:
            material=UsdShade.Material.Define(stage,path+"_material")
            shader=UsdShade.Shader.Define(stage,path+"_material/shader")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor",Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*part.color))
            if part.opacity is not None:
                shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(part.opacity))
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
            UsdShade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(material)
