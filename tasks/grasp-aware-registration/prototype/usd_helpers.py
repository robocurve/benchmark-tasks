"""Task-only compound-box material authoring, reused from the validated slider."""

from . import layout as L


def cube(stage, path, position, size, color, collision=True, emission=None):
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade
    c = UsdGeom.Cube.Define(stage, path)
    c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d(*position))
    c.AddScaleOp().Set(Gf.Vec3f(*size))
    c.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    material = UsdShade.Material.Define(stage, path + '/material')
    shader = UsdShade.Shader.Define(stage, path + '/material/shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    # Match the station's sRGB-to-linear material convention.
    from yam_arena.station import linear_rgb
    from pxr import Sdf
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.6)
    if emission is not None:
        shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emission))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(material)
    if collision:
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
    return c.GetPrim()
