"""Create one rigid compound carrier/tab from the documented box parts."""


def replace_mesh(prim, config, parts, contact_offset=.0002):
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    stage=prim.GetStage();path=str(prim.GetPath())
    stage.RemovePrim(path+'/geometry/mesh')
    for part in parts:
        cube=UsdGeom.Cube.Define(stage,path+'/geometry/'+part.name)
        cube.CreateSizeAttr(1.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*part.center))
        cube.AddScaleOp().Set(Gf.Vec3f(*part.size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        collision=PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
        collision.CreateContactOffsetAttr(contact_offset)
        collision.CreateRestOffsetAttr(0.)
        bind_physics_material(str(cube.GetPath()),path+'/geometry/'+config.physics_material_path)
        # A material per part preserves the visible module/fixture distinction.
        from isaaclab.sim.spawners.materials import PreviewSurfaceCfg,spawn_preview_surface
        material_path=str(cube.GetPath())+'/material'
        spawn_preview_surface(material_path,PreviewSurfaceCfg(diffuse_color=part.color))
        bind_visual_material(str(cube.GetPath()),material_path)
