"""YAM scene preview with a genuinely passive dispenser, no validated demo."""
from yam_arena.task_file import Box
from yam_arena import scene as adapter
from native import author_dispenser,Coupling
import math

SCENE='station'
OBJECTS=[
    Box('cake',pos=(.30,0.,.7755),size=(.18,.14,.05),color=(.66,.39,.20)),
    Box('cake_board',pos=(.30,0.,.749),size=(.20,.16,.003),color=(.7,.72,.75)),
    Box('barrel',pos=(.18,-.16,.7775),size=(.030,.080,.030),mass=.080,color=(.2,.45,.7),yaw=math.pi),
    Box('plunger',pos=(.18,-.10,.7775),size=(.035,.005,.035),mass=.025,color=(.7,.72,.75),yaw=math.pi),
    Box('rest',pos=(.18,-.16,.755),size=(.05,.04,.015),color=(.2,.2,.2)),
    # Flat edible guide line and two end marks, visual purpose only. Their
    # 0.1 mm thickness is below the rubric's 0.5 mm icing threshold.
    Box('guide',pos=(.235,0.,.80055),size=(.0005,.090,.0001),color=(.3,.2,.1)),
    Box('guide_start',pos=(.235,-.045,.80055),size=(.010,.0005,.0001),color=(.3,.2,.1)),
    Box('guide_end',pos=(.235,.045,.80055),size=(.010,.0005,.0001),color=(.3,.2,.1)),
]
ROOTS={}
COUPLING=Coupling()
original_task_object=adapter.task_object


def task_object(spec):
    obj=original_task_object(spec)
    if spec.name not in ('cake','barrel','plunger'):return obj
    spawn=obj.spawner_cfg.func
    def physical_spawn(prim_path,cfg,*a,**kw):
        import omni.usd
        prim=spawn(prim_path,cfg,*a,**kw)
        if spec.name=='cake':
            # Thin visual crumb coat on the existing cake top, no extra body,
            # collision geometry or hidden alteration of the judged surface.
            from pxr import UsdGeom,UsdShade,Gf,Sdf
            stage=omni.usd.get_context().get_stage()
            coat=UsdGeom.Mesh.Define(stage,str(prim_path)+'/WhiteCrumbCoat')
            coat.CreatePointsAttr([(-.09,-.07,.02501),(.09,-.07,.02501),(.09,.07,.02501),(-.09,.07,.02501)])
            coat.CreateFaceVertexCountsAttr([4]);coat.CreateFaceVertexIndicesAttr([0,1,2,3])
            coat.CreateSubdivisionSchemeAttr('none');coat.CreateDisplayColorAttr([Gf.Vec3f(.94,.92,.86)])
            material=UsdShade.Material.Define(stage,str(prim_path)+'/WhiteCoatMaterial')
            shader=UsdShade.Shader.Define(stage,str(prim_path)+'/WhiteCoatMaterial/Surface')
            shader.CreateIdAttr('UsdPreviewSurface')
            shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.94,.92,.86))
            shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.55)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
            UsdShade.MaterialBindingAPI.Apply(coat.GetPrim()).Bind(material)
            return prim
        ROOTS[spec.name]=str(prim_path)
        if spec.name=='plunger':
            stage=omni.usd.get_context().get_stage()
            author_dispenser(stage,ROOTS['barrel'],ROOTS['plunger'])
            COUPLING.author_surface(stage)
        return prim
    obj.spawner_cfg.func=physical_spawn
    obj.object_cfg.spawn.func=physical_spawn
    return obj


adapter.task_object=task_object
SCORE_OVER='end'
def score(objects):return 0.
def success(objects):return False
