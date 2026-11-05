"""Spawn puzzle pieces from Isaac Lab primitive API calls, without authored asset files.

The stock task-file API supports individual primitives and registered assets, but does not expose
compound primitives. The scoped hook replaces this task's four placeholder boxes during environment
creation; all other task objects use stock behavior.
The simulator still maintains its usual in-memory USD scene graph.
"""
from pathlib import Path
import atexit
import importlib.util
import math
import os

ROOT=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('square_api_geometry',ROOT/'geometry.py')
geometry=importlib.util.module_from_spec(spec);spec.loader.exec_module(geometry)


def _object(specification,scene):
    import isaaclab.sim as sim
    from isaaclab.sim.utils import clone
    from pxr import UsdPhysics

    index=int(specification.name.rsplit('_',1)[1])-1
    blocks=list(geometry.rectangles(geometry.cells_for(index)))
    color=scene.linear_rgb(specification.color)

    @clone
    def spawn(prim_path,cfg,translation=None,orientation=None,**kwargs):
        sim.create_prim(prim_path,prim_type='Xform',translation=translation,orientation=orientation)
        for i,(x0,x1,y0,y1) in enumerate(blocks):
            size=((x1-x0)*geometry.STEP,(y1-y0)*geometry.STEP,geometry.HEIGHT)
            pos=((x0+x1)*geometry.STEP/2,(y0+y1)*geometry.STEP/2,0.)
            part=sim.CuboidCfg(size=size,collision_props=sim.CollisionPropertiesCfg(contact_offset=.0005,rest_offset=0.),
                physics_material=scene._grippy(),visual_material=sim.PreviewSurfaceCfg(diffuse_color=color,roughness=.4))
            sim.spawn_cuboid(f'{prim_path}/block_{i}',part,translation=pos)
        root=sim.get_current_stage().GetPrimAtPath(prim_path)
        UsdPhysics.RigidBodyAPI.Apply(root)
        sim.define_rigid_body_properties(prim_path,sim.RigidBodyPropertiesCfg(
            solver_position_iteration_count=32,solver_velocity_iteration_count=4))
        UsdPhysics.MassAPI.Apply(root).CreateMassAttr().Set(float(specification.mass))
        return root

    # The input xy denotes the local bounding-box centre after rotation.
    cells = geometry.cells_for(index)
    dx = (min(x for x, _ in cells) + max(x for x, _ in cells) + 1) * geometry.STEP / 2
    dy = (min(y for _, y in cells) + max(y for _, y in cells) + 1) * geometry.STEP / 2
    angle=specification.yaw
    rx=math.cos(angle)*dx-math.sin(angle)*dy
    ry=math.sin(angle)*dx+math.cos(angle)*dy
    x,y,z=specification.pos
    pose=scene.Pose(position_xyz=(x-rx,y-ry,z),rotation_xyzw=(0.,0.,math.sin(angle/2),math.cos(angle/2)))
    spawner=sim.CuboidCfg(func=spawn,size=(.05,.05,geometry.HEIGHT))
    return scene.Object(name=specification.name,object_type=scene.ObjectType.RIGID,spawner_cfg=spawner,initial_pose=pose)


def install(task_root):
    import yam_arena.inspect_embodiment as embodiment
    cls=embodiment.YamArenaEmbodiment
    roots = getattr(cls, '_square_api_roots', None)
    if roots is None:
        roots = {}
        cls._square_api_roots = roots
    roots[Path(task_root).resolve()] = _object
    if getattr(cls,'_square_api_installed',False):
        return
    original=cls._ensure_sim
    def ensure(self,goal):
        current=getattr(self,'task',None)
        matches = [] if current is None else [
            (root, builder) for root, builder in roots.items()
            if Path(current.path).resolve().is_relative_to(root)
        ]
        if not matches:
            return original(self,goal)
        _, builder = max(matches, key=lambda pair: len(pair[0].parts))
        if embodiment._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
            kw={'headless':self.headless,'device':self.device,'enable_cameras':True}
            if not self.headless:
                kw['visualizer']=['kit']
                os.environ['DISPLAY']=embodiment.window_display(os.environ)
                if os.environ['DISPLAY']==embodiment.DESKTOP_DISPLAY:
                    kw['window_width'],kw['window_height']=embodiment.DESKTOP_SIZE
            embodiment._APP=AppLauncher(**kw).app
            atexit.register(embodiment._close_app_at_exit)
        from yam_arena import scene
        previous=scene.task_object
        def build(item):
            if item.name.startswith('square_api_piece_'):
                return builder(item,scene)
            return previous(item)
        scene.task_object=build
        try:
            return original(self,goal)
        finally:
            scene.task_object=previous
    cls._ensure_sim=ensure
    cls._square_api_installed=True
