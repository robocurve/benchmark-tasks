"""YAM extension: collision-driven ink; observations remain ordinary cameras/joints."""
from pathlib import Path
import atexit
import os
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

FEEDBACK = {}


class AnswerSheetEmbodiment(YamArenaEmbodiment):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        from dataclasses import replace
        import hashlib
        root=Path(__file__).resolve().parent.parent
        digest=hashlib.sha256()
        for p in sorted(root.rglob('*')):
            if p.is_file() and p.suffix in ('.py','.usdc','.png','.toml') and '__pycache__' not in p.parts:
                digest.update(str(p.relative_to(root)).encode()+b'\0'+p.read_bytes())
        self.info=replace(self.info,environment_revision=digest.hexdigest())

    def _ensure_sim(self, goal):
        from yam_arena import inspect_embodiment as base
        if base._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
            base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        import isaaclab.sim as sim
        from pxr import UsdGeom, UsdShade, Sdf, Gf
        original=scene.task_object
        def spawn(spec):
            if spec.name=='marker':
                return Object(name='marker',usd_path=str(Path(__file__).resolve().parent.parent/'assets/marker.usdc'),object_type=ObjectType.RIGID,initial_pose=Pose(position_xyz=tuple(spec.pos)),spawn_cfg_addon={
                    'physics_material':scene._grippy(),
                    'collision_props':sim.CollisionPropertiesCfg(contact_offset=.0003,rest_offset=0.),
                    'rigid_props':sim.RigidBodyPropertiesCfg(solver_position_iteration_count=32,solver_velocity_iteration_count=4),
                    'activate_contact_sensors':True,
                })
            obj=original(spec)
            if spec.name=='paper':
                obj.spawner_cfg.collision_props=sim.CollisionPropertiesCfg(contact_offset=.0001,rest_offset=0.)
                obj.object_cfg=obj._init_object_cfg()
            return obj
        scene.task_object=spawn
        try: super()._ensure_sim(goal)
        finally: scene.task_object=original
        self._base.sim.set_setting('/physics/disableContactProcessing',False)
        self._stage=self._base.sim.stage
        # Printed plane lies over the paper collider; no additional collision.
        s=self._stage
        plane=UsdGeom.Mesh.Define(s,'/World/AnswerPrint')
        x,y,_=self.task.objects[0].pos
        sx,sy,_=self.task.objects[0].size
        z=self.task.module.PAPER_TOP+.00003
        plane.CreatePointsAttr([Gf.Vec3f(x-sx/2,y-sy/2,z),Gf.Vec3f(x+sx/2,y-sy/2,z),Gf.Vec3f(x+sx/2,y+sy/2,z),Gf.Vec3f(x-sx/2,y+sy/2,z)])
        plane.CreateFaceVertexCountsAttr([4]);plane.CreateFaceVertexIndicesAttr([0,1,2,3]);plane.CreateSubdivisionSchemeAttr('none')
        uv=UsdGeom.PrimvarsAPI(plane).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'vertex');uv.Set([(1,0),(1,1),(0,1),(0,0)])
        mat=UsdShade.Material.Define(s,'/World/AnswerPrintMaterial')
        sh=UsdShade.Shader.Define(s,mat.GetPath().AppendChild('Surface'));sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.)
        sh.CreateInput('ior',Sdf.ValueTypeNames.Float).Set(1.)
        sh.CreateInput('specularColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0,0,0))
        tex=UsdShade.Shader.Define(s,mat.GetPath().AppendChild('Texture'));tex.CreateIdAttr('UsdUVTexture')
        tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(Path(__file__).resolve().parent.parent/'assets/form.png'))
        tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
        st=UsdShade.Shader.Define(s,mat.GetPath().AppendChild('UV'));st.CreateIdAttr('UsdPrimvarReader_float2');st.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
        tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(),'result')
        sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(mat)
        self._ink_mesh=UsdGeom.Mesh.Define(s,'/World/PermanentInk');self._ink_mesh.CreateSubdivisionSchemeAttr('none');self._ink_mesh.CreateDoubleSidedAttr(True)
        self._ink_mesh.CreateDisplayColorAttr([Gf.Vec3f(.005,.015,.12)])
        inkmat=UsdShade.Material.Define(s,'/World/InkMaterial')
        inksh=UsdShade.Shader.Define(s,'/World/InkMaterial/Surface');inksh.CreateIdAttr('UsdPreviewSurface')
        inksh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.001,.003,.025))
        inksh.CreateInput('ior',Sdf.ValueTypeNames.Float).Set(1.)
        inksh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.)
        inkmat.CreateSurfaceOutput().ConnectToSource(inksh.ConnectableAPI(),'surface')
        UsdShade.MaterialBindingAPI.Apply(self._ink_mesh.GetPrim()).Bind(inkmat)
        self._drawn=(-1,-1)
        self._robot_contacts=[]
        if not hasattr(self,'_contact_subscription'):
            import omni.physx
            self._contact_subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._contacts)

    def _contacts(self, headers, data):
        if not getattr(self,'_recording',False): return
        from pxr import PhysicsSchemaTools
        for h in headers:
            if not h.num_contact_data: continue
            paths=[str(PhysicsSchemaTools.intToSdfPath(getattr(h,n))) for n in ('collider0','collider1')]
            if any('/marker' in p for p in paths) and any('Arm' in p for p in paths):
                self._robot_contacts=paths
            if not any('/Tip' in p and 'marker' in p for p in paths) or not any('/paper' in p for p in paths): continue
            points=[]
            for d in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]:
                p=np.array([d.position.x,d.position.y,d.position.z])
                # Contact offsets permit an early pair event; do not draw a gap.
                if abs(p[2]-self.task.module.PAPER_TOP)<.00045 and d.separation <= .00005:points.append(p)
            if points:
                p=np.mean(points,axis=0)
                self.task.module.INK.contact(p[:2],self._base._sim_step_counter*self._base.physics_dt)

    def reset(self, scene, *, seed=None):
        from answer_sheet.scoring import Ink
        self._recording=False
        # Clear the existing visual before the parent's reset renders and
        # captures its first observation, including resets in the same process.
        if hasattr(self,'_ink_mesh'):
            self._ink_mesh.GetPointsAttr().Set([])
            self._ink_mesh.GetFaceVertexCountsAttr().Set([])
            self._ink_mesh.GetFaceVertexIndicesAttr().Set([])
        self.task.module.INK=Ink(self.task.module.BOXES,self.task.module.ANSWERS)
        obs=super().reset(scene,seed=seed)
        self._base.sim.set_setting('/physics/disableContactProcessing',False)
        self._ink_mesh.GetPointsAttr().Set([]);self._ink_mesh.GetFaceVertexCountsAttr().Set([]);self._ink_mesh.GetFaceVertexIndicesAttr().Set([])
        self._drawn=(-1,-1)
        # Renderer updates are keyed to physics steps. Plain render() calls can
        # reuse a cached frame; settle the ready robot through real control
        # steps so temporal image history is also cleared on every reset.
        for _ in range(48):self._env.step(self._ready)
        for name in ('top_cam','left_cam','right_cam'):
            self._base.scene[name].reset()
        obs=self._observation()
        self._recording=True
        self._feedback()
        return obs

    def _feedback(self):
        from yam_arena.urdf import quat_xyzw_to_matrix
        pose=self._base.arena_world.get_pose_w('marker')[0].detach().cpu().numpy()
        FEEDBACK['pos']=pose[:3].copy();FEEDBACK['rot']=quat_xyzw_to_matrix(pose[3:])
        FEEDBACK['tip']=pose[:3]+FEEDBACK['rot']@np.array([-.066,0,0])

    def _draw(self):
        from pxr import Gf
        segments=self.task.module.INK.segments
        dots=self.task.module.INK.dots
        count=(len(segments),len(dots))
        if count==self._drawn:return
        points=[]
        z=self.task.module.PAPER_TOP+.00010
        for a,b in segments:
            delta=b-a;n=np.array([-delta[1],delta[0]])/np.linalg.norm(delta)*.00065
            points.extend(Gf.Vec3f(float(p[0]),float(p[1]),z) for p in (a+n,b+n,b-n,a-n))
        for a in dots:
            points.extend(Gf.Vec3f(float(a[0]+dx),float(a[1]+dy),z) for dx,dy in ((-.00065,-.00065),(.00065,-.00065),(.00065,.00065),(-.00065,.00065)))
        self._ink_mesh.GetPointsAttr().Set(points)
        self._ink_mesh.GetFaceVertexCountsAttr().Set([4]*(len(segments)+len(dots)))
        self._ink_mesh.GetFaceVertexIndicesAttr().Set(list(range(len(points))))
        self._drawn=count

    def _task_step(self):
        self._feedback();self._draw()
        result=super()._task_step()
        if round(self._t*self.info.control_hz)%20==0:
            h=self.task.module.INK
            from yam_arena import ik,station
            a,f=self._ids['right_robot'];q=self._base.scene['right_robot'].data.joint_pos.torch[0]
            tcp,_=ik.tcp_pose(q[a].cpu().numpy(),station.BASES['right'])
            print('INK_STATE',round(self._t,2),'pos',np.round(FEEDBACK['pos'],4).tolist(),'tip',np.round(FEEDBACK['tip'],4).tolist(),'tcp',np.round(tcp,4).tolist(),'fingers',q[f].cpu().numpy().round(4).tolist(),'robot_contacts',self._robot_contacts,'contacts',h.contact_count,'segments',len(h.segments),'cells',[[len(c) for c in row] for row in h.cells],'score',result.info.get('score'),flush=True)
        return result


def make_embodiment(**kwargs):
    return AnswerSheetEmbodiment(**kwargs)
