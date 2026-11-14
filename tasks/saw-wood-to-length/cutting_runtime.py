"""Contact-driven fracture of the freely moving task wood in its local frame."""
from __future__ import annotations
import numpy as np
from stock import Stock


def rotation(q):
    x,y,z,w=map(float,q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


class Cutting:
    def __init__(self,mechanism):
        self.m=mechanism;self.stage=mechanism.stage;self.stock=Stock()
        self.revision=0;self.initial_path=mechanism.paths['wood']+'/box_0'
        self.current_shapes=[self.initial_path]
        self.fragment_view=None;self.fragment_path=None;self.cut_time=None
        self.prev_tooth=None;self.value=0.;self.distance=1.;self.last_gap=None
        self.cut_before_clamp=False
        self.events=[];self.last_score={'value':0.,'reason':'not_two_pieces'}

    def teardown(self):
        """Restore the intact authored wood before a same-scene reset.

        The YAM embodiment resets assets in place; it does not recreate the
        USD stage for another episode of the same task.
        """
        from pxr import UsdGeom,UsdPhysics
        self.fragment_view=None
        wood_path=self.m.paths['wood']
        for revision in range(1,self.revision+1):
            path=f'{wood_path}/geometry_{revision}'
            if self.stage.GetPrimAtPath(path).IsValid():self.stage.RemovePrim(path)
        fragment='/World/envs/env_0/saw_offcut'
        if self.stage.GetPrimAtPath(fragment).IsValid():self.stage.RemovePrim(fragment)
        initial=self.stage.GetPrimAtPath(self.initial_path)
        UsdPhysics.CollisionAPI(initial).GetCollisionEnabledAttr().Set(True)
        UsdGeom.Imageable(initial).MakeVisible()
        mark=self.stage.GetPrimAtPath(wood_path+'/vis_0')
        if mark.IsValid():UsdGeom.Imageable(mark).MakeVisible()
        UsdPhysics.MassAPI(self.stage.GetPrimAtPath(wood_path)).GetMassAttr().Set(.060)

    def _clamp_state(self,wood_pose):
        jaw_y=float(self.m.views['jaw'].get_transforms()[0,1])
        gap=-.035-jaw_y
        angle=self.m.screw_angle
        position_ok=(abs(float(wood_pose[0])-.20)<.003 and
                     abs(float(wood_pose[1])+.03)<.003 and
                     abs(float(wood_pose[2])-(.7475+.080))<.003)
        # Under saw load the jaw deflects up to 0.16 mm despite the screw
        # remaining turned and the wood staying between the two jaw faces.
        return .0192<=gap<=.0202 and angle>1.5 and position_ok,gap

    def _cube(self,path,center,size):
        from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
        obj=UsdGeom.Cube.Define(self.stage,path)
        obj.CreateSizeAttr(1.)
        obj.AddTranslateOp().Set(Gf.Vec3d(*map(float,center)))
        obj.AddScaleOp().Set(Gf.Vec3f(*map(float,size)))
        obj.CreateDisplayColorAttr([Gf.Vec3f(.69,.45,.24)])
        prim=obj.GetPrim();UsdPhysics.CollisionAPI.Apply(prim)
        c=PhysxSchema.PhysxCollisionAPI.Apply(prim)
        c.CreateContactOffsetAttr(.000005);c.CreateRestOffsetAttr(0.)
        return prim

    def _disable_current(self):
        from pxr import UsdGeom,UsdPhysics
        for path in self.current_shapes:
            prim=self.stage.GetPrimAtPath(path)
            UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(False)
            UsdGeom.Imageable(prim).MakeInvisible()
        self.current_shapes=[]
        mark=self.stage.GetPrimAtPath(self.m.paths['wood']+'/vis_0')
        if mark.IsValid():UsdGeom.Imageable(mark).MakeInvisible()

    def _add_boxes(self,parent,boxes,offset=(0,0,0)):
        shapes=[]
        for i,b in enumerate(boxes):
            path=f'{parent}/box_{i:04d}'
            self._cube(path,np.asarray(b[:3])-offset,b[3:])
            shapes.append(path)
        return shapes

    def _spawn_fragment(self,mask,wood_pose,wood_velocity):
        from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf
        boxes=self.stock.boxes(mask)
        volumes=np.array([b[3]*b[4]*b[5] for b in boxes])
        centre=np.average(np.array([b[:3] for b in boxes]),axis=0,weights=volumes)
        base='/World/envs/env_0/saw_offcut'
        UsdGeom.Xform.Define(self.stage,base)
        obj=UsdGeom.Xform.Define(self.stage,base+'/body')
        p=np.asarray(wood_pose[:3],float);q=wood_pose[3:7]
        world_centre=p+rotation(q)@centre
        obj.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float,world_centre)))
        obj.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(float(q[3]),float(q[0]),float(q[1]),float(q[2])))
        body=obj.GetPrim();rb=UsdPhysics.RigidBodyAPI.Apply(body)
        vel=np.asarray(wood_velocity[:3],float)+np.cross(wood_velocity[3:6],world_centre-p)
        rb.CreateVelocityAttr(Gf.Vec3f(*map(float,vel)))
        rb.CreateAngularVelocityAttr(Gf.Vec3f(*map(float,np.degrees(wood_velocity[3:6]))))
        UsdPhysics.MassAPI.Apply(body).CreateMassAttr(float(500*volumes.sum()))
        PhysxSchema.PhysxRigidBodyAPI.Apply(body).CreateSolverPositionIterationCountAttr(16)
        self._add_boxes(base+'/body/geometry',boxes,centre)
        self.fragment_path=base+'/body'

    def _rebuild(self,parts,wood_pose,wood_velocity):
        from pxr import UsdGeom,UsdPhysics
        self._disable_current();self.revision+=1
        if len(parts)>1:
            assert len(parts)==2 and parts[0][0,:].any()
            print('CUT_SPLIT_GEOMETRY',self.stock.score(),flush=True)
            rootmask=parts[0]
            other=parts[1]
            self._spawn_fragment(other,wood_pose,wood_velocity)
            self.cut_time=self.m.t
        else:rootmask=parts[0]
        name=self.m.paths['wood']+f'/geometry_{self.revision}'
        UsdGeom.Xform.Define(self.stage,name)
        boxes=self.stock.boxes(rootmask)
        self.current_shapes=self._add_boxes(name,boxes)
        UsdPhysics.MassAPI(self.stage.GetPrimAtPath(self.m.paths['wood'])).GetMassAttr().Set(
            float(500*sum(b[3]*b[4]*b[5] for b in boxes)))

    def step(self,fn):
        wood=self.m.views['wood'];saw=self.m.views['saw']
        wp=wood.get_transforms()[0];sp=saw.get_transforms()[0]
        rw=rotation(wp[3:7]);rs=rotation(sp[3:7])
        tooth=[rw.T@(sp[:3]+rs@np.array([0,y,-.035])-wp[:3]) for y in [-.055,-.255]]
        mid=(tooth[0]+tooth[1])/2
        ds=0. if self.prev_tooth is None else abs(float(mid[1]-self.prev_tooth[1]))
        self.prev_tooth=mid
        if self.cut_time is None:
            changed=self.stock.remove(tooth[0],tooth[1],ds,fn)
            if self.stock.removed_last_step and not self._clamp_state(wp)[0]:
                self.cut_before_clamp=True
            if changed:
                parts=self.stock.components()
                self._rebuild(parts,wp,wood.get_velocities()[0])
                self.events.append({'t':self.m.t,'tool_x_m':float(mid[0]),'depth_m':float(self.stock.progress.max()),
                                    'normal_force_N':fn,'relative_stroke_m':ds,'pieces':len(parts)})
        if self.fragment_path and self.fragment_view is None and self.m.t>self.cut_time+1/480:
            self.fragment_view=self.m.sv.create_rigid_body_view(self.fragment_path)
            assert self.fragment_view.count==1
        self.distance=max(0.,1-self.stock.progress.max()/.020)
        if self.fragment_view is None:return
        self.last_score=self.stock.score()
        # The scored item must remain clamped after cutting; the saw's tooth
        # line must be clear above the stock. All are ruler/photo criteria.
        clamp_ok,gap=self._clamp_state(wp)
        tool_clear=min(v[2] for v in tooth)>.025
        self.value=self.last_score['value'] if clamp_ok and tool_clear and not self.cut_before_clamp else 0.
        self.last_gap=gap
