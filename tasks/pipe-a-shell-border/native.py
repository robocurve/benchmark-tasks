"""Native dispenser geometry and material coupling, imported after SimulationApp.

The plunger is a free rigid body constrained by an ordinary prismatic joint.
There are no motor targets. Equal and opposite material-pressure forces resist
compression. Only emitted material geometry is authored from material state.
Native pressure controls and robot pickup/extrusion diagnostics have run.
A completed border has not yet been demonstrated.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from material import Icing, IcingParameters


@dataclass(frozen=True)
class DispenserParameters:
    # A passive seal damper. These are declared engineering approximations,
    # not a controller or a hidden piston motor.
    seal_damping_ns_m: float = 10.
    seal_friction_n: float = .15
    friction_regularization_m_s: float = .0005
    # Reduced mass of the 80 g barrel and 25 g plunger. Limiting a dissipative
    # impulse to the relative momentum avoids discrete Coulomb chatter.
    seal_effective_mass_kg: float = .080*.025/(.080+.025)


def rotation(q):
    q=np.asarray(q,float);q=q/np.linalg.norm(q)
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def author_dispenser(stage,barrel_path,plunger_path,*,diagnostic_fixture=False):
    """Add the joint/nozzle to existing Box rigid bodies, before physics starts.

    Barrel frame: long axis +Y towards tip, plunger initial centre Y=-60 mm.
    The bare barrel/plunger boxes remain the normal grip/contact surfaces.
    """
    from pxr import Gf,Sdf,UsdGeom,UsdPhysics,PhysxSchema
    icing=IcingParameters()
    b,p=stage.GetPrimAtPath(barrel_path),stage.GetPrimAtPath(plunger_path)
    for prim in (b,p):
        if not prim or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise RuntimeError("Dispenser needs two existing dynamic rigid bodies")
        rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr(64)
        rb.CreateSolverVelocityIterationCountAttr(16)
        rb.CreateEnableCCDAttr(True)
    # Barrel/plunger overlap only at the sliding seal. Other object contact is
    # retained. No task/policy condition changes this permanent mechanism.
    UsdPhysics.FilteredPairsAPI.Apply(b).CreateFilteredPairsRel().SetTargets([Sdf.Path(plunger_path)])
    joint=UsdPhysics.PrismaticJoint.Define(stage,barrel_path+'/PistonGuide')
    joint.CreateBody0Rel().SetTargets([Sdf.Path(barrel_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(plunger_path)])
    joint.CreateAxisAttr('Y')
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.,-.060,0.))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.,0.,0.))
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(icing.stroke_m)
    # A hollow metal frustum, with an actual six-point aperture. Each sector is
    # one convex contact piece. Icing is emitted through this same aperture.
    for i in range(24):
        angles=np.array((i,i+1))*2*np.pi/24
        pts=[]
        for y,outer in ((.040,.010),(.080,.004)):
            for inner in (False,True):
                for angle in angles:
                    radius=(.008 if y==.040 else icing.nozzle_radius_m*(1+icing.star_depth*np.cos(icing.star_lobes*angle))) if inner else outer
                    pts.append((radius*np.cos(angle),y,radius*np.sin(angle)))
        faces=((0,1,3,2),(4,6,7,5),(0,4,5,1),(2,3,7,6),(0,2,6,4),(1,5,7,3))
        mesh=UsdGeom.Mesh.Define(stage,f'{barrel_path}/NozzleSector{i:02d}')
        mesh.CreatePointsAttr(pts);mesh.CreateFaceVertexCountsAttr([4]*6)
        mesh.CreateFaceVertexIndicesAttr([j for f in faces for j in f])
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.70,.72,.75)])
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
        contact=PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
        contact.CreateContactOffsetAttr(.0002);contact.CreateRestOffsetAttr(0.)
    if diagnostic_fixture:
        # Test fixture only, expressly absent from the robot scene.
        world=UsdGeom.XformCache().GetLocalToWorldTransform(b)
        anchor=UsdPhysics.FixedJoint.Define(stage,barrel_path+'/DiagnosticClamp')
        anchor.CreateBody1Rel().SetTargets([Sdf.Path(barrel_path)])
        anchor.CreateLocalPos0Attr(Gf.Vec3f(*world.ExtractTranslation()))
        q=world.ExtractRotationQuat()
        anchor.CreateLocalRot0Attr(Gf.Quatf(q.GetReal(),Gf.Vec3f(*q.GetImaginary())))
    return joint


class Coupling:
    def __init__(self,material=None,dispenser=None,*,boundary_velocity='displacement'):
        if boundary_velocity not in ('displacement','native'):
            raise ValueError('Unknown boundary velocity convention')
        self.boundary_velocity=boundary_velocity
        self.previous_boundary_pose=None
        self.material=material or Icing()
        self.dispenser=dispenser or DispenserParameters()
        self.last_piston_m=0.
        self.last_piston_velocity_m_s=0.
        self.last_seal_force_n=0.
        self.previous_piston_m=None
        self.last_reaction_n=0.
        self.last_contact_force=np.zeros(3)
        self.last_contact_torque=np.zeros(3)
        self.mesh=None
        # Area-weighted samples of the entire metal outlet face. Resolve it
        # with the material grid; two fixed circular rows leave growing gaps
        # when cell size is reduced and omit metal beside the star recesses.
        n=max(48,24*int(np.ceil(2*np.pi*.004/self.material.dx/24)))
        minimum_radius=self.material.p.nozzle_radius_m*(1-self.material.p.star_depth)
        rings=max(2,int(np.ceil((.004-minimum_radius)/self.material.dx)))
        samples=[];weights=[]
        for angle in np.arange(n)*2*np.pi/n:
            inner=self.material.p.nozzle_radius_m*(1+self.material.p.star_depth*np.cos(self.material.p.star_lobes*angle))
            edges=np.linspace(inner,.004,rings+1)
            areas=np.diff(edges**2)
            radii=(2/3)*np.diff(edges**3)/areas
            for radius,area in zip(radii,areas):
                samples.append((radius*np.cos(angle),.080,radius*np.sin(angle)))
                weights.append(area)
        self.contact_samples=np.asarray(samples)
        self.contact_weights=np.asarray(weights)/sum(weights)

    def prime_boundary(self,pose):
        """Record an episode-reset pose without changing any native state."""
        self.previous_boundary_pose=np.asarray(pose,float).copy()

    def boundary_motion(self,pose,velocity,dt):
        """Use actual, unsmoothed surface displacement for explicit coupling.

        PhysX split impulses make the reported end velocity differ from the
        transform motion. Material transport uses the preceding native-step
        displacement, including rotation, at each actual boundary point. This
        is a staggered one-step coupling, not a velocity filter or pose edit.
        The native convention is retained solely for replaying older evidence.
        """
        if dt<=0:raise ValueError('Positive integration step required')
        b,v=np.asarray(pose,float),np.asarray(velocity,float)
        r=rotation(b[3:]);tip=b[:3]+r@np.array((0.,.080,0.))
        world=self.contact_samples@r.T+b[:3]
        if self.boundary_velocity=='native':
            tipvel=v[:3]+np.cross(v[3:],tip-b[:3])
            velocities=v[:3]+np.cross(np.broadcast_to(v[3:],world.shape),world-b[:3])
        elif self.previous_boundary_pose is None:
            tipvel=np.zeros(3);velocities=np.zeros_like(world)
        else:
            previous=self.previous_boundary_pose;pr=rotation(previous[3:])
            tipvel=(tip-previous[:3]-pr@np.array((0.,.080,0.)))/dt
            velocities=(world-(self.contact_samples@pr.T+previous[:3]))/dt
        self.previous_boundary_pose=b.copy()
        return tip,tipvel,world,velocities

    def advance(self,barrel_pose,barrel_velocity,plunger_pose,dt):
        """Return world pressure wrenches from measured native body state."""
        b,p,v=(np.asarray(a,float) for a in (barrel_pose,plunger_pose,barrel_velocity))
        r=rotation(b[3:]);axis=r[:,1]
        q=float(np.dot(p[:3]-b[:3],axis)+.060)
        # A tolerance handles PhysX joint round-off, not arbitrary out-of-range
        # state. Larger constraint violations fail rather than earn material.
        if not -.0002<=q<=self.material.p.stroke_m+.0002:
            raise RuntimeError(f"Piston escaped its guide: {q:.6f} m")
        q=float(np.clip(q,0.,self.material.p.stroke_m))
        speed=0. if self.previous_piston_m is None else (q-self.previous_piston_m)/dt
        self.previous_piston_m=q
        tip,tipvel,world,velocities=self.boundary_motion(b,v,dt)
        nr=r@np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])
        reaction=self.material.step(dt,piston_m=q,nozzle_pos=tip,
                                    nozzle_rotation=nr,nozzle_velocity=tipvel)
        seal=(self.dispenser.seal_damping_ns_m*speed+
              self.dispenser.seal_friction_n*np.tanh(speed/self.dispenser.friction_regularization_m_s))
        stopping_force=self.dispenser.seal_effective_mass_kg*abs(speed)/dt
        seal=float(np.clip(seal,-stopping_force,stopping_force))
        contact=self.material.tool_contact(world,velocities,dt,
                                          stiffness_n_m=1500.*self.contact_weights,
                                          damping_ns_m=self.contact_weights)
        self.last_contact_force=contact.sum(axis=0)
        self.last_contact_torque=np.cross(world-b[:3],contact).sum(axis=0)
        self.last_piston_m,self.last_reaction_n=q,reaction
        self.last_piston_velocity_m_s,self.last_seal_force_n=speed,float(seal)
        # The fluid pushes the piston backwards and barrel forwards. Torque is
        # zero for these centred axial forces.
        return axis*(reaction+seal)+self.last_contact_force,-axis*(reaction+seal)

    def reset(self):
        """Episode reset only; never used to control the running dispenser."""
        self.material.reset()
        self.previous_boundary_pose=None
        self.previous_piston_m=None
        self.last_piston_m=self.last_piston_velocity_m_s=self.last_reaction_n=self.last_seal_force_n=0.
        self.last_contact_force=np.zeros(3);self.last_contact_torque=np.zeros(3)

    def author_surface(self,stage,path='/World/DepositedIcing'):
        from pxr import Gf,UsdGeom
        m=self.material
        ny=len(m.y)
        a=(np.arange(len(m.x)-1)[:,None]*ny+np.arange(ny-1)[None,:]).ravel()
        self.all_faces=np.stack((np.column_stack((a,a+ny,a+ny+1)),np.column_stack((a,a+ny+1,a+1))),axis=1).reshape(-1,3)
        self.mesh=UsdGeom.Mesh.Define(stage,path)
        self.mesh.CreatePointsAttr([])
        self.mesh.CreateSubdivisionSchemeAttr('none')
        self.mesh.CreateDoubleSidedAttr(True)
        self.mesh.CreateDisplayColorAttr([Gf.Vec3f(.92,.18,.42)])
        self.update_surface()

    def update_surface(self):
        if self.mesh is None:return
        m=self.material
        # No icing is drawn until the material solver deposits it. The same
        # measured surface is used by the scorer; blank cells remain invisible.
        active=m.height.ravel()>1e-5
        faces=self.all_faces[active[self.all_faces].any(axis=1)] if active.any() else np.empty((0,3),int)
        used=np.unique(faces)
        points=np.column_stack((m.X.ravel()[used],m.Y.ravel()[used],(m.base+m.height).ravel()[used]))
        # Compact only render indexing. Material cells, volume, footprint and
        # scoring stay unchanged; no hidden geometry appears before extrusion.
        faces=np.searchsorted(used,faces)
        self.mesh.GetPointsAttr().Set(points.tolist())
        self.mesh.CreateFaceVertexCountsAttr([3]*len(faces))
        self.mesh.CreateFaceVertexIndicesAttr(faces.ravel().tolist())
