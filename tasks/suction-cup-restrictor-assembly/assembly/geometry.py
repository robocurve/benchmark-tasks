"""Custom40mm plug-in vacuum cup; segmented passive neck, not OEM CAD/FEM."""
import numpy as np
from .native import ROOT,box,wedge,ring,body,radial_joint,filter_pairs,POSITION_ITERATIONS,VELOCITY_ITERATIONS
PHYSICS_HZ=960
SOLVER_TYPE='PGS'
CONTACT_CORRELATION=.0005
FRICTION_OFFSET=.0002
CUP_START=np.array([.38,-.09,.90])
CORE_START=np.array([.38,.08,.90])
TARGET=np.array([.24,-.024,.89])
HOLE_ORIGIN=np.array([.24,0.,.89])
CORE_TARGET=TARGET+np.array([0,.0315,0])
BODY_NAMES=['cup']+[f'barb{i}' for i in range(4)]+['core']
PREFIX='vac_'
FIXED=[]


def build(stage):
 from pxr import UsdGeom,UsdPhysics,PhysxSchema
 global FIXED
 p=ROOT+'/vac_cup';body(stage,p,CUP_START,.018)
 # Floating-base reduced-coordinate assembly: four radial DOFs remain passive.
 # Unlike independent maximal-coordinate joints, the intended locked angular
 # axes are represented directly rather than corrected after contact impulses.
 root=stage.GetPrimAtPath(p);UsdPhysics.ArticulationRootAPI.Apply(root)
 art=PhysxSchema.PhysxArticulationAPI.Apply(root)
 art.CreateSolverPositionIterationCountAttr(POSITION_ITERATIONS)
 art.CreateSolverVelocityIterationCountAttr(VELOCITY_ITERATIONS)
 # Rigid hollow bellows envelope: bellows compliance is outside this installation test.
 for i in range(3):
  y=-.010+i*.008
  ring(stage,p+f'/bellows{i}',.007,.020,y,y+.004,(.10,.62,.32))
  ring(stage,p+f'/web{i}',.007,.013,y+.004,y+.008,(.10,.55,.27))
 ring(stage,p+'/neck',.0046,.0083,.014,.024)
 ring(stage,p+'/lower_shoulder',.0046,.011,.020,.0214)
 # Four independently compliant fingers. Each is ONE convex solid with an
 # inner flat at4.5mm and retained outer corners on the10.5mm circle. Avoid
 # hidden internal compound seams at the interference fit.
 from .native import collider
 from pxr import Gf
 from yam_arena.station import linear_rgb
 for i in range(4):
  c=ROOT+f'/vac_barb{i}';body(stage,c,CUP_START,.001)
  a=i*np.pi/2
  base=np.sqrt(.0105**2-.0035**2);tip=np.sqrt(.0078**2-.0025**2)
  points=np.array([[.0045,.026,-.002],[base,.026,-.0035],[base,.026,.0035],[.0045,.026,.002],
                   [.0045,.030,-.002],[tip,.030,-.0025],[tip,.030,.0025],[.0045,.030,.002]])
  rotation=np.array([[np.cos(a),0.,-np.sin(a)],[0.,1.,0.],[np.sin(a),0.,np.cos(a)]])
  faces=[[3,2,1,0],[4,5,6,7]]+[[j,(j+1)%4,(j+1)%4+4,j+4] for j in range(4)]
  mesh=UsdGeom.Mesh.Define(stage,c+'/head')
  mesh.CreatePointsAttr((points@rotation.T).tolist());mesh.CreateFaceVertexCountsAttr([4]*6)
  mesh.CreateFaceVertexIndicesAttr([v for f in faces for v in f]);mesh.CreateSubdivisionSchemeAttr('none')
  mesh.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb((.15,.65,.35)))])
  collider(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
  radial_joint(stage,p+f'/springs/r{i}',p,c,[np.cos(a),0,np.sin(a)])
 filter_pairs(stage,[p]+[ROOT+f'/vac_barb{i}' for i in range(4)])
 # White POM core, actual0.8mm flow bore. Cap/shoulder and tapered retention rib.
 c=ROOT+'/vac_core';body(stage,c,CORE_START,.0012)
 ring(stage,c+'/shaft',.0004,.0044,-.0215,-.0009,(.91,.92,.90))
 for i in range(24):
  wedge(stage,c+f'/tip/s{i:02d}',[(.0004,-.024),(.0032,-.024),(.0044,-.0215),(.0004,-.0215)],i*2*np.pi/24,(i+1)*2*np.pi/24,(.91,.92,.90))
 ring(stage,c+'/cap',.0004,.0065,-.0009,.0015,(.91,.92,.90))
 for i in range(24):
  wedge(stage,c+f'/rib/s{i:02d}',[(.0044,-.007),(.0053,-.006),(.0053,-.0055),(.0044,-.0055)],i*2*np.pi/24,(i+1)*2*np.pi/24,(.91,.92,.90))
 # One static concave surface mesh preserves the real bore. Separate convex
 # wedges exposed artificial radial seam faces during elastic insertion.
 from pxr import Gf
 from yam_arena.station import linear_rgb
 from .native import collider
 path=ROOT+'/vac_plate/annulus';n=48
 # A 1 mm x 45 degree entry chamfer guides the elastic head into the
 # 18 mm through bore. The sharp entrance jammed in robot contact tests
 # despite good centering; the bevel supplies a defined radial lead-in.
 vertices=[[r*np.cos(i*2*np.pi/n),y,r*np.sin(i*2*np.pi/n)]
           for r,y in [(.010,-.002),(.055,-.002),(.009,.002),(.055,.002),(.009,-.001)] for i in range(n)]
 faces=[]
 for i in range(n):
  j=(i+1)%n
  faces += [[i,n+i,n+j,j], [2*n+i,2*n+j,3*n+j,3*n+i],
            [i,j,4*n+j,4*n+i], [4*n+i,4*n+j,2*n+j,2*n+i],
            [n+i,3*n+i,3*n+j,n+j]]
 mesh=UsdGeom.Mesh.Define(stage,path)
 mesh.CreatePointsAttr((np.asarray(vertices)+HOLE_ORIGIN).tolist())
 mesh.CreateFaceVertexCountsAttr([4]*len(faces));mesh.CreateFaceVertexIndicesAttr([v for f in faces for v in f])
 mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDisplayColorAttr([Gf.Vec3f(*linear_rgb((.19,.25,.29)))])
 collider(mesh.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('none')
 FIXED=[path]
 for name,center,size in [('pillar',[.24,0.,.803],[.03,.018,.11]),('base',[.24,0.,.7535],[.14,.08,.012])]:
  path=ROOT+'/vac_plate/'+name;box(stage,path,center,size,(.18,.23,.27));FIXED.append(path)
 return [ROOT+'/'+PREFIX+n for n in BODY_NAMES]


def contract():
 return dict(kind='CUSTOM_ENGINEERING_FIXTURE_NOT_OEM',physics_hz=PHYSICS_HZ,
    physics_solver=SOLVER_TYPE,solve_articulation_contact_last=True,
    cup_joint_representation='floating_base_reduced_coordinate_radial_articulation',
    solver_position_iterations=POSITION_ITERATIONS,
    solver_velocity_iterations=VELOCITY_ITERATIONS,
    friction_correlation_distance_m=CONTACT_CORRELATION,friction_offset_threshold_m=FRICTION_OFFSET,
    cup_start=CUP_START.tolist(),core_start=CORE_START.tolist(),cup_target=TARGET.tolist(),core_target=CORE_TARGET.tolist(),hole_center=HOLE_ORIGIN.tolist(),
    plate_hole_diameter_mm=18,plate_thickness_mm=4,entry_chamfer_diameter_mm=20,entry_chamfer_depth_mm=1,cup_diameter_mm=40,neck_shoulder_y_mm=[21.4,26],
    compliant_head_radius_mm=10.5,tip_radius_mm=7.8,inner_lip_radius_mm=4.5,
    head_retention_corner_mm=[float(np.sqrt(10.5**2-3.5**2)),3.5],head_collider='one_convex_tapered_finger_per_radial_joint',
    radial_spring_N_m=180,radial_damping_Ns_m=.8,radial_travel_mm=[-3,1.5],core_shaft_diameter_mm=8.8,core_cap_diameter_mm=13,
    core_retention_rib_diameter_mm=10.6,core_length_mm=25.5,flow_bore_mm=.8,
    fidelity='Rigid bellows plus four passive radial head sectors approximate neck elasticity; head-sector self-contact omitted, no snap trigger/pose drive/weld. Uncalibrated force curve; no vacuum seal or airflow simulation.',
    fixed_colliders=FIXED)
