"""Dimensioned custom service foot:4 compliant wrap corners and2 snap clamp bars.

Not a Spot replica. Native passive hinges/sliders are a lumped elastic surrogate.
No event-triggered latching, pose edits, or fastener-state switches.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .native import ROOT,box,wedge,ring,body,radial_joint,filter_pairs
PHYSICS_HZ=480;PREFIX='foot_'
R=Rotation.from_euler('z',90,degrees=True).as_matrix()
PAD_ROTATION=R.copy()
ORIGIN=np.array([.24,0.,.8915])
PAD_START=np.array([.29,-.18,.825])
CLAMP_START={'left':np.array([.38,.095,.91]),'right':np.array([.38,-.095,.91])}
CLAMP_TARGET={'left':ORIGIN+R@np.array([.034,0,-.0105]),'right':ORIGIN+R@np.array([-.034,0,-.0105])}
BODY_NAMES=['pad']+[f'flap{i}' for i in range(4)]+[s+n for s in ['left','right'] for n in ['_clamp','_end0','_end1']]
FIXED=[]
FLAPS=[(sign,y) for sign in [-1,1] for y in [-.012,.012]]


def hinge(stage,path,parent,child,anchor,sign):
 from pxr import UsdPhysics,Gf
 j=UsdPhysics.RevoluteJoint.Define(stage,path);j.CreateBody0Rel().SetTargets([parent]);j.CreateBody1Rel().SetTargets([child]);j.CreateAxisAttr('Y');j.CreateLocalPos0Attr(Gf.Vec3f(*anchor));j.CreateLocalPos1Attr(Gf.Vec3f(*anchor));j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(False)
 j.CreateLowerLimitAttr(-25 if sign>0 else 0);j.CreateUpperLimitAttr(0 if sign>0 else 25)
 d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');d.CreateTypeAttr('force');d.CreateStiffnessAttr(.001);d.CreateDampingAttr(.00005);d.CreateTargetPositionAttr(0.)


def build(stage):
 global FIXED
 # Foot held inverted in a bench service fixture. Core uses real ledges/undercuts.
 FIXED=[]
 def fixed(name,center,size,color=(.44,.48,.53)):
  path=ROOT+'/service_foot/'+name;box(stage,path,ORIGIN+R@np.array(center),size,color,R);FIXED.append(path)
 fixed('leg',[0,0,-.0750],[.035,.026,.121],(.16,.19,.24))
 fixed('middle',[0,0,-.0085],[.050,.032,.012])
 for sign in [-1,1]:
  for end in [-1,1]:fixed(f'ear_{sign+1}_{end+1}',[sign*.026,end*.0155,-.0085],[.002,.007,.012])
 fixed('base',[0,0,-.138],[.13,.10,.012],(.17,.20,.25))
 p=ROOT+'/foot_pad';body(stage,p,PAD_START,.025,PAD_ROTATION)
 box(stage,p+'/sole',[0,0,0],[.064,.048,.005],(.12,.22,.31))
 for i,(sign,y) in enumerate(FLAPS):
  c=ROOT+f'/foot_flap{i}';body(stage,c,PAD_START,.003,PAD_ROTATION)
  box(stage,c+'/side',[sign*.030,y,-.008],[.003,.014,.013],(.12,.24,.34))
  # Beveled hook toe; geometric overlap under the foot flange retains the cover.
  vertices=[(.0255,-.0145),(.0315,-.0145),(.0315,-.0205),(.0280,-.0205)]
  from pxr import UsdGeom,UsdPhysics,Gf
  # Extruded XZ profile,14mm in local Y. All pieces are real collision hulls.
  pts=[[sign*x,y+dy,z] for dy in [-.007,.007] for x,z in vertices]
  m=UsdGeom.Mesh.Define(stage,c+'/hook');m.CreatePointsAttr(pts);m.CreateFaceVertexCountsAttr([4]*6);m.CreateFaceVertexIndicesAttr([3,2,1,0,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]);m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(.035,.08,.14)])
  from .native import collider
  collider(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
  hinge(stage,p+f'/hinges/h{i}',p,c,[sign*.030,y,-.0015],sign)
 # Molded inner guide ribs locate the cover on the core before final seating.
 # Entry bevels centre ordinary placement error; there is no pose snap or latch.
 for axis,half,wide,span in [('y',.0163,.021,.015),('x',.0253,.029,.004)]:
  for sign in [-1,1]:
   profile=[(half,-.0025),(wide,-.0025),(wide,-.026),(wide-.001,-.026),(half,-.015)]
   pts=[([v,sign*u,z] if axis=='y' else [sign*u,v,z]) for v in [-span,span] for u,z in profile]
   m=UsdGeom.Mesh.Define(stage,p+f'/guide_{axis}_{sign+1}');m.CreatePointsAttr(pts);m.CreateFaceVertexCountsAttr([5,5]+[4]*5);m.CreateFaceVertexIndicesAttr([4,3,2,1,0,5,6,7,8,9]+[j for i in range(5) for j in [i,(i+1)%5,(i+1)%5+5,i+5]]);m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(.10,.19,.26)])
   collider(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
 filter_pairs(stage,[p]+[ROOT+f'/foot_flap{i}' for i in range(4)])
 # Custom service clamp bars lock the four wrap flaps against opening.
 # Each bar has two passive end clips that flex over core ears and recover behind them.
 for side,sign in [('left',1),('right',-1)]:
  base=CLAMP_START[side];b=ROOT+'/foot_'+side+'_clamp';body(stage,b,base,.018,R)
  box(stage,b+'/cover',[0,0,0],[.004,.048,.016],(.78,.44,.11) if side=='left' else (.8,.5,.15))
  # Stepped inner lands fit the 31.5 mm flap envelope below the 32 mm sole.
  # A flat inner wall left 0.5 mm free play and let the bar rock on its top lip.
  for end in [-1,1]:
   box(stage,b+f'/flap_land_{end+1}',[-sign*.00225,end*.012,-.002],[.0005,.012,.008],(.56,.59,.62))
  # Return lip rests on the sole: end hooks resist pull-out, lip supports gravity.
  box(stage,b+'/top_bridge',[0,0,.011],[.004,.048,.006],(.70,.42,.12))
  box(stage,b+'/support_lip',[-sign*.003,0,.014],[.010,.040,.002],(.78,.47,.13))
  for k,end in enumerate([-1,1]):
   c=ROOT+f'/foot_{side}_end{k}';body(stage,c,base,.002,R)
   # Body frame x=0 is side34mm. Retaining face reaches24.5mm; leading bevel reaches21mm outside the ear end.
   box(stage,c+'/arm',[-sign*.0055,end*.021,0],[.007,.003,.012],(.56,.59,.62))
   # Tapered entry nose spreads the end clip outward while the bar is pushed inward.
   xz=[(-sign*.0095,end*.018),(-sign*.0100,end*.018),(-sign*.0130,end*.0205),(-sign*.0095,end*.0225)]
   from pxr import UsdGeom,UsdPhysics,Gf
   pts=[[x,y,z] for z in [-.006,.006] for x,y in xz]
   m=UsdGeom.Mesh.Define(stage,c+'/nose');m.CreatePointsAttr(pts);m.CreateFaceVertexCountsAttr([4]*6);m.CreateFaceVertexIndicesAttr([3,2,1,0,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7]);m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(.42,.45,.49)])
   collider(m.GetPrim());UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
   # generic radial helper requires a direction perpendicular to localY; here use nativeY slider directly
   from pxr import UsdPhysics
   j=UsdPhysics.PrismaticJoint.Define(stage,b+f'/clips/c{k}');j.CreateBody0Rel().SetTargets([b]);j.CreateBody1Rel().SetTargets([c]);j.CreateAxisAttr('Y');j.CreateLowerLimitAttr(0 if end>0 else -.003);j.CreateUpperLimitAttr(.003 if end>0 else 0);j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(False)
   d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'linear');d.CreateTypeAttr('force');d.CreateStiffnessAttr(500.);d.CreateDampingAttr(.1);d.CreateTargetPositionAttr(0.)
  filter_pairs(stage,[b]+[ROOT+f'/foot_{side}_end{k}' for k in range(2)])
 return [ROOT+'/'+PREFIX+n for n in BODY_NAMES]


def contract():
 return dict(kind='CUSTOM_SERVICEABLE_QUADRUPED_FOOT_NOT_SPOT',physics_hz=PHYSICS_HZ,
  pad_target=ORIGIN.tolist(),pad_start=PAD_START.tolist(),pad_start_rotation=PAD_ROTATION.tolist(),rotation=R.tolist(),clamp_targets={k:v.tolist() for k,v in CLAMP_TARGET.items()},
  sole_mm=[64,48,5],foot_envelope_mm=[54,38,12],hook_tip_x_mm=25.5,core_outer_half_width_mm=27,
  flap_spring_Nm_deg=.001,clamp_spring_N_m=500,flap_limits_deg=[0,25],clamp_travel_mm=3,
  fixed_colliders=FIXED,fidelity='Rigid sole and4passive bending hinges approximate wrap corners;2spring clips per removable bar provide mechanical fastening. Custom quick-clamp design,not the Spot M3 screw design;no rubber continuum,force calibration or walking-load test.')
