"""Cold-loading, schema, scorer adversarial checks; absolutely no physics."""
import copy,json,math,hashlib,ast
from pathlib import Path
import numpy as np
from pxr import Usd,UsdPhysics
import geometry as g
from build_usd import cold_export
from scoring import classify,Monitor
ROOT=Path(__file__).resolve().parent

def record(pos):return dict(pos=list(map(float,pos)),quat_xyzw=[0.,0.,0.,1.],lin_vel=[0.,0.,0.],ang_vel=[0.,0.,0.])
def initial():return dict(time_s=0.,sample_id=0,case=record(g.ORIGIN),cards={n:record(p)for n,p in g.initial_cards().items()},robot_clearance_m=.10)
def terminal():
 r=initial();r['cards']['card_4']=record(g.TRAY+np.array([0,0,.001+g.CARD_SIZE[2]/2]))
 # Follower rises by one thickness when a card is dispensed; nominal snapshot.
 for i in range(4):r['cards'][f'card_{i}']['pos'][2]+=g.CARD_SIZE[2]
 return r

def run():
 checks=[]
 def check(name,pred):
  checks.append(dict(name=name,pass_=bool(pred)))
  if not pred:raise AssertionError(name)
 stage=cold_export(ROOT/'cold-stage.usda');del stage
 stage=Usd.Stage.Open(str(ROOT/'cold-stage.usda'));bodies=[];joints=[];colliders=[]
 for p in stage.Traverse():
  if p.HasAPI(UsdPhysics.RigidBodyAPI):
   m=UsdPhysics.MassAPI(p);bodies.append(dict(path=str(p.GetPath()),mass=float(m.GetMassAttr().Get()),inertia=list(m.GetDiagonalInertiaAttr().Get()),com=list(m.GetCenterOfMassAttr().Get())))
  if p.IsA(UsdPhysics.Joint):
   j=UsdPhysics.PrismaticJoint(p);joints.append(dict(path=str(p.GetPath()),type=p.GetTypeName(),body0=list(map(str,j.GetBody0Rel().GetTargets())),body1=list(map(str,j.GetBody1Rel().GetTargets())),lower=j.GetLowerLimitAttr().Get(),upper=j.GetUpperLimitAttr().Get(),spring=float(UsdPhysics.DriveAPI(p,'linear').GetStiffnessAttr().Get())))
  if p.HasAPI(UsdPhysics.CollisionAPI):colliders.append(str(p.GetPath()))
 check('cold readback 7 rigid bodies 1 joint',len(bodies)==7 and len(joints)==1)
 check('positive actual mass inertia',all(b['mass']>0 and min(b['inertia'])>0 for b in bodies))
 check('source spring preserved',joints[0]['spring']==g.SPRING_K)
 check('single layer throat',g.CARD_SIZE[2]<g.SLOT_HEIGHT<2*g.CARD_SIZE[2])
 check('initial no-op zero',classify(initial())['score']==0)
 r=terminal();check('one delivered partial before release hold',classify(r)['score']==.75)
 m=Monitor()
 for k in range(21):r['time_s']=k*.05;r['sample_id']=k;m.update(r)
 check('fresh finite one-second released terminal full',m.last['score']==1 and m.last['success'])
 for label,mutate in [
 ('two outputs',lambda r:r['cards']['card_3'].update(pos=(g.TRAY+np.array([0,0,.00138])).tolist())),
 ('card missing',lambda r:r['cards'].pop('card_2')),
 ('card identity replaced',lambda r:r['cards'].update({'fake':r['cards'].pop('card_2')})),
 ('card spilled',lambda r:r['cards']['card_0'].update(pos=[.1,.3,g.TABLE+.00038])),
 ('nan pose',lambda r:r['cards']['card_0']['pos'].__setitem__(0,float('nan'))),
 ('nan speed',lambda r:r['cards']['card_0']['lin_vel'].__setitem__(0,float('nan'))),
 ('case overturned',lambda r:r['case'].update(quat_xyzw=[1,0,0,0])),
 ('bad quaternion',lambda r:r['cards']['card_4'].update(quat_xyzw=[0,0,0,2])),
 ('card mat corner outside',lambda r:r['cards']['card_4']['pos'].__setitem__(0,float(g.TRAY[0]+.04)))]:
  rr=terminal();mutate(rr);check(label+' cannot full',not classify(rr).get('full_candidate',False))
 r=terminal();r['robot_clearance_m']=.001;check('held card not full',not classify(r)['full_candidate'])
 r=terminal();r['cards']['card_4']['lin_vel']=[.03,0,0];check('moving card not full',not classify(r)['full_candidate'])
 r=terminal();m=Monitor();m.update(r)
 for _ in range(30):m.update(r)
 check('duplicate samples cannot hold',m.last['score']==0 and not m.last['valid'])
 r=terminal();m=Monitor();m.update(r);r['time_s']=2.;r['sample_id']=40;m.update(r)
 check('unobserved gap cannot hold',m.last['score']==.75 and m.last['held_s']==0)
 r=terminal();r['cards']['card_4']['quat_xyzw']=[1,0,0,0];check('face-down legally on mat',classify(r)['full_candidate'])
 r=terminal();delta=np.array([.11,.11,0.]);r['case']['pos']=(np.array(r['case']['pos'])+delta).tolist()
 for i in range(4):r['cards'][f'card_{i}']['pos']=(np.array(r['cards'][f'card_{i}']['pos'])+delta).tolist()
 check('legal relocation of retained case not penalized',classify(r)['full_candidate'])
 for path in ROOT.glob('*.py'):ast.parse(path.read_text())
 check('all author Python AST parse',True)
 report=dict(kind='CPU offline synthetic observations and cold USD; not native physics',pass_=all(c['pass_']for c in checks),checks=checks,bodies=bodies,joints=joints,colliders=colliders,
  native_contact=None,reference_score=None,model_score=None,backend_created_joints=None,source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()for p in ROOT.glob('*.py')})
 (ROOT/'CPU-COLD-RESULT.json').write_text(json.dumps(report,indent=2)+'\n');(ROOT/'ENGINEERING.json').write_text(json.dumps(g.engineering(),indent=2)+'\n')
 print(json.dumps(dict(pass_=report['pass_'],checks=len(checks),bodies=len(bodies),joints=len(joints),colliders=len(colliders))))
if __name__=='__main__':run()
