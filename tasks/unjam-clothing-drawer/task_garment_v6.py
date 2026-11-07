"""DEVELOPMENT: release a spring-supported garment-fold proxy and open a drawer.

The cabinet has a full physical lid. A low front flap is accessible after the jam. Folded garments
are rigid approximations fixed inside the drawer. The orange obstruction is a
single rigid textile-shaped fold on a vertical spring, NOT deformable cloth.
A passive lip blocks outward movement until contact pushes that fold down.
The wider, deeper drawer has a 200 mm passive rail and fixed viscous damping; no rail drive. Single environment / one trial per process.
"""
from pathlib import Path
import importlib.util
import json
import os
import sys
import numpy as np
from yam_arena.task_file import Box

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('_drawer_jam_visual', HERE/'task_visual.py')
visual = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = visual
spec.loader.exec_module(visual)
base = visual.base
TOP = base.TOP
SCENE = 'station'
SUCCESS_HOLD_S = 1.0
SCORE_OVER = 'end'
sys.path.insert(0,str(HERE/'cap_v6'))
from drawer_layout_v6 import *
FOLD_SIZE = (.104, .050, .026)
ASSETS = HERE/'assets'
OBJECTS = [Box(name,pos=pos,size=size) for name,(pos,size) in PANELS.items()] + [
    Box('drawer',pos=DRAWER_ORIGIN,size=(.32,.40,.082),mass=.50),
    Box('garment_fold',pos=FOLD_POS,size=FOLD_SIZE,mass=.04),
]


def measurement(objects):
    state=objects['drawer']
    if not np.isfinite(np.r_[state.low,state.high,state.lin_vel,state.ang_vel]).all():return 0.,False,False
    opening=DRAWER_CLOSED_BACK-float(state.high[0])
    valid=bool(-.002<=opening<=TRAVEL+.002
        and abs((state.high[1]+state.low[1])/2-DRAWER_ORIGIN[1])<=.004
        and abs(state.low[2]-DRAWER_BOTTOM)<=.004
        and abs(state.high[1]-state.low[1]-DRAWER_WIDTH)<=.006
        and abs(state.high[0]-state.low[0]-DRAWER_BOUND_DEPTH)<=.006)
    still=bool(np.linalg.norm(state.lin_vel)<.01 and np.linalg.norm(state.ang_vel)<.1)
    return opening,valid,still


def _spawn_fold(prim_path, cfg, translation=None, orientation=None, **kwargs):
    # Run after the drawer has spawned. Resolve its actual body rather than
    # hard-coding /World/envs/env_0, which is not portable across Arena versions.
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from pxr import Usd, UsdPhysics, Gf
    prim = spawn_from_usd(prim_path, cfg, translation=translation,
                          orientation=orientation, **kwargs)
    stage = prim.GetStage()
    drawer = stage.GetPrimAtPath(prim.GetPath().GetParentPath().AppendChild('drawer'))
    if not drawer or not drawer.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError('Garment spring: expected the drawer rigid body to spawn first.')
    joint = UsdPhysics.PrismaticJoint.Define(stage, prim.GetPath().AppendChild('Compression'))
    joint.CreateBody0Rel().SetTargets([drawer.GetPath()])
    joint.CreateBody1Rel().SetTargets([prim.GetPath()])
    joint.CreateAxisAttr('Z')
    joint.CreateLocalPos0Attr(Gf.Vec3f(*np.subtract(FOLD_POS, DRAWER_ORIGIN)))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    joint.CreateLowerLimitAttr(-.040)
    joint.CreateUpperLimitAttr(0)
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateCollisionEnabledAttr(False)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), 'linear')
    drive.CreateTypeAttr('force')
    drive.CreateStiffnessAttr(30.)
    drive.CreateDampingAttr(1.5)
    # Fixed preload balances the fold's weight at its initial height.
    # These values never change during the trial. No scripted unlocking.
    drive.CreateTargetPositionAttr(.04*9.81/30.)
    drive.CreateTargetVelocityAttr(0.)
    drive.CreateMaxForceAttr(5.)
    print('Garment compression spring connected:', drawer.GetPath(), '->', prim.GetPath())
    return prim


def build_scene_objects():
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    import isaaclab.sim as sim_utils
    from yam_arena import scene as scene_module
    scene_module.WINDOW_EYE=(-.60,.04,1.95)
    scene_module.WINDOW_LOOKAT=(.36,.04,.82)
    result=[]
    for item in OBJECTS:
        path=ASSETS/(item.name+'.usda')
        if not path.is_file(): raise FileNotFoundError(path)
        options={'usd_path':str(path)}
        if item.name=='garment_fold':
            options={'spawner_cfg':sim_utils.UsdFileCfg(usd_path=str(path),func=_spawn_fold)}
        result.append(Object(name=item.name,
            object_type=ObjectType.RIGID if item.mass else ObjectType.BASE,
            initial_pose=Pose(position_xyz=item.pos,rotation_xyzw=(0.,0.,0.,1.)),**options))
    return result


class Progress:
    """Object-event evidence only; does not certify which arm made contact."""
    def __init__(self):
        self.steps=0; self.stalled=0; self.jam=False; self.pressed=False; self.cleared=False
        self.previous=0.; self.initial_seen=False
    def update(self, objects):
        self.steps+=1
        opening, valid, still=measurement(objects)
        fold=objects['garment_fold']
        xyz=np.array([fold.high[0]-.022, (fold.high[1]+fold.low[1])/2, fold.high[2]-.013])
        compression=FOLD_POS[2]-float(xyz[2])
        valid=valid and bool(np.isfinite(np.r_[xyz,fold.lin_vel,fold.ang_vel]).all())
        valid=valid and abs(xyz[0]-(FOLD_POS[0]-opening))<.008
        valid=valid and abs(xyz[1]-FOLD_POS[1])<.008 and -.005<compression<.045
        if valid and opening<.005 and compression<.005: self.initial_seen=True
        at_lip=valid and JAM_LOW<opening<JAM_HIGH and compression<.009
        if at_lip and abs(opening-self.previous)<.0006:
            self.stalled+=1
        else: self.stalled=0
        if self.initial_seen and self.stalled>=10: self.jam=True
        if valid and self.jam and compression>.014: self.pressed=True
        if valid and self.pressed and opening>CLEAR_SCORE: self.cleared=True
        self.previous=opening
        full=bool(valid and still and opening>=FULL_OPEN and self.cleared)
        self.last_score=(1. if full else .75 if self.cleared else .50 if self.pressed else .25 if self.jam else 0.) if valid else 0.
        self.valid=bool(valid)
        if os.environ.get('JAM_TRACE_PATH'):
            row=dict(step=self.steps,time_s=self.steps/20,opening_m=opening,
                     compression_m=compression,valid=bool(valid),jam=self.jam,
                     pressed=self.pressed,cleared=self.cleared,full=full)
            with open(os.environ['JAM_TRACE_PATH'],'a') as f: f.write(json.dumps(row)+'\n')
        return full

progress=Progress()
def success(objects): return progress.update(objects)
def score(objects): return getattr(progress,'last_score',0.)
def distance(objects):
    opening,valid,_=measurement(objects)
    return max(0.,FULL_OPEN-opening) if valid else FULL_OPEN


def demo():
    raise RuntimeError('Use cap_v6/run.sh: independent timed waypoint plans are disabled for this scene.')
