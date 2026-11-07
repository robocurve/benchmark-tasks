"""Adversarial outcome contracts, not tests of the scripted trajectory."""
import math
from types import SimpleNamespace
import numpy as np
import pytest
from guide_task.scoring import Evaluator
from guide_task.measurements import pose_measurements,rotation_xyzw
from guide_task.contact import Monitor
from guide_task import layout as L
from guide_task import rules as R

ROOT='/World/envs/env_0/'
COLLAR=ROOT+'collar'
GOAL=ROOT+'dock_goal/geometry/mesh'
FINGER=ROOT+'LeftArm/Geometry/base/link1/link2/link3/link4/link5/gripper/tip_left'

def pose(angle=L.FINISH,centre_offset=(0,0,0),yaw_offset=0,flip=False,tilt=0):
    from scipy.spatial.transform import Rotation
    rot=Rotation.from_euler('z',angle+yaw_offset)*Rotation.from_euler('y',math.pi if flip else tilt)
    centre=L.point(angle)+np.asarray(centre_offset)
    return np.r_[centre+rot.as_matrix()@np.array([L.HANDLE_OFFSET,0,0]),rot.as_quat()]

class History:
    def __init__(self):
        self.ready=True;self.error=None;self.samples=0;self.events=[];self.violations=[];self.active_grasp=False
    def advance(self,support=True,robot=(),bad=(),duplicate=False):
        old=self.samples;self.samples+=12
        robot=range(1,13) if robot=='all' else (6,) if robot=='mid' else robot
        for i in range(1,13):
            pairs=[]
            if support:pairs.append((COLLAR,GOAL,False))
            if i in robot:pairs.append((COLLAR,FINGER,False))
            if i in bad:pairs.append((COLLAR,ROOT+f'guide_{old//12:02d}/geometry/mesh',True))
            for a,b,prohibited in pairs:
                e=dict(a=a,b=b,physics_sample=old+i,force_n=1.,separation_m=0.,prohibited=prohibited)
                self.events.append(e)
                if prohibited:self.violations.extend([e,e] if duplicate else [e])
        self.active_grasp=12 in robot

def tick(e,h,p=None,**kw):
    h.advance(**kw);return e.update(pose() if p is None else p,h)
def settle(e=None,h=None,p=None,n=21,**kw):
    e,h=e or Evaluator(),h or History()
    for _ in range(n):r=tick(e,h,p,**kw)
    return r

def test_full_second_is_21_poses_not_20():
    e,h=Evaluator(),History()
    for _ in range(20):r=tick(e,h)
    assert not r['complete'] and r['stationary_window_s']==.95 and r['score']==0
    r=tick(e,h);assert r['complete'] and r['hold_s']==1

def test_held_or_unsupported_aligned_goal_only_partial():
    assert settle(robot='all')['score']==.75
    assert settle(support=False)['score']==.75

def test_intermittent_grasp_is_not_release_even_if_last_substep_is_free():
    r=settle(robot='mid');assert not r['complete'] and not r['released']

def test_one_missing_support_substep_prevents_full_credit():
    e,h=Evaluator(),History()
    for _ in range(20):tick(e,h)
    h.advance();h.events=[x for x in h.events if x['physics_sample']!=h.samples-5]
    assert not e.update(pose(),h)['complete']

def test_current_geometry_not_gripper_percent_controls_release():
    # Empty gripper closure has no input to the scorer. Only real contacts matter.
    assert settle()['complete']

@pytest.mark.parametrize('flip',[False,True])
def test_symmetric_placement_is_legal(flip):
    assert settle(p=pose(flip=flip))['complete']

def test_handle_up_is_not_penalized_for_a_square_collar():
    assert settle(p=pose(tilt=math.pi/2))['complete']

def test_passing_through_goal_has_no_transient_credit():
    e,h=Evaluator(),History()
    for a in np.linspace(L.START,L.FINISH,21):r=tick(e,h,pose(a),robot='all')
    assert r['score']==0 and not r['stable']

def test_slow_creep_is_not_stillness():
    e,h=Evaluator(),History()
    for z in np.linspace(0,.0035,21):r=tick(e,h,pose(centre_offset=(0,0,z)))
    assert not r['stable'] and r['score']==0

@pytest.mark.parametrize('closeness,expected',[(.2499,0),(.25,.25),(.6499,.25),(.65,.5),(.9,.5)])
def test_partial_thresholds(closeness,expected):
    a=L.START+closeness*(L.FINISH-L.START)
    assert settle(p=pose(a),robot='all',support=False)['score']==expected

def test_off_guide_orientation_cannot_retain_partial():
    p=pose(math.radians(80));p[3:]=[0,0,0,1]
    assert not pose_measurements(p)['threaded']
    assert settle(p=p,robot='all',support=False)['score']==0

def test_progress_is_not_a_best_so_far_latch():
    e,h=Evaluator(),History()
    assert settle(e,h,pose(math.radians(80)),robot='all',support=False)['score']==.5
    r=tick(e,h,pose(centre_offset=(0,0,.1)),robot='all',support=False)
    assert r['score']==0

def test_safe_detour_then_legal_completion_is_allowed():
    e,h=Evaluator(),History();tick(e,h,pose(L.START,centre_offset=(0,0,.1)),robot='all',support=False)
    assert settle(e,h,n=22)['complete']

@pytest.mark.parametrize('distance,at_goal',[(.007,True),(.007001,False)])
def test_position_boundary(distance,at_goal):
    assert tick(Evaluator(),History(),pose(centre_offset=(0,0,distance)))['at_goal']==at_goal

@pytest.mark.parametrize('degrees,at_goal',[(7.,True),(7.001,False)])
def test_alignment_boundary(degrees,at_goal):
    assert tick(Evaluator(),History(),pose(yaw_offset=math.radians(degrees)))['at_goal']==at_goal

@pytest.mark.parametrize('degrees,at_goal',[(3.,True),(3.001,False)])
def test_upright_boundary(degrees,at_goal):
    assert tick(Evaluator(),History(),pose(tilt=math.radians(degrees)))['at_goal']==at_goal

@pytest.mark.parametrize('count',[1,2,12])
def test_any_contact_witness_is_permanent(count):
    e,h=Evaluator(),History()
    tick(e,h,bad=range(1,count+1),duplicate=True)
    r=settle(e,h)
    assert r['contact_latched'] and r['score']==0
    assert r['contact_witness']['physics_sample']==1

def test_contact_gap_cannot_erase_a_mark():
    e,h=Evaluator(),History()
    tick(e,h,bad=(6,));r=settle(e,h)
    assert r['contact_latched'] and r['score']==0
    assert r['contact_witness']['physics_sample']==6
    e.reset();assert settle(e,History())['complete']

@pytest.mark.parametrize('fault',['stale','gap','unavailable','history_truncated'])
def test_monitor_fault_invalidates_until_reset(fault):
    e,h=Evaluator(),History();tick(e,h)
    if fault=='gap':h.samples+=24
    elif fault=='unavailable':h.ready=False
    elif fault=='history_truncated':h.events=[]
    r=e.update(pose(),h);assert not r['valid'] and r['score']==0
    h.ready=True
    assert not settle(e,h)['valid']
    e.reset();assert settle(e,History())['complete']

@pytest.mark.parametrize('p',[np.zeros(7),np.ones(6),[0,0,0,0,0,0,float('nan')],[0,0,0,0,0,0,2]])
def test_malformed_pose_fails_closed(p):
    assert not tick(Evaluator(),History(),p)['valid']

def test_quaternion_sign_does_not_change_pose_or_stillness():
    e,h=Evaluator(),History();p=pose()
    for _ in range(21):p[3:]*=-1;r=tick(e,h,p)
    assert r['complete']

@pytest.mark.parametrize('a,b,bad',[
 (FINGER,COLLAR,False),(FINGER.rsplit('/',1)[0],COLLAR,True),
 (COLLAR,ROOT+'guide_00/geometry/mesh',True),(COLLAR,ROOT+'anchor_start/geometry/mesh',True),
 (COLLAR,GOAL,False),(COLLAR,ROOT+'base/geometry/mesh',False),(FINGER,GOAL,True),
 (FINGER,FINGER.replace('LeftArm','RightArm'),True)])
def test_contact_classes_and_pair_order(a,b,bad):
    m=Monitor();assert m.prohibited(a,b)==bad and m.prohibited(b,a)==bad

def test_unknown_contact_is_not_silently_safe():
    with pytest.raises(RuntimeError):Monitor().prohibited(FINGER,ROOT+'unknown/geometry/mesh')
