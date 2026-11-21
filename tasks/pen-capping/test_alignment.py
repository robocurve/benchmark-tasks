"""Active002-controller regression and command continuity; no Isaac Sim launch."""
import math
import numpy as np
import pytest
from pen_capping.alignment import AxisServo,matrix,rotation_vector,rotation_about_object,safe_depth,RetainedReleaseClock
from pen_capping.retention import engagement,SEATED_X


def test_axis_integral_removes_persistent_tracking_disturbance():
    servo=AxisServo();theta=math.radians(2)
    for _ in range(240):
        turn=servo.correction((math.cos(theta),math.sin(theta),0),(1,0,0),.05)
        theta+=.3*(math.atan2(turn[1,0],turn[0,0])+math.radians(2))
    assert abs(theta)<math.radians(.1)
    assert np.linalg.norm(servo.integral)<=math.radians(3)


def test_axis_command_and_integral_are_bounded():
    servo=AxisServo()
    for _ in range(200):
        rotation=servo.correction((0,1,0),(1,0,0),.05)
        assert math.acos(np.clip((np.trace(rotation)-1)/2,-1,1))<=math.radians(4)+1e-12
    assert np.linalg.norm(servo.integral)==0


def test_pivot_compensation_preserves_object_root():
    tcp=np.array([.3,-.02,.85]);root=tcp+np.array([-.055,0,-.01]);rotation=matrix(np.array([.01,-.03,.02]))
    shifted=tcp+rotation_about_object(tcp,root,rotation)
    assert np.allclose(shifted+rotation@(root-tcp),root,atol=1e-12)


def test_approach_guard_is_original002_controller_guard():
    assert safe_depth(SEATED_X,math.radians(.7),.00049)==SEATED_X
    assert safe_depth(SEATED_X,math.radians(1),.00049)==-.100
    assert safe_depth(SEATED_X,0,.0006)==-.100
    assert safe_depth(SEATED_X,0,.0006,-.086)==-.086


def test_geometric_gate_still_rejects_tilted_engagement():
    result=engagement((SEATED_X,0,0),matrix(np.array([0,0,math.radians(2.4)])))
    assert not result['eligible'] and 'axis_angle' in result['failed_conditions']
    assert result['overlap_m']>0 and result['eccentricity_m']>.0003


def test_opposite_axis_rotation_is_defined():
    delta=matrix(rotation_vector((1,0,0),(-1,0,0)))
    assert np.allclose(delta@np.array([1.,0,0]),[-1.,0,0])


def test_release_clock_waits_for_observed_retention_and_half_second_hold():
    clock=RetainedReleaseClock([(1.,0)]);time=.95
    for _ in range(40):time=clock.advance(time,.05,{});assert time<1. and clock.failed is None
    for _ in range(9):time=clock.advance(time,.05,{0:0});assert time<1.
    assert clock.advance(time,.05,{0:0})>=1.


def test_release_wait_is_bounded_and_budget_fits():
    clock=RetainedReleaseClock([(1.,0)]);time=.95
    for _ in range(125):
        time=clock.advance(time,.05,{})
        if clock.failed is not None:break
    assert clock.failed==0 and clock.waited[0]<=6.+1e-9
    assert 261+4*6<=300


@pytest.mark.parametrize('side,name,label,follow',[('left','cap_0','seat',0),('right','pen_0','hold seat',None)])
def test_engagement_preserves_last_command_not_offset_measured_pose(monkeypatch,side,name,label,follow):
    import sys
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    from yam_arena import ik
    from yam_arena.constants import READY,FINGER_OPEN,FINGER_CLOSED
    body=np.array([.30,0,.85]);cap=body+np.array([SEATED_X,0,0])
    feedback=dict(poses={'pen_0':np.r_[body,[0,0,0,1]],'cap_0':np.r_[cap,[0,0,0,1]]},
                  rotations={'pen_0':np.eye(3),'cap_0':np.eye(3)},pairs={0:0})
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK=feedback))
    points=[ToolWaypoint(1.,tuple(cap),FINGER_CLOSED,label,object_name=name,follow_pen=follow,depth=SEATED_X),
            ToolWaypoint(1.,tuple(cap),FINGER_CLOSED,'hold reached',object_name=name),
            ToolWaypoint(1.,tuple(cap),FINGER_OPEN,'release supporting hand',object_name=name,release=True),
            ToolWaypoint(1.,(.4,0,1.),FINGER_OPEN,'retreat supporting hand')]
    plan=FeedbackPlan(points,(0,0,0),side)
    q=np.asarray(READY).copy();q[0]+=.01;command_p,command_r=ik.tcp_pose(q,(0,0,0))
    plan.q=q.copy();plan.last=(command_p.copy(),command_r.copy(),FINGER_CLOSED,label)
    plan.actual_pose=(command_p+np.array([.001,0,0]),matrix(np.array([0,0,math.radians(2)]))@command_r)
    for t in (.1,.8,1.2,2.5):
        action,_=plan.action7(t)
        assert np.array_equal(action[:6],q)
        assert np.array_equal(plan.last[0],command_p) and np.array_equal(plan.last[1],command_r)
    assert 0<action[6]<1  # support finger opens while arm target stays exact
    assert not np.allclose(plan.action7(3.5)[0][:6],q)  # scheduled retreat runs


def test_nonretained_ik_matches_base_action7(monkeypatch):
    import sys
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    from yam_arena import ik
    from yam_arena.demo import ArmPlan
    from yam_arena.constants import READY
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK={}))
    points=[ToolWaypoint(1.,(0,0,0),0.,'test')]
    plan=FeedbackPlan(points,(0,0,0),'left');base=ArmPlan(points,(0,0,0))
    position,rotation=ik.tcp_pose(np.asarray(READY),(0,0,0));position+=np.array([.001,0,0])
    def target(t):
        plan.active_hold=None;plan.last=(position,rotation,0.,'test');return plan.last
    plan.target=target;base.target=lambda t:(position,rotation,0.,'test')
    for _ in range(3):assert np.array_equal(plan.action7(.1)[0],base.action7(.1)[0])


def test_action_diagnostics_separate_tracking_from_ik_error(monkeypatch,capsys):
    import sys,json
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    from yam_arena import ik
    from yam_arena.constants import READY
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK={}))
    plan=FeedbackPlan([ToolWaypoint(1.,(0,0,0),0.,'diagnostic')],(0,0,0),'left')
    position,rotation=ik.tcp_pose(np.asarray(READY),(0,0,0))
    def target(t):
        plan.active_hold=None;plan.last=(position,rotation,0.,'diagnostic');return plan.last
    plan.target=target;plan.actual_pose=(position+np.array([.001,0,0]),rotation)
    for _ in range(10):plan.action7(.1)
    row=json.loads([l.split(' ',1)[1] for l in capsys.readouterr().out.splitlines() if l.startswith('ARM_CONTROL ')][0])
    assert row['ik_position_error_m']<1e-10
    assert row['observed_tracking_position_error_m']==pytest.approx(.001)


def test_plan_preserves_complete_four_pair_budget():
    pytest.importorskip('yam_arena')
    from pen_capping.policy import make_plan
    plans=make_plan(.7475,[(.16,-.16,.7575,0)]*4,[(.18,.15,.7638,0)]*4,['red','blue','red','blue'])
    durations=[sum(w.duration for w in points) for points in plans.values()]
    assert durations==[256,256] and max(durations)+5+24<=300


def test_deeper_controller_target_adds_overlap_with_nib_clearance():
    pytest.importorskip('yam_arena')
    from pen_capping.policy import INSERT_TARGET_X,make_plan
    assert SEATED_X==-.0821
    assert INSERT_TARGET_X==pytest.approx(-.0806)
    fit=engagement((INSERT_TARGET_X,0,0))
    assert fit['eligible'] and fit['overlap_m']==pytest.approx(.0054)
    assert fit['tip_clearance_m']==pytest.approx(.0027)
    plans=make_plan(.7475,[(.16,-.16,.7575,0)]*4,[(.18,.15,.7638,0)]*4,['red','blue','red','blue'])
    travel=[w.depth for w in plans['left'] if w.label in ('insert','seat')]
    assert len(travel)==8 and all(depth==INSERT_TARGET_X for depth in travel)


def test_blocked_guard_clears_axial_integral_preserving_world_lateral_component():
    from pen_capping.alignment import follow_bias_update
    axis=np.array([-1.,-1.,0.])/math.sqrt(2)
    lateral=np.array([.0002,-.0002,.0001]);previous=.00366*axis+lateral
    updated=follow_bias_update(previous,np.zeros(3),axis,np.zeros(3),True)
    assert abs(updated@axis)<1e-12
    assert np.allclose(updated,lateral,atol=1e-12)


def test_blocked_guard_keeps_proportional_axial_restoring_error():
    from pen_capping.alignment import follow_bias_update
    error=np.array([.003,0,0])
    integral=follow_bias_update(np.array([.002,0,0]),error,(1,0,0),(0,0,0),True)
    assert integral[0]==pytest.approx(0)
    assert (error+integral)[0]==pytest.approx(.003)


def test_saturation_stops_outward_windup_but_allows_unwinding():
    from pen_capping.alignment import follow_bias_update
    previous=np.array([.002,0,0]);axis=(1,0,0)
    outward=follow_bias_update(previous,(.003,0,0),axis,(0,0,0),False)
    assert outward[0]==pytest.approx(previous[0])
    inward=follow_bias_update(previous,(-.003,0,0),axis,(0,0,0),False)
    assert inward[0]<previous[0]


def test_guard_projection_is_bounded_for_world_axis_rotations():
    from pen_capping.alignment import follow_bias_update
    for axis in [(1,0,0),(0,0,1),(-.7,-.7,.03)]:
        bias=follow_bias_update((.003,-.003,.003),(.005,.005,-.005),axis,(0,0,0),True)
        assert np.linalg.norm(bias)<=math.sqrt(3)*.003+1e-12
        assert abs(bias@np.asarray(axis))<1e-12


def test_body_latch_after_alignment_preserves_command_and_ignores_root_motion(monkeypatch):
    import sys
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    from yam_arena import ik
    from yam_arena.constants import READY
    feedback=dict(poses={'pen_2':np.r_[[.27,-.02,.85],[0,0,0,1]]},rotations={'pen_2':np.eye(3)},pairs={})
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK=feedback))
    points=[ToolWaypoint(6.,(.27,-.02,.85),0.,'hold insertion',object_name='pen_2'),
            ToolWaypoint(5.,(.27,-.02,.85),0.,'hold seat',object_name='pen_2')]
    plan=FeedbackPlan(points,(0,0,0),'right');q=np.asarray(READY).copy();q[0]+=.01
    commanded_p,commanded_r=ik.tcp_pose(q,(0,0,0));plan.q=q.copy();plan.last=(commanded_p.copy(),commanded_r.copy(),0.,'align body')
    plan.actual_pose=(commanded_p+.001,matrix(np.array([.01,0,.02]))@commanded_r)
    first=plan.action7(.05)[0]
    assert np.array_equal(first[:6],q)
    feedback['poses']['pen_2'][:3]+=.003
    for t in (1.,5.,6.5,10.):
        action=plan.action7(t)[0]
        assert np.array_equal(action[:6],q)
        assert np.array_equal(plan.last[0],commanded_p) and np.array_equal(plan.last[1],commanded_r)


def test_body_latch_does_not_bypass_carry_or_alignment_controller(monkeypatch):
    import sys
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    from yam_arena import ik
    from yam_arena.constants import READY
    feedback=dict(poses={'pen_0':np.r_[[.2,0,.85],[0,0,0,1]]},rotations={'pen_0':np.eye(3)},pairs={})
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK=feedback))
    points=[ToolWaypoint(4.,(.27,-.02,.85),0.,'align body',object_name='pen_0')]
    plan=FeedbackPlan(points,(0,0,0),'right');plan.actual_pose=ik.tcp_pose(np.asarray(READY),(0,0,0))
    plan.target(1.)
    assert not plan.body_command_holds and plan.active_hold is None


def test_release_clock_skips_unused_dwell_only_after_observed_half_second():
    clock=RetainedReleaseClock([(30.,0),(70.,1)]);time=20.
    for _ in range(9):
        time=clock.advance(time,.05,{0:0})
        assert time<30.
    time=clock.advance(time,.05,{0:0})
    assert time==pytest.approx(30.05)
    assert clock.advance(time,.05,{0:0})==pytest.approx(30.1)
