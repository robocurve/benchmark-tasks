"""Velocity braking properties and integration; no calibrated robot dynamics."""
import numpy as np
import pytest
from pen_capping.alignment import LateralDamping,matrix


def test_first_observation_and_axial_motion_cannot_create_lateral_braking():
    servo=LateralDamping()
    assert np.array_equal(servo.observe((-.1,.003,-.004),np.eye(3),.05),np.zeros(3))
    assert np.array_equal(servo.observe((-.08,.003,-.004),np.eye(3),.05),np.zeros(3))


def test_constant_lateral_motion_is_braked_and_bounded_in_current_world_frame():
    servo=LateralDamping();R=matrix(np.array([.2,-.3,.5]))
    for i in range(100):
        brake=servo.observe((-.1+i*.0002,i*.0005,-i*.0002),R,.05)
        assert abs(brake@R[:,0])<1e-12
        assert np.linalg.norm(brake)<=.001+1e-12
    velocity=R@np.array([0,.01,-.004])
    assert brake@velocity<0
    assert np.linalg.norm(brake)==pytest.approx(.001)


def test_filter_attenuates_alternating_pose_noise_and_braking_decays_at_rest():
    servo=LateralDamping();out=[]
    for i in range(100):
        y=.00005*(-1)**i
        out.append(servo.observe((-.1,y,0),np.eye(3),.05)[1])
    # A100um difference at20Hz gives300um unfiltered braking.
    assert max(abs(x) for x in out[-20:])<.0001
    last=servo.previous.copy()
    for _ in range(30):brake=servo.observe(last,np.eye(3),.05)
    assert np.linalg.norm(brake)<1e-8
    assert np.array_equal(LateralDamping().observe(last,np.eye(3),.05),np.zeros(3))


def test_servo_limits_remain_consistent_for_smaller_control_steps():
    results=[]
    for dt in [.025,.05]:
        servo=LateralDamping()
        for i in range(round(2/dt)+1):brake=servo.observe((-.1,i*dt*.001,0),np.eye(3),dt)
        results.append(brake)
    assert np.allclose(results[0],results[1],atol=1e-9)
    assert results[0][1]==pytest.approx(-.00015,abs=1e-9)


def test_controller_adds_only_lateral_braking_and_retention_preserves_last_command(monkeypatch):
    import sys
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from yam_arena import ik
    from yam_arena.constants import READY
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    body=np.array([.27,-.02,.85]);cap=body+np.array([-.101,0,0])
    feedback=dict(poses={'pen_0':np.r_[body,[0,0,0,1]],'cap_0':np.r_[cap,[0,0,0,1]]},
                  rotations={'pen_0':np.eye(3),'cap_0':np.eye(3)},pairs={},backing_contacts={})
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK=feedback))
    point=ToolWaypoint(5.,tuple(cap),0.,'insert',object_name='cap_0',follow_pen=0,depth=-.0806)
    plan=FeedbackPlan([point],(0,0,0),'left');plan.actual_pose=ik.tcp_pose(np.asarray(READY),(0,0,0))
    plan.episode_time=.05;plan.target(.05)
    feedback['poses']['cap_0'][1]+=.0001
    plan.episode_time=.1;plan.target(.1)
    info=plan.object_diagnostic['follow'];brake=np.array(info['lateral_damping_world_m'])
    assert info['lateral_damping_active'] and brake[1]<0 and abs(brake[0])<1e-12
    assert np.allclose(info['raw_command_delta_m'],np.array(info['root_error_world_m'])+info['bias_after_world_m']+np.array(info['pivot_world_m'])+brake)
    assert max(abs(x) for x in info['limited_command_delta_m'])<=.004
    last_p,last_r=plan.last[:2];last_q=plan.q.copy();feedback['pairs'][0]=0
    feedback['poses']['cap_0'][:3]+=.002
    target=plan.target(.15)
    assert np.array_equal(target[0],last_p) and np.array_equal(target[1],last_r)
    assert np.array_equal(plan.active_hold,last_q)
