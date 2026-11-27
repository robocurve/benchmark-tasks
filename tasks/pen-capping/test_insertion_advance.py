from pen_capping.alignment import InsertionAdvance
import pytest

def test_free_approach_stops_before_barrel():
    s=InsertionAdvance()
    assert s.step(-.08,-.14,.05)==-.100

def test_speed_and_lag_are_bounded():
    s=InsertionAdvance();actual=-.1;previous=actual
    for i in range(300):
        target=s.step(-.0806,actual,.05)
        assert target-previous <= .00015+1e-12
        assert target-actual <= .0025+1e-12
        actual += .2*(target-actual)
        previous=target
    assert actual==pytest.approx(-.0806,abs=1e-6)

def test_blocked_guard_withdraws_target_and_stall_cannot_wind_up():
    s=InsertionAdvance()
    for _ in range(100):target=s.step(-.08,-.095,.05)
    assert target==pytest.approx(-.0925)
    assert s.step(-.100,-.095,.05)==-.100

def test_speed_limit_does_not_clear_axial_compensation(monkeypatch):
    import sys
    import numpy as np
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    body=np.array([.27,-.02,.85]);cap=body+np.array([-.101,0,0])
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK={
        'poses':{'pen_0':np.r_[body,[0,0,0,1]],'cap_0':np.r_[cap,[0,0,0,1]]},
        'rotations':{'pen_0':np.eye(3),'cap_0':np.eye(3)},'pairs':{},'backing_contacts':{}}))
    point=ToolWaypoint(5.,tuple(cap),0.,'insert',object_name='cap_0',follow_pen=0,depth=-.0806)
    plan=FeedbackPlan([point],(0,0,0),'left');plan.target(3.)
    follow=plan.object_diagnostic['follow']
    assert follow['effective_depth_m']<follow['requested_depth_m']
    assert not follow['depth_guard_blocked']
    assert follow['bias_axial_m']>0


def test_alignment_pause_preserves_the_commanded_depth():
    s=InsertionAdvance()
    target=s.step(-.08,-.1,.05)
    for _ in range(10):
        assert s.step(-.08,-.1005,.05,ready=False)==target
    assert s.step(-.08,-.1005,.05,ready=True)>target
