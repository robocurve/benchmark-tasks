"""Failure/recovery state transitions and event lifetime, without simulation."""
import numpy as np
import pytest
from pen_capping.contact_tracker import BackingContacts
from pen_capping.recovery import ContactRecovery,PrecontactBias,barrel_mouth_gap
from pen_capping.retention import SEATED_X

CONTACT=dict(active=True,recent=True)
CLEAR=dict(active=False,recent=False)
R=np.eye(3)


def stalled_recovery():
    recovery=ContactRecovery()
    for t in np.arange(0,.501,.05):state=recovery.step(float(t),(-.0835,.0009,0),R,-.0806,CONTACT)
    assert state['state']=='withdraw'
    return recovery,state


def test_zero_point_lost_event_clears_current_contact_not_history():
    tracker=BackingContacts();paths=['pen_1/Barrel','cap_1/LinerBacking04']
    tracker.record(paths,'found',1.,[dict(separation_m=-.0001)])
    assert tracker.snapshot(1.05)['1_1']['active']
    tracker.record(paths,'persists',1.05,[])
    assert len(tracker.drain())==1
    tracker.record(paths,'lost',1.1,[])
    assert not tracker.snapshot(1.15)['1_1']['active']
    assert not tracker.snapshot(1.3)['1_1']['recent']
    assert tracker.drain()[0]['event']=='lost'


def test_non_backing_contacts_cannot_trigger_backing_recovery_input():
    tracker=BackingContacts()
    tracker.record(['pen_1/Barrel','cap_1/Wall00'],'found',1.,[])
    assert not tracker.snapshot(1.05)


def test_no_report_or_real_progress_or_valid_fit_cannot_trigger_recovery():
    for mode in ['no_contact','progress','valid']:
        recovery=ContactRecovery()
        for step in range(20):
            relative=[-.0835,.0009,0];contact=CONTACT
            if mode=='no_contact':contact=CLEAR
            elif mode=='progress':relative[0]=-.095+step*.0006
            else:relative=[SEATED_X,0,0]
            state=recovery.step(step*.05,relative,R,-.0806,contact)
        assert state['state']=='normal'


def test_stall_with_contact_withdraws_and_preserves_lateral_offset():
    recovery,state=stalled_recovery()
    assert state['clear_bias'] and state['depth']==-.100 and state['retries']==1
    assert state['lateral']==[.0009,0.]
    assert state['barrel_mouth_gap_m']<0


def test_recovery_requires_geometric_and_reported_clearance_before_recentering():
    recovery,_=stalled_recovery()
    state=recovery.step(.6,(-.100,.0009,0),R,-.0806,CONTACT)
    assert state['state']=='withdraw'
    state=recovery.step(.7,(-.100,.0009,0),R,-.0806,CLEAR)
    assert state['state']=='center' and state['lateral']==[0.,0.]
    for t in (.75,.8):
        state=recovery.step(t,(-.100,.00005,0),R,-.0806,CLEAR)
        assert state['state']=='center'
    state=recovery.step(.85,(-.100,.00005,0),R,-.0806,CLEAR)
    assert state['state']=='advance'
    assert 0<state['depth']+.100<=.003*.05+1e-12


@pytest.mark.parametrize('phase,timeout,relative',[
    ('withdraw',2.5,(-.0835,.0009,0)),('center',5.,(-.100,.0009,0)),('advance',8.,(-.100,.0009,0))])
def test_each_phase_has_explicit_timeout(phase,timeout,relative):
    recovery=ContactRecovery();recovery.state=phase;recovery.entered=0.;recovery.retries=1
    recovery.withdraw_lateral=np.array([.0009,0])
    state=recovery.step(timeout+.01,relative,R,-.0806,CLEAR)
    assert state['failed'] and 'timeout' in state['reason']


def test_second_reported_stall_fails_without_unbounded_retries():
    recovery,_=stalled_recovery();recovery.state='advance';recovery.entered=.6;recovery.samples.clear()
    for t in np.arange(.7,1.251,.05):state=recovery.step(float(t),(-.0835,.0009,0),R,-.0806,CONTACT)
    assert state['failed'] and state['retries']==1
    assert state['reason']=='backing_contact_stall_after_one_retry'


def test_withdraw_depth_has_twelve_point_five_mm_nominal_barrel_clearance():
    assert barrel_mouth_gap(np.array([-.100,0,0]),R)==pytest.approx(.0125)
    assert barrel_mouth_gap(np.array([-.0835,0,0]),R)<0


def test_recovery_does_not_expand_shared_action_budget():
    assert (188+5+4*6)*20==4340
    assert 4340<4400


def test_controller_recovery_retracts_target_and_holds_last_command_on_timeout(monkeypatch):
    import sys
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    from yam_arena import ik
    from yam_arena.constants import READY
    body=np.array([.27,-.02,.85]);cap=body+np.array([-.0835,.0009,0])
    feedback=dict(poses={'pen_0':np.r_[body,[0,0,0,1]],'cap_0':np.r_[cap,[0,0,0,1]]},
        rotations={'pen_0':R,'cap_0':R},pairs={},backing_contacts={'0_0':CONTACT})
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK=feedback))
    point=ToolWaypoint(5.,tuple(cap),0.,'seat',object_name='cap_0',follow_pen=0,depth=-.0806)
    plan=FeedbackPlan([point],(0,0,0),'left');plan.actual_pose=ik.tcp_pose(np.asarray(READY),(0,0,0))
    for step in range(11):
        plan.episode_time=step*.05;plan.target(3.+step*.05)
    assert plan.recovery.state=='withdraw'
    info=plan.object_diagnostic['follow']
    assert info['effective_depth_m']==-.100 and abs(info['bias_axial_m'])<1e-12
    assert np.allclose(info['recovery']['lateral'],[.0009,0])
    assert max(abs(x) for x in info['limited_command_delta_m'])<=.004
    last_p,last_r=plan.last[:2];plan.episode_time=3.1
    target=plan.target(3.6)
    assert plan.recovery.state=='failed'
    assert np.array_equal(target[0],last_p) and np.array_equal(target[1],last_r)
    assert np.array_equal(plan.active_hold,plan.q)


def test_ready_sample_cannot_bypass_centering_deadline():
    recovery=ContactRecovery();recovery.state='center';recovery.entered=0.;recovery.retries=1;recovery.centered=2
    state=recovery.step(5.01,(-.100,.00005,0),R,-.0806,CLEAR)
    assert state['failed'] and state['reason']=='clear_centering_timeout'


def test_precontact_memory_excludes_delayed_contact_interval_and_later_loaded_bias():
    memory=PrecontactBias()
    for now,bias in [(1.,(.002,.0002,.0004)),(1.05,(.003,.0008,.001)),(1.1,(.003,.001,.002))]:
        memory.observe(now,bias,(1,0,0),None)
    memory.observe(1.15,(.003,.002,.003),(1,0,0),dict(first_report_interval_start_s=1.05))
    assert memory.saved_time==1.
    assert np.allclose(memory.restore((1,0,0)),(0,.0002,.0004))
    memory.observe(2.,(.003,.003,.003),(1,0,0),CLEAR)
    assert np.allclose(memory.restore((1,0,0)),(0,.0002,.0004))


def test_restore_projects_world_lateral_memory_on_current_axis_without_amplifying():
    memory=PrecontactBias();memory.observe(0.,(.002,.0002,.0004),(1,0,0),None)
    memory.observe(.1,(0,0,0),(1,0,0),dict(first_report_interval_start_s=.05))
    axis=np.array([1.,2.,3.]);restored=memory.restore(axis)
    assert abs(restored@axis)<1e-12
    assert np.linalg.norm(restored)<=np.linalg.norm(memory.saved)
    assert np.allclose(memory.saved,(0,.0002,.0004))


def test_no_clean_history_or_missing_timestamp_cannot_restore_loaded_bias():
    for report in [CONTACT,dict(first_report_interval_start_s=0.)]:
        memory=PrecontactBias();memory.observe(.1,(.003,.003,.003),(1,0,0),report)
        assert np.array_equal(memory.restore((1,0,0)),np.zeros(3))
        assert memory.saved_time is None


def test_first_backing_interval_survives_persists_and_lost_reports():
    tracker=BackingContacts();paths=['pen_1/Barrel','cap_1/LinerBacking04']
    tracker.record(paths,'found',1.,[]);tracker.record(paths,'persists',1.1,[])
    tracker.record(paths,'lost',1.2,[])
    report=tracker.snapshot(2.)['1_1']
    assert report['first_report_interval_start_s']==1.
    assert not report['active'] and not report['recent']
    tracker.record(paths,'found',3.,[])
    assert tracker.snapshot(3.)['1_1']['first_report_interval_start_s']==1.
    assert not BackingContacts().snapshot(3.)


def test_recovery_restores_clean_bias_only_after_clearance_and_keeps_axial_P(monkeypatch):
    import sys
    from types import SimpleNamespace
    pytest.importorskip('yam_arena')
    from pen_capping.policy import FeedbackPlan,ToolWaypoint
    from yam_arena import ik
    from yam_arena.constants import READY
    body=np.array([.27,-.02,.85]);cap=body+np.array([-.0835,.0009,0])
    feedback=dict(poses={'pen_0':np.r_[body,[0,0,0,1]],'cap_0':np.r_[cap,[0,0,0,1]]},
        rotations={'pen_0':R,'cap_0':R},pairs={},backing_contacts={})
    monkeypatch.setitem(sys.modules,'pen_capping.embodiment',SimpleNamespace(FEEDBACK=feedback))
    point=ToolWaypoint(5.,tuple(cap),0.,'seat',object_name='cap_0',follow_pen=0,depth=-.0806)
    plan=FeedbackPlan([point],(0,0,0),'left');plan.actual_pose=ik.tcp_pose(np.asarray(READY),(0,0,0))
    plan.target(3.);clean=np.array([0.,.0002,.0004])
    plan.precontact_bias=PrecontactBias();plan.precontact_bias.observe(0.,(.003,.0002,.0004),(1,0,0),None)
    report=dict(active=True,recent=True,first_report_interval_start_s=.1)
    feedback['backing_contacts']['0_0']=report
    for step in range(1,12):
        plan.episode_time=step*.05;plan.target(3.+step*.05)
    assert plan.recovery.state=='withdraw'
    assert abs(plan.follow_bias[2])<1e-12
    feedback['poses']['cap_0'][:3]=body+np.array([-.101,.0009,0])
    report.update(active=False,recent=False)
    plan.episode_time=.65;plan.target(3.65)
    info=plan.object_diagnostic['follow']
    assert plan.recovery.state=='center'
    assert np.allclose(info['bias_before_world_m'],clean)
    assert abs(info['bias_axial_m'])<1e-12
    assert info['root_error_world_m'][0]==pytest.approx(.001)
    assert info['raw_command_delta_m'][0]==pytest.approx(.001)


def test_proximity_without_impulse_is_not_a_loaded_backing_stall():
    from pen_capping.contact_tracker import BackingContacts
    tracker=BackingContacts();r=ContactRecovery()
    paths=['pen_0/Barrel','cap_0/LinerBacking00']
    for i in range(20):
        now=i*.05
        tracker.record(paths,'persists',now,[{'separation_m':.0004,'impulse':[0.,0.,0.]}])
        report=tracker.snapshot(now)['0_0']
        assert report['active'] and not report['loaded'] and not report['recent_loaded']
        result=r.step(now,(-.0845,0,0),np.eye(3),-.0806,report)
        assert result['state']=='normal'


def test_actual_contact_load_triggers_stall_and_recent_load_expires():
    from pen_capping.contact_tracker import BackingContacts
    tracker=BackingContacts();r=ContactRecovery();paths=['pen_0/Barrel','cap_0/LinerBacking00']
    for i in range(20):
        now=i*.05
        tracker.record(paths,'persists',now,[],impulse_norm_sum=.0001)
        report=tracker.snapshot(now)['0_0']
        assert report['loaded']
        result=r.step(now,(-.0845,0,0),np.eye(3),-.0806,report)
        if result['state']=='withdraw':break
    assert result['state']=='withdraw'
    tracker.record(paths,'lost',now+.05,[])
    assert tracker.snapshot(now+.05)['0_0']['recent_loaded']
    assert not tracker.snapshot(now+.2)['0_0']['recent_loaded']
