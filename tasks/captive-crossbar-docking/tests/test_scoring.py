"""Synthetic raw measurements test scoring, not physical robot feasibility."""
import math
import numpy as np
import pytest

from captive_crossbar_docking import contract as C
from captive_crossbar_docking.scoring import DockingScorer, config_dict, part_corners


def record(step=0, *, q=None, p=None, yaw=0., opening=1., tcp=None):
    q = C.CARRIER_INITIAL_Q if q is None else q
    p = np.array(C.CARRIER_INITIAL_POS if p is None else p, dtype=float)
    rot = np.array([[math.cos(yaw),-math.sin(yaw),0],
                    [math.sin(yaw),math.cos(yaw),0],[0,0,1]])
    quat = [0,0,math.sin(yaw/2),math.cos(yaw/2)]
    bp = p+rot @ (np.array(C.CROSSBAR_ORIGIN_LOCAL)+[0,q,0])
    return dict(t=step*.05, step=step,
                poses=dict(carrier=[*p,*quat], crossbar=[*bp,*quat], dock=[*C.DOCK_POS,0,0,0,1]),
                velocities=dict(carrier=[0.]*6,crossbar=[0.]*6),
                joint_position=q, joint_velocity=0.,
                tcp={h:[*(tcp if tcp is not None else [.05, .3 if h=='left' else -.3,1.1]),0,0,0,1] for h in ('left','right')},
                grippers=dict(left=opening,right=opening), joints=[0.]*14)


def seeded():
    scorer = DockingScorer()
    assert scorer.update(record())['score'] == 0
    return scorer


def settled(scorer, q=0., p=C.CARRIER_FINAL_POS, samples=25, **kw):
    start = 1 if scorer.previous is None else scorer.previous[1]+1
    result = None
    for step in range(start,start+samples):
        result = scorer.update(record(step,q=q,p=p,**kw))
    return result


def test_nominal_full_and_nonlatched_terminal():
    s = seeded()
    result = settled(s)
    assert result['score'] == 1
    assert result['success']
    assert result['release_hold_seconds'] >= 1
    assert settled(s,q=.05)['score'] == 0


def test_partial_outcomes_and_complete_body_occupancy():
    s = seeded()
    assert settled(s,p=C.CARRIER_INITIAL_POS)['score'] == .25
    assert settled(s,p=(C.DOCK_MOUTH,0,C.TABLE_TOP+.008))['score'] == .5
    assert settled(s,p=(.379,0,C.TABLE_TOP+.008))['score'] == .25
    assert settled(s,p=(.46,0,C.TABLE_TOP+.008))['score'] == .5
    assert settled(s)['score'] == 1


@pytest.mark.parametrize('q,score',[(.004,.25),(-.004,.25),(.004001,0),(-.004001,0)])
def test_exact_compaction_boundaries(q,score):
    assert settled(seeded(),q=q,p=C.CARRIER_INITIAL_POS)['score'] == score


@pytest.mark.parametrize('q',[.05,-.05])
def test_offcenter_joint_never_scores_even_if_body_docked(q):
    assert settled(seeded(),q=q)['score'] == 0


@pytest.mark.parametrize('z',[C.TABLE_TOP+.014,C.TABLE_TOP+.1])
def test_hover_cannot_score(z):
    r = settled(seeded(),p=(.475,0,z))
    assert r['score'] == 0
    assert not r['supported']


@pytest.mark.parametrize('yaw',[math.pi/2,math.pi,math.pi/4])
def test_rotated_bypass_no_full(yaw):
    assert settled(seeded(),yaw=yaw)['score'] < 1


def test_actual_crossbar_and_joint_must_agree():
    s = seeded()
    r = record(1,q=0)
    r['poses']['crossbar'][1] = .05
    assert s.update(r)['invalid_reason'] == 'broken_or_inconsistent_linkage'
    assert settled(s)['score'] == 0


@pytest.mark.parametrize('delta,invalid',[(.002,False),(.00201,True)])
def test_link_translation_boundary(delta,invalid):
    s = seeded()
    r = record(1,q=0)
    r['poses']['crossbar'][0] += delta
    assert s.update(r)['invalid'] == invalid


def test_orientation_linkage_break_detected():
    s = seeded()
    r = record(1,q=0)
    r['poses']['crossbar'][3:] = [0,0,math.sin(.03),math.cos(.03)]
    assert s.update(r)['invalid']


def test_fixed_dock_cannot_follow_tool():
    s = seeded()
    r = record(1,q=0)
    r['poses']['dock'][0] -= .02
    assert s.update(r)['invalid_reason'] == 'fixed_dock_moved'


def test_drop_is_permanent():
    s = seeded()
    assert s.update(record(1,p=(.28,0,.6)))['invalid_reason'] == 'assembly_dropped_below_table'
    assert settled(s)['invalid']


@pytest.mark.parametrize('field',['pose','velocity','joint','joint_velocity','tcp','gripper','robot_joint','time'])
def test_nonfinite_invalidates(field):
    s = seeded()
    r = record(1)
    if field == 'pose': r['poses']['carrier'][0] = float('nan')
    if field == 'velocity': r['velocities']['crossbar'][0] = float('inf')
    if field == 'joint': r['joint_position'] = float('nan')
    if field == 'joint_velocity': r['joint_velocity'] = float('nan')
    if field == 'tcp': r['tcp']['left'][1] = float('nan')
    if field == 'gripper': r['grippers']['left'] = float('nan')
    if field == 'robot_joint': r['joints'][4] = float('nan')
    if field == 'time': r['t'] = float('nan')
    assert s.update(r)['invalid']


@pytest.mark.parametrize('step,t',[(0,.05),(1,0),(-1,-.05),(1.5,.05)])
def test_nonmonotonic_or_fractional_time_step_invalid(step,t):
    s = seeded()
    r = record(1)
    r.update(step=step,t=t)
    assert s.update(r)['invalid']


def test_starting_full_or_compact_not_credited():
    assert DockingScorer().update(record(p=C.CARRIER_FINAL_POS,q=0))['invalid']
    assert DockingScorer().update(record(q=0))['invalid']


def test_one_second_continuous_release_required():
    s = seeded()
    assert settled(s,samples=21)['score'] == .5  # first arrival sample was moving
    assert settled(s,samples=1)['score'] == 1
    assert settled(s,samples=1,opening=.7999)['score'] == .5
    assert settled(s,samples=20)['score'] == .5
    assert settled(s,samples=1)['score'] == 1


def test_missing_samples_break_hold():
    s = seeded()
    settled(s,samples=12)
    last = s.previous[1]
    assert s.update(record(last+20,q=0,p=C.CARRIER_FINAL_POS))['score'] == .5
    assert settled(s,samples=19)['score'] == .5


def test_opening_threshold_and_clearance_include_rear_handle():
    assert settled(seeded(),opening=.8)['score'] == 1
    assert settled(seeded(),opening=.8-1e-5)['score'] == .5
    handle_centre = np.array(C.CARRIER_FINAL_POS)+C.CARRIER_GRIP_LOCAL
    assert settled(seeded(),tcp=handle_centre)['score'] == .5
    assert settled(seeded(),tcp=handle_centre+[0,0,.096])['score'] == 1  # .016 half handle + .080
    assert settled(seeded(),tcp=handle_centre+[0,0,.09599])['score'] == .5


@pytest.mark.parametrize('reported,expected',[(.01,1),(.01001,0)])
def test_reported_speed_threshold(reported,expected):
    s = seeded()
    for step in range(1,27):
        r = record(step,q=0,p=C.CARRIER_FINAL_POS)
        r['velocities']['carrier'][0] = reported
        result = s.update(r)
    assert result['score'] == expected


def test_false_zero_reported_motion_cannot_accumulate_hold():
    s = seeded()
    for step in range(1,40):
        x = .475 if step%2 else .480
        result = s.update(record(step,q=0,p=(x,0,C.TABLE_TOP+.008)))
    assert result['score'] == 0
    assert result['linear_speed_m_s'] > .01


@pytest.mark.parametrize('yaw_rate,expected',[(.1,1),(.10001,0)])
def test_angular_speed_threshold(yaw_rate,expected):
    s = seeded()
    for step in range(1,27):
        r = record(step,q=0,p=C.CARRIER_FINAL_POS)
        r['velocities']['crossbar'][5] = yaw_rate
        result = s.update(r)
    assert result['score'] == expected


@pytest.mark.parametrize('x,expected',[(.4695,1),(.46949,.5)])
def test_full_payload_x_boundary(x,expected):
    assert settled(seeded(),p=(x,0,C.TABLE_TOP+.008))['score'] == expected


def test_both_hands_must_release():
    s = seeded()
    for step in range(1,27):
        r = record(step,q=0,p=C.CARRIER_FINAL_POS)
        r['grippers']['right'] = .4
        result = s.update(r)
    assert result['score'] == .5


def test_invalid_quaternion_and_missing_measurement_fail_closed():
    for mutate in [lambda r: r['poses']['carrier'].__setitem__(6,2.),
                   lambda r: r.pop('joint_position'),
                   lambda r: r['tcp'].pop('right')]:
        s = seeded()
        r = record(1)
        mutate(r)
        assert s.update(r)['invalid']


def test_collision_assisted_centering_is_allowed():
    s = seeded()
    for step in range(1,27):
        r = record(step,q=0,p=C.CARRIER_FINAL_POS)
        r['contacts'] = [{'objects':['crossbar','dock'],'force':12.}]
        r['phase'] = 'accidental_wall_contact'
        result = s.update(r)
    assert result['score'] == 1


def test_new_scorer_resets_hold_and_invalid_state():
    first = seeded()
    assert settled(first)['success']
    second = seeded()
    assert settled(second,samples=10)['score'] == .5
    bad = record(11)
    bad['joint_position'] = float('nan')
    assert second.update(bad)['invalid']
    third = seeded()
    assert settled(third)['success']


def test_policy_flags_cannot_forge_success():
    s = seeded()
    r = record(1)
    r.update(score=1.,success=True,phase='complete',accepted=True)
    assert s.update(r)['score'] == 0


def test_geometric_yaw_sweep_uncompacted_never_fits_opening():
    """Even free lateral translation cannot fit the initial envelope at any yaw."""
    fits = []
    centred_fits = []
    for q,target in [(C.CARRIER_INITIAL_Q,fits),(0.,centred_fits)]:
        for yaw in np.linspace(-math.pi,math.pi,1441):
            r = np.array([[math.cos(yaw),-math.sin(yaw),0],
                          [math.sin(yaw),math.cos(yaw),0],[0,0,1]])
            points = [part_corners(np.zeros(3),r,s,c) for n,s,c,_ in C.CARRIER_PARTS if n in C.PAYLOAD_CARRIER_PART_NAMES]
            bp = r @ (np.array(C.CROSSBAR_ORIGIN_LOCAL)+[0,q,0])
            points += [part_corners(bp,r,s,c) for _,s,c,_ in C.CROSSBAR_PARTS]
            points = np.concatenate(points)
            widths = np.ptp(points,axis=0)
            target.append(widths[1] <= 2*C.DOCK_HALF_WIDTH and widths[0] <= C.DOCK_BACK-C.DOCK_MOUTH)
    assert not any(fits)
    assert any(centred_fits)


def test_config_matches_contract():
    cfg = config_dict()
    assert cfg['dock_mouth'] == C.DOCK_MOUTH
    assert cfg['carrier_parts'] == C.CARRIER_PARTS
    assert cfg['crossbar_parts'] == C.CROSSBAR_PARTS
    assert cfg['payload_carrier_parts'] == C.PAYLOAD_CARRIER_PART_NAMES
