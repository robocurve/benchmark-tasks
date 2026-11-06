# SPDX-License-Identifier: MIT
"""Adversarial measurement episodes, without Isaac Sim or controller flags."""
from copy import deepcopy
import math
import json
import numpy as np
import pytest
from missing_washer_rework import contract as C
from missing_washer_rework.scoring import ReworkScorer
from missing_washer_rework.regrade import regrade_records, regrade_file


def pose(xyz, q=(0,0,0,1)):
    return list(xyz)+list(q)


def record():
    return dict(t=0., step=0, poses=dict(
        fixture=pose(C.FIXTURE_POS), cap=pose(C.CAP_INITIAL), washer=pose(C.WASHER_INITIAL),
        reference_fixture=pose(C.REFERENCE_POS),
        reference_washer=pose(np.array(C.REFERENCE_POS)+C.WASHER_SEATED_LOCAL),
        reference_cap=pose(np.array(C.REFERENCE_POS)+[0,0,.022])),
        velocities={n:[0.]*6 for n in C.DYNAMIC_NAMES},
        tcp=dict(left=pose((.1,-.3,1.)),right=pose((.1,.3,1.))),
        grippers=dict(left=1.,right=1.),joints=[0.]*14)


def advance(trace, duration=.05, **poses):
    r = deepcopy(trace[-1])
    r['t'] = round(r['t']+duration, 8)
    r['step'] += 1
    for name, value in poses.items():
        r['poses'][name] = pose(value) if len(value)==3 else list(value)
    trace.append(r)
    return r


def dwell(trace, n=36, **poses):
    for _ in range(n):
        advance(trace, **poses)


def park_trace(park_xy=(.17,-.12)):
    trace = [record()]
    advance(trace,cap=(.29,0,C.TABLE_TOP+.10))
    advance(trace,cap=(*park_xy,C.TABLE_TOP+(.006 if park_xy==(.17,-.12) else 0.)))
    dwell(trace,10)
    return trace


def success_trace():
    trace=park_trace()
    advance(trace,washer=np.array(C.FIXTURE_POS)+C.WASHER_SEATED_LOCAL)
    advance(trace,cap=np.array(C.FIXTURE_POS)+[0,0,.022])
    dwell(trace)
    return trace


def grade(trace):
    return regrade_records(trace)['terminal']


def test_physical_ordered_rework_and_release_succeeds():
    m=grade(success_trace())
    assert m['success'] and m['score']==1.
    assert m['stable_seconds']>=1.5


def test_parking_is_not_a_prescribed_tray_route():
    trace=park_trace((.15,.15))
    advance(trace,washer=np.array(C.FIXTURE_POS)+C.WASHER_SEATED_LOCAL)
    advance(trace,cap=np.array(C.FIXTURE_POS)+[0,0,.022])
    dwell(trace)
    assert grade(trace)['success']


def test_stage_credit_and_current_outcome_not_historical_max():
    trace=park_trace()
    assert grade(trace)['score']==.25
    advance(trace,washer=np.array(C.FIXTURE_POS)+C.WASHER_SEATED_LOCAL)
    assert grade(trace)['score']==.5
    advance(trace,washer=C.WASHER_INITIAL)
    assert grade(trace)['score']==.25


@pytest.mark.parametrize('field', ['pose','velocity','tcp','gripper','joints','time'])
def test_nonfinite_measurements_poison_episode(field):
    trace=success_trace()
    r=trace[3]
    if field=='pose': r['poses']['washer'][0]=float('nan')
    elif field=='velocity': r['velocities']['cap'][2]=float('inf')
    elif field=='tcp': r['tcp']['left'][0]=float('nan')
    elif field=='gripper': r['grippers']['right']=float('nan')
    elif field=='joints': r['joints'][4]=float('inf')
    else: r['t']=float('nan')
    assert grade(trace)['score']==0.
    assert not grade(trace)['valid']


def test_preexisting_success_has_no_rework_credit():
    trace=[record()]
    trace[0]['poses']['washer']=pose(np.array(C.FIXTURE_POS)+C.WASHER_SEATED_LOCAL)
    trace[0]['poses']['cap']=pose(np.array(C.FIXTURE_POS)+[0,0,.022])
    dwell(trace)
    assert not grade(trace)['success']
    assert not grade(trace)['valid']


def test_local_or_com_frame_cannot_masquerade_as_actor_world_frame():
    trace=success_trace()
    for r in trace:
        for p in r['poses'].values():
            p[2]-=C.TABLE_TOP
    assert not grade(trace)['valid']


def test_washer_seated_before_cap_park_is_not_rework():
    trace=[record()]
    advance(trace,cap=(.29,0,C.TABLE_TOP+.10),washer=np.array(C.FIXTURE_POS)+C.WASHER_SEATED_LOCAL)
    advance(trace,cap=(.17,-.12,C.TABLE_TOP+.006))
    dwell(trace,10)
    advance(trace,cap=np.array(C.FIXTURE_POS)+[0,0,.022])
    dwell(trace)
    assert grade(trace)['score']==.25
    assert not grade(trace)['success']


def test_lateral_teleport_without_observed_cap_lift_is_not_disassembly():
    trace=[record()]
    advance(trace,cap=(.17,-.12,C.TABLE_TOP+.006))
    dwell(trace,10)
    advance(trace,washer=np.array(C.FIXTURE_POS)+C.WASHER_SEATED_LOCAL)
    advance(trace,cap=np.array(C.FIXTURE_POS)+[0,0,.022])
    dwell(trace)
    assert grade(trace)['score']==0.


@pytest.mark.parametrize('offset',[(.02,0,0),(0,0,.015)])
def test_washer_beside_post_or_hovering_does_not_seat(offset):
    trace=success_trace()
    for r in trace[13:]:
        r['poses']['washer']=pose(np.array(C.FIXTURE_POS)+C.WASHER_SEATED_LOCAL+np.array(offset))
    assert not grade(trace)['success']
    assert grade(trace)['score']==.25


def test_hovering_cap_held_by_closed_hand_fails():
    trace=success_trace()
    for r in trace[14:]:
        r['poses']['cap'][2]+=.015
        r['grippers']['right']=0.
        r['tcp']['right']=pose(np.array(r['poses']['cap'][:3])+[0,0,.061])
    assert grade(trace)['score']==.5
    assert not grade(trace)['success']


def test_closed_nearby_hand_cannot_count_as_released_even_at_correct_height():
    trace=success_trace()
    for r in trace[14:]:
        r['grippers']['right']=0.
        r['tcp']['right']=pose(np.array(r['poses']['cap'][:3])+[0,0,.061])
    assert not grade(trace)['success']


@pytest.mark.parametrize('name', ['fixture','reference_fixture','reference_washer','reference_cap'])
def test_shifted_scene_invalidates_even_if_returned(name):
    trace=success_trace()
    trace[8]['poses'][name][0]+=.025
    assert not grade(trace)['valid']
    assert grade(trace)['score']==0.


def test_tipped_fixture_does_not_count_even_when_relative_stack_is_correct():
    trace=success_trace()
    angle=math.radians(15)
    q=(math.sin(angle/2),0,0,math.cos(angle/2))
    rot=np.array([[1,0,0],[0,math.cos(angle),-math.sin(angle)],[0,math.sin(angle),math.cos(angle)]])
    for r in trace[13:]:
        for name,local in [('fixture',(0,0,0)),('washer',C.WASHER_SEATED_LOCAL),('cap',(0,0,.022))]:
            r['poses'][name]=pose(np.array(C.FIXTURE_POS)+rot@local,q)
    assert not grade(trace)['success']


def test_fixture_suspended_above_table_does_not_count():
    trace=success_trace()
    for r in trace[13:]:
        for name in C.DYNAMIC_NAMES:
            r['poses'][name][2]+=.010
    assert not grade(trace)['success']


def test_sporadic_clear_frames_do_not_accumulate_stable_hold():
    trace=success_trace()
    dwell(trace,40)
    for i in range(15,len(trace),10):
        trace[i]['grippers']['left']=0.
    assert not grade(trace)['success']
    assert grade(trace)['stable_seconds']<1.5


def test_missing_samples_interrupt_continuous_hold():
    trace=success_trace()
    # No continuous final block reaches 1.5 s after a missing control sample.
    cut=30
    trace=trace[:cut]+trace[cut+1:]
    assert not grade(trace)['success']


def test_zero_reported_velocity_does_not_hide_real_motion():
    trace=success_trace()
    for i,r in enumerate(trace[13:]):
        r['poses']['washer'][0]+=.002 if i%2 else -.002
    assert not grade(trace)['success']
    assert grade(trace)['stable_seconds']==0.


def test_zero_angular_velocity_does_not_hide_pose_rotation():
    trace=success_trace()
    for i,r in enumerate(trace[13:]):
        a=math.radians(2 if i%2 else -2)
        r['poses']['cap'][3:]=[0,0,math.sin(a/2),math.cos(a/2)]
    assert not grade(trace)['success']


def test_final_height_is_relational_to_washer_not_nominal_world_height():
    trace=success_trace()
    for r in trace[13:]:
        r['poses']['washer'][2]+=.0025
        r['poses']['cap'][2]-=.0025
    assert not grade(trace)['success']


def test_quaternion_sign_is_not_motion():
    trace=success_trace()
    for i,r in enumerate(trace):
        if i%2:
            for p in r['poses'].values(): p[3:]=[-v for v in p[3:]]
    assert grade(trace)['success']


def test_reset_clears_completed_and_faulted_episodes():
    scorer=ReworkScorer()
    for r in success_trace(): m=scorer.update(r)
    assert m['success']
    scorer.reset()
    m=scorer.update(record())
    assert m['score']==0. and not m['cap_removed'] and not m['cap_parked']
    bad=record(); bad['poses']['cap'][0]=float('nan')
    assert not scorer.update(bad)['valid']
    scorer.reset()
    for r in success_trace(): m=scorer.update(r)
    assert m['success']


def test_regrade_ignores_spoofed_saved_grades_and_policy_flags(tmp_path):
    trace=[record()]
    dwell(trace)
    for r in trace:
        r.update(score=1.,success=True,cap_parked=True,washer_seated=True,phase='done')
    path=tmp_path/'trace.jsonl'
    path.write_text('\n'.join(json.dumps(r) for r in trace),encoding='utf-8')
    assert regrade_file(path)['terminal']['score']==0.


def test_empty_and_malformed_traces_fail_explicitly(tmp_path):
    with pytest.raises(ValueError,match='empty'):
        regrade_records([])
    path=tmp_path/'bad.jsonl'; path.write_text('{broken}',encoding='utf-8')
    with pytest.raises(ValueError,match='line 1'):
        regrade_file(path)


def test_reversed_time_and_unphysical_quaternion_are_rejected():
    trace=success_trace()
    trace[10]['t']=trace[9]['t']
    assert not grade(trace)['valid']
    trace=success_trace(); trace[10]['poses']['cap'][6]=0.
    assert not grade(trace)['valid']

@pytest.mark.parametrize('name', ['washer','cap'])
def test_square_aperture_must_enclose_actual_post_corners(name):
    trace=success_trace()
    a=math.pi/4
    # A 45-degree square aperture at 3.9 mm offset intersects a post corner,
    # despite meeting centre, height, tilt, speed, and hand-clear criteria.
    for r in trace[13:]:
        r['poses'][name][0]+=.0039/math.sqrt(2)
        r['poses'][name][1]+=.0039/math.sqrt(2)
        r['poses'][name][3:]=[0,0,math.sin(a/2),math.cos(a/2)]
    assert not grade(trace)['success']


@pytest.mark.parametrize('yaw_degrees, physically_fits', [(5., True), (45., False)])
def test_centred_washer_yaw_is_limited_by_real_aperture_geometry(yaw_degrees, physically_fits):
    trace=success_trace()
    a=math.radians(yaw_degrees)
    for r in trace[13:]:
        r['poses']['washer'][3:]=[0,0,math.sin(a/2),math.cos(a/2)]
    assert grade(trace)['success'] == physically_fits


def test_cap_hovering_above_table_is_not_parked():
    trace=[record()]
    advance(trace,cap=(.29,0,C.TABLE_TOP+.10))
    advance(trace,cap=(.15,.15,C.TABLE_TOP+.010))
    dwell(trace,10)
    assert not grade(trace)['cap_parked']


def test_final_relational_geometry_tracks_small_fixture_shift_and_yaw():
    trace=success_trace()
    angle=math.radians(20)
    rot=np.array([[math.cos(angle),-math.sin(angle),0],[math.sin(angle),math.cos(angle),0],[0,0,1]])
    q=(0,0,math.sin(angle/2),math.cos(angle/2))
    f=np.array(C.FIXTURE_POS)+[.01,.004,0]
    for r in trace[13:]:
        for name,local in [('fixture',(0,0,0)),('washer',C.WASHER_SEATED_LOCAL),('cap',(0,0,.022))]:
            r['poses'][name]=pose(f+rot@local,q)
    assert grade(trace)['success']


def test_cap_dropped_after_parking_permanently_loses_partial_credit():
    trace=park_trace()
    advance(trace,cap=(.17,-.12,C.TABLE_TOP-.030))
    advance(trace,cap=(.17,-.12,C.TABLE_TOP+.006))
    assert grade(trace)['score']==0.
    assert 'dropped' in grade(trace)['error']


def test_washer_dropped_after_seating_permanently_loses_partial_credit():
    trace=success_trace()
    advance(trace,washer=(.6,-.5,C.TABLE_TOP-.020))
    advance(trace,washer=np.array(C.FIXTURE_POS)+C.WASHER_SEATED_LOCAL)
    assert grade(trace)['score']==0.
    assert not grade(trace)['valid']


def test_inverted_cap_handle_below_table_fails_even_with_root_above_table():
    trace=park_trace()
    # Bottom-root is above tabletop; inverted handle reaches down 82 mm.
    advance(trace,cap=pose((.17,-.12,C.TABLE_TOP+.050),(1,0,0,0)))
    assert grade(trace)['score']==0.
    assert 'dropped' in grade(trace)['error']


def test_washer_center_above_drop_cutoff_does_not_hide_low_edge():
    trace=park_trace()
    # A vertical 68 mm washer extends 34 mm below its centre.
    a=math.pi/2
    advance(trace,washer=pose((.6,-.5,C.TABLE_TOP+.020),(math.sin(a/2),0,0,math.cos(a/2))))
    assert grade(trace)['score']==0.
    assert 'dropped' in grade(trace)['error']


def test_legitimate_lifted_parts_do_not_trigger_drop_guard():
    trace=park_trace()
    advance(trace,washer=(.3,-.2,C.TABLE_TOP+.12),cap=(.17,-.12,C.TABLE_TOP+.15))
    assert grade(trace)['valid']
    assert grade(trace)['score']==.25


def test_drop_boundary_has_numeric_tolerance_without_changing_success():
    trace=park_trace()
    advance(trace,cap=(.17,-.12,C.TABLE_TOP-.010))
    assert grade(trace)['valid']
    advance(trace,cap=(.17,-.12,C.TABLE_TOP-.0101))
    assert not grade(trace)['valid']


def test_drop_guard_local_boxes_match_actual_usd_collision_assets():
    from pathlib import Path
    from pxr import Usd, UsdGeom
    from missing_washer_rework.scoring import _collider_boxes
    for name in C.DYNAMIC_NAMES:
        path=Path(__file__).resolve().parents[1]/'missing_washer_rework'/'assets'/(name+'.usda')
        stage=Usd.Stage.Open(str(path))
        actual=[]
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Cube):
                actual.append((tuple(prim.GetAttribute('xformOp:scale').Get()),
                               tuple(prim.GetAttribute('xformOp:translate').Get())))
        expected=_collider_boxes(name)
        assert len(actual)==len(expected)
        for (size,centre),(expected_size,expected_centre) in zip(actual,expected):
            assert np.allclose(size,expected_size,atol=1e-9)
            assert np.allclose(centre,expected_centre,atol=1e-9)
