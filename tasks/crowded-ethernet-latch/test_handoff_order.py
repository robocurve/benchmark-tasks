"""Frozen certificate-order parity and CPU timing regressions."""
import numpy as np
from yam_arena import ik

def original_handoff(self,measured,next_target,next_rotation,next_phase,stage,position,rotation,plug,bodyR,point,exclude):
    """Use measured geometry and progress, rather than all-joint precision."""
    if len(self.functional_errors)<5:return False
    witness=measured[1][2]+measured[1][3]@point
    local=bodyR.T@(witness-plug[:3]);next_witness=next_target+next_rotation@point
    down=float((witness-next_witness)@bodyR[:,2])
    qnext,pe,re=ik.solve(next_target,ik.matrix_to_quat_xyzw(next_rotation),self.press_joint_servo.reference,
                         base_pos=self.env.rig.bases['right'],iters=220,tol=1e-6)
    dq3=float(qnext[2]-measured[1][0][2]);error=max(self.functional_errors)
    report={'stage':stage,'max_mesh_error_5_frames_m':error,'next_witness_down_m':down,'next_q3_delta_rad':dq3,
            'wire_scope':'Measured gaps only; error reserve applies to non-target rigid geometry and other hand', 'accepted':False}
    self.functional_handoff_report=report
    if not np.isfinite([down,dq3,pe,re]).all() or pe>.0001 or re>.001 or down<=0. or dq3>=0.:return False
    if stage=='PREPRESS':
        if self.hover_departure is None:return False
        departed=self.hover_departure
        progress=float(departed['witness_local_z_m']-local[2]);qprogress=float(departed['q3_rad']-measured[1][0][2])
        report.update({'measured_down_progress_m':progress,'required_down_progress_m':.5*departed['planned_down_m'],
                       'measured_negative_q3_progress_rad':qprogress,'required_negative_q3_progress_rad':-.5*departed['planned_q3_delta_rad'],
                       'last_substantial_q3_movement_rad':self.prepress_last_q3_movement})
        if (progress<.5*departed['planned_down_m'] or qprogress<-.5*departed['planned_q3_delta_rad']
                or self.prepress_last_q3_movement is None or self.prepress_last_q3_movement>=0.):return False
    actual_safe=self.certify_press_path(measured,next_target,next_rotation,next_phase,exclude)
    actual=dict(self.press_approach_certificate);report['actual_path']=actual
    if not actual_safe:return False
    nominal=[measured[0],(measured[1][0],measured[1][1],position,rotation)]
    nominal_safe=self.certify_press_path(nominal,next_target,next_rotation,'nominal_'+next_phase,exclude)
    planned=dict(self.press_approach_certificate);self.press_approach_certificate=actual;report['nominal_path']=planned
    reserve=min(planned['rigid_gap_m'],planned['other_hand_gap_m'])-planned['rotation_chord_bound_m']-error
    report['non_target_rigid_and_hand_reserve_m']=reserve
    accepted=bool(nominal_safe and reserve>=.0002);report['accepted']=accepted
    if accepted and stage=='HOVER':
        self.hover_departure={'witness_local_z_m':float(local[2]),'q3_rad':float(measured[1][0][2]),
                              'planned_down_m':down,'planned_q3_delta_rad':dq3}
        self.prepress_last_q3_movement=None
    return accepted


import json
from pathlib import Path
import time
import unittest
from unittest.mock import patch
from yam_arena.urdf import quat_xyzw_to_matrix
from robot import PressTrackingProbe
from test_robot import environment
import test_functional_handoff as fixtures
from geometry import object_obstacles
from config import SERVICE,port

TRACE=Path('/private/tmp/ethernet-tracking028.jsonl')


def recorded_fixture(age):
    rows=[json.loads(line) for line in TRACE.read_text().splitlines()]
    rows=[r for r in rows if r['phase']=='prepress_endpoint' and r['robot_control'].get('functional_handoff',{}).get('stage')=='PREPRESS']
    row=rows[-1] if age is None else min(rows,key=lambda r:abs(r['robot_control']['phase_age_s']-age))
    c=row['robot_control'];report=c['functional_handoff']
    probe=PressTrackingProbe(environment(),np.array(port(SERVICE)))
    # Order parity replays028's original target, independent of later aim changes.
    probe.local_contact=np.array((-.010,-.0015,.0045))
    measured=[]
    for arm in c['arms']:
        q=np.array(arm['actual_joints']);p,r=ik.tcp_pose(q,probe.env.rig.bases[arm['side']])
        measured.append((q,np.array(arm['actual_finger_joints_m']),p,r))
    plug=np.array(row['plug_poses'][str(SERVICE)]);body=quat_xyzw_to_matrix(plug[3:])
    rotation=body@probe.pressR;point=probe.point.copy();point[0]+=measured[1][1][0]+.028
    target=plug[:3]+body@(probe.local_contact-[0,0,.0023])-rotation@point
    probe.press_joint_servo.latch(c['press_joint_reference'])
    position,nominal_rotation=ik.tcp_pose(probe.press_joint_servo.reference,probe.env.rig.bases['right'])
    probe.functional_errors=[report['max_mesh_error_5_frames_m']]*5
    local=body.T@(measured[1][2]+measured[1][3]@point-plug[:3])
    probe.hover_departure={'witness_local_z_m':local[2]+report['measured_down_progress_m'],
                          'q3_rad':measured[1][0][2]+report['measured_negative_q3_progress_rad'],
                          'planned_down_m':2*report['required_down_progress_m'],
                          'planned_q3_delta_rad':-2*report['required_negative_q3_progress_rad']}
    probe.prepress_last_q3_movement=report['last_substantial_q3_movement_rad']
    poses={**{'plug_'+i:np.array(p) for i,p in row['plug_poses'].items()},
           **{'clip_'+i:np.array(p) for i,p in row['clip_poses'].items()},
           **{name:np.array(p) for name,p in row['wire_body_poses'].items()}}
    probe.object_pose=lambda name:poses[name]
    plugs={int(i):np.array(p) for i,p in row['plug_poses'].items()}
    clips={int(i):np.array(p) for i,p in row['clip_poses'].items()}
    probe.current_press_obstacles=object_obstacles(plugs,clips,include_wires=False)
    probe.current_press_left=[a@measured[0][3].T+measured[0][2] for _,a in probe.geometry.pieces(measured[0][1])]
    exclude=((f'clip_{SERVICE}','tab'),(f'clip_{SERVICE}','hook'))
    args=(measured,target,rotation,'depress_clip','PREPRESS',position,nominal_rotation,plug,body,point,exclude)
    return probe,args,row['robot_control']['phase_age_s']


class HandoffOrderTests(unittest.TestCase):
    def fixture(self,name='safe020'):
        return fixtures.FunctionalHandoffTests().fixture(name)

    def test_early_reserve_reject_skips_actual_and_cable_certificate(self):
        probe,case,args,certificate=self.fixture()
        case['nominal_certificate']['rigid_gap_m']=.00035
        probe.press_approach_certificate={'stale':True}
        with patch.object(probe,'certify_press_path',side_effect=AssertionError('must skip cable work')):
            self.assertFalse(probe.functional_press_certificate(*args))
        report=probe.functional_handoff_report
        self.assertIsNone(report['actual_path']);self.assertIsNone(probe.press_approach_certificate)
        self.assertNotIn('wire_gap_m',report['nominal_path'])
        self.assertIn('wire_not_evaluated',report['nominal_path']['wire_scope'])

    def test_accepted_and_actual_unsafe_paths_call_actual_wire_certificate_once(self):
        for unsafe in (False,True):
            probe,case,args,certificate=self.fixture()
            if unsafe:case['actual_certificate']['rigid_gap_m']=.0001
            with patch.object(probe,'certify_press_path',side_effect=certificate) as actual:
                self.assertEqual(probe.functional_press_certificate(*args),not unsafe)
            actual.assert_called_once();self.assertIs(actual.call_args.args[0],args[0])
            self.assertEqual(actual.call_args.args[-1],args[-1])
            self.assertEqual(probe.functional_handoff_report['actual_path']['wire_gap_m'],case['actual_certificate']['wire_gap_m'])

    def test_nonfinite_nominal_reserve_rejects(self):
        for key in ('rigid_gap_m','other_hand_gap_m','rotation_chord_bound_m'):
            probe,case,args,certificate=self.fixture();case['nominal_certificate'][key]=np.nan
            with patch.object(probe,'certify_press_path',side_effect=AssertionError('must skip')):
                self.assertFalse(probe.functional_press_certificate(*args))

    def test_nominal_and_actual_use_identical_clip_exclusions(self):
        probe,case,args,certificate=self.fixture();args=list(args)
        args[-1]=((f'clip_{SERVICE}','tab'),(f'clip_{SERVICE}','hook'))
        with patch.object(probe,'rigid_motion_certificate',wraps=probe.rigid_motion_certificate) as nominal,patch.object(probe,'certify_press_path',side_effect=certificate) as actual:
            self.assertTrue(probe.functional_press_certificate(*args))
        self.assertEqual(nominal.call_args.kwargs['exclude'],args[-1])
        self.assertEqual(actual.call_args.args[-1],args[-1])

    def test_recorded_fixture_decisions_and_reserves_match_frozen_original(self):
        for name in ('safe020','partial019'):
            old,case,args,certificate=self.fixture(name)
            with patch.object(old,'certify_press_path',side_effect=certificate):expected=original_handoff(old,*args)
            new,case,args,certificate=self.fixture(name)
            with patch.object(new,'certify_press_path',side_effect=certificate):actual=new.functional_press_certificate(*args)
            self.assertEqual(actual,expected)
            if 'non_target_rigid_and_hand_reserve_m' in old.functional_handoff_report:
                self.assertEqual(new.functional_handoff_report['non_target_rigid_and_hand_reserve_m'],old.functional_handoff_report['non_target_rigid_and_hand_reserve_m'])

    @unittest.skipUnless(TRACE.exists(),'local028 trace')
    def test_native028_rejected_and_accepted_decisions_match(self):
        for age,expected in ((6.85,False),(None,True)):
            old,args,actual_age=recorded_fixture(age);before=original_handoff(old,*args)
            new,args,_=recorded_fixture(age);after=new.functional_press_certificate(*args)
            self.assertEqual(before,expected);self.assertEqual(after,before)
            self.assertAlmostEqual(new.functional_handoff_report['non_target_rigid_and_hand_reserve_m'],old.functional_handoff_report['non_target_rigid_and_hand_reserve_m'],delta=2e-15)


def benchmark(repeats=3):
    results=[]
    for age in (6.85,None):
        result={}
        for label,function in (('original',original_handoff),('early_nominal',lambda p,*a:p.functional_press_certificate(*a))):
            durations=[]
            for _ in range(repeats):
                probe,args,actual_age=recorded_fixture(age)
                started=time.perf_counter();accepted=function(probe,*args);durations.append(time.perf_counter()-started)
            result[label]={'accepted':accepted,'reserve_m':probe.functional_handoff_report['non_target_rigid_and_hand_reserve_m'],'median_s':float(np.median(durations)),'samples_s':durations}
        result['phase_age_s']=actual_age
        result['speedup']=result['original']['median_s']/result['early_nominal']['median_s']
        results.append(result)
    print(json.dumps(results,indent=2))


if __name__=='__main__':unittest.main()
