"""Real native end-state regression; no simulation or successful-roll claim.

The compact fixture is taken from the final measured13.9s state of native001.
PI bias is reconstructed by integrating the unchanged old Cartesian errors
from13.05..13.85s; the trace SHA identifies the preserved full raw capture.
Only the native-shaped tensor/world plumbing is replaced. IK, collision meshes,
all315-piece guards, scene poses and BOTH commanded arms remain real.
"""
import copy,json,unittest,builtins
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix
from turnover import CartonTurnoverProbe
from turnover_safety import BASES

FIXTURE=json.loads(r'''
{
  "trace_sha256": "5d2cccf5bcad96ff38611974989b27a867e523a7040e28294b969283dcca0752",
  "time": 13.9,
  "poses": {
    "north": [
      0.25000274181365967,
      -0.009969030506908894,
      0.8389725685119629,
      -8.447197615168989e-05,
      1.2133767086197622e-05,
      1.4566297068085987e-05,
      1.0
    ],
    "north_top": [
      0.25000426173210144,
      -0.006438909564167261,
      0.8839743137359619,
      -0.7111294269561768,
      -1.494561911385972e-05,
      5.408281140262261e-06,
      -0.703061044216156
    ],
    "north_bottom": [
      0.2500007450580597,
      -0.053492650389671326,
      0.7489961981773376,
      4.5581720769405365e-05,
      -1.940532001754036e-06,
      -2.5621391159802442e-06,
      1.0
    ],
    "east": [
      0.3600074052810669,
      -0.09996505081653595,
      0.8389850854873657,
      -6.905281043145806e-05,
      -5.100492126075551e-05,
      -0.7070877552032471,
      0.707125723361969
    ],
    "east_top": [
      0.3630084991455078,
      -0.09995777159929276,
      0.8809835910797119,
      -0.49996447563171387,
      0.5000224709510803,
      0.4999506175518036,
      -0.5000622272491455
    ],
    "east_bottom": [
      0.3165041208267212,
      -0.09998102486133575,
      0.7519999742507935,
      -0.0001381782494718209,
      3.309293242637068e-05,
      -0.7070876955986023,
      0.707125723361969
    ],
    "south": [
      0.2500135600566864,
      -0.1899622529745102,
      0.8390003442764282,
      1.6627545846858993e-05,
      9.913597750710323e-05,
      0.9999999403953552,
      -1.2701209016086068e-05
    ],
    "south_top": [
      0.2500157356262207,
      -0.19295212626457214,
      0.8839988708496094,
      -2.5559290861565387e-06,
      -0.7071659564971924,
      -0.7070475220680237,
      2.2089225240051746e-05
    ],
    "south_bottom": [
      0.25000831484794617,
      -0.1464790552854538,
      0.7489994168281555,
      3.8740196828257467e-07,
      6.293785645539174e-06,
      0.9999999403953552,
      -1.1552138857950922e-05
    ],
    "west": [
      0.14000828564167023,
      -0.09996728599071503,
      0.8389906883239746,
      -4.0332353819394484e-05,
      5.952884384896606e-05,
      0.707124650478363,
      0.707088828086853
    ],
    "west_top": [
      0.13671603798866272,
      -0.09996191412210464,
      0.8809899687767029,
      -0.5015801191329956,
      -0.5016834735870361,
      -0.49833789467811584,
      -0.4983876943588257
    ],
    "west_bottom": [
      0.18350371718406677,
      -0.0999787449836731,
      0.7519997358322144,
      -0.00012034632527502254,
      -2.3628204871783964e-05,
      0.7071208953857422,
      0.7070925831794739
    ]
  },
  "cutter_pose": [
    0.12870724499225616,
    0.27001461386680603,
    0.7655304670333862,
    -5.281004632706754e-06,
    -0.035644691437482834,
    -9.595148003427312e-05,
    0.99936443567276
  ],
  "native_hands": [
    {
      "side": "left",
      "joints": [
        -0.5347421169281006,
        1.8313263654708862,
        0.678758442401886,
        0.3023527264595032,
        0.8420670628547668,
        -0.8608315587043762
      ],
      "finger_joints": [
        -0.012118315324187279,
        -0.011884376406669617
      ],
      "tcp": [
        0.2501587609100844,
        0.009780455272123012,
        0.7786084509035119
      ],
      "tcp_rotation": [
        [
          -0.006940986587129368,
          -0.9999736514271282,
          -0.002125830824189398
        ],
        [
          -0.5003636892785555,
          0.005313699908661683,
          -0.8657990200068569
        ],
        [
          0.8657875034653424,
          -0.004945810831043777,
          -0.5003873877292202
        ]
      ],
      "opening": 0.25561970472335815
    },
    {
      "side": "right",
      "joints": [
        -0.21321047842502594,
        1.636372447013855,
        1.1275885105133057,
        -1.4117923974990845,
        -1.0040088891983032,
        -0.4074520766735077
      ],
      "finger_joints": [
        -0.011827231384813786,
        -0.012109766714274883
      ],
      "tcp": [
        0.2491329507653181,
        -0.21254021610016188,
        0.9049241327062255
      ],
      "tcp_rotation": [
        [
          0.00011672704590453543,
          -0.9999991102430329,
          -0.0013288671639335979
        ],
        [
          -0.5044152223852618,
          -0.0012063041188038295,
          0.8634603802470616
        ],
        [
          -0.8634612149951055,
          0.0005695116465741657,
          -0.5044149143866195
        ]
      ],
      "opening": 0.254920095205307
    }
  ],
  "previous_action": [
    -0.5336930751800537,
    1.8343019485473633,
    0.6802789568901062,
    0.3055018186569214,
    0.8414276242256165,
    -0.8685108423233032,
    0.2555910646915436,
    -0.20769652724266052,
    1.6352932453155518,
    1.1302908658981323,
    -1.4088237285614014,
    -1.011099100112915,
    -0.3988620638847351,
    0.2555910646915436
  ],
  "bias_before_rejected_command": [
    [
      0.0008051576612362759,
      -0.00010690578849038414,
      -0.00010246383761085554
    ],
    [
      0.0005169119427282021,
      0.0008825600510992843,
      0.0008061125891239084
    ]
  ],
  "last_control_angle_deg": 0.009181396669333708,
  "first_roll_started": 13.0
}
''')

class Tensor:
 def __init__(self,value):self.value=np.asarray(value)
 def detach(self):return self
 def cpu(self):return self
 def numpy(self):return self.value

def make_probe():
 f=copy.deepcopy(FIXTURE);hands=f['native_hands'];ready=np.r_[READY,1.,READY,1.]
 values={side:Tensor([[*hands[i]['joints'],*hands[i]['finger_joints']]]) for i,side in enumerate(('left','right'))}
 def pose(name):return Tensor([[*(f['cutter_pose'] if name=='cutter' else f['poses'][name])]])
 env=SimpleNamespace(_ready=Tensor([ready]),_low=np.r_[ik.LOWER,0.,ik.LOWER,0.],_high=np.r_[ik.UPPER,1.,ik.UPPER,1.],_ids={s+'_robot':(list(range(6)),[6,7]) for s in ('left','right')},rig=SimpleNamespace(bases=BASES),_base=SimpleNamespace(arena_world=SimpleNamespace(get_pose_w=pose),scene={s+'_robot':SimpleNamespace(data=SimpleNamespace(joint_pos=values[s])) for s in ('left','right')}))
 probe=CartonTurnoverProbe(env);measured=probe.measure();probe.change('first_roll',f['first_roll_started'],measured);probe.previous=np.array(f['previous_action']);probe.bias=[np.array(v) for v in f['bias_before_rejected_command']];probe.angle=probe.last_raw=f['last_control_angle_deg'];probe.best_angle=0.;probe.last_progress=f['first_roll_started']
 return probe

class Tests(unittest.TestCase):
 def test_real_native_rejected_advance_freezes_pi_and_keeps_guarded_candidate(self):
  probe=make_probe();before=[b.copy() for b in probe.bias];progress=probe.last_progress;previous=probe.previous.copy()
  action,reports=probe.command(FIXTURE['time']);report=reports[0]
  self.assertIsNone(report['failure']);self.assertLess(report['advance_fraction'],1.)
  self.assertTrue(report['planner_and_integrator_frozen']);self.assertEqual(probe.last_progress,progress)
  self.assertEqual(report['advance_trials'][0]['fraction'],1.);self.assertFalse(report['advance_trials'][0]['safe']);self.assertLess(report['advance_trials'][0]['clearance'][1]['target_gap_m'],-.002)
  for actual,old in zip(probe.bias,before):np.testing.assert_array_equal(actual,old)
  self.assertEqual(probe.stage,'first_roll');self.assertFalse(probe.done)
  self.assertEqual(len(probe.geometry.model.raw),315)
  self.assertTrue(np.isfinite(action).all());self.assertTrue(np.all(action>=probe.env._low));self.assertTrue(np.all(action<=probe.env._high))
  self.assertTrue(np.all(np.abs(action-previous)<=np.array([.025]*6+[.04]+[.025]*6+[.04])+1e-12))
  for g in report['previous_command_clearance']:self.assertTrue(g['safe']);self.assertGreaterEqual(g['target_gap_m'],-.002)
  for index,g in enumerate(report['clearance']):self.assertTrue(g['safe']);self.assertGreaterEqual(g['target_gap_m'],-.0006 if index%2==0 else -.002)
  # With no measured angular progress, rejection/holding never buys extra time.
  _,reports=probe.command(progress+5.01);self.assertIn('no measured angular progress',reports[0]['failure'])

 def test_rejected_transition_rolls_back_phase_clock_cache_and_integrator(self):
  probe=make_probe();probe.started=12.;probe.last_progress=12.
  probe.phase_goal_q=[np.arange(6,dtype=float),np.arange(6,dtype=float)+1.]
  before=probe._candidate_state();original=probe.geometry.guard;calls=[0];sums=[0]
  def trigger_overcentre(values):
   values=list(values);sums[0]+=1
   # State-machine event injection ONLY, not physical over-centre evidence:
   # first mass sum is unchanged; second weighted-COM sum triggers transition.
   return builtins.sum(values) if sums[0]==1 else .25*builtins.sum(p['mass'] for p in probe.parts.values())
  def reject_full_advance(*args,**kwargs):
   g=original(*args,**kwargs);calls[0]+=1
   # Fresh actual/previous guards remain real. Inject one inadmissible full
   # proposal; the half proposal still passes the unchanged real315 guard.
   return dict(g,safe=False) if 5<=calls[0]<=6 else g
  with patch('turnover.sum',side_effect=trigger_overcentre,create=True),patch.object(probe.geometry,'guard',side_effect=reject_full_advance):
   _,reports=probe.command(FIXTURE['time'])
  report=reports[0];self.assertIsNone(report['failure']);self.assertEqual(report['candidate_phase'],'clear_first')
  self.assertEqual(probe.stage,'first_roll');self.assertLess(report['advance_fraction'],1.);self.assertGreater(report['advance_fraction'],0.)
  self.assertEqual(probe.started,before['started']);self.assertEqual(probe.last_progress,before['last_progress'])
  for a,b in zip(probe.bias,before['bias']):np.testing.assert_array_equal(a,b)
  for a,b in zip(probe.phase_goal_q,before['phase_goal_q']):np.testing.assert_array_equal(a,b)
  for pair,old in zip(probe.starts,before['starts']):
   for a,b in zip(pair,old):np.testing.assert_array_equal(a,b)
  for a,b in zip(probe.start_q,before['start_q']):np.testing.assert_array_equal(a,b)

 def test_contact_requires_both_measured_mesh_proximity_and_five_settled_frames(self):
  probe=make_probe();measured=probe.measure();probe.change('contact',0.,measured)
  probe.phase_goal_q=[m[0].copy() for m in measured]
  probe.previous=np.r_[measured[0][0],.012/abs(FINGER_OPEN),measured[1][0],.012/abs(FINGER_OPEN)]
  for index in range(5):
   _,reports=probe.command(3.+.05*index)
   self.assertIsNone(reports[0]['failure']);self.assertEqual(probe.stage,'contact')
   self.assertFalse(reports[0]['geometric_opposed_settle']['contact_force_verified'])
  _,reports=probe.command(3.25);self.assertIsNone(reports[0]['failure']);self.assertEqual(probe.stage,'first_roll')

 def test_a_close_endpoint_without_both_wall_proximities_never_starts_roll(self):
  probe=make_probe();measured=probe.measure();probe.change('contact',0.,measured);probe.phase_goal_q=[m[0].copy() for m in measured]
  probe.previous=np.r_[measured[0][0],.012/abs(FINGER_OPEN),measured[1][0],.012/abs(FINGER_OPEN)]
  original=probe.geometry.guard
  def north_gap(*args,**kwargs):
   g=original(*args,**kwargs)
   return dict(g,target_gap_m=.001) if args[0]=='left' else g
  with patch.object(probe.geometry,'guard',side_effect=north_gap):
   for index in range(7):
    _,reports=probe.command(3.+.05*index);self.assertIsNone(reports[0]['failure']);self.assertEqual(probe.stage,'contact')
    self.assertFalse(reports[0]['geometric_opposed_settle']['both_mesh_gaps_in_range']);self.assertEqual(probe.contact_stable,0)

 def test_no_admissible_advance_holds_only_freshly_guarded_previous(self):
  probe=make_probe();old=probe.previous.copy();bias=[b.copy() for b in probe.bias];progress=probe.last_progress
  original=probe.geometry.guard;calls=[0]
  def only_previous(*args,**kwargs):
   g=original(*args,**kwargs);calls[0]+=1
   # Actual guards and the first fresh previous-command pair remain real.
   # Later candidate checks are deliberately made inadmissible to test hold.
   if calls[0]>4:g=dict(g,safe=False)
   return g
  with patch.object(probe.geometry,'guard',side_effect=only_previous):action,reports=probe.command(FIXTURE['time'])
  self.assertIsNone(reports[0]['failure']);self.assertEqual(reports[0]['advance_fraction'],0.)
  np.testing.assert_array_equal(action,old);self.assertEqual(probe.last_progress,progress)
  for a,b in zip(probe.bias,bias):np.testing.assert_array_equal(a,b)
  # If that old command has become unsafe, returning it is an abort signal;
  # failure prevents the runner from issuing any further simulation step.
  probe=make_probe();old=probe.previous.copy();original=probe.geometry.guard;calls=[0]
  def previous_invalid(*args,**kwargs):
   g=original(*args,**kwargs);calls[0]+=1
   if 3<=calls[0]<=4:g=dict(g,safe=False)
   return g
  with patch.object(probe.geometry,'guard',side_effect=previous_invalid):action,reports=probe.command(FIXTURE['time'])
  self.assertIn('previous issued',reports[0]['failure']);np.testing.assert_array_equal(action,old)

if __name__=='__main__':unittest.main()
