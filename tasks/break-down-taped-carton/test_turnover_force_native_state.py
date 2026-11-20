"""Actual force002 end states; synthetic controller phase/clock binding only.

Poses, joint readbacks, last-issued commands and native normal-force vectors
are copied exactly from the14.60/14.65s raw trace. Sensor-clock values were NOT
recorded by that old diagnostic; ControlledReader supplies explicitly synthetic
clocks for CPU contract testing. Initialising the NEW acquisition state on
these old states is a CPU hypothesis, not an attained native acquisition.
No collision geometry, normal-force values or safety bounds are mocked.
"""
import json,unittest
from types import SimpleNamespace
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY,FINGER_OPEN
from yam_arena.urdf import quat_xyzw_to_matrix
from turnover import CartonTurnoverProbe
from turnover_safety import BASES
from scene import layout
from test_turnover_transactional import Tensor
from test_turnover_force_acquisition import ControlledReader

FIXTURES=json.loads(r'''
[
  {
    "trace_sha256": "4ddf7973d0da93af669591108eeba3421b62b7b8d276507696bb1e1c92229440",
    "time": 14.600000000000001,
    "poses": {
      "north": [
        0.2500024139881134,
        -0.009965707547962666,
        0.8389714956283569,
        -8.923823043005541e-05,
        1.230377074534772e-05,
        1.2448424968170002e-05,
        1.0
      ],
      "north_top": [
        0.2500038146972656,
        -0.006177496165037155,
        0.8839767575263977,
        -0.7131355404853821,
        -1.2552670341392513e-05,
        6.19591537542874e-06,
        -0.7010260224342346
      ],
      "north_bottom": [
        0.2500007450580597,
        -0.05349193140864372,
        0.7489960789680481,
        4.708489723270759e-05,
        -2.0369764115457656e-06,
        -3.312792387077934e-06,
        0.9999999403953552
      ],
      "east": [
        0.3600059449672699,
        -0.09996240586042404,
        0.8389853835105896,
        -7.289885979844257e-05,
        -5.6344761105719954e-05,
        -0.7070929408073425,
        0.7071205377578735
      ],
      "east_top": [
        0.36300694942474365,
        -0.09995457530021667,
        0.880983829498291,
        -0.4999578297138214,
        0.5000300407409668,
        0.49995049834251404,
        -0.500061571598053
      ],
      "east_bottom": [
        0.31650328636169434,
        -0.09997954219579697,
        0.7520005702972412,
        -0.00014740852930117399,
        3.2579693652223796e-05,
        -0.707091212272644,
        0.7071222066879272
      ],
      "south": [
        0.25001081824302673,
        -0.18995881080627441,
        0.8390011191368103,
        1.4871598978061229e-05,
        0.00010731503425631672,
        0.9999999403953552,
        -1.0623131856846157e-05
      ],
      "south_top": [
        0.2500128149986267,
        -0.19294792413711548,
        0.8839994668960571,
        -2.6198931664112024e-06,
        -0.707170307636261,
        -0.7070431709289551,
        1.9303604858578183e-05
      ],
      "south_bottom": [
        0.25000622868537903,
        -0.14647690951824188,
        0.7489994764328003,
        3.6810428127864725e-07,
        5.383834377425956e-06,
        0.9999999403953552,
        -8.77111597219482e-06
      ],
      "west": [
        0.14000695943832397,
        -0.09996360540390015,
        0.8389903903007507,
        -4.480249481275678e-05,
        6.154662696644664e-05,
        0.7071195840835571,
        0.7070939540863037
      ],
      "west_top": [
        0.13662169873714447,
        -0.09995783865451813,
        0.8809903264045715,
        -0.5020942687988281,
        -0.5021955966949463,
        -0.49781492352485657,
        -0.4978766143321991
      ],
      "west_bottom": [
        0.18350286781787872,
        -0.09997681528329849,
        0.7520000338554382,
        -0.00012877148401457816,
        -2.3587803298141807e-05,
        0.707116425037384,
        0.7070971131324768
      ]
    },
    "cutter_pose": [
      0.12870724499225616,
      0.27001461386680603,
      0.7655304670333862,
      -5.368280653783586e-06,
      -0.03564466908574104,
      -9.597195457899943e-05,
      0.99936443567276
    ],
    "native_hands": [
      {
        "side": "left",
        "joints": [
          -0.5347520112991333,
          1.8313547372817993,
          0.6782280802726746,
          0.3041422367095947,
          0.8409562706947327,
          -0.8610793352127075
        ],
        "finger_joints": [
          -0.012118126265704632,
          -0.011885927058756351
        ],
        "tcp": [
          0.2503586378391193,
          0.009803760708095577,
          0.7785668000205788
        ],
        "tcp_rotation": [
          [
            -0.007616019652013147,
            -0.9999707113816619,
            -0.0007567189130242738
          ],
          [
            -0.5004485565923003,
            0.0044666898600026015,
            -0.8657547521592809
          ],
          [
            0.8657327754274661,
            -0.006214906318499734,
            -0.5004679175432816
          ]
        ],
        "opening": 0.25563421845436096
      },
      {
        "side": "right",
        "joints": [
          -0.21338772773742676,
          1.6363106966018677,
          1.1275032758712769,
          -1.411311388015747,
          -1.0047985315322876,
          -0.4066149592399597
        ],
        "finger_joints": [
          -0.011830849573016167,
          -0.012109974399209023
        ],
        "tcp": [
          0.2492542636138409,
          -0.21253358189640853,
          0.9049998109824218
        ],
        "tcp_rotation": [
          [
            -0.0004242876917624101,
            -0.9999997066977926,
            -0.0006376396185476889
          ],
          [
            -0.5038732452002642,
            -0.00033699166403054076,
            0.8637775403470346
          ],
          [
            -0.8637775018784114,
            0.000687779722655888,
            -0.503872954431714
          ]
        ],
        "opening": 0.254960834980011
      }
    ],
    "previous_action": [
      -0.5340871810913086,
      1.8339769840240479,
      0.6793332695960999,
      0.30645331740379333,
      0.8411670327186584,
      -0.8689544200897217,
      0.2555910646915436,
      -0.20754538476467133,
      1.6350524425506592,
      1.1298131942749023,
      -1.4083596467971802,
      -1.0111498832702637,
      -0.3985939919948578,
      0.2555910646915436
    ],
    "normal_force_vectors": {
      "left": [
        0.0,
        0.0,
        0.0
      ],
      "right": [
        -0.002564250025898218,
        -0.9089330434799194,
        0.0005235934513621032
      ]
    },
    "blocked_right_proposal": [
      -0.5340841895564548,
      1.8340058359925542,
      0.6793134882152798,
      0.30649942395899893,
      0.8411690148736485,
      -0.8689510751184129,
      0.25559105431309903,
      -0.2073604537495968,
      1.6349626932482049,
      1.1298860450264696,
      -1.4082440752812524,
      -1.0112120552423816,
      -0.3982658275844635,
      0.25559105431309903
    ]
  },
  {
    "trace_sha256": "4ddf7973d0da93af669591108eeba3421b62b7b8d276507696bb1e1c92229440",
    "time": 14.65,
    "poses": {
      "north": [
        0.250002384185791,
        -0.00996586587280035,
        0.8389716148376465,
        -8.843644900480285e-05,
        1.215926658915123e-05,
        1.2545013305498287e-05,
        1.0
      ],
      "north_top": [
        0.2500038146972656,
        -0.006156132090836763,
        0.8839772939682007,
        -0.7133044004440308,
        -1.2711283488897607e-05,
        6.122885224613128e-06,
        -0.7008542418479919
      ],
      "north_bottom": [
        0.2500007450580597,
        -0.05349194258451462,
        0.7489960789680481,
        4.705564788309857e-05,
        -2.030370751526789e-06,
        -3.268730552008492e-06,
        0.9999999403953552
      ],
      "east": [
        0.3600059449672699,
        -0.09996253997087479,
        0.8389853835105896,
        -7.265751628438011e-05,
        -5.6061679060803726e-05,
        -0.7070929408073425,
        0.7071205377578735
      ],
      "east_top": [
        0.36300694942474365,
        -0.09995470196008682,
        0.8809838891029358,
        -0.4999583065509796,
        0.5000295639038086,
        0.49995091557502747,
        -0.5000610947608948
      ],
      "east_bottom": [
        0.31650328636169434,
        -0.09997957944869995,
        0.7520005106925964,
        -0.00014607043704017997,
        3.256827767472714e-05,
        -0.7070912718772888,
        0.7071221470832825
      ],
      "south": [
        0.25001078844070435,
        -0.18995898962020874,
        0.8390010595321655,
        1.473588781664148e-05,
        0.00010624349670251831,
        0.9999999403953552,
        -1.064852131094085e-05
      ],
      "south_top": [
        0.25001275539398193,
        -0.19294820725917816,
        0.8839994072914124,
        -2.5245392407668987e-06,
        -0.7071695923805237,
        -0.7070438861846924,
        1.9114060705760494e-05
      ],
      "south_bottom": [
        0.25000622868537903,
        -0.14647692441940308,
        0.7489994764328003,
        3.494214695365372e-07,
        5.432596026366809e-06,
        0.9999999403953552,
        -8.65093261381844e-06
      ],
      "west": [
        0.14000694453716278,
        -0.09996377676725388,
        0.838990330696106,
        -4.4396278099156916e-05,
        6.110587855800986e-05,
        0.7071196436882019,
        0.7070938944816589
      ],
      "west_top": [
        0.13661348819732666,
        -0.09995803236961365,
        0.8809903860092163,
        -0.5021398067474365,
        -0.5022404789924622,
        -0.49776968359947205,
        -0.49783065915107727
      ],
      "west_bottom": [
        0.18350286781787872,
        -0.09997684508562088,
        0.7519999742507935,
        -0.0001273324160138145,
        -2.3737467927276157e-05,
        0.7071164846420288,
        0.707097053527832
      ]
    },
    "cutter_pose": [
      0.12870724499225616,
      0.27001461386680603,
      0.7655304670333862,
      -5.363613126974087e-06,
      -0.035644665360450745,
      -9.59636308834888e-05,
      0.99936443567276
    ],
    "native_hands": [
      {
        "side": "left",
        "joints": [
          -0.5347520112991333,
          1.8314746618270874,
          0.6782473921775818,
          0.3040395975112915,
          0.8409722447395325,
          -0.8610106706619263
        ],
        "finger_joints": [
          -0.012118136510252953,
          -0.011885936371982098
        ],
        "tcp": [
          0.2503410516139035,
          0.009812077684960119,
          0.7785195204112588
        ],
        "tcp_rotation": [
          [
            -0.007533280469839574,
            -0.9999712576832225,
            -0.0008564419403381893
          ],
          [
            -0.500528907928499,
            0.004512183031606869,
            -0.8657080642642724
          ],
          [
            0.8656870462316432,
            -0.006092947704002923,
            -0.5005485131083793
          ]
        ],
        "opening": 0.2556343972682953
      },
      {
        "side": "right",
        "joints": [
          -0.21338772773742676,
          1.6363106966018677,
          1.1275032758712769,
          -1.4112656116485596,
          -1.0048121213912964,
          -0.40653878450393677
        ],
        "finger_joints": [
          -0.01183107029646635,
          -0.012109990231692791
        ],
        "tcp": [
          0.24926210988362213,
          -0.21253401734028357,
          0.90499905483108
        ],
        "tcp_rotation": [
          [
            -0.0004618262568986561,
            -0.9999997074805054,
            -0.0006097174852653484
          ],
          [
            -0.5038708607519953,
            -0.0002939604197949325,
            0.8637789469953034
          ],
          [
            -0.8637788735559304,
            0.0007061346719948292,
            -0.5038705776013054
          ]
        ],
        "opening": 0.25496336817741394
      }
    ],
    "previous_action": [
      -0.5340871810913086,
      1.8339769840240479,
      0.6793332695960999,
      0.30645331740379333,
      0.8411670327186584,
      -0.8689544200897217,
      0.2555910646915436,
      -0.20754538476467133,
      1.6350524425506592,
      1.1298131942749023,
      -1.4083596467971802,
      -1.0111498832702637,
      -0.3985939919948578,
      0.2555910646915436
    ],
    "normal_force_vectors": {
      "left": [
        0.0,
        0.0,
        0.0
      ],
      "right": [
        -0.002582188928499818,
        -0.9015560746192932,
        0.0005171449156478047
      ]
    },
    "blocked_right_proposal": [
      -0.5340838401909147,
      1.8339980096281887,
      0.6793219538275185,
      0.3064828312112145,
      0.8411692470576481,
      -0.8689506754830695,
      0.25559105431309903,
      -0.20736166115468982,
      1.6349615950260679,
      1.1298841332347547,
      -1.4082450688519497,
      -1.0112116491742165,
      -0.39826796161533917,
      0.25559105431309903
    ]
  }
]
''')

def make_probe(f):
 ready=np.r_[READY,1.,READY,1.];hands=f['native_hands'];values={s:Tensor([[*hands[i]['joints'],*hands[i]['finger_joints']]]) for i,s in enumerate(('left','right'))}
 world=SimpleNamespace(get_pose_w=lambda n:Tensor([[*(f['cutter_pose'] if n=='cutter' else f['poses'][n])]]))
 env=SimpleNamespace(_ready=Tensor([ready]),_low=np.r_[ik.LOWER,0.,ik.LOWER,0.],_high=np.r_[ik.UPPER,1.,ik.UPPER,1.],_ids={s+'_robot':(list(range(6)),[6,7]) for s in ('left','right')},rig=SimpleNamespace(bases=BASES),_base=SimpleNamespace(arena_world=world,scene={s+'_robot':SimpleNamespace(data=SimpleNamespace(joint_pos=values[s])) for s in ('left','right')}))
 env._turnover_contact_reader=ControlledReader(f['normal_force_vectors']['left'],f['normal_force_vectors']['right'])
 probe=CartonTurnoverProbe(env);measured=probe.measure();probe.change('contact',10.,measured);probe.phase_goal_q=[m[0].copy() for m in measured];probe.previous=np.array(f['previous_action']);probe.contact_acquire_started=13.;return probe

class Tests(unittest.TestCase):
 def test_loaded_right_held_while_left_advances_with_real_whole_pair_guard(self):
  f=FIXTURES[0];p=make_probe(f);old=p.previous.copy();old_bias=[b.copy() for b in p.bias]
  action,reports=p.command(f['time']);r=reports[0]
  self.assertIsNone(r['failure']);self.assertEqual(r['accepted_branch'],'left');self.assertEqual(r['arm_advance_fractions'],[1.,0.])
  self.assertEqual(p.stage,'contact');self.assertEqual(p.started,10.);self.assertEqual(p.contact_stable,0)
  self.assertEqual(r['integrator_committed_arms'],['left']);np.testing.assert_array_equal(p.bias[1],old_bias[1]);np.testing.assert_array_equal(action[7:],old[7:])
  self.assertGreater(np.max(np.abs(action[:6]-old[:6])),1e-6)
  self.assertTrue(np.all(np.abs(action-old)<=np.array([.025]*6+[.04]+[.025]*6+[.04])+1e-12))
  self.assertTrue(np.all(action>=p.env._low));self.assertTrue(np.all(action<=p.env._high))
  self.assertEqual(r['contact_force_evidence']['signed_normal_force_N'][0],0.);self.assertGreater(r['contact_force_evidence']['signed_normal_force_N'][1],.1)
  self.assertFalse(r['contact_force_evidence']['both_opposed_normal_forces'])
  for g in r['previous_command_clearance']:self.assertTrue(g['safe']);self.assertGreaterEqual(g['target_gap_m'],-.002)
  for i,g in enumerate(r['clearance']):self.assertTrue(g['safe']);self.assertGreaterEqual(g['target_gap_m'],-.0006 if i%2==0 else -.002)
  # The recorded, rejected RIGHT proposal stays inadmissible even with this
  # accepted LEFT command; a whole-vector shrink must not starve the brace.
  candidate=action.copy();candidate[7:]=f['blocked_right_proposal'][7:]
  poses=p.poses();obs={n:(pos,R,p.parts[n]['size']) for n,(pos,R) in poses.items()};tool=f['cutter_pose'];tr=quat_xyzw_to_matrix(tool[3:])
  for n,size,off in [('handle',(.1,.025,.025),(0,0,0)),('guard',(.02,.03,.02),(.052,0,.002)),('blade',(.014,.0005,.008),(.062,0,-.016))]:obs['cutter_'+n]=(np.array(tool[:3])+tr@off,tr,size)
  other=p.geometry.parts('left',candidate[:6],[FINGER_OPEN*candidate[6]]*2)
  g=p.geometry.guard('right',candidate[7:13],[FINGER_OPEN*candidate[13]]*2,obs,other,intended=True,intended_targets=('south','south_top'))
  self.assertLess(g['target_gap_m'],-.002)

 def test_freshly_unsafe_previous_right_command_still_aborts(self):
  f=FIXTURES[1];p=make_probe(f);old=p.previous.copy();action,reports=p.command(f['time'])
  self.assertIn('previous issued',reports[0]['failure']);np.testing.assert_array_equal(action,old)
  self.assertEqual(p.stage,'contact');self.assertFalse(p.done)

if __name__=='__main__':unittest.main()
