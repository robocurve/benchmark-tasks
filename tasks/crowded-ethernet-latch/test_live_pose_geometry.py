"""Measured settled/perturbed poses protect the real boot/clip approach.

Numeric native cases are anonymous. Certificates cover gripper meshes and
ordinary rigid connector geometry; full arm links/cables need native testing.
"""
import json
import unittest
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from collision_geometry import FingerGeometry,minimum_separation
from robot import R_GRIP,BOOT_LOCAL,GraspProbe
from config import *
from test_robot import environment,Array
POSE_CASES=json.loads('[{"time":2.0,"plug_pose":[0.4042681157588959,-0.024426443502306938,0.9503999948501587,-0.0016994017641991377,-0.073758065700531,0.020874440670013428,0.9970561861991882],"clip_pose":[0.403757244348526,-0.024433398619294167,0.9535950422286987,0.004804662894457579,-0.06130469962954521,0.020645972341299057,0.9978939890861511],"neighbours":{"1":[0.4038752317428589,-0.008240418508648872,0.950390636920929,0.0001698001433396712,-0.07772243767976761,-0.003818382043391466,0.996967613697052],"2":[0.4040031433105469,0.008190110325813293,0.9503301978111267,0.0015346853761002421,-0.07125305384397507,-0.02352415770292282,0.9971796274185181],"3":[0.40405941009521484,0.024228163063526154,0.950401782989502,-0.0002091237693093717,-0.07639695703983307,0.0008247665246017277,0.9970770478248596],"5":[0.404127836227417,-0.008304081857204437,0.9641244411468506,-0.00019412327674217522,-0.07578468322753906,-0.00022359889408107847,0.9971241354942322],"6":[0.4039019048213959,0.008044040761888027,0.9640668034553528,-0.0010689340997487307,-0.07391311228275299,0.012409249320626259,0.9971868395805359],"7":[0.4039648473262787,0.024578779935836792,0.9641461968421936,-0.0007302055601030588,-0.07945654541254044,0.007142945192754269,0.9968124628067017]}},{"time":16.1,"plug_pose":[0.40417277812957764,-0.023954646661877632,0.9507483839988708,-0.0007541413651779294,-0.10148442536592484,0.0010362797183915973,0.9948362708091736],"clip_pose":[0.4036276340484619,-0.02402825653553009,0.9534106850624084,0.013146975077688694,-0.07861629128456116,-0.0025510727427899837,0.9968149662017822],"neighbours":{"1":[0.4038528800010681,-0.008144835010170937,0.9502890110015869,0.001019415445625782,-0.0696316733956337,-0.016071362420916557,0.9974427223205566],"2":[0.40401434898376465,0.008190229535102844,0.9503173828125,0.0014991241041570902,-0.07009376585483551,-0.023530596867203712,0.997261643409729],"3":[0.404096394777298,0.02422572672367096,0.9503570199012756,-0.0001881840144051239,-0.07239292562007904,0.0007060684147290885,0.9973758459091187],"5":[0.40440633893013,-0.008359076455235481,0.9642571210861206,-0.0003754814970307052,-0.08257616311311722,0.00020186245092190802,0.9965845942497253],"6":[0.4039057195186615,0.008043911308050156,0.9640626311302185,-0.001066976459696889,-0.07352537661790848,0.01239556260406971,0.9972156882286072],"7":[0.40394049882888794,0.02459687925875187,0.9641445279121399,-0.0008319717017002404,-0.07963300496339798,0.00858449749648571,0.996786892414093]}}]')

def obstacles(row,ownclip=True):
 # These historical tilted fixtures predate guides and wire telemetry. Keep
 # their certificate explicitly scoped; current full builders default to both.
 from geometry import object_obstacles
 plugs={i:np.array(row['plug_pose'] if i==SERVICE else row['neighbours'][str(i)]) for i in USED}
 clips={i:np.r_[p[:3]+quat_xyzw_to_matrix(p[3:])@[0,0,CLIP_Z],p[3:]] for i,p in plugs.items()}
 clips[SERVICE]=np.array(row['clip_pose'])
 result=[]
 for owner,name,triangles in object_obstacles(plugs,clips,include_wires=False,include_guides=False):
  k=-1 if owner=='switch' else int(owner.split('_')[1])
  if not ownclip and owner==f'clip_{SERVICE}':continue
  result.append((k,'boot' if name=='boot_bound' else name,triangles))
 return result

class LivePoseGeometryTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.geometry=FingerGeometry()
 def test_pregrasp_uses_measured_boot_position_and_axis(self):
  for row in POSE_CASES:
   env=environment();env._base.arena_world.get_pose_w=lambda name:Array([row['clip_pose'] if name.startswith('clip') else row['plug_pose']])
   probe=GraspProbe(env,np.array(port(SERVICE)));probe.phase='narrow_in_front_of_boot'
   _,report=probe.command(1.)
   plug=np.array(row['plug_pose']);rotation=quat_xyzw_to_matrix(plug[3:])
   expected=plug[:3]+rotation@BOOT_LOCAL
   np.testing.assert_allclose(probe.boot_goal,expected)
   np.testing.assert_allclose(report['left_target'],expected+rotation@[-.040,0,0])
   np.testing.assert_allclose(probe.boot_rotation,R_GRIP)
   self.assertGreater(np.linalg.norm(expected-(np.array(port(SERVICE))+BOOT_LOCAL)),.004)

 def test_loaded_entry_and_closure_clear_actual_own_clip_and_neighbours(self):
  for row in POSE_CASES:
   plug=np.array(row['plug_pose']);rotation=quat_xyzw_to_matrix(plug[3:]);goal=plug[:3]+rotation@BOOT_LOCAL
   world=[a for k,n,a in obstacles(row) if not(k==SERVICE and n=='boot')]
   sweep=[np.concatenate([a@R_GRIP.T+goal+rotation@[dx,0,0] for dx in (-.040,0.)])
          for _,a in self.geometry.pieces([-.0065,-.0065])]
   self.assertGreater(minimum_separation(sweep,world),.001)
   for jaw in (-.0035,-.0038,-.0045,-.005,-.0065):
    parts=[a@R_GRIP.T+goal for _,a in self.geometry.pieces([jaw,jaw])]
    self.assertGreater(minimum_separation(parts,world),.001)
   old_goal=plug[:3]+rotation@[-.0265,.0005,-.0005]
   old=[a@R_GRIP.T+old_goal for _,a in self.geometry.pieces([-.0045,-.0045])]
   own_tab=[a for k,n,a in obstacles(row) if k==SERVICE and n=='tab']
   self.assertEqual(minimum_separation(old,own_tab),0.)

 def test_actual_press_stroke_has_environment_and_mutual_certificates(self):
  from pathlib import Path
  spec=json.loads(Path(__file__).with_name('press_point.json').read_text());point=np.array(spec['point_tcp']);press=np.array(spec['rotation'])
  for row in POSE_CASES:
   plug=np.array(row['plug_pose']);rotation=quat_xyzw_to_matrix(plug[3:]);right_rotation=rotation@press
   sweep=[np.concatenate([(a-point)@right_rotation.T+plug[:3]+rotation@[-.010,0,z] for z in (.0022,.0086)])
          for _,a in self.geometry.pieces([-.028,-.028])]
   world=[a for k,n,a in obstacles(row) if not(k==SERVICE and n in ('tab','hook'))]
   self.assertGreater(minimum_separation(sweep,world),.0002)
   for jaw in (-.0035,-.0038,-.0045,-.005):
    left=[a@R_GRIP.T+plug[:3]+rotation@BOOT_LOCAL for _,a in self.geometry.pieces([jaw,jaw])]
    self.assertGreater(minimum_separation(left,sweep),.002)
