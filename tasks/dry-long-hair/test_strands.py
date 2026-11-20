import math,unittest
from config import *
class RestShapeTests(unittest.TestCase):
 def test_mass_and_water_conservation(self):
  self.assertAlmostEqual(DRY_MASS_PER_SEGMENT*LOCKS*SEGMENTS,.032)
  self.assertAlmostEqual(WATER_PER_PATCH*LOCKS*SEGMENTS*4,.0064)
 def test_lengths_anchors_and_scalp_clearance(self):
  for k in range(LOCKS):
   ps=points(k)
   for j,(a,b) in enumerate(zip(ps,ps[1:])):
    self.assertAlmostEqual(math.dist(a,b),LENGTH)
    center=segment(k,j);q=segment_quat(k,j)
    for sign,want in [(1,a),(-1,b)]:
     local=rotate(q,(0,0,sign*LENGTH/2));actual=tuple(x+y for x,y in zip(center,local));self.assertLess(math.dist(want,actual),1e-10)
    for i in range(21):
     p=tuple(x+(y-x)*i/20 for x,y in zip(a,b));self.assertGreater(math.dist(p,HEAD),HEAD_RADIUS+RADIUS-.0005)
 def test_inner_and_outer_lengths_have_occluded_layers(self):
  inner=math.dist(points(0)[-1][:2],HEAD[:2]);outer=math.dist(points(1)[-1][:2],HEAD[:2]);self.assertLess(inner+.010,outer)

class OppositeHeadFrameTests(unittest.TestCase):
 def test_every_segment_frame_and_position_rotates_with_head(self):
  self.assertEqual(rotate(HEAD_QUAT,(-1.,0.,0.)),(1.,0.,0.))
  for k in range(LOCKS):
   old=points_local(k);world=points(k)
   for local,p in zip(old,world):
    self.assertAlmostEqual(p[0]-HEAD[0],-local[0]);self.assertAlmostEqual(p[1]-HEAD[1],-local[1]);self.assertAlmostEqual(p[2]-HEAD[2],local[2])
   self.assertEqual(root_local(k),old[0])
   for j in range(SEGMENTS):
    # Transport local capsule cross-sections as well as their centreline.
    a,b=old[j:j+2];axis=tuple((x-y)/LENGTH for x,y in zip(a,b))
    q=(-axis[1],axis[0],0.,1+axis[2]);n=math.sqrt(sum(v*v for v in q));expected=multiply(HEAD_QUAT,tuple(v/n for v in q))
    for v,w in zip(segment_quat(k,j),expected):self.assertAlmostEqual(v,w)
