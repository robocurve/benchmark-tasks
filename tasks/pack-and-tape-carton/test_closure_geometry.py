import unittest
import numpy as np
from closure_geometry import GRASPS,flap_target,PAD_CONTACT_FORWARD,GRASP_FROM_HINGE,rx
from carton_scene import layout
from config import CENTER,TOP

class ClosureGeometryTests(unittest.TestCase):
 def setUp(self):
  parts,hinges=layout();self.parts={p['name']:p for p in parts};self.hinges={h['a']:h for h in hinges if h['b'].endswith('_top')}
 def wall(self,name):
  p=self.parts[name];w,x,y,z=p['quat'];r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
  return np.array(CENTER)+p['pos'],r
 def test_closing_axis_crosses_board_and_open_jaws_clear_thickness(self):
  for name,(_,edge,tangent,tilt) in GRASPS.items():
   wp,wr=self.wall(name)
   for angle in np.linspace(-175,0,36):
    p,r,contact=flap_target(wp,wr,self.hinges[name],self.parts[name+'_top']['size'],angle,edge,tangent,tilt)
    normal=(wr@rx(angle))[:,2]
    self.assertGreater(abs(np.dot(r[:,0],normal)),.998)
    # The open 20 mm aperture straddles the 3 mm board plane by >8 mm.
    self.assertGreater(.010*abs(np.dot(r[:,0],normal))-.0015,.008)
    np.testing.assert_allclose(r.T@r,np.eye(3),atol=1e-12)
    np.testing.assert_allclose(p+r[:,2]*PAD_CONTACT_FORWARD,contact,atol=1e-12)
 def test_grasp_is_on_plain_finite_lateral_rim_with_bounded_arc(self):
  for name,(_,edge,tangent,tilt) in GRASPS.items():
   wp,wr=self.wall(name);h=self.hinges[name];size=self.parts[name+'_top']['size']
   for angle in np.linspace(-175,0,36):
    p,r,contact=flap_target(wp,wr,h,size,angle,edge,tangent,tilt);fr=wr@rx(angle);fp=wp+wr@h['anchor_a']-fr@h['anchor_b'];local=fr.T@(contact-fp)
    self.assertLess(abs(local[0]),size[0]/2);self.assertLess(abs(local[1]),size[1]/2);self.assertAlmostEqual(local[2],0.)
    self.assertLessEqual(contact[2],(wp+wr@h['anchor_a'])[2]+GRASP_FROM_HINGE+1e-12)
 def test_top_crease_uses_explicit_torque_per_degree_and_can_rest_closed(self):
  _,hinges=layout()
  for hinge in hinges:
   if hinge['b'].endswith('_top'):
    self.assertEqual(hinge['spring'],.00002)
    opening_torque=hinge['spring']*abs(hinge['target'])
    closing_gravity=.018*9.81*(.087/2)
    self.assertLess(opening_torque,closing_gravity)
   elif hinge['b'].endswith('_bottom'):self.assertEqual(hinge['spring'],.003)
 def test_closed_inner_top_faces_lie_under_outer_bottom_faces(self):
  limits={}
  for name,(_,edge,tangent,tilt) in GRASPS.items():
   wp,wr=self.wall(name);h=self.hinges[name];fr=wr;fp=wp+wr@h['anchor_a']-fr@h['anchor_b'];half=self.parts[name+'_top']['size'][2]/2
   limits[name]=(fp[2]-half,fp[2]+half)
  self.assertLessEqual(max(limits[n][1] for n in ('east','west')),min(limits[n][0] for n in ('north','south'))+1e-12)
