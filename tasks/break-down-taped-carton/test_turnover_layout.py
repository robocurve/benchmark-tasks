"""Catch a parked cutter being covered by the planned inverted carton."""
import unittest,math
import numpy as np
from scene import layout
from config import CENTER,TOP,CUTTER_START,CUTTER_PAD_START
from model import WIDTH,HEIGHT
from check_turnover import parked_cutter
from yam_arena.urdf import quat_xyzw_to_matrix
class Tests(unittest.TestCase):
 def test_parked_props_clear_full_inverted_carton_footprint(self):
  r=np.diag([1.,-1.,-1.]);corners=np.array([[x,y,z]for x in(-1,1)for y in(-1,1)for z in(-1,1)])
  raw=[]
  for p in layout()[0]:
   q=p['quat'];Q=quat_xyzw_to_matrix([*q[1:],q[0]]);raw.extend(np.asarray(p['pos'])+(corners*np.array(p['size'])/2)@Q.T)
  v=np.array(raw)@r.T
  # Same table-supported rolling prediction as the reviewed reach screen.
  y=CENTER[1]+WIDTH/2+.003+HEIGHT+(WIDTH/2+.0045)
  o=np.array([CENTER[0],y,TOP+.0002-v[:,2].min()]);v+=o
  for name,(p,Q,size)in parked_cutter().items():
   prop=np.asarray(p)+(corners*np.array(size)/2)@Q.T
   separation=np.maximum(prop.min(0)-v.max(0),v.min(0)-prop.max(0))
   self.assertGreaterEqual(float(separation.max()),.01,name+' lies in landing footprint')
  self.assertEqual(CUTTER_PAD_START[:2],CUTTER_START[:2])
if __name__=='__main__':unittest.main()
