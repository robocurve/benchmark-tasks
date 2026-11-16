import unittest
from bond_geometry import width_contacts
I=[[1,0,0],[0,1,0],[0,0,1]]
# Film adhesive local +Y points down; width +Z goes world +Y.
R=[[1,0,0],[0,0,1],[0,-1,0]]
class FootprintTests(unittest.TestCase):
 def test_face_bond_width_not_just_center_distance(self):
  film=dict(pos=(0,0,.001525),rotation=R)
  target=dict(pos=(0,0,0),rotation=I,size=(.010,.048,.003))
  self.assertEqual(len(width_contacts(film,target)),12)
  target['size']=(.010,.008,.003)
  self.assertEqual(len(width_contacts(film,target)),2)
 def test_wrong_side_and_no_contact_rejected(self):
  target=dict(pos=(0,0,0),rotation=I,size=(.01,.048,.003))
  self.assertEqual(width_contacts(dict(pos=(0,0,.003),rotation=R),target),[])
  self.assertEqual(width_contacts(dict(pos=(0,0,.001525),rotation=I),target),[])
