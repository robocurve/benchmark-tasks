import unittest
import numpy as np
from hands import pad_endpoints

def link_poses(travel,g=None):
    # Installed URDF origins/axes, including the 180-degree roll.
    g=np.eye(4) if g is None else g
    r=np.array([[0,1,0],[1,0,0],[0,0,-1.]])
    result={}
    for name,origin,axis,q in (
        ('tip_left',[.0445119,-.0239931,.0566829],[0,1,0],travel[0]),
        ('tip_right',[-.0445119,.0239931,.0566829],[0,-1,0],travel[1])):
        t=np.eye(4);t[:3,:3]=r;t[:3,3]=origin+r@np.array(axis)*q
        result[name]=g@t
    return result

class IndependentJawTests(unittest.TestCase):
    def test_contact_stalls_one_jaw_without_mirroring_it(self):
        g=np.eye(4);g[:3,3]=[.3,.2,.753]
        tips=pad_endpoints(link_poses([-.00405,-.04195],g))
        np.testing.assert_allclose(tips,[[.29595,.2,.9],[.34195,.2,.9]])

    def test_rotated_asymmetric_jaws(self):
        g=np.eye(4);g[:3,3]=[.153,.2,.9]
        g[:3,:3]=np.array([[0,0,1],[1,0,0],[0,1,0]])
        tips=pad_endpoints(link_poses([0,-.04695],g))
        np.testing.assert_allclose(tips,[[.3,.2,.9],[.3,.24695,.9]])

    def test_closed_landmark_is_the_installed_pad_tip(self):
        np.testing.assert_allclose(pad_endpoints(link_poses([0,0])),[[0,0,.147]]*2,atol=1e-12)

if __name__=='__main__':unittest.main()
