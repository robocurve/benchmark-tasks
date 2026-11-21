import numpy as np
from rdb_planter.web_center import center

def test_asymmetric_opening_from_recorded_cradle():
    cords=np.array([[[.065,.016]],[[-.073,.015]],[[.056,-.015]],[[.070,-.021]]])
    c=center(cords,np.array([-.016,-.005]))
    assert c is not None
    angles=np.sort(np.arctan2((cords[:,0]-c)[:,1],(cords[:,0]-c)[:,0]))
    assert np.diff(np.r_[angles,angles[0]+2*np.pi]).max()<np.pi

def test_tangled_section_has_no_measured_opening():
    assert center([[],[],[],[]],np.zeros(2)) is None
