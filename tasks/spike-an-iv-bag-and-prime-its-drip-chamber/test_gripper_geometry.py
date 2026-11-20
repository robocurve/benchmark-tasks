import numpy as np
from gripper_geometry import segment_hits_planes


def cube():
    return np.array([[1,0,0,-1],[-1,0,0,-1],[0,1,0,-1],[0,-1,0,-1],[0,0,1,-1],[0,0,-1,-1]],float)


def test_crossing_segment_hits_even_with_both_endpoints_outside():
    assert segment_hits_planes(np.array([-2.,0,0]),np.array([2.,0,0]),0,cube())


def test_parallel_clear_segment_misses():
    assert not segment_hits_planes(np.array([-2.,1.2,0]),np.array([2.,1.2,0]),.1,cube())


def test_capsule_radius_detects_surface_contact():
    assert segment_hits_planes(np.array([-2.,1.05,0]),np.array([2.,1.05,0]),.1,cube())


def test_point_inside_and_point_outside():
    assert segment_hits_planes(np.zeros(3),np.zeros(3),0,cube())
    assert not segment_hits_planes(np.array([2.,0,0]),np.array([2.,0,0]),0,cube())
