"""Material-model checks for physical work and scoring-independent outcomes."""

from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from cutting import CuttingModel
from geometry import wood_mesh


def material():
    return CuttingModel(np.zeros(2),np.array([.18,.14,.03]),0.)


def feed(model,trigger=.005,qualified=True,x=0.):
    for z in np.linspace(.002,-.012,2400):
        force,torque=model.step([x,0,z],[0,0,-1],trigger,1/240,qualified=qualified)
    return force,torque


def test_trigger_without_contact_does_not_make_a_hole():
    m=material()
    for _ in range(1200):
        force,_=m.step([0,0,.01],[0,0,-1],.005,1/240,qualified=True)
    assert m.rpm>800 and m.depth==0 and m.entry is None
    assert np.array_equal(force,np.zeros(3))


def test_downward_motion_without_trigger_is_resisted_and_cannot_cut():
    m=material()
    force,_=feed(m,trigger=0.)
    assert m.depth==0 and m.entry is None and force[2]>20.


def test_light_touch_with_spinning_bit_cannot_make_a_hole():
    m=material()
    for _ in range(2400):
        m.step([0,0,-.0001],[0,0,-1],.005,1/240,qualified=True)
    assert m.depth==0


def test_stationary_bit_cannot_accumulate_deep_hole_by_waiting():
    m=material()
    for _ in range(2400):
        m.step([0,0,-.0009],[0,0,-1],.005,1/240,qualified=True)
    assert 0<m.depth<.0009


def test_loaded_feed_creates_persistent_depth_and_reaction_wrenches():
    m=material()
    force,torque=feed(m)
    assert .010<m.depth<.012 and m.qualified_depth==m.depth
    assert force[2]>=8. and torque[2]>0.
    depth=m.depth
    for _ in range(120):
        m.step([0,0,.03],[0,0,-1],0.,1/240,qualified=True)
    assert m.depth==depth and m.rpm<50.


def test_material_cuts_at_actual_entry_independent_of_target_and_arm_identity():
    m=material()
    feed(m,x=.020,qualified=False)
    assert m.depth>.010 and m.entry[0]==.020 and m.qualified_depth==0.


def test_material_reset_has_no_preexisting_hole_or_spin():
    m=material()
    feed(m)
    fresh=material()
    assert fresh.depth==0 and fresh.entry is None and fresh.rpm==0.


def test_lateral_wall_response_does_not_launch_a_light_free_tool():
    m=material()
    m.entry=np.zeros(3)
    m.bore_axis=np.array([0.,0.,-1.])
    m.depth=.015
    m.engaged=True
    tip=np.array([.001,0.,-.005])
    velocity=np.zeros(3)
    # The bit's effective lateral mass can be far below the tool's total
    # mass because the body rotates about its grasp. Exercise that case.
    for _ in range(600):
        force,_=m.step(tip,[0,0,-1],0.,1/240)
        velocity[0]+=force[0]/.005/240
        tip[0]+=velocity[0]/240
        assert abs(tip[0])<.002


def test_sidewall_contact_does_not_replace_bore_floor_with_uncut_surface():
    m=material()
    m.entry=np.zeros(3)
    m.bore_axis=np.array([0.,0.,-1.])
    m.depth=.015
    m.engaged=True
    for x in (.0036,.0038,.004):
        force,_=m.step([x,0,-.010],[0,0,-1],0.,1/240)
        assert abs(force[2])<.001
        assert force[0]<0.


def test_reentry_away_from_existing_bore_hits_intact_top_surface():
    m=material()
    feed(m)
    m.step([0,0,.010],[0,0,-1],0.,1/240)
    force,_=m.step([.025,0,-.001],[0,0,-1],0.,1/240)
    assert force[2]>8. and not m.engaged


def test_visible_bore_geometry_has_finite_bounds_and_floor_at_depth():
    for centre in ([0.,0.,.015],[.020,-.01,.015]):
        points,faces,colours=wood_mesh([.18,.14,.03],centre,[0,0,-1],.011)
        points=np.asarray(points)
        assert np.all(np.isfinite(points))
        assert np.allclose(points.min(axis=0),[-.09,-.07,-.015])
        assert np.allclose(points.max(axis=0),[.09,.07,.015])
        assert any(np.allclose(points[f,2],.004) for f in faces)
        assert len(faces)==len(colours)
