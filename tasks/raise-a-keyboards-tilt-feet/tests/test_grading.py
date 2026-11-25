"""Endpoint integrity: clocks, real supports, contact exclusions and hinge history."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation
from rdb_keyboard_feet import layout as L
from rdb_keyboard_feet.grading import KeyboardGrade


def supported_pose():
    def candidate(t):
        R=L.ry(-t)
        z=-(L.FRONT_PAD_CORNERS@R.T)[:,:,2].min()
        p=np.zeros((4,7));p[0]=np.r_[[.30,0,z],Rotation.from_matrix(R).as_quat()]
        for i,pivot in enumerate(L.PIVOTS):
            p[i+1]=np.r_[p[0,:3]+R@pivot,Rotation.from_matrix(R@L.ry(L.OPEN_ANGLE)).as_quat()]
        p[3]=L.START_POSES[3]
        return p
    def height(t):
        p=candidate(t)
        return (L.LEG_CORNERS@L.quat_matrix(p[1,3:]).T+p[1,:3])[:,2].min()
    return candidate(brentq(height,.04,.24))


def grade(g,p,**kw):
    return g.update(p,np.zeros((4,6)),robot_counts=kw.get('robot',np.zeros((18,4))),
                    support_loads_n=kw.get('loads',np.ones(4)),
                    notebook_contacts=kw.get('book',np.zeros((3,1))))


def opened(g):
    p=L.START_POSES.copy()
    for i in (1,2):p[i,3:]=Rotation.from_matrix(L.START_ROT@L.ry(L.OPEN_ANGLE)).as_quat()
    grade(g,p)


def test_one_second_between_endpoints_not_from_first_observation():
    g=KeyboardGrade();opened(g);p=supported_pose()
    for _ in range(240):r=grade(g,p)
    assert r['score']==.6 and r['stable_seconds']==239/240
    assert grade(g,p)['score']==1.


def test_one_broken_interval_resets_the_entire_clock():
    g=KeyboardGrade();opened(g);p=supported_pose()
    for _ in range(240):grade(g,p)
    grade(g,p,robot=np.ones((18,4)))
    assert grade(g,p)['stable_seconds']==0


def test_one_foot_folds_back_after_turnover():
    g=KeyboardGrade();opened(g);p=supported_pose()
    p[2,3:]=p[0,3:]
    for _ in range(260):r=grade(g,p)
    assert r['score']==.6 and r['feet_opened']==[True, True]


def test_resting_keyboard_needs_no_force_measurement():
    g=KeyboardGrade();opened(g);p=supported_pose()
    for _ in range(241):r=grade(g,p,loads=[0.,0.,0.,0.])
    assert r['grounded'] and r['score']==1.


def test_notebook_cannot_replace_any_desk_support():
    g=KeyboardGrade();opened(g);p=supported_pose()
    for _ in range(260):r=grade(g,p,book=[[1],[0],[0]])
    assert not r['notebook_clear'] and r['score']==.6


def test_final_open_pose_requires_the_inverted_opening_history():
    g=KeyboardGrade();p=supported_pose()
    for _ in range(260):r=grade(g,p)
    assert r['score']==0. and not any(r['feet_opened'])


def test_real_footprint_must_fit_on_desk():
    g=KeyboardGrade();opened(g);p=supported_pose();p[:3,1]+=.4
    for _ in range(260):r=grade(g,p)
    assert not r['contained'] and r['score']==.6


def test_untouched_inverted_start_earns_no_progress():
    g=KeyboardGrade()
    for _ in range(260):r=grade(g,L.START_POSES)
    assert r['inverted'] and r['score']==0. and not any(r['feet_opened'])

def test_one_actual_open_foot_earns_partial_progress():
    g=KeyboardGrade();p=L.START_POSES.copy()
    p[1,3:]=Rotation.from_matrix(L.START_ROT@L.ry(L.OPEN_ANGLE)).as_quat()
    r=grade(g,p)
    assert r['score']==.3 and r['feet_opened']==[True,False]

def test_nonfinite_robot_contacts_cannot_count_as_released():
    g=KeyboardGrade();opened(g);p=supported_pose();bad=np.zeros((18,4));bad[7,0]=np.nan
    assert grade(g,p,robot=bad)['score']==0. and g.void

def test_nonfinite_notebook_contacts_cannot_count_as_clear():
    g=KeyboardGrade();opened(g);p=supported_pose();bad=np.zeros((3,1));bad[0,0]=np.inf
    assert grade(g,p,book=bad)['score']==0. and g.void

def test_negative_contact_counts_are_invalid():
    g=KeyboardGrade();opened(g);p=supported_pose();bad=np.zeros((18,4));bad[8,0]=-1
    assert grade(g,p,robot=bad)['score']==0. and g.void

def test_nonunit_quaternion_cannot_be_silently_normalized():
    g=KeyboardGrade();opened(g);p=supported_pose();p[0,3:]*=2.
    assert grade(g,p)['score']==0. and g.void

def test_zero_quaternion_is_invalid():
    g=KeyboardGrade();opened(g);p=supported_pose();p[1,3:]=0.
    assert grade(g,p)['score']==0. and g.void

def test_footprint_includes_the_actual_key_top_height():
    g=KeyboardGrade();opened(g);p=supported_pose();R=L.quat_matrix(p[0,3:])
    x=L.TABLE_BOUNDS[0]+L.DEPTH/2*R[0,0]+.015*abs(R[0,2])
    p[:3,0]+=x-p[0,0]
    assert not grade(g,p)['contained']

def test_ground_reaction_pushes_the_deployed_foot_into_its_open_stop():
    # Positive world X behind the hinge makes an upward desk load produce
    # negative Y torque, toward the native lower stop rather than folding.
    for pitch in np.deg2rad([-8.,0.,15.,24.]):
        toe=L.LEG_CORNERS@(L.ry(pitch)@L.ry(L.OPEN_ANGLE)).T
        contacts=toe[np.isclose(toe[:,2],toe[:,2].min(),atol=1e-9)]
        assert contacts[:,0].min()>0.
    # A front-up pitch beyond the available over-centre angle reverses it.
    toe=L.LEG_CORNERS@(L.ry(np.deg2rad(35.))@L.ry(L.OPEN_ANGLE)).T
    contacts=toe[np.isclose(toe[:,2],toe[:,2].min(),atol=1e-9)]
    assert contacts[:,0].max()<0.


def test_one_rear_corner_clear_of_desk_blocks_full_score():
    g=KeyboardGrade();opened(g);p=supported_pose()
    p[2,2]+=.002
    for _ in range(241):r=grade(g,p)
    assert r['feet_opened']==[True,True] and not r['grounded'] and r['score']==.6


def test_sliding_keyboard_does_not_finish_the_hold():
    g=KeyboardGrade();opened(g);p=supported_pose()
    for _ in range(245):
        p[:3,0]+=.00002
        r=grade(g,p)
    assert r['score']==.6 and r['stable_seconds']<1
