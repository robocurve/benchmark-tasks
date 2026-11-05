import math
import numpy as np
from rotary_box.scoring import History,PITCH,error

CENTERS=[(.2,.12,.8),(.2,0,.8),(.2,-.12,.8)]

def hold(h,angles,clear=True,positions=CENTERS,speeds=None,steps=20):
    for _ in range(steps):h.update(angles,positions,[0.]*3,[0.]*3 if speeds is None else speeds,[clear]*3)

def test_partial_and_all_wheels_retained():
    h=History((3,1,7),CENTERS)
    hold(h,[3*PITCH,0,8*PITCH]);assert np.isclose(h.wheel_score(),.2)
    hold(h,[3*PITCH,PITCH,7*PITCH]);assert np.isclose(h.wheel_score(),.6)

def test_wrap_and_digit_tolerance():
    assert abs(error(-3*PITCH,7))<1e-10
    h=History((3,1,7),CENTERS);hold(h,[3*PITCH+math.radians(6),PITCH,7*PITCH]);assert np.isclose(h.wheel_score(),.4)

def test_held_or_spinning_wheels_do_not_verify_release():
    h=History((3,1,7),CENTERS);hold(h,[3*PITCH,PITCH,7*PITCH],clear=False);assert h.wheel_score()==0
    hold(h,[3*PITCH,PITCH,7*PITCH],speeds=[.2,.2,.2]);assert h.wheel_score()==0

def test_settings_lost_later_lose_credit():
    h=History((3,1,7),CENTERS);hold(h,[3*PITCH,PITCH,7*PITCH]);assert np.isclose(h.wheel_score(),.6)
    hold(h,[0,0,0]);assert h.wheel_score()==0

def test_lifting_or_tilting_wheels_cannot_fake_a_setting():
    h=History((3,1,7),CENTERS);positions=np.array(CENTERS);positions[0,2]+=.02
    hold(h,[3*PITCH,PITCH,7*PITCH],positions=positions);assert np.isclose(h.wheel_score(),.4)
    hold(h,[3*PITCH,PITCH,7*PITCH]);assert np.isclose(h.wheel_score(),.4)

def test_unlock_and_opening_threshold():
    h=History((3,1,7),CENTERS);hold(h,[3*PITCH,PITCH,7*PITCH]);h.update_lid(math.radians(59),0,0,True);assert np.isclose(h.score(),.6)
    h.update_lid(math.radians(61),0,0,True);assert np.isclose(h.score(),1.)

def test_forced_opening_or_moving_hinge_never_earns_opening():
    for position,tilt,unlock in ((.02,0,True),(0,math.radians(8),True),(0,0,False)):
        h=History((3,1,7),CENTERS);h.update_lid(math.radians(70),position,tilt,unlock);h.update_lid(math.radians(70),0,0,True);assert h.score()==0
