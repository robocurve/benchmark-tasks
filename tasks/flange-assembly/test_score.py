"""Scoring must check holder, matching holes, and distinct seated pins."""
from types import SimpleNamespace
import numpy as np
import task


def state(pos,lo,hi,q):
    return SimpleNamespace(pos=np.array(pos,dtype=float),low=np.array(lo,dtype=float),high=np.array(hi,dtype=float),
                           quat_xyzw=np.array(q,dtype=float),resting=lambda:True)


def assembly():
    x,y=task.HOLDER; t=task.TOP
    a=state((x,y,t+.0505),(x-.075,y-.075,t+.025),(x+.075,y+.075,t+.076),(1,0,0,0))
    b=state((x,y,t+.1015),(x-.075,y-.075,t+.076),(x+.075,y+.075,t+.127),(0,0,0,1))
    out={'lower_flange':a,'upper_flange':b}
    for i,(dx,dy) in enumerate(((.05499,0),(-.05499,0),(0,.05499),(0,-.05499)),1):
        out[f'pin_{i}']=state((x+dx,y+dy,t+.09),(x+dx-.005,y+dy-.005,t+.061),(x+dx+.005,y+dy+.005,t+.121),(0,0,0,1))
    return out


def test_all_pins(): assert abs(task.score(assembly())-1.) < 1e-8

def test_missing_pin():
    d=assembly(); d['pin_1'].pos[0]+=.04
    assert abs(task.score(d)-.85)<1e-8

def test_misaligned_bolt_circle():
    d=assembly(); d['upper_flange'].quat_xyzw=np.array([0,0,.13,.9915])
    assert task.score(d)==.2

def test_duplicate_hole_not_credited_twice():
    d=assembly(); d['pin_2'].pos=d['pin_1'].pos.copy()
    assert abs(task.score(d)-.85)<1e-8


def test_lower_flange_outside_holder():
    d=assembly(); d['lower_flange'].pos[0]+=.02
    assert task.score(d)==0.


def test_upper_flange_not_stacked():
    d=assembly(); d['upper_flange'].low[2]+=.02
    assert task.score(d)==.2


def test_pin_above_hole_is_not_seated():
    d=assembly(); d['pin_1'].low[2]+=.05; d['pin_1'].high[2]+=.05
    assert abs(task.score(d)-.85)<1e-8


def test_pin_on_table_below_hole_is_not_seated():
    d=assembly(); d['pin_1'].low[2]-=.06; d['pin_1'].high[2]-=.06
    assert abs(task.score(d)-.85)<1e-8


def test_tilted_pin_is_not_seated():
    d=assembly(); d['pin_1'].quat_xyzw=np.array([.15,0,0,np.sqrt(1-.15**2)])
    assert abs(task.score(d)-.85)<1e-8


def test_moving_pin_is_not_seated():
    d=assembly(); d['pin_1'].resting=lambda:False
    assert abs(task.score(d)-.85)<1e-8


def test_moving_stack_is_not_aligned():
    d=assembly(); d['upper_flange'].resting=lambda:False
    assert task.score(d)==.2
