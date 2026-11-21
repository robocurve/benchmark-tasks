import numpy as np
from pen_capping.scoring import History,Insertion
TOP=.7475
D={'red':(.25,.20,.24,.20),'blue':(.25,-.20,.24,.20)}
def update(h,pairs,clear=True,positions=None,speeds=None,steps=20):
 n=len(h.colors)
 for _ in range(steps):h.update(pairs,positions or [(.25,.20,TOP+.015)]*n,[(1,0,0)]*n,speeds or [0.]*n,[clear]*n,TOP,D)
def test_true_insertion_requires_approach_and_seating():
 a=Insertion()
 for _ in range(8):assert not a.update((-.0821,0,0),1.)
 a.update((-.14,0,0),1.)
 for _ in range(3):assert not a.update((-.0821,0,0),1.)
 assert a.update((-.0821,0,0),1.)
def test_shortcuts_bad_alignment_and_side_collision():
 for rel,align in [((-.0821,.004,0),1.),((-.0821,0,0),.9),((-.10,0,0),1.)]:
  a=Insertion();a.update((-.14,0,0),1.)
  for _ in range(30):assert not a.update(rel,align)
def test_success_partial_wrong_color_uncapped():
 h=History(('red','red','blue','blue'));update(h,['red',None,'red','blue']);assert np.isclose(h.score(),.375)
 h=History(('red','blue'));update(h,['red','blue'],positions=[(.25,.2,TOP+.015),(.25,-.2,TOP+.015)]);assert h.score()==.5
 h=History(('red',));update(h,['blue']);assert h.score()==0
 h=History(('red',));update(h,[None]);assert h.score()==0
def test_held_or_moving_assembly_has_no_placement():
 h=History(('red',));update(h,['red'],clear=False);assert h.score()==0
 update(h,['red'],speeds=[.1]);assert h.score()==.125
 update(h,['red']);assert h.score()==.25
def test_release_requires_full_second_and_wrong_area_loses_placement():
 h=History(('red',));update(h,['red'],steps=19);assert h.score()==0
 update(h,['red'],steps=1);assert h.score()==.25
 update(h,['red'],positions=[(.25,-.2,TOP+.015)]);assert h.score()==.125


def test_holding_the_tail_is_not_release():
 from pen_capping.scoring import hands_clear
 assert not hands_clear([(.072,0,.01)],(0,0,0),(1,0,0))
 assert hands_clear([(.072,0,.15)],(0,0,0),(1,0,0))


def test_overhanging_cap_does_not_earn_placement():
 h=History(('red',));update(h,['red'],positions=[(.16,.20,TOP+.015)]);assert h.score()==.125
