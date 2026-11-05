import numpy as np
from answer_sheet.scoring import Ink

BOXES=[[(.0,.0),(.04,0),(.08,0),(.12,0)] for _ in range(4)]

def stroke(h,a,b,t=0):
    for i,p in enumerate(np.linspace(a,b,100)):
        h.contact(p,t+i/240)

def test_one_row_partial_and_full():
    boxes=[[(r*.04,c*.04) for c in range(4)] for r in range(4)]
    h=Ink(boxes,[0,2,3,1])
    for r,c in enumerate(h.answers):
        x,y=boxes[r][c]
        stroke(h,(x-.006,y+.005),(x+.002,y+.001),r*2)
        stroke(h,(x+.002,y+.001),(x-.006,y-.007),r*2+.42)
        assert h.score()==(r+1)/4

def test_dot_and_jitter_are_not_a_stroke():
    h=Ink([BOXES[0]],[0])
    for i in range(5000):h.contact((.0001*np.sin(i),.0001*np.cos(i)),i/240)
    assert h.score()==0

def test_hover_gap_does_not_connect_two_contacts():
    h=Ink([BOXES[0]],[0]);h.contact((-.008,0),0);h.contact((.008,0),1)
    assert h.score()==0 and not h.segments

def test_wrong_box_invalidates_correct_row_and_reset_is_clean():
    h=Ink([BOXES[0]],[0]);stroke(h,(-.008,0),(.008,0));assert h.score()==1
    stroke(h,(.039,0),(.041,0),1);assert h.score()==0
    assert Ink([BOXES[0]],[0]).score()==0

def test_stroke_outside_boxes_and_instant_teleport_have_no_credit():
    h=Ink([BOXES[0]],[0]);stroke(h,(-.1,0),(-.08,0));assert h.score()==0
    h.contact((-.008,0),1);h.contact((.008,0),1+1/240);assert h.score()==0

def test_wrong_answer_dot_invalidates_row_but_correct_dot_never_scores():
    h=Ink([BOXES[0]],[0]);h.contact((0,0),0);assert h.mark_present(0,0) and h.score()==0
    stroke(h,(-.008,0),(.008,0),1);assert h.score()==1
    h.contact((.04,0),2);assert h.score()==0

def test_disconnected_dashes_do_not_form_one_long_stroke():
    h=Ink([BOXES[0]],[0])
    for i,x in enumerate([-.008,0,.008]):stroke(h,(x-.0015,0),(x+.0015,0),i)
    assert h.score()==0
