"""Protocol, release, low-corner, temporal, seed and display adversaries."""
import math
from pathlib import Path
from dataclasses import replace
from contextlib import redirect_stdout
from io import StringIO
import numpy as np
import pytest
from yam_arena.task_file import load
from yam_arena import ik
from yam_parts import PartState

MODULE=next(p for p in Path(__file__).parent.glob('*.py') if not p.name.startswith('test_') and
            p.stem not in ('colorimeter_model','colorimeter_visuals','yam_parts','yam_joints'))
T=load(MODULE).module


@pytest.fixture(autouse=True)
def isolate():
    saved=dict(T._RUN)
    T._RUN.update(env=None,t=None,step=0,episode=T._RUN['episode']+1)
    T.TEST_LAYOUT,T.TEST_INSTRUMENT,T.TEST_GRIPPERS=None,None,None
    T.reset_history()
    yield
    T.TEST_LAYOUT,T.TEST_INSTRUMENT,T.TEST_GRIPPERS=None,None,None
    T._RUN.clear();T._RUN.update(saved);T.reset_history()


def part(spec,origin=None,yaw=0.,tilt=0.,velocity=0.):
    o=np.array(spec.pos if origin is None else origin,float)
    a,b=math.radians(yaw),math.radians(tilt)
    R=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1.]])@np.array([[1,0,0],[0,math.cos(b),-math.sin(b)],[0,math.sin(b),math.cos(b)]])
    lo,hi=spec.aabb(o,R)
    return PartState(origin=o,pos=(lo+hi)/2,low=lo,high=hi,quat_xyzw=ik.matrix_to_quat_xyzw(R),
                     lin_vel=np.array([velocity,0,0.]),ang_vel=np.zeros(3))


def states(index=0,where=None,ridged=False,lid=0.,press=None):
    T.TEST_LAYOUT=T.LAYOUTS[index]
    d={s.name:part(s) for s in (T.CUVETTE_A,T.CUVETTE_B,T.RACK,T.CARD)}
    blank,sample=T.identity_names()
    for i,name in enumerate(('cuvette_a','cuvette_b')):
        spec=T.CUVETTE_A if i==0 else T.CUVETTE_B
        d[name]=part(spec,(*T.POCKETS[i],T.POCKET_FLOOR))
    if where:
        name=blank if where=='blank' else sample
        turned=T.current_layout()[2 if where=='blank' else 3]
        d[name]=part(T.CUVETTE_A if name=='cuvette_a' else T.CUVETTE_B,(*T.WELL_XY,T.WELL_FLOOR),
                     yaw=(-90 if turned else 0)+(90 if ridged else 0))
    q={'lid':T.LID_TRAVEL-abs(lid),**({} if press is None else {n:-v for n,v in press.items()})}
    d['colorimeter']=T.COLORIMETER.offline_state(T.COLORIMETER.pos,0,q)
    return d


def grade(index=0,record=False,reference='blank',d=None):
    T.TEST_LAYOUT=T.LAYOUTS[index];T.TEST_INSTRUMENT=T.nominal_instrument(index,record,reference)
    return T.score(states(index) if d is None else d)


def tick(d,count=1):
    for _ in range(count):
        T._RUN.update(step=T._RUN['step']+1,t=T._RUN['t']+.05)
        value=T.score(d)
        assert T.score(d)==value and T.success(d)==(value==1.)
    return value


def start_live(index=0):
    T.TEST_LAYOUT=T.LAYOUTS[index];T.TEST_INSTRUMENT=None
    T._RUN.update(step=0,t=0.,episode=T._RUN['episode']+1)
    T.reset_history()
    # Set the visible starting wavelength with actual button press/release edges.
    requested,start,*_=T.current_layout()
    for _ in range(abs(requested-start)):
        name='more' if requested>start else 'less'
        tick(states(index,press={name:.0036}));tick(states(index))


@pytest.mark.parametrize('index',range(16))
def test_layout_start_and_nominal_end(index):
    T.TEST_LAYOUT=T.LAYOUTS[index]
    assert T.score(states(index))==0.
    assert grade(index,True)==1.
    assert T.success(states(index))
    assert grade(index,False,None)==0.


@pytest.mark.parametrize('index',range(16))
def test_actual_blank_calibration_measurement_and_return(index):
    start_live(index)
    assert tick(states(index,'blank',press={'cal_button':.0036}))==.25
    tick(states(index,'blank'))
    assert tick(states(index,'sample'),33)==.5
    assert tick(states(index),33)==1.
    # Record survives opening and emptying; reference tier remains current only.
    assert tick(states(index,lid=T.LID_TRAVEL))==1.
    assert tick(states(index,press={'more':.0036}))==1.


PROTOCOLS=(('sample CAL then sample','sample',False,'sample',False,0.,0.),
           ('ridged blank CAL','blank',True,'sample',False,0.,0.),
           ('sample ridged reading','blank',False,'sample',True,0.,.25),
           ('open lid reading','blank',False,'sample',False,.060,.25))


@pytest.mark.parametrize('name,cal,cal_ridge,read,read_ridge,lid,want',PROTOCOLS,ids=[p[0] for p in PROTOCOLS])
def test_wrong_protocols(name,cal,cal_ridge,read,read_ridge,lid,want):
    start_live()
    tick(states(0,cal,ridged=cal_ridge,press={'cal_button':.0036}))
    tick(states(0,cal,ridged=cal_ridge))
    assert tick(states(0,read,ridged=read_ridge,lid=lid),40)==want
    assert not T._EVER.get('record',False)


def test_correct_cal_then_recal_on_sample_before_reading():
    start_live();tick(states(0,'blank',press={'cal_button':.0036}));tick(states(0,'blank'))
    assert tick(states(0,'sample',press={'cal_button':.0036}),40)==0.
    assert T._INSTRUMENT.reading==pytest.approx(0.)


def test_cal_start_wavelength_then_change_loses_reference():
    T.TEST_LAYOUT=T.LAYOUTS[1];T._RUN.update(t=0.);T.reset_history()
    tick(states(1,'blank',press={'cal_button':.0036}));assert T._INSTRUMENT.ref is not None
    tick(states(1,'blank'))
    assert tick(states(1,'sample',press={'less':.0036}),40)==0.
    assert T._INSTRUMENT.ref is None


@pytest.mark.parametrize('bad',('below','edge_below'))
def test_sticky_loss_then_restoration(bad):
    start_live();T.TEST_INSTRUMENT=T.nominal_instrument(0,True)
    d=states();s=d['cuvette_a']
    if bad=='below':s=part(T.CUVETTE_A,s.origin*[1,1,0]+[0,0,T.TOP-.0051])
    else:
        # Origin is only 4 mm below table, but lowest bottom corner is >5 mm below.
        s=part(T.CUVETTE_A,(*T.POCKETS[0],T.TOP-.004),tilt=20)
        assert s.origin[2]>T.TOP-.005
    d['cuvette_a']=s
    assert tick(d)==0.
    assert tick(states(),40)==0.


@pytest.mark.parametrize('when',('before_record','after_record'))
def test_released_tip_over_then_restored_is_not_a_loss(when):
    # A sealed cuvette knocked over on the table and set upright again does not lose the trial.
    start_live()
    def tipped(where=None):
        d=states(0,where);blank,_=T.identity_names()
        d[blank]=part(T.CUVETTE_A,(.13,-.15,T.TOP+.00625),tilt=90)
        return d
    if when=='before_record':
        assert tick(tipped(),5)==0.
        assert not T._EVER.get('dropped',False)
    tick(states(0,'blank',press={'cal_button':.0036}));tick(states(0,'blank'))
    assert tick(states(0,'sample'),33)==.5
    if when=='after_record':
        # Sample back in its pocket and still; the blank lies on its side on the table.
        assert tick(tipped(),15)==.75
        assert not T._EVER.get('dropped',False)
    assert tick(states(),33)==1.


def test_tilt_held_or_released_is_not_a_loss():
    d=states();s=part(T.CUVETTE_A,(*T.POCKETS[0],T.TOP+.1),tilt=45);d['cuvette_a']=s
    T.TEST_GRIPPERS={'left':(s.pos,0.)}
    assert grade(record=True,d=d)==.75
    T.TEST_GRIPPERS=None
    assert grade(record=True,d=d)==.75


def test_live_return_tiers_release_motion_no_best_so_far():
    T.TEST_INSTRUMENT=T.nominal_instrument(0,True)
    d=states();blank,sample=T.identity_names()
    d[blank]=part(T.CUVETTE_A,(.13,-.15,T.TOP))
    assert T.score(d)==.75
    d=states();d[blank]=replace(d[blank],lin_vel=np.array([.2,0,0]))
    assert T.score(d)==.75
    d=states()
    for arm in ('left','right'):
        T.TEST_GRIPPERS={arm:(d[sample].pos,1.)}
        assert T.score(d)==.5
    T.TEST_GRIPPERS=None;assert T.score(d)==1.
    # A currently displaced sample loses its return tier.
    d[sample]=part(T.CUVETTE_B,(.13,-.15,T.TOP))
    assert T.score(d)==.5
    T.TEST_INSTRUMENT=T.nominal_instrument(0,False,None)
    assert T.score(states())==0.


def test_swapped_pockets_offset_and_one_per_pocket():
    d=states()
    d['cuvette_a']=part(T.CUVETTE_A,(*T.POCKETS[1],T.POCKET_FLOOR))
    d['cuvette_b']=part(T.CUVETTE_B,(*T.POCKETS[0],T.POCKET_FLOOR))
    assert grade(record=True,d=d)==1.
    d=states();d['cuvette_a']=part(T.CUVETTE_A,(* (T.POCKETS[0]+[.0025,0]),T.POCKET_FLOOR))
    assert grade(record=True,d=d)==1.
    d['cuvette_a']=part(T.CUVETTE_A,(*T.POCKETS[1],T.POCKET_FLOOR))
    assert grade(record=True,d=d)==.75
    d=states();d['cuvette_b']=part(T.CUVETTE_B,(*T.POCKETS[1],T.POCKET_FLOOR+.063))
    assert grade(record=True,d=d)==.5


def test_instrument_edges_hysteresis_order_clamps_invalid_cal_and_reset():
    l=T.LAYOUTS[0];h=T.Instrument(tuple(T.WELL_XY),T.WELL_FLOOR)
    s=T.snapshot(states(0,'blank'));step=0
    def run(buttons=None,lid=0.,episode=1):
        nonlocal step
        step+=1;s['buttons']=dict.fromkeys(('less','more','cal_button'),0.)
        s['buttons'].update(buttons or {});s['lid']=lid
        h.update(s,l,(episode,step),step*.05)
        ref=h.ref;h.update(s,l,(episode,step),step*.05)
        assert h.ref==ref
    run({'less':.003,'cal_button':.003});assert h.w==0 and h.ref is not None
    run({'less':.003});assert h.ref is not None  # End press leaves it intact.
    run({'more':.002,'cal_button':.002});assert h.w==1 and h.ref[0]==1
    run({'more':.0015});assert h.w==1
    run({'more':.002});assert h.w==1  # Not rearmed yet.
    run({'more':.001});run({'more':.002});assert h.w==2 and h.ref is None
    run();run({'more':.002});assert h.w==3
    run();run({'more':.002,'cal_button':.002});assert h.w==3 and h.ref is not None
    saved=h.ref;run();run({'cal_button':.002},lid=.060)
    assert h.ref==saved and h.last_failure is not None and h.cal_lamp(step*.05)=='red'
    run({'less':.002,'more':.002,'cal_button':.002})
    # CAL was already down: no new CAL; less then more both invalidate reference.
    assert h.w==3 and h.ref is None
    assert h.cal_lamp(step*.05)=='red';assert h.cal_lamp(step*.05+2.)=='off'
    run(episode=2);assert h.w==l[1] and h.ref is None
    run({'cal_button':.003});assert h.ref is not None


def test_measurement_continuity_release_lid_and_skipped_steps():
    start_live();tick(states(0,'blank',press={'cal_button':.0036}));tick(states(0,'blank'))
    d=states(0,'sample');sample=T.identity_names()[1]
    assert tick(d,20)==.25
    T.TEST_GRIPPERS={'left':(d[sample].pos,1.)};assert tick(d)==.25
    T.TEST_GRIPPERS=None;assert tick(d,10)==.25
    assert tick(states(0,'sample',lid=.060))==.25
    assert tick(d,20)==.25
    T._RUN['step']+=2;T._RUN['t']+=.1;assert tick(d)==.25
    assert tick(d,33)==.5


def test_pose_window_jitter_excursions_and_slow_tip():
    T._RUN.update(t=0.)
    d=states();n='cuvette_a';s=d[n]
    for step in range(11):
        T._RUN.update(step=step,t=step*.05)
        noisy=replace(s,ang_vel=np.array([.11,0,0]))
        assert T.quiet(noisy,n)==(step>=10)
        assert T.quiet(noisy,n)==(step>=10)
    T._RUN.update(step=11,t=.55)
    assert not T.quiet(part(T.CUVETTE_A,s.origin+[.0006,0,0]),n)
    T._RUN.update(step=12,t=.60);assert not T.quiet(s,n)
    T.reset_history()
    for step in range(25):
        T._RUN.update(step=step,t=step*.05)
        assert not T.quiet(part(T.CUVETTE_A,s.origin,tilt=step*.1),n)
    T._RUN.update(step=27,t=1.35);assert not T.quiet(s,n)
    T._RUN.update(step=2,t=.1);assert not T.quiet(s,n)


def test_lazy_demo_changes_after_reset_and_constant_timing():
    plan=T.demo();durations={a:[w.duration for w in ws] for a,ws in plan.items()}
    for i,l in enumerate(T.LAYOUTS):
        T.TEST_LAYOUT=l
        assert durations=={a:[w.duration for w in ws] for a,ws in plan.items()}
        for identity in ('blank','sample'):
            w=next(w for w in plan['right'] if w.label==identity+' in descend')
            pocket=l[4] if identity=='blank' else 1-l[4]
            assert np.allclose(w.pos[:2],T.POCKETS[pocket])
        inactive=[slot for slot in range(3) if slot>=abs(l[0]-l[1])]
        for slot in inactive:
            w=next(w for w in plan['left'] if w.label==f'wavelength slot {slot+1} press')
            assert np.allclose(w.pos,np.r_[T.LEFT_PARK_XY,T.TRAVEL_TCP])


def test_default_scorer_and_visual_updates_are_silent():
    import colorimeter_visuals as v
    # Diagnostic copy deliberately logs; the official module is silent.
    if T.DIAG_EVENTS:return
    with redirect_stdout(StringIO()) as out:
        start_live();tick(states(0,'blank',press={'cal_button':.0036}));v.update(states(0,'blank'))
    assert out.getvalue()==''


def test_button_held_before_eligibility_does_not_auto_calibrate():
    start_live();assert tick(states(press={'cal_button':.0036}))==0.
    assert tick(states(0,'blank',press={'cal_button':.0036}),40)==0.
    assert T._INSTRUMENT.ref is None
    tick(states(0,'blank',press={'cal_button':.001}))
    assert tick(states(0,'blank',press={'cal_button':.002}))==.25


def test_two_seated_cuvettes_invalidate_detector_and_cal():
    start_live();d=states(0,'blank',press={'cal_button':.0036})
    d['cuvette_b']=part(T.CUVETTE_B,(*T.WELL_XY,T.WELL_FLOOR))
    assert tick(d,35)==0.
    assert T._INSTRUMENT.ref is None and T._INSTRUMENT.reading is None


@pytest.mark.parametrize('offset,want',((.0029,True),(.003,False),(.0031,False)))
def test_well_lateral_boundary(offset,want):
    s=part(T.CUVETTE_A,(* (T.WELL_XY+[offset,0]),T.WELL_FLOOR))
    assert T.seated(s)==want


@pytest.mark.parametrize('offset,want',((.0034,True),(.0035,False),(.0036,False)))
def test_pocket_lateral_boundary(offset,want):
    s=part(T.CUVETTE_A,(* (T.POCKETS[0]+[offset,0]),T.POCKET_FLOOR))
    assert T.in_pocket(s,0)==want


def test_current_reference_lost_without_measurement_and_held_lid_motion():
    start_live();tick(states(0,'blank',press={'cal_button':.0036}));tick(states(0,'blank'))
    assert tick(states(0,'sample',press={'more':.0036}))==0.
    assert T._INSTRUMENT.ref is None
    # A slowly moving lid remains geometrically closed but cannot record.
    start_live();tick(states(0,'blank',press={'cal_button':.0036}));tick(states(0,'blank'))
    for i in range(40):
        assert tick(states(0,'sample',lid=(i%20)*.00006))==.25


@pytest.mark.parametrize('index',range(16))
def test_matching_ridged_faces_cannot_record(index):
    start_live(index)
    tick(states(index,'blank',ridged=True,press={'cal_button':.0036}))
    tick(states(index,'blank',ridged=True))
    assert tick(states(index,'sample',ridged=True),40)==0.
    assert T._INSTRUMENT.reading==pytest.approx(T.ABSORBANCE[T.LAYOUTS[index][0]])
    assert not T._EVER.get('record',False)
    assert tick(states(index),40)==0.


def test_held_inverted_lowest_body_corner_is_a_drop():
    d=states();s=part(T.CUVETTE_A,(*T.POCKETS[0],T.TOP+.038),tilt=180)
    assert min(T.bottom_corners(s)[:,2])>T.TOP
    assert T.lowest_body_corner(s)<T.TOP-.005
    d['cuvette_a']=s;T.TEST_GRIPPERS={'right':(s.pos,0.)}
    assert grade(record=True,d=d)==0.
