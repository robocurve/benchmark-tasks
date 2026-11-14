"""A teal bench colorimeter has a square well at its right end, a sliding hood,
three push buttons (down-arrow, up-arrow, CAL), four labelled wavelength lamps and an absorbance
display. A rack to the right holds two sealed square cuvettes, with white and
blue stoppers and clear versus ridged face markings. A white card on the left
shows three wavelength digits. Dimensions are metres; use zero object jitter.
"""
import math
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl
from yam_joints import Mechanism, Link, Slide
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
from colorimeter_model import Instrument, WAVELENGTHS, ABSORBANCE, I0
SCENE = "station"
# Release-and-stillness block, verbatim from hang_holder_on_keyhole_studs.py (see release-stillness.md).
# The task module must define: import math; import numpy as np; from yam_arena.constants import FINGER_OPEN;
# and SCENE ("station" or "tabletop"). Paste after the imports.

# Release and stillness
# Released means every gripper TCP is at least RELEASE_CLEAR from the object's box.
# Still means below the task's linear and angular speed limits for STILL_S seconds.
# Evaluate every sticky condition and timer at the top of score() on every call,
# before any early return, so interrupted conditions cannot retain elapsed time.
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_TIMERS = {}
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, LEFT_BASE_POS, RIGHT_BASE_POS

    bases = station.BASES if SCENE == "station" else {"left": LEFT_BASE_POS, "right": RIGHT_BASE_POS}
    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], bases[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / FINGER_OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def released(state, clear=RELEASE_CLEAR):
    hands = grippers()
    if hands is None:
        return True  # Offline arm poses are unknown, so release is assumed.
    low, high = np.asarray(state.low), np.asarray(state.high)
    return all(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0)) >= clear
               for tcp, _ in hands.values())


def _episode_memory():
    global _MEMORY_EPISODE
    if _MEMORY_EPISODE != _RUN["episode"]:
        _TIMERS.clear()
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]


def held_for(key, cond):
    """Elapsed time over consecutive observed control steps; repeat calls are harmless."""
    _episode_memory()
    t, step = _RUN["t"], _RUN["step"]
    if t is None:
        return math.inf if cond else 0.0
    previous_step, start = _TIMERS.get(key, (None, None))
    if not cond:
        _TIMERS[key] = (step, None)
        return 0.0
    if start is None or step not in (previous_step, previous_step + 1) or t < start:
        start = t
    _TIMERS[key] = (step, start)
    return float(t - start)


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    if _RUN["t"] is None: return bool(cond)
    _episode_memory()
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]



DEMO_PACE = 1.0
DIAG_EVENTS = False
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 5200
HOUSING = (.110,.160,.065)
HOUSING_XY = np.array([.168,-.005])
WELL_LOCAL = np.array([.022,-.040])
WELL_XY = HOUSING_XY+WELL_LOCAL
RACK_XY = np.array([.190,-.250])
POCKETS = (RACK_XY+[0.,-.060],RACK_XY+[0.,.060])
WELL_WIDTH, WELL_FLOOR, MOUTH = .0145,TOP+.030,TOP+.065
POCKET_WIDTH, POCKET_FLOOR, RACK_TOP = .0155,TOP+.004,TOP+.024
LEAD_R, DROP_CLEAR = .005,.001
CUVETTE_OPEN = .25*FINGER_OPEN
BODY_WIDTH, BODY_HEIGHT, CAP_WIDTH, CAP_HEIGHT = .0125,.045,.014,.012
NECK_HEIGHT, CAP_BOTTOM, GRIP_LOCAL = .006,.051,.062
RIDE_LOW, SHORTFALL = .0035,.0014
TRAVEL_TCP = TOP+.165
WELL_AIM, POCKET_AIM = np.zeros(2),np.zeros(2)
LID_TRAVEL, HOOD_UNDERSIDE, HOOD_TOP = .060,.098,.102
TAB_CENTER, TAB_SIZE, TAB_TCP = (-.038,-.040,.121),(.024,.018,.020),TOP+.128
BUTTON_XYS = dict(zip(('less','more','cal_button'),
                     (HOUSING_XY+[x,.060] for x in (-.036,0.,.036))))
BUTTON_TOP, BUTTON_TRAVEL, SWITCH, RELEASE_SWITCH = TOP+.084,.005,.002,.001
BUTTON_PRESS_TCP = BUTTON_TOP+.013-(SWITCH+.003)
PARK_XY, LEFT_PARK_XY = np.array([.130,-.300]),np.array([.130,.300])
CARD_XY, CARD_SIZE, CARD_DIGIT_HEIGHT = np.array([.180,.225]),(.120,.080,.001),.024
READ_XY, READ_TCP = CARD_XY+[.037895,.000330],TOP+.150
DISPLAY_LOCAL = (-.040,.018)
PANEL_STAGE_XY = np.array([.180,.150])
PANEL_VIEW_XY, PANEL_VIEW_TCP = np.array([.150,.050]),TOP+.135
DIGITS = ('abcdef','bc','abdeg','abcdg','bcfg','acdfg','acdefg','abc','abcdefg','abcdfg')
# requested index, starting index, blank turned, sample turned, blank pocket.
LAYOUTS = (
    (0,0,False,False,0),(0,1,False,True,1),(0,2,True,False,0),(0,3,True,True,1),
    (1,1,False,True,0),(1,0,True,False,1),(1,2,False,False,1),(1,3,True,True,0),
    (2,2,True,False,0),(2,3,False,True,1),(2,1,True,True,0),(2,0,False,False,1),
    (3,3,True,True,1),(3,2,True,False,0),(3,1,False,True,1),(3,0,False,False,0),
)
TEST_LAYOUT = TEST_INSTRUMENT = None
_CURRENT_LAYOUT = LAYOUTS[0]


def current_layout():
    return TEST_LAYOUT if TEST_LAYOUT is not None else _CURRENT_LAYOUT


def layout_self_check():
    assert len(LAYOUTS)==len(set(LAYOUTS))==16
    assert all(sum(l[0]==i for l in LAYOUTS)==4 for i in range(4))
    diff=[l[0]-l[1] for l in LAYOUTS]
    assert diff.count(0)==4 and set(diff)==set(range(-3,4))
    assert sum(l[2] for l in LAYOUTS)==sum(l[3] for l in LAYOUTS)==8
    assert sum(l[2]!=l[3] for l in LAYOUTS)>=6
    assert sum(l[4]==0 for l in LAYOUTS)==8
    for i in (1,2): assert any(d>0 for d,l in zip(diff,LAYOUTS) if l[0]==i) and any(d<0 for d,l in zip(diff,LAYOUTS) if l[0]==i)
    return [(i,WAVELENGTHS[l[0]],WAVELENGTHS[l[1]],l[0]-l[1],l[2],l[3],l[4]) for i,l in enumerate(LAYOUTS)]


def identity_names(layout=None):
    l=current_layout() if layout is None else layout
    return ('cuvette_a','cuvette_b') if l[4]==0 else ('cuvette_b','cuvette_a')


def lead_in(xy,width,z):
    h=width/2+LEAD_R
    return [cyl((xy[0]+s*h,xy[1],z),LEAD_R,width+2*LEAD_R,axis='y') for s in (-1,1)]+[
        cyl((xy[0],xy[1]+s*h,z),LEAD_R,width+2*LEAD_R,axis='x') for s in (-1,1)]


layout_self_check()
# Solid lower floor and four wall strips, leaving a genuine 14.5 mm square well.
h=WELL_WIDTH/2; wx,wy=WELL_LOCAL
base=[box((0,0,.015),(.110,.160,.030))]
for lo,hi in ((-.055,wx-h),(wx+h,.055)):
    base.append(box(((lo+hi)/2,0,.0475),(hi-lo,.160,.035)))
for lo,hi in ((-.080,wy-h),(wy+h,.080)):
    base.append(box((wx,(lo+hi)/2,.0475),(WELL_WIDTH,hi-lo,.035)))
base += lead_in(WELL_LOCAL,WELL_WIDTH,.065)
# Rails and pillars attach the sliding hood to the housing rather than floating.
for y in (wy-.035,wy+.035):
    base.append(cyl((wx-.025,y,.092),.002,.100,axis='x'))
    for x in (wx-.073,wx+.023): base.append(box((x,y,.0785),(.004,.004,.027)))
hood=[box((wx-LID_TRAVEL,wy,.100),(.032,.032,.004)),box((wx-LID_TRAVEL,wy,.1065),(.008,.008,.009)),box(TAB_CENTER,TAB_SIZE)]
hood += [box((wx-LID_TRAVEL,wy+s*.017,.084),(.032,.002,.028)) for s in (-1,1)]
hood += [box((wx-LID_TRAVEL,wy+s*.025,.092),(.006,.016,.004)) for s in (-1,1)]
links=[Link('lid',hood,Slide('x',(wx-LID_TRAVEL,wy,.100),0,LID_TRAVEL,damping=.8),.055,
            color=(.10,.18,.26),friction=(.25,.18))]
for name,x in zip(('less','more','cal_button'),(-.036,0.,.036)):
    # Twelve mm cap protrudes above the housing and stays clear at full travel.
    links.append(Link(name,[box((x,.060,.078),(.016,.016,.012))],
        Slide('z',(x,.060,.078),-BUTTON_TRAVEL,0,spring=250,rest=.003,damping=1.2),
        .008,color=(.95,.49,.04) if name=='cal_button' else (.70,.74,.76)))
COLORIMETER=Mechanism('colorimeter',(*HOUSING_XY,TOP),base,links,
    contact_offset=.0001,color=(.15,.32,.40),self_collide=True)


def cuvette(name,pocket):
    # Both collider unions are identical and fourfold symmetric, including grip.
    return Part(name,(*POCKETS[pocket],POCKET_FLOOR+.001),[
        box((0,0,BODY_HEIGHT/2),(BODY_WIDTH,BODY_WIDTH,BODY_HEIGHT),color=(.62,.85,.90)),
        box((0,0,BODY_HEIGHT+NECK_HEIGHT/2),(.006,.006,NECK_HEIGHT),color=(.96,.96,.88)),
        box((0,0,CAP_BOTTOM+CAP_HEIGHT/2),(CAP_WIDTH,CAP_WIDTH,CAP_HEIGHT),color=(.96,.96,.88))],
        mass=.018,yaw=0.,grip=2,contact_offset=.0001,friction=(1.2,1.))


CUVETTE_A,CUVETTE_B=cuvette('cuvette_a',0),cuvette('cuvette_b',1)
p=POCKET_WIDTH/2
rack=[box((0,0,.002),(.044,.164,.004))]
rack += [box((s*(.022+p)/2,0,.014),(.022-p,.164,.020)) for s in (-1,1)]
for lo,hi in ((-.082,-.060-p),(-.060+p,.060-p),(.060+p,.082)):
    rack.append(box((0,(lo+hi)/2,.014),(POCKET_WIDTH,hi-lo,.020)))
for y in (-.060,.060): rack+=lead_in((0,y),POCKET_WIDTH,.024)
RACK=Part('rack',(*RACK_XY,TOP),rack,contact_offset=.0001,color=(.30,.37,.46))
CARD=Part('card',(*CARD_XY,TOP),[box((0,0,.0005),CARD_SIZE)],
          contact_offset=.0001,color=(.96,.95,.89))
OBJECTS=[COLORIMETER,CUVETTE_A,CUVETTE_B,RACK,CARD]


def bottom_corners(state):
    h=BODY_WIDTH/2
    return np.array([state.to_world((x,y,0.)) for x in (-h,h) for y in (-h,h)])


def lowest_body_corner(state):
    h=BODY_WIDTH/2
    return min(float(state.to_world((x,y,z))[2]) for x in (-h,h) for y in (-h,h) for z in (0.,BODY_HEIGHT))


def snapshot(objects):
    c=objects['colorimeter']; l=current_layout(); blank,sample=identity_names(l)
    cuvettes={}
    for name in ('cuvette_a','cuvette_b'):
        s=objects[name]; turned=l[2] if name==blank else l[3]
        axis=s.rotation()[:,1 if turned else 0]
        cuvettes[name]=dict(bottom=tuple(s.to_world((0,0,0))),lowest=float(bottom_corners(s)[:,2].min()),
                           tilt=float(s.tilt_deg()),clear_axis=tuple(axis),blank=name==blank)
    return dict(buttons={n:-float(c.joints[n]) for n in ('less','more','cal_button')},
                lid=float(c.joints['lid']),cuvettes=cuvettes)


_INSTRUMENT=Instrument(tuple(WELL_XY),WELL_FLOOR,WELL_WIDTH,BODY_WIDTH,closed_lid=LID_TRAVEL)


def instrument(objects):
    l=current_layout(); token=(_RUN['episode'],_RUN['step'])
    _INSTRUMENT.update(snapshot(objects),l,token,float(_RUN['t'] or 0.))
    return _INSTRUMENT


def nominal_instrument(index=0,record=False,reference='blank'):
    w=LAYOUTS[index][0]
    loss={'blank':0.,'ridged_blank':.6,'sample':ABSORBANCE[w],None:None}[reference]
    return dict(w=w,ref=None if loss is None else (w,I0[w]*10**(-loss)),record=record,dropped=False)


# A 12.5 mm body bottomed in the 15.5 mm pocket with 20 mm walls can lean up to ~8.8 degrees on a wall.
POCKET_TILT_DEG=9.


def in_pocket(s,pocket):
    b=s.to_world((0,0,0)); low=float(bottom_corners(s)[:,2].min())
    return bool(s.tilt_deg()<=POCKET_TILT_DEG+1e-10 and POCKET_FLOOR-.002-1e-12<=low<=POCKET_FLOOR+.002+1e-12 and
                np.all(np.abs(b[:2]-POCKETS[pocket])<.0035-1e-12))


def seated(s):
    b=s.to_world((0,0,0)); low=float(bottom_corners(s)[:,2].min())
    return bool(s.tilt_deg()<=3.+1e-10 and WELL_FLOOR-.002-1e-12<=low<=WELL_FLOOR+.002+1e-12 and
                np.all(np.abs(b[:2]-WELL_XY)<.003-1e-12))


def closed(c): return abs(float(c.joints['lid'])-LID_TRAVEL)<=.002+1e-12

class PoseHistory:
    """One pose per object/control step, with a complete 0.5 s observed window."""
    def __init__(self): self.reset()
    def reset(self): self.episode=None; self.samples={}; self.results={}
    def still(self,name,position,rotation=None):
        if self.episode!=_RUN['episode']:
            self.reset(); self.episode=_RUN['episode']
        t,step=_RUN['t'],_RUN['step']
        if t is None: return False  # A single offline pose cannot prove stillness.
        history=self.samples.setdefault(name,[])
        if history and step==history[-1][0] and t==history[-1][1]:
            return self.results[name]
        if history and (step!=history[-1][0]+1 or t<=history[-1][1]):
            history.clear()  # Missing observations or clock rewind need a fresh window.
        position=np.asarray(position,float).copy()
        rotation=None if rotation is None else np.asarray(rotation,float).copy()
        history.append((step,t,position,rotation))
        cutoff=t-.5
        # Keep the sample at/before the window boundary, not just newer poses.
        while len(history)>1 and history[1][1]<=cutoff+1e-12:
            history.pop(0)
        # Every observed pose must be within 0.5 mm / 0.5 deg of the current one;
        # this also rejects an excursion that returns to its starting pose.
        stable=history[0][1]<=cutoff+1e-12 and all(
            np.linalg.norm(position-p)<.0005-1e-12 and
            (rotation is None or (np.trace(r.T@rotation)-1)/2>math.cos(math.radians(.5))+1e-12)
            for _,_,p,r in history)
        self.results[name]=bool(stable)
        return self.results[name]


_POSE_HISTORY = PoseHistory()


def quiet(s,name):
    pose_still=(_RUN['t'] is None or _POSE_HISTORY.still(name,s.origin,s.rotation()))
    # Reported angular velocity jitters for resting light parts; use pose history.
    return bool(pose_still and np.linalg.norm(s.lin_vel)<.01)




def reset_history():
    _INSTRUMENT.reset(current_layout()[1])
    _POSE_HISTORY.reset()
    _TIMERS.clear(); _EVER.clear()
    import colorimeter_visuals
    colorimeter_visuals.reset_diagnostics()


def score(objects):
    live=instrument(objects)
    view=TEST_INSTRUMENT if TEST_INSTRUMENT is not None else dict(w=live.w,ref=live.ref)
    blank,sample=identity_names(); w=current_layout()[0]
    # Observe all poses every step, including while release or another gate fails.
    still={n:quiet(objects[n],n) for n in (blank,sample)}
    lidstill=(_RUN['t'] is None or _POSE_HISTORY.still('lid',[objects['colorimeter'].joints['lid']]))
    # Loss only: a cuvette whose body goes >5 mm below the table top has left the table. Tipping over on the
    # table is not penalised by itself; it only fails the seated and pocket tiers while it lasts.
    fallen=any(lowest_body_corner(objects[n])<TOP-.005-1e-12 for n in (blank,sample))
    dropped=ever('dropped',fallen or bool(view.get('dropped',False)))
    ref=view.get('ref'); good=(view['w']==w and ref is not None and ref[0]==w and ref[1]>0 and
                             abs(math.log10(I0[w]/ref[1]))<=.02+1e-12)
    axis=objects[sample].rotation()[:,1 if current_layout()[3] else 0]
    aligned=abs(axis[0])/np.linalg.norm(axis)>=math.cos(math.radians(10))-1e-12
    eligible=(good and aligned and live.reading is not None and live.w==w and live.seated==(sample,) and
              abs(live.reading-ABSORBANCE[w])<=.02+1e-12 and released(objects[sample]) and still[sample] and lidstill)
    elapsed=held_for('measurement',eligible)
    record=ever('record',bool(view.get('record',False)) if TEST_INSTRUMENT is not None else bool(eligible and elapsed>=1.-1e-10))
    pockets={n:next((i for i in range(2) if in_pocket(objects[n],i)),None) for n in (blank,sample)}
    samplehome=pockets[sample] is not None and released(objects[sample]) and still[sample]
    bothhome=samplehome and pockets[blank] is not None and pockets[blank]!=pockets[sample] and released(objects[blank]) and still[blank]
    returned=held_for('both_home',bool(record and bothhome))
    # Diagnostics share the physical instrument, never the test overrides.
    if DIAG_EVENTS:
        import colorimeter_visuals
        colorimeter_visuals.scoring_events(record,dropped)
    if dropped: return 0.
    if record:
        if bothhome and returned>=1.-1e-10: return 1.
        return .75 if samplehome else .5
    return .25 if good else 0.


def success(objects): return score(objects)==1.


class _LayoutWaypoint(Waypoint):
    """Resolve only visible start markings after reset; timing is always fixed."""
    def __init__(self,dt,pos,finger,label,yaw=0.):
        self.duration,self._pos,self.finger,self.label,self._yaw=dt*DEMO_PACE,pos,finger,label,yaw
    @property
    def pos(self): return tuple(self._pos() if callable(self._pos) else self._pos)
    @property
    def yaw(self): return self._yaw() if callable(self._yaw) else self._yaw


def demo():
    plans={'left':[],'right':[]}
    def add(arm,dt,xy,z,f,label,yaw=0.):
        pos=lambda: (*((xy() if callable(xy) else xy)),(z() if callable(z) else z))
        plans[arm].append(_LayoutWaypoint(dt,pos,f,label,yaw))
    def elapsed(arm): return sum(w.duration for w in plans[arm])/DEMO_PACE
    def wait_until(arm,t,xy,label):
        dt=t-elapsed(arm)
        if dt>1e-9: add(arm,dt,xy,TRAVEL_TCP,FINGER_OPEN,label)
    def lid(tag,start,end):
        # Joint limits stop the hood; the pinched rubber tab absorbs overtravel.
        xy=WELL_XY+[start,0]; dest=WELL_XY+[end+(.004 if end>start else -.004),0]
        for dt,p,z,f,part in (
            (1.5,xy,TRAVEL_TCP,FINGER_OPEN,'hover'),(.8,xy,TRAVEL_TCP,FINGER_OPEN,'settle'),
            (1.2,xy,TAB_TCP,FINGER_OPEN,'descend'),(.7,xy,TAB_TCP,FINGER_CLOSED,'close'),
            (2.5,dest,TAB_TCP,FINGER_CLOSED,'slide'),(.6,dest,TAB_TCP,FINGER_CLOSED,'stop'),
            (.7,dest,TAB_TCP,FINGER_OPEN,'release'),(1.,dest,TRAVEL_TCP,FINGER_OPEN,'rise')):
            add('right',dt,p,z,f,tag+' '+part)
    def transfer(tag,identity,inward,wait_lift_until=None):
        pocket=lambda: POCKETS[current_layout()[4] if identity=='blank' else 1-current_layout()[4]]
        turned=lambda: current_layout()[2 if identity=='blank' else 3]
        src=pocket if inward else WELL_XY; dst=WELL_XY if inward else pocket
        floor=POCKET_FLOOR if inward else WELL_FLOOR
        y0=0. if inward else math.pi/2; grasp=math.pi/2 if inward else 0.
        arrive=(lambda: 0. if turned() else math.pi/2) if inward else (lambda: math.pi/2 if turned() else 0.)
        pickz=floor+GRIP_LOCAL
        releasez=(MOUTH if inward else RACK_TOP)+LEAD_R+DROP_CLEAR+GRIP_LOCAL+RIDE_LOW
        for dt,p,z,f,part,yaw in (
            (1.5,src,TRAVEL_TCP,FINGER_OPEN,'hover',y0),(.8,src,TRAVEL_TCP,CUVETTE_OPEN,'settle',y0),
            (1.5,src,pickz,CUVETTE_OPEN,'descend',y0),(.8,src,pickz,FINGER_CLOSED,'close',y0),
            (.4,src,pickz,FINGER_CLOSED,'first hold',y0),(1.,src,pickz,CUVETTE_OPEN,'regrasp open',y0),
            (1.5,src,pickz,CUVETTE_OPEN,'regrasp turn',grasp),(.8,src,pickz,FINGER_CLOSED,'regrasp close',grasp),
            (.4,src,pickz,FINGER_CLOSED,'second hold',grasp),(1.5,src,TRAVEL_TCP,FINGER_CLOSED,'lift',grasp)):
            add('right',dt,p,z,f,tag+' '+part,yaw)
        if wait_lift_until is not None and elapsed('right')<wait_lift_until:
            add('right',wait_lift_until-elapsed('right'),src,TRAVEL_TCP,FINGER_CLOSED,tag+' wait for panel',grasp)
        for dt,p,z,f,part,yaw in (
            (2.,dst,TRAVEL_TCP,FINGER_CLOSED,'travel',arrive),(1.,dst,TRAVEL_TCP,FINGER_CLOSED,'aim settle',arrive),
            (2.,dst,releasez,FINGER_CLOSED,'lower',arrive),(.65,dst,releasez,FINGER_CLOSED,'drop settle',arrive),
            (1.9,dst,releasez,CUVETTE_OPEN,'release',arrive),(.6,dst,releasez,CUVETTE_OPEN,'released dwell',arrive),
            (1.,dst,TRAVEL_TCP,CUVETTE_OPEN,'rise',arrive),
            (1.,PARK_XY,TRAVEL_TCP,FINGER_OPEN,'back off',0.)):
            add('right',dt,p,z,f,tag+' '+part,yaw)
    add('left',1.5,READ_XY,READ_TCP,FINGER_OPEN,'read card hover')
    add('left',.8,READ_XY,READ_TCP,FINGER_OPEN,'read card settle')
    # The hood starts open; the right arm goes straight to the blank.
    for slot in range(3):
        active=lambda slot=slot: slot<abs(current_layout()[0]-current_layout()[1])
        xy=lambda active=active: (BUTTON_XYS['more' if current_layout()[0]>current_layout()[1] else 'less'] if active() else LEFT_PARK_XY)
        z=lambda active=active: BUTTON_PRESS_TCP if active() else TRAVEL_TCP
        for dt,zz,part in ((1.5,TRAVEL_TCP,'hover'),(.8,TRAVEL_TCP,'settle'),(1.5,z,'lower'),(.5,z,'press'),(1.,TRAVEL_TCP,'rise')):
            add('left',dt,xy,zz,FINGER_CLOSED,f'wavelength slot {slot+1} '+part)
    add('left',1.5,LEFT_PARK_XY,TRAVEL_TCP,FINGER_OPEN,'panel park')
    transfer('blank in','blank',True,elapsed('left'))
    lid('blank lid close',-LID_TRAVEL,0)
    add('right',1.,PARK_XY,TRAVEL_TCP,FINGER_OPEN,'blank parked')
    wait_until('left',elapsed('right'),LEFT_PARK_XY,'wait for blank')
    for dt,z,part in ((1.5,TRAVEL_TCP,'hover'),(.8,TRAVEL_TCP,'settle'),(1.5,BUTTON_PRESS_TCP,'lower'),(.5,BUTTON_PRESS_TCP,'press'),(1.,TRAVEL_TCP,'rise')):
        add('left',dt,BUTTON_XYS['cal_button'],z,FINGER_CLOSED,'CAL '+part)
    add('left',1.5,LEFT_PARK_XY,TRAVEL_TCP,FINGER_OPEN,'CAL park')
    wait_until('right',elapsed('left'),PARK_XY,'wait for CAL')
    lid('blank lid open',0,-LID_TRAVEL)
    transfer('blank out','blank',False)
    transfer('sample in','sample',True)
    lid('sample lid close',-LID_TRAVEL,0)
    # Stage outside the housing zone while right finishes closing the hood.
    wait_until('left',elapsed('right')-1.5,LEFT_PARK_XY,'wait for sample')
    add('left',1.5,PANEL_STAGE_XY,TRAVEL_TCP,FINGER_OPEN,'display staging')
    add('right',1.,PARK_XY,TRAVEL_TCP,FINGER_OPEN,'measurement park')
    wait_until('left',elapsed('right'),PANEL_STAGE_XY,'wait for measurement park')
    add('left',.6,PANEL_VIEW_XY,PANEL_VIEW_TCP,FINGER_OPEN,'display hover')
    add('left',1.9,PANEL_VIEW_XY,PANEL_VIEW_TCP,FINGER_OPEN,'display reading hold')
    # Right's next hover lasts 1.5 s: left is parked before it reaches the hood.
    add('left',1.,LEFT_PARK_XY,TRAVEL_TCP,FINGER_OPEN,'display return park')
    add('right',2.5,PARK_XY,TRAVEL_TCP,FINGER_OPEN,'measurement hold')
    lid('sample lid open',0,-LID_TRAVEL)
    transfer('sample out','sample',False)
    add('right',3.,PARK_XY,TRAVEL_TCP,FINGER_OPEN,'stillness tail')
    wait_until('left',elapsed('right'),LEFT_PARK_XY,'left stillness tail')
    return plans


import colorimeter_visuals
colorimeter_visuals.install(globals())
