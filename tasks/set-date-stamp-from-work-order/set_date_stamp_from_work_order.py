"""An orange two-band date stamp stands in a blue cradle beside a black ink pad
and a white work order. The sheet has a large printed DD.MM date above an empty
box. Day and month windows sit beside the setting fins; positive wheel rotation
is counter-clockwise seen from above. The face is passively spring cushioned.
All dimensions are metres; zero object jitter, right wrist camera for the date.
"""
import math
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import Part, box, cyl, ball
from yam_joints import Mechanism, Link, Hinge, Slide
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP
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
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
RUN_LIMIT_STEPS = 2400
CRADLE_XY = np.array([.275, -.417])
PAD_XY = np.array([.225, -.290])
DOC_XY = np.array([.225, -.130])
PAPER_TOP, PAD_TOP = TOP + .001, TOP + .010
DOCUMENT_SIZE, BOX_SIZE = np.array([.150, .130]), np.array([.080, .044])
BOX_XY = DOC_XY + [0., -.020]
ORDER_XY = DOC_XY + [0., .035]
ORDER_DIGIT_HEIGHT, BAND_DIGIT_HEIGHT = .024, .010
DOT_PITCH, DOT_R = .0008, .0005
BOX_LINE, BOX_MARGIN = .001, .005
SEAT_ROOT, START_ROOT = TOP + .013, TOP + .014
HOUSING_SIZE = np.array([.180, .060, .045])
HOUSING_BOTTOM, HOUSING_TOP = .012, .057
HANDLE = np.array([0., 0., .097])
HANDLE_SIZE = (.030, .018, .026)
POCKET_HALF = np.array([.100, .042])
FACE_POINT, FACE_SIZE = np.zeros(3), (.040, .018, .004)
# Four bottom corners of the 40x18 mm face, face-link frame: the lowest one decides contact.
FACE_CORNERS = tuple(np.array([sx*FACE_SIZE[0]/2, sy*FACE_SIZE[1]/2, 0.])
                     for sx in (-1, 1) for sy in (-1, 1))
FACE_TRAVEL = .004
FIN_SIZE = (.030, .016, .022)
MONTH_FIN_SIZE = (.044, .016, .022)
WHEEL_Z, FIN_Z, KNOB_TCP_LOCAL = .065, .126, .120 + PAD_TIP_PAST_TCP
# name, x, count, pin ring, pin radius, ball radius, hub radius, travel, fin
BANDS = (("day", .060, 31, .020, .0015, .0025, .012, .002, FIN_SIZE),
         ("month", -.060, 12, .025, .003, .005, .022, .008, MONTH_FIN_SIZE))
BAND_SEATED_DEG, INK_Q, PRINT_Q = 4., .001, .001
SURFACE_Z, INK_TILT, PRINT_TILT, PRINT_YAW = .002, 10., 5., 5.
REARM_Q, REARM_RISE = .0005, .003
SMEAR_XY, SMEAR_YAW = .002, 2.
SEAT_TILT, SEAT_Z = 5., .003
STILL_SPEED, STILL_JOINT_SPEED = .010, .050
BAND_CREDIT, PRINT_CREDIT, SEAT_CREDIT = .15, .7, .9
RIDE_LOW, SHORTFALL = .0035, .0014
TRAVEL_TCP = TOP + .170
READ_XY = np.array([.262895, -.094670])
READ_TCP = TOP + .160
PAD_PRESS_TCP = PAD_TOP + HANDLE[2] + RIDE_LOW - SHORTFALL - .003
DOC_PRESS_TCP = PAPER_TOP + HANDLE[2] + RIDE_LOW - SHORTFALL - .0035
# Constant measured arm/grip aim correction, applied identically to every layout.
PAPER_AIM_XY = np.array([.0050, .0047])
PAPER_AIM_YAW = math.radians(-2.)
RETURN_TCP = SEAT_ROOT + HANDLE[2] + RIDE_LOW + .0025
PARK_XY = np.array([.220, -.110])
PARK_TCP = TOP + .040
# Each tuple is visible start day/month, printed requested day/month.
LAYOUTS = (
    (3, 12, 5, 1), (11, 2, 14, 2), (24, 1, 28, 3), (29, 1, 3, 4),
    (10, 6, 16, 5), (23, 8, 30, 6), (5, 10, 7, 7), (18, 8, 21, 8),
    (4, 8, 2, 9), (21, 8, 18, 10), (31, 8, 27, 11), (14, 1, 9, 12),
    (31, 3, 25, 1), (17, 5, 10, 2), (6, 3, 4, 3), (25, 4, 22, 4),
)
# Four zero month changes: layouts 1, 7, 14, 15. Layout 0 needs both bands changed.
TEST_LAYOUT = TEST_EVENTS = TEST_INKED = TEST_SMEARED = None
_CURRENT_LAYOUT = LAYOUTS[0]


def current_layout():
    return TEST_LAYOUT if TEST_LAYOUT is not None else _CURRENT_LAYOUT


def shortest_change(start, requested, count):
    return (requested-start+count//2) % count-count//2


def changes(layout):
    a,b,c,d = layout
    return shortest_change(a,c,31), shortest_change(b,d,12)


def layout_self_check():
    assert len(LAYOUTS) == len(set(LAYOUTS)) == 16
    rows = [(i,l,*changes(l)) for i,l in enumerate(LAYOUTS)]
    ds, ms = [r[2] for r in rows], [r[3] for r in rows]
    assert sum(d>0 for d in ds) == sum(d<0 for d in ds) == 8
    assert set(abs(d) for d in ds) == set(range(2,8))
    assert all(2 <= abs(d) <= 7 for d in ds)
    assert ms.count(0) == 4 and sum(d>0 for d in ms) == sum(d<0 for d in ms) == 6
    assert set(ms) == set(range(-3,4))
    assert len({l[3] for l in LAYOUTS}) >= 8
    assert min(l[2] for l in LAYOUTS) <= 7 and max(l[2] for l in LAYOUTS) >= 25
    for a,b,c,d in LAYOUTS:
        assert 1 <= a <= 31 and 1 <= b <= 12
        assert 1 <= c <= (31,28,31,30,31,30,31,31,30,31,30,31)[d-1]
    return rows


layout_self_check()
base = [box((0,0,.0345), HOUSING_SIZE),
        box((0,0,.0705), (.010,.010,.027)),
        box(HANDLE, HANDLE_SIZE, color=(.12,.14,.18))]
links, FIN_SHAPES = [], {}
for name,x,count,ring,pin_r,ball_r,hub_r,travel,fin in BANDS:
    seat = ring*math.cos(math.pi/count)+math.sqrt((pin_r+ball_r)**2-(ring*math.sin(math.pi/count))**2)
    wheel = [cyl((x,0,.060 if name=='day' else .0585),ring+pin_r,.002), cyl((x,0,.088),hub_r,.054)]
    for k in range(count):
        a = math.pi/2 + math.pi/count + k*2*math.pi/count
        wheel.append(cyl((x+ring*math.cos(a),ring*math.sin(a),WHEEL_Z),pin_r,.008 if name=="day" else .012))
    wheel.append(box((x,0,FIN_Z),fin,color=(.90,.28,.045) if name=="day" else (.10,.45,.85)))
    FIN_SHAPES[name] = len(base)+sum(len(l.shapes) for l in links)+len(wheel)-1
    links.append(Link(name,wheel,Hinge("z",(x,0,WHEEL_Z),-350,350,
                                      damping=.00035,armature=1e-6),.020,friction=(.1,.08)))
    rod_r,rod_h,rod_y = (.0008,.002,.003) if name=="day" else (.0015,.004,.005)
    links.append(Link(name+"_plunger",[ball((x,seat,WHEEL_Z),ball_r),
                        cyl((x,seat+rod_y,WHEEL_Z),rod_r,rod_h,axis="y")],
                      Slide("y",(x,seat,WHEEL_Z),-.0002,travel,spring=300,rest=-.003,
                            damping=.4 if name=="day" else 2.),.003,friction=(.1,.08)))
links.append(Link("face",[box((0,0,.002),FACE_SIZE)],
                  Slide("z",(0,0,.002),0,FACE_TRAVEL,spring=800,rest=-.001,damping=2),
                  .015,color=(.055,.060,.075),friction=(.6,.4)))
STAMP = Mechanism("stamp",(*CRADLE_XY,START_ROOT),base,links,mass=.197,
                  color=(.92,.23,.035),friction=(1.2,1.),contact_offset=.0001,grip=2,self_collide=True)
cradle_shapes = [box((0,0,.002),(.208,.092,.004))]
cradle_shapes += [box((0,s*.0245,.0145),(.180,.009,.021)) for s in (-1,1)]
cradle_shapes += [box((s*.102,0,.0165),(.004,.092,.025)) for s in (-1,1)]
cradle_shapes += [box((0,s*.044,.0165),(.200,.004,.025)) for s in (-1,1)]
CRADLE = Part("cradle",(*CRADLE_XY,TOP),cradle_shapes,color=(.22,.35,.48),
              friction=(1.2,1.),contact_offset=.0001)
PAD = Part("pad",(*PAD_XY,TOP),[box((0,0,.005),(.090,.060,.010))],
           color=(.055,.07,.10),friction=(1.2,1.),contact_offset=.0001)
WORK_ORDER = Part("work_order",(*DOC_XY,TOP),[box((0,0,.0005),(*DOCUMENT_SIZE,.001))],
                  color=(.96,.95,.89),friction=(1.2,1.),contact_offset=.0001)
OBJECTS = [STAMP,CRADLE,PAD,WORK_ORDER]


# Shared seven-segment geometry: four 10 mm glyphs plus decimal dot; exact
# printed area, readout and mirrored underside use the same rectangles.
DIGITS = ('abcdef','bc','abdeg','abcdg','bcfg','acdfg','acdefg','abc','abcdefg','abcdfg')
GLYPH_HEIGHT, GLYPH_WIDTH, GLYPH_PITCH, SEGMENT_WIDTH = .010,.006,.007,.0015
DIGIT_CENTRES = (-.0115,-.0045,.0045,.0115)


def glyph_rectangles():
    out = []
    for digit,x in enumerate(DIGIT_CENTRES):
        for segment,y in (('a',.00425),('g',0.),('d',-.00425)):
            out.append((digit,segment,np.array([x,y]),np.array([.0045,.0015])))
        for segment,dx,y in (('b',.00225,.002125),('c',.00225,-.002125),
                             ('e',-.00225,-.002125),('f',-.00225,.002125)):
            out.append((digit,segment,np.array([x+dx,y]),np.array([.0015,.00425])))
    out.append((-1,'.',np.array([0.,-.004]),np.array([.0015,.0015])))
    return out


def active_rectangles(day,month):
    text = (None if day is None else f"{day:02d}",None if month is None else f"{month:02d}")
    return [(xy,size) for d,s,xy,size in glyph_rectangles()
            if d == -1 or (text[d//2] is not None and s in DIGITS[int(text[d//2][d%2])])]


def band_reading(stamp,name):
    index = 0 if name == "day" else 1
    count = BANDS[index][2]
    angle, pitch = stamp.angle_deg(name),360/count
    detent = int(math.floor(angle/pitch+.5))
    if abs(angle-detent*pitch) > BAND_SEATED_DEG+1e-10: return None
    return (current_layout()[index]-1+detent) % count+1


def face_pose(stamp):
    p,R = stamp.link_to_world("face",FACE_POINT),stamp.link_rotation("face")
    return p, math.atan2(R[1,0],R[0,0]), math.degrees(math.acos(float(np.clip(R[2,2],-1,1))))


def over(xy,centre,size):
    return bool(np.all(np.abs(np.asarray(xy)-centre) <= np.asarray(size)/2+1e-12))


def event(day=None,month=None,dx=0.,dy=0.,yaw_deg=0.,tilt=0.,on_sheet=True):
    """Independent test fixture; yaw stored in radians, pose in world metres."""
    l = current_layout()
    return dict(pose=np.r_[BOX_XY+[dx,dy],PAPER_TOP],yaw=math.radians(yaw_deg),tilt=tilt,
                day=l[2] if day is None else day,month=l[3] if month is None else month,on_sheet=on_sheet)


def nominal_event(layout_index):
    l=LAYOUTS[layout_index]
    return event(day=l[2],month=l[3])


def correct_impression(e):
    l=current_layout()
    if (e['day'],e['month']) != (l[2],l[3]) or not e['on_sheet']: return False
    p=np.asarray(e['pose']); a=float(e['yaw']); tilt=float(e['tilt'])
    if not np.all(np.isfinite(np.r_[p,a,tilt])): return False
    low=float(e.get('low',p[2]))
    if not over(p[:2],DOC_XY,DOCUMENT_SIZE) or abs(low-PAPER_TOP) > SURFACE_Z+1e-12: return False
    if abs(math.degrees(math.atan2(math.sin(a),math.cos(a)))) > PRINT_YAW+1e-10 or not 0 <= tilt <= PRINT_TILT+1e-10: return False
    R=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    bound=BOX_SIZE/2-BOX_LINE/2-BOX_MARGIN
    # Each ink rectangle, including its corners, rather than a digit-area AABB.
    for xy,size in active_rectangles(e['day'],e['month']):
        if np.any(np.abs(p[:2]-BOX_XY+R@xy)+np.abs(R)@(size/2) > bound+1e-12): return False
    return True


class PressHistory:
    """Pose/joint-only recorder; independent instances for scoring and visuals.

    A low-compression sample AND a 3 mm rise are both needed to re-arm.
    Retain the smallest press height so later deeper compression cannot hide a
    real lift. Until re-armed, any off-pad surface contact compressed >= INK_Q
    whose face centre is > SMEAR_XY from, or yaw > SMEAR_YAW from, the recorded
    impression is a sticky smear. Surface contact uses the lowest of the face's
    four bottom corners, so a tilted edge press counts; location, yaw and pad
    tests use the face centre. Tokens make repeat calls on one step harmless.
    """
    def __init__(self): self.reset()
    def reset(self):
        self.token,self.inked,self.armed,self.press_z = None,False,True,None
        self.events,self.mark,self.smeared=[],None,False
    def update(self,stamp,token):
        if self.token is not None and token[0] != self.token[0]: self.reset()
        if token == self.token: return None
        p,yaw,tilt=face_pose(stamp); q=stamp.joints['face']
        pad=over(p[:2],PAD_XY,(.090,.060))
        low=min(float(stamp.link_to_world('face',c)[2]) for c in FACE_CORNERS)
        surface=min(abs(low-PAPER_TOP),abs(low-TOP)) <= SURFACE_Z+1e-12
        if pad and q >= INK_Q and abs(p[2]-PAD_TOP) <= SURFACE_Z+1e-12 and tilt <= INK_TILT+1e-10:
            self.inked=True
        if not self.armed and self.mark is not None and not pad and q >= INK_Q and surface:
            turn=math.degrees(math.atan2(math.sin(yaw-self.mark[1]),math.cos(yaw-self.mark[1])))
            if np.linalg.norm(p[:2]-self.mark[0]) > SMEAR_XY+1e-12 or abs(turn) > SMEAR_YAW+1e-10:
                self.smeared=True
        if not self.armed:
            self.press_z=min(self.press_z,low)
            if q < REARM_Q and low-self.press_z >= REARM_RISE-1e-12: self.armed=True
        e=None
        if self.armed and self.inked and not pad and q >= PRINT_Q and surface:
            on_sheet=over(p[:2],DOC_XY,DOCUMENT_SIZE) and abs(low-PAPER_TOP)<=SURFACE_Z
            e=dict(pose=p.copy(),low=low,yaw=yaw,tilt=tilt,day=band_reading(stamp,'day'),
                   month=band_reading(stamp,'month'),on_sheet=on_sheet)
            self.events.append(e); self.armed=False; self.press_z=low
            self.mark=(p[:2].copy(),yaw)
        self.token=token
        return e


_SCORE_HISTORY=PressHistory()


def reset_history():
    _SCORE_HISTORY.reset()


def record_press(stamp):
    return _SCORE_HISTORY.update(stamp,(_RUN['episode'],_RUN['step']))


def seated(stamp):
    R=stamp.rotation()
    extent=np.abs(R[:2,:]) @ (HOUSING_SIZE/2)
    centre=stamp.to_world((0,0,.0345))
    return bool(stamp.tilt_deg()<=SEAT_TILT and abs(stamp.origin[2]-SEAT_ROOT)<=SEAT_Z+1e-12 and
                np.all(np.abs(centre[:2]-CRADLE_XY)+extent <= POCKET_HALF+1e-12))


def stamp_still(stamp):
    # Preloaded detent contacts can report knob velocity without angle motion.
    return bool(np.linalg.norm(stamp.lin_vel) < STILL_SPEED and
                np.linalg.norm(stamp.ang_vel) < STILL_JOINT_SPEED and
                all(abs(stamp.joint_vel[n]) < STILL_JOINT_SPEED
                    for n in ('face','day_plunger','month_plunger')) and
                all(abs(stamp.angle_deg(n)-round(stamp.angle_deg(n)/(360/count))*(360/count)) <= 1.0+1e-10
                    for n,count in (('day',31),('month',12))))


def score(objects):
    stamp=objects['stamp']; record_press(stamp)
    events=TEST_EVENTS if TEST_EVENTS is not None else _SCORE_HISTORY.events
    inked=TEST_INKED if TEST_INKED is not None else _SCORE_HISTORY.inked
    smeared=TEST_SMEARED if TEST_SMEARED is not None else _SCORE_HISTORY.smeared
    invalid=bool(smeared) or (bool(events) and (not inked or len(events)!=1 or
                                                any(not correct_impression(e) for e in events)))
    violation=ever('permanent_bad_impression',invalid)
    printed=ever('correct_impression',bool(inked and len(events)==1 and correct_impression(events[0])))
    in_cradle=seated(stamp)
    done=not violation and printed and in_cradle and released(stamp) and stamp_still(stamp)
    elapsed=held_for('printed_returned_released_still',done)
    if violation: return 0.
    if printed:
        if not in_cradle: return PRINT_CREDIT
        return 1. if done and elapsed>=STILL_S else SEAT_CREDIT
    # A band that needed a change earns credit when it is right; a band whose
    # requested value equals its start earns credit only when every band is right.
    l,need=current_layout(),changes(current_layout())
    right=[band_reading(stamp,n)==l[i+2] for i,n in enumerate(('day','month'))]
    bands=sum(r for r,c in zip(right,need) if c!=0)+(sum(c==0 for c in need) if all(right) else 0)
    return round(BAND_CREDIT*bands,6)


def success(objects): return score(objects)==1.


def goal_joints(layout_index):
    d,m=changes(LAYOUTS[layout_index])
    return {'day':math.radians(d*360/31),'month':math.radians(m*30)}


def cheat_cases():
    l=LAYOUTS[0]
    bad=event();bad['day']=None
    # Put the outermost active glyph exactly 3 mm from the box's inner line.
    xmax=max(xy[0]+size[0]/2 for xy,size in active_rectangles(l[2],l[3]))
    margin=event(dx=BOX_SIZE[0]/2-BOX_LINE/2-.003-xmax)
    return [
        ('wrong day',[event(day=l[0])]),
        ('wrong month',[event(month=l[1])]),
        ('start date then correct',[event(day=l[0],month=l[1]),nominal_event(0)]),
        ('two correct impressions',[nominal_event(0),nominal_event(0)]),
        ('glyph 3 mm from box line',[margin]),
        ('rotated 8 deg',[event(yaw_deg=8)]),
        ('on table beside sheet',[event(dx=.100,on_sheet=False)]),
        ('blank day 6 deg off detent',[bad]),
        ('correct then later table press',[nominal_event(0),event(dx=.100,on_sheet=False)]),
        ('tilted 6 deg',[event(tilt=6)]),
    ]


# Fixed timing is built before reset. Only yaw resolves lazily, from the layout
# tuple: its start date is what the stamp windows show and its requested date is
# what the work order prints. No physical-state or scorer reads.
class _SettingWaypoint(Waypoint):
    def __init__(self,dt,pos,finger,label,band,turned):
        self.duration,self.pos,self.finger,self.label=dt*DEMO_PACE,pos,finger,label
        self.band,self.turned=band,turned
    @property
    def yaw(self):
        if not self.turned: return 0.
        return math.radians(changes(current_layout())[self.band]*(360/(31 if self.band==0 else 12)))


def demo():
    right=[]
    def add(dt,xy,z,f,label,yaw=0.): right.append(Waypoint(dt*DEMO_PACE,(*xy,z),f,label,yaw))
    # Empty gripper views the date with the right wrist before configuring.
    add(1.5,READ_XY,READ_TCP,FINGER_OPEN,'read order hover')
    add(.8,READ_XY,READ_TCP,FINGER_OPEN,'read order settle')
    for i,(name,x,*_) in enumerate(BANDS):
        xy=CRADLE_XY+[x,0]; z=SEAT_ROOT+KNOB_TCP_LOCAL
        for dt,zz,f,label,turned in (
            (1.5,TRAVEL_TCP,FINGER_OPEN,'hover',False),(.8,TRAVEL_TCP,FINGER_OPEN,'settle',False),
            (1.5,z,FINGER_OPEN,'descend',False),(.8,z,FINGER_CLOSED,'close',False),
            (3.,z,FINGER_CLOSED,'twist',True),(.5,z,FINGER_CLOSED,'hold',True),
            (.8,z,FINGER_OPEN,'open',True),(1.,TRAVEL_TCP,FINGER_OPEN,'lift',True),
            (.8,TRAVEL_TCP,FINGER_OPEN,'after',True)):
            right.append(_SettingWaypoint(dt,(*xy,zz),f,name+' '+label,i,turned))
    hxy=CRADLE_XY+HANDLE[:2]
    add(1.5,hxy,TRAVEL_TCP,FINGER_OPEN,'handle hover')
    add(.8,hxy,TRAVEL_TCP,FINGER_OPEN,'handle settle')
    add(1.5,hxy,SEAT_ROOT+HANDLE[2],FINGER_OPEN,'handle descend')
    add(.8,hxy,SEAT_ROOT+HANDLE[2],FINGER_CLOSED,'handle close')
    add(1.5,hxy,TRAVEL_TCP,FINGER_CLOSED,'stamp lift')
    for name,xy,z,yaw in (('pad',PAD_XY,PAD_PRESS_TCP,0.),
                          ('paper',BOX_XY+PAPER_AIM_XY,DOC_PRESS_TCP,PAPER_AIM_YAW)):
        add(2.,xy,TRAVEL_TCP,FINGER_CLOSED,name+' travel',yaw)
        add(.8,xy,TRAVEL_TCP,FINGER_CLOSED,name+' settle',yaw)
        add(2.,xy,z,FINGER_CLOSED,name+' lower',yaw)
        add(.5,xy,z,FINGER_CLOSED,name+' press',yaw)
        add(1.5,xy,TRAVEL_TCP,FINGER_CLOSED,name+' lift',yaw)
    add(2.,hxy,TRAVEL_TCP,FINGER_CLOSED,'return travel')
    add(.8,hxy,TRAVEL_TCP,FINGER_CLOSED,'return settle')
    add(2.,hxy,RETURN_TCP,FINGER_CLOSED,'return lower')
    add(.8,hxy,RETURN_TCP,FINGER_OPEN,'return open')
    add(1.,hxy,TRAVEL_TCP,FINGER_OPEN,'return rise')
    add(1.5,PARK_XY,PARK_TCP,FINGER_OPEN,'back off')
    add(4.,PARK_XY,PARK_TCP,FINGER_OPEN,'stillness tail')
    return {'right':right}


import dater_visuals
dater_visuals.install(globals())
