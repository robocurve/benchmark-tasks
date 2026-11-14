"""Offline geometry, anti-cheat, timing-window and nominal bimanual-plan tests."""
import contextlib
import io
import itertools
import math
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

T = load(Path(__file__).with_name("shim_door_hinge_jamb.py")).module
EPISODES = itertools.count(1000000)


@pytest.fixture(autouse=True)
def isolated():
    saved=dict(T._RUN)
    T._RUN.update(env=None,t=None,step=0,episode=next(EPISODES))
    T.TEST_GRIPPERS=T.TEST_GAP=None
    T.DEMO_PACE=1.
    yield
    T.TEST_GRIPPERS=T.TEST_GAP=None
    T._RUN.clear(); T._RUN.update(saved)


def placed(name,side,tip,flat,z=None,flip=False):
    spec=T.SHIM_LEFT if name=="shim_left" else T.SHIM_RIGHT
    return T.shim_state(spec,*T.shim_origin(side,tip,flat,z,flip))


@pytest.mark.parametrize("overlap",[.070,.100,.150,.190])
def test_exact_wedge_pack(overlap):
    a=placed("shim_left","left",-overlap/2,T.XS)
    b=placed("shim_right","right",overlap/2,T.XS+.007)
    for y in np.linspace(-overlap/2,overlap/2,9):
        assert T.thickness_at(a,y)+T.thickness_at(b,y)==pytest.approx(2*T.T_TIP+T.SLOPE*overlap)


@pytest.mark.parametrize("g",[.004,.005,.00635])
def test_target_overlap(g):
    o=(g+T.PRELOAD-2*T.T_TIP)/T.SLOPE
    assert 2*T.T_TIP+T.SLOPE*o==pytest.approx(g+.00015)
    assert .070<o<T.L


@pytest.mark.parametrize("seed,want",[(0,.004),(1,.00405),(47,.00635),(48,.004),(None,.005)])
def test_seed(seed,want):
    assert T.gap_for_seed(seed)==pytest.approx(want)


def test_trial_gap_precedence():
    T.ACTIVE_GAP=.004; T.FORCE_GAP=.006; T.TEST_GAP=.005
    assert T.trial_gap()==.005
    T.TEST_GAP=None; assert T.trial_gap()==.006
    T.FORCE_GAP=None; assert T.trial_gap()==.004
    T.ACTIVE_GAP=.005


@pytest.mark.parametrize("g",[.004,.005,.00635])
def test_goals(g):
    T.TEST_GAP=g
    assert T.score(T.pair_layout(g))==1.


@pytest.mark.parametrize("bow,want",[(.00035,1.),(.00045,.75),(.0008,.75),(.0010,.5),(.0013,.5),(.0018,.25)])
def test_bow_tiers(bow,want):
    assert T.score(T.pair_layout(.005,bow))==want


@pytest.mark.parametrize("bow",[.0010,.0012])
def test_ram_bow_measured_on_gpu_earns_half(bow):
    """The GPU ram (second shim driven 80 mm past touch) bowed the jamb 1.0 mm while pushing, 1.2 mm let go."""
    assert T.score(T.pair_layout(.004,bow))==.5


@pytest.mark.parametrize("loose,want",[(.00003,1.),(.0001,.25),(.0003,.25),(.0008,.25),(.002,0.)])
def test_loose_pair(loose,want):
    """Snug means the pack bears on the jamb: free space of 0.1 or 0.3 mm is a loose pack."""
    assert T.score(T.pair_layout(.005,bow=0.,loose=loose))==want


def test_fixed_depth_scores_full_credit_on_few_gaps():
    """One memorised pack thickness for every trial reaches 1.0 on at most 10 of the 48 gaps (README)."""
    gaps=[.004+.00005*k for k in range(48)]
    best=0
    for pack in np.arange(.0035,.0075,.00001):
        full=0
        for g in gaps:
            T.TEST_GAP=g
            lay=T.pair_layout(g,bow=max(0.,pack-g),loose=max(0.,g-pack))
            full+=T.terms(lay)["tier"]==1.
        best=max(best,full)
    T.TEST_GAP=None
    assert best<=10


def test_hand_guard_stops_the_drive(monkeypatch):
    """At wide gaps the drive stops before a tip comes within HAND_GUARD of the other hand's centre."""
    plan=T.demo()
    lay=T.pair_layout(.00635)
    tip_a=float(T.tip(lay["shim_left"])[1])
    hands={"left":(np.array([.26,.5,.88]),0.),"right":(np.array([.26,tip_a-T.HAND_GUARD+.001,.88]),0.)}
    monkeypatch.setattr(T,"_objects_now",lambda names:{n:lay[n] for n in names})
    monkeypatch.setattr(T,"grippers",lambda:hands)
    monkeypatch.setattr(T,"_jamb_now",lambda:T.JAMB.offline_state(T.JAMB.pos,T.JAMB.yaw,{"setter":.0025,"jamb":0.}))
    step=next(w for w in plan["right"] if w.label=="step B")
    assert step._skip() is True
    T._DEMO["stop"]=False
    hands["right"]=(np.array([.26,tip_a-T.HAND_GUARD-.001,.88]),0.)
    assert step._skip() is False


def single(loose):
    o=T.pair_layout(.005)
    g=T.gap_now(o["jamb"])
    y=T.YJ-(g-loose-T.T_TIP)/T.SLOPE
    o["shim_left"]=placed("shim_left","left",y,T.XS)
    o["shim_right"]=T.shim_state(T.SHIM_RIGHT,T.HOME_RIGHT,[0,0,1,0])
    return o


@pytest.mark.parametrize("loose,want",[(0.,.25),(.001,0.)])
def test_single(loose,want):
    assert T.score(single(loose))==want


def test_same_face():
    o=T.pair_layout(.005)
    o["shim_right"]=o["shim_left"]
    assert not T.pair(o["shim_left"],o["shim_right"],o["jamb"])


def test_butt_first():
    o=T.pair_layout(.005)
    o["shim_left"]=placed("shim_left","right",.150,T.XS+.005)
    assert T.side(o["shim_left"]) is None
    assert not T.terms(o)["pair"]


def test_flipped():
    o=T.pair_layout(.005)
    o["shim_left"]=placed("shim_left","left",T.tip(o["shim_left"])[1],T.XS,flip=True)
    assert not T.terms(o)["pair"]


def test_beside_middle():
    o=T.pair_layout(.005,centre=-.065,loose=.00515-(2*T.T_TIP+T.SLOPE*.070))
    assert T.overlap_interval(o["shim_left"],o["shim_right"])==pytest.approx((-.100,-.030))
    assert not T.terms(o)["pair"]


def test_high():
    o=T.pair_layout(.005)
    for name,spec in (("shim_left",T.SHIM_LEFT),("shim_right",T.SHIM_RIGHT)):
        s=o[name];o[name]=T.shim_state(spec,s.origin+[0,0,.030],s.quat_xyzw)
    assert T.score(o)==0.


def test_short_overlap():
    o=T.pair_layout(.005,loose=.00515-(2*T.T_TIP+T.SLOPE*.060))
    assert not T.terms(o)["pair"]


def test_held_goal():
    o=T.pair_layout(.005)
    T.TEST_GRIPPERS={"left":(o["shim_left"].origin,0.)}
    assert T.score(o)==.25


def test_jamb_release_box():
    o=T.pair_layout(.005)
    T.TEST_GRIPPERS={"right":(o["jamb"].link_view("jamb").pos,1.)}
    assert T.score(o)==.25


def tick(o,k):
    T._RUN.update(step=k,t=k*.05)
    return T.score(o)


def test_release_and_pose_windows():
    o=T.pair_layout(.005)
    T._RUN["t"]=0.
    T.TEST_GRIPPERS={"left":(o["shim_left"].origin,0.)}
    for k in range(20): assert tick(o,k)==.25
    T.TEST_GRIPPERS={"left":(np.array([.15,.40,T.TOP+.30]),1.)}
    for k in range(20,60): assert tick(o,k)==.25
    assert tick(o,60)==1.
    # Movement invalidates the last-second pose window even with zero reported velocities.
    moving=dict(o)
    s=o["shim_left"]
    moving["shim_left"]=T.shim_state(T.SHIM_LEFT,s.origin+[0,0,.0007],s.quat_xyzw)
    assert tick(moving,61)==.25
    for k in range(62,81): assert tick(moving,k)==.25
    assert tick(moving,81)==1.
    assert T.score(moving)==1.  # Repeated score and success calls do not add samples.


def test_joint_pose_window_and_interruption():
    o=T.pair_layout(.005)
    T._RUN["t"]=0.
    for k in range(42): tick(o,k)
    moved=T.pair_layout(.005,bow=.00021)
    assert tick(moved,42)==.25
    assert tick(moved,65)==.25  # Missed steps restart both timer and window.


def test_offline_velocities():
    o=T.pair_layout(.005)
    o["shim_left"]=replace(o["shim_left"],lin_vel=np.array([.003,0,0]))
    assert T.score(o)==.25
    o=T.pair_layout(.005)
    o["jamb"]=replace(o["jamb"],joint_vel={"setter":0.,"jamb":.003})
    assert T.score(o)==.25


def test_no_sticky_overdrive():
    assert T.score(T.pair_layout(.005,bow=.0018))==.25
    assert T.score(T.pair_layout(.005))==1.


def test_dynamic_not_jittered_and_exact_mesh():
    assert not T.SHIM_LEFT.free and not T.SHIM_RIGHT.free
    assert len(T.WEDGE.proxy)==8 and T.WEDGE.collision=="convex"
    assert len(T.SHIM_LEFT.shapes)==0


@pytest.mark.parametrize("g",[.004,.005,.00635])
@pytest.mark.parametrize("pace",[1.,.8])
def test_plan(g,pace):
    T.TEST_GAP,T.DEMO_PACE=g,pace
    with contextlib.redirect_stdout(io.StringIO()):
        plan=T.demo()
        # Resolve in time order, as the two arms are played by ArmPlan.
        arms={a:ArmPlan(ws,station.BASES[a]) for a,ws in plan.items()}
        for t in np.arange(0,max(p.duration for p in arms.values())+.051,.05):
            for arm,p in arms.items():
                pos,_,_,_=p.target(t)
                assert np.linalg.norm((pos-np.array(station.BASES[arm]))[:2])<=.36
        for arm,ws in plan.items():
            for w in ws:
                assert np.linalg.norm((np.array(w.pos)-np.array(station.BASES[arm]))[:2])<=.36
    va=-T.GRIP_LOCAL
    vb=-np.diag([-1.,-1.,1.])@T.GRIP_LOCAL
    advance=next(w for w in plan['left'] if w.label=='advance A')
    staging=next(w for w in plan['right'] if w.label=='carry staging')
    tip_a=(np.array(advance.pos)+va)[1]-T.L/2
    tip_b=(np.array(staging.pos)+vb)[1]+T.L/2
    assert tip_b<tip_a-.0199
    right_back=sum(w.duration for w in plan['right'][:next(i+1 for i,w in enumerate(plan['right']) if w.label=='back off')])
    left_hold=sum(w.duration for w in plan['left'][:next(i+1 for i,w in enumerate(plan['left']) if w.label=='hold for B release')])
    assert left_hold==pytest.approx(right_back+.3)


def test_adaptive_skip_latches():
    stop=[False]
    w=T.LazyWaypoint(.45,lambda:[0,0,0],0.,"step",skip=lambda:stop[0])
    assert w.duration==.45
    stop[0]=True
    _=w.pos
    assert w.duration==.05
    stop[0]=False
    assert w.duration==.05


@pytest.mark.parametrize("g",[.004,.005,.00635])
def test_both_turned_over(g):
    o=T.pair_layout(g,bow=.00015)
    for name,side,x in (("shim_left","left",T.XS+g+.00015),("shim_right","right",T.XS)):
        o[name]=placed(name,side,T.tip(o[name])[1],x,flip=True)
    assert T.terms(o)["pair"]
    assert T.score(o)==1.0


def test_grip_top_tilts_outward():
    from yam_arena import demo
    for arm,sign in (("left",1),("right",-1)):
        r=demo.grip_rotation(T.Grip(T.GRIP_YAWS[arm],T.GRIP_PITCH))
        assert (-r[:,2])[1]*sign>0.34


def test_ease_uses_measured_offset_after_aim(monkeypatch):
    plan=T.demo()
    for w in plan["left"][:11]: _=w.pos
    h=T._DEMO["left"]
    h["v"]+=np.array([.001,.002,.003])
    shim=T.pair_layout(.005)["shim_left"]
    monkeypatch.setattr(T,"_shim_now",lambda arm:shim)
    k=next(i for i,w in enumerate(plan["left"]) if w.label=="ease")
    ease=plan["left"][k]
    before=np.array(plan["left"][k-1].pos)
    # Only the push along the shim is taken out; sideways and up the hand stays put.
    assert np.array(ease.pos)==pytest.approx([before[0],(shim.origin-h["v"])[1],before[2]])


def test_pitch_held_through_backoff_and_slot_depth():
    plan=T.demo()
    for arm,ws in plan.items():
        for w in ws:
            expected=0. if w.label in ('park','stillness tail') else T.GRIP_PITCH
            assert w.yaw.grip_pitch==pytest.approx(expected)
    assert T.HOME_LEFT[2]-T.W/2-T.TOP==pytest.approx(.0221)
    assert T.HOME_RIGHT[2]-T.W/2-T.TOP==pytest.approx(.0221)
    floor=T.HOLDER_LEFT.shapes[0]
    assert floor.center[2]+floor.size[2]/2==pytest.approx(.022)


@pytest.mark.parametrize("g,bow,loose,want",[(.00525,.00015,0.,1.),(.004,.0014,0.,.5),(.00635,0.,.00095,.25)])
def test_memorised_depth(g,bow,loose,want):
    """One fixed pack, sized for a 5.25 mm gap with 0.15 mm of preload (5.40 mm), in other trials' gaps: full credit
    only near 5.25 mm; at 4.0 mm it must bow the jamb 1.4 mm (0.5); at 6.35 mm it is 0.95 mm loose (0.25, and on the
    simulator a pack that does not touch cannot hold itself up)."""
    assert T.score(T.pair_layout(g,bow=bow,loose=loose))==want


def test_grip_point_by_gap():
    """Up to 6.0 mm the hands hold each shim 10 mm from its butt; above, 7 mm from it, so that at the widest gap the
    drive can stop (HAND_GUARD) with the pack bowing the jamb at least 0.2 mm before a tip reaches the far finger."""
    assert T.grip_u(.004)==T.U_G and T.grip_u(.006)==T.U_G and T.grip_u(.00605)==T.U_G_WIDE
    guard_pack=2*T.T_TIP+T.SLOPE*(T.U_G_WIDE-T.HAND_GUARD)
    assert guard_pack-T.gap_for_seed(47)>=.0002
    assert T.U_G_WIDE-T.HAND_GUARD>=(.006+T.BOW_STOP-2*T.T_TIP)/T.SLOPE  # up to 6.0 mm the bow stop comes first
    assert T.L-T.U_G_WIDE>=.007


def test_roll_and_yaw_grip_rotation():
    from yam_arena import demo
    r0=demo.grip_rotation(T.Grip(T.GRIP_YAWS["left"],T.GRIP_PITCH))
    r1=demo.grip_rotation(T.Grip(T.GRIP_YAWS["left"],T.GRIP_PITCH,math.radians(2)))
    from yam_arena.urdf import axis_angle_matrix
    assert r1==pytest.approx(axis_angle_matrix(np.array([0.,1.,0.]),math.radians(2))@r0)
