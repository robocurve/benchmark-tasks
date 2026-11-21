"""On the station table: a section of a car's windscreen, laminated glass leaning back 30 deg from the horizontal (60 deg
from vertical, the rake of a typical car windscreen), held on a bench stand that stands in for the car's dashboard;
a day/night interior rear-view mirror hangs from it on its stem, the way it hangs in a car, aimed and in its day
position. Task (instruction): switch the mirror to its night (anti-glare) position without changing where it is aimed.
The instruction does not say how.

How it is done (Kia owner's manual, "Day/night rear view mirror"): "Make this adjustment before you start driving and
while the day/night lever is in the day position. Pull the day/night lever toward you to reduce the glare from the
headlights of the vehicles behind you during night driving." The mirror is aimed in the day position. The lever tips
the housing a few degrees on the bracket inside it, so that the dim reflection from the wedge-shaped glass's uncoated
front face takes the place of the bright one from its silvered back: the night image is the same view, dimmer,
provided the ball joint the housing hangs on has not turned. (That built-in tilt is not modelled: here the tab's two
positions are the device's state, and any turn of the housing is a turn of the ball joint.)

Why two hands: the lever is a 22 mm tab on a two-position over-centre hinge at the middle of the housing's bottom edge,
38 mm below the ball joint. Pulling it over its centre takes 0.10 N m about its hinge (about 5 N at the tab), and the
hinge's spring pushes the same torque back on the housing; the ball joint of this mirror holds the housing with only
0.07 N m of friction (a mirror you can aim with a push of 0.55 N at its end). So whichever way one hand flips the tab,
the housing tips on its ball before the tab goes over centre and the aim is lost. The other hand steadies the housing
at its end while the tab is flipped, then both let go.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- windscreen stand (fixed): a dashboard block 12 cm deep, 38 cm wide, its top 18.5 cm above the table at x 0.50-0.62;
  the windscreen section, 5 mm laminated glass 36 cm wide and 38 cm along its slope, rising toward the robot at 30 deg
  from its lower edge on the dashboard (x 0.52, 18.5 cm up) to its upper edge (x 0.20, 37.5 cm up), with a 3 cm
  black ceramic band round its edge (the clear glass itself is not drawn, so the cameras see the mirror through it);
  two side brackets under its far half. The mirror's button is bonded to the glass's inside face at (0.299, 0.050), 31.5 cm above the
  table, with the mirror's wedge mount slid onto it.
- mirror (as a CIPA CM32000, 10 in day/night wedge-mount mirror): housing 254 x 64 x 35 mm (long side along y), black
  plastic, 180 g with its glass, its middle at (0.255, 0.050), 24.0 cm above the table, glass facing the robot; prism
  glass 244 x 54 mm. The housing hangs on a ball joint inside it, 6 mm above its middle and straight above its centre
  of mass (friction 0.07 N m about each axis, the ball turns at most 25 deg each way), on a stem from the wedge mount.
- day/night tab: 28 mm wide, 22 mm long, 5 mm thick, hinged across the housing's bottom edge, 3 mm behind the glass
  face; in the day position it points down and 15 deg toward the robot. Night is 35 deg further toward the robot
  (pulled toward the driver), where it rests on its stop. An over-centre spring holds it at whichever end it is
  nearer: its torque rises from 0 at the stop to 0.10 N m just before it goes over centre at 17.5 deg.

Score (best stage reached, from the device's state; 0 for the rest of the run once the aim is lost):
- 0: the housing has turned more than 1.5 deg from its start at any time (its worst turn over the run, about any
  axis). 1.5 deg of mirror turns the reflected view 3 deg: at the rear window, 2.5 m behind the mirror, the view moves
  13 cm, about a third of the window's height; the driver would have to re-aim.
- 0.2: the tab moved 10 deg or more off its day position, the aim kept.
- 0.5: the tab latched in night (30 deg or more off day: over centre and at its stop), the aim kept.
- 1.0 (success, held 1 s): latched in night, the aim kept, every finger point 1 cm or more from the housing and the
  tab, and the mirror still (housing and tab each turn less than 0.05 deg in 1 s).
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import BoxShape, CylShape, Link, Linkage, SphereShape, allow_matrix_grips  # noqa: E402
from bodykit import as_numpy, on_reset, on_step, plate, rotation, segment  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001
DEG = math.pi / 180.0

# -- the layout ----------------------------------------------------------------------------------------------------
C = np.array([0.255, 0.050, TOP + 0.240])  # the housing's middle
HD, HL, HH = 0.035, 0.254, 0.064  # housing depth (x), length (y), height (z)
X_GLASS = float(C[0] - HD / 2)  # the glass face
HB = float(C[2] - HH / 2)  # the housing's bottom edge
TAB_W, TAB_L, TAB_T = 0.028, 0.022, 0.005
TAB_DAY = 15.0 * math.pi / 180.0  # the tab's day position: down and this far toward the robot
HOUSING_KG, TAB_KG = 0.18, 0.004


def _balance_x() -> float:
    """x of the housing-and-tab's centre of mass (uniform density over the housing's solid shapes): the ball sits
    straight above it, so the hanging housing puts no torque on the ball."""
    parts = [(HD * HL * HH, C[0]), (0.0012 * (HL - 0.010) * (HH - 0.010), C[0] - HD / 2 - 0.0006),
             (math.pi * 0.013 ** 2 * 0.008, C[0] + HD / 2 + 0.004), (2 * (HD - 0.006) * 0.004 * (HH - 0.008), C[0])]
    xh = sum(v * x for v, x in parts) / sum(v for v, _ in parts)
    xt = C[0] - HD / 2 + 0.003 - math.sin(TAB_DAY) * TAB_L / 2
    return float((HOUSING_KG * xh + TAB_KG * xt) / (HOUSING_KG + TAB_KG))


BALL = np.array([_balance_x(), C[1], C[2] + 0.006])  # the ball joint's centre (inside the housing, over its CG)
MOUNT = BALL + np.array([0.040, 0.0, 0.060])  # the wedge mount's middle, on the button
RAKE = 30.0 * DEG  # the windscreen's angle from the horizontal
N_OUT = np.array([math.sin(RAKE), 0.0, math.cos(RAKE)])  # the glass's normal, out of the car (up and forward)
UPSLOPE = np.array([-math.cos(RAKE), 0.0, math.sin(RAKE)])  # along the glass, up toward the robot
SURF = MOUNT + 0.010 * N_OUT  # a point on the glass's inside face (the mount is 10 mm thick)
GLASS_T = 0.005
PANE_W = 0.36
PANE_LO, PANE_HI = -0.26, 0.12  # the pane's extent along UPSLOPE from SURF

BALL_NM = float(os.environ.get("MIRROR_BALL", "0.07"))  # the ball joint's Coulomb friction about each axis (N m)
BALL_LIMIT = 25.0  # deg each way
GIMBAL_KG = float(os.environ.get("MIRROR_GMASS", "0.010"))  # the ball's two inner axes (the ball and its socket ring)

PIVOT = np.array([X_GLASS + 0.003, C[1], HB])  # the tab's hinge axis (along y)
TAB_TRAVEL = 35.0  # deg from day to night (toward the robot: positive about +y)
TAB_MID = TAB_TRAVEL / 2
TAB_PEAK = float(os.environ.get("MIRROR_TAB_PEAK", "0.10"))  # N m just before it goes over centre
TAB_PRE = float(os.environ.get("MIRROR_PRE", "0"))  # deg: each end's spring target lies this far past its stop
TAB_K = TAB_PEAK / ((TAB_MID + TAB_PRE) * DEG)  # N m/rad
TAB_DIR0 = np.array([-math.sin(TAB_DAY), 0.0, -math.cos(TAB_DAY)])  # pivot -> tab tip, day

BLACK, GLASS_C, PANE_C, FRIT, DASH, STEM, BRACKET = (
    (0.06, 0.06, 0.07), (0.70, 0.74, 0.80), (0.55, 0.68, 0.72, 0.0), (0.03, 0.03, 0.03), (0.22, 0.22, 0.24),
    (0.10, 0.10, 0.11), (0.35, 0.35, 0.37))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _rot_y(a: float) -> np.ndarray:
    """Right-handed rotation about +y (a positive angle moves a point below the axis toward -x)."""
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _mirror() -> Linkage:
    o = BALL  # the linkage origin: the ball's centre
    rel = lambda p: _v(np.asarray(p, float) - o)  # noqa: E731
    up, n = UPSLOPE, N_OUT
    side = np.array([0.0, 1.0, 0.0])
    # the windscreen section and its stand
    mid_s = (PANE_LO + PANE_HI) / 2
    pc = SURF + mid_s * up + GLASS_T / 2 * n
    lo_edge = SURF + PANE_LO * up
    hi_edge = SURF + PANE_HI * up
    base = [plate(rel(lo_edge + GLASS_T / 2 * n), rel(hi_edge + GLASS_T / 2 * n), PANE_W, GLASS_T, n, PANE_C)]
    for sgn in (-1.0, 1.0):  # the black ceramic band along the side edges (on the inside face)
        e = side * sgn * (PANE_W / 2 - 0.015)
        base.append(plate(rel(lo_edge + e - 0.0006 * n), rel(hi_edge + e - 0.0006 * n), 0.03, 0.001, n, FRIT, False))
    for s_pos in (PANE_LO + 0.015, PANE_HI - 0.015):  # and along the top and bottom edges
        q = SURF + s_pos * up - 0.0006 * n
        base.append(plate(rel(q - side * PANE_W / 2), rel(q + side * PANE_W / 2), 0.03, 0.001, n, FRIT, False))
    # the dashboard block under the lower edge
    dz = float(lo_edge[2] - TOP)
    base.append(BoxShape(rel((0.56, C[1], TOP + dz / 2)), (0.12, PANE_W + 0.02, dz), DASH))
    # two side brackets under the far half of the glass, from the dashboard up to the glass
    for sgn in (-1.0, 1.0):
        y = C[1] + sgn * (PANE_W / 2 - 0.02)
        a = SURF + (PANE_LO + 0.02) * up - 0.004 * n
        b = SURF + (PANE_LO + 0.17) * up - 0.004 * n
        a[1] = b[1] = y
        foot = np.array([b[0], y, lo_edge[2]])
        base += [segment(rel(a), rel(b), 0.012, BRACKET), segment(rel(foot), rel(b), 0.012, BRACKET)]
        base.append(segment(rel(np.array([b[0], y, TOP])), rel(foot), 0.012, BRACKET))
    # the button bonded to the glass, the wedge mount on it, and the stem down to the ball
    base.append(CylShape(rel(SURF - 0.0015 * n), 0.011, 0.003, BLACK, collide=False))
    base.append(plate(rel(MOUNT - 0.016 * up), rel(MOUNT + 0.016 * up), 0.024, 0.010, n, STEM))
    base.append(segment(rel(MOUNT - 0.003 * n), rel(BALL + np.array([0.012, 0.0, 0.006])), 0.010, STEM))
    base.append(segment(rel(BALL + np.array([0.012, 0.0, 0.006])), rel(BALL), 0.008, STEM))
    # the ball joint as three axes through the ball's centre, each with the same Coulomb friction
    lim = (-BALL_LIMIT, BALL_LIMIT)
    links = [Link("ball_yaw", (SphereShape(rel(BALL), 0.0075, STEM),), mass=GIMBAL_KG, anchor=_v(BALL), axis="Z",
                  friction=BALL_NM, damping=0.002, limits=lim),
             Link("ball_pitch", (SphereShape(rel(BALL), 0.0070, STEM),), mass=GIMBAL_KG, anchor=_v(BALL), axis="Y",
                  parent="ball_yaw", friction=BALL_NM, damping=0.002, limits=lim)]
    # the housing: shell, prism glass, the socket boss on the back round the ball
    housing = [BoxShape(rel(C), (HD, HL, HH), BLACK),
               BoxShape(rel((X_GLASS - 0.0006, C[1], C[2])), (0.0012, HL - 0.010, HH - 0.010), GLASS_C),
               CylShape(rel((C[0] + HD / 2 + 0.004, C[1], BALL[2])), 0.013, 0.008, BLACK, axis="X")]
    for sgn in (-1.0, 1.0):  # the ends' rounded caps (a step in)
        housing.append(BoxShape(rel((C[0], C[1] + sgn * (HL / 2 + 0.002), C[2])), (HD - 0.006, 0.004, HH - 0.008),
                                BLACK))
    links.append(Link("housing", tuple(housing), mass=HOUSING_KG, anchor=_v(BALL), axis="X", parent="ball_pitch",
                      friction=BALL_NM, damping=0.002, limits=lim))
    tip = PIVOT + TAB_DIR0 * TAB_L
    tab_n = np.array([math.cos(TAB_DAY), 0.0, -math.sin(TAB_DAY)])
    tab = [plate(rel(PIVOT), rel(tip), TAB_W, TAB_T, tab_n, BLACK),
           CylShape(rel(PIVOT), 0.0035, TAB_W + 0.002, BLACK, axis="Y"),
           CylShape(rel(tip), TAB_T / 2, TAB_W, BLACK, axis="Y")]
    links.append(Link("tab", tuple(tab), mass=TAB_KG, anchor=_v(PIVOT), axis="Y", parent="housing", friction=0.002,
                      damping=0.0002, limits=(0.0, TAB_TRAVEL), stiffness=TAB_K, spring_damping=0.0005))
    return Linkage.build("mirror", pos=_v(o), mass=5.0, color=BLACK, shapes=base, links=links, friction=(0.8, 0.7),
                         contact_offset=0.0015, solver_iterations=64)


MIRROR = _mirror()
OBJECTS = [MIRROR]

# -- tracking and scoring -------------------------------------------------------------------------------------------
AIM_DEG = 1.5  # the housing turned more than this: the aim is lost
MOVED_DEG = 10.0  # the tab this far off day: partly pulled
LATCH_DEG = 30.0  # the tab this far off day: over centre and at its night stop
CLEAR = 0.010  # every finger point this far from the housing and the tab
STILL_STEPS, STILL_DEG = 20, 0.05
_DEBUG = os.environ.get("MIRROR_DEBUG") == "1"
_DEBUG_EVERY = int(os.environ.get("MIRROR_DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


def _box_surface(centre, size, step=0.005) -> np.ndarray:
    c, h = np.asarray(centre, float), np.asarray(size, float) / 2
    axes = [np.linspace(-h[i], h[i], max(2, int(round(2 * h[i] / step)) + 1)) for i in range(3)]
    pts = []
    for i in range(3):
        j, k = [a for a in range(3) if a != i]
        for s in (-h[i], h[i]):
            for u in axes[j]:
                for w in axes[k]:
                    p = np.zeros(3)
                    p[i], p[j], p[k] = s, u, w
                    pts.append(p)
    return np.array(pts) + c


HOUSING_SAMPLES = _box_surface(C - BALL, (HD, HL, HH))  # housing frame (origin at the ball, world-aligned at start)
TAB_SAMPLES = np.array([PIVOT - BALL + TAB_DIR0 * s + np.array([0.0, y, 0.0])
                        for s in np.linspace(0.0, TAB_L, 6) for y in np.linspace(-TAB_W / 2, TAB_W / 2, 5)])


@on_reset
def _reset() -> None:
    _EP.pop("r0", None)
    _EP.update(tick=0, step=0, counted=-1, failed="", turn=0.0, worst=0.0, tab=0.0, tab_max=0.0, night=False,
               near=1.0, recent=[], gaps={}, tcp={}, stage=0.0)


_reset()


def _finger_points() -> dict:
    import struct
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = pathlib.Path(YAM_URDF).parent
    out = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in ("tip_left", "tip_right"):
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]) if o is not None else np.zeros(3)
            b = (root / col.find("geometry").find("mesh").get("filename")).read_bytes()
            n = struct.unpack("<I", b[80:84])[0]
            tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            v = tri["v"].reshape(-1, 3).astype(float)
            pts.append(np.unique(np.round(v, 4), axis=0) @ rpy_matrix(rpy).T + xyz)
        out[name] = np.concatenate(pts)
    return out


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("MIRROR_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    import torch
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[side + "_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        q = as_numpy(robot.data.joint_pos)[0]
        tcp_p, tcp_r = ik.tcp_pose(q[_IDS[side]], station.BASES[side])
        _EP["tcp"][side] = np.asarray(tcp_p, float)
        _EP.setdefault("tcp_r", {})[side] = np.asarray(tcp_r, float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers[side] = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                        for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    mir = base.scene["mirror"]
    if "bodies" not in _IDS:
        names, jn = list(mir.body_names), list(mir.joint_names)
        _IDS["bodies"] = {k: names.index(k) for k in ("housing", "tab")}
        _IDS["tab_j"] = jn.index("tab_turn")
    bp = as_numpy(mir.data.body_link_pose_w)[0]
    hp, hr = bp[_IDS["bodies"]["housing"]][:3] - origin, rotation(bp[_IDS["bodies"]["housing"]][3:7])
    tp, tr = bp[_IDS["bodies"]["tab"]][:3] - origin, rotation(bp[_IDS["bodies"]["tab"]][3:7])
    # the housing's turn from its start (its body frame starts world-aligned)
    if "r0" not in _EP:
        _EP["r0"] = hr.copy()
    turn = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(_EP["r0"].T @ hr)) - 1.0) / 2.0))))
    tab = math.degrees(float(as_numpy(mir.data.joint_pos)[0][_IDS["tab_j"]]))
    # the over-centre spring: it pulls toward whichever end the tab is nearer (past that end's stop, so it preloads)
    night = tab > TAB_MID
    target = (TAB_TRAVEL + TAB_PRE) if night else -TAB_PRE
    tgt = torch.tensor([[target * DEG]], dtype=torch.float32, device=base.device)
    mir.set_joint_position_target(tgt, joint_ids=[_IDS["tab_j"]])
    pts = np.concatenate([hp + HOUSING_SAMPLES @ hr.T, tp + TAB_SAMPLES @ tr.T])
    tree = cKDTree(pts)
    near = min(float(tree.query(fp, k=1)[0].min()) for fp in fingers.values())
    if not _EP["failed"] and turn > AIM_DEG:
        _EP["failed"] = f"the housing turned {turn:.2f} deg (over {AIM_DEG} deg): the aim is lost"
    _EP.update(turn=turn, worst=max(_EP["worst"], turn), tab=tab, tab_max=max(_EP["tab_max"], tab), night=night,
               near=near, hp=hp, joints=" ".join(f"{n[:-5]}={math.degrees(float(v)):+.3f}" for n, v in
                                                zip(mir.joint_names, as_numpy(mir.data.joint_pos)[0])))


def _tally(o) -> dict:
    recent = _EP["recent"]
    if _EP["counted"] != _EP["tick"] and "hp" in _EP:
        _EP["counted"] = _EP["tick"]
        recent.append((_EP["turn"], _EP["tab"]))
        del recent[:-STILL_STEPS]
    still = len(recent) == STILL_STEPS and all(abs(a - recent[-1][0]) < STILL_DEG and abs(b - recent[-1][1]) < STILL_DEG
                                               for a, b in recent)
    latched = _EP["tab"] >= LATCH_DEG
    moved = _EP["tab_max"] >= MOVED_DEG
    done = bool(latched and _EP["near"] >= CLEAR and still)
    return dict(still=still, latched=latched, moved=moved, done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    print(f"DBG step={_EP['step']} tab={_EP['tab']:.2f}deg max={_EP['tab_max']:.1f} turn={_EP['turn']:.3f}deg "
          f"worst={_EP['worst']:.3f} near={_EP['near'] * 1000:.0f}mm still={int(t['still'])} "
          f"failed={_EP['failed'] or '-'} [{_EP.get('joints', '')}] | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{v[2] - TOP:.3f}) gap={_EP['gaps'][k] * 1000:.0f} "
                     f"ap=({_EP['tcp_r'][k][0, 2]:+.2f},{_EP['tcp_r'][k][1, 2]:+.2f},{_EP['tcp_r'][k][2, 2]:+.2f})"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    """0 once the aim is lost; 0.2 tab moved 10 deg; 0.5 latched in night. Full score is success()."""
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    if _EP["failed"]:
        return 0.0
    if t["latched"]:
        return 0.5
    if t["moved"]:
        return 0.2
    return 0.0


def success(objects) -> bool:
    t = _tally(objects)
    return bool(t["done"] and not _EP["failed"])


# -- scripted demo --------------------------------------------------------------------------------------------------
# Open loop, from the known layout. The right hand comes in level from the right, pointing +y, fingers closing along x,
# and takes the housing's right end between its glass and its back. Then the left hand comes in level, pointing +x,
# fingers closing along y, takes the tab by its sides near its tip, and turns it about its hinge toward the robot over
# 6 s, the hand turning with it, aimed 40 deg round (the arm turns short under load; past centre the spring takes the
# tab on to its night stop), lets go and backs away along its fingers. Last the right hand opens and backs away. MIRROR_ONE=1 (the wrong way): the right hand stays at its ready pose, so nothing
# steadies the housing.
OPEN_C = -0.047
ONE = os.environ.get("MIRROR_ONE") == "1"


def _frame(xc, zc) -> np.ndarray:
    x, z = np.array(xc, float), np.array(zc, float)
    return np.column_stack([x, np.cross(z, x), z])


R_STEADY = _frame((1, 0, 0), (0, 1, 0))  # right hand: pointing +y, fingers closing along x
R_TAB = _frame((0, -1, 0), (1, 0, 0))  # left hand: pointing +x, fingers closing along y
END_IN = float(os.environ.get("MIRROR_END_IN", "0.040"))  # the right TCP this far in from the housing's end
Y_STEADY = float(C[1] - HL / 2 + END_IN)
STEADY_PINCH = -(HD / 2 - 0.004)
TAB_PINCH = -float(os.environ.get("MIRROR_TAB_PINCH", "0.006"))
PINCH_FROM_TIP = 0.004
TAB_TCP0 = PIVOT + TAB_DIR0 * (TAB_L - PINCH_FROM_TIP)
PULL_S = float(os.environ.get("MIRROR_PULL_S", "6.0"))  # s
PULL = float(os.environ.get("MIRROR_PULL", "40"))  # deg the left hand turns the tab


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0, steps: int = 15) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, p, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _stay(arm: str, t: float = 120.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def _steady_path() -> list:
    at = np.array([C[0], Y_STEADY, C[2]])
    pre = at + np.array([0.0, -0.06, 0.0])
    out = _lead_in("right", pre, R_STEADY, OPEN_C, "beside the housing's right end, open", 4.0)
    out += [_w(2.0, at, OPEN_C, "slide the open fingers over the housing's end", R_STEADY),
            _w(0.5, at, OPEN_C, "settle", R_STEADY),
            _w(1.5, at, STEADY_PINCH, "take the housing's end, glass and back", R_STEADY),
            _w(13.0, at, STEADY_PINCH, "hold the housing steady while the tab is flipped", R_STEADY),
            _w(1.0, at, OPEN_C, "let go", R_STEADY),
            _w(2.0, pre, OPEN_C, "back away to the right", R_STEADY),
            _w(4.0, pre, OPEN_C, "stay clear", R_STEADY)]
    return out


def _tab_path() -> list:
    pre = TAB_TCP0 + np.array([-0.06, 0.0, 0.0])
    side = np.array([0.22, 0.24, pre[2]])  # out to the left of the windscreen, under its upper edge's height
    out = _lead_in("left", side, R_TAB, OPEN_C, "down and back, beside the windscreen", 4.0)
    out += [_w(3.0, pre, OPEN_C, "in under the mirror, in front of the tab", R_TAB),
            _w(1.0, pre, OPEN_C, "wait for the other hand", R_TAB),
            _w(2.0, TAB_TCP0, OPEN_C, "fingers either side of the tab", R_TAB),
            _w(0.5, TAB_TCP0, OPEN_C, "settle", R_TAB),
            _w(1.0, TAB_TCP0, TAB_PINCH, "take the tab by its sides", R_TAB)]
    n = 12
    for k in range(1, n + 1):  # turn it about its hinge toward the robot, the hand turning with it
        ry = _rot_y(PULL * DEG * k / n)
        out.append(_w(PULL_S / n, PIVOT + ry @ (TAB_TCP0 - PIVOT), TAB_PINCH, "pull the tab over toward the robot",
                      ry @ R_TAB))
    ry = _rot_y(PULL * DEG)
    p = PIVOT + ry @ (TAB_TCP0 - PIVOT)
    back = ry @ np.array([-0.06, 0.0, 0.0])
    out += [_w(1.0, p, TAB_PINCH, "settle", ry @ R_TAB),
            _w(1.0, p, OPEN_C, "let go of the tab", ry @ R_TAB),
            _w(2.0, p + back, OPEN_C, "back away along the fingers", ry @ R_TAB),
            _w(2.0, p + back + np.array([-0.03, 0.0, -0.02]), OPEN_C, "stay clear", ry @ R_TAB)]
    return out


IDLE = os.environ.get("MIRROR_IDLE") == "1"


def demo():
    if IDLE:
        return {"left": _stay("left"), "right": _stay("right")}
    return {"left": _tab_path(), "right": _stay("right") if ONE else _steady_path()}
