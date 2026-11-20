"""On the station table: a hardback pocket notebook (Moleskine Classic Large size, 13 x 21 cm, 17 mm thick, red
cloth boards with rounded corners) lies loose, back cover down, its long side along x (head away from the robot), its
fore-edge toward the robot's right (-y) and its spine toward the left (+y). Its front cover stands open: the notebook
is new and its stiff spine springs the cover back to 70 degrees whenever it is let go. Its black elastic closure band
(5 mm wide), sewn into the back cover at the head and at the tail 10 mm in from the fore-edge, lies stretched across
the first page, where it snaps when the cover is opened with the band still on.
Task (instruction): close the notebook and fasten it shut with its elastic band, then let go of it. The instruction
does not say how.

The closure: "Each notebook features an undeniably stylish hard black cover with rounded corners, an elastic closure
& folding pocket for notes" (retailer's description of the Moleskine Classic Large hard cover). Closed, the band runs
from the back cover's tail edge up over the tail, along the front cover 10 mm in from the fore-edge, and down over the
head to the back cover again; its pull holds the cover shut.

Why two hands: the band lies on the pages, under the open cover, so shutting the cover first shuts the band inside the
book; one hand must draw the band out past the fore-edge and hold it there while the cover is shut, and the cover
springs back open the moment nothing holds it, so the other hand must keep it shut until the band is across it. Let
go of the band too early and it snaps back onto the pages or round the fore-edge.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z 0.7475.
- notebook (back board, page block, spine): loose, 0.27 kg; back board 210 x 130 x 2.5 mm (x 0.12 to 0.33, y -0.065
  to 0.065), its fore-edge corners rounded to 8 mm; page block 12 mm thick, 3 mm in from the head, tail and fore-edge;
  front cover 210 x 130 x 2.5 mm, 30 g, on a hinge along the spine (x axis at y 0.0625, 15.75 mm up), opened 70
  degrees at the start and sprung back toward 70 degrees (0.15 N m/rad: 0.18 N m when shut), shut at 0 degrees where
  it rests on the pages.
- elastic band: 5 mm wide, 2 mm thick, black; sewn in at (0.118, -0.055) and (0.332, -0.055), 1.25 mm up (the back
  board's mid-thickness), 2 mm outside the boards' tail and head edges; 226 mm long unstretched, 80 N/m (estimate);
  251 mm and 2.0 N at the start (up the tail and head and across the first page at y -0.055), about 250 mm and
  1.9 N fastened. Simulated as 16 rigid 2 g links: an 18 mm sewn-in root at each end (it leans only toward the fore-edge
  or back, sprung upright at 0.02 N m/rad), then 14 links (6 mm at the ends, 17 mm between) on ball joints that
  stretch along the band.

Score: see ``score``.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Joint, allow_matrix_grips, as_numpy, on_reset, on_step, plate, rotation  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
MM = 0.001


def _env(name: str, default: str) -> str:
    return os.environ.get("NBAND_" + name, default)


# -- the notebook ---------------------------------------------------------------------------------------------------
X0, X1 = 0.12, 0.33  # tail and head edges
YF, YS = -0.065, 0.065  # fore-edge and spine
BT = 0.0025  # board thickness
PT = 0.012  # page block thickness
SQ = 0.003  # the boards stand out past the pages this far at the head, tail and fore-edge
CORNER_R = float(_env("CORNER_R", "0.008"))  # the rounded corners at the fore-edge
PAGE_TOP = TOP + BT + PT
COVER_TOP = PAGE_TOP + BT
XC = (X0 + X1) / 2
HINGE = (XC, YS - BT, PAGE_TOP + BT / 2)  # the front cover's hinge line (along x)
OPEN_DEG = float(_env("OPEN", "70"))  # the cover's rest angle
START_DEG = float(_env("START", str(OPEN_DEG)))  # its angle at the start (a test: 0 starts it shut, band on it)
COVER_K = float(_env("COVER_K", "0.15"))  # N m/rad, springing it back open
BOOK_M = 0.27
COVER_M = 0.030
BOARD = (0.50, 0.07, 0.08)  # red cloth-covered boards
PAGES = (0.93, 0.91, 0.84)
EDGE = (0.80, 0.78, 0.70)

# -- the band -------------------------------------------------------------------------------------------------------
BY = YF + float(_env("BAND_IN", "0.010"))  # the band's line, 10 mm in from the fore-edge
AX0, AX1 = X0 - 0.002, X1 + 0.002  # where it runs up the tail and head edges (just outside the boards)
ANCH_Z = TOP + BT / 2
BW, BTH = 0.005, 0.002  # width, thickness
N_SPAN = 14
SPAN_Z = (PAGE_TOP if START_DEG > 0 else PAGE_TOP + BT) + BTH / 2  # its middle line on the first page (or cover)
RISE = SPAN_Z - ANCH_Z
SEG = (AX1 - AX0) / N_SPAN
L_REST = float(_env("L0", "0.226"))
K_BAND = float(_env("K", "80"))  # N/m, the whole band
LINK_M = float(_env("LINK_M", "0.002"))
N_STRETCH = N_SPAN + 1  # joints that stretch: between span links (13) and at the roots' tops (2)
BAND_C = (0.08, 0.08, 0.09)
ROOT_K = float(_env("ROOT_K", "0.02"))  # N m/rad: the stitched root springs back upright
RISER_AXIS = _env("RISER_AXIS", "X")  # the band's sewn-in root leans toward the fore-edge or back, not along it
BAND_DAMP = float(_env("BAND_DAMP", "2.0"))  # linear damping of the band's links (air and its own internal friction)
MID_NAME = "b7"  # the span link at the band's middle (b1..b14 span it), where the demo takes it
ROOT_H = PAGE_TOP - 0.0035 - (TOP + BT / 2)  # the sewn-in root's height
RISE_OVER = float(_env("RISE_OVER", "0.002"))
RISE_H = (PAGE_TOP + BT + RISE_OVER) - (TOP + BT / 2)  # the root stands just above the shut cover's face
END_SEG = 0.006  # the span's first and last links
SPAN_X = ([AX0, AX0 + END_SEG] + list(np.linspace(AX0 + END_SEG, AX1 - END_SEG, N_SPAN - 1)[1:-1])
          + [AX1 - END_SEG, AX1])
_PTS = [(AX0, TOP + BT / 2 + RISE_H)] + [(x, SPAN_Z) for x in SPAN_X[1:-1]] + [(AX1, TOP + BT / 2 + RISE_H)]
L_START = 2 * RISE_H + sum(math.dist(a, b) for a, b in zip(_PTS, _PTS[1:]))  # its path at the start
PRE = -(L_START - L_REST) / N_STRETCH  # each stretching joint's spring target (m)
K_SEG = K_BAND * N_STRETCH
BAND_MU = (float(_env("BAND_MU", "0.30")), float(_env("BAND_MU", "0.30")) - 0.05)  # woven elastic on paper/board


def _v(p) -> tuple:
    return tuple(float(v) for v in p)


def _rounded(x0, x1, y0, y1, z, t, r, color) -> list:
    """A plate x0..x1, y0..y1 (body frame), ``t`` thick about height ``z``, its two corners at y0 rounded to ``r``."""
    return [BoxShape(((x0 + x1) / 2, (y0 + r + y1) / 2, z), (x1 - x0, y1 - y0 - r, t), color),
            BoxShape(((x0 + x1) / 2, y0 + r / 2, z), (x1 - x0 - 2 * r, r, t), color),
            CylShape((x0 + r, y0 + r, z), r, t, color), CylShape((x1 - r, y0 + r, z), r, t, color)]


def _book() -> Body:
    hx = (X1 - X0) / 2
    shapes = (_rounded(-hx, hx, YF, YS, BT / 2, BT, CORNER_R, BOARD)  # back board, its fore-edge corners rounded
              + _rounded(-hx + SQ, hx - SQ, YF + SQ, YS - BT, BT + PT / 2, PT, CORNER_R - SQ, PAGES)  # page block
              + [BoxShape((0.0, YS - BT / 2, (BT + PT) / 2 + 0.0), (X1 - X0, BT, BT + PT), BOARD)])  # spine
    # a line of page edges drawn on the fore-edge (visual only)
    shapes.append(BoxShape((0.0, YF + SQ - 0.0002, BT + PT / 2), (X1 - X0 - 2 * CORNER_R, 0.0004, PT * 0.9), EDGE,
                           collide=False))
    return Body.build("notebook", pos=(XC, 0.0, TOP + 0.0005), mass=BOOK_M, shapes=shapes, friction=(0.45, 0.4),
                      damping=(0.0, 0.05), solver_iterations=32)


def _cover() -> Body:
    a = math.radians(START_DEG)
    w = YS - YF - BT  # from the hinge to the free edge
    # the cover's middle, opened ``a`` about the hinge (x axis): its free edge rises toward -y and up
    mid = np.array(HINGE) + np.array([0.0, -w / 2 * math.cos(a), w / 2 * math.sin(a)])
    q = (math.sin(-a / 2), 0.0, 0.0, math.cos(-a / 2))
    hx, hy = (X1 - X0) / 2, (w + BT) / 2
    shapes = _rounded(-hx, hx, -hy, hy, 0.0, BT, CORNER_R, BOARD)  # its free edge at -y, corners rounded
    return Body.build("cover", pos=_v(mid), mass=COVER_M, shapes=shapes, init_quat=q, friction=(0.45, 0.4),
                      joint=Joint(anchor=_v(HINGE), kind="revolute", axis="X", parent="notebook",
                                  limits=(START_DEG - 95.0, START_DEG), stiffness=COVER_K,
                                  target=START_DEG - OPEN_DEG, damping=0.002),
                      solver_iterations=32)


def _band() -> list:
    """Tail root b0, span links b1..b14 (tail to head), head root b15. Each root is the band's sewn-in end: it stands
    up the book's tail (or head) from the anchor in the back board to 2 mm above the shut cover, can lean only toward
    the fore-edge or back (a pin along x through the anchor) and springs back upright (the stitching's stiffness).
    The span's end links are short, so the band can bend over the cover's head and tail edges. Every joint after a
    root turns freely and stretches along the band on a spring."""
    out = []
    kw = dict(friction=BAND_MU, damping=(BAND_DAMP, 0.5), solver_iterations=32, contact_offset=0.0015)
    stretch = dict(kind="ball", axial=(K_SEG, 0.4, 0.0), slide_range=(-0.004, 0.025), slide_target=PRE)
    root = dict(kind="revolute", axis=RISER_AXIS, parent="notebook", stiffness=ROOT_K, damping=0.0005)
    z_top = ANCH_Z + RISE_H

    def riser(name, x, joint, extra=()):
        return Body.build(name, pos=(x, BY, ANCH_Z + RISE_H / 2), mass=LINK_M, color=BAND_C,
                          shapes=[BoxShape((0.0, 0.0, 0.0), (BTH, BW, RISE_H * 0.92), BAND_C)], joint=joint,
                          extra_joints=extra, **kw)

    def link(name, a, b, joint):
        a, b = np.array(a), np.array(b)
        d = b - a
        shape = plate(-d / 2 * 0.92, d / 2 * 0.92, BW, BTH, (-d[2], 0.0, d[0]), BAND_C)
        return Body.build(name, pos=_v((a + b) / 2), mass=LINK_M, color=BAND_C, shapes=[shape], joint=joint, **kw)

    out.append(riser("b0", AX0, Joint(anchor=(AX0, BY, ANCH_Z), **root)))
    pts = [(AX0, BY, z_top)] + [(x, BY, SPAN_Z) for x in SPAN_X[1:-1]] + [(AX1, BY, z_top)]
    for i in range(N_SPAN):
        par = "b0" if i == 0 else f"b{i}"
        out.append(link(f"b{i + 1}", pts[i], pts[i + 1],
                        Joint(anchor=pts[i], parent=par, ball_axis="Z" if i == 0 else "X", **stretch)))
    out.append(riser(f"b{N_SPAN + 1}", AX1, Joint(anchor=(AX1, BY, z_top), parent=f"b{N_SPAN}", ball_axis="X",
                                                  **stretch),
                     (Joint(anchor=(AX1, BY, ANCH_Z), **root),)))
    return out


BOOK = _book()
COVER = _cover()
BAND = _band()
BAND_NAMES = [b.name for b in BAND]
OBJECTS = [BOOK, COVER] + BAND

# -- tracking and score ---------------------------------------------------------------------------------------------
SHUT_DEG = 3.0  # the cover within this of the pages counts as shut
ON_H = 0.004  # its middle at most this above the cover's top face
IN_FROM_EDGE = 0.003  # and at least this far in from the cover's fore-edge
CLEAR = 0.03  # grippers at least this far from the band and the notebook
STILL_MM = 2.0  # everything moved less than this over the last second
HOLD_STEPS = 20  # 1 s at 20 Hz
BOOK_TILT_DEG = 5.0
_DEBUG = _env("DEBUG", "0") == "1"
_DEBUG_EVERY = int(_env("DEBUG_EVERY", "5"))
_EP: dict = {}
_IDS: dict = {}


@on_reset
def _reset() -> None:
    _EP.update(step=0, tick=0, tcp={}, gaps={}, fingers={}, hist=[], vmax=(0.0, ""))


_reset()


def _finger_points() -> dict:
    """Each fingertip link's collision-mesh vertices (link frame), from the YAM URDF."""
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

        print("NBAND_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _track_inner(env) -> None:
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[side + "_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        if "hull" not in _IDS:
            _IDS["hull"] = _finger_points()
        _EP["fingers"][side] = np.concatenate([_IDS["hull"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                               for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    _EP["tick"] = _EP.get("tick", 0) + 1
    pos = []
    for n in ["notebook", "cover"] + BAND_NAMES:
        pos.append(as_numpy(base.scene[n].data.root_link_pose_w)[0][:3] - origin)
    _EP["hist"].append(np.array(pos))
    del _EP["hist"][:-HOLD_STEPS - 1]


def _cover_angle(o) -> float:
    """The front cover's opening angle (deg) relative to the back board: 0 shut."""
    rb, rc = o["notebook"].rotation(), o["cover"].rotation()
    rel = rb.T @ rc
    return math.degrees(math.atan2(rel[2, 1], rel[1, 1]))


def _tally(o) -> dict:
    book, cover = o["notebook"], o["cover"]
    ang = abs(_cover_angle(o))
    shut = ang <= SHUT_DEG
    tilt = math.degrees(book.tilt())
    on_desk = tilt <= BOOK_TILT_DEG and float(book.low[2]) <= TOP + 0.003
    mid = o[MID_NAME]
    m_local = cover.to_local(mid.pos)  # in the cover's frame (its middle; y across, z up through its face)
    w = YS - YF - BT
    on_face = (m_local[2] <= BT / 2 + BTH / 2 + ON_H and m_local[2] >= 0.0
               and m_local[1] >= -(w + BT) / 2 + IN_FROM_EDGE and abs(m_local[0]) <= (X1 - X0) / 2)
    band_on = shut and on_face
    # around the edge: the middle link beyond the cover's fore-edge (outside the book), the cover shut
    edge = shut and not band_on and m_local[1] < -(w + BT) / 2 + IN_FROM_EDGE and m_local[2] > -(BT + PT + 0.004)
    near = 1.0
    fingers = [f for side in ("left", "right") for f in [_EP["fingers"].get(side)] if f is not None and len(f)]
    if fingers:
        F = np.concatenate(fingers)
        for spec in OBJECTS:  # distance from every finger point to each body's box (in the body's frame)
            st = o[spec.name]
            lo, hi = spec.local_bounds()
            loc = (F - st.pos) @ st.rotation()
            gap = np.maximum(np.maximum(lo - loc, 0.0), loc - hi)
            dn = float(np.linalg.norm(gap, axis=1).min())
            if dn < near:
                near, _EP["near_to"] = dn, spec.name
    h = _EP["hist"]
    still = False
    if len(h) > HOLD_STEPS:
        moved = np.linalg.norm(np.array(h) - h[-1], axis=-1).max(axis=0)  # per body, over the last second
        k = int(np.argmax(moved))
        _EP["vmax"] = (float(moved[k]) * 1000, (["notebook", "cover"] + BAND_NAMES)[k])
        still = float(moved[k]) * 1000 <= STILL_MM
    released = near >= CLEAR
    return dict(ang=ang, shut=shut, tilt=tilt, on_desk=on_desk, on_face=on_face, band_on=band_on,
                edge=edge, near=near, still=still, released=released, m_local=m_local)


def _debug(o, t, s) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    mids = " ".join(f"{n}=({o[n].pos[0]:.3f},{o[n].pos[1]:.3f},{(o[n].pos[2] - TOP) * 1000:.1f})"
                    for n in ("b0", "b3", MID_NAME, "b12", f"b{N_SPAN + 1}"))
    tc = " ".join(f"{k[0]}tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.0f})g{_EP['gaps'].get(k, 0) * 1000:.0f}"
                  for k, v in _EP["tcp"].items())
    if _env("DEBUG_BAND", "0") == "1":
        rel = [o["notebook"].to_local(o[n].pos) for n in BAND_NAMES]
        print("DBGBAND step=%d " % _EP["step"] + " ".join(f"{n}:{r[1] * 1000:.0f},{(r[2]) * 1000:.0f}"
                                                         for n, r in zip(BAND_NAMES, rel)), flush=True)
    print(f"DBG step={_EP['step']} score={s:.2f} ang={t['ang']:.1f} onface={t['on_face']} "
          f"edge={t['edge']} mloc=({t['m_local'][0]:.3f},{t['m_local'][1]:.3f},{t['m_local'][2] * 1000:.1f}) "
          f"book=({o['notebook'].pos[0]:.3f},{o['notebook'].pos[1]:.3f}) tilt={t['tilt']:.1f} "
          f"near={t['near']:.3f}:{_EP.get('near_to', '')} still={t['still']} mv={_EP['vmax'][0]:.1f}mm:{_EP['vmax'][1]} {mids} {tc}", flush=True)


def score(objects) -> float:
    """1.0: the front cover shut (within 3 deg of the pages), the band's middle link on the front cover's face (above
    it by at most 4 mm, and at least 3 mm in from the cover's fore-edge), the notebook flat on the desk (tilted under
    5 deg), every finger point 3 cm or more from the notebook, its cover and the band, and nothing moving more than
    2 mm over the last second. 0.3: the same, but the band's middle not on the cover (round its fore-edge, say). 0
    otherwise (the cover springs open when nothing holds it). Nothing is latched: the score is the state at the end."""
    t = _tally(objects)
    s = 0.0
    if t["shut"] and t["on_desk"] and t["released"] and t["still"]:
        s = 1.0 if t["band_on"] else 0.3
    if _DEBUG:
        _debug(objects, t, s)
    return s


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- scripted demo --------------------------------------------------------------------------------------------------
OPEN_C = -0.047
CLOSED = 0.0
MODE = _env("MODE", "demo")  # demo | one (wrong way: one hand shuts the cover over the band and lets go)
PINCH = float(_env("PINCH", "-0.0005"))  # fingers commanded 1 mm apart round the 5 mm band
GRAB_X = float(_env("GRAB_X", "0.2175"))
TIP = 0.0125  # fingertips this far below the TCP
DOWN_R = grip_rotation(0.0)  # right hand straight down, fingers closing along y (across the band)
DOWN_L = grip_rotation(0.0)
TILT = float(_env("TILT", "40"))  # deg: the right hand leans out (wrist toward -y) while it holds the band out


def _rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


LEAN_R = _rot_x(TILT) @ DOWN_R


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


def _stay(arm: str, t: float = 200.0) -> list:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return [Waypoint(t, tuple(float(v) for v in pos), OPEN_C, "stay put", rot)]


def P(x, y, z) -> np.ndarray:
    """A TCP target: z is the fingertips' height above the table."""
    return np.array([x, y, TOP + z + TIP])


PULL_Y = float(_env("PULL_Y", "-0.100"))
PULL_Z = float(_env("PULL_Z", "0.004"))
PRESS_X = float(_env("PRESS_X", "0.305"))
PRESS_R = float(_env("PRESS_R", "0.035"))  # the presser pushes the cover this far from the hinge
T_PULL = 13.2  # s: the right hand has the band out past the fore-edge by then (its path below)
T_SHUT = float(_env("T_SHUT", "8.0"))  # s the left hand takes to shut the cover
T_CARRY = 2.5 + 2.0 + 2.5 + 2.0 + 1.0 + 1.5  # s: the right hand lifts the band over, lays it, lets go and rises
OVER = float(_env("OVER", "0.035"))  # m: laid this far past the band's line toward the spine, so its ends come over too
LIFT = float(_env("LIFT", "0.060"))  # m above the cover: high enough that the band's ends ride over its corners


def _cover_point(deg: float, r: float, out: float = 0.0) -> np.ndarray:
    """A point on the cover's outer face ``r`` from the hinge with the cover open ``deg`` (``out`` above the face)."""
    a = math.radians(deg)
    h = np.array(HINGE)
    along = np.array([0.0, -math.cos(a), math.sin(a)])
    normal = np.array([0.0, math.sin(a), math.cos(a)])
    return h + along * r + normal * (BT / 2 + out)


def PL(x, y, z, lean: float = TILT) -> np.ndarray:
    """A TCP target for the right hand leaning ``lean`` deg: the fingertips' middle at (x, y, z above the table)."""
    d = _rot_x(lean) @ np.array([0.0, 0.0, -1.0])  # the fingers' direction
    return np.array([x, y, TOP + z]) - TIP * d


def _right() -> list:
    z_band = PAGE_TOP - TOP  # the page's surface
    zc = COVER_TOP - TOP  # the shut cover's face
    out = _lead_in("right", P(GRAB_X, BY, z_band + 0.05), DOWN_R, OPEN_C, "above the band, open")
    out += [_w(1.0, P(GRAB_X, BY, z_band + 0.05), -0.012, "narrow the fingers", DOWN_R),
            _w(2.0, P(GRAB_X, BY, z_band - 0.002), -0.012, "fingertips down onto the page astride the band", DOWN_R),
            _w(1.2, P(GRAB_X, BY, z_band - 0.002), PINCH, "pinch the band", DOWN_R),
            _w(1.5, P(GRAB_X, BY, z_band + 0.02), PINCH, "lift it off the page", DOWN_R),
            _w(2.5, PL(GRAB_X, PULL_Y, PULL_Z), PINCH, "draw it out past the fore-edge, leaning out", LEAN_R),
            _w(2.0, PL(GRAB_X, PULL_Y, PULL_Z), PINCH, "wait: the band's ends slide off the page", LEAN_R)]
    t_used = sum(wp.duration for wp in out)
    hold = max(0.5, _left_path()[1] + 1.0 - t_used)
    out += [_w(hold, PL(GRAB_X, PULL_Y, PULL_Z), PINCH, "hold it out while the cover is shut", LEAN_R),
            _w(2.5, PL(GRAB_X, PULL_Y + 0.02, zc + LIFT), PINCH, "stretch it straight up beside the fore-edge",
               LEAN_R),
            _w(2.0, PL(GRAB_X, BY + OVER, zc + LIFT), PINCH, "carry it over the cover", LEAN_R),
            _w(2.5, PL(GRAB_X, BY + OVER, zc + 0.002), PINCH, "lay it on the cover", LEAN_R),
            _w(2.0, PL(GRAB_X, BY, zc + 0.002), PINCH, "slide it back to its line on the cover", LEAN_R),
            _w(1.0, PL(GRAB_X, BY, zc + 0.002), -0.012, "let go", LEAN_R),
            _w(1.5, PL(GRAB_X, BY - 0.03, zc + 0.06), -0.012, "rise and lean away", LEAN_R),
            _w(30.0, P(0.15, -0.20, 0.15), OPEN_C, "stay clear", DOWN_R)]
    return out


def _shut(arm: str, x: float, depth: float = 0.004, r: float | None = None) -> list:
    """Come over the open cover's back and push it shut along its arc, ``depth`` into the shut cover's face."""
    rot = DOWN_L if arm == "left" else DOWN_R
    r = PRESS_R if r is None else r
    p0 = _cover_point(OPEN_DEG, r, 0.012)
    p0[0] = x
    out = [_w(2.0, p0 + np.array([0.0, 0.03, 0.05 + TIP]), CLOSED, "above the open cover's back", rot),
           _w(1.5, p0 + np.array([0.0, 0.0, TIP]), CLOSED, "against the cover's back", rot)]
    n = 10
    for k in range(1, n + 1):
        deg = OPEN_DEG * (1 - k / n)
        p = _cover_point(deg, r, -depth * k / n)
        p[0] = x
        out.append(_w(T_SHUT / n, p + np.array([0.0, 0.0, TIP]), CLOSED, "push the cover shut", rot))
    return out


def _left_path() -> tuple:
    """The left hand's waypoints up to the cover shut, the time (s) that takes, and where it then holds the cover."""
    p_up = _cover_point(OPEN_DEG, PRESS_R, 0.012)
    p_up[0] = PRESS_X
    above = p_up + np.array([0.0, 0.03, 0.05 + TIP])
    out = _lead_in("left", above, DOWN_L, CLOSED, "above the open cover")
    out.append(_w(max(0.5, T_PULL - 3.0 - 1.5), above, CLOSED, "wait for the band to be out", DOWN_L))
    out += _shut("left", PRESS_X, PRESS_DEPTH)[1:]
    t_shut = sum(wp.duration for wp in out)
    pin = _cover_point(0.0, PRESS_R, -PRESS_DEPTH)
    pin[0] = PRESS_X
    return out, t_shut, pin


PRESS_DEPTH = float(_env("PRESS_DEPTH", "0.010"))  # the left hand is told this far into the shut cover


def _left() -> list:
    out, t_shut, pin = _left_path()
    out += [_w(T_CARRY + 2.0, pin + np.array([0.0, 0.0, TIP]), CLOSED, "hold the cover shut", DOWN_L),
            _w(1.5, pin + np.array([0.0, 0.0, TIP + 0.06]), CLOSED, "let go of the cover: rise", DOWN_L),
            _w(30.0, P(0.20, 0.20, 0.15), OPEN_C, "stay clear", DOWN_L)]
    return out


def demo():
    if MODE == "one":  # wrong way: one hand shuts the cover with the band still on the pages, then lets go
        out, t_shut, pin = _left_path()
        out += [_w(3.0, pin + np.array([0.0, 0.0, TIP]), CLOSED, "hold the cover shut", DOWN_L),
                _w(1.5, pin + np.array([0.0, 0.0, TIP + 0.06]), CLOSED, "let go of the cover: rise", DOWN_L),
                _w(20.0, P(0.20, 0.20, 0.15), OPEN_C, "stay clear", DOWN_L)]
        return {"right": _stay("right"), "left": out}
    return {"right": _right(), "left": _left()}
