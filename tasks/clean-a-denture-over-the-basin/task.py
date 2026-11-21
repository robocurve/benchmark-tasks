"""Brushing a patient's upper denture at the bedside sink station. A complete upper denture lies teeth-up on a folded
towel at the left, a long-handled denture brush on a towel at the right, a plastic wash basin with water in the middle
and an open denture cup behind the denture's towel.

A denture is cleaned held in one hand and brushed with the other, and always over a basin of water (or a towel-lined
sink): acrylic dentures crack or shatter when dropped on a hard counter or floor, and a broken denture leaves the
patient unable to eat until it is remade. Brushing held over the counter does not count. Then the denture goes into
its cup.

Kits: yamkit (the loose denture and brush, the fixed basin station, gripper states).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Brush the denture's teeth with the denture brush, then put the denture in its cup."

# --- Scene, world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
BASIN = (0.340, -0.020)  # the wash basin's center
BASIN_SIZE = (0.280, 0.240, 0.090)  # outside: 28 x 24 cm, 9 cm deep
BASIN_WALL = 0.008
WATER_Z = 0.045
CUP = (0.300, 0.220)  # the denture cup (a rectangular denture bath, open)
CUP_SIZE = (0.095, 0.085, 0.060)
CUP_WALL = 0.006
TOWEL_D = (0.150, 0.165)  # the denture's towel
TOWEL_D_SIZE = (0.110, 0.100, 0.010)
TOWEL_B = (0.140, -0.130)  # the brush's towel
TOWEL_B_SIZE = (0.070, 0.240, 0.010)

# The denture, in its own frame: origin at the center of the arch's front curve, on the base's underside; teeth up (+z),
# the front teeth toward -y, the heels toward +y. The arch's centerline is a half circle (radius R) in front and two
# straight runs of back teeth behind it; behind the last molar on each side the pink base runs on as a flange (the
# tuberosity), where it is held.
R = 0.022
ARCH_W = 0.011  # the arch's width across
BASE_H, TEETH_H = 0.009, 0.007
HT = BASE_H + TEETH_H  # 1.6 cm: the teeth's tops
LEG_T = 0.020  # back teeth behind the curve, each side
FLANGE = 0.012  # the base behind the last molar
DENTURE0 = (0.150, 0.150)  # on its towel

# The brush, in its own frame: origin at the head's center, the handle along -x, bristles down (-z).
HEAD_L, HEAD_W, HEAD_T = 0.024, 0.012, 0.006
BRISTLE_L, BRISTLE_W, BRISTLE_H = 0.022, 0.010, 0.010
TIP_Z = -HEAD_T / 2 - BRISTLE_H  # the bristle tips
HANDLE_X = (-0.175, -HEAD_L / 2)
HANDLE_W, HANDLE_H, HANDLE_Z = 0.007, 0.014, 0.004
GRIP_X = -0.150  # where the demo holds it, 15 cm behind the head
BRUSH0 = (0.140, -0.050)  # the head's center on its towel; the handle runs toward -y

# --- Scoring.
ZONES = ("front", "far side", "near side")
STROKES = 4  # strokes a zone needs
STROKE_MIN = 0.010  # a stroke: the brush moving 1 cm or more one way across the teeth, bristles on them
TIP_BAND = (-0.0005, 0.0055)  # the tips resting on the teeth: the rigid bristles stop 0.5 to 5 mm above the drawn tops (the colliders' margin), so no tip goes lower and a brush hovering higher is not touching
COVER_STEP = 0.0005  # m: the brush moving at least 0.5 mm in a control step is brushing, not resting
COVER_R = 0.004  # m: a tooth point is brushed when the middle of the bristles passes within 4 mm of it along the brush
UPRIGHT = math.cos(math.radians(30.0))  # teeth up, bristles down, within 30 degrees
GAP_STEPS = 4  # bristles off a zone for under 0.2 s is still the same stroke
DROP_V = 0.5  # m/s downward: falling, not being lowered

PINK = kit.Look((0.86, 0.45, 0.52), 0.35)
TOOTH = kit.Look((0.95, 0.94, 0.88), 0.25)
BASIN_LOOK = kit.Look((0.55, 0.72, 0.85), 0.5)
WATER = kit.Look((0.45, 0.65, 0.85), 0.05, opacity=0.35)
CUP_LOOK = kit.Look((0.92, 0.92, 0.95), 0.4, opacity=0.85)
TOWEL_LOOK = kit.Look((0.30, 0.62, 0.62), 0.95)
BRUSH_LOOK = kit.Look((0.20, 0.45, 0.80), 0.4)
BRISTLE = kit.Look((0.95, 0.95, 0.95), 0.8)


def _arch_path(z, n=24):
    """The arch's centerline at height z, from the near heel round the front to the far heel."""
    pts = [(-R, LEG_T, z)]
    for k in range(n + 1):
        phi = -math.pi / 2 + math.pi * k / n
        pts.append((R * math.sin(phi), -R * math.cos(phi), z))
    pts.append((R, LEG_T, z))
    return pts


def _denture():
    m = [kit.band([(-R, LEG_T + FLANGE, BASE_H / 2)] + _arch_path(BASE_H / 2) + [(R, LEG_T + FLANGE, BASE_H / 2)],
                  BASE_H, ARCH_W, PINK, normal=(0, 0, 1)),
         kit.box((-R, LEG_T + FLANGE / 2, HT / 2), (ARCH_W, FLANGE, HT), PINK),  # the heels: base up to tooth height
         kit.box((R, LEG_T + FLANGE / 2, HT / 2), (ARCH_W, FLANGE, HT), PINK),
         kit.box((0, (LEG_T + FLANGE) / 2 - 0.004, 0.001), (2 * R, LEG_T + FLANGE + 0.008, 0.002), PINK),  # palate
         kit.box((0, -R * 0.45, 0.001), (1.5 * R, R * 0.9, 0.002), PINK)]
    path = _arch_path(BASE_H + TEETH_H / 2, n=60)
    seg = 0.0085  # one tooth's length along the arch
    acc, cur = 0.0, [path[0]]
    for a, b in zip(path, path[1:]):  # teeth: short white blocks along the arch with thin gaps
        acc += math.dist(a, b)
        cur.append(b)
        if acc >= seg:
            if len(cur) > 1:
                m.append(kit.band(cur, TEETH_H, ARCH_W * 0.92, TOOTH, normal=(0, 0, 1)))
            acc, cur = 0.0, []  # the next tooth starts one point on: a thin gap between teeth
    c = [kit.Collider((-R, (LEG_T + FLANGE) / 2, HT / 2), size=(ARCH_W, LEG_T + FLANGE, HT)),
         kit.Collider((R, (LEG_T + FLANGE) / 2, HT / 2), size=(ARCH_W, LEG_T + FLANGE, HT)),
         kit.Collider((0, -R * 0.8, HT / 2), size=(1.4 * R, ARCH_W, HT)),
         kit.Collider((-R * 0.75, -R * 0.55, HT / 2), size=(ARCH_W, 0.45 * R, HT)),
         kit.Collider((R * 0.75, -R * 0.55, HT / 2), size=(ARCH_W, 0.45 * R, HT)),
         kit.Collider((0, (LEG_T + FLANGE - R) / 2, 0.001), size=(2 * R, LEG_T + FLANGE + R, 0.002))]
    return kit.Prop("denture", pos=(*DENTURE0, TOP + TOWEL_D_SIZE[2]), size=(2 * R + ARCH_W, R + LEG_T + FLANGE + ARCH_W, HT),
                    mass=0.03, meshes=tuple(m), colliders=tuple(c))


def _brush():
    hx = (HANDLE_X[0] + HANDLE_X[1]) / 2
    hl = HANDLE_X[1] - HANDLE_X[0]
    m = [kit.box((0, 0, 0), (HEAD_L, HEAD_W, HEAD_T), BRUSH_LOOK),
         kit.box((0, 0, -HEAD_T / 2 - BRISTLE_H / 2), (BRISTLE_L, BRISTLE_W, BRISTLE_H), BRISTLE),
         kit.box((hx, 0, HANDLE_Z), (hl, HANDLE_W, HANDLE_H), BRUSH_LOOK)]
    # The bristles collide over their whole length (rigid: real bristles bend), so the teeth stop the tips and push back
    # on the brush with a real contact force: a stroke needs the bristle tips in contact with the teeth.
    c = [kit.Collider((0, 0, 0), size=(HEAD_L, HEAD_W, HEAD_T)),
         kit.Collider((0, 0, -HEAD_T / 2 - BRISTLE_H / 2), size=(BRISTLE_L, BRISTLE_W, BRISTLE_H)),
         kit.Collider((hx, 0, HANDLE_Z), size=(hl, HANDLE_W, HANDLE_H))]
    z0 = TOP + TOWEL_B_SIZE[2] + HEAD_T / 2 + 0.004
    return kit.Prop("brush", pos=(*BRUSH0, z0), yaw=math.pi / 2, size=(HANDLE_X[1] - HANDLE_X[0] + HEAD_L / 2, HEAD_W,
                    HANDLE_H + 0.01), mass=0.04, meshes=tuple(m), colliders=tuple(c))


def _station():
    """The fixed station: the wash basin with water, the two towels and the open denture cup."""
    m, c = [], []

    def tub(center, size, wall, look):
        x, y = center
        sx, sy, sz = size
        parts = [((x, y, wall / 2), (sx, sy, wall)),
                 ((x - sx / 2 + wall / 2, y, sz / 2), (wall, sy, sz)), ((x + sx / 2 - wall / 2, y, sz / 2), (wall, sy, sz)),
                 ((x, y - sy / 2 + wall / 2, sz / 2), (sx, wall, sz)), ((x, y + sy / 2 - wall / 2, sz / 2), (sx, wall, sz))]
        for ctr, sz_ in parts:
            m.append(kit.box(ctr, sz_, look))
            c.append(kit.Collider(ctr, size=sz_))

    tub(BASIN, BASIN_SIZE, BASIN_WALL, BASIN_LOOK)
    m.append(kit.box((*BASIN, WATER_Z / 2 + BASIN_WALL / 2),
                     (BASIN_SIZE[0] - 2 * BASIN_WALL - 0.001, BASIN_SIZE[1] - 2 * BASIN_WALL - 0.001,
                      WATER_Z - BASIN_WALL), WATER))
    tub(CUP, CUP_SIZE, CUP_WALL, CUP_LOOK)
    for ctr, size in ((TOWEL_D, TOWEL_D_SIZE), (TOWEL_B, TOWEL_B_SIZE)):
        m.append(kit.box((*ctr, size[2] / 2), size, TOWEL_LOOK))
        c.append(kit.Collider((*ctr, size[2] / 2), size=size))
    return kit.Prop("station", pos=(0.0, 0.0, TOP), size=(0.6, 0.6, BASIN_SIZE[2]), meshes=tuple(m), colliders=tuple(c))


OBJECTS = [_station(), _denture(), _brush()]


# --- Scoring ---------------------------------------------------------------------------------------------------------


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _zone_points():
    """Points along the teeth's tops (denture frame), by zone. Every point must be brushed, with the middle of the bristles passing within 4 mm of it. Front teeth: the curve from
    -45 to 45 degrees every 15. Each side: the curve from 55 to 85 degrees every 15 and the back teeth along the leg
    (every 4 mm). The far heel's last teeth are under the holding gripper's fingers, so the far side stops at the
    curve; the near side runs on to the last molar."""
    pt = lambda a_: (R * math.sin(math.radians(a_)), -R * math.cos(math.radians(a_)), HT)  # noqa: E731
    front = [pt(a_) for a_ in range(-45, 46, 15)]
    far = [pt(a_) for a_ in (55, 70, 85)]
    near = [(-x, y, z) for x, y, z in far] + [(-R, y, HT) for y in (0.004, 0.008, 0.012, 0.016)]
    return [np.array(front), np.array(far), np.array(near)]


ZONE_PTS = _zone_points()


def over(xy, center, size, wall):
    """Is the point inside the tub's inner footprint?"""
    return (abs(xy[0] - center[0]) <= size[0] / 2 - wall and abs(xy[1] - center[1]) <= size[1] / 2 - wall)


def brushing(objects):
    """Which zones the bristles are on now, the brush head's place in the denture's frame, and whether the denture is
    over the basin."""
    d, b = objects["denture"], objects["brush"]
    Rd, Rb = _rot(d.quat_xyzw), _rot(b.quat_xyzw)
    pd, pb = np.asarray(d.pos, float), np.asarray(b.pos, float)
    tip = pb + Rb @ np.array([0.0, 0.0, TIP_Z])
    on, pts_on = [], []
    if Rd[2, 2] >= UPRIGHT and float(np.dot(Rb[:, 2], Rd[:, 2])) >= UPRIGHT:
        for k, pts in enumerate(ZONE_PTS):
            hit = False
            for i, p in enumerate(pts):
                w = pd + Rd @ p
                h = float(np.dot(tip - w, Rd[:, 2]))
                loc = Rb.T @ (w - tip)
                if TIP_BAND[0] <= h <= TIP_BAND[1] and abs(loc[0]) <= BRISTLE_L / 2 and abs(loc[1]) <= BRISTLE_W / 2:
                    hit = True
                    if abs(loc[0]) <= COVER_R:
                        pts_on.append((k, i))
            if hit:
                on.append(k)
    rel = Rd.T @ (tip - pd)
    _T["pts_on"] = pts_on
    return on, rel, over(pd, BASIN, BASIN_SIZE, BASIN_WALL)


_T = {"episode": None, "objects": None, "step": 0, "strokes": None, "runs": None, "dropped": None, "cov": None, "prev_rel": None, "pts_on": []}


def _track(objects):
    if _T["episode"] != kit.episode():
        _T.update(episode=kit.episode(), objects=None, step=0, strokes=[0] * 3, runs=[None] * 3, dropped=None,
                  cov=[set(), set(), set()], prev_rel=None, pts_on=[])
    if objects is _T["objects"]:
        return
    _T["objects"] = objects
    _T["step"] += 1
    n = _T["step"]
    on, rel, in_basin = brushing(objects)
    moving = _T["prev_rel"] is not None and float(np.linalg.norm(rel[:2] - _T["prev_rel"][:2])) >= COVER_STEP
    _T["prev_rel"] = rel.copy()
    if in_basin and moving:  # a tooth counts as brushed when the bristles on it are moving, over the basin
        for k, i in _T["pts_on"]:
            _T["cov"][k].add(i)
    for k in range(3):
        run = _T["runs"][k]
        if k not in on or not in_basin:
            if run is not None and n - run["seen"] > GAP_STEPS:
                _T["runs"][k] = None  # off this zone too long: the next contact starts a new stroke
            continue
        if run is None or n - run["seen"] > GAP_STEPS:
            run = {"start": rel[:2].copy(), "last": rel[:2].copy(), "dir": None, "counted": False}
        step = rel[:2] - run["last"]
        if np.linalg.norm(step) > 1e-4:
            u = step / np.linalg.norm(step)
            if run["dir"] is not None and float(np.dot(u, run["dir"])) < -0.3:  # the brush turned back: a new stroke
                run = {"start": run["last"].copy(), "last": run["last"], "dir": u, "counted": False}
            elif run["dir"] is None:
                run["dir"] = u
        run["last"] = rel[:2].copy()
        run["seen"] = n
        if not run["counted"] and np.linalg.norm(rel[:2] - run["start"]) >= STROKE_MIN:
            _T["strokes"][k] += 1
            run["counted"] = True
        _T["runs"][k] = run
    d = objects["denture"]
    pd = np.asarray(d.pos, float)
    if (_T["dropped"] is None and float(d.lin_vel[2]) < -DROP_V and not over(pd, BASIN, BASIN_SIZE, BASIN_WALL)
            and not over(pd, CUP, CUP_SIZE, CUP_WALL)):
        _T["dropped"] = n  # falling toward the counter or the floor
    if os.environ.get("DENT_DEBUG"):
        b = objects["brush"]
        tip = np.asarray(b.pos, float) + _rot(b.quat_xyzw) @ np.array([0.0, 0.0, TIP_Z])
        Rb_, Rd_ = _rot(b.quat_xyzw), _rot(d.quat_xyzw)
        print(f"DENT {n} tilt={math.degrees(math.acos(min(1.0, float(np.dot(Rb_[:, 2], Rd_[:, 2]))))):.1f} den={np.round(pd, 3)} vz={float(d.lin_vel[2]):.2f} basin={in_basin} on={on} "
              f"tip_rel={np.round(rel, 4)} tip={np.round(tip, 3)} strokes={_T['strokes']} cov={[len(c_) for c_ in _T['cov']]}/{[len(z) for z in ZONE_PTS]} dropped={_T['dropped']} "
              f"in_cup={in_cup(objects)} L={np.round(objects['left_gripper'].pos, 3)} "
              f"R={np.round(objects['right_gripper'].pos, 3)}", flush=True)


def in_cup(objects):
    """The denture inside the cup: its middle within the cup's inner footprint and its base below the cup's rim."""
    pd = np.asarray(objects["denture"].pos, float)
    return over(pd, CUP, CUP_SIZE, CUP_WALL) and pd[2] - TOP < CUP_SIZE[2]


def zones_done(objects):
    _track(objects)
    return sum(1 for k, s in enumerate(_T["strokes"]) if s >= STROKES and len(_T["cov"][k]) == len(ZONE_PTS[k]))


def _job(objects):
    n = zones_done(objects)
    if _T["dropped"] is not None:
        return 0.0
    if n == 3:
        return 1.0 if in_cup(objects) else 0.5
    return 0.25 if n >= 1 else 0.0


def success(objects):
    """All three zones of teeth brushed (4 strokes each) over the basin, the denture never dropped outside the basin,
    and the denture in its cup."""
    return _job(objects) == 1.0


def score(objects):
    """A stroke: the bristle tips in contact with a zone's teeth (resting 0.5 mm below to 5.5 mm above their drawn tops, teeth up
    and bristles down within 30 degrees) while the brush moves 1 cm or more one way across them, with the denture's
    middle over the basin's inside. A zone (the front teeth, either side's teeth) is brushed after 4 strokes AND once
    bristles moving on every tooth of its arch (the back teeth included) have touched it. 1.0 with all 3 zones
    brushed and the denture in its cup; 0.5 with all 3 brushed and not in the cup; 0.25 with 1 or 2 brushed; 0 with
    none. Latched 0: the denture falls (downward faster than 0.5 m/s) anywhere but over the basin or the cup. Possible
    scores: 0, 0.25, 0.5, 1.0."""
    return _job(objects)


# --- Demo ------------------------------------------------------------------------------------------------------------

HOLD = (0.300, 0.010, 0.080)  # the denture's origin while it is brushed: over the basin, 3.5 cm above the water
COUNTER_HOLD = (0.150, 0.020, 0.080)  # DEMO_COUNTER=1: held over the counter instead
GZ_D = 0.010  # tool point above the middle of a thin part held in the fingertips
GZ_B = 0.006  # tool point above the brush handle's middle


def _grip_point(o):
    """The left hand's tool point on the far heel's flange for the denture's origin at o (x, y, z above TOP)."""
    return (o[0] + R, o[1] + LEG_T + FLANGE / 2, TOP + o[2] + HT / 2 + GZ_D)


def _brush_hand(tip_xy, tip_z):
    """The right hand's tool point holding the brush with its bristle tips' middle at tip_xy, height tip_z above TOP
    (the handle along y toward -y)."""
    z = TOP + tip_z - TIP_Z + HANDLE_Z + GZ_B
    return (tip_xy[0], tip_xy[1] + GRIP_X, z)


H_ZONE = (-0.004, -0.004, -0.004)  # commanded bristle-tip height from the teeth's tops, per zone
SHIFT = ((0.0, 0.0), (0.005, 0.0), (0.005, 0.0))  # xy aim offset per zone (m)


def demo():
    """Left arm: the denture by its far heel's flange, over the basin. Right arm: the brush by its handle, 15 cm behind
    the head; four-plus strokes on each zone. Then the brush back on its towel and the denture into its cup.
    (DEMO_COUNTER=1: a negative control brushing it held over the counter.)"""
    o = COUNTER_HOLD if os.environ.get("DEMO_COUNTER") else HOLD
    yl, yr = -math.pi / 2, math.pi / 2  # both closing along x: across the heel, across the handle
    d0 = (DENTURE0[0], DENTURE0[1], TOWEL_D_SIZE[2])
    gd0 = _grip_point(d0)
    gd = _grip_point(o)
    bz0 = TOWEL_B_SIZE[2] + HEAD_T / 2 + 0.004  # the brush's origin height on its towel
    gb0 = (BRUSH0[0], BRUSH0[1] + GRIP_X, TOP + bz0 + HANDLE_Z + GZ_B)
    tz = o[2] + HT  # the teeth's tops
    lo, op_d, op_b = FINGER_OPEN * 0.25, FINGER_OPEN * 0.6, FINGER_OPEN * 0.5  # lo: let go inside the cup's walls

    left = [Waypoint(2.0, (gd0[0], gd0[1], gd0[2] + 0.07), op_d, "over the far heel", yl),
            Waypoint(1.0, gd0, op_d, "fingers round the heel", yl),
            Waypoint(0.8, gd0, FINGER_CLOSED, "hold it by the heel", yl),
            Waypoint(1.0, (gd0[0], gd0[1], gd[2] + 0.02), FINGER_CLOSED, "lift it", yl),
            Waypoint(2.0, (gd[0], gd[1], gd[2] + 0.02), FINGER_CLOSED, "over the basin", yl),
            Waypoint(0.8, gd, FINGER_CLOSED, "hold it low over the water", yl)]
    right = [Waypoint(2.0, (gb0[0], gb0[1], gb0[2] + 0.07), op_b, "over the brush handle", yr),
             Waypoint(1.0, gb0, op_b, "fingers round the handle", yr),
             Waypoint(0.8, gb0, FINGER_CLOSED, "grip the handle", yr),
             Waypoint(1.2, (gb0[0], gb0[1], TOP + 0.19), FINGER_CLOSED, "lift the brush", yr)]
    tl = sum(w.duration for w in left)
    tr = sum(w.duration for w in right)
    right.append(Waypoint(tl - tr, right[-1].pos, FINGER_CLOSED, "wait for the denture", yr))

    ox, oy = o[0], o[1]

    def sweep(zk, label, h, passes=4, dt=0.2, shift=(0.0, 0.0)):
        """`passes` strokes along the whole arch of zone `zk` (its tooth points, ends extended 2 mm), the bristle tips
        at about h from the teeth's tops (negative: pressed on them, the rigid bristles stop at contact)."""
        pts = [np.array([ox + p[0] + shift[0], oy + p[1] + shift[1]]) for p in ZONE_PTS[zk]]
        d0, d1 = pts[0] - pts[1], pts[-1] - pts[-2]
        pts = [pts[0] + 0.002 * d0 / np.linalg.norm(d0)] + pts + [pts[-1] + 0.002 * d1 / np.linalg.norm(d1)]
        out = [Waypoint(1.2, _brush_hand(pts[0], tz + 0.015), FINGER_CLOSED, f"over the {label}", yr),
               Waypoint(0.5, _brush_hand(pts[0], tz + h), FINGER_CLOSED, f"bristles on the {label}", yr)]
        seq = pts
        for k in range(passes):
            seq = pts if k % 2 == 0 else pts[::-1]
            for p in seq[1:]:
                out.append(Waypoint(dt, _brush_hand(p, tz + h), FINGER_CLOSED, f"brush the {label}", yr))
        out.append(Waypoint(0.4, _brush_hand(seq[-1], tz + 0.015), FINGER_CLOSED, "lift off", yr))
        return out

    # The brush hangs 15 cm out of a friction grip, so its tips sit a few mm off the command: depth and aim per zone
    # are set from simulator runs (the rigid bristles stop at the teeth, so a deeper command only presses harder).
    right += sweep(0, "front teeth", h=H_ZONE[0], shift=SHIFT[0])
    right += sweep(1, "far side's teeth", h=H_ZONE[1], shift=SHIFT[1])
    right += sweep(2, "near side's teeth", h=H_ZONE[2], shift=SHIFT[2])
    brushed = sum(w.duration for w in right)
    left.append(Waypoint(brushed - sum(w.duration for w in left), gd, FINGER_CLOSED, "hold it still for the brush", yl))
    # The brush back on its towel.
    right += [Waypoint(1.0, (right[-1].pos[0], right[-1].pos[1], TOP + 0.19), FINGER_CLOSED, "brush up and away", yr),
              Waypoint(2.0, (gb0[0], gb0[1], gb0[2] + 0.05), FINGER_CLOSED, "back over its towel", yr),
              Waypoint(1.0, (gb0[0], gb0[1], gb0[2] + 0.004), FINGER_CLOSED, "set it down", yr),
              Waypoint(0.6, (gb0[0], gb0[1], gb0[2] + 0.004), op_b, "let go", yr),
              Waypoint(0.8, (gb0[0], gb0[1], gb0[2] + 0.07), op_b, "lift clear", yr)]
    # The denture into its cup, its heel's flange near the cup's middle so the fingers clear the walls.
    oc = (CUP[0] - 0.006, CUP[1] - 0.008, CUP_WALL + 0.004)
    gc = _grip_point(oc)
    left += [Waypoint(1.0, (gd[0], gd[1], TOP + 0.13), FINGER_CLOSED, "lift it, clear of the cup's rim", yl),
             Waypoint(2.0, (gc[0], gc[1], TOP + 0.13), FINGER_CLOSED, "over the cup", yl),
             Waypoint(1.4, gc, FINGER_CLOSED, "lower it into the cup", yl),
             Waypoint(0.6, gc, lo, "let go", yl),
             Waypoint(1.0, (gc[0], gc[1], TOP + 0.13), lo, "lift clear", yl)]  # noqa: E501
    pad = sum(w.duration for w in right) - sum(w.duration for w in left)
    if pad > 0:
        left.append(Waypoint(pad, left[-1].pos, lo, "wait", yl))
    else:
        right.append(Waypoint(-pad, right[-1].pos, op_b, "wait", yr))
    left += [kit.Home(1.8), kit.Home(1.0, label="stay home")]
    right += [kit.Home(1.8), kit.Home(1.0, label="stay home")]
    return {"left": left, "right": right}
