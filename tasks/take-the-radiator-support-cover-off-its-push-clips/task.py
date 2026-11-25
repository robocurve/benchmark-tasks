"""The front of a 2010-2015 Toyota Prius with the hood open, seen from in front of the car. The black radiator support
opening cover lies along the top of the upper radiator support, between the two headlamp tops, with the hood latch
standing up through a notch in its middle. Three black two-piece push-type clips hold it down through holes in the
cover and the support, and two claws on its front edge hook under the support's front edge. A blue fender cover lies
over the bumper top in front of the radiator support. On it are a grey plastic parts tray and two Lisle 35460
double-ended clip lifters, one on each side; each lifter lies flat with its narrower fork end toward the middle.
"""
from __future__ import annotations
import json
import math
import os
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import parts  # noqa: E402
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.demo import FINGER_OPEN, FINGER_CLOSED  # noqa: E402

SCENE = 'station'
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 0.0   # score() and success() share the measured one-second hold
EPS = 1e-6

# -- colours (sRGB) ---------------------------------------------------------------------------------------------
ECOAT = (.16, .17, .18)     # radiator support (black e-coat steel)
PP = (.09, .09, .10)        # cover: textured black polypropylene
NYLON = (.05, .05, .06)     # clip bodies
PIN = (.30, .30, .32)       # clip centre pins (dark grey)
LATCH = (.45, .46, .48)
LAMP = (.80, .82, .85)
CHROME = (.70, .71, .73)
RAD = (.25, .27, .28)
MAT = (.18, .30, .55)       # fender cover
TRAYC = (.55, .56, .58)
ZINC = (.72, .74, .76)
GRIPC = (.85, .12, .10)     # the lifter's red vinyl grip

# -- heights (metres; x away from the robot, y to its left, z up) -------------------------------------------------
MAT_T = .002
Z_MAT = TOP + MAT_T                 # fender cover top
Z_S = TOP + .040                    # radiator support top (estimate: 40 mm above the bumper top)
PLATE_T = .003                      # support top plate (simulated 3 mm; real about 1.4 mm steel)
COVER_T = .0025                     # cover wall
Z_C = Z_S + COVER_T                 # cover top
SUP_X0, SUP_X1 = .290, .420         # radiator support top, front and rear edges
COV_X0, COV_X1 = .300, .410         # cover, fitted (110 mm deep, estimate)
COV_Y = .450                        # cover reaches y = +-450 mm (900 mm long, estimate)
HOLE = .008                         # 8 mm square holes in cover and support for the 6.5 mm shanks
CLIP_X = .355
CLIP_Y = {'clip_a': .230, 'clip_b': .080, 'clip_c': -.200}   # estimate from the part
CLAW_Y = (.300, -.300)
LATCH_BOX = ((.328, .395), (-.035, .035), .045)              # hood latch body: x, y ranges, height above support
NOTCH = ((.315, COV_X1 + .001), (-.045, .045))               # the cover's notch round the latch, open at the rear
RIB_X = .3135                       # raised front lip on the cover (12 mm thick, 10 mm tall, estimate)
RIB_T, RIB_H = .012, .010           # box-section stiffening rib along the front edge (estimate)


def plate_boxes(x0, x1, y0, y1, holes, z0, t, color, friction=None):
    """Boxes covering the rectangle [x0,x1] x [y0,y1] (thickness t, bottom z0) except the hole rectangles."""
    ys = sorted({y0, y1} | {v for h in holes for v in (h[2], h[3]) if y0 < v < y1})
    out = []
    for ya, yb in zip(ys[:-1], ys[1:]):
        ym = (ya + yb) / 2
        cuts = sorted((h[0], h[1]) for h in holes if h[2] < ym < h[3])
        xa = x0
        for cx0, cx1 in cuts + [(x1, x1)]:
            if cx0 - xa > 1e-6:
                out.append(parts.box((cx0 - xa, yb - ya, t), at=((xa + cx0) / 2, ym, z0 + t / 2), color=color, friction=friction))
            xa = max(xa, cx1)
    return out


def _hole(x, y, s=HOLE): return (x - s / 2, x + s / 2, y - s / 2, y + s / 2)


# -- the car (fixed) ------------------------------------------------------------------------------------------
CLAW_POCKET = [(.304, .322, cy - .015, cy + .015) for cy in CLAW_Y]
_sup_holes = [_hole(CLIP_X, y) for y in CLIP_Y.values()] + CLAW_POCKET
_car = [parts.box((.250, 1.10, MAT_T), at=(.135, 0, TOP + MAT_T / 2), color=MAT)]                       # fender cover
_car += plate_boxes(SUP_X0, SUP_X1, -.62, .62, _sup_holes, Z_S - PLATE_T, PLATE_T, ECOAT, friction=(.3, .25))  # support top
_car += [parts.box((.004, 1.24, Z_S - PLATE_T - TOP), at=(SUP_X0 + .002, 0, TOP + (Z_S - PLATE_T - TOP) / 2), color=ECOAT),
         parts.box((.004, 1.24, Z_S - PLATE_T - TOP), at=(SUP_X1 - .002, 0, TOP + (Z_S - PLATE_T - TOP) / 2), color=ECOAT)]
_car += [parts.box((.026, 1.24, Z_S - .008 - TOP), at=(.275, 0, TOP + (Z_S - .008 - TOP) / 2), color=CHROME)]  # grille top
(_lx0, _lx1), (_ly0, _ly1), _lh = LATCH_BOX
_car += [parts.box((_lx1 - _lx0, _ly1 - _ly0, _lh), at=((_lx0 + _lx1) / 2, 0, Z_S + _lh / 2), color=LATCH)]
for s in (1, -1):                                                                                     # headlamp tops
    _car += [parts.box((.250, .150, Z_S + .020 - TOP), at=(.325, s * .545, TOP + (Z_S + .020 - TOP) / 2), color=LAMP)]
_car += [parts.box((.120, .800, Z_S - .015 - TOP), at=(.500, 0, TOP + (Z_S - .015 - TOP) / 2), color=RAD)]  # radiator top
CAR = parts.Part('car', pos=(0., 0., 0.), shapes=_car)

# -- the cover (loose rigid) ---------------------------------------------------------------------------------------
_cov_holes = [_hole(CLIP_X, y) for y in CLIP_Y.values()] + [(NOTCH[0][0], NOTCH[0][1], NOTCH[1][0], NOTCH[1][1])]
_cov = plate_boxes(COV_X0, COV_X1, -COV_Y, COV_Y, _cov_holes, Z_S, COVER_T, PP)
_cov += [parts.box((RIB_T, COV_Y - .050, RIB_H), at=(RIB_X, s * (COV_Y + .050) / 2, Z_C + RIB_H / 2), color=PP) for s in (1, -1)]
for cy in CLAW_Y:   # claw: a tab down through the support's pocket with a hook reaching forward under its front edge
    _cov += [parts.box((.003, .020, .009), at=(.3065, cy, Z_S - .0045), color=PP),
             parts.box((.008, .020, .0045), at=(.304, cy, Z_S - .00675), color=PP)]
COVER_MASS = .35
_cov_c = np.array([(COV_X0 + COV_X1) / 2, 0., Z_S])
COVER = parts.Part('cover', pos=tuple(float(v) for v in _cov_c),
                   shapes=[parts.box(s.size, at=tuple(float(v) for v in np.asarray(s.at) - _cov_c), color=s.color) for s in _cov],
                   mass=COVER_MASS, friction=(.3, .25), contact_offset=.002, angular_damping=1.)

# -- clips (loose, jointed): body + centre pin on a slider ---------------------------------------------------------
HEAD_R0, HEAD_R1, HEAD_H = .010, .0095, .0016   # body head: 20 mm across, rounded edge (12-sided frustum)
SHANK, SHANK_L = .0065, .013
NECK_R, GAP = .002, .0025                        # pin neck and the gap under the pin head (a fork slides in here)
PIN_R, PIN_H = .005, .0014                       # pin head 10 mm across
PIN_TRAVEL, UNLOCK = .006, .004                  # the pin unlocks the legs once raised 4 mm; stops at 6 mm
PIN_K = 200.                                     # N/m: the pin's detent pulls it back down until it unlocks
HOLD_LOCKED = 25.                                # N: the spread legs hold the body down while the pin is down
DRAG_OPEN = 1.0                                  # N: legs dragging in the hole after the pin is up (first 6 mm)
BODY_M, PIN_M = .010, .006                       # simulated masses (real clip about 2 g in all)


def frustum(r0, r1, h, n=12):
    bot = [(r0 * math.cos(2 * math.pi * i / n), r0 * math.sin(2 * math.pi * i / n), 0.) for i in range(n)]
    top = [(r1 * math.cos(2 * math.pi * i / n), r1 * math.sin(2 * math.pi * i / n), h) for i in range(n)]
    faces = [tuple(range(n, 2 * n)), tuple(range(n - 1, -1, -1))] + [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
    return bot + top, faces


def make_clip(name, y):
    pts, faces = frustum(HEAD_R0, HEAD_R1, HEAD_H)
    body = [parts.hull(pts, faces, color=NYLON),
            parts.box((SHANK, SHANK, SHANK_L), at=(0, 0, -SHANK_L / 2), color=NYLON),
            parts.box((.0025, .007, .007), at=(.0042, 0, -SHANK_L + .004), color=NYLON, collide=False),
            parts.box((.0025, .007, .007), at=(-.0042, 0, -SHANK_L + .004), color=NYLON, collide=False)]
    pin = parts.Slider('pin', axis=(0., 0., 1.), travel=PIN_TRAVEL, mass=PIN_M, spring=PIN_K, damping=1.0, max_effort=50.,
                       shapes=(parts.cylinder(NECK_R, GAP, at=(0, 0, HEAD_H + GAP / 2), color=PIN),
                               parts.cylinder(PIN_R, PIN_H, at=(0, 0, HEAD_H + GAP + PIN_H / 2), color=PIN),
                               parts.cylinder(.0015, .010, at=(0, 0, -.005), color=PIN)))
    return parts.Part(name, pos=(CLIP_X, y, Z_C), mass=BODY_M, shapes=body, joints=[pin], friction=(.6, .5),
                      contact_offset=.001, angular_damping=1.)


CLIPS = [make_clip(n, y) for n, y in CLIP_Y.items()]
CLIP_NAMES = list(CLIP_Y)

# -- the parts tray (loose) ----------------------------------------------------------------------------------------
TRAY_X, TRAY_Y, TRAY_H, TRAY_W = .090, .240, .020, .003     # 90 x 240 x 20 mm, 3 mm walls (estimate)
TRAY_AT = (.095, 0.)
_tr = [parts.box((TRAY_X, TRAY_Y, TRAY_W), at=(0, 0, TRAY_W / 2), color=TRAYC),
       parts.box((TRAY_W, TRAY_Y, TRAY_H), at=(TRAY_X / 2 - TRAY_W / 2, 0, TRAY_H / 2), color=TRAYC),
       parts.box((TRAY_W, TRAY_Y, TRAY_H), at=(-TRAY_X / 2 + TRAY_W / 2, 0, TRAY_H / 2), color=TRAYC),
       parts.box((TRAY_X, TRAY_W, TRAY_H), at=(0, TRAY_Y / 2 - TRAY_W / 2, TRAY_H / 2), color=TRAYC),
       parts.box((TRAY_X, TRAY_W, TRAY_H), at=(0, -TRAY_Y / 2 + TRAY_W / 2, TRAY_H / 2), color=TRAYC)]
TRAY = parts.Part('tray', pos=(TRAY_AT[0], TRAY_AT[1], Z_MAT), mass=.20, shapes=_tr, friction=(.6, .5),
                  contact_offset=.002, angular_damping=1.)

# -- the clip lifters (Lisle 35460: 8 in overall, 3-1/2 in grip, zinc-plated steel; end shapes estimated) ----------
# Tool frame: x along the tool, narrow (pin) fork at +x, wide (clip) fork at -x; origin at the grip's centre.
GRIP_L, GRIP_W = .089, .020
SH_L, SH_W, SH_T = .039, .012, .003      # 89 + 2 x (39 + 18) = 203 mm overall
HEAD_L = .018                                # fork ends 18 mm long, straight and flush with the grip's underside (estimate)
TINE_T, TINE_W = .0015, .004
SLOT_N, SLOT_W = .005, .010                  # narrow and wide fork openings (estimate)
FLARE = .002                                 # each tine's inner edge flares 2 mm at the tip
P0X = GRIP_L / 2 + SH_L
P0Z = -GRIP_W / 2 + SH_T / 2


def _qy(a): return (0., math.sin(a / 2), 0., math.cos(a / 2))


def tool_shapes():
    s = [parts.box((GRIP_L, GRIP_W, GRIP_W), at=(0, 0, 0), color=GRIPC, friction=(1.2, 1.0))]   # vinyl grip
    for sx, slot in ((1, SLOT_N), (-1, SLOT_W)):
        s.append(parts.box((SH_L, SH_W, SH_T), at=(sx * (GRIP_L / 2 + SH_L / 2), 0, P0Z), color=ZINC))
        u = np.array([float(sx), 0., 0.])
        p0 = np.array([sx * P0X, 0., -GRIP_W / 2 + TINE_T / 2])     # tines flush with the underside
        web = p0 + .003 * u + np.array([0., 0., (.002 - TINE_T) / 2])
        s.append(parts.box((.006, slot + 2 * TINE_W, .002), at=tuple(float(v) for v in web), color=ZINC))
        nrm = np.array([0., 0., 1.])
        for sy in (1, -1):
            # a tine 12 mm long, its inner edge flaring 2 mm outward over the last 4 mm so the opening guides the
            # pin's neck in (as on the real fork); a convex piece
            prof = [(.006, slot / 2), (.006, slot / 2 + TINE_W), (.018, slot / 2 + TINE_W), (.018, slot / 2 + FLARE),
                    (.014, slot / 2)]
            pts = []
            for h in (-TINE_T / 2, TINE_T / 2):
                for a, w in prof:
                    v = p0 + a * u + h * nrm + np.array([0., sy * w, 0.])
                    pts.append(tuple(float(c) for c in v))
            n = len(prof)
            faces = [tuple(range(n)), tuple(range(2 * n - 1, n - 1, -1))] + [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
            s.append(parts.hull(pts, faces, color=ZINC))
    return s


TIP_DX = P0X + HEAD_L                         # tool x of the narrow fork's tip
TIP_DZ = -GRIP_W / 2                          # the tines' underside (tool z), flush with the grip's
TOOL_REST_Z = Z_MAT - TIP_DZ                  # grip centre height lying flat on the fender cover
TOOL_M = .080


def _qz(a): return (0., 0., math.sin(a / 2), math.cos(a / 2))


TOOL_L_AT = (.130, .370)    # lot A: left lifter, along y, narrow end toward -y (the middle)
TOOL_R_AT = (.130, -.370)   # lot A: right lifter, narrow end toward +y
LIFTER_L = parts.Part('lifter_left', pos=(TOOL_L_AT[0], TOOL_L_AT[1], TOOL_REST_Z), quat=_qz(-math.pi / 2), mass=TOOL_M,
                      shapes=tool_shapes(), friction=(.4, .3), contact_offset=.001, angular_damping=1.)
LIFTER_R = parts.Part('lifter_right', pos=(TOOL_R_AT[0], TOOL_R_AT[1], TOOL_REST_Z), quat=_qz(math.pi / 2), mass=TOOL_M,
                      shapes=tool_shapes(), friction=(.4, .3), contact_offset=.001, angular_damping=1.)
TOOLS = ['lifter_left', 'lifter_right']

OBJECTS = [CAR, COVER] + CLIPS + [TRAY, LIFTER_L, LIFTER_R]

# Two layouts. Only rigid loose things move (the jointed clips and the cover they hold keep one pose): in lot B the
# tray is 30 mm further left and 10 mm further out, and the lifters lie in other places.
_B_TOOL_L, _B_TOOL_R, _B_TRAY = (.150, .395), (.115, -.345), (.105, .030)
LOTS = lots.declare(OBJECTS, lots.Lot('A', place={'tray': (TRAY_AT[0], TRAY_AT[1], Z_MAT)}),
                    lots.Lot('B', place={'tray': (_B_TRAY[0], _B_TRAY[1], Z_MAT),
                                         'lifter_left': (_B_TOOL_L[0], _B_TOOL_L[1], TOOL_REST_Z),
                                         'lifter_right': (_B_TOOL_R[0], _B_TOOL_R[1], TOOL_REST_Z)}))

# -- scoring constants ---------------------------------------------------------------------------------------------
BROKEN_RISE = .002      # a clip body raised 2 mm while its pin is still down: legs snapped
TORN_RISE = .005        # the cover lifted 5 mm at a clip still fitted: hole torn
TOUCH = .005            # a gripper within 5 mm of a clip still touches it
TOOL_TOUCH = .002       # a lifter within 2 mm of a clip still holds it
CLEAR = .030            # hands 30 mm clear of cover, clips, lifters and tray at the end
OFF_X = SUP_X0 - .002   # cover off: wholly in front of the support's front edge ...
OFF_Z = .015            # ... and its lowest point within 15 mm of the fender cover
FALL = TOP - .010


def fresh():
    return dict(known=False, bad=False, why='', unlocked={n: False for n in CLIP_NAMES}, maxrise={n: 0. for n in CLIP_NAMES},
                hold=0., anchor=None, prev_t=None, clear=False, substeps=0, monitor_ok=True, wrench_ok=True)


REC = fresh()


def rot(st): return parts._quat_to_matrix(st.quat_xyzw)
def world(st, p): return np.asarray(p) @ rot(st).T + np.asarray(st.pos)
def local(st, p): return (np.asarray(p) - np.asarray(st.pos)) @ rot(st)


def fail(why):
    if not REC['bad']:
        REC['why'] = str(why)
    REC['bad'] = True


def rise(clip):
    return float(clip.pos[2]) - Z_C


def seated_xy(clip, name):
    return abs(float(clip.pos[0]) - CLIP_X) <= .003 + EPS and abs(float(clip.pos[1]) - CLIP_Y[name]) <= .003 + EPS


def pin_up(clip): return float(clip.joints.get('pin', 0.))


def clip_check(states, cover=True):
    """Whole-trial clip and cover rules on one set of states (the cover rule only with ``cover``). Returns a reason
    or ''."""
    cov0 = COVER.state()
    for n in CLIP_NAMES:
        c = states[n]
        if pin_up(c) >= UNLOCK - EPS:
            REC['unlocked'][n] = True
        r = rise(c)
        REC['maxrise'][n] = max(REC['maxrise'][n], r)
        if not REC['unlocked'][n] and r > BROKEN_RISE + EPS:
            return '%s pulled up %.1f mm with its centre pin still down: legs snapped' % (n, r * 1000)
        if cover and not REC['unlocked'][n] and seated_xy(c, n):
            p = np.array([CLIP_X, CLIP_Y[n], Z_S])
            lift = float(world(states['cover'], local(cov0, p))[2]) - Z_S
            if lift > TORN_RISE + EPS:
                return 'cover lifted %.1f mm at %s with the clip still fitted: hole torn' % (lift * 1000, n)
    return ''


# -- end-state tests -------------------------------------------------------------------------------------------------
def in_tray(clip, tray):
    """The clip's centre inside the tray's walls and its lowest point on the tray floor (within 4 mm), the tray
    upright (tilted under 18 degrees)."""
    p = local(tray, clip.pos)
    floor = float(world(tray, [0., 0., TRAY_W])[2])
    return bool(abs(p[0]) <= TRAY_X / 2 - TRAY_W + EPS and abs(p[1]) <= TRAY_Y / 2 - TRAY_W + EPS
                and float(clip.low[2]) <= floor + .004 + EPS and float(clip.low[2]) >= floor - .003
                and float(rot(tray)[2, 2]) > .95)


def cover_off(cover):
    return bool(float(cover.high[0]) <= OFF_X + EPS and float(cover.low[2]) <= Z_MAT + OFF_Z + EPS
                and float(cover.low[2]) >= Z_MAT - .003)


def shape_boxes(part, st):
    poses = {'base': (np.asarray(st.pos), rot(st))}
    for name, _, _ in part.links:
        if name != 'base' and name in getattr(st, 'links', {}):
            p, q = st.links[name]
            poses[name] = (np.asarray(p), parts._quat_to_matrix(q))
    out = []
    for item in part.world_boxes(poses):
        _, c, r, size = item
        out.append((np.asarray(c), np.asarray(r), np.asarray(size)))
    return out


def min_gap(points, boxes, cap=.06):
    from yam_arena.analysis import box_signed_distance
    best = float('inf')
    for c, r, s in boxes:
        near = points[np.linalg.norm(points - c, axis=1) <= np.linalg.norm(s) / 2 + cap]
        if len(near):
            best = min(best, float(box_signed_distance(near, c, r, s).min()))
    return min(best, cap) if best == float('inf') else best


def box_gap(a_boxes, b_boxes):
    """Smallest distance between two sets of boxes, from each box's corners and face centres to the other set."""
    pts = []
    for c, r, s in a_boxes:
        h = s / 2
        corners = np.array([(x, y, z) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)], float) * h
        pts.append(corners @ r.T + c)
    return min_gap(np.concatenate(pts), b_boxes, cap=.02)


SPECS = {p.name: p for p in OBJECTS}


def released_clips(objects, hands_pts):
    """Clips lying in the tray, at rest, with no gripper within 5 mm and no lifter within 2 mm."""
    tray = objects['tray']
    tool_boxes = [b for t in TOOLS for b in shape_boxes(SPECS[t], objects[t])]
    n = 0
    for name in CLIP_NAMES:
        c = objects[name]
        if not (in_tray(c, tray) and c.resting(.02, 1.)):
            continue
        cb = shape_boxes(SPECS[name], c)
        if hands_pts is None or min_gap(hands_pts, cb) < TOUCH - EPS:
            continue
        if box_gap(cb, tool_boxes) < TOOL_TOUCH - EPS:
            continue
        n += 1
    return n


def marks(objects):
    pts = [world(objects['cover'], [[x, y, 0] for x in (-.05, .05) for y in (-.4, .4)])]
    pts += [np.asarray(objects[n].pos)[None, :] for n in CLIP_NAMES + TOOLS + ['tray']]
    return np.concatenate(pts)


def observe(objects, hands, t):
    """``hands`` is a dict of each hand's surface points, or a callable that returns it (computed only when needed)."""
    r = REC
    if r['prev_t'] is not None and t <= r['prev_t']:
        return
    dt = .05 if r['prev_t'] is None else min(.1, max(0., t - r['prev_t']))
    r['prev_t'] = float(t)
    need = ['cover', 'tray'] + CLIP_NAMES + TOOLS
    if hands is None or not all(n in objects for n in need):
        r['known'] = False; r['hold'] = 0.; fail('unknown hand or object state'); return
    r['known'] = True
    for n in need:
        if float(objects[n].low[2]) < FALL:
            fail(n + ' fell off')
    why = clip_check(objects)
    if why:
        fail(why)
    if not r['monitor_ok']:
        fail('physics monitor failed')
    _pts = []

    def allpts_fn():
        if not _pts:
            h = hands() if callable(hands) else hands
            if h is None:
                raise ValueError('unknown hand state')
            _pts.append(np.concatenate(list(h.values())))
        return _pts[0]
    # The hand checks are costly, so each runs only once the cheap conditions it guards hold.
    r['n_in'] = released_clips(objects, allpts_fn()) if any(in_tray(objects[n], objects['tray']) for n in CLIP_NAMES) else 0
    if r['n_in'] == 3 and cover_off(objects['cover']):
        cb = shape_boxes(SPECS['cover'], objects['cover'])
        r['cover_free'] = min_gap(allpts_fn(), cb) >= TOUCH - EPS
        boxes = list(cb)
        for n in ['tray'] + CLIP_NAMES + TOOLS:
            boxes += shape_boxes(SPECS[n], objects[n])
        r['clear'] = min_gap(allpts_fn(), boxes) >= CLEAR - EPS
    else:
        r['clear'] = False
        r['cover_free'] = False
    still = all(objects[n].resting(.02, 1.) for n in need)
    cur = marks(objects)
    if not (still and r['clear'] and not r['bad']):
        r['anchor'] = None; r['hold'] = 0.
    elif r['anchor'] is None or np.linalg.norm(cur - r['anchor'], axis=1).max() > .001 + EPS:
        r['anchor'] = cur.copy(); r['hold'] = 0.
    else:
        r['hold'] += dt


def score(objects):
    """0 after a whole-trial fault; 0.2 per intact clip released in the tray; 0.8 with all three in and the cover off,
    lying on the fender cover; 1.0 with that, hands 30 mm clear and everything still for 1 s."""
    r = REC
    if not r['known'] or r['bad']:
        return 0.
    n = int(r.get('n_in', 0))
    cov = objects['cover']
    if n == 3 and cover_off(cov) and cov.resting(.02, 1.) and r.get('cover_free', False):
        if r['clear'] and r['hold'] >= 1. - EPS:
            return 1.
        return .8
    return round(.2 * n, 6)


def success(objects):
    return score(objects) == 1.


# -- hands: face-sampled hulls (3 mm) -------------------------------------------------------------------------------
_FACE_CACHE = {}


def face_points(name):
    if name in _FACE_CACHE:
        return _FACE_CACHE[name]
    from yam_arena import analysis
    pts = []
    for h in analysis.link_hulls().get(name, []):
        pts.append(h.vertices)
        for plane in np.unique(np.round(h.planes, 7), axis=0):
            nrm, d = plane[:3], plane[3]
            on = h.vertices[np.abs(h.vertices @ nrm - d) < 1e-6]
            if len(on) < 3:
                continue
            c = on.mean(axis=0); u = on[0] - c; u /= np.linalg.norm(u); v = np.cross(nrm, u)
            ring = on[np.argsort(np.arctan2((on - c) @ v, (on - c) @ u))]
            for a, b in zip(ring, np.roll(ring, -1, axis=0)):
                count = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / .003)))
                ii, jj = np.meshgrid(np.arange(count + 1), np.arange(count + 1)); valid = ii + jj <= count
                pts.append(c + (a - c) * ii[valid, None] / count + (b - c) * jj[valid, None] / count)
    if not pts:
        raise ValueError('missing hand hull ' + name)
    p = np.concatenate(pts); _, ix = np.unique(np.round(p / .0015).astype(int), axis=0, return_index=True)
    _FACE_CACHE[name] = p[np.sort(ix)]
    return _FACE_CACHE[name]


def hand_clouds(sim):
    base = sim.env.unwrapped; origin = parts._np(base.scene.env_origins[0]); out = {}
    for arm in ('left', 'right'):
        asset = base.scene[arm + '_robot']; poses = parts._np(asset.data.body_link_pose_w)[0]; pts = []
        for n in ('gripper', 'tip_left', 'tip_right'):
            i = list(asset.body_names).index(n); pose = poses[i]
            pts.append(face_points(n) @ parts._quat_to_matrix(pose[3:]).T + pose[:3] - origin)
        out[arm] = np.concatenate(pts)
    if not all(np.isfinite(p).all() for p in out.values()):
        raise ValueError('nonfinite hands')
    return out


# -- the clips' legs: a declared latch (forces on each clip body, and the pin's detent target) ----------------------
def drive_clips(sim, states):
    for n in CLIP_NAMES:
        c = states[n]
        if pin_up(c) >= UNLOCK - EPS and not REC['unlocked'][n]:
            REC['unlocked'][n] = True
        if REC['unlocked'][n]:
            sim.set_target(n, 'pin', PIN_TRAVEL)
        r = rise(c)
        near = seated_xy(c, n) and r < .006
        if not REC['unlocked'][n] and near:
            f = HOLD_LOCKED
        elif REC['unlocked'][n] and near:
            f = DRAG_OPEN
        else:
            f = 0.
        try:
            sim.set_wrench(n, (0., 0., -f), (0., 0., 0.), body='base')
        except Exception as exc:  # fail closed: without the legs' hold the trial cannot be scored
            REC['wrench_ok'] = False; fail('clip hold not applied: ' + repr(exc)); raise


@parts.on_physics_step
def monitor_physics(sim):
    """Every physics substep: the clip and cover rules, so a fast pull between two control steps is not missed."""
    try:
        base = sim.env.unwrapped
        REC['substeps'] += 1
        st = {n: parts._read(base, SPECS[n]) for n in CLIP_NAMES}
        why = clip_check(st, cover=False)
        if why:
            fail(why)
    except Exception as exc:  # fail closed
        REC['monitor_ok'] = False; fail('physics monitor: ' + repr(exc))


@parts.on_reset
def reset(sim):
    global REC
    REC = fresh()
    for n in CLIP_NAMES:
        sim.set_target(n, 'pin', 0.)


@parts.on_step
def step(sim):
    try:
        objects = sim.states()
        drive_clips(sim, objects)
        observe(objects, lambda: hand_clouds(sim), float(sim.t))
        log = os.environ.get('CLIP_LOG') or (str(Path(os.environ['TRACE_OUT']).with_name('clipout.jsonl')) if os.environ.get('TRACE_OUT') else None)
        if log and round(sim.t * 20) % 2 == 0:
            from yam_arena import ik, station
            base = sim.env.unwrapped
            row = dict(t=float(sim.t), lot=lots.current().name, bad=REC['bad'], why=REC['why'], clear=REC['clear'],
                       hold=REC['hold'], n_in=REC.get('n_in', 0), score=score(objects), substeps=REC['substeps'],
                       unlocked=dict(REC['unlocked']), maxrise={k: v * 1000 for k, v in REC['maxrise'].items()},
                       cover_off=cover_off(objects['cover']), tcp={}, fingers={})
            for n in CLIP_NAMES:
                c = objects[n]
                row[n] = dict(pos=[round(float(v), 5) for v in c.pos], pin=round(pin_up(c) * 1000, 2),
                              in_tray=in_tray(c, objects['tray']), rest=c.resting(.02, 1.))
            for n in ['cover', 'tray'] + TOOLS:
                row[n] = [round(float(v), 5) for v in objects[n].pos]
                row[n + '_q'] = [round(float(v), 4) for v in objects[n].quat_xyzw]
            row['cover_low'] = float(objects['cover'].low[2]); row['cover_hx'] = float(objects['cover'].high[0])
            for side in ('left', 'right'):
                robot = base.scene[side + '_robot']; ix = sim.embodiment._ids[side + '_robot'][0]
                qj = parts._np(robot.data.joint_pos)[0]
                row['tcp'][side] = [round(float(v), 5) for v in ik.tcp_pose(qj[ix], station.BASES[side])[0]]
                row['fingers'][side] = [float(qj[list(robot.joint_names).index(k)]) for k in ('joint7', 'joint8')]
            if not hasattr(step, '_names'):
                step._names = True
                row['clip_bodies'] = list(base.scene['clip_a'].body_names)
            with Path(log).open('a') as f:
                f.write(json.dumps(row) + '\n')
    except Exception as e:
        REC['known'] = False; fail(e)
        if os.environ.get('TRACE_OUT'):
            with Path(os.environ['TRACE_OUT']).with_name('clipout-errors.txt').open('a') as f:
                f.write(repr(e) + '\n')
        raise


# -- open-loop demo from the published layout (no object, force or joint feedback) ---------------------------------
GRIP_DZ = .005                     # TCP above the grip's centre when holding a lifter
TIP_BELOW_TCP = GRIP_DZ - TIP_DZ   # the narrow fork's lowest point below the TCP
TOOL_SQ = -GRIP_W / 2 + .006       # finger target: 6 mm past the grip's faces
GRIP_AT = .030                     # the grip is held 30 mm from its centre, toward the narrow fork
CAL = dict(transit=.045, over=.045, press=.001, start=.0055, past=.0095, unlock=.008, lift=.020, up=.035, floor=.003, withdraw=.045,
           tray_dx=0., tray_dy=.055, drop_a=.015, drop_b=.055, rib_dz=.002, claw=.030, cover_lift=.030, cover_dx=-.170, cover_drop=.003,
           park_left=(.250, .250, TOP + .16), park_right=(.250, -.250, TOP + .16), sag=0.)


def demo():
    lot = lots.current()
    tl, tr, tray = lot.pos('lifter_left'), lot.pos('lifter_right'), lot.pos('tray')
    z_hold = TOOL_REST_Z + GRIP_DZ
    hover = .06

    YAW = {'left': -math.pi / 2, 'right': math.pi / 2}   # fingers close along x; the left wrist turns the other way

    def W(d, p, f, label, arm):
        return tilt.Waypoint(float(d), tuple(float(v) for v in p), float(f), label, yaw=YAW[arm])

    def pick(tool, arm):
        x, y = tool[0], tool[1] + (-GRIP_AT if arm == 'left' else GRIP_AT)
        return [W(3.0, (x, y, z_hold + hover), FINGER_OPEN, 'above the lifter grip', arm),
                W(1.5, (x, y, z_hold), FINGER_OPEN, 'fingers either side of the grip', arm),
                W(1.0, (x, y, z_hold), TOOL_SQ, 'pinch the grip', arm),
                W(1.5, (x, y, z_hold + hover), TOOL_SQ, 'lift the lifter', arm)]

    def place(tool, arm):
        x, y = tool[0], tool[1] + (-GRIP_AT if arm == 'left' else GRIP_AT)
        return [W(3.0, (x, y, z_hold + hover), TOOL_SQ, 'back over its place', arm),
                W(1.5, (x, y, z_hold + .003), TOOL_SQ, 'lower it flat onto the fender cover', arm),
                W(1.0, (x, y, z_hold + .003), FINGER_OPEN, 'let go', arm),
                W(1.5, (x, y, z_hold + hover), FINGER_OPEN, 'up', arm)]

    def clip(yc, sgn, drop_y):
        """sgn = +1: the lifter points -y (left arm), the TCP stays on the +y side of the clip; -1 for the right arm."""
        z_on = Z_C + HEAD_H - CAL['press'] + TIP_BELOW_TCP + CAL['sag']
        ty = lambda tip_y: tip_y + sgn * (TIP_DX - GRIP_AT)
        y0 = yc + sgn * CAL['start']
        y1 = yc - sgn * CAL['past']
        drop = (tray[0] + CAL['tray_dx'], drop_y)
        # flat fork, clip hanging from it by the pin head: its shank's foot is this far below the TCP
        foot = GRIP_DZ + GRIP_W / 2 - TINE_T + HEAD_H + GAP + PIN_TRAVEL + SHANK_L
        z_drop = Z_MAT + TRAY_W + foot - CAL['floor']
        arm = 'left' if sgn > 0 else 'right'
        zl = z_on + CAL['unlock']
        return [W(3.0, (CLIP_X, ty(y0), z_on + CAL['transit']), TOOL_SQ, 'over the clip, high enough to clear the cover rib', arm),
                W(1.0, (CLIP_X, ty(y0), z_on + .015), TOOL_SQ, 'narrow fork over the clip head', arm),
                W(1.5, (CLIP_X, ty(y0), z_on), TOOL_SQ, 'fork tips down on the clip head', arm),
                W(2.5, (CLIP_X, ty(y1), z_on), TOOL_SQ, 'slide the fork under the centre pin head to its throat', arm),
                W(1.5, (CLIP_X, ty(y1), zl), TOOL_SQ, 'lift the pin until it is up', arm),
                W(2.0, (CLIP_X, ty(y1), zl + CAL['lift']), TOOL_SQ, 'draw the clip out on the fork', arm),
                W(1.5, (CLIP_X, ty(y1), zl + CAL['lift'] + CAL['up']), TOOL_SQ, 'clear of the cover', arm),
                W(3.5, (drop[0], ty(drop[1]), z_drop + CAL['over']), TOOL_SQ, 'carry the clip over the tray', arm),
                W(1.5, (drop[0], ty(drop[1]), z_drop), TOOL_SQ, 'stand the clip on the tray floor', arm),
                W(1.5, (drop[0], ty(drop[1]) + sgn * CAL['withdraw'], z_drop), TOOL_SQ, 'draw the fork back off the clip', arm),
                W(1.0, (drop[0], ty(drop[1]) + sgn * CAL['withdraw'], z_drop + .03), TOOL_SQ, 'up', arm)]

    # Drop places (fork tips): each later clip goes where the lifter's shank will not pass over an earlier one.
    L = (pick(tl, 'left') + clip(CLIP_Y['clip_a'], 1, tray[1] + CAL['drop_a']) + clip(CLIP_Y['clip_b'], 1, tray[1] + CAL['drop_b'])
         + place(tl, 'left'))
    ty_r = tray[1] - CAL['tray_dy']
    R = pick(tr, 'right') + clip(CLIP_Y['clip_c'], -1, ty_r) + place(tr, 'right')
    tL = sum(w.duration for w in L); tR = sum(w.duration for w in R)
    R.append(W(2.0, CAL['park_right'], FINGER_OPEN, 'right hand waits clear', 'right'))
    R.append(W(max(.5, tL - tR - 2.0), CAL['park_right'], FINGER_OPEN, 'wait', 'right'))
    # Both hands take the cover by its front lip, slide it 7 mm back to free the claws, lift it and lay it on the
    # fender cover in front of the support.
    zr = Z_C + CAL['rib_dz'] + .013
    for plan, ys, arm in ((L, .170, 'left'), (R, -.170, 'right')):
        x0 = RIB_X
        x1 = x0 + CAL['claw']
        x2 = x1 + CAL['cover_dx']
        z_set = Z_MAT + .009 + CAL['cover_drop'] - Z_S + zr
        plan += [W(3.0, (x0, ys, zr + .05), FINGER_OPEN, 'over the cover lip', arm),
                 W(1.5, (x0, ys, zr), FINGER_OPEN, 'fingers either side of the lip', arm),
                 W(1.0, (x0, ys, zr), FINGER_CLOSED, 'grip the lip', arm),
                 W(1.5, (x1, ys, zr), FINGER_CLOSED, 'push the cover back off its claws until its notch meets the hood latch', arm),
                 W(2.0, (x1, ys, zr + CAL['cover_lift']), FINGER_CLOSED, 'lift it off the support', arm),
                 W(4.0, (x2, ys, zr + CAL['cover_lift']), FINGER_CLOSED, 'bring it forward over the bumper', arm),
                 W(2.5, (x2, ys, z_set), FINGER_CLOSED, 'lower it onto the fender cover', arm),
                 W(1.0, (x2, ys, z_set), FINGER_OPEN, 'let go', arm),
                 W(2.0, (x2, ys, z_set + .08), FINGER_OPEN, 'hands up and clear', arm),
                 W(4.0, (x2, ys, z_set + .08), FINGER_OPEN, 'stay still', arm)]
    return {'left': L, 'right': R}
