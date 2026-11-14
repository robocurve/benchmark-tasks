"""Load four Han-Modular connector modules into a HARTING hinged frame that its own spring holds open, then squeeze it shut round them until it clicks.

A Han hinged frame plus, size 16B (09 14 016 0361, HARTING's drawing: 84.6 mm long over its end flanges, 34.4 mm across them, side walls 19.5 to 20.5 mm
tall; 64 mm inside, so four 16 mm positions A to D; 70 g) sits on a bench assembly stand by its two end flanges. Its two zinc die-cast side walls are hinged
at their feet and sprung open 15 degrees. Each wall has a narrow support edge (a 2.5 mm lip) along its foot and four windows, one per position. Four assembled Han E
crimp modules (09 14 006 3001: 8 g, polycarbonate, pebble grey; 14.6 x 25 x 34 mm; six crimp contacts fitted, their wires leaving the underside as a 30 mm
tail) stand in a rack, their tails hanging between its rails. A module set into the open frame
rests on the two support edges; one pushed sideways off them drops through. Squeezed shut, the walls close round the modules, each module's two lugs (2 mm)
enter their windows, and at the closed stop the frame clicks (a code-gated latch) and stays shut. A module out of line by more than about 1.7 mm along the
frame (the windows' chamfered mouths steer a lug that far) puts its lug against a window's post, and the frame cannot close.

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --stills 20 --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Load the four modules into the hinged frame and close it until it clicks, then move both hands clear."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 1 s hold: score() and success() agree
HOLD_STEPS = 20
TOP = TABLE_TOP[SCENE]

# -- the frame, in its own frame: origin at the middle of its foot, x along it, y across, z up (mating side)
F = np.array([0.30, 0.0])  # on the table, station metres
STAND_H = 0.035  # the stand lifts the frame's foot this far off the table: a seated module's 30 mm wire tail hangs free under it, as through a panel's cut-out, clear of the table
PITCH = 0.016
SLOT_X = tuple((k - 1.5) * PITCH for k in range(4))  # A..D
END_IN, END_OUT, END_W = 0.032, 0.0423, 0.0344  # the end flanges: from 32 mm to 42.3 mm off the middle, 34.4 mm across
WALL_H, WALL_T, WALL_IN = 0.020, 0.0015, 0.013  # the side walls: 20 mm tall, 1.5 mm thick, inner faces 13 mm off the middle when shut
HINGE_Y = WALL_IN + WALL_T  # each wall turns about its outer foot
WALL_X = 0.034
WIN_W, WIN_Z0, WIN_Z1 = 0.006, 0.0065, 0.0155  # a window per position: 6 mm for the 5 mm lug, so a seated module sits within 0.5 mm along the frame
LIP_W, LIP_H = 0.0025, 0.0015  # the support edge along each wall's foot, inside
OPEN = float(os.environ.get("FRAME_OPEN_DEG", "15"))
SPRING = float(os.environ.get("FRAME_SPRING", "0.3"))  # N m/rad per wall: about 8 N at the walls' middle (10 mm up), 4 N at the top, to hold one shut from 15 degrees; approximated, to be measured
CLICK = 0.5  # degrees short of shut at which the latch takes

# -- a module, in its own frame: origin at the middle of its foot
MOD = (0.0146, 0.025, 0.034)  # along the frame, across, tall: a distributor's listing gives 15.5 x 25 x 34 mm (unverified); modelled 14.6 wide, 1.4 mm between modules at the 16 mm pitch
LUG = (0.005, 0.002, 0.006)  # each wide face's lug: 5 mm along, 2 mm out, 6 mm tall
LUG_Z = 0.0065  # the lug's foot above the module's foot: shut, it sits from 8 to 14 mm up the wall, in the window (6.5 to 15.5)
MOD_MASS = 0.008
SEAT_Z = LIP_H  # a seated module's foot, on the support edges, in the frame's frame
OPEN_RISE = 0.001  # the support edges' inner corners rise about 1 mm as the walls open 15 degrees: a module on the open frame sits this far over its shut seat
TRAY = tuple(np.array([0.205 + 0.025 * k, -0.16]) for k in range(4))
TRAY_H = 0.045  # the modules' rack: they stand on its two rails, their wire tails hanging down between them
TAIL_SEG, TAIL_N, TAIL_MASS = 0.030, 1, 0.003  # each module's wire tail: six 0.75 mm2 wires, a 30 mm hanging segment of 3 g on a sprung joint (approximated; real tails run on to a harness)
TAIL_K, TAIL_D = 0.0005, 0.005  # the bundle's bending, N m/rad and N m s/rad at each joint (approximated, to be measured): the damping keeps the tail from swinging the module in the pinch

LOOKS = {
    "zinc": m.Look(color=m.rgb(0.70, 0.71, 0.72), metallic=0.8, roughness=0.45),
    "steel": m.Look(color=m.rgb(0.80, 0.81, 0.82), metallic=0.9, roughness=0.3),
    "screw": m.Look(color=m.rgb(0.62, 0.63, 0.64), metallic=0.9, roughness=0.35),
    "module": m.Look(color=m.rgb(0.71, 0.69, 0.63), roughness=0.55),  # RAL 7032 pebble grey
    "contact": m.Look(color=m.rgb(0.05, 0.05, 0.05), roughness=0.8),
    "mark": m.Look(color=m.rgb(0.12, 0.12, 0.13), roughness=0.6),
    "stand": m.Look(color=m.rgb(0.20, 0.21, 0.23), metallic=0.5, roughness=0.5),
    "tray": m.Look(color=m.rgb(0.06, 0.06, 0.07), roughness=0.7),
    "brass": m.Look(color=m.rgb(0.78, 0.62, 0.28), metallic=0.9, roughness=0.35),
    "wire": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
}


FUNNEL = 0.0012  # each window's sides are chamfered this much on the wall's inner face: an 8.4 mm mouth narrowing to 6 mm, which steers a lug up to 1.7 mm off into its window as the wall closes


def _wall_pieces(side: float):
    """The wall's pieces, shut, as (name, Mesh): strips over and under the windows, posts between them with funnel chamfers at the windows, and the support edge."""
    yc, yi, yo = side * (WALL_IN + WALL_T / 2), side * WALL_IN, side * (WALL_IN + WALL_T)
    out = [("low", m.box((2 * WALL_X, WALL_T, WIN_Z0), (0.0, yc, WIN_Z0 / 2))), ("high", m.box((2 * WALL_X, WALL_T, WALL_H - WIN_Z1), (0.0, yc, (WALL_H + WIN_Z1) / 2)))]
    edges = [-WALL_X] + [v for x in SLOT_X for v in (x - WIN_W / 2, x + WIN_W / 2)] + [WALL_X]
    for i in range(0, len(edges), 2):
        x0, x1 = edges[i], edges[i + 1]
        c0, c1 = (FUNNEL if i > 0 else 0.0), (FUNNEL if i + 2 < len(edges) else 0.0)  # chamfer only the sides that face a window
        outline = [(x0 + c0, yi), (x1 - c1, yi), (x1, yo), (x0, yo)] if side > 0 else [(x0, yo), (x1, yo), (x1 - c1, yi), (x0 + c0, yi)]
        out.append((f"post{i // 2}", m.prism(outline, WIN_Z0, WIN_Z1)))
    out.append(("lip", m.box((2 * END_IN, LIP_W, LIP_H), (0.0, side * (WALL_IN - LIP_W / 2), LIP_H / 2))))
    return out


def _opened(mesh: m.Mesh, side: float) -> m.Mesh:
    """A wall's piece turned open about the wall's outer foot (joint zero is open)."""
    h = np.array([0.0, side * HINGE_Y, 0.0])
    return mesh.moved(-h).turned(-side * OPEN, "x").moved(h)


def _wall(side: float) -> m.Link:
    name = "wall_l" if side > 0 else "wall_r"
    parts, cols = [], []
    for n, piece in _wall_pieces(side):
        mesh = _opened(piece, side)
        parts.append((f"{name}_{n}", mesh, "steel"))
        cols.append(m.col_hull(f"{name}_{n}_c", mesh))
    lower, upper = (0.0, OPEN) if side > 0 else (-OPEN, 0.0)
    # a 15 g plate turned about its foot has about 2e-6 kg m^2; the sim needs more (armature and inertia) to stay stable against the gripper's contacts
    return m.hinge(name, "X", (0.0, side * HINGE_Y, 0.0), lower, upper, parts, cols, mass=0.015, spring=SPRING, damping=0.02, friction=0.002, armature=2e-4,
                   inertia=(5e-5, 5e-5, 5e-5))


def _flanges():
    parts, cols = [], []
    for sx in (-1.0, 1.0):
        xc = sx * (END_IN + END_OUT) / 2
        size = (END_OUT - END_IN, END_W, WALL_H)
        parts.append((f"end{int(sx > 0)}", m.box(size, (xc, 0.0, WALL_H / 2)), "zinc"))
        cols.append(m.col_box(f"end{int(sx > 0)}_c", size, (xc, 0.0, WALL_H / 2)))
        for sy in (-1.0, 1.0):  # the fixing screws, on the diagonal, and the PE screw
            if sx * sy > 0:
                parts.append((f"screw{int(sx > 0)}", m.cylinder(0.0032, 0.003, (sx * 0.03875, sy * 0.0135, WALL_H + 0.0015), segments=20), "screw"))
            else:
                parts.append((f"pe{int(sx > 0)}", m.cylinder(0.0045, 0.004, (sx * 0.037, sy * 0.008, WALL_H + 0.002), segments=20), "screw"))
    parts.append(("triangle", m.prism([(0.033, -0.003), (0.036, 0.0), (0.033, 0.003)], WALL_H, WALL_H + 0.0003), "mark"))  # the orientation triangle on the A..D frame's end
    return parts, cols


def _frame():
    parts, cols = _flanges()
    return m.mechanism("frame", (F[0], F[1], TOP + STAND_H), [_wall(1.0), _wall(-1.0)], LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.4, 0.3), solver_iters=48)


def _stand():
    parts, cols = [], []
    for sx in (-1.0, 1.0):
        size, c = (0.007, 0.030, STAND_H), (sx * 0.0395, 0.0, STAND_H / 2)  # under the flanges' outer part only: clear of the walls' feet (x within 34 mm)
        parts.append((f"block{int(sx > 0)}", m.box(size, c), "stand"))
        cols.append(m.col_box(f"block{int(sx > 0)}_c", size, c))
    parts.append(("base", m.box((0.11, 0.05, 0.002), (0.0, 0.0, 0.001)), "stand"))
    return m.fixed("stand", (F[0], F[1], TOP), parts, cols, LOOKS)


def _tray():
    """The modules' rack: two rails along x, 45 mm tall, under the two ends of each module's foot, the wire tails hanging between them; ribs between
    the modules."""
    parts, cols = [], []
    for k, sy in enumerate((-1.0, 1.0)):
        size, c = (0.11, 0.007, TRAY_H), (0.2425, -0.16 + sy * 0.009, TRAY_H / 2)
        parts.append((f"rail{k}", m.box(size, c), "tray"))
        cols.append(m.col_box(f"rail{k}_c", size, c))
    for k in range(5):
        x = 0.1925 + 0.025 * k
        size, c = (0.004, 0.036, 0.008), (x, -0.16, TRAY_H + 0.004)
        parts.append((f"rib{k}", m.box(size, c), "tray"))
        cols.append(m.col_box(f"rib{k}_c", size, c))
    return m.fixed("tray", (0.0, 0.0, TOP), parts, cols, LOOKS)


def _module(k: int):
    """An assembled module: its body with crimp contacts in its six holes, and its wire tail hanging from its underside (a free articulation)."""
    body = m.box(MOD, (0.0, 0.0, MOD[2] / 2))
    lugs = [m.box(LUG, (0.0, sy * (MOD[1] / 2 + LUG[1] / 2), LUG_Z + LUG[2] / 2)) for sy in (-1.0, 1.0)]
    spots = [(dx, dy) for dx in (-0.0035, 0.0035) for dy in (-0.0075, 0.0, 0.0075)]
    holes = m.join(*[m.cylinder(0.0021, 0.0004, (dx, dy, MOD[2] + 0.0001), segments=14) for dx, dy in spots])
    pins = m.join(*[m.cylinder(0.0008, 0.0006, (dx, dy, MOD[2] + 0.0002), segments=10) for dx, dy in spots])  # the crimp contacts' tips, in the holes
    parts = [("body", body, "module"), ("lug_a", lugs[0], "module"), ("lug_b", lugs[1], "module"), ("holes", holes, "contact"), ("pins", pins, "brass")]
    cols = [m.col_box("body_c", MOD, (0.0, 0.0, MOD[2] / 2))] + [m.col_box(f"lug{i}_c", LUG, (0.0, sy * (MOD[1] / 2 + LUG[1] / 2), LUG_Z + LUG[2] / 2)) for i, sy in enumerate((-1.0, 1.0))]
    links = []
    for i in range(TAIL_N):  # the wire tail: hanging segments, the first turning about x (across the frame), a second about y; one segment swings across the frame only
        z0 = -i * TAIL_SEG
        wires = m.join(*[m.cylinder(0.0009, TAIL_SEG, (dx * 0.6, dy * 0.4, z0 - TAIL_SEG / 2), segments=8) for dx, dy in spots])
        seg_c = m.col_box(f"tail{i}_c", (0.004, 0.006, TAIL_SEG - 0.002), (0.0, 0.0, z0 - TAIL_SEG / 2))
        links.append(m.hinge(f"tail{i}", "X" if i % 2 == 0 else "Y", (0.0, 0.0, z0), -90.0, 90.0, [("wires", wires, "wire")], [seg_c],
                             parent=None if i == 0 else f"tail{i - 1}", mass=TAIL_MASS, spring=TAIL_K, damping=TAIL_D, friction=0.0, gravity=True))
    t = TRAY[k]
    I = (MOD_MASS / 12 * (MOD[1] ** 2 + MOD[2] ** 2), MOD_MASS / 12 * (MOD[0] ** 2 + MOD[2] ** 2), MOD_MASS / 12 * (MOD[0] ** 2 + MOD[1] ** 2))
    return m.mechanism(f"module_{'abcd'[k]}", (t[0], t[1], TOP + TRAY_H + 0.0005), links, LOOKS, frame_parts=parts, frame_colliders=cols, friction=(0.5, 0.4),
                       solver_iters=48, free_mass=MOD_MASS, frame_inertia=I)


MODULES = tuple(f"module_{c}" for c in "abcd")
OBJECTS = [_stand(), _frame(), _tray()] + [_module(k) for k in range(4)]


# -- the scorer ----------------------------------------------------------------------------

def seat_world(env, k: int, shut: bool = True) -> np.ndarray:
    """Where module position k's seat is now, in the station frame (the frame does not move: it is fixed on its stand)."""
    return np.array([F[0] + SLOT_X[k], F[1], TOP + STAND_H + SEAT_Z])


def seated(pos: np.ndarray, tilt: float, k: int, tol: float, deg: float, over: float = 0.0) -> bool:
    """Within `tol` of position k's seat along and across the frame, from 0.5 mm under its shut seat to `tol` over it plus `over`, and within `deg`
    of upright. On the open frame `over` is OPEN_RISE: there the support edges stand about 1 mm higher, and the foot may be up to `tol` over them."""
    off = pos - seat_world(None, k)
    return bool(abs(off[0]) <= tol and abs(off[1]) <= tol and -0.0005 <= off[2] <= tol + over and tilt <= deg)


def moment(c: dict) -> float:
    """The scoring text, line by line, on one moment's facts."""
    if c["dropped"] or c["forced"]:
        return 0.0
    if c["clicked"]:
        if c["seated_shut"] == 4:
            return 1.0 if c["hands"] >= 0.03 else 0.6  # all four in: 1.0 once every fingertip is 3 cm clear of the frame and the modules
        if c["seated_shut"] == 3 and c["empty"] == 1 and not c["fell"]:
            return 0.6  # three in, the fourth never set (none fell through the frame)
        return 0.075 * c["seated_shut"]  # otherwise each module seated in the shut frame: the same credit as on the open frame
    return 0.075 * c["seated_open"]  # each module seated in the open frame: 0.075 (one), up to 0.3 (all four)


FRAME_LINES = [np.array([[F[0] + x, sy * WALL_IN, TOP + STAND_H + z] for x in (-0.042, 0.042)]) for sy in (-1.0, 1.0) for z in (0.0, WALL_H)]


class Trial:
    def __init__(self) -> None:
        self.reset()

    def reset(self, env=None) -> None:
        self.steps, self.hold = 0, 0
        self.clicked = self.dropped = self.forced = self.full = self.fell = False
        self.score = 0.0
        self.report: dict = {}

    def step(self, env) -> None:
        self.steps += 1
        fr = m.state(env, "frame")
        a_l, a_r = fr.deg("wall_l"), -fr.deg("wall_r")  # each wall's closing, degrees: 0 open, OPEN shut
        mods = [m.state(env, n) for n in MODULES]
        for s in mods:
            if s.pos[2] < TOP - 0.02:
                self.dropped = True
            if abs(s.pos[0] - F[0]) < END_OUT and abs(s.pos[1] - F[1]) < END_W / 2 and s.pos[2] < TOP + STAND_H - 0.002:
                self.fell = True  # through the frame: under its foot, inside its outline
        # which module sits at each position: the nearest, if any is within 8 mm along the frame and 6 mm across
        near = []
        for k in range(4):
            s0 = seat_world(env, k)
            d = [np.linalg.norm(s.pos[:2] - s0[:2]) for s in mods]
            j = int(np.argmin(d))
            near.append(j if abs(mods[j].pos[0] - s0[0]) < 0.008 and abs(mods[j].pos[1] - s0[1]) < 0.006 else None)
        if not self.clicked and a_l >= OPEN - CLICK and a_r >= OPEN - CLICK:
            self.clicked = True
            m.lock(env, "frame", "wall_l")
            m.lock(env, "frame", "wall_r")
            high = [round(1000 * float(mods[j].pos[2] - seat_world(env, k)[2]), 2) for k, j in enumerate(near) if j is not None]
            self.forced = any(h > 2.0 for h in high)
            print(f"[frame] click at step {self.steps}: modules' heights over their seats {high} mm", flush=True)
        seated_shut = sum(1 for k, j in enumerate(near) if j is not None and seated(mods[j].pos, mods[j].tilt_deg(), k, 0.001, 3.0))
        pts = m.fingertips(env)  # all 24: both hands
        axes = [gk.along(s, (0.0, 0.0, 0.0), (0.0, 0.0, MOD[2]), 5) for s in mods]  # each module's axis; with the radius below, its 25 mm faces
        free = [gk.min_distance(pts, [a], radius=MOD[1] / 2) >= 0.005 for a in axes]  # let go: no fingertip within 5 mm of it
        seated_open = sum(1 for k, j in enumerate(near) if j is not None and free[j] and seated(mods[j].pos, mods[j].tilt_deg(), k, 0.0015, 5.0, OPEN_RISE))
        hands = min(gk.min_distance(pts, axes, radius=MOD[1] / 2),  # every module's faces
                    gk.min_distance(pts, FRAME_LINES))  # both walls, foot and top edge, end to end
        c = {"clicked": self.clicked, "dropped": self.dropped, "forced": self.forced, "fell": self.fell, "seated_shut": seated_shut, "seated_open": seated_open,
             "empty": sum(1 for j in near if j is None), "hands": hands}
        mo = moment(c)
        self.hold = self.hold + 1 if mo == 1.0 else 0
        self.full = self.hold >= HOLD_STEPS
        self.score = 1.0 if self.full else (0.6 if mo == 1.0 else mo)
        self.report = {"step": self.steps, "walls_deg": [round(a_l, 2), round(a_r, 2)], **c, "hands": round(hands, 3), "score": self.score,
                       "modules_mm_over_foot": [[round(1000 * float(v), 1) for v in (s.pos - np.array([F[0], F[1], TOP + STAND_H]))] for s in mods],
                       "tilt_deg": [round(s.tilt_deg(), 1) for s in mods], "near": near}
        if self.steps % 20 == 0:
            d = os.environ.get("FRAME_SCORE_DIR", "frame_scores")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"{os.environ.get('TASK_DEMO', 'run')}.json"), "w") as f:
                json.dump(self.report, f, indent=0, default=str)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo ------------------------------------------------------------------------------

GRIP = MOD[2] - 0.0085 + 0.013  # the TCP over a module's foot: fingertips 8.5 mm down its 34 mm, a 3.5 mm pinch at the top: the pads, wider than the 16 mm pitch, pass over the neighbours' tops, not against their sides
DROP = 0.0045  # let go with the module's foot this far over its shut seat (3.5 mm over the open edges): a short drop, the pads' undersides level with the open-seated neighbours' tops


def demo_script(ctx):
    """The right hand takes each module from the rack by its two lugged faces, at the top above the lugs, carries it in high over the modules already
    seated, lowers it to 4.5 mm over its seat on the open frame's support edges and lets go: D first, then C, B and A; it parks clear. The left hand comes
    down over the frame's middle with its fingers outside the two walls and closes at 20 N until the frame clicks, then lets go.

    Cheat checks (TASK_DEMO): `noclose` sets all four and never squeezes; `one` sets D only; `three` sets D, C and B, then squeezes; `low` carries each
    module in at a fixed 20 mm over its seat, as for a module with no tail, so its tail meets the open wall."""
    import demokit as d

    l, r = ctx.left, ctx.right
    g = d.down(0.0)
    up = np.array([0.0, 0.0, 1.0])
    for _ in range(5):
        yield
    mode = ctx.mode or "full"
    order = list(reversed(list(enumerate(MODULES))))[: {"one": 1, "three": 3}.get(mode, 4)]
    for k, name in order:  # D first, at the frame's far end: each later module is lowered beside seated ones, never carried low past them
        yield from r.pick(name, GRIP, width=MOD[1], grip=g, above=0.06, lift=0.06, force=8.0, label=f"take {name}")
        seat = seat_world(None, k) + np.array([0.0, 0.0, DROP])
        over = seat + ((TAIL_N * TAIL_SEG if mode != "low" else 0.0) + 0.02) * up  # high enough that the wire tail hanging under the module clears the walls' tops; it goes in first, straight down
        held = lambda name=name: ctx.pos(name)  # noqa: E731  (the module's foot, as the cameras see it)
        high = seat + (MOD[2] + TAIL_N * TAIL_SEG + 0.01) * up  # carried in over the seated modules' tops with its tail clear of them
        if mode != "low":
            yield from r.move(high + GRIP * up, t=1.5, label=f"set {name}: over, high")
        yield from r.move(over + GRIP * up, t=0.8, label=f"set {name}: over")
        yield from r.servo(over, read=held, tol=0.0005, timeout=2.0, label=f"set {name}: over")
        along = lambda: [round(1000 * float(ctx.pos(n)[0] - seat_world(None, j)[0]), 1) for j, n in enumerate(MODULES)]  # noqa: E731
        yield from r.servo(lambda t, over=over, seat=seat: over + (seat - over) * min(1.0, t / 2.4), read=held, tol=0.0006, timeout=4.0, label=f"set {name}: down")
        print(f"[frame] {name} down: along x {along()} mm", flush=True)
        yield from r.open(t=0.4, label=f"set {name}: let go")
        print(f"[frame] {name} let go: along x {along()} mm", flush=True)
        yield from r.move(r.at() + 0.06 * up, t=0.8, label=f"set {name}: up")
        print(f"[frame] {name} set: at {np.round(1000 * (ctx.pos(name) - seat_world(None, k)), 1)} mm from its seat; all along x: "
              f"{[round(1000 * float(ctx.pos(n)[0] - seat_world(None, j)[0]), 1) for j, n in enumerate(MODULES)]} mm", flush=True)
    yield from r.move(np.array([0.22, -0.20, TOP + 0.15]), t=1.5, label="park")
    if mode in ("noclose", "one"):
        yield from r.wait(2.0)
        print(f"[frame] no squeeze: {TRIAL.report}", flush=True)
        return
    mid = np.array([F[0], F[1], TOP + STAND_H + 0.005 + 0.013])  # fingertips 5 mm over the frame's foot: the finger's underside 10 mm up the walls
    yield from l.move(mid + 0.08 * up, grip=g, t=2.5, label="over the frame")
    yield from l.servo(mid + 0.08 * up, timeout=1.0, label="over the frame")
    yield from l.move(mid, t=1.2, label="fingers round the walls")
    yield from l.servo(mid, timeout=1.0, label="fingers round the walls")
    print(f"[frame] before the squeeze, along x: {[round(1000 * float(ctx.pos(n)[0] - seat_world(None, j)[0]), 1) for j, n in enumerate(MODULES)]} mm", flush=True)
    yield from l.close(2 * HINGE_Y, 20.0, t=1.5, label="squeeze")  # 20 N: the walls also steer the modules' lugs into their windows (a module may sit 1.3 mm off)
    yield from l.wait(0.5)
    print(f"[frame] after the squeeze: {TRIAL.report}", flush=True)
    yield from l.open(t=0.5, label="let go")
    yield from l.move(l.at() + 0.10 * up, t=1.0, label="up")
    yield from l.move(np.array([0.22, 0.20, TOP + 0.15]), t=1.0, label="park")
