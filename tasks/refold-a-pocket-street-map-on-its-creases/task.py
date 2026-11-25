"""Refold an opened laminated pocket street map back into its packet along its own accordion creases.

The map is a Michelin Streetwise laminated city map (Streetwise Chicago, ISBN 9782067259973): 4 x 8.5 in (10.2 x 21.6 cm) folded,
8.5 x 28 in (21.6 x 71.1 cm) open, 68 g, accordion folded into seven panels by six creases that alternate valley, mountain, valley ...
The lamination keeps every crease springy, so the opened map lies on the table as a shallow zigzag (each crease sprung back to about 60
degrees fold, each panel 30 degrees from level). Refolded right, each crease closes the way it was made and the cover panel ends up outside with its printed cover facing out.
A crease closed the wrong way puts the cover inside (crease 1) or makes a fold that will not lie flat; a laminated crease pushed the
wrong way springs back open.

Model (declared in code):
- Map ("map", one free articulation): seven rigid panels, each 10.16 x 21.59 cm x 1.5 mm and 9.7 g, the middle one (p4) the frame and
  the others on six hinges along the creases (x). Each hinge sits on the face that closes onto its neighbour, so a closed crease lays
  the two panels face to face. Joint zero is the opened map's sprung zigzag (60 degrees each). Joint limits: a crease closes to 180
  degrees (flat face to face) the right way, and can be bent up to 140 degrees the wrong way. Self-collision is off inside the
  articulation, so a closed stack is held by the joint limits, not by contact.
- Crease memory (code, each control step): a crease's spring pulls it back to its 60 degree set while it is open; once it has been
  closed past 160 degrees the right way it takes its folded set (target 185 degrees, i.e. pressed into its 180 degree stop) and keeps
  it until it is opened again below 110 degrees. A crease bent the wrong way always springs back toward its 60 degree set.
- Friction: the map's film slides at 0.35/0.3 and takes the lower friction at a contact (PhysX 'min' combine).

    python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import mechkit as m  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Fold this street map back up into its pocket-size packet, cover on the outside, and leave it lying flat on the table."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own hold
HOLD_STEPS = 20  # 1 s at 20 Hz
TOP = TABLE_TOP[SCENE]

# -- the map: Streetwise Chicago, 8.5 x 28 in open, 4 x 8.5 in folded, 68 g, seven panels ---------------------------------------------------
N = 7
W = 0.1016  # panel width across the creases (4 in), along world y
L = 0.2159  # panel length along the creases (8.5 in), along world x
T = 0.0015  # panel thickness in the model (the real laminated sheet is about 0.5 mm: see fidelity)
MASS = 0.068
PM = MASS / N  # 9.7 g a panel
MEM = 60.0  # degrees: each crease's sprung-open set
CLOSED_LIM = 180.0  # the right way: face to face
WRONG_LIM = 140.0  # the wrong way, from flat
K_CREASE = 0.03  # N m/rad: the lamination's springiness at a crease
SET_ON, SET_OFF = 160.0, 110.0  # degrees: a crease takes its folded set past SET_ON, and loses it below SET_OFF
FOLDED_TARGET = 185.0
VALLEY = [True, False, True, False, True, False]  # creases 1..6 from the cover panel, seen from the map side with the map open
XC, YC = 0.27, -0.05  # the middle panel's centre on the table


def _rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]])


def _layout():
    """Each panel's bottom-face origin (its -y edge) and angle in the (y, z) plane at joint zero, and each crease's hinge point."""
    a = math.radians(MEM)
    O, TH, H = [np.zeros(2)], [-a / 2], []
    for k in range(N - 1):
        o, th = O[k], TH[k]
        if VALLEY[k]:
            h = o + _rot(th) @ np.array([W, T])
            th2 = th + a
            o2 = h + _rot(th2) @ np.array([0.0, -T])
        else:
            h = o + _rot(th) @ np.array([W, 0.0])
            th2 = th - a
            o2 = h
        H.append(h)
        O.append(o2)
        TH.append(th2)
    corners = [O[k] + _rot(TH[k]) @ np.array(c) for k in range(N) for c in ((0, 0), (W, 0), (0, T), (W, T))]
    zmin = min(c[1] for c in corners)
    centre = O[3] + _rot(TH[3]) @ np.array([W / 2, T / 2])
    shift = np.array([centre[0], centre[1]])
    O = [o - shift for o in O]
    H = [h - shift for h in H]
    return O, TH, H, centre[1] - zmin


P_ORIGIN, P_THETA, HINGE, Z_CENTRE = _layout()
# closing direction of each crease's joint (+1: closing is a positive joint angle); the left branch (creases 4-6) has its child on +y
CLOSE_SIGN = [-1, +1, -1, -1, +1, -1]
LINK_OF_CREASE = ["p1", "p2", "p3", "p5", "p6", "p7"]
PARENT = {"p3": None, "p2": "p3", "p1": "p2", "p5": None, "p6": "p5", "p7": "p6"}

LOOKS = {
    "sheet": m.Look(color=m.rgb(0.93, 0.92, 0.86), roughness=0.25),
    "park": m.Look(color=m.rgb(0.62, 0.80, 0.52), roughness=0.25),
    "water": m.Look(color=m.rgb(0.55, 0.74, 0.90), roughness=0.2),
    "road": m.Look(color=m.rgb(0.98, 0.83, 0.35), roughness=0.25),
    "street": m.Look(color=m.rgb(0.78, 0.76, 0.72), roughness=0.25),
    "cover": m.Look(color=m.rgb(0.80, 0.12, 0.10), roughness=0.2),
    "cover_band": m.Look(color=m.rgb(0.98, 0.95, 0.90), roughness=0.2),
    "back": m.Look(color=m.rgb(0.20, 0.30, 0.55), roughness=0.2),
}


def _panel_meshes(k: int):
    """Panel k (0-based) as seen: the sheet, a printed map on its top face, and (k = 0) the cover on its bottom face, k = 6 the back."""
    th = math.degrees(P_THETA[k])
    o = P_ORIGIN[k]

    def place(mesh):
        return mesh.turned(th, "x").moved((0.0, o[0], o[1]))

    rng = np.random.default_rng(11 + k)
    parts = [(f"sheet{k}", place(m.box((L, W, T), (0.0, W / 2, T / 2))), "sheet")]
    up = T + 0.0002
    parts.append((f"park{k}", place(m.box((0.05, 0.03, 0.0002), (rng.uniform(-0.07, 0.07), rng.uniform(0.02, 0.08), up))), "park"))
    for j, xx in enumerate(np.linspace(-0.09, 0.09, 6)):
        parts.append((f"st{k}_{j}", place(m.box((0.0015, W - 0.006, 0.0002), (xx, W / 2, up))), "street"))
    for j, yy in enumerate(np.linspace(0.012, W - 0.012, 4)):
        parts.append((f"av{k}_{j}", place(m.box((L - 0.006, 0.0015, 0.0002), (0.0, yy, up))), "street"))
    parts.append((f"rd{k}", place(m.box((L - 0.006, 0.004, 0.0003), (0.0, W * (0.3 + 0.4 * rng.random()), up + 0.0001))), "road"))
    if k in (5, 6):
        parts.append((f"lake{k}", place(m.box((L - 0.006, 0.025, 0.0002), (0.0, 0.06 if k == 5 else 0.02, up))), "water"))
    if k == 0:  # the printed cover on the bottom face of panel 1, with its title band
        parts.append(("cover", place(m.box((L - 0.002, W - 0.002, 0.0002), (0.0, W / 2, -0.0002))), "cover"))
        parts.append(("cover_band", place(m.box((0.045, W - 0.014, 0.0002), (0.06, W / 2, -0.0004))), "cover_band"))
    if k == 6:
        parts.append(("back", place(m.box((L - 0.002, W - 0.002, 0.0002), (0.0, W / 2, -0.0002))), "back"))
    return parts


def _panel_collider(k: int) -> str:
    """Panel k's collider: a box primitive turned about x to the panel's angle (a thin convex hull would not cook for the GPU)."""
    th = P_THETA[k]
    c = P_ORIGIN[k] + _rot(th) @ np.array([W / 2, T / 2])
    return f"""
    def Cube "panel{k}_c" {m._COL_HEAD}
        double size = 1
        double3 xformOp:translate = (0.0, {c[0]:.6f}, {c[1]:.6f})
        quatf xformOp:orient = ({math.cos(th / 2):.7f}, {math.sin(th / 2):.7f}, 0.0, 0.0)
        float3 xformOp:scale = ({L:.6f}, {W:.6f}, {T:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }}"""


PANEL_INERTIA = (PM * (W * W + T * T) / 12, PM * (L * L) / 12, PM * (L * L + W * W) / 12)


def _map():
    links = []
    for c in range(N - 1):
        name = LINK_OF_CREASE[c]
        k = int(name[1]) - 1
        s = CLOSE_SIGN[c]
        lo, hi = (-WRONG_LIM - MEM, CLOSED_LIM - MEM) if s > 0 else (-(CLOSED_LIM - MEM), WRONG_LIM + MEM)
        h = HINGE[c]
        com = P_ORIGIN[k] + _rot(P_THETA[k]) @ np.array([W / 2, T / 2])
        links.append(m.hinge(name, "X", (0.0, h[0], h[1]), lo, hi, _panel_meshes(k), [_panel_collider(k)], mass=PM, spring=K_CREASE,
                             damping=0.004, friction=0.0005, effort=2.0, armature=0.0002, gravity=True, com=(0.0, com[0], com[1]),
                             inertia=PANEL_INERTIA, parent=PARENT[name]))
    return m.mechanism("map", (XC, YC, TOP + Z_CENTRE + 0.0005), links, LOOKS, frame_parts=_panel_meshes(3), frame_colliders=[_panel_collider(3)],
                       friction=(0.35, 0.3), solver_iters=48, free_mass=PM, frame_inertia=PANEL_INERTIA)


MAP = _map()
MAP_FRICTION = (0.35, 0.3)  # laminated (polypropylene) film on a table top


def _laminate(build):
    """The map's surface takes the lower of the two frictions at a contact (PhysX 'min' combine), so that laminated film on the table
    slides at 0.3 rather than at the average with the table's rubber-like 1.0."""
    def built() -> str:
        text = build()
        old = '        prepend apiSchemas = ["PhysicsMaterialAPI"]\n    )'
        assert old in text
        return text.replace(old, '        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n    )', 1).replace(
            "        float physics:restitution = 0", '        float physics:restitution = 0\n        uniform token physxMaterial:frictionCombineMode = "min"', 1)
    return built


object.__setattr__(MAP, "build", _laminate(MAP.build)) if hasattr(MAP, "build") else None

# -- distractors (SimReady, loose): reading glasses and an empty paper coffee cup ----------------------------------------------------------------
GLASSES = SimReady("glasses", pos=(0.53, 0.14, TOP), size=(0.145, 0.174, 0.057), mass=0.03, package="accessory_eyeglasses_a01",
                   usd="sm_accessory_eyeglasses_a01_01.usd", yaw=1.9)
CUP = SimReady("paper_cup", pos=(0.50, -0.20, TOP + 0.001), size=(0.080, 0.080, 0.110), mass=0.01, package="dishware_cup_paper_a01",
               usd="sm_dishware_cup_paper_a01_01.usd")

OBJECTS = [MAP, GLASSES, CUP]


# -- reading the map -----------------------------------------------------------------------------------------------------------------------------


def fold_deg(state, c: int) -> float:
    """Crease c (0-based) folded from flat, in degrees: 180 closed the right way, 0 flat, negative bent the wrong way."""
    return MEM + CLOSE_SIGN[c] * state.deg(LINK_OF_CREASE[c])


def folds(state) -> list[float]:
    return [fold_deg(state, c) for c in range(N - 1)]


def _body_index(asset, k: int) -> int:
    return asset.body_names.index("frame" if k == 3 else f"p{k + 1}")


def panels(env):
    """Each panel's world pose: list of (R 3x3, p 3) for the part frame carried by that panel's body, plus each body's linear speed."""
    base = env.unwrapped
    asset = base.scene["map"]
    origin = m._np(base.scene.env_origins[0])
    pose = m._np(asset.data.body_link_pose_w)[0]
    vel = m._np(asset.data.body_link_vel_w)[0]
    out = []
    for k in range(N):
        i = _body_index(asset, k)
        out.append((m.quat_matrix(pose[i, 3:7]), pose[i, :3] - origin, float(np.linalg.norm(vel[i, :3])), float(np.linalg.norm(vel[i, 3:6]))))
    return out


def panel_corners(R, p, k: int) -> np.ndarray:
    o, th = P_ORIGIN[k], P_THETA[k]
    pts = []
    for x in (-L / 2, L / 2):
        for u, v in ((0, 0), (W, 0), (0, T), (W, T)):
            yz = o + _rot(th) @ np.array([u, v])
            pts.append((x, yz[0], yz[1]))
    return np.asarray(pts) @ R.T + p


def panel_normal(R, k: int) -> np.ndarray:
    th = P_THETA[k]
    return R @ np.array([0.0, -math.sin(th), math.cos(th)])


def panel_gap(R, p, k: int, pts: np.ndarray) -> np.ndarray:
    """Distance (m) from each point to panel k's box."""
    local = (pts - p) @ R  # into the part frame as authored
    o, th = P_ORIGIN[k], P_THETA[k]
    yz = (local[:, 1:3] - o) @ _rot(th)  # into the panel's (u, v)
    d = np.stack([np.maximum(np.abs(local[:, 0]) - L / 2, 0.0), np.maximum(np.maximum(-yz[:, 0], yz[:, 0] - W), 0.0),
                  np.maximum(np.maximum(-yz[:, 1], yz[:, 1] - T), 0.0)], axis=1)
    return np.linalg.norm(d, axis=1)


CLOSED_DEG = 165.0  # a crease counts as closed the right way at 165 degrees or more (panels within 15 degrees of face to face)
WRONG_DEG = -90.0  # and as bent the wrong way at 90 degrees or more the wrong way
FLAT_DEG = 15.0  # every panel within 15 degrees of level
FLAT_H = 0.03  # every corner of every panel within 3 cm of the table top
CLEAR = 0.04  # every fingertip point at least 4 cm from every panel
STILL = 0.02  # m/s
STILL_W = 0.2  # rad/s
OFF_TABLE = 0.03  # a panel's corner more than 3 cm below the table top: the map has gone over the edge


def now(env) -> dict:
    st = m.state(env, "map")
    f = folds(st)
    ps = panels(env)
    tips = m.fingertips(env)
    corners = np.vstack([panel_corners(R, p, k) for k, (R, p, _, _) in enumerate(ps)])
    tilt = max(math.degrees(math.acos(min(1.0, abs(float(panel_normal(R, k)[2]))))) for k, (R, _, _, _) in enumerate(ps))
    gap = min(float(panel_gap(R, p, k, tips).min()) for k, (R, p, _, _) in enumerate(ps))
    closed = [x >= CLOSED_DEG for x in f]
    return {"folds": f, "n_closed": sum(closed), "all_closed": all(closed), "wrong": any(x <= WRONG_DEG for x in f),
            "tilt": tilt, "zmax": float(corners[:, 2].max() - TOP), "zmin": float(corners[:, 2].min() - TOP),
            "flat": tilt <= FLAT_DEG and float(corners[:, 2].max() - TOP) <= FLAT_H, "gap": gap, "clear": gap >= CLEAR,
            "still": all(v < STILL and w < STILL_W for _, _, v, w in ps), "off": float(corners[:, 2].min()) < TOP - OFF_TABLE}


class Creases:
    """Each crease's set: open (spring to its 40 degree memory) or folded (spring into its closed stop), with hysteresis."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):
        self.folded = [False] * (N - 1)
        if env is not None:
            for c in range(N - 1):
                m.set_target(env, "map", LINK_OF_CREASE[c], 0.0)

    def step(self, env):
        st = m.state(env, "map")
        for c in range(N - 1):
            f = fold_deg(st, c)
            want = self.folded[c]
            if not want and f > SET_ON:
                want = True
            elif want and f < SET_OFF:
                want = False
            if want != self.folded[c]:
                self.folded[c] = want
                target = math.radians(FOLDED_TARGET - MEM) * CLOSE_SIGN[c] if want else 0.0
                m.set_target(env, "map", LINK_OF_CREASE[c], target)


CREASES = Creases()
m.on_step(CREASES.step)
m.on_reset(CREASES.reset)


class Trial:
    """The score, and the record of the map going over the table's edge (cleared only at a reset)."""

    def __init__(self):
        self.reset()

    def reset(self, env=None):  # the only place the record is cleared
        self.hold = 0
        self.full = False
        self.score = 0.0
        self.dropped = False
        self.last = {}

    def step(self, env):
        c = now(env)
        self.last = c
        if c["off"]:
            self.dropped = True
        ok = c["all_closed"] and c["flat"] and c["clear"] and c["still"] and not self.dropped
        self.hold = self.hold + 1 if ok else 0
        self.full = self.full or self.hold >= HOLD_STEPS
        if self.dropped:
            self.score = 0.0
        elif self.full:
            self.score = 1.0
        elif c["all_closed"] and c["clear"]:
            self.score = 0.5
        elif c["n_closed"] >= 3 and not c["wrong"] and c["clear"]:
            self.score = 0.25
        else:
            self.score = 0.0


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return bool(TRIAL.full)


def score(objects) -> float:
    return float(TRIAL.score)


# -- the demonstration -----------------------------------------------------------------------------------------------------------------------


def _report(ctx, env, label=""):
    c = now(env)
    print(f"[map] t {ctx.t:5.1f} {label}folds {' '.join(f'{x:6.1f}' for x in c['folds'])} closed {c['n_closed']} tilt {c['tilt']:.1f} "
          f"z {c['zmin'] * 1000:.0f}..{c['zmax'] * 1000:.0f} mm gap {c['gap'] * 1000:.0f} mm still {c['still']} score {TRIAL.score:.2f} "
          f"full {TRIAL.full} dropped {TRIAL.dropped}", flush=True)


def crease_yz(env, c: int) -> np.ndarray:
    """Crease c's (0-based) line, seen as (y, z) in the world: the edge panel c shares with panel c + 1."""
    R, p, _, _ = panels(env)[c]
    return panel_corners(R, p, c)[[1, 3, 5, 7]][:, 1:3].mean(axis=0)


def demo_script(ctx):
    """Gathering the map like a concertina between two hands. First the right gripper (fingers closed, pointing down) pushes the cover
    panel's raised outer edge in until the panel turns up over panel 2 and crease 1 shuts. Then both grippers, leaned 45 degrees in
    towards the map, push the two ends (the folded cover pair at crease 1, and the far panel's edge) in towards each other, low on the
    end panels (a quarter of their height), so that every ridge rises and every valley stays on the table and each crease closes the
    way it was made; they stop when every crease is seen shut. The packet then stands on its edge between them. The left hand lifts
    away and the right pushes the packet's top over towards +y, so that it lies on its back panel with the cover up, and moves clear.
    It reads only the map's pose: the panels and their crease angles, which the top camera shows.
    Failure checks (--mode): none does nothing; careless pushes both ends in at once at table height with no cover fold first (the map
    arches up); onearm pushes the far end in with the left hand only; stand gathers the packet and leaves it standing on its edge;
    hold does the full job but keeps the right gripper pressing on the packet."""
    import demokit as d

    env = ctx.emb._env
    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode not in ("full", "none", "careless", "onearm", "stand", "hold"):
        raise ValueError(mode)
    for _ in range(4):
        yield
    _report(ctx, env, f"{mode} start: ")
    if mode == "none":
        for _ in range(80):
            yield
        _report(ctx, env, "end: ")
        return

    TIP = 0.013  # the pads below the TCP
    HALF = 0.009  # half the closed fingers' thickness along y
    LOW = TOP + 0.0015 + TIP
    UP = 0.12
    EDGE = 0.008  # push the cover panel's edge this far below its top
    FRAC = 0.28  # push the gathering ends this far up their end panels
    LEAN = 42.0
    SPD = 0.02  # m/s per hand
    down = d.down(0.0)
    xm = float(np.mean([p[0] for _, p, _, _ in panels(env)]))

    def end_y(k, side):
        R, p, _, _ = panels(env)[k]
        ys = panel_corners(R, p, k)[:, 1]
        return float(ys.min() if side < 0 else ys.max())

    def height(k):
        R, p, _, _ = panels(env)[k]
        return float(panel_corners(R, p, k)[:, 2].max() - TOP)

    y_lo, y_hi = end_y(0, -1), end_y(6, +1)
    gens = [l.move((xm, y_hi + HALF + 0.004, TOP + UP), grip=down, finger=0.0, t=2.0, label="left over the far end")]
    if mode != "onearm":
        gens.append(r.move((xm, y_lo - HALF - 0.004, TOP + UP), grip=down, finger=0.0, t=2.0, label="right over the cover end"))
    yield from ctx.together(*gens)
    gens = [l.move((xm, y_hi + HALF + 0.004, LOW), t=1.0, label="left down")]
    if mode != "onearm":
        gens.append(r.move((xm, y_lo - HALF - 0.004, LOW), t=1.0, label="right down"))
    yield from ctx.together(*gens)

    if mode in ("careless", "onearm"):  # both ends (or only the far end) pushed in at table height, no cover fold first
        mid = 0.5 * (y_lo + y_hi)
        for k in range(500):
            if min(folds(ctx.state("map"))) >= 172.0:
                break
            gens = [l.move((xm, max(l.at()[1] - SPD / 20, (mid if mode == "careless" else y_lo + 0.10) + HALF + 3.5 * T), LOW), t=0.05, label="push in")]
            if mode == "careless":
                gens.append(r.move((xm, min(r.at()[1] + SPD / 20, mid - HALF - 3.5 * T), LOW), t=0.05, label="push in"))
            yield from ctx.together(*gens)
            if k % 60 == 0:
                _report(ctx, env, "pushing: ")
        _report(ctx, env, "pushed: ")
        gens = [l.move((xm, l.at()[1] + 0.03, TOP + 0.20), t=1.5, label="left away")]
        if mode == "careless":
            gens.append(r.move((xm, r.at()[1] - 0.03, TOP + 0.20), t=1.5, label="right away"))
        yield from ctx.together(*gens)
        for k in range(60):
            yield
        _report(ctx, env, "end: ")
        return

    # 1. the cover panel turned up over panel 2 by its raised edge: crease 1 shuts
    for k in range(300):
        if folds(ctx.state("map"))[0] >= 172.0:
            break
        z = TOP + float(np.clip(height(0) - EDGE, 0.0015, 0.09)) + TIP
        yield from r.move((xm, r.at()[1] + SPD / 20, z), t=0.05, label="cover panel over")
        if k % 40 == 0:
            _report(ctx, env, "cover over: ")
    _report(ctx, env, "cover over: ")
    c1 = crease_yz(env, 0)
    yield from r.move((xm, r.at()[1] - 0.02, TOP + UP), t=0.8, label="right up")
    yield from r.move((xm, float(c1[0]) - HALF - 0.006, TOP + UP), t=0.8, label="right over crease 1")
    yield from r.move((xm, float(c1[0]) - HALF - 0.006, LOW), t=0.8, label="right down at crease 1")

    # 2. both ends pushed in together, leaned in, low on the end panels, until every crease is seen shut
    mid = 0.5 * (float(c1[0]) + end_y(6, +1))
    yield from ctx.together(r.move(r.at(), grip=d.tilted(LEAN, 90.0), t=0.8, label="right leans in"),
                            l.move(l.at(), grip=d.tilted(LEAN, -90.0), t=0.8, label="left leans in"))
    for k in range(700):
        if min(folds(ctx.state("map"))) >= 172.0:
            break
        yl_, yr_ = l.at()[1], r.at()[1]
        if yl_ - yr_ <= 2 * HALF + N * T + 0.002:
            break
        zl = TOP + float(np.clip(FRAC * height(6), 0.0015, 0.04)) + TIP
        zr = TOP + float(np.clip(FRAC * height(1), 0.0015, 0.04)) + TIP
        yield from ctx.together(l.move((xm, max(yl_ - SPD / 20, mid + HALF + 0.5 * N * T), zl), t=0.05, label="gather"),
                                r.move((xm, min(yr_ + SPD / 20, mid - HALF - 0.5 * N * T), zr), t=0.05, label="gather"))
        if k % 60 == 0:
            _report(ctx, env, "gathering: ")
    yield from ctx.together(l.wait(0.5), r.wait(0.5))
    _report(ctx, env, "gathered: ")
    # 2b. if a valley is still seen open at the bottom of the stack, both hands squeeze lower down, a few millimetres more
    for k in range(80):
        if min(folds(ctx.state("map"))) >= 172.0:
            break
        yl_, yr_ = l.at()[1], r.at()[1]
        if yl_ - yr_ <= 2 * HALF + N * T - 0.004:
            break
        yield from ctx.together(l.move((xm, yl_ - 0.0005, TOP + 0.004 + TIP), t=0.05, label="squeeze low"),
                                r.move((xm, yr_ + 0.0005, TOP + 0.004 + TIP), t=0.05, label="squeeze low"))
    yield from ctx.together(l.wait(0.3), r.wait(0.3))
    _report(ctx, env, "squeezed: ")

    if mode == "stand":  # both hands away, the packet left standing on its edge
        yield from ctx.together(l.move(l.at() + np.array([0.0, 0.03, 0.0]), t=0.6, label="left off"), r.move(r.at() + np.array([0.0, -0.03, 0.0]), t=0.6, label="right off"))
        yield from ctx.together(l.move((xm, 0.25, TOP + 0.22), grip=down, t=1.5, label="left clear"), r.move((xm, -0.25, TOP + 0.22), grip=down, t=1.5, label="right clear"))
        for k in range(60):
            yield
        _report(ctx, env, "end: ")
        return

    # 3. the left hand away; the right pushes the packet's top over towards +y: it lies on its back panel, cover up
    yc = 0.5 * (l.at()[1] + r.at()[1])
    if mode == "hold":  # the right hand goes onto the packet as the left lets go, and keeps pressing on it
        yield from ctx.together(l.move(l.at() + np.array([0.0, 0.03, 0.0]), t=0.5, label="left off the packet"),
                                r.move((xm, yc + 0.02, TOP + 0.05 + TIP), t=0.5, label="right onto the packet"))
        yield from ctx.together(l.move((xm, yc + 0.22, TOP + 0.20), grip=down, t=1.2, label="left away"),
                                r.move((xm, yc + 0.05, TOP + 0.010 + TIP), t=1.2, label="press on the packet"))
        for k in range(80):
            yield
        _report(ctx, env, "end: ")
        return
    yield from l.move(l.at() + np.array([0.0, 0.03, 0.0]), t=0.5, label="left off the packet")
    yield from l.move((xm, yc + 0.22, TOP + 0.20), grip=down, t=1.2, label="left away")
    yield from r.move((xm, r.at()[1] - 0.01, TOP + 0.075 + TIP), t=1.0, label="right up the packet's face")
    yield from r.move((xm, yc + 0.07, TOP + 0.06 + TIP), t=1.5, label="tip the packet over")
    _report(ctx, env, "tipped: ")
    yield from r.move((xm, yc + 0.07, TOP + 0.20), t=1.0, label="right up")
    yield from r.move((xm, -0.22, TOP + 0.22), grip=down, t=1.2, label="right clear")
    for k in range(60):
        yield
        if k % 30 == 29:
            _report(ctx, env, "end: ")
