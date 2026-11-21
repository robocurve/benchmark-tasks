"""Scoop a ball of ice cream from a tub with a Zeroll scoop and set it on a sugar cone held upright in the other hand.

A 1.42 L ice cream tub (full, lid off) stands left of centre, a Zeroll 1020 #20 scoop lies on a plate to its right, and a JOY #310 sugar cone
stands in a wire cone stand on the far left. The right hand takes the scoop by its handle's end and draws the bowl about 1 cm deep through the
ice cream for over 8 cm, so a ball forms in the bowl; it lifts the ball clear. The left hand takes the cone out of its stand and holds it upright.
The right hand turns the bowl over the cone's mouth and the ball drops onto it; the cone stays in the left hand.

    python demokit.py task.py OUT --mode full
    python demokit.py task.py OUT --no-record --stills 40 --mode full
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import gatekit as gk  # noqa: E402,F401
import mechkit as m  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Scoop a ball of ice cream from the tub with the scoop and put it on the sugar cone. Hold the cone upright in your other hand, and don't crack it."
SUCCESS_HOLD_S = 0.0  # score() and success() agree: the scorer counts its own 3 s hold
HOLD_STEPS = 60  # 3 s at 20 Hz
TOP = TABLE_TOP[SCENE]
G = 9.81

# -- the tub: a 1.42 L (48 fl oz) rectangular supermarket tub, about 18 x 13 x 9 cm (estimate), full, lid off
TUB_C = np.array([0.40, -0.12])
TUB_HX, TUB_HY, TUB_H, TUB_WALL = 0.09, 0.065, 0.09, 0.002
ICE_Z = 0.085  # the ice cream's surface, 5 mm under the rim
# the ice cream is not a body: a rule stands in for it (see fidelity)
DRAG_K = 300.0  # N per m of depth: 0.3 N per mm (estimate; about 3 N at the 1 cm depth: a 2.5 to 8 kPa yield stress over about 5 cm2 of rim gives 1.3 to 4 N)
DEPTH_MIN, DEPTH_MAX = 0.006, 0.015  # the bowl's lowest point 6 to 15 mm under the surface forms a ball
TRAVEL = 0.08  # 8 cm of drag at that depth
V_DROP = 1.2  # m/s: a ball that moves faster than this between leaving the scoop and reaching the cone (the arrival step itself is not counted) is thrown or dropped, not served: the score is capped at 0.3
V_DRAW = 0.01  # travel counts only while the lowest point moves 1 cm/s or more, averaged over the last half second, where the full drag acts
DEPTH_CAP = 0.03

# -- the scoop: Zeroll 1020 #20, 2 oz, aluminium, about 184 mm long; its frame: the bowl's centre at the origin, handle along +x, the bowl opening +z
BOWL_R, BOWL_T = 0.0255, 0.0015  # 51 mm across outside
HANDLE_R, HANDLE_X0, HANDLE_X1, HANDLE_Z = 0.0125, 0.020, 0.1585, -0.013  # a 25 mm handle, flush with the bowl's underside; 184 mm over all
SCOOP_M = 0.15
HANDLE_GRIP_X = 0.040  # the grip on the handle, from its rear end
SCOOP_AT = np.array([0.40, -0.40])
SCOOP_YAW = 180.0  # the handle toward the robot, the bowl away
PLATE_R, PLATE_H = 0.13, 0.003
PLATE_C = np.array([0.32, -0.40])

# -- the ice cream ball: a #20 scoop is 1/20 quart (47 ml), a 44 mm ball; 40 g (estimate)
BALL_R, BALL_M = 0.022, 0.040
BALL_PARK = np.array([TUB_C[0] - 0.05, TUB_C[1] + 0.04, BALL_R + 0.004])  # inside the full tub under the surface, out of the scoop's reach, until a ball forms

# -- the cone: JOY #310 sugar cone, 117 mm tall, 48 mm top diameter (43 mm inside); its frame: the tip at the origin, the mouth up along +z
CONE_H, CONE_R, CONE_RI, CONE_M = 0.117, 0.024, 0.0215, 0.012
STAND_C = np.array([0.40, 0.10])
# a single-cone wire stand modelled on the RAK Porcelain PIRC08 (stainless steel rod): a 79 mm (3 1/8 in) base ring of 4 mm rod, one slanted 4 mm rod,
# and a narrow top ring (a 6 mm band) 22 mm across inside, its top 60 mm up (top ring and height estimated from the maker's photo)
STAND_H, STAND_HOLE, STAND_BAND, STAND_WALL = 0.060, 0.011, 0.006, 0.0015
STAND_BASE_R, STAND_ROD = 0.0395, 0.002  # base ring radius (to the rod's axis) and rod radius
CONE_SEAT = 0.0112 / CONE_R * CONE_H  # the tip's height over the cone's seat: where the cone is 22 mm across
CONE_TIP_Z = STAND_H - CONE_SEAT + 0.0008  # tip height at rest
CRACK_N = 20.0  # a finger force on the cone over this cracks it (estimate)

# -- the scorer's thresholds
BALL_BOWL = 0.012  # the ball's centre within 12 mm of where it sits in the bowl
CLEAR = 0.010  # the bowl's lowest point 1 cm over the tub's rim
UPRIGHT_COS = math.cos(math.radians(20.0))
LIFT_CONE = 0.06  # the cone 6 cm up out of its stand
FINGER_NEAR = 0.008

LOOKS = {
    "tub": m.Look(color=m.rgb(0.92, 0.92, 0.90), roughness=0.5),
    "lid_band": m.Look(color=m.rgb(0.25, 0.30, 0.55), roughness=0.5),
    "cream": m.Look(color=m.rgb(0.96, 0.92, 0.80), roughness=0.85),
    "ball": m.Look(color=m.rgb(0.96, 0.92, 0.80), roughness=0.8),
    "alu": m.Look(color=m.rgb(0.72, 0.73, 0.75), metallic=0.9, roughness=0.3),
    "handle": m.Look(color=m.rgb(0.62, 0.63, 0.66), metallic=0.85, roughness=0.4),
    "gold": m.Look(color=m.rgb(0.83, 0.62, 0.25), metallic=0.8, roughness=0.35),
    "cone": m.Look(color=m.rgb(0.78, 0.57, 0.28), roughness=0.9),
    "steel": m.Look(color=m.rgb(0.70, 0.71, 0.73), metallic=0.9, roughness=0.35),
    "plate": m.Look(color=m.rgb(0.95, 0.95, 0.93), roughness=0.4),
}


def _bx(x, y, z) -> m.Mesh:
    return m.box((x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


def _cbox(name, x, y, z) -> str:
    return m.col_box(name, (x[1] - x[0], y[1] - y[0], z[1] - z[0]), ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2))


# -- the tub ------------------------------------------------------------------------------------------------------------------------

def _tub():
    cx, cy = TUB_C
    x, y, w, h = (cx - TUB_HX, cx + TUB_HX), (cy - TUB_HY, cy + TUB_HY), TUB_WALL, TUB_H
    walls = {"wx0": ((x[0], x[0] + w), y, (0.0, h)), "wx1": ((x[1] - w, x[1]), y, (0.0, h)),
             "wy0": (x, (y[0], y[0] + w), (0.0, h)), "wy1": (x, (y[1] - w, y[1]), (0.0, h)), "floor": (x, y, (0.0, 0.003))}
    parts = [(n, _bx(*b), "tub") for n, b in walls.items()]
    parts.append(("band", _bx((x[0] - 0.0005, x[1] + 0.0005), (y[0] - 0.0005, y[1] + 0.0005), (0.060, 0.080)), "lid_band"))
    parts.append(("cream", _bx((x[0] + w, x[1] - w), (y[0] + w, y[1] - w), (0.003, ICE_Z)), "cream"))
    cols = [_cbox(f"{n}_c", *b) for n, b in walls.items()]
    return m.fixed("tub", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.4))


def _plate():
    c = (PLATE_C[0], PLATE_C[1], PLATE_H / 2)
    return m.fixed("plate", (0.0, 0.0, TOP), [("plate", m.cylinder(PLATE_R, PLATE_H, c), "plate")], [m.col_cylinder("plate_c", PLATE_R, PLATE_H, c)], LOOKS, friction=(0.6, 0.5))


def _stand():
    cx, cy = STAND_C
    h, ri, bh, w, rb, rr = STAND_H, STAND_HOLE, STAND_BAND, STAND_WALL, STAND_BASE_R, STAND_ROD
    th = np.linspace(0.0, 2 * math.pi, 49)
    base = [(cx + rb * math.cos(t), cy + rb * math.sin(t), rr) for t in th]
    band = m.lathe([(ri, h - bh), (ri + w, h - bh), (ri + w, h), (ri, h)], 32).moved((cx, cy, 0.0))
    # the rod rises from the base ring's far side (+x, away from the robot) to the top ring's far side
    r0, r1 = np.array([cx + rb, cy, rr]), np.array([cx + ri + w + rr, cy, h - bh / 2])
    rod = [r0 + (r1 - r0) * k / 8 for k in range(9)]
    parts = [("base_ring", m.tube(base, rr, 10), "steel"), ("top_ring", band, "steel"), ("rod", m.tube(rod, rr, 10), "steel")]
    # colliders: the top ring as 16 thin boxes round the hole (the cone sits on its inner top edge), the rod and the base ring as short hulls
    cols = []
    n = 16
    for k in range(n):
        a = 2 * math.pi * k / n
        c = (cx + (ri + w / 2) * math.cos(a), cy + (ri + w / 2) * math.sin(a))
        seg = 2 * (ri + w) * math.tan(math.pi / n)
        box = m.box((w, seg, bh)).turned(math.degrees(a), "z").moved((c[0], c[1], h - bh / 2))
        cols.append(m.col_hull(f"ring{k}_c", box))
    for k in range(len(rod) - 1):
        cols.append(m.col_hull(f"rod{k}_c", m.tube(rod[k:k + 2], rr, 6)))
    for k in range(0, len(base) - 2, 2):
        cols.append(m.col_hull(f"base{k}_c", m.tube(base[k:k + 3], rr, 6)))
    return m.fixed("stand", (0.0, 0.0, TOP), parts, cols, LOOKS, friction=(0.5, 0.4))


# -- the scoop ----------------------------------------------------------------------------------------------------------------------

def _bowl_patch(a0, a1, p0, p1, ri, ro) -> m.Mesh:
    """A convex piece of the bowl's wall: azimuth a0..a1, polar angle p0..p1 (0 at the bottom), inner and outer radius."""
    # the inner corners sit out by the chord's sag, so the hull's flat inner face is nowhere inside the bowl's real inner surface
    ri = ri / (math.cos((a1 - a0) / 2) * math.cos((p1 - p0) / 2))
    pts = [(r * math.sin(p) * math.cos(a), r * math.sin(p) * math.sin(a), -r * math.cos(p)) for r in (ri, ro) for p in (p0, p1) for a in (a0, a1)]
    pts = np.array(pts)
    return m.Mesh(pts, np.array([(0, 1, 2)]))  # col_hull uses the points only


def _scoop():
    prof_o = [(BOWL_R * math.sin(p), -BOWL_R * math.cos(p)) for p in np.linspace(0.0, math.pi / 2, 9)]
    ri = BOWL_R - BOWL_T
    prof_i = [(ri * math.sin(p), -ri * math.cos(p)) for p in np.linspace(math.pi / 2, 0.0, 9)]
    bowl = m.lathe([(0.0, -BOWL_R)] + prof_o[1:] + prof_i[:-1] + [(0.0, -ri)], 32)
    hl = HANDLE_X1 - HANDLE_X0
    handle = m.cylinder(HANDLE_R, hl - 0.003, ((HANDLE_X0 + HANDLE_X1 - 0.003) / 2, 0.0, HANDLE_Z), axis="x")
    cap = m.cylinder(HANDLE_R - 0.0015, 0.003, (HANDLE_X1 - 0.0015, 0.0, HANDLE_Z), axis="x")  # the 1020's gold end cap (seen only)
    cols = []
    nseg, nband = 16, 4
    for i in range(nseg):
        for b in range(nband):
            p0, p1 = (math.pi / 2) * b / nband, (math.pi / 2) * (b + 1) / nband
            a0, a1 = 2 * math.pi * i / nseg, 2 * math.pi * (i + 1) / nseg
            cols.append(m.col_hull(f"bowl{i}_{b}", _bowl_patch(a0, a1, p0, p1, ri, BOWL_R + 0.002)))  # the collider's wall 2 mm thicker outside, so the GPU's hull needs no thickening
    # the handle's collider starts at the bowl's rim: from 20 mm its end would stand 4 mm into the bowl's hollow and wedge the ball there
    cx0 = BOWL_R - BOWL_T + 0.0005
    cols.append(m.col_hull("handle_c", m.cylinder(HANDLE_R, HANDLE_X1 - cx0, ((cx0 + HANDLE_X1) / 2, 0.0, HANDLE_Z), axis="x")))
    return m.free("scoop", (SCOOP_AT[0], SCOOP_AT[1], TOP + PLATE_H + BOWL_R + 0.0005), [("bowl", bowl, "alu"), ("handle", handle, "handle"), ("cap", cap, "gold")], cols, LOOKS,
                  mass=SCOOP_M, friction=(0.7, 0.6), yaw_deg=SCOOP_YAW)


def bowl_points() -> np.ndarray:
    """Points over the bowl's outside, in the scoop's frame: the dome and the rim."""
    pts = [(BOWL_R * math.sin(p) * math.cos(a), BOWL_R * math.sin(p) * math.sin(a), -BOWL_R * math.cos(p))
           for p in np.linspace(0.0, math.pi / 2, 6) for a in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
    return np.array(pts)


BOWL_PTS = bowl_points()


def bowl_skin() -> np.ndarray:
    """Dense points over the bowl's inner and outer surfaces and its rim, in the scoop's frame: what the ball can touch."""
    ri = BOWL_R - BOWL_T
    pts = [(r * math.sin(p) * math.cos(a), r * math.sin(p) * math.sin(a), -r * math.cos(p))
           for r in (ri, BOWL_R) for p in np.linspace(0.0, math.pi / 2, 16) for a in np.linspace(0, 2 * math.pi, 48, endpoint=False)]
    return np.array(pts)


BOWL_SKIN = bowl_skin()


# -- the ball -----------------------------------------------------------------------------------------------------------------------

def _ball():
    mesh = m.sphere(BALL_R, rings=10, segments=24)
    # a true sphere collider: a GPU hull is capped at 64 points, whose flats keep a ball from rolling out of a tipped bowl
    col = f"""
    def Sphere "ball_c" {m._COL_HEAD}
        double radius = {BALL_R:.6f}
    }}"""
    return m.free("ball", (BALL_PARK[0], BALL_PARK[1], TOP + BALL_PARK[2]), [("ball", mesh, "ball")], [col], LOOKS, mass=BALL_M, friction=(0.8, 0.7), torsional_patch=None)


# -- the cone -----------------------------------------------------------------------------------------------------------------------

def _cone_ring(z0, z1, a0, a1):
    r = lambda z, rad: rad * z / CONE_H  # noqa: E731
    pts = []
    for z in (z0, z1):
        for rad in (CONE_R, CONE_RI):
            for a in (a0, a1):
                rr = r(z, rad)
                pts.append((rr * math.cos(a), rr * math.sin(a), z))
    return m.Mesh(np.array(pts), np.array([(0, 1, 2)]))


def _cone():
    ri0 = 0.002
    prof = [(0.0, 0.0), (CONE_R, CONE_H), (CONE_RI, CONE_H), (CONE_RI * (ri0 + 0.004) / CONE_H * 0 + 0.0008, ri0 + 0.0005), (0.0, ri0)]
    # outer wall tip to rim, the rim, the inner wall back down to a point just over the tip
    body = m.lathe([(0.0, 0.0), (CONE_R * 0.5, CONE_H * 0.5), (CONE_R, CONE_H), (CONE_RI, CONE_H), (CONE_RI * 0.5, CONE_H * 0.5 + 0.002), (0.0, 0.004)], 32)
    cols = []
    nseg = 12
    zs = np.linspace(0.0, CONE_H, 4)
    for i in range(nseg):
        a0, a1 = 2 * math.pi * i / nseg, 2 * math.pi * (i + 1) / nseg
        for b in range(3):
            cols.append(m.col_hull(f"cone{i}_{b}", _cone_ring(zs[b], zs[b + 1], a0, a1)))
    return m.free("cone", (STAND_C[0], STAND_C[1], TOP + CONE_TIP_Z), [("cone", body, "cone")], cols, LOOKS, mass=CONE_M, friction=(0.8, 0.7), torsional_patch=None)


OBJECTS = [_tub(), _plate(), _stand(), _scoop(), _ball(), _cone()]


# -- geometry reads -----------------------------------------------------------------------------------------------------------------

def cone_gap(points: np.ndarray, cone) -> np.ndarray:
    """Each point's distance outside the cone's outer surface (the cone frame's z is its axis, the tip at 0)."""
    loc = cone.to_local(points)
    z = np.clip(loc[:, 2], 0.0, CONE_H)
    rad = np.linalg.norm(loc[:, :2], axis=1)
    return np.hypot(np.maximum(rad - CONE_R * z / CONE_H, 0.0), loc[:, 2] - z)


def finger_forces(env) -> tuple[float, float]:
    """The force each hand's gripper fingers press with now (N): the larger of the two fingers' applied force, left then right."""
    out = []
    base = env.unwrapped
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        j = [robot.find_joints(n)[0][0] for n in ("joint7", "joint8") if robot.find_joints(n)[0]]
        t = robot.data.applied_torque[0, j].abs().max().item() if j else 0.0
        out.append(float(t))
    return out[0], out[1]


def moment(c: dict) -> float:
    """The scoring text on one moment's facts: three tiers, each read from the state now; 0 once the cone cracked or the ball reached a cone the left hand did not hold."""
    if c["cracked"] or c["served"] is False:
        return 0.0
    if c["on_cone"] and c["held"] and c["upright"]:
        return 1.0
    if c["on_cone"] and c["held"]:
        return 0.7
    if c["ball_in_bowl"] and c["clear"]:
        return 0.3
    return 0.0


class Trial:
    def __init__(self) -> None:
        self.env = None
        self.reset()

    def reset(self, env=None) -> None:
        self.steps = 0
        self.cracked = gk.Latch("cone cracked")
        self.formed = False
        self.travel = 0.0
        self.crack_n = 0
        self.hold_n = 0
        self.score = 0.0
        self.report: dict = {}
        self.cone_z0 = 0.0
        self.drag_f = 0.0
        self.depth = 0.0
        self.why = ""
        self.entry = None
        self.lows: list = []  # the bowl centre's last 11 positions (0.5 s)
        self.best = 0.0
        self.vmax = 0.0
        self.lead = True
        self.served = None  # set on the first step the formed ball is in the cone's mouth: was the cone then held out of its stand by the left hand
        if env is not None and hasattr(env, "unwrapped"):
            self.env = env

    def _form(self, env, scoop) -> None:
        """A ball forms in the bowl: the ball body is put in the scoop's bowl at rest."""
        import torch

        asset = env.unwrapped.scene["ball"]
        origin = np.asarray(env.unwrapped.scene.env_origins[0].cpu().numpy(), float)
        rot = scoop.rotation()
        c = scoop.pos + rot @ np.array([0.0, 0.0, -(BOWL_R - BOWL_T - BALL_R - 0.0005)]) + origin
        q = scoop.quat_xyzw
        pose = torch.tensor([[c[0], c[1], c[2], q[3], q[0], q[1], q[2]]], dtype=torch.float32, device=asset.device)
        asset.write_root_pose_to_sim(pose)
        asset.write_root_velocity_to_sim(torch.zeros((1, 6), device=asset.device))

    def step(self, env) -> None:
        self.steps += 1
        dt = 0.05
        scoop, ball, cone = m.state(env, "scoop"), m.state(env, "ball"), m.state(env, "cone")
        tips = m.fingertips(env) - [0.0, 0.0, TOP]
        sp, bp, cp = scoop.pos - [0.0, 0.0, TOP], ball.pos - [0.0, 0.0, TOP], cone.pos - [0.0, 0.0, TOP]
        if self.steps <= 20:
            self.cone_z0 = float(cp[2])

        # the ice cream: a drag proportional to the bowl's depth, against the bowl's motion, while it is in the tub
        pts = scoop.to_world(BOWL_PTS) - [0.0, 0.0, TOP]
        low = pts[np.argmin(pts[:, 2])]
        inside = abs(low[0] - TUB_C[0]) <= TUB_HX - TUB_WALL and abs(low[1] - TUB_C[1]) <= TUB_HY - TUB_WALL
        depth = float(ICE_Z - low[2]) if inside else 0.0
        depth = min(max(depth, 0.0), DEPTH_CAP)
        self.depth = depth
        w = scoop.ang_vel
        v = scoop.lin_vel + np.cross(w, low + [0.0, 0.0, TOP] - scoop.pos)  # the bowl's lowest point's velocity
        vh = np.array([v[0], v[1], 0.0])
        sp_h = float(np.linalg.norm(vh))
        if depth > 0.0 and sp_h > 0.003:
            f = -DRAG_K * depth * vh / sp_h * min(1.0, sp_h / V_DRAW)
        else:
            f = np.zeros(3)
        self.drag_f = float(np.linalg.norm(f))
        # the drag acts at the handle's grip point, so the scoop in the hand feels the pull and no extra torque from the bowl's lever
        grip_w = scoop.to_world(np.array([[HANDLE_X1 - HANDLE_GRIP_X, 0.0, HANDLE_Z]]))[0]
        m.push(env, "scoop", f, at=grip_w, body=None)
        # travel: the straight-line distance the bowl has been drawn from where it went in, while its lowest point stays 6 to 15 mm deep (to and fro counts nothing);
        # read at the bowl's centre, which moves with the hand (the lowest of the sample points can jump a centimetre as the bowl rocks);
        # a step when the bowl has moved under 5 mm in the last half second moves the entry point with it, so a slow creep (in bursts or not) counts nothing
        self.lows = (self.lows + [sp[:2].copy()])[-11:]
        slow = len(self.lows) < 11 or float(np.linalg.norm(self.lows[-1] - self.lows[0])) < V_DRAW * 0.5
        nrm = scoop.rotation() @ np.array([0.0, 0.0, 1.0])  # the bowl's opening: it must lead the draw (a bowl pulled opening-last scoops nothing)
        mv = self.lows[-1] - self.lows[0]  # the bowl centre's move over the last half second (positions, not the body's velocity readings)
        lead = bool(float(np.dot(nrm[:2], mv)) >= -0.005)
        if DEPTH_MIN <= depth <= DEPTH_MAX and not self.formed and lead:
            if self.entry is None:
                self.entry = sp[:2].copy()
            elif slow:
                self.entry = self.entry + (self.lows[-1] - self.lows[-2])
            self.travel = max(self.travel, float(np.linalg.norm(sp[:2] - self.entry)))
            self.best = max(self.best, self.travel)
            if self.travel >= TRAVEL:
                self.formed = True
                self._form(env, scoop)
                print(f"[icecream] ball forms at step {self.steps}", flush=True)
        elif not self.formed:
            self.entry, self.travel = None, 0.0
        self.lead = lead

        # 1. the ball in the bowl, the bowl clear of the tub
        bowl_c = sp  # the bowl's centre is the scoop's origin
        in_bowl = bool(self.formed and float(np.linalg.norm(bp - bowl_c)) <= BALL_BOWL)
        clear = bool(float(low[2]) >= TUB_H + CLEAR or not (abs(low[0] - TUB_C[0]) <= TUB_HX + 0.02 and abs(low[1] - TUB_C[1]) <= TUB_HY + 0.02))

        # 2. the ball on the cone: resting in its mouth, off the scoop
        lb = cone.to_local(ball.pos[None, :])[0]
        in_mouth = bool(abs(lb[2] - CONE_H) <= 0.02 and float(np.hypot(lb[0], lb[1])) <= 0.012)
        gap_scoop = float(np.linalg.norm(scoop.to_world(BOWL_SKIN) - ball.pos, axis=1).min()) - BALL_R
        on_cone = bool(self.formed and in_mouth and gap_scoop >= 0.002)  # at rest: 1.0 asks for this 3 s in a row

        # 3. the cone held by the left hand: both its fingers within 8 mm of it, 6 cm out of its stand, its axis within 20 degrees of vertical
        axis = cone.rotation()[:, 2]
        upright = bool(float(axis[2]) >= UPRIGHT_COS)
        gap = cone_gap(tips + [0.0, 0.0, TOP], cone)
        # tips: left arm's 12 then right's 12; per arm the two fingers are the 6 with sx -1 and the 6 with sx +1
        held_by = []
        for a in (0, 1):
            seg = gap[12 * a:12 * a + 12]
            held_by.append(bool(seg[:6].min() <= FINGER_NEAR and seg[6:].min() <= FINGER_NEAR))
        # out of its stand: lifted 6 cm straight up, or carried clear of the stand's base ring
        lifted = bool(float(cp[2]) - self.cone_z0 >= LIFT_CONE or float(np.linalg.norm(cp[:2] - STAND_C)) >= STAND_BASE_R + CONE_R)
        held = bool(lifted and float(cp[2]) >= 0.01 and held_by[0])
        # the ball must reach a cone already in the left hand: set on a cone in its stand (or held by the scoop's hand) and lifted after, it voids the run
        if self.formed and in_mouth and self.served is None:
            self.served = held
            print(f"[icecream] ball reaches the mouth at step {self.steps}: cone {'held by the left hand' if held else 'NOT held by the left hand'}", flush=True)
            if not held:
                self.why = f"the ball reached the cone at step {self.steps} while the left hand did not hold it out of its stand"

        # the ball's fastest speed from leaving the scoop until it reaches the cone's mouth
        if self.formed and self.served is None and gap_scoop >= 0.002:
            self.vmax = max(self.vmax, float(np.linalg.norm(ball.lin_vel)))

        # the crack: a hand squeezing the cone with over 20 N a finger
        fl, fr = finger_forces(env)
        sq = max(fl if held_by[0] else 0.0, fr if held_by[1] else 0.0)
        self.crack_n = self.crack_n + 1 if sq > CRACK_N else 0
        if self.crack_n >= 2 and not self.cracked:
            self.why = f"cone cracked at step {self.steps}: a finger pressed {sq:.1f} N"
            print(f"[icecream] {self.why}", flush=True)
        self.cracked.trip(self.crack_n >= 2)

        c = {"cracked": bool(self.cracked), "served": self.served, "on_cone": on_cone, "held": held, "upright": upright, "ball_in_bowl": in_bowl, "clear": clear}
        mo = moment(c)
        self.hold_n = self.hold_n + 1 if mo == 1.0 else 0
        self.score = 1.0 if self.hold_n >= HOLD_STEPS else min(mo, 0.7)
        if self.vmax > V_DROP:
            self.score = min(self.score, 0.3)
        self.report = {"step": self.steps, "score": self.score, "moment": mo, "depth_mm": round(1000 * depth, 1), "drag_N": round(self.drag_f, 1), "travel_cm": round(100 * self.best, 1),
                       "formed": self.formed, "in_bowl": in_bowl, "clear": clear, "on_cone": on_cone, "gap_scoop_mm": round(1000 * gap_scoop, 1), "ball_in_cone_frame_mm": np.round(1000 * lb, 0).tolist(),
                       "held": held, "held_by": held_by, "in_mouth": in_mouth, "lifted": lifted, "upright": upright, "finger_N": [round(fl, 1), round(fr, 1)], "hold_n": self.hold_n, "vmax": round(self.vmax, 2), "lead": self.lead, "served": self.served, "cracked": bool(self.cracked), "why": self.why}
        if self.steps % 20 == 0:
            d = os.environ.get("ICECREAM_SCORE_DIR", "icecream_scores")
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"{os.environ.get('TASK_DEMO', 'run')}.json"), "w") as fh:
                json.dump(self.report, fh, indent=0, default=str)


TRIAL = Trial()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


def size_check() -> list[tuple[str, float, float, str]]:
    """The scene's key sizes against the published ones (cm): name, in the scene, published, source."""
    return [("scoop length", 100 * (HANDLE_X1 + BOWL_R), 18.4, "Zeroll 1020, 7.25 in"),
            ("scoop bowl across", 100 * 2 * BOWL_R, 5.1, "Zeroll 1020 (2 in)"),
            ("scoop handle across", 100 * 2 * HANDLE_R, 2.5, "estimate"),
            ("cone height", 100 * CONE_H, 11.7, "JOY #310, 4 5/8 in"),
            ("cone top diameter", 100 * 2 * CONE_R, 4.8, "JOY #310, 1 7/8 in"),
            ("tub length", 100 * 2 * TUB_HX, 18.0, "estimate for 1.42 L tubs"),
            ("tub width", 100 * 2 * TUB_HY, 13.0, "estimate"),
            ("tub height", 100 * TUB_H, 9.0, "estimate")]


# -- the demo -----------------------------------------------------------------------------------------------------------------------

def _rotm(axis, deg):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    t = math.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K


LEFT_GRIP_H = 0.085  # the left hand holds the cone this far above its tip
LEFT_PHI = -90.0  # and comes at it from the left, so the two hands do not meet under the bowl
PRE_TURN, TURN = 55.0, 110.0  # the bowl turned over about its handle: first part way, with the ball still in, then on until the ball rolls out
MOUTH_OFF = np.array([0.0, 0.015, -0.035])  # the cone's mouth under the bowl's lip: 15 mm toward the opening, 35 mm under the bowl's centre (the ball drops about 2.5 cm)


def demo_script(ctx):
    """The left hand takes the cone out of its stand from the left, tilted, fingers either side of it, and lifts it. The right hand takes the scoop by its
    handle's rear end, fingers along the handle, and tilts it so the bowl's opening faces forward and up. It sets the bowl's lowest point 1 cm under the
    ice cream's surface and draws it 10 cm along the tub, steering by the bowl's own lowest point; a ball forms in the bowl. It lifts the ball clear, levels
    the scoop and holds it up between the hands, then turns the bowl partly over about its handle, the opening toward the cone. The left hand brings the
    cone's mouth under the bowl's lip; the right hand turns the bowl on over the mouth until the ball rolls out onto the cone, and draws the scoop away.
    The left hand lifts the cone a little and holds it upright to the end.

    Check modes (TASK_DEMO): none; one_hand (the right hand scoops and turns the ball out over the cone left in its stand); hover (scoop held over the tub,
    not drawn); shallow (drawn 3 mm deep); short (drawn 4 cm); slow_draw (drawn 9 cm at 2.9 mm/s, under the drag's speed ramp); squeeze (the cone held with
    40 N); bare_left (the left hand lifts the cone and holds it, no ball); stand_then_lift (as one_hand, then the left hand takes the cone out of its stand
    and holds it up); back_draw (drawn 10 cm toward the robot, opening trailing); high_drop (the bowl 12 cm higher over the mouth when it turns over); same_hand (as one_hand, then the right hand lets the scoop go, takes the cone out of its stand and holds it up)."""
    import demokit as d

    r, l = ctx.right, ctx.left
    mode = ctx.mode or "full"
    live = ctx.obs is not None
    gl = d.tilted(60.0, LEFT_PHI)
    gr = d.tilted(60.0, 0.0)
    for _ in range(5):
        yield
    if mode == "none":
        yield from r.wait(4.0)
        return

    def scoop_state():
        return ctx.state("scoop")

    def low_point():
        pts = scoop_state().to_world(BOWL_PTS)
        return pts[np.argmin(pts[:, 2])]

    def bowl_c():
        return np.array(scoop_state().pos, float)

    def mouth():
        c = ctx.state("cone")
        return np.array(c.pos + c.rotation() @ np.array([0.0, 0.0, CONE_H]), float)

    # -- the cone first: the left hand takes it out of its stand
    cone0 = np.array([STAND_C[0], STAND_C[1], TOP + CONE_TIP_Z])
    hold = cone0 + [0.0, 0.0, LEFT_GRIP_H]
    alone = mode in ("one_hand", "stand_then_lift", "same_hand")  # the ball turned out onto the cone in its stand
    if not alone:
        a = math.radians(LEFT_PHI)
        yield from l.move(hold + [-0.04 * math.cos(a), -0.04 * math.sin(a), 0.10], grip=gl, finger=-0.045, t=3.0, label="left: over the cone, tilted, open")
        yield from l.move(hold, t=1.5, label="left: fingers either side of it")
        yield from l.servo(hold, timeout=1.0, label="left: in place")
        yield from l.close(0.042, 40.0 if mode == "squeeze" else 10.0, t=0.8, label="left: close on the cone")  # measured: the fingers meet the cone 42 mm apart
        yield from l.move(l.at() + [0.0, 0.0, 0.08], t=1.5, label="left: lift it out of the stand")
        if mode == "bare_left":
            yield from l.wait(4.0)
            return

    # -- the scoop: the right hand from behind its handle
    s0 = np.array([SCOOP_AT[0], SCOOP_AT[1], TOP + PLATE_H + HANDLE_Z + BOWL_R + 0.001])
    grip_pt = np.array([SCOOP_AT[0] - HANDLE_X1 + HANDLE_GRIP_X, SCOOP_AT[1], s0[2] + 0.002])
    yield from r.move(grip_pt + [-0.04, 0.0, 0.10], grip=gr, finger=-0.040, t=3.0, label="right: over the scoop's handle, tilted, open")
    yield from r.move(grip_pt, t=1.5, label="right: fingers either side of the handle")
    yield from r.close(2 * HANDLE_R, 40.0, t=0.8, label="right: close on the handle")
    yield from r.move(r.at() + [0.0, 0.0, 0.10], t=1.5, label="right: lift the scoop")
    yield from r.turn(35.0, axis=(0.0, 1.0, 0.0), t=1.5, label="right: tilt the bowl forward")
    # over the tub, the bowl's lowest point at the tub's near end, 3 cm over the surface
    back = mode == "back_draw"  # drawn toward the robot, the bowl's opening trailing
    start_x = TUB_C[0] + (0.05 if back else -0.05)
    depth = {"shallow": 0.003, "hover": -0.04}.get(mode, 0.010)
    goal0 = np.array([start_x, TUB_C[1], TOP + ICE_Z + 0.03])
    yield from r.move(r.at() + (goal0 - low_point()), t=2.5, label="right: bowl over the tub")
    if live:
        yield from r.servo(goal0, read=low_point, tol=0.002, timeout=2.0, label="right: bowl over the near end")
    goal1 = np.array([start_x, TUB_C[1], TOP + ICE_Z - depth])
    yield from r.move(r.at() + (goal1 - goal0), t=1.5, label="right: set the rim 1 cm in")
    if live:
        yield from r.servo(goal1, read=low_point, tol=0.0015, timeout=2.0, label="right: 1 cm deep")
    run = {"short": 0.04, "slow_draw": 0.09}.get(mode, 0.10) * (-1 if back else 1)
    t_draw = 31.0 if mode == "slow_draw" else 3.5
    p0 = goal1.copy()
    if live:
        yield from r.servo(lambda t: p0 + np.array([run * min(1.0, t / t_draw), 0.0, 0.0]) if mode != "hover" else p0, read=low_point, tol=0.003, timeout=t_draw + 1.0, gain=0.25,
                           label="right: draw it along the tub")
    else:
        yield from r.move(r.at() + [run, 0.0, 0.0], t=t_draw, label="right: draw it along the tub")
    yield from r.move(r.at() + [0.0, 0.0, 0.09], t=1.5, label="right: lift the ball out")
    yield from r.wait(0.5)
    print(f"[icecream] lifted: {TRIAL.report}", flush=True)
    if mode in ("hover", "shallow", "short", "slow_draw"):
        yield from r.wait(2.0)
        return

    # -- the ball onto the cone
    yield from r.turn(-35.0, axis=(0.0, 1.0, 0.0), t=1.5, label="right: level the scoop")
    up = np.array([0.40, -0.10, TOP + 0.24])
    yield from r.move(r.at() + (up - bowl_c()), t=2.0, label="right: the ball held up between the hands")
    ax = scoop_state().rotation() @ np.array([1.0, 0.0, 0.0])  # the handle's axis
    n = scoop_state().rotation() @ np.array([0.0, 0.0, 1.0])  # the bowl's opening
    sign = 1.0 if (_rotm(ax, 90.0) @ n)[1] > 0 else -1.0  # the turn that tips the opening toward the left hand
    yield from r.arc(bowl_c(), ax, sign * PRE_TURN, t=1.5, label="right: turn the bowl's opening toward the cone")
    if alone:
        tgt = cone0 + [0.0, 0.0, CONE_H]
        yield from r.move(r.at() + (tgt - (bowl_c() + MOUTH_OFF)), t=2.5, label="right: bowl's lip over the cone in its stand")
    else:
        tgt = bowl_c() + MOUTH_OFF
        yield from l.move(l.at() + (tgt - mouth()), t=2.5, label="left: the cone's mouth under the bowl's lip")
        if live:
            yield from l.servo(tgt, read=mouth, tol=0.003, timeout=2.0, label="left: mouth in place")
    if mode == "high_drop":
        yield from r.move(r.at() + [0.0, 0.0, 0.12], t=1.0, label="right: lift the bowl 12 cm over the mouth")
    yield from r.arc(bowl_c(), ax, sign * (TURN - PRE_TURN), t=2.0, label="right: turn the bowl over: the ball rolls out into the mouth")
    yield from r.wait(1.0)
    print(f"[icecream] ball on: {TRIAL.report}", flush=True)
    yield from r.move(r.at() + [0.0, -0.10, 0.05], t=1.5, label="right: draw the scoop away")
    if mode == "stand_then_lift":
        a = math.radians(LEFT_PHI)
        yield from l.move(hold + [-0.04 * math.cos(a), -0.04 * math.sin(a), 0.10], grip=gl, finger=-0.045, t=3.0, label="left: over the cone, tilted, open")
        yield from l.move(hold, t=1.5, label="left: fingers either side of it")
        yield from l.close(0.042, 10.0, t=0.8, label="left: close on the cone, the ball on it")
        yield from l.move(l.at() + [0.0, 0.0, 0.08], t=1.5, label="left: lift it out of the stand")
    elif mode == "same_hand":
        yield from r.open(t=0.8, label="right: let the scoop go")
        yield from r.move(hold + [-0.06, 0.0, 0.10], grip=gr, finger=-0.045, t=3.0, label="right: over the cone, tilted, open")
        yield from r.move(hold + [-0.01, 0.0, 0.0], t=1.5, label="right: fingers either side of it")
        yield from r.close(0.042, 10.0, t=0.8, label="right: close on the cone, the ball on it")
        yield from r.move(r.at() + [0.0, 0.0, 0.08], t=1.5, label="right: lift it out of the stand")
    elif not alone:
        yield from l.move(l.at() + [0.0, 0.0, 0.05], t=1.5, label="left: lift the cone, upright")
    yield from l.wait(5.0)
    print(f"[icecream] final: {TRIAL.report}", flush=True)
