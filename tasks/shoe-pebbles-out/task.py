"""Get the small stones out of a running shoe: hold it by the heel, turn it opening-down with the toe up, shake until all 8 are out, set it back down sole-first.

A men's US 9 road running shoe (modelled on the Nike Pegasus 41: 297 g, 10 mm heel-to-toe drop, 37/27 mm stack; 290 mm long, collar opening about 100 x 67 mm), one rigid free
body whose shell is a compound of convex pieces with walls at least 4.5 mm thick: sole, two side walls, a closed vamp over the toe box, a padded heel counter with a rigid pull tab (a real tab is a stiff stitched loop). Eight gravel pebbles,
4 to 8 mm, lie inside it, most in the toe box under the vamp. The right hand pinches the heel pull tab at its root (one finger in front of the tab, one behind it), lifts the shoe, turns it toe-up
past vertical so the opening faces down, shakes it along its length and sets it back down.

    python demokit.py task.py OUT --no-record --mode full,none,collar_up,heel_up,on_side,drop
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "There are some small stones in this shoe. Get them all out, then put the shoe back down on the table, sole down."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (2 s, counted in control steps)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
LAYOUT = os.environ.get("SHOE_LAYOUT", "A")

# -- the shoe, in its own frame: x from the back of the heel (0) to the toe tip, y across, z up from the sole's underside ---------------------------------------------------------
L = 0.290
T = 0.0045  # upper wall thickness
BACK = 0.012  # the padded heel counter at the back, solid from the sole to the heel tab
THROAT = 0.115  # the collar opening runs from BACK to here; the vamp and tongue close the shoe in front of it
CROWN = 0.008  # the vamp's rise from its sides to its centre line
TAB = (0.015, 0.100, 0.140)  # the heel pull tab on the back of the counter: half-width, bottom, top (12 mm thick, padded)
SHOE_M = 0.297
# key sections: (x, outer half-width, top of the upper: the collar edge behind THROAT, the vamp's centre line in front of it)
KEY = [(0.000, 0.022, 0.105), (0.012, 0.032, 0.103), (0.030, 0.037, 0.094), (0.060, 0.038, 0.090), (0.090, 0.037, 0.089), (0.115, 0.037, 0.092),
       (0.140, 0.040, 0.091), (0.170, 0.045, 0.084), (0.200, 0.050, 0.075), (0.230, 0.050, 0.066), (0.255, 0.045, 0.058), (0.272, 0.036, 0.051),
       (0.283, 0.024, 0.045), (0.290, 0.010, 0.036)]
KX = np.array([k[0] for k in KEY])
KW = np.array([k[1] for k in KEY])
KT = np.array([k[2] for k in KEY])
STATIONS = np.unique(np.r_[KX, np.arange(0.015, 0.28, 0.015)])


def half_w(x):
    return np.interp(x, KX, KW)


def top_z(x):
    return np.interp(x, KX, KT)


def insole_z(x):
    """The insole's top: 37 mm stack at the heel falling to 27 mm under the forefoot."""
    return np.interp(x, [0.0, 0.06, 0.20, L], [0.037, 0.037, 0.027, 0.027])


def _hull_mesh(pts) -> m.Mesh:
    """A convex piece's surface from its 8 corners: [4 at the back section, 4 at the front], each loop going round the section."""
    p = np.asarray(pts, float)
    f = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6)]
    for i in range(4):
        j = (i + 1) % 4
        f += [(i, j, 4 + j), (i, 4 + j, 4 + i)]
    f = np.array(f)
    c = p.mean(axis=0)
    for k, (a, b, e) in enumerate(f):
        n = np.cross(p[b] - p[a], p[e] - p[a])
        if np.dot(n, (p[a] + p[b] + p[e]) / 3 - c) < 0:
            f[k] = (a, e, b)
    return m.Mesh(p, f)


def _quad(x, y0, y1, z0, z1):
    return [(x, y0, z0), (x, y1, z0), (x, y1, z1), (x, y0, z1)]


def shoe_pieces():
    """(name, corners, look) for every convex piece of the shell."""
    out = []
    for i in range(len(STATIONS) - 1):
        a, b = STATIONS[i], STATIONS[i + 1]
        wa, wb, za, zb, ta, tb = half_w(a), half_w(b), insole_z(a), insole_z(b), top_z(a), top_z(b)
        out.append((f"out{i}", _quad(a, -wa, wa, 0.0, 0.006) + _quad(b, -wb, wb, 0.0, 0.006), "outsole"))
        out.append((f"mid{i}", _quad(a, -wa, wa, 0.006, za) + _quad(b, -wb, wb, 0.006, zb), "midsole"))
        if b <= BACK + 1e-9 or a >= 0.283 - 1e-9:  # the heel counter and the toe cap: solid
            out.append((f"end{i}", _quad(a, -wa, wa, za - 0.002, ta) + _quad(b, -wb, wb, zb - 0.002, tb), "upper"))
            continue
        closed = a >= THROAT - 1e-9
        sa, sb = (ta - CROWN, tb - CROWN) if closed else (ta, tb)
        for s, nm in ((1.0, "l"), (-1.0, "r")):
            out.append((f"wall{nm}{i}", _quad(a, s * wa, s * (wa - T), za - 0.003, sa) + _quad(b, s * wb, s * (wb - T), zb - 0.003, sb), "upper"))
            if closed:  # the vamp: two slabs rising from the sides to the centre line
                pa = [(a, 0.0, ta - T), (a, s * wa, sa - T), (a, s * wa, sa), (a, 0.0, ta)]
                pb = [(b, 0.0, tb - T), (b, s * wb, sb - T), (b, s * wb, sb), (b, 0.0, tb)]
                out.append((f"vamp{nm}{i}", pa + pb, "upper"))
    out.append(("tab", _quad(0.0, -TAB[0], TAB[0], TAB[1], TAB[2]) + _quad(BACK, -TAB[0], TAB[0], TAB[1], TAB[2]), "collar"))
    return out


def inside_shoe(p_local) -> np.ndarray:
    """Whether points (shoe frame) lie in the shoe: within its outer envelope and above the sole (the cavity's floor, the insole, is 27 to 37 mm up; a stone on the table under
    the sole's edge is not in). Over the collar opening the envelope reaches 1 cm above the edge, so a stone on the lip is still in."""
    p = np.atleast_2d(p_local)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    lid = np.where(x < THROAT, 0.010, 0.0)
    return (x >= 0.0) & (x <= L) & (np.abs(y) <= half_w(x)) & (z >= 0.010) & (z <= top_z(x) + lid)


def gap_to_shoe(p_local) -> np.ndarray:
    """A lower bound on each point's distance from the shoe's outer envelope (0 inside it)."""
    p = np.atleast_2d(p_local)
    x, y, z = p[:, 0], p[:, 1], p[:, 2]
    xc = np.clip(x, 0.0, L)
    top = np.where((xc <= BACK) & (np.abs(y) <= TAB[0]), TAB[2], top_z(xc))  # the pull tab
    g = np.stack([-x, x - L, np.abs(y) - half_w(xc), -z, z - top], axis=1)
    return np.maximum(g.max(axis=1), 0.0)


# -- the pebbles: irregular hulls 4 to 8 mm across, quartz 2.65 g/cm3 ---------------------------------------------------------------------------------------------------------------
N_PEB = 8
SIZES = [0.0080, 0.0074, 0.0068, 0.0062, 0.0056, 0.0050, 0.0045, 0.0041]  # longest axis
RHO = 2650.0
# where each starts in the shoe (x, y): 4 in the toe box, 2 at the midfoot, 2 at the heel
START = {"A": [(0.238, -0.013), (0.242, 0.015), (0.256, 0.001), (0.222, 0.002), (0.160, -0.011), (0.172, 0.013), (0.050, 0.010), (0.072, -0.010)],
         "B": [(0.232, 0.012), (0.248, -0.012), (0.258, 0.010), (0.218, -0.014), (0.150, 0.006), (0.185, -0.006), (0.040, -0.012), (0.085, 0.012)]}[LAYOUT]


def pebble_mesh(k: int) -> m.Mesh:
    rng = np.random.default_rng(100 + k)
    a = SIZES[k] / 2
    ax = np.array([a, a * rng.uniform(0.65, 0.85), a * rng.uniform(0.45, 0.65)])
    s = m.sphere(1.0, rings=6, segments=10)
    v = s.v / np.linalg.norm(s.v, axis=1, keepdims=True)
    bump = 1.0 + 0.12 * np.sin(3.1 * v[:, 0] + rng.uniform(0, 6)) * np.cos(2.3 * v[:, 1] + rng.uniform(0, 6))
    return m.Mesh(v * ax * bump[:, None], s.f)


def pebble_mass(k: int) -> float:
    mesh = pebble_mesh(k)
    v, f = mesh.v, mesh.f
    vol = abs(float(np.einsum("ij,ij->i", v[f[:, 0]], np.cross(v[f[:, 1]], v[f[:, 2]])).sum()) / 6.0)
    return RHO * vol


# -- the scene ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
GRIP_XY, SHOE_YAW = {"A": ((0.33, -0.08), 60.0), "B": ((0.32, -0.10), 50.0)}[LAYOUT]  # where the pull tab stands, and the shoe's heading (heel to toe) from +x
SHOE_X = GRIP_XY[0] - (BACK / 2) * math.cos(math.radians(SHOE_YAW))
SHOE_Y = GRIP_XY[1] - (BACK / 2) * math.sin(math.radians(SHOE_YAW))
LOOKS = {"outsole": m.Look(m.rgb(0.10, 0.10, 0.11), roughness=0.9), "midsole": m.Look(m.rgb(0.93, 0.93, 0.90), roughness=0.7),
         "upper": m.Look(m.rgb(0.16, 0.36, 0.78), roughness=0.75), "lace": m.Look(m.rgb(0.96, 0.96, 0.96), roughness=0.8),
         "collar": m.Look(m.rgb(0.12, 0.12, 0.14), roughness=0.8), "stone": m.Look(m.rgb(0.55, 0.50, 0.44), roughness=0.9)}


def _laces():
    parts = []
    for k, x in enumerate((0.128, 0.143, 0.158, 0.173, 0.188)):
        z = top_z(x) + 0.0015
        w = half_w(x) * 0.55
        parts.append((f"lace{k}", m.tube([(x, -w, z - 0.002), (x, 0.0, z), (x, w, z - 0.002)], 0.0018, sides=6), "lace"))
    for s, nm in ((1.0, "l"), (-1.0, "r")):  # the padded collar's dark lining along both edges of the opening
        xs = np.linspace(BACK, THROAT, 8)
        parts.append((f"collar{nm}", m.tube([(x, s * (half_w(x) - T / 2), top_z(x) + 0.001) for x in xs], 0.0032, sides=6), "collar"))
    return parts


def _shoe() -> m.Part:
    pieces = shoe_pieces()
    parts = [(n, _hull_mesh(p), look) for n, p, look in pieces] + _laces()
    cols = [m.col_hull(f"c_{n}", _hull_mesh(p)) for n, p, _ in pieces]
    return m.free("shoe", (SHOE_X, SHOE_Y, TOP + 0.0005), parts, cols, LOOKS, mass=SHOE_M, friction=(0.6, 0.5), torsional_patch=0.015, solver_iters=48,
                  yaw_deg=SHOE_YAW, contact_offset=0.002, ccd=True)


def shoe_to_world(p_local) -> np.ndarray:
    c, s = math.cos(math.radians(SHOE_YAW)), math.sin(math.radians(SHOE_YAW))
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return np.array([SHOE_X, SHOE_Y, TOP + 0.0005]) + R @ np.asarray(p_local, float)


def _pebble(k: int) -> m.Part:
    x, y = START[k]
    mesh = pebble_mesh(k)
    lift = float(-mesh.v[:, 2].min()) + 0.0015
    pos = shoe_to_world((x, y, float(insole_z(x)) + lift))
    return m.free(f"pebble{k}", tuple(pos), [("body", mesh, "stone")], [m.col_hull("c", mesh)], LOOKS, mass=pebble_mass(k), friction=(0.5, 0.45), torsional_patch=None,
                  solver_iters=32, yaw_deg=37.0 * k, contact_offset=0.0015, ccd=True)


OBJECTS = [_shoe()] + [_pebble(k) for k in range(N_PEB)]


def scene_ccd() -> bool:
    """Turn on the physics scene's CCD flag on the live stage, which the bodies' own CCD (authored in their USD) needs. Returns whether a scene was found."""
    import omni.usd
    from pxr import Sdf

    stage = omni.usd.get_context().get_stage()
    found = False
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsScene":
            prim.CreateAttribute("physxScene:enableCCD", Sdf.ValueTypeNames.Bool).Set(True)
            found = True
    return found

# -- the scorer -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
HOLD_STEPS = 40  # 2 s at 20 Hz
CLEAR_M = 0.03
LEVEL_DEG = 20.0
DROP_SPEED = 0.45  # m/s, with no hand closed on the shoe: what a free fall of 1 cm reaches (sqrt(2 g 0.01) = 0.44)
TABLE_X, TABLE_Y = (0.0, 0.85), (-0.60, 0.60)  # the table top's extent in the station frame (estimate, larger than the reach)


def opening(env, side: str) -> float:
    robot = env.unwrapped.scene[f"{side}_robot"]
    return abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))


def gripper_points(env) -> np.ndarray:
    """Points filling both grippers: fingers, pads, palm and housing, and the wrist flange, as they are now (station frame). (2 x 165, 3): the left's, then the right's."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        opening = abs(float(m._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        g = np.stack(np.meshgrid(np.linspace(-(opening + 0.012), opening + 0.012, 5), np.linspace(-0.025, 0.025, 3), np.linspace(-0.04, 0.16, 11), indexing="ij"), -1).reshape(-1, 3)
        out.append(pose[:3] - origin + g @ R.T)
    return np.vstack(out)


def tier(n_out: int, set_down: bool) -> float:
    if n_out >= N_PEB and set_down:
        return 1.0
    if n_out >= 6:
        return 0.75
    if n_out >= 4:
        return 0.5
    if n_out >= 2:
        return 0.25
    return 0.0


class Trial:
    def reset(self, env=None):
        self.n = 0
        self.void = False
        self.why = ""
        self.ok_steps = 0
        self.n_out = 0
        self.set_down = False
        self.peak_v = 0.0
        self.last = {}

    def step(self, env):
        self.n += 1
        if self.n == 1:
            print(f"[shoe] scene CCD on: {scene_ccd()}", flush=True)
        shoe = m.state(env, "shoe")
        R = shoe.rotation()
        peb = np.array([m.state(env, f"pebble{k}").pos for k in range(N_PEB)])
        loc = (peb - shoe.pos) @ R
        self.n_out = int((~inside_shoe(loc)).sum())
        v = float(np.linalg.norm(shoe.lin_vel))
        gaps = gap_to_shoe((gripper_points(env) - shoe.pos) @ R)
        held = any(opening(env, s) < 0.025 and gaps[k * 165:(k + 1) * 165].min() < 0.01 for k, s in enumerate(("left", "right")))  # a hand closed on the shoe
        if self.n > 10:  # after the harness's settle
            if not held:
                self.peak_v = max(self.peak_v, v)
            if v > DROP_SPEED and not held and not self.void:
                self.void, self.why = True, f"shoe moved at {v:.2f} m/s with no hand closed on it (dropped or thrown)"
        on_table = TABLE_X[0] < shoe.pos[0] < TABLE_X[1] and TABLE_Y[0] < shoe.pos[1] < TABLE_Y[1] and shoe.pos[2] > TOP - 0.05
        if not on_table and self.n > 10 and not self.void:
            self.void, self.why = True, "shoe off the table"
        level = float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))
        corners = (np.array([[0.0, -0.03, 0.0], [0.0, 0.03, 0.0], [L, -0.01, 0.0], [L, 0.01, 0.0], [0.20, -0.05, 0.0], [0.20, 0.05, 0.0]]) @ R.T + shoe.pos)[:, 2]
        resting = bool(corners.min() > TOP - 0.004 and corners.max() < TOP + 0.012 and np.linalg.norm(shoe.lin_vel) < 0.02)
        gap = float(gaps.min())
        open_ = all(opening(env, s) > 0.025 for s in ("left", "right"))
        self.set_down = bool(level <= LEVEL_DEG and resting and gap >= CLEAR_M and open_)
        full = self.n_out >= N_PEB and self.set_down
        self.ok_steps = self.ok_steps + 1 if full else 0
        self.last = {"n": self.n, "out": self.n_out, "level": round(level, 1), "resting": resting, "gap": round(gap, 3), "open": open_, "v": round(v, 3), "peak_v_unheld": round(self.peak_v, 3), "held": held,
                     "ok": self.ok_steps, "void": self.why, "tier": self.score()}
        if self.n % 20 == 0:
            loc_r = [[round(float(c) * 1000) for c in q] for q in loc]
            print("[shoe] " + json.dumps({"mode": os.environ.get("TASK_DEMO", ""), **self.last, "peb_mm": loc_r}), flush=True)

    def score(self) -> float:
        if self.void:
            return 0.0
        t = tier(self.n_out, False)
        if self.n_out >= N_PEB and self.set_down and self.ok_steps >= HOLD_STEPS:
            return 1.0
        return max(t, 0.75) if self.n_out >= N_PEB else t


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def score(objects) -> float:
    return TRIAL.score()


def success(objects) -> bool:
    return TRIAL.score() >= 1.0


# -- the scripted demo ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TILT = float(os.environ.get("SHOE_TILT", "135"))
TILT2 = float(os.environ.get("SHOE_TILT2", "162"))  # degrees the toe is raised through, past vertical
SHAKES = int(os.environ.get("SHOE_SHAKES", "8"))
AMP = float(os.environ.get("SHOE_AMP", "0.025"))
HALF = float(os.environ.get("SHOE_HALF", "0.25"))  # seconds per stroke
GRIP_Z = float(os.environ.get("SHOE_GRIPZ", "0.124"))  # TCP height on the pull tab, above the sole's underside
LIFT = float(os.environ.get("SHOE_LIFT", "0.18"))


def demo_script(ctx):
    """Modes: full. none: the arms wait. collar_up: lift and shake upright. heel_up: turn the opening down with the toe kept low, and shake. on_side: empty it, then lay it on its side.
    drop: empty it, then let go of it 10 cm above the table."""
    mode = ctx.mode
    if mode not in ("full", "none", "collar_up", "heel_up", "on_side", "drop"):
        raise ValueError(f"unknown mode {mode!r}")
    R_ = ctx.right
    if mode == "none":
        yield from R_.wait(3.0)
        return
    c, s = math.cos(math.radians(SHOE_YAW)), math.sin(math.radians(SHOE_YAW))
    along = np.array([c, s, 0.0])  # heel to toe
    axis = np.cross(along, [0.0, 0.0, 1.0])  # across the shoe: a positive turn about this raises the toe
    grip = d.level(SHOE_YAW - 90.0)  # fingers level, pointing across the shoe, closing along its length: one in front of the pull tab, one behind it
    g0 = shoe_to_world((BACK / 2, 0.0, GRIP_Z))
    yield from R_.move(g0 - 0.07 * axis + [0.0, 0.0, 0.03], grip=grip, finger=d.FINGER_OPEN, label="beside the heel")
    yield from R_.move(g0 - 0.07 * axis, t=0.8, label="level with the pull tab")
    yield from R_.move(g0, t=1.2, label="astride the pull tab")
    yield from R_.close(BACK, 30.0, label="pinch the pull tab")
    yield from R_.move(R_.at() + [0.0, 0.0, LIFT], t=1.5, label="lift")
    tilt = {"collar_up": 0.0, "heel_up": -100.0}.get(mode, TILT)
    if tilt:
        yield from R_.arc(R_.at(), axis, tilt, t=abs(tilt) / 40.0, label="turn the toe up, opening down")
        yield from R_.wait(0.5)
    stroke = along * math.cos(math.radians(tilt)) + np.array([0.0, 0.0, math.sin(math.radians(tilt))])  # the shoe's length, as it is held now
    home = R_.at().copy()
    for rnd in range(2):
        if rnd == 1 and tilt > 0:  # the last stones catch on the heel counter and the inner finger: turn on until the opening faces straight down, and shake again
            yield from R_.arc(R_.at(), axis, TILT2 - tilt, t=1.0, label="opening straight down")
            tilt = TILT2
            stroke = along * math.cos(math.radians(tilt)) + np.array([0.0, 0.0, math.sin(math.radians(tilt))])
            home = R_.at().copy()
        for k in range(SHAKES):
            yield from R_.move(home + AMP * stroke, t=HALF, label="shake")
            yield from R_.move(home - AMP * stroke, t=HALF, label="shake")
        if rnd == 1:  # a stone jammed between the inner finger and the heel counter: stroke across the shoe too
            for _ in range(3):
                yield from R_.move(home + 0.02 * axis, t=HALF)
                yield from R_.move(home - 0.02 * axis, t=HALF)
        yield from R_.move(home, t=HALF)
        yield from R_.wait(0.6)
    if tilt:
        yield from R_.arc(R_.at(), axis, -tilt, t=abs(tilt) / 40.0, label="back upright")
    if mode == "on_side":
        yield from R_.arc(R_.at(), along, 80.0, t=2.0, label="roll onto its side")
    if mode == "drop":
        yield from R_.move(g0 + [0.0, 0.0, 0.10], t=1.5)
        yield from R_.open(t=0.2)
        yield from R_.move(R_.at() + [0.0, 0.0, 0.08], t=1.0)
        yield from R_.wait(4.0)
        print("[shoe] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
        return
    if mode != "on_side":
        yield from R_.move(g0 + [0.0, 0.0, 0.006], t=2.0, label="set it down")
        yield from R_.move(g0 + [0.0, 0.0, 0.002], t=0.6, label="sole on the table")
    yield from R_.open()
    yield from R_.move(R_.at() + [0.0, 0.0, 0.09], t=1.2, label="hand up off the tab")
    yield from R_.move(R_.at() - 0.09 * axis, t=1.0, label="hand clear")
    yield from R_.wait(3.0)
    print("[shoe] END " + json.dumps({"mode": mode, **TRIAL.last}), flush=True)
