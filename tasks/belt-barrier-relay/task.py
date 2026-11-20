"""On the station table, a wall board 1.22 m wide along x = 0.45 m carries a yellow Tensabarrier-896-type belt unit (belt mouth at y = -0.44 m) on a 40 mm
timber batten, and a Tensabarrier 898 wall clip (its face toward the unit at y = +0.46 m) on a timber stand-off, 0.90 m apart, the belt 13 cm above the
table in the plane x = 0.33 m. The belt is webbing 50 mm wide and 1.5 mm thick, 40 g/m: a chain of 37 rigid links 25 mm long, hinged in turn about its width
(flatwise bending, soft) and about its thickness (edgewise bending, stiff), so it bends round what it meets, sags under its weight and cannot stretch. Its end
fitting, 17 mm long x 58 mm tall x 14 mm thick (50 g), is the chain's last link; on its front a neck and a bead 28 mm tall. Inside the unit the belt runs back
to the reel, a carriage on a slide along the belt's line pulled by a constant 8.9 N (a 2 lb constant-force spring) and slowed by the retractor's brake
(7.2 N s/m). The clip's slot runs top to bottom: a channel behind two lips 4 mm apart, closed at its foot, with a window above where the lips are cut away.
The clip is out of the right arm's reach and the unit out of the left arm's, so the belt changes hands: the right hand comes straight down on the end
and pinches its upper part by its faces, and draws the belt out; at y = -0.06 m the left comes in level from the front and closes across the belt's width
(its two edges) just behind the end, taking the pull, and the right lets go; the left carries the belt, the end leading, to the clip, lifts it 54 mm so the
bead clears the lips, squares the end by its own pose, brings the bead in over the channel and lowers it 54 mm to the clip's foot, then lets go of the belt.
No latch: the lips hold the bead against the belt's pull, and its weight keeps it on the foot.

Run it:  python demokit.py task.py OUT --mode full
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent.parent / "kit"))

import gatekit as gk  # noqa: E402
import mechkit as m  # noqa: E402  (hooks yam_arena so that the parts below spawn from USD)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Close the belt barrier: pull the belt out, lift its end over the wall clip and lower it down into the clip's slot."
SUCCESS_HOLD_S = 0.0
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
DT = 0.05
_F = os.environ.get


def _f(name, default):
    return float(_F(name, str(default)))


# -- layout (m; x forward from the arm bases, y to the robot's left, z above the world floor) -------------------------------------------------------------
XB = _f("BELT_XB", 0.33)  # the belt's plane
WALL_X = _f("BELT_WALL", 0.45)  # the wall board's front face
WALL_W, WALL_H = 1.22, 0.61  # the board (y, z): a 1220 x 610 mm (4 x 2 ft) sheet of 18 mm plywood, standing on the table
CASE_D = 0.100  # the case's depth: the belt leaves it 20 mm behind its front face
BATTEN = WALL_X - (XB - 0.02 + CASE_D)  # a timber batten this thick behind the case
ZB = TOP + _f("BELT_ZH", 0.13)  # the belt's centre line height
Y_MOUTH = _f("BELT_YM", -0.44)  # the unit's mouth
Y_RECV = _f("BELT_YR", 0.46)  # the clip's face
# The end fitting (the maker's photo: about 17 mm long and 58 mm tall; thickness our estimate)
E_L, E_H, E_T = 0.017, 0.058, 0.014
E_MASS = _f("BELT_EMASS", 0.05)
# The webbing: 50 mm (2 in) polyester belt, 1.5 mm thick, 40 g/m, as links hinged in turn flatwise and edgewise
W_H, W_T = 0.050, 0.0015
LINK = _f("BELT_LINK", 0.025)
N_LINK = int(_f("BELT_NLINK", 37))
W_MASS = 0.040 * LINK
EI_FLAT = _f("BELT_EIF", 0.004)  # N m^2, flatwise: our estimate 0.001 to 0.01 (a 50 mm strip of 1.5 mm polyester webbing)
EI_EDGE = _f("BELT_EIE", 0.25)  # N m^2, edgewise: our estimate 0.1 to 1 (the weave shears, far below solid polyester's)
K_FLAT, K_EDGE = EI_FLAT / (2 * LINK), EI_EDGE / (2 * LINK)  # N m/rad: each hinge lumps 2 links' length of bending
D_FLAT, D_EDGE = _f("BELT_DF", 0.01), _f("BELT_DE", 0.03)  # N m s/rad: the weave's internal damping, small (estimate; at 0.002 and 0.01 the seated end rattled 2 to 3 mm)
J_ARM = _f("BELT_ARM", 1e-5)  # kg m^2 joint armature, for the solver
J_INERTIA = 1e-6  # kg m^2: each link's inertia floored here for the solver (a 1 g link's own is 5e-8 to 3e-7)
SETTLE_K = 200.0  # N m/rad: while the scene settles, before the reel's pull acts, the belt's tilt at the reel and its edgewise hinges are held this stiff;
# then the tilt is let free (the belt pivots in the mouth slot) and the edgewise hinges get their own K_EDGE
# The retractor: a constant-force spring on the reel, read as a pull on the belt's stored end, and the brake as damping of the reel's travel
PULL = _f("BELT_PULL", 8.9)  # N: a 2 lb constant-force spring; our estimate for the real unit is 5 to 12 N (the maker publishes none)
BRAKE = _f("BELT_BRAKE", 7.2)  # N s/m: a released end runs back at about 1.2 m/s
REEL_ARM = 0.02  # kg: the reel's inertia as felt at the belt (a 50 g reel 30 mm round, estimate)
Y_TAIL = Y_MOUTH + 0.0005 - N_LINK * LINK  # the belt's stored end, the reel's carriage, at the start
Y_END0 = Y_MOUTH + 0.0005 + E_L / 2  # the end fitting's centre at the start: its back face 0.5 mm off the unit's mouth
# The wall receiver: a Tensabarrier 898 wall clip, 112 mm tall, 16 mm wide (y), 11 mm deep (x), its two 4 mm screw holes 90 mm apart (the maker's drawing).
# Its slot runs top to bottom: a channel 8 mm across (x) behind two lips 4 mm apart, open toward the unit (-y), closed at the bottom by the clip's foot.
# Above the channel the lips are cut away (the entry window), so the end's bead is brought in over the channel and lowered into it.
CLIP_H, CLIP_W, CLIP_D = 0.112, 0.016, 0.011
LIP_T, BORE_Y = 0.003, 0.005  # the lips' thickness along y; the channel behind them, along y
BORE_X, SLIT_X = 0.008, 0.004  # the channel across (x) and the gap between the lips
FOOT, CHAN_H, WIN_H = 0.011, 0.050, 0.036  # from the clip's bottom: the foot (the slot's floor), the channel with its lips, the entry window; the clip's head above
# The end fitting's front (+y) face: a neck through the lips' gap and a bead behind them (the end's tongue), 28 mm tall.
GAP = 0.0015  # the end's front face this far clear of the clip's face when seated
NECK_X, NECK_L = 0.0025, GAP + LIP_T
BEAD_X, BEAD_L = 0.006, 0.0035
FIN_H = _f("BELT_FINH", 0.028)
BEAD_YC = E_L / 2 + NECK_L + BEAD_L / 2  # the bead's centre in the end's frame
Z_CLIP = ZB - FIN_H / 2 - FOOT  # the clip's bottom: the seated bead's bottom rests on the foot
Z_FLOOR, Z_LIPTOP, Z_HEAD = Z_CLIP + FOOT, Z_CLIP + FOOT + CHAN_H, Z_CLIP + FOOT + CHAN_H + WIN_H
LIFT = _f("BELT_LIFT", 0.054)  # the end's centre this far above the belt's line to enter the window: the bead 4 mm over the lips, 4 mm under the head
Y_SEAT = Y_RECV + LIP_T - (E_L / 2 + NECK_L)  # the seated end's centre, the bead's back face on the lips under the belt's pull
Y_ENTER = Y_SEAT + 0.0008  # over the channel, the bead in the middle of it before lowering
Y_HAND = _f("BELT_YH", -0.06)  # the hand-over point (the end's centre)
RECV_MU = _f("BELT_RMU", 0.25)  # metal on metal, the bead on the clip's lips

LOOKS = {
    "wall": m.Look(color=m.rgb(0.62, 0.60, 0.56), roughness=0.8),
    "case": m.Look(color=m.rgb(0.95, 0.75, 0.05), roughness=0.45),
    "zinc": m.Look(color=m.rgb(0.62, 0.64, 0.67), roughness=0.35, metallic=0.85),
    "belt": m.Look(color=m.rgb(0.95, 0.78, 0.05), roughness=0.75),
    "end": m.Look(color=m.rgb(0.30, 0.31, 0.33), roughness=0.3, metallic=0.9),
    "cap": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "wood": m.Look(color=m.rgb(0.70, 0.55, 0.36), roughness=0.85),
    "dark": m.Look(color=m.rgb(0.08, 0.08, 0.09), roughness=0.6),
    "clip": m.Look(color=m.rgb(0.05, 0.05, 0.06), roughness=0.4, metallic=0.6),
}


def _wall():
    t = 0.018
    c = (WALL_X + t / 2, 0.0, TOP + WALL_H / 2)
    size = (t, WALL_W, WALL_H)
    return m.fixed("wall", (0, 0, 0), [("board", m.box(size, c), "wall")], [m.col_box("board", size, c)], LOOKS, friction=(0.6, 0.5))


SLOT_HX, SLOT_HZ = 0.004, 0.0265  # half the mouth slot: across (x) and up and down (z) round the 1.5 x 50 mm belt


def _case():
    """Yellow case 100 deep (x) x 115 long (y) x 70 tall (z), its mouth on the +y face. The belt runs through it in a slot 8 mm across, from the mouth back to
    the reel: the case's colliders are a front and a back slab either side of the slot, and two lips at the mouth above and below the belt. The end fitting,
    wider and taller than the slot, stops against the mouth."""
    size = (CASE_D, 0.115, 0.070)
    x0 = XB - 0.02  # the case's front face
    c = (x0 + CASE_D / 2, Y_MOUTH - 0.0575, ZB)
    bat = (BATTEN, 0.115, 0.090)
    bc = (WALL_X - BATTEN / 2, Y_MOUTH - 0.0575, ZB)
    mouth = (0.004, 0.003, 0.054)
    cm = (XB, Y_MOUTH - 0.0012, ZB)
    front, back = XB - SLOT_HX - x0, x0 + CASE_D - (XB + SLOT_HX)
    lip = (2 * SLOT_HX, 0.004, 0.035 - SLOT_HZ)
    cols = [m.col_box("front", (front, 0.115, 0.070), (x0 + front / 2, c[1], ZB)), m.col_box("back", (back, 0.115, 0.070), (x0 + CASE_D - back / 2, c[1], ZB)),
            m.col_box("lip_top", lip, (XB, Y_MOUTH - 0.002, ZB + (0.035 + SLOT_HZ) / 2)), m.col_box("lip_bot", lip, (XB, Y_MOUTH - 0.002, ZB - (0.035 + SLOT_HZ) / 2))]
    parts = [("body", m.box(size, c), "case"), ("slot", m.box(mouth, cm), "dark")]
    if BATTEN > 0.002:
        parts.append(("batten", m.box(bat, bc), "wood"))
        cols.append(m.col_box("batten", bat, bc))
    return m.fixed("case", (0, 0, 0), parts, cols, LOOKS, friction=(0.5, 0.4))


def _xz_prism(outline_xz, y0, y1):
    """A solid from an outline in (x, z), extruded along y from y0 to y1."""
    p = m.prism(outline_xz, y0, y1)
    return m.Mesh(p.v[:, [0, 2, 1]], p.f[:, ::-1])


def _receiver():
    """The Tensabarrier 898 wall clip, black metal, 112 tall x 16 wide (y) x 11 deep (x), on a timber stand-off block that brings its face to the belt's line.
    Its slot runs top to bottom: two lips 3 mm thick, 4 mm apart, in front of a channel 8 mm across, from the foot (11 mm) up 50 mm; the lips' top inner edges
    are chamfered 1 mm. Above them a 36 mm window, where the lips are cut away, lets the end's bead in over the channel; the head above carries the top screw.
    No latch: the bead is held behind the lips by the belt's pull and stands on the foot by its weight."""
    parts, cols = [], []
    y0, z0 = Y_RECV, Z_CLIP
    hx = CLIP_D / 2
    zf, zt = Z_FLOOR, Z_LIPTOP
    yb = y0 + LIP_T + BORE_Y  # the channel's back face; behind it the clip is solid
    solid = [("spine", (CLIP_D, y0 + CLIP_W - yb, CLIP_H), (XB, (yb + y0 + CLIP_W) / 2, z0 + CLIP_H / 2)),
             ("foot", (CLIP_D, yb - y0, FOOT), (XB, (y0 + yb) / 2, z0 + FOOT / 2)),
             ("head", (CLIP_D, yb - y0, CLIP_H - FOOT - CHAN_H - WIN_H), (XB, (y0 + yb) / 2, (Z_HEAD + z0 + CLIP_H) / 2))]
    for nm, size, c in solid:
        parts.append((nm, m.box(size, c), "clip"))
        cols.append(m.col_box(nm, size, c))
    ch = 0.001
    for s, side in ((1.0, "p"), (-1.0, "n")):  # the two lips (y0 to y0 + LIP_T) and the channel's two side walls (behind them), each with its top inner edge chamfered
        for nm, a, ya, yz in ((f"lip_{side}", SLIT_X / 2, y0, y0 + LIP_T), (f"wall_{side}", BORE_X / 2, y0 + LIP_T, yb)):
            out = [(a, zf), (hx, zf), (hx, zt), (a + ch, zt), (a, zt - ch)]
            out = [(XB + s * x, z) for x, z in out]
            if s < 0:
                out = out[::-1]
            mesh = _xz_prism(out, ya, yz)
            parts.append((nm, mesh, "clip"))
            cols.append(m.col_hull(nm, mesh))
    for k, zs in enumerate((z0 + (CLIP_H - 0.090) / 2, z0 + (CLIP_H + 0.090) / 2)):  # the two screw heads on its face, 90 mm apart
        parts.append((f"screw{k}", m.box((0.001, 0.007, 0.007), (XB - hx - 0.0005, y0 + CLIP_W / 2 + 0.002, zs)), "zinc"))
    # the stand-off: three 130 mm pieces of 2x4 (38 x 89 mm) screwed face to face, 114 mm deep, from the clip's back to the board
    xs = XB + hx
    bsize, bcen = (WALL_X - xs, 0.089, 0.130), ((WALL_X + xs) / 2, y0 + 0.0445, z0 + CLIP_H / 2)
    parts.append(("standoff", m.box(bsize, bcen), "wood"))
    cols.append(m.col_box("standoff", bsize, bcen))
    return m.fixed("receiver", (0, 0, 0), parts, cols, LOOKS, friction=(RECV_MU, RECV_MU * 0.8))


def link_axis(j: int) -> str:
    """Link j's hinge to the one before it: w0's is the belt's tilt at the reel (about x), then flatwise (about z, the belt's width) and edgewise (about x,
    its thickness) in turn; the end fitting's is flatwise."""
    return "X" if j % 2 == 0 else "Z"


def _belt():
    """The belt as one mechanism, fixed to the unit: the reel's carriage on a slide along the belt's line (y), the 37 webbing links, then the end fitting.
    All are authored at the start, the belt fully in: the links run straight back from the mouth through the case to the carriage, the end at the mouth."""
    links = [m.Link("reel", "slide", "Y", (XB, Y_TAIL, ZB), -0.01, N_LINK * LINK - 0.015,
                    parts=(("reel", m.box((0.004, 0.01, 0.01), (XB, Y_TAIL - 0.005, ZB)), "dark", True),), mass=0.02, spring=0.0, damping=BRAKE, friction=0.0,
                    effort=200.0, armature=REEL_ARM, gravity=False, com=(XB, Y_TAIL - 0.005, ZB), inertia=(1e-5, 1e-5, 1e-5))]
    inertia = (J_INERTIA,) * 3
    for j in range(N_LINK):
        y0 = Y_TAIL + j * LINK
        c = (XB, y0 + LINK / 2, ZB)
        ax = link_axis(j)
        if j == 0:
            k, d, lim = SETTLE_K, 0.05, 25.0
        elif ax == "Z":
            k, d, lim = K_FLAT, D_FLAT, 150.0
        else:
            k, d, lim = SETTLE_K, D_EDGE, 30.0
        links.append(m.Link(f"w{j}", "hinge", ax, (XB, y0, ZB), -lim, lim, parts=(("web", m.box((W_T, LINK - 0.0002, W_H), c), "belt"),),
                            colliders=(m.col_box(f"w{j}_c", (W_T, LINK - 0.0006, W_H), c),), mass=W_MASS, spring=k, damping=d, friction=0.0, effort=50.0,
                            armature=J_ARM, gravity=True, com=c, parent="reel" if j == 0 else f"w{j - 1}", inertia=inertia))
    yb = Y_MOUTH + 0.0005  # the end's back face
    ec = (XB, Y_END0, ZB)
    neck_c, bead_c = (XB, Y_END0 + E_L / 2 + NECK_L / 2, ZB), (XB, Y_END0 + BEAD_YC, ZB)
    neck_s, bead_s = (NECK_X, NECK_L + 0.0004, FIN_H), (BEAD_X, BEAD_L, FIN_H)
    caps = [(f"cap_{n}", m.box((E_T + 0.0006, E_L + 0.0006, 0.004), (XB, Y_END0, ZB + s * (E_H / 2 - 0.002))), "cap") for n, s in (("lo", -1), ("hi", 1))]
    parts = (("body", m.box((E_T, E_L, E_H), ec), "end"), ("neck", m.box(neck_s, neck_c), "end"), ("bead", m.box(bead_s, bead_c), "end"), *caps)
    cols = (m.col_box("end_c", (E_T, E_L, E_H), ec), m.col_box("neck_c", neck_s, neck_c), m.col_box("bead_c", bead_s, bead_c))
    ie = (E_MASS * (E_L ** 2 + E_H ** 2) / 12, E_MASS * (E_T ** 2 + E_H ** 2) / 12, max(E_MASS * (E_T ** 2 + E_L ** 2) / 12, J_INERTIA))
    links.append(m.Link("end", "hinge", "Z", (XB, yb, ZB), -150.0, 150.0, parts=parts, colliders=cols, mass=E_MASS, spring=K_FLAT, damping=D_FLAT, friction=0.0,
                        effort=50.0, armature=J_ARM, gravity=True, com=ec, parent=f"w{N_LINK - 1}", inertia=ie))
    return m.mechanism("belt", (0, 0, 0), links, LOOKS, friction=(0.7, 0.6), solver_iters=32)


OBJECTS = [_wall(), _case(), _receiver(), _belt()]
END_C0 = np.array([XB, Y_END0, ZB])  # the end's centre as authored (its link frame is the part's frame, moved: README 14)
LINK_C0 = np.array([[XB, Y_TAIL + (j + 0.5) * LINK, ZB] for j in range(N_LINK)])


# -- reading the belt -----------------------------------------------------------------------------------------------------------------------------------------
def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


class Belt:
    """The belt's bodies and joints now: the end fitting's pose (a gk.Pose about its centre), the links' centres, the belt out (the reel's travel, m) and the
    hinge angles (rad)."""

    def __init__(self, env):
        base = env.unwrapped
        a = base.scene["belt"]
        origin = _np(base.scene.env_origins[0])
        names = list(a.body_names)
        pose = _np(a.data.body_link_pose_w)[0]
        vel = _np(a.data.body_link_vel_w)[0]
        q = _np(a.data.joint_pos)[0]
        self.joints = {n: float(q[i]) for i, n in enumerate(a.joint_names)}
        i = names.index("end")
        R = m.quat_matrix(pose[i, 3:7])
        p = pose[i, :3] - origin
        c = R @ END_C0 + p
        w = vel[i, 3:6]
        self.end = gk.Pose(c, R, vel[i, :3] + np.cross(w, R @ END_C0), w)
        self.link_R = [m.quat_matrix(pose[names.index(f"w{j}"), 3:7]) for j in range(N_LINK)]
        self.links = np.array([self.link_R[j] @ LINK_C0[j] + pose[names.index(f"w{j}"), :3] - origin for j in range(N_LINK)])
        self.out = self.joints["reel_joint"]

    def link(self, j: int):
        """Link j as a gk.Pose about its centre."""
        return gk.Pose(self.links[j], self.link_R[j])

    def bends(self) -> tuple[float, float]:
        """The largest flatwise and edgewise hinge angles (deg) along the belt that is out of the unit."""
        flat, edge = 0.0, 0.0
        for j in range(1, N_LINK + 1):
            name = "end_joint" if j == N_LINK else f"w{j}_joint"
            if j < N_LINK and self.links[j][1] < Y_MOUTH:
                continue
            a = abs(math.degrees(self.joints[name]))
            if link_axis(j) == "Z":
                flat = max(flat, a)
            else:
                edge = max(edge, a)
        return flat, edge


def _dry_end(ctx):
    """In pathcheck's dry run (no simulator): the end's pose from pathcheck's own scene, where a closed hand carries it."""
    for cell in type(ctx).state.__closure__ or ():
        sc = cell.cell_contents
        if type(sc).__name__ == "Scene":
            b = sc.body("belt:end")
            if not b.holders:  # the belt held behind the end: the end rides with that link (the dry run moves held bodies only)
                b = next((x for x in sc.bodies if x.part == "belt" and x.holders), b)
            return gk.Pose(b.R @ END_C0 + b.t, b.R)
    raise RuntimeError("no dry scene")


def end_pose(ctx):
    env = ctx.emb._env
    if getattr(env, "unwrapped", None) is None:
        return _dry_end(ctx)
    return Belt(env).end


def pitch_deg(pose) -> float:
    """The end's front up (+) or down (-): its length axis's rise, about the x axis."""
    R = pose.rotation()
    return math.degrees(math.atan2(R[2, 1], R[1, 1]))


def roll_deg(pose) -> float:
    """The end's top leaning toward the wall (+) or away (-): its height axis's lean, about the y axis."""
    R = pose.rotation()
    return math.degrees(math.atan2(R[0, 2], R[2, 2]))


def yaw_deg(pose) -> float:
    """The end's turn about the vertical: its length axis's heading from +y."""
    R = pose.rotation()
    return math.degrees(math.atan2(-R[0, 1], R[1, 1]))


# -- scoring: the end state only ------------------------------------------------------------------------------------------------------------------------------
OUT_MIN = 0.50  # belt out at the end
SEAT_TOL = 0.003  # seated: the bead's bottom within this of the clip's foot (lowered at least 47 of the channel's 50 mm)
UNDER_TOL = 0.002  # the bead's bottom may sit this far under the foot's top face (contact penetration) and still count
ENGAGE = 0.005  # in the slot: the bead's bottom at least this far below the lips' top, the bead behind the lips
CLEAR = 0.05
STILL = 0.001  # m: the end's centre moves less than this over the last 1 s
FOLD = 30.0  # deg: a flatwise hinge of the belt out bent more than this is a fold or a snag (a straight run under the pull bends under 5)
HOLD_STEPS = 20  # 1 s at 20 Hz
# the end held by its faces (jaws across x)
GRIPS = (gk.Grip(half=E_T / 2, across="x", lo=(-1.0, -E_L / 2 - 0.004, -E_H / 2 - 0.004), hi=(1.0, E_L / 2 + 0.004, E_H / 2 + 0.004)),
         gk.Grip(half=E_H / 2, across="z", lo=(-0.03, -E_L / 2 - 0.004, -1.0), hi=(0.03, E_L / 2 + 0.004, 1.0)))
# a link of belt held across its width (its two edges) or flat
# (a flat pinch: jaws closed on the 1.5 mm webbing; open jaws hovering round it are outside the window)
BELT_GRIPS = (gk.Grip(half=W_H / 2, across="z", lo=(-0.03, -LINK / 2 - 0.002, -1.0), hi=(0.03, LINK / 2 + 0.002, 1.0)),
              gk.Grip(half=W_T / 2, across="x", lo=(-1.0, -LINK / 2 - 0.002, -W_H / 2 - 0.004), hi=(1.0, LINK / 2 + 0.002, W_H / 2 + 0.004)))
NEAR_LINKS = 8  # the last 0.2 m of belt behind the end


def holders(tips, h, belt=None, whole: bool = False) -> tuple[str, ...]:
    """The hands that hold the end, or the belt within 0.2 m of it; with `whole`, or the belt anywhere along its length."""
    hands = {hand for g in GRIPS for hand in gk.holding(tips, h, g, resting=False)}
    if belt is not None:
        for j in range(0 if whole else N_LINK - NEAR_LINKS, N_LINK):
            hands |= {hand for g in BELT_GRIPS for hand in gk.holding(tips, belt.link(j), g, resting=False)}
    return tuple(sorted(hands))


def palm_points(env) -> np.ndarray:
    """Points on both gripper bodies behind the fingers (station frame): a grid 8 cm across, 6 cm wide, along the gripper's axis from 0.02 to 0.09 m. Shape (36, 3): the left's 18, then the right's."""
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        R = m.quat_matrix(pose[3:7])
        for sx in (-0.04, 0.0, 0.04):
            for dy in (-0.03, 0.03):
                for dz in (0.02, 0.055, 0.09):
                    out.append(pose[:3] - origin + R @ np.array([sx, dy, dz]))
    return np.array(out)


def bead_ends(pose) -> tuple[np.ndarray, np.ndarray]:
    """The bead's bottom and top centres in the world."""
    w = pose.to_world(np.array([[0.0, BEAD_YC, -FIN_H / 2], [0.0, BEAD_YC, FIN_H / 2]]))
    return np.asarray(w[0], float), np.asarray(w[1], float)


def behind_lips(p) -> bool:
    """A point of the bead inside the clip's channel, in plan: across (x) within the channel, along y behind the lips."""
    return abs(float(p[0]) - XB) < BORE_X / 2 and Y_RECV + LIP_T - 0.001 < float(p[1]) < Y_RECV + LIP_T + BORE_Y + 0.0005


def slot_state(pose) -> tuple[bool, bool, float]:
    """(in the slot, seated, the bead's bottom above the foot in m). In the slot: both ends of the bead behind the lips, its bottom at least 5 mm below the lips'
    top, and the whole bead inside the channel's depth: its bottom no more than 2 mm under the foot, its top under the clip's head. Seated: in the slot, its
    bottom within 3 mm of the foot."""
    lo, hi = bead_ends(pose)
    above = float(lo[2]) - Z_FLOOR
    in_slot = (behind_lips(lo) and behind_lips(hi) and float(lo[2]) < Z_LIPTOP - ENGAGE and float(lo[2]) >= Z_FLOOR - UNDER_TOL and float(hi[2]) <= Z_HEAD)
    return bool(in_slot), bool(in_slot and above <= SEAT_TOL), above


def grade(seated: bool, clean_steps: int, hands: tuple, out: float, folded: bool = False, still: bool = True) -> float:
    """The end-state tiers, one per key substep; any run-back of the belt on the way does not count, and a hand is read on the end or on the belt near it, as it is now.
    0.2: drawn out: the belt out 0.5 m with a hand shut on the end or on the belt within 0.2 m of it.
    0.4: handed over: that, and the left hand is the one shut on it.
    0.6: seated in the clip: the bead down the channel behind the lips, to within 3 mm of the foot.
    1.0: seated and left to the clip: for the last 1 s the end seated and still, no fingertip, gripper body or palm within 5 cm of it or of the last 0.2 m of belt, and no hand shut on the belt anywhere.
    0: otherwise, and whenever the belt is folded or snagged (the one failure that voids the run)."""
    if folded:
        return 0.0
    if seated:
        return 1.0 if clean_steps >= HOLD_STEPS and still and out >= OUT_MIN else 0.6
    if hands and out >= OUT_MIN:
        return 0.4 if "left" in hands else 0.2
    return 0.0


def _after_settle(env) -> None:
    """Once the scene has settled: the reel's pull on, the belt's tilt at the reel let free (it pivots in the mouth slot), the edgewise hinges at their own
    stiffness. The next reset gives them their settling springs back (mechkit restores touched joints)."""
    import torch

    a = env.unwrapped.scene["belt"]
    for j in range(N_LINK):
        if link_axis(j) == "X":
            i = a.find_joints(f"w{j}_joint")[0][0]
            a.write_joint_stiffness_to_sim(torch.tensor([[0.0 if j == 0 else K_EDGE]], device=a.device), joint_ids=[i])
            m._TOUCHED.add(("belt", f"w{j}"))
    m.push(env, "belt", (0.0, -PULL, 0.0), body="reel")


class Trial:
    def reset(self, env=None) -> None:
        self.steps = 0
        self.max_out = 0.0
        self.end = gk.EndHold(HOLD_STEPS)
        self.track = []
        self.score, self.full = 0.0, False
        self.r = {}
        if env is not None:
            _after_settle(env)
            b = Belt(env)
            for j in range(N_LINK):  # the stored belt is inside the case: nothing is drawn outside it
                m.set_visible(f"{m.ENV_PATH}/belt/w{j}/visual", bool(b.links[j][1] > Y_MOUTH - 0.115 + LINK / 2))

    def step(self, env) -> None:
        self.steps += 1
        m.push(env, "belt", (0.0, -PULL, 0.0), body="reel")  # the constant-force spring, on the belt's stored end
        b = Belt(env)
        h, out = b.end, b.out
        for j in range(N_LINK):  # links run back through the case to the reel: hide those that have gone out of its back
            m.set_visible(f"{m.ENV_PATH}/belt/w{j}/visual", bool(b.links[j][1] > Y_MOUTH - 0.115 + LINK / 2))
        self.max_out = max(self.max_out, out)
        tips = m.fingertips(env)
        hands = holders(tips, h, b)
        anywhere = holders(tips, h, b, whole=True)  # a hand on the end or on any link of the belt
        in_slot, seated, above = slot_state(h)
        flat, edge = b.bends()
        folded = flat > FOLD
        near = b.links[N_LINK - NEAR_LINKS:]  # the last 0.2 m of belt: its middle line and its two edges (so a finger 3 cm over an edge is not clear)
        clear = bool(gk.hands_clear(tips, [near, near + (0.0, 0.0, W_H / 2), near - (0.0, 0.0, W_H / 2), gk.along(h, (0.0, -E_L / 2, 0.0), (0.0, BEAD_YC, 0.0), 6),
                                           gk.along(h, (0.0, 0.0, -E_H / 2), (0.0, 0.0, E_H / 2), 6)], CLEAR))  # and the end's centre lines along its length and height
        self.track = (self.track + [np.asarray(h.pos, float).copy()])[-(HOLD_STEPS + 1):]
        drift = float(np.linalg.norm(np.ptp(np.array(self.track), axis=0))) if len(self.track) > HOLD_STEPS else 1.0
        still = drift < STILL
        pp = palm_points(env)
        palm = bool(min(float(gk._to_polyline(pp, L, True).min()) for L in (near, gk.along(h, (0.0, -E_L / 2, 0.0), (0.0, BEAD_YC, 0.0), 6), gk.along(h, (0.0, 0.0, -E_H / 2), (0.0, 0.0, E_H / 2), 6))) < CLEAR)  # a gripper body or palm near the end
        self.end.step(bool(seated and clear and not anywhere and not palm))  # the last 1 s: seated, no fingertip or gripper body near, off the belt; the end's own drift is read over the same 1 s
        sc = grade(seated, self.end.count, hands, out, folded, still)
        self.score, self.full = sc, sc >= 1.0
        sag = float(ZB - np.min(b.links[b.links[:, 1] > Y_MOUTH + 0.01][:, 2])) if np.any(b.links[:, 1] > Y_MOUTH + 0.01) else 0.0
        self.r = dict(dz_mm=float(h.pos[2] - ZB) * 1000, dx_mm=float(h.pos[0] - XB) * 1000, yaw=yaw_deg(h), tilt=h.tilt_deg(), gapL=gk.jaw_gap(tips, "left") * 1000,
                      gapR=gk.jaw_gap(tips, "right") * 1000, out_mm=out * 1000, max_out_mm=self.max_out * 1000, bead_above_mm=above * 1000, hands=",".join(hands),
                      in_slot=in_slot, seated=seated, clear=clear, flat=flat, edge=edge, sag_mm=sag * 1000, v_mm_s=float(np.linalg.norm(h.lin_vel)) * 1000,
                      pitch_mm=float(np.max(np.linalg.norm(np.diff(b.links, axis=0), axis=1))) * 1000, drift_mm=drift * 1000)
        if not np.all(np.isfinite(b.links)):
            print("[belt] NON-FINITE link positions", flush=True)
        log = os.environ.get("BELT_LOG")
        if log and self.steps % int(log) == 0:
            print(f"[belt] {self.steps}: " + " ".join(f"{k} {round(v, 2) if isinstance(v, float) else v}" for k, v in self.r.items()) + f" score {sc}", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.score >= 1.0


def score(objects) -> float:
    return TRIAL.score


def size_check() -> list[str]:
    """The scene's key sizes against the stated ones (README, size table)."""
    rows = [("belt width", W_H * 1000, 50.8, "2 in belt, Tensator 896 page"), ("belt thickness", W_T * 1000, 1.5, "estimate (polyester webbing 1 to 2 mm)"),
            ("belt mass per m", W_MASS / LINK * 1000, 40.0, "estimate (webbing 30 to 50 g/m)"), ("belt link length", LINK * 1000, 25.0, "model"),
            ("belt length", N_LINK * LINK * 1000, 925.0, "model: 0.89 m out at the clip; the real unit holds 13 ft"),
            ("end fitting length", E_L * 1000, 17.0, "maker's photo, scaled to the 50 mm belt"), ("end fitting height", E_H * 1000, 58.0, "maker's photo, scaled"),
            ("end fitting thickness", E_T * 1000, 14.0, "estimate"), ("end fitting mass", E_MASS * 1000, 50.0, "estimate (30 to 90 g)"),
            ("wall clip height", CLIP_H * 1000, 112.0, "Tensabarrier 898 page"), ("wall clip width", CLIP_W * 1000, 16.0, "898 page"), ("wall clip depth", CLIP_D * 1000, 11.0, "898 page"),
            ("clip channel, foot to lips' top", CHAN_H * 1000, 50.0, "estimate (898 drawing, scaled)"), ("end fitting bead height", FIN_H * 1000, 28.0, "estimate"),
            ("case depth", CASE_D * 1000, 100.0, "estimate"), ("batten behind the unit", BATTEN * 1000, 40.0, "layout"), ("stand-off behind the clip", (WALL_X - XB - CLIP_D / 2) * 1000, 114.0, "three 2x4s, 38 mm each"),
            ("unit mouth to receiver mouth", (Y_RECV - Y_MOUTH) * 1000, 900.0, "layout"), ("wall board width", WALL_W * 1000, 1220.0, "4 x 2 ft plywood sheet"),
            ("belt pull", PULL, 8.9, "2 lb constant-force spring; real unit estimated 5 to 12 N")]
    return [f"{n}: scene {s:.1f}, stated {p:.1f} ({src})" for n, s, p, src in rows]


# -- the demo -----------------------------------------------------------------------------------------------------------------------------------------------
def _frame(a, c):
    import demokit as d

    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (c @ a)
    c /= np.linalg.norm(c)
    return d._as_grip(np.stack([c, np.cross(a, c), a], axis=1))


ANG_R = _f("BELT_AR", 90)  # right: fingers pointing -y tilted this far down (90: straight down); jaws across x on the end's faces
ANG_L = _f("BELT_AL", -15)  # left: level from the front, turned this far about z (toward the unit), jaws across z on the belt's two edges (README, pathcheck)
# where each hand's TCP sits (the end's centre frame): the right's pads on the end's upper part, fingertips 15 mm over its centre (clear of the bead's top at
# 14 mm), their back edge 2.5 mm in front of its back face; the left's across the belt's width just behind the end, the pads' front edge 3 mm behind its back face
R_DY, R_DZ = _f("BELT_RDY", 0.012), _f("BELT_RDZ", 0.028)
L_DY, L_DZ = _f("BELT_LDY", -0.030), _f("BELT_LDZ", 0.0)


def _grip_right():
    a = math.radians(ANG_R)
    return _frame((0.0, -math.cos(a), -math.sin(a)), (1.0, 0.0, 0.0))


def _grip_left():
    b = math.radians(ANG_L)
    return _frame((math.cos(b), math.sin(b), 0.0), (0.0, 0.0, 1.0))


R_OPEN, L_OPEN = -0.016, -0.035  # finger joints on approach: jaws 32 mm round the 14 mm faces, 70 mm round the 50 mm belt
L_FORCE = _f("BELT_LFORCE", 20)  # N a finger on the belt's edges: friction 2 x 20 x 0.6 = 24 N against the 8.9 N pull
FORCE = _f("BELT_FORCE", 40)  # N a finger (Robocurve's figure for the YAM)
MU_PAD = 0.6  # the end's dynamic friction, the lower of its pair (0.7 static)

SNAP_SETUP = True  # the demo takes the setup picture itself: arms clear of the case and the clip, the belt fully in


PARK_R = np.array([0.24, -0.20, ZB + 0.10])  # the right's rest (the pose it already reaches when it lets go): beside the case, off its mouth
PARK_L = np.array([XB - _f("BELT_PX", 0.12), _f("BELT_PY", 0.12), ZB + _f("BELT_PZ", 0.12)])



def demo_script(ctx):
    """A scripted reference, not a policy: it reads the end fitting's pose from the simulator to steer.
    TASK_DEMO: full; carry (the left takes the end and carries it 0.15 m on and holds it; must score 0.4); rightfar (the right alone draws the end to y = 0.31 and holds it; at most 0.2); rightback (full, then the right pinches the belt flat 0.65 m behind the seated end and holds it; must score 0.6); palm (full, then the left's body and palm rest 2 cm from the seated end, fingertips 15 cm away; must score 0.6); latestill (the full motion, run cut so that the end has been still for about 1.1 s at the run's end: --max-steps; must score 1.0); farback (full, but after the end is seated the left shuts on the belt 0.30 m behind it and holds the pull; must score 0.6); rightdiag (the right hand alone draws the end diagonally to (0.10, 0.05) at the belt's height and holds it; at most 0.2); under (the end carried to the clip's level but 40 mm under it, bead behind the lips' plane, and held there; at most 0.4); flatpinch (the left takes the
    end as in full, then the right pinches the belt flat 0.10 m behind it, the left lets go, and the right draws the belt 0.5 m out and holds it; the end is never
    in the clip); farhold (full, but after the end is seated the left shuts on the belt 0.12 m behind it and holds the pull); right (right hand only: draws out and holds); letgo (the left lets go at the receiver); face (the end carried at the belt's height
    straight into the clip's face, not lifted, then let go); hover (lifted into the window over the channel and held there, not lowered); short (lowered
    30 mm down the channel, to 20 mm over the foot, and held there); none; still (waits 1 s)."""
    import demokit as d
    from yam_arena import ik as _ik

    l, r = ctx.left, ctx.right
    mode = ctx.mode
    if mode in ("still", "none"):
        yield from l.wait({"still": 1.0, "none": 3.0}[mode])
        return
    gr, gl = _grip_right(), _grip_left()
    live = getattr(ctx.emb._env, "unwrapped", None) is not None

    dry_hold = []  # the dry run (pathcheck, no simulator): the hand that carries the end, and the end's pose in that hand's TCP frame

    def pose():
        if live or not dry_hold:
            return end_pose(ctx)
        if not isinstance(dry_hold[-1], tuple):  # let go: it stays where it was
            return dry_hold[-1]
        arm, Rr, tr = dry_hold[-1]
        p, R = tcp_frame(arm)
        return gk.Pose(p + R @ tr, R @ Rr)

    def dry_take(arm):
        if not live:
            p0 = pose()
            p, R = tcp_frame(arm)
            dry_hold.append((arm, R.T @ p0.rotation(), R.T @ (p0.pos - p)))

    def hpos():
        return np.asarray(pose().pos, float).copy()

    def out_now():
        return Belt(ctx.emb._env).out if live else float(hpos()[1] - Y_END0)

    def report(label):
        p = pose()
        extra = ""
        if live:
            b = Belt(ctx.emb._env)
            flat, edge = b.bends()
            extra = f" belt: flat {flat:.1f} edge {edge:.1f} deg, lowest link {1000 * (ZB - b.links[:, 2].min()):.1f} mm under the line"
        print(f"[demo] {label}: end {np.round(p.pos, 4)} out {out_now() * 1000:.0f} mm yaw {yaw_deg(p):.1f} roll {roll_deg(p):.1f} pitch {pitch_deg(p):.1f} "
              f"bead {slot_state(p)[2] * 1000:.1f} mm over the foot  R tcp {np.round(r.tcp(), 4)}  L tcp {np.round(l.tcp(), 4)}{extra}", flush=True)

    def ramp(arm, pos, grip, n=10, t=3.0, finger=None):
        """Move to `pos` in `n` short seeded steps, the grip turned on the way."""
        p0, R0 = _ik.tcp_pose(arm.q, arm.base)
        p0 = p0 - arm.trim
        for k in range(1, n + 1):
            sm = _ik.smoothstep(k / n)
            yield from arm.move(p0 + (np.asarray(pos, float) - p0) * sm, grip=d._as_grip(_ik.slerp_matrix(R0, grip.rot, sm)), finger=finger, t=t / n, label=f"ramp {k}")

    def slide(arm, dy, speed, label):
        """Carry the hand along y at `speed`, closing out lag as it goes (the goal moves, then holds)."""
        p0 = arm.at().copy()
        t = abs(dy) / speed

        def goal(s):
            return p0 + np.array([0.0, math.copysign(speed * min(s, t), dy), 0.0])

        yield from arm.servo(goal, tol=0.0015, timeout=t + 2.0, gain=0.15, label=label)

    GAIN, SPEED = _f("BELT_GAIN", 0.3), _f("BELT_SPEED", 0.06)

    def steer(arm, y_to, speed, label, tol=0.0015, extra=2.0, dz=0.0, dx=0.0):
        """Bring the end's centre along the belt's line (x = XB, z = ZB) to y_to, steering the hand by the end's own pose at every step."""
        y0 = float(hpos()[1])
        t = abs(y_to - y0) / speed

        def goal(s):
            return np.array([XB + dx, y0 + math.copysign(min(s * speed, abs(y_to - y0)), y_to - y0), ZB + dz])

        yield from arm.servo(goal, tol=tol, timeout=t + extra, gain=GAIN, read=hpos, label=label)

    def tcp_frame(arm):
        q = np.asarray(ctx.observed(arm.side)[:6], float)
        return _ik.tcp_pose(q, arm.base)

    def in_hand(arm):
        """The end's centre in the hand's own frame (mm): its drift there is slip in the grip."""
        p, R = tcp_frame(arm)
        return (R.T @ (hpos() - p)) * 1000

    def grip_check(arm, label, squeeze=FORCE):
        """The belt's pull against what the pinch can hold: friction 2 x squeeze x mu, and the pull's moment about the pinch (the belt meets the end at its
        back face's centre) against the pads' torsional hold."""
        p, R = tcp_frame(arm)
        pe = pose()
        anchor = pe.to_world(np.array([0.0, -E_L / 2, 0.0]))[0]
        force = PULL * np.array([0.0, -1.0, 0.0])
        moment = float(np.linalg.norm(np.cross(anchor - p, force)))
        print(f"[demo] {label}: belt pull about {PULL:.1f} N, pinch friction 2 x {squeeze:.0f} N x {MU_PAD} = {2 * squeeze * MU_PAD:.0f} N; "
              f"pull's moment about the pinch {moment:.3f} N m (limit about 1.2 N m)", flush=True)

    def vert(arm, z_to, speed, y, label, tol=0.0008, extra=1.5):
        """Raise or lower the end's centre to z_to at `speed`, kept at y and on the belt's plane, steering by the end's own pose."""
        z0 = float(hpos()[2])
        t = abs(z_to - z0) / speed

        def goal(s):
            return np.array([XB, y, z0 + math.copysign(min(s * speed, abs(z_to - z0)), z_to - z0)])

        yield from arm.servo(goal, tol=tol, timeout=t + extra, gain=GAIN, read=hpos, label=label)

    def bead_off():
        """The bead's middle off the channel's line across (x, mm), and its height over the entry height (mm)."""
        lo, hi = bead_ends(pose())
        c = (lo + hi) / 2
        return (float(c[0]) - XB) * 1000, (float(c[2]) - (ZB + LIFT)) * 1000

    r_at = np.array([0.0, R_DY, R_DZ])
    l_at = np.array([0.0, L_DY, L_DZ])
    report("start")
    # setup picture: both hands drawn in to mid-table, off the case and the clip, the belt fully in
    yield from ctx.together(ramp(r, PARK_R, gr, finger=R_OPEN, t=2.0), ramp(l, PARK_L, gl, finger=L_OPEN, t=2.0))
    yield from r.wait(0.5)
    ctx.snaps.append("setup")
    yield from r.wait(0.2)
    # the right comes straight down on the docked end; the left waits in front of the hand-over point
    back_r = -gr.rot[:, 2]
    r_front = hpos() + r_at + 0.08 * back_r
    back_l = -gl.rot[:, 2]
    l_high = np.array([XB, Y_HAND, ZB]) + l_at + 0.10 * back_l
    yield from ctx.together(ramp(r, r_front, gr, finger=R_OPEN), ramp(l, l_high, gl, finger=L_OPEN, t=3.5))
    yield from r.servo(r_front, tol=0.003, timeout=2.0, label="R front")
    report("right in front")
    yield from r.move(hpos() + r_at, t=1.2, label="R in")
    yield from r.servo(hpos() + r_at, tol=0.002, timeout=2.0, label="R on")
    yield from r.close(E_T, FORCE, label="R grip")
    dry_take(r)
    yield from r.wait(0.3)
    report("right has the end")
    grip_check(r, "right's pinch")
    rel0 = in_hand(r)
    # draw the belt out to the hand-over point
    if mode == "rightdiag":  # cheat check: the right hand alone draws the end diagonally across, past the case's corner, so the belt is out 0.5 m or more, and holds it there
        yield from r.move(np.array([0.10, 0.05, ZB]), t=3.0, label="R diagonal")
        yield from r.wait(4.0)
        report("right diagonal, end")
        return
    if mode == "rightfar":  # cheat check: the right hand alone draws the end as far as it reaches along the wall (to y = 0.31) and holds it
        yield from slide(r, 0.31 - float(hpos()[1]), 0.08, "R draw to its reach")
        yield from r.wait(3.0)
        report("right far, end")
        return
    if mode == "right":  # cheat check: the right hand alone draws the belt as far as it reaches and holds it there
        yield from slide(r, 0.0 - float(hpos()[1]), 0.08, "R draw far")
        yield from r.wait(3.0)
        report("right only, end")
        return
    yield from slide(r, Y_HAND - float(hpos()[1]), 0.08, "R draw")
    slip = in_hand(r) - rel0
    print(f"[demo] slip in the right's grip over the draw: {np.round(slip, 1)} mm (|{np.linalg.norm(slip):.1f}| mm)", flush=True)
    report("drawn to the hand-over")
    # the left comes in level from the front and closes across the belt's width just behind the end, taking the pull; then the right lets go and lifts out
    yield from l.servo(hpos() + l_at + 0.10 * back_l, tol=0.004, timeout=1.5, label="L front")
    yield from l.move(hpos() + l_at + 0.03 * back_l, t=1.2, label="L in")
    yield from l.servo(hpos() + l_at, tol=0.002, timeout=2.5, label="L on")
    yield from l.close(W_H, L_FORCE, label="L grip")
    yield from l.wait(0.4)
    report("both hold")
    shut = float(ctx.observed("left")[6])  # the left's own gripper reading, 0 shut to 1 open: shut on nothing means it missed the end
    print(f"[demo] left gripper reads {shut:.3f} ({W_H * 1000:.0f} mm across the jaws is about {W_H / 0.0939:.3f})", flush=True)
    if live and shut < 0.06:
        print("[demo] the left missed the end: the right keeps it", flush=True)
        yield from r.wait(2.0)
        return
    dry_take(l)
    y_hold = float(hpos()[1])
    yield from ctx.together(r.open(t=0.5), steer(l, y_hold, SPEED, "L takes the pull", tol=0.002, extra=1.5))  # the left holds the end where it is as the right lets go
    yield from r.move(r.at() + 0.08 * back_r, t=1.2, label="R back out")
    yield from r.move(np.array([0.24, -0.20, ZB + 0.10]), t=1.5, label="R away")
    report("left has it")
    grip_check(l, "left's pinch", L_FORCE)
    if mode == "flatpinch":  # cheat check: the right pinches the belt flat 0.10 m behind the end, the left lets go, the right draws the belt out and holds
        b_at = np.array([XB, float(hpos()[1]) - 0.10, ZB + 0.01])
        yield from ramp(r, b_at + 0.08 * back_r, gr, finger=R_OPEN)
        yield from r.servo(b_at + 0.08 * back_r, tol=0.003, timeout=2.0, label="R over the belt")
        yield from r.move(b_at, t=1.2, label="R down on the belt")
        yield from r.servo(b_at, tol=0.002, timeout=2.0, label="R on the belt")
        yield from r.close(W_T, FORCE, label="R pinch the belt flat")
        yield from r.wait(0.3)
        report("right pinches the belt")
        yield from l.open(t=0.5)
        yield from l.move(l.at() + 0.06 * back_l, t=1.2, label="L out")
        yield from slide(r, _f("BELT_FPD", 0.20), 0.05, "R draw the belt")
        yield from r.wait(4.0)
        report("flat pinch, end")
        return
    if mode == "carry":  # cheat check: the left takes the end and carries it 0.15 m on and holds it, never to the clip
        yield from steer(l, float(hpos()[1]) + 0.15, SPEED, "L carry 0.15 m", tol=0.002)
        yield from l.wait(4.0)
        report("carry, end")
        return
    rel0 = in_hand(l)
    # carry it toward the receiver at the belt's height, to 3 cm short of the clip
    y_pre = Y_SEAT - 0.03
    yield from steer(l, y_pre, SPEED, "L carry")
    slip = in_hand(l) - rel0
    print(f"[demo] slip in the left's grip over the carry: {np.round(slip, 1)} mm", flush=True)
    report("at the receiver")
    if mode == "letgo":  # cheat check: let go of the end at the receiver
        yield from l.open(t=0.3)
        yield from l.wait(0.5)
        report("let go")
        yield from l.wait(3.0)
        return
    if mode == "face":  # cheat check: push the end straight at the clip at the belt's height, as if it clicked in from the side, then let go
        yield from steer(l, Y_ENTER, 0.02, "L into the clip's face", tol=0.0008, extra=1.5)
        report("against the clip's face")
        yield from l.open(t=0.3)
        yield from l.wait(0.5)
        report("let go")
        yield from l.wait(3.0)
        return
    if mode == "under":  # cheat check: the end carried 40 mm under the belt's line, in under the clip, held there
        yield from vert(l, ZB - 0.040, 0.03, y_pre, "L down")
        yield from steer(l, Y_ENTER, 0.02, "L under the clip", tol=0.0008, extra=1.5, dz=-0.040)
        report("under the clip")
        yield from l.wait(4.0)
        report("under, end")
        return
    # lift it above the channel: the bead over the lips' top, under the clip's head
    yield from vert(l, ZB + LIFT, 0.03, y_pre, "L lift")
    # aim by feedback: read the end's heading, pitch and roll and the bead's offset from the channel's line; turn the hand about the end's centre
    # to null the angles, and steer it back onto the line, until all are within 0.6 mm and 0.6 degree
    for k in range(8):
        p = pose()
        yaw, pit, rol = yaw_deg(p), pitch_deg(p), roll_deg(p)
        dx, dz = bead_off()
        print(f"[demo] aim {k}: bead off the channel line x {dx:+.1f} mm, z {dz:+.1f} mm from the entry height; heading {yaw:+.1f} pitch {pit:+.1f} roll {rol:+.1f} deg", flush=True)
        if abs(dx) < 0.6 and abs(dz) < 1.5 and max(abs(yaw), abs(pit), abs(rol)) < 0.6:
            break
        for ang, ax in ((yaw, (0.0, 0.0, 1.0)), (pit, (1.0, 0.0, 0.0)), (rol, (0.0, 1.0, 0.0))):
            if abs(ang) >= 0.6:
                yield from l.arc(hpos(), ax, -ang, t=0.4, label=f"L aim turn {k}")
        yield from steer(l, y_pre, SPEED, f"L aim line {k}", tol=0.0006, extra=1.0, dz=LIFT)
    # in over the channel through the window, the bead behind where the lips stand below it
    yield from steer(l, Y_ENTER, 0.02, "L over the channel", tol=0.0006, extra=1.5, dz=LIFT)
    p = pose()
    dx, dz = bead_off()
    print(f"[demo] over the channel: bead x {dx:+.1f} mm, z {dz:+.1f} mm; heading {yaw_deg(p):+.1f} pitch {pitch_deg(p):+.1f} roll {roll_deg(p):+.1f}", flush=True)
    report("over the channel")
    ctx.snaps.append("over")
    if mode == "hover":  # cheat check: hold the end in the window over the channel, not lowered
        yield from l.wait(4.0)
        report("hover, end")
        return
    # lower it down the channel to the clip's foot, kept over the channel's line
    z_end = ZB + 0.020 if mode == "short" else ZB - 0.0005
    yield from vert(l, z_end, 0.02, Y_ENTER, "L lower", tol=0.0015, extra=2.0)
    report("lowered")
    ctx.snaps.append("down")
    if mode == "short":  # cheat check: 20 mm over the foot, still held
        yield from l.wait(4.0)
        report("short, end")
        return
    yield from l.wait(0.3)
    if not live:
        dry_hold.append(pose())
    if mode in ("farhold", "farback"):  # cheat check: seated, the left opens (jaws 70 mm, round the belt) and slides 0.09 m back along the belt, shuts on it 0.12 m behind the end
        z_seat = float(l.at()[2])
        yield from l.move(l.at(), finger=L_OPEN, t=0.5, label="L opens round the belt")
        yield from slide(l, -0.09 if mode == "farhold" else -0.27, 0.03, "L back along the belt")
        yield from l.servo(np.array([float(l.at()[0]), float(l.at()[1]), z_seat]), tol=0.001, timeout=1.5, label="L down to the belt's line")  # shut at the belt's own height: no lift on the end
        yield from l.close(W_H, L_FORCE, label="L grip the belt")
        yield from l.wait(4.0)
        report("far hold, end")
        return
    if mode == "rightback":  # cheat check: seated and still held by the left, the right pinches the belt flat 0.65 m behind the end and holds it
        b_at = np.array([XB, -0.20, ZB + 0.01])
        yield from ramp(r, b_at + 0.08 * back_r, gr, finger=R_OPEN)
        yield from r.servo(b_at + 0.08 * back_r, tol=0.003, timeout=2.0, label="R over the belt")
        yield from r.move(b_at, t=1.2, label="R down on the belt")
        yield from r.servo(b_at, tol=0.002, timeout=2.0, label="R on the belt")
        yield from r.close(W_T, FORCE, label="R pinch the belt flat")
        yield from r.wait(0.5)
        yield from l.open(t=0.4)
        yield from l.move(l.at() + _f("BELT_OUT", 0.10) * back_l, t=_f("BELT_OUTT", 0.8), label="L out")
        yield from r.wait(4.0)
        report("right back, end")
        return
    if mode == "palm":  # cheat check: seated and let go, the left's hand turned about so that its body and palm rest 2 cm from the end, the fingertips 15 cm away
        yield from l.open(t=0.3)
        yield from ramp(l, np.array([XB - 0.15, float(hpos()[1]), ZB]), _frame((-1.0, 0.0, 0.0), (0.0, 0.0, 1.0)), n=6, t=1.0, finger=L_OPEN)
        yield from l.wait(4.0)
        report("palm, end")
        return
    yield from l.open(t=0.4)
    # both hands to rest over the middle of the table, the left backing off the clip as it goes: the case's mouth, the belt and the seated end in view
    yield from ctx.together(ramp(r, PARK_R, gr, finger=R_OPEN, n=8, t=1.0), ramp(l, PARK_L, gl, finger=L_OPEN, n=8, t=1.0))
    ctx.snaps.append("still")
    yield from l.wait(4.0)
    report("end")
