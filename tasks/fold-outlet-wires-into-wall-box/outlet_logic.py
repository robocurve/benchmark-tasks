"""Numbers, geometry, the copper model and the scoring checks for outlet-into-box. Imports only numpy, so the tests run
on any machine; task.py and labkit read everything from here.

Frame: metres, x forward from the robot, y to its left, z up; the table top is at z = TOP. The wall stands across the
right arm's side of the table, its face toward the robot at x = WALL_X, with a single-gang box set flush in it.

STATUS: wire-probe stage. This file sizes a first Isaac Sim test of the wire stand-in (does a jointed copper wire hold
a bend, can an outlet on three of them be packed into the box). Screws, the second cable and the task's real scoring
come after that test.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np

TOP = 0.7475  # the station's table top (yam_arena.task_file.TABLE_TOP["station"])
CONTROL_HZ = 20
INCH = 0.0254

# -- the wall and the box ------------------------------------------------------------------------------------------------
# The wall's face (toward the robot). At 0.30 the right arm couldn't reach in front of the hanging outlet with a forward
# grip (18 mm short, yam_arena IK, GPU run 15 Nov 2026). At 0.375 every probe waypoint reaches with yam_arena's own IK,
# and still does with each waypoint moved 3 cm in any direction (2 cm downward).
WALL_X = 0.375
WALL_T = 0.0127  # 1/2 in drywall
WALL_Y = (-0.24, 0.08)  # the wall panel's extent along y
WALL_H = 0.30  # it stands on the table, this tall
# The box opening's centre (y, z). Raised from 14 to 19 cm for round 2: a forward grip holding the outlet loses reach
# low down, and at 19 cm every held move reaches (yam_arena IK) with 2 cm to spare in every direction.
OPEN_C = np.array([-0.08, TOP + 0.19])

# An 18 cu in single-gang box (nonmetallic, new-work): inside 57 mm wide x 91 mm tall x 57 mm deep. Its two mounting
# bosses stand in from the top and bottom walls at the centre; the device's screw holes are 3 9/32 in (83.3 mm) apart.
# Round 13 (16 Nov): real sizes. A Carlon B118A is 57 x 95 x 68 to 73 mm outside (retailer listings, no drawing), so
# about 52 x 90 x 66 mm inside: narrower and deeper than rounds 1-12's 57 x 91.4 x 57 (the right volume, the wrong shape).
BOX_W, BOX_H, BOX_D = 0.052, 0.090, 0.066
BOX_WALL = 0.0025
SCREW_SPACING = 3.28125 * INCH  # 83.3 mm, the NEMA WD-6 device mounting-screw spacing
BOSS_W = 0.010
BOSS_P = 2 * (BOX_H / 2 - SCREW_SPACING / 2)  # how far each boss stands in: its hole sits at the screw spacing
BOX_VOLUME_IN3 = BOX_W * BOX_H * BOX_D / INCH**3

# -- the outlet (a duplex receptacle) --------------------------------------------------------------------------------------
# Device frame: origin at the centre of the strap's back face (the face that lands on the wall), axes as the world's
# when it is mounted upright: +x into the wall, +y to the robot's left, +z up.
STRAP_T = 0.0015  # the mounting strap (yoke), steel
# Round 13: a Leviton 5320 is 4.2 in tall, 1.31 to 1.33 in wide and 0.90 to 0.98 in deep overall (retailer listings).
STRAP_W, STRAP_H = 0.0335, 0.1067  # overall; its ends rest on the wall face above and below the box
# The yoke is only as wide as the body through its middle (STRAP_MID_H); the plaster ears widen it at both ends. So a
# hand, or the gripper, can hold the body by its sides.
STRAP_MID_H = 0.0947
EAR_H = (STRAP_H - STRAP_MID_H) / 2
BODY = (0.020, 0.0335, 0.068)  # the body behind the strap (x depth, y width, z height)
FACE = (0.0045, 0.0335, 0.068)  # the face in front of the strap
DEVICE_MASS = 0.07
SCREW_TABS = np.array([[0.0, 0.0, SCREW_SPACING / 2], [0.0, 0.0, -SCREW_SPACING / 2]])  # device frame
OUT = 0.115  # at the start, the strap's back face stands this far out from the wall (the wires pulled straight out)
DEVICE_START = np.array([WALL_X - OUT, OPEN_C[0], OPEN_C[1]])  # device origin, world


def face_x(device_x: float) -> float:
    """World x of the outlet's front face, for a device origin at ``device_x`` (upright)."""
    return device_x - STRAP_T - FACE[0]


# -- the wires (12 AWG copper) ---------------------------------------------------------------------------------------------
COPPER_E = 117e9  # Pa
COPPER_YIELD = 70e6  # Pa: annealed (soft-drawn) building wire
COPPER_DENSITY = 8960.0
D12 = 0.00205  # 12 AWG conductor diameter (m)
INSULATED_R = 0.0015  # THHN-style insulation, about 3.0 mm across
BARE_R = D12 / 2
# Round 13b: 8 in (203 mm) of free hot and neutral, each wire keeping two old folds (KINKS) from when it was last in
# the box. On the laptop model, smooth 7.5 and 8 in wires could be fed in with one gripper and no wire handling (tip the
# outlet in top edge first), and smooth 9 in wires could not; with two old folds, 7.5, 8 and 9 in wires all stalled
# 30 mm or more out with every wire outside. Round 13a (smooth 9 in) ran in Isaac on 16 Nov: the straight push stalled
# 49 mm out.
WIRE_LENGTH = 0.203
SEGMENT = WIRE_LENGTH / 19  # the wire length one joint stands for (10.68 mm): 61 links in all, PhysX allows 64
# The old folds: where along the wire (0 at the cable, 1 at the outlet), which way (a share of "up" and of "sideways"
# across the wire), each a V standing KINK_H off the wire's line over 2 * KINK_HALF of wire.
KINK_H, KINK_HALF = 0.009, 0.016
KINKS = ((0.62, 1.0, 0.0), (0.34, -0.6, -0.8))
MAX_BEND = math.radians(90.0)  # per joint: a 90-degree turn over 14 mm is about a 4.4-diameter bend radius


def bending_stiffness() -> float:
    """EI of the copper (N m^2); the insulation adds little and is left out."""
    return COPPER_E * math.pi * D12**4 / 64


def yield_moment() -> float:
    """Bending moment at which the copper's outer fibre yields (N m)."""
    return COPPER_YIELD * (math.pi * D12**4 / 64) / (D12 / 2)


def joint_stiffness(segment: float) -> float:
    """A jointed wire's joint stiffness (N m/rad) that bends like the solid wire: EI over the link length."""
    return bending_stiffness() / segment


def yield_angle(segment: float) -> float:
    """The elastic range of one joint (rad): bent further than this from its rest angle, the rest angle follows."""
    return yield_moment() / joint_stiffness(segment)


def plastic_targets(q, target, theta_y: float) -> np.ndarray:
    """One step of the elastic-perfectly-plastic rule. Each joint has a spring to a rest angle (``target``); when the
    joint is bent more than ``theta_y`` past its rest, the rest moves so it sits exactly ``theta_y`` behind. Unloaded,
    the joint then springs back by ``theta_y`` and keeps the rest of the bend, as annealed copper does."""
    q, target = np.asarray(q, float), np.asarray(target, float)
    return np.clip(target, q - theta_y, q + theta_y)


@dataclass(frozen=True)
class WireRun:
    """One conductor: it leaves the cable at ``root`` (world, at the back of the box) and ends on the outlet at
    ``terminal`` (device frame)."""

    name: str
    root: tuple
    terminal: tuple
    radius: float
    color: tuple
    s_dir: float = -1.0  # which way its pre-bent S goes first: -1 down then up, +1 up then down
    slack: float | None = None  # its length over its straight distance; None: SLACK
    length: float | None = None  # its free length (m); set, it overrides slack
    kinks: tuple | None = None  # its old folds; None: KINKS


# Round 13: the cable comes in at the top back, off the centre line (the box's bosses run down the centre line).
ANCHOR = np.array([WALL_X + BOX_D - 0.008, OPEN_C[0] + 0.012, OPEN_C[1] + 0.030])  # where the cable's sheath ends, world
HOT, NEUTRAL, GROUND = (0.08, 0.08, 0.08), (0.95, 0.95, 0.93), (0.72, 0.45, 0.20)
TERMINAL_Y = BODY[1] / 2 + 0.003  # the wire leaves its screw 3 mm off the body's side (the screw head's height)
END_FILTER = 3  # links at each wire's outlet end that don't collide with the outlet (they run under its screws)
WIRES = (
    WireRun("hot", tuple(ANCHOR + (0.0, -0.0045, 0.0)), (0.010, -TERMINAL_Y, 0.012), INSULATED_R, HOT,
            length=WIRE_LENGTH),  # brass screw, the robot's right side of the face
    WireRun("neutral", tuple(ANCHOR + (0.0, 0.0045, 0.0)), (0.010, TERMINAL_Y, 0.012), INSULATED_R, NEUTRAL,
            length=WIRE_LENGTH),  # silver screw
    # Green screw low on the body's side (round 10: over the mounting hole it sat inside the bottom boss when seated).
    # Its folds are placed to stay 1.4 mm clear of the neutral, which runs beside it (folded the same way they touch).
    WireRun("ground", tuple(ANCHOR + (0.0, 0.0, -0.004)), (0.004, TERMINAL_Y, -0.030), BARE_R, GROUND,
            length=18 * SEGMENT, kinks=((0.62, -1.0, 0.0), (0.30, -0.6, -0.8))),
)
SIDE_GROUND = WIRES[2]
GROUNDS = {"side15": SIDE_GROUND}
BUILDS = {  # name -> (ground wire or None, hot/neutral slack or None); one simulator launch per build
    "side15": (SIDE_GROUND, None),  # the candidate task (the name is kept from round 10)
    "none": (None, None),  # diagnostic: hot and neutral only
}


def wires(ground: str = "side15") -> tuple:
    """The wires of build ``ground`` (BUILDS): hot, neutral and the ground, or hot and neutral only ("none")."""
    if ground not in BUILDS:
        raise ValueError(f"ground must be one of {tuple(BUILDS)}, got {ground!r}")
    g, _ = BUILDS[ground]
    return (WIRES[0], WIRES[1]) if g is None else (WIRES[0], WIRES[1], g)


def terminal_world(wire: WireRun, device_pos=DEVICE_START, device_rot=None) -> np.ndarray:
    rot = np.eye(3) if device_rot is None else np.asarray(device_rot, float)
    return np.asarray(device_pos, float) + rot @ np.asarray(wire.terminal, float)


# Round 4: the wires start pre-bent, the way they come out of a box when an outlet has been pulled out to change it,
# not as straight rods (pushed end-on, a straight 12 AWG rod needs far more force to start folding than the arm gives
# before it sags: round 3). Each wire is SLACK times its straight root-to-terminal distance, in one gentle S in the
# vertical plane: down first (inside the box, below the cable entry and clear of the top boss), then up outside. The S
# is the wire's relaxed shape: labkit spawns it with that as zero bend.
SLACK = 1.25


def straight_distance(wire: WireRun) -> float:
    return float(np.linalg.norm(terminal_world(wire) - np.asarray(wire.root)))


def _s_curve(wire: WireRun, amp: float, n: int = 6000) -> np.ndarray:
    """Dense points of the wire's S: the straight line plus s_dir * amp * sin(2 pi s), in the vertical plane through
    it."""
    a, b = np.asarray(wire.root, float), terminal_world(wire)
    d = (b - a) / np.linalg.norm(b - a)
    up = np.array([0.0, 0.0, 1.0]) - d[2] * d
    up /= np.linalg.norm(up)
    s = np.linspace(0.0, 1.0, n)[:, None]
    side = np.cross(d, up)
    half = KINK_HALF / wire_length(wire)
    folds = sum(KINK_H * np.maximum(0.0, 1.0 - np.abs(s - at) / half) * (u * up + v * side)
                for at, u, v in (KINKS if wire.kinks is None else wire.kinks))
    return a + s * (b - a) + wire.s_dir * amp * np.sin(2 * np.pi * s) * up + folds


def _arc(points: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def _s_amplitude(wire: WireRun) -> float:
    """The S's amplitude that makes the wire SLACK times its straight distance."""
    want, lo, hi = wire_length(wire), 0.0, straight_distance(wire)
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _arc(_s_curve(wire, mid)) < want else (lo, mid)
    return (lo + hi) / 2


def _march(curve: np.ndarray, chord: float, n: int) -> np.ndarray:
    """n + 1 points along a dense curve, each ``chord`` (straight-line) from the last."""
    pts, i = [curve[0]], 0
    for _ in range(n):
        last = pts[-1]
        d = np.linalg.norm(curve[i + 1:] - last, axis=1)
        j = np.argmax(d >= chord) if np.any(d >= chord) else len(d) - 1
        k = i + 1 + j  # first dense point at least ``chord`` away; refine between k - 1 and k
        p0, p1 = curve[k - 1], curve[k]
        lo, hi = 0.0, 1.0
        for _ in range(50):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if np.linalg.norm(p0 + mid * (p1 - p0) - last) < chord else (lo, mid)
        pts.append(p0 + lo * (p1 - p0))
        i = k - 1
    return np.array(pts)


def wire_length(wire: WireRun) -> float:
    """The wire's free length along its relaxed S: its ``length``, or its slack (or SLACK) times the straight distance."""
    return wire.length if wire.length else (wire.slack or SLACK) * straight_distance(wire)


def joints(wire: WireRun) -> int:
    """Bending joints along the wire: one per SEGMENT of its length."""
    return max(2, round(wire_length(wire) / SEGMENT))


def segments(wire: WireRun) -> int:
    """Rigid links: a half link clamped at each end and whole links between (one more than the joints)."""
    return joints(wire) + 1


def start_polyline(wire: WireRun) -> np.ndarray:
    """The wire's link end points at the start (segments + 1, 3), along its S: a half link, whole links, a half link,
    so every joint sits at the middle of the piece of wire it stands for (a clamped wire then bends by the right
    amount; with a joint at the clamp it was 8 to 11% out)."""
    curve, n = _s_curve(wire, _s_amplitude(wire)), joints(wire)
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))])
    s = cum[-1] / n
    arcs = np.concatenate([[0.0], s / 2 + s * np.arange(n), [cum[-1]]])
    pts = np.array([[np.interp(a, cum, curve[:, k]) for k in range(3)] for a in arcs])
    pts[0], pts[-1] = curve[0], curve[-1]
    return pts


def start_hardening(wire: WireRun) -> np.ndarray:
    """The plastic curvature (1/m) each joint starts with: the bend its start shape already has. A wire that holds a
    bent shape was bent into it, so its old folds start work-hardened and stiffer than the straight runs between."""
    pts = start_polyline(wire)
    u = np.diff(pts, axis=0)
    u = u / np.linalg.norm(u, axis=1)[:, None]
    turn = np.arccos(np.clip(np.sum(u[:-1] * u[1:], axis=1), -1.0, 1.0))
    return turn / (wire_length(wire) / joints(wire))


def links_needed() -> int:
    """Articulation links: the anchor, every wire link, the outlet. PhysX allows 64."""
    return 1 + sum(segments(w) for w in wires()) + 1


def box_fill_in3(insulated: int, grounds: int, devices: int = 1, internal_clamps: bool = True) -> float:
    """NEC 314.16(B) box fill for 12 AWG (2.25 cu in per allowance): each insulated conductor 1, all grounds together
    1, each device yoke 2, internal clamps together 1."""
    n = insulated + (1 if grounds else 0) + 2 * devices + (1 if internal_clamps else 0)
    return 2.25 * n


# -- checks the scorer and the probe use ---------------------------------------------------------------------------------
FLUSH_GAP = 0.002  # the strap may stand at most 2 mm off the wall at any corner
TAB_TOL = 0.003  # each screw tab within 3 mm of its boss hole, across the wall (y, z)
SHOWING = 0.002  # a wire more than this in front of the wall face is "showing"


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def strap_corners(device_pos, device_quat) -> np.ndarray:
    """The four corners of the strap's back face, world (4, 3)."""
    rot = _rot(device_quat)
    local = np.array([[0.0, sy * STRAP_W / 2, sz * STRAP_H / 2] for sy in (-1, 1) for sz in (-1, 1)])
    return np.asarray(device_pos, float) + local @ rot.T


def strap_gaps(device_pos, device_quat) -> np.ndarray:
    """How far each strap corner stands off the wall face (m; positive = out toward the robot)."""
    return WALL_X - strap_corners(device_pos, device_quat)[:, 0]


def boss_holes() -> np.ndarray:
    """The two boss holes' centres, world (2, 3), on the wall face."""
    return np.array([[WALL_X, OPEN_C[0], OPEN_C[1] + s * SCREW_SPACING / 2] for s in (1, -1)])


def tab_offsets(device_pos, device_quat) -> np.ndarray:
    """Each screw tab's distance from its boss hole across the wall (y, z), m."""
    tabs = np.asarray(device_pos, float) + SCREW_TABS @ _rot(device_quat).T
    return np.linalg.norm(tabs[:, 1:] - boss_holes()[:, 1:], axis=1)


def flush(device_pos, device_quat) -> bool:
    return bool(np.max(strap_gaps(device_pos, device_quat)) <= FLUSH_GAP
                and np.max(tab_offsets(device_pos, device_quat)) <= TAB_TOL)


def wire_showing(polylines: dict) -> list:
    """Names of wires with any joint point more than SHOWING in front of the wall face."""
    return [n for n, pts in polylines.items() if np.min(np.asarray(pts)[:, 0]) < WALL_X - SHOWING]


RIM = 0.003  # a wire point this close behind the wall face (or in front of it) must be inside the box opening


def wires_outside(polylines: dict) -> list:
    """Names of wires not wholly inside the box: any point more than SHOWING in front of the wall face, or outside the
    opening's outline while within RIM of the face (looped over the edge, or trapped under the strap). A trapped bare
    ground leaves the strap only 2 mm off the wall, so the strap's gap alone can't catch it.

    Each wire's LAST point, where it is fixed to the outlet's terminal, is not judged by the SHOWING rule: it is part
    of the outlet and sits wherever the outlet does, about 6.5 mm behind the strap, so with the strap allowed NEAR_GAP
    off the wall it can be up to 8.5 mm in front of the face however well the wire is folded (17 Nov: a run with every
    wire folded in and the strap 9 mm out at the ground's corner had only that point at the limit). Every other point
    of the wire is judged, and the outline rule still covers the last point."""
    out = []
    for n, pts in polylines.items():
        p = np.asarray(pts, float)
        off = (np.abs(p[:, 1] - OPEN_C[0]) > BOX_W / 2) | (np.abs(p[:, 2] - OPEN_C[1]) > BOX_H / 2)
        loose = p[:-1] if len(p) > 1 else p
        if np.min(loose[:, 0]) < WALL_X - SHOWING or np.any(off & (p[:, 0] < WALL_X + RIM)):
            out.append(n)
    return out


CLEAR = 0.03  # every gripper point (TCP and pad tips, both arms) at least this far from the outlet: it was let go
NEAR_GAP = 0.015  # partial credit: the strap within 15 mm of the wall at every corner, the body mostly in the box
UPRIGHT_DEG = 15.0  # the outlet's long axis within this of vertical (an outlet turned over or sideways scores 0)
STILL_SPEED = 0.01  # m/s: the outlet must be at rest for credit (task_score's own check, for a speed handed to it)
STILL_MOVE = 0.001  # m: at rest means no corner of the strap further than this from where it was (rest_count)


def device_hull() -> tuple:
    """The outlet's outline in its own frame (min corner, max corner): face, strap with its ears, and body."""
    lo = np.array([-STRAP_T - FACE[0], -STRAP_W / 2, -STRAP_H / 2])
    hi = np.array([BODY[0], STRAP_W / 2, STRAP_H / 2])
    return lo, hi


def hand_clearance(points, device_pos, device_quat) -> float:
    """The nearest gripper point's distance to the outlet's outline (m; 0 if touching or inside)."""
    pts = np.atleast_2d(np.asarray(points, float))
    local = (pts - np.asarray(device_pos, float)) @ _rot(device_quat)  # world -> device frame
    lo, hi = device_hull()
    out = np.maximum(np.maximum(lo - local, local - hi), 0.0)
    return float(np.min(np.linalg.norm(out, axis=1)))


def tilt_from_upright(device_quat) -> float:
    """Degrees between the outlet's long axis (device z) and world vertical."""
    up = _rot(device_quat)[:, 2]
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def roll_deg(device_quat) -> float:
    """The outlet's turn in the wall plane (about world x), degrees: positive lifts its +y side (the neutral side)."""
    across = _rot(device_quat)[:, 1]
    return float(np.degrees(np.arctan2(across[2], across[1])))


CLEAR_S = 2.0  # full credit needs the grippers clear of the outlet and of every wire, and the outlet still, for this long


def wire_clearance(points, polylines: dict) -> float:
    """The nearest gripper point's distance to any wire (m): a gripper holding the wires in the box is not "let go"."""
    pts = np.atleast_2d(np.asarray(points, float))
    best = float("inf")
    for line in polylines.values():
        p = np.asarray(line, float)
        a, b = p[:-1], p[1:]
        ab = b - a
        t = np.clip(np.einsum("gsk,sk->gs", pts[:, None, :] - a[None], ab) / np.maximum(np.einsum("sk,sk->s", ab, ab), 1e-12), 0, 1)
        near = a[None] + t[..., None] * ab[None]
        best = min(best, float(np.min(np.linalg.norm(pts[:, None, :] - near, axis=2))))
    return best


def rest_count(count_s: float, anchor, corners, hand_clear: float, dt: float):
    """How long the outlet has been let go and at rest, one control step on: (seconds, the strap corners the count
    is measured from). It is at rest while no corner of its strap is more than STILL_MOVE from where it was when the
    count began, the way a ruler would show it; a gripper within CLEAR, or a corner that has moved, starts the count
    again from here. (The simulator's own speed reading is not used: it flickers on a body that is going nowhere.)"""
    corners = np.asarray(corners, float)
    if anchor is not None and hand_clear >= CLEAR and float(np.max(np.linalg.norm(corners - anchor, axis=1))) <= STILL_MOVE:
        return count_s + dt, anchor
    return 0.0, corners


def task_score(device_pos, device_quat, polylines: dict, hand_clear: float, speed: float = 0.0,
               clear_s: float | None = None) -> float:
    """The task's score at one step (task.py's score() feeds it the outlet's state, the grippers' clearance from the
    outlet and the wires, and how long they have been clear with the outlet still).

    Ready for its screws. 0 unless the outlet is upright (within 15 degrees), every wire is inside the box
    (wires_outside), every strap corner is within 15 mm of the wall (a 1 in mounting screw still reaches its hole) AND
    it has been let go: both grippers at least 3 cm from the outlet and from every wire, the outlet at rest, for the
    last CLEAR_S seconds. Then 1.0 if both screw tabs are within 3 mm of their boss holes, 0.5 if it is off its holes.
    Nothing is scored while a gripper still holds or presses the outlet, or while it moves: held in, it would spring
    out when let go (17 Nov: half credit used to be given while held; a run scored 0.5 pressed and 0 once released).
    With real copper it springs back about 7 mm when let go, so flat on the wall without screws is not asked for."""
    if tilt_from_upright(device_quat) > UPRIGHT_DEG or wires_outside(polylines):
        return 0.0
    if float(np.max(strap_gaps(device_pos, device_quat))) > NEAR_GAP:
        return 0.0
    if not (hand_clear >= CLEAR and speed <= STILL_SPEED and (clear_s is None or clear_s >= CLEAR_S)):
        return 0.0
    return 1.0 if float(np.max(tab_offsets(device_pos, device_quat))) <= TAB_TOL else 0.5


def probe_score(device_pos, device_quat, polylines: dict) -> float:
    """The probe's own check (not the task's score): 1.0 flush with no wire showing, 0.5 flush with a wire showing
    (the strap can't be flush over a trapped wire in reality, so this flags the stand-in), else 0."""
    if not flush(device_pos, device_quat):
        return 0.0
    return 0.5 if wire_showing(polylines) else 1.0


# -- the probe's scripted moves -------------------------------------------------------------------------------------------
PAD_TIP = 0.013  # how far the pad tips stand past the TCP (yam_arena.constants.PAD_TIP_PAST_TCP)
FACE_C = np.array([face_x(DEVICE_START[0]), *OPEN_C])  # the outlet face's centre at the start, world
PUSH_OVER = 0.004  # press this far past flush (the arm gives a little)


def _tcp_forward(tip) -> tuple:
    """TCP of a forward grip (fingers pointing +x) whose pad tips are at ``tip``."""
    return (float(tip[0] - PAD_TIP), float(tip[1]), float(tip[2]))


# Round 2 (after the first GPU run): the right arm holds the outlet's body by its sides (fingers close along y), so it
# can't slip off as a fingertip did. The pad tips go GRIP_DEPTH past the face's front, a little below centre
# (GRIP_DZ): clear of the hot and neutral terminals (12 mm above centre, 12 mm behind the strap).
GRIP_DEPTH = 0.012
GRIP_DZ = -0.012
GRIP = np.array([-STRAP_T - FACE[0] + GRIP_DEPTH - PAD_TIP, 0.0, GRIP_DZ])  # TCP minus device origin, while held
# Round 4: the fingers are told to close only to GRIP_FINGER (each finger's joint, m; yam_arena: 0 closed, -0.04695
# open), 5 mm inside the body's sides (they touch at -BODY_W/2). Each finger's drive is a 5000 N/m spring capped at
# 50 N, so this squeezes about 25 N a side: under the cap, so the grip stays stiff sideways and along the push. Fully
# closed (round 3), both fingers sat at the cap and the outlet slid about 12 mm between them.
FINGER_OPEN_J, FINGER_STIFFNESS, FINGER_EFFORT = -0.04695, 5000.0, 50.0
GRIP_FINGER = -(BODY[1] / 2 - 0.005)
SEATED_X = WALL_X + PUSH_OVER  # where a seating move drives the device origin (the wall stops it at WALL_X)
STAND_OFF = 0.035  # fold paths line up this far in front of the wall before the last straight push


def device_paths() -> dict:
    """Each held path as device-origin waypoints [(seconds, (x, y, z), label)], from the start pose. The arm's TCP is
    the device origin + GRIP. Straight lines between waypoints (yam_arena interpolates the TCP that way)."""
    x0, y, z = DEVICE_START
    front = WALL_X - STAND_OFF
    # Round 4: the pushes are slower (the arm is soft: the faster it is asked to go against a load, the more its hand
    # sags behind its target).
    seat = [(2.0, (front, y, z), "line up in front of the box"), (5.0, (SEATED_X, y, z), "push it home")]
    return {
        # The habit move, held: straight in, slowly, all the way.
        "straight": [(12.0, (SEATED_X, y, z), "press straight in")],
        # Technique paths: move in while carrying the outlet low (or high, or to the side), so the wires take a
        # deliberate bend at the box, then bring it level in front of the opening and push it straight home.
        "fold-down": [(3.0, (WALL_X - 0.07, y, z - 0.035), "in and down"),
                      (2.0, (front, y, z - 0.03), "close in, still low"),
                      (1.5, (front, y, z), "up level with the box"), *seat[1:]],
        "fold-up": [(3.0, (WALL_X - 0.07, y, z + 0.035), "in and up"),
                    (2.0, (front, y, z + 0.03), "close in, still high"),
                    (1.5, (front, y, z), "down level with the box"), *seat[1:]],
        "fold-side": [(3.0, (WALL_X - 0.07, y - 0.035, z), "in and to the right"),
                      (2.0, (front, y - 0.03, z), "close in, still right"),
                      (1.5, (front, y, z), "back in line with the box"), *seat[1:]],
        # The bend test: carry it 4 cm to the robot's left (toward the middle: the right arm reaches that way more
        # easily than toward its own base), in front of the wall, and let go.
        "bend": [(3.0, (x0, y + 0.04, z), "carry it 4 cm left"), (1.0, (x0, y + 0.04, z), "hold")],
    }


def probe_plan(variant: str) -> dict:
    """{"left": [...], "right": [...]}, each step (seconds, TCP (x, y, z) or None, fingers, grip, label); grip is
    "forward" (fingers pointing +x) or "ready". The left arm stays at ready throughout. Variants:

    hang       no motion for 12 s: is the start stable, how far does the outlet sag on its wires?
    bend       grip the outlet, carry it 4 cm right, let go, back out: how much of the bend stays?
    straight   grip it and push it straight in to the wall: the habit move, held so it can't slip off.
    fold-down  grip it, carry it in low so the wires bend down at the box, level it, push it home.
    fold-up    the same, carried high.
    fold-side  the same, carried to the right.
    """
    ready = (1.5, None, "open", "ready", "ready")
    if variant == "hang":
        return {"left": [(12.0, None, "open", "ready", "wait")], "right": [(12.0, None, "open", "ready", "wait")]}
    if variant == "air":
        # Diagnostic (round 3): the bend's hand motion, timing and closed fingers, but AIR_LIFT above the outlet,
        # touching nothing. How well does the arm track this carry with no load at all?
        up = np.array([0.0, 0.0, AIR_LIFT])
        right = [(s[0], None if s[1] is None else tuple(float(v) for v in np.asarray(s[1]) + up), "closed", s[3],
                  f"air: {s[4]}") for s in probe_plan("bend")["right"][:-1]]
        right.append(ready)
        total = sum(s[0] for s in right)
        return {"left": [(total, None, "open", "ready", "wait")], "right": right}
    if variant not in device_paths():
        raise ValueError(f"unknown probe variant {variant!r}: one of {VARIANTS}")
    tcp = lambda d, dx=0.0: tuple(float(v) for v in np.asarray(d) + GRIP + (dx, 0.0, 0.0))  # noqa: E731
    start = DEVICE_START
    right = [
        (2.5, tcp(start, -0.05), "open", "forward", "in front of the outlet, open"),
        (1.5, tcp(start), "open", "forward", "fingers either side of the body"),
        (1.0, tcp(start), "grip", "forward", "grip"),
    ]
    last = start
    for seconds, d, label in device_paths()[variant]:
        right.append((seconds, tcp(d), "grip", "forward", label))
        last = d
    right += [
        (1.5, tcp(last), "grip", "forward", "hold"),
        (0.8, tcp(last), "open", "forward", "let go"),
        (1.5, tcp(last, -0.06), "open", "forward", "back straight out"),
        (3.0, tcp(last, -0.06), "open", "forward", "watch it spring back"),
        # Away from the outlet before heading home: in run 1 the arm's way back to ready knocked the outlet.
        (1.5, (0.20, float(last[1]), float(last[2]) + 0.10), "open", "forward", "up and away"),
        ready,
    ]
    total = sum(s[0] for s in right)
    return {"left": [(total, None, "open", "ready", "wait")], "right": right}


AIR_LIFT = 0.12  # the free-air diagnostic runs this far above the bend's path: clear of the outlet's top ear

VARIANTS = ("hang", "bend", "air", "straight", "fold-down", "fold-up", "fold-side")
FOLDS = ("fold-down", "fold-up", "fold-side")


def plan_seconds(plan: dict) -> float:
    return max(sum(s[0] for s in steps) for steps in plan.values())


# -- round 5: many trials in one simulator launch -------------------------------------------------------------------------
# Round 4 (15 Nov): the straight push carried the outlet 82 mm in (strap 117 -> 36 mm off the wall), the grip held and
# the wires stayed folded after release. It stalled because the hand was aimed only 4 mm past the wall and the YAM's
# soft joint servos let the hand trail its target by 33 mm, pushing about 20 N, with every joint well under its torque
# limit. So round 5 aims the push past the wall (AIM), the way a person pushes "through" a part that is meant to stop
# against something, and tries the techniques electricians use (wiggle, pump) against the same aim.
#
# It also tries a fidelity fix. The plastic rule runs once per control step (20 Hz) but PhysX runs 12 substeps per
# control step: within one step a joint is a pure spring, so a fast-moving wire resists with up to k * (angle moved in
# 50 ms), several times copper's yield moment. Capping each wire joint's drive torque at the yield moment (``cap``)
# makes it elastic-perfectly-plastic at every substep: it can never push back harder than copper does.

@dataclass(frozen=True)
class Trial:
    name: str
    path: str  # straight, wiggle, pump, fold-down, fold-up, bend, wall-press, lift, left-tuck
    aim: float = 0.05  # how far past the wall face the hand drives the device origin (m); the wall stops the strap
    push_s: float = 8.0  # how long the push from the start pose to the aim takes
    cap: bool | None = True  # wire joint drive torque capped at copper's yield moment; None: the wires play no part
    why: str = ""
    ground: str = "below"  # the ground wire's start shape (GROUNDS); one simulator launch per shape
    lift: float = 0.0  # lift: how high the outlet is carried in before it's lowered level and pushed home (m)
    pokes: tuple = ()  # left-tuck: each poke's fingertip (y at the start, y at the end, z), from the opening's centre
    park: str = "right"  # left-tuck: where the right hand holds the outlet while the left hand works (PARKS)
    poke_from: float = 0.04  # left-tuck: how far in front of the wall each poke starts (m)
    tuck: tuple = ()  # ground-tuck: (x in front of the wall, lifted height, pushed-to x, yaw) of the left fingertip
    press: float = 0.05  # aim-thumb: how far past the seated face the closed fingertips press (m)
    mouth: float = 0.0  # aim-thumb: pause to centre the outlet with its strap this far off the wall (0: no pause)
    centre_s: float = 2.5  # aim-thumb: how long that pause lasts (the steering updates every 0.5 s)
    # Round 12 (aim-thumb):
    steer: str = "slow"  # "slow": rounds 10-11 (y, z every 0.5 s); "fast": y, z and the outlet's roll every 0.1 s
    gate: bool = False  # the push stops advancing while the outlet is off-centre or turned (probe.py)
    mouth_s: float = 4.0  # how long the push to the mouth takes
    crest: tuple = ()  # the left hand presses the hot/neutral crest into the box mouth: names in CRESTS, in order
    dip: float = 0.0  # at the stand-off, lower the outlet this far and bring it back up (drags the crest down)
    grip_off: tuple = (0.0, 0.0)  # (y, z) offset of the grip on the body (m): a robustness repeat, not a technique
    # Round 13b: the left hand's presses aimed at where each wire's loop really is (read from the scene just before it
    # moves; a scripted demo may read the scene), not at fixed spots.
    live: bool = False
    crest_in: float = 0.012  # live: how far past the wall face the fingertips end their sweep (m)
    crest_low: float = 0.016  # live: the fingertips' height above the box centre at the end of the sweep (lip: 0.045)
    crest_back: float = 0.008  # live: the sweep starts this far to the outlet's side of the loop's top (m)
    crest_up: float = 0.010  # live: and this far above it
    # Round 13d: let the outlet do most of the wire work. ``tip``: the right hand brings the outlet in
    # with its top edge leading by this many degrees and levels it on the last push (on the laptop this fed the hot
    # wire in under the strap's top edge; the neutral, standing beside the strap, was the one left out). ``wire_move``
    # "lean": the left hand pushes a loop sideways toward the opening's centre with the side of its closed fingers,
    # so it stands behind the strap too (lean_plan_at); "sweep": crest_plan_at.
    tip: float = 0.0
    wire_move: str = "sweep"
    # Round 13e: the loops ride 60 to 75 mm above the outlet's centre wherever it is (rounds 13b-d), so an outlet
    # brought in at the opening's height carries them in above the box's top lip. ``low_in``: bring it in this far
    # BELOW the opening's centre instead (m), up to ``low_out`` in front of the wall, so the loops arrive in front of
    # the open mouth, under the lip; then lift it level with the opening (``rise_s`` seconds) and push home.
    low_in: float = 0.0
    low_out: float = 0.022
    rise_s: float = 4.0
    # Round 13f, from what rounds 13b-e measured. The left fingertip is a soft spring (about 0.2 N per mm sideways):
    # ``crest_over`` and ``lean_over`` aim each move that far PAST where the wire should end up, so it is pushed with
    # 8 N or more, and the probe stops a lean once the loop is there. ``track``: during a sweep the probe keeps the
    # fingertips on the wire's measured y. ``crest_drop``: how far the right hand lowers the outlet for the left hand
    # (the arm gives up about a third of it); lower keeps the strap's top out from under the left gripper's root and
    # brings the loops under the box's top lip. ``moves``: one of "sweep" / "lean" per entry of ``crest`` (a wire may be
    # named twice), used when ``wire_move`` is "seq".
    crest_over: float = 0.0
    lean_over: float = 0.0
    track: bool = False
    crest_drop: float = 0.028
    moves: tuple = ()
    # Round 13g: with the loops low the sweep's line was shallower than the fingers' own slope and the fingertips
    # skimmed over the loop's top (g-run). ``along``: sweep along the fingers' own axis, on a line through the loop's
    # top, so their end face meets it square (used when that line passes under the box's lip).
    along: bool = False
    # Round 13i: the final fingertip press lands this far above the face's centre (negative: below, to bring in a
    # bottom edge that stands further out than the top).
    press_dz: float = 0.0
    press_dy: float = 0.0  # and this far toward +y (the ground wire's side: i-run left that bottom corner 8.9 mm out)
    # The two-handed run built a piece at a time: 0 runs it all; 1 stops with the outlet held low at the stand-off;
    # 2, 3, ... stop after the first, second, ... wire is pressed in; one more than that stops with the outlet pushed
    # home and still held. The trial then holds still for 2 s, so the log shows what that piece left behind.
    pieces: int = 0


ROUND5 = (
    Trial("deep50", "straight", 0.05, cap=False,
          why="round 4's physics, the push aimed 5 cm past the wall: was the stall only the aim?"),
    Trial("deep50-cap", "straight", 0.05,
          why="the same with wire torque capped at copper's yield moment (the fidelity fix)"),
    Trial("wall-press", "wall-press", cap=None,
          why="the arm's own limit: closed hand pressed on the bare wall, aimed 0 to 8 cm past it, force from torques"),
    Trial("wiggle50-cap", "wiggle", 0.05, why="push while rocking the outlet up and down 8 mm, as electricians do"),
    Trial("pump50-cap", "pump", 0.05, why="push 60% of the way, ease back 15 mm, push home: lets the folds settle"),
    Trial("fold-down50-cap", "fold-down", 0.05, why="carry it in low, level it, push home aimed 5 cm past"),
    Trial("deep80-cap", "straight", 0.08, why="a harder push: how does seating change with aim (force)?"),
    Trial("slow50-cap", "straight", 0.05, push_s=16.0, why="half the speed: is the wire's resistance rate-dependent?"),
    Trial("bend-cap", "bend", cap=True, why="does capped wire still keep a 4 cm bend once let go?"),
    Trial("fold-up50-cap", "fold-up", 0.05, why="carry it in high, level it, push home aimed 5 cm past"),
)
# Round 6 (15 Nov), for the record: the dressed-inside ground was invalid (it spawned through the body). left-tuck got
# the outlet to 22 mm, the closest yet, but its pokes touched nothing: with the outlet parked up and to the right, the
# ground crossed the box mouth 7 to 31 mm right of centre and the fingertip went 5 mm left of it. lift15: 42 mm.
ROUND6 = (
    Trial("left-tuck", "left-tuck", 0.05, pokes=((0.015, -0.005, -0.010), (0.015, -0.005, -0.030))),
    Trial("lift15", "lift", 0.05, lift=0.015),
    Trial("left-tuck3", "left-tuck", 0.05, pokes=((0.015, -0.005, -0.030), (0.015, -0.005, -0.010),
                                                  (0.015, -0.005, -0.030))),
)
# Round 7 (16 Nov), for the record: nothing seated (35 to 60 mm). Lifting the outlet 90 mm raised the ground's belly to
# 20 mm below centre, but the low sweep passed under it and the higher one stalled in the wire bundle; and lowering the
# outlet again dropped the belly back to 65 mm below centre: the slack hangs from the ground screw, so it follows the
# outlet wherever the outlet goes.
ROUND7 = (
    Trial("scoop-up", "left-tuck", 0.05, park="up", poke_from=0.095,
          pokes=((0.005, 0.005, -0.030), (0.005, 0.005, -0.012))),
    Trial("poke-right", "left-tuck", 0.05, park="right", poke_from=0.045,
          pokes=((-0.020, -0.020, -0.020), (-0.012, -0.012, -0.005))),
    Trial("scoop-up3", "left-tuck", 0.05, park="up", poke_from=0.095,
          pokes=((0.005, 0.005, -0.030), (0.0, 0.0, -0.015), (-0.005, -0.005, -0.030))),
)
# Round 8 (16 Nov, for the record: 34 to 43 mm; the thin fingertip lifted one point of the ground and the slack
# draped over it): move the belly on its own, the outlet left hanging where it starts. At the start the belly lies 52 to 58 mm
# below centre, 56 to 83 mm in front of the wall, on the centre line: between the outlet body (which ends 88 mm out) and
# the wall, with nothing over it but its own wire. The left fingertip, turned 70 degrees so the finger runs back to the
# robot's left clear of the outlet (at 45 degrees the finger's wide base reached the body's bottom corner; at 70 it
# clears every part by 10 mm or more, yam.urdf meshes), comes in under it from the left, lifts it into the opening's
# height and carries it toward the box mouth (at 70 degrees the finger's base, 48 mm wide, would hit the wall left of
# the box if the tip went closer than about 20 mm); then the right hand grips the outlet and pushes it straight home,
# the body sweeping the ground in. Tuck the ground first, then press the device in: the electrician's order.
ROUND8 = (
    Trial("tuck-ground", "ground-tuck", 0.05, tuck=(0.060, -0.035, -0.030, -70.0),
          why="left fingertip lifts the ground's belly into the opening's height and carries it to 30 mm from the box "
              "mouth; then the right hand pushes the outlet straight home"),
    Trial("tuck-ground-hi", "ground-tuck", 0.05, tuck=(0.060, -0.028, -0.030, -70.0),
          why="the same, the belly lifted 7 mm higher"),
    Trial("tuck-ground-60", "ground-tuck", 0.05, tuck=(0.060, -0.035, -0.012, -60.0),
          why="the finger turned 60 degrees instead, so it can carry the belly to 12 mm from the box mouth"),
)
# Round 9 (16 Nov, for the record: 34 to 38 mm at every length; see round 10's note at BUILDS): the ground cut
# shorter, one launch per length, the tightest first. In each: the straight push (aimed 5 cm past
# the wall, wire torque capped) and the same push while rocking the outlet 8 mm up and down.
ROUND9 = tuple(
    Trial(f"s{pct:02d}-{kind}", path, 0.05, ground=f"slack{pct:02d}",
          why=f"ground {pct}% longer than straight: " + ("does the straight push seat it?" if kind == "straight"
                                                          else "push while rocking it 8 mm"))
    for pct in (6, 10, 8) for kind, path in (("straight", "straight"), ("wiggle", "wiggle"))
)
# Round 10 (16 Nov, for the record): b1-aim-thumb went flush (strap gap 0.0, held after release) but 3.4 mm
# low (the body on the bottom boss) and 3 mm off-centre (tabs 7.2/3.3 mm off; hot wire 2.5 mm proud); every
# naive push stalled 37-45 mm out. Under the push the wrist bends and the hand ends 22 mm below its target, with
# the steering at its +25 mm cap; once the strap is flat on the wall, friction stops any re-centring.
# Round 10: three builds, one launch each. The candidate task (side15) first: the "aim-thumb" technique (firm grip,
# the hand steered so the outlet stays centred on the opening, a pause at a stand-off, a push past the wall, then let go
# and press the face with closed fingertips, as a thumb does), the naive straight push, and a slower aim-thumb. Then
# every wire shortened to 10% (side15-short), then a diagnostic with no ground wire at all.
ROUND10 = (
    Trial("b1-aim-thumb", "aim-thumb", 0.04, ground="side15",
          why="side ground: firm grip, outlet kept centred, stand-off, push 4 cm past, then a fingertip press"),
    Trial("b1-naive", "straight", 0.05, ground="side15",
          why="side ground: round 9's straight push unchanged (is the one-cable box too easy now?)"),
    Trial("b1-aim-thumb-slow", "aim-thumb", 0.06, ground="side15", push_s=12.0, press=0.06,
          why="the same technique, a 12 s push aimed 6 cm past and a harder press"),
    Trial("b3-aim-thumb", "aim-thumb", 0.04, ground="side15-short",
          why="every wire 10% over straight: the technique"),
    Trial("b3-naive", "straight", 0.05, ground="side15-short", why="every wire 10% over straight: the straight push"),
    Trial("b2-naive", "straight", 0.05, ground="none", why="no ground wire: the straight push (isolates the ground)"),
    Trial("b2-aim-thumb", "aim-thumb", 0.04, ground="none", why="no ground wire: the technique"),
)
# Round 11 (overnight, about 1 h 45 min): three builds, one launch each. The technique gains a pause with the body just
# inside the box mouth: the steering (now up to +50 mm of lift) centres the outlet there, where nothing presses it on the
# wall, and only then does the hand push the last few millimetres. Swept: how far out the pause is (4, 8, 12 mm), how
# long it lasts, how fast and how deep the last push goes, how hard the fingertip press is. The naive push closes
# each build for the record; the shorter-wire and no-ground builds repeat the best guesses.
_A = dict(path="aim-thumb", aim=0.04, push_s=3.0)
ROUND11 = (
    Trial("c1-align8", ground="side15", mouth=0.008, **_A,
          why="pause with the strap 8 mm out, centre the outlet, push the last 8 mm, then the fingertip press"),
    Trial("c1-align4", ground="side15", mouth=0.004, **_A, why="the same pause 4 mm out"),
    Trial("c1-align12", ground="side15", mouth=0.012, **_A, why="the same pause 12 mm out"),
    Trial("c1-align8-long", ground="side15", mouth=0.008, centre_s=4.5, **_A, why="the 8 mm pause, 4.5 s long"),
    Trial("c1-align8-slow", ground="side15", mouth=0.008, **{**_A, "push_s": 5.0}, press=0.06,
          why="the 8 mm pause, a 5 s last push and a harder press"),
    Trial("c1-align4-slow", ground="side15", mouth=0.004, **{**_A, "push_s": 5.0}, press=0.06,
          why="the 4 mm pause, a 5 s last push and a harder press"),
    Trial("c1-align8-deep", ground="side15", mouth=0.008, **{**_A, "aim": 0.06},
          why="the 8 mm pause, the last push aimed 6 cm past the wall"),
    Trial("c1-naive", "straight", 0.05, ground="side15",
          why="the naive straight push again, for the record (round 10: 43.6 mm)"),
    Trial("c3-align8", ground="side15-short", mouth=0.008, **_A, why="every wire 10% over straight: the 8 mm pause"),
    Trial("c3-align4", ground="side15-short", mouth=0.004, **_A, why="every wire 10% over straight: the 4 mm pause"),
    Trial("c3-naive", "straight", 0.05, ground="side15-short", why="every wire 10% over straight: the naive push"),
    Trial("c2-align8", ground="none", mouth=0.008, **_A, why="no ground wire: the 8 mm pause"),
    Trial("c2-align4", ground="none", mouth=0.004, **_A, why="no ground wire: the 4 mm pause"),
)
# Round 11 (16 Nov, for the record): all ten technique trials went flush and stayed flush after release, both naive
# pushes stalled 35-45 mm out, and none seated: tabs 4-12 mm off. The mouth pause began 33 mm out (the loaded hand fell
# 25 mm short of its target), and on the way the outlet dropped 11 mm while the 0.5 s steering wound up to its +50 mm
# cap; it ended 3 mm low on the bottom boss, 2-5 mm left, turned 8-10 degrees (side ground; none without it). The side
# views show why it drops: the hot and neutral crest rides over the box's top lip and folds onto it (see CRESTS).
# Round 12: one launch, the side-ground build, about 2 h 15 min. The full technique (the left hand tucks the crest, fast
# steering with the roll, the gated push) first, then each part on its own, then the full technique gripped 3 mm off
# its usual spot (does the steering absorb it?), and the naive push for the record.
_B = dict(path="aim-thumb", aim=0.04, push_s=3.0, ground="side15", mouth=0.008)
_TUCK = ("hot", "neutral")
TRIALS = (
    Trial("d1-full", crest=_TUCK, steer="fast", gate=True, **_B,
          why="left hand tucks the hot and neutral crest past the lip; fast steering with roll; gated push"),
    Trial("d1-crest-open", crest=_TUCK, steer="none", **_B,
          why="the crest tucked, then no steering at all: does the tuck alone let it seat? (an open-loop demo)"),
    Trial("d1-fast-gate", steer="fast", gate=True, **_B, why="no tuck: fast steering with roll and the gated push"),
    Trial("d1-crest-fast", crest=_TUCK, steer="fast", **_B, why="the tuck and fast steering, no gate"),
    Trial("d1-full-gz-3", crest=_TUCK, steer="fast", gate=True, grip_off=(0.0, -0.003), **_B,
          why="the full technique gripped 3 mm low on the body (robustness)"),
    Trial("d1-full-gz+3", crest=_TUCK, steer="fast", gate=True, grip_off=(0.0, 0.003), **_B,
          why="the full technique gripped 3 mm high on the body (robustness)"),
    Trial("d1-full-gy+3", crest=_TUCK, steer="fast", gate=True, grip_off=(0.003, 0.0), **_B,
          why="the full technique gripped 3 mm to the left on the body (robustness)"),
    Trial("d1-dip", steer="fast", gate=True, dip=0.025, **_B,
          why="no left hand: dip the held outlet 25 mm at the stand-off to drag the crest down, then the push"),
    Trial("d1-full-slow", crest=_TUCK, steer="fast", gate=True, mouth_s=8.0, **_B,
          why="the full technique with an 8 s push to the mouth"),
    Trial("d1-fast", steer="fast", **_B, why="fast steering with roll only (against round 11's slow steering)"),
    Trial("d1-crest-open-gz-3", crest=_TUCK, steer="none", grip_off=(0.0, -0.003), **_B,
          why="the tuck, no steering, gripped 3 mm low (does an open-loop demo survive a slightly different grip?)"),
    Trial("d1-crest-open-gy+3", crest=_TUCK, steer="none", grip_off=(0.003, 0.0), **_B,
          why="the tuck, no steering, gripped 3 mm to the left"),
    Trial("d1-naive", "straight", 0.05, ground="side15", why="the naive straight push, for the record"),
)
ROUND12 = TRIALS
# Round 13 (the first with the copper rule, real sizes and 9 in wires). The laptop model (taskkit outlet-box) says:
# every push with no wire handling leaves a loop of hot and neutral standing above the outlet in front of the wall,
# 17 mm or more out; tipping the outlet in top edge first is the best of them (one wire left out); and the outlet
# springs back about 7 mm when let go. This launch checks those with the real arm, then tries the two-handed tuck.
_C = dict(path="aim-thumb", aim=0.04, push_s=3.0, ground="side15", mouth=0.012)
TRIALS = (
    Trial("e1-naive", "straight", 0.05, ground="side15", why="the straight push: expected to stall, wires outside"),
    Trial("e1-careful", steer="fast", gate=True, **_C,
          why="round 12's careful push, no wire handling: centred, gated, then the fingertip press"),
    Trial("e1-tilt", "tilt", 0.05, ground="side15",
          why="tip the outlet in top edge first, level it, push home: the best one-gripper trick on the laptop"),
    Trial("e1-tuck", crest=_TUCK, steer="fast", gate=True, **_C,
          why="left hand presses the hot and neutral loops down past the box's top lip, then the careful push"),
    Trial("e1-tuck-open", crest=_TUCK, steer="none", **_C, why="the same tuck, then an open-loop push"),
    Trial("e1-tuck-low", crest=_TUCK, steer="fast", gate=True, **{**_C, "mouth": 0.020},
          why="the tuck, pausing 20 mm out before the last push"),
)
# Round 13b's two-handed trials (kinked 8 in wires): the presses aimed live at each loop.
_ALL = ("hot", "neutral", "ground")
TUCKS = (
    Trial("f1-tuck", crest=_ALL, live=True, steer="fast", gate=True, **_C,
          why="left hand presses each wire's loop past the top lip (aimed at where it is), then the careful push"),
    Trial("f1-tuck-deep", crest=_ALL, live=True, crest_in=0.020, crest_low=0.008, steer="fast", gate=True, **_C,
          why="the same, each loop pushed 20 mm past the wall face and lower"),
    Trial("f1-tuck-two", crest=_TUCK, live=True, steer="fast", gate=True, **_C,
          why="the same for the hot and neutral only: does the ground need its own press?"),
)
# The same run stopped after each piece (Trial.pieces), for building it a piece at a time.
PIECES = tuple(Trial(f"p{k}", crest=_ALL, live=True, pieces=k, steer="fast", gate=True, **_C,
                     why=f"the two-handed run, stopped after piece {k}") for k in (1, 2, 3, 4, 5))
# Round 13c: the press redone as a sweep (crest_plan_at), each wire aimed just before its own press, hot and neutral
# only (the ground's loop sits in front of the mouth below the lip, and the finger cannot reach it past the outlet).
SWEEPS = (
    Trial("s-press", crest=_TUCK, live=True, pieces=3, steer="fast", gate=True, **_C,
          why="sweep the hot loop, then the neutral loop, into the box mouth; stop there"),
    Trial("s-run", crest=_TUCK, live=True, steer="fast", gate=True, **_C,
          why="the two sweeps, then the careful push"),
    Trial("s-run-deep", crest=_TUCK, live=True, crest_in=0.020, crest_low=0.008, steer="fast", gate=True, **_C,
          why="the same with each sweep ending 20 mm past the wall face and 8 mm lower"),
)
# Round 13d: tip the outlet in (Trial.tip), with and without the left hand leaning the neutral loop behind the strap.
_T = {**_C, "mouth": 0.020}
TIPS = (
    Trial("t-tip", tip=15.0, steer="fast", gate=True, **_T,
          why="no wire handling: the careful push with the top edge leading 15 degrees, levelled on the last push"),
    Trial("t-lean", crest=("neutral",), live=True, wire_move="lean", tip=15.0, steer="fast", gate=True, **_T,
          why="lean the neutral loop toward the centre with the side of the fingers, then the tipped push"),
    Trial("t-lean-press", crest=("neutral",), live=True, wire_move="lean", pieces=2, tip=15.0, steer="fast", gate=True,
          **_T, why="the lean alone, stopped there"),
)
# Round 13e: in low, then up.
_L = {**_C, "mouth": 0.022, "mouth_s": 1.0}
LOWS = (
    Trial("u-low", low_in=0.048, steer="fast", gate=True, **_L,
          why="no left hand: bring the outlet in 48 mm low so the loops come in under the box's top lip, lift it, push"),
    Trial("u-low60", low_in=0.060, steer="fast", gate=True, **_L, why="the same, 60 mm low"),
)
# Round 13f: the two-handed way again with the measured faults fixed (see Trial.crest_over): lean the neutral off the
# mouth's edge, then sweep the hot and the neutral in, each pushed firmly; stopped there (pieces) to read the loops.
_F = dict(crest=("neutral", "hot", "neutral"), live=True, wire_move="seq", moves=("lean", "sweep", "sweep"),
          track=True, lean_over=0.045, steer="fast", gate=True, **_C)
FIRMS = (
    Trial("g-fold", crest_drop=0.055, crest_over=0.040, pieces=4, **_F,
          why="outlet held 55 mm low; lean the neutral, sweep the hot, sweep the neutral, each aimed 40 mm past; stop"),
    Trial("g-fold-70", crest_drop=0.075, crest_over=0.060, pieces=4, **_F,
          why="the same, held 75 mm low and aimed 60 mm past"),
    Trial("g-run", crest_drop=0.055, crest_over=0.040, **_F, why="g-fold, then the careful push"),
)
HITS = (
    Trial("h-fold", crest_drop=0.055, crest_over=0.040, along=True, pieces=4, **_F,
          why="g-fold with each sweep along the fingers' own axis through the loop's top; stop after the finger moves"),
    Trial("h-run", crest_drop=0.055, crest_over=0.040, along=True, **_F, why="h-fold, then the careful push"),
)
SEATS = (
    Trial("i-run", crest_drop=0.055, crest_over=0.040, along=True, press_dz=-0.020, **{**_F, "centre_s": 4.0, "push_s": 5.0},
          why="h-run with longer centring, a slower push, and the fingertip press 20 mm below the face's centre"),
)
SEATS = SEATS + (
    Trial("j-run", crest_drop=0.055, crest_over=0.040, along=True, press_dz=-0.020, press_dy=0.008,
          **{**_F, "centre_s": 4.0, "push_s": 5.0},
          why="i-run with the fingertip press on the ground wire's bottom corner (20 mm low, 8 mm toward +y)"),
)
TRIALS = TRIALS + TUCKS + PIECES + SWEEPS + TIPS + LOWS + FIRMS + HITS + SEATS
TRIAL_NAMES = tuple(t.name for t in TRIALS)
WIGGLE = 0.008  # up and down while pushing (m): the body has 11.7 mm to spare above and below in the box
PUMP_BACK = 0.015
PRESS_Y = -0.17  # the wall-press spot: bare wall to the right of the box, clear of the hanging outlet by 4 cm or more
PRESS_STEPS = (0.0, 0.02, 0.04, 0.06, 0.08)  # how far past the wall face the closed fingertips are aimed


def trial(name: str) -> Trial:
    for t in (*TRIALS, *ROUND12, *ROUND11, *ROUND10, *ROUND9, *ROUND8, *ROUND7, *ROUND6, *ROUND5):
        if t.name == name:
            return t
    raise ValueError(f"unknown trial {name!r}: one of {TRIAL_NAMES} (or round 5's)")


def trial_path(t: Trial) -> list:
    """Device-origin waypoints [(seconds, (x, y, z), label)] from the start pose, for the held paths."""
    x0, y, z = DEVICE_START
    aim = WALL_X + t.aim
    front = WALL_X - STAND_OFF
    if t.path == "straight":
        return [(t.push_s, (aim, y, z), "push in, aimed past the wall")]
    if t.path == "wiggle":
        n = max(2, int(round(t.push_s)))
        out = []
        for i in range(1, n + 1):
            dz = 0.0 if i == n else (WIGGLE if i % 2 else -WIGGLE)
            out.append((t.push_s / n, (x0 + (aim - x0) * i / n, y, z + dz), "wiggle it in"))
        return out
    if t.path == "pump":
        part = x0 + 0.6 * (aim - x0)
        return [(0.55 * t.push_s, (part, y, z), "push part way"), (1.0, (part - PUMP_BACK, y, z), "ease back"),
                (0.45 * t.push_s, (aim, y, z), "push it home")]
    if t.path in ("fold-down", "fold-up"):
        s = -1.0 if t.path == "fold-down" else 1.0
        return [(3.0, (WALL_X - 0.07, y, z + s * 0.035), "carry in " + ("low" if s < 0 else "high")),
                (2.0, (front, y, z + s * 0.03), "close in"), (1.5, (front, y, z), "level with the box"),
                (0.6 * t.push_s, (aim, y, z), "push it home")]
    if t.path == "tilt":  # top edge first (a positive pitch tips the top toward the wall), then level while pushing
        return [(3.0, (WALL_X - 0.05, y, z), "carry in, top edge leading", 15.0),
                (2.0, (WALL_X - 0.02, y, z), "close in, still tipped", 15.0),
                (0.6 * t.push_s, (aim, y, z), "level it and push it home")]
    if t.path == "bend":
        return device_paths()["bend"][:1]
    if t.path == "lift":
        up = z + t.lift
        return [(3.0, (WALL_X - 0.075, y, up), "carry in, raised"), (2.5, (WALL_X - 0.040, y, up), "close in, raised"),
                (1.5, (WALL_X - 0.040, y, z), "lower it level"), (0.6 * t.push_s, (aim, y, z), "push it home")]
    raise ValueError(f"trial {t.name}: path {t.path!r} has no device waypoints")


# The two-handed tuck. The YAM's closed fingers taper: +-8 mm tall at the tip, +-20 mm 40 mm back, +-34 mm (and
# +-48 mm across) from 60 mm back (yam.urdf meshes), so no hand fits under the outlet while it hangs in front of the
# box. The right hand lifts it up and to the right; the left hand, turned 30 degrees so its wrist stays on its own side
# (clear of the right arm), pokes into the box mouth with its fingertips, at mid-height and low, and backs out.
PARKS = {  # device origin while the left hand works
    "right": np.array([WALL_X - 0.10, OPEN_C[0] - 0.06, OPEN_C[1] + 0.05]),  # round 6: up and to the right
    "up": np.array([WALL_X - 0.10, OPEN_C[0], OPEN_C[1] + 0.09]),  # round 7: straight up, the belly near the centre;
    # 90 mm up puts the strap's bottom edge 30 mm above centre: the sweeps pass under it with 10 mm or more to spare,
    # and the right arm stays high, away from the left (checked against yam.urdf's collision meshes)
}
PARK = PARKS["right"]
LEFT_YAW = -30.0
POKE_LOWEST = -0.030  # the lowest a poke may go: the fingers are +-14 mm tall where they pass the wall face
POKE_IN = 0.012  # how far past the wall face the fingertips go


def _left_tcp(tip, yaw: float = LEFT_YAW) -> tuple:
    d = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.0])
    return tuple(float(v) for v in np.asarray(tip, float) - PAD_TIP * d)


def left_tuck_plan(t: Trial) -> dict:
    tcp = lambda d, dx=0.0: tuple(float(v) for v in np.asarray(d) + GRIP + (dx, 0.0, 0.0))  # noqa: E731
    start, x, y, z = DEVICE_START, WALL_X, float(OPEN_C[0]), float(OPEN_C[1])
    park = PARKS[t.park]
    right = [(2.5, tcp(start, -0.05), "open", "forward", "in front of the outlet, open"),
             (1.5, tcp(start), "open", "forward", "fingers either side of the body"),
             (1.0, tcp(start), "grip", "forward", "grip"),
             (3.0, tcp(park), "grip", "forward", "lift it out of the way")]
    lead = sum(s[0] for s in right)
    yaw = f"yaw:{LEFT_YAW:g}"
    left = [(lead, None, "open", "ready", "wait for the outlet to be lifted")]
    y0, _, h0 = t.pokes[0]
    # come in from the robot's left at the first poke's height, clear of the outlet above, then slide over to it
    left.append((2.5, _left_tcp((x - t.poke_from, y + y0 + 0.04, z + h0)), "closed", yaw,
                 "in front of the box mouth, fingers closed"))
    for ya, yb, h in t.pokes:
        out_ = (x - t.poke_from, y + ya, z + h)
        in_ = (x + POKE_IN, y + yb, z + h)
        left += [(1.0, _left_tcp(out_), "closed", yaw, "line up a poke"),
                 (3.0 if t.poke_from > 0.06 else 2.0, _left_tcp(in_), "closed", yaw, "poke the wires into the box"),
                 (0.5, _left_tcp(in_), "closed", yaw, "hold them in"),
                 (1.5, _left_tcp(out_), "closed", yaw, "back out")]
    left.append((2.0, None, "open", "ready", "back to ready"))
    work = sum(s[0] for s in left) - lead
    right.append((work, tcp(park), "grip", "forward", "hold it out of the way"))
    aim = (x + t.aim, y, z)
    right += [(2.5, tcp((x - 0.05, y, z)), "grip", "forward", "bring it back level"),
              (0.6 * t.push_s, tcp(aim), "grip", "forward", "push it home"),
              (2.5, tcp(aim), "grip", "forward", "hold"),
              (0.8, tcp(aim), "open", "forward", "let go"),
              (1.0, tcp((SEATED_X, y, z), -0.07), "open", "forward", "back straight out"),
              (2.0, tcp((SEATED_X, y, z), -0.07), "open", "forward", "watch it spring back")]
    total = sum(s[0] for s in right)
    left.append((total - sum(s[0] for s in left), None, "open", "ready", "wait"))
    return {"left": left, "right": right}


TUCK_UNDER = 0.068  # the fingertip's height below centre when it slides in under the belly (the belly is 52-58 mm
# below, the tip +-7.7 mm tall, the strap's bottom edge 60 mm below at 115 mm out)


def ground_tuck_plan(t: Trial) -> dict:
    """The left hand tucks the ground's belly into the box mouth, then the right hand pushes the outlet straight
    home. The arms take turns: the right one waits at ready until the left one is back at ready."""
    tx, lift, push_to, yaw = t.tuck
    x, y, z = WALL_X, float(OPEN_C[0]), float(OPEN_C[1])
    g = f"yaw:{yaw:g}"
    tip = lambda dx, dy, dz: _left_tcp((x - dx, y + dy, z + dz), yaw)  # noqa: E731
    left = [(2.5, tip(tx, 0.06, -TUCK_UNDER), "closed", g, "low, to the left of the ground's belly"),
            (1.5, tip(tx, 0.0, -TUCK_UNDER), "closed", g, "slide in under the belly"),
            (1.5, tip(tx, 0.0, lift), "closed", g, "lift the belly into the opening's height"),
            (2.5, tip(-push_to, 0.0, lift), "closed", g, "press it to the box mouth"),
            (0.5, tip(-push_to, 0.0, lift), "closed", g, "hold it there"),
            (1.0, tip(-push_to, 0.0, -TUCK_UNDER), "closed", g, "drop out from under it"),
            (1.5, tip(tx, 0.06, -TUCK_UNDER), "closed", g, "back out to the left"),
            (2.0, None, "open", "ready", "back to ready")]
    lead = sum(s[0] for s in left)
    tcp = lambda d, dx=0.0: tuple(float(v) for v in np.asarray(d) + GRIP + (dx, 0.0, 0.0))  # noqa: E731
    start = DEVICE_START
    aim = (x + t.aim, y, z)
    right = [(lead, None, "open", "ready", "wait for the ground to be tucked"),
             (2.5, tcp(start, -0.05), "open", "forward", "in front of the outlet, open"),
             (1.5, tcp(start), "open", "forward", "fingers either side of the body"),
             (1.0, tcp(start), "grip", "forward", "grip"),
             (t.push_s, tcp(aim), "grip", "forward", "push it home"),
             (2.5, tcp(aim), "grip", "forward", "hold"),
             (0.8, tcp(aim), "open", "forward", "let go"),
             (1.0, tcp((SEATED_X, y, z), -0.07), "open", "forward", "back straight out"),
             (2.0, tcp((SEATED_X, y, z), -0.07), "open", "forward", "watch it spring back")]
    total = sum(s[0] for s in right)
    left.append((total - lead, None, "open", "ready", "wait"))
    return {"left": left, "right": right}


GRIP_FIRM = -0.009  # round 10: fingers told to close to 9 mm from centre, 8 mm inside the body's sides: 5000 N/m x
# about 7 mm, 32-39 N a side, still under the 50 N cap (round 9's -12 mm slipped at 35-43 N of push)
STANDOFF = 0.045  # aim-thumb pauses with the strap this far off the wall, before the stall point of rounds 5-9
AIM_GAIN, AIM_Y, AIM_Z = 0.6, 0.012, (-0.010, 0.050)  # the probe's steering of the held outlet (see probe.py);
# round 11 lifts the cap from +25 to +50 mm: under the push the hand ran 22 mm below its target
# Round 12's "fast" steering. Round 11 (all ten technique trials flush, none centred): pushing to the mouth, the outlet
# dropped 11 mm in 1.5 s while the 0.5 s steering wound up to its +50 mm cap; it ended on the bottom boss, 3 mm low,
# 2-5 mm to the left, and turned 8-10 degrees in the wall plane (the side ground wire pushes its low corner). So: update
# every 2 steps (0.1 s) at a third of the gain per update (a faster integral, 2 per second), a +80 mm lift cap (the
# hand sagged up to 52 mm below its target), and the roll steered too, by turning the wrist about the push axis.
# Round 13i (h-run: every wire inside, 7 mm out, but 3 mm to one side and 2.5 mm low with the sideways steering
# pinned at its 15 mm cap all through the push): the right hand is about 0.2 N per mm sideways and up, so 15 mm was
# 3 N. Caps raised to 40 mm sideways and 110 mm up; the gate may hold the push back for 8 s, not 4.
FAST_EVERY, FAST_GAIN, FAST_Y, FAST_Z = 2, 0.2, 0.040, (-0.015, 0.110)
TWIST_GAIN, TWIST_CAP = 0.2, 15.0  # per update, and the most the wrist is turned (degrees)
GATE_MM, GATE_DEG = 1.5, 2.0  # gated push: x advances only with the outlet this close to centre and this square
GATE_SPEED, GATE_MAX_S = 0.03, 8.0  # then catches up with the plan at up to 30 mm/s; held back 8 s at most in all
AIM_LABELS = ("carry to the stand-off", "settle", "push to the mouth", "centre it at the mouth", "push it home",
              "grip hold")  # steps the steering runs in (not the dip or the crest press: it would pull the outlet back up)
GATED_LABELS = ("push to the mouth", "push it home")
FACE_LABELS = ("close the fingers", "fingertips to the face", "press the face", "hold")  # follow the outlet's y, z


def aim_thumb_plan(t: Trial) -> dict:
    """Grip firmly, carry to a stand-off and settle, push past the wall and hold (the probe steers the hand so the
    outlet stays centred on the opening); let go, back off, close the fingers, and press the face's centre with the
    fingertips (the probe aims them at the outlet's measured y, z), hold, back out."""
    x, y, z = WALL_X, float(OPEN_C[0]), float(OPEN_C[1])
    tcp = lambda d, dx=0.0: tuple(float(v) for v in np.asarray(d) + GRIP + (dx, 0.0, 0.0))  # noqa: E731
    start = DEVICE_START
    # A robustness repeat grips the body off its usual spot (grip_off) but plans the rest as if it hadn't: the outlet
    # then rides that far off-centre and only the steering can bring it back.
    at_grip = tuple(float(v) for v in np.asarray(tcp(start)) + (0.0, *t.grip_off))
    touch = face_x(WALL_X) - PAD_TIP  # TCP x with closed fingertips on the face of a seated outlet
    f = lambda dx: (float(touch + dx), y + t.press_dy, z + t.press_dz)  # noqa: E731
    stand = (x - STANDOFF, y, z)
    right = [(2.5, tuple(float(v) for v in np.asarray(at_grip) + (-0.05, 0.0, 0.0)), "open", "forward",
              "in front of the outlet, open"),
             (1.5, at_grip, "open", "forward", "fingers either side of the body"),
             (1.0, at_grip, "grip9", "forward", "grip"),
             (5.0, tcp(stand), "grip9", "forward", "carry to the stand-off"),
             (1.5, tcp(stand), "grip9", "forward", "settle")]
    if t.low_in > 0:  # round 13e: in low, under the lip, then up (not steered: these labels are not AIM_LABELS)
        low_at = lambda out_: tcp((x - out_, y, z - t.low_in))  # noqa: E731
        right = right[:3] + [(4.0, low_at(0.075), "grip9", "forward", "carry in low"),
                             (1.0, low_at(0.075), "grip9", "forward", "settle low"),
                             (3.0, low_at(t.low_out), "grip9", "forward", "push in, still low"),
                             (1.0, low_at(t.low_out), "grip9", "forward", "hold it low at the wall"),
                             (t.rise_s, tcp((x - t.low_out, y, z)), "grip9", "forward", "lift it to the opening")]
    if t.dip > 0:
        right += [(2.0, tcp((stand[0], y, z - t.dip)), "grip9", "forward", "dip it"),
                  (2.0, tcp(stand), "grip9", "forward", "bring it back up"),
                  (1.0, tcp(stand), "grip9", "forward", "settle")]
    left = []
    # Round 13d: with Trial.tip the outlet comes in top edge first (the hand pitched about world y, the device origin
    # where the level plan has it) and is levelled by the last push.
    tipped = (lambda d: tuple(float(v) for v in np.asarray(d) + roty(t.tip) @ GRIP)) if t.tip else tcp
    gt = f"pitch:{t.tip:g}" if t.tip else "forward"
    if t.crest:  # held CREST_DROP low while the left hand works, so its finger passes over the strap's top ear
        low = tcp((stand[0], y, z - t.crest_drop))
        right.append((2.0, low, "grip9", "forward", "lower it for the left hand"))
        lead = sum(s_[0] for s_ in right)
        left = [(lead, None, "open", "ready", "wait for the outlet at the stand-off")] + crest_plan(t.crest)
        right += [(sum(s_[0] for s_ in left) - lead, low, "grip9", "forward", "hold it low for the left hand"),
                  (2.0, tipped(stand), "grip9", gt, "bring it back up"),
                  (1.0, tipped(stand), "grip9", gt, "settle")]
    right += [*([(t.mouth_s, tipped((x - t.mouth, y, z)), "grip9", gt, "push to the mouth"),
                 (t.centre_s, tipped((x - t.mouth, y, z)), "grip9", gt, "centre it at the mouth")]
                if t.mouth > 0 else []),
             (t.push_s, tcp((x + t.aim, y, z)), "grip9", "forward", "push it home"),
             (2.5, tcp((x + t.aim, y, z)), "grip9", "forward", "grip hold"),
             (0.8, tcp((x + t.aim, y, z)), "open", "forward", "let go"),
             (1.0, f(-0.06), "open", "forward", "back off"),
             (0.7, f(-0.06), "closed", "forward", "close the fingers"),
             (1.5, f(-0.005), "closed", "forward", "fingertips to the face"),
             (3.0 if t.press <= 0.05 else 4.0, f(t.press), "closed", "forward", "press the face"),
             (2.5, f(t.press), "closed", "forward", "hold"),
             (1.0, f(-0.07), "closed", "forward", "back straight out"),
             (3.5, f(-0.07), "closed", "forward", "watch it spring back")]
    total = sum(s_[0] for s_ in right)
    rest = total - sum(s_[0] for s_ in left)
    plan = {"left": left + ([(rest, None, "open", "ready", "wait")] if rest > 1e-9 else []), "right": right}
    end = piece_end(t, plan)
    return plan if end is None else {arm: cut_steps(steps, end) for arm, steps in plan.items()}


def piece_end(t: Trial, plan: dict) -> float | None:
    """When piece ``t.pieces`` of the two-handed run ends (s into ``plan``, the whole run), or None for the whole run."""
    if not t.pieces or not t.crest:
        return None

    def end_of(label: str) -> float:
        acc = 0.0
        for s_ in plan["right"]:
            acc += s_[0]
            if s_[4] == label:
                return acc
        raise ValueError(f"no step {label!r} in the right hand's plan")

    lead = end_of("lower it for the left hand")
    pressed = t.pieces - 1
    if pressed <= len(t.crest):  # each wire: over it, onto it, press it in, hold, lift away
        return lead + sum((2.5 if i == 0 else 1.5) + sum(CREST_TIMES) for i in range(pressed))
    return end_of("grip hold") if pressed == len(t.crest) + 1 else None


def cut_steps(steps: list, t_end: float, hold: float = 2.0) -> list:
    """``steps`` up to ``t_end``, then ``hold`` seconds still where it stopped. Cut only where the arm is waiting or
    holding: a moving step cut short would be done faster, not left half done."""
    out, acc = [], 0.0
    for s_ in steps:
        if acc + s_[0] <= t_end + 1e-9:
            out.append(s_)
            acc += s_[0]
            continue
        if t_end - acc > 1e-9:
            out.append((t_end - acc, *s_[1:]))
        break
    last = out[-1]
    return out + [(hold, last[1], last[2], last[3], "hold still: the end of this piece")]


# Round 12's left-hand crest press. Round 11's side views: the hot and neutral wires leave the top of the body, rise to a
# crest 11 mm in front of the wall and 57 mm above centre (12 mm above the box's top lip) and fall into the box; the
# push folds that crest onto the lip, where it pins the outlet low and ends 2-3 mm proud (the "showing" hot wire). An
# electrician tucks it first. With the outlet held at the stand-off, the left hand, fingers closed and pointing forward
# and 30 degrees down (a straight-down press is out of the left arm's reach this high), comes over the crest from in
# front, presses it against the wall face and slides it down past the lip, then lifts straight up and away. The copper
# keeps the bend. One press per wire (CRESTS: tip positions from the wall face and the opening's centre).
CREST_TILT = 60.0  # degrees the forward grip is tipped down from straight down: the fingertips point 30 degrees low
CRESTS = {  # name: (y of the wire, tip x in front of the wall while pressing, yaw of the hand in degrees). The yaw
    # swings the finger's shaft out past the strap's top ear (60 mm up, 22 mm either side of centre at the stand-off),
    # which a square press would rest its shaft on (yam.urdf meshes; scratchpad check crestpath.py).
    "hot": (-0.017, 0.006, 0.0),
    "neutral": (0.018, 0.006, 0.0),
    "ground": (0.020, 0.006, 0.0),
}
CREST_DROP = 0.028  # the right hand holds the outlet this much lower while the left hand presses (9 mm clear)
CREST_HIGH, CREST_LOW = 0.085, 0.034  # the tip's height above centre: over the crest, and pressed down past the lip (its
# underside about 8 mm lower; the body's top back edge is 34 mm up, 18 mm out at the stand-off)


def crest_rotation(yaw_deg: float = 0.0) -> np.ndarray:
    """The left hand's rotation for the press: a straight-down grip turned ``yaw_deg`` about the vertical (yam_arena's
    grip_rotation), then tipped toward the wall by CREST_TILT about world y."""
    a, b = math.radians(-CREST_TILT), math.radians(yaw_deg)
    tip = np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])
    turn = np.array([[math.cos(b), -math.sin(b), 0.0], [math.sin(b), math.cos(b), 0.0], [0.0, 0.0, 1.0]])
    from_down = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # yam_arena's DOWN_GRIP (test_task)
    return tip @ turn @ from_down


def crest_tcp(tip, yaw_deg: float = 0.0) -> tuple:
    """The left TCP that puts the closed pad tips at ``tip`` (world) with the press rotation."""
    approach = crest_rotation(yaw_deg) @ np.array([0.0, 0.0, 1.0])
    return tuple(float(v) for v in np.asarray(tip, float) - PAD_TIP * approach)


def crest_plan(names: tuple) -> list:
    """The left hand's steps for the crest presses, from ready back to ready."""
    x, y, z = WALL_X, float(OPEN_C[0]), float(OPEN_C[1])
    out = []
    for n in names:
        wy, tx, yaw = CRESTS[n]
        g = f"crest:{yaw:g}"
        above = crest_tcp((x - tx - 0.025, y + wy, z + CREST_HIGH + 0.010), yaw)
        touch = crest_tcp((x - tx, y + wy, z + CREST_HIGH - 0.010), yaw)
        down = crest_tcp((x - tx, y + wy, z + CREST_LOW), yaw)
        out += [(2.5 if not out else 1.5, above, "closed", g, f"over the {n} wire's crest"),
                (1.0, touch, "closed", g, f"onto the {n} crest, against the wall"),
                (2.0, down, "closed", g, f"slide the {n} crest down past the lip"),
                (0.5, down, "closed", g, "hold it there"),
                (1.5, above, "closed", g, "lift straight up and away")]
    return out + [(2.0, None, "open", "ready", "back to ready")]


CREST_TIMES = (1.0, 2.0, 0.5, 1.5)  # onto the loop, press it in, hold, lift away (crest_plan's own times)


def loop_tops(polylines: dict, names: tuple) -> dict:
    """For each named wire, the highest point of the part still in front of the wall (world), or None if none is."""
    tops = {}
    for n in names:
        p = np.asarray(polylines[n], float)
        front = p[p[:, 0] < WALL_X - SHOWING]
        tops[n] = None if len(front) == 0 else front[int(np.argmax(front[:, 2]))]
    return tops


ALONG_BACK = 0.015  # an "along" sweep starts with the fingertips this far short of the loop's top, on their own axis
CROSS_LOW = 0.015  # the sweeping fingertips' centre crosses the wall face at least this far under the box's top lip


def crest_plan_at(tops: dict, names: tuple, crest_in: float = 0.012, crest_low: float = 0.016,
                  back: float = 0.008, up: float = 0.010, over: float = 0.0, along: bool = False) -> list:
    """crest_plan, each press aimed at its wire's loop where it is (``tops[name]``, world; from loop_tops) and done as
    a sweep. Round 13c: the first live presses came down over the loop's top and slid on to the wall above the box's
    lip without touching the wire (the path crossed the wall face at the lip's height). So the closed fingertips now
    start on the outlet's side of the top (``back`` further from the wall, ``up`` above it) and sweep in a straight line
    toward the wall and down, to end ``crest_in`` inside the wall face and ``crest_low`` above the opening's centre:
    their end face meets the loop's top and carries it under the lip, and whatever slips beneath them is held down by
    their underside. Then hold, and lift away. A wire with nothing in front of the wall gets the same moves in the air
    above the box, so the timing never changes. Same step times as crest_plan."""
    x, y, z = WALL_X, float(OPEN_C[0]), float(OPEN_C[1])
    side = 0.007 + 0.52 * crest_in  # half the closed fingers' width where they cross the wall face, plus 2 mm
    out = []
    for n in names:
        yaw = CRESTS.get(n, (0.0, 0.0, 0.0))[2]
        g = f"crest:{yaw:g}"
        top = tops.get(n)
        if top is None:
            start = np.array([x - 0.030, y + CRESTS.get(n, (0.0,))[0], z + 0.110])
            end_tip = tuple(float(v) for v in start)
        else:
            top = np.asarray(top, float)
            start = top + (-back, 0.0, up)
            inside_y = float(np.clip(top[1], y - (BOX_W / 2 - side), y + (BOX_W / 2 - side)))
            axis = crest_rotation(yaw) @ np.array([0.0, 0.0, 1.0])  # the way the fingers point: at the wall, 30 deg down
            cross = float(top[2] + (x - top[0]) * axis[2] / axis[0])  # where a line along them through the top meets the wall
            if along and cross - z <= BOX_H / 2 - CROSS_LOW:
                start = top - ALONG_BACK * axis
                end = top + ((x + crest_in - float(top[0])) / axis[0] + over) * axis
                end_tip = (float(end[0]), inside_y, float(end[2]))
                above = crest_tcp((start[0] - 0.015, start[1], start[2] + 0.030), yaw)
                out += [(2.5 if not out else 1.5, above, "closed", g, f"over the {n} wire's loop"),
                        (CREST_TIMES[0], crest_tcp(start, yaw), "closed", g, f"behind the top of the {n} loop"),
                        (CREST_TIMES[1], crest_tcp(end_tip, yaw), "closed", g, f"sweep the {n} loop down and into the box"),
                        (CREST_TIMES[2], crest_tcp(end_tip, yaw), "closed", g, "hold it there"),
                        (CREST_TIMES[3], above, "closed", g, "lift up and away")]
                continue
            # A high loop makes the line steep: end lower if need be, so the tip's centre still crosses the wall face
            # CROSS_LOW or more under the lip (its top edge is 8 mm above its centre).
            run, rise = x - float(start[0]) + crest_in, float(start[2]) - z
            cross_max = BOX_H / 2 - CROSS_LOW
            low = min(crest_low, (cross_max - crest_in * rise / run) / (1 - crest_in / run))
            end_tip = (x + crest_in, inside_y, z + low)
            if over > 0:  # aimed past the goal along the same line: the arm is a spring, so this is the push's force
                line = np.asarray(end_tip, float) - start
                end_tip = tuple(float(v) for v in np.asarray(end_tip, float) + over * line / np.linalg.norm(line))
        above = crest_tcp((start[0] - 0.015, start[1], start[2] + 0.030), yaw)
        touch = crest_tcp(start, yaw)
        down = crest_tcp(end_tip, yaw)
        out += [(2.5 if not out else 1.5, above, "closed", g, f"over the {n} wire's loop"),
                (CREST_TIMES[0], touch, "closed", g, f"behind the top of the {n} loop"),
                (CREST_TIMES[1], down, "closed", g, f"sweep the {n} loop down and into the box"),
                (CREST_TIMES[2], down, "closed", g, "hold it there"),
                (CREST_TIMES[3], above, "closed", g, "lift up and away")]
    return out + [(2.0, None, "open", "ready", "back to ready")]


LEAN_HALF = 0.011  # half the closed fingers' width 10 mm back from their tips, plus the wire's radius (m)


LEAN_TO = 0.008  # a leaned loop's top ends this far from the opening's centre line (m)


def lean_plan_at(tops: dict, names: tuple, to_y: float = 0.004, beside: float = 0.018, tip_out: float = 0.013,
                 drop: float = 0.008, over: float = 0.0) -> list:
    """The left hand leans each named loop toward the opening's centre with the SIDE of its closed fingers (same hand
    rotation and step times as crest_plan_at). Round 13d: a loop is an arch in compression, and the fingertips' end
    face slid off the side of it (rounds 13b, 13c); the fingers' side is a broad face the wire cannot get round. The
    fingertips come down beside the loop's top on its outer side (``beside`` further from the centre, ``tip_out`` in
    front of the wall, ``drop`` below the top) and move straight toward the centre until the wire stands ``to_y`` from
    it, behind the strap. A loop already that near the centre is left alone."""
    x, y, z = WALL_X, float(OPEN_C[0]), float(OPEN_C[1])
    out = []
    for n in names:
        yaw = CRESTS.get(n, (0.0, 0.0, 0.0))[2]
        g = f"crest:{yaw:g}"
        top = tops.get(n)
        if top is None:
            start = np.array([x - 0.030, y + 0.045, z + 0.110])
            end = start.copy()
        else:
            top = np.asarray(top, float)
            side = 1.0 if top[1] >= y else -1.0  # which side of the centre the loop stands on
            start = np.array([x - tip_out, top[1] + side * beside, top[2] - drop])
            end = start.copy()
            end[1] = y + side * (to_y + LEAN_HALF - over)  # ``over``: aimed past it, for force (the probe stops it)
            if side * (top[1] - y) <= to_y:  # already standing that near the centre
                end = start.copy()
        above = crest_tcp((start[0] - 0.015, start[1], start[2] + 0.030), yaw)
        out += [(2.5 if not out else 1.5, above, "closed", g, f"over the {n} wire's loop"),
                (CREST_TIMES[0], crest_tcp(start, yaw), "closed", g, f"down beside the top of the {n} loop"),
                (CREST_TIMES[1], crest_tcp(end, yaw), "closed", g, f"lean the {n} loop toward the centre"),
                (CREST_TIMES[2], crest_tcp(end, yaw), "closed", g, "hold it there"),
                (CREST_TIMES[3], above, "closed", g, "lift up and away")]
    return out + [(2.0, None, "open", "ready", "back to ready")]


def live_plan(t: "Trial", aims) -> list:
    """The left hand's wire moves for trial ``t``. ``aims``: each move's loop top (loop_tops), one per entry of
    ``t.crest`` in order (a list), or a dict by wire name."""
    if isinstance(aims, dict):
        aims = [aims.get(n) for n in t.crest]
    kinds = t.moves if t.wire_move == "seq" else (t.wire_move,) * len(t.crest)
    out = []
    for k, (n, kind) in enumerate(zip(t.crest, kinds)):
        one = {n: aims[k]}
        if kind == "lean":
            steps = lean_plan_at(one, (n,), to_y=LEAN_TO, over=t.lean_over)
        else:
            steps = crest_plan_at(one, (n,), t.crest_in, t.crest_low, t.crest_back, t.crest_up, t.crest_over, t.along)
        steps = steps[:-1]  # its own "back to ready"
        if out:
            steps[0] = (1.5, *steps[0][1:])
        out += steps
    return out + [(2.0, None, "open", "ready", "back to ready")]


def crest_start(k: int) -> float:
    """Seconds after the left hand's first move that the ``k``-th wire's press begins (0: the first wire)."""
    return sum((2.5 if i == 0 else 1.5) + sum(CREST_TIMES) for i in range(k))


def roty(deg: float) -> np.ndarray:
    """Rotation about world y by ``deg``: negative tips a held outlet's bottom toward the wall (+x)."""
    a = math.radians(deg)
    return np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])


def trial_plan(t: Trial) -> dict:
    """{"left": [...], "right": [...]} in probe_plan's step format. Trials end with the hand backed off the outlet,
    not at ready: the next trial's reset puts the arms back."""
    if t.path == "left-tuck":
        return left_tuck_plan(t)
    if t.path == "ground-tuck":
        return ground_tuck_plan(t)
    if t.path == "aim-thumb":
        return aim_thumb_plan(t)
    if t.path == "wall-press":
        tip = WALL_X - PAD_TIP  # TCP x with the closed fingertips on the wall face
        z = float(OPEN_C[1])
        right = [(2.5, (tip - 0.04, PRESS_Y, z), "closed", "forward", "in front of the bare wall"),
                 (1.5, (tip - 0.002, PRESS_Y, z), "closed", "forward", "touch the wall")]
        for d in PRESS_STEPS[1:]:
            right += [(1.0, (tip + d, PRESS_Y, z), "closed", "forward", f"press {1000 * d:.0f} mm"),
                      (1.0, (tip + d, PRESS_Y, z), "closed", "forward", f"hold {1000 * d:.0f} mm")]
        right.append((1.0, (tip - 0.04, PRESS_Y, z), "closed", "forward", "back off"))
    else:
        def tcp(d, dx=0.0, pitch=0.0):  # the hand holding the device origin at d, tipped ``pitch`` degrees
            return tuple(float(v) for v in np.asarray(d) + roty(pitch) @ GRIP + (dx, 0.0, 0.0))

        start = DEVICE_START
        right = [(2.5, tcp(start, -0.05), "open", "forward", "in front of the outlet, open"),
                 (1.5, tcp(start), "open", "forward", "fingers either side of the body"),
                 (1.0, tcp(start), "grip", "forward", "grip")]
        last = start
        for seconds, d, label, *pitch in trial_path(t):
            pitch = pitch[0] if pitch else 0.0
            right.append((seconds, tcp(d, pitch=pitch), "grip", f"pitch:{pitch:g}" if pitch else "forward", label))
            last = d
        seated = (min(float(last[0]), SEATED_X), float(last[1]), float(last[2]))
        right += [(2.5 if t.path != "bend" else 1.0, tcp(last), "grip", "forward", "hold"),
                  (0.8, tcp(last), "open", "forward", "let go"),
                  (1.0, tcp(seated, -0.07), "open", "forward", "back straight out"),
                  (3.5, tcp(seated, -0.07), "open", "forward", "watch it spring back")]
    total = sum(s[0] for s in right)
    return {"left": [(total, None, "open", "ready", "wait")], "right": right}
