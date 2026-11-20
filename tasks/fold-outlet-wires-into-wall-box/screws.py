"""The outlet's two mounting screws and the ratcheting screwdriver: sizes, shapes, and the rule that drives a screw.

Pure numpy, so the tests run without the simulator. The simulator cannot cut a thread, so the thread is this rule
(the task file applies it before every physics step):

- Each screw is a 6-32 x 1 in device mounting screw, captive in a slot at one end of the outlet's strap. Until it is
  started it rides with the strap. Its tip cannot pass the box's boss (or the wall), so as the outlet comes up to the
  wall the tip stops there and the head stands proud of the strap.
- The screwdriver's bit engages a screw when its tip is on the head, within ENGAGE_R of the screw's axis and within
  ENGAGE_DEG of its direction. While engaged, a spring holds the tip on the axis, as the recess does.
- The screwdriver is a ratchet set to drive: only forward turns (clockwise seen from the handle) turn the screw.
- A forward turn starts the thread once the screw's slot is within CATCH_R of the boss hole and its tip is at the
  boss. From then the screw is in the box: it stays on the boss hole's axis and each turn takes it TRAVEL / TURNS
  deeper. Its head comes down on the strap and draws that end of the strap onto the wall by contact, nothing else.
- TURNS forward turns take a started screw home (a real 6-32 thread takes about 30 for the same travel).

The strap's two slots and the bosses' two holes are drawn where the rule has them (slot_marks, boss_marks). They are
looks only: no hole is cut in a collider, and the rule above is the only thing that places a screw.
"""
from __future__ import annotations

import math

import numpy as np

import outlet_logic as L

# -- the screws: 6-32 x 1 in combination-head device mounting screws, zinc-plated steel --------------------------------
SCREW_LEN = 1.0 * L.INCH  # under the head
SCREW_D = 0.0035  # 6-32 major diameter
HEAD_D = 0.0069  # pan head across
HEAD_H = 0.0023  # head height
SCREW_MASS = 0.0017  # kg (a steel 6-32 x 1 in screw)
REAL_PITCH = L.INCH / 32  # 6-32: 32 threads per inch
NAMES = ("screw_top", "screw_bottom")  # in outlet_logic.SCREW_TABS order

# -- the thread rule -----------------------------------------------------------------------------------------------------
TURNS = 2.0  # forward turns that take a started screw home
HOME_CLEAR = 0.0003  # home: the head's underside stands the strap's thickness plus this off the boss
S_HOME = L.STRAP_T + HOME_CLEAR
TRAVEL = SCREW_LEN - S_HOME  # how far a started screw goes in
SLOP = 0.0005  # a captive screw's head rides this far off the strap, so it never presses on it
CATCH_R = L.TAB_TOL  # the strap's slot within 3 mm of the boss hole: the screw finds its hole
CATCH_DEG = L.UPRIGHT_DEG  # and the outlet square to the wall within this
ENGAGE_R, RELEASE_R = 0.0020, 0.0032  # the bit's tip from the screw's axis: in the recess / camming out (m)
ENGAGE_BACK, ENGAGE_IN = 0.0020, 0.0030  # the tip this far short of the head's top / this far past it (m)
RELEASE_BACK = 0.0040
ENGAGE_DEG, RELEASE_DEG = 15.0, 22.0  # the screwdriver off the screw's direction
K_RECESS, C_RECESS, F_RECESS = 1500.0, 4.0, 3.0  # the recess holding the tip on the axis: N/m, N s/m, at most N
K_HEAD, C_HEAD, F_HEAD = 6000.0, 6.0, 20.0  # the head under the bit (the two don't collide): N/m, N s/m, at most N
HEAD_REACH = HEAD_H + 0.004  # the head stops a tip this far past its top at most (deeper: it came from the side)
# A started screw's shank runs through its slot in the strap and so keeps that slot near the hole. No shank is
# simulated: the rule pushes the slot toward its hole with a spring, at most F_PIN. Tuned on the task's own runs: with
# this spring both screws drive home with their slots 1 to 2 mm off their holes. Each shank's push is sent to the
# simulator on its own; where the simulator keeps only the last push of a step, the lower screw's shank is the one
# that bears once both have started. (Softer or damped springs, and both pushes summed, were tried on 20 Nov: the
# outlet pivoted on the upper screw and took the lower slot off its hole.) C_PIN and KI_PIN are off.
PIN_PLAY = 0.0  # m: how far a slot may sit off its started screw's axis before the shank bears on it
K_PIN, C_PIN, KI_PIN, F_PIN = 5000.0, 0.0, 0.0, 30.0  # N/m, N s/m, N/(m s) of build-up, at most N
PHYSICS_DT = 1.0 / 240.0  # s: the rule runs once per physics step
HOME_TOL = 0.0010  # a screw counts as driven home with its head within this of home

# -- the ratcheting screwdriver: #2 Phillips, a full-size handle with two flats for the pads --------------------------
HANDLE_L = 0.110  # handle length
HANDLE_R = 0.0165  # handle radius at its thickest
HANDLE_FLAT = 0.026  # across the two flats (they face left and right when it lies on its rest)
SHAFT_L, SHAFT_D = 0.062, 0.0060  # the bare shaft past the handle
TIP_L, TIP_D = 0.008, 0.0020  # the Phillips point: a taper down to this across
DRIVER_L = HANDLE_L + SHAFT_L + TIP_L  # 180 mm overall
DRIVER_MASS = 0.150  # kg
DRIVER_TIP = np.array([HANDLE_L / 2 + SHAFT_L + TIP_L, 0.0, 0.0])  # the tip in the screwdriver's frame (origin: mid-handle)

# -- its rest: two posts on the table, on the robot's left ----------------------------------------------------------------
REST_AXIS_Z = 0.120  # the screwdriver's axis above the table
REST_XY = (0.400, 0.300)  # the handle's middle (x, y): ahead of the left arm, the screwdriver pointing forward
REST_POST = 0.012  # each post is this thick along the screwdriver
REST_CHEEK = 0.004  # the front post's cheeks, which keep the flats upright


def _unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def rot_x(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def matrix_to_quat_xyzw(m) -> np.ndarray:
    m = np.asarray(m, float)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    return q / np.linalg.norm(q)


# -- shapes (labkit meshes: points, face counts, face indices), all with the long axis along x --------------------------
def lathe(profile, n: int = 20, flat: float | None = None) -> tuple:
    """A solid of revolution about x from ``profile``, a list of (x, radius) from one end to the other (radius 0 at an
    end closes it to a point). ``flat`` cuts two flats this far apart (facing +y and -y). Convex if the profile is."""
    pts, rings = [], []
    for x, r in profile:
        if r <= 1e-9:
            rings.append([len(pts)])
            pts.append((x, 0.0, 0.0))
            continue
        ring = []
        for j in range(n):
            a = 2 * math.pi * j / n
            y, z = r * math.cos(a), r * math.sin(a)
            if flat is not None:
                y = max(-flat / 2, min(flat / 2, y))
            ring.append(len(pts))
            pts.append((x, y, z))
        rings.append(ring)
    faces = []
    if len(rings[0]) > 1:
        faces.append(tuple(rings[0]))  # the cap at the first end, facing -x
    for a, b in zip(rings[:-1], rings[1:], strict=True):
        for j in range(n):
            k = (j + 1) % n
            if len(a) == 1 and len(b) == 1:
                continue
            if len(a) == 1:
                faces.append((a[0], b[k], b[j]))
            elif len(b) == 1:
                faces.append((a[j], a[k], b[0]))
            else:
                faces.append((a[j], a[k], b[k], b[j]))
    if len(rings[-1]) > 1:
        faces.append(tuple(reversed(rings[-1])))  # the cap at the last end, facing +x
    return (tuple(tuple(round(float(v), 7) for v in p) for p in pts), tuple(len(f) for f in faces),
            tuple(int(i) for f in faces for i in f))


def box_mesh(size, at=(0.0, 0.0, 0.0)) -> tuple:
    hx, hy, hz = (float(v) / 2 for v in size)
    cx, cy, cz = at
    pts = [(cx + sx * hx, cy + sy * hy, cz + sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return (tuple(tuple(round(float(v), 7) for v in p) for p in pts), tuple(len(f) for f in faces),
            tuple(int(i) for f in faces for i in f))


def screw_meshes() -> dict:
    """A screw's parts in its own frame: the origin is the middle of the head, +x runs down the shank to the tip
    (the head's top is at -HEAD_H / 2, its underside at +HEAD_H / 2). The head is a pan head; the cross and slot of
    its combination recess are a dark mark on top."""
    r, h = HEAD_D / 2, HEAD_H
    u = h / 2  # the underside
    head = lathe([(u - h, 0.62 * r), (u - 0.8 * h, 0.86 * r), (u - 0.45 * h, r), (u, r)], n=20)
    core = SCREW_D / 2
    shank = lathe([(u, core), (u + SCREW_LEN - 0.0015, core), (u + SCREW_LEN, 0.55 * core)], n=12)
    return {
        "head": head, "shank": shank,
        "slot": box_mesh((0.0002, 0.90 * HEAD_D * 0.62, 0.0010), (u - h - 0.0001, 0.0, 0.0)),
        "cross": box_mesh((0.0002, 0.0010, 0.55 * HEAD_D * 0.62), (u - h - 0.0001, 0.0, 0.0)),
    }


SCREW_CORE = (HEAD_H, 0.0040, 0.0040)  # the screw body's own box, inside the head
DRIVER_CORE = (0.066, 0.0250, 0.0100)  # the screwdriver body's own box, inside the handle

# -- the holes the screws go through, as drawn: looks only, nothing here collides or is measured -----------------------
SLOT_W, SLOT_H = 0.0100, 0.0045  # the strap's screw slot: long across the strap, so the outlet can be squared up
BOSS_HOLE_D = 0.0040  # a boss's screw hole as drawn (the 3.5 mm shank goes into it)
BOSS_LEAD_D = 0.0058  # the lead-in round the hole's mouth (the boss's face is 6.7 mm tall)
MARK = 0.0002  # a drawn hole stands this far off the surface it is on (less than a head ever comes to the strap)


def prism(outline, x0: float, x1: float) -> tuple:
    """``outline``, a list of (y, z) going round counter-clockwise seen from +x, pulled out along x from ``x0`` to
    ``x1``. Convex if the outline is."""
    n = len(outline)
    pts = [(x, y, z) for x in (x0, x1) for y, z in outline]
    faces = [tuple(range(n - 1, -1, -1)), tuple(range(n, 2 * n))]
    faces += [(j, (j + 1) % n, n + (j + 1) % n, n + j) for j in range(n)]
    return (tuple(tuple(round(float(v), 7) for v in p) for p in pts), tuple(len(f) for f in faces),
            tuple(int(i) for f in faces for i in f))


def slot_mesh(n: int = 8) -> tuple:
    """A strap screw slot as drawn: a dark plug the slot's shape (round ends, long across the strap), through the
    strap and MARK proud of both its faces. Its own frame: the origin is the slot's middle, half-way through the
    strap; x is through the strap, y along the slot. Place it at a slot's point (outlet_logic.SCREW_TABS) less half
    the strap's thickness."""
    r = SLOT_H / 2
    c = SLOT_W / 2 - r  # the round ends' centres
    outline = [(side * c + r * math.cos(a), r * math.sin(a))
               for side, a0 in ((1, -math.pi / 2), (-1, math.pi / 2))
               for a in (a0 + math.pi * j / n for j in range(n + 1))]
    half = L.STRAP_T / 2 + MARK
    return prism(outline, -half, half)


def slot_marks() -> list:
    """The two slots as (name, mesh, centre in the device frame), in outlet_logic.SCREW_TABS order."""
    mid = np.array([-L.STRAP_T / 2, 0.0, 0.0])
    return [(f"slot_{end}", slot_mesh(), tuple(float(v) for v in tab + mid))
            for end, tab in zip(("top", "bottom"), L.SCREW_TABS, strict=True)]


def boss_hole_meshes() -> dict:
    """A boss's screw hole as drawn, on the boss's face (x = 0, the box behind at +x): the hole, a dark disc MARK
    proud, and the lead-in round its mouth, a wider disc half as proud. Both run a little way into the boss."""
    return {"hole": lathe([(-MARK, BOSS_HOLE_D / 2), (0.0005, BOSS_HOLE_D / 2)], n=20),
            "lead": lathe([(-MARK / 2, BOSS_LEAD_D / 2), (0.0005, BOSS_LEAD_D / 2)], n=24)}


def boss_marks() -> list:
    """The two bosses' holes as (name, mesh, centre in the world), at outlet_logic.boss_holes(): lead-in first."""
    m = boss_hole_meshes()
    return [(f"{part}_{end}", m[part], tuple(float(v) for v in hole))
            for end, hole in zip(("top", "bottom"), L.boss_holes(), strict=True) for part in ("lead", "hole")]


def grip_profile() -> list:
    a, r = HANDLE_L / 2, HANDLE_R
    return [(-a, 0.70 * r), (-a + 0.006, 0.93 * r), (-0.25 * a, r), (0.45 * a, 0.92 * r), (a - 0.012, 0.74 * r)]


def grip_radius(x: float) -> float:
    """The handle's radius at ``x`` along it (its own frame)."""
    xs, rs = zip(*grip_profile(), strict=True)
    return float(np.interp(x, xs, rs))


def driver_meshes() -> dict:
    """The screwdriver's parts in its own frame: the origin is the middle of the handle, +x runs to the tip."""
    a, r = HANDLE_L / 2, HANDLE_R
    grip = lathe(grip_profile(), n=24, flat=HANDLE_FLAT)
    collar = lathe([(a - 0.012, 0.66 * r), (a, 0.58 * r)], n=20)  # the ratchet's collar, at the front of the handle
    cap = lathe([(-a - 0.004, 0.50 * r), (-a, 0.70 * r)], n=20)
    s0, s1 = a, a + SHAFT_L
    shaft = lathe([(s0, SHAFT_D / 2), (s1, SHAFT_D / 2)], n=12)
    tip = lathe([(s1, SHAFT_D / 2), (s1 + 0.4 * TIP_L, 0.75 * SHAFT_D / 2), (s1 + TIP_L, TIP_D / 2)], n=12)
    return {"grip": grip, "collar": collar, "cap": cap, "shaft": shaft, "tip": tip}


def rest_posts() -> list:
    """The rest's boxes as (name, size, centre) in world coordinates: a post under the handle's back end, and a post
    with two cheeks (they keep the flats upright) under its front end. The middle of the handle is left free for the
    gripper."""
    x, y = REST_XY
    base = 0.004  # the base plate's thickness; the posts stand on it
    z = L.TOP + base
    back, front = -HANDLE_L / 2 + 0.012, HANDLE_L / 2 - 0.024  # where the posts stand along the handle
    back_h = REST_AXIS_Z - grip_radius(back) - base
    front_h = REST_AXIS_Z - grip_radius(front) - base
    cheek_h = front_h + 0.012
    seat = HANDLE_FLAT + 0.0016  # the front post is as wide as the gap between its cheeks
    cheek_y = seat / 2 + REST_CHEEK / 2
    return [
        ("rest_base", (HANDLE_L + 0.020, 0.060, base), (x, y, L.TOP + base / 2)),
        ("rest_back", (REST_POST, 0.020, back_h), (x + back, y, z + back_h / 2)),
        ("rest_front", (REST_POST, seat, front_h), (x + front, y, z + front_h / 2)),
        ("rest_cheek_l", (REST_POST, REST_CHEEK, cheek_h), (x + front, y + cheek_y, z + cheek_h / 2)),
        ("rest_cheek_r", (REST_POST, REST_CHEEK, cheek_h), (x + front, y - cheek_y, z + cheek_h / 2)),
    ]


def driver_start() -> np.ndarray:
    """The middle of the handle with the screwdriver lying on its rest, world."""
    return np.array([REST_XY[0], REST_XY[1], L.TOP + REST_AXIS_Z + 0.0005])


# -- where a screw is ------------------------------------------------------------------------------------------------------
def slot_points(device_pos, device_quat) -> np.ndarray:
    """The strap's two screw slots on its FRONT face, world (2, 3)."""
    rot = L._rot(device_quat)
    return np.asarray(device_pos, float) + (L.SCREW_TABS + np.array([-L.STRAP_T, 0.0, 0.0])) @ rot.T


def stopped_by_wall(point) -> bool:
    """Whether a screw tip pushed in along x at ``point`` (y, z from a world point) meets the wall or a boss, not the
    open box."""
    y, z = float(point[1]) - L.OPEN_C[0], float(point[2]) - L.OPEN_C[1]
    if abs(y) > L.BOX_W / 2 or abs(z) > L.BOX_H / 2:
        return True  # the wall round the opening
    return abs(y) <= L.BOSS_W / 2 and abs(z) >= L.BOX_H / 2 - L.BOSS_P  # a boss


def captive_proud(slot) -> float:
    """How far a captive screw's head stands off the strap's front face at ``slot`` (world): its tip rests on the boss
    or the wall once the strap is nearer than the screw is long."""
    reach = L.WALL_X - float(slot[0])  # from the strap's front face to the wall's face
    if stopped_by_wall(slot):
        return max(SLOP, SCREW_LEN - reach)
    return SLOP


def _capped(v, most: float):
    n = float(np.linalg.norm(v))
    return v if n <= most else v * (most / n)


def pin_forces(device_pos, device_quat, started, memory=None, dt: float = PHYSICS_DT) -> list:
    """What the started screws' shanks do to the strap: [(force on the outlet (N, world), the point it acts at)], one
    for each started screw. The force pulls the slot toward its hole across the wall (y, z), damps the slot's slide,
    and has no part along the screw. ``memory`` is the caller's dict for one trial (Screws.pin): the slots' last
    places, for their speed, and each shank's built-up force."""
    out = []
    slots, holes = slot_points(device_pos, device_quat), L.boss_holes()
    memory = {} if memory is None else memory
    last, built = memory.get("slots"), memory.setdefault("built", np.zeros((2, 3)))
    for i in range(2):
        if not started[i]:
            built[i] = 0.0
            continue
        off = np.array([0.0, slots[i][1] - holes[i][1], slots[i][2] - holes[i][2]])
        r = float(np.linalg.norm(off))
        pull = np.zeros(3) if r <= PIN_PLAY else -off / r * (r - PIN_PLAY)  # from the slot to where it may sit (m)
        vel = np.zeros(3) if last is None else (np.asarray(slots[i], float) - last[i]) / dt
        vel[0] = 0.0
        built[i] = _capped(built[i] + KI_PIN * pull * dt, F_PIN)
        out.append((_capped(K_PIN * pull - C_PIN * vel + built[i], F_PIN), np.asarray(slots[i], float)))
    memory["slots"] = [np.asarray(p, float).copy() for p in slots]
    return out


def catches(device_pos, device_quat, i: int) -> bool:
    """Whether screw ``i`` can start its thread: its slot within CATCH_R of the boss hole across the wall, its tip at the
    boss, and the outlet square to the wall."""
    slot = slot_points(device_pos, device_quat)[i]
    hole = L.boss_holes()[i]
    if float(np.linalg.norm(slot[1:] - hole[1:])) > CATCH_R:
        return False
    if L.WALL_X - float(slot[0]) > SCREW_LEN - SLOP:  # the tip doesn't reach the boss yet
        return False
    normal = L._rot(device_quat)[:, 0]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(normal[0]))))) <= CATCH_DEG


def standoff(turns: float) -> float:
    """A started screw's head underside from the boss face after ``turns`` forward turns (m)."""
    return SCREW_LEN - TRAVEL * min(max(turns, 0.0), TURNS) / TURNS


class Screws:
    """The two screws' state through a trial, advanced once per physics step from the outlet's and the screwdriver's
    poses. ``step`` returns each screw's pose and the force the recess puts on the screwdriver's tip."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.turns = [0.0, 0.0]  # forward turns since the thread started
        self.spin = [0.0, 0.0]  # how far the screw has turned in all (rad), for its look
        self.started = [False, False]
        self.engaged = [False, False]
        self.roll = [0.0, 0.0]  # the screwdriver's roll about the screw at the last step
        self.pin = {}  # pin_forces' memory: the slots' last places and each shank's built-up force

    def home(self, i: int) -> bool:
        return self.started[i] and self.turns[i] >= TURNS - 1e-6

    def pose(self, device_pos, device_quat, i: int):
        """The middle of screw ``i``'s head underside, its axis (unit, down the shank) and its rotation."""
        if self.started[i]:
            hole = L.boss_holes()[i]
            pos = np.array([L.WALL_X - standoff(self.turns[i]), hole[1], hole[2]])
            base = np.eye(3)
        else:
            base = L._rot(device_quat)
            slot = slot_points(device_pos, device_quat)[i]
            pos = slot - base[:, 0] * captive_proud(slot)
        return pos, base[:, 0].copy(), base @ rot_x(self.spin[i])

    def step(self, device_pos, device_quat, driver_pos, driver_rot, driver_vel=None, driver_ang=None):
        """One physics step. ``driver_pos``/``driver_rot``: the screwdriver's origin and rotation (3x3), world.
        Returns ([(origin, quat_xyzw)] for the two screw bodies, the force on the screwdriver's tip (N, world), and
        the tip (world))."""
        driver_pos, driver_rot = np.asarray(driver_pos, float), np.asarray(driver_rot, float)
        tip = driver_pos + driver_rot @ DRIVER_TIP
        along = driver_rot[:, 0]
        force = np.zeros(3)
        poses = []
        for i in range(2):
            pos, axis, _ = self.pose(device_pos, device_quat, i)
            top = pos - axis * HEAD_H  # the middle of the head's top
            d = tip - top
            axial = float(d @ axis)  # > 0: the tip is past the head's top
            radial_v = d - axial * axis
            radial = float(np.linalg.norm(radial_v))
            off = math.degrees(math.acos(max(-1.0, min(1.0, float(along @ axis)))))
            roll = _roll(driver_rot, axis)
            if radial <= HEAD_D / 2 and 0.0 < axial <= HEAD_REACH:  # the bit is on the head: the head stops it
                push = K_HEAD * axial
                if driver_vel is not None:
                    v_tip = np.asarray(driver_vel, float)
                    if driver_ang is not None:
                        v_tip = v_tip + np.cross(np.asarray(driver_ang, float), tip - driver_pos)
                    push += C_HEAD * max(0.0, float(v_tip @ axis))
                force -= axis * min(F_HEAD, push)
            if not self.engaged[i]:
                if radial <= ENGAGE_R and -ENGAGE_BACK <= axial <= ENGAGE_IN and off <= ENGAGE_DEG:
                    self.engaged[i], self.roll[i] = True, roll
            elif radial > RELEASE_R or axial < -RELEASE_BACK or off > RELEASE_DEG:
                self.engaged[i] = False
            if self.engaged[i]:
                f = -K_RECESS * radial_v
                if driver_vel is not None:
                    v = np.asarray(driver_vel, float)
                    if driver_ang is not None:
                        v = v + np.cross(np.asarray(driver_ang, float), tip - driver_pos)
                    f = f - C_RECESS * (v - (v @ axis) * axis)
                n = float(np.linalg.norm(f))
                force += f if n <= F_RECESS else f * (F_RECESS / n)
                turn = _wrap(roll - self.roll[i])
                self.roll[i] = roll
                if turn > 0.0:  # the ratchet: only forward turns reach the screw
                    if not self.started[i] and catches(device_pos, device_quat, i):
                        self.started[i] = True
                    if self.started[i]:
                        gain = min(turn / (2 * math.pi), TURNS - self.turns[i])
                        self.turns[i] += gain
                        self.spin[i] += 2 * math.pi * gain
                    else:
                        self.spin[i] += turn  # not at its hole: it only spins in the strap
            pos, axis, rot = self.pose(device_pos, device_quat, i)
            # The body itself is not turned with the screw: a turning head that nothing can stop drags the strap it
            # presses on round with it (first Isaac run of a screw driven home: the other slot swung 2 mm off its
            # hole), where a real screw's shank keeps the strap on the hole. The head still grips the strap by
            # friction, which is what holds the outlet in place once it is home. (``spin`` keeps count for pose().)
            still = rot @ rot_x(-self.spin[i])
            poses.append((pos - axis * HEAD_H / 2, matrix_to_quat_xyzw(still)))  # the body's origin: the head's middle
        return poses, force, tip


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _roll(driver_rot, axis) -> float:
    """The screwdriver's turn about ``axis``, measured against the world: positive is clockwise seen from the handle
    (right-handed about the way the screw goes in)."""
    ref = np.array([0.0, 0.0, 1.0]) - axis[2] * axis
    v = driver_rot[:, 2] - (driver_rot[:, 2] @ axis) * axis
    if np.linalg.norm(v) < 1e-9 or np.linalg.norm(ref) < 1e-9:
        return 0.0
    ref, v = _unit(ref), _unit(v)
    return math.atan2(float(np.cross(ref, v) @ axis), float(ref @ v))


# -- scoring ---------------------------------------------------------------------------------------------------------------
def screw_home(screw_pos, i: int) -> bool:
    """Whether screw ``i`` is driven home, from where its head is (``screw_pos``: the middle of the head, world): the
    head's underside no more than the strap's thickness plus HOME_TOL off the wall, on its boss hole."""
    hole = L.boss_holes()[i]
    p = np.asarray(screw_pos, float)
    under = L.WALL_X - (p[0] + HEAD_H / 2)
    return bool(under <= S_HOME + HOME_TOL and float(np.linalg.norm(p[1:] - hole[1:])) <= 0.001)


def screws_home(screw_positions) -> int:
    return sum(screw_home(p, i) for i, p in enumerate(screw_positions))


def task_score(seated: float, homes: int) -> float:
    """The task's score from the seating score (outlet_logic.task_score: 0, 0.5 off its holes, 1.0 on them, each only
    once let go) and how many screws are home."""
    if seated <= 0.0:
        return 0.0
    if seated < 1.0:
        return 0.25
    return (0.5, 0.75, 1.0)[max(0, min(2, int(homes)))]
