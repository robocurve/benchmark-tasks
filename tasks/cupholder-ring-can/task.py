"""Set a can of soda in a seat-back fold-down cup holder: fold down its tray, swing out its ring, set the can in upright through the ring onto the tray, let go.

A seat-back panel section (45 cm wide, 40 cm tall, 5 cm thick) stands on the bench facing the arms. On its face, a fold-up drink holder (marine type) at true size: a 98 x 95 mm
plate, 4 mm thick, with two folding parts. A ring (70 mm inside, 80 mm outside, band 8 mm tall) on two side arms pivots at the plate's top corners; a tray (72 x 84 mm, 3 mm floor,
7 mm lips) hinges at its bottom edge. Stowed, the ring hangs flat against the plate and the tray stands folded up over it (33 mm deep in all; the stowed ring stands 10 mm off the plate). Open, both stand out level at hard
stops: the ring's centre 60 mm out from the plate, its top face 16 cm above the bench and 84 mm over the tray floor. Stiff pivots hold each part where it is put (damping
0.05 N m s/rad, estimate; weight left out), no spring. A 12 oz / 355 ml can (66 mm across, 122 mm tall, 370 g, rigid) stands on a 10 cm mat on the bench.
Geometry is real: the can goes through the ring or it does not, with 2 mm a side, and stands on the tray. Code: only the scorer.

    python demokit.py task.py OUT --no-record --mode full,none,ringonly,beside,hold,drop,onearm,ringfirst
    python demokit.py task.py OUT          (final: 640 x 360, the GPU to yourself)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402
from yam_arena.ik import smoothstep as ik_smooth  # noqa: E402

SCENE = "station"
INSTRUCTION = "Put the can of soda in the seat-back cup holder: fold down its tray, swing out its ring, and set the can upright through the ring onto the tray, then let go."
SUCCESS_HOLD_S = 0.0  # the scorer holds the end state itself (HOLD_STEPS)
SCORE_OVER = "end"
TOP = d.TABLE_TOP
MM = 1e-3

# -- sizes (metres) ------------------------------------------------------------------------------------------------------------------------
# Fold-up drink holder (marine type): a 98 x 95 mm plate; a ring (70 mm inside) on two side arms pivoted at the plate's top corners; a square tray hinged at the plate's bottom edge.
# Folded (stowed), the ring hangs flat against the plate and the tray stands up over it (33 mm deep in all; E 15/16 in, 24 mm: the stowed ring stands 10 mm off the plate here). Open, both stand out level: the can stands on the tray, through the ring.
R_I, R_O, RB_H = 35.0 * MM, 40.0 * MM, 8.0 * MM  # the ring: 70 mm inside (drinks up to 2-3/4 in, 69.9 mm), 80 mm outside, its band 8 mm tall
C_R = 42.0 * MM  # the ring's centre out from its pivot line
ARM_X, ARM_Y = (-34.0 * MM, 0.0), (39.0 * MM, 43.5 * MM)  # the ring's side arms, from the pivot out, and their span across y
T_L, T_W, FLOOR_T, LIP_H, LIP_T = 72.0 * MM, 84.0 * MM, 3.0 * MM, 7.0 * MM, 2.5 * MM  # the tray: 72 deep x 84 wide, 3 mm floor, 7 mm lips on its front and sides
RING_ABOVE = 84.0 * MM  # the open ring's top face above the tray floor's top (read from the product photo, estimate); the hanging ring (82 mm below its pivot) swings out clear of the open tray
PLATE_W, PLATE_H, PLATE_T = 98.0 * MM, 95.0 * MM, 4.0 * MM
PANEL_W, PANEL_H, PANEL_T = 0.45, 0.40, 0.05
CAN_R, CAN_H, CAN_MASS = 33.0 * MM, 122.2 * MM, 0.370  # Crown 12 oz standard: 66 mm x 122.2 mm; 355 g of drink + 15 g of can
SELF_COLLIDE = os.environ.get("CH_SELFCOL", "1") == "1"  # the ring and the tray collide with each other: the stowed tray blocks the ring until it is folded down
LINK_MASS, HINGE_DAMPING = 0.02, 0.05  # kg; N m s/rad (estimate): stiff pivots hold the ring and the tray where they are put (no spring, no detent published); their weight is left out

PANEL_X = float(os.environ.get("CH_PANEL_X", "0.360"))  # the panel's face
PLATE_X = PANEL_X - PLATE_T  # the plate's front face
RING_TOP = TOP + float(os.environ.get("CH_RING_H", "0.16"))
PIVOT = (PLATE_X - 18.0 * MM, 0.0, RING_TOP)  # the ring's pivot line (along y), at the open ring's top face; the holder's origin. 18 mm out: the stowed ring stands 10 mm off the plate, room for a fingertip behind it
TRAY_Z = RING_TOP - RING_ABOVE  # the tray floor's top, open, and its hinge line
TRAY_HX = PIVOT[0] - 8.0 * MM  # the tray's hinge line, 26 mm in front of the plate
TH = np.array([TRAY_HX - PIVOT[0], 0.0, TRAY_Z - RING_TOP])  # the tray's hinge in the holder's frame
PLATE_Z = (TRAY_Z - RING_TOP - 6.0 * MM, 5.0 * MM)  # the plate's bottom and top in the holder's frame: 6 mm under the tray hinge to 5 mm over the ring pivot (95 mm)
CAN_AT = (float(os.environ.get("CH_CX", "0.22")), float(os.environ.get("CH_CY", "-0.17")), TOP + 0.002)  # the can's base, on its mat (2 mm thick)

LOOKS = {
    "abs": m.Look(m.rgb(0.06, 0.06, 0.07), roughness=0.45),
    "plate": m.Look(m.rgb(0.09, 0.09, 0.10), roughness=0.5),
    "seat": m.Look(m.rgb(0.52, 0.50, 0.46), roughness=0.95),
    "trim": m.Look(m.rgb(0.30, 0.29, 0.27), roughness=0.7),
    "mat": m.Look(m.rgb(0.55, 0.45, 0.32), roughness=0.9),
    "alu": m.Look(m.rgb(0.80, 0.81, 0.83), metallic=1.0, roughness=0.25),
    "red": m.Look(m.rgb(0.78, 0.06, 0.08), roughness=0.35),
    "white": m.Look(m.rgb(0.95, 0.95, 0.95), roughness=0.4),
}


def ry(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


RING_FOLD, TRAY_FOLD = -90.0, 90.0  # each link is drawn open, then turned by this about y to its stowed pose (joint zero): the ring hangs down, the tray stands up


def ring_link() -> m.Link:
    """Drawn open, pivot line at the origin, band top face at z 0, ring toward -x; turned to hang down. Joint 0 stowed, +90 open (hard stop: arms level)."""
    c = np.array([-C_R, 0.0, 0.0])
    f = lambda g: g.turned(RING_FOLD, "y")
    cols = []
    for i in range(16):  # the band, 16 convex segments
        a0, a1 = 2 * math.pi * i / 16, 2 * math.pi * (i + 1) / 16
        seg = np.array([[R_I * math.cos(a0), R_I * math.sin(a0)], [R_O * math.cos(a0), R_O * math.sin(a0)], [R_O * math.cos(a1), R_O * math.sin(a1)], [R_I * math.cos(a1), R_I * math.sin(a1)]])
        cols.append(m.col_hull(f"band{i}_c", f(m.prism(seg, -RB_H, 0.0).moved(c))))
    band = m.lathe([(R_I, -RB_H), (R_O, -RB_H), (R_O, 0.0), (R_I, 0.0), (R_I, -RB_H)], 48).moved(c)
    parts = [("band", f(band), "abs")]
    for k, s in enumerate((-1, 1)):
        arm = m.box((ARM_X[1] - ARM_X[0], ARM_Y[1] - ARM_Y[0], RB_H), ((ARM_X[0] + ARM_X[1]) / 2, s * (ARM_Y[0] + ARM_Y[1]) / 2, -RB_H / 2))
        knuckle = m.cylinder(4.0 * MM, 4.0 * MM, (0.0, s * 44.0 * MM, -4.0 * MM), axis="y")
        parts += [(f"arm{k}", f(arm), "abs"), (f"knuckle{k}", f(knuckle), "abs")]
        cols.append(m.col_hull(f"arm{k}_c", f(arm)))
    return m.hinge("ring", "Y", (0.0, 0.0, 0.0), 0.0, 90.0, parts, cols, mass=LINK_MASS, gravity=False, friction=0.0, damping=HINGE_DAMPING)


def tray_link() -> m.Link:
    """Drawn open, hinge line at the origin, floor top at z 0, tray toward -x; turned to stand up; moved to its hinge. Joint 0 stowed, -90 open (hard stop: level)."""
    f = lambda g: g.turned(TRAY_FOLD, "y").moved(TH)
    floor = m.box((T_L, T_W, FLOOR_T), (-T_L / 2, 0.0, -FLOOR_T / 2))
    front = m.box((LIP_T, T_W, LIP_H), (-T_L + LIP_T / 2, 0.0, LIP_H / 2))
    sides = [m.box((T_L, LIP_T, LIP_H), (-T_L / 2, s * (T_W - LIP_T) / 2, LIP_H / 2)) for s in (-1, 1)]
    knuckles = [m.cylinder(4.0 * MM, 8.0 * MM, (0.0, s * (T_W / 2 - 4.0 * MM), -1.5 * MM), axis="y") for s in (-1, 1)]
    geo = [("floor", floor), ("front", front), ("side0", sides[0]), ("side1", sides[1])]
    parts = [(n, f(g), "abs") for n, g in geo] + [(f"knuckle{k}", f(g), "abs") for k, g in enumerate(knuckles)]
    cols = [m.col_hull(f"{n}_c", f(g)) for n, g in geo]
    return m.hinge("tray", "Y", tuple(TH), -90.0, 0.0, parts, cols, mass=LINK_MASS, gravity=False, friction=0.0, damping=HINGE_DAMPING)


PLATE_BOX = (np.array([PLATE_X - PIVOT[0], -PLATE_W / 2, PLATE_Z[0]]), np.array([PANEL_X - PIVOT[0], PLATE_W / 2, PLATE_Z[1]]))
LUGS = [(np.array([-6.0 * MM, s * 47.5 * MM - 1.5 * MM, -12.0 * MM]), np.array([PLATE_X - PIVOT[0], s * 47.5 * MM + 1.5 * MM, 5.0 * MM])) for s in (-1, 1)] + \
       [(np.array([TH[0] - 6.0 * MM, s * 46.0 * MM - 2.0 * MM, TH[2] - 6.0 * MM]), np.array([PLATE_X - PIVOT[0], s * 46.0 * MM + 2.0 * MM, TH[2] + 6.0 * MM])) for s in (-1, 1)]


def holder() -> m.Part:
    """The plate and its four lugs are the fixed frame (origin on the ring's pivot line); the ring and the tray are two links, each on its own pivot, independent (the real unit's
    two parts are not linked, as far as its photos show)."""
    box = lambda lo, hi: m.box(tuple(hi - lo), tuple((hi + lo) / 2))
    fparts = [("plate", box(*PLATE_BOX), "plate")] + [(f"lug{i}", box(lo, hi), "plate") for i, (lo, hi) in enumerate(LUGS)]
    pz = (PLATE_Z[0] + PLATE_Z[1]) / 2
    fparts += [(f"screw{i}", m.cylinder(3.0 * MM, 0.6 * MM, (PLATE_X - PIVOT[0] - 0.2 * MM, sy * 38 * MM, pz + sz * 30 * MM), axis="x"), "trim") for i, (sy, sz) in enumerate([(-1, -1), (-1, 1), (1, -1), (1, 1)])]
    lo, hi = PLATE_BOX
    return m.mechanism("holder", PIVOT, [ring_link(), tray_link()], {k: LOOKS[k] for k in ("abs", "plate", "trim")}, frame_parts=fparts,
                       frame_colliders=[m.col_box("plate_c", tuple(hi - lo), tuple((hi + lo) / 2))], self_collide=SELF_COLLIDE)


def panel() -> m.Part:
    """The seat back's lower section, its face at PANEL_X, standing on the bench; a stitched seam and a darker trim band for the look."""
    cx = PANEL_X + PANEL_T / 2
    parts = [("board", m.box((PANEL_T, PANEL_W, PANEL_H), (0.0, 0.0, PANEL_H / 2)), "seat"),
             ("trim", m.box((PANEL_T + 2 * MM, PANEL_W + 2 * MM, 0.03), (0.0, 0.0, 0.015)), "trim"),
             ("seam_l", m.box((1 * MM, 2 * MM, PANEL_H - 0.04), (-PANEL_T / 2 - 0.4 * MM, 0.16, PANEL_H / 2 + 0.01)), "trim"),
             ("seam_r", m.box((1 * MM, 2 * MM, PANEL_H - 0.04), (-PANEL_T / 2 - 0.4 * MM, -0.16, PANEL_H / 2 + 0.01)), "trim")]
    return m.fixed("panel", (cx, 0.0, TOP), parts, [m.col_box("board_c", (PANEL_T, PANEL_W, PANEL_H), (0.0, 0.0, PANEL_H / 2))], {k: LOOKS[k] for k in ("seat", "trim")})


def mat() -> m.Part:
    return m.fixed("mat", (CAN_AT[0], CAN_AT[1], TOP), [("mat", m.box((0.10, 0.10, 0.002), (0.0, 0.0, 0.001)), "mat")], [m.col_box("mat_c", (0.10, 0.10, 0.002), (0.0, 0.0, 0.001))], {"mat": LOOKS["mat"]})


CAN_PROFILE = [(0.0, 0.0), (27.0 * MM, 0.0), (CAN_R, 6.0 * MM), (CAN_R, 108.0 * MM), (27.5 * MM, 119.0 * MM), (26.5 * MM, CAN_H), (0.0, CAN_H)]


def can() -> m.Part:
    """A full 12 oz can, origin at the middle of its base, axis +z: one rigid body, the drink's mass in it (no fluid)."""
    body = m.lathe(CAN_PROFILE, 40)
    parts = [("body", body, "alu"), ("label", m.lathe([(CAN_R + 0.2 * MM, 14.0 * MM), (CAN_R + 0.2 * MM, 100.0 * MM)], 40), "red"),
             ("band", m.lathe([(CAN_R + 0.3 * MM, 52.0 * MM), (CAN_R + 0.3 * MM, 62.0 * MM)], 40), "white"),
             ("tab", m.box((18 * MM, 9 * MM, 1.0 * MM), (5 * MM, 0.0, CAN_H + 0.3 * MM)), "alu")]
    return m.free("can", CAN_AT, parts, [m.col_hull("can_c", body)], {k: LOOKS[k] for k in ("alu", "red", "white")}, mass=CAN_MASS, friction=(0.5, 0.4))


OBJECTS = [panel(), holder(), mat(), can()]

# -- the scorer -----------------------------------------------------------------------------------------------------------------------------
OPEN_DEG = 80.0  # 0.33: the ring and the tray each within 10 degrees of fully open (90)
IN_DEG, IN_OFF_MM, IN_BELOW_MM = 10.0, 3.0, 3.0  # 0.67: the can's axis within 10 degrees of the ring's and of vertical, crossing the ring's top face within 3 mm of its centre (2 mm play), its base 3 mm or more below that face
SEAT_MM = 2.0  # 1.0: the base within 2 mm of the tray floor's top, tilt 10 degrees or less, both whole grippers 50 mm or more from the can and the holder, still, for 2 s
CLEAR = 0.050
HOLD_STEPS = 40  # 2 s at 20 Hz
TIP_DEG = 30.0  # void: the can tilted past 30 degrees once off its mat
DROP_SPEED = 0.30  # void: the can moving down faster than this, held or falling, once off its mat; read after every physics step
# gripper volume: a box over each whole gripper, in its own frame (palm, housing, pads, fingers, and the wrist link 12 cm back), sampled every 10 mm
_LAT = np.array([(x, y, z) for x in np.arange(-0.04, 0.0401, 0.01) for y in np.arange(-0.03, 0.0301, 0.01) for z in np.arange(-0.12, 0.1501, 0.01)])


def gripper_points(env) -> np.ndarray:
    base = env.unwrapped
    origin = m._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        i = robot.find_bodies("gripper")[0][0]
        pose = m._np(robot.data.body_link_pose_w)[0, i]
        out.append(pose[:3] - origin + _LAT @ m.quat_matrix(pose[3:7]).T)
    return np.concatenate([np.concatenate(out), m.fingertips(env)])


def ring_frame(hs) -> tuple[np.ndarray, np.ndarray]:
    """The ring's top-face centre (world) and its rotation (world from drawn-open axes: z up through the ring when open), from the holder's pose and the ring's joint."""
    r = ry(RING_FOLD + hs.deg("ring"))
    return hs.to_world(np.array([r @ np.array([-C_R, 0.0, 0.0])]))[0], hs.rotation() @ r


def tray_frame(hs) -> tuple[np.ndarray, np.ndarray]:
    """The tray's hinge (world) and its rotation (world from drawn-open axes: z up off the floor when open)."""
    return hs.to_world(np.array([TH]))[0], hs.rotation() @ ry(TRAY_FOLD + hs.deg("tray"))


def _box_dist(loc, lo, hi) -> np.ndarray:
    return np.linalg.norm(np.maximum(np.maximum(lo - loc, loc - hi), 0.0), axis=1)


def _cyl_dist(loc, r, z0, z1) -> np.ndarray:
    dr = np.maximum(np.hypot(loc[:, 0], loc[:, 1]) - r, 0.0)
    dz = np.maximum(np.maximum(z0 - loc[:, 2], loc[:, 2] - z1), 0.0)
    return np.hypot(dr, dz)


def clear_of(cs, hs, pts) -> tuple[float, float]:
    """Least distance (m) from the gripper points to the can's envelope (a cylinder 33 mm by 122 mm), and to the holder: the ring's band and arms, the tray (floor and lips), the plate and its lugs."""
    can_d = float(np.min(_cyl_dist(cs.to_local(pts), CAN_R, 0.0, CAN_H)))
    rc, rr = ring_frame(hs)
    rl = (pts - rc) @ rr  # ring frame, origin at the band's top-face centre
    ring = np.minimum(_cyl_dist(rl, R_O, -RB_H, 0.0), _box_dist(rl, np.array([C_R + ARM_X[0], -ARM_Y[1], -RB_H]), np.array([C_R + 4.0 * MM, ARM_Y[1], 0.0])))
    tc, tr = tray_frame(hs)
    tray = _box_dist((pts - tc) @ tr, np.array([-T_L, -T_W / 2, -FLOOR_T]), np.array([4.0 * MM, T_W / 2, LIP_H]))
    hl = hs.to_local(pts)
    frame = _box_dist(hl, *PLATE_BOX)
    for lo, hi in LUGS:
        frame = np.minimum(frame, _box_dist(hl, lo, hi))
    return can_d, float(np.min(np.minimum(np.minimum(ring, tray), frame)))


class Trial:
    def reset(self, env=None) -> None:
        self.tipped = self.dropped = False  # only the failures latch
        self.off_mat = False  # arms the void checks: the can's base has been more than 10 mm up
        self.held = 0
        self.tier_now = 0.0
        self.info = {}
        self.n = 0
        self.warned = False
        if env is not None:
            self.step(env)

    def _fall(self, cs) -> None:
        if self.off_mat and float(cs.lin_vel[2]) < -DROP_SPEED:
            self.dropped = True

    def step(self, env) -> None:
        cs, hs = m.state(env, "can"), m.state(env, "holder")
        self._hook(env)
        base = np.asarray(cs.pos, float)
        tilt = cs.tilt_deg()
        if base[2] - CAN_AT[2] > 0.010:
            self.off_mat = True
        self._fall(cs)
        if self.off_mat and tilt > TIP_DEG:
            self.tipped = True
        ring_deg, tray_deg = hs.deg("ring"), -hs.deg("tray")
        ring_c, ring_r = ring_frame(hs)
        bl = (base - ring_c) @ ring_r  # the can's base in the ring's frame
        al = cs.rotation()[:, 2] @ ring_r  # and its axis
        rel_tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(al[2])))))
        cross = bl - al * (bl[2] / al[2]) if al[2] > 0.5 else np.array([1.0, 1.0, 0.0])  # where the axis crosses the top face
        off = float(np.hypot(cross[0], cross[1]))
        is_open = ring_deg >= OPEN_DEG and tray_deg >= OPEN_DEG
        inside = is_open and rel_tilt <= IN_DEG and tilt <= IN_DEG and off <= IN_OFF_MM * MM and bl[2] <= -IN_BELOW_MM * MM
        tc, tr = tray_frame(hs)
        on_floor = float(((base - tc) @ tr)[2])  # the can's base above the tray floor's top
        can_d, hold_d = clear_of(cs, hs, gripper_points(env))
        still = float(np.linalg.norm(cs.lin_vel)) < 0.01 and float(np.linalg.norm(cs.ang_vel)) < 0.1
        seated = inside and abs(on_floor) <= SEAT_MM * MM and can_d >= CLEAR and hold_d >= CLEAR and still
        self.held = self.held + 1 if seated else 0
        if self.tipped or self.dropped:
            t = 0.0
        elif self.held >= HOLD_STEPS:
            t = 1.0
        elif inside:
            t = 0.67
        elif is_open:
            t = 0.33
        else:
            t = 0.0
        self.tier_now = t
        self.info = dict(ring=ring_deg, tray=tray_deg, tilt=tilt, off_mm=off * 1000, depth_mm=-bl[2] * 1000, floor_mm=on_floor * 1000, can_mm=can_d * 1000, holder_mm=hold_d * 1000, still=still, held=self.held,
                         tipped=self.tipped, dropped=self.dropped)
        self.n += 1
        if self.n % 20 == 0 and os.environ.get("CH_TRACE"):
            report(f"step {self.n}")

    def _hook(self, env) -> None:
        """Read the fall after every physics step too: a short drop lasts a few tens of ms and slips between 20 Hz samples."""
        scene = env.unwrapped.scene
        if getattr(scene, "_ch_hooked", None) is self:
            return
        orig = scene.update
        trial = self

        def update(*a, **k):
            r = orig(*a, **k)
            try:
                trial._fall(m.state(env, "can"))
            except Exception as e:  # the extra read must not break the run
                if not trial.warned:
                    trial.warned = True
                    print(f"[ch] substep read failed: {e!r}", flush=True)
            return r

        scene.update = update
        scene._ch_hooked = self


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.tier_now >= 1.0


def score(objects) -> float:
    return TRIAL.tier_now


def report(tag: str) -> None:
    i = TRIAL.info
    if i:
        print(f"[ch] {tag}: tier {TRIAL.tier_now}, ring {i['ring']:.1f} deg, tray {i['tray']:.1f} deg open, can tilt {i['tilt']:.1f}, axis off centre {i['off_mm']:.1f} mm, base {i['depth_mm']:.1f} mm below the ring top, {i['floor_mm']:.1f} mm over the tray floor, "
              f"hands {i['can_mm']:.0f} mm from can, {i['holder_mm']:.0f} mm from holder, still {i['still']}, held {i['held']}, tipped {i['tipped']}, dropped {i['dropped']}", flush=True)


# -- the demo -------------------------------------------------------------------------------------------------------------------------------
MODES = ("full", "none", "ringonly", "beside", "hold", "drop", "onearm", "ringfirst")
TRAY_PINCH = (float(os.environ.get("CH_TX", "0.002")), float(os.environ.get("CH_TZ", "0.030")))  # the pinch on the stowed tray's side lips, from its hinge (x, z)
RING_PINCH = (float(os.environ.get("CH_RX", "0.0")), -C_R)  # the pinch on the hanging ring's sides, from its pivot (x, z): across its widest point
PINCH_TILT = float(os.environ.get("CH_PINCH_TILT", "60"))  # the holder hand's fingers this far from straight down, toward the panel; jaws across y
GRIP_Z = float(os.environ.get("CH_GRIP_Z", "0.110"))  # the right hand closes on the can this far up it: seated, the fingers stay over the ring
CAN_TILT = float(os.environ.get("CH_CAN_TILT", "60"))  # the can hand's fingers this far from straight down, toward the panel; the jaws close across y, so the can hangs plumb from the pinch
CARRY = 0.035  # the can's base this far over the ring's top face before it goes down


def demo_script(ctx):
    """Modes: full (left pinches the stowed tray across its sides and swings it down to its stop, lets go; pinches the hanging ring across its sides and swings it up and out
    to its stop, lets go; meanwhile right picks the can high; right centres it over the ring, lowers it straight through onto the tray, lets go; both back away); none;
    ringonly (tray and ring opened, nothing else: 0.33); beside (can stood on the bench by the panel: 0.33); hold (as full, right keeps hold of the can: 0.67);
    drop (let go 35 mm above the ring: void, 0); onearm (the right arm alone opens the tray and the ring, then sets the can); ringfirst (the ring swung out
    with the tray still stowed over it: blocked, 0)."""
    mode = ctx.mode
    if mode not in MODES:
        raise ValueError(f"unknown mode {mode!r}")
    l, r = ctx.left, ctx.right
    if mode == "none":
        yield from r.wait(3.0)
        report("end")
        return
    P = np.array(PIVOT)
    T = P + TH
    gp = d.level(0.0) if PINCH_TILT >= 89.9 else d.tilted(PINCH_TILT, 0.0)  # level: the wrist camera rides above the fingers, clear of the panel
    fdir = np.array([math.sin(math.radians(PINCH_TILT)), 0.0, -math.cos(math.radians(PINCH_TILT))])  # along the holder hand's fingers

    def on_circle(c, v, deg):
        """The point c + v carried round the y line through c by `deg` (positive: top toward the arms)."""
        a = math.radians(deg)
        return c + np.array([v[0] * math.cos(a) - v[1] * math.sin(a), 0.0, v[0] * math.sin(a) + v[1] * math.cos(a)])
    ring_top = lambda: ring_frame(ctx.state("holder"))[0]
    tray_floor = lambda: tray_frame(ctx.state("holder"))[0][2]
    can_base = lambda: np.asarray(ctx.state("can").pos, float)

    def swing(arm, c, v, deg, width, what):
        start = on_circle(c, v, 0.0)
        yield from arm.move(start - 0.06 * fdir, grip=gp, t=2.0, label=f"over the stowed {what}")
        yield from arm.move(start, t=1.0, label=f"fingers either side of the {what}")
        yield from arm.close(width, force=15.0, label=f"pinch the {what}")
        # carry the pinch round the pivot, the grip held: the part turns between the pads about the jaws' line
        yield from arm.servo(lambda s: on_circle(c, v, deg * ik_smooth(min(1.0, s / 3.0))), tol=0.003, timeout=4.5, gain=0.3, label=f"swing the {what} to its stop")
        yield from arm.wait(0.3)
        yield from arm.open(t=0.5, label=f"let go of the {what}")
        yield from arm.move(arm.at() - 0.08 * fdir, t=1.2, label=f"back off the {what}")
        report(f"{what} open")

    def open_holder(arm):
        yield from swing(arm, T, TRAY_PINCH, 92.0, T_W, "tray")
        yield from swing(arm, P, RING_PINCH, -92.0, 2 * R_O, "ring")
        side = 1.0 if arm is l else -1.0
        yield from arm.move(np.array([0.18, side * 0.22, TOP + 0.22]), t=2.0, label="clear of the holder, back by its own base")

    def pick_can(arm):
        yield from arm.pick(lambda: can_base(), GRIP_Z, width=2 * CAN_R, grip=d.tilted(CAN_TILT, 0.0), above=0.06, lift=0.05, force=20.0, t=2.5, label="can")
        yield from arm.move(arm.at() + np.array([0.0, 0.0, RING_TOP + CARRY - can_base()[2] + 0.01]), t=1.5, label="lift over the ring's height")
        report("can up")

    def level_up(arm):
        for _ in range(2):
            a = ctx.state("can").rotation()[:, 2]
            if math.degrees(math.acos(min(1.0, a[2]))) < 0.7:
                return
            ax = np.cross(a, [0.0, 0.0, 1.0])
            ang = math.degrees(math.asin(min(1.0, np.linalg.norm(ax))))
            yield from arm.arc(arm.tcp(), ax / np.linalg.norm(ax), ang, t=0.8, label="stand the can straight")
            yield from arm.wait(0.4)

    def set_can(arm, spot=None):
        goal = (lambda dz: ring_top() + np.array([0.0, 0.0, dz])) if spot is None else (lambda dz: np.asarray(spot) + np.array([0.0, 0.0, dz]))
        off = arm.tcp() - can_base()
        yield from arm.move(goal(CARRY) + off, t=2.5, label="over the ring")
        yield from level_up(arm)
        yield from arm.servo(lambda s: goal(CARRY), read=can_base, tol=0.001, timeout=3.0, label="centre over the ring")
        report("over the ring")
        if mode == "drop":
            yield from arm.open(t=0.4, label="let go above")
            return
        bottom = (tray_floor() - ring_top()[2] + 0.0005) if spot is None else 0.0005
        yield from arm.servo(lambda s: goal(max(bottom, CARRY - 0.02 * s)), read=can_base, tol=0.0015, timeout=8.0, gain=0.25, label="lower it straight through onto the tray")
        yield from arm.wait(0.5)
        report("can in")
        if mode == "hold":
            yield from arm.wait(3.5)
            return
        yield from arm.open(t=0.6, label="let go of the can")
        yield from arm.wait(0.3)
        yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.10]), t=1.5, label="hand up and off")
        yield from arm.move(arm.at() + np.array([-0.14, -0.14, 0.0]), t=1.8, label="hand back, out of the top camera's view")

    yield from r.wait(0.5)
    if mode == "onearm":  # the right arm alone: open the tray and the ring, then fetch and set the can
        yield from open_holder(r)
        yield from pick_can(r)
        yield from set_can(r)
    elif mode == "ringonly":
        yield from open_holder(l)
    elif mode == "ringfirst":  # the ring swung out with the tray still stowed over it: the tray blocks it
        yield from swing(l, P, RING_PINCH, -92.0, 2 * R_O, "ring")
    else:
        yield from ctx.together(open_holder(l), pick_can(r))
        yield from set_can(r, spot=(PANEL_X - 0.06, -0.12, TOP) if mode == "beside" else None)
    yield from r.wait(3.0)
    report("end")
