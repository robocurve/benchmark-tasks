"""Slit carton: open a taped carton with a utility knife without cutting into the product packed under its flaps, so that both flaps are free; the blade starts drawn in, the
left hand pushes it out on its slider and it goes back in before the knife is put down.

The carton is a single-wall 40 x 30 x 25 cm box, its two top flaps sealed with one strip of 48 mm packing tape along the centre seam, stopping at the two top edges. A product box
fills it, its top 6 mm under the tape. Pushed out, the knife's blade stands 20 mm out, so with its nose on the board the tip goes 20 mm x sin(angle) deep: through the tape above
about 2 degrees, into the product above about 17. Holding that window along the whole 40 cm seam, in a straight line, is the task.

    python demokit.py task.py OUT --no-record --mode full,none,deep,half,lockpush    debug
    python demokit.py task.py OUT                                                   final video

Simulation: the flaps are rigid plates on crease hinges, held shut while any tape across the seam is whole. The tape is a row of 2 cm pieces; a piece is cut when the blade's
edge has passed through it below the tape's surface. The blade has no collider (a box cutter cuts board), and while its tip is under the surface a 3 N drag opposes the knife's
motion, standing in for the tape and board resisting the cut. Any moment a corner of the blade is below the product's top, over the product, counts as a cut into it.
"""

from __future__ import annotations

import dataclasses
import math
import os

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # mechkit.py and demokit.py sit beside this file

import demokit as d  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
INSTRUCTION = "Use the knife to cut the box open along its taped seam without cutting what's inside, then draw the blade back in."
SUCCESS_HOLD_S = 0.0  # the scorer counts its own 3 s hold: score() and success() agree
HOLD_STEPS = 60  # 3 s at 20 Hz
SCORE_OVER = "end"
TOP = d.TABLE_TOP

# -- the carton, its frame: origin at the middle of its bottom, x along its 40 cm, the seam on y = 0 -----------------------------
CARTON_AT = (0.50, 0.025)
LX, LY, LZ, BOARD = 0.40, 0.30, 0.25, 0.004
GAP = 0.003
FLAP_W = LY / 2 - GAP / 2
TAPE_W, TAPE_T = 0.048, 0.0006
PRODUCT_TOP = LZ - BOARD - 0.002  # 6 mm under the tape's top (LZ)
CARTON_MASS = 3.0
SEG = 0.02
N_SEG = round(LX / SEG)
CUT_BELOW = 0.0005  # the blade's edge this far under the tape's surface cuts it
DRAG_N = 3.0
RISEN_DEG = 5.0  # a freed flap springs up about 10 degrees on its crease; this much shows it free in the simulator

LOOKS = {"board": m.Look(m.rgb(0.68, 0.52, 0.33), roughness=0.9), "product": m.Look(m.rgb(0.88, 0.88, 0.85), roughness=0.7),
         "tape": m.Look(m.rgb(0.72, 0.55, 0.32), roughness=0.3), "handle": m.Look(m.rgb(0.90, 0.68, 0.05), roughness=0.45),
         "grey": m.Look(m.rgb(0.20, 0.20, 0.22), roughness=0.5), "blade": m.Look(m.rgb(0.78, 0.79, 0.81), metallic=0.9, roughness=0.25)}


def _frame():
    parts, cols = [], []
    walls = (("wall_xn", (BOARD, LY, LZ - BOARD), (-LX / 2 + BOARD / 2, 0, (LZ - BOARD) / 2)), ("wall_xp", (BOARD, LY, LZ - BOARD), (LX / 2 - BOARD / 2, 0, (LZ - BOARD) / 2)),
             ("wall_yn", (LX, BOARD, LZ - BOARD), (0, -LY / 2 + BOARD / 2, (LZ - BOARD) / 2)), ("wall_yp", (LX, BOARD, LZ - BOARD), (0, LY / 2 - BOARD / 2, (LZ - BOARD) / 2)),
             ("bottom", (LX, LY, BOARD), (0, 0, BOARD / 2)))
    for name, size, c in walls:
        parts.append((name, m.box(size, c), "board"))
        cols.append(m.col_box(name + "_c", size, c))
    psize = (LX - 2 * BOARD - 0.004, LY - 2 * BOARD - 0.004, PRODUCT_TOP - BOARD)
    pc = (0, 0, BOARD + psize[2] / 2)
    parts.append(("product", m.box(psize, pc), "product"))
    cols.append(m.col_box("product_c", psize, pc))
    return parts, cols


def _flap(name: str, sign: float) -> m.Link:
    """sign -1: the flap on the -y side, hinged at y = -LY/2, opening by positive angles; +1 the other."""
    c = (0.0, sign * (LY / 2 - FLAP_W / 2), LZ - BOARD / 2)
    plate = m.box((LX, FLAP_W, BOARD), c)
    tape = m.box((LX, TAPE_W / 2 - GAP / 2, TAPE_T), (0.0, sign * (GAP / 2 + (TAPE_W / 2 - GAP / 2) / 2), LZ + TAPE_T / 2))
    hinge_at = (0.0, sign * LY / 2, LZ - BOARD / 2)
    lower, upper = (0.0, 200.0) if sign < 0 else (-200.0, 0.0)  # shut, a flap rests on its stop: it may not sag into the product
    return m.hinge(name, "X", hinge_at, lower, upper, [("plate", plate, "board"), ("tape", tape, "tape")], [m.col_box(f"{name}_c", (LX, FLAP_W, BOARD), c)],
                   mass=0.035, spring=0.3, damping=0.05, friction=0.01, gravity=True, com=c)


_fp, _fc = _frame()
CARTON = m.mechanism("carton", (CARTON_AT[0], CARTON_AT[1], TOP), [_flap("flap_n", -1.0), _flap("flap_p", 1.0)], LOOKS, frame_parts=_fp, frame_colliders=_fc, free_mass=CARTON_MASS)

# -- the knife: a retractable utility knife. Frame x along it toward the tip, y in the blade's plane toward the spine, z across it; origin the handle's middle.
# The handle is the frame; the blade, with the slider button on the spine, is a slide along x: joint zero retracted (its tip at the nose), BLADE_OUT fully out.
H_LEN, H_TALL, H_THICK = 0.150, 0.028, 0.018
BLADE_OUT, BLADE_T = 0.020, 0.0006
OUT_AT = BLADE_OUT - 0.001  # the blade counts as out from here (its stop is BLADE_OUT)
IN_AT = 0.0005  # and as back in under this
DETENT_N = 2.0  # the slider's friction, N (PhysX joint friction, which grows with the load through the joint)
NOTCH, NOTCH_K = 0.0015, 300.0  # the slider's two notches, at the blade's in and out stops: within NOTCH of a stop a NOTCH_K spring (at most 0.45 N) holds it there
NOSE = np.array([H_LEN / 2, -H_TALL / 2, 0.0])  # where the blade leaves the nose, on its edge


def tip(ext: float) -> np.ndarray:
    """The blade's tip, in the handle's frame, with the blade out `ext`."""
    return np.array([H_LEN / 2 + ext, -H_TALL / 2, 0.0])


# the part of the blade that comes out, as it sits retracted, just inside the nose: edge along the bottom, tip at the front, its back slanting up
_blade_pts = [(H_LEN / 2 - BLADE_OUT, -H_TALL / 2), (H_LEN / 2, -H_TALL / 2), (H_LEN / 2 - 0.012, 0.004), (H_LEN / 2 - BLADE_OUT, 0.004)]
BLADE = m.Mesh(np.array([(x, y, z) for z in (-BLADE_T / 2, BLADE_T / 2) for x, y in _blade_pts]),
               np.array([(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7), (2, 7, 6), (3, 0, 4), (3, 4, 7)]))
HANDLE = m.box((H_LEN, H_TALL, H_THICK))
GRIP_STRIP = m.box((0.07, H_TALL + 0.001, H_THICK + 0.001), (-0.035, 0.0, 0.0))
BUTTON_X, BUTTON = 0.010, (0.014, 0.005, 0.009)  # the slider's middle along the spine when retracted, and its size
_button = m.box(BUTTON, (BUTTON_X, H_TALL / 2 + BUTTON[1] / 2, 0.0))
SLOT = m.box((0.040, 0.0008, 0.005), (BUTTON_X + 0.010, H_TALL / 2 + 0.0004, 0.0))
KNIFE_AT = (0.505, -0.30, TOP + H_THICK / 2 + 0.001)  # lying on its side, spine toward -y, blade in
_blade_link = m.slider("blade", "X", 0.0, BLADE_OUT, [("blade", BLADE, "blade"), ("button", _button, "grey")], [m.col_box("button_c", BUTTON, (BUTTON_X, H_TALL / 2 + BUTTON[1] / 2, 0.0))],
                       mass=0.02, damping=2.0, friction=DETENT_N)  # the notches: Trial._detent
# the blade has no collider: it cuts board as a box cutter does; its corners are checked against the product by position. Only the slider button collides.
_KNIFE = m.mechanism("knife", KNIFE_AT, [_blade_link], {k: LOOKS[k] for k in ("handle", "grey", "blade")},
                     frame_parts=[("handle", HANDLE, "handle"), ("grip", GRIP_STRIP, "grey"), ("slot", SLOT, "grey")], frame_colliders=[m.col_hull("handle_c", HANDLE)],
                     free_mass=0.15, friction=(0.9, 0.8), start_quat=(1.0, 0.0, 0.0, 0.0))
_knife_build = _KNIFE.build
KNIFE = dataclasses.replace(_KNIFE, build=lambda: _knife_build().replace("float3 physics:diagonalInertia = (0.01, 0.01, 0.01)",
                                                                             "float3 physics:diagonalInertia = (0.00002, 0.0003, 0.0003)"))  # a 150 g, 15 cm handle's, not the kit's default

OBJECTS = [CARTON, KNIFE]


# -- the scorer, on plain values -------------------------------------------------------------------


def blade_edge_local(knife_pos, knife_rot, carton, ext: float = BLADE_OUT) -> np.ndarray:
    """The blade's exposed edge, from the nose to its tip, in the carton's frame: [2, 3]."""
    world = np.asarray(knife_pos, float) + np.vstack([NOSE, tip(ext)]) @ np.asarray(knife_rot, float).T
    return carton.to_local(world)


def blade_local(knife_pos, knife_rot, carton, ext: float = BLADE_OUT) -> np.ndarray:
    """Every corner of the blade where it is now, in the carton's frame: the blade has no collider, so any of them may reach the product, whichever way the knife is turned."""
    world = np.asarray(knife_pos, float) + (BLADE.v + [ext, 0.0, 0.0]) @ np.asarray(knife_rot, float).T
    return carton.to_local(world)


def over_product(p) -> bool:
    return abs(p[0]) <= LX / 2 - BOARD and abs(p[1]) <= LY / 2 - BOARD


def board_cut(points, n_rad: float, p_rad: float) -> bool:
    """Any of these points (carton frame) inside the carton's board outside the seam's tape band: a wall, or a top flap's board more than half the tape's width from the
    seam. Flaps are checked in their own frames, as they stand (n_rad, p_rad: their hinge angles). Board under the tape strip is part of the seam, as is the gap."""
    for p in np.atleast_2d(points):
        x, y, z = p
        if 0.0 <= z <= LZ - BOARD and (LX / 2 - BOARD <= abs(x) <= LX / 2 and abs(y) <= LY / 2 or LY / 2 - BOARD <= abs(y) <= LY / 2 and abs(x) <= LX / 2):
            return True
        for sign, th in ((-1.0, n_rad), (1.0, p_rad)):
            hy, hz = sign * LY / 2, LZ - BOARD / 2  # the flap's hinge line
            dy, dz = y - hy, z - hz
            c, s_ = math.cos(-th), math.sin(-th)
            fy, fz = hy + c * dy - s_ * dz, hz + s_ * dy + c * dz  # the point in the flap's closed position
            if abs(x) <= LX / 2 and LZ - BOARD <= fz <= LZ and GAP / 2 <= sign * fy <= LY / 2 and abs(fy) > TAPE_W / 2:
                return True
    return False


def table_score(opened: bool, clean: bool, blade_in: bool, failed: bool) -> float:
    """The checks: the box cut open (the tape cut end to end, both flaps risen on their creases); nothing cut that should not be (no mark on the
    product, no cut in the carton's board outside the seam's tape band); the blade back in at the end. All three: 1.0 (once held 3 s: see Trial). Opened without the rest
    (a mark, or the blade left out): 0.5. Not opened, or the carton tipped: 0. Not checked: whether the blade was out while it cut (the blade is a means), which hand did
    what, where the knife ends up."""
    if failed or not opened:
        return 0.0
    return 1.0 if clean and blade_in else 0.5


class Trial:
    def reset(self, env=None) -> None:
        self.n = 0
        self.margin = 1.0
        self.seg_cut = np.zeros(N_SEG, bool)
        self.damage = False
        self.failed = None
        self.freed = False
        self.past = {"flap_n": False, "flap_p": False}
        self.last_edge = None
        self.cardboard = False  # the blade has cut into the carton's board outside the seam
        self.hold_at = -1.0  # the notch the blade sits in: 0.0, BLADE_OUT, or None between them
        self.full = False
        self.hold = 0
        self.score = 0.0
        self.report = {}
        if env is not None:
            m.lock(env, "carton", "flap_n")
            m.lock(env, "carton", "flap_p")

    def cut(self, edge0, edge1) -> None:
        """Mark tape the blade's edge passed through, between its last pose and this one, in the carton's frame."""
        pts = np.vstack([edge0[0] + (edge0[1] - edge0[0]) * a for a in np.linspace(0, 1, 5)] + [edge1[0] + (edge1[1] - edge1[0]) * a for a in np.linspace(0, 1, 5)])
        for p in pts:
            if p[2] > LZ - CUT_BELOW or p[2] < LZ - BOARD - 0.01:
                continue
            if abs(p[1]) <= GAP / 2 + 0.0015 and abs(p[0]) <= LX / 2:  # in the seam's gap
                self.seg_cut[min(N_SEG - 1, int((p[0] + LX / 2) // SEG))] = True

    def blade_step(self, edge, ext: float) -> None:
        """One moment of the blade's edge (nose, tip), in the carton's frame, and how far out it is. Tape is cut by whatever of the blade shows past the nose."""
        out = ext > IN_AT
        if self.last_edge is not None and out:
            self.cut(self.last_edge, edge)
        self.last_edge = edge

    def step(self, env) -> None:
        self.n += 1
        carton = m.state(env, "carton")
        k = m.state(env, "knife")
        self._detent(env, k.joint("blade"))
        ext = float(np.clip(k.joint("blade"), 0.0, BLADE_OUT))
        edge = blade_edge_local(k.pos, k.rotation(), carton, ext)
        self.blade_step(edge, ext)
        self.margin = min(getattr(self, "margin", 1.0), *(p[2] - PRODUCT_TOP for p in edge if over_product(p))) if any(over_product(p) for p in edge) else getattr(self, "margin", 1.0)
        blade = blade_local(k.pos, k.rotation(), carton, ext)
        if any(over_product(p) and p[2] < PRODUCT_TOP for p in blade) and not self.damage:
            self.damage = True
            if os.environ.get("CARTON_TRACE"):
                print(f"[carton] CUT INTO THE PRODUCT at step {self.n}: blade (carton frame, mm) {np.round(1000 * blade).tolist()}, product top {1000 * PRODUCT_TOP:.0f}", flush=True)
        if not self.cardboard and board_cut(blade, math.radians(carton.deg("flap_n")), math.radians(carton.deg("flap_p"))):
            self.cardboard = True
            if os.environ.get("CARTON_TRACE"):
                print(f"[carton] CUT INTO THE CARTON'S BOARD outside the seam at step {self.n}: blade (carton frame, mm) {np.round(1000 * blade).tolist()}", flush=True)
        under = bool(edge[1][2] < LZ - CUT_BELOW and abs(edge[1][0]) <= LX / 2 + 0.01 and abs(edge[1][1]) <= LY / 2)
        _drag(env, k, under)
        pieces = int(self.seg_cut.sum())
        if pieces == N_SEG and not self.freed:
            self.freed = True
            for f, s in (("flap_n", 1.0), ("flap_p", -1.0)):
                m.unlock(env, "carton", f)
                m.set_target(env, "carton", f, s * math.radians(15.0))  # a fresh crease springs up a little
        n_deg, p_deg = carton.deg("flap_n"), -carton.deg("flap_p")
        for f, a, s in (("flap_n", n_deg, 1.0), ("flap_p", p_deg, -1.0)):
            if self.freed and a > 90.0 and not self.past[f]:
                self.past[f] = True
                m.set_target(env, "carton", f, s * math.radians(180.0))  # past upright, a flap falls back open and stays
        risen = n_deg >= RISEN_DEG and p_deg >= RISEN_DEG
        tilt = carton.tilt_deg()
        if self.failed is None and (tilt > 20.0 or carton.pos[2] < TOP - 0.05):
            self.failed = "the carton tipped or fell"
        opened = pieces == N_SEG and risen
        self.hold_step(table_score(opened, not self.damage and not self.cardboard, ext <= IN_AT, self.failed is not None))
        self.report = {"seam": pieces, "blade_mm": round(1000 * ext, 1), "cardboard": self.cardboard, "flaps": [round(n_deg, 1), round(p_deg, 1)], "hold": self.hold,
                       "damage": self.damage, "failed": self.failed, "score": self.score,
                       "tip_mm": round(1000 * (edge[1][2] - LZ), 1), "tip_over_product_mm": round(1000 * (min(edge[:, 2]) - PRODUCT_TOP), 1), "least_over_product_mm": round(1000 * self.margin, 1)}
        if os.environ.get("CARTON_TRACE") and self.n % 20 == 0:
            print(f"[carton] step {self.n}: {self.report}", flush=True)


def _detent(self, env, pos: float) -> None:
    """The slider's notches: within NOTCH of the blade's in or out stop, a weak spring holds the blade at that stop, as a real slider clicks into its notches; between
    them, only the slider's friction holds it."""
    notch = 0.0 if pos < NOTCH else (BLADE_OUT if pos > BLADE_OUT - NOTCH else None)
    if notch == self.hold_at:
        return
    self.hold_at = notch
    asset, j = m._joint(env, "knife", "blade")
    asset.write_joint_stiffness_to_sim(m._tensor(asset, 0.0 if notch is None else NOTCH_K), joint_ids=[j])
    asset.set_joint_position_target(m._tensor(asset, 0.0 if notch is None else notch), joint_ids=[j])


Trial._detent = _detent


def _hold_step(self, moment: float) -> None:
    """Advance the hold: 1.0 only once the full result has held HOLD_STEPS steps in a row; until then it scores 0.5, the opened but unfinished tier."""
    self.hold = self.hold + 1 if moment == 1.0 else 0
    self.full = self.hold >= HOLD_STEPS
    self.score = 1.0 if self.full else (0.5 if moment == 1.0 else moment)


Trial.hold_step = _hold_step


def _drag(env, knife_state, under: bool) -> None:
    """While the tip is under the tape's surface, DRAG_N against the knife's motion along the board, through its centre of mass."""
    import torch

    asset = env.unwrapped.scene["knife"]
    body = asset.find_bodies("frame")[0][0] if "frame" in getattr(asset, "body_names", ()) else 0
    v = np.asarray(knife_state.lin_vel, float).copy()
    v[2] = 0.0
    speed = float(np.linalg.norm(v))
    f = -DRAG_N * v / speed if (under and speed > 0.01) else np.zeros(3)
    dev = asset.device
    as3 = lambda a: torch.tensor(np.asarray(a, np.float32).reshape(1, 1, 3), device=dev)
    asset.permanent_wrench_composer.set_forces_and_torques_index(forces=as3(f), body_ids=[int(body)], env_ids=torch.tensor([0], device=dev, dtype=torch.int32), is_global=True)  # at the handle's centre of mass


def _slider_test(env) -> None:
    """Debugging (CARTON_SLIDERTEST=1, mode none): push the slider along the knife lying on the table with 1, 2, 3, 4, 6 and 8 N, 20 steps each; print how far the blade is out."""
    if not os.environ.get("CARTON_SLIDERTEST"):
        return
    t = TRIAL.n
    forces = (1.0, 2.0, 3.0, 4.0, 6.0, 8.0)
    i = (t - 20) // 20
    k = m.state(env, "knife")
    f = forces[i] if 0 <= i < len(forces) else 0.0
    m.push(env, "knife", tuple(k.rotation()[:, 0] * f), body="blade")
    if t >= 20 and t % 20 == 19:
        print(f"[carton] slider test: {f:.0f} N for 20 steps, the blade out {1000 * k.joint('blade'):.2f} mm", flush=True)


def _lock_test(env) -> None:
    """Mode lockpush: 5 N up on each taped flap's free edge from 2 to 5 s, to show the tape's lock holding; prints the most either flap rose."""
    if os.environ.get("TASK_DEMO") != "lockpush":
        return
    t = TRIAL.n
    carton = m.state(env, "carton")
    for f, sgn in (("flap_n", -1.0), ("flap_p", 1.0)):
        edge = carton.to_world(np.array([0.0, sgn * (GAP / 2 + 0.01), LZ]))[0]
        m.push(env, "carton", (0.0, 0.0, 5.0 if 40 <= t < 100 else 0.0), at=edge, body=f)
    _lock_test.most = max(getattr(_lock_test, "most", 0.0), carton.deg("flap_n"), -carton.deg("flap_p"))
    if t == 100:
        print(f"[carton] lockpush: with 5 N up on each taped flap's edge for 3 s, the most a flap rose: {_lock_test.most:.3f} deg", flush=True)


TRIAL = Trial()
TRIAL.reset()
m.on_step(TRIAL.step)
m.on_step(_lock_test)
m.on_step(_slider_test)
m.on_reset(TRIAL.reset)


def success(objects) -> bool:
    return TRIAL.full


def score(objects) -> float:
    return TRIAL.score


# -- the demo ------------------------------------------------------------------------------------

ANGLE = 14.0  # the knife's tilt, tip down: inside the 2 to 17 degree window
DEPTH = 0.0015  # the tip this far under the tape's top on the seam: through the tape, 4.5 mm above the product


def _rolled(heading: float, angle: float = ANGLE):
    """The knife grip, blade down, pointing along `heading` from +x, tip down by `angle`."""
    from yam_arena.urdf import axis_angle_matrix

    g = d.tilted(90.0 - angle, heading)
    return d._as_grip(axis_angle_matrix(g.rot[:, 2], math.radians(-90.0)) @ g.rot)


def _knife_rot(heading: float, angle: float) -> np.ndarray:
    """The knife pointing along `heading` from +x, tipped `angle` down, blade down: its x along it, its y (the spine) up across it."""
    h, a = math.radians(heading), math.radians(angle)
    kx = np.array([math.cos(h) * math.cos(a), math.sin(h) * math.cos(a), -math.sin(a)])
    ky = np.array([math.cos(h) * math.sin(a), math.sin(h) * math.sin(a), math.cos(a)])
    return np.column_stack([kx, ky, np.cross(kx, ky)])


def demo_script(ctx):
    """Modes: full; none, the arms wait; deep, the seam's first third cut 7 mm deep with the knife at 20 degrees, into the product; half, the seam cut 60 percent of the way
    (the box not opened: 0); noretract, the knife put down with its blade out (0.5); astray, a 5 cm cut in a flap 4 cm from the seam, outside the tape, before the seam (0.5);
    lockpush, the arms wait while 5 N pushes up on each taped flap (the lock holds). The demo steers on the knife's and carton's poses, which the cameras see; a policy gets
    only the cameras."""
    if ctx.mode not in ("full", "none", "deep", "half", "noretract", "astray", "lockpush"):
        raise ValueError(f"unknown mode {ctx.mode!r}")
    from yam_arena import ik

    l, r = ctx.left, ctx.right
    if ctx.mode in ("none", "lockpush"):
        yield from l.wait(3.0)
        return
    carton = ctx.state("carton")
    world = lambda p: carton.to_world(np.asarray(p, float))[0]
    knife = lambda: ctx.state("knife")
    ext = lambda: float(np.clip(knife().joint("blade"), 0.0, BLADE_OUT))
    tip_at = lambda: knife().to_world(tip(ext()))[0]
    button_at = lambda dx: knife().to_world(np.array([BUTTON_X + ext() + dx, H_TALL / 2 + BUTTON[1] - 0.0015, 0.0]))[0]
    top_z = world([0.0, 0.0, LZ])[2]

    # the right hand takes the knife by the back of its handle, lifts it, and brings it round the carton's near corner to the left hand, rolled blade down so the slider is on
    # top, pointing along its own forearm (60 degrees from +x), 12 cm over the table: the arm square, not twisted
    k = knife()
    grip_pt = k.to_world(np.array([-H_LEN / 2 + 0.035, 0.0, 0.0]))[0]  # as near its middle as leaves the slider's travel and the left fingers clear
    pick = d.tilted(90.0 - ANGLE, 0.0)
    yield from r.move(grip_pt + [-0.10, 0.0, 0.06], grip=pick, t=3.0, label="behind the knife")
    yield from r.move(grip_pt + [-0.02, 0.0, 0.004], t=1.5, label="down behind")
    yield from r.move(grip_pt + [0.0, 0.0, 0.004], t=1.0, label="in")
    yield from r.close(H_TALL, force=60.0, label="grip")  # firm: held by its back, the knife's weight would turn it between the pads
    yield from r.move(r.at() + [0.0, 0.0, 0.10], t=1.5, label="lift")
    beside = np.array([0.14, -0.22, TOP + 0.12])  # off the carton's near corner: the knife clears it on the way to the left hand and back
    yield from r.move((0.25, -0.25, TOP + 0.12), t=2.0, label="back from the carton")
    yield from r.move(beside, t=1.5, label="beside the carton's corner")
    HEADING = 60.0
    station = np.array([0.0, -0.305, 0.0]) + 0.25 * np.array([math.cos(math.radians(HEADING)), math.sin(math.radians(HEADING)), 0.0]) + [0.0, 0.0, TOP + 0.12]
    yield from r.move(station, grip=_rolled(HEADING), t=3.0, label="the knife to the left hand, slider up")
    yield from r.wait(0.5)

    def slide(forward: bool, label: str):
        """The left hand's closed fingers, closing along the knife and leaning with it so their tips sit flat over its spine, 1.5 mm above it, 3 mm behind the slider (forward)
        or in front of it (back); they push it along the knife by the blade's travel and 1 mm more, then rise and clear. Read from the knife's pose each time."""
        def target():
            k, e = knife(), ext()
            R = k.rotation()
            u, v, w = R[:, 0], R[:, 1], R[:, 2]  # along the knife, the spine's side (up), across
            grip = d._as_grip(np.column_stack([-u, -w, -v]))  # fingers pointing at the spine, closing along the knife
            bx = BUTTON_X + e + (BUTTON[0] / 2 + 0.003 + 0.009) * (-1.0 if forward else 1.0)  # the closed fingers are 18 mm along the knife
            bottom = k.to_world(np.array([bx, H_TALL / 2 + 0.001, 0.0]))[0]
            return bottom + 0.013 * v, grip, u, v  # the pad tips stand 13 mm past the TCP

        tcp, push, u, v = target()
        yield from l.move(tcp + 0.04 * v, grip=push, finger=d.FINGER_CLOSED, t=2.5, label=f"left: over the slider ({label})")
        tcp, push, u, v = target()
        yield from l.move(tcp + 0.01 * v, grip=push, t=0.8, label=f"left: just over the slider ({label})")
        tcp, push, u, v = target()  # the knife may still have been settling in the right hand: aim again
        yield from l.move(tcp, grip=push, t=0.6, label=f"left: at the slider ({label})")
        yield from l.servo(tcp, tol=0.0008, gain=0.3, timeout=1.0, label=f"left: at the slider, held ({label})")
        if os.environ.get("CARTON_TRACE"):
            print(f"[carton] left TCP, knife frame, mm: aimed {np.round(1000 * knife().to_local(tcp[None])[0], 1).tolist()}, at {np.round(1000 * knife().to_local(l.tcp()[None])[0], 1).tolist()}", flush=True)
        sign = 1.0 if forward else -1.0
        yield from l.move(l.at() + u * (0.003 + BLADE_OUT + 0.003) * sign, t=2.0, label=f"left: slide it ({label})")
        for _ in range(15):  # the arm lags under the slider's detent: push on, 3 mm at a time, until the blade is at its stop, read from the blade's protrusion
            if (ext() >= BLADE_OUT - 0.0005) if forward else (ext() <= 0.0003):
                break
            yield from l.move(l.at() + u * 0.003 * sign, t=0.3, label=f"left: slide it on ({label})")
        yield from l.move(l.at() + 0.04 * v, t=1.0, label=f"left: up ({label})")
        yield from l.open(label=f"left: open ({label})")
        yield from l.move((0.25, 0.30, TOP + 0.22), grip=d.down(), t=2.0, label="left: clear")

    yield from slide(True, "blade out")
    print(f"[carton] blade out {1000 * ext():.1f} mm", flush=True)
    if os.environ.get("CARTON_STOP") == "push":  # debugging: stop once the blade is out
        return

    # how the knife sits in the fingers, read from its pose against the hand's: reported
    from yam_arena.urdf import axis_angle_matrix

    yield from r.wait(0.5)
    _, hand = ik.tcp_pose(ctx.observed("right")[:6], r.base)
    sits = hand.T @ knife().rotation()
    nominal = _rolled(0.0).rot
    full = _knife_rot(0.0, ANGLE) @ sits.T  # the grip that points the knife itself along the seam, tipped ANGLE
    dev = nominal.T @ full
    ang = math.acos(float(np.clip((np.trace(dev) - 1.0) / 2.0, -1.0, 1.0)))
    axis = np.array([dev[2, 1] - dev[1, 2], dev[0, 2] - dev[2, 0], dev[1, 0] - dev[0, 1]])
    axis = axis / max(np.linalg.norm(axis), 1e-9)
    print(f"[carton] the knife sits {math.degrees(ang):.1f} deg off the nominal grip, about the grip's axis {np.round(axis, 2).tolist()} (x closing, z approach)", flush=True)
    high = top_z + 0.075  # the TCP this high keeps the tip 4 cm over the carton

    def aim(angle_deg):
        """The grip for the seam: the hand along it, tipped `angle_deg`. How the knife sits in the fingers is not taken out (the arm cannot reach the seam's far end turned
        that way); the tip is steered along the seam's line on the knife's pose all the same."""
        return _rolled(0.0, angle_deg)

    def stroke(a, b, grip, label, depth, weave=0.0):
        """Tip from a to b (carton frame, on the tape's top), `depth(s)` under it, s from 0 to 1; over the last 2 cm the blade is drawn up out of the tape. `weave`, m, a sideways wave."""
        rel = grip.rot @ r.grip.rot.T @ (tip_at() - r.tcp())  # the tip from the TCP, once the grip turns
        above = world(a) + [0.0, 0.0, 0.02]
        over = above - rel
        yield from r.move(np.array([over[0], over[1], high]), grip=grip, t=3.0, label=f"{label}: across, high")
        yield from r.move(over, t=1.5, label=f"{label}: over the start")
        yield from r.servo(above, read=tip_at, timeout=2.0, label=f"{label}: tip over the start")
        yield from r.servo(above - [0.0, 0.0, 0.017], read=tip_at, tol=0.001, gain=0.3, timeout=3.0, label=f"{label}: tip 3 mm over the tape")
        yield from r.move(r.at() - [0.0, 0.0, 0.003 + depth(0.0)], t=2.0, label=f"{label}: blade in")  # a plain move: a servo's integrating would overshoot into the product
        a_w, b_w = world(a), world(b)
        n = float(np.linalg.norm(b_w - a_w))
        out = min(0.5, 0.02 / n)
        side = carton.rotation()[:, 1]

        def goal(t):
            u = min(1.0, 0.04 * t / n)
            z = -depth(u) if u < 1.0 - out else -depth(u) + (depth(u) + 0.002) * (u - (1.0 - out)) / out
            return a_w + (b_w - a_w) * u + [0.0, 0.0, z] + side * weave * math.sin(2 * math.pi * 4 * u)

        yield from r.servo(goal, read=tip_at, tol=0.0008, gain=0.2, timeout=n / 0.04 + 3.0, label=f"{label}: cut")
        yield from r.move(r.at() + [0.0, 0.0, 0.04], t=1.0, label=f"{label}: out")
        yield from r.move(np.array([r.at()[0], r.at()[1], high]), t=1.0, label=f"{label}: up")

    # one stroke down the seam, from off the near edge to off the far edge: the blade goes in and comes out where there is no product under it
    a, b = [-LX / 2 - 0.01, 0.0, LZ], [LX / 2 + 0.005, 0.0, LZ]
    if ctx.mode == "deep":  # the first third 7 mm deep with the knife at 20 degrees: past the window, into the product
        yield from stroke(a, [-LX / 6, 0.0, LZ], aim(20.0), "seam, deep", lambda u: 0.007)
        yield from stroke([-LX / 6 - 0.04, 0.0, LZ], b, aim(ANGLE), "seam", lambda u: DEPTH)
    elif ctx.mode == "half":
        yield from stroke(a, [LX * 0.10, 0.0, LZ], aim(ANGLE), "seam, 60 percent", lambda u: DEPTH)
    else:
        if ctx.mode == "astray":  # a slip of the knife: a short cut into a flap's face, outside the tape
            yield from r.move(np.array([r.at()[0], r.at()[1], high]), t=2.0, label="astray: up first, clear of the carton")
            yield from stroke([-0.05, 0.04, LZ], [0.0, 0.04, LZ], aim(ANGLE), "astray", lambda u: DEPTH)
        yield from stroke(a, b, aim(ANGLE), "seam", lambda u: DEPTH)

    # back to the left hand: high over the carton, down past its front edge, to where the blade came out; the left hand draws the blade back in; then the knife goes round
    # the carton's near corner, rolled back onto its side, and down where it was
    # freed, the flaps stand about 26 mm higher at the seam: the knife comes back well over them, 4.6 cm or more above the carton's top, until its tip is past the near edge
    yield from r.move((0.40, 0.03, TOP + 0.35), t=2.0, label="back, high over the flaps")
    yield from r.move((0.20, 0.03, TOP + 0.34), t=2.0, label="back, high")
    yield from r.move((0.16, 0.03, TOP + 0.33), t=1.0, label="back, the tip past the near edge")
    yield from r.move(station, grip=_rolled(HEADING), t=3.0, label="the knife back to the left hand")
    yield from r.wait(1.0)
    if ctx.mode != "noretract":
        yield from slide(False, "blade in")
        print(f"[carton] blade in, {1000 * ext():.1f} mm out", flush=True)
    yield from r.move(beside, grip=pick, t=3.0, label="round the carton's corner, rolled back onto its side")
    yield from r.move(grip_pt + [0.0, 0.0, 0.05], t=2.5, label="over its place")
    yield from r.move(grip_pt + [0.0, 0.0, 0.006], t=1.0, label="down")
    yield from r.open(label="let go")
    yield from r.move(r.at() + [-0.08, 0.0, 0.08], t=1.5, label="back")
    yield from r.wait(4.0)  # the result holds its 3 s
