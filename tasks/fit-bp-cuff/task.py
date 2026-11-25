"""Fit a blood-pressure cuff on a training arm: bladder end round first, hook end over it, pressed shut, snug, at the mark.

On the station table a life-size training arm lies palm up on a 3 cm foam arm board, its shoulder end away from the robot
and its hand towards it. The upper arm is 31 cm round (10 cm across); the elbow crease and the brachial artery are marked on
its skin. An adult cuff (16 cm wide, 50 cm long, a 16 x 30 cm bladder for 27-34 cm arms) lies open across the board under the upper arm, its
lower edge 2.5 cm above the crease: its bladder end (the end with the artery index line) towards the robot's right, its
hook (Velcro) end towards the left, both ends lying on the table with a stiffened end hem.

The robot has to wrap the bladder end up round the near side and over the top of the arm, then bring the hook end up round
the far side, over the top and down onto the bladder end's outer face, and press it shut: snug, the index line over the
artery mark, the lower edge 2-3 cm above the crease.

Simulation: the arm and board are fixed shapes; the cuff is a chain of 20 thin rigid plates hinged across its width
(cuffkit), with light bending springs; the Velcro is a rule (the hook plates pressed flat onto the cuff's outer face).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
Offline: python3 task.py   (the scoring rules and the demo's kinematics, no GPU)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import cuffkit  # noqa: E402,F401
import presskit  # noqa: E402
from cuffkit import Plug, cable  # noqa: E402
from scenekit import Shape, UsdBody, on_reset, on_step, robot_links  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0

INSTRUCTION = ("Put the blood-pressure cuff on the arm: wrap the bladder end round first, then the hook end over it, press it "
               "shut, snug, with the index line over the artery mark.")

T = TABLE_TOP[SCENE]
E = float(os.environ.get("CUFF_ENV_SCALE", "1.0"))
# -- the training arm, along x: shoulder away from the robot, hand towards it; palm up --------------------------------
BOARD_H = 0.030  # foam arm board
R = 0.050  # upper arm radius (31.4 cm round)
RF = 0.035  # forearm radius
GAP = float(os.environ.get("ARM_GAP", "0.004"))  # the arm rests just over the cuff (3 mm)
ZA = T + BOARD_H + GAP + R  # upper arm axis height
ZF = T + BOARD_H + GAP + RF
X_ELBOW, X_SHOULDER, X_WRIST = 0.300, 0.560, 0.090
X_CREASE = 0.315  # the elbow crease, marked across the inner (top) arm
ARM_Y = float(os.environ.get("ARM_Y", "0.06"))
SKIN = (0.86, 0.70, 0.60)
# collider: the upper arm as a capsule (its cylinder from x 0.335 to 0.520); visible: a skin mesh 1 mm proud of it
SKIN_MU = float(os.environ.get("SKIN_MU", "1.5"))  # skin-like vinyl against cuff nylon: grippy
UPPER = Shape("upper_arm", pos=(0.4275, ARM_Y, ZA), kind="capsule", radius=R, height=0.185, axis="X", color=SKIN,
              size=(0.29, 0.10, 0.10), friction=(SKIN_MU, SKIN_MU * 0.9))
ARM_MESH = UsdBody("arm_skin", usd="assets/arm.usdc", pos=(0.0, ARM_Y, T), mass=None, material=False, size=(0.6, 0.1, 0.1))
BOARD = Shape("arm_board", pos=(0.30, ARM_Y, T + BOARD_H / 2), size=(0.56, 0.12, BOARD_H), color=(0.30, 0.32, 0.36), friction=(1.0, 0.9))
CREASE = Shape("crease_mark", pos=(X_CREASE, ARM_Y, ZA + R + 0.0005), size=(0.003, 0.06, 0.001), color=(0.15, 0.15, 0.15),
               collide=False)
# -- the cuff: 50 cm x 14 cm, 20 plates of 2.5 cm; along +y from its bladder end (y < 0) to its hook end ---------------
CUFF_L, CUFF_W, CUFF_T, NSEG = 0.50, 0.16, 0.003, 20
SEG = CUFF_L / NSEG
LOW_EDGE = 0.025  # its lower edge above the crease at the start
X_CUFF = X_CREASE + LOW_EDGE + CUFF_W / 2  # the cuff's centre line
NEAR = float(os.environ.get("CUFF_NEAR", "0.24"))  # length of the bladder end, from the arm's underside to its end
Z_CUFF = T + BOARD_H + CUFF_T / 2 + 0.0005
BLADDER = range(0, 12)  # plates 0-11 (0-30 cm): the 16 x 30 cm adult bladder (AHA: for 27-34 cm arms)
INDEX_SEG = 6  # the white index line, at the bladder's middle
HOOK = range(16, 20)  # plates 16-19 (40-50 cm): the hook (Velcro) tab on the inner face
NAVY, NAVY2, WHITE, HOOKC = (0.09, 0.13, 0.32), (0.12, 0.17, 0.40), (0.95, 0.95, 0.95), (0.72, 0.72, 0.68)
COLORS = tuple(WHITE if i == INDEX_SEG else HOOKC if i in HOOK else NAVY if i in BLADDER else NAVY2 for i in range(NSEG))
HEM = Plug(size=(0.016, CUFF_W, 0.016), mass=0.01, color=(0.20, 0.20, 0.22))
CUFF = cable("cuff", start=(X_CUFF, ARM_Y - NEAR, Z_CUFF), yaw=math.pi / 2, length=CUFF_L, radius=CUFF_T / 2,
             segment=SEG, width=CUFF_W, thickness=CUFF_T, seg_colors=COLORS, cable_mass=0.10, bend_stiffness=float(os.environ.get("CUFF_K", "0.0003")),
             bend_damping=float(os.environ.get("CUFF_D", "0.002")), max_bend=1.4, friction=0.9, self_collision=True, start_plug=HEM, end_plug=HEM,
             color=NAVY2)
# -- the artery mark: where the index line lies on a snug cuff (its plate's middle, 13.75 cm from the end, wrapped from
# the arm's underside 6.25 cm round the near side)
R_EFF = R + CUFF_T / 2
ART_ANGLE = (NEAR - (INDEX_SEG + 0.5) * SEG) / R_EFF  # rad from the underside, round the near (-y) side
ARTERY = Shape("artery_mark", pos=(X_CUFF, ARM_Y - (R + 0.0005) * math.sin(ART_ANGLE), ZA - (R + 0.0005) * math.cos(ART_ANGLE)),
               size=(0.22, 0.003, 0.003), color=(0.70, 0.10, 0.10), collide=False)
OBJECTS = [BOARD, UPPER, ARM_MESH, CREASE, ARTERY, CUFF]

# -- scoring -----------------------------------------------------------------------------------------------------------
WRAP_MIN = 2 * math.pi  # the plates on the arm must go round it once
TOUCH_GAP = 0.002  # a hook plate lies on the layer under it within this (plus the cuff's thickness)
ALIGN = math.radians(25.0)  # ... and runs within this angle of it (flat on it, not across or on edge)
FASTEN_N, FASTEN_S = 2, 0.5  # hook plates lying flat on the cuff at once, while pressed, for this long: fastened (latched)
HOOK_FORCE = 2.0  # N: the hook tab (its plates and hem) pressed on the cuff's outer face, measured from PhysX contacts
PAD_FORCE = 2.0  # N: ... while the finger pads push the tab towards the arm with this net force (a pinch cancels)
EMA_S = 0.1  # s: both forces smoothed over this
HOLD_FORCE = 0.2  # N: still fastened at the end = latched, a hook plate still flat on the cuff, and the tab still in contact
# with the cuff (contact force at least this, smoothed)
RELEASE_D = 0.10  # m: hands clear = no finger pad touching the cuff, and every pad this far from every plate and hem
EDGE_LO, EDGE_HI, SKEW_MAX = 0.015, 0.035, 0.020  # lower edge above the crease; skew of the wrapped plates along x
INDEX_ARC = 0.020  # the index line within this arc of the artery mark
SNUG_GAP = 0.006  # mean gap between the inner layer and the skin
TIGHT_MAX = 0.001  # ... and no plate pressed more than this into the skin (not over-tightened)
CLEAR_D = 0.04  # full credit only with both hands' pads this far clear of the arm's surface (or 15 cm along it from the cuff)
PAD_REACH = 0.080  # the pads' centre lies this far beyond the tip links' origins along the hand's approach axis
_STATE: dict = {"touch_s": 0.0, "fastened": False, "clear": False, "t": 0.0, "last": -1.0, "hook_f": 0.0, "pad_f": 0.0,
                "pad_any": 0.0}


def pads(side: dict) -> np.ndarray:
    mid = (side["tip_left"] + side["tip_right"]) / 2
    a = mid - side["gripper"]
    return mid + PAD_REACH * a / max(np.linalg.norm(a), 1e-6)


def hands_clear(pad_points, cuff_points, pad_contact: float) -> bool:
    """No finger pad touching any part of the cuff (contact force under 0.05 N) and every pad at least RELEASE_D from
    every plate centre and hem, and CLEAR_D clear of the arm's surface near the cuff."""
    if pad_contact >= 0.05:
        return False
    c = np.asarray(cuff_points, float)
    for p in pad_points:
        p = np.asarray(p, float)
        if np.min(np.linalg.norm(c - p, axis=1)) < RELEASE_D:
            return False
        if abs(p[0] - X_CUFF) < 0.15 and math.hypot(p[1] - ARM_Y, p[2] - ZA) < R + CUFF_T * 2 + CLEAR_D:
            return False
    return True


def polar(points) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Each plate centre's distance from the upper arm's axis, its angle round it (from the underside, positive round the
    near -y side), and its position along the arm."""
    p = np.asarray(points, float)
    dy, dz = p[:, 1] - ARM_Y, p[:, 2] - ZA
    return np.hypot(dy, dz), np.arctan2(-dy, -dz), p[:, 0]


def on_arm(rho) -> np.ndarray:
    return rho < R + 0.025


def wrap_angle(points) -> float:
    """How far round the arm the cuff goes (rad): the unwrapped angle swept by the run of plates lying on the arm."""
    rho, ang, _ = polar(points)
    m = on_arm(rho)
    idx = np.flatnonzero(m)
    if len(idx) < 2:
        return 0.0
    a = np.unwrap(ang[idx])
    return float(a.max() - a.min())


def tangents(points) -> np.ndarray:
    p = np.asarray(points, float)
    t = np.zeros_like(p)
    t[1:-1] = p[2:] - p[:-2]
    t[0], t[-1] = p[1] - p[0], p[-1] - p[-2]
    return t / np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)


def hook_touching(points) -> int:
    """Hook plates lying flat on the cuff's own outer face: another plate under them (same angle round the arm within
    0.15 rad, nearer the axis) with a gap of at most TOUCH_GAP, the two running within ALIGN of each other."""
    rho, ang, x = polar(points)
    tan = tangents(points)
    n = 0
    for h in HOOK:
        if not on_arm(rho[h]):
            continue
        under = [j for j in range(HOOK.start - 1) if on_arm(rho[j]) and abs(math.remainder(ang[h] - ang[j], 2 * math.pi)) < 0.15
                 and rho[j] < rho[h] - CUFF_T * 0.5]
        if any(rho[h] - rho[j] - CUFF_T <= TOUCH_GAP and abs(float(tan[h] @ tan[j])) >= math.cos(ALIGN) for j in under):
            n += 1
    return n


def press_force(points, pad_vec) -> float:
    """How hard the grippers press the hook tab onto the arm (N): the four finger pads' contact forces on the tab (hook
    plates and the hook end's hem, as one piece; each force points into the body it pushes, see presskit), added, and
    the component of the sum towards the arm's axis. A pinch pushes equally from both sides and adds nothing; pulling
    the tab outward or dragging it along the cuff gives nothing (never negative)."""
    tab = list(HOOK) + [NSEG + 1]
    net = np.asarray(pad_vec, float)[tab].sum(axis=(0, 1))
    c = np.asarray(points, float)[list(HOOK)].mean(axis=0)
    inward = np.array([0.0, ARM_Y - c[1], ZA - c[2]])
    inward /= max(np.linalg.norm(inward), 1e-9)
    return max(0.0, float(net @ inward))


def pressed(n_flat: int, hook_force: float, pad_force: float) -> bool:
    """One control step of fastening: at least FASTEN_N hook plates lie flat on the cuff's outer face, the hook tab is
    in contact with the cuff pressing on it with at least HOOK_FORCE, and the finger pads push the tab towards the arm
    with a net PAD_FORCE (press_force; a pinch cancels) (both from the simulator's contact forces, smoothed over EMA_S). Proximity alone, or the
    tab resting on the cuff under its own weight, does not count."""
    return n_flat >= FASTEN_N and hook_force >= HOOK_FORCE and pad_force >= PAD_FORCE


def status(points, fastened=None, clear=None, contact=None) -> dict:
    rho, ang, x = polar(points)
    m = on_arm(rho)
    inner = [i for i in range(NSEG) if m[i] and not any(m[j] and abs(math.remainder(ang[i] - ang[j], 2 * math.pi)) < 0.12
                                                        and rho[j] < rho[i] - CUFF_T * 0.5 for j in range(NSEG))]
    gaps = [rho[i] - R - CUFF_T / 2 for i in inner]
    gap = float(np.mean(gaps)) if inner else 1.0
    min_gap = float(np.min(gaps)) if inner else 1.0
    xs = x[m] if m.any() else x
    idx_arc = abs(math.remainder(ang[INDEX_SEG] - ART_ANGLE, 2 * math.pi)) * R_EFF if m[INDEX_SEG] else 1.0
    return {"wrap": wrap_angle(points), "touching": hook_touching(points),
            "fastened": _STATE["fastened"] if fastened is None else fastened,
            "contact": _STATE["hook_f"] if contact is None else contact,
            "edge": float(xs.min() - CUFF_W / 2 - X_CREASE), "skew": float(xs.max() - xs.min()), "index_arc": float(idx_arc),
            "gap": gap, "min_gap": min_gap, "clear": _STATE["clear"] if clear is None else clear}


def score_status(st: dict) -> float:
    """0 unless the cuff goes once round the arm. 0.25 wrapped; 0.5 also fastened (the hook tab pressed flat on the
    cuff, latched, and at the end a hook plate still flat on it with the tab in contact, at least HOLD_FORCE); 0.75 also in place (lower edge 1.5-3.5 cm above
    the crease, skew along the arm under 2 cm, index line within 2 cm of the artery mark); 1.0 also snug (mean gap
    between the inner layer and the skin at most 6 mm, and no plate pressed more than 1 mm into the skin: not over-tight)
    with both hands off the cuff (no pad touching it, every pad at least 10 cm from it)."""
    if st["wrap"] < WRAP_MIN:
        return 0.0
    s = 0.25
    if st["fastened"] and st["touching"] >= 1 and st["contact"] >= HOLD_FORCE:
        s = 0.5
        if EDGE_LO <= st["edge"] <= EDGE_HI and st["skew"] <= SKEW_MAX and st["index_arc"] <= INDEX_ARC:
            s = 0.75
            if st["gap"] <= SNUG_GAP and st["min_gap"] >= -TIGHT_MAX and st["clear"]:
                s = 1.0
    return s


def score(objects) -> float:
    return score_status(status(objects["cuff"].points))


def success(objects) -> bool:
    return score(objects) == 1.0


@on_reset
def _reset(environment) -> None:
    _STATE.update(touch_s=0.0, fastened=False, clear=False, t=0.0, last=-1.0, hook_f=0.0, pad_f=0.0, pad_any=0.0)
    presskit.reset()


@on_step
def _watch(environment) -> None:
    try:
        from yam_arena.scene import object_states

        dt = environment.unwrapped.step_dt
        cuff = object_states(environment, ["cuff"])["cuff"]
        n = hook_touching(cuff.points)
        hems = [cuff.start_plug, cuff.end_plug]
        centers = np.vstack([cuff.points] + [h[None] if h is not None else cuff.points[-1:] for h in hems])
        f = presskit.forces(environment, centers)
        k = min(1.0, dt / EMA_S)
        hook_now = float(f["hook_on_cuff"].sum())
        pad_now = press_force(cuff.points, f["pad_vec"])
        _STATE["hook_f"] += k * (hook_now - _STATE["hook_f"])
        _STATE["pad_f"] += k * (pad_now - _STATE["pad_f"])
        _STATE["pad_any"] = float(f["pads"].sum())
        ok = pressed(n, _STATE["hook_f"], _STATE["pad_f"])
        _STATE["touch_s"] = _STATE["touch_s"] + dt if ok else 0.0
        if _STATE["touch_s"] >= FASTEN_S:
            _STATE["fastened"] = True
        links = robot_links(environment)
        pp = [pads(links[s_]) for s_ in ("left", "right")]
        hems = [cuff.start_plug, cuff.end_plug]
        _STATE["clear"] = hands_clear(pp, np.vstack([cuff.points] + [h[None] for h in hems if h is not None]), _STATE["pad_any"])
        _STATE["pads"] = pp
        _STATE["t"] += dt
        if os.environ.get("CUFF_LOG") and _STATE["t"] - _STATE["last"] >= 0.5:
            _STATE["last"] = _STATE["t"]
            st = status(cuff.points)
            rho, ang, x = polar(cuff.points)
            print(f"LOG t={_STATE['t']:.1f} wrap={math.degrees(st['wrap']):.0f}deg touching={n} fastened={st['fastened']} "
                  f"edge={st['edge'] * 1000:.0f}mm skew={st['skew'] * 1000:.0f}mm index={st['index_arc'] * 1000:.0f}mm "
                  f"gap={st['gap'] * 1000:.1f}mm mingap={st['min_gap'] * 1000:.1f}mm clear={st['clear']} hookF={_STATE['hook_f']:.2f}N padF={_STATE['pad_f']:.2f}N padAny={_STATE['pad_any']:.2f}N padL={np.round(_STATE['pads'][0], 3).tolist()} padR={np.round(_STATE['pads'][1], 3).tolist()} score={score_status(st):.2f}", flush=True)
            if os.environ.get("CUFF_LOG") == "2":
                print("SEG " + " ".join(f"{i}:{r * 1000:.0f}/{math.degrees(a):.0f}" for i, (r, a) in enumerate(zip(rho, ang))), flush=True)
    except Exception:
        import traceback

        traceback.print_exc()
        raise


def ideal_points(near=NEAR, extra=0.0) -> np.ndarray:
    """Plate centres of a snug cuff: from its underside contact (NEAR from the bladder end) the bladder end goes round
    the near side, the rest round the far side and on over the bladder end, one cuff thickness out."""
    pts = []
    for i in range(NSEG):
        s = (i + 0.5) * SEG
        if s <= near:
            a, r = (near - s) / R_EFF, R_EFF
        else:
            a = -(s - near) / R_EFF
            r = R_EFF if a > -2 * math.pi + (near / R_EFF) else R_EFF + CUFF_T
        r += extra
        pts.append((X_CUFF, ARM_Y - r * math.sin(a), ZA - r * math.cos(a)))
    return np.array(pts)


def self_test() -> int:
    flat = np.array([(X_CUFF, ARM_Y - NEAR + (i + 0.5) * SEG, Z_CUFF) for i in range(NSEG)])
    snug = ideal_points()
    loose = ideal_points(extra=0.012)
    _STATE["hook_f"] = 1.0  # the tab in contact with the cuff, unless a case says otherwise
    assert score_status(status(snug, fastened=True, clear=True, contact=0.0)) == 0.25  # pressed once, but no longer touching
    cases = {
        "flat at the start": (status(flat, fastened=False), 0.0),
        "snug, never pressed": (status(snug, fastened=False), 0.25),
        "snug, fastened, hands clear": (status(snug, fastened=True, clear=True), 1.0),
        "snug, fastened, a hand still on it": (status(snug, fastened=True, clear=False), 0.75),
        "loose (12 mm off the skin), fastened": (status(loose, fastened=True, clear=True), 0.5 if status(loose, True)["touching"] < 1 else 0.75),
    }
    for name, (st, want) in cases.items():
        got = score_status(st)
        assert abs(got - want) < 1e-9, (name, got, want, st)
    st = status(snug, fastened=True, clear=True)
    assert st["touching"] >= FASTEN_N, st
    assert abs(st["edge"] - LOW_EDGE) < 0.002 and st["skew"] < 0.002 and st["index_arc"] < 0.003, st
    shifted = snug + np.array([0.03, 0.0, 0.0])
    assert score_status(status(shifted, fastened=True, clear=True)) == 0.5  # 3 cm up the arm: out of place
    turned_pts = []
    for p in snug:  # the whole cuff turned 25 degrees round the arm: index off the mark
        dy, dz = p[1] - ARM_Y, p[2] - ZA
        c, s = math.cos(0.45), math.sin(0.45)
        turned_pts.append((p[0], ARM_Y + c * dy - s * dz, ZA + s * dy + c * dz))
    assert score_status(status(np.array(turned_pts), fastened=True, clear=True)) == 0.5
    half = ideal_points()[:12]
    assert wrap_angle(np.vstack([half, flat[12:]])) < WRAP_MIN  # bladder end round, hook end still on the table
    assert R * 2 * math.pi < CUFF_L - 0.10  # the cuff overlaps itself by more than 10 cm on this arm
    squeezed = ideal_points(extra=-0.003)  # pressed 3 mm into the skin all round: over-tight
    assert score_status(status(squeezed, fastened=True, clear=True)) == 0.75
    far = [(X_CUFF, ARM_Y, ZA + R + 0.15)]
    assert hands_clear(far, snug, 0.0)
    assert not hands_clear(far, snug, 0.3)  # a pad still touching the cuff
    assert not hands_clear([tuple(snug[18] + np.array([0.0, 0.0, 0.01]))], snug, 0.0)  # a pad 1 cm off the hook end
    # fastening needs the press itself: flat hook plates, the tab pressed on the cuff, and a gripper pressing it
    c = snug[list(HOOK)].mean(axis=0)
    inward = np.array([0.0, ARM_Y - c[1], ZA - c[2]]); inward /= np.linalg.norm(inward)
    vec = np.zeros((NSEG + 2, 4, 3))  # force each pad puts on each body
    vec[18, 0] = 3.0 * inward  # one pad pushes the tab onto the cuff
    assert abs(press_force(snug, vec) - 3.0) < 1e-6
    vec[NSEG + 1, 2] = 40.0 * inward; vec[NSEG + 1, 3] = -40.0 * inward  # plus a 40 N pinch on the hem: cancels
    assert abs(press_force(snug, vec) - 3.0) < 1e-6
    vec[:] = 0.0; vec[18, 1] = -4.0 * inward  # a pad between the tab and the arm pulls it outward, off the cuff
    assert press_force(snug, vec) == 0.0
    vec[:] = 0.0; vec[19, 0] = 5.0 * np.cross(inward, [1.0, 0.0, 0.0])  # dragging the tab along the cuff: no press
    assert press_force(snug, vec) < 1e-6
    assert pressed(3, 2.5, 3.0)
    assert not pressed(3, 2.5, 0.0)  # tab lying on the cuff, nobody pressing (its own weight)
    assert not pressed(3, 0.3, 3.0)  # pressing the tab, but it is not in loaded contact with the cuff
    assert not pressed(1, 2.5, 3.0)  # pressed, but only one hook plate lies flat on the cuff
    assert hook_touching(snug) >= FASTEN_N
    return len(cases) + 16


# -- the scripted demo ------------------------------------------------------------------------------------------------
def demo():
    """1. The right hand pinches the bladder end's hem, lifts it and carries it up round the near side and over the top
    of the arm, and lets it go on the far side. 2. The left hand pinches the hook end's hem, carries it up round the far
    side, over the top and down the near side, pulls it snug and presses it onto the cuff, then lets go."""
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    import tiltkit

    tiltkit.install()
    NN, GN = -0.022, -0.012
    right, left = [], []
    HEM_Z = T + float(os.environ.get("HEM_GRIP_Z", "0.016"))  # the pinch height on a hem lying on the table
    RP = R + float(os.environ.get("WRAP_CLEAR", "0.022"))  # the hem's path round the arm
    HEM_OFF = float(os.environ.get("HEM_OFF", "0.010"))  # the TCP outside the band's centre line, holding the hem

    def on_circle(ang, r=RP):
        """A point round the upper arm at angle ``ang`` from the underside (positive round the near -y side)."""
        return (X_CUFF, ARM_Y - r * math.sin(ang), ZA - r * math.cos(ang))

    def involute(th, length, side, layer=0.0, out=0.0):
        """The hem's position when the band is wrapped round the arm from its underside up to contact angle ``th``
        (rad) on one side (+1: the near -y side, -1: the far side) and the rest of its ``length`` runs straight off
        the arm along the tangent (an involute: the band stays taut). ``out`` moves it along the outward normal; the
        TCP sits HEM_OFF further out than the hem's centre line."""
        r = R_EFF + layer + HEM_OFF
        free = max(length - R_EFF * th, 0.0)
        py, pz = -side * r * math.sin(th), -r * math.cos(th)
        ty, tz = -side * math.cos(th), math.sin(th)
        ny, nz = -side * math.sin(th), -math.cos(th)
        return (X_CUFF, ARM_Y + py + free * ty + out * ny, ZA + pz + free * tz + out * nz)

    def wait(arm, other):
        d = sum(w.duration for w in other) - sum(w.duration for w in arm)
        if d > 0.05:
            arm.append(W(d, arm[-1].pos, arm[-1].finger, "wait", arm[-1].yaw))

    DEG = float(os.environ.get("WRAP_DEG", "-60"))  # the hand tilted this far forward while it carries a hem round

    def tilt(deg=DEG, joint=False):
        a_ = math.radians(deg)
        Ry = np.array([[math.cos(a_), 0, math.sin(a_)], [0, 1, 0], [-math.sin(a_), 0, math.cos(a_)]])
        return tiltkit.grip(tuple(Ry @ (0, 0, -1)), tuple(Ry @ (0, -1, 0)), joint=joint)

    near_hem = (X_CUFF, ARM_Y - NEAR - HEM.size[0] / 2, HEM_Z)
    # wrapping the bladder end draws the cuff about 3.6 cm round the arm (measured): the hook end lies that much nearer
    FAR_SHIFT = float(os.environ.get("FAR_SHIFT", "0.036"))
    FAR = CUFF_L - NEAR - FAR_SHIFT
    FAR_SLACK = float(os.environ.get("FAR_SLACK", "0.0"))
    far_hem = (X_CUFF, ARM_Y + FAR + HEM.size[0] / 2, HEM_Z)
    right.append(W(2.0, (0.25, -0.20, T + 0.14), O, "start"))
    left.append(W(2.0, (0.25, 0.25, T + 0.14), O, "start"))
    # 1. the bladder end round the near side and over the top
    right += [W(2.0, (near_hem[0], near_hem[1], T + 0.10), NN, "over the bladder end's hem"),
              W(1.5, near_hem, NN, "down round the hem"), W(1.0, near_hem, C, "pinch the hem"),
              W(1.5, (near_hem[0], near_hem[1] + 0.03, T + 0.05), C, "lift it"),
              W(2.0, (near_hem[0], near_hem[1] + 0.06, T + 0.08), C, "tilt the hand", tilt(joint=True))]
    for k, th in enumerate(np.linspace(0.35, NEAR / R_EFF - 0.25, 12)):
        right.append(W(1.6 if k == 0 else 0.8, involute(th, NEAR, +1), C, f"wrap, band taut, contact at {math.degrees(th):.0f} deg", tilt()))
    th_r = NEAR / R_EFF - 0.25
    rel_r = involute(th_r, NEAR, +1, out=0.05)  # straight out from the arm first: the open fingers must not drag the hem back
    right += [W(1.0, right[-1].pos, NN, "let go on the far side", tilt()),
              W(1.2, rel_r, NN, "straight out from the arm", tilt()),
              W(1.0, (rel_r[0], rel_r[1], rel_r[2] + 0.03), O, "up a little", tilt()),
              W(2.5, (0.25, -0.20, T + 0.14), O, "clear", tilt(joint=True))]
    # 2. the hook end round the far side, over the top, down the near side, snug and pressed
    wait(left, right)
    left += [W(2.0, (far_hem[0], far_hem[1], T + 0.10), NN, "over the hook end's hem"),
             W(1.5, far_hem, NN, "down round the hem"), W(1.0, far_hem, C, "pinch the hem"),
             W(1.5, (far_hem[0], far_hem[1] - 0.05, T + 0.05), C, "lift it"),
             W(2.0, (far_hem[0], far_hem[1] - 0.10, T + 0.08), C, "tilt the hand", tilt(joint=True))]
    for k, th in enumerate(np.linspace(0.35, FAR / R_EFF - 0.25, 14)):
        slack = FAR_SLACK * (1 - k / 13)  # loose at first, so lifting the hook end does not drag the cuff round
        left.append(W(1.6 if k == 0 else 0.8, involute(th, FAR + slack, -1, layer=CUFF_T), C,
                      f"wrap, band taut, contact at {-math.degrees(th):.0f} deg", tilt()))
    PRESS = float(os.environ.get("PRESS_IN", "0.016"))
    pr = involute(FAR / R_EFF - 0.25, FAR, -1, layer=CUFF_T, out=-PRESS)
    left += [W(1.0, pr, C, "pull snug and press the tab onto the cuff", tilt()),
             W(1.5, pr, C, "hold it pressed", tilt()),
             W(1.0, pr, NN, "let go", tilt()),
             W(1.2, involute(FAR / R_EFF - 0.25, FAR, -1, layer=CUFF_T, out=0.05), NN,
               "straight out from the arm", tilt()),
             W(1.2, tuple(np.add(involute(FAR / R_EFF - 0.25, FAR, -1, layer=CUFF_T, out=0.05), (0, 0, 0.03))), O,
               "up a little", tilt()),
             W(2.0, (0.25, 0.25, T + 0.14), O, "clear", tilt(joint=True))]
    if os.environ.get("DEMO_MODE", "simple") == "simple":
        # The simple plan: one fixed (tilted) hand orientation throughout. Right hand: pinch the bladder end's hem, lift it
        # straight up above the arm, carry it left over the top, lower it on the far side and let go, then move further
        # left (clear of the cuff) and away. Left hand: pinch the hook end's hem, lift it, carry it right over the top,
        # lower it onto the bladder end on the near side and press it there (the hook-and-loop), let go, and leave right.
        right, left = [], []
        g = tilt()
        PO = O if os.environ.get("PICK_OPEN") == "1" else NN  # the opening while going down round a hem
        top_z = ZA + R + float(os.environ.get("OVER_Z", "0.05"))
        OVERSHOOT = float(os.environ.get("OVERSHOOT", "0.03"))  # carry the hook end this much past the near side, taut
        HOLD = os.environ.get("HOLD", "0") == "1"  # the right hand holds the bladder end down while the left wraps
        right += [W(2.0, (0.25, -0.20, T + 0.14), O, "start", tilt(joint=True)),
                  W(2.0, (near_hem[0], near_hem[1], T + 0.10), PO, "over the bladder end's hem", g),
                  W(1.5, near_hem, PO, "down round the hem", g), W(1.0, near_hem, C, "pinch the hem", g),
                  W(2.0, (near_hem[0], near_hem[1], top_z), C, "lift it straight up", g),
                  W(2.5, (X_CUFF, ARM_Y + 0.03, top_z), C, "carry it left over the top", g),
                  W(1.5, (X_CUFF, ARM_Y + R + 0.012, ZA + 0.01), C, "lay it down the far side", g),
                  W(1.0, (X_CUFF, ARM_Y + R + 0.012, ZA + 0.01), NN, "let go", g),
                  W(1.2, (X_CUFF, ARM_Y + R + 0.06, ZA + 0.02), NN, "further left, clear of the cuff", g),
                  W(1.5, (X_CUFF, ARM_Y + R + 0.06, top_z - 0.02), O, "up", g)]
        hold_p = (X_CUFF, ARM_Y - R - 0.004, ZA - 0.035)  # low on the near side, below where the hook tab lands
        if HOLD:
            right += [W(2.5, (X_CUFF, ARM_Y - R - 0.05, ZA - 0.02), C, "round to the near side", tilt(joint=True)),
                      W(1.5, hold_p, C, "hold the bladder end against the arm", tilt())]
        else:
            right.append(W(2.5, (0.25, -0.20, T + 0.14), O, "clear", tilt(joint=True)))
        far_h = (X_CUFF, ARM_Y + FAR + HEM.size[0] / 2, HEM_Z)
        left.append(W(2.0, (0.25, 0.25, T + 0.14), O, "start", tilt(joint=True)))
        wait(left, right)
        PRESS2 = float(os.environ.get("PRESS2", "0.020"))
        lay = (X_CUFF, ARM_Y - R - 0.016, ZA + 0.015)  # the hem on the near side, outside the bladder end
        NEG = os.environ.get("CUFF_NEGATIVE", "")  # scorer checks: "nopress" lays the tab on and lets go; "held" never lets go
        left += [W(2.0, (far_h[0], far_h[1], T + 0.10), PO, "over the hook end's hem", g),
                 W(1.5, far_h, PO, "down round the hem", g), W(1.0, far_h, C, "pinch the hem", g),
                 W(2.0, (far_h[0], far_h[1], top_z), C, "lift it straight up", g),
                 W(2.5, (X_CUFF, ARM_Y - 0.03, top_z), C, "slide it right over the top", g),
                 W(1.5, (X_CUFF, lay[1] - OVERSHOOT, top_z - 0.02), C, "further right, pulling it taut", g),
                 W(1.5, lay, C, "lay it down the near side, over the bladder end", g),
                 W(1.0, (lay[0], lay[1] + PRESS2, lay[2]), C, "press the hook tab on", g),
                 W(1.5, (lay[0], lay[1] + PRESS2, lay[2]), C, "hold it pressed", g),
                 W(1.0, (lay[0], lay[1] + PRESS2, lay[2]), NN, "let go", g),
                 W(1.2, (lay[0], lay[1] - 0.05, lay[2] + 0.01), NN, "further right, clear of the cuff", g),
                 W(1.5, (lay[0], lay[1] - 0.05, top_z - 0.02), O, "up", g),
                 W(2.5, (0.25, 0.25, T + 0.14), O, "clear", tilt(joint=True))]
        if NEG == "nopress":  # carry it taut over the near side and let go there, above the cuff: the hand never comes
            # down on the tab; the band's tension and gravity lay it on the cuff
            i = next(k for k, w in enumerate(left) if w.label == "lay it down the near side, over the bladder end")
            taut = left[i - 1].pos
            left[i:] = [W(1.0, taut, NN, "let go, taut above the near side", g),
                        W(1.5, (taut[0], taut[1] - 0.06, taut[2] + 0.04), O, "away from the cuff", g),
                        W(2.5, (0.25, 0.25, T + 0.14), O, "clear", tilt(joint=True))]
        if NEG == "held":  # press it on and keep holding it to the end
            i = next(k for k, w in enumerate(left) if w.label == "let go" and k > 10)
            left[i:] = [W(20.0, left[i - 1].pos, C, "keep holding", left[i - 1].yaw)]
        wait(right, left)
        if HOLD:
            right += [W(1.5, (X_CUFF, ARM_Y - R - 0.06, ZA - 0.03), C, "let the bladder end go", tilt()),
                      W(2.5, (0.25, -0.20, T + 0.14), O, "clear", tilt(joint=True))]
            wait(left, right)
        return {"left": left, "right": right}
    wait(right, left)
    return {"left": left, "right": right}


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    print(f"artery mark at {math.degrees(ART_ANGLE):.0f} deg round the near side; cuff lower edge at x {X_CUFF - CUFF_W / 2:.3f}")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
