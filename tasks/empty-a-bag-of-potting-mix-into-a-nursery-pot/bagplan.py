"""Geometry of the bag, the pot and the layout, and the scripted bimanual pour. NumPy only (no Isaac Sim).

Frames: world x forward from the arms, y left, z up (m). The bag's own frame has its origin at the middle of the
inside of its floor, z along the bag from bottom to mouth, x across its depth (the front panel, facing the arms, at
-x), y across its width. The bag starts standing upright, unrotated.

The demo plans the bag as if it were held straight between the two hands: one path of the bag frame's pose, each
hand's TCP target that pose times its fixed grip. The film itself is limp (see clothbag, bagkit), so the bottom hand
also turns its end further over than the mouth, and shakes it.
"""

from __future__ import annotations

import math

import numpy as np

import fastik

TOP = 0.7475  # the station's table top
BASES = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}
READY = (0.0, 1.0, 1.0, -0.6, 0.0, 0.0)
DT = 0.05  # 20 Hz

# -- the bag: one film mesh (see clothbag) -------------------------------------------------------------------------
import clothbag as cb  # noqa: E402

W, D = cb.WIDTH, cb.DEPTH  # filled cross-section across y and x
H = cb.HEIGHT  # bottom to the cut mouth
FILM_MASS = 0.050  # the simulated film (see DESIGN.md: a real bag weighs about 12 g)
REST = 0.003  # the film's contact rest offset: it stands this far above the bench

# -- the mix (damp peat/bark/perlite potting mix, as small angular rigid grains) -------------------------------------
GRAIN_R = (0.0045, 0.0060)  # bounding radii: grains 9 to 12 mm across
GRAIN_DENSITY = 925.0  # kg/m^3 of a grain; set so the settled fill weighs MIX_KG (see prepare_mix.py)
MIX_KG = 0.80
MIX_FRICTION = (1.0, 0.9)  # grain on grain (peat mix stands at about 45 degrees)

# -- the pot (a black 1-gallon nursery pot) --------------------------------------------------------------------------
POT_R_TOP, POT_R_BASE, POT_H = 0.0825, 0.0675, 0.160  # outside radii and height
POT_WALL = 0.002
POT_RIM = 0.006  # rolled rim, outward
POT_MASS = 0.070

# -- layout ----------------------------------------------------------------------------------------------------------
BAG_AT = np.array([0.40, 0.125, TOP + REST])  # bag frame origin (middle of its floor) at the start
POT_AT = np.array([0.40, -0.135, TOP])  # pot base centre on the bench
SET_DOWN_AT = np.array([0.40, 0.140, TOP + REST])  # where the emptied bag is stood back down


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


# -- grips: TCP frames in the bag frame (columns: gripper x = finger closing axis, y, z = approach) ------------------
# Both hands pinch a front corner tab (a gathered corner seal, turned forward), closing across it and approaching from
# in front of it. The right hand takes the right tab just under the mouth, the left hand the left tab low on the filled
# part, its approach tilted down.
PAD_SHORT = 0.003  # the pad tips stop this far outside the film's corner
PAD_TIP = 0.013  # pad tips are this far past the TCP along the approach
BOTTOM_TILT = math.radians(15.0)  # the bottom hand approaches this far down from level
FLIP = {"right": False, "left": False}  # which finger goes on which side of the tab


def _pinch(normal, inward, phi: float, flip: bool) -> np.ndarray:
    """TCP rotation: closing axis (x) along +-normal; approach (z) = inward turned by phi about the normal."""
    n = np.asarray(normal, float)
    z = axis_rot(n, phi) @ np.asarray(inward, float)
    x = -n if flip else n
    return np.column_stack([x, np.cross(z, x), z])


def axis_rot(u, th: float) -> np.ndarray:
    u = np.asarray(u, float) / np.linalg.norm(u)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(th) * k + (1 - math.cos(th)) * k @ k


def grips_local() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Each arm's TCP (position, rotation) in the bag frame: on its tab's bisector, at the middle of its height."""
    out = {}
    for arm, tab, tilt in (("right", "mouth_tab", 0.0), ("left", "bottom_tab", BOTTOM_TILT)):
        d = cb.tab_out(tab)
        normal = np.cross([0.0, 0.0, 1.0], d)  # across the tab
        pos = cb.tab_corner(tab) + d * (PAD_SHORT + PAD_TIP)
        rot = _pinch(normal, -d, 0.0, FLIP[arm])
        if tilt:  # turn the approach down about the closing axis
            down = _pinch(normal, -d, tilt, FLIP[arm])
            rot = down if down[2, 2] < 0 else _pinch(normal, -d, -tilt, FLIP[arm])
        out[arm] = (pos, rot)
    return out


def tcp_targets(origin, rotation) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    return {arm: (origin + rotation @ p, rotation @ r) for arm, (p, r) in grips_local().items()}


# -- the pour --------------------------------------------------------------------------------------------------------
TIP = math.radians(118.0)  # the mouth hand turns the mouth this far over (its wrist roll limit is 120 degrees)
LIFT = 0.22
TRAVEL_TIP = 0.30  # the share of the tip made while the mouth travels to the pot
APPROACH_ABOVE = 0.12  # the mouth comes in this far above its pour point, then down
APPROACH = 0.10  # the hands line up this far in front of the tabs, then slide on
LIP_CLEAR = 0.045  # the mouth's lower lip this far above the pot's rim while pouring
POUR_BACK = 0.060  # and this far from the pot's axis toward the bag's side: mix leaves the lip moving away from it
SHAKES = int(__import__("os").environ.get("BAG_SHAKES", 22))  # BAG_SHAKES=0: the same plan without shaking
SHAKE_IN = 0.0  # each shake moves the bottom this far along the bag
SHAKE_UP = 0.05  # and lifts it this far (across the line between the hands), then jerks it back down
SHAKE_STEPS = (4, 2)  # control steps up, steps down
# The bottom hand's swing between its two poses bows out this way (weights: away from the mouth hand, up) and this far.
DETOUR = (1.0, 1.0, 0.05)
# When the bottom is turned over, the limp bag pivots and carries its mouth toward the arms and the bag's side; both
# hands move this much (world, m) while it is turned over to keep the mouth over the pot.
TURN_SHIFT = (0.045, -0.025, 0.0)
SPACING_MAX = 0.128  # the shakes never take the hands further apart than this (the film between them stays slack)
BOTTOM_TIP = math.radians(140.0)  # the bottom end's turn while it empties
BOTTOM_YAW = math.radians(-60.0)
BOTTOM_SPIN = math.radians(30.0)


def bag_at_tip(tip: float, f_move: float, above: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    """Bag pose while tipping: turned ``tip`` about world x (mouth toward the pot, -y); the mouth's centre moves
    from above the standing bag (lifted) to the pour point as ``f_move`` goes 0 to 1."""
    r = rx(tip)
    start = BAG_AT + np.array([0.0, 0.0, LIFT + H])
    lip_drop = (W / 2) * math.sin(TIP)  # the lower lip is this far below the mouth's centre at full tip
    lip_y = (W / 2) * math.cos(TIP)  # and this far along y (toward the bag's bottom: +y)
    pour = np.array([POT_AT[0], POT_AT[1] + POUR_BACK + lip_y, TOP + POT_H + POT_RIM + LIP_CLEAR + lip_drop])
    mouth = start + (pour + [0.0, 0.0, above] - start) * f_move
    return mouth - r @ np.array([0.0, 0.0, H]), r


def slerp(r0: np.ndarray, r1: np.ndarray, f: float) -> np.ndarray:
    v = fastik._rotvec(r0.T @ r1)
    return r0 @ axis_rot(v, f * np.linalg.norm(v)) if np.linalg.norm(v) > 1e-9 else r0.copy()


TRAVEL_END = 0.55  # share of the tip phase spent travelling to over the pot (the rest: coming down while tipping)


def _tip_schedule(f: float) -> float:
    """While the mouth travels to the pot the bag turns only part way (the mix stays in); it turns the rest of the
    way while the mouth comes down over the pot. (Coming down first, with the bag only part way over, lowers its
    hanging bottom onto the pot's rim.)"""
    a = TRAVEL_TIP
    return a * smooth(min(1.0, f / TRAVEL_END)) + (1 - a) * smooth(max(0.0, (f - TRAVEL_END) / (1 - TRAVEL_END)))


def _travel(f: float) -> float:
    return smooth(min(1.0, f / TRAVEL_END))


def _above(f: float) -> float:
    return APPROACH_ABOVE * (1.0 - smooth(max(0.0, (f - TRAVEL_END) / (1 - TRAVEL_END))))


def smooth(f: float) -> float:
    f = min(1.0, max(0.0, f))
    return f * f * (3 - 2 * f)


def solve_weighted(pos, rot, q0, base, w_rot: float = 0.3, iters: int = 120):
    """IK that puts the TCP at ``pos`` and turns it as close to ``rot`` as the joint limits let it (rotation error
    weighted by ``w_rot``): for the bottom hand, whose hold on the limp film need not keep the bag straight."""
    q = np.array(q0, float)
    for _ in range(iters):
        p, r, J = fastik.fk(q, base, jac=True)
        e = np.r_[np.asarray(pos) - p, w_rot * fastik._rotvec(rot @ r.T)]
        Jw = J.copy()
        Jw[3:] *= w_rot
        dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + 0.02**2 * np.eye(6), e)
        n = np.abs(dq).max()
        if n > 0.3:
            dq *= 0.3 / n
        q = np.clip(q + dq, fastik.LOWER + 0.01, fastik.UPPER - 0.01)
    p, r = fastik.fk(q, base)
    return q, float(np.linalg.norm(np.asarray(pos) - p)), float(np.linalg.norm(fastik._rotvec(rot @ r.T)))


SEEDS = [np.array(s) for s in __import__("itertools").product(
    (-0.6, 0.6), (0.8, 1.6), (0.6, 1.4), (-0.8, 0.8), (-1.0, 1.0), (-1.6, 0.0, 1.6))]


def start_grip_q(arm: str) -> np.ndarray:
    """The grip configuration on the standing bag whose wrist roll leaves the most room to tip (multi-start IK)."""
    p, r = tcp_targets(BAG_AT, np.eye(3))[arm]
    best = None
    for s in SEEDS:
        q, pe, re = fastik.solve(p, r, s, BASES[arm], iters=150)
        if pe < 1e-4 and re < 1e-3:
            room = 2.0944 - q[5] if arm == "right" else min(np.minimum(q - fastik.LOWER, fastik.UPPER - q))
            if best is None or room > best[1]:
                best = (q, room)
    return best[0]


class Plan:
    """Joint targets for both arms at 20 Hz: per step {arm: (q, finger 0 open .. 1 closed, label)}."""

    def __init__(self):
        self.steps: list[dict] = []
        self.q = {arm: np.array(READY, float) for arm in BASES}
        self.grip = {arm: 0.0 for arm in BASES}
        self.worst = {arm: 0.0 for arm in BASES}

    def add(self, label: str, n: int, targets, grip=None, exact=("right", "left"), w_rot: float = 0.3):
        """``targets(f)`` -> {arm: (pos, rot)} for f in (0, 1]; each arm follows by IK from where it is."""
        g0 = dict(self.grip)
        for k in range(1, n + 1):
            f = k / n
            row = {}
            for arm, (pos, rot) in targets(f).items():
                if arm in exact:
                    q, pe, re = fastik.solve(pos, rot, self.q[arm], BASES[arm], iters=80)
                    self.worst[arm] = max(self.worst[arm], pe + 0.05 * re)
                else:
                    q, pe, re = solve_weighted(pos, rot, self.q[arm], BASES[arm], w_rot=w_rot)
                    self.worst[arm] = max(self.worst[arm], pe)
                self.q[arm] = q
            for arm in BASES:
                if grip is not None:
                    self.grip[arm] = g0[arm] + (grip.get(arm, g0[arm]) - g0[arm]) * smooth(f)
                row[arm] = (self.q[arm].copy(), self.grip[arm], label)
            self.steps.append(row)

    def joint_move(self, label: str, n: int, goal: dict):
        q0 = {arm: self.q[arm].copy() for arm in BASES}
        for k in range(1, n + 1):
            s = smooth(k / n)
            row = {}
            for arm in BASES:
                self.q[arm] = q0[arm] + (goal.get(arm, q0[arm]) - q0[arm]) * s
                row[arm] = (self.q[arm].copy(), self.grip[arm], label)
            self.steps.append(row)


def steps_for(seconds: float) -> int:
    return max(1, round(seconds / DT))


def plan() -> Plan:
    """The scripted bimanual demo: grip both tabs, lift, tip the mouth over the pot, shake the bottom, carry the empty
    bag back and lay it on the bench, let go."""
    pl = Plan()
    grips = grips_local()
    qg = {arm: start_grip_q(arm) for arm in BASES}
    back = {}
    for arm, (p, r) in tcp_targets(BAG_AT, np.eye(3)).items():
        back[arm] = fastik.solve(p - APPROACH * r[:, 2] + [0, 0, 0.03], r, qg[arm], BASES[arm], iters=150)[0]
    pl.joint_move("reach in front of the corner tabs", steps_for(3.0), back)

    def at_bag(origin, rot, approach=0.0, up=0.0):
        return {arm: (origin + rot @ p - approach * (rot @ r)[:, 2] + [0, 0, up], rot @ r)
                for arm, (p, r) in grips.items()}

    pl.add("line up with the tabs", steps_for(1.0), lambda f: at_bag(BAG_AT, np.eye(3), APPROACH, 0.03 * (1 - smooth(f))))
    pl.add("hold still in front of the tabs", steps_for(1.0), lambda f: at_bag(BAG_AT, np.eye(3), APPROACH))
    pl.add("slide onto the tabs", steps_for(3.0), lambda f: at_bag(BAG_AT, np.eye(3), APPROACH * (1 - smooth(f))))
    pl.add("hold still on the tabs", steps_for(0.5), lambda f: at_bag(BAG_AT, np.eye(3)))
    pl.add("pinch both tabs", steps_for(1.0), lambda f: at_bag(BAG_AT, np.eye(3)), grip={"left": 1.0, "right": 1.0})
    pl.add("lift the bag", steps_for(2.0), lambda f: at_bag(BAG_AT + [0, 0, LIFT * smooth(f)], np.eye(3)))
    exact_r = ("right",)
    pl.add("tip the mouth over the pot", steps_for(5.0),
           lambda f: at_bag(*bag_at_tip(TIP * _tip_schedule(f), _travel(f), _above(f))), exact=exact_r)
    origin, rot = bag_at_tip(TIP, 1.0)
    rigid = at_bag(origin, rot)
    mouth = origin + rot @ np.array([0.0, 0.0, H])
    # The film is limp: the bottom hand turns its (mix-filled) end further over than the mouth hand can turn the
    # mouth, so the mix in it runs down into the mouth. Its end then points down toward the mouth.
    r_bot = rz(BOTTOM_YAW) @ rx(BOTTOM_TIP) @ rz(BOTTOM_SPIN)
    g_pos, g_rot = grips["left"]
    steep = (mouth - r_bot @ np.array([0.0, 0.0, H]) + r_bot @ g_pos, r_bot @ g_rot)

    away = rigid["left"][0] - rigid["right"][0]
    up = np.array([0.0, 0.0, 1.0])
    detour_dir = DETOUR[0] * away / np.linalg.norm(away) + DETOUR[1] * up
    detour_dir = detour_dir / max(np.linalg.norm(detour_dir), 1e-9)

    def bottom(f, extra=np.zeros(3), detour=False):
        """The bottom hand between its rigid-bag pose (f = 0) and the steep pose (f = 1); ``detour`` swings it out
        on the way so the two grippers do not meet."""
        t = dict(rigid)
        s_ = smooth(f)
        bump = DETOUR[2] * math.sin(math.pi * s_) * detour_dir if detour else 0.0
        t["left"] = (rigid["left"][0] + (steep[0] - rigid["left"][0]) * s_ + extra + bump,
                     slerp(rigid["left"][1], steep[1], s_))
        shift = np.asarray(TURN_SHIFT) * s_  # both hands, so the mouth stays over the pot as the bag pivots
        t["left"] = (t["left"][0] + shift, t["left"][1])
        t["right"] = (rigid["right"][0] + shift, rigid["right"][1])
        return t

    pl.add("let it pour", steps_for(1.0), lambda f: rigid, exact=exact_r)
    pl.add("turn the bottom right over", steps_for(3.0), lambda f: bottom(f, detour=True), exact=exact_r, w_rot=0.3)
    pl.add("let it run", steps_for(1.0), lambda f: bottom(1.0), exact=exact_r, w_rot=0.3)
    for k in range(SHAKES):
        def push(f, out=False):
            s_ = smooth(f)
            d = (1 - s_) if out else s_
            base = bottom(1.0)
            gap = base["left"][0] - base["right"][0]
            u = np.array([0, 0, 1.0]) - (gap @ [0, 0, 1.0]) / (gap @ gap) * gap  # up, across the hands' line
            t = bottom(1.0, d * SHAKE_UP * u / np.linalg.norm(u))
            gap = t["left"][0] - t["right"][0]
            if np.linalg.norm(gap) > SPACING_MAX:  # slide the bottom hand back toward the mouth hand
                t["left"] = (t["right"][0] + gap * (SPACING_MAX / np.linalg.norm(gap)), t["left"][1])
            return t
        pl.add(f"shake {k + 1}: bottom up", SHAKE_STEPS[0], push, exact=exact_r, w_rot=0.3)
        pl.add(f"shake {k + 1}: jerk it down", SHAKE_STEPS[1], lambda f: push(f, True), exact=exact_r, w_rot=0.3)
    pl.add("bottom back level with the mouth", steps_for(2.5), lambda f: bottom(1.0 - f, detour=True), exact=exact_r,
           w_rot=0.3)
    pl.add("let the last of it fall", steps_for(1.5), lambda f: at_bag(origin, rot), exact=exact_r)
    down = SET_DOWN_AT + np.array([0, 0, LIFT])
    pl.add("turn the empty bag back up", steps_for(4.0),
           lambda f: at_bag(*_back_up(origin, rot, down, smooth(f))), exact=exact_r)
    pl.add("stand it on the bench", steps_for(2.0),
           lambda f: at_bag(down - [0, 0, LIFT * smooth(f)] + [0, 0, 0.004], np.eye(3)), exact=exact_r)
    pl.add("let go", steps_for(1.0), lambda f: at_bag(SET_DOWN_AT + [0, 0, 0.004], np.eye(3)),
           grip={"left": 0.0, "right": 0.0}, exact=exact_r)
    pl.add("back off", steps_for(2.0),
           lambda f: at_bag(SET_DOWN_AT + [0, 0, 0.004], np.eye(3), 0.07 * smooth(f)), exact=exact_r)
    pl.joint_move("back to ready", steps_for(2.5), {arm: np.array(READY, float) for arm in BASES})
    return pl


def _back_up(origin, rot, down, f):
    """From the pour pose back to standing upright over ``down`` (the mouth's centre path is a straight line)."""
    tip = TIP * (1 - f)
    r = rx(tip)
    m0 = origin + rot @ np.array([0, 0, H])
    m1 = down + np.array([0, 0, H])
    m = m0 + (m1 - m0) * f
    return m - r @ np.array([0, 0, H]), r


if __name__ == "__main__":
    import time

    t = time.time()
    pl = plan()
    print(f"{len(pl.steps)} steps ({len(pl.steps) * DT:.1f} s), planned in {time.time() - t:.1f} s")
    print("worst IK error (m):", {a: round(v, 4) for a, v in pl.worst.items()})
    for arm in BASES:
        q = np.array([s[arm][0] for s in pl.steps])
        print(arm, "largest joint step (rad):", np.round(np.abs(np.diff(q, axis=0)).max(0), 3),
              "closest to a limit:", round(float(np.minimum(q - fastik.LOWER, fastik.UPPER - q).min()), 3))
    last = None
    for i, s in enumerate(pl.steps):
        if s["left"][2] != last:
            last = s["left"][2]
            ps = {a: fastik.fk(s[a][0], BASES[a])[0] for a in BASES}
            print(f"{i:4d} {last:40s} spacing {np.linalg.norm(ps['left'] - ps['right']):.3f}")
