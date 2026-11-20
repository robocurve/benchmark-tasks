"""The scoring rules, as plain functions of poses and latched events, so that they can be checked without the simulator.

Measured as a person would with a ruler laid across the new pot's rim at four points (front, back, left, right): depth
is the mean gap from the rim down to the root ball's top edge at those points, tilt the largest difference between any
two of them, and off-centre half the larger difference between opposite gaps from the ball's top edge to the pot wall.
"""

from __future__ import annotations

import math

import numpy as np

import cactusassets as ca

# limits (m)
FULL = dict(depth=(0.010, 0.020), off=0.004, tilt=0.008)
PART = dict(depth=(0.005, 0.025), off=0.007, tilt=0.015)
RISEN = 0.01           # the ball has risen this far in the old pot (0.25; and the zero when the pot was not squeezed)
OLD_POT_LIFT = 0.02    # the old pot lifted this far off the table: zero
DROP = 0.03            # the cactus and ball fell this far unsupported: zero
LIE_OVER = math.radians(30.0)
NEW_POT_MOVE = 0.02    # m
NEW_POT_TIP = math.radians(15.0)
REST_SPEED = 0.02      # m/s: the cactus and ball are resting
HOME_TOL = 0.03        # m: each gripper's TCP within this of its start pose
CLEAR = 0.03           # m: and every fingertip this far from the surface of each object
HOME_STEPS = 20        # ... for 1 s running (20 control steps)
BREAK_GAP = 0.003      # m: the cactus and its ball this far from where the bond holds them: torn apart
ON_LAYER = 0.004       # m: the ball's base within this of the base layer's top, in the new pot

FLEX = ca.FLEX_AT_X     # squeezed: two opposite wall panels each moved in this far (3 mm), 4.5 cm up ...
SQUEEZE_STEPS = 20     # ... for 1 s running (20 control steps at 20 Hz): the matted roots let go of the wall
TOUCH_STEPS = 2        # robot contact with the cactus outside its collar, or a finger on the root ball, this many
                       # control steps running: zero

TIERS = (1.0, 0.75, 0.5, 0.25, 0.0)


def collar_point(local) -> bool:
    """A contact point, in the cactus's frame (origin on the soil face, z up), lies on the collar's outside or its
    edges rather than on the cactus inside it."""
    r = math.hypot(local[0], local[1])
    return r > ca.COLLAR_R_IN + 0.0008 and ca.COLLAR_Z0 - 0.002 < local[2] < ca.COLLAR_Z1 + 0.002


def rot(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_of(q_xyzw) -> float:
    """Angle (rad) between a body's z axis and the vertical."""
    return math.acos(max(-1.0, min(1.0, rot(q_xyzw)[2, 2])))


def rim_readings(ball_pos, ball_q, pot_pos, pot_q) -> dict:
    """Depth, tilt and off-centre of the ball in the new pot, read at the four rim points."""
    rb, rp = rot(ball_q), rot(pot_q)
    axis_b = rb[:, 2]
    top_c = np.asarray(ball_pos, float) + axis_b * ca.BALL_H
    pot_c = np.asarray(pot_pos, float)
    gaps, depths = [], []
    for d in (rp[:, 0], -rp[:, 0], rp[:, 1], -rp[:, 1]):  # front, back, left, right
        u = d - np.dot(d, axis_b) * axis_b  # the ball's top edge towards d, in its top face's plane
        u /= max(np.linalg.norm(u), 1e-9)
        edge = top_c + ca.BALL_R1 * u
        rim = pot_c + rp[:, 2] * ca.NEW_H + d * ca.NEW_RIM_IN
        depths.append(float(np.dot(rim - edge, rp[:, 2])))
        gaps.append(float(ca.NEW_RIM_IN - np.dot(edge - pot_c, d)))
    return {"depth": float(np.mean(depths)), "tilt": float(max(depths) - min(depths)),
            "off": 0.5 * max(abs(gaps[0] - gaps[1]), abs(gaps[2] - gaps[3])), "depths": depths, "gaps": gaps}


def on_layer(ball_pos, ball_q, pot_pos, pot_q) -> bool:
    """The ball stands in the new pot on its base layer: its centre inside the pot's wall and below the rim, the
    lowest point of its base within ON_LAYER of the layer's top."""
    rp, rb = rot(pot_q), rot(ball_q)
    pos = np.asarray(ball_pos, float)
    rel = rp.T @ (pos + rb[:, 2] * (ca.BALL_H / 2) - np.asarray(pot_pos, float))
    if math.hypot(rel[0], rel[1]) > ca.NEW_RIM_IN or not (ca.LAYER_TOP < rel[2] < ca.NEW_H):
        return False
    lowest = min(float((rp.T @ (pos + ca.BALL_R0 * (math.cos(a) * rb[:, 0] + math.sin(a) * rb[:, 1])
                                - np.asarray(pot_pos, float)))[2]) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False))
    return abs(lowest - ca.LAYER_TOP) < ON_LAYER


def old_pot_relation(ball_pos, ball_q, pot_pos, pot_q) -> dict:
    """How far the ball's base has risen in the old pot along its axis, and whether it is fully out (no part of the
    ball within the pot's body)."""
    rp, rb = rot(pot_q), rot(ball_q)
    pos = np.asarray(ball_pos, float)
    local = lambda p: rp.T @ (np.asarray(p, float) - np.asarray(pot_pos, float))  # noqa: E731
    risen = float(local(pos)[2] - ca.BALL_Z0)
    inside = False
    for t in np.linspace(0.0, ca.BALL_H, 6):
        for a in np.linspace(0, 2 * math.pi, 8, endpoint=False):
            r = ca.BALL_R0 + ca.TAPER * t
            p = local(pos + rb[:, 2] * t + r * (math.cos(a) * rb[:, 0] + math.sin(a) * rb[:, 1]))
            if -0.003 < p[2] < ca.OLD_H and math.hypot(p[0], p[1]) < ca.OLD_RIM_R:
                inside = True
    return {"risen": risen, "out": not inside}


def tier(m: dict) -> float:
    """The score of an end state. ``m`` holds the latched zero ``fault`` and, for now: ``whole`` (cactus still on its
    ball), ``out`` (fully out of the old pot), ``released`` (squeeze-freed at some point) and ``max_risen`` (the most
    the ball rose in the old pot), ``on_layer``, ``upright`` (within 30 degrees), ``resting``, ``held`` (any robot
    contact with the cactus, collar or ball), ``home``, ``on_table``, ``depth``, ``off``, ``tilt``."""
    if m.get("fault") or not m["whole"] or m["held"] or not m.get("on_table", True):
        return 0.0
    if m["out"] and m["home"] and m["resting"] and m["upright"]:
        if m["on_layer"]:
            lo, hi = FULL["depth"]
            if lo <= m["depth"] <= hi and m["off"] <= FULL["off"] and m["tilt"] <= FULL["tilt"]:
                return 1.0
            lo, hi = PART["depth"]
            if lo <= m["depth"] <= hi and m["off"] <= PART["off"] and m["tilt"] <= PART["tilt"]:
                return 0.75
        return 0.5
    if m["released"] and m["max_risen"] >= RISEN:  # freed and risen 1 cm, in the pot or out of it
        return 0.25
    return 0.0


def cylinder_gap(point, pos, q_xyzw, radius: float, z0: float, z1: float) -> float:
    """Distance from a world point to an upright cylinder in a body's frame (radius, from z0 to z1 along its z): 0
    inside it."""
    loc = rot(q_xyzw).T @ (np.asarray(point, float) - np.asarray(pos, float))
    dr = max(0.0, math.hypot(loc[0], loc[1]) - radius)
    dz = max(0.0, loc[2] - z1, z0 - loc[2])
    return math.hypot(dr, dz)


def object_hulls(obs: dict) -> list:
    """Each object as an upright cylinder round its outside: the old pot (rim), the new pot (rim), the root ball (its
    top), the cactus with its collar (the collar's outside, up to its crown)."""
    return [(*obs["old_pot"], ca.OLD_RIM_R, 0.0, ca.OLD_H), (*obs["new_pot"], ca.NEW_RIM_R, 0.0, ca.NEW_H),
            (*obs["ball"], ca.BALL_R1, 0.0, ca.BALL_H), (*obs["cactus"], ca.COLLAR_R_OUT, 0.0, ca.CACTUS_H)]


def tips_clear(tips, hulls) -> bool:
    """Every fingertip point at least CLEAR from every object's outside."""
    return all(cylinder_gap(t, *h) >= CLEAR for t in tips for h in hulls)


def pad_on_ball_point(in_ball, in_old_pot) -> bool:
    """Whether a fingertip's contact with the root ball counts as touching it: any contact on its soil face (the top,
    inside its edge), or on the rest of the ball once that part is out of the old pot. A contact on the ball's side
    below the old pot's rim and within its wall is a contact through the pot's thin wall, which lies between the finger
    and the ball (the squeeze), and does not count. ``in_ball``: the point in the ball's frame (origin at its base
    centre, z up); ``in_old_pot``: in the old pot's frame (origin at its bottom centre)."""
    r_b = math.hypot(in_ball[0], in_ball[1])
    if in_ball[2] > ca.BALL_H - 0.0015 and r_b < ca.BALL_R1 - 0.0015:
        return True  # the soil face
    r_p = math.hypot(in_old_pot[0], in_old_pot[1])
    inside_wall = -0.003 < in_old_pot[2] < ca.OLD_H and r_p < ca.old_outer_r(max(0.0, in_old_pot[2])) + 0.001
    return not inside_wall


class Trial:
    """The trial's memory: the squeeze rule and every zero, latched over the trial. ``update`` takes what the simulator
    shows after one control step (a dict; see below) and returns the end-state measures with their ``score``.

    obs: ``ball``, ``cactus``, ``old_pot``, ``new_pot``: (position, quaternion xyzw) in the table's frame; ``flex``:
    the old pot's squeeze now, the larger over its two opposite pairs of wall panels of the pair's smaller inward
    deflection 4.5 cm up (m); ``off_collar``: a robot part touches the cactus outside its collar now; ``pad_on_ball``:
    a gripper finger touches the root ball or its soil face now; ``held``: a robot part touches the cactus, collar or ball now; ``fall``: the
    furthest the cactus and ball have fallen with nothing touching them; ``speed``: the ball's speed (m/s); ``tcp_home``:
    both grippers' TCPs within 3 cm of their start pose; ``tips``: the fingertip points of both grippers (world); home
    is ``tcp_home`` with every fingertip 3 cm clear of every object, held for 1 s. Older callers may give ``home``:
    both grippers at their start pose and clear; ``top``: the table's height."""

    def __init__(self):
        self.faults: list[str] = []
        self.squeeze_run = 0
        self.released = False
        self.just_released = False
        self.max_risen = 0.0
        self.whole = True
        self.touch_run = 0
        self.pad_run = 0
        self.home_run = 0
        self.starts: dict | None = None

    def fault(self, why: str) -> None:
        if why not in self.faults:
            self.faults.append(why)

    def _home(self, obs: dict) -> bool:
        if "tips" in obs:
            now = bool(obs["tcp_home"]) and tips_clear(obs["tips"], object_hulls(obs))
        else:
            now = bool(obs["home"])
        self.home_run = self.home_run + 1 if now else 0
        return self.home_run >= HOME_STEPS

    def update(self, obs: dict) -> dict:
        bp, bq = (np.asarray(v, float) for v in obs["ball"])
        cp, cq = (np.asarray(v, float) for v in obs["cactus"])
        opp, opq = (np.asarray(v, float) for v in obs["old_pot"])
        npp, npq = (np.asarray(v, float) for v in obs["new_pot"])
        top = obs["top"]
        if self.starts is None:
            self.starts = {"old_pot": opp.copy(), "new_pot": npp.copy(), "stem": rot(bq).T @ (cp - bp)}
        # the squeeze rule: the ball's bond lets go once the wall has been flexed in 3 mm each side for 1 s
        self.squeeze_run = self.squeeze_run + 1 if obs["flex"] >= FLEX else 0
        self.just_released = not self.released and self.squeeze_run >= SQUEEZE_STEPS
        self.released |= self.just_released
        rel = old_pot_relation(bp, bq, opp, opq)
        self.max_risen = max(self.max_risen, rel["risen"])
        if rel["risen"] >= RISEN and not self.released:
            self.fault("the ball rose 1 cm in the old pot before the pot was squeezed")
        if opp[2] - self.starts["old_pot"][2] >= OLD_POT_LIFT:
            self.fault("the old pot was lifted 2 cm")
        if np.linalg.norm(rot(bq).T @ (cp - bp) - self.starts["stem"]) > BREAK_GAP:
            self.whole = False
        self.touch_run = self.touch_run + 1 if obs["off_collar"] else 0
        if self.touch_run >= TOUCH_STEPS:
            self.fault("a gripper or arm touched the cactus outside the collar")
        self.pad_run = self.pad_run + 1 if obs["pad_on_ball"] else 0
        if self.pad_run >= TOUCH_STEPS:
            self.fault("a gripper finger touched the root ball or its soil face")
        if obs["fall"] > DROP:
            self.fault("the cactus fell 3 cm or more unsupported")
        if tilt_of(bq) > LIE_OVER:
            self.fault("the cactus lay over")
        if np.linalg.norm((npp - self.starts["new_pot"])[:2]) > NEW_POT_MOVE:
            self.fault("the new pot moved more than 2 cm")
        if tilt_of(npq) > NEW_POT_TIP:
            self.fault("the new pot tipped")
        on_table = True
        for name, p in (("ball", bp), ("cactus", cp), ("old pot", opp), ("new pot", npp)):
            if p[2] < top - 0.02 or not (-0.19 < p[0] < 0.91 and -0.65 < p[1] < 0.65):
                self.fault(f"the {name} left the table")
                on_table = False
        m = {"fault": bool(self.faults), "whole": self.whole, "out": rel["out"], "risen": rel["risen"],
             "released": self.released, "max_risen": self.max_risen, "on_layer": on_layer(bp, bq, npp, npq),
             "upright": tilt_of(bq) < LIE_OVER, "resting": obs["speed"] < REST_SPEED, "held": obs["held"],
             "home": self._home(obs), "on_table": on_table}
        m.update({k: v for k, v in rim_readings(bp, bq, npp, npq).items() if k in ("depth", "tilt", "off")})
        m["score"] = tier(m)
        return m
