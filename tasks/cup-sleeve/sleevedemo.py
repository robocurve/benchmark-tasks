"""The scripted demo: open-loop TCP waypoints for each arm, planned from the fixed start layout (object_jitter=0) and
the visible start poses of the sleeve and the cup. It never reads the scorer or hidden state. A few constants were
measured in the simulator (where the opened ring stands, where the fingers first touch, where the held cup hangs).

Route: the flat sleeve gapes a little in the organizer; each hand slips its inner finger into the gap and pinches one
layer, and the hands lift it out and draw the layers apart into a ring, set it down standing and press its sides
round. The right hand lifts the full cup lightly by its upper wall, under the lid, carries it over the ring and lowers
it in, narrow end first, onto the counter. The left hand closes lightly round the sleeve's lower part and slides it up
the cup's taper until it holds; both hands go back to rest.

TASK_DEMO selects a variant: success; the scorer checks notfull (the sleeve is left low and loose round the cup's
base), squeeze (the cup is gripped too hard) and flat (the sleeve is never opened; the cup is set down where the ring
would be).
"""

from __future__ import annotations

import math

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER, READY
import yam_arena.demo as harness_demo
from yam_arena.demo import Waypoint
from yam_arena.urdf import axis_angle_matrix

import sleevegeom as G
import sleevekit as K

BASES = {"left": np.array((0.0, 0.305, 0.75)), "right": np.array((0.0, -0.305, 0.75))}
TOP = K.TOP


class Grip(float):
    """A waypoint's yaw that carries a whole TCP rotation (tilted grips); harmless to the harness's plain yaw."""

    rot: np.ndarray = np.eye(3)


def _install_grips() -> None:
    if getattr(harness_demo.grip_rotation, "_takes_grips", False):
        return
    plain = harness_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        rot = getattr(yaw, "rot", None)
        return plain(yaw) if rot is None else rot

    grip_rotation._takes_grips = True
    grip_rotation._plain = plain
    harness_demo.grip_rotation = grip_rotation


_install_grips()
_plain = harness_demo.grip_rotation._plain


def down(yaw_deg: float = 0.0) -> Grip:
    g = Grip(math.radians(yaw_deg))
    g.rot = _plain(math.radians(yaw_deg))
    return g


def tilted(theta_deg: float, phi_deg: float) -> Grip:
    """Tilted theta from straight down toward the direction phi from +x (the fingers point that way), the fingers
    closing across that direction; the wrist stands back the other way."""
    phi = math.radians(phi_deg)
    r = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.radians(theta_deg)) @ _plain(phi)
    g = Grip(0.0)
    g.rot = r
    return g


def ready_tcp(side):
    return ik.tcp_pose(np.array(READY, float), BASES[side])[0]


def finger_for(width: float, force: float) -> float:
    """Finger target to close on something `width` across with about `force` newtons a finger."""
    return float(np.clip(-(width / 2 - force / GRIPPER["stiffness"]), FINGER_OPEN, FINGER_CLOSED))


def W(t, p, f, label="", yaw=0.0):
    return Waypoint(float(t), tuple(float(v) for v in p), float(f), label, yaw)


def _idle():
    return {s: [W(6.0, ready_tcp(s), FINGER_OPEN, "hold")] for s in ("left", "right")}


def _crease_y(z_above_floor: float) -> float:
    """|y| of a crease line (from the organizer's middle) at a height above the compartment floor."""
    g = G.flat_generators(K.SLEEVE_ORIGIN)
    b, t = g[0]
    s = (K.SLEEVE_ORIGIN[2] + z_above_floor - b[2]) / (t[2] - b[2])
    return abs(float(b[1] + s * (t[1] - b[1]) - K.SLEEVE_ORIGIN[1]))


# -- tuned in the simulator ------------------------------------------------------------------------------------------
TILT_OPEN = 25.0  # the opening grips lean outboard so that the two forearms stay clear of each other
PINCH_TIP_Z = 0.038  # pad tips above the compartment floor (the pad reaches 21 mm up from its tip)
PAD_IN = 0.003  # the pad's middle this far in from the crease (the pad is about 27 mm wide, 21 mm tall)
PAD_HALF = 0.012
PUSH_CREASE = 0.040  # half the crease-to-crease distance the pinched creases are pushed to
RING_XY = (0.330, 0.000)  # where the opened ring is set down
RING_AT = (0.324, 0.005)  # where it then stands (seen: the middle of the standing ring)
ROUND_IN = 0.036  # the crease's distance from the ring's middle after pressing it in
ROUND_TIP_Z = 0.025  # pad tips above the counter while pressing
CUP_DY = 0.019  # the grip this far toward +y of the cup's axis, so that the pinch line passes over its centre: it hangs upright
CUP_CARRY_FIX = (0.008, 0.011)  # where the hanging cup sits off the planned spot (measured), made up when carrying it over
CUP_TILT = 45.0  # the cup grip leans outboard (right arm)
CUP_TIP_H = 0.085  # tip point 13 mm past the TCP: the pads bear on the wall about 106 mm up, the fingers clear the lid
CUP_W, CUP_F = 0.0895, 4.0  # where the fingers stop on the cup (measured), and the pinch: under the 6 N limit
CUP_CLEAR = 0.036  # cup base above the ring's upper edge while carried over, as planned (the held cup sags about 15 mm)


def _tcp(tip, grip: Grip, past: float = 0.013) -> np.ndarray:
    """The TCP for a pad-tip point: the tips are `past` beyond the TCP along the approach axis."""
    return np.asarray(tip, float) - past * grip.rot[:, 2]


def _block_tcp(edge, grip: Grip, toward) -> np.ndarray:
    """The TCP that puts the closed fingers' side edge (the pads' width is about 24 mm) on `edge`, facing `toward`."""
    w = grip.rot[:, 1]
    s = 1.0 if float(w @ np.asarray(toward, float)) > 0 else -1.0
    return np.asarray(edge, float) - s * BLOCK_HALF * w


def _open(plan: dict) -> None:
    """The flat sleeve gapes a little in the organizer (its creases spring back). Each hand slips its inner finger into
    the gap and pinches one layer (left: the far layer, right: the near one); they lift it out and draw the layers
    apart, so that it opens into a ring, and set the ring down standing."""
    x0, y0 = K.ORG_XY
    zf = K.SLEEVE_ORIGIN[2]
    xr, yr = RING_XY
    L, R = plan["left"], plan["right"]
    lift = 0.050
    for side, arm in (("left", L), ("right", R)):
        s = 1 if side == "left" else -1  # left takes the +x layer at +y
        g = tilted(TILT_OPEN, -90.0 * s)
        y = y0 + s * GAP_Y
        # the layer's inner face is at x0 + s * GAP_HALF; the inner finger's pad goes just inside it
        x_mid = x0 + s * (GAP_HALF + G.BOARD_T / 2)  # the pinch's middle: the layer
        tip = np.array((x_mid, y, zf + GAP_TIP_Z))
        t = _tcp(tip, g)
        w_in = finger_for(INSERT_W, 0.0)
        pinch = finger_for(G.BOARD_T, 5.0)
        y_ring = yr + s * GAP_Y
        x_ring = xr + s * PULL_X
        t_apart = t + np.array((x_ring - x_mid, y_ring - y, lift))
        t_set = t_apart + np.array((0.0, 0.0, -lift - (zf - TOP) + 0.0015))
        arm += [
            W(2.5, t + (0, 0, 0.06), w_in, "over the gap", g),
            W(1.5, t, w_in, "inner finger into the gap", g),
            W(0.8, t, pinch, "pinch one layer", g),
            W(1.5, t + (0, 0, lift), pinch, "lift it out", g),
            W(3.0, t_apart, pinch, "draw the layers apart: a ring", g),
            W(1.5, t_set, pinch, "set it down standing", g),
            W(0.6, t_set, w_in, "let go", g),
            W(1.0, t_set + (0, 0, 0.09), w_in, "up out of the ring", g),
        ]
        # round it: press its crease side (along y) in with the closed fingers' edge
        press = np.array((xr, yr + s * ROUND_R, TOP + ROUND_Z))
        tp = _block_tcp(press, g, (0, -s, 0))
        arm += [
            W(1.2, tp + (0, s * 0.03, 0.06), FINGER_CLOSED, "beside the ring", g),
            W(0.8, tp + (0, s * 0.03, 0.0), FINGER_CLOSED, "down", g),
            W(1.5, tp, FINGER_CLOSED, "press its side in: round", g),
            W(0.8, tp + (0, s * 0.03, 0.0), FINGER_CLOSED, "back", g),
            W(0.8, tp + (0, s * 0.03, 0.08), FINGER_CLOSED, "up", g),
        ]


PULL_X = 0.028  # each pinch drawn this far from the ring's middle along x (the pinches end a diameter apart)
GAP_Y = 0.022  # each hand's pinch this far from the organizer's middle (the gap is widest there)
GAP_HALF = 0.0064  # half the gap between the gaping layers' inner faces at the pinch (measured: about 13 mm)
GAP_TIP_Z = 0.052  # the pad tips above the compartment floor (the pads take the top 18 mm)
INSERT_W = 0.010  # the fingers' opening as the inner one goes into the gap
ROUND_R = 0.0365  # the ring's outer radius at the pressing height, along y
ROUND_Z = 0.030  # pressing height above the counter
BLOCK_HALF = 0.012  # half the closed fingers' width, at the pads
RING_R = 0.036  # the opened ring's radius at mid-height (its crease from the middle)
EAR = 0.008  # how far the pinched crease A stands out beyond the round until it is pressed in


def _cup_and_wedge(plan: dict, mode: str = "success") -> None:
    """Right: lift the cup by its upper wall, carry it over the ring, lower it into the ring onto the counter, let go.
    Left: close lightly round the ring's lower part (pressing it onto the cup) and slide it up the taper until it
    holds; the fingers open as they rise, following the cone, so that the press stays light."""
    xr, yr = RING_AT
    L, R = plan["left"], plan["right"]
    gr = tilted(CUP_TILT, 90.0)
    cx, cy = K.CUP_XY
    tc = _tcp(np.array((cx, cy + CUP_DY, TOP + CUP_TIP_H)), gr)
    grip = finger_for(CUP_W, CUP_F if mode != "squeeze" else SQUEEZE_F)
    lift = G.SLEEVE_H + CUP_CLEAR
    over = tc + np.array((xr - cx + CUP_CARRY_FIX[0], yr - cy + CUP_CARRY_FIX[1], lift))
    low = over + np.array((0.0, 0.0, -lift + 0.0005))
    mid = tc + np.array((0.5 * (xr - cx), 0.5 * (yr - cy), lift))
    R += [
        W(1.0 if R else 2.0, np.asarray(R[-1].pos) + (0, 0, 0.06) if R else ready_tcp("right") + (0.0, 0.05, -0.12), FINGER_OPEN, "up", gr),
        W(1.5, tc + (0, -0.04, 0.07), FINGER_OPEN, "over beside the cup", gr),
        W(1.0, tc + (0, -0.04, 0.0), FINGER_OPEN, "down beside it", gr),
        W(1.5, tc, FINGER_OPEN, "in round the cup, under the lid", gr),
        W(1.0, tc, grip, "grip the cup lightly", gr),
        W(3.0, tc + (0, 0, lift), grip, "lift", gr),
        W(0.8, tc + (0, 0, lift), grip, "steady", gr),
        W(2.0, mid, grip, "toward the ring", gr),
        W(2.0, over, grip, "over the ring", gr),
        W(1.0, over, grip, "steady", gr),
        W(3.0, low, grip, "down into the ring, onto the counter", gr),
        W(0.6, low, FINGER_OPEN, "let go", gr),
        W(1.2, low + (0, -0.05, 0.0), FINGER_OPEN, "out", gr),
        W(1.0, low + (0, -0.08, 0.08), FINGER_OPEN, "away", gr),
    ]
    _sync(plan)
    if mode in ("notfull", "flat"):
        _rest(plan)
        return
    # left: slide the sleeve up the cup
    gl = tilted(SLIDE_TILT, -90.0)
    t0 = _tcp(np.array((xr, yr + SLIDE_DY, TOP + SLIDE_TIP)), gl)
    w0 = SLIDE_TOUCH - SLIDE_PRESS
    w1 = w0 + SLIDE_OPEN_RATE * 2 * G.CUP_SLOPE * SLIDE_UP
    L += [
        W(1.5, t0 + (0, 0.06, 0.04), FINGER_OPEN, "beside the sleeve", gl),
        W(1.0, t0 + (0, 0.06, 0.0), FINGER_OPEN, "down beside it", gl),
        W(1.5, t0, FINGER_OPEN, "in round the sleeve's lower part", gl),
        W(1.0, t0, finger_for(w0, 0.0), "close lightly", gl),
        W(3.0, t0 + (0, 0, SLIDE_UP), finger_for(w1, 0.0), "slide it up the taper", gl),
        W(0.6, t0 + (0, 0, SLIDE_UP), FINGER_OPEN, "let go", gl),
        W(1.0, t0 + (0, 0.06, SLIDE_UP), FINGER_OPEN, "out", gl),
        W(1.0, t0 + (0, 0.08, SLIDE_UP + 0.08), FINGER_OPEN, "away", gl),
    ]
    _sync(plan)
    _rest(plan)


def _rest(plan: dict) -> None:
    for s in ("left", "right"):
        plan[s].append(W(2.0, ready_tcp(s) + np.array((-0.06, 0.0, -0.08)), FINGER_OPEN, "back to rest", down(0.0)))
        plan[s].append(W(2.0, plan[s][-1].pos, FINGER_OPEN, "rest", plan[s][-1].yaw))


SQUEEZE_F = 9.0  # the "squeeze" variant's pinch on the cup
SLIDE_TILT = 60.0  # the sliding grip leans well back toward the left arm, so the fingers stay clear of the cup above
SLIDE_TIP = 0.015  # tip point above the counter
SLIDE_DY = 0.0
SLIDE_TOUCH = 0.0775  # the fingers' opening where they first touch the sleeve there (finger meshes and the measured offset)
SLIDE_UP = 0.034  # how far the fingers rise
SLIDE_OPEN_RATE = 0.0  # the fingers hold their opening as they rise (the light press stays under 2 N)
SLIDE_PRESS = 0.0097  # closed past touching: the loose sleeve's 2.8 mm a side onto the cup, then a light press
FINGER_PROUD = 0.0044  # the fingers touch about 2.2 mm before their meshes would (measured)


def _sync(plan: dict) -> None:
    """Pad the shorter arm's plan with a hold so that both reach this point together."""
    tl = sum(w.duration for w in plan["left"])
    tr = sum(w.duration for w in plan["right"])
    for s, dt in (("left", tr - tl), ("right", tl - tr)):
        if dt > 1e-6:
            w = plan[s][-1]
            plan[s].append(W(dt, w.pos, w.finger, "wait", w.yaw))


def plan(mode: str) -> dict:
    if mode == "idle":
        return _idle()
    if mode not in ("success", "notfull", "squeeze", "flat", "ring"):
        raise ValueError(f"TASK_DEMO={mode}: one of success, notfull, squeeze, flat")
    p = {"left": [], "right": []}
    if mode == "flat":  # the sleeve is left flat in the organizer; the left arm waits at its ready pose
        p["left"].append(W(1.0, ready_tcp("left"), FINGER_OPEN, "wait", down(-90.0)))
    else:
        _open(p)
        _sync(p)
    if mode == "ring":
        return p
    _cup_and_wedge(p, mode)
    return p
