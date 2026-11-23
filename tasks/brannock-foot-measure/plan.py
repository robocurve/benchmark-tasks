"""Key gripper poses shared by the scripted demo and the reach check. Pure Python.

Each pose is (TCP world position, yaw, tilt): yaw pi/2 closes the fingers along world x (the device's v axis); tilt
(deg) then turns the grip about world x, + leaning its approach toward +y. The arm holding the post leans toward its
own side and the arm working the sliders toward the other, which keeps the two elbows apart.
"""

from __future__ import annotations

import math

import numpy as np

import footform as F
import layout as LY
from sizing import BAR_START, DEVICE as D, PTR_START

# slider parts (mm, device frame); see device_usd.py
FIN_H = 24.0  # grasp fins (printed tabs) stand this tall above their 2 mm flange
TAB_BASE = 2.0  # the tab's flange, taped to the slider's top
PTR_CARR_H = 6.0  # pointer carriage thickness on the guide top
PTR_FIN_Z = (D["guide_h"] + PTR_CARR_H + TAB_BASE, D["guide_h"] + PTR_CARR_H + TAB_BASE + FIN_H)
PTR_FIN_V = D["guide_v"] + D["guide_w"] / 2
BAR_FIN_Z = (D["bar_z"][1] + TAB_BASE, D["bar_z"][1] + TAB_BASE + FIN_H)
BAR_FIN_U = (D["bar_u"][0] + D["bar_u"][1]) / 2
BAR_FIN_OUT = 40.0  # the bar's fin stands on a bridge this far lateral of the beam's outer face
YAW = math.pi / 2  # right arm
YAW_L = -math.pi / 2  # left arm: the same finger axis, the wrist roll kept inside its limits
TILT_LEFT, TILT_RIGHT = -20.0, 30.0
GRIP_BELOW_TOP = 3.0  # TCP this far below a fin's top (the pads reach 13 mm further down)
POST_GRIP_Z = F.POST_TOP - 24.0
HOVER = 40.0  # mm above a fin grasp height
APPROACH_BACK = 0.05  # the post is approached and left along the tilted gripper axis (m)
PAD_TIP = 13.0  # pad tips below the TCP (yam_arena.constants.PAD_TIP_PAST_TCP)


def grip_matrix(yaw: float, tilt_deg: float) -> np.ndarray:
    from yam_arena.demo import grip_rotation

    b = math.radians(tilt_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(b), -math.sin(b)], [0, math.sin(b), math.cos(b)]])
    return rx @ grip_rotation(yaw)


def key_poses(k: int) -> dict:
    """Waypoints for layout k: left arm holds the ankle post; right arm slides the pointer, then the width bar."""
    form, pos, _ = LY.form_start(k)
    fo = F.Form(form["L"], form["A"], form["W"])
    pu, pv = fo.post_center()
    lay = LY.LAYOUTS[k]
    psi = math.radians(lay["yaw"])
    # post centre at the start: form origin + rotation of (pu, pv)
    su = lay["fwd"] + pu * math.cos(psi) - pv * math.sin(psi)
    sv = lay["side"] + pu * math.sin(psi) + pv * math.cos(psi)
    zg = POST_GRIP_Z
    back = -APPROACH_BACK * grip_matrix(YAW_L, TILT_LEFT)[:, 2]  # 50 mm back along the gripper axis (world, m)
    seat_u = pu - 1.5  # pull 1.5 mm past contact to keep the heel against the cup
    left = [
        (LY.to_world(su, sv, zg) + back, YAW_L, TILT_LEFT),
        (LY.to_world(su, sv, zg), YAW_L, TILT_LEFT),
        (LY.to_world(seat_u, 0.0, zg), YAW_L, TILT_LEFT),
        (LY.to_world(seat_u, 0.0, zg), YAW_L, TILT_LEFT),
        (LY.to_world(seat_u, 0.0, zg) + back, YAW_L, TILT_LEFT),
    ]
    zb = BAR_FIN_Z[1] - GRIP_BELOW_TOP
    zp = PTR_FIN_Z[1] - GRIP_BELOW_TOP
    fin_off = D["bar_t"] + BAR_FIN_OUT  # fin centre lateral of the bar face
    bar_v0 = D["guide_v"] - BAR_START - fin_off
    bar_v1 = D["guide_v"] - form["W"] - fin_off + 1.0
    right = [
        (LY.to_world(PTR_START, PTR_FIN_V, zp + HOVER), YAW, TILT_RIGHT),
        (LY.to_world(PTR_START, PTR_FIN_V, zp), YAW, TILT_RIGHT),
        (LY.to_world(form["A"], PTR_FIN_V, zp), YAW, TILT_RIGHT),
        (LY.to_world(form["A"], PTR_FIN_V, zp + HOVER), YAW, TILT_RIGHT),
        (LY.to_world(BAR_FIN_U, bar_v0, zb + HOVER), YAW, TILT_RIGHT),
        (LY.to_world(BAR_FIN_U, bar_v0, zb), YAW, TILT_RIGHT),
        (LY.to_world(BAR_FIN_U, bar_v1, zb), YAW, TILT_RIGHT),
        (LY.to_world(BAR_FIN_U, bar_v1, zb + HOVER), YAW, TILT_RIGHT),
    ]
    return {"left": left, "right": right, "left_hold_index": None, "nose_offset": 0.0}
