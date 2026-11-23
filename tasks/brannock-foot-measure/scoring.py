"""Measurement and scoring, pure numpy (no simulator): the same quantities a person takes in a real trial.

measure(...) turns the simulator state (the form's pose, the two slider joints, the grippers' finger points) into
readings; score(...) turns readings into the graded score. Every quantity has a real-trial counterpart:

  heel_gap_mm   gap behind the heel at the cup's centre            3 mm feeler gauge
  width_steps   width read on the bar at the form's size           the device's width scale
  bar_gap_mm    gap between the width bar and the foot's side      3 mm feeler gauge
  arch_size     size read at the pointer's index                   the device's arch scale
  nose_gap_mm   gap between the pointer's curve and the ball joint 3 mm feeler gauge
  held_*        a gripper within 20 mm of the part                 trial video
  clear_mm      nearest gripper to the form, pointer or bar        trial video / photo, 50 mm
  still         form < 5 mm/s, < 3 deg/s; sliders < 2 mm/s          trial video: over the last 1 s (20 frames) the form
                                                                   moves under 5 mm and each slider under 2 mm
  fallen        any part of the form below the base plate's top    trial video (the form touches the table)
  tipped        the ankle post more than 20 deg from vertical      trial video / photo
  heel_out      heel 10 mm or more off the cup's back wall after a slider has moved 2 mm   trial video
"""

from __future__ import annotations

import math

import numpy as np

import footform as F
import layout as LY
import sizing as S
from sizing import DEVICE as D

FEELER = 3.0  # mm
HELD_MM = 20.0
CLEAR_MM = 50.0
TIP_DEG = 20.0
FALL_MM = 5.0
STILL_LIN = 0.005  # m/s
STILL_ANG = math.radians(3.0)  # rad/s
STILL_JOINT = 0.002  # m/s
WIDTH_TOL = 1.0  # widths
ARCH_TOL = 0.5  # sizes


def quat_matrix(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class FormGeometry:
    """Sample points of one form (mm, form frame): outline at the widest level, heel point, a cloud for distances."""

    def __init__(self, spec: dict):
        self.spec = spec
        fo = F.Form(spec["L"], spec["A"], spec["W"], spec["name"])
        self.form = fo
        us = np.linspace(1.0, fo.u_end, 260)
        self.medial = np.array([(u, fo.v_bounds(u)[1], float(fo.zmid(u))) for u in us])
        self.lateral = np.array([(u, fo.v_bounds(u)[0], float(fo.zmid(u))) for u in us])
        self.heel = np.array([0.0, 0.0, float(fo.zmid(0.0))])
        cloud = [np.vstack(fo.hulls())]
        self.cloud = cloud[0][:: max(1, len(cloud[0]) // 900)]
        pu, pv = fo.post_center()
        self.post_axis = np.array([[pu, pv, 40.0], [pu, pv, F.POST_TOP]])


def to_device_pts(local_mm: np.ndarray, pos_w, quat_xyzw) -> np.ndarray:
    """Form-frame points (mm) -> device frame (mm), given the form's world pose."""
    R = quat_matrix(quat_xyzw)
    w = (np.asarray(local_mm) / 1000.0) @ R.T + np.asarray(pos_w)
    return np.array([LY.to_device(p) for p in w])


def measure(geo: FormGeometry, pos_w, quat_xyzw, lin_vel, ang_vel, ptr_q, bar_q, ptr_v, bar_v,
            finger_pts_w) -> dict:
    """Readings from the state. ptr_q: pointer joint (m, = index position u); bar_q: bar joint (m, = -d)."""
    spec = geo.spec
    heel = to_device_pts(geo.heel[None], pos_w, quat_xyzw)[0]
    med = to_device_pts(geo.medial, pos_w, quat_xyzw)
    lat = to_device_pts(geo.lateral, pos_w, quat_xyzw)
    cloud = to_device_pts(geo.cloud, pos_w, quat_xyzw)
    index = ptr_q * 1000.0
    d = -bar_q * 1000.0
    face_v = D["guide_v"] - d
    in_bar = (lat[:, 0] >= D["bar_u"][0]) & (lat[:, 0] <= D["bar_u"][1])
    bar_gap = float((lat[in_bar, 1] - face_v).min()) if in_bar.any() else math.inf
    nose = np.array([index - S.NOSE_TO_INDEX, D["guide_v"] - D["ptr_reach"] + D["ptr_nose_r"]])
    nose_gap = float(np.hypot(med[:, 0] - nose[0], med[:, 1] - nose[1]).min() - D["ptr_nose_r"])
    R = quat_matrix(quat_xyzw)
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2, 2]))))
    fingers = np.array([LY.to_device(p) for p in np.asarray(finger_pts_w).reshape(-1, 3)]) if len(finger_pts_w) else \
        np.zeros((0, 3))
    ptr_fin = np.array([index, (D["guide_v"] + D["guide_v"] + D["guide_w"]) / 2, D["guide_h"] + 20.0])  # tab centre
    bar_fin = np.array([(D["bar_u"][0] + D["bar_u"][1]) / 2, face_v - D["bar_t"] - 40.0, D["bar_z"][1] + 14.0])

    def nearest(points):
        if len(fingers) == 0:
            return math.inf
        return float(np.min(np.linalg.norm(fingers[:, None, :] - np.atleast_2d(points)[None], axis=2)))

    post = to_device_pts(geo.post_axis, pos_w, quat_xyzw)
    post_mm = math.inf
    for f in fingers:  # distance to the ankle post's surface (axis segment minus radius), negative inside
        a, b = post
        t = float(np.clip(np.dot(f - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0))
        post_mm = min(post_mm, float(np.linalg.norm(f - (a + t * (b - a)))) - F.POST_R)
    in_guide = (med[:, 0] >= D["guide_u"][0]) & (med[:, 0] <= D["guide_u"][1])
    guide_gap = float(D["guide_v"] - med[in_guide, 1].max()) if in_guide.any() else math.inf
    return dict(
        guide_gap_mm=guide_gap,
        heel_gap_mm=float(heel[0]), heel_side_mm=float(heel[1]),
        width_steps=S.width_steps(d, spec["size"]), width_ref=S.WIDTHS.index(spec["letter"]), bar_d_mm=d,
        bar_gap_mm=bar_gap,
        arch_size=S.arch_reading(index), arch_ref=spec["size"], nose_gap_mm=nose_gap,
        form_finger_mm=min(nearest(cloud), post_mm), ptr_finger_mm=nearest(ptr_fin), bar_finger_mm=nearest(bar_fin),
        lin_speed=float(np.linalg.norm(lin_vel)), ang_speed=float(np.linalg.norm(ang_vel)),
        joint_speed=float(max(abs(ptr_v), abs(bar_v))),
        lowest_mm=float(cloud[:, 2].min()), tilt_deg=tilt,
    )


HEEL_OUT_MM = 10.0  # latched zero: the heel, once seated, 10 mm or more off the cup's back wall after a slider moved
SLIDER_MOVED_MM = 2.0  # a slider counts as moved once it is 2 mm or more from its start


def faults(m: dict, armed: bool = False) -> list[str]:
    """Latched-zero events visible in this state. armed: the heel has been seated at least once and a slider has
    since been moved 2 mm or more from its start."""
    out = []
    if m["lowest_mm"] < -FALL_MM:
        out.append("fallen")
    if m["tilt_deg"] > TIP_DEG:
        out.append("tipped")
    if armed and m["heel_gap_mm"] >= HEEL_OUT_MM:
        out.append("heel_out")
    return out


def levels(m: dict) -> dict:
    heel = m["heel_gap_mm"] < FEELER and m["form_finger_mm"] > HELD_MM
    bar_on = m["bar_gap_mm"] < FEELER and m["bar_finger_mm"] > HELD_MM
    ptr_on = m["nose_gap_mm"] < FEELER and m["ptr_finger_mm"] > HELD_MM
    bar_ok = bar_on and abs(m["width_steps"] - m["width_ref"]) <= WIDTH_TOL
    ptr_ok = ptr_on and abs(m["arch_size"] - m["arch_ref"]) <= ARCH_TOL
    clear = min(m["form_finger_mm"], m["ptr_finger_mm"], m["bar_finger_mm"]) >= CLEAR_MM
    still = m["lin_speed"] < STILL_LIN and m["ang_speed"] < STILL_ANG and m["joint_speed"] < STILL_JOINT
    one_on = heel and (bar_on or ptr_on)  # 0.25: two actions, never a single push
    one_ok = heel and (bar_ok or ptr_ok)
    both_ok = heel and bar_ok and ptr_ok
    return dict(heel=heel, bar_on=bar_on, ptr_on=ptr_on, bar_ok=bar_ok, ptr_ok=ptr_ok, one_on=one_on, one_ok=one_ok,
                both_ok=both_ok, clear=clear, still=still, full=both_ok and clear and still)


def score(m: dict, latched: bool) -> float:
    if latched:
        return 0.0
    lv = levels(m)
    if lv["full"]:
        return 1.0
    return 0.75 if lv["both_ok"] else 0.5 if lv["one_ok"] else 0.25 if lv["one_on"] else 0.0
