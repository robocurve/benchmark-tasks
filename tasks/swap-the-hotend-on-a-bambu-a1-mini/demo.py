"""Scripted demo: swap the A1 mini's hotend, and the failure / shortcut variants.

The right arm pinches the buckle's tab top and bottom, pushes the buckle 2.5 mm to the right along its slide (freeing
its left end from the catch), swings it out to 80 degrees and lets go; the buckle's light spring takes it on to its
95-degree stop. The left arm takes the old hotend by the sides of its heatsink, from the front as a hand does (the
heatsink's front 8.5 mm stands proud of the toolhead's body), pulls it straight out of the recess and stands it in
the holder's empty (left) slot. The right arm takes the new hotend from the holder's right slot the same way, brings
it level in front of the recess, slides it in until its magnet block meets the mounting plate and lets go. It then
pinches the tab again, swings the buckle shut (still pushed right, so its end passes the catch), pushes it back left
so that its end goes behind the catch, and lets go.

Variants: no_lock (the buckle is swung shut but not pushed back left, so it is not locked), no_unlock (the right arm
first tries to swing the buckle open without pushing it right: the catch holds it; then as full), half_in (the new
hotend is slid in 3 mm short of its seat and let go, then the buckle is closed), drop_old (the old hotend is let go
over the bed instead of in the holder), drop_new (the new hotend is let go over the bed on its way to the toolhead),
buckle_only (a mechanism check: unlock, open, close and lock the buckle over the old hotend).

The script reads the parts' poses each step (as any scripted policy with full state may) and shifts the gripper's
target so that the part it holds follows the plan: the old hotend by its nozzle's tip, the new one by the point where
its tube meets the heatsink, the buckle by its tab. Parts move only through the grasps and contacts; no pose or joint
of a part is ever set.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "no_lock", "no_unlock", "half_in", "drop_old", "drop_new", "buckle_only")
GRASP = np.array([-0.0165, 0.0, 0.00825])  # TCP relative to a hotend's origin: pinch on the heatsink's sides from the
#   front, fingertips 3.5 mm in front of the toolhead's body, pads centred on the heatsink's height
TAB_BACK = 0.0093  # m: the TCP sits this far in front of the tab's centre (fingertips 0.3 mm short of the bar)
OPEN_DEG = 95.0  # the buckle's open stop: where its spring leaves it, and where the hand takes it to close it
SWING_TO = 80.0  # the hand swings it this far and lets go; the spring takes it on to its stop
PRESS_DEG = 0.5  # closing: the hand holds the bar this close to shut (its end 0.7 mm behind the catch's face, and
#   not pressed into the heating assembly) while it pushes it left
LOCK_OVER = 0.0005  # m: the push left is planned 0.5 mm past the slide's locked end, so the soft arm gets it there
TAB_OPEN = 0.15  # grip command around the tab near the hotend: fingers about 1.4 cm apart (the upper finger stays
#   under the heatsink)
TAB_C = (-0.005, 0.0105, 0.0)  # the tab's centre in the buckle's frame (task.TAB)
OPEN_H = 0.33  # grip command for the hotend grasps: fingers about 3.1 cm apart
PUSH = 0.0025  # m: how far the hand moves the tab to the right (the track's travel is 2.5 mm)
SWING_TRACK = 0.0025  # m: the track's offset the swings are planned with (at its 2.5 mm stop, pressed 0.2 mm on)
TIP_Z = -0.0281  # the nozzle's end in the hotend frame (task.TIP_Z)
SLOT_YAW = 15.0  # deg: the left hand turned toward +y when standing the old hotend in the holder


def R_front(sgn=1.0):
    """A pinch from the front: approach along +x, fingers closing along y; the pads' width is vertical."""
    x = np.array([0.0, sgn, 0.0])
    z = np.array([1.0, 0.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def R_tab(phi_deg):
    """On the tab: approach toward the toolhead (turned with the buckle), fingers closing on the tab's top and
    bottom."""
    ph = math.radians(phi_deg)
    z = np.array([math.cos(ph), math.sin(ph), 0.0])
    x = np.array([0.0, 0.0, 1.0])
    return np.column_stack([x, np.cross(z, x), z])


def build_plan(t, variant="full", params=None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    L, Rt = arms["left"], arms["right"]
    TH = np.asarray(t.TH)
    seat = TH + np.asarray(t.SEAT)
    hinge0 = np.asarray(t.BUCKLE_POS)
    tab_c = np.asarray(t.TAB["center"])
    Rh_l = Rh_r = R_front(-1.0)  # the same wrist roll suits both arms
    G = GRASP

    def tab_tcp(phi_deg, track=0.0, back=TAB_BACK):
        Rz = dc.rot((0, 0, 1), math.radians(phi_deg))
        c = hinge0 + np.array([0.0, track, 0.0]) + Rz @ tab_c
        R = R_tab(phi_deg)
        return c - back * R[:, 2], R

    def swing(dur, a0, a1, track, label):
        Rt.add(dur, grip=0.0, label=label, path=lambda u: tab_tcp(a0 + (a1 - a0) * u, track(a0 + (a1 - a0) * u)
                                                              if callable(track) else track))

    slot = {k: np.asarray(t.STAND_POS) + np.array([0.0, y, t.STAND_H]) for k, y in t.SLOT_Y.items()}
    fwd = np.array([-0.030, 0.0, 0.0])

    # -- right arm: unlock and open the buckle ----------------------------------------------------------------------
    p, R = tab_tcp(0.0, back=TAB_BACK + 0.030)
    Rt.add(3.0, p + [0, 0, -0.02], R, 1.0, "right: below in front of the tab")
    Rt.add(1.2, p, R, TAB_OPEN, "right: before the tab")
    Rt.add(1.2, *tab_tcp(0.0), TAB_OPEN, "right: around the tab")
    Rt.add(0.8, *tab_tcp(0.0), 0.0, "right: grip tab")
    if variant == "no_unlock":  # try to swing it open while it is still locked, then do it properly
        Rt.add(1.5, *tab_tcp(20.0), 0.0, "right: pull while locked")
        Rt.add(1.0, *tab_tcp(0.0), 0.0, "right: back to shut")
    p, R = tab_tcp(0.0)
    Rt.add(1.0, p + [0, -PUSH, 0], R, 0.0, "right: push right")
    swing(2.6, 0.0, SWING_TO, -SWING_TRACK, "right: swing open")
    Rt.add(0.6, grip=0.6, label="right: let go of the tab")
    p, R = tab_tcp(SWING_TO, -SWING_TRACK, TAB_BACK + 0.030)
    Rt.add(1.0, p, R, 0.6, "right: back off")
    Rt.add(1.5, p + [-0.05, -0.05, -0.02], R, 1.0, "right: clear")
    Rt.add(2.0, *starts["right"], 1.0, "right: home")  # between wrist branches: through the ready pose
    t_open = Rt.duration

    def close_buckle(lock=True, tag=""):  # the track is still pushed right: the bar's end passes the catch
        p, R = tab_tcp(OPEN_DEG, -SWING_TRACK, TAB_BACK + 0.030)
        Rt.add(2.0, *starts["right"], 1.0, f"right: home{tag}")
        Rt.add(2.0, p + [-0.03, -0.03, -0.02], R, 1.0, f"right: to the tab{tag}")
        Rt.add(1.2, p, R, 0.35, f"right: before the tab{tag}")
        Rt.add(1.2, *tab_tcp(OPEN_DEG, -SWING_TRACK), 0.35, f"right: around the tab{tag}")
        Rt.add(0.8, *tab_tcp(OPEN_DEG, -SWING_TRACK), 0.0, f"right: grip tab{tag}")
        swing(4.0, OPEN_DEG, PRESS_DEG, -SWING_TRACK, f"right: swing shut{tag}")
        Rt.add(1.5, *tab_tcp(PRESS_DEG, -SWING_TRACK), 0.0, f"right: press shut{tag}")
        if lock:  # push it back left, still pressing it shut: its left end goes behind the catch
            Rt.add(2.5, *tab_tcp(PRESS_DEG, LOCK_OVER), 0.0, f"right: push left to lock{tag}")
            Rt.add(1.5, label=f"right: hold it there{tag}")
        Rt.add(0.6, grip=TAB_OPEN, label=f"right: let go of the tab{tag}")
        p, R = tab_tcp(0.0, 0.0 if lock else -SWING_TRACK, TAB_BACK + 0.012)
        Rt.add(0.8, p, R, TAB_OPEN, f"right: off the tab{tag}")
        p, R = tab_tcp(0.0, 0.0, TAB_BACK + 0.030)
        Rt.add(1.0, p, R, 0.6, f"right: back off{tag}")

    if variant == "buckle_only":
        close_buckle(True)
        Rt.add(2.5, *starts["right"], 1.0, "right: back")
        L.add(Rt.duration + 3.0, label="left: wait")
        Rt.add(3.0, label="right: wait")
        return L, Rt, {}

    # -- left arm: the old hotend out and into the holder's empty (left) slot --------------------------------------
    L.add(max(t_open - 3.0, 0.5), label="left: wait")
    L.add(2.5, seat + G + fwd + [0, 0, 0.02], Rh_l, OPEN_H, "left: in front of the old hotend")
    L.add(1.2, seat + G + fwd, Rh_l, OPEN_H, "left: level with it")
    L.add(1.5, seat + G, Rh_l, OPEN_H, "left: around the old hotend")
    L.add(1.0, seat + G, Rh_l, 0.0, "left: grip old")
    L.add(2.0, seat + G + [-0.022, 0, 0.0], Rh_l, 0.0, "left: pull out")
    L.add(1.2, seat + G + [-0.040, 0, -0.010], Rh_l, 0.0, "left: away from the toolhead")
    if variant == "drop_old":
        L.add(0.8, grip=OPEN_H, label="left: let go over the bed")
        L.add(1.0, seat + G + [-0.070, 0.06, -0.010], Rh_l, OPEN_H, "left: away")
    else:
        Rz = dc.rot((0, 0, 1), math.radians(SLOT_YAW))  # the left arm reaches this low slot better turned a little
        Rs, Gs = Rz @ Rh_l, Rz @ G
        L.add(2.2, slot["empty"] + Gs + [0, 0, 0.040], Rs, 0.0, "left: above the empty slot")
        L.add(1.6, slot["empty"] + Gs + [0, 0, 0.0008], Rs, 0.0, "left: into the slot")
        L.add(0.8, grip=OPEN_H, label="left: let go old")
        L.add(1.0, slot["empty"] + Gs + [0, 0, 0.045], Rs, OPEN_H, "left: up off the old one")
    L.add(2.0, *starts["left"], 1.0, "left: back")
    t_old_out = L.duration - 2.0 - 1.0 - 0.8  # the old hotend is stood in its slot

    # -- right arm: the new hotend from the holder's right slot into the recess ------------------------------------
    new_g = slot["new"] + G
    Rt.add(2.5, new_g + fwd + [0, 0, 0.03], Rh_r, OPEN_H, "right: in front of the new hotend")
    Rt.add(1.2, new_g + fwd, Rh_r, OPEN_H, "right: level with it")
    Rt.add(1.5, new_g, Rh_r, OPEN_H, "right: around the new hotend")
    Rt.add(1.0, new_g, Rh_r, 0.0, "right: grip new")
    Rt.add(1.5, new_g + [0, 0, 0.040], Rh_r, 0.0, "right: lift new")
    Rt.add(1.0, new_g + [-0.02, 0, 0.040], Rh_r, 0.0, "right: out from the holder")
    out_front = seat + G + np.array([-0.022, 0.0, 0.0005])  # level with the seat, 22 mm in front, 0.5 mm high
    Rt.add(max(t_old_out + 0.5 - Rt.duration, 0.5), new_g + [-0.02, 0, 0.040], Rh_r, 0.0, "right: wait with it")
    Rt.add(2.5, out_front + [-0.02, 0, 0.0], Rh_r, 0.0, "right: toward the toolhead")
    if variant == "drop_new":
        Rt.add(0.8, grip=OPEN_H, label="right: let go over the bed")
        Rt.add(1.5, out_front + [-0.06, -0.06, 0.0], Rh_r, OPEN_H, "right: away")
        Rt.add(2.5, *starts["right"], 1.0, "right: back")
        end = max(L.duration, Rt.duration) + 3.0
        L.add(max(end - L.duration, 0.5), label="left: wait")
        Rt.add(max(end - Rt.duration, 0.5), label="right: wait")
        return L, Rt, {}
    Rt.add(1.5, out_front, Rh_r, 0.0, "right: in front of the recess")
    if variant == "half_in":  # stops 3 mm short of the seat
        Rt.add(3.0, out_front + [0.019, 0, 0], Rh_r, 0.0, "right: slide in")
        home = out_front + [0.019, 0, 0]
    else:  # slide in to the plate, then press 1.5 mm further (the compliant arm holds it against the plate)
        Rt.add(3.0, out_front + [0.022, 0, 0], Rh_r, 0.0, "right: slide in")
        Rt.add(0.8, out_front + [0.0235, 0, 0], Rh_r, 0.0, "right: press home")
        home = out_front + [0.022, 0, 0]
    Rt.add(0.6, label="right: hold")
    Rt.add(0.8, grip=OPEN_H, label="right: let go new")
    Rt.add(1.2, home + fwd, Rh_r, OPEN_H, "right: out")

    # -- right arm: close and lock the buckle -------------------------------------------------------------------------
    close_buckle(variant != "no_lock", " again")  # no_lock: swung shut but not pushed back left
    Rt.add(2.5, *starts["right"], 1.0, "right: back")
    end = max(L.duration, Rt.duration) + 3.0
    L.add(max(end - L.duration, 0.5), label="left: wait")
    Rt.add(max(end - Rt.duration, 0.5), label="right: wait")
    return L, Rt, {}


def frozen(label: str) -> bool:
    """Labels on which the arm pushes on something: the joint-space integral term is not updated."""
    return label.endswith(("grip tab", "grip old", "grip new", "grip tab again", "push right", "push right again",
                           "press shut", "press shut again", "push left to lock", "push left to lock again",
                           "hold it there", "hold it there again", "pull while locked", "back to shut", "pull out",
                           "press home", "hold", "into the slot", "slide in"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    pads = []
    for arm in kk.ARMS:
        pts = kk.STATE.fingers.get(arm)
        if pts is None:
            continue
        n = len(pts) // 2
        a, b = pts[:n].mean(axis=0), pts[n:].mean(axis=0)
        c = 1000 * ((a + b) / 2 - np.asarray(t.TH))
        pads.append(f"{arm[0].upper()} gap {1000 * np.linalg.norm(a - b) - 8:.1f} c[{c[0]:.0f},{c[1]:.0f},{c[2]:.0f}]")
    print(f"  dbg {k}: buckle {r.get('buckle_deg')} track {r.get('track_mm')} latch {r.get('latched')} "
          f"new {r.get('new_local_mm')} seat {r.get('new_seated')} old {r.get('old_local_mm')} "
          f"slot {r.get('old_slot')} "
          f"drop {r.get('old_dropped')}/{r.get('new_dropped')} off {r.get('hands_off')} hold {r.get('hold_steps')} "
          f"tilt {r.get('new_tilt')} shift {r.get('printer_shift_mm')} | {' | '.join(pads)} | {labL} | {labR}"
          + (f" | tab servo {np.round(1000 * _DBG['tab'].off, 1)}" if "tab" in _DBG else ""),
          flush=True)


def _quat_R(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class PointServo(dc.ObjectServo):
    """ObjectServo on a point of the held part (given in the part's frame) instead of its origin, with an optional
    axis mask per label set (``yz``: steer only y and z, for pressing along x)."""

    def __init__(self, arm, obj, offset, point, yz=(), **kw):
        super().__init__(arm, obj, offset, **kw)
        self.point = np.asarray(point, float)
        self.yz = set(yz)

    def update(self, kk, label: str) -> None:
        key = label.split(": ")[-1]
        if self.desired is None or self.obj not in kk.STATE.poses or key in self.hold:
            return
        if key in self.xyz or key in self.xy or key in self.yz:
            p, q = kk.STATE.poses[self.obj]
            e = self.desired - (np.asarray(p, float) + _quat_R(q) @ self.point)
            if key in self.xy:
                e[2] = 0.0
            if key in self.yz:
                e[0] = 0.0
            self.off = np.clip(self.off + self.gain * e, -self.clip, self.clip)
        else:
            self.off = np.zeros(3)

    def correct(self, pos, R, label):
        key = label.split(": ")[-1]
        self.desired = np.asarray(pos, float) + np.asarray(R, float) @ self.offset
        if key in self.xyz or key in self.xy or key in self.yz or key in self.hold:
            return np.asarray(pos, float) + self.off, R
        return pos, R


_DBG: dict = {}  # the servos, for debug()


def servos_for(name, arm):
    """Steer each held hotend (not the gripper) along the plan: the left hand's old hotend by its nozzle's tip into
    its slot, the right hand's new hotend level into the recess."""
    tip = np.array([0.0, 0.0, TIP_Z])
    if name == "left":
        R = R_front(-1.0)
        return [PointServo(arm, "hotend_old", R.T @ (-GRASP + tip), tip,
                           xyz=("away from the toolhead", "above the empty slot"), xy=("into the slot",),
                           hold=("let go old",))]
    R = R_front(-1.0)
    swings = tuple(k + tag for k in ("push right", "swing open", "swing shut", "press shut")
                   for tag in ("", " again"))
    lefts = tuple(k + tag for k in ("push left to lock", "hold it there") for tag in ("", " again"))
    out = [PointServo(arm, "hotend_new", R.T @ (-GRASP), np.zeros(3),
                       xyz=("wait with it", "toward the toolhead", "in front of the recess"),
                       yz=("slide in", "press home"), hold=("hold", "let go new")),
            # the buckle's tab (not the gripper) along the planned arc, so the soft arm's lag does not wrench the
            # buckle off its hinge's arc
            # (pushing it left: y and z only, so the hand does not press the bar into the heating assembly)
            PointServo(arm, "buckle", (0.0, 0.0, TAB_BACK), TAB_C, xyz=swings, yz=lefts,
                       hold=("let go of the tab", "let go of the tab again"), gain=0.15, clip=0.015)]
    _DBG["tab"] = out[-1]
    return out


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
