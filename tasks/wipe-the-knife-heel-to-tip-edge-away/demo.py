"""Scripted demo for wipe-the-knife-heel-to-tip-edge-away.

    python demo.py task.py out_dir [--variant V] [--plan-only] [--no-record]
    DEMO_DEBUG=10 python demo.py ...                                # print the wipe's state every 10 steps

full         The left arm takes the knife by the butt of its handle, pinched top and bottom from behind it, lifts it off
             its rest and holds it level, edge away from the robot. The right arm takes the towel pad from its stand,
             presses it on the spine half of the blade at the heel and draws it to past the tip, then lifts it off.
Failure and cheating variants:
one_arm      the right arm wipes the knife where it lies on its rest; nobody holds it by the handle, so no pass
             counts (0)
tip_to_heel  the towel is drawn from the tip to the heel, toward the holding hand (0.5)
across_edge  the towel is drawn across the blade, from the spine over the cutting edge, at three places (0)
half         the towel is drawn only from the heel to the middle of the blade (0.5)
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "one_arm", "tip_to_heel", "across_edge", "half")
_CACHE: dict = {}
_T: dict = {}
LIFT = 0.030  # m: the knife held this far above its rest while it is wiped (higher, the right arm cannot come
# straight down over the heel)
GRIP_Y = 0.068  # m: the left TCP this far along the handle from the heel (the pads then span 5.5-9.8 cm, past the rest)
HANDLE_OPEN = 0.70  # about 6.6 cm around the 16 mm handle, top and bottom (2.5 cm clear each side on the way in)
TOWEL_OPEN = 0.60  # about 5.6 cm around the 3 cm towel pad
TOWEL_X = 0.002  # m: the towel's centre this far from the handle's axis toward the edge (its x span -1.05..+1.45 cm;
# the spine at -1.0, the edge at +2.5)
TCP_UP = 0.035  # m: the right TCP this far above the towel's underside (fingertips 2.2 cm above it, so they pass
# 1.1 cm over the handle's top at the heel)
PRESS_DEPTH = 0.004  # m: the towel's underside commanded this far below the blade's top face
S0, S1 = 0.010, 0.172  # m: the towel's centre from the heel at the start and the end of the stroke


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    L, Rt = arms["left"], arms["right"]
    k0 = np.array(t.KNIFE_POS)
    one_arm = variant == "one_arm"
    hold = k0 + ([0.0, 0.0, 0.0] if one_arm else [0.0, 0.0, LIFT])  # the knife frame's origin while wiped
    _T.update(t=t, hold=hold, variant=variant)
    grip = np.array([0.0, GRIP_Y, 0.0])
    tw0 = np.array(t.TOWEL_POS)
    up = np.array([0.0, 0.0, TCP_UP])
    top_z = t.BLADE_T / 2 - PRESS_DEPTH

    def on_blade(s, x=TOWEL_X, z=top_z):
        """Right TCP with the towel's underside centre at (x, -s, z) in the knife frame (knife held at `hold`)."""
        return hold + np.array([x, -s, z]) + up

    if "grips" not in _CACHE:
        # left: approach level along -y (from the butt), closing vertical: the pads lie on the handle's top and bottom
        # and span 4.3 cm along it, so the towel's press (about the across-handle axis) is resisted by the whole pad
        # patch, not by twist friction about the closing axis
        # exactly level (tl 90), so the closing axis is exactly vertical: tilted along the handle (tl 80 was chosen), the
        # clamping pads pitched the knife 11 deg
        lc = [R for tl in (90,) for az in range(0, 360, 5) for roll in (0, 90, 180, 270)
              for R in [dc.R_approach(tl, az, roll)] if R[1, 2] <= -0.95 and abs(R[2, 0]) >= 0.995]
        lp = [k0 + grip + [0, 0.06, 0], k0 + grip, k0 + grip + [0, 0, LIFT]]
        RL, e1 = dc.best_rotation("left", lp, lc)
        # right: from above, closing along y (the fingers on the towel's ends, none of them toward the edge)
        # straight down (tl 0), so the pad's underside is level on the blade: approaching 20 deg off vertical, the pad met
        # the blade on a corner, turned about the closing axis and fell out of the fingers
        rc = [R for tl in (0,) for az in range(0, 360, 5) for roll in (0, 90, 180, 270)
              for R in [dc.R_approach(tl, az, roll)] if abs(R[1, 0]) >= 0.995]
        rp = [tw0 + up + [0, 0, 0.06], tw0 + up, on_blade(S0) + [0, 0, 0.03], on_blade(S0), on_blade(S1),
              on_blade(S1) + [0, 0, 0.05]]
        RW, e2 = dc.best_rotation("right", rp, rc)
        _CACHE["grips"] = (RL, RW)
        print(f"handle grip {1000 * e1:.1f}, towel grip {1000 * e2:.1f} (mm + 0.05 rad)", flush=True)
        print(f"RL close {np.round(RL[:, 0], 3).tolist()} appr {np.round(RL[:, 2], 3).tolist()} | RW close "
              f"{np.round(RW[:, 0], 3).tolist()} appr {np.round(RW[:, 2], 3).tolist()}", flush=True)
    RL, RW = _CACHE["grips"]

    if not one_arm:
        L.add(2.5, k0 + grip + [0, 0.06, 0.02], RL, HANDLE_OPEN, "knife: behind the handle")
        L.add(1.5, k0 + grip + [0, 0.06, 0], RL, HANDLE_OPEN, "knife: level with the handle")
        L.add(2.0, k0 + grip, RL, HANDLE_OPEN, "knife: around the handle")
        L.add(1.0, k0 + grip, RL, 0.0, "knife: grip")
        L.add(2.0, hold + grip, RL, 0.0, "knife: lift")
    Rt.add(2.5, tw0 + up + [0, 0, 0.06], RW, TOWEL_OPEN, "towel: above")
    Rt.add(1.5, tw0 + up, RW, TOWEL_OPEN, "towel: around")
    Rt.add(1.0, tw0 + up, RW, 0.0, "towel: grip")
    Rt.add(1.5, tw0 + up + [0, 0, 0.06], RW, 0.0, "towel: lift")
    if not one_arm and Rt.duration < L.duration:
        Rt.add(L.duration - Rt.duration, label="towel: wait for the knife")
    if variant == "across_edge":
        for i, s in enumerate((0.04, 0.09, 0.14)):
            Rt.add(2.5 if i == 0 else 1.5, on_blade(s, x=-0.004) + [0, 0, 0.03], RW, 0.0, f"towel: above the spine {i}")
            Rt.add(1.0, on_blade(s, x=-0.004), RW, 0.0, f"towel: onto the spine {i}")
            Rt.add(1.5, on_blade(s, x=0.040), RW, 0.0, f"towel: across the edge {i}")
            Rt.add(0.8, on_blade(s, x=0.040) + [0, 0, 0.04], RW, 0.0, f"towel: up {i}")
    else:
        a, b = (S1, S0) if variant == "tip_to_heel" else (S0, 0.090 if variant == "half" else S1)
        Rt.add(2.5, on_blade(a) + [0, 0, 0.03], RW, 0.0, "towel: above the blade")
        Rt.add(1.2, on_blade(a), RW, 0.0, "towel: onto the blade")
        Rt.add(0.5, on_blade(a), RW, 0.0, "towel: press")
        Rt.add(3.5, on_blade(b), RW, 0.0, "towel: draw along the blade")
        Rt.add(0.8, on_blade(b) + [0, 0, 0.05], RW, 0.0, "towel: off the blade")
    Rt.add(2.5, tw0 + up + [0, 0, 0.06], RW, 0.0, "towel: over the stand")
    Rt.add(0.8, grip=TOWEL_OPEN, label="towel: let go")
    Rt.add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
    if not one_arm:
        L.add(max(0.5, Rt.duration - L.duration - 6.0), hold + grip, RL, 0.0, "knife: hold")
        L.add(2.0, k0 + grip + [0, 0, 0.003], RL, 0.0, "knife: back on the rest")
        L.add(0.8, grip=HANDLE_OPEN, label="knife: let go")
        L.add(1.5, k0 + grip + [0, 0.06, 0], RL, HANDLE_OPEN, "knife: out from the handle")
        L.add(2.0, starts["left"][0], starts["left"][1], 1.0, "left: home")
    return L, Rt, {}


def servos_for(name, arm):
    if name == "left":
        return [TcpServo("left_robot")] if _T.get("variant") != "one_arm" else []
    return [TcpServo("right_robot", approach=("above", "around"), carry=()),
            *([] if _T.get("variant") == "one_arm" else [KnifeFollow()])]


class TcpServo:
    """Steers the arm's measured TCP (from its finger pads) onto the planned TCP on the way to a grasp, so the pinch
    lands where planned (the YAM arms track 5-15 mm off, more when folded low); the correction is kept through the grip
    and fades out after."""

    PAD_C = np.array([0.0, 0.0, -0.008])  # the finger pads' centroid in the TCP frame (kitchenkit.PAD_Z mean)

    def __init__(self, robot, approach=("behind the handle", "level with the handle", "around the handle"),
                 carry=("lift",)):
        self.robot, self.approach, self.carry = robot, tuple(approach), tuple(carry)
        self.off, self.cmd, self.R = np.zeros(3), None, None

    def update(self, kk, label: str) -> None:
        key = label.split(": ")[-1]
        if key in ("grip", "hold"):
            return
        if key not in self.approach + self.carry:
            self.off = 0.8 * self.off
            return
        pts = kk.STATE.fingers.get(self.robot)
        if self.cmd is None or pts is None or len(pts) == 0:
            return
        meas = np.asarray(pts, float).mean(axis=0) - self.R @ self.PAD_C
        gain = 0.1 if key in self.approach else 0.05
        self.off = np.clip(self.off + gain * (self.cmd - meas), -0.03, 0.03)

    def correct(self, pos, R, label):
        self.cmd, self.R = np.asarray(pos, float), np.asarray(R, float)
        return self.cmd + self.off, R


class KnifeFollow:
    """While the towel goes onto, presses and is drawn along the blade, its target moves with the knife as the left
    hand actually holds it (the plan assumes the knife exactly at its hold pose; the holding arm's 5-15 mm tracking error
    would put the towel off the spine half)."""

    KEYS = ("above the blade", "onto the blade", "press", "draw along the blade", "off the blade")

    def __init__(self):
        self.corr = np.zeros(3)

    def update(self, kk, label: str) -> None:
        key = label.split(": ")[-1]
        if not (key in self.KEYS or key.startswith(("above the spine", "onto the spine", "across the edge"))):
            self.corr = 0.8 * self.corr
            return
        if "knife" not in kk.STATE.poses:
            return
        p, R = kk.pose_of("knife")
        self.p, self.R = np.asarray(p, float), np.asarray(R, float)
        self.live = True

    def correct(self, pos, R, label):
        key = label.split(": ")[-1]
        hold = _T["hold"]
        if getattr(self, "live", False) and (key in self.KEYS or key.startswith(("above the spine", "onto the spine",
                                                                                  "across the edge"))):
            rel = np.asarray(pos, float) - hold  # the planned TCP in the (planned, unrotated) knife frame
            target = self.p + self.R @ rel
            self.corr = np.clip(target - np.asarray(pos, float), -0.04, 0.04)
            self.live = False
        return np.asarray(pos, float) + self.corr, R


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "knife: hold")) or label.startswith(("towel: press", "towel: draw",
                                                                                    "towel: onto"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    print(f"  dbg {k}: F {r.get('force')} s {r.get('towel_s')} x {r.get('towel_x')} gap {r.get('towel_gap')} cov "
          f"{r.get('coverage')} h2t {r.get('heel_to_tip')} edge {r.get('edge_breached')} crossed "
          f"{r.get('towel_crossed_edge')} drop {r.get('drop_flag')} held {r.get('held')} tilt {r.get('knife_tilt')} knife "
          f"{np.round(kk.pose_of('knife', t.KNIFE_POS)[0], 4).tolist()} | {labL} | {labR}",
          flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
