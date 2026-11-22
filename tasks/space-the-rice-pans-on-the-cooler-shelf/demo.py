"""Scripted demo for space-the-rice-pans-on-the-cooler-shelf, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the pans' state every 10 steps

full        Both arms pinch the rim at the two ends of the pan in the middle of the shelf from above (one finger
            inside on the end wall, one outside on the flange's edge), lift the pan, move it to the shelf's right end and
            let go; then they pinch the rim of the pan on the hot-holding block, lift it, carry it level onto the
            shelf's left end, aimed 9.6 cm clear of the first, and let go.
Failure and cheating variants:
squeeze     the first pan is left in the middle; the second is set down beside it, hanging off the shelf's end (0)
one_arm     the right arm alone lifts the hot pan by the rim at its right end (it tipped, slipped out of the single
            pinch on the way over and came to rest mostly off the shelf, 0)
Not recorded: pans set down closer than 5 cm (0.5), touching (0.25) and stacked (0). scorer_check.py feeds those end
states to task.py's scorer directly.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "squeeze", "one_arm")
_CACHE: dict = {}
_G_OFF: dict = {}  # side -> the pan's underside centre to the TCP when pinching its rim (world frame), for PanServo
_GRIP_FILE = pathlib.Path(__file__).resolve().parent / "grips_cache.json"  # optional: the grip search takes ~20 min


def _grip_key(t, variant, dest, grip_offset) -> str:
    import hashlib
    import json
    r = lambda v: None if v is None else [round(float(x), 5) for x in v]  # noqa: E731
    s = json.dumps(["rim", variant, {k: r(v) for k, v in sorted(dest.items())},
                    {k: r(v) for k, v in sorted(t.PAN_POS.items())},
                    [r(grip_offset(t, side)) for side in ("left", "right")]])
    return hashlib.sha1(s.encode()).hexdigest()[:16]


LIFT = 0.040  # m: a pan raised this far off what it stands on before it is carried
CLEAR = 0.045  # m: a carried pan's underside this far above the shelf top (its front lip is 1.5 cm)
OPEN = 0.5  # gripper opening around the rim (4.7 cm between the pads)
RIM_DEPTH = 0.015  # m: the TCP this far below the rim when pinching it (pad tips up to 2.8 cm below, above the rice)


def _closing(axis: int, tilts=(0, 10, 20, 30)):
    return [R for tl in tilts for az in range(0, 360, 45) for roll in (0, 90)
            for R in [dc.R_approach(tl, az, roll)] if abs(R[axis, 0]) >= 0.95]


def grip_offset(t, side: str) -> np.ndarray:
    """The TCP when pinching the rim at an end, fingers from above: one pad on the end wall's inner face, the other on
    the flange's outer edge (the pinch closes along y across wall and flange, 2.1 cm), pad tips up to 2.8 cm below the
    rim (clear of the rice, 3 cm deep: 2.9 cm below the rim)."""
    sy = 1.0 if side == "left" else -1.0
    return np.array([0.0, sy * (t.PAN_L / 2 + (t.FLANGE_W - t.PAN_T) / 2), t.PAN_H - RIM_DEPTH])


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    shelf_z = t.SHELF_TOP + 0.0005
    a_y = {"full": -t.SLOT_Y}.get(variant)  # where the first pan goes (None: left in the middle)
    b_y = {"full": t.SLOT_Y, "squeeze": 0.33, "one_arm": t.SLOT_Y}[variant]
    dest = {"pan_a": None if a_y is None else np.array([t.SHELF_XY[0], a_y, shelf_z]),
            "pan_b": np.array([t.SHELF_XY[0], b_y, shelf_z])}
    carry_z = t.SHELF_TOP + CLEAR
    _G_OFF.update({side: grip_offset(t, side) for side in ("left", "right")})

    def pan_path(name):
        p0, p4 = np.array(t.PAN_POS[name]), dest[name]
        z = max(p0[2] + LIFT, carry_z, p4[2] + CLEAR)
        return p0, p0 + [0, 0, LIFT], np.array([*p0[:2], z]), np.array([*p4[:2], z]), p4

    key = _grip_key(t, variant, dest, grip_offset)
    if variant not in _CACHE and _GRIP_FILE.exists():
        import json
        saved = json.loads(_GRIP_FILE.read_text()).get(key)
        if saved:
            _CACHE[variant] = {tuple(k.split("|")): (np.array(v[0]), np.array(v[1])) for k, v in saved.items()}
            print(f"rim grips loaded from {_GRIP_FILE.name}", flush=True)
    if variant not in _CACHE:
        rimc = [R for tl in range(0, 75, 10) for az in range(0, 360, 15) for roll in (0, 90, 180, 270)
                for R in [dc.R_approach(tl, az, roll)] if abs(R[1, 0]) >= 0.95]
        ry = lambda d: dc.rot([0.0, 1.0, 0.0], math.radians(d))  # noqa: E731
        G = {}  # each pan gets its own grips: one rotation to pick it up, one to set it down
        for n in ("pan_a", "pan_b"):
            if dest[n] is None:
                continue
            p0, p1, p2, p3, p4 = pan_path(n)
            for side in ("left", "right"):
                g = grip_offset(t, side)
                pick = [p0 + g, p0 + g + [0, 0, 0.06], p1 + g, p2 + g]
                place = [p3 + g, p4 + g, p4 + g - [0, 0, 0.004], p4 + g + [0, 0, 0.06]]
                # on the way the wrist turns about the closing axis (y): the rim pivots in the pinch and the pan, held
                # at both ends on that axis, hangs level. Ranked from the set-down side (an arm reaching along the
                # shelf), then the pick-up rotation found as a turn about the closing axis from it
                # the smallest turn wins: held in a firm pinch the pan turns with the wrists, and a 30-degree turn on
                # the way tipped it past 30 degrees (rice spilled)
                ranked = sorted((dc.best_rotation(side, place, [R])[1], i) for i, R in enumerate(rimc))[:8]
                best = None
                for e2, i in ranked:
                    if e2 > 0.002:
                        continue
                    for d in sorted(range(-60, 61, 5), key=abs):
                        Rp_, e1_ = dc.best_rotation(side, pick, [ry(d) @ rimc[i]])
                        if e1_ <= 0.002:
                            break
                    cand = (abs(d), max(e1_, e2), Rp_, rimc[i], e1_, e2)
                    if best is None or cand[:2] < best[:2]:
                        best = cand
                _, _, Rp, Rs, e1, e2 = best
                G[(side, n)] = (Rp, Rs)
                print(f"{side} rim grip on {n}: worst IK error {1000 * e1:.1f} picking up, {1000 * e2:.1f} setting "
                      f"down (mm + 0.05 rad)", flush=True)
        _CACHE[variant] = G
        try:
            import json
            data = json.loads(_GRIP_FILE.read_text()) if _GRIP_FILE.exists() else {}
            data[key] = {"|".join(k): [np.asarray(v[0]).tolist(), np.asarray(v[1]).tolist()] for k, v in G.items()}
            _GRIP_FILE.write_text(json.dumps(data))
        except OSError:
            pass
    G = _CACHE[variant]

    def carry(name, users=("left", "right")):
        p0, p1, p2, p3, p4 = pan_path(name)
        for side in users:
            A, g, (R, Rs) = arms[side], grip_offset(t, side), G[(side, name)]
            A.add(2.5, p0 + g + [0, 0, 0.06], R, OPEN, f"{name}: above the rim")
            A.add(1.5, p0 + g, R, OPEN, f"{name}: around the rim")
            A.add(1.0, p0 + g, R, 0.0, f"{name}: grip")
            A.add(2.5, p1 + g, R, 0.0, f"{name}: lift")
            A.add(3.0, p2 + g, R, 0.0, f"{name}: up")
            A.add(8.0, p3 + g, Rs, 0.0, f"{name}: over the shelf")
            A.add(3.0, p4 + g + [0, 0, 0.003], Rs, 0.0, f"{name}: down")
            A.add(1.0, p4 + g - [0, 0, 0.004], Rs, 0.0, f"{name}: onto the shelf")
            A.add(0.5, p4 + g - [0, 0, 0.004], Rs, 0.0, f"{name}: settle")
            A.add(0.8, grip=OPEN, label=f"{name}: let go")
            A.add(1.5, p4 + g + [0, 0, 0.06], Rs, OPEN, f"{name}: up from the rim")

    if variant == "one_arm":
        carry("pan_b", users=("right",))
        arms["right"].add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
        return arms["left"], arms["right"], {}
    if dest["pan_a"] is not None:
        carry("pan_a")
        for side in ("left", "right"):  # back to the ready pose between the pans (no wrist flip on the way)
            arms[side].add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: clear")
    carry("pan_b")
    for side in ("left", "right"):
        arms[side].add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: home")
    return arms["left"], arms["right"], {}


def servos_for(name, arm):
    return [PanServo(name), TcpServo(f"{name}_robot")]


class PanServo:
    """Steers the carried pan, not the hand, onto the plan: on the carry over the shelf and the way down, both arms add
    the same correction (the pan's planned position less its measured one, integrated), so the hands move together and
    do not strain the pinch. Without it the pan arrived about 4 cm forward of its plan and came down on the shelf's
    front lip. The correction is kept while setting down and letting go, and dropped after."""

    STEER = ("over the shelf", "down")
    KEEP = ("onto the shelf", "settle", "let go", "up from the rim")

    def __init__(self, side: str, gain: float = 0.05, clip: float = 0.05):
        self.side, self.gain, self.clip = side, gain, clip
        self.off, self.desired, self.pan = np.zeros(3), None, None

    def update(self, kk, label: str) -> None:
        pan, _, key = label.partition(": ")
        if key in self.KEEP:
            return
        if key in self.STEER and self.desired is not None and pan == self.pan and pan in kk.STATE.poses:
            e = self.desired - np.asarray(kk.STATE.poses[pan][0], float)
            self.off = np.clip(self.off + self.gain * e, -self.clip, self.clip)
        else:
            self.off = np.zeros(3)

    def correct(self, pos, R, label):
        pan, _, key = label.partition(": ")
        if key in self.STEER + self.KEEP and pan.startswith("pan_"):
            self.pan = pan
            self.desired = np.asarray(pos, float) - _G_OFF[self.side]
            return np.asarray(pos, float) + self.off, R
        return pos, R


class TcpServo:
    """Steers the arm's measured TCP (from its finger pads) onto the planned TCP on the way to the rim, so the pinch
    lands on it (folded low by its base, an arm can track a few cm off its plan). The correction is then held, not
    updated, while the pan is carried and set down and the fingers rise off the rim, and fades out after."""

    APPROACH = ("above the rim", "around the rim")
    CARRY = ()  # no TCP correction while both hands hold a pan (each arm steering to its own plan strained the pinch)
    HOLD = ("grip", "lift", "up", "over the shelf", "down", "onto the shelf", "settle", "let go",
            "up from the rim")  # the approach correction is kept until the open fingers are off the rim
    PAD_C = np.array([0.0, 0.0, -0.008])  # the finger pads' centroid in the TCP frame (kitchenkit.PAD_Z mean)

    def __init__(self, robot: str):
        self.robot, self.off, self.cmd, self.R = robot, np.zeros(3), None, None

    def update(self, kk, label: str) -> None:
        key = label.split(": ")[-1]
        if key in self.HOLD:
            return
        if key not in self.APPROACH + self.CARRY:
            self.off = 0.8 * self.off
            return
        pts = kk.STATE.fingers.get(self.robot)
        if self.cmd is None or pts is None or len(pts) == 0:
            return
        meas = np.asarray(pts, float).mean(axis=0) - self.R @ self.PAD_C
        gain = 0.1
        self.off = np.clip(self.off + gain * (self.cmd - meas), -0.05, 0.05)

    def correct(self, pos, R, label):
        self.cmd, self.R = np.asarray(pos, float), np.asarray(R, float)
        return self.cmd + self.off, R


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "onto the shelf", "settle"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    pans = r.get("pans", {})
    s = " ".join(f"{n}: shelf {p.get('on_shelf')} share {p.get('shelf_share')} tilt {p.get('tilt')} y {p.get('y')} "
                 f"rel {p.get('released')}" for n, p in pans.items())
    pads = []
    for n in t.PAN_POS:  # finger pad points near a pan, in that pan's frame, cm
        p, R = kk.pose_of(n, t.PAN_POS[n])
        for arm, pts in sorted(kk.STATE.fingers.items()):
            q = (np.asarray(pts, float) - p) @ R
            if np.abs(q[:, 1]).min() < 0.25 and np.abs(q[:, 0]).min() < 0.15 and q[:, 2].min() < 0.12:
                pads.append(f"{n[-1]}{arm[0]} y {100 * q[:, 1].min():.1f}..{100 * q[:, 1].max():.1f} "
                            f"z {100 * q[:, 2].min():.1f}..{100 * q[:, 2].max():.1f}")
    print(f"  dbg {k}: placed {r.get('placed')} gap {r.get('gap')} stacked {r.get('stacked')} max_tilt {r.get('max_tilt')} "
          f"spill {r.get('spilled')} drop {r.get('drop_flag')} | {s} | {' ; '.join(pads)} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
