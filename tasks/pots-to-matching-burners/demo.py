"""Scripted demo for pots-to-matching-burners, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print both pots' state every 10 steps

full        Both arms take the large pot (1.5 kg) by its two side handles, lift it 4 cm off the small burner, carry
            it over to the large burner and set it down. Then the right arm pinches the small pot's near rim on the
            counter, lifts it 11 cm (over the large pot's rim), carries it across to the small burner and sets it down.
Failure and cheating variants:
drag        both arms slide the large pot across the glass to the large burner (0)
one_hand    the right arm alone moves the large pot by one handle (recorded: it slides on the glass, 0)
one_only    moves only the large pot (0.5)
wrong       the right arm puts the small pot on the large burner, which is free; the large pot stays put (0)
drop        both arms let go of the large pot 6 cm above the large burner (0)
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "drag", "one_hand", "one_only", "wrong", "drop")
_T = [None]
_CACHE: dict = {}
GRIP_DEPTH = 0.015  # m: the small pot: the TCP this far below the rim (the pads then cover the top ~3 cm of the wall)
LIFT_LARGE = 0.040  # m: the large pot's base above the glass while carried
LIFT_SMALL = 0.110  # m: the small pot's base above the glass while carried (clears the large pot's 9 cm rim)


def _closing(axis: int):
    """Straight-down and tilted grips whose closing axis (TCP x) is within ~18 deg of world x (axis 0) or y (1)."""
    return [R for tl in (0, 10, 20, 30) for az in (0, 90, 180, 270) for roll in (0, 90, 180, 270)
            for R in [dc.R_approach(tl, az, roll)] if abs(R[axis, 0]) >= 0.95]


def handle_offset(t, side: str) -> np.ndarray:
    """TCP relative to the large pot's frame for a pinch on one side handle (left: +y), 4 mm out from its centre."""
    sy = 1.0 if side == "left" else -1.0
    return np.array([0.0, sy * (t.EAR_Y + 0.004), t.EAR_Z])


def rim_offset(t) -> np.ndarray:
    """TCP relative to the small pot's frame for a pinch on its near (-x) rim: mid-wall, GRIP_DEPTH below the rim."""
    return np.array([-(t.SMALL_OUT - t.WALL / 2), 0.0, t.SMALL_H - GRIP_DEPTH])


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    _T[0] = t
    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {"left": dc.Arm("left", bases["left"], READY, *starts["left"]),
            "right": dc.Arm("right", bases["right"], READY, *starts["right"])}
    L0, S0 = np.array(t.LARGE_POS), np.array(t.SMALL_POS)
    L1 = np.array([*t.LARGE_XY, t.CT + 0.0005])  # the large pot on the large burner
    S1 = np.array([*(t.SMALL_XY if variant != "wrong" else t.LARGE_XY), t.CT + 0.0005])
    gS = rim_offset(t)
    if "grips" not in _CACHE:
        cx = _closing(0)  # handles and the near rim are pinched across x
        _CACHE["grips"] = {}
        for side in ("left", "right"):
            g = handle_offset(t, side)
            pts = [L0 + g, L0 + g + [0, 0, LIFT_LARGE + 0.06], L1 + g, L1 + g + [0, 0, LIFT_LARGE + 0.06]]
            _CACHE["grips"][side], e = dc.best_rotation(side, pts, cx)
            print(f"{side} handle grip chosen: worst IK error {1000 * e:.1f} (mm + 0.05 rad)", flush=True)
        pts = [S0 + gS, S0 + gS + [0, 0, LIFT_SMALL + 0.01], np.array([*t.SMALL_XY, t.CT]) + gS + [0, 0, LIFT_SMALL],
               np.array([*t.SMALL_XY, t.CT]) + gS, np.array([*t.LARGE_XY, t.CT]) + gS + [0, 0, LIFT_SMALL],
               np.array([*t.LARGE_XY, t.CT]) + gS]
        _CACHE["grips"]["small"], e = dc.best_rotation("right", pts, cx)
        print(f"small-pot grip chosen: worst IK error {1000 * e:.1f} (mm + 0.05 rad)", flush=True)
    G = _CACHE["grips"]

    # the large pot, small burner -> large burner: both hands, or one hand in the one_hand variant
    if variant != "wrong":
        users = ("right",) if variant == "one_hand" else ("left", "right")
        for side in users:
            A, g, R = arms[side], handle_offset(t, side), G[side]
            A.add(2.5, L0 + g + [0, 0, 0.06], R, 1.0, "large: above the handle")
            A.add(1.5, L0 + g, R, 1.0, "large: around the handle")
            A.add(1.0, L0 + g, R, 0.0, "large: grip")
            if variant == "drag":
                A.add(4.0, L1 + g, R, 0.0, "large: slide")
                last = L1 + g
            else:
                A.add(1.5, L0 + g + [0, 0, LIFT_LARGE], R, 0.0, "large: lift")
                A.add(3.0, L1 + g + [0, 0, LIFT_LARGE], R, 0.0, "large: over the large burner")
                if variant == "drop":
                    A.add(0.8, L1 + g + [0, 0, 0.06], R, 0.0, "large: up")
                    last = L1 + g + [0, 0, 0.06]
                else:
                    A.add(1.5, L1 + g + [0, 0, 0.002], R, 0.0, "large: down")
                    # press ~8 mm past contact: the soft wrists let the tilted pot seat flat before release
                    A.add(1.2, L1 + g - [0, 0, 0.008], R, 0.0, "large: onto the burner")
                    A.add(0.5, L1 + g - [0, 0, 0.008], R, 0.0, "large: settle onto the burner")
                    last = L1 + g - [0, 0, 0.001]
            A.add(0.8, grip=1.0, label="large: let go")
            A.add(1.5, last + [0, 0, 0.06], R, 1.0, "large: up and away")
            if side == "left" or variant == "one_only":
                A.add(2.0, starts[side][0], starts[side][1], 1.0, f"{side}: home")
    # the small pot, counter -> small burner, right arm, after the large pot is down
    if variant not in ("one_only",):
        R_ = arms["right"]
        if R_.duration < arms["left"].duration - 2.0:
            R_.add(arms["left"].duration - 2.0 - R_.duration, label="small: wait")
        R, hi = G["small"], t.CT + LIFT_SMALL - S0[2]
        R_.add(2.5, S0 + gS + [0, 0, 0.06], R, 1.0, "small: above the rim")
        R_.add(1.5, S0 + gS, R, 1.0, "small: around the rim")
        R_.add(1.0, S0 + gS, R, 0.0, "small: grip")
        R_.add(2.5, S0 + gS + [0, 0, hi], R, 0.0, "small: lift")
        R_.add(3.5, S1 + gS + [0, 0, LIFT_SMALL - 0.0005], R, 0.0, "small: over the burner")
        R_.add(2.5, S1 + gS + [0, 0, 0.002], R, 0.0, "small: down")
        R_.add(0.8, S1 + gS - [0, 0, 0.001], R, 0.0, "small: onto the burner")
        last = S1 + gS - [0, 0, 0.001]
        R_.add(0.8, grip=1.0, label="small: let go")
        R_.add(1.5, last + [0, 0, 0.06], R, 1.0, "small: up and away")
        R_.add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
    return arms["left"], arms["right"], {}


def servos_for(name, arm):
    t = _T[0]
    if name != "right":
        return []
    off = _CACHE["grips"]["small"].T @ -rim_offset(t)
    return [dc.ObjectServo(arm, "small_pot", off, xyz=("lift", "over the burner", "down"),
                           xy=("onto the burner",), hold=("let go",))]


def frozen(label: str) -> bool:
    return label.endswith(("grip", "onto the burner", "let go", "slide"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    lp, sp = r.get("large_pot", {}), r.get("small_pot", {})
    print(f"  dbg {k}: large L{lp.get('to_large_burner')} S{lp.get('to_small_burner')} z{lp.get('base_above_glass')} "
          f"tilt{lp.get('tilt')} | small L{sp.get('to_large_burner')} S{sp.get('to_small_burner')} "
          f"z{sp.get('base_above_glass')} tilt{sp.get('tilt')} | rel {r.get('released')} still {r.get('still')} "
          f"slid {r.get('slid')} tipped {r.get('tipped')} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
