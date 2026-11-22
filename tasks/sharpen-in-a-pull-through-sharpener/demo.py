"""Scripted demo for sharpen-in-a-pull-through-sharpener, run in the yam_arena embodiment (fixed layout).

    python demo.py task.py OUT_DIR [--variant full] [--no-record]   # demo.mp4, setup.png, result.json
    python demo.py task.py --plan-only                              # IK check of every variant's plan
    DEMO_DEBUG=10 python demo.py ...                                # print the sharpening state every 10 steps

full        The left arm pinches the sharpener's handle and holds it; the right arm lifts the knife out of its dock,
            sets the heel in the coarse slot, presses and pulls the knife toward the robot from heel to tip, lifts it
            out and starts again (three passes), then does three passes in the fine slot, and puts the knife back in its
            dock; the left arm lets go.
Failure and cheating variants:
one_arm     the same passes with nobody holding the sharpener (expected: it slides along with the knife)
saw         as full, but in the coarse slot the knife is pushed back to the heel while still pressed (0)
fine_first  three passes in the fine slot, then three in the coarse slot (0.5)
light       as full, but the blade only skims the slots without pressing (0)
no_return   as full, but the knife is kept in the hand at the end (0.75)
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import democore as dc  # noqa: E402

VARIANTS = ("full", "one_arm", "saw", "fine_first", "light", "no_return")
_CACHE: dict = {}
PRESS_DEPTH = 0.018  # m: the edge commanded this far below the slot's floor (10 mm gave about 1.6 N)
PULL = 0.160  # m: the knife pulled this far toward the robot in one pass
S0 = 0.005  # m: the slots' centre line this far along the edge from the heel when a pass starts
ABOVE = 0.035  # m: the edge this far above the slot's floor between passes
PASSES = 3


def _knife_grip(pitch: float, flip: bool) -> np.ndarray:
    """TCP rotation in the knife frame: pinched across the blade's normal from above, tilted back over the handle."""
    x_T = np.array([0.0, 1.0 if flip else -1.0, 0.0])
    z_T = dc.rot((0, 1, 0), -math.radians(pitch)) @ np.array([0.0, 0.0, -1.0])
    return np.column_stack([x_T, np.cross(z_T, x_T), z_T])


def build_plan(t, variant: str = "full", params: dict | None = None):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    bases = station.BASES
    starts = {s: ik.tcp_pose(np.array(READY), bases[s]) for s in ("left", "right")}
    arms = {s: dc.Arm(s, bases[s], READY, *starts[s]) for s in ("left", "right")}
    L, Rt = arms["left"], arms["right"]
    sh = np.array(t.SH_POS)
    floor = sh[2] + t.BASE_TOP  # the slots' floor (world z)
    hgrip = sh + np.array(t.SH_HANDLE_C) + [0.0, 0.0, t.SH_HANDLE[2] / 2 - 0.012]  # TCP on the handle, from above
    k0 = np.array(t.KNIFE_POS)

    def heel_at(slot, s, dz):
        """Knife pose (heel position) with the slots' centre line s along the edge and the edge dz above the floor."""
        return np.array([sh[0] + t.SLOT_X - s, sh[1] + t.SLOTS[slot], floor + dz])

    if "grips" not in _CACHE:
        lc = [R for tl in (0, 10, 20, 30) for az in range(0, 360, 45) for roll in (0, 90, 180)
              for R in [dc.R_approach(tl, az, roll)] if abs(R[0, 0]) >= 0.95]
        RL, e1 = dc.best_rotation("left", [hgrip, hgrip + [0, 0, 0.05]], lc)
        kc = [_knife_grip(p, f) for p in (30, 40, 50, 60, 70) for f in (False, True)]
        heels = [k0, k0 + [0, 0, 0.06]]
        for slot in t.SLOTS:
            heels += [heel_at(slot, S0, ABOVE), heel_at(slot, S0, -PRESS_DEPTH), heel_at(slot, S0 + PULL, -PRESS_DEPTH),
                      heel_at(slot, S0 + PULL, ABOVE)]
        best = None
        for G in kc:
            pts = [h + t.KNIFE_GRASP for h in heels]
            _, err = dc.best_rotation("right", pts, [G])
            if best is None or err < best[1]:
                best = (G, err)
        _CACHE["grips"] = (RL, best[0])
        print(f"handle grip {1000 * e1:.1f}, knife grip {1000 * best[1]:.1f} (mm + 0.05 rad)", flush=True)
    RL, GK = _CACHE["grips"]

    def knife_to(dur, heel, label, grip=0.0):
        Rt.add(dur, np.asarray(heel, float) + t.KNIFE_GRASP, GK, grip, label)

    def knife_line(dur, h0, h1, label):
        h0, h1 = np.asarray(h0, float), np.asarray(h1, float)
        Rt.add(dur, grip=0.0, label=label, path=lambda u: (h0 + u * (h1 - h0) + t.KNIFE_GRASP, GK))

    if variant != "one_arm":
        L.add(2.5, hgrip + [0, 0, 0.05], RL, 1.0, "sharpener: above the handle")
        L.add(1.5, hgrip, RL, 1.0, "sharpener: around the handle")
        L.add(1.0, hgrip, RL, 0.0, "sharpener: grip")
    knife_to(3.0, k0 + [0, 0, 0.05], "knife: above the handle", 1.0)
    knife_to(1.5, k0, "knife: around the handle", 1.0)
    knife_to(1.0, k0, "knife: grip", 0.0)
    knife_to(1.5, k0 + [0, 0, 0.06], "knife: out of the dock")
    depth = -0.003 if variant == "light" else PRESS_DEPTH
    order = ["fine", "coarse"] if variant == "fine_first" else ["coarse", "fine"]
    first = True
    for slot in order:
        for i in range(PASSES):
            knife_to(2.5 if first else 1.2, heel_at(slot, S0, ABOVE), f"{slot} {i + 1}: heel over the slot")
            if first:
                Rt.add(max(0.0, L.duration - Rt.duration), label=f"{slot} {i + 1}: wait for the hold")
                first = False
            knife_to(0.8, heel_at(slot, S0, 0.002), f"{slot} {i + 1}: heel in the slot")
            knife_to(0.4, heel_at(slot, S0, -depth), f"{slot} {i + 1}: press")
            knife_line(2.5, heel_at(slot, S0, -depth), heel_at(slot, S0 + PULL, -depth), f"{slot} {i + 1}: pull")
            if variant == "saw" and slot == "coarse":
                knife_line(2.0, heel_at(slot, S0 + PULL, -depth), heel_at(slot, S0, -depth), f"{slot} {i + 1}: push")
                knife_to(0.6, heel_at(slot, S0, ABOVE), f"{slot} {i + 1}: lift")
            else:
                knife_to(0.6, heel_at(slot, S0 + PULL, ABOVE), f"{slot} {i + 1}: lift")
    if variant == "no_return":
        knife_to(2.0, heel_at("coarse", S0, 0.10), "knife: up")
        Rt.add(2.0, label="knife: keep")
    else:
        knife_to(2.5, k0 + [0, 0, 0.05], "knife: over the dock")
        knife_to(1.5, k0 + [0, 0, 0.002], "knife: down")
        Rt.add(0.8, grip=1.0, label="knife: let go")
        knife_to(1.2, k0 + [0, 0, 0.06], "knife: up", 1.0)
        Rt.add(2.0, starts["right"][0], starts["right"][1], 1.0, "right: home")
    if variant != "one_arm":
        L.add(max(0.5, Rt.duration - L.duration - 3.0), hgrip, RL, 0.0, "sharpener: hold")
        L.add(0.8, grip=1.0, label="sharpener: let go")
        L.add(1.5, hgrip + [0, 0, 0.05], RL, 1.0, "sharpener: up")
        L.add(2.0, starts["left"][0], starts["left"][1], 1.0, "left: home")
    return L, Rt, {}


def servos_for(name, arm):
    return []


def frozen(label: str) -> bool:
    return label.endswith(("grip", "let go", "sharpener: hold", ": press", ": pull", ": push"))


def debug(k, t, kk, labL, labR):
    r = t.report()
    last = r.get("passes", [])[-1:] or None
    print(f"  dbg {k}: F {r.get('force')} slot {r.get('slot')} s {r.get('s')} coarse {r.get('coarse_passes')} fine "
          f"{r.get('fine_passes')} early {r.get('fine_before_coarse')} saw {r.get('sawing')} hand {r.get('hand_at_slots')}"
          f" shift {r.get('sharpener_shift')} dock {r.get('knife_in_dock')} last {last} | {labL} | {labR}", flush=True)


if __name__ == "__main__":
    dc.main(build_plan, VARIANTS, servos_for, frozen, debug)
