"""Fasten a prong-buckle belt at its third hole on a waist form.

On the station table a lower-torso display form (women's, 26 in waist) lies on its back with its waist toward the
arms, wearing jeans with five belt loops. A brown leather belt (30 mm wide, single-prong frame buckle, fixed keeper,
five holes) runs through all five loops and its free end is already through the buckle's frame, but the belt is not
fastened: no hole is on the prong, which lies turned back under the strap, and the tail hangs free over the form's
left side. The robot must put the prong through the third hole (counting from the tip) so that it rests on the
buckle's front bar, and let go. The reset seed picks one of three start layouts (how far the tip is drawn through the buckle and
where the buckle sits on the waist); ``BELT_START=0|1|2`` in the environment overrides it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
Demo:    python demo.py task.py demo_out [--seed N]
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import beltkit  # noqa: E402
import form  # noqa: E402
import geom as G  # noqa: E402
import measure as M  # noqa: E402
import partkit  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0           # success() itself already requires hands off for 1 s and the pull test

HOLE = 3                       # the requested hole, counted from the tip
INSTRUCTION = ("Fasten the belt at its third hole, counting from the tip: put the buckle's prong through the third "
               "hole so that it rests on the buckle's front bar, then let go of the belt.")

OBJECTS = [form.form_part(),
           beltkit.Belt(name="belt", pos=(G.X_B, 0.0, G.ZC), size=(0.03, 0.3, 0.2), color=beltkit.LEATHER,
                        layout_kind="threaded")]

CONTROL_DT = 1 / 20
HANDS_OFF_M = 0.015            # no finger, pad or palm point within 15 mm of the belt
HANDS_OFF_S = 1.0
PULL_N = 10.0                  # the pull test: 10 N straight out from the form at the right hip, for 1 s
PULL_S = 1.0
SETTLE_S = 0.5
PRONG_ON_BAR_M = 0.006         # the prong's tip rests on the front bar: tip within 6 mm of the bar's axis
BAR_GAP_M = (-0.0015, 0.015)   # the front bar lies over the strap: 0 to 15 mm off its outer face
STRAP_MOVE_M = 0.010           # under the pull the strap may move at most 10 mm through the buckle

_S: dict = {}


def _reset_state():
    _S.clear()
    _S.update(off_s=0.0, phase="idle", t=0.0, score=0.0, last={}, success=False, pull_link=None, pull_hole=0,
              pull_ref=None, held=False, tested_hole=0)


_reset_state()


def _belt(env):
    return beltkit.read(env, beltkit._SPECS["belt"])


def _point_box_dist(pt, c, R, half):
    q = np.abs(R.T @ (pt - c)) - half
    return float(np.linalg.norm(np.maximum(q, 0.0)))


def hands_off_distance(env, st) -> float:
    """Smallest distance from any gripper point (palm, finger pads) to the strap or the buckle frame."""
    pts = []
    for arm, (p, _gap) in partkit.gripper_points(env).items():
        pts += p
    best = 1e9
    for i in range(G.N_LINKS):
        R, p = st.R(f"link_{i:02d}"), st.p(f"link_{i:02d}")
        for c, s in beltkit.link_boxes(i):
            cc, half = p + R @ np.array(c), np.array(s) / 2
            for q in pts:
                best = min(best, _point_box_dist(q, cc, R, half))
    Rf, pf = st.R("frame"), st.p("frame")
    for q in pts:
        for k in np.linspace(0, 1, 6):
            for yy in (-0.02, 0.0, 0.02):
                best = min(best, float(np.linalg.norm(q - (pf + Rf @ np.array([-G.FRAME_L * k, yy, 0.0])))))
    return best


def evaluate(st) -> dict:
    """What a photo of the buckle shows (no hands-off or pull bookkeeping). Fails closed: anything that cannot be
    measured counts as not done."""
    info = dict(loops=0, hole=0, right_way=False, on_bar=False, under_bar=False, window=None, keeper=False,
                tip_bar_mm=None, bar_gap_mm=None, frame_deg=None, state="none")
    try:
        if not all(np.all(np.isfinite(st.p(n))) and np.all(np.isfinite(st.R(n))) for n in st.poses):
            return info
        loops = M.in_loops(st)
        info["loops"] = sum(loops.values())
        hole = M.hole_engaged(st)
        info["hole"] = hole
        info["right_way"] = bool(hole and M.prong_from_body_side(st, hole))
        tb = M.prong_tip_to_front_bar(st)
        info["tip_bar_mm"] = round(tb * 1000, 1)
        info["on_bar"] = tb <= PRONG_ON_BAR_M
        fb = M.front_bar_over_strap(st)
        win = M.through_window_outward(st)
        info["window"] = win
        info["bar_gap_mm"] = None if fb is None else round(fb[0] * 1000, 1)
        info["under_bar"] = bool(fb is not None and BAR_GAP_M[0] <= fb[0] <= BAR_GAP_M[1] and win is not None)
        info["frame_deg"] = round(M.frame_angle_deg(st)[0], 1)
        info["keeper"] = bool(M.through_keeper(st) is not None and M.tip_past_keeper(st))
        if not all(loops.values()):
            info["state"] = "loop_lost"
            return info
        engaged = hole > 0 and info["right_way"] and info["under_bar"]
        if engaged and info["on_bar"] and hole == HOLE:
            info["state"] = "fastened"
        elif engaged and hole == HOLE:
            info["state"] = "through"          # prong through the right hole but not laid along the frame
        else:
            info["state"] = "none"
    except Exception as ex:  # noqa: BLE001
        info["state"] = f"error {type(ex).__name__}"
    return info


def _pull(env, on: bool):
    import torch
    art = beltkit.articulation(env)
    bid = {n: i for i, n in enumerate(art.data.body_names)}
    f = torch.zeros((1, len(bid), 3), device=art.device)
    if on and _S["pull_link"] is not None:
        f[0, bid[_S["pull_link"]]] = torch.tensor((0.0, -PULL_N, 0.0), device=art.device)
    art.permanent_wrench_composer.set_forces_and_torques_index(forces=f, torques=torch.zeros_like(f), is_global=True)


def _right_hip_link(st):
    """The strap link nearest the right hip (the waist's widest point on the -y side)."""
    target = np.array([G.X_B, -G.A1, G.ZC])
    return min((float(np.linalg.norm(st.p(f"link_{i:02d}") - target)), f"link_{i:02d}")
               for i in range(2, G.HOLE_LINK[5]))[1]


def _tier(info, held: bool) -> float:
    """The score for a photo state ``info``; ``held`` says the pull test was passed at this hole."""
    s = info["state"]
    if s == "fastened" and held:
        return 1.0
    if s in ("fastened", "through"):
        return 0.25
    return 0.0


@partkit.on_reset
def _on_reset(env):
    _reset_state()
    try:
        _pull(env, False)
    except Exception:  # noqa: BLE001  (before the belt exists)
        pass


@partkit.on_step
def _on_step(env):
    st = _belt(env)
    info = evaluate(st)
    d = hands_off_distance(env, st)
    info["hands_mm"] = round(d * 1000, 1)
    info["phase"] = _S["phase"]
    _S["last"] = info
    off = d >= HANDS_OFF_M
    if not off:                                     # a gripper on the belt: nothing counts, any test starts again
        _pull(env, False)
        _S.update(off_s=0.0, score=0.0, phase="idle", t=0.0, held=False, tested_hole=0, success=False)
        return
    _S["off_s"] += CONTROL_DT
    if _S["phase"] == "idle":
        if _S["off_s"] < HANDS_OFF_S:
            return
        if info["state"] == "fastened":
            _S.update(phase="pull", t=0.0, pull_link=_right_hip_link(st), pull_hole=info["hole"],
                      pull_ref=M.hole_in_frame(st, info["hole"]).copy())
            _pull(env, True)
            print(f"PULL TEST start: hole {info['hole']} at {_S['pull_link']} {info}", flush=True)
        else:
            _S["score"] = _tier(info, False)
        return
    if _S["phase"] == "pull":
        _S["t"] += CONTROL_DT
        if _S["t"] >= PULL_S:
            _pull(env, False)
            _S.update(phase="settle", t=0.0)
        return
    if _S["phase"] == "settle":
        _S["t"] += CONTROL_DT
        if _S["t"] >= SETTLE_S:
            moved = float(np.linalg.norm(M.hole_in_frame(st, _S["pull_hole"]) - _S["pull_ref"])) if _S["pull_hole"] else 1.0
            held = (info["state"] == "fastened" and info["hole"] == _S["pull_hole"]
                    and moved <= STRAP_MOVE_M)
            _S.update(phase="done", held=held, tested_hole=_S["pull_hole"])
            _S["score"] = _tier(info, held)
            _S["success"] = _S["score"] >= 1.0
            print(f"PULL TEST done: hole {info['hole']} moved {moved * 1000:.1f} mm held {held} score {_S['score']} "
                  f"{info}", flush=True)
        return
    # done: the score stands while the state it was given for stands (hands still off)
    held = _S["held"] and info["hole"] == _S["tested_hole"]
    _S["score"] = _tier(info, held)
    _S["success"] = _S["score"] >= 1.0


def score(objects) -> float:
    return float(_S["score"])


def success(objects) -> bool:
    return bool(_S["success"])


def demo() -> dict:
    """Not a demo: an empty plan, so ``--policy yam_scripted`` only holds both arms still (a do-nothing run).
    The full-score scripted demo is demo.py."""
    return {}
