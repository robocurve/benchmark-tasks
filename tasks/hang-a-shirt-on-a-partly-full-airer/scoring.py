"""Scoring: the end state of the T-shirt, the two face cloths and the hands. NumPy only (testable offline).

Real trial and simulation use the same rules; in simulation every quantity comes from the fabric's vertices.

Score (judged at the end of the run; the policy must have let go and the cloths must have stopped moving):

* 0     the T-shirt touches the table, or is not held up by the airer (on the floor, still in a hand), or a face
        cloth has come off its rail or touches the table.
* 0.25  the T-shirt hangs from the airer, clear of the table, and the hands have let go of it.
* 0.5   and it hangs over exactly one rail, a free one (rail B): it rests on no other rail and on neither face
        cloth's rail, and each half hangs at least ``MIN_DROP`` (80 mm) below the top of that rail (it is folded over
        the rail, not lying across the airer).
* 0.75  and it is spread: where it lies on the rail it covers at least ``MIN_SPAN`` of the rail's length (85 % of
        its 220 mm flat body width: 187 mm), so it hangs open, not gathered into a rope.
* 1.0   and both face cloths are undisturbed: every corner of each is within ``MAX_CLOTH_MOVE`` (30 mm) of where
        it started, and both hands are open and at least ``HAND_CLEAR`` (50 mm) from the T-shirt.

"Rests on a rail" means part of the T-shirt lies on top of that rail, or on top of the cloth hanging on it: within
``REST_BAND`` of the rail's top surface (on the rail itself, or on a 3 mm terry cloth over it), above the rail's axis.
Touching the side of a neighbouring cloth that hangs into the same gap does not count.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

import layout as L

MIN_DROP = 0.080
MIN_SPAN = 0.85 * L.TEE_W
MAX_CLOTH_MOVE = 0.030
HAND_CLEAR = 0.050
TABLE_TOUCH = 0.004  # a vertex this close to the table top touches it (the fabric's contact rest offset is ~0.4 mm)
REST_BAND = 0.010  # beyond a rail's radius: fabric on the rail (6 mm) or on the terry cloth over it (+3 mm)


@dataclass
class Result:
    score: float
    metrics: dict


def _on_top(points: np.ndarray, rail: str, extra: float = 0.0) -> np.ndarray:
    """Which ``points`` lie on top of ``rail``: within REST_BAND (+extra) of its surface, above its axis."""
    dx = points[:, 0] - L.RAIL_X[rail]
    dz = points[:, 2] - L.RAIL_Z
    r = np.hypot(dx, dz)
    return (r <= L.RAIL_R + REST_BAND + extra) & (dz > 0.0) & (np.abs(points[:, 1]) < L.RAIL_LEN / 2)


def rests_on(points: np.ndarray, rail: str, extra: float = 0.0) -> bool:
    """Whether any of ``points`` lies on top of ``rail``."""
    return bool(_on_top(points, rail, extra).any())


def evaluate(tee: np.ndarray, cloths: dict, cloth_corners_start: dict, cloth_corner_idx: dict,
             hands: dict) -> Result:
    """``tee``: the T-shirt's vertices (V x 3, env frame); ``cloths``: {rail: vertices} of the face cloths;
    ``cloth_corners_start``: {rail: 4 x 3}; ``cloth_corner_idx``: {rail: 4 vertex indices}; ``hands``: {arm: (TCP
    position, finger gap, holding: bool)}."""
    top = L.RAIL_Z + L.RAIL_R
    m: dict = {}
    m["tee_low"] = float(tee[:, 2].min() - L.TOP)
    m["tee_on_table"] = bool(m["tee_low"] < TABLE_TOUCH)
    m["held"] = any(h[2] for h in hands.values())
    rails = [r for r in L.RAIL_X if rests_on(tee, r)]
    m["tee_rests_on"] = rails
    cloth_ok, worst_move = True, 0.0
    for rail, pts in cloths.items():
        move = float(np.linalg.norm(pts[cloth_corner_idx[rail]] - cloth_corners_start[rail], axis=1).max())
        worst_move = max(worst_move, move)
        on_rail = rests_on(pts, rail, extra=0.004)
        on_table = bool(pts[:, 2].min() - L.TOP < TABLE_TOUCH)
        m[f"cloth_{rail}_move_mm"] = round(move * 1000, 1)
        m[f"cloth_{rail}_on_rail"] = on_rail
        cloth_ok &= on_rail and not on_table
    m["cloths_on_rails"] = bool(cloth_ok)
    m["cloth_worst_move_mm"] = round(worst_move * 1000, 1)
    hanging = bool(rails) and not m["tee_on_table"] and not m["held"]
    m["hangs_from_airer"] = hanging
    # the halves, either side of the one rail it rests on
    one = len(rails) == 1 and rails[0] == L.FREE_RAIL
    if len(rails) == 1:
        x = L.RAIL_X[rails[0]]
        front, back = tee[tee[:, 0] < x - L.RAIL_R], tee[tee[:, 0] > x + L.RAIL_R]
        m["drop_front_mm"] = round(float(top - front[:, 2].min()) * 1000, 1) if len(front) else 0.0
        m["drop_back_mm"] = round(float(top - back[:, 2].min()) * 1000, 1) if len(back) else 0.0
        on = _on_top(tee, rails[0])
        m["span_mm"] = round(float(np.ptp(tee[on, 1])) * 1000, 1)
    else:
        m["drop_front_mm"] = m["drop_back_mm"] = m["span_mm"] = 0.0
    folded = one and min(m["drop_front_mm"], m["drop_back_mm"]) >= MIN_DROP * 1000
    spread = m["span_mm"] >= MIN_SPAN * 1000
    clear = min(float(np.linalg.norm(tee - h[0], axis=1).min()) for h in hands.values())
    m["hand_clear_mm"] = round(clear * 1000, 1)
    m["hands_open"] = all(h[1] > 0.010 for h in hands.values())
    undisturbed = worst_move <= MAX_CLOTH_MOVE and m["hands_open"] and clear >= HAND_CLEAR

    score = 0.0
    if hanging and cloth_ok:
        score = 0.25
        if folded:
            score = 0.5
            if spread:
                score = 0.75
                if undisturbed:
                    score = 1.0
    m["one_free_rail"] = bool(one)
    m["folded_over"] = bool(folded)
    m["spread"] = bool(spread)
    m["score"] = score
    return Result(score, m)
