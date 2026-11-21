"""Offline checks of the scoring rules on synthetic end states (no simulator):

    python -m pytest tasks/hang-a-shirt-on-a-partly-full-airer/test_scoring.py
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import layout as L  # noqa: E402
import scoring  # noqa: E402

TEE = L.tshirt()
CLOTHS = {r: L.washcloth(r) for r in L.LOADED}
CORNERS = {r: m.marks["corners"] for r, m in CLOTHS.items()}
CORNER_START = {r: m.start[m.marks["corners"]] for r, m in CLOTHS.items()}
FAR = {"left": (np.array([0.25, 0.2, 1.0]), 0.094, False), "right": (np.array([0.25, -0.2, 1.0]), 0.094, False)}


def tee_over(rail: str, offset: float = 0.0, squash: float = 1.0) -> np.ndarray:
    """The T-shirt folded over ``rail`` at its middle (plus ``offset`` toward the hem), its width scaled by
    ``squash``: flat planar coordinates (v along the shirt, u across), draped like a cloth."""
    uv = np.r_[TEE.marks["uv"], TEE.marks["uv"]]  # front and back layers (they coincide here)
    s = uv[:, 1] - (L.TEE_L / 2 + offset)
    return L.draped(np.c_[s, uv[:, 0] * squash], L.RAIL_X[rail], L.RAIL_R + 0.002)


def run(tee, cloths=None, hands=FAR):
    cloths = cloths or {r: m.start for r, m in CLOTHS.items()}
    return scoring.evaluate(tee, cloths, CORNER_START, CORNERS, hands)


def test_full_score_when_folded_over_the_free_rail():
    r = run(tee_over(L.FREE_RAIL))
    assert r.score == 1.0, r.metrics


def test_lying_on_the_table_scores_zero():
    assert run(TEE.start).score == 0.0


def test_over_a_loaded_rail_is_only_hanging():
    r = run(tee_over("B"))
    assert r.score == 0.25, r.metrics


def test_one_half_too_short_is_only_hanging():
    r = run(tee_over(L.FREE_RAIL, offset=0.09))  # hem half 50 mm, neck half 230 mm (touches nothing, but not folded over)
    assert r.metrics["drop_front_mm"] < 80 or r.metrics["drop_back_mm"] < 80
    assert r.score in (0.0, 0.25)


def test_bunched_shirt_is_not_spread():
    r = run(tee_over(L.FREE_RAIL, squash=0.6))
    assert r.score == 0.5, r.metrics


def test_disturbed_cloth_caps_at_three_quarters():
    cloths = {r: m.start.copy() for r, m in CLOTHS.items()}
    cloths["B"] = cloths["B"] + [0.0, 0.05, 0.0]  # slid 50 mm along its rail
    r = run(tee_over(L.FREE_RAIL), cloths)
    assert r.score == 0.75, r.metrics


def test_a_hand_still_holding_scores_zero():
    hands = dict(FAR, left=(np.array([0.38, 0.11, 1.05]), 0.0, True))
    assert run(tee_over(L.FREE_RAIL), hands=hands).score == 0.0


def test_hand_resting_on_the_shirt_caps_at_three_quarters():
    tee = tee_over(L.FREE_RAIL)
    near = tee[np.argmax(tee[:, 2])] + [0.0, 0.0, 0.02]
    hands = dict(FAR, left=(near, 0.094, False))
    assert run(tee, hands=hands).score == 0.75
