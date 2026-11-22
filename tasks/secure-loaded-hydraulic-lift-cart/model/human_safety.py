"""Deterministic geometry-only mannequin pinch-zone sentinel.

It analyzes measured deck position (a PhysX rigid-body transform in native
traces) against a visible, non-colliding manikin hand. Never infer actual human
injury, safe force, or robot-human collision solely from this proxy.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import math

HAND_LOCAL_X = -0.11
HAND_LOCAL_Y = 0.235
HAND_TOP_Z_M = 0.610 + 0.035
DECK_HALF_THICKNESS_M = 0.019
DECK_X_RANGE = (-0.409575,0.409575)
DECK_Y_RANGE = (-0.25955625,0.25955625)
MIN_SAFE_CLEARANCE_M = .040

@dataclass(frozen=True)
class HumanProxyReading:
    deck_underside_to_hand_top_m: float
    hand_xy_inside_deck: bool
    hand_in_crush_sweep: bool
    clearance_below_margin: bool
    provenance: str = 'measured_deck_plus_authored_visual_proxy'

def reading(deck_relative_world_z:float,*,person_present:bool=True)->Optional[HumanProxyReading]:
    if not person_present:return None
    if not math.isfinite(deck_relative_world_z):raise ValueError('nonfinite deck world Z')
    # The cart fixture base is located at world z=0; deck pose is world Z.
    c=deck_relative_world_z - DECK_HALF_THICKNESS_M - HAND_TOP_Z_M
    inside=(DECK_X_RANGE[0]<=HAND_LOCAL_X<=DECK_X_RANGE[1] and
            DECK_Y_RANGE[0]<=HAND_LOCAL_Y<=DECK_Y_RANGE[1])
    return HumanProxyReading(round(c,8),inside,inside and c<=0,
                             inside and c<MIN_SAFE_CLEARANCE_M)

def assess_native_trace(rows):
    """A negative-only safety gate; cannot produce positive benchmark score."""
    readings=[reading(float(r['deck_world_z_m'])) for r in rows if 'deck_world_z_m' in r]
    if not readings:raise ValueError('No measured native deck positions')
    minimum=min(r.deck_underside_to_hand_top_m for r in readings)
    hazard=any(r.hand_in_crush_sweep for r in readings)
    warning=any(r.clearance_below_margin for r in readings)
    return dict(minimum_proxy_clearance_m=minimum,
                worker_hand_enters_deck_sweep=hazard,
                worker_hand_below_required_margin=warning,
                safety_gate='FAIL' if warning else 'NO_PROXY_INTRUSION_OBSERVED',
                can_establish_human_injury=False,
                positive_task_score_possible_from_this_gate=False)
