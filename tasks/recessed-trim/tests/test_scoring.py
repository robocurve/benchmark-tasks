"""Unit checks of the recessed-trim scorer, geometry and spring law (numpy only): python tests/test_scoring.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

results = []


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def legs_at(tp, tr, theta):
    """World poses of the four legs for trim pose (tp, tr) and per-leg half-angle theta (dict or float)."""
    out = {}
    for leg in L.LEGS:
        th = theta[leg] if isinstance(theta, dict) else theta
        out[leg] = (tp + tr @ L.leg_root(leg), tr @ L.leg_rot(leg, th))
    return out


def theta_through_slot_end(coil_z_world):
    """Half-angle at which a leg from a coil at this height crosses the tab mid-plane at the slot's end (wire inside)."""
    zm = L.H[2] + L.TAB_Z - L.TAB_T / 2
    return math.atan2(L.SLOT_U - L.WIRE / 2 - L.ROOT_U, zm - coil_z_world)


def run(tp, tr, legs, phi, force=0.0, steps=25, v=0.0, w=0.0):
    sc = S.Scorer()
    lvl = 0.0
    for _ in range(steps):
        lvl = sc.step(S.measure(tp, tr, legs, phi, force), v, w)
    return lvl, sc


# ---------------------------------------------------------------- geometry and spring law
th_sq = L.THETA_MIN
tip_u = L.ROOT_U + L.LEG_L * math.sin(th_sq)
check("squeezed legs (with hooks) pass the slot: outer hook edge inside the slot half-length",
      tip_u + L.HOOK_L + L.WIRE / 2 < L.SLOT_U - 0.0005)
th_free = L.THETA_FREE
check("free legs (with hooks) do not pass the slot",
      L.ROOT_U + L.LEG_L * math.sin(th_free) - L.WIRE / 2 > L.SLOT_U)
# two degrees open: the hooks no longer pass (a squeeze is needed, not a light touch)
check("legs 3 deg open no longer pass the slot",
      L.ROOT_U + L.LEG_L * math.sin(math.radians(3.0)) + L.HOOK_L + L.WIRE / 2 > L.SLOT_U)
f_sq = L.squeeze_force(L.THETA_MIN, 0.008)
check(f"squeeze force at 8 mm from the coil {f_sq:.1f} N is well under the 40 N finger limit", 10.0 < f_sq < 25.0)
check("free half-angle above the squeezed one", L.THETA_FREE > L.THETA_MIN)
# lift at flush: legs cross the slot ends, pressing outward on them
tp_flush = L.H + np.array([0.0, 0.0, -L.FLANGE_T])
coil_flush = tp_flush[2] + L.COIL_Z
th = theta_through_slot_end(coil_flush)
d = (L.H[2] + L.TAB_Z - L.TAB_T / 2 - coil_flush) / math.cos(th)
n = L.spring_torque(th) / d
lift = 4 * n * math.sin(th)
check(f"spring lift at flush {lift:.2f} N exceeds the trim's weight {L.TRIM_MASS * 9.81:.2f} N",
      lift > 1.3 * L.TRIM_MASS * 9.81)
tip_u = L.ROOT_U + L.LEG_L * math.sin(th) + L.HOOK_L + L.WIRE / 2
check(f"at flush the legs press the slot ends before their hooks reach the can wall (hook tip r "
      f"{math.hypot(L.LEG_R, tip_u) * 1e3:.1f} mm < {L.CAN_R[0] * 1e3:.1f} mm)", math.hypot(L.LEG_R, tip_u) < L.CAN_R[0] - 0.0005)
f_sq10 = L.squeeze_force(L.THETA_MIN, 0.010)
check(f"squeeze force at the demo's pinch (10 mm) {f_sq10:.1f} N", 8.0 < f_sq10 < 20.0)
check("tab clears the baffle at flush (radial gap > 1 mm)", L.TAB_R[0] - L.baffle_r_at(L.TAB_Z) > 0.001)
check("legs' plane inside the slot", L.SLOT_R[0] + L.WIRE / 2 < L.LEG_R < L.SLOT_R[1] - L.WIRE / 2)
check("flange covers the cut-out", L.FLANGE_R[1] > L.CUTOUT_R + 0.008)
# heuristic: clocking to the robot's left-right line misses the slots (smallest phi 3.0 deg) for legs squeezed
# between their stop and 1 deg past it
miss = L.LEG_R * math.sin(math.radians(min(abs(L.SEEDED_PHI[0][1]), abs(L.SEEDED_PHI[1][0]))))
for th_deg in (-1.0, -2.0):
    ext = L.ROOT_U + L.LEG_L * math.sin(math.radians(th_deg)) + L.HOOK_L + L.WIRE / 2
    check(f"clock-to-0 shortcut: hooks squeezed at {th_deg} deg reach {(ext + miss) * 1e3:.2f} mm > slot half {L.SLOT_U * 1e3:.1f} mm",
          ext + miss > L.SLOT_U)
# leg-angle round trip
for leg in L.LEGS:
    r = L.leg_rot(leg, 0.4)
    check(f"leg {leg} angle round trip", abs(L.leg_theta(np.eye(3), r, leg) - 0.4) < 1e-9)
# seeded start states: phi never within 4 deg of 0, trim turned at least 4 deg
ok = True
for s in range(200):
    sp = L.setup_spec(("seed", s))
    ok &= 3.5 <= abs(sp["phi"]) <= 5.0 and abs(sp["psi0"] - sp["phi"]) >= 4.0 and abs(sp["psi0"]) <= 3.0
check("seeded setups stay in their ranges", ok)
sp = L.setup_spec(("seed", 1415158438))
check(f"official seed start is non-trivial (phi {sp['phi']:.1f}, psi0 {sp['psi0']:.1f})",
      abs(sp["phi"] - sp["psi0"]) >= 4.0)

# ---------------------------------------------------------------- scorer
for phi in (-4.5, 4.0):
    tr = L.rz(phi)
    legs = legs_at(tp_flush, tr, th - 0.01)
    lvl, sc = run(tp_flush, tr, legs, phi)
    check(f"phi {phi}: flush, both captured, still, hands off -> 1.0", lvl == 1.0)
    lvl, _ = run(tp_flush, tr, legs, phi, steps=10)
    check(f"phi {phi}: flush but held still only 0.5 s -> 0.5", lvl == 0.5)
    lvl, _ = run(tp_flush, tr, legs, phi, force=2.0)
    check(f"phi {phi}: robot still touching -> 0", lvl == 0.0)
    lvl, _ = run(tp_flush, tr, legs, phi, v=0.05)
    check(f"phi {phi}: moving -> not 1.0", lvl == 0.5)
    tp_low = tp_flush + np.array([0.0, 0.0, -0.010])
    legs_low = legs_at(tp_low, tr, theta_through_slot_end(tp_low[2] + L.COIL_Z) - 0.01)
    lvl, _ = run(tp_low, tr, legs_low, phi)
    check(f"phi {phi}: both captured, flange 10 mm low -> 0.5", lvl == 0.5)
    # one spring: B's legs squeezed and below the tab (not through)
    tp_tilt = tp_flush + np.array([0.0, 0.0, -0.030])
    th_a = theta_through_slot_end(tp_tilt[2] + L.COIL_Z) - 0.005
    legs1 = legs_at(tp_tilt, tr, {"Ap": th_a, "Am": th_a, "Bp": L.THETA_FREE, "Bm": L.THETA_FREE})
    s_a = S.spring_captured(legs1, "A", phi)
    s_b = S.spring_captured(legs1, "B", phi)
    check(f"phi {phi}: A through its slot, B (free legs, spread) not", s_a and not s_b)
    lvl, _ = run(tp_tilt, tr, legs1, phi)
    check(f"phi {phi}: one spring captured -> 0.25", lvl == 0.25)
    # clocked wrong by 5 deg: legs miss the slots
    tr_bad = L.rz(phi + 5.0)
    legs_bad = legs_at(tp_flush, tr_bad, th - 0.01)
    lvl, _ = run(tp_flush, tr_bad, legs_bad, phi)
    check(f"phi {phi}: trim turned 5 deg off the brackets -> legs outside the slots, 0", lvl == 0.0)
    # trim on its carton
    tp0, tr0 = L.trim_start_pose(0)
    lvl, _ = run(tp0, tr0, legs_at(tp0, tr0, L.THETA_FREE), phi)
    check(f"phi {phi}: trim on the carton -> 0", lvl == 0.0)
# gap measurement: tilted flange
tr_t = L.rz(0.0) @ np.array([[1, 0, 0], [0, math.cos(0.03), -math.sin(0.03)], [0, math.sin(0.03), math.cos(0.03)]])
g = S.flange_gaps(tp_flush, tr_t)
check(f"flange tilted 1.7 deg: max gap {max(g) * 1e3:.2f} mm > 2 mm", max(g) > 0.002)

n_fail = sum(1 for _, ok_ in results if not ok_)
print(f"{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
