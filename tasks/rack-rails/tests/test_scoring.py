"""Unit checks of the rack-rails latch rule, scorer and geometry (numpy only): python tests/test_scoring.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

results = []
FAR = [np.array([0.0, 0.0, 1.3])] * 4  # fingertips far from the rail


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def mounted(spec, u=None, gap_r=0.0, gap_f=0.0, dz_f=0.0):
    po, ro = L.mounted_outer_pose(spec, u)
    po = po - np.array([0.0, gap_r, 0.0])
    xh, y0 = L.rack_origin(spec)[:2]
    e = (po[1] - (y0 - L.Y_F) - gap_f) - L.L0
    pi, ri = L.inner_from_outer(po, ro, e)
    pi = pi + np.array([0.0, 0.0, dz_f])
    return po, ro, pi, ri


def latch_through(latch, spec, po, ro, pi, ri, steps=5):
    for _ in range(steps):
        latch.step("rear", L.end_state("rear", po, ro, spec), 0.0)
        latch.step("front", L.end_state("front", pi, ri, spec), 0.0)


def score(spec, po, ro, pi, ri, latch, tips=FAR, steps=25, v=0.0):
    sc = S.Scorer()
    lvl = 0.0
    for _ in range(steps):
        lvl = sc.step(S.measure(spec, po, ro, pi, ri, latch, tips), v)
    return lvl


spec = L.setup_spec(0)
tu = spec["target_u"]

# geometry
check("EIA pitch: 36 holes, spacing 15.875/15.875/12.7 mm", np.allclose(
    np.diff(L.hole_zs())[:3], [0.015875, 0.015875, 0.0127]))
check("pegs 31.75 mm apart match only the top and bottom hole of one U",
      abs(2 * L.PEG_DZ - (L.HOLE_OFFSETS[2] - L.HOLE_OFFSETS[0])) < 1e-9)
check("peg play 0.25 mm a side", abs((L.HOLE - L.PEG) / 2 - 0.00025) < 1e-9)
check("closed rail with pegs fits between the flanges", L.L0 + 2 * L.PEG_L < 2 * L.Y_F)
check("closed rail too short to span; extension reaches", L.L0 < 2 * L.Y_F < L.L0 + L.E_MAX)
check("u_of_z round trip", all(L.u_of_z(L.u_center(u))[0] == u for u in range(1, 13)))
seeded = [L.setup_spec(("seed", s)) for s in range(200)]
check("seeded target U covers U2-U4 only", {s["target_u"] for s in seeded} == set(L.TARGET_US))
check("installed server always one U below the target", all(s["server_u"] == s["target_u"] - 1 for s in seeded))
check("rack never within 4 mm of nominal in x (replay exclusion band)", all(abs(s["rack_dx"]) >= 0.004 - 1e-12 for s in seeded))
st = L.end_state("rear", *L.mounted_outer_pose(spec), spec)
check("mounted rear end: gap 0, in holes, at target U", abs(st["gap"]) < 1e-9 and st["in_holes"] and st["u"] == tu)

# latch rule
lat = S.Latch()
po, ro, pi, ri = mounted(spec, gap_r=0.0015, gap_f=0.0015)
f = lat.step("rear", L.end_state("rear", po, ro, spec), 0.0)
check("inside the catch zone, not latched: 10 N push-back", abs(f + L.LATCH_PUSH_N) < 1e-9 and not lat.latched["rear"])
lat = S.Latch()
po, ro, pi, ri = mounted(spec, gap_r=0.004)
check("outside the zone: no force", lat.step("rear", L.end_state("rear", po, ro, spec), 0.0) == 0.0)
lat = S.Latch()
po, ro, pi, ri = mounted(spec)
po2 = po + np.array([0.0, 0.0, 0.002])  # 2 mm off the hole axis (cannot happen with pegs in; rule must not latch)
lat.step("rear", L.end_state("rear", po2, ro, spec), 0.0)
check("misaligned end does not latch", not lat.latched["rear"])
lat = S.Latch()
latch_through(lat, spec, po, ro, pi, ri)
check("seated ends latch at the target U", lat.latched["rear"] and lat.latched_u["rear"] == tu and lat.latched["front"])
po3 = po - np.array([0.0, 0.001, 0.0])
f = lat.step("rear", L.end_state("rear", po3, ro, spec), -0.01)
check("latched end backing out is held (spring)", f > 15.0)
po4 = po - np.array([0.0, 0.05, 0.0])
check("retention capped at 150 N", lat.step("rear", L.end_state("rear", po4, ro, spec), -1.0) == L.LATCH_HOLD_N)

# scorer
lat = S.Latch()
latch_through(lat, spec, po, ro, pi, ri)
check("both latched at target, hands clear, still: 1.0", score(spec, po, ro, pi, ri, lat) == 1.0)
check("both latched but still moving: 0.5", score(spec, po, ro, pi, ri, lat, v=0.01) == 0.5)
near = [po + np.array([0.017, -0.04, 0.0])] * 4
check("both latched, a hand on the rail: 0.5", score(spec, po, ro, pi, ri, lat, tips=near) == 0.5)
lat = S.Latch()
lat.step("rear", L.end_state("rear", po, ro, spec), 0.0)
pof, rof, pif, rif = mounted(spec, gap_f=0.003)
check("rear latched, front pegs in (not latched): 0.5", score(spec, pof, rof, pif, rif, lat) == 0.5)
pof, rof, pif, rif = mounted(spec, gap_f=0.030)
check("rear latched, front not in: 0.25", score(spec, pof, rof, pif, rif, lat) == 0.25)
lat = S.Latch()
pw, rw, piw, riw = mounted(spec, u=tu - 1)
latch_through(lat, spec, pw, rw, piw, riw)
check("both latched one U low: 0", lat.latched["rear"] and score(spec, pw, rw, piw, riw, lat) == 0.0)
lat = S.Latch()
lat.step("rear", L.end_state("rear", po, ro, spec), 0.0)
pm, rm, pim, rim = mounted(spec, gap_f=0.003, dz_f=-L.U)
check("rear latched at target, front pegged one U low (sloped): 0", score(spec, pm, rm, pim, rim, lat) == 0.0)
lat = S.Latch()
po0, ro0 = L.rail_rest_pose(spec)
pi0, ri0 = L.inner_from_outer(po0, ro0)
check("rail in its cradle: 0", score(spec, po0, ro0, pi0, ri0, lat) == 0.0)

n_fail = sum(1 for _, ok in results if not ok)
print(f"{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
