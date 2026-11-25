"""Unit checks of the jedec-flip scorer and geometry (numpy only): python tests/test_scoring.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

FAR = [np.array([0.0, 0.4, 1.2])] * 4
results = []
SEED = ("seed", 1415158438)


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def run(poses, tips=FAR, steps=25, v=0.0, w=0.0, sc=None):
    sc = sc or S.Scorer()
    lvl = 0.0
    for _ in range(steps):
        lvl = sc.step(S.measure(poses, tips), v, w)
    return lvl, sc


def stacked(s, dx=0.0, dy=0.0, dz=0.0, dyaw=0.0):
    pa, ra = s["A"]
    rb = ra @ L.rz(dyaw)
    return dict(s, B=(pa + ra @ np.array([dx, dy, L.STACK_DZ + dz]), rb))


def flipped(s):
    """The stack turned 180 deg about world y, set down 85 mm lower (rigid), packages fallen onto B's cell floors."""
    pa, ra = s["A"]
    c = pa + np.array([0.0, 0.0, 0.008])
    R = L.ry(np.pi)

    def f(p, r):
        return c + R @ (np.asarray(p) - c), R @ r

    out = dict(A=f(*s["A"]), B=f(*s["B"]))
    pb, rb = out["B"]
    chips = []
    for (p, r), pc in zip(s["chips"], L.pocket_centers()):
        q = rb.T @ (R @ (np.asarray(p) - c) + c - pb)  # chip position in B's frame after the turn
        # drop it onto B cell floor: lid on the web underside (B z = WEB_Z[0]), package origin 3.4 mm further
        q[2] = L.WEB_Z[0] - L.CHIP_H
        chips.append((pb + rb @ q, R @ r))
    out["chips"] = chips
    return out


def aside(s, dx=-0.17):
    pa, ra = s["A"]
    return dict(s, A=(np.array([pa[0] + dx, pa[1], L.TOP + L.TRAY_H + L.LIP_H]), ra))


s0 = L.start_poses(SEED)
# ---------------------------------------------------------------- geometry
pieces = L.tray_pieces()
check("tray has 43 convex pieces, all inside the JEDEC outline", len(pieces) == 43 and all(
    v[:, 0].max() <= L.TRAY_L / 2 + 1e-9 and v[:, 1].max() <= L.TRAY_W / 2 + 1e-9 for _, v in pieces))
check("outline 322.6 x 135.9 mm, 7.62 mm pitch", abs(L.TRAY_L - 0.3226) < 1e-9 and abs(L.TRAY_W - 0.1359) < 1e-9
      and abs(L.TRAY_H - 0.00762) < 1e-9)
check("pocket 0.5 mm a side on the 35 mm package", abs(L.POCKET - L.CHIP_SUB[0] - 0.001) < 1e-9)
check("stacking lip clears the upper tray's inner wall by 0.5 mm",
      abs((L.WALL_OUT_T) - (L.LIP_INSET[0] - 0.0005)) < 1e-9)
check("lip fits under the rebate", L.LIP_H < L.REBATE_H)
gap = (L.STACK_DZ + L.UNDER_RIB_Z[0]) - L.TOP_RIB_Z[1]
check("nested: gap between pocket walls and cell walls (0.42 mm) far below the package thickness",
      0.0 < gap < L.CHIP_H / 4)
check("nested: the package has vertical room (cavity minus package) of 3.0 mm",
      abs((L.STACK_DZ + L.WEB_Z[0] - L.WEB_Z[1]) - L.CHIP_H - 0.00302) < 1e-6)
check("cell walls overlap the dead-bug package's substrate by >= 0.7 mm",
      (L.WEB_Z[0] - L.UNDER_RIB_Z[0]) - L.CHIP_LID[1] >= 0.0007)
check("end handling slot is >= 16 mm long", L.SLOT_X[1] - L.SLOT_X[0] >= 0.016)
check("riser leaves both tray ends free by >= 45 mm", (L.TRAY_L - L.RISER_SIZE[1]) / 2 - L.A_JITTER["dy"] >= 0.045)
check("riser is wider than the tray (tray walls bear on it)", L.RISER_SIZE[0] > L.TRAY_W)
# ---------------------------------------------------------------- start states
for k in range(5):
    sp = L.setup_spec(k)
    check(f"setup {k}: empty tray >= 8 mm or 3 deg off a straight move",
          max(abs(sp["b"][0] - sp["a"][0]), abs(sp["b"][1] - sp["a"][1])) >= 0.008 or abs(sp["b"][2] - sp["a"][2]) >= 3)
bad = 0
for n in range(300):
    sp = L.setup_spec(("seed", n))
    if not (max(abs(sp["b"][0] - sp["a"][0]), abs(sp["b"][1] - sp["a"][1])) >= 0.008
            or abs(sp["b"][2] - sp["a"][2]) >= 3):
        bad += 1
check("300 seeds: no near-trivial start", bad == 0)
check("named setups and 2000 seeds: loaded tray >= 3 mm off nominal at its ends (a blind cover never fits)",
      all(L.a_misfit(L.SETUPS[k][0]) >= 0.003 for k in range(5))
      and all(L.a_misfit(L.setup_spec(("seed", n))["a"]) >= 0.003 for n in range(2000)))
sp = L.setup_spec(SEED)
check("default seed: empty tray 11.6 mm / 7.5 mm / 3.7 deg off", abs((sp["b"][0] - sp["a"][0]) * 1e3 + 11.56) < 0.1)
lvl, sc = run(s0)
check("start state scores 0, nothing out, 21 captive in A", lvl == 0.0 and sc.last["measure"]["out"] == []
      and sc.last["measure"]["n_captive"] == 21)
# ---------------------------------------------------------------- levels
st = stacked(s0)
lvl, sc = run(st)
check("B nested on A: 0.25", lvl == 0.25)
lvl, _ = run(stacked(s0, dx=0.0015))
check("B 1.5 mm off: not nested (0)", lvl == 0.0)
lvl, _ = run(stacked(s0, dyaw=0.6))
check("B 0.6 deg off: not nested (0)", lvl == 0.0)
lvl, _ = run(stacked(s0, dz=0.0015))
check("B riding 1.5 mm high on the lip: not nested (0)", lvl == 0.0)
lvl, _ = run(stacked(s0, dyaw=180.0))
check("B turned end for end (chamfers apart): not nested (0)", lvl == 0.0)
fl = flipped(st)
lvl, sc = run(fl, sc=run(st)[1])
check("turned stack, packages captive: 0.5", lvl == 0.5)
fa = aside(fl)
lvl, sc = run(fa, sc=run(fl, sc=run(st)[1])[1])
check("A aside, packages dead-bug in B's cells, still, hands clear: 1.0", lvl == 1.0 and sc.done)
lvl, sc = run(fa, steps=10, sc=run(fl, sc=run(st)[1])[1])
check("1.0 needs the 1 s hold (10 steps: 0.5)", lvl == 0.5)
lvl, _ = run(fa, sc=run(fl, sc=run(st)[1])[1], v=0.02)
check("still moving: not 1.0", lvl == 0.5)
near = [fa["B"][0] + np.array([0.0, 0.0, -0.01])] * 4
lvl, _ = run(fa, tips=near, sc=run(fl, sc=run(st)[1])[1])
check("hand at B: not 1.0", lvl == 0.5)
near_a = [fa["A"][0] + fa["A"][1] @ np.array([L.TRAY_L / 2 - 0.002, 0.0, 0.004])] * 4
lvl, _ = run(fa, tips=near_a, sc=run(fl, sc=run(st)[1])[1])
check("old tray still in the hands: not 1.0", lvl == 0.5)
lvl, _ = run(fa, sc=S.Scorer())
check("end state without the stacked/turned history: 0", lvl == 0.0)
# one package lid up (not turned) in B
bad = dict(fa)
ch = list(fa["chips"])
p, r = ch[5]
ch[5] = (p, r @ L.rx(180.0))
bad["chips"] = ch
lvl, _ = run(bad, sc=run(fl, sc=run(st)[1])[1])
check("one package lid side up: not 1.0", lvl == 0.5)
ch = list(fa["chips"])
p, r = ch[7]
ch[7] = (p + fa["B"][1] @ np.array([0.0025, 0.0, 0.0]), r)
bad["chips"] = ch
lvl, _ = run(bad, sc=run(fl, sc=run(st)[1])[1])
check("one package 2.5 mm off its cell (on a cell wall): not 1.0", lvl == 0.5)
lvl, _ = run(dict(fa, A=(fa["B"][0] + np.array([0.0, 0.0, 0.03]), fa["A"][1])), sc=run(fl, sc=run(st)[1])[1])
check("old tray held above B: not 1.0", lvl == 0.5)
# veto
ch = list(st["chips"])
ch[0] = (np.array([0.33, 0.30, L.TOP]), np.eye(3))
lvl, sc = run(dict(st, chips=ch))
check("a package on the bench: 0 (veto)", lvl == 0.0 and sc.veto)
lvl, sc = run(fa, sc=run(dict(st, chips=ch))[1])
check("veto is permanent", lvl == 0.0)

n_fail = sum(1 for _, ok in results if not ok)
print(f"{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
