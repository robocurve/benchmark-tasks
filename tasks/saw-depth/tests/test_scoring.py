"""Unit checks of the saw-depth scorer and geometry (numpy only): python tests/test_scoring.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

FAR = [np.array([0.0, 0.4, 1.0])] * 4  # fingertips far from the saw
results = []


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def poses_for(t, depth, phi=0.0, dy=0.0, tilt_deg=0.0, dz=0.0):
    spec = dict(board=0, t=t, depth0=depth, dy=dy)
    sp, sr = L.shoe_pose(spec)
    if tilt_deg:
        a = math.radians(tilt_deg)
        rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
        sr = rx @ sr
    sp = sp + [0, 0, dz]
    th = L.theta_of_depth(depth)
    up, ur = L.upper_pose(sp, sr, th)
    kp, kr = L.lever_pose(up, ur, phi)
    return dict(shoe=(sp, sr), upper=(up, ur), lever=(kp, kr)), th


def run(t, below, phi=0.0, clamp=True, tips=FAR, steps=25, v=0.0, w=0.0, tilt=0.0, dz=0.0):
    poses, th = poses_for(t, t + below, phi=phi, tilt_deg=tilt, dz=dz)
    sc = S.Scorer()
    lvl = 0.0
    for _ in range(steps):
        lvl = sc.step(S.measure(poses, (v, w), th, phi, clamp, t, tips))
    return lvl, sc


# geometry
check("depth range 0 .. 2-9/16 in", abs(L.depth_of(L.THETA_MAX) - 0.0651) < 1e-6 and L.depth_of(0.0) < 0.0)
check("depth pivot swing 20 .. 35 deg (circular-saw like)", 20 < math.degrees(L.THETA_MAX) < 35)
for d in (0.005, 0.02, 0.045, 0.064):
    check(f"theta_of_depth inverts depth_of at {d * 1e3:.0f} mm", abs(L.depth_of(L.theta_of_depth(d)) - d) < 1e-7)
for t in (L.BOARDS[0][1], L.BOARDS[3][1]):
    poses, th = poses_for(t, t + 0.005)
    p = L.projection_below(*poses["upper"])
    check(f"projection equals depth - thickness ({t * 1e3:.1f} mm board)", abs(p - 0.005) < 1e-6)
check("upper COM behind the pivot (motor weight drops the blade)", L.upper_com_u()[0] < -0.1)
load = L.M_UPPER * 9.81 * (-L.upper_com_u()[0]) / (-L.grip_point_u()[0][0])
check(f"handle load {load:.1f} N under the 40 N finger limit", 10 < load < 25)
check("clamp holds the motor's weight plus a 20 N push on the handle",
      L.CLAMP_NM > L.M_UPPER * 9.81 * (-L.upper_com_u()[0]) + 20 * (-L.grip_point_u()[0][0]))
check("saw 4.0 kg (8.8 lb)", abs(L.M_UPPER + L.M_SHOE - 4.0) < 0.01)
# lower parts stay above the board top over the full depth range (except the blade, past the board's edge)
low_ok = True
for th in np.linspace(0, L.THETA_MAX, 8):
    for nm, v, _, _ in L.upper_pieces():
        if nm in ("blade",):
            continue
        w = (v @ L.rot_v(th).T)[:, 2] + L.PIVOT[2]
        vv = v[:, 1]
        if w.min() < -0.0005 and vv.max() > L.BLADE_GAP:
            low_ok = False
            print("  low part", nm, round(math.degrees(th), 1), w.min())
check("no upper part reaches below the board top over the board", low_ok)

# seeded starts: never in or near the scoring band
bad = 0
for s in range(2000):
    sp = L.setup_spec(("seed", 1415158438 + s))
    p0 = sp["depth0"] - sp["t"]
    if -0.0044 + 1e-9 < p0 < 0.015 - 1e-9 or not (0 <= sp["depth0"] <= L.DEPTH_MAX):
        bad += 1
check("2000 seeded starts: blade >= 4.4 mm above or >= 15 mm below the underside", bad == 0)
# the start depth carries no information about the board: same start-depth distribution for every board
import collections
bins = collections.defaultdict(lambda: [0, 0])
for s in range(6000):
    sp_ = L.setup_spec(("seed", s))
    bins[sp_["board"]][0 if sp_["depth0"] < 0.03 else 1] += 1
fr = [v[0] / sum(v) for v in bins.values()]
check(f"retracted-start share per board {min(fr):.2f}..{max(fr):.2f} (independent of the board)", max(fr) - min(fr) < 0.06)
sp = L.setup_spec(("seed", 1415158438))
check(f"official seed start: {sp['name']} board {sp['t'] * 1e3:.1f} mm depth {sp['depth0'] * 1e3:.1f} mm",
      sp["depth0"] - sp["t"] < -0.0044 or sp["depth0"] - sp["t"] > 0.015)
boards = {L.setup_spec(("seed", s))["board"] for s in range(300)}
check("all six boards drawn", boards == set(range(len(L.BOARDS))))
check("official seed is not the 2x6 and needs > 15 mm of pivoting",
      sp["board"] != 4 and abs(sp["t"] + 0.0048 - sp["depth0"]) > 0.015)

# scorer levels
for below, want in ((0.005, 1.0), (0.0033, 1.0), (0.0063, 1.0), (0.0030, 0.4), (0.0066, 0.4), (0.002, 0.4),
                    (0.008, 0.4), (0.0008, 0.0),
                    (-0.002, 0.0), (0.0095, 0.0), (0.030, 0.0)):
    lvl, _ = run(0.0183, below)
    check(f"tooth {below * 1e3:.1f} mm below the underside -> {want}", lvl == want)
lvl, _ = run(0.0183, 0.005, phi=math.radians(60), clamp=False)
check("lever released -> 0", lvl == 0.0)
lvl, _ = run(0.0183, 0.005, phi=math.radians(4), clamp=True)
check("lever 4 deg off its stop (clamp engaged) -> 1.0", lvl == 1.0)
lvl, _ = run(0.0183, 0.005, steps=19)
check("full score needs 1 s at rest", lvl == 0.0)
lvl, _ = run(0.0183, 0.005, v=0.01)
check("saw moving -> no score", lvl == 0.0)
lvl, _ = run(0.0183, 0.005, tilt=2.0)
check("shoe rocked 2 deg -> 0", lvl == 0.0)
lvl, _ = run(0.0183, 0.005, dz=0.004)
check("shoe 4 mm off the board -> 0", lvl == 0.0)
poses, th = poses_for(0.0183, 0.0233)
up, ur = poses["upper"]
grip = up + ur @ L.grip_point_u()[0]
lvl, _ = run(0.0183, 0.005, tips=[grip] * 4)
check("hand on the handle -> 0", lvl == 0.0)
# veto latches
sc = S.Scorer()
pt, th = poses_for(0.0183, 0.0233, tilt_deg=15.0)
sc.step(S.measure(pt, (0, 0), th, 0.0, True, 0.0183, FAR))
pg, th = poses_for(0.0183, 0.0233)
for _ in range(30):
    lvl = sc.step(S.measure(pg, (0, 0), th, 0.0, True, 0.0183, FAR))
check("saw tipped once -> 0 for good", lvl == 0.0 and sc.tipped)

# blind / open-loop: best fixed depth (same setting on every board) over the four boards
best = (0.0, None)
for d in np.arange(0.0, 0.0651, 0.0001):
    sc = [S.band_level(d - t) for _, t in L.BOARDS]
    m = float(np.mean(sc))
    if m > best[0]:
        best = (m, d, sc)
print(f"  best fixed depth {best[1] * 1e3:.1f} mm: per board {best[2]} mean {best[0]:.3f}")
check("a fixed depth scores 1.0 on at most one board", sum(s == 1.0 for s in best[2]) <= 1)
check("a fixed depth scores > 0 on at most two boards", sum(s > 0 for s in best[2]) <= 2)

n_fail = sum(1 for _, ok in results if not ok)
print(f"{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
