"""Unit checks of the clock-dst scorer and geometry (numpy only): python tests/test_scoring.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

W = L.CLOCK_MASS * 9.81
results = []


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def run(pos, rot, nail, shown, nail_n=W, robot_n=0.0, steps=25, v=0.0, w=0.0, vz=0.0):
    sc = S.Scorer()
    lvl = 0.0
    for _ in range(steps):
        lvl = sc.step(S.measure(pos, rot, nail, shown, nail_n, robot_n, v, w), vz)
    return lvl, sc


setup = 0
nail = L.nail_point(setup)
p, r = L.clock_hung_pose(setup)

# time arithmetic
check("q_start inside the drive range", all(abs(L.q_start(m)) < 2 * math.pi - 0.2 for m in range(int(L.START_RANGE[0]), int(L.START_RANGE[1]) + 1)))
check("q_start matches the minute hand", all(abs(((math.pi + L.q_start(m)) - L.minute_angle(m) + math.pi) % (2 * math.pi) - math.pi) < 1e-9 for m in range(515, 591)))
for m0 in (L.START_RANGE[0], 9 * 60 + 7.3, L.START_RANGE[1]):  # noqa
    q0 = L.q_start(m0)
    q_end = q0 + (L.TARGET_MIN - m0) / 60 * 2 * math.pi
    check(f"end joint angle same for all starts ({L.fmt(m0)})", abs(q_end - (math.radians(60) - math.pi + 2 * math.pi)) < 1e-9)
    check(f"end joint angle inside drive range with 15 min overshoot room ({L.fmt(m0)})", abs(q_end) < 2 * math.pi - 0.2 - math.radians(15 * 6))
check("shown minutes", abs(L.shown_minutes(550.0, 1.0 + math.pi, 1.0) - 580.0) < 1e-9)
check("time error wraps 12 h", abs(S.time_error_min(610.0 + 720.0)) < 1e-9 and abs(S.time_error_min(605.0) + 5.0) < 1e-9)
check("hour angle 10:10 = 305 deg -> -55", abs(math.degrees(L.hour_angle(610.0)) + 55.0) < 1e-9)
check("fmt", L.fmt(610.0) == "10:10.00" and L.fmt(9 * 60 + 2.5) == "9:02.50")

# setups: seeded start times / nail offsets inside their ranges; default seed non-trivial
for sd in (1415158438, 0, 7, 13, 2):
    sp = L.setup_spec(("seed", sd))
    ok = (
        any(lo <= sp["start_min"] <= hi for lo, hi in L.START_SEGMENTS)
        and abs(sp["nail_y"] - L.NAIL_Y0) <= L.NAIL_JITTER_Y
        and abs(sp["nail_z"] - L.NAIL_Z0) <= L.NAIL_JITTER_Z
    )
    check(f"seed {sd} in range ({L.fmt(sp['start_min'])})", ok)
n0 = L.TARGET_MIN - L.setup_spec(("seed", 1415158438))["start_min"]
check(f"default seed: no round fixed turn (5 min steps) within 2 min, none of 50/60/70 within 5 (needs {n0:.2f})",
      all(abs(n0 - x) > 2.0 for x in range(0, 130, 5)) and all(abs(n0 - x) > 5.0 for x in (50, 60, 70)))

# geometry: the hung pose puts the shank in the slot top, head behind the panel
nk, hk = L.nail_in_k(p, r, nail)
check("hung: nail in keyhole", S.nail_in_keyhole(nk, hk))
check("hung: shank at the slot top", abs(nk[1] - L.HANG_SHANK_K[1]) < 1e-9 and abs(nk[0]) < 1e-9)
check("keyhole entry passes the head", L.KEY_HOLE_R > L.NAIL_HEAD_R + 0.001)
check("slot holds the head", L.KEY_SLOT_W / 2 < L.NAIL_HEAD_R - 0.001)
check("slot passes the shank", L.KEY_SLOT_W / 2 > L.NAIL_SHANK_R + 0.0003)
check("head fits inside the case behind the panel", L.NAIL_PROUD < L.MOVE_Z[1] and L.NAIL_PROUD - L.NAIL_HEAD_T > L.BACK_T + 0.003)
# entry hole on the nail (clock 8.6 mm lower): not captured
p_low = p - np.array([0.0, 0.0, L.HANG_SHANK_K[1] - L.KEY_HOLE_C[1]])
nk2, hk2 = L.nail_in_k(p_low + np.array([0, 0, 0.0]), r, nail)
check("shank in the entry hole: not captured", not S.nail_in_keyhole(L.nail_in_k(p + np.array([0, 0, 0.0086]), r, nail)[0], hk))
# hanging but 6 mm off the wall at the keyhole (head just caught behind the panel): not a proper hang
check("hung 6 mm off the wall: not on nail", not S.nail_in_keyhole(*L.nail_in_k(p - np.array([0.006, 0, 0]), r, nail)))
check("hung 2 mm off the wall: on nail", S.nail_in_keyhole(*L.nail_in_k(p - np.array([0.002, 0, 0]), r, nail)))
# clock off the wall (head in front of the panel)
check("clock 15 mm off the wall: not on nail", not S.nail_in_keyhole(*L.nail_in_k(p - np.array([0.015, 0, 0]), r, nail)))

# levels
lvl, _ = run(p, r, nail, L.TARGET_MIN)
check("hung level, 10:10 -> 1.0", lvl == 1.0)
lvl, _ = run(p, r, nail, L.TARGET_MIN + 1.9)
check("1.9 min fast -> 1.0", lvl == 1.0)
lvl, _ = run(p, r, nail, L.TARGET_MIN - 4.0)
check("4 min slow -> 0.5", lvl == 0.5)
lvl, _ = run(p, r, nail, L.TARGET_MIN + 6.0)
check("6 min fast -> 0", lvl == 0.0)
# heuristic shortcuts: any face-blind fixed turn of N minutes reaches 1.0 (within 2 min) on <= 8 % of seeds, and
# turning exactly one hour is never within 5 min
needs = np.array([L.TARGET_MIN - L.setup_spec(("seed", sd))["start_min"] for sd in range(3000)])
p_best = max(float((np.abs(needs - x) <= 2.0).mean()) for x in np.arange(0.0, 130.0, 0.25))
check(f"any fixed turn reaches 1.0 on <= 8 % of 3000 seeds (best {p_best:.3f})", p_best <= 0.08)
worst = float(np.abs(needs - 60.0).min())
check(f"'turn exactly one hour' never within 5 min over 3000 seeds (closest {worst:.2f} min)", worst > 5.0)
lvl, _ = run(p, r, nail, L.TARGET_MIN - 55.0)
check("not set (55 min slow, the start) -> 0", lvl == 0.0)
lvl, _ = run(p, r, nail, L.TARGET_MIN, steps=19)
check("held < 1 s -> 0", lvl == 0.0)
lvl, _ = run(p, r, nail, L.TARGET_MIN, robot_n=0.5)
check("still touched by the robot -> 0", lvl == 0.0)
lvl, _ = run(p, r, nail, L.TARGET_MIN, nail_n=0.3 * W)
check("nail not carrying it -> 0", lvl == 0.0)
lvl, _ = run(p, r, nail, L.TARGET_MIN, v=0.02)
check("moving -> 0", lvl == 0.0)
tilt = L.rz(0.0)
rt = r @ np.array([[math.cos(math.radians(7)), -math.sin(math.radians(7)), 0], [math.sin(math.radians(7)), math.cos(math.radians(7)), 0], [0, 0, 1]])
check("tilt measured 7 deg", abs(S.tilt_in_wall_plane(rt) - 7.0) < 1e-6)
lvl, _ = run(p, rt, nail, L.TARGET_MIN)
check("hung 7 deg off level -> 0", lvl == 0.0)
rt4 = r @ np.array([[math.cos(math.radians(4)), -math.sin(math.radians(4)), 0], [math.sin(math.radians(4)), math.cos(math.radians(4)), 0], [0, 0, 1]])
# rotated about the nail (as a hung clock turns), not about its centre
p4 = nail + (r @ rt4.T @ r.T) @ (p - nail) if False else nail - rt4 @ np.array([L.HANG_SHANK_K[0], L.HANG_SHANK_K[1], 0.0])
lvl, _ = run(p4, rt4, nail, L.TARGET_MIN)
check("hung 4 deg off level -> 1.0", lvl == 1.0)
lvl, sc = run(p, r, nail, L.TARGET_MIN, vz=-0.6)
check("dropped -> 0 forever", lvl == 0.0 and sc.dropped)
sc = S.Scorer()
for k in range(40):
    sc.step(S.measure(p, r, nail, L.TARGET_MIN, W, 0.0, 0.0, 0.0), -0.6 if k == 0 else 0.0)
check("dropped once then hung -> 0", sc.level == 0.0)

# back panel cells: none overlaps the keyhole or the knob opening
cells = L.back_panel_cells()
bad = 0
for cpoly in cells:
    for y in np.linspace(L.KEY_HOLE_C[1] - L.KEY_HOLE_R + 0.0006, L.KEY_SLOT_TOP - 0.0006, 40):
        hw = L.keyhole_half_width(y) - 0.0003
        for x in (np.linspace(-hw, hw, 5) if hw > 0 else []):
            # point strictly inside a convex cell?
            inside = True
            n = len(cpoly)
            sgn = None
            for i in range(n):
                a, b = cpoly[i], cpoly[(i + 1) % n]
                cr = (b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0])
                if abs(cr) < 1e-12:
                    continue
                s = cr > 0
                if sgn is None:
                    sgn = s
                elif s != sgn:
                    inside = False
                    break
            bad += inside
check(f"no back-panel cell covers the keyhole slot ({len(cells)} cells)", bad == 0)
area = sum(0.5 * abs(np.dot(c_[:, 0], np.roll(c_[:, 1], -1)) - np.dot(c_[:, 1], np.roll(c_[:, 0], -1))) for c_ in cells)
want = math.pi * L.SHELL_R[1] ** 2 - L.OPENING**2
check(f"back panel area {area * 1e4:.1f} cm2 ~ disc - opening - keyhole ({want * 1e4:.1f})", want - 2.0e-4 < area < want)

n_fail = sum(1 for _, ok in results if not ok)
print(f"{len(results) - n_fail} passed, {n_fail} failed")
sys.exit(1 if n_fail else 0)
