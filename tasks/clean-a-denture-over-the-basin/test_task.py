"""Offline checks of the denture-brushing scorer: denture and brush poses stepped through the stroke counter, the drop
latch and the scorer, near misses included, plus the demo's plan. No simulator.

    python -B tasks/clean-a-denture-over-the-basin/test_task.py      (prints "all pass")
"""

import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_spec = importlib.util.spec_from_file_location("denture_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit = m.kit
from yam_arena.task_file import ObjectState  # noqa: E402

fails = 0


def check(cond, what):
    global fails
    print(("ok  " if cond else "FAIL") + " " + what)
    fails += not cond


def st(pos, q=(0, 0, 0, 1), v=np.zeros(3)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=np.asarray(v, float), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def q_axis(axis, ang):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    return (*(a * math.sin(ang / 2)), math.cos(ang / 2))


Q_BRUSH = q_axis((0, 0, 1), math.pi / 2)  # handle toward -y, bristles down
G = {"left_gripper": (0.2, 0.2, m.TOP + 0.2), "right_gripper": (0.2, -0.2, m.TOP + 0.2)}
HOLD = np.array([m.HOLD[0], m.HOLD[1], m.TOP + m.HOLD[2]])
COUNTER = np.array([m.COUNTER_HOLD[0], m.COUNTER_HOLD[1], m.TOP + m.COUNTER_HOLD[2]])
IN_CUP = np.array([m.CUP[0], m.CUP[1], m.TOP + m.CUP_WALL])


def new_trial():
    kit._EPISODE["count"] += 1


def step(den, tip, dq=(0, 0, 0, 1), bq=Q_BRUSH, dv=(0, 0, 0)):
    """The denture's origin at den (world), the brush's bristle tips' middle at tip (world)."""
    Rb = m._rot(bq)
    bpos = np.asarray(tip, float) - Rb @ np.array([0.0, 0.0, m.TIP_Z])
    objs = {"denture": st(den, dq, np.asarray(dv, float)), "brush": st(bpos, bq),
            "station": st((0, 0, m.TOP)), **{k: st(v, (1, 0, 0, 0)) for k, v in G.items()}}
    return m.score(objs), m.success(objs)


def zone_center(k):
    pts = m.ZONE_PTS[k]
    return pts[len(pts) // 2]


TANG = {0: (1.0, 0.0), 1: (0.0, 1.0), 2: (0.0, 1.0)}


def brush_point(k, den=HOLD, strokes=4, amp=0.008, h=0.0035, dq=(0, 0, 0, 1), bq=Q_BRUSH, n=6):
    """Back-and-forth strokes at ONE spot of zone k with the denture's origin at den (the old way: no longer enough)."""
    c = den + m._rot(dq) @ zone_center(k)
    t = np.array([*TANG[k], 0.0])
    if k == 1:
        c = den + m._rot(dq) @ np.array([m.R, 0.004, m.HT])
    if k == 2:
        c = den + m._rot(dq) @ np.array([-m.R, 0.004, m.HT])
    out = None
    up = m._rot(dq)[:, 2]
    for s in range(strokes):
        a, b = (-amp, amp) if s % 2 == 0 else (amp, -amp)
        for f in np.linspace(a, b, n):
            out = step(den, c + t * f + up * h, dq, bq)
    return out


def zone_path(k, den, dq, skip_last=0, skip_first=0):
    """The zone's teeth points in the world (the denture at den), the path ends extended 2 mm."""
    pts = [den + m._rot(dq) @ np.asarray(p, float) for p in m.ZONE_PTS[k]]
    pts = pts[skip_first:len(pts) - skip_last]
    d0 = pts[0] - pts[1] if len(pts) > 1 else np.array([0.0, 1.0, 0.0])
    d1 = pts[-1] - pts[-2] if len(pts) > 1 else np.array([0.0, 1.0, 0.0])
    pts = [pts[0] + 0.002 * d0 / np.linalg.norm(d0)] + pts + [pts[-1] + 0.002 * d1 / np.linalg.norm(d1)]
    return pts


def brush_zone(k, den=HOLD, strokes=4, h=0.0035, dq=(0, 0, 0, 1), bq=Q_BRUSH, n=4, skip_last=0, skip_first=0, **_):
    """Strokes along the whole arch of zone k (forward, back, forward, ...) with the denture's origin at den."""
    path = zone_path(k, den, dq, skip_last, skip_first)
    up = m._rot(dq)[:, 2]
    out = None
    for s in range(strokes):
        seq = path if s % 2 == 0 else path[::-1]
        for p0, p1 in zip(seq, seq[1:]):
            for f in np.linspace(0.0, 1.0, n, endpoint=False):
                out = step(den, p0 + (p1 - p0) * f + up * h, dq, bq)
    return out


def all_zones(den=HOLD, **kw):
    out = None
    for k in range(3):
        out = brush_zone(k, den, **kw)
    return out


def park(den):
    """The brush away on its towel, the denture at den."""
    return step(den, (m.BRUSH0[0], m.BRUSH0[1], m.TOP + 0.01))


new_trial()
s, ok = park((m.DENTURE0[0], m.DENTURE0[1], m.TOP + 0.01))
check(s == 0.0 and not ok, "untouched -> 0")

new_trial()
all_zones()
s, ok = park(HOLD)
check(s == 0.5 and not ok, f"all three zones brushed over the basin, denture still held -> 0.5 (got {s})")
s, ok = park(IN_CUP)
check(s == 1.0 and ok and all(x >= 4 for x in m._T["strokes"]), f"...then into its cup -> 1.0 (got {s}, {m._T['strokes']})")

new_trial()
all_zones(COUNTER)
s, _ = park(IN_CUP)
check(s == 0.0 and m._T["strokes"] == [0, 0, 0], f"brushed held over the counter -> nothing counts, 0 (got {s})")

new_trial()
brush_zone(0)
s, _ = park(IN_CUP)
check(s == 0.25, f"front teeth only -> 0.25 (got {s})")
new_trial()
brush_zone(0)
brush_zone(1)
s, _ = park(IN_CUP)
check(s == 0.25, f"two zones -> 0.25 (got {s})")
new_trial()
all_zones(strokes=3)
s, _ = park(IN_CUP)
check(s == 0.0 and m._T["strokes"] == [3, 3, 3], f"3 strokes on each zone (4 needed) -> 0 (got {s})")

# four strokes at one spot of each zone are no longer enough: the whole arch must be brushed
new_trial()
for k in range(3):
    brush_point(k)
s, _ = park(IN_CUP)
check(s == 0.0 and m._T["strokes"] == [4, 4, 4], f"four strokes at one spot per zone -> no zone done, 0 (got {s}, {m._T['strokes']})")
new_trial()
for k in range(3):
    brush_zone(k, skip_last=2)
s, _ = park(IN_CUP)
check(s < 1.0, f"each arch brushed short of its last two teeth -> not complete (got {s})")
new_trial()
brush_zone(0); brush_zone(1)
brush_zone(2, skip_last=2)
s, _ = park(IN_CUP)
check(s == 0.25, f"the near side's last two back teeth missed -> only the front and far side done, 0.25 (got {s})")
new_trial()
brush_zone(0); brush_zone(1); brush_zone(2)
s, _ = park(IN_CUP)
check(s == 1.0 and all(len(m._T["cov"][k]) == len(m.ZONE_PTS[k]) for k in range(3)), f"every tooth point brushed (back teeth included) -> 1.0 (got {s})")
new_trial()
all_zones(den=COUNTER)
check(all(len(c_) == 0 for c_ in m._T["cov"]), "brushing over the counter marks no teeth as brushed")

new_trial()
all_zones(h=0.009)
check(m._T["strokes"] == [0, 0, 0], "bristles 9 mm above the teeth (air brushing) -> no strokes")
new_trial()
all_zones(h=-0.008)
check(m._T["strokes"] == [0, 0, 0], "bristle tips 8 mm into the teeth (not possible) -> no strokes")
new_trial()
all_zones(h=-0.004)
check(m._T["strokes"] == [0, 0, 0], "bristle tips 4 mm into the teeth (rigid bristles cannot go that deep) -> no strokes")
new_trial()
all_zones(h=0.005)
check(all(x >= 4 for x in m._T["strokes"]), "bristle tips resting 5 mm above the drawn tops (in contact) -> counts")
new_trial()
all_zones(h=-0.002)
check(m._T["strokes"] == [0, 0, 0], "bristle tips 2 mm into the teeth (rigid bristles cannot get there) -> no strokes")
new_trial()
for k in range(3):
    brush_point(k, amp=0.004)
check(m._T["strokes"] == [0, 0, 0], "8 mm jiggles -> no strokes (1 cm needed)")
new_trial()
qb = np.array([0, 0, 0, 1.0])
Rt = m._rot(q_axis((0, 1, 0), math.radians(45))) @ m._rot(Q_BRUSH)
w = math.sqrt(1 + np.trace(Rt)) / 2
qb = ((Rt[2, 1] - Rt[1, 2]) / (4 * w), (Rt[0, 2] - Rt[2, 0]) / (4 * w), (Rt[1, 0] - Rt[0, 1]) / (4 * w), w)
all_zones(bq=qb)
check(m._T["strokes"] == [0, 0, 0], "brush tilted 45 degrees (bristles not on the teeth) -> no strokes")
new_trial()
dq = q_axis((1, 0, 0), math.radians(90))
all_zones(dq=dq)
check(m._T["strokes"] == [0, 0, 0], "denture held on its side -> the teeth's tops not brushed")

# drops
new_trial()
step(COUNTER, (m.BRUSH0[0], m.BRUSH0[1], m.TOP + 0.01), dv=(0, 0, -1.2))
all_zones()
s, _ = park(IN_CUP)
check(s == 0.0 and m._T["dropped"] is not None, "dropped over the counter (falling 1.2 m/s), then all done -> 0")
new_trial()
step(HOLD, (m.BRUSH0[0], m.BRUSH0[1], m.TOP + 0.01), dv=(0, 0, -1.2))
all_zones()
s, _ = park(IN_CUP)
check(s == 1.0, f"dropped into the basin's water, picked out, all done -> 1.0 (got {s})")
new_trial()
step(IN_CUP + (0, 0, 0.02), (m.BRUSH0[0], m.BRUSH0[1], m.TOP + 0.01), dv=(0, 0, -0.6))
check(m._T["dropped"] is None, "let go just above the cup's floor -> not a drop")
new_trial()
step(COUNTER, (m.BRUSH0[0], m.BRUSH0[1], m.TOP + 0.01), dv=(0, 0, -0.2))
check(m._T["dropped"] is None, "lowered by hand at 0.2 m/s over the counter -> not a drop")

# the cup
new_trial()
all_zones()
s, _ = park(IN_CUP + (0, 0, 0.08))
check(s == 0.5, "held above the cup (base above its rim) -> not in it, 0.5")
s, _ = park(IN_CUP + (0.06, 0, 0))
check(s == 0.5, "beside the cup -> 0.5")

# the demo's plan
d = m.demo()
check(abs(sum(w.duration for w in d["left"]) - sum(w.duration for w in d["right"])) < 1e-9, "both plans same length")
total = sum(w.duration for w in d["right"])
check(total * 20 <= 1200, f"demo plan {total:.1f} s = {total * 20:.0f} steps, under the 1,200-step run limit")


def at(plan, t):
    """The plan's tool point at time t (straight lines between waypoints; Home waypoints are skipped)."""
    p, t0 = None, 0.0
    for w in plan:
        if isinstance(w, kit.Home):
            return p
        q = np.asarray(w.pos, float)
        if t <= t0 + w.duration:
            return q if p is None else p + (q - p) * (t - t0) / w.duration
        p, t0 = q, t0 + w.duration
    return p


gap = min(float(np.linalg.norm(at(d["left"], t) - at(d["right"], t)))
          for t in np.arange(0.0, total, 0.05) if at(d["left"], t) is not None and at(d["right"], t) is not None)
check(gap >= 0.16, f"the two hands stay at least 16 cm apart (closest {gap * 100:.1f} cm)")
print("all pass" if not fails else f"{fails} FAILED")


def test_task():
    """pytest entry point: every check above ran when this file was imported."""
    assert fails == 0, f"{fails} check(s) failed (run this file directly to see which)"


if __name__ == "__main__":
    sys.exit(1 if fails else 0)
