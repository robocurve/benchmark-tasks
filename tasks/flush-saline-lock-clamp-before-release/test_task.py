"""Offline checks of the saline-lock flush scorer: step-by-step plunger, clamp and gripper histories (near misses
included) run through the Plunger law, the release tracker and the scorer, plus the demo's timing. No simulator.

    python -B tasks/flush-saline-lock-clamp-before-release/test_task.py      (prints "all pass")
"""

import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_spec = importlib.util.spec_from_file_location("flush_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit = m.kit
from yam_arena.task_file import ObjectState  # noqa: E402

fails = 0
PLUNGER = m.OBJECTS[0].links[0]
CLAMP = m.OBJECTS[0].links[1]


def quat_z(a):
    """Pointing straight down (180 degrees about x), then turned `a` about the vertical."""
    down = np.array([1.0, 0.0, 0.0, 0.0])
    rz = np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])
    x1, y1, z1, w1 = rz
    x2, y2, z2, w2 = down
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def st(pos, q=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def check(cond, what):
    global fails
    print(("ok  " if cond else "FAIL") + " " + what)
    fails += not cond


PARK_L = (0.20, 0.27, m.TOP + 0.20)
PARK_R = (0.20, -0.27, m.TOP + 0.20)
SIM = {"q": 0.0, "c": 0.0}
YAW_PUSH = 0.0  # fingers closing along y (the demo's yaw for the push)


def new_trial():
    kit._EPISODE["count"] += 1
    SIM.update(q=0.0, c=0.0)


def right_on_press(q):
    """The right gripper's tool point pressing the thumb press with the plunger at q (closed fingers, outer face on
    the press's back face)."""
    face = m.PRESS0_Y - m.PRESS_T / 2 + q
    return (m.SX, face - m.FINGER_HALF + 0.0005, m.TOP + m.PUSH_Z)


def right_on_rod(q):
    """The right gripper's tool point on the plunger's rod (closed fingers), clear of the thumb press, plunger at q."""
    return (m.SX, m.STOP0_Y + q - (m.STOPPER_L + m.ROD) / 2, m.TOP + m.PUSH_Z)


def left_on_clamp():
    return (m.SX, m.CLAMP_Y, m.TOP + sum(m.JAW_Z) / 2 - 0.012)


def step(q=None, c=None, right=None, left=None, l_open=0.0, vq=0.0, vc=0.0):
    """One control step: the plunger at q (bounded by the law's limits from the last step), the clamp jaw at c (a
    caught latch stays caught), the grippers where given, then the law runs, then the scorer."""
    mem = kit._law_mem("iv_set/plunger", PLUNGER)
    lo, hi = mem.get("limits") or (0.0, m.STROKE)
    if q is not None:
        SIM["q"] = min(max(q, lo), hi)
    key_c = "iv_set/clamp"
    latched = kit._LATCHED.get(key_c) == kit._EPISODE["count"]
    if c is not None and not latched:
        SIM["c"] = c
    if not latched and abs(SIM["c"] - CLAMP.latch.at) <= CLAMP.latch.zone:
        kit._LATCHED[key_c] = kit._EPISODE["count"]
        latched = True
    if latched:
        SIM["c"] = CLAMP.latch.at
    kit._JOINTS["iv_set/plunger"] = kit.JointState(SIM["q"], vq, False)
    kit._JOINTS[key_c] = kit.JointState(SIM["c"], vc, latched)
    PLUNGER.law.drive("iv_set/plunger", SIM["q"], vq, {"plunger": SIM["q"], "clamp": SIM["c"]}, mem)
    objs = {
        "iv_set": st((m.SX, 0.0, m.TOP)),
        "iv_set/plunger": st((m.SX, m.STOP0_Y + SIM["q"], m.TOP + m.AXIS_Z)),
        "iv_set/clamp": st((m.MOVING_JAW_X0 + SIM["c"], m.CLAMP_Y, m.TOP + sum(m.JAW_Z) / 2)),
        "right_gripper": st(right or PARK_R, quat_z(YAW_PUSH)),
        "left_gripper": st(left or PARK_L, quat_z(math.pi / 2)),
    }
    kit._OPENING["right_gripper"] = 0.0
    kit._OPENING["left_gripper"] = l_open
    return m.score(objs), m.success(objs)


def push(q_to, n=20, **kw):
    q0 = SIM["q"]
    out = None
    for k in range(1, n + 1):
        q = q0 + (q_to - q0) * k / n
        out = step(q, right=right_on_press(q), vq=0.003, **kw)
    return out


def push_rod(q_to, n=20, **kw):
    """Push the plunger by gripping its rod (the thumb press untouched)."""
    q0 = SIM["q"]
    out = None
    for k in range(1, n + 1):
        q = q0 + (q_to - q0) * k / n
        out = step(q, right=right_on_rod(q), vq=0.003, **kw)
    return out


def clamp_shut_while(right_q=None, n=6):
    """The left hand squeezes the clamp shut over n steps (the right hand still on the press if right_q is given)."""
    out = None
    for k in range(1, n + 1):
        c = m.SQUEEZE * k / n
        r = right_on_press(right_q) if right_q is not None else None
        out = step(c=c, left=left_on_clamp(), l_open=max(0.0, 0.25 - 0.25 * k / n), right=r)
    return out


def hands_off(n=8):
    out = None
    for _ in range(n):
        out = step()
    return out


# --- geometry sanity
check(abs(m.STROKE * m.ML_PER_M - 5.0) < 1e-9, "the stroke holds exactly 5 mL")
check(abs(m.ML_PER_M - 165.1) < 0.2, "a 14.5 mm bore: 165 mL per metre (6.05 mm per mL)")
check(m.clamp_open_fraction(0.0) == 1.0 and m.clamp_open_fraction(m.SQUEEZE) == 0.0, "clamp open/shut fraction")

# --- 1. The right order: push all 5 mL, clamp shut while still pressing, then let go.
new_trial()
s, _ = step()
check(s == 0.0, "untouched: the clamp is open and nothing is flushed -> 0")
s, _ = step(right=right_on_press(0.0))
check(m.on_press(m._T["objects"]), "the demo's push pose touches the thumb press")
push(m.STROKE)
check(abs(m.flushed_ml(m._T["objects"]) - 5.0) < 0.01, "pushed to the end: 5 mL flushed")
for _ in range(10):
    step(m.STROKE, right=right_on_press(m.STROKE))
clamp_shut_while(m.STROKE)
check(m.clamp_shut(m._T["objects"]), "the clamp caught shut")
mem = kit._law_mem("iv_set/plunger", PLUNGER)
check(mem["limits"][1] - mem["limits"][0] < 1e-4, "clamp shut: the plunger is locked")
s, ok = step(m.STROKE, right=right_on_press(m.STROKE), left=left_on_clamp())
check(s == 0.0, "hands still on the press and the clamp -> 0 (not finished)")
s, ok = hands_off()
check(s == 1.0 and ok, f"clamped under pressure, then let go, hands off -> 1.0 (got {s})")
check(m.reflux(m._T["objects"]) is None, "no reflux logged")

# --- 2. The classic mistake: push all 5 mL, let go, then clamp.
new_trial()
push(m.STROKE)
hands_off(4)
check(m.reflux(m._T["objects"]) is not None, "let go with the clamp open: reflux latched")
clamp_shut_while(None)
s, _ = hands_off()
check(s == 0.0, "clamping after letting go does not undo it -> 0")

# --- 2b. The same mistake with the rod: grip the rod (thumb press untouched), push, let go before the clamp.
new_trial()
push_rod(m.STROKE)
check(m.on_press(m._T["objects"]) and not m.on_thumb_press(m._T["objects"]), "a hand on the rod counts as on the plunger, not as on the thumb press")
hands_off(4)
check(m.reflux(m._T["objects"]) is not None, "rod pushed, let go with the clamp open: reflux latched")
clamp_shut_while(None)
s, _ = hands_off()
check(s == 0.0, "open-line release after a rod push, clamped afterwards -> 0")

# --- 2c. Rod push, clamp shut while the hand is still on the rod, then let go -> 1.0 (the rod is a legitimate way to push).
new_trial()
push_rod(m.STROKE)
q_end = SIM["q"]
for k in range(1, 7):
    c = m.SQUEEZE * k / 6
    step(c=c, left=left_on_clamp(), l_open=max(0.0, 0.25 - 0.25 * k / 6), right=right_on_rod(q_end))
s, ok = hands_off()
check(m.reflux(m._T["objects"]) is None and s == 1.0, f"rod pushed, clamped under pressure, then let go -> 1.0 (got {s})")

# --- 2d. A hand resting on the rod without pushing is not a release.
new_trial()
for _ in range(10): step(0.0, right=right_on_rod(0.0))
hands_off(6)
check(m.reflux(m._T["objects"]) is None, "touching the rod without pushing: no reflux")

# --- 3. Contact chatter: off the press for 2 steps mid-push is not a release.
new_trial()
push(0.010)
step(0.010)
step(0.010)
push(m.STROKE)
check(m.reflux(m._T["objects"]) is None, "2 steps off the press (0.1 s) is not a release")
clamp_shut_while(m.STROKE)
s, _ = hands_off()
check(s == 1.0, f"... and the trial still scores 1.0 (got {s})")

# --- 4. Touching the press without pushing, then lifting, is not a release under pressure.
new_trial()
for _ in range(5):
    step(0.0, right=right_on_press(0.0))
hands_off(4)
check(m.reflux(m._T["objects"]) is None, "touch and lift without pushing: no reflux")
push(m.STROKE)
clamp_shut_while(m.STROKE)
s, _ = hands_off()
check(s == 1.0, "then a proper flush and clamp -> 1.0")

# --- 5. Partial flushes, clamped properly.
for ml, want in ((3.5, 0.5), (1.5, 0.25), (0.5, 0.0), (4.85, 1.0), (4.7, 0.5)):
    new_trial()
    q = ml / m.ML_PER_M
    push(q)
    clamp_shut_while(q)
    s, _ = hands_off()
    check(s == want, f"{ml} mL flushed, clamped under pressure -> {want} (got {s})")

# --- 6. Clamp shut first, then try to push: the line is closed, nothing flushes.
new_trial()
clamp_shut_while(None)
for _ in range(4):
    step(left=None)
push(m.STROKE)
check(m.flushed_ml(m._T["objects"]) < 0.01, "clamp shut first: the plunger is locked, 0 mL flushed")
s, _ = hands_off()
check(s == 0.0 and m.reflux(m._T["objects"]) is None, "-> 0 (nothing flushed), no reflux (nothing pushed)")

# --- 7. The plunger cannot be drawn back.
new_trial()
push(0.02)
s, _ = step(0.005, right=right_on_press(0.02))
check(abs(SIM["q"] - 0.02) < 1e-9, "the plunger's lower stop follows it in: no drawing back")

# --- 8. Never clamped: flushed and held to the end of the trial.
new_trial()
push(m.STROKE)
s, _ = step(m.STROKE, right=right_on_press(m.STROKE))
check(s == 0.0, "still holding, clamp open -> 0")

# --- 9. Still moving at the end.
new_trial()
push(m.STROKE)
clamp_shut_while(m.STROKE)
hands_off(4)
s, _ = step(vc=0.01)
check(s == 0.0, "the clamp jaw still moving -> 0")
s, _ = step()
check(s == 1.0, "settled -> 1.0")

# --- 10. A gripper resting on the clamp after shutting it.
new_trial()
push(m.STROKE)
clamp_shut_while(m.STROKE)
s, _ = step(left=left_on_clamp(), l_open=0.0)
check(s == 0.0, "left fingers still closed on the clamp -> 0")
s, _ = hands_off()
check(s == 1.0, "lifted -> 1.0")

# --- 11. The demo's timing: the clamp closes while the right hand is still on the press.
d = m.demo()


def t_of(plan, label):
    t = 0.0
    for w in plan:
        if w.label == label:
            return t, t + w.duration
        t += w.duration
    return None


sq = t_of(d["left"], "squeeze the clamp shut")
hold = t_of(d["right"], "keep pushing while the clamp shuts")
back = t_of(d["right"], "back off the press")
check(sq is not None and hold is not None and back is not None, "demo labels found")
check(hold[0] <= sq[0] and sq[1] + 0.8 <= back[0], f"demo: squeeze {sq} and its hold inside the push hold {hold}")
check(abs(sum(w.duration for w in d["left"]) - sum(w.duration for w in d["right"])) < 1e-9, "both plans same length")
total = sum(w.duration for w in d["right"])
check(total * 20 <= 600, f"demo plan {total:.1f} s = {total * 20:.0f} steps, under the 600-step run limit")

print("all pass" if not fails else f"{fails} FAILED")
sys.exit(1 if fails else 0)
