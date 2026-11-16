"""Real-size reach screen for mortise-latchbolt-handing-flip (official yam_arena IK, straight-down grip + yaw).
Run: PYTHONPATH=<official runtime> <arena venv python> -B screen.py   (exit 1 = blocked)

Real product: commercial grade-1 mortise lock case 113 x 154 x 25 mm (4-7/16 x 6-1/16 x 1 in), 3/4 in (19 mm)
latchbolt throw (maker's L/LV9000 installation instructions 108367, p.1). The case stands faceplate-up in a bench
holder whose floor is HOLDER_FLOOR above the bench, so the faceplate top is at HOLDER_FLOOR + 113 + 3.2 mm.
The pads pinch the latchbolt's end faces 10 mm below its top; handing pull = PULL above the rated throw.
"""
import sys
from yam_arena import station
from yam_arena.constants import CONTROL_HZ, FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import ArmPlan, Waypoint as W

T = station.TABLE_TOP
HOLDER_FLOOR, CASE_DEPTH, FRONT_T, THROW, PULL = 0.012, 0.113, 0.0032, 0.019, 0.017
FACE_TOP = HOLDER_FLOOR + CASE_DEPTH + FRONT_T
GRIP = FACE_TOP + THROW - 0.008 + PAD_TIP_PAST_TCP          # pads 8 mm below latch top
PULLED = GRIP + PULL
HOVER = FACE_TOP + THROW + 0.013 + PAD_TIP_PAST_TCP + 0.018
X, Y = 0.32, -0.15
print(f"face top {FACE_TOP:.4f} m, grip TCP {GRIP:.4f}, pulled TCP {PULLED:.4f}, hover {HOVER:.4f} (ceiling 0.215)")
cases = [("control low", "right", 0.25, -0.15, 0.10, 0.0),
         ("control known ceiling", "right", 0.25, -0.15, 0.21, 0.0)]
for yaw in (0.0, 1.5708, -1.5708):
    for h, what in ((GRIP, "grip"), (PULLED, "pulled"), (HOVER, "hover")):
        cases.append((f"{what} yaw {yaw:+.2f}", "right", X, Y, h, yaw))
# left arm: closed-finger push on the jamb's back end (y = +0.020), from behind the open jamb to 41 mm past the stop
PUSH_TCP = FACE_TOP + 0.003 + 0.008 + PAD_TIP_PAST_TCP
for px in (0.157, 0.182, 0.25, 0.317):
    cases.append((f"jamb push x {px:.3f}", "left", px, 0.020, PUSH_TCP, 0.0))
cases.append(("jamb push lift", "left", 0.247, 0.020, PUSH_TCP + 0.03, 0.0))
cases.append(("right park", "right", 0.16, -0.32, 0.20, 0.0))
bad = 0
for what, arm, x, y, h, yaw in cases:
    p = ArmPlan([W(1.5, (0.15, -0.20 if arm == "right" else 0.20, T + 0.15), FINGER_OPEN, "start"),
                 W(2.0, (x, y, T + h), FINGER_OPEN, "reach", yaw=yaw), W(1.0, (x, y, T + h), FINGER_OPEN, "hold", yaw=yaw)],
                station.BASES[arm])
    for k in range(round(p.duration * CONTROL_HZ) + 5):
        p.action7((k + 1) / CONTROL_HZ)
    ok = p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1
    bad += (not ok) and not what.startswith("control known")
    print(f"{arm:5s} ({x:.2f},{y:+.3f}) h={h:.3f} worst_ik_mm={p.worst_ik_mm:7.3f} deg={p.worst_ik_deg:6.3f} {'OK' if ok else 'FAIL'} [{what}]")
print("SCREEN", "PASS" if not bad else "BLOCKED")
sys.exit(1 if bad else 0)
