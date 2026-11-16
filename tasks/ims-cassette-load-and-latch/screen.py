"""Reach screen for ims-cassette-load-and-latch (official yam_arena IK): every demo waypoint of both arms.
Run: PYTHONPATH=<official runtime> <arena venv python> -B screen.py   (exit 1 = blocked)"""
import sys
from yam_arena import station
from yam_arena.constants import CONTROL_HZ
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

T = load(__import__("pathlib").Path(__file__).resolve().parent / "ims_cassette_load_and_latch.py")
bad = 0
for arm, wps in T.demo().items():
    p = ArmPlan(wps, station.BASES[arm])
    for k in range(round(p.duration * CONTROL_HZ) + 5):
        p.action7((k + 1) / CONTROL_HZ)
    ok = p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1
    bad += not ok
    print(f"{arm:5s} waypoints={len(wps)} worst_ik_mm={p.worst_ik_mm:.3f} deg={p.worst_ik_deg:.3f} {'OK' if ok else 'FAIL'}")
print("SCREEN", "PASS" if not bad else "BLOCKED")
sys.exit(1 if bad else 0)
