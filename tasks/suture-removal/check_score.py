"""The hook's way under the stitch, the cut, the instruments' return and the scoring table on hand-built cases, no simulator: python check_score.py"""
import importlib.util
import pathlib
import sys
import types

import numpy as np

# task.py imports the simulator's modules at import time; the gates (hook_beneath, notch_step, at_rest) and the table (_tier) need none of them
for name in ("demokit", "mechkit", "yam_arena", "yam_arena.task_file"):
    sys.modules[name] = types.ModuleType(name)
sys.modules["yam_arena.task_file"].TABLE_TOP = {"station": 0.0}
mk = sys.modules["mechkit"]
mk.Look = lambda **kw: kw
mk.rgb = lambda *a: a
for fn in ("box", "col_box", "slider", "hinge", "mechanism", "free", "fixed", "on_step", "on_reset"):
    setattr(mk, fn, lambda *a, **kw: None)
spec = importlib.util.spec_from_file_location("task", pathlib.Path(__file__).with_name("task.py"))
task = importlib.util.module_from_spec(spec)
spec.loader.exec_module(task)

bad = 0
SKIN, R = task.SKIN, task.R_ARC
LIFT = 0.0022  # the knot lifted 2.2 mm, as in the demo


def path(points):
    """points: the hook (x across the strand's line, y along it from the pad's middle, height) per step; whether it ends beneath the stitch."""
    st = (False, False, None, False)
    for h in points:
        st = task.hook_beneath(st, h, LIFT, 0.0)
    return st[0]


low = SKIN + 0.0007  # the hook's height as it slides, under the lifted loop
routes = [
    ("slid in level from beyond the far leg (the demo's way)", [(0.0, y, low) for y in np.linspace(-R - 0.008, -R, 9)], True),
    ("slid in level from beside the strand, across it", [(x, -R, low) for x in np.linspace(0.008, 0.0, 9)], True),
    ("brought down from above onto the far leg, through the loop", [(0.0, -R, z) for z in np.linspace(SKIN + 0.010, low, 9)], False),
    ("brought down beside the strand inside the 3 mm band, then slid under it", [(0.002, -R, z) for z in np.linspace(SKIN + 0.010, low, 9)] + [(0.0, -R, low)], False),
    ("brought down 8 mm beyond the leg, then slid in level", [(0.0, -R - 0.008, z) for z in np.linspace(SKIN + 0.010, low, 5)] + [(0.0, y, low) for y in np.linspace(-R - 0.008, -R, 9)], True),
    ("slid in level, then lifted out over the loop and back down", [(0.0, y, low) for y in np.linspace(-R - 0.008, -R, 9)] + [(0.0, -R, SKIN + 0.004), (0.0, -R, low)], False),
    ("brought down through the loop slowly, 0.25 mm a step", [(0.0, -R, z) for z in np.arange(SKIN + 0.010, low, -0.00025)], False),
    ("brought down through the loop very slowly, 0.1 mm a step", [(0.0, -R, z) for z in np.arange(SKIN + 0.006, low, -0.0001)], False),
    ("brought down over the loop slowly, then slid sideways inside the band", [(0.0, -R, z) for z in np.arange(SKIN + 0.006, low, -0.0002)] + [(x, -R, low) for x in (0.001, 0.002, 0.0)], False),
]
for name, pts, want in routes:
    got = path(pts)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} hook beneath {got!s:5} (want {want!s:5})  {name}")

# the cut, read in the cutter's frame: the leg into the notch through its mouth, then pressed onto its inner edge (the knot lifted 2.2 mm)
import math
legy = -R
def leg_points():
    return np.array([[0.0, legy, z] for z in np.linspace(SKIN, SKIN + LIFT + 0.0008, 6)])
def cuts(hook_path, yaw=0.0):
    """hook_path: the notch floor's middle (station frame, the pad's middle at x 0) per step; the cutter upright, turned `yaw` degrees about z."""
    c, s_ = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    Rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    floor_local = np.array([(task.NOTCH_X0 + task.NOTCH_X1) / 2, 0.0, -task.TIP_L + task.FOOT_H])
    st, out = (False, False), None
    for hk in hook_path:
        origin = np.asarray(hk) - Rz @ floor_local
        local = (leg_points() - origin) @ Rz
        st, k = task.notch_step(st, local, True)
        out = out if out is not None else k
    return out is not None
lvl = SKIN + 0.0005
line = lambda x0, x1, y=legy, z=lvl, n=12: [(x, y, z) for x in np.linspace(x0, x1, n)]  # noqa: E731
cut_paths = [
    ("the demo: slid in across the leg from 10 mm out, then pressed 1.5 mm on", line(0.010, 0.0) + line(0.0, -0.0015, n=6)[1:], True),
    ("slid in, but stopped with the leg in the notch, never pressed onto its edge", line(0.010, 0.0), False),
    ("stopped 1.5 mm short of the leg (on the open side), then nudged sideways", line(0.010, 0.0015) + [(0.0015, legy + dy, lvl) for dy in (0.001, 0.002)], False),
    ("the same path 1.0 mm off the leg, across the blade's plane (+)", line(0.010, 0.0, y=legy + 0.001) + line(0.0, -0.0015, y=legy + 0.001, n=6)[1:], False),
    ("the same path 1.0 mm off the leg, across the blade's plane (-)", line(0.010, 0.0, y=legy - 0.001) + line(0.0, -0.0015, y=legy - 0.001, n=6)[1:], False),
    ("the same path 1.5 mm off the leg, across the blade's plane (+)", line(0.010, 0.0, y=legy + 0.0015) + line(0.0, -0.0015, y=legy + 0.0015, n=6)[1:], False),
    ("the same path 1.5 mm off the leg, across the blade's plane (-)", line(0.010, 0.0, y=legy - 0.0015) + line(0.0, -0.0015, y=legy - 0.0015, n=6)[1:], False),
    ("the same path 0.6 mm off the leg (the strand touching the steel)", line(0.010, 0.0, y=legy + 0.0006) + line(0.0, -0.0015, y=legy + 0.0006, n=6)[1:], True),
    ("the same path 1.6 mm off the leg, across the blade's plane", line(0.010, 0.0, y=legy + 0.0016) + line(0.0, -0.0015, y=legy + 0.0016, n=6)[1:], False),
    ("the same path 2 mm off the leg, across the blade's plane", line(0.010, 0.0, y=legy - 0.002) + line(0.0, -0.0015, y=legy - 0.002, n=6)[1:], False),
    ("the cutter turned 90 degrees, the same path", line(0.010, 0.0) + line(0.0, -0.0015, n=6)[1:], False),
    ("slid in from the spine's side (from -x)", line(-0.010, 0.0) + line(0.0, 0.0015, n=6)[1:], False),
    ("the notch's floor 4 mm over the skin (the leg below the notch)", line(0.010, 0.0, z=SKIN + 0.004) + line(0.0, -0.0015, z=SKIN + 0.004, n=6)[1:], False),
]
for name, path_, want in cut_paths:
    got = cuts(path_, yaw=90.0 if "turned" in name else 0.0)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} cut {got!s:5} (want {want!s:5})  {name}")

# back in its tray: let go and at rest, inside the tray and off the gauze (the forceps' tray at (0.34, 0.16), 16 x 9 cm; the gauze at (0.24, 0.20))
tray = (0.34, 0.16, 0.08, 0.045)
gauze = (0.24, 0.20, 0.0375, 0.0375)
TOP = task.TOP
standing = np.array([[0.34, 0.16, TOP + 0.004 + z] for z in np.linspace(0.0, 0.12, 5)])
far_tips = np.tile(np.array([0.30, 0.30, TOP + 0.20]), (24, 1))
gripped = far_tips.copy()
gripped[0] = standing[-1] + np.array([0.004, 0.0, -0.03])
gripped[6] = standing[-1] + np.array([-0.004, 0.0, -0.03])
on_gauze = np.array([[0.255 + 0.02 * k, 0.17, TOP + 0.003] for k in range(5)])
rests = [
    ("standing in its rest, both hands away", standing, far_tips, True),
    ("standing in its rest, still gripped", standing, gripped, False),
    ("held 3 cm above its rest's floor, hands 20 cm away (the height alone)", standing + np.array([0.0, 0.0, 0.03]), far_tips, False),
    ("lying across the corner where the gauze overlaps the tray", on_gauze - np.array([0.02, 0.0, 0.0]), far_tips, False),
]
for name, pts, tips, want in rests:
    got = task.at_rest(pts, tips, TOP + 0.004, tray, gauze)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} in tray {got!s:5} (want {want!s:5})  {name}")

# letting go of the stitch: over the gauze once drawn out it lands there; after the cut, anywhere else it is dropped for good (the scorer then refuses
# any new grasp, so it cannot be picked up again at the knot's old place and carried to the gauze)
releases = [
    ("drawn out, opened over the gauze", (True, 1.0, (0.24, 0.20, TOP + 0.02)), (True, False)),
    ("drawn out, opened over the pad (then re-closed at the knot's old place: refused)", (True, 1.0, (0.32, 0.0, TOP + 0.04)), (False, True)),
    ("cut, opened over the pad half way through the draw", (True, 0.5, (0.32, -0.005, TOP + 0.04)), (False, True)),
    ("cut, not drawn out, opened over the gauze", (True, 0.3, (0.24, 0.20, TOP + 0.02)), (False, True)),
    ("not yet cut, opened at the knot (it is still in the skin, and may be taken again)", (False, 0.0, (0.32, 0.0045, TOP + 0.031)), (False, False)),
]
for name, args, want in releases:
    got = task.let_go(*args)
    ok = got == want
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} let go {got} (want {want})  {name}")

base = {"broke": False, "dug": False, "lifted_max_mm": 0.0, "cut": False, "cut_side": None, "cut_h_mm": None, "drawn": 0.0, "gap_max_mm": 0.0,
        "on_gauze": False, "in_rests": [False, False], "clear": False}
done = dict(lifted_max_mm=2.2, cut=True, cut_side="far", cut_h_mm=0.4, drawn=1.0, gap_max_mm=0.2, on_gauze=True, in_rests=[True, True], clear=True)
cases = [
    ("the full solution: lifted, far leg cut flush, drawn toward the wound, on the gauze, instruments back, hands clear", done, 1.0),
    ("all done but the instruments left out of their trays", {**done, "in_rests": [True, False]}, 0.75),
    ("cut and drawn, but let go off the gauze", {**done, "on_gauze": False}, 0.75),
    ("cut flush, never drawn out", {**done, "drawn": 0.3, "on_gauze": False}, 0.5),
    ("drawn out away from the wound: it opened 2.5 mm", {**done, "gap_max_mm": 2.5}, 0.5),
    ("the far leg cut 1.4 mm above the skin (an outside length left to pull through)", {**done, "cut_h_mm": 1.4}, 0.25),
    ("the near leg cut instead of the far one", {**done, "cut_side": "near"}, 0.25),
    ("both legs cut (the buried part left in the skin), then drawn and tidied", {**done, "cut_side": "both"}, 0.5),
    ("the knot lifted 2.2 mm, nothing more", dict(lifted_max_mm=2.2), 0.25),
    ("the knot lifted under 1 mm", dict(lifted_max_mm=0.6), 0.0),
    ("the knot pulled up past 4.5 mm before the cut (torn through the skin)", {**done, "broke": True}, 0.0),
    ("the cutter's foot pressed more than 2 mm into the skin", {**done, "dug": True}, 0.0),
    ("nothing done", {}, 0.0),
]
for name, over, want in cases:
    got = task._tier({**base, **over})
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok ' if ok else 'BAD'} {got:.2f} (want {want:.2f})  {name}")
sys.exit(1 if bad else 0)
