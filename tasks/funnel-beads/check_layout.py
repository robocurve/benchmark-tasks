"""Offline checks for funnel_beads.py: the bench layout and the demo's kinematics, without Isaac Sim.

Everything here is NumPy-only -- the task file, the bench geometry and Arena's IK all import before
the simulator starts -- so a layout change can be rejected in seconds instead of after a two-minute
sim launch. Run from this folder:  python check_layout.py
"""

from __future__ import annotations

import importlib.util
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("funnel_beads_check", HERE / "funnel_beads.py")
task = importlib.util.module_from_spec(spec)
sys.modules["funnel_beads_check"] = task
spec.loader.exec_module(task)

bench = task.bench

from yam_arena import ik  # noqa: E402  (after the task file, which patches yam_arena.demo)
from yam_arena.constants import LEFT_BASE_POS, PAD_TIP_PAST_TCP  # noqa: E402

TOP = task.TOP
SEEDS = [
    np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]),
    np.array([-0.5, 0.9, 1.1, -0.7, 0.7, 0.5]),
    np.array([0.5, 0.9, 1.1, -0.7, -0.7, -0.5]),
]

failures: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"{'PASS' if ok else 'FAIL'}  {name}{'  ' + detail if detail else ''}")
    if not ok:
        failures.append(name)


def solve(pos, rot, warm=None):
    quat = ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float))
    best = None
    for seed in ([warm] if warm is not None else []) + SEEDS:
        q, pe, re = ik.solve(pos, quat, q_init=seed, base_pos=LEFT_BASE_POS, iters=120)
        if best is None or (pe, re) < best[1:]:
            best = (q, pe, re)
        if pe < 0.003 and re < 0.05:
            break
    return best


# -- 1. the funnel actually fits the bottle, and the wrong tool does not help --------------

check("the funnel's spout fits down the bottle's neck",
      bench.spout_outer_r() < bench.NECK_R,
      f"spout {(bench.spout_outer_r()) * 1000:.0f} mm outer, neck {bench.NECK_R * 1000:.0f} mm inner")

check("the funnel's cone is too wide to pass through the neck",
      bench.CONE_TOP_R > bench.NECK_R,
      f"cone {bench.CONE_TOP_R * 1000:.0f} mm, neck {bench.NECK_R * 1000:.0f} mm -- it rests on the rim")

check("the spout is wide enough that the beads cannot bridge it",
      bench.SPOUT_R * 2 >= bench.BEAD_R * 2 * 3,
      f"{bench.SPOUT_R * 2 * 1000:.0f} mm bore, {bench.SPOUT_R / bench.BEAD_R:.1f} bead diameters")

check("beads pass the funnel's spout", bench.SPOUT_R > bench.BEAD_R * 1.5,
      f"spout bore {bench.SPOUT_R * 2000:.0f} mm, bead {bench.BEAD_R * 2000:.0f} mm")

check("beads pass the bottle's neck", bench.NECK_R > bench.BEAD_R * 1.5,
      f"neck bore {bench.NECK_R * 2000:.0f} mm, bead {bench.BEAD_R * 2000:.0f} mm")

check("the mouth is narrow against the jug it is poured from",
      bench.NECK_R * 2 < (bench.JUG_R + bench.JUG_WALL) * 2 * 0.7,
      f"mouth {bench.NECK_R * 2000:.0f} mm against an {(bench.JUG_R + bench.JUG_WALL) * 2000:.0f} mm jug")

check("the funnel is the only thing on the bench that could do the job",
      not hasattr(bench, "TUBE_XY"),
      "jug, bottle, funnel")

# -- 2. the beads start in the jug and nowhere near the bottle ----------------------------

beads = bench.bead_objects(TOP)
check("every bead starts inside the jug",
      all(math.dist(b.pos[:2], bench.JUG_XY) < bench.JUG_R - bench.BEAD_R for b in beads),
      f"{len(beads)} beads, furthest {max(math.dist(b.pos[:2], bench.JUG_XY) for b in beads) * 1000:.0f} mm out")

states = {b.name: type("S", (), {"pos": np.array(b.pos)})() for b in beads}
check("no bead starts in the bottle", bench.beads_in_bottle(states, TOP) == 0)

check("the jug holds them below its rim",
      max(b.pos[2] for b in beads) < bench.jug_rim_z(TOP),
      f"highest bead {(max(b.pos[2] for b in beads) - TOP) * 1000:.0f} mm,"
      f" rim {(bench.jug_rim_z(TOP) - TOP) * 1000:.0f} mm")

# -- 3. nothing overlaps anything -----------------------------------------------------------

FOOTPRINT = {
    "bottle": (bench.BOTTLE_XY, bench.BODY_R + bench.WALL),
    "jug": (bench.JUG_XY, bench.JUG_R + bench.JUG_WALL),
    "funnel": (bench.FUNNEL_XY, bench.CONE_TOP_R + bench.WALL),
}
names = sorted(FOOTPRINT)
worst = min(
    (math.dist(FOOTPRINT[a][0], FOOTPRINT[b][0]) - FOOTPRINT[a][1] - FOOTPRINT[b][1], f"{a}/{b}")
    for i, a in enumerate(names) for b in names[i + 1:]
)
check("nothing on the bench overlaps anything else", worst[0] > 0.020,
      f"closest pair {worst[1]} with {worst[0] * 1000:.0f} mm between them")

# A tool's handle sticks out well past its body; the jaws must meet it in clear air.
for tool, grip in (("funnel", bench.funnel_grip_xy()), ("jug", bench.jug_grip_xy())):
    clear = min(math.dist(grip, xy) - r for name, (xy, r) in FOOTPRINT.items() if name != tool)
    check(f"the jaws reach the {tool}'s handle in clear air", clear > 0.030,
          f"{clear * 1000:.0f} mm from the nearest other object")

check("both tools stand clear of the arm pedestals",
      min(math.dist(xy, (0.0, sign * 0.305)) for xy, _ in FOOTPRINT.values() for sign in (1, -1)) > 0.150,
      f"nearest {min(math.dist(xy, (0.0, sign * 0.305)) for xy, _ in FOOTPRINT.values() for sign in (1, -1)) * 1000:.0f} mm")

# -- 4. the demo is reachable throughout ----------------------------------------------------

plan = task.demo()["left"]
worst_pos, worst_rot, worst_label = 0.0, 0.0, ""
warm = None
for wp in plan:
    q, pe, re = solve(np.asarray(wp.pos, dtype=float), wp.yaw, warm)
    warm = q
    if pe > worst_pos:
        worst_pos, worst_label = pe, wp.label
    worst_rot = max(worst_rot, re)
check("every demo waypoint is reachable", worst_pos < 0.005 and worst_rot < math.radians(5),
      f"worst {worst_pos * 1000:.1f} mm ({worst_label}), {math.degrees(worst_rot):.1f} deg")

# Tipping the jug swings its rim a long way below the hand: at 84 degrees nearly a full radius of
# it.
_jug_rise = bench.jug_grip_z(TOP) - (TOP + 0.004) - PAD_TIP_PAST_TCP
_jug_lip_z = (task.POUR_Z + (bench.JUG_H - _jug_rise) * math.cos(task.POUR_TILT)
              - bench.JUG_R * math.sin(task.POUR_TILT))
_funnel_rim_z = bench.funnel_seat_body_z(TOP) + bench.funnel_rim_above_body()
check("the jug's rim clears the funnel as it goes over",
      0.020 < _jug_lip_z - _funnel_rim_z < 0.060,
      f"{(_jug_lip_z - _funnel_rim_z) * 1000:.0f} mm between the jug's lip and the funnel's rim")

check("the pour reaches the tilt the jug needs to empty", task.POUR_TILT >= math.radians(60),
      f"{math.degrees(task.POUR_TILT):.0f} deg")

# What decides whether the jug keeps its beads or sheds them is the slope of its lining, not the
# tilt on its own.
TAPER = math.atan2(bench.JUG_R - bench.JUG_BORE, bench.JUG_H)
MU = (bench.BEAD_FRICTION[1] + bench.JUG_LINING[1]) / 2
GRIP_ANGLE = math.atan(MU)


def lining_slope(tilt: float) -> float:
    """How far the lining's downhill wall points at the mouth, in radians. Negative: uphill."""
    return tilt + TAPER - math.pi / 2


check("the jug holds its beads at the tilt it is carried at",
      lining_slope(task.CARRY_TILT) < -GRIP_ANGLE,
      f"carried at {math.degrees(task.CARRY_TILT):.0f} deg, lining still"
      f" {-math.degrees(lining_slope(task.CARRY_TILT)):.0f} deg uphill")

# -- 5. the funnel ends up seated, not hovering ---------------------------------------------

tip = bench.funnel_spout_tip_z(TOP)
check("the seated spout reaches down the neck",
      tip < bench.bottle_mouth_z(TOP) - 0.020,
      f"spout tip {(tip - TOP) * 1000:.0f} mm, mouth"
      f" {(bench.bottle_mouth_z(TOP) - TOP) * 1000:.0f} mm")

check("the spout is long enough to be guided by the neck",
      bench.SPOUT_H > bench.NECK_H * 0.8,
      f"{bench.SPOUT_H * 1000:.0f} mm spout, {bench.NECK_H * 1000:.0f} mm neck")

check("the cone is too wide to follow the spout down the neck",
      bench.CONE_TOP_R > bench.NECK_R,
      f"cone {bench.CONE_TOP_R * 2000:.0f} mm, mouth {bench.NECK_R * 2000:.0f} mm -- it jams on the rim")

check("the jaws fit round the grip post", bench.POST_SIDE < 0.080,
      f"{bench.POST_SIDE * 1000:.0f} mm post in a 94 mm jaw")

check("the jaws clear the bottle while the funnel is seated",
      bench.POST_R > bench.MOUTH_LEAD_R + bench.WALL + 0.020,
      f"pads at {bench.POST_R * 1000:.0f} mm, lip {(bench.MOUTH_LEAD_R + bench.WALL) * 1000:.0f} mm outer")

# The lip is what makes seating forgiving: it is the radius the spout can be out by and still be
# guided in, rather than landing on a rim and levering the funnel over.
catch = bench.MOUTH_LEAD_R - (bench.spout_outer_r())
# The funnel used to arrive about 11 mm off the bottle's axis; pausing over the bottle before the
# descent, so it stops swinging on its post, brought that down to a couple of millimetres.
check("the bottle's lip catches a spout that arrives off-centre", catch > 0.010,
      f"{catch * 1000:.0f} mm of lead-in, against about 2 mm of drift on the way over")

check("the spout has play in the neck rather than jamming on the rim",
      bench.NECK_R - bench.spout_outer_r() > 0.003,
      f"{(bench.NECK_R - bench.spout_outer_r()) * 1000:.1f} mm of play all round")

# -- 6. the run fits the limit --------------------------------------------------------------

seconds = sum(wp.duration for wp in plan)
check("the demo fits a 2,400-step run at 20 Hz", seconds * 20 < 2400,
      f"{seconds:.0f} s, {seconds * 20:.0f} steps")

check("the pour slopes the jug's lining downhill harder than the beads can hold",
      lining_slope(task.POUR_TILT) > GRIP_ANGLE + math.radians(3),
      f"tipped to {math.degrees(task.POUR_TILT):.0f} deg, lining"
      f" {math.degrees(lining_slope(task.POUR_TILT)):.0f} deg downhill against a"
      f" {math.degrees(GRIP_ANGLE):.0f} deg grip")

check("the beads start inside the jug's cone, not perched on its taper",
      bench.BEAD_PITCH * math.sqrt(2) + bench.BEAD_R < bench.JUG_BORE,
      f"corner bead {(bench.BEAD_PITCH * math.sqrt(2) + bench.BEAD_R) * 1000:.0f} mm out,"
      f" bore {bench.JUG_BORE * 1000:.0f} mm")

check("the jaws squeeze each tool rather than crushing past it",
      bench.funnel_grip_opening() > 0 and bench.jug_grip_opening() > 0,
      f"funnel {bench.funnel_grip_opening() * 1000:.0f} mm, jug {bench.jug_grip_opening() * 1000:.0f} mm")

print()
print("all checks passed" if not failures else f"{len(failures)} FAILED: " + ", ".join(failures))
sys.exit(1 if failures else 0)
