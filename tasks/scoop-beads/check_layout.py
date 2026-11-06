"""Offline checks for scoop_beads.py: the bench layout and the demo's kinematics, without Isaac Sim.

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
spec = importlib.util.spec_from_file_location("scoop_beads_check", HERE / "scoop_beads.py")
task = importlib.util.module_from_spec(spec)
sys.modules["scoop_beads_check"] = task
spec.loader.exec_module(task)

bench = task.bench
simkit = task.simkit

from yam_arena import ik  # noqa: E402  (after the task file, which patches yam_arena.demo)
from yam_arena.constants import LEFT_BASE_POS  # noqa: E402

TOP = task.TOP
CENTRE = (0.330, 0.000)  # the middle of the bench's working area
SOURCE_R = bench.SOURCE_R + bench.SOURCE_WALL  # the tray's outside radius
TARGET_R = bench.TARGET_EXTENT / 2
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
    """Best IK solution over a few seeds, warm-started from the previous waypoint."""
    quat = ik.matrix_to_quat_xyzw(np.asarray(rot, dtype=float))
    best = None
    for seed in ([warm] if warm is not None else []) + SEEDS:
        q, pe, re = ik.solve(pos, quat, q_init=seed, base_pos=LEFT_BASE_POS, iters=120)
        if best is None or (pe, re) < best[1:]:
            best = (q, pe, re)
        if pe < 0.003 and re < 0.05:
            break
    return best


# -- 1. the bowls sit either side of the middle -------------------------------------------

mid = tuple((a + b) / 2 for a, b in zip(bench.SOURCE_XY, bench.TARGET_XY))
check("bowl pair is centred on the bench", math.dist(mid, CENTRE) < 0.001,
      f"midpoint ({mid[0]:.3f}, {mid[1]:+.3f})")

gap = math.dist(bench.SOURCE_XY, bench.TARGET_XY)
check("the two vessels are far enough apart to need carrying", gap - SOURCE_R - TARGET_R > 0.020,
      f"{gap * 1000:.0f} mm apart, {(gap - SOURCE_R - TARGET_R) * 1000:.0f} mm of clear bench between them")

# -- 2. the tools stand clear of both bowls ----------------------------------------------

# A tool is a long thin thing, not a disc: the spatula runs *away* from the bowls, so a bounding
# circle around it would condemn a layout that is actually clear.
TOOLS = {
    "scoop": (bench.SCOOP_XY, bench.SCOOP_YAW, bench.PAN_L / 2,
              bench.HANDLE_DY - bench.HANDLE_L / 2, bench.PAN_W / 2),
    "spatula": (bench.SPATULA_XY, bench.SPATULA_YAW, bench.SPATULA_EXTENT / 2,
                -bench.SPATULA_EXTENT / 2, 0.035),
}


def tool_points(name, step=0.005):
    xy, yaw, front, back, _ = TOOLS[name]
    n = max(2, int(round((front - back) / step)) + 1)
    return [(xy[0] + d * math.cos(yaw), xy[1] + d * math.sin(yaw)) for d in np.linspace(back, front, n)]


for tool in TOOLS:
    half = TOOLS[tool][4]
    clear = min(math.dist(pt, xy) - r - half
                for pt in tool_points(tool)
                for xy, r in ((bench.SOURCE_XY, SOURCE_R), (bench.TARGET_XY, TARGET_R)))
    check(f"{tool} does not overlap a vessel", clear > 0.0, f"{clear * 1000:.0f} mm of clearance")

# Both tools point the same way, so what matters between them is the gap across their headings,
# not centre-to-centre: laid side by side they are only as wide as their pans.
_offset = np.subtract(bench.SPATULA_XY, bench.SCOOP_XY)
_across = abs(float(-_offset[0] * math.sin(bench.SCOOP_YAW) + _offset[1] * math.cos(bench.SCOOP_YAW)))
check("the two tools do not overlap each other", _across > bench.PAN_W / 2 + 0.040,
      f"{_across * 1000:.0f} mm between their axes")

# Clear of the arms' own pedestals, which stand at (0, +-0.305).
_to_base = min(math.dist(pt, (0.0, sign * 0.305))
               for tool in TOOLS for pt in tool_points(tool) for sign in (1, -1))
check("both tools stand clear of the arm pedestals", _to_base > 0.120,
      f"nearest tool point {_to_base * 1000:.0f} mm from a base")

check("the tools are laid parallel", abs(bench.SCOOP_YAW - bench.SPATULA_YAW) < 1e-9)

# -- 3. each tool is held up by its own posts ---------------------------------------------

racks = bench.rack_objects(TOP)
check("every post reaches the same height", all(abs(r.pos[2] - (TOP + bench.RACK_TOP / 2)) < 1e-9 for r in racks))

for tool, xy, yaw in (("scoop", bench.SCOOP_XY, bench.SCOOP_YAW),
                      ("spatula", bench.SPATULA_XY, bench.SPATULA_YAW)):
    along = (math.cos(yaw), math.sin(yaw))
    across = [abs(-(r.pos[0] - xy[0]) * along[1] + (r.pos[1] - xy[1]) * along[0]) for r in racks]
    under = sum(1 for d in across if d < bench.POST)
    check(f"{tool} has posts running along its own heading", under >= 2,
          f"{under} posts within a post-width of its axis")

spatula = bench.spatula_object(TOP)
check("the spatula is raised onto its rack rather than left on the bench",
      isinstance(spatula, simkit.Lifted) and abs(spatula.lift - bench.RACK_TOP) < 1e-9,
      f"lift {spatula.lift * 1000:.0f} mm, posts {bench.RACK_TOP * 1000:.0f} mm")

# -- 4. the beads start in the source bowl, under its rim ---------------------------------

beads = bench.bead_objects(TOP)
check("every bead starts inside the source bowl",
      all(math.dist(b.pos[:2], bench.SOURCE_XY) < bench.SOURCE_INNER_R for b in beads),
      f"{len(beads)} beads, furthest {max(math.dist(b.pos[:2], bench.SOURCE_XY) for b in beads) * 1000:.0f} mm out")

_states = {b.name: type("S", (), {"pos": np.array(b.pos)})() for b in beads}
check("no bead starts in the target bowl", bench.beads_in(_states, bench.TARGET_XY, TOP) == 0)
check("the scorer sees every bead in the source bowl",
      bench.beads_in(_states, bench.SOURCE_XY, TOP) == bench.BEAD_COUNT,
      f"{bench.beads_in(_states, bench.SOURCE_XY, TOP)} of {bench.BEAD_COUNT}")

# -- 5. the demo's waypoints are all reachable --------------------------------------------

plan = task.demo()["left"]
worst_pos, worst_rot, worst_label = 0.0, 0.0, ""
worst_rot_seen, worst_rot_label, rot_fail = 0.0, "", False
warm = None
for wp in plan:
    q, pe, re = solve(np.asarray(wp.pos, dtype=float), wp.yaw, warm)
    warm = q
    if pe > worst_pos:
        worst_pos, worst_label = pe, wp.label
    # Attitude matters where the pan has to lie flat on the tray's floor; over the far bowl it
    # does not, because a pan open at the lip tips its load out either way.
    limit = math.radians(12) if wp.label.startswith(("tip it out", "let it run out")) else math.radians(5)
    if re > limit:
        worst_rot, worst_rot_label = re, wp.label
        rot_fail = True
    worst_rot_seen = max(worst_rot_seen, re)
check("every demo waypoint is reachable", worst_pos < 0.005 and not rot_fail,
      f"worst {worst_pos * 1000:.1f} mm ({worst_label}),"
      f" {math.degrees(worst_rot_seen):.1f} deg of attitude")

# -- 6. the hand stays out of the bowl while the pan is in it -----------------------------
#
# This is the one that matters. A scoop whose handle is too short for the bowl puts the finger pads
# inside it alongside the pan, and the bowl wall then levers the scoop out of the jaws -- which is
# exactly what happened with the 90 mm handle this started with: pads 45 mm from the bowl's centre
# and 39 mm below its rim, and the scoop dropped on the first dig of every run.

worst_r, worst_over, worst_where = 9.9, 9.9, ""
for across in task.DIG_PASSES:
    for _, pan, tilt, label, *_rest in task.dig_pass(across):
        tips = bench.pad_tips(pan, tilt, task.PSI)
        out = math.dist(tips[:2], bench.SOURCE_XY) - SOURCE_R
        over = tips[2] - (TOP + bench.SOURCE_RIM)
        if max(out, over) < max(worst_r, worst_over):
            worst_r, worst_over, worst_where = out, over, f"{label} (across {across * 1000:+.0f} mm)"
check("the hand stays clear of the dish through every dig", max(worst_r, worst_over) > 0.010,
      f"worst {worst_where}: {worst_r * 1000:+.0f} mm past the rim, {worst_over * 1000:+.0f} mm above it")

# -- 7. the handle rides over the tray's wall, not onto it --------------------------------
#
# The handle runs back from the pan in the plane of the pads, so on the way out of the tray it has
# to cross the wall. It crosses it about 110 mm behind the pan, rising at the pan's own tilt; the
# wall is kept at 30 mm so that a working dig tilt is enough to clear it.

worst_gap, worst_at = 9.9, ""
for across in task.DIG_PASSES:
    for _, pan, tilt, label, *_rest in task.dig_pass(across):
        offset = ((pan[0] - bench.SOURCE_XY[0]) * math.cos(task.PSI)
                  + (pan[1] - bench.SOURCE_XY[1]) * math.sin(task.PSI))
        crossing = offset + SOURCE_R
        gap = (pan[2] + crossing * math.tan(tilt + bench.CRANK)) - (TOP + bench.SOURCE_RIM)
        if gap < worst_gap:
            worst_gap, worst_at = gap, f"{label} (across {across * 1000:+.0f} mm)"
check("the handle clears the tray's wall through every dig", worst_gap > 0.005,
      f"worst {worst_at}: {worst_gap * 1000:+.0f} mm over the wall")

# -- 8. the pan stays on the flat part of the dish's floor --------------------------------

worst_corner, corner_at = 0.0, ""
for across in task.DIG_PASSES:
    for _, pan, _tilt, label, *_rest in task.dig_pass(across):
        along = ((pan[0] - bench.SOURCE_XY[0]) * math.cos(task.PSI)
                 + (pan[1] - bench.SOURCE_XY[1]) * math.sin(task.PSI))
        side = ((pan[1] - bench.SOURCE_XY[1]) * math.cos(task.PSI)
                - (pan[0] - bench.SOURCE_XY[0]) * math.sin(task.PSI))
        reach = math.hypot(abs(along) + bench.PAN_L / 2, abs(side) + bench.PAN_W / 2)
        if reach > worst_corner:
            worst_corner, corner_at = reach, f"{label} (across {across * 1000:+.0f} mm)"
# The sweep may finish a little up the dish's wall -- that is where the beads are cornered -- but
# it must stay well inside the dish itself.
check("the pan stays inside the dish", worst_corner < SOURCE_R - 0.015,
      f"worst corner {worst_corner * 1000:.0f} mm out, dish {SOURCE_R * 1000:.0f} mm,"
      f" flat to {bench.SOURCE_FLAT_R * 1000:.0f} mm ({corner_at})")

# -- 9. the dig starts on clear floor, beyond the pile ------------------------------------

# The near edge of the pile, along the draw -- not its far edge: the lip only has to start behind
# the first bead it will meet.
_near = min(
    (b.pos[0] - bench.SOURCE_XY[0]) * math.cos(task.PSI)
    + (b.pos[1] - bench.SOURCE_XY[1]) * math.sin(task.PSI)
    for b in beads
) - bench.BEAD_R
_entry = task.dig_pass(0.0)[1][1]  # the pan where it first touches the floor
_lip = ((_entry[0] - bench.SOURCE_XY[0]) * math.cos(task.PSI)
        + (_entry[1] - bench.SOURCE_XY[1]) * math.sin(task.PSI)) + bench.PAN_L / 2
# 5 mm, not 15: the bowl's floor is 138 mm across and the charge of beads takes up 61 mm of it, so
# there is not a scoop-length of clear floor to be had at either end.
check("the dig starts on clear floor, behind the pile", _lip < _near - 0.005,
      f"lip enters at {_lip * 1000:+.0f} mm, nearest bead edge at {_near * 1000:+.0f} mm")

# And the draw has to carry the lip past the far edge, or the back of the pile is never touched.
_far = max(
    (b.pos[0] - bench.SOURCE_XY[0]) * math.cos(task.PSI)
    + (b.pos[1] - bench.SOURCE_XY[1]) * math.sin(task.PSI)
    for b in beads
) + bench.BEAD_R
_end = task.dig_pass(0.0)[-2][1]  # the pan at the end of the draw
_lip_end = ((_end[0] - bench.SOURCE_XY[0]) * math.cos(task.PSI)
            + (_end[1] - bench.SOURCE_XY[1]) * math.sin(task.PSI)) + bench.PAN_L / 2
check("the draw carries the lip past the far edge of the pile", _lip_end > _far,
      f"lip ends at {_lip_end * 1000:+.0f} mm, far bead edge at {_far * 1000:+.0f} mm")

# -- 10. the dig really puts the lip on the dish's floor ----------------------------------

# The commanded lip sits a measured sag *below* the floor, because that is how much the scoop gives
# in the jaws: command it level with the floor and it rides 6 mm above it.
lip_z = bench.dig_pan_z(TOP, task.DIG_TILT) - bench.scoop_lip_rise(task.DIG_TILT) - bench.PAN_FLOOR / 2
check("the dig commands the lip onto the floor, allowing for the sag",
      abs((lip_z + bench.GRIP_SAG + bench.DIG_PRESS) - bench.source_floor_z(TOP)) < 1e-9,
      f"commanded {(lip_z - TOP) * 1000:.1f} mm, floor {bench.SOURCE_FLOOR * 1000:.1f} mm,"
      f" sag {bench.GRIP_SAG * 1000:.0f} mm + press {bench.DIG_PRESS * 1000:.0f} mm")
check("the lip gets under a bead rather than riding over it",
      bench.GRIP_SAG < bench.BEAD_R,
      f"lip settles within {bench.GRIP_SAG * 1000:.0f} mm of the floor, bead centres sit at"
      f" {bench.BEAD_R * 1000:.0f} mm")

# -- 11. the run fits the limit ------------------------------------------------------------

seconds = sum(wp.duration for wp in plan)
check("the demo fits a 2,400-step run at 20 Hz", seconds * 20 < 2400, f"{seconds:.0f} s, {seconds * 20:.0f} steps")

# -- 12. the jaws actually close on the handle ---------------------------------------------

opening = bench.HANDLE_W - bench.GRIP_SQUEEZE
check("the grip squeezes the handle rather than crushing past it", 0.0 < opening < bench.HANDLE_W,
      f"jaws {opening * 1000:.0f} mm on a {bench.HANDLE_W * 1000:.0f} mm handle")

print()
print("all checks passed" if not failures else f"{len(failures)} FAILED: " + ", ".join(failures))
sys.exit(1 if failures else 0)
