"""Keep a hand-off tray stocked, one block at a time, for a second robot arm that empties it on a fixed clock.

Station layout (station frame: x forward from the arm bases, y left, z above the table top):
- the LEFT YAM arm is the arm under test; the RIGHT YAM arm is the coworker and is driven by a fixed routine
  (``coworker.py`` replaces the right half of every action with the routine's joint targets, whatever the policy sends);
- a white hand-off tray (station B: an 89 x 89 x 25 mm square polystyrene weigh boat) just right of the middle of the
  table, preloaded with one 1.2 in (30.5 mm) unfinished wood cube (block_1);
- five more wood cubes (block_2..block_6) on the table on the left arm's side, laid out like the five on a die;
- a clear crate (a 6 x 6 x 2 in stackable organizer bin, 152 x 152 x 51 mm) in front of the right arm.

The coworker reaches into the tray at 4 s and every 11 s after, eight times, closes on whatever is there and drops it in
the crate; if the tray is empty it closes on nothing and goes back empty-handed. It never waits and never reads the
scene. The arm under test must put the five table blocks on the tray one at a time, each after the previous one has been
taken and before the coworker comes back, and keep clear of the coworker.

Scoring (``score()``), from the end-state block poses plus one trial-long flag:
- the share of the FIVE blocks the tested arm must supply (block_2..block_6) that end inside the crate; the preloaded
  block_1 is never credited, so an arm that holds still scores 0.0 (the coworker crates block_1 only);
- 0.0 if the two arms ever came within ``TOUCH`` of each other (watched every control step, see ``_watch_arms``).
Possible scores: 0.0, 0.2, 0.4, 0.6, 0.8, 1.0.

Run it:
  inspect-robots run --embodiment yam_arena -E task=keep_a_station_stocked_for_a_timed_arm.py -E object_jitter=0 \\
    -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<see task.json>" --scorer task_score ...
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import arena_assets_fix  # noqa: E402, F401
import labkit  # noqa: E402
import coworker  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP

WOOD, WHITE, CLEAR = (0.80, 0.64, 0.42), (0.93, 0.93, 0.92), (0.80, 0.86, 0.90)

# -- layout ------------------------------------------------------------------------------------------------------
CUBE, CUBE_M = 0.0305, 0.016  # m, kg: 1.2 in wood cube; 16 g = estimate (28.4 cm3 x ~0.55 g/cm3, no listed mass)
TX, TY = 0.28, -0.04  # hand-off tray (station B) centre
TRAY_IN, TRAY_WALL, TRAY_FLOOR, TRAY_H = 0.083, 0.003, 0.002, 0.025  # weigh boat 89 x 89 x 25 mm outside
MX, MY = 0.30, 0.22  # centre of the die-five block layout (left arm's side)
DIE = 0.065  # m: corner offset of the die-five layout
CX, CY = 0.30, -0.30  # crate centre (in front of the right arm)
CRATE_IN_X, CRATE_IN_Y, CRATE_WALL, CRATE_H, CRATE_FLOOR = 0.146, 0.146, 0.003, 0.051, 0.003  # bin 152 x 152 x 51 mm

MAT_SPOTS = [(MX, MY), (MX - DIE, MY - DIE), (MX - DIE, MY + DIE), (MX + DIE, MY - DIE), (MX + DIE, MY + DIE)]


def _tray_boxes(inner_x, inner_y, wall, floor, height, colour):
    ox, oy = inner_x / 2 + wall / 2, inner_y / 2 + wall / 2
    h = height - floor
    return [((0.0, 0.0, floor / 2), (inner_x + 2 * wall, inner_y + 2 * wall, floor), colour),
            ((ox, 0.0, floor + h / 2), (wall, inner_y + 2 * wall, h), colour),
            ((-ox, 0.0, floor + h / 2), (wall, inner_y + 2 * wall, h), colour),
            ((0.0, oy, floor + h / 2), (inner_x, wall, h), colour),
            ((0.0, -oy, floor + h / 2), (inner_x, wall, h), colour)]


TRAY = labkit.part("station_b", (TX, TY, TOP), fixed=True, friction=0.8, color=WHITE,
                   boxes=_tray_boxes(TRAY_IN, TRAY_IN, TRAY_WALL, TRAY_FLOOR, TRAY_H, WHITE))
CRATE = labkit.part("crate", (CX, CY, TOP), fixed=True, friction=0.8, color=CLEAR,
                    boxes=_tray_boxes(CRATE_IN_X, CRATE_IN_Y, CRATE_WALL, CRATE_FLOOR, CRATE_H, CLEAR))

BLOCKS = ["block_1", "block_2", "block_3", "block_4", "block_5", "block_6"]
SUPPLIED = BLOCKS[1:]  # the five the tested arm must supply; block_1 starts on the tray and is never credited
_CUBES = [Box(name="block_1", pos=(TX, TY, TOP + TRAY_FLOOR + CUBE / 2), size=(CUBE,) * 3, mass=CUBE_M, color=WOOD)]
_CUBES += [Box(name=n, pos=(x, y, TOP + CUBE / 2), size=(CUBE,) * 3, mass=CUBE_M, color=WOOD)
           for n, (x, y) in zip(SUPPLIED, MAT_SPOTS)]
OBJECTS = [TRAY, CRATE, *_CUBES]

# -- scoring ------------------------------------------------------------------------------------------------------
TOUCH = 0.050  # m: two arm bodies (link4, link5, gripper, both finger tips) closer than this = the arms touched
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_ARM_BODIES = ("link4", "link5", "gripper", "tip_left", "tip_right")
_WATCH = {"touched": False, "min_gap": float("inf")}


def in_crate(state) -> bool:
    """Block centre inside the crate's inner walls and below its rim."""
    p = np.asarray(state.pos, float)
    return (abs(p[0] - CX) <= CRATE_IN_X / 2 and abs(p[1] - CY) <= CRATE_IN_Y / 2
            and TOP + CRATE_FLOOR <= p[2] <= TOP + CRATE_H)


def crated(objects) -> list:
    return [n for n in SUPPLIED if in_crate(objects[n])]


def score(objects) -> float:
    """Share of the five supplied blocks in the crate; 0.0 if the arms ever touched. block_1 never counts."""
    if _WATCH["touched"]:
        return 0.0
    return len(crated(objects)) / len(SUPPLIED)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    return float(sum(np.hypot(*(np.asarray(objects[n].pos, float)[:2] - (CX, CY))) for n in SUPPLIED
                     if not in_crate(objects[n])))


@labkit.on_reset
def _reset_watch(env) -> None:
    _WATCH["touched"], _WATCH["min_gap"] = False, float("inf")


@labkit.on_step
def _watch_arms(env) -> None:
    """Latch 'touched' once any watched body of one arm comes within TOUCH of one of the other arm."""
    base = env.unwrapped
    origin = base.scene.env_origins[0]
    pts = []
    for key in ("left_robot", "right_robot"):
        art = base.scene[key]
        names = list(art.body_names)
        raw = art.data.body_pos_w
        pos = (getattr(raw, "torch", raw)[0] - origin).detach().cpu().numpy().astype(float)
        pts.append(np.array([pos[names.index(b)] for b in _ARM_BODIES if b in names]))
    gap = float(np.min(np.linalg.norm(pts[0][:, None, :] - pts[1][None, :, :], axis=2)))
    _WATCH["min_gap"] = min(_WATCH["min_gap"], gap)
    if gap < TOUCH:
        _WATCH["touched"] = True


# -- the coworker's routine (fixed clock, open loop) ----------------------------------------------------------------
GRAB_T0, CYCLE_S, N_VISITS = 4.0, 11.0, 8  # it closes on the tray at 4 s, 15 s, ... 81 s
OPEN_TRAY = -0.030  # finger target: 60 mm pad gap, fits inside the 83 mm tray and around a 30.5 mm cube
TIP = PAD_TIP_PAST_TCP - 0.002  # m: TCP above the pad tips, straight-down grip
PINCH_TRAY = TRAY_FLOOR + 0.010  # pad-tip height over the table for a cube on the tray
PINCH_MAT = 0.008  # pad-tip height for a cube on the table
CW_WAIT = (0.12, -0.48, TOP + 0.20)  # coworker's wait pose: clear of the tray and of the top cameras' view of the crate
LT_WAIT = (0.20, 0.14, TOP + 0.18)  # tested arm's staging pose, clear of the tray and the coworker
# one drop spot per visit on the crate floor (8 of a 3 x 3 grid, 40 mm pitch), so the blocks lie side by side below the rim
DROPS = [(CX + dx, CY + dy) for dy in (-0.04, 0.0, 0.04) for dx in (-0.04, 0.0, 0.04)][:N_VISITS]


def _p(x, y, tip_z):
    return (x, y, TOP + tip_z + TIP)


def _timed(keys):
    """Waypoints from absolute-time keyframes [(t, pos, finger, label)]: each waypoint lasts until its keyframe time."""
    out, t_prev = [], 0.0
    for t, pos, finger, label in keys:
        out.append(Waypoint(max(0.05, t - t_prev), pos, finger, label))
        t_prev = t
    return out


def coworker_plan() -> list:
    """The right arm's routine: eight visits to the tray on a fixed clock, crating whatever it closes on."""
    keys = [(1.5, CW_WAIT, OPEN_TRAY, "coworker to its wait pose")]
    for k in range(N_VISITS):
        g = GRAB_T0 + k * CYCLE_S
        dx, dy = DROPS[k]
        keys += [(g - 1.6, _p(TX, TY, 0.12), OPEN_TRAY, f"coworker visit {k + 1}: above the tray"),
                 (g - 0.6, _p(TX, TY, PINCH_TRAY), OPEN_TRAY, f"coworker visit {k + 1}: down into the tray"),
                 (g, _p(TX, TY, PINCH_TRAY), FINGER_CLOSED, f"coworker visit {k + 1}: close on whatever is there"),
                 (g + 0.8, _p(TX, TY, 0.12), FINGER_CLOSED, f"coworker visit {k + 1}: lift out of the tray"),
                 (g + 2.4, _p(dx, dy, 0.11), FINGER_CLOSED, f"coworker visit {k + 1}: over the crate"),
                 (g + 3.0, _p(dx, dy, 0.08), FINGER_CLOSED, f"coworker visit {k + 1}: lower into the crate"),
                 (g + 3.6, _p(dx, dy, 0.08), OPEN_TRAY, f"coworker visit {k + 1}: let go"),
                 (g + 4.8, CW_WAIT, OPEN_TRAY, f"coworker visit {k + 1}: back to its wait pose")]
    return _timed(keys)


coworker.install(coworker_plan)

# -- scripted demo (left arm; geometry constants and the coworker's fixed clock only) -------------------------------
PLACE_AFTER, PICK_AFTER = 2.6, 6.4  # s after a coworker grab: start placing / start fetching the next block


def demo(t_wait=0.0):
    """Left arm fetches each table block, waits at its staging pose, and puts it on the tray right after the coworker has
    lifted the previous one away; the right arm runs the coworker's routine (the embodiment forces it anyway)."""
    o = OPEN_TRAY
    keys = []
    order = list(zip(SUPPLIED, MAT_SPOTS))
    for k, (name, (bx, by)) in enumerate(order):
        fetch = 0.4 if k == 0 else GRAB_T0 + (k - 1) * CYCLE_S + PICK_AFTER
        place = GRAB_T0 + k * CYCLE_S + PLACE_AFTER
        keys += [(fetch + 1.0, _p(bx, by, 0.10), o, f"above {name}"),
                 (fetch + 1.8, _p(bx, by, PINCH_MAT), o, f"straddle {name}"),
                 (fetch + 2.4, _p(bx, by, PINCH_MAT), FINGER_CLOSED, f"pinch {name}"),
                 (fetch + 3.2, _p(bx, by, 0.10), FINGER_CLOSED, f"lift {name}"),
                 (fetch + 4.4, LT_WAIT, FINGER_CLOSED, f"hold {name} at the staging pose"),
                 (place, LT_WAIT, FINGER_CLOSED, f"wait for the coworker to clear the tray ({name})"),
                 (place + 1.0, _p(TX, TY, 0.10), FINGER_CLOSED, f"carry {name} over the tray"),
                 (place + 1.8, _p(TX, TY, PINCH_TRAY + 0.002), FINGER_CLOSED, f"set {name} down in the tray"),
                 (place + 2.3, _p(TX, TY, PINCH_TRAY + 0.002), o, f"release {name}"),
                 (place + 2.9, _p(TX, TY, 0.10), o, f"rise out of the tray ({name})"),
                 (place + 3.6, LT_WAIT, o, f"back to the staging pose ({name})")]
    left = _timed(keys)
    right = coworker_plan()
    end = GRAB_T0 + len(SUPPLIED) * CYCLE_S + 3.6 + SUCCESS_HOLD_S + 2.0 + t_wait  # last block crated 3.6 s after its grab
    left.append(Waypoint(max(0.2, end - sum(w.duration for w in left)), LT_WAIT, FINGER_OPEN, "hold clear"))
    return {"left": left, "right": right}
