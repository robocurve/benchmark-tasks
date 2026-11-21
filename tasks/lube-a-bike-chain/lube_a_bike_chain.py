"""On the station table: a whole 14 inch child's bike (freewheel, open chain) stands across the table in two wheel
stands, its chain side toward the robot and its rear wheel toward the robot's left. A squeeze bottle of chain lube
stands on the table in front of the rear wheel.

The job: drip lube onto the chain while the other hand turns the near pedal backwards, then stand the bottle back.
The scorer counts the links with lube on them and checks how the place is left; see `scoring` in task.json.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))  # labkit and drivekit sit next to this file

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

import drivekit as dk  # noqa: E402
from labkit import Custom, Live, Pose, gripper_points, on_reset, ready_pose, tilt_deg  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]


def _k(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


# the point on the table under the cranks' axle: the bike's middle plane 42 cm in front of the robot
ORIGIN = np.array([_k("DRIVE_X", 0.42), _k("DRIVE_Y", 0.09), TOP])


def _start(spec) -> dk.DriveState:
    return dk.state(ORIGIN)


def _build(stage, prim_path, origin):
    dk.build(stage, prim_path)


def _read(environment, spec, prim_root) -> dk.DriveState:
    return dk.read(environment, ORIGIN, prim_root)


DRIVE = Custom("drivetrain", pos=tuple(float(v) for v in ORIGIN), size=dk.SIZE, color=(0.6, 0.6, 0.6),
               build=_build, reader=_read, starter=_start)

# the squeeze bottle of chain lube, standing to the left of the stand
BOTTLE_AT = np.array([_k("BOTTLE_X", 0.24), _k("BOTTLE_Y", 0.40), TOP])


def _bottle_start(spec) -> dk.BottleState:
    return dk.bottle_state(BOTTLE_AT)


def _bottle_build(stage, prim_path, origin):
    dk.build_bottle(stage, prim_path)


def _bottle_read(environment, spec, prim_root) -> dk.BottleState:
    return dk.read_bottle(environment, prim_root)


BOTTLE = Custom("lube_bottle", pos=tuple(float(v) for v in BOTTLE_AT), size=dk.BOTTLE_SIZE, color=dk.BOTTLE_GREEN,
                build=_bottle_build, reader=_bottle_read, starter=_bottle_start)

OBJECTS = [BOTTLE, DRIVE]  # the bottle is read first: the drivetrain's read then lands its drops


on_reset(dk.reset_turns)


# -- scoring ----------------------------------------------------------------------------------------------------------------
# What counts is the chain, link by link, and how the place is left. The chain has 56 links.
WET_1, WET_2, WET_3 = 14, 28, 50  # links with lube on them for 0.25, 0.5 and 0.75: a quarter, half, nine in ten
MAX_MISSED = 12  # drops that may miss the chain, about one in ten (lube on the tyre, the rim or the floor is the harm)
MAX_DROPS = 140  # drops in all: two and a half a link. Guides say one drop a roller; a flooded chain collects grit
CHAIN_ON = 0.005  # m a pin may sit off the chain's line and the chain still be on its teeth
UPRIGHT_DEG = 10.0  # the bottle put back: standing within this of upright
ON_TABLE = 0.01  # m: its base within this of the table's top
CLEAR = 0.05  # m from each gripper's TCP to the bottle and to the near pedal: let go
STILL_CRANK = 0.1  # rad/s


def wet_links(drive) -> int:
    return int(np.count_nonzero(np.asarray(drive.lubed) > 0))


def chain_on(drive) -> bool:
    """Every pin still on the chain's line: the chain has not been pushed off its teeth."""
    across = dk.to_bike(ORIGIN, drive.pins)[:, 1]
    return bool(np.all(np.abs(across - dk.CHAINLINE) <= CHAIN_ON))


def clean(drive) -> bool:
    return bool(drive.spills <= MAX_MISSED and drive.drops <= MAX_DROPS)


def bottle_put_back(bottle) -> bool:
    """Standing on the table, upright and still."""
    return bool(tilt_deg(bottle.quat_xyzw) <= UPRIGHT_DEG and abs(float(bottle.pos[2]) - TOP) <= ON_TABLE
                and bottle.resting(max_speed=0.01, max_spin=0.5))


def hands_clear(drive, bottle) -> bool:
    mid = np.asarray(bottle.pos, float) + (np.asarray(bottle.tip, float) - np.asarray(bottle.pos, float)) / 2
    for p in gripper_points().values():
        if np.linalg.norm(p - mid) <= CLEAR + dk.BOTTLE_H / 2 or np.linalg.norm(p - np.asarray(drive.pedal, float)) <= CLEAR:
            return False
    return True


def levels(objects) -> dict:
    drive, bottle = objects["drivetrain"], objects["lube_bottle"]
    return {"wet": wet_links(drive), "missed": int(drive.spills), "drops": int(drive.drops), "chain_on": chain_on(drive),
            "clean": clean(drive), "bottle_back": bottle_put_back(bottle), "hands_clear": hands_clear(drive, bottle),
            "cranks_still": abs(float(drive.crank_speed)) <= STILL_CRANK}


def score(objects) -> float:
    lv = levels(objects)
    wet = lv["wet"]
    if wet < WET_1:
        return 0.0
    base = 0.25 if wet < WET_2 else 0.5 if wet < WET_3 else 0.75
    if not (lv["clean"] and lv["chain_on"]):
        return min(base, 0.5) if wet >= WET_2 else 0.25  # lubed, but messily or with the chain knocked off: half at most
    if base == 0.75 and lv["bottle_back"] and lv["hands_clear"] and lv["cranks_still"]:
        return 1.0
    return base


def success(objects) -> bool:
    return score(objects) >= 1.0


# -- the scripted run. Both grasps are the ones a person drove by keyboard on this layout (20 Nov 2026): the left hand
# takes the bottle 3 cm above its base, pointing forward and a little to the right; the right hand comes down onto the
# near pedal pointing 58 degrees down, its fingers across the pedal's 5 cm. BIKE_STAGE: 0 the right hand only turns the
# cranks; 1 the left also picks the bottle up; 2 and tips it nozzle-down over the chain's top run; 3 and squeezes while
# the right hand turns the cranks backwards
STAGE = int(_k("BIKE_STAGE", 3))
TURNS = _k("DRIVE_TURNS", 3.5)  # 56 links pass in 2.15 turns of the 26-tooth chainring: three passes, since the bottle
# is lifted clear for part of every turn
STEP_DEG = 30.0
STEP_S = _k("DRIVE_STEP_S", 1.0)  # 30 degrees a second: 2.2 links a second pass the nozzle, a drop every 0.45 s
PEDAL_GRIP = -0.021  # finger target: pads 42 mm apart on the 50 mm pedal (a firm hold)
HOLD = -_k("LUBE_HOLD", 0.0134)  # finger target carrying the bottle: a firm 18 N a side, the wall in 2 mm, dry
SQUEEZE = -_k("LUBE_SQUEEZE", 0.0060)  # squeezing: the wall in about 4.5 mm a side
TIP_DEG = _k("LUBE_TIP_DEG", -145.0)  # the wrist's roll that tips the bottle: its nozzle toward the rear wheel and down
OVER_Y = _k("LUBE_OVER_Y", 0.20)  # the nozzle's place along the top run, behind the cranks' axle (toward the rear wheel)
OVER_H = _k("LUBE_OVER_H", 0.030)  # the nozzle's tip above the chain while it drips
CLEAR_H = _k("LUBE_CLEAR_H", 0.095)  # and while the near pedal passes under the left hand (lower, the two hands meet)
CLEAR_FROM, CLEAR_TO = _k("LUBE_CLEAR_FROM", 0.0), _k("LUBE_CLEAR_TO", 0.0)  # that part of each turn, degrees of crank
AIM = np.array([_k("LUBE_AIM_DX", 0.0), _k("LUBE_AIM_DY", 0.0), _k("LUBE_AIM_DZ", 0.0)])  # allowance for where the
# hand really lands with the tipped bottle
R_LEFT = np.array([[-0.41764, -0.15946, 0.89451], [-0.90686, 0.01201, -0.42127], [0.05643, -0.98713, -0.14963]])
R_RIGHT = np.array([[-0.04451, -0.85027, 0.52446], [-0.99863, 0.05233, 0.00008], [-0.02751, -0.52373, -0.85144]])
TURN_DEG = _k("LUBE_TURN_DEG", -25.0)  # before tipping, the hand is turned this much about the vertical (to point 50
# degrees right of straight ahead), so that hand and bottle stay on the rear wheel's side of the near pedal's sweep:
# pointing as it was driven, the hand lay across the pedal's path and the two hands met at 200 to 230 degrees of crank
R_TURNED = axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(TURN_DEG)) @ R_LEFT
R_TIPPED = axis_angle_matrix(R_TURNED[:, 2], math.radians(TIP_DEG)) @ R_TURNED
GRASP = np.array([0.24, 0.40, TOP + _k("LUBE_GRASP_Z", 0.045)]) - 0.004 * np.array([R_LEFT[0, 2], R_LEFT[1, 2], 0.0])  # the left TCP on the
# bottle standing at (0.24, 0.40): the bottle between the finger pads' middles. (The driven grasp had it 3 cm deeper in
# the hand; tipped over the chain, the fingertips then reached past the chain onto the bike's seat stay, which held
# the hand 6 cm too high, 20 Nov.)
TIP_IN_HAND = R_LEFT.T @ (np.array([0.24, 0.40, TOP + dk.BOTTLE_H]) - GRASP)  # the nozzle's tip in the hand's frame
PEDAL_HOLD = np.array([-0.007, 0.0, -0.034])  # the right TCP from the pedal's centre in that grip
CRANK_REST = math.radians(_k("DRIVE_REST_DEG", 6.9))  # where the cranks settle before anything touches them


def pedal_at(theta: float) -> np.ndarray:
    """The turned (robot's side) pedal's centre with the cranks turned ``theta`` (rad) backwards from the start."""
    return dk.to_world(ORIGIN, [-dk.CRANK * math.sin(theta), dk.PEDAL_Y, -dk.CRANK * math.cos(theta)])


LIVE_S = 3.0  # the left hand's time to put the nozzle right before the cranks start to turn
AIM_GAIN = _k("LUBE_AIM_GAIN", 0.7)  # how much of the nozzle's error the hand takes out in one step


def _in_clear_part(deg: float) -> bool:
    return CLEAR_FROM <= deg % 360.0 < CLEAR_TO


def _drip_steps(over: np.ndarray, tip_goal: np.ndarray, n: int) -> list:
    """(left) The dripping, one step for each of the other hand's steps round the cranks, each planned as the arm
    reaches it from where the nozzle really is (the bottle shifts a little in the fingers as it is tipped and
    squeezed): the hand is moved most of the way from there to where the nozzle should be. Down at the chain it
    squeezes; for the part of every turn where the near pedal passes under this hand it eases off and holds clear."""
    hand = {"at": np.array(over, float)}  # where the hand was last sent
    low_goal = tip_goal - np.array([0.0, 0.0, CLEAR_H - OVER_H])

    def step(goal, finger, label, duration, by_crank=False):
        def plan(objects):
            nonlocal goal, finger, label
            drive = objects.get("drivetrain")
            if by_crank and drive is not None and getattr(drive, "crank", None) is not None:
                # where the cranks really are, not where the clock says: after a slow patch the pedal may be late
                deg = math.degrees(float(drive.crank))
                if any(_in_clear_part(deg + a) for a in (0.0, STEP_DEG, 1.5 * STEP_DEG)):
                    goal, finger, label = tip_goal, HOLD, f"held clear {deg:.0f}"
                elif _in_clear_part(deg - 0.5 * STEP_DEG):
                    goal, finger, label = low_goal, HOLD, f"back down {deg:.0f}"
                else:
                    goal, finger, label = low_goal, SQUEEZE, f"squeeze {deg:.0f}"
            bottle = objects.get("lube_bottle")
            if bottle is not None:  # offline there is no bottle to read: the hand stays where the plan put it
                err = goal - np.asarray(bottle.tip, float)
                err = err * min(1.0, 0.08 / max(1e-9, float(np.linalg.norm(err))))  # never more than 8 cm in a step
                hand["at"] = hand["at"] + AIM_GAIN * err
            else:
                hand["at"] = np.array(over, float) + (goal - tip_goal)
            return [Pose(duration, tuple(hand["at"]), finger, label, rot=R_TIPPED)]

        return Live(duration, plan, label)

    out = [step(tip_goal, HOLD, "nozzle put right over the chain", LIVE_S / 3) for _ in range(2)]
    out.append(step(low_goal, HOLD, "down to the chain", LIVE_S / 3))
    for i in range(1, n + 1):
        deg = math.degrees(CRANK_REST) + i * STEP_DEG  # where this step takes the cranks
        if _in_clear_part(deg) or _in_clear_part(deg + STEP_DEG):  # the pedal is, or is about to be, under this hand
            out.append(step(tip_goal, HOLD, f"held clear {deg:.0f}", STEP_S, by_crank=True))
        elif _in_clear_part(deg - STEP_DEG):  # just past: back down, and squeeze from the next step
            out.append(step(low_goal, HOLD, f"back down {deg:.0f}", STEP_S, by_crank=True))
        else:
            out.append(step(low_goal, SQUEEZE, f"squeeze {deg:.0f}", STEP_S, by_crank=True))
    out.append(step(tip_goal, HOLD, "ease off, up", 1.0))
    return out


def demo():
    off = BOTTLE_AT - np.array([0.24, 0.40, TOP])  # the driven poses were for a bottle at (0.24, 0.40): shift with it

    def at(x, y, z):  # z above the table
        return tuple(np.array([x, y, TOP + z]) + off)

    # the left hand
    left = [ready_pose("left", 2.0, "let the chain settle")]
    grasp = tuple(GRASP + off)
    chain_top = dk.to_world(ORIGIN, [OVER_Y / math.cos(dk.TILT), dk.CHAINLINE, 0.0])  # on the axles' line, under the nozzle
    f = OVER_Y / (dk.CENTRES * math.cos(dk.TILT))
    chain_top[2] += dk.R_RING + f * (dk.R_COG - dk.R_RING)  # up to the top run
    tip_goal = chain_top + np.array([0.0, 0.0, CLEAR_H])  # where the nozzle should be, held clear of the pedal's hand
    over = tuple(tip_goal + AIM - R_TIPPED @ TIP_IN_HAND)
    roll_at = (0.27, 0.36, TOP + 0.30)  # clear of the bike: the bottle swings 9 cm round the hand as it tips
    n_roll = 5
    if STAGE >= 1:
        left += [Pose(2.5, at(0.201, 0.411, 0.072), FINGER_OPEN, "low, pointing at the bottle", rot=R_LEFT),
                 Pose(2.0, at(0.222, 0.397, 0.033), FINGER_OPEN, "down to the table in front of it", rot=R_LEFT),
                 Pose(1.5, grasp, FINGER_OPEN, "forward round the bottle", rot=R_LEFT),
                 Pose(1.0, grasp, HOLD, "take it lightly", rot=R_LEFT),
                 Pose(2.5, at(0.266, 0.38, 0.256), HOLD, "lift it", rot=R_LEFT)]
    if STAGE >= 2:
        left.append(Pose(1.5, roll_at, HOLD, "clear of the bike", rot=R_LEFT))
        left += [Pose(0.8, roll_at, HOLD, f"turn the hand {k}/2",
                      rot=axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(TURN_DEG * k / 2)) @ R_LEFT) for k in (1, 2)]
        left += [Pose(0.6, roll_at, HOLD, f"tip it over {k}/{n_roll}",
                      rot=axis_angle_matrix(R_TURNED[:, 2], math.radians(TIP_DEG * k / n_roll)) @ R_TURNED)
                 for k in range(1, n_roll + 1)]
        left += [Pose(2.5, (over[0] - 0.04, over[1], over[2] + 0.03), HOLD, "toward the chain", rot=R_TIPPED),
                 Pose(1.5, over, HOLD, "nozzle over the top run, held clear", rot=R_TIPPED),
                 Pose(1.0, over, HOLD, "let it come to rest", rot=R_TIPPED)]
    t_squeeze = sum(w.duration for w in left)

    # the right hand: take the near pedal, wait for the bottle, then turn the cranks backwards
    p0 = pedal_at(CRANK_REST) + PEDAL_HOLD
    right = [ready_pose("right", 2.0, "let the chain settle"),
             Pose(2.5, (0.354, -0.055, TOP + 0.264), FINGER_OPEN, "toward the bike", rot=R_RIGHT),
             Pose(2.0, tuple(p0 + np.array([0.0, 0.0, 0.07])), FINGER_OPEN, "over the pedal", rot=R_RIGHT),
             Pose(1.5, tuple(p0), FINGER_OPEN, "down onto the pedal", rot=R_RIGHT),
             Pose(1.0, tuple(p0), PEDAL_GRIP, "take the pedal", rot=R_RIGHT)]
    wait = t_squeeze + LIVE_S - sum(w.duration for w in right)
    if wait > 0:
        right.append(Pose(wait, tuple(p0), PEDAL_GRIP, "wait for the bottle", rot=R_RIGHT))
    n = int(round(TURNS * 360 / STEP_DEG))
    def crank_now(objects, planned: float) -> float:
        drive = objects.get("drivetrain")
        return planned if drive is None or getattr(drive, "crank", None) is None else float(drive.crank)

    def turn(i):
        def plan(objects):  # 30 degrees on from where the cranks really are: the hand never runs ahead of the pedal
            th = crank_now(objects, CRANK_REST + math.radians((i - 1) * STEP_DEG)) + math.radians(1.3 * STEP_DEG)
            return [Pose(STEP_S, tuple(pedal_at(th) + PEDAL_HOLD), PEDAL_GRIP, f"turn back {i * STEP_DEG:.0f}", rot=R_RIGHT)]

        return Live(STEP_S, plan, f"turn back {i * STEP_DEG:.0f}")

    def leave(objects):
        end = pedal_at(crank_now(objects, CRANK_REST + math.radians(n * STEP_DEG))) + PEDAL_HOLD
        return [Pose(1.0, tuple(end), FINGER_OPEN, "let go", rot=R_RIGHT),
                Pose(1.5, tuple(end + np.array([-0.03, 0.0, 0.08])), FINGER_OPEN, "clear", rot=R_RIGHT)]

    right += [turn(i) for i in range(1, n + 1)]
    right += [Live(2.5, leave, "let go and clear"), ready_pose("right", 2.5)]

    if STAGE >= 3:
        left += _drip_steps(np.array(over), tip_goal, n)
    if STAGE >= 2:
        left.append(Pose(2.5, roll_at, HOLD, "away from the bike", rot=R_TIPPED))
        left += [Pose(0.6, roll_at, HOLD, f"stand it up {k}/{n_roll}",
                      rot=axis_angle_matrix(R_TURNED[:, 2], math.radians(TIP_DEG * k / n_roll)) @ R_TURNED)
                 for k in range(n_roll - 1, -1, -1)]
        left += [Pose(0.8, roll_at, HOLD, f"turn the hand back {k}/2",
                      rot=axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(TURN_DEG * k / 2)) @ R_LEFT) for k in (1, 0)]
    if STAGE >= 1:
        down = tuple(np.array(grasp) + np.array([0.0, 0.0, 0.004]))
        left += [Pose(2.0, at(0.266, 0.38, 0.256), HOLD, "back over its place", rot=R_LEFT),
                 Pose(2.5, down, HOLD, "set it down", rot=R_LEFT),
                 Pose(0.8, down, FINGER_OPEN, "let go", rot=R_LEFT),
                 Pose(1.5, at(0.222, 0.397, 0.037), FINGER_OPEN, "back off it", rot=R_LEFT),
                 Pose(2.0, at(0.201, 0.411, 0.11), FINGER_OPEN, "clear", rot=R_LEFT),
                 ready_pose("left", 2.0)]
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in right)
    if tl < tr:
        left.append(ready_pose("left", tr - tl, "idle"))
    elif tr < tl:
        right.append(ready_pose("right", tl - tr, "idle"))
    return {"left": left, "right": right}
