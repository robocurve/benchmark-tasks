"""slide-flush-cutting-board-to-edge-and-rack -- slide a flush cutting board to the counter edge, lift it level and rack it.

A 457 x 305 x 12.7 mm polyethylene cutting board lies flush on a maple butcher block. Its only graspable
purchase is an edge pushed out into free air: the left arm slides it toward the block's -Y counter edge into a
safe overhang window (stopping short of the 50 % balance point, or the board tips off), the right arm pinches
the overhanging edge from below in free air and pulls it low while the rear edge stays supported, the left arm
pinches the now-exposed -X edge, both hands lift level together so the load splits between them, and they seat
the board flat on the floor inside a full-size dishwashing rack.
"""
from __future__ import annotations

import math
import pathlib
import sys

_HERE = str(pathlib.Path(__file__).resolve().parent)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import numpy as np

import arena_assets_fix  # noqa: F401,E402
import labkit  # noqa: E402
import pitch_demo  # noqa: E402
from labkit import PartState, on_reset, on_step, part  # noqa: E402,F401

SCENE = "station"
CHECK = SCENE

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_CLOSED, FINGER_OPEN  # noqa: E402

INSTRUCTION = (
    "Slide the cutting board from the butcher block to the counter-edge overhang, lift it level from below, "
    "and place it flat inside the dishwashing rack."
)

TOP = station.TABLE_TOP

# -- the butcher block (20 x 15 x 2-1/4 in maple prep block, fixed) ---------------------------------------
BW_BLK, BD_BLK, BH_BLK = 0.508, 0.381, 0.0572
BLK_CX, BLK_CY = 0.320, 0.3405          # x in [0.066, 0.574], y in [+0.150, +0.531]
BLK_TOP = TOP + BH_BLK                  # 0.8047
BLK_CLIFF_Y = BLK_CY - BD_BLK / 2.0     # +0.150: the -Y counter edge the board slides over
BLK_FRICTION = 0.20                     # oiled maple, cited Engineering ToolBox band

# -- the cutting board (18 x 12 x 1/2 in white polyethylene board, loose) ---------------------------------
BW_BRD, BD_BRD, BH_BRD = 0.4572, 0.3048, 0.0127
BRD_X0, BRD_Y0 = 0.320, 0.3224          # flush on the block: x in [0.091, 0.549], y in [+0.170, +0.475]
BRD_MASS, BRD_FRICTION = 1.47, 0.20     # HDPE on maple, cited mu = 0.20

# -- the dishwashing rack (19-3/4 x 19-3/4 x 4 in full-size base rack, fixed) -----------------------------
RK_W, RK_H, RK_FLR, RK_WALL = 0.502, 0.1016, 0.019, 0.006
RK_CX, RK_CY = 0.635, -0.395            # outer x in [0.384, 0.886], y in [-0.646, -0.144]
RK_FRICTION = 0.40                      # polypropylene rack
RK_X_MIN = RK_CX - RK_W / 2.0           # 0.384: outer +X wall face (arm approach must stay clear)
RK_IN_X0 = RK_CX - RK_W / 2.0 + RK_WALL  # 0.390
RK_IN_X1 = RK_CX + RK_W / 2.0 - RK_WALL  # 0.880
RK_IN_Y0 = RK_CY - RK_W / 2.0 + RK_WALL  # -0.640
RK_IN_Y1 = RK_CY + RK_W / 2.0 - RK_WALL  # -0.150

# -- scoring constants -----------------------------------------------------------------------------------
PASS_SCORE = 1.0
S_PARTIAL = 0.5
HAND_CLEAR = 0.040                      # both grippers >= 40 mm clear of the board
TILT_FLAT = 6.0                         # flat inside the rack
TILT_TIP = 15.0                         # tipped over / spilled
TILT_WINDOW = 10.0                      # upright limit for the overhang-window partial
WINDOW_PASS = BLK_CLIFF_Y - 0.030       # 0.120: the -Y edge must be > 30 mm past the cliff
BLK_X0, BLK_X1 = BLK_CX - BW_BLK / 2.0, BLK_CX + BW_BLK / 2.0  # 0.066 / 0.574: block x-span
WINDOW_X_SLACK = 0.050                  # overhang partial keeps the board over the block's x-span +/-50 mm
DROP_Z = TOP + 0.012                    # 0.7595: underside this low and outside the rack = on the table
SEAT_LO = TOP + RK_FLR - 0.008          # 0.7585: underside lowest allowed for 'seated on the rack floor'
SEAT_HI = TOP + RK_FLR + 0.012          # 0.7785: underside highest allowed for 'seated on the rack floor'
REAL_COST_USD = 198.93                  # board $6.99 + butcher block $170.95 + rack $20.99

MAPLE = (0.78, 0.62, 0.42)
HDPE_WHITE = (0.94, 0.94, 0.92)
RACK_GREY = (0.45, 0.47, 0.50)

# -- objects ----------------------------------------------------------------------------------------------
BUTCHER_BLOCK = part("butcher_block", (BLK_CX, BLK_CY, TOP), fixed=True, friction=BLK_FRICTION,
                     boxes=(((0, 0, BH_BLK / 2), (BW_BLK, BD_BLK, BH_BLK), MAPLE),))
BOARD = part("board", (BRD_X0, BRD_Y0, BLK_TOP), mass=BRD_MASS, friction=BRD_FRICTION,
             boxes=(((0, 0, BH_BRD / 2), (BW_BRD, BD_BRD, BH_BRD), HDPE_WHITE),))
_hi, _wm = RK_W / 2 - RK_WALL / 2, RK_H / 2
DISH_RACK = part("dish_rack", (RK_CX, RK_CY, TOP), fixed=True, friction=RK_FRICTION,
                 boxes=(
                     ((0, 0, RK_FLR / 2), (RK_W, RK_W, RK_FLR), RACK_GREY),
                     ((-_hi, 0, _wm), (RK_WALL, RK_W, RK_H), RACK_GREY),
                     ((+_hi, 0, _wm), (RK_WALL, RK_W, RK_H), RACK_GREY),
                     ((0, -_hi, _wm), (RK_W - 2 * RK_WALL, RK_WALL, RK_H), RACK_GREY),
                     ((0, +_hi, _wm), (RK_W - 2 * RK_WALL, RK_WALL, RK_H), RACK_GREY),
                 ))
OBJECTS = (BUTCHER_BLOCK, BOARD, DISH_RACK)


# -- trial state ------------------------------------------------------------------------------------------
_trial: dict = {"tcps": None, "step": -1}
_ids: dict = {}


def _set_test_trial(*, tcps=None) -> None:
    """Explicit hook for offline scoring tests (never called by demo())."""
    if tcps is not None:
        _trial["tcps"] = tuple(np.asarray(p, float) for p in tcps)


@on_reset
def _reset(_env) -> None:
    _trial.update(tcps=None, step=-1)
    _ids.clear()


def _arm_tcps(env):
    """Both TCPs from arm joint positions (same FK the official action layer uses)."""
    try:
        base = env.unwrapped
        out = []
        for side in ("left", "right"):
            asset = base.scene[f"{side}_robot"]
            key = f"arm_{side}"
            if key not in _ids:
                _ids[key] = asset.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = labkit._np(asset.data.joint_pos)[0, _ids[key]]
            tcp_p, _ = ik.tcp_pose(q, station.BASES[side])
            out.append(np.asarray(tcp_p, float))
        return tuple(out)
    except Exception:
        return None


@on_step
def _watch(env) -> None:
    _trial["step"] += 1
    _trial["tcps"] = _arm_tcps(env)


# -- scoring helpers --------------------------------------------------------------------------------------
def _rotm(state) -> np.ndarray:
    """Rotation matrix from a state quat (works on plain ObjectState and labkit PartState alike)."""
    x, y, z, w = state.quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _tilt_deg(state) -> float:
    return float(np.degrees(np.arccos(np.clip(_rotm(state)[2, 2], -1.0, 1.0))))


def _to_local(state, world) -> np.ndarray:
    return _rotm(state).T @ (np.asarray(world, float) - state.pos)


def _grippers_clear(board: PartState, tcps) -> bool:
    """Both TCPs outside the board's world box (+ HAND_CLEAR)."""
    if tcps is None or len(tcps) < 2:
        return False
    lo = np.asarray(board.low, float) - HAND_CLEAR
    hi = np.asarray(board.high, float) + HAND_CLEAR
    for t in tcps:
        t = np.asarray(t, float)
        if np.all(t >= lo) and np.all(t <= hi):
            return False
    return True


def components(objects: dict[str, PartState]) -> dict:
    b = objects["board"]
    tilt = _tilt_deg(b)
    in_xy = (float(b.low[0]) >= RK_IN_X0 and float(b.high[0]) <= RK_IN_X1
             and float(b.low[1]) >= RK_IN_Y0 and float(b.high[1]) <= RK_IN_Y1)
    return {
        "tilt_deg": tilt,
        "past_window": float(b.low[1]) < WINDOW_PASS,
        "in_rack_xy": in_xy,
        "seated_z": SEAT_LO <= float(b.low[2]) <= SEAT_HI,
        "dropped_low": float(b.low[2]) < DROP_Z,
        "resting": bool(b.resting()),
        "grippers_clear": _grippers_clear(b, _trial["tcps"]),
    }


def score(objects: dict[str, PartState]) -> float:
    """Everything measurable with a tape measure / level / overhead photo -- no force sensing.

    0.0: board tipped past 15 deg; or dropped on the table outside the rack (underside below table + 12 mm);
         or inside the rack but not seated flat (tilted past 6 deg, or riding the walls, or still moving);
         or tilted past 10 deg anywhere; or still flush on the block (-Y edge under 30 mm past the cliff);
         or either gripper within 40 mm of the board.
    0.5: board upright within 10 deg with its -Y edge more than 30 mm past the block cliff (or lifted
         clear), both grippers at least 40 mm clear, not yet seated flat inside the rack.
    1.0: board completely inside the rack inner cavity, flat (tilt <= 6 deg) with its underside on the
         rack floor (within -8/+12 mm of the floor top), at rest, both grippers >= 40 mm clear.
    """
    c = components(objects)
    if c["tilt_deg"] > TILT_TIP or not c["grippers_clear"]:
        return 0.0
    if c["in_rack_xy"]:
        if c["tilt_deg"] <= TILT_FLAT and c["seated_z"] and c["resting"]:
            return PASS_SCORE
        return 0.0
    b = objects["board"]
    windowed = (c["past_window"] and float(b.low[2]) <= BLK_TOP + 0.006
                and BLK_X0 - WINDOW_X_SLACK <= float(b.low[0])
                and float(b.high[0]) <= BLK_X1 + WINDOW_X_SLACK)
    lifted = float(b.low[2]) > BLK_TOP + 0.006
    if c["dropped_low"] or c["tilt_deg"] > TILT_WINDOW or not (windowed or lifted):
        return 0.0
    return S_PARTIAL


def success(objects: dict[str, PartState]) -> bool:
    return score(objects) >= PASS_SCORE


def distance(objects: dict[str, PartState]) -> float:
    """Horizontal distance of the board centre from the rack cavity centre plus the height left to drop."""
    b = objects["board"]
    return float(np.linalg.norm(np.asarray(b.pos[:2], float) - np.array([RK_CX, RK_CY]))
                 + max(0.0, float(b.low[2]) - (TOP + RK_FLR)))


# -- scripted demo -----------------------------------------------------------------------------------------
def demo():
    """The diagonal opposing-edge sequence: the left arm slides the flush board to the safe overhang window,
    the right arm pinches the exposed -Y edge in free air and pulls it low while the rear stays on the block,
    the left arm pinches the -X edge, both lift level to z = 0.925 with matched displacement (opposing grips
    bracket the board's centre of mass and split the 1.47 kg load, ~1.0 N m worst per gripper -- under the
    1.2 N m budget), seat it flat inside the rack, and a final settle check hovers a closed hand over the
    seated board before both arms park outboard. Waypoints are constants -- demo() reads no simulator state.
    """
    W = pitch_demo.Waypoint
    yr1, yr2, yr3 = math.radians(90), math.radians(45), math.radians(40)
    yl0, yl1, yl2, yl3 = 0.0, math.radians(15), math.radians(-25), math.radians(-40)
    prl, prr, rr = math.radians(80), math.radians(85), math.pi / 2
    left = [
        W(0.7, (0.32, +0.505, 0.855), FINGER_CLOSED, "over the board rear edge"),
        W(0.4, (0.32, +0.505, 0.802), FINGER_CLOSED, "down to push height"),
        W(1.5, (0.32, +0.340, 0.802), FINGER_CLOSED, "slide to the overhang window"),
        W(0.5, (0.32, +0.380, 0.895), FINGER_OPEN, "release and lift"),
        W(1.2, (0.30, +0.36, 0.90), FINGER_OPEN, "rotate to side grip", yl0, prl, rr),
        W(2.0, (0.15, +0.10, 0.90), FINGER_OPEN, "in front of the cliff", yl0, prl, rr),
        W(1.3, (0.05, +0.095, 0.822), FINGER_OPEN, "down beside the -X edge", yl0, prl, rr),
        W(1.5, (0.18, +0.095, 0.822), FINGER_OPEN, "straddle the -X edge", yl0, prl, rr),
        W(0.4, (0.18, +0.095, 0.822), FINGER_OPEN, "jaw to slab-normal", yl0, prr, rr),
        W(0.5, (0.18, +0.095, 0.822), FINGER_CLOSED, "pinch the -X edge", yl0, prr, rr),
        W(0.8, (0.195, +0.090, 0.925), FINGER_CLOSED, "lift level together", yl0, prr, rr),
        W(1.1, (0.30, -0.05, 0.925), FINGER_CLOSED, "carry over the rack", yl0, prr, rr),
        W(1.1, (0.42, -0.295, 0.925), FINGER_CLOSED, "carry over the rack", yl2, prr, rr),
        W(0.7, (0.45, -0.320, 0.872), FINGER_CLOSED, "diagonal lower to the rack floor", yl3, prr, rr),
        W(0.4, (0.45, -0.320, 0.872), FINGER_OPEN, "release the -X edge", yl3, prr, rr),
        W(0.6, (0.42, -0.210, 0.945), FINGER_OPEN, "diagonal lift clear", yl3, prr, rr),
        W(0.8, (0.30, -0.05, 0.94), FINGER_OPEN, "derotate the wrist", yl3, math.radians(50), math.radians(60)),
        W(0.8, (0.24, 0.18, 0.93), FINGER_OPEN, "derotate the wrist", yl1, math.radians(25), math.radians(30)),
        W(0.8, (0.200, +0.340, 0.920), FINGER_OPEN, "park outboard"),
    ]
    right = [
        W(3.1, (0.35, -0.025, 0.900), FINGER_OPEN, "approach in free air, jaw vertical", yr1, prr, rr),
        W(1.0, (0.35, -0.025, 0.814), FINGER_OPEN, "down to pinch height", yr1, prr, rr),
        W(0.9, (0.35, +0.105, 0.814), FINGER_OPEN, "under the overhanging -Y edge", yr1, prr, rr),
        W(0.6, (0.35, +0.105, 0.814), FINGER_CLOSED, "pinch the -Y edge", yr1, prr, rr),
        W(1.6, (0.35, -0.095, 0.818), FINGER_CLOSED, "pull low, rear still on the block", yr1, prr, rr),
        W(2.8, (0.35, -0.095, 0.818), FINGER_CLOSED, "hold for the left pinch", yr1, prr, rr),
        W(0.8, (0.35, -0.095, 0.925), FINGER_CLOSED, "lift level together", yr1, prr, rr),
        W(2.2, (0.600, -0.420, 0.925), FINGER_CLOSED, "carry over the rack", yr2, prr, rr),
        W(0.7, (0.638, -0.505, 0.872), FINGER_CLOSED, "diagonal lower to the rack floor", yr3, prr, rr),
        W(0.4, (0.638, -0.505, 0.872), FINGER_OPEN, "release the -Y edge", yr3, prr, rr),
        W(0.8, (0.600, -0.420, 0.945), FINGER_OPEN, "diagonal lift out", yr2, prr, rr),
        W(0.8, (0.648, -0.470, 0.808), FINGER_OPEN, "diagonal settle hover", yr2, prr, 0.0),
        W(0.5, (0.648, -0.470, 0.808), FINGER_CLOSED, "settle check", yr2, prr, 0.0),
        W(0.4, (0.648, -0.470, 0.808), FINGER_OPEN, "release", yr2, prr, 0.0),
        W(0.8, (0.600, -0.420, 0.945), FINGER_OPEN, "diagonal lift clear", yr2, prr, 0.0),
        W(0.8, (0.50, -0.40, 0.96), FINGER_OPEN, "derotate the wrist", yr2, math.radians(60), math.radians(60)),
        W(0.8, (0.38, -0.38, 0.95), FINGER_OPEN, "derotate the wrist", yr2, math.radians(30), math.radians(30)),
        W(0.8, (0.100, -0.520, 0.860), FINGER_OPEN, "park outboard"),
    ]
    return {"left": left, "right": right}
