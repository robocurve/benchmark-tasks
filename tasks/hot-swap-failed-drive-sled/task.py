"""Yam-arena task file: Hot-swap a failed drive sled in a rack without disturbing healthy bays.

Story:
Datacenter maintenance:
Three drive bays in a rack. Middle bay has a red status lamp indicating drive failure.
Left arm depresses the retention latch on the rack top; right arm pulls the failed sled
clear of the chassis detent. The old sled is transferred to the discard pad on the left.
Right arm retrieves the blue-handled keyed spare from the spare stand on the right, aligns
and seats it into the bay. Once seated and latch is released, status lamp turns green.
Adjacent healthy sleds must not be disturbed (> 5 mm = fault).
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan
import geometry as g
import logic
import parts
from parts import Slider, env, gripper_open, on_reset, on_step, part, write_pose
from scoring import Scorer

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "One bay has a red lamp: hold its latch button down, pull its sled out, and slide the spare with the blue handle all the way in until the lamp turns green, without moving the other two sleds."

# 1. Rack fixture with 3 latch button sliders on top
LATCH_SLIDERS = []
for bay in ("bay_0", "bay_1", "bay_2"):
    y = g.BAY_Y[bay]
    lx, ly = g.latch_xy(bay)
    local_rest = (lx - g.RACK_POS[0], ly - g.RACK_POS[1], g.BUTTON_REST_Z - g.RACK_POS[2])
    slider = Slider(
        f"latch_{bay}",
        rest=local_rest,
        boxes=((local_rest, g.BUTTON_SIZE, g.BUTTON_LATCH),),
        travel=g.BUTTON_TRAVEL,
        rate=100.0,
        damping=5.0,
        mass=0.005,
        axis="Z",
    )
    LATCH_SLIDERS.append(slider)

def world_part(name, pos, boxes=(), cylinders=(), **kw):
    """Like part(), but the shapes are given at their WORLD positions: shift them into the body frame here."""
    px, py, pz = pos

    def shift(c):
        return (c[0] - px, c[1] - py, c[2] - pz)

    return part(name, pos, boxes=tuple((shift(c), s, col) for c, s, col in boxes),
                cylinders=tuple((shift(c), *rest) for c, *rest in cylinders), **kw)


RACK_FIXTURE = world_part(
    "rack_fixture",
    pos=g.RACK_POS,
    yaw=0.0,
    boxes=tuple(g.rack_world_boxes()),
    mass=10.0,
    sliders=tuple(LATCH_SLIDERS),
    color=g.RACK_GREY,
    friction=1.0,
)

# 2. Static fixtures
SPARE_STAND = world_part(
    "spare_stand",
    pos=g.STAND_POS,
    yaw=0.0,
    boxes=((g.STAND_POS, g.STAND_SIZE, g.DARK_GREY),),
    mass=4.0,
    friction=1.0,
)

DISCARD_PAD = world_part(
    "discard_pad",
    pos=g.DISCARD_PAD_POS,
    yaw=0.0,
    boxes=((g.DISCARD_PAD_POS, g.DISCARD_PAD_SIZE, g.PAD_RUBBER),),
    mass=2.0,
    friction=1.0,
)

# 3. Status lamps (markers)
STATUS_LAMPS = {}
for bay in ("bay_0", "bay_1", "bay_2"):
    color = g.LAMP_RED if bay == g.FAILED_BAY else g.LAMP_GREEN
    pos = (g.LAMP_X, g.BAY_Y[bay], g.LAMP_Z)
    STATUS_LAMPS[bay] = part(
        f"lamp_{bay}",
        pos=pos,
        mass=0.005,
        boxes=(((0.0, 0.0, 0.0), g.LAMP_SIZE, color),),
        collision=False,
    )

# The failed bay's lamp turns green: a separate green marker is parked below the table until then.
GREEN_LAMP_BAY_1 = part(
    "lamp_bay_1_green",
    pos=(g.LAMP_X, g.BAY_Y[g.FAILED_BAY], g.LAMP_OFF_Z),
    mass=0.005,
    boxes=(((0.0, 0.0, 0.0), g.LAMP_SIZE, g.LAMP_GREEN),),
    collision=False,
)

# 4. Three sleds inside rack bays
SLEDS = {}
for bay in ("bay_0", "bay_1", "bay_2"):
    y = g.BAY_Y[bay]
    sled_pos = (g.SLED_SEATED_X, y, g.SLED_REST_Z)
    handle_pos = (g.HANDLE_SEATED_X, y, g.SLED_REST_Z)
    handle_color = g.HANDLE_BLACK
    SLEDS[bay] = world_part(
        f"sled_{bay}",
        pos=sled_pos,
        yaw=0.0,
        boxes=(
            (sled_pos, g.SLED_SIZE, g.SLED_METAL),
            (handle_pos, g.HANDLE_SIZE, handle_color),
        ),
        mass=g.SLED_MASS,
        friction=1.5,
    )

# 5. Two spare sleds on spare stand
SPARES = {}
for color in ("blue", "orange"):
    sp_pos = g.SPARE_POS[color]
    h_pos = (sp_pos[0] + g.HANDLE_OFFSET_X, sp_pos[1], sp_pos[2])
    h_color = g.HANDLE_BLUE if color == "blue" else g.HANDLE_ORANGE
    SPARES[f"spare_{color}"] = world_part(
        f"spare_{color}",
        pos=sp_pos,
        yaw=0.0,
        boxes=(
            (sp_pos, g.SLED_SIZE, g.SLED_METAL),
            (h_pos, g.HANDLE_SIZE, h_color),
        ),
        mass=g.SLED_MASS,
        friction=1.5,
    )

OBJECTS = [
    RACK_FIXTURE,
    SPARE_STAND,
    DISCARD_PAD,
    *STATUS_LAMPS.values(),
    GREEN_LAMP_BAY_1,
    *SLEDS.values(),
    *SPARES.values(),
]

_SCORER = Scorer()
_last_states_ref = None
_last_result = None


def reset(_environment=None) -> None:
    global _last_states_ref, _last_result
    _last_states_ref = None
    _last_result = None
    _SCORER.reset()


on_reset(reset)


def _trace(row: dict) -> None:
    path = os.environ.get("TASK_TRACE")
    if path:
        with open(path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")


def _arm_info(e):
    if e is None:
        return None, None
    try:
        from parts import _to_numpy
        from yam_arena import runtime

        base = e.unwrapped
        origin = _to_numpy(base.scene.env_origins[0])
        tcps = {}
        vels = {}
        for arm in ("left", "right"):
            robot = base.scene[f"{arm}_robot"]
            tcps[arm] = (runtime.tcp_world(robot) - origin).tolist()
            lv = _to_numpy(robot.data.root_link_lin_vel_w)[0]
            av = _to_numpy(robot.data.root_link_ang_vel_w)[0]
            vels[arm] = (lv, av)
        return tcps, vels
    except Exception:
        return None, None


def _update(objects):
    global _last_states_ref, _last_result
    if objects is _last_states_ref and _last_result is not None:
        return _last_result

    e = env()
    rack = objects["rack_fixture"]
    latch_disps = {
        bay: float(rack.sliders.get(f"latch_{bay}", 0.0))
        for bay in ("bay_0", "bay_1", "bay_2")
    }

    sled_positions = {
        bay: objects[f"sled_{bay}"].pos
        for bay in ("bay_0", "bay_1", "bay_2")
    }
    spare_positions = {
        "blue": objects["spare_blue"].pos,
        "orange": objects["spare_orange"].pos,
    }

    left_open = gripper_open(e, "left") if e is not None else True
    right_open = gripper_open(e, "right") if e is not None else True

    arm_tcps, arm_vels = _arm_info(e)

    result = _SCORER.update(
        latch_disps=latch_disps,
        sled_positions=sled_positions,
        spare_positions=spare_positions,
        grippers_open=left_open and right_open,
        arm_tcps=arm_tcps,
        arm_velocities=arm_vels,
    )

    # Once the lamp turns green: show the green marker and park the red one below the table
    if e is not None and result.details.get("lamp_green"):
        write_pose(e, "lamp_bay_1_green", (g.LAMP_X, g.BAY_Y[g.FAILED_BAY], g.LAMP_ON_Z))
        write_pose(e, "lamp_bay_1", (g.LAMP_X, g.BAY_Y[g.FAILED_BAY], g.LAMP_OFF_Z))

    _trace({
        "score": result.score,
        "success": result.success,
        "left_open": left_open,
        "right_open": right_open,
        **result.details,
        "dbg_sled_x": [round(float(sled_positions[b][0]), 4) for b in ("bay_0", "bay_1", "bay_2")],
        "dbg_sled_z": round(float(sled_positions["bay_1"][2]), 4),
        "dbg_rack": [round(float(v), 4) for v in rack.pos],
        "dbg_sled_y": round(float(sled_positions["bay_1"][1]), 4),
        "dbg_latch": round(latch_disps["bay_1"], 4),
        "dbg_spare": {c: [round(float(v), 3) for v in spare_positions[c]] for c in spare_positions},
        "dbg_tcps": {a: [round(float(v), 3) for v in arm_tcps[a]] for a in (arm_tcps or {})},
    })
    _last_states_ref = objects
    _last_result = result
    return result


def success(objects) -> bool:
    return _update(objects).success


def score(objects) -> float:
    return _update(objects).score


def demo():
    return demo_plan.plans()


def plans():
    return demo_plan.plans()

