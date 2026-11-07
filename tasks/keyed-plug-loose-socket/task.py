"""On the station table: a loose grey block (85 x 110 x 60 mm, 0.30 kg) with a keyed socket on its face towards the
right arm, and a keyed plug lying on its side, key up, with its nose turned 45 degrees away from the socket. The socket
opens along -y; a spring plunger inside it clicks into a groove on the plug's nose when the plug is fully seated, and
it takes a firm push to get there, so the block slides away unless it is held. The plug is the small dark part with a
red key rib; the key only fits one way up.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 ...
One-arm check (the left arm idle):  DEMO_NO_BRACE=1 before the demo command.
"""

from __future__ import annotations

import json
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import demo_plan  # noqa: E402
import geometry as g  # noqa: E402
from parts import Slider, env, gripper_open, on_reset, part  # noqa: E402
from scoring import Scorer  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = "Plug the connector into the socket on the loose block, then let go of both. Do not let the block slide."

_centre, _cylinders = g.plunger_slider_spec()
PLUNGER = Slider(
    "plunger",
    rest=_centre,
    cylinders=_cylinders,
    travel=g.PLUNGER_TRAVEL,
    rate=g.PLUNGER_RATE,
    damping=g.PLUNGER_DAMPING,
    mass=g.PLUNGER_MASS,
    axis="X",
)

BLOCK = part(
    "block",
    pos=g.BLOCK_POS,
    yaw=g.BLOCK_YAW,
    boxes=g.block_boxes(),
    mass=g.BLOCK_MASS,
    sliders=(PLUNGER,),
    color=g.GREY,
    friction=g.BLOCK_FRICTION,
)
PLUG = part(
    "plug",
    pos=(*g.PLUG_XY0, g.PLUG_Z0),
    yaw=g.PLUG_YAW0,
    boxes=g.plug_boxes(),
    mass=g.PLUG_MASS,
    color=g.DARK,
    friction=g.PLUG_FRICTION,
)
OBJECTS = [BLOCK, PLUG]

_SCORER = Scorer()


@on_reset
def _reset_scorer(environment=None) -> None:
    _SCORER.reset()


def _trace(row: dict) -> None:
    """With TASK_TRACE=<file>, append one JSON line per scorer call: what the scorer saw and decided."""
    path = os.environ.get("TASK_TRACE")
    if path:
        with open(path, "a") as f:
            f.write(json.dumps(row, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")


def _arm_trace(e) -> dict:
    """Where each gripper's TCP and finger tips are (world frame, relative to the env origin), for the trace only: it
    never reaches the scorer, and a failure to read it is recorded instead of raised."""
    try:
        from parts import _to_numpy
        from yam_arena import runtime

        base = e.unwrapped
        origin = _to_numpy(base.scene.env_origins[0])
        out = {}
        for arm in ("left", "right"):
            robot = base.scene[f"{arm}_robot"]
            poses = runtime.link_poses(robot)
            out[f"{arm}_tcp"] = (runtime.tcp_world(robot) - origin).tolist()
            out[f"{arm}_gripper_x_axis"] = poses["gripper"][:3, 0].tolist()  # the gripper link's own x, y, z axes in the world
            out[f"{arm}_gripper_y_axis"] = poses["gripper"][:3, 1].tolist()
            out[f"{arm}_gripper_z_axis"] = poses["gripper"][:3, 2].tolist()
            names = list(robot.joint_names)
            q = _to_numpy(robot.data.joint_pos)[0]
            out[f"{arm}_finger_q"] = [float(q[names.index(j)]) for j in ("joint7", "joint8")]
            for tip in ("tip_left", "tip_right"):
                out[f"{arm}_{tip}"] = (poses[tip][:3, 3] - origin).tolist()
        return out
    except Exception as exc:  # noqa: BLE001
        return {"arm_trace_error": repr(exc)}


def _update(objects):
    plug, block = objects["plug"], objects["block"]
    e = env()
    left_open, right_open = gripper_open(e, "left"), gripper_open(e, "right")
    result = _SCORER.update(
        plug.pos,
        plug.quat_xyzw,
        plug.lin_vel,
        plug.ang_vel,
        block.pos,
        block.quat_xyzw,
        block.lin_vel,
        block.ang_vel,
        block.sliders["plunger"],
        left_open and right_open,
    )
    _trace(
        {
            "score": result.score,
            "success": result.success,
            "left_open": left_open,
            "right_open": right_open,
            "plunger": block.sliders["plunger"],
            "plug_pos": plug.pos,
            "block_pos": block.pos,
            **_arm_trace(e),
            **result.details,
        }
    )
    return result


def success(objects) -> bool:
    """Seated, latched, the block still within 5 mm of its start, both grippers open, everything still (for 1 s)."""
    return _update(objects).success


def score(objects) -> float:
    """0.25 plug lifted 3 cm and within 10 cm of the socket; 0.5 nose 6 mm in, or seated with the block shifted; 0.75
    seated with the block in place but still held or moving; 1.0 seated, released and still. 0 if the block ever moved
    30 mm or tilted 20 degrees, or the plug fell off the table."""
    return _update(objects).score


def demo():
    """Left arm grips the block; right arm carries the plug, turns it to the key, pushes it in; both let go.
    DEMO_HOLD_ONLY=1 returns no plan, so both arms hold the ready pose (a settling check)."""
    return {} if os.environ.get("DEMO_HOLD_ONLY") == "1" else demo_plan.plans()
