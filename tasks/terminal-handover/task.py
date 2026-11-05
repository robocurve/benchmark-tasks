"""On the station table, in front of the left arm: one DIN-rail terminal block (grey polyamide, 77.2 mm long,
49.8 mm high, 12.1 mm thick, 38 g) lying flat on its side, as if it had dropped. Its latch foot, which clips onto
the rail, runs along its bottom edge; that edge faces the arms. Nothing else is on the table.

Goal: pick it up, hand it over in the air to the right arm, and have the right arm alone hold it by its two narrow
ends, upright with the latch foot down, ready to be clipped onto a rail, without it touching the table again once it
has been lifted.
"""

import os
from pathlib import Path

import numpy as np

from yam_arena.task_file import TABLE_TOP, Asset

SCENE = "station"
TOP = TABLE_TOP[SCENE]
HERE = Path(__file__).resolve().parent

# -- the terminal block ----------------------------------------------------------------------------------------

TERMINAL_ASSET = "terminal_block_flat"
TERMINAL_USD = HERE / "assets" / "terminal_flat.usd"  # built by tools/make_terminal_usd.py from the STEP model
TERMINAL_MASS = 0.0379  # kg
LENGTH, HEIGHT, THICKNESS = 0.0772, 0.0498, 0.0121  # m
START_XY = (0.25, 0.18)
START_YAW = np.pi  # the latch foot's edge faces the arms (-x)
UP_IN_MODEL = np.array([-1.0, 0.0, 0.0])  # foot -> top edge, in the model's own frame


def _register_terminal_asset() -> None:
    """Make the terminal USD known to Arena's asset registry, where yam_arena's Asset looks models up.

    The registry is plain Python (no Isaac Sim import), and yam_arena reads only ``usd_path`` from the entry.
    """
    from isaaclab_arena.assets.registries import AssetRegistry

    class TerminalBlockFlat:
        name = TERMINAL_ASSET
        tags = ["object"]
        usd_path = str(TERMINAL_USD)

    if not AssetRegistry().is_registered(TERMINAL_ASSET, ensure_loaded=False):
        AssetRegistry().register(TerminalBlockFlat, TERMINAL_ASSET)


_register_terminal_asset()

OBJECTS = [
    Asset(
        "terminal",
        asset=TERMINAL_ASSET,
        xy=START_XY,
        extent=LENGTH,  # its longest side: keeps the model at scale 1
        mass=TERMINAL_MASS,
        color=(0.62, 0.63, 0.62),
        yaw=float(START_YAW),
    )
]

# -- scoring --------------------------------------------------------------------------------------------------
#
# Besides the terminal, scoring looks at both grippers: their tool centre point (between the finger pads, 13 mm
# short of the tips) and how far the fingers are apart. yam_arena hands a task file only its own objects, so the
# grippers are added to that dict under GRIPPERS (see _add_grippers).

LIFTED = 0.05  # m: bottom of the terminal above the table
TOUCHING = 0.004  # m: bottom of the terminal this close to the table counts as set down
UPRIGHT_DEG = 15.0  # top edge within this angle of vertical, foot down
HOLD_REACH = 0.03  # m: a holding gripper's centre point is at most this far from the terminal
HOLD_GAP = (0.003, 0.085)  # m: a holding gripper's fingers are closed onto something, not shut and not wide open
END_GRIP_GAP = (0.070, 0.085)  # m: pads this far apart hold the terminal by its two narrow ends (77.2 mm apart)
RELEASED = 0.08  # m: a released gripper's centre point is at least this far from the terminal
STILL_MM, STILL_STEPS = 5.0, 10  # still: the centre stays within 5 mm over the last 10 control steps (0.5 s at 20 Hz)
SCORE_LIFTED, SCORE_UPRIGHT, SCORE_BOTH_HOLD = 0.25, 0.5, 0.75
SUCCESS_HOLD_S = 3.0  # s: the success state must hold this long in a row (yam_arena ends the episode then)
GRIPPERS = ("left_gripper", "right_gripper")

_history = {"lifted": False, "set_down": False, "both_held": False, "steps": 0, "recent": []}
_TRACE = bool(os.environ.get("TERMINAL_TRACE"))  # TERMINAL_TRACE=1 prints the scoring state every 10 steps


class GripperState:
    def __init__(self, tcp, gap):
        self.tcp = np.asarray(tcp, dtype=float)  # world frame, m
        self.gap = float(gap)  # m between the finger pads


def _quat_rotate(q_xyzw, v):
    x, y, z, w = q_xyzw
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def _distance_to_box(point, low, high) -> float:
    return float(np.linalg.norm(np.maximum(0.0, np.maximum(low - point, point - high))))


def _add_grippers() -> None:
    """Wrap yam_arena's object_states so each step's dict also holds both grippers' state. Installed on the first
    scored step (yam_arena.scene imports Isaac Sim, so not when the task file loads); scores read from step 2."""
    import yam_arena.scene as scene
    from yam_arena.constants import FINGER_JOINTS, TCP_OFFSET

    original = scene.object_states
    if getattr(original, "_with_grippers", False):
        return

    def object_states(env, names):
        out = original(env, names)
        base = env.unwrapped
        origin = base.scene.env_origins[0].detach().cpu().numpy()
        for key, robot in base.scene.articulations.items():
            if "gripper" not in robot.body_names:
                continue
            body = robot.body_names.index("gripper")
            pos = robot.data.body_pos_w.torch[0, body].detach().cpu().numpy() - origin
            quat = robot.data.body_quat_w.torch[0, body].detach().cpu().numpy()
            fingers = [robot.joint_names.index(j) for j in FINGER_JOINTS]
            q = robot.data.joint_pos.torch[0, fingers].detach().cpu().numpy()
            side = "left" if float(robot.data.root_pos_w.torch[0, 1]) > float(origin[1]) else "right"  # by the arm's base
            out[f"{side}_gripper"] = GripperState(pos + _quat_rotate(quat, np.array(TCP_OFFSET)), float(np.abs(q).sum()))
        return out

    object_states._with_grippers = True
    scene.object_states = object_states


def _view(objects) -> dict:
    """What scoring looks at; also keeps the episode's history (lifted, set down, held by both)."""
    _add_grippers()
    t = objects["terminal"]
    height = float(t.low[2] - TOP)
    up = _quat_rotate(np.asarray(t.quat_xyzw, dtype=float), UP_IN_MODEL)
    tilt = float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))

    def reach(side):
        g = objects.get(f"{side}_gripper")
        return None if g is None else _distance_to_box(g.tcp, t.low, t.high)

    def holds(side):
        g, d = objects.get(f"{side}_gripper"), reach(side)
        return g is not None and d <= HOLD_REACH and HOLD_GAP[0] <= g.gap <= HOLD_GAP[1]

    left_reach = reach("left")
    at_start = np.hypot(t.pos[0] - START_XY[0], t.pos[1] - START_XY[1]) < 0.002 and height < TOUCHING and t.resting()
    if at_start:  # a fresh episode: the terminal lies untouched where it starts
        _history.update(lifted=False, set_down=False, both_held=False, recent=[])
    if height > LIFTED:
        _history["lifted"] = True
    elif _history["lifted"] and height < TOUCHING:
        _history["set_down"] = True
    in_air = height >= LIFTED
    if in_air and holds("left") and holds("right"):
        _history["both_held"] = True
    return {
        "height": height,
        "in_air": in_air,
        "upright": tilt < UPRIGHT_DEG,
        "tilt": tilt,
        "right_holds": holds("right"),
        "right_by_ends": holds("right") and END_GRIP_GAP[0] <= objects["right_gripper"].gap <= END_GRIP_GAP[1],
        "left_released": left_reach is not None and left_reach >= RELEASED,
        "left_reach": left_reach,
        "still": _still(t.pos),
    }


def _still(pos) -> bool:
    """Whether the terminal's centre stayed within STILL_MM over the last STILL_STEPS steps. Contact jitter in a
    grip makes the simulator's velocities unreliable for a held part, so this looks at positions instead."""
    recent = _history["recent"]
    if len(recent) < STILL_STEPS:
        return False
    pts = np.array(recent)
    return float(np.max(np.linalg.norm(pts - pts.mean(axis=0), axis=1))) * 1000 <= STILL_MM


def success(objects):
    """In the air, upright (foot down), held by the right gripper alone by its two narrow ends after both grippers
    held it at once, still, and never set down after the lift. Must hold for SUCCESS_HOLD_S."""
    _history["recent"].append(np.asarray(objects["terminal"].pos, dtype=float))  # success() runs once per step
    del _history["recent"][:-STILL_STEPS]
    v = _view(objects)
    ok = bool(
        v["in_air"]
        and v["upright"]
        and v["right_by_ends"]
        and v["left_released"]
        and _history["both_held"]
        and v["still"]
        and not _history["set_down"]
    )
    if _TRACE:
        _history["steps"] += 1
        if _history["steps"] % 10 == 0:
            t, g = objects["terminal"], {s: objects.get(f"{s}_gripper") for s in ("left", "right")}
            gaps = " ".join(f"{s}:tcp={np.round(x.tcp, 3)},gap={x.gap * 1000:.0f}mm" for s, x in g.items() if x)
            print(
                f"TRACE {_history['steps']} pos={np.round(t.pos, 3)} h={v['height']:.3f} tilt={v['tilt']:.0f} "
                f"right_holds={v['right_holds']} left_reach={v['left_reach']} both={_history['both_held']} "
                f"by_ends={v['right_by_ends']} set_down={_history['set_down']} still={v['still']} ok={ok} | {gaps}",
                flush=True,
            )
    return ok


def score(objects):
    """Partial credit (success scores 1.0): in the air 0.25, and upright 0.5, held by both grippers at once 0.75;
    0.25 at most once the terminal was set down after the lift."""
    v = _view(objects)
    if not v["in_air"]:
        return 0.0
    credit = SCORE_UPRIGHT if v["upright"] else SCORE_LIFTED
    if _history["both_held"]:
        credit = max(credit, SCORE_BOTH_HOLD)
    return min(credit, SCORE_LIFTED) if _history["set_down"] else credit


# -- scripted demo ----------------------------------------------------------------------------------------------


class Orientation(float):
    """A waypoint's ``yaw`` that carries a full TCP rotation matrix (yam_arena's waypoints only turn a downward
    grip about the vertical). Its float value is unused."""

    def __new__(cls, matrix):
        obj = super().__new__(cls, 0.0)
        obj.matrix = np.asarray(matrix, dtype=float)
        return obj


def _enable_full_orientations(demo_module) -> None:
    """Let yam_arena's ArmPlan follow an :class:`Orientation` passed as a waypoint's yaw. Only the scripted demo
    calls this; policies under evaluation never touch yam_arena.demo."""
    original = demo_module.grip_rotation
    if getattr(original, "_full_orientations", False):
        return

    def grip_rotation(yaw: float = 0.0):
        return yaw.matrix if isinstance(yaw, Orientation) else original(yaw)

    grip_rotation._full_orientations = True
    demo_module.grip_rotation = grip_rotation


def demo():
    """Left arm: grips the flat terminal across its height (top and bottom edge), lifts it and turns it upright
    in the air, gripper pointing forward. Right arm: comes down from above and grips the terminal by its two
    narrow ends, across its length. Left lets go and backs off; right carries it to its side."""
    import yam_arena.demo as yd
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, READY, RIGHT_BASE_POS
    from yam_arena.demo import Waypoint
    from yam_arena.ik import tcp_pose
    from yam_arena.urdf import axis_angle_matrix

    _enable_full_orientations(yd)

    def rot(axis, deg):
        return axis_angle_matrix(np.asarray(axis, dtype=float), np.radians(deg))

    down_x = yd.grip_rotation(np.pi / 2)  # pointing down, fingers closing along world x
    g_down = Orientation(down_x)
    g_fwd = Orientation(rot((0, 1, 0), -90) @ down_x)  # pointing +x, fingers closing along z
    g_ends = Orientation(yd.grip_rotation(0.0))  # pointing down, fingers closing along world y (the length)

    # Pick: TCP over the solid band of the terminal (GRIP_ALONG from its centre), pad tips 2 mm above the table.
    x0, y0 = START_XY[0], START_XY[1] + GRIP_ALONG
    grasp_z = TOP + PAD_TIP_PAST_TCP + 0.002
    hover_z = TOP + 0.12
    # Handover: the terminal's centre in the air between the arms, upright.
    c = np.array(HANDOVER_CENTRE)
    # Gripped at grasp_z, the centre sits below the TCP by (grasp_z - TOP) - THICKNESS/2; turned, that is along +x.
    left_tcp = c + np.array([THICKNESS / 2 - (grasp_z - TOP), GRIP_ALONG, -GRIP_SLIP])
    # Right jaws come down over both narrow ends (the length) with the pad tips RIGHT_TIPS_ABOVE_FOOT above the
    # terminal's bottom edge; the TCP is PAD_TIP_PAST_TCP above the tips.
    right_tcp = np.array([c[0], c[1], c[2] - HEIGHT / 2 + RIGHT_TIPS_ABOVE_FOOT + PAD_TIP_PAST_TCP])
    right_approach = right_tcp + np.array([0.0, 0.0, 0.05])
    right_goal_tcp = np.array(GOAL_CENTRE) + (right_tcp - c)

    left = [
        Waypoint(1.5, (x0, y0, hover_z), FINGER_OPEN, "to the terminal", g_down),
        Waypoint(1.5, (x0, y0, grasp_z), FINGER_OPEN, "descend", g_down),
        Waypoint(0.8, (x0, y0, grasp_z), FINGER_CLOSED, "close", g_down),
        Waypoint(1.2, (x0, y0, hover_z), FINGER_CLOSED, "lift", g_down),
        Waypoint(2.5, tuple(left_tcp), FINGER_CLOSED, "turn upright", g_fwd),
        Waypoint(3.0, tuple(left_tcp), FINGER_CLOSED, "hold until the right arm has it", g_fwd),
        Waypoint(0.8, tuple(left_tcp), FINGER_OPEN, "let go", g_fwd),
        Waypoint(1.2, tuple(left_tcp - np.array([0.08, 0.0, 0.0])), FINGER_OPEN, "back off", g_fwd),
        Waypoint(1.5, tuple(left_tcp + np.array([-0.08, 0.08, 0.02])), FINGER_OPEN, "clear", g_fwd),
    ]
    # The right arm closes on the ends while the left still holds; left lets go, backs off and clears; only then
    # does the right arm move off with the terminal.
    right_closed_at = 3.0 + 4.5 + 1.5 + 0.8
    right_wait_for_left = sum(w.duration for w in left) - right_closed_at
    assert right_wait_for_left > 0 and sum(w.duration for w in left[:6]) > right_closed_at
    right_start, right_start_rot = tcp_pose(np.array(READY, dtype=float), np.asarray(RIGHT_BASE_POS, dtype=float))
    right = [
        Waypoint(3.0, tuple(right_start), FINGER_OPEN, "wait", Orientation(right_start_rot)),
        Waypoint(4.5, tuple(right_approach), FINGER_OPEN, "above the terminal", g_ends),
        Waypoint(1.5, tuple(right_tcp), FINGER_OPEN, "down over its ends", g_ends),
        Waypoint(0.8, tuple(right_tcp), RIGHT_SQUEEZE, "close on the ends", g_ends),
        Waypoint(right_wait_for_left, tuple(right_tcp), RIGHT_SQUEEZE, "hold until left is clear", g_ends),
        Waypoint(2.5, tuple(right_goal_tcp), RIGHT_SQUEEZE, "carry right", g_ends),
        Waypoint(5.0, tuple(right_goal_tcp), RIGHT_SQUEEZE, "hold ready to mount", g_ends),
    ]
    return {"left": left, "right": right}


HANDOVER_CENTRE = (0.26, 0.02, TOP + 0.12)  # m: the terminal's centre when the right arm takes it
GOAL_CENTRE = (0.26, -0.10, TOP + 0.12)  # m: where the right arm holds it at the end
GRIP_SLIP = 0.0205  # m: measured; the terminal settles this much higher in the left grip than its bbox centre
GRIP_ALONG = 0.0146  # m, world +y: the left arm grips 24 mm from one end, where the housing is solid top to bottom
RIGHT_SQUEEZE = -0.036  # finger joint target: pads 72 mm apart, 5 mm inside the 77 mm length -> a limited squeeze
RIGHT_TIPS_ABOVE_FOOT = 0.008  # m: right pad tips above the bottom edge; both ends are solid from ~10 to 41 mm
