"""On the station table, in front of the arms: a 10-inch cordless chainsaw lies on its side, its bar pointing to the
robot's right, the chain brake off (the hand guard back) and the bar bare. A yellow plastic bar cover (a scabbard,
26.5 cm long) rests on a holder beyond the bar tip, its open end toward the bar. The robot has to make the saw safe to
carry: engage the chain brake (swing the hand guard forward past its detent) and slide the cover over the bar.
TASK_VARIANT=B (environment variable) is the mirror image across the table's centre line: the bar points to the robot's
left, with the cover beyond its tip on that side. A is the default.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import Shape, UsdBody, before_reset, on_reset, on_step, set_joint_targets  # noqa: E402

import geometry as g  # noqa: E402
import logic  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = g.HOLD_S

INSTRUCTION = "Make the chainsaw safe to carry."

TOP = g.TOP

# -- objects -------------------------------------------------------------------------------------------------------
SAW = UsdBody("chainsaw", usd="assets/chainsaw.usda" if g.S > 0 else "assets/chainsaw_b.usda", pos=(g.SAW_X, g.SAW_Y, TOP),
              size=(0.14, g.BATTERY_Y[1], g.BAR_Z), articulation=True, joint_pos=(("guard", 0.0),),
              drives=(("guard", g.GUARD_K, g.GUARD_DAMP),), friction=g.SAW_FRICTION, solver_iters=32)
SCABBARD = UsdBody("scabbard", usd="assets/scabbard.usda", pos=(g.SCAB_X, g.SCAB_CENTER_Y_START, TOP + g.SCAB_CENTER_Z),
                   rot_xyzw=(0.0, 0.0, math.sin(g.SCAB_YAW / 2), math.cos(g.SCAB_YAW / 2)),
                   size=(g.SCAB_OUT_W, g.SCAB_LEN, g.SCAB_OUT_H), mass=g.SCAB_MASS, friction=g.SCAB_FRICTION,
                   solver_iters=32)
HOLDER = Shape("scabbard_holder", pos=(g.SCAB_X, g.HOLDER_Y, TOP + g.HOLDER_TOP / 2),
               size=(g.HOLDER_W, g.HOLDER_L, g.HOLDER_TOP), color=(0.30, 0.31, 0.33))
OBJECTS = [SAW, SCABBARD, HOLDER]

# -- monitor ---------------------------------------------------------------------------------------------------------
_SCORER = logic.Scorer()
_STATE: dict = {}


def _target(environment, angle: float) -> None:
    """The hand guard's detent: a spring drive toward the brake stop once it is past the detent (it snaps over centre),
    toward the rest position once it is brought back past the release point, and holding its last target between."""
    if angle <= g.BRAKE_DETENT:
        want = g.BRAKE_STOP
    elif angle >= g.BRAKE_RELEASE:
        want = g.GUARD_REST
    else:
        want = _STATE["target"]
    if want != _STATE["target"]:
        _STATE["target"] = want
        set_joint_targets(environment, "chainsaw", {"guard": want})


@on_reset
def _reset_state(environment=None) -> None:
    _SCORER.reset()
    _STATE.update(target=g.GUARD_REST, angle=0.0, result={"score": 0.0, "success": False}, why=[], tcps=[])
    if environment is not None:
        set_joint_targets(environment, "chainsaw", {"guard": g.GUARD_REST})


@before_reset
def _restart_guard(environment) -> None:
    from scenekit import write_joint_positions

    write_joint_positions(environment, "chainsaw", {"guard": g.GUARD_REST})
    set_joint_targets(environment, "chainsaw", {"guard": g.GUARD_REST})


_reset_state()


def _tool_points(environment) -> list:
    """Both grippers' tool points (world frame), by forward kinematics from the measured arm joints."""
    from scenekit import arm_joints
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    out = []
    for side in ("left", "right"):
        q = arm_joints(environment, side, ARM_JOINTS)
        out.append(ik.tcp_pose(q, station.BASES[side])[0])
    return out


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    st = object_states(environment, ["chainsaw", "scabbard"])
    saw, scab = st["chainsaw"], st["scabbard"]
    angle = float(saw.joints["guard"])
    _STATE["angle"] = angle
    _target(environment, angle)
    tcps = _tool_points(environment)
    res = _SCORER.step(saw_pos=saw.pos, saw_quat=saw.quat_xyzw, guard_angle=angle, scab_pos=scab.pos,
                       scab_quat=scab.quat_xyzw, tcps=tcps)
    _STATE.update(result=res, why=list(_SCORER.why), tcps=tcps, saw=saw.pos.copy(), scab=scab.pos.copy(),
                  scab_tilt=logic.tilt(scab.quat_xyzw), scab_yaw=logic.yaw(scab.quat_xyzw))


def success(objects) -> bool:
    """The brake is engaged, the bar cover is on the bar, nothing was knocked, the arms are clear: held 2 s."""
    return bool(_STATE["result"]["success"])


def score(objects) -> float:
    """0.25 the chain brake engaged; 0.5 the bar cover fitted (>= 95 percent of the bar), brake off; 0.75 both, not yet
    held 2 s with the arms clear; 1.0 both, held 2 s. Capped at 0.25 if the saw was moved 3 cm or tipped, or a gripper
    touched the bar or chain. End-state: releasing the brake or pulling the cover off lowers the score again."""
    return float(_STATE["result"]["score"])
