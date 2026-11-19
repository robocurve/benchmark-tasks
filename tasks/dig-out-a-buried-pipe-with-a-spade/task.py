"""Use both hands on the spade to uncover the marked pipe and collect the spoil."""

import json
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit
import contact_monitor
import digplan as d
import fastik
import soilkit
from bodykit import Body, BoxShape, CylShape
from scoring import Progress
from spade import build_rests, build_spade
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

SCENE = "station"
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
CEDAR = (0.62, 0.42, 0.27)
PVC = (0.95, 0.95, 0.93)
bodykit.FINISH[PVC] = (0.25, 0.0)
bodykit.allow_matrix_grips()

BED = Body.build(
    "planter",
    pos=(*d.CENTRE, d.TOP),
    mass=5.0,
    kinematic=True,
    yaw=d.YAW,
    shapes=[
        BoxShape(
            (0, 0, d.FLOOR / 2),
            (d.IN_U + 2 * d.WALL, d.IN_V + 2 * d.WALL, d.FLOOR),
            CEDAR,
        ),
        BoxShape(
            ((d.IN_U + d.WALL) / 2, 0, d.WALL_H / 2),
            (d.WALL, d.IN_V + 2 * d.WALL, d.WALL_H),
            CEDAR,
        ),
        BoxShape(
            (-(d.IN_U + d.WALL) / 2, 0, 0.045 / 2),
            (d.WALL, d.IN_V + 2 * d.WALL, 0.045),
            CEDAR,
        ),
        *[
            BoxShape(
                (0, s * (d.IN_V + d.WALL) / 2, d.WALL_H / 2),
                (d.IN_U, d.WALL, d.WALL_H),
                CEDAR,
            )
            for s in (-1, 1)
        ],
        *[
            BoxShape(
                (u, s * (d.IN_V + d.WALL) / 2, d.WALL_H + 0.001),
                (0.002, d.WALL, 0.002),
                (0.95, 0.65, 0.10),
                collide=False,
            )
            for u in (-0.020, 0.100)
            for s in (-1, 1)
        ],
    ],
)
PIPE = Body.build(
    "pipe",
    pos=(*d.CENTRE, d.PIPE_Z),
    mass=0.2,
    kinematic=True,
    yaw=d.YAW,
    shapes=[CylShape((0, 0, 0), d.PIPE_R, d.IN_U + 0.04, PVC, axis="X")],
)
TARP = Body.build(
    "tarp",
    pos=(*d.TARP_C, d.TOP),
    mass=1.0,
    kinematic=True,
    yaw=d.TARP_YAW,
    shapes=[
        BoxShape((0, 0, 0.001), (*d.TARP_SIZE, 0.002), (0.20, 0.35, 0.22)),
        *[
            BoxShape(
                (s * (d.TARP_SIZE[0] + 0.004) / 2, 0, 0.0125),
                (0.004, d.TARP_SIZE[1] + 0.008, 0.025),
                (0.20, 0.35, 0.22),
            )
            for s in (-1, 1)
        ],
        *[
            BoxShape(
                (0, s * (d.TARP_SIZE[1] + 0.004) / 2, 0.0125),
                (d.TARP_SIZE[0], 0.004, 0.025),
                (0.20, 0.35, 0.22),
            )
            for s in (-1, 1)
        ],
    ],
)
SPADE = build_spade()
SPILL_FLOOR = Body.build(
    "spill_floor",
    pos=(0, 0, -0.275),
    mass=1.0,
    kinematic=True,
    shapes=[BoxShape((0, 0, 0), (4.0, 4.0, 0.5), (0.45, 0.45, 0.46))],
)
# Below the station's visible floor; a thick collider catches small grains that
# tunnel through its thin slab. Nothing is deleted or moved back into the bed.
OBJECTS = [BED, PIPE, TARP, build_rests(), SPADE, SPILL_FLOOR]
soilkit.soil(
    "soil",
    host="planter",
    origin=(*d.CENTRE, d.TOP),
    yaw=d.YAW,
    low=(-d.IN_U / 2, -d.IN_V / 2, d.FLOOR),
    high=(d.IN_U / 2, d.IN_V / 2, d.FLOOR + 0.210),
    grain_shape="icosahedron",
    density=2650.0,
    angular_damping=0.1,
    start_file=None
    if os.environ.get("DIG_FRESH_FILL")
    else str(pathlib.Path(__file__).with_name("soil_shallow_start.npz")),
    keep_out=(((*d.CENTRE, d.PIPE_Z), (*d.U, 0), d.PIPE_R),),
)

_step = 0
_trace = None
_hands = {}
_progress = Progress()


@bodykit.on_reset
def reset():
    global _step, _trace, _hands, _progress
    _step, _hands = 0, {}
    _progress = Progress()
    # Long granular trials showed progressively corrupted shadow bands with
    # temporal lighting caches. Use the renderer's supported reproducible mode;
    # this changes rendering only, not contact physics or the task state.
    from isaaclab_physx.renderers.isaac_rtx_renderer_utils import (
        apply_isaac_rtx_determinism_settings,
    )

    apply_isaac_rtx_determinism_settings()
    if _trace is not None:
        _trace.close()
    path = os.environ.get("DIG_TRACE")
    _trace = open(path, "w") if path else None  # noqa: SIM115 -- closes on episode reset


@bodykit.on_step
def observe(env):
    global _step
    _step += 1
    base = env.unwrapped
    spade = bodykit._body_state(env, SPADE)
    for arm in ("left", "right"):
        robot = base.scene[arm + "_robot"]
        q = bodykit.as_numpy(robot.data.joint_pos)[0]
        ids, _ = robot.find_joints(
            ["joint" + str(i) for i in range(1, 7)], preserve_order=True
        )
        fingers, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)
        p, r = fastik.fk(q[ids], d.BASES[arm])
        _hands[arm] = (p, r, float(q[fingers].mean()))
    pos, rad = soilkit.grains("soil")
    _progress.update(
        pos,
        rad,
        spade,
        _hands,
        pipe_contact=contact_monitor.pipe_touched,
        hand_soil_contact=contact_monitor.robot_soil_touched,
        dt=base.step_dt,
        grip_forces=contact_monitor.grip_forces,
        orientations=soilkit.grain_orientations("soil"),
    )
    if _trace is not None and _step % 10 == 0:
        pos, rad = soilkit.grains("soil")
        local = (pos - spade.pos) @ spade.rotation()
        row = {
            "step": _step,
            "spade_pos": spade.pos.tolist(),
            "spade_quat": spade.quat_xyzw.tolist(),
            "hands": {
                a: {
                    "pos": p.tolist(),
                    "finger": f,
                    "grip_error": float(
                        np.linalg.norm(
                            p - spade.pos - spade.rotation() @ d.grips_local()[a][0]
                        )
                    ),
                }
                for a, (p, r, f) in _hands.items()
            },
            "soil_top": float(np.percentile(pos[:, 2], 98) - d.TOP),
            "metrics": _progress.metrics,
            "pipe_force": contact_monitor.peak_pipe_force,
            "robot_soil_force": contact_monitor.peak_robot_soil_force,
            "any_robot_soil_force": contact_monitor.peak_any_soil_force,
            "grip_forces": contact_monitor.grip_forces.copy(),
            "on_blade": int(
                (
                    (local[:, 0] > 0)
                    & (local[:, 0] < d.BLADE_LEN)
                    & (abs(local[:, 1]) < d.BLADE_W / 2)
                    & (local[:, 2] > -0.01)
                    & (local[:, 2] < 0.06)
                ).sum()
            ),
        }
        _trace.write(json.dumps(row) + "\n")
        _trace.flush()


def score(objects):
    return _progress.metrics.get("score", 0.0)


def success(objects):
    return score(objects) == 1.0


def demo():
    if os.environ.get("DIG_SETTLE"):
        return {
            arm: [
                Waypoint(
                    20.0,
                    tuple(fastik.fk(d.READY, d.BASES[arm])[0]),
                    FINGER_OPEN,
                    "settle soil",
                    fastik.fk(d.READY, d.BASES[arm])[1],
                )
            ]
            for arm in ("left", "right")
        }
    origin, rotation = d.REST_POSE
    out = {}
    for arm, (local, grip) in d.grips_local().items():
        p, r = origin + rotation @ local, rotation @ grip
        out[arm] = [
            Waypoint(3.0, tuple(p - 0.07 * r[:, 2]), FINGER_OPEN, "approach spade", r),
            Waypoint(2.0, tuple(p), FINGER_OPEN, "descend to grip", r),
            Waypoint(1.0, tuple(p), FINGER_CLOSED, "close both grips", r),
        ]
    keys, start = d.spade_path()
    for _, origin, rotation, label in d.sample(keys, start):
        for arm, (local, grip) in d.grips_local().items():
            out[arm].append(
                Waypoint(
                    d.DT,
                    tuple(origin + rotation @ local),
                    FINGER_CLOSED,
                    label,
                    rotation @ grip,
                )
            )
    for waypoints in out.values():
        w = waypoints[-1]
        waypoints.append(Waypoint(1.0, w.pos, FINGER_OPEN, "release spade", w.yaw))
        waypoints.append(
            Waypoint(
                2.0,
                tuple(np.array(w.pos) - 0.07 * w.yaw[:, 2]),
                FINGER_OPEN,
                "retreat",
                w.yaw,
            )
        )
    return out
