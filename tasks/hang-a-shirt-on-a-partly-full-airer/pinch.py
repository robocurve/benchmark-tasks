"""pinch: a gripper holds fabric that it has physically closed on, as a dry pinch does.

PhysX cannot hold a thin FEM shell between two position-controlled finger pads by friction alone (the shell slips
out or tunnels at any pinch force the YAM gripper gives). So a pinch is modelled as a contact-gated attachment, as
the fidelity note says:

* an arm is *armed* while its finger gap is over ``OPEN_GAP`` (10 mm);
* when an armed arm's gap closes to ``GRIP_GAP`` (6 mm) or less, every fabric vertex then lying in the space between
  its two pads (the pads' 20 mm width, from 12 mm behind the TCP to the pad tips, and no wider than the gap) is fixed
  to the left finger's link at its current position (a PhysX vertex-to-body attachment). If there are fewer than
  ``MIN_VERTICES`` such vertices it holds nothing: closing on air, or on the edge of a fold the pads pushed away,
  does not grip;
* opening the gap past ``OPEN_GAP`` again removes the attachment, and the fabric falls or hangs as it lies.

Touching fabric with closed fingers never grips it, and nothing but closing the gripper on fabric creates a hold.
The pads still collide with the fabric throughout.
"""

from __future__ import annotations

import numpy as np

import bodykit
import clothkit
import fastik

BASES = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}
OPEN_GAP = 0.010
GRIP_GAP = 0.006
MIN_VERTICES = 3
PAD_HALF_WIDTH = 0.010  # across the pads (gripper y)
ZONE_Z = (-0.012, 0.0155)  # along the approach, from the TCP (pad tips are 13 mm past it; 2.5 mm under them)

_STATE: dict = {}


def _reset_state() -> None:
    _STATE.clear()
    for arm in BASES:
        _STATE[arm] = {"armed": False, "held": None, "gap": 0.094, "tip": None}


_reset_state()


def held(arm: str):
    """(sheet name, vertex indices) the arm holds, or None."""
    return _STATE[arm]["held"]


def gap(arm: str) -> float:
    return _STATE[arm]["gap"]


def _tip_path(stage, arm: str) -> str:
    key = "LeftArm" if arm == "left" else "RightArm"
    for prim in stage.Traverse():
        p = prim.GetPath().pathString
        if key in p and p.endswith("/gripper/tip_left"):
            return p
    raise RuntimeError(f"pinch: no tip_left link for the {arm} arm")


def _release(stage, arm: str) -> None:
    h = _STATE[arm]["held"]
    if h is not None:
        stage.RemovePrim(h[2])
        _STATE[arm]["held"] = None


def tcp_frame(env, arm: str):
    robot = env.unwrapped.scene[f"{arm}_robot"]
    ids, _ = robot.find_joints([f"joint{i}" for i in range(1, 7)], preserve_order=True)
    q = bodykit.as_numpy(robot.data.joint_pos)[0][ids]
    return fastik.fk(q, BASES[arm])


@bodykit.on_step
def _update(env) -> None:
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, Sdf, Vt

    stage = get_current_stage()
    origin = bodykit.as_numpy(env.unwrapped.scene.env_origins[0])
    for arm, st in _STATE.items():
        robot = env.unwrapped.scene[f"{arm}_robot"]
        fid, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)
        g = float(-2.0 * bodykit.as_numpy(robot.data.joint_pos)[0][fid].mean())
        st["gap"] = g
        if g > OPEN_GAP:
            _release(stage, arm)
            st["armed"] = True
            continue
        if not st["armed"] or g > GRIP_GAP:
            continue
        st["armed"] = False  # one try per closing
        pos, rot = tcp_frame(env, arm)
        if st["tip"] is None:
            st["tip"] = _tip_path(stage, arm)
        bid, _ = robot.find_bodies(["tip_left"], preserve_order=True)
        bpos = bodykit.as_numpy(robot.data.body_pos_w)[0][bid[0]]
        brot = bodykit.rotation(tuple(bodykit.as_numpy(robot.data.body_quat_w)[0][bid[0]]))
        for name in clothkit.SHEETS:
            p = clothkit.points(name)
            local = (p - pos) @ rot  # TCP frame: x closing, y across the pads, z approach
            sel = np.nonzero((np.abs(local[:, 0]) <= g / 2 + 0.0015) & (np.abs(local[:, 1]) <= PAD_HALF_WIDTH)
                             & (local[:, 2] >= ZONE_Z[0]) & (local[:, 2] <= ZONE_Z[1]))[0]
            if len(sel) < MIN_VERTICES:
                continue
            path = f"{clothkit.path(name)}_held_{arm}"
            prim = stage.DefinePrim(path, "OmniPhysicsVtxXformAttachment")
            in_body = (p[sel] + origin - bpos) @ brot
            prim.CreateRelationship("omniphysics:src0").SetTargets([Sdf.Path(clothkit.path(name))])
            prim.CreateRelationship("omniphysics:src1").SetTargets([Sdf.Path(st["tip"])])
            prim.CreateAttribute("omniphysics:vtxIndicesSrc0", Sdf.ValueTypeNames.IntArray).Set(
                Vt.IntArray([int(v) for v in sel]))
            prim.CreateAttribute("omniphysics:localPositionsSrc1", Sdf.ValueTypeNames.Point3fArray).Set(
                Vt.Vec3fArray([Gf.Vec3f(*map(float, q)) for q in in_body]))
            prim.CreateAttribute("omniphysics:attachmentEnabled", Sdf.ValueTypeNames.Bool).Set(True)
            st["held"] = (name, sel, path)
            print(f"[pinch] {arm} hand holds {len(sel)} vertices of {name}", flush=True)
            break


@bodykit.before_reset
def _drop_all() -> None:
    try:
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
        for arm in _STATE:
            _release(stage, arm)
    except Exception:  # noqa: BLE001 -- before the first episode there is no stage yet
        pass
    _reset_state()
