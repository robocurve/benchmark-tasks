"""Clean one wireless earbud and replace its silicone eartip.

The old tip must be pulled from the nozzle and discarded.  Visible dry debris
on the speaker mesh is removed only by brush-bristle contact plus tangential
motion; charging-contact soil is removed only by cotton-tip contact plus
tangential motion.  A fresh tip must then be stretched onto the nozzle and
left fully seated.  No liquid, putty, sharp tool, or charging case is present.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, body_state,  # noqa: E402
                     on_reset, on_step, rotation, set_slide_target, set_thread_on, thread)
from scoring import Evidence, score_evidence  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = ("Remove the worn eartip and put it in the discard cup. Dry-brush the speaker mesh, clean the "
               "charging contacts with the cotton swab, then fit the new eartip. Leave the earbud and tools released.")

# Layout and published Buds+ envelope (22.5 x 17.5 x 19.2 mm; 6.3 g).
MAT_Z = TOP + 0.004
EB0 = np.array([0.325, -0.015, MAT_Z + 0.011])
NOZZLE_BASE = np.array([0.010, 0.0, 0.0])
NOZZLE_FACE_X = 0.019
TIP_OFF = 0.014
TIP_SEATED = 0.002
TIP_CAPTURE_R = 0.006
TIP_CAPTURE_DEG = 30.0

BRUSH0 = np.array([0.270, 0.145, MAT_Z + 0.055])
SWAB0 = np.array([0.340, -0.080, MAT_Z + 0.013])  # clear of the brush's mesh strokes
NEW0 = np.array([0.225, -0.145, MAT_Z + 0.009])
DISCARD = np.array([0.430, -0.150, MAT_Z])
DISCARD_R, DISCARD_H = 0.030, 0.025

# Deliberately conservative prototype thresholds; physical calibration is still required.
CONTACT_GAP = 0.0025
MESH_RADIUS = 0.0050
CONTACT_HALF_X, CONTACT_HALF_Y = 0.008, 0.006
STROKE_QUANTUM = 0.0020
CONTACT_STROKE_QUANTUM = 0.0005
MESH_MARKS, CONTACT_MARKS = 8, 6
RELEASE_CLEARANCE = 0.040
SUCCESS_HOLD_S = 1.0
CAL_STAGE = os.environ.get("EARBUD_CAL_STAGE", "")

BLACK = (0.035, 0.045, 0.055)
BLACK_2 = (0.10, 0.12, 0.14)
SILICONE_OLD = (0.38, 0.40, 0.42)
SILICONE_NEW = (0.72, 0.84, 0.96)
STEEL = (0.68, 0.70, 0.72)
GOLD = (0.95, 0.67, 0.12)
SOIL = (0.58, 0.28, 0.08)
MAT = (0.17, 0.23, 0.29)
TRAY = (0.32, 0.42, 0.52)
CRADLE = (0.24, 0.34, 0.42)


def _mat() -> Body:
    return Body.build("inspection_mat", (0.325, 0.0, TOP), 1.0,
                      [BoxShape((0, 0, 0.002), (0.34, 0.40, 0.004), MAT)], kinematic=True)


def _earbud() -> Body:
    shapes = [
        SphereShape((0.000, 0.000, 0.000), 0.0090, BLACK),
        SphereShape((-0.004, 0.000, -0.002), 0.0075, BLACK_2),
        BoxShape((0.001, 0.000, -0.003), (0.016, 0.014, 0.010), BLACK),
        CylShape((0.0145, 0, 0), 0.0042, 0.009, BLACK_2, axis="X"),
        CylShape((NOZZLE_FACE_X + 0.0002, 0, 0), 0.0037, 0.0005, STEEL, collide=False, axis="X"),
        # Two visible charging contacts on the upward-facing service side.
        CylShape((-0.004, -0.004, 0.0085), 0.0020, 0.0005, GOLD, collide=False),
        CylShape((-0.004, 0.004, 0.0085), 0.0020, 0.0005, GOLD, collide=False),
    ]
    return Body.build("earbud", EB0, 0.0063, shapes, color=BLACK, friction=(0.9, 0.8),
                      kinematic=True, damping=(0.05, 0.12), solver_iterations=32)


def _cradle() -> Body:
    """A low silicone service cradle restrains the shell while leaving its top and nozzle exposed."""
    shapes = [
        BoxShape((-0.014, 0.0, 0.006), (0.004, 0.030, 0.012), CRADLE),
        BoxShape((-0.002, -0.0105, 0.005), (0.026, 0.003, 0.010), CRADLE),
        BoxShape((-0.002, 0.0105, 0.005), (0.026, 0.003, 0.010), CRADLE),
    ]
    return Body.build("service_cradle", (EB0[0], EB0[1], MAT_Z), 1.0, shapes,
                      color=CRADLE, kinematic=True, friction=(1.4, 1.2))


def _tip(name: str, pos, color) -> Body:
    # A thick elastomer-like collar and bell. Deformation is approximated by a compliant axial capture joint.
    shapes = [CylShape((0.002, 0, 0), 0.0050, 0.004, color, axis="X"),
              CylShape((0.006, 0, 0), 0.0070, 0.004, color, axis="X"),
              CylShape((0.009, 0, 0), 0.0085, 0.002, color, axis="X"),
              CylShape((0.004, 0, 0), 0.0030, 0.010, BLACK, collide=False, axis="X")]
    # Soft silicone is lossy: it settles in a cup instead of rocking, so the tips carry real rolling damping.
    return Body.build(name, pos, 0.00035, shapes, color=color, friction=(1.4, 1.2),
                      ignore=("earbud",), damping=(0.2, 1.0), solver_iterations=32)


def _tip_holder() -> Body:
    """A shallow presentation peg keeps the loose replacement tip upright for pickup."""
    shapes = [CylShape((0, 0, 0.0015), 0.014, 0.003, TRAY),
              CylShape((0, 0, 0.0065), 0.0026, 0.010, TRAY)]
    return Body.build("tip_holder", (NEW0[0], NEW0[1], MAT_Z), 1.0, shapes,
                      color=TRAY, kinematic=True, friction=(1.4, 1.2))


def _brush() -> Body:
    shapes = [BoxShape((0.015, 0, 0), (0.100, 0.040, 0.032), (0.20, 0.55, 0.84)),
              BoxShape((-0.010, -0.047, 0.0), (0.055, 0.100, 0.012), (0.20, 0.55, 0.84), collide=False),
              BoxShape((-0.040, -0.093, -0.002), (0.025, 0.030, 0.015),
                       (0.85, 0.88, 0.90), collide=False)]
    for y in (-0.103, -0.098, -0.093, -0.088, -0.083):
        for z in np.linspace(-0.025, -0.013, 5):
            shapes.append(CylShape((-0.0128, y, float(z)), 0.0008, 0.012,
                                   (0.88, 0.90, 0.92), axis="X", collide=False))
    return Body.build("dry_brush", BRUSH0, 0.030, shapes, color=(0.2, 0.55, 0.84),
                      friction=(3.0, 2.5), damping=(0.04, 0.15), solver_iterations=48,
                      com=(0.015, 0.0, 0.0))


def _swab() -> Body:
    shapes = [BoxShape((0, 0, 0), (0.090, 0.020, 0.020), (0.82, 0.86, 0.90)),
              SphereShape((-0.049, 0, 0), 0.0050, (0.98, 0.98, 0.96)),
              SphereShape((0.049, 0, 0), 0.0050, (0.98, 0.98, 0.96))]
    return Body.build("cotton_swab", SWAB0, 0.080, shapes, color=(0.95, 0.95, 0.94),
                      friction=(1.2, 1.0), damping=(0.03, 0.12), solver_iterations=32,
                      com=(0.0, 0.0, 0.0))


def _brush_rest() -> Body:
    """Raised service-tray ledge supporting the brush handle while its offset bristles clear the mat."""
    return Body.build("brush_rest", (BRUSH0[0] + 0.015, BRUSH0[1], MAT_Z + 0.040), 1.0,
                      [BoxShape((0, 0, 0), (0.070, 0.042, 0.004), TRAY)],
                      color=TRAY, kinematic=True)


def _cup() -> Body:
    shapes = [CylShape((0, 0, 0.002), DISCARD_R, 0.004, TRAY)]
    # Twelve overlapping wall panels make a genuinely hollow cup; a single collision cylinder would seal the mouth.
    n, wall = 12, 0.002
    radius = DISCARD_R - wall / 2
    chord = 2 * radius * math.sin(math.pi / n) + 0.001
    for k in range(n):
        a = 2 * math.pi * k / n
        shapes.append(BoxShape((radius * math.cos(a), radius * math.sin(a), DISCARD_H / 2),
                               (chord, wall, DISCARD_H), TRAY, yaw=a + math.pi / 2))
    return Body.build("discard_cup", DISCARD, 1.0, shapes, color=TRAY, kinematic=True)


MESH_MARK_LOCAL = [np.array([NOZZLE_FACE_X + 0.0006,
                             0.0025 * math.cos(2 * math.pi * k / MESH_MARKS),
                             0.0025 * math.sin(2 * math.pi * k / MESH_MARKS)])
                   for k in range(MESH_MARKS)]
CONTACT_MARK_LOCAL = [np.array([-0.004 + dx, y + dy, 0.0092])
                      for y in (-0.004, 0.004)
                      for dx, dy in ((-0.0012, 0.0), (0.0008, -0.0010), (0.0008, 0.0010))]


def _soil_mark(name: str, local: np.ndarray) -> Body:
    """A visual-only kinematic speck that follows the earbud until its inspection mark is cleared."""
    return Body.build(name, EB0 + local, 1.0, [SphereShape((0, 0, 0), 0.00055, SOIL, collide=False)],
                      color=SOIL, kinematic=True)


EARBUD = _earbud()
OLD_TIP = _tip("old_tip", DISCARD + [0, 0, 0.008] if CAL_STAGE else EB0 + NOZZLE_BASE, SILICONE_OLD)
NEW_TIP = _tip("new_tip", NEW0, SILICONE_NEW)
DRY_BRUSH = _brush()
COTTON_SWAB = _swab()
SOIL_MARKS = ([_soil_mark(f"mesh_soil_{k}", p) for k, p in enumerate(MESH_MARK_LOCAL)]
              + [_soil_mark(f"contact_soil_{k}", p) for k, p in enumerate(CONTACT_MARK_LOCAL)])

thread("old_tip_fit", "earbud", "old_tip", tuple(NOZZLE_BASE), (1, 0, 0), (0, 0, 0), (1, 0, 0),
       (-0.002, TIP_OFF + 0.004), 0.001, 1500.0, 4.0, 1.5, play=0.0008, tilt_deg=4.0,
       always_on=not bool(CAL_STAGE))
thread("new_tip_fit", "earbud", "new_tip", tuple(NOZZLE_BASE), (1, 0, 0), (0, 0, 0), (1, 0, 0),
       (-0.002, TIP_OFF + 0.004), 0.001, 1500.0, 4.0, 1.5, play=0.0008, tilt_deg=4.0)
thread("new_tip_holder", "tip_holder", "new_tip", (0, 0, 0.009), (1, 0, 0), (0, 0, 0), (1, 0, 0),
       (-0.0005, 0.0005), 0.0, 2000.0, 4.0, 2.0, play=0.0003, tilt_deg=2.0, always_on=True)
thread("brush_holder", "brush_rest", "dry_brush", (-0.015, 0, 0.018), (1, 0, 0), (0, 0, 0), (1, 0, 0),
       (-0.0005, 0.0005), 0.0, 2000.0, 4.0, 2.0, play=0.0003, tilt_deg=2.0, always_on=True)

OBJECTS = [_mat(), _cup(), _cradle(), _brush_rest(), _tip_holder(), EARBUD, OLD_TIP, NEW_TIP,
           DRY_BRUSH, COTTON_SWAB, *SOIL_MARKS]


def _fresh():
    mesh_done = MESH_MARKS if CAL_STAGE in ("swab", "tip") else 0
    contacts_done = CONTACT_MARKS if CAL_STAGE == "tip" else 0
    return {"old_on": not bool(CAL_STAGE), "new_on": False, "holder_on": True, "brush_holder_on": True,
            "old_removed": bool(CAL_STAGE),
            "mesh": mesh_done, "contacts": contacts_done, "mesh_travel": 0.0, "contact_travel": 0.0,
            "last_mesh": None, "last_contact": None, "tcps": None, "evidence": Evidence()}


EP = _fresh()
DEBUG_STEP = [0]


@on_reset
def _reset_episode():
    EP.clear()
    EP.update(_fresh())
    set_slide_target("old_tip_fit", 0.0)
    set_thread_on("old_tip_fit", not bool(CAL_STAGE))
    set_thread_on("new_tip_fit", False)
    set_thread_on("new_tip_holder", True)
    set_thread_on("brush_holder", True)
    DEBUG_STEP[0] = 0


def _pose(env, spec):
    state = body_state(env, spec)
    return np.asarray(state.pos), state.rotation(), state


def _tip_relation(pe, re, pt, rt):
    q = re.T @ (pt - (pe + re @ NOZZLE_BASE))
    angle = math.degrees(math.acos(np.clip(float(re[:, 0] @ rt[:, 0]), -1.0, 1.0)))
    return float(q[0]), float(np.linalg.norm(q[1:])), angle


def _tool_point(pos, rot, locals_):
    points = [pos + rot @ np.asarray(p, float) for p in locals_]
    return points


def _travel(last_key, travel_key, p_local, valid, marks_key, total, quantum=STROKE_QUANTUM):
    if not valid:
        EP[last_key] = None
        return
    if EP[last_key] is not None:
        EP[travel_key] += float(np.linalg.norm(p_local - EP[last_key]))
        EP[marks_key] = min(total, int(EP[travel_key] / quantum))
    EP[last_key] = p_local.copy()


def _tcps(env):
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        pose = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(pose[:3] - origin + rotation(pose[3:7]) @ np.asarray(TCP_OFFSET))
    return out


def _finger_centers(env):
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        out.append([poses[k, :3] - origin for k, name in enumerate(robot.body_names)
                    if "tip_left" in name or "tip_right" in name])
    return out


def _sync_soil_marks(env, pe: np.ndarray, re: np.ndarray, quat_xyzw) -> None:
    """Keep uncleared specks on the moving earbud and park cleared specks below the table."""
    import torch

    base = env.unwrapped
    origin = base.scene.env_origins
    local_marks = [*MESH_MARK_LOCAL, *CONTACT_MARK_LOCAL]
    visible = [k >= EP["mesh"] for k in range(MESH_MARKS)]
    visible += [k >= EP["contacts"] for k in range(CONTACT_MARKS)]
    for spec, local, show in zip(SOIL_MARKS, local_marks, visible):
        asset = base.scene[spec.name]
        pose = asset.data.default_root_pose.torch.clone()
        world = pe + re @ local if show else np.array([pe[0], pe[1], TOP - 0.10])
        pose[:, :3] = torch.as_tensor(world, dtype=pose.dtype, device=pose.device) + origin
        pose[:, 3:7] = torch.as_tensor(quat_xyzw, dtype=pose.dtype, device=pose.device)
        asset.write_root_pose_to_sim_index(root_pose=pose)
        asset.write_root_velocity_to_sim_index(root_velocity=asset.data.root_vel_w.torch * 0.0)


@on_step
def _follow(env):
    DEBUG_STEP[0] += 1
    pe, re, se = _pose(env, EARBUD)
    po, ro, so = _pose(env, OLD_TIP)
    pn, rn, sn = _pose(env, NEW_TIP)
    pb, rb, sb = _pose(env, DRY_BRUSH)
    ps, rs, ss = _pose(env, COTTON_SWAB)

    old_d, _, _ = _tip_relation(pe, re, po, ro)
    new_d, new_off, new_angle = _tip_relation(pe, re, pn, rn)
    tcps = _tcps(env)
    finger_centers = _finger_centers(env)
    right_fingers = finger_centers[1]
    finger_sep = (float(np.linalg.norm(right_fingers[0] - right_fingers[1]))
                  if len(right_fingers) >= 2 else 0.0)
    if EP["brush_holder_on"] and float(np.linalg.norm(tcps[1] - pb)) < 0.020 and finger_sep > 0.067:
        set_thread_on("brush_holder", False)
        EP["brush_holder_on"] = False
    # The peg's light retaining clip lets go only once the fingers have closed on the bell and stopped moving.
    rr = env.unwrapped.scene["right_robot"]
    f_pos = float(as_numpy(rr.data.joint_pos)[0][6])
    f_vel = abs(float(as_numpy(rr.data.joint_vel)[0][6]))
    if EP["holder_on"] and float(np.linalg.norm(tcps[1] - pn)) < 0.014 and f_pos > -0.0105 and f_vel < 0.002:
        set_thread_on("new_tip_holder", False)
        EP["holder_on"] = False
    if EP["old_on"] and old_d > TIP_OFF:
        set_thread_on("old_tip_fit", False)
        EP["old_on"] = False
        EP["old_removed"] = True
    if (not EP["old_on"] and not EP["new_on"] and new_off <= TIP_CAPTURE_R
            and -0.002 <= new_d <= TIP_OFF and new_angle <= TIP_CAPTURE_DEG):
        # Once the compliant collar catches, its elastic fit draws it toward the nozzle shoulder.
        set_slide_target("new_tip_fit", 0.0)
        set_thread_on("new_tip_fit", True)
        EP["new_on"] = True
    elif EP["new_on"]:
        if new_d > TIP_OFF:
            set_thread_on("new_tip_fit", False)
            EP["new_on"] = False
        else:
            set_slide_target("new_tip_fit", 0.0)

    # Correct-tool mesh evidence: bristle tips at the nozzle face, with the bristle axis roughly normal to it.
    bristle_points = _tool_point(
        pb, rb,
        [(-0.0188, y, z) for y in (-0.103, -0.098, -0.093, -0.088, -0.083)
         for z in np.linspace(-0.025, -0.013, 5)],
    )
    b_locals = [re.T @ (point - pe) for point in bristle_points]
    b_local = min(b_locals, key=lambda p: abs(float(p[0]) - NOZZLE_FACE_X) + np.linalg.norm(p[1:]))
    brush_normal = -(rb[:, 0])
    mesh_valid = (abs(float(b_local[0]) - NOZZLE_FACE_X) <= CONTACT_GAP
                  and np.linalg.norm(b_local[1:]) <= MESH_RADIUS
                  and abs(float(brush_normal @ re[:, 0])) >= math.cos(math.radians(50)))
    _travel("last_mesh", "mesh_travel", b_local[1:], mesh_valid, "mesh", MESH_MARKS)

    # Correct-tool contact evidence: whichever cotton end is closer must touch and slide across the top contacts.
    swab_points = _tool_point(ps, rs, [(-0.049, 0, 0), (0.049, 0, 0)])
    locals_ = [re.T @ (p - pe) for p in swab_points]
    contact_center = np.array([-0.004, 0.0, 0.009])
    c_local = min(locals_, key=lambda p: float(np.linalg.norm(p - contact_center)))
    contact_valid = (abs(float(c_local[2]) - 0.009) <= CONTACT_GAP
                     and abs(float(c_local[0]) + 0.004) <= CONTACT_HALF_X
                     and abs(float(c_local[1])) <= CONTACT_HALF_Y)
    _travel("last_contact", "contact_travel", c_local[:2], contact_valid, "contacts", CONTACT_MARKS,
            CONTACT_STROKE_QUANTUM)

    old_in_discard = (math.hypot(float(po[0] - DISCARD[0]), float(po[1] - DISCARD[1])) < DISCARD_R - 0.004
                      and DISCARD[2] < po[2] < DISCARD[2] + DISCARD_H)
    EP["tcps"] = tcps
    tools_released = all(min(float(np.linalg.norm(tcp - p)) for tcp in tcps) > RELEASE_CLEARANCE
                         for p in (pb, ps))
    earbud_released = all(min(float(np.linalg.norm(tcp - p)) for tcp in tcps) > RELEASE_CLEARANCE
                          for p in (pe, po, pn))
    objects_resting = all(x.resting() for x in (se, so, sn, sb, ss))
    EP["evidence"] = Evidence(
        old_tip_removed=EP["old_removed"], old_tip_in_discard=old_in_discard,
        mesh_marks_clean=EP["mesh"], mesh_marks_total=MESH_MARKS,
        contact_marks_clean=EP["contacts"], contact_marks_total=CONTACT_MARKS,
        new_tip_seated=EP["new_on"] and new_d <= TIP_SEATED,
        tools_released=tools_released, earbud_released=earbud_released, objects_resting=objects_resting)
    _sync_soil_marks(env, pe, re, se.quat_xyzw)
    if os.environ.get("EARBUD_DEBUG") and DEBUG_STEP[0] % 20 == 0:
        print("[earbud]", {"step": DEBUG_STEP[0], "old_d_mm": round(old_d * 1000, 1),
                           "old_removed": EP["old_removed"], "old_in_discard": old_in_discard,
                           "mesh": EP["mesh"], "contacts": EP["contacts"],
                           "brush_holder_on": EP["brush_holder_on"], "finger_sep_mm": round(finger_sep * 1000, 1),
                           "holder_on": EP["holder_on"], "finger_q": round(f_pos, 4), "finger_v": round(f_vel, 4),
                           "new_d_mm": round(new_d * 1000, 1), "new_off_mm": round(new_off * 1000, 1),
                           "new_angle": round(new_angle, 1), "new_on": EP["new_on"],
                           "earbud_mm": np.round(pe * 1000, 1).tolist(),
                           "old_tip_mm": np.round(po * 1000, 1).tolist(),
                           "new_tip_mm": np.round(pn * 1000, 1).tolist(),
                           "new_axis": np.round(rn[:, 0], 3).tolist(),
                           "brush_mm": np.round(pb * 1000, 1).tolist(),
                           "brush_quat": np.round(sb.quat_xyzw, 4).tolist(),
                           "bristle_local_mm": np.round(b_local * 1000, 1).tolist(),
                           "swab_mm": np.round(ps * 1000, 1).tolist(),
                           "cotton_local_mm": np.round(c_local * 1000, 1).tolist(),
                           "tcps_mm": [np.round(t * 1000, 1).tolist() for t in tcps],
                           "finger_mm": [[np.round(p * 1000, 1).tolist() for p in arm]
                                         for arm in finger_centers],
                           "rest": {n: [round(float(np.linalg.norm(x.lin_vel)) * 1000, 1),
                                        round(float(np.linalg.norm(x.ang_vel)), 2)]
                                    for n, x in (("bud", se), ("old", so), ("new", sn), ("brush", sb), ("swab", ss))},
                           "tools_rel": tools_released, "bud_rel": earbud_released,
                           "score": score_evidence(EP["evidence"])}, flush=True)


def score(objects) -> float:
    return score_evidence(EP["evidence"])


def success(objects) -> bool:
    return score(objects) == 1.0


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    """Choose a roomy IK solution instead of letting the Cartesian controller settle in a poor local branch."""
    import itertools

    from yam_arena import ik, station

    best = None
    seeds = itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8),
                              (-0.4, 0.0, 0.4), (-0.4, 0.4))
    for seed in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(seed, float),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for {arm} at {np.round(pos, 3)}")
    return best[0]


def _joint_lead(arm: str, q1, duration: float, finger, label: str) -> list[Waypoint]:
    from yam_arena.constants import READY

    return _joint_path(arm, np.asarray(READY, float), q1, duration, finger, label)


class _PlanJoints:
    """Replays the scripted controller's own warm-started IK so a joint-space transit starts from the joint
    configuration the controller will actually be in, not from a different branch reaching the same pose."""

    def __init__(self, waypoints: list[Waypoint], arm: str = "right", hz: float = 20.0):
        from yam_arena import station
        from yam_arena.demo import ArmPlan

        self.plan = ArmPlan(waypoints, station.BASES[arm])
        self.hz, self.k = hz, 0

    def q(self) -> np.ndarray:
        while self.k / self.hz < self.plan.duration:
            self.plan.action7(self.k / self.hz)
            self.k += 1
        return self.plan.q.copy()


def _joint_path(arm: str, q0, q1, duration: float, finger, label: str) -> list[Waypoint]:
    from yam_arena import ik, station

    # Dense samples (one per control step) keep the warm-started Cartesian IK on the interpolated joint branch.
    n = max(15, int(round(duration * 20)))
    out = []
    for k in range(1, n + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / n, station.BASES[arm])
        out.append(Waypoint(duration / n, tuple(float(v) for v in p), finger, label, r))
    return out


def demo():
    """Service the earbud in its silicone cradle with the right hand; the left remains safely parked."""
    down = grip_rotation(0.0)

    def ry(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

    def rz(angle):
        c, s = math.cos(angle), math.sin(angle)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])

    # Pitching the right wrist back gives the arm room to reach the centre line from its negative-y base.
    right_down = ry(math.radians(-25.0)) @ down
    # A 50-degree pitch keeps the wrist reachable while directing the bristles into the +x nozzle face.
    brush_face = ry(math.radians(-50.0)) @ down
    over = 0.075
    brush_over_height = 0.040
    swab_over_height = 0.040
    old_tip_hold, tip_hold = -0.006, FINGER_CLOSED
    brush_hold, swab_hold = FINGER_CLOSED, FINGER_CLOSED

    tip0 = EB0 + NOZZLE_BASE + [0.006, 0.0, 0.013]
    right_over = tip0 + [0, 0, over]
    brush_grip = BRUSH0 + [0.015, 0.0, 0.0] - 0.004 * right_down[:, 2]
    swab_grip = SWAB0 - 0.004 * right_down[:, 2]
    new_grip = NEW0 + [0.005, 0, 0.003]
    # The bell is pinched across its diameter with its bore already along +x (the peg holds it until the fingers
    # close), so the same wrist pose carries it coaxial to the nozzle.  TCP-to-tip offset measured in YAM Arena.
    # In the pinch the light bell pivots about 25 deg nose-up; its compliant collar still catches within the
    # 30 deg capture cone.  TCP-to-tip offset measured in the full YAM Arena run while carried.
    tip_carry = right_down
    grip_to_tip = np.array([0.0008, 0.0015, -0.0111])
    tip_front_tcp = EB0 + NOZZLE_BASE + [TIP_OFF + 0.004, 0, 0] - grip_to_tip
    tip_home_tcp = EB0 + NOZZLE_BASE + [0.001, 0, 0] - grip_to_tip
    if CAL_STAGE == "brush":
        brush_over = brush_grip + [0, 0, brush_over_height]
        right: list[Waypoint] = _joint_lead("right", _good_ik("right", brush_over, right_down),
                                            3.0, FINGER_OPEN, "move over the dry brush")
    elif CAL_STAGE == "swab":
        swab_over = swab_grip + [0, 0, swab_over_height]
        right = _joint_lead("right", _good_ik("right", swab_over, right_down),
                            3.0, -0.020, "move over the cotton swab")
    elif CAL_STAGE == "tip":
        new_over = new_grip + [0, 0, over]
        right = _joint_lead("right", _good_ik("right", new_over, right_down),
                            3.0, -0.012, "move over the fresh eartip")
    else:
        right_over_q = _good_ik("right", right_over, right_down)
        right_tip_q = _good_ik("right", tip0, right_down)
        right = _joint_lead("right", right_over_q, 3.5, -0.012, "move over the worn eartip")
        right += _joint_path("right", right_over_q, right_tip_q, 3.0, -0.012, "lower around the worn eartip")
        right += [Waypoint(1.5, tuple(float(v) for v in tip0), -0.012,
                           "settle around the worn eartip", right_down)]

    track = _PlanJoints(right)

    def go(t, p, finger, label, rot=right_down):
        right.append(Waypoint(t, tuple(float(v) for v in p), finger, label, rot))

    if CAL_STAGE == "tip":
        go(0.9, new_grip, -0.012, "lower around the fresh eartip")
        go(0.8, new_grip, tip_hold, "take the fresh eartip")
        go(1.0, new_grip + [0, 0, 0.040], tip_hold, "lift the fresh eartip off its peg")
        go(2.5, tip_front_tcp + [0, 0, 0.040], tip_hold, "carry it in front of the nozzle", tip_carry)
        go(0.9, tip_front_tcp, tip_hold, "align the fresh eartip with the nozzle", tip_carry)
        go(1.5, tip_home_tcp, tip_hold, "push the fresh eartip fully home", tip_carry)
        go(0.5, tip_home_tcp, FINGER_OPEN, "release the seated eartip", tip_carry)
        go(1.0, EB0 + NOZZLE_BASE + [0.045, 0, 0.060], FINGER_OPEN, "clear the work")
        go(2.0, (0.38, -0.02, TOP + 0.18), FINGER_OPEN, "park clear and hold")
        return {"right": right}

    if not CAL_STAGE:
        go(0.6, tip0, old_tip_hold, "grip the worn eartip")
        go(1.5, tip0 + [TIP_OFF + 0.014, 0, 0], old_tip_hold, "pull the worn eartip off")
        go(1.0, tip0 + [TIP_OFF + 0.014, 0, 0.060], old_tip_hold, "lift the worn eartip")
        go(1.5, DISCARD + [0, 0, DISCARD_H + 0.040], old_tip_hold, "carry it over the discard cup")
        go(0.8, DISCARD + [0, 0, DISCARD_H + 0.008], old_tip_hold, "lower it into the discard cup")
        go(0.5, DISCARD + [0, 0, DISCARD_H + 0.008], FINGER_OPEN, "release the worn eartip")
        go(0.8, DISCARD + [0, 0, DISCARD_H + 0.070], FINGER_OPEN, "clear the cup")

    if CAL_STAGE != "swab":
        # Take the dry brush by its handle, turn its bristle face toward the nozzle and make repeated cross-mesh strokes.
        brush_over = brush_grip + [0, 0, brush_over_height]
        if CAL_STAGE == "brush":
            go(0.5, brush_over, FINGER_OPEN, "over the dry brush")
        else:
            from yam_arena.constants import READY

            cup_clear = DISCARD + [0, 0, DISCARD_H + 0.070]
            right += _joint_path("right", track.q(), np.asarray(READY, float),
                                 3.0, FINGER_OPEN, "reset the arm after discarding the worn eartip")
            right += _joint_path("right", np.asarray(READY, float), np.asarray(READY, float),
                                 2.0, FINGER_OPEN, "settle at the ready posture")
            right += _joint_lead("right", _good_ik("right", brush_over, right_down), 3.0, FINGER_OPEN,
                                 "move over the dry brush")
        go(3.0, brush_over, FINGER_OPEN, "settle directly over the dry brush")
        go(0.9, brush_grip, FINGER_OPEN, "lower around the brush handle")
        go(1.0, brush_grip, FINGER_OPEN, "settle around the brush handle")
        go(1.5, brush_grip, brush_hold, "close firmly on the dry brush")
        go(1.0, brush_grip, brush_hold, "confirm the dry brush grasp")
        # The offset head already points its bristles along -x, so a translation preserves the secure handle grasp.
        mesh_tcp = EB0 + [0.0500, 0.0990, -0.0070]
        go(2.0, mesh_tcp + [0, 0, 0.040], brush_hold, "carry the bristles toward the mesh")
        go(1.0, mesh_tcp, brush_hold, "settle the bristles on the speaker mesh")
        for k in range(12):
            y = -0.006 if k % 2 == 0 else 0.006
            go(0.35, mesh_tcp + [0, y, 0], brush_hold, "dry-brush across the speaker mesh")
        go(0.8, mesh_tcp + [0.045, 0, 0], brush_hold, "withdraw the dry brush")
        go(1.4, brush_grip + [0, 0, 0.045], brush_hold, "return above the brush mark", right_down)
        go(0.8, brush_grip, brush_hold, "lower the dry brush")
        go(0.4, brush_grip, FINGER_OPEN, "release the dry brush")
        # A short lift keeps the elbow away from its folded limit before the joint-space transit.
        go(0.7, brush_grip + [0, 0, 0.020], FINGER_OPEN, "clear the dry brush")

    # Return through the roomy ready posture so each tool approach uses the independently calibrated arm branch.
    swab_over = swab_grip + [0, 0, swab_over_height]
    if CAL_STAGE == "swab":
        go(0.5, swab_over, -0.020, "over the cotton swab")
    else:
        from yam_arena.constants import READY

        brush_clear = brush_grip + [0, 0, 0.045]
        right += _joint_path("right", track.q(), np.asarray(READY, float),
                             3.0, FINGER_OPEN, "reset the arm after returning the dry brush")
        right += _joint_path("right", np.asarray(READY, float), np.asarray(READY, float),
                             2.0, FINGER_OPEN, "settle at the ready posture")
        right += _joint_lead("right", _good_ik("right", swab_over, right_down), 3.0, -0.020,
                             "move over the cotton swab")
    # Hold the swab near its centre; its +x cotton end traces both upward-facing contacts.
    go(0.8, swab_grip, -0.020, "lower around the swab shaft")
    go(1.5, swab_grip, swab_hold, "take the cotton swab")
    go(0.8, swab_grip, swab_hold, "confirm the cotton swab grasp")
    contact_tcp = EB0 + [-0.0505, 0.0062, 0.0183]
    go(1.5, contact_tcp + [0, 0, 0.040], swab_hold, "carry the cotton tip over the contacts")
    for k in range(8):
        x = -0.006 if k % 2 == 0 else 0.006
        go(0.35, contact_tcp + [x, 0, 0], swab_hold, "wipe the charging contacts", right_down)
    go(0.8, contact_tcp + [0, 0, 0.050], swab_hold, "lift the cotton swab")
    go(1.4, swab_grip + [0, 0, 0.045], swab_hold, "return above the swab mark")
    go(0.8, swab_grip, swab_hold, "lower the cotton swab")
    go(0.4, swab_grip, FINGER_OPEN, "release the cotton swab")
    go(0.7, swab_grip + [0, 0, 0.060], FINGER_OPEN, "clear the cotton swab")

    # Take the fresh tip, align its bore with the nozzle and push it to the seated gap.
    from yam_arena.constants import READY

    swab_clear = swab_grip + [0, 0, 0.045]
    right += _joint_path("right", track.q(), np.asarray(READY, float),
                         3.0, FINGER_OPEN, "reset the arm after returning the cotton swab")
    right += _joint_path("right", np.asarray(READY, float), np.asarray(READY, float),
                         2.0, FINGER_OPEN, "settle at the ready posture")
    right += _joint_lead("right", _good_ik("right", new_grip + [0, 0, over], right_down),
                         3.0, -0.012, "move over the fresh eartip")
    go(0.9, new_grip, -0.012, "lower around the fresh eartip")
    go(0.8, new_grip, tip_hold, "take the fresh eartip")
    go(1.0, new_grip + [0, 0, 0.040], tip_hold, "lift the fresh eartip off its peg")
    go(2.5, tip_front_tcp + [0, 0, 0.040], tip_hold, "carry it in front of the nozzle", tip_carry)
    go(0.9, tip_front_tcp, tip_hold, "align the fresh eartip with the nozzle", tip_carry)
    go(1.5, tip_home_tcp, tip_hold, "push the fresh eartip fully home", tip_carry)
    go(0.5, tip_home_tcp, FINGER_OPEN, "release the seated eartip", tip_carry)
    go(1.0, EB0 + NOZZLE_BASE + [0.045, 0, 0.060], FINGER_OPEN, "clear the work")
    go(2.0, (0.38, -0.02, TOP + 0.18), FINGER_OPEN, "park clear and hold")

    # The left arm is omitted, so the scripted policy holds it at its ready posture, clear of the right arm.
    return {"right": right}
