"""A floor bay beside a twin bed (no table): the station's two arms, each on a steel pedestal, 76 cm above a vinyl floor.
Mrs. Lee, a life-size mannequin in a gown and wig, lies on her back in the bed, along the robot's right, her head at the
far end. A Drive Medical Blue Streak wheelchair (open, armrests down, footrests off) stands in front of the arms,
square to the bed and facing away from it, both its push-to-lock wheel locks off. The instruction is only "Mrs. Lee is
moving from her bed into the wheelchair. Bring the wheelchair alongside the bed for her."

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score --max-steps 3600 ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, before_reset, floor_bay, on_reset, on_step, physics  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.5

INSTRUCTION = "Mrs. Lee is moving from her bed into the wheelchair. Bring the wheelchair alongside the bed for her."

physics(g.PHYSICS_HZ)
floor_bay()

SIDES = {"l": 1.0, "r": -1.0}
JOINTS = [f"{k}_{s}" for k in ("axle", "swivel", "caster_axle", "lock_joint", "spring") for s in SIDES]
Q_C = g.TH_C - g.TH_OFF                       # the lever's centre, joint space (0 = off)
Q_ON = g.TH_ON - g.TH_OFF
CHAIR = UsdBody("wheelchair", usd="assets/wheelchair.usda", pos=(*g.START_M, g.FZ),
                rot_xyzw=(0.0, 0.0, math.sin(g.START_YAW / 2), math.cos(g.START_YAW / 2)), size=(0.61, 0.80, 0.91),
                articulation=True, joint_pos=tuple((j, 0.0) for j in JOINTS),
                drives=(("axle_.*", 0.0, g.WHEEL_DAMP, g.AXLE_LOCK_MAX), ("swivel_.*", 0.0, g.SWIVEL_DAMP),
                        ("caster_axle_.*", 0.0, 0.005), ("lock_joint_.*", g.K_OFF, g.LEVER_DAMP, 20.0),
                        ("spring_.*", g.SPRING_K, g.SPRING_D, 500.0)),
                friction=None, solver_iters=32)
ROOM = UsdBody("bedroom", usd="assets/room.usda", pos=(0.0, 0.0, 0.0), size=(1.9, 1.0, 0.8), friction=None)
OBJECTS = [CHAIR, ROOM]

_STATE: dict = {}
_SUB: dict = {}
_DEBUG = bool(os.environ.get("WC_DEBUG"))
_COL = np.load(pathlib.Path(__file__).resolve().parent / "assets" / "colliders.npz")


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x, np.float64)


def _t(x):
    return getattr(x, "torch", x)


def _rotation(xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# -- the chair's pose against the bed -------------------------------------------------------------------------------------
def chair_pose(p, R) -> dict:
    """Gaps, heading and hip offset of a chair whose frame is at p (world) with rotation R. The bed-side seat rail is
    the one nearer the mattress edge; its gaps are measured square to the edge, its heading from the two gaps over the
    rail's 42 cm, the hip offset along the edge from the hip tick to its front end."""
    out = {}
    for s_name, s in SIDES.items():
        front = p + R @ np.array([g.RAIL_X[1], s * g.W_RAIL, g.RAIL_Z])
        rear = p + R @ np.array([g.RAIL_X[0], s * g.W_RAIL, g.RAIL_Z])
        out[s_name] = (front, rear, front[1] - g.BED_EDGE_Y, rear[1] - g.BED_EDGE_Y)
    side = min(SIDES, key=lambda k: out[k][2] + out[k][3])
    front, rear, gf, gr = out[side]
    rail_len = g.RAIL_X[1] - g.RAIL_X[0]
    heading = math.asin(min(1.0, abs(gf - gr) / rail_len))
    fwd = R[:, 0]
    facing_foot = bool(fwd[0] < 0 and abs(math.atan2(fwd[1], -fwd[0])) < math.pi / 2)
    return dict(side=side, gap_front=float(gf), gap_rear=float(gr), heading=float(heading),
                hip=float(abs(front[0] - g.HIP_X)), facing_foot=facing_foot,
                yaw=float(math.atan2(fwd[1], fwd[0])), tilt=float(math.acos(max(-1.0, min(1.0, R[2, 2])))))


def pose_tier(c: dict) -> str:
    for name, lim in (("alongside", g.ALONG), ("nearly", g.NEARLY)):
        if (c["facing_foot"] and max(c["gap_front"], c["gap_rear"]) <= lim["gap"] and c["heading"] <= lim["heading"]
                and c["hip"] <= lim["hip"]):
            return name
    return "neither"


def tier(pose: str, held: dict, touching: bool) -> float:
    """The score of an end state (see score())."""
    if touching or not all(held.values()):
        return 0.0
    return {"alongside": 1.0, "nearly": 0.5}.get(pose, 0.25)


# -- distances: the robot, the chair and Mrs. Lee -------------------------------------------------------------------------
def _robot_points() -> dict:
    """Points on each link's collision hulls (vertices and the centres of their larger faces, thinned to one per 3 mm
    cell), link frame, with each link's bounding sphere."""
    if "robot_pts" in _SUB:
        return _SUB["robot_pts"]
    import xml.etree.ElementTree as ET

    from yam_arena import ASSET_DIR, YAM_URDF
    from yam_arena.analysis import read_stl
    from yam_arena.urdf import rpy_matrix

    pts: dict = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name == "base":
            continue
        for col in link.findall("collision"):
            o = col.find("origin")
            T = np.eye(4)
            if o is not None:
                T[:3, :3] = rpy_matrix(np.array([float(v) for v in o.get("rpy", "0 0 0").split()]))
                T[:3, 3] = [float(v) for v in o.get("xyz", "0 0 0").split()]
            tris = read_stl(ASSET_DIR / col.find("geometry/mesh").get("filename")) @ T[:3, :3].T + T[:3, 3]
            area = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
            pts.setdefault(name, []).append(np.vstack([tris.reshape(-1, 3), tris[area > 2e-5].mean(1)]))
    out = {}
    for name, v in pts.items():
        P = np.vstack(v)
        _, keep = np.unique(np.floor(P / 0.003).astype(np.int64), axis=0, return_index=True)
        P = P[np.sort(keep)]
        c = (P.min(0) + P.max(0)) / 2
        out[name] = (P, c, float(np.linalg.norm(P - c, axis=1).max()))
    _SUB["robot_pts"] = out
    return out


def robot_world_points(environment, near=None, margin: float = 0.02) -> np.ndarray:
    """World points of the robot's links; with ``near`` (a function: world point -> distance), only of the links whose
    bounding sphere comes within ``margin`` of it."""
    from scenekit import robot_link_poses

    out = [np.zeros((0, 3))]
    pts = _robot_points()
    for side, poses in robot_link_poses(environment).items():
        for name, (P, c, rad) in pts.items():
            if name not in poses:
                continue
            T = poses[name]
            if near is not None and near(T[:3, :3] @ c + T[:3, 3]) > rad + margin:
                continue
            out.append(P @ T[:3, :3].T + T[:3, 3])
    return np.vstack(out)


def _seg_dist(P, caps) -> np.ndarray:
    """Distance of points P (n, 3) to capsules caps (m, 7: a, b, r): (n,) minimum."""
    a, b, r = caps[:, :3], caps[:, 3:6], caps[:, 6]
    ab = b - a
    t = np.clip(np.einsum("nmk,mk->nm", P[:, None, :] - a[None], ab) / np.maximum((ab * ab).sum(1), 1e-12), 0, 1)
    d = np.linalg.norm(P[:, None, :] - (a[None] + t[..., None] * ab[None]), axis=2) - r[None]
    return d.min(1)


def _box_dist(P, boxes) -> np.ndarray:
    lo, hi = boxes[:, :3], boxes[:, 3:6]
    c, h = (lo + hi) / 2, (hi - lo) / 2
    q = np.abs(P[:, None, :] - c[None]) - h[None]
    d = np.linalg.norm(np.maximum(q, 0), axis=2) + np.minimum(q.max(2), 0)
    return d.min(1)


def _hull_planes(Pts):
    from scipy.spatial import ConvexHull

    h = ConvexHull(Pts)
    return h.equations                      # (f, 4): n . x + d <= 0 inside


def _hull_dist(P, eq) -> np.ndarray:
    return (P @ eq[:, :3].T + eq[:, 3]).max(1)


def _body_shapes() -> dict:
    if "shapes" in _SUB:
        return _SUB["shapes"]
    shapes: dict = {}
    for key in _COL.files:
        parts = key.split("|")
        if len(parts) < 2:
            continue
        body, kind = parts[0], parts[1]
        d = shapes.setdefault(body, {"capsule": np.zeros((0, 7)), "box": np.zeros((0, 6)), "hull": []})
        if kind == "hull":
            d["hull"].append(_hull_planes(_COL[key]))
        else:
            d[kind] = _COL[key]
    _SUB["shapes"] = shapes
    return shapes


def dist_to_body(P, body: str, T=None) -> float:
    """Minimum distance (m, negative inside) from world points P to a body's colliders; T its pose (4x4, world)."""
    if len(P) == 0:
        return math.inf
    sh = _body_shapes()[body]
    if T is not None:
        P = (P - T[:3, 3]) @ T[:3, :3]
    best = math.inf
    if len(sh["capsule"]):
        best = min(best, float(_seg_dist(P, sh["capsule"]).min()))
    if len(sh["box"]):
        best = min(best, float(_box_dist(P, sh["box"]).min()))
    for eq in sh["hull"]:
        best = min(best, float(_hull_dist(P, eq).min()))
    return best


def _chair_samples() -> dict:
    """Surface points of the chair's colliders, per body frame (for the chair-to-Mrs. Lee check)."""
    if "chair_samples" in _SUB:
        return _SUB["chair_samples"]
    out = {}
    for body, sh in _body_shapes().items():
        if body in ("bed", "lee"):
            continue
        pts = []
        for a_b_r in sh["capsule"]:
            a, b, r = a_b_r[:3], a_b_r[3:6], a_b_r[6]
            for t in np.linspace(0, 1, max(2, int(np.linalg.norm(b - a) / 0.02) + 1)):
                c = a + t * (b - a)
                pts += [c + r * v for v in np.eye(3)] + [c - r * v for v in np.eye(3)]
        for lo_hi in sh["box"]:
            lo, hi = lo_hi[:3], lo_hi[3:]
            pts += [np.array([x, y, z]) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
        for key in _COL.files:
            if key.startswith(f"{body}|hull"):
                pts += list(_COL[key])
        out[body] = np.array(pts)
    _SUB["chair_samples"] = out
    return out


# -- the simulation's hooks: wheel locks, the held test's forces ----------------------------------------------------------------
def _install(environment) -> None:
    base = environment.unwrapped
    scene = base.scene
    chair = scene["wheelchair"]
    jn, bn = list(chair.joint_names), list(chair.body_names)
    _SUB.update(chair=chair, jid={n: jn.index(n) for n in JOINTS}, bid={n: bn.index(n) for n in bn},
                origin=_np(scene.env_origins[0]), dt=float(base.step_dt))
    if getattr(scene, "_wheelchair_hook", False):
        return
    write = scene.write_data_to_sim

    def write_and_push(*a, **k):
        try:
            _substep()
        except Exception as e:                          # never take the simulation down
            _SUB["err"] = repr(e)
            if _DEBUG:
                raise
        return write(*a, **k)

    scene.write_data_to_sim = write_and_push
    scene._wheelchair_hook = True


def _set_drive(environment, joint: str, k: float, d: float, target: float) -> None:
    import torch

    from scenekit import set_joint_targets

    chair = _SUB["chair"]
    ids = [_SUB["jid"][joint]]
    chair.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[k]], dtype=torch.float32, device=chair.device),
                                             joint_ids=ids)
    chair.write_joint_damping_to_sim_index(damping=torch.tensor([[d]], dtype=torch.float32, device=chair.device),
                                           joint_ids=ids)
    set_joint_targets(environment, "wheelchair", {joint: target})


def _set_lock(environment, side: str, on: bool, axle_q: float = 0.0) -> None:
    """A lever past centre is pushed on to its stop (the over-centre snap) and holds its wheel's axle as the shoe does;
    short of centre it is pushed back off. Both by joint drives written here."""
    _SUB.setdefault("lock", {})[side] = on
    if on:
        _set_drive(environment, f"lock_joint_{side}", g.K_ON, g.LEVER_DAMP, Q_ON + g.ON_PRELOAD)
        _set_drive(environment, f"axle_{side}", g.AXLE_LOCK_K, g.AXLE_LOCK_D, axle_q)
    else:
        _set_drive(environment, f"lock_joint_{side}", g.K_OFF, g.LEVER_DAMP, -g.OFF_PRELOAD)
        _set_drive(environment, f"axle_{side}", 0.0, g.WHEEL_DAMP, 0.0)


def _substep() -> None:
    """The held test's pull or push: a horizontal force along the chair's heading, at the middle of a strap between the
    two push handles' grips."""
    S = _SUB
    f = S.get("force")
    chair = S["chair"]
    if f is None:
        if S.get("applied"):
            chair.permanent_wrench_composer.reset()
            S["applied"] = False
        return
    import torch

    _, mag = f
    i = S["bid"]["frame"]
    q = _t(chair.data.body_link_quat_w)[0, i]
    p = _t(chair.data.body_link_pos_w)[0, i]
    x, y, z, w = q[0], q[1], q[2], q[3]
    fwd = torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), torch.zeros_like(x)])
    fwd = fwd / torch.linalg.norm(fwd)
    Rm = torch.stack([torch.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)]),
                      torch.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)]),
                      torch.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)])])
    grip = torch.tensor(g.strap_point(), dtype=p.dtype, device=p.device)
    pos = p + Rm @ grip
    force = fwd * float(mag)
    chair.permanent_wrench_composer.set_forces_and_torques_index(
        forces=force.reshape(1, 1, 3), positions=pos.reshape(1, 1, 3), body_ids=[i], is_global=True)
    S["applied"] = True


@before_reset
def _restart(environment) -> None:
    """Every joint back to 0 (the reset moves the chair only by its root), both locks off, no test force."""
    from scenekit import set_joint_targets, write_joint_positions

    _install(environment)
    _SUB["force"] = None
    _substep()
    write_joint_positions(environment, "wheelchair", {j: 0.0 for j in JOINTS})
    for side in SIDES:
        _set_lock(environment, side, False)
    # the caster springs: preloaded by the chair's weight at the start (joint 0 is the stem at its unloaded height
    # less the preload)
    set_joint_targets(environment, "wheelchair", {f"spring_{s}": -g.SPRING_PRELOAD for s in SIDES})


TEST_PHASES = [("strap", -1.0), ("strap", 1.0)]                            # back, then forward


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(step=0, tier=0.0, pose={}, pose_tier="neither", levers={}, locked={s: False for s in SIDES},
                  tipped=False, lee_touch=False, touching=False, chair_touch_d=math.inf, left_ready=False,
                  still=0, prev_q=None, test=None, final=None, events=[], why=[], test_record=None, hubs={},
                  robot_lee_d=math.inf, chair_lee_d=math.inf)
    _SUB["force"] = None


_reset_state()


def _arms(environment) -> dict:
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints

    return {side: arm_joints(environment, side, ARM_JOINTS) for side in ("left", "right")}


def _poses(environment):
    """The chair frame's pose (p, R), every chair body's pose (4x4) and the hubs (world xy), env-relative."""
    chair = _SUB["chair"]
    o = _SUB["origin"]
    pos = _np(chair.data.body_link_pos_w)[0] - o
    quat = _np(chair.data.body_link_quat_w)[0]
    T = {}
    for n, i in _SUB["bid"].items():
        M = np.eye(4)
        M[:3, :3] = _rotation(quat[i])
        M[:3, 3] = pos[i]
        T[n] = M
    hubs = {s: T[f"wheel_{s}"][:3, 3].copy() for s in SIDES}
    return T["frame"][:3, 3], T["frame"][:3, :3], T, hubs


def _chair_dist(P, T) -> float:
    best = math.inf
    for body, M in T.items():
        if body in _body_shapes():
            best = min(best, dist_to_body(P, body, M))
    return best


def touching_chair(environment, T) -> float:
    """The robot's closest approach to the chair (m)."""
    P = robot_world_points(environment, near=lambda c: _chair_dist(c[None], T), margin=0.01)
    return _chair_dist(P, T) if len(P) else math.inf


def _lee_distances(environment, T) -> tuple[float, float]:
    """The robot's and the chair's closest approach to Mrs. Lee (m)."""
    P = robot_world_points(environment, near=lambda c: dist_to_body(c[None], "lee"), margin=0.01)
    d_robot = dist_to_body(P, "lee") if len(P) else math.inf
    d_chair = math.inf
    for body, S in _chair_samples().items():
        M = T.get(body)
        if M is None or not len(S):
            continue
        W = S @ M[:3, :3].T + M[:3, 3]
        W = W[W[:, 1] < g.BED_EDGE_Y + 0.15]
        if len(W):
            d_chair = min(d_chair, dist_to_body(W, "lee"))
    return d_robot, d_chair


@on_step
def _monitor(environment) -> None:
    from yam_arena.constants import READY

    S = _STATE
    S["step"] += 1
    p, R, T, hubs = _poses(environment)
    chair = _SUB["chair"]
    q = _np(chair.data.joint_pos)[0]
    # the wheel locks: over centre each way (with a 1 degree band), the axle held while on
    for side in SIDES:
        ql = float(q[_SUB["jid"][f"lock_joint_{side}"]])
        S["levers"][side] = math.degrees(ql + g.TH_OFF)
        on = _SUB["lock"].get(side, False)
        if not on and ql >= Q_C:
            _set_lock(environment, side, True, float(q[_SUB["jid"][f"axle_{side}"]]))
            S["events"].append((S["step"], f"lock {side} on"))
        elif on and ql < Q_C - g.TH_BAND:
            _set_lock(environment, side, False)
            S["events"].append((S["step"], f"lock {side} off"))
        S["locked"][side] = bool(_SUB["lock"][side])
    # the chair against the bed; tipping; Mrs. Lee untouched
    c = chair_pose(p, R)
    S["pose"], S["pose_tier"], S["hubs"] = c, pose_tier(c), {s: hubs[s][:2].round(4).tolist() for s in SIDES}
    if c["tilt"] > g.TIP and not S["tipped"]:
        S["tipped"] = True
        S["why"].append((S["step"], f"chair tipped {math.degrees(c['tilt']):.1f} deg"))
    if S["step"] % 2 == 0:
        d_robot, d_chair = _lee_distances(environment, T)
        S["robot_lee_d"], S["chair_lee_d"] = min(S["robot_lee_d"], d_robot), min(S["chair_lee_d"], d_chair)
        if min(d_robot, d_chair) < g.TOUCH and not S["lee_touch"]:
            S["lee_touch"] = True
            S["why"].append((S["step"], f"Mrs. Lee touched: robot {d_robot * 1000:.1f} mm, chair {d_chair * 1000:.1f} mm"))
    if S["step"] % 5 == 0 or S["test"] is not None:
        S["chair_touch_d"] = touching_chair(environment, T)
        S["touching"] = bool(S["chair_touch_d"] < g.TOUCH)
    # the end: both arms back at their ready pose and still for 1 s, after leaving it
    arms = _arms(environment)
    dev = max(float(np.abs(v - np.array(READY)).max()) for v in arms.values())
    qa = np.concatenate(list(arms.values()))
    if dev > g.READY_TOL:
        S["left_ready"] = True
    moving = S["prev_q"] is not None and float(np.abs(qa - S["prev_q"]).max()) / _SUB["dt"] > g.STILL_V
    S["prev_q"] = qa
    S["still"] = S["still"] + 1 if (dev <= g.READY_TOL and not moving) else 0
    if S["test"] is None and S["final"] is None and S["left_ready"] and S["still"] >= round(g.STILL_S / _SUB["dt"]):
        S["test"] = dict(k=0, start_pose=dict(c), start_tier=S["pose_tier"], levers=dict(S["levers"]),
                         locked=dict(S["locked"]), touching=S["touching"], moved={s: 0.0 for s in SIDES},
                         tick={}, phase=-1)
        S["events"].append((S["step"], "arms at ready: held test"))
    # the held test: each push handle pulled straight back to 20 N, then pushed straight forward, its hub watched
    t = S["test"]
    if t is not None:
        n_ramp, n_keep = round(g.RAMP_S / _SUB["dt"]), round(g.KEEP_S / _SUB["dt"])
        per = n_ramp + n_keep
        ph = t["k"] // per
        if ph < len(TEST_PHASES):
            _, sign = TEST_PHASES[ph]
            if not t["tick"]:
                t["tick"] = {sd: hubs[sd][:2].copy() for sd in SIDES}    # the floor ticks, plumb under the hubs
            t["phase"] = ph
            k = t["k"] % per
            mag = g.HOLD_F * min(1.0, (k + 1) / n_ramp)
            _SUB["force"] = ("strap", sign * mag)
            for sd in SIDES:
                t["moved"][sd] = max(t["moved"][sd], float(np.linalg.norm(hubs[sd][:2] - t["tick"][sd])))
            t["k"] += 1
            if _DEBUG and t["k"] % 3 == 0:
                print(f"[test] k {t['k']} force {_SUB['force']} moved_mm "
                      f"{ {sd: round(v * 1000, 1) for sd, v in t['moved'].items()} } axle "
                      f"{ {sd: round(float(q[_SUB['jid'][f'axle_{sd}']]), 4) for sd in SIDES} } locked {S['locked']} "
                      f"yaw {math.degrees(c['yaw']):.2f} tilt {math.degrees(c['tilt']):.2f} swivel "
                      f"{ {sd: round(math.degrees(float(q[_SUB['jid'][f'swivel_{sd}']])), 1) for sd in SIDES} }", flush=True)
            if all(v > g.HOLD_TOL for v in t["moved"].values()):  # both wheels have failed: let go
                t["k"] = len(TEST_PHASES) * per
                _SUB["force"] = None
        else:
            _SUB["force"] = None
            held = {s: bool(t["locked"][s] and S["locked"][s] and t["moved"][s] <= g.HOLD_TOL) for s in SIDES}
            S["final"] = 0.0 if (S["tipped"] or S["lee_touch"]) else tier(t["start_tier"], held, t["touching"])
            S["test_record"] = dict(pose=t["start_tier"], held=held, moved_mm={s: round(v * 1000, 2)
                                                                              for s, v in t["moved"].items()},
                                    levers_deg={s: round(v, 2) for s, v in t["levers"].items()},
                                    touching=t["touching"], score=S["final"],
                                    gaps_cm=[round(t["start_pose"]["gap_front"] * 100, 2),
                                             round(t["start_pose"]["gap_rear"] * 100, 2)],
                                    heading_deg=round(math.degrees(t["start_pose"]["heading"]), 2),
                                    hip_cm=round(t["start_pose"]["hip"] * 100, 2), step=S["step"])
            S["test"] = None
            S["events"].append((S["step"], f"held test: {held} -> {S['final']}"))
    # the score now: the test's, once it has run; before that, the end state as it stands (in the simulation a lever
    # past centre holds its axle, so it is held unless the test moves the chair)
    if S["final"] is not None:
        S["tier"] = S["final"]
    elif S["tipped"] or S["lee_touch"]:
        S["tier"] = 0.0
    else:
        S["tier"] = tier(S["pose_tier"], dict(S["locked"]), S["touching"])
    if _DEBUG and S["step"] % 20 == 0:
        print(f"[wc] step {S['step']} yaw {math.degrees(c['yaw']):.1f} gaps {c['gap_front'] * 100:.1f}/"
              f"{c['gap_rear'] * 100:.1f} head {math.degrees(c['heading']):.1f} hip {c['hip'] * 100:.1f} "
              f"pose {S['pose_tier']} levers {S['levers']} locked {S['locked']} touch {S['chair_touch_d'] * 1000:.0f}mm "
              f"lee {S['robot_lee_d'] * 1000:.0f}/{S['chair_lee_d'] * 1000:.0f}mm ready_dev {math.degrees(dev):.1f} "
              f"still {S['still']} tier {S['tier']} err {_SUB.get('err')}", flush=True)


def success(objects) -> bool:
    """True once the held test has run and scored 1.0 (it runs the first time both arms are back at their ready pose
    and still for 1 s after leaving it)."""
    return _STATE.get("final") == 1.0


def score(objects) -> float:
    """Read every control step (20 Hz) from the chair's pose, its wheels and wheel locks, the arms and the mannequin.

    - Gaps: from the mattress edge, square to it, to the front and rear ends of the chair's bed-side seat rail (42 cm
      apart). Heading: asin(gap difference / 42 cm). Hip offset: along the edge from the hip tick to the rail's front.
    - Alongside: facing the bed's foot, both gaps <= 25 cm, heading <= 10 degrees, hip offset <= 20 cm. Nearly
      alongside: facing the foot, gaps <= 40 cm, heading <= 30 degrees, hip offset <= 40 cm.
    - End: the run limit, or the first time both arms are back at their ready pose (every joint within 3 degrees) and
      still for 1 s after leaving it.
    - Held: a wheel's lock lever is forward past centre, and its hub stays within 1 cm of a tick plumb under it while
      a strap between the two push handles is pulled straight back to 20 N and then straight forward to 20 N (task
      code, once, at the end; in a real trial a luggage scale at 2 kg).
    - 0 for the run if the chair tips more than 10 degrees, or the chair or the robot touches Mrs. Lee.
    - Otherwise: 1.0 both wheels held and alongside; 0.5 both held and nearly alongside; 0.25 both held, neither; 0 if
      either wheel is not held, or the robot is still touching the chair at the end."""
    return float(_STATE.get("tier", 0.0))
