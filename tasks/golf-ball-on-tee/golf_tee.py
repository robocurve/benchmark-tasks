"""Push the tee into the rubber holder, then balance the golf ball on the tee.

A rubber tee holder stands on the table. A wooden tee and a golf ball lie on the table beside it. The robot pushes the
tee into the holder so it stands upright with its top at a playable driver height, then sets the ball on the tee's
cup so it stays there.

Model: the holder's rubber grip is four pads on radial sliders, sprung inward; the tee is held only by the pads'
squeeze and friction (no rule fixes it). The tee's cup is a ring of beads the ball rests on. All bodies rigid.
Frame: x forward from the arm bases, y left, z up, metres. The tee's body origin is its tip, z toward the head.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402  (hooks yam_arena so multi-shape bodies are simulated)
import layout as L  # noqa: E402
from bodykit import Body, BoxShape, CylShape, SphereShape  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

bodykit.allow_matrix_grips()

SCENE = "station"
TOP = L.TOP
SCORE_OVER = "end"
CONTROL_HZ = 20.0

RUBBER_C, WOOD_C, BALL_C, PAD_C = (0.08, 0.08, 0.08), (0.93, 0.90, 0.80), (0.97, 0.97, 0.97), (0.12, 0.12, 0.12)


def _ring(o: float, t: float, z: float) -> list:
    """Four boxes, t tall centred at height z, around the square opening the pads move in (outer half-width o)."""
    h = L.HOLE_HALF
    boxes = [((-o, -o), (o, -h)), ((-o, h), (o, o)), ((-o, -h), (-h, h)), ((h, -h), (o, h))]
    return [BoxShape(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, z), (b[0] - a[0], b[1] - a[1], t), RUBBER_C)
            for a, b in boxes]


def _holder() -> Body:
    """The rubber tee holder: a tube on a flat disc, its collision as square rings around the pads' opening, drawn
    round. The bore runs through to the table."""
    shapes = _ring(L.TUBE_HALF, L.HOLDER_H, L.HOLDER_H / 2) + _ring(L.DISC_R / math.sqrt(2), L.DISC_T, L.DISC_T / 2)
    shapes += [CylShape((0.0, 0.0, L.DISC_T / 2), L.DISC_R, L.DISC_T, RUBBER_C, collide=False),
               CylShape((0.0, 0.0, L.HOLDER_H / 2), L.TUBE_HALF, L.HOLDER_H, RUBBER_C, collide=False)]
    return Body.build("holder", pos=(L.HOLE[0], L.HOLE[1], TOP), mass=0.05, shapes=shapes, color=RUBBER_C,
                      kinematic=True, friction=(0.8, 0.7))


PADS = {"pad_px": ((1, 0), "X"), "pad_nx": ((-1, 0), "X"), "pad_py": ((0, 1), "Y"), "pad_ny": ((0, -1), "Y")}
PAD_Z = L.HOLDER_TOP - 0.002 - L.PAD_H / 2


def _pad_pos(name: str) -> tuple:
    (ux, uy), _ = PADS[name]
    r = L.PAD_IN + L.PAD_DEPTH / 2
    return (L.HOLE[0] + ux * r, L.HOLE[1] + uy * r, PAD_Z)


def _pad(name: str) -> Body:
    (ux, uy), _ = PADS[name]
    size = (L.PAD_DEPTH, L.PAD_W, L.PAD_H) if ux else (L.PAD_W, L.PAD_DEPTH, L.PAD_H)
    return Body.build(name, pos=_pad_pos(name), mass=L.PAD_MASS, shapes=[BoxShape((0.0, 0.0, 0.0), size, PAD_C)],
                      color=PAD_C, friction=L.PAD_MU, ignore=("holder",), solver_iterations=32)


def _tee_shapes() -> list:
    """Tip cone, shank, tapered neck and head, as stacked cylinders, plus the cup's rim as a ring of beads."""
    n = L.TEE_L - 0.01185  # the shank ends 11.85 mm below the head's top
    segs = [(0.0, 0.003, 0.0008), (0.003, 0.006, 0.0016), (0.006, 0.009, 0.0021), (0.009, n, L.SHANK_R),
            (n, n + 0.004, 0.0032), (n + 0.004, n + 0.008, 0.0042), (n + 0.008, L.TEE_L, L.HEAD_R)]
    shapes = [CylShape((0.0, 0.0, (a + b) / 2), r, b - a, WOOD_C) for a, b, r in segs]
    for k in range(10):
        ph = 2 * math.pi * k / 10
        shapes.append(SphereShape((L.CUP_RING_R * math.cos(ph), L.CUP_RING_R * math.sin(ph), L.TEE_L), L.CUP_BEAD_R,
                                  WOOD_C))
    return shapes


TEE_Q0 = (-math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))  # lying: body z along world +y (tip toward -y)
TEE_POS0 = (L.TEE_START_TIP[0], L.TEE_START_TIP[1], TOP + L.HEAD_R + 0.0008)
BALL_POS0 = (L.BALL_START[0], L.BALL_START[1], TOP + L.BALL_R + 0.0005)


def _build() -> list:
    objs = [_holder()] + [_pad(n) for n in PADS]
    objs.append(Body.build("tee", pos=TEE_POS0, mass=L.TEE_MASS, shapes=_tee_shapes(), color=WOOD_C,
                           friction=L.TEE_MU, start_xyzw=TEE_Q0, solver_iterations=32, max_ang_vel=200.0))
    objs.append(Body.build("ball", pos=BALL_POS0, mass=L.BALL_MASS,
                           shapes=[SphereShape((0.0, 0.0, 0.0), L.BALL_R, BALL_C)], color=BALL_C, friction=L.BALL_MU,
                           damping=(0.05, 0.2), solver_iterations=32))
    return objs


OBJECTS = _build()
NAMES = [o.name for o in OBJECTS]
LOOSE = ("tee", "ball")

for _n, ((_ux, _uy), _ax) in PADS.items():
    _s = 1.0 if (_ux + _uy) > 0 else -1.0  # outward along the axis
    bodykit.slider(f"{_n}_grip", "holder", _n, anchor=_pad_pos(_n), axis=_ax,
                   limits=(0.0, L.PAD_TRAVEL) if _s > 0 else (-L.PAD_TRAVEL, 0.0),
                   stiffness=L.PAD_K, damping=L.PAD_DAMP, target=-_s * L.PAD_PRELOAD, max_force=50.0)

# -- episode state --------------------------------------------------------------------------------------------------
_EP: dict = {}
_DBG = open(os.environ["GOLF_DEBUG"], "w") if os.environ.get("GOLF_DEBUG") else None


def _blank() -> None:
    start = _EP.get("start")
    _EP.clear()
    _EP.update(step=0, hands=None, report={}, now=None, start=start)


_blank()


@bodykit.on_reset
def _reset() -> None:
    _blank()


def finger_points(p, r, half: float) -> list[np.ndarray]:
    """Points on both fingers of a gripper (TCP at p, rotation r, pads half m from the centre line)."""
    pts = []
    for side in (-1.0, 1.0):
        for x, y, z in ((half + 0.002, 0.0, 0.013), (half + 0.002, 0.0077, 0.008), (half + 0.002, -0.0077, 0.008),
                        (half + 0.004, 0.0, -0.005), (half + 0.008, 0.0, -0.030), (half + 0.015, 0.0, -0.055)):
            pts.append(np.asarray(p, float) + np.asarray(r, float) @ np.array([side * x, y, z]))
    return pts


def _hand_points(base) -> list:
    from bodykit import as_numpy
    from yam_arena import ik, station

    out = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        qj = as_numpy(robot.data.joint_pos)[0]
        names = list(robot.data.joint_names)
        q6 = np.array([qj[names.index(f"joint{i}")] for i in range(1, 7)])
        half = abs(float(qj[names.index("joint7")]))
        p, r = ik.tcp_pose(q6, station.BASES[arm])
        out += finger_points(p, r, half)
    return out


@bodykit.on_step
def _watch(env) -> None:
    from yam_arena import scene as _scene

    st = _scene.object_states(env, NAMES)
    _EP["now"] = st
    try:
        pts = _hand_points(env.unwrapped)
        _EP["hands"] = pts if all(np.all(np.isfinite(p)) for p in pts) else None
    except Exception:  # noqa: BLE001  (fail closed: no hand points means the hands do not count as away)
        _EP["hands"] = None
    if _DBG is not None and _EP["step"] % 10 == 0:
        tee = tee_state(st["tee"])
        b = st["ball"]
        _DBG.write(f"{_EP['step']:4d} tee top={tee['top_above_holder_m']:.4f} tilt={tee['tilt_deg']:.1f} "
                   f"tip_off={tee['tip_off_axis_m']:.4f} tip_z={tee['tip_below_holder_top_m']:.4f} "
                   f"ball=({b.pos[0]:.4f},{b.pos[1]:.4f},{b.pos[2]:.4f}) "
                   f"pads=" + ",".join(f"{np.linalg.norm(np.asarray(st[n].pos[:2]) - L.HOLE) * 1000:.2f}" for n in PADS)
                   + "\n")
        _DBG.flush()
    _EP["step"] += 1


# -- scoring ----------------------------------------------------------------------------------------------------------
def tee_points(s) -> tuple[np.ndarray, np.ndarray]:
    """The tee's tip and the centre of its head's top face (world)."""
    r = s.rotation()
    p = np.asarray(s.pos, float)
    return p, p + r @ np.array([0.0, 0.0, L.TEE_L])


def tee_state(s) -> dict:
    tip, top = tee_points(s)
    axis = s.rotation()[:, 2]
    return {"tilt_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(axis[2]))))), 2),
            "top_above_holder_m": round(float(top[2] - L.HOLDER_TOP), 4),
            "tip_off_axis_m": round(float(np.linalg.norm(tip[:2] - np.asarray(L.HOLE))), 4),
            "tip_below_holder_top_m": round(float(L.HOLDER_TOP - tip[2]), 4)}


def _seg_dist(p, a, b) -> float:
    ab = b - a
    u = max(0.0, min(1.0, float((p - a) @ ab / (ab @ ab))))
    return float(np.linalg.norm(p - (a + u * ab)))


def report(objects) -> dict:
    tee, ball = objects["tee"], objects["ball"]
    t = tee_state(tee)
    tip, top = tee_points(tee)
    c = np.asarray(ball.pos, float)
    pts = _EP.get("hands")
    hands = (min(min(np.linalg.norm(p - c) - L.BALL_R, _seg_dist(p, tip, top) - L.HEAD_R) for p in pts)
             if pts else None)
    tee_ok = (t["tilt_deg"] <= L.UPRIGHT_DEG and L.HEIGHT_RANGE[0] <= t["top_above_holder_m"] <= L.HEIGHT_RANGE[1]
              and t["tip_off_axis_m"] <= L.IN_HOLDER_M and t["tip_below_holder_top_m"] > 0.0)
    cup = top
    off = float(np.linalg.norm(c[:2] - cup[:2]))
    rise = float(c[2] - cup[2])
    speed = float(np.linalg.norm(np.asarray(ball.lin_vel, float)))
    ball_ok = off <= L.BALL_ON_M and 0.015 <= rise <= 0.025 and speed <= L.BALL_REST
    away = hands is not None and hands >= L.HANDS_CLEAR
    score = 0.0
    if tee_ok and away:
        score = 1.0 if ball_ok else L.PART_SCORE
    return {"score": score, "tee": t, "tee_ok": tee_ok, "ball_off_axis_m": round(off, 4),
            "ball_centre_above_cup_m": round(rise, 4), "ball_speed": round(speed, 4), "ball_on_tee": ball_ok,
            "hands_clear_m": None if hands is None else round(hands, 3), "hands_away": away}


def score(objects) -> float:
    """1.0: tee upright within 5 deg, tip in the holder, top 38-51 mm above the holder's top; ball resting on its cup;
    hands at least 3 cm away. 0.5: the tee so, the ball not on it. 0 otherwise."""
    rep = report(objects)
    _EP["report"] = rep
    if _DBG is not None and _EP["step"] % 20 == 0:
        _DBG.write(f"score report {_EP['step']} {rep}\n")
        _DBG.flush()
    return float(rep["score"])


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    _, top = tee_points(objects["tee"])
    return float(np.linalg.norm(np.asarray(objects["ball"].pos, float) - top))


# -- the start (fixed layout) ----------------------------------------------------------------------------------------
@bodykit.on_reset_env
def _lay_out(emb, seed):
    import torch
    from yam_arena import inspect_embodiment as ie
    from yam_arena import scene as _scene

    base = emb._base
    for name, pos, quat in (("tee", TEE_POS0, TEE_Q0), ("ball", BALL_POS0, (0.0, 0.0, 0.0, 1.0))):
        obj = base.scene[name]
        pose = obj.data.default_root_pose.torch.clone()
        pose[:, :3] = torch.tensor(pos, dtype=pose.dtype, device=pose.device) + base.scene.env_origins
        pose[:, 3:7] = torch.tensor(quat, dtype=pose.dtype, device=pose.device)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(obj.data.default_root_vel.torch))
    for _ in range(max(1, round(max(1.0, getattr(ie, "SETTLE_S", 0.5)) / base.step_dt))):
        emb._env.step(emb._ready)
    for _ in range(getattr(ie, "RESET_RENDERS", 3)):
        base.sim.render()
    st = _scene.object_states(emb._env, NAMES)
    _EP["start"] = {n: st[n] for n in LOOSE}
    _blank()
    if _DBG is not None:
        t = st["tee"]
        _DBG.write(f"seed {seed}: tee=({t.pos[0]:.4f},{t.pos[1]:.4f},{t.pos[2]:.4f}) tilt={tee_state(t)['tilt_deg']} "
                   f"ball=({st['ball'].pos[0]:.4f},{st['ball'].pos[1]:.4f},{st['ball'].pos[2]:.4f})\n")
    return emb._observation()


# -- the scripted demo (reads the tee's and ball's poses from the simulator; moves them only through the grippers) ----
VARIANT = os.environ.get("DEMO_VARIANT", "")  # "" (full), careless (ball let go 9 mm off the axis), nothing
PRESS = -0.001  # the push stops with the tee's tip 1 mm over the table (pressing past it tilts the tee)
CARELESS_OFF = 0.009
CUP_RISE = math.sqrt((L.BALL_R + L.CUP_BEAD_R) ** 2 - L.CUP_RING_R ** 2)  # ball centre over the bead ring
SHANK_AT = 0.058  # the right hand pinches the lying tee's shank this far from its tip (below the neck)
PRESENT = np.array([L.HOLE[0], L.HOLE[1], L.HOLDER_TOP + 0.02])  # the tee's tip, upright over the holder
START_IN = 0.033  # the right hand starts the tee this far in (tip 5 mm over the table); the left presses it home
LOOKS = 3
L_TIP = 0.013  # the closed fingertips reach this far past the TCP
DOWN = np.column_stack([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])  # tool straight down, pinch along x
DOWN_FLIP = np.column_stack([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
Z = np.array([0.0, 0.0, 1.0])


def _wp(sec, pos, f, label, rot) -> Waypoint:
    return Waypoint(float(sec), tuple(float(v) for v in pos), float(f), label, np.asarray(rot, float))


def ready_pose(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def park(arm: str, sec: float, label: str = "back to the ready pose") -> Waypoint:
    p, r = ready_pose(arm)
    return _wp(sec, p, FINGER_OPEN, label, r)


def _dur(ws) -> float:
    return sum(w.duration for w in ws)


def _pad(plans: dict) -> None:
    tl, tr = _dur(plans["left"]), _dur(plans["right"])
    if tl < tr - 1e-9:
        plans["left"].append(park("left", tr - tl, "wait"))
    elif tr < tl - 1e-9:
        plans["right"].append(park("right", tl - tr, "wait"))


def _best(arm: str, tcp, rots) -> np.ndarray:
    """Of the given grip rotations, the one the arm reaches best from the ready pose."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    best = None
    for rot in rots:
        q, pe, re = ik.solve(tcp, ik.matrix_to_quat_xyzw(rot), READY, base_pos=station.BASES[arm], iters=300)
        key = (round(pe + 0.05 * re, 3), abs(float(q[5])))
        if best is None or key < best[0]:
            best = (key, rot)
    return best[1]


def _align(a, b) -> np.ndarray:
    """The smallest rotation taking unit vector a to unit vector b."""
    a, b = np.asarray(a, float) / np.linalg.norm(a), np.asarray(b, float) / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1.0 + c)


def stage0(st) -> dict:
    """Right hand: pinch the lying tee's shank from above, lift it, turn it tip down over the holder, start it 33 mm
    into the holder, let go and back away. Left hand waits."""
    tee = st["tee"]
    r0 = tee.rotation()
    p0 = np.asarray(tee.pos, float)
    at = p0 + r0 @ np.array([0.0, 0.0, SHANK_AT])
    tcp_p = np.array([at[0], at[1], TOP + 0.0135])  # fingertips 0.5 mm over the table
    rot_p = _best("right", tcp_p, (DOWN, DOWN_FLIP))
    g_r, g_p = rot_p.T @ r0, rot_p.T @ (p0 - tcp_p)  # the grasp: the tee's pose in the hand's frame
    rot_v = (_align(r0[:, 2], Z) @ r0) @ g_r.T
    tcp_v = PRESENT - rot_v @ g_p
    tcp_in = tcp_v - (PRESENT[2] - (L.HOLDER_TOP - START_IN)) * Z
    back = tcp_in - 0.05 * rot_v[:, 2]
    right = [_wp(2.5, tcp_p + 0.06 * Z, -0.015, "over the tee", rot_p),
             _wp(2.0, tcp_p, -0.015, "down to the tee's shank", rot_p),
             _wp(0.5, tcp_p, -0.015, "down to the tee's shank", rot_p),
             _wp(0.8, tcp_p, 0.0, "pinch the shank", rot_p),
             _wp(0.4, tcp_p, 0.0, "pinch the shank", rot_p),
             _wp(1.5, tcp_p + 0.10 * Z, 0.0, "lift the tee", rot_p),
             _wp(3.0, tcp_v, 0.0, "turn it tip down over the holder", rot_v),
             _wp(0.5, tcp_v, 0.0, "line up", rot_v),
             _wp(2.0, tcp_in, 0.0, "start it into the holder", rot_v),
             _wp(0.5, tcp_in, 0.0, "start it into the holder", rot_v),
             _wp(0.8, tcp_in, -0.015, "let go of the tee", rot_v),
             _wp(1.2, back, -0.015, "back off", rot_v),
             _wp(1.0, back + 0.05 * Z, -0.015, "back off up", rot_v)]
    return {"right": right, "left": [park("left", _dur(right), "wait")]}


def stage1(st) -> dict:
    """Left hand: fingers closed, press the started tee's head straight down until its tip meets the table, then lift
    away. Right hand then (the arms would meet if both worked at once): pick the ball up by its equator and hold it
    beside the holder."""
    tip, top = tee_points(st["tee"])
    tcp_t = np.array([top[0], top[1], top[2] + L_TIP - 0.001])  # closed fingertips 1 mm into the head's top
    rot_l = _best("left", tcp_t, (DOWN, DOWN_FLIP))
    push = tcp_t - (tip[2] - (L.TOP - PRESS)) * Z
    left = [_wp(2.5, tcp_t + 0.05 * Z, 0.0, "over the tee's head", rot_l),
            _wp(1.0, tcp_t + 0.05 * Z, 0.0, "over the tee's head", rot_l),
            _wp(2.0, tcp_t + 0.003 * Z, 0.0, "fingertips onto the head", rot_l),
            _wp(3.0, push, 0.0, "push the tee home", rot_l),
            _wp(0.8, push, 0.0, "push the tee home", rot_l),
            _wp(0.8, push, -0.015, "open, so the fingertips let go of the head", rot_l),
            _wp(1.5, push + 0.10 * Z, -0.015, "back off up", rot_l),
            park("left", 2.0)]
    c = np.asarray(st["ball"].pos, float)
    tcp_b = c + np.array([0.0, 0.0, 0.0085])  # the equator 8.5 mm past the TCP: fingertips 4.5 mm under it
    rot_r = _best("right", tcp_b, (DOWN, DOWN_FLIP))
    hold = np.array([L.HOLE[0], L.HOLE[1] - 0.09, L.HOLDER_TOP + 0.14]) + (tcp_b - c)
    t_clear = _dur(left[:-1])  # the right hand waits until the left has pressed the tee home and lifted away
    right = [park("right", t_clear, "wait"),
             _wp(2.5, tcp_b + 0.07 * Z, -0.040, "over the ball", rot_r),
             _wp(1.0, tcp_b + 0.07 * Z, -0.040, "over the ball", rot_r),
             _wp(2.5, tcp_b, -0.040, "down to the ball", rot_r),
             _wp(0.8, tcp_b, -0.040, "down to the ball", rot_r),
             _wp(1.0, tcp_b, 0.0, "pinch the ball", rot_r),
             _wp(0.5, tcp_b, 0.0, "pinch the ball", rot_r),
             _wp(2.0, tcp_b + 0.10 * Z, 0.0, "lift the ball", rot_r),
             _wp(2.0, hold, 0.0, "hold the ball beside the holder", rot_r)]
    plans = {"left": left, "right": right}
    _pad(plans)
    _EP["hold"] = (hold, rot_r)
    return plans


def stage2(st) -> dict:
    """Right hand: set the ball on the cup, centred over the tee's axis; open slowly; lift straight up."""
    _, top = tee_points(st["tee"])
    hold, rot_r = _EP["hold"]
    c = np.asarray(st["ball"].pos, float)
    target = np.array([top[0], top[1], top[2] + CUP_RISE + 0.001])
    if VARIANT == "careless":
        target = target + np.array([CARELESS_OFF, 0.0, 0.0])
    tcp_set = hold + (target - c)
    right = [_wp(2.5, tcp_set + 0.04 * Z, 0.0, "over the tee", rot_r),
             _wp(1.0, tcp_set + 0.04 * Z, 0.0, "over the tee", rot_r),
             _wp(2.5, tcp_set, 0.0, "lower the ball onto the cup", rot_r),
             _wp(0.8, tcp_set, 0.0, "lower the ball onto the cup", rot_r),
             _wp(1.5, tcp_set, -0.026, "open slowly", rot_r),
             _wp(1.0, tcp_set, -0.026, "open slowly", rot_r),
             _wp(2.0, tcp_set + 0.08 * Z, -0.026, "back off up", rot_r),
             park("right", 2.0)]
    return {"left": [park("left", 1.0, "wait")], "right": right}


def plan_stage(stage: int, img, module) -> dict:
    if VARIANT == "nothing":
        return {}
    st = _EP["start"] if stage == 0 else _EP["now"]
    t = tee_state(st["tee"])
    b = st["ball"].pos
    print(f"golfdemo: stage {stage}: tee top {t['top_above_holder_m']:.4f} tilt {t['tilt_deg']:.1f} tip off "
          f"{t['tip_off_axis_m']:.4f}; ball ({b[0]:.4f}, {b[1]:.4f}, {b[2]:.4f})", flush=True)
    return (stage0, stage1, stage2)[stage](st)


def placeholder_plans() -> dict:
    return {arm: [park(arm, 1.0, "look")] for arm in ("left", "right")}


def demo():
    return placeholder_plans()


import golfdemo  # noqa: E402  (the demo's control hooks for yam_scripted)

golfdemo.install()
golfdemo_task = True
