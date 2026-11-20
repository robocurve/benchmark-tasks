"""On the station table: an AmScope B120C compound microscope, eyepieces toward the arms, powered off, its stage fully
lowered with the 4x objective in the path and the stage's X and Y scales at their centred readings. The stage carries a
metal template (back strip and fixed jaw on the left) and a spring caliper on the right, closed; its lever knob sits at
the stage's right edge. To the robot's left, an angled bench slide holder holds one prepared slide in its middle slot,
label end up, coverslip toward the robot. Load the slide into the holder: coverslip up, seated against the template's
back edge and the jaw, the caliper's arm lowered gently onto its right end.

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

from scenekit import UsdBody, before_reset, on_reset, on_step, physics  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Load the prepared slide into the microscope's slide holder, coverslip up and seated against the holder's "
               "back edge and jaw, so the specimen is under the objective. Handle the slide only by its edges and label "
               "end.")

physics(400)                        # 20 PhysX substeps per 20 Hz control step (thin glass)

M0 = (g.AXIS_X, g.AXIS_Y, g.STAGE_Z)
METAL = (0.3, 0.3)                  # glass on painted metal / steel (min-combined: rules glass contacts)
FRAME = UsdBody("frame", usd="assets/frame.usda", pos=M0, size=(0.2, 0.17, 0.2), friction=(0.5, 0.5))
OPTICS = UsdBody("optics", usd="assets/optics.usda", pos=M0, size=(0.1, 0.1, 0.1), friction=(0.3, 0.3),
                 friction_min=True)
STAGE = UsdBody("stage", usd="assets/stage.usda", pos=M0, size=(0.125, 0.116, 0.012), articulation=True,
                joint_pos=(("stage_y", 0.0), ("stage_x", 0.0), ("caliper", g.CAL_SHUT)),
                drives=(("stage_y", g.STAGE_K, g.STAGE_DAMP), ("stage_x", g.STAGE_K, g.STAGE_DAMP),
                        ("caliper", g.CAL_K, g.CAL_D)),
                friction=METAL, friction_min=True, solver_iters=32)
_hp = g.holder_slot_floor()
HOLDER = UsdBody("holder", usd="assets/holder.usda", pos=tuple(float(v) for v in _hp),
                 rot_xyzw=(0.0, 0.0, math.sin(g.HOLDER_YAW / 2), math.cos(g.HOLDER_YAW / 2)), size=g.HOLDER_SIZE,
                 friction=(0.3, 0.3), friction_min=True)


def _quat_xyzw(R) -> tuple:
    w = math.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return tuple(float(v) for v in ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
                                    (R[1, 0] - R[0, 1]) / (4 * w), w))


_sp, _sR = g.slide_start()
SLIDE = UsdBody("slide", usd="assets/slide.usda", pos=tuple(float(v) for v in _sp), rot_xyzw=_quat_xyzw(_sR),
                size=(g.SLIDE_L, g.SLIDE_W, g.SLIDE_T), mass=g.SLIDE_MASS, friction=(g.PAD_MU, g.PAD_MU),
                solver_iters=32)
OBJECTS = [FRAME, OPTICS, STAGE, HOLDER, SLIDE]

HAND_LINKS = ("gripper", "tip_left", "tip_right")
_STATE: dict = {}
_VIEWS: dict = {}
_DEBUG = bool(os.environ.get("MS_DEBUG"))
GRIP_LIMIT = 10.0                   # N: the fingers' force limit in this task (the glass)


def _rotation(xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), np.float64)
    return np.asarray(x, np.float64)


@before_reset
def _restart(environment) -> None:
    """Stage centred, caliper closed (on its stop: no slide in the seat), the caliper's spring drive preloaded; the
    fingers' force limited."""
    import torch

    from scenekit import set_joint_targets, write_joint_positions

    write_joint_positions(environment, "stage", {"stage_y": 0.0, "stage_x": 0.0, "caliper": g.CAL_SHUT})
    set_joint_targets(environment, "stage", {"stage_y": 0.0, "stage_x": 0.0, "caliper": g.CAL_PRELOAD})
    for side in ("left", "right"):
        robot = environment.unwrapped.scene[f"{side}_robot"]
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        lim = torch.full((1, len(ids)), GRIP_LIMIT, dtype=torch.float32, device=robot.device)
        robot.write_joint_effort_limit_to_sim_index(limits=lim, joint_ids=ids)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(last_pose=None, stage_rest=[0.0, 0.0], why=[], zero=False, step=0, ref_z=None, rest=0, start_q=None, touch_hist=[], rest_tip=0, landed=False, landings=[],
                  hard=False, landing=None, rung=0.0, rungs=[], flat=False, held=False, seated=False, under=False,
                  clear=False, gap=1.0, grip_f=0.0, face=[], corners=np.zeros(4), back=(9, 9), jaw=(9, 9),
                  tip_slide=0.0, tip_hand=0.0, cal=0.0, carriage=(0.0, 0.0), optic_f=0.0, up=1.0, still=False)
    _VIEWS.clear()


_reset_state()


def _fail(why: str) -> None:
    if why not in _STATE["why"]:
        _STATE["why"].append(why)
    _STATE["zero"] = True


def _views(environment) -> dict:
    """PhysX contact views: the slide against the grippers (detailed: points and normals), the optics against the
    grippers and the slide, the caliper against the slide and the grippers."""
    if not _VIEWS:
        from yam_arena.robot import asset_link_paths

        import yam_arena.constants as C

        psv = environment.unwrapped.sim.physics_sim_view
        ns = "/World/envs/env_*"
        lp = asset_link_paths()
        hands = [f"{ns}/{arm}{lp[link]}" for arm in ("LeftArm", "RightArm") for link in HAND_LINKS]
        _VIEWS["slide"] = psv.create_rigid_contact_view(f"{ns}/slide", filter_patterns=hands, max_contact_data_count=512)
        _VIEWS["optics"] = psv.create_rigid_contact_view(f"{ns}/optics", filter_patterns=hands + [f"{ns}/slide"])
        _VIEWS["caliper"] = psv.create_rigid_contact_view(f"{ns}/stage/caliper",
                                                          filter_patterns=[f"{ns}/slide"] + hands)
        _VIEWS["dt"] = float(C.PHYSICS_DT)
    return _VIEWS


def _slide_contacts(V, origin, o, R):
    """Gripper contacts on the slide carrying force: (total force, [(point slide frame, normal slide frame, force)])."""
    f, pts, nrm, sep, cnt, start = (_np(a) for a in V["slide"].get_contact_data(V["dt"]))
    f, pts, nrm = f.reshape(-1), pts.reshape(-1, 3), nrm.reshape(-1, 3)
    cnt, start = cnt.reshape(-1).astype(int), start.reshape(-1).astype(int)
    out, total = [], 0.0
    for c, s in zip(cnt, start):
        for k in range(s, s + c):
            if abs(f[k]) > g.TOUCH_F:
                total += abs(f[k])
                out.append((R.T @ (pts[k] - origin - o), R.T @ nrm[k], abs(f[k])))
    return total, out


def _face_violation(contacts) -> str | None:
    """Rule 1: a gripper on the coverslip, or on either face of the slide outside the frosted label end: a contact
    whose normal is within 45 degrees of the face normal and whose point lies on the face more than FACE_INSET (0.3 mm)
    inside the slide's outline. A contact on the arris of a 1 mm edge (a rounded fingertip pushing the edge) is an edge
    contact; a pinch across the faces presses its pads on the faces' interior and is caught."""
    hl, hw = g.SLIDE_L / 2, g.SLIDE_W / 2
    cs = g.COVER_S / 2 + 0.0002
    for p, n, _ in contacts:
        on_cover = abs(p[0]) <= cs and abs(p[1]) <= cs and p[2] >= g.SLIDE_T / 2 - 0.0001
        if on_cover:
            return "touched the coverslip"
        inside = abs(p[0]) < hl - g.FACE_INSET and abs(p[1]) < hw - g.FACE_INSET
        if abs(n[2]) >= g.FACE_COS and inside and p[0] < hl - g.LABEL_L:
            return "touched a face of the slide outside the label end"
    return None


def caliper_parts(phi: float, carriage) -> list:
    """The caliper's arm, tip, lever link, hub and knob as boxes (rotation, centre, size), station frame."""
    c, s = math.cos(phi), math.sin(phi)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    out = []
    zc, zs = (g.ARM_Z[0] + g.ARM_Z[1]) / 2, g.ARM_Z[1] - g.ARM_Z[0]
    for (xr, yr) in (g.ARM_A, g.ARM_TIP, g.LEVER_LINK):
        ctr = g.caliper_point(((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, zc), phi, carriage)
        out.append((Rz, g.to_world(ctr), np.array([xr[1] - xr[0], yr[1] - yr[0], zs])))
    kz = (g.KNOB_Z[0] + g.KNOB_Z[1]) / 2
    k = g.caliper_point((*g.KNOB, kz), phi, carriage)
    out.append((Rz, g.to_world(k), np.array([2 * g.KNOB_R, 2 * g.KNOB_R, g.KNOB_Z[1] - g.KNOB_Z[0]])))
    return out


def _edge_points(size, step: float = 0.005) -> np.ndarray:
    half, pts = np.asarray(size) / 2, []
    for a in range(3):
        b, cc = [i for i in range(3) if i != a]
        t = np.linspace(-half[a], half[a], max(2, int(np.ceil(size[a] / step)) + 1))
        for sb in (-1.0, 1.0):
            for sc in (-1.0, 1.0):
                p = np.zeros((len(t), 3))
                p[:, a], p[:, b], p[:, cc] = t, sb * half[b], sc * half[cc]
                pts.append(p)
    return np.concatenate(pts)


def _hand_gap(environment, boxes) -> float:
    """The smallest gap from either gripper (its body's and fingers' collision hulls) to these boxes."""
    from yam_arena.analysis import box_signed_distance, world_hulls

    from scenekit import robot_link_poses

    hulls = []
    for poses in robot_link_poses(environment).values():
        hulls.extend(h for n, h in world_hulls(poses) if n in HAND_LINKS)
    best = 1.0
    verts = np.concatenate([h.vertices for h in hulls])
    for R, centre, size in boxes:
        best = min(best, float(box_signed_distance(verts, centre, R, size).min()))
        edges = centre + _edge_points(size) @ R.T
        lo, hi = edges.min(0) - 0.04, edges.max(0) + 0.04
        for h in hulls:
            if np.all(h.vertices.max(0) >= lo) and np.all(h.vertices.min(0) <= hi):
                p = h.planes
                best = min(best, float((edges @ p[:, :3].T - p[:, 3]).max(axis=1).min()))
    return best


def _stage_friction(environment, carriage) -> None:
    """Stick-slip friction of the stage's two carriage axes: each is held by a stiff spring (STAGE_K) to a rest point;
    when the spring force passes the breakaway force (STAGE_BREAK) the rest point is dragged along (it slips), so a
    shove harder than the breakaway moves the stage and it stays where it is left."""
    from scenekit import set_joint_targets

    rest = _STATE["stage_rest"]
    lim = g.STAGE_BREAK / g.STAGE_K
    moved = False
    for i, q in enumerate(carriage):
        if abs(q - rest[i]) > lim:
            rest[i] = q - math.copysign(lim, q - rest[i])
            moved = True
    if moved:
        set_joint_targets(environment, "stage", {"stage_y": rest[0], "stage_x": rest[1]})


def _final_landing(hist):
    """The landing that just ended in the arm resting on the slide: (mean tip speed m/s, gripper on the caliper when
    the tip met the slide, step). hist: per-step (step, tip position, tip touching the slide, gripper on the caliper).
    The final approach is the tip's last continuous motion (steps moving >= TIP_STILL) before it stopped; the speed is
    over its last LAST_MM of travel (or all of it if shorter): path over time. The meeting is the first step of that
    window with the tip on the slide (its first step if the tip was already touching, e.g. after a brush)."""
    pos = [h[1] for h in hist]
    k = len(hist) - 1
    while k > 0 and np.linalg.norm(pos[k] - pos[k - 1]) < g.TIP_STILL:
        k -= 1
    stop = k
    if stop == 0:
        return None
    path, j = 0.0, stop
    while j > 0 and path < g.LAST_MM:
        d = float(np.linalg.norm(pos[j] - pos[j - 1]))
        if d < g.TIP_STILL:
            break
        path += d
        j -= 1
    dt = (hist[stop][0] - hist[j][0]) * 0.05
    speed = path / dt if dt > 0 else 0.0
    meet = next((i for i in range(j, len(hist)) if hist[i][2]), j)
    return speed, bool(hist[meet][3]), hist[meet][0]


@on_step
def _monitor(environment) -> None:
    from yam_arena.scene import object_states

    S = _STATE
    S["step"] += 1
    V = _views(environment)
    origin = _np(environment.unwrapped.scene.env_origins[0])
    st = object_states(environment, ["slide", "stage"])
    sl, stg = st["slide"], st["stage"]
    o, R = np.asarray(sl.pos, float), _rotation(sl.quat_xyzw)
    q = stg.joints
    carriage = (float(q["stage_y"]), float(q["stage_x"]))
    phi = float(q["caliper"])
    S.update(cal=phi, carriage=carriage)
    _stage_friction(environment, carriage)
    if _DEBUG and S["step"] == 1:
        a = environment.unwrapped.scene["stage"]
        print("stage joints", a.joint_names, "K", _np(a.data.joint_stiffness)[0].round(4).tolist(),
              "D", _np(a.data.joint_damping)[0].round(5).tolist(), "effort", _np(a.data.joint_effort_limits)[0].tolist(),
              flush=True)
    if S["start_q"] is None:
        S["start_q"] = carriage
    # gripper contacts on the slide (rule 1) and the grip
    grip_f, contacts = _slide_contacts(V, origin, o, R)
    S["grip_f"] = grip_f
    held = grip_f > g.TOUCH_F
    why = _face_violation(contacts)
    if why:
        S["face"] = [(np.round(p * 1000, 2).tolist(), np.round(n, 2).tolist(), round(f, 3)) for p, n, f in contacts][:4]
        _fail(why)
    # rule 2: a gripper or the slide on an objective or the nosepiece
    fo = np.linalg.norm(_np(V["optics"].get_contact_force_matrix(V["dt"]))[0], axis=-1)
    S["optic_f"] = float(fo.max())
    if fo.max() > g.TOUCH_F:
        S.setdefault("optic_who", (S["step"], np.round(fo, 3).tolist()))
        _fail("touched an objective or the nosepiece" if fo[:-1].max() > g.TOUCH_F else "slide touched an objective")
    # rule 3: a free fall over 10 mm, or at rest anywhere but the holder, a gripper or the stage
    if held:
        S["ref_z"] = None
    else:
        S["ref_z"] = o[2] if S["ref_z"] is None else max(S["ref_z"], o[2])
        if S["ref_z"] - o[2] > g.DROP:
            _fail("dropped")
    # still: moved less than REST_SPEED x 50 ms and turned less than 0.5 deg since the last control step (positions,
    # not PhysX's velocities, which jitter on a 4.7 g body under a spring)
    last = S.get("last_pose")
    still = last is not None and bool(np.linalg.norm(o - last[0]) < g.REST_SPEED * 0.05
                                      and np.degrees(np.arccos(np.clip((np.trace(last[1].T @ R) - 1) / 2, -1, 1))) < 0.5)
    S["last_pose"] = (o.copy(), R.copy())
    S["still"] = still
    S["rest"] = S["rest"] + 1 if (still and not held) else 0
    m = o - np.array(M0) - np.array([carriage[0], carriage[1], 0.0])
    on_stage = (g.STAGE_X[0] <= m[0] <= g.STAGE_X[1] and g.STAGE_Y[0] <= m[1] <= g.STAGE_Y[1]
                and -0.002 <= m[2] <= 0.006)
    hp = g.holder_slot_floor()
    hl = g.holder_rot().T @ (o - hp)
    in_holder = (abs(hl[0]) <= g.HOLDER_SIZE[0] / 2 and abs(hl[1]) <= g.HOLDER_SIZE[1] / 2
                 and o[2] <= g.TOP + g.HOLDER_SIZE[2] + 0.06)
    if S["rest"] >= 10 and not (on_stage or in_holder):
        _fail("came to rest off the stage and holder")
    # rule 5: the stage's carriage moved more than 1 mm from its start
    if max(abs(carriage[0] - S["start_q"][0]), abs(carriage[1] - S["start_q"][1])) > g.STAGE_MOVED:
        _fail("stage moved")
    # the caliper: its tip against the slide, a gripper on it; the landing (hard-landing flag)
    fc = np.linalg.norm(_np(V["caliper"].get_contact_force_matrix(V["dt"]))[0], axis=-1)
    S["tip_slide"], S["tip_hand"] = float(fc[0]), float(fc[1:].max())
    tip = g.to_world(g.caliper_point((g.ARM_TIP[0][0] / 2 + g.ARM_TIP[0][1] / 2, g.ARM_TIP[1][1], 0.001), phi,
                                     carriage))
    on_slide, on_cal = S["tip_slide"] > g.TOUCH_F, S["tip_hand"] > g.TOUCH_F
    hist = S["touch_hist"]
    still_tip = bool(hist) and np.linalg.norm(tip - hist[-1][1]) < g.TIP_STILL
    S["touch_hist"] = hist = (hist + [(S["step"], tip, on_slide, on_cal)])[-800:]
    # a landing is the arm coming to rest on the slide with no gripper on it (2 steps); a touch while a hand holds
    # the arm is not one. Each landing is measured (its final approach); a hard one latches the flag.
    resting = on_slide and not on_cal and still_tip
    S["rest_tip"] = S["rest_tip"] + 1 if resting else 0
    if S["rest_tip"] == 2 and not S["landed"]:
        got = _final_landing(hist[:-1])
        if got is not None:
            speed, on_lever, at = got
            S["landing"] = (round(speed * 1000, 1), on_lever, at)
            S["landings"].append(S["landing"])
            if speed > g.V_MAX or not on_lever:
                S["hard"] = True
        S["landed"] = True
    if not (on_slide and not on_cal):
        S["landed"] = False
    # the end-state ladder, this step
    corners = o + g.slide_corners() @ R.T
    S["corners"] = corners[:, 2] - g.STAGE_Z
    S["up"] = float(R[2, 2])
    flat = bool(np.all(np.abs(S["corners"]) <= g.FLAT_TOL) and R[2, 2] > 0.99 and still and on_stage)
    cm = corners - np.array(M0) - np.array([carriage[0], carriage[1], 0.0])     # stage (carriage) frame
    back = np.sort(cm[:, 0])[-2:]
    left = np.sort(cm[:, 1])[-2:]
    S["back"] = tuple(np.round((g.BACK_EDGE_X - back) * 1000, 2).tolist())
    S["jaw"] = tuple(np.round((g.JAW_Y - left) * 1000, 2).tolist())
    seated = bool(np.all(np.abs(g.BACK_EDGE_X - back) <= g.SEAT_TOL) and np.all(np.abs(g.JAW_Y - left) <= g.SEAT_TOL))
    cover = o + R @ np.array([0.0, 0.0, g.SLIDE_T / 2 + g.COVER_T / 2])
    under = bool(math.hypot(cover[0] - g.AXIS_X, cover[1] - g.AXIS_Y) <= g.UNDER_TOL)
    held_by_arm = bool(S["tip_slide"] > g.TOUCH_F and S["tip_hand"] <= g.TOUCH_F)
    S.update(flat=flat, held=held_by_arm, seated=seated, under=under)
    rung = 0.0
    if flat:
        boxes = [(R, o + R @ np.array([0, 0, g.COVER_T / 2]), np.array([g.SLIDE_L, g.SLIDE_W, g.SLIDE_T + g.COVER_T]))]
        boxes += caliper_parts(phi, carriage)
        S["gap"] = _hand_gap(environment, boxes)
        S["clear"] = S["gap"] >= g.CLEAR
        if S["clear"]:
            if not held_by_arm:
                rung = 0.25
            elif not (seated and under):
                rung = 0.5
            else:
                rung = 0.75 if S["hard"] else 1.0
    else:
        S["clear"] = False
    S["rung"] = rung
    S["rungs"] = (S["rungs"] + [rung])[-g.HOLD_STEPS:]


def success(objects) -> bool:
    """The full-score state with no zero condition (yam_arena ends the trial once it has held for 1 s)."""
    S = _STATE
    return bool(not S["zero"] and S["rung"] == 1.0)


def score(objects) -> float:
    """Every control step (20 Hz):
    0 for the run (latched) if a gripper touches the coverslip or a face of the slide outside its frosted label end (a
    contact normal within 45 degrees of the face normal); a gripper or the slide touches an objective or the nosepiece;
    the slide falls freely more than 10 mm or comes to rest anywhere but the bench holder, a gripper or the stage; or
    the stage's X or Y carriage moves more than 1 mm from its start.
    Hard-landing flag (latched, caps at 0.75): whenever the caliper's arm comes to rest on the slide with no gripper on
    it, its landing is measured: the tip's mean speed over the last 5 mm of its final approach (its last continuous
    motion before it stopped) exceeds V_max (20 mm/s), or no gripper was on the caliper when the tip met the slide on
    that approach. A touch while a hand holds the arm (open) is not a landing.
    Otherwise, from the final 1 s (the lowest rung any of its steps reached), each level with both grippers 3 cm or
    more from the slide, the caliper's arm and its lever:
    - 0.25: flat (four corners within 0.3 mm of the stage plate, coverslip up, still), not held by the arm;
    - 0.5: flat and held (the arm's tip on the slide, no gripper on the caliper), not seated or not under the objective;
    - 0.75: flat, held, seated (back edge's two ends within 0.3 mm of the template's back edge, left end's within 0.3
      mm of the jaw) and the coverslip's centre within 8 mm of the optical axis, with the hard-landing flag;
    - 1.0: the same with a gentle landing.
    0 for every other end state (coverslip down, riding on the template, still held, untouched)."""
    S = _STATE
    if S["zero"] or not S["rungs"]:
        return 0.0
    return float(min(S["rungs"]))
