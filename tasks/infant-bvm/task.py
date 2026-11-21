"""On a rubber mat on the station table, a Laerdal Little Baby QCPR infant CPR manikin lies face up, head toward the
robot, its head in the neutral (sniffing) position; a white card flag with a black dot stands on its chest. To its left
an Ambu SPUR II Infant bag-valve mask stands mask rim down on the mat, the bag pointing toward the robot. Give the
manikin two rescue breaths with the bag-valve mask, then lift the mask off and keep holding it.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score --max-steps 2400 ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import Shape, UsdBody, before_reset, on_reset, on_step  # noqa: E402

import bvm_rules as rules  # noqa: E402
import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Give the infant manikin two rescue breaths with the bag-valve mask: keep its head in the sniffing "
               "position, hold the mask sealed over its nose and mouth, squeeze the bag just until the chest rises "
               "each time, then lift the mask off its face and keep holding it.")

HERE = pathlib.Path(__file__).resolve().parent
LM = dict(np.load(HERE / "assets" / "landmarks.npz"))
SEGS = [f"seg_{i:02d}" for i in range(g.N_SEG)]
WALLS = ("top", "left", "bottom", "right")


def _quat_xyzw(R) -> tuple:
    R = np.asarray(R, float)
    w = math.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return tuple(float(v) for v in ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w),
                                    (R[1, 0] - R[0, 1]) / (4 * w), w))


# -- objects -----------------------------------------------------------------------------------------------------------
MAT = Shape("mat", pos=(g.MAT_C[0], g.MAT_C[1], g.TOP + g.MAT_SIZE[2] / 2), size=g.MAT_SIZE, color=(0.18, 0.22, 0.30))
MANIKIN = UsdBody("manikin", usd="assets/manikin.usda", pos=(g.CROWN_X, 0.0, g.MAT_TOP), size=(0.65, 0.26, 0.14),
                  articulation=True, joint_pos=(("neck", 0.0), ("rise", 0.0)),
                  drives=(("neck", g.NECK_K, g.NECK_D), ("rise", 2000.0, 20.0)), friction=(g.MAT_MU, g.MAT_MU),
                  solver_iters=32)
_bp, _bR = g.bvm_start(LM["face_h"])
BVM = UsdBody("bvm", usd="assets/bvm.usda", pos=tuple(float(v) for v in _bp), rot_xyzw=_quat_xyzw(_bR),
              size=(0.27, 0.07, 0.1), articulation=True,
              joint_pos=tuple((s, 0.0) for s in SEGS) + (("swivel", 0.0),) + tuple((f"wall_{w}", 0.0) for w in WALLS),
              drives=(("seg_.*", g.SEG_K, g.SEG_D), ("wall_.*", g.WALL_K, g.WALL_D), ("swivel", g.SWIVEL_K, g.SWIVEL_D)),
              friction=(g.BVM_MU, g.BVM_MU), solver_iters=64)
OBJECTS = [MAT, MANIKIN, BVM]

HAND_LINKS = ("gripper", "tip_left", "tip_right")
ARM_LINKS = ("base", "link1", "link2", "link3", "link4", "link5") + HAND_LINKS
_STATE: dict = {}
_VIEWS: dict = {}
_DEBUG = bool(os.environ.get("BVM_DEBUG"))


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
    """Neck at neutral and its friction's rest angle there; chest down; the cushion and the bag's walls at rest with
    their springs preloaded; swivel straight."""
    from scenekit import set_joint_targets, write_joint_positions

    write_joint_positions(environment, "manikin", {"neck": 0.0, "rise": 0.0})
    set_joint_targets(environment, "manikin", {"neck": 0.0, "rise": 0.0})
    joints = {s: 0.0 for s in SEGS} | {f"wall_{w}": 0.0 for w in WALLS} | {"swivel": 0.0}
    write_joint_positions(environment, "bvm", joints)
    set_joint_targets(environment, "bvm", {s: -g.SEG_PRELOAD / g.SEG_K for s in SEGS}
                      | {f"wall_{w}": -g.WALL_PRELOAD / g.WALL_K for w in WALLS} | {"swivel": 0.0})


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.pop("ep", None)
    _STATE.update(step=0, why=[], zero=False, lungs=rules.Lungs(), breaths=rules.Breaths(), neck_rest=0.0, margins=[],
                  swivel_rest=0.0,
                  start=None, back0=None, eye_steps=0, seated_run=0, seated_2s=False, held_steps=0, held_off=False,
                  last_bvm=None, ref_z=None, rise=0.0, v_bag=0.0, v_lung=0.0, head_deg=0.0, seated=False,
                  centre_mm=99.0, pressed=0, clean=True, who=[], mask_hand=None, gripped=None, face_gap=1.0,
                  segs_mm=[], tier=0.0, airway=True, on_eye=False, moved_mm=0.0, lift_mm=0.0)
    _VIEWS.clear()


_reset_state()


def _fail(why: str) -> None:
    if why not in _STATE["why"]:
        _STATE["why"].append(why)
    _STATE["zero"] = True


def _views(environment) -> dict:
    """PhysX contact views: the manikin's torso and head against every BVM body and every arm link; the BVM's
    bodies against the grippers' links (and the cushion against the head)."""
    if not _VIEWS:
        from yam_arena.robot import asset_link_paths

        import yam_arena.constants as C

        psv = environment.unwrapped.sim.physics_sim_view
        ns = "/World/envs/env_*"
        lp = asset_link_paths()
        bvm = [f"{ns}/bvm/dome"] + [f"{ns}/bvm/{s}" for s in SEGS] + [f"{ns}/bvm/valve"] + \
              [f"{ns}/bvm/wall_{w}" for w in WALLS]
        arms = [f"{ns}/{arm}{lp[link]}" for arm in ("LeftArm", "RightArm") for link in ARM_LINKS]
        hands = [f"{ns}/{arm}{lp[link]}" for arm in ("LeftArm", "RightArm") for link in HAND_LINKS]
        others = bvm + arms
        _VIEWS["names"] = [p.rsplit("/", 1)[1] for p in bvm] + [f"{a}:{l}" for a in ("left", "right") for l in ARM_LINKS]
        _VIEWS["n_bvm"] = len(bvm)
        _VIEWS["torso"] = psv.create_rigid_contact_view(f"{ns}/manikin/torso", filter_patterns=others)
        _VIEWS["head"] = psv.create_rigid_contact_view(f"{ns}/manikin/head", filter_patterns=others)
        _VIEWS["dome"] = psv.create_rigid_contact_view(f"{ns}/bvm/dome", filter_patterns=hands)
        _VIEWS["segs"] = psv.create_rigid_contact_view(f"{ns}/bvm/seg_*",
                                                       filter_patterns=[f"{ns}/manikin/head"] + hands)
        _VIEWS["valve"] = psv.create_rigid_contact_view(f"{ns}/bvm/valve", filter_patterns=hands)
        _VIEWS["walls"] = psv.create_rigid_contact_view(f"{ns}/bvm/wall_*", filter_patterns=hands)
        _VIEWS["dt"] = float(C.PHYSICS_DT)
    return _VIEWS


def _forces(view, dt) -> np.ndarray:
    """|force| per (sensor, filter)."""
    return np.linalg.norm(_np(view.get_contact_force_matrix(dt)).reshape(-1, view.filter_count, 3), axis=-1)


def body_poses(environment, name: str) -> dict:
    """Each body's (position, rotation) of a UsdBody articulation, station frame."""
    base = environment.unwrapped
    a = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(a.data.body_link_pose_w)[0]
    return {n: (pose[i, :3] - origin, _rotation(pose[i, 3:7])) for i, n in enumerate(a.body_names)}


def joints(environment, name: str) -> dict:
    a = environment.unwrapped.scene[name]
    q = _np(a.data.joint_pos)[0]
    return {n: float(q[i]) for i, n in enumerate(a.joint_names)}


def face_point(head) -> np.ndarray:
    """The face's mask point (the nominal mask centre), station frame."""
    p, R = head
    return p + R @ (LM["mask_o"] - LM["hinge"])


_TREE: dict = {}


def face_gap(head, pts) -> float:
    """The smallest distance from these points to the head's surface (vertices every ~1 mm)."""
    if "t" not in _TREE:
        from scipy.spatial import cKDTree
        _TREE["t"] = cKDTree(LM["head_pts"])
    p, R = head
    d, _ = _TREE["t"].query((np.asarray(pts) - p) @ R)
    return float(d.min())


_CP = np.c_[g.contact_points()[0], np.zeros(g.N_SEG)]
_OUT = np.c_[g.mask_outline(96), LM["out_h"] - g.SEG_NOM]
_EYES = LM["eyes"] - LM["hinge"]


def _stick_slip(environment, asset: str, joint: str, q: float, k: float, brk: float) -> None:
    """A joint's friction as stick-slip: a stiff spring holds it to a rest angle that is dragged along once the
    spring's moment passes the breakaway moment, so a push harder than that turns it and it stays where it is left."""
    from scenekit import set_joint_targets

    key = f"{joint}_rest"
    lim = brk / k
    r = _STATE[key]
    if abs(q - r) > lim:
        _STATE[key] = q - math.copysign(lim, q - r)
        set_joint_targets(environment, asset, {joint: _STATE[key]})


@on_step
def _monitor(environment) -> None:
    from scenekit import set_joint_targets

    S = _STATE
    S["step"] += 1
    V = _views(environment)
    dt = V["dt"]
    man, bvm = body_poses(environment, "manikin"), body_poses(environment, "bvm")
    qm, qb = joints(environment, "manikin"), joints(environment, "bvm")
    torso, head, dome = man["torso"], man["head"], bvm["dome"]
    _stick_slip(environment, "manikin", "neck", qm["neck"], g.NECK_K, g.NECK_BREAK)
    _stick_slip(environment, "bvm", "swivel", qb["swivel"], g.SWIVEL_K, g.SWIVEL_BREAK)
    head_deg = -math.degrees(qm["neck"])                 # + = tilted back (extension)
    airway = g.AIRWAY[0] <= head_deg <= g.AIRWAY[1]

    # -- contacts ---------------------------------------------------------------------------------------------------
    ft, fh = _forces(V["torso"], dt)[0], _forces(V["head"], dt)[0]
    f_segs = _forces(V["segs"], dt)                      # (12, 1 + 6): head, then the hand links
    f_dome = _forces(V["dome"], dt)[0]
    f_valve = _forces(V["valve"], dt)[0]
    f_walls = _forces(V["walls"], dt).sum(0)
    f_hand = f_segs[:, 1:].sum(0) + f_dome + f_valve + f_walls      # each hand link's contact with the BVM
    touch = g.TOUCH_F
    on_mask = [(f_segs[:, 1 + 3 * k: 4 + 3 * k].sum() + f_dome[3 * k: 3 * k + 3].sum()) > touch for k in range(2)]
    gripped = [bool(f_hand[3 * k + 1] > touch and f_hand[3 * k + 2] > touch) for k in range(2)]
    any_grip = bool((f_hand > touch).any())
    # clean: nothing touches the manikin but the mask's dome and cushion, and the hand holding the mask on the head
    names, nb = V["names"], V["n_bvm"]
    who = []
    for k, n in enumerate(names):
        allowed_t = k < 1 + g.N_SEG                      # the dome and cushion segments
        allowed_h = allowed_t
        if k >= nb:
            arm, link = n.split(":")
            allowed_h = link in HAND_LINKS and on_mask[0 if arm == "left" else 1]
        if ft[k] > touch and not allowed_t:
            who.append(f"torso<-{n}")
        if fh[k] > touch and not allowed_h:
            who.append(f"head<-{n}")
    clean = not who
    mask_hand = [s for s, m in zip(("left", "right"), on_mask) if m]

    # -- the mask on the face ---------------------------------------------------------------------------------------
    centre = float(np.linalg.norm(dome[0] - face_point(head)))
    seg_q = np.array([qb[s] for s in SEGS])
    pressed = (seg_q >= g.SEG_IN) & (f_segs[:, 0] > touch)
    on_face = bool((f_segs[:, 0] > touch).any())
    outline_h = (dome[0] + _OUT @ dome[1].T - head[0]) @ head[1]          # the cushion's outer edge, head frame
    d_eye = min(float(np.linalg.norm(outline_h[:, :2] - e[:2], axis=1).min()) for e in _EYES)
    on_eye = on_face and d_eye < g.EYE_R
    S["eye_steps"] = S["eye_steps"] + 1 if on_eye else 0
    if S["eye_steps"] * rules.DT >= g.EYE_LATCH_S:
        _fail("the mask's rim pressed on an eye for 1 s")
    seated = bool(centre <= g.SEAT_TOL and pressed.all() and not on_eye)
    S["seated_run"] = S["seated_run"] + 1 if seated else 0
    if S["seated_run"] * rules.DT >= g.SEATED_025:
        S["seated_2s"] = True

    # -- the air rule: bag -> lungs -> chest ------------------------------------------------------------------------
    d_lr = qb["wall_left"] + qb["wall_right"]
    d_tb = qb["wall_top"] + qb["wall_bottom"]
    v_bag = g.bag_volume_ml(max(0.0, d_lr), max(0.0, d_tb))
    rise = S["lungs"].step(v_bag, seated, airway)
    set_joint_targets(environment, "manikin", {"rise": rise})
    n_closed = len(S["breaths"].breaths) + len(S["breaths"].too_big) + len(S["breaths"].void)
    S["breaths"].step(rise, seated, clean)
    # margins of each episode (README): the least-pressed segment, the head's range, the centre, the squeeze
    if rise > g.ONSET:
        ep = S.setdefault("ep", dict(seg_min=9.0, head=[99.0, -99.0], centre=0.0, v_bag=0.0, clean=True))
        ep["seg_min"] = min(ep["seg_min"], float(seg_q.min() * 1000))
        ep["head"] = [min(ep["head"][0], head_deg), max(ep["head"][1], head_deg)]
        ep["centre"] = max(ep["centre"], centre * 1000)
        ep["v_bag"] = max(ep["v_bag"], v_bag)
        ep["clean"] = ep["clean"] and clean
    n_now = len(S["breaths"].breaths) + len(S["breaths"].too_big) + len(S["breaths"].void)
    if n_now > n_closed and "ep" in S:
        ep = S.pop("ep")
        S.setdefault("margins", []).append({k: (np.round(v, 2).tolist() if isinstance(v, list) else
                                                (round(v, 2) if isinstance(v, float) else v)) for k, v in ep.items()})

    # -- zero latches -----------------------------------------------------------------------------------------------
    corners = np.array([[x, y, z] for x in LM["torso_bounds"][:, 0] for y in LM["torso_bounds"][:, 1]
                        for z in LM["torso_bounds"][:, 2]])
    pts = torso[0] + corners @ torso[1].T
    back = torso[0] + LM["back_pts"] @ torso[1].T
    if S["start"] is None:
        S["start"], S["back0"] = pts, back[:, 2].copy()
    S["moved_mm"] = float(np.linalg.norm(pts - S["start"], axis=1).max() * 1000)
    S["lift_mm"] = float((back[:, 2] - S["back0"]).max() * 1000)
    if S["moved_mm"] >= g.MOVED * 1000:
        _fail("the manikin moved 20 mm")
    if S["lift_mm"] >= g.BACK_LIFT * 1000:
        _fail("the manikin's back lifted 5 mm off the mat")
    z_bvm = np.array([dome[0][2], bvm["valve"][0][2]])
    if any_grip:
        S["ref_z"] = None
    else:
        S["ref_z"] = z_bvm if S["ref_z"] is None else np.maximum(S["ref_z"], z_bvm)
        bvm_on_manikin = bool((ft[:nb] > touch).any() or (fh[:nb] > touch).any())
        if float((S["ref_z"] - z_bvm).max()) >= g.DROP_FALL and bvm_on_manikin:
            _fail("the bag-valve mask was dropped onto the manikin")

    # -- held off: mask 30 mm or more from the face, the BVM in a gripper and still, for 1 s ------------------------
    tips = np.array([p - dome[1][:, 2] * g.SEG_R for n, (p, R) in bvm.items() if n.startswith("seg_")])
    gap = min(face_gap(head, tips), face_gap(head, [dome[0] + dome[1] @ [0, 0, g.DOME_TOP]]))
    last = S["last_bvm"]
    now = np.r_[dome[0], bvm["valve"][0]]
    still = last is not None and float(np.abs(now - last).reshape(2, 3).__pow__(2).sum(1).max() ** 0.5) < g.HELD_STILL
    S["last_bvm"] = now
    held = gap >= g.HELD_OFF_GAP and any(gripped) and still
    S["held_steps"] = S["held_steps"] + 1 if held else 0
    S["held_off"] = S["held_steps"] * rules.DT >= g.HOLD_S

    n_b, n_big = S["breaths"].counts()
    S["tier"] = 0.0 if S["zero"] else rules.tier(n_b, n_big, S["held_off"], S["seated_2s"])
    S.update(rise=rise, v_bag=v_bag, v_lung=S["lungs"].v_lung, head_deg=head_deg, airway=airway, seated=seated,
             centre_mm=centre * 1000, pressed=int(pressed.sum()), clean=clean, who=who[:4], mask_hand=mask_hand,
             gripped=gripped, face_gap=gap, segs_mm=np.round(seg_q * 1000, 2).tolist(), on_eye=on_eye,
             d_eye_mm=d_eye * 1000, d_lr_mm=d_lr * 1000, n_breath=n_b, n_big=n_big)
    if _DEBUG and (S["step"] % 10 == 0 or rise > 0.0005 or S["step"] < 6):
        print(f"[bvm] s{S['step']} head {head_deg:5.1f} centre {centre * 1000:5.1f} pressed {int(pressed.sum())} "
              f"seated {int(seated)} clean {int(clean)} {who[:2]} sq {d_lr * 1000:5.1f} V {v_bag:5.1f} "
              f"rise {rise * 1000:5.2f} br {n_b}/{n_big} gap {gap * 1000:5.1f} grip {gripped} eye {d_eye * 1000:4.1f} "
              f"moved {S['moved_mm']:.1f} lift {S['lift_mm']:.1f}",
              flush=True)


def success(objects) -> bool:
    """1.0 (two breaths, none too big, then held off), with no zero condition; yam_arena ends the trial once it has
    held for 1 s."""
    return bool(not _STATE["zero"] and _STATE["tier"] == 1.0)


def score(objects) -> float:
    """Read every 50 ms step.
    0 for the run (latched) if the manikin moves 20 mm or its back lifts 5 mm off the mat; the mask's rim presses on
    an eye for 1 s; or the bag-valve mask falls 30 mm ungripped onto the manikin.
    Seated: the mask's centre within 6 mm of the face's mask point and all 12 cushion segments pressed on the face (in
    1 mm or more, touching), none on an eye. Clean: nothing touches the manikin but the mask's dome and cushion, and
    the fingers of the hand that holds the mask on the head.
    A breath: while seated and clean, the chest's flag dot rises 5 mm or more, peaks no more than 12 mm up 0.5 s to 2 s
    after it started to rise, and falls back within 2 mm. Too big: seated and clean, peaking over 12 mm.
    Held off: the mask 30 mm or more from the face, the BVM in a gripper (both pads on it) and still (under 2 mm per
    step) for 1 s.
    At the end: 1.0 exactly two breaths, none too big, then held off; 0.75 the same but not held off; 0.5 one breath,
    or three or more, or any breath with a too-big one; 0.25 no breath but the mask seated for 2 s or more; else 0."""
    return float(_STATE["tier"])
