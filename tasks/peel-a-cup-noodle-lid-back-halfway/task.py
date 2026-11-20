"""On the station table: an unopened cup of instant noodles (dry), its paper-foil lid heat-sealed to the rim, the tab
pointing to the robot's right.

The lid is a chain of 5 mm strips (make_assets.py). A strip is sealed to the rim until the pull that reaches it through
the film is more than its seal can take; the seal is a 3 mm ring on the rim, so near the tab the front runs along a
long stretch of ring and holds hard (about 6 N to start the tab), and across the middle only two short crossings hold
it (about 1.2 N). The cup weighs 75 g, so any of those pulls lifts or tips it unless it is held.

The job: peel the lid back halfway, as the cup's directions say, so that it can be closed down again over the hot
water while the noodles steep.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import make_assets as A  # noqa: E402
from dynkit import UsdBody, arms, now, on_reset, on_step  # noqa: E402

INSTRUCTION = "Peel back the lid of the cup noodles halfway, ready for the hot water."

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

# World frame: x forward from the arms, y to the robot's left, z up (m). The tab points to -y (the right arm).
BASE = np.array([0.36, -0.02, TOP])
N, W = A.N_STRIP, A.strip_width()
JOINTS = [f"j{k}" for k in range(N)]

# -- the seal -----------------------------------------------------------------------------------------------------
PEEL_N_PER_M = 200.0        # seal strength per metre of sealed front (3 N per 15 mm, an easy-peel lid)
START_FACTOR = 1.5          # the first strip starts the peel from the tab's edge: it takes half as much again


def _seal_strengths() -> np.ndarray:
    R, ri = A.R_TOP, A.R_TOP - A.RIM_W
    out = []
    for k in range(N):
        y0, y1 = A.strip_y(k)
        ys = np.linspace(y0, y1, 400)
        ch = 2 * np.sqrt(np.clip(R * R - ys * ys, 0, None)) - 2 * np.sqrt(np.clip(ri * ri - ys * ys, 0, None))
        front = float(np.trapezoid(ch, ys)) / W            # m of sealed ring the front crosses in this strip
        out.append(PEEL_N_PER_M * front * (START_FACTOR if k == 0 else 1.0))
    return np.array(out)


SEAL_N = _seal_strengths()
LOCK_K, LOCK_D = 20.0, 0.01            # N m/rad, N m s/rad: a sealed strip lies on the rim
ARMATURE = 2e-5                         # kg m^2 on each hinge, for a stiff chain of light strips
TAB_K, TAB_D = 0.01, 0.0005             # N m/rad, N m s/rad: the paper tab's bend at the rim
MAX_RUN = 3                             # strips the peel front may run per control step (30 cm/s)
GRASP_AT = 0.75                         # where along the tab its grasp point (tab_tip) is
LOCK_TOL = 0.002                        # rad
FREE_K, FREE_D = 0.0004, 0.00005        # a peeled strip: the film's own slight stiffness
FOLD_K, FOLD_RAD = 0.002, math.radians(-105.0)  # the crease at the peel front: the lid stays folded back, standing up
# (a hinge's + turns its strip down, under the lid; a peeled strip can't pass back under the sealed part)
PEELED_LIM = (math.radians(-175.0), math.radians(5.0))

# -- scoring constants --------------------------------------------------------------------------------------------
HALF_LO, HALF_HI = 0.038, 0.057         # m of the 95 mm lid peeled: 40 % .. 60 % is "halfway"
SOME = 0.015                            # m peeled: the peel has properly begun
TOO_FAR = 0.080                         # m peeled: past this the lid can't be closed down again (0)
TIP_DEG = 30.0
UPRIGHT_DEG = 10.0
LIFT_MAX = 0.010                        # m: the cup's base this far above the table has left it (0)
DENT_N = 20.0                           # N per finger: a grip on the paper cup harder than this dents its wall (0)
FINGER_K, FINGER_MAX = 5000.0, 50.0     # the YAM finger servo: N/m and its force limit (yam_arena GRIPPER)
LET_GO_CUP = 0.065                      # m from the cup's axis (horizontal) to each tool point
LET_GO_TAB = 0.04                       # m from the tab to each tool point

CUP = UsdBody("cup", pos=tuple(map(float, BASE + [0, 0, 0.0005])), size=(0.094, 0.094, 0.106), usd="assets/cup.usda",
              articulation=True, joint_pos=tuple((j, 0.0) for j in JOINTS + ["jt"]), joint_damping=0.0,
              friction=(0.8, 0.7), solver_iters=64)
OBJECTS = [CUP]

_STATE: dict = {}


def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def tilt_deg(R) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))


def _r_at(z: float) -> float:
    """The cup's outer radius at height z above its base (m)."""
    z = min(max(z, 0.0), A.H)
    return A.R_BASE + (A.R_TOP - A.R_BASE) * z / A.H


def peeled_m(m: int) -> float:
    """How far the lid is peeled back from the tab's edge (m) with ``m`` strips free."""
    return m * W


# -- joint drives ---------------------------------------------------------------------------------------------------
def _set_drives(environment, m: int) -> None:
    import torch

    art = environment.unwrapped.scene["cup"]
    ids = art.find_joints(JOINTS, preserve_order=True)[0]
    k = [FREE_K] * N
    d = [FREE_D] * N
    tgt = [0.0] * N
    for i in range(N):
        if i >= m:
            k[i], d[i] = LOCK_K, LOCK_D
        elif i == m - 1:
            k[i], tgt[i] = FOLD_K, FOLD_RAD
    dev = art.device
    art.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([k], dtype=torch.float32, device=dev), joint_ids=ids)
    art.write_joint_damping_to_sim_index(damping=torch.tensor([d], dtype=torch.float32, device=dev), joint_ids=ids)
    art.set_joint_position_target(torch.tensor([tgt], dtype=torch.float32, device=dev), joint_ids=ids)
    jt = art.find_joints(["jt"])[0]                          # the tab bends limply at the rim edge
    art.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[TAB_K]], device=dev), joint_ids=jt)
    art.write_joint_damping_to_sim_index(damping=torch.tensor([[TAB_D]], device=dev), joint_ids=jt)
    # a sealed strip is held flat on the rim by its hinge's limits (a hard constraint); a peeled one bends freely
    lim = [[-LOCK_TOL, LOCK_TOL] if i >= m else list(PEELED_LIM) for i in range(N)]
    art.write_joint_position_limit_to_sim_index(limits=torch.tensor([lim], dtype=torch.float32, device=dev),
                                                joint_ids=ids, warn_limit_violation=False)


@on_reset
def _reset(environment=None) -> None:
    _STATE.update(t=0.0, m=0, written=False, violation=None, cup_pos=BASE.copy(), tilt=0.0, upright=True,
                  on_table=True, released=True, done_since=None, max_force=0.0, force=0.0, peel_log=[],
                  root=None, fmax=np.zeros(N), max_lift=0.0, squeeze=0.0, max_squeeze=0.0)
    if environment is not None:
        _write_layout(environment)


_reset()


def _write_layout(environment) -> None:
    import torch

    base = environment.unwrapped
    origin = base.scene.env_origins
    dev = origin.device
    art = base.scene["cup"]
    rp = art.data.root_link_pose_w[0, :3]
    bp = art.data.body_pos_w[0]
    root = art.body_names[int(torch.argmin(torch.linalg.norm(bp - rp, dim=1)))]
    off = np.zeros(3)
    if root != "cup":                          # PhysX may root the free articulation at any link: place that one
        y = A.strip_y(0)[0] - 0.001 if root == "tab" else A.strip_y(int(root[1:]))[1]
        off = np.array([0.0, y, A.H + 0.0002])
    pose = torch.tensor([[*(BASE + [0, 0, 0.0005] + off), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=dev)
    pose[:, :3] += origin
    art.write_root_pose_to_sim_index(root_pose=pose)
    art.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev))
    ids = art.find_joints(JOINTS, preserve_order=True)[0]
    z = torch.zeros((1, len(ids)), dtype=torch.float32, device=dev)
    art.write_joint_position_to_sim_index(position=z, joint_ids=ids)
    art.write_joint_velocity_to_sim_index(velocity=z, joint_ids=ids)
    art.write_joint_armature_to_sim_index(armature=torch.full((1, len(ids)), ARMATURE, device=dev), joint_ids=ids)
    _set_drives(environment, 0)
    _ = art.data.body_pos_w[0].detach().cpu().numpy()      # read back: the next step starts from fresh buffers
    _STATE["written"] = True
    _STATE["root"] = root


def _flag(what: str) -> None:
    if _STATE["violation"] is None:
        _STATE["violation"] = (round(_STATE["t"], 2), what)


def incoming_forces(environment) -> dict:
    """The force (N) each strip's hinge passes on from the strip to its parent: the pull reaching that strip."""
    import warp as wp

    art = environment.unwrapped.scene["cup"]
    raw = wp.to_torch(art.root_view.get_link_incoming_joint_force())[0].detach().cpu().numpy().reshape(-1, 6)
    names = art.body_names
    return {n: raw[i] for i, n in enumerate(names)}


def bodies(environment) -> dict:
    base = environment.unwrapped
    org = base.scene.env_origins[0].detach().cpu().numpy()
    art = base.scene["cup"]
    pos = art.data.body_pos_w[0].detach().cpu().numpy() - org
    quat = art.data.body_quat_w[0].detach().cpu().numpy()
    return {n: (pos[i].astype(float), quat[i].astype(float)) for i, n in enumerate(art.body_names)}


@on_step
def _monitor(environment) -> None:
    S = _STATE
    S["t"] = now()
    if not S["written"]:
        _write_layout(environment)
    b = bodies(environment)
    cp, cq = b["cup"]
    R = quat_matrix(cq)
    S["cup_pos"] = cp
    # the seal: the first still-sealed strip lets go once the pull reaching it beats its seal
    m = S["m"]
    if m < N:
        # the pull on the front: the force through the hinge between strip m and the peeled side (the tab's hinge
        # for m = 0). PhysX reports, for each link, the force its parent applies through its incoming joint; which
        # side of the hinge is the child depends on where PhysX rooted the articulation
        art = environment.unwrapped.scene["cup"]
        jn = "jt" if m == 0 else f"j{m - 1}"
        child = art.body_names[art.joint_names.index(jn) + 1]
        f = incoming_forces(environment)[child]
        free = child == "tab" or (child.startswith("s") and int(child[1:]) < m)
        fl = (-1.0 if free else 1.0) * (R.T @ np.asarray(f[:3], float))
        # only a pull peels: outward along the lid (-y, back toward the tab) and up off the rim (+z); pushing the
        # lid in along itself or down onto the rim does not
        mag = float(math.hypot(max(0.0, -fl[1]), max(0.0, fl[2])))
        S["force"] = mag
        S["fmax"][m] = max(S["fmax"][m], mag)
        S["max_force"] = max(S["max_force"], mag)
        # the peel can run up to MAX_RUN strips (MAX_RUN * 5 mm) in one 50 ms control step: as fast as the film is
        # pulled; a pull that beats this strip's seal also beats the next ones while it is still above theirs
        n0 = m
        while S["m"] < N and S["m"] - n0 < MAX_RUN and mag > SEAL_N[S["m"]]:
            S["peel_log"].append((round(S["t"], 2), S["m"], round(mag, 2)))
            S["m"] += 1
        if S["m"] != n0:
            _set_drives(environment, S["m"])
    if peeled_m(S["m"]) > TOO_FAR + 1e-9:
        _flag("lid peeled past the point it can be closed again")
    tilt = tilt_deg(R)
    S["tilt"] = tilt
    S["upright"] = tilt < UPRIGHT_DEG
    if tilt > TIP_DEG:
        _flag(f"cup tipped ({tilt:.0f} deg)")
    lift = float(cp[2] - (TOP + 0.0005))
    S["max_lift"] = max(S["max_lift"], lift)
    S["on_table"] = lift < LIFT_MAX
    if lift > LIFT_MAX or cp[2] < TOP - 0.05:
        _flag(f"cup left the table ({lift * 1000:.0f} mm)")
    a = arms(environment)
    # the squeeze on the paper cup: each finger is a stiff position servo, so its push is stiffness x how far it is held
    # back from its target; a grip harder than DENT_N on both fingers, with the fingertips at the cup, dents the wall
    robot_f = {}
    for side in a:
        rb = environment.unwrapped.scene[f"{side}_robot"]
        ids = rb.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        q = rb.data.joint_pos[0, ids].detach().cpu().numpy()
        qt = rb.data.joint_pos_target[0, ids].detach().cpu().numpy()
        f = np.clip(FINGER_K * np.abs(q - qt), 0.0, FINGER_MAX)
        tip = np.asarray(a[side]["tip"], float) - cp
        zt = float(R[:, 2] @ tip)
        near = 0.0 <= zt <= A.H - 0.008 and float(np.linalg.norm(tip - zt * R[:, 2])) < _r_at(zt) + 0.02
        robot_f[side] = float(f.min()) if near else 0.0
    S["squeeze"] = max(robot_f.values())
    S["max_squeeze"] = max(S["max_squeeze"], S["squeeze"])
    if S["squeeze"] > DENT_N:
        _flag(f"cup squeezed ({S['squeeze']:.0f} N)")
    tab, _ = b["tab"]
    S["tcp"] = {s: np.asarray(a[s]["tcp"], float) for s in a}
    S["tab_tip"] = tab + quat_matrix(b["tab"][1]) @ np.array([0.0, -A.TAB_L * GRASP_AT * math.cos(A.TAB_UP),
                                                              A.TAB_L * GRASP_AT * math.sin(A.TAB_UP) + A.TAB_T / 2])
    S["released"] = all(float(np.hypot(*(a[s]["tcp"][:2] - cp[:2]))) > LET_GO_CUP
                        and float(np.linalg.norm(a[s]["tcp"] - tab)) > LET_GO_TAB for s in a)
    p = peeled_m(S["m"])
    done = HALF_LO - 1e-9 <= p <= HALF_HI + 1e-9 and S["upright"] and S["on_table"] and S["released"]
    S["done_since"] = (S["done_since"] if S["done_since"] is not None else S["t"]) if done else None


def violation() -> bool:
    return _STATE["violation"] is not None


def score_from(S) -> float:
    if S["violation"] is not None:
        return 0.0
    p = peeled_m(S["m"])
    if HALF_LO - 1e-9 <= p <= HALF_HI + 1e-9:
        if S["done_since"] is not None and S["t"] - S["done_since"] >= SUCCESS_HOLD_S - 1e-6:
            return 1.0
        return 0.5
    if p >= SOME - 1e-9:
        return 0.5
    return 0.25 if S["m"] >= 1 else 0.0


def score(objects) -> float:
    return score_from(_STATE)


def success(objects) -> bool:
    """Lid peeled back 38..57 mm of its 95 mm, the cup upright on the table and let go, for 1 s; never tipped."""
    return score(objects) >= 1.0


def status() -> str:
    S = _STATE
    return (f"m {S['m']} peeled {peeled_m(S['m']) * 1000:.0f}mm F {S['force']:.2f}N tilt {S['tilt']:.0f} "
            f"z {S['cup_pos'][2] - TOP:.3f} sq {S['squeeze']:.1f} rel {S['released']} viol {S['violation']}")
