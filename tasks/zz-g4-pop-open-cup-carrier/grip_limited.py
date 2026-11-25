"""grip_limited.py: force-limited two-pad grasp for YAM tasks (use it instead of a rigid grasp weld / FixedJoint).

Generalised from the GPU-card task (zz-L3-a6000-into-x16), which passed every strict gate with it.

Mechanism (per arm, checked once per control step):
  engage   only when BOTH finger pads touch the object (PhysX contact force on tip_left AND tip_right, filtered to the
           object) AND the jaws are commanded closed past their measured opening (they squeeze). Optional per-task
           `engage_ok` (e.g. "pads inside the card's top grip band"). Never on distance alone.
  hold     a 6-axis spring-damper joint gripper->object anchored at the CURRENT relative pose (no snap). Joint frame =
           gripper frame at the pad midpoint, so rotX = the jaw closing axis (pad normal). Each drive is force-capped:
             pull   hold_n  = 2 pads x mu x squeeze            (default 2 x 0.5 x 50 N = 50 N)
             twist  twist_nm = hold_n x twist_radius           (about the pad normal; default x 10 mm = 0.50 N m)
             tip    tip_nm  = 2 x squeeze x tip_half_len       (in the pad plane; default 2 x 50 N x 15 mm = 1.50 N m)
  slip     when the spring load needed exceeds a cap, the anchor moves to the current pose (the object slides/turns in
           the jaws, Coulomb style). Every slip is logged (grip.slips).
  seat     the anchor is taken once the jaws have stalled on the object (jaw opening steady, or after SETTLE_MAX
           squeezing steps) and follows the object for SEAT_STEPS more steps, so the spring starts unloaded instead of
           preloaded against pads that are still closing (fix 2026-11-22: that preload caused spurious slips in travel).
           After a slip it follows for SLIP_SEAT steps too, so the new anchor is where the object comes to rest.
  release  when the jaws are commanded open (or, in contact mode, both-pad contact is lost for `lose_steps` steps).

Defaults come from the YAM gripper effort limit (50 N, yam_arena.constants.GRIPPER effort, i2rt linear_4310 actuator)
and a rubber-pad friction coefficient of 0.5. Override any of them per task.

USAGE (task.py)
  1. Symlink it next to task.py (submit_task.py and remote.sh resolve symlinks into real files):
       cd tasks_dev/<task> && ln -sf ../../cablekit/grip_limited.py .
  2. In task.py (task.py already does sys.path.insert(0, <its folder>) in the kit pattern):
       import grip_limited as gl
       GRIP = gl.LimitedGrip(lambda: _P.get("card"), label="card")   # body prim path (str, list or callable)
     then either
       GRIP.install()                 # hooks YamArenaEmbodiment._task_step / reset itself, or
       GRIP.update(env); GRIP.reset() # call from your own step hook / reset hook (recommended if you have them)
  3. Read it in scoring: GRIP.holding("left"), GRIP.holders(), GRIP.held_body("right"), GRIP.slips, GRIP.summary().
  4. Text: gl.caps_text(GRIP) for the README sim-fidelity section, gl.caps_text(GRIP, short=True) (<= 500 chars) for
     task.json. Paste the printed text; the numbers then match the code exactly.
  Contact mode needs PhysX contact processing: the module re-enables it (Isaac Lab turns it off when no ContactSensor
  exists). If a task can't get contacts, pass contact="gap", gap_band=(lo, hi) (measured finger-joint opening band,
  in m, that means both pads are pressed on the object's faces) as the GPU card did.

Self-test (no sim):  python3 grip_limited.py
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from typing import Callable, Iterable

import numpy as np

# -- defaults (cite these) -------------------------------------------------------------------------------------------
YAM_SQUEEZE_N = 50.0      # YAM gripper actuator effort limit (yam_arena.constants.GRIPPER effort; i2rt linear_4310)
PAD_MU = 0.5              # rubber finger pad on plastic/metal, static friction (conservative)
PAD_TWIST_R = 0.010       # m, mean radius of the ~20 x 30 mm pad contact patch (twist about the pad normal)
PAD_TIP_HALF = 0.015      # m, half pad length (tipping moment in the pad plane)
K_LIN, C_LIN = 20000.0, 200.0   # N/m, N s/m: pad shear stiffness (~1 mm per 20 N) and damping
K_ROT, C_ROT = 50.0, 0.5        # N m/rad, N m s/rad
CONTACT_MIN_N = 0.05      # N per pad to count as touching
SQUEEZE_MARGIN = 0.001    # m: commanded opening must be this much below the measured opening ("squeezing")
OPEN_MARGIN = 0.002       # m: commanded opening this much above measured -> release
LOSE_STEPS = 5            # control steps without two-pad contact -> release (contact mode); 0 disables
# Seating (2026-11-22 fix): the anchor used to be taken at the FIRST two-pad touch while the jaws were still closing.
# The closing jaws then seated/turned the object in the pads, so the spring sat preloaded against the pad contacts
# (measured on a pizza wheel hanging still in the air: 12.5 N and 1.4 N m tip, against caps of 50 N / 1.5 N m) and any
# small travel load tipped it over a cap -> spurious slips that ratchet the object round in the jaws.
SETTLE_M = 0.0002         # m: jaw opening change per control step below which the jaws count as stalled (seated)
SETTLE_MAX = 10           # control steps of squeezing with two-pad contact after which it engages anyway
SEAT_STEPS = 6            # control steps after engage during which the anchor follows the object (no load, no slip)
SLIP_SEAT = 2             # control steps after a slip during which the anchor keeps following the object, so it is
                          # re-anchored where the object comes to rest in the pads, not at a pose caught mid-slide
SOURCES = ("YAM gripper effort limit 50 N per finger (yam_arena GRIPPER effort, i2rt linear_4310 actuator); "
           "rubber pad friction mu {mu:g}; pad patch ~20 x 30 mm")

_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"
_ALL: list["LimitedGrip"] = []


def _np(t):
    t = getattr(t, "torch", t)
    if hasattr(t, "detach"):
        return np.asarray(t.detach().cpu().numpy(), dtype=np.float64)
    if hasattr(t, "numpy"):  # warp array (tensor-API views)
        return np.asarray(t.numpy(), dtype=np.float64)
    return np.asarray(t, dtype=np.float64)


def _qwxyz_to_R(w, x, y, z) -> np.ndarray:
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _rotvec(R: np.ndarray) -> np.ndarray:
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.linalg.norm(v)
    if s < 1e-12:  # ~180 deg
        w, V = np.linalg.eigh((R + np.eye(3)) / 2.0)
        return V[:, -1] * ang
    return v / s * ang


def _gfquat(R):
    from pxr import Gf

    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    if w > 1e-6:
        x, y, z = (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        r = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k]))
        q = [0.0, 0.0, 0.0]
        q[i] = r / 2
        q[j] = (R[j, i] + R[i, j]) / (2 * r)
        q[k] = (R[k, i] + R[i, k]) / (2 * r)
        w = (R[k, j] - R[j, k]) / (2 * r)
        x, y, z = q
    return Gf.Quatf(float(w), float(x), float(y), float(z))


# -- contact processing (Isaac Lab disables it when the scene has no ContactSensor) -------------------------------------
def enable_contacts() -> None:
    for fn in (
        lambda: __import__("carb").settings.get_settings().set_bool("/physics/disableContactProcessing", False),
        lambda: __import__("isaaclab.app.settings_manager", fromlist=["x"]).get_settings_manager().set_bool(
            "/physics/disableContactProcessing", False),
    ):
        try:
            fn()
        except Exception:  # noqa: BLE001
            pass


def _patch_mgr(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr._configure_physics, "_gl_hooked", False):
        return
    base = mgr._configure_physics.__func__

    def _configure_physics(cls) -> None:
        base(cls)
        enable_contacts()

    _configure_physics._gl_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)


class _MgrFinder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path, target=None):
        if fullname != _PHYSX_MGR:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig = spec.loader.exec_module

        def exec_module(module):
            orig(module)
            _patch_mgr(module)

        spec.loader.exec_module = exec_module
        return spec


def _hook_contacts() -> None:
    if _PHYSX_MGR in sys.modules:
        _patch_mgr(sys.modules[_PHYSX_MGR])
        enable_contacts()
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())


# -- the grip ---------------------------------------------------------------------------------------------------------
class LimitedGrip:
    """Force-limited two-pad grasp on one or more rigid bodies, for the left and right YAM grippers."""

    def __init__(
        self,
        bodies,
        *,
        label: str = "object",
        name: str = "limited_grip",
        sides: Iterable[str] = ("left", "right"),
        joint_root: str | Callable[[], str] | None = None,
        squeeze_n: float = YAM_SQUEEZE_N,
        mu: float = PAD_MU,
        twist_radius: float = PAD_TWIST_R,
        tip_half_len: float = PAD_TIP_HALF,
        hold_n: float | None = None,
        twist_nm: float | None = None,
        tip_nm: float | None = None,
        k_lin: float = K_LIN, c_lin: float = C_LIN, k_rot: float = K_ROT, c_rot: float = C_ROT,
        contact: str = "pads",
        contact_min_n: float = CONTACT_MIN_N,
        gap_band: tuple[float, float] | None = None,
        engage_ok: Callable[[str, np.ndarray, str], bool] | None = None,
        pose_fn: Callable[[str], tuple[np.ndarray, np.ndarray]] | None = None,
        lose_steps: int = LOSE_STEPS,
        log_prefix: str = "[grip]",
        max_log: int = 20,
    ):
        """
        bodies       rigid-body prim path(s) that may be gripped: str, list of str, or a callable returning either
                     (paths are usually only known after authoring, e.g. lambda: _P.get("card")).
        label        what the README calls the object ("card", "lid").
        contact      "pads": PhysX contact on both pads (default, the reviewer rule) | "gap": jaw opening in gap_band.
        gap_band     (lo, hi) m of measured finger-joint opening |q7|+|q8| meaning both pads pressed (gap mode; also
                     accepted as an extra check in pads mode when given).
        engage_ok    optional f(side, pad_mid_world, body_path) -> bool, e.g. pads must be on a handle band.
        pose_fn      optional f(body_path) -> (pos_world[3], R_world[3x3]); default reads PhysX.
        hold_n / twist_nm / tip_nm override the derived caps directly.
        """
        self._bodies = bodies
        self.label = label
        self.name = name
        self.sides = tuple(sides)
        self._joint_root = joint_root
        self.squeeze_n, self.mu = float(squeeze_n), float(mu)
        self.twist_radius, self.tip_half_len = float(twist_radius), float(tip_half_len)
        self.hold_n = float(hold_n) if hold_n is not None else 2.0 * self.mu * self.squeeze_n
        self.twist_nm = float(twist_nm) if twist_nm is not None else self.hold_n * self.twist_radius
        self.tip_nm = float(tip_nm) if tip_nm is not None else 2.0 * self.squeeze_n * self.tip_half_len
        self.k_lin, self.c_lin, self.k_rot, self.c_rot = k_lin, c_lin, k_rot, c_rot
        if contact not in ("pads", "gap"):
            raise ValueError("contact must be 'pads' or 'gap'")
        if contact == "gap" and gap_band is None:
            raise ValueError("contact='gap' needs gap_band=(lo, hi)")
        self.contact, self.contact_min_n, self.gap_band = contact, float(contact_min_n), gap_band
        self.engage_ok, self.pose_fn = engage_ok, pose_fn
        self.lose_steps, self.log_prefix, self.max_log = int(lose_steps), log_prefix, int(max_log)
        self._paths: dict = {}
        self._cviews: dict = {}
        self._rbviews: dict = {}
        self.reset()
        if contact == "pads":
            _hook_contacts()
        _ALL.append(self)

    # -- state ---------------------------------------------------------------------------------------------------------
    def reset(self) -> None:
        """Disable the grip joints and clear the episode state (call from the task's reset hook)."""
        for side in self.sides:
            self._set_enabled(side, False)
        self.step = 0
        self.held: dict = {s: None for s in self.sides}      # side -> body path
        self.anchor: dict = {s: None for s in self.sides}    # side -> (p0 in gripper, p1 in body, R1 body<-joint)
        self.lost: dict = {s: 0 for s in self.sides}
        self.cmd_at_grip: dict = {}
        self.prev_ap: dict = {s: None for s in self.sides}
        self.prev_cmd: dict = {}
        self.sq_steps: dict = {s: 0 for s in self.sides}
        self.seat: dict = {s: 0 for s in self.sides}
        self.peak: dict = {s: {"load_n": 0.0, "twist_nm": 0.0, "tip_nm": 0.0} for s in self.sides}
        self.slips: list[dict] = []
        self.events: list[dict] = []
        self.pad_f: dict = {s: (0.0, 0.0) for s in self.sides}
        self.err = None

    def holding(self, side: str) -> bool:
        return self.held.get(side) is not None

    def held_body(self, side: str):
        return self.held.get(side)

    def holders(self, body: str | None = None) -> int:
        return sum(1 for s in self.sides if self.held[s] is not None and (body is None or self.held[s] == body))

    def summary(self) -> dict:
        return dict(caps=dict(hold_n=self.hold_n, twist_nm=self.twist_nm, tip_nm=self.tip_nm), slips=len(self.slips),
                    peak=self.peak, held={s: self.held[s] for s in self.sides}, events=self.events[-10:])

    # -- helpers -------------------------------------------------------------------------------------------------------
    def bodies(self) -> list[str]:
        b = self._bodies() if callable(self._bodies) else self._bodies
        if not b:
            return []
        return [b] if isinstance(b, str) else [x for x in b if x]

    def _log(self, msg: str) -> None:
        print(f"{self.log_prefix} {msg}", flush=True)

    def _stage(self):
        from isaaclab.sim.utils.stage import get_current_stage

        return get_current_stage()

    def _jpath(self, side: str) -> str | None:
        root = self._joint_root() if callable(self._joint_root) else self._joint_root
        if not root:
            b = self.bodies()
            if not b:
                return None
            root = b[0].rsplit("/", 1)[0]
        return f"{root}/{self.name}_{side}"

    def _set_enabled(self, side: str, on: bool) -> None:
        try:
            p = self._jpath(side)
            if p is None:
                return
            j = self._stage().GetPrimAtPath(p)
            if j and j.IsValid():
                j.GetAttribute("physics:jointEnabled").Set(bool(on))
        except Exception:  # noqa: BLE001
            pass

    def _link(self, side: str, link: str) -> str | None:
        key = (id(self._stage()), side, link)
        if key in self._paths:
            return self._paths[key]
        from pxr import UsdPhysics

        arm = f"/{side.capitalize()}Arm"
        for prim in self._stage().Traverse():
            ps = prim.GetPath().pathString
            if "/env_0/" in ps and arm in ps and ps.endswith(f"/{link}") and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                self._paths[key] = ps
                return ps
        return None

    def _sim_view(self):
        from isaaclab_physx.physics import PhysxManager

        return PhysxManager.get_physics_sim_view()

    def body_pose(self, path: str, env=None) -> tuple[np.ndarray, np.ndarray]:
        """World position and rotation matrix of a rigid body."""
        if self.pose_fn is not None:
            p, R = self.pose_fn(path)
            return np.asarray(p, dtype=np.float64), np.asarray(R, dtype=np.float64)
        if env is not None:
            try:
                pose = _np(env.unwrapped.arena_world.get_pose_w(path.rsplit("/", 1)[-1])[0]).reshape(-1)
                x, y, z, w = pose[3:7]
                return pose[:3], _qwxyz_to_R(w, x, y, z)
            except Exception:  # noqa: BLE001
                pass
        sv = self._sim_view()
        key = (id(sv), path)
        if key not in self._rbviews:
            self._rbviews[key] = sv.create_rigid_body_view(path)
        T = _np(self._rbviews[key].get_transforms()).reshape(-1)
        return T[:3], _qwxyz_to_R(T[6], T[3], T[4], T[5])

    def pad_forces(self, side: str) -> dict:
        """{body_path: (F_tip_left, F_tip_right)} PhysX normal contact force [N] of each pad against each body."""
        bodies = self.bodies()
        sv = self._sim_view()
        if sv is None or not bodies:
            return {}
        key = (id(sv), side, tuple(bodies))
        if key not in self._cviews:
            enable_contacts()
            views = []
            for link in ("tip_left", "tip_right"):
                lp = self._link(side, link)
                if lp is None:
                    return {}
                views.append(sv.create_rigid_contact_view([lp], filter_patterns=[list(bodies)],
                                                          max_contact_data_count=256))
            self._cviews[key] = views
            self._log(f"{side}: pad contact views on {len(bodies)} body(ies)")
        from yam_arena.constants import PHYSICS_DT

        out = {}
        mats = [_np(v.get_contact_force_matrix(dt=PHYSICS_DT)).reshape(len(bodies), 3) for v in self._cviews[key]]
        for i, b in enumerate(bodies):
            out[b] = (float(np.linalg.norm(mats[0][i])), float(np.linalg.norm(mats[1][i])))
        return out

    # -- per control step ----------------------------------------------------------------------------------------------
    def update(self, env) -> None:
        """Engage / hold / slip / release for every side. Call once per control step after the physics step."""
        self.step += 1
        try:
            for side in self.sides:
                self._update_side(env, side)
        except Exception as e:  # noqa: BLE001
            if self.err is None:
                self.err = repr(e)
                import traceback

                self._log(f"update error: {e!r}\n{traceback.format_exc()}")

    def _jaws(self, rob) -> tuple[float, float]:
        from yam_arena.constants import FINGER_JOINTS

        ids = rob.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        ap = float(np.abs(_np(rob.data.joint_pos[0])[ids]).sum())
        cmd = float(np.abs(_np(rob.data.joint_pos_target[0])[ids]).sum())
        return ap, cmd

    def _update_side(self, env, side: str) -> None:
        from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

        rob = env.unwrapped.scene[f"{side}_robot"]
        ap, cmd = self._jaws(rob)
        dap = abs(ap - self.prev_ap[side]) if self.prev_ap.get(side) is not None else 1.0
        self.prev_ap[side] = ap
        widening = cmd > self.prev_cmd.get(side, cmd) + 1e-5  # the jaw command is opening (no new grasp then)
        self.prev_cmd[side] = cmd
        squeezing = cmd < ap - SQUEEZE_MARGIN
        # opening: commanded wider than the jaws are, or wider than the command they gripped with (the gripper action
        # may ramp its target, so the first test alone can lag)
        opening = cmd > ap + OPEN_MARGIN or cmd > self.cmd_at_grip.get(side, 1.0) + OPEN_MARGIN
        held = self.held[side]
        g = link_poses(rob)["gripper"]
        gp, gR = g[:3, 3], g[:3, :3]

        forces = {}
        if self.contact == "pads" and (held is not None or squeezing):
            forces = self.pad_forces(side)

        if held is not None:
            why = None
            if opening:
                why = "jaws opened"
            elif self.contact == "pads" and self.lose_steps > 0:
                fl, fr = forces.get(held, (0.0, 0.0))
                self.pad_f[side] = (fl, fr)
                self.lost[side] = 0 if min(fl, fr) >= self.contact_min_n else self.lost[side] + 1
                if self.lost[side] >= self.lose_steps:
                    why = f"two-pad contact lost for {self.lose_steps} steps"
            elif self.contact == "gap" and not (self.gap_band[0] <= ap <= self.gap_band[1]):
                why = f"jaw opening {ap * 1000:.1f} mm left the grip band"
            if why:
                self._set_enabled(side, False)
                self.held[side] = None
                self.anchor[side] = None
                self.events.append(dict(step=self.step, side=side, ev="release", body=held, why=why))
                self._log(f"{side} released {held.rsplit('/', 1)[-1]} at step {self.step} ({why}; peak load "
                          f"{self.peak[side]['load_n']:.1f} N, slips {len(self.slips)})")
            elif self.seat[side] > 0:
                # still seating: the anchor follows the object (the jaws finish closing and settle it in the pads)
                self.seat[side] -= 1
                self._reanchor(env, side, gp, gR)
            else:
                self._slip(env, side, gp, gR)
            return

        if not squeezing or widening:
            self.sq_steps[side] = 0
            return
        tcp = np.asarray(tcp_world(rob), dtype=np.float64)
        tip = np.asarray(pad_tips_world(rob), dtype=np.float64)
        mid = 0.5 * (tcp + tip)  # pad-midpoint
        cand = []
        for b in self.bodies():
            if self.contact == "pads":
                fl, fr = forces.get(b, (0.0, 0.0))
                if min(fl, fr) < self.contact_min_n:
                    continue
                if self.gap_band is not None and not (self.gap_band[0] <= ap <= self.gap_band[1]):
                    continue
                f = min(fl, fr)
            else:
                if not (self.gap_band[0] <= ap <= self.gap_band[1]):
                    continue
                f = 0.0
            if self.engage_ok is not None and not self.engage_ok(side, mid, b):
                continue
            cand.append((f, b, fl if self.contact == "pads" else 0.0, fr if self.contact == "pads" else 0.0))
        if not cand:
            return
        self.sq_steps[side] += 1
        if dap > SETTLE_M and self.sq_steps[side] < SETTLE_MAX:
            return  # jaws still closing: anchor only once they have stalled on the object
        f, b, fl, fr = max(cand)
        self._engage(env, side, b, mid, gp, gR, ap, fl, fr)

    def _engage(self, env, side, body, mid, gp, gR, ap, fl, fr) -> None:
        from pxr import Gf, Sdf, UsdPhysics

        gpath = self._link(side, "gripper")
        jp = self._jpath(side)
        if gpath is None or jp is None:
            return
        kp, kR = self.body_pose(body, env)
        p0 = gR.T @ (mid - gp)          # pad midpoint in gripper frame
        p1 = kR.T @ (mid - kp)          # same point in body frame
        R1 = kR.T @ gR                  # joint frame (= gripper axes) seen from the body
        st = self._stage()
        j = st.GetPrimAtPath(jp)
        if not (j and j.IsValid()):
            jj = UsdPhysics.Joint.Define(st, jp)
            jj.CreateBody0Rel().SetTargets([Sdf.Path(gpath)])
            jj.CreateBody1Rel().SetTargets([Sdf.Path(body)])
            jj.CreateExcludeFromArticulationAttr(True)
            jj.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            prim = jj.GetPrim()
            d2r = math.pi / 180.0
            for ax in ("transX", "transY", "transZ"):
                d = UsdPhysics.DriveAPI.Apply(prim, ax)
                d.CreateTypeAttr("force"); d.CreateStiffnessAttr(self.k_lin); d.CreateDampingAttr(self.c_lin)
                d.CreateMaxForceAttr(self.hold_n); d.CreateTargetPositionAttr(0.0)
            for ax, cap in (("rotX", self.twist_nm), ("rotY", self.tip_nm), ("rotZ", self.tip_nm)):
                d = UsdPhysics.DriveAPI.Apply(prim, ax)  # USD angular gains are per degree
                d.CreateTypeAttr("force"); d.CreateStiffnessAttr(self.k_rot * d2r); d.CreateDampingAttr(self.c_rot * d2r)
                d.CreateMaxForceAttr(cap); d.CreateTargetPositionAttr(0.0)
            j = prim
        else:
            j.GetAttribute("physics:jointEnabled").Set(False)
            UsdPhysics.Joint(j).GetBody1Rel().SetTargets([Sdf.Path(body)])
        j.GetAttribute("physics:localPos0").Set(Gf.Vec3f(*map(float, p0)))
        j.GetAttribute("physics:localPos1").Set(Gf.Vec3f(*map(float, p1)))
        j.GetAttribute("physics:localRot1").Set(_gfquat(R1))
        j.GetAttribute("physics:jointEnabled").Set(True)
        self.anchor[side] = (p0, p1, R1)
        self.cmd_at_grip[side] = self._jaws(env.unwrapped.scene[f"{side}_robot"])[1]
        self.held[side] = body
        self.lost[side] = 0
        self.seat[side] = SEAT_STEPS
        self.sq_steps[side] = 0
        self.events.append(dict(step=self.step, side=side, ev="engage", body=body, ap_mm=round(ap * 1000, 1),
                                pad_n=(round(fl, 2), round(fr, 2))))
        how = f"pads {fl:.2f}/{fr:.2f} N" if self.contact == "pads" else "pads pressed (gap band)"
        self._log(f"{side} grip ON {body.rsplit('/', 1)[-1]} at step {self.step} ({how}, jaw {ap * 1000:.1f} mm)")

    def loads(self, side: str, gp, gR, kp, kR) -> tuple[float, float, float, np.ndarray, np.ndarray]:
        """Spring load (N), twist (N m), tip (N m) the anchor currently asks for, plus the current p1/R1."""
        p0, p1, R1 = self.anchor[side]
        pg = gp + gR @ p0
        pb = kp + kR @ p1
        f = self.k_lin * float(np.linalg.norm(pb - pg))
        rv = _rotvec(gR.T @ kR @ R1)  # in gripper/joint frame: x = pad normal
        tw = self.k_rot * abs(float(rv[0]))
        tp = self.k_rot * float(math.hypot(rv[1], rv[2]))
        return f, tw, tp, kR.T @ (pg - kp), kR.T @ gR

    def _move_anchor(self, side, p1n, R1n) -> None:
        j = self._stage().GetPrimAtPath(self._jpath(side))
        if j and j.IsValid():
            from pxr import Gf

            j.GetAttribute("physics:localPos1").Set(Gf.Vec3f(*map(float, p1n)))
            j.GetAttribute("physics:localRot1").Set(_gfquat(R1n))
        self.anchor[side] = (self.anchor[side][0], p1n, R1n)

    def _reanchor(self, env, side, gp, gR) -> None:
        """Move the anchor to the current relative pose without counting a slip (seating)."""
        kp, kR = self.body_pose(self.held[side], env)
        *_, p1n, R1n = self.loads(side, gp, gR, kp, kR)
        self._move_anchor(side, p1n, R1n)

    def _slip(self, env, side, gp, gR) -> None:
        body = self.held[side]
        kp, kR = self.body_pose(body, env)
        f, tw, tp, p1n, R1n = self.loads(side, gp, gR, kp, kR)
        pk = self.peak[side]
        pk["load_n"], pk["twist_nm"], pk["tip_nm"] = max(pk["load_n"], f), max(pk["twist_nm"], tw), max(pk["tip_nm"], tp)
        if f <= self.hold_n and tw <= self.twist_nm and tp <= self.tip_nm:
            return
        self._move_anchor(side, p1n, R1n)
        self.seat[side] = SLIP_SEAT
        rec = dict(step=self.step, side=side, body=body, load_n=round(f, 2), twist_nm=round(tw, 3), tip_nm=round(tp, 3))
        self.slips.append(rec)
        n = len(self.slips)
        if n <= self.max_log or n % 50 == 0:
            self._log(f"{side} slip #{n} at step {self.step}: load {f:.1f}/{self.hold_n:.0f} N, twist {tw:.2f}/"
                      f"{self.twist_nm:.2f} N m, tip {tp:.2f}/{self.tip_nm:.2f} N m")

    # -- optional self-installing hooks --------------------------------------------------------------------------------
    def install(self) -> None:
        """Hook YamArenaEmbodiment so update()/reset() run on their own (skip if your task calls them itself)."""
        try:
            from yam_arena import inspect_embodiment as ie
        except Exception:  # noqa: BLE001
            return
        cls = ie.YamArenaEmbodiment
        if getattr(cls.reset, "_task_reset_hooks", None) is None:
            base_reset = cls.reset

            def reset(emb, *a, **k):
                for fn in reset._task_reset_hooks:
                    fn()
                return base_reset(emb, *a, **k)

            reset._task_reset_hooks = []
            cls.reset = reset
        if self.reset not in cls.reset._task_reset_hooks:
            cls.reset._task_reset_hooks.append(self.reset)
        if not getattr(cls._task_step, "_gl_hooked", False):
            base_step = cls._task_step

            def _task_step(emb, *a, **k):
                out = base_step(emb, *a, **k)
                for g in _ALL:
                    if getattr(g, "_installed", False):
                        g.update(emb._env)
                return out

            _task_step._gl_hooked = True
            cls._task_step = _task_step
        self._installed = True


# -- README / task.json text -------------------------------------------------------------------------------------------
def caps_text(grip: LimitedGrip, short: bool = False) -> str:
    """Sentence(s) stating the grasp model, its caps and their sources, with the code's own numbers."""
    g = grip
    if g.contact == "pads":
        trig = f"both finger pads touch the {g.label} (PhysX contact on each pad) and the jaws are commanded closed"
    else:
        trig = (f"the jaws are commanded closed and stall on the {g.label} with both pads pressed on its faces "
                f"(finger opening {g.gap_band[0] * 1000:.1f}-{g.gap_band[1] * 1000:.1f} mm)")
    derived_hold = abs(g.hold_n - 2 * g.mu * g.squeeze_n) < 1e-9
    hold = (f"{g.hold_n:g} N pull (2 pads x mu {g.mu:g} x {g.squeeze_n:g} N squeeze)" if derived_hold
            else f"{g.hold_n:g} N pull")
    twist = f"{g.twist_nm:.2f} N m twist about the pad normal"
    tip = f"{g.tip_nm:.2f} N m tipping"
    if short:
        return (f"Grasp: force-limited pad grip, not a weld. It engages only when {trig}, holds through a compliant "
                f"spring-damper anchored at the current pose (no snap), capped at {hold}, {twist}, {tip}; above a "
                f"cap the {g.label} slips in the jaws. Opening the jaws releases it.")[:500]
    src = SOURCES.format(mu=g.mu)
    geo = (f" Twist cap = pull cap x {g.twist_radius * 1000:g} mm mean pad-patch radius; tipping cap = 2 x "
           f"{g.squeeze_n:g} N x {g.tip_half_len * 1000:g} mm half pad length.")
    rel = f" It releases when the jaws are commanded open" + (
        f" or both-pad contact is lost for {g.lose_steps} control steps." if g.contact == "pads" and g.lose_steps
        else ".")
    return (f"Grasp model (sim stand-in for pad friction; no rigid weld): a 6-axis spring-damper joint between gripper "
            f"and {g.label} engages only when {trig}. It is anchored at the current relative pose, so nothing snaps. "
            f"Each axis is force-limited to what two rubber pads can transmit: {hold}, {twist}, {tip}.{geo} When the "
            f"load needed exceeds a cap, the {g.label} slips in the jaws (the anchor moves) and the slip is logged."
            f"{rel} Sources: {src}.")


if __name__ == "__main__":  # offline self-test (no sim)
    g = LimitedGrip("/World/envs/env_0/obj", label="card")
    assert abs(g.hold_n - 50.0) < 1e-9 and abs(g.twist_nm - 0.5) < 1e-9 and abs(g.tip_nm - 1.5) < 1e-9
    g2 = LimitedGrip("/x", mu=0.4, squeeze_n=30, contact="gap", gap_band=(0.041, 0.045))
    assert abs(g2.hold_n - 24.0) < 1e-9
    R = _qwxyz_to_R(math.cos(0.2), math.sin(0.2), 0, 0)
    assert np.allclose(_rotvec(R), [0.4, 0, 0], atol=1e-9)
    # loads(): a 2 mm offset reads 40 N, a 0.02 rad twist reads 1 N m
    g.anchor["left"] = (np.zeros(3), np.zeros(3), np.eye(3))
    f, tw, tp, *_ = g.loads("left", np.zeros(3), np.eye(3), np.array([0.002, 0, 0]), _qwxyz_to_R(math.cos(0.01), math.sin(0.01), 0, 0))
    assert abs(f - 40.0) < 1e-6 and abs(tw - 1.0) < 1e-6 and tp < 1e-6, (f, tw, tp)
    s = caps_text(g, short=True)
    assert len(s) <= 500, len(s)
    print(caps_text(g))
    print(f"\nshort ({len(s)} chars): {s}")
    print(f"\ngap mode: {caps_text(g2, short=True)}")
    print("\nself-test OK")
