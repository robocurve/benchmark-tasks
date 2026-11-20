"""The saw's runtime on the simulator (GPU host only): the trigger, the cut model's push on the blade, and the kerf.

- The trigger is pulled while PhysX reports the robot pressing the trigger's face with at least F_TRIGGER (a rigid
  contact view: the saw against every robot link, as in tasks/move-live-wire-with-stick).
- The blade's collider does not collide with the pipe (a PhysX filtered pair, authored when the saw is spawned): it
  still meets everything else. It rests instead on a small kinematic pad that cut_model places, before every physics
  substep, under the uncut pipe the blade meets first; PhysX takes the blade's push on it, and that push (from the
  contact view) is the feed the model cuts with. The model's other forces (a kerf's side, the pull stroke's drag) are
  applied to the saw through Isaac Lab, as the screw model of the repository's stud and lever-set tasks applies its.
- The water line (PEX) under the drain is an ordinary solid: the blade meets it. The same contact view reports the
  blade pressing on it, and the largest such push in the trial is kept (any touch scores 0).
- The kerf is shown by small dark marks (kinematic, without colliders) on the pipe's outside, each moved from under
  the table onto the pipe once the cut has passed that point of the circumference.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np

import cut_model as CM

SAW, PIPE, PLATE, PEX = "saw", "pipe", "kerf_plate", "pex_line"
PLATE_SIZE = (0.006, 0.004, 0.004)  # across the kerf, across the pipe, height
PLATE_HIDDEN = np.zeros(3)
EDGE_LOCAL = (np.zeros(3), np.zeros(3))  # the cutting edge's ends, saw frame (set by the task file)
NORMAL_LOCAL = np.array((0.0, 1.0, 0.0))  # the blade's side normal, saw frame
TRIGGER_LOCAL = (np.zeros(3), np.zeros(3))  # the trigger's face region (low and high corners), saw frame
BLADE_LOCAL = (np.zeros(3), np.zeros(3))  # the blade's region (low and high corners), saw frame
F_TRIGGER = 35.0  # N
MARKS: tuple = ()  # the kerf marks' names, round the pipe from its top
MARK_HIDDEN = np.zeros(3)  # where the marks wait, under the table
MARK_LIFT = 0.0  # a mark's centre this far outside... (set by the task file)

STATE = {"trigger": 0.0, "pulled": False, "pex": 0.0, "env": None, "shown": set()}  # pex: the blade's top push on it
_PHYSX_MGR = "isaaclab_physx.physics.physx_manager"
_VIEW = {"view": None, "err": None}
_DEBUG = bool(os.environ.get("CUT_DEBUG"))


def _np(x):
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


def _quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))))


def _matrix_quat(R):
    """XYZW."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array(((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s))
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


# -- contact reports (Isaac Lab switches PhysX contact processing off unless one of its ContactSensors exists) --------
def _patch_manager(mod) -> None:
    mgr = mod.PhysxManager
    if getattr(mgr._configure_physics, "_cut_hooked", False):
        return
    base_cfg = mgr._configure_physics.__func__

    def _configure_physics(cls) -> None:
        base_cfg(cls)
        try:
            import carb

            carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        except Exception as e:  # noqa: BLE001
            print(f"CUT_CONTACTS could not switch contact processing on: {e!r}", flush=True)

    _configure_physics._cut_hooked = True
    mgr._configure_physics = classmethod(_configure_physics)


class _MgrFinder(importlib.abc.MetaPathFinder):
    """Patches Isaac Lab's PhysxManager when it is first imported (it needs Kit, so not at task import)."""

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
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_manager(module)

        spec.loader.exec_module = exec_module
        return spec


def prepare() -> None:
    """Called at task import: contact processing on when the physics starts, and the embodiment hooked."""
    if _PHYSX_MGR in sys.modules:
        _patch_manager(sys.modules[_PHYSX_MGR])
    elif not any(isinstance(f, _MgrFinder) for f in sys.meta_path):
        sys.meta_path.insert(0, _MgrFinder())
    _hook_embodiment()


def _make_view():
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    sv = sys.modules[_PHYSX_MGR].PhysxManager.get_physics_sim_view()
    if sv is None:
        return
    saw, plate, pex, robot = None, None, None, []
    for prim in get_current_stage().Traverse():
        p = prim.GetPath().pathString
        if "/env_0/" not in p or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        leaf = p.rsplit("/", 1)[-1]
        if leaf == SAW:
            saw = p
        elif leaf == PLATE:
            plate = p
        elif leaf == PEX:
            pex = p
        elif "/LeftArm/" in p or "/RightArm/" in p:
            robot.append(p)
    if saw is None or plate is None or pex is None or not robot:
        raise RuntimeError(f"cut contacts: saw {saw}, plate {plate}, water line {pex}, {len(robot)} robot links")
    filters = [plate, pex] + robot
    _VIEW["view"] = sv.create_rigid_contact_view([saw], filter_patterns=[filters],
                                                 max_contact_data_count=64 * len(filters))
    print(f"CUT_CONTACTS view: saw x (plate + water line + {len(robot)} robot links)", flush=True)


def _contacts(origin, p_saw, R_saw, dt):
    """(the robot's push on the trigger's face, the plate's push on the blade, the blade's push on the water line), N,
    from the last physics substep."""
    try:
        if _VIEW["view"] is None:
            _make_view()
        view = _VIEW["view"]
        if view is None:
            return 0.0, 0.0, 0.0
        forces, points, normals, _, counts, starts = view.get_contact_data(dt=dt)
    except Exception as e:  # noqa: BLE001
        if _VIEW["err"] is None:
            _VIEW["err"] = repr(e)
            print(f"CUT_CONTACTS error: {e!r}", flush=True)
        return 0.0, 0.0, 0.0
    counts = _np(counts).astype(int).reshape(-1)
    if not counts.any():
        return 0.0, 0.0, 0.0
    starts = _np(starts).astype(int).reshape(-1)
    forces, points = _np(forces).reshape(-1), _np(points).reshape(-1, 3)
    trigger = feed = pex = 0.0
    for k in range(len(counts)):
        for j in range(starts[k], starts[k] + counts[k]):
            f = abs(float(forces[j]))  # the view's forces are signed by its normal convention
            if k == 0:
                feed += f
                continue
            local = R_saw.T @ (points[j] - origin - p_saw)
            lo, hi = BLADE_LOCAL if k == 1 else TRIGGER_LOCAL
            if not (np.all(local >= lo) and np.all(local <= hi)):
                continue
            if k == 1:
                pex += f
            else:
                trigger += f
    return trigger, feed, pex


# -- the model, before every physics substep ---------------------------------------------------------------------------
def install(environment):
    """Run the cut model before every physics substep (Isaac Lab calls ``write_data_to_sim`` then) and apply its
    forces to the saw. Idempotent."""
    import warp as wp
    from yam_arena.constants import PHYSICS_DT

    base = environment.unwrapped
    saw = base.scene.rigid_objects[SAW]
    plate = base.scene.rigid_objects[PLATE]
    _VIEW["view"] = None  # rebuilt on the next substep (the environment may have been rebuilt)
    if getattr(saw, "_cut_hooked", False):
        return
    orig = saw.write_data_to_sim
    origin = _np(base.scene.env_origins)[0][:3]
    log = {"n": 0}

    def write():
        import torch

        orig()
        pose = _np(saw.data.root_link_pose_w)[0]
        vel = _np(saw.data.root_link_vel_w)[0]
        p, R = pose[:3] - origin, _quat_matrix(pose[3:7])
        v, w = vel[:3], vel[3:6]
        f_trig, feed, f_pex = _contacts(origin, p, R, PHYSICS_DT)
        STATE["trigger"] = f_trig
        STATE["pulled"] = f_trig >= F_TRIGGER
        if f_pex > STATE["pex"]:
            if STATE["pex"] == 0.0:
                print(f"CUT_PEX the blade touched the water line ({f_pex:.2f} N)", flush=True)
            STATE["pex"] = f_pex
        edge = [p + R @ e for e in EDGE_LOCAL]
        edge_vel = [v + np.cross(w, R @ e) for e in EDGE_LOCAL]
        pushes = CM.MODEL.step(edge, R @ NORMAL_LOCAL, feed, STATE["pulled"], PHYSICS_DT, edge_vel)
        # The plate: its top face where the model has it, or out of the way under the table.
        if CM.MODEL.plate is None:
            centre, rot = PLATE_HIDDEN, np.eye(3)
        else:
            top, rot, _ = CM.MODEL.plate
            centre = top - rot[:, 2] * PLATE_SIZE[2] / 2
        plate.write_root_link_pose_to_sim_index(root_pose=torch.tensor(
            [[*(centre + origin), *_matrix_quat(rot)]], dtype=torch.float32, device=base.device))
        log["n"] += 1
        if log["n"] % 24 == 0 and (STATE["trigger"] > 5.0 or feed > 0.0 or CM.MODEL.kerfs):
            k = CM.MODEL.main_kerf()
            extra = "" if k is None else (f" kerf x {k.x() * 1000:+.1f} mm sq {math.degrees(k.squareness()):.2f} deg "
                                          f"round {k.round_share():.2f} through {k.through()}")
            mid = 0.5 * (edge[0] + edge[1])
            teeth = mid[2] - (CM.CENTER[2] + CM.R_OUT)
            mode = "none" if CM.MODEL.plate is None else CM.MODEL.plate[2]
            print(f"CUT_STATE trigger {STATE['trigger']:.1f} N pulled {STATE['pulled']} feed {feed:.1f} N "
                  f"cutting {CM.MODEL.cutting} plate {mode} teeth {teeth * 1000:+.1f} mm over the pipe's top "
                  f"water line {STATE['pex']:.2f} N kerfs {len(CM.MODEL.kerfs)}{extra}", flush=True)
        if _DEBUG and STATE["pulled"] and log["n"] % 6 == 0:
            print(f"CUT_POSE {log['n']} saw {np.round(p * 1000, 1).tolist()} vel {np.round(v * 1000, 0).tolist()} "
                  f"feed {feed:.1f} normal {np.round(R @ NORMAL_LOCAL, 3).tolist()}", flush=True)
        if not pushes:
            return
        f_tot, t_tot = np.zeros(3), np.zeros(3)
        com = _np(saw.data.body_com_pos_w)[0].reshape(-1)[:3] - origin if hasattr(saw.data, "body_com_pos_w") else p
        for f, at in pushes:
            f_tot += f
            t_tot += np.cross(np.asarray(at) - com, f)
        dev = base.device
        saw.root_view.apply_forces_and_torques_at_position(
            force_data=wp.array(f_tot.astype(np.float32), dtype=wp.float32, device=dev),
            torque_data=wp.array(t_tot.astype(np.float32), dtype=wp.float32, device=dev),
            position_data=None, indices=saw._ALL_INDICES, is_global=True)

    saw.write_data_to_sim = write
    saw._cut_hooked = True


# -- the kerf's marks, once per control step ---------------------------------------------------------------------------
def mark_pose(i: int, kerf) -> tuple[np.ndarray, np.ndarray]:
    """Mark ``i``'s centre and rotation on the pipe's outside, in the kerf's plane (marks go round from the top)."""
    t = 2 * math.pi * (i + 0.5) / len(MARKS)
    side = np.cross(CM.UP, CM.AXIS)
    radial = math.sin(t) * side + math.cos(t) * CM.UP
    x = kerf.x()
    centre = CM.CENTER + x * CM.AXIS + (CM.R_OUT + MARK_LIFT) * radial
    n = kerf.normal
    radial = radial - (radial @ n) * n
    radial /= np.linalg.norm(radial)
    tangent = np.cross(n, radial)
    return centre, np.column_stack((n, tangent, radial))


def _show_marks(base, origin):
    import torch

    k = CM.MODEL.kerfs[0] if CM.MODEL.kerfs else None
    want = set()
    if k is not None:
        share = []
        for i in range(len(MARKS)):
            t = 2 * math.pi * (i + 0.5) / len(MARKS)
            y, z = CM.R_OUT * math.sin(t), CM.R_OUT * math.cos(t)
            strips = CM._strips()
            j = min(CM.BINS - 2, max(1, int(round((y + CM.R_OUT) / (strips[1] - strips[0])))))
            top_i, bottom_i = CM._walls(strips[j])
            cut = k.floor[j] < top_i[1] - 1e-4 if z >= 0 else k.floor[j] <= (bottom_i or top_i)[0] - CM.THROUGH / 2
            if cut:
                want.add(i)
            share.append(cut)
    dev = base.device
    for i, name in enumerate(MARKS):
        if (i in want) == (i in STATE["shown"]):
            continue
        if i in want:
            c, R = mark_pose(i, k)
            q = _matrix_quat(R)
            pose = [*(c + origin), *q]
        else:
            pose = [*(MARK_HIDDEN + origin), 0.0, 0.0, 0.0, 1.0]
        base.scene.rigid_objects[name].write_root_link_pose_to_sim_index(
            root_pose=torch.tensor([pose], dtype=torch.float32, device=dev))
        STATE["shown"].symmetric_difference_update({i})


def _hook_embodiment():
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_cut_run"):
        return
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        STATE["env"] = self._env
        try:
            base = self._env.unwrapped
            _show_marks(base, _np(base.scene.env_origins)[0][:3])
        except Exception as e:  # noqa: BLE001
            if STATE.get("mark_err") is None:
                STATE["mark_err"] = repr(e)
                print(f"CUT_MARKS failed: {e!r}", flush=True)
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        CM.MODEL.clear()
        STATE["shown"] = set()
        STATE["pex"] = 0.0
        result = original_reset(self, *args, **kwargs)
        CM.MODEL.clear()  # whatever the settle steps did, every trial starts with the pipe whole
        STATE["shown"] = set()
        STATE["pex"] = 0.0
        STATE["env"] = self._env
        try:
            install(self._env)
        except Exception as e:  # noqa: BLE001
            print(f"CUT_HOOK failed: {e!r}", flush=True)
        return result

    cls._task_step, cls.reset = task_step, reset
    cls._cut_run = STATE
