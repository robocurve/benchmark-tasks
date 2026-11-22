"""Strut spring-nut task: Arena props and TaskBase (props: strutnut_sim/usd/props.py, layout: strutnut_sim/spec.py).

Mechanism (geometry and contact, plus one screw rule):
- The channel's lips are fixed colliders as the P1000 section shows them; the nut is a free body whose outline lets
  it through the slot only edge-on (19 mm wide in the 22.2 mm slot; 34.5 mm long, more than the slot) and turn only
  clockwise inside the channel (two leading corners rounded, two square ones stop it just past a quarter turn); its
  top has two grooves the lip returns drop into. Its spring is a sprung prismatic foot (SPRING_K) standing on the
  channel's back: it pushes the nut up against the lips once it is turned under them, and back out of the slot if
  it is not.
- The bolt starts free, not in the nut (the robot starts it). Thread rule (``_screw``): while its pilot end is over
  the nut's hole it sits in the hole's mouth and cannot go deeper (the first thread has not caught); turned clockwise
  CATCH_DEG there it catches: square to the nut within ALIGN_DEG the thread starts, otherwise it cross-threads (a
  hazard; it jams). Started, the bolt is held on the nut's axis and square to it and coupled every control step like
  a screw: the thread's stiff drive targets the catch height - PITCH x turns (clockwise seen from the head runs the
  bolt into the nut). Blocked, the drive's pull is the bolt's tension F = CLAMP_K (p - target). The turn joint
  carries thread friction as a Coulomb drive (a stiff spring whose anchor slips once it carries T = RUN_NM +
  NUT_K d F, the bolt formula): the free nut below the lips turns with the bolt; once its square corners stop against
  the walls (or its grooves hold it on the lips) the bolt turns in it. Backed out past its start it is free again.
- The fitting (P1064 square washer) is a free body with a clearance hole round the shank, held by geometry only.

After-release check (the success check), once every finger pad has stayed RELEASE_CLEAR from the fitting and the
bolt for TEST_WAIT_S without a break (the count restarts on any approach): the 75 conditions, the fitting seated and
square and the head down on it must hold; then the fitting is pulled along the channel with TEST_PULL_N (ramped over
TEST_RAMP_S, then held to TEST_S); it passes if it moved less than TEST_MAX_MOVE. ``force_test(env)`` starts the
same check at once (control runs).
"""

from __future__ import annotations

import math
import os
from dataclasses import MISSING
from pathlib import Path

import torch

import isaaclab.utils.math as math_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import ManagerBasedEnv
from isaaclab.managers import EventTermCfg
from isaaclab.utils.configclass import configclass

from isaaclab_arena.assets.object import Object
from isaaclab_arena.assets.object_type import ObjectType
from isaaclab_arena.embodiments.common.arm_mode import ArmMode
from isaaclab_arena.metrics.metric_base import MetricBase
from isaaclab_arena.metrics.success_rate import SuccessRateMetric
from isaaclab_arena.progress_tracking.completion_criteria import CompletionCriteria
from isaaclab_arena.tasks.task_base import TaskBase
from isaaclab_arena.tasks.task_termination_cfg import TaskTerminationCfg
from isaaclab_arena.utils.pose import Pose

from strutnut_sim import spec as S
from strutnut_sim.kit import Fingertips, reset_robot_home
from strutnut_sim.scorer import MILESTONES, Scorer, Step

_ASSETS = Path(__file__).resolve().parent / "assets"
IDENTITY = (0.0, 0.0, 0.0, 1.0)
HAZARD_TEST = os.environ.get("SNC_HAZARD_TEST", "")
DEBUG_EVERY = int(os.environ.get("SNC_DEBUG", "0") or 0)


class Stand(Object):
    def __init__(self):
        super().__init__(name="stand", usd_path=str(_ASSETS / "stand.usda"), object_type=ObjectType.BASE,
                         initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=IDENTITY))


CHAIN = ("bx", "by", "thread", "tilt_x", "tilt_y", "turn")
"""The bolt's free chain to the nut (props.py): drives off until the thread is started."""


def bolt_start_q() -> dict:
    k = S.kit_start()
    n, b = k["nut"], k["bolt"]
    return {"spring": 0.0, "bx": b[0] - n[0], "by": b[1] - n[1], "thread": b[2] - n[2], "tilt_x": 0.0, "tilt_y": 0.0,
            "turn": 0.0}


class Kit(Object):
    def __init__(self):
        k = S.kit_start()
        super().__init__(name="kit", usd_path=str(_ASSETS / "kit.usda"), object_type=ObjectType.ARTICULATION,
                         initial_pose=Pose(position_xyz=k["nut"], rotation_xyzw=IDENTITY))

    def _generate_articulation_cfg(self) -> ArticulationCfg:
        cfg = super()._generate_articulation_cfg()
        cfg.actuators = {
            "spring": ImplicitActuatorCfg(joint_names_expr=["spring"], stiffness=S.SPRING_K, damping=S.SPRING_D),
            "lock": ImplicitActuatorCfg(joint_names_expr=["bx", "by"], stiffness=S.LOCK_K, damping=S.LOCK_D,
                                        effort_limit_sim=S.LOCK_MAX_N),
            "tilt": ImplicitActuatorCfg(joint_names_expr=["tilt_x", "tilt_y"], stiffness=S.TILT_K, damping=S.TILT_D,
                                        effort_limit_sim=S.TILT_MAX_NM),
            "thread": ImplicitActuatorCfg(joint_names_expr=["thread"], stiffness=S.CLAMP_K, damping=S.CLAMP_D,
                                          effort_limit_sim=S.CLAMP_MAX_N),
            "turn": ImplicitActuatorCfg(joint_names_expr=["turn"], stiffness=S.TURN_K, damping=S.TURN_D,
                                        effort_limit_sim=S.RUN_NM, armature=S.TURN_ARMATURE),
        }
        cfg.init_state.joint_pos = bolt_start_q()
        return cfg


class Fitting(Object):
    def __init__(self):
        k = S.kit_start()
        super().__init__(name="fitting", usd_path=str(_ASSETS / "fitting.usda"), object_type=ObjectType.RIGID,
                         initial_pose=Pose(position_xyz=k["fit"], rotation_xyzw=IDENTITY))


def all_props() -> list:
    return [Stand(), Kit(), Fitting()]


def _axis(quat, v):
    vv = torch.tensor(v, device=quat.device, dtype=quat.dtype).expand(quat.shape[0], 3)
    return math_utils.quat_apply(quat, vv)


class _TaskState:
    def __init__(self, env: ManagerBasedEnv, task: "StrutNutTask"):
        n, dev = env.num_envs, env.device
        self.robot = env.scene["robot"]
        self.kit = env.scene["kit"]
        self.fit = env.scene["fitting"]
        self.tips = Fingertips(self.robot)
        self.j_spring = self.kit.find_joints("spring")[0][0]
        self.j_thread = self.kit.find_joints("thread")[0][0]
        self.j_turn = self.kit.find_joints("turn")[0][0]
        self.j_chain = [self.kit.find_joints(nm)[0][0] for nm in CHAIN]
        self.j_lat = self.j_chain[:2]
        self.j_tilt = self.j_chain[3:5]
        self.b_nut = self.kit.find_bodies("nut")[0][0]
        self.b_bolt = self.kit.find_bodies("bolt")[0][0]
        self.b_foot = self.kit.find_bodies("foot")[0][0]
        self.scorer = Scorer(n, dev)
        self.last_reports: dict[int, dict] = {}
        zf = lambda: torch.zeros(n, device=dev)  # noqa: E731
        zb = lambda: torch.zeros(n, dtype=torch.bool, device=dev)  # noqa: E731
        self.turn_prev, self.turn_acc, self.turn_hold, self.force, self.peak_force = zf(), zf(), zf(), zf(), zf()
        self.force_test, self.test_passed, self.ready_at_start = zb(), zb(), zb()
        # Thread start: 0 free (not started), 1 started (thread caught square), 2 cross-threaded (jammed).
        self.tphase = torch.zeros(n, dtype=torch.long, device=dev)
        self.catch_acc, self.z_catch, self.stop_z, self.tilt, self.tip_lat = zf(), zf(), zf(), zf(), zf()
        self.stop_on, self.ever_started = zb(), zb()
        self.jam_q = torch.zeros(n, 6, device=dev)
        self.dbg_z, self.dbg_stop = zf(), zf()
        self.catch_tilt = zf()
        self.clear_steps = torch.zeros(n, dtype=torch.long, device=dev)
        self.phase = torch.zeros(n, dtype=torch.long, device=dev)
        self.phase_steps = torch.zeros(n, dtype=torch.long, device=dev)
        self.test_start_y, self.test_move = zf(), zf()
        self.step_dt = env.step_dt
        self.wait_steps = max(1, round(S.TEST_WAIT_S / env.step_dt))
        self.test_len = max(1, round(S.TEST_S / env.step_dt))
        self.ramp_len = max(1, round(S.TEST_RAMP_S / env.step_dt))
        self.m: dict[str, torch.Tensor] = {}
        self.hazard_done = False


class StrutNutTask(TaskBase):
    def __init__(self, episode_length_s: float = S.EPISODE_SECONDS):
        super().__init__(episode_length_s=episode_length_s, task_description=S.instruction())
        self._k_nom = torch.tensor([[S.LOCK_K, S.LOCK_K, S.CLAMP_K, S.TILT_K, S.TILT_K, S.TURN_K]])
        self._d_nom = torch.tensor([[S.LOCK_D, S.LOCK_D, S.CLAMP_D, S.TILT_D, S.TILT_D, S.TURN_D]])

    def state(self, env: ManagerBasedEnv) -> _TaskState:
        if getattr(env, "_task_state", None) is None:
            env._task_state = _TaskState(env, self)
        return env._task_state

    def force_test(self, env: ManagerBasedEnv) -> None:
        self.state(env).force_test[:] = True

    def episode_report(self, env: ManagerBasedEnv, env_id: int = 0, ended: bool = False) -> dict:
        st = self.state(env)
        if ended and env_id in st.last_reports:
            return st.last_reports.pop(env_id)
        return self._report(st, env_id)

    def _report(self, st: _TaskState, env_id: int) -> dict:
        r = st.scorer.report(env_id)
        for k, v in st.m.items():
            val = float(v[env_id])
            r[k] = round(val * 1000, 2) if k.endswith("_mm") else round(val, 3)
        r.update(test_phase=int(st.phase[env_id]), ready_at_test_start=bool(st.ready_at_start[env_id]),
                 test_move_mm=round(float(st.test_move[env_id]) * 1000, 3), test_passed=bool(st.test_passed[env_id]),
                 bolt_tension_n=round(float(st.force[env_id]), 1), peak_tension_n=round(float(st.peak_force[env_id]), 1))
        return r

    def get_scene_cfg(self):
        return None

    def get_events_cfg(self):
        return EventsCfg(self)

    def get_termination_cfg(self) -> TaskTerminationCfg:
        sequence = [(self._milestone_predicate(m), 1.0 / len(MILESTONES)) for m in MILESTONES]
        return TaskTerminationCfg(timeout_s=self.episode_length_s,
                                  success=[CompletionCriteria(name="milestones", predicate_sequence=sequence)])

    def get_mimic_env_cfg(self, arm_mode: ArmMode):
        raise NotImplementedError

    def get_metrics(self) -> list[MetricBase]:
        return [SuccessRateMetric()]

    def _milestone_predicate(self, milestone: int):
        def predicate(env: ManagerBasedEnv) -> torch.Tensor:
            sc = self.state(env).scorer
            return (sc.milestone >= milestone) & ~sc.hazard

        predicate.__name__ = f"milestone_{milestone}"
        return predicate

    # Events ------------------------------------------------------------------------------------------------------
    def reset(self, env: ManagerBasedEnv, env_ids: torch.Tensor) -> None:
        st = self.state(env)
        reset_robot_home(env, env_ids)
        for env_id in env_ids.tolist():
            if int(st.scorer.step[env_id]) > 0:
                st.last_reports[env_id] = self._report(st, env_id)
        dev, n = env.device, len(env_ids)
        origins = env.scene.env_origins
        st.scorer.reset(env_ids)
        for buf in (st.force_test, st.test_passed, st.ready_at_start):
            buf[env_ids] = False
        for buf in (st.clear_steps, st.phase, st.phase_steps):
            buf[env_ids] = 0
        for buf in (st.turn_prev, st.turn_acc, st.turn_hold, st.force, st.peak_force, st.test_start_y, st.test_move):
            buf[env_ids] = 0.0
        st.hazard_done = False
        st.fit.permanent_wrench_composer.reset(env_ids=env_ids)
        k = S.kit_start()
        pose = torch.tensor([[*k["nut"], *IDENTITY]], device=dev).repeat(n, 1)
        pose[:, :3] += origins[env_ids]
        st.kit.write_root_pose_to_sim(pose, env_ids=env_ids)
        st.kit.write_root_velocity_to_sim(torch.zeros(n, 6, device=dev), env_ids=env_ids)
        q = torch.zeros(n, st.kit.num_joints, device=dev)
        for name, v in bolt_start_q().items():
            q[:, st.kit.find_joints(name)[0][0]] = v
        st.kit.write_joint_state_to_sim(q, torch.zeros_like(q), env_ids=env_ids)
        st.kit.set_joint_position_target(q.clone(), env_ids=env_ids)
        zc = torch.zeros(n, len(st.j_chain), device=dev)
        st.kit.write_joint_effort_limit_to_sim_index(limits=zc, joint_ids=st.j_chain, env_ids=env_ids)
        st.kit.write_joint_stiffness_to_sim_index(stiffness=zc, joint_ids=st.j_chain, env_ids=env_ids)
        st.kit.write_joint_damping_to_sim_index(damping=zc, joint_ids=st.j_chain, env_ids=env_ids)
        lim = torch.tensor([[-0.6, 0.6]], device=dev).repeat(n, 1).unsqueeze(1)
        st.kit.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=[st.j_thread], env_ids=env_ids,
                                                       warn_limit_violation=False)
        st.tphase[env_ids] = 0
        for buf in (st.catch_acc, st.z_catch, st.stop_z, st.tilt, st.tip_lat, st.catch_tilt):
            buf[env_ids] = 0.0
        st.stop_on[env_ids] = False
        st.ever_started[env_ids] = False
        st.turn_prev[env_ids] = 0.0
        fpose = torch.tensor([[*k["fit"], *IDENTITY]], device=dev).repeat(n, 1)
        fpose[:, :3] += origins[env_ids]
        st.fit.write_root_pose_to_sim(fpose, env_ids=env_ids)
        st.fit.write_root_velocity_to_sim(torch.zeros(n, 6, device=dev), env_ids=env_ids)

    def _relative(self, st: _TaskState):
        """The bolt in the nut's frame: (origin = head underside centre, its axis, its tip)."""
        nut_q = st.kit.data.body_quat_w.torch[:, st.b_nut]
        d = st.kit.data.body_pos_w.torch[:, st.b_bolt] - st.kit.data.body_pos_w.torch[:, st.b_nut]
        p = math_utils.quat_apply_inverse(nut_q, d)
        ax = math_utils.quat_apply_inverse(nut_q, _axis(st.kit.data.body_quat_w.torch[:, st.b_bolt], (0.0, 0.0, 1.0)))
        return p, ax, p - S.BOLT_L * ax

    def _screw(self, st: _TaskState) -> None:
        """The thread rule. Free (not started): every drive of the bolt's chain is off (a free body); while its pilot end
        is over the nut's hole it cannot go deeper than ENTRY_DEPTH (the first thread has not caught: the thread's
        major diameter does not pass the nut's minor diameter). Turned clockwise CATCH_DEG there, the thread catches:
        square to the nut within ALIGN_DEG it starts (the drives hold it on the nut's axis and square to it, and the
        screw coupling below runs it in, PITCH per turn); more tilted it cross-threads (a hazard; the bolt jams).
        Backed out past the start it is free again."""
        n, dev = st.tphase.shape[0], st.tphase.device
        jp = st.kit.data.joint_pos.torch
        q_raw = jp[:, st.j_turn]
        d = torch.remainder(q_raw - st.turn_prev + math.pi, 2 * math.pi) - math.pi
        st.turn_prev = q_raw.clone()
        st.turn_acc = st.turn_acc + d
        p_rel, ax, tip = self._relative(st)
        tilt = torch.rad2deg(torch.acos(ax[:, 2].clamp(-1.0, 1.0)))
        lat = tip[:, :2].norm(dim=-1)
        st.tilt, st.tip_lat = tilt, lat
        z = jp[:, st.j_thread]
        free, started, crossed = st.tphase == 0, st.tphase == 1, st.tphase == 2

        # Free: the entry stop and the catch.
        stop = S.BOLT_L * ax[:, 2] - S.ENTRY_DEPTH
        over = (lat < torch.where(st.stop_on, S.OVER_HOLE + S.MOUTH_SLACK, S.OVER_HOLE)) & (tilt < 20.0) & (ax[:, 2] > 0)
        st.stop_on = free & over & (st.stop_on | (z >= stop - 0.0003))
        st.stop_z = stop
        at_entry = st.stop_on & (z - stop < 0.0003)
        st.catch_acc = torch.where(at_entry, (st.catch_acc - d).clamp(min=0.0), torch.zeros_like(st.catch_acc))
        caught = free & at_entry & (st.catch_acc >= math.radians(S.CATCH_DEG))
        square = tilt <= S.ALIGN_DEG
        go = caught & square
        jam = caught & ~square
        if go.any() or jam.any():
            st.z_catch = torch.where(caught, z, st.z_catch)
            st.turn_acc = torch.where(caught, torch.zeros_like(st.turn_acc), st.turn_acc)
            st.turn_hold = torch.where(caught, torch.zeros_like(st.turn_hold), st.turn_hold)
            st.catch_tilt = torch.where(caught, tilt, st.catch_tilt)
            st.tphase = torch.where(go, torch.ones_like(st.tphase), st.tphase)
            st.tphase = torch.where(jam, torch.full_like(st.tphase, 2), st.tphase)
            st.ever_started |= go
            st.jam_q = torch.where(jam[:, None], jp[:, st.j_chain], st.jam_q)
            for i in go.nonzero().flatten().tolist():
                print(f"[strut-nut] THREAD STARTED (tilt {float(tilt[i]):.2f} deg, tip {float(lat[i]) * 1000:.2f} mm off"
                      f" axis)", flush=True)
            for i in jam.nonzero().flatten().tolist():
                print(f"[strut-nut] CROSS-THREADED (tilt {float(tilt[i]):.2f} deg > {S.ALIGN_DEG})", flush=True)
            started, crossed = st.tphase == 1, st.tphase == 2

        # Started: the screw coupling (as before the thread was a rule: START + PITCH x turns).
        lo, hi = S.BOLT_L - S.NUT_T - 0.004, S.BOLT_L
        target = (st.z_catch + S.PITCH * st.turn_acc / (2 * math.pi)).clamp(lo, hi)
        backed_out = started & (target > st.z_catch + 0.0003)
        if backed_out.any():
            st.tphase = torch.where(backed_out, torch.zeros_like(st.tphase), st.tphase)
            started = st.tphase == 1
        target = torch.where(crossed, st.jam_q[:, 2], target)
        force = torch.where(started | crossed, (S.CLAMP_K * (z - target)).clamp(min=0.0, max=S.CLAMP_MAX_N),
                            torch.zeros_like(z))
        st.force = force
        st.peak_force = torch.maximum(st.peak_force, force)
        limit = torch.where(crossed, torch.full_like(force, S.JAM_NM), S.RUN_NM + S.NUT_K * S.BOLT_D * force)
        st.turn_hold = torch.minimum(torch.maximum(st.turn_hold, st.turn_acc - limit / S.TURN_K),
                                     st.turn_acc + limit / S.TURN_K)
        held = started | crossed
        vz = st.kit.data.joint_vel.torch[:, st.j_thread]
        window = (vz.abs() * st.step_dt * 1.5).clamp(min=0.0003)
        stopping = st.stop_on & ~held & (z < stop + window)
        zero = torch.zeros_like(force)
        eff = torch.stack([torch.where(held, zero + S.LOCK_MAX_N, zero), torch.where(held, zero + S.LOCK_MAX_N, zero),
                           torch.where(held, zero + S.CLAMP_MAX_N, torch.where(stopping, zero + S.STOP_MAX_N, zero)),
                           torch.where(held, zero + S.TILT_MAX_NM, zero), torch.where(held, zero + S.TILT_MAX_NM, zero),
                           torch.where(held, limit, zero)], dim=1)
        st.kit.write_joint_effort_limit_to_sim_index(limits=eff, joint_ids=st.j_chain)
        # Drives fully off while free (no stiffness or damping), so the free bolt drags nothing; at the entry (within
        # 0.3 mm of it, or of the distance it would fall in 1.5 control steps) only the z slide's drive is on, holding
        # the bolt's end there.
        on = held.to(torch.float32)[:, None]
        k = on * self._k_nom.to(dev)
        dmp = on * self._d_nom.to(dev)
        k[:, 2] = torch.where(stopping, torch.full_like(z, S.STOP_K), k[:, 2])
        dmp[:, 2] = torch.where(stopping, torch.full_like(z, S.STOP_D), dmp[:, 2])
        st.kit.write_joint_stiffness_to_sim_index(stiffness=k, joint_ids=st.j_chain)
        st.kit.write_joint_damping_to_sim_index(damping=dmp, joint_ids=st.j_chain)
        cur = jp[:, st.j_chain]
        tgt = cur.clone()
        tgt[:, 0:2] = torch.where(crossed[:, None], st.jam_q[:, 0:2], torch.where(started[:, None], 0.0, cur[:, 0:2]))
        tgt[:, 3:5] = torch.where(crossed[:, None], st.jam_q[:, 3:5], torch.where(started[:, None], 0.0, cur[:, 3:5]))
        tgt[:, 2] = torch.where(held, target, torch.where(stopping, stop, z))
        tgt[:, 5] = torch.where(held, q_raw + st.turn_hold - st.turn_acc, q_raw)
        st.kit.set_joint_position_target(tgt, joint_ids=st.j_chain)
        st.dbg_z, st.dbg_stop = z, stop

    def _hazard_hooks(self, env, st: _TaskState) -> None:
        """Test harness (SNC_HAZARD_TEST, empty in scored runs): move a part as a hand would, once."""
        if not HAZARD_TEST or st.hazard_done:
            return
        case, _, when = HAZARD_TEST.partition("_")
        step = int(st.scorer.step[0])
        m = int(st.scorer.milestone[0])
        due = (when == "start" and step == round(1.0 / env.step_dt)) or (when.isdigit() and m >= int(when))
        if not due:
            return
        st.hazard_done = True
        origins = env.scene.env_origins
        ids = torch.arange(env.num_envs, device=env.device)
        if case == "drop":
            # The fitting falls to the table in front of the blocks. At the start it stays there (the bolt drops into
            # its block's hole); later it is put back where it was the next step (the demo carries on).
            pose = torch.cat([st.fit.data.root_pos_w.torch, st.fit.data.root_quat_w.torch], dim=-1).clone()
            low = pose.clone()
            low[:, 0] = origins[:, 0] + 0.12
            low[:, 1] = origins[:, 1] - 0.17
            low[:, 2] = origins[:, 2] + 0.0005
            low[:, 3:] = torch.tensor([0.0, 0.0, 0.0, 1.0], device=env.device)
            st.fit.write_root_pose_to_sim(low, env_ids=ids)
            st.fit.write_root_velocity_to_sim(torch.zeros(len(ids), 6, device=env.device), env_ids=ids)
            if when != "start":
                st._restore_fit = pose
        elif case == "nutoff":
            # The bolt is backed out of the nut past its first thread (as if turned anticlockwise until it comes off).
            st.turn_acc[:] = 2 * math.pi * 0.5
        print(f"[strut-nut] HAZARD TEST {HAZARD_TEST} applied at step {step} (milestone {m})", flush=True)

    def step(self, env: ManagerBasedEnv, env_ids: torch.Tensor) -> None:
        st = self.state(env)
        origins = env.scene.env_origins
        n, dev = env.num_envs, env.device
        restore = getattr(st, "_restore_fit", None)  # a test drop is measured on this step, then undone
        self._screw(st)
        idle = st.phase == 0

        body_pos = st.kit.data.body_pos_w.torch - origins[:, None]
        body_quat = st.kit.data.body_quat_w.torch
        nut_p, nut_q = body_pos[:, st.b_nut], body_quat[:, st.b_nut]
        bolt_p, bolt_q = body_pos[:, st.b_bolt], body_quat[:, st.b_bolt]
        fit_p = st.fit.data.root_pos_w.torch - origins
        fit_q = st.fit.data.root_quat_w.torch
        jp = st.kit.data.joint_pos.torch
        started_now = st.tphase == 1
        engaged = torch.where(started_now, (S.BOLT_L - jp[:, st.j_thread]).clamp(min=0.0), torch.zeros_like(fit_p[:, 0]))
        turns = engaged / S.PITCH
        started_ok = turns >= S.STARTED_TURNS

        # Nut seated: long axis across the channel (along x), top at the seated height, centred, level.
        long_ax = _axis(nut_q, (0.0, 1.0, 0.0))
        up_n = _axis(nut_q, (0.0, 0.0, 1.0))
        across = torch.rad2deg(torch.acos(long_ax[:, 0].abs().clamp(max=1.0)))
        nut_tilt = torch.rad2deg(torch.acos(up_n[:, 2].clamp(-1.0, 1.0)))
        seat_err = nut_p[:, 2] - S.SEATED_TOP
        centre_err = (nut_p[:, 0] - S.CH_X).abs()
        seated = (across <= S.NUT_ACROSS_DEG) & (seat_err.abs() <= S.SEAT_TOL) & (centre_err <= S.CENTRE_TOL) & \
            (nut_tilt <= 5.0)
        mark_err = bolt_p[:, 1] - S.MARK_Y
        at_mark = mark_err.abs() <= S.MARK_TOL
        engaged_ok = turns >= S.ENGAGED_TURNS

        # Fitting: flat on the channel, square to it, the head down on it.
        up_f = _axis(fit_q, (0.0, 0.0, 1.0))
        fx = _axis(fit_q, (1.0, 0.0, 0.0))
        fit_tilt = torch.rad2deg(torch.acos(up_f[:, 2].clamp(-1.0, 1.0)))
        yaw = torch.rad2deg(torch.atan2(fx[:, 1], fx[:, 0]))
        square_err = (torch.remainder(yaw + 45.0, 90.0) - 45.0).abs()
        fit_gap = fit_p[:, 2] - S.CH_TOP
        head_gap = bolt_p[:, 2] - (fit_p[:, 2] + S.FIT_T)
        fit_ok = (fit_gap.abs() <= S.FIT_FLAT_TOL) & (fit_tilt <= S.FIT_TILT_DEG) & (square_err <= S.FIT_SQUARE_DEG) & \
            (head_gap <= S.HEAD_GAP_TOL)
        ready = started_ok & seated & at_mark & engaged_ok & fit_ok

        # Hazards.
        def fell(p):
            off_channel = ((p[:, 0] - S.CH_X).abs() > S.CH_W / 2 + 0.004) | (p[:, 1] < S.CH_Y[0]) | (p[:, 1] > S.CH_Y[1])
            return off_channel & (p[:, 2] < S.DROP_Z)

        dropped = fell(fit_p) | fell(nut_p - torch.tensor([0.0, 0.0, S.NUT_T], device=dev)) | fell(bolt_p)
        in_channel = ((nut_p[:, 0] - S.CH_X).abs() < S.CH_W / 2) & (nut_p[:, 1] > S.CH_Y[0]) & \
            (nut_p[:, 1] < S.CH_Y[1]) & (nut_p[:, 2] < S.CH_TOP)
        nut_off = in_channel & (turns < S.NUT_OFF_TURNS)
        crossed = st.tphase == 2

        # Finger pads: distance to the fitting (its plate) and to the bolt's head.
        pads = st.tips.points(origins)
        d_head = (pads - (bolt_p + _axis(bolt_q, (0.0, 0.0, S.HEAD_H / 2)))[:, None]).norm(dim=-1).min(dim=1).values
        rel = math_utils.quat_apply_inverse(fit_q[:, None].expand(n, 4, 4).reshape(-1, 4),
                                            (pads - fit_p[:, None]).reshape(-1, 3)).reshape(n, 4, 3)
        lo = torch.tensor([-S.FIT_W / 2, -S.FIT_W / 2, 0.0], device=dev)
        hi = torch.tensor([S.FIT_W / 2, S.FIT_W / 2, S.FIT_T], device=dev)
        d_fit = torch.maximum(lo - rel, rel - hi).clamp(min=0.0).norm(dim=-1).min(dim=1).values
        clear = (d_head > S.RELEASE_CLEAR) & (d_fit > S.RELEASE_CLEAR)

        # After-release check.
        go = idle & ((st.scorer.milestone >= 80) | st.force_test) & (clear | st.force_test)
        st.clear_steps = torch.where(go, st.clear_steps + 1, torch.zeros_like(st.clear_steps))
        start = go & ((st.clear_steps >= st.wait_steps) | st.force_test)
        if start.any():
            st.ready_at_start = torch.where(start, ready, st.ready_at_start)
            st.phase = torch.where(start, torch.ones_like(st.phase), st.phase)
            st.phase_steps = torch.where(start, torch.zeros_like(st.phase_steps), st.phase_steps)
            st.test_start_y = torch.where(start, fit_p[:, 1], st.test_start_y)
        testing = st.phase == 1
        if testing.any():
            ids = testing.nonzero().flatten()
            ramp = ((st.phase_steps[ids].to(torch.float32) + 1) / st.ramp_len).clamp(max=1.0)
            forces = torch.zeros(len(ids), 1, 3, device=dev)
            forces[:, 0, 1] = S.TEST_PULL_N * ramp
            centre = fit_p[ids] + _axis(fit_q[ids], (0.0, 0.0, S.FIT_T / 2)) + origins[ids]
            st.fit.permanent_wrench_composer.set_forces_and_torques_index(
                forces=forces, torques=torch.zeros_like(forces), positions=centre[:, None], body_ids=[0], env_ids=ids,
                is_global=True)
            st.test_move = torch.where(testing, torch.maximum(st.test_move, (fit_p[:, 1] - st.test_start_y).abs()),
                                       st.test_move)
            st.phase_steps = torch.where(testing, st.phase_steps + 1, st.phase_steps)
            end = testing & ((st.phase_steps >= st.test_len) | (st.test_move > 0.02))
            if end.any():
                eids = end.nonzero().flatten()
                st.fit.permanent_wrench_composer.reset(env_ids=eids)
                st.test_passed[eids] = (st.test_move[eids] < S.TEST_MAX_MOVE) & st.ready_at_start[eids]
                st.phase[eids] = 2

        st.m = {"nut_across_err_deg": across, "nut_seat_err_mm": seat_err, "nut_centre_err_mm": centre_err,
                "nut_tilt_deg": nut_tilt, "bolt_from_mark_mm": mark_err, "engaged_turns": turns,
                "fit_gap_mm": fit_gap, "fit_tilt_deg": fit_tilt, "fit_square_err_deg": square_err,
                "head_gap_mm": head_gap, "pad_to_head_mm": d_head, "pad_to_fitting_mm": d_fit,
                "fit_x_mm": fit_p[:, 0], "fit_y_mm": fit_p[:, 1], "nut_x_mm": nut_p[:, 0], "nut_y_mm": nut_p[:, 1], "nut_z_mm": nut_p[:, 2],
                "spring_mm": jp[:, st.j_spring], "turn_acc_rev": st.turn_acc / (2 * math.pi),
                "thread_state": st.tphase.to(torch.float32), "bolt_tilt_deg": st.tilt, "tip_off_axis_mm": st.tip_lat,
                "catch_deg": torch.rad2deg(st.catch_acc), "tilt_at_catch_deg": st.catch_tilt,
                "thread_z_mm": st.dbg_z, "entry_stop_mm": st.dbg_stop, "stop_on": st.stop_on.to(torch.float32)}
        st.scorer.update(Step(started_ok & idle, seated & idle, at_mark & idle, engaged_ok & idle,
                              st.test_passed.clone(), dropped & idle, nut_off & idle, crossed & idle))
        if restore is not None:
            st.fit.write_root_pose_to_sim(restore, env_ids=torch.arange(n, device=dev))
            st._restore_fit = None
        self._hazard_hooks(env, st)
        k = int(st.scorer.step[0])
        if DEBUG_EVERY and k % DEBUG_EVERY == 0:
            print(f"[strut-nut] k={k} {self._report(st, 0)}", flush=True)


@configclass
class EventsCfg:
    reset_task: EventTermCfg = MISSING
    step_task: EventTermCfg = MISSING

    def __init__(self, task: StrutNutTask):
        self.reset_task = EventTermCfg(func=task.reset, mode="reset")
        self.step_task = EventTermCfg(func=task.step, mode="interval", interval_range_s=(0.0, 0.0))
