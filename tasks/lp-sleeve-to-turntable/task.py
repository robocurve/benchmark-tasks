"""Load an LP, lower the stylus, and physically press START to play it."""
import json, math
from pathlib import Path
import numpy as np
from resist_isaac.clearance_recovery import ClearanceRecovery, export_assets
CONDITIONS = ('recoverable', 'playback_probe', 'button_probe')


def rotation_speed(previous, current, dt):
    # Native pose encoder, like a tachometer; avoids substep contact-velocity chatter.
    delta = current @ previous.T
    return math.atan2(delta[1, 0] - delta[0, 1], delta[0, 0] + delta[1, 1]) / dt


class LPTask(ClearanceRecovery):
    stop_when_success = True
    physics_hz = 240

    def __init__(self, folder):
        self.folder = Path(folder)
        self.cfg = json.loads((self.folder / 'pilot.json').read_text())
        self.descriptions = json.loads((self.folder / 'geometry.json').read_text())
        self.poses = self.cfg['poses']
        self.hold_s = self.cfg["playback_hold_s"]
        self._unsafe = False
        self._removed = False
        self._motor = False
        self._pressed = False
        self._qualified_start = False
        self._cue_valid = False
        self._previous_observation = None
        self._previous_motor_rotation = None
        self._start_radius = self._cue_radius = None

    def configure_native(self, env_cfg):
        # One station: bounded native capacities, with overflow errors retained in run logs.
        env_cfg.sim.physics.gpu_max_rigid_contact_count = 65536
        env_cfg.sim.physics.gpu_max_rigid_patch_count = 16384
        env_cfg.sim.physics.gpu_found_lost_pairs_capacity = 65536
        env_cfg.sim.physics.gpu_found_lost_aggregate_pairs_capacity = 65536
        env_cfg.sim.physics.gpu_total_aggregate_pairs_capacity = 262144
        env_cfg.sim.physics.solver_type = 0
        env_cfg.sim.physics.enable_external_forces_every_iteration = False
        if getattr(self, '_button_probe', False):
            from isaaclab.sensors import ContactSensorCfg
            env_cfg.scene.lp_tracking_contacts = ContactSensorCfg(
                prim_path="{ENV_REGEX_NS}/player/tonearm", update_period=0., history_length=0,
                track_contact_points=True, track_friction_forces=True,
                max_contact_data_count_per_prim=128,
                filter_prim_paths_expr=["{ENV_REGEX_NS}/record/record"])


    def initial_action(self):
        from yam_arena.constants import READY
        return np.r_[READY, 1.0, READY, 1.0]

    def reset(self, base):
        super().reset(base)
        self._removed = False
        if getattr(self, '_button_probe', False):
            from resist_isaac.state import reset_joints
            reset_joints(base, 'player', {'arm_yaw': -.385, 'arm_pitch': 0.})
        self._controller = None
        self._motor = self._pressed = self._qualified_start = self._cue_valid = False
        self._previous_observation = None
        self._previous_motor_rotation = None
        self._start_radius = self._cue_radius = None

    def scripted_action(self, action, phase, base):
        if getattr(self, '_controller', None) is None:
            import importlib.util
            spec = importlib.util.spec_from_file_location('_lp_scripted', self.folder / 'scripted.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self._controller = module.ScriptedLP(self)
        return self._controller.action(action, phase, base)

    def validate_reset(self, base):
        for key in ('record', 'sleeve'):
            p, r, v = self.states(base)[key]
            if np.linalg.norm(p - self.poses[key]) > 0.005 or np.linalg.norm(v[:3]) > 0.03:
                raise RuntimeError('Unstable LP task reset: ' + key)

    def before_step(self, base):
        """Electronic switch and capped motor torque; never set object pose/velocity."""
        import torch
        if hasattr(self, '_contacts'):
            self._contacts = dict.fromkeys(self._contacts, 0.)
        state = self.states(base)
        travel = self.cfg['button_rest_z'] - state['player/button'][0][2]
        if travel >= .003 and not self._pressed:
            self._pressed = True
            self._motor = not self._motor
            self._qualified_start = bool(self._motor and self._cue_valid)
            self._start_radius = self._cue_radius if self._qualified_start else None
        elif travel < .001:
            self._pressed = False
        platter = base.scene['player']
        rotation = state['player/platter'][1]
        velocity = 0. if self._previous_motor_rotation is None else rotation_speed(self._previous_motor_rotation, rotation, .05)
        self._previous_motor_rotation = rotation.copy()
        target = -self.cfg['play_rpm'] * 2 * math.pi / 60 if self._motor else 0.
        torque = float(np.clip(.04 * (target - velocity), -.06, .06))
        force = torch.zeros((1, 1, 3), device=base.device)
        moment = torch.zeros_like(force)
        moment[0, 0, 2] = torque
        platter.permanent_wrench_composer.reset()
        platter.permanent_wrench_composer.add_forces_and_torques(force, moment, body_ids=[platter.body_names.index('platter')], is_global=True)

    def begin_trial(self, base):
        self.validate_reset(base)
        print('LP_NATIVE_PHYSICS ' + json.dumps(dict(dt=base.sim.cfg.dt, solver=base.sim.cfg.physics.solver_type, external_forces_every_iteration=base.sim.cfg.physics.enable_external_forces_every_iteration)), flush=True)
        if getattr(self, '_button_probe', False):
            from resist_isaac.state import tensor
            player = base.scene['player']
            print('LP_NATIVE_MATERIALS ' + json.dumps({name: np.unique(
                np.array(base.scene[name].root_view.get_material_properties().numpy(), copy=True).reshape(-1, 3),
                axis=0).tolist() for name in ('player', 'record')}), flush=True)
            print('LP_NATIVE_JOINTS ' + json.dumps(dict(names=player.joint_names,
                limits=tensor(player.data.joint_pos_limits).cpu().tolist(),
                stiffness=tensor(player.data.joint_stiffness).cpu().tolist(), damping=tensor(player.data.joint_damping).cpu().tolist())), flush=True)
            import omni.usd
            from omni.physx import get_physx_simulation_interface
            from pxr import PhysxSchema
            stage = omni.usd.get_context().get_stage()
            for path in ('player/button', 'player/player', 'record/record'):
                prim = stage.GetPrimAtPath('/World/envs/env_0/' + path)
                PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)
            self._contacts = dict(button_contact_impulse_Ns=0., deck_contact_impulse_Ns=0., record_contact_impulse_Ns=0.)
            self._contact_subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.contact_report)

    def contact_report(self, headers, contacts):
        from pxr import PhysicsSchemaTools
        for header in headers:
            paths = [str(PhysicsSchemaTools.intToSdfPath(actor)) for actor in (header.actor0, header.actor1)]
            if not any('/RightArm/' in path or '/LeftArm/' in path for path in paths):
                continue
            for suffix, name in (('/player/button', 'button'), ('/player/player', 'deck'), ('/record/record', 'record')):
                if any(path.endswith(suffix) for path in paths):
                    for i in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
                        self._contacts[name + '_contact_impulse_Ns'] += float(np.linalg.norm(contacts[i].impulse))

    def score_states(self, state, tcps, observed_motion=None, observed_tip_velocity=None):
        p, r, v = state['record']
        s, sr, sv = state['sleeve']
        c = self.cfg
        rad = c['record_radius']
        half = c['record_half_thickness']
        for pos, rot, vel in state.values():
            if not np.isfinite(np.r_[pos, rot.ravel(), vel]).all():
                raise RuntimeError('Non-finite native state')
        zextent = rad * np.linalg.norm(r[2, :2]) + half * abs(r[2, 2])
        low = p[2] - zextent
        self._unsafe |= bool(low <= c['table_top'] + 0.0005 or p[2] > 1.45 or abs(p[1]) > 0.65 or (p[0] < -0.18) or (p[0] > 0.9))
        sleeve_extent = np.abs(sr) @ np.array([.1575, .1575, .0022])
        sleeve_low = s[2] - sleeve_extent[2]
        self._unsafe |= bool(sleeve_low < c['table_top'] - .01 or s[2] > 1.45
                             or abs(s[1]) > .65 or s[0] < -.18 or s[0] > .9)
        sleeve_safe = bool(s[0] - sleeve_extent[0] >= -.05 and s[0] + sleeve_extent[0] <= .75
                           and abs(s[1]) + sleeve_extent[1] <= .60
                           and sleeve_low >= c['table_top'] - .001
                           and np.linalg.norm(sv[:3]) < .02 and np.linalg.norm(sv[3:]) < .1)
        rel = sr.T @ (p - s)
        rr = sr.T @ r
        ey = rad * np.linalg.norm(rr[1, :2]) + half * abs(rr[1, 2])
        self._removed |= bool(rel[1] + ey < -0.158)
        offset = float(np.linalg.norm(p[:2] - c['spindle_xy']))
        aligned = offset < c['hole_radius'] - c['spindle_radius'] and r[2, 2] > math.cos(math.radians(1))
        seated = aligned and abs(p[2] - half - c['mat_top']) < 0.0006 and (np.linalg.norm(v[:3]) < 0.01) and (np.linalg.norm(v[3:5]) < 0.1)
        home = np.linalg.norm(s[:2] - np.asarray(c['sleeve_return'])[:2]) < 0.02 and abs(s[2] - c['sleeve_return'][2]) < 0.003 and (sr[2, 2] > math.cos(math.radians(5))) and (np.linalg.norm(sv[:3]) < 0.02)
        clear = True
        for tcp in tcps:
            lp = r.T @ (np.asarray(tcp) - p)
            sd = sr.T @ (np.asarray(tcp) - s)
            disc_distance = math.hypot(max(0, np.linalg.norm(lp[:2]) - rad), max(0, abs(lp[2]) - half))
            sleeve_distance = np.linalg.norm(np.maximum(np.abs(sd) - [0.1575, 0.1575, 0.0022], 0))
            clear &= disc_distance > 0.060 and sleeve_distance > 0.060
        ap, ar, av = state['player/tonearm']
        stylus_offset = ar @ np.asarray(c['stylus_local'])
        stylus = ap + stylus_offset
        stylus_velocity = av[:3] + np.cross(av[3:], stylus_offset)
        local = r.T @ (stylus - p)
        stylus_radius = float(np.linalg.norm(local[:2]))
        gap = float(local[2] - half)
        contact = seated and -.0001 <= gap <= .0001
        on_music = contact and c['recorded_radius'][0] <= stylus_radius <= c['recorded_radius'][1]
        cue = contact and c['lead_in_radius'][0] <= stylus_radius <= c['lead_in_radius'][1]
        stylus_speed = float(np.linalg.norm(stylus_velocity[:2]))
        rpm = float(-v[5] * 60 / (2 * math.pi))
        platter_rpm = float(-state['player/platter'][2][5] * 60 / (2 * math.pi))
        instantaneous_rpm = rpm
        if observed_motion is not None:
            rpm, platter_rpm, stylus_speed = observed_motion
        tip_velocity = stylus_velocity if observed_tip_velocity is None else observed_tip_velocity
        record_omega = v[3:].copy()
        record_omega[2] = -rpm * 2 * math.pi / 60
        relative_speed = float(np.linalg.norm((tip_velocity - v[:3] - np.cross(record_omega, stylus - p))[:2]))
        # Detect contact hazards before seating too, without treating an LP carried
        # well above the parked stylus as a penetration. Moving the LP under a
        # stationary stylus is also a drag.
        near_disc = stylus_radius < rad and abs(gap) < .002
        self._unsafe |= bool(near_disc and (gap < -.001 or stylus_radius < c['recorded_radius'][0]
                            or (not self._motor and max(stylus_speed, relative_speed) > .03)))
        self._cue_valid = bool(cue and stylus_speed < .01 and not self._unsafe and abs(rpm) < .1 * 60 / (2 * math.pi))
        if self._cue_valid:
            self._cue_radius = stylus_radius
        rotating = abs(rpm - c['play_rpm']) < 1.5 and abs(platter_rpm - c['play_rpm']) < 1.5
        # Clearance from the entire arm prevents a rear grip from counting as release.
        back = ap + ar @ np.array([.061, 0., 0.])
        segment = stylus - back
        arm_clear = True
        for tcp in tcps:
            point = np.asarray(tcp)
            fraction = np.clip(np.dot(point - back, segment) / np.dot(segment, segment), 0., 1.)
            arm_clear &= np.linalg.norm(point - (back + fraction * segment)) > .06
        button_released = c['button_rest_z'] - state['player/button'][0][2] < .001
        drift = abs(stylus_radius - self._start_radius) if self._start_radius is not None else -1.
        tracking_stable = bool(0. <= drift < .005 and stylus_speed < .001)
        playing = bool(self._motor and on_music and rotating and stylus_speed < .001 and not self._unsafe)
        success = bool(self._removed and seated and sleeve_safe and clear and arm_clear and playing and self._qualified_start and button_released and tracking_stable)
        score = 0.0 if self._unsafe else 1.0 if success else .75 if self._removed and seated and on_music else .5 if self._removed and seated else .25 if self._removed else 0.0
        return dict(success=success, score=score, unsafe_event=self._unsafe,
                    record_removed=self._removed, record_seated=bool(seated), sleeve_home=bool(home), sleeve_safe=sleeve_safe,
                    spindle_offset_m=offset, record_lowest_z_m=float(low),
                    record_height_error_m=float(p[2] - half - c['mat_top']),
                    stylus_radius_m=stylus_radius, stylus_gap_m=gap, stylus_speed_m_s=stylus_speed, stylus_relative_speed_m_s=relative_speed,
                    stylus_on_record=bool(on_music), cue_ready=self._cue_valid,
                    button_travel_m=float(c['button_rest_z'] - state['player/button'][0][2]),
                    motor_on=self._motor, start_after_cue=self._qualified_start,
                    startup_radial_drift_m=drift, tracking_stable=tracking_stable,
                    record_rpm=rpm, platter_rpm=platter_rpm, instantaneous_record_rpm=instantaneous_rpm, music_playing=playing)

    def evaluate(self, base):
        from yam_arena.runtime import tcp_world
        state = self.states(base)
        ap, ar, av = state['player/tonearm']
        stylus = ap + ar @ np.asarray(self.cfg['stylus_local'])
        record_r, platter_r = state['record'][1], state['player/platter'][1]
        current = (record_r.copy(), platter_r.copy(), stylus.copy())
        previous = self._previous_observation
        observed = None
        observed_tip_velocity = None
        if previous is not None:
            observed_tip_velocity = (stylus - previous[2]) / .05
            observed = (-rotation_speed(previous[0], record_r, .05) * 60 / (2 * math.pi),
                        -rotation_speed(previous[1], platter_r, .05) * 60 / (2 * math.pi),
                        float(np.linalg.norm((stylus - previous[2])[:2]) / .05))
        self._previous_observation = current
        metrics = self.score_states(state, [tcp_world(base.scene[a + '_robot']) for a in ['left', 'right']], observed, observed_tip_velocity)
        from resist_isaac.state import tensor
        player = base.scene['player']
        joints = tensor(player.data.joint_pos)[0].cpu().numpy()
        for name in ('arm_yaw', 'arm_pitch', 'start_button', 'platter_axle'):
            metrics['joint_' + name] = float(joints[player.joint_names.index(name)])
        metrics.update(getattr(self, "_contacts", {}))
        if getattr(self, '_button_probe', False):
            view = base.scene.sensors['lp_tracking_contacts'].contact_view
            normal = np.array(view.get_contact_force_matrix(dt=base.physics_dt).numpy(), copy=True).reshape(-1, 3).sum(axis=0)
            force, points, count, start = [np.array(v.numpy(), copy=True) for v in view.get_friction_data(dt=base.physics_dt)]
            n, a = int(count[0, 0]), int(start[0, 0])
            if n < 0 or a < 0 or a+n > len(force) or n >= 128:
                raise RuntimeError('Tracking contact data exceeds native allocation')
            friction = force[a:a+n].sum(axis=0)
            pivot = state['player/gimbal'][0]
            torque = np.cross(points[a:a+n] - pivot, force[a:a+n]).sum(axis=0)
            metrics.update(tracking_normal_z_N=float(normal[2]), tracking_friction_x_N=float(friction[0]),
                           tracking_friction_y_N=float(friction[1]), tracking_yaw_torque_Nm=float(torque[2]))

        return metrics

def make_task(condition):
    if condition not in CONDITIONS:
        raise ValueError(condition)
    task = LPTask(Path(__file__).parent)
    if condition in ('playback_probe', 'button_probe'):
        # Isolated mechanism test, never evidence of extraction/loading.
        task.poses['record'] = [*task.cfg['spindle_xy'], task.cfg['mat_top'] + task.cfg['record_half_thickness']]
    task._button_probe = condition == "button_probe"
    return task

def create_assets(directory, condition='recoverable'):
    """Export editable parts plus one self-contained, positioned USD prop assembly."""
    from pxr import Gf, Usd, UsdGeom
    folder = Path(__file__).parent
    paths = export_assets(folder, directory)
    config = make_task(condition).cfg
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, '/World')
    stage.SetDefaultPrim(root.GetPrim())
    for name, path in paths.items():
        part = UsdGeom.Xform.Define(stage, '/World/' + name)
        part.GetPrim().GetReferences().AddReference(str(Path(path).resolve()))
        part.AddTranslateOp().Set(Gf.Vec3d(*config['poses'][name]))
    scene = Path(directory) / 'scene.usdc'
    stage.Flatten().Export(str(scene))
    paths['scene'] = scene
    return paths
