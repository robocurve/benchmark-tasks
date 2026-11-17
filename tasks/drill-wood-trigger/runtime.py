"""Task-local physics extension. No installed adapter files are modified."""

import atexit
import json
from pathlib import Path

import numpy as np

from yam_arena import inspect_embodiment as adapter
from yam_arena.inspect_embodiment import YamArenaEmbodiment
from yam_arena.constants import FINGER_JOINTS, FINGER_OPEN, PAD_TIP_PAST_TCP

ACTIVE = None


def array(value):
    if hasattr(value, "torch"):
        value = value.torch
    return value.detach().cpu().numpy()


def spawn_drill(prim_path, cfg, *args, **kwargs):
    from isaaclab.sim.spawners.from_files import spawn_from_usd
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema

    prim = spawn_from_usd.__wrapped__(prim_path, cfg, *args, **kwargs)
    stage = prim.GetStage()
    bit = UsdGeom.Cylinder.Define(stage, prim_path + "/BluntBit")
    bit.CreateAxisAttr("X")
    bit.CreateRadiusAttr(0.003)
    bit.CreateHeightAttr(0.035)
    bit.AddTranslateOp().Set(Gf.Vec3d(-0.1095, -0.0695, 0.0))
    bit.CreateDisplayColorAttr([Gf.Vec3f(0.48, 0.50, 0.55)])
    UsdPhysics.CollisionAPI.Apply(bit.GetPrim())
    collision = PhysxSchema.PhysxCollisionAPI.Apply(bit.GetPrim())
    collision.CreateContactOffsetAttr(0.001)
    collision.CreateRestOffsetAttr(0.0)
    return prim


def spawn_trigger(prim_path, cfg, *args, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from pxr import Gf, Sdf, UsdPhysics, PhysxSchema
    from task import TRIGGER_LOCAL, TRIGGER_TRAVEL

    prim = spawn_cuboid.__wrapped__(prim_path, cfg, *args, **kwargs)
    PhysxSchema.PhysxContactReportAPI.Apply(prim)
    namespace = prim_path.rsplit("/", 1)[0]
    joint = UsdPhysics.PrismaticJoint.Define(prim.GetStage(), namespace + "/TriggerJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(namespace + "/drill")])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(prim_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*TRIGGER_LOCAL))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(TRIGGER_TRAVEL)
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(150.0)
    drive.CreateDampingAttr(2.0)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateMaxForceAttr(5.0)
    return prim


class WoodDrillEmbodiment(YamArenaEmbodiment):
    def __init__(self, **kwargs):
        global ACTIVE
        super().__init__(**kwargs)
        ACTIVE = self
        self.latest = None
        self._diagnostics = None
        self._steps = 0
        self._states_time = self._robot_time = None

    def _ensure_sim(self, goal):
        if self._env is not None:
            return
        if adapter._APP is None:
            from isaaclab.app import AppLauncher
            options = dict(headless=self.headless, device=self.device, enable_cameras=True)
            if not self.headless:
                options["visualizer"] = ["kit"]
            adapter._APP = AppLauncher(**options).app
            atexit.register(adapter._close_app_at_exit)

        from yam_arena import scene, embodiment
        from isaaclab.sensors import ContactSensorCfg
        from isaaclab_physx.sensors.contact_sensor import ContactSensor
        from isaaclab.sim.utils import clone
        from isaaclab_arena.utils.configclass import make_configclass
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_physx.sim.schemas import PhysxRigidBodyPropertiesCfg

        original_object, original_embodiment = scene.task_object, embodiment.BimanualYamEmbodiment

        class SubstepContacts(ContactSensor):
            def update(self, dt, force_recompute=False):
                super().update(dt, force_recompute=True)

        def make_object(spec):
            obj = original_object(spec)
            if spec.name == "drill":
                obj.spawn_cfg_addon["func"] = clone(spawn_drill)
                obj.object_cfg = obj._init_object_cfg()
            elif spec.name == "trigger":
                obj.spawner_cfg.func = clone(spawn_trigger)
                obj.object_cfg = obj._init_object_cfg()
            elif spec.name == "wood":
                # Kinematic fixture: the workpiece stays secured, while its contacts
                # can be filtered independently from the tool's grasp contacts.
                obj.object_type = ObjectType.RIGID
                obj.spawner_cfg.rigid_props = PhysxRigidBodyPropertiesCfg(kinematic_enabled=True)
                obj.object_cfg = obj._init_object_cfg()
            elif spec.name == "target":
                obj.spawner_cfg.collision_props.collision_enabled = False
                obj.object_cfg = obj._init_object_cfg()
            return obj

        def make_embodiment(*args, **kwargs):
            kwargs["contact_sensors"] = True
            emb = original_embodiment(*args, **kwargs)
            # A filtered sensor provides proof that the left arm touches the trigger,
            # independently of the trigger displacement and of the scripted policy.
            trigger_sensor = ContactSensorCfg(
                class_type=SubstepContacts,
                prim_path="{ENV_REGEX_NS}/trigger",
                filter_prim_paths_expr=[
                    "{ENV_REGEX_NS}/LeftArm" + emb.link_paths[name]
                    for name in ("tip_left", "tip_right")
                ],
                history_length=12,
            )
            emb.scene_config = make_configclass(
                "WoodDrillScene", [
                    ("trigger_contact", ContactSensorCfg, trigger_sensor),
                    ("bit_wood_contact", ContactSensorCfg, ContactSensorCfg(
                        class_type=SubstepContacts,
                        prim_path="{ENV_REGEX_NS}/drill",
                        filter_prim_paths_expr=["{ENV_REGEX_NS}/wood"], history_length=12,
                    )),
                    ("drill_grip_contact", ContactSensorCfg, ContactSensorCfg(
                        class_type=SubstepContacts,
                        prim_path="{ENV_REGEX_NS}/drill",
                        filter_prim_paths_expr=[
                            "{ENV_REGEX_NS}/RightArm" + emb.link_paths[name]
                            for name in ("tip_left", "tip_right")
                        ], history_length=12,
                    )),
                ],
                bases=(type(emb.scene_config),),
            )()
            return emb

        scene.task_object, embodiment.BimanualYamEmbodiment = make_object, make_embodiment
        try:
            super()._ensure_sim(goal)
        finally:
            scene.task_object, embodiment.BimanualYamEmbodiment = original_object, original_embodiment

    def robot_state(self):
        if self._robot_time == self._t:
            return self._robot_cache
        from yam_arena.runtime import tcp_world
        base = self._base
        right, left = base.scene["right_robot"], base.scene["left_robot"]
        tip_sensor = base.scene["trigger_contact"]
        def control_period_force(sensor):
            history = array(sensor.data.normal_force_matrix_w_history)[0]
            return float(np.linalg.norm(history,axis=-1).sum(axis=(1,2)).mean())
        trigger_force = control_period_force(tip_sensor)
        wood_force = control_period_force(base.scene["bit_wood_contact"])
        grip_history = array(base.scene["drill_grip_contact"].data.normal_force_matrix_w_history)[0]
        finger_forces = np.linalg.norm(grip_history, axis=-1).sum(axis=1).mean(axis=0)
        right_ids = right.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        gap = float(np.sum(np.abs(array(right.data.joint_pos)[0, right_ids])))
        rtcp, ltcp = tcp_world(right), tcp_world(left)
        drill = self.object_states()["drill"]
        from task import MODEL_CENTRE
        from yam_arena.urdf import quat_xyzw_to_matrix
        rot = quat_xyzw_to_matrix(drill.quat_xyzw)
        handle = np.asarray(drill.pos) + rot @ (np.array([0.040, 0.005, 0.0]) - MODEL_CENTRE)
        held = self.task.module.holding_handle(rtcp, handle, gap, finger_forces)
        self._robot_cache = {
            "right_holding": held, "left_trigger_contact": trigger_force > 0.05,
            "bit_wood_contact": wood_force > 0.05, "wood_contact_force_n": wood_force,
            "trigger_contact_force_n": trigger_force, "right_gripper_gap_m": gap,
            "right_pad_drill_forces_n": finger_forces.tolist(),
            "right_tcp_m": rtcp.tolist(), "left_tcp_m": ltcp.tolist(),
        }
        self._robot_time = self._t
        return self._robot_cache

    def object_states(self):
        from yam_arena.scene import object_states
        if self._states_time != self._t:
            self._states_cache = object_states(self._env, self.task.names)
            self._states_time = self._t
        return self._states_cache

    def reset(self, scene, *, seed=None):
        self._steps = 0
        self._states_time = self._robot_time = None
        self.task.module._robot = {}
        try:
            result = super().reset(scene, seed=seed)
        except Exception:
            import traceback
            traceback.print_exc()
            raise
        self.latest = None
        self._states_time = self._robot_time = None
        return result

    def _task_step(self):
        module = self.task.module
        module._robot = self.robot_state()
        self.latest = module.measure(self.object_states(), module._robot)
        self._steps += 1
        if self._steps % 20 == 0:
            m = self.latest
            print(f"[wood drill] t={self._t:.1f} held={m['right_holding']} "
                  f"tilt={m['tilt_deg']:.1f} xy={m['target_error_m']*1000:.1f}mm "
                  f"z={m['surface_error_m']*1000:.1f}mm trigger={m['trigger_travel_m']*1000:.1f}mm "
                  f"left_force={m['trigger_contact_force_n']:.2f}N wood_force={m['wood_contact_force_n']:.2f}N success={m['success_now']}", flush=True)
        if self._diagnostics:
            with Path(self._diagnostics).open("a") as stream:
                stream.write(json.dumps({"time_s": self._t, **self.latest}) + "\n")
        result = super()._task_step()
        result.info.update(self.latest)
        return result
