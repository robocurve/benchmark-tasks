"""Existing rigid spatula mesh, PhysX contact reports, and episode history."""
from pathlib import Path
import numpy as np
from yam_arena.inspect_embodiment import YamArenaEmbodiment

DEMO_STATES = {}


class RetrievalEmbodiment(YamArenaEmbodiment):
    def _ensure_sim(self, goal):
        import os, atexit
        from yam_arena import inspect_embodiment as base
        if base._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
            base._APP = AppLauncher(headless=self.headless, device=self.device, enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        import isaaclab.sim as sim_utils
        path = Path(__file__).resolve().parent.parent / "assets" / "spatula.usdc"
        original = scene.task_object

        def make_object(spec):
            if spec.name == "recovery_hood":
                return Object(name=spec.name,
                              usd_path=str(path.parent / "recovery_hood.usdc"),
                              object_type=ObjectType.BASE,
                              initial_pose=Pose(position_xyz=tuple(spec.pos)),
                              spawn_cfg_addon={"physics_material": scene._grippy(),
                                               "collision_props": scene._contact_props()})
            if spec.name == "spatula":
                return Object(name=spec.name, usd_path=str(path), object_type=ObjectType.RIGID,
                              initial_pose=Pose(position_xyz=tuple(spec.pos)),
                              spawn_cfg_addon={"physics_material": scene._grippy(),
                                  "mass_props": {"": [sim_utils.MassCfg(mass=.08)]},
                                  "mass_props_create_if_missing": True,
                                  "collision_props": scene._contact_props()})
            obj = original(spec)
            if spec.name == "part":
                obj.spawner_cfg.activate_contact_sensors = True
                obj.object_cfg = obj._init_object_cfg()
            return obj

        scene.task_object = make_object
        try:
            super()._ensure_sim(goal)
        finally:
            scene.task_object = original
        # Isaac Lab's backend disables Python contact processing by default.
        # Re-enable reports; this does not change collision response.
        self._base.sim.set_setting("/physics/disableContactProcessing", False)
        if not hasattr(self, "_contact_subscription"):
            import omni.physx
            self._contact_subscription = omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self._contacts)

    def _contacts(self, headers, data):
        from pxr import PhysicsSchemaTools
        for h in headers:
            # CONTACT_LOST carries no current contact points.
            if h.num_contact_data == 0:
                continue
            a = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            b = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            self.task.module.HISTORY.contact(a, b)

    def reset(self, scene, *, seed=None):
        from tool_retrieval.scoring import History
        self.task.module.HISTORY = History()
        observation = super().reset(scene, seed=seed)
        self._base.sim.set_setting("/physics/disableContactProcessing", False)
        self._states()
        return observation

    def _states(self):
        from yam_arena.scene import object_states
        global DEMO_STATES
        DEMO_STATES = object_states(self._env, self.task.names)
        return DEMO_STATES

    def _task_step(self):
        from yam_arena import ik, station
        from yam_arena.constants import FINGER_OPEN
        states = self._states()
        h = self.task.module.HISTORY
        h.update(states, self.task.module.TOP)
        clear = True
        robots = {}
        for side, key in (("left", "left_robot"), ("right", "right_robot")):
            arm_ids, finger_ids = self._ids[key]
            q = self._base.scene[key].data.joint_pos.torch[0]
            tcp, _ = ik.tcp_pose(q[arm_ids].cpu().numpy(), station.BASES[side])
            opened = q[finger_ids].mean().item() / FINGER_OPEN > .75
            robots[side] = {"tcp": [round(float(x),4) for x in tcp], "opening": round(q[finger_ids].mean().item() / FINGER_OPEN,3)}
            clear &= opened and all(np.linalg.norm(tcp - states[k].pos) > .09 for k in ("part", "spatula"))
        h.open_clear = bool(clear)
        result = super()._task_step()
        if round(self._t * self.info.control_hz) % 20 == 0:
            print("RETRIEVAL_STATE", round(self._t, 2),
                  {k: [round(float(x), 4) for x in states[k].pos] for k in ("part", "spatula")},
                  vars(h), "robots", robots, "tool_quat", states['spatula'].quat_xyzw.tolist(),
                  "tool_height", float(states['spatula'].high[2]-states['spatula'].low[2]),
                  "score", result.info.get("score"), flush=True)
        return result


def make_embodiment(**kwargs):
    return RetrievalEmbodiment(**kwargs)
