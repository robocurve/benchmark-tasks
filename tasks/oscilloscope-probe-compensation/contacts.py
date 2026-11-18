"""Read impulses at the native trimmer link and identify the loose driver."""

import numpy as np

from .geometry import TRIMMER_CENTER


class TrimmerContacts:
    def __init__(self, base):
        import omni.usd
        from omni.physx import get_physx_interface
        from pxr import Usd, UsdPhysics

        self.base = base
        self.dt = float(base.sim.get_physics_dt())
        self.step = base.sim.get_physics_step_count()
        self.consumed_step = self.step
        self.error = None
        self.subscription = None
        self.cache = {}
        self.peak_force = 0.0
        self.peak_torque = 0.0
        self.count = 0
        stage = omni.usd.get_context().get_stage()
        paths = [str(prim.GetPath()) for prim in stage.Traverse(Usd.TraverseInstanceProxies())
                 if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        trimmers = [path for path in paths if path.endswith("/scope/trimmer")]
        tools = [path for path in paths if path.endswith("/tool/driver")]
        if len(trimmers) != 1 or len(tools) != 1:
            raise RuntimeError(f"Expected one native trimmer and tool: {trimmers}, {tools}")
        self.tool_path = tools[0]
        self.view = base.sim.physics_manager.get_physics_sim_view().create_rigid_contact_view(
            trimmers, filter_patterns=[], max_contact_data_count=128,
        )
        if list(self.view.sensor_paths) != trimmers:
            raise RuntimeError("Native contact view did not bind the trimmer")
        self.subscription = get_physx_interface().subscribe_physics_on_step_events(
            self.sample, pre_step=False, order=0,
        )

    def sample(self, dt):
        if self.error is not None:
            return
        try:
            import warp as wp

            step = self.base.sim.get_physics_step_count()
            if step != self.step + 1 or abs(float(dt) - self.dt) > 1e-9:
                raise RuntimeError("Missing native trimmer contact frame")
            self.step = step
            data = [np.asarray(array.numpy()) for array in self.view.get_raw_contact_data(dt=1.0)]
            if len(data) != 7:
                raise RuntimeError("Unknown native contact format")
            impulse, point, normal, separation, count, offset, ids = data
            size, start = int(count[0]), int(offset[0])
            if size == 0:
                return
            stop = start + size
            if size < 0 or start < 0 or stop > len(impulse) or size >= len(impulse):
                raise RuntimeError("Native trimmer contact buffer was invalid or saturated")
            active = slice(start, stop)
            actor_ids = ids[active]
            unknown = sorted(set(map(int, actor_ids)) - self.cache.keys())
            if unknown:
                found = self.view.get_other_actor_paths_from_ids(
                    wp.array(unknown, dtype=wp.uint64, device="cpu"),
                )
                if len(found) != len(unknown):
                    raise RuntimeError("Native contact actor ID could not be resolved")
                self.cache.update(zip(unknown, map(str, found), strict=True))
            matches = np.asarray([self.cache[int(actor)] == self.tool_path for actor in actor_ids])
            loads = np.abs(impulse[active, 0]) / self.dt
            matches &= (separation[active, 0] <= 0.0001) & (loads > 0.001)
            if not np.any(matches):
                return
            forces = loads[matches, None] * normal[active][matches]
            lever = point[active][matches] - TRIMMER_CENTER
            torque = float(np.sum(np.cross(lever, forces)[:, 2]))
            force = float(np.sum(loads[matches]))
            if not np.isfinite([torque, force]).all():
                raise RuntimeError("Nonfinite native contact load")
            self.count += int(np.count_nonzero(matches))
            self.peak_force = max(self.peak_force, force)
            if abs(torque) > abs(self.peak_torque):
                self.peak_torque = torque
        except Exception as error:
            self.error = repr(error)

    def consume_peak(self):
        """Return peak normal load, count of loaded samples and signed peak Z torque."""
        if self.error is not None:
            raise RuntimeError(f"Trimmer contact observer failed: {self.error}")
        interval = self.base.sim.get_physics_step_count() - self.consumed_step
        if (interval not in (0, round(float(self.base.step_dt) / self.dt))
                or self.step != self.consumed_step + interval):
            raise RuntimeError("Incomplete native trimmer contact interval")
        self.consumed_step = self.step
        result = self.peak_force, self.count, self.peak_torque
        self.peak_force = 0.0
        self.peak_torque = 0.0
        self.count = 0
        return result

    def close(self):
        self.subscription = None
