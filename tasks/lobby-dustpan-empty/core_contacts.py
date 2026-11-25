"""Named PhysX contact evidence for the four debris items (rev3)."""
import numpy as np

CORE_NAMES = ("core_1", "paper_ball", "bottle_cap", "card")   # rev3 mixed debris: every item is a sensor


def enable_core_contact_reports(prim):
    """Call when each core's rigid body is authored, before PhysX starts."""
    from pxr import PhysxSchema

    report = PhysxSchema.PhysxContactReportAPI.Apply(prim)
    report.CreateThresholdAttr().Set(0.0)


def _numpy(data):
    if hasattr(data, "detach"):
        return data.detach().cpu().numpy()
    return data.numpy()


class TensorCoreContactMonitor:
    def __init__(self, view, filters, pan_body):
        self.view = view
        self.filters = filters
        self.pan_body = pan_body
        self.first_hand_contact = None
        self.support_events = {name: set() for name in CORE_NAMES}
        self.failed = False
        self.sample_count = 0
        self.subscription = None

    @classmethod
    def attach(cls, env):
        from omni.physx import get_physx_interface
        from pxr import Usd, UsdPhysics

        base = env.unwrapped
        scene = base.scene
        if len(scene.env_prim_paths) != 1:
            raise RuntimeError("The dustpan scorer requires one active trial per process.")
        root = scene.env_prim_paths[0]
        sensors = [f"{root}/{name}/body" for name in CORE_NAMES]
        pan_body = f"{root}/dustpan/body"
        filters = [pan_body, f"{root}/wastebasket/body"]
        for name, arm in (("left_robot", "LeftArm"), ("right_robot", "RightArm")):
            arm_prim = scene.stage.GetPrimAtPath(f"{root}/{arm}")
            bodies = [
                str(prim.GetPath())
                for prim in Usd.PrimRange(arm_prim, Usd.TraverseInstanceProxies())
                if prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ]
            if not set(scene[name].body_names) <= {path.rsplit("/", 1)[-1] for path in bodies}:
                raise RuntimeError(f"Could not resolve every contact filter for {arm}.")
            filters.extend(bodies)
        view = base.sim.physics_sim_view.create_rigid_contact_view(
            sensors, filter_patterns=[filters] * len(sensors), max_contact_data_count=512
        )
        if view.sensor_count != len(sensors) or view.filter_count != len(filters):
            raise RuntimeError("PhysX contact view returned a mismatched sensor or filter count.")
        monitor = cls(view, filters, pan_body)
        monitor.subscription = get_physx_interface().subscribe_physics_on_step_events(
            monitor.capture, False, 0
        )
        if monitor.subscription is None:
            raise RuntimeError("Could not subscribe to post-physics contact samples.")
        print(f"[dustpan-contacts] tensor view: {view.sensor_count} cores, "
              f"{view.filter_count} named filters", flush=True)
        return monitor

    def close(self):
        self.subscription = None

    def capture(self, dt):
        if self.failed:
            return
        try:
            normal, _, _, separation, counts, offsets = self.view.get_contact_data(dt=dt)
            counts = _numpy(counts)
            self.sample_count += 1
            if not counts.any():
                return
            offsets = _numpy(offsets)
            normal = _numpy(normal).reshape(-1)
            separation = _numpy(separation).reshape(-1)
            for row, column in np.argwhere(counts > 0):
                start = int(offsets[row, column])
                end = start + int(counts[row, column])
                if start < 0 or end > len(normal):
                    raise ValueError("contact data buffer overflow")
                force = normal[start:end]
                gap = separation[start:end]
                if not np.isfinite(force).all() or not np.isfinite(gap).all():
                    raise ValueError("nonfinite tensor contact sample")
                if not (np.any(force != 0.0) or np.any(gap <= 0.0)):
                    continue
                name, other = CORE_NAMES[row], self.filters[column]
                if column < 2:
                    self.support_events[name].add(other)
                elif self.first_hand_contact is None:
                    self.first_hand_contact = (name, other)
                    print(f"[dustpan-contacts] forbidden arm/core contact: {name} {other}", flush=True)
        except Exception as exc:
            self.failed = True
            print(f"[dustpan-contacts] tensor reporter error: {exc}", flush=True)

    @property
    def verified(self):
        return not self.failed and all(
            self.pan_body in self.support_events[name] for name in CORE_NAMES
        )
