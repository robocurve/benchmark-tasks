"""Read native contacts on the connector locks and between the stock arms."""

import json

import numpy as np


class Contacts:
    def __init__(self, names, path):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        arms = {"LeftArm": [], "RightArm": []}
        parts = {}
        for prim in get_current_stage().Traverse():
            path_string = prim.GetPath().pathString
            elements = path_string.split("/")
            if "env_0" not in elements or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            for arm, bodies in arms.items():
                if arm in elements:
                    bodies.append(path_string)
            if elements[-2] in names and elements[-1] == "body":
                parts[elements[-2]] = path_string
            if elements[-2] == "plug" and elements[-1] in ("safety", "latch", "detent"):
                parts[elements[-1]] = path_string
        self.left, self.right = arms.values()
        physics = PhysxManager.get_physics_sim_view()
        self.arm_view = physics.create_rigid_contact_view(
            self.left, filter_patterns=[self.right] * len(self.left),
            max_contact_data_count=128 * len(self.left),
        )
        fingers = [body for arm in arms.values() for body in arm
                   if body.rsplit("/", 1)[-1] in ("tip_left", "tip_right")]
        if len(fingers) != 4:
            raise RuntimeError("Expected two fingers on each YAM arm")
        filters = [parts[name] for name in names]
        self.part_view = physics.create_rigid_contact_view(
            fingers, filter_patterns=[filters] * 4, max_contact_data_count=512
        )
        self.pick_surfaces = ("safety", "latch", "plug", "socket")
        self.pick_view = physics.create_rigid_contact_view(
            [parts["pick"]], filter_patterns=[[parts[name] for name in self.pick_surfaces]], max_contact_data_count=128,
        ) if "pick" in parts else None
        self.cpa_internal = physics.create_rigid_contact_view(
            [parts["safety"]], filter_patterns=[[parts["detent"], parts["latch"]]], max_contact_data_count=64,
        ) if path and "detent" in parts else None
        self.hook_view = physics.create_rigid_contact_view(
            [parts["latch"]], filter_patterns=[[parts["socket"]]], max_contact_data_count=64,
        ) if path else None
        self.mating_view = physics.create_rigid_contact_view(
            [parts["plug"]], filter_patterns=[[parts["socket"]]], max_contact_data_count=128,
        ) if path else None
        self.names = names
        self.path = path
        self.count = 0
        self.substep = 0
        self.vectors = np.zeros((4, len(names), 3))
        self.loads = np.zeros((4, len(names)))

    def poll(self, dt):
        self.substep += 1
        self.dt = dt
        if self.path:
            _, _, _, separation, counts, starts = self.arm_view.get_contact_data(dt=dt)
            separation, counts, starts = (np.asarray(value.numpy()) for value in (separation, counts, starts))
            for i, j in zip(*np.nonzero(counts)):
                start = starts[i, j]
                gap = float(separation[start:start + counts[i, j]].min())
                if gap <= 0:
                    self.count += 1
                    with open(self.path, "a") as stream:
                        stream.write(json.dumps(dict(substep=self.substep, separation_m=gap,
                                                     left=self.left[i], right=self.right[j])) + "\n")
        data = self.part_view.get_contact_force_matrix(dt=dt)
        self.vectors = np.asarray(data.numpy())
        self.loads = np.linalg.norm(self.vectors, axis=-1)

    def surface_contacts(self, part):
        """Return individual fingertip forces and contact geometry for one body."""
        yield from self._surface_contacts(self.part_view, self.names.index(part))

    def pick_contacts(self, part="safety"):
        if self.pick_view is not None:
            yield from self._surface_contacts(self.pick_view, self.pick_surfaces.index(part))

    def pick_pad_contacts(self):
        for _, force, point, normal in self.pick_contacts("latch"):
            yield 4, force, point, normal

    def internal_cpa_contacts(self):
        if self.cpa_internal is None:
            return
        for index, name in enumerate(("detent", "latch")):
            for _, force, point, normal in self._surface_contacts(self.cpa_internal, index):
                yield name, force, point, normal

    def hook_contacts(self):
        if self.hook_view is not None:
            yield from self._surface_contacts(self.hook_view, 0)

    def housing_contacts(self):
        if self.mating_view is not None:
            yield from self._surface_contacts(self.mating_view, 0)

    def _surface_contacts(self, view, column):
        if not self.substep:
            return
        force, point, normal, _, counts, starts = (
            np.asarray(value.numpy()) for value in view.get_contact_data(dt=self.dt)
        )
        for finger in range(counts.shape[0]):
            first = int(starts[finger, column])
            stop = first + int(counts[finger, column])
            for value, location, direction in zip(force[first:stop], point[first:stop], normal[first:stop]):
                load = abs(float(np.asarray(value).item()))
                if load > .05:
                    yield finger, load, location, direction
