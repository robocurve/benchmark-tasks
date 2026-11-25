"""Optional PhysX diagnostics for contact between the left and right YAM arms."""

import json

import numpy as np


class ArmContacts:
    def __init__(self, path):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        bodies = {"LeftArm": [], "RightArm": []}
        workpieces = {}
        for prim in get_current_stage().Traverse():
            parts = prim.GetPath().pathString.split("/")
            if "env_0" not in parts or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            workpieces["/".join(parts[-2:])] = prim.GetPath().pathString
            for arm in bodies:
                if arm in parts:
                    bodies[arm].append(prim.GetPath().pathString)
        left, right = bodies.values()
        if not left or not right:
            raise RuntimeError("Could not locate both YAM arms for contact diagnostics")

        physics = PhysxManager.get_physics_sim_view()
        def contact(senders, receivers):
            return physics.create_rigid_contact_view(
                senders,filter_patterns=[receivers]*len(senders),
                max_contact_data_count=128*len(senders))

        self.view = contact(left,right)
        fingers = [p for p in left+right if p.rsplit("/",1)[-1] in ("tip_left","tip_right")]
        names = ("harp/left_leg","harp/right_leg","left_sleeve/body","right_sleeve/body",
                 "harp/body","shade/body")
        parts = [workpieces[name] for name in names]
        self.grips = contact(fingers,parts)
        surfaces = ("table_lamp/body","harp/body","shade/body","harp/left_leg","harp/right_leg")
        self.interface = contact(parts,[workpieces[name] for name in surfaces])
        self.lamp = contact(left+right,[workpieces["table_lamp/body"]])
        self.dt = PhysxManager.get_physics_dt()
        self.left = left
        self.right = right
        self.path = path
        self.substeps = self.contacts = 0
        self.record(substep=0,left_bodies=left,right_bodies=right,
                    finger_order=fingers,piece_order=names,interface_order=surfaces)

    def record(self, **event):
        with open(self.path, "a") as log:
            log.write(json.dumps(event) + "\n")

    @staticmethod
    def loads(view,dt):
        forces, _, _, _, counts, starts = view.get_contact_data(dt=dt)
        forces, counts, starts = (np.asarray(v.numpy()) for v in (forces,counts,starts))
        return [[float(np.abs(forces[i:i+n]).sum()) for i,n in zip(row,ns)]
                for row,ns in zip(starts,counts)]

    def snapshot(self):
        interface = self.loads(self.interface,self.dt)
        return {"finger_loads_n":self.loads(self.grips,self.dt),
                "interface_loads_n":interface,
                "lamp_robot_loads_n":[row[0] for row in self.loads(self.lamp,self.dt)],
                "cross_arm_load_n": sum(sum(row) for row in self.loads(self.view,self.dt))}

    def poll(self, dt):
        self.dt = dt
        self.substeps += 1
        _, _, _, separations, counts, starts = self.view.get_contact_data(dt=dt)
        separations, counts, starts = (np.asarray(v.numpy()) for v in (separations, counts, starts))
        for left, right in zip(*np.nonzero(counts)):
            start, count = starts[left, right], counts[left, right]
            gap = float(separations[start:start + count].min())
            if gap <= 0:
                self.contacts += 1
                self.record(substep=self.substeps, time_s=self.substeps * dt, separation_m=gap,
                            left=self.left[left].rsplit("/", 1)[-1],
                            right=self.right[right].rsplit("/", 1)[-1])
