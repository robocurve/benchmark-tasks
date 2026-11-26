"""Read native contact force from the cart's wall leaves and floor supports."""

import numpy as np


class Contacts:
    def __init__(self):
        import carb
        from isaaclab.sim.utils.stage import get_current_stage
        from isaaclab_physx.physics.physx_manager import PhysxManager
        from pxr import UsdPhysics

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        paths = [str(p.GetPath()) for p in get_current_stage().Traverse()
                 if "env_0" in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI)]

        def body(name):
            return next(p for p in paths if p.lower().endswith(f"/{name}/body"))

        fingers = [[p for p in paths if f"/{side}Arm/" in p and p.rsplit('/',1)[-1] in ("tip_left","tip_right")]
                   for side in ("Left","Right")]
        if not all(len(p) == 2 for p in fingers):
            raise ValueError(f"Could not find both stock tip pairs: {fingers}")
        sim = PhysxManager.get_physics_sim_view()
        walls = [body(f"{side}_{leaf}_leaf") for side in ("left","right") for leaf in ("back","front")]
        self.walls = sim.create_rigid_contact_view(walls, filter_patterns=[*([fingers[0]]*2),*([fingers[1]]*2)],
                                                   max_contact_data_count=96)
        palms = [[next(p for p in paths if f"/{side}Arm/" in p and p.endswith("/gripper"))]
                 for side in ("Left","Right")]
        self.wall_palms = sim.create_rigid_contact_view(walls,filter_patterns=[*([palms[0]]*2),*([palms[1]]*2)],
                                                       max_contact_data_count=96)
        self.floor = sim.create_rigid_contact_view([body("cart_floor")],
                                filter_patterns=[[body("cart_front"),body("cart_back")]],max_contact_data_count=96)
        self.floor_wall = sim.create_rigid_contact_view([body("cart_floor")],
                                filter_patterns=[walls],max_contact_data_count=96)
        bars = [body(f"{side}_rigidity_bar") for side in ("left","right")]
        self.floor_bar = sim.create_rigid_contact_view([body("cart_floor")],
                                 filter_patterns=[bars],max_contact_data_count=96)
        self.bar_capture = sim.create_rigid_contact_view(
            bars,filter_patterns=[[walls[1]],[walls[3]]],max_contact_data_count=96,
        )
        left = [p for p in paths if "/LeftArm/" in p]
        right = [p for p in paths if "/RightArm/" in p]
        self.cross = sim.create_rigid_contact_view(left,filter_patterns=[right]*len(left),
                                                    max_contact_data_count=64*len(left))
        self.arm_collisions = 0
        self.clear()

    def clear(self):
        self.loads = np.zeros(2)
        self.palm_loads = np.zeros(2)
        self.floor_force = 0.
        self.floor_end_force = np.zeros(2)
        self.floor_pinches = np.zeros(4)
        self.bar_forces = np.zeros(2)
        self.floor_bar_force = np.zeros(2)

    def poll(self, dt):
        # Force on positive-Y leaf is inward when its signed force is negative.
        r = self.walls.get_contact_force_matrix(dt=dt).numpy().reshape(2,2,2,3).sum(axis=(1,2))
        self.loads = np.maximum(self.loads, r[:,1]*[-1.,1.])
        palms = self.wall_palms.get_contact_force_matrix(dt=dt).numpy().reshape(2,2,3).sum(axis=1)
        self.palm_loads = np.maximum(self.palm_loads,palms[:,1]*[-1.,1.])
        floor = self.floor.get_contact_force_matrix(dt=dt).numpy()[0,:,2]
        self.floor_force = max(self.floor_force, float(floor.sum()))
        self.floor_end_force = np.maximum(self.floor_end_force,floor)
        edge = self.floor_wall.get_contact_force_matrix(dt=dt).numpy()[0]
        self.floor_pinches = np.maximum(self.floor_pinches,np.linalg.norm(edge,axis=1))
        receiver = self.bar_capture.get_contact_force_matrix(dt=dt).numpy()[:,0]
        self.bar_forces = np.maximum(self.bar_forces,np.linalg.norm(receiver,axis=1))
        interaction = self.floor_bar.get_contact_force_matrix(dt=dt).numpy()[0]
        self.floor_bar_force = np.maximum(self.floor_bar_force,np.linalg.norm(interaction,axis=1))
        _,_,_,gaps,count,start = (np.asarray(a.numpy()) for a in self.cross.get_contact_data(dt=dt))
        self.arm_collisions += sum(np.any(gaps[i:i+n] <= 0.) for i,n in zip(start.flat,count.flat) if n)
