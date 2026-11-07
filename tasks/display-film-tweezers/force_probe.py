"""Diagnostic contact-load sampling at every physics step, without applying forces.

Sum the normal and tangential contact forces on the tab from both tool halves.
Opposing clamp forces cancel in this resultant; it is the transmitted film load,
not a constant peel-force assumption. The tab must have contact reporting enabled.
"""
import numpy as np


def array(value):
    if hasattr(value,'numpy') and not hasattr(value,'detach'):
        return value.numpy()
    return value.detach().cpu().numpy()


class ContactLoad:
    def __init__(self,emb,tab_path,tool_paths):
        from isaaclab_physx.physics import PhysxManager
        view=PhysxManager.get_physics_sim_view()
        self.contact=view.create_rigid_contact_view([tab_path],filter_patterns=[list(tool_paths)],
                                                    max_contact_data_count=256)
        self.bodies=view.create_rigid_body_view([tab_path,*tool_paths])
        self.robot=emb._base.scene['right_robot'].root_view
        self.fingers=emb._ids['right_robot'][1]
        self.dt=float(emb._base.physics_dt)
        self.samples=[]
        self.sim=emb._base.sim
        self.original=self.sim.step

        def measured_step(*args,**kwargs):
            result=self.original(*args,**kwargs)
            per_jaw=array(self.contact.get_contact_force_matrix(dt=self.dt)).reshape(-1,3)
            normal=per_jaw.sum(axis=0)
            friction,_,counts,starts=self.contact.get_friction_data(dt=self.dt)
            friction=array(friction);counts=array(counts).ravel();starts=array(starts).ravel()
            per_jaw_friction=np.array([friction[int(s):int(s+c)].sum(axis=0) for s,c in zip(starts,counts)])
            tangent=per_jaw_friction.sum(axis=0)
            self.samples.append(dict(normal_N=normal.tolist(),friction_N=tangent.tolist(),
                                     resultant_N=(normal+tangent).tolist(),
                                     jaw_normal_N=per_jaw.tolist(),jaw_friction_N=per_jaw_friction.tolist(),
                                     body_order=['pull_tab','tweezer_a','tweezer_b'],
                                     body_poses=array(self.bodies.get_transforms()).tolist(),
                                     body_velocities=array(self.bodies.get_velocities()).tolist(),
                                     finger_q_m=array(self.robot.get_dof_positions())[0,self.fingers].tolist(),
                                     finger_v_m_s=array(self.robot.get_dof_velocities())[0,self.fingers].tolist(),
                                     # Projected joint reactions are not a direct motor-force sensor.
                                     finger_projected_joint_force_N=array(self.robot.get_dof_projected_joint_forces())[0,self.fingers].tolist()))
            return result
        self.sim.step=measured_step

    def drain(self):
        samples=self.samples;self.samples=[]
        return samples

    def close(self):
        self.sim.step=self.original
