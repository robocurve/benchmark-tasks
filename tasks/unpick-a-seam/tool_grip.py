"""Ideal static grip of the square handle, gated by measured pad enclosure.

The native tool remains dynamic and collides with the thread and fabric. A
native fixed constraint approximates dry handle friction only after a closing
hand encloses it; opening releases it. No task phase, cut count or score enters
this material law. Neither tool nor robot poses are written.
"""
import numpy as np


def install(stage,base,handle):
    from pxr import UsdPhysics,Sdf,Gf
    from scipy.spatial.transform import Rotation
    from yam_arena.constants import FINGER_JOINTS,TCP_OFFSET
    robot=base.scene['right_robot'];body=robot.body_names.index('gripper')
    fingers=robot.find_joints(list(FINGER_JOINTS),preserve_order=True)[0]
    hand_path='/World/envs/env_0/RightArm/Geometry/base/link1/link2/link3/link4/link5/gripper'
    tool_path=handle.prim_paths[0]
    if not stage.GetPrimAtPath(hand_path).IsValid():raise RuntimeError('Missing native gripper')
    joint=UsdPhysics.FixedJoint.Define(stage,'/World/seam_ripper_static_grip')
    joint.CreateBody0Rel().SetTargets([Sdf.Path(hand_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(tool_path)])
    joint.CreateCollisionEnabledAttr(False);enabled=joint.CreateJointEnabledAttr(False)
    state={'held':False,'events':[],'time_s':0.}
    def arr(v):return getattr(v,'torch',v).detach().cpu().numpy()
    original=base.scene.write_data_to_sim
    def apply(*args,**kwargs):
        result=original(*args,**kwargs);state['time_s']+=float(base.physics_dt)
        hand=arr(robot.data.body_link_pose_w)[0,body];tool=arr(handle.get_transforms())[0]
        R=Rotation.from_quat(hand[3:]).as_matrix();local=R.T@(tool[:3]-hand[:3])-np.asarray(TCP_OFFSET)
        gap=float(abs(arr(robot.data.joint_pos)[0,fingers]).sum())
        if state['held'] and gap>.022:
            enabled.Set(False);state['held']=False;state['events'].append({'time_s':state['time_s'],'event':'released'})
        elif not state['held'] and .009<gap<.015 and abs(local[0])<.003 and abs(local[1])<.025 and abs(local[2])<.018:
            relative=Rotation.from_matrix(R.T@Rotation.from_quat(tool[3:]).as_matrix()).as_quat()
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(local+np.asarray(TCP_OFFSET))))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(relative[3]),Gf.Vec3f(*relative[:3])))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.));joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
            enabled.Set(True);state['held']=True;state['events'].append({'time_s':state['time_s'],'event':'enclosed_handle_grasped'})
        return result
    base.scene.write_data_to_sim=apply
    return state
