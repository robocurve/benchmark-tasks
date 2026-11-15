"""Ideal dry grip for the top8mm already placed between the bench jaws.

Native fixed constraints represent a tightened bench clamp's static friction.
Only the enclosed upper edge is retained; the remaining panels stay dynamic.
This fixture approximation is independent of the policy, cuts and score.
No cloth or robot state is written during execution.
"""
import numpy as np


def install(stage, shell, view):
    from pxr import UsdPhysics,Sdf,Gf
    poses=np.asarray(view.native_state()[0],dtype=float)
    centres=np.asarray(shell['model']['centres'])
    selected=np.flatnonzero((centres[:,2]>=.8697)&(centres[:,2]<=.8777)&
                            (abs(centres[:,1])<=.0425))
    if not 1<=len(selected)<len(poses)/3:
        raise RuntimeError('Expected only the top edge inside the bench jaws')
    paths=[]
    for i in selected:
        p=poses[i]
        if abs(p[0]-.2996)>.002:
            raise RuntimeError('Bench clamp cannot retain cloth outside its jaws')
        joint=UsdPhysics.FixedJoint.Define(stage,f'/World/fabric_clamp_grip_{i:03d}')
        joint.CreateBody1Rel().SetTargets([Sdf.Path(shell['bodies'][i])])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*p[:3]))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(p[6]),Gf.Vec3f(*p[3:6])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.))
        joint.CreateCollisionEnabledAttr(False)
        joint.CreateJointEnabledAttr(True)
        paths.append(str(joint.GetPath()))
    return dict(model='Ideal static grip of the8mm top edge in the bench clamp',
                retained_panels=selected.tolist(),native_constraints=paths,
                runtime_cloth_state_writes=False,force_limited_slip_calibrated=False)
