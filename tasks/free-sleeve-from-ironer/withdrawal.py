"""Distances in the station frame using outward-rounded bounds of the YAM link meshes."""
import numpy as np

# Wrist and jaw link boxes, metres, in each URDF link's local frame.
BOUNDS={
    "link4": ((-.099,-.014,-.103),(.018,.074,.018)),
    "link5": ((.000,-.029,-.036),(.070,.029,.034)),
    "gripper": ((-.052,-.033,-.011),(.051,.033,.064)),
    "tip_left": ((-.011,-.069,-.091),(.058,.004,.004)),
    "tip_right": ((-.058,-.004,-.091),(.011,.069,.004)),
}
# Includes the board, full roller span, release lever and normally seated surrogate.
WORK_LOW=np.array((-.417,-.254,0.))
WORK_HIGH=np.array((.100,.168,.200))
MIN_CLEAR=.035
MIN_OPEN=.80  # per jaw, fraction of its full measured travel
def arm_state(poses, finger_positions, side):
    from yam_arena.constants import FINGER_OPEN
    opening=min(float(q)/FINGER_OPEN for q in finger_positions)
    clear=float("inf");outboard=float("inf")
    for name,(lower,upper) in BOUNDS.items():
        origin,basis=poses[name]
        lo,hi=np.asarray(lower),np.asarray(upper)
        centre=origin+basis@((lo+hi)/2)
        radius=np.abs(basis)@((hi-lo)/2)
        lo,hi=centre-radius,centre+radius
        if not np.isfinite(lo).all() or not np.isfinite(hi).all():raise ValueError("Invalid measured link pose")
        gap=np.maximum(0.,np.maximum(WORK_LOW-hi,lo-WORK_HIGH))
        clear=min(clear,float(np.linalg.norm(gap)))
        outboard=min(outboard,float(lo[1]-WORK_HIGH[1] if side=="left" else WORK_LOW[1]-hi[1]))
    return opening,clear,outboard

def native_arm_state(env,side):
    from yam_arena import YAM_URDF,ik,station
    from yam_arena.constants import ALL_JOINTS
    from scenekit import arm_joints
    from fixture import WORLD
    measured=arm_joints(env,side,ALL_JOINTS)
    fk=ik.forward_kinematics(YAM_URDF,dict(zip(ALL_JOINTS,measured)))
    offset=np.asarray(station.BASES[side])-WORLD
    poses={name:(mat[:3,3]+offset,mat[:3,:3]) for name,mat in fk.items()}
    state=arm_state(poses,measured[-2:],side)
    if side=="right":
        from surface import clearance
        return state,clearance(poses)
    return state,{}
