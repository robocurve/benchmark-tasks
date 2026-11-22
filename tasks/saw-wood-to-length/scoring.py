"""End-state scoring and conservative whole-saw clearance, without simulator IO."""
import itertools
import numpy as np

# Match the four physical saw boxes in task.py: grip, blade, teeth, guard.
SAW_BOXES=((0,0,0,.026,.110,.028),
           (0,-.155,-.014,.0006,.200,.040),
           (0,-.155,-.0345,.001,.200,.001),
           (0,-.050,0,.030,.010,.032))
SIGNS=np.array(list(itertools.product((-1,1),repeat=3)))


def rotation(q):
    x,y,z,w=map(float,q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])


def saw_clear(saw_pose,wood_bodies,margin=.005):
    """5 mm lower-bound gap between saw and every remaining wood collider.

    Bound each saw box in each wood body's frame. Bounding boxes can reject a
    close diagonal pose conservatively, but cannot accept an actual overlap.
    Wood bodies include the retained stock AND the freely moving offcut.
    """
    sp=np.asarray(saw_pose);rs=rotation(sp[3:7])
    for pose,boxes in wood_bodies:
        pose=np.asarray(pose);rw=rotation(pose[3:7])
        wood=np.asarray(boxes,dtype=float)
        if not len(wood):continue
        lower=wood[:,:3]-wood[:,3:]/2;upper=wood[:,:3]+wood[:,3:]/2
        for box in SAW_BOXES:
            b=np.asarray(box)
            corners=(b[:3]+SIGNS*b[3:]/2)@rs.T+sp[:3]
            local=(corners-pose[:3])@rw
            lo=local.min(axis=0);hi=local.max(axis=0)
            gap=np.maximum(0,np.maximum(lower-hi,lo-upper))
            if np.any(np.linalg.norm(gap,axis=1)<margin-1e-12):return False
    return True


def end_score(geometry,clamp_ok,tool_clear,cut_before_clamp):
    # Every credited end state must be clamped, safe to measure, and have no
    # measurable (>=0.5mm per column) cumulative removal while stock was loose.
    if not clamp_ok or not tool_clear or cut_before_clamp:return 0.
    return float(geometry['stage_value'])


def clamp_state(wood_pose,jaw_y,angle):
    gap=-.035-float(jaw_y)
    position_ok=np.all(np.abs(np.asarray(wood_pose[:3])-[.20,-.03,.8275])<.003)
    # A stock flipped or rotated out of the jaws is not correctly clamped.
    q=np.asarray(wood_pose[3:7],float)
    aligned=2*np.arccos(np.clip(abs(q[3])/np.linalg.norm(q),0,1))<=np.deg2rad(2)+1e-12
    return bool(.0192-1e-12<=gap<=.0202+1e-12 and angle>1.5 and position_ok and aligned),gap
