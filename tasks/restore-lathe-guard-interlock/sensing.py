"""Common labeling for all robot links and the two contact systems."""
import numpy as np
from fixture import WORLD

def clearance(links):
    # Distal link capsules. Test all segments against the chuck/guard work volume.
    low=np.array([-.147,-.148,.032]);high=np.array([.100,.085,.258])
    margin=10.
    for poses in links.values():
        for a,b,radius in (("link4","link5",.038),("link5","gripper",.040),
                           ("gripper","tip_left",.030),("gripper","tip_right",.030)):
            p=np.asarray(poses[a]);q=np.asarray(poses[b])
            pts=p+(q-p)*np.linspace(0,1,15)[:,None]
            gaps=np.maximum(np.maximum(low-pts,pts-high),0.)
            margin=min(margin,float(np.linalg.norm(gaps,axis=1).min())-radius-float(np.linalg.norm(q-p))/28)
    return margin

def classify(a,b,sa,sb):
    names={a,b};shapes={sa,sb}
    robot=any(x.startswith(("left_","right_")) for x in names)
    person=any(x.startswith("operator_") for x in shapes)
    return (person and (robot or bool(names & {"insert","rotor","shield","cam"})),
            robot and "shield" in names,
            robot and bool(names & {"shield","cam","insert","rotor","testpad","button"}),
            "insert" in names and "tool_tray" in shapes,
            next((x for x in names if x.endswith(("tip_left","tip_right"))),"") if "insert" in names else "")


def opposed_pads(points):
    """Both pads of one gripper, on separated faces with opposing contact normals."""
    for side in ("left","right"):
        left = points.get(side+"_tip_left",())
        right = points.get(side+"_tip_right",())
        for p,n in left:
            for q,m in right:
                separation = np.asarray(p)-q
                if np.dot(n,m)<-.5 and abs(np.dot(separation,n))>.006:
                    return True
    return False
