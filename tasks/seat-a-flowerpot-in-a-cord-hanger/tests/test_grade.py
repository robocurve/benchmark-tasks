import numpy as np
from rdb_planter import fixed_scene as s
from rdb_planter.grading import measurements,Grader
def seated():
    p=np.array([.38,.04,s.TOP+.11])
    cords=[]
    for a in np.arange(4)*np.pi/2:
        d=np.array([np.cos(a),np.sin(a),0])
        cords.append([p+[0,0,.28],p+d*.062+[0,0,.096],p+d*.050+[0,0,.01],p+[0,0,-.005]])
    return p,cords
def test_requires_load_test_and_clear_hands():
    p,c=seated();m=measurements(p,np.eye(3),p+[0,0,-.005],c,[[.05,.30,1.0]])
    assert m['seated'] and m['hands_clear']
    g=Grader()
    for i in range(120):assert g.update(m,.05)['score']==.5
    g.pulse_steps=3;assert g.update(m,.05)['score']==1
def test_knot_or_cluster_outside_rejected():
    p,c=seated()
    assert not measurements(p,np.eye(3),p+[0,.1,-.005],c,[[0,0,1]])['seated']
    assert not measurements(p,np.eye(3),p,[c[0]]*4,[[0,0,1]])['seated']
