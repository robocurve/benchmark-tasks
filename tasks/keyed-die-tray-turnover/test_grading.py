import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from grading import Grader, EXPECTED

NAMES=['source','receiver','source_frame','receiver_frame','source_knob','receiver_knob','d0','d1','d2','d3','roller0','roller2']
FILTERS=NAMES[:4]+['LeftArm/link3','finger']


def endpoint():
    p=np.zeros((12,7))
    p[:,6]=1
    p[[0,2],0]=.20
    c=np.zeros((12,6),dtype=int)
    c[1,3]=1
    for i,cell in enumerate(EXPECTED,6):
        p[i,:3]=[*np.multiply(.013,cell),-.00075]
        p[i,3:]=[1,0,0,0]
        c[i,1]=1
    return p,c


def grade(p,c,n=21):
    g=Grader(NAMES,FILTERS)
    for _ in range(n): g.update(p,c)
    return g


def test_hold_and_release():
    p,c=endpoint()
    assert grade(p,c,20).score==0.
    g=grade(p,c)
    assert g.score==1.
    for part in (1,3,5):
        cc=c.copy()
        cc[part,4]=1
        assert g.update(p,cc)==0.
        for _ in range(20): assert g.update(p,c)==0.
        assert g.update(p,c)==1.
    p[1,0]+=.003
    assert g.update(p,c)==0.


def test_support_access_and_orientation():
    p,c=endpoint()
    for body in (0,2):
        v=p.copy()
        v[body,:3]=([0,0,.02] if body==0 else [0,-.042,.02])
        assert grade(v,c).score==0.
    s=c.copy()
    s[1,3]=0
    assert grade(p,s).score==0.
    v=p.copy()
    v[1,3:]=Rotation.from_euler('x',12,degrees=True).as_quat()
    assert grade(v,c).score==0.


def test_one_die_failures_and_duplicate():
    p,c=endpoint()
    v,s=p.copy(),c.copy()
    v[6,3:]=[0,0,0,1]
    assert grade(v,s).score==.75
    for dx,dz in ((.002,0.),(0.,.0009),(0.,-.001)):
        v=p.copy()
        v[6,[0,2]] += [dx,dz]
        assert grade(v,c).score==.75
    for i,val in ((1,0),(4,1)):
        s=c.copy()
        s[6,i]=val
        assert grade(p,s).score==.75
    v=p.copy()
    v[6,:3]=v[7,:3]+[0,0,.0015]
    assert grade(v,c).score==.5
    g=grade(p,c)
    assert g.update(v,c)==.5
    assert g.update(p,c)==.5


def test_wedged_recess_and_high_corner():
    p,c=endpoint()
    # A level die remains inside the mouth and touches the tray side walls.
    for gap,expected in ((.00009,1.),(.00011,.75),(.000258,.75)):
        v=p.copy()
        v[6,2]+=gap
        assert grade(v,c).score==expected
    v=p.copy()
    # Pivot around a low edge: one edge is seated and the other stands high.
    tilt=Rotation.from_euler('y',2,degrees=True)
    v[6,3:]=(tilt*Rotation.from_quat(v[6,3:])).as_quat()
    v[6,2] += .0045*np.sin(np.deg2rad(2))
    assert grade(v,c).score==.75


def test_anonymous_identity_and_corrupt_inputs():
    p,c=endpoint()
    p[[6,7]]=p[[7,6]]
    assert grade(p,c).score==1.
    v=p.copy()
    v[6,0]=np.nan
    with pytest.raises(ValueError): grade(v,c)
    c[6,1]=16384
    with pytest.raises(ValueError): grade(p,c)

def test_diagonal_neighbor_does_not_block_mouth_by_bounding_box_alone():
    p,c=endpoint()
    # A different die sits just beyond the northeast corner of the first mouth.
    # Its axis-aligned box overlaps the mouth, but the diamond does not.
    p[7,:2]=p[6,:2]+[.009,.009]
    p[7,3:]=(Rotation.from_euler('z',45,degrees=True)*Rotation.from_euler('x',180,degrees=True)).as_quat()
    assert grade(p,c).good_cells == [True,False,True,True]
    p[7,:2]=p[6,:2]+[.007,.007]
    assert grade(p,c).good_cells == [False,False,True,True]
