import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from grading import bite, backing_rows


@pytest.mark.parametrize('angle',[0,180])
def test_either_broad_face_and_misses(angle):
    q=Rotation.from_euler('y',angle,degrees=True).as_quat()
    wood=np.r_[.0207,0,0,q]
    axis=np.array([1.,0,0])
    assert bite(wood,np.zeros(3),axis)
    assert not bite(wood,np.array([0,.14,0]),axis)
    assert not bite(wood+np.r_[.015,0,0,0,0,0,0],np.zeros(3),axis)


def test_seven_screws_and_broken_and_adjacent_rows():
    def row(name,z):
        return [dict(wood=name,front=f,head=[0,y,z]) for f,y in
                [('jamb_minus',-.115),('jamb_plus',.115),('patch',-.06),('patch',.06)]]
    shots=row('strip',.06)+row('spare_strip',-.06)
    mounts=[dict(broken=False,gap=0) for _ in shots]
    assert backing_rows(shots,mounts,0)==(2,2,True)
    assert backing_rows(shots[:-1],mounts[:-1],0)==(2,2,False)
    mounts[1]['broken']=True
    assert backing_rows(shots,mounts,0)==(1,1,False)
    mounts[1]['broken']=False
    assert backing_rows(row('strip',.06)+row('spare_strip',.03),mounts,0)==(1,1,False)

def test_centre_panel_screws_support_partial_only():
    shots = [dict(wood=n, front=front, head=[0,y,z])
             for n,z in [('strip',.05),('spare_strip',-.05)]
             for front,y in [('jamb_plus',.115),('jamb_minus',-.115),('patch',0)]]
    intact = [dict(broken=False,gap=0) for _ in shots]
    assert backing_rows(shots,intact,0)==(2,2,False)
