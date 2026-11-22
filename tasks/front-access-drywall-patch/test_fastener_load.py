"""Mechanical regression checks for the reduced fastener model."""
import numpy as np
from scipy.spatial.transform import Rotation as R
from fastener_load import Connections,NAMES


def parts():
    return np.array([[.371,0,.985,0,0,0,1],[.371,0,.863,0,0,0,1],[.356,0,.919,0,0,0,1.]])


def test_no_screw_supplies_no_force():
    assert Connections().forces(parts(),np.zeros((3,6)),1/240) is None


def test_seating_adds_tension_at_the_installed_site_without_moving_the_parts():
    p=parts()
    p[0,:3]+=[.001,.004,.006]
    p[0,3:]=R.from_euler('xyz',[.016,-.027,.008]).as_quat()
    c=Connections();c.insert('jamb_minus','strip',np.array([.350,-.114,.985]),dict(zip(NAMES,p)))
    before=p.copy()
    force=c.forces(p,np.zeros((3,6)),1/240)
    np.testing.assert_array_equal(p,before)
    assert -12.01<=force[0,0]<0 and c.mounts[0]['gap'] < 1.e-10
    p[0,2]-=.001
    assert c.forces(p,np.zeros((3,6)),1/240)[0,2]>0


def test_internal_screw_obeys_force_and_moment_balance():
    p=parts()
    c=Connections();c.insert('patch','strip',np.array([.350,.067,.985]),dict(zip(NAMES,p)))
    v=np.zeros((3,6));v[2,2]=.008
    w=c.forces(p,v,1/240)
    assert np.linalg.norm(w[0,:3])>.0001
    np.testing.assert_allclose(w[:,:3].sum(0),0.,atol=1e-12)
    np.testing.assert_allclose((np.cross(p[:,:3],w[:,:3])+w[:,3:]).sum(0),0.,atol=1e-12)


def test_broken_fastener_cannot_carry_load():
    p=parts();c=Connections()
    c.insert('patch','strip',np.array([.350,.06,.985]),dict(zip(NAMES,p)))
    p[2,2]+=.5
    for _ in range(20):c.forces(p,np.zeros((3,6)),1/240)
    assert c.mounts[0]['broken']
    assert c.forces(p,np.zeros((3,6)),1/240) is None


def test_embedded_screw_support_has_no_axial_component_and_releases_on_slip():
    from repair_scene import embedded_shear
    n=np.array([1.,0.,0.])
    v=np.zeros(3)
    off=np.array([.0004,.001,0.])
    for depth in (0.,.0005):
        np.testing.assert_array_equal(embedded_shear(off,v,n,depth),0)
    f=embedded_shear(off,v,n,.005)
    assert f[0]==0 and f[1]<0 and np.linalg.norm(f)<=8.75
    np.testing.assert_array_equal(embedded_shear(np.array([0.,.008,0.]),v,n,.02),0)
