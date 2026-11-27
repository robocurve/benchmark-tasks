"""CPU checks of actual overlap and authored USD, without launching Isaac Sim."""
from pathlib import Path
import math
import numpy as np
import pytest
from pen_capping.retention import engagement, SEATED_X, LINER_START, LINER_END
from pen_capping.scoring import Insertion


def test_nominal_overlap_and_interference_follow_pen_and_liner_dimensions():
    state = engagement((SEATED_X, 0, 0))
    assert state['eligible']
    assert state['overlap_m'] == pytest.approx(.0039)
    assert state['compression_m'] == pytest.approx(.0002)
    assert state['tip_clearance_m'] == pytest.approx(.0042)


@pytest.mark.parametrize('relative', [(-.10, 0, 0), (-.0838, 0, 0),
    (SEATED_X, .0004, 0), (SEATED_X, 0, .0004), (-.078, 0, 0)])
def test_shallow_eccentric_and_bottom_penetrating_states_cannot_retain(relative):
    history = Insertion()
    history.update((-.14, 0, 0), 1.)
    for _ in range(8):
        assert not history.update(relative, 1.)


def test_actual_cap_frame_rejects_tilt_despite_old_five_degree_acceptance():
    angle = math.radians(1.5)
    rotation = np.array([[math.cos(angle), -math.sin(angle), 0],
                         [math.sin(angle), math.cos(angle), 0], [0, 0, 1]])
    assert not engagement((SEATED_X, 0, 0), rotation)['eligible']
    # Rotation about the common barrel axis must not affect cylindrical fit.
    roll = np.array([[1., 0, 0], [0, 0, -1], [0, 1, 0]])
    assert engagement((SEATED_X, 0, 0), roll)['eligible']


def test_seating_timer_restarts_after_lost_engagement():
    history = Insertion()
    history.update((-.14, 0, 0), 1.)
    for _ in range(3): assert not history.update((SEATED_X, 0, 0), 1.)
    assert not history.update((SEATED_X, .002, 0), 1.)
    for _ in range(3): assert not history.update((SEATED_X, 0, 0), 1.)
    assert history.update((SEATED_X, 0, 0), 1.)


@pytest.mark.parametrize('color', ['red', 'blue'])
def test_usd_contains_visible_interfering_liner_and_clear_rigid_backing(color):
    from pxr import Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(Path(__file__).parent / 'assets' / f'cap-{color}.usdc'))
    grip = [p for p in stage.Traverse() if p.GetName().startswith('LinerGrip')]
    entrance = [p for p in stage.Traverse() if p.GetName().startswith('LinerLeadIn')]
    backing = [p for p in stage.Traverse() if p.GetName().startswith('LinerBacking')]
    assert (len(grip), len(entrance), len(backing)) == (64, 64, 16)
    for p in grip + entrance:
        assert not p.HasAPI(UsdPhysics.CollisionAPI)
        assert UsdGeom.Imageable(p).ComputeVisibility() != 'invisible'
    for p in grip:
        points = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
        assert points[:, 0].min() == pytest.approx(LINER_START)
        assert points[:, 0].max() == pytest.approx(LINER_END)
        assert np.linalg.norm(points[:, 1:], axis=1).min() == pytest.approx(.0095)
    for p in backing:
        assert p.HasAPI(UsdPhysics.CollisionAPI)
        points = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get())
        # The innermost face clears the barrel by 0.9mm radially; it must
        # never be confused with simulated deformable interference contact.
        assert np.linalg.norm(points[:, 1:], axis=1).min() * math.cos(math.pi/16) == pytest.approx(.0106)
        assert UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() == 'convexHull'
    assert len([p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]) == 49
    assert not list(p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint))


@pytest.mark.parametrize('color', ['red', 'blue'])
def test_existing_barrel_collider_agrees_with_engagement_model(color):
    from pxr import Usd, UsdGeom
    from pen_capping.retention import BARREL_RADIUS, BARREL_START, BARREL_END
    stage = Usd.Stage.Open(str(Path(__file__).parent / 'assets' / f'pen-{color}.usdc'))
    barrel = UsdGeom.Cylinder(stage.GetPrimAtPath('/Asset/Barrel'))
    assert barrel.GetRadiusAttr().Get() == pytest.approx(BARREL_RADIUS)
    assert barrel.GetHeightAttr().Get() / 2 == pytest.approx(BARREL_END)
    assert -barrel.GetHeightAttr().Get() / 2 == pytest.approx(BARREL_START)
    assert barrel.GetAxisAttr().Get() == 'X'
