"""Check contact, fixed attachment, and four spatial tool axes without Isaac Sim."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parent / 'calibration_tool' / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def scorer():
    return load('scoring').CalibrationScore(control_hz=20)


def pose(azimuth=0, tilt=25, offset=(0., 0., 0.)):
    az, angle = np.radians([azimuth, tilt])
    axis = np.array([-np.sin(az), np.cos(az), 0.])
    cross = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
    rotation = (np.eye(3) + np.sin(angle)*cross + (1-np.cos(angle))*cross@cross) @ np.diag([1., -1., -1.])
    target = np.array([.27, .22, .7985]) + offset
    origin = target - rotation @ np.array([0., 0., .088])
    return origin, rotation


def hold(scorer, azimuth=0, steps=20, offset=(0.,0.,0.), contact=True, tool_shift=(0.,0.,0.), tilt=25):
    origin, rotation = pose(azimuth, tilt, offset)
    for _ in range(steps):
        score = scorer.update(origin, rotation, origin + tool_shift, rotation, 0.2 if contact else 0.)
    return score


def test_four_contact_poses_reach_full_credit(scorer):
    for index, az in enumerate((90, 150, 210, 270), 1):
        assert hold(scorer, az) == index/4
    assert scorer.success


@pytest.mark.parametrize('offset', [(0.,0.,.006), (0.,0.,.001), (.002,0.,0.)])
def test_hovering_or_missing_the_reference_cannot_score(scorer, offset):
    for az in (90,150,210,270):
        assert hold(scorer, az, offset=offset) == 0.


def test_geometric_alignment_without_physical_contact_cannot_score(scorer):
    assert hold(scorer, contact=False, steps=100) == 0.


def test_a_loose_tool_cannot_score(scorer):
    for az in (90,150,210,270):
        assert hold(scorer, az, tool_shift=(.002,0.,0.)) == 0.


def test_rotation_at_the_mount_cannot_score(scorer):
    pos, tool_rot = pose()
    _, grip_rot = pose(azimuth=20)
    for _ in range(40):
        assert scorer.update(pos,tool_rot,pos,grip_rot,.2) == 0.


def test_tip_drift_restarts_the_hold(scorer):
    hold(scorer,steps=19)
    assert hold(scorer,steps=1,offset=(.0006,0.,0.)) == 0.
    assert hold(scorer,steps=18,offset=(.0006,0.,0.)) == 0.
    assert hold(scorer,steps=1,offset=(.0006,0.,0.)) == .25


def test_contact_must_hold_for_twenty_control_periods(scorer):
    assert hold(scorer, steps=19) == 0.
    assert hold(scorer, steps=1) == .25


def test_losing_contact_restarts_the_hold(scorer):
    hold(scorer, steps=19)
    hold(scorer, steps=1, contact=False)
    assert hold(scorer, steps=19) == 0.
    assert hold(scorer, steps=1) == .25


def test_one_pose_cannot_earn_repeated_credit(scorer):
    assert hold(scorer, steps=100) == .25
    assert not scorer.success


def test_pose_separation_uses_spatial_axes(scorer):
    hold(scorer, azimuth=0)
    assert hold(scorer, azimuth=10) == .25
    assert hold(scorer, azimuth=90) == .5


@pytest.mark.parametrize('tilt',[0,9,36,60])
def test_pose_tilt_limits(scorer, tilt):
    assert hold(scorer, tilt=tilt) == 0.


def test_reset_removes_old_pose_credits(scorer):
    hold(scorer)
    scorer.reset()
    assert hold(scorer, steps=19) == 0.
