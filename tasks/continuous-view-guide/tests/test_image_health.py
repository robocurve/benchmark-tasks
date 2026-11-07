"""Reject lost rendering without penalizing rest or legitimately blank wrists."""
import numpy as np
import pytest
from guide_task.image_health import ImageMonitor


def images(blank=False):
    rgb=np.empty((176,320,3),dtype=np.uint8)
    rgb[:]=[131,139,141]
    if not blank:rgb[40:150,60:250]=[220,180,100]
    return dict(top_cam=rgb,left_cam=np.zeros_like(rgb),right_cam=np.zeros_like(rgb))


def test_stationary_scene_and_blank_wrist_are_valid():
    monitor=ImageMonitor();frame=images();before=frame['top_cam'].copy()
    for _ in range(60):monitor.update(frame)
    assert monitor.report()['valid'] and np.array_equal(before,frame['top_cam'])


def test_colored_grey_render_latches_invalidity():
    monitor=ImageMonitor();monitor.update(images())
    monitor.update(images(True));monitor.update(images(True))
    with pytest.raises(RuntimeError):monitor.update(images(True))
    with pytest.raises(RuntimeError):monitor.update(images())
    assert not monitor.report()['valid']


def test_transient_blank_does_not_accumulate():
    monitor=ImageMonitor()
    for _ in range(5):
        monitor.update(images(True));monitor.update(images(True));monitor.update(images())
    assert monitor.report()['valid']


@pytest.mark.parametrize('bad', [None,np.zeros((176,320,3),dtype=np.float32),np.zeros((16,16,3),dtype=np.uint8)])
def test_invalid_camera_format(bad):
    frame=images();frame['right_cam']=bad
    with pytest.raises(RuntimeError):ImageMonitor().update(frame)
