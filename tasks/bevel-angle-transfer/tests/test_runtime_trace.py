"""Test the runtime measurement contract without launching Isaac Sim."""
from types import SimpleNamespace

import numpy as np
import pytest

from bevel_angle_transfer.runtime import BevelAngleTransferEmbodiment, _review_resolution
from yam_arena.constants import FINGER_OPEN


class Tensor:
    def __init__(self, values):
        self.values = np.asarray(values)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.values


def test_runtime_exports_measured_jaws_for_both_arms():
    runtime = BevelAngleTransferEmbodiment.__new__(BevelAngleTransferEmbodiment)
    identity = [0., 0., 0., 0., 0., 0., 1.]
    runtime._root_pose = lambda name: np.asarray(identity)
    runtime._gauge = SimpleNamespace(data=SimpleNamespace(
        body_link_pose_w=Tensor([[identity, identity]]), joint_pos=Tensor([[.1]]), joint_vel=Tensor([[0.]])))
    runtime._stock_id, runtime._blade_id, runtime._hinge_id = 0, 1, 0
    runtime._right_ids = runtime._left_ids = [0, 1, 2]
    runtime._ids = {name: (list(range(6)), [6, 7]) for name in ("left_robot", "right_robot")}
    for side, opening in (("left", .2), ("right", .9)):
        robot = SimpleNamespace(data=SimpleNamespace(
            body_link_pose_w=Tensor([[identity, identity, identity]]),
            joint_pos=Tensor([[0.] * 6 + [FINGER_OPEN * opening] * 2])))
        setattr(runtime, "_" + side, robot)
    raw = runtime._poses()
    assert raw["left_gripper_open"] is False
    assert raw["right_gripper_open"] is True
    assert raw["left_gripper_opening"] == pytest.approx(.2)
    assert raw["right_gripper_opening"] == pytest.approx(.9)
    assert len(raw["left_finger_poses"]) == len(raw["right_finger_poses"]) == 2
    runtime._left.data.joint_pos.values[0, 6] = np.nan
    with pytest.raises(RuntimeError, match="Nonfinite left gripper"):
        runtime._poses()


def test_physical_ink_contact_point_does_not_require_force_telemetry():
    runtime = BevelAngleTransferEmbodiment.__new__(BevelAngleTransferEmbodiment)
    points = Tensor([[[[.25, -.03, .7635]]]])
    runtime._sensors = {"marker_board": SimpleNamespace(data=SimpleNamespace(contact_pos_w=points))}
    force, point = runtime._pair("marker_board", required=True)
    assert force == [None] * 3
    assert point == [.25, -.03, .7635]


def test_missing_required_contact_channel_fails_explicitly():
    runtime = BevelAngleTransferEmbodiment.__new__(BevelAngleTransferEmbodiment)
    runtime._sensors = {"marker_board": SimpleNamespace(data=SimpleNamespace())}
    assert runtime._pair("marker_board") == ([None] * 3, [None] * 3)
    assert runtime._pair("unused_force_diagnostic") == ([None] * 3, [None] * 3)
    with pytest.raises(RuntimeError, match="contact point unavailable"):
        runtime._pair("marker_board", required=True)


def test_native_review_resize_rebuilds_camera_and_keeps_wrist_configuration():
    wrist = object()
    requested = []
    def make_camera(height, width, overview):
        requested.append((height, width, overview))
        return SimpleNamespace(top_cam=SimpleNamespace(height=height, width=width, calibrated=True))
    camera = SimpleNamespace(camera_config=SimpleNamespace(top_cam=SimpleNamespace(height=352, width=640),
                                                         left_cam=wrist, right_cam=wrist),
                             _camera_cfg=make_camera)
    assert _review_resolution(camera) is camera
    assert requested == [(704, 1280, False)]
    assert camera.camera_config.top_cam.calibrated
    assert camera.camera_config.left_cam is camera.camera_config.right_cam is wrist
