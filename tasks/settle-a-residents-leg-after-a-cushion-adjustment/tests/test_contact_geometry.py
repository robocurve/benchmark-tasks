"""Static jaw-readout checks on an inert oval gauge using the bundled YAM meshes."""
import unittest

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

from task.station import make_model
from task.witness import Witness


class OpposedPads(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = make_model(dual=True)

    def observe(self, side, orientation, jaw_position, center_offset, mount_height, *, withdraw_one=False):
        model = self.model
        data = mj.MjData(model)
        for joint in ("joint7", "joint8"):
            data.joint(joint).qpos[0] = jaw_position
        mj.mj_kinematics(model, data)

        # Preserve the physical relative pose of the two YAM tips at this aperture.
        gripper = model.body("gripper").id
        mount = data.xmat[gripper].reshape(3, 3)
        relative = {}
        for name in ("tip_left", "tip_right"):
            tip = model.geom("arm_" + name).id
            relative[name] = (mount.T @ (data.geom_xpos[tip] - data.xpos[gripper]),
                              mount.T @ data.geom_xmat[tip].reshape(3, 3))

        # Isolate a gauge with the dimensions of an elongated scored region.
        # No controller, motor, human subject or contact force is involved.
        for geom in range(model.ngeom):
            data.geom_xpos[geom] = [100 + geom, 100, 100]
        gauge = model.geom("arm_rest").id
        center = np.array([0.1, 0.4, 0.57])
        world = Rotation.from_euler("yz", [orientation, orientation / 2]).as_matrix()
        fixture_mount = np.diag([-1., 1., -1.])
        data.geom_xpos[gauge] = center
        data.geom_xmat[gauge] = world.ravel()
        gaps = []
        for name, (offset, axes) in relative.items():
            tip = model.geom(side + name).id
            height = mount_height + (0.1 if withdraw_one and name == "tip_right" else 0.)
            data.geom_xpos[tip] = center + world @ (fixture_mount @ offset + [0., center_offset, height])
            data.geom_xmat[tip] = (world @ fixture_mount @ axes).ravel()
            gaps.append(1000 * mj.mj_geomDistance(model, data, gauge, tip, 0.25, None))
        witness = Witness(model).sample(model, data, 0., 0., arms=("arm_", "other_"))
        return witness["both_jaws_on_limb"], gaps

    def test_centered_and_offset_opposition(self):
        for side in ("arm_", "other_"):
            for orientation in (0., 0.66):
                for offset in (0., -0.03426):
                    with self.subTest(side=side, orientation=orientation, offset=offset):
                        opposed, gaps = self.observe(side, orientation, -0.04263, offset, 0.16)
                        self.assertLess(max(gaps), 1.)
                        self.assertTrue(opposed)

    def test_same_face_and_single_pad(self):
        for side in ("arm_", "other_"):
            opposed, gaps = self.observe(side, 0.66, -0.01, 0., 0.189983)
            self.assertTrue(all(0.1 < g < 0.9 for g in gaps))
            self.assertFalse(opposed)
            opposed, gaps = self.observe(side, 0.66, -0.04263, -0.03426, 0.16, withdraw_one=True)
            self.assertLess(gaps[0], 1.)
            self.assertGreater(gaps[1], 1.)
            self.assertFalse(opposed)
