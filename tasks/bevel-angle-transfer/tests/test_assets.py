# SPDX-License-Identifier: MIT
"""Validate the gauge topology and geometry without launching Isaac Sim."""

from pathlib import Path
import unittest

from pxr import Gf, Usd, UsdGeom, UsdPhysics

from bevel_angle_transfer import task_spec as spec


ASSETS = Path(spec.GAUGE_ASSET).parent


class AssetTests(unittest.TestCase):
    def test_gauge_is_one_free_articulation_with_two_colliding_links(self):
        stage = Usd.Stage.Open(spec.GAUGE_ASSET)
        stock = stage.GetPrimAtPath("/Gauge/stock")
        blade = stage.GetPrimAtPath("/Gauge/blade")
        self.assertTrue(stock.HasAPI(UsdPhysics.ArticulationRootAPI))
        bodies = [prim for prim in stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        self.assertEqual(bodies, [stock, blade])
        for body in bodies:
            self.assertFalse(UsdPhysics.RigidBodyAPI(body).GetKinematicEnabledAttr().Get())
            self.assertGreater(UsdPhysics.MassAPI(body).GetMassAttr().Get(), 0)
            self.assertTrue(stage.GetPrimAtPath(body.GetPath().AppendChild("Body")).HasAPI(UsdPhysics.CollisionAPI))

        joints = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Joint)]
        self.assertEqual(len(joints), 1)
        joint = UsdPhysics.RevoluteJoint(joints[0])
        self.assertTrue(joint)
        self.assertEqual(joint.GetAxisAttr().Get(), "Z")
        self.assertEqual(joint.GetBody0Rel().GetTargets(), [stock.GetPath()])
        self.assertEqual(joint.GetBody1Rel().GetTargets(), [blade.GetPath()])
        self.assertFalse(joint.GetCollisionEnabledAttr().Get())
        self.assertEqual(joint.GetLowerLimitAttr().Get(), -45)
        self.assertEqual(joint.GetUpperLimitAttr().Get(), 45)
        drive = UsdPhysics.DriveAPI.Get(joints[0], "angular")
        self.assertTrue(drive)
        self.assertEqual(drive.GetTypeAttr().Get(), "force")
        self.assertEqual(drive.GetStiffnessAttr().Get(), 0)
        self.assertEqual(drive.GetTargetVelocityAttr().Get(), 0)
        self.assertFalse(drive.GetTargetPositionAttr().HasAuthoredValueOpinion())
        self.assertAlmostEqual(drive.GetDampingAttr().Get(), spec.GAUGE_DAMPER_PER_DEGREE)
        self.assertAlmostEqual(drive.GetMaxForceAttr().Get(), spec.GAUGE_DAMPER_MAX_TORQUE)
        for attr, expected in (
            ("staticFrictionEffort", spec.GAUGE_STATIC_FRICTION),
            ("dynamicFrictionEffort", spec.GAUGE_DYNAMIC_FRICTION),
            ("viscousFrictionCoefficient", spec.GAUGE_VISCOUS_FRICTION),
        ):
            self.assertAlmostEqual(joints[0].GetAttribute("physxJointAxis:angular:" + attr).Get(), expected)
        self.assertAlmostEqual(
            (joint.GetLocalPos0Attr().Get() - Gf.Vec3f(*spec.GAUGE_PIVOT_LOCAL)).GetLength(), 0,
        )

    def test_blade_does_not_tip_stock_at_reset(self):
        stage = Usd.Stage.Open(spec.GAUGE_ASSET)
        stock = stage.GetPrimAtPath("/Gauge/stock")
        blade = stage.GetPrimAtPath("/Gauge/blade")
        stock_mass, blade_mass = (UsdPhysics.MassAPI(body).GetMassAttr().Get() for body in (stock, blade))
        stock_size = stock.GetChild("Body").GetAttribute("xformOp:scale").Get()
        # The farther of the two blade collider centres bounds the blade COM.
        # Even at that bound, the assembly COM lies within the stock footprint.
        tab_x = blade.GetChild("GraspTab").GetAttribute("xformOp:translate").Get()[0]
        combined_x_bound = tab_x * blade_mass / (stock_mass + blade_mass)
        self.assertLess(combined_x_bound, stock_size[0] / 2)

    def test_grasp_tab_clears_working_edge(self):
        stage = Usd.Stage.Open(spec.GAUGE_ASSET)
        tab = stage.GetPrimAtPath("/Gauge/blade/GraspTab")
        self.assertTrue(tab.HasAPI(UsdPhysics.CollisionAPI))
        transform = UsdGeom.Xformable(tab).GetLocalTransformation()
        self.assertEqual(tuple(transform.ExtractTranslation()), spec.BLADE_GRASP_LOCAL)
        self.assertGreater(
            transform.Transform(Gf.Vec3d(0, -.5, 0))[1], spec.BLADE_LOCAL_EDGE[0][1],
        )

    def test_marker_nib_and_board_size_match_contact_configuration(self):
        marker = Usd.Stage.Open(str(ASSETS / "marker.usda"))
        nib = UsdGeom.Sphere(marker.GetPrimAtPath("/Marker/Nib"))
        self.assertTrue(nib.GetPrim().HasAPI(UsdPhysics.CollisionAPI))
        self.assertAlmostEqual(nib.GetRadiusAttr().Get(), spec.MARKER_NIB_RADIUS)
        self.assertEqual(
            tuple(UsdGeom.Xformable(nib).GetLocalTransformation().ExtractTranslation()), spec.MARKER_NIB_LOCAL,
        )
        board = Usd.Stage.Open(str(ASSETS / "board.usda"))
        self.assertTrue(UsdPhysics.RigidBodyAPI(board.GetDefaultPrim()).GetKinematicEnabledAttr().Get())
        self.assertEqual(tuple(board.GetPrimAtPath("/Board/Wood").GetAttribute("xformOp:scale").Get()), spec.BOARD_SIZE)
