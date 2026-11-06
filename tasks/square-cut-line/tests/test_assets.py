# SPDX-License-Identifier: MIT
"""Check the physical USD assets, including the square's open L geometry."""
from pathlib import Path
import unittest
from pxr import Usd, UsdGeom, UsdPhysics

ASSETS = Path(__file__).resolve().parents[1] / "square_cut_line" / "assets"

class AssetTests(unittest.TestCase):
    def test_single_root_bodies_and_real_colliders(self):
        for name, expected in [("board",1),("square",2),("marker",3)]:
            stage=Usd.Stage.Open(str(ASSETS/(name+".usda")))
            root=stage.GetDefaultPrim()
            self.assertTrue(root.HasAPI(UsdPhysics.RigidBodyAPI))
            self.assertEqual(sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse()),1)
            self.assertEqual(sum(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse()),expected)
            self.assertGreater(UsdPhysics.MassAPI(root).GetMassAttr().Get(),0)
            self.assertEqual(bool(UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get()),name=="board")

    def test_marker_nib_offset_and_radius(self):
        stage=Usd.Stage.Open(str(ASSETS/"marker.usda"))
        nib=UsdGeom.Sphere(stage.GetPrimAtPath("/Marker/Nib"))
        self.assertAlmostEqual(nib.GetRadiusAttr().Get(),.002)
        transform=UsdGeom.Xformable(nib).GetLocalTransformation()
        self.assertAlmostEqual(transform.ExtractTranslation()[2],-.05)

    def test_square_has_two_distinct_box_colliders(self):
        stage=Usd.Stage.Open(str(ASSETS/"square.usda"))
        stock=UsdGeom.Xformable(stage.GetPrimAtPath("/Square/Stock")).GetLocalTransformation()
        blade=UsdGeom.Xformable(stage.GetPrimAtPath("/Square/Blade")).GetLocalTransformation()
        self.assertAlmostEqual(stock.ExtractTranslation()[0],0)
        self.assertAlmostEqual(blade.ExtractTranslation()[0],.085)
        self.assertAlmostEqual(blade.ExtractTranslation()[1],-.1075)

if __name__ == "__main__":unittest.main()
