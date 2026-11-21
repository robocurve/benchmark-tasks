"""Regression: a tilted round dial resting on the table is inside the workspace."""
import math
import unittest
from pxr import Gf, Usd, UsdGeom
from geometry_bounds import world_bounds


class CircularBounds(unittest.TestCase):
    def test_tilted_dial_uses_its_surface_and_detects_real_escape(self):
        stage = Usd.Stage.CreateInMemory()
        body = UsdGeom.Xform.Define(stage, '/tool')
        cylinder = UsdGeom.Cylinder.Define(stage, '/tool/dial')
        cylinder.CreateRadiusAttr(.030)
        cylinder.CreateHeightAttr(.001)
        cylinder.CreateAxisAttr('Z')
        # With this diagonal tilt, the enclosing square extends >10 mm below
        # the circular disk. The disk itself touches, but never crosses, z=0.
        z = .030*math.sin(math.radians(60)) + .0005*math.cos(math.radians(60))
        translation = body.AddTranslateOp()
        translation.Set(Gf.Vec3d(0, 0, z))
        body.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 1, 0), 60).GetQuat()))
        old = UsdGeom.BBoxCache(0, ['default']).ComputeWorldBound(body.GetPrim()).ComputeAlignedRange()
        self.assertLess(old.GetMin()[2], -.010)
        low, _ = world_bounds(body.GetPrim())
        self.assertAlmostEqual(low[2], 0., places=8)
        translation.Set(Gf.Vec3d(0, 0, z-.020))
        low, _ = world_bounds(body.GetPrim())
        self.assertLess(low[2], -.0199)


if __name__ == '__main__':
    unittest.main()
