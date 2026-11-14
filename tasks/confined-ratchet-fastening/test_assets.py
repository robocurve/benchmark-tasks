"""Offline geometric acceptance checks; run with the documented pxr Python.

These inspect authored collision solids and composed USD, without importing
Isaac Sim or applying forces. A passing check is not dynamic validation.
"""
from pathlib import Path
import unittest

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

from ratchet_task.config import BOLT_LENGTH, HEAD_AF, HEAD_HEIGHT, BRACKET_TOP, BRACKET_SECTORS

ASSETS = Path(__file__).resolve().parent / "assets"


def open_asset(name):
    result = Usd.Stage.Open(str(ASSETS / (name + ".usda")))
    if not result:
        raise AssertionError(f"Missing asset {name}")
    return result


class ConvexSolid:
    """Independent point containment from USD cube or convex mesh face planes."""

    def __init__(self, prim, cache):
        self.name = str(prim.GetPath())
        self.inverse = np.asarray(cache.GetLocalToWorldTransform(prim).GetInverse())
        self.planes = []
        self.half_size = None
        if prim.IsA(UsdGeom.Cube):
            self.half_size = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) / 2
            return
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        centre = points.mean(axis=0)
        counts = mesh.GetFaceVertexCountsAttr().Get()
        indices = mesh.GetFaceVertexIndicesAttr().Get()
        start = 0
        for count in counts:
            face = points[list(indices[start:start + count])]
            start += count
            normal = np.cross(face[1] - face[0], face[2] - face[0])
            norm = np.linalg.norm(normal)
            if norm < 1e-14:
                continue
            normal /= norm
            if np.dot(normal, face[0] - centre) < 0:
                normal *= -1
            self.planes.append((normal, np.dot(normal, face[0])))
        if not self.planes:
            raise AssertionError(f"No valid convex faces in {self.name}")

    def contains(self, points, tolerance=1e-9):
        points = np.atleast_2d(points)
        local = (np.c_[points, np.ones(len(points))] @ self.inverse)[:, :3]
        if self.half_size is not None:
            return (np.abs(local) <= self.half_size + tolerance).all(axis=1)
        return np.logical_and.reduce([
            local @ normal <= offset + tolerance for normal, offset in self.planes])


def colliders(stage):
    cache = UsdGeom.XformCache()
    return [ConvexSolid(prim, cache) for prim in stage.Traverse()
            if prim.HasAPI(UsdPhysics.CollisionAPI)]


def occupied(solids, points):
    masks = [solid.contains(points) for solid in solids]
    return np.logical_or.reduce(masks) if masks else np.zeros(len(points), dtype=bool)


def hex_probe(af, z_values):
    angles = np.arange(6) * np.pi / 3 + np.pi / 6
    xy = af / np.sqrt(3) * np.c_[np.cos(angles), np.sin(angles)]
    # The convex body fits if its vertices fit; include its centre as a
    # regression check against accidental centre plugs or caps.
    xy = np.vstack([xy, [0, 0]])
    return np.asarray([(*p, z) for z in z_values for p in xy])


class AssetGeometryTests(unittest.TestCase):
    def test_si_units_and_colliders(self):
        for name in ("bolt", "fixture", "bolt_holder", "socket", "walls"):
            with self.subTest(asset=name):
                stage = open_asset(name)
                self.assertEqual(UsdGeom.GetStageMetersPerUnit(stage), 1)
                self.assertEqual(UsdGeom.GetStageUpAxis(stage), "Z")
                self.assertTrue(colliders(stage))

    def test_bolt_head_is_13mm_across_flats(self):
        stage = open_asset("bolt")
        points = np.asarray(UsdGeom.Mesh(stage.GetPrimAtPath("/Asset/HeadContact")).GetPointsAttr().Get())
        for angle in np.arange(3) * np.pi / 3:
            direction = np.array([np.cos(angle), np.sin(angle)])
            projected = points[:, :2] @ direction
            self.assertAlmostEqual(projected.max() - projected.min(), HEAD_AF, places=8)
        self.assertAlmostEqual(points[:, 2].max(), HEAD_HEIGHT, places=8)

    def test_scored_seating_shapes_match_authored_planes(self):
        bolt=open_asset('bolt');fixture=open_asset('fixture')
        head=bolt.GetPrimAtPath('/Asset/HeadContact')
        self.assertTrue(head.HasAPI(UsdPhysics.CollisionAPI))
        points=np.asarray(UsdGeom.Mesh(head).GetPointsAttr().Get())
        self.assertEqual(float(points[:,2].min()),0.)
        for i in range(BRACKET_SECTORS):
            prim=fixture.GetPrimAtPath(f'/Asset/Bracket/Sector{i:02}')
            self.assertTrue(prim.HasAPI(UsdPhysics.CollisionAPI))
            points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
            self.assertAlmostEqual(float(points[:,2].max()),BRACKET_TOP,places=8)

    def test_real_socket_cavity_has_clearance_and_rejects_oversize(self):
        solids = colliders(open_asset("socket"))
        # 13.38mm probes leave 0.01mm per face in the 13.4mm cavity.
        self.assertFalse(occupied(solids, hex_probe(.01338, [.0001, .003, .0053, .0089])).any())
        self.assertTrue(occupied(solids, hex_probe(.0135, [.003])).any())

    def test_socket_square_entry_remains_open(self):
        solids = colliders(open_asset("socket"))
        half = .009525 / 2  # 3/8 inch male drive
        probes = np.array([(x, y, z) for x in (-half, 0, half)
                           for y in (-half, 0, half) for z in (.0221, .025, .0279)])
        self.assertFalse(occupied(solids, probes).any())
        oversize = probes.copy()
        oversize[:, :2] *= 1.03
        self.assertTrue(occupied(solids, oversize).any())

    def test_thread_entry_stop_blocks_unstarted_full_shank(self):
        solids=colliders(open_asset('entry_stop'))
        angles=np.arange(96)*2*np.pi/96
        for radius,expected in [(.0029,False),(.00365,True)]:
            probes=np.c_[radius*np.cos(angles),radius*np.sin(angles),np.full(96,.08725)]
            self.assertEqual(bool(occupied(solids,probes).any()),expected)

    def test_fixture_bore_clears_shank_and_visual_thread(self):
        solids = colliders(open_asset("fixture"))
        angles = np.arange(96) * 2 * np.pi / 96
        for radius in (.00365, .004):
            points = np.array([(radius * np.cos(a), radius * np.sin(a), z)
                               for z in (.093 - BOLT_LENGTH, .085, .0901, .0929)
                               for a in angles])
            self.assertFalse(occupied(solids, points).any(), f"Bore obstructs radius {radius}")
        # The receiver is a genuine wall rather than a disabled collision mesh.
        self.assertTrue(occupied(solids, np.array([[.005, 0, .08]])).all())

    def test_holder_clears_initial_bolt_shank(self):
        solids = colliders(open_asset("bolt_holder"))
        angles = np.arange(96) * 2 * np.pi / 96
        points = np.array([(.00365 * np.cos(a), .00365 * np.sin(a), z)
                           for z in (.022 - BOLT_LENGTH, .014, .0219) for a in angles])
        self.assertFalse(occupied(solids, points).any())

    def test_split_vendor_visuals_have_no_physics_and_complete_materials(self):
        for filename, paths in (
            ("ratchet_visual", ["/RatchetVisual/Handle", "/RatchetVisual/Drive"]),
            ("socket_visual", ["/SocketVisual/Geometry"]),
        ):
            for path in paths:
                with self.subTest(asset=filename, subprim=path):
                    stage = Usd.Stage.CreateInMemory()
                    root = stage.DefinePrim("/Part")
                    root.GetReferences().AddReference(str(ASSETS / (filename + ".usda")), path)
                    mesh_count = 0
                    texture_count = 0
                    for prim in stage.Traverse():
                        self.assertFalse([s for s in prim.GetAppliedSchemas() if "Phys" in s])
                        if prim.IsA(UsdGeom.Mesh):
                            mesh_count += 1
                            mesh = UsdGeom.Mesh(prim)
                            self.assertGreater(len(mesh.GetPointsAttr().Get()), 100)
                            self.assertTrue(prim.GetAttribute("primvars:st").Get())
                            material = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
                            self.assertTrue(material)
                            self.assertTrue(material.GetPath().HasPrefix(root.GetPath()))
                        for attribute in prim.GetAttributes():
                            if attribute.GetTypeName() == Sdf.ValueTypeNames.Asset:
                                value = attribute.Get()
                                if value and value.path:
                                    texture_count += 1
                                    self.assertTrue(value.resolvedPath, value.path)
                                    self.assertFalse(Path(value.path).is_absolute())
                    self.assertGreater(mesh_count, 0)
                    self.assertEqual(texture_count, 3)

    def test_handle_colliders_cover_actual_grip_surface(self):
        visual=open_asset('ratchet_visual');cache=UsdGeom.XformCache()
        prim=visual.GetPrimAtPath('/RatchetVisual/Handle/mesh_2')
        pts=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(),dtype=float)
        pts=(np.c_[pts,np.ones(len(pts))]@np.asarray(cache.GetLocalToWorldTransform(prim)))[:,:3]
        pts=pts[(pts[:,0]>-.131)&(pts[:,0]<-.129)]
        self.assertGreater(len(pts),10)
        masks=[solid.contains(pts,tolerance=2e-7) for solid in colliders(open_asset('handle'))]
        self.assertTrue(np.logical_or.reduce(masks).all(),'Handle grip collider underfills the rendered surface')
        self.assertGreater(np.ptp(pts[:,1]),.027)
        self.assertGreater(np.ptp(pts[:,2]),.023)

    def test_dynamic_assets_have_one_body_and_cookable_hulls(self):
        for name in ['bolt','handle','drive']:
            stage=open_asset(name)
            self.assertEqual(sum(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in stage.Traverse()),1)
            for p in stage.Traverse():
                if p.HasAPI(UsdPhysics.CollisionAPI) and p.IsA(UsdGeom.Mesh):
                    self.assertLessEqual(len(UsdGeom.Mesh(p).GetPointsAttr().Get()),255)

    def test_walls_leave_neutral_vendor_ratchet_clear(self):
        walls = colliders(open_asset("walls"))
        stage = open_asset("ratchet_visual")
        cache = UsdGeom.XformCache()
        intersections = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh) or not str(prim.GetPath()).startswith("/RatchetVisual/Handle/"):
                continue
            points = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
            transform = np.asarray(cache.GetLocalToWorldTransform(prim))
            points = (np.c_[points, np.ones(len(points))] @ transform)[:, :3]
            # Compare around the common output-axis origin. The fixture's
            # world placement translates both walls and tool identically.
            count = int(occupied(walls, points).sum())
            if count:
                intersections.append((str(prim.GetPath()), count))
        self.assertFalse(intersections, f"Walls overlap neutral tool mesh vertices: {intersections}")

    def test_wall_box_transforms_preserve_right_angles(self):
        stage = open_asset("walls")
        cache = UsdGeom.XformCache()
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Cube):
                continue
            with self.subTest(wall=str(prim.GetPath())):
                basis = np.asarray(cache.GetLocalToWorldTransform(prim))[:3, :3]
                unit = basis / np.linalg.norm(basis, axis=1)[:, None]
                orthogonality_error = float(np.abs(unit @ unit.T - np.eye(3)).max())
                self.assertLess(orthogonality_error, 1e-6,
                    "Wall cube is sheared: nonuniform scale must follow rotation in xformOpOrder")


if __name__ == "__main__":
    unittest.main(verbosity=2)
