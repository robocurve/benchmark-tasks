"""World bounds of actual primitive surfaces, including rotated circular dials."""
import itertools
import numpy as np
from pxr import Gf, Usd, UsdGeom


def world_bounds(root):
    lows, highs = [], []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Gprim):
            continue
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        if prim.IsA(UsdGeom.Cylinder):
            cylinder = UsdGeom.Cylinder(prim)
            axis = 'XYZ'.index(cylinder.GetAxisAttr().Get())
            radial = [i for i in range(3) if i != axis]
            center = np.asarray(matrix.Transform(Gf.Vec3d(0.)))
            basis = np.asarray([np.asarray(matrix.Transform(Gf.Vec3d(*v))) - center
                                for v in np.eye(3)])
            extent = (cylinder.GetRadiusAttr().Get()
                      * np.sqrt(basis[radial[0]]**2 + basis[radial[1]]**2)
                      + cylinder.GetHeightAttr().Get()/2 * np.abs(basis[axis]))
            low, high = center - extent, center + extent
        else:
            if prim.IsA(UsdGeom.Cube):
                half = UsdGeom.Cube(prim).GetSizeAttr().Get()/2
                local = itertools.product([-half, half], repeat=3)
            elif prim.IsA(UsdGeom.Mesh):
                local = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            else:
                raise ValueError(f'Unsupported workspace geometry: {prim.GetTypeName()}')
            points = np.asarray([matrix.Transform(Gf.Vec3d(*v)) for v in local])
            low, high = points.min(0), points.max(0)
        lows.append(low)
        highs.append(high)
    if not lows:
        raise ValueError('Workspace object has no physical geometry')
    return np.min(lows, axis=0), np.max(highs, axis=0)
