"""Geometrically matched convex capsule candidate for submillimetre contact.

PhysX's public PCM capsule/capsule implementation treats squared centreline
distances below 1e-6 as zero and substitutes an axial contact normal. That
matches the 0.9 mm false full-depth contacts in diagnostic134. This optional
mesh representation avoids that pair routine; it does not suppress contacts.
The inscribed surface error is bounded by the sampling angles below. Native
cooking and the resulting mechanics must still be checked before promotion.
"""
import numpy as np


def capsule_points(length, radius, sides=24, cap_intervals=5):
    if not np.isfinite((length,radius)).all() or length <= 0 or radius <= 0 or sides < 12 or cap_intervals < 3:
        raise ValueError('Positive dimensions and resolved capsule mesh required')
    angles=np.arange(sides)*2*np.pi/sides
    points=[]
    for sign in (-1.,1.):
        for phi in np.linspace(0.,np.pi/2,cap_intervals+1)[:-1]:
            x=sign*(length/2+radius*np.sin(phi));r=radius*np.cos(phi)
            points.extend(np.column_stack((np.full(sides,x),r*np.cos(angles),r*np.sin(angles))))
        points.append(np.array((sign*(length/2+radius),0.,0.)))
    return np.asarray(points)


def replace_capsules(stage, paths, mesh_units_per_m=1000.):
    """Before physics only: preserve existing local transforms/material/mass."""
    from scipy.spatial import ConvexHull
    from pxr import Gf,UsdGeom,UsdPhysics,PhysxSchema,Vt
    if mesh_units_per_m not in (1.,1000.):
        raise ValueError('Only explicit metre or millimetre cooking candidates')
    records=[]
    for path in paths:
        capsule=UsdGeom.Capsule(stage.GetPrimAtPath(path))
        if not capsule or capsule.GetAxisAttr().Get()!='X':
            raise ValueError('Expected authored X-axis capsule: '+path)
        length=float(capsule.GetHeightAttr().Get());radius=float(capsule.GetRadiusAttr().Get())
        points=capsule_points(length,radius,sides=24,cap_intervals=5)
        hull=ConvexHull(points)
        prim=capsule.GetPrim();prim.SetTypeName('Mesh');mesh=UsdGeom.Mesh(prim)
        # Cook a resolved millimetre-space hull, then use the ordinary native
        # mesh scale to retain identical world dimensions. Metre-space cooking
        # in137 collapsed the tiny thread to an asymmetric tetrahedral hull.
        mesh.CreatePointsAttr(Vt.Vec3fArray((points*mesh_units_per_m).tolist()))
        mesh.AddScaleOp().Set(Gf.Vec3f(*(3*[1/mesh_units_per_m])))
        mesh.CreateFaceVertexCountsAttr([3]*len(hull.simplices))
        mesh.CreateFaceVertexIndicesAttr(hull.simplices.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
        cook=PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
        cook.CreateHullVertexLimitAttr(255);cook.CreateMinThicknessAttr(1e-6)
        records.append(dict(path=path,length_m=length,radius_m=radius,vertices=len(points),
            mesh_units_per_m=mesh_units_per_m,
            maximum_surface_deficit_m=float(radius*(1-np.cos(np.pi/24)*np.cos(np.pi/20))),
            analytic_volume_m3=float(np.pi*radius**2*length+4*np.pi*radius**3/3),
            mesh_volume_m3=float(hull.volume)))
    return records
