"""Build the neutral-zone tray: a stainless-steel instrument tray, 10 x 6 x 3/4 in (25.4 x 15.2 x 1.9 cm), its 0.8 mm
sheet pressed into a shallow pan with sloped walls, rounded corners and a rolled rim. Writes assets/tray.usda (the
visual shell, stainless steel, and a collider of convex pieces: the floor as one slab and the wall as 3 mm slabs; a
pan can't be one convex hull), assets/tray_rim.npy (the rim's outline seen from above, for the scoring) and
assets/tray_mesh.npz (for the offline previews).

Frame: the floor's centre at the origin, its bottom on z = 0, its long side along x.     python make_tray.py
"""
import pathlib

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
RIM = (0.254, 0.152)        # outside of the rim (m)
FLOOR = (0.234, 0.132)      # the floor: the walls slope in 1 cm over the depth
DEPTH = 0.019               # rim height above the bottom
CORNER = 0.018              # corner radius at the rim (the floor's is 1 cm smaller)
SHEET = 0.0008
COLL_FLOOR = 0.002          # collider floor thickness
N_CORNER = 8                # samples per rounded corner


def outline(size, r, n=N_CORNER):
    """A rounded rectangle ``size`` (x, y) with corner radius ``r``, seen from above, counter-clockwise; the same
    number of points for every size, so that outlines can be lofted."""
    hx, hy = size[0] / 2 - r, size[1] / 2 - r
    pts = []
    for cx, cy, a0 in ((hx, hy, 0.0), (-hx, hy, 90.0), (-hx, -hy, 180.0), (hx, -hy, 270.0)):
        for a in np.radians(a0 + np.linspace(0.0, 90.0, n)):
            pts.append((cx + r * np.cos(a), cy + r * np.sin(a)))
    return np.array(pts)


def _grow(size, d):
    return (size[0] + 2 * d, size[1] + 2 * d)


def shell():
    """Triangles of the sheet: the inside (floor and wall), the outside (bottom and wall) and the rolled rim."""
    V, F = [], []
    fl, rim = outline(FLOOR, CORNER - 0.01), outline(RIM, CORNER)
    fl_o, rim_o = outline(_grow(FLOOR, SHEET), CORNER - 0.01 + SHEET), outline(_grow(RIM, SHEET), CORNER + SHEET)
    n = len(fl)

    def ring(xy, z):
        V.extend([(x, y, z) for x, y in xy])
        return len(V) - len(xy)

    def strip(a, b, flip=False):
        for i in range(n):
            j = (i + 1) % n
            q = [(a + i, a + j, b + j), (a + i, b + j, b + i)]
            F.extend([t[::-1] for t in q] if flip else q)

    def cap(xy, z, up):                       # the rounded rectangle is convex: a fan from its centre
        base = ring(xy, z)
        V.append((0.0, 0.0, z))
        c = len(V) - 1
        for i in range(n):
            t = (c, base + i, base + (i + 1) % n)
            F.append(t if up else t[::-1])

    strip(ring(fl, SHEET), ring(rim, DEPTH), flip=True)          # inner wall
    cap(fl, SHEET, up=True)                                      # inner floor
    strip(ring(fl_o, 0.0), ring(rim_o, DEPTH - SHEET))           # outer wall
    cap(fl_o, 0.0, up=False)                                     # bottom
    r1 = ring(rim, DEPTH)                                        # rolled rim: a 2.5 mm bead
    r2 = ring(outline(_grow(RIM, 0.002), CORNER + 0.002), DEPTH + 0.0012)
    r3 = ring(outline(_grow(RIM, 0.0028), CORNER + 0.0028), DEPTH - 0.0015)
    r4 = ring(rim_o, DEPTH - SHEET)
    strip(r1, r2, flip=True)
    strip(r2, r3, flip=True)
    strip(r3, r4, flip=True)
    return np.array(V), np.array(F)


def collider_pieces():
    fl, rim = outline(FLOOR, CORNER - 0.01), outline(RIM, CORNER)
    fl_o, rim_o = outline(_grow(FLOOR, 0.003), CORNER - 0.007), outline(_grow(RIM, 0.003), CORNER + 0.003)
    pieces = [np.vstack([np.c_[fl, np.zeros(len(fl))], np.c_[fl, np.full(len(fl), COLL_FLOOR)]])]   # the floor
    n = len(fl)
    for i in range(n):
        j = (i + 1) % n
        pieces.append(np.array([np.r_[fl[i], 0.0], np.r_[fl[j], 0.0], np.r_[rim[i], DEPTH], np.r_[rim[j], DEPTH],
                                np.r_[fl_o[i], 0.0], np.r_[fl_o[j], 0.0], np.r_[rim_o[i], DEPTH], np.r_[rim_o[j], DEPTH]]))
    return pieces


if __name__ == '__main__':
    import trimesh
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
    v, f = shell()
    out = HERE / 'assets'
    st = Usd.Stage.CreateNew(str(out / 'tray.usda'))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = UsdGeom.Xform.Define(st, '/Tray')
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mat = UsdShade.Material.Define(st, '/Tray/Looks/stainless')
    sh = UsdShade.Shader.Define(st, '/Tray/Looks/stainless/surface')
    sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.66, 0.67, 0.69))
    sh.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(0.6)
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.4)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    mesh = UsdGeom.Mesh.Define(st, '/Tray/shell')
    mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(v.astype('f4')))
    mesh.CreateFaceVertexCountsAttr([3] * len(f))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(f.astype('i4').ravel()))
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateExtentAttr([Gf.Vec3f(*v.min(0).tolist()), Gf.Vec3f(*v.max(0).tolist())])
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('displayColor', Sdf.ValueTypeNames.Color3fArray).Set([Gf.Vec3f(0.66, 0.67, 0.69)])
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    UsdGeom.Scope.Define(st, '/Tray/collider')
    pieces = collider_pieces()
    for k, p in enumerate(pieces):
        hull = trimesh.convex.convex_hull(p)
        cm = UsdGeom.Mesh.Define(st, f'/Tray/collider/piece_{k:02d}')
        cm.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(hull.vertices.astype('f4')))
        cm.CreateFaceVertexCountsAttr([3] * len(hull.faces))
        cm.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(hull.faces.astype('i4').ravel()))
        cm.CreatePurposeAttr(UsdGeom.Tokens.guide)        # not rendered
        UsdPhysics.CollisionAPI.Apply(cm.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(cm.GetPrim()).CreateApproximationAttr('convexHull')
    st.GetRootLayer().Save()
    np.save(out / 'tray_rim.npy', outline(RIM, CORNER).astype('f4'))
    np.savez(out / 'tray_mesh.npz', v=v.astype('f4'), f=f.astype('i4'))
    print('shell', len(v), 'vertices', len(f), 'triangles; size cm', ((v.max(0) - v.min(0)) * 100).round(1),
          '| collider pieces', len(pieces))
