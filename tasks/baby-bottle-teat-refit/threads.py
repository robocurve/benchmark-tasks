"""Reference prototype in mm: two interrupted right-hand helices and matching internal segments."""
import numpy as np

STARTS = (0., 180.)
LEAD_MM = 13.
ARC_DEG = 134.
START_DEG = -11.
ENTRY_Z = 67.8
LUG_Z = -10.5


def coords(r, degrees, z):
    angle = np.deg2rad(degrees)
    return (r*np.cos(angle)*.001, r*np.sin(angle)*.001, z*.001)


def profile(inner):
    if inner:
        return ((27.05,-.32),(27.4,-.71),(29.88,-.71),(29.88,.71),(27.4,.71),(27.05,.32))
    return ((24.88,-.70),(28.05,-.70),(28.65,-.31),(28.65,.31),(28.05,.70),(24.88,.70))


def slices(inner):
    for start in STARTS:
        angles = np.linspace(-4,4,3) if inner else np.linspace(START_DEG-ARC_DEG,START_DEG,25)
        for a,b in zip(angles,angles[1:]):
            vertices = []
            for deg in (a,b):
                height = (LUG_Z if inner else ENTRY_Z-LEAD_MM*START_DEG/360.)+deg*LEAD_MM/360.
                vertices.extend(coords(r,deg+start,height+dz) for r,dz in profile(inner))
            yield vertices


def add(stage, path, inner):
    from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
    for i, vertices in enumerate(slices(inner)):
        mesh = UsdGeom.Mesh.Define(stage, path+f'{i:02d}')
        n = len(vertices)//2
        faces = [list(range(n-1,-1,-1)),list(range(n,2*n))]
        faces.extend([j,(j+1)%n,(j+1)%n+n,j+n] for j in range(n))
        mesh.CreatePointsAttr(vertices)
        mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
        mesh.CreateFaceVertexIndicesAttr(sum(faces,[]))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*((.79,.84,.83) if inner else (.43,.60,.61)))])
        prim=mesh.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
        col=PhysxSchema.PhysxCollisionAPI.Apply(prim)
        col.CreateContactOffsetAttr(.00020)
        col.CreateRestOffsetAttr(0)

SKIRT = [(30.30,-14),(31,-14),(31,0),(29.70,0),(29.70,-12.2)]
NECK = [(22.8,58),(25.0,58),(25.0,68.7),(22.8,68.7)]
RIM = [(22.8,68.7),(25.,68.7),(26.4,70.6),(26.,71.),(23.2,71.),(22.8,70.6)]

def cleared_wedges(wedges,stage,path,profile,segments,color):
    if path.endswith('/skirt'):
        return wedges(stage,path,SKIRT,segments,color)
    if path.endswith('/mouth'):
        # Integral shoulder land centres the skirt; the taper lets the two parts
        # approach off-axis and does not constrain their yaw or translation.
        pilot=[(24.8,56.9),(26.9,56.9),(29.28,58.7),(29.28,60.9),(25.0,63.1),(24.8,63.1)]
        wedges(stage,path+'Pilot',pilot,64,color)
        wedges(stage,path,NECK,segments,color)
        return wedges(stage,path+'Rim',RIM,segments,color)
    return wedges(stage,path,profile,segments,color)
