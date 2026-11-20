"""Visible wood and bore geometry driven by the material state, without colliders."""

import numpy as np


def wood_mesh(size, entry=None, axis=None, depth=0.):
    x, y, z = np.asarray(size)/2
    points, faces, colours = [], [], []

    def face(vertices, colour=(.67, .46, .22)):
        start = len(points)
        points.extend(vertices)
        faces.append(list(range(start, len(points))))
        colours.append(colour)

    corners = np.array([[-x,-y], [x,-y], [x,y], [-x,y]])
    for a,b in zip(corners, np.roll(corners,-1,axis=0)):
        face([[*a,z],[*b,z],[*b,-z],[*a,-z]])
    face([[*c,-z] for c in corners])
    if entry is None or depth <= 0.:
        face([[*c,z] for c in corners])
    else:
        centre = np.asarray(entry)[:2]
        angles = sorted(set(np.linspace(0,2*np.pi,65)[:-1].tolist()
                            + (np.arctan2(*(corners-centre).T[::-1]) % (2*np.pi)).tolist()))
        rays = np.c_[np.cos(angles),np.sin(angles)]
        distances = []
        for k,extent in enumerate((x,y)):
            d=np.full(len(rays),np.inf)
            np.divide(np.where(rays[:,k]>=0,extent-centre[k],-extent-centre[k]),rays[:,k],
                      out=d,where=abs(rays[:,k])>1e-10)
            distances.append(d)
        distances = np.minimum(*distances)
        outer = np.c_[centre+rays*distances[:,None], np.full(len(rays),z)]
        inner = np.c_[centre+rays*.003, np.full(len(rays),z)]
        floor = inner + np.asarray(axis)*depth
        for i in range(len(rays)):
            j = (i+1)%len(rays)
            face([outer[i],outer[j],inner[j],inner[i]])
            face([inner[i],inner[j],floor[j],floor[i]],(.30,.18,.08))
        face(floor.tolist(),(.09,.05,.025))
    return points, faces, colours


def write_mesh(mesh, data):
    from pxr import UsdGeom
    points, faces, colours = data
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr('none')
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr(colours)
    mesh.GetDisplayColorPrimvar().SetInterpolation(UsdGeom.Tokens.uniform)


def target_mesh(radius, hole_centre=None):
    rays = np.c_[np.cos(np.linspace(0,2*np.pi,65)[:-1]), np.sin(np.linspace(0,2*np.pi,65)[:-1])]
    if hole_centre is None:
        return ([[*p,0.] for p in rays*radius], [list(range(len(rays)))],[(.9,.04,.03)])
    centre = np.asarray(hole_centre)
    dot = rays@centre
    distance = -dot+np.sqrt(np.maximum(0.,dot**2+radius**2-centre@centre))
    outer = centre+rays*distance[:,None]
    inner = centre+rays*.003
    points = [[*p,0.] for p in np.r_[outer,inner]]
    n=len(rays)
    faces=[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    return points,faces,[(.9,.04,.03)]*n
