"""Triangle cloth meshes and static integrity checks; no simulation or pose control.

The geometry is original, in metres. ``rest`` describes the sewn garment before
gravity acts. During a trial only the native surface-deformable solver moves it.
"""
from dataclasses import dataclass
from collections import Counter, defaultdict
import numpy as np


@dataclass
class Mesh:
    rest: np.ndarray
    faces: np.ndarray
    markers: dict

    def audit(self):
        p, f = self.rest, self.faces
        if p.ndim != 2 or p.shape[1] != 3 or not np.isfinite(p).all():
            raise ValueError('invalid points')
        if f.ndim != 2 or f.shape[1] != 3 or f.min() < 0 or f.max() >= len(p):
            raise ValueError('invalid topology')
        area = np.linalg.norm(np.cross(p[f[:, 1]]-p[f[:, 0]],
                                      p[f[:, 2]]-p[f[:, 0]]), axis=1)/2
        if np.any(area < 1e-12):
            raise ValueError('degenerate triangle')
        edges = Counter(tuple(sorted((int(a), int(b))))
                        for tri in f for a, b in zip(tri, np.roll(tri, -1)))
        if max(edges.values()) > 2:
            raise ValueError('nonmanifold edge')
        adjacency = defaultdict(set)
        for a, b in edges:
            adjacency[a].add(b)
            adjacency[b].add(a)
        todo, visited = [0], {0}
        while todo:
            for b in adjacency[todo.pop()]:
                if b not in visited:
                    visited.add(b)
                    todo.append(b)
        if len(visited) != len(p):
            raise ValueError('disconnected or unused vertices')
        boundary = defaultdict(set)
        for (a, b), n in edges.items():
            if n == 1:
                boundary[a].add(b)
                boundary[b].add(a)
        if any(len(ns) != 2 for ns in boundary.values()):
            raise ValueError('open or branching boundary')
        unseen, loops = set(boundary), []
        while unseen:
            first = unseen.pop()
            component, pending = {first}, [first]
            while pending:
                for node in boundary[pending.pop()]:
                    if node not in component:
                        component.add(node)
                        unseen.discard(node)
                        pending.append(node)
            loops.append(len(component))
        lens = np.array([np.linalg.norm(p[a]-p[b]) for a, b in edges])
        return dict(vertices=len(p), triangles=len(f), surface_area_m2=float(area.sum()),
                    boundary_loops=sorted(loops), connected=True,
                    min_edge_m=float(lens.min()), max_edge_m=float(lens.max()),
                    min_triangle_area_m2=float(area.min()))


def tshirt(spacing=.01, layer_gap=.0016):
    """Small child's tee: 280 mm body width, 300 mm length, 400 mm sleeve span.

    Front and back are distinct triangle layers sewn along shoulders, sleeve
    undersides and torso sides. Neck, sleeve cuffs and bottom hem remain open.
    Four boundary loops distinguish a garment from a rigid T-shaped cutout.
    The rectangular pattern is a deliberately simple sewn garment, not a claim
    to reproduce a named commercial brand or calibrated cotton mechanics.
    """
    if abs(round(.01/spacing)*spacing-.01) > 1e-9:
        raise ValueError('spacing must divide 10 mm pattern dimensions')
    xs = np.arange(-.20, .20+spacing/2, spacing)
    ys = np.arange(0., .30+spacing/2, spacing)
    planar, face = [], []
    ids = {}
    def vertex(i, j):
        key = (i, j)
        if key not in ids:
            ids[key] = len(planar)
            planar.append((xs[i], ys[j]))
        return ids[key]
    for i in range(len(xs)-1):
        for j in range(len(ys)-1):
            x, y = (xs[i]+xs[i+1])/2, (ys[j]+ys[j+1])/2
            inside = (abs(x) < .14 or y > .20) and not (abs(x) < .04 and y > .27)
            if inside:
                a,b,c,d = vertex(i,j),vertex(i+1,j),vertex(i+1,j+1),vertex(i,j+1)
                face.extend(((a,b,c), (a,c,d)))
    uv = np.asarray(planar)
    # A seam boundary is the union of all perimeter edges except four openings.
    edges = Counter(tuple(sorted((a,b))) for tri in face for a,b in zip(tri, np.roll(tri,-1)))
    seams = []
    for (a,b), n in edges.items():
        if n != 1:
            continue
        x,y = (uv[a]+uv[b])/2
        opening = (y < 1e-8 or abs(x) > .1999 or
                   (abs(x) < .0401 and y > .2699))
        if not opening:
            seams.append((a,b))
    # Keep the front/back seam vertices distinct and join them by narrow strips.
    # This avoids initially coincident nodes and gives a manifold two-layer tee.
    n = len(uv)
    points = np.vstack((np.column_stack((uv, np.full(n, layer_gap/2))),
                        np.column_stack((uv, np.full(n, -layer_gap/2)))))
    faces = list(face)+[(a+n,c+n,b+n) for a,b,c in face]
    # Orient seam quads consistently with the front's directed boundary edges.
    directed = {(a,b) for tri in face for a,b in zip(tri,np.roll(tri,-1))}
    for a,b in seams:
        if (a,b) not in directed:
            a,b=b,a
        faces.extend(((b,a,a+n), (b,a+n,b+n)))
    left = np.flatnonzero((uv[:,0] <= -.1099) & (uv[:,0] >= -.1301) & (uv[:,1] < .0201))
    right = np.flatnonzero((uv[:,0] >= .1099) & (uv[:,0] <= .1301) & (uv[:,1] < .0201))
    return Mesh(points,np.asarray(faces,int),dict(left_hem=left.tolist(), right_hem=right.tolist(),
                front=list(range(n)),back=list(range(n,2*n)),
                dimensions_m=[.40,.30,layer_gap]))


def necktie(length=1.40, rows=280, columns=8):
    """Tapered 2D tie strip with pointed tips, width 35–70 mm.

    An effective one-layer shell approximates the sewn/interlined article. It
    preserves width, face orientation, in-plane strain, bending and self-contact.
    It does not pretend to simulate every individual textile/interlining layer.
    """
    if rows < 10 or columns < 4 or columns % 2:
        raise ValueError('use at least ten rows and an even number of columns >=4')
    s = np.linspace(0., length, rows+1)
    width = np.interp(s, [0.,.035,.32,.75,1.36,1.40], [0.,.07,.055,.035,.035,0.])
    vertices, ids, faces = [], {}, []
    for j in range(rows+1):
        for i in range(columns+1):
            if j in (0,rows) and i:
                ids[j,i] = ids[j,0]
            else:
                ids[j,i] = len(vertices)
                vertices.append(((i/columns-.5)*width[j], s[j], 0.))
    for j in range(rows):
        for i in range(columns):
            a,b,c,d = ids[j,i],ids[j,i+1],ids[j+1,i+1],ids[j+1,i]
            for tri in ((a,b,c),(a,c,d)):
                if len(set(tri)) == 3:
                    faces.append(tri)
    centres = [ids[j,columns//2] for j in range(rows+1)]
    return Mesh(np.asarray(vertices),np.asarray(faces,int),dict(centreline=centres,
                wide_tip=ids[0,0],narrow_tip=ids[rows,0],length_m=length,
                row_nodes=[[ids[j,i] for i in range(columns+1)] for j in range(rows+1)]))


def edge_strain(rest, current, faces):
    """Assessor diagnostic. Zero means unchanged length; it is not a tear model."""
    r,c,f = np.asarray(rest),np.asarray(current),np.asarray(faces)
    edges=np.unique(np.sort(np.vstack((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]])),axis=1),axis=0)
    initial=np.linalg.norm(r[edges[:,1]]-r[edges[:,0]],axis=1)
    return np.linalg.norm(c[edges[:,1]]-c[edges[:,0]],axis=1)/initial-1.
