"""Rebuild the real-trial PETG parts in millimetres; not simulation geometry.

Requires numpy and trimesh. The supplied STL files can be printed without Python.
The first 18 mm of the drain retains the task's 44 x 20 mm oral opening;
the subsequent bend is plumbing, outside the allowed spoon insertion.
"""
from pathlib import Path
import json
import numpy as np
import trimesh

OUT = Path(__file__).resolve().parent
N = 96
ANGLE = np.arange(N) * 2 * np.pi / N


def surface(rings):
    points = np.asarray(rings).reshape(-1, 3)
    faces = []
    for i in range(len(rings) - 1):
        for j in range(N):
            a, b = i*N+j, i*N+(j+1)%N
            c, d = a+N, b+N
            faces.extend([(a, b, d), (a, d, c)])
    return points, faces


def shell(outer, inner):
    p, f = surface(outer)
    q, g = surface(inner)
    off = len(p)
    f += [(a+off, c+off, b+off) for a, b, c in g]
    for ring in [0, len(outer)-1]:
        for j in range(N):
            a, b = ring*N+j, ring*N+(j+1)%N
            f.extend([(a, a+off, b+off), (a, b+off, b)])
    return trimesh.Trimesh(np.vstack([p, q]), f, process=True)


def ellipse(x, y_radius, z_radius, z=0):
    return np.c_[np.full(N, x), y_radius*np.cos(ANGLE), z+z_radius*np.sin(ANGLE)]


def face():
    rings = [ellipse(42*r*r, 24+51*r, 12+88*r, 30*r)
             for r in np.linspace(0, 1, 33)]
    return shell(rings, [r+[3, 0, 0] for r in rings])


def cover():
    # Closed rear dome, with an open annular mounting rim at x=45 mm.
    outer = [ellipse(45+75*np.sin(t), 75*np.cos(t), 100*np.cos(t), 30)
             for t in np.linspace(0, np.pi/2, 33)[:-1]]
    inner = [ellipse(45+72*np.sin(t), 72*np.cos(t), 97*np.cos(t), 30)
             for t in np.linspace(0, np.pi/2, 33)[:-1]]
    p, f = surface(outer)
    q, g = surface(inner)
    off = len(p)
    pts = np.vstack([p, q, [120, 0, 30], [117, 0, 30]])
    f += [(a+off, c+off, b+off) for a, b, c in g]
    for j in range(N):
        k=(j+1)%N
        f.extend([(j, j+off, k+off), (j, k+off, k)])
        a=(len(outer)-1)*N+j; b=(len(outer)-1)*N+k
        f.extend([(a,b,len(pts)-2),(a+off,len(pts)-1,b+off)])
    return trimesh.Trimesh(pts, f, process=True)


def drain():
    # x,z, half-width, half-height, tangent angle: straight mouth then gravity bend.
    profiles=[(0,0,22,10,0),(18,0,22,10,0),(28,-1,20,11,-.15),
              (34,-7,14,10,-.65),(38,-16,6,6,-1.15),
              (40,-28,3,3,-np.pi/2),(40,-45,3,3,-np.pi/2),
              (40,-65,3,3,-np.pi/2)]
    outer=[]; inner=[]
    for x,z,a,b,t in profiles:
        vertical=np.array([-np.sin(t),0,np.cos(t)])
        for rows,wall in [(inner,0),(outer,2)]:
            rows.append(np.array([x,0,z])+np.outer(np.cos(ANGLE),[0,a+wall,0])
                        +np.outer(np.sin(ANGLE)*(b+wall),vertical))
    return shell(outer,inner)


def main():
    nose=trimesh.creation.icosphere(subdivisions=3)
    nose.vertices=nose.vertices*[18,12,25]+[-8,0,36]
    parts={'face_shell':face(),'rear_cover':cover(),'nose':nose,'oral_drain':drain()}
    report={}
    for name,mesh in parts.items():
        # Orient adjacent triangles without optional graph-library dependencies.
        edges={}; adjacent=[[] for _ in mesh.faces]
        for i,tri in enumerate(mesh.faces):
            for a,b in zip(tri,np.roll(tri,-1)):
                key=tuple(sorted((int(a),int(b))))
                if key in edges:
                    j,direction=edges[key]; flip=(direction==(a<b))
                    adjacent[i].append((j,flip)); adjacent[j].append((i,flip))
                else: edges[key]=(i,a<b)
        orientation={}
        for start in range(len(mesh.faces)):
            if start in orientation: continue
            orientation[start]=False; todo=[start]
            while todo:
                i=todo.pop()
                for j,flip in adjacent[i]:
                    wanted=bool(orientation[i]) ^ bool(flip)
                    if j in orientation:
                        assert orientation[j]==wanted
                    else: orientation[j]=wanted; todo.append(j)
        faces=mesh.faces.copy()
        for i,flip in orientation.items():
            if flip: faces[i]=faces[i,::-1]
        mesh.faces=faces
        if mesh.volume<0: mesh.invert()
        if not mesh.is_watertight or not mesh.is_winding_consistent or mesh.volume <= 0:
            raise ValueError(f'Invalid closed print solid: {name}')
        mesh.export(OUT/(name+'.stl'))
        report[name]={'watertight':bool(mesh.is_watertight),
                      'consistent_winding':bool(mesh.is_winding_consistent),
                      'bounds_mm':mesh.bounds.tolist(),
                      'solid_volume_cm3':float(mesh.volume/1000)}
    report['limits']='CAD mesh checks only; no physical print, seal, drain or recovery trial has been performed.'
    (OUT/'mesh-checks.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
