"""Two flexible canvas patches and six closed tacking stitches, in SI units.

Needle holes are openings in both cloth meshes. Thread passes through them;
there are no attachments between thread and cloth. Each closed thread loop
approximates a tied tack. It is not a machine lockstitch or a running seam.
"""
from collections import Counter,defaultdict
from dataclasses import dataclass
import numpy as np

TABLE_TOP=.7475
HOLE_HALF=.0006
STITCH_Y=np.linspace(-.030,.030,6)


@dataclass
class Mesh:
    rest: np.ndarray
    faces: np.ndarray

    def audit(self):
        p,f=self.rest,self.faces
        if not np.isfinite(p).all() or p.ndim!=2 or p.shape[1]!=3 or f.ndim!=2 or f.shape[1]!=3:
            raise ValueError('Invalid cloth geometry')
        areas=np.linalg.norm(np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]]),axis=1)/2
        if np.any(areas<1e-12):raise ValueError('Degenerate cloth triangles')
        edges=Counter(tuple(sorted((int(a),int(b)))) for tri in f for a,b in zip(tri,np.roll(tri,-1)))
        if max(edges.values())>2:raise ValueError('Nonmanifold cloth mesh')
        adjacency=defaultdict(set);boundary=defaultdict(set)
        for (a,b),n in edges.items():
            adjacency[a].add(b);adjacency[b].add(a)
            if n==1:boundary[a].add(b);boundary[b].add(a)
        def components(graph):
            unseen=set(graph);result=[]
            while unseen:
                first=unseen.pop();todo=[first];seen={first}
                while todo:
                    for n in graph[todo.pop()]:
                        if n not in seen:seen.add(n);unseen.discard(n);todo.append(n)
                result.append(seen)
            return result
        if len(components(adjacency))!=1 or len(adjacency)!=len(p):raise ValueError('Disconnected fabric')
        if any(len(v)!=2 for v in boundary.values()):raise ValueError('Branching hole boundary')
        return {'vertices':len(p),'triangles':len(f),'surface_area_m2':float(areas.sum()),
                'boundary_loops':len(components(boundary)),'connected':True}


def refine_hole_neighborhood(mesh,hole_half,max_edge):
    """Conforming midpoint refinement; preserve the exact physical surface.

    Refine triangles touching a band two target edge lengths around each
    existing needle opening. Splitting a shared edge splits both incident
    triangles, so no T-junction, additional hole, attachment or stiff rim is
    introduced. Rest shape, material area and all hole boundaries are unchanged.
    """
    if not np.isfinite(max_edge) or not .0001 <= max_edge <= .001:
        raise ValueError('Diagnostic hole mesh spacing must be 0.1 to 1.0 mm')
    points=mesh.rest.copy();faces=mesh.faces.copy()
    centres=np.array([(x,y) for x in (.295,.305) for y in STITCH_Y])
    def near(points):
        d=np.maximum(np.abs(points[:,:2,None]-centres.T[None])-hole_half,0.)
        return (np.linalg.norm(d,axis=1)<=2*max_edge+1e-12).any(1)
    for iteration in range(24):
        affected=near(points)[faces].any(1)|near(points[faces].mean(1))
        selected=set()
        for face in faces[affected]:
            for a,b in zip(face,np.roll(face,-1)):
                if np.linalg.norm(points[a]-points[b])>max_edge*(1+1e-10):
                    selected.add(tuple(sorted((int(a),int(b)))))
        if not selected:
            result=Mesh(points,faces);result.audit();return result
        vertices=points.tolist();mid={}
        for edge in sorted(selected):
            mid[edge]=len(vertices);vertices.append(((points[edge[0]]+points[edge[1]])/2).tolist())
        refined=[]
        for face in faces:
            edge_mid=[mid.get(tuple(sorted((int(face[i]),int(face[(i+1)%3]))))) for i in range(3)]
            count=sum(v is not None for v in edge_mid)
            if count==0:
                refined.append(face.tolist())
            elif count==1:
                i=next(i for i,v in enumerate(edge_mid) if v is not None)
                a,b,c=(int(face[(i+j)%3]) for j in range(3));m=edge_mid[i]
                refined.extend(((a,m,c),(m,b,c)))
            elif count==2:
                i=edge_mid.index(None)
                a,b,c=(int(face[(i+j)%3]) for j in range(3))
                mbc,mca=edge_mid[(i+1)%3],edge_mid[(i+2)%3]
                refined.append((c,mca,mbc))
                if np.linalg.norm(np.asarray(vertices[a])-vertices[mbc])<=np.linalg.norm(np.asarray(vertices[b])-vertices[mca]):
                    refined.extend(((a,b,mbc),(a,mbc,mca)))
                else:
                    refined.extend(((a,b,mca),(b,mbc,mca)))
            else:
                a,b,c=map(int,face);mab,mbc,mca=edge_mid
                refined.extend(((a,mab,mca),(mab,b,mbc),(mca,mbc,c),(mab,mbc,mca)))
        points=np.asarray(vertices);faces=np.asarray(refined,int)
    raise RuntimeError('Hole mesh refinement did not converge')


def patch(left=True,*,hole_half=HOLE_HALF,hole_mesh_spacing=None):
    xmin,xmax=(.235,.310) if left else (.290,.365)
    xs=np.unique(np.round(np.r_[np.linspace(xmin,xmax,26),
                                 .295-hole_half,.295+hole_half,.305-hole_half,.305+hole_half],9))
    ys=np.unique(np.round(np.r_[np.linspace(-.040,.040,33),STITCH_Y-hole_half,STITCH_Y+hole_half],9))
    vertices=[];faces=[];ids={}
    def vertex(i,j):
        if (i,j) not in ids:
            ids[i,j]=len(vertices)
            vertices.append((xs[i],ys[j],TABLE_TOP+(.004 if left else .0032)))
        return ids[i,j]
    for i in range(len(xs)-1):
        for j in range(len(ys)-1):
            x,y=(xs[i]+xs[i+1])/2,(ys[j]+ys[j+1])/2
            hole=any(abs(x-hx)<hole_half and abs(y-hy)<hole_half for hx in (.295,.305) for hy in STITCH_Y)
            if hole:continue
            a,b,c,d=vertex(i,j),vertex(i+1,j),vertex(i+1,j+1),vertex(i,j+1)
            faces.extend(((a,b,c),(a,c,d)))
    mesh=Mesh(np.asarray(vertices),np.asarray(faces,int));mesh.audit()
    return mesh if hole_mesh_spacing is None else refine_hole_neighborhood(mesh,hole_half,hole_mesh_spacing)


def stitch_loop(y,spacing=.001,*,clear_capsules=False,long_side_links=10):
    """Closed 10 x 2.4 mm centreline; spans both layers through real holes."""
    import os
    bridge=.0078 if os.environ.get('ROBOCURVE_SEAM_LOOSE_TACKING')=='1' else .0048
    points=np.array([[.295,y,TABLE_TOP+bridge],[.305,y,TABLE_TOP+bridge],
                     [.305,y,TABLE_TOP+.0024],[.295,y,TABLE_TOP+.0024]])
    nodes=[]
    if clear_capsules and spacing!=.001:raise ValueError('Clear-capsule sampling has fixed physical segment counts')
    if long_side_links not in (4,10) or (long_side_links!=10 and not clear_capsules):
        raise ValueError('Reduced thread discretization requires clear capsules')
    for edge,(a,b) in enumerate(zip(points,np.roll(points,-1,axis=0))):
        # At0.8mm diameter, three0.8mm links on the short side make
        # non-neighbouring caps exactly tangent. Two1.2mm links retain the
        # identical centreline while giving ordinary self-contact clearance.
        n=(long_side_links,2,long_side_links,2)[edge] if clear_capsules else max(1,int(np.ceil(np.linalg.norm(b-a)/spacing)))
        nodes.extend(a+(b-a)*s/n for s in range(n))
    return np.asarray(nodes)


def audit_scene():
    cloth=[patch(True),patch(False)]
    result={'cloth':[m.audit() for m in cloth],'stitches':[]}
    for y in STITCH_Y:
        nodes=stitch_loop(y)
        lengths=np.linalg.norm(np.roll(nodes,-1,axis=0)-nodes,axis=1)
        result['stitches'].append({'nodes':len(nodes),'length_m':float(lengths.sum()),
                                  'minimum_link_m':float(lengths.min()),'closed':True})
    # Both layers must have one outer boundary and twelve real needle holes.
    if any(row['boundary_loops']!=13 for row in result['cloth']):
        raise ValueError('The thread geometry requires twelve holes per patch')
    return result
