"""Build fixed, locally refined paper meshes; development-only SciPy dependency.

The deployed task loads checked NumPy assets and does not require SciPy. Local
puncture rings avoid the long, thin triangles created by extending every hole
feature across the entire A5 sheet. Delaunay output is accepted only if the
actual rectangular boundaries, area and triangle-quality checks all pass.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from paper import PaperMesh


def make(size,centres,hole_size,spacing=.008,profile="baseline"):
    if profile not in ("baseline","coarse","clearance-candidate"):raise ValueError("Unknown mesh profile")
    rings=(0.,.00015,.0004,.0009,.0018,.0036,.0072) if profile=="baseline" else (0.,.0004,.0009,.0018,.0036,.0072)
    minimum_pitch=.00015 if profile=="baseline" else .00035
    from scipy.spatial import Delaunay
    lo=-np.array(size)/2;hi=-lo;half=np.array(hole_size)/2;centres=np.array(centres)
    points=[]
    for centre in centres:
        for distance in rings:
            extent=half+distance;pitch=max(minimum_pitch,distance*.6)
            for axis in range(2):
                other=1-axis
                values=np.linspace(-extent[axis],extent[axis],max(2,int(np.ceil(2*extent[axis]/pitch))+1))
                for sign in (-1,1):
                    row=np.empty((len(values),2));row[:,axis]=centre[axis]+values
                    row[:,other]=centre[other]+sign*extent[other]
                    points.extend(np.clip(row,lo,hi))
    grid=np.array([(x,y) for x in np.linspace(lo[0],hi[0],int(np.ceil(size[0]/spacing))+1)
                   for y in np.linspace(lo[1],hi[1],int(np.ceil(size[1]/spacing))+1)])
    distance=np.min(np.max(np.abs(grid[:,None,:]-centres[None,:,:])-half,axis=2),axis=1)
    points.extend(grid[distance>.0072])
    points=np.unique(np.round(points,10),axis=0)
    kept=[]
    for point in points:
        radius=min(np.max(np.abs(point-centre)-half) for centre in centres)
        threshold=max(.000025,min(.0007,radius*.09))
        if not kept or np.min(np.linalg.norm(np.asarray(kept)-point,axis=1))>=threshold:
            kept.append(point)
    points=np.asarray(kept)
    # Different puncture sizes can place an outer regular-grid point close to
    # a refinement ring. Refine bad triangles geometrically, keeping both the
    # exact puncture boundary and the original quality bound.
    for refinement in range(12 if profile=='clearance-candidate' else 1):
        faces=Delaunay(points).simplices
        center=points[faces].mean(1)
        outside=np.all(np.any(np.abs(center[:,None,:]-centres[None,:,:])>=half-1e-12,axis=2),axis=1)
        faces=faces[outside]
        if profile!='clearance-candidate':break
        tri=points[faces];angles=[]
        lengths=np.linalg.norm(np.roll(tri,-1,axis=1)-tri,axis=2)
        for j in range(3):
            a=tri[:,(j+1)%3]-tri[:,j];b=tri[:,(j+2)%3]-tri[:,j]
            angles.append(np.arccos(np.clip(np.sum(a*b,axis=1)/np.linalg.norm(a,axis=1)/np.linalg.norm(b,axis=1),-1.,1.)))
        bad=(np.min(angles,axis=0)<np.deg2rad(15.))|(lengths.max(1)/lengths.min(1)>4.)
        if not np.any(bad):break
        extra=[]
        for a,b,c in tri[bad]:
            candidate=np.linalg.solve(2*np.array((b-a,c-a)),np.array((b@b-a@a,c@c-a@a)))
            if np.any(candidate<lo) or np.any(candidate>hi):continue
            if np.any(np.all(np.abs(candidate-centres)<half,axis=1)):continue
            if np.min(np.linalg.norm(points-candidate,axis=1))<.000025:continue
            extra.append(candidate)
        if not extra:break
        points=np.unique(np.round(np.vstack((points,extra)),10),axis=0)
    holes=[[c[0]-half[0],c[0]+half[0],c[1]-half[1],c[1]+half[1]] for c in centres]
    mesh=PaperMesh(np.column_stack((points,np.zeros(len(points)))),faces,holes)
    quality(mesh,size)
    return mesh


def quality(mesh,size):
    """Reject unconstrained-Delaunay output if it changed any intended boundary."""
    audit=mesh.audit();points=mesh.rest[:,:2];triangle=points[mesh.faces]
    vectors=np.roll(triangle,-1,axis=1)-triangle
    lengths=np.linalg.norm(vectors,axis=2);angles=[]
    for i in range(3):
        a=triangle[:,(i+1)%3]-triangle[:,i];b=triangle[:,(i+2)%3]-triangle[:,i]
        angles.append(np.arccos(np.clip(np.sum(a*b,axis=1)/np.linalg.norm(a,axis=1)/np.linalg.norm(b,axis=1),-1.,1.)))
    minimum=float(np.degrees(angles).min());aspect=float(np.max(lengths.max(1)/lengths.min(1)))
    if minimum<15. or aspect>4.:raise ValueError('Paper triangle quality is below design bound')
    expected=np.prod(size)-sum((x1-x0)*(y1-y0) for x0,x1,y0,y1 in mesh.holes)
    if abs(audit['surface_area_m2']-expected)>1e-12:raise ValueError('Triangulation changed paper area')
    edges=np.sort(np.concatenate((mesh.faces[:,[0,1]],mesh.faces[:,[1,2]],mesh.faces[:,[2,0]])),axis=1)
    unique,count=np.unique(edges,axis=0,return_counts=True);boundary=points[unique[count==1]]
    def on_rectangle(edge,box):
        x0,x1,y0,y1=box
        inside=np.all((edge[:,0]>=x0-1e-10)&(edge[:,0]<=x1+1e-10)&(edge[:,1]>=y0-1e-10)&(edge[:,1]<=y1+1e-10))
        return inside and any(np.all(np.abs(edge[:,axis]-value)<1e-10) for axis,value in ((0,x0),(0,x1),(1,y0),(1,y1)))
    outer=(-size[0]/2,size[0]/2,-size[1]/2,size[1]/2)
    if any(not any(on_rectangle(edge,box) for box in [outer,*mesh.holes]) for edge in boundary):
        raise ValueError('Triangulation introduced an unintended open boundary')
    perimeters=[]
    for box in mesh.holes:
        selected=[edge for edge in boundary if on_rectangle(edge,box)]
        perimeter=sum(np.linalg.norm(edge[1]-edge[0]) for edge in selected)
        expected_perimeter=2*(box[1]-box[0]+box[3]-box[2])
        if abs(perimeter-expected_perimeter)>1e-10:raise ValueError('Puncture boundary is incomplete')
        perimeters.append(float(perimeter))
    return dict(**audit,minimum_triangle_angle_deg=minimum,maximum_edge_length_ratio=aspect,
                puncture_perimeters_m=perimeters)


def main():
    import scipy
    import argparse
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--profile",choices=("baseline","coarse","clearance-candidate"),default="baseline");args=parser.parse_args()
    destination=Path(__file__).with_name("assets")
    if args.profile!="baseline":destination=destination/args.profile
    destination.mkdir(parents=True,exist_ok=True)
    hole_size=(.0008,.0006) if args.profile=='clearance-candidate' else (.00062,.00042)
    manifest={'profile':args.profile,'hole_size_m':hole_size,
              'meaning':'Explicit candidate for the deformed puncture opening after staple insertion; not a tear allowance or runtime geometry change' if args.profile=='clearance-candidate' else 'Original proposed puncture geometry',
              'generator_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'numpy_version':np.__version__,'scipy_version':scipy.__version__,'meshes':[]}
    for i in range(2):
        centres=[(-.093,-.05+side*.0064-i*.012) for side in (-1,1)]
        mesh=make((.21,.148),centres,hole_size,profile=args.profile);path=destination/f'paper_{i}.npz'
        np.savez_compressed(path,rest=mesh.rest,faces=mesh.faces,holes=mesh.holes,
                            size=(.21,.148),centres=centres,hole_size=hole_size)
        manifest['meshes'].append({'file':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                                  **quality(mesh,(.21,.148))})
    (destination/'mesh-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__=='__main__':main()
