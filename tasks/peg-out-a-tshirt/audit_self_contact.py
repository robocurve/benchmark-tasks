"""Offline transverse self-crossing audit for native triangle-cloth recordings.

This rejects actual crossings of nonadjacent triangles, not small contact gaps.
Coplanar/tangential contact is deliberately reported as outside this audit's
scope. Passing is not a knot certificate, a tear certificate or a score.
"""
import argparse,gzip,hashlib,json
from collections import defaultdict
from pathlib import Path
import numpy as np


def crossing_pairs(nodes,faces,tolerance=1e-6,cell=.02):
    nodes=np.asarray(nodes,dtype=float);faces=np.asarray(faces,dtype=int)
    if not np.isfinite(nodes).all():raise ValueError('Nonfinite cloth nodes')
    triangles=nodes[faces];low=triangles.min(1);high=triangles.max(1)
    cells=defaultdict(list)
    for i,(lo,hi) in enumerate(zip(np.floor(low/cell).astype(int),np.floor(high/cell).astype(int))):
        if np.prod(hi-lo+1)>1000:raise ValueError('Triangle spans an implausibly large region')
        for x in range(lo[0],hi[0]+1):
            for y in range(lo[1],hi[1]+1):
                for z in range(lo[2],hi[2]+1):cells[x,y,z].append(i)
    pairs=set()
    for items in cells.values():
        pairs.update((a,b) for index,a in enumerate(items) for b in items[index+1:])
    if not pairs:return []
    pairs=np.asarray(sorted(pairs),dtype=int);i,j=pairs.T
    shared=(faces[i,:,None]==faces[j,None,:]).any((1,2))
    overlap=np.all(np.maximum(low[i],low[j])<=np.minimum(high[i],high[j])+tolerance,axis=1)
    pairs=pairs[~shared&overlap]
    if not len(pairs):return []
    a=triangles[pairs[:,0]];b=triangles[pairs[:,1]]
    ea=np.roll(a,-1,axis=1)-a;eb=np.roll(b,-1,axis=1)-b
    na=np.cross(ea[:,0],ea[:,1]);nb=np.cross(eb[:,0],eb[:,1])
    na/=np.maximum(np.linalg.norm(na,axis=1)[:,None],1e-30)
    nb/=np.maximum(np.linalg.norm(nb,axis=1)[:,None],1e-30)
    da=np.sum((a-b[:,0,None,:])*nb[:,None,:],axis=2)
    db=np.sum((b-a[:,0,None,:])*na[:,None,:],axis=2)
    transverse=(da.min(1)<-tolerance)&(da.max(1)>tolerance)&(db.min(1)<-tolerance)&(db.max(1)>tolerance)
    a,b,ea,eb,na,nb,pairs=(v[transverse] for v in (a,b,ea,eb,na,nb,pairs))
    if not len(pairs):return []
    axes=np.concatenate((na[:,None,:],nb[:,None,:],np.cross(ea[:,:,None,:],eb[:,None,:,:]).reshape(-1,9,3)),axis=1)
    norm=np.linalg.norm(axes,axis=2);axes/=np.maximum(norm[:,:,None],1e-30)
    # Shift by the first vertex to avoid sensitivity to the station origin.
    pa=np.einsum('nvc,nac->nva',a-a[:,0,None,:],axes)
    pb=np.einsum('nvc,nac->nva',b-a[:,0,None,:],axes)
    separated=((pa.max(1)<pb.min(1)-tolerance)|(pb.max(1)<pa.min(1)-tolerance))&(norm>1e-15)
    return pairs[~separated.any(1)].tolist()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('trial',type=Path)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--frames',type=int,nargs='+',default=(0,-1),help='Recorded frame indices; default initial and final')
    args=p.parse_args()
    probe=args.trial/'nodes.npz'
    if probe.exists():
        data=np.load(probe,allow_pickle=False)
        nodes=data['positions'];faces=data['faces'];rest=data['rest'];times=data['times_s']
        source=probe
    else:
        source=args.trial/'tshirt-nodes.npy'
        nodes=np.load(source,allow_pickle=False,mmap_mode='r')
        mesh=json.loads((args.trial/'meshes.json').read_text())['tshirt']
        faces=np.asarray(mesh['faces']);rest=np.asarray(mesh['rest']);times=np.arange(len(nodes))/20
        completed=json.loads((args.trial/'run.json').read_text())['completed_steps']
        nodes=nodes[:completed+1];times=times[:completed+1]
    rows=[]
    for frame in dict.fromkeys(index if index>=0 else len(nodes)+index for index in args.frames):
        if not 0<=frame<len(nodes):raise ValueError('Requested frame is outside the recorded interval')
        pairs=crossing_pairs(nodes[frame],faces,cell=.005)
        opposite_layer_pairs=sum(bool(np.all(rest[faces[a],2]>0) and np.all(rest[faces[b],2]<0) or np.all(rest[faces[a],2]<0) and np.all(rest[faces[b],2]>0)) for a,b in pairs)
        rows.append(dict(frame=frame,time_s=float(times[frame]),transverse_pairs=pairs,
                         crossing_count=len(pairs),front_back_pair_count=opposite_layer_pairs))
    result=dict(kind='offline_native_cloth_self_crossing_rejection_audit',score=None,
        source_file=source.name,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        samples=rows,max_crossing_pairs=max(row['crossing_count'] for row in rows),
        tolerance_m=1e-6,shared_vertex_pairs_excluded=True,
        limitation='Discrete transverse triangle crossings only. Shared-vertex seam neighbours are excluded; coplanar or tangential contact is not called a crossing. Does not establish continuous integrity, garment retention or a task score.')
    args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'},indent=2))
    print([(row['frame'],row['crossing_count']) for row in rows])


if __name__=='__main__':main()
