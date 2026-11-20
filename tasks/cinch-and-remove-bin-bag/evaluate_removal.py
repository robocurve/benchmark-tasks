"""Offline re-scoring of a recorded run with the primary tiered drawstring judge.

Also holds the shared non-convex containment check for the three waste blocks.
Usage: python evaluate_removal.py runs/demo_001  (writes verified_result.json)
"""
import argparse, json
from pathlib import Path
import numpy as np
from scipy.spatial import QhullError
from scipy.spatial.transform import Rotation

def inside_bag(points, vertices, tolerance=.008):
    """Per-point containment in the liner, using two evaluator-only caps:
    across the hem (the mouth once the cuff is lifted) and across the fold
    over the rim (the mouth while the surplus is folded down outside). A
    point inside either closed film surface is inside the bag."""
    from containment import closed_triangles, contains_points
    from realistic_meshes_v32 import dirty_mesh, fold_ring_ids, N_AROUND, FOLD_RING
    _,faces,mouth_ids=dirty_mesh()
    points=np.asarray(points,float)
    inside=contains_points(points,closed_triangles(vertices,faces,mouth_ids),tolerance=tolerance,signed=True)
    if not inside.all():
        lower=faces[np.all((faces-1)//N_AROUND<=FOLD_RING,axis=1)]
        inside[~inside]=contains_points(points[~inside],closed_triangles(vertices,lower,fold_ring_ids()),tolerance=tolerance)
    return inside

def contents_retained(s, vertices):
    centres=np.asarray(s['rubbish'],float)
    quats=np.asarray(s['rubbish_quaternions'],float)
    from removal_assets import BLOCK_SIZES
    n=len(BLOCK_SIZES)
    if centres.shape!=(n,3) or quats.shape!=(n,4):return False,None
    corner_list,other_list=[],[]
    for c,q,size in zip(centres,quats,BLOCK_SIZES):
        hx,hy,hz=(b/2 for b in size)
        offsets=np.array([[x*hx,y*hy,z*hz] for x in [-1,1] for y in [-1,1] for z in [-1,1]])
        # Corners, face centres and the centre of each waste block are checked.
        face_offsets=np.array([[hx,0,0],[-hx,0,0],[0,hy,0],[0,-hy,0],[0,0,hz],[0,0,-hz],[0,0,0]])
        r=Rotation.from_quat(q)
        corner_list.append(c+r.apply(offsets));other_list.append(c+r.apply(face_offsets))
    corners=np.concatenate(corner_list);other=np.concatenate(other_list)
    # Block centres and face centres must lie inside the film within the 8 mm
    # contact tolerance. Corners get 20 mm, the largest film triangle edge: a
    # sharp box corner can protrude through a film triangle between its nodes
    # (observed up to about 15 mm) while the film still encloses the block.
    ok=bool(np.all(inside_bag(other,vertices))) and bool(np.all(inside_bag(corners,vertices,tolerance=.020)))
    return ok,corners

def evaluate(trace,snapshots,parameters):
    """Primary score: the tiered drawstring judge shared with the live tracker."""
    from drawstring_score import evaluate as tiered
    try:return tiered(trace,snapshots,parameters)
    except (ValueError,KeyError,TypeError,IndexError,QhullError) as exc:return {'score':0.,'full_task_success':False,'reason':'Invalid evidence: '+str(exc)}

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();root=a.run
    trace=json.loads((root/'trace.json').read_text())
    states={s['step']:np.load(root/('state_%04d.npz'%s['step']))['loaded_bag'][0] for s in trace}
    result=evaluate(trace,states,json.loads((root/'task_parameters.json').read_text()))
    result['execution_error']=any((root/n).exists() for n in ['error.txt','trial_error.txt'])
    # Preserve the geometric score. The launcher separately rejects an
    # exception as successful demonstration evidence, even if the final
    # physical state was complete before a later recording/runtime error.
    (root/'verified_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if __name__=='__main__':main()

