"""Native contact-point witness for gel damage by any robot/tool/lid body.

Root-body pair filters are intentional: the backend does not provide reliable
child-collider filtering. Points, normals and signed separation distinguish the
upper gel surface from the dish floor, side wall and external support.
This is the same disclosed 0.3 mm indentation approximation as the T-bar model.
"""
PAIR_CAPACITY=1024
EVENT_CAPACITY=1024


def contact_candidates(raw, base_pose, base_filter, sensor_rows):
    import torch
    _, points, normals, separation, counts, starts = raw
    rows=torch.as_tensor(sensor_rows,dtype=torch.long,device=points.device)
    count=counts[rows,base_filter].long();start=starts[rows,base_filter].long()
    offsets=torch.arange(PAIR_CAPACITY,device=points.device)
    indices=(start[:,None]+offsets).clamp(0,len(points)-1)
    valid=(offsets[None,:]<count[:,None]).reshape(-1)
    indices=indices.reshape(-1)
    p=points[indices]-base_pose[:3];n=normals[indices]
    # Inverse rotation from native xyzw quaternion, preserving dish rotation.
    q=base_pose[3:6].expand_as(p);w=base_pose[6]
    def local(v):return v-2*w*torch.cross(q,v,dim=-1)+2*torch.cross(q,torch.cross(q,v,dim=-1),dim=-1)
    p=local(p)*1000;n=local(n)
    gel=valid&(p[:,:2].square().sum(-1)<=43.4**2)&(p[:,2]>=3.)&(p[:,2]<=6.2)&(n[:,2].abs()>=.5)
    depth=(-separation.reshape(-1)[indices]*1000).clamp(min=0)
    damaged=gel&(depth>=.3)
    events=torch.cat((p[:,:2],torch.where(damaged,depth,0.)[:,None],rows[:,None].expand(-1,PAIR_CAPACITY).reshape(-1,1).float()),dim=1)
    events=torch.where(damaged[:,None],events,0.)
    bad=valid&(~torch.isfinite(p).all(-1)|~torch.isfinite(n).all(-1)|~torch.isfinite(depth))
    stats=torch.stack((count.sum(),gel.sum(),damaged.sum(),(count>PAIR_CAPACITY).sum()+bad.sum())).float()
    return events,stats


def collect(robot_raw,object_raw,base_pose,base_filter,robot_rows,object_rows):
    import torch
    a,sa=contact_candidates(robot_raw,base_pose,base_filter,robot_rows)
    b,sb=contact_candidates(object_raw,base_pose,base_filter,object_rows)
    # Body tag >=100 distinguishes object/lid contacts from robot links.
    b[:,3]+=100
    all_events=torch.cat((a,b));stats=sa+sb
    stats[3]+=(stats[2]>EVENT_CAPACITY).float()
    _,indices=torch.topk(all_events[:,2],EVENT_CAPACITY,sorted=True)
    return torch.cat((all_events[indices],stats[None,:]))


def apply_events(model,events):
    import numpy as np
    events=np.asarray(events,float)
    if not np.isfinite(events).all():raise RuntimeError('Invalid native gel contact witness')
    for x,y,depth,*_ in events:
        if depth<.3:continue
        i=int(np.floor((x-model.x[0])/model.dx+.5));j=int(np.floor((y-model.y[0])/model.dx+.5))
        if 0<=i<len(model.x) and 0<=j<len(model.y) and model.domain[i,j]:
            model.damage[i,j]=max(model.damage[i,j],depth)
