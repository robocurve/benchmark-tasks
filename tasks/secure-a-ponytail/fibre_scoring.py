"""Equal-mass fibre samples in each native lock; no change to hair physics.

Rendered fibres share each articulated lock's deformation. They are not
independent physics strands; this proxy resolves partial enclosure of a lock.
"""
import numpy as np
from geometry import STRANDS,SEGMENTS
from hair_visual import offsets,LENGTHS

def captured_fractions(model,ring,centre,axes):
    normal=axes[2];result=[]
    for s in range(STRANDS):
        names=[f'hair_{s}_{i}' for i in range(SEGMENTS)]
        p=np.array([model.p(names[0],[0,0,-LENGTHS[names[0]]/2])]+[model.p(n,[0,0,LENGTHS[n]/2]) for n in names])
        tangent=np.gradient(p,axis=0);tangent/=np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1e-12)
        ref=np.tile([1.,0.,0.],(len(p),1));ref[np.abs(tangent[:,0])>.9]=[0,1,0]
        u=np.cross(tangent,ref);u/=np.maximum(np.linalg.norm(u,axis=1)[:,None],1e-12);v=np.cross(tangent,u)
        fibres=p[None,:,:]+offsets[:,0,None,None]*u+offsets[:,1,None,None]*v
        signed=(fibres-centre)@normal
        crosses=signed[:,:-1]*signed[:,1:]<=0
        selected=np.argwhere(crosses)
        held=np.zeros(len(offsets),bool)
        if len(selected):
            f,i=selected.T;den=signed[f,i]-signed[f,i+1]
            t=np.divide(signed[f,i],den,out=np.zeros_like(den),where=abs(den)>1e-12)
            c=fibres[f,i]+t[:,None]*(fibres[f,i+1]-fibres[f,i])
            local=(ring[None,:,:]-c[:,None,:])@axes[:2].T
            a=np.arctan2(local[:,:,1],local[:,:,0]);delta=np.diff(np.c_[a,a[:,0]],axis=1)
            inside=abs(np.sum((delta+np.pi)%(2*np.pi)-np.pi,axis=1))>1.6*np.pi
            held[f[inside]]=True
        result.append(float(held.mean()))
    return result

def retained_count(fractions):
    return sum(f>=.9 for f in fractions)
