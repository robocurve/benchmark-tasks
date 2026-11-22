"""Full-width wood voxels with path-dependent cuts in the wood's own frame.

Top-down removal remains an approximation; X/Y connectivity is real voxel
connectivity, so a groove that misses one side cannot split the board.
"""
from collections import deque
import numpy as np


class Stock:
    def __init__(self, dx=.00025, dy=.002, dz=.0005):
        self.nx=round(.300/dx);self.ny=round(.020/dy);self.nz=round(.020/dz)
        self.dx=.300/self.nx;self.dy=.020/self.ny;self.dz=.020/self.nz
        self.progress=np.zeros((self.nx,self.ny),np.float64)
        self.cleared=np.zeros((self.nx,self.ny),np.int16)
        self.removed_last_step=False
        # Per-column cumulative removal attributable to unclamped contact.
        self.loose_progress=np.zeros_like(self.progress)

    @property
    def unsafe_loose_cut(self):
        return bool(np.any(self.loose_progress>=.0005-1e-12))

    def remove(self,tooth0,tooth1,ds,fn,clamped=True):
        """Update teeth at their measured local pose, force and stroke.

        Attribute positive per-column depth increments to loose history only
        while the measured clamp condition fails; clamping never clears it.
        """
        self.removed_last_step=False
        if fn<=.05 or ds<=1e-8:return False
        a,b=np.asarray(tooth0,float),np.asarray(tooth1,float)
        if abs(b[1]-a[1])<1e-8:return False
        y=(np.arange(self.ny)+.5)*self.dy-.010
        t=(y-a[1])/(b[1]-a[1]);rows=(t>=0)&(t<=1)
        if not rows.any():return False
        x=a[0]+t*(b[0]-a[0]);z=a[2]+t*(b[2]-a[2])
        changed=False;active=[]
        for j in np.flatnonzero(rows):
            lo=max(0,int(np.floor((x[j]-.000525)/self.dx)))
            hi=min(self.nx,int(np.ceil((x[j]+.000525)/self.dx)))
            for i in range(lo,hi):
                if self.cleared[i,j]>=self.nz:continue
                left=i*self.dx;right=left+self.dx
                if left>=x[j]+.000525 or right<=x[j]-.000525:continue
                surface=.020-self.cleared[i,j]*self.dz
                gap=z[j]-surface
                if -.0005<=gap<=.0001:active.append((i,j))
        if not active:return False
        self.removed_last_step=True
        dh=.85*fn/(5+fn)*ds
        volume=dh*.001*len(np.unique([j for _,j in active]))*self.dy
        inc=volume/(len(active)*self.dx*self.dy)
        for i,j in active:
            old=int(self.cleared[i,j])
            before=self.progress[i,j]
            self.progress[i,j]=min(.020,self.progress[i,j]+min(inc,(old+1)*self.dz-self.progress[i,j]))
            if not clamped:
                self.loose_progress[i,j]+=self.progress[i,j]-before
            n=min(self.nz,int((self.progress[i,j]+1e-12)/self.dz))
            if n>old:self.cleared[i,j]=n;changed=True
        return changed

    def components(self):
        alive=self.cleared<self.nz
        if not np.all(np.any(~alive,axis=0)):
            return [alive]
        labels=np.zeros(alive.shape,np.int16);out=[]
        for i,j in zip(*np.where(alive & (labels==0))):
            if labels[i,j]:continue
            label=len(out)+1;mask=np.zeros_like(alive)
            q=deque([(int(i),int(j))]);labels[i,j]=label
            while q:
                x,y=q.popleft();mask[x,y]=True
                for u,v in [(x-1,y),(x+1,y),(x,y-1),(x,y+1)]:
                    if 0<=u<self.nx and 0<=v<self.ny and alive[u,v] and labels[u,v]==0:
                        labels[u,v]=label;q.append((u,v))
            out.append(mask)
        return out

    def boxes(self,mask=None):
        if mask is None:mask=self.cleared<self.nz
        # Merge equal-height X runs, then identical adjacent Y rows.
        pending={};boxes=[]
        for j in range(self.ny+1):
            row={}
            if j<self.ny:
                i=0
                while i<self.nx:
                    if not mask[i,j]:i+=1;continue
                    n=int(self.cleared[i,j]);k=i+1
                    while k<self.nx and mask[k,j] and self.cleared[k,j]==n:k+=1
                    row[(i,k,n)]=True;i=k
            for key,start in list(pending.items()):
                if key not in row:
                    i,k,n=key;h=.020-n*self.dz
                    if h>0:boxes.append(((i+k)*self.dx/2,(start+j)*self.dy/2-.010,h/2,
                                         (k-i)*self.dx,(j-start)*self.dy,h))
                    del pending[key]
            for key in row:
                if key not in pending:pending[key]=j
        return boxes

    def score(self,target=.150,tol=.001):
        parts=self.components()
        if len(parts)!=2:return {'value':0.,'reason':'not_two_pieces','pieces':len(parts)}
        left=next((p for p in parts if p[0,:].any()),None)
        if left is None:return {'value':0.,'reason':'no_retained_end','pieces':2}
        # Judge every width/depth sample of the retained end face.
        lengths=[]
        for j in range(self.ny):
            for layer in range(self.nz):
                x=np.flatnonzero(left[:,j] & (self.cleared[:,j]<=layer))
                if not len(x):return {'value':0.,'reason':'missing_cross_section','pieces':2}
                lengths.append((int(x[-1])+1)*self.dx)
        err=max(abs(x-target) for x in lengths)
        return {'value':float(err<=tol+1e-12),'reason':'within_tolerance' if err<=tol+1e-12 else 'wrong_length',
                'pieces':2,'min_length_m':min(lengths),'max_length_m':max(lengths),'max_error_m':err}


    def stages(self):
        """Geometry-only stages; runtime also requires safe clamping/clearance.

        A 5 mm work zone includes the kerf and positioning tolerance. Half-
        depth credit needs one connected groove at the target across all rows.
        Fractional work without a cleared collider layer never counts as depth.
        """
        full=self.score()
        damaged=self.cleared*self.dz>=.0005-1e-12
        x=np.arange(self.nx)*self.dx
        zone=(x>=.149-1e-12)&(x+self.dx<=.154+1e-12)
        if np.any(damaged[~zone]):
            return dict(full,stage_value=0.,stage_reason='off_target_removal')
        if full['pieces']>1:
            return dict(full,stage_value=full['value'])
        deep=(self.cleared*self.dz>=.010-1e-12)&zone[:,None]
        seen=np.zeros_like(deep)
        for i,j in zip(*np.where(deep)):
            if seen[i,j]:continue
            q=deque([(int(i),int(j))]);seen[i,j]=True
            left=np.full(self.ny,np.inf)
            while q:
                a,b=q.popleft();left[b]=min(left[b],a*self.dx)
                for u,v in ((a-1,b),(a+1,b),(a,b-1),(a,b+1)):
                    if 0<=u<self.nx and 0<=v<self.ny and deep[u,v] and not seen[u,v]:
                        seen[u,v]=True;q.append((u,v))
            if np.all((left>=.149-1e-12)&(left<=.151+1e-12)):
                return dict(full,stage_value=.5,stage_reason='full_width_half_depth')
        return dict(full,stage_value=.25,stage_reason='clamped_only')
