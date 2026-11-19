"""Small length-constrained ribbon-centreline solver for the unfinished wide blade.

The pre-wrapped knot is a fixed compliant guide, not a full textile solver.
Gravity and finite pad contact move the free blade. Passage is checked against
the actual open front sleeve; the knot cannot become threaded by a timer or
by pulling a blade outside that sleeve.
"""
import numpy as np

N=33
LENGTH=.33

class LooseBlade:
    def __init__(self, centre):
        self.centre=np.asarray(centre).copy()
        root=self.centre+[.010,0,.046]
        corners=np.array([root,root+[-.070,0,.010],root+[-.070,0,-.2493]])
        length=np.r_[0,np.cumsum(np.linalg.norm(np.diff(corners,axis=0),axis=1))]
        self.points=np.column_stack([np.interp(np.linspace(0,length[-1],N),length,corners[:,k]) for k in range(3)])
        self.previous=self.points.copy();self.rest=LENGTH/(N-1)
        self.entered=False;self.exited=False;self.max_stretch=1.
        self.root=root;self.last_tip=self.points[-1].copy()

    def advance(self, grip=None, dt=.05):
        """A captured material node follows its enclosed finger contact only."""
        step=dt/4
        weights=np.ones(N);weights[0]=0
        if grip is not None:weights[grip[0]]=0
        for _ in range(4):
            x=self.points;v=(x-self.previous)*.97;self.previous=x.copy()
            x+=v+np.array([0,0,-9.81])*step*step
            for _ in range(300):
                x[0]=self.root
                if grip is not None:x[grip[0]]=grip[1]
                for parity in [0,1]:
                    ids=np.arange(parity,N-1,2);d=x[ids+1]-x[ids]
                    ln=np.maximum(np.linalg.norm(d,axis=1),1e-9)
                    den=np.maximum(weights[ids]+weights[ids+1],1.)
                    corr=d*((ln-self.rest)/ln/den)[:,None]
                    x[ids]+=corr*weights[ids,None];x[ids+1]-=corr*weights[ids+1,None]
                # Project penetrations to the nearest surface of the finite
                # sleeve wall; the top and bottom apertures remain open.
                c=self.centre;q=x-c;inner=(q[:,0]/.014)**2+(q[:,1]/.034)**2
                outer=(q[:,0]/.023)**2+(q[:,1]/.043)**2
                wall=(abs(q[:,2])<.028)&(inner>1)&(outer<1)
                ids=np.flatnonzero(wall)
                if len(ids):
                    qh=q[ids]
                    candidates=np.zeros((len(ids),4,3))
                    candidates[:,0,:2]=qh[:,:2]*(.999/np.sqrt(inner[ids])[:,None]-1)
                    candidates[:,1,:2]=qh[:,:2]*(1.001/np.sqrt(outer[ids])[:,None]-1)
                    candidates[:,2,2]=.0285-qh[:,2]
                    candidates[:,3,2]=-.0285-qh[:,2]
                    nearest=np.argmin(np.linalg.norm(candidates,axis=2),axis=1)
                    x[ids]+=candidates[np.arange(len(ids)),nearest]
            x[0]=self.root
            if grip is not None:x[grip[0]]=grip[1]
        stretch=np.max(np.linalg.norm(np.diff(self.points,axis=0),axis=1)/self.rest)
        self.max_stretch=max(self.max_stretch,float(stretch))
        tip=self.points[-1];top=self.centre[2]+.028;bottom=self.centre[2]-.028
        if self.last_tip[2]>top>=tip[2]:
            u=(top-self.last_tip[2])/(tip[2]-self.last_tip[2]);p=self.last_tip+u*(tip-self.last_tip)
            self.entered=bool(((p[0]-self.centre[0])/.014)**2+((p[1]-self.centre[1])/.034)**2<1)
        if self.entered and self.last_tip[2]>bottom>=tip[2]:
            u=(bottom-self.last_tip[2])/(tip[2]-self.last_tip[2]);p=self.last_tip+u*(tip-self.last_tip)
            self.exited=bool(((p[0]-self.centre[0])/.014)**2+((p[1]-self.centre[1])/.034)**2<1)
        if tip[2]>top+.01:self.exited=False
        self.last_tip=tip.copy()
        return self.entered and self.exited and tip[2]<bottom-.23

    def samples(self):
        # Material coordinate measured from the free tip, as in the snugging model.
        distances=np.linspace(LENGTH,0,N)
        ids=np.arange(4,N-2)
        return self.points[ids],distances[ids],ids
