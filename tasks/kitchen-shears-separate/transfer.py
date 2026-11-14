"""Robot-target-only sequential transfer after observed physical separation."""
import numpy as np


def rz(a):
    c,s=np.cos(a),np.sin(a)
    return np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]])


def rotation_between(a,b,u):
    q=b@a.T
    ang=np.arccos(np.clip((np.trace(q)-1)/2,-1.,1.))
    if ang<1e-9:return a.copy()
    axis=np.array([q[2,1]-q[1,2],q[0,2]-q[2,0],q[1,0]-q[0,1]])/(2*np.sin(ang))
    x,y,z=axis
    k=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
    return (np.eye(3)+np.sin(u*ang)*k+(1-np.cos(u*ang))*(k@k))@a


class Transfer:
    def __init__(self,top):self.top=top;self.reset()
    def reset(self):self.started=None;self.elapsed=0.;self.segments=[]

    def begin(self,t,positions,rotations,commands):
        self.started=float(t);self.elapsed=0.
        self.positions=np.asarray(positions[:2],float).copy()
        self.rotations=np.asarray(rotations[:2],float).copy()
        self.commands=[(np.array(p).copy(),np.array(r).copy(),0.) for p,r in commands]
        state=[(p.copy(),r.copy(),f) for p,r,f in self.commands]
        self.segments=[]

        def add(dt,i,p,r,f,label):
            new=[(v[0].copy(),v[1].copy(),v[2]) for v in state]
            new[i]=(np.asarray(p).copy(),np.asarray(r).copy(),f)
            self.segments.append((dt,[(v[0].copy(),v[1].copy(),v[2]) for v in state],new,label))
            state[:]=new

        for i,sgn in ((1,1),(0,-1)):
            angle=float(np.arctan2(self.rotations[i,1,0],self.rotations[i,0,0]))
            stages=[(2.,[.255,0.,self.top+.075],angle,'raise clear'),
                    (2.,[.235,sgn*.100,self.top+.075],angle,'outboard of other hand'),
                    (2.,[.235,sgn*.100,self.top+.075],0.,'turn above table'),
                    (3.,[.489,sgn*.060,self.top+.075],0.,'carry over tray'),
                    (3.,[.489,sgn*.060,self.top+.004],0.,'lower onto tray floor')]
            for dt,p,a,label in stages:
                rr=rz(a)@self.rotations[i].T
                tcp=np.asarray(p)+rr@(self.commands[i][0]-self.positions[i])
                rot=rr@self.commands[i][1]
                add(dt,i,tcp,rot,0.,f'half {i}: {label}')
            tcp,rot,_=state[i]
            # 24 mm pad gap releases the 8 mm loop wall without sweeping the tray rim.
            add(1.,i,tcp,rot,-.012,f'half {i}: release loop')
            add(2.,i,tcp+[0,0,.060],rot,-.012,f'half {i}: withdraw up')
            add(2.,i,[.17,sgn*.28,self.top+.22],rot,-.012,f'half {i}: park clear')
        add(3.,0,*state[0],'final released stillness')
        self.duration=sum(s[0] for s in self.segments)

    def target(self,i,elapsed=None):
        elapsed=self.elapsed if elapsed is None else elapsed
        for dt,start,end,label in self.segments:
            if elapsed<=dt:
                u=float(np.clip(elapsed/dt,0.,1.));u=u*u*(3-2*u)
                p0,r0,f0=start[i];p1,r1,f1=end[i]
                return p0+(p1-p0)*u,rotation_between(r0,r1,u),f0+(f1-f0)*u,label
            elapsed-=dt
        p,r,f=self.segments[-1][2][i]
        return p.copy(),r.copy(),f,'final hold'
