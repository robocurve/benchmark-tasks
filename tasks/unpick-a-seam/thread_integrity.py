"""Measured continuity of native thread joints; no object or constraint writes.

Capsule axes alone omit the small solver gap at a spherical joint. Closed
junction connectors complete the audited centreline. Only a force-qualified
severed joint is omitted. Disjoint end caps are a definite invalid model state,
not a simulated physical cut or a scoreable removal.
"""
import numpy as np


class ThreadIntegrity:
    def __init__(self,link_counts,radius):
        self.counts=tuple(map(int,link_counts));self.radius=float(radius)
        if not self.counts or min(self.counts)<3 or not self.radius>0:
            raise ValueError('Positive-radius closed thread chains required')
        self.previous=np.concatenate([np.arange(start,start+count)-1 for start,count in
                                      zip(np.cumsum((0,)+self.counts[:-1]),self.counts)])
        start=0
        for count in self.counts:self.previous[start]=start+count-1;start+=count
        self.maximum_closed_gap_m=0.;self.first_failure=None

    def update(self,endpoints,severed,time_s):
        endpoints=np.asarray(endpoints,float)
        if endpoints.shape!=(sum(self.counts),2,3) or not np.isfinite(endpoints).all():
            raise ValueError('Measured endpoints for every thread capsule required')
        severed=set(map(int,severed))
        if any(j<0 or j>=len(endpoints) for j in severed):raise ValueError('Unknown severed joint')
        connectors=np.stack((endpoints[self.previous,1],endpoints[:,0]),axis=1)
        closed=np.ones(len(endpoints),bool)
        if severed:closed[list(severed)]=False
        gaps=np.linalg.norm(connectors[:,1]-connectors[:,0],axis=1)
        current=float(gaps[closed].max()) if closed.any() else 0.
        self.maximum_closed_gap_m=max(self.maximum_closed_gap_m,current)
        failed=closed&(gaps>2*self.radius)
        if failed.any() and self.first_failure is None:
            joint=int(np.flatnonzero(failed)[np.argmax(gaps[failed])])
            self.first_failure={'time_s':float(time_s),'joint':joint,'gap_m':float(gaps[joint]),
                                'maximum_connected_endcap_gap_m':2*self.radius,
                                'reason':'unsevered native joint has physically disjoint end caps'}
        # Keep a fixed array shape for the persistent GPU auditor. A disabled
        # connector is zero length and cannot cross a face; actual endcaps and
        # every remaining physical capsule still undergo ordinary audits.
        connectors[~closed,1]=connectors[~closed,0]
        return connectors

    def summary(self):
        return {'maximum_closed_joint_gap_m':self.maximum_closed_gap_m,
                'first_failure':self.first_failure,'valid':self.first_failure is None}


class ClosedJunctionCrossings:
    """Every-substep connector audit, with coherent history after a real cut.

    A severed connection ceases to exist. Remove that connector from the current
    and previous arrays together, while retaining every other connector's sweep
    history. Collapsing its endpoint instead would create a fictitious sweep.
    """
    def __init__(self,faces,number_of_joints,*,boundary_tolerance_m=0.):
        from contact_audit import ThreadClothAudit
        self.audits=[ThreadClothAudit(f,boundary_tolerance_m=boundary_tolerance_m) for f in faces]
        self.active=np.arange(number_of_joints)
        self.severed=set()

    def update(self,cloth,connectors,severed,time_s):
        severed=set(map(int,severed))
        if not self.severed.issubset(severed):raise ValueError('A severed joint cannot silently reconnect')
        if severed!=self.severed:
            keep=np.array([int(j) not in severed for j in self.active])
            for audit in self.audits:
                if audit.previous is not None:audit.previous=audit.previous[keep]
            self.active=self.active[keep];self.severed=severed
        selected=np.asarray(connectors)[self.active]
        for audit,vertices in zip(self.audits,cloth,strict=True):
            audit.update(vertices,selected,time_s)
            if audit.first_failure is not None and 'joint_indices_in_segment_order' not in audit.first_failure:
                audit.first_failure['joint_indices_in_segment_order']=self.active.tolist()
        return any(a.first_failure is not None for a in self.audits)

    def summary(self):return [audit.summary() for audit in self.audits]
