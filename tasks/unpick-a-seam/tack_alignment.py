"""Scripted-demo alignment to the measured flexible seam; robot targets only."""
import re
import numpy as np


class TackAlignment:
    def __init__(self,poses,counts):
        self.slices=[];self.front=[];self.reference=[];start=0
        for n in counts:
            self.slices.append(slice(start,start+n));part=np.asarray(poses)[start:start+n,:3]
            mask=part[:,0]<part[:,0].min()+.0006
            self.front.append(mask);self.reference.append(part[mask].mean(0));start+=n
        self.offset=np.zeros(3);self.frozen={};self.records=[]

    def apply(self,position,phase,poses,closed,step):
        match=re.search(r'tack (\d+)',phase)
        index=int(match.group(1))-1 if match else 0 if ('first tack' in phase or 'first cut' in phase) else None
        wanted=np.zeros(3)
        if index is not None:
            measured=np.asarray(poses)[self.slices[index],:3][self.front[index]].mean(0)
            if closed[index]:self.frozen[index]=measured-self.reference[index]
            wanted=self.frozen.get(index,np.zeros(3))
            if np.linalg.norm(wanted)>.012:raise RuntimeError('Stitch moved outside the supported sewing window')
            if not closed[index]:
                # Once actual blade contact has opened the tack, withdraw
                # normal to the canvas before continuing the lateral return.
                # This is demo feedback through ordinary robot targets only.
                wanted=wanted+np.array((-.006,0.,0.))
        self.offset+=np.clip((wanted-self.offset)*.2,-.0003,.0003)
        if step%20==0:self.records.append(dict(step=step,stitch=index,translation_m=self.offset.tolist()))
        return np.asarray(position)+self.offset
