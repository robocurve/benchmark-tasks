"""Live, source-aware ice scoring with useful credit for contaminated delivery."""
import math
import numpy as np


def rotation(q):
    x,y,z,w=np.asarray(q,float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


class IceScorer:
    def __init__(self,layout,hz=20):
        self.layout=layout;self.hz=hz
        self.ice={n:r['ice_source'] for n,r in layout['objects'].items() if 'ice_source' in r}
        self.bowl=np.array(layout['objects']['Bowl']['position'])
        self.lid=np.array(layout['objects']['CleanLid']['position'])
        self.reset()

    def reset(self):
        self.step=0;self.opened=False;self.scooped=set();self.handled=set();self.fish_entered=set()
        self.dirty={'Fish','FishBin',*(n for n,s in self.ice.items() if s=='fish')}
        self.last_contact=dict.fromkeys(self.ice,-10000);self.support=dict.fromkeys(self.ice,0)
        self.candidate=0.;self.held=0;self.score=0.;self.events=[]

    def update(self,poses,velocities,contacts,tcps,healthy=True):
        self.step+=1
        if not all(np.isfinite(v).all() for v in [*poses.values(),*velocities.values(),*tcps.values()]):
            self.score=0.;self.held=0
            return {'score':0.,'success':False,'valid_state':False}
        lid=poses['CleanLid'][:3]
        if not self.opened and (lid[2]>self.lid[2]+.040 or np.linalg.norm(lid[:2]-self.lid[:2])>.150):
            self.opened=True;self.events.append({'event':'lid_opened','step':self.step})
        pairs={tuple(sorted(p)) for p in contacts}
        changed=True
        while changed:
            changed=False
            for a,b in pairs:
                if (a in self.dirty)!=(b in self.dirty):self.dirty.update((a,b));changed=True
        sp=poses['Scoop'];sr=rotation(sp[3:]);held=[];served=[];clean=[];fish=[];delivered=[]
        for n,source in self.ice.items():
            if tuple(sorted(('Scoop',n))) in pairs:self.last_contact[n]=self.step
            if any(n in pair and any(r in pair for r in ('left_robot','right_robot')) for pair in pairs):self.handled.add(n)
            p=poses[n][:3];local=sr.T@(p-sp[:3])
            inside=(-.008<local[0]<.113 and abs(local[1])<self.layout['scoop_bucket_inner_width_m']/2+.003 and .005<local[2]<.060)
            recent=self.step-self.last_contact[n]<=2
            rim=self.layout['objects']['CleanBin' if source=='clean' else 'FishBin']['position'][2]+self.layout['bin_height_m']
            lifted=p[2]-.010>rim+.005
            self.support[n]=self.support[n]+1 if inside and recent and lifted and n not in self.handled else 0
            if self.support[n]>=3 and n not in self.scooped:
                self.scooped.add(n);self.events.append({'event':'ice_scooped','ice':n,'source':source,'step':self.step})
            if inside and recent and n in self.scooped:held.append(n)
            in_bowl=np.linalg.norm(p[:2]-self.bowl[:2])<.077 and self.bowl[2]+.007<p[2]<self.bowl[2]+.062
            if in_bowl:
                (clean if source=='clean' else fish).append(n)
                if source=='fish':self.fish_entered.add(n)
            v=velocities[n]
            settled=np.linalg.norm(v[:3])<=.010 and np.linalg.norm(v[3:])<=.5
            released=self.step-self.last_contact[n]>4 and bool(tcps) and all(np.linalg.norm(t-p)>=.060 for t in tcps.values())
            if in_bowl and settled and released and n in self.scooped and n not in self.handled:
                delivered.append(n)
                if source=='clean' and n not in self.dirty:served.append(n)
        full=bool(served and self.opened and not self.fish_entered and 'Scoop' not in self.dirty and 'Bowl' not in self.dirty)
        raw=1. if full else (.5 if delivered else (.25 if self.opened or held else 0.))
        if not healthy:raw=0.
        self.held=self.held+1 if raw>0 and raw==self.candidate else (1 if raw>0 else 0)
        self.candidate=raw;self.score=raw if self.held>=self.hz else 0.
        return {'score':self.score,'success':self.score==1.,'candidate_score':raw,'held_steps':self.held,
            'valid_state':bool(healthy),'lid_opened':self.opened,'clean_ice_in_bowl':clean,'fish_ice_in_bowl':fish,
            'scooped_ice':sorted(self.scooped),'held_ice':held,'delivered_by_scoop':delivered,
            'uncontaminated_clean_delivery':served,'fish_ice_ever_in_bowl':sorted(self.fish_entered),
            'scoop_contaminated':'Scoop' in self.dirty,'bowl_contaminated':'Bowl' in self.dirty,'directly_handled':sorted(self.handled)}
