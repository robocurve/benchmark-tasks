import numpy as np
from native_model import NativeModel
from geometry import JOINTS,BODIES,GRID,PARTICLE_DIAMETER,SPACING
CELLS=[np.where(np.all((GRID>=np.array(c)-1)&(GRID<=np.array(c)),axis=1))[0] for c in np.ndindex(2,2,2)]
class Model(NativeModel):
 def __init__(self):super().__init__();self.grasp_state={};self.damage={str(k):0. for k in range(12)}
 def bind_native(self,base):
  super().bind_native(base)
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,['masher'],.010,0.,.008,(-.020,.012))
 def strain_candidates(self):
  result={};R=self.orientation('masher');origin=self.p('masher')
  bars=next(b for b in BODIES if b['name']=='masher')['shapes'][:7]
  for k in range(12):
   pts=np.array([self.p('food_%02d_%d'%(k,n)) for n in range(27)]);local=(pts-origin)@R
   contact=np.zeros(27,dtype=bool)
   for bar in bars:
    gap=np.maximum(np.abs(local-np.asarray(bar['pos']))-np.asarray(bar['size'])/2,0.)
    contact|=np.linalg.norm(gap,axis=1)<PARTICLE_DIAMETER/2+.0005
   # Local loaded compression can fracture a cooked chunk after other parts
   # have separated; scattered fragments must not define its current thickness.
   strain=0.
   for indices in CELLS:
    if not contact[indices].any():continue
    centred=pts[indices]-pts[indices].mean(axis=0);_,_,axes=np.linalg.svd(centred,full_matrices=False)
    thickness=float(np.min(np.ptp(centred@axes.T,axis=0))+PARTICLE_DIAMETER)
    strain=max(strain,1.-thickness/(SPACING+PARTICLE_DIAMETER))
   result[str(k)]=float(np.clip((strain-.15)/.30,0,1))
  return result
 def material(self,hands,dt):
  self.pinch.update(hands)
  # Cooked tissue loses cohesion under actual tool-loaded compression. The
  # native solver handles forces; complete compressive failure removes cohesion.
  # No particle pose is changed.
  for key,loaded_strain in self.strain_candidates().items():
   # Irreversible cooked-tissue damage accumulates while a cell is actually
   # compressed by a masher bar; unloaded time produces no softening.
   candidate=min(1.,max(loaded_strain,self.damage[key]+dt*loaded_strain))
   if candidate<=self.damage[key]+1e-5:continue
   first=self.damage[key]==0.;self.damage[key]=candidate
   strength=.04*(1-candidate)+.001*candidate
   for j in JOINTS:
    if j['name'].startswith('cohesion_'+key+'_') and j['name'] not in self.broken:
     if candidate>=1.-1e-6:self.disable(j['name'],'cooked_tissue_ruptured_under_loaded_compression')
     else:self.joints[j['name']].GetBreakForceAttr().Set(strength)
   if first:self.events.append(dict(event='food_softened_under_loaded_compression',piece=int(key),time=self.time))
 def state(self):return dict(**super().state(),damage=self.damage,grasp_state=self.grasp_state)

 def measure(self):
  names=[n for n in self.names if n.startswith('food_')];parent={n:n for n in names}
  def root(n):
   while parent[n]!=n:n=parent[n]
   return n
  for j in JOINTS:
   if j['name'] not in self.broken and j['name'] not in self.disabled:parent[root(j['a'])]=root(j['b'])
  groups={}
  for n in names:groups.setdefault(root(n),[]).append(n)
  sizes={}
  for key,ns in groups.items():
   p=np.array([self.p(n) for n in ns]);sizes[key]=float(np.max(np.linalg.norm(p[:,None]-p[None,:],axis=-1))+PARTICLE_DIAMETER)
  local=np.array([self.orientation('bowl').T@(self.p(n)-self.p('bowl')) for n in names]);inside=(np.linalg.norm(local[:,:2],axis=1)<.095)&(local[:,2]>.003)&(local[:,2]<.080)
  small=np.array([sizes[root(n)]<=.009 for n in names])
  return dict(small_lump_mass_fraction=float((small&inside).sum()/max(1,inside.sum())),retained_mass_fraction=float(inside.mean()),largest_lump_mm=max(sizes.values())*1000,released_seconds=self.release,failed=self.failed)
 def integrity_failed(self):return super().integrity_failed() or (False)
 def score(self):
  self.failed |= self.integrity_failed()
  m=self.measure()
  if self.failed or m['retained_mass_fraction']<.95:return 0.
  f=m['small_lump_mass_fraction']
  if f>=.95 and self.release>=3:return 1.
  return .5 if f>=.75 else .25 if f>=.5 else 0.
