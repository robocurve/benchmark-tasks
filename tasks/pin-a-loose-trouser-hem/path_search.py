"""Find an ordinary straight needle route through the observed lifted fold."""
import numpy as np

def select_path(m):
 names=[n for n in m.names if n.startswith(('front_','hem_','back_'))]
 centres=np.array([m.p(n) for n in names]);Rs=np.array([m.orientation(n) for n in names]);normals=Rs[:,:,2]
 half=np.array([.004 if n.startswith('hem_') else .011 for n in names])
 front=np.array([n.startswith('front_') for n in names]);hem=np.array([n.startswith('hem_') for n in names])
 loose=np.array([4<=int(n.split('_')[1])<14 for n in names]);apex=max(m.p(n)[2] for n in names if n.startswith('hem_'))
 # Work near the held fold, where the fingers support the fabric against
 # puncture load, just beyond the finger tips rather than in the slack hanging area.
 origins=np.array([[.34,y,z] for y in np.arange(.095,.130,.0015) for z in np.arange(max(.79,apex-.010),min(.885,apex-.006),.0015)])
 if not len(origins):raise RuntimeError('The hem was not lifted clear for threading.')
 choices=[]
 for az in range(-40,41,10):
  for el in range(-60,1,5):
   a,e=np.radians([az,el]);direction=np.array([np.cos(a)*np.cos(e),np.sin(a)*np.cos(e),np.sin(e)]);den=normals@direction
   distance=np.einsum('ni,oni->on',normals,centres[None]-origins[:,None])/np.where(abs(den)>.2,den,np.nan)
   points=origins[:,None]+distance[:,:,None]*direction;local=np.einsum('nji,onj->oni',Rs,points-centres)
   mask=(abs(den)>.2)&(abs(local[:,:,0])<.004)&(abs(local[:,:,1])<half)
   for row in np.where(mask.sum(axis=1)>=4)[0]:
    ids=np.where(mask[row])[0];ids=ids[np.argsort(distance[row,ids])]
    for start in range(len(ids)-3):
     ix=ids[start:start+4];ds=distance[row,ix];span=ds[-1]-ds[0]
     if front[ix].sum()!=2 or hem[ix].sum()!=2 or not loose[ix].all() or not .008<span<.058:continue
     if start and distance[row,ids[start-1]]>ds[0]-.025:continue
     if start+4<len(ids) and distance[row,ids[start+4]]<ds[-1]+.003:continue
     entry=origins[row]+(ds[0]-.025)*direction;end=origins[row]+(ds[-1]+.003)*direction
     choices.append((-span-.0002*(abs(az)+abs(el+35)),entry,end,direction,[names[i] for i in ix]))
 if not choices:raise RuntimeError('The held hem does not present a clear four-layer needle path; stop this development trial.')
 _,entry,end,direction,patches=max(choices,key=lambda x:x[0])
 return entry,end,direction,patches
