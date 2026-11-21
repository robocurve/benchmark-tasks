import sys; sys.path.insert(0,'people')
from make_child import *
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
SKIN=(0.87,0.68,0.56); SHIRT=(0.20,0.45,0.75); TROUSERS=(0.25,0.27,0.33); SHOES=(0.92,0.92,0.90); HAIR=(0.25,0.17,0.10)
def dominant():
    idx=m.vertex_bone_indices.numpy(); w=m.vertex_bone_weights.numpy()
    return np.array([m.bone_labels[i] for i in idx[np.arange(len(idx)),w.argmax(1)]])
def colours(v):
    d=dominant(); c=np.tile(SKIN,(len(v),1))
    def on(keys): return np.array([any(k in b for k in keys) for b in d])
    c[on(['spine','shoulder','clavicle','breast','upperarm','root'])]=SHIRT
    c[on(['pelvis','upperleg','lowerleg'])]=TROUSERS
    c[on(['foot','toe'])]=SHOES
    head=on(['head','jaw','eye','tongue','levator','special','oris','risorius','temporal','orbicularis'])|on(['neck03'])
    hz=v[on(['head']),2]; top=hz.max(); 
    hair=(v[:,2]>top-0.40*(top-hz.min()))&(v[:,1]>v[on(['head']),1].mean()-0.02)  # crown and back (face is -y)
    hair|=(v[:,2]>top-0.22*(top-hz.min()))
    c[hair&(v[:,2]>hz.min())&(np.abs(v[:,0]-v[on(['head']),0].mean())<0.2)]=HAIR
    return c
POSES={'toddler_at_table':(0.13,0.5,{'upperarm01.L':('x',-55),'upperarm01.R':('x',-55),'lowerarm01.L':('x',-25),'lowerarm01.R':('x',-25)}),
       'toddler_arms_down':(0.13,0.5,{'upperarm01.L':('y',42),'upperarm01.R':('y',-42)}),
       'baby':(0.0,0.2,{'upperarm01.L':('y',25),'upperarm01.R':('y',-25)})}
if __name__=='__main__':
    fig=plt.figure(figsize=(13,5))
    for i,(n,(age,h,pose)) in enumerate(POSES.items()):
        v=body(age,h,pose); v=v-[v[:,0].mean(),v[:,1].mean(),v[:,2].min()]; c=colours(v)
        np.savez(f'people/{n}.npz',v=v,f=F,c=c); trimesh.Trimesh(v,F,vertex_colors=(c*255).astype(np.uint8),process=False).export(f'people/{n}.ply')
        e=v.max(0)-v.min(0); print(n,[round(float(x)*100,1) for x in e],'hands z',round(float(v[dominant()=='wrist.L',2].mean())*100,1))
        ax=fig.add_subplot(1,3,i+1,projection='3d'); tri=v[F]; fc=c[F].mean(1)
        nrm=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]); nrm/=(np.linalg.norm(nrm,axis=1,keepdims=True)+1e-9)
        sh=(0.4+0.6*np.clip(nrm@np.array([0.4,-0.8,0.4]),0,1))[:,None]
        ax.add_collection3d(Poly3DCollection(tri,facecolors=fc*sh,edgecolor='none'))
        r=e.max()/2; ax.set_xlim(-r,r);ax.set_ylim(-r,r);ax.set_zlim(0,2*r); ax.view_init(10,-60); ax.set_axis_off(); ax.set_title(f"{n}: {e[2]*100:.0f} cm")
    plt.tight_layout(); plt.savefig('people/preview2.png',dpi=100)


SKIN2=(0.78,0.56,0.44); LIPS=(0.72,0.35,0.36); EYE_W=(0.93,0.93,0.92); IRIS=(0.20,0.12,0.07); BROW=(0.22,0.14,0.08)
def paint(age,height,linear=True):
    """Vertex colours from the rest pose (same vertex order in every pose): clothes, hair, and a face with eyes,
    eyebrows and lips. ``linear`` converts sRGB to linear for the renderer."""
    v=body(age,height,{}); v=v-[v[:,0].mean(),v[:,1].mean(),v[:,2].min()]; d=dominant(); c=colours(v)
    c[(c==np.array(SKIN)).all(1)]=SKIN2
    eL,eR=v[d=='eye.L'],v[d=='eye.R']; cL,cR=eL.mean(0),eR.mean(0); dist=np.linalg.norm(cL-cR); r=dist*0.19
    head=(d=='head')
    for e,ctr in ((eL,cL),(eR,cR)):
        m_=(d=='eye.L')|(d=='eye.R'); front=e[e[:,1].argmin()]
        idx=np.where(m_)[0]; c[idx]=EYE_W
        c[idx[np.linalg.norm(v[idx]-front,axis=1)<r*0.75]]=IRIS
        brow=head&(np.abs(v[:,0]-ctr[0])<r*1.5)&(v[:,2]>ctr[2]+r*0.95)&(v[:,2]<ctr[2]+r*1.55)&(v[:,1]<ctr[1]+r*0.9)
        c[brow]=BROW
    mid=(cL+cR)/2; mz=mid[2]-dist*1.08
    fronty=np.array([v[head&(np.abs(v[:,2]-mz)<dist*0.25)&(np.abs(v[:,0]-mid[0])<dist*0.1),1].min()])[0]
    lips=head&(np.abs(v[:,0]-mid[0])<dist*0.42)&(np.abs(v[:,2]-mz)<dist*0.12)&(v[:,1]<fronty+dist*0.22)
    c[lips]=LIPS
    return np.power(c,2.2) if linear else c
