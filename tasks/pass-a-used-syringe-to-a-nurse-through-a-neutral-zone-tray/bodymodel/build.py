import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_child import *
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
