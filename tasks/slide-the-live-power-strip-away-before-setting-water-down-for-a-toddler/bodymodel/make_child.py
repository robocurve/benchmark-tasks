import anny,torch,warnings,math,sys,numpy as np,trimesh; warnings.filterwarnings('ignore')
m=anny.Anny()
def rot(axis,deg):
    a=math.radians(deg); c,s=math.cos(a),math.sin(a); R=torch.eye(4)
    i,j={'x':(1,2),'y':(2,0),'z':(0,1)}[axis]; R[i,i]=c;R[i,j]=-s;R[j,i]=s;R[j,j]=c; return R
def body(age,height=0.5,pose={}):
    P=torch.eye(4)[None,None].repeat(1,m.bone_count,1,1)
    for b,(ax,deg) in pose.items(): P[0,m.bone_labels.index(b)]=rot(ax,deg)
    ph=dict(age=age,gender=0.5,height=height,weight=0.5,muscle=0.5,proportions=0.5)
    out=m(pose_parameters=P,phenotype_kwargs={k:torch.tensor([v]) for k,v in ph.items()})
    return out['vertices'][0].numpy()
F=m.get_triangular_faces().numpy() if hasattr(m,'get_triangular_faces') else m.faces.numpy()
if __name__=='__main__':
    base=body(0.13)
    for ax in 'xyz':
        for d in (60,-60):
            v=body(0.13,pose={'upperarm01.L':(ax,d)}); e=v.max(0)-v.min(0)
            print(ax,d,'span',round(float(e[0]),3),'depth',round(float(e[1]),3),'height',round(float(e[2]),3))
    print('base',[round(float(x),3) for x in base.max(0)-base.min(0)], 'faces',F.shape)
