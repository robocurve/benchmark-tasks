"""Render the reduced model's current dimensions, never a timed animation."""
import numpy as np
from material import RADIUS,KNOT_X

class Drawing:
    def __init__(self,stage):
        from pxr import UsdGeom,UsdShade,Sdf,Gf
        self.meshes={};self.topology_written=set();self.last_points={};self.last_faces={}
        for name,color in [('wide',(.50,.06,.13)),('narrow',(.38,.035,.08)),('band',(.50,.06,.13)),('knot',(.55,.07,.15)),('stripe',(.85,.63,.18))]:
            path='/World/PreparedTie/'+name;mesh=UsdGeom.Mesh.Define(stage,path);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
            mat=UsdShade.Material.Define(stage,path+'/Material');shader=UsdShade.Shader.Define(stage,path+'/Material/Surface');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.75);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat);self.meshes[name]=mesh
    def set(self,name,points,faces):
        from pxr import Vt
        m=self.meshes[name];p=np.asarray(points,dtype=np.float32)
        if name not in self.last_points or not np.array_equal(p,self.last_points[name]):
            m.GetPointsAttr().Set(Vt.Vec3fArray(p.tolist()));self.last_points[name]=p.copy()
        if name not in self.last_faces or not np.array_equal(self.last_faces[name],faces):
            m.GetFaceVertexCountsAttr().Set([3]*len(faces));m.GetFaceVertexIndicesAttr().Set(np.asarray(faces,int).ravel().tolist());self.topology_written.add(name);self.last_faces[name]=np.asarray(faces).copy()
    def strip(self,name,centers,widths,directions):
        c=np.asarray(centers);w=np.broadcast_to(widths,(len(c),));d=np.broadcast_to(directions,c.shape);points=np.stack([c-d*w[:,None]/2,c+d*w[:,None]/2],axis=1).reshape(-1,3);f=[]
        for i in range(len(c)-1):a=2*i;f.extend([[a,a+1,a+2],[a+1,a+3,a+2]])
        self.set(name,points,f)
    def update(self,tie):
        u=np.linspace(0,1,51);direction=tie.wide_direction
        width_dir=np.array([np.sin(tie.wide_twist),np.cos(tie.wide_twist),0.]);width_dir-=width_dir.dot(direction)*direction;width_dir/=max(np.linalg.norm(width_dir),1e-9)
        wide=tie.wide_root+u[:,None]*tie.wide_length*direction if tie.threaded else tie.loose.points
        u=np.linspace(0,1,len(wide))
        widths=np.interp(u,[0.,.75,.90,1.],[.036,.066,.070,0.])
        self.strip('wide',wide,widths,width_dir)
        self.strip('stripe',wide+width_dir[None,:]*(widths[:,None]*.35)+[-.0005,0,0],.002,width_dir)
        u=np.linspace(0,1,51)
        self.strip('narrow',tie.narrow_root+u[:,None]*tie.narrow_direction*tie.narrow_length,np.interp(u,[0,.88,1],[.035,.035,0]),[0,1,0])
        # Prepared neckband: gravity drape along the collar and two straight
        # front segments. No shoulder or invisible external support is added.
        root=tie.center+[.009,0.,.020];left=np.array([.40-RADIUS/np.sqrt(2),-RADIUS/np.sqrt(2),1.25]);right=np.array([.40-RADIUS/np.sqrt(2),RADIUS/np.sqrt(2),1.25]);theta=np.linspace(-3*np.pi/4,3*np.pi/4,65)
        arc=np.c_[.40+RADIUS*np.cos(theta),RADIUS*np.sin(theta),np.full_like(theta,1.25)]
        centers=np.vstack([root+(left-root)*np.linspace(0,1,25)[:,None],arc[1:],right+(root-right)*np.linspace(0,1,25)[1:,None]])
        tangents=np.gradient(centers,axis=0);tangents/=np.maximum(np.linalg.norm(tangents,axis=1)[:,None],1e-9)
        dirs=np.tile([0.,0.,-1.],(len(centers),1));dirs-=np.sum(dirs*tangents,axis=1)[:,None]*tangents;dirs/=np.maximum(np.linalg.norm(dirs,axis=1)[:,None],1e-9)
        self.strip('band',centers,.035,dirs)
        # A conical outer wrap represents the prepared knot. Its dimensions
        # follow material fed through it; internal folds are not resolved.
        if not tie.threaded:
            # Match the collision sleeve, including its open bore and rim.
            angles=np.linspace(0,2*np.pi,65);n=len(angles);pts=[];faces=[]
            for rx,ry,z in [(.023,.043,-.028),(.023,.043,.028),(.014,.034,-.028),(.014,.034,.028)]:
                pts.extend(tie.center+np.c_[rx*np.cos(angles),ry*np.sin(angles),np.full(n,z)])
            for a,b in [(0,1),(2,3),(0,2),(1,3)]:
                for i in range(n-1):
                    j=a*n+i;k=b*n+i;faces.extend([[j,j+1,k],[j+1,k+1,k]])
            self.set('knot',pts,faces)
            return
        angles=np.linspace(0,2*np.pi,65);rx=.018+.06*tie.slack;ry=.026+.22*tie.slack;pts=[]
        for h,scale in [(-tie.half_height,.65),(tie.half_height,1.)]:
            pts.extend(tie.center+np.c_[rx*scale*np.cos(angles),ry*scale*np.sin(angles),np.full_like(angles,h)+.004*np.sin(angles)])
        n=len(angles);faces=[]
        for i in range(n-1):faces.extend([[i,i+1,i+n],[i+1,i+n+1,i+n]])
        self.set('knot',pts,faces)
