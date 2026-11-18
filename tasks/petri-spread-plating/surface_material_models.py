"""Conservative reduced material responses driven by recorded physical contact.

These are task-local approximations, not calibrated CFD or biological growth.
They never move a robot, tool or workpiece. No time, action label, target pose,
waypoint or arm identity is an input to material transport.
"""
import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation


class WetFilm:
    """Finite-volume squeezing of a drop beneath the measured cover plane.

    A 20 um regularized film gap represents unresolved thin-film compliance.
    Excess over that local gap moves to adjacent cells, conserving water.
    Enclosed air remains a void until a vent is reopened by lifting the glass.
    No blanket failure/success is assigned to flat/tilted placement.
    """
    kind='wet_mount'
    visible_depth_mm=.012
    def __init__(self,volume_ul=10.,dx_mm=.5):
        self.dx=dx_mm;self.area=self.dx**2
        self.x=np.arange(-16+self.dx/2,16,self.dx);self.y=np.arange(-12.5+self.dx/2,12.5,self.dx)
        self.xx,self.yy=np.meshgrid(self.x,self.y,indexing='ij')
        self.xy=np.stack([self.xx,self.yy],-1)
        w=np.maximum(0,1-(self.xx**2+self.yy**2)/2.5**2)
        self.water=w/w.sum()*volume_ul;self.initial=float(volume_ul);self.lost=0.
        self.trapped=np.zeros_like(w,bool);self.covered=np.zeros_like(w,bool)
        self.gap=np.full_like(w,100.);self.last_contact=False;self.steps=0
    def step(self,cover_pose,dt):
        p=np.asarray(cover_pose);rot=Rotation.from_quat(p[3:]).as_matrix();n=rot[:,2].copy()
        # Both faces of a plain, uncoated coverslip are equivalent.
        if n[2]<0:n=-n
        world=np.dstack([self.xx/1000+.33,self.yy/1000+.025,np.full_like(self.xx,.7775)])
        gap=(p[2]-.000075*n[2]-.7775-(n[0]*(world[...,0]-p[0])+n[1]*(world[...,1]-p[1]))/max(n[2],.2))*1000
        intersection=world.copy();intersection[...,2]+=(gap/1000)
        local=(intersection-p[:3])@rot
        self.covered=(np.abs(local[...,0])<=.011)&(np.abs(local[...,1])<=.011)&(n[2]>.2)
        self.gap=np.where(self.covered,np.maximum(.020,gap),100.)
        # An air pocket may vent after a physically opened gap, not after a timer.
        self.trapped&=self.covered&(self.gap<.10)
        # Test air connectivity before squeeze redistribution. A pre-existing
        # liquid ring can seal air as the physical cover approaches it.
        air=self.water<self.visible_depth_mm*self.area
        seeds=air&((~self.covered)|(self.gap>=.10));seeds[[0,-1],:]|=air[[0,-1],:];seeds[:,[0,-1]]|=air[:,[0,-1]]
        self.trapped|=air&~ndimage.binary_propagation(seeds,mask=air)&self.covered&(self.gap<.10)
        capacity=self.gap*self.area
        self.last_contact=bool(np.any(self.water>capacity+1e-12))
        for _ in range(max(1,min(80,round(dt/.0004)))):
            excess=np.maximum(self.water-capacity,0)
            if excess.max()<1e-10:break
            available=~self.trapped
            degree=np.zeros_like(self.water)
            for axis in [0,1]:
                for shift in [-1,1]:
                    target=np.roll(available,shift,axis);sl=[slice(None)]*2;sl[axis]=0 if shift==1 else -1;target[tuple(sl)]=False;degree+=target
            portion=np.divide(.8*excess,degree,out=np.zeros_like(excess),where=degree>0)
            delta=np.zeros_like(self.water)
            for axis in [0,1]:
                for shift in [-1,1]:
                    allowed=np.roll(available,shift,axis);sl=[slice(None)]*2;sl[axis]=0 if shift==1 else -1;allowed[tuple(sl)]=False
                    transfer=portion*allowed;delta-=transfer;delta+=np.roll(transfer,-shift,axis)
            self.water+=delta
        wet=self.water>=self.visible_depth_mm*self.area
        air=~wet
        seeds=air&((~self.covered)|(self.gap>=.10));seeds[[0,-1],:]|=air[[0,-1],:];seeds[:,[0,-1]]|=air[:,[0,-1]]
        vented=ndimage.binary_propagation(seeds,mask=air)
        self.trapped|=air&~vented&self.covered&(self.gap<.10)
        self.steps+=1;self.check()
    def check(self):
        assert np.isfinite(self.water).all() and self.water.min()>=-1e-10
        assert abs(self.water.sum()+self.lost-self.initial)<1e-7
    def metrics(self):
        roi=(np.abs(self.xx)<=6)&(np.abs(self.yy)<=6)
        wet=self.water>=self.visible_depth_mm*self.area
        labels,n=ndimage.label(self.trapped&roi);areas=np.bincount(labels.ravel())[1:]*self.area
        return dict(wet_specimen_fraction=float((wet&self.covered&roi).sum()/roi.sum()),largest_trapped_air_mm2=float(areas.max(initial=0)),water_ul=float(self.water.sum()),conservation_error_ul=float(abs(self.water.sum()+self.lost-self.initial)),wet_area_mm2=float(wet.sum()*self.area),trapped_area_mm2=float(self.trapped.sum()*self.area))


class SpreadFilm:
    """Finite liquid carried and redistributed by the actual contacting T-bar.

    Liquid pools exchange with a finite tool film; empty tool passes do not paint.
    A purely rigid substrate cannot establish biological damage. The separate
    geometrical indentation witness is disclosed as an approximate damage rule.
    """
    kind='spread_plate'
    visible_depth_mm=.005
    def __init__(self,volume_ul=100.,dx_mm=1.):
        self.dx=dx_mm;self.area=self.dx**2
        self.x=np.arange(-43.5+dx_mm/2,43.5,dx_mm);self.y=self.x.copy()
        self.xx,self.yy=np.meshgrid(self.x,self.y,indexing='ij');self.xy=np.stack([self.xx,self.yy],-1)
        self.domain=self.xx**2+self.yy**2<=43.4**2
        w=np.maximum(0,1-(self.xx**2+self.yy**2)/4.**2)
        self.water=w/w.sum()*volume_ul;self.initial=float(volume_ul);self.carried=0.;self.lost=0.
        self.damage=np.zeros_like(w);self.previous=None;self.swept_mm=0.;self.last_contact=False
    def step(self,base_pose,tool_pose,contact_force,dt):
        base=np.asarray(base_pose);tool=np.asarray(tool_pose);br=Rotation.from_quat(base[3:]);tr=Rotation.from_quat(tool[3:])
        gravity=br.inv().apply([0.,0.,-9.81])
        if gravity[2]>=0:
            self.lost+=float(self.water.sum());self.water*=0
        elif np.linalg.norm(gravity[:2])>.0981:
            # Reduced thin-film gravity flux, with a small pinning threshold.
            # Water kinematic viscosity is1e-6m²/s; cap the numerical transfer
            # per step for positivity. This is not calibrated agar wettability.
            h=np.minimum(self.water/self.area,.4)/1000
            velocity=h[...,None]**2*gravity[:2]/(3e-6)*1000
            fraction=np.minimum(.8,np.abs(velocity).sum(-1)*dt/self.dx)
            total=np.abs(velocity).sum(-1)
            old=self.water.copy();delta=np.zeros_like(old)
            for axis in [0,1]:
                part=old*fraction*np.divide(abs(velocity[...,axis]),total,out=np.zeros_like(old),where=total>0)
                sign=1 if gravity[axis]>0 else -1
                target=np.roll(part,sign,axis);edge=[slice(None)]*2;edge[axis]=0 if sign==1 else -1
                self.lost+=float(target[tuple(edge)].sum());target[tuple(edge)]=0
                self.lost+=float(target[~self.domain].sum());target[~self.domain]=0
                delta+=target-part
            self.water+=delta
        ends=tr.apply([[-.0175,.068,0],[.0175,.068,0]])+tool[:3]
        ends=br.inv().apply(ends-base[:3])*1000
        a,b=ends[:,:2];ab=b-a;length2=float(ab@ab)
        u=np.clip(np.sum((self.xy-a)*ab,axis=-1)/max(length2,1e-12),0,1)
        nearest=a+u[...,None]*ab;dist=np.linalg.norm(self.xy-nearest,axis=-1)
        lower=ends[0,2]+u*(ends[1,2]-ends[0,2])-2.
        footprint=(dist<=2.+self.dx*.5)&(lower<=6.05)&self.domain
        mask=footprint&(lower>=4.)
        touching=bool(footprint.any() and abs(float(contact_force))>1e-6)
        center=(a+b)/2;travel=0. if self.previous is None else float(np.linalg.norm(center-self.previous));self.previous=center
        self.last_contact=touching
        if touching:
            self.damage[footprint]=np.maximum(self.damage[footprint],np.maximum(0,6.-lower[footprint]))
            if travel>1e-5 and mask.any():
                self.swept_mm+=travel
                fraction=1-np.exp(-travel/1.)
                # Finite local exchange. Tool film capacity is10uL; excess stays
                # on the actual contact footprint, never on a target pattern.
                old=float(self.water[mask].sum());pool=old+self.carried
                carry=min(10.,pool*.15)
                target=(pool-carry)/mask.sum()
                self.water[mask]=(1-fraction)*self.water[mask]+fraction*target
                self.carried=(1-fraction)*self.carried+fraction*carry
        if not self.domain.all():self.water[~self.domain]=0.
        self.check()
    def check(self):
        assert np.isfinite(self.water).all() and self.water.min()>=-1e-10
        assert abs(self.water.sum()+self.carried+self.lost-self.initial)<1e-7
    def metrics(self):
        wet=(self.water>=self.visible_depth_mm*self.area)&self.domain
        usable=self.xx**2+self.yy**2<=40.**2
        regions=[]
        for sx in [-1,1]:
            for sy in [-1,1]:
                # The four physical rim marks define these diameters. Split
                # axis-crossing cells by area instead of counting them twice.
                wx=(self.xx*sx>0).astype(float)+.5*(self.xx==0)
                wy=(self.yy*sy>0).astype(float)+.5*(self.yy==0)
                q=usable*wx*wy;regions.append(float((wet*q).sum()/q.sum()))
        return dict(wet_fraction=float((wet&usable).sum()/usable.sum()),usable_diameter_mm=80.,whole_surface_wet_fraction=float(wet.sum()/self.domain.sum()),quadrant_wet_fraction=regions,water_on_agar_ul=float(self.water.sum()),water_on_tool_ul=float(self.carried),lost_ul=self.lost,conservation_error_ul=float(abs(self.water.sum()+self.carried+self.lost-self.initial)),max_indentation_mm=float(self.damage.max()),damage_area_mm2=float((self.damage>=.3).sum()*self.area),swept_mm=self.swept_mm)


class FilmDisplay:
    """Render the model's actual finite material field in the original cameras."""
    def __init__(self,stage,path,model):
        from pxr import UsdGeom,UsdShade,Sdf,Gf
        self.model=model;self.mesh=UsdGeom.Mesh.Define(stage,path);self.mesh.CreateSubdivisionSchemeAttr('none');self.mesh.CreateDoubleSidedAttr(True)
        mat=UsdShade.Material.Define(stage,path+'/appearance');s=UsdShade.Shader.Define(stage,path+'/appearance/surface');s.CreateIdAttr('UsdPreviewSurface');s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.025,.22,.48));s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.16);s.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(.75);mat.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(self.mesh.GetPrim()).Bind(mat)
        self.scar=None
        if model.kind=='spread_plate':
            self.scar=UsdGeom.Mesh.Define(stage,path+'_damage');self.scar.CreateSubdivisionSchemeAttr('none');self.scar.CreateDisplayColorAttr([Gf.Vec3f(.12,.06,.02)])
    def update(self,base_pose=None):
        m=self.model;mask=m.water>=m.area*m.visible_depth_mm
        x=m.xx[mask];y=m.yy[mask];half=m.dx/2
        h=np.minimum(m.water[mask]/m.area,1.)
        if m.kind=='wet_mount':
            h=np.minimum(h,m.gap[mask]);xyz=np.c_[x/1000+.33,y/1000+.025,.7775+h/1000+.000001]
        else:
            xyz=np.c_[x/1000,y/1000,.006+h/1000+.000001]
        quads=xyz[:,None,:]+np.array([[-half,-half,0],[half,-half,0],[half,half,0],[-half,half,0]])[None]/1000
        if m.kind=='spread_plate':
            if base_pose is None:base_pose=np.array([.33,.04,.7475,0,0,0,1.])
            quads=Rotation.from_quat(base_pose[3:]).apply(quads.reshape(-1,3)).reshape(-1,4,3)+base_pose[:3]
        self.mesh.GetPointsAttr().Set(quads.reshape(-1,3).tolist());self.mesh.GetFaceVertexCountsAttr().Set([4]*len(quads));self.mesh.GetFaceVertexIndicesAttr().Set(np.arange(len(quads)*4).tolist())
        if self.scar is not None:
            hit=m.damage>=.3;c=np.c_[m.xx[hit]/1000,m.yy[hit]/1000,np.full(hit.sum(),.006005)]
            q=c[:,None,:]+np.array([[-half,-half,0],[half,-half,0],[half,half,0],[-half,half,0]])[None]/1000
            q=Rotation.from_quat(base_pose[3:]).apply(q.reshape(-1,3))+base_pose[:3] if len(q) else np.empty((0,3))
            self.scar.GetPointsAttr().Set(q.tolist());self.scar.GetFaceVertexCountsAttr().Set([4]*int(len(q)/4));self.scar.GetFaceVertexIndicesAttr().Set(np.arange(len(q)).tolist())
