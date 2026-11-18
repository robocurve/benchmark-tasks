"""Finite-volume, pressure-dependent printing through actual stencil apertures.

Explained material-response approximation, not calibrated solder rheology. Native
rigid spheres separately validate hole passage. Here soft material is subdivided
into finite volume elements so it can fill, adhere and release rather than remain
dry marbles. Rigid tools/frame never receive scripted transforms or attachments.
"""
import numpy as np
from scipy.spatial.transform import Rotation

class StencilPaste:
    kind='stencil_paste'
    def __init__(self,data,volume_ul=120.):
        self.data=data;xy=[];area=[];pad=[];width=[]
        for k,p in enumerate(data['pads']):
            w,h=np.array(p['size_m'])*1000;nx,ny=np.maximum(1,np.ceil(np.array([w,h])/.15).astype(int))
            x,y=np.array(p['xy_m'])*1000
            for dx in (np.arange(nx)+.5)*w/nx-w/2:
                for dy in (np.arange(ny)+.5)*h/ny-h/2:xy.append([x+dx,y+dy]);area.append(w*h/nx/ny);pad.append(k);width.append([w/nx,h/ny])
        self.xy=np.asarray(xy);self.area=np.asarray(area);self.pad=np.asarray(pad);self.width=np.asarray(width)
        self.capacity=self.area*.125;self.fill=np.zeros(len(xy));self.deposit=np.zeros(len(xy));self.retained=np.zeros(len(xy));self.deposit_xy=np.zeros((len(xy),2))
        self.bins=np.arange(-38,38.001,.5);self.reservoir=np.full(len(self.bins),volume_ul/len(self.bins));self.roll_y=np.full(len(self.bins),-24.)
        self.initial=float(volume_ul);self.lost=0.;self.released=False;self.previous=None;self.normal_force=0.;self.fill_poses=np.zeros((len(xy),2));self.cumulative_smear_mm=0.
        self.cell_bin=np.clip(np.rint((self.xy[:,0]+38)/.5).astype(int),0,len(self.bins)-1)
        self.last_stencil=np.array([.33,.075,.793,0,0,0,1.]);self.metrics_cache=None;self.previous_world_edge=None;self.previous_line=None
    def step(self,stencil,blade,normal_force,dt,board_force=0.):
        s=np.asarray(stencil);b=np.asarray(blade);sr=Rotation.from_quat(s[3:]);br=Rotation.from_quat(b[3:]);self.last_stencil=s.copy()
        corners=sr.inv().apply(br.apply([[-.04,-.00025,-.0325],[.04,-.00025,-.0325],[-.04,.00025,-.0325],[.04,.00025,-.0325]])+b[:3]-s[:3])*1000
        edge=corners[:2] if corners[:2,2].mean()<corners[2:,2].mean() else corners[2:]
        world_edge=sr.apply(edge/1000)+s[:3]
        old_world=self.previous_world_edge;self.previous_world_edge=world_edge.mean(0).copy()
        if self.released and board_force>1e-6 and old_world is not None:
            a,e=world_edge[:,:2]*1000;v=e-a
            u=np.clip((self.deposit_xy-a)@v/max(v@v,1e-12),0,1)
            distance=np.linalg.norm(self.deposit_xy-(a+u[:,None]*v),axis=1)
            z=world_edge[0,2]+u*(world_edge[1,2]-world_edge[0,2])
            hit=(distance<.5)&(self.deposit>0)&(z<=.7805+self.deposit/self.area/1000+.0001)
            # A contacting scraper can move previously printed material too;
            # the printed result is not frozen after first removal of the foil.
            self.deposit_xy[hit]+=(self.previous_world_edge-old_world)[:2]*1000
            if hit.any():self.metrics_cache=None
        lo,hi=sorted(edge[:,0]);self.normal_force=max(0.,float(normal_force))
        old=self.previous;self.previous=edge.mean(0)
        horizontal_shift=0. if old is None else float(np.linalg.norm(self.previous[:2]-old[:2]))
        dx=edge[1,0]-edge[0,0]
        yline=edge[0,1]+(self.bins-edge[0,0])*(edge[1,1]-edge[0,1])/dx if abs(dx)>1e-6 else np.full_like(self.bins,1e6)
        zline=edge[0,2]+(self.bins-edge[0,0])*(edge[1,2]-edge[0,2])/dx if abs(dx)>1e-6 else np.full_like(self.bins,1e6)
        radius=np.cbrt(self.reservoir*3/(4*np.pi))
        loaded=(self.bins>=lo)&(self.bins<=hi)&(zline<.15)&(zline>-.3)&(self.normal_force>.01)
        if self.previous_line is not None:
            delta=yline-self.previous_line
            # Swept-front contact, not a sign-triggered jump to either side of
            # the knife. Tiny native jitter cannot throw the bead1.3mm behind
            # the blade. A blade already ahead of a bead cannot pull it along.
            forward=loaded&(delta>0)&(self.previous_line<=self.roll_y)&(yline+radius>=self.roll_y)
            reverse=loaded&(delta<0)&(self.previous_line>=self.roll_y)&(yline-radius<=self.roll_y)
            self.roll_y[forward]=np.maximum(self.roll_y[forward],yline[forward]+radius[forward])
            self.roll_y[reverse]=np.minimum(self.roll_y[reverse],yline[reverse]-radius[reverse])
        self.previous_line=yline.copy()
        touched=loaded&(np.abs(yline-self.roll_y)<=radius+.10)
        xyz=sr.apply(np.c_[self.xy/1000,np.full(len(self.xy),-.0000625)])+s[:3]
        gap=(xyz[:,2]-.7805)*1000
        seated=(gap>=-.10)&(gap<=.20)
        if not self.released and self.normal_force>1.6:
            active=touched[self.cell_bin]&(np.abs(self.xy[:,1]-self.roll_y[self.cell_bin])<=radius[self.cell_bin])&seated
            # Yield load20N/m across80mm; characteristic fill time30ms at8N.
            # These are disclosed engineering coefficients, not a fit to score.
            rate=(self.normal_force-1.6)/6.4/.03
            requested=np.minimum(self.capacity-self.fill,(self.capacity-self.fill)*(1-np.exp(-rate*dt)))*active
            demand=np.bincount(self.cell_bin,weights=requested,minlength=len(self.bins))
            scale=np.minimum(1.,np.divide(self.reservoir,demand,out=np.ones_like(demand),where=demand>0))
            transfer=requested*scale[self.cell_bin];self.reservoir-=np.bincount(self.cell_bin,weights=transfer,minlength=len(self.bins))
            self.fill+=transfer
        # Keep the last actual near-board location of occupied cavities. A
        # centre-height test would prematurely release all paste when just one
        # side of a tilted stencil lifts during a stroke.
        near=seated&(self.fill>0)
        self.fill_poses[near]=xyz[near,:2]*1000
        if not self.released and self.fill.sum()>1e-8 and gap.min()>.30:
            # Adhesion/release: high aperture area-to-wall ratio retains less
            # material on foil. No deposition is possible from an empty cavity.
            w=np.array([p['size_m'] for p in self.data['pads']])*1000
            ratio=np.prod(w,axis=1)/(2*w.sum(1)*.125)
            efficiency=.95*np.minimum(1.,ratio[self.pad]/.66)
            self.deposit=self.fill*efficiency;self.retained=self.fill-self.deposit;self.fill*=0
            self.deposit_xy=self.fill_poses.copy();self.released=True;self.metrics_cache=None
        self.check()
    def check(self):
        arrays=[self.fill,self.deposit,self.retained,self.reservoir]
        assert all(np.isfinite(a).all() and a.min()>=-1e-8 for a in arrays)
        assert abs(sum(a.sum() for a in arrays)+self.lost-self.initial)<1e-7
    def metrics(self):
        if self.metrics_cache is not None:return self.metrics_cache
        coverage=np.zeros(len(self.data['pads']));on_area=0.;total_area=0.;bridges=0
        if np.any(self.deposit):
            from shapely.geometry import box
            from shapely.ops import unary_union
            from shapely.strtree import STRtree
            pads=[]
            for p in self.data['pads']:
                c=np.array(p['xy_m'])*1000+[330,-65];h=np.array(p['size_m'])*500
                pads.append(box(*(c-h),*(c+h)))
            # The same visible-deposit cutoff is used by the native material mesh.
            # Grade planar coverage from the union, never sum overlapping footprints
            # or require that paste came from a particular aperture.
            mask=self.deposit>=.010*self.area
            polys=[box(*(c-w/2),*(c+w/2)) for c,w in zip(self.deposit_xy[mask],self.width[mask])]
            union=unary_union(polys)
            coverage=np.array([union.intersection(p).area/p.area for p in pads])
            on_area=union.intersection(unary_union(pads)).area;total_area=union.area
            bridges=0
            if self.released and not union.is_empty:
                tree=STRtree(pads)
                # A nanometre numerical join only removes floating-point seams
                # between adjacent material cells; real inter-pad gaps remain open.
                connected=union.buffer(.000001)
                for part in getattr(connected,'geoms',[connected]):
                    touching=[int(j) for j in tree.query(part) if part.intersection(pads[int(j)]).area>.01]
                    bridges+=len(set(touching))>1
        m=dict(pad_coverage=coverage.tolist(),all_pads_nonempty=bool(np.all(coverage>=.5)),qualified_pad_fraction=float(np.mean(coverage>=.75)),mean_pad_coverage=float(coverage.mean()),on_pad_fraction=float(on_area/total_area) if total_area else 0.,bridges=int(bridges),deposited_ul=float(self.deposit.sum()),deposited_mass_mg=float(self.deposit.sum()*4.2),reservoir_ul=float(self.reservoir.sum()),in_apertures_ul=float(self.fill.sum()),retained_ul=float(self.retained.sum()),released=self.released,normal_force_N=self.normal_force,conservation_error_ul=float(abs(self.fill.sum()+self.deposit.sum()+self.retained.sum()+self.reservoir.sum()+self.lost-self.initial)))
        if self.released:self.metrics_cache=m
        return m

class PasteDisplay:
    def __init__(self,stage,path,model):
        from pxr import UsdGeom,UsdShade,Sdf,Gf
        self.model=model;self.roll=UsdGeom.Points.Define(stage,path+'_bead');self.roll.CreateDisplayColorAttr([Gf.Vec3f(.3,.32,.34)])
        self.parts=[]
        for name in ['foil','board']:
            mesh=UsdGeom.Mesh.Define(stage,path+'_'+name);mesh.CreateSubdivisionSchemeAttr('none');mesh.CreateDoubleSidedAttr(True)
            mat=UsdShade.Material.Define(stage,path+'_'+name+'/appearance');s=UsdShade.Shader.Define(stage,path+'_'+name+'/appearance/surface');s.CreateIdAttr('UsdPreviewSurface');s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.3,.32,.34));s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.3);mat.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat);self.parts.append(mesh)
    def update(self,base_pose=None):
        m=self.model;s=m.last_stencil;r=Rotation.from_quat(s[3:])
        radius=np.cbrt(m.reservoir*3/(4*np.pi));points=np.c_[m.bins,m.roll_y,radius+.0625]/1000
        self.roll.GetPointsAttr().Set((r.apply(points)+s[:3]).tolist());self.roll.GetWidthsAttr().Set((radius*2/1000).tolist())
        for name,mesh,volume in [('foil',self.parts[0],m.fill+m.retained),('board',self.parts[1],m.deposit)]:
            mask=volume>=m.area*(.010 if name=='board' else .001);h=volume[mask]/m.area[mask]
            if name=='foil':xyz=np.c_[m.xy[mask],np.full(mask.sum(),.0625)+h]
            else:xyz=np.c_[m.deposit_xy[mask],np.full(mask.sum(),780.5)+h]
            half=m.width[mask]/2;corners=np.stack([-half,np.c_[half[:,0],-half[:,1]],half,np.c_[-half[:,0],half[:,1]]],1)
            q=np.repeat(xyz[:,None,:],4,1);q[:,:,:2]+=corners;q/=1000
            if name=='foil':q=r.apply(q.reshape(-1,3)).reshape(-1,4,3)+s[:3]
            mesh.GetPointsAttr().Set(q.reshape(-1,3).tolist());mesh.GetFaceVertexCountsAttr().Set([4]*len(q));mesh.GetFaceVertexIndicesAttr().Set(np.arange(len(q)*4).tolist())
