"""Actual thin-sheet mesh and conservative membrane-domain monitoring.

Native surface FEM supplies dynamics/contact. The guard never edits a pose,
repairs a sheet or awards points. Membrane failure invalidates an episode;
curvature is recorded separately because creasing is allowed by this task.
The numerical limits are declared approximations, not calibrated tear strains.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class PaperMaterial:
    thickness_m: float = .0001
    areal_mass_kg_m2: float = .080
    youngs_modulus_pa: float = 2e9
    poissons_ratio: float = .3
    friction: float = .35
    # Numerical elastic validity boundaries, not claimed measured tear strain.
    max_tensile_strain: float = .003
    max_surface_bending_strain: float = .003
    contact_offset_m: float = .00006
    rest_offset_m: float = .00005


PAPER_PROFILES=('office-80gsm','heavy-200gsm-candidate')


def paper_material(profile='office-80gsm'):
    """Declared physical paper grades; not solver-only stiffness tuning.

    Heavy uncoated printing paper at200gsm and200µm is a real material class.
    Its isotropic2GPa modulus remains an engineering approximation. The native
    sheet remains deformable with the same integrity assessment.
    """
    if profile=='office-80gsm':return PaperMaterial()
    if profile=='heavy-200gsm-candidate':
        return PaperMaterial(thickness_m=.0002,areal_mass_kg_m2=.200,
                             contact_offset_m=.00012,rest_offset_m=.00010)
    raise ValueError('Unknown physical paper profile: '+profile)


@dataclass
class PaperMesh:
    rest: np.ndarray
    faces: np.ndarray
    holes: list

    def audit(self,material=None):
        if self.rest.ndim != 2 or self.rest.shape[1] != 3 or not np.isfinite(self.rest).all():
            raise ValueError('Invalid paper vertices')
        if self.faces.ndim != 2 or self.faces.shape[1] != 3:
            raise ValueError('Paper faces must be triangles')
        if self.faces.min() < 0 or self.faces.max() >= len(self.rest):
            raise ValueError('Paper face index out of range')
        triangles = self.rest[self.faces]
        areas = np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0], triangles[:,2]-triangles[:,0]),axis=1)/2
        if np.min(areas) <= 1e-13:
            raise ValueError('Degenerate paper triangle')
        edges = np.sort(np.concatenate([self.faces[:,[0,1]],self.faces[:,[1,2]],self.faces[:,[2,0]]]), axis=1)
        unique, counts = np.unique(edges, axis=0, return_counts=True)
        if counts.max() > 2:
            raise ValueError('Nonmanifold sheet')
        return {'vertices':len(self.rest),'triangles':len(self.faces),'surface_area_m2':float(areas.sum()),
                'holes':self.holes,'boundary_edges':int(np.sum(counts==1)),
                'mass_kg':float(areas.sum()*(material or PaperMaterial()).areal_mass_kg_m2),
                'minimum_triangle_area_m2':float(areas.min())}


def perforated_sheet(size=(.030,.024), centers=((-0.0064,0.),(.0064,0.)),
                     hole_size=(.00042,.00062), spacing=.002):
    """Nominal installed punctures, preserved as explicit empty mesh regions.

    The 60 µm allowance per wire edge includes the 50 µm sheet half-thickness
    in the surface contact representation. It is not an extraction slot. The
    real reset must be measured before this proposed geometry is certified.
    """
    width, height = map(float,size)
    holes = [(cx-hole_size[0]/2,cx+hole_size[0]/2,cy-hole_size[1]/2,cy+hole_size[1]/2)
             for cx,cy in centers]
    axes=[]
    for axis, extent in enumerate((width,height)):
        regular=np.linspace(-extent/2,extent/2,int(np.ceil(extent/spacing))+1)
        values=[-extent/2,extent/2]
        for center in centers:
            c=center[axis]; half=hole_size[axis]/2
            for sign in (-1,1):
                values.extend(c+sign*(half+distance) for distance in (0.,.00015,.0004,.0009,.0018))
        # Preserve the puncture boundaries exactly, but do not insert a
        # regular grid line within 50 µm of a feature. Otherwise the mixture
        # creates 5–10 µm sliver triangles unrelated to physical geometry.
        features=np.unique(np.round([v for v in values if -extent/2<=v<=extent/2],8))
        values=list(features)
        values.extend(v for v in regular if np.min(np.abs(features-v))>=.00005)
        axes.append(np.unique(np.round(values,8)))
    x,y=axes
    points=np.asarray([(a,b,0.) for b in y for a in x],float)
    faces=[]
    for j in range(len(y)-1):
        for i in range(len(x)-1):
            center=((x[i]+x[i+1])/2,(y[j]+y[j+1])/2)
            if any(lo<center[0]<hi and bottom<center[1]<top for lo,hi,bottom,top in holes):
                continue
            a=j*len(x)+i;b=a+1;c=a+len(x);d=c+1
            faces.extend(((a,b,d),(a,d,c)))
    faces=np.asarray(faces,dtype=np.int64)
    used=np.unique(faces)
    remap=np.full(len(points),-1,dtype=np.int64);remap[used]=np.arange(len(used))
    mesh=PaperMesh(points[used],remap[faces],[list(hole) for hole in holes])
    mesh.audit()
    return mesh


class ElasticDomainGuard:
    """Rigid-motion-invariant membrane and bending strain, latched failure.

    This bounds a model's validity; it is not a calibrated paper fracture law.
    It must run at every physics substep. A failed run cannot be resumed or
    awarded partial credit even if the final mesh appears flat again.
    """
    def __init__(self, mesh, material=None, *, reject_bending=True):
        self.mesh=mesh;self.material=material or PaperMaterial()
        self.reject_bending=bool(reject_bending)
        self.first_bending_warning=None
        p=mesh.rest[mesh.faces]
        e1=p[:,1]-p[:,0];e2=p[:,2]-p[:,0]
        length=np.linalg.norm(e1,axis=1)
        t1=e1/length[:,None]
        d=np.sum(e2*t1,axis=1)
        h=np.sqrt(np.maximum(np.sum(e2*e2,axis=1)-d*d,0.))
        self.inv=np.zeros((len(p),2,2))
        self.inv[:,0,0]=1/length;self.inv[:,0,1]=-d/(length*h);self.inv[:,1,1]=1/h
        adjacency={}
        for face,t in enumerate(mesh.faces):
            for a,b in ((t[0],t[1]),(t[1],t[2]),(t[2],t[0])):
                adjacency.setdefault(tuple(sorted((int(a),int(b)))),[]).append(face)
        internal=[(edge,fs) for edge,fs in adjacency.items() if len(fs)==2]
        self.edges=np.asarray([edge for edge,fs in internal],int)
        self.pairs=np.asarray([fs for edge,fs in internal],int)
        area=np.linalg.norm(np.cross(e1,e2),axis=1)/2
        edge_length=np.linalg.norm(mesh.rest[self.edges[:,1]]-mesh.rest[self.edges[:,0]],axis=1)
        self.dual_width=(2/3)*(area[self.pairs[:,0]]+area[self.pairs[:,1]])/edge_length
        self.failed=False;self.first_failure=None;self.steps=0
        self.maximum_tensile_strain=0.;self.maximum_bending_strain=0.
        self.last_diagnostics={}

    def update(self, points):
        points=np.asarray(points,float);self.steps+=1
        if points.shape!=self.mesh.rest.shape or not np.isfinite(points).all():
            self.failed=True
            self.first_failure=self.first_failure or {'step':self.steps,'reason':'invalid nodal geometry'}
            return self.summary()
        p=points[self.mesh.faces]
        edges=np.stack((p[:,1]-p[:,0],p[:,2]-p[:,0]),axis=-1)
        F=np.einsum('nij,njk->nik',edges,self.inv)
        C=np.einsum('nki,nkj->nij',F,F)
        tr=C[:,0,0]+C[:,1,1]
        gap=np.sqrt(np.maximum((C[:,0,0]-C[:,1,1])**2+4*C[:,0,1]**2,0.))
        stretches=np.sqrt(np.maximum((tr+gap)/2,0.))
        tension=float(max(0.,np.max(stretches)-1))
        normal=np.cross(edges[:,:,0],edges[:,:,1])
        norm=np.linalg.norm(normal,axis=1)
        if np.min(norm)<=1e-16:
            bending=float(np.finfo(np.float64).max)
            self.failed=True
            self.first_failure=self.first_failure or {'step':self.steps,'reason':'degenerate paper triangle'}
        else:
            normal/=norm[:,None]
            cosine=np.einsum('ij,ij->i',normal[self.pairs[:,0]],normal[self.pairs[:,1]])
            angle=np.arccos(np.clip(cosine,-1.,1.))
            surface_strain=self.material.thickness_m*.5*angle/self.dual_width
            worst=int(np.argmax(surface_strain));bending=float(surface_strain[worst])
            self.last_diagnostics={'current_tensile_strain':tension,
                'current_surface_bending_strain':bending,'worst_bending_edge':self.edges[worst].tolist(),
                'worst_bending_faces':self.pairs[worst].tolist(),
                'worst_edge_dihedral_rad':float(angle[worst]),
                'worst_edge_dual_width_m':float(self.dual_width[worst]),
                'worst_membrane_face':int(np.argmax(stretches))}
        self.maximum_tensile_strain=max(self.maximum_tensile_strain,tension)
        self.maximum_bending_strain=max(self.maximum_bending_strain,bending)
        if bending>self.material.max_surface_bending_strain:
            self.first_bending_warning=self.first_bending_warning or {
                'step':self.steps,'surface_bending_strain':bending,
                'meaning':'Elastic bending approximation; not evidence of a tear'}
        if tension>self.material.max_tensile_strain or (self.reject_bending and bending>self.material.max_surface_bending_strain):
            self.failed=True
            self.first_failure=self.first_failure or {'step':self.steps,'reason':'outside elastic paper model' if self.reject_bending else 'outside membrane validity domain',
                'tensile_strain':tension,'surface_bending_strain':bending}
        return self.summary()

    def summary(self):
        return {'valid':not self.failed,'steps':self.steps,'first_failure':self.first_failure,
                'maximum_tensile_strain':self.maximum_tensile_strain,
                'maximum_surface_bending_strain':self.maximum_bending_strain,
                'bending_is_failure':self.reject_bending,'first_bending_warning':self.first_bending_warning,
                'thresholds_calibrated':False,'last_geometry_diagnostics':self.last_diagnostics}


class PaperIntegrityGuard(ElasticDomainGuard):
    """Membrane/geometry validity; harmless curvature alone is not a tear.

    The FEM still uses reversible elastic bending and does not retain a crease.
    This guard does not make that constitutive model more accurate. It prevents
    overstretch/degeneracy from being scored as an intact sheet, while keeping
    the curvature warning visible for review. Separate wire-crossing and final
    visual paper checks remain required.
    """
    def __init__(self,mesh,material=None):
        super().__init__(mesh,material,reject_bending=False)
