"""True-scale pad array, open foil apertures and free framed stencil.

Native rigid geometry; the launcher installs the separate finite paste response.
Frame and squeegee dimensions are engineering specifications, not an OEM replica.
"""
import json
from pathlib import Path
import numpy as np
from surface_runtime import box,body,WORLD
DATA=json.loads((Path(__file__).parent/'board-geometry.json').read_text())
BODIES=['pcb_stencil','pcb_squeegee']
FIXED=['pcb','pcb_bed','stencil_support0','stencil_support1','blade_support0','blade_support1']
PCB=np.array([.33,-.065,.7797]);STENCIL=np.array([.33,.075,.793]);BLADE=np.array([.44,-.14,.8075])
TARGETS=[('pcb',PCB+[0,0,.0008]),('stencil',STENCIL),('blade',BLADE)]
COMPONENT_CONTACT=False
FIXTURE_CLAMP=False
SDF_TILES=False
EDGE_REGISTER=False

def registering_skirt(stage):
    """Passive PCB-outline skirt; gravity/contact only, with a lead-in mouth.

    The straight pocket has50um clearance on each PCB edge. Its0.5mm tapered
    entry guides placement; the1.3mm depth clears the1.6mm board's support bed.
    This is a physically modeled nominal holder, not an invisible lock.
    Fabrication tolerances and fit on a real PCB remain unverified.
    """
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,Gf,UsdShade
    import trimesh
    bx,by,_=DATA['board_bbox_m'];gap=.00005;wall=.001;lead=.0005
    for axis,half,length in [('x',bx/2+gap,by+2*(gap+wall)),('y',by/2+gap,bx+2*(gap+wall))]:
        for sign in [-1,1]:
            profile=np.array([[half,-.0000625],[half,-.0008],[half+lead,-.0013],[half+wall,-.0013],[half+wall,-.0000625]])
            vs=[]
            for across in [-length/2,length/2]:
                for radial,z in profile:vs.append([sign*radial,across,z] if axis=='x' else [across,sign*radial,z])
            m=trimesh.convex.convex_hull(np.asarray(vs));assert m.is_watertight and m.volume>0
            path=WORLD+f'/pcb_stencil/registration_{axis}_{sign+1}'
            sh=UsdGeom.Mesh.Define(stage,path);sh.CreatePointsAttr(m.vertices.tolist());sh.CreateFaceVertexCountsAttr([3]*len(m.faces));sh.CreateFaceVertexIndicesAttr(m.faces.reshape(-1).tolist());sh.CreateSubdivisionSchemeAttr('none');sh.CreateDisplayColorAttr([Gf.Vec3f(.28,.31,.34)])
            prim=sh.GetPrim();UsdPhysics.CollisionAPI.Apply(prim);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
            pc=PhysxSchema.PhysxCollisionAPI.Apply(prim);pc.CreateContactOffsetAttr(.00001);pc.CreateRestOffsetAttr(0.)
            mat=UsdShade.Material.Define(stage,path+'/material');pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(.35);pm.CreateDynamicFrictionAttr(.25)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,materialPurpose='physics')

def query_native_sdf(sim,out,device):
    """Archive cooked collision queries, not just visible triangle meshes."""
    import torch
    from shapely.geometry import box as polybox,Point
    from shapely.ops import unary_union
    holes=unary_union([polybox(p['xy_m'][0]-p['size_m'][0]/2,p['xy_m'][1]-p['size_m'][1]/2,p['xy_m'][0]+p['size_m'][0]/2,p['xy_m'][1]+p['size_m'][1]/2) for p in DATA['pads']])
    points=[];labels=[]
    for i,p in enumerate(DATA['pads']):
        for z in [-.0002,0,.0002]:points.append([*p['xy_m'],z]);labels.append(f'aperture{i}:{z}')
    for x in np.linspace(-.04,.04,7):
        for y in np.linspace(-.035,.035,7):
            if holes.distance(Point(x,y))<.0005:continue
            for z in [-.0002,0,.0002]:points.append([x,y,z]);labels.append(f'solid:{x}:{y}:{z}')
    a=np.asarray(points,dtype=np.float32)
    paths=[WORLD+f'/pcb_stencil/sdf_{x}_{y}' for x in range(4) for y in range(4)]
    # Installed tensor binding accepts a string pattern, despite newer docs
    # advertising list support. Two underscores exclude sdf_friction material.
    q=sim.create_sdf_shape_view(WORLD+'/pcb_stencil/sdf_*_*',len(a));assert q.count==16
    # Save both possible coordinate conventions; known inside/outside controls
    # determine which one is used by this installed native API.
    results={}
    for name,ps in [('local',a),('world',a+STENCIL.astype(np.float32))]:
        pts=torch.tensor(np.broadcast_to(ps,(16,*ps.shape)).copy(),device=device)
        results[name]=q.get_sdf_and_gradients(pts).cpu().numpy().copy()
    np.savez_compressed(out/'native-sdf-query.npz',points=a,labels=labels,paths=list(q.object_paths),**results)

def sdf_foil(stage):
    """Sixteen closed true-thickness tiles; real apertures remain open.

    Tiling limits SDF voxel spacing to45um at resolution512, without thickening
    the125um foil. The static load and aperture queries must validate this mode.
    """
    from shapely.geometry import box as polybox
    from shapely.geometry.polygon import orient
    from shapely.ops import unary_union,triangulate
    import trimesh
    from pxr import UsdGeom,UsdPhysics,PhysxSchema,UsdShade,Gf
    holes=unary_union([polybox(p['xy_m'][0]-p['size_m'][0]/2,p['xy_m'][1]-p['size_m'][1]/2,p['xy_m'][0]+p['size_m'][0]/2,p['xy_m'][1]+p['size_m'][1]/2) for p in DATA['pads']])
    sheet=polybox(-.046,-.039,.046,.039).difference(holes)
    mat=UsdShade.Material.Define(stage,WORLD+'/pcb_stencil/sdf_friction')
    phys=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());phys.CreateStaticFrictionAttr(.5);phys.CreateDynamicFrictionAttr(.4);phys.CreateRestitutionAttr(0.)
    for ix,(x0,x1) in enumerate(zip(np.linspace(-.046,.046,5)[:-1],np.linspace(-.046,.046,5)[1:])):
        for iy,(y0,y1) in enumerate(zip(np.linspace(-.039,.039,5)[:-1],np.linspace(-.039,.039,5)[1:])):
            # Overlap only internal tile edges; every tile is still clipped to
            # the same true sheet/holes. This avoids micron cracks at SDF seams.
            part=sheet.intersection(polybox(x0-.0001,y0-.0001,x1+.0001,y1+.0001));vs=[];fs=[];h=DATA['foil_thickness_m']/2
            def face(ps):
                start=len(vs);vs.extend(ps)
                for k in range(1,len(ps)-1):fs.append([start,start+k,start+k+1])
            for p in getattr(part,'geoms',[part]):
                p=orient(p,sign=1.)
                for t in triangulate(p):
                    if not p.covers(t):continue
                    xy=list(t.exterior.coords)[:-1];face([[x,y,h] for x,y in xy]);face([[x,y,-h] for x,y in xy[::-1]])
                for ring in [p.exterior,*p.interiors]:
                    xy=list(ring.coords)
                    for (x,y),(xx,yy) in zip(xy[:-1],xy[1:]):face([[x,y,-h],[xx,yy,-h],[xx,yy,h],[x,y,h]])
            mesh=trimesh.Trimesh(vertices=vs,faces=fs,process=True)
            assert mesh.is_watertight and mesh.is_winding_consistent
            assert abs(mesh.volume-part.area*2*h)<1e-12
            path=WORLD+f'/pcb_stencil/sdf_{ix}_{iy}'
            m=UsdGeom.Mesh.Define(stage,path);m.CreatePointsAttr(mesh.vertices.tolist());m.CreateFaceVertexIndicesAttr(mesh.faces.reshape(-1).tolist());m.CreateFaceVertexCountsAttr([3]*len(mesh.faces));m.CreateSubdivisionSchemeAttr('none');m.CreateDisplayColorAttr([Gf.Vec3f(.4,.43,.46)])
            prim=m.GetPrim();UsdPhysics.CollisionAPI.Apply(prim);UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('sdf')
            sdf=PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim);sdf.CreateSdfResolutionAttr(512);sdf.CreateSdfSubgridResolutionAttr(6);sdf.CreateSdfNarrowBandThicknessAttr(.02)
            pc=PhysxSchema.PhysxCollisionAPI.Apply(prim);pc.CreateContactOffsetAttr(.00002);pc.CreateRestOffsetAttr(0.)
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,materialPurpose='physics')

def component_contact_fixture():
    global STENCIL,COMPONENT_CONTACT
    STENCIL=PCB+np.array([0,0,.0008+DATA['foil_thickness_m']/2]);COMPONENT_CONTACT=True

def build(stage):
    box(stage,WORLD+'/pcb_bed',[PCB[0],PCB[1],.7632],[.075,.060,.0314],[.25,.28,.3])
    # Fixed PCB is supported on a normal printing bed; pad centroids/size are
    # taken from the original design, never magnified for the camera.
    box(stage,WORLD+'/pcb',PCB,DATA['board_bbox_m'],[.1,.36,.23])
    for i,p in enumerate(DATA['pads']):
        box(stage,WORLD+f'/pcb_pad_{i}',PCB+[*p['xy_m'],.000805],[*p['size_m'],.00001],[.9,.72,.3],False)
    for i,y in enumerate([-.035,.035]):
        # The ordinary separate rest remains in the scene for the pre-aligned
        # component fixture, but does not support or drive the printing tool.
        box(stage,WORLD+f'/stencil_support{i}',[.33,.075+y,.76875],[.10,.008,.0425],[.3,.33,.37])
    body(stage,WORLD+'/pcb_stencil',STENCIL,.055)
    if EDGE_REGISTER:registering_skirt(stage)
    if SDF_TILES:sdf_foil(stage)
    else:
        for i,(x,y,w,h) in enumerate(DATA['foil_rectangles']):
            box(stage,WORLD+f'/pcb_stencil/foil{i}',[x,y,0],[w,h,DATA['foil_thickness_m']],[.65,.68,.7])
    for i,(c,s) in enumerate([([-.049,0,.002],[.006,.09,.006]),([.049,0,.002],[.006,.09,.006]),([0,-.042,.002],[.092,.006,.006]),([0,.042,.002],[.092,.006,.006])]):
        box(stage,WORLD+f'/pcb_stencil/frame{i}',c,s,[.24,.28,.32])
    if FIXTURE_CLAMP:
        from pxr import UsdPhysics,Gf
        for i in range(2):
            name=f'clamp_visible_{i}'
            if name not in FIXED:FIXED.append(name)
        joint=UsdPhysics.FixedJoint.Define(stage,WORLD+'/diagnostic_fixture_clamp')
        joint.CreateBody1Rel().SetTargets([WORLD+'/pcb_stencil'])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float,STENCIL)))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
        # Test-fixture restraint is present before reset. This is not a robot
        # clamp/registration demonstration and is recorded in configuration.json.
        for i,x in enumerate([-.049,.049]):
            box(stage,WORLD+f'/clamp_visible_{i}',STENCIL+[x,-.025,.009],[.012,.012,.007],[.23,.35,.48])
    for i,dy in enumerate([-.0065,.0065]):
        box(stage,WORLD+f'/blade_support{i}',[BLADE[0],BLADE[1]+dy,.7725],[.018,.004,.05],[.3,.33,.37])
    body(stage,WORLD+'/pcb_squeegee',BLADE,.065)
    box(stage,WORLD+'/pcb_squeegee/handle',[0,0,0],[.025,.018,.020],[.7,.32,.12])
    box(stage,WORLD+'/pcb_squeegee/steel',[0,0,-.020],[.080,.0005,.025],[.7,.73,.75])
