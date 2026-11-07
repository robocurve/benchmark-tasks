"""Reproducible original CAD-like meshes and open compound colliders (SI units)."""
from pathlib import Path
import json, math, hashlib
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf
from ratchet_task.config import *
ROOT=Path(__file__).resolve().parent
STEEL=(.46,.49,.53); BRACKET=(.17,.29,.40); DARK=(.12,.14,.16)
def stage(name,mass=None):
    s=Usd.Stage.CreateNew(str(ROOT/'assets'/f'{name}.usda'))
    UsdGeom.SetStageMetersPerUnit(s,1.); UsdGeom.SetStageUpAxis(s,'Z')
    p=UsdGeom.Xform.Define(s,'/Asset').GetPrim();s.SetDefaultPrim(p)
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(p);UsdPhysics.MassAPI.Apply(p).CreateMassAttr(mass)
        p.AddAppliedSchema('PhysxRigidBodyAPI');p.CreateAttribute('physxRigidBody:enableCCD',Sdf.ValueTypeNames.Bool).Set(True)
    return s

def material(s,name,rgb,metal=.0,rough=.4):
    path='/Asset/Materials/'+name
    mat=UsdShade.Material.Define(s,path);shader=UsdShade.Shader.Define(s,path+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    for key,typ,val in [('diffuseColor',Sdf.ValueTypeNames.Color3f,Gf.Vec3f(*rgb)),('metallic',Sdf.ValueTypeNames.Float,metal),('roughness',Sdf.ValueTypeNames.Float,rough)]:shader.CreateInput(key,typ).Set(val)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    return mat

def mesh(s,name,verts,faces,rgb=STEEL,collide=False,visible=True):
    p=UsdGeom.Mesh.Define(s,'/Asset/'+name)
    p.CreatePointsAttr([Gf.Vec3f(*map(float,v)) for v in verts]);p.CreateFaceVertexCountsAttr([len(f) for f in faces]);p.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);p.CreateSubdivisionSchemeAttr('none')
    p.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdShade.MaterialBindingAPI.Apply(p.GetPrim()).Bind(material(s,name.replace('/','_'),rgb,.85 if rgb==STEEL else .25,.28))
    if collide:
        UsdPhysics.CollisionAPI.Apply(p.GetPrim()).CreateCollisionEnabledAttr(True);UsdPhysics.MeshCollisionAPI.Apply(p.GetPrim()).CreateApproximationAttr('convexHull')
    if not visible:p.CreateVisibilityAttr('invisible')
    return p

def prism(s,name,xy,z0,z1,rgb=STEEL,collide=True,visible=True):
    n=len(xy);v=[(*p,z) for z in [z0,z1] for p in xy]
    f=[list(reversed(range(n))),list(range(n,2*n))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
    return mesh(s,name,v,f,rgb,collide,visible)

def ring(s,name,rin,rout,z0,z1,n=48,rgb=STEEL,rect=None,collide=True):
    for i in range(n):
        a,b=2*np.pi*np.array([i,i+1])/n
        def outer(t):
            r=rout if rect is None else min(rect[0]/max(abs(np.cos(t)),1e-12),rect[1]/max(abs(np.sin(t)),1e-12))
            return r*np.array([np.cos(t),np.sin(t)])
        xy=[rin*np.array([np.cos(a),np.sin(a)]),outer(a),outer(b),rin*np.array([np.cos(b),np.sin(b)])]
        prism(s,f'{name}/Sector{i:02}',xy,z0,z1,rgb,collide)

def box(s,name,p,size,rgb=DARK,collide=True):
    o=UsdGeom.Cube.Define(s,'/Asset/'+name);o.CreateSizeAttr(1.);o.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));o.AddScaleOp().Set(Gf.Vec3f(*size));o.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdShade.MaterialBindingAPI.Apply(o.GetPrim()).Bind(material(s,name.replace('/','_'),rgb,.35,.45))
    if collide:UsdPhysics.CollisionAPI.Apply(o.GetPrim()).CreateCollisionEnabledAttr(True)
    return o

def bolt():
    s=stage('bolt',BOLT_MASS)
    s.GetDefaultPrim().AddAppliedSchema('PhysxContactReportAPI')
    s.GetDefaultPrim().CreateAttribute('physxContactReport:threshold',Sdf.ValueTypeNames.Float).Set(0.)
    # Bevelled hex head; collision uses the exact head envelope.
    n=6;r=HEAD_AF/np.sqrt(3)
    xy=r*np.array([[np.cos(a),np.sin(a)] for a in np.arange(n)*np.pi/3+np.pi/6])
    prism(s,'HeadContact',xy,0,HEAD_HEIGHT,STEEL,True,False)
    verts=[]
    for z,scale in [(0,.92),(.0004,1),(HEAD_HEIGHT-.0005,1),(HEAD_HEIGHT,.90)]:verts.extend([(*p*scale,z) for p in xy])
    faces=[list(reversed(range(6))),list(range(18,24))]+[[j*6+i,j*6+(i+1)%6,(j+1)*6+(i+1)%6,(j+1)*6+i] for j in range(3) for i in range(6)]
    mesh(s,'MachinedHead',verts,faces)
    # Smooth minor-diameter collider; helical ridge is visual. Physical thread uses a native PhysX joint constraint.
    angles=np.arange(48)*2*np.pi/48
    prism(s,'ShankContact',.00365*np.c_[np.cos(angles),np.sin(angles)],-BOLT_LENGTH+.001,-.00005,STEEL,True,False)
    tipverts=[(radius*np.cos(a),radius*np.sin(a),z) for z,radius in [(-BOLT_LENGTH,.0029),(-BOLT_LENGTH+.001,.00365)] for a in angles]
    tipfaces=[list(reversed(range(48))),list(range(48,96))]+[[i,(i+1)%48,(i+1)%48+48,i+48] for i in range(48)]
    mesh(s,'TipContact',tipverts,tipfaces,STEEL,True,False)
    # Continuous thread surface profile sampled axially, with a real spiral crest.
    rings=256;n=64;verts=[]
    for z in np.linspace(-BOLT_LENGTH,0,rings):
        for a in np.arange(n)*2*np.pi/n:
            phase=((z/PITCH-a/(2*np.pi))%1)
            r=.00325+.00075*max(0,1-abs(phase-.5)/.42)
            r=min(r,.0029+.0011*min(1,(z+BOLT_LENGTH)/.001))
            verts.append((r*np.cos(a),r*np.sin(a),z))
    faces=[[j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i] for j in range(rings-1) for i in range(n)]
    mesh(s,'ThreadVisual',verts,faces);s.GetRootLayer().Save()

def build():
    (ROOT/'assets').mkdir(exist_ok=True);bolt()
    s=stage('fixture');box(s,'Base',[0,0,.006],[.18,.16,.012])
    ring(s,'Receiver',.00435,.045,.012,.090,n=48,rgb=DARK,rect=(.045,.04))
    ring(s,'Bracket',.0046,.06,.090,.093,n=48,rgb=BRACKET,rect=(.07,.045))
    s.GetRootLayer().Save()
    s=stage('entry_stop');ring(s,'ThreadEntry',.0034,.00435,.087,.0875,n=32,rgb=DARK);s.GetRootLayer().Save()
    s=stage('bolt_holder');ring(s,'Holder',.0044,.014,0,.016,n=32,rgb=BRACKET);ring(s,'Neck',.0044,.0055,.016,.022,n=32,rgb=BRACKET);box(s,'Foot',[0,0,.002],[.044,.044,.004]);s.GetRootLayer().Save()
    s=stage('socket',.040)
    # Hexagonal physical cavity: inner AF 13.4mm, 0.2mm clearance per face.
    n=6;rin=.0134/np.sqrt(3);rout=.0105
    for i in range(n):
        a,b=np.pi/6+2*np.pi*np.array([i,i+1])/n
        xy=[rin*np.array([np.cos(a),np.sin(a)]),rout*np.array([np.cos(a),np.sin(a)]),rout*np.array([np.cos(b),np.sin(b)]),rin*np.array([np.cos(b),np.sin(b)])]
        prism(s,f'HexWall{i}',xy,.0015,.009)
        # Machined lead-in:14.4mm mouth tapers to13.4mm AF over1.5mm.
        bottom=xy.copy();bottom[0]=(.0144/np.sqrt(3))*np.array([np.cos(a),np.sin(a)]);bottom[3]=(.0144/np.sqrt(3))*np.array([np.cos(b),np.sin(b)])
        v=[(*point,z) for points,z in [(bottom,0.),(xy,.0015)] for point in points]
        f=[[3,2,1,0],[4,5,6,7]]+[[j,(j+1)%4,(j+1)%4+4,j+4] for j in range(4)]
        mesh(s,f'LeadIn{i}',v,f,STEEL,True)
    ring(s,'UpperBarrel',.0049,.0105,.009,.022,n=48)
    # 3/8in drive square; cavity remains open.
    inner=.0097/2;outer=.0105
    box(s,'DriveFront',[0,(outer+inner)/2,.025],[2*outer,outer-inner,.006],STEEL)
    box(s,'DriveBack',[0,-(outer+inner)/2,.025],[2*outer,outer-inner,.006],STEEL)
    box(s,'DriveLeft',[(outer+inner)/2,0,.025],[outer-inner,2*inner,.006],STEEL)
    box(s,'DriveRight',[-(outer+inner)/2,0,.025],[outer-inner,2*inner,.006],STEEL)
    s.GetRootLayer().Save()
    s=stage('walls')
    from asset_geometry import add_neck_walls
    add_neck_walls(s)
    s.GetRootLayer().Save()
    s=stage('tool_cradle');box(s,'Foot',[0,0,.004],[.23,.065,.008]);
    for x in [-.12,0]:
        for y in [-.026,.026]:box(s,f'Support{str(x).replace("-","N").replace(".","D")}{str(y).replace("-","N").replace(".","D")}',[x,y,.016],[.018,.012,.024],BRACKET)
    # Saddles support the grip underside while the socket rests on the table.
    for j,x in enumerate([-.07,-.02]):box(s,f'GripSaddle{j}',[x,0,.0226],[.010,.040,.0292],BRACKET)
    s.GetRootLayer().Save()
    hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'assets').glob('*.usda'))}
    (ROOT/'assets/generated-hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    (ROOT/'assets/generated-provenance.json').write_text(json.dumps({'source':'Original parametric geometry','builder':'build_assets.py','units':'m, kg','thread_pitch_m':PITCH,'bolt_underhead_length_m':BOLT_LENGTH,'bolt_head_across_flats_m':HEAD_AF,'simplifications':['Thread ridge is visual; native PhysX helical constraint supplies force behavior','Smooth minor-diameter shank collider','Socket/receiver holes preserved by compound convex pieces']},indent=2)+'\n')
if __name__=='__main__':build()

def build_tool():
    """Two-body ratchet. Socket is the drive body's permanently assembled tool."""
    s=stage('handle',HANDLE_MASS)
    p=UsdGeom.Xform.Define(s,'/Asset/Visual').GetPrim();p.GetReferences().AddReference('ratchet_visual.usda','/RatchetVisual/Handle')
    from asset_geometry import add_handle_colliders
    report,_=add_handle_colliders(s,ROOT/'assets/ratchet_visual.usda')
    # Flat rubber side pads spread the finger contact over the rounded grip.
    # They overlap the original shell by 1 mm and share this rigid body.
    pad_color=(.035,.11,.19)
    rubber=material(s,'GripPadRubber',pad_color,metal=0.,rough=.78)
    for sign,name in [(-1,'GripPadNegative'),(1,'GripPadPositive')]:
        pad=box(s,name,[-.13,sign*.0145,.0132],[.045,.003,.023],pad_color)
        UsdShade.MaterialBindingAPI.Apply(pad.GetPrim()).Bind(rubber)
    report['grip_pads']={'source':'Original flat rubber grip pads',
        'center_x_m':-.13,'center_y_magnitude_m':.0145,'center_z_m':.0132,
        'size_m':[.045,.003,.023],'added_mass_kg':.004}
    (ROOT/'assets/handle-collision-provenance.json').write_text(json.dumps(report,indent=2)+'\n')
    s.GetRootLayer().Save()
    s=stage('pawl',PAWL_MASS)
    p=UsdGeom.Xform.Define(s,'/Asset/DriveVisual').GetPrim();p.GetReferences().AddReference('ratchet_visual.usda','/RatchetVisual/Drive')
    c=UsdGeom.Cylinder.Define(s,'/Asset/BearingContact');c.CreateRadiusAttr(.007283);c.CreateHeightAttr(.012081);c.CreateAxisAttr('Z');c.AddTranslateOp().Set(Gf.Vec3d(0,0,.000632));c.CreateVisibilityAttr('invisible');UsdPhysics.CollisionAPI.Apply(c.GetPrim()).CreateCollisionEnabledAttr(False)
    inertia=UsdPhysics.MassAPI(s.GetDefaultPrim());inertia.CreateCenterOfMassAttr(Gf.Vec3f(0,0,.000632));inertia.CreateDiagonalInertiaAttr(Gf.Vec3f(*[PAWL_MASS*(3*.007283**2+.012081**2)/12]*2,PAWL_MASS*.007283**2/2))
    s.GetRootLayer().Save()
    s=stage('drive',SOCKET_MASS)
    socket=UsdGeom.Xform.Define(s,'/Asset/Socket');socket.GetPrim().GetReferences().AddReference('socket.usda','/Asset');socket.AddTranslateOp().Set(Gf.Vec3d(0,0,-.028))
    # The socket is permanently part of this rigid drive; strip nested body schemas.
    socket.GetPrim().RemoveAPI(UsdPhysics.RigidBodyAPI);socket.GetPrim().RemoveAPI(UsdPhysics.MassAPI)
    # Root-layer delete list is required to suppress schemas inherited by reference.
    from pxr import Sdf
    socket.GetPrim().SetMetadata('apiSchemas',Sdf.TokenListOp.CreateExplicit([]))
    s.GetRootLayer().Save()
if __name__=='__main__' and (ROOT/'assets/ratchet_visual.usda').exists():build_tool()

if __name__=='__main__':
    for p in (ROOT/'assets').glob('*.usda'):p.write_text(p.read_text().rstrip()+'\n')
    (ROOT/'assets/generated-hashes.json').write_text(json.dumps({p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted((ROOT/'assets').glob('*.usda'))},indent=2)+'\n')
