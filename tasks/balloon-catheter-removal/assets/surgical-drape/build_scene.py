"""Original procedural surgical training phantom; visual scene only, metres, Z up.
No borrowed anatomical assets. The drape is a static mesh with a genuine opening.
"""
from pathlib import Path
import math, json
import numpy as np
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf, Sdf

TOP=.7475
WINDOW=(.36,0.,.12,.105)  # centre x/y, ellipse radii

def linear(c):
    return tuple(v/12.92 if v<=.04045 else ((v+.055)/1.055)**2.4 for v in c)

class Builder:
    def __init__(self,stage):
        self.s=stage; self.root='/Surgical'; UsdGeom.Xform.Define(stage,self.root)
        stage.SetDefaultPrim(stage.GetPrimAtPath(self.root))
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z);UsdGeom.SetStageMetersPerUnit(stage,1.)
        self.mats={}
    def material(self,name,color,rough=.7,metal=0.):
        p=self.root+'/Looks/'+name
        m=UsdShade.Material.Define(self.s,p);sh=UsdShade.Shader.Define(self.s,p+'/Shader')
        sh.CreateIdAttr('UsdPreviewSurface')
        sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear(color)))
        sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metal)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface');self.mats[name]=m
    def bind(self,g,mat,collide=False):
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(self.mats[mat])
        if collide: UsdPhysics.CollisionAPI.Apply(g.GetPrim())
        return g
    def mesh(self,name,pts,faces,mat,subdiv='none',collide=False):
        g=UsdGeom.Mesh.Define(self.s,self.root+'/'+name)
        g.CreatePointsAttr([Gf.Vec3f(*p) for p in pts]);g.CreateFaceVertexCountsAttr([len(f) for f in faces])
        g.CreateFaceVertexIndicesAttr([i for f in faces for i in f]);g.CreateSubdivisionSchemeAttr(subdiv)
        g.CreateDoubleSidedAttr(True);return self.bind(g,mat,collide)
    def box(self,name,pos,size,mat,collide=False):
        g=UsdGeom.Cube.Define(self.s,self.root+'/'+name);g.CreateSizeAttr(1.)
        g.AddTranslateOp().Set(Gf.Vec3d(*pos));g.AddScaleOp().Set(Gf.Vec3f(*size));return self.bind(g,mat,collide)
    def sphere(self,name,pos,scale,mat):
        g=UsdGeom.Sphere.Define(self.s,self.root+'/'+name);g.CreateRadiusAttr(1.)
        g.AddTranslateOp().Set(Gf.Vec3d(*pos));g.AddScaleOp().Set(Gf.Vec3f(*scale));return self.bind(g,mat)
    def tube(self,name,points,radius,mat,sides=12):
        points=np.asarray(points,float);pts=[]
        for i,p in enumerate(points):
            tangent=points[min(i+1,len(points)-1)]-points[max(0,i-1)];tangent/=np.linalg.norm(tangent)
            v=np.cross(tangent,[0,0,1])
            if np.linalg.norm(v)<.01:v=np.cross(tangent,[0,1,0])
            v/=np.linalg.norm(v);w=np.cross(tangent,v)
            for k in range(sides):pts.append(tuple(p+radius*(math.cos(2*math.pi*k/sides)*v+math.sin(2*math.pi*k/sides)*w)))
        faces=[(i*sides+k,i*sides+(k+1)%sides,(i+1)*sides+(k+1)%sides,(i+1)*sides+k) for i in range(len(points)-1) for k in range(sides)]
        return self.mesh(name,pts,faces,mat)
    def ring(self,name,center,rx,ry,r,mat):
        points=[(center[0]+rx*math.cos(t),center[1]+ry*math.sin(t),center[2]) for t in np.linspace(0,2*math.pi,97)]
        return self.tube(name,points,r,mat)
    def rounded(self,name,pos,size,r,mat,collide=False):
        # Chamfered rounded-rectangle solid, with a flat top and bottom.
        x,y,z=pos; sx,sy,sz=size;pts=[];n=12
        bevel=min(.003,sz/4,r/3)
        for zz,inset in [(-sz/2,bevel),(-sz/2+bevel,0),(sz/2-bevel,0),(sz/2,bevel)]:
            for cx,cy,start in [(sx/2-r,sy/2-r,0),(-sx/2+r,sy/2-r,90),(-sx/2+r,-sy/2+r,180),(sx/2-r,-sy/2+r,270)]:
                for t in np.linspace(start,start+90,n,endpoint=False):
                    a=math.radians(t);pts.append((x+cx+(r-inset)*math.cos(a),y+cy+(r-inset)*math.sin(a),z+zz))
        N=4*n;faces=[tuple(reversed(range(N))),tuple(range(3*N,4*N))]
        faces += [(j*N+k,j*N+(k+1)%N,(j+1)*N+(k+1)%N,(j+1)*N+k) for j in range(3) for k in range(N)]
        return self.mesh(name,pts,faces,mat,collide=collide)

def build(path,contents='heart-bullet'):
    s=Usd.Stage.CreateNew(str(path));b=Builder(s)
    for args in [('drape',(.13,.48,.35),.92,0),('seam',(.22,.58,.44),.92,0),('steel',(.68,.74,.77),.27,.75),('darksteel',(.28,.34,.37),.38,.7),('rubber',(.12,.17,.18),.9,0),('wall',(.78,.85,.85),.8,0),('floor',(.39,.51,.54),.85,0),('panel',(.53,.69,.68),.7,0),('white',(.89,.93,.92),.6,0),('gauze',(.97,.95,.89),.95,0),('thread',(.82,.81,.75),.95,0),('heart',(.60,.16,.19),.49,0),('vein',(.27,.18,.23),.5,0),('artery',(.69,.23,.25),.5,0),('silicone',(.77,.48,.42),.79,0),('socket',(.25,.11,.13),.8,0),('brass',(.79,.58,.25),.28,.8),('screen',(.025,.065,.074),.4,0),('trace',(.21,.72,.51),.65,0)]:b.material(*args)
    # Work surface top matches the source station exactly. Bases and stand remain exposed.
    b.rounded('Table/Top',(.36,0,TOP-.03),(1.10,1.30,.06),.055,'steel',True)
    b.rounded('Table/Mattress',(.47,0,TOP+.007),(.84,1.14,.014),.045,'rubber',True)
    b.box('Table/Column',(.40,0,.35),(.18,.30,.66),'white',True)
    b.rounded('Table/Foot',(.40,0,.045),(.64,.80,.06),.07,'darksteel',True)
    for y in [-.61,.61]:
        b.tube('Table/Rail'+str(y).replace('.','_').replace('-','n'),[(.13,y,TOP-.07),(.85,y,TOP-.07)],.012,'steel')
    # Drape generated as an annulus: NO faces span the ellipse opening.
    cx,cy,rx,ry=WINDOW;N=256;R=44;pts=[];faces=[]
    xmin,xmax,ymin,ymax=.055,.91,-.57,.57
    def height(x,y):
        mound=.047*math.exp(-((x-.39)/.23)**4-(y/.155)**4)
        folds=.0016*math.sin(73*x+8*math.sin(13*y))*math.sin(32*y+7*x)+.0007*math.sin(173*y+12*x)
        return TOP+.018+mound+folds
    boundary=[]
    for i in range(N):
        a=i*2*math.pi/N;dx,dy=math.cos(a),math.sin(a)
        out=min((xmax-cx)/dx if dx>1e-9 else (xmin-cx)/dx if dx< -1e-9 else 1e9,(ymax-cy)/dy if dy>1e-9 else (ymin-cy)/dy if dy< -1e-9 else 1e9)
        xi,yi=cx+rx*dx,cy+ry*dy;xo,yo=cx+out*dx,cy+out*dy
        boundary.append((xo,yo))
        for j in range(R):
            t=j/(R-1);x=xi*(1-t)+xo*t;y=yi*(1-t)+yo*t;pts.append((x,y,height(x,y)))
    for i in range(N):
        for j in range(R-1):faces.append((i*R+j,((i+1)%N)*R+j,((i+1)%N)*R+j+1,i*R+j+1))
    b.mesh('Drape/Surface',pts,faces,'drape')
    edge=[(cx+rx*math.cos(a),cy+ry*math.sin(a),height(cx+rx*math.cos(a),cy+ry*math.sin(a))+.001) for a in np.linspace(0,2*math.pi,257)]
    b.tube('Drape/WindowHem',edge,.0019,'seam')
    # Draping over three exposed edges; near bases edge lies flat.
    for side in ['left','right','far']:
        pp=[];ff=[];A=140;D=24
        for i in range(A):
            t=i/(A-1)
            for j in range(D):
                q=j/(D-1);w=math.sin(t*math.pi*22)*.007*math.sin(q*math.pi/2)
                if side=='far':x=xmax+.01*math.sin(q*math.pi/2)+w;y=ymin+t*(ymax-ymin)
                else:x=xmin+t*(xmax-xmin);y=(ymax if side=='left' else ymin)+(1 if side=='left' else -1)*(.01*math.sin(q*math.pi/2)+w)
                z=height(min(x,xmax),max(ymin,min(ymax,y)))-.25*q+.007*math.sin(t*24)*q
                pp.append((x,y,z))
        for i in range(A-1):
            for j in range(D-1):ff.append((i*D+j,(i+1)*D+j,(i+1)*D+j+1,i*D+j+1))
        b.mesh('Drape/Hanging_'+side,pp,ff,'drape')
    # Recessed torso-training insert with replaceable anatomy content under the window.
    b.rounded('Phantom/Insert',(.36,0,TOP+.021),(.255,.225,.04),.055,'silicone')
    # Low wall round the opening; organ remains below the drape's highest rim.
    if contents=='heart-bullet':
        # Asymmetric tapered anatomical ventricles, tilted apex, atria and three hollow vessels.
        pp=[];ff=[];U=72;V=36
        for j in range(V+1):
            t=j*math.pi/V
            for i in range(U):
                a=i*2*math.pi/U;rad=math.sin(t)*(1+.16*math.cos(t))
                x=.355+.046*rad*math.cos(a)+.015*(j/V)**2
                y=.027+.034*rad*math.sin(a)-.014*(j/V)**2
                z=TOP+.048+.033*math.cos(t)
                x+=.003*math.sin(3*a)*math.sin(t)**2
                pp.append((x,y,z))
        for j in range(V):
            for i in range(U):ff.append((j*U+i,j*U+(i+1)%U,(j+1)*U+(i+1)%U,(j+1)*U+i))
        b.mesh('Contents/Heart/Ventricles',pp,ff,'heart','catmullClark')
        b.sphere('Contents/Heart/AtriumA',(.326,.041,TOP+.067),(.022,.022,.018),'heart')
        b.sphere('Contents/Heart/AtriumB',(.333,.006,TOP+.064),(.019,.018,.019),'heart')
        vessels=[([(.322,.028,TOP+.072),(.310,.030,TOP+.091),(.313,.040,TOP+.101),(.326,.044,TOP+.097)],.008,'artery'),([(.335,.003,TOP+.072),(.329,-.003,TOP+.087),(.314,-.01,TOP+.091)],.007,'vein'),([(.319,.056,TOP+.068),(.299,.065,TOP+.085)],.005,'vein')]
        for i,(points,r,mat) in enumerate(vessels):
            b.tube(f'Contents/Heart/Vessel{i}',points,r,mat)
            b.sphere(f'Contents/Heart/VesselMouth{i}',points[-1],(r*.72,r*.72,r*.5),'socket')
        # Coronary vessel on the visible upper surface, decorative (no fluid simulation).
        for i,(points,r,mat) in enumerate([
            ([(.324,.03,TOP+.079),(.345,.033,TOP+.081),(.369,.021,TOP+.075),(.389,.007,TOP+.056)],.0018,'artery'),
            ([(.345,.033,TOP+.081),(.353,.049,TOP+.074),(.368,.053,TOP+.064)],.0012,'vein'),
            ([(.359,.027,TOP+.079),(.362,.003,TOP+.073),(.375,-.003,TOP+.060)],.0013,'artery')]):b.tube(f'Contents/Heart/Coronary{i}',points,r,mat)
        b.ring('Contents/ExtractionSocket',(.373,-.056,TOP+.043),.013,.013,.003,'socket')
        b.sphere('Contents/SocketInterior',(.373,-.056,TOP+.038),(.011,.011,.006),'socket')
        b.tube('Contents/Projectile/Body',[(.373,-.056,TOP+.038),(.373,-.056,TOP+.060)],.006,'brass',24)
        b.sphere('Contents/Projectile/Nose',(.373,-.056,TOP+.061),(.006,.006,.008),'brass')
    # Tool dishes on green cloth. One spring forceps tool and one folded gauze swab.
    def dish(name,x,y,sx,sy):
        z=height(x,y)+.006
        b.rounded(name+'/Floor',(x,y,z),(sx,sy,.008),.018,'steel')
        for side in [-1,1]:
            b.box(name+('/RimXpos' if side>0 else '/RimXneg'),(x+side*(sx/2-.004),y,z+.009),(.006,sy-.025,.018),'steel')
            b.box(name+('/RimYpos' if side>0 else '/RimYneg'),(x,y+side*(sy/2-.004),z+.009),(sx-.025,.006,.018),'steel')
        return z+.009
    z=dish('Tools/ForcepsDish',.18,.26,.255,.13)
    # Distinct twin prongs joined at heel; matte grip pads visually show the squeeze surface.
    for sign in [-1,1]:
        pts=[(.075,.26,z+.008),(.11,.26+sign*.013,z+.010),(.17,.26+sign*.014,z+.010),(.235,.26+sign*.007,z+.007),(.28,.26+sign*.004,z+.005)]
        b.tube('Tools/Forceps/Jaw'+str(sign).replace('-','n'),pts,.0027,'steel')
        b.rounded('Tools/Forceps/Grip'+str(sign).replace('-','n'),(.151,.26+sign*.014,z+.010),(.055,.007,.009),.003,'darksteel')
        for k in range(8):b.box(f'Tools/Forceps/Groove{sign}_{k}'.replace('-','n'),(.128+k*.006,.26+sign*.014,z+.015),(.0015,.006,.0005),'steel')
    gz=dish('Tools/GauzeDish',.18,-.26,.14,.13)
    for k in range(6):b.rounded(f'Tools/Gauze/Layer{k}',(.18,-.26,gz+.002+k*.002),(.060,.055,.0025),.009,'gauze')
    for i in range(27):
        v=-.026+i*.002
        b.tube(f'Tools/Gauze/Weft{i}',[(.153,-.26+v,gz+.014),(.18,-.26+v,gz+.0146),(.207,-.26+v,gz+.014)],.00021,'thread',6)
        b.tube(f'Tools/Gauze/Warp{i}',[(.18+v,-.284,gz+.0142),(.18+v,-.26,gz+.0148),(.18+v,-.236,gz+.0142)],.00021,'gauze',6)
    # Background has no new lights or emissive materials.
    b.box('Room/Floor',(0,0,-.023),(12,12,.02),'floor',True)
    b.box('Room/BackWall',(-1.18,0,1.35),(.08,6,2.75),'wall',True)
    b.box('Room/WallBand',(-1.13,0,.78),(.014,6,.06),'panel')
    for k in range(7):b.box(f'Room/WallJoint{k}',(-1.135,-2.4+k*.8,1.4),(.004,.005,2.5),'panel')
    b.rounded('Room/Cabinet',(-.88,.95,.48),(.38,.80,.94),.035,'white')
    for k in range(3):
        b.box(f'Room/Drawer{k}',(-.68,.95,.25+k*.23),(.012,.71,.205),'panel')
        b.tube(f'Room/Handle{k}',[(-.66,.84,.28+k*.23),(-.66,1.06,.28+k*.23)],.006,'steel')
    # Monitor faces the overview camera; its pattern is unlabelled, non-diagnostic decoration.
    b.box('Room/Monitor',(-.76,-.78,1.25),(.055,.37,.26),'darksteel')
    b.box('Room/Screen',(-.728,-.78,1.25),(.006,.335,.22),'screen')
    line=[]
    for i in range(101):
        t=i/100; yy=-.93+.3*t
        zz=1.26+.003*math.sin(32*t)+.037*math.exp(-((t-.48)/.018)**2)-.016*math.exp(-((t-.52)/.025)**2)
        line.append((-.723,yy,zz))
    b.tube('Room/MonitorTrace',line,.0017,'trace',6)
    s.GetRootLayer().Save()
    info={'units':'metres','table_top':TOP,'window':{'centre':list(WINDOW[:2]),'ellipse_diameters':[2*rx,2*ry]},'contents':contents,'visual_only':['static drape','rigid-looking anatomical phantom','unactuated forceps','gauze visual mesh','no bleeding or hemostasis dynamics'],'new_lights':0,'original_procedural_assets':True}
    Path(path).with_suffix('.json').write_text(json.dumps(info,indent=2)+'\n')
    return info

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('output');p.add_argument('--contents',choices=['heart-bullet','empty'],default='heart-bullet');a=p.parse_args()
    build(Path(a.output),a.contents)
