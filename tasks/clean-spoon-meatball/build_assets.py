"""Author reusable food/tool assets and a local YAM scene. Metres, kilograms, Z up."""
from pathlib import Path
import json
import math
import numpy as np
from PIL import Image, ImageFilter
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade
from usd_tools import (ROOT, ASSETS, TABLE_TOP, new_stage, material, mesh, box,
    transform, collision, body, physics_material, physics_bind, instance, base_scene)

SPOON_YAW = 0.0
PLACEMENTS = {
    "CleanPlate": {"asset": "platter.usda", "position": [0.335, 0.320, TABLE_TOP]},
    "RawPlate": {"asset": "platter.usda", "position": [0.335, -0.320, TABLE_TOP]},
    "MeatballBowl": {"asset": "bowl.usda", "position": [0.390, 0.000, TABLE_TOP]},
    "Plate": {"asset": "plate.usda", "position": [0.130, 0.000, TABLE_TOP]},
    "CleanSpoon": {"asset": "spoon_clean.usda", "position": [0.335, 0.320, TABLE_TOP + 0.0045], "yaw": SPOON_YAW},
    "UsedSpoon": {"asset": "spoon_used.usda", "position": [0.335, -0.320, TABLE_TOP + 0.0045], "yaw": SPOON_YAW},
}


def hidden_hull(s, path, points, faces):
    m = mesh(s, path, points, faces)
    collision(m.GetPrim(), convex=True, offset=0.0003)
    m.CreateVisibilityAttr("invisible")
    m.CreatePurposeAttr("guide")
    return m.GetPrim()


def prism(s, path, bottom, top):
    n = len(bottom)
    return hidden_hull(s, path, [*bottom, *top],
        [tuple(reversed(range(n))), tuple(range(n, 2*n))] +
        [(i, (i+1)%n, (i+1)%n+n, i+n) for i in range(n)])


def rounded_rect(hx, hy, radius, steps=12):
    result=[]
    for cx,cy,start in ((hx-radius,hy-radius,0),(-hx+radius,hy-radius,90),
                        (-hx+radius,-hy+radius,180),(hx-radius,-hy+radius,270)):
        for i in range(steps):
            a=math.radians(start+i*90/steps)
            result.append((cx+radius*math.cos(a),cy+radius*math.sin(a)))
    return np.asarray(result)


def ring_surface(s, path, rings, mat):
    n=len(rings[0]); points=np.concatenate(rings)
    faces=[(j*n+i,(j+1)*n+i,(j+1)*n+(i+1)%n,j*n+(i+1)%n)
           for j in range(len(rings)-1) for i in range(n)]
    return mesh(s,path,points,faces,mat,smooth=True)


def textures():
    rng=np.random.default_rng(41); n=768
    y,x=np.mgrid[:n,:n]/n
    def noise(k):
        a=Image.fromarray((rng.random((k,k))*255).astype('uint8'))
        return np.asarray(a.resize((n,n),Image.Resampling.BICUBIC).filter(ImageFilter.GaussianBlur(2)))/255
    coarse=noise(18); fine=noise(100)
    t=np.clip((coarse-.27)/.5,0,1)
    dark=np.array([73,36,16]); gold=np.array([194,127,57])
    meat=dark[None,None,:]*(1-t[...,None])+gold[None,None,:]*t[...,None]
    meat+=(fine[...,None]-.5)*37
    Image.fromarray(np.uint8(np.clip(meat,0,255))).save(ASSETS/'textures/meatball_seared.png')
    # Planar fillet UVs: coherent muscle bundles, interrupted fine fibres and moist patches.
    broad=noise(7); mid=noise(27); micro=noise(160)
    flow=y*54+.38*np.sin(x*9)+.15*np.sin(x*23+y*5)
    fibres=(.5+.5*np.sin(flow*2*math.pi))**16
    bundles=(.5+.5*np.sin((y*14+.15*np.sin(x*8))*2*math.pi))**6
    chicken=np.array([211,139,126])[None,None,:]*(.90+.16*broad[...,None])
    chicken+=(mid[...,None]-.5)*np.array([15,21,17])
    chicken+=fibres[...,None]*np.array([15,18,15])*(.35+.65*mid[...,None])
    chicken+=bundles[...,None]*np.array([4,6,5])+(micro[...,None]-.5)*5
    # Thin pale connective tissue follows one irregular edge rather than covering the fillet.
    fascia=np.exp(-((y-(.79+.025*np.sin(x*17)))/.019)**2)
    fascia*=np.clip((mid-.35)*2.5,0,1)
    chicken=chicken*(1-.55*fascia[...,None])+np.array([238,213,191])*.55*fascia[...,None]
    Image.fromarray(np.uint8(np.clip(chicken,0,255))).save(ASSETS/'textures/raw_chicken.png')
    height=.35*micro+.18*mid+.32*fibres+.15*bundles
    dy,dx=np.gradient(height)
    normal=np.stack([-dx*4,-dy*4,np.ones_like(dx)],axis=-1)
    normal/=np.linalg.norm(normal,axis=-1,keepdims=True)
    Image.fromarray(np.uint8(np.clip((normal*.5+.5)*255,0,255))).save(ASSETS/'textures/raw_chicken_normal.png')
    rough=np.uint8(np.clip(255*(.26+.16*mid+.06*broad),0,255))
    Image.fromarray(rough).save(ASSETS/'textures/raw_chicken_roughness.png')


def spoon():
    s=new_stage(ASSETS/'spoon.usda','/Spoon')
    steel=material(s,'/Spoon/Looks/Steel',(.77,.79,.81),.20,.95)
    black=material(s,'/Spoon/Looks/Handle',(.055,.060,.064),.36)
    # Local +X points from the handle toward the bowl. Real shell thickness 1.8 mm.
    cx,rx,ry,depth,thickness=.054,.050,.050,.024,.0018
    def point(r,a,outer=False):
        opening=np.clip(math.cos(a)/.45,0.0,1.0)
        opening=opening*opening*(3-2*opening)
        bevel=opening*r*r
        shell=thickness*(1-bevel)+.0004*bevel
        # A low rounded nose enters below food; deep sides and back retain it.
        front_drop=.0254*opening*r*r
        x=math.copysign(abs(math.cos(a))**.25,math.cos(a))
        y=math.copysign(abs(math.sin(a))**.25,math.sin(a))
        return (cx+rx*r*x,ry*r*y,.0035+depth*r*r-front_drop-(shell if outer else 0))
    radii=np.linspace(.0001,1,13); angles=np.arange(64)*2*math.pi/64
    rings=[np.array([point(r,a) for a in angles]) for r in radii]
    rings += [np.array([point(r,a,True) for a in angles]) for r in radii[::-1]]
    ring_surface(s,'/Spoon/Visuals/Bowl',rings,steel)
    # Distinct convex shell tiles preserve the empty bowl interior in PhysX.
    colliders=[]; segments=24; radii=[0,.25,.50,.75,1.0]
    for j,(r0,r1) in enumerate(zip(radii[:-1],radii[1:])):
        for i in range(segments):
            a0,a1=i*2*math.pi/segments,(i+1)*2*math.pi/segments
            inner=[point(r,a) for r,a in ((r0,a0),(r1,a0),(r1,a1),(r0,a1))]
            outer=[point(r,a,True) for r,a in ((r0,a0),(r1,a0),(r1,a1),(r0,a1))]
            colliders.append(prism(s,f'/Spoon/Colliders/Bowl_{j}_{i}',outer,inner))
    # Rounded black handle with a 78 mm unobstructed grasp region, 24 mm wide.
    shape=rounded_rect(.051,.012,.005,8)
    hrings=[]
    for scale,z in ((.90,.017),(1,.019),(1,.035),(.90,.037)):
        hrings.append(np.column_stack([shape[:,0]*scale-.069,shape[:,1]*scale,np.full(len(shape),z)]))
    handle=ring_surface(s,'/Spoon/Visuals/Handle',hrings,black)
    # Cap both ends of the rounded handle.
    pts=np.concatenate(hrings); n=len(shape)
    faces=[tuple(reversed(range(n))),tuple(range(3*n,4*n))]
    mesh(s,'/Spoon/Visuals/HandleCaps',pts,faces,black)
    hc=box(s,'/Spoon/Colliders/Handle',(.099,.024,.020),(-.069,0,.027),collide=True)
    UsdGeom.Imageable(hc).MakeInvisible(); UsdGeom.Imageable(hc).CreatePurposeAttr('guide')
    colliders.append(hc)
    # Visible rear rest keeps the handle above the tray floor without a hidden fixture.
    rest=box(s,'/Spoon/Visuals/HandleRest',(.012,.020,.017),(-.112,0,.0095),black,collide=True)
    colliders.append(rest)
    bottom=[(-.020,-.005,.023),(.030,-.005,.009),(.030,.005,.009),(-.020,.005,.023)]
    top=[(x,y,z+.004) for x,y,z in bottom]
    neck=mesh(s,'/Spoon/Visuals/Neck',bottom+top,
       [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)],steel,smooth=True)
    collision(neck.GetPrim(),convex=True,offset=.0003); colliders.append(neck.GetPrim())
    pm=physics_material(s,'/Spoon',.95)
    for c in colliders: physics_bind(c,pm)
    body(s,'/Spoon',.068,(-.008,0,.019))
    s.GetDefaultPrim().SetCustomDataByKey('graspCenterLocal',Gf.Vec3d(-.069,0,.027))
    s.GetRootLayer().Save()
    for used in (False,True):
        filename='spoon_used.usda' if used else 'spoon_clean.usda'
        v=new_stage(ASSETS/filename,'/Spoon')
        v.GetDefaultPrim().GetReferences().AddReference('spoon.usda')
        v.GetDefaultPrim().SetCustomDataByKey('cleanliness','used_with_raw_chicken' if used else 'clean')
        if used:
            residue=material(v,'/Spoon/Looks/ChickenResidue',(.78,.48,.35),.53)
            pale=material(v,'/Spoon/Looks/PaleResidue',(.89,.70,.57),.48)
            rng=np.random.default_rng(5)
            for k in range(22):
                center=np.array([rng.uniform(-.60,.60),rng.uniform(-.65,.65)])
                if np.linalg.norm(center)>.8: center*=.7
                scale=rng.uniform(.05,.14)
                cp=point(np.linalg.norm(center),math.atan2(center[1],center[0]))
                pts=[(cp[0],cp[1],cp[2]+.0003)]
                for i in range(15):
                    a=2*math.pi*i/15
                    uv=center+np.array([math.cos(a),math.sin(a)])*scale*rng.uniform(.65,1.3)
                    rp=point(np.linalg.norm(uv),math.atan2(uv[1],uv[0]))
                    pts.append((rp[0],rp[1],rp[2]+.00032))
                mesh(v,f'/Spoon/Visuals/Residue_{k}',pts,[(0,i+1,(i+1)%15+1) for i in range(15)],residue if k%3 else pale,smooth=True)
        v.GetRootLayer().Save()


def container(name, rectangular, radius=.110, white=False, half_size=(.125,.1225)):
    root='/'+name; s=new_stage(ASSETS/(name.lower()+'.usda'),root)
    metal=rectangular and not white
    mat=material(s,root+'/Looks/Surface',(.72,.75,.77) if metal else (.965,.970,.975),.26 if metal else .13,.92 if metal else 0)
    if rectangular:
        outline=rounded_rect(*half_size,.025)
        if white:
            profile=[(.0001,.004),(.90,.004),(.945,.0048),(.985,.0075),(1,.008),
                     (1.008,.0068),(1.005,.0045),(.94,.0015),(.85,0),(.0001,0)]
            floor_scale=.90
        else:
            profile=[(.0001,.004),(.82,.004),(.88,.007),(.98,.026),(1,.029),(1.015,.028),(1.015,.025),(.90,.002),(.84,0),(.0001,0)]
            floor_scale=.82
    else:
        a=np.arange(64)*2*math.pi/64
        outline=radius*np.column_stack([np.cos(a),np.sin(a)])
        # Broad flat eating surface and a small rolled edge: a dinner plate, not a bowl.
        profile=[(.0001,.004),(.90,.004),(.945,.0048),(.985,.0075),(1,.008),
                 (1.008,.0068),(1.005,.0045),(.94,.0015),(.85,0),(.0001,0)]
        floor_scale=.90
    if not metal:
        shader=UsdShade.Shader(s.GetPrimAtPath(root+'/Looks/Surface/Surface'))
        shader.CreateInput('clearcoat',Sdf.ValueTypeNames.Float).Set(.32)
        shader.CreateInput('clearcoatRoughness',Sdf.ValueTypeNames.Float).Set(.10)
        shader.CreateInput('ior',Sdf.ValueTypeNames.Float).Set(1.48)
    rings=[np.column_stack([outline*f,np.full(len(outline),z)]) for f,z in profile]
    ring_surface(s,root+'/Visuals/Shell',rings,mat)
    colliders=[]
    colliders.append(prism(s,root+'/Colliders/Floor',[(x*floor_scale,y*floor_scale,0) for x,y in outline],[(x*floor_scale,y*floor_scale,.004) for x,y in outline]))
    # Radial sectors follow the rim. Never apply one convex hull to an entire vessel.
    wall_profile=profile[1:-1]
    for i in range(len(outline)):
        j=(i+1)%len(outline)
        points=[(outline[k,0]*f,outline[k,1]*f,z) for k in (i,j) for f,z in wall_profile]
        n=len(wall_profile)
        faces=[tuple(reversed(range(n))),tuple(range(n,2*n))]+[(k,(k+1)%n,(k+1)%n+n,k+n) for k in range(n)]
        colliders.append(hidden_hull(s,f'{root}/Colliders/Wall_{i}',points,faces))
    pm=physics_material(s,root,.60)
    if rectangular and white:
        # A primitive support avoids thin, wide convex-mesh cooking tolerances.
        support=UsdGeom.Cube.Define(s,root+'/Colliders/FlatSupport')
        support.CreateSizeAttr(1.0)
        transform(support.GetPrim(),(0,0,.002),scale=(half_size[0]*1.8,half_size[1]*1.8,.004))
        collision(support.GetPrim(),offset=.0003)
        support.CreateVisibilityAttr('invisible');support.CreatePurposeAttr('guide')
        colliders.append(support.GetPrim())
    for c in colliders:physics_bind(c,pm)
    s.GetRootLayer().Save()


def meatball_bowl():
    root='/Bowl'; s=new_stage(ASSETS/'bowl.usda',root)
    mat=material(s,root+'/Looks/Surface',(.965,.970,.975),.16)
    shader=UsdShade.Shader(s.GetPrimAtPath(root+'/Looks/Surface/Surface'))
    shader.CreateInput('clearcoat',Sdf.ValueTypeNames.Float).Set(.25)
    shader.CreateInput('clearcoatRoughness',Sdf.ValueTypeNames.Float).Set(.12)
    radius=.120; segments=64; angles=np.arange(segments)*2*math.pi/segments
    inner=[(.90,.006),(.94,.0075),(.98,.012),(1,.015)]
    def ring(r,z,outer=False):
        rr=radius*r+(.002 if outer else 0)
        # A low entry lip clears the gripper; the back rim retains rolling food.
        entry=np.clip((np.cos(angles-math.radians(135))-.5)/.5,0,1)
        entry=entry*entry*(3-2*entry)
        gain=1+(1-entry)*(.045-.015)/(.015-.006)
        height=np.full(segments,z) if z<=.006 else .006+(z-.006)*gain
        return np.column_stack([rr*np.cos(angles),rr*np.sin(angles),height-(.004 if outer else 0)])
    rings=[ring(.0001,.006)]+[ring(r,z) for r,z in inner]
    rings+=[ring(r,z,True) for r,z in inner[::-1]]+[ring(.90,0),ring(.0001,0)]
    ring_surface(s,root+'/Visuals/Shell',rings,mat)
    colliders=[prism(s,root+'/Colliders/Floor',ring(.90,0),ring(.90,.006))]
    for j,((r0,z0),(r1,z1)) in enumerate(zip(inner[:-1],inner[1:])):
        a,b=ring(r0,z0),ring(r1,z1)
        oa,ob=ring(r0,z0,True),ring(r1,z1,True)
        for i in range(segments):
            k=(i+1)%segments
            colliders.append(prism(s,f'{root}/Colliders/Wall_{j}_{i}',[oa[i],ob[i],ob[k],oa[k]],[a[i],b[i],b[k],a[k]]))
    pm=physics_material(s,root,.40)
    for c in colliders:physics_bind(c,pm)
    s.GetDefaultPrim().SetCustomDataByKey('assetDescription','Shallow 240 mm bowl with curved scooping walls, a flat central well and a 56 mm rim.')
    s.GetRootLayer().Save()


def raw_chicken():
    root='/RawChicken'; s=new_stage(ASSETS/'raw_chicken.usda',root)
    mat=material(s,root+'/Looks/Food',(.83,.54,.48),.34,texture='textures/raw_chicken.png')
    shader=UsdShade.Shader(s.GetPrimAtPath(root+'/Looks/Food/Surface'))
    shader.CreateInput('clearcoat',Sdf.ValueTypeNames.Float).Set(.20)
    shader.CreateInput('clearcoatRoughness',Sdf.ValueTypeNames.Float).Set(.22)
    shader.CreateInput('ior',Sdf.ValueTypeNames.Float).Set(1.38)
    for label,file,output in (('Normal','raw_chicken_normal.png','rgb'),('Roughness','raw_chicken_roughness.png','r')):
        tex=UsdShade.Shader.Define(s,root+'/Looks/Food/'+label)
        tex.CreateIdAttr('UsdUVTexture')
        tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath('textures/'+file))
        tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('raw')
        tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(UsdShade.Shader(s.GetPrimAtPath(root+'/Looks/Food/UV')).ConnectableAPI(),'result')
        tex.CreateOutput(output,Sdf.ValueTypeNames.Float3 if output=='rgb' else Sdf.ValueTypeNames.Float)
        if label=='Normal':
            tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(2,2,2,1))
            tex.CreateInput('bias',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(-1,-1,-1,0))
            shader.CreateInput('normal',Sdf.ValueTypeNames.Normal3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
        else:
            shader.GetInput('roughness').ConnectToSource(tex.ConnectableAPI(),'r')
    # Asymmetric breast-fillet outline, with a tapered end and a slightly ragged cut edge.
    control=np.array([(-.043,-.006),(-.033,-.020),(-.012,-.026),(.015,-.024),(.034,-.012),
                      (.036,.005),(.021,.021),(-.006,.024),(-.026,.014),(-.041,.003)])
    outline=[]
    for k in range(len(control)):
        p0,p1,p2,p3=[control[j%len(control)] for j in (k-1,k,k+1,k+2)]
        for j in range(10):
            t=j/10
            xy=.5*((2*p1)+(-p0+p2)*t+(2*p0-5*p1+4*p2-p3)*t*t+(-p0+3*p1-3*p2+p3)*t*t*t)
            outline.append(xy)
    outline=np.array(outline)
    def surface(r):
        xy=outline*r
        thickness=.0105*(1-r*r)**.72
        z=.002+thickness*(1+.18*np.sin(xy[:,0]*53)+.10*np.sin(xy[:,1]*70+1))
        flow=xy[:,1]/.00145+.35*np.sin(xy[:,0]*100)
        z+=.00010*np.sin(flow*2*math.pi)*min(1,r*7)*(1-.8*r)
        return np.column_stack([xy,z])
    rings=[surface(r) for r in np.linspace(.0001,1,25)]
    rings+=[np.column_stack([outline*.99,np.full(len(outline),.0005)]),
            np.column_stack([outline*.0001,np.full(len(outline),.0005)])]
    pts=np.concatenate(rings); n=len(outline)
    fs=[(j*n+i,(j+1)*n+i,(j+1)*n+(i+1)%n,j*n+(i+1)%n) for j in range(len(rings)-1) for i in range(n)]
    uv=np.column_stack([(pts[:,0]+.05)/.10,(pts[:,1]+.035)/.07])
    mesh(s,root+'/Visuals/Food',pts,fs,mat,uv,smooth=True)
    fascia=material(s,root+'/Looks/ConnectiveTissue',(.90,.79,.69),.34)
    # A thin broken membrane follows one edge, without replacing the pink flesh.
    for k,(start,end) in enumerate(((57,68),(72,79),(82,88))):
        a=surface(.90)[start:end].copy(); b=surface(.975)[start:end].copy()
        a[:,2]+=.00008; b[:,2]+=.00008
        m=len(a); pp=np.concatenate([a,b])
        mesh(s,f'{root}/Visuals/ConnectiveTissue_{k}',pp,[(i,m+i,m+i+1,i+1) for i in range(m-1)],fascia,smooth=True)
    hull=hidden_hull(s,root+'/Colliders/Food',pts,fs)
    pm=physics_material(s,root,.65);physics_bind(hull,pm)
    body(s,root,.046,(.003,0,.006))
    s.GetDefaultPrim().SetCustomDataByKey('assetDescription','Irregular raw chicken fillet with flat cut underside, pink muscle fibres and moist connective tissue.')
    s.GetRootLayer().Save()


def food(chicken=False):
    if chicken:
        return raw_chicken()
    name='RawChicken' if chicken else 'Meatball'; root='/'+name
    s=new_stage(ASSETS/('raw_chicken.usda' if chicken else 'meatball.usda'),root)
    mat=material(s,root+'/Looks/Food',(.86,.64,.53) if chicken else (.56,.30,.10),.44 if chicken else .73,
                 texture='textures/raw_chicken.png' if chicken else 'textures/meatball_seared.png')
    pts=[]; uv=[]; fs=[]; segments=48; rings=28
    for j in range(rings+1):
        v=j/rings; phi=math.pi*(1-v)
        for i in range(segments+1):
            u=i/segments; a=2*math.pi*u
            n=np.array([math.sin(phi)*math.cos(a),math.sin(phi)*math.sin(a),math.cos(phi)])
            if chicken:
                r=1+.08*math.sin(a*3+phi)*math.sin(phi)**2
                p=n*np.array([.032,.023,.011])*r
                p[0]+=.007*n[2]; p[2]+=.011
            else:
                r=.018*(1+.035*math.sin(n[0]*13+n[2]*7)*math.sin(n[1]*11-2)+.020*math.cos(n[2]*19+n[0]*8))
                p=n*r+np.array([0,0,.018])
            pts.append(p); uv.append((u,v))
    n=segments+1
    for j in range(rings):
        for i in range(segments):
            a=j*n+i; fs.append((a,a+1,a+n+1,a+n))
    visual=mesh(s,root+'/Visuals/Food',pts,fs,mat,uv,smooth=True)
    hull=hidden_hull(s,root+'/Colliders/Food',pts,fs)
    pm=physics_material(s,root,.65 if chicken else .55);physics_bind(hull,pm)
    body(s,root,.046 if chicken else .018,(0,0,.011 if chicken else .018))
    s.GetRootLayer().Save()


def task_scene():
    placements=dict(PLACEMENTS)
    bowl=PLACEMENTS['MeatballBowl']['position']
    for i,(dx,dy) in enumerate(((.02475,-.02475),),1):
        placements[f'Meatball_{i}']={'asset':'meatball.usda','position':[bowl[0]+dx,bowl[1]+dy,TABLE_TOP+.0065]}
    used=PLACEMENTS['UsedSpoon']['position']
    for i,(dx,dy,yaw) in enumerate(((.095,-.080,80),(.020,-.092,0),(-.095,-.100,0),(.095,.090,40),(.010,.100,0))):
        placements[f'RawChicken_{i+1}']={'asset':'raw_chicken.usda','position':[used[0]+dx,used[1]+dy,TABLE_TOP+.0045],'yaw':yaw}
    s=new_stage(ROOT/'task_objects.usda','/Environment')
    for name,row in placements.items():
        angle=math.radians(row.get('yaw',0))/2
        instance(s,'/Environment/'+name,'assets/'+row['asset'],row['position'],(math.cos(angle),0,0,math.sin(angle)))
    s.GetDefaultPrim().SetCustomDataByKey('instruction','Scoop one meatball onto the empty plate.')
    s.GetRootLayer().Save()
    scene=new_stage(ROOT/'scene.usda','/World')
    scene.GetRootLayer().subLayerPaths=['assets/base_station.usda']
    instance(scene,'/World/envs/env_0/Task','task_objects.usda',(0,0,0))
    scene.GetRootLayer().Save()
    grasps={}
    a=math.radians(SPOON_YAW); rotation=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
    for name in ('CleanSpoon','UsedSpoon'):
        center=np.array(placements[name]['position'])+rotation@np.array([-.069,0,.027])
        grasps[name]={'arm':'left' if name=='CleanSpoon' else 'right',
                      'center_world':center.tolist(),'yaw_degrees':SPOON_YAW,'handle_width_m':.024,
                      'handle_thickness_m':.020,'under_handle_clearance_m':.017,'pregrasp_opening_m':.056}
    result={'instruction':'Scoop one meatball onto the empty plate.','scene':'scene.usda','environment_only':'task_objects.usda',
        'units':'metres, kilograms, seconds; Z up','table_top_z':TABLE_TOP,'objects':placements,'spoon_grasps':grasps,
        'spoon_bowl':{'inner_length_m':.100,'inner_width_m':.100,'footprint_power':8,'depth_m':.024,'shell_thickness_m':.0018,'leading_edge_thickness_m':.0004,'front_lip_drop_m':.0254,'collision_tiles':96},
        'meatball_diameter_m':.036,'spoon_mass_kg':.068,
        'evaluation_intent':{'target_count':1,'target':'Plate','required_tool':'CleanSpoon','contaminated_tool':'UsedSpoon',
           'contamination_sources':[f'RawChicken_{i}' for i in range(1,6)]+['UsedSpoon'],
           'scores':{'clean_delivery':1.0,'dirty_delivery':.5,'no_valid_delivery':0.0}},
        'spoon_plates':{'separate':True,'size_each_m':[.300,.340,.008],
                        'clean_plate':'CleanPlate','raw_meat_plate':'RawPlate'},
        'arrangement':'Three columns: clean spoon on its own plate at left; fixed meatball bowl above serving plate in center; used spoon and five raw chicken pieces on a separate plate at right.',
        'meatball_bowl':{'diameter_m':.240,'rim_height_m':.045,'entry_rim_height_m':.015,'entry_yaw_degrees':135,'floor_height_m':.006,'flat_floor_radius_m':.108,
                         'fixed_to_world':True,
                         'purpose':'One rigid meatball offset 35 mm toward the back of the scoop approach with an unobstructed approach for a physical scoop.'},
        'physics_validation':'pending'}
    (ROOT/'layout.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Built local clean-spoon meatball scene:',ROOT)


if __name__=='__main__':
    textures(); spoon(); container('Tray',True); container('Plate',False)
    container('Spoon_Plate',False,radius=.130)
    container('Platter',True,white=True,half_size=(.150,.170))
    meatball_bowl(); food(); food(True); base_scene(); task_scene()
