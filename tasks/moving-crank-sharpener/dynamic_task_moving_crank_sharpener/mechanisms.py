"""Visible machine geometry tied to native moving bodies and contact velocities.

Only non-colliding belt markings are animated. Payloads and support rigid-body
poses are never written here. Linear carriages, rails and bases use real solids.
"""
import math
from dynamic_task_moving_crank_sharpener.geometry import Solid

SILVER=(.58,.63,.68)
BLUE=(.055,.19,.32)
BLACK=(.055,.065,.075)

def rotary_stage(stage):
    """170 mm platter, clear spindle bore, stationary bearing and index marks."""
    from yam_arena.task_file import Box
    from dynamic_task_moving_crank_sharpener.geometry import square_ring
    if any(abs(v-e)>1e-6 for v,e in zip(stage.size,(.170,.170,.012))):
        raise ValueError('This rotary fixture requires a170x170x12mm stage envelope')
    x,y,z=stage.pos;prefix=stage.name
    base=Box(prefix+'_rotary_base',pos=(x,y,z-.026),size=(.14,.14,.016),color=BLUE)
    bearing=Box(prefix+'_bearing',pos=(x,y,z-.012),size=(.06,.06,.012),color=BLACK)
    pointer=Box(prefix+'_pointer',pos=(x+.094,y,z+.006),size=(.018,.004,.006),color=(1.,.75,.04))
    ring=[];marks=[]
    for i in range(24):
        angle=2*math.pi*i/24
        ring.append(Solid('box',(.012,2*.030*math.tan(math.pi/24),.012),
                          (.024*math.cos(angle),.024*math.sin(angle),0),yaw=angle))
    for i in range(12):
        angle=2*math.pi*i/12
        marks.append(Solid('box',(.012,.002,.0002),
            (.073*math.cos(angle),.073*math.sin(angle),.0062),yaw=angle,
            color=(1.,.75,.04) if i==0 else BLACK))
    bodies={prefix:(Solid('cylinder',(.085,.012)),
                    Solid('cylinder',(.016,.028),(0,0,-.014),color=SILVER)),
            base.name:square_ring(.14,.04,.016),bearing.name:tuple(ring)}
    return [base,bearing,pointer],bodies,{prefix:tuple(marks)}

def linear_stage(stage,amplitude):
    """Keep the original deck height while adding bearing blocks and fixed rails."""
    from yam_arena.task_file import Box
    x,y,z=stage.pos;w,l,h=stage.size
    if abs(h-.03)>1e-6:raise ValueError('This low-profile rail assembly needs a 30 mm stage envelope')
    rail_length=l+2*amplitude+.04
    parts=[Solid('box',(w,l,.008),(0,0,.011),color=SILVER)]
    specs=[];bodies={}
    for side in (-1,1):
        dx=side*(w/2-.025)
        # Four bars form an actual 8 mm square bearing passage around a 7 mm rail.
        parts.extend([Solid('box',(.003,.050,.014),(dx+s*.0055,0,-.005),color=BLUE) for s in (-1,1)])
        parts.extend([Solid('box',(.008,.050,.003),(dx,0,-.005+s*.0055),color=BLUE) for s in (-1,1)])
        parts.append(Solid('box',(.014,.050,.006),(dx,0,.0045),color=BLUE))
        for end in (-1,1):
            parts.append(Solid('cylinder',(.0025,.001),(side*(w/2-.010),end*(l/2-.010),.0155),color=BLACK))
        name=stage.name+('_rail_left' if side==-1 else '_rail_right')
        specs.append(Box(name,pos=(x+dx,y,z-.005),size=(.007,rail_length,.007),color=SILVER))
        bodies[name]=(Solid('cylinder',(.0035,rail_length),axis='Y',color=SILVER),)
    specs.append(Box(stage.name+'_base',pos=(x,y,z-.01375),size=(w+.035,rail_length+.035,.0025),color=BLUE))
    for end in (-1,1):
        specs.append(Box(stage.name+('_end_front' if end==-1 else '_end_back'),
                         pos=(x,y+end*(rail_length/2+.0075),z-.004),size=(w+.035,.015,.022),color=BLUE))
    bodies[stage.name]=tuple(parts)
    return specs,bodies

def conveyor_frame(belt):
    """A tabletop conveyor with an endless-belt profile and aluminium side beams."""
    from yam_arena.task_file import Box
    x,y,z=belt.pos;w,l,h=belt.size;r=h/2;straight=l-2*r
    # Preserve the tested primitive box contact bed. Animated tread geometry
    # belongs outside its physics hierarchy, as in the supplied conveyor demo.
    bodies={}
    specs=[]
    for side in (-1,1):
        name=belt.name+('_frame_left' if side==-1 else '_frame_right')
        specs.append(Box(name,pos=(x+side*(w/2+.007),y,z-.001),size=(.012,l+.020,h-.008),color=SILVER))
        for end in (-1,1):
            name=belt.name+('_bearing_left' if side==-1 else '_bearing_right')+('_front' if end==-1 else '_back')
            specs.append(Box(name,pos=(x+side*(w/2+.009),y+end*straight/2,z),size=(.016,.026,.024),color=BLUE))
    return specs,bodies

class MechanismVisuals:
    """Native-body decorations; tread markers follow declared belt surface speed."""
    def __init__(self,scene,definitions,paths,objects,conveyors):
        from pxr import Gf,UsdGeom,UsdShade,Sdf
        self.Gf=Gf;self._belts=[];self._last_tick=-1
        def material(path,color,metallic=0.,roughness=.45):
            m=UsdShade.Material.Define(scene,path);s=UsdShade.Shader.Define(scene,path+'/shader');s.CreateIdAttr('UsdPreviewSurface')
            s.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color));s.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness)
            s.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metallic);m.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(),'surface');return m
        def cube(path,size,position,mat):
            if not path.startswith('/') or any(not p.isidentifier() for p in path.split('/')[1:]):
                raise ValueError('Machine visual path needs absolute, identifier-only components: '+path)
            g=UsdGeom.Cube.Define(scene,path);g.CreateSizeAttr(1.)
            move=g.AddTranslateOp();move.Set(Gf.Vec3d(*position));turn=g.AddRotateXOp();g.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(mat)
            return move,turn
        for name,definition in definitions.items():
            obj=objects[name];w,l,h=obj.size
            root=paths[name]+'/machine_visuals'
            if definition['kind']=='conveyor':
                root=paths[name].rsplit('/',1)[0]+'/machine_visuals_'+name
                group=UsdGeom.Xform.Define(scene,root)
                group.AddTranslateOp().Set(Gf.Vec3d(*obj.pos))
            else:UsdGeom.Xform.Define(scene,root)
            if definition['kind']=='conveyor':
                marking=material(root+'/tread_material',(.30,.34,.36),roughness=.7)
                steel=material(root+'/roller_material',SILVER,metallic=.75)
                for side in (-1,1):
                    roller=UsdGeom.Cylinder.Define(scene,root+('/roller_front' if side==-1 else '/roller_back'))
                    roller.CreateRadiusAttr(h/2-.001);roller.CreateHeightAttr(w+.014);roller.CreateAxisAttr('X')
                    roller.AddTranslateOp().Set(Gf.Vec3d(0,side*(l-h)/2,0))
                    UsdShade.MaterialBindingAPI.Apply(roller.GetPrim()).Bind(steel)
                r=h/2+.0006;straight=l-h;loop=2*straight+2*math.pi*r
                speed=float(conveyors[name][1]);count=max(16,int(loop/.025))
                ops=[cube(root+f'/tread_{i:03}',(w-.006,.004,.0004),(0,0,0),marking) for i in range(count)]
                self._belts.append((ops,r,straight,loop,speed))
            elif definition['kind']=='linear_stage':
                black=material(root+'/slot_material',BLACK);yellow=material(root+'/indicator_material',(.98,.68,.06))
                for side in (-1,1):cube(root+('/slot_left' if side==-1 else '/slot_right'),(.002,l-.018,.00012),(side*w*.28,0,h/2+.0001),black)
                cube(root+'/pointer',(.009,.018,.0002),(w/2-.005,0,h/2+.0002),yellow)
                # Fixed ruler is parented to the stage's stationary base, not its carriage.
                base_root=paths[name].rsplit('/',1)[0]+'/'+name+'_base'
                gauge=base_root+'/ruler';UsdGeom.Xform.Define(scene,gauge)
                extent=l/2+definition['amplitude']
                count=int(2*extent/.01)+1
                for i in range(count):
                    yy=-extent+i*.01
                    cube(gauge+f'/tick_{i:03}',(.010 if i%5==0 else .005,.0006,.0002),(w/2+.009,yy,.0014),yellow)
            else:raise ValueError('Unknown machine visual kind')
        self.update(0.)
    def update(self,time_s):
        tick=int(time_s*20+1e-6)
        if tick==self._last_tick:return
        self._last_tick=tick
        for ops,r,straight,loop,speed in self._belts:
            for i,(move,turn) in enumerate(ops):
                s=(i*loop/len(ops)-speed*time_s)%loop
                if s<straight:y=straight/2-s;z=r;angle=0.
                elif s<straight+math.pi*r:
                    theta=(s-straight)/r;y=-straight/2-r*math.sin(theta);z=r*math.cos(theta);angle=theta
                elif s<2*straight+math.pi*r:y=-straight/2+(s-straight-math.pi*r);z=-r;angle=0.
                else:
                    theta=(s-2*straight-math.pi*r)/r;y=straight/2+r*math.sin(theta);z=-r*math.cos(theta);angle=theta
                move.Set(self.Gf.Vec3d(0,y,z));turn.Set(math.degrees(angle))

def anatomical_hand():
    """Rounded palm and three phalanges per finger, with a clear forearm envelope."""
    skin=(.60,.36,.23)
    parts=[Solid('capsule_x',(.022,.16),(.13,0,-.010),color=skin)]
    parts.extend(Solid('capsule_x',(.0125,.060),(0,y,0),color=skin) for y in (-.025,0,.025))
    for y,lengths in [(.027,(.024,.021,.015)),(.009,(.028,.025,.018)),(-.009,(.026,.023,.016)),(-.026,(.021,.017,.014))]:
        edge=-.0425
        for j,length in enumerate(lengths):
            radius=.0075-j*.0007
            parts.append(Solid('capsule_x',(radius,max(.001,length-2*radius)),(edge-length/2,y,.001),color=skin))
            edge-=length-.001
    parts.append(Solid('capsule_x',(.010,.045),(-.005,-.050,.006),yaw=-.65,color=skin))
    return tuple(parts)
