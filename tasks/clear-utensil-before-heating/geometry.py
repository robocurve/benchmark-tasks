"""Locally authored cookware and single hotplate, in metres; no external downloads."""
from pathlib import Path
import math
import numpy as np
import usdkit as u
HERE=Path(__file__).parent
BURNER_RADIUS=.11
LOOKS={'steel':u.Look((.38,.4,.42),metallic=.9,roughness=.3),
       'black':u.Look((.025,.025,.03),roughness=.65),
       'white':u.Look((.85,.85,.8),roughness=.5),
       'red':u.Look((.5,.005,.002),emissive=(.08,0,0)),
       'grey':u.Look((.13,.14,.15),metallic=.6,roughness=.4),
       'green':u.Look((.03,.4,.04),roughness=.6),
       'panel':u.Look((1,1,1),metallic=.5,roughness=.4,texture='knob-panel.png',tile=.10)}

def cyl_col(name,r,h,z,root):
    return f'\n    def Cylinder "{name}" (prepend apiSchemas=["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]) {{\n rel material:binding:physics=</{root}/PhysicsMaterial>\n uniform token purpose="guide"\n double radius={r}\n double height={h}\n uniform token axis="Z"\n double3 xformOp:translate=(0,0,{z})\n uniform token[] xformOpOrder=["xformOp:translate"]\n float physxCollision:contactOffset=.004\n float physxCollision:restOffset=0\n }}\n'

def build():
    global PAN_TRIANGLES, HOTPLATE_TRIANGLES, KNOB_BASE_TRIANGLES, KNOB_MOVING_TRIANGLES
    global PAN_COMPONENT_POINTS,HOTPLATE_COMPONENT_POINTS,KNOB_BASE_COMPONENT_POINTS,KNOB_MOVING_COMPONENT_POINTS
    root='Pan'
    # Open metal vessel with 3 mm bottom and 2 mm wall, diameter 16 cm.
    profile=[(0,0),(.0775,0),(.08,.003),(.08,.058),(.079,.060),(.078,.058),(.078,.003),(0,.003)]
    parts=[('vessel',u.lathe(profile,48),'steel'),
      ('neck',u.box((.024,.038,.015),center=(0,-.091,.049),bevel=.002),'steel'),
      ('handle',u.box((.026,.105,.020),center=(0,-.151,.050),bevel=.001,round_xy=.003),'black')]
    PAN_TRIANGLES=np.concatenate([p[1].v[p[1].f] for p in parts])
    PAN_COMPONENT_POINTS=np.array([p[1].v[0] for p in parts])
    cols=[]
    # Circular bottom and 24 tangential wall segments keep visual error below 2 mm.
    cols.append(cyl_col('bottom',.0785,.003,.0015,root))
    for i in range(24):
        a=i*2*math.pi/24
        # Rotated guide cube; inner side .078 m, outer .080 m.
        s=u.collider(f'wall{i}',(.002,.021,.055),(.079,0,.0305),root)
        s=s.replace('double3 xformOp:translate = (0.079000, 0.000000, 0.030500)',
          f'double3 xformOp:translate = ({.079*math.cos(a):.6f}, {.079*math.sin(a):.6f}, 0.0305)\n        float xformOp:rotateZ = {math.degrees(a):.6f}')
        s=s.replace('["xformOp:translate", "xformOp:scale"]','["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]')
        cols.append(s)
    cols += [u.collider('neck_col',(.024,.038,.015),(0,-.091,.049),root),u.collider('handle_col',(.026,.105,.020),(0,-.151,.050),root)]
    u.write_usda(str(HERE/'pan.usda'),root,parts,LOOKS,cols,mass=.4,friction=(.9,.8))
    # Apache-2 furniture_sim meshes, uniformly scaled in metres.
    import trimesh
    burner=trimesh.load(HERE/'burnerplate.stl',force='mesh')
    scale=(2*BURNER_RADIUS)/float(np.ptp(burner.vertices[:,0]))
    bv=burner.vertices*scale; bv[:,2]-=bv[:,2].min();bv[:,2]+=.004
    global BURNER_TOP
    BURNER_TOP=float(bv[:,2].max())
    bm=u.Mesh(bv,np.asarray(burner.faces))
    parts=[('common_base',u.box((.36,.25,.004),center=(-.05,0,.002),bevel=.001),'steel'),
      ('library_burner',bm,'black'),
      ('heatspreader',u.cylinder(BURNER_RADIUS,BURNER_TOP-.004-.00075,center=(0,0,(BURNER_TOP+.004-.00075)/2),segments=96),'black')]
    HOTPLATE_TRIANGLES=np.concatenate([m.v[m.f] for _,m,_ in parts])
    HOTPLATE_COMPONENT_POINTS=np.array([m.v[0] for _,m,_ in parts])
    # Kinematic original ring mesh uses exact triangle collision; no filled hull.
    cols=[u.collider('base_col',(.36,.25,.004),(-.05,0,.002),'Hotplate'),cyl_col('spreader',BURNER_RADIUS,BURNER_TOP-.004-.00075,(BURNER_TOP+.004-.00075)/2,'Hotplate')]
    meshcol=u._mesh_usda('library_ring_collision',bm,'black','Hotplate',LOOKS['black'],45,hidden=True)
    meshcol=meshcol.replace('["MaterialBindingAPI"]','["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxCollisionAPI"]')
    meshcol=meshcol.replace('uniform bool doubleSided = 0','uniform bool doubleSided = 0\n token physics:approximation="none"\n float physxCollision:contactOffset=.004\n float physxCollision:restOffset=0')
    cols.append(meshcol)
    u.write_usda(str(HERE/'hotplate.usda'),'Hotplate',parts,LOOKS,cols,mass=1.)
    parts=[('OFF_ON_panel',u.box((.10,.10,.0001),center=(0,0,.00015)),'panel'),('mount',u.cylinder(.031,.010,center=(0,0,.005),segments=48),'steel')]
    for deg in (0,30,60,75,90):
      a=-math.radians(deg)
      parts.append((f'tick{deg}',u.box((.005,.002,.001)).turned(-deg).moved((.039*math.cos(a),.039*math.sin(a),.005)),'green' if deg>=60 else 'white'))
    KNOB_BASE_TRIANGLES=np.concatenate([m.v[m.f] for _,m,_ in parts])
    KNOB_BASE_COMPONENT_POINTS=np.array([m.v[0] for _,m,_ in parts])
    u.write_usda(str(HERE/'knob-base.usda'),'Knob',parts,LOOKS,[cyl_col('mount_col',.031,.01,.005,'Knob')],mass=1.)
    km=trimesh.load(HERE/'knob.stl',force='mesh');kv=km.vertices*.56
    kv[:,2]-=kv[:,2].min();kv[:,2]+=.010
    # Native long grip y becomes x to match open-loop wrist yaw=0.
    kv=kv@np.array([[0,-1,0],[1,0,0],[0,0,1]]).T
    grip=u.Mesh(kv,np.asarray(km.faces))
    parts=[('library_knob',grip,'black'),('pointer',u.box((.008,.003,.001),center=(.014,0,.0407)),'white')]
    KNOB_MOVING_TRIANGLES=np.concatenate([m.v[m.f] for _,m,_ in parts])
    KNOB_MOVING_COMPONENT_POINTS=np.array([m.v[0] for _,m,_ in parts])
    # Disk plus two <=64-vertex convex pieces of the actual tapered upper grip.
    cols=[cyl_col('disc_col',.0273,.010,.015,'Moving')]
    bar=np.load(HERE/'knob-bar-hulls.npz')
    for i in range(2):
      text=u._mesh_usda('bar_hull'+str(i),u.Mesh(bar['v'+str(i)],bar['f'+str(i)]),'black','Moving',LOOKS['black'],45,hidden=True)
      text=text.replace('["MaterialBindingAPI"]','["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxCollisionAPI", "PhysxConvexHullCollisionAPI"]')
      text=text.replace('uniform bool doubleSided = 0','uniform bool doubleSided = 0\n token physics:approximation="convexHull"\n int physxConvexHullCollision:hullVertexLimit=64\n float physxConvexHullCollision:minThickness=.00001\n float physxCollision:contactOffset=.004\n float physxCollision:restOffset=0')
      cols.append(text)
    u.write_usda(str(HERE/'knob-moving.usda'),'Moving',parts,LOOKS,cols,mass=.05)
    base=(HERE/'knob-base.usda').read_text();body=(HERE/'knob-moving.usda').read_text()
    base=base[base.index('def Xform'):].replace('def Xform "Knob"','def Xform "base"',1).replace('/Knob/','/Knob/base/')
    base=base.replace('"PhysicsRigidBodyAPI",','"PhysicsArticulationRootAPI", "PhysicsRigidBodyAPI",',1)
    body=body[body.index('def Xform'):].replace('def Xform "Moving"','def Xform "moving"',1).replace('/Moving/','/Knob/moving/')
    joints='''
 def PhysicsFixedJoint "fixed" {rel physics:body0=</Knob>
rel physics:body1=</Knob/base>
}
 def PhysicsRevoluteJoint "turn" (prepend apiSchemas=["PhysicsDriveAPI:angular"]) {
rel physics:body0=</Knob/base>
rel physics:body1=</Knob/moving>
uniform token physics:axis="Z"
quatf physics:localRot0=(0,1,0,0)
quatf physics:localRot1=(0,1,0,0)
float physics:lowerLimit=0
float physics:upperLimit=90
float drive:angular:physics:stiffness=0
float drive:angular:physics:damping=.01
float drive:angular:physics:maxForce=5
uniform token drive:angular:physics:type="force"
}
'''
    (HERE/'knob.usda').write_text('#usda 1.0\n(defaultPrim="Knob"\nmetersPerUnit=1\nupAxis="Z")\ndef Xform "Knob" {\n'+base+body+joints+'\n}')

if __name__=='__main__':build()
