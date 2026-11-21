"""Original full-scale dryer with free folding hinge and a bound movable coil."""
from pathlib import Path
import sys,argparse
import numpy as np
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rdb_hairdryer import layout as L,fixed_scene as F
def fmt(a):return '('+', '.join(f'{float(x):.12g}' for x in a)+')'
def material(name,color,mu):
 return f'''    def Material "{name}" (prepend apiSchemas=["PhysicsMaterialAPI", "MaterialBindingAPI"])
    {{
        float physics:staticFriction={mu}
        float physics:dynamicFriction={mu}
        float physics:restitution=0
        token outputs:surface.connect=</Fixture/{name}/Surface.outputs:surface>
        def Shader "Surface" {{ uniform token info:id="UsdPreviewSurface"
            color3f inputs:diffuseColor={fmt(color)}
            float inputs:roughness=.45
            token outputs:surface
        }}
    }}
'''
def binding(mat):return f'            rel material:binding=</Fixture/{mat}>\n            rel material:binding:physics=</Fixture/{mat}>\n'
def mesh(name,pts,mat):
 pts=np.asarray(pts);h=ConvexHull(pts);faces=[]
 for f,e in zip(h.simplices,h.equations):
  f=list(map(int,f));a,b,c=pts[f]
  if np.dot(np.cross(b-a,c-a),e[:3])<0:f.reverse()
  faces.extend(f)
 return f'''            def Mesh "{name}" (prepend apiSchemas=["MaterialBindingAPI","PhysicsCollisionAPI","PhysxCollisionAPI","PhysicsMeshCollisionAPI"])
            {{
                point3f[] points=[{', '.join(fmt(p*1000) for p in pts)}]
                int[] faceVertexCounts={[3]*len(h.simplices)}
                int[] faceVertexIndices={faces}
                uniform token subdivisionScheme="none"
                token physics:approximation="convexHull"
                double3 xformOp:scale=(.001,.001,.001)
                uniform token[] xformOpOrder=["xformOp:scale"]
                float physxCollision:contactOffset=.00015
                float physxCollision:restOffset=0
'''+binding(mat)+'            }\n'
def cube(name,size,p,mat,collide=True):
 schemas='"MaterialBindingAPI","PhysicsCollisionAPI","PhysxCollisionAPI"' if collide else '"MaterialBindingAPI"'
 return f'''            def Cube "{name}" (prepend apiSchemas=[{schemas}])
            {{
                double size=1
                double3 xformOp:scale={fmt(size)}
                double3 xformOp:translate={fmt(p)}
                uniform token[] xformOpOrder=["xformOp:translate","xformOp:scale"]
                float physxCollision:contactOffset=.00015
                float physxCollision:restOffset=0
'''+binding(mat)+'            }\n'
def axial(name,x0,x1,r0,r1,mat,n=24):
 return mesh(name,[(x,r*np.cos(t),r*np.sin(t)) for x,r in [(x0,r0),(x1,r1)] for t in np.linspace(0,2*np.pi,n,endpoint=False)],mat)
def ring(name,inner,outer,half,axis,mat,centre=(0,0,0),n=24):
 out='';centre=np.array(centre)
 for j in range(n):
  pts=[]
  for z in [-half,half]:
   for r in [inner,outer]:
    for a in [2*np.pi*j/n,2*np.pi*(j+1)/n]:
     v=[r*np.cos(a),r*np.sin(a),z] if axis=='Z' else [r*np.cos(a),z,r*np.sin(a)]
     pts.append(np.array(v)+centre)
  out+=mesh(f'{name}_{j}',pts,mat)
 return out
def capsule(name):
 return f'''            def Capsule "{name}" (prepend apiSchemas=["MaterialBindingAPI","PhysicsCollisionAPI","PhysxCollisionAPI"])
            {{
                double radius={L.CABLE_RADIUS}
                double height={L.LINK_LENGTH-2*L.CABLE_RADIUS}
                uniform token axis="Z"
                float physxCollision:contactOffset=.00015
                float physxCollision:restOffset=0
'''+binding('cord')+'            }\n'
def body(i,parts,com,I):
 p=L.START_POSES[i,:3]+[0,0,F.TOP];q=L.START_POSES[i,3:]
 return f'''        def Xform "{L.LINKS[i]}" (prepend apiSchemas=["PhysicsRigidBodyAPI","PhysicsMassAPI","PhysxRigidBodyAPI","PhysxContactReportAPI"])
        {{
            bool physics:rigidBodyEnabled=1
            bool physics:kinematicEnabled=0
            float physics:mass={L.MASSES[i]}
            point3f physics:centerOfMass={fmt(com)}
            float3 physics:diagonalInertia={fmt(I)}
            quatf physics:principalAxes=(1,0,0,0)
            float physxRigidBody:linearDamping=.02
            float physxRigidBody:angularDamping=.10
            float physxRigidBody:sleepThreshold=0
            bool physxRigidBody:enableCCD=1
            float physxRigidBody:maxDepenetrationVelocity=.5
            int physxRigidBody:solverPositionIterationCount=32
            int physxRigidBody:solverVelocityIterationCount=8
            float physxContactReport:threshold=0
            double3 xformOp:translate={fmt(p)}
            quatf xformOp:orient={fmt((q[3],*q[:3]))}
            uniform token[] xformOpOrder=["xformOp:translate","xformOp:orient"]
'''+parts+'        }\n'
def ball(name,a,b,pa,pb):
 return f'''    def PhysicsSphericalJoint "{name}"
    {{
        rel physics:body0=</Fixture/Actors/{a}>
        rel physics:body1=</Fixture/Actors/{b}>
        point3f physics:localPos0={fmt(pa)}
        point3f physics:localPos1={fmt(pb)}
        float physics:coneAngle0Limit=179.9
        float physics:coneAngle1Limit=179.9
        uniform bool physics:excludeFromArticulation=1
        bool physics:collisionEnabled=0
    }}
'''
def usd():
 head=axial('outlet',-.077,-.060,.025,.035,'white')+axial('main_barrel',-.060,.050,.035,.035,'white')+axial('rear_motor',.050,.077,.0395,.0395,'silver')
 # One millimetre clearance per side admits the full real grip sweep;
 # zero cheek/grip clearance would create an unintended friction clamp.
 for s in [-1,1]:head+=cube('hinge_clevis_'+str(s+1),(.026,.010,.024),(.035,s*.021,-.0425),'white')
 # The pin is a native convex solid; the handle eye below has a real 0.5 mm bore clearance.
 pin=[(.035+.0075*np.cos(a),y,-.050+.0075*np.sin(a)) for y in [-.026,.026] for a in np.linspace(0,2*np.pi,24,endpoint=False)]
 head+=mesh('hinge_pin',pin,'silver')
 head+=cube('rear_grille',(.001,.048,.048),(.0775,0,0),'dark',False)
 handle=ring('hinge_eye',.008,.010,.014,'Y','white')
 handle+=cube('grip',(.028,.030,.0385),(0,0,-.03125),'white')
 handle+=cube('power_switch',(.007,.031,.014),(.0145,0,-.027),'blue',False)
 handle+=cube('cord_strain_relief',(.012,.014,.009),(0,0,-.046),'cord')
 actors=body(0,head,(.008,0,-.003),(.00023,.00063,.00063))+body(1,handle,(0,0,-.027),(.000025,.000023,.000013))
 for i in range(6):actors+=body(i+2,capsule('rubber_cable'),(0,0,0),(2.64e-7,2.64e-7,3.91e-9))
 coil=ring('bound_coil',.030,.040,.0125,'Z','cord',n=24)
 coil+=cube('tied_plug',(.040,.025,.018),(.052,0,0),'dark')
 coil+=cube('strap_top',(.014,.081,.001),(0,0,.013),'blue',False)
 coil+=cube('strap_bottom',(.014,.081,.001),(0,0,-.013),'blue',False)
 actors+=body(8,coil,(.006,0,0),(.000048,.000065,.000090))
 hinge=f'''    def PhysicsRevoluteJoint "folding_hinge" (prepend apiSchemas=["PhysicsDriveAPI:angular"])
    {{
        rel physics:body0=</Fixture/Actors/head>
        rel physics:body1=</Fixture/Actors/handle>
        point3f physics:localPos0={fmt(L.HINGE)}
        point3f physics:localPos1=(0,0,0)
        uniform token physics:axis="Y"
        float physics:lowerLimit=0
        float physics:upperLimit={L.HINGE_LIMIT}
        uniform bool physics:excludeFromArticulation=1
        bool physics:collisionEnabled=1
        token drive:angular:physics:type="force"
        float drive:angular:physics:stiffness=0
        float drive:angular:physics:damping={L.HINGE_DAMPING}
        float drive:angular:physics:targetVelocity=0
        float drive:angular:physics:maxForce={L.HINGE_FRICTION_CAP}
    }}
'''
 joints=hinge+ball('loose_tail_0','handle','cord_0',L.CABLE_ANCHOR_LOCAL,(0,0,-.025))
 for i in range(5):joints+=ball('loose_tail_'+str(i+1),f'cord_{i}',f'cord_{i+1}',(0,0,.025),(0,0,-.025))
 joints+=ball('bound_tail','cord_5','cord_bundle',(0,0,.025),(-.04,0,0))
 return '#usda 1.0\n( defaultPrim="Fixture"\n metersPerUnit=1\n upAxis="Z"\n)\ndef Xform "Fixture"\n{\n'+material('white',(.86,.88,.90),.4)+material('silver',(.48,.52,.56),.4)+material('dark',(.08,.09,.10),.4)+material('cord',(.06,.06,.065),.5)+material('blue',(.12,.30,.55),.4)+'    def Scope "Actors"\n    {\n'+actors+'    }\n'+joints+'}\n'
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');a=p.parse_args();s=usd()
 if a.check:assert F.USD.read_text()==s
 else:F.USD.write_text(s)
