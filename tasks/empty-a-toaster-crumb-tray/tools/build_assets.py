"""Analytic rigid boxes: intact toaster, thin removable metal tray and real-size dry crumbs."""
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from rdb_toaster_tray import layout as L,fixed_scene as F

def fmt(v):return '('+', '.join(f'{float(x):.10g}' for x in v)+')'


def material(name,color,roughness,friction,restitution=0.,metallic=0.):
    return f'''        def Material "{name}" (prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"])
        {{
            float physics:staticFriction = {friction}
            float physics:dynamicFriction = {friction}
            float physics:restitution = {restitution}
            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>
            def Shader "Surface"
            {{
                uniform token info:id = "UsdPreviewSurface"
                color3f inputs:diffuseColor = {fmt(color)}
                float inputs:metallic = {metallic}
                float inputs:roughness = {roughness}
                token outputs:surface
            }}
        }}
'''


def cube(name,size,position,material,indent='        '):
    return f'''{indent}def Cube "{name}" (prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"])
{indent}{{
{indent}    double size = 1
{indent}    double3 xformOp:translate = {fmt(position)}
{indent}    double3 xformOp:scale = {fmt(size)}
{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
{indent}    rel material:binding = </Fixture/Materials/{material}>
{indent}    rel material:binding:physics = </Fixture/Materials/{material}>
{indent}    float physxCollision:contactOffset = {L.CONTACT_OFFSET}
{indent}    float physxCollision:restOffset = 0
{indent}}}
'''


def cylinder(name,a,b,radius,material,collide=False,indent='            '):
    a,b=np.asarray(a),np.asarray(b);d=b-a;length=np.linalg.norm(d);d=d/length
    if d[2]<-.999999:q=np.array((1.,0.,0.,0.))
    else:q=np.r_[np.cross((0.,0.,1.),d),1.+d[2]];q/=np.linalg.norm(q)
    schemas='"MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"' if collide else '"MaterialBindingAPI"'
    return f'''{indent}def Cylinder "{name}" (prepend apiSchemas = [{schemas}])
{indent}{{
{indent}    double radius = {radius}
{indent}    double height = {length:.10g}
{indent}    uniform token axis = "Z"
{indent}    double3 xformOp:translate = {fmt((a+b)/2)}
{indent}    quatf xformOp:orient = {fmt((q[3],*q[:3]))}
{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
{indent}    rel material:binding = </Fixture/Materials/{material}>
{indent}    rel material:binding:physics = </Fixture/Materials/{material}>
{indent}}}
'''


def body(name,pose,mass,inertia,children,com=(0,0,0)):
    p=pose[:3]+(0,0,F.TOP);q=pose[3:]
    return f'''        def Xform "{name}" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsMassAPI", "PhysxContactReportAPI"])
        {{
            bool physics:rigidBodyEnabled = 1
            bool physics:kinematicEnabled = 0
            bool physxRigidBody:enableCCD = 1
            bool physxRigidBody:enableSpeculativeCCD = 1
            float physics:mass = {mass:.10g}
            point3f physics:centerOfMass = {fmt(com)}
            float3 physics:diagonalInertia = {fmt(inertia)}
            quatf physics:principalAxes = (1, 0, 0, 0)
            float physxRigidBody:linearDamping = {L.LINEAR_DAMPING}
            float physxRigidBody:angularDamping = {L.ANGULAR_DAMPING}
            float physxRigidBody:maxDepenetrationVelocity = 0.5
            int physxRigidBody:solverPositionIterationCount = 64
            int physxRigidBody:solverVelocityIterationCount = 4
            float physxRigidBody:sleepThreshold = 0
            float physxContactReport:threshold = 0
            double3 xformOp:translate = {fmt(p)}
            quatf xformOp:orient = {fmt((q[3],*q[:3]))}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
'''+children+'        }\n'



def usd():
 actors=''
 # Housing is free on four physical rubber feet, never fixed to the bench.
 c=''
 for i,x in enumerate((-.065,.065)):
  for j,y in enumerate((-.145,.105)):c+=cube(f'rubber_foot_{i}_{j}',(.020,.025,.008),(x,y,.004),'rubber','            ')
 c+=cube('base_pan',(.166,.296,.006),(0.,-.020,.011),'steel','            ')
 # Long steel side housings form the two native tray side guides.
 for i,x in enumerate((-.082,.082)):
  c+=cube(f'outer_side_{i}',(.006,.300,.176),(x,-.020,.102),'cream','            ')
  c+=cube(f'tray_rail_{i}',(.010,.269,.006),(np.sign(x)*.067,-.0045,.017),'steel','            ')
  c+=cube(f'tray_side_guide_{i}',(.003,.269,.014),(np.sign(x)*.0725,-.0045,.027),'steel','            ')
 c+=cube('rear_panel',(.164,.006,.176),(0.,-.167,.102),'cream','            ')
 c+=cube('front_panel',(.164,.006,.156),(0.,.127,.112),'cream','            ')
 c+=cube('tray_rear_stop',(.140,.004,.007),(0.,-.1324,.0235),'steel','            ')
 # Actual slots remain visible in the upper panel; carriage/heat are out of scope.
 for i,x in enumerate((-.076,0.,.076)):
  sx=.016 if i!=1 else .042
  c+=cube(f'top_bridge_{i}',(sx,.300,.006),(x,-.020,.187),'steel','            ')
 for i,y in enumerate((-.145,.105)):
  c+=cube(f'top_end_{i}',(.140,.050,.006),(0.,y,.187),'steel','            ')
 for i,x in enumerate((-.042,.042)):
  c+=cube(f'dark_slot_floor_{i}',(.035,.180,.003),(x,-.020,.154),'dark','            ')
  for j,y in enumerate(np.linspace(-.100,.060,5)):
   c+=cube(f'carriage_wire_{i}_{j}',(.026,.0015,.002),(x,y,.158),'steel','            ')
 # Ordinary lever and dial are noninteractive exterior appliance features.
 c+=cube('lift_lever_slot',(.002,.055,.009),(.086,.040,.105),'dark','            ')
 c+=cube('lift_lever',(.017,.020,.008),(.093,.040,.105),'plastic','            ')
 c+=cylinder('browning_dial',(.087,-.070,.075),(.098,-.070,.075),.014,'plastic',True)
 inertia=L.MASSES[L.TOASTER]*(np.sum(L.TOASTER_SIZE**2)-L.TOASTER_SIZE**2)/12
 actors+=body('toaster',L.START_POSES[L.TOASTER],L.MASSES[L.TOASTER],inertia,c,com=(0.,-.020,.095))
 tray=''
 for i,(size,pos,mat) in enumerate(L.TRAY_PARTS):tray+=cube(f'tray_part_{i}',size,pos,mat,'            ')
 actors+=body('tray',L.START_POSES[L.TRAY],L.TRAY_MASS,L.TRAY_INERTIA,tray,com=L.TRAY_COM)
 for i,index in enumerate(L.CRUMBS):
  size=L.CRUMB_SIZES[i];inertia=L.CRUMB_MASSES[i]*(np.sum(size*size)-size*size)/12
  actors+=body(L.LINKS[index],L.START_POSES[index],L.CRUMB_MASSES[i],inertia,cube('dry_bread_chunk',size,(0.,0.,0.),f'crumb{i%4}','            '))
 hardware=''
 bc=L.BIN_CENTER
 hardware+=cube('waste_bin_floor',(*L.BIN_INNER,L.BIN_FLOOR),(*bc[:2],F.TOP+L.BIN_FLOOR/2),'bin')
 for i,(size,pos) in enumerate([
  ((.005,L.BIN_INNER[1]+.010,L.BIN_RIM),(bc[0]-L.BIN_INNER[0]/2-.0025,bc[1],F.TOP+L.BIN_RIM/2)),
  ((.005,L.BIN_INNER[1]+.010,L.BIN_RIM),(bc[0]+L.BIN_INNER[0]/2+.0025,bc[1],F.TOP+L.BIN_RIM/2)),
  ((L.BIN_INNER[0],.005,L.BIN_RIM),(bc[0],bc[1]-L.BIN_INNER[1]/2-.0025,F.TOP+L.BIN_RIM/2)),
  ((L.BIN_INNER[0],.005,L.BIN_RIM),(bc[0],bc[1]+L.BIN_INNER[1]/2+.0025,F.TOP+L.BIN_RIM/2))]):hardware+=cube(f'waste_bin_wall_{i}',size,pos,'bin')
 # Visible disconnected plug, no power outlet or hidden electrical predicate.
 hardware+=cube('unplugged_plug',(.024,.035,.020),(.585,.310,F.TOP+.010),'plastic')
 for i in (-1,1):hardware+=cube(f'plug_pin_{i+1}',(.003,.012,.004),(.585+i*.006,.333,F.TOP+.010),'steel')
 # Cord is a static visual only; it never affects actions or scoring.
 cordpoints=[(.898,.130,.024),(.905,.130,.008),(.900,.290,.004),(.740,.300,.004),(.620,.290,.004),(.585,.294,.010)]
 for i,(a,b) in enumerate(zip(cordpoints,cordpoints[1:])):
  hardware+=cylinder(f'cord_visual_{i}',np.asarray(a)+(0,0,F.TOP),np.asarray(b)+(0,0,F.TOP),.0023,'plastic',False,'        ')
 mats=material('steel',(.52,.55,.57),.30,.35,metallic=1.)+material('cream',(.72,.68,.57),.43,.35,metallic=.15)+material('dark',(.025,.025,.023),.75,.4)+material('plastic',(.055,.055,.048),.6,.6)+material('rubber',(.04,.04,.035),.9,.9)+material('bin',(.09,.21,.17),.75,.5)
 for i,color in enumerate(((.42,.23,.09),(.55,.32,.13),(.62,.42,.20),(.36,.19,.07))):mats+=material(f'crumb{i}',color,.95,.55)
 return '#usda 1.0\n(\n defaultPrim = "Fixture"\n metersPerUnit = 1\n upAxis = "Z"\n)\ndef Xform "Fixture"\n{\n def Scope "Actors"\n {\n'+actors+' }\n def Scope "Hardware"\n {\n'+hardware+' }\n def Scope "Materials"\n {\n'+mats+' }\n}\n'

if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');a=p.parse_args()
 if a.check:assert F.USD.read_text()==usd()
 else:F.USD.write_text(usd())
