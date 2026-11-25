"""Original open V supports from two convex HDPE halves, CC0-1.0."""
from pathlib import Path
import json
root=Path(__file__).resolve().parent
text='#usda 1.0\n(defaultPrim="RecoveryRests"\n metersPerUnit=1\n upAxis="Z")\ndef Xform "RecoveryRests" {\n'
RISE=.035
profiles={'LeftWing':[(-.010,0),(0,0),(0,.0173),(-.010,.0273)],
          'RightWing':[(0,0),(.010,0),(.010,.0273),(0,.0173)]}
for name,y in [('Rest0',0),('Rest1',-.150)]:
 text+=f' def Xform "{name}" {{\n double3 xformOp:translate=(0,{y},0)\n uniform token[] xformOpOrder=["xformOp:translate"]\n'
 text+=f'''  def Cube "Pedestal" (prepend apiSchemas=["PhysicsCollisionAPI"]) {{
   double size=1
   double3 xformOp:translate=(0,0,{RISE/2})
   double3 xformOp:scale=(.020,.028,{RISE})
   uniform token[] xformOpOrder=["xformOp:translate","xformOp:scale"]
   bool physics:collisionEnabled=true
   color3f[] primvars:displayColor=[(0.16,0.55,0.58)]
  }}
'''
 for side,profile in profiles.items():
  points=[(x,dy,z+RISE)for dy in(-.014,.014)for x,z in profile]
  faces=[[0,1,2,3],[7,6,5,4]]+[[i,i+4,(i+1)%4+4,(i+1)%4]for i in range(4)]
  vertices=', '.join('(%g,%g,%g)'%p for p in points);counts=', '.join(str(len(f))for f in faces);indices=', '.join(str(i)for f in faces for i in f)
  text+=f'''  def Mesh "{side}" (prepend apiSchemas=["PhysicsCollisionAPI","PhysicsMeshCollisionAPI"]) {{
   point3f[] points=[{vertices}]
   int[] faceVertexCounts=[{counts}]
   int[] faceVertexIndices=[{indices}]
   uniform token subdivisionScheme="none"
   token physics:approximation="convexHull"
   bool physics:collisionEnabled=true
   color3f[] primvars:displayColor=[(0.16,0.55,0.58)]
  }}
'''
 text+=' }\n'
text+='}\n';(root/'rests.usda').write_text(text)
(root/'dimensions.json').write_text(json.dumps({'units':'m','width':.020,'depth':.028,'height':.0273+RISE,'pedestal_height':RISE,'groove_bottom_height':.0173+RISE,'groove_angle_degrees':90,'spacing':.150,'shaft_diameter':.006,'shaft_axis_height':.0173+RISE+2**.5*.003,'material':'Rigid HDPE','construction':'Two convex halves per open V receiving rest; no clamp, drive, joint or attachment.'},indent=2)+'\n')
