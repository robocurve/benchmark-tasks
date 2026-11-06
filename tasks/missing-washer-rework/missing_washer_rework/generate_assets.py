# SPDX-License-Identifier: MIT
"""Regenerate original compound rigid assets without simulator dependencies."""
from pathlib import Path

def cube(name,size,pos,color):
 return f'''    def Cube "{name}" (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"])
    {{
        double size = 1
        double3 xformOp:translate = {tuple(pos)}
        double3 xformOp:scale = {tuple(size)}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        color3f[] primvars:displayColor = [{tuple(color)}]
        float physxCollision:contactOffset = 0.0005
        float physxCollision:restOffset = 0
    }}
'''

def asset(parts,mass,position_iterations,velocity_iterations):
 return '''#usda 1.0
(
    defaultPrim = "Part"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Part" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"])
{
    bool physics:rigidBodyEnabled = true
    bool physics:kinematicEnabled = false
'''+f'    float physics:mass = {mass}\n'+'''    float physxRigidBody:maxDepenetrationVelocity = 1
    int physxRigidBody:solverPositionIterationCount = 32
    int physxRigidBody:solverVelocityIterationCount = 8
'''.replace('solverPositionIterationCount = 32',f'solverPositionIterationCount = {position_iterations}').replace('solverVelocityIterationCount = 8',f'solverVelocityIterationCount = {velocity_iterations}')+''.join(cube(*part) for part in parts)+'}\n'

def generate():
 out=Path(__file__).parent/'assets';out.mkdir(exist_ok=True)
 import runpy
 from types import SimpleNamespace
 c=SimpleNamespace(**runpy.run_path(str(Path(__file__).with_name('contract.py'))))
 blue=(.08,.32,.82);orange=(1.,.32,.03);grey=(.52,.55,.60)
 fixture=[('Base',c.FIXTURE_SIZE,(0,0,0),blue),('Extension',c.FIXTURE_EXTENSION_SIZE,c.FIXTURE_EXTENSION_LOCAL,blue),('Post',c.POST_SIZE,c.POST_CENTER_LOCAL,c.POST_COLOR),
          ('HandleStem',(.024,.020,.018),(0,c.FIXTURE_GRIP_LOCAL[1],.017),blue),
          ('Handle',c.FIXTURE_GRIP_SIZE,c.FIXTURE_GRIP_LOCAL,blue)]
 outer=c.WASHER_SIZE[0];hole=c.WASHER_HOLE;thick=c.WASHER_SIZE[2]
 wall=(outer-hole)/2;offset=(outer+hole)/4
 washer=[('Left',(wall,outer,thick),(-offset,0,0),orange),('Right',(wall,outer,thick),(offset,0,0),orange),
         ('Near',(hole,wall,thick),(0,-offset,0),orange),('Far',(hole,wall,thick),(0,offset,0),orange),
         ('Grip',c.WASHER_GRIP_SIZE,c.WASHER_GRIP_LOCAL,orange)]
 outer=c.CAP_SIZE[0];hole=c.CAP_HOLE;height=c.CAP_SIZE[2];cavity=c.CAP_CAVITY_HEIGHT
 wall=(outer-hole)/2;offset=(outer+hole)/4
 cap=[('Left',(wall,outer,cavity),(-offset,0,cavity/2),grey),('Right',(wall,outer,cavity),(offset,0,cavity/2),grey),
      ('Near',(hole,wall,cavity),(0,-offset,cavity/2),grey),('Far',(hole,wall,cavity),(0,offset,cavity/2),grey),
      ('Roof',(outer,outer,height-cavity),(0,0,(height+cavity)/2),grey),
      ('Grip',c.CAP_GRIP_SIZE,c.CAP_GRIP_LOCAL,grey)]
 reference=[(name,size,pos,c.POST_COLOR if name=='Post' else (.20,.24,.30)) for name,size,pos,color in fixture]
 for name,parts,mass in [('fixture',fixture,.30),('reference_fixture',reference,.30),('washer',washer,.035),('cap',cap,.045)]:
  (out/(name+'.usda')).write_text(asset(parts,mass,c.POSITION_ITERATIONS,c.VELOCITY_ITERATIONS),encoding='utf-8',newline='\n')
if __name__=='__main__':generate()
