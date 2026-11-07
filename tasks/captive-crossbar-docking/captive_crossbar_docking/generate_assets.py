# SPDX-License-Identifier: MIT
"""Generate original primitive USD assets; no simulator dependencies.

Floating-articulation layout follows the MIT bevel-angle-transfer example.
All task geometry is read from contract.py rather than duplicated here.
"""
from pathlib import Path
import runpy
from types import SimpleNamespace


def cube(name, size, pos, color, collide=True):
    api = ' (prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"])' if collide else ''
    collision = """        float physxCollision:contactOffset = 0.0005
        float physxCollision:restOffset = 0
""" if collide else ''
    return f'''    def Cube "{name}"{api}
    {{
        double size = 1
        double3 xformOp:translate = {tuple(pos)}
        double3 xformOp:scale = {tuple(size)}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        color3f[] primvars:displayColor = [{tuple(color)}]
{collision}    }}
'''


def header(root):
    return f'''#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)
'''


def generate():
    c = SimpleNamespace(**runpy.run_path(str(Path(__file__).with_name('contract.py'))))
    out = Path(__file__).parent / 'assets'
    out.mkdir(exist_ok=True)
    root_api = '"PhysicsArticulationRootAPI", "PhysxArticulationAPI", '
    common_api = '"PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsMassAPI"'
    body = ''
    for name, parts, mass, offset in (
        ('carrier', c.CARRIER_PARTS, c.CARRIER_MASS, (0.,0.,0.)),
        ('crossbar', c.CROSSBAR_PARTS, c.CROSSBAR_MASS, c.CROSSBAR_ORIGIN_LOCAL),
    ):
        api = (root_api if name == 'carrier' else '') + common_api
        body += f'''    def Xform "{name}" (prepend apiSchemas = [{api}])
    {{
        double3 xformOp:translate = {offset}
        uniform token[] xformOpOrder = ["xformOp:translate"]
        float physics:mass = {mass}
        float physxRigidBody:maxDepenetrationVelocity = 1
'''
        if name == 'carrier':
            body += f'''        bool physxArticulation:enabledSelfCollisions = false
        int physxArticulation:solverPositionIterationCount = {c.POSITION_ITERATIONS}
        int physxArticulation:solverVelocityIterationCount = {c.VELOCITY_ITERATIONS}
'''
        body += ''.join(cube(*part) for part in parts)
        if name == 'carrier':
            # This physical witness band reaches the mouth at correct depth.
            body += cube('DepthWitness', (.006,.024,.0004),
                         (-.095,0.,.0083), c.WHITE, False)
            # Wide white centre bands remain visible fore/aft of orange bar.
            for x in (-.055,.055):
                body += cube('CentreBand' + ('Rear' if x < 0 else 'Front'),
                             (.045,.008,.0004), (x,0.,.0083), c.WHITE, False)
            # High contrast end boundaries make the desired full bar envelope visible.
            for y in (-.067,.067):
                body += cube('EdgeBand' + ('Left' if y > 0 else 'Right'),
                             (.165,.004,.0004), (0.,y,.0084), c.WHITE, False)
        else:
            body += cube('GripCentreBand', (.0205,.007,.0004),
                         (0.,0.,.0383), c.WHITE, False)
        body += '    }\n'
    body += f'''    def PhysicsPrismaticJoint "crossbar_slide" (
        prepend apiSchemas = ["PhysxJointAxisAPI:linear", "PhysicsDriveAPI:linear"]
    )
    {{
        rel physics:body0 = </Carrier/carrier>
        rel physics:body1 = </Carrier/crossbar>
        uniform token physics:axis = "Y"
        point3f physics:localPos0 = {c.CROSSBAR_ORIGIN_LOCAL}
        point3f physics:localPos1 = (0,0,0)
        float physics:lowerLimit = {c.SLIDE_LIMITS[0]}
        float physics:upperLimit = {c.SLIDE_LIMITS[1]}
        bool physics:collisionEnabled = false
        float physxJointAxis:linear:staticFrictionEffort = {c.CARRIER_STATIC_FRICTION}
        float physxJointAxis:linear:dynamicFrictionEffort = {c.CARRIER_DYNAMIC_FRICTION}
        float physxJointAxis:linear:viscousFrictionCoefficient = {c.CARRIER_VISCOUS_FRICTION}
        uniform token drive:linear:physics:type = "force"
        float drive:linear:physics:stiffness = 0
        float drive:linear:physics:targetVelocity = 0
        float drive:linear:physics:damping = {c.CARRIER_DAMPING}
        float drive:linear:physics:maxForce = {c.CARRIER_MAX_FORCE}
    }}
'''
    (out/'carrier.usda').write_text(header('Carrier') + 'def Xform "Carrier"\n{\n' + body + '}\n', encoding='utf-8', newline='\n')
    dock = header('Dock') + '''def Xform "Dock" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsMassAPI"])
{
    bool physics:rigidBodyEnabled = true
    bool physics:kinematicEnabled = true
    float physics:mass = 1
''' + ''.join(cube(*part) for part in c.DOCK_PARTS) + '}\n'
    (out/'dock.usda').write_text(dock, encoding='utf-8', newline='\n')


if __name__ == '__main__':
    generate()
