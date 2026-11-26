"""Generate the original, self-contained laptop USD from the declared geometry."""

import math
from pathlib import Path
from textwrap import indent

from geometry import CHASSIS_Z, COM, DISPLAY_Z, FEET, FOOT_X, FOOT_Y, FRONT_X, HINGE_LIMIT, MASS, REAR_X, WIDTH


def vector(values):
    return '(' + ', '.join(f'{v:.8g}' for v in values) + ')'


def cube(name, center, size, *, color=None, material=None):
    schemas = '' if color is not None else ''' (
    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]
)'''
    appearance = (f'color3f[] primvars:displayColor = [{vector(color)}]' if color is not None else f'''
    token visibility = "invisible"
    float physxCollision:contactOffset = 0.0005
    float physxCollision:restOffset = 0
    rel material:binding:physics = </Laptop/{material}>
    ''')
    return f'''def Cube "{name}"{schemas}
{{
    double size = 2
    double3 xformOp:translate = {vector(center)}
    double3 xformOp:scale = {vector(v / 2 for v in size)}
    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    {appearance.strip()}
}}
'''


def rounded_panel(name, xlim, ylim, zlim, radius, color):
    """Two rims with a small bevel. Frame, glass and touchpad share the contour code."""
    bevel = min((zlim[1] - zlim[0]) / 3, radius / 2, 0.0006)
    points = []
    for z, inset in ((zlim[0], bevel), (zlim[0] + bevel, 0.), (zlim[1] - bevel, 0.), (zlim[1], bevel)):
        rad = radius - inset
        for x, y, direction in (
            (xlim[1] - radius, ylim[1] - radius, 0), (xlim[0] + radius, ylim[1] - radius, 90),
            (xlim[0] + radius, ylim[0] + radius, 180), (xlim[1] - radius, ylim[0] + radius, 270),
        ):
            for angle in range(direction, direction + 91, 30):
                theta = math.radians(angle)
                points.append((x + rad * math.cos(theta), y + rad * math.sin(theta), z))
    ring = len(points) // 4
    faces = [list(reversed(range(ring))), list(range(3 * ring, 4 * ring))]
    for row in range(3):
        for i in range(ring):
            j = (i + 1) % ring
            faces.append([row * ring + i, row * ring + j, (row + 1) * ring + j, (row + 1) * ring + i])
    return f'''def Mesh "{name}"
{{
    uniform token subdivisionScheme = "none"
    point3f[] points = [{', '.join(map(vector, points))}]
    int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]
    int[] faceVertexIndices = [{', '.join(str(i) for face in faces for i in face)}]
    color3f[] primvars:displayColor = [{vector(color)}]
}}
'''


def shell(bounds, color):
    thick = bounds[1] - bounds[0]
    return (
        rounded_panel('Case', (FRONT_X, REAR_X), (-WIDTH / 2, WIDTH / 2), bounds, 0.005, color)
        + cube('CaseContact', ((FRONT_X + REAR_X) / 2, 0, sum(bounds) / 2),
               (REAR_X - FRONT_X, WIDTH, thick), material='ShellFriction')
    )


def laptop_base():
    parts = [shell(CHASSIS_Z, (0.085, 0.155, 0.175))]
    for i, (x, y) in enumerate((x, y) for x in FOOT_X for y in FOOT_Y):
        center = (x, y, CHASSIS_Z[0] - FEET[2] / 2)
        parts.extend((cube(f'Foot{i}', center, FEET, color=(0.034, 0.039, 0.043)),
                      cube(f'FootContact{i}', center, FEET, material='FootFriction')))
    parts.append(rounded_panel('KeyboardWell', (-0.126, -0.031), (-0.138, 0.138),
                               (0.00002, 0.00016), 0.002, (0.035, 0.051, 0.058)))
    for row in range(6):
        x = -0.038 - row * 0.0156
        if row == 5:
            keys = [(-0.124, 0.021), (-0.099, 0.022), (-0.075, 0.019), (0.0, 0.119),
                    (0.074, 0.020), (0.099, 0.020), (0.124, 0.019)]
        else:
            keys = [(i * 0.0185, 0.0167) for i in range(-7, 8)]
        for key, (y, width) in enumerate(keys):
            name = f'Key_{row}_{key}'
            parts.append(cube(name, (x, y, 0.00068), (0.0126, width, 0.00095), color=(0.024, 0.034, 0.040)))
            if not (row == 5 and key == 3):
                parts.append(cube(name + 'Legend', (x, y - 0.001, 0.00118), (0.0012, 0.0022, 0.00004),
                                  color=(0.38, 0.45, 0.47)))
    parts.append(rounded_panel('Touchpad', (-0.207, -0.132), (-0.060, 0.060),
                               (0.00006, 0.00035), 0.003, (0.115, 0.216, 0.238)))
    for i, y in enumerate((-0.112, 0.112)):
        parts.append(cube(f'ChassisHingeMount{i}', (-0.006, y, -0.0025), (0.010, 0.031, 0.006),
                          color=(0.060, 0.099, 0.111)))
        parts.append(f'''def Cylinder "HingeBarrel{i}"
{{
    uniform token axis = "Y"
    double height = 0.032
    double radius = 0.004
    double3 xformOp:translate = (0, {y}, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
    color3f[] primvars:displayColor = [(0.040, 0.071, 0.083)]
}}
''')
    return ''.join(parts)


def display():
    return (
        shell(DISPLAY_Z, (0.073, 0.134, 0.155))
        + rounded_panel('Glass', (-0.210, -0.012), (-0.148, 0.148),
                        (0.00042, 0.00056), 0.004, (0.018, 0.032, 0.044))
        + rounded_panel('DisplayArea', (-0.205, -0.026), (-0.143, 0.143),
                        (0.00030, 0.00043), 0.001, (0.044, 0.132, 0.254))
        + rounded_panel('Wallpaper', (-0.158, -0.073), (-0.083, 0.083),
                        (0.00023, 0.00031), 0.031, (0.068, 0.251, 0.394))
        + rounded_panel('WallpaperInset', (-0.141, -0.090), (-0.061, 0.061),
                        (0.00017, 0.00024), 0.024, (0.106, 0.405, 0.509))
        + rounded_panel('WallpaperCenter', (-0.127, -0.104), (-0.031, 0.031),
                        (0.00011, 0.00018), 0.010, (0.197, 0.576, 0.619))
        + cube('Webcam', (-0.2115, 0., 0.00044), (0.0018, 0.007, 0.00008), color=(0.008, 0.012, 0.015))
        + cube('DisplayHingeMount0', (-0.006, -0.112, 0.003), (0.012, 0.019, 0.005), color=(0.062, 0.108, 0.125))
        + cube('DisplayHingeMount1', (-0.006, 0.112, 0.003), (0.012, 0.019, 0.005), color=(0.062, 0.108, 0.125))
    )


def rigid_link(name, index, zlim, contents):
    x, y, z = REAR_X - FRONT_X, WIDTH, zlim[1] - zlim[0]
    diagonal = (MASS[index] * (a * a + b * b) / 12 for a, b in ((y, z), (x, z), (x, y)))
    schemas = ['PhysicsRigidBodyAPI', 'PhysicsMassAPI', 'PhysxRigidBodyAPI']
    root_properties = ''
    if index == 0:
        schemas.extend(('PhysicsArticulationRootAPI', 'PhysxArticulationAPI'))
        root_properties = '''
    bool physxArticulation:enabledSelfCollisions = 0
    int physxArticulation:solverPositionIterationCount = 24
    int physxArticulation:solverVelocityIterationCount = 4'''
    quoted_schemas = ', '.join('"' + v + '"' for v in schemas)
    return f'''def Xform "{name}" (
    prepend apiSchemas = [{quoted_schemas}]
)
{{
    float physics:mass = {MASS[index]}
    point3f physics:centerOfMass = {vector(COM[index])}
    float3 physics:diagonalInertia = {vector(diagonal)}
    quatf physics:principalAxes = (1, 0, 0, 0)
    float physxRigidBody:maxDepenetrationVelocity = 1.0{root_properties}
{indent(contents, '    ')}
}}
'''


def friction(name, static, dynamic):
    return f'''def Material "{name}" (
    prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
)
{{
    float physics:staticFriction = {static}
    float physics:dynamicFriction = {dynamic}
    float physics:restitution = 0
}}
'''


def asset_text():
    lower, upper = (v - 180 for v in HINGE_LIMIT)
    content = ''.join((friction('ShellFriction', 0.55, 0.42), friction('FootFriction', 0.86, 0.66),
                       rigid_link('Chassis', 0, CHASSIS_Z, laptop_base()),
                       rigid_link('Display', 1, DISPLAY_Z, display())))
    return f'''#usda 1.0
(
    defaultPrim = "Laptop"
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "Laptop"
{{
{indent(content, '    ')}
    def PhysicsRevoluteJoint "Hinge"
    {{
        rel physics:body0 = </Laptop/Chassis>
        rel physics:body1 = </Laptop/Display>
        uniform token physics:axis = "Y"
        quatf physics:localRot0 = (0, 0, 1, 0)
        quatf physics:localRot1 = (1, 0, 0, 0)
        float physics:lowerLimit = {lower}
        float physics:upperLimit = {upper}
        bool physics:collisionEnabled = 0
    }}
}}
'''


if __name__ == '__main__':
    Path(__file__).with_name('assets').joinpath('laptop.usda').write_text(asset_text(), encoding='utf-8')
