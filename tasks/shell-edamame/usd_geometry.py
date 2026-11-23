"""Small OpenUSD writer for the task assets."""
BLUE = (.04, .38, .62)
NAVY = (.055, .082, .112)
METAL = (.56, .62, .66)
WHITE = (.91, .91, .87)
ORANGE = (.92, .43, .12)
WOOD = (.60, .37, .20)
SILICONE = (.26, .69, .59)
PAINT = (.22, .49, .63)


def vec(values):
    return '(' + ', '.join(f'{v:.8g}' for v in values) + ')'


def prim(kind, name, contents='', apis=(), metadata=()):
    fields = list(metadata)
    if apis:
        fields.append('prepend apiSchemas = [' + ', '.join(f'"{a}"' for a in apis) + ']')
    meta = ' (\n    ' + '\n    '.join(fields) + '\n)' if fields else ''
    body = '\n'.join('    ' + line if line else '' for line in contents.splitlines())
    return f'def {kind} "{name}"{meta}\n{{\n{body}\n}}\n'


def transform(position=None, rotate=None, scale=None):
    values = []
    if position is not None:
        values.append(('double3', 'translate', vec(position)))
    if rotate is not None:
        values.append(('float3', 'rotateXYZ', vec(rotate)))
    if scale is not None:
        values.append(('float3', 'scale', vec(scale)))
    if not values:
        return ''
    order = ', '.join(f'"xformOp:{op}"' for _, op, _ in values)
    return ''.join(f'{t} xformOp:{op} = {v}\n' for t, op, v in values) + f'uniform token[] xformOpOrder = [{order}]\n'


def color(rgb):
    return f'color3f[] primvars:displayColor = [{vec(rgb)}]\n'


def cube(name, size, at, rgb, collide=True, rotate=None):
    return prim('Cube', name, 'double size = 1\n' + transform(at, rotate, size) + color(rgb),
                ('PhysicsCollisionAPI',) if collide else ())


def cylinder(name, radius, height, at, rgb, axis='Z', collide=True, hidden=False):
    data = (f'double radius = {radius}\ndouble height = {height}\nuniform token axis = "{axis}"\n'
            + transform(at) + color(rgb))
    if hidden:
        data += 'token visibility = "invisible"\n'
    return prim('Cylinder', name, data, ('PhysicsCollisionAPI',) if collide else ())


def mesh(name, pts, faces, rgb, collide=False):
    data = 'point3f[] points = [' + ', '.join(vec(p) for p in pts) + ']\n'
    data += 'int[] faceVertexCounts = [' + ', '.join(str(len(f)) for f in faces) + ']\n'
    data += 'int[] faceVertexIndices = [' + ', '.join(str(i) for f in faces for i in f) + ']\n'
    data += 'uniform token subdivisionScheme = "none"\n' + color(rgb)
    if collide:
        data += 'uniform token physics:approximation = "convexHull"\n'
    return prim('Mesh', name, data, ('PhysicsCollisionAPI', 'PhysicsMeshCollisionAPI') if collide else ())


def body(name, parts, mass, at=(0, 0, 0), rotate=None, fixed=False, articulation=False, inertia=None):
    data = transform(at, rotate) + f'float physics:mass = {mass}\n'
    if inertia is not None:
        data += f'float3 physics:diagonalInertia = {vec(inertia)}\n'
    if fixed:
        data += 'bool physics:kinematicEnabled = true\n'
    schema = ('PhysicsRigidBodyAPI', 'PhysicsMassAPI')
    if articulation:
        schema += ('PhysicsArticulationRootAPI',)
    return prim('Xform', name, data + parts, schema)


def point(name, at):
    return prim('Xform', name, transform(at))


def joint(kind, name, a, b, loc0=(0,0,0), loc1=(0,0,0), *, extra='', apis=(),
          rot0=None, rot1=None, collide=False, articulation=False):
    data = (f'rel physics:body0 = <{a}>\nrel physics:body1 = <{b}>\n'
            f'point3f physics:localPos0 = {vec(loc0)}\npoint3f physics:localPos1 = {vec(loc1)}\n'
            f'bool physics:collisionEnabled = {str(collide).lower()}\n'
            f'uniform bool physics:excludeFromArticulation = {str(not articulation).lower()}\n')
    if rot0:
        data += f'quatf physics:localRot0 = {vec(rot0)}\n'
    if rot1:
        data += f'quatf physics:localRot1 = {vec(rot1)}\n'
    return prim(f'Physics{kind}', name, data + extra, apis)
