"""Generate realistic-size, curved two-half edamame pod and three rigid beans.

The POD is an articulation: a root half, a hinged upper half, and an
optional breakable seam latch authored *before* PhysX creates the scene.
The bowl and bin have collision geometry. No bean teleportation/auto-ejection.

The seam latch is EXPERIMENTAL. Set EDAMAME_LATCH=0 only for diagnosis;
a latch-disabled version is NOT construct-valid for benchmark submission.
"""
from __future__ import annotations
import math
import os
from pathlib import Path
from usd_geometry import body,cube,cylinder,mesh,prim,joint,transform,color

HERE=Path(__file__).resolve().parent
TABLE=.7475
COMPACT_LAYOUT = os.getenv("EDAMAME_COMPACT_LAYOUT") == "1"
POD_POS=(.313 if COMPACT_LAYOUT else .357,-.052,TABLE+.017)
BOWL_POS = (
    (.445, .124, TABLE + .004) if os.getenv("EDAMAME_PLATE_RECOVERY_LAYOUT", "1") == "1"
    else (.488 if COMPACT_LAYOUT else .530, -.061, TABLE + .004)
)
BOWL_RADIUS = .078
BIN_POS=(.515,-.222,TABLE+.004)
PLATE_POS=(POD_POS[0],POD_POS[1],TABLE+.004)
POD_LENGTH=.066
Y_SCALE=.78
Z_SCALE=.65
HALF=.033
BEANS=(-.019,0.,.019)
BEAN_RAD=.0045
BEAN_MASS=.00105
LATCH=os.getenv('EDAMAME_LATCH','1')!='0'
FORCE_PROBE=os.getenv('EDAMAME_FORCE_PROBE','0')=='1'
LATCH_FORCE=float(os.getenv('EDAMAME_LATCH_FORCE_N','2.0'))
LATCH_TORQUE=float(os.getenv('EDAMAME_LATCH_TORQUE_NM','.022'))
SHELL_COLOR=(.31,.64,.18)
SHELL_INNER=(.47,.70,.31)
SEED_COLOR=(.64,.86,.28)
BOWL_COLOR=(.92,.94,.88)
PLATE_COLOR=(.92,.88,.79)
BIN_COLOR=(.23,.37,.29)

# local pod is 66 mm x about 19 mm x 19 mm with gentle lateral curvature.
def bow(x):
    u=min(1,max(-1,x/HALF))
    return .0016*(1-u*u)
def radius(x):
    u=min(1,abs(x)/HALF)
    return .0034+.0061*(max(0,1-u**3))**.55

def save(name,inside):
    path=HERE/'assets'/f'{name}.usda'
    path.parent.mkdir(exist_ok=True,parents=True)
    header='#usda 1.0\n(\n    defaultPrim = "Asset"\n    metersPerUnit = 1\n    kilogramsPerUnit = 1\n    upAxis = "Z"\n)\n'
    path.write_text(header+prim('Xform','Asset',inside),encoding='utf-8')
    return path

def _visual_half(upper, low=-HALF, high=HALF):
    """Thin shell mesh with a bumpy curved profile; visual only.

    Physics uses many small *solid wall bars* instead of a single convex hull,
    because a convex hull would seal the concave cavity and trap beans outside.
    """
    nx=31; na=17
    aa=(0,math.pi) if upper else (math.pi,2*math.pi)
    verts=[]
    for offset in (0.,.0011):
        for i in range(nx):
            x=low+i*(high-low)/(nx-1)
            for j in range(na):
                a=aa[0]+(aa[1]-aa[0])*j/(na-1)
                r=radius(x)+offset
                verts.append((x,Y_SCALE*(bow(x)+r*math.cos(a)),Z_SCALE*r*math.sin(a)))
    layer=nx*na
    faces=[]
    for face_layer in (0,1):
        off=layer*face_layer
        for i in range(nx-1):
            for j in range(na-1):
                k=off+i*na+j
                f=(k,k+na,k+na+1,k+1)
                faces.append(f if face_layer else tuple(reversed(f)))
    for i in range(nx-1):
        for j in (0,na-1):
            k=i*na+j
            faces.append((k,k+na,k+na+layer,k+layer))
    for i in (0,nx-1):
        for j in range(na-1):
            k=i*na+j
            faces.append((k,k+1,k+1+layer,k+layer))
    return mesh('OuterSkin' if upper else 'InnerSkin',verts,faces,SHELL_COLOR,collide=False)

def _collider(name, size, center, angle):
    # Preserve the anisotropic pod cross-section in the collision geometry.
    angle = math.radians(angle)
    cos, sin = math.cos(angle), math.sin(angle)
    half_x, half_y, half_z = (value/2 for value in size)
    points = []
    for x in (-half_x,half_x):
        for y,z in ((-half_y,-half_z),(half_y,-half_z),(half_y,half_z),(-half_y,half_z)):
            points.append((center[0]+x, Y_SCALE*(center[1]+cos*y-sin*z),
                           Z_SCALE*(center[2]+sin*y+cos*z)))
    faces = [(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
    return _precise_contact(mesh(name,points,faces,SHELL_COLOR,collide=True)).replace(
        'uniform token subdivisionScheme', 'token visibility = "invisible"\n    uniform token subdivisionScheme', 1)


def _precise_contact(shape):
    return shape.replace(
        '"PhysicsCollisionAPI"', '"PhysicsCollisionAPI", "PhysxCollisionAPI"').replace(
            '\n{\n', '\n{\n    float physxCollision:contactOffset = 0.00018\n'
            '    float physxCollision:restOffset = 0\n',1)


def _contact_body(*args, **kwargs):
    # Limit corrective contact separation only. External forces and actual
    # velocities remain governed by dynamics.
    return body(*args, **kwargs).replace(
        '"PhysicsRigidBodyAPI"', '"PhysicsRigidBodyAPI", "PhysxRigidBodyAPI"').replace(
            '\n{\n', '\n{\n    float physxRigidBody:maxDepenetrationVelocity = 0.25\n', 1)


def _colliders(upper, low=-HALF, high=HALF):
    out=''
    # Five longitudinal slices, five small bars per half; 50 wall bars total.
    # These do not fill the pod cavity (unlike a convex-hull mesh).
    count=0
    slices = max(1, round((high-low)/.0125))
    length = (high-low)/slices
    for i in range(slices):
        x=low + (i+.5)*length
        rr=radius(x)+.00025
        for j in range(5):
            a=(.24+(math.pi-.48)*j/4) if upper else (math.pi+.24+(math.pi-.48)*j/4)
            y=bow(x)+rr*math.cos(a)
            z=rr*math.sin(a)
            out+=_collider(f'Wall{count:02d}',(length+.001,.0030,.0017),(x,y,z),math.degrees(a)-90.)
            count+=1
    # End closures to stop beans rolling out unless half lifted/tilted.
    for name,x in [('TipL',-.031),('TipR',.031)]:
        if not low < x < high:
            continue
        out+=_collider(name,(.0018,.012,.007),(x,bow(x),.0026 if upper else -.0026),0.)
    return out


def _joined_wall(upper, low=-HALF, high=HALF):
    """Cover the curved wall with adjacent thin convex sectors."""
    slices = max(1, math.ceil((high - low) / .011))
    angles = 8
    start = 0. if upper else math.pi
    faces = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    pieces = []
    for i in range(slices):
        for j in range(angles):
            a0, a1 = (start + math.pi * k / angles for k in (j, j + 1))
            points = []
            for section in (i, i + 1):
                x = low + (high - low) * section / slices
                for a, skin in ((a0, 0.), (a1, 0.), (a1, .0011), (a0, .0011)):
                    r = radius(x) + skin
                    points.append((x, Y_SCALE * (bow(x) + r * math.cos(a)), Z_SCALE * r * math.sin(a)))
            shape = mesh(f"Sector{i:02d}_{j:02d}", points, faces, SHELL_COLOR, collide=True)
            pieces.append(_precise_contact(shape).replace(
                'uniform token subdivisionScheme', 'token visibility = "invisible"\n    uniform token subdivisionScheme', 1))
    for name, x in (("TipL", -.031), ("TipR", .031)):
        if low < x < high:
            pieces.append(_collider(name, (.0018, .012, .007),
                                    (x, bow(x), .0026 if upper else -.0026), 0.))
    return "".join(pieces)


def pod(probe=False, joined=False):
    wall = _joined_wall if joined else _colliders
    lower=_contact_body('pod_lower',_visual_half(False)+wall(False),.0036,
               articulation=True,inertia=(1.4e-7,2.3e-6,2.3e-6))
    upper_sections = (
        ("pod_upper_left", -HALF, -.011),
        ("pod_upper", -.011, .011),
        ("pod_upper_right", .011, HALF),
    )
    upper = "".join(_contact_body(name, _visual_half(True, low, high)+wall(True, low, high),
                         .00097, inertia=(4e-8,5e-7,5e-7))
                    for name, low, high in upper_sections)
    hinge=joint('RevoluteJoint','SeamHinge','/Asset/pod_lower','/Asset/pod_upper',
                loc0=(0,-.0091*Y_SCALE,0),loc1=(0,-.0091*Y_SCALE,0),articulation=True,
                extra=('uniform token physics:axis = "X"\n'
                       'float physics:lowerLimit = 0\n'
                       'float physics:upperLimit = 135\n'))
    for tip,x in (("left",-.011),("right",.011)):
        hinge += joint("RevoluteJoint", f"TipFlex_{tip}", "/Asset/pod_upper", f"/Asset/pod_upper_{tip}",
                       loc0=(x,0.,0.), loc1=(x,0.,0.), articulation=True,
                       extra='uniform token physics:axis = "Y"\nfloat physics:lowerLimit = -20\n'
                             'float physics:upperLimit = 20\n')
    anchor=''
    if probe:
        # Diagnostic only: fixture holds the lower shell stationary while
        # a probe force pulls the upper shell against the breakable seam.
        anchor=prim('PhysicsFixedJoint','probe_world_anchor',
                    'rel physics:body1 = </Asset/pod_lower>\n')
    name = 'edamame_pod_probe' if probe else 'edamame_pod_joined' if joined else 'edamame_pod'
    return save(name, lower+upper+hinge+anchor)

def bean():
    # Explicit hull coordinates keep the visual and cooked collision the same
    # dimensions. PhysX does not preserve non-uniform scale on a sphere.
    around = 16
    rings = 7
    points = [(0., 0., -.0040)]
    for ring in range(1, rings + 1):
        lat = math.pi * ring / (rings + 1) - math.pi / 2
        for k in range(around):
            az = 2 * math.pi * k / around
            points.append((.0050 * math.cos(lat) * math.cos(az),
                           .0042 * math.cos(lat) * math.sin(az), .0040 * math.sin(lat)))
    points.append((0., 0., .0040))
    faces = []
    for k in range(around):
        nxt = (k + 1) % around
        faces.append((0, 1 + nxt, 1 + k))
        for i in range(rings - 1):
            row = 1 + i * around
            faces.append((row + k, row + nxt, row + around + nxt, row + around + k))
        top = 1 + (rings - 1) * around
        faces.append((len(points) - 1, top + k, top + nxt))
    seed = _precise_contact(mesh('EdibleBean', points, faces, SEED_COLOR, collide=True))
    return save('edamame_bean',_contact_body('bean',seed,BEAN_MASS,
        inertia=(8.8e-9,9.2e-9,10.0e-9)))

def plate():
    # Simpler than banana: plate is a flat resting surface with an edge.
    p=cylinder('Surface',.092,.006,(0,0,0),PLATE_COLOR)
    for i in range(20):
        a=2*math.pi*i/20
        p+=cube(f'Edge{i:02}',(.030,.004,.006),(.086*math.cos(a),.086*math.sin(a),.005),PLATE_COLOR,rotate=(0,0,math.degrees(a)+90))
    return save('edamame_plate',body('plate',p,.22,fixed=True))

def bowl():
    radius=BOWL_RADIUS; wall=.004; height=.032
    p=cylinder('BowlBase',radius,.006,(0,0,0),BOWL_COLOR)
    for i in range(24):
        a=2*math.pi*i/24
        p+=cube(f'Side{i:02}',(.022,wall,height),
                ((radius-wall/2)*math.cos(a),(radius-wall/2)*math.sin(a),height/2),
                BOWL_COLOR,rotate=(0,0,math.degrees(a)+90))
    return save('bean_bowl',body('bowl',p,.35,fixed=True))

def trash():
    width=.165;h=.125;t=.005
    p=cube('Base',(width,width,.008),(0,0,0),BIN_COLOR)
    for sign,name in ((-1,'Negative'),(1,'Positive')):
        p+=cube(f'{name}X',(t,width,h),(sign*(width-t)/2,0,h/2),BIN_COLOR)
        p+=cube(f'{name}Y',(width,t,h),(0,sign*(width-t)/2,h/2),BIN_COLOR)
    return save('shell_scraps_bin',body('bin',p,.5,fixed=True))

def generate():
    return [pod(),pod(probe=True),bean(),plate(),bowl(),trash()]

if __name__=='__main__':
    for path in generate() + [pod(joined=True)]:print(path)
