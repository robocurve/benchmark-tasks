"""Design dimensions in metres, relative to the floor."""
from pathlib import Path
import math
from usd_geometry import body, color, cube, cylinder, joint, mesh, prim, transform

HERE = Path(__file__).parent
WIDTH = HEIGHT = 1.8288
MASS = .32
COUNT = 12
ROWS = 8
PITCH = .147
STRIP = WIDTH / COUNT
SEGMENT = (HEIGHT - .026) / ROWS
X = .378
Z = 1.929
ROD_X = X + .036
ROD_Z = Z + .049
SPAN = 1.93
CENTERS = tuple((i - 5.5) * PITCH for i in range(COUNT))
TARGETS = (5, 6)
BORE = .0083
TIP_HEIGHT = .0165
BALL_RADIUS = .004445
WIRE_RADIUS = .001778
LOWER_RETURN = ((0., TIP_HEIGHT), (.0015, .008), (.005, .0015),
                (.011, -.002), (.017, -.002), (.022, .002), (.026, .010))
BASES = {"left": (0., .305, 1.88), "right": (0., -.305, 1.88)}
CLOTH = (.20, .62, .71)
METAL = (.68, .71, .75)

def save(name, contents):
    path = HERE / "assets" / f"{name}.usda"
    path.parent.mkdir(exist_ok=True)
    doc = '#usda 1.0\n(defaultPrim = "Asset"\n metersPerUnit = 1\n upAxis = "Z")\n'
    path.write_text(doc + prim("Xform", "Asset", contents))

def precision(text):
    return text.replace(
        '"PhysicsCollisionAPI"', '"PhysxCollisionAPI", "PhysicsCollisionAPI"'
    ).replace(
        '\n{\n', '\n{\nfloat physxCollision:contactOffset = 0.00065\nfloat physxCollision:restOffset = 0\n'
    )

def ring_wedge(i):
    n = 12
    angles = [math.tau * j / n for j in (i, i + 1)]
    outline = [(r * math.cos(theta), r * math.sin(theta))
               for theta, r in ((angles[0], BORE), (angles[0], .014),
                                (angles[1], .014), (angles[1], BORE))]
    pts = [(x, y, z) for z in (-.002, .002) for x, y in outline]
    text = mesh(f"Eye{i}", pts, [(3,2,1,0),(4,5,6,7),(0,1,5,4),
                 (1,2,6,5),(2,3,7,6),(3,0,4,7)], METAL, True)
    return text

def strip():
    eye = ''.join(ring_wedge(i) for i in range(12))
    eye += cube("ReinforcedHeader", (.038, .024, .006), (-.027, 0, 0), (.14,.40,.51))
    # The small horizontal header carries the eye and folds where the film begins.
    text = body("eye", precision(eye), .0043, articulation=True,
                inertia=(1.1e-6, 1.8e-6, 2.2e-6))
    for row in range(ROWS):
        name = f"sheet{row}"
        z = -.013 - (row + .5) * SEGMENT
        # Resolved cloth panels carry the complete distributed weight. Seams are internal forces.
        panel = cube("Film", (.0016, PITCH - .003, SEGMENT - .002),
                     (0, 0, 0), CLOTH if row % 2 else (.22,.65,.74))
        text += body(name, precision(panel), (MASS / COUNT - .0043) / ROWS,
                     at=(-.011, 0, z), inertia=(1.9e-5, 1.25e-5, 5.1e-6))
        parent = "eye" if row == 0 else f"sheet{row-1}"
        offset = (-.011, 0, -.013) if row == 0 else (0, 0, -SEGMENT/2)
        text += joint("RevoluteJoint", f"bend{row}", f"/Asset/{parent}",
                      f"/Asset/{name}", offset, (0,0,SEGMENT/2), articulation=True,
                      extra='uniform token physics:axis = "Y"\nfloat physics:lowerLimit = -70\nfloat physics:upperLimit = 70\n')
    save("strip", text)

def sphere(name, x, z, radius):
    return prim("Sphere", name, f"double radius = {radius}\n"
                + transform((x, 0., z)) + color(METAL), ("PhysicsCollisionAPI",))


def wire_segment(index, start, stop):
    point = tuple((a + b) / 2 for a, b in zip(start, stop))
    dx, dz = (b - a for a, b in zip(start, stop))
    angle = math.degrees(math.atan2(dx, dz))
    data = (f'double radius = {WIRE_RADIUS}\ndouble height = {math.hypot(dx, dz)}\n'
            + 'uniform token axis = "Z"\n'
            + transform((point[0], 0., point[1]), rotate=(0., angle, 0.)) + color(METAL))
    return prim("Capsule", f"Wire{index}", data, ("PhysicsCollisionAPI",))


def hook():
    # Centerline simplified from Moen SR2201 drawing (3.10 x 2.84 in envelope).
    # The deeper return opens toward the working side of this single-liner bay.
    profile = [
        *LOWER_RETURN, (.027, .018),
        (.026, .036), (.024, .048), (.023, .055), (.029, .061),
        (.037, .064), (.046, .063), (.054, .059), (.056, .051),
        (.054, .043), (.052, .034), (.054, .026), (.060, .020),
        (.067, .019), (.071, .024), (.06985, .034),
    ]
    geo = "".join(wire_segment(i, a, b) for i, (a, b) in enumerate(zip(profile, profile[1:])))
    geo += sphere("LinerCap", 0., TIP_HEIGHT, BALL_RADIUS)
    geo += sphere("CurtainCap", .06985, .034, BALL_RADIUS)
    rollers = ((.022, .054), (.029, .061), (.037, .064), (.046, .063), (.054, .058))
    geo += "".join(sphere(f"Roller{i}", x, z, BALL_RADIUS) for i, (x, z) in enumerate(rollers))
    save("hook", body("hook", precision(geo), .026, fixed=True))


def room():
    rail = cylinder("Tube", .0127, SPAN, (ROD_X,0,ROD_Z), METAL, "Y")
    wall=(.78,.78,.73)
    rail += cube("BackWall", (.055,SPAN+.12,2.17), (.72,0,1.085), wall)
    for i, sign in enumerate((-1,1)):
        rail += cube(f"ReturnWall{i}", (.65,.055,2.17),(.43,sign*(SPAN/2+.024),1.085),wall)
        rail += cylinder(f"Flange{i}", .032,.016,(ROD_X, sign*(SPAN/2-.010),ROD_Z), METAL, "Y")
    save("shower", body("shower", precision(rail), 12., fixed=True))

if __name__ == "__main__":
    strip()
    hook()
    room()
