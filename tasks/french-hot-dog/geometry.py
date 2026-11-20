"""Full-size French hot dog roll, cold piercing mandrel and serving stand.

All dimensions are metres. Food poses are centered along their length and
their local +Z points toward the roll opening and the sausage grasp end.
The stand and mandrel use station coordinates. Shapes and dimensions are
original approximations, not commercial-product measurements.
"""

from math import cos, pi, sin, sqrt


TABLE_Z = .7475
BUN_LENGTH = .155
BUN_RADIUS = .027
# Clearance represents the local give of bread around the steel shaft.
CAVITY_RADIUS = .01525
CAVITY_TRAVEL = .108
SAUSAGE_LENGTH = .145
SAUSAGE_RADIUS = .011
BUN_START = (.30, .20, TABLE_Z + BUN_LENGTH / 2)
SAUSAGE_START = (.32, -.20, TABLE_Z + SAUSAGE_RADIUS)
SAUSAGE_ROTATION_XYZW = (-sqrt(.5), 0., 0., sqrt(.5))
SPIKE_TIP = (.40, .11, .924)
SPIKE_RADIUS = .0125
BUN_FINAL = (.39, -.065, .850)
CRADLE_AXIS = (0., -sqrt(.5), sqrt(.5))
BUN_FINAL_ROTATION_XYZW = (sin(pi / 8), 0., 0., cos(pi / 8))

CRUST = (.63, .30, .10, 1.)
CRUMB = (.87, .69, .43, 1.)
STEEL = (.44, .49, .50, 1.)
DARK_STEEL = (.20, .23, .24, 1.)
CRADLE = (.26, .34, .39, 1.)
ROLL_PROFILE = ((-.0775, .0205), (-.074, .023), (-.066, .025),
                (-.048, .0265), (-.024, .027), (.025, .027),
                (.051, .0265), (.066, .025), (.074, .0225),
                (.0775, .0205))
BLIND_BOTTOM_Z = -.050
CRUMB_THICKNESS = .015


def _polar(z, radius, angle):
    return (radius * cos(angle), radius * sin(angle), z)


def _surfaces(profile, sides, *, inward=False, texture=None, caps=False):
    """Create the visible rings and cylindrical UVs of a surface of revolution."""
    points = [_polar(z, radius, 2 * pi * i / sides)
              for z, radius in profile for i in range(sides)]
    faces, uv = [], []
    for layer in range(len(profile) - 1):
        for i in range(sides):
            j = (i + 1) % sides
            face = [layer * sides + i, layer * sides + j,
                    (layer + 1) * sides + j, (layer + 1) * sides + i]
            v0, v1 = ((profile[k][0] - profile[0][0]) / (profile[-1][0] - profile[0][0])
                      for k in (layer, layer + 1))
            coords = [(i / sides, v0), ((i + 1) / sides, v0),
                      ((i + 1) / sides, v1), (i / sides, v1)]
            faces.append(face[::-1] if inward else face)
            uv.extend(coords[::-1] if inward else coords)
    if caps:
        faces.append(list(reversed(range(sides))))
        faces.append(list(range((len(profile) - 1) * sides, len(profile) * sides)))
    return dict(points=points, faces=faces, **(dict(uv=uv, texture=texture) if texture else {}))


def _mesh(mesh, color, **kwargs):
    return dict(type="mesh", mesh=mesh, color=color, **kwargs)


def _solid(kind, size, position, color, **kwargs):
    return dict(type=kind, size=size, position=position, color=color, **kwargs)


def _body(name, shapes, mass, size, position=(0., 0., 0.)):
    x, y, z = size
    diagonal = tuple(mass * pair / 12 for pair in (y*y + z*z, x*x + z*z, x*x + y*y))
    return dict(name=name, position=position, mass=mass, inertia=dict(diagonal=diagonal),
                shapes=shapes)


def _wedge(start, stop):
    """One solid collider sector with no hull crossing the bore."""
    profile = [(BLIND_BOTTOM_Z, .0263), *[(z, r) for z, r in ROLL_PROFILE if z > BLIND_BOTTOM_Z]]
    points = [_polar(z, radius, angle) for z, outer in profile
              for radius, angle in ((outer, start), (outer, stop),
                                    (CAVITY_RADIUS, stop), (CAVITY_RADIUS, start))]
    faces = [[3, 2, 1, 0], [(len(profile) - 1) * 4 + i for i in range(4)]]
    faces.extend([4 * layer + i, 4 * layer + (i + 1) % 4,
                  4 * (layer + 1) + (i + 1) % 4, 4 * (layer + 1) + i]
                 for layer in range(len(profile) - 1) for i in range(4))
    return _mesh(dict(points=points, faces=faces), CRUST, visible=False,
                 friction=(1.15, .92), contact_offset=.00015)


def _shell():
    sides = 64
    mouth = BUN_LENGTH / 2
    points = [_polar(mouth, radius, 2 * pi * i / sides)
              for radius in (ROLL_PROFILE[-1][1], CAVITY_RADIUS) for i in range(sides)]
    faces = [[i, (i + 1) % sides, (i + 1) % sides + sides, i + sides] for i in range(sides)]
    lower = ROLL_PROFILE[:3] + ((BLIND_BOTTOM_Z, .0263),)
    shapes = [
        _mesh(_surfaces(ROLL_PROFILE, sides, texture="assets/bun-crust.png"), CRUST,
              roughness=.92, collision=False),
        _mesh(dict(points=points, faces=faces), CRUST, roughness=.92, collision=False),
        _mesh(_surfaces(((BLIND_BOTTOM_Z, CAVITY_RADIUS), (mouth, CAVITY_RADIUS)), sides,
                        inward=True), CRUMB, roughness=.96, collision=False),
        _mesh(_surfaces(lower, 32, caps=True), CRUST, visible=False,
              friction=(1.15, .92), contact_offset=.00015),
        *[_wedge(2 * pi * i / 16, 2 * pi * (i + 1) / 16) for i in range(16)],
    ]
    return _body("shell", shapes, .050, (.054, .054, BUN_LENGTH))


def _crumb():
    # The flush face blocks the mandrel until the prismatic friction is overcome.
    # The tiny annular clearance avoids self-contact in the sliding guide.
    r = CAVITY_RADIUS - .0006
    face = _mesh(_surfaces(((-CRUMB_THICKNESS / 2, r), (CRUMB_THICKNESS / 2, r)),
                           32, caps=True), (.70, .43, .20, 1.),
                 roughness=.97, friction=(.55, .42), contact_offset=.0001)
    return _body("crumb", [face], .004, (2 * r, 2 * r, CRUMB_THICKNESS),
                 position=(0., 0., BUN_LENGTH / 2 - CRUMB_THICKNESS / 2))


def _sausage():
    r = SAUSAGE_RADIUS
    half_cylinder = SAUSAGE_LENGTH / 2 - r
    # A narrow ring of darker casing at both rounded ends avoids a plain rod.
    profile = [(-half_cylinder - r * cos(theta), max(.0001, r * sin(theta)))
               for theta in (i * pi / 16 for i in range(9))]
    profile += [(z, r) for z in (-half_cylinder * .72, 0., half_cylinder * .72)]
    profile += [(-z, radius) for z, radius in reversed(profile[:9])]
    shapes = [
        _mesh(_surfaces(profile, 48, texture="assets/sausage-casing.png"), (.54, .15, .078, 1.),
              roughness=.58, collision=False),
        _solid("capsule", (r, half_cylinder), (0., 0., 0.), (.54, .15, .078, 1.),
               visible=False, friction=(1.05, .83), contact_offset=.00015),
    ]
    return _body("meat", shapes, .054, (2 * r, 2 * r, SAUSAGE_LENGTH))


def _quat_multiply(q, p):
    w, x, y, z = q
    a, b, c, d = p
    return (w*a - x*b - y*c - z*d, w*b + x*a + y*d - z*c,
            w*c - x*d + y*a + z*b, w*d + x*c - y*b + z*a)


def _cradle_position(x, y, z):
    return (BUN_FINAL[0] + x, BUN_FINAL[1] + (y - z) * sqrt(.5),
            BUN_FINAL[2] + (y + z) * sqrt(.5))


def _fixture():
    shapes = [
        _solid("box", (.091, .073, .004), (.501, .121, TABLE_Z + .004), DARK_STEEL),
        _solid("box", (.014, .017, .154), (.566, .110, .9095), STEEL),
        _solid("box", (.086, .017, .008), (.483, .110, 1.072), STEEL, roughness=.33),
        _solid("box", (.048, .070, .003), (.390, -.043, TABLE_Z + .003), CRADLE),
    ]
    tilt = (cos(pi / 8), sin(pi / 8), 0., 0.)
    for side in (-1, 1):
        rail_angle = (cos(pi / 8), 0., 0., side * sin(pi / 8))
        rail = _cradle_position(side * .020, -.0208, -.0255)
        shapes.append(_solid("box", (.024, .002, .0475), rail, CRADLE,
                            orientation=_quat_multiply(tilt, rail_angle), roughness=.42))
        feet_z = TABLE_Z + .006
        for offset in (-.037, .028):
            x, y, z = _cradle_position(side * .029, -.014, offset - .025)
            height = z - feet_z - .003
            shapes.append(_solid("box", (.005, .005, height / 2), (x, y, feet_z + height / 2), CRADLE))
    shapes.append(_solid("box", (.025, .019, .003), _cradle_position(0., -.022, -.081),
                         CRADLE, orientation=tilt, friction=(.63, .52)))
    return dict(name="stand", static=True, shapes=shapes)


def _mandrel():
    # A blunt rounded tip, tapered lead, full-diameter shaft and mounting collar.
    point_profile = ((0., .0011), (.0012, .0022), (.0205, SPIKE_RADIUS),
                     (.119, SPIKE_RADIUS))
    head = _mesh(_surfaces(point_profile, 40, caps=True), STEEL,
                 roughness=.23, friction=(.27, .21), contact_offset=.00012)
    collar = _solid("cylinder", (.017, .010), (0., 0., .1295), DARK_STEEL, roughness=.30)
    return _body("pike", [head, collar], .15, (.034, .034, .14), position=SPIKE_TIP)


def geometry():
    """Return the native bodies and the roll's passive crumb compaction joint."""
    bun = dict(
        name="FrenchRoll", bodies=[_shell(), _crumb()], articulation=True, self_collisions=False,
        solver_position_iterations=32, solver_velocity_iterations=4,
        joints=[dict(name="cavity", type="prismatic", parent="shell", child="crumb", axis="Z",
                     position_parent=(0., 0., BUN_LENGTH / 2 - CRUMB_THICKNESS / 2),
                     lower=-CAVITY_TRAVEL, upper=0., static_friction_effort=4., dynamic_friction_effort=3.)],
    )
    return {
        "bun": bun,
        "sausage": dict(name="Sausage", bodies=[_sausage()], solver_position_iterations=32,
                        solver_velocity_iterations=4),
        "fixture": dict(name="ServingStand", bodies=[_fixture()]),
        "mandrel": dict(name="BreadPiercer", bodies=[_mandrel()], fixed_base="pike"),
    }
