"""Regenerate ``rdb_ukulele_tuning/ukulele.usda`` from ``layout.py`` and ``fixed_scene.py``.

Run from the task folder: ``python tools/build_assets.py``. Needs only the Python standard library.

``/Fixture/Ukulele`` is one fixed-base articulation (the articulation root on the parent Xform, the root link welded
to the world by ``PhysicsFixedJoint "root_joint"``):

- ``instrument``, the root link: body, neck, fretboard, headstock, the open gears (one chrome box each, standing in
  for the plate, worm and gear), posts, nut, bridge, saddle, the strings and the clip-on tuner, one rigid body that
  cannot move (as if strapped to its cradle);
- ``button_g`` .. ``button_a``: the four pearloid tuning buttons, each turning on a revolute joint about its shaft
  (no end stops), with a drive capped at 0.03 N m standing in for the gear's drag;
- ``pluck``: the G string's 36 mm stretch over the soundhole, sliding across the strings on a prismatic joint with a
  spring;
- ``needle``: the tuner's needle, turning on a revolute joint whose position target the adapter sets every step.

Parts of the articulation do not collide with each other. Rigid colliders only: boxes, capsules, cylinders and
convex hulls; the body's figure-eight is drawn as one mesh over hidden convex slices. ``/Fixture/Static`` holds the
foam pad and the neck rest (fixed colliders), ``/Fixture/Drawn`` the strap (drawn only) and ``/Fixture/Lamps`` the
tuner's note and in-tune lamps, drawn hidden; the adapter shows the lit ones.
"""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_ukulele_tuning import fixed_scene as scene  # noqa: E402
from rdb_ukulele_tuning import layout as L  # noqa: E402

# colour, roughness, metallic, friction, emissive colour (lamps and needle glow)
MATERIALS = {
    "mahogany": ((.36, .17, .08), .7, 0., .5, None),          # satin mahogany
    "rosewood": ((.10, .06, .04), .6, 0., .5, None),          # fretboard and bridge
    "bone": ((.92, .90, .82), .4, 0., .4, None),              # nut and saddle
    "nickel": ((.75, .75, .73), .3, 1., .4, None),            # frets
    "chrome": ((.80, .80, .82), .2, 1., .8, None),            # posts, the open gears' stand-in boxes, shafts
    "pearloid": ((.90, .88, .83), .3, 0., .8, None),          # ESTIMATE (friction): the buttons under the gripper's pads
    "nylgut": ((.93, .91, .84), .5, 0., .3, None),            # white strings
    "dot": ((.95, .95, .92), .5, 0., .5, None),
    "soundhole": ((.015, .012, .01), .9, 0., .5, None),
    "rosette": ((.62, .45, .25), .6, 0., .5, None),
    "foam": ((.05, .05, .055), .95, 0., .9, None),
    "rest_pad": ((.12, .12, .13), .9, 0., .9, None),
    "strap": ((.04, .04, .045), .9, 0., .8, None),
    "tuner_black": ((.03, .03, .035), .5, 0., .6, None),
    "screen": ((.012, .014, .018), .2, 0., .5, None),
    "print": ((.42, .42, .45), .6, 0., .5, None),
    "needle": ((.95, .35, .05), .4, 0., .5, (.95, .35, .05)),
    "lamp_note": ((1., .78, .30), .4, 0., .5, (1., .78, .30)),
    "lamp_green": ((.15, 1., .30), .4, 0., .5, (.15, 1., .30)),
}
PREFIX = "</Fixture/Materials/"
ART = "/Fixture/Ukulele/"
SAMPLES = 6                              # drawn body outline: points per outline segment
NEEDLE_LIFT = .0006                      # the needle above the screen, the lamps .2 mm, the print .4 mm
IDENTITY = (1., 0., 0., 0.)


# -- small vector and rotation helpers ---------------------------------------------------------------------------
def _f(v):
    return f"{v:.6g}"


def _v3(p):
    return f"({_f(p[0])}, {_f(p[1])}, {_f(p[2])})"


def _q(q):
    return "(" + ", ".join(f"{v:.7g}" for v in q) + ")"


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, s):
    return tuple(x * s for x in a)


def norm(a):
    return math.sqrt(sum(x * x for x in a))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def quat_from_columns(c0, c1, c2):
    """(w, x, y, z) of the rotation whose columns are c0, c1, c2."""
    m = ((c0[0], c1[0], c2[0]), (c0[1], c1[1], c2[1]), (c0[2], c1[2], c2[2]))
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0:
        s = 2. * math.sqrt(trace + 1.)
        q = (.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = 2. * math.sqrt(1. + m[0][0] - m[1][1] - m[2][2])
        q = ((m[2][1] - m[1][2]) / s, .25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    elif m[1][1] > m[2][2]:
        s = 2. * math.sqrt(1. + m[1][1] - m[0][0] - m[2][2])
        q = ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, .25 * s, (m[1][2] + m[2][1]) / s)
    else:
        s = 2. * math.sqrt(1. + m[2][2] - m[0][0] - m[1][1])
        q = ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, .25 * s)
    if q[0] < 0:
        q = tuple(-v for v in q)
    return q


def quat_y_to(d):
    """(w, x, y, z) turning +y onto the direction ``d``."""
    d = scale(d, 1. / norm(d))
    axis = cross((0., 1., 0.), d)
    s, c = norm(axis), d[1]
    if s < 1e-12:
        return IDENTITY if c > 0 else (0., 1., 0., 0.)
    angle = math.atan2(s, c)
    axis = scale(axis, math.sin(angle / 2) / s)
    return (math.cos(angle / 2), *axis)


SCREEN = L.screen_frame()
SCREEN_Q = quat_from_columns(*SCREEN)                    # the screen in the instrument frame
ROT_Q = scene.ROTATION_WXYZ                              # the instrument frame on the table (its x up, its z to -x)


def quat_mul(a, b):
    """Hamilton product a * b of (w, x, y, z) quaternions (b first, then a)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


WORLD_SCREEN_Q = quat_mul(ROT_Q, SCREEN_Q)              # the screen on the table


# -- prims --------------------------------------------------------------------------------------------------------
def shape(kind, name, material, centre, *, collide, visible=True, orient=None, scale_=None, attrs=(),
          pad="            ", offset=.001):
    """A geometric prim; with ``collide`` also a rigid collider. Drawn unless ``visible`` is False."""
    apis = ['"MaterialBindingAPI"'] + (['"PhysicsCollisionAPI"', '"PhysxCollisionAPI"'] if collide else [])
    text = f'{pad}def {kind} "{name}" (\n{pad}    prepend apiSchemas = [{", ".join(apis)}]\n{pad})\n{pad}{{\n'
    text += f"{pad}    rel material:binding = {PREFIX}{material}>\n"
    if collide:
        text += (f"{pad}    rel material:binding:physics = {PREFIX}{material}>\n"
                 f"{pad}    float physxCollision:contactOffset = {_f(offset)}\n"
                 f"{pad}    float physxCollision:restOffset = 0\n")
    if not visible:
        text += f'{pad}    token visibility = "invisible"\n'
    for line in attrs:
        text += f"{pad}    {line}\n"
    ops = ['"xformOp:translate"']
    text += f"{pad}    double3 xformOp:translate = {_v3(centre)}\n"
    if orient is not None:
        text += f"{pad}    quatf xformOp:orient = {_q(orient)}\n"
        ops.append('"xformOp:orient"')
    if scale_ is not None:
        text += f"{pad}    float3 xformOp:scale = {_v3(scale_)}\n"
        ops.append('"xformOp:scale"')
    return text + f"{pad}    uniform token[] xformOpOrder = [{', '.join(ops)}]\n{pad}}}\n\n"


def box(name, material, centre, half, *, collide=True, **kw):
    return shape("Cube", name, material, centre, collide=collide, scale_=half, attrs=("double size = 2",), **kw)


def span(name, material, x, y, z, **kw):
    """A box given by its (low, high) extents on each axis."""
    return box(name, material, tuple((a + b) / 2 for a, b in (x, y, z)), tuple((b - a) / 2 for a, b in (x, y, z)), **kw)


def cylinder(name, material, centre, radius, height, axis="Z", *, collide=True, **kw):
    return shape("Cylinder", name, material, centre, collide=collide,
                 attrs=(f'uniform token axis = "{axis}"', f"double height = {_f(height)}", f"double radius = {_f(radius)}"),
                 **kw)


def capsule(name, material, a, b, radius, *, collide=True, **kw):
    """A capsule whose straight part runs from ``a`` to ``b`` (the round ends reach ``radius`` beyond them)."""
    d = sub(b, a)
    return shape("Capsule", name, material, scale(add(a, b), .5), collide=collide, orient=quat_y_to(d),
                 attrs=('uniform token axis = "Y"', f"double height = {_f(norm(d))}", f"double radius = {_f(radius)}"),
                 **kw)


def mesh(name, material, points, faces, *, collide=False, visible=True, pad="            ", offset=.001):
    """A polygon mesh; with ``collide`` also a rigid collider (the convex hull of its points)."""
    apis = '"MaterialBindingAPI"' + (', "PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI"'
                                     if collide else "")
    text = (f'{pad}def Mesh "{name}" (\n{pad}    prepend apiSchemas = [{apis}]\n{pad})\n{pad}{{\n'
            f"{pad}    rel material:binding = {PREFIX}{material}>\n")
    if collide:
        text += (f"{pad}    rel material:binding:physics = {PREFIX}{material}>\n"
                 f"{pad}    float physxCollision:contactOffset = {_f(offset)}\n"
                 f"{pad}    float physxCollision:restOffset = 0\n"
                 f'{pad}    uniform token physics:approximation = "convexHull"\n')
    if not visible:
        text += f'{pad}    token visibility = "invisible"\n'
    return (text + f"{pad}    uniform bool doubleSided = 1\n"
            f"{pad}    int[] faceVertexCounts = [{', '.join(str(len(f)) for f in faces)}]\n"
            f"{pad}    int[] faceVertexIndices = [{', '.join(str(i) for f in faces for i in f)}]\n"
            f"{pad}    point3f[] points = [{', '.join(_v3(p) for p in points)}]\n"
            f'{pad}    uniform token subdivisionScheme = "none"\n{pad}}}\n\n')


def prism(name, material, outline, z0, z1, **kw):
    """A straight prism over a convex outline [(x, y), ...] between heights z0 and z1."""
    n = len(outline)
    points = [(x, y, z0) for x, y in outline] + [(x, y, z1) for x, y in outline]
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    return mesh(name, material, points, faces, **kw)


def hexahedron(name, material, low, high, **kw):
    """Eight corners: ``low`` and ``high`` are the four bottom and four top corners in the same order."""
    faces = [[3, 2, 1, 0], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]]
    return mesh(name, material, list(low) + list(high), faces, **kw)


def screen_box(name, material, across, up, size, lift, *, collide=False, visible=True, pad="            ", world=None,
               orient=SCREEN_Q):
    """A thin rectangle lying on the tuner's screen: (across, up) from the screen's middle, ``size`` (w, h)."""
    centre = L.on_screen(across, up, lift)
    if world is not None:
        centre = world(centre)
    return box(name, material, centre, (size[0] / 2, size[1] / 2, .0001), orient=orient, collide=collide,
               visible=visible, pad=pad)


# -- the parts -----------------------------------------------------------------------------------------------------
def catmull(p0, p1, p2, p3, t):
    return tuple(.5 * (2 * b + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t * t + (-a + 3 * b - 3 * c + d) * t ** 3)
                 for a, b, c, d in zip(p0, p1, p2, p3))


def body_outline():
    """The drawn body's outline: (y, half-width) through BODY_OUTLINE, smoothed."""
    pts = L.BODY_OUTLINE
    out = []
    for i in range(len(pts) - 1):
        p0, p1, p2, p3 = pts[max(i - 1, 0)], pts[i], pts[i + 1], pts[min(i + 2, len(pts) - 1)]
        for k in range(SAMPLES):
            out.append(catmull(p0, p1, p2, p3, k / SAMPLES))
    out.append(pts[-1])
    return out


def body_shell():
    """Soundboard, sides and back as one drawn mesh (doubled-sided quads)."""
    points, faces = [], []
    outline = body_outline()
    for y, w in outline:
        points += [(-w, y, L.TOP_Z), (w, y, L.TOP_Z), (-w, y, L.BACK_Z), (w, y, L.BACK_Z)]
    for i in range(len(outline) - 1):
        a, b = 4 * i, 4 * (i + 1)
        faces += [[a, a + 1, b + 1, b], [b + 2, b + 3, a + 3, a + 2], [a + 2, a, b, b + 2], [a + 1, a + 3, b + 3, b + 1]]
    last = 4 * (len(outline) - 1)
    faces += [[2, 3, 1, 0], [last, last + 1, last + 3, last + 2]]
    return mesh("body_shell", "mahogany", points, faces)


def disc(name, material, centre, r0, r1, z, segments=40):
    """A flat ring (a disc when r0 is 0) lying at height z."""
    points, faces = [], []
    for k in range(segments):
        a = 2 * math.pi * k / segments
        points += [(centre[0] + r0 * math.cos(a), centre[1] + r0 * math.sin(a), z),
                   (centre[0] + r1 * math.cos(a), centre[1] + r1 * math.sin(a), z)]
    for k in range(segments):
        j = (k + 1) % segments
        faces.append([2 * k, 2 * k + 1, 2 * j + 1, 2 * j])
    return mesh(name, material, points, faces)


GLYPHS = {   # strokes (x0, x1, y0, y1) in a unit cell: a blocky print of the note names on the screen
    "F": ((0., .22, 0., 1.), (0., 1., .8, 1.), (0., .8, .42, .58)),
    "G": ((0., 1., .8, 1.), (0., .22, 0., 1.), (0., 1., 0., .2), (.78, 1., 0., .5), (.5, 1., .42, .58)),
    "#": ((.22, .38, 0., 1.), (.62, .78, 0., 1.), (0., 1., .26, .40), (0., 1., .60, .74)),
}
GLYPH = (.0026, .0032)       # cell width and height
LABELS = {"lamp_fs": "F#", "lamp_g": "G", "lamp_gs": "G#"}


def tuner_parts():
    """The clip-on tuner, part of the instrument body: clip, head, screen, printed note names and scale marks."""
    parts = []
    x = (-L.CLIP_W / 2, L.CLIP_W / 2)
    face, back = L.TOP_Z, L.TOP_Z - L.HEAD_T
    parts.append(span("clip_upper_jaw", "tuner_black", x, L.CLIP_Y, (face, face + L.CLIP_T)))
    parts.append(span("clip_lower_jaw", "tuner_black", x, L.CLIP_Y, (back - L.CLIP_T, back)))
    parts.append(span("clip_spine", "tuner_black", x, (L.TIP_Y - L.CLIP_SPINE, L.TIP_Y), (back - L.CLIP_T, face + L.CLIP_T)))
    parts.append(span("clip_stalk", "tuner_black", (-.004, .004), (L.CLIP_Y[0] + .004, L.CLIP_Y[1]),
                      (face + L.CLIP_T, L.TUNER_CENTRE[2])))
    half = tuple(v / 2 for v in L.TUNER_HEAD)
    parts.append(box("tuner_head", "tuner_black", L.TUNER_CENTRE, half, orient=SCREEN_Q))
    parts.append(screen_box("screen", "screen", 0., 0., L.TUNER_SCREEN, .0001))
    for lamp, label in LABELS.items():
        across, up = L.LAMPS[lamp]
        left = across - len(label) * GLYPH[0] / 2 - (len(label) - 1) * .0003
        for i, char in enumerate(label):
            x0 = left + i * (GLYPH[0] + .0006)
            for k, (a0, a1, b0, b1) in enumerate(GLYPHS[char]):
                w, h = (a1 - a0) * GLYPH[0], (b1 - b0) * GLYPH[1]
                parts.append(screen_box(f"print_{lamp}_{i}_{k}", "print", x0 + (a0 + a1) / 2 * GLYPH[0],
                                        up - GLYPH[1] / 2 + (b0 + b1) / 2 * GLYPH[1], (w, h), .0004))
    px, py = L.NEEDLE_PIVOT
    for k, degrees in enumerate((-L.NEEDLE_MAX_DEG, 0., L.NEEDLE_MAX_DEG)):     # marks at -50, 0, +50 cents
        a = math.radians(degrees)
        r = L.NEEDLE_LEN + .0015
        parts.append(screen_box(f"scale_mark_{k}", "print", px + r * math.sin(a), py + r * math.cos(a), (.0008, .0008), .0004))
    parts.append(screen_box("needle_hub", "print", px, py, (.0024, .0024), .0004))
    return "".join(parts)


def instrument_parts():
    """The root link, instrument frame (its origin is on the table under the middle of the nut)."""
    parts = []
    # body: hidden convex slices for contact, one drawn shell, soundhole and rosette
    for k, ((y0, w0), (y1, w1)) in enumerate(zip(L.BODY_OUTLINE, L.BODY_OUTLINE[1:])):
        parts.append(prism(f"body_collider_{k:02d}", "mahogany", [(-w0, y0), (w0, y0), (w1, y1), (-w1, y1)],
                           L.BACK_Z, L.TOP_Z, collide=True, visible=False))
    parts.append(body_shell())
    hole = (0., L.SOUNDHOLE_Y)
    parts.append(disc("rosette", "rosette", hole, L.SOUNDHOLE_R, L.SOUNDHOLE_R + .004, L.TOP_Z + .00015))
    parts.append(disc("soundhole", "soundhole", hole, 0., L.SOUNDHOLE_R, L.TOP_Z + .0002))
    # neck under the fretboard, fretboard, frets, position dots, nut
    y1 = L.BODY_Y0
    n0, n1 = L.NUT_W / 2, L.NECK_W12 / 2
    parts.append(hexahedron("neck", "mahogany",
                            [(-n0, 0., L.neck_back_z(0.)), (n0, 0., L.neck_back_z(0.)), (n1, y1, L.neck_back_z(y1)), (-n1, y1, L.neck_back_z(y1))],
                            [(-n0, 0., L.TOP_Z), (n0, 0., L.TOP_Z), (n1, y1, L.TOP_Z), (-n1, y1, L.TOP_Z)], collide=True))
    f1 = L.neck_width(L.FB_END) / 2
    parts.append(hexahedron("fretboard", "rosewood",
                            [(-n0, 0., L.TOP_Z), (n0, 0., L.TOP_Z), (f1, L.FB_END, L.TOP_Z), (-f1, L.FB_END, L.TOP_Z)],
                            [(-n0, 0., L.FB_TOP), (n0, 0., L.FB_TOP), (f1, L.FB_END, L.FB_TOP), (-f1, L.FB_END, L.FB_TOP)],
                            collide=True))
    for n in range(1, L.FRETS + 1):
        y = L.fret(n)
        parts.append(box(f"fret_{n:02d}", "nickel", (0., y, L.FB_TOP + .0005), (L.neck_width(y) / 2, .0004, .0005),
                         collide=False))
    for n in (5, 7, 10):
        parts.append(cylinder(f"dot_{n:02d}", "dot", (0., (L.fret(n - 1) + L.fret(n)) / 2, L.FB_TOP + .0001), .0025,
                              .0002, collide=False))
    nut_top = L.string_point(L.TUNED, 0.)[2] - L.STRING_R
    parts.append(span("nut", "bone", (-n0, n0), (-.004, 0.), (L.TOP_Z, nut_top)))
    # bridge and saddle
    saddle_top = L.string_point(L.TUNED, L.SCALE)[2] - L.STRING_R
    parts.append(span("bridge", "rosewood", (-L.BRIDGE_W / 2, L.BRIDGE_W / 2), L.BRIDGE_Y, (L.TOP_Z, L.TOP_Z + L.BRIDGE_H)))
    parts.append(span("saddle", "bone", (-.028, .028), (L.SCALE - .0012, L.SCALE + .0012), (L.TOP_Z + L.BRIDGE_H, saddle_top)))
    # headstock, posts, and a box behind the headstock for each open gear (its plate, worm and gear)
    side = [(-w, y) for y, w in L.HEAD_OUTLINE]
    outline = side + [(-x, y) for x, y in reversed(side)]
    parts.append(prism("headstock", "mahogany", outline, L.TOP_Z - L.HEAD_T, L.TOP_Z, collide=True))
    for name in L.STRINGS:
        s, y = L.SIDE[name], L.POST_Y[name]
        parts.append(cylinder(f"post_{name}", "chrome", (L.POST_X[name], y, L.TOP_Z + L.POST_H / 2), L.POST_R, L.POST_H))
        x = sorted((s * L.HOUSING_IN, s * L.HOUSING_OUT))
        parts.append(span(f"housing_{name}", "chrome", x, (y - L.HOUSING[0] / 2, y + L.HOUSING[0] / 2),
                          (L.AXIS_Z - L.HOUSING[1] / 2, L.AXIS_Z + L.HOUSING[1] / 2)))
    # strings: nut to saddle (the G string's stretch over the soundhole is its own body), and drawn behind the nut
    gap = (L.PLUCK_Y - L.PLUCK_LEN / 2, L.PLUCK_Y + L.PLUCK_LEN / 2)
    for name in L.STRINGS:
        pieces = [(0., L.SCALE)] if name != L.TUNED else [(0., gap[0]), (gap[1], L.SCALE)]
        for k, (y0, y1) in enumerate(pieces):
            parts.append(capsule(f"string_{name}_{k}", "nylgut", L.string_point(name, y0), L.string_point(name, y1),
                                 L.STRING_R, offset=.0005))
        post = (L.POST_X[name] - L.SIDE[name] * L.POST_R, L.POST_Y[name], L.TOP_Z + L.POST_WRAP_Z)    # inner side
        parts.append(capsule(f"string_{name}_head", "nylgut", L.string_point(name, 0.), post, L.STRING_R * .8,
                             collide=False))
    return "".join(parts) + tuner_parts()


def button_parts(name):
    """A tuning button in its own frame (origin at the button's middle, on its shaft): a flat pearloid button on a
    short chrome shaft. At rest its broad faces face along the strings (it stands on edge)."""
    s = L.SIDE[name]
    parts = box("button", "pearloid", (0., 0., 0.), (L.BUTTON_LEN / 2, L.BUTTON_THICK / 2, L.BUTTON_WIDE / 2))
    shaft_mid = (L.HOUSING_OUT + L.BUTTON_IN) / 2 - (L.BUTTON_IN + L.BUTTON_LEN / 2)
    parts += cylinder("shaft", "chrome", (s * shaft_mid, 0., 0.), L.SHAFT_R, L.BUTTON_IN - L.HOUSING_OUT, "X",
                      collide=False)
    return parts


def pluck_parts():
    a = (0., -(L.PLUCK_LEN / 2 - L.STRING_R), 0.)
    b = (0., L.PLUCK_LEN / 2 - L.STRING_R, 0.)
    return capsule("stretch", "nylgut", a, b, L.STRING_R, offset=.0005)


def needle_parts():
    """In the needle's frame: across, up and out of the screen; origin on the pivot."""
    return (box("pointer", "needle", (0., L.NEEDLE_LEN / 2, 0.), (.0004, L.NEEDLE_LEN / 2, .00015), collide=False)
            + box("hidden_hub", "tuner_black", (0., 0., -.0045), (.001, .001, .001), visible=False))


# -- bodies and joints ----------------------------------------------------------------------------------------------
def body(name, mass, position, children, *, orient=IDENTITY):
    apis = ['"PhysicsRigidBodyAPI"', '"PhysxRigidBodyAPI"', '"PhysxContactReportAPI"', '"PhysicsMassAPI"']
    return (f'        def Xform "{name}" (\n            prepend apiSchemas = [{", ".join(apis)}]\n        )\n        {{\n'
            f"            float physics:mass = {_f(mass)}\n"
            "            bool physics:rigidBodyEnabled = 1\n"
            "            float physxContactReport:threshold = 0\n"
            "            float physxRigidBody:maxDepenetrationVelocity = 1\n"
            "            float physxRigidBody:sleepThreshold = 0\n"
            "            int physxRigidBody:solverPositionIterationCount = 32\n"
            "            int physxRigidBody:solverVelocityIterationCount = 8\n"
            f"            quatf xformOp:orient = {_q(orient)}\n"
            f"            double3 xformOp:translate = {_v3(position)}\n"
            '            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n\n'
            f"{children}        }}\n\n")


def joint(kind, name, body1, anchor0, axis, *, rot0=IDENTITY, limits=None, stiffness=0., damping=0., target=0.,
          max_force=None, armature=0., max_velocity=720.):
    """Joint from the instrument (body0) to ``body1``. ``kind`` "Revolute": limits and target in degrees, stiffness
    and damping per degree; "Prismatic": metres. The joint frame is ``rot0`` in the instrument's frame and the
    identity in body1's (body1 is authored turned by ``rot0``)."""
    drive = "angular" if kind == "Revolute" else "linear"
    text = (f'        def Physics{kind}Joint "{name}" (\n'
            f'            prepend apiSchemas = ["PhysicsDriveAPI:{drive}", "PhysxJointAPI"]\n        )\n        {{\n'
            f"            float drive:{drive}:physics:damping = {damping:.7g}\n")
    if max_force is not None:
        text += f"            float drive:{drive}:physics:maxForce = {max_force:.7g}\n"
    text += (f"            float drive:{drive}:physics:stiffness = {stiffness:.7g}\n"
             f"            float drive:{drive}:physics:targetPosition = {target:.7g}\n"
             f"            float drive:{drive}:physics:targetVelocity = 0\n"
             f'            uniform token drive:{drive}:physics:type = "force"\n'
             f'            uniform token physics:axis = "{axis}"\n'
             f"            rel physics:body0 = <{ART}instrument>\n"
             f"            rel physics:body1 = <{ART}{body1}>\n"
             f"            point3f physics:localPos0 = {_v3(anchor0)}\n"
             "            point3f physics:localPos1 = (0, 0, 0)\n"
             f"            quatf physics:localRot0 = {_q(rot0)}\n"
             "            quatf physics:localRot1 = (1, 0, 0, 0)\n")
    if limits is not None:
        text += (f"            float physics:lowerLimit = {limits[0]:.7g}\n"
                 f"            float physics:upperLimit = {limits[1]:.7g}\n")
    return text + (f"            float physxJoint:armature = {armature:.7g}\n"
                   f"            float physxJoint:maxJointVelocity = {max_velocity:.7g}\n        }}\n\n")


def material(name):
    color, roughness, metallic, friction, emissive = MATERIALS[name]
    glow = (f"                color3f inputs:emissiveColor = ({emissive[0]}, {emissive[1]}, {emissive[2]})\n"
            if emissive else "")
    return (f'        def Material "{name}" (\n            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]\n'
            "        )\n        {\n"
            f"            token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>\n"
            f"            float physics:dynamicFriction = {friction}\n            float physics:restitution = 0\n"
            f"            float physics:staticFriction = {friction}\n"
            '            uniform token physxMaterial:frictionCombineMode = "average"\n'
            '            uniform token physxMaterial:restitutionCombineMode = "min"\n\n'
            '            def Shader "Surface"\n            {\n                uniform token info:id = "UsdPreviewSurface"\n'
            f"                color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})\n" + glow +
            f"                float inputs:metallic = {metallic}\n"
            f"                float inputs:roughness = {roughness}\n                token outputs:surface\n            }}\n        }}\n\n")


def usd():
    top = scene.TOP

    def world(point):
        p = scene.to_station(point)
        return (p[0], p[1], p[2] + top)

    deg = math.pi / 180.
    pivot = L.on_screen(*L.NEEDLE_PIVOT, lift=NEEDLE_LIFT)
    # Every body is authored turned as the instrument is (ROT_Q), so each joint frame is its pose in the instrument frame.
    bodies = body("instrument", L.INSTRUMENT_MASS, world((0., 0., 0.)), instrument_parts(), orient=ROT_Q)
    for name in L.STRINGS:
        bodies += body(f"button_{name}", L.BUTTON_MASS, world(L.button_centre(name)), button_parts(name), orient=ROT_Q)
    bodies += body("pluck", L.PLUCK_MASS, world(L.pluck_point()), pluck_parts(), orient=ROT_Q)
    bodies += body("needle", L.NEEDLE_MASS, world(pivot), needle_parts(), orient=WORLD_SCREEN_Q)

    joints = ('        def PhysicsFixedJoint "root_joint"\n        {\n'
              f"            rel physics:body1 = <{ART}instrument>\n"
              f"            point3f physics:localPos0 = {_v3(world((0., 0., 0.)))}\n"
              "            point3f physics:localPos1 = (0, 0, 0)\n"
              f"            quatf physics:localRot0 = {_q(ROT_Q)}\n            quatf physics:localRot1 = (1, 0, 0, 0)\n"
              "        }\n\n")
    for name in L.STRINGS:          # no end stops: a geared tuner turns on and on
        joints += joint("Revolute", f"peg_{name}", f"button_{name}", L.button_centre(name), "X",
                        damping=L.BUTTON_DAMPING, max_force=L.BUTTON_DRAG, armature=L.BUTTON_ARMATURE)
    joints += joint("Prismatic", "pluck_slide", "pluck", L.pluck_point(), "X", limits=(-L.PLUCK_TRAVEL, L.PLUCK_TRAVEL),
                    stiffness=L.PLUCK_STIFFNESS, damping=L.PLUCK_DAMPING, armature=L.PLUCK_ARMATURE, max_velocity=2.)
    joints += joint("Revolute", "needle_pivot", "needle", pivot, "Z", rot0=SCREEN_Q, limits=(-60., 60.),
                    stiffness=L.NEEDLE_STIFFNESS * deg, damping=L.NEEDLE_DAMPING * deg,
                    target=L.NEEDLE_SIGN * L.NEEDLE_REST_DEG, armature=L.NEEDLE_ARMATURE)

    articulation = ('    def Xform "Ukulele" (\n        prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n'
                    "    )\n    {\n"
                    "        bool physxArticulation:enabledSelfCollisions = 0\n"
                    "        int physxArticulation:solverPositionIterationCount = 32\n"
                    "        int physxArticulation:solverVelocityIterationCount = 8\n"
                    "        float physxArticulation:sleepThreshold = 0\n"
                    "        float physxArticulation:stabilizationThreshold = 0\n\n" + bodies + joints + "    }\n\n")

    def placed(name, material, x, y, z, **kw):
        """A box given by its extents on the table (tabletop frame), written in world coordinates."""
        return span(name, material, x, y, (z[0] + top, z[1] + top), pad="        ", **kw)

    pad = "        "
    front, depth = scene.SOUNDBOARD_X, L.BODY_DEPTH         # the soundboard's plane; the back is depth behind it
    cheek, rise = L.REST_CHEEK
    # foam V-block under the lower bout's widest point: the rib sits on its base, two cheeks either side of the body
    y = scene.NUT_Y + L.BODY_REST_Y
    ys = (y - L.REST_LEN / 2, y + L.REST_LEN / 2)
    rib = scene.CENTRE_Z - L.body_half_width(L.BODY_REST_Y)
    static = placed("body_rest_base", "rest_pad", (front - cheek, front + depth + cheek), ys, (0., rib - .0005))
    static += placed("body_rest_front", "rest_pad", (front - cheek, front - .0005), ys, (rib - .0005, rib + rise))
    static += placed("body_rest_back", "rest_pad", (front + depth + .0005, front + depth + cheek), ys, (rib - .0005, rib + rise))
    # foam V-block under the neck's lower edge at the 3rd fret: base and a back cheek (a front one would reach the A string)
    y = scene.NUT_Y + L.NECK_REST_Y
    ys = (y - L.REST_LEN / 2, y + L.REST_LEN / 2)
    edge = scene.CENTRE_Z - L.neck_width(L.NECK_REST_Y) / 2
    back = front + L.TOP_Z - L.neck_back_z(L.NECK_REST_Y)
    static += placed("neck_rest_base", "rest_pad", (front - L.FB_T - cheek, back + cheek), ys, (0., edge - .0005))
    static += placed("neck_rest_back", "rest_pad", (back + .0005, back + cheek), ys, (edge - .0005, edge + rise))

    # the strap round the lower bout behind the bridge: over the top rib, down the soundboard and the back
    ys = (scene.NUT_Y + L.STRAP_Y[0], scene.NUT_Y + L.STRAP_Y[1])
    w = max(L.body_half_width(v) for v in (L.STRAP_Y[0], sum(L.STRAP_Y) / 2, L.STRAP_Y[1]))
    low = scene.CENTRE_Z - L.body_half_width(L.BODY_REST_Y) - .008
    t = L.STRAP_T
    drawn = placed("strap_top", "strap", (front - t, front + depth + t), ys, (scene.CENTRE_Z + w, scene.CENTRE_Z + w + t),
                   collide=False)
    drawn += placed("strap_front", "strap", (front - t, front), ys, (low, scene.CENTRE_Z + w + t), collide=False)
    drawn += placed("strap_back", "strap", (front + depth, front + depth + t), ys, (low, scene.CENTRE_Z + w + t),
                    collide=False)

    lamps = ""
    for lamp, (across, up) in L.LAMPS.items():
        lamps += screen_box(lamp, "lamp_note", across, up, L.LAMP_SIZE, .0002, visible=False, pad=pad, world=world,
                            orient=WORLD_SCREEN_Q)
    name, (across, up), size = L.IN_TUNE_LAMP
    lamps += screen_box(name, "lamp_green", across, up, size, .0002, visible=False, pad=pad, world=world,
                        orient=WORLD_SCREEN_Q)

    return ('#usda 1.0\n(\n    defaultPrim = "Fixture"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n\n'
            'def Xform "Fixture"\n{\n' + articulation +
            '    def Scope "Static"\n    {\n' + static + "    }\n\n"
            '    def Scope "Drawn"\n    {\n' + drawn + "    }\n\n"
            '    def Scope "Lamps"\n    {\n' + lamps + "    }\n\n"
            '    def Scope "Materials"\n    {\n' + "".join(material(name) for name in MATERIALS) + "    }\n}\n")


if __name__ == "__main__":
    scene.USD.write_text(usd())
    print("wrote", scene.USD.name, f"{scene.USD.stat().st_size / 1e3:.0f} kB")
