"""Reusable full-size hand file with a bare tang, and a push-on plastic file handle.

What it models
- make_file: an engineer's flat hand file, 200 mm cut length, 20 x 4-5 mm section (maker figures), and a
  tapered tang of 70 mm from a 10 x 5 mm shoulder to a 2 x 1 mm point (estimates). Origin at the shoulder,
  cut length along -y, tang along +y. The blade is cut into four convex pieces so that a grasp checker can
  find the two places a hand holds it (BODY_GRIP from the shoulder = the file's own mass centre;
  CARRY_GRIP = the mass centre of file plus seated handle). Teeth are visual only.
- make_handle: an oval two-material push-on file handle (size for 200 mm flat files), 50 g (published).
  Length 105 mm, 33 x 26 mm across its flats (estimates). Origin at the front face; +y runs to the butt.
  The nose has a 24 x 9 mm recess, 6 mm deep, that covers the file's shoulder when the handle is home; behind
  it a 13 mm bore (1.5 mm entry chamfer) holds the tang. Inside the bore two passive sprung faces stand for
  the handle's insert: each on a 1.5 mm slide across the tang, 1800 N/m, 0.3 mm preload, friction 0.6, faces
  7.4 mm apart at rest, 12-31 mm behind the front face. They touch the tang's edges only in its last ~17 mm
  of travel; the push starts at about 0.7 N and rises to about 3 N to seat, and a seated handle resists a
  pull of about 2.5 N, enough to hold the 145 g file (1.4 N) hanging from it (estimates from the spring,
  taper and friction numbers; not measured). No attachment, latch, teleport or rule: the handle goes on
  only by contact and friction.
Tuned numbers: spring 1800 N/m, preload 0.3 mm, travel 1.5 mm, friction 0.6 (insert), 0.7/0.6 (file),
0.65/0.55 (handle shell). All dimensions other than the file's cut length and section and the handle's mass
are estimates.
"""
import math
import numpy as np
from yam_parts import box
from yam_joints import Link, Slide
from yam_mesh import Mesh, MeshPart, MeshMechanism

STEEL = (.50, .53, .56)
TANG_GREY = (.36, .37, .39)
ORANGE = (.93, .38, .05)
BLACK = (.06, .065, .07)

TANG = .070               # tang length, shoulder to point (estimate)
CUT = .200                # cut length (maker)
BODY_GRIP = .0875         # file mass centre, from the shoulder toward the point
CARRY_GRIP = .052         # mass centre of file + seated handle, from the shoulder toward the point
H_LEN = .105              # handle length (estimate)
H_W, H_H = .033, .026     # handle width and height across its flats (estimate)
RECESS = .006             # depth of the nose recess that covers the shoulder
RECESS_W, RECESS_H = .024, .009
BORE_R = .0065
AXIS_H = H_H / 2          # bore axis above the bench when the handle lies on its flat


def convex(name, points, faces, color, proxy=True):
    p = np.array(points, float); f = np.array(faces, int)
    tri = (p - p.mean(0))[f]
    if np.einsum('ij,ij->i', tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        f = f[:, ::-1]
    lo, hi = p.min(0), p.max(0)
    px = (box((lo + hi) / 2, np.maximum(hi - lo, .0002)),) if proxy else ()
    return Mesh(name, p, f, collision='convex', color=color, proxy=px)


def loft(name, rings, color, proxy=True):
    """Convex solid through equal-size rings of points (each ring a convex polygon)."""
    n = len(rings[0]); pts = np.array(rings, float).reshape(-1, 3); fs = []
    for i in range(len(rings) - 1):
        for j in range(n):
            a = i * n + j; b = i * n + (j + 1) % n; c = a + n; d = b + n
            fs.extend(((a, b, d), (a, d, c)))
    fs.extend((0, j + 1, j) for j in range(1, n - 1))
    k = (len(rings) - 1) * n
    fs.extend((k, k + j, k + j + 1) for j in range(1, n - 1))
    return convex(name, pts, fs, color, proxy)


def rect(y, w, t, cx=0.):
    return [(cx - w / 2, y, -t / 2), (cx + w / 2, y, -t / 2), (cx + w / 2, y, t / 2), (cx - w / 2, y, t / 2)]


def make_file(pos):
    """Hand file lying flat; origin at the shoulder, point along -y, tang along +y."""
    def thick(y):  # 4 mm at the point rising to 5 mm, 10 mm short of the shoulder
        return .004 + .001 * min(1., (y + CUT) / .190)
    meshes = []
    cuts = [-CUT, -.110, -.065, -.039, 0.]
    names = ['point_end', 'body', 'body_front', 'shoulder_end']
    for name, a, b in zip(names, cuts[:-1], cuts[1:]):
        rings = [rect(a, .020, thick(a))]
        if a < -.010 < b:
            rings.append(rect(-.010, .020, .005))
        rings.append(rect(b, .020, thick(b)))
        meshes.append(loft(name, rings, STEEL))
    meshes.append(loft('tang', [rect(0, .010, .005), rect(TANG, .002, .001)], TANG_GREY))
    for i in range(48):  # crossed tooth ridges, drawn only (no collider proxy, no cutting)
        y = -.194 + i * .0038
        m = loft('tooth' + str(i), [rect(y, .019, .00014), rect(y + .0003, .019, .00014)], (.30, .32, .33), False)
        p = m.points.copy(); p[:, 2] += .00253
        meshes.append(Mesh(m.name, p, m.faces, collision='convex', color=m.color, proxy=()))
    return MeshPart('file', pos, (), mass=.145, meshes=tuple(meshes), color=STEEL,
                    friction=(.7, .6), contact_offset=.0005)


def _oval(y, scale=1.):
    """Eight corners of the handle's oval section (flats on top, bottom and both sides)."""
    a = scale * H_W / 2 / math.cos(math.pi / 8); b = scale * H_H / 2 / math.cos(math.pi / 8)
    return [(a * math.cos(t), y, b * math.sin(t)) for t in (math.pi / 8 + k * math.pi / 4 for k in range(8))]


def _clip_top(y, z0):
    """The oval section above z = z0 (z0 below the side-flat corners), as a convex hexagon."""
    o = _oval(y)
    return [(o[0][0], y, z0), o[0], o[1], o[2], o[3], (o[3][0], y, z0)]


def make_handle(pos):
    """Push-on file handle lying on its flat; front face (recess mouth) at the origin, butt toward +y."""
    body = []
    # Nose: four slabs framing the recess that covers the shoulder (front face y=0 to recess floor y=RECESS).
    zr, xr = RECESS_H / 2, RECESS_W / 2
    top = [_clip_top(y, zr) for y in (0., RECESS)]
    body.append(loft('nose_top', top, BLACK))
    body.append(loft('nose_bottom', [[(x, y, -z) for x, y, z in r] for r in top], BLACK))
    for s, nm in ((1, 'nose_l'), (-1, 'nose_r')):
        xo = H_W / 2
        body.append(loft(nm, [[(s * xr, y, -zr), (s * xo, y, -zr), (s * xo, y, zr), (s * xr, y, zr)][::s]
                              for y in (0., RECESS)], BLACK))
    # Body: eight sectors round the bore; entry chamfer 1.5 mm, bore radius 6.5 mm to 90 mm, solid butt.
    ys = [(RECESS, RECESS + .0015, BORE_R + .0015, BORE_R), (RECESS + .0015, .090, BORE_R, BORE_R)]
    for k in range(8):
        t0 = math.pi / 8 + k * math.pi / 4; t1 = t0 + math.pi / 4
        for part, (y0, y1, r0, r1) in enumerate(ys):
            rings = []
            for y, r in ((y0, r0), (y1, r1)):
                o = _oval(y)
                rings.append([(r * math.cos(t0), y, r * math.sin(t0)), o[k], o[(k + 1) % 8],
                              (r * math.cos(t1), y, r * math.sin(t1))])
            body.append(loft('shell%d_%d' % (k, part), rings, ORANGE))
    body.append(loft('butt', [_oval(.090), _oval(H_LEN, .62)], ORANGE))
    meshes = {'base': tuple(body)}; links = []
    for sign, name in ((-1, 'insert_a'), (1, 'insert_b')):
        face, back = sign * .0037, sign * .0057
        rings = [[(face, y, -.003), (back, y, -.003), (back, y, .003), (face, y, .003)] for y in (.012, .031)]
        if sign > 0:
            rings = [r[::-1] for r in rings]
        meshes[name] = (loft(name + '_pad', rings, BLACK),)
        lo, hi = (-.0015, 0.) if sign < 0 else (0., .0015)
        links.append(Link(name, (), Slide('x', (sign * .004, .0215, 0.), lo, hi, spring=1800, damping=4,
                                          rest=-sign * .0003, armature=.01), .002, color=BLACK, friction=(.6, .6)))
    return MeshMechanism('handle', pos, (), links, mass=.050, color=ORANGE, meshes=meshes,
                         friction=(.65, .55), contact_offset=.0005)
