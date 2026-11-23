"""A shopping-trolley coin lock with its chain key, built from the kit's parts (one self-contained module).

What it models (lock body 90 x 70 x 68 mm on a 25 mm handle tube; 260 mm stainless chain; a lock that takes coins of
23-28 mm, here a US quarter). Lock frame: origin on the handle tube's axis under the lock's middle, axes world
aligned; the tube runs along y; the key goes in along +y through a funnel in the lock's -y end; the coin pusher
slides out of the lock's -x side. All in metres; the comments give millimetres.

near_lock(): the returning trolley's lock, a MeshMechanism (self-collision on, so its parts push each other):
  - pusher (slide along x, 30 mm travel, 12 g): the coin carrier. A compression spring (40 N/m, rest 60 mm out:
    2.4 N when in, 1.2 N when out; 10 N s/m damper) drives it out. While in, its tooth rests on the plunger's pawl.
    Its pocket (25.3 x 2.6 mm, 12.6 mm deep) holds the quarter on edge; out, the coin's upper half (11.6 mm)
    stands clear above the pusher, 6.9 mm outside the lock's side.
  - plunger (slide along y, 6.5 mm travel to a hard stop, 4 g, spring 80 N/m preloaded 10 mm: 0.8-1.3 N): the
    key's tip meets it 18 mm past the mouth. At 5.5 mm its pawl clears the pusher's tooth and the pusher springs out.
  - ball (7 mm locking ball on a slide along z, 4 mm lift, spring 60 N/m preloaded 5 mm: 0.3-0.4 N, simulated at
    3 g): it sits 1.5 mm down into the key slot 12 mm past the mouth; the key's front edge lifts it, and at full
    depth (tip 22.9-25.1 mm in) it drops into the key's 8 x 7 mm hole. Once the pusher is out, the pusher's
    blocker plate lies 0.3 mm over the ball, so the ball cannot rise and the key cannot be pulled out (Wanzl
    EP0427819B1's arrangement: ball on the key, coin slide at right angles on a higher plane, spring-ejected slide).
  Tuned estimate of the push to latch: about 2.5 N (plunger 1.3 N at the stop, pawl friction ~0.6 N, the ball and
  the key's own friction ~0.3 N, chain drag ~0.2 N); the ball needs ~0.6 N to ride up. Inner parts' friction 0.2.
front_lock(): the trolley in front: the same lock body (fixed, no moving parts) with its chain and key: 13 segments of
  20 mm (two crossed hinges per node, 1.4 g a segment), three hinges at the key's eye, and the key, a flat stainless
  plate 58 x 15 x 3 mm (the last 6 mm a 9 mm wide eye lug) with an 8 x 7 mm hole 8 mm behind its tip (18 g). Self-collision off: the chain and key do
  not collide with their own lock (they do with everything else).
quarter(): a US quarter (24.26 mm, 1.75 mm, 5.67 g) as a 32-sided prism of the disc's area, standing in the pocket.
"""
import math

import numpy as np

import yam_mesh
from yam_joints import Hinge, Link, Slide
from yam_mesh import Mesh, MeshMechanism, MeshPart
from yam_parts import ball, box, cyl

MM = 0.001
Z_T = 0.100  # the handle tube's axis above the bench (low stands: the arms reach the locks)
NEST = 0.178  # handle to handle: nesting distance of a tight-nesting cart (7 in)
TUBE_R, TUBE_L = 0.0125, 0.300
LOCK_RED, LOCK_BLACK, CHROME, STEEL, POST_GREY = (0.72, 0.09, 0.09), (0.10, 0.10, 0.11), (0.78, 0.79, 0.81), \
    (0.62, 0.63, 0.66), (0.35, 0.35, 0.37)
LOCK_FRICTION = (0.45, 0.35)  # plastic on steel (estimate)
STEEL_FRICTION = (0.50, 0.40)

# slot geometry (lock frame)
MOUTH_Y = -0.045  # outer face of the -y end wall
KEY_Z = 0.0175  # key centre height when it rests on the slot's floor (floor 16 mm, key 3 mm, slot 3.5 mm)
HOME_DEPTH = 0.0245  # tip past the mouth at the plunger's hard stop
LATCH_DEPTH = (0.020, 0.0255)  # tip depth while latched (held by the ball it backs off ~2 mm from the stop)
PUSHER_TRAVEL = 0.030
PUSHER_OUT = -PUSHER_TRAVEL
PUSHER_TOP = 0.0399
POCKET_X, POCKET_Y, POCKET_FLOOR = -0.024, -0.020, 0.02727

COIN_R, COIN_T, COIN_MASS = 0.02426 / 2, 0.00175, 0.00567  # US Mint
COIN_SILVER = (0.66, 0.67, 0.70)
COIN_SIDES = 32
COIN_RC = COIN_R * math.sqrt(2 * math.pi / (COIN_SIDES * math.sin(2 * math.pi / COIN_SIDES)))
COIN_EDGE_QUAT = (0.0, 0.0, 0.0, 1.0)  # as authored: on edge, faces along y

KEY_W, KEY_T, KEY_L, KEY_MASS = 0.015, 0.003, 0.058, 0.018
SEG, SEGS = 0.020, 13


def bx(x0, x1, y0, y1, z0, z1, color=None):
    """A box from millimetre bounds."""
    lo, hi = np.array([x0, y0, z0]) * MM, np.array([x1, y1, z1]) * MM
    return box(tuple((lo + hi) / 2), tuple(hi - lo), color)


def hexa(name, corners, color):
    """A convex six-faced solid from 8 corners (mm), indexed ix + 2 iy + 4 iz like a box's."""
    p = np.asarray(corners, float) * MM
    quads = ((0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5))
    c = p.mean(axis=0)
    faces = []
    for a, b, cc, d in quads:
        for tri in ((a, b, cc), (a, cc, d)):
            n = np.cross(p[tri[1]] - p[tri[0]], p[tri[2]] - p[tri[0]])
            faces.append(tri if np.dot(n, p[tri[0]] - c) > 0 else tri[::-1])
    lo, hi = p.min(axis=0), p.max(axis=0)
    return Mesh(name, p, np.asarray(faces, int), collision="convex", color=color,
                proxy=(box(tuple((lo + hi) / 2), tuple(hi - lo)),))


def _wedge(name, x0, x1, ya, yb, z_lo_a, z_lo_b, z_hi_a, z_hi_b):
    """Corners: x across, y from ya (mouth) to yb (inside); bottom and top heights at each end."""
    pts = []
    for iz in (0, 1):
        for iy in (0, 1):
            for ix in (0, 1):
                y = (ya, yb)[iy]
                z = ((z_lo_a, z_lo_b), (z_hi_a, z_hi_b))[iz][iy]
                pts.append(((x0, x1)[ix], y, z))
    return hexa(name, pts, LOCK_BLACK)


def _side_wedge(name, sign):
    """A funnel cheek: inner face from 13.5 mm (mouth) to 8.5 mm (inside) off the slot's centre line."""
    pts = []
    for iz in (0, 1):
        for iy in (0, 1):
            for ix in (0, 1):
                y = (-45.0, FUNNEL_END)[iy]
                inner = (13.5, 8.5)[iy]
                x = (inner, 16.0)[ix] if sign > 0 else (-16.0, -inner)[ix]
                pts.append((x, y, (12.5, 27.0)[iz]))
    return hexa(name, pts, LOCK_BLACK)


FUNNEL_END = -39.0  # mm: the lead-in is 6 mm deep


def funnel():
    """The key slot's lead-in, 6 mm deep: mouth 27 x 11 mm narrowing to the 17 x 3.5 mm slot."""
    return (_wedge("funnel_floor", -16.0, 16.0, -45.0, FUNNEL_END, 12.5, 12.5, 13.0, 16.0),
            _wedge("funnel_roof", -16.0, 16.0, -45.0, FUNNEL_END, 24.0, 19.5, 27.0, 27.0),
            _side_wedge("funnel_left", 1), _side_wedge("funnel_right", -1))


def housing():
    """Lock body 70 (x) x 90 (y) x 68 (z) mm, clamped on the tube, with its key slot, coin opening and the stand."""
    R, K = LOCK_RED, LOCK_BLACK
    s = [
        bx(32, 35, -45, 45, -13, 55, R),  # +x side
        bx(-35, -32, -45, -29.5, -13, 55, R),  # -x side, around the pusher's opening and the coin slit
        bx(-35, -32, -10.5, 45, -13, 55, R),
        bx(-35, -32, -29.5, -10.5, -13, 24.3, R),
        bx(-35, -32, -29.5, -21.6, 40.4, 55, R),
        bx(-35, -32, -18.4, -10.5, 40.4, 55, R),
        bx(-35, -32, -21.6, -18.4, 52.6, 55, R),
        bx(-32, 32, -45, 45, 52.6, 55, R),  # top cover
        bx(-32, 32, 42, 45, -13, 52.6, R),  # +y end
        bx(-32, -16, -45, -42, -13, 52.6, R),  # -y end, around the funnel
        bx(16, 32, -45, -42, -13, 52.6, R),
        bx(-16, 16, -45, -42, 27, 52.6, R),
        bx(-16, 16, -45, -42, -13, 12.5, R),
        bx(-32, 32, -42, 42, -13, -11, R),  # underside
        bx(-8.5, 8.5, -39, -10, 13, 16, K),  # key slot (17 x 3.5 mm): floor, cheeks, roof
        bx(-16, -8.5, -39, -10, 13, 21.5, K),
        bx(8.5, 16, -39, -10, 13, 21.5, K),
        bx(-16, 16, -39, -10, 19.5, 21.5, K),
        cyl((0.0, 0.0, 0.0), TUBE_R, TUBE_L, "y", CHROME),  # handle tube
    ]
    for y in (-0.140, 0.140):  # two posts under the tube, each on a foot
        s.append(cyl((0.0, y, (-Z_T + 0.004 - TUBE_R) / 2), 0.009, Z_T - TUBE_R - 0.004, "z", POST_GREY))
        s.append(box((0.0, y, -Z_T + 0.002), (0.030, 0.050, 0.004), POST_GREY))
    return s


def _pusher_link():
    K = LOCK_BLACK
    shapes = (
        bx(-39, 1, -29, -21.3, 24.8, 39.9, K),  # the block stands 4 mm proud of the lock's side when in
        bx(-39, 1, -18.7, -11, 24.8, 39.9, K),
        bx(-39, 1, -21.3, -18.7, 24.8, 27.27, K),  # pocket floor
        bx(-39, -36.63, -21.3, -18.7, 27.27, 39.9, K),  # pocket ends (25.26 mm apart)
        bx(-11.37, 1, -21.3, -18.7, 27.27, 39.9, K),
        bx(-3, 1, -22, -19, 21, 24.8, K),  # tooth (rests on the plunger's pawl while in)
        bx(5, 33, -37, -29, 24.8, 26.8, K),  # blocker plate: over the ball once the pusher is out
    )
    joint = Slide("x", (POCKET_X, POCKET_Y, PUSHER_TOP), PUSHER_OUT, 0.0, damping=10.0, spring=40.0, rest=-0.060,
                  armature=0.005)
    return Link("pusher", shapes, joint, 0.012, color=K, friction=(0.30, 0.25))


def _plunger_link():
    shapes = (bx(-7, 7, -27, -15, 16.5, 19.5, STEEL), bx(-7, -3, -24.5, -16, 19.5, 24, STEEL))
    joint = Slide("y", (0.0, -0.027, KEY_Z), 0.0, HOME_DEPTH - 0.018, damping=2.0, spring=80.0, rest=-0.010,
                  armature=0.003)
    return Link("plunger", shapes, joint, 0.004, color=STEEL, friction=(0.20, 0.15))


def _ball_link():
    joint = Slide("z", (0.0, -0.033, 0.021), 0.0, 0.004, damping=0.5, spring=60.0, rest=-0.005, armature=0.002)
    return Link("ball", (ball((0.0, -0.033, 0.021), 0.0035, STEEL),), joint, 0.003, color=STEEL,
                friction=(0.20, 0.15))


def near_lock(name, pos):
    return MeshMechanism(name, pos, housing(), (_pusher_link(), _plunger_link(), _ball_link()), mass=None,
                         color=LOCK_RED, friction=LOCK_FRICTION, contact_offset=0.001, self_collide=True,
                         meshes={"base": funnel()})


# ---------------------------------------------------------------- chain and key (front lock frame, at q = 0)
ANCHOR = np.array([-0.020, -0.048, -Z_T + 0.0015 + 6 * SEG])  # chain eye on the -y end, 20 mm toward the near lock


def chain_nodes():
    """14 node points: 6 segments hanging, then 7 along -y on the bench, past the end of the handle tube."""
    nodes = [ANCHOR.copy()]
    for d, n in ((np.array([0, 0, -1.0]), 6), (np.array([0, -1.0, 0]), 7)):
        for _ in range(n):
            nodes.append(nodes[-1] + d * SEG)
    return np.array(nodes)


NODES = chain_nodes()
EYE = NODES[-1]
KEY_BACK = EYE + np.array([0.0, 0.002, 0.0])  # the key lies on along -y from its eye, tip away from the chain
KEY_TIP = KEY_BACK + np.array([0.0, -KEY_L, 0.0])
KEY_GRIP = KEY_TIP + np.array([0.0, 0.050, 0.0])  # where the pads pinch it, 50 mm behind the tip


def _axes(d):
    if abs(d[2]) > 0.5:
        return ("x", "y")
    if abs(d[0]) > 0.5:
        return ("y", "z")
    return ("x", "z")


def _key_shapes():
    t = KEY_TIP
    x0, z0 = t[0] / MM, t[2] / MM - KEY_T / 2 / MM
    yt = t[1] / MM

    def kb(xa, xb, ya, yb):
        return bx(x0 + xa, x0 + xb, yt + ya, yt + yb, z0, z0 + KEY_T / MM, STEEL)

    return (kb(-7.5, 7.5, 0, 8), kb(-7.5, -3.5, 8, 16), kb(3.5, 7.5, 8, 16), kb(-7.5, 7.5, 16, 52),
            kb(-4.5, 4.5, 52, 58))


CHAIN_LIMIT = 135.0
KEY_LIMITS = {"key_z": 170.0, "key_x": 100.0, "key": 100.0}


def chain_links():
    links, parent = [], None
    for i in range(SEGS):
        a, b = NODES[i], NODES[i + 1]
        d = (b - a) / SEG
        ax1, ax2 = _axes(d)
        k = f"c{i + 1:02d}a"
        links.append(Link(k, (ball(tuple(a), 0.0010, STEEL),),
                          Hinge(ax1, tuple(a), -CHAIN_LIMIT, CHAIN_LIMIT, damping=2e-5, armature=1e-6), 0.0002,
                          parent=parent, color=STEEL, friction=STEEL_FRICTION))
        s = f"c{i + 1:02d}"
        balls = tuple(ball(tuple(a + (b - a) * f), 0.0015, STEEL) for f in (1 / 6, 0.5, 5 / 6))
        links.append(Link(s, balls, Hinge(ax2, tuple(a), -CHAIN_LIMIT, CHAIN_LIMIT, damping=2e-5, armature=1e-6),
                          0.0012, parent=k, color=STEEL, friction=STEEL_FRICTION))
        parent = s
    links.append(Link("key_z", (ball(tuple(EYE), 0.0010, STEEL),),
                      Hinge("z", tuple(EYE), -KEY_LIMITS["key_z"], KEY_LIMITS["key_z"], damping=2e-5, armature=1e-6),
                      0.0003, parent=parent, color=STEEL, friction=STEEL_FRICTION))
    links.append(Link("key_x", (ball(tuple(EYE), 0.0010, STEEL),),
                      Hinge("x", tuple(EYE), -KEY_LIMITS["key_x"], KEY_LIMITS["key_x"], damping=2e-5, armature=1e-6),
                      0.0003, parent="key_z", color=STEEL, friction=STEEL_FRICTION))
    links.append(Link("key", _key_shapes(),
                      Hinge("y", tuple(EYE), -KEY_LIMITS["key"], KEY_LIMITS["key"], damping=2e-5, armature=1e-6),
                      KEY_MASS, parent="key_x", color=STEEL, friction=STEEL_FRICTION))
    return tuple(links)


def front_lock(name, pos):
    lug = bx(ANCHOR[0] / MM - 3, ANCHOR[0] / MM + 3, -46, -44, ANCHOR[2] / MM - 3, ANCHOR[2] / MM + 3, STEEL)
    return MeshMechanism(name, pos, housing() + [lug], chain_links(), mass=None, color=LOCK_RED,
                         friction=LOCK_FRICTION, contact_offset=0.001, self_collide=False,
                         meshes={"base": funnel()})


# ---------------------------------------------------------------- the quarter
def pocket_local(q_pusher=0.0):
    """The coin's centre resting in the pocket, lock frame."""
    return np.array([POCKET_X + q_pusher, POCKET_Y, POCKET_FLOOR + COIN_R])


def coin_out_local():
    return pocket_local(PUSHER_OUT)


def quarter(name, lock):
    a = np.arange(COIN_SIDES) * 2 * math.pi / COIN_SIDES
    disc = yam_mesh.prism(np.column_stack((COIN_RC * np.cos(a), COIN_RC * np.sin(a))), COIN_T, axis="y",
                          name="disc", collision="convex", color=COIN_SILVER,
                          proxy=(cyl((0.0, 0.0, 0.0), COIN_R, COIN_T, "y"),))
    p = np.asarray(lock.pos, float) + pocket_local(0.0)
    return MeshPart(name, tuple(float(v) for v in p), (), mass=COIN_MASS, color=COIN_SILVER,
                    friction=(0.40, 0.35), contact_offset=0.001, meshes=(disc,))


def coin_in_pusher(near_state, c_world):
    """The coin's centre still in the pusher's pocket (where the pusher is now), not lifted out."""
    c = near_state.to_local(np.asarray(c_world, float)) - pocket_local(near_state.joints["pusher"])
    return bool(abs(c[0]) < 0.006 and abs(c[1]) < 0.003 and c[2] < 0.006)


def coin_low_z(coin_state):
    """Height of the coin's lowest point (a disc of COIN_R, COIN_T with its face normal along its frame's y)."""
    n_z = abs(float(coin_state.rotation()[2, 1]))
    return float(coin_state.origin[2]) - COIN_R * math.sqrt(max(0.0, 1.0 - n_z * n_z)) - COIN_T / 2 * n_z


def coin_flat(coin_state):
    """Lying flat: the coin's face normal (its frame's y axis) within 20 degrees of vertical."""
    return bool(abs(float(coin_state.rotation()[2, 1])) > math.cos(math.radians(20.0)))


# Offline test layouts (chain joints fitted by damped least squares so the key sits in the near lock): degrees.
KEY_HOME_Q = {'c01a': 70.291, 'c01': 36.507, 'c02a': 20.883, 'c02': 30.328, 'c03a': -0.839, 'c03': 18.439, 'c04a': -4.331, 'c04': 4.854, 'c05a': 0.113, 'c05': -14.845, 'c06a': 6.869, 'c06': -45.095, 'c07a': -1.614, 'c07': -48.182, 'c08a': 0.649, 'c08': -35.95, 'c09a': -6.842, 'c09': -21.285, 'c10a': -17.705, 'c10': -6.531, 'c11a': -30.311, 'c11': 8.476, 'c12a': -36.932, 'c12': 27.6, 'c13a': -24.499, 'c13': 59.586, 'key_z': 94.815, 'key_x': -49.186, 'key': -68.659}
KEY_MOUTH_Q = {'c01a': 57.154, 'c01': 50.396, 'c02a': 8.897, 'c02': 29.904, 'c03a': -5.46, 'c03': 13.59, 'c04a': -4.543, 'c04': 0.6, 'c05a': 0.426, 'c05': -17.478, 'c06a': 3.764, 'c06': -48.784, 'c07a': -11.004, 'c07': -56.237, 'c08a': -2.981, 'c08': -34.787, 'c09a': -8.101, 'c09': -15.714, 'c10a': -18.427, 'c10': 0.048, 'c11a': -31.957, 'c11': 15.215, 'c12a': -40.831, 'c12': 35.986, 'c13a': -32.16, 'c13': 71.604, 'key_z': 110.644, 'key_x': -59.097, 'key': -60.453}
