"""Lock out a universal travel adapter's UK pins, then push the adapter fully into a UK single-socket extension lead.

An EPICKA TA-105 universal travel adapter (71.1 x 50.0 x 52.1 mm, 190 g) stands on the table on its pin face, all pin
sets retracted. Its UK pins come out of the bottom when the middle of the three toggles on its side is pushed all the
way along its slot: in the last 0.8 mm a sprung catch drops behind the pin carrier ("pop") and locks the pins out. A
Masterplug EXS1315B-01 single-socket heavy-duty extension lead (90 x 60 x 33 mm) lies flat on the table, socket face
up. The robot lifts the adapter, pushes the UK toggle down until it locks, carries the adapter over the socket, lines
the three pins up with the socket's openings and pushes the adapter home until its base sits flush on the socket face.

The socket's two current-carrying openings hold sprung contact plates that grip the pins (about 8 N to push home,
estimate). If the toggle was not pushed far enough to lock, that push drives the pins back into the adapter.
Metres; x forward from the arm bases, y left, z up. Module sections "the travel adapter" and "the socket" are reusable.
"""
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_parts import box, cyl  # noqa: E402
from yam_joints import Link, Mechanism, Slide  # noqa: E402
from yam_mesh import Mesh, MeshMechanism  # noqa: E402
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
RELEASE_CLEAR = 0.030
STILL_S = 1.0
TEST_GRIPPERS = None
DIAG = False  # GPU diagnostic copies print PLUG_DIAG lines
DEMO_PACE = 1.0  # scales every demo duration except the fingers' close and open
DEMO_TOGGLE = 1.0  # wrong-way copies: push the toggle only this fraction of its travel (0.5 = half way, not locked)


def _run_context():
    context = {"env": None, "t": None, "step": 0, "episode": 0}
    try:
        from yam_arena import inspect_embodiment as _ie
    except ImportError:
        return context
    cls = _ie.YamArenaEmbodiment
    if hasattr(cls, "_rc_run"):
        return cls._rc_run
    original_step, original_reset = cls._task_step, cls.reset

    def task_step(self, *args, **kwargs):
        context["env"], context["t"] = self._env, self._t
        context["step"] += 1
        return original_step(self, *args, **kwargs)

    def reset(self, *args, **kwargs):
        result = original_reset(self, *args, **kwargs)
        context["episode"] += 1
        context["t"] = 0.0
        context["env"] = self._env
        return result

    task_step._rc_wrapped = reset._rc_wrapped = True
    cls._rc_run = context
    cls._task_step, cls.reset = task_step, reset
    return context


_RUN = _run_context()
_MEMORY_EPISODE = None
_EVER = {}


def grippers():
    if TEST_GRIPPERS is not None:
        return TEST_GRIPPERS
    env = _RUN["env"]
    if env is None:
        return None
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, FINGER_OPEN as OPEN

    out = {}
    for arm in ("left", "right"):
        art = env.unwrapped.scene[arm + "_robot"]
        arm_ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
        finger_ids = art.find_joints(FINGER_JOINTS, preserve_order=True)[0]
        joints = art.data.joint_pos
        q = getattr(joints, "torch", joints)[0]
        if hasattr(q, "detach"):
            q = q.detach().cpu().numpy()
        q = np.asarray(q)
        tcp = ik.tcp_pose(q[arm_ids], station.BASES[arm])[0]
        opening = float(np.clip(np.mean(q[finger_ids]) / OPEN, 0.0, 1.0))
        out[arm] = (tcp, opening)
    return out


def ever(key, cond):
    """Remember a satisfied condition until the next episode; offline each call stands alone."""
    global _MEMORY_EPISODE
    if _RUN["t"] is None:
        return bool(cond)
    if _MEMORY_EPISODE != _RUN["episode"]:
        _EVER.clear()
        _MEMORY_EPISODE = _RUN["episode"]
    _EVER[key] = bool(_EVER.get(key, False) or cond)
    return _EVER[key]


TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S, SCORE_OVER = 1.0, "end"
RUN_LIMIT_STEPS = 2400  # 120 s at 20 Hz
MM = 0.001
DEG = math.pi / 180

ADAPTER_GREY = (0.62, 0.64, 0.68)  # the TA-105 in its grey finish
TOGGLE_GREY = (0.30, 0.31, 0.34)
MARK_DARK = (0.10, 0.10, 0.11)
PIN_NICKEL = (0.70, 0.68, 0.60)  # nickel-plated pins, a shade darker so the top camera keeps them
SLEEVE_BLACK = (0.08, 0.08, 0.08)
SOCKET_BLACK = (0.09, 0.09, 0.10)
FACE_BLACK = (0.14, 0.14, 0.15)
CABLE_BLACK = (0.05, 0.05, 0.05)
CONTACT_BRASS = (0.72, 0.58, 0.25)


# ---------------------------------------------------------------- convex pieces
def _oriented(name, points, faces, color, proxy):
    points, faces = np.asarray(points, float), np.asarray(faces, int)
    tri = (points - points.mean(axis=0))[faces]
    if np.einsum("ij,ij->i", tri[:, 0], np.cross(tri[:, 1], tri[:, 2])).sum() < 0:
        faces = faces[:, ::-1]
    return Mesh(name, points, faces, collision="convex", color=color, proxy=tuple(proxy))


def _prism_x(name, yz, x0, x1, color, proxy_scale=1.0):
    """A convex prism: the convex polygon `yz` ((y, z) points in order, metres) extruded from x0 to x1."""
    k = len(yz)
    points = [(x0, y, z) for y, z in yz] + [(x1, y, z) for y, z in yz]
    faces = []
    for j in range(k):
        a, b = j, (j + 1) % k
        faces += [(a, b, b + k), (a, b + k, a + k)]
    faces += [(0, j + 1, j) for j in range(1, k - 1)]
    faces += [(k, k + j, k + j + 1) for j in range(1, k - 1)]
    p = np.asarray(points, float)
    lo, hi = p.min(axis=0), p.max(axis=0)
    proxy = (box(tuple((lo + hi) / 2), tuple(np.maximum((hi - lo) * proxy_scale, 0.3 * MM))),)
    return _oriented(name, points, faces, color, proxy)


def _prism_y(name, xz, y0, y1, color):
    """As _prism_x, with the polygon in (x, z) extruded along y."""
    k = len(xz)
    points = [(x, y0, z) for x, z in xz] + [(x, y1, z) for x, z in xz]
    faces = []
    for j in range(k):
        a, b = j, (j + 1) % k
        faces += [(a, b, b + k), (a, b + k, a + k)]
    faces += [(0, j + 1, j) for j in range(1, k - 1)]
    faces += [(k, k + j, k + j + 1) for j in range(1, k - 1)]
    p = np.asarray(points, float)
    lo, hi = p.min(axis=0), p.max(axis=0)
    proxy = (box(tuple((lo + hi) / 2), tuple(np.maximum(hi - lo, 0.3 * MM))),)
    return _oriented(name, points, faces, color, proxy)


def _box(x0, x1, y0, y1, z0, z1, color=None):
    """An axis-aligned box from its bounds (metres)."""
    return box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), color=color)


# ---------------------------------------------------------------- the travel adapter (reusable: copy this section)
# A UNIVERSAL TRAVEL ADAPTER WITH A SLIDE-OUT UK PIN SET AND ITS CLICK LOCK (EPICKA TA-105).
# Frame A: origin at the centre of the pin face (the bottom), X along its 71.1 mm length (the toggles near +X), Y
# across its 50.0 mm depth (the toggle side is +Y), Z up its 52.1 mm height (universal socket on top).
# Body: the base link, rigid (172 g). UK pin carrier: link "uk", a Slide along -Z of UK_TRAVEL (22.8 mm): line and
# neutral pins 6.4 x 4.0 mm, 17.7 mm out when locked; earth pin 4.0 x 8.0 mm, 22.3 mm out (BS 1363 sizes); the middle
# toggle's tab (7.5 wide, 3.5 proud, 9 tall) rides on the carrier outside the +Y face; joint stiction 2.0 N static /
# 1.5 N sliding (estimate: holds the pins wherever they are pushed, as the real slider does). The lock: link "lock",
# a sprung catch on a 2.5 mm Slide along X inside the body, pressed against a rail on the carrier (0.4 N of drag) and
# dropping over the rail's top in the last 0.8 mm of travel; then the rail's top bears on the catch and the pins cannot
# be pushed back (the release button that pulls the catch is not modelled: it is only used to put the pins away).
# Catch and rail are sibling links, so the mechanism has self-collision on. Tuned numbers: catch spring 300 N/m with
# 4.5 mm of preload past its inner stop; carrier armature 0.02 kg; catch armature 0.005 kg.
ADAPTER_L, ADAPTER_W, ADAPTER_H = 71.1 * MM, 50.0 * MM, 52.1 * MM
ADAPTER_MASS = 0.190
UK_TRAVEL = 22.8 * MM
LN_LEN, E_LEN = 17.7 * MM, 22.3 * MM
LN_X, LN_Y, E_Y = 11.1 * MM, -12.1 * MM, 10.1 * MM  # pin centres on the pin face (estimate of where the set sits)
LN_SIZE = (6.4 * MM, 4.0 * MM)
E_SIZE = (4.0 * MM, 8.0 * MM)
TOGGLE_XS = (7.5 * MM, 18.5 * MM, 29.5 * MM)  # US/AU, UK (middle), EU toggles along the +Y side (estimates)
UK_TOGGLE_X = TOGGLE_XS[1]
TAB_W, TAB_OUT, TAB_H = 7.5 * MM, 3.5 * MM, 9.0 * MM
TAB_TOP0 = 48.5 * MM  # the UK tab's top when the pins are in
LOCK_IN = 2.5 * MM
LOCK_AT = 22.0 * MM  # the catch drops in once the carrier has gone this far
UK_STICTION = (2.0, 1.5)
CATCH_K, CATCH_PRE = 300.0, 4.5 * MM
CARRIER_MASS, CATCH_MASS = 0.015, 0.002
ADAPTER_FRICTION = (0.6, 0.5)  # ABS case (estimate)
PIN_FRICTION = (0.3, 0.25)  # nickel-plated brass pins (estimate)
_RAIL = (-27.0 * MM, -23.0 * MM, -9.0 * MM, 3.0 * MM)  # rail x0, x1, y0, y1 on the carrier
_CATCH_Z = (25.0 * MM, 28.0 * MM)


def adapter_shapes():
    """(base shapes, carrier shapes, catch shapes) in frame A at q = 0 (pins in)."""
    L, W, H = ADAPTER_L / 2, ADAPTER_W / 2, ADAPTER_H
    base = [_box(-L, L, -W, W, 0.0, 46.0 * MM),
            _box(-L + 1.0 * MM, L - 1.0 * MM, -W + 1.0 * MM, W - 1.0 * MM, 46.0 * MM, H)]
    # universal socket (look only): its openings as dark marks on the top face
    for x, y, sx, sy in ((-12, 6, 7, 3), (-12, -6, 7, 3), (3, 0, 9, 9), (14, 6, 3, 7), (14, -6, 3, 7), (24, 0, 5, 9)):
        base.append(box((x * MM, y * MM, H + 0.15 * MM), (sx * MM, sy * MM, 0.3 * MM), color=MARK_DARK))
    # USB ports on the -X end (look only)
    for k, (y, z, sy, sz) in enumerate(((-13.0, 34.0, 12.0, 4.5), (-13.0, 24.0, 12.0, 4.5), (5.0, 34.0, 8.4, 2.6),
                                        (5.0, 24.0, 8.4, 2.6), (15.0, 29.0, 8.4, 2.6))):
        base.append(box((-L - 0.15 * MM, y * MM, z * MM), (0.3 * MM, sy * MM, sz * MM), color=MARK_DARK))
    # the three toggle slots on the +Y side and the two toggles that stay in (US/AU and EU)
    for x in TOGGLE_XS:
        base.append(_box(x - 1.5 * MM, x + 1.5 * MM, W, W + 0.15 * MM, 12.0 * MM, 50.0 * MM, color=MARK_DARK))
    for x in (TOGGLE_XS[0], TOGGLE_XS[2]):
        base.append(_box(x - TAB_W / 2, x + TAB_W / 2, W, W + TAB_OUT, TAB_TOP0 - TAB_H, TAB_TOP0, color=TOGGLE_GREY))
    # the other pin sets' slots on the pin face (look only)
    for x, y, sx, sy in ((-22, 0, 2, 7), (-14, 14, 2, 7), (25, -8, 5, 5), (25, 8, 5, 5)):
        base.append(box((x * MM, y * MM, -0.15 * MM), (sx * MM, sy * MM, 0.3 * MM), color=MARK_DARK))
    z_in = 0.5 * MM  # the earth pin's tip sits this far inside the pin face when the pins are in
    top = z_in + E_LEN  # the pins' roots (in the carrier) at q = 0
    carrier = [
        _box(-LN_X - LN_SIZE[0] / 2, -LN_X + LN_SIZE[0] / 2, LN_Y - LN_SIZE[1] / 2, LN_Y + LN_SIZE[1] / 2,
             top - LN_LEN, top, color=PIN_NICKEL),
        _box(LN_X - LN_SIZE[0] / 2, LN_X + LN_SIZE[0] / 2, LN_Y - LN_SIZE[1] / 2, LN_Y + LN_SIZE[1] / 2,
             top - LN_LEN, top, color=PIN_NICKEL),
        _box(-E_SIZE[0] / 2, E_SIZE[0] / 2, E_Y - E_SIZE[1] / 2, E_Y + E_SIZE[1] / 2, z_in, top, color=PIN_NICKEL),
        _box(-14.0 * MM, 22.0 * MM, -15.0 * MM, W, top, top + 3.2 * MM, color=SLEEVE_BLACK),  # pin holder (inside)
        _box(UK_TOGGLE_X - 3.5 * MM, UK_TOGGLE_X + 3.5 * MM, W - 4.0 * MM, W, top + 3.2 * MM, TAB_TOP0 - TAB_H,
             color=TOGGLE_GREY),  # stem up to the tab (inside)
        _box(UK_TOGGLE_X - TAB_W / 2, UK_TOGGLE_X + TAB_W / 2, W, W + TAB_OUT, TAB_TOP0 - TAB_H, TAB_TOP0,
             color=TOGGLE_GREY),  # the UK toggle's tab
        _box(_RAIL[0], -14.0 * MM, -9.0 * MM, -4.0 * MM, top, top + 3.2 * MM, color=SLEEVE_BLACK),  # rail arm
        _box(_RAIL[0], _RAIL[1], _RAIL[2], _RAIL[3], top + 1.2 * MM, _CATCH_Z[0] + LOCK_AT, color=SLEEVE_BLACK),
    ]
    catch = [_box(_RAIL[1] + 0.2 * MM, _RAIL[1] + 4.2 * MM, -2.5 * MM, 3.0 * MM, _CATCH_Z[0], _CATCH_Z[1],
                  color=SLEEVE_BLACK),
             _box(_RAIL[1] + 4.2 * MM, _RAIL[1] + 7.0 * MM, -2.5 * MM, 3.0 * MM, _CATCH_Z[0], _CATCH_Z[1],
                  color=SLEEVE_BLACK)]
    return base, carrier, catch


def make_adapter(pos, yaw):
    base, carrier, catch = adapter_shapes()
    top = 0.5 * MM + E_LEN
    links = [
        Link("uk", carrier, Slide("z", (0.0, 0.0, top), -UK_TRAVEL, 0.0, damping=2.0, armature=0.02,
                                  static_friction=UK_STICTION[0], dynamic_friction=UK_STICTION[1]),
             CARRIER_MASS, color=TOGGLE_GREY, friction=PIN_FRICTION),
        Link("lock", catch, Slide("x", (_RAIL[1] + 3.5 * MM, 0.0, sum(_CATCH_Z) / 2), -LOCK_IN, 0.5 * MM,
                                  damping=1.0, spring=CATCH_K, rest=-CATCH_PRE, armature=0.005),
             CATCH_MASS, color=SLEEVE_BLACK, friction=(0.3, 0.25)),
    ]
    return Mechanism("adapter", pos, base, links, mass=ADAPTER_MASS - CARRIER_MASS - CATCH_MASS, color=ADAPTER_GREY,
                     yaw=yaw, friction=ADAPTER_FRICTION, contact_offset=0.0005, self_collide=True, grip=0)


# the pins' tips in frame A at carrier position q (metres, q <= 0 is out)
def pin_tips(q):
    z_in = 0.5 * MM
    return {"L": np.array([-LN_X, LN_Y, z_in + (E_LEN - LN_LEN) + q]),
            "N": np.array([LN_X, LN_Y, z_in + (E_LEN - LN_LEN) + q]),
            "E": np.array([0.0, E_Y, z_in + q])}


# ---------------------------------------------------------------- the socket (reusable: copy this section)
# A BS 1363 SOCKET FACE ON A SINGLE-SOCKET EXTENSION LEAD (Masterplug EXS1315B-01, 90 x 60 x 33 mm, black).
# Frame S: origin at the centre of its underside, X along its 90 mm length, Z up; the socket face is the top (z = 33
# mm). Openings line up with frame A's pins when the adapter sits centred and square on the face. Openings 0.75 mm
# wider than the pins each side, 1.0 mm 45-degree lead-in round each (estimates). Line and neutral each have a
# contact plate (links "c_l", "c_n"): a 2.5 mm plate on a 3 mm Slide along Y on the +Y side of the opening, sprung
# toward the pin (2000 N/m, 2.0 mm preload: 4 N at rest, ~5.5 N on a pin), its pin-side face bevelled 20 degrees from
# upright over its top 6.9 mm; the pin is pushed onto the opening's rigid -Y wall. Pins slide on brass and plastic at
# 0.3/0.25: about 6 N to start the pins past the bevels, ~5.5 N to slide home (estimates). The shutters (opened by the
# earth pin first) are not modelled; the earth opening is a plain slot.
SOCKET_L, SOCKET_W, SOCKET_H = 90.0 * MM, 60.0 * MM, 33.0 * MM
FACE_T = 3.0 * MM
CLEAR = 0.75 * MM
LEAD = 1.0 * MM
CONTACT_K, CONTACT_PRE, CONTACT_TRAVEL = 2000.0, 2.0 * MM, 3.0 * MM
CONTACT_T, CONTACT_REST = 2.5 * MM, 0.5 * MM  # plate thickness; rest face this far +Y of the opening's centre line
CONTACT_Z = (12.5 * MM, SOCKET_H - FACE_T - 0.5 * MM)
CONTACT_BEVEL = 6.9 * MM
SOCKET_FRICTION = (0.3, 0.25)


def _openings():
    """{pin: (x0, x1, y0, y1)} of the face openings (frame S)."""
    out = {}
    for name, (x, y, (sx, sy)) in {"L": (-LN_X, LN_Y, LN_SIZE), "N": (LN_X, LN_Y, LN_SIZE),
                                   "E": (0.0, E_Y, E_SIZE)}.items():
        out[name] = (x - sx / 2 - CLEAR, x + sx / 2 + CLEAR, y - sy / 2 - CLEAR, y + sy / 2 + CLEAR)
    return out


def _solid_with_pockets(x0, x1, y0, y1, z0, z1, pockets, color):
    """Boxes filling the block [x0,x1]x[y0,y1]x[z0,z1] minus axis-aligned pockets (x0,x1,y0,y1,z0,z1)."""
    xs = sorted({x0, x1, *[p[0] for p in pockets], *[p[1] for p in pockets]})
    ys = sorted({y0, y1, *[p[2] for p in pockets], *[p[3] for p in pockets]})
    out = []
    for j in range(len(ys) - 1):
        cy0, cy1 = ys[j], ys[j + 1]
        row = []
        for i in range(len(xs) - 1):
            cx0, cx1 = xs[i], xs[i + 1]
            cx, cy = (cx0 + cx1) / 2, (cy0 + cy1) / 2
            empty = sorted((p[4], p[5]) for p in pockets if p[0] < cx < p[1] and p[2] < cy < p[3])
            solid, z = [], z0
            for e0, e1 in empty:
                if e0 > z + 1e-9:
                    solid.append((z, min(e0, z1)))
                z = max(z, e1)
            if z < z1 - 1e-9:
                solid.append((z, z1))
            row.append((cx0, cx1, tuple((round(a, 7), round(b, 7)) for a, b in solid)))
        merged = []
        for cx0, cx1, solid in row:
            if merged and merged[-1][2] == solid and abs(merged[-1][1] - cx0) < 1e-9:
                merged[-1] = (merged[-1][0], cx1, solid)
            else:
                merged.append((cx0, cx1, solid))
        for cx0, cx1, solid in merged:
            for a, b in solid:
                if b - a > 1e-6:
                    out.append(_box(cx0, cx1, cy0, cy1, a, b, color=color))
    return out


def socket_parts():
    """(base boxes, base meshes, {link: (meshes, slide)}) of the socket in frame S."""
    L, W, H = SOCKET_L / 2, SOCKET_W / 2, SOCKET_H
    ops = _openings()
    pockets = []
    for name, (x0, x1, y0, y1) in ops.items():
        floor = H - (E_LEN if name == "E" else LN_LEN) - 3.0 * MM
        pockets.append((x0, x1, y0, y1, floor, H + 1.0))  # the opening, down to its floor
        pockets.append((x0 - LEAD, x1 + LEAD, y0 - LEAD, y1 + LEAD, H - LEAD, H + 1.0))  # room for the lead-in
        if name != "E":  # room under the face for the contact plate to move into
            pockets.append((x0, x1, y0, y1 + CONTACT_REST + CONTACT_T + CONTACT_TRAVEL + 1.0 * MM - (y1 - y0) / 2,
                            floor, H - FACE_T))
    base = _solid_with_pockets(-L, L, -W, W, 0.0, H, pockets, SOCKET_BLACK)
    base.append(_box(-L - 120.0 * MM, -L, -4.0 * MM, 4.0 * MM, 0.5 * MM, 8.5 * MM, color=CABLE_BLACK))  # lead stub
    meshes = []
    for name, (x0, x1, y0, y1) in ops.items():  # 45-degree lead-in round each opening
        zl, zh = H - LEAD, H
        meshes.append(_prism_x(f"lead_{name}_s", [(y0, zl), (y0 - LEAD, zl), (y0 - LEAD, zh)], x0 - LEAD, x1 + LEAD,
                               FACE_BLACK))
        meshes.append(_prism_x(f"lead_{name}_n", [(y1 + LEAD, zl), (y1, zl), (y1 + LEAD, zh)], x0 - LEAD, x1 + LEAD,
                               FACE_BLACK))
        meshes.append(_prism_y(f"lead_{name}_w", [(x0, zl), (x0 - LEAD, zl), (x0 - LEAD, zh)], y0 - LEAD, y1 + LEAD,
                               FACE_BLACK))
        meshes.append(_prism_y(f"lead_{name}_e", [(x1 + LEAD, zl), (x1, zl), (x1 + LEAD, zh)], y0 - LEAD, y1 + LEAD,
                               FACE_BLACK))
    contacts = {}
    for name in ("L", "N"):
        x0, x1, y0, y1 = ops[name]
        yc = (y0 + y1) / 2
        f = yc + CONTACT_REST  # the plate's pin-side face at rest
        z0, z1 = CONTACT_Z
        plate = _prism_x(f"contact_{name}", [(f, z0), (f + CONTACT_T, z0), (f + CONTACT_T, z1), (f, z1 - CONTACT_BEVEL)],
                         x0 + 0.45 * MM, x1 - 0.45 * MM, CONTACT_BRASS, proxy_scale=0.6)  # bevel 20 deg from upright
        slide = Slide("y", ((x0 + x1) / 2, f + CONTACT_T / 2, (z0 + z1) / 2), 0.0, CONTACT_TRAVEL, damping=5.0,
                      spring=CONTACT_K, rest=-CONTACT_PRE, armature=0.02)
        contacts["c_" + name.lower()] = ((plate,), slide)
    return base, meshes, contacts


def make_socket(pos, yaw):
    base, meshes, contacts = socket_parts()
    links = [Link(k, [], slide, 0.002, color=CONTACT_BRASS, friction=SOCKET_FRICTION) for k, (_, slide) in
             contacts.items()]
    return MeshMechanism("socket", pos, base, links, mass=None, color=SOCKET_BLACK, yaw=yaw, friction=SOCKET_FRICTION,
                         contact_offset=0.0005, meshes={"base": tuple(meshes),
                                                        **{k: m for k, (m, _) in contacts.items()}})


# ---------------------------------------------------------------- layout
YAW = 90.0 * DEG  # frame A's X (and S's X) along world +y; the toggle side (+Y) faces the robot (-x)
ADAPTER_XY = (0.200, -0.140)  # the adapter's pin-face centre at the start
HOLD = np.array([0.220, -0.040, TOP + 0.090])  # where the right hand holds it while the left pushes the toggle
SOCKET_XY = (0.280, -0.115)
ADAPTER = make_adapter((*ADAPTER_XY, TOP + 0.3 * MM), YAW)
SOCKET = make_socket((*SOCKET_XY, TOP), YAW)
OBJECTS = [SOCKET, ADAPTER]
PRESSED_PARTS = ("socket", "adapter")  # pins pushed into the socket; the closed fingertips press the adapter home


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _T(R, p):
    m = np.eye(4)
    m[:3, :3], m[:3, 3] = R, p
    return m


def _inv(m):
    R, p = m[:3, :3], m[:3, 3]
    return _T(R.T, -R.T @ p)


def frame_of(state):
    """4x4 world pose of a part's frame from its state."""
    return _T(state.rotation(), np.asarray(state.origin, float))


# ---------------------------------------------------------------- scoring
LOCKED_Q = -(UK_TRAVEL - 1.0 * MM)  # carrier this far out (the catch engages in the last 0.8 mm)
FLUSH, SHALLOW = 1.5 * MM, 6.0 * MM
IN_DEPTH = 1.0 * MM  # a pin is in its opening once its tip is this far below the socket face
OFF_TABLE = 0.020
STILL_MOVE, STILL_DEG = 1.0 * MM, 2.0
_WINDOW = {"episode": None, "hist": []}


def adapter_terms(adapter, socket):
    a, s = frame_of(adapter), frame_of(socket)
    to_s = _inv(s) @ a
    q = float(adapter.joints["uk"])
    catch = float(adapter.joints["lock"])
    locked = bool(q <= LOCKED_Q and catch <= -LOCK_IN / 2)
    ops = _openings()
    in_open = {}
    for name, tip in pin_tips(q).items():
        p = to_s[:3, :3] @ tip + to_s[:3, 3]
        x0, x1, y0, y1 = ops[name]
        in_open[name] = bool(x0 - 0.3 * MM <= p[0] <= x1 + 0.3 * MM and y0 - 0.3 * MM <= p[1] <= y1 + 0.3 * MM
                             and p[2] <= SOCKET_H - IN_DEPTH)
    corners = [np.array([sx * ADAPTER_L / 2, sy * ADAPTER_W / 2, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    gap = max(float((to_s[:3, :3] @ c + to_s[:3, 3])[2]) - SOCKET_H for c in corners)
    low = float(np.asarray(adapter.low, float)[2])
    return {"q": q, "catch": catch, "locked": locked, "in": in_open, "all_in": all(in_open.values()), "gap": gap,
            "off_table": low < TOP - OFF_TABLE, "tilt": adapter.tilt_deg()}


def _box_clear(state, clear):
    hands = grippers()
    if hands is None:
        return True
    low, high = np.asarray(state.low, float), np.asarray(state.high, float)
    return all(float(np.linalg.norm(np.maximum(np.maximum(low - tcp, tcp - high), 0.0))) >= clear
               for tcp, _ in hands.values())


def _rot_deg(a, b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1.0, 1.0))))


def still(adapter):
    """Over the last second the adapter moved under 1 mm and turned under 2 degrees (offline: its velocity)."""
    t = _RUN["t"]
    if t is None:
        return bool(np.linalg.norm(adapter.lin_vel) < 0.01)
    hist = _WINDOW["hist"]
    if _WINDOW["episode"] != _RUN["episode"] or (hist and t < hist[-1][0]):
        hist.clear()
        _WINDOW["episode"] = _RUN["episode"]
    now = (np.asarray(adapter.origin, float), adapter.rotation())
    if not hist or t > hist[-1][0] + 1e-9:
        hist.append((t, now))
    while len(hist) > 1 and hist[1][0] <= t - STILL_S + 1e-9:
        hist.pop(0)
    if t - hist[0][0] < STILL_S - 0.051:
        return False
    return all(np.linalg.norm(now[0] - old[0]) <= STILL_MOVE and _rot_deg(now[1], old[1]) <= STILL_DEG
               for _, old in hist)


def score_terms(objects):
    adapter, socket = objects["adapter"], objects["socket"]
    t = adapter_terms(adapter, socket)
    t["fell"] = ever("adapter off the table", t["off_table"])
    t["released"] = _box_clear(adapter, RELEASE_CLEAR)
    t["still"] = still(adapter)
    if DIAG:
        _diag(adapter, socket, t)
    return t


def score(objects):
    t = score_terms(objects)
    if not t["locked"] or t["fell"] or not (t["released"] and t["still"]):
        return 0.0
    if not t["all_in"]:
        return 0.25
    if t["gap"] <= FLUSH:
        return 1.0
    if t["gap"] <= SHALLOW:
        return 0.75
    return 0.5


def success(objects):
    return score(objects) >= 1.0


# ---------------------------------------------------------------- diagnostics (GPU copies only)
_DIAG = {"step": -1}


def _diag(adapter, socket, t):
    step = _RUN["step"]
    if _RUN["t"] is None or step == _DIAG["step"] or step % 5:
        return
    _DIAG["step"] = step
    o = np.asarray(adapter.origin, float)
    hands = grippers() or {}
    tcp = " ".join(f"{a[0]}=({','.join(f'{v * 1000:.1f}' for v in hands[a][0])};{hands[a][1]:.2f})" for a in hands)
    cj = ",".join(f"{float(socket.joints[k]) * 1000:.2f}" for k in ("c_l", "c_n"))
    ins = "".join(k for k, v in t["in"].items() if v)
    print(f"PLUG_DIAG t={_RUN['t']:.2f} uk={t['q'] * 1000:.2f} catch={t['catch'] * 1000:.2f} lock={int(t['locked'])} "
          f"in={ins or '-'} gap={t['gap'] * 1000:.1f} tilt={t['tilt']:.1f} contacts=({cj}) "
          f"o=({o[0] * 1000:.1f},{o[1] * 1000:.1f},{(o[2] - TOP) * 1000:.1f}) v={np.linalg.norm(adapter.lin_vel) * 1000:.0f} "
          f"free={int(t['released'])} still={int(t['still'])} {tcp} F={_hand_forces()}", flush=True)


def _hand_forces():
    """Each hand's force on what it touches (N, world x, y, z) from the joints' PD error. Diagnostic only."""
    env = _RUN["env"]
    if env is None:
        return "-"
    try:
        from yam_arena import ik, station
        from yam_arena.constants import ARM_JOINTS, SHOULDER, WRIST
        k = np.array([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
        out = []
        for arm in ("left", "right"):
            art = env.unwrapped.scene[arm + "_robot"]
            ids = art.find_joints(ARM_JOINTS, preserve_order=True)[0]
            q = art.data.joint_pos[0][ids].detach().cpu().numpy().astype(float)
            qt = art.data.joint_pos_target[0][ids].detach().cpu().numpy().astype(float)
            tau = k * (qt - q)
            p0, r0 = ik.tcp_pose(q, station.BASES[arm])
            jac = np.zeros((6, 6))
            for j in range(6):
                dq = q.copy()
                dq[j] += 1e-5
                p1, r1 = ik.tcp_pose(dq, station.BASES[arm])
                ax, a = _axis_angle(r1 @ r0.T)
                jac[:3, j] = (p1 - p0) / 1e-5
                jac[3:, j] = ax * a / 1e-5
            f = np.linalg.lstsq(jac.T, tau, rcond=None)[0]
            out.append(f"{arm[0]}({f[0]:.1f},{f[1]:.1f},{f[2]:.1f})")
        return " ".join(out)
    except Exception as e:  # noqa: BLE001
        return f"err:{type(e).__name__}"


def _axis_angle(rot):
    c = (np.trace(rot) - 1.0) / 2.0
    a = math.acos(float(np.clip(c, -1.0, 1.0)))
    if a < 1e-9:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]]) / (2 * math.sin(a))
    return axis, a


def _limit_rotation(rot, max_deg):
    axis, ang = _axis_angle(rot)
    if ang <= math.radians(max_deg):
        return rot
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    a = math.radians(max_deg)
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


# ---------------------------------------------------------------- demo
# Right hand: comes down over the standing adapter, pinches it by its two ends (pads 12 mm toward the toggle side, so
# the toggle push twists the pinch little), lifts it to the hold point and keeps still. Left hand: gripper open,
# pitched 55 degrees down and pointing forward, puts its right-hand fingertip on the UK toggle's tab, pushes the tab
# down the slot until the carrier locks (1 mm past the catch point), lifts off and parks. Right hand: carries the
# adapter over the socket, lines the earth pin's tip up over its opening (reading the adapter and socket poses),
# lowers until all three pins are in their openings, lets go, closes, and presses the adapter's top down past flush
# (the arm's spring gives the push), eases off, lifts away and parks. The demo reads simulator poses; it never moves
# an object.
PAD_TIP = 0.0128  # pad tips past the TCP
RIGHT_PAD_Z = 34.0 * MM  # TCP height on the adapter (frame A): pad tips 21 mm above its base
RIGHT_PAD_Y = -4.0 * MM  # pads this far toward the toggle side (negative: away; keeps the two hands apart)
PINCH = -(ADAPTER_L / 2 - 3.0 * MM)  # jaws 3 mm inside each end face (~15 N each: the toggle push twists the pinch)
OPEN_WIDE = FINGER_OPEN
LEFT_PITCH = 50.0 * DEG
LEFT_CLOSE = -1.0  # the left hand's closing axis (TCP x) is LEFT_CLOSE * world y (-1 keeps its wrist roll in range)
LEFT_FINGER = np.array([-LEFT_CLOSE * 0.050, 0.0, PAD_TIP])  # its -y fingertip's centre (TCP frame, gripper open)
LEFT_EDGE_IN = 1.0 * MM  # the fingertip's face-side edge aims this far into the toggle side (a light side preload)
PRESS_LEAD = 0.015  # the press aims this far below flush
PARK = {"left": np.array([0.200, 0.250, TOP + 0.200]), "right": np.array([0.150, -0.250, TOP + 0.200])}


def _grip_down():
    from yam_arena.demo import grip_rotation
    return getattr(grip_rotation, "__wrapped__", grip_rotation)(0.0)  # top-down, closing along world y


def _left_rot():
    """Pitched LEFT_PITCH below horizontal, pointing +x, closing along world y."""
    a = np.array([math.cos(LEFT_PITCH), 0.0, -math.sin(LEFT_PITCH)])
    x = np.array([0.0, LEFT_CLOSE, 0.0])
    return np.column_stack([x, np.cross(a, x), a])


R_A0 = _rz(YAW)


def adapter_pose(p):
    """World pose of frame A, upright at yaw YAW, origin p."""
    return _T(R_A0, np.asarray(p, float))


A_START = adapter_pose((*ADAPTER_XY, TOP + 0.3 * MM))
A_HOLD = adapter_pose(HOLD)
HAND_R_IN_A = _inv(A_START) @ _T(_grip_down(), A_START[:3, :3] @ np.array([0.0, RIGHT_PAD_Y, RIGHT_PAD_Z])
                                 + A_START[:3, 3])
S_POSE = _T(R_A0, np.array([*SOCKET_XY, TOP]))


def a_over_socket(dz, dxy=(0.0, 0.0)):
    """Frame A square over the socket, its pin face dz above the socket face."""
    return S_POSE @ _T(np.eye(3), np.array([dxy[0], dxy[1], SOCKET_H + dz]))


def left_hand_in_a(tab_dz, out=0.0):
    """Left hand pose in frame A: its fingertip's face-side edge on the tab's top (tab_dz above the tab's start top,
    negative = pushed down), `out` metres off the toggle side."""
    R = R_A0.T @ _left_rot()  # left hand rotation in frame A
    s = LEFT_CLOSE * 7.7 * MM  # the fingertip's face-side (+x) edge: TCP y = +7.7 mm (-7.7 when LEFT_CLOSE < 0)
    edge_tcp = LEFT_FINGER + np.array([0.0, s, 0.0])
    target = np.array([UK_TOGGLE_X, ADAPTER_W / 2 + out - LEFT_EDGE_IN * (out == 0.0), TAB_TOP0 + tab_dz])
    # the tip's end face slopes down away from the toggle side: lift the edge so the face meets the tab's outer corner
    drop = (TAB_OUT + LEFT_EDGE_IN) * math.tan(math.pi / 2 - LEFT_PITCH)
    target[2] += drop
    p = target - R @ edge_tcp
    return _T(R, p)


# ---- reads: compare a planned adapter pose with the simulator's and correct the plan
READ_MAX_SHIFT, READ_MAX_DEG = 0.025, 12.0
_READS = {"episode": None, "values": {}}


def _state_now(name):
    env = _RUN["env"]
    if env is None:
        return None
    try:
        from yam_arena import scene as arena_scene
        return arena_scene.object_states(env, [name])[name]
    except Exception as e:  # noqa: BLE001
        print(f"PLUG_READ {name} failed: {e!r}", flush=True)
        return None


def _values():
    if _READS["episode"] != _RUN["episode"]:
        _READS["episode"], _READS["values"] = _RUN["episode"], {}
    return _READS["values"]


def _clamped(m):
    rot = _limit_rotation(m[:3, :3], READ_MAX_DEG)
    t = m[:3, 3].copy()
    n = float(np.linalg.norm(t))
    if n > READ_MAX_SHIFT:
        t *= READ_MAX_SHIFT / n
    return _T(rot, t)


def _socket_shift():
    """World transform from the socket's planned pose to where it is (identity offline; it is fixed)."""
    v = _values()
    if "socket" not in v:
        ss = _state_now("socket")
        v["socket"] = np.eye(4) if ss is None else _clamped(frame_of(ss) @ _inv(S_POSE))
    return v["socket"]


def read_follow(key, planned, ref):
    """World transform D (cached per episode): hand = D @ planned pose @ hand-in-A puts the hand where the adapter
    actually is (turned about `ref`, a point in frame A). Used before a grasp and for the press."""
    v = _values()
    if key in v:
        return v[key]
    st = _state_now("adapter")
    if st is None:
        v[key] = np.eye(4)
        return v[key]
    actual = frame_of(st)
    rot = _limit_rotation(actual[:3, :3] @ planned[:3, :3].T, READ_MAX_DEG)
    src, dst = planned[:3, :3] @ ref + planned[:3, 3], actual[:3, :3] @ ref + actual[:3, 3]
    move = dst - src
    n = float(np.linalg.norm(move))
    if n > READ_MAX_SHIFT:
        move *= READ_MAX_SHIFT / n
    v[key] = _T(rot, src + move - rot @ src)
    print(f"PLUG_READ {key} t={_RUN['t']:.2f} ref_err_mm={tuple(round(float(x) * 1000, 2) for x in src - dst)} "
          f"rot_err_deg={math.degrees(_axis_angle(actual[:3, :3] @ planned[:3, :3].T)[1]):.2f}", flush=True)
    return v[key]


def read_move(key, prev, planned_prev, on_socket):
    """Body correction C (cached): hand = P @ C @ hand-in-A brings the HELD adapter to P. From the hand's command at
    the previous waypoint (planned_prev @ C_prev @ hand-in-A) and where the adapter actually is:
    C = inv(actual) @ planned_prev @ C_prev (any slip in the grip and the arm's sag are taken out)."""
    v = _values()
    if key in v:
        return v[key]
    c_prev = v.get(prev, np.eye(4)) if prev else np.eye(4)
    st = _state_now("adapter")
    if st is None:
        v[key] = c_prev
        return c_prev
    pp = (_socket_shift() @ planned_prev) if on_socket else planned_prev
    actual = frame_of(st)
    c = _inv(actual) @ pp @ c_prev
    v[key] = _clamped(c @ _inv(c_prev)) @ c_prev
    tip_p, tip_a = pp[:3, :3] @ E_TIP_REF + pp[:3, 3], actual[:3, :3] @ E_TIP_REF + actual[:3, 3]
    print(f"PLUG_READ {key} t={_RUN['t']:.2f} tip_err_mm={tuple(round(float(x) * 1000, 2) for x in tip_p - tip_a)} "
          f"rot_err_deg={math.degrees(_axis_angle(actual[:3, :3] @ pp[:3, :3].T)[1]):.2f}", flush=True)
    return v[key]


class Grip(float):
    def __new__(cls, yaw, matrix=None):
        value = super().__new__(cls, yaw)
        value.grip_matrix = None if matrix is None else np.array(matrix, float)
        return value


def _install_grip_matrix():
    from yam_arena import demo as arena_demo
    original = arena_demo.grip_rotation
    if getattr(original, "_grip_matrix", False):
        return

    def grip_rotation(yaw=0.0):
        matrix = getattr(yaw, "grip_matrix", None)
        return matrix.copy() if matrix is not None else original(float(yaw))
    grip_rotation._grip_matrix = True
    grip_rotation.__wrapped__ = original
    arena_demo.grip_rotation = grip_rotation


_install_grip_matrix()


class PlanWaypoint(Waypoint):
    """A waypoint worked out when played. read: None (hand = P @ hand-in-A), ("follow", key, planned, ref) (hand =
    D @ P @ hand-in-A) or ("move", key, prev key, planned previous pose, on_socket) (hand = P' @ C @ hand-in-A, P'
    shifted with the socket). A read happens when its first waypoint starts; later waypoints reuse it."""

    def __init__(self, arm, duration, a_pose, hand_in_a, finger, label, read=None, offset=(0.0, 0.0, 0.0),
                 keep=False):
        self.duration = duration * (1.0 if keep else DEMO_PACE)
        self.arm, self.a_pose, self.hand_in_a, self.finger, self.label = arm, a_pose, hand_in_a, finger, label
        self.read = read
        self.offset = np.asarray(offset, float)

    def _hand(self):
        if self.read is None:
            h = self.a_pose @ self.hand_in_a
        elif self.read[0] == "follow":
            h = read_follow(*self.read[1:]) @ self.a_pose @ self.hand_in_a
        else:
            _, key, prev, pp, on_socket = self.read
            c = read_move(key, prev, pp, on_socket)
            p = (_socket_shift() @ self.a_pose) if on_socket else self.a_pose
            h = p @ c @ self.hand_in_a
        h = h.copy()
        h[:3, 3] += self.offset
        return h

    @property
    def pos(self):
        return tuple(float(v) for v in self._hand()[:3, 3])

    @property
    def yaw(self):
        return Grip(0.0, self._hand()[:3, :3])


PUSH_STEP = 3.5 * MM  # each push step asks the carrier for this much more travel ...
PUSH_LEAD = 9.0 * MM  # ... with the fingertip aimed this far past the tab: the two arms give ~400 N/m together, and
# the carrier needs ~2.3 N to slide, so a 4 mm lead (probe 1) crept 1.3 mm a step; this lead keeps ~3.5 N at the end
PUSH_STEPS, PUSH_STEP_S = 10, 0.55


class TogglePush(PlanWaypoint):
    """One step of the left fingertip's push on the UK tab, worked out when it starts from where the adapter and the
    pin carrier are then (the holding arm sags under the push): the tab is asked to go PUSH_STEP further, never past
    the end of its slot (or past DEMO_TOGGLE of its travel), and the fingertip aims PUSH_LEAD below that."""

    def __init__(self, k, duration, label):
        super().__init__("left", duration, A_HOLD, np.eye(4), OPEN_WIDE, label)
        self.k = k

    def _hand(self):
        v = _values()
        key = f"push{self.k}"
        limit = -UK_TRAVEL * min(DEMO_TOGGLE, 1.0)
        if key not in v:
            st = _state_now("adapter")
            if st is None:  # offline: the planned schedule
                q_des = max(limit, -PUSH_STEP * (self.k + 1))
                v[key] = (A_HOLD, q_des)
            else:
                q = float(st.joints["uk"])
                q_des = max(limit, q - PUSH_STEP)
                v[key] = (frame_of(st), q_des)
                print(f"PLUG_PUSH {self.k} t={_RUN['t']:.2f} uk={q * 1000:.2f} aim={q_des * 1000:.2f}", flush=True)
        pose, q_des = v[key]
        # at the end of the slot the full lead stays on (the slot end takes it); the half-way copy eases off there
        lead = PUSH_LEAD if DEMO_TOGGLE >= 1.0 or q_des > limit + 0.2 * MM else 0.5 * MM
        return pose @ left_hand_in_a(q_des - lead, 0.0)


E_TIP_REF = pin_tips(-UK_TRAVEL)["E"]  # the earth pin's tip (locked out) in frame A
CENTRE_REF = np.array([0.0, 0.0, ADAPTER_H / 2])


def right_plan():
    """Two lists of (label, seconds, adapter pose, hand-in-A, finger, read, offset, keep) for the right hand: up to
    the hold point, and from the hold to letting go in the socket."""
    hr = HAND_R_IN_A
    up = lambda pose, dz: _T(pose[:3, :3], pose[:3, 3] + np.array([0.0, 0.0, dz]))  # noqa: E731
    g = ("follow", "grasp", A_START, CENTRE_REF)
    carry_hi = a_over_socket(0.060)
    over = a_over_socket(E_LEN + 0.012)
    touch = a_over_socket(E_LEN + 0.002)
    entered = a_over_socket(LN_LEN - 3.0 * MM)  # L/N tips 3 mm into their openings, earth 7.6 mm
    al = ("move", "align", None, over, True)
    tc = ("move", "touch", "align", touch, True)
    first = [
        ("over the adapter", 2.5, up(A_START, 0.050), hr, OPEN_WIDE, g, None, False),
        ("fingers down at its ends", 1.5, A_START, hr, OPEN_WIDE, g, None, False),
        ("pinch it by its ends", 1.0, A_START, hr, PINCH, g, None, True),
        ("lift it", 1.2, up(A_START, 0.040), hr, PINCH, None, None, False),
        ("to the hold point", 1.8, A_HOLD, hr, PINCH, None, None, False),
    ]
    after = [
        ("lift with the pins out", 1.2, up(A_HOLD, 0.030), hr, PINCH, None, None, False),
        ("over the socket", 2.2, carry_hi, hr, PINCH, None, None, False),
        ("down over the openings", 1.5, over, hr, PINCH, None, None, False),
        ("line the earth pin up", 1.0, over, hr, PINCH, al, None, False),
        ("earth pin just over its opening", 1.0, touch, hr, PINCH, al, None, False),
        ("line up again", 0.8, touch, hr, PINCH, tc, None, False),
        ("pins into the openings", 1.5, entered, hr, PINCH, tc, None, False),
        ("let go", 0.8, entered, hr, OPEN_WIDE, tc, None, True),
        ("fingers up", 1.0, entered, hr, OPEN_WIDE, tc, (0.0, 0.0, 0.055), False),
    ]
    return first, after


def press_plan():
    """Right hand, closed, presses the adapter's top (read where it now sits) to past flush, eases off, lifts."""
    top_in_a = _T(_grip_down(), np.array([0.0, -2.0 * MM, ADAPTER_H + PAD_TIP]))  # closed fingertips on the top face
    seated = a_over_socket(0.0)
    rest = a_over_socket(LN_LEN - 3.0 * MM)
    pr = ("follow", "press", rest, CENTRE_REF)
    steps = [("close over the top", 1.0, rest, top_in_a, FINGER_CLOSED, pr, (0.0, 0.0, 0.030), True),
             ("fingertips on the top", 1.2, rest, top_in_a, FINGER_CLOSED, pr, (0.0, 0.0, 0.002), False)]
    n, z = 8, 0.0
    for k in range(n):
        z = 0.002 + (-(LN_LEN - 3.0 * MM) - PRESS_LEAD - 0.002) * (k + 1) / n
        steps.append((f"press down ({k + 1})", 0.5, rest, top_in_a, FINGER_CLOSED, pr, (0.0, 0.0, z), False))
    steps += [("hold the press", 1.0, rest, top_in_a, FINGER_CLOSED, pr, (0.0, 0.0, z), False),
              ("ease off", 1.0, seated, top_in_a, FINGER_CLOSED, None, (0.0, 0.0, -0.002), False),
              ("lift off", 1.2, seated, top_in_a, FINGER_CLOSED, None, (0.0, 0.0, 0.050), False)]
    return steps


def left_plan():
    """Left hand: beside the toggle tab (read the held adapter), then the push steps, then lift off."""
    rd = ("follow", "tab", A_HOLD, np.array([UK_TOGGLE_X, ADAPTER_W / 2, TAB_TOP0]))
    before = [("high over the toggle", 1.6, A_HOLD, left_hand_in_a(0.070, 0.030), OPEN_WIDE, rd, None, False),
              ("over the toggle", 1.2, A_HOLD, left_hand_in_a(0.025, 0.012), OPEN_WIDE, rd, None, False),
              ("beside the toggle", 1.2, A_HOLD, left_hand_in_a(0.004, 0.0), OPEN_WIDE, rd, None, False)]
    end = -UK_TRAVEL * min(DEMO_TOGGLE, 1.0)
    after = [("lift the finger", 1.0, A_HOLD, left_hand_in_a(end + 0.030, 0.006), OPEN_WIDE, rd, None, False),
             ("back off", 1.0, A_HOLD, left_hand_in_a(end + 0.060, 0.040), OPEN_WIDE, rd, None, False),
             ("clear to the left", 1.2, A_HOLD, left_hand_in_a(end + 0.080, 0.050), OPEN_WIDE, rd, (0.0, 0.120, 0.0),
              False)]
    return before, after


def _waypoints(arm, plan):
    return [PlanWaypoint(arm, secs, pose, hand, finger, label, read, offset if offset is not None else (0, 0, 0),
                         keep) for label, secs, pose, hand, finger, read, offset, keep in plan]


FINAL_HOLD_S = 3.0


def demo():
    LEFT_PARK_GRIP = Grip(0.0, _left_rot())  # the left hand parks pitched like its work pose (near its ready pose)
    r_first, r_after = right_plan()
    l_before, l_after = left_plan()
    right = _waypoints("right", r_first)
    t_hold = sum(w.duration for w in right)
    left = [Waypoint(2.0, tuple(float(v) for v in PARK["left"]), OPEN_WIDE, "park", LEFT_PARK_GRIP),
            Waypoint(max(t_hold - 2.0, 0.5), tuple(float(v) for v in PARK["left"]), OPEN_WIDE, "wait for the hold",
                     LEFT_PARK_GRIP)]
    lw = _waypoints("left", l_before)
    lw += [TogglePush(k, PUSH_STEP_S, f"push the toggle down ({k + 1})") for k in range(PUSH_STEPS)]
    lw.append(TogglePush(PUSH_STEPS, 0.8, "hold it at the end"))
    lw += _waypoints("left", l_after)
    left += lw
    t_toggle = sum(w.duration for w in lw)
    left.append(Waypoint(2.0 * DEMO_PACE, tuple(float(v) for v in PARK["left"]), OPEN_WIDE, "park again", LEFT_PARK_GRIP))
    right.append(PlanWaypoint("right", t_toggle, A_HOLD, HAND_R_IN_A, PINCH, "hold still for the toggle", keep=True))
    right += _waypoints("right", r_after)
    right += _waypoints("right", press_plan())
    end_rot = _grip_down()
    right.append(Waypoint(2.0 * DEMO_PACE, tuple(float(v) for v in PARK["right"]), FINGER_CLOSED, "park",
                          Grip(0.0, end_rot)))
    right.append(Waypoint(FINAL_HOLD_S, tuple(float(v) for v in PARK["right"]), FINGER_CLOSED, "hold clear",
                          Grip(0.0, end_rot)))
    total = sum(w.duration for w in right)
    t_left = sum(w.duration for w in left)
    left.append(Waypoint(max(total - t_left, 0.5), tuple(float(v) for v in PARK["left"]), OPEN_WIDE, "wait", LEFT_PARK_GRIP))
    return {"left": left, "right": right}


# ---------------------------------------------------------------- offline test layouts (states.yaml)
def _quat(rot):
    from yam_arena.ik import matrix_to_quat_xyzw
    return [float(v) for v in matrix_to_quat_xyzw(np.asarray(rot, float))]


def adapter_on_socket(gap_mm, dx_mm=0.0):
    """[origin, quat] of the adapter square over the socket, its pin face gap_mm above the socket face."""
    p = a_over_socket(gap_mm * MM, (dx_mm * MM, 0.0))
    return [float(v) for v in p[:3, 3]], _quat(p[:3, :3])


def adapter_standing(x, y):
    p = adapter_pose((x, y, TOP + 0.3 * MM))
    return [float(v) for v in p[:3, 3]], _quat(p[:3, :3])
