"""Geometry of the hidden-leak inspection. NumPy only (no Isaac).

Units: metres, kilograms, radians; quaternions xyzw. Station frame: x forward from the line between the arm bases,
y left, z up; TOP is the table top. Heights are above the table top.

A steel machine housing stands 60 mm from a wall, both running straight away from the robot. The gap between them is
open at its near end (facing the robot) and closed at its far end, at the bottom, and on top by a steel cover strip.
A coolant leak has left a bright green trace on the housing's side face, inside the gap, at one of five places along
it (drawn per trial from the episode seed). Looking into the open end, that face is seen edge-on, so nothing shows the
trace. An Ullman HTC-2LT lighted inspection mirror (60 mm round glass head, set at 45 degrees on its swivel) slid into the gap
shows the face to a camera behind the handle. The robot then marks the leak's place on the cover strip, directly
above the trace, with an oil-based paint marker, so the repair goes to the right spot.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top

# -- housing, wall, gap (the gap runs along x) ----------------------------------------------------------
GAP_X = (0.225, 0.405)  # open end at x 0.225 (facing the robot), closed end at 0.405
GAP = 0.060  # housing side face to wall: the 60.3 mm, 8 mm thick head at 45 degrees is 48 mm across, so 6 mm each side
FACE_Y = -0.030  # the hidden face: the housing's side face, facing +y; the wall's inner face is at FACE_Y + GAP
PANEL_T = 0.005
HEIGHT = 0.130
CAP_T = 0.005
CAP_TOP = HEIGHT + CAP_T  # the cover strip's top face (where the mark goes)
CAP_Y = (FACE_Y - PANEL_T, FACE_Y + GAP + PANEL_T)  # the strip spans the housing side, the gap and the wall
BASE_T = 0.005

# -- the leak trace (hidden state: one of five places, drawn from the episode seed) ----------------------
LEAK_X = (0.300, 0.320, 0.340, 0.360, 0.380)  # 20 mm apart, 7.5 to 15.5 cm in from the open end
LEAK_Z = 0.075  # centre height of the trace on the face
LEAK_SIZE = (0.022, 0.050)  # width along x, height: a run-down trace
LEAK_PARK = np.array([0.0, -0.60, -0.30])  # where the unused trace body is never; the live one is placed at reset
GREEN = (0.10, 0.85, 0.20)  # coolant dye


def leak_index(seed: int | None) -> int:
    """The trial's leak place from the episode seed (no seed: the middle place)."""
    if seed is None:
        return len(LEAK_X) // 2
    return int(np.random.default_rng(int(seed)).integers(len(LEAK_X)))


# -- inspection mirror (Ullman HTC-2LT, free tool): the handle along the tool's +x, the head at the +x end -------
# Maker's product page (opened 2026-11-19): 2-3/8 in (60.3 mm) round mirror, built-in LED (two CR2016 cells in a
# black plastic case behind the glass), telescopes 10 to 37 in (254 to 940 mm), 4.5 oz (128 g).
GRIP = (0.100, 0.0127)  # vinyl grip: length, diameter (diameter [estimate])
SHAFT = (0.120, 0.0050)  # the telescoped steel shaft past the grip: length set for this job, diameter [estimate]
HEAD_D = 0.0603
HEAD_T = 0.008  # glass and the black plastic battery case behind it [estimate]
HEAD_YAW = math.radians(45.0)  # set on the swivel: a ray travelling +x is turned toward -y (onto the hidden face)
# The swivel sits on the head's rim at its top, so the head hangs below the shaft's axis:
HEAD_DROP = HEAD_D / 2 + SHAFT[1] / 2  # shaft axis to head centre
HEAD_REACH = GRIP[0] / 2 + SHAFT[0]  # tool origin (grip centre) to head centre, along the tool's +x
TOOL_MASS = 0.128  # the maker's published 4.5 oz
LED_INTENSITY = 1.5e5  # the HTC-2LT's built-in LED (render units)
TOOL_REST = np.array([0.165, -0.130, 0.100])  # grip centre at rest, the free end of the grip toward the robot
SUPPORT = (0.012, 0.030, 0.094)  # each support block (x, y, z) under the grip, clear of the hand at its free end
SUPPORT_DX = (-0.040, 0.060)  # under the grip and under the shaft's root; the hand takes the grip between them

# -- paint marker (Sharpie oil-based paint marker, medium point: about 142 x 16 mm) ---------------------------
MARKER = (0.142, 0.016)  # length, barrel diameter
MARKER_TIP = (0.008, 0.004)  # tip length, diameter
MARKER_MASS = 0.022  # [estimate]
HOLDER_XY = np.array([0.190, 0.090])  # a steel pen cup: the marker stands in it tip down
HOLDER = (0.034, 0.034, 0.060)  # outer size; 18 mm square bore, 8 mm floor
HOLDER_BORE = 0.018
HOLDER_FLOOR = 0.008
HOLDER_STAND = 0.050  # the cup sits on a 50 mm steel stand (keeps the side grasp within the arm's reach)
MARK_DOTS = 4  # paint dots the scene can show (a fifth dab still counts in the score; it is just not drawn)
DOT_D = 0.008

# -- scoring ------------------------------------------------------------------------------------------
FULL_MM = 6.0  # a mark within 6 mm (along the gap) of the trace's centre earns 1.0
ZERO_MM = 14.0  # 0 at 14 mm or more (less than the 20 mm between places: a neighbouring place earns nothing)
REST_SPEED = 0.01  # still: moving under 1 cm/s ...
REST_TURN = 0.10  # ... and turning under 0.1 rad/s (about 6 deg/s): under 5 mm and 3 degrees in 0.5 s
CONTACT = 0.00025  # touching: within 0.25 mm (the simulator holds a resting contact 0.06 to 0.10 mm apart)
RELEASE = 0.005  # a tool is released when no part of the robot is within 5 mm of it
GLASS_RIM = 0.0015  # a reflection counts only 1.5 mm or more inside the glass's edge
IMG_MARGIN = 4.0  # pixels: a point counts as in the image this far inside its border (640 x 360 render)
TRACE_IN = (0.008, 0.015)  # the trace's middle, +-8 mm along the gap and +-15 mm up (16 x 30 mm), must show


def _axes(q):
    x, y, z, w = (float(v) for v in q)
    ax = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])
    ay = np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)])
    az = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
    return ax, ay, az


def head_frame(tool_pose):
    """World centre, normal and in-plane axes of the mirror head for a tool pose."""
    ax, ay, az = _axes(tool_pose[3:])
    c = np.asarray(tool_pose[:3], float) + ax * HEAD_REACH - az * HEAD_DROP
    cy, sy = math.cos(HEAD_YAW), math.sin(HEAD_YAW)
    n = cy * ax + sy * ay  # the plate's thin axis (body x turned by the yaw)
    u = -sy * ax + cy * ay  # in-plane, horizontal
    return c, n, u, az


def head_center(tool_pose) -> np.ndarray:
    return head_frame(tool_pose)[0]


def head_face_clearance(tool_pose) -> float:
    """Distance (m) from the hidden face to the head's nearest point (negative: through it)."""
    c, n, u, v = head_frame(tool_pose)
    reach_y = HEAD_D / 2 * math.hypot(u[1], v[1]) + HEAD_T / 2 * abs(n[1])
    return float(c[1] - reach_y - FACE_Y)


def marker_tip(marker_pose) -> np.ndarray:
    """World point of the marker's tip: the marker's body -z end (origin at the barrel's centre)."""
    _, _, az = _axes(marker_pose[3:])
    return np.asarray(marker_pose[:3], float) - az * (MARKER[0] / 2 + MARKER_TIP[0])


def on_strip(p) -> bool:
    """Does the tip touch the cover strip's top face (within CONTACT of it; a hover above that is no mark)?"""
    return (GAP_X[0] <= p[0] <= GAP_X[1] + PANEL_T and CAP_Y[0] <= p[1] <= CAP_Y[1]
            and -0.002 <= p[2] - (TOP + CAP_TOP) <= CONTACT)


def start_poses() -> dict[str, np.ndarray]:
    ident = [0.0, 0.0, 0.0, 1.0]
    out = {"housing": np.array([0.0, 0.0, TOP, *ident]),
           "leak": np.array([*LEAK_PARK, *ident]),
           "mirror": np.array([TOOL_REST[0], TOOL_REST[1], TOP + TOOL_REST[2], *ident]),
           "marker": np.array([HOLDER_XY[0], HOLDER_XY[1],  # tip down on the cup's floor
                               TOP + HOLDER_STAND + HOLDER_FLOOR + MARKER_TIP[0] + MARKER[0] / 2, *ident])}
    for i in range(MARK_DOTS):
        out[f"dot_{i}"] = np.array([0.0, -0.60 - 0.02 * i, -0.30, *ident])
    return out


def leak_pose(index: int) -> np.ndarray:
    """The trace body's pose: its origin on the hidden face at the place's centre."""
    return np.array([LEAK_X[index], FACE_Y, TOP + LEAK_Z, 0.0, 0.0, 0.0, 1.0])


def head_in_gap(tool_pose) -> bool:
    """Is the mirror head's centre inside the gap (along it, under the strip, between the face and the wall)?"""
    c = head_center(tool_pose)
    return GAP_X[0] < c[0] < GAP_X[1] and 0.0 < c[2] - TOP < HEIGHT and FACE_Y < c[1] < FACE_Y + GAP


def rot(q) -> np.ndarray:
    """Rotation matrix (columns: body x, y, z in the world) of a quaternion xyzw."""
    return np.column_stack(_axes(q))


def motion(prev_pose, pose, dt: float, point=None) -> tuple[float, float]:
    """Speed (m/s) of a body point (default: its origin) and turn rate (rad/s) between two poses ``dt`` apart."""
    r0, r1 = rot(prev_pose[3:]), rot(pose[3:])
    off = np.zeros(3) if point is None else np.asarray(point, float)
    p0 = np.asarray(prev_pose[:3], float) + r0 @ off
    p1 = np.asarray(pose[:3], float) + r1 @ off
    c = (np.trace(r0.T @ r1) - 1.0) / 2.0
    return float(np.linalg.norm(p1 - p0) / dt), float(math.acos(max(-1.0, min(1.0, c))) / dt)


HEAD_POINT = np.array([HEAD_REACH, 0.0, -HEAD_DROP])  # the head's centre in the mirror's body frame


# -- the housing's solid parts (world, axis-aligned boxes: centre, size); the assets build the same boxes -------------
def housing_boxes() -> dict[str, tuple[tuple, tuple]]:
    x0, x1 = GAP_X
    xc, lx = (x0 + x1) / 2, x1 - x0
    t, g, hz = PANEL_T, GAP, HEIGHT
    yc, ly = (CAP_Y[0] + CAP_Y[1]) / 2, CAP_Y[1] - CAP_Y[0]
    return {"side": ((xc, FACE_Y - t / 2, hz / 2), (lx, t, hz)),  # the hidden face is its +y face
            "wall": ((xc, FACE_Y + g + t / 2, hz / 2), (lx, t, hz)),
            "strip": ((xc + t / 2, yc, hz + CAP_T / 2), (lx + t, ly, CAP_T)),
            "far_end": ((x1 + t / 2, yc, hz / 2), (t, ly, hz)),
            "floor": ((xc, FACE_Y + g / 2, BASE_T / 2), (lx, g, BASE_T))}


# -- contact with the hidden face (the whole bounded face: x 22.5 to 40.5 cm, from the table to the strip) --------------
FACE_Z = (0.0, HEIGHT)


def face_distance(points) -> tuple[np.ndarray, np.ndarray]:
    """Distance (m) from world points to the bounded hidden face, and the unit direction from the face's nearest point
    to each. Negative only for a point inside the housing side's 5 mm panel, behind the face within its span."""
    p = np.atleast_2d(np.asarray(points, float))
    q = np.column_stack([np.clip(p[:, 0], *GAP_X), np.full(len(p), FACE_Y), np.clip(p[:, 2] - TOP, *FACE_Z) + TOP])
    v = p - q
    dist = np.linalg.norm(v, axis=1)
    inside = (q[:, 0] == p[:, 0]) & (q[:, 2] == p[:, 2])  # within the face's span (x and height)
    panel = inside & (v[:, 1] < 0) & (v[:, 1] >= -PANEL_T)
    dist = np.where(panel, v[:, 1], dist)  # in the panel: negative; elsewhere (in front, or out behind it): positive
    u = np.where(inside[:, None], np.array([0.0, 1.0, 0.0]), v / np.maximum(dist, 1e-9)[:, None])
    return dist, u


def _disc_points(c, n, u, v, radius, rings=8, spokes=48):
    pts = [c]
    for k in range(1, rings + 1):
        r = radius * k / rings
        a = np.linspace(0, 2 * math.pi, spokes, endpoint=False)
        pts.append(c + r * (np.cos(a)[:, None] * u + np.sin(a)[:, None] * v))
    return np.vstack(pts)


def _proj_face(p):
    """Nearest point of the bounded hidden face (a rectangle) to each point."""
    return np.stack([np.clip(p[..., 0], *GAP_X), np.full(p.shape[:-1], FACE_Y),
                     np.clip(p[..., 2], TOP + FACE_Z[0], TOP + FACE_Z[1])], axis=-1)


def _proj_boxes(p, c, r, h):
    """Nearest point of each solid box (centre c, rotation r with the box axes as columns, half sizes h) to p[k]."""
    local = np.clip(np.einsum("kji,kj->ki", r, p - c), -h, h)
    return c + np.einsum("kij,kj->ki", r, local)


def _proj_cyls(p, c, a, radius, half):
    """Nearest point of each solid cylinder (centre c, unit axis a, radius, half length) to p[k]."""
    w = p - c
    s = np.einsum("ki,ki->k", w, a)
    q = w - s[:, None] * a
    rho = np.linalg.norm(q, axis=1)
    q = q * np.minimum(1.0, radius / np.maximum(rho, 1e-12))[:, None]
    return c + np.clip(s, -half, half)[:, None] * a + q


def _alt_dist(proj_a, proj_b, start, iters: int = 400) -> np.ndarray:
    """Distance between pairs of convex solids by alternating projections (exact in the limit; 0 when they meet)."""
    x = proj_a(start)
    for _ in range(iters):
        y = proj_b(x)
        x = proj_a(y)
    return np.linalg.norm(x - proj_b(x), axis=1)


def _mirror_solids(tool_pose):
    """The mirror as convex solids: the head (a 57 x 4 mm cylinder), the shaft (a cylinder), the grip (a box) and the
    swivel (a box)."""
    r = rot(tool_pose[3:])
    o = np.asarray(tool_pose[:3], float)
    ax, az = r[:, 0], r[:, 2]
    c, n, _, _ = head_frame(tool_pose)
    cyls = (np.array([c, o + ax * (GRIP[0] / 2 + SHAFT[0] / 2)]), np.array([n, ax]),
            np.array([HEAD_D / 2, SHAFT[1] / 2]), np.array([HEAD_T / 2, SHAFT[0] / 2]))
    boxes = (np.array([o, o + ax * HEAD_REACH - az * (SHAFT[1] / 2 + 0.002)]), np.array([r, r]),
             np.array([[GRIP[0] / 2, GRIP[1] / 2, GRIP[1] / 2], [0.004, 0.003, 0.003]]))
    return cyls, boxes


def _marker_solids(marker_pose):
    r = rot(marker_pose[3:])
    o = np.asarray(marker_pose[:3], float)
    az = r[:, 2]
    return (np.array([o, o - az * (MARKER[0] / 2 + MARKER_TIP[0] / 2)]), np.array([az, az]),
            np.array([MARKER[1] / 2, MARKER_TIP[1] / 2]), np.array([MARKER[0] / 2, MARKER_TIP[0] / 2]))


def _cyls_face(cyls) -> float:
    c, a, rad, half = cyls
    return float(_alt_dist(lambda p: _proj_cyls(p, c, a, rad, half), _proj_face, c).min())


def _boxes_face(boxes) -> float:
    c, r, h = boxes
    if len(c) == 0:
        return 1.0
    return float(_alt_dist(lambda p: _proj_boxes(p, c, r, h), _proj_face, c).min())


def mirror_face_clearance(tool_pose) -> float:
    """Clearance (m) from the bounded hidden face to the mirror's solids (head, shaft, grip, swivel), wherever they are:
    the exact distance between convex shapes, 0 when they meet."""
    cyls, boxes = _mirror_solids(tool_pose)
    return min(_cyls_face(cyls), _boxes_face(boxes))


def marker_face_clearance(marker_pose) -> float:
    """Clearance (m) from the bounded hidden face to the paint marker (barrel and tip)."""
    return _cyls_face(_marker_solids(marker_pose))


def _stack_boxes(boxes):
    if boxes is None or len(boxes) == 0:
        return np.zeros((0, 3)), np.zeros((0, 3, 3)), np.zeros((0, 3))
    return (np.array([b[0] for b in boxes], float), np.array([b[1] for b in boxes], float),
            np.array([b[2] for b in boxes], float))


def robot_face_clearance(boxes) -> float:
    """Clearance (m) from the bounded hidden face to the robot's collision pieces, each as its bounding box
    ``(centre, rotation, half sizes)`` (whole solids, so a flat finger face counts, not only its corners)."""
    return _boxes_face(_stack_boxes(boxes))


# -- release: how close the robot is to each tool ---------------------------------------------------------------------
def _robot_to(cyls, tool_boxes, boxes) -> float:
    rc, rr, rh = _stack_boxes(boxes)
    if len(rc) == 0:
        return 1.0
    k = len(rc)
    out = []
    c, a, rad, half = cyls
    for i in range(len(c)):
        rep = lambda v: np.repeat(v[None], k, axis=0)  # noqa: E731
        out.append(_alt_dist(lambda p: _proj_boxes(p, rc, rr, rh),
                             lambda p: _proj_cyls(p, rep(c[i]), rep(a[i]), np.full(k, rad[i]), np.full(k, half[i])),
                             rc).min())
    if tool_boxes is not None:
        bc, br, bh = tool_boxes
        for i in range(len(bc)):
            out.append(_alt_dist(lambda p: _proj_boxes(p, rc, rr, rh),
                                 lambda p: _proj_boxes(p, np.repeat(bc[i][None], k, 0), np.repeat(br[i][None], k, 0),
                                                       np.repeat(bh[i][None], k, 0)), rc).min())
    return float(min(out))


def mirror_robot_distance(tool_pose, boxes) -> float:
    """Nearest distance (m) from the robot's collision pieces (whole boxes) to the mirror's solids."""
    cyls, tool_boxes = _mirror_solids(tool_pose)
    return _robot_to(cyls, tool_boxes, boxes)


def marker_robot_distance(marker_pose, boxes) -> float:
    return _robot_to(_marker_solids(marker_pose), None, boxes)


def on_housing(pose) -> bool:
    """Is a tool's origin over the housing (on the strip or in the gap) rather than on the table, blocks or cup?"""
    return GAP_X[0] - PANEL_T <= pose[0] <= GAP_X[1] + PANEL_T and CAP_Y[0] <= pose[1] <= CAP_Y[1]


def tool_down(speed: float, turn: float, robot_dist: float, pose) -> bool:
    """A tool put down: still (under 1 cm/s and 0.1 rad/s), released (no part of the robot within 5 mm, so nothing
    but the table, its blocks or its cup can hold it still) and not left on the housing."""
    return speed < REST_SPEED and turn < REST_TURN and robot_dist >= RELEASE and not on_housing(pose)


# -- a view of the trace through the mirror -----------------------------------------------------------------------------
_TO_GL = {"opengl": np.eye(3), "ros": np.diag([1.0, -1.0, -1.0]),
          "world": np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])}


def camera_pose(link_pose, mount_pos, mount_xyzw, convention: str):
    """World position and rotation (OpenGL axes: x right, y up, looking along -z) of a camera fixed to a link."""
    rl = rot(link_pose[3:])
    pos = np.asarray(link_pose[:3], float) + rl @ np.asarray(mount_pos, float)
    return pos, rl @ rot(mount_xyzw) @ _TO_GL[convention]


def project(cam, p):
    """Pixel (u, v) of a world point in a camera ``(pos, R, (fx, fy, cx, cy, width, height))``, or None behind it."""
    pos, r, (fx, fy, cx, cy, _, _) = cam
    d = r.T @ (np.asarray(p, float) - pos)
    if d[2] >= -1e-6:
        return None
    return cx + fx * d[0] / -d[2], cy - fy * d[1] / -d[2]


def in_image(cam, p) -> bool:
    uv = project(cam, p)
    w, h = cam[2][4], cam[2][5]
    return uv is not None and IMG_MARGIN <= uv[0] <= w - IMG_MARGIN and IMG_MARGIN <= uv[1] <= h - IMG_MARGIN


def _segment_hits(a, b, boxes, skip_holding_a: bool = False) -> bool:
    """Does the segment a-b pass through any box ``(centre, R, half sizes)``? With ``skip_holding_a``, a box that
    holds ``a`` is skipped (the camera's own mount on the robot's gripper)."""
    for c, r, half in boxes:
        la, lb = r.T @ (a - c), r.T @ (b - c)
        half = np.asarray(half, float) - 1e-4
        if skip_holding_a and np.all(np.abs(la) <= half):
            continue
        d = lb - la
        t0, t1 = 0.0, 1.0
        hit = True
        for k in range(3):
            if abs(d[k]) < 1e-12:
                if abs(la[k]) > half[k]:
                    hit = False
                    break
                continue
            s0, s1 = (-half[k] - la[k]) / d[k], (half[k] - la[k]) / d[k]
            t0, t1 = max(t0, min(s0, s1)), min(t1, max(s0, s1))
            if t0 > t1:
                hit = False
                break
        if hit:
            return True
    return False


def static_occluders():
    return [(np.array([c[0], c[1], c[2] + TOP]), np.eye(3), np.asarray(s, float) / 2) for c, s in housing_boxes().values()]


def tool_occluders(tool_pose, marker_pose=None):
    """The mirror's grip and shaft (and the marker) as boxes that can block a view."""
    r = rot(tool_pose[3:])
    o = np.asarray(tool_pose[:3], float)
    out = [(o, r, np.array([GRIP[0], GRIP[1], GRIP[1]]) / 2),
           (o + r[:, 0] * (GRIP[0] / 2 + SHAFT[0] / 2), r, np.array([SHAFT[0], SHAFT[1], SHAFT[1]]) / 2)]
    if marker_pose is not None:
        rm = rot(marker_pose[3:])
        out.append((np.asarray(marker_pose[:3], float) - rm[:, 2] * MARKER_TIP[0] / 2, rm,
                    np.array([MARKER[1], MARKER[1], MARKER[0] + MARKER_TIP[0]]) / 2))
    return out


def trace_points(leak_x: float) -> np.ndarray:
    """The trace's middle: its centre and four points +-8 mm along the gap and +-15 mm up, on the film's surface."""
    y, z = FACE_Y + 0.001, TOP + LEAK_Z
    dx, dz = TRACE_IN
    return np.array([[leak_x, y, z], [leak_x - dx, y, z], [leak_x + dx, y, z], [leak_x, y, z - dz], [leak_x, y, z + dz]])


def reflected_point(tool_pose, cam_pos, p):
    """Where on the glass a camera at ``cam_pos`` sees point ``p`` reflected, or None (wrong side, or off the glass)."""
    c, n, _, _ = head_frame(tool_pose)
    g = -n  # the glass faces -n: toward the robot and the hidden face; the back is steel
    if (cam_pos - c) @ g <= 0 or (p - c) @ g <= 0:
        return None
    pv = p - 2 * ((p - c) @ g) * g
    den = (pv - cam_pos) @ g
    if abs(den) < 1e-12:
        return None
    t = ((c - cam_pos) @ g) / den
    if not 0 < t < 1:
        return None
    m = cam_pos + t * (pv - cam_pos)
    if np.linalg.norm(m - c) > HEAD_D / 2 - GLASS_RIM:
        return None
    return m


def trace_seen(tool_pose, leak_x: float, cams, occluders, robot_boxes=()) -> bool:
    """Does any camera see the trace's middle (every point of ``trace_points``) in the mirror's glass, inside the
    image, with no solid between the camera, the glass and the trace? ``cams``: [(pos, R, lens)]; ``occluders``: the
    housing and the tools; ``robot_boxes``: the robot's collision pieces (both arms)."""
    pts = trace_points(leak_x)
    for cam in cams:
        ok = True
        for p in pts:
            m = reflected_point(tool_pose, cam[0], p)
            if m is None or not in_image(cam, m) or _segment_hits(cam[0], m, occluders) \
                    or _segment_hits(cam[0], m, robot_boxes, skip_holding_a=True) \
                    or _segment_hits(m, p, occluders) or _segment_hits(m, p, robot_boxes):
                ok = False
                break
        if ok:
            return True
    return False
