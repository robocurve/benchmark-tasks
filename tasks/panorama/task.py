"""Take a phone panorama of three boards, turning the phone about its camera above the cross.

An iPhone 12 stands in a desk stand with the Camera app open in Pano. Three boards stand on a 130-degree fan round a
tape cross on the mat; each board has a printed vertical line, and a 6 mm rod stands on the line's sight line from
the cross, 150 mm in front of it. Seen from a lens above the cross the rod hides the line; seen from a lens that has
moved off to the side, the line shows beside the rod (parallax). The instruction asks for a panorama of the three
boards with the phone turned about its camera above the cross, and for the phone back in its stand.

The Camera app is not simulated: a rule stands in for it (see score()), reading the phone's pose, the fingertips'
touches on the glass and the robot's forces on the edge buttons every control step.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
         --scorer task_score --max-steps 3000 ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from scenekit import UsdBody, on_reset, on_step, physics  # noqa: E402

import geometry as g  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0

INSTRUCTION = ("Take a panorama that shows all three boards in full, turning the phone about its camera above the "
               "cross, then put the phone back in its stand.")

physics(g.PHYSICS_HZ)

TOP = g.TOP
_P0, _R0 = g.phone_in_stand()


def _quat_xyzw(R) -> tuple:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


FIXTURES = UsdBody("fixtures", usd="assets/fixtures.usda", pos=(0.0, 0.0, TOP), size=(0.9, 1.2, 0.32),
                   friction=(0.5, 0.45))
PHONE = UsdBody("phone", usd="assets/phone.usda", pos=(float(_P0[0]), float(_P0[1]), float(_P0[2] + TOP)),
                rot_xyzw=_quat_xyzw(_R0), size=(g.PT, g.PW, g.PH), mass=g.PHONE_MASS, friction=(0.5, 0.45),
                solver_iters=32, torsional_patch=0.006)
OBJECTS = [FIXTURES, PHONE]

_STATE: dict = {}
_VIEWS: dict = {}
_DEBUG = bool(os.environ.get("PANO_DEBUG"))
DT = 1.0 / g.CONTROL_HZ


def _rotation(xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# -- the robot's collision hulls --------------------------------------------------------------------------------------
_SAMPLES: dict = {}


def _link_samples() -> dict:
    """Each link's collision hulls as points (vertices and points along the hull edges every 10 mm), link frame, for
    the robot-in-view check."""
    if not _SAMPLES:
        from scipy.spatial import ConvexHull
        from yam_arena.analysis import link_hulls

        for name, hulls in link_hulls().items():
            pts = []
            for h in hulls:
                v = h.vertices
                pts.append(v)
                try:
                    ch = ConvexHull(v)
                except Exception:
                    continue
                for s in ch.simplices:
                    for i, j in ((0, 1), (1, 2), (2, 0)):
                        a, b = v[s[i]], v[s[j]]
                        k = int(np.linalg.norm(b - a) / 0.010)
                        if k >= 1:
                            pts.append(a + np.linspace(0, 1, k + 2)[1:-1, None] * (b - a))
            _SAMPLES[name] = np.unique(np.round(np.concatenate(pts) / 0.004) * 0.004, axis=0)   # a 4 mm grid
    return _SAMPLES


TAPE_SHORT = 0.002                  # the conductive tape stops this short of the pad's edge


def _end_faces() -> dict:
    """Each finger link's taped fingertip end-face points (link frame): its hull vertices within 1.2 mm of the tip end
    and 2 mm or more out from the pad's plane (the tape stops 2 mm short of the pad, so a pinching finger's end face
    does not touch the glass it pinches)."""
    if "_ends" not in _SAMPLES:
        from yam_arena import YAM_URDF
        from yam_arena.analysis import link_hulls
        from yam_arena.urdf import forward_kinematics

        q = {f"joint{i}": 0.0 for i in range(1, 9)}
        fk = forward_kinematics(YAM_URDF, q)
        Gi = np.linalg.inv(fk["gripper"])
        out = {}
        for n in ("tip_left", "tip_right"):
            T = Gi @ fk[n]
            v = np.concatenate([h.vertices for h in link_hulls()[n]])
            zg = v @ T[2, :3] + T[2, 3]                # how far along the approach, gripper frame
            xg = v @ T[0, :3] + T[0, 3]                # across the closing axis (the pads at 0 when closed)
            out[n] = v[(zg >= zg.max() - 0.0012) & (np.abs(xg) >= TAPE_SHORT)]
        _SAMPLES["_ends"] = out
    return _SAMPLES["_ends"]


def robot_hulls(link_poses_by_side: dict) -> list:
    """[(side, link, Hull)] of every robot link, world frame (each hull carries its bounding box as .lo, .hi)."""
    from yam_arena.analysis import world_hulls

    out = []
    for side, poses in link_poses_by_side.items():
        for n, h in world_hulls(poses):
            h.lo, h.hi = h.vertices.min(0), h.vertices.max(0)
            out.append((side, n, h))
    return out


def robot_points(link_poses_by_side: dict) -> np.ndarray:
    S = _link_samples()
    out = []
    for poses in link_poses_by_side.values():
        for n, T in poses.items():
            if n in S:
                out.append(S[n] @ T[:3, :3].T + T[:3, 3])
    return np.concatenate(out)


def end_face_points(link_poses_by_side: dict) -> dict:
    E = _end_faces()
    return {(side, n): E[n] @ poses[n][:3, :3].T + poses[n][:3, 3] for side, poses in link_poses_by_side.items()
            for n in E if n in poses}


# -- geometry predicates (pure) ---------------------------------------------------------------------------------------
def _box_edge_points(size, step: float = 0.003) -> np.ndarray:
    half, pts = np.asarray(size) / 2, []
    for a in range(3):
        b, c = [i for i in range(3) if i != a]
        t = np.linspace(-half[a], half[a], max(2, int(np.ceil(size[a] / step)) + 1))
        for sb in (-1.0, 1.0):
            for sc in (-1.0, 1.0):
                p = np.zeros((len(t), 3))
                p[:, a], p[:, b], p[:, c] = t, sb * half[b], sc * half[c]
                pts.append(p)
    return np.concatenate(pts)


_EDGE_CACHE: dict = {}


def hulls_box_gap(hulls, centre, R, size) -> float:
    """Smallest gap (m) between any hull and an oriented box: hull vertices against the box, and the box's edges
    against each hull's planes (negative: overlap). Hulls farther than 30 mm (bounding boxes) are skipped."""
    from yam_arena.analysis import box_signed_distance

    if not hulls:
        return 1.0
    centre, R, size = np.asarray(centre, float), np.asarray(R, float), np.asarray(size, float)
    key = tuple(np.round(size, 6))
    if key not in _EDGE_CACHE:
        _EDGE_CACHE[key] = _box_edge_points(size)
    edges = centre + _EDGE_CACHE[key] @ R.T
    lo, hi = edges.min(0) - 0.03, edges.max(0) + 0.03
    H_lo = np.array([getattr(h, "lo", None) if hasattr(h, "lo") else h.vertices.min(0) for h in hulls])
    H_hi = np.array([getattr(h, "hi", None) if hasattr(h, "hi") else h.vertices.max(0) for h in hulls])
    near = np.flatnonzero(np.all(H_hi >= lo, axis=1) & np.all(H_lo <= hi, axis=1))
    best = 1.0
    for j in near:
        h = hulls[j]
        best = min(best, float(box_signed_distance(h.vertices, centre, R, size).min()))
        p = h.planes
        best = min(best, float((edges @ p[:, :3].T - p[:, 3]).max(axis=1).min()))
    return best


def phone_frame(pos, R):
    return np.asarray(pos, float), np.asarray(R, float)


def lens_world(pos, R) -> np.ndarray:
    return np.asarray(pos) + np.asarray(R) @ g.lens_local()


def camera_angles(R) -> tuple[float, float, float]:
    """(heading, pitch, roll) of the camera, degrees: heading of the optical axis, its elevation, and the image's
    roll (image right's rise)."""
    f, r, u = g.camera_axes(np.asarray(R))
    heading = math.degrees(math.atan2(f[1], f[0]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, f[2]))))
    roll = math.degrees(math.atan2(r[2], u[2]))
    return heading, pitch, roll


GLASS = (0.06942, 0.14461)


def touches(end_pts: dict, pos, R) -> dict:
    """Taped fingertip end faces on the glass, one touch per hand (its two fingertips together make one contact
    patch): {(side, "hand"): (y, z) the touching points' centre on the glass, phone frame}."""
    p, Rm = np.asarray(pos), np.asarray(R)
    pts = {}
    for (side, _), P in end_pts.items():
        loc = (P - p) @ Rm
        on = (loc[:, 0] >= g.PT / 2 - 0.002) & (loc[:, 0] <= g.PT / 2 + g.TOUCH_GAP) & \
             (np.abs(loc[:, 1]) <= GLASS[0] / 2) & (np.abs(loc[:, 2]) <= GLASS[1] / 2)
        if on.any():
            pts.setdefault(side, []).append(loc[on][:, 1:])
    return {(side, "hand"): np.concatenate(v).mean(0) for side, v in pts.items()}


def corner_box():
    """The camera corner as a box in the phone frame (centre, size): the back (with the plateau and lens rings),
    the top edge and the camera-side edge within CORNER_TOP of the top and CORNER_SIDE of the +y edge. Its front
    face stops 2 mm short of the screen glass, so a finger on the glass is not on it."""
    x0, x1 = -g.PT / 2 - g.LENS_PROUD - 0.0005, g.PT / 2 - 0.002
    y0, y1 = g.PW / 2 - g.CORNER_SIDE, g.PW / 2
    z0, z1 = g.PH / 2 - g.CORNER_TOP, g.PH / 2
    return np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]), np.array([x1 - x0, y1 - y0, z1 - z0])


def band_boxes() -> dict:
    """The button bands as boxes on their edges (phone frame): centre, size."""
    out = {}
    for name, (a, b), side in (("volume_up", g.VOL_UP, -1), ("volume_down", g.VOL_DOWN, -1), ("side", g.SIDE_BTN, 1)):
        zc = g.PH / 2 - (a + b) / 2
        out[name] = (np.array([0.0, side * (g.PW / 2 - 0.0005), zc]), np.array([g.PT - 0.002, 0.001, b - a]))
    return out


def in_view_flags(points: np.ndarray, lens, R, slit_half: float) -> dict:
    """Robot points within SEEN_RANGE in front of the camera and inside the frame's height: how many fall in the
    central slit, the left half and the right half of the frame."""
    f, r, u = g.camera_axes(np.asarray(R))
    rel = points - np.asarray(lens)
    x, y, z = rel @ f, rel @ r, rel @ u
    ok = (x > 0.003) & (np.linalg.norm(rel, axis=1) <= g.SEEN_RANGE)
    ax = np.degrees(np.arctan2(y, x))
    az = np.degrees(np.arctan2(z, np.hypot(x, y)))
    ok &= np.abs(az) <= g.FRAME_H / 2
    return dict(slit=int((ok & (np.abs(ax) <= slit_half)).sum()),
                left=int((ok & (ax <= 0) & (ax >= -g.FRAME_W / 2)).sum()),
                right=int((ok & (ax >= 0) & (ax <= g.FRAME_W / 2)).sum()))


def sheet_edges(i: int):
    """Board i's sheet: its two vertical edges' (x, y), and its bottom and top heights (above the table)."""
    u = g.bearing_vec(g.BEARINGS[i])
    v = np.array([-u[1], u[0]])
    c = g.board_xy(i)
    return c + v * g.SHEET_W / 2, c - v * g.SHEET_W / 2, g.SHEET_Z0, g.SHEET_Z0 + g.SHEET_H


def unwrap_to(a: float, ref: float) -> float:
    return ref + g.wrap(a - ref)


def evaluate_capture(frames: list, moved: set) -> dict:
    """Judge one saved capture from its frames: [{"c": heading (unwrapped), "lens": xyz (z above the table),
    "pitch": deg}, ...] in order. Returns the boards in full, aligned, and the sideways errors (mm).

    The panorama is the first frame's left half, the central strip of every frame, and the last frame's right half.
    A board's line enters it from the frame that holds it there: the first frame for a line left of that frame's
    centre, the last frame for a line right of its centre, otherwise the frame whose centre crosses the line. The
    lens's sideways distance from the board's sight line at that frame decides whether the rod hides the line."""
    c = np.array([f["c"] for f in frames])
    lens = np.array([f["lens"] for f in frames])
    pitch = np.array([f["pitch"] for f in frames])
    left_edge, right_edge = c[0] + g.FRAME_W / 2, c[-1] - g.FRAME_W / 2
    band = (pitch.max() - g.FRAME_H / 2, pitch.min() + g.FRAME_H / 2)
    out = dict(full=[], aligned=[], err_mm=[], via=[], cover=(round(float(left_edge), 2), round(float(right_edge), 2)))
    for i in range(3):
        line = g.board_xy(i)
        h = np.array([unwrap_to(g.heading_of(line - L[:2]), ci) for L, ci in zip(lens, c)])
        if h[0] >= c[0]:
            Lx, via = lens[0], "first frame"
        elif h[-1] <= c[-1]:
            Lx, via = lens[-1], "last frame"
        else:
            d = c - h
            k = next(j for j in range(len(d) - 1) if d[j] == 0 or (d[j] > 0) != (d[j + 1] > 0))
            t = 0.0 if d[k] == d[k + 1] else d[k] / (d[k] - d[k + 1])
            Lx, via = lens[k] + t * (lens[k + 1] - lens[k]), "crossing"
        vb = np.array([-math.sin(math.radians(g.BEARINGS[i])), math.cos(math.radians(g.BEARINGS[i]))])
        err = float(abs((Lx[:2] - np.asarray(g.CROSS)) @ vb))
        a, b, z0, z1 = sheet_edges(i)
        ha = unwrap_to(g.heading_of(a - Lx[:2]), float(c.mean()))
        hb = unwrap_to(g.heading_of(b - Lx[:2]), float(c.mean()))
        lo_h, hi_h = min(ha, hb), max(ha, hb)
        dist = float(np.linalg.norm(line - Lx[:2]))
        e_top = math.degrees(math.atan2(z1 - Lx[2], dist))
        e_bot = math.degrees(math.atan2(z0 - Lx[2], dist))
        full = bool(hi_h <= left_edge and lo_h >= right_edge and e_top <= band[1] and e_bot >= band[0])
        out["full"].append(full)
        out["aligned"].append(bool(full and err <= g.ALIGN_T and i not in moved))
        out["err_mm"].append(round(err * 1000, 2))
        out["via"].append(via)
    return out


def panorama_tier(ev: dict, clean: bool, level: bool, clear: bool) -> tuple[float, bool]:
    """(tier, perfect): 0.5 for three boards in full (1.0 or 0.75 if also aligned, clean, level and hands clear,
    by set-down), 0.25 for exactly two, else 0."""
    n = sum(ev["full"])
    if n == 3:
        perfect = bool(all(ev["aligned"]) and clean and level and clear)
        return (1.0 if perfect else 0.5), perfect
    return (0.25 if n == 2 else 0.0), False


def phone_set_down(pos, R) -> bool:
    """The phone back in its stand: its bottom edge in the stand's slot (on the seat, between the back plate and the
    lip), screen facing +x and leaning back on the plate."""
    _, Rs = g.phone_in_stand()
    n, yv, u = Rs[:, 0], Rs[:, 1], Rs[:, 2]
    O = np.array([g.STAND_SEAT_X, g.STAND_XY[1], g.SEAT_UP])
    p, Rm = np.asarray(pos), np.asarray(R)
    bottom = p + Rm @ np.array([0.0, 0.0, -g.PH / 2]) - np.array([0, 0, 0])
    rel = bottom - O
    in_slot = (-0.001 <= rel @ n <= g.SLOT + 0.001) and (-0.002 <= rel @ u <= g.LIP_H) and abs(rel @ yv) <= 0.025
    screen = Rm[:, 0]
    facing = abs(g.wrap(g.heading_of(screen[:2]))) <= 20.0 and screen[2] > 0.0
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, Rm[2, 2]))))
    return bool(in_slot and facing and 10.0 <= tilt <= 30.0)


# -- the monitor ------------------------------------------------------------------------------------------------------
class Monitor:
    """The rule that stands in for the Camera app, plus the trial's other checks. ``update`` takes one control
    step's observation (pure inputs, so the CPU tests can drive it)."""

    def __init__(self):
        self.step = 0
        self.locked = False
        self.capture = None                     # the running capture
        self.saved = []                         # saved captures, judged
        self.events = []
        self.touch = {}                         # per finger key: the running touch
        self.pressing = {"volume": False, "side": False}
        self.moved = set()
        self.fell = False
        self.off_table = False
        self.z_top = None
        self.still = 0
        self.set_down = False
        self.tier = 0.0
        self.best = None
        self.info = {}

    # -- inputs: pos, R (phone, z above the table), vel, angvel; hulls [(side, link, Hull)] (z above the table);
    #    points (robot points); ends {(side, link): pts}; forces {(side, link): N}
    def update(self, pos, R, vel, angvel, hulls, points, ends, forces) -> None:
        self.step += 1
        t = self.step * DT
        pos, R = np.asarray(pos, float), np.asarray(R, float)
        hand = [h for s, n, h in hulls if n in g.HAND_LINKS]
        all_h = [h for s, n, h in hulls]
        # held: a gripper within 2 mm of the phone's body
        body_c, body_s = pos, np.array([g.PT, g.PW, g.PH])
        hand_gap = hulls_box_gap(hand, body_c, R, body_s)
        held = hand_gap <= 0.002
        # fall and off the table
        z = float(pos[2])
        if held or self.z_top is None:
            self.z_top = z
        else:
            self.z_top = max(self.z_top, z) if self.z_top is not None else z
            if self.z_top - z >= g.FALL:
                if not self.fell:
                    self.events.append((self.step, "fell"))
                self.fell = True
        if not (g.TABLE_X[0] < pos[0] < g.TABLE_X[1] and g.TABLE_Y[0] < pos[1] < g.TABLE_Y[1]) or z < -0.05:
            self.off_table = True
        # rods and boards touched: moved
        for i in range(3):
            if i in self.moved:
                continue
            rx, ry = g.rod_xy(i)
            u = g.bearing_vec(g.BEARINGS[i])
            Rb = g.rot_z(math.atan2(u[1], u[0]))
            rod = hulls_box_gap(all_h, (rx, ry, g.ROD_LEN / 2), Rb, (g.ROD_D, g.ROD_D, g.ROD_LEN))
            blk = hulls_box_gap(all_h, (rx, ry, g.ROD_BLOCK[2] / 2), Rb, g.ROD_BLOCK)
            c = g.board_xy(i)
            brd = hulls_box_gap(all_h, (c[0], c[1], (g.SHEET_Z0 + g.SHEET_H) / 2) + Rb @ np.array([0.003, 0, 0]), Rb,
                                (0.006, g.SHEET_W, g.SHEET_Z0 + g.SHEET_H))
            hld = hulls_box_gap(all_h, np.array([c[0], c[1], g.HOLDER_H / 2]) + Rb @ np.array([0.003, 0, 0]), Rb,
                                (g.HOLDER_D, g.HOLDER_W, g.HOLDER_H))
            if min(rod, blk, brd, hld) <= 0.001:
                self.moved.add(i)
                self.events.append((self.step, f"board {i + 1}'s rod or board touched"))
        # button presses: robot forces on links touching a band
        band_f = {"volume": 0.0, "side": 0.0}
        for key, f in forces.items():
            if f < 0.2:
                continue
            hs = [h for s, n, h in hulls if (s, n) == key]
            for name, (bc, bs) in band_boxes().items():
                if hulls_box_gap(hs, pos + R @ bc, R, bs) <= 0.001:
                    band_f["side" if name == "side" else "volume"] += f
        presses = []
        for b in ("volume", "side"):
            if not self.pressing[b] and band_f[b] >= g.PRESS_N:
                self.pressing[b] = True
                presses.append(b)
            elif self.pressing[b] and band_f[b] < g.PRESS_N / 2:
                self.pressing[b] = False
        # touches and taps
        taps = []
        now = touches(ends, pos, R)
        for key in set(self.touch) | set(now):
            if key in now:
                off = float(np.linalg.norm(now[key] - g.shutter_local()[1:]))
                if key not in self.touch:
                    self.touch[key] = dict(t0=self.step, off0=off, far=off)
                else:
                    tc = self.touch[key]
                    tc["far"] = max(tc["far"], off)
            else:
                tc = self.touch.pop(key)
                dur = (self.step - tc["t0"]) * DT
                ok = dur <= g.TAP_MAX_S + 1e-9 and tc["far"] <= g.TAP_R
                self.events.append((self.step, f"touch {key[0]} {dur:.2f} s, {tc['off0'] * 1000:.1f} mm from the "
                                               f"shutter's centre, {tc['far'] * 1000:.1f} mm at the most"
                                               f"{' -> tap' if ok else ''}"))
                if ok:
                    taps.append(key)
        # the camera
        lens = lens_world(pos, R)
        heading, pitch, roll = camera_angles(R)
        if "side" in presses and not self.locked:
            self.locked = True
            self.events.append((self.step, "side button: locked"))
            if self.capture is not None:
                self.events.append((self.step, "capture lost (locked)"))
                self.capture = None
        trigger = (len(taps) > 0 or "volume" in presses) and not self.locked
        if self.capture is not None:
            cap = self.capture
            c = unwrap_to(heading, cap["frames"][-1]["c"])
            cap["frames"].append(dict(c=c, lens=lens.tolist(), pitch=pitch, roll=roll, step=self.step))
            prog = cap["c0"] - c
            cap["max_prog"] = max(cap["max_prog"], prog)
            k = round(g.SLOW_WINDOW_S / DT)
            if len(cap["frames"]) > k:
                rate = abs(c - cap["frames"][-1 - k]["c"]) / g.SLOW_WINDOW_S
                cap["max_rate"] = max(cap["max_rate"], rate)
                if rate > g.SLOW_DPS and "slow down" not in cap["warn"]:
                    cap["warn"].append("slow down")
            drift = abs(pitch - cap["pitch0"]) + math.degrees(math.atan2(abs(lens[2] - cap["z0"]), g.DRIFT_DIST))
            cap["max_drift"] = max(cap["max_drift"], drift)
            if drift > g.DRIFT_DEG and "off the line" not in cap["warn"]:
                cap["warn"].append("off the line")
            cap["max_tilt"] = max(cap["max_tilt"], abs(pitch))
            cap["max_roll"] = max(cap["max_roll"], abs(roll))
            if abs(pitch) > g.LEVEL_PITCH or abs(roll) > g.LEVEL_ROLL:
                cap["level"] = False
            # hands clear: the camera corner, and robot parts in the central slit
            ch_c, ch_s = corner_box()
            if hulls_box_gap(all_h, pos + R @ ch_c, R, ch_s) <= 0.001:
                if cap["clear"]:
                    self.events.append((self.step, "camera corner touched"))
                cap["clear"] = False
                cap["corner"] = True
            dc = abs(c - cap["frames"][-2]["c"])
            v = in_view_flags(points, lens, R, max(1.0, dc / 2 + 0.5))
            if v["slit"]:
                if cap["clear"]:
                    self.events.append((self.step, "robot in the panorama (slit)"))
                cap["clear"] = False
                cap["seen"] += v["slit"]
            ended, saved = None, False
            if prog < cap["max_prog"] - g.BACK_DEG:
                ended = "turned back"
            elif prog >= g.AUTO_END_DEG:
                ended, saved = "240 degrees", True
            elif trigger:
                ended, saved = "stopped", True
                trigger = False
                v_last = in_view_flags(points, lens, R, 0.0)
                if v_last["right"]:
                    cap["clear"] = False
                    cap["seen"] += v_last["right"]
                    self.events.append((self.step, "robot in the last frame"))
            if ended:
                self.events.append((self.step, f"capture ended: {ended}{' (saved)' if saved else ' (not saved)'}"))
                if saved:
                    ev = evaluate_capture(cap["frames"], self.moved)
                    clean = not cap["warn"]
                    tier, perfect = panorama_tier(ev, clean, cap["level"], cap["clear"])
                    rec = dict(start=cap["frames"][0]["step"], end=self.step, sweep=round(cap["max_prog"], 2),
                               full=ev["full"], aligned=ev["aligned"], err_mm=ev["err_mm"], cover=ev["cover"],
                               warn=list(cap["warn"]), level=cap["level"], clear=cap["clear"],
                               corner=cap["corner"], seen=cap["seen"], max_rate=round(cap["max_rate"], 1),
                               max_drift=round(cap["max_drift"], 2), max_tilt=round(cap["max_tilt"], 2),
                               max_roll=round(cap["max_roll"], 2), tier=tier, perfect=perfect,
                               frames=[(f["step"], round(f["c"], 3), [round(x, 5) for x in f["lens"]],
                                        round(f["pitch"], 3), round(f["roll"], 3)) for f in cap["frames"]])
                    self.saved.append(rec)
                self.capture = None
        if trigger and self.capture is None:
            self.capture = dict(c0=heading, z0=float(lens[2]), pitch0=pitch, max_prog=0.0, max_rate=0.0,
                                max_drift=0.0, max_tilt=abs(pitch), max_roll=abs(roll), warn=[],
                                level=abs(pitch) <= g.LEVEL_PITCH and abs(roll) <= g.LEVEL_ROLL, clear=True,
                                corner=False, seen=0,
                                frames=[dict(c=heading, lens=lens.tolist(), pitch=pitch, roll=roll, step=self.step)])
            self.events.append((self.step, f"capture started at heading {heading:.1f}"))
            v = in_view_flags(points, lens, R, 0.0)
            if v["left"]:
                self.capture["clear"] = False
                self.capture["seen"] += v["left"]
                self.events.append((self.step, "robot in the first frame"))
        # set down: in the stand, still, both grippers 30 mm or more away, for STILL_S
        down = phone_set_down(pos, R) and hand_gap >= g.CLEAR and float(np.linalg.norm(vel)) < 0.005 and \
            float(np.linalg.norm(angvel)) < 0.05
        self.still = self.still + 1 if down else 0
        self.set_down = self.still >= round(g.STILL_S / DT)
        # the score
        best = max(self.saved, key=lambda r: (r["tier"], r["perfect"]), default=None)
        self.best = best
        if self.fell or self.off_table or best is None:
            tier = 0.0
        elif best["perfect"]:
            tier = 1.0 if self.set_down else 0.75
        else:
            tier = best["tier"]
        self.tier = tier
        self.info = dict(heading=round(heading, 2), pitch=round(pitch, 2), roll=round(roll, 2),
                         lens=[round(float(v) * 1000, 1) for v in lens], hand_gap=round(hand_gap * 1000, 1),
                         held=held, capturing=self.capture is not None, band_f={k: round(v, 2) for k, v in band_f.items()},
                         touching={f"{k[0]}:{k[1]}": [round(float(x) * 1000, 1) for x in v] for k, v in now.items()},
                         set_down=self.set_down, locked=self.locked, fell=self.fell)


# -- the simulation's inputs ------------------------------------------------------------------------------------------
def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), np.float64)       # a warp array
    return np.asarray(x, np.float64)


LINKS = ("base", "link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")


def _views(environment) -> dict:
    """A PhysX contact view: the phone against each robot link (both arms)."""
    if not _VIEWS:
        from yam_arena import constants as C
        from yam_arena.robot import asset_link_paths

        psv = environment.unwrapped.sim.physics_sim_view
        ns = "/World/envs/env_*"
        lp = asset_link_paths()
        keys = [(side, n) for side in ("left", "right") for n in LINKS if n in lp]
        arm = {"left": "LeftArm", "right": "RightArm"}
        _VIEWS["phone"] = psv.create_rigid_contact_view(f"{ns}/phone",
                                                        filter_patterns=[f"{ns}/{arm[s]}{lp[n]}" for s, n in keys])
        _VIEWS["keys"] = keys
        _VIEWS["dt"] = float(C.PHYSICS_DT)
    return _VIEWS


def link_forces(environment) -> dict:
    V = _views(environment)
    m = _np(V["phone"].get_contact_force_matrix(V["dt"]))[0]
    return {k: float(np.linalg.norm(m[j])) for j, k in enumerate(V["keys"])}


GRIP_K, GRIP_MAX = 5000.0, 50.0    # yam_arena's finger drive: N/m and its force cap (yam_arena.constants.GRIPPER)


def finger_forces(environment) -> dict:
    """Each fingertip's squeeze from its drive (stiffness x how far short of its target the finger stands, toward
    closing, capped): a closed hand pressing on something between its fingers."""
    out = {}
    try:
        for side in ("left", "right"):
            robot = environment.unwrapped.scene[f"{side}_robot"]
            ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
            q = _np(robot.data.joint_pos)[0, ids]
            qt = _np(robot.data.joint_pos_target)[0, ids]
            f = float(np.mean([min(GRIP_MAX, max(0.0, GRIP_K * (qt[j] - q[j]))) for j in range(2)]))
            out[(side, "tip_left")] = out[(side, "tip_right")] = f
    except Exception as e:                      # the drive's target unreadable: contact forces alone
        out["error"] = str(e)
    return out


def observe(environment) -> dict:
    """One control step's inputs for the monitor, station frame, heights above the table top."""
    from yam_arena.scene import object_states

    from scenekit import robot_link_poses

    st = object_states(environment, ["phone"])["phone"]
    shift = np.array([0.0, 0.0, TOP])
    poses = robot_link_poses(environment)
    for side in poses:
        for n in poses[side]:
            poses[side][n] = poses[side][n].copy()
            poses[side][n][2, 3] -= TOP
    forces = link_forces(environment)
    grip = finger_forces(environment)
    for k, v in grip.items():                   # a press: the larger of the contact force and a finger's squeeze
        if k != "error":
            forces[k] = max(forces.get(k, 0.0), v)
    return dict(pos=np.asarray(st.pos) - shift, R=_rotation(st.quat_xyzw), vel=np.asarray(st.lin_vel),
                angvel=np.asarray(st.ang_vel), hulls=robot_hulls(poses), points=robot_points(poses),
                ends=end_face_points(poses), forces=forces, poses=poses, contact=link_forces(environment),
                grip=grip)


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.clear()
    _STATE["mon"] = Monitor()
    _STATE["step"] = 0
    _VIEWS.clear()


_reset_state()


@on_step
def _monitor(environment) -> None:
    M = _STATE["mon"]
    o = observe(environment)
    M.update(o["pos"], o["R"], o["vel"], o["angvel"], o["hulls"], o["points"], o["ends"], o["forces"])
    _STATE["step"] = M.step
    _STATE["obs"] = o
    if _DEBUG and M.step % 20 == 0:
        print(f"[pano-forces] step {M.step} contact " + str({f"{k[0]}:{k[1]}": round(v, 2) for k, v in
                                                              o["contact"].items() if v > 0.05})
              + " grip " + str({str(k): (round(v, 2) if not isinstance(v, str) else v) for k, v in o["grip"].items()}),
              flush=True)
        print(f"[pano] step {M.step} {M.info} tier {M.tier} saved {[{k: r[k] for k in ('full', 'aligned', 'err_mm', 'warn', 'level', 'clear', 'tier')} for r in M.saved]}",
              flush=True)


def success(objects) -> bool:
    """Never: the trial runs to its step limit and is scored on its end state."""
    return False


def score(objects) -> float:
    """From the Camera rule's saved panoramas and the end state, read every control step (20 Hz).

    A tap is a taped fingertip end face within 0.5 mm of the glass for 0.5 s or less, the touch staying within 6 mm
    of the on-screen shutter's centre (a hand's two fingertips are one touch). A press is 3 N or more from
    the robot on a button band (volume: the -y edge 33.7-45.0 and 47.7-59.0 mm from the top; side button: the +y
    edge 37.4-55.2 mm). A tap or a volume press starts a capture and the next one ends and saves it (or a 240-degree
    turn does); a side-button press locks the phone and loses a running capture; turning back more than 5 degrees ends
    it unsaved; a capture running at the end is lost.
    During a capture: "slow down" (the heading turning faster than 40 deg/s over 0.25 s) and "off the line" (the
    view's pitch change plus the lens's height change seen at 0.45 m over 5 degrees) are warnings; level means the
    optical axis within 3 degrees of horizontal and the roll within 4 at every step.
    A saved capture covers from its first frame's left edge to its last frame's right edge (frames 52 x 60 degrees).
    A board is in full if its sheet lies inside that and inside the height band all frames share. Its line is judged
    from the frame that puts it in the panorama (the first frame, a frame whose centre crosses it, or the last frame):
    aligned if the lens is there within 6 mm of the board's sight line (its sideways offset only) and the robot never
    touched its rod, block, sheet or holder. Hands clear: no robot part touched the camera corner during the capture,
    and no point of the arms' collision hulls within 1.5 m was in a step's central strip, the first frame's left half
    or the last frame's right half.
    0 if the phone fell 50 mm or more with no gripper on it, or left the table. Otherwise, from the best saved
    panorama: 1.0 if all three boards are in full and aligned, with no warning, level and hands clear, and the phone
    is set down (back in the stand's slot, screen to +x, still for 1 s, both grippers 30 mm or more from it, at the
    end); 0.75 for that panorama without the set-down; 0.5 for all three boards in full otherwise; 0.25 for exactly
    two; 0 otherwise."""
    return float(_STATE["mon"].tier) if "mon" in _STATE else 0.0
