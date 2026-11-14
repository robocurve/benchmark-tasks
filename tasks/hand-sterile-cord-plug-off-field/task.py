"""Operating-room bench (no patient) on the station table. On the left half of the table stands a raised sterile
field: a 5 cm tall box covered with a blue drape (44 x 48 cm), whose right-hand edge runs along the table's midline
(y = 0) in front of the robot. A white sterile device cord (8 mm, 40 cm) leaves a white device holster fixed near the
field's front-right corner and lies straight across the drape, diagonally away from the robot and to the left; it
ends in a black 125 x 16 x 14 mm connector (plug body with its moulded strain-relief grip). A grey cord holder is
fixed on a line running from the field edge diagonally (-60 deg) towards a grey device console on the right half of
the table: an open-top slot on the drape (14 mm wide, 30 mm walls, 8.5-12.5 cm in from the edge along the line) and a
flat hand-off tray at drape height that reaches 17.5 cm beyond the edge. The console's jack (a 22 x 19 mm opening,
30 mm deep) faces the holder, 18 cm beyond the field edge along the line.

The left arm is the scrubbed (sterile) team member: its gripper centre must stay on the field side of the edge, and its
gripper (housing and both fingers, as collision-hull points) is its sterile glove: once any part of it touches a
non-sterile surface (off the field at or below drape height, the console, the right arm, non-sterile cord) it must not
touch the drape, the holder slot, the holster or sterile cord again. The
right arm is the non-sterile circulator: no part of it (every collision hull point of every link, base to fingertips)
may enter the volume above the field. Anything of the cord that has touched a non-sterile surface (off the field at or
below drape height, the console, or the right arm) must never come back over the field. The left arm lays the cord
in the slot and leaves the connector on the tray, pointing off the field; the right arm takes it from the far end of
the tray, draws it further out, re-grips it and pushes it fully into the console jack. The connector must never
touch the table; at the end both arms let go.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score \\
           --instruction "<INSTRUCTION below>"
Test-only demo variants (env var STERILE_CHEAT): nothing | drop | scrub_carry | noclip | hold | hold_cord | reach_over |
lean_over | drag_back | glove_touch; tier03 stops with the connector on the tray (partial-credit check, expect 0.3).
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import cablekit  # noqa: E402,F401  (hooks yam_arena so the Cable below is simulated)
import numpy as np  # noqa: E402
from cablekit import Plug, cable  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = ("Plug the sterile white cord into the grey console's jack, with the cord lying in the grey holder's slot. The "
               "left arm and the blue drape are sterile, the right arm is not. Let go with both arms at the end.")

# -- colours ----------------------------------------------------------------------------------------------------------
DRAPE_BLUE = (0.16, 0.42, 0.68)
WHITE = (0.92, 0.92, 0.90)
HOLDER_GREY = (0.62, 0.64, 0.66)
CONSOLE_GREY = (0.78, 0.79, 0.80)
JACK_DARK = (0.22, 0.23, 0.25)
PLUG_BLACK = (0.07, 0.07, 0.08)

# -- sterile field: a draped raised box, right edge on y = 0 ------------------------------------------------------------
FIELD_H = 0.05
FT = TOP + FIELD_H  # drape top (sterile surface)
FIELD_X0, FIELD_X1 = 0.08, 0.52
FIELD_Y0, FIELD_Y1 = 0.0, 0.48  # y0 = the field's right edge (the boundary between the two arms' zones)
FIELD = Box("field", pos=((FIELD_X0 + FIELD_X1) / 2, (FIELD_Y0 + FIELD_Y1) / 2, TOP + FIELD_H / 2),
            size=(FIELD_X1 - FIELD_X0, FIELD_Y1 - FIELD_Y0, FIELD_H), color=DRAPE_BLUE)

# -- hand-off axis: holder slot -> tray -> jack, all on one line through E at heading -60 deg ----------------------------
AX_YAW = math.radians(-60.0)
U = np.array([math.cos(AX_YAW), math.sin(AX_YAW)])  # along the axis, off the field towards the console
NRM = np.array([-math.sin(AX_YAW), math.cos(AX_YAW)])  # sideways (left of the axis)
E = np.array([0.30, 0.0])  # where the axis crosses the field edge


def ax(d: float, b: float = 0.0) -> np.ndarray:
    """World xy of the point d m along the hand-off axis from the field edge (positive = off the field), b sideways."""
    return E + U * d + NRM * b


def ax_coords(xy) -> tuple[float, float]:
    v = np.asarray(xy[:2], dtype=float) - E
    return float(v @ U), float(v @ NRM)


def _abox(name, d0, d1, b, w, z0, z1, color):
    """Box aligned with the hand-off axis: from d0 to d1 along it, centred b sideways, w wide, z0..z1 high."""
    c = ax((d0 + d1) / 2, b)
    return Box(name, pos=(float(c[0]), float(c[1]), (z0 + z1) / 2), size=(d1 - d0, w, z1 - z0), yaw=AX_YAW,
               color=color)


# cord holder: open-top U-slot on the drape + flat hand-off tray sticking out beyond the edge
SLOT_D0, SLOT_D1 = -0.125, -0.085  # slot extent along the axis (on the field)
SLOT_W, SLOT_WALL, SLOT_H = 0.014, 0.004, 0.030
LEDGE_D0, LEDGE_D1, LEDGE_W, LEDGE_T = -0.025, 0.175, 0.030, 0.010
HOLDER = [
    _abox("slot_wall_l", SLOT_D0, SLOT_D1, (SLOT_W + SLOT_WALL) / 2, SLOT_WALL, FT, FT + SLOT_H, HOLDER_GREY),
    _abox("slot_wall_r", SLOT_D0, SLOT_D1, -(SLOT_W + SLOT_WALL) / 2, SLOT_WALL, FT, FT + SLOT_H, HOLDER_GREY),
    _abox("ledge", LEDGE_D0, LEDGE_D1, 0.0, LEDGE_W, FT - LEDGE_T, FT, HOLDER_GREY),
]

# console with a jack facing the holder
PL, PW, PH = 0.125, 0.016, 0.014  # connector: length along the cord, width, height
CAV_W, CAV_H, CAV_D = PW + 0.006, PH + 0.005, 0.030
JACK_D = 0.18  # jack face, m along the axis from the field edge
SLEEVE_LEN, SLEEVE_T = 0.040, 0.004
JACK_ZC = FT + PH / 2 + 0.002  # cavity centre height (= plug centre carried 2 mm above the tray)
CAV_Z0 = JACK_ZC - CAV_H / 2
CONSOLE_LEN, CONSOLE_W, CONSOLE_TOP = 0.12, 0.16, FT + 0.025
_SW = CAV_W + 2 * SLEEVE_T
CONSOLE = [
    _abox("console_body", JACK_D + SLEEVE_LEN, JACK_D + SLEEVE_LEN + CONSOLE_LEN, 0.0, CONSOLE_W, TOP, CONSOLE_TOP,
          CONSOLE_GREY),
    _abox("jack_base", JACK_D, JACK_D + SLEEVE_LEN, 0.0, _SW, TOP, CAV_Z0, CONSOLE_GREY),
    _abox("jack_wall_l", JACK_D, JACK_D + CAV_D, (CAV_W + SLEEVE_T) / 2, SLEEVE_T, CAV_Z0, CAV_Z0 + CAV_H, JACK_DARK),
    _abox("jack_wall_r", JACK_D, JACK_D + CAV_D, -(CAV_W + SLEEVE_T) / 2, SLEEVE_T, CAV_Z0, CAV_Z0 + CAV_H, JACK_DARK),
    _abox("jack_roof", JACK_D, JACK_D + CAV_D, 0.0, _SW, CAV_Z0 + CAV_H, CAV_Z0 + CAV_H + SLEEVE_T, JACK_DARK),
    _abox("jack_back", JACK_D + CAV_D, JACK_D + SLEEVE_LEN, 0.0, _SW, CAV_Z0, CAV_Z0 + CAV_H + SLEEVE_T, JACK_DARK),
]
# console + jack sleeve footprint in axis coordinates (d0, d1, half-width, top): non-sterile equipment
CONSOLE_BOX = (JACK_D, JACK_D + SLEEVE_LEN + CONSOLE_LEN, CONSOLE_W / 2, CONSOLE_TOP)

# sterile device cord: device end fixed in a holster on the field, connector on the free end
R = 0.004  # 8 mm cord
CORD_LEN = 0.40
A = np.array([0.14, 0.06])  # cord leaves the holster here
CORD_YAW = math.radians(50.0)
_DIR = np.array([math.cos(CORD_YAW), math.sin(CORD_YAW)])
PLUG = Plug(size=(PL, PW, PH), mass=0.025, color=PLUG_BLACK)
CORD = cable("cord", start=(float(A[0]), float(A[1]), FT + PH / 2 + 0.0005), yaw=CORD_YAW, length=CORD_LEN, radius=R,
             anchor_start=True, end_plug=PLUG, color=WHITE, cable_mass=0.054, bend_stiffness=0.005, bend_damping=0.001)
HOLSTER_SIZE = (0.050, 0.035, 0.030)
_hc = A - _DIR * (0.006 + HOLSTER_SIZE[0] / 2)
HOLSTER = Box("holster", pos=(float(_hc[0]), float(_hc[1]), FT + HOLSTER_SIZE[2] / 2), size=HOLSTER_SIZE,
              yaw=CORD_YAW, color=WHITE)

OBJECTS = [FIELD, *HOLDER, *CONSOLE, HOLSTER, CORD]
TWIN_EXTRA: list = []  # everything is in OBJECTS (the cord's device end is glued into the holster)

# -- robot geometry: collision hull points of every link, read once from the arm's USD asset -----------------------------
_HULLS: dict = {}


def _load_hulls() -> dict:
    """{body name: (N, 3) points in that body's frame}: the vertices of the convex collision pieces of every rigid body
    of the YAM arm (base, link1-5, gripper, both fingers), thinned to one point per 4 mm cell (keeps the outline)."""
    if _HULLS:
        return _HULLS
    import yam_arena
    from pxr import Gf, Usd, UsdGeom
    from scipy.spatial import ConvexHull

    path = pathlib.Path(yam_arena.__file__).parent / "assets/yam/usd/yam_arm/yam_arm.usda"
    stage = Usd.Stage.Open(str(path))
    xc = UsdGeom.XformCache()
    bodies = [p for p in stage.Traverse() if "PhysicsRigidBodyAPI" in p.GetAppliedSchemas()]
    for b in bodies:
        inv = xc.GetLocalToWorldTransform(b).GetInverse()
        pieces = []
        for p in Usd.PrimRange(b, Usd.TraverseInstanceProxies()):
            q, other = p, False
            while q != b:
                if "PhysicsRigidBodyAPI" in q.GetAppliedSchemas():
                    other = True
                    break
                q = q.GetParent()
            if other or not p.IsA(UsdGeom.Mesh) or "PhysicsCollisionAPI" not in p.GetAppliedSchemas():
                continue
            m = xc.GetLocalToWorldTransform(p) * inv
            v = np.array([list(m.Transform(Gf.Vec3d(*map(float, x)))) for x in UsdGeom.Mesh(p).GetPointsAttr().Get()])
            try:
                v = v[ConvexHull(v).vertices]
            except Exception:  # noqa: BLE001  (flat piece: keep all its points)
                pass
            pieces.append(v)
        if pieces:
            v = np.vstack(pieces)
            keep = np.unique(np.round(v / 0.004).astype(int), axis=0, return_index=True)[1]
            _HULLS[b.GetName()] = v[np.sort(keep)]
    if len(_HULLS) < 9:
        raise RuntimeError(f"arm hulls: only {sorted(_HULLS)}")
    return _HULLS


def _quat_wxyz(q) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _t(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    return (x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)).astype(np.float64)


def arm_points(key: str) -> dict:
    """{link name: (N, 3) world points (env-origin corrected)} of the collision hulls of one arm ("left_robot" or
    "right_robot"), from the sim's live body poses. Raises when the sim is not available."""
    env = _ENV["env"]
    hulls = _load_hulls()
    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0]
    art = base.scene[key]
    names = list(art.body_names)
    pos = _t(art.data.body_pos_w)[0] - origin
    quat = _t(art.data.body_quat_w)[0]
    # quaternion layout check, every call: the gripper body's frame must put the TCP (134 mm along its z axis)
    # where the embodiment reports it; whichever layout (wxyz or xyzw) does so is used, neither = error
    from yam_arena.runtime import tcp_world

    tcp = np.asarray(tcp_world(art), dtype=np.float64) - origin
    gi = names.index("gripper")
    rot = None
    for conv in (lambda q: q, lambda q: np.r_[q[3], q[:3]]):
        if np.linalg.norm(pos[gi] + _quat_wxyz(conv(quat[gi]))[:, 2] * 0.134 - tcp) < 0.004:
            rot = conv
            break
    if rot is None:
        raise RuntimeError("arm body frames do not match the reported TCP")
    out = {}
    for n, P in hulls.items():
        i = names.index(n)  # a missing body raises: the rule could not be checked
        out[n] = P @ _quat_wxyz(rot(quat[i])).T + pos[i]
    return out


def in_field(p, margin: float = 0.0) -> np.ndarray:
    """Points over the drape's footprint, at any height above the table (the sterile field volume): x within the
    drape, y > FIELD_Y0 + margin."""
    p = np.atleast_2d(p)
    return (p[:, 0] >= FIELD_X0) & (p[:, 0] <= FIELD_X1) & (p[:, 1] > FIELD_Y0 + margin) & (p[:, 1] <= FIELD_Y1) & (
        p[:, 2] >= TOP)


# -- per-step monitor: arm zones, non-sterile arm geometry, cord contamination, plug on the table (latched) ----------------
TABLE_TOUCH = 0.002  # plug's lowest point within 2 mm of the table top = touching it
TAKE_DIST = 0.03  # right gripper centre this close to the plug centre = taking it
CONTACT_GAP = 0.010  # a right-arm hull point this close to the cord surface / plug side = touching it
LOW_GAP = 0.003  # a cord/plug point whose underside is within 3 mm of drape height (or lower) off the field = touching
RETURN_MARGIN = 0.005  # a non-sterile cord point that comes 5 mm or more over the field edge = brought back
_ENV: dict = {"env": None}
_MON: dict = {}
_CORD_SUB = 4  # cord sample points per segment gap
_PLUG_SAMPLES = np.linspace(-0.5, 0.5, 33)  # along the plug (every 3.9 mm), as fractions of its length
GLOVE = ("gripper", "tip_left", "tip_right")  # the scrub arm's "gloved hand": gripper housing and both fingers
GLOVE_GAP = 0.003  # a glove hull point this close to a surface (or below it) = touching it
PLUG_REACH = PW / 2 + GLOVE_GAP  # a glove point within this of the connector's axis (and along its length) touches it
CORD_REACH = R + GLOVE_GAP + 0.0025  # ... within this of a cord sample point (samples 5 mm apart) touches the cord
STERILE_KIT = [HOLSTER, HOLDER[0], HOLDER[1]]  # sterile equipment standing on the drape (slot walls, holster)


def _reset_monitor() -> None:
    _MON.update(left_out=False, right_in=False, right_link=None, plug_table=False, taken_on_field=False,
                right_reached=False, dirty=None, dirty_back=False, err=False, events=[], steps=0, min_left_y=9.0,
                max_right_field_y=-9.0, min_plug_low=9.0, n_dirty=0, glove_dirty=False, glove_dirty_why=None,
                glove_dirty_touch=False, min_glove_off_z=9.0)


_reset_monitor()


def grippers() -> list[np.ndarray]:
    """World TCPs (between the finger pads, 13 mm above the pad tips) of the left and right gripper; [] offline."""
    env = _ENV["env"]
    if env is None:
        return []
    from yam_arena.runtime import tcp_world

    base = env.unwrapped
    origin = _t(base.scene.env_origins)[0]
    return [np.asarray(tcp_world(base.scene[k]), dtype=np.float64) - origin for k in ("left_robot", "right_robot")]


def plug_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    """(plug centre, unit axis from the cord through the plug tip)."""
    c = objects[CORD.name]
    p = np.asarray(c.end_plug, dtype=float)
    a = p - np.asarray(c.points[-1], dtype=float)
    return p, a / (np.linalg.norm(a) + 1e-12)


def plug_low(objects) -> float:
    """Height of the plug's lowest point (m, world z), allowing for its tilt."""
    p, a = plug_pose(objects)
    az = abs(float(a[2]))
    return float(p[2] - az * PL / 2 - math.hypot(PW / 2, PH / 2) * math.sqrt(max(0.0, 1 - az * az)))


def cord_samples(objects) -> tuple[np.ndarray, np.ndarray]:
    """Material points of the cord and connector, in a fixed order (index i is always the same piece of cord):
    (N, 3) centres and (N,) half-thickness below the centre. Cord: segment centres and 3 points between each pair;
    connector: 33 points along its axis."""
    c = objects[CORD.name]
    pts = np.asarray(c.points, dtype=float)
    f = np.arange(_CORD_SUB) / _CORD_SUB
    cord = (pts[:-1, None, :] * (1 - f)[None, :, None] + pts[1:, None, :] * f[None, :, None]).reshape(-1, 3)
    cord = np.vstack([cord, pts[-1:]])
    p, a = plug_pose(objects)
    plug = p[None, :] + _PLUG_SAMPLES[:, None] * PL * a[None, :]
    half = np.r_[np.full(len(cord), R), np.full(len(plug), PH / 2)]
    return np.vstack([cord, plug]), half


def _on_console(p) -> np.ndarray:
    d0, d1, hw, top = CONSOLE_BOX
    v = p[:, :2] - E
    d, b = v @ U, v @ NRM
    return (d >= d0 - 0.005) & (d <= d1 + 0.005) & (np.abs(b) <= hw + 0.005) & (p[:, 2] <= top + 0.012)


def _box_gap(P, box) -> np.ndarray:
    """Distance (m) from each point to a (yawed) Box; 0 inside it."""
    v = np.atleast_2d(P) - np.asarray(box.pos, dtype=float)
    c, s_ = math.cos(box.yaw), math.sin(box.yaw)
    loc = np.c_[v[:, 0] * c + v[:, 1] * s_, -v[:, 0] * s_ + v[:, 1] * c, v[:, 2]]
    return np.linalg.norm(np.maximum(np.abs(loc) - np.asarray(box.size) / 2, 0.0), axis=1)


def glove_touched_samples(G, objects) -> np.ndarray:
    """Indices (into cord_samples) of the cord / connector material the glove points G touch: cord sample points
    within CORD_REACH of a glove point; for the connector, glove points within PLUG_REACH of its axis and along its
    length (+ GLOVE_GAP) touch the connector sample nearest to them along the axis."""
    s, half = cord_samples(objects)
    nc = int((half == R).sum())
    d = np.sqrt(((G[:, None, :] - s[None, :nc, :]) ** 2).sum(-1))
    idx = set(np.nonzero((d <= CORD_REACH).any(axis=0))[0].tolist())
    p, a = plug_pose(objects)
    v = G - p
    t = v @ a
    rad = np.linalg.norm(v - t[:, None] * a[None, :], axis=1)
    on = (np.abs(t) <= PL / 2 + GLOVE_GAP) & (rad <= PLUG_REACH)
    if on.any():
        k = np.clip(np.rint((t[on] / PL + 0.5) * (len(_PLUG_SAMPLES) - 1)), 0, len(_PLUG_SAMPLES) - 1).astype(int)
        idx |= set((nc + k).tolist())
    return np.array(sorted(idx), dtype=int)


def _glove(objects, allr) -> None:
    """Sterility of the scrub arm's gripper (its 'glove'), from its collision-hull points (housing and both fingers).
    It becomes non-sterile, and stays so, when any glove point: is off the drape's footprint at or below drape height
    + 3 mm (the table beside the field, the drape's sides, the tray beyond the edge); is within 3 mm of the console or
    jack; is within 1 cm of a right-arm hull point; or touches cord / connector material that is non-sterile. Once it
    is non-sterile it must not touch the drape (a point over the footprint at or below drape height + 3 mm), the slot
    walls or holster (within 3 mm), or any sterile cord / connector material."""
    lp = arm_points("left_robot")
    G = np.vstack([lp[n] for n in GLOVE])
    over = (G[:, 0] >= FIELD_X0) & (G[:, 0] <= FIELD_X1) & (G[:, 1] >= FIELD_Y0) & (G[:, 1] <= FIELD_Y1)
    low = G[:, 2] <= FT + GLOVE_GAP
    if (~over).any():
        _MON["min_glove_off_z"] = min(_MON["min_glove_off_z"], float(G[~over, 2].min() - FT))
    touched = glove_touched_samples(G, objects)
    dirty = _MON["dirty"]
    why = []
    if (~over & low).any():
        why.append("a surface off the field at or below drape height")
    if any((_box_gap(G, b) <= GLOVE_GAP).any() for b in CONSOLE):
        why.append("the console")
    lo, hi = G.min(axis=0) - CONTACT_GAP, G.max(axis=0) + CONTACT_GAP
    cand = allr[np.all((allr >= lo) & (allr <= hi), axis=1)]
    if len(cand) and np.sqrt(((G[:, None, :] - cand[None, :, :]) ** 2).sum(-1)).min() <= CONTACT_GAP:
        why.append("the right arm")
    if len(touched) and dirty[touched].any():
        why.append("non-sterile cord/connector")
    sterile = []
    if (over & low).any():
        sterile.append("the drape")
    if any((_box_gap(G, b) <= GLOVE_GAP).any() for b in STERILE_KIT):
        sterile.append("the holder slot / holster")
    if len(touched) and (~dirty[touched]).any():
        sterile.append("sterile cord/connector")
    if _MON["glove_dirty"] and sterile and not _MON["glove_dirty_touch"]:
        _MON["glove_dirty_touch"] = True
        _MON["events"].append((f"non-sterile left gripper touched {', '.join(sterile)}", _MON["steps"]))
        print(f"[sterile] step {_MON['steps']}: non-sterile left gripper touched {sterile}", flush=True)
    if why and not _MON["glove_dirty"]:
        _MON["glove_dirty"], _MON["glove_dirty_why"] = True, why
        _MON["events"].append((f"left gripper made non-sterile by {', '.join(why)}", _MON["steps"]))
        print(f"[sterile] step {_MON['steps']}: left gripper made non-sterile by {why}", flush=True)


def _monitor(objects) -> None:
    g = grippers()
    if len(g) != 2:
        raise RuntimeError("gripper poses unavailable")
    _MON["min_left_y"] = min(_MON["min_left_y"], float(g[0][1]))
    if g[0][1] < FIELD_Y0 and not _MON["left_out"]:
        _MON["left_out"] = True
        _MON["events"].append(("left gripper left the field", _MON["steps"]))
    # non-sterile arm: every collision hull point of every link must stay out of the field volume
    rpts = arm_points("right_robot")
    allr = np.vstack(list(rpts.values()))
    if _MON["steps"] % 200 == 0:
        print(f"[sterile] step {_MON['steps']}: right arm hull max y {float(allr[:, 1].max()):.3f}", flush=True)
    over = in_field(allr)
    sel = (allr[:, 0] >= FIELD_X0) & (allr[:, 0] <= FIELD_X1)
    if sel.any():
        _MON["max_right_field_y"] = max(_MON["max_right_field_y"], float(allr[sel, 1].max()))
    if over.any() and not _MON["right_in"]:
        k = 0
        for n, P in rpts.items():
            if in_field(P).any():
                _MON["right_link"] = n
                break
            k += 1
        _MON["right_in"] = True
        _MON["events"].append((f"right arm ({_MON['right_link']}) over the field", _MON["steps"]))
    p, _ = plug_pose(objects)
    if not _MON["right_reached"] and np.linalg.norm(g[1] - p) <= TAKE_DIST:
        _MON["right_reached"] = True  # first time the right gripper is at the plug: it must be off the field
        if p[1] >= FIELD_Y0:
            _MON["taken_on_field"] = True
            _MON["events"].append(("right gripper took the plug over the field", _MON["steps"]))
    # cord contamination: off the field at/below drape height, on the console, or touched by the right arm
    s, half = cord_samples(objects)
    if _MON["dirty"] is None or len(_MON["dirty"]) != len(s):
        _MON["dirty"] = np.zeros(len(s), dtype=bool)
    off = ~((s[:, 0] >= FIELD_X0) & (s[:, 0] <= FIELD_X1) & (s[:, 1] >= FIELD_Y0) & (s[:, 1] <= FIELD_Y1))
    low = s[:, 2] - half <= FT + LOW_GAP
    near = np.zeros(len(s), dtype=bool)
    box_lo, box_hi = s.min(axis=0) - 0.05, s.max(axis=0) + 0.05
    cand = allr[np.all((allr >= box_lo) & (allr <= box_hi), axis=1)]
    if len(cand):
        dmin = np.sqrt(((s[:, None, :] - cand[None, :, :]) ** 2).sum(-1)).min(axis=1)
        reach = np.r_[np.full(int((half == R).sum()), R), np.full(int((half != R).sum()), PW / 2)] + CONTACT_GAP
        near = dmin <= reach
    new = (off & low) | _on_console(s) | near
    _MON["dirty"] |= new
    _MON["n_dirty"] = int(_MON["dirty"].sum())
    if not _MON["dirty_back"] and (_MON["dirty"] & in_field(s, RETURN_MARGIN)).any():
        _MON["dirty_back"] = True
        _MON["events"].append(("non-sterile part of the cord brought back over the field", _MON["steps"]))
    _glove(objects, allr)
    lowp = plug_low(objects)
    _MON["min_plug_low"] = min(_MON["min_plug_low"], lowp)
    if lowp <= TOP + TABLE_TOUCH and not _MON["plug_table"]:
        _MON["plug_table"] = True
        _MON["events"].append(("plug touched the table", _MON["steps"]))
    _MON["steps"] += 1


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # pragma: no cover  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls._task_step, "_sterile_hooked", False):
        return
    base_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        try:
            from yam_arena.scene import object_states

            _monitor(object_states(self._env, [CORD.name]))
        except Exception as e:  # pragma: no cover
            _MON["err"] = True  # a step the rules could not be checked on never passes
            print(f"[sterile] monitor error: {e!r}", flush=True)
        return base_step(self, *a, **k)

    def reset(self, *a, **k):
        out = base_reset(self, *a, **k)
        _reset_monitor()  # arms start at READY (left over the field, right off it), plug on the drape
        return out

    _task_step._sterile_hooked = True
    reset._sterile_hooked = True
    cls._task_step = _task_step
    cls.reset = reset


_install_hooks()

# -- scoring ----------------------------------------------------------------------------------------------------------
SEATED_MIN = 0.015  # plug tip at least 15 mm inside the jack
RELEASE_PLUG = 0.045  # let go = both gripper TCPs at least this far from the plug's centre line, end to end,
RELEASE_CORD = 0.03  # and at least this far from every point of the cord


def violations() -> list[str]:
    out = []
    if _MON["err"] or _MON["steps"] == 0:
        out.append("monitor_error")  # no step was checked, or one could not be: never a pass
    if _MON["left_out"]:
        out.append("left_out")
    if _MON["right_in"]:
        out.append("right_in")
    if _MON["plug_table"]:
        out.append("plug_table")
    if _MON["taken_on_field"]:
        out.append("taken_on_field")
    if _MON["dirty_back"]:
        out.append("dirty_back")
    if _MON["glove_dirty_touch"]:
        out.append("dirty_glove_touch")
    return out


def jack_depth(objects) -> float:
    """How far (m) the plug tip is inside the jack; -1 if the plug is not lined up with the opening (centre > 8 mm
    off the jack axis sideways or vertically, or pointing more than ~25 deg away from it)."""
    p, a = plug_pose(objects)
    if not np.isfinite(p).all():
        return -1.0
    d, b = ax_coords(p)
    if abs(b) > 0.008 or abs(p[2] - JACK_ZC) > 0.008 or float(a[:2] @ U) < 0.9:
        return -1.0
    return float(d + PL / 2 - JACK_D)


def cord_in_slot(objects) -> bool:
    """Some cord segment centre lies inside the holder slot (between its walls, below their tops)."""
    for q in objects[CORD.name].points:
        d, b = ax_coords(q)
        if SLOT_D0 - 0.002 <= d <= SLOT_D1 + 0.002 and abs(b) <= SLOT_W / 2 and FT <= q[2] <= FT + SLOT_H:
            return True
    return False


def plug_on_ledge(objects) -> bool:
    """Plug lying on the tray with its centre beyond the field edge."""
    p, _ = plug_pose(objects)
    d, b = ax_coords(p)
    return bool(p[1] < FIELD_Y0 and LEDGE_D0 <= d <= LEDGE_D1 and abs(b) <= LEDGE_W / 2
                and p[2] <= FT + PH / 2 + 0.004)


def _seg_dist(p, a, b) -> float:
    ab = b - a
    t = float(np.clip(np.dot(p - a, ab) / (np.dot(ab, ab) + 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def released(objects) -> bool:
    """Every gripper centre at least 4.5 cm from the plug's centre line and 3 cm from the cord; False when the gripper
    poses cannot be read."""
    g = grippers()
    if len(g) != 2:
        return False
    p, a = plug_pose(objects)
    rear, tip = p - a * PL / 2, p + a * PL / 2
    poly = np.asarray(objects[CORD.name].polyline(), dtype=float)
    for x in g:
        if _seg_dist(x, rear, tip) < RELEASE_PLUG:
            return False
        if min(_seg_dist(x, u, v) for u, v in zip(poly[:-1], poly[1:])) < RELEASE_CORD:
            return False
    return True


def score(objects) -> float:
    """0 if, at any step: the left gripper centre crossed to the right of the field edge (y < 0); the left gripper,
    after any of its hull points touched something non-sterile, touched the drape, slot, holster or sterile cord
    (see _glove); any collision hull
    point of any right-arm link (base to fingertips) entered the volume above the drape; the plug touched the table;
    the plug centre was still over the field when the right gripper first came within 3 cm of it; or a piece of cord
    or plug that had become non-sterile (off the field with its underside at or below drape height + 3 mm, on the
    console, or within 1 cm of a right-arm hull point) later came 5 mm or more over the field. These rules are latched
    in _MON and only reset() clears them. Also 0 if at the end the cord is not in the holder slot or a gripper is
    within 45 mm of the plug's centre line or 3 cm of the cord. Otherwise 1.0 if the plug tip is >= 15 mm inside the
    jack (lined up), 0.3 if the plug lies on the tray with its centre off the field, else 0."""
    if violations() or not cord_in_slot(objects) or not released(objects):
        return 0.0
    if jack_depth(objects) >= SEATED_MIN:
        return 1.0
    return 0.3 if plug_on_ledge(objects) else 0.0


def success(objects) -> bool:
    return score(objects) >= 1.0 and objects[CORD.name].resting(max_speed=0.02)


def distance(objects) -> float:
    """Plug tip's remaining distance to 15 mm inside the jack (m)."""
    p, a = plug_pose(objects)
    tip = p[:2] + a[:2] * PL / 2
    goal = ax(JACK_D + SEATED_MIN)
    return float(np.linalg.norm(tip - goal))


def progress(objects) -> dict:
    """Stage diagnostic (not the score)."""
    p, _ = plug_pose(objects)
    return {
        "cord_in_slot": int(cord_in_slot(objects)),
        "plug_off_field": int(p[1] < FIELD_Y0),
        "plug_in_jack": round(float(np.clip(jack_depth(objects) / SEATED_MIN, 0.0, 1.0)), 3),
        "released": int(released(objects)),
        "zones_kept": int(not (_MON["left_out"] or _MON["right_in"] or _MON["taken_on_field"])),
        "no_contaminated_return": int(not _MON["dirty_back"]),
        "left_gripper_kept_sterile": int(not _MON["glove_dirty_touch"]),
        "plug_never_on_table": int(not _MON["plug_table"]),
    }


VARIATION = {
    "cable:cord": {"dx": 0.015, "dy": 0.015, "yaw": 0.10},
}

# -- closed-loop scripted demo (cabledemo.py): camera images + the arms' own poses + the fixed layout only -------------
SCRUB_GRIP_BACK = 0.004  # left arm pinches the plug this far from its cord end (pads reach 13 mm either side)
SCRUB_TCP_D = -0.0175  # left TCP when it sets the plug down: its pads end 4 mm inside the edge, on sterile connector
PULL_GRIP = 0.010  # right arm first pinches the plug this far from its tip ...
PULL_DIST = 0.050  # ... and draws it this far further out along the tray
SEAT_GRIP = 0.045  # then re-grips it this far from its tip for the push into the jack
SEAT_DEPTH = 0.024
CAM_Z = FT + PH  # the plug's top face (plane used to back-project its pixels)
CAM_BIAS = np.zeros(2)  # constant camera offset (filled after the offline check against the sim)


def _hd(v) -> float:
    return math.atan2(float(v[1]), float(v[0]))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def see_plug(ctx, near_xy, toward_xy, radius: float = 0.12, on_axis: bool = False):
    """Camera estimate of the black connector lying flat near ``near_xy``: (centre xy, unit axis xy pointing to the
    end nearer ``toward_xy`` (its tip), length seen). Uses only the top camera image: dark pixels
    back-projected to the plug's top-face plane, kept within ``radius`` of ``near_xy``; ends from the 3rd/97th
    percentiles along the main axis."""
    import vision

    img = vision.top_image(ctx)
    if img.shape[1] < 160:  # debug runs without recording render 64x36 frames: too coarse, use the layout instead
        return None
    mask = (img.max(axis=-1) < 45) & (img.max(axis=-1) - img.min(axis=-1) < 25)
    cam = vision.top_camera(img.shape)
    pts = cam.to_plane(mask, CAM_Z)
    if len(pts) == 0:
        return None
    pts = pts[np.linalg.norm(pts - np.asarray(near_xy)[:2], axis=1) <= radius]
    if on_axis:  # on the tray: keep the strip along the hand-off line, short of the (dark) jack sleeve
        v = pts - E
        pts = pts[(np.abs(v @ NRM) <= 0.03) & (v @ U <= JACK_D - 0.006)]
    if len(pts) < 10:
        return None
    c = pts.mean(axis=0)
    w, v = np.linalg.eigh(np.cov((pts - c).T))
    a = v[:, -1]
    s = (pts - c) @ a
    lo, hi = np.percentile(s, 3), np.percentile(s, 97)
    centre = c + a * (lo + hi) / 2 + CAM_BIAS
    if float((np.asarray(toward_xy)[:2] - centre) @ a) < 0:
        a = -a
    return centre, a, float(hi - lo)


def demo_script(ctx):
    """Left (scrub) arm looks for the connector with the top camera, pinches it near its cord end, turns it to point
    along the hand-off line, brings it in from behind the holder so the trailing cord drops into the slot, sets it on
    the tray (tip far off the field, its own gripper centre just on the field side) and moves back. Right
    (circulating) arm looks again, pinches the connector 1 cm from its tip at the far end of the tray (its whole arm
    stays clear of the field), draws it 5 cm further out, lets go, looks again, re-grips 4.5 cm from the tip (wrist
    camera towards the jack), pushes it along the line into the jack, lets go and backs off."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN

    L, Rt = ctx.left, ctx.right
    cheat = os.environ.get("STERILE_CHEAT", "")
    dbg = bool(os.environ.get("STERILE_DEBUG"))

    def log(tag, est=None):
        st = ctx.states()  # logging only: never used to choose a motion
        p, a = plug_pose(st)
        extra = ""
        if est is not None:
            ed, eb = ax_coords(est[0])
            extra = (f" cam=({est[0][0]:.3f},{est[0][1]:.3f}) cam_err={np.round(est[0] - p[:2], 4).tolist()} "
                     f"cam_hd_err={math.degrees(_wrap(_hd(est[1]) - _hd(a))):.1f} len={est[2]:.3f}")
        print(f"[demo t={ctx.t:5.1f}] {tag}: plug={np.round(p, 3).tolist()} hd={math.degrees(_hd(a)):.0f} "
              f"ax={np.round(ax_coords(p), 3).tolist()} depth={jack_depth(st):.3f} slot={cord_in_slot(st)} "
              f"ledge={plug_on_ledge(st)} rel={released(st)} viol={violations()} low={plug_low(st) - TOP:.3f} "
              f"minLy={_MON['min_left_y']:.3f} maxRfieldY={_MON['max_right_field_y']:.3f} dirty={_MON['n_dirty']} "
              f"glove_dirty={_MON['glove_dirty_why']} glove_off_minz={_MON['min_glove_off_z']:.4f} "
              f"score={score(st):.2f}{extra}", flush=True)

    def look(near, toward, on_axis=True, tries=4):
        est = None
        for _ in range(tries):
            yield from L.wait(0.3)
            est = see_plug(ctx, near, toward, on_axis=on_axis)
            if est is not None and 0.08 <= est[2] <= 0.16:
                break
        return est

    log("start")
    if cheat == "nothing":
        yield from L.wait(5.0)
        log("end")
        return
    if cheat == "glove_touch":
        # the scrub gripper (centre still at y >= 0) brushes the bare table just beyond the drape's far-left edge,
        # lifts, then does the full run: it then handles the sterile connector on the drape with a non-sterile glove
        t0 = (0.30, FIELD_Y1 + 0.09)
        yield from L.move((t0[0], t0[1], FT + 0.08), yaw=0.0, duration=1.5)
        yield from L.move((t0[0], t0[1], TOP + 0.014), yaw=0.0, duration=1.5)
        yield from L.wait(0.3)
        log("scrub gripper touching the table beside the drape")
        yield from L.move((t0[0], t0[1], FT + 0.10), yaw=0.0, duration=1.2)

    hi = FT + 0.06
    # 0) left arm out of the camera's line of sight, then find the connector (fixed layout: it lies at the cord's end)
    plug0 = A + _DIR * (CORD_LEN + PL / 2)
    yield from L.move((0.14, 0.40, FT + 0.10), yaw=0.0, duration=1.2)
    est = yield from look(plug0, plug0 + _DIR, on_axis=False)  # tip = the end away from the holster
    log("seen on drape", est)
    if est is None:
        est = (plug0, _DIR, PL)
    c0, a0, _ = est
    rear = c0 - a0 * PL / 2
    gy = _wrap(_hd(a0))
    if abs(gy) > math.pi / 2:
        gy = _wrap(gy + math.pi)
    g = rear + a0 * SCRUB_GRIP_BACK
    yield from L.move((g[0], g[1], hi), yaw=gy, finger=FINGER_OPEN, duration=1.5)
    yield from L.move((g[0], g[1], FT + PH / 2 + 0.010), yaw=gy, duration=1.0)
    yield from L.grip(FINGER_CLOSED)
    yield from L.move((g[0], g[1], hi), yaw=gy, duration=0.8)
    log("scrub has plug")

    # 1) turn the plug to point along the axis and come in from behind the holder
    tcp_d = SCRUB_TCP_D if cheat != "scrub_carry" else 0.045  # scrub_carry: carries it out past the edge itself
    if cheat == "noclip":  # comes in sideways from the left of the axis: the trailing cord misses the slot
        back, pre = ax(tcp_d - 0.02, 0.16), ax(tcp_d - 0.02, 0.02)
    else:  # comes in along the axis from behind the holder: the trailing cord lies over the slot
        back, pre = ax(tcp_d - 0.20), ax(tcp_d - 0.02)
    yield from L.move((back[0], back[1], hi), yaw=AX_YAW, duration=2.0)
    yield from L.move((pre[0], pre[1], FT + 0.035), yaw=AX_YAW, duration=1.5)
    tgt = ax(tcp_d)
    yield from L.move((tgt[0], tgt[1], FT + 0.030), yaw=AX_YAW, duration=1.0)
    yield from L.move((tgt[0], tgt[1], FT + PH / 2 + 0.011), yaw=AX_YAW, duration=1.0)
    yield from L.wait(0.3)
    log("plug set on tray")
    yield from L.grip(FINGER_OPEN, 0.5)
    up = tgt - U * 0.02
    yield from L.move((up[0], up[1], hi), yaw=AX_YAW, duration=0.8)
    yield from L.move((0.20, 0.30, hi + 0.05), yaw=0.0, duration=1.2)

    if cheat == "drag_back":
        # the tip end of the plug has lain on the tray off the field (non-sterile); the scrub arm pinches it again
        # and drags it 5 cm back over the field, then sets it out again; everything else as in the full run
        est = yield from look(ax(0.05), ax(0.3))
        log("seen on tray (drag_back)", est)
        if est is None:
            est = (ax(SCRUB_TCP_D - SCRUB_GRIP_BACK + PL / 2), U, PL)
        c, a, _ = est
        gg = c - a * (PL / 2 - SCRUB_GRIP_BACK + 0.006)  # 4 mm from the cord end: its gripper centre stays on the field
        yield from L.move((gg[0], gg[1], hi), yaw=AX_YAW, finger=FINGER_OPEN, duration=1.2)
        yield from L.move((gg[0], gg[1], FT + PH / 2 + 0.010), yaw=AX_YAW, duration=0.8)
        yield from L.grip(FINGER_CLOSED)
        bk = gg - U * 0.05
        yield from L.move((gg[0], gg[1], FT + PH / 2 + 0.013), yaw=AX_YAW, duration=0.4)
        yield from L.move((bk[0], bk[1], FT + PH / 2 + 0.013), yaw=AX_YAW, duration=1.2)
        yield from L.wait(0.3)
        log("dragged back over the field")
        yield from L.move((gg[0], gg[1], FT + PH / 2 + 0.013), yaw=AX_YAW, duration=1.2)
        yield from L.move((gg[0], gg[1], FT + PH / 2 + 0.011), yaw=AX_YAW, duration=0.4)
        yield from L.grip(FINGER_OPEN, 0.5)
        up = gg - U * 0.02
        yield from L.move((up[0], up[1], hi), yaw=AX_YAW, duration=0.8)
        yield from L.move((0.20, 0.30, hi + 0.05), yaw=0.0, duration=1.2)

    if cheat == "tier03":  # left arm's half only: the plug stays on the tray, both arms clear
        yield from L.wait(3.0)
        log("end")
        return

    if cheat == "lean_over":
        # right gripper centre stays off the field (y = -3 cm) but its wrist leans over the drape; then the full run
        yield from Rt.move((0.26, -0.03, FT + 0.12), yaw=0.0, duration=1.5)
        yield from Rt.wait(0.5)
        log("right wrist over the field, gripper centre off it")
        yield from Rt.move((0.25, -0.25, FT + 0.12), yaw=0.0, duration=1.2)

    # 2) look at the plug on the tray (both arms out of the line of sight)
    est = yield from look(ax(0.05), ax(0.3))
    log("seen on tray", est)
    if est is None:
        est = (ax(SCRUB_TCP_D - SCRUB_GRIP_BACK + PL / 2), U, PL)
    c1, a1, _ = est
    tip1 = c1 + a1 * PL / 2
    ry = _wrap(_hd(a1))
    if abs(ry) > math.pi / 2:
        ry = _wrap(ry + math.pi)
    if cheat == "reach_over":  # takes the plug at its cord end, over the field edge
        g = c1 - a1 * (PL / 2 - 0.004)
    else:
        g = tip1 - a1 * PULL_GRIP
    yield from Rt.move((g[0], g[1], hi), yaw=ry, finger=FINGER_OPEN, duration=1.5)
    yield from Rt.move((g[0], g[1], FT + PH / 2 + 0.010), yaw=ry, duration=1.0)
    yield from Rt.grip(FINGER_CLOSED)
    log("circulator has plug by its tip")

    if cheat == "drop":
        # carries the plug off the tray, lets it fall onto the table
        side = g - NRM * 0.07
        yield from Rt.move((g[0], g[1], FT + PH / 2 + 0.03), yaw=ry, duration=0.6)
        yield from Rt.move((side[0], side[1], FT + PH / 2 + 0.03), yaw=ry, duration=1.0)
        yield from Rt.grip(FINGER_OPEN, 0.4)
        yield from Rt.move((side[0], side[1], hi + 0.05), yaw=ry, duration=0.8)
        yield from Rt.wait(1.5)
        log("end")
        return

    # 3) draw it further out along the tray, set it down, let go and clear the camera's view
    g2 = g + U * PULL_DIST
    yield from Rt.move((g[0], g[1], FT + PH / 2 + 0.013), yaw=ry, duration=0.4)
    yield from Rt.move((g2[0], g2[1], FT + PH / 2 + 0.013), yaw=ry, duration=1.5)
    yield from Rt.move((g2[0], g2[1], FT + PH / 2 + 0.010), yaw=ry, duration=0.4)
    yield from Rt.grip(FINGER_OPEN, 0.5)
    yield from Rt.move((g2[0], g2[1], hi), yaw=ry, duration=0.6)
    yield from Rt.move((0.25, -0.30, hi + 0.04), yaw=0.0, duration=1.0)
    est = yield from look(ax(0.10), ax(0.4))
    log("seen drawn out", est)
    if est is None:
        est = (ax(SCRUB_TCP_D - SCRUB_GRIP_BACK + PL / 2 + PULL_DIST), U, PL)
    c2, a2, _ = est
    tip2 = c2 + a2 * PL / 2

    # 4) re-grip further back, turn onto the axis, push into the jack (plug held centred between the pads)
    g3 = tip2 - a2 * SEAT_GRIP
    yield from Rt.move((g3[0], g3[1], hi), yaw=ry, finger=FINGER_OPEN, duration=1.2)
    yield from Rt.move((g3[0], g3[1], FT + PH / 2 + 0.010), yaw=ry, duration=0.8)
    yield from Rt.grip(FINGER_CLOSED)
    log("circulator re-gripped")
    zoff = 0.010  # TCP above the plug centre (grasp height)
    tip_d = ax_coords(tip2)[0]
    yaw_ax = AX_YAW if abs(_wrap(AX_YAW - ry)) < math.pi / 2 else _wrap(AX_YAW + math.pi)

    def tcp_for_tip(d_tip: float, dz: float = 0.0):
        x = ax(d_tip - SEAT_GRIP)
        return (x[0], x[1], JACK_ZC + zoff + dz)

    yield from Rt.move(tcp_for_tip(tip_d, 0.004), yaw=ry, duration=0.5)
    yield from Rt.move(tcp_for_tip(JACK_D - 0.025, 0.002), yaw=yaw_ax, duration=1.5)
    yield from Rt.move(tcp_for_tip(JACK_D - 0.008, 0.0), yaw=yaw_ax, duration=1.0)
    log("at jack mouth")
    yield from Rt.move(tcp_for_tip(JACK_D + SEAT_DEPTH, 0.0), yaw=yaw_ax, duration=1.5)
    yield from Rt.wait(0.3)
    log("seated (held)")
    if cheat == "hold":
        yield from Rt.wait(3.0)
        log("end")
        return
    if cheat == "hold_cord":  # the scrub arm closes on the cord near the slot, then the right arm lets go and backs off
        q = ax(-0.05)
        yield from L.move((q[0], q[1], hi), yaw=AX_YAW, finger=-0.014, duration=1.2)
        yield from L.move((q[0], q[1], FT + R + 0.010), yaw=AX_YAW, duration=0.8)
        yield from L.grip(FINGER_CLOSED)
        log("scrub holding the cord")
        yield from Rt.grip(FINGER_OPEN, 0.5)
        cur = Rt.tcp()
        bk = cur[:2] - U * 0.03
        yield from Rt.move((bk[0], bk[1], cur[2] + 0.01), yaw=yaw_ax, duration=0.8)
        yield from Rt.move((0.25, -0.30, hi + 0.05), yaw=0.0, duration=1.2)
        yield from Rt.wait(3.0)
        log("end")
        return
    yield from Rt.grip(FINGER_OPEN, 0.5)
    cur = Rt.tcp()
    bk = cur[:2] - U * 0.03
    yield from Rt.move((bk[0], bk[1], cur[2] + 0.01), yaw=yaw_ax, duration=0.8)
    yield from Rt.move((0.25, -0.30, hi + 0.05), yaw=0.0, duration=1.2)
    yield from Rt.wait(1.5)
    log("end")
