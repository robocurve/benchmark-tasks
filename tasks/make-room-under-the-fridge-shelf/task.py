"""The fresh-food compartment of a French-door refrigerator stands open in front of the robot, its doors open and its
upper shelves taken out: white liner walls left, right and behind, a glass lower shelf, and above it a full-width
glass shelf split into a front half and a back half. A full 500 mL bottle of water stands upright at the front of
the lower shelf, in front of the split shelf. Both arms start at their ready pose with the grippers open.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
from functools import lru_cache

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import parts  # noqa: E402  (before lots: its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.1  # the 1 s release-and-still hold is counted inside score()
INSTRUCTION = ("Stand the tall bottle upright on the lower shelf with its whole base behind the front ends of the split "
               "shelf's side brackets.")
MM = 1e-3
EPS = 1e-9

# -- geometry (m; x away from the arms, y to the robot's left, z above the table top) ------------------------------
LZ = 0.004  # the lower shelf's glass top (4 mm tempered glass on the table)
H = LZ + 0.161  # top of the split shelf's glass, both halves, before the front half is moved
XF = 0.200  # front face of the front half's trim, level with the front ends of the two side brackets
FRONT_D = 0.188  # front half: trim face to the glass's back edge
REAR_X0 = XF + 0.190  # back half's front trim face
REAR_X1 = XF + 0.440  # back half's rear edge (hooks into the wall tracks)
WALL_X = XF + 0.460  # rear liner wall's inner face
HALF_W = 0.410  # liner side walls' inner faces at y = +-0.41 (822 mm inside)
LIFT = 0.008  # how far the front half can be raised: the top of its guides
TRAVEL = 0.176  # front half's travel back from where it starts to its stop on the lower track
PULL = 0.024  # front half's travel forward to the end of its upper seats
DROP = 0.030  # the lower track is 30 mm below the upper seats

LINER = (0.93, 0.94, 0.95)
GLASS = (0.60, 0.77, 0.84)
GLASS_LO = (0.88, 0.92, 0.93)
TRIM = (0.80, 0.82, 0.84)
FRONT_TRIM = (0.33, 0.36, 0.40)  # the front half's front trim, darker so it reads in the video
BRACKET = (0.66, 0.68, 0.70)
TRACK = (0.30, 0.31, 0.33)
WATER = (0.62, 0.80, 0.93)
LABEL = (0.95, 0.95, 0.96)
CAP = (0.12, 0.32, 0.78)
PLASTIC_MU = (0.5, 0.4)  # ABS liner, brackets and trims (static, dynamic; estimate)
GLASS_MU = (0.35, 0.30)  # the lower shelf's glass under a PET bottle (estimate)
PET_MU = (0.40, 0.35)  # the bottle (PhysX averages the two surfaces' values)


def _b(lo, hi, color, collide=True):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(hi - lo), tuple((lo + hi) / 2), color, collide=collide)


def _pair(x0, x1, y0, y1, z0, z1, color, collide=True):
    """The same box on both sides of the midline: y0..y1 on the left, -y1..-y0 on the right."""
    return [_b((x0, y0, z0), (x1, y1, z1), color, collide), _b((x0, -y1, z0), (x1, -y0, z1), color, collide)]


def _fridge_shapes():
    s = []
    # liner: side walls, rear wall, the lower shelf (glass on a white front trim), the rear wall's shelf tracks
    s += _pair(0.040, WALL_X + 0.020, HALF_W, HALF_W + 0.020, 0.0, 0.600, LINER)
    s.append(_b((WALL_X, -HALF_W - 0.020, 0.0), (WALL_X + 0.020, HALF_W + 0.020, 0.600), LINER))
    s.append(parts.box((WALL_X - 0.052, 2 * HALF_W, LZ), ((WALL_X + 0.052) / 2, 0.0, LZ / 2), GLASS_LO,
                       friction=GLASS_MU))
    s.append(_b((0.040, -HALF_W, 0.0), (0.052, HALF_W, LZ + 0.006), TRIM))
    for yc in (-0.398, 0.0, 0.398):
        s.append(_b((WALL_X - 0.008, yc - 0.012, 0.020), (WALL_X, yc + 0.012, 0.560), TRACK))
    # the split shelf's two side brackets, hooked into the outer tracks
    s += _pair(XF, WALL_X - 0.008, 0.396, HALF_W, H - 0.075, H + 0.002, BRACKET)
    # on each bracket: the upper seat the front half's rear tab rests on, with a lip in front of it ...
    s += _pair(XF + 0.164, XF + 0.190, 0.381, 0.396, H - 0.030, H - 0.024, BRACKET)
    s += _pair(XF + 0.164, XF + 0.170, 0.381, 0.396, H - 0.030, H - 0.019, BRACKET)
    # ... and the lower track under the back half, ending in a stop
    s += _pair(XF + 0.140, XF + 0.400, 0.381, 0.396, H - 0.061, H - 0.055, BRACKET)
    s += _pair(XF + TRAVEL + 0.1855, XF + TRAVEL + 0.194, 0.381, 0.396, H - 0.061, H - 0.035, BRACKET)
    # the back half: glass in a frame, fixed on the brackets
    s.append(_b((REAR_X0 + 0.004, -0.380, H - 0.004), (REAR_X1, 0.380, H), GLASS))
    s.append(_b((REAR_X0, -0.382, H - 0.008), (REAR_X0 + 0.012, 0.382, H + 0.010), TRIM))
    s += _pair(REAR_X0, REAR_X1, 0.382, 0.396, H - 0.012, H + 0.002, TRIM)
    s.append(_b((REAR_X1 - 0.012, -0.382, H - 0.008), (REAR_X1, 0.382, H + 0.010), TRIM))
    return s


def _front_half_shapes():
    s = [_b((XF + 0.012, -0.370, H - 0.004), (XF + FRONT_D, 0.370, H), GLASS),
         _b((XF, -0.380, H - 0.014), (XF + 0.012, 0.380, H + 0.010), FRONT_TRIM)]
    s += _pair(XF + 0.012, XF + FRONT_D, 0.370, 0.380, H - 0.012, H + 0.002, TRIM)  # side runners
    s += _pair(XF + 0.172, XF + 0.184, 0.380, 0.390, H - 0.024, H - 0.010, TRIM)  # rear tabs
    return s


FRONT_MASS = 1.45  # kg, estimate: 4 mm tempered glass 740 x 176 mm (1.30 kg) plus its plastic trim
FRIDGE = parts.Part(
    "fridge", pos=(0.0, 0.0, TOP), mass=None, contact_offset=0.002, friction=PLASTIC_MU, shapes=_fridge_shapes(),
    joints=[
        # the front half rides its side channels: a slide along the depth carrying a rise and fall between the
        # upper seats and the lower track (the carriage is the channel itself: no collider, hidden in the bracket)
        parts.Slider("slide", (_b((XF + 0.20, 0.400, H - 0.040), (XF + 0.204, 0.404, H - 0.036), BRACKET, False),),
                     axis=(1.0, 0.0, 0.0), travel=TRAVEL, extend=PULL, mass=0.05, damping=4.0),
        parts.Slider("level", tuple(_front_half_shapes()), axis=(0.0, 0.0, 1.0), travel=LIFT, extend=DROP,
                     mass=FRONT_MASS, damping=4.0, parent="slide"),
    ])

BOTTLE_R = 0.0325  # 65 mm body
BOTTLE_H = 0.205
BOTTLE_MASS = 0.52  # kg: 500 g of water and an estimated 20 g bottle and cap
_UP = (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))  # turns a prism's x axis onto z


def _bottle_shapes():
    return [parts.prism(BOTTLE_R, 0.150, 24, at=(0.0, 0.0, 0.075), color=WATER, quat=_UP),
            parts.prism(BOTTLE_R + 0.0004, 0.050, 24, at=(0.0, 0.0, 0.085), color=LABEL, quat=_UP, collide=False),
            parts.prism(0.024, 0.025, 20, at=(0.0, 0.0, 0.1625), color=WATER, quat=_UP),
            parts.prism(0.0145, 0.018, 16, at=(0.0, 0.0, 0.184), color=WATER, quat=_UP),
            parts.prism(0.0155, 0.012, 16, at=(0.0, 0.0, 0.199), color=CAP, quat=_UP)]


BOTTLE_A = (XF - 0.075, 0.060, TOP + LZ + 0.0005)
BOTTLE_B = (XF - 0.075, -0.060, TOP + LZ + 0.0005)
BOTTLE = parts.Part("bottle", pos=BOTTLE_A, mass=BOTTLE_MASS, friction=PET_MU, shapes=_bottle_shapes())

OBJECTS = [FRIDGE, BOTTLE]
LOTS = lots.declare(OBJECTS,
                    lots.Lot("A", place={"bottle": BOTTLE_A}, side="left"),
                    lots.Lot("B", place={"bottle": BOTTLE_B}, side="right"))

# -- reading the state ---------------------------------------------------------------------------------------------


def _R(q):
    return parts._quat_to_matrix(q)


def _finite(*vals):
    return all(np.isfinite(np.asarray(v, float)).all() for v in vals)


def shelf_read(st):
    """(travel back from the start, rise above the upper seats) of the front half, in m."""
    return float(st.joints["slide"]), float(st.joints["level"])


def tucked(st):
    """On the lower track and within 5 mm of its stop: the front trim within 19 mm of the back half's front edge."""
    s, z = shelf_read(st)
    return s >= TRAVEL - 0.005 - EPS and z <= -DROP + 0.003 + EPS


def lowered(st):
    """Down on the lower track (within 3 mm of it), anywhere along it."""
    return shelf_read(st)[1] <= -DROP + 0.003 + EPS


def bottle_tilt(st):
    axis = _R(st.quat_xyzw)[:, 2]
    return math.degrees(math.acos(float(np.clip(axis[2], -1.0, 1.0))))


def bottle_base(st):
    """Centre of the bottle's base (world)."""
    return np.asarray(st.pos, float)


def standing(st):
    """Upright within 5 degrees, base on the lower shelf (within 3 mm above it, 1 mm below)."""
    z = bottle_base(st)[2] - (TOP + LZ)
    return bottle_tilt(st) <= 5.0 + EPS and -0.001 - EPS <= z <= 0.003 + EPS


def base_x_range(st):
    """Front-most and back-most x of the bottle's base circle (world)."""
    c = bottle_base(st)
    return c[0] - BOTTLE_R, c[0] + BOTTLE_R


def inside(st):
    """The whole base behind the line of the split shelf's front edge (where the doors' bins come when shut)."""
    c = bottle_base(st)
    return base_x_range(st)[0] >= XF - EPS and abs(c[1]) + BOTTLE_R <= HALF_W + EPS


def centre_inside(st):
    c = bottle_base(st)
    return c[0] >= XF - EPS and abs(c[1]) <= HALF_W + EPS


SHELF_FRONT = 0.040  # front edge of the lower shelf's trim
BOTTLE_MID = 0.090  # height of the full bottle's centre of mass above its base


def fallen_out(st):
    """Out of the fridge: the bottle's middle has passed the lower shelf's front edge, or any part of it is more than
    20 mm below the lower shelf's top."""
    mid = np.asarray(st.pos, float) + _R(st.quat_xyzw) @ np.array([0.0, 0.0, BOTTLE_MID])
    return float(st.low[2]) < TOP + LZ - 0.020 - EPS or float(mid[0]) < SHELF_FRONT - EPS


def tier(fr, bo):
    """The state's value before the release gate (0, 0.25, 0.5, 0.75, 1.0). Nothing counts until the front half is
    down on its lower track; the bottle can only stand behind the front line once the half has gone back far enough."""
    if not lowered(fr):
        return 0.0
    if standing(bo) and inside(bo):
        return 1.0
    if standing(bo) and centre_inside(bo):
        return 0.75
    return 0.5 if tucked(fr) else 0.25


# -- hands clear ---------------------------------------------------------------------------------------------------
CLEAR = 8.0 * MM  # 5 mm clear plus 3 mm for the 3 mm face sampling
SCORED_LINKS = ("level",)


def scored_boxes(objects):
    fr, bo = objects[FRIDGE.name], objects[BOTTLE.name]
    out = []
    poses = {k: (np.asarray(fr.links[k][0], float), _R(fr.links[k][1])) for k in SCORED_LINKS}
    out += [(c, r, s) for _, c, r, s in FRIDGE.world_boxes(poses)]
    out += [(c, r, s) for _, c, r, s in BOTTLE.world_boxes({"base": (np.asarray(bo.pos, float), _R(bo.quat_xyzw))})]
    return out


def hands_clear(clouds, objects):
    """True only when every sampled gripper point is more than CLEAR from every box of the front half and the
    bottle."""
    if not clouds:
        return False
    pts = []
    for p in clouds:
        p = np.asarray(p, float)
        if p.ndim != 2 or p.shape[1] != 3 or not len(p) or not np.isfinite(p).all():
            return False
        pts.append(p)
    pts = np.concatenate(pts)
    for c, r, s in scored_boxes(objects):
        local = np.abs((pts - c) @ r) - s / 2
        if np.linalg.norm(np.maximum(local, 0.0), axis=1).min() <= CLEAR + EPS:
            return False
    return True


STILL_V, DRIFT_POS, DRIFT_DEG = 3.0 * MM, 0.5 * MM, 1.0
HOLD_S = 1.0


def poses(objects):
    fr, bo = objects[FRIDGE.name], objects[BOTTLE.name]
    pts = np.array([np.asarray(fr.links["level"][0], float), np.asarray(bo.pos, float)])
    return pts, [_R(fr.links["level"][1]), _R(bo.quat_xyzw)]


def drifted(ref, now):
    if ref is None:
        return True
    if np.linalg.norm(now[0] - ref[0], axis=1).max() > DRIFT_POS + EPS:
        return True
    for a, b in zip(ref[1], now[1]):
        if math.degrees(math.acos(float(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))) > DRIFT_DEG + EPS:
            return True
    return False


def calm(objects):
    fr, bo = objects[FRIDGE.name], objects[BOTTLE.name]
    return float(np.linalg.norm(bo.lin_vel)) <= STILL_V + EPS and float(np.linalg.norm(fr.lin_vel)) <= STILL_V + EPS


# -- the trial record: written only by the simulator's step and reset hooks -----------------------------------------


def _fresh():
    return dict(known=False, invalid=False, t=None, held=0.0, ref=None, stamp=None, clear=False, still=False,
                value=None, out=False)


REC = _fresh()


def stamp(objects):
    fr, bo = objects[FRIDGE.name], objects[BOTTLE.name]
    out = [*np.round(np.asarray(bo.pos, float), 9), *np.round(np.asarray(bo.quat_xyzw, float), 9)]
    out += [round(float(fr.joints[k]), 9) for k in ("slide", "level")]
    return tuple(float(v) for v in out)


def observe(objects, clouds, t):
    """Update the trial record from one control step's state and gripper surface samples (None: unknown)."""
    try:
        fr, bo = objects[FRIDGE.name], objects[BOTTLE.name]
        ok = _finite(bo.pos, bo.quat_xyzw, bo.lin_vel, bo.low, fr.lin_vel, *(fr.joints[k] for k in ("slide", "level")),
                     fr.links["level"][0], fr.links["level"][1])
    except (KeyError, TypeError, ValueError, AttributeError):
        ok = False
    dt = None if REC["t"] is None else float(t) - REC["t"]
    if not ok or clouds is None or (dt is not None and not 0.0 < dt <= 0.2):
        REC.update(known=False, invalid=True, held=0.0, t=float(t), stamp=None)
        return
    if fallen_out(bo):
        REC["out"] = True
    value = tier(fr, bo)
    clear = bool(hands_clear(clouds, objects))
    now = poses(objects)
    still = calm(objects)
    if clear and still and value == REC["value"] and dt is not None and not drifted(REC["ref"], now):
        REC["held"] += dt
    else:
        REC["held"], REC["ref"] = 0.0, now
    REC.update(known=True, t=float(t), value=value, stamp=stamp(objects), clear=clear, still=still)


def score(objects):
    """0 for good once the bottle has left the fridge or the record is invalid. Otherwise, with both grippers clear of
    the front half and the bottle and both still for the last 1 s, and the front half down on its lower track: 1.0 the
    bottle upright on the lower shelf with its whole base behind the brackets' front ends; 0.75 upright with only its
    base centre behind that line; 0.5 the front half tucked to its stop; 0.25 lowered but not tucked."""
    try:
        if not REC["known"] or REC["invalid"] or REC["out"] or REC["stamp"] != stamp(objects):
            return 0.0
        if not (REC["clear"] and REC["still"] and REC["held"] >= HOLD_S - EPS):
            return 0.0
        return float(REC["value"])
    except (KeyError, TypeError, ValueError, AttributeError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - EPS


# -- simulator hooks -------------------------------------------------------------------------------------------------


@lru_cache(maxsize=1)
def hand_samples():
    """Points on every collision face of each gripper link (link frame), at most 3 mm apart."""
    from yam_arena import analysis

    out = {}
    for name in ("gripper", "tip_left", "tip_right"):
        cloud = []
        for h in analysis.link_hulls().get(name, []):
            cloud.extend(h.vertices)
            for pl in np.unique(h.planes.round(7), axis=0):
                face = h.vertices[np.abs(h.vertices @ pl[:3] - pl[3]) < 2e-6]
                if len(face) < 3:
                    continue
                c = face.mean(0)
                u = face[0] - c
                u /= np.linalg.norm(u)
                v = np.cross(pl[:3], u)
                face = face[np.argsort(np.arctan2((face - c) @ v, (face - c) @ u))]
                for a, b in zip(face, np.roll(face, -1, axis=0)):
                    n = max(1, math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / 0.003))
                    for i in range(n + 1):
                        for j in range(n - i + 1):
                            cloud.append(c + (a - c) * (i / n) + (b - c) * (j / n))
        if not cloud:
            raise ValueError("missing hand geometry")
        out[name] = np.unique(np.round(np.asarray(cloud), 4), axis=0)
    return out


def hand_clouds(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    clouds = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        body = parts._np(asset.data.body_link_pose_w)[0]
        if body.ndim != 2 or body.shape[1] != 7 or not np.isfinite(body).all():
            raise ValueError("unknown robot body poses")
        for name, pts in hand_samples().items():
            pose = body[asset.body_names.index(name)]
            clouds.append(pts @ _R(pose[3:]).T + pose[:3] - origin)
    return clouds


@parts.on_reset
def _reset(sim):
    REC.clear()
    REC.update(_fresh())


@parts.on_step
def _step(sim):
    objects = sim.states()
    try:
        clouds = hand_clouds(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        clouds = None
    observe(objects, clouds, float(sim.t))
    if os.environ.get("TRACE_OUT"):
        fr, bo = objects[FRIDGE.name], objects[BOTTLE.name]
        s, z = shelf_read(fr)
        row = dict(t=round(float(sim.t), 3), lot=lots.current().name, slide_mm=round(s * 1000, 2),
                   level_mm=round(z * 1000, 2), bottle=np.round(bo.pos, 4).tolist(), tilt=round(bottle_tilt(bo), 2),
                   bottle_low_z=round(float(bo.low[2]) - TOP, 4), value=REC["value"], held=round(REC["held"], 2),
                   clear=REC["clear"], still=REC["still"], known=REC["known"], out=REC["out"], score=score(objects))
        try:
            from yam_arena.constants import TCP_OFFSET
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            tcp = {}
            for arm in ("left_robot", "right_robot"):
                rob = base.scene[arm]
                pose = parts._np(rob.data.body_link_pose_w)[0, rob.body_names.index("gripper")]
                tcp[arm] = np.round(pose[:3] - origin + _R(pose[3:]) @ np.array(TCP_OFFSET), 4).tolist()
                tcp[arm + "_f"] = np.round(parts._np(rob.data.joint_pos)[0, -2:], 4).tolist()
            row["tcp"] = tcp
        except Exception as e:  # noqa: BLE001  trace only
            row["tcp_err"] = str(e)
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("shelf.jsonl").open("a") as f:
            f.write(json.dumps(row, default=lambda o: o.item() if hasattr(o, "item") else str(o)) + "\n")


# -- scripted demo: open loop from the published layout of this trial ---------------------------------------------------
GRIP_Y = 0.200  # the two hands take the front trim this far either side of the midline
PINCH_Z = H + 0.010 + 0.004  # TCP 4 mm above the trim's top: the pad tips reach 1 mm above the glass
PINCH = -0.001  # finger target: 5 mm past the 12 mm trim's faces
LIFT_CMD = 0.035  # the arms sag under the half's weight: aim 35 mm up for its 8 mm rise
PUSH_Z = H - 0.014  # TCP when pushing the lowered trim: the closed tips meet its face, the hand stays above the back half
PARK = {"left": (0.200, 0.250, TOP + 0.200), "right": (0.200, -0.250, TOP + 0.200)}
BOTTLE_PUSH_Z = LZ + 0.055  # TCP while pushing the bottle: the leaning fingertips meet its base
LEAN = 40.0  # degrees the fingertips lean forward for the bottle push: the hand trails behind and above
BOTTLE_TIP_LEAD = 0.034  # TCP to the bottle's axis when the leaning, open fingertips touch its base (offline FK)
BAY_X = XF + 0.070  # where the demo stands the bottle: its base 37 mm behind the shelf's front line


def _yaw(arm):
    return math.pi / 2 if arm == "right" else -math.pi / 2


def shelf_plan(arm):
    """One hand's part in moving the front half: take its trim, lift it off the seats, draw it forward, lower it onto
    the track, let go, then push it back along the track to its stop with the closed fingers."""
    y = GRIP_Y if arm == "left" else -GRIP_Y
    yaw = _yaw(arm)
    x0 = XF + 0.006  # trim centre
    z0 = TOP + PINCH_Z
    xp = x0 - PULL - 0.006
    zl = z0 - DROP - 0.008
    push_x0 = XF - PULL - 0.008  # closed tips 4 mm in front of the lowered trim's face
    push_x1 = XF + TRAVEL + 0.002  # 6 mm past the stop
    zp = TOP + PUSH_Z
    return [
        Waypoint(2.5, (x0, y, z0 + 0.018), -0.025, "over the front trim", yaw),
        Waypoint(1.5, (x0, y, z0), -0.025, "down astride the trim", yaw),
        Waypoint(1.0, (x0, y, z0), PINCH, "pinch the trim", yaw),
        Waypoint(1.5, (x0, y, z0 + LIFT_CMD), PINCH, "lift the half off its seats", yaw),
        Waypoint(0.5, (x0, y, z0 + LIFT_CMD), PINCH, "hold", yaw),
        Waypoint(1.5, (xp, y, z0 + LIFT_CMD), PINCH, "draw it forward past the seats", yaw),
        Waypoint(2.0, (xp, y, zl), PINCH, "lower it onto the track", yaw),
        Waypoint(0.5, (xp, y, zl), PINCH, "settle", yaw),
        Waypoint(0.8, (xp, y, zl), -0.025, "let go", yaw),
        Waypoint(1.0, (xp, y, z0 + 0.010), -0.025, "up clear of the trim", yaw),
        Waypoint(1.0, (push_x0, y, z0 + 0.010), FINGER_CLOSED, "in front of the trim, fingers shut", yaw),
        Waypoint(1.0, (push_x0, y, zp), FINGER_CLOSED, "down to the trim", yaw),
        Waypoint(4.0, (push_x1, y, zp), FINGER_CLOSED, "push it back under the back half", yaw),
        Waypoint(0.5, (push_x1, y, zp), FINGER_CLOSED, "hold at the stop", yaw),
        Waypoint(1.0, (push_x1 - 0.030, y, zp), FINGER_CLOSED, "back off", yaw),
        Waypoint(1.0, (push_x1 - 0.030, y, zp + 0.040), FINGER_CLOSED, "up", yaw),
    ]


def _lean(t, p, f, label):
    return tilt.Waypoint(float(t), tuple(float(v) for v in p), float(f), label, yaw=0.0, tilt=LEAN, heading=0.0)


def bottle_plan(arm, start):
    """Slide the bottle upright along the lower shelf into the opened space: the hand comes down beside it, moves in
    behind it, and pushes it at its base with the two open fingertips leaning forward (a V that keeps it centred)."""
    bx, by = float(start[0]), float(start[1])
    side = 1.0 if arm == "left" else -1.0
    z = TOP + BOTTLE_PUSH_Z
    behind = bx - BOTTLE_TIP_LEAD - 0.025  # fingertips 10 mm in front of the bottle's front
    beside = by - side * 0.120  # on the midline side: the elbow stays clear of the liner wall
    v = -0.025  # pads 50 mm apart
    return [
        _lean(2.0, (0.250, beside, TOP + 0.170), v, "across the opened space"),
        _lean(2.0, (behind, beside, TOP + 0.150), v, "beside the bottle, in front of the shelf"),
        _lean(1.0, (behind, beside, z), v, "down"),
        _lean(1.5, (behind, by, z), v, "in front of its base"),
        _lean(1.5, (bx - BOTTLE_TIP_LEAD - 0.004, by, z), v, "fingertips to its base"),
        _lean(4.0, (BAY_X - BOTTLE_TIP_LEAD, by, z), v, "slide it in under the opened space"),
        _lean(0.5, (BAY_X - BOTTLE_TIP_LEAD, by, z), v, "stop"),
        _lean(1.5, (BAY_X - BOTTLE_TIP_LEAD - 0.050, by, z), v, "back away"),
        _lean(1.0, (BAY_X - BOTTLE_TIP_LEAD - 0.050, by, TOP + 0.150), v, "up"),
    ]


def demo():
    lot = lots.current()
    side = lot.side
    other = "right" if side == "left" else "left"
    plans = {a: shelf_plan(a) for a in ("left", "right")}
    t_shelf = sum(w.duration for w in plans["left"])
    plans[other] += [Waypoint(2.0, PARK[other], FINGER_OPEN, "park", 0.0)]
    bottle = bottle_plan(side, lot.pos(BOTTLE.name))
    plans[side] += bottle + [Waypoint(2.0, PARK[side], FINGER_OPEN, "park", 0.0)]
    total = max(sum(w.duration for w in p) for p in plans.values()) + 2.5
    for a in plans:
        rest = total - sum(w.duration for w in plans[a])
        plans[a].append(Waypoint(rest, PARK[a], FINGER_OPEN, "rest", 0.0))
    _ = t_shelf
    return plans
