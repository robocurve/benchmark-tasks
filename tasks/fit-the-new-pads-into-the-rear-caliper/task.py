"""On the station table: a rear brake caliper on its carrier, off the car, bolted to a low bench stand with its
piston bore pointing up. The caliper body has a 40 mm piston with two slots in its face, two outer fingers above the
bore with a gap between them, and an open window in its bridge on the side facing the robot. The carrier's abutment
channels run out through the window at two levels. The piston stands out of its bore. Two new brake pads are started
in the channels just outside the window, one at each level. To the right a piston-return tool stands in a bench
holder: a two-pin adapter on an extension with a red ratcheting handle.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402
import tilt  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

IRON, CARRIER, CHROME = (0.42, 0.43, 0.45), (0.22, 0.22, 0.24), (0.80, 0.81, 0.83)
STAND = (0.30, 0.24, 0.18)
BACKING, LINING = (0.10, 0.10, 0.11), (0.45, 0.33, 0.22)
ZINC, DARK, RED = (0.74, 0.75, 0.77), (0.15, 0.15, 0.16), (0.82, 0.14, 0.12)
EPS = 1e-6

# --- caliper frame: origin on the table under the piston's axis; x away from the robot, z up -------------------
CAL_XY = (0.28, -0.06)
Z_IN = 0.063  # top of the inner pad's ledges (and the fully back piston face)
Z_FING = Z_IN + 0.043  # underside of the outer fingers: 16 + 10 (disc) + 16 mm of pads, 1 mm free
FING_T = 0.012
Z_OUT = Z_FING - 0.0065  # top of the outer pad's ledges (its backing plate 1 mm under the fingers)
PISTON_R, BORE_R = 0.020, 0.021
CROWN_H = 0.004  # face slots 4 mm deep
PISTON_L = 0.034
PROUD = 0.005  # the piston face starts 5 mm above the inner ledges (worn-pad position)
FACE0 = Z_IN + PROUD
NUT_R, NUT_H = 0.016, 0.010
NUT_Z = FACE0 - PISTON_L - NUT_H / 2  # adjuster nut centre at the start
LEAD = 0.002  # m of travel per turn of the adjuster thread
BACK_MAX = 0.012  # the adjuster's travel from the start to fully back (face 5 mm below the ledges)
FLOOR = 0.010  # bore floor / stand top

# Thread friction of the adjuster (self-locking): a breakaway torque. The nut's hinge drive holds it up to this
# torque, and its rest point follows the nut once it slips (on_step), so it acts as Coulomb friction, not a spring.
HOLD_TORQUE = 0.12  # N m
HOLD_K = 20.0  # N m/rad: stiffness of the stick phase (6 mrad of give at breakaway)

# Pads: 110 x 45 mm plates, 5.5 mm steel backing, 10.5 mm lining, ears 8 mm long and 20 mm wide on the backing.
PAD_L, PAD_W, BACK_T, LINE_T = 0.110, 0.045, 0.0055, 0.0105
LINE_L, LINE_W = 0.100, 0.040
EAR_L, EAR_W = 0.008, 0.020
PAD_IN_X = -0.060  # the pads start 60 mm out from their seated place, toward the robot
EAR_Y = PAD_L / 2 + EAR_L  # 63 mm: outer end of an ear
STOP_X = EAR_W / 2 + 0.0002  # abutment stop face (x), 0.2 mm in front of a seated ear
LEDGE_X0 = -0.078  # channels run out this far toward the robot


def _slab(x0, x1, y0, y1, z0, z1, color, **kw):
    return parts.box((x1 - x0, y1 - y0, z1 - z0), at=((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), color=color, **kw)


def _ring(r_in, r_out, z0, z1, color, n=16, phase=0.0):
    """n tangent slabs between r_in and r_out: a round bore of r_in (across flats)."""
    depth = r_out - r_in
    width = 2 * math.tan(math.pi / n) * r_out
    out = []
    for i in range(n):
        a = phase + i * 2 * math.pi / n
        c = r_in + depth / 2
        out.append(parts.box((depth, width, z1 - z0), at=(c * math.cos(a), c * math.sin(a), (z0 + z1) / 2),
                             color=color, yaw=a))
    return out


def _caliper_shapes():
    out = [_slab(-0.040, 0.040, -0.072, 0.072, 0.0, FLOOR, STAND)]  # bench stand block under the body
    out += _ring(BORE_R, 0.033, FLOOR, Z_IN - 0.002, IRON)  # piston housing round the 42 mm bore
    for s in (-1, 1):
        y0, y1 = sorted((s * 0.054, s * 0.064))
        w0, w1 = sorted((s * 0.064, s * 0.072))
        l0, l1 = sorted((s * 0.0555, s * 0.064))
        for z in (Z_IN, Z_OUT):  # carrier abutments at both pad levels: ledge, stop, lip over the ears
            out.append(_slab(LEDGE_X0, STOP_X + 0.006, y0, y1, z - 0.006, z, CARRIER))
            out.append(_slab(STOP_X, STOP_X + 0.006, *sorted((s * 0.057, s * 0.064)), z, z + 0.007, CARRIER))
            out.append(_slab(LEDGE_X0, STOP_X, l0, l1, z + BACK_T + 0.0015, z + BACK_T + 0.0045, CARRIER))
        # carrier side plate (channel ends) and caliper bridge sides, from the stand up past the fingers
        out.append(_slab(LEDGE_X0, STOP_X + 0.006, w0, w1, FLOOR, Z_FING + FING_T, IRON))
        out.append(_slab(-0.040, STOP_X + 0.006, w0, w1, 0.0, FLOOR, STAND))
        # side beams joining the fingers to the bridge, over the outer channel
        out.append(_slab(-0.040, 0.034, *sorted((s * 0.0555, s * 0.072)), Z_FING, Z_FING + FING_T, IRON))
        # the two outer fingers over the bore, 50 mm apart
        f0, f1 = sorted((s * 0.025, s * 0.0555))
        out.append(_slab(0.0, 0.034, f0, f1, Z_FING, Z_FING + FING_T, IRON))
        # the bridge's lower side members to the housing
        out.append(_slab(-0.040, 0.040, sorted((s * 0.030, s * 0.054))[0], sorted((s * 0.030, s * 0.054))[1],
                         FLOOR, Z_IN - 0.0065, IRON))
    return out + _caliper_looks()


def _caliper_looks():
    """Visual-only details (no colliders, no effect on the physics): the housing's round casting, the integral
    parking-brake lever and its cable bracket on the far side, the two slide pins with their boots, a bleed nipple."""
    v = dict(collide=False)
    rubber = (0.05, 0.05, 0.05)
    out = [parts.cylinder(0.0338, Z_IN - 0.002 - FLOOR, at=(0, 0, (FLOOR + Z_IN - 0.002) / 2), color=IRON, **v)]
    # parking-brake lever: pivot boss on the housing, arm out beyond the caliper, cable from a bracket to its end
    out.append(parts.cylinder(0.009, 0.022, at=(0.045, -0.035, FLOOR + 0.011), color=IRON, **v))
    out.append(_slab(0.040, 0.095, -0.041, -0.029, 0.026, 0.034, DARK, **v))
    out.append(parts.cylinder(0.004, 0.012, at=(0.090, -0.035, 0.036), color=ZINC, **v))  # cable eye pin
    out.append(_slab(0.040, 0.100, 0.025, 0.031, FLOOR, 0.042, DARK, **v))  # cable bracket, bolted to the body
    out.append(parts.cylinder(0.0015, 0.063, at=(0.090, -0.0035, 0.032), color=ZINC, axis="y", **v))  # inner cable
    out.append(parts.cylinder(0.004, 0.064, at=(0.090, 0.060, 0.032), color=rubber, axis="y", **v))  # cable sheath
    for s in (-1, 1):  # slide pins: lugs on the body, rubber boots, bolt heads
        y = s * 0.084
        out.append(_slab(-0.032, -0.008, *sorted((s * 0.072, s * 0.094)), 0.020, 0.050, IRON, **v))
        out.append(parts.cylinder(0.008, 0.024, at=(-0.020, y, 0.062), color=rubber, **v))
        out.append(parts.cylinder(0.0075, 0.007, at=(-0.020, y, 0.0775), color=ZINC, **v))
    # bleed nipple on a boss at the housing's far side, with its rubber cap
    out.append(_slab(0.033, 0.042, 0.012, 0.028, 0.030, 0.046, IRON, **v))
    out.append(parts.cylinder(0.006, 0.006, at=(0.0375, 0.020, 0.049), color=ZINC, **v))
    out.append(parts.cylinder(0.0035, 0.006, at=(0.0375, 0.020, 0.055), color=rubber, **v))
    return out


def _piston_shapes():
    zc = FACE0 - CROWN_H  # crown base
    out = [parts.cylinder(PISTON_R, PISTON_L - CROWN_H, at=(0, 0, FACE0 - PISTON_L + (PISTON_L - CROWN_H) / 2),
                          color=CHROME)]
    # the crown: two stepped half-discs leave a channel 9 mm wide across the face (along y); a boss in its middle
    # leaves two slots 10 mm long, open at the rim on opposite sides
    strips = [(0.0045, 0.008), (0.008, 0.011), (0.011, 0.015), (0.015, 0.018)]
    for s in (-1, 1):
        for y0, y1 in strips:
            hx = math.sqrt(PISTON_R ** 2 - y1 ** 2)
            out.append(_slab(*sorted((s * y0, s * y1)), -hx, hx, zc, FACE0, CHROME))
    out.append(_slab(-0.0045, 0.0045, -0.010, 0.010, zc, FACE0, CHROME))  # boss between the slots
    return out


FEED = parts.Slider("feed", shapes=[parts.box((0.004, 0.004, 0.004), at=(0, 0, NUT_Z), color=ZINC, collide=False)],
                    axis=(0.0, 0.0, 1.0), travel=0.002, back=BACK_MAX, mass=0.02, damping=0.0, thread=("nut", LEAD))
NUT = parts.Hinge("nut", shapes=[parts.cylinder(0.012, NUT_H - 0.002, at=(0, 0, NUT_Z - 0.001), color=ZINC),
                                 *_ring(0.012, NUT_R, NUT_Z + NUT_H / 2 - 0.002, NUT_Z + NUT_H / 2, ZINC, n=12)],
                  pivot=(0.0, 0.0, NUT_Z), axis=(0.0, 0.0, 1.0), lower=-60.0, upper=60.0, mass=0.06,
                  spring=HOLD_K, damping=0.01, max_effort=HOLD_TORQUE, parent="feed")
LIFT = parts.Slider("lift", shapes=[parts.box((0.004, 0.004, 0.004), at=(0, 0, FACE0 - 0.017), color=ZINC,
                                              collide=False)],
                    axis=(0.0, 0.0, 1.0), travel=0.001, back=BACK_MAX + 0.004, mass=0.02, damping=1.0)
PISTON = parts.Hinge("piston", shapes=_piston_shapes(), pivot=(0.0, 0.0, FACE0 - 0.017), axis=(0.0, 0.0, 1.0),
                     lower=-100.0, upper=100.0, mass=0.12, damping=0.002, parent="lift")
CALIPER = parts.Part("caliper", pos=(*CAL_XY, TOP), shapes=_caliper_shapes(), joints=[FEED, NUT, LIFT, PISTON],
                     contact_offset=0.0005)


def _pad_shapes(outer: bool):
    """Pad frame: origin at the centre of the backing plate's underside (inner pad) or top (outer pad)."""
    if not outer:  # backing plate down on the ledges, lining up
        zb, zl = BACK_T / 2, BACK_T + LINE_T / 2
    else:  # lining down, backing plate up against the fingers; the ears are on the backing
        zb, zl = -BACK_T / 2, -BACK_T - LINE_T / 2
    out = [parts.box((PAD_W, PAD_L, BACK_T), at=(0, 0, zb), color=BACKING),
           parts.box((LINE_W, LINE_L, LINE_T), at=(0, 0, zl), color=LINING)]
    for s in (-1, 1):
        out.append(parts.box((EAR_W, EAR_L, BACK_T), at=(0, s * (PAD_L / 2 + EAR_L / 2), zb), color=BACKING))
    return out


PAD_IN = parts.Part("inner_pad", pos=(CAL_XY[0] + PAD_IN_X, CAL_XY[1], TOP + Z_IN), mass=0.20,
                    shapes=_pad_shapes(False), contact_offset=0.0005)
PAD_OUT = parts.Part("outer_pad", pos=(CAL_XY[0] + PAD_IN_X, CAL_XY[1], TOP + Z_OUT + BACK_T), mass=0.20,
                     shapes=_pad_shapes(True), contact_offset=0.0005)

# --- the piston-return tool (loose) and its holder (fixed) -------------------------------------------------------
ADAPT_R, ADAPT_H = 0.020, 0.008
PIN = (0.004, 0.004, 0.0035)
PIN_X = 0.013
EXT_R, EXT_TOP = 0.006, 0.065
HANDLE_AF, HANDLE_L = 0.032, 0.090  # octagonal handle, 32 mm across flats
HANDLE_PHASE = math.radians(10.0)  # a pair of flats faces the fingers at the pick yaw
TOOL_H = EXT_TOP + HANDLE_L
RATCHET_SLACK = 0.02
SHAFT = parts.Hinge(
    "shaft", pivot=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0), lower=-80.0, upper=RATCHET_SLACK, mass=0.03,
    damping=1e-4, shapes=[
        parts.cylinder(ADAPT_R, ADAPT_H, at=(0, 0, ADAPT_H / 2), color=ZINC),
        parts.box(PIN, at=(PIN_X, 0, -PIN[2] / 2), color=DARK),
        parts.box(PIN, at=(-PIN_X, 0, -PIN[2] / 2), color=DARK),
        parts.cylinder(EXT_R, EXT_TOP - ADAPT_H, at=(0, 0, ADAPT_H + (EXT_TOP - ADAPT_H) / 2), color=ZINC),
    ])
HOLDER_XY = (0.30, -0.215)
LEDGE_H, WALL_H = 0.030, 0.060
TOOL = parts.Part(
    "tool", pos=(*HOLDER_XY, TOP + LEDGE_H), mass=0.13, contact_offset=0.0003, self_collision=False,
    shapes=[*(parts.box((HANDLE_AF, HANDLE_AF * math.tan(math.pi / 8), HANDLE_L), at=(0, 0, EXT_TOP + HANDLE_L / 2),
                        color=RED, yaw=HANDLE_PHASE + k * math.pi / 4) for k in range(4)),
            parts.cylinder(0.006, 0.004, at=(0, 0, TOOL_H + 0.002), color=DARK)],
    joints=[SHAFT],
)
_pawl = {"upper": None}


def _write_pawl(sim, up) -> None:
    import torch

    base = sim.env.unwrapped
    tool = base.scene["tool"]
    ids = tool.find_joints(["shaft"], preserve_order=True)[0]
    lim = torch.tensor([[[SHAFT.lower, up]]], dtype=torch.float32, device=base.device)
    tool.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


@parts.on_step
def _ratchet(sim) -> None:
    """The ratchet pawl: the adapter may turn anticlockwise in the handle (q falling) but drives clockwise."""
    q = sim.states()["tool"].joints["shaft"]
    up = min(RATCHET_SLACK if _pawl["upper"] is None else _pawl["upper"], q + RATCHET_SLACK)
    if _pawl["upper"] is not None and up >= _pawl["upper"] - 1e-4:
        return
    _pawl["upper"] = up
    _write_pawl(sim, up)


_thread = {"rest": 0.0}


@parts.on_step
def _thread_friction(sim) -> None:
    """Self-locking thread: the nut's drive rest point stays put while the nut is within breakaway of it, and is
    dragged along behind the nut once it slips."""
    q = sim.states()["caliper"].joints["nut"]
    give = HOLD_TORQUE / HOLD_K
    rest = min(max(_thread["rest"], q - give), q + give)
    if abs(rest - _thread["rest"]) > 1e-7:
        _thread["rest"] = rest
        sim.set_target("caliper", "nut", rest)


@parts.on_reset
def _mech_reset(sim) -> None:
    _pawl["upper"] = RATCHET_SLACK
    _write_pawl(sim, RATCHET_SLACK)
    _thread["rest"] = 0.0
    sim.set_target("caliper", "nut", 0.0)


def _holder_shapes():
    black = (0.22, 0.22, 0.24)
    return (_ring(0.016, 0.023, 0.0, LEDGE_H, black, n=12, phase=math.pi / 12)
            + _ring(0.023, 0.029, 0.0, WALL_H, black, n=12, phase=math.pi / 12))


HOLDER = parts.Part("tool_holder", pos=(*HOLDER_XY, TOP), shapes=_holder_shapes())

OBJECTS = [CALIPER, PAD_IN, PAD_OUT, TOOL, HOLDER]

# --- scoring ---------------------------------------------------------------------------------------------------
SEAT_Z = 0.001  # ears on their ledges: within 1 mm
SEAT_X = 0.0015  # ears against the abutment stops: within 1.5 mm
SEAT_Y = 0.0015
FLAT_DEG = 3.0
CLEAR = 0.005

_grip = parts.__dict__.setdefault("_caliper_task_grip", {"clear": None})


def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def piston_face(objects) -> float:
    """Height of the piston face above the table (m)."""
    return FACE0 + objects["caliper"].joints.get("lift", 0.0)


def _tilt_deg(q) -> float:
    return math.degrees(math.acos(float(np.clip((_rot(q) @ np.array([0.0, 0.0, 1.0]))[2], -1.0, 1.0))))


def _yaw_deg(q) -> float:
    a = _rot(q) @ np.array([0.0, 1.0, 0.0])
    return math.degrees(math.atan2(-a[0], a[1]))


def pad_seated(objects, name: str) -> bool:
    pad = objects[name]
    z_ledge = TOP + (Z_IN if name == "inner_pad" else Z_OUT + BACK_T)
    p = np.asarray(pad.pos, float)
    x_seat, y_seat = CAL_XY
    return bool(abs(p[2] - z_ledge) <= SEAT_Z + EPS and abs(p[0] - x_seat) <= SEAT_X + EPS
                and abs(p[1] - y_seat) <= SEAT_Y + EPS and _tilt_deg(pad.quat_xyzw) <= FLAT_DEG + EPS
                and abs(_yaw_deg(pad.quat_xyzw)) <= FLAT_DEG + EPS and pad.resting())


def _scored_boxes(objects):
    """(centre, rotation, size) of the piston's shapes and both pads' shapes, world frame."""
    cal = objects["caliper"]
    poses = {"piston": tuple((np.asarray(cal.links["piston"][0]), _rot(cal.links["piston"][1])))}
    out = [(c, r, s) for _, c, r, s in CALIPER.world_boxes(poses)]
    for spec in (PAD_IN, PAD_OUT):
        st = objects[spec.name]
        out += [(c, r, s) for _, c, r, s in spec.world_boxes({"base": (np.asarray(st.pos), _rot(st.quat_xyzw))})]
    return out


def _tool_clear(objects) -> bool:
    t = objects["tool"]
    tposes = {"base": (np.asarray(t.pos), _rot(t.quat_xyzw))}
    tposes.update({k: (np.asarray(p), _rot(q)) for k, (p, q) in t.links.items()})
    tool_boxes = TOOL.world_boxes(tposes)
    for c, r, s in _scored_boxes(objects):
        for _, tc, tr, ts in tool_boxes:
            lo1, hi1 = c - np.abs(r) @ (np.asarray(s) / 2), c + np.abs(r) @ (np.asarray(s) / 2)
            lo2, hi2 = tc - np.abs(tr) @ (np.asarray(ts) / 2), tc + np.abs(tr) @ (np.asarray(ts) / 2)
            gap = np.maximum(np.maximum(lo2 - hi1, lo1 - hi2), 0.0)
            if np.linalg.norm(gap) < CLEAR - EPS:
                return False
    return True


def _done(objects) -> float:
    """0.5 once the piston face is back to the inner ledges, plus 0.25 per pad seated; released and still."""
    if _grip["clear"] is not True or not _tool_clear(objects):
        return 0.0
    if piston_face(objects) > Z_IN + EPS:
        return 0.0
    return 0.5 + 0.25 * sum(pad_seated(objects, n) for n in ("inner_pad", "outer_pad"))


def success(objects):
    """Both pads seated in the carrier abutments with the piston back, everything let go and still."""
    return bool(_done(objects) == 1.0)


def score(objects):
    return _done(objects)


@parts.on_step
def _measure_grip(sim) -> None:
    from yam_arena import analysis

    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    boxes = _scored_boxes(sim.states())
    hulls = []
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in ("tip_left", "tip_right", "gripper"):
                m = np.eye(4)
                m[:3, :3] = _rot(poses[i, 3:])
                m[:3, 3] = poses[i, :3] - origin
                hulls += analysis.world_hulls({n: m})
    _grip["clear"] = bool(hulls) and all(analysis.hulls_to_box(hulls, c, r, s)[0] > CLEAR for c, r, s in boxes)


@parts.on_reset
def _reset_grip(sim) -> None:
    _grip["clear"] = None


# --- demo: open loop from the published start layout -------------------------------------------------------------
TIP = 0.013  # finger tips below the TCP
PALM = 0.0695  # the gripper's palm above the TCP: held with the palm just over the handle's cap, which takes the push
TIP_BELOW_TCP = TOOL_H + 0.004 - PALM  # adapter face below the TCP while held
HANDLE_GRIP = -0.0115  # pads sent to 23 mm apart on the 32 mm flats, about 4.5 mm past each face
HANDLE_LOOSE = -0.025
YAW_A, YAW_MID, YAW_B = math.radians(145.0), math.radians(50.0), math.radians(-45.0)
YAW_PICK = YAW_A - math.pi / 2  # picked a quarter turn short, so the pins lie along y at YAW_A
STROKE = math.radians(190.0)
N_STROKES = 20  # 20 x 190 deg = 10.6 turns commanded for the 2.5 needed (the clutch and the soft wrist lose some)
PRESS = 0.050  # commanded below the seated pose: the soft arm's push


def _tool_plan():
    hx, hy = HOLDER_XY
    cx, cy = CAL_XY
    pick_z = TOP + LEDGE_H + TIP_BELOW_TCP - 0.002  # the palm pressed 2 mm onto the cap, so the tool cannot slide down
    carry = TOP + 0.165
    over = TOP + 0.210  # open fingers clear of the standing tool's cap
    face = TOP + FACE0
    seat = face + TIP_BELOW_TCP  # adapter resting on the face, pins in the slots
    low = seat + 0.010  # pins 6.5 mm over the face (the held tool rides a few mm low)
    # the held adapter was measured to settle 0.8 mm short in x and turned 6 degrees short (the ratchet's play)
    # at this pose, open loop: aim that much past
    ax, ay = cx + 0.0008, cy + 0.0014
    yaw_s = YAW_A + math.radians(6.0)
    side = cx + 0.060  # clear of the caliper's far end (stand 40 mm, adapter 20 mm)
    unhook = dict(tilt=30.0, heading=math.radians(-36.0))
    upright = dict(tilt=23.0, heading=math.radians(-45.0))
    rx, ry = hx + 0.015, hy - 0.013  # the held tool was measured to hang this far short of the hand at this lean
    w = [
        Waypoint(2.0, (hx, hy, over), HANDLE_LOOSE, "over the tool, fingers above its cap", YAW_PICK),
        Waypoint(1.5, (hx, hy, pick_z), HANDLE_LOOSE, "down round the handle", YAW_PICK),
        Waypoint(0.8, (hx, hy, pick_z), HANDLE_GRIP, "grip the handle", YAW_PICK),
        Waypoint(2.0, (hx, hy, over - 0.010), HANDLE_GRIP, "lift it out", YAW_PICK),
        # the caliper's walls stand higher than the arm can carry the tool over the piston, so the adapter goes in
        # from the far side, low, through the gap between the fingers
        Waypoint(2.0, (side, hy, carry), HANDLE_GRIP, "beyond the caliper", YAW_PICK),
        Waypoint(2.0, (side, cy, carry), HANDLE_GRIP, "behind the finger gap", YAW_PICK),
        Waypoint(1.5, (side, cy, carry), HANDLE_GRIP, "turn the pins along the slots", yaw_s),
        Waypoint(1.5, (side, cy, low), HANDLE_GRIP, "down, adapter just over the face height", yaw_s),
        Waypoint(2.5, (ax, ay, low), HANDLE_GRIP, "in through the gap", yaw_s),
        Waypoint(1.5, (ax, ay, seat), HANDLE_GRIP, "pins into the slots", yaw_s),
    ]
    w.append(Waypoint(1.0, (ax, ay, seat - PRESS), HANDLE_GRIP, "press in", YAW_A))
    per = LEAD * STROKE / (2 * math.pi)  # the face recedes about this much per stroke
    z = seat - PRESS
    for k in range(N_STROKES):
        z_end = z - per
        w += [Waypoint(1.25, (ax, ay, (z + z_end) / 2), HANDLE_GRIP, f"turn {k + 1}", YAW_MID),
              Waypoint(1.25, (ax, ay, z_end), HANDLE_GRIP, f"turn {k + 1}", YAW_B)]
        z = z_end
        if k < N_STROKES - 1:
            w += [Waypoint(0.75, (ax, ay, z), HANDLE_GRIP, f"swing back {k + 1}", YAW_MID),
                  Waypoint(0.75, (ax, ay, z), HANDLE_GRIP, f"swing back {k + 1}", YAW_A)]
    w += [
        Waypoint(1.0, (ax, ay, seat - PROUD - 0.001), HANDLE_GRIP, "ease the push", YAW_B),
        Waypoint(1.0, (ax, ay, seat - PROUD + 0.005), HANDLE_GRIP, "pins out of the slots", YAW_B),
        Waypoint(2.5, (side, cy, seat - PROUD + 0.005), HANDLE_GRIP, "out through the gap", YAW_B),
        # the turning leaves the tool leaning in the grip with its adapter back under a finger; lean the hand so the
        # adapter swings out past the fingers' ends before lifting (open loop: the lean was measured on a trace)
        tilt.Waypoint(1.5, (side, cy, seat - PROUD + 0.005), HANDLE_GRIP, "swing the adapter clear", YAW_B, **unhook),
        tilt.Waypoint(1.5, (side, cy, carry), HANDLE_GRIP, "up", YAW_B, **unhook),
        # and carry it with the hand leaned the other way, so the tool hangs upright over its holder
        tilt.Waypoint(2.0, (side, hy, carry), HANDLE_GRIP, "beyond the caliper", YAW_B, **upright),
        tilt.Waypoint(2.0, (rx, ry, over - 0.010), HANDLE_GRIP, "over its holder", YAW_B, **upright),
        tilt.Waypoint(2.0, (rx, ry, pick_z + 0.004), HANDLE_GRIP, "lower it in", YAW_B, **upright),
        tilt.Waypoint(0.8, (rx, ry, pick_z + 0.004), HANDLE_LOOSE, "let go", YAW_B, **upright),
        tilt.Waypoint(1.5, (rx, ry, over), HANDLE_LOOSE, "up clear of its cap", YAW_B, **upright),
    ]
    return w


def _duration(ws):
    return sum(x.duration for x in ws)


def _pad_push():
    """Once the tool is back in its holder the right arm stays clear, and the left arm slides the outer pad home, then
    the inner pad, with its closed fingers leaning finger tips forward so only the tips touch a pad's edge (the fingers
    widen higher up). For the inner pad it leans 45 degrees, so the hand stays behind the seated outer pad's edge."""
    cx, cy = CAL_XY
    yaw = -math.pi / 2  # fingers close along x: their thin side leads

    def w(d, x, z, label, lean):
        return tilt.Waypoint(d, (x, cy, z), 0.0, label, yaw, tilt=lean, heading=0.0)

    edge0 = cx + PAD_IN_X - PAD_W / 2  # the pads' near edges at the start
    seat_edge = cx - PAD_W / 2
    # outer pad: 20 degrees; the tips' front face is 7.5 mm ahead of the TCP and 13 mm x cos(20) below it
    lead_o = 0.0075
    start_o = edge0 - lead_o - 0.008
    home_o = seat_edge - lead_o + 0.020  # 20 mm past seated: the soft arm stops well short of where it is sent here
    z_out = TOP + Z_OUT - 0.008 + TIP
    # inner pad: 45 degrees; the tips swing 9 mm forward and sit 9 mm below the TCP
    lead_i = 0.014
    start_i = edge0 - lead_i - 0.008
    home_i = seat_edge - lead_i + 0.030
    z_in = TOP + Z_IN + 0.009 + TIP * math.cos(math.radians(45.0))  # tips 9 mm up the pad's 16 mm edge
    hi = TOP + 0.150
    return [
        w(2.0, start_o, hi, "behind the pads", 20.0),
        w(1.5, start_o, z_out, "down to the outer pad", 20.0),
        w(3.0, home_o, z_out, "slide it home", 20.0),
        w(1.0, start_o, z_out, "back off", 20.0),
        w(1.5, start_i, z_out, "lean further", 45.0),
        w(1.0, start_i, z_in, "down to the inner pad", 45.0),
        w(3.0, home_i, z_in, "slide it home", 45.0),
        w(1.0, start_i, z_in, "back off", 45.0),
        w(1.5, start_i - 0.03, hi, "up and clear", 45.0),
        Waypoint(2.0, (0.20, 0.25, TOP + 0.20), -0.010, "away to the left"),
        Waypoint(3.0, (0.20, 0.25, TOP + 0.20), -0.010, "wait clear"),
    ]


def demo():
    tool = _tool_plan()
    push = _pad_push()
    right = tool + [Waypoint(2.0, (0.20, -0.26, TOP + 0.20), -0.010, "away to the right"),
                    Waypoint(_duration(push), (0.20, -0.26, TOP + 0.20), -0.010, "wait clear")]
    left = [Waypoint(3.0, (0.20, 0.25, TOP + 0.20), -0.010, "stay clear"),
            Waypoint(_duration(tool) + 2.0 - 3.0, (0.20, 0.25, TOP + 0.20), -0.010, "wait")] + push
    return {"left": left, "right": right}
