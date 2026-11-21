"""At the station table: a car's back at the bench's left end, a 2 in hitch ball on a ball mount in its receiver, and a
small utility trailer's tongue running from the ball through its jack to the trailer's front at the bench's right end.
On the tongue's end is the coupler, CURT 25153 (2 in straight-tongue coupler with Posi-Lock latch, 3,500 lb, zinc,
27 cm long, 5 cm wide); the tongue pivots at the jack. The coupler hovers 25 mm above the ball, its yellow latch handle
up (unlatched), as CURT's hook-up sheet says to start. Car, ball mount, jack, trailer front and chains are drawn only. The instruction asks only to hitch the trailer. Hitched means: the coupler pushed down until the
ball is fully seated in its socket, THEN the handle pushed down until it latches, which drives the underjaw in under the
ball. Latching first (the handle down while the coupler still hovers) jams the underjaw against the ball; the handle
stops part-way and cannot be finished from there. A latch closed with the ball not in the socket scores 0.
Score at the end: 1.0 seated and latched, at rest, gripper 3 cm clear, for 1 s; 0.5 seated only (at rest 1 s);
0.25 coupler lowered onto the ball but not seated; 0 otherwise or if latched with the ball not seated."""
from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import hhdebug  # noqa: E402
import hhkit  # noqa: E402
import mechkit as m  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Hitch the trailer to the ball."
VARIANT = hhkit.demo_variant()

LOOKS = {
    "zinc": m.Look(m.rgb(0.78, 0.80, 0.82), metallic=0.8, roughness=0.3),
    "ball": m.Look(m.rgb(0.55, 0.57, 0.60), metallic=0.9, roughness=0.2),
    "base": m.Look(m.rgb(0.15, 0.15, 0.17), roughness=0.6),
    "handle": m.Look(m.rgb(0.95, 0.80, 0.05), roughness=0.5),
    "jaw": m.Look(m.rgb(0.85, 0.25, 0.10), roughness=0.5),
    "tongue": m.Look(m.rgb(0.20, 0.35, 0.60), roughness=0.5),
    "black": m.Look(m.rgb(0.06, 0.06, 0.07), roughness=0.55),
    "powder": m.Look(m.rgb(0.10, 0.10, 0.11), metallic=0.3, roughness=0.45),
    "paint": m.Look(m.rgb(0.55, 0.05, 0.06), metallic=0.5, roughness=0.25),
    "lamp": m.Look(m.rgb(0.85, 0.08, 0.06), roughness=0.2),
    "amber": m.Look(m.rgb(0.95, 0.55, 0.10), roughness=0.2),
    "plate_w": m.Look(m.rgb(0.93, 0.93, 0.90), roughness=0.4),
    "glass": m.Look(m.rgb(0.05, 0.06, 0.08), metallic=0.4, roughness=0.1),
    "wood": m.Look(m.rgb(0.52, 0.40, 0.27), roughness=0.8),
    "chain": m.Look(m.rgb(0.70, 0.72, 0.74), metallic=0.8, roughness=0.35),
}
BX, BY = 0.36, 0.22  # the ball; (0.40, 0.16) in the first build: moved toward the left arm for the straight-down reach to the open handle
# v6 (realism): the whole hitch is turned a quarter turn (yaw -90 deg) so the car can be at the bench's left end and the
# trailer at its right end. The mechanism's own +x (from the ball toward the trailer) now points along world -y; its
# own -x (from the ball toward the car) along world +y.
YAW_DEG = -90.0
BALL_DY = 0.0  # the ball sits under the socket (a probe moved it 45 mm aside to test the latched-not-seated rule)
BZ = 0.070
R = 0.0254
SHANK_R = 0.0095
POCKET_IN = 0.0262
TOPPL = BZ + R + 0.001 + 0.004  # coupler top plate's upper face (local z, seated)
HOVER = 0.0249
START_DEG = 6.5
PIV = (0.22, 0.0, BZ + 0.0284)
HP = (0.05, 0.0, BZ + 0.0344)  # handle hinge
HANDLE_L = 0.055
HANDLE_OPEN = 60.0
JAW_TRAVEL = 0.065
SEAT_TOL_DEG, SEAT_XY = 0.5, 0.0015
RELEASE_M = 0.03  # every gripper point this far from the coupler for full credit
LATCH_DEG, JAW_TOL = 3.0, 0.002


def _head():
    parts, cols = [], []
    pl = m.box((0.075, 0.052, 0.004), (0, 0, BZ + R + 0.001 + 0.002))
    parts.append(("plate", pl, "zinc")); cols.append(m.col_hull("c_plate", pl))
    for k in range(12):
        a = 30.0 * k
        seg = m.box((0.008, 0.0166, 0.0299), (POCKET_IN + 0.004, 0, BZ + 0.01145)).turned(a, "z")
        parts.append((f"ring{k}", seg, "zinc")); cols.append(m.col_hull(f"c_ring{k}", seg))
        L = 0.016
        fun = m.box((0.003, 0.0185, L), (0, 0, 0)).turned(-40.0, "y").moved((POCKET_IN + (L / 2) * math.sin(math.radians(40)), 0, BZ - 0.0035 - (L / 2) * math.cos(math.radians(40)))).turned(a, "z")
        parts.append((f"fun{k}", fun, "zinc")); cols.append(m.col_hull(f"c_fun{k}", fun))
    bar = m.box((0.20, 0.034, 0.008), (0.1375, 0, BZ + 0.0284))
    parts.append(("bar", bar, "tongue")); cols.append(m.col_hull("c_bar", bar))
    # drawn only (no colliders): the coupler's channel body over the tongue's end, the 2 x 2 in tongue beam to the
    # jack, and the two safety chains hanging off the coupler, not yet hooked to the car (they go on after latching)
    parts.append(("sleeve", m.box((0.085, 0.058, 0.052), (0.1025, 0, BZ + 0.0284)), "zinc"))
    parts.append(("beam", m.box((0.0925, 0.050, 0.050), (0.1913, 0, BZ + 0.0284)), "tongue"))
    for k, sy in (("l", 1.0), ("r", -1.0)):
        pts = [(0.125, sy * 0.032, BZ + 0.012), (0.118, sy * 0.040, BZ - 0.020), (0.105, sy * 0.050, 0.030),
               (0.085, sy * 0.058, 0.014), (0.055, sy * 0.066, 0.010), (0.020, sy * 0.072, 0.010), (-0.010, sy * 0.076, 0.012)]
        parts.append((f"chain_{k}", m.tube(pts, 0.0035, sides=8), "chain"))
        parts.append((f"hook_{k}", m.box((0.018, 0.005, 0.016), (-0.020, sy * 0.077, 0.016)), "chain"))
    return parts, cols


_HP, _HC = _head()
TONGUE = m.hinge("tongue", "Y", PIV, 0.0, 10.0, tuple(_HP), tuple(_HC), mass=0.35, damping=0.3, friction=0.4, effort=100.0)

_hb = m.box((HANDLE_L, 0.016, 0.008), (HP[0] - HANDLE_L / 2, 0, HP[2]))
_pad = m.box((0.016, 0.030, 0.006), (HP[0] - HANDLE_L, 0, HP[2] + 0.001))
HANDLE = m.hinge("handle", "Y", HP, 0.0, HANDLE_OPEN, (("hb", _hb, "handle"), ("pad", _pad, "handle")),
                 (m.col_hull("c_hb", _hb), m.col_hull("c_pad", _pad)), mass=0.05, damping=0.2, friction=0.08, parent="tongue", effort=50.0)

_tz = BZ - 0.0300
_t1 = m.box((0.047, 0.008, 0.003), (-0.0085 + JAW_TRAVEL, 0.0145, _tz))
_t2 = m.box((0.047, 0.008, 0.003), (-0.0085 + JAW_TRAVEL, -0.0145, _tz))
_tb = m.box((0.004, 0.037, 0.003), (0.017 + JAW_TRAVEL, 0, _tz))
JAW = m.slider("jaw", "X", -JAW_TRAVEL, 0.0, (("t1", _t1, "jaw"), ("t2", _t2, "jaw"), ("tb", _tb, "jaw")),
               (m.col_hull("c_t1", _t1), m.col_hull("c_t2", _t2), m.col_hull("c_tb", _tb)), mass=0.03, spring=4000.0, damping=40.0, friction=0.0, parent="tongue", effort=100.0)


CAR_X = -0.40  # the car's rear bumper face, in the hitch's own x (world y 0.62, the bench's left end)
TRAILER_X = 0.86  # the trailer's front rail, in the hitch's own x (world y -0.64, the bench's right end)


def _frame():
    shank = m.cylinder(SHANK_R, BZ - 0.012, (0, BALL_DY, 0.012 + (BZ - 0.012) / 2))
    ball = m.sphere(R, (0, BALL_DY, BZ))
    parts = [("shank", shank, "ball"), ("ball", ball, "ball")]
    # drawn only (no colliders). The car: a ball mount (2 in square shank, platform under the ball, hitch pin) in a
    # 2 in receiver under the rear bumper of a car whose back stands at the bench's left end. Real size, cut off at the
    # sides (a car is 1.8 m wide; 0.84 m of its back is drawn) and above the tailgate's lower half.
    parts += [("bm_platform", m.box((0.080, 0.064, 0.012), (-0.008, 0, 0.024)), "zinc"),
              ("bm_shank", m.box((0.250, 0.050, 0.050), (-0.180, 0, 0.025)), "powder"),
              ("bm_nut", m.cylinder(0.017, 0.010, (0, 0, 0.013)), "zinc"),
              ("receiver", m.box((0.120, 0.064, 0.064), (CAR_X + 0.060, 0, 0.032)), "black"),
              ("hitch_pin", m.cylinder(0.008, 0.090, (CAR_X + 0.040, 0, 0.032), axis="y"), "zinc"),
              ("crossmember", m.box((0.050, 0.70, 0.045), (CAR_X - 0.010, 0, 0.040)), "black"),
              ("bumper", m.box((0.130, 0.84, 0.130), (CAR_X - 0.065, 0, 0.125)), "black"),
              ("fascia", m.box((0.120, 0.84, 0.060), (CAR_X - 0.075, 0, 0.220)), "paint"),
              ("tailgate", m.box((0.090, 0.84, 0.220), (CAR_X - 0.105, 0, 0.360)), "paint"),
              ("rear_glass", m.box((0.010, 0.70, 0.050), (CAR_X - 0.062, 0, 0.445)), "glass"),
              ("plate", m.box((0.006, 0.300, 0.150), (CAR_X - 0.058, 0, 0.330)), "plate_w"),
              ("plate_text", m.box((0.004, 0.240, 0.050), (CAR_X - 0.055, 0, 0.330)), "glass"),
              ("lamp_l", m.box((0.010, 0.120, 0.110), (CAR_X - 0.058, 0.350, 0.330)), "lamp"),
              ("lamp_r", m.box((0.010, 0.120, 0.110), (CAR_X - 0.058, -0.350, 0.330)), "lamp"),
              ("reflector_l", m.box((0.010, 0.060, 0.020), (CAR_X - 0.002, 0.360, 0.150)), "amber"),
              ("reflector_r", m.box((0.010, 0.060, 0.020), (CAR_X - 0.002, -0.360, 0.150)), "amber")]
    # the trailer: the tongue beam runs on from the jack to an A-frame and the front of a small utility trailer
    # (front rail and a wooden front board) at the bench's right end, at real size, cut off at the bench's edge
    zt = BZ + 0.0284
    parts += [("beam_fixed", m.box((TRAILER_X - 0.2375, 0.050, 0.050), ((TRAILER_X + 0.2375) / 2, 0, zt)), "tongue"),
              ("jack_tube", m.cylinder(0.022, 0.210, (PIV[0], 0.050, 0.105)), "powder"),
              ("jack_foot", m.cylinder(0.045, 0.008, (PIV[0], 0.050, 0.004)), "powder"),
              ("jack_cap", m.cylinder(0.026, 0.030, (PIV[0], 0.050, 0.225)), "black"),
              ("jack_crank", m.box((0.090, 0.010, 0.010), (PIV[0] + 0.045, 0.050, 0.245)), "zinc"),
              ("jack_knob", m.cylinder(0.008, 0.030, (PIV[0] + 0.090, 0.050, 0.262)), "black"),
              ("jack_bracket", m.box((0.050, 0.030, 0.050), (PIV[0], 0.032, zt)), "powder"),
              ("front_rail", m.box((0.050, 0.66, 0.050), (TRAILER_X, 0, zt)), "tongue"),
              ("front_board", m.box((0.020, 0.66, 0.240), (TRAILER_X + 0.035, 0, zt + 0.145)), "wood")]
    for k, sy in (("l", 1.0), ("r", -1.0)):  # A-frame braces from the beam to the front rail's corners
        a, b = np.array([0.56, 0.0]), np.array([TRAILER_X - 0.02, sy * 0.30])
        L = float(np.linalg.norm(b - a))
        ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
        brace = m.box((L, 0.040, 0.040), (0, 0, 0)).turned(ang, "z").moved(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, zt))
        parts.append((f"aframe_{k}", brace, "tongue"))
    # the old 14 x 12 cm base stays as an (undrawn) collider under the ball mount
    cols = [m.col_box("c_base", (0.14, 0.12, 0.012), (0, BALL_DY, 0.006)), m.col_cylinder("c_shank", SHANK_R, BZ - 0.012, (0, BALL_DY, 0.012 + (BZ - 0.012) / 2)), m.col_hull("c_ball", ball)]
    return parts, cols


_FP, _FC = _frame()
COUPLER = m.mechanism("coupler", (BX, BY, TOP), [TONGUE, HANDLE, JAW], LOOKS, frame_parts=_FP, frame_colliders=_FC, yaw_deg=YAW_DEG,
                      start=(("tongue", math.radians(START_DEG)), ("handle", math.radians(HANDLE_OPEN)), ("jaw", 0.0)))
OBJECTS = [COUPLER]


def _clear(mem):
    mem.update(th=START_DEG, hd=HANDLE_OPEN, jaw=0.0, lowered=False, seated=False, latched=False, captured=False, held=0, held_seat=0, jam=False, clear=True, minth=99.0)


TRIAL = hhkit.Trial(on_clear=_clear)


@m.on_reset
def _reset(env):
    import torch
    asset = env.unwrapped.scene["coupler"]
    for nm, val in (("tongue", math.radians(START_DEG)), ("handle", math.radians(HANDLE_OPEN)), ("jaw", 0.0)):
        j = asset.find_joints(f"{nm}_joint")[0][0]
        asset.write_joint_state_to_sim(torch.tensor([[val]], device=asset.device), torch.zeros((1, 1), device=asset.device), joint_ids=[j])
    m.set_target(env, "coupler", "jaw", 0.0)


@m.on_step
def _drive(env):
    if not TRIAL.mem:
        return
    st = m.state(env, "coupler")
    hd = math.degrees(st.joint("handle"))
    m.set_target(env, "coupler", "jaw", -JAW_TRAVEL * min(max((HANDLE_OPEN - hd) / (HANDLE_OPEN - 2.0), 0.0), 1.0))


def pocket_offset(th_rad):
    d = np.array([-PIV[0], 0.0, BZ - PIV[2]])
    c, s = math.cos(th_rad), math.sin(th_rad)
    x = PIV[0] + d[0] * c + d[2] * s
    z = PIV[2] + (-d[0]) * s + d[2] * c
    return x, z  # pocket centre (local x, z)


def to_world(x, y, z=0.0):
    """A point in the hitch's own frame (x from the ball toward the trailer) -> the env frame (yaw YAW_DEG about the ball)."""
    c, s = math.cos(math.radians(YAW_DEG)), math.sin(math.radians(YAW_DEG))
    return np.array([BX + c * x - s * y, BY + s * x + c * y, TOP + z])


# a box round coupler, handle and tongue: centre (local (0.10, 0, BZ + 0.03)) and size in the env frame (turned)
COUPLER_BOX = (tuple(to_world(0.10, 0.0, BZ + 0.03) - np.array([BX, BY, TOP])), (0.07, 0.30, 0.10))


def _clear_of_coupler(st) -> bool:
    c = np.asarray(st.pos, float) + np.asarray(COUPLER_BOX[0])
    return all(hhkit.point_box(p, c, COUPLER_BOX[1]) >= RELEASE_M for g in TRIAL.grippers() for p in (g["tcp"], *g["tips"]))


def _track(objects):
    if not TRIAL.once():
        return
    mm = TRIAL.mem
    st = objects["coupler"]
    th, hd, jaw = math.degrees(st.joint("tongue")), math.degrees(st.joint("handle")), float(st.joint("jaw"))
    px, pz = pocket_offset(math.radians(th))
    off = math.hypot(px - 0.0, 0.0 - BALL_DY)
    mm.update(th=round(th, 2), hd=round(hd, 1), jaw=round(jaw, 4), px=round(px, 4), pz=round(pz - BZ, 4), off=round(off, 4))
    mm["minth"] = min(mm["minth"], th)
    mm["lowered"] = th <= 3.0
    mm["seated"] = th <= SEAT_TOL_DEG and off <= SEAT_XY
    mm["latched"] = hd <= LATCH_DEG and jaw <= -JAW_TRAVEL + JAW_TOL
    mm["captured"] = mm["seated"] and mm["latched"]
    mm["jam"] = hd <= LATCH_DEG and jaw > -JAW_TRAVEL + JAW_TOL
    mm["clear"] = _clear_of_coupler(st)
    rest = float(np.linalg.norm(st.lin_vel)) < 0.03
    mm["held"] = mm["held"] + 1 if (mm["captured"] and rest and mm["clear"]) else 0
    mm["held_seat"] = mm["held_seat"] + 1 if (mm["seated"] and rest) else 0
    hhdebug.log(TRIAL, objects, [])


def score(objects):
    _track(objects)
    mm = TRIAL.mem
    if mm["held"] >= TRIAL.frames(1.0):
        return 1.0
    if mm["latched"] and not mm["seated"]:
        return 0.0
    if mm["held_seat"] >= TRIAL.frames(1.0):
        return 0.5
    return 0.25 if mm["lowered"] else 0.0


def success(objects):
    return score(objects) >= 1.0


def distance(objects):
    _track(objects)
    return float(TRIAL.mem["th"]) * 0.01 + float(TRIAL.mem["hd"]) * 0.001


REST_L = (0.20, 0.30, TOP + 0.18)
REST_R = (0.20, -0.30, TOP + 0.18)
PAD = 0.013  # TCP above fingertip


GRIP_YAW = math.radians(YAW_DEG)  # the gripper turned with the hitch, so it meets the plate and pad as before


def W(x, y, z):
    return tuple(float(v) for v in to_world(x, y, z))


def push_plate(lift=0.0):
    x = -0.020
    top0 = TOPPL + HOVER
    return [Waypoint(1.5, W(x, 0, top0 + PAD + 0.06), FINGER_CLOSED, "above the coupler head", yaw=GRIP_YAW),
            Waypoint(1.0, W(x, 0, top0 + PAD + 0.004), FINGER_CLOSED, "down to the top plate", yaw=GRIP_YAW),
            Waypoint(0.5, W(x, 0, top0 + PAD + 0.001), FINGER_CLOSED, "touch", yaw=GRIP_YAW),
            Waypoint(2.0, W(x, 0, TOPPL + PAD - 0.002), FINGER_CLOSED, "push the coupler down onto the ball", yaw=GRIP_YAW),
            Waypoint(1.0, W(x, 0, TOPPL + PAD - 0.006), FINGER_CLOSED, "press home", yaw=GRIP_YAW),
            Waypoint(0.6, W(x, 0, TOPPL + PAD - 0.006), FINGER_CLOSED, "hold", yaw=GRIP_YAW),
            Waypoint(1.0, W(x, 0, TOPPL + PAD + 0.06), FINGER_CLOSED, "straight up", yaw=GRIP_YAW)]


def push_handle(dz=0.0):
    """End pad path: handle angle 60 -> 0 (end at hinge + R(theta)(-L,0)); dz = how much higher the coupler stands."""
    def end(th):
        t = math.radians(th)
        return (HP[0] - HANDLE_L * math.cos(t) - 0.002, HP[2] + HANDLE_L * math.sin(t) + 0.004 + 0.001 + 0.003 + dz)
    wps = []
    ex, ez = end(60)
    wps.append(Waypoint(1.5, W(ex, 0, ez + PAD + 0.05), FINGER_CLOSED, "above the open handle's end", yaw=GRIP_YAW))
    wps.append(Waypoint(1.0, W(ex, 0, ez + PAD + 0.002), FINGER_CLOSED, "onto the end pad", yaw=GRIP_YAW))
    for th in (45, 30, 15, 0):
        ex, ez = end(th)
        wps.append(Waypoint(0.9, W(ex, 0, ez + PAD - (0.004 if th == 0 else 0.0)), FINGER_CLOSED, f"push handle to {th} deg", yaw=GRIP_YAW))
    wps += [Waypoint(0.6, W(ex, 0, ez + PAD - 0.006), FINGER_CLOSED, "hold", yaw=GRIP_YAW), Waypoint(1.0, W(ex, 0, ez + PAD + 0.05), FINGER_CLOSED, "up", yaw=GRIP_YAW)]
    return wps


def demo():
    v = hhkit.demo_variant()
    rest = [Waypoint(1.0, REST_R, FINGER_OPEN, "stay", yaw=0.0)]
    back = [Waypoint(1.5, REST_L, FINGER_OPEN, "rest", yaw=0.0)]
    start = [Waypoint(1.0, REST_L, FINGER_CLOSED, "start", yaw=0.0)]
    if v == "naive":  # handle first (coupler still hovering), then the coupler
        return {"left": start + push_handle(HOVER) + push_plate() + back, "right": rest}
    if v == "plate":  # only seat it
        return {"left": start + push_plate() + back, "right": rest}
    return {"left": start + push_plate() + push_handle(0.0) + back, "right": rest}
