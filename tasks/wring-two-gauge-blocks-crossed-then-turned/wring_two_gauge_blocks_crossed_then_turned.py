"""Wring a 25 mm gauge block onto a 50 mm one: cross them at 90 degrees mid-face, then turn them into line.

On a black granite surface plate two rectangular steel gauge blocks (35 x 9 mm section, 50 mm and 25 mm long) stand
on a measuring face. The maker's method for thick blocks: cross the blocks at 90 degrees in the middle of the
measuring faces, rotate them while applying slight force, then align the faces. One hand may steady the tall,
narrow 50 mm block while the other sets the 25 mm block down crossed on its top face; the steadying hand lets go, and
the other keeps the faces in contact under a light press through the 90 degree turn into line, then lets go.

Wringing (the oil film's hold) is not simulated. The scorer credits it from the work that makes it: the 25 mm block
held and turned at least 60 degrees about the face's middle with its face touching the 50 mm block's face (closest
point within 0.05 mm, tilt under 1.5 deg), pressed onto it with at least 3 N on the bench scale under the plate, and no
gripper within 5 mm of the 50 mm block, the plate or the scale, so the scale's load can only come through the faces. Lifting it more than
1 mm off the face loses it. A real trial checks wringing by lifting the top block: a wrung stack lifts as one. Nothing
is moved or latched by code.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import ourkit as k  # noqa: E402
from ourkit import demo as D  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1.0 state already includes 1 s released and still
TOP = TABLE_TOP[SCENE]
INSTRUCTION = ("Wring the 25 mm gauge block onto the top face of the 50 mm block so the two make one stack with their "
               "faces in line, then let go of both.")

# == the scene (x forward from the arm bases' midpoint, y left, z up from the table top; metres) ======================
PLATE_L, PLATE_H, PLATE_X, PLATE_KG = 0.305, 0.076, 0.26, 21.0  # 12 x 12 x 3 in granite plate
BLOCK_W, BLOCK_T = 0.035, 0.009  # rectangular gauge block section (blocks over 10 mm)
BASE_L, TOP_L = 0.050, 0.025  # nominal lengths (the measuring faces are the 35 x 9 ends)
STEEL_DENSITY = 7850.0  # kg/m^3
BASE_KG = round(STEEL_DENSITY * BLOCK_W * BLOCK_T * BASE_L, 3)  # 0.124
TOP_KG = round(STEEL_DENSITY * BLOCK_W * BLOCK_T * TOP_L, 3)  # 0.062
BASE_X, BASE_Y = 0.25, -0.02  # base block: 35 mm side along x
TOP_X, TOP_Y = 0.25, -0.12  # top block: 35 mm side along y (it starts crossed to the base block)
SCALE_T = 0.012  # the bench scale's platform under the plate
SURF = TOP + SCALE_T + PLATE_H  # the plate's top
SPAWN = 0.0003

GRANITE, STEEL = (0.10, 0.10, 0.11), (0.72, 0.74, 0.77)

SCALE_K, SCALE_DAMP = 20000.0, 600.0  # N/m, N s/m: the bench scale's platform (0.25 mm at 5 N); the plate's weight tared
SCALE_KG = 4.0
PLATEN = k.slider("platen", (0.0, 0.0, SCALE_T), "Z", -0.003, 0.002,
                  [k.box((0.0, 0.0, PLATE_H / 2), (PLATE_L, PLATE_L, PLATE_H), GRANITE, label="granite_plate")],
                  rate=SCALE_K, target=0.0, damping=SCALE_DAMP, mass=PLATE_KG, weightless=True)
PLATE = k.mech("plate", (PLATE_X, 0.0, TOP + SPAWN),
               [k.box((0.0, 0.0, SCALE_T / 2), (PLATE_L + 0.010, PLATE_L + 0.010, SCALE_T), (0.25, 0.25, 0.27),
                      label="scale_platform")],
               SCALE_KG, joints=[PLATEN], color=GRANITE, surface=0.5)


def _block(name: str, length: float, xy, yaw: float, mass: float) -> k.Mech:
    return k.mech(name, (xy[0], xy[1], SURF + SPAWN), [k.box((0.0, 0.0, length / 2), (BLOCK_W, BLOCK_T, length), STEEL,
                                                             label=f"{name}_block")],
                  mass, yaw=yaw, color=STEEL, surface=0.3, contact_offset=0.001)


BASE = _block("base", BASE_L, (BASE_X, BASE_Y), 0.0, BASE_KG)
TOPB = _block("top", TOP_L, (TOP_X, TOP_Y), math.pi / 2, TOP_KG)
OBJECTS = [PLATE, BASE, TOPB]

# == scoring ===========================================================================================================
GAP_TOL = 0.00005  # m: the faces in contact: no gap over 0.05 mm where they overlap (numerical contact)
PRESS_MIN = 3.0  # N: the maker's 'slight force', pressing the faces together, read on the bench scale
FLAT_TOL = math.radians(1.5)  # the faces flat on each other: tilt under 1.5 deg
TURN_MIN = math.radians(60.0)  # turned at least 60 deg in that contact while held
FREE_MM = 5.0  # mm: no gripper this near the 50 mm block, the plate or the scale while the turn counts (a push on
#               any of them would reach the scale; only the 25 mm block's press through the faces may)
_g = np.linspace(-PLATE_L / 2, PLATE_L / 2, 16)  # every 2 cm
ON_SCALE_PTS = np.array([(PLATE_X + a, b, SURF) for a in _g for b in _g]  # the plate's top face
                        + [(PLATE_X + a, s * (PLATE_L / 2 + 0.005), z) for a in _g for s in (-1, 1)  # its sides and
                           for z in (TOP + SCALE_T / 2, SURF - PLATE_H / 2, SURF)]  # the platform's edge
                        + [(PLATE_X + s * (PLATE_L / 2 + 0.005), a, z) for a in _g for s in (-1, 1)
                           for z in (TOP + SCALE_T / 2, SURF - PLATE_H / 2, SURF)])
MID_TOL = 0.006  # m: about a point within 6 mm of the face's middle
HELD_MM = 3.0  # held: one gripper's pads within 3 mm of the 25 mm block
HELD_GAP = BLOCK_W + 0.002  # and that gripper's pads closed to under 37 mm apart (closed on its 35 mm ends)
SEP = 0.001  # m: lifted more than 1 mm off the face loses wringing; on the face means within 1 mm of it
ON_TOL = 0.020  # m: standing on the face: centres within 20 mm
ON_TILT = math.radians(3.0)
LINE_TOL = 0.003  # m: in line: centres within 3 mm across both axes
YAW_TOL = math.radians(4.0)  # and long sides within 4 deg
CLEAR_MM = 30.0  # released: no gripper within 30 mm of either block
STILL_S, STILL_TOL = 1.0, 0.001  # still: neither block moves more than 1 mm over the last 1 s
FALL_DEG = 30.0  # a block tilted more than 30 deg has fallen over
OFF_DROP = 0.005  # a block more than 5 mm below the plate's top, or off its edge, has dropped off; the 50 mm block
#                  lifted more than 5 mm off the plate scores 0 too (it is to stay on the plate)

BASE_BOXES, TOP_BOXES = BASE.boxes, TOPB.boxes


def relation(states) -> dict:
    """The 25 mm block's face against the 50 mm block's top face, in the 50 mm block's frame."""
    b, t = states["base"], states["top"]
    rb, rt = b.rotation(), t.rotation()
    face = b.to_world((0.0, 0.0, BASE_L))
    foot = t.to_world((0.0, 0.0, 0.0))
    d = rb.T @ (foot - face)
    tilt = math.acos(float(np.clip(rb[:, 2] @ rt[:, 2], -1.0, 1.0)))
    xt = rb.T @ rt[:, 0]
    yaw = math.atan2(xt[1], xt[0])  # the top block's long side against the base block's, rad
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2  # a block is the same turned 180 deg: (-90, 90]
    # contact: the smallest gap at points of the 25 mm block's lower face that lie over the 50 mm block's top face
    gaps = []
    for gx in np.linspace(-BLOCK_W / 2, BLOCK_W / 2, 8):
        for gy in np.linspace(-BLOCK_T / 2, BLOCK_T / 2, 4):
            q = rb.T @ (t.to_world((gx, gy, 0.0)) - face)
            if abs(q[0]) <= BLOCK_W / 2 and abs(q[1]) <= BLOCK_T / 2:
                gaps.append(float(q[2]))
    contact = min(gaps) if gaps else float(d[2])
    return {"gap": float(d[2]), "dx": float(d[0]), "dy": float(d[1]), "off": float(math.hypot(d[0], d[1])),
            "tilt": tilt, "yaw": yaw, "contact": contact}


def _wrap(a: float) -> float:
    return (a + math.pi / 2) % math.pi - math.pi / 2


FELL, END = k.Sticky(), k.Settle(STILL_S, STILL_TOL)
RUN = {"turn": 0.0, "prev": None, "wrung": False, "held": False, "base_free": True, "q0": None, "qs": [], "force": 0.0,
       "t": 0.0}
SETTLE_S = 1.0  # the scale is tared over the first second (blocks standing, hands away)
REC = k.run_record.RunRecord(__file__, "base", "top")


def on_face(r) -> bool:
    return -SEP <= r["gap"] <= SEP and r["off"] <= ON_TOL and r["tilt"] <= ON_TILT


def in_line(r) -> bool:
    return abs(r["dx"]) <= LINE_TOL and abs(r["dy"]) <= LINE_TOL and abs(r["yaw"]) <= YAW_TOL


def grade(states) -> float:
    if FELL:
        return 0.0
    r = relation(states)
    if RUN["wrung"] and in_line(r):
        return 1.0 if END.ok else 0.75
    if RUN["wrung"]:
        return 0.5
    if on_face(r):
        return 0.25
    return 0.0


def score(states) -> float:
    return grade(states)


def success(states) -> bool:
    return grade(states) >= 1.0


def distance(states) -> float:
    r = relation(states)
    return abs(r["gap"]) + r["off"] + abs(r["yaw"]) * 0.01


@k.on_reset
def _reset(states) -> None:
    FELL.reset(), END.reset()
    RUN.update(turn=0.0, prev=None, wrung=False, held=False, base_free=True, q0=None, qs=[], force=0.0, t=0.0)
    REC.reset()


def scale_force(states) -> float:
    """N pressed down onto the plate beyond what stood on it at the tare: the bench scale's reading."""
    if RUN["q0"] is None:
        return 0.0
    return SCALE_K * (RUN["q0"] - float(states["plate"].q["platen"]))


def _watch_wring(r, held: bool) -> None:
    if r["gap"] > SEP or r["off"] > ON_TOL:  # off the face: the air comes back
        RUN["turn"], RUN["wrung"] = 0.0, False
    elif not (held and RUN["base_free"] and r["contact"] <= GAP_TOL and r["tilt"] <= FLAT_TOL and r["off"] <= MID_TOL
              and RUN["force"] >= PRESS_MIN):
        RUN["turn"] = 0.0  # the turn must be one continuous gripped, flat, centred contact
    elif RUN["prev"] is not None:
        RUN["turn"] += _wrap(r["yaw"] - RUN["prev"])  # net, signed: a back-and-forth wiggle adds up to nothing
        if abs(RUN["turn"]) >= TURN_MIN - 1e-9:
            RUN["wrung"] = True
    RUN["prev"] = r["yaw"]


def _gripped(pts) -> bool:
    """One gripper closed on the block: both its pads within HELD_MM of the block and under HELD_GAP apart."""
    e = k.scene.env()
    if e is None:
        return False
    from yam_arena import station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    for arm in ("left", "right"):
        robot = e.unwrapped.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        joints = k.scene._np(robot.data.joint_pos)[0]
        finger = float(np.mean([joints[names.index(j)] for j in FINGER_JOINTS]))
        hull = k.score.gripper_hulls(joints[[names.index(j) for j in ARM_JOINTS]], finger, station.BASES[arm])
        if D.P.gap_for_finger(finger) > HELD_GAP:
            continue
        pads = [k.score.clearance(pts, [[(n, h) for n, h in hull if n == tip]]) for tip in ("tip_left", "tip_right")]
        if max(pads) <= HELD_MM / 1000:  # both pads on the block: it is between them
            return True
    return False


@k.on_step
def _watch(states) -> None:
    b, t = states["base"], states["top"]
    RUN["t"] += 1.0 / 20.0
    q = float(states["plate"].q["platen"])
    if RUN["t"] <= SETTLE_S:
        RUN["qs"].append(q)
        RUN["q0"] = float(np.mean(RUN["qs"][len(RUN["qs"]) // 2:]))
    RUN["force"] = scale_force(states)
    for s in (b, t):
        c = s.to_world((0.0, 0.0, 0.0))
        off = abs(c[0] - PLATE_X) > PLATE_L / 2 or abs(c[1]) > PLATE_L / 2 or c[2] < SURF - OFF_DROP
        if s is b and c[2] > SURF + OFF_DROP:
            FELL.trip("the 50 mm block was lifted off the plate")
        if s.tilt_deg() > FALL_DEG or off:
            FELL.trip(f"the {s.name if hasattr(s, 'name') else 'block'} fell or dropped")
    grips = k.score.grippers()
    pts_t = k.score.surface_points(t, "", TOP_BOXES, 0.01)
    pts_b = k.score.surface_points(b, "", BASE_BOXES, 0.01)
    held = _gripped(pts_t)
    RUN["held"] = held
    on_scale = np.concatenate([pts_b, ON_SCALE_PTS])  # everything the scale carries but the 25 mm block
    RUN["base_free"] = not grips or k.score.clearance(on_scale, grips, beyond=FREE_MM / 1000) >= FREE_MM / 1000
    r = relation(states)
    _watch_wring(r, held)
    clear = k.hands_clear(np.concatenate([pts_t, pts_b]), CLEAR_MM, grips)
    END.push(np.concatenate([t.to_world((0, 0, TOP_L)), t.to_world((BLOCK_W / 2, 0, 0)), b.to_world((0, 0, BASE_L)),
                             b.to_world((BLOCK_W / 2, 0, 0))]), clear)
    if __import__("os").environ.get("WRING_DEBUG") and int(round(RUN["t"] * 20)) % 10 == 0:
        print(f"[wring] t {RUN['t']:.1f} dxy {r['dx'] * 1000:+.1f}/{r['dy'] * 1000:+.1f} press {RUN['force']:.2f} N contact {r['contact'] * 1000:+.3f} gap {r['gap'] * 1000:+.3f} mm tilt "
              f"{math.degrees(r['tilt']):.2f} yaw {math.degrees(r['yaw']):+.1f} turn {math.degrees(RUN['turn']):.0f} "
              f"held {held} free {RUN['base_free']} wrung {RUN['wrung']} score {grade(states):g}", file=sys.stderr)
    REC.tick(states, lambda: f"press {RUN['force']:.1f} N gap {r['gap'] * 1000:+.3f} mm off {r['dx'] * 1000:+.1f}/{r['dy'] * 1000:+.1f} mm tilt "
                             f"{math.degrees(r['tilt']):.2f} yaw {math.degrees(r['yaw']):+.1f} turn "
                             f"{math.degrees(RUN['turn']):.0f} held {held} free {RUN['base_free']} wrung {RUN['wrung']} score {grade(states):g}")


# == the demo ==========================================================================================================
# Left hand: fingers in from the robot's left, 30 deg below horizontal, closing along x across the 50 mm block's
# 35 mm ends 15 mm up; it steadies the block while the 25 mm block is set down, opens its fingers 8 mm off the block
# (without moving) before the press, and closes on it again only after the counted turn, for the squaring up. Right hand: straight down across the 25 mm block's 35 mm ends; carried over the 50 mm block crossed, set on
# the face's middle at a light touch, then pressed until the bench scale reads 6 N and turned 90 deg in small steps
# under that press (the target re-reads the base block's face every step), eased off by the scale, squared up, let go.
LEFT_GRIP = D.Grip.deg(-90.0, 65.0)
BASE_HOLD_Z = 0.015  # m up the 50 mm block where the left pads close
TIP_UP = 0.014  # m: the right pad tips this far above the 25 mm block's lower face (11 mm of block between the pads),
#               high enough to keep 10 mm between the two grippers at the end of the turn
GRIP_DZ = TIP_UP + D.P.PAD_TIP  # TCP above the 25 mm block's lower face when held
HOVER = 0.008  # m: the lower face this far over the top face before going down
PRESS = 0.004  # m commanded past contact to start the press
PRESS_N = 6.0  # N the right hand holds on the scale through the turn (the scorer needs 3)
PRESS_GAIN, PRESS_MAX = 0.0008, 0.030  # m per N of error per control step; the most commanded past contact
LIGHT = 0.001  # m past contact while squaring up sideways (a full press there rocks the 50 mm block)
LET_GO_S = 3.0  # s the right hand rests the block on the face while the left hand lets go and backs out
LEFT_OFF = 0.008  # m each left pad stands off the 50 mm block's ends while the turn counts (FREE_MM is 5 mm)
LEFT_RISE = 0.012  # m the open left hand rises so its fingertips stand clear of the plate too
LEFT_PUSH = 0.012  # m the 'leftpress' cheat drives the left hand down past its grip on the 50 mm block (about 5 N)
T_SET = {"t": 0.0, "turned": 0.0}  # when the right hand has set the 25 mm block down (the left hand lets go) and
#                                    when the counted turn is over (the left hand steadies the block again for squaring)
TURN_S, TURN_STEPS = 5.0, 18  # the turn in 18 steps in 5 s
TURN_DEG = 90.0  # the wrist turns into line (92 put joint 4 on its limit in the rise afterwards)
MAX_FIX = 0.008  # m: the in-line correction's largest move
OPEN_GAP = BLOCK_W + 0.020


def _face(st) -> np.ndarray:
    return st["base"].to_world((0.0, 0.0, BASE_L))


def _over_face(dz: float):
    return lambda st: _face(st) + np.array([0.0, 0.0, GRIP_DZ + dz])


def _pressing(p: D.Plan, newtons: float = PRESS_N):
    """Over the face, pushed down until the bench scale reads ``newtons``: the depth past contact is integrated from
    the scale's error every control step (the arm is soft and its stiffness changes with the pose)."""
    def fn(st):
        dz = p.memo.get("dz", PRESS)
        if RUN["q0"] is not None:
            dz = float(np.clip(dz + PRESS_GAIN * (newtons - RUN["force"]), 0.0, PRESS_MAX))
        p.memo["dz"] = dz
        p.memo["press_cmd"] = _face(st) + np.array([0.0, 0.0, GRIP_DZ - dz])
        return p.memo["press_cmd"]
    return fn


FACE0 = np.array([BASE_X, BASE_Y, SURF + BASE_L])


def _right(how: str) -> D.Plan:
    p = D.Plan("right")
    T_SET.update(t=0.0, turned=0.0)
    p.hold(5.0, "wait for the left hand")
    D.grasp(p, lambda st: st["top"].to_world((0.0, 0.0, TOP_L / 2)), BLOCK_W, height=TOP_L, grip=D.Grip.deg(0.0),
            tip_clearance=TIP_UP,
            fallback=(TOP_X, TOP_Y, SURF + TOP_L / 2), lift=0.060)
    carry = D.Grip.deg(90.0) if how in ("slam", "wiggle") else D.Grip.deg(0.0)
    p.move(2.5, _over_face(0.030), fallback=tuple(FACE0 + [0, 0, GRIP_DZ + 0.030]), grip=carry, label="over the face")
    if how == "hover":
        p.move(1.5, _over_face(0.005), fallback=tuple(FACE0 + [0, 0, GRIP_DZ + 0.005]), label="5 mm over the face")
        for i in range(TURN_STEPS):
            p.move(TURN_S / TURN_STEPS, _over_face(0.005), fallback=tuple(FACE0 + [0, 0, GRIP_DZ + 0.005]),
                   grip=D.Grip.deg(90.0 * (i + 1) / TURN_STEPS), track=True, label="turn in the air")
        carry = D.Grip.deg(90.0)
    p.move(1.5, _over_face(HOVER), fallback=tuple(FACE0 + [0, 0, GRIP_DZ + HOVER]), grip=carry, label="just over it")
    D.ease(p, _over_face(-LIGHT), 0.004, fallback=tuple(FACE0 + [0, 0, GRIP_DZ - LIGHT]), track=True,
           label="down onto the face, a light touch")
    T_SET["t"] = p.duration
    p.move(LET_GO_S, _over_face(-LIGHT), fallback=tuple(FACE0 + [0, 0, GRIP_DZ - LIGHT]), track=True,
           label="rest it on the face while the left hand lets go")
    if how in ("leftpress", "platepress"):  # the left hand pushes the 50 mm block (or the plate) down while this hand
        #                                     turns a merely resting block
        for i in range(TURN_STEPS):
            p.move(TURN_S / TURN_STEPS, _over_face(0.0), fallback=tuple(FACE0 + [0, 0, GRIP_DZ]),
                   grip=D.Grip.deg(TURN_DEG * (i + 1) / TURN_STEPS), track=True, label="turn it resting, unpressed")
        p.fingers(OPEN_GAP, 1.5, "let go")
        p.back(0.020, 1.2, "rise clear")
        return p
    newtons = 1.0 if how == "light" else PRESS_N
    p.move(2.0, _pressing(p, newtons), fallback=tuple(FACE0 + [0, 0, GRIP_DZ - PRESS]), track=True,
           label="press until the scale reads the press")
    if how == "wiggle":  # set down in line, then twisted 35 deg each way three times, pressing: no net turn
        for a in (55.0, 125.0, 55.0, 125.0, 55.0, 125.0, 90.0):
            p.move(0.8, _over_face(-PRESS), fallback=tuple(FACE0 + [0, 0, GRIP_DZ - PRESS]), grip=D.Grip.deg(a),
                   track=True, label="wiggle on the face")
    if how in ("demo", "held", "light"):
        for i in range(TURN_STEPS):
            fn = _pressing(p, newtons)
            if i == TURN_STEPS - 1:
                fn = D.remember(p, "turned", fn)
            p.move(TURN_S / TURN_STEPS, fn, fallback=tuple(FACE0 + [0, 0, GRIP_DZ - PRESS]),
                   grip=D.Grip.deg(TURN_DEG * (i + 1) / TURN_STEPS), track=True, label="turn into line, pressing")
        p.hold(0.5, "hold pressed")
        T_SET["turned"] = p.duration
        p.hold(2.5, "hold pressed while the left hand takes the 50 mm block again")
        for n in (4.0, 2.0):  # ease off by the scale, not by position: the pads may sit lower on the block
            p.move(0.6, _pressing(p, n), fallback=tuple(FACE0 + [0, 0, GRIP_DZ - LIGHT]), track=True,
                   label="ease the press off on the scale")
        D.corrected(p, 1.5, want=lambda st: np.array([*_face(st)[:2], st["top"].to_world((0.0, 0.0, 0.0))[2]]),
                    have=lambda st: st["top"].to_world((0.0, 0.0, 0.0)), key="lined", base="press_cmd",
                    max_fix=MAX_FIX, label="square it up sideways, still pressed",
                    fallback=tuple(FACE0 + [0, 0, GRIP_DZ - LIGHT]))
        p.hold(0.5, "hold square")
        for n in (1.0, 0.5):
            p.move(0.6, _pressing(p, n), fallback=tuple(FACE0 + [0, 0, GRIP_DZ - LIGHT]), track=True,
                   label="ease the press off on the scale")
    else:
        p.move(1.0, _over_face(0.0), fallback=tuple(FACE0 + [0, 0, GRIP_DZ]), track=True, label="ease the press off")
    if how == "held":
        p.hold(6.0, "keep holding")
        return p
    p.fingers(OPEN_GAP, 1.5, "let go")
    p.back(0.020, 1.2, "rise clear")  # 40 mm put joint 4 on its limit
    return p


def _left(how: str, until: float, end: float) -> D.Plan:
    """Steady the 50 mm block until the 25 mm block is set down (``until``), then let go and back out; the
    'leftpress' cheat instead drives down on its grip through the turn (to ``end``) and lets go after."""
    p = D.Plan("left")
    hold_at = lambda st: st["base"].to_world((0.0, 0.0, BASE_HOLD_Z))  # noqa: E731
    D.grasp(p, hold_at, BLOCK_W, grip=LEFT_GRIP, fallback=(BASE_X, BASE_Y, SURF + BASE_HOLD_Z), lift=None)
    p.hold(max(0.0, until - p.duration), "steady the block")
    if how == "platepress":  # let go, then push straight down on the plate's top beside the blocks
        p.release(seconds=1.0)
        p.back(0.030, 1.0, "back out")
        p.move(1.5, (PLATE_X - 0.06, 0.08, SURF + 0.030), grip=D.Grip.deg(0.0), finger=D.gap_finger(0.0),
               label="over the plate")
        p.move(1.0, (PLATE_X - 0.06, 0.08, SURF - LEFT_PUSH), grip=D.Grip.deg(0.0), label="push on the plate")
        p.hold(max(0.0, end - p.duration), "keep pushing")
        p.move(1.0, (PLATE_X - 0.06, 0.08, SURF + 0.050), grip=D.Grip.deg(0.0), label="lift off the plate")
        return p
    if how == "leftpress":
        p.move(1.0, lambda st: hold_at(st) - np.array([0.0, 0.0, LEFT_PUSH]),
               fallback=(BASE_X, BASE_Y, SURF + BASE_HOLD_Z - LEFT_PUSH), grip=LEFT_GRIP, label="push the block down")
        p.hold(max(0.0, end - p.duration), "keep pushing")
    if how in ("demo", "held", "light") and T_SET["turned"] > p.duration:
        closed = D.P.gap_for_finger(p.last.finger)
        p.fingers(BLOCK_W + 2 * LEFT_OFF, 1.0, "open the fingers off the block")
        p.move(1.0, lambda st: hold_at(st) + np.array([0.0, 0.0, LEFT_RISE]), grip=LEFT_GRIP,
               fallback=(BASE_X, BASE_Y, SURF + BASE_HOLD_Z + LEFT_RISE), label="rise clear of the plate, fingers open")
        p.hold(max(0.0, T_SET["turned"] - p.duration), "wait out the pressed turn, clear of the block and plate")
        p.move(1.0, hold_at, grip=LEFT_GRIP, fallback=(BASE_X, BASE_Y, SURF + BASE_HOLD_Z), label="back down, open")
        p.fingers(closed, 1.0, "close on the block again")
        p.hold(max(0.0, end - p.duration), "steady the block for squaring up")
    p.release(seconds=1.0)
    p.back(0.050, 1.5, "back out")
    return p


def _hands(how: str) -> dict:
    r = _right(how)
    l = _left(how, T_SET["t"], r.duration)
    l.home()
    if how != "held":
        r.hold(max(0.0, l.duration - r.duration), "wait")
        r.home()
    return D.together(l, r)


def demo():
    return D.choose(demo=lambda: _hands("demo"), slam=lambda: _hands("slam"), hover=lambda: _hands("hover"),
                    held=lambda: _hands("held"), wiggle=lambda: _hands("wiggle"), light=lambda: _hands("light"),
                    leftpress=lambda: _hands("leftpress"), platepress=lambda: _hands("platepress"))


# == what the template tests read ======================================================================================
TEXT_NUMBERS = [("35 mm wide", BLOCK_W), ("9 mm thick", BLOCK_T), ("50 mm nominal", BASE_L), ("25 mm Mitutoyo", TOP_L),
                ("124 g", BASE_KG), ("62 g", TOP_KG), ("x 25 cm, y -2", BASE_X), ("y -2 cm, its", BASE_Y),
                ("y -12 cm, its", TOP_Y), ("30.5 cm square", PLATE_L), ("7.6 cm thick", PLATE_H), ("x 26 cm", PLATE_X),
                ("about 21 kg", PLATE_KG), ("no gap over 0.05 mm", GAP_TOL), ("at least 3 N", PRESS_MIN), ("tilt under 1.5 deg", math.degrees(FLAT_TOL)),
                ("at least 60 deg", math.degrees(TURN_MIN)), ("within 6 mm of the face's middle", MID_TOL),
                ("within 3 mm of the 25 mm block", HELD_MM / 1000), ("within 5 mm of the 50 mm block", FREE_MM / 1000), ("under 37 mm apart", HELD_GAP), ("more than 1 mm off", SEP),
                ("within 3 mm across", LINE_TOL), ("within 4 deg", math.degrees(YAW_TOL)),
                ("centres within 20 mm", ON_TOL), ("within 30 mm", CLEAR_MM / 1000), ("more than 1 mm over", STILL_TOL),
                ("over the last 1 s", STILL_S), ("more than 30 deg", FALL_DEG), ("more than 5 mm off the plate", OFF_DROP)]
CHEATS = {"none": 0.0, "slam": 0.25, "hover": 0.25, "held": 0.75, "wiggle": 0.25, "light": 0.25, "leftpress": 0.25, "platepress": 0.25}  # mode: the most it may score
HOLD = {}  # no joints; the blocks are watched as bodies
