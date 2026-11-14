"""On the station table:

- a two-wheeled utility trailer (fixed): a blue steel deck (42 x 34 cm, 12 cm off the table, 5 cm rails) on one
  axle with two 18 cm wheels. Its drawbar ends in a painted steel block (80 x 80 mm, 77.5 mm tall) with a towing
  lug on its front face: a 15 mm steel plate projecting 50 mm toward the robot, 40 mm wide, its top flush with the
  block's top (lug centre 7 cm above the table), with a 19 mm round hole 30 mm out from the block;
- a 1/2 in screw-pin D shackle without its pin (galvanised steel, 0.25 kg): 12.7 mm bar, two round ears 30 mm across
  and 12.7 mm thick with 20.6 mm between them, 47.8 mm inside length. The upper ear has a plain 17.5 mm hole; the
  lower ear is threaded 5/8-11 UNC (2.31 mm per turn) for its bottom 5.8 mm, under a 6.9 mm plain bore. It stands upright in a
  pine stand (a 10 mm base with a slot that holds its lower ear and lower bar), threaded ear down, bow toward the
  robot, left of the block;
- its screw pin (steel, red, 0.11 kg): a 15.8 mm pin 46 mm long under a 25 mm collar, with a flat eye 26 mm wide and
  8 mm thick on top. It stands eye up in a hole through a pine block (40 x 40 x 30 mm), right of the block.

The robot must put the shackle on the lug (the lug between its ears), push the pin through the upper ear, the lug
and into the lower ear, and screw it fully home: two and a half clockwise turns (seen from above) from the thread's first catch
seat its collar on the upper ear.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from jointkit import (Attach, PartState, on_physics, on_reset, part, rot_z, set_attach_target,  # noqa: E402
                      set_attached, set_collision)
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Shackle the trailer's towing lug: the lug between the shackle's ears and the pin screwed fully home."

GALV, RED = (0.66, 0.68, 0.70), (0.78, 0.10, 0.08)
PAINT, PINE = (0.16, 0.30, 0.52), (0.80, 0.64, 0.42)
SIDES = 16


def ring(a_in, a_out, z0, z1, color, sides=SIDES):
    """A 16-sided ring (or, with a_in = 0, a disc) between apothems ``a_in`` and ``a_out`` (m), from z0 to z1."""
    side = 2 * a_out * math.tan(math.pi / sides) * 1.02
    return [(((a_in + a_out) / 2 * math.cos(t), (a_in + a_out) / 2 * math.sin(t), (z0 + z1) / 2),
             (a_out - a_in, side, z1 - z0), color, math.degrees(t))
            for t in (2 * math.pi * k / sides for k in range(sides))]


def fixed(name, lo, hi, color):
    """A fixed box between world corners ``lo`` and ``hi``."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return Box(name=name, pos=tuple(float(v) for v in (lo + hi) / 2), size=tuple(float(v) for v in hi - lo),
               color=color)


# -- the block and its lug (fixed) ------------------------------------------------------------------------------------
XH, YH = 0.29, -0.08                 # the lug hole's axis
LUG_Z = TOP + 0.07                   # the lug's mid-plane
LUG_T, HOLE, LUG_W = 0.015, 0.019, 0.040
FACE = XH + 0.030                    # the block's front face; the lug's tip is 50 mm in front of it
TIP = FACE - 0.050
LUG_TOP, LUG_BOT = LUG_Z + LUG_T / 2, LUG_Z - LUG_T / 2
h2, w2 = HOLE / 2, LUG_W / 2
BLOCK = ([fixed("block", (FACE, YH - 0.04, TOP), (FACE + 0.08, YH + 0.04, LUG_TOP), PAINT),
          fixed("lug_front", (TIP, YH - w2, LUG_BOT), (XH - w2 + 0.001, YH + w2, LUG_TOP), PAINT),
          fixed("lug_back", (XH + w2 - 0.001, YH - w2, LUG_BOT), (FACE, YH + w2, LUG_TOP), PAINT)]
         # the round 19 mm hole: a 16-sided ring out to the plate's edges, and the four corners around it
         + [Box(name=f"lug_ring_{k}", pos=(XH + p[0], YH + p[1], LUG_Z), size=sz, color=PAINT, yaw=math.radians(yaw))
            for k, (p, sz, _c, yaw) in enumerate(ring(h2, w2, -LUG_T / 2, LUG_T / 2, PAINT))]
         + [fixed(f"lug_corner_{k}", (XH + sx * 0.010 - (sx < 0) * 0.010, YH + sy * 0.010 - (sy < 0) * 0.010, LUG_BOT),
                  (XH + sx * 0.010 + (sx > 0) * 0.010, YH + sy * 0.010 + (sy > 0) * 0.010, LUG_TOP), PAINT)
            for k, (sx, sy) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1)))])

# -- the trailer behind the drawbar (fixed): a deck with side rails on one axle with two wheels ------------------------
TIRE, HUB = (0.08, 0.08, 0.09), (0.70, 0.70, 0.72)
DX0, DX1 = FACE + 0.08, FACE + 0.50          # deck from 8 to 50 cm behind the block's front face (42 cm long)
DW = 0.17                                     # deck half-width (34 cm wide)
AX, WR, WW = (DX0 + DX1) / 2, 0.09, 0.04      # axle x, wheel radius (18 cm wheels), wheel width
DZ0, DZ1, RZ = TOP + 0.10, TOP + 0.12, TOP + 0.17  # deck underside, deck top, rail top
DECK = [fixed("riser", (FACE + 0.04, YH - 0.03, LUG_TOP), (DX0 + 0.02, YH + 0.03, DZ0), PAINT),
        fixed("deck", (DX0, YH - DW, DZ0), (DX1, YH + DW, DZ1), PAINT),
        fixed("rail_l", (DX0, YH + DW - 0.012, DZ1), (DX1, YH + DW, RZ), PAINT),
        fixed("rail_r", (DX0, YH - DW, DZ1), (DX1, YH - DW + 0.012, RZ), PAINT),
        fixed("rail_b", (DX1 - 0.012, YH - DW, DZ1), (DX1, YH + DW, RZ), PAINT),
        fixed("axle", (AX - 0.012, YH - DW - WW, TOP + WR - 0.01), (AX + 0.012, YH + DW + WW, TOP + WR + 0.01), HUB)]


def wheel(name, yc):
    """A wheel (axis along y) drawn as horizontal slabs whose lengths follow a circle, plus a hub."""
    out, n = [], 12
    for k in range(n):
        z0, z1 = 2 * WR * k / n, 2 * WR * (k + 1) / n
        zm = (z0 + z1) / 2 - WR
        half = math.sqrt(max(WR * WR - zm * zm, 0.0))
        out.append(fixed(f"{name}_{k}", (AX - half, yc - WW / 2, TOP + z0), (AX + half, yc + WW / 2, TOP + z1), TIRE))
    out.append(fixed(f"{name}_hub", (AX - 0.035, yc - WW / 2 - 0.002, TOP + WR - 0.035),
                     (AX + 0.035, yc + WW / 2 + 0.002, TOP + WR + 0.035), HUB))
    return out


TRAILER = DECK + wheel("wheel_l", YH + DW + WW / 2 + 0.002) + wheel("wheel_r", YH - DW - WW / 2 - 0.002)

# -- the shackle (origin: on the pin axis midway between the ears; pin axis z, bow toward -x) -------------------------
PITCH, TURNS = 0.0254 / 11, 2.5     # 5/8-11 UNC thread; two and a half turns from the first catch to seated
TRAVEL = PITCH * TURNS               # 5.8 mm (shortened from the real pin's full-ear thread to fit the 60 s run)
GAP, EAR_T, EAR_R, BAR = 0.0206, 0.0127, 0.0151, 0.0127   # Crosby G-209 1/2 in: A 20.6, F 30.2, D 12.7 mm
EAR_IN = GAP / 2                     # 10.3 mm: the ears' inner faces
EAR_OUT = EAR_IN + EAR_T             # 23.0 mm: their outer faces
HOLE_A, MINOR_A = 0.00875, 0.00675   # plain hole (17.5 mm) and thread minor diameter (13.5 mm), as apothems
LEAD = EAR_T - TRAVEL                # 6.9 mm plain bore above the threaded part of the lower ear
STEP = -EAR_IN - LEAD                # where the pin's end rests before the thread catches
LEG_Z = EAR_IN + BAR / 2             # 16.65 mm: the bars' centre lines
APEX_X = -(0.0478 + BAR / 2)         # 47.8 mm inside length
SHACKLE_BOXES = (
    ring(HOLE_A, EAR_R, EAR_IN, EAR_OUT, GALV)                                     # upper ear: plain hole
    + ring(HOLE_A, EAR_R, -EAR_IN - LEAD, -EAR_IN, GALV)                           # lower ear: lead-in
    + ring(MINOR_A, EAR_R, -EAR_OUT, -EAR_IN - LEAD, GALV)                         # lower ear: threaded
    + [((( -EAR_R + 0.0005 + APEX_X) / 2, 0.0, s * LEG_Z), (-(EAR_R - 0.0005) - APEX_X, BAR, BAR), GALV) for s in (1, -1)]
    + [((APEX_X, 0.0, 0.0), (BAR, BAR, 2 * EAR_OUT), GALV)])

# -- the stand: the shackle stands in it, lower ear and lower bar in a slot ---------------------------------------------
XS, YS = 0.27, 0.07                  # where the shackle's pin axis stands
STAND_FLOOR = TOP + 0.010
S0 = np.array([XS, YS, STAND_FLOOR + EAR_OUT])  # the shackle's origin in the stand
WALL = STAND_FLOOR + 0.012
P_, E_ = 0.016, 0.021                # the pocket around the lower ear (30.2 mm across) and its outer wall
STAND = [fixed("stand_base", (XS - 0.0665, YS - 0.025, TOP), (XS + E_, YS + 0.025, STAND_FLOOR), PINE),
         fixed("stand_slot_l", (XS - 0.0665, YS + 0.0075, STAND_FLOOR), (XS - P_, YS + 0.025, WALL), PINE),
         fixed("stand_slot_r", (XS - 0.0665, YS - 0.025, STAND_FLOOR), (XS - P_, YS - 0.0075, WALL), PINE),
         fixed("stand_slot_end", (XS - 0.0665, YS - 0.0075, STAND_FLOOR), (XS - 0.0615, YS + 0.0075, WALL), PINE),
         fixed("stand_ear_l", (XS - P_, YS + P_, STAND_FLOOR), (XS + E_, YS + 0.025, WALL), PINE),
         fixed("stand_ear_r", (XS - P_, YS - 0.025, STAND_FLOOR), (XS + E_, YS - P_, WALL), PINE),
         fixed("stand_ear_end", (XS + P_, YS - P_, STAND_FLOOR), (XS + E_, YS + P_, WALL), PINE)]
SHACKLE = part("shackle", pos=tuple(float(v) for v in S0 + [0, 0, 0.0005]), mass=0.25, color=GALV, friction=0.6,
               boxes=SHACKLE_BOXES)

# -- the pin (origin: on its axis at the collar's underside; shank down, eye up) ---------------------------------------
PIN_A, SHANK = 0.0079, 0.046         # 15.8 mm pin; 46 mm under the collar: flush with the lower ear when seated
EYE_T, EYE_W = 0.008, 0.026
Z_SEAT = EAR_OUT                     # the pin's origin, seated: the collar on the upper ear
K_ROT, K_LIN = 6.0, 20000.0
SCREW = Attach("shackle", pos0=(0.0, 0.0, Z_SEAT), stiffness=K_ROT, damping=0.02, travel=TRAVEL + 0.002,
               lin_stiffness=K_LIN, lin_damping=100.0, lin_max_force=40.0, enabled=False)
HX, HY = 0.22, -0.23                 # the pin holder's hole
PIN_YAW = -math.pi / 4               # the eye's flat faces face (1, 1) / sqrt 2
CHAMFER = 0.0015                     # the pin's end: a 1.5 mm x 45 degree chamfer


def cone_tip(a, c, z0, color, sides=SIDES, plate=0.002):
    """A 45-degree chamfer at the bottom of a 16-sided pin of apothem ``a``: a core disc and 16 plates pitched 45 degrees
    whose outer faces run from apothem a - c at z0 to a at z0 + c."""
    side = 2 * a * math.tan(math.pi / sides) * 1.02
    k = plate / (2 * math.sqrt(2))
    rc, zc = a - c / 2 - k, z0 + c / 2 + k
    return ring(0, a - c, z0, z0 + c, color) + [
        ((rc * math.cos(t), rc * math.sin(t), zc), (plate, side, c * math.sqrt(2)), color, math.degrees(t), 45.0)
        for t in (2 * math.pi * k2 / sides for k2 in range(sides))]


THREAD_BOXES = cone_tip(PIN_A, CHAMFER, -SHANK, RED) + ring(0, PIN_A, -SHANK + CHAMFER, -SHANK + TRAVEL, RED)
PIN = part("pin", pos=(HX, HY, TOP + SHANK + 0.0005), mass=0.11, color=RED, friction=0.6, yaw=PIN_YAW,
           attach=SCREW, boxes=(
               THREAD_BOXES                                                        # the threaded end
               + ring(0, PIN_A, -SHANK + TRAVEL, 0.0, RED)                         # the plain shank
               + ring(0, 0.0125, 0.0, 0.006, RED)                                  # the collar
               + [((0, 0, 0.0105), (EYE_W, EYE_T, 0.009), RED), ((0, 0, 0.0275), (EYE_W, EYE_T, 0.005), RED)]
               + [((s * 0.009, 0, 0.020), (0.008, EYE_T, 0.010), RED) for s in (1, -1)]))
THREADED = range(len(THREAD_BOXES))  # indices of the threaded end's boxes in PIN.boxes
HOLDER = [fixed("holder_l", (HX - 0.020, HY + 0.009, TOP), (HX + 0.020, HY + 0.020, TOP + 0.030), PINE),
          fixed("holder_r", (HX - 0.020, HY - 0.020, TOP), (HX + 0.020, HY - 0.009, TOP + 0.030), PINE),
          fixed("holder_f", (HX - 0.020, HY - 0.009, TOP), (HX - 0.009, HY + 0.009, TOP + 0.030), PINE),
          fixed("holder_b", (HX + 0.009, HY - 0.009, TOP), (HX + 0.020, HY + 0.009, TOP + 0.030), PINE)]

OBJECTS = BLOCK + TRAILER + STAND + HOLDER + [SHACKLE, PIN]

# -- the thread (pure Python; the physics hook below feeds it) ------------------------------------------------------
# n: the pin's net tightening turn (rad, clockwise seen from above), 0 at the first catch, 2 pi TURNS seated.
# h: the pin's origin above its seat (m). The thread wants h = PITCH * (TURNS - n / 2 pi).
FRICTION = 0.05                 # N m: running torque of the thread (hand-turned, free running)
SLACK = math.radians(30)        # give: the pin can be turned 30 degrees past where its height says it should be
SEATED = 0.001                  # m: a collar within 1 mm of the ear is seated
RELEASE = math.radians(10)      # unscrewed 10 degrees past the first catch: the pin is off the thread
SETTLE = 60                     # physics steps (0.25 s) after a catch in which the pin settles on the thread


def thread_height(n: float) -> float:
    return PITCH * (TURNS - min(max(n, -RELEASE), 2 * math.pi * TURNS) / (2 * math.pi))


class Thread:
    def __init__(self):
        self.reset(0.0)

    def reset(self, n: float) -> None:
        self.anchor = n
        self.settle = SETTLE

    def step(self, n: float, h: float) -> tuple[float, float]:
        pull = K_ROT * (self.anchor - n)  # breakaway friction: a stiff spring to an anchor that slips
        if abs(pull) > FRICTION:
            self.anchor = n + math.copysign(FRICTION, pull) / K_ROT
        n_max = 2 * math.pi * (TURNS - h / PITCH) + SLACK  # it cannot turn further on than its height allows
        self.anchor = min(self.anchor, n_max)
        if self.settle > 0:
            self.settle -= 1
        return self.anchor, thread_height(n)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _twist(shackle: PartState, pin: PartState) -> float:
    r = shackle.rotation().T @ pin.rotation()
    return math.atan2(r[1, 0], r[0, 0])


def _relative(shackle: PartState, pin: PartState):
    """The pin's origin in the shackle frame and its tilt from the shackle's pin axis (rad)."""
    p = shackle.to_local(pin.pos)
    tilt = math.acos(max(-1.0, min(1.0, float((shackle.rotation().T @ pin.rotation())[2, 2]))))
    return p, tilt


def can_catch(shackle: PartState, pin: PartState) -> bool:
    """The pin's end rests on the threaded ear's lead-in: on the shackle's axis within 1.5 mm, tilted under 6 degrees,
    within 1.5 mm of the thread's start, nearly still."""
    p, tilt = _relative(shackle, pin)
    return bool(math.hypot(p[0], p[1]) < 0.003 and tilt < math.radians(8) and abs(p[2] - Z_SEAT - TRAVEL) < 0.003
                and np.linalg.norm(pin.lin_vel - shackle.lin_vel) < 0.05)


THREAD = Thread()
_STATE: dict = {}
_DEBUG = [0] if os.environ.get("SHACKLE_DEBUG") else []


def _reset_state(environment=None) -> None:
    THREAD.reset(0.0)
    _STATE.update(engaged=False, n=0.0, twist=None, dropped=False, catches=0)
    if environment is not None:
        set_attached(environment, "pin", False)
        set_collision(environment, "pin", THREADED, True)


_reset_state()
on_reset(_reset_state)

if _DEBUG:
    from jointkit import _np, on_step, quat_to_matrix

    _TICK = [0]

    @on_step
    def _watch_hands(environment) -> None:
        """SHACKLE_DEBUG: each arm's TCP, and the shackle / pin relative to it (world axes, mm)."""
        from yam_arena.scene import object_states

        _TICK[0] += 1
        if _TICK[0] % 4:
            return
        base = environment.unwrapped
        s = object_states(environment, ["shackle", "pin"])
        out = []
        for arm, obj in (("left", "shackle"), ("right", "pin")):
            robot = base.scene[f"{arm}_robot"]
            i = robot.find_bodies("gripper")[0][0]
            pose = _np(robot.data.body_link_pose_w)[0, i]
            r = quat_to_matrix(pose[3:7])
            tcp = pose[:3] - _np(base.scene.env_origins)[0] + r @ np.array([0.0, 0.0, 0.134])
            out.append(f"{arm} tcp {np.round(tcp * 1000, 1)} {obj}-tcp {np.round((s[obj].pos - tcp) * 1000, 1)}")
        z = s["shackle"].rotation()[:, 2]
        print(f"[hands] step {_TICK[0]:5d} t {_TICK[0] / 20:6.2f}  " + "  ".join(out) + f"  shackle z {np.round(z, 3)}",
              flush=True)


@on_physics
def _thread(environment) -> None:
    """Every physics step: catch the thread, count the pin's turn, set the screw's drive targets, release it."""
    from yam_arena.scene import object_states

    s = object_states(environment, ["shackle", "pin"])
    shackle, pin = s["shackle"], s["pin"]
    twist = _twist(shackle, pin)
    if _DEBUG:
        _DEBUG[0] += 1
        if _DEBUG[0] % 48 == 0:
            p, tilt = _relative(shackle, pin)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(shackle.rotation()[2, 2])))))
            print(f"[shackle] t {_DEBUG[0] / 240:6.2f} s  shackle {np.round(shackle.pos * 1000, 1)} tilt {ang:4.1f}  "
                  f"pin {np.round(pin.pos * 1000, 1)}  pin in shackle {np.round(p * 1000, 1)} tilt "
                  f"{math.degrees(tilt):4.1f}  engaged {_STATE['engaged']} n {math.degrees(_STATE['n']):7.1f}", flush=True)
    if not _STATE["engaged"]:
        if can_catch(shackle, pin):
            THREAD.reset(0.0)
            _STATE.update(engaged=True, n=0.0, twist=twist, catches=_STATE["catches"] + 1)
            set_attach_target(environment, "pin", twist, TRAVEL)
            set_attached(environment, "pin", True)
            set_collision(environment, "pin", THREADED, False)
            if _DEBUG:
                print(f"[shackle] thread caught at {_DEBUG[0] / 240:.2f} s", flush=True)
        return
    if _STATE["twist"] is not None:
        _STATE["n"] -= _wrap(twist - _STATE["twist"])  # clockwise tightens
    _STATE["twist"] = twist
    n = _STATE["n"]
    h = float(shackle.to_local(pin.pos)[2] - Z_SEAT)
    target, height = THREAD.step(n, h)
    set_attach_target(environment, "pin", _wrap(twist - (target - n)), height)
    if n < -RELEASE:
        set_attached(environment, "pin", False)  # unscrewed off the thread
        set_collision(environment, "pin", THREADED, True)
        _STATE["engaged"] = False


# -- scoring --------------------------------------------------------------------------------------------------------
OFF_TABLE = TOP - 0.03


def _pin_axis_at(pin: PartState, z: float) -> np.ndarray:
    """Where the pin's axis crosses the height z (world)."""
    r = pin.rotation()
    d = r[:, 2]
    t = (z - pin.pos[2]) / d[2] if abs(d[2]) > 1e-6 else 0.0
    return pin.pos + t * d


def through_lug(pin: PartState) -> bool:
    """The pin passes through the lug's hole: its axis inside the hole at the lug's mid-plane, tilted under 15 degrees,
    its end below the lug and its collar above it."""
    if pin.tilt() > math.radians(15):
        return False
    c = _pin_axis_at(pin, LUG_Z)
    tip = pin.to_world((0.0, 0.0, -SHANK))
    return bool(abs(c[0] - XH) < HOLE / 2 and abs(c[1] - YH) < HOLE / 2 and tip[2] < LUG_BOT and pin.pos[2] > LUG_TOP)


def through_ears(shackle: PartState, pin: PartState) -> bool:
    """The pin is on the shackle's axis (within 3 mm, tilted under 10 degrees), its collar on or above the upper ear and
    its end into the threaded ear."""
    p, tilt = _relative(shackle, pin)
    return bool(math.hypot(p[0], p[1]) < 0.003 and tilt < math.radians(10) and p[2] > Z_SEAT - 0.001
                and p[2] - SHANK < -EAR_IN)


def _update(objects) -> float:
    shackle, pin = objects["shackle"], objects["pin"]
    _STATE["dropped"] |= bool(shackle.low[2] < OFF_TABLE or pin.low[2] < OFF_TABLE)
    if _STATE["dropped"] or not (through_lug(pin) and through_ears(shackle, pin)):
        return 0.0
    if _STATE["engaged"]:
        h = float(shackle.to_local(pin.pos)[2] - Z_SEAT)
        if h <= SEATED:
            return 1.0
        if _STATE["n"] >= 2 * math.pi:
            return 0.5
    return 0.25


def success(objects) -> bool:
    """Pin through the upper ear, the lug and the threaded ear, its collar seated (within 1 mm); held 1 s."""
    return _update(objects) == 1.0


def score(objects) -> float:
    """0 if the shackle or the pin fell off the table, or the pin is not through the lug and both ears. Otherwise 0.25
    pin through all three, not threaded; 0.5 threaded at least one turn, not seated; 1.0 collar seated (1 mm or less)."""
    return _update(objects)


# -- scripted demo (pure NumPy plans; demo() wraps them in yam_arena Waypoints) -------------------------------------
OPEN = -0.04695
SHACKLE_PINCH = -(BAR / 2 - 0.005)   # 5 mm into the bar: about 25 N per finger
PIN_PINCH = -(EYE_T / 2 - 0.006)
PIN_GRIP = 0.021                     # TCP above the pin's origin when pinching the eye (pad tips 2 mm above the collar)


def frame(z, x) -> np.ndarray:
    """Gripper rotation with approach axis ``z`` and finger-closing axis ``x`` (world)."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    x = np.asarray(x, float) - z * np.dot(x, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def down(yaw: float) -> np.ndarray:
    """Straight-down grip turned ``yaw`` about the vertical (yam_arena.demo.grip_rotation)."""
    return rot_z(yaw) @ frame((0, 0, -1), (0, -1, 0))


# Left arm: pinches the shackle's upper bar through its thickness, 45 mm out from the pin axis, leaning 35 degrees back.
LEAN = math.radians(35)
A_L = np.array([math.sin(LEAN), 0.0, -math.cos(LEAN)])
HOLD = frame(A_L, (0, -1, 0))
GRASP = np.array([-0.045, 0.0, LEG_Z]) - 0.004 * A_L  # TCP in the shackle frame: the bar 4 mm past it
# Left arm's tracking error with the shackle in hand (measured in simulation; added to its commands at the lug).
LEFT_FIX = np.array([0.0050, -0.0044, 0.0110])


def rot_x(a: float) -> np.ndarray:
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s_], [0.0, s_, c]])


def rot_y(a: float) -> np.ndarray:
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s_], [0.0, 1.0, 0.0], [-s_, 0.0, c]])


# Held this way the shackle hangs tilted in the fingers: its pin axis leans 2.35 degrees toward +x and 0.8 toward -y
# (measured). The hand carries it turned back by that much (about the tool centre point), so the pin axis is vertical.
COMP = rot_x(math.radians(-0.8)) @ rot_y(math.radians(-2.35))
HOLD_C = COMP @ HOLD
AT_LUG = np.array([XH, YH, LUG_Z])    # the shackle's origin with the lug centred between its ears
ENTRY = 0.045                         # it slides onto the lug from this far in front
YAW_PIN = 3 * math.pi / 4             # right grip on the pin's eye; the turns go from here to -pi/4
REST_Z = LUG_TOP - EAR_IN             # the shackle's origin once its upper ear rests on the lug
STROKE_PAD = PIN_GRIP


def _eye_z(h: float, shackle_z: float = REST_Z) -> float:
    """TCP height for pinching the pin's eye with the pin ``h`` above its seat."""
    return shackle_z + Z_SEAT + h + STROKE_PAD


def _strokes(k: int, h0: float, extra: float = 0.0, extra_at: int | None = None):
    """``k`` clockwise turns of 180 degrees, the hand following the thread down; between turns it lets go, rises 1.5 cm
    and turns back by 180 degrees. Turn number ``extra_at`` (default: the last) goes ``extra`` further."""
    extra_at = k - 1 if extra_at is None else extra_at
    at = (XH, YH)
    plan, h, a0 = [], h0, YAW_PIN
    for i in range(k):
        if i:
            a0 = end + math.pi  # the eye lies along the same line after half a turn
        end = a0 - math.pi - (extra if i == extra_at else 0.0)
        h1 = max(h - PITCH / 2, 0.0)
        z0, z1 = _eye_z(h), _eye_z(h1)
        plan += [(1.0, (*at, z0 + 0.015), down(a0), OPEN, "over the eye"), (0.8, (*at, z0), down(a0), OPEN, "down"),
                 (0.8, (*at, z0), down(a0), PIN_PINCH, "pinch"),
                 (1.25, (*at, (z0 + z1) / 2), down((a0 + end) / 2), PIN_PINCH, "turn"),
                 (1.25, (*at, z1), down(end), PIN_PINCH, "turn"), (0.6, (*at, z1), down(end), OPEN, "let go"),
                 (0.6, (*at, z1 + 0.015), down(end), OPEN, "up")]
        h = h1
    return plan


def _left(wait_insert: float):
    pick = S0 + GRASP
    pre = AT_LUG + [-ENTRY, 0, 0] + GRASP + LEFT_FIX
    held = AT_LUG + GRASP + LEFT_FIX
    back = held - 0.05 * A_L
    return [(2.5, pick + [0, 0, 0.04], HOLD, OPEN, "over the shackle"), (1.5, pick, HOLD, OPEN, "down"),
            (0.8, pick, HOLD, SHACKLE_PINCH, "grip the shackle"),
            (1.5, pick + [0, 0, 0.06], HOLD, SHACKLE_PINCH, "lift it out of the stand"),
            (2.5, pre + [0, 0, 0.01], HOLD_C, SHACKLE_PINCH, "carry it to the lug"),
            (1.0, pre, HOLD_C, SHACKLE_PINCH, "line the ears up with the lug"),
            (2.5, held, HOLD_C, SHACKLE_PINCH, "slide the ears over the lug"),
            (wait_insert, held, HOLD_C, SHACKLE_PINCH, "hold the shackle"),
            (0.6, held, HOLD_C, -0.012, "let go"), (1.0, back, HOLD_C, -0.012, "back off"),
            (1.0, back + [0, 0.05, 0.02], HOLD_C, -0.012, "clear of the shackle"),
            (1.5, (0.20, 0.20, TOP + 0.16), HOLD, OPEN, "away")]


def _right_insert():
    pick = np.array([HX, HY, TOP + SHANK + PIN_GRIP])
    up = np.array([HX, HY, TOP + 0.18])
    g = down(YAW_PIN)
    return [(2.5, pick + [0, 0, 0.04], g, OPEN, "over the pin"), (1.5, pick, g, OPEN, "down"),
            (0.8, pick, g, PIN_PINCH, "grip the eye"), (1.5, up, g, PIN_PINCH, "lift the pin out")]


# Right arm's tracking error with the pin in hand over the lug, and where the pin hangs in the fingers (measured).
RIGHT_FIX = np.array([0.0002, 0.0023, 0.0])
PIN_HELD = 0.0227                    # TCP above the pin's origin once the pin is lifted (it slips 1.7 mm on lifting)
OVER = np.array([XH, YH, TOP + 0.18]) + RIGHT_FIX


def _right_over():
    return [(2.5, OVER, down(YAW_PIN), PIN_PINCH, "over the lug")]


def _right_place():
    above = np.array([XH, YH, LUG_Z + EAR_OUT + 0.003 + SHANK + PIN_HELD]) + RIGHT_FIX
    # the chamfer rests on the thread's edge 0.35 mm below the lead-in's floor; pressed 1 mm further
    rest = np.array([XH, YH, LUG_Z + STEP - 0.00035 + SHANK + PIN_HELD - 0.001]) + RIGHT_FIX
    g = down(YAW_PIN)
    return [(1.5, above, g, PIN_PINCH, "pin over the upper ear"),
            (3.0, rest, g, PIN_PINCH, "pin down through the ears and the lug"),
            (0.6, rest, g, -0.012, "let go"), (0.8, rest + [0, 0, 0.016], g, -0.012, "up")]


def plan(route: str = "demo") -> dict[str, list]:
    left_head = _left(0.0)[:7]
    t_in = sum(w[0] for w in left_head)           # the shackle is on the lug
    first = _right_insert() + _right_over()
    wait = max(0.0, t_in + 0.5 - sum(w[0] for w in first))
    right = first + [(wait, first[-1][1], first[-1][2], PIN_PINCH, "wait for the shackle")] + _right_place()
    t_rest = sum(w[0] for w in right)             # the pin rests on the thread; the right hand is up
    left = _left(t_rest - t_in + 0.5)
    t_free = sum(w[0] for w in left[:11])         # the left hand is clear of the shackle
    right += [(max(0.5, t_free - t_rest), right[-1][1], right[-1][2], -0.012, "wait for the left hand")]
    # two and a half turns, in half turns with a regrip between them; the last goes 35 degrees further (the wrist's limit)
    # because the fingers slip a little on the eye, and one spare half turn follows in case it is not seated yet
    n = int(round(2 * TURNS))
    right += _strokes(n + 1, TRAVEL, extra=math.radians(35), extra_at=n - 1)
    right += [(1.0, (XH, YH, TOP + 0.16), right[-1][2], OPEN, "clear")]
    return {"left": left, "right": right}


def demo():
    """Left arm lifts the shackle out of its stand and slides its ears over the lug. Right arm lifts the pin out of its
    holder and lowers it through the upper ear and the lug onto the threaded ear, where the thread catches. Left arm lets
    go. Right arm screws the pin home: clockwise half turns of the eye, regripping between them."""
    from yam_arena.demo import Waypoint

    return {arm: [Waypoint(d, tuple(p), f, label, r) for d, p, r, f, label in wps] for arm, wps in plan().items()}
