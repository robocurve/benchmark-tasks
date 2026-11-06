"""PVC solvent-cement joint: push the pipe into the tee's socket with a quarter turn and hold it 30 s (Oatey).

On the table:
- a white PVC tee lying on its side at the back centre, its open socket facing up (a black tick on the socket's rim),
  with short grey pipe stubs already cemented into both ends of its run
- a grey pipe (25 mm across, 11 cm long) standing in a white holder at the front right, its lower end already
  primed and cemented (wet), with a dark stripe up its side

Oatey's solvent-cement instructions (printed on the can): push the pipe fully into the fitting using a 1/4 turning
motion until it bottoms; hold pipe and fitting together for 30 seconds to prevent push-out. Assemble while the cement
is wet (here: within 15 s of the start).
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0
DEBUG = False

# The rules (Oatey solvent cement instructions)
WET_S = 15.0  # assemble within this many seconds of the start, while the cement is wet
HOLD_S = 30.0  # hold pipe and fitting together this long
TURN_DEG = 70.0  # a quarter turn (90 degrees), allowing 20
TEE_MOVED_M, TEE_TURNED_DEG = 0.02, 10.0  # the tee held still: it moves less than this

# Tee: run along y, socket up
RUN_A = 0.016  # the run's half-thickness (apothem of its 12-sided section)
TEE_C = (0.27, 0.0, TOP + RUN_A)
SOCKET_IN = 0.0155  # socket bore's apothem (3 mm clear of the pipe)
FLARE = 0.01  # the socket's mouth flares out 1 cm (a 45-degree lead-in)
SOCKET_WALL = 0.004
SOCKET_TOP = TEE_C[2] + RUN_A + 0.025  # 25 mm deep
SEAT_Z = TEE_C[2] + RUN_A  # the pipe bottoms out on the run
STUB_END = 0.125  # the run (with its stubs) reaches +-12.5 cm along y
# Pipe
PIPE_R, PIPE_L = 0.0125, 0.11
HOLDER_XY = (0.17, -0.13)
PIPE_C = (*HOLDER_XY, TOP + 0.002 + PIPE_L / 2)
SEATED_M = 0.003  # pipe bottom within this of the seat: fully in

WHITE, GREY, BLACK, PURPLE = (0.93, 0.93, 0.92), (0.55, 0.57, 0.6), (0.08, 0.08, 0.08), (0.45, 0.2, 0.55)
_T15 = math.tan(math.radians(15.0))


def _slabs(length, y0, rgb):
    """A bar along y with a 12-sided section (apothem RUN_A): six slabs turned 30 degrees apart about y."""
    return [((2 * RUN_A, length, 2 * RUN_A * _T15 + 0.0004), (0.0, y0, 0.0), rgb, (0.0, 30.0 * k, 0.0)) for k in range(6)]


def _ring(apothem, wall, z0, z1, rgb):
    """A 12-sided tube standing up: twelve wall boxes around the z axis, centred on the tee."""
    mid, h = apothem + wall / 2, z1 - z0
    side = 2 * (apothem + wall) * _T15 + 0.0004
    parts = []
    for k in range(12):
        t = math.radians(30.0 * k)
        parts.append(((wall, side, h), (mid * math.cos(t), mid * math.sin(t), (z0 + z1) / 2), rgb, (0.0, 0.0, 30.0 * k)))
    return parts


def _flare(apothem, z0, rgb):
    """A 45-degree flared mouth on top of the socket: twelve slanted plates leaning outward."""
    side = 2 * (apothem + FLARE + 0.003) * _T15 + 0.0004
    mid, slant = apothem + FLARE / 2 + 0.0015, FLARE * math.sqrt(2.0)
    parts = []
    for k in range(12):
        t = math.radians(30.0 * k)
        parts.append(((slant, side, 0.003), (mid * math.cos(t), mid * math.sin(t), z0 + FLARE / 2), rgb, (0.0, -45.0, 30.0 * k)))
    return parts


_HOLD_IN, _HOLD_W, _HOLD_H = 0.016, 0.005, 0.03

OBJECTS = [
    Box("tee", size=(0.02, 0.05, 0.02), pos=TEE_C, color=WHITE, mass=0.08),
    Cylinder("pipe", radius=PIPE_R, height=PIPE_L, pos=PIPE_C, color=GREY, mass=0.05),
    *[Box(f"holder_{k}", size=s, pos=(HOLDER_XY[0] + dx, HOLDER_XY[1] + dy, TOP + _HOLD_H / 2), color=WHITE)
      for k, (s, dx, dy) in enumerate([
          ((2 * (_HOLD_IN + _HOLD_W), _HOLD_W, _HOLD_H), 0.0, _HOLD_IN + _HOLD_W / 2),
          ((2 * (_HOLD_IN + _HOLD_W), _HOLD_W, _HOLD_H), 0.0, -_HOLD_IN - _HOLD_W / 2),
          ((_HOLD_W, 2 * _HOLD_IN, _HOLD_H), _HOLD_IN + _HOLD_W / 2, 0.0),
          ((_HOLD_W, 2 * _HOLD_IN, _HOLD_H), -_HOLD_IN - _HOLD_W / 2, 0.0)])],
]
mech.compound("tee", [
    *_slabs(0.05, 0.0, WHITE),  # the tee's run
    *_slabs(STUB_END - 0.025, (STUB_END + 0.025) / 2, GREY),  # pipe stubs cemented into both ends
    *_slabs(STUB_END - 0.025, -(STUB_END + 0.025) / 2, GREY),
    *_ring(SOCKET_IN, SOCKET_WALL, RUN_A - 0.004, SOCKET_TOP - TEE_C[2], WHITE),  # the socket, facing up
    *_flare(SOCKET_IN, SOCKET_TOP - TEE_C[2], WHITE),  # its flared mouth
    ((0.006, 0.003, 0.002), (SOCKET_IN + FLARE + 0.004, 0.0, SOCKET_TOP - TEE_C[2] + FLARE + 0.001), BLACK),  # rim tick
])
mech.compound("pipe", [((0.003, 0.005, 0.03), (PIPE_R + 0.001, 0.0, 0.02), PURPLE)])  # stripe up its side


# -- measurements ---------------------------------------------------------------------------


def _yaw(o) -> float:
    q = o.quat_xyzw
    return math.degrees(2.0 * math.atan2(q[2], q[3]))


def _off(a, b) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def seated(objects) -> bool:
    """The pipe is in the socket, bottomed out (the tee's reported centre is its bounding box's, so heights are
    taken relative to where the tee started)."""
    p, t = objects["pipe"], objects["tee"]
    dz = 0.0 if _state.get("tee0") is None else float(t.pos[2] - _state["tee0"][0][2])
    return math.hypot(p.pos[0] - t.pos[0], p.pos[1] - t.pos[1]) < 0.006 and p.low[2] < SEAT_Z + SEATED_M + dz


def entering(objects) -> bool:
    """The pipe's end is in the socket's mouth."""
    p, t = objects["pipe"], objects["tee"]
    dz = 0.0 if _state.get("tee0") is None else float(t.pos[2] - _state["tee0"][0][2])
    return math.hypot(p.pos[0] - t.pos[0], p.pos[1] - t.pos[1]) < 0.01 and p.low[2] < SOCKET_TOP + dz


def holding_pipe(p, objects) -> bool:
    o = objects["pipe"]
    return math.hypot(p[0] - o.pos[0], p[1] - o.pos[1]) < 0.02 and o.low[2] < p[2] < o.high[2] + 0.01


def holding_tee(p, objects) -> bool:
    t = objects["tee"]
    return abs(p[0] - t.pos[0]) < 0.025 and 0.03 < abs(p[1] - t.pos[1]) < STUB_END + 0.005 and p[2] < t.pos[2] + 0.03


# -- episode state, joint physics ---------------------------------------------------------------


def _fresh():
    return dict(t=0.0, tee0=None, ref=None, pipe_yaw0=None, seated_at=None, turned=False, held_s=0.0, done=False,
                failed=False, violated=False, why="", n=0)


_state = _fresh()


@mech.on_tick
def _joint(sim):
    """A new episode resets the scorer. While the pipe sits in the wet socket, turning it turns a tee nobody holds."""
    s, st = sim.states, _state
    if sim.t <= 1.5 / CONTROL_HZ:
        st.clear()
        st.update(_fresh())
    st["t"] = sim.t
    if seated(s) and not any(holding_tee(s[f"{a}_tcp"].pos, s) for a in ("left", "right")):
        w = float(s["pipe"].ang_vel[2])
        sim.set_velocity("tee", ang=(0.0, 0.0, w))


def score(objects):
    st = _state
    pipe, tee = objects["pipe"], objects["tee"]
    if st["tee0"] is None:
        st["tee0"] = (tee.pos.copy(), _yaw(tee))
        st["pipe_yaw0"] = _yaw(pipe)
    if st["ref"] is None and entering(objects):
        st["ref"] = (tee.pos.copy(), _yaw(tee))  # the tee must stay still from when the pipe enters its socket
    tcps = [objects[f"{a}_tcp"].pos for a in ("left", "right")]
    ref = st["ref"] or (tee.pos, _yaw(tee))
    moved = float(((tee.pos - ref[0]) ** 2).sum() ** 0.5)
    if st["ref"] is not None and (moved > TEE_MOVED_M or _off(_yaw(tee), ref[1]) > TEE_TURNED_DEG):
        st["violated"], st["why"] = True, "tee moved"
    is_seated = seated(objects)
    if is_seated and st["seated_at"] is None:
        st["seated_at"] = st["t"]
    if is_seated and _off(_yaw(pipe), st["pipe_yaw0"]) >= TURN_DEG:
        st["turned"] = True
    if st["seated_at"] is not None and not st["done"] and not st["failed"]:
        if is_seated and any(holding_pipe(p, objects) for p in tcps):
            st["held_s"] += 1.0 / CONTROL_HZ
            if st["held_s"] >= HOLD_S:
                st["done"] = True
        else:
            st["failed"] = True  # let go (or pulled out) before 30 s: the pipe pushes back out
    if DEBUG:
        st["n"] += 1
        if st["n"] % 20 == 0:
            print(f"DBG t={st['t']:.1f} seated={is_seated} at={st['seated_at']} pipe_low={pipe.low[2] - TOP:.4f} "
                  f"yaw={_off(_yaw(pipe), st['pipe_yaw0']):.0f} turned={st['turned']} held={st['held_s']:.1f} "
                  f"done={st['done']} failed={st['failed']} tee_moved={moved:.4f} viol={st['why']} "
                  f"tee_dz={tee.pos[2] - st['tee0'][0][2]:.4f} tee_yaw={_off(_yaw(tee), ref[1]):.1f} ref={st['ref'] is not None} "
                  f"pipe_xy_off={math.hypot(pipe.pos[0] - tee.pos[0], pipe.pos[1] - tee.pos[1]):.4f}", flush=True)
    if st["violated"]:
        return 0.0
    if st["seated_at"] is None:
        return 0.0
    in_time = st["seated_at"] <= WET_S
    if not in_time or not st["turned"] or st["failed"]:
        return 0.25
    return 1.0 if st["done"] and is_seated else 0.5


# -- scripted demo ----------------------------------------------------------------------------

GRIP_FROM_BOTTOM = PIPE_L / 2 + 0.005  # the right fingers grip the pipe near its middle: it can't swing


def _tcp_for(bottom_z):
    return bottom_z + GRIP_FROM_BOTTOM


TURN_END = -0.8  # the gripper turns ~136 degrees the other way: the pipe slips in the fingers, so it turns about 90


def demo():
    travel = TOP + 0.17
    q = math.pi / 2
    hold = (TEE_C[0], 0.10, TEE_C[2] + 0.005)  # 10 cm out along the stub: clear of the right gripper
    left = [
        Waypoint(2.0, (hold[0], hold[1], travel), FINGER_OPEN, "over the tee's stub", q),
        Waypoint(1.5, hold, FINGER_OPEN, "down around it", q),
        Waypoint(0.6, hold, FINGER_CLOSED, "hold the tee still", q),
        Waypoint(41.5, hold, FINGER_CLOSED, "keep holding it", q),
    ]
    px, py = HOLDER_XY
    sx, sy = TEE_C[0], TEE_C[1]
    grip = _tcp_for(TOP + 0.002)
    right = [
        Waypoint(2.0, (px, py, travel), FINGER_OPEN, "over the pipe", q),
        Waypoint(1.5, (px, py, grip), FINGER_OPEN, "down around it", q),
        Waypoint(0.6, (px, py, grip), FINGER_CLOSED, "grip it", q),
        Waypoint(1.5, (px, py, _tcp_for(TOP + 0.07)), FINGER_CLOSED, "lift it out of the holder", q),
        Waypoint(2.0, (sx, sy, _tcp_for(SOCKET_TOP + FLARE + 0.015)), FINGER_CLOSED, "over the tee's socket", q),
        Waypoint(1.5, (sx, sy, _tcp_for(SOCKET_TOP + 0.003)), FINGER_CLOSED, "line it up", q),
        Waypoint(0.5, (sx, sy, _tcp_for(SOCKET_TOP + 0.003)), FINGER_CLOSED, "steady", q),
        Waypoint(1.5, (sx, sy, _tcp_for(SEAT_Z + 0.012)), FINGER_CLOSED, "push it in", 0.35),
        Waypoint(1.2, (sx, sy, _tcp_for(SEAT_Z - 0.008)), FINGER_CLOSED, "fully in, with a quarter turn", TURN_END),
        Waypoint(33.0, (sx, sy, _tcp_for(SEAT_Z - 0.008)), FINGER_CLOSED, "hold it 30 seconds (and keep holding)", TURN_END),
    ]
    return {"left": left, "right": right}
