"""Proven demo primitives for yam_arena scripted demos (pipeline v2). NumPy only; no simulator.

Each function returns demo waypoints in the format every task demo uses: a list of ``yam_arena.demo.Waypoint``
(duration s, TCP position m in the world, finger joint target m, label, yaw or ``tiltgrip.Grip``), or for the two
two-handed primitives a ``{"left": [...], "right": [...]}`` plan. A task's ``demo()`` concatenates them:

    import primitives as P
    take = P.grasp_down_rigid((0.30, 0.15, TOP + 0.02), width=0.028, height=0.040)
    return {"left": take + P.place_in_fixture((0.25, 0.20, TOP + 0.03), part_height=0.040, width=0.028,
                                              after=take) + P.hold_final("left")}

Every number below comes from a scripted demo that reached 1.0, on the GPU (Isaac Sim) or in physcheck (MuJoCo), and
says which; ``PROVEN`` lists each primitive's sources and ``README.md`` is the catalogue. A request outside the tested
range raises ``ValueError`` unless ``unproven=True``: then the step needs its own crux prototype (``../crux``).

Conventions: world frame x forward from the arm bases, y left, z up, metres. ``grip`` is a waypoint yaw (rad, straight
down) or a ``tiltgrip.Grip``. The TCP is the pinch point between the finger pads; the pad tips are 13 mm past it along
the approach axis. A finger target f puts the pads ``-2 f`` apart (0 shut, ``FINGER_OPEN`` = 94 mm).
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path

import numpy as np

try:  # a task folder's own copy first (physcheck and the GPU import it from there); any copy works with any other
    from tiltgrip import Grip
except ImportError:  # the library's own tests: the pipeline's copy, loaded by path so nothing else there is importable
    _spec = importlib.util.spec_from_file_location("tiltgrip", Path(__file__).resolve().parent.parent / "tiltgrip.py")
    _tg = importlib.util.module_from_spec(_spec)
    sys.modules["tiltgrip"] = _tg
    _spec.loader.exec_module(_tg)
    Grip = _tg.Grip

from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, READY  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

PRIMITIVES = ["grasp_down_rigid", "grasp_tilted", "steady_and_operate", "place_in_fixture", "roll_thumb_wheel",
              "lift_hinged_lid", "push_slider", "handover", "press_with_overshoot", "squeeze_bulb", "hold_final"]

# Where each primitive's numbers come from. GPU: the demo reached 1.0 in Isaac Sim; physcheck: 1.0 in MuJoCo only.
PROVEN = {
    "grasp_down_rigid": ["GPU fill-four-tubes-and-tick-each-one-off + pipette-dye-into-cups: pipette bulb, 28 mm, "
                         "straight down, slip 1.3 mm", "physcheck empty-the-catheter-bag-into-the-graduate: alcohol "
                         "pad, 19 mm, slip 2.7 mm"],
    "grasp_tilted": ["GPU draw-a-splinter-out-with-tweezers: pitch 40 deg, slip 3.8 mm",
                     "physcheck roll-the-iv-roller-clamp-to-20-drops-a-minute: 20 mm housing, pitch 30, slip 0.8 mm",
                     "physcheck empty-the-catheter-bag-into-the-graduate: 12 mm spout, pitch -45"],
    "steady_and_operate": ["GPU feed-a-carton-into-the-case-sealer-holding-its-flaps: press flaps + push carton",
                           "physcheck roll-the-iv-roller-clamp-to-20-drops-a-minute: pinch housing + roll wheel",
                           "physcheck empty-the-catheter-bag-into-the-graduate: hold spout + slide tap"],
    "place_in_fixture": ["GPU fill-four-tubes-and-tick-each-one-off + pipette-dye-into-cups: pipette stem into its "
                         "holder (8 mm in 22 mm)", "physcheck empty-the-catheter-bag-into-the-graduate: spout into "
                         "its holder"],
    "roll_thumb_wheel": ["physcheck roll-the-iv-roller-clamp-to-20-drops-a-minute: 14 mm wheel, 1.5 N breakaway"],
    "lift_hinged_lid": ["physcheck seat-a-filter-pad-in-its-frame-and-latch-the-gate: fork round a 3 mm wire, "
                        "0.25 kg gate, 98 -> 80 deg"],
    "push_slider": ["GPU fill-four-tubes-and-tick-each-one-off: closed fingers push the tick tabs 38 mm",
                    "GPU feed-a-carton-into-the-case-sealer-holding-its-flaps: push the carton, 20 mm lead"],
    "handover": [],  # no demo that reached 1.0 hands a part between arms
    "press_with_overshoot": ["GPU feed-a-carton-into-the-case-sealer-holding-its-flaps: 22 mm into sprung flaps",
                             "physcheck seat-a-filter-pad-in-its-frame-and-latch-the-gate: 32-36 mm presses"],
    "squeeze_bulb": ["GPU fill-four-tubes-and-tick-each-one-off + pipette-dye-into-cups: 9 mm bulb travel"],
    "hold_final": ["all seven: 2.2-5 s still at the end; IV clamp and filter pad 3 s"],
}

# -- geometry ---------------------------------------------------------------------------------------------------------
PAD_TIP = PAD_TIP_PAST_TCP  # 0.013 m: pad tips past the TCP along the approach
MAX_GAP = -2.0 * FINGER_OPEN  # 0.0939 m between the pads, fully open
PAD_HALF_X = 0.0137  # m: the closed fingers' half-width across the pads (fill-four-tubes PAD_HALF_X)
TOP = station.TABLE_TOP  # 0.7475 m, the station table top
BASES = station.BASES

# -- grasp margins ----------------------------------------------------------------------------------------------------
SQUEEZE_RIGID = 0.002  # m commanded into each side of a rigid part: IV housing (PINCH 16 mm on 20 mm, 18.9 N, slip
#                        0.8 mm) and catheter spout (HOLD_SPOUT 2 mm into the 12 mm spout). The pipette's 0.5 N grip
#                        crept 50 mm in MuJoCo (agreement.md): rigid parts get the firm 2 mm.
SQUEEZE_SPRUNG = 0.0015  # m into each sprung or soft side: pipette bulb HOLD (GPU, slip 1.3 mm), alcohol pad HOLD_PAD
OPEN_CLEARANCE = 0.010  # m each side when opening round a part: IV OPEN 40 mm round its 20 mm housing
TIP_CLEARANCE = 0.003  # m: pad tips at least this far above the surface a part rests on (IV PINCH_Z, "3 mm clear")
GRIP_ABOVE_CENTRE = 0.004  # m: the TCP this far above a gripped part's centre (pipette GRIP_ABOVE_CENTRE)
MIN_PAD_OVERLAP = 0.005  # m of part between the pads, along the approach; the cannula held 3.6 mm and lost it
RIGID_WIDTH = (0.020, 0.080)  # m: grasp_down_rigid's range (YAM profile); demos proved 19-28 mm straight down
TILTED_WIDTH = (0.010, 0.080)  # m: grasp_tilted's range; demos proved 4.6 mm sprung tweezers, 12 mm spout, 20 mm
TILT_RANGE = (-45.0, 65.0)  # deg pitch tested: catheter spout -45 (toward the robot), tweezers in use 65

# -- heights and distances --------------------------------------------------------------------------------------------
APPROACH = 0.060  # m above a straight-down grasp, or back along a tilted one, before closing in: IV LIFT 60 mm,
#                   catheter pad right_pad_tcp(0.06)
TILT_APPROACH = 0.050  # m back along a tilted approach: tweezers APPROACH 50 mm (catheter 40, IV 60)
RIM_CLEARANCE = 0.020  # m: a carried part's lowest point over any rim (fill-four-tubes TIP_TRAVEL, rim + 20 mm)

# -- timing (s) and speeds (m/s): a segment never runs faster than its proven pace --------------------------------------
T_APPROACH = 2.0  # to the approach point (pipette 1.5-2.0, catheter pad 2.0, tweezers 2.0)
T_DESCEND = 1.5  # approach point to the grasp (pipette 1.5, catheter pad 1.5, tweezers 1.5)
T_CLOSE = 0.8  # fingers closing (pipette, alcohol pad, spout 0.8; IV 1.0)
T_LIFT = 1.5  # back up to the approach point (pad 1.5, IV 1.5, pipette 1.2-2.0)
T_SETTLE = 0.8  # still above a target before going down (pipette "settle" 0.8)
T_RELEASE = 1.0  # fingers opening (pipette "let go" 1.0, spout 0.8)
V_NEAR = 0.040  # m/s mean within an approach distance of a part (60 mm in 1.5 s)
V_CARRY = 0.100  # m/s mean carrying across the table (pipette: about 0.2 m in 2.0 s)

# -- force from position: the arm is a soft spring (YAM profile, checked against the demos) ------------------------------
K_VERTICAL = 0.45  # N/mm commanded past contact, vertically
K_ALONG = 0.20  # N/mm along a horizontal push or pull (splinter: "about 5 mm per newton along a pull")
OVERSHOOT_RANGE = (0.010, 0.040)  # m: presses proven from 10 mm (filter-pad table aim) to 36 mm (filter-pad panel)

# -- thumb wheel (IV roller clamp, physcheck) ------------------------------------------------------------------------
WHEEL_REST_BACK = (0.040, 0.010)  # m back off the face: over the wheel, then at the wheel
WHEEL_EASE = 0.003  # m off the face between pushes, while watching (EASE)
WHEEL_BACK_OFF = 0.020  # m straight back off the face when done (BACK_OFF)
WHEEL_PUSH_SPEED = 0.00125  # m/s of fingertip: the long push sent ~10 mm in 8 s
WHEEL_LAG = ((1.5, 0.0011), (3.0, 0.0035))  # (breakaway N, m the wheel stops short of where it is sent)
WHEEL_CARRY = 0.0006  # m a push can carry the wheel past what it was sent (0.3-0.6 mm)
WHEEL_BREAKAWAY_MAX = 3.0  # N: tested at 1.5 and 3 N

# -- hinged lid by a fork round its edge wire (filter-pad gate, physcheck) --------------------------------------------
FORK_SLACK = 0.0185  # m each side of the edge inside the open fork: 40 mm fork round a 3 mm wire
FORK_DEPTH = 0.035  # m: the edge this far along the approach past the TCP (48 mm in from the fingertips)
FORK_OFF = 0.013  # m the TCP is shifted so the trailing finger bears on the edge
FORK_PITCH = 60.0  # deg: fork approach tilted this far from straight down, away from the robot
LID_DEG_PER_S = 18.0 / 2.5  # the gate was turned 18 degrees in 2.5 s
LID_STEP_DEG = 6.0  # one waypoint per this many degrees of arc
LID_MAX_DEG = 30.0  # turned in one primitive call; the gate was proven over 18 deg

# -- slider push (fill-four-tubes tick tab, GPU) -----------------------------------------------------------------------
PUSH_START_BACK = 0.012  # m behind the face before the push (BEHIND_X)
PUSH_BACK_OFF = 0.006  # m back after the push ("back off")
PUSH_SPEED = 0.025  # m/s of TCP: 50 mm in 2.0 s
PUSH_TIPS_ABOVE = 0.009  # m: closed fingertips this far above the surface the slider rides on (PUSH_Z)

# -- squeeze bulb (pipette, GPU) ---------------------------------------------------------------------------------------
SQUEEZE_PAST = 0.001  # m each side past the bulb's full travel, so it bottoms out
T_SQUEEZE = 0.8

# -- end of the demo --------------------------------------------------------------------------------------------------
HOME_S = 3.0  # s to the start pose (IV clamp HOME_S)
FINAL_HOLD_S = 3.0  # s still at the end (IV clamp HOLD_S, filter pad, splinter "keep still")

# -- two hands -------------------------------------------------------------------------------------------------------
STEADY_SETTLE = 0.5  # s between the steady hand's grip and the first move of the other (carton "wait for the flaps",
#                      IV clamp right hand waits 0.5 s after the pinch)
SPACING_DOWN = 0.180  # m between two straight-down TCPs (envelope: wrists touch closer than 18-20 cm)
SPACING_TILTED = 0.040  # m between hands tilted toward each other (envelope: 11-16 mm clearance at 40 mm)


# ======================================================================================================================
# helpers


def finger_for_gap(gap: float) -> float:
    """Finger joint target (m) that puts the pads ``gap`` m apart (clipped to what the gripper does)."""
    return -float(np.clip(gap, 0.0, MAX_GAP)) / 2.0


def gap_for_finger(finger: float) -> float:
    return -2.0 * float(finger)


def rotation(grip) -> np.ndarray:
    """The TCP rotation of a waypoint yaw or Grip (columns: closing axis, palm normal, approach axis)."""
    return np.asarray(_demo.grip_rotation(grip), dtype=float)


def approach_axis(grip) -> np.ndarray:
    return rotation(grip)[:, 2].copy()


def closing_axis(grip) -> np.ndarray:
    return rotation(grip)[:, 0].copy()


def tilt_deg(grip) -> float:
    return float(np.degrees(np.arccos(np.clip(-approach_axis(grip)[2], -1.0, 1.0))))


def _p(v) -> tuple[float, float, float]:
    return tuple(float(x) for x in np.asarray(v, dtype=float).reshape(3))


def _t(dist: float, base: float, speed: float = V_NEAR) -> float:
    """A segment's duration: its proven base time, longer if the distance would make it faster than ``speed``."""
    return max(base, float(dist) / speed)


def _need(ok: bool, unproven: bool, msg: str) -> None:
    if not ok and not unproven:
        raise ValueError(msg + " (outside the proven range: pass unproven=True and prototype the crux)")


def duration(ws) -> float:
    return float(sum(w.duration for w in ws))


def last(ws) -> Waypoint:
    if not ws:
        raise ValueError("no waypoints to continue from")
    return ws[-1]


def sync(plan: dict, label: str = "wait") -> dict:
    """Pad every arm's list with a still waypoint so all end together."""
    end = max(duration(ws) for ws in plan.values())
    out = {}
    for arm, ws in plan.items():
        ws = list(ws)
        gap = end - duration(ws)
        if gap > 1e-9:
            w = last(ws)
            ws.append(Waypoint(gap, w.pos, w.finger, label, w.yaw))
        out[arm] = ws
    return out


def wait(ws, seconds: float, label: str = "wait") -> list[Waypoint]:
    w = last(ws)
    return [Waypoint(float(seconds), w.pos, w.finger, label, w.yaw)]


def home(arm: str) -> tuple[tuple[float, float, float], Grip]:
    """(TCP position, grip) of the arm's start pose (READY) at the station."""
    pos, rot = ik.tcp_pose(np.asarray(READY, dtype=float), BASES[arm])
    return _p(pos), Grip.from_matrix(rot)


def check_ik(waypoints, arm: str, hz: float = 20.0) -> tuple[float, float]:
    """Play the waypoints as yam_scripted does; (worst IK position error mm, worst orientation error deg). Keep both
    under 2: above that the pose is out of reach and the demo is not doing what it says."""
    plan = _demo.ArmPlan(list(waypoints), BASES[arm])
    for k in range(int(math.ceil(plan.duration * hz)) + 2):
        plan.action7((k + 1) / hz)
    return plan.worst_ik_mm, plan.worst_ik_deg


class StateFeed:
    """The latest object states, for waypoints worked out when the arm starts toward them (a scripted demo may read
    the simulator's state to plan; nothing is moved by script). Register it in the task file:

        FEED = P.StateFeed()
        on_step(FEED.update)   # benchkit's hooks
        on_reset(FEED.reset)
    """

    def __init__(self):
        self.states, self.steps = None, 0

    def update(self, states) -> None:
        self.states, self.steps = states, self.steps + 1

    def reset(self, states=None) -> None:
        self.states, self.steps = states, 0


class LivePoint:
    """A Waypoint position given by ``fn(states)``, worked out the first time it is read once the run has stepped
    (when ``ArmPlan`` starts the arm toward it) and kept. Read while planning (no step yet), it gives ``fallback`` and
    keeps nothing, so planning-time checks see the offline plan."""

    def __init__(self, fn, feed: StateFeed | None, fallback):
        self.fn, self.feed, self.fallback, self.value = fn, feed, fallback, None

    def resolve(self) -> tuple[float, float, float]:
        if self.value is not None:
            return self.value
        if self.feed is None or not self.feed.steps or self.feed.states is None:
            return _p(self.fallback() if callable(self.fallback) else self.fallback)
        self.value = _p(self.fn(self.feed.states))
        return self.value

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.resolve(), dtype=dtype or float)

    def __iter__(self):
        return iter(self.resolve())

    def __len__(self):
        return 3

    def __getitem__(self, i):
        return self.resolve()[i]


# ======================================================================================================================
# the primitives


def grasp_down_rigid(centre, width: float, height: float, *, grip=0.0, support_z: float | None = None,
                     squeeze: float = SQUEEZE_RIGID, approach: float = APPROACH, lift: float | None = APPROACH,
                     tip_clearance: float = TIP_CLEARANCE, label: str = "the part",
                     unproven: bool = False) -> list[Waypoint]:
    """Straight-down grasp of a rigid part ``width`` m across the closing axis and ``height`` m tall, centred at
    ``centre``; then (unless ``lift`` is None) lift it ``lift`` m. ``grip`` is the yaw (0 closes along world y).

    The fingers open to the width plus 10 mm a side, come down from ``approach`` m above, close to 2 mm into each side
    (``SQUEEZE_SPRUNG`` for a sprung or soft part) and lift. The TCP sits 4 mm above the centre, but never lower than
    puts the pad tips 3 mm above the surface the part rests on (``support_z``, default its bottom); at least 5 mm of
    the part must then lie between the pads."""
    c = np.asarray(centre, dtype=float)
    _need(RIGID_WIDTH[0] - 1e-9 <= width <= RIGID_WIDTH[1] + 1e-9, unproven,
          f"grasp width {width * 1000:.1f} mm not in {RIGID_WIDTH[0] * 1000:.0f}-{RIGID_WIDTH[1] * 1000:.0f} mm")
    _need(tilt_deg(grip) < 1.0, unproven, "grasp_down_rigid takes a straight-down grip; use grasp_tilted")
    support = c[2] - height / 2 if support_z is None else float(support_z)
    tcp_z = max(c[2] + GRIP_ABOVE_CENTRE, support + tip_clearance + PAD_TIP)
    overlap = (c[2] + height / 2) - (tcp_z - PAD_TIP)
    _need(overlap >= MIN_PAD_OVERLAP - 1e-9, unproven,
          f"only {overlap * 1000:.1f} mm of the part between the pads (need {MIN_PAD_OVERLAP * 1000:.0f})")
    at = np.array([c[0], c[1], tcp_z])
    above = at + np.array([0.0, 0.0, approach])
    opened = finger_for_gap(width + 2 * OPEN_CLEARANCE)
    hold = finger_for_gap(width - 2 * squeeze)
    ws = [
        Waypoint(T_APPROACH, _p(above), opened, f"over {label}", grip),
        Waypoint(_t(approach, T_DESCEND), _p(at), opened, f"down round {label}", grip),
        Waypoint(T_CLOSE, _p(at), hold, f"grip {label}", grip),
    ]
    if lift:
        ws.append(Waypoint(_t(lift, T_LIFT), _p(at + np.array([0.0, 0.0, lift])), hold, f"lift {label}", grip))
    return ws


def grasp_tilted(centre, width: float, *, grip: Grip, depth: float = 0.0, squeeze: float = SQUEEZE_RIGID,
                 approach: float = TILT_APPROACH, retreat: float | None = TILT_APPROACH, label: str = "the part",
                 unproven: bool = False) -> list[Waypoint]:
    """Tilted grasp (``tiltgrip.Grip``) of a part ``width`` m across the closing axis: come in along the grip's own
    approach axis from ``approach`` m back, close 2 mm into each side, then (unless ``retreat`` is None) draw it back
    out the same way. ``centre`` is the point to hold; ``depth`` puts it that far past the TCP along the approach
    (0: centred between the pads; up to the 13 mm pad tips)."""
    _need(TILTED_WIDTH[0] - 1e-9 <= width <= TILTED_WIDTH[1] + 1e-9, unproven,
          f"grasp width {width * 1000:.1f} mm not in {TILTED_WIDTH[0] * 1000:.0f}-{TILTED_WIDTH[1] * 1000:.0f} mm")
    pitch = getattr(grip, "_angles", None)
    pitch_deg = math.degrees(pitch[1]) if pitch else tilt_deg(grip)
    _need(TILT_RANGE[0] - 1e-6 <= pitch_deg <= TILT_RANGE[1] + 1e-6, unproven,
          f"pitch {pitch_deg:.0f} deg not in {TILT_RANGE[0]:.0f}..{TILT_RANGE[1]:.0f}")
    _need(0.0 <= depth <= PAD_TIP, unproven, "depth past the TCP must be 0-13 mm (within the pads)")
    a = approach_axis(grip)
    at = np.asarray(centre, dtype=float) - depth * a
    opened = finger_for_gap(min(width + 2 * OPEN_CLEARANCE, MAX_GAP))
    hold = finger_for_gap(width - 2 * squeeze)
    ws = [
        Waypoint(T_APPROACH, _p(at - approach * a), opened, f"in line with {label}", grip),
        Waypoint(_t(approach, T_DESCEND), _p(at), opened, f"onto {label}", grip),
        Waypoint(T_CLOSE, _p(at), hold, f"grip {label}", grip),
    ]
    if retreat:
        ws.append(Waypoint(_t(retreat, T_LIFT), _p(at - retreat * a), hold, f"draw {label} back", grip))
    return ws


def steady_and_operate(steady: list[Waypoint], operate: list[Waypoint], *, steady_arm: str = "left",
                       release_gap: float | None = None, retreat: float = APPROACH, settle: float = STEADY_SETTLE,
                       unproven: bool = False) -> dict:
    """One hand steadies, the other operates. ``steady`` ends with the part held (or pressed) where it is to stay,
    e.g. ``grasp_tilted(..., retreat=None)``; ``operate`` is the other hand's moves, its first waypoint a still pose
    clear of the part. The operating hand waits there until the steady hand has gripped and ``settle`` s more; the
    steady hand holds until the operating hand's last move ends, then opens to ``release_gap`` (default: 10 mm a
    side wider than it held) and backs ``retreat`` m out along its approach. Both lists end together.

    Hand spacing (TCP to TCP, YAM profile): both straight down, at least 18 cm; either hand tilted, at least 4 cm (the
    IV clamp's hands, tilted 30 and 45 degrees toward each other, worked on one 64 mm housing). The real test is the
    arm-to-arm clearance the crux check measures."""
    if not steady or not operate:
        raise ValueError("both hands need waypoints")
    other = "right" if steady_arm == "left" else "left"
    hold = last(steady)
    tilted = tilt_deg(hold.yaw) > 10.0 or any(tilt_deg(w.yaw) > 10.0 for w in operate)
    need = SPACING_TILTED if tilted else SPACING_DOWN
    gap = min(float(np.linalg.norm(np.asarray(w.pos) - np.asarray(hold.pos))) for w in operate)
    _need(gap >= need - 1e-9, unproven,
          f"the hands come {gap * 1000:.0f} mm apart (need {need * 1000:.0f} mm {'tilted' if tilted else 'straight down'})")
    t_grip = duration(steady) + settle
    first = operate[0]
    ops = [first]
    if t_grip > first.duration:
        ops.append(Waypoint(t_grip - first.duration, first.pos, first.finger, "wait for the steady hand", first.yaw))
    ops += operate[1:]
    keep = list(steady)
    t_end = duration(ops)
    if t_end > duration(keep):
        keep.append(Waypoint(t_end - duration(keep), hold.pos, hold.finger, "keep it steady", hold.yaw))
    opened = finger_for_gap(gap_for_finger(hold.finger) + 2 * OPEN_CLEARANCE) if release_gap is None \
        else finger_for_gap(release_gap)
    if gap_for_finger(hold.finger) < 0.001:  # a press with closed fingers: nothing to open
        opened = hold.finger
    a = approach_axis(hold.yaw)
    keep += [Waypoint(T_RELEASE, hold.pos, opened, "let go", hold.yaw),
             Waypoint(_t(retreat, T_LIFT), _p(np.asarray(hold.pos) - retreat * a), opened, "back away", hold.yaw)]
    return sync({steady_arm: keep, other: ops})


def place_in_fixture(target, *, after: list[Waypoint], part_below_tcp: float, rim_z: float, clearance: float,
                     release_gap: float | None = None, trim=(0.0, 0.0), grip=None, label: str = "the fixture",
                     unproven: bool = False) -> list[Waypoint]:
    """Carry a held part over a fixture and set it in. ``target`` is where the part's lowest point goes (world);
    ``part_below_tcp`` is how far that point hangs below the TCP; ``rim_z`` the fixture's (and the path's) highest rim;
    ``clearance`` the fixture's play each side of the part (m). The part is carried 20 mm over ``rim_z``, held still
    0.8 s above the opening, lowered, let go and the hand goes back up. ``trim`` (x, y) aims off by what the arm's sag
    measured in the simulator (a 15 cm pipette needed 4-13 mm)."""
    _need(clearance >= 0.003 - 1e-9, unproven, f"{clearance * 1000:.1f} mm of play a side (proven 7 mm; need 3)")
    w0 = last(after)
    grip = w0.yaw if grip is None else grip
    tx, ty, tz = (float(v) for v in target)
    x, y = tx + trim[0], ty + trim[1]
    travel = max(rim_z + RIM_CLEARANCE + part_below_tcp, float(w0.pos[2]))
    down = tz + part_below_tcp
    opened = finger_for_gap(gap_for_finger(w0.finger) + 2 * OPEN_CLEARANCE) if release_gap is None \
        else finger_for_gap(release_gap)
    over = np.array([x, y, travel])
    carry = float(np.linalg.norm(over - np.asarray(w0.pos)))
    return [
        Waypoint(_t(carry, T_APPROACH, V_CARRY), _p(over), w0.finger, f"over {label}", grip),
        Waypoint(T_SETTLE, _p(over), w0.finger, "settle", grip),
        Waypoint(_t(travel - down, 2.0), (x, y, down), w0.finger, f"into {label}", grip),
        Waypoint(T_RELEASE, (x, y, down), opened, "let go", grip),
        Waypoint(_t(travel - down, T_LIFT), _p(over), opened, "up", grip),
    ]


def wheel_lag(breakaway_n: float) -> float:
    """How far short of where it is sent the compliant arm leaves a wheel that holds until pushed past
    ``breakaway_n`` (m): 1.1 mm at 1.5 N, 3.5 mm at 3 N in physcheck, straight between, in proportion below."""
    (f0, l0), (f1, l1) = WHEEL_LAG
    if breakaway_n <= f0:
        return l0 * breakaway_n / f0
    return l0 + (breakaway_n - f0) * (l1 - l0) / (f1 - f0)


def roll_thumb_wheel(face, roll_dir, travel: float, *, grip: Grip | None = None, breakaway_n: float = 1.5,
                     lead: float | None = None, feed: StateFeed | None = None, read_travel=None,
                     rest_offset: float = 0.0, watch_s: float = 4.0, label: str = "the wheel",
                     unproven: bool = False) -> list[Waypoint]:
    """Roll a thumb wheel ``travel`` m from shut with a closed fingertip on its face. ``face`` is the middle of the face
    the fingertip pushes with the wheel shut (world), ``roll_dir`` the way the wheel travels. Default grip: tilted 45
    degrees along the travel (the IV clamp's right hand). The fingertip comes to the face from 40 mm then 10 mm back
    and rests on it 3 s; then one slow push (1.25 mm/s) to ``travel`` plus the arm's lag (``wheel_lag``); eases 3 mm off
    and waits ``watch_s`` s; backs 20 mm straight off and goes 60 mm up.

    Resting on the face moves the wheel by the arm's own offset, and the push carries that offset on: 3.5 mm in the
    crux check, which open loop left the wheel at 14.7 mm for 10.4. So, as the IV clamp demo does, pass ``feed`` (a
    StateFeed the task's on_step updates) and ``read_travel(states) -> m``: the push is then aimed short by the travel
    the wheel shows when the push starts (no lag lead: the offset already carries it; adding the 1.1 mm lead too
    left the wheel 1.0 mm over), and the moves after it follow. Without them ``rest_offset`` is assumed and the lag
    lead added. Expect the wheel within about 1 mm (up to 0.6 mm carry); finer needs the IV demo's read-and-correct
    pushes."""
    _need(0.0 < breakaway_n <= WHEEL_BREAKAWAY_MAX, unproven, f"breakaway {breakaway_n} N (tested 1.5-3 N)")
    _need(0.0 < travel <= 0.020, unproven, f"travel {travel * 1000:.1f} mm (the clamp's 20 mm run)")
    d = np.asarray(roll_dir, dtype=float)
    d = d / np.linalg.norm(d)
    if grip is None:
        yaw = math.atan2(d[1], d[0])  # pitch swings the fingertips along the travel
        grip = Grip(yaw, math.radians(45.0))
    a = approach_axis(grip)
    closed_loop = feed is not None and read_travel is not None
    if lead is None:  # aimed by the rest offset, the push already carries the lag (crux check: within 0.1 mm)
        lead = 0.0 if closed_loop else wheel_lag(breakaway_n)
    face = np.asarray(face, dtype=float)
    memo: dict = {}

    def tcp(s: float, back: float = 0.0) -> np.ndarray:
        return face + s * d - back * a - PAD_TIP * a

    def offset(states) -> float:
        if states is None or read_travel is None:  # planning, or open loop: nothing to read, nothing kept
            return rest_offset
        if "off" not in memo:  # the wheel's travel when the push starts, kept for the moves after it
            memo["off"] = float(read_travel(states))
        return memo["off"]

    def at(s_fn, back: float = 0.0, up: float = 0.0):
        def fn(states):
            return tcp(s_fn(offset(states)), back) + np.array([0.0, 0.0, up])

        if not closed_loop:
            return _p(fn(None))
        return LivePoint(fn, feed, lambda: fn(None))

    def pushed(off):
        return travel + lead - off

    def left(off):  # the fingertip's place along the travel after the push, the lag taken back off
        return travel - off

    return [
        Waypoint(3.0, _p(tcp(0.0, WHEEL_REST_BACK[0])), FINGER_CLOSED, f"over {label}", grip),
        Waypoint(2.0, _p(tcp(0.0, WHEEL_REST_BACK[1])), FINGER_CLOSED, f"fingertip to {label}", grip),
        Waypoint(1.5, _p(tcp(0.0)), FINGER_CLOSED, "onto its face", grip),
        Waypoint(3.0, _p(tcp(0.0)), FINGER_CLOSED, "rest on the face", grip),
        Waypoint(max(2.0, (travel + lead) / WHEEL_PUSH_SPEED), at(pushed), FINGER_CLOSED, f"roll {label}", grip),
        Waypoint(1.0, at(left, WHEEL_EASE), FINGER_CLOSED, f"ease off {label}", grip),
        Waypoint(watch_s, at(left, WHEEL_EASE), FINGER_CLOSED, "watch", grip),
        Waypoint(1.5, at(left, WHEEL_BACK_OFF), FINGER_CLOSED, f"straight back off {label}", grip),
        Waypoint(1.5, at(left, WHEEL_BACK_OFF, APPROACH), FINGER_CLOSED, "up", grip),
    ]


def lid_edge(hinge, axis, edge, angle_deg: float) -> np.ndarray:
    """``edge`` (world) turned ``angle_deg`` about the hinge line through ``hinge`` along ``axis`` (right hand)."""
    from yam_arena.urdf import axis_angle_matrix

    k = np.asarray(axis, dtype=float)
    k = k / np.linalg.norm(k)
    h = np.asarray(hinge, dtype=float)
    return h + axis_angle_matrix(k, math.radians(angle_deg)) @ (np.asarray(edge, dtype=float) - h)


def fork_grip(arm: str = "left", pitch_deg: float = FORK_PITCH) -> Grip:
    """The filter-pad gate's fork: approach tilted ``pitch_deg`` away from the robot, closing axis in the x-z plane."""
    p = math.radians(pitch_deg)
    a = np.array([math.sin(p), 0.0, -math.cos(p)])
    c = np.array([math.cos(p), 0.0, math.sin(p)]) * (-1.0 if arm == "left" else 1.0)
    return Grip.from_matrix(np.column_stack([c, np.cross(a, c), a]))


def lift_hinged_lid(hinge, axis, edge, angle_deg: float, *, lip: float = 0.003, grip: Grip | None = None,
                    arm: str = "left", label: str = "the lid", unproven: bool = False) -> list[Waypoint]:
    """Turn a hinged lid or gate ``angle_deg`` about its hinge (right hand about ``axis``) with an open fork round its
    free edge: ``edge`` is the free edge's centre now (world), ``lip`` its thickness across the fork. The fork opens
    18.5 mm a side round the lip, comes in 60 mm back along its approach (tilted 60 degrees away), takes the edge
    35 mm past the TCP, shifts 13 mm so the trailing finger bears on it, follows the edge's arc at 7.2 deg/s (one
    waypoint per 6 deg) and pulls back off. Proven pushing a 0.25 kg wire gate 18 deg over its top."""
    _need(0.0 < abs(angle_deg) <= LID_MAX_DEG, unproven, f"{angle_deg:.0f} deg in one call (proven 18, limit 30)")
    fork = lip + 2 * FORK_SLACK
    _need(fork <= MAX_GAP, unproven, f"a {lip * 1000:.0f} mm lip needs a {fork * 1000:.0f} mm fork")
    grip = fork_grip(arm) if grip is None else grip
    a, c = approach_axis(grip), closing_axis(grip)
    f = finger_for_gap(fork)
    n = max(1, int(math.ceil(abs(angle_deg) / LID_STEP_DEG)))

    def at(theta: float, off: float, back: float = FORK_DEPTH):
        p = lid_edge(hinge, axis, edge, theta)
        ahead = lid_edge(hinge, axis, edge, theta + math.copysign(1.0, angle_deg)) - p  # the edge's way at theta
        side = math.copysign(1.0, float(np.dot(ahead, c))) if abs(np.dot(ahead, c)) > 1e-9 else 1.0
        return _p(p + back * a + side * off * c)

    ws = [
        Waypoint(2.0, at(0.0, 0.0, FORK_DEPTH - APPROACH), f, f"reach for {label}", grip),
        Waypoint(1.5, at(0.0, 0.0), f, f"fork round the edge of {label}", grip),
        Waypoint(1.0, at(0.0, FORK_OFF), f, "finger on the edge", grip),
    ]
    for i in range(1, n + 1):
        th = angle_deg * i / n
        ws.append(Waypoint(abs(angle_deg) / n / LID_DEG_PER_S, at(th, FORK_OFF), f, f"turn {label}", grip))
    end = np.asarray(at(angle_deg, 0.0))
    ws += [Waypoint(0.7, _p(end - 0.07 * a), f, "fork off the edge", grip),
           Waypoint(1.5, _p(end - 0.07 * a + np.array([0.0, 0.0, 0.05])), f, "clear", grip)]
    return ws


def push_slider(face, direction, travel: float, *, surface_z: float, force_n: float = 0.0, grip=0.0,
                travel_z: float | None = None, label: str = "the slider", unproven: bool = False) -> list[Waypoint]:
    """Push a slider (tab, drawer, carton) ``travel`` m along ``direction`` with the closed fingertips, straight
    down. ``face`` is the point on the face to push (world; its height is ignored), ``surface_z`` the top of what the
    slider rides on: the fingertips run 9 mm above it. Start 12 mm behind the face, push at 25 mm/s, back 6 mm off,
    then up. ``force_n`` is the force the slider needs: the arm lags 5 mm per newton along a push, so the push is
    sent that much further (the carton's 20 mm lead). Aim ``travel`` 2 mm short of any hard stop."""
    d = np.asarray(direction, dtype=float)
    d = np.array([d[0], d[1], 0.0])
    d /= np.linalg.norm(d)
    lead = force_n / K_ALONG / 1000.0  # m: N / (N/mm) gives mm
    _need(lead <= 0.025 + 1e-9, unproven, f"{force_n} N needs {lead * 1000:.0f} mm of lead (proven 20)")
    f = np.array([face[0], face[1], surface_z + PUSH_TIPS_ABOVE + PAD_TIP])
    behind = f - (PAD_HALF_X + PUSH_START_BACK) * d
    pushed = f + (travel + lead - PAD_HALF_X) * d
    hz = (surface_z + 0.100) if travel_z is None else travel_z
    up = np.array([0.0, 0.0, hz - f[2]])
    dist = float(np.linalg.norm(pushed - behind))
    return [
        Waypoint(1.5, _p(behind + up), FINGER_CLOSED, f"behind {label}", grip),
        Waypoint(0.5, _p(behind + up), FINGER_CLOSED, "settle", grip),
        Waypoint(_t(up[2], 1.0, 0.05), _p(behind), FINGER_CLOSED, "down", grip),
        Waypoint(0.4, _p(behind), FINGER_CLOSED, "settle", grip),
        Waypoint(_t(dist, 2.0, PUSH_SPEED), _p(pushed), FINGER_CLOSED, f"push {label}", grip),
        Waypoint(0.4, _p(pushed - PUSH_BACK_OFF * d - lead * d), FINGER_CLOSED, "back off", grip),
        Waypoint(_t(up[2], 1.0, 0.05), _p(pushed - PUSH_BACK_OFF * d - lead * d + up), FINGER_CLOSED, "up", grip),
    ]


def overshoot_for(force_n: float, vertical: bool = True) -> float:
    """Commanded travel past contact (m) for ``force_n`` at the tip: F / 0.45 N/mm vertically, F / 0.2 N/mm along a
    push, never under 10 mm. physcheck peaks ran 1-2x this (filter pad 35 N at 36 mm, carton 20 N at 22 mm)."""
    mm = force_n / (K_VERTICAL if vertical else K_ALONG)
    return max(OVERSHOOT_RANGE[0], mm / 1000.0)


def press_with_overshoot(contact, force_n: float, *, direction=(0.0, 0.0, -1.0), grip=0.0, finger: float = FINGER_CLOSED,
                         above: float = APPROACH, hold_s: float = 1.0, label: str = "press",
                         unproven: bool = False) -> list[Waypoint]:
    """Press the fingertips onto ``contact`` (world: where the pad tips meet the part) with about ``force_n``: the
    compliant arm only pushes in proportion to how far past the contact it is sent (``overshoot_for``). Comes from
    ``above`` m back, presses over 2 s, holds ``hold_s`` s, lifts off in 1 s. Closed fingers by default; the carton's
    open fingers straddling a seam work too (``finger``)."""
    d = np.asarray(direction, dtype=float)
    d /= np.linalg.norm(d)
    vertical = abs(d[2]) > 0.7
    o = overshoot_for(force_n, vertical)
    _need(o <= OVERSHOOT_RANGE[1] + 1e-9, unproven, f"{force_n} N needs {o * 1000:.0f} mm of overshoot (proven to 36)")
    a = approach_axis(grip)
    tip_to_tcp = -PAD_TIP * a
    c = np.asarray(contact, dtype=float)
    start = c + tip_to_tcp - above * d
    press = c + tip_to_tcp + o * d
    return [
        Waypoint(2.0, _p(start), finger, f"over the {label} point", grip),
        Waypoint(2.0, _p(press), finger, label, grip),
        Waypoint(hold_s, _p(press), finger, f"{label}: hold", grip),
        Waypoint(1.0, _p(start), finger, "lift off", grip),
    ]


def squeeze_bulb(tcp, rest_width: float, travel: float, *, grip=0.0, hold_squeeze: float = SQUEEZE_SPRUNG,
                 dwell_s: float = 0.0, label: str = "the bulb") -> list[Waypoint]:
    """Squeeze a held sprung bulb (each side moving ``travel`` m in, ``rest_width`` m across at rest) and let it go
    back to the holding grip, the TCP still at ``tcp``: closes to 1 mm a side past full travel in 0.8 s, so the sides
    bottom out, waits ``dwell_s``, and opens back to the 1.5 mm hold in 0.8 s."""
    hold = finger_for_gap(rest_width - 2 * hold_squeeze)
    full = finger_for_gap(rest_width - 2 * travel - 2 * SQUEEZE_PAST)
    ws = [Waypoint(T_SQUEEZE, _p(tcp), full, f"squeeze {label}", grip)]
    if dwell_s:
        ws.append(Waypoint(dwell_s, _p(tcp), full, f"keep {label} squeezed", grip))
    ws.append(Waypoint(T_SQUEEZE, _p(tcp), hold, f"release {label}", grip))
    return ws


def handover(part_centre, width: float, *, giver: str = "left", height: float = 0.040, giver_squeeze: float = SQUEEZE_RIGID,
             receiver_tilt_deg: float = 45.0, unproven: bool = False) -> dict:
    """NOT PROVEN: no demo that reached 1.0 hands a part between arms; prototype the crux first. The giver, holding
    the part straight down by its top at ``part_centre`` (already there), keeps still; the receiver comes in tilted
    ``receiver_tilt_deg`` toward the giver and grasps the part's lower half; the giver opens and lifts 60 mm; the
    receiver draws the part back. Built from grasp_tilted's numbers. Reach: the straight-down giver reaches the
    centre line between the arms only low and near (IK under 2 mm at x 0.22-0.25 m, part 10 cm over the table;
    13 mm off at x 0.30), so pass the part there."""
    _need(False, unproven, "handover has no proven demo")
    c = np.asarray(part_centre, dtype=float)
    recv = "right" if giver == "left" else "left"
    toward = 1.0 if recv == "right" else -1.0  # the receiver reaches toward +y (right arm) or -y (left arm)
    r = math.radians(receiver_tilt_deg)
    a = np.array([0.0, toward * math.sin(r), -math.cos(r)])  # approach: down and toward the giver
    cl = np.array([1.0, 0.0, 0.0])  # fingers close along x, across the part
    rgrip = Grip.from_matrix(np.column_stack([cl, np.cross(a, cl), a]))
    low = c - np.array([0.0, 0.0, height / 4])
    take = grasp_tilted(low, width, grip=rgrip, retreat=None, label="the part", unproven=True)
    w_g = Waypoint(0.1, _p(c + np.array([0.0, 0.0, height / 4 + GRIP_ABOVE_CENTRE])),
                   finger_for_gap(width - 2 * giver_squeeze), "hold the part out", 0.0)
    t_rx = duration(take) + STEADY_SETTLE
    give = [w_g, Waypoint(t_rx, w_g.pos, w_g.finger, "keep still while it is taken", w_g.yaw),
            Waypoint(T_RELEASE, w_g.pos, finger_for_gap(width + 2 * OPEN_CLEARANCE), "let go", w_g.yaw),
            Waypoint(T_LIFT, _p(np.asarray(w_g.pos) + np.array([0, 0, APPROACH])),
                     finger_for_gap(width + 2 * OPEN_CLEARANCE), "up and away", w_g.yaw)]
    held = last(take)
    take = [Waypoint(0.1, take[0].pos, take[0].finger, "wait", rgrip)] + take
    take += [Waypoint(T_RELEASE + T_LIFT, held.pos, held.finger, "hold while the giver lets go", rgrip),
             Waypoint(T_LIFT, _p(np.asarray(held.pos) - TILT_APPROACH * a), held.finger, "draw the part back", rgrip)]
    return sync({giver: give, recv: take})


def hold_final(arm: str, *, finger: float = FINGER_OPEN, seconds: float = FINAL_HOLD_S, at=None, grip=None,
               after: list[Waypoint] | None = None) -> list[Waypoint]:
    """End the demo still: back to the start pose in 3 s (or to ``at``) and hold ``seconds`` s, so the score is read
    on a settled scene and the video's last frames show it done."""
    if at is None:
        pos, g = home(arm)
    else:
        pos, g = _p(at), (grip if grip is not None else (last(after).yaw if after else 0.0))
    dist = float(np.linalg.norm(np.asarray(pos) - np.asarray(last(after).pos))) if after else 0.0
    first = _t(dist, HOME_S, V_CARRY) if at is None else _t(dist, 0.5, V_CARRY)
    return [Waypoint(first, pos, finger, "back to the start pose" if at is None else "to rest", g),
            Waypoint(float(seconds), pos, finger, "hold still", g)]
