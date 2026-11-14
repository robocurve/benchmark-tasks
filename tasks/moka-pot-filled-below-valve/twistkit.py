"""twistkit: threaded caps, knobs, nuts and filters for a yam_arena task file (built on yamkit).

A threaded part (a bottle cap, a lid, a nut, a filter canister, a valve knob) turns about its axis on a thread that
couples turning to travel along the axis. This module builds such a part as a yamkit `Mechanism` and drives its
thread every control step.

    import yamkit as kit
    import twistkit as tk

    bottle = tk.Threaded("bottle1", pos=(0.20, 0.15, TOP + 0.05), size=(0.046, 0.046, 0.10), mass=0.12,
                         meshes=(...), colliders=(...),                  # the body (fixed if mass is None)
                         cap=tk.Cap("cap", anchor=(0, 0, 0.03), mass=0.01, meshes=(...), colliders=(...),
                                    grip=(0, 0, 0.01)),
                         thread=tk.Thread(pitch=0.004, turns=1.25, start=20.0))

The model (angles about the thread's axis, seen from the cap's top, i.e. looking down the axis at the part):
- Right-hand thread: turning the cap counterclockwise (seen from its top) loosens it; `hand=-1` makes it left-hand.
- The seat is where the cap first bottoms out on the body. Before it, the cap runs freely on the thread. Past it,
  turning further tightens it: the angle past the seat is the tightness, up to `max_tight` degrees, a hard stop.
  `Thread.start` is where every trial begins, in degrees past the seat (20 = snug and tight; -90 = a quarter turn
  loose).
- Loosening raises the cap along the axis by `pitch` per turn. Past `turns` full turns back from the seat the cap is
  off the thread: it is flagged off, the axial coupling lets go, and the cap can be lifted up to `free_lift` along the
  axis. It stays captive on that axis (it cannot be carried away or set down elsewhere), so a scorer should treat
  "came off" as the cap having been removed. Turning it back below `turns` re-engages the thread.
- Resistance: the thread holds the cap where it is left, gives way to a turning grip, and needs more torque past the
  seat (`resist` and `tight_resist`, N*m/rad, a hold toward the angle at the last control step, plus `drag`). They are
  kept small so that a free-standing body does not spin along with its cap when the cap is turned; if it does in a
  given scene, lower them or make the body heavier.

Scorer API (`objects` is what score() gets; key is "body/cap"):
- `turns(objects, key)`: turns backed off from the seat (negative: tightened past it).
- `past_seat_deg(objects, key)`: degrees past the seat in the tightening direction (positive = tight).
- `backed_off_deg(objects, key)`: the opposite sign, degrees loosened past the seat.
- `rise(objects, key)`: the cap's travel along the axis from the seat (m).
- `is_off(objects, key)`: off the thread now. `came_off(objects, key)`: off at any point in this trial.
- `held(objects, key)`: a gripper's fingers closed on the cap: its opening matches the cap's `width` (within 0.15 of
  fully open; an empty closed hand does not count), its tool point within 1.5 cm of the cap's axis and of the grip
  point along it.
- `gripped_deg(objects, key)`: degrees the cap was loosened while a gripper held it, net, this trial, counting only
  steps with the cap held at both ends (a cap that turned without a grip on it, e.g. knocked, does not add to this).
- `state(objects, key)`: all of the above as a `TwistState`.
All of them follow the trial: on each scene reset (yamkit's `kit.episode()`) the thread goes back to `start` and the
counters clear. `objects[key]` is the cap's link (pos = its origin, on the axis at the seat) and `objects[body]` the
body, as for any yamkit mechanism.

Demo helper: `twist_waypoints(xy, z_grip, degrees, ...)` is a regrasp loop for one arm with a straight-down grip:
open above the cap, descend, close, roll the wrist `stroke` degrees in small steps (rising with the thread), open,
roll back with the fingers open, close again, and repeat until `degrees` is reached, then lift clear. The YAM's wrist
roll (joint6) spans +-120 degrees and a straight-down grip already uses part of that, so one stroke can turn at most
about 200 degrees at the middle of the table and less toward the sides; `yaw_range(side, xyz)` reports the usable
yaw range at a point (from the arm's IK, no simulator) and `twist_waypoints` checks every stroke against it. A
parallel gripper grips the same at yaw and yaw + 180 degrees, so strokes can start anywhere.

How it works: `Threaded` is a `kit.Mechanism` with two links in a chain, an axial slide ("<cap>_lift", no geometry,
tiny mass) and the cap on a hinge on top of it. yamkit spawns it, puts its joints back at their start on every reset
and reads the links' states; this module rewrites the cap joint's parent in the USD (yamkit joins every link to the
base) and, after yamkit's per-step update, sets each thread's drives: the slide stiffly to pitch x turns, the hinge's
resistance, and the hard stop past the seat. The hinge angle is unwrapped step by step, so several turns count
correctly whatever range PhysX reports the joint in. Plain Python and NumPy at import time.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field

import numpy as np
import yamkit as kit
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

# A gripper holds a cap when its tool point is within HOLD_AXIS of the cap's axis and HOLD_ALONG of its grip point along
# the axis, and its fingers are open to the cap's width (opening = width / 94 mm) within HOLD_BAND: closed on the cap,
# not empty or open.
HOLD_AXIS = 0.015
HOLD_ALONG = 0.015
HOLD_BAND = 0.15  # fingers may squeeze up to 15 % of full opening narrower than the cap's width
HOLD_OVER = 0.03  # and be at most 3 % (2.8 mm) wider: an open hand hovering around the cap does not hold it
YAW_TOL = math.radians(3.0)  # (kept for reference; replaced by the roll budget below)
ROLL_GAIN = 1.25  # a held cap may be credited up to 1.25x the holding wrist's own roll (pads lag and catch up a little)
ROLL_BUDGET_MAX = math.radians(15.0)  # unused roll credit carried over between steps is capped at 15 degrees (one regrip
# stroke's lag), so a creeping wrist cannot bank credit for another arm's swipe
GRIPPERS = ("left_gripper", "right_gripper")
# Extra links (Threaded.extra) get a PhysX drive with this effort limit (N or N*m), so yamkit's `stay` and springs act.
EXTRA_EFFORT = 20.0


@dataclass(frozen=True)
class Thread:
    """A thread. `pitch` (m per turn) is how far the cap rises per turn loosened; `turns` is how many turns back from
    the seat the cap comes off; `start` (degrees past the seat, positive = tight) is where each trial begins;
    `max_tight` (degrees past the seat) is a hard stop. `resist` and `tight_resist` (N*m/rad) hold the cap where it was
    at the last control step, before and past the seat; `drag` (N*m*s/rad) opposes its speed. `hand` is +1 for a
    right-hand thread (loosens counterclockwise seen from the cap's top), -1 for left-hand. `free_lift` (m) is how far
    an unscrewed cap can be lifted along the axis; `lift_stiffness` (N/m) holds the cap's height to the thread, with at
    most `lift_force` (N); the hinge's drives give at most `max_torque` (N*m)."""

    pitch: float = 0.004
    turns: float = 1.25
    start: float = 20.0
    max_tight: float = 60.0
    resist: float = 0.05
    tight_resist: float = 0.15
    drag: float = 0.0005
    hand: int = 1
    free_lift: float = 0.03
    lift_stiffness: float = 3000.0
    lift_force: float = 50.0
    max_torque: float = 2.0


@dataclass(frozen=True)
class Cap:
    """The turning part: `name`, `anchor` (the axis point at the seat, in the body's frame), `mass`, and `meshes` and
    `colliders` in the cap's own frame (origin at the anchor, matching the body's frame at the seat). `grip` is the
    point a gripper should hold, in the cap's frame, and `width` (m) how wide the cap is where the fingers close on it
    (for `held`: fingers closed on nothing do not count)."""

    name: str
    anchor: tuple
    mass: float = 0.01
    meshes: tuple = ()
    colliders: tuple = ()
    grip: tuple = (0.0, 0.0, 0.0)
    width: float = 0.04


@dataclass(frozen=True)
class Threaded(kit.Mechanism):
    """A body (a `kit.Mechanism` base: meshes, colliders, mass or fixed) with a threaded `cap` turning about the
    body's `axis` ("X", "Y" or "Z", default "Z": up). Build it with `pos`, `size`, `mass`, `meshes`, `colliders`, `cap`
    and `thread`; `links` are filled in."""

    cap: Cap | None = None
    thread: Thread = field(default_factory=Thread)
    axis: str = "Z"
    extra: tuple = ()  # more kit.Links on the same base (e.g. a wire that slides); each gets a PhysX drive

    def __post_init__(self):
        if self.cap is None:
            raise ValueError(f"{self.name}: a Threaded part needs a cap")
        th, c = self.thread, self.cap
        u0 = math.radians(-th.start)  # unscrew angle at the start (rad); the joint's coordinate is hand * u
        lo, hi = math.radians(-th.max_tight), 2 * math.pi * th.turns + math.radians(30.0)
        jlo, jhi = (lo, hi) if th.hand > 0 else (-hi, -lo)
        lift = kit.Link(f"{c.name}_lift", "prismatic", self.axis, c.anchor,
                        lower=th.pitch * lo / (2 * math.pi) - 0.002, upper=th.pitch * th.turns + th.free_lift,
                        start=th.pitch * u0 / (2 * math.pi), mass=0.005, damping=5.0)
        cap = kit.Link(c.name, "revolute", self.axis, c.anchor, lower=jlo, upper=jhi, start=th.hand * u0,
                       mass=c.mass, meshes=c.meshes, colliders=c.colliders, damping=0.0005)
        object.__setattr__(self, "links", (lift, cap) + tuple(self.extra))


@dataclass
class _Track:
    episode: int
    u: float  # unscrew angle from the seat (rad), unwrapped
    q_last: float  # the hinge's last reported coordinate
    off: bool = False
    came_off: bool = False
    gripped: float = 0.0  # rad loosened while held, net
    u_acct: float = 0.0  # u at the last gripper accounting
    held: bool = False
    rise: float = 0.0
    holder: str | None = None
    yaw_last: dict = field(default_factory=dict)
    unheld_loosen: float = 0.0  # rad turned in the loosening direction without a gripper's credit
    unheld_tighten: float = 0.0  # rad turned in the tightening direction without a gripper's credit


@dataclass(frozen=True)
class TwistState:
    turns: float
    past_seat_deg: float
    rise: float
    off: bool
    came_off: bool
    held: bool
    gripped_deg: float


_TRACK: dict[str, _Track] = {}
_SPECS: dict[str, Threaded] = {}  # "body/cap" -> spec


def _key(spec: Threaded) -> str:
    return f"{spec.name}/{spec.cap.name}"


def _start_u(spec: Threaded) -> float:
    return math.radians(-spec.thread.start)


def _track(key: str) -> _Track:
    """The thread's record for this trial (fresh after a reset)."""
    t = _TRACK.get(key)
    if t is None or t.episode != kit.episode():
        spec = _SPECS[key]
        u0 = _start_u(spec)
        t = _TRACK[key] = _Track(kit.episode(), u0, spec.thread.hand * u0, u_acct=u0,
                                 rise=spec.thread.pitch * u0 / (2 * math.pi))
    return t


# -- USD: chain the cap onto its slide ------------------------------------------------------------


def _chain_usda(spec: Threaded, text: str) -> str:
    """yamkit joins every link to the base; make the cap's hinge sit on the slide instead, and give the slide's body
    (which has no shapes) a tiny explicit inertia."""
    lift, cap = spec.links[:2]
    head = f'def PhysicsRevoluteJoint "{cap.name}_joint"'
    i = text.index(head)
    j = text.index("}\n", i)
    block = text[i:j]
    block = block.replace("rel physics:body0 = </Root/base>", f"rel physics:body0 = </Root/{lift.name}>")
    block = block.replace(f"point3f physics:localPos0 = {kit._fmt(cap.anchor)}", "point3f physics:localPos0 = (0, 0, 0)")
    text = text[:i] + block + text[j:]
    body = f'def Xform "{lift.name}" ('
    k = text.index(body)
    m = text.index("float physics:mass", k)
    eol = text.index("\n", m) + 1
    pad = text[text.rindex("\n", 0, m) + 1:m]
    text = text[:eol] + f"{pad}float3 physics:diagonalInertia = (1e-06, 1e-06, 1e-06)\n" \
                        f"{pad}point3f physics:centerOfMass = (0, 0, 0)\n" + text[eol:]
    # Give both joints a PhysX drive (yamkit's joints have none, and PhysX then ignores stiffness and targets).
    drives = [(f"{lift.name}_joint", "linear", spec.thread.lift_force),
              (f"{cap.name}_joint", "angular", spec.thread.max_torque)]
    drives += [(f"{lk.name}_joint", "linear" if lk.kind == "prismatic" else "angular", EXTRA_EFFORT)
               for lk in spec.links[2:]]
    for jname, kind, force in drives:
        a = text.index(f'"{jname}" (')
        b = text.index(")\n", a)
        text = text[:a] + text[a:b].replace('"PhysxJointAPI"', f'"PhysxJointAPI", "PhysicsDriveAPI:{kind}"', 1) + text[b:]
        c = text.index("{\n", a) + 2
        text = (text[:c] + f'        uniform token drive:{kind}:physics:type = "force"\n'
                f"        float drive:{kind}:physics:maxForce = {force}\n"
                f"        float drive:{kind}:physics:stiffness = 0\n"
                f"        float drive:{kind}:physics:damping = 0\n"
                f"        float drive:{kind}:physics:targetPosition = 0\n" + text[c:])
    if spec.mass is not None:
        # A free-floating articulation's root is the most central body unless one is named: here that would be the
        # slide, and the scene would place the slide (not the body) at `pos`. Name the body as the root instead.
        root_api = 'def Xform "Root" (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)\n{\n'
        settings = ("    bool physxArticulation:enabledSelfCollisions = 0\n"
                    "    int physxArticulation:solverPositionIterationCount = 32\n"
                    "    int physxArticulation:solverVelocityIterationCount = 4\n")
        text = text.replace(root_api + settings, 'def Xform "Root"\n{\n', 1)
        old = '    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]\n    )\n    {\n'
        if old not in text:
            raise RuntimeError("twistkit: unexpected yamkit USD layout for the base")
        new = ('    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
               '"PhysxRigidBodyAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n    )\n    {\n'
               + settings.replace("    ", "        ", 1).replace("\n    ", "\n        "))
        text = text.replace(old, new, 1)
    return text


_orig_mech_usda = kit._mech_usda


def _mech_usda(mech):
    text = _orig_mech_usda(mech)
    return _chain_usda(mech, text) if isinstance(mech, Threaded) else text


kit._mech_usda = _mech_usda


# -- per step: the thread's drives ------------------------------------------------------------------


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _drive_thread(base, spec: Threaded) -> None:
    import os

    art = base.scene[spec.name]
    np_ = lambda t: getattr(t, "torch", t).detach().cpu().numpy().astype(np.float64)  # noqa: E731
    q = np_(art.data.joint_pos)[0]
    jn = list(art.joint_names)
    lift, cap = spec.links[:2]
    jl, jc = jn.index(f"{lift.name}_joint"), jn.index(f"{cap.name}_joint")
    jx = [jn.index(f"{lk.name}_joint") for lk in spec.links[2:]]
    th, key = spec.thread, _key(spec)
    if not getattr(art, "_twistkit_limits", False):
        # yamkit's actuators leave these joints' effort limits at the USD default, 0 here, which silences every drive
        import torch

        dev = getattr(art.data.joint_pos, "torch", art.data.joint_pos).device
        art.write_joint_effort_limit_to_sim_index(
            limits=torch.tensor([[th.lift_force, th.max_torque] + [EXTRA_EFFORT] * len(jx)], dtype=torch.float32,
                                device=dev), joint_ids=[jl, jc] + jx)
        art._twistkit_limits = True
    t = _track(key)
    qc = float(q[jc])
    t.u += th.hand * _wrap(qc - t.q_last)  # unwrapped: counts whole turns whatever range PhysX reports in
    t.q_last = qc
    t.rise = float(q[jl])
    end = 2 * math.pi * th.turns
    t.off = t.u > end
    t.came_off = t.came_off or t.off
    lo = math.radians(-th.max_tight)
    # the slide: held to the thread, or (off the thread) held up where it was left, free to lift
    if t.off:
        kit._set_drive(art, jl, target=max(t.rise, th.pitch * th.turns), stiffness=200.0, effort=0.0)
    else:
        kit._set_drive(art, jl, target=th.pitch * t.u / (2 * math.pi), stiffness=th.lift_stiffness, effort=0.0)
    # the hinge: a hard stop past the seat; otherwise a hold where it was at the last step (more past the seat)
    if t.u < lo:
        kit._set_drive(art, jc, target=qc + th.hand * (lo - t.u), stiffness=5.0, effort=0.0)
    else:
        k = th.tight_resist if t.u < 0 else th.resist
        vel = float(np_(art.data.joint_vel)[0][jc])
        kit._set_drive(art, jc, target=qc, stiffness=k, effort=-th.drag * vel)


_orig_mech_states = kit._mech_states


def _mech_states(base, name, origin):
    out = _orig_mech_states(base, name, origin)
    spec = kit._MECHS.get(name)
    if isinstance(spec, Threaded):
        _drive_thread(base, spec)
    return out


kit._mech_states = _mech_states


def _grip_yaw(state) -> float:
    """The gripper's roll about the vertical: the angle of its finger-closing axis (local x) in the world xy-plane."""
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    cx, cy = 1 - 2 * (y * y + z * z), 2 * (x * y + w * z)
    return math.atan2(cy, cx)


def _account(objects) -> None:
    """After every step's states: whether a gripper holds each cap, and the turning done while it does.

    A step counts as held if a gripper held the cap at either end of it (the roll in the steps where the fingers land
    or open is the grip's own). A held step is credited only from a budget the holding gripper's own wrist roll earns
    (ROLL_GAIN x its roll, at most ROLL_BUDGET_MAX carried over), in the same direction: a cap turned by something else while a hand rests on it is not credited. Whatever
    is not credited is booked as unheld turning, kept separately for each direction (`unheld_deg`)."""
    for key, spec in _SPECS.items():
        if key not in objects:
            continue
        t = _track(key)
        yaws = {g: _grip_yaw(objects[g]) for g in GRIPPERS if g in objects}
        was, before = t.held, t.holder
        holder = _holder(objects, key)
        t.held, t.holder = holder is not None, holder
        du = t.u - t.u_acct
        credit = 0.0
        g = holder or before
        budget = getattr(t, "roll_budget", 0.0)  # signed, in cap-turn units: wrist roll not yet matched by the cap
        if (was or t.held) and g in yaws and g in t.yaw_last:
            turn = spec.thread.hand * _wrap(yaws[g] - t.yaw_last[g])  # the cap turn the wrist roll accounts for
            budget += ROLL_GAIN * turn  # signed: rolling back while held takes credit back (no wiggle laundering)
            if du * budget > 0:  # the cap turned the way the wrist's net roll this grip allows: credit it
                credit = math.copysign(min(abs(du), abs(budget)), du)
                budget -= credit
            budget = max(-ROLL_BUDGET_MAX, min(budget, ROLL_BUDGET_MAX))
        else:
            budget = 0.0  # not held: nothing carries over to the next grip
        t.roll_budget = budget
        t.gripped += credit
        rest = du - credit
        if rest > 0:
            t.unheld_loosen += rest
        else:
            t.unheld_tighten -= rest
        t.u_acct = t.u
        t.yaw_last = yaws


def _holder(objects, key):
    """The gripper whose fingers are closed on the cap now, or None (see _held_now)."""
    for g in GRIPPERS:
        if g in objects and _held_by(objects, key, g):
            return g
    return None


def _held_now(objects, key) -> bool:
    return _holder(objects, key) is not None


def _held_by(objects, key, g) -> bool:
    """Fingers closed on the cap: opening between the cap's width minus HOLD_BAND and plus HOLD_OVER (pads touching,
    not hovering open around it, and not an empty closed hand), the tool point within HOLD_AXIS of the axis, and the
    pads (tool point +5 mm to -13 mm along the approach) overlapping the cap's length by at least 3 mm."""
    from yam_arena.constants import FINGER_OPEN

    spec = _SPECS[key]
    s = objects[key]
    x, y, z, w = (float(v) for v in s.quat_xyzw)
    rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    axis = rot[:, "XYZ".index(spec.axis)]
    lo, hi = _cap_span(spec)
    want = spec.cap.width / (2 * abs(FINGER_OPEN))  # the opening with the pads on the cap
    d = np.asarray(objects[g].pos, float) - np.asarray(s.pos, float)
    along = float(np.dot(d, axis))
    across = float(np.linalg.norm(d - along * axis))
    op = kit.opening(objects, g)
    return (want - HOLD_BAND <= op <= want + HOLD_OVER and across < HOLD_AXIS
            and lo - 0.002 <= along <= hi + 0.010)


def _cap_span(spec):
    """The cap's extent along its axis, from its colliders (cap frame), else its grip point +-1 cm."""
    k = "XYZ".index(spec.axis)
    ends = []
    for c in spec.cap.colliders:
        h = c.size[k] / 2 if c.size is not None else (c.height / 2 if k == 2 else c.radius)
        ends += [c.center[k] - h, c.center[k] + h]
    if not ends:
        g = spec.cap.grip[k]
        return g - 0.01, g + 0.01
    return min(ends), max(ends)


_orig_patch = kit._patch


def _patch(scene) -> None:
    _orig_patch(scene)
    if getattr(scene, "_twistkit", False):
        return
    scene._twistkit = True
    inner_task_object, inner_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Threaded):
            _SPECS[_key(spec)] = spec
        return inner_task_object(spec)

    def object_states(env, names):
        out = inner_states(env, names)
        _account(out)
        return out

    scene.task_object, scene.object_states = task_object, object_states


kit._patch = _patch
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])


def register(*specs: Threaded) -> None:
    """Make threaded parts known to the scorer API without a simulator (offline tests). The simulator registers them
    itself when it builds the scene."""
    for s in specs:
        _SPECS[_key(s)] = s


# -- scorer API -----------------------------------------------------------------------------------


def turns(objects, key: str) -> float:
    """Turns backed off from the seat (negative: tightened past it)."""
    return _track(key).u / (2 * math.pi)


def past_seat_deg(objects, key: str) -> float:
    """Degrees past the seat in the tightening direction (positive = tight, negative = backed off)."""
    return -math.degrees(_track(key).u)


def backed_off_deg(objects, key: str) -> float:
    """Degrees loosened past the seat (negative = still tight past it)."""
    return math.degrees(_track(key).u)


def rise(objects, key: str) -> float:
    """The cap's travel along its axis from the seat (m)."""
    return _track(key).rise


def is_off(objects, key: str) -> bool:
    """The cap is off its thread now."""
    return _track(key).off


def came_off(objects, key: str) -> bool:
    """The cap has been off its thread at some point in this trial."""
    return _track(key).came_off


def held(objects, key: str) -> bool:
    """A gripper's fingers are closed on the cap (see the module docstring), as of the last step."""
    return _track(key).held


def gripped_deg(objects, key: str) -> float:
    """Degrees the cap was loosened (net) while a gripper held it, in this trial."""
    return math.degrees(_track(key).gripped)


def unheld_deg(objects, key: str, tightening: bool = True) -> float:
    """Degrees the cap turned in one direction (tightening by default) without a gripper's credit, this trial (not
    netted against the other direction, so swiping back and forth cannot launder it)."""
    t = _track(key)
    return math.degrees(t.unheld_tighten if tightening else t.unheld_loosen)


def state(objects, key: str) -> TwistState:
    t = _track(key)
    return TwistState(turns=t.u / (2 * math.pi), past_seat_deg=-math.degrees(t.u), rise=t.rise, off=t.off,
                      came_off=t.came_off, held=t.held, gripped_deg=math.degrees(t.gripped))


def set_angle(key: str, past_seat: float, held_now: bool | None = None) -> None:
    """Offline tests only: put a thread at `past_seat` degrees past the seat, as if turned there (the gripped total
    follows if `held_now`, else as before). Updates came_off."""
    spec, t = _SPECS[key], _track(key)
    u = math.radians(-past_seat)
    if held_now is not None:
        t.held = held_now
    if t.held:
        t.gripped += u - t.u
    elif u > t.u:
        t.unheld_loosen += u - t.u
    else:
        t.unheld_tighten += t.u - u
    t.u = t.u_acct = u
    t.rise = spec.thread.pitch * u / (2 * math.pi)
    t.off = u > 2 * math.pi * spec.thread.turns
    t.came_off = t.came_off or t.off


# -- demo helpers -----------------------------------------------------------------------------------


def yaw_range(side: str, xyz, step: float = 5.0, tol_mm: float = 1.0, tol_deg: float = 1.0) -> tuple[float, float]:
    """The range of straight-down grip yaw (degrees) the arm on `side` ("left" or "right") reaches at `xyz` (world,
    m) within its joint limits, by IK from the ready pose. Plain NumPy."""
    from yam_arena import demo, ik
    from yam_arena.constants import LEFT_BASE_POS, READY, RIGHT_BASE_POS

    base = np.asarray(LEFT_BASE_POS if side.startswith("left") else RIGHT_BASE_POS, float)  # as yam_scripted uses
    ok = []
    q = np.array(READY, float)
    for yd in np.arange(-240.0, 240.0 + 1e-9, step):
        qq, pe, re = ik.solve(np.asarray(xyz, float), ik.matrix_to_quat_xyzw(demo.grip_rotation(math.radians(yd))),
                              q, base_pos=base, iters=200)
        if pe * 1000 < tol_mm and math.degrees(re) < tol_deg:
            ok.append(yd)
    if not ok:
        raise ValueError(f"no straight-down grip reaches {xyz} from the {side} arm")
    # the longest contiguous run
    runs, cur = [], [ok[0]]
    for a, b in zip(ok, ok[1:]):
        if b - a > step + 1e-6:
            runs.append(cur)
            cur = []
        cur.append(b)
    runs.append(cur)
    best = max(runs, key=len)
    return float(best[0]), float(best[-1])


def twist_waypoints(xy, z_grip: float, degrees: float, *, side: str | None = None, start_yaw: float | None = None,
                    stroke: float = 90.0, step: float = 15.0, pitch: float = 0.004, hand: int = 1,
                    clear: float = 0.05, step_s: float = 0.3, grip_s: float = 0.7, approach_s: float = 1.2,
                    label: str = "twist") -> list[Waypoint]:
    """A regrasp loop that turns a cap at `xy` by `degrees` (positive = loosen) with a straight-down grip whose tool
    point is at height `z_grip`: approach `clear` above with the fingers open, descend, close, roll `stroke` degrees in
    `step`-degree waypoints (each `step_s` seconds, rising with the thread by `pitch` per turn), open, roll back with
    the fingers open, close again, and so on; finally open and lift `clear` back up. `hand` is the thread's hand
    (loosening turns the grip counterclockwise seen from above for +1). With `side` given, every stroke is checked
    against that arm's reachable yaw range (`yaw_range`) and `start_yaw` defaults to one end of it."""
    sgn = 1 if degrees >= 0 else -1
    turn = sgn * hand  # +1: the grip's yaw increases (counterclockwise seen from above)
    if side is not None:
        lo, hi = yaw_range(side, (xy[0], xy[1], z_grip))
        need = min(stroke, abs(degrees))
        if hi - lo < need + 10:
            raise ValueError(f"the {side} arm rolls only {hi - lo:.0f} degrees at {xy}; use a shorter stroke")
        if start_yaw is None:
            start_yaw = lo + 15 if turn > 0 else hi - 15
        end = start_yaw + turn * need
        if not (lo <= start_yaw <= hi and lo <= end <= hi):
            raise ValueError(f"stroke {start_yaw:.0f} -> {end:.0f} degrees is outside the {side} arm's {lo:.0f} to "
                             f"{hi:.0f} at {xy}")
    y0 = math.radians(start_yaw or 0.0)
    x, y = xy
    out = [Waypoint(approach_s, (x, y, z_grip + clear), FINGER_OPEN, f"{label}: over the cap", y0),
           Waypoint(approach_s, (x, y, z_grip), FINGER_OPEN, f"{label}: around the cap", y0)]
    done, z = 0.0, z_grip
    left = abs(degrees)
    while left > 1e-6:
        s = min(stroke, left)
        out.append(Waypoint(grip_s, (x, y, z), FINGER_CLOSED, f"{label}: grip", y0))
        n = max(1, math.ceil(s / step))
        for k in range(1, n + 1):
            a = s * k / n
            zz = z + sgn * pitch * a / 360.0
            out.append(Waypoint(step_s, (x, y, zz), FINGER_CLOSED, f"{label}: roll", y0 + turn * math.radians(a)))
        z = z + sgn * pitch * s / 360.0
        done += s
        left -= s
        yend = y0 + turn * math.radians(s)
        out.append(Waypoint(grip_s, (x, y, z), FINGER_OPEN, f"{label}: let go", yend))
        if left > 1e-6:  # roll back with the fingers open, in steps under 180 degrees so the slerp goes the right way
            for k in range(1, n + 1):
                out.append(Waypoint(step_s * 0.6, (x, y, z), FINGER_OPEN, f"{label}: roll back",
                                    yend - turn * math.radians(s * k / n)))
    out.append(Waypoint(approach_s, (x, y, z + clear), FINGER_OPEN, f"{label}: lift clear", yend))
    return out
