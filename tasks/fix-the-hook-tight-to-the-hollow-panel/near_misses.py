"""Near-successes and unintended strategies, scored offline by the factory checker. Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state from the part's own geometry (HOOK.state(...)). Cases run
in order and share the scorer's state (the 1 s count and the tug test's progress).

Offline there is no robot and no physics, so the arms' clearance is set here as measured clear (in the simulator it is
measured every step from the hands' face-sampled hulls, and is None, no credit, until then), and the tug test's
response is given as the state the hook takes while it is pulled: a clamped hook barely moves, a loose one comes off
the wall until its wings meet the back of the board.
"""

import math

import numpy as np

import parts
from task import (DRIVER, FACE_X, GAP0, GAUGE, HOLE_Y, HOOK, HOOK_POS, LEAD, MOVE_MAX, OUT, PLATE, PLATE_Y, PULL_N,
                  PULL_STEPS, SETTLE_STEPS, STILL_STEPS, WALL_ROT)

parts.__dict__.setdefault("_hook_task_grip", {})["clear"] = True
# offline: the robot is taken to have handled the hook already (in the simulator a gripper must have come within 5 mm)
parts.__dict__.setdefault("_hook_task_record", {"still": 0})["handled"] = True
# Offline stand-in for the simulator's force: PULL_N is reported as applied on every pull step, except while the
# hook is in a state marked by no_wrench() (the pull failed to act).
_NO_WRENCH = set()
parts.__dict__.setdefault("_hook_task_pull", {"phase": "idle"})["offline_applied"] = (
    lambda objects: 0.0 if id(objects["hook"]) in _NO_WRENCH else PULL_N)

P0 = np.asarray(HOOK_POS, float)
WALL_ACROSS = WALL_ROT[:, 0]  # world +y


def turn(axis, deg) -> np.ndarray:
    return parts._rot(axis, math.radians(deg))


def hook(thread=GAP0, out=0.0, plumb=0.0, extra=None, pos=None, vel=None):
    """The hook `out` m off the face, turned `plumb` degrees about the wall's normal (off the plumb line), or by
    `extra` (a world rotation applied about the bolt)."""
    r = (extra if extra is not None else turn(OUT, plumb)) @ WALL_ROT
    p = np.asarray(pos, float) if pos is not None else P0 + out * OUT
    return {"hook": HOOK.state(pos=tuple(p), quat_xyzw=tuple(parts._matrix_to_quat(r)), lin_vel=vel,
                               joints={"thread": thread, "nut": thread / LEAD * 2 * math.pi})}


def rocked(gap, thread=GAP0):
    """The plate's bottom (hook) end `gap` off the face, pivoting on its top edge (as a tug at the hook rocks it)."""
    a = math.degrees(math.atan2(gap, PLATE_Y[1] - PLATE_Y[0]))
    r = turn(WALL_ACROSS, a)  # positive: the lower end swings out toward the robot (world -x)
    pivot = P0 + WALL_ROT @ np.array([0.0, PLATE_Y[0], 0.0])
    if np.dot(r @ (WALL_ROT @ np.array([0.0, PLATE_Y[1] - PLATE_Y[0], 0.0])), OUT) < 0:
        r = turn(WALL_ACROSS, -a)
    pos = pivot + r @ (P0 - pivot)
    return hook(thread=thread, extra=r, pos=pos)


def no_wrench(state):
    _NO_WRENCH.add(id(state["hook"]))
    return state


TIGHT = hook()
GIVE = 0.0006  # a tight hook's plate edge under the 20 N tug, on the springy board (0.28 to 0.63 mm measured so far)
TIGHT_PULLED = rocked(GIVE)


def tested(label, state, score, pulled=None, success=None, after=None):
    """A state reached, held still for 1 s, then put through the tug test: PULL_STEPS steps pulled (the hook in
    `pulled`; default: a tight hook's give), then SETTLE_STEPS steps let off (the hook in `after`, default `state`),
    after which the score is given. success() sees the result one step later. Each case starts from the hook moving
    (which also restarts the test)."""
    pulled = TIGHT_PULLED if pulled is None else pulled
    after = state if after is None else after
    out = [(f"{label}: hook still moving", {**state, **hook(vel=(0.0, 0.0, -0.02))}, 0.0, False)]
    out += [(f"{label} (step {k + 1} of the 1 s hold)", state, 0.0, False) for k in range(STILL_STEPS - 1)]
    out.append((f"{label} (held 1 s: the tug test starts, no credit yet)", state, 0.0, False))
    out += [(f"{label} (pulled, step {k + 1})", pulled, 0.0, False) for k in range(PULL_STEPS)]
    out += [(f"{label} (pull off, step {k + 1})", after, 0.0, False) for k in range(SETTLE_STEPS - 1)]
    out.append((f"{label} (tug test done)", after, score, False))
    out.append((f"{label} (tug test done + 1 step)", after, score, score == 1.0 if success is None else success))
    return out


G = GAUGE * 1000
CASES = [
    ("start: hook loose, wings 1.5 mm behind the back of the board", {}, 0.0, False),
    *tested("clamped tight and plumb, hook end gives 0.6 mm under the tug", TIGHT, 1.0),
    *tested("clamped, hook end gives 0.95 mm under the tug", TIGHT, 1.0, pulled=rocked(0.00095)),
    # fail closed: the pull must really have been applied
    *tested("clamped, but the tug was never applied (force not set)", TIGHT, 0.0, pulled=no_wrench(rocked(GIVE))),
    ("then the hook moving: the hold and the test start again", hook(vel=(0.0, 0.0, 0.02)), 0.0, False),
    ("tight and at rest again, first step", TIGHT, 0.0, False),
    # square to the plumb line
    *tested("clamped, 2.9 degrees off plumb", hook(plumb=2.9), 1.0),
    *tested("clamped, 3.1 degrees off plumb", hook(plumb=-3.1), 0.5),
    *tested("clamped, 10 degrees off plumb", hook(plumb=10.0), 0.5),
    *tested("clamped, 10.5 degrees off plumb (no test)", hook(plumb=10.5), 0.0),
    *tested("clamped but upside down (hook end above the bolt)", hook(plumb=180.0), 0.0),
    # the tug: no 1 mm gap may open at any edge
    *tested(f"clamped, plate comes {G - 0.01:.2f} mm straight off the wall under the tug", TIGHT, 1.0,
            pulled=hook(out=GAUGE - 0.00001)),
    *tested(f"plate comes {G:.1f} mm off the wall under the tug (a 1 mm gauge goes in)", TIGHT, 0.0,
            pulled=hook(out=GAUGE)),
    *tested(f"hook end rocks {G - 0.01:.2f} mm off the wall under the tug", TIGHT, 1.0, pulled=rocked(GAUGE - 0.00001)),
    *tested(f"hook end rocks {G:.1f} mm off the wall under the tug", TIGHT, 0.0, pulled=rocked(GAUGE)),
    # loose hooks resting flat: under the tug they come off the wall until the wings meet the back of the board
    *tested("resting flat, wings 0.5 mm off the back of the board (unclamped)", hook(thread=GAP0 - 0.0005), 0.0,
            pulled=rocked(3 * 0.0005 + GIVE, thread=GAP0 - 0.0005)),
    *tested("resting flat, half the thread taken up", hook(thread=GAP0 / 2), 0.0,
            pulled=rocked(3 * GAP0 / 2 + GIVE, thread=GAP0 / 2)),  # the tug at the arm rocks it out (x3, as above)
    *tested("turned without the pull: toggle spun, no travel", hook(thread=0.0), 0.0,
            pulled=hook(thread=0.0, out=GAP0)),
    # after the tug the plate must be where it was (pencil ticks at its ends)
    *tested(f"clamped, plate slips {MOVE_MAX * 1000 - 0.01:.2f} mm down under the tug", TIGHT, 1.0,
            after=hook(pos=P0 + np.array([0.0, 0.0, -(MOVE_MAX - 0.00001)]))),
    *tested(f"plate slips {MOVE_MAX * 1000 + 0.1:.1f} mm down under the tug", TIGHT, 0.0,
            after=hook(pos=P0 + np.array([0.0, 0.0, -(MOVE_MAX + 0.0001)]))),
    # flush on the wall (the feeler gauge), with no tug
    *tested(f"clamped, plate {G - 0.1:.1f} mm off the face", hook(out=GAUGE - 0.0001), 1.0,
            pulled=hook(out=GAUGE - 0.0001 + 0.00005)),
    *tested(f"plate {G + 0.1:.1f} mm off the face (no test)", hook(out=GAUGE + 0.0001), 0.0),
    *tested("plate's hook end 1.2 mm off the face (rocked, no test)", rocked(0.0012), 0.0),
    # let go: the screwdriver
    *tested("clamped, nut driver left on the head",
            {**TIGHT, "screwdriver": DRIVER.state(pos=(FACE_X - PLATE[2] - 0.0006, HOLE_Y, P0[2]),
                                                  quat_xyzw=tuple(parts._matrix_to_quat(turn((0, 1, 0), -90.0))))},
            0.0),
    *tested("clamped, nut driver held 4 mm in front of the head",
            {**TIGHT, "screwdriver": DRIVER.state(pos=(FACE_X - PLATE[2] - 0.0036 - 0.004, HOLE_Y, P0[2]),
                                                  quat_xyzw=tuple(parts._matrix_to_quat(turn((0, 1, 0), -90.0))))},
            0.0),
    *tested("clamped, nut driver back in its holder", TIGHT, 1.0),
    ("then the hook pulled 30 mm off the wall", hook(thread=0.0, out=0.030), 0.0, False),
    *tested("then back, tight and plumb", TIGHT, 1.0),
]


# --- the release check measures hull faces, not only hull vertices ---------------------------------------------------
def _face_check():
    """A gripper whose broad face lies 0.5 mm from the plate's side edge, with every hull vertex more than 5 mm from
    the hook, must count as not let go. Offline geometry only (the simulator uses the same functions every step)."""
    from yam_arena import analysis

    import task

    hulls = analysis.link_hulls()["gripper"]
    h = max(hulls, key=lambda h: len(task._face_points(h)))
    best = None
    for n, d in zip(h.planes[:, :3], h.planes[:, 3]):  # the face with the most sampled points (the broadest)
        on = h.vertices[np.abs(h.vertices @ n - d) < 1e-6]
        if len(on) >= 3 and (best is None or np.ptp(on, axis=0).max() > best[2]):
            best = (n, on, np.ptp(on, axis=0).max())
    n, on, _ = best
    c = on.mean(axis=0)
    state = TIGHT
    plate_edge_y = HOLE_Y + PLATE[0] / 2  # the plate's left edge (world y)
    target = np.array([FACE_X - PLATE[2] / 2, plate_edge_y + 0.0005, P0[2] + 0.040])
    want_n = np.array([0.0, -1.0, 0.0])  # the face looks at the plate's edge
    results = []
    for spin in np.radians(np.arange(0, 180, 15)):
        v = np.cross(n, want_n)
        ang = math.atan2(np.linalg.norm(v), float(np.dot(n, want_n)))
        r0 = parts._rot(v, ang) if np.linalg.norm(v) > 1e-9 else (np.eye(3) if np.dot(n, want_n) > 0
                                                                   else parts._rot((1, 0, 0), math.pi))
        r = parts._rot(want_n, spin) @ r0
        p = target - r @ c
        verts = h.vertices @ r.T + p
        vgap = min(analysis.box_signed_distance(verts, cc, rr, ss).min() for _, cc, rr, ss in task._hook_boxes(
            state["hook"]))
        pts = task._face_points(h) @ r.T + p
        results.append((vgap, pts))
    vgap, pts = max(results, key=lambda x: x[0])
    fgap = task.hand_gap(state, pts)
    assert vgap > task.CLEAR, f"face check setup: a vertex is {vgap * 1000:.1f} mm from the hook"
    assert fgap < task.CLEAR and abs(fgap - 0.0005) < 0.0003, f"face-sampled gap {fgap * 1000:.2f} mm, want 0.5 mm"
    return vgap, fgap


FACE_CHECK = _face_check()  # (closest hull vertex, closest face point), m: e.g. (0.0x, 0.0005)
print(f"release check: gripper face 0.5 mm from the plate, nearest hull vertex {FACE_CHECK[0] * 1000:.1f} mm away: "
      f"measured {FACE_CHECK[1] * 1000:.2f} mm (not let go)")


def _handled_check():
    """Before the robot has handled the hook (no gripper within 5 mm yet) the scorer never tugs it, so even a tight,
    flush, plumb hook scores 0; the scorer's state is put back afterwards."""
    import task

    rec, pull = task._record, task._pull
    saved_rec, saved_pull = dict(rec), dict(pull)
    rec.update(still=0, handled=False)
    pull.update(phase="idle")
    st = {**{o.name: o.state() for o in task.OBJECTS if hasattr(o, "state")}, **TIGHT}
    best = max(task.score(st) for _ in range(3 * (STILL_STEPS + PULL_STEPS + SETTLE_STEPS)))
    phase = pull["phase"]
    rec.clear(); rec.update(saved_rec)
    pull.clear(); pull.update(saved_pull)
    assert best == 0.0 and phase == "idle", f"an unhandled hook was tested (score {best}, phase {phase})"


_handled_check()
