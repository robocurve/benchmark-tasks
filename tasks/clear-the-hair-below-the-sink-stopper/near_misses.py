"""Near misses and unintended strategies, each played as a short timeline through the scorer's own step record.

The scorer keeps a per-trial record that only the simulator's step and reset hooks write: whether the tool's hooked
pad has stayed in the clump long enough to take it, whether the tool ever dropped off the vanity, whether both grippers
are clear, and how long everything has been still. Each case resets that record, plays a timeline of states at 20 Hz
through every loaded copy of the task's step function, and leaves the timeline's last state for check.py to score.
Run alone: python near_misses.py
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import task  # noqa: E402
import lots  # noqa: E402
import parts  # noqa: E402

DT = 0.05
MM = 1e-3
TOP, ZF, CT = task.TOP, task.ZF, task.CT
FAR = [np.array([[0.20, 0.30, TOP + 0.45]])]  # the hands parked well away
DOWN = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))  # tool frame x pointing straight down (a quarter turn about y)


def _q(R):
    return tuple(parts._matrix_to_quat(R))


def lying(lot="A", dx=0.0, dy=0.0, dz=0.0, joints=None):
    """The tool lying as the lot starts it, moved by (dx, dy, dz)."""
    p = np.asarray(task.LOTS[lots.index(lot)].pos(task.TOOL.name), float)
    p[2] = task.REST_Z
    p = p + [dx, dy, dz]
    return task.TOOL.state(pos=p, quat_xyzw=task.LYING, joints=joints)


def upright(pad_depth, r=0.011):
    """The tool held straight up and down over the drain, its pad's centre ``pad_depth`` below the flange's top and
    ``r`` from the drain's axis on the wand's side (only the pad's position matters to the scorer)."""
    side = -np.array([math.cos(task.PSI), math.sin(task.PSI), 0.0])
    pad = np.array([task.BX, task.BY, TOP + ZF - pad_depth]) + side * r
    origin = pad + np.array([0.0, 0.0, task.WAND_L - task.PAD_LINKS * task.LINK_L / 2])
    return task.TOOL.state(pos=origin, quat_xyzw=DOWN)


def hair_on(tool):
    """The clump where the declared rule carries it once caught: on the pad."""
    lp, lq = tool.links[task.LAST]
    p = np.asarray(lp, float) + task._R(lq) @ task.HAIR_ON_PAD
    return task.HAIR_PART.state(pos=p, quat_xyzw=lq)


def in_drain():
    return task.HAIR_PART.state()


def scene(tool, hair=None, vel=None):
    if vel is not None:
        tool = task.TOOL.state(pos=tool.pos, quat_xyzw=tool.quat_xyzw, lin_vel=vel)
    return {task.TOOL.name: tool, task.HAIR_PART.name: hair if hair is not None else in_drain()}


def _instances():
    """Every loaded copy of task.py (check.py loads its own), found through the step hooks they registered."""
    seen = {}
    for fn in parts._ON_STEP:
        g = fn.__globals__
        if "observe" in g and "REC" in g:
            seen[id(g["REC"])] = g
    return list(seen.values())


def play(*segments):
    """Reset every copy's trial record, then play (objects, hands, seconds) segments at 20 Hz."""
    t = 0.0
    for g in _instances():
        g["REC"].clear()
        g["REC"].update(g["_fresh"]())
    last = None
    for objects, hands, seconds in segments:
        n = max(1, round(seconds / DT))
        for _ in range(n):
            t += DT
            for g in _instances():
                g["observe"](objects, hands, t)
            last = objects
    return last if last is not None else scene(lying())


def near_tool(objects, gap):
    """One gripper point ``gap`` above the top of the tool's handle grip."""
    tool = objects[task.TOOL.name]
    p = np.asarray(tool.pos, float) + task._R(tool.quat_xyzw) @ np.array([task.GRIP_AT, 0.0, 0.0])
    return FAR + [np.array([[p[0], p[1], p[2] + task.GRIP_R + gap]])]


IN = upright(0.035)  # pad 35 mm down, in the clump


def caught_then(final_tool, hold=1.2, dwell=0.6, depth=0.035, pull=0.012, hair=None, r=0.011):
    """Pad ``depth`` below the flange's top in the clump for ``dwell`` s, drawn back up ``pull`` in the bore, then the
    tool and the clump left as ``final_tool`` for ``hold`` s."""
    end = scene(final_tool, hair if hair is not None else hair_on(final_tool))
    hand = [np.array([[0.30, 0.20, TOP + 0.30]])]
    return play((scene(upright(depth, r)), hand, dwell), (scene(upright(depth - pull, r)), hand, 0.2), (end, FAR, hold))


def _along(direction, origin, joints=None):
    """The tool with its axis along ``direction`` (wand end first), its hinges' axis level, wand junction at ``origin``."""
    x = np.asarray(direction, float) / np.linalg.norm(direction)
    y = np.cross([0.0, 0.0, 1.0], x)
    y /= np.linalg.norm(y)
    return task.TOOL.state(pos=origin, quat_xyzw=_q(np.column_stack([x, y, np.cross(x, y)])), joints=joints)


def in_basin():
    """The tool lying in the basin, its handle end up the bowl's left side, the wand's end curled up the right side."""
    return _along((0.0, -1.0, -0.25), (task.BX, task.BY + 0.090, TOP + ZF + 0.030),
                  joints={f"w{i}": -0.30 for i in range(task.N_LINKS)})


def across_rim():
    """The wand's end and the hair on the countertop's front strip, the handle down in the basin."""
    end = np.array([task.CTR_FRONT + 0.040, task.BY - 0.020, TOP + CT + 0.016])
    d = np.array([-0.30, -0.05, 0.40])  # from the handle down in the bowl up toward the front strip
    d /= np.linalg.norm(d)
    return _along(d, end - d * task.WAND_L)


class _Timelines:
    """check.py iterates CASES in order; each case's timeline is played just before check.py scores it."""

    def __init__(self, cases):
        self.cases = cases

    def __iter__(self):
        for label, build, want, ok in self.cases:
            lots.use("B" if label.startswith("lot B") else "A")
            yield label, build(), want, ok
        lots.use(None)


def hair_at(z_above_flange, r=0.010):
    """A caught clump left ``r`` from the drain's axis, its centre this far above the flange's top."""
    return task.HAIR_PART.state(pos=(task.BX - r, task.BY, TOP + ZF + z_above_flange))


def moved_after(objects, tool_by=(0.0, 0.0, 0.0), hair_by=(0.0, 0.0, 0.0)):
    """The state the step scores: the simulator moves the caught clump onto the pad after each observation (the
    declared rule), so the scored hair can sit a little off the observed one; the tool is never moved in between."""
    tool, hair = objects[task.TOOL.name], objects[task.HAIR_PART.name]
    tool = task.TOOL.state(pos=np.asarray(tool.pos) + tool_by, quat_xyzw=tool.quat_xyzw, joints=tool.joints)
    hair = task.HAIR_PART.state(pos=np.asarray(hair.pos) + hair_by, quat_xyzw=hair.quat_xyzw)
    return {task.TOOL.name: tool, task.HAIR_PART.name: hair}


SPEC = [
    ("no history: nothing observed yet", lambda: play() or scene(lying()), 0.0, False),
    ("arms idle: tool on the counter, hair in the drain", lambda: play((scene(lying()), FAR, 2.0)), 0.0, False),
    ("full: pad in the clump 0.6 s, tool laid back on the counter with the hair, hands clear, still 1.2 s",
     lambda: caught_then(lying()), 1.0, True),
    ("full but still for only 0.85 s", lambda: caught_then(lying(), hold=0.85), 0.0, False),
    ("pad in the clump 10 control steps (0.45 s counted): the hooks do not take it",
     lambda: caught_then(lying(), dwell=0.50, hair=in_drain()), 0.0, False),
    ("pad in the clump 11 control steps (0.50 s counted, limit)", lambda: caught_then(lying(), dwell=0.55), 1.0, True),
    ("pad 21 mm below the flange's top, above the clump: nothing caught",
     lambda: caught_then(lying(), depth=0.021, hair=in_drain()), 0.0, False),
    ("pad 22 mm below the flange's top (limit): caught", lambda: caught_then(lying(), depth=0.022), 1.0, True),
    ("pad in the clump 0.6 s, then drawn up only 9 mm before leaving the bore sideways: not taken",
     lambda: caught_then(lying(), pull=0.009, hair=in_drain()), 0.0, False),
    ("pad in the clump 0.6 s, then drawn up 10 mm in the bore (limit): taken", lambda: caught_then(lying(), pull=0.010),
     1.0, True),
    ("pad outside the drain's bore, beside the flange: nothing caught",
     lambda: caught_then(lying(), r=0.020, hair=in_drain()), 0.0, False),
    ("naive: tool never taken to the drain, laid back where it was", lambda: play((scene(lying(dz=0.03)), FAR, 1.0),
                                                                                   (scene(lying()), FAR, 1.2)), 0.0, False),
    ("caught, tool and hair left lying in the basin", lambda: caught_then(in_basin()), 0.25, False),
    ("caught, clump left over the drain's mouth 9 mm above the flange's top: still in the drain",
     lambda: caught_then(in_basin(), hair=hair_at(0.009)), 0.0, False),
    ("caught, clump over the drain's mouth 10 mm above the flange's top (limit)",
     lambda: caught_then(in_basin(), hair=hair_at(0.010)), 0.25, False),
    ("caught, clump shaken off onto the basin's bottom 28 mm from the drain's axis, beside the flange",
     lambda: caught_then(in_basin(), hair=hair_at(-0.002, r=0.028)), 0.25, False),
    ("caught, hair on the counter but the tool's handle in the basin", lambda: caught_then(across_rim()), 0.5, False),
    ("caught, tool laid 65 mm further back: its wand over the basin's front edge, the end and the hair hanging in",
     lambda: caught_then(lying(dx=0.065, joints={f"w{i}": 1.0 for i in (5, 6, 7)})), 0.25, False),
    ("caught and laid on the counter, still held: a fingertip 9 mm above the grip",
     lambda: play((scene(IN), FAR, 0.6), (scene(upright(0.020)), FAR, 0.2),
                  (scene(lying(), hair_on(lying())), near_tool(scene(lying()), 0.009), 2.0)), 0.0, False),
    ("caught and laid on the counter, a fingertip 14 mm above the grip",
     lambda: play((scene(IN), FAR, 0.6), (scene(upright(0.020)), FAR, 0.2),
                  (scene(lying(), hair_on(lying())), near_tool(scene(lying()), 0.014), 1.2)), 1.0, True),
    ("1.0 earned, then a fingertip comes back to the tool",
     lambda: play((scene(IN), FAR, 0.6), (scene(upright(0.020)), FAR, 0.2), (scene(lying(), hair_on(lying())), FAR, 1.5),
                  (scene(lying(), hair_on(lying())), near_tool(scene(lying()), 0.005), 0.5)), 0.0, False),
    ("hand state unknown at the end", lambda: play((scene(IN), FAR, 0.6), (scene(upright(0.020)), FAR, 0.2),
                                                  (scene(lying(), hair_on(lying())), FAR, 1.5),
                                                  (scene(lying(), hair_on(lying())), None, 0.1)), 0.0, False),
    ("tool still sliding at 1 cm/s", lambda: caught_then(task.TOOL.state(pos=lying().pos, quat_xyzw=task.LYING,
                                                                       lin_vel=(0.01, 0.0, 0.0))), 0.0, False),
    ("tool dropped off the vanity onto the floor earlier, then a full end state",
     lambda: play((scene(lying(dz=-(CT - 0.020) - 0.002)), FAR, 0.2), (scene(IN), FAR, 0.6), (scene(upright(0.020)), FAR, 0.2),
                  (scene(lying(), hair_on(lying())), FAR, 1.2)), 0.0, False),
    ("tool's handle dipped 19 mm below the countertop beside the vanity, then a full end state (limit)",
     lambda: play((scene(lying(dz=-0.019)), FAR, 0.2), (scene(IN), FAR, 0.6), (scene(upright(0.020)), FAR, 0.2),
                  (scene(lying(), hair_on(lying())), FAR, 1.2)), 1.0, True),
    ("full, the clump moved 0.2 mm onto the pad after the last observation (the simulator's step order)",
     lambda: moved_after(caught_then(lying()), hair_by=(0.0002, 0.0, 0.0)), 1.0, True),
    ("full, but the tool scored is not the state last observed (moved 0.1 mm): invalid",
     lambda: moved_after(caught_then(lying()), tool_by=(0.0001, 0.0, 0.0)), 0.0, False),
    ("lot B: full in lot B's layout", lambda: caught_then(lying("B")), 1.0, True),
    ("lot B: tool laid back in lot A's place", lambda: caught_then(lying("A")), 1.0, True),
]

CASES = _Timelines(SPEC)


def run():
    bad = 0
    for label, objects, want, ok in CASES:
        got = task.score(objects)
        good = abs(got - want) < 1e-9 and task.success(objects) == ok
        bad += not good
        print("ok  " if good else "FAIL", f"{got:.2f} (want {want:.2f})", label)
    return bad


if __name__ == "__main__":
    raise SystemExit(run())
