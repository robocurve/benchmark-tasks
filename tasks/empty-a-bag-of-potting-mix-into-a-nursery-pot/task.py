"""Empty an opened bag of potting mix into a nursery pot with both hands. See README.md and DESIGN.md."""

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bagkit
import bagplan as p
import bodykit
import fastik
import scoring
import soilkit
from bodykit import Body, BoxShape
from scoring import Tally
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

SCENE = "station"
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
bodykit.allow_matrix_grips()

SPILL_FLOOR = Body.build("spill_floor", pos=(0, 0, -0.275), mass=1.0, kinematic=True,
                         shapes=[BoxShape((0, 0, 0), (4.0, 4.0, 0.5), (0.45, 0.45, 0.46))])
POT = bagkit.pot_body()
OBJECTS = [*bagkit.TAB_BODIES, POT, SPILL_FLOOR]

MIX = soilkit.soil(  # the settled fill comes from mix_start.npz (prepare_bag.py); the lattice is only a fallback
    "mix",
    host=bagkit.TAB_BODIES[0].name,
    origin=tuple(p.BAG_AT),
    low=(-0.035, -0.055, 0.005),
    high=(0.035, 0.055, 0.30),
    r_min=p.GRAIN_R[0],
    r_max=p.GRAIN_R[1],
    grain_shape="icosahedron",
    density=p.GRAIN_DENSITY,
    friction=p.MIX_FRICTION,
    angular_damping=0.5,
    colors=((0.17, 0.11, 0.07), (0.22, 0.14, 0.09), (0.13, 0.09, 0.06), (0.27, 0.18, 0.11), (0.86, 0.86, 0.83)),
    start_file=None if os.environ.get("MIX_FRESH") else os.environ.get("MIX_START", str(pathlib.Path(__file__).with_name("mix_start.npz"))),
)


_tally = Tally()
_trace = None
_step = 0


@bodykit.on_reset
def reset():
    global _tally, _trace, _step
    _tally, _step = Tally(), 0
    if _trace is not None:
        _trace.close()
    path = os.environ.get("BAG_TRACE")
    _trace = open(path, "w") if path else None  # noqa: SIM115 -- closed at the next reset


def finger_gaps(env) -> dict:
    out = {}
    for arm in ("left", "right"):
        robot = env.unwrapped.scene[arm + "_robot"]
        ids, _ = robot.find_joints(["joint7", "joint8"], preserve_order=True)
        out[arm] = float(-2.0 * bodykit.as_numpy(robot.data.joint_pos)[0][ids].mean())
    return out


def tab_states(env) -> list:
    """[(position, rotation, size)] of each tab (env frame)."""
    out = []
    for body in bagkit.TAB_BODIES:
        st = bodykit._body_state(env, body)
        shape = body.shapes[0]
        out.append((st.pos, st.rotation() @ bodykit.rotation(shape.quat()), np.array(shape.size)))
    return out


@bodykit.on_step
def observe(env):
    global _step
    _step += 1
    film = bagkit.nodes()
    tabs = tab_states(env)
    pot = bodykit._body_state(env, POT)
    pos, rad = soilkit.grains("mix")
    gaps = finger_gaps(env)
    _tally.update(pos, rad, film, tabs, pot.pos, pot.rotation(), gaps)
    if _trace is not None and _step % 5 == 0:
        row = {"step": _step, "metrics": _tally.metrics, "gaps": gaps, "pot": pot.pos.round(4).tolist(),
               "film_lo": film.min(0).round(4).tolist(), "film_hi": film.max(0).round(4).tolist(),
               "mouth": film[bagkit.MOUTH].mean(0).round(4).tolist(),
               "tabs": {b.name: t[0].round(4).tolist() for b, t in zip(bagkit.TAB_BODIES, tabs)}, "tcp": {}}
        if _step % 50 == 0 and _tally.spilled is not None:  # where the spilled grains are (env frame, mm)
            row["spilled_at"] = (pos[_tally.spilled] * 1000).round().astype(int).tolist()
        for arm in ("left", "right"):
            robot = env.unwrapped.scene[arm + "_robot"]
            ids, _ = robot.find_joints(["joint" + str(i) for i in range(1, 7)], preserve_order=True)
            row["tcp"][arm] = fastik.fk(bodykit.as_numpy(robot.data.joint_pos)[0][ids], p.BASES[arm])[0].round(4).tolist()
        _trace.write(json.dumps(row) + "\n")
        _trace.flush()


def score(objects):
    return _tally.metrics.get("score", 0.0)


def success(objects):
    return score(objects) == 1.0


def demo():
    """The scripted plan as TCP waypoints (forward kinematics of its joint targets, one per control step)."""
    if os.environ.get("HOLD_S"):
        out = {}
        for arm in ("left", "right"):
            pos, rot = fastik.fk(p.READY, p.BASES[arm])
            out[arm] = [Waypoint(float(os.environ["HOLD_S"]), tuple(pos), FINGER_OPEN, "hold", rot)]
        return out
    out = {"left": [], "right": []}
    for row in p.plan().steps:
        for arm, (q, grip, label) in row.items():
            pos, rot = fastik.fk(q, p.BASES[arm])
            out[arm].append(Waypoint(p.DT, tuple(pos), FINGER_OPEN * (1.0 - grip), label, rot))
    return out
