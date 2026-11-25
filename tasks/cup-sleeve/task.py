"""Put a corrugated sleeve on a full, lidded 16 oz paper hot cup.

On the station counter: a full 16 oz single-wall paper hot cup with its sip lid on, and a clear acrylic sleeve
organizer holding one flat-packed kraft corrugated cup sleeve standing on its narrow edge. The robot must open the
sleeve into a ring, get the cup into it narrow end first and slide the sleeve up the cup's taper until it fits snugly,
then leave the cup standing on the counter and let go, never squeezing the full cup across its wall.

Scoring: sleevescore.py (task.json's scoring). Model: sleevekit.py and sleevegeom.py (task.json's fidelity).
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402
import robotpts  # noqa: E402
import sleevegeom as G  # noqa: E402
import sleevekit as K  # noqa: E402
import sleevescore  # noqa: E402
import steptime  # noqa: E402

bodykit.install()

SCENE = "station"
SUCCESS_HOLD_S = 1.0
OBJECTS = K.bodies()
MODE = os.environ.get("TASK_DEMO", "success")

MODEL = K.Model()
_S = {"scorer": sleevescore.Scorer(), "n": 0, "log": []}


@bodykit.on_reset
def _reset() -> None:
    MODEL.reset()
    _S.update(scorer=sleevescore.Scorer(), n=0, log=[])


def _cup_points(pos, quat) -> np.ndarray:
    R = G.rotmat(quat)
    pts = []
    for h in np.linspace(0.0, G.CUP_H + G.LID_H, 9):
        r = G.cup_radius(h) if h <= G.CUP_H else G.LID_D / 2
        for a in np.linspace(0, 2 * math.pi, 16, endpoint=False):
            pts.append((r * math.cos(a), r * math.sin(a), h))
    return np.asarray(pos) + np.array(pts) @ R.T


@bodykit.on_step
def _step(env) -> None:
    try:
        _step_inner(env)
    except Exception:
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        raise


def _step_inner(env) -> None:
    MODEL.attach(env)
    _S["n"] += 1
    m = MODEL.step()
    P = m["poses"]
    cpos, cq = P["cup"]
    strips = G.facet_points(K.STRIP_LOCAL, [P[n] for n in K.STRIPS])
    oc = G.on_cup(strips, cpos, cq)
    _S["centres"] = [p.mean(axis=0) for p in strips]
    R = G.rotmat(cq)
    tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1, 1))))
    sq = K.squeeze_by_hand(m["robot_f"], m["strip_cup_f"], m["link_side"], cq)
    robot = robotpts.robot_points(env)
    things = np.vstack([_cup_points(cpos, cq)] + strips)
    d = float(np.min(np.linalg.norm(robot[:, None, :] - things[None, :, :], axis=2))) if len(robot) else 1.0
    f = sleevescore.Frame(t=_S["n"] / 20.0, squeeze=sq, cup_tilt_deg=tilt, cup_base_z=float(cpos[2] - K.TOP),
                          around=oc["around"], lows=oc["low"], gaps=oc["gap"], opened=G.opened_size(strips),
                          robot_dist=d)
    sc = _S["scorer"]
    sc.update(f)
    if os.environ.get("TASK_CLOSEUP"):  # the demo video's close-up only; it changes nothing in the trial
        import closeup
        closeup.start(K._arr(env.unwrapped.scene.env_origins)[0])
        closeup.step(np.mean(_S["centres"], axis=0))
    _log(m, f, sc, cpos)


def _touch(m) -> dict:
    """Development log: which robot links push on the cup (row 0) or the sleeve (other rows), N."""
    out = {}
    f = np.linalg.norm(m["robot_f"], axis=2)
    for j, path in enumerate(MODEL.links):
        name = path.rsplit("/", 1)[-1] + m["link_side"][j][0]
        c, s = float(f[0, j]), float(f[1:, j].sum())
        if c > 0.2 or s > 0.2:
            out[name] = [round(c, 1), round(s, 1)]
    return out


def _tips() -> dict:
    """Each arm's TCP (from its own joints) and finger opening (m between the pads), for the development log."""
    env = bodykit.env()
    if env is None:
        return {}
    from yam_arena import ik
    base = env.unwrapped
    out = {}
    for side, y in (("left", 0.305), ("right", -0.305)):
        r = base.scene[f"{side}_robot"]
        q = K._arr(r.data.joint_pos)[0]
        names = list(r.joint_names)
        arm = [q[names.index(f"joint{i}")] for i in range(1, 7)]
        p = ik.tcp_pose(np.array(arm), np.array((0.0, y, 0.75)))[0]
        out[side[0]] = np.round(p, 4).tolist() + [round(-2000 * float(q[names.index("joint7")]), 1)]
        if os.environ.get("TASK_DEBUG_LINKS"):
            bn = list(r.body_names)
            bp = K._arr(r.data.body_pos_w)[0] - K._arr(base.scene.env_origins)[0]
            bq = K._arr(r.data.body_quat_w)[0]
            out[side[0] + "_q"] = [round(float(q[names.index(f"joint{i}")]), 4) for i in range(1, 9)]
            for ln in ("gripper", "tip_left", "tip_right"):
                i = bn.index(ln)
                out[side[0] + "_" + ln] = np.round(bp[i], 5).tolist() + np.round(bq[i], 5).tolist()
    return out


def _log(m, f, sc, cpos) -> None:
    out = os.environ.get("TASK_SCORE_DIR")
    if not out:
        return
    rec = {"step": _S["n"], "score": sc.score, "check": sc.last, "sq": {k: round(v, 2) for k, v in f.squeeze.items()},
           "tilt": round(f.cup_tilt_deg, 1), "cup": np.round(cpos - np.array((*K.CUP_XY, K.TOP)), 4).tolist(),
           "ang": np.round(m["angle"], 1).tolist(), "rest": np.round(m["rest"], 1).tolist(),
           "dist_mm": round(1000 * f.robot_dist, 1), "tips": _tips(), "cupf": _touch(m),
           "strips": [np.round(c, 4).tolist() for c in _S.get("centres", [])]}
    _S["log"].append(rec)
    if _S["n"] % 20 == 0:
        pathlib.Path(out).mkdir(parents=True, exist_ok=True)
        (pathlib.Path(out) / "task_log.json").write_text(json.dumps(_S["log"], default=float))


def success(objects) -> bool:
    steptime.tick()
    return bool(_S["scorer"].success)


def score(objects) -> float:
    return float(_S["scorer"].score)


def demo():
    """Open-loop waypoints for each arm (sleevedemo.py), played by the harness's yam_scripted policy. They are planned
    from the start layout, which is fixed (object_jitter=0)."""
    import sleevedemo
    return sleevedemo.plan(MODE)
