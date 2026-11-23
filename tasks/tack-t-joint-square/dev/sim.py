"""Dev only: several episodes in one simulator session, to measure the arms' give and to try the shortcuts.

    python dev/sim.py cal cal demo demo one lean hover     (under the same environment as an inspect-robots run)

Each word is one episode of the scripted plan, played open loop by the stock yam_scripted policy on the yam_tack
embodiment. "cal" plays the "hover" plan (the torch stops 10 mm from the root, so nothing is tacked), then measures where the
coupon stood and where the tungsten tip was against where they were aimed, and moves task.HOLD_CAL / task.TORCH_CAL by
the difference for the episodes that follow; it prints the values to copy into task.py. Any other word is a task.set_variant
name ("demo" is the full demo). The demo itself reads nothing from the simulator: only these fixed numbers go into it."""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
from inspect_robots import Scene  # noqa: E402
from tack_t_joint.embodiment import TackEmbodiment  # noqa: E402
from yam_arena.scripted import YamScriptedPolicy  # noqa: E402

INSTR = json.load(open(os.path.join(HERE, "task.json")))["instruction"] if os.path.exists(os.path.join(HERE, "task.json")) else "tack the T-joint"
MAX_STEPS = int(os.environ.get("TACK_STEPS", "1800"))


def windows(m):
    """(start, end) seconds of each labelled waypoint, per arm."""
    out = {}
    for arm, ws in m.demo().items():
        t, rows = 0.0, []
        for w in ws:
            rows.append((w.label, t, t + w.duration))
            t += w.duration
        out[arm] = rows
    return out


def measure(m, trace, dt):
    """Update the calibration from a hover episode's trace."""
    win = windows(m)
    strike = [r for r in win["left"] if "strike" in r[0]]
    k = int(strike[0][1] / dt) - 2  # just before the first tack: the coupon held on the line
    p, R, _, _ = trace[k]
    lean = math.asin(max(-1.0, min(1.0, R[2, 2])))  # top toward +y
    turn = math.atan2(R[1, 0], R[0, 0])
    up = R[:, 1] if R[2, 1] > 0 else -R[:, 1]
    foot = p - (m.M.W / 2) * up
    err = dict(x=p[0] - m.M.XC, y=p[1] - (m.M.YC + (m.M.W / 2) * math.sin(lean)), foot_y=foot[1] - m.M.YC, lean=lean, turn=turn, foot_z=foot[2] - m.BASE_Z)
    print("CAL hold: centre x %+.2f mm, foot y %+.2f mm, foot z %+.2f mm, lean %+.2f deg, turn %+.2f deg" % (
        1000 * err["x"], 1000 * err["foot_y"], 1000 * err["foot_z"], math.degrees(lean), math.degrees(turn)), flush=True)
    m.HOLD_CAL["x"] -= err["x"]
    m.HOLD_CAL["y"] -= p[1] - m.M.YC  # the centre: with the lean taken out, the foot comes under it
    m.HOLD_CAL["tilt"] += lean
    for label, t0, t1 in [r for r in win["left"] if "dwell" in r[0]]:
        i = int(label.split()[1].rstrip(":"))
        tips = np.array([trace[j][2] for j in range(int((t1 - 1.0) / dt), int(t1 / dt))])
        give = tips.mean(axis=0) - m.tack_tip(i)  # against the aim as commanded: the arm's give
        left = give + np.asarray(m.TORCH_CAL[i])  # against the mark's aim itself: what the current offset leaves over
        print("CAL torch mark %d: give (%+.2f, %+.2f, %+.2f) mm, sd %.2f; tip off the aim by (%+.2f, %+.2f, %+.2f) mm" % (
            i, *(1000 * give), 1000 * tips.std(axis=0).max(), *(1000 * left)), flush=True)
        m.TORCH_CAL[i] = tuple(float(v) for v in -give)
    print("CAL now: HOLD_CAL = dict(x=%.5f, y=%.5f, z=%.5f, tilt=math.radians(%.3f), yaw=math.radians(%.3f))" % (
        m.HOLD_CAL["x"], m.HOLD_CAL["y"], m.HOLD_CAL["z"], math.degrees(m.HOLD_CAL["tilt"]), math.degrees(m.HOLD_CAL["yaw"])), flush=True)
    print("CAL now: TORCH_CAL = {%s}" % ", ".join("%d: (%.5f, %.5f, %.5f)" % (i, *m.TORCH_CAL[i]) for i in sorted(m.TORCH_CAL)), flush=True)


def main():
    emb = TackEmbodiment(task=os.path.join(HERE, "task.py"), object_jitter=0, control_hz=20, cam_width=320, cam_height=176)
    pol = YamScriptedPolicy()
    pol.bind(emb.info)
    m = emb.task.module
    dt = 1.0 / emb.info.control_hz
    for n, word in enumerate(sys.argv[1:]):
        m.set_variant({"cal": "hover", "demo": ""}.get(word, word))
        m.TRACE = []
        scene = Scene(id=f"ep{n}", instruction=INSTR)
        print(f"EPISODE {n} {word} start", flush=True)
        obs = emb.reset(scene, seed=0)
        pol.reset(scene)
        info, steps = {}, 0
        for steps in range(1, MAX_STEPS + 1):
            action = pol.act(obs).actions[0]
            res = emb.step(action)
            obs, info = res.observation, res.info
            if action.meta.get("request_stop") or res.terminated:
                break
        tr = m._trial
        print(f"EPISODE {n} {word} end: steps {steps} score {info.get('score')} sound {m.sound_tacks()} faults {m.faults()} cracked {tr['cracked']} "
              f"tacks {[(tk['id'], tk['intact']) for tk in tr['tacks']]}", flush=True)
        if word == "cal":
            measure(m, m.TRACE, dt)
    os._exit(0)


main()
