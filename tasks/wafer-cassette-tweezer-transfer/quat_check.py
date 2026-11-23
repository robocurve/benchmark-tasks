"""Measure the quaternion order reaching the scorer (simulator). Prints the raw 4-vectors from object_states (rigid /
kinematic objects) and from artkit.body_pose (articulation links), next to the spawn rotation written both ways, and
where each reading would send the body's x, y, z axes."""
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))



def R_from_xyzw(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    emb = yam_arena_embodiment(task=str(HERE / "quat_probe_task.py"), object_jitter=0, control_hz=20, cam_width=64, cam_height=36)
    emb.reset(Scene(id="q", instruction="q"))
    from yam_arena.scene import object_states
    import artkit as ak
    import quat_probe_task as QT

    for _ in range(5):
        emb.step(Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0])))
    st = object_states(emb._env, emb.task.names)
    exp = np.array(QT.Q_TEST)
    out = {"spawn_q_test_as_xyzw": exp.round(4).tolist(), "spawn_q_test_as_wxyz": np.r_[exp[3], exp[:3]].round(4).tolist(),
           "spawn_q_z90_as_xyzw": np.array(QT.Q_Z90).round(4).tolist()}
    raw = np.asarray(st["probe"].quat_xyzw, float)
    out["object_states_probe_raw"] = raw.round(4).tolist()
    _, qa = ak.body_pose("tweezers", "half_a")
    out["body_pose_tweezers_half_a_raw"] = np.asarray(qa, float).round(4).tolist()
    out["object_states_tweezers_raw"] = np.asarray(st["tweezers"].quat_xyzw, float).round(4).tolist()
    Rexp = R_from_xyzw(exp)
    out["expected_axes_probe(x,y,z cols)"] = Rexp.round(3).tolist()
    for label, q in (("read_as_xyzw", raw), ("read_as_wxyz", np.r_[raw[1:], raw[0]])):
        Rr = R_from_xyzw(q / np.linalg.norm(q))
        out[f"probe_{label}_axis_error_deg"] = [round(math.degrees(math.acos(np.clip(Rr[:, i] @ Rexp[:, i], -1, 1))), 2) for i in range(3)]
    Rz90 = R_from_xyzw(np.array(QT.Q_Z90))
    for label, q in (("read_as_xyzw", np.asarray(qa, float)), ("read_as_wxyz", np.r_[np.asarray(qa)[1:], np.asarray(qa)[0]])):
        Rr = R_from_xyzw(q / np.linalg.norm(q))
        out[f"tweezers_{label}_axis_error_deg"] = [round(math.degrees(math.acos(np.clip(Rr[:, i] @ Rz90[:, i], -1, 1))), 2) for i in range(3)]
    Path(sys.argv[1]).write_text(json.dumps(out, indent=1))
    print("QUAT " + json.dumps(out), flush=True)
    os._exit(0)


if __name__ == "__main__":
    main()
