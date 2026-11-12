"""tracekit: log both arms' gripper positions and finger joints every control step (debugging only).

Import it from a task file AFTER `parts` (it registers an on_step callback through parts.py):

    import tracekit  # noqa: F401  (writes <TASK_TRACE>.hands.jsonl when TASK_TRACE is set)

One JSON line per step: step number, and for each arm the gripper link position (world frame minus the env origin, metres),
the finger joint positions, and the list of body names once at the start. No effect when TASK_TRACE is not set.
"""

from __future__ import annotations

import json
import os

from parts import _to_numpy, env, on_step

_STATE = {"n": 0, "names": {}}


def _hand_trace(e) -> None:
    base_path = os.environ.get("TASK_TRACE")
    if not base_path or e is None:
        return
    path = base_path + ".hands.jsonl"
    try:
        scene = e.unwrapped.scene
        origin = _to_numpy(scene.env_origins[0])
        record = {"n": _STATE["n"]}
        _STATE["n"] += 1
        for arm in ("left", "right"):
            robot = scene[f"{arm}_robot"]
            names = list(robot.body_names)
            if arm not in _STATE["names"]:
                _STATE["names"][arm] = names
                record[f"{arm}_body_names"] = names
            pos = _to_numpy(robot.data.body_pos_w)[0]
            idx = names.index("gripper") if "gripper" in names else len(names) - 1
            record[arm] = [round(float(v), 4) for v in (pos[idx] - origin)]
            jn = list(robot.joint_names)
            jp = _to_numpy(robot.data.joint_pos)[0]
            record[f"{arm}_fingers"] = [round(float(jp[jn.index(j)]), 4) for j in ("joint7", "joint8") if j in jn]
        # Every compound part registered through parts.py: body-origin position and slider joint positions (m).
        try:
            from parts import _PARTS, _read_part_state

            states = {}
            for pname, spec in _PARTS.items():
                st = _read_part_state(e, spec)
                states[pname] = {
                    "p": [round(float(v), 4) for v in st.pos],
                    "s": {k: round(float(v), 4) for k, v in st.sliders.items()},
                    "v": round(float((st.lin_vel**2).sum() ** 0.5), 4),
                }
            record["parts"] = states
        except Exception as exc2:
            record["parts_error"] = f"{type(exc2).__name__}: {exc2}"
        with open(path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(record) + "\n")
    except Exception as exc:  # never break an episode for a debug trace
        if not _STATE.get("warned"):
            _STATE["warned"] = True
            with open(path, "a", encoding="utf-8") as fh:
                fh.write(json.dumps({"error": f"{type(exc).__name__}: {exc}"}) + "\n")


on_step(_hand_trace)
