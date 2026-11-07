"""Paired physics control. Teleport the stopper only here, never in the demo.

The identical 0.6 N, 0.4 s foot pull is applied with and without the stopper.
This isolates the restraint's physical effect from policy and scorer behavior.
"""

import json
from pathlib import Path

import numpy as np
from inspect_robots import Scene
from inspect_robots.types import Action
from run import install
from yam_arena.constants import READY
from yam_arena.inspect_embodiment import YamArenaEmbodiment


def main():
    install()
    env = YamArenaEmbodiment(
        task=str(Path(__file__).with_name("task.py")),
        object_jitter=0,
        control_hz=20,
        cam_width=320,
        cam_height=176,
    )
    scene = Scene(
        id="panel-physics-control", instruction="Physical restraint validation"
    )
    ready = Action(data=[*READY, 1.0, *READY, 1.0])
    results = []

    for braced in (False, True, False):
        env.reset(scene, seed=0)
        from yam_arena.scene import object_states

        m = env.task.module
        # Test harness bypasses scoring; every test still uses normal physics.
        m.MONITOR.armed = False
        m.MONITOR.control = lambda objects, clear: None
        if braced:
            obj = env._base.scene["stopper"]
            pose = obj.data.default_root_pose.torch.clone()
            pose[:, :3] = env._torch.tensor(
                m.STOP_GOAL, dtype=pose.dtype, device=pose.device
            )
            obj.write_root_pose_to_sim_index(root_pose=pose)
            obj.write_root_velocity_to_sim_index(
                root_velocity=env._torch.zeros_like(obj.data.default_root_vel.torch)
            )
        for _ in range(10):
            env.step(ready)
        initial = object_states(env._env, m.NAMES)
        foot0 = m.panel_foot(initial["panel"])
        peak_drift, peak_tilt = 0.0, 0.0
        for i in range(round((m.PULL_S + m.SETTLE_S) * 20)):
            states = object_states(env._env, m.NAMES)
            env._panel_probe_force_n = -m.PULL_N if i < round(m.PULL_S * 20) else 0.0
            env.step(ready)
            states = object_states(env._env, m.NAMES)
            peak_drift = max(
                peak_drift,
                float(np.linalg.norm(m.panel_foot(states["panel"])[:2] - foot0[:2])),
            )
            peak_tilt = max(peak_tilt, m.tilt(states["panel"]))
        env._panel_probe_force_n = 0.0
        result = {
            "braced": braced,
            "peak_foot_drift_m": peak_drift,
            "peak_tilt_deg": peak_tilt,
            "stopper_seated": m.seated(states["stopper"]),
            "initial_panel": initial["panel"].pos.tolist(),
            "final_panel": states["panel"].pos.tolist(),
        }
        results.append(result)
        print(json.dumps(result), flush=True)
    output = Path("panel-physics-control.json")
    output.write_text(json.dumps(results, indent=2) + "\n")
    assert all(
        r["peak_foot_drift_m"] > 0.018 or r["peak_tilt_deg"] > 32
        for r in (results[0], results[2])
    ), "Unbraced panel did not fail the same physical pull"
    assert (
        results[1]["peak_foot_drift_m"] < 0.018
        and results[1]["peak_tilt_deg"] < 32
        and results[1]["stopper_seated"]
    ), "Stopper did not physically restrain the panel"
    print("PASS: paired physical control, including reset repeat", flush=True)
    env.close()


if __name__ == "__main__":
    main()
