"""Commissioning control that presets each guide and tests the same physics.

The control presets guide and keeper poses to isolate physical travel.
"""

import argparse
import json
from pathlib import Path

import task
from inspect_robots import Scene
from inspect_robots.types import Action
from run import install
from yam_arena.constants import READY
from yam_arena.inspect_embodiment import YamArenaEmbodiment


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trace", type=Path)
    args = parser.parse_args()
    install(trace=args.trace)
    env = YamArenaEmbodiment(
        task=str(Path(task.__file__)),
        object_jitter=0,
        control_hz=20,
        cam_width=320,
        cam_height=176,
    )
    scene = Scene(id="split-guide-probe", instruction=task.INSTRUCTION)
    hold = Action(data=[*READY, 1.0, *READY, 1.0])
    for name, expected in [("wide_guide", 1.0), ("narrow_guide", 0.5), (None, 0.0)]:
        env.reset(scene, seed=0)
        base = env._base
        if name:
            for obj_name, target in (
                (name, (task.X, 0, task.Z)),
                ("keeper", (task.KEEPER_POS[0], 0, task.KEEPER_POS[2])),
            ):
                body = base.scene[obj_name]
                pose = body.data.default_root_pose.torch.clone()
                pose[:, :3] = env._torch.tensor(
                    target, dtype=pose.dtype, device=base.device
                )
                body.write_root_pose_to_sim_index(root_pose=pose)
                body.write_root_velocity_to_sim_index(
                    root_velocity=env._torch.zeros_like(
                        body.data.default_root_vel.torch
                    )
                )
            for _ in range(8):
                env.step(hold)
        env.fixture_trial.start = env._t
        env.fixture_trial.eligible_guide = name
        for _ in range(100):
            result = env.step(hold)
            if result.terminated:
                break
        print(json.dumps({"probe": name, **result.info}), flush=True)
        assert result.info["score"] == expected, (
            f"{name}: expected {expected}, got {result.info}"
        )
        if name == "narrow_guide":
            assert 0.005 <= result.info["drawer_travel_m"] < 0.015
        else:
            assert result.info["drawer_travel_m"] >= task.PASS_TRAVEL
    env.close()


if __name__ == "__main__":
    main()
