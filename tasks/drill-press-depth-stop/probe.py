"""Check full, partial and zero credit at three preset physical detents."""

import json
from pathlib import Path

from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.constants import READY
from yam_arena.inspect_embodiment import YamArenaEmbodiment

from run import install
import task


def main():
    install(None)
    env = YamArenaEmbodiment(
        task=str(Path(task.__file__)),
        object_jitter=0,
        control_hz=20,
        cam_width=320,
        cam_height=176,
    )
    scene = Scene(id="depth-stop-control", instruction=task.INSTRUCTION)
    hold = Action(data=[*READY, 1.0, *READY, 1.0])
    try:
        for index, expected in ((0, 0.0), (1, 0.5), (2, 1.0)):
            env.reset(scene, seed=0)
            stop = env._base.scene["depth_stop"]
            pose = stop.data.default_root_pose.torch.clone()
            pose[:, :3] += env._base.scene.env_origins
            pose[:, 2] -= index * task.PITCH
            # Preset the fixture at reset for commissioning. The actual trial
            # demonstrates the robot moving the stop with the catch held open.
            stop.write_root_pose_to_sim_index(root_pose=pose)
            stop.write_root_velocity_to_sim_index(
                root_velocity=env._torch.zeros_like(stop.data.default_root_vel.torch)
            )
            for _ in range(8):
                result = env.step(hold)
            assert result.info["score"] == 0.0
            loaded = []
            for _ in range(2):
                # Emulate the press trigger to isolate the physical load test.
                env.proof.start = env._t
                env.proof.peak = result.info["depth_m"]
                env.proof.candidate.clear()
                env.proof.valid = True
                peak = float("-inf")
                for _ in range(41):
                    result = env.step(hold)
                    peak = max(peak, result.info["depth_m"])
                    if result.terminated:
                        break
                loaded.append(round(peak * 1000, 3))
            print(json.dumps({"detent": index, "peaks_mm": loaded, **result.info}), flush=True)
            assert result.info["score"] == expected, (index, result.info)
    finally:
        env.close()


if __name__ == "__main__":
    main()
