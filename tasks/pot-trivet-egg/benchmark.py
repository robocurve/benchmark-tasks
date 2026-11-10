"""Custom-USD task through Inspect Robots' public embodiment interface.

The policy receives only the same 14 joint values and three RGB images as the
supplied yam_arena embodiment. Privileged object state is used only for scoring.
"""

import atexit
import hashlib
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import threading

import numpy as np

from inspect_robots import EmbodimentBase, EmbodimentInfo, Observation, StepResult
from inspect_robots.embodiment import PRIVILEGED_SUCCESS, RESETTABLE
from inspect_robots_yam.config import DEFAULT_CAMERAS, action_box, observation_space
from yam_arena import ik
from yam_arena.constants import READY

import task


class PotTrivetEggEmbodiment(EmbodimentBase):
    def __init__(self):
        low, high = np.r_[ik.LOWER, 0, ik.LOWER, 0], np.r_[ik.UPPER, 1, ik.UPPER, 1]
        digest = hashlib.sha256()
        revision_files = [task.ROOT/name for name in (
            "task.py", "task_objects.usda", "scoring.py", "layout.json", "benchmark.py", "demo_runtime.py",
            "assets/pot.usda", "assets/trivet.usda", "assets/egg.usda", "assets/egg_cup.usda",
        )]
        revision_files.extend(sorted((task.ROOT/"assets/textures").glob("*.png")))
        for path in revision_files:
            digest.update(path.relative_to(task.ROOT).as_posix().encode())
            digest.update(path.read_bytes())
        self.info = EmbodimentInfo(
            name="yam_arena", action_space=action_box(low, high),
            observation_space=observation_space(176, 320, DEFAULT_CAMERAS),
            control_hz=20, is_simulated=True,
            capabilities=frozenset({RESETTABLE, PRIVILEGED_SUCCESS}),
            environment_id="pot-trivet-egg-local",
            environment_revision=digest.hexdigest()[:16],
            docs="Two YAM arms with absolute joint-position targets and normalized parallel grippers. "
                 "Action/state order: left six joints, left gripper, right six joints, right gripper. "
                 "Joints are radians; grippers are 0 closed, 1 open. "
                 "RGB cameras: top_cam, left_cam, right_cam. Task objects are observed in the images.",
        )
        self.low, self.high = low, high
        self.app = self.env = self.reader = None
        self._lock = None
        self.steps, self.instruction = 0, task.INSTRUCTION
        self.state_log = None

    def _start(self):
        if self.env is not None:
            return
        visible = subprocess.check_output(["nvidia-smi", "--query-gpu=uuid", "--format=csv,noheader"], text=True).strip().splitlines()
        # The host launcher explicitly supplies the selected single GPU UUID.
        expected = os.environ.get('YAM_EXPECTED_GPU_UUID')
        if visible != [expected]:
            raise RuntimeError("Expose exactly the explicitly allocated single GPU")
        self._lock = Path("/outputs/.scene-run.lock").open("w")
        fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        os.environ["OMNICLIENT_HUB_MODE"] = "disabled"
        from isaaclab.app import AppLauncher
        from demo_runtime import StateReader

        self.app = AppLauncher(
            headless=True, enable_cameras=True, device="cuda:0", multi_gpu=False,
            kit_args=f"--portable-root {Path.home()/'kit'} --/renderer/multiGpu/enabled=false "
                     "--/renderer/multiGpu/autoEnable=false --/renderer/multiGpu/maxGpuCount=1 "
                     "--/renderer/activeGpu=0 --/plugins/carb.tasking.plugin/threadCount=8",
        ).app
        self.env, specs = task.build_environment(width=640,height=360)
        self.reader = StateReader(self.env, specs)
        if os.environ.get("YAM_STATE_LOG"):
            path = Path(os.environ["YAM_STATE_LOG"])
            path.parent.mkdir(parents=True, exist_ok=True)
            self.state_log = path.open("w")
        atexit.register(self.shutdown)

    def reset(self, scene, *, seed=None):
        import torch

        self._start()
        self.env.reset()
        self.reader.reset()
        ready = torch.tensor([[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device="cuda:0")
        for _ in range(60):
            self.env.step(ready)
        for _ in range(4):
            self.env.unwrapped.sim.render()
        initial=self.reader.controls()
        if not initial['valid'] or initial['egg_clear'] or initial['pot_on_trivet'] or initial['pot_egg_force_n']>.20:
            raise RuntimeError('Invalid initial pot/trivet/egg scene; refusing model evaluation')
        self.reader.scorer.reset()
        self.steps, self.instruction = 0, scene.instruction
        return self._observation()

    def _observation(self):
        import cv2

        images = {name: np.ascontiguousarray(cv2.resize(self.reader.rgb(name), task.POLICY_SIZE))
                  for name in task.POLICY_CAMERAS}
        joints = np.concatenate([np.r_[self.reader.arm(side)[0], self.reader.arm(side)[1]]
                                 for side in ("left", "right")])
        return Observation(images=images, state={"joint_pos": joints}, instruction=self.instruction,
                           image_times=dict.fromkeys(images, self.steps/20), state_time=self.steps/20)

    def step(self, action):
        import torch

        values = np.asarray(action.data, dtype=float).reshape(-1)
        if values.shape != (14,) or not np.isfinite(values).all():
            raise ValueError("Expected 14 finite joint/gripper targets")
        self.env.step(torch.as_tensor(np.clip(values, self.low, self.high)[None], dtype=torch.float32, device="cuda:0"))
        self.steps += 1
        score = self.reader.score()
        if self.state_log:
            self.state_log.write(json.dumps(dict(
                step=self.steps, action=values.tolist(),
                measured=self.reader.controls(), **score)) + "\n")
            self.state_log.flush()
        if self.steps % 20 == 0:
            print("DREAM_PROGRESS",json.dumps(dict(step=self.steps,**score)),flush=True)
        return StepResult(observation=self._observation(), terminated=score["success"],
                          termination_reason="success" if score["success"] else None, info=score)

    def close(self):
        if self.state_log:
            self.state_log.close()
            self.state_log = None
        if self.env is not None:
            self.env.close()
            self.env = self.reader = None

    def shutdown(self, exit_code=0):
        if self.app is None:
            return
        app, self.app = self.app, None
        timer = threading.Timer(25, lambda: os._exit(exit_code))
        timer.daemon = True
        timer.start()
        try:
            self.close()
            app.close()
        finally:
            timer.cancel()
            if self._lock is not None:
                self._lock.close()
                self._lock = None
