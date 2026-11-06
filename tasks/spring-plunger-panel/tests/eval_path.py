"""The scripted demo on the Dream-1 command path, without calling Dream-1.

``inspect-robots run --policy dream`` runs ``inspect_robots.eval`` on an ad-hoc task (the instruction, the
``task_score`` scorer, 600 steps) with the CLI's default guardrails (clamp and delta limit), stored frames and a JSON
log. This script does the same with the demo of ``demo.py`` as the policy, and the cameras at 320 x 176 as the model
gets them, then writes ``summary.json`` to the log folder. Set ``TOP_CAM_VIDEO`` to record the top camera as the
README's Dream-1 command does.

    python tests/eval_path.py LOG_DIR
"""

import json
import sys
from pathlib import Path

TASK_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(TASK_DIR))
sys.argv = [sys.argv[0], "--/renderer/multiGpu/activeCudaGpus=0,"]  # Kit reads its settings from argv

import numpy as np  # noqa: E402
from inspect_robots import Scene, Task  # noqa: E402
from inspect_robots import eval as run_eval  # noqa: E402
from inspect_robots.cli import _build_guardrails  # noqa: E402
from inspect_robots.policy import PolicyBase, PolicyInfo  # noqa: E402
from inspect_robots.registry import resolve  # noqa: E402
from inspect_robots.types import Action, ActionChunk  # noqa: E402
from inspect_robots_yam.config import action_box  # noqa: E402
from inspect_robots_yam.packing import pack  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.inspect_embodiment import yam_arena_embodiment  # noqa: E402

import demo  # noqa: E402  (the task's demo.py)


class ScriptedDemoPolicy(PolicyBase):
    """The task's scripted demo as an Inspect Robots policy (it reads the panel and plunger from the simulator)."""

    def __init__(self, embodiment):
        low, high = np.append(ik.LOWER, 0.0), np.append(ik.UPPER, 1.0)
        self.info = PolicyInfo(name="panel_demo", action_space=action_box(pack(low, low), pack(high, high)))
        self.embodiment, self.script, self.dt, self.steps = embodiment, None, 0.05, 0

    def bind(self, embodiment_info) -> None:
        if embodiment_info.control_hz:
            self.dt = 1.0 / float(embodiment_info.control_hz)

    def reset(self, scene) -> None:
        self.script, self.steps = demo.Scripted(self.embodiment.task.module, "demo"), 0

    def act(self, observation) -> ActionChunk:
        from yam_arena.scene import object_states

        joints = object_states(self.embodiment._env, ["cabinet"])["cabinet"].joints
        t = float(observation.state_time) + self.dt
        action = self.script.step(t, joints["panel_hinge"], joints["plunger_slide"])
        self.steps += 1
        meta = {"request_stop": True} if self.script.done(t) else {}
        return ActionChunk(actions=[Action(data=pack(action[:7], action[7:]), meta=meta)], control_hz=1.0 / self.dt)


emb = yam_arena_embodiment(task=str(TASK_DIR / "task.py"), object_jitter=0, control_hz=20, cam_width=320,
                           cam_height=176)
instruction = emb.task.module.INSTRUCTION
task = Task(name="adhoc", scenes=[Scene(id="scene-0", instruction=instruction)], scorer=resolve("scorer", "task_score"),
            max_steps=600, metadata={"instruction": instruction, "adhoc": True})
approver, active, warnings = _build_guardrails(emb.info.action_space, None, emb)
policy = ScriptedDemoPolicy(emb)
run_eval(task, policy, emb, log_dir=str(LOG_DIR), seed=0, approver=approver, store_frames=True)
(log_path,) = LOG_DIR.glob("*.json")  # the JSON log, as the CLI writes it
data = json.loads(log_path.read_text())
sample = data["samples"][0]
summary = {
    "status": data["status"],
    "task_score": data["results"]["metrics"]["task_score"],
    "termination": sample["termination_reasons"][0],
    "steps": policy.steps,
    "guardrails": list(active),
    "episode": emb.task.module.episode(),
}
(LOG_DIR / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary), flush=True)
