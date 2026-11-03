"""Run one audited die-roll trial using the guide's Inspect Robots workflow."""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import platform
import time
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import numpy as np

from scoring import RollTracker, SCORE_VERSION

HERE = Path(__file__).resolve().parent
INSTRUCTION = "Pick up the small blue cube and roll it like a die on the table. Release it so it tumbles freely, then leave it untouched until it stops."


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("demo", "dream"), default="demo")
    parser.add_argument("--max-steps", type=int, default=600)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--log-dir", type=Path, required=True)
    parser.add_argument("--headless", choices=("true", "false"), default="true")
    parser.add_argument("--dream-server-url", help="optional task batch bridge, e.g. http://127.0.0.1:8210")
    args = parser.parse_args()
    if not 1 <= args.max_steps <= 1200:
        parser.error("max-steps must be 1..1200 (60 seconds at 20 Hz)")
    if args.dream_server_url and args.policy != "dream":
        parser.error("dream-server-url only applies to the Dream policy")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    if (args.log_dir / "trajectory.jsonl").exists():
        parser.error("choose a fresh log directory; existing audit data will not be overwritten")

    from inspect_robots.cli import main as inspect_main
    from inspect_robots.registry import register
    from yam_arena import ik
    from yam_arena.constants import FINGER_OPEN, GRIPPER, SHOULDER, WRIST
    from yam_arena.inspect_embodiment import YamArenaEmbodiment

    class AuditedDieEmbodiment(YamArenaEmbodiment):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            if not self.info.is_simulated:
                raise RuntimeError("this task runner is for simulation only")
            hz = self.info.control_hz
            per_arm = (SHOULDER["velocity"] / hz,) * 3 + (WRIST["velocity"] / hz,) * 3 + (GRIPPER["velocity"] / abs(FINGER_OPEN) / hz,)
            semantics = replace(self.info.action_space.semantics, max_step=per_arm * 2)
            self.info = replace(self.info, action_space=replace(self.info.action_space, semantics=semantics))

        def reset(self, scene, *, seed=None):
            observation = super().reset(scene, seed=seed)
            import cv2

            # Save the actual settled initial scene, before any policy action.
            rgb = self._base.scene["top_cam"].data.output["rgb"]
            rgb = rgb.torch if hasattr(rgb, "torch") else rgb
            rgb = rgb[0, ..., :3].detach().cpu().numpy().astype(np.uint8)
            if not cv2.imwrite(str(args.log_dir / "setup.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
                raise RuntimeError("could not save the initial scene image")
            self.task.module._tracker = RollTracker()
            self.audit = (args.log_dir / "trajectory.jsonl").open("x")
            self._audit_sample()
            return observation

        def _audit_sample(self):
            from yam_arena.scene import object_states
            die = object_states(self._env, ("die",))["die"]
            hands = []
            for side in ("left", "right"):
                arm_name = side + "_robot"
                arm_ids, finger_ids = self._ids[arm_name]
                q = self._base.scene[arm_name].data.joint_pos.torch[0]
                joints = q[arm_ids].detach().cpu().numpy().astype(float)
                tcp, tcp_rotation = ik.tcp_pose(joints, self.rig.bases[side])
                opening = float(np.clip(q[finger_ids].mean().item() / FINGER_OPEN, 0, 1))
                hands.append({
                    "tcp": tcp.tolist(), "opening": opening,
                    "closing_axis": tcp_rotation[:, 0].tolist(),
                    "joints": joints.tolist(), "tcp_rotation": tcp_rotation.tolist(),
                })
            sample = {"time": float(self._t), "hands": hands}
            for key in ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high"):
                sample[key] = getattr(die, key).tolist()
            self.task.module._tracker.update(sample)
            sample["metrics"] = self.task.module._tracker.summary()
            self.audit.write(json.dumps(sample, allow_nan=False) + "\n")
            self.audit.flush()

        def _task_step(self):
            self._audit_sample()
            return super()._task_step()

        def close(self):
            if hasattr(self, "audit") and not self.audit.closed:
                self.audit.close()
                (args.log_dir / "score-summary.json").write_text(json.dumps(self.task.module._tracker.summary(), indent=2) + "\n")
            super().close()

    register("embodiment", "audited_die")(AuditedDieEmbodiment)
    policy_name = "dream"
    if args.policy == "demo":
        from demo_policy import register_policy
        policy_name = register_policy()
    elif args.dream_server_url:
        from dream_bridge import BatchedBridgeSession
        from inspect_robots_dream.policy import DreamPolicy

        def bridged_dream():
            return DreamPolicy(
                server_url=args.dream_server_url,
                session=BatchedBridgeSession(args.dream_server_url),
            )

        policy_name = "die_dream_bridge"
        register("policy", policy_name)(bridged_dream)
    cli_args = [
        "run", "--embodiment", "audited_die", "--policy", policy_name,
        "-E", f"task={HERE / 'task.py'}", "-E", "object_jitter=0", "-E", "control_hz=20",
        "-E", "cam_width=320", "-E", "cam_height=176", "-E", f"headless={args.headless}",
        "--instruction", INSTRUCTION, "--scorer", "task_score", "--epochs", "1", "--seed", str(args.seed),
        "--max-steps", str(args.max_steps), "--store-frames", "--log-dir", str(args.log_dir),
        "--no-prompt", "--grader", "none",
    ]
    metadata = {"python": platform.python_version(), "policy": args.policy, "seed": args.seed, "max_steps": args.max_steps, "control_hz": 20, "score_version": SCORE_VERSION, "packages": {}}
    if args.policy == "dream":
        metadata["dream_transport"] = "batched_bridge" if args.dream_server_url else "direct"
    snapshot = args.log_dir / "task-source"
    snapshot.mkdir(exist_ok=True)
    metadata["source_sha256"] = {}
    for source in sorted(HERE.glob("*.py")):
        if source.name.startswith("."):
            continue  # Exclude filesystem metadata such as macOS AppleDouble files.
        content = source.read_bytes()
        (snapshot / source.name).write_bytes(content)
        metadata["source_sha256"][source.name] = hashlib.sha256(content).hexdigest()
    for package in ("numpy", "yam-arena", "inspect-robots", "inspect-robots-yam", "inspect-robots-dream", "dreamlabs-client", "isaaclab", "isaacsim"):
        try:
            metadata["packages"][package] = version(package)
        except PackageNotFoundError:
            metadata["packages"][package] = "not registered"
    started = time.monotonic()
    try:
        return inspect_main(cli_args)
    finally:
        metadata["elapsed_wall_seconds"] = round(time.monotonic() - started, 3)
        (args.log_dir / "runtime.json").write_text(json.dumps(metadata, indent=2) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
