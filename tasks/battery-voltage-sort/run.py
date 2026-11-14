"""Run and record this task's native PhysX reference demo or view its initial scene."""

import argparse
import hashlib
from importlib import metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

import numpy as np

import spec as S


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", choices=("demo", "hold", "view"), default="demo")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-steps", type=int, default=S.MAX_STEPS)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--record", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def source_digest():
    digest = hashlib.sha256()
    for file in sorted(S.DIRECTORY.glob("*.py")) + [S.DIRECTORY/"settings.json"] + sorted((S.DIRECTORY/"assets").iterdir()):
        digest.update(file.name.encode())
        digest.update(file.read_bytes())
    return digest.hexdigest()


class Recorder:
    def __init__(self, output, enabled):
        self.output, self.enabled = output, enabled
        self.writers = {}
        self.streams = []

    def save(self, env, initial=False):
        from PIL import Image
        import cv2

        images = {}
        for name in ("top_cam", "left_cam", "right_cam"):
            data = env.unwrapped.scene[name].data.output["rgb"]
            pixels = (data.torch if hasattr(data, "torch") else data)[0, ..., :3].cpu().numpy().astype(np.uint8)
            images[name] = pixels
            if initial:
                Image.fromarray(pixels).save(self.output/(name+"_initial.png"))
        if initial:
            Image.fromarray(images["top_cam"]).save(self.output/"setup.png")
        if not self.enabled:
            return
        top = cv2.resize(images["top_cam"], (640, 360))
        wrists = np.hstack([cv2.resize(images[name], (320, 180)) for name in ("left_cam", "right_cam")])
        images["demo"] = np.vstack([top, wrists])
        for name, pixels in images.items():
            if name not in self.writers:
                height, width = pixels.shape[:2]
                error = (self.output/(name+"_encoder.log")).open("w")
                self.streams.append(error)
                self.writers[name] = subprocess.Popen([
                    "ffmpeg", "-y", "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
                    "-r", str(S.HZ), "-i", "-", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-preset", "veryfast",
                    "-crf", "23", "-movflags", "+faststart", str(self.output/(name+".mp4"))],
                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=error)
            self.writers[name].stdin.write(np.ascontiguousarray(pixels).tobytes())

    def close(self):
        for process in self.writers.values():
            process.stdin.close()
        for name, process in self.writers.items():
            if process.wait(timeout=30) != 0:
                raise RuntimeError("Video encoding failed: " + name)
        for stream in self.streams:
            stream.close()

    def snapshot(self, env, filename):
        from PIL import Image
        data = env.unwrapped.scene["top_cam"].data.output["rgb"]
        pixels = (data.torch if hasattr(data, "torch") else data)[0, ..., :3].cpu().numpy().astype(np.uint8)
        Image.fromarray(pixels).save(self.output/filename)


def main():
    args = parse_args()
    if args.max_steps < 1 or args.max_steps > S.MAX_STEPS:
        raise ValueError(f"Use an action limit in 1..{S.MAX_STEPS}")
    if os.environ.get("OMNI_KIT_ACCEPT_EULA") != "YES" and os.environ.get("ACCEPT_EULA") != "Y":
        raise RuntimeError("Activate the simulator environment with your existing NVIDIA license settings")
    args.output.mkdir(parents=True, exist_ok=False)
    began = time.monotonic()
    initial_digest = source_digest()
    from yam_arena.task_file import load

    task = load(S.DIRECTORY/"task.py")
    from integration import STATE
    from isaaclab.app import AppLauncher

    options = {"headless": args.headless, "enable_cameras": True, "device": "cuda:0"}
    if not args.headless:
        options["visualizer"] = ["kit"]
    app = AppLauncher(**options).app
    recorder = Recorder(args.output, args.record and args.policy != "view")
    report = {"policy": args.policy, "steps": 0, "error": None}
    exit_code = 1
    try:
        from pxr import Usd
        for file in sorted((S.DIRECTORY/"assets").glob("*.usda")):
            if Usd.Stage.Open(str(file)) is None:
                raise RuntimeError("Could not load apparatus asset " + file.name)
        import torch
        from yam_arena.constants import READY
        from yam_arena import station
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import make_env
        from controller import Demonstration

        embodiment = BimanualYamEmbodiment(enable_cameras=True, rig=station.rig(), robot_contact_offset=0.0005)
        env = make_env(embodiment, scene="task", task=task, control_hz=S.HZ)
        base = env.unwrapped
        env.reset()
        ready = np.r_[READY, 1.0, READY, 1.0]
        for _ in range(10):
            env.step(torch.as_tensor(ready, dtype=torch.float32, device=base.device).unsqueeze(0))
        STATE.reset()
        STATE.update(env)
        for _ in range(3):
            base.sim.render()
        recorder.save(env, initial=True)
        demo = Demonstration(env, STATE) if args.policy == "demo" else None
        plan = iter(demo.run()) if demo else None
        print("SCENE READY:", args.policy, flush=True)
        previous_phase = None
        measured_count = 0
        with (args.output/"trace.jsonl").open("w") as trace:
            step = 0
            while app.is_running() and (args.policy == "view" or step < args.max_steps):
                if plan is not None:
                    try:
                        next(plan)
                    except StopIteration:
                        break
                action = demo.action if demo else ready
                env.step(torch.as_tensor(action, dtype=torch.float32, device=base.device).unsqueeze(0))
                STATE.update(env)
                step += 1
                report["steps"] = step
                if args.policy != "view":
                    record = {"step": step, "phase": demo.phase if demo else "hold", "action": action.tolist(),
                              "score": STATE.trial.score, "display_v": STATE.trial.report()["display_v"],
                              "contacts_n": STATE.forces,
                              "bodies": {name: {"pos": body.pos.tolist(), "quat_xyzw": body.quat_xyzw.tolist()}
                                         for name, body in STATE.bodies.items()}}
                    from yam_arena import runtime as yam_runtime
                    record["robot"] = {side: {"tcp": yam_runtime.tcp_world(base.scene[side+"_robot"]).tolist(),
                                               "joint_pos": base.scene[side+"_robot"].data.joint_pos.torch[0].cpu().tolist()}
                                       for side in ("left", "right")}
                    record["robot_contacts_n"] = {side: yam_runtime.contact_forces(base.scene[side+"_contacts"])
                                                  for side in ("left", "right")}
                    trace.write(json.dumps(record)+"\n")
                    recorder.save(env)
                    phase = demo.phase if demo else "hold"
                    if phase != previous_phase or len(STATE.trial.measured) != measured_count:
                        from PIL import Image
                        data = base.scene["top_cam"].data.output["rgb"]
                        image = (data.torch if hasattr(data, "torch") else data)[0, ..., :3].cpu().numpy().astype(np.uint8)
                        Image.fromarray(image).save(args.output/f"checkpoint_{step:04d}.png")
                        previous_phase, measured_count = phase, len(STATE.trial.measured)
                if step % 200 == 0:
                    print(f"STEP {step}: measured={len(STATE.trial.measured)}, score={STATE.trial.score:.2f}", flush=True)
                if STATE.trial.success and args.policy == "demo":
                    break
        report.update(STATE.trial.report())
        recorder.snapshot(env, "final.png")
        report["worst_ik_mm"] = {side: arm.worst_ik_mm for side, arm in demo.arms.items()} if demo else {}
        exit_code = 0 if args.policy != "demo" or STATE.trial.success else 1
    except Exception:
        report.update(STATE.trial.report())
        report["error"] = traceback.format_exc()
        print(report["error"], flush=True)
        if "env" in locals():
            try:
                recorder.snapshot(env, "final.png")
            except Exception:
                pass
    finally:
        try:
            recorder.close()
        except Exception:
            report["video_error"] = traceback.format_exc()
            exit_code = 1
        report["wall_time_s"] = round(time.monotonic()-began, 3)
        report.update(seed=0, control_hz=S.HZ, action_limit=S.MAX_STEPS, simulated_time_s=report["steps"]/S.HZ)
        report["packages"] = {name: metadata.version(name) for name in ("isaacsim", "yam-arena", "inspect-robots", "torch")}
        report["source_sha256"] = initial_digest
        report["source_changed_during_run"] = source_digest() != initial_digest
        if report["source_changed_during_run"] and args.policy == "demo":
            exit_code = 1
        (args.output/"result.json").write_text(json.dumps(report, indent=2)+"\n")
        print("RESULT:", json.dumps({k: report.get(k) for k in ("score", "success", "steps", "wall_time_s", "error")}), flush=True)
        sys.stdout.flush()
        sys.stderr.flush()
        # Kit's close() can swallow exit codes or hang. Encoders and the receipt
        # are finished above; exit explicitly, as the upstream stage checks do.
        os._exit(exit_code)


if __name__ == "__main__":
    main()
