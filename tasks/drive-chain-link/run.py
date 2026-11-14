#!/usr/bin/env python3
"""Record actual native scripted/control runs. This command never calls Dream-1."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import socket
import subprocess
import time
import traceback

import numpy as np

from chain_link.config import INSTRUCTION, MAX_STEPS


def execute(args):
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from chain_link.controller import ScriptedPolicy
    from chain_link.environment import ChainLinkEnvironment, source_fingerprint

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    fingerprint = source_fingerprint()
    receipt = {
        "scope": "actual native PhysX robot execution"
        if not args.no_cameras
        else "actual native CPU PhysX development execution without cameras; no media feasibility claim",
        "status": "starting",
        "hostname": socket.gethostname(),
        "source_sha256": fingerprint,
        "policy": args.policy,
        "seed": args.seed,
        "max_steps": args.max_steps,
        "control_hz": 20,
        "physics_hz": 240,
        "instruction": INSTRUCTION,
        "versions": {
            name: importlib.metadata.version(name)
            for name in (
                "isaacsim",
                "isaaclab",
                "isaaclab-arena",
                "yam-arena",
                "inspect-robots",
            )
        },
        "gpu": subprocess.check_output(
            [
                "nvidia-smi",
                "--query-gpu=index,name,driver_version",
                "--format=csv,noheader",
            ],
            text=True,
        ).splitlines(),
        "dream_api_used": False,
    }
    (output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    environment, writers = None, {}
    start = time.monotonic()
    steps, last = 0, {}
    try:
        environment = ChainLinkEnvironment(
            device=args.device,
            physics_device=args.physics_device,
            render_images=not args.no_cameras,
            cam_width=args.width,
            cam_height=args.height,
        )
        scene = Scene(id="drive-chain-link", instruction=INSTRUCTION)
        observation = environment.reset(scene, seed=args.seed)
        policy = ScriptedPolicy(mode=args.policy)
        policy.bind(environment.info)
        policy.reset(scene)
        receipt["controller"] = {
            "kind": "precalibrated fourteen-joint/gripper program",
            "inputs": ["elapsed time"],
            "program_sha256": policy.program_sha256,
            "runtime_object_state_feedback": False,
        }
        initial = environment.metrics.copy()
        receipt["initial_metrics"] = initial
        if args.record:
            import cv2

            for name, frame in observation.images.items():
                cv2.imwrite(
                    str(output / f"setup-{name}.png"),
                    cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
                )
                h, w = frame.shape[:2]
                writers[name] = subprocess.Popen(
                    [
                        "ffmpeg",
                        "-nostdin",
                        "-loglevel",
                        "error",
                        "-f",
                        "rawvideo",
                        "-pixel_format",
                        "rgb24",
                        "-video_size",
                        f"{w}x{h}",
                        "-framerate",
                        "20",
                        "-i",
                        "pipe:0",
                        "-an",
                        "-c:v",
                        "libx264",
                        "-preset",
                        "fast",
                        "-crf",
                        "23",
                        "-pix_fmt",
                        "yuv420p",
                        "-movflags",
                        "+faststart",
                        str(output / f"{name}.mp4"),
                    ],
                    stdin=subprocess.PIPE,
                )
                writers[name].stdin.write(np.ascontiguousarray(frame).tobytes())
        with (output / "trace.jsonl").open("w") as trace:
            for step in range(args.max_steps):
                action = policy.action_at((step + 1) / 20)
                result = environment.step(Action(data=action))
                steps, last = step + 1, environment.metrics.copy()
                row = dict(
                    step=steps,
                    time_s=steps / 20,
                    phase=policy.phase,
                    action=action.tolist(),
                    metrics=last,
                    native_state=environment.diagnostics(),
                )
                trace.write(json.dumps(row) + "\n")
                if step % 20 == 0:
                    trace.flush()
                    print(
                        json.dumps(
                            dict(
                                step=steps,
                                phase=policy.phase,
                                score=last["score"],
                                ends_joined=last["ends_joined"],
                                retained=last["retained"],
                                violations=last["violations"],
                            )
                        ),
                        flush=True,
                    )
                for name, writer in writers.items():
                    writer.stdin.write(
                        np.ascontiguousarray(result.observation.images[name]).tobytes()
                    )
                if result.terminated or steps / 20 > policy.duration:
                    break
        receipt.update(
            status="completed",
            final_metrics=last,
            steps=steps,
            simulated_s=steps / 20,
        )
    except Exception:
        receipt.update(
            status="error",
            error=traceback.format_exc(),
            final_metrics=last,
            steps=steps,
        )
        raise
    finally:
        video_errors = {}
        for name, writer in writers.items():
            writer.stdin.close()
            code = writer.wait()
            if code:
                video_errors[name] = code
        receipt["video_errors"] = video_errors
        receipt["elapsed_wall_s"] = time.monotonic() - start
        receipt["source_unchanged"] = source_fingerprint() == fingerprint
        receipt["physics_device"] = getattr(environment, "physics_device", None)
        receipt["reset_image_stats"] = getattr(environment, "reset_image_stats", None)
        receipt["reset_diagnostics"] = getattr(environment, "reset_diagnostics", None)
        receipt["artifacts"] = {
            p.name: {
                "bytes": p.stat().st_size,
                "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
            }
            for p in output.iterdir()
            if p.suffix in {".png", ".mp4"}
        }
        (output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
        if environment is not None:
            environment.close()
    print(json.dumps(receipt, indent=2))
    if args.policy == "demo" and not last.get("success"):
        raise SystemExit(
            "Scripted trial did not reach full score; inspect its native trace and video"
        )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--policy",
        choices=["demo", "connector", "hold", "ends", "unlocked"],
        default="demo",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--physics-device", default=None)
    parser.add_argument(
        "--no-cameras",
        action="store_true",
        help="Native physics development only; does not produce submission media",
    )
    parser.add_argument("--max-steps", type=int, default=MAX_STEPS)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=176)
    parser.add_argument("--no-record", dest="record", action="store_false")
    execute(parser.parse_args())
