"""Actual native trials, never hosted inference. Refuses to overwrite results."""

import argparse
import hashlib
import importlib
import json
import socket
import subprocess
import time
import traceback
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

PACKAGE = "page_protector"


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def main():
    import moviepy.config  # noqa: F401 - import before Isaac initializes Kit

    c = importlib.import_module(PACKAGE + ".config")
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--device", default="cpu")
    p.add_argument("--physics-device", default="cpu")
    p.add_argument("--no-cameras", action="store_true")
    p.add_argument("--policy", default="demo")
    p.add_argument("--max-steps", type=int, default=c.MAX_STEPS)
    p.add_argument("--seed", type=int, default=1415158438)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=360)
    a = p.parse_args()
    if not 1 <= a.max_steps <= c.MAX_STEPS:
        raise ValueError(f"max-steps must be in [1,{c.MAX_STEPS}]")
    a.output = a.output.resolve()
    a.output.mkdir(parents=True, exist_ok=False)
    from inspect_robots import Scene
    from inspect_robots.types import Action

    c = importlib.import_module(PACKAGE + ".config")
    e = importlib.import_module(PACKAGE + ".environment")
    pc = importlib.import_module(PACKAGE + ".controller")
    source = e.source_fingerprint()
    start = time.monotonic()
    env = None
    writers = {}
    steps = 0
    receipt = {
        "status": "starting",
        "hostname": socket.gethostname(),
        "task": c.KEY,
        "source_sha256": source,
        "seed": a.seed,
        "policy": a.policy,
        "dream_api_used": False,
        "started_at_utc": datetime.now(UTC).isoformat(),
        "control_hz": 20,
        "physics_hz": c.PHYSICS_HZ,
        "instruction": c.INSTRUCTION,
        "controller_inputs": ["elapsed time", "joint positions", "native page link poses"],
        "cameras_enabled": not a.no_cameras,
    }
    (a.output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    try:
        env = e.Environment(
            device=a.device,
            physics_device=a.physics_device,
            render_images=not a.no_cameras,
            cam_width=a.width,
            cam_height=a.height,
        )
        scene = Scene(id=c.KEY, instruction=c.INSTRUCTION)
        obs = env.reset(scene, seed=a.seed)
        receipt["native_asset_sha256"] = {
            s["name"]: hashlib.sha256(Path(s["file"]).read_bytes()).hexdigest()
            for s in env.spec["assets"]
        }
        policy = pc.ScriptedPolicy(mode=a.policy)
        policy.feedback = env.diagnostics
        policy.bind(env.info)
        policy.reset(scene)
        (a.output / "reset.json").write_text(
            json.dumps(plain(env.diagnostics())) + "\n"
        )
        import cv2

        for name, frame in obs.images.items():
            cv2.imwrite(
                str(a.output / ("setup-" + name + ".png")),
                cv2.cvtColor(frame, cv2.COLOR_RGB2BGR),
            )
            h, w = frame.shape[:2]
            writer = subprocess.Popen(
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
                    str(a.output / (name + ".mp4")),
                ],
                stdin=subprocess.PIPE,
            )
            writer.stdin.write(np.ascontiguousarray(frame).tobytes())
            writers[name] = writer
        with (a.output / "trace.jsonl").open("w") as trace:
            for k in range(a.max_steps):
                action = policy.action_at((k + 1) / 20, obs.state["joint_pos"])
                result = env.step(Action(data=action))
                steps = k + 1
                obs = result.observation
                diagnostics = env.diagnostics()
                row = {
                    "step": steps,
                    "phase": policy.phase,
                    "action": action.tolist(),
                    "metrics": plain(env.metrics),
                    "objects": plain(diagnostics["objects"]),
                    "material": plain(diagnostics["material"]),
                    "hands": plain(diagnostics["hands"]),
                }
                trace.write(json.dumps(row) + "\n")
                if k % 20 == 0:
                    trace.flush()
                    print(
                        json.dumps(
                            {
                                "step": steps,
                                "phase": policy.phase,
                                "metrics": plain(env.metrics),
                            }
                        ),
                        flush=True,
                    )
                for name, writer in writers.items():
                    writer.stdin.write(
                        np.ascontiguousarray(result.observation.images[name]).tobytes()
                    )
                if result.terminated or steps / 20 > policy.duration:
                    break
        (a.output / "final.json").write_text(
            json.dumps(plain(env.diagnostics())) + "\n"
        )
        receipt.update(
            status="completed",
            final_metrics=plain(env.metrics),
            steps=steps,
            worst_ik_m=policy.worst_ik_m,
            worst_ik_rad=policy.worst_ik_rad,
        )
    except Exception:
        receipt.update(status="error", steps=steps, error=traceback.format_exc())
        raise
    finally:
        errors = {}
        for name, w in writers.items():
            w.stdin.close()
            rc = w.wait()
            if rc:
                errors[name] = rc
        receipt.update(
            elapsed_wall_s=time.monotonic() - start,
            source_unchanged=e.source_fingerprint() == source,
            video_errors=errors,
        )
        receipt["media"] = {
            p.name: {
                "bytes": p.stat().st_size,
                "sha256": hashlib.sha256(p.read_bytes()).hexdigest(),
            }
            for p in a.output.iterdir()
            if p.suffix in (".mp4", ".png")
        }
        (a.output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
        if env:
            env.close()
    print(json.dumps(receipt, indent=2), flush=True)


if __name__ == "__main__":
    main()
