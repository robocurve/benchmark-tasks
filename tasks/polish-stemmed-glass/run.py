"""Actual native trials, never hosted inference. Refuses to overwrite results."""

import argparse
import hashlib
import importlib
import json
import shutil
import socket
import subprocess
import time
import traceback
from datetime import UTC, datetime
from pathlib import Path

import numpy as np

PACKAGE = "stemware_polish"


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


def compose(output):
    """Assemble the three native views in timestamp order."""
    sources = [
        arg for view in ("top_cam", "left_cam", "right_cam")
        for arg in ("-i", str(output / (view + ".mp4")))
    ]
    command = [
        "ffmpeg", "-nostdin", "-v", "error", *sources,
        "-filter_complex",
        "[0:v]scale=1280:720[t];[1:v][2:v]hstack=inputs=2[w];[t][w]vstack=inputs=2[out]",
        "-map", "[out]", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "22",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output / "demo.mp4"),
    ]
    subprocess.run(command, check=True)
    shutil.copyfile(output / "setup-top_cam.png", output / "setup.png")


def main():
    import moviepy.config
    c = importlib.import_module(PACKAGE + ".config")
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--device", default="cpu")
    p.add_argument("--physics-device", default="cpu")
    p.add_argument("--no-cameras", action="store_true")
    p.add_argument("--policy", default="demo", choices=("hold", "demo", "air-pass", "short-pass"))
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
        "controller_inputs": ["elapsed time", "joint positions", "glass and cloth pose during polishing and return"],
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
                action = policy.action_at(
                    (k + 1) / 20, obs.state["joint_pos"], env.diagnostics()
                )
                result = env.step(Action(data=action))
                steps = k + 1
                obs = result.observation
                diagnostics = env.diagnostics()
                row = {
                    "step": steps,
                    "phase": policy.phase,
                    "action": action.tolist(),
                    "metrics": plain(env.metrics),
                    "state": plain(diagnostics),
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
        if writers and not errors and receipt["status"] == "completed":
            try:
                compose(a.output)
            except (subprocess.SubprocessError, OSError) as exc:
                errors["composite"] = str(exc)
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
