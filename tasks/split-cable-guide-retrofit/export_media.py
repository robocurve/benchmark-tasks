"""Export blind-review media from completed, frame-recorded Arena trials."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np


def trial(directory, policy):
    (log,) = sorted(directory.glob("adhoc_*.json"))
    data = json.loads(log.read_text())
    revision = hashlib.sha256(
        Path(__file__).with_name("task.py").read_bytes()
    ).hexdigest()
    if (
        data["status"] != "success"
        or data["error"]
        or data["results"]["errored_trials"]
    ):
        raise ValueError(f"{log}: run did not complete successfully")
    if data["eval"]["environment_revision"] != revision[:12]:
        raise ValueError(f"{log}: trial used a different task revision")
    if data["eval"]["policy"] != policy:
        raise ValueError(f"{log}: expected policy {policy}")
    if data["eval"]["max_steps"] != 800 or data["stats"]["total_steps"] > 800:
        raise ValueError(f"{log}: wrong run limit")
    frames = sorted(directory.glob("frames/**/*top_cam_*.npy"))
    if len(frames) < 2:
        raise ValueError(f"{log}: top-camera recording missing")
    return data, frames


def encode(frames, output):
    height, width = np.load(frames[0]).shape[:2]
    command = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "error",
        "-y",
        "-f",
        "rawvideo",
        "-pixel_format",
        "rgb24",
        "-video_size",
        f"{width}x{height}",
        "-framerate",
        "20",
        "-i",
        "-",
        "-an",
        "-c:v",
        "libx264",
        "-crf",
        "22",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output),
    ]
    with subprocess.Popen(command, stdin=subprocess.PIPE) as process:
        for frame in frames:
            process.stdin.write(np.load(frame).tobytes())
        process.stdin.close()
        if process.wait():
            raise RuntimeError(f"Encoding failed for {output}")
    subprocess.run(
        ["ffmpeg", "-v", "error", "-i", str(output), "-f", "null", "-"], check=True
    )
    if output.stat().st_size > 50_000_000:
        raise ValueError(f"{output}: video exceeds 50 MB")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", type=Path, required=True)
    parser.add_argument("--dream", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    demo, demo_frames = trial(args.demo, "yam_scripted")
    dream, dream_frames = trial(args.dream, "dream")
    if demo["results"]["metrics"]["task_score"] != 1.0:
        raise ValueError("Full-score scripted run required")
    args.out.mkdir(parents=True, exist_ok=True)
    encode(demo_frames, args.out / "demo.mp4")
    encode(dream_frames, args.out / "model_run.mp4")
    setup = cv2.cvtColor(np.load(demo_frames[0]), cv2.COLOR_RGB2BGR)
    if not cv2.imwrite(str(args.out / "setup.png"), setup):
        raise RuntimeError("Unable to write setup screenshot")
    summary = {
        name: {
            "metrics": log["results"]["metrics"],
            "steps": log["stats"]["total_steps"],
            "wall_seconds": log["stats"]["duration_s"],
        }
        for name, log in (("demo", demo), ("dream", dream))
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
