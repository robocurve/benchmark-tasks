"""Render only the recorded top camera from an inspect-robots run."""
import argparse
import json
from pathlib import Path
import subprocess

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="completed adhoc_*.json evaluation log")
    parser.add_argument("output", type=Path, help="top-view MP4 path")
    parser.add_argument("--fps", type=int, default=20)
    args = parser.parse_args()
    result = json.loads(args.log.read_text())
    frames_dir = Path(result["stats"]["frames_dir"])
    frames = sorted(frames_dir.glob("*top_cam_*.npy"))
    if not frames:
        raise SystemExit(f"no top camera frames in {frames_dir}")
    first = np.load(frames[0])
    if first.dtype != np.uint8 or first.ndim != 3 or first.shape[2] != 3:
        raise SystemExit(f"unexpected frame shape/dtype: {first.shape} {first.dtype}")
    height, width, _ = first.shape
    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = ["ffmpeg", "-y", "-v", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{width}x{height}", "-r", str(args.fps), "-i", "-",
               "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(args.output)]
    with subprocess.Popen(command, stdin=subprocess.PIPE) as encoder:
        for frame_path in frames:
            frame = np.load(frame_path)
            if frame.shape != first.shape or frame.dtype != np.uint8:
                raise ValueError(f"inconsistent top camera frame: {frame_path}")
            encoder.stdin.write(frame.tobytes())
        encoder.stdin.close()
        if encoder.wait() != 0:
            raise SystemExit("ffmpeg failed to render the top view")
    print(f"wrote {args.output} ({len(frames)} top-camera frames)")


if __name__ == "__main__":
    main()
