"""Turn recorded front and wrist camera frames into a synchronized MP4."""
import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np


def label(canvas, text, x, y, size=.47, color=(232, 239, 245)):
    cv2.putText(canvas, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trial", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--title", required=True)
    args = parser.parse_args()
    streams = [sorted(args.trial.glob(f"frames/*/*~{camera}_cam_*.npy"))
               for camera in ("top", "left", "right")]
    if not streams[0] or len({len(stream) for stream in streams}) != 1:
        raise ValueError("Missing or mismatched saved camera streams")
    trace = [json.loads(line) for line in (args.trial / "mechanism.jsonl").read_text().splitlines()]
    if not trace:
        raise ValueError("No scores recorded")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = [
        "ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "rawvideo",
        "-pixel_format", "rgb24", "-video_size", "640x472", "-framerate", "10", "-i", "-",
        "-vf", "tpad=stop_mode=clone:stop_duration=1.0",
        "-c:v", "libx264", "-preset", "fast", "-crf", "27", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(args.output),
    ]
    indices = sorted({*range(0, len(streams[0]), 2), len(streams[0])-1})
    with subprocess.Popen(ffmpeg, stdin=subprocess.PIPE) as encoder:
        for index in indices:
            state = trace[min(index, len(trace) - 1)]
            canvas = np.full((472, 640, 3), (22, 30, 42), dtype=np.uint8)
            top, left, right = (np.load(stream[index])[..., :3] for stream in streams)
            canvas[34:264, 111:529] = cv2.resize(top, (418, 230), interpolation=cv2.INTER_AREA)
            for x, camera in ((0, left), (320, right)):
                canvas[284:444, x:x+320] = cv2.resize(camera, (320, 160), interpolation=cv2.INTER_AREA)
            label(canvas, args.title, 12, 22, .57)
            label(canvas, f"{state['t']:05.2f} s", 565, 22)
            label(canvas, "OVERVIEW", 120, 51)
            label(canvas, "LEFT WRIST", 11, 279)
            label(canvas, "RIGHT WRIST", 331, 279)
            label(canvas, state["caption"], 12, 463)
            label(canvas, f"Score {state['score']:.2f}", 536, 463)
            encoder.stdin.write(canvas.tobytes())
        encoder.stdin.close()
        if encoder.wait():
            raise RuntimeError("ffmpeg could not write a playable recording")


if __name__ == "__main__":
    main()
