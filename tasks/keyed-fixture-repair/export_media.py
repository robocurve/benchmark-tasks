"""Export the actual YAM camera streams, setup still, and scored trace summary."""

import argparse
import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

CAMERAS = ("top_cam", "left_cam", "right_cam")
ACTION_LIMIT = 1800


def read_trial(folder, policy):
    (report,) = [p for p in folder.glob("adhoc_*.json") if not p.name.endswith(".live.json")]
    trial = json.loads(report.read_text())
    revision = hashlib.sha256(Path(__file__).with_name("task.py").read_bytes()).hexdigest()
    if (trial["status"] != "success" or trial["results"]["errored_trials"]
            or trial["eval"]["policy"] != policy
            or trial["eval"]["environment_revision"] != revision[:12]
            or trial["eval"]["seed"] != 0 or trial["eval"]["max_steps"] != ACTION_LIMIT):
        raise ValueError(f"{folder}: expected a completed {policy} trial on this task at seed 0")
    score = trial["results"]["metrics"]["task_score"]
    if policy == "yam_scripted" and score != 1:
        raise ValueError(f"{folder}: scripted trial must reach full credit")
    if trial["stats"]["total_steps"] > ACTION_LIMIT:
        raise ValueError(f"{folder}: trial exceeded the action limit")
    rows = [json.loads(line) for line in (folder / "proof-1.jsonl").read_text().splitlines()]
    streams = [sorted(folder.glob(f"frames/**/*_{cam}_*.npy")) for cam in CAMERAS]
    if (len(rows) != trial["stats"]["total_steps"] or rows[-1]["score"] != score
            or any(len(stream) != len(rows) + 1 for stream in streams)):
        raise ValueError(f"{folder}: frame, trace, or score mismatch")
    return trial, streams, [None, *rows]


def frame(paths, row):
    top = cv2.resize(np.load(paths[0]), (640, 360))
    wrists = [cv2.resize(np.load(path), (320, 180)) for path in paths[1:]]
    bar = np.full((32, 960, 3), (21, 27, 32), dtype=np.uint8)
    message = "Guide verification"
    if row:
        message += f"  |  Score {row['score']:.2f}"
        if row["reading"]:
            orient, result = row["reading"]
            message += f"  |  {orient}: {result}, {row['height_above_seat_mm']:+.1f} mm above seat"
        if row["verified"]:
            message += f"  |  Replacement checks {len(row['verified'])}/2"
    cv2.putText(bar, message, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, .55, (240, 245, 248),
                1, cv2.LINE_AA)
    return np.vstack((np.hstack((top, np.vstack(wrists))), bar))


def encode(ffmpeg, streams, rows, path):
    args = [ffmpeg, "-nostdin", "-v", "error", "-y", "-f", "rawvideo",
            "-pixel_format", "rgb24", "-video_size", "960x392", "-framerate", "20",
            "-i", "-", "-an", "-c:v", "libx264", "-crf", "20", "-pix_fmt", "yuv420p",
            "-movflags", "+faststart", str(path)]
    with subprocess.Popen(args, stdin=subprocess.PIPE) as process:
        try:
            for paths, row in zip(zip(*streams, strict=True), rows, strict=True):
                process.stdin.write(frame(paths, row).tobytes())
        finally:
            process.stdin.close()
        if process.wait():
            raise RuntimeError(f"Unable to encode {path}")
    subprocess.run([ffmpeg, "-nostdin", "-v", "error", "-i", str(path), "-f", "null", "-"],
                   check=True)
    if path.stat().st_size > 50_000_000:
        raise ValueError(f"{path}: video exceeds 50 MB")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", required=True, type=Path)
    parser.add_argument("--dream", type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--ffmpeg", default=shutil.which("ffmpeg"))
    args = parser.parse_args()
    if not args.ffmpeg:
        parser.error("Install ffmpeg or pass --ffmpeg /path/to/ffmpeg.")
    args.out.mkdir(parents=True, exist_ok=True)
    summary = {}
    for folder, policy, name in (
        (args.demo, "yam_scripted", "demo"), (args.dream, "dream", "model_run")
    ):
        if folder is None:
            continue
        trial, streams, rows = read_trial(folder, policy)
        if policy == "yam_scripted":
            top = cv2.cvtColor(np.load(streams[0][0]), cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(args.out / "setup.png"), top):
                raise RuntimeError("Unable to save setup image")
        encode(args.ffmpeg, streams, rows, args.out / f"{name}.mp4")
        changes = [row for i, row in enumerate(rows[1:], 1)
                   if i == 1 or row["score"] != rows[i - 1]["score"]]
        summary[name] = {
            "score": trial["results"]["metrics"]["task_score"],
            "steps": trial["stats"]["total_steps"],
            "wall_seconds": trial["stats"]["duration_s"],
            "milestones": [{"time_s": row["t"], "score": row["score"]} for row in changes],
        }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
