"""Export the initial setup image and both recorded policy trials for review."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np

CAMERAS = ("top_cam", "left_cam", "right_cam")


def load_trial(folder, policy):
    (log,) = folder.glob("adhoc_*.json")
    run = json.loads(log.read_text())
    task_hash = hashlib.sha256(Path(__file__).with_name("task.py").read_bytes()).hexdigest()
    if (
        run["status"] != "success"
        or run["results"]["errored_trials"]
        or run["eval"]["policy"] != policy
        or run["eval"]["environment_revision"] != task_hash[:12]
        or run["eval"]["max_steps"] != 600
    ):
        raise ValueError(f"{folder}: expected a completed {policy} trial of this task")
    score = run["results"]["metrics"]["task_score"]
    if policy == "yam_scripted" and score != 1.0:
        raise ValueError("A full-score demo is required")
    trace = [json.loads(line) for line in (folder / "proof-1.jsonl").read_text().splitlines()]
    streams = [sorted(folder.glob(f"frames/**/*_{cam}_*.npy")) for cam in CAMERAS]
    if len(trace) != run["stats"]["total_steps"] or any(
        len(stream) != len(trace) + 1 for stream in streams
    ):
        raise ValueError(f"{folder}: expected an initial image and one frame per step")
    if trace[-1]["score"] != score:
        raise ValueError(f"{folder}: score disagrees with trace")
    return run, streams, [None, *trace]


def compose(paths, row):
    top = cv2.resize(np.load(paths[0]), (640, 360))
    wrists = [cv2.resize(np.load(path), (320, 180)) for path in paths[1:]]
    frame = np.hstack((top, np.vstack(wrists)))
    cv2.rectangle(frame, (0, 327), (640, 359), (24, 28, 34), -1)
    message = "Target depth 10 mm"
    if row and row["proof_force_n"] > 0:
        message += f"  |  Probe {row['depth_m'] * 1000:.1f} mm  |  Load {row['proof_force_n']:.1f} N"
    if row and row["proof_depths_m"]:
        readings = ", ".join(f"{depth * 1000:.2f}" for depth in row["proof_depths_m"])
        if row["proof_force_n"] == 0:
            message += f"  |  Verified {readings} mm  |  Score {row['score']:.1f}"
    cv2.putText(frame, message, (12, 349), cv2.FONT_HERSHEY_SIMPLEX,
                0.56, (244, 247, 250), 1, cv2.LINE_AA)
    return frame


def encode(streams, trace, output):
    args = [
        "ffmpeg", "-nostdin", "-loglevel", "error", "-y",
        "-f", "rawvideo", "-pixel_format", "rgb24", "-video_size", "960x360",
        "-framerate", "20", "-i", "-", "-an", "-c:v", "libx264", "-crf", "21",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output),
    ]
    with subprocess.Popen(args, stdin=subprocess.PIPE) as process:
        final_frame = None
        for paths, row in zip(zip(*streams, strict=True), trace, strict=True):
            final_frame = compose(paths, row).tobytes()
            process.stdin.write(final_frame)
        # Keep the recorded final state on screen long enough to read the result.
        if final_frame is not None:
            process.stdin.write(final_frame * 30)
        process.stdin.close()
        if process.wait() != 0:
            raise RuntimeError(f"Encoding failed for {output}")
    subprocess.run(["ffmpeg", "-nostdin", "-v", "error", "-i", str(output),
                    "-f", "null", "-"], check=True)
    if output.stat().st_size > 50_000_000:
        raise ValueError(f"{output}: video exceeds 50 MB")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", required=True, type=Path)
    parser.add_argument("--dream", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    summary = {}
    for policy, folder, name in (
        ("yam_scripted", args.demo, "demo"),
        ("dream", args.dream, "model_run"),
    ):
        run, streams, trace = load_trial(folder, policy)
        if policy == "yam_scripted":
            setup = cv2.cvtColor(np.load(streams[0][0]), cv2.COLOR_RGB2BGR)
            if not cv2.imwrite(str(args.out / "setup.png"), setup):
                raise RuntimeError("Unable to write setup screenshot")
        encode(streams, trace, args.out / f"{name}.mp4")
        summary[name] = {
            "score": run["results"]["metrics"]["task_score"],
            "steps": run["stats"]["total_steps"],
            "wall_seconds": run["stats"]["duration_s"],
            "proof_depths_mm": [depth * 1000 for depth in trace[-1]["proof_depths_m"]],
            "proof_peaks_mm": [depth * 1000 for depth in trace[-1]["proof_peaks_m"]],
        }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
