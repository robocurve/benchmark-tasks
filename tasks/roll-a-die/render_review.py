"""Combine the complete review recording and three original policy camera videos.

First export policy streams with ``inspect-robots video EVAL.json --out RUN/media
--fps 20``. This script keeps every recorded frame in order, without generating
or interpolating frames. The close-up camera is never a Dream observation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    args = parser.parse_args()
    ffmpeg = shutil.which(args.ffmpeg)
    if ffmpeg is None:
        try:
            import imageio_ffmpeg
            ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        except ImportError:
            parser.error("install FFmpeg or pass --ffmpeg /path/to/ffmpeg")
    inputs = [args.run_dir / "review.mp4"]
    for camera in ("top_cam", "left_cam", "right_cam"):
        candidates = list((args.run_dir / "media").glob(f"*_{camera}.mp4"))
        if len(candidates) != 1:
            parser.error(f"expected one {camera} video in RUN_DIR/media; export it at 20 fps first")
        inputs.extend(candidates)
    if any(not path.is_file() for path in inputs):
        parser.error("missing review.mp4")
    if args.output.exists():
        parser.error("choose a fresh output path")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Explicit installed fonts also work when a bundled FFmpeg has no
    # Fontconfig configuration (common on macOS).
    font = next((p for p in (
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
        Path("/System/Library/Fonts/Supplemental/Arial.ttf"),
        Path("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"),
    ) if p.is_file()), None)
    filters = ["[0:v]pad=2240:1280:0:0:color=0x15191e[base]"]
    for i in range(1, 4):
        filters.append(f"[{i}:v]scale=640:352:flags=lanczos[cam{i}]")
    filters += [
        "[base][cam1]overlay=1600:64:shortest=1[a]",
        "[a][cam2]overlay=1600:448:shortest=1[b]",
        "[b][cam3]overlay=1600:832:shortest=1[c]",
        "[c]drawtext=text='REVIEW CLOSE-UP':x=24:y=24:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.6,"
        "drawtext=text='POLICY OVERHEAD':x=1620:y=24:fontsize=24:fontcolor=white,"
        "drawtext=text='POLICY LEFT WRIST':x=1620:y=416:fontsize=24:fontcolor=white,"
        "drawtext=text='POLICY RIGHT WRIST':x=1620:y=800:fontsize=24:fontcolor=white,"
        r"drawtext=text='SIM TIME %{pts\:hms}':x=1620:y=1210:fontsize=24:fontcolor=white[out]",
    ]
    if font is not None:
        filters = [f.replace("drawtext=", f"drawtext=fontfile='{font}':") for f in filters]
    command = [ffmpeg, "-hide_banner", "-loglevel", "error", "-n"]
    for path in inputs:
        command.extend(["-i", str(path)])
    command.extend([
        "-filter_complex", ";".join(filters), "-map", "[out]", "-an",
        "-c:v", "libx264", "-preset", "medium", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(args.output),
    ])
    subprocess.run(command, check=True)
    metadata = json.loads((args.run_dir / "review-video.json").read_text())
    decoded = subprocess.run([
        ffmpeg, "-hide_banner", "-loglevel", "error", "-i", str(args.output),
        "-map", "0:v:0", "-an", "-f", "null", "-", "-progress", "pipe:1", "-nostats",
    ], capture_output=True, text=True, check=True)
    counts = [int(line.split("=", 1)[1]) for line in decoded.stdout.splitlines() if line.startswith("frame=")]
    if not counts or counts[-1] != metadata["frames"]:
        raise RuntimeError("composite frame count differs from the complete review recording")
    metadata.update({
        "layout": "1600x1280 review plus three 640x352 original policy views",
        "input_sha256": {str(p.relative_to(args.run_dir)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
        "output_sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
        "output_bytes": args.output.stat().st_size,
        "interpolated_frames": False,
        "verified_decoded_frames": counts[-1],
    })
    args.output.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
