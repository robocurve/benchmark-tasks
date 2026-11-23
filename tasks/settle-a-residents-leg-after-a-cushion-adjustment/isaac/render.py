#!/usr/bin/env python3
"""Convert YAM Inspect Robots saved RGB .npy frames into uploadable H.264 videos.

Works with a native run folder or with its ZIP archive. No simulator required.
Does not alter task source, score traces, or image arrays.

"""

from __future__ import annotations

import argparse
import io
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import numpy as np

VIEWS = ("top_cam", "left_cam", "right_cam")
FRAME_RE = re.compile(r"~(?P<view>top_cam|left_cam|right_cam)_(?P<index>\d{6})\.npy$")


def source_frames(run: Path | None, archive: Path | None):
    if archive:
        z = zipfile.ZipFile(archive)
        names = [n for n in z.namelist() if "/frames/" in n and n.endswith(".npy")]

        def loader(name):
            return np.load(io.BytesIO(z.read(name)), allow_pickle=False)
    else:
        z = None
        if not run or not run.exists():
            raise FileNotFoundError(f"Run does not exist: {run}")
        names = [str(n) for n in run.glob("frames/*/*.npy")]

        def loader(name):
            return np.load(name, allow_pickle=False)

    files = {v: {} for v in VIEWS}
    for name in names:
        match = FRAME_RE.search(name)
        if match:
            view = match.group("view")
            i = int(match.group("index"))
            if i in files[view]:
                raise ValueError(f"Duplicate frame {view}:{i}")
            files[view][i] = name
    keys = sorted(files["top_cam"])
    if not keys or keys != list(range(keys[0], keys[-1] + 1)):
        raise ValueError("Top camera frames missing or not contiguous")
    if keys[0] != 0:
        raise ValueError("First frame index is not zero")
    for view in VIEWS:
        if sorted(files[view]) != keys:
            raise ValueError(f"{view} frame indices do not match top camera")
    return files, keys, loader, z


def validate_frame(array, shape):
    if array.dtype != np.uint8 or array.ndim != 3 or array.shape[2] != 3:
        raise ValueError(
            f"Frame must be RGB uint8 HxWx3, got {array.shape} {array.dtype}"
        )
    if shape is not None and array.shape != shape:
        raise ValueError(f"Inconsistent frame shape: {array.shape}, expected {shape}")
    return np.ascontiguousarray(array)


def ffmpeg_encode(output, files, keys, loader, mode, fps):
    initial = validate_frame(loader(files["top_cam"][keys[0]]), None)
    height, width = initial.shape[:2]
    if mode == "three":
        out_width = 3 * width
    elif mode == "top":
        out_width = width
    else:
        raise ValueError(mode)
    if out_width % 2 or height % 2:
        raise ValueError("H.264 yuv420p requires even width and height")
    temp = output.with_name(output.stem + ".inprogress.mp4")
    cmd = [
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
        f"{out_width}x{height}",
        "-framerate",
        str(fps),
        "-i",
        "pipe:0",
        "-an",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "20",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(temp),
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        for j, k in enumerate(keys):
            top = validate_frame(loader(files["top_cam"][k]), initial.shape)
            if mode == "top":
                rgb = top
            else:
                left = validate_frame(loader(files["left_cam"][k]), initial.shape)
                right = validate_frame(loader(files["right_cam"][k]), initial.shape)
                rgb = np.concatenate((top, left, right), axis=1)
            proc.stdin.write(rgb.tobytes())
            if (j + 1) % 200 == 0:
                print(f"  encoded {j + 1}/{len(keys)} {mode} frames", flush=True)
    except Exception:
        if proc.stdin:
            try:
                proc.stdin.close()
            except BrokenPipeError:
                pass
        proc.kill()
        proc.communicate()
        temp.unlink(missing_ok=True)
        raise
    proc.stdin.close()
    proc.stdin = None
    _, err = proc.communicate()
    if proc.returncode:
        temp.unlink(missing_ok=True)
        raise RuntimeError(f"ffmpeg failed: {err.decode(errors='replace')[-1500:]}")
    temp.replace(output)
    print(
        f"  {output.name}: {len(keys)} frames, {len(keys) / fps:.2f}s, {output.stat().st_size / 1024**2:.2f} MiB",
        flush=True,
    )


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    inp = p.add_mutually_exclusive_group(required=True)
    inp.add_argument("--run", type=Path, help="Native run directory containing frames/")
    inp.add_argument(
        "--archive", type=Path, help="ZIP containing one native run with saved frames"
    )
    p.add_argument(
        "--output", type=Path, required=True, help="Separate media output folder"
    )
    p.add_argument("--fps", type=int, default=20)
    p.add_argument("--mode", choices=("demo", "model"), default="demo")
    p.add_argument(
        "--no-triptych", action="store_true", help="Only encode top camera video"
    )
    args = p.parse_args(argv)
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required (sudo apt install ffmpeg)")
    if args.fps <= 0:
        raise ValueError("fps must be positive")
    files, keys, loader, z = source_frames(args.run, args.archive)
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    mode = args.mode
    first = validate_frame(loader(files["top_cam"][keys[0]]), None)
    # Native setup screenshot, direct PNG frame conversion; no synthetic rendering.
    setup = out / "setup.png"
    cmd = [
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
        f"{first.shape[1]}x{first.shape[0]}",
        "-i",
        "pipe:0",
        "-frames:v",
        "1",
        str(setup),
    ]
    proc = subprocess.run(cmd, input=first.tobytes(), capture_output=True)
    if proc.returncode:
        raise RuntimeError(
            "Screenshot encoding failed: " + proc.stderr.decode(errors="replace")
        )
    print(f"Encoded native setup frame: {setup}")
    ffmpeg_encode(out / (mode + ".mp4"), files, keys, loader, "top", args.fps)
    if not args.no_triptych:
        ffmpeg_encode(
            out / "review_three_cameras.mp4", files, keys, loader, "three", args.fps
        )
    if z:
        z.close()
    print("Done. These media files were rendered from real saved camera arrays.")


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, FileNotFoundError, OSError) as exc:
        print("VIDEO_RENDER_FAILED:", exc, file=sys.stderr)
        sys.exit(2)
