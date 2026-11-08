"""Add factual phase/time/load labels to a separate copy of review.mp4.

Usage: python render_review.py RUN_DIR --output review-labelled.mp4

Requires FFmpeg with libass (or imageio-ffmpeg) and Arial or DejaVu Sans.
Every source frame keeps its original 20 Hz timestamp; only text is overlaid.
No score is displayed. Labels describe recorded telemetry, not task success.
"""

from __future__ import annotations

import argparse
from collections import Counter
import gzip
import hashlib
import json
import lzma
import math
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


FPS, WIDTH, HEIGHT = 20, 1600, 1280
PHASES = {"policy": "Robot actions", "seating": "Evaluator: seating ramp",
          "proof": "Evaluator: load hold"}


def read_labels(path):
    """Read only label fields; reject missing, conflicting or mistimed data."""
    opener = gzip.open if path.suffix == ".gz" else lzma.open if path.suffix == ".xz" else open
    labels = []
    phase_index = 0
    with opener(path, "rt") as stream:
        for index, line in enumerate(stream):
            sample = json.loads(line)
            stamp = float(sample["time"])
            if not math.isfinite(stamp) or abs(stamp - index / FPS) > 1e-6:
                raise ValueError(f"trajectory frame {index}: expected time {index / FPS:.2f} s")
            phase = sample.get("phase")
            if phase not in PHASES:
                raise ValueError(f"trajectory frame {index}: missing or unknown phase")
            order = tuple(PHASES).index(phase)
            if order < phase_index or order > phase_index + 1 or (index == 0 and order != 0):
                raise ValueError(f"trajectory frame {index}: invalid phase sequence")
            phase_index = order
            detail = f"Elapsed time: {stamp:.2f} s"
            if phase != "policy":
                proof = sample.get("proof", {})
                if proof.get("active") is not True or proof.get("source") != "simulator_applied_external_force":
                    raise ValueError(f"trajectory frame {index}: applied-load readback missing")
                force = float(proof["applied_force_n"])
                vectors = proof["endpoint_forces_n"]
                if len(vectors) != 2 or any(len(v) != 3 for v in vectors):
                    raise ValueError(f"trajectory frame {index}: invalid endpoint-force vectors")
                values = [float(v) for vector in vectors for v in vector]
                if not all(math.isfinite(v) for v in [force, *values]) or force < 0:
                    raise ValueError(f"trajectory frame {index}: non-finite or negative load")
                magnitudes = [math.sqrt(sum(float(v) ** 2 for v in vector)) for vector in vectors]
                if abs(force - min(magnitudes)) > 1e-6:
                    raise ValueError(f"trajectory frame {index}: scalar load contradicts force vectors")
                detail += f" | Applied load: {force:.3f} N"
            elif sample.get("proof", {}).get("active"):
                raise ValueError(f"trajectory frame {index}: active load labelled as robot actions")
            labels.append((phase, PHASES[phase] + r"\N" + detail))
    if not labels:
        raise ValueError("trajectory is empty")
    return labels


def ass_time(frame):
    # Twenty Hz is exactly representable by ASS's centisecond clock.
    centiseconds = frame * 5
    hours, remainder = divmod(centiseconds, 360000)
    minutes, remainder = divmod(remainder, 6000)
    seconds, fraction = divmod(remainder, 100)
    return f"{hours}:{minutes:02}:{seconds:02}.{fraction:02}"


def write_subtitles(path, labels, font_family):
    header = f"""[Script Info]
ScriptType: v4.00+
PlayResX: {WIDTH}
PlayResY: {HEIGHT}
WrapStyle: 2
[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Label,{font_family},28,&H00FFFFFF,&H00FFFFFF,&H80000000,&H80000000,0,0,0,0,100,100,0,0,3,8,0,7,24,24,20,1
[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""
    events = [f"Dialogue: 0,{ass_time(i)},{ass_time(i+1)},Label,,0,0,0,,{text}"
              for i, (_, text) in enumerate(labels)]
    path.write_text(header + "\n".join(events) + "\n", encoding="utf-8")


def video_frames(ffmpeg, path):
    """Decode the whole video and check each displayed frame's PTS and size."""
    result = subprocess.run([
        ffmpeg, "-hide_banner", "-loglevel", "info", "-i", str(path),
        "-map", "0:v:0", "-an", "-vf", "showinfo", "-fps_mode", "passthrough",
        "-f", "null", "-",
    ], capture_output=True, text=True, check=True)
    pattern = re.compile(r"\bn:\s*(\d+)\s+pts:\s*-?\d+\s+pts_time:([\d.eE+-]+).*?\bs:(\d+)x(\d+)")
    count = 0
    for match in pattern.finditer(result.stderr):
        index, stamp, width, height = match.groups()
        if int(index) != count or not math.isfinite(float(stamp)) or abs(float(stamp) - count / FPS) > 1e-6:
            raise ValueError("video timestamps must start at zero and advance exactly at 20 Hz")
        if (int(width), int(height)) != (WIDTH, HEIGHT):
            raise ValueError("review video must be 1600x1280")
        count += 1
    if count == 0:
        raise ValueError("no decoded video frames")
    return count


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--trajectory", type=Path, help="Defaults to RUN_DIR/trajectory.jsonl")
    parser.add_argument("--ffmpeg", default="ffmpeg")
    args = parser.parse_args()
    source = (args.run_dir / "review.mp4").resolve()
    trajectory = (args.trajectory or args.run_dir / "trajectory.jsonl").resolve()
    output = args.output.resolve()
    manifest = output.with_suffix(".json")
    if output.suffix.lower() != ".mp4" or output == source or output.exists() or manifest.exists():
        parser.error("choose a fresh .mp4 output path and matching .json manifest path")
    if not source.is_file() or not trajectory.is_file():
        parser.error("review.mp4 or trajectory is missing")
    ffmpeg = shutil.which(args.ffmpeg)
    if ffmpeg is None:
        if args.ffmpeg != "ffmpeg":
            parser.error("specified FFmpeg executable was not found")
        try:
            import imageio_ffmpeg
            ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
        except ImportError:
            parser.error("install FFmpeg or imageio-ffmpeg")
    fonts = (("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "DejaVu Sans"),
             ("/System/Library/Fonts/Supplemental/Arial.ttf", "Arial"),
             ("/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf", "Liberation Sans"))
    font = next(((Path(path), family) for path, family in fonts if Path(path).is_file()), None)
    if font is None:
        parser.error("install DejaVu Sans or Arial to render consistent readable labels")
    labels = read_labels(trajectory)
    source_count = video_frames(ffmpeg, source)
    if source_count != len(labels):
        raise ValueError(f"source video has {source_count} frames but trajectory has {len(labels)} samples")
    source_hashes = {"review.mp4": digest(source), "trajectory": digest(trajectory)}
    output.parent.mkdir(parents=True, exist_ok=True)
    # Controlled relative filter paths avoid FFmpeg escaping of user paths.
    # Render into a temporary file; publish only after complete validation.
    with tempfile.TemporaryDirectory(prefix="rope-review-", dir=output.parent) as temp:
        work = Path(temp)
        shutil.copyfile(font[0], work / "review-font.ttf")
        write_subtitles(work / "labels.ass", labels, font[1])
        rendered = work / "labelled.mp4"
        subprocess.run([
            ffmpeg, "-hide_banner", "-loglevel", "error", "-n", "-copyts",
            "-i", str(source), "-map", "0:v:0", "-an", "-map_metadata", "-1",
            "-map_chapters", "-1", "-vf", "ass=labels.ass:fontsdir=.",
            "-fps_mode", "passthrough", "-c:v", "libx264", "-preset", "medium",
            "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(rendered),
        ], cwd=work, check=True)
        count = video_frames(ffmpeg, rendered)
        if count != len(labels):
            raise ValueError("labelled video lost or added frames")
        if source_hashes != {"review.mp4": digest(source), "trajectory": digest(trajectory)}:
            raise ValueError("source recording changed during rendering")
        report = {"source_sha256": source_hashes, "output_sha256": digest(rendered),
                  "output_bytes": rendered.stat().st_size, "frames": count, "fps": FPS,
                  "width": WIDTH, "height": HEIGHT, "first_time_s": 0.,
                  "last_time_s": (count-1)/FPS, "phase_frames": dict(Counter(p for p, _ in labels)),
                  "overlay": "recorded phase, elapsed simulation time, applied load during assay",
                  "score_displayed": False, "interpolated_frames": False,
                  "source_and_output_timestamps_verified": True}
        # Exclusive creation also protects against a competing writer.
        with output.open("xb") as target, rendered.open("rb") as stream:
            shutil.copyfileobj(stream, target)
        with manifest.open("x", encoding="utf-8") as stream:
            stream.write(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
