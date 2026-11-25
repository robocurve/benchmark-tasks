"""Tile a ``--store-frames`` yam_pair run into one H.264 MP4 (command ``yam-pair-video``).

    yam-pair-video LOG_DIR_or_FRAMES_DIR_or_LOG.json out.mp4 [--every 1] [--no-overview] [--trial ID] [--fps 20]

Layout: a 2x3 grid, station A's top / left wrist / right wrist cameras on the first row and B's on the second, each
labelled; when the run has ``overview_cam`` (``-E overview=true``), the overview sits to the left of the grid, as tall
as it. Frames are read one step at a time, so long runs never need to fit in memory.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

CAMS = ("top_cam", "left_cam", "right_cam")
LABELS = {"top_cam": "top", "left_cam": "left wrist", "right_cam": "right wrist"}
NAME = re.compile(r"^(?P<trial>.+?)_(?P<cam>(?:a|b)_(?:top|left|right)_cam|overview_cam)_(?P<t>\d{6})\.npy$")
TILE = (320, 176)


def find_frames(src: Path, trial: str | None = None) -> tuple[str, dict[str, dict[int, Path]]]:
    """(trial id, {camera: {t: path}}) of the stored frames under ``src`` (a dir, or an eval log JSON's dir)."""
    if src.is_file():
        try:
            data = json.loads(src.read_text())
            hits = [v for k, v in _walk(data) if k == "path" and isinstance(v, str) and v.endswith(".npy")]
            src = Path(hits[0]).parent if hits else src.parent
        except (json.JSONDecodeError, UnicodeDecodeError):
            src = src.parent
    found: dict[str, dict[str, dict[int, Path]]] = defaultdict(lambda: defaultdict(dict))
    for p in src.rglob("*.npy"):
        m = NAME.match(p.name)
        if m:
            found[m["trial"]][m["cam"]][int(m["t"])] = p
    if not found:
        raise SystemExit(f"no yam_pair frames (a_top_cam_000000.npy, ...) under {src}")
    if trial is None:
        if len(found) > 1:
            print(f"several trials {sorted(found)}; using {sorted(found)[0]} (choose with --trial)", file=sys.stderr)
        trial = sorted(found)[0]
    if trial not in found:
        raise SystemExit(f"no trial {trial!r}; have {sorted(found)}")
    return trial, found[trial]


def _walk(obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            yield k, v
            yield from _walk(v)
    elif isinstance(obj, list):
        for v in obj:
            yield from _walk(v)


def _font(size: int):
    from PIL import ImageFont

    for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(path, size)
        except OSError:
            continue
    return ImageFont.load_default()


def label(img: np.ndarray, text: str, font) -> np.ndarray:
    from PIL import Image, ImageDraw

    im = Image.fromarray(img)
    d = ImageDraw.Draw(im, "RGBA")
    w = d.textlength(text, font=font)
    d.rectangle([4, 4, 14 + w, 24], fill=(0, 0, 0, 150))
    d.text((9, 6), text, font=font, fill=(255, 255, 255))
    return np.asarray(im)


def fit(img: np.ndarray, w: int, h: int) -> np.ndarray:
    from PIL import Image

    img = np.asarray(img)[..., :3]
    if img.shape[:2] == (h, w):
        return img
    return np.asarray(Image.fromarray(img).resize((w, h), Image.BICUBIC))


def compose(frames: dict[str, np.ndarray], t: int, hz: float, font, overview: bool = True,
            caption: str | None = None) -> np.ndarray:
    """One video frame from one step's images."""
    w, h = TILE
    rows = []
    for s in ("a", "b"):
        tiles = []
        for c in CAMS:
            img = frames.get(f"{s}_{c}")
            tile = fit(img, w, h) if img is not None else np.zeros((h, w, 3), np.uint8)
            tiles.append(label(tile, f"Robot {s.upper()}: {LABELS[c]}", font))
        rows.append(np.hstack(tiles))
    grid = np.vstack(rows)
    if overview and frames.get("overview_cam") is not None:
        ov = np.asarray(frames["overview_cam"])[..., :3]
        oh = grid.shape[0]
        ow = int(round(ov.shape[1] * oh / ov.shape[0]))
        ow -= ow % 2
        grid = np.hstack([label(fit(ov, ow, oh), "Overview (not seen by either robot)", font), grid])
    text = f"t = {t / hz:5.2f} s   step {t}" + (f"   {caption}" if caption else "")
    grid = label(grid, text, font) if not overview or "overview_cam" not in frames else grid
    if overview and "overview_cam" in frames:
        from PIL import Image, ImageDraw

        im = Image.fromarray(grid)
        d = ImageDraw.Draw(im, "RGBA")
        y = grid.shape[0] - 26
        d.rectangle([4, y, 14 + d.textlength(text, font=font), y + 20], fill=(0, 0, 0, 150))
        d.text((9, y + 2), text, font=font, fill=(255, 255, 255))
        grid = np.asarray(im)
    if grid.shape[0] % 2 or grid.shape[1] % 2:
        grid = grid[: grid.shape[0] // 2 * 2, : grid.shape[1] // 2 * 2]
    return grid


def render(src: str | Path, out: str | Path, *, every: int = 1, overview: bool = True, trial: str | None = None,
           fps: float | None = None, hz: float = 20.0, caption: str | None = None) -> int:
    import imageio.v2 as iio

    trial, cams = find_frames(Path(src), trial)
    steps = sorted(set().union(*(set(v) for v in cams.values())))[:: max(1, int(every))]
    font = _font(14)
    fps = fps or hz / max(1, int(every))
    n = 0
    with iio.get_writer(str(out), fps=fps, codec="libx264", macro_block_size=1,
                        pixelformat="yuv420p", output_params=["-movflags", "+faststart", "-crf", "22"]) as writer:
        for t in steps:
            frames = {c: np.load(paths[t]) for c, paths in cams.items() if t in paths}
            writer.append_data(compose(frames, t, hz, font, overview, caption))
            n += 1
    print(f"{out}: {n} frames of trial {trial}, cameras {sorted(cams)}")
    return n


def main(argv=None) -> None:
    p = argparse.ArgumentParser(prog="yam-pair-video", description=__doc__.split("\n\n")[0])
    p.add_argument("src", help="inspect-robots --log-dir, its frames dir, or an eval log JSON")
    p.add_argument("out", help="output .mp4")
    p.add_argument("--every", type=int, default=1, help="keep every n-th step (and play at 20/n fps)")
    p.add_argument("--no-overview", action="store_true", help="leave the overview camera out")
    p.add_argument("--trial", help="trial id when the frames hold several (e.g. scene-0-e0)")
    p.add_argument("--fps", type=float)
    p.add_argument("--hz", type=float, default=20.0, help="control rate of the run (default 20)")
    p.add_argument("--caption")
    a = p.parse_args(argv)
    render(a.src, a.out, every=a.every, overview=not a.no_overview, trial=a.trial, fps=a.fps, hz=a.hz,
           caption=a.caption)


if __name__ == "__main__":
    main()
