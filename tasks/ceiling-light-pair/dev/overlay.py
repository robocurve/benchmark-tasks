"""dev/overlay.py LOG FRAMES OUT: re-render a demo video in yam-pair-video's layout and encoding, with status lines for
the light and for each station in a black band below the cameras (no image is covered). The status comes from the run's monitor log
(dev/runs/NAME.log.gz or dev/NAME.log): the task's monitor prints the light's and the screws' state every 10 steps, and
each station's script prints its stage and each arm's phase when they change; each value holds until the next. Step 0
is left out, because the overview camera is placed at the reset, after it.

    python dev/overlay.py dev/runs/d1.log.gz dev/logs/d1 media/demo.mp4
"""

from __future__ import annotations

import gzip
import re
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from yam_pair import media

HZ = 20.0
BAND_LINES, LINE_H, BAND_PAD = 6, 21, 8      # the status band below the cameras: up to 6 lines
STAGES = {
    "reach": "taking hold of its end",
    "lift": "lifting the light off its stands",
    "carry toward A": "carrying it toward A: centre holes under the heads",
    "line up": "lining its keyholes up under the heads",
    "raise": "raising it: the heads go up through the centre holes",
    "slide toward A": "sliding it toward A, into its outline",
    "lower onto the heads": "lowering it onto the screw heads",
    "let go": "letting go",
    "hands off": "hands off the light",
}
PHASES = {
    "fetch driver": "takes its screwdriver",
    "to the screw": "brings the screwdriver under its screw",
    "up to the head": "puts the bit in the screw head",
    "push and turn": "pushes: the motor turns the screw",
    "park the driver": "parks its screwdriver, still in hand",
}
SCREW = re.compile(r"(?P<s>[ab]_(?:left|right)): hung (?P<hung>\w+) along (?P<along>[-+\d.]+) across [-+\d.]+ "
                   r"gap (?P<gap>[-+\d.]+) clamp (?P<clamp>[-\d]+) edge (?P<edge>[-+\d.]+) turns (?P<turns>[-\d.]+) "
                   r"tight (?P<tight>\w+)")


def read_log(path: Path):
    opener = gzip.open if path.suffix == ".gz" else open
    clp, stages, phases, events = {}, {"a": [], "b": []}, {}, []
    with opener(path, "rt", errors="replace") as f:
        for line in f:
            if line.startswith("[clp] step "):
                step = int(line.split()[2])
                hz = float(re.search(r" hz ([-+\d.]+)", line).group(1))
                touch = " touch True " in line
                clp[step] = (hz, touch, {m["s"]: m.groupdict() for m in SCREW.finditer(line)})
            elif line.startswith("[clp] event step "):
                m = re.match(r"\[clp\] event step (\d+): (.*)", line)
                events.append((int(m.group(1)), m.group(2)))
            elif line.startswith("[scr] "):
                m = re.match(r"\[scr\] ([ab]) t ([\d.]+) stage (.*)", line)
                if m:
                    stages[m.group(1)].append((float(m.group(2)), m.group(3).strip()))
                m = re.match(r"\[scr\] ([ab]) (left|right) t ([\d.]+) phase (.*)", line)
                if m:
                    phases.setdefault(f"{m.group(1)}_{m.group(2)}", []).append((float(m.group(3)), m.group(4).strip()))
    return clp, stages, phases, events


def last(rows, t):
    rows = [(tt, p) for tt, p in rows if tt <= t + 1e-6]
    return rows[-1][1] if rows else None


def light_text(hz: float, touch: bool, screws: dict, step: int, events) -> str:
    if any(s <= step and t.startswith("dropped") for s, t in events):
        return "Light: dropped"
    n = sum(st["hung"] == "True" for st in screws.values())
    if n == 4 and not touch:
        tight = sum(st["tight"] == "True" for st in screws.values())
        return f"Light: hangs on its four screws, no robot touching it; {tight} of 4 tight"
    if n == 4:
        return "Light: on its four screws (shanks in the narrow slots), a hand on it"
    if hz < -0.028:
        return "Light: on its stands"
    return f"Light: back {-hz * 1000:.0f} mm under the ceiling"


def screw_text(name: str, st: dict) -> str:
    side = name.split("_")[1]
    if st["tight"] == "True":
        return f"{side} screw tight: clamps the back with {int(st['clamp'])} N"
    turns = float(st["turns"])
    if turns > 0.05:
        return f"{side} screw {turns:.1f} turns in, back {max(0.0, float(st['edge'])):.1f} mm off the ceiling"
    if st["hung"] == "True":
        return f"{side} screw in its keyhole's narrow slot"
    return f"{side} screw head 9.5 mm under the ceiling"


def main(log: str, frames: str, out: str) -> None:
    import imageio.v2 as iio

    clp, stages, phases, events = read_log(Path(log))
    trial, cams = media.find_frames(Path(frames))
    steps = [t for t in sorted(set().union(*(set(v) for v in cams.values()))) if t > 0]
    logged = sorted(clp)
    font, small = media._font(14), media._font(16)
    widest = 0.0
    with iio.get_writer(str(out), fps=HZ, codec="libx264", macro_block_size=1, pixelformat="yuv420p",
                        output_params=["-movflags", "+faststart", "-crf", "22"]) as writer:
        for t in steps:
            frames_t = {c: np.load(paths[t]) for c, paths in cams.items() if t in paths}
            grid = media.compose(frames_t, t, HZ, font, True, None)
            prev = [s for s in logged if s <= t]
            hz, touch, screws = clp[prev[-1]] if prev else (-0.03, False, {})
            lines = [light_text(hz, touch, screws, t, events)]
            for s in "ab":
                stage = last(stages[s], t / HZ)
                arm_rows = [(side, last(phases.get(f"{s}_{side}", []), t / HZ)) for side in ("left", "right")]
                if stage == "hands off" and any(p for _, p in arm_rows):
                    lines.append(f"{s.upper()}: " + "; ".join(f"{side} arm {PHASES.get(p, p) if p else 'waits'}"
                                                           for side, p in arm_rows))
                else:
                    lines.append(f"{s.upper()}: {STAGES.get(stage, stage) if stage else 'taking hold of its end'}")
                if screws:
                    lines.append("    " + "; ".join(screw_text(n, screws[n]) for n in (f"{s}_left", f"{s}_right")
                                                    if n in screws))
            # the status lines go in their own black band below the cameras, so they cover no image
            h, w = grid.shape[:2]
            band = BAND_LINES * LINE_H + 2 * BAND_PAD
            im = Image.new("RGB", (w, h + band + (h + band) % 2), (0, 0, 0))
            im.paste(Image.fromarray(grid), (0, 0))
            d = ImageDraw.Draw(im)
            widest = max(widest, max(d.textlength(x, font=small) for x in lines))
            for i, x in enumerate(lines):
                d.text((10, h + BAND_PAD + LINE_H * i), x, font=small, fill=(255, 255, 255))
            writer.append_data(np.asarray(im))
    print(f"{out}: {len(steps)} frames of trial {trial}; widest status line {widest:.0f} px")


if __name__ == "__main__":
    main(*sys.argv[1:4])
