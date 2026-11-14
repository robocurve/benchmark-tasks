"""Add a velocity strip to the demo film: a bar per note at the bottom edge, its length the note's MIDI velocity, from onset to release (a melody note's until the next melody note), labelled with
the note and its velocity. For the film only: the policy's camera images never carry it.

    python velocity_strip.py demo.log demo.mp4 demo_strip.mp4 [offset_s]

`demo.log` is the demo's printed output ("[keys] note E5 at 3.141 s, velocity 96 ..." and "[keys] E5 up at 3.300 s"); `offset_s` is the film's time of
the key log's zero (the scene's reset), 0 by default.
"""

from __future__ import annotations

import re
import subprocess
import sys

MELODY_KEYS = {"D5", "E5", "F5", "G5"}


def notes(log: str) -> list[tuple[str, float, int, float | None]]:
    out, open_ = [], {}
    for line in open(log, errors="replace"):
        a = re.search(r"\[keys\] note (\S+) at ([\d.]+) s, velocity (\d+)", line)
        b = re.search(r"\[keys\] (\S+) up at ([\d.]+) s", line)
        if a:
            open_[a.group(1)] = len(out)
            out.append([a.group(1), float(a.group(2)), int(a.group(3)), None])
        elif b and b.group(1) in open_:
            out[open_.pop(b.group(1))][3] = float(b.group(2))
    return [tuple(n) for n in out]


def main() -> None:
    log, src, dst = sys.argv[1:4]
    off = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height", "-of", "csv=p=0", src],
                           capture_output=True, text=True, check=True).stdout.strip().split(",")
    w, h = int(probe[0]), int(probe[1])
    strip = 36
    end = 1e9
    f = [f"pad={w}:{h + strip}:0:0:color=0x202020"]
    f.append(f"drawtext=text='key velocity (MIDI 1-127)':x=6:y={h + 4}:fontsize=11:fontcolor=0xa0a0a0")
    lane_w = w - 220
    seq = notes(log)
    mel_on = [n[1] for n in seq if n[0] in MELODY_KEYS]
    for name, on, vel, up in seq:
        melody = name in MELODY_KEYS
        if melody:  # a melody note's bar stays until the next melody note sounds (its key goes up sooner), so a still frame shows its loudness
            later = [t for t in mel_on if t > on]
            up = later[0] if later else (up if up is not None else end)
        t0, t1 = on + off, (up if up is not None else end) + off
        y = h + (6 if melody else 20)
        colour = "0xe05040" if melody else "0x4080e0"
        en = f"enable='between(t,{t0:.3f},{t1:.3f})'"
        f.append(f"drawbox=x=200:y={y}:w={int(lane_w * vel / 127)}:h=11:color={colour}@0.95:t=fill:{en}")
        f.append(f"drawtext=text='{'melody' if melody else 'bass'} {name} {vel}':x={200 + int(lane_w * vel / 127) + 4}:y={y}:fontsize=11:fontcolor=white:{en}")
    subprocess.run(["ffmpeg", "-y", "-v", "error", "-i", src, "-vf", ",".join(f), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart", dst], check=True)


if __name__ == "__main__":
    main()
