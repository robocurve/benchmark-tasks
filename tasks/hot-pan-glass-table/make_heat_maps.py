"""Write this task's own images into tex/: the glow of the hot pan's metal and the hob's LED display. Pure numpy and Pillow; our own images.

Usage: python make_heat_maps.py
"""

from __future__ import annotations

import pathlib

import numpy as np
from PIL import Image

TEX = pathlib.Path(__file__).resolve().parent / "tex"

# A plain 5 x 7 LED font, our own drawing.
GLYPHS = {
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "o": ["00000", "00000", "01110", "10001", "10001", "10001", "01110"],
    "t": ["01000", "01000", "11110", "01000", "01000", "01001", "00110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "8": ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "3": ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    "0": ["01110", "10011", "10101", "10101", "10101", "11001", "01110"],
    "C": ["01110", "10001", "10000", "10000", "10000", "10001", "01110"],
    "d": ["01100", "10010", "10010", "01100", "00000", "00000", "00000"],  # the degree sign
    " ": ["00000"] * 7,
}


def glow(path: pathlib.Path, hot_at_top: bool, mottle: float) -> None:
    """Metal glowing with heat: white-orange at the hottest end, through red, to nothing. Mottled, and seamless round the pan (image x)."""
    w, h = 256, 256
    u = np.linspace(0.0, 2 * np.pi, w, endpoint=False)[None, :]
    hot = np.linspace(1.0, 0.0, h)[:, None] if hot_at_top else np.linspace(0.0, 1.0, h)[:, None]
    wave = mottle * (0.05 * np.sin(u * 3 + 1.0) + 0.04 * np.sin(u * 7 + 2.3 + 3 * hot) + 0.03 * np.sin(u * 13 + 0.7 - 4 * hot))
    t = np.clip(hot + wave * (0.4 + hot), 0.0, 1.0)
    stops = [(0.0, (0.0, 0.0, 0.0)), (0.30, (0.10, 0.005, 0.0)), (0.55, (0.55, 0.05, 0.0)), (0.80, (1.0, 0.24, 0.015)), (1.0, (1.0, 0.55, 0.10))]
    rgb = np.stack([np.interp(t, [s[0] for s in stops], [s[1][c] for s in stops]) for c in range(3)], axis=-1)
    Image.fromarray((rgb * 255).astype(np.uint8)).save(path, quality=90)


def display(path: pathlib.Path, lines: list[str], colour: tuple[float, float, float]) -> None:
    """Text on a dark screen, each line centred, 512 x 196 px: 104 mm x 40 mm on the panel."""
    w, h = 512, 196
    img = np.full((h, w, 3), 0.012, dtype=np.float32)
    cell = 11  # px per LED dot
    gap = 14
    total = len(lines) * 7 * cell + (len(lines) - 1) * gap
    y0 = (h - total) // 2
    for i, line in enumerate(lines):
        width = (len(line) * 6 - 1) * cell
        x0 = (w - width) // 2
        y = y0 + i * (7 * cell + gap)
        for k, ch in enumerate(line):
            for r, row in enumerate(GLYPHS[ch]):
                for c, bit in enumerate(row):
                    if bit == "1":
                        img[y + r * cell:y + (r + 1) * cell, x0 + (k * 6 + c) * cell:x0 + (k * 6 + c + 1) * cell] = colour
    Image.fromarray((np.clip(img, 0.0, 1.0) * 255).astype(np.uint8)).save(path, quality=92)


def main() -> None:
    TEX.mkdir(exist_ok=True)
    glow(TEX / "pan_glow_up.jpg", hot_at_top=False, mottle=0.6)  # for a surface whose v rises away from the heat: the wall outside
    glow(TEX / "pan_glow_in.jpg", hot_at_top=True, mottle=0.25)  # for a surface whose v rises towards the heat: the wall inside, down to the floor
    display(TEX / "display_on.jpg", ["P8", "230dC"], (1.0, 0.50, 0.04))
    display(TEX / "display_hot.jpg", ["Hot", "230dC"], (1.0, 0.07, 0.03))


if __name__ == "__main__":
    main()
