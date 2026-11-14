"""Draws the task's textures as image files in textures/ (our own artwork, no third-party images). Run once with any
Python that has Pillow and NumPy: python make_textures.py. The task loads the image files; it does not need Pillow.

- oak.jpg: oak for the letter rack, the grain along u.
- invoice.png: the printed face of a folded invoice (a generic layout with no names or logos). The image covers
  21 x 21 cm; the 21 x 9.9 cm face is its middle band. It is mirrored and rolled by half in both directions, because the
  stack's face is mapped from its centre and seen from the robot's side.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = Path(__file__).resolve().parent
OUT = HERE / "textures"


def _noise(shape, scale: float, rng) -> np.ndarray:
    """Smooth value noise in [-1, 1] with features about `scale` pixels across."""
    h, w = shape
    gh, gw = max(2, int(h / scale) + 2), max(2, int(w / scale) + 2)
    g = Image.fromarray((rng.random((gh, gw)) * 255).astype(np.uint8)).resize((w, h), Image.BICUBIC)
    return np.asarray(g, float) / 127.5 - 1.0


def oak(size: int = 512, seed: int = 5, mm: float = 100.0) -> Image.Image:
    """Oak's open grain: wavy darker late-wood lines and pores along u, on a warm mid-brown."""
    rng = np.random.default_rng(seed)
    px_mm = size / mm
    ys = np.arange(size, dtype=float)
    wav = _noise((size, size), 120, rng) * 14.0 + _noise((size, size), 30, rng) * 2.0
    rows = (ys[:, None] + wav) / px_mm
    lines = np.zeros((size, size))
    for period, amp in ((2.2, 0.10), (5.5, 0.08), (11.0, 0.05)):
        lines += amp * (0.5 + 0.5 * np.sin(2 * np.pi * rows / period + rng.uniform(0, 6.3))) ** 3
    tint = np.array((176, 132, 84), float)
    tone = tint[None, None, :] * (1.0 - lines[..., None]) * (1.0 + 0.04 * _noise((size, size), 20, rng)[..., None])
    pores = (rng.random((size, size)) < 0.004).astype(float)
    pores = np.asarray(Image.fromarray((pores * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.7)), float)
    tone *= 1.0 - 0.25 * (pores / max(1.0, pores.max()))[..., None]
    return Image.fromarray(np.clip(tone, 0, 255).astype(np.uint8))


def _font(px: int):
    for name in ("DejaVuSans-Bold.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(name, px)
        except OSError:
            continue
    return ImageFont.load_default()


def invoice(size: int = 1024, seed: int = 9) -> Image.Image:
    """A generic printed invoice panel: a heading, address blocks of grey text bars, a ruled item table and a total box."""
    rng = np.random.default_rng(seed)
    px_m = size / 0.21
    img = Image.new("RGB", (size, size), (247, 247, 243))
    d = ImageDraw.Draw(img)
    face_h = int(round(0.099 * px_m))
    top = (size - face_h) // 2
    ink, grey, light = (40, 42, 48), (120, 122, 128), (196, 198, 202)

    def P(x_mm, y_mm):  # mm from the face's top-left corner
        return int(x_mm / 1000 * px_m), top + int(y_mm / 1000 * px_m)

    d.text(P(12, 7), "INVOICE", fill=ink, font=_font(int(0.007 * px_m)))
    d.rectangle([P(12, 16), P(80, 16.6)], fill=ink)
    for row in range(4):                                   # sender block, right
        w = rng.uniform(30, 52)
        d.rectangle([P(150, 6 + 4 * row), P(150 + w, 8 + 4 * row)], fill=grey)
    for row in range(3):                                   # customer block, left
        w = rng.uniform(28, 48)
        d.rectangle([P(12, 22 + 4 * row), P(12 + w, 24 + 4 * row)], fill=grey)
    for row in range(3):                                   # number and date, right
        d.rectangle([P(150, 24 + 4 * row), P(170, 26 + 4 * row)], fill=grey)
        d.rectangle([P(175, 24 + 4 * row), P(175 + rng.uniform(12, 22), 26 + 4 * row)], fill=ink)
    d.rectangle([P(12, 40), P(198, 45)], fill=light)       # table header
    for row in range(7):
        y = 48 + 5 * row
        d.rectangle([P(14, y), P(14 + rng.uniform(40, 90), y + 1.8)], fill=grey)
        for x in (120, 145, 175):
            d.rectangle([P(x, y), P(x + rng.uniform(10, 18), y + 1.8)], fill=grey)
        d.line([P(12, y + 3.4), P(198, y + 3.4)], fill=light, width=1)
    d.rectangle([P(140, 85), P(198, 93)], outline=ink, width=2)
    d.rectangle([P(145, 88), P(165, 90.5)], fill=ink)
    d.rectangle([P(175, 88), P(194, 90.5)], fill=ink)
    img = img.filter(ImageFilter.GaussianBlur(0.4))
    a = np.asarray(img)[:, ::-1]                           # read the right way round from the robot's side (-x)
    a = np.roll(np.roll(a, size // 2, axis=0), size // 2, axis=1)
    return Image.fromarray(np.ascontiguousarray(a))


def main() -> None:
    OUT.mkdir(exist_ok=True)
    oak().save(OUT / "oak.jpg", quality=90)
    invoice().save(OUT / "invoice.png")
    print("wrote", sorted(p.name for p in OUT.iterdir()))


if __name__ == "__main__":
    main()
