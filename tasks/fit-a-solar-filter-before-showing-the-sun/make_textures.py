"""Draws the task's textures as image files in textures/ (our own artwork, no third-party images). Run once with any
Python that has Pillow and NumPy: python make_textures.py. The task loads the image files; it does not need Pillow.

- foam.jpg: light grey pick-and-pluck case foam, cut into 1 cm squares (a 4 cm tile).
- film.jpg: silvery solar film, faintly wrinkled (a 5 cm tile).
- brushed.jpg: brushed aluminium for the case's frame (a 10 cm tile).
- decoy_top.png: the eyepiece filter's top, seen from above: a black anodised ring with "SUN" engraved in white.
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


def _save(rgb: np.ndarray, name: str) -> None:
    Image.fromarray(np.clip(rgb * 255, 0, 255).astype(np.uint8)).save(OUT / name, quality=90)


def foam(size: int = 256) -> None:
    """Four 1 cm squares across, with dark cut lines between them and a fine open-cell grain."""
    rng = np.random.default_rng(1)
    v = 0.78 + 0.03 * _noise((size, size), 40, rng) + 0.035 * rng.normal(size=(size, size))
    cell = size // 4
    idx = np.arange(size)
    cut = (np.minimum(idx % cell, cell - idx % cell) < 2)
    v[cut, :] *= 0.62
    v[:, cut] *= 0.62
    v = np.asarray(Image.fromarray(np.clip(v * 255, 0, 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.6)), float) / 255
    _save(np.repeat(v[..., None], 3, axis=2) * np.array([0.98, 0.99, 1.0]), "foam.jpg")


def film(size: int = 256) -> None:
    rng = np.random.default_rng(2)
    v = 0.86 + 0.07 * _noise((size, size), 60, rng) + 0.04 * _noise((size, size), 14, rng)
    _save(np.repeat(v[..., None], 3, axis=2), "film.jpg")


def brushed(size: int = 256) -> None:
    rng = np.random.default_rng(3)
    streaks = rng.normal(size=(size, 1)) * 0.05 + _noise((size, size), 80, rng) * 0.02
    v = 0.80 + np.repeat(streaks[:, :1], size, axis=1) + 0.015 * rng.normal(size=(size, size))
    v = np.asarray(Image.fromarray(np.clip(v * 255, 0, 255).astype(np.uint8)).filter(ImageFilter.BoxBlur(1)), float) / 255
    _save(np.repeat(v[..., None], 3, axis=2), "brushed.jpg")


def decoy_top(size: int = 256) -> None:
    """The filter's top face, 32 mm across, mapped onto a 32 x 32 mm square: a black ring round dark green glass (the
    glass is a separate mesh), with "SUN" engraved in white on the ring's near side."""
    img = Image.new("RGB", (size, size), (14, 14, 16))
    d = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size // 10)
    except OSError:
        font = ImageFont.load_default()
    text = "SUN"
    box = d.textbbox((0, 0), text, font=font)
    w, h = box[2] - box[0], box[3] - box[1]
    # On the ring, between radius 13.5 and 16 mm: near the image's bottom edge.
    d.text(((size - w) / 2 - box[0], size * 0.905 - h / 2 - box[1]), text, fill=(235, 235, 230), font=font)
    img.save(OUT / "decoy_top.png")


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    foam()
    film()
    brushed()
    decoy_top()
    print("wrote", sorted(p.name for p in OUT.iterdir()))
