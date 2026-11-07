"""Make the textures in textures/. Each image is at most 512 px and repeats without a seam.

From ambientCG (CC0), downloaded as <id>_1K-JPG.zip and unpacked into <SOURCES>/x_<id>/:
  Metal032     brushed steel: steel_color, steel_rough, steel_normal
  Plastic012A  moulded plastic: plastic_rough, plastic_normal
  Plastic008   blue painted surface: blue_paint
  Rubber004    rubber: rubber_color, rubber_normal; worn_rubber is its pale, scuffed version

Our own, generated here: pei_color and pei_normal (the gold powder-coated PEI of a print plate), pla_color and pla_normal (printed
layer lines).

The screen's pictures, screen_atlas.png, are our own, drawn in DejaVu Sans Bold (set SCREEN_FONT to its path).

Usage: python make_textures.py SOURCES OUT_DIR name [name ...]
"""

from __future__ import annotations

import glob
import math
import os
import sys

import numpy as np
from PIL import Image

SIZE = 512


def _blur(a: np.ndarray, sigma: float) -> np.ndarray:
    """A Gaussian blur that wraps round, so the result still tiles."""
    fy = np.fft.fftfreq(a.shape[0])[:, None]
    fx = np.fft.fftfreq(a.shape[1])[None, :]
    return np.real(np.fft.ifft2(np.fft.fft2(a) * np.exp(-2 * (np.pi * sigma) ** 2 * (fx**2 + fy**2))))


def _noise(rng, sigma: float) -> np.ndarray:
    a = _blur(rng.standard_normal((SIZE, SIZE)), sigma)
    return a / a.std()


def _normal_from_height(h: np.ndarray, strength: float) -> np.ndarray:
    dx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * strength
    dy = (np.roll(h, 1, 0) - np.roll(h, -1, 0)) * strength  # image rows run down, v runs up
    n = np.dstack([-dx, -dy, np.ones_like(h)])
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return np.uint8(np.clip((n * 0.5 + 0.5) * 255.0, 0, 255))


def pei_color() -> np.ndarray:
    rng = np.random.default_rng(3)
    grain = 0.55 * _noise(rng, 1.0) + 0.45 * _noise(rng, 3.0)
    speck = _noise(rng, 0.7) > 2.0
    rgb = np.array([0.78, 0.56, 0.20])[None, None, :] * (1.0 + 0.10 * grain[..., None])
    rgb = np.where(speck[..., None], rgb * 0.72, rgb)
    return np.uint8(np.clip(rgb * 255.0, 0, 255))


def pei_normal() -> np.ndarray:
    rng = np.random.default_rng(4)
    return _normal_from_height(_noise(rng, 1.0) + 0.5 * _noise(rng, 3.0), 0.10)


def _layers(rng, rows: int = 50) -> tuple[np.ndarray, np.ndarray]:
    """(per-layer offset, position within the layer from 0 to 1) for each image row."""
    v = np.arange(SIZE) * rows / SIZE
    return rng.normal(0.0, 1.0, rows)[v.astype(int)], v % 1.0


def pla_color() -> np.ndarray:
    rng = np.random.default_rng(5)
    offset, frac = _layers(rng)
    groove = np.abs(np.sin(np.pi * frac)) ** 0.5  # dark in the seam between two layers
    shade = (0.72 + 0.28 * groove + 0.03 * offset)[:, None] * np.ones((SIZE, SIZE))
    shade = shade * (1.0 + 0.02 * _noise(rng, 6.0))
    rgb = np.array([1.0, 0.50, 0.10])[None, None, :] * shade[..., None]
    return np.uint8(np.clip(rgb * 255.0, 0, 255))


def pla_normal() -> np.ndarray:
    rng = np.random.default_rng(5)
    offset, frac = _layers(rng)
    h = (np.abs(np.sin(np.pi * frac)) ** 0.5 + 0.05 * offset)[:, None] * np.ones((SIZE, SIZE))
    return _normal_from_height(h, 1.5)


def screen_atlas() -> np.ndarray:
    """One picture of the printer's screen for each second of cooling, from 60 to 35 C, left to right and top to bottom: the print's status, the bed's
    temperature, and a hot-surface symbol while the bed is too hot to touch. Its colours follow the lamp: red, orange from 5 s, green when cool."""
    from PIL import ImageDraw, ImageFont

    font = os.environ["SCREEN_FONT"]
    cols, rows, w, h = 3, 4, 340, 194
    atlas = Image.new("RGB", (cols * w, rows * h), (6, 10, 18))
    draw = ImageDraw.Draw(atlas)

    def text(x: float, y: float, words: str, size: int, fill, centred_in: float | None = None) -> None:
        f = ImageFont.truetype(font, size)
        box_ = draw.textbbox((0, 0), words, font=f)
        if centred_in is not None:
            x = x + (centred_in - (box_[2] - box_[0])) / 2
        draw.text((x - box_[0], y - box_[1]), words, font=f, fill=fill)

    def hot_symbol(x: float, y: float, s: float) -> None:
        """A hot-surface sign: a yellow warning triangle with three heat waves rising from a bar."""
        draw.polygon([(x + s / 2, y), (x + s, y + s * 0.9), (x, y + s * 0.9)], fill=(250, 205, 40), outline=(20, 20, 20))
        for dx in (-0.14, 0.0, 0.14):
            cx = x + s * (0.5 + dx)
            pts = [(cx + s * 0.035 * math.sin(k * 0.9), y + s * (0.74 - 0.05 * k)) for k in range(8)]
            draw.line(pts, fill=(15, 15, 15), width=max(3, int(s * 0.045)))
        draw.line([(x + s * 0.27, y + s * 0.79), (x + s * 0.73, y + s * 0.79)], fill=(15, 15, 15), width=max(3, int(s * 0.05)))

    for k in range(11):
        x0, y0 = (k % cols) * w, (k // cols) * h
        cool = k == 10
        colour = (70, 240, 110) if cool else (255, 150, 30) if k >= 5 else (255, 70, 55)
        draw.rectangle((x0 + 2, y0 + 2, x0 + w - 3, y0 + h - 3), fill=(10, 16, 28), outline=(60, 70, 85), width=2)
        text(x0, y0 + 10, "PRINT DONE", 30, (225, 230, 235), centred_in=w)
        temperature = f"{round(60 - 2.5 * k)}\N{DEGREE SIGN}C"
        if cool:
            text(x0, y0 + 60, temperature, 84, colour, centred_in=w)
            text(x0, y0 + 152, "BED COOL", 30, colour, centred_in=w)
        else:
            hot_symbol(x0 + 14, y0 + 50, 128)
            text(x0 + 142, y0 + 66, temperature, 66, colour, centred_in=w - 150)
            text(x0 + 142, y0 + 146, "BED HOT", 30, colour, centred_in=w - 150)
    return np.asarray(atlas)


def _source(sources: str, ident: str, kind: str) -> np.ndarray:
    path = glob.glob(os.path.join(sources, f"x_{ident}", f"*_{kind}.jpg"))[0]
    return np.asarray(Image.open(path).convert("RGB").resize((SIZE, SIZE), Image.LANCZOS))


def worn_rubber(sources: str) -> np.ndarray:
    rng = np.random.default_rng(6)
    base = _source(sources, "Rubber004", "Color").mean(axis=2) / 255.0
    scuff = 0.6 * _noise(rng, 18.0) + 0.4 * _noise(rng, 5.0)
    shade = 0.62 + 0.9 * (base - base.mean()) + 0.055 * scuff
    rgb = np.array([1.0, 0.97, 0.90])[None, None, :] * shade[..., None]
    return np.uint8(np.clip(rgb * 255.0, 0, 255))


def main() -> None:
    sources, out, names = sys.argv[1], sys.argv[2], sys.argv[3:]
    made = {
        "pei_color": pei_color,
        "pei_normal": pei_normal,
        "pla_color": pla_color,
        "pla_normal": pla_normal,
        "screen_atlas": screen_atlas,
        "worn_rubber": lambda: worn_rubber(sources),
        "steel_color": lambda: _source(sources, "Metal032", "Color"),
        "steel_rough": lambda: _source(sources, "Metal032", "Roughness"),
        "steel_normal": lambda: _source(sources, "Metal032", "NormalGL"),
        "plastic_rough": lambda: _source(sources, "Plastic012A", "Roughness"),
        "plastic_normal": lambda: _source(sources, "Plastic012A", "NormalGL"),
        "blue_paint": lambda: _source(sources, "Plastic008", "Color"),
        "rubber_color": lambda: _source(sources, "Rubber004", "Color"),
        "rubber_normal": lambda: _source(sources, "Rubber004", "NormalGL"),
    }
    os.makedirs(out, exist_ok=True)
    for name in names:
        file = os.path.join(out, f"{name}.png" if name == "screen_atlas" else f"{name}.jpg")
        Image.fromarray(made[name]()).save(file, **({} if name == "screen_atlas" else {"quality": 88}))
        print(name, os.path.getsize(file) // 1024, "KB")


if __name__ == "__main__":
    main()
