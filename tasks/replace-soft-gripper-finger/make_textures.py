"""Make the textures in textures/. Each image is at most 512 px and repeats without a seam.

From ambientCG (CC0), downloaded as <id>_1K-JPG.zip and unpacked into <SOURCES>/x_<id>/:
  Metal032     brushed steel: steel_color, steel_rough, steel_normal
  Plastic012A  moulded plastic: plastic_rough, plastic_normal
  Rubber004    rubber: rubber_color, rubber_normal; worn_rubber is its pale, scuffed version

Our own, generated here: pla_normal (printed layer lines), print_white (white printed plastic), tpu_orange and tpu_worn (new and worn orange TPU).

Usage: python make_textures.py SOURCES OUT_DIR name [name ...]
"""

from __future__ import annotations

import glob
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


def _layers(rng, rows: int = 50) -> tuple[np.ndarray, np.ndarray]:
    """(per-layer offset, position within the layer from 0 to 1) for each image row."""
    v = np.arange(SIZE) * rows / SIZE
    return rng.normal(0.0, 1.0, rows)[v.astype(int)], v % 1.0


def pla_normal() -> np.ndarray:
    rng = np.random.default_rng(5)
    offset, frac = _layers(rng)
    h = (np.abs(np.sin(np.pi * frac)) ** 0.5 + 0.05 * offset)[:, None] * np.ones((SIZE, SIZE))
    return _normal_from_height(h, 1.5)


def _layer_shade(rng, rows: int) -> np.ndarray:
    offset, frac = _layers(rng, rows)
    groove = np.abs(np.sin(np.pi * frac)) ** 0.5
    return ((0.8 + 0.2 * groove + 0.02 * offset)[:, None] * np.ones((SIZE, SIZE))) * (1.0 + 0.02 * _noise(rng, 6.0))


def print_white() -> np.ndarray:
    """White printed plastic with fine layer lines."""
    rng = np.random.default_rng(7)
    return np.uint8(np.clip(np.array([0.97, 0.96, 0.93])[None, None, :] * _layer_shade(rng, 50)[..., None] * 255.0, 0, 255))


def tpu_orange() -> np.ndarray:
    """New orange TPU, printed."""
    rng = np.random.default_rng(8)
    return np.uint8(np.clip(np.array([1.0, 0.36, 0.03])[None, None, :] * _layer_shade(rng, 40)[..., None] * 255.0, 0, 255))


def tpu_worn() -> np.ndarray:
    """Orange TPU worn pale: faded in blotches, scuffed and dirty."""
    rng = np.random.default_rng(9)
    fade = np.clip(0.55 + 0.28 * (0.6 * _noise(rng, 22.0) + 0.4 * _noise(rng, 6.0)), 0.0, 1.0)[..., None]
    new = np.array([0.90, 0.46, 0.20])[None, None, :]
    pale = np.array([0.84, 0.76, 0.66])[None, None, :]
    rgb = (new * (1.0 - fade) + pale * fade) * _layer_shade(rng, 40)[..., None] * (1.0 + 0.05 * _noise(rng, 1.5))[..., None]
    return np.uint8(np.clip(rgb * 255.0, 0, 255))


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
        "pla_normal": pla_normal,
        "print_white": print_white,
        "tpu_orange": tpu_orange,
        "tpu_worn": tpu_worn,
        "worn_rubber": lambda: worn_rubber(sources),
        "steel_color": lambda: _source(sources, "Metal032", "Color"),
        "steel_rough": lambda: _source(sources, "Metal032", "Roughness"),
        "steel_normal": lambda: _source(sources, "Metal032", "NormalGL"),
        "plastic_rough": lambda: _source(sources, "Plastic012A", "Roughness"),
        "plastic_normal": lambda: _source(sources, "Plastic012A", "NormalGL"),
        "rubber_color": lambda: _source(sources, "Rubber004", "Color"),
        "rubber_normal": lambda: _source(sources, "Rubber004", "NormalGL"),
    }
    os.makedirs(out, exist_ok=True)
    for name in names:
        Image.fromarray(made[name]()).save(os.path.join(out, f"{name}.jpg"), quality=88)
        print(name, os.path.getsize(os.path.join(out, f"{name}.jpg")) // 1024, "KB")


if __name__ == "__main__":
    main()
