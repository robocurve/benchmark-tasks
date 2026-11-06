"""Write this task's two generated textures to textures/: a printed plastic's layer lines, and the same surface worn. Needs NumPy and Pillow."""

from __future__ import annotations

import numpy as np
from PIL import Image, ImageFilter

SIZE = 512
rng = np.random.default_rng(20261106)


def lines() -> np.ndarray:
    """Fine parallel extrusion lines, 8 to a tile, with a little noise. One tile spans 3.2 cm on the part, so a line is 0.4 mm wide."""
    y = np.arange(SIZE)[:, None]
    wave = 0.5 + 0.5 * np.sin(2 * np.pi * y * 80 / SIZE)
    noise = rng.normal(0.0, 0.03, (SIZE, SIZE))
    return np.clip(0.86 + 0.12 * wave + noise, 0.0, 1.0)


def blur(a: np.ndarray, r: float) -> np.ndarray:
    im = Image.fromarray((np.clip(a, 0, 1) * 255).astype(np.uint8))
    return np.asarray(im.filter(ImageFilter.GaussianBlur(r)), float) / 255.0


def worn() -> np.ndarray:
    """The same lines, faded in patches, with pale scratches and dark grime."""
    base = 0.45 * lines() + 0.45
    patches = blur(rng.random((SIZE, SIZE)), 28)
    patches = (patches - patches.min()) / (patches.max() - patches.min())
    out = base * (0.72 + 0.34 * patches)
    scratch = np.zeros((SIZE, SIZE))
    for _ in range(60):
        x, y = rng.integers(0, SIZE, 2)
        a = rng.uniform(0, np.pi)
        for t in np.linspace(0, rng.integers(30, 180), 240):
            scratch[int(y + t * np.sin(a)) % SIZE, int(x + t * np.cos(a)) % SIZE] = 1.0
    grime = blur((rng.random((SIZE, SIZE)) > 0.996).astype(float), 3) * 6
    return np.clip(out + 0.28 * blur(scratch, 0.6) - 0.35 * np.clip(grime, 0, 1), 0, 1)


Image.fromarray((lines() * 255).astype(np.uint8)).save("textures/printed_lines.jpg", quality=88)
Image.fromarray((worn() * 255).astype(np.uint8)).save("textures/printed_worn.jpg", quality=88)
