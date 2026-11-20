"""Draws the task's textures into textures/ (run once; needs NumPy and Pillow). All are our own, drawn here.

- basswood.png: the plywood sheet's basswood face veneer (pale, fine straight grain), tiled every 10 cm.
- ply_edge.png: the sheet's edge, three plies with darker glue lines, for a 3 mm edge.
- brushed.png: brushed anodised aluminium (frame rails, gantry beam, plate, module body), tinted by each material.
- fan.png: the module's top: a round fan grille with its four corner screws, one tile per top face.
- knurl.png: the hand screws' diamond knurl, one tile per 6 mm.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image

OUT = Path(__file__).resolve().parent / "textures"
rng = np.random.default_rng(11)


def save(name: str, a: np.ndarray) -> None:
    OUT.mkdir(exist_ok=True)
    Image.fromarray(np.clip(a, 0, 255).astype(np.uint8)).save(OUT / name, optimize=True)


def _blur_rows(a: np.ndarray, k: int) -> np.ndarray:
    ker = np.ones(k) / k
    return np.apply_along_axis(lambda r: np.convolve(np.r_[r[-k:], r, r[:k]], ker, mode="same")[k:-k], 1, a)


def basswood(n=1024):
    y, x = np.mgrid[0:n, 0:n] / n
    warp = 0.012 * np.sin(2 * np.pi * (x * 1.0 + 0.3)) + 0.006 * np.sin(2 * np.pi * (x * 3.0 + 0.7))
    lines = rng.normal(0, 1, n)
    lines = np.convolve(np.r_[lines, lines[:6]], np.ones(6) / 6, mode="same")[:n]
    idx = ((y + warp) * n).astype(int) % n
    grain = lines[idx]
    fleck = _blur_rows(rng.normal(0, 1, (n, n)), 9)
    v = 222 + 9 * grain + 4 * fleck
    return np.dstack([v * 1.0, v * 0.90, v * 0.72])


def ply_edge(n=256):
    y = np.arange(n)[:, None] / n * 3.0            # three plies across the tile's height
    glue = np.exp(-((y % 1.0) - 0.0) ** 2 / 0.002) + np.exp(-((y % 1.0) - 1.0) ** 2 / 0.002)
    a = 200 - 70 * glue + rng.normal(0, 5, (n, n))
    core = (np.floor(y) % 2 == 1) * -12
    a = a + core
    return np.dstack([a, a * 0.86, a * 0.66])


def brushed(n=512):
    streak = rng.normal(0, 1, n)
    streak = np.convolve(streak, np.ones(3) / 3, mode="same")
    a = 215 + 10 * streak[None, :] + rng.normal(0, 3, (n, n))
    return np.dstack([a, a, a])


def fan(n=512):
    y, x = (np.mgrid[0:n, 0:n] + 0.5) / n - 0.5          # one 6 cm face; units of the face
    r = np.hypot(x, y)
    a = np.full((n, n), 30.0)
    ring = (r > 0.40) & (r < 0.43)
    spokes = (np.abs(np.sin(np.arctan2(y, x) * 2)) < 0.06) & (r < 0.40)
    grille = (np.abs(np.sin(2 * np.pi * r / 0.08)) < 0.25) & (r < 0.40) & (r > 0.10)
    hub = r < 0.10
    a[ring | spokes | grille] = 120
    a[hub] = 70
    for cx in (-0.40, 0.40):
        for cy in (-0.40, 0.40):
            a[np.hypot(x - cx, y - cy) < 0.035] = 190
    a = a + rng.normal(0, 3, (n, n))
    return np.dstack([a, a, a])


def knurl(n=128):
    y, x = (np.mgrid[0:n, 0:n] + 0.5) / n
    d1 = np.abs(np.sin(np.pi * 4 * (x + y)))
    d2 = np.abs(np.sin(np.pi * 4 * (x - y)))
    a = 150 + 80 * np.minimum(d1, d2) + rng.normal(0, 4, (n, n))
    return np.dstack([a, a, a * 1.02])


if __name__ == "__main__":
    save("basswood.png", basswood())
    save("ply_edge.png", ply_edge())
    save("brushed.png", brushed())
    save("fan.png", fan())
    save("knurl.png", knurl())
    print(sorted(p.name for p in OUT.iterdir()))
