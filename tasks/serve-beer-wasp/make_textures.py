"""Write this task's generated textures to textures/: the customer's two faces and a dark wood. Needs NumPy and Pillow. Run: python make_textures.py

A face fills its square texture: the texture is laid on the front of the head by projecting along the head's depth, so the square's edges meet the head's sides.
The features are drawn large, like an illustration, because at 320 x 176 the whole face is about 35 pixels wide.
"""

from __future__ import annotations

import pathlib

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

HERE = pathlib.Path(__file__).resolve().parent / "textures"
S = 1024
SKIN = (232, 186, 150)
DARK = (46, 28, 22)


def _base(flush: float = 0.0) -> Image.Image:
    im = Image.new("RGB", (S, S), SKIN)
    d = ImageDraw.Draw(im, "RGBA")
    for x in (290, 734):  # cheeks
        d.ellipse((x - 110, 590, x + 110, 730), fill=(226, 120, 110, int(70 + 90 * flush)))
    if flush:
        d.rectangle((0, 0, S, S), fill=(214, 90, 70, int(46 * flush)))
    d.polygon([(512, 500), (472, 640), (552, 640)], fill=(205, 150, 120, 150))  # nose
    return im


def happy() -> Image.Image:
    im = _base()
    d = ImageDraw.Draw(im)
    for x, sign in ((330, 1), (694, -1)):
        d.arc((x - 112, 262, x + 112, 430), 200, 340, fill=DARK, width=26)  # raised, arched brows
        d.chord((x - 92, 400, x + 92, 560), 180, 360, fill=(250, 250, 250))  # eyes, wide and smiling
        d.ellipse((x - 46, 408, x + 46, 500), fill=(36, 24, 20))
        d.ellipse((x - 16 + 12, 420, x + 12 + 12, 448), fill=(255, 255, 255))
        d.arc((x - 92, 400, x + 92, 560), 180, 360, fill=DARK, width=16)
    d.chord((236, 560, 788, 930), 8, 172, fill=(120, 24, 34))  # a wide open smile
    d.chord((276, 600, 748, 800), 10, 170, fill=(255, 255, 255))
    d.arc((236, 560, 788, 930), 8, 172, fill=(176, 40, 52), width=26)
    return im.filter(ImageFilter.GaussianBlur(1.2))


def upset() -> Image.Image:
    im = _base(flush=1.0)
    d = ImageDraw.Draw(im)
    for x, sign in ((330, 1), (694, -1)):
        d.line([(x - 118 * sign, 300), (x + 110 * sign, 412)], fill=DARK, width=40)  # brows pulled down to the middle
        d.ellipse((x - 76, 430, x + 76, 546), fill=(250, 250, 250))  # eyes wide
        d.ellipse((x - 36, 452, x + 36, 524), fill=(36, 24, 20))
        d.polygon([(x - 90 * sign, 396), (x + 96 * sign, 448), (x + 96 * sign, 410), (x - 90 * sign, 366)], fill=SKIN)  # the brow's lid over the eye
        d.line([(x - 118 * sign, 300), (x + 110 * sign, 412)], fill=DARK, width=40)
    d.chord((300, 712, 724, 1010), 196, 344, fill=(70, 14, 22))  # a wide downturned mouth
    d.arc((300, 712, 724, 1010), 196, 344, fill=(150, 34, 44), width=28)
    return im.filter(ImageFilter.GaussianBlur(1.2))


def wood() -> Image.Image:
    rng = np.random.default_rng(20261106)
    n = 512
    y = np.arange(n)[:, None]
    grain = sum(a * np.sin(2 * np.pi * (f * y / n + rng.random())) for a, f in ((0.5, 3), (0.3, 9), (0.14, 27), (0.06, 64)))
    tone = 0.62 + 0.22 * grain + rng.normal(0.0, 0.02, (n, n))
    rgb = np.clip(np.stack([tone * 92, tone * 54, tone * 30], axis=2), 0, 255).astype(np.uint8)
    return Image.fromarray(rgb).filter(ImageFilter.GaussianBlur(0.8))


if __name__ == "__main__":
    happy().save(HERE / "face_happy.png")
    upset().save(HERE / "face_upset.png")
    wood().save(HERE / "dark_wood.jpg", quality=88)
