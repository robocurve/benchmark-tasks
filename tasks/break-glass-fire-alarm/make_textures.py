"""Write this task's generated textures to textures/: the alarm's printed front. Needs Pillow. Run: python make_textures.py

The wording is Pillow's built-in font. There is no brand or logo.
"""

from __future__ import annotations

import pathlib

from PIL import Image, ImageDraw, ImageFont

HERE = pathlib.Path(__file__).resolve().parent / "textures"
RED = (196, 24, 20)
WHITE = (250, 250, 248)


def _font(size: int):
    return ImageFont.load_default(size=size)


def _centre(d: ImageDraw.ImageDraw, y: int, text: str, size: int, width: int, bold: int = 0, fill=WHITE) -> None:
    f = _font(size)
    w = d.textlength(text, font=f)
    d.text(((width - w) / 2, y), text, font=f, fill=fill, stroke_width=bold, stroke_fill=fill)


def front() -> Image.Image:
    """The face of the box, 12 x 20 cm at 60 pixels per cm, top of the image towards the far end of the slope. The window is cut from this face by geometry."""
    W, H = 720, 1200
    im = Image.new("RGB", (W, H), RED)
    d = ImageDraw.Draw(im)
    _centre(d, -6, "FIRE", 232, W, 7)
    _centre(d, 208, "ALARM", 128, W, 4)
    _centre(d, 1000, "BREAK GLASS", 86, W, 3)
    _centre(d, 1094, "PULL HANDLE", 72, W, 2)
    return im


def floor() -> Image.Image:
    """What is seen behind the glass, 9 x 9.5 cm at 60 pixels per cm: PULL and a big arrow pointing the way the handle slides, down the slope, towards the robot. The arrow is under the handle's bar and its travel."""
    W, H = 540, 570
    yellow = (250, 205, 30)
    im = Image.new("RGB", (W, H), (22, 22, 24))
    d = ImageDraw.Draw(im)
    _centre(d, 6, "PULL", 112, W, 4, yellow)
    x = W // 2
    d.rectangle((x - 62, 150, x + 62, 400), fill=yellow)
    d.polygon([(x - 190, 390), (x + 190, 390), (x, 560)], fill=yellow)
    for sx in (x - 216, x + 216):
        d.rectangle((sx - 7, 150, sx + 7, 372), fill=(70, 70, 74))  # the slots the bar's ends run in
    return im


if __name__ == "__main__":
    front().save(HERE / "alarm_front.png")
    floor().save(HERE / "alarm_floor.png")
