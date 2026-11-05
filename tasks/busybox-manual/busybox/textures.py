"""Label textures for one busybox configuration, drawn with PIL in the bundled DejaVu fonts (``assets/fonts``)."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from busybox.design import (
    BIG_BUTTON_COLORS,
    DIAL_POSITIONS,
    GREEK_LETTERS,
    MODULE_SIZE,
    PLATE_COLOR,
    SLIDER_POSITIONS,
    SLIDER_TRAVEL,
    BusyboxConfig,
    slider_x,
    slider_ys,
    switch_ys,
)

FONT_DIR = Path(__file__).resolve().parent.parent / "assets" / "fonts"
PLATE_PX = 512  # module plate resolution; 1 px = MODULE_SIZE / PLATE_PX


def _font(size: int, mono: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(FONT_DIR / ("DejaVuSansMono-Bold.ttf" if mono else "DejaVuSans-Bold.ttf")), size)


def _text(draw: ImageDraw.ImageDraw, xy, s: str, size: int, fill, mono: bool = False) -> None:
    draw.text(xy, s, font=_font(size, mono), fill=fill, anchor="mm")


def _rgb8(c) -> tuple[int, int, int]:
    return tuple(round(255 * v) for v in c[:3])  # type: ignore[return-value]


def _luma(c) -> float:
    return 0.3 * c[0] + 0.59 * c[1] + 0.11 * c[2]


def _to_px(x: float, y: float) -> tuple[float, float]:
    """Module frame (x up, y left) to plate image coordinates (right, down)."""
    m, px = MODULE_SIZE, PLATE_PX
    return (px / 2 - y / m * px, px / 2 - x / m * px)


def _blank_plate() -> tuple[Image.Image, ImageDraw.ImageDraw]:
    img = Image.new("RGB", (PLATE_PX, PLATE_PX), _rgb8(PLATE_COLOR))
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, PLATE_PX - 1, PLATE_PX - 1), outline=(40, 40, 40), width=6)
    return img, d


def plate_switches() -> Image.Image:
    """Switch numbers 1-4 below the switches (the lamps sit above)."""
    img, d = _blank_plate()
    for i, y in enumerate(switch_ys()):
        _text(d, _to_px(-0.038, y), str(i + 1), 40, (20, 20, 20))
    return img


def plate_sliders() -> Image.Image:
    """Slots, tick marks and the numbers 1..5 between the two sliders."""
    img, d = _blank_plate()
    for k in range(SLIDER_POSITIONS):
        x = slider_x(k)
        _text(d, _to_px(x, 0), str(k + 1), 40, (20, 20, 20))
        for y in slider_ys():
            x0, y0 = _to_px(x, y + 0.016)
            x1, _ = _to_px(x, y - 0.016)
            d.line((x0, y0, x1, y0), fill=(30, 30, 30), width=4)
    for y in slider_ys():
        a, b = _to_px(SLIDER_TRAVEL / 2 + 0.006, y + 0.003), _to_px(-SLIDER_TRAVEL / 2 - 0.006, y - 0.003)
        d.rectangle((*a, *b), fill=(25, 25, 25))
    return img


def plate_dial() -> Image.Image:
    """Numbers 1..8 clockwise from the top (+x), with a tick at each."""
    img, d = _blank_plate()
    for k in range(DIAL_POSITIONS):
        a = -k * 2 * math.pi / DIAL_POSITIONS
        _text(d, _to_px(0.043 * math.cos(a), 0.043 * math.sin(a)), str(k + 1), 44, (20, 20, 20))
        r0, r1 = 0.026, 0.033
        tick = (*_to_px(r0 * math.cos(a), r0 * math.sin(a)), *_to_px(r1 * math.cos(a), r1 * math.sin(a)))
        d.line(tick, fill=(20, 20, 20), width=5)
    return img


def plate_plain() -> Image.Image:
    return _blank_plate()[0]


def lamp(on: bool) -> Image.Image:
    """A switch lamp seen from above: a glowing red dome with a hot centre when lit, dull dark red when off."""
    n = 128
    yy, xx = np.mgrid[0:n, 0:n] / (n - 1) * 2 - 1
    r = np.hypot(xx, yy).clip(0, 1)[..., None]
    spot = np.exp(-((xx + 0.3) ** 2 + (yy + 0.3) ** 2) / 0.04)[..., None]
    if on:
        img, gloss = (1 - r) * np.array([255, 185, 150]) + r * np.array([235, 30, 20]), 0.8
    else:
        img, gloss = (1 - r) * np.array([95, 18, 16]) + r * np.array([45, 6, 6]), 0.35
    img = img + gloss * spot * (255 - img)
    return Image.fromarray(img.clip(0, 255).astype(np.uint8))


def serial(s: str) -> Image.Image:
    img = Image.new("RGB", (640, 192), (245, 245, 240))
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, 639, 191), outline=(20, 20, 20), width=6)
    _text(d, (320, 40), "SERIAL NO.", 30, (60, 60, 60))
    _text(d, (320, 120), s, 96, (10, 10, 10), mono=True)
    return img


def display(word: str) -> Image.Image:
    """Green on black, like a small LCD."""
    img = Image.new("RGB", (640, 192), (12, 18, 12))
    _text(ImageDraw.Draw(img), (320, 96), word, 120, (90, 240, 110), mono=True)
    return img


def big_label(label: str, color: str) -> Image.Image:
    """The big button's cap: its colour with the label, dark text on light caps."""
    rgb = BIG_BUTTON_COLORS[color]
    img = Image.new("RGB", (256, 256), _rgb8(rgb))
    ink = (15, 15, 15) if _luma(rgb) > 0.5 else (245, 245, 245)
    _text(ImageDraw.Draw(img), (128, 128), label, 64 if len(label) > 4 else 76, ink)
    return img


def greek(name: str) -> Image.Image:
    img = Image.new("RGB", (128, 128), (232, 230, 220))
    _text(ImageDraw.Draw(img), (64, 66), GREEK_LETTERS[name], 100, (10, 10, 10))
    return img


def textures(cfg: BusyboxConfig) -> dict[str, Image.Image]:
    """Every texture the configuration shows, by name (the USD references ``textures/<name>.png``)."""
    out = {
        "plate_switches": plate_switches(),
        "plate_sliders": plate_sliders(),
        "plate_dial": plate_dial(),
        "plate_plain": plate_plain(),
        "lamp_on": lamp(True),
        "lamp_off": lamp(False),
        "serial": serial(cfg.serial),
        "display": display(cfg.word),
        "big_label": big_label(cfg.big_button_label, cfg.big_button_color),
    }
    for name in cfg.letters:
        out[f"greek_{name}"] = greek(name)
    return out


def write_textures(cfg: BusyboxConfig, directory: Path) -> dict[str, Path]:
    directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    for name, img in textures(cfg).items():
        paths[name] = directory / f"{name}.png"
        img.save(paths[name])
    return paths
