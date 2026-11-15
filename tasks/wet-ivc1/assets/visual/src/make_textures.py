"""Generate the PBR textures for the wet-ivc1 visual meshes (deterministic, synthetic): python3 make_textures.py OUT_DIR"""

import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
rng = np.random.default_rng(7)
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT_M = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"


def noise(n, scale):
    """Smooth value noise in [0, 1] at a feature size of ``scale`` px."""
    k = max(2, n // scale)
    small = rng.random((k, k))
    return np.asarray(Image.fromarray((small * 255).astype(np.uint8)).resize((n, n), Image.BICUBIC), float) / 255


def save(name, arr):
    Image.fromarray(np.clip(arr * 255, 0, 255).astype(np.uint8)).save(OUT / name)


# corncob bedding: packed 3-6 mm granules, tan to brown
n = 1024
img = Image.new("RGB", (n, n), (150, 118, 70))
d = ImageDraw.Draw(img)
for _ in range(9000):
    x, y = rng.random(2) * n
    r = rng.uniform(5, 13)
    t = rng.random()
    c = tuple(int(v) for v in (np.array([196, 160, 104]) * (1 - t) + np.array([140, 100, 52]) * t) * rng.uniform(0.85, 1.1))
    d.ellipse([x - r, y - r * rng.uniform(0.6, 1.0), x + r, y + r * rng.uniform(0.6, 1.0)], fill=c)
img = img.filter(ImageFilter.GaussianBlur(0.8))
arr = np.asarray(img, float) / 255 * (0.85 + 0.15 * noise(n, 64))[..., None]
save("bedding_albedo.png", arr)

# brushed stainless: streaks along u
n = 1024
streak = np.repeat(rng.random((n, 1)), n, axis=1)
streak = np.asarray(Image.fromarray((streak * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(0.6)), float) / 255
lum = 0.70 + 0.07 * streak + 0.04 * noise(n, 128)
save("steel_brushed_albedo.png", np.stack([lum * 0.97, lum * 0.98, lum * 1.0], -1))
save("steel_brushed_rough.png", np.repeat((0.22 + 0.18 * streak)[..., None], 3, -1))

# silicone bench mat: slate blue with a fine raised-dot grip pattern
n = 1024
base = np.array([0.17, 0.29, 0.38])
lum = 0.94 + 0.06 * noise(n, 96)
yy, xx = np.mgrid[0:n, 0:n]
dots = (((xx % 32) - 16) ** 2 + ((yy % 32) - 16) ** 2 < 36).astype(float)
save("mat_albedo.png", base * (lum - 0.10 * dots)[..., None])

# PTFE filter sheet: white fibrous
n = 512
fib = 0.88 + 0.08 * noise(n, 8) + 0.04 * noise(n, 64)
save("filter_albedo.png", np.repeat(fib[..., None], 3, -1))

# cage card (100 x 60 mm), printed
W, H = 1000, 600
card = Image.new("RGB", (W, H), (250, 250, 246))
d = ImageDraw.Draw(card)
d.rectangle([0, 0, W, 92], fill=(38, 92, 160))
d.text((24, 16), "CAGE CARD", font=ImageFont.truetype(FONT_B, 54), fill="white")
d.text((640, 22), "No. 0417", font=ImageFont.truetype(FONT_B, 46), fill="white")
f = ImageFont.truetype(FONT, 40)
rows = [("Strain", "C57BL/6J"), ("Sex / n", "F / 4"), ("DOB", "2026-09-02"), ("Protocol", "2026-031"), ("Rack", "R2  C3")]
for i, (k, v) in enumerate(rows):
    y = 118 + i * 74
    d.text((28, y), k, font=f, fill=(90, 90, 90))
    d.text((300, y), v, font=ImageFont.truetype(FONT_B, 40), fill=(20, 20, 20))
    d.line([28, y + 62, 620, y + 62], fill=(200, 200, 200), width=2)
x = 660
for i in range(46):
    w = int(rng.choice([4, 4, 8, 12]))
    if i % 2 == 0:
        d.rectangle([x, 150, x + w, 470], fill="black")
    x += w + 3
d.text((660, 488), "0417-26-031", font=ImageFont.truetype(FONT_M, 34), fill="black")
card.save(OUT / "cage_card.png")

# bottle graduation label (40 x 60 mm), white print area
W, H = 400, 600
lab = Image.new("RGB", (W, H), (244, 246, 248))
d = ImageDraw.Draw(lab)
for i, ml in enumerate(range(50, 300, 50)):
    y = H - 40 - i * 110
    d.line([20, y, 150, y], fill=(30, 60, 120), width=6)
    d.text((170, y - 26), f"{ml}", font=ImageFont.truetype(FONT_B, 46), fill=(30, 60, 120))
    if i < 4:
        d.line([20, y - 55, 90, y - 55], fill=(30, 60, 120), width=4)
d.text((40, 16), "mL", font=ImageFont.truetype(FONT_B, 50), fill=(30, 60, 120))
lab.save(OUT / "bottle_label.png")

# rack cell position plate (60 x 20 mm)
W, H = 600, 200
pl = Image.new("RGB", (W, H), (236, 238, 240))
d = ImageDraw.Draw(pl)
d.rectangle([6, 6, W - 7, H - 7], outline=(40, 40, 40), width=6)
d.text((34, 40), "R2 - C3", font=ImageFont.truetype(FONT_B, 104), fill=(25, 25, 25))
pl.save(OUT / "rack_plate.png")
print("textures ->", OUT)
