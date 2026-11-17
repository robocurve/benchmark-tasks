"""Generate the PBR texture images for the qc-met10 visual meshes (deterministic; seed 0). Run with system python3."""
import pathlib
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

OUT = pathlib.Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
rng = np.random.default_rng(0)
N = 1024
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"


def save(name, a):
    Image.fromarray(np.clip(a * 255, 0, 255).astype(np.uint8)).save(OUT / f"{name}.png")


def blur_noise(sx, sy, scale=1.0):
    n = rng.standard_normal((N, N)).astype(np.float32)
    im = Image.fromarray(((n - n.min()) / np.ptp(n) * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(scale))
    a = np.asarray(im, np.float32) / 255
    if sx != sy:  # anisotropic streaks: smear along x
        k = int(sx)
        a = np.cumsum(np.pad(a, ((0, 0), (k, 0)), mode="wrap"), axis=1)
        a = (a[:, k:] - a[:, :-k]) / k
    return (a - a.mean()) / (a.std() + 1e-6)


def rgb(c, var, n):
    return np.stack([np.full((N, N), ch) + var * n for ch in c], -1)


# brushed stainless steel (jaws): streaks along u
s = blur_noise(60, 1, 0.6)
save("steel_albedo", rgb((0.62, 0.63, 0.64), 0.035, s))
save("steel_rough", 0.28 + 0.06 * s)
# bead-blasted aluminium (adapters, rail base)
a = blur_noise(1, 1, 0.8)
save("alu_albedo", rgb((0.78, 0.79, 0.80), 0.02, a))
save("alu_rough", 0.42 + 0.05 * a)
# fine-texture powder coat (stand base, crosshead): light warm grey
p = blur_noise(1, 1, 1.5)
save("powder_albedo", rgb((0.70, 0.71, 0.70), 0.015, p))
save("powder_rough", 0.62 + 0.06 * p)
# black anodised (grip housings, backstops)
b = blur_noise(1, 1, 0.7)
save("anod_black_albedo", rgb((0.045, 0.047, 0.05), 0.006, b))
save("anod_black_rough", 0.38 + 0.04 * b)
# blue anodised (specimen tray)
save("anod_blue_albedo", rgb((0.17, 0.30, 0.48), 0.012, b))
# ESD bench mat: green two-layer dissipative rubber (a plain black slab read as a block), pebbled
m = blur_noise(1, 1, 2.2)
save("esd_albedo", rgb((0.22, 0.33, 0.29), 0.02, m))
save("esd_rough", 0.82 + 0.05 * m)
# serrations (lower/upper jaw faces): 1 mm pitch ridges -> tangent-space normal map; texture spans 64 mm
pitch = N / 64
u = np.arange(N)
h = np.abs(((u / pitch) % 1.0) - 0.5) * 2  # triangle wave
dh = np.gradient(h) * 6
nx = np.tile(-dh, (N, 1))
nrm = np.stack([nx, np.zeros_like(nx), np.ones_like(nx)], -1)
nrm /= np.linalg.norm(nrm, axis=-1, keepdims=True)
save("serration_normal", nrm * 0.5 + 0.5)
save("serration_albedo", rgb((0.55, 0.56, 0.57), 0.0, s) * (0.85 + 0.15 * np.tile(h, (N, 1)))[..., None])


def panel(name, size, bg, draw_fn):
    im = Image.new("RGB", size, bg)
    draw_fn(ImageDraw.Draw(im))
    im.save(OUT / f"{name}.png")


# force-gauge LCD (backlit, emissive): live load and peak
def lcd(d):
    d.text((24, 10), "TENSION  N", font=ImageFont.truetype(FONT, 34), fill=(25, 40, 30))
    d.text((40, 60), "0.00", font=ImageFont.truetype(MONO, 150), fill=(15, 25, 20))
    d.text((24, 238), "PEAK T   0.00", font=ImageFont.truetype(MONO, 36), fill=(25, 40, 30))
panel("gauge_lcd", (512, 300), (150, 205, 175), lcd)


# gauge keypad / top label strip
def keys(d):
    for i, (lab, col) in enumerate((("ZERO", (60, 62, 66)), ("PEAK", (60, 62, 66)), ("UNITS", (60, 62, 66)),
                                     ("MENU", (60, 62, 66)), ("I/O", (150, 30, 25)))):
        x = 20 + i * 98
        d.rounded_rectangle((x, 20, x + 84, 90), 14, fill=col, outline=(20, 20, 22), width=3)
        d.text((x + 10, 40), lab, font=ImageFont.truetype(FONT, 22), fill=(230, 230, 230))
panel("gauge_keys", (512, 110), (36, 38, 42), keys)


# stand control panel (front face of the base): membrane keys
def ctrl(d):
    d.text((14, 6), "TEST STAND CONTROL", font=ImageFont.truetype(FONT, 20), fill=(215, 215, 215))
    for i, (lab, col) in enumerate((("<", (55, 58, 64)), ("STOP", (170, 30, 28)), (">", (55, 58, 64)),
                                     ("RUN", (30, 120, 60)))):
        x = 16 + i * 120
        d.rounded_rectangle((x, 40, x + 104, 108), 12, fill=col, outline=(15, 15, 15), width=3)
        d.text((x + 22, 56), lab, font=ImageFont.truetype(FONT, 30), fill=(240, 240, 240))
panel("stand_panel", (512, 120), (40, 42, 46), ctrl)


# crosshead travel read-out (emissive)
def travel(d):
    d.text((14, 8), "TRAVEL mm", font=ImageFont.truetype(FONT, 28), fill=(255, 150, 60))
    d.text((24, 48), "000.00", font=ImageFont.truetype(MONO, 92), fill=(255, 120, 30))
panel("travel_led", (420, 160), (18, 8, 4), travel)


# engraved mm scale on the rail: 300 mm long, ticks every 1 mm, labels every 10 mm
def scale(d):
    W, H = 3000, 120
    for mm in range(0, 301):
        x = int(mm / 300 * (W - 1))
        L = 70 if mm % 10 == 0 else (45 if mm % 5 == 0 else 28)
        d.line((x, 0, x, L), fill=(25, 25, 25), width=3 if mm % 10 == 0 else 2)
        if mm % 20 == 0 and 0 < mm < 300:
            d.text((x - 18, 76), str(mm - 150), font=ImageFont.truetype(FONT, 30), fill=(25, 25, 25))
im = Image.new("RGB", (3000, 120), (200, 202, 204))
scale(ImageDraw.Draw(im))
im.save(OUT / "scale_rail.png")


# ASTM specimen marking on the dogbone tab: specimen ID moulded/printed
def tag(d):
    d.text((8, 6), "D638-I  #07", font=ImageFont.truetype(FONT, 40), fill=(40, 40, 40))
panel("specimen_id", (300, 60), (236, 229, 205), tag)
print("textures ->", OUT)
