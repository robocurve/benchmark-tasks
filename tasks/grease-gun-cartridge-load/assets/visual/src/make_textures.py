"""Generate the PBR textures for the grease-gun visual meshes (deterministic, synthetic): python3 make_textures.py OUT_DIR"""

import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
rng = np.random.default_rng(11)
FONT_B = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def noise(n, scale):
    k = max(2, n // scale)
    small = rng.random((k, k))
    return np.asarray(Image.fromarray((small * 255).astype(np.uint8)).resize((n, n), Image.BICUBIC), float) / 255


# cartridge label: wraps the 240 mm tube (u = around, v = along); generic multipurpose grease, no brand
W, H = 1024, 1536
img = Image.new("RGB", (W, H), (24, 58, 140))
d = ImageDraw.Draw(img)
d.rectangle([0, 0, W, 120], fill=(230, 196, 40))
d.rectangle([0, H - 120, W, H], fill=(230, 196, 40))
fb = ImageFont.truetype(FONT_B, 96)
f = ImageFont.truetype(FONT, 48)
for k, (txt, font, y) in enumerate((("MULTI-PURPOSE", fb, 380), ("GREASE", fb, 500), ("NLGI 2  LITHIUM", f, 680),
                                     ("14 oz (397 g) CARTRIDGE", f, 780), ("OPEN END FIRST", f, 1000))):
    for x0 in (40, 552):
        d.text((x0, y), txt, fill=(245, 245, 240), font=font)
lab = np.asarray(img, float) / 255
lab *= (0.92 + 0.08 * noise(H, 64)[:, :W, None]) if H == W else (0.92 + 0.08 * noise(max(W, H), 64)[:H, :W, None])
Image.fromarray(np.clip(lab * 255, 0, 255).astype(np.uint8)).save(OUT / "cartridge_label.png")

# ribbed rubber mat: dark rubber with 6 mm ribs along x and fine grain
n = 1024
ribs = 0.5 + 0.5 * np.sin(np.linspace(0, 2 * np.pi * 90, n))[None, :] * np.ones((n, 1))
grain = noise(n, 6)
base = 0.075 + 0.03 * ribs + 0.02 * grain
Image.fromarray(np.clip(np.stack([base, base, base * 1.05], -1) * 255, 0, 255).astype(np.uint8)).save(OUT / "mat_albedo.png")
Image.fromarray(np.clip((0.75 + 0.15 * grain - 0.1 * ribs) * 255, 0, 255).astype(np.uint8)).save(OUT / "mat_rough.png")

# brushed zinc-plated steel
streak = np.repeat(rng.random((1, n)), n, 0)
streak = np.asarray(Image.fromarray((streak * 255).astype(np.uint8)).resize((n, n), Image.BILINEAR), float) / 255
st = 0.70 + 0.10 * streak + 0.04 * noise(n, 32)
Image.fromarray(np.clip(np.stack([st, st * 1.01, st * 1.03], -1) * 255, 0, 255).astype(np.uint8)).save(OUT / "steel_albedo.png")

# shop rag: faded red cotton with weave
wv = 0.5 + 0.25 * np.sin(np.arange(n) * 1.3)[None, :] + 0.25 * np.sin(np.arange(n) * 1.3)[:, None]
rg = 0.85 + 0.15 * wv * noise(n, 48)
Image.fromarray(np.clip(np.stack([0.62 * rg, 0.16 * rg, 0.14 * rg], -1) * 255, 0, 255).astype(np.uint8)).save(OUT / "rag_albedo.png")
print("textures ->", OUT)
