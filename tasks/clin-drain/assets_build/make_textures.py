"""Generate the PBR textures for the clin-drain visuals (top-disc print; absorbent-pad maps for the mat).

Run: python3 assets_build/make_textures.py <out_dir> [<SimReady absorbent-pad Textures dir>]
(PIL + numpy only; deterministic, seed 7)
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
rng = np.random.default_rng(7)
BOLD = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
GREEN = (18, 150, 118)  # lid ink of the catalogue photo


def font(path, size):
    return ImageFont.truetype(path, size)


def sheared_text(img, xy, text, fnt, fill, shear=0.18):
    """Draw text centred at xy, slanted like the heavy italic print of the real lid."""
    w, h = int(fnt.getlength(text)) + 40, fnt.size + 40
    tile = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    ImageDraw.Draw(tile).text((w / 2, h / 2), text, font=fnt, fill=fill, anchor="mm")
    tile = tile.transform(tile.size, Image.Transform.AFFINE, (1, shear, -shear * h / 2, 0, 1, 0),
                          resample=Image.Resampling.BICUBIC)
    img.alpha_composite(tile, (int(xy[0] - w / 2), int(xy[1] - h / 2)))


def top_label():
    """Top disc print, planar UV over the 112 mm disc: u = x / 0.112 + 0.5, v = y / 0.112 + 0.5 (part frame).

    Lid layout and green ink follow the closed-wound suction drainage reservoir in the cited catalogue photo (README),
    laid out around this model's inlet port (0, +36) mm and pouring spout (+62, -2) mm. Generic markings only: no
    brand word mark, maker's logo, address, CE mark, lot or catalogue number is printed.
    """
    n = 2048
    c = n / 2
    px = n / 0.112  # pixels per metre

    def at(x_mm, y_mm):
        return c + x_mm * 1e-3 * px, c - y_mm * 1e-3 * px

    def pt(mm):
        return int(round(mm * 1e-3 * px))

    base = np.full((n, n, 3), (238, 239, 235), float)
    base += rng.normal(0, 1.2, (n, n, 1))
    yy, xx = np.mgrid[0:n, 0:n]
    r = np.hypot(xx - c, yy - c) / px  # metres
    base[(r > 0.0505) & (r < 0.0525)] -= 12  # moulded rim step
    img = Image.fromarray(np.clip(base, 0, 255).astype(np.uint8)).convert("RGBA")
    d = ImageDraw.Draw(img)
    g = GREEN + (255,)
    sheared_text(img, at(23.0, 17.0), "400 ml", font(BOLD, pt(8.5)), g)
    d.text(at(23.0, 8.6), "COMPACT", font=font(BOLD, pt(3.4)), fill=g, anchor="mm")
    d.text(at(23.0, 4.4), "EVACUATOR", font=font(BOLD, pt(3.4)), fill=g, anchor="mm")
    d.text(at(0.0, 48.6), "SUCTION PORT", font=font(BOLD, pt(2.5)), fill=g, anchor="mm")
    d.text(at(0.0, 44.9), "INSERT TUBE COMPLETELY", font=font(BOLD, pt(2.0)), fill=g, anchor="mm")
    d.text(at(37.0, -10.0), "POURING", font=font(BOLD, pt(2.8)), fill=g, anchor="mm")
    d.text(at(37.0, -14.0), "SPOUT", font=font(BOLD, pt(2.8)), fill=g, anchor="mm")
    # caution triangle, instructions lines, single-use symbol
    tx, ty = at(-21.5, -11.4)
    s = pt(3.6)
    d.polygon([(tx, ty - s * 0.55), (tx - s * 0.6, ty + s * 0.5), (tx + s * 0.6, ty + s * 0.5)], outline=g, width=5)
    d.text((tx, ty + s * 0.1), "!", font=font(BOLD, pt(2.2)), fill=g, anchor="mm")
    d.text(at(-31.0, -21.0), "PLEASE SEE INSTRUCTIONS FOR USE", font=font(BOLD, pt(1.9)), fill=g, anchor="lm")
    d.text(at(-31.0, -24.4), "DO NOT RESTERILIZE OR REUSE", font=font(BOLD, pt(1.9)), fill=g, anchor="lm")
    ux, uy = at(-12.0, -11.5)
    ur = pt(1.9)
    d.ellipse([ux - ur, uy - ur, ux + ur, uy + ur], outline=g, width=5)
    d.text((ux, uy), "2", font=font(BOLD, pt(2.4)), fill=g, anchor="mm")
    d.line([ux - ur * 0.7, uy + ur * 0.7, ux + ur * 0.7, uy - ur * 0.7], fill=g, width=5)
    img = img.filter(ImageFilter.GaussianBlur(0.6))
    img.convert("RGB").save(OUT / "drain_top_label.png", optimize=True)


def pad_textures(src):
    """Mat textures: albedo and normal map of SimReady consumable_absorbent_pad_stack_a01 (4096 px, CC-BY-4.0),
    resampled to 2048 px JPEG. Its roughness/occlusion map is not used."""
    for key, name in (("alb", "pad_albedo.jpg"), ("nor", "pad_normal.jpg")):
        im = Image.open(Path(src) / f"t_consumable_absorbent_pad_stack_a01_{key}.png").convert("RGB")
        im.resize((2048, 2048), Image.Resampling.LANCZOS).save(OUT / name, quality=92, subsampling=0)


if __name__ == "__main__":
    top_label()
    if len(sys.argv) > 2:
        pad_textures(sys.argv[2])
    print("ok", sorted(p.name for p in OUT.iterdir()))
