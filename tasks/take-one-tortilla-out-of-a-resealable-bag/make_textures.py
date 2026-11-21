"""Write the textures under textures/: a flour tortilla (pale with toasted spots) and the bag's two films (clear
film, a generic printed front panel, the zip strip). Deterministic. ``python make_textures.py``."""

from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageOps, ImageFilter, ImageFont

import layout as L

OUT = Path(__file__).with_name("textures")


def _noise(rng, n, scale):
    small = rng.random((max(2, n // scale), max(2, n // scale)))
    img = Image.fromarray((small * 255).astype(np.uint8)).resize((n, n), Image.BICUBIC)
    return np.asarray(img, float) / 255.0


def tortilla(seed=0, n=1024):
    rng = np.random.default_rng(seed)
    base = np.array([238, 222, 186], float)  # pale wheat
    img = np.ones((n, n, 3)) * base
    tone = 0.6 * _noise(rng, n, 64) + 0.4 * _noise(rng, n, 16)
    img *= (0.94 + 0.08 * tone)[..., None]
    # toasted spots: brown blotches of many sizes, darker in their middle
    spots = np.zeros((n, n))
    yy, xx = np.mgrid[0:n, 0:n] / n
    for _ in range(220):
        cx, cy = rng.random(2)
        r = rng.choice([0.004, 0.007, 0.012, 0.02], p=[0.45, 0.3, 0.17, 0.08]) * rng.uniform(0.6, 1.4)
        sx, sy = rng.uniform(0.6, 1.6, 2)
        d = ((xx - cx) / (r * sx)) ** 2 + ((yy - cy) / (r * sy)) ** 2
        spots = np.maximum(spots, np.exp(-d * 1.6) * rng.uniform(0.45, 1.0))
    spots = np.asarray(Image.fromarray((spots * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(1.2)),
                       float) / 255
    brown = np.array([150, 98, 48], float)
    img = img * (1 - 0.75 * spots[..., None]) + brown * 0.75 * spots[..., None]
    # a faint lighter rim, where the dough was pressed thinner
    r = np.hypot(xx - 0.5, yy - 0.5) * 2
    img *= (1 + 0.03 * np.clip((r - 0.85) / 0.15, 0, 1))[..., None]
    return Image.fromarray(np.clip(img, 0, 255).astype(np.uint8))


def _font(size):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
              "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"):
        if Path(f).exists():
            return ImageFont.truetype(f, size)
    return ImageFont.load_default()


def film(which, w=1024):
    """RGBA over the film's uv: u across the mouth (0 at -u), v from the bottom lip's edge (0) to the bottom seal."""
    total_v = L.IN_L + L.BOTTOM_SEAL + L.LIP_BOTTOM
    total_u = L.IN_W + 2 * L.SEAL_W
    h = int(round(w * total_v / total_u))
    img = Image.new("RGBA", (w, h), (235, 240, 245, 26))  # clear film: faint, slightly blue-grey
    d = ImageDraw.Draw(img)

    def vpix(v):  # v measured from the zip into the bag (m) -> pixel row; uv v=0 is the image's bottom row
        return h - 1 - int(round((v + L.LIP_BOTTOM) / total_v * (h - 1)))

    def upix(u):
        return int(round((u + total_u / 2) / total_u * (w - 1)))

    # heat seals: frosted bands
    seal = (240, 244, 248, 90)
    d.rectangle([0, 0, upix(-L.IN_W / 2), h], fill=seal)
    d.rectangle([upix(L.IN_W / 2), 0, w, h], fill=seal)
    d.rectangle([0, 0, w, vpix(L.IN_L)], fill=seal)
    # the zip: two pale blue ribs a few mm apart
    for dv in (-0.0045, -0.0015):
        d.rectangle([0, vpix(dv + 0.0015), w, vpix(dv)], fill=(120, 170, 215, 210))
    if which == "top":
        # a generic printed front panel (no brand): a warm band with plain words
        v0, v1 = 0.150, 0.225
        d.rectangle([upix(-0.10), vpix(v1), upix(0.10), vpix(v0)], fill=(196, 70, 36, 235))
        d.rectangle([upix(-0.10), vpix(v1) + 8, upix(0.10), vpix(v1) + 16], fill=(250, 200, 60, 245))
        # the words go on their own layer, mirrored in u: the film's u runs right-to-left seen from outside the bag
        words = Image.new("RGBA", (w, h), (0, 0, 0, 0))
        dw = ImageDraw.Draw(words)
        f1, f2 = _font(int(w * 0.07)), _font(int(w * 0.035))
        for text, font, v in (("FLOUR", f1, 0.212), ("TORTILLAS", f1, 0.190), ("8 IN  ·  SOFT", f2, 0.165)):
            tw = dw.textlength(text, font=font)
            dw.text(((w - tw) / 2, vpix(v)), text, font=font, fill=(255, 248, 235, 250))
        f3 = _font(int(w * 0.028))
        t = "PRESS TO SEAL"
        dw.text(((w - dw.textlength(t, font=f3)) / 2, vpix(0.016)), t, font=f3, fill=(70, 120, 175, 200))
        img.alpha_composite(ImageOps.mirror(words))
    else:
        f3 = _font(int(w * 0.025))
        for i, line in enumerate(("KEEP REFRIGERATED AFTER OPENING", "BEST BY: SEE ZIP")):
            d.text((upix(-0.09), vpix(0.12 - 0.012 * i)), line, font=f3, fill=(60, 60, 60, 120))
    return img


if __name__ == "__main__":
    OUT.mkdir(exist_ok=True)
    for k in range(L.N_TORTILLAS):
        tortilla(seed=k).save(OUT / f"tortilla{k}.png")
    # the films' uv is normalised, so their images are resampled to a power-of-two square (the renderer did not
    # bind a 1024 x 1316 RGBA texture on the deformable films)
    for which in ("top", "bottom"):
        film(which).resize((1024, 1024), Image.LANCZOS).save(OUT / f"film_{which}.png")
    print("wrote", sorted(p.name for p in OUT.iterdir()))
