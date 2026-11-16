"""Author the printed-ink textures for the cap1 visual models (system python3 + Pillow).

tube_print.png  RGBA, wraps the tube's printed sector (see make_visuals.py PRINT_*): mL graduations computed from the
                17 x 100 mm round-bottom tube's inner volume (r_in 7.5 mm hemisphere + cylinder) and a frosted
                writing patch at z = 56..78 mm (SCENE.md). Generic laboratory print, not a copy of vendor artwork.
rack_label.png  RGB, the white well-number strip on the rack's -X face.
"""
import math
import pathlib
import sys

from PIL import Image, ImageDraw, ImageFont

OUT = pathlib.Path(sys.argv[1])
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
INK = (22, 40, 92, 255)  # dark-blue printing ink

# Must match make_visuals.py
Z0, Z1 = 0.010, 0.086           # printed sector height on the tube body (m)
A0, A1 = 0.40, 0.70             # printed sector as a fraction of the circumference (0.5 faces -X)
R_OUT = 0.0085
R_IN = 0.0075                   # inner radius: hemisphere bottom + cylinder

H = 2048
W = int(round(H * (A1 - A0) * 2 * math.pi * R_OUT / (Z1 - Z0)))  # square texels on the tube
img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
d = ImageDraw.Draw(img)
arc_m = (A1 - A0) * 2 * math.pi * R_OUT  # width of the sector in metres


def px_u(frac_of_circ):  # absolute circumference fraction -> image x
    return int((frac_of_circ - A0) / (A1 - A0) * W)


def px_z(z):  # metres above tube bottom -> image y (top row = Z1)
    return int((Z1 - z) / (Z1 - Z0) * H)


def z_of_ml(ml):
    hemi = 2.0 / 3.0 * math.pi * R_IN ** 3 * 1e6  # mL
    return 0.0010 + R_IN + (ml - hemi) / (math.pi * R_IN ** 2 * 1e6)  # 1 mm wall at the bottom


font = ImageFont.truetype(FONT, 58)
small = ImageFont.truetype(FONT, 40)
x_tick = px_u(0.50)
for half in range(2, 29):
    ml = half / 2
    z = z_of_ml(ml)
    if not Z0 + 0.002 < z < Z1 - 0.002:
        continue
    y = px_z(z)
    major = half % 2 == 0
    L = 62 if major else 36
    d.rectangle([x_tick - L, y - 4, x_tick, y + 4], fill=INK)
    if major and int(ml) % 2 == 0:
        d.text((x_tick - L - 12, y), f"{int(ml)}", font=font, fill=INK, anchor="rm")
# vertical spine of the scale and the unit
d.rectangle([x_tick, px_z(z_of_ml(14)) - 4, x_tick + 6, px_z(z_of_ml(1)) + 4], fill=INK)
d.text((x_tick - 8, px_z(z_of_ml(14)) - 60), "mL", font=small, fill=INK, anchor="rm")

# frosted writing patch (z = 56..78 mm) beside the scale, with faint printed rules
x0, x1 = px_u(0.52), px_u(0.685)
d.rectangle([x0, px_z(0.078), x1, px_z(0.056)], fill=(236, 238, 236, 255))
for k in range(1, 4):
    y = px_z(0.056 + k * 0.0055)
    d.line([x0 + 14, y, x1 - 14, y], fill=(150, 160, 175, 255), width=3)
d.rectangle([x0, px_z(0.078), x1, px_z(0.056)], outline=INK, width=5)
img.save(OUT / "tube_print.png")

# rack label strip: 130 x 4.5 mm, three well numbers above the tube centres (y = +50, 0, -50 mm)
LW, LH = 2048, 72
lab = Image.new("RGB", (LW, LH), (232, 234, 236))
dl = ImageDraw.Draw(lab)
f2 = ImageFont.truetype(FONT, 56)
for i, y_mm in enumerate((50.0, 0.0, -50.0)):
    # strip faces -X: viewer's left is +Y, so u = (65 - y)/130
    u = (65.0 - y_mm) / 130.0
    dl.text((int(u * LW), LH // 2), f"{i + 1}", font=f2, fill=(30, 34, 40), anchor="mm")
dl.text((40, LH // 2), "17 mm", font=ImageFont.truetype(FONT, 44), fill=(30, 34, 40), anchor="lm")
lab.save(OUT / "rack_label.png")
print("textures ->", OUT, "sector width mm %.1f" % (arc_m * 1e3))
