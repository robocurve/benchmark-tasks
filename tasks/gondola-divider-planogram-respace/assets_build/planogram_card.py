"""Draw the planogram card texture (148 x 105 mm card, 1036 x 735 px): the shelf section seen from above, holes -10..+10
along the rear row, the outer dividers (black), the two re-spaced dividers at holes +-3 (red), lane widths 7-6-7 in
holes, and the glove boxes in the outer lanes. Generic, no brand or maker text.

Run: python3 -I assets_build/planogram_card.py <out.png>
"""
import sys

from PIL import Image, ImageDraw, ImageFont

W, H = 1036, 735
im = Image.new("RGB", (W, H), (250, 250, 246))
d = ImageDraw.Draw(im)


def font(px):
    for f in ("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf"):
        try:
            return ImageFont.truetype(f, px)
        except OSError:
            pass
    return ImageFont.load_default()


d.rectangle((0, 0, W - 1, 92), fill=(32, 78, 140))
d.text((34, 20), "PLANOGRAM  -  SECTION 4  /  SHELF 2", font=font(44), fill=(255, 255, 255))
x0, x1, ytop, ybot = 70, W - 70, 190, 560
pitch = (x1 - x0) / 20.0
d.rectangle((x0 - 14, ytop, x1 + 14, ybot), outline=(60, 60, 60), width=4, fill=(232, 233, 230))
for k in range(-10, 11):  # rear hole row
    x = x0 + (k + 10) * pitch
    d.ellipse((x - 7, ytop + 26, x + 7, ytop + 40), fill=(40, 40, 40))
    if k % 5 == 0:
        d.text((x - 16, ytop - 46), f"{k:+d}" if k else "0", font=font(30), fill=(40, 40, 40))
for k, col, w in ((-10, (20, 20, 20), 10), (10, (20, 20, 20), 10), (-3, (205, 30, 30), 12), (3, (205, 30, 30), 12)):
    x = x0 + (k + 10) * pitch
    d.line((x, ytop + 8, x, ybot - 8), fill=col, width=w)
for k in (-2, 2):  # old divider positions, dashed grey
    x = x0 + (k + 10) * pitch
    for yy in range(ytop + 12, ybot - 12, 26):
        d.line((x, yy, x, yy + 12), fill=(150, 150, 150), width=5)
for kc, n in ((-6.5, 7), (0, 6), (6.5, 7)):
    x = x0 + (kc + 10) * pitch
    d.text((x - 24, 330), str(n), font=font(96), fill=(205, 30, 30) if n == 6 else (30, 30, 30))
for s in (-1, 1):  # glove boxes in the outer lanes
    xc = x0 + (s * 6.5 + 10) * pitch
    d.rectangle((xc - 58, 450, xc + 58, 530), fill=(150, 196, 232), outline=(60, 110, 160), width=3)
d.text((x0 - 10, 590), "LANE WIDTHS IN HOLES (1 IN PITCH):  7 - 6 - 7", font=font(38), fill=(30, 30, 30))
d.text((x0 - 10, 648), "MOVE MIDDLE DIVIDERS  +2 -> +3  AND  -2 -> -3", font=font(34), fill=(205, 30, 30))
im.save(sys.argv[1], optimize=True)
