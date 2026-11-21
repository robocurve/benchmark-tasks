"""Writes zones-1to1.svg: the 1:1 dusting stencil and photo overlay for the three pads of a Robotiq 2F-85
(AGC-TIP-MLD-KIT-85 fingertips). Print at 100% (check the 50 mm scale bar), cut the zone windows out of thin card.
Each window is one zone; the thin grid lines show the 5-6 mm cells the simulation scores (count dusty cells in the
after photo: a zone is clean when at most 10% of its area still shows chalk)."""
W, H = 150, 110  # mm page


def rect(x, y, w, h, stroke=0.3, fill="none", dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="{fill}" stroke="black" stroke-width="{stroke}"{d}/>'


def text(x, y, s, size=3.0):
    return f'<text x="{x:.2f}" y="{y:.2f}" font-family="Helvetica" font-size="{size}">{s}</text>'


out = []
# palm pad: 31 x 22 mm, zones = left and right halves (as seen from the fingertips), 6 x 4 cells
px, py = 10, 15
out += [rect(px, py, 31, 22, 0.5), text(px, py - 3, "Palm pad 31 x 22 mm (seen from the fingertips)")]
out += [f'<line x1="{px + 15.5}" y1="{py}" x2="{px + 15.5}" y2="{py + 22}" stroke="black" stroke-width="0.5"/>']
for i in range(1, 6):
    out.append(f'<line x1="{px + i * 31 / 6:.2f}" y1="{py}" x2="{px + i * 31 / 6:.2f}" y2="{py + 22}" stroke="grey" stroke-width="0.1"/>')
for j in range(1, 4):
    out.append(f'<line x1="{px}" y1="{py + j * 5.5:.2f}" x2="{px + 31}" y2="{py + j * 5.5:.2f}" stroke="grey" stroke-width="0.1"/>')
out += [text(px + 2, py + 26, "zone 1 (robot's left)", 2.2), text(px + 17, py + 26, "zone 2 (robot's right)", 2.2)]
# fingertip pads: 16.2 x 38 mm rubber face, zones = thirds along the finger (12.7 mm), 3 x 6 cells
for k, (fx, name, z0) in enumerate(((60, "Left fingertip", 3), (95, "Right fingertip", 6))):
    fy = 15
    out += [rect(fx, fy, 16.2, 38, 0.5), text(fx - 4, fy - 3, f"{name} pad 16.2 x 38 mm")]
    for t in range(1, 3):
        out.append(f'<line x1="{fx}" y1="{fy + t * 38 / 3:.2f}" x2="{fx + 16.2}" y2="{fy + t * 38 / 3:.2f}" stroke="black" stroke-width="0.5"/>')
    for i in range(1, 3):
        out.append(f'<line x1="{fx + i * 5.4:.2f}" y1="{fy}" x2="{fx + i * 5.4:.2f}" y2="{fy + 38}" stroke="grey" stroke-width="0.1"/>')
    for j in range(1, 6):
        out.append(f'<line x1="{fx}" y1="{fy + j * 38 / 6:.2f}" x2="{fx + 16.2}" y2="{fy + j * 38 / 6:.2f}" stroke="grey" stroke-width="0.1"/>')
    out += [text(fx + 17.5, fy + 5, f"zone {z0 + 2} (tip end)", 2.2), text(fx + 17.5, fy + 19, f"zone {z0 + 1} (middle)", 2.2),
            text(fx + 17.5, fy + 32, f"zone {z0} (palm end)", 2.2)]
# scale bar
out += [rect(10, 95, 50, 2, 0.3, "black"), text(10, 93, "50 mm (check the print scale)")]
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}mm" height="{H}mm" viewBox="0 0 {W} {H}">'
       + "".join(out) + "</svg>\n")
open(__file__.replace("make_stencil.py", "zones-1to1.svg"), "w").write(svg)
print("wrote zones-1to1.svg")
