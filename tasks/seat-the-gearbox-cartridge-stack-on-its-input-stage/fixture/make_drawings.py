"""Writes the two full-size (1:1) drawings of the parts to make: motor-stand-1to1.svg and cradle-1to1.svg.

    python3 make_drawings.py

Millimetres throughout; print at 100 % and check the 50 mm bar before cutting or printing.
"""

from pathlib import Path

HERE = Path(__file__).resolve().parent
FONT = 'font-family="Helvetica, Arial, sans-serif"'


class Sheet:
    def __init__(self, w, h, title, notes):
        self.w, self.h, self.items = w, h, []
        self.text(10, 12, title, 5.0)
        for i, n in enumerate(notes):
            self.text(10, 20 + 4.6 * i, n, 3.2)

    def text(self, x, y, s, size=3.2, anchor="start"):
        self.items.append(f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" {FONT} text-anchor="{anchor}" '
                          f'fill="#000">{s}</text>')

    def rect(self, x, y, w, h, fill="#dddddd", stroke="#000", sw=0.3, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" fill="{fill}" '
                          f'stroke="{stroke}" stroke-width="{sw}"{d}/>')

    def circle(self, cx, cy, r, fill="white", stroke="#000", sw=0.3, dash=None):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.items.append(f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{r:.2f}" fill="{fill}" stroke="{stroke}" '
                          f'stroke-width="{sw}"{d}/>')

    def line(self, x1, y1, x2, y2, sw=0.2, color="#000"):
        self.items.append(f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" stroke="{color}" '
                          f'stroke-width="{sw}"/>')

    def dim_h(self, x1, x2, y, label):
        self.line(x1, y, x2, y)
        self.line(x1, y - 2, x1, y + 2)
        self.line(x2, y - 2, x2, y + 2)
        self.text((x1 + x2) / 2, y - 1.2, label, 3.0, "middle")

    def dim_v(self, x, y1, y2, label):
        self.line(x, y1, x, y2)
        self.line(x - 2, y1, x + 2, y1)
        self.line(x - 2, y2, x + 2, y2)
        self.text(x + 2.5, (y1 + y2) / 2 + 1, label, 3.0)

    def bar(self, x, y):
        self.rect(x, y, 50, 3, fill="#000")
        self.text(x + 25, y + 8, "50 mm check bar", 3.0, "middle")

    def save(self, name):
        body = "\n".join(self.items)
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}mm" height="{self.h}mm" '
               f'viewBox="0 0 {self.w} {self.h}">\n<rect width="{self.w}" height="{self.h}" fill="white"/>\n{body}\n</svg>\n')
        (HERE / name).write_text(svg)


def motor_stand():
    s = Sheet(200, 280, "Motor stand: full size (1:1), mm. PLA, 100 % or 40 % infill, 0.2 mm layers", [
        "Holds a NEO V1.1 motor (60 mm across, 58.25 mm long) upright, shaft up, its back end 5 mm above the bench.",
        "Pocket 61 x 61 mm square, 25 mm deep (the simulation's pocket; a 60.5 mm round pocket also serves).",
        "A 12 mm wide cable slot in one wall, full depth, for the motor's leads. Two 5 mm holes for bench screws.",
        "Two M4 clamp screws (nylon-tipped thumb screws in heat-set inserts) through adjacent walls, 15 mm up, hold the motor.",
        "Its centre is the motor axis: 290 mm forward of the midpoint between the arm bases, on the centre line.",
    ])
    ox, oy = 30, 62  # plan, top-left
    s.text(ox, oy - 4, "PLAN", 4)
    s.rect(ox, oy, 80, 80)
    s.rect(ox + 9.5, oy + 9.5, 61, 61, fill="white")
    s.rect(ox + 74, oy + 34, 6, 12, fill="white", dash="1,1")
    s.text(ox + 81, oy + 41, "cable slot 12 wide", 2.6)
    s.circle(ox + 40, oy + 40, 30, fill="none", dash="2,1")
    s.text(ox + 40, oy + 42, "motor 60", 3.0, "middle")
    for hx in (ox + 4.5, ox + 75.5):
        s.circle(hx, oy + 4.5, 2.5)
    for (x1, y1, x2, y2, lab, tx, ty) in ((ox, oy + 40, ox + 9.5, oy + 40, "M4 clamp", ox + 11, oy + 37),
                                          (ox + 40, oy, ox + 40, oy + 9.5, "M4 clamp", ox + 42, oy - 1)):
        s.line(x1, y1, x2, y2, sw=1.2, color="#444444")
        s.text(tx, ty, lab, 2.6)
    s.dim_h(ox, ox + 80, oy + 88, "80")
    s.dim_h(ox + 9.5, ox + 70.5, oy + 76, "61")
    s.dim_v(ox - 8, oy, oy + 80, "80")
    sx, sy = 30, 212  # section
    s.text(sx, sy - 4, "SECTION through the centre", 4)
    s.rect(sx, sy, 80, 30)
    s.rect(sx + 9.5, sy, 61, 25, fill="white")
    s.rect(sx + 10, sy - 33.25, 60, 58.25, fill="none", dash="2,1")
    s.text(sx + 40, sy - 15, "motor (bought)", 3.0, "middle")
    s.dim_v(sx + 88, sy, sy + 30, "30")
    s.dim_v(sx + 100, sy + 25, sy + 30, "floor 5")
    s.bar(130, 262)
    s.save("motor-stand-1to1.svg")


def cradle():
    s = Sheet(200, 200, "Stack cradle: full size (1:1), mm. PLA or plywood with glued posts", [
        "A 70 x 70 x 5 mm base with four 12 x 12 x 20 mm posts, centres 19 mm from the middle each way.",
        "The gearbox stack stands on the four post tops (25 mm above the bench), its spline and tabs in the gaps.",
        "Loose on the bench. Lot A: centre 200 mm forward, 140 mm to the robot's right, turned 30 deg counter-clockwise.",
        "Lot B: 205 mm forward, 140 mm to the robot's left, turned 30 deg clockwise (seen from above).",
    ])
    ox, oy = 40, 50
    s.text(ox, oy - 4, "PLAN", 4)
    s.rect(ox, oy, 70, 70)
    for px in (-19, 19):
        for py in (-19, 19):
            s.rect(ox + 35 + px - 6, oy + 35 + py - 6, 12, 12, fill="#999999")
    s.rect(ox + 35 - 25.4, oy + 35 - 25.4, 50.8, 50.8, fill="none", dash="2,1")
    s.text(ox + 35, oy + 37, "stack outline 50.8", 2.8, "middle")
    s.dim_h(ox, ox + 70, oy + 78, "70")
    s.dim_h(ox + 35 - 19, ox + 35 + 19, oy + 88, "38 (post centres)")
    sx, sy = 40, 160
    s.text(sx, sy - 4, "ELEVATION", 4)
    s.rect(sx, sy + 20, 70, 5)
    for px in (-19, 19):
        s.rect(sx + 35 + px - 6, sy, 12, 20, fill="#999999")
    s.dim_v(sx + 78, sy, sy + 25, "25")
    s.bar(130, 180)
    s.save("cradle-1to1.svg")


if __name__ == "__main__":
    motor_stand()
    cradle()
