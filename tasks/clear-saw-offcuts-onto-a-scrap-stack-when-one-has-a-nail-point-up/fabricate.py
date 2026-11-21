"""Build files for the prepared and printed parts, from the same dimensions as the scene
(clear_saw_offcuts_onto_a_scrap_stack_when_one_has_a_nail_point_up.py), in mm:
  - offcut_nailed.stl: 220 x 64 x 38 mm Western Red Cedar 2x3 offcut with the 20d nail driven up 35 mm from the +y end
    (64 mm proud), usable as a drilling/protrusion gauge for preparing the real wood-and-nail test piece.
  - stack_bearers.stl: the two 130 x 89 x 38 mm 2x4 bearers spaced 21 mm apart along y, usable as a placement template.
  - label_nailed_end.stl, label_nailed_side.stl, label_nailed_floor.stl: raised-letter warning plates (print in yellow
    with a filament change to black for the raised letters, or print on yellow adhesive vinyl).

    python3 fabricate.py [out_dir]      writes ASCII STL files (mm) and prints the hardware and cut list

Standard library only; nothing is written on import.
"""

import math
import os
import sys

# Scene dimensions (mm)
OFFCUT_W, OFFCUT_H = 64.0, 38.0  # actual 2x3 (1.5 x 2.5 in)
LEN_A, LEN_N, LEN_B = 180.0, 220.0, 200.0
NAIL_Y = LEN_N / 2 - 35.0  # +75 mm from board centre (35 mm from +y end)
NAIL_R, NAIL_SHANK, NAIL_PROUD = 2.45, 58.0, 64.0  # 20d bright common nail (4.9 mm shank, 102 mm total, 64 mm proud)
BEARER_X, BEARER_Y, BEARER_H, BEARER_HALF_SEP = 130.0, 89.0, 38.0, 55.0  # actual 2x4 laid flat, 21 mm gap along y

_GLYPHS = {
    "N": [((0.08, 0.0), (0.08, 1.0)), ((0.92, 0.0), (0.92, 1.0)), ((0.08, 0.94), (0.92, 0.06))],
    "A": [((0.06, 0.0), (0.46, 1.0)), ((0.94, 0.0), (0.54, 1.0)), ((0.22, 0.36), (0.78, 0.36))],
    "I": [((0.50, 0.0), (0.50, 1.0)), ((0.12, 0.94), (0.88, 0.94)), ((0.12, 0.06), (0.88, 0.06))],
    "L": [((0.12, 0.0), (0.12, 1.0)), ((0.12, 0.06), (0.94, 0.06))],
    "E": [((0.12, 0.0), (0.12, 1.0)), ((0.12, 0.94), (0.94, 0.94)), ((0.12, 0.50), (0.82, 0.50)), ((0.12, 0.06), (0.94, 0.06))],
    "D": [((0.12, 0.0), (0.12, 1.0)), ((0.12, 0.94), (0.72, 0.94)), ((0.12, 0.06), (0.72, 0.06)),
          ((0.72, 0.94), (0.92, 0.72)), ((0.92, 0.72), (0.92, 0.28)), ((0.92, 0.28), (0.72, 0.06))],
    "S": [((0.10, 0.94), (0.92, 0.94)), ((0.10, 0.94), (0.10, 0.50)), ((0.10, 0.50), (0.92, 0.50)),
          ((0.92, 0.50), (0.92, 0.06)), ((0.10, 0.06), (0.92, 0.06))],
    "C": [((0.12, 0.06), (0.12, 0.94)), ((0.12, 0.94), (0.92, 0.94)), ((0.12, 0.06), (0.92, 0.06))],
    "R": [((0.12, 0.0), (0.12, 1.0)), ((0.12, 0.94), (0.86, 0.94)), ((0.86, 0.94), (0.86, 0.52)),
          ((0.12, 0.52), (0.86, 0.52)), ((0.44, 0.52), (0.92, 0.0))],
    "P": [((0.12, 0.0), (0.12, 1.0)), ((0.12, 0.94), (0.88, 0.94)), ((0.88, 0.94), (0.88, 0.50)),
          ((0.12, 0.50), (0.88, 0.50))],
}


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _rot_box_xy(cx, cy, z0, z1, length, width, angle):
    ca, sa = math.cos(angle), math.sin(angle)
    hl, hw = length / 2, width / 2
    pts2 = []
    for sx, sy in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)):
        pts2.append((cx + ca * sx - sa * sy, cy + sa * sx + ca * sy))
    v = [(x, y, z0) for x, y in pts2] + [(x, y, z1) for x, y in pts2]
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _lathe_z(cx, cy, z_base, profile, sides=16):
    tris = []
    angs = [2.0 * math.pi * i / sides for i in range(sides)]
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        zb0, zb1 = z_base + z0, z_base + z1
        for i in range(sides):
            j = (i + 1) % sides
            c0, s0 = math.cos(angs[i]), math.sin(angs[i])
            c1, s1 = math.cos(angs[j]), math.sin(angs[j])
            p00 = (cx + r0 * c0, cy + r0 * s0, zb0)
            p01 = (cx + r0 * c1, cy + r0 * s1, zb0)
            p10 = (cx + r1 * c0, cy + r1 * s0, zb1)
            p11 = (cx + r1 * c1, cy + r1 * s1, zb1)
            if r0 > 0 and r1 > 0:
                tris += [(p00, p01, p11), (p00, p11, p10)]
            elif r0 > 0:
                tris.append((p00, p01, (cx, cy, zb1)))
            else:
                tris.append(((cx, cy, zb0), p11, p10))
    return tris


def offcut_nailed():
    """Origin at the board's bottom centre; nail proud above z = 38 mm at y = +75 mm."""
    tris = _box(-OFFCUT_W / 2, -LEN_N / 2, 0.0, OFFCUT_W / 2, LEN_N / 2, OFFCUT_H)
    tris += _lathe_z(0.0, NAIL_Y, OFFCUT_H, [(NAIL_R, 0.0), (NAIL_R, NAIL_SHANK), (0.0, NAIL_PROUD)])
    return tris


def stack_bearers():
    """Two 130 x 89 x 38 mm 2x4 bearers centred at y = +-55 mm (21 mm gap along y)."""
    hx, hy = BEARER_X / 2, BEARER_Y / 2
    t = _box(-hx, -BEARER_HALF_SEP - hy, 0.0, hx, -BEARER_HALF_SEP + hy, BEARER_H)
    t += _box(-hx, BEARER_HALF_SEP - hy, 0.0, hx, BEARER_HALF_SEP + hy, BEARER_H)
    return t


def label_plate(text, pw, ph, pt, letter_h, letter_w, gap, stroke, raise_t):
    """Flat backing plate on z = 0..pt with raised block letters on z = pt..pt + raise_t."""
    tris = _box(-pw / 2, -ph / 2, 0.0, pw / 2, ph / 2, pt)
    widths = [letter_w * 0.55 if ch == " " else letter_w for ch in text]
    total_w = sum(widths) + gap * max(0, len(text) - 1)
    x_cur = -total_w / 2
    for ch, cw in zip(text, widths):
        if ch != " ":
            for (u0, v0), (u1, v1) in _GLYPHS[ch]:
                x0, y0 = x_cur + u0 * cw, (v0 - 0.5) * letter_h
                x1, y1 = x_cur + u1 * cw, (v1 - 0.5) * letter_h
                du, dv = x1 - x0, y1 - y0
                seg_len = math.hypot(du, dv) + stroke * 0.55
                tris += _rot_box_xy((x0 + x1) / 2, (y0 + y1) / 2, pt, pt + raise_t, seg_len, stroke, math.atan2(dv, du))
        x_cur += cw + gap
    return tris


CUT_LIST = """Hardware and cut list (mm):
  Offcuts (3)       Kiln-dried Western Red Cedar 2x3 lumber (38 x 64 actual; Home Depot SKU 535265 / Model #2112000):
                      A: 180 mm (0.175 kg), N: 220 mm (0.214 kg cedar), B: 200 mm (0.195 kg)
  Nail (1)          20d bright steel common nail (102 mm, 4.9 mm shank, 16 g; Grip-Rite 20C1 / Hillman 461366),
                    driven up through N 35 mm from its +y end so 64 mm stands proud; file the tip blunt (1 mm radius)
                    for real-world trials so staff and gripper pads stay safe
  Stack bearers (2) Pressure-treated 2x4 Southern Pine lumber (38 x 89 actual; Home Depot SKU 218458 / Lowe's #2622584):
                    2 x 130 mm laid flat, fixed 21 mm apart along y (centres x 465, y -270 and -160)
  Tote (1)          Dark grey straight-wall polypropylene open shoebox tote, inside ~300 x 160 x 120 mm
                    (Really Useful Box 5 L or Sterilite 6 Qt 1642, lid removed), taped down at x 295, y +205
  Labels (4)        Safety-yellow vinyl or printed plates with black lettering: NAILED on near outer wall
                    (156 x 74), far inner wall (152 x 72) and inside floor (228 x 130); NAILED SCRAP on
                    -y long wall (290 x 60)"""


def _write(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            f.write("  facet normal 0 0 0\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    parts = {
        "offcut_nailed": offcut_nailed(),
        "stack_bearers": stack_bearers(),
        "label_nailed_end": label_plate("NAILED", 156.0, 74.0, 1.5, 54.0, 18.5, 5.0, 6.5, 1.5),
        "label_nailed_side": label_plate("NAILED SCRAP", 290.0, 60.0, 1.5, 40.0, 17.0, 4.5, 5.5, 1.5),
        "label_nailed_floor": label_plate("NAILED", 228.0, 130.0, 1.0, 88.0, 27.0, 8.5, 8.5, 1.0),
    }
    for name, tris in parts.items():
        p = os.path.join(out, name + ".stl")
        _write(p, name, tris)
        print("wrote", p)
    print(CUT_LIST)
