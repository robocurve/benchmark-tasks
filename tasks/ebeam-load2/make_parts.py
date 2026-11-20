"""Write DXF cutting files (mm) for the real-trial hardware of this task: python make_parts.py [out_dir].

Laser-cut parts: the stainless sample plate, the eight clip tabs, the two layers of the aluminium ring holder, the
loading stand's top plate and the load lock's door. Plain DXF R12 (LINE, CIRCLE, ARC), readable by any laser-cutting
service. Threads are tapped by hand after cutting (see README, Real-trial hardware).
"""
import math
import os
import sys

TAP_M3 = 2.5      # M3 x 0.5 tap drill
CLEAR_M3 = 3.2    # M3 clearance


def _dxf(entities):
    out = ["0", "SECTION", "2", "ENTITIES"]
    for e in entities:
        if e[0] == "C":
            _, x, y, r = e
            out += ["0", "CIRCLE", "8", "0", "10", f"{x:.4f}", "20", f"{y:.4f}", "30", "0", "40", f"{r:.4f}"]
        elif e[0] == "L":
            _, x0, y0, x1, y1 = e
            out += ["0", "LINE", "8", "0", "10", f"{x0:.4f}", "20", f"{y0:.4f}", "30", "0",
                    "11", f"{x1:.4f}", "21", f"{y1:.4f}", "31", "0"]
        elif e[0] == "A":
            _, x, y, r, a0, a1 = e
            out += ["0", "ARC", "8", "0", "10", f"{x:.4f}", "20", f"{y:.4f}", "30", "0", "40", f"{r:.4f}",
                    "50", f"{a0:.4f}", "51", f"{a1:.4f}"]
    out += ["0", "ENDSEC", "0", "EOF"]
    return "\n".join(out) + "\n"


def _ring_holes(r, n, off, d):
    return [("C", r * math.cos(math.radians(off + 360.0 * i / n)), r * math.sin(math.radians(off + 360.0 * i / n)), d / 2)
            for i in range(n)]


def _rect(x0, y0, x1, y1):
    return [("L", x0, y0, x1, y0), ("L", x1, y0, x1, y1), ("L", x1, y1, x0, y1), ("L", x0, y1, x0, y0)]


def sample_plate():
    """304 stainless, 1.5 mm: 200 mm disc; M3 tap holes in rings for 2, 3, 4 and 6 inch wafers (the 4 inch ring, 59.5
    mm out, every 45 degrees from +x, carries the eight clips); four M3 tap holes at 14 mm radius (28 mm pitch-circle diameter) for the boss."""
    e = [("C", 0.0, 0.0, 100.0)]
    for r, n, off in ((34.0, 6, 30.0), (47.0, 6, 0.0), (59.5, 8, 0.0), (85.0, 8, 22.5)):
        e += _ring_holes(r, n, off, TAP_M3)
    e += _ring_holes(14.0, 4, 45.0, TAP_M3)
    return e


def boss():
    """304 stainless, 3 mm: 40 mm disc screwed under the plate's centre (four M3 x 4 countersunk screws); it sits in the
    stand's socket."""
    return [("C", 0.0, 0.0, 20.0)] + _ring_holes(14.0, 4, 45.0, CLEAR_M3)


def clip_tab():
    """301 stainless, half hard, 1.0 mm: 15 x 4 mm tab, 13 mm from the pivot (its 3.2 mm hole) to the tip. Cut 9 (one spare)."""
    w = 2.0
    return [("A", 0.0, 0.0, w, 90.0, 270.0), ("L", 0.0, w, 13.0, w), ("L", 0.0, -w, 13.0, -w), ("L", 13.0, -w, 13.0, w),
            ("C", 0.0, 0.0, CLEAR_M3 / 2)]


def ring_layers():
    """6061 aluminium. Layer A, 2.5 mm: 220 mm outside, 180 mm inside. Layer B, 1.5 mm: 220 mm outside, 203 mm inside
    (the recess that takes the 200 mm plate). Bond B under A (epoxy) or join with six M3 x 4 countersunk screws."""
    a = [("C", 0.0, 0.0, 110.0), ("C", 0.0, 0.0, 90.0)] + _ring_holes(105.75, 6, 0.0, TAP_M3)
    b = [("C", 0.0, 0.0, 110.0), ("C", 0.0, 0.0, 101.5)] + _ring_holes(105.75, 6, 0.0, CLEAR_M3)
    return a, b


def stand_top():
    """Mild steel, 6 mm: 120 x 120 mm top plate with a 41 mm socket hole for the boss and four M5 clearance holes for a
    50 x 50 mm square-tube post (130 mm to the top of this plate)."""
    return _rect(-60, -60, 60, 60) + [("C", 0.0, 0.0, 20.5)] + _ring_holes(35.0, 4, 45.0, 5.5)


def door():
    """6061 aluminium, 6 mm: 760 x 230 mm door (slides up 250 mm in two U-channel rails). Holes: two M6 for the 140 mm
    pull handle (centres 70 mm either side of y = 0, 105 mm above the door's bottom edge), a 16 mm hole for the T-handle
    quarter-turn latch at y = -200 mm, 55 mm above the bottom edge."""
    return (_rect(-380, 0, 380, 230) + [("C", -70.0, 105.0, 3.25), ("C", 70.0, 105.0, 3.25), ("C", -200.0, 55.0, 8.0)])


def main(out):
    os.makedirs(out, exist_ok=True)
    a, b = ring_layers()
    for name, ents in (("sample_plate_1p5mm_ss304", sample_plate()), ("plate_boss_3mm_ss304", boss()),
                       ("clip_tab_1mm_ss301", clip_tab()), ("ring_holder_A_2p5mm_al6061", a),
                       ("ring_holder_B_1p5mm_al6061", b), ("stand_top_6mm_steel", stand_top()),
                       ("door_6mm_al6061", door())):
        with open(os.path.join(out, name + ".dxf"), "w") as f:
            f.write(_dxf(ents))
        print("wrote", name + ".dxf")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "parts")
