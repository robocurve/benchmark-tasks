"""Read the frame's in-plane geometry from Salsa's 1:1 Rangefinder frame-bag drawing. Created 11_16_26 (America/New_York).

    curl -L -o rangefinder.pdf https://cdn.shopify.com/s/files/1/0912/4973/8003/files/470236_SAL_Frame_Bag_Guide_Rangefinder_VF_OL.pdf
    uv pip install pymupdf numpy        # in any scratch environment
    python dev/frame_from_drawing.py rangefinder.pdf

The drawing is vector art at true scale (68 x 44 in sheet, "scale 1:1"), so 1 pt = 25.4/72 mm. It prints: the scale bar's
length (should be 127 mm), the bottom bracket's centre (a least-squares circle through the shell's outline), and, in mm
from that centre (X forward, Z up), the edges and bosses bike_cage/geom.py uses. The page coordinates below are the
ends of the drawing's own line segments (XS frame, left third of page 1); the bosses are the gaps in the down tube's
inner edge where the drawing marks them, 64 mm apart as dimensioned."""

import sys

import numpy as np
import pymupdf

K = 25.4 / 72
LINES = {  # page points (pt): segment ends of the outline (stroke width 2.127) as found on page 1
    "down tube inner edge": [(774.3, 1945.3), (1394.7, 830.8)],
    "down tube outer edge": [(895.5, 2013.1), (984.3, 1854.4), (1518.3, 899.8)],
    "top tube lower edge": [(437.7, 1381.6), (621.3, 1259.8), (806.1, 1140.1), (992.1, 1023.1), (1139.7, 932.8), (1287.9, 844.3), (1437.3, 757.0)],
    "top tube upper edge": [(394.5, 1318.6), (519.3, 1231.6), (643.5, 1145.8), (767.7, 1060.9), (893.1, 976.0), (1025.4, 887.2), (1158.0, 799.0), (1289.7, 712.6)],
    "seat tube front edge": [(603.6, 2082.4), (562.5, 1910.5), (542.1, 1829.8), (515.4, 1739.8), (508.8, 1716.7), (414.6, 1397.2)],
    "bosses (gaps in the down tube's inner edge)": [(872.7, 1768.3), (961.0, 1610.0), (1049.2, 1451.4)],
    "head tube edge": [(1873.8, 662.8), (1855.2, 616.9)],
}


def main(path):
    pg = pymupdf.open(path)[0]
    pts, bar = [], None
    for d in pg.get_drawings():
        for it in d["items"]:
            if it[0] != "l":
                continue
            p, q = it[1], it[2]
            if abs(p.y - q.y) < 0.01 and 2100 < p.x < 2600 and p.y < 300 and abs(p.x - q.x) > 300:
                bar = max(bar or 0.0, abs(p.x - q.x) * K)
            if d.get("width") and abs(d["width"] - 2.127) < 0.01 and all(450 < v.x < 800 and 2050 < v.y < 2350 for v in (p, q)):
                pts.append((p.x, p.y))
    print(f"scale bar {bar:.2f} mm" if bar else "scale bar not found")
    P = np.unique(np.round(np.array(pts), 2), axis=0)
    centres = []
    for band in ((15.5, 17.5), (18.0, 20.0)):  # the shell is drawn as two circles
        c = np.array((575.0, 2163.0))
        for _ in range(6):
            d = np.linalg.norm(P - c, axis=1) * K
            Q = P[(d > band[0]) & (d < band[1])]
            A = np.c_[2 * Q, np.ones(len(Q))]
            s = np.linalg.lstsq(A, (Q ** 2).sum(1), rcond=None)[0]
            c = s[:2]
        centres.append(c)
        print(f"shell circle: centre {np.round(c, 1).tolist()} pt, radius {np.sqrt(s[2] + c @ c) * K:.1f} mm")
    print(f"mean centre {np.round(np.mean(centres, axis=0), 1).tolist()} pt (geom.py uses (575.0, 2163.0); the two differ by under 1 mm)")
    bb = np.array((575.0, 2163.0))
    for name, seg in LINES.items():
        mm = [(round(float((x - bb[0]) * K), 1), round(float((bb[1] - y) * K), 1)) for x, y in seg]
        print(f"{name}: {mm}")
    a, b = np.array(LINES["head tube edge"])
    ang = np.degrees(np.arctan2(a[1] - b[1], b[0] - a[0])) % 180
    print(f"head tube axis {180 - ang:.1f} deg from the page's horizontal (Salsa: 68.0 head angle, 27.5+ build)")


if __name__ == "__main__":
    main(sys.argv[1])
