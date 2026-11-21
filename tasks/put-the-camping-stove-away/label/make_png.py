"""Draws isopro.png, the printed label wrapped round the canister body: black capitals on white.

Line 1: a flammable-gas mark (a red-bordered diamond with a black flame, in the style of the GHS02 pictogram) and
ISOPRO in 18 mm capitals. Line 2: FLAMMABLE in 14 mm capitals. The label is 120 mm of arc by 44 mm high.
Pure Python (zlib only), no font files: letters come from a 5 x 7 block grid. Run it from anywhere:

    python label/make_png.py
"""

import math
import pathlib
import struct
import zlib

W_MM, H_MM = 120.0, 44.0
PX_PER_MM = 6
SS = 2
WHITE = (240, 240, 236)
BLACK = (20, 20, 20)
RED = (205, 25, 25)

GLYPHS = {
    "I": ["XXXXX", "..X..", "..X..", "..X..", "..X..", "..X..", "XXXXX"],
    "S": [".XXXX", "X....", "X....", ".XXX.", "....X", "....X", "XXXX."],
    "O": [".XXX.", "X...X", "X...X", "X...X", "X...X", "X...X", ".XXX."],
    "P": ["XXXX.", "X...X", "X...X", "XXXX.", "X....", "X....", "X...."],
    "R": ["XXXX.", "X...X", "X...X", "XXXX.", "X.X..", "X..X.", "X...X"],
    "F": ["XXXXX", "X....", "X....", "XXXX.", "X....", "X....", "X...."],
    "L": ["X....", "X....", "X....", "X....", "X....", "X....", "XXXXX"],
    "A": [".XXX.", "X...X", "X...X", "XXXXX", "X...X", "X...X", "X...X"],
    "M": ["X...X", "XX.XX", "X.X.X", "X.X.X", "X...X", "X...X", "X...X"],
    "B": ["XXXX.", "X...X", "X...X", "XXXX.", "X...X", "X...X", "XXXX."],
    "E": ["XXXXX", "X....", "X....", "XXXX.", "X....", "X....", "XXXXX"],
}

DIAMOND = 22.0  # mark's corner-to-corner size (mm)
# (text, cap height, letter width, gap, baseline y, start x); x is from the label's left edge as read
LINE1 = ("ISOPRO", 18.0, 11.0, 3.0, 22.5)
LINE2 = ("FLAMMABLE", 14.0, 9.0, 2.5, 4.0)


def _line_width(text, w, gap):
    return len(text) * w + (len(text) - 1) * gap


def _text(x, y, text, cap, w, gap, base, x0):
    if not base <= y <= base + cap or x < x0:
        return False
    i = int((x - x0) // (w + gap))
    if i >= len(text) or x - x0 - i * (w + gap) > w:
        return False
    col = min(4, int((x - x0 - i * (w + gap)) / w * 5))
    row = min(6, int((base + cap - y) / cap * 7))
    return GLYPHS[text[i]][row][col] == "X"


def _mark(x, y, cx, cy):
    """RED, BLACK or None for the flammable-gas diamond centred at (cx, cy)."""
    u, v = abs(x - cx), abs(y - cy)
    half = DIAMOND / 2
    if u + v > half:
        return None
    if u + v > half - 2.2:
        return RED
    # flame: a tall tongue leaning right and a shorter one on its left, over a short bar
    if cy - 6.2 <= y <= cy - 5.0 and u <= 4.6:
        return BLACK
    for bx, w, y0, y1, lean in ((0.6, 3.4, cy - 4.6, cy + 6.4, 1.6), (-2.4, 2.0, cy - 4.6, cy + 2.2, -1.0)):
        if y0 <= y <= y1:
            t = (y - y0) / (y1 - y0)
            half_w = w * math.sqrt(max(0.0, 1 - t)) * min(1.0, 0.55 + 1.8 * t)
            if abs(x - cx - bx - lean * t * t) <= half_w:
                return BLACK
    return WHITE


def colour(x, y):
    text1, cap1, w1, g1, b1 = LINE1
    width1 = DIAMOND + 4.0 + _line_width(text1, w1, g1)
    left1 = (W_MM - width1) / 2
    m = _mark(x, y, left1 + DIAMOND / 2, b1 + cap1 / 2)
    if m is not None:
        return m
    if _text(x, y, text1, cap1, w1, g1, b1, left1 + DIAMOND + 4.0):
        return BLACK
    text2, cap2, w2, g2, b2 = LINE2
    if _text(x, y, text2, cap2, w2, g2, b2, (W_MM - _line_width(text2, w2, g2)) / 2):
        return BLACK
    return WHITE


def render(path):
    wpx, hpx = int(W_MM * PX_PER_MM), int(H_MM * PX_PER_MM)
    rows = []
    for row in range(hpx):  # PNG rows run top to bottom
        line = bytearray([0])
        for col in range(wpx):
            acc = [0, 0, 0]
            for i in range(SS):
                for j in range(SS):
                    c = colour((col + (i + 0.5) / SS) / PX_PER_MM, H_MM - (row + (j + 0.5) / SS) / PX_PER_MM)
                    acc = [a + b for a, b in zip(acc, c)]
            line += bytes(round(a / (SS * SS)) for a in acc)
        rows.append(bytes(line))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", wpx, hpx, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(b"".join(rows), 9)) + chunk(b"IEND", b"")
    pathlib.Path(path).write_bytes(png)


if __name__ == "__main__":
    render(pathlib.Path(__file__).resolve().parent / "isopro.png")
