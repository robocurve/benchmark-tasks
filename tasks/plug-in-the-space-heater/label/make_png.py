"""Draws the heater's two printed panels as PNG files, in pure Python (zlib only, no font files).

    python label/make_png.py

top.png   (140 x 140 mm, on the heater's top face): LASKO / 754200 / HEATER / 1500 W on a black panel.
front.png (130 x 62 mm, the rating plate on the heater's front): black 754200 and 1500 W on white, with three
          orange heat waves at the left.

Letters come from the 5 x 7 dot patterns below, drawn as solid squares.
"""

import math
import pathlib
import struct
import zlib

FONT = {
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("01110", "10001", "00001", "00010", "00100", "01000", "11111"),
    "4": ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    "5": ("11111", "10000", "11110", "00001", "00001", "10001", "01110"),
    "7": ("11111", "00001", "00010", "00100", "01000", "01000", "01000"),
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "K": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "W": ("10001", "10001", "10001", "10101", "10101", "10101", "01010"),
    " ": ("00000",) * 7,
}
PX_PER_MM = 5
WHITE, BLACK, ORANGE = (245, 245, 240), (20, 20, 22), (235, 95, 20)


def text_ink(x, y, text, x0, y0, cell):
    """True where ``text`` (lower-left corner at x0, y0 mm; dots ``cell`` mm square) has ink at (x, y) mm."""
    col, row = math.floor((x - x0) / cell), math.floor((y - y0) / cell)
    if not (0 <= row < 7 and 0 <= col < 6 * len(text) - 1):
        return False
    k, c = divmod(col, 6)
    return c < 5 and FONT[text[k]][6 - row][c] == "1"


def text_width(text, cell):
    return (6 * len(text) - 1) * cell


def write_png(path, w_mm, h_mm, colour_at):
    """``colour_at(x, y)`` gives the RGB colour at (x, y) mm from the panel's lower-left corner."""
    wpx, hpx = round(w_mm * PX_PER_MM), round(h_mm * PX_PER_MM)
    rows = []
    for row in range(hpx):  # PNG rows run top to bottom
        line = bytearray([0])
        y = h_mm - (row + 0.5) / PX_PER_MM
        for col in range(wpx):
            line += bytes(colour_at((col + 0.5) / PX_PER_MM, y))
        rows.append(bytes(line))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", wpx, hpx, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(b"".join(rows), 9)) + chunk(b"IEND", b"")
    pathlib.Path(path).write_bytes(png)


TOP_MM, TOP_CELL = (140.0, 140.0), 3.8
TOP_LINES = ("LASKO", "754200", "HEATER", "1500 W")
FRONT_MM, FRONT_CELL = (130.0, 62.0), 2.6
FRONT_LINES = ("754200", "1500 W")


def top_colour(x, y):
    h = 7 * TOP_CELL
    gap = (TOP_MM[1] - len(TOP_LINES) * h) / (len(TOP_LINES) + 1)
    for k, text in enumerate(TOP_LINES):  # first line at the top
        y0 = TOP_MM[1] - (k + 1) * (gap + h)
        if text_ink(x, y, text, (TOP_MM[0] - text_width(text, TOP_CELL)) / 2, y0, TOP_CELL):
            return WHITE
    return BLACK


def front_colour(x, y):
    if min(x, y, FRONT_MM[0] - x, FRONT_MM[1] - y) < 1.5:
        return BLACK  # border
    for k in range(3):  # three heat waves, 5 mm apart, at the left
        cx = 12.0 + 7.0 * k + 2.2 * math.sin(2 * math.pi * (y - 8.0) / 23.0)
        if 8.0 <= y <= FRONT_MM[1] - 8.0 and abs(x - cx) <= 1.3:
            return ORANGE
    h = 7 * FRONT_CELL
    gap = (FRONT_MM[1] - len(FRONT_LINES) * h) / (len(FRONT_LINES) + 1)
    for k, text in enumerate(FRONT_LINES):
        y0 = FRONT_MM[1] - (k + 1) * (gap + h)
        if text_ink(x, y, text, 34.0, y0, FRONT_CELL):
            return BLACK
    return WHITE


if __name__ == "__main__":
    here = pathlib.Path(__file__).resolve().parent
    write_png(here / "top.png", *TOP_MM, top_colour)
    write_png(here / "front.png", *FRONT_MM, front_colour)
