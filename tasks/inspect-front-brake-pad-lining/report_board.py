"""A bench marker records the measured lining bracket on the inspection card."""

from math import hypot
import numpy as np
from geometry import box

BOARD_ORIGIN = np.array([.313, .204])
BOARD_SIZE = (.161, .202, .006)
TOKEN_HEIGHT = .028
TOKEN_RADIUS = .017
SLOTS = {
    "START": (.352, .262),
    "<8 MM": (.352, .146),
    "8-10 MM": (.275, .146),
    "10+ MM": (.275, .262),
}
RESULT = "8-10 MM"
INK = (.09, .12, .17)
PAPER = (.89, .89, .79)
LINE = (.28, .31, .34)

# Five-row glyphs used for the actual printed card in the scene.
GLYPHS = {
    "0": ("111", "101", "101", "101", "111"),
    "1": ("010", "110", "010", "010", "111"),
    "8": ("111", "101", "111", "101", "111"),
    "-": ("000", "000", "111", "000", "000"),
    "+": ("000", "010", "111", "010", "000"),
    "<": ("001", "010", "100", "010", "001"),
    "M": ("10001", "11011", "10101", "10001", "10001"),
    "S": ("111", "100", "111", "001", "111"),
    "T": ("111", "010", "010", "010", "010"),
    "A": ("010", "101", "111", "101", "101"),
    "R": ("110", "101", "110", "101", "101"),
    " ": ("0",) * 5,
}


def board_marks():
    for label, (x, y) in SLOTS.items():
        x, y = x - BOARD_ORIGIN[0], y - BOARD_ORIGIN[1]
        resolution = .0039
        cells = sum(len(GLYPHS[char][0]) + 1 for char in label) - 1
        column = -cells / 2 + .5
        for char in label:
            glyph = GLYPHS[char]
            for row, line in enumerate(glyph):
                for offset, pixel in enumerate(line):
                    if pixel == "1":
                        yield box((x + .029 + (2-row)*resolution, y - (column+offset)*resolution,
                                   BOARD_SIZE[2]/2+.00012),
                                  (resolution, resolution, .00012), INK)
            column += len(glyph[0]) + 1
        for dx, dy, sx, sy in ((-.022,0,.002,.045),(.022,0,.002,.045),
                               (0,-.022,.045,.002),(0,.022,.045,.002)):
            yield box((x+dx, y+dy, BOARD_SIZE[2]/2+.00012), (sx,sy,.00012), LINE)


def record_is_set(position, velocity, angular_velocity, table_height):
    x, y = SLOTS[RESULT]
    z = table_height + BOARD_SIZE[2] + TOKEN_HEIGHT/2
    return (
        hypot(position[0]-x, position[1]-y) <= .008
        and abs(position[2]-z) <= .002
        and np.linalg.norm(velocity) < .004
        and np.linalg.norm(angular_velocity) < .12
    )
