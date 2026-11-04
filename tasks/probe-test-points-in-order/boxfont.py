"""Block letters built from fixed thin boxes (the task-file API has no textures): a 5x7 bitmap font whose
pixel runs are merged into rectangles. Text lies flat on the table, read from the robot side (top camera):
letter tops point to +x (away from the robot), text runs from +y to -y."""
from __future__ import annotations

GLYPHS = {
    "A": ["01110", "10001", "10001", "11111", "10001", "10001", "10001"],
    "B": ["11110", "10001", "10001", "11110", "10001", "10001", "11110"],
    "C": ["01111", "10000", "10000", "10000", "10000", "10000", "01111"],
    "D": ["11110", "10001", "10001", "10001", "10001", "10001", "11110"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "F": ["11111", "10000", "10000", "11110", "10000", "10000", "10000"],
    "H": ["10001", "10001", "10001", "11111", "10001", "10001", "10001"],
    "I": ["11111", "00100", "00100", "00100", "00100", "00100", "11111"],
    "L": ["10000", "10000", "10000", "10000", "10000", "10000", "11111"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "O": ["01110", "10001", "10001", "10001", "10001", "10001", "01110"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "R": ["11110", "10001", "10001", "11110", "10100", "10010", "10001"],
    "S": ["01111", "10000", "10000", "01110", "00001", "00001", "11110"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "W": ["10001", "10001", "10001", "10101", "10101", "11011", "10001"],
    "X": ["10001", "10001", "01010", "00100", "01010", "10001", "10001"],
    "Y": ["10001", "10001", "01010", "00100", "00100", "00100", "00100"],
    "2": ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    "1": ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    "3": ["11111", "00010", "00100", "00010", "00001", "10001", "01110"],
    "4": ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    "5": ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    "6": ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    " ": ["00000"] * 7,
}


def _rects(rows: list[str]) -> list[tuple[int, int, int, int]]:
    """(col0, row0, ncols, nrows) rectangles covering the 1-pixels: horizontal runs, merged down identical runs."""
    runs = []
    for r, row in enumerate(rows):
        c = 0
        while c < len(row):
            if row[c] == "1":
                c0 = c
                while c < len(row) and row[c] == "1":
                    c += 1
                runs.append([c0, r, c - c0, 1])
            else:
                c += 1
    merged: list[list[int]] = []
    for run in runs:
        for m in merged:
            if m[0] == run[0] and m[2] == run[2] and m[1] + m[3] == run[1]:
                m[3] += 1
                break
        else:
            merged.append(run)
    return [tuple(m) for m in merged]


def text_width(text: str, px: float, gap: int = 1) -> float:
    n = len(text)
    return px * (5 * n + gap * (n - 1))


def text_boxes(Box, prefix: str, text: str, center_xy, z: float, px: float, thick: float = 0.001,
               color=(0.02, 0.02, 0.02), gap: int = 1) -> list:
    """Boxes spelling ``text`` centred at ``center_xy``; their bottoms at ``z``."""
    cx, cy = center_xy
    w = text_width(text, px, gap)
    h = 7 * px
    out = []
    for i, ch in enumerate(text):
        y_left = cy + w / 2 - i * (5 + gap) * px  # +y edge of this letter cell
        for k, (c0, r0, nc, nr) in enumerate(_rects(GLYPHS[ch])):
            x = cx + h / 2 - (r0 + nr / 2) * px
            y = y_left - (c0 + nc / 2) * px
            out.append(Box(f"{prefix}_{i}_{k}", pos=(x, y, z + thick / 2), size=(nr * px, nc * px, thick), color=color))
    return out
