"""Printed text for a yam_arena task file: block capitals as thin black boxes on a flat white board.

Each letter is a 5 x 7 grid of square cells. The text reads from the top camera, which looks forward over the arms:
lines run from the far edge of the board (+x) toward the arms, and letters from the robot's left (+y) to its right.
For the real build, :func:`svg` writes the same board at full size for printing.
"""

from __future__ import annotations

from dataclasses import dataclass

GLYPH_W, GLYPH_H = 5, 7  # cells
ADVANCE, LINE_GAP, MARGIN = 6, 2, 2  # cells: letter pitch, gap between lines, board margin
INK_THICK = 0.001  # m
WHITE = (0.95, 0.95, 0.95)
BLACK = (0.04, 0.04, 0.04)

FONT = {
    "A": (".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"),
    "B": ("####.", "#...#", "#...#", "####.", "#...#", "#...#", "####."),
    "C": (".###.", "#...#", "#....", "#....", "#....", "#...#", ".###."),
    "D": ("####.", "#...#", "#...#", "#...#", "#...#", "#...#", "####."),
    "E": ("#####", "#....", "#....", "####.", "#....", "#....", "#####"),
    "G": (".###.", "#...#", "#....", "#.###", "#...#", "#...#", ".####"),
    "I": ("#####", "..#..", "..#..", "..#..", "..#..", "..#..", "#####"),
    "K": ("#...#", "#..#.", "#.#..", "##...", "#.#..", "#..#.", "#...#"),
    "L": ("#....", "#....", "#....", "#....", "#....", "#....", "#####"),
    "N": ("#...#", "##..#", "#.#.#", "#..##", "#...#", "#...#", "#...#"),
    "O": (".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."),
    "P": ("####.", "#...#", "#...#", "####.", "#....", "#....", "#...."),
    "R": ("####.", "#...#", "#...#", "####.", "#.#..", "#..#.", "#...#"),
    "S": (".####", "#....", "#....", ".###.", "....#", "....#", "####."),
    "T": ("#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."),
    "U": ("#...#", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."),
    "Y": ("#...#", "#...#", ".#.#.", "..#..", "..#..", "..#..", "..#.."),
    "F": ("#####", "#....", "#....", "####.", "#....", "#....", "#...."),
    "H": ("#...#", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"),
    "J": ("..###", "...#.", "...#.", "...#.", "...#.", "#..#.", ".##.."),
    "M": ("#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"),
    "Q": (".###.", "#...#", "#...#", "#...#", "#.#.#", "#..#.", ".##.#"),
    "V": ("#...#", "#...#", "#...#", "#...#", "#...#", ".#.#.", "..#.."),
    "W": ("#...#", "#...#", "#...#", "#.#.#", "#.#.#", "#.#.#", ".#.#."),
    "X": ("#...#", "#...#", ".#.#.", "..#..", ".#.#.", "#...#", "#...#"),
    "Z": ("#####", "....#", "...#.", "..#..", ".#...", "#....", "#####"),
    "0": (".###.", "#...#", "#..##", "#.#.#", "##..#", "#...#", ".###."),
    "1": ("..#..", ".##..", "..#..", "..#..", "..#..", "..#..", ".###."),
    "2": (".###.", "#...#", "....#", "...#.", "..#..", ".#...", "#####"),
    "3": ("####.", "....#", "....#", ".###.", "....#", "....#", "####."),
    "4": ("...#.", "..##.", ".#.#.", "#..#.", "#####", "...#.", "...#."),
    "5": ("#####", "#....", "####.", "....#", "....#", "#...#", ".###."),
    "6": (".###.", "#....", "#....", "####.", "#...#", "#...#", ".###."),
    "7": ("#####", "....#", "...#.", "..#..", ".#...", ".#...", ".#..."),
    "8": (".###.", "#...#", "#...#", ".###.", "#...#", "#...#", ".###."),
    "9": (".###.", "#...#", "#...#", ".####", "....#", "....#", ".###."),
    "-": (".....", ".....", ".....", "#####", ".....", ".....", "....."),
    "+": (".....", "..#..", "..#..", "#####", "..#..", "..#..", "....."),
    ".": (".....", ".....", ".....", ".....", ".....", ".##..", ".##.."),
    ":": (".....", ".##..", ".##..", ".....", ".##..", ".##..", "....."),
    " ": (".....",) * 7,
}


@dataclass(frozen=True)
class Board:
    """A printed board: ``lines`` of text in cells of ``cell`` m, its near edge (toward the arms) at ``near_x``,
    centred on ``center_y``. Sizes and positions in metres, world frame."""

    lines: tuple[str, ...]
    cell: float
    near_x: float
    center_y: float

    @property
    def cols(self) -> int:
        return max(len(t) for t in self.lines) * ADVANCE - (ADVANCE - GLYPH_W) + 2 * MARGIN

    @property
    def rows(self) -> int:
        return len(self.lines) * GLYPH_H + (len(self.lines) - 1) * LINE_GAP + 2 * MARGIN

    @property
    def size(self) -> tuple[float, float]:
        """(depth along x, width along y), m."""
        return self.rows * self.cell, self.cols * self.cell

    @property
    def far_x(self) -> float:
        return self.near_x + self.size[0]

    def cell_rects(self) -> list[tuple[int, int, int, int]]:
        """The ink as rectangles of cells (row, col, rows, cols), row 0 at the far edge, col 0 at the left edge.
        Each line is centred; horizontal runs of ink are merged, then stacked runs of the same span."""
        runs = []
        for i, text in enumerate(self.lines):
            unknown = sorted(set(text) - set(FONT))
            if unknown:
                raise ValueError(f"no glyph for {unknown} in {text!r}")
            width = len(text) * ADVANCE - (ADVANCE - GLYPH_W)
            left = MARGIN + (self.cols - 2 * MARGIN - width) // 2
            top = MARGIN + i * (GLYPH_H + LINE_GAP)
            for k, ch in enumerate(text):
                for r, row in enumerate(FONT[ch]):
                    c = 0
                    while c < GLYPH_W:
                        if row[c] == "#":
                            start = c
                            while c < GLYPH_W and row[c] == "#":
                                c += 1
                            runs.append((top + r, left + k * ADVANCE + start, c - start))
                        else:
                            c += 1
        rects: list[list[int]] = []
        for row, col, n in sorted(runs):
            for rect in rects:
                if rect[1] == col and rect[3] == n and rect[0] + rect[2] == row:
                    rect[2] += 1
                    break
            else:
                rects.append([row, col, 1, n])
        return [tuple(r) for r in rects]

    def ink_rects(self) -> list[tuple[float, float, float, float]]:
        """The ink as (centre x, centre y, size x, size y), m, world frame."""
        c, left_y = self.cell, self.center_y + self.size[1] / 2
        return [
            (self.far_x - (row + rows / 2) * c, left_y - (col + cols / 2) * c, rows * c, cols * c)
            for row, col, rows, cols in self.cell_rects()
        ]

    def svg(self) -> str:
        """The board at full size, for printing (millimetres)."""
        mm = 1000 * self.cell
        w, h = self.cols * mm, self.rows * mm
        out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.1f}mm" height="{h:.1f}mm" '
               f'viewBox="0 0 {w:.1f} {h:.1f}">', f'<rect width="{w:.1f}" height="{h:.1f}" fill="white"/>']
        for row, col, rows, cols in self.cell_rects():
            out.append(f'<rect x="{col * mm:.1f}" y="{row * mm:.1f}" width="{cols * mm:.1f}" '
                       f'height="{rows * mm:.1f}" fill="black"/>')
        return "\n".join(out + ["</svg>"]) + "\n"
