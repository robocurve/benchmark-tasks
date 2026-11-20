"""Draw the stylus line on the screen as a dark ribbon, so that every rendered frame shows what has been drawn.

Display only (a stated rule standing in for the aluminium powder being scraped off the glass). The ribbon is
3 mm wide, wider than the real line (under 1 mm), so that the 320 x 176 overhead view, about 5 mm of table per
pixel, shows it; the scorer uses the stylus path itself, not this ribbon. Same technique as the cutting-wire
and drawer skins of earlier packages that rendered in Isaac: one USD mesh whose points are rewritten.
"""

import numpy as np

from . import layout

RISE = .0003                      # above the screen
QUADS = 900
SPACING = .0007                   # ribbon points are at least this far apart


def ribbon(points, width=layout.LINE_WIDTH, quads=QUADS):
    """Corner points (quads x 4 x 2) of the ribbon along a path of (x, y) points. A single point is drawn as a
    small square dab. Unused quads collapse to the last point."""
    out = np.zeros((quads, 4, 2))
    line = np.asarray(points, dtype=float).reshape(-1, 2)
    keep = [line[0]]
    for point in line[1:]:
        if np.linalg.norm(point - keep[-1]) >= SPACING:
            keep.append(point)
    if len(keep) == 1:
        keep = [keep[0] - (width / 2, 0.), keep[0] + (width / 2, 0.)]
    used = 0
    for a, b in zip(keep, keep[1:]):
        if used == quads:
            break
        along = (b - a) / np.linalg.norm(b - a)
        # each quad overhangs half a width at both ends, so that corners are filled
        a, b = a - along * width / 2, b + along * width / 2
        side = np.array((-along[1], along[0])) * width / 2
        out[used] = (a - side, b - side, b + side, a + side)
        used += 1
    out[used:] = out[used - 1, 2] if used else line[0]
    return out


class LineSkin:
    def __init__(self, stage, path, color=(.035, .035, .04)):
        from pxr import Gf, UsdGeom, Vt

        mesh = UsdGeom.Mesh.Define(stage, path)
        self.points = mesh.CreatePointsAttr()
        self.extent = mesh.CreateExtentAttr()
        self.vectors = lambda data: Vt.Vec3fArray([Gf.Vec3f(*map(float, point)) for point in data])
        mesh.CreateFaceVertexCountsAttr([4] * QUADS)
        mesh.CreateFaceVertexIndicesAttr(list(range(4 * QUADS)))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorPrimvar("constant").Set(self.vectors([color]))
        # Points from the start (all quads collapsed far below the table), so that the renderer never sees a mesh
        # with faces but no points (run 1 logged "Skipping invalid mesh DrawnLine" once before the first update).
        hidden = [(0., 0., -5.)] * (4 * QUADS)
        self.points.Set(self.vectors(hidden))
        self.extent.Set(self.vectors((hidden[0], hidden[0])))
        self.shown = None

    def update(self, path_toy, toy_position, toy_rotation, top):
        """``path_toy``: drawn points in the toy frame (x, y), metres; the toy's pose in the tabletop frame."""
        key = (len(path_toy), tuple(np.round(toy_position, 4)), tuple(np.round(np.asarray(toy_rotation)[:2, 0], 4)))
        if key == self.shown:
            return
        self.shown = key
        flat = ribbon(path_toy).reshape(-1, 2)
        local = np.column_stack((flat, np.full(len(flat), layout.SCREEN_Z + RISE)))
        verts = np.asarray(toy_position) + (0., 0., top) + local @ np.asarray(toy_rotation).T
        self.points.Set(self.vectors(verts))
        self.extent.Set(self.vectors((verts.min(axis=0), verts.max(axis=0))))
