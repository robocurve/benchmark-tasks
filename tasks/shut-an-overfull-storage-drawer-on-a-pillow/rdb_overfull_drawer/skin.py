"""Draw the pillow as one continuous fabric surface over its 16 sprung pads. Display only.

The physics is unchanged: the pillow is still a base slab with 16 rigid pads, and only those collide. This
module hides the pads' and the base's boxes from the renderer and draws, in their place, one closed cushion
shape with rounded edges whose upper side follows the measured height of the pads at every control step, so
a pressed part shows as a hollow and the part held under the frame's top shows as flattened. The surface has
no collider and is never read by the scorer or given to a policy except as pixels.

``PillowSurface`` is NumPy only, so the shape can be tested without the simulator.
"""

import numpy as np

from . import layout

NX, NY = 67, 49                         # vertices along and across the pillow, per side
SEAM_ABOVE_FLOOR = .050                 # height of the pillow's side seam above the drawer floor
UNDERSIDE_ABOVE_FLOOR = .001
BELOW_PAD = .001                        # the drawn top lies this far under the pads' tops, never above them
FLAT = .30                              # share of the way to the next pad over which a pad's height is kept
COLOR = (.20, .29, .42)                 # heathered blue-grey polyester cover
SEAM_COLOR = (.12, .18, .28)


def _blend(coords, centres):
    """Weights (len(coords), len(centres)) that hold each centre's value near it and ease to the next one."""
    centres = np.asarray(centres, dtype=float)
    weights = np.zeros((len(coords), len(centres)))
    index = np.clip(np.searchsorted(centres, coords) - 1, 0, len(centres) - 2)
    u = np.clip((coords - centres[index]) / (centres[index + 1] - centres[index]), 0., 1.)
    s = np.clip((u - FLAT) / (1. - 2. * FLAT), 0., 1.)
    s = s * s * (3. - 2. * s)
    rows = np.arange(len(coords))
    weights[rows, index] = 1. - s
    weights[rows, index + 1] += s
    return weights


class PillowSurface:
    """Vertices of the drawn pillow, in the tabletop frame, from the pads' heights."""

    def __init__(self):
        half_x, half_y = layout.BASE_SIZE[0] / 2, layout.BASE_SIZE[1] / 2
        # denser toward the edges, where the surface curves down to the seam
        sx = np.sin(np.linspace(-np.pi / 2, np.pi / 2, NX))
        sy = np.sin(np.linspace(-np.pi / 2, np.pi / 2, NY))
        self.x, self.y = np.meshgrid(half_x * sx, half_y * sy, indexing="ij")
        # a rectangular cushion: full height over the middle, curving down to the seam along all four edges
        r = 1. - (1. - np.abs(self.x / half_x) ** 8) * (1. - np.abs(self.y / half_y) ** 8)
        self.rise = np.clip(1. - np.clip(r, 0., 1.), 0., 1.) ** .40                     # 1 in the middle, 0 at the seam
        base_centre = layout.FRONT_T + layout.DRAWER_INNER[0] / 2
        row_x = [layout.row_centre(row) - base_centre for row in range(layout.ROWS)]
        wx = _blend(self.x[:, 0], row_x)                                            # (NX, ROWS)
        wy = _blend(self.y[0], sorted(layout.PAD_Y))                                # (NY, 2), columns right then left
        order = np.argsort(layout.PAD_Y)
        self.weights = np.einsum("ir,jc->ijrc", wx, wy)[..., np.argsort(order)]     # (NX, NY, ROWS, COLS)
        n = NX * NY
        quads, colors = [], []
        for side in (0, 1):                 # upper sheet, then lower sheet (wound the other way)
            for i in range(NX - 1):
                for j in range(NY - 1):
                    a, b, c, d = (side * n + i * NY + j, side * n + (i + 1) * NY + j,
                                  side * n + (i + 1) * NY + j + 1, side * n + i * NY + j + 1)
                    quads.append((a, b, c, d) if side == 0 else (a, d, c, b))
                    edge = i in (0, NX - 2) or j in (0, NY - 2)
                    colors.append(SEAM_COLOR if edge else COLOR)
        self.faces = np.asarray(quads)
        self.colors = np.asarray(colors)

    def vertices(self, base_position, base_rotation, pad_tops):
        """``base_position``: middle of the pillow's underside. ``base_rotation``: its 3x3 rotation.
        ``pad_tops``: (ROWS, COLS) height of each pad's top above the table. Returns (2 * NX * NY, 3)."""
        base_position = np.asarray(base_position, dtype=float)
        rotation = np.asarray(base_rotation, dtype=float)
        tops = np.asarray(pad_tops, dtype=float).reshape(layout.ROWS, layout.COLS)
        floor = base_position[2] - .0005
        crown = np.einsum("ijrc,rc->ij", self.weights, tops) - BELOW_PAD
        seam = floor + SEAM_ABOVE_FLOOR
        crown = np.maximum(crown, seam + .004)
        upper = seam + (crown - seam) * self.rise
        lower = seam - (seam - (floor + UNDERSIDE_ABOVE_FLOOR)) * self.rise
        xy = np.stack((self.x, self.y), axis=-1) @ rotation[:2, :2].T + base_position[:2]
        sheets = [np.concatenate((xy, z[..., None]), axis=-1).reshape(-1, 3) for z in (upper, lower)]
        return np.concatenate(sheets)


class PillowSkin:
    """The surface as a mesh on the simulator's stage. Created once per scene, updated every control step."""

    def __init__(self, stage, fixture_path, table_top):
        from pxr import Gf, UsdGeom, Vt

        for name in layout.ACTORS[1:]:                       # the pillow's base and pads: still simulated, not drawn
            body = stage.GetPrimAtPath(f"{fixture_path}/Actors/{name}")
            for child in body.GetChildren():
                UsdGeom.Imageable(child).MakeInvisible()
        self.surface = PillowSurface()
        self.table_top = table_top
        self.vectors = lambda data: Vt.Vec3fArray([Gf.Vec3f(*map(float, point)) for point in np.asarray(data).tolist()])
        mesh = UsdGeom.Mesh.Define(stage, f"{fixture_path}/PillowFabric")
        self.points = mesh.CreatePointsAttr()
        self.extent = mesh.CreateExtentAttr()
        mesh.CreateFaceVertexCountsAttr([4] * len(self.surface.faces))
        mesh.CreateFaceVertexIndicesAttr(self.surface.faces.ravel().tolist())
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorPrimvar("uniform").Set(self.vectors(self.surface.colors))

    def update(self, base_position, base_rotation, pad_tops):
        vertices = self.surface.vertices(base_position, base_rotation, pad_tops) + (0., 0., self.table_top)
        self.points.Set(self.vectors(vertices))
        self.extent.Set(self.vectors((vertices.min(axis=0), vertices.max(axis=0))))
