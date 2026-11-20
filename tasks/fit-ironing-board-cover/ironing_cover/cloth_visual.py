"""Render the cotton skin and sewn skirt from the actual PhysX pad and hem poses."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from .builder import DECK_OUTLINE
from .config import BOARD_X, HALF_LENGTH, NEUTRAL_LENGTH, START_Z


class CoverVisual:
    """An explicitly visual skin. The colliders and scorer use the native bodies."""

    def __init__(self, stage, root):
        from pxr import Gf, Sdf, UsdGeom, UsdShade

        self.outline = np.asarray(DECK_OUTLINE)
        vertices = len(self.outline)
        self.mesh = UsdGeom.Mesh.Define(stage, root + "/CoverFabric")
        self.mesh.CreateSubdivisionSchemeAttr("none")
        self.mesh.CreateDoubleSidedAttr(True)
        indices = []
        for i in range(vertices):
            j = (i + 1) % vertices
            indices.extend((0, 1 + i, 1 + j))
        for ring in range(2):
            a, b = 1 + ring * vertices, 1 + (ring + 1) * vertices
            for i in range(vertices):
                j = (i + 1) % vertices
                indices.extend((a + i, b + i, b + j, a + j))
        self.points = self.mesh.CreatePointsAttr()
        self.extent = self.mesh.CreateExtentAttr()
        self.update({
            "pad_pose": [BOARD_X, 0., START_Z, 0., 0., 0., 1.],
            "hem_positions": {
                "Nose": [BOARD_X, NEUTRAL_LENGTH, START_Z + .008],
                "Heel": [BOARD_X, -NEUTRAL_LENGTH, START_Z + .008],
            },
        })
        self.mesh.CreateFaceVertexCountsAttr([3] * vertices + [4] * (2 * vertices))
        self.mesh.CreateFaceVertexIndicesAttr(indices)

        color = Gf.Vec3f(.065, .42, .53)
        self.mesh.CreateDisplayColorAttr([color])
        material = UsdShade.Material.Define(stage, root + "/CoverCotton")
        shader = UsdShade.Shader.Define(stage, root + "/CoverCotton/FabricSurface")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.92)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.)
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(self.mesh.GetPrim()).Bind(material)

    def update(self, state):
        from pxr import Gf

        pad = np.asarray(state["pad_pose"])
        orientation = quat_xyzw_to_matrix(pad[3:])
        top, skirt = [], []
        for x, y in self.outline:
            sign = 1 if y > 0 else -1
            name = "Nose" if sign == 1 else "Heel"
            seam = orientation.T @ (np.asarray(state["hem_positions"][name]) - pad[:3])
            drop = float(np.clip((.004 - seam[2]) / .034, 0., 1.))
            end_fraction = max(0., (abs(y) - .170) / (HALF_LENGTH - .170))
            loose_tip = abs(float(seam[1])) + .012
            fitted_tip = max(HALF_LENGTH + .003, abs(float(seam[1])) + .004)
            edge_y = y + sign * end_fraction * (loose_tip * (1. - drop) + fitted_tip * drop - HALF_LENGTH)
            z = .006 + .009 * (1. - drop) * end_fraction**2
            top.append([x * 1.017, edge_y, z])
            pull = drop * end_fraction**2
            skirt.append([x * 1.021, edge_y * (1. - pull) + seam[1] * pull,
                          -.006 * (1. - pull) + (seam[2] + .002) * pull])

        edge = np.asarray(top)
        middle = edge * [.78, .78, .22] + [0., 0., .0047]
        vertices = np.vstack(([0., 0., .006], middle, edge, skirt)) @ orientation.T + pad[:3]
        self.points.Set([Gf.Vec3f(*point) for point in vertices])
        self.extent.Set([Gf.Vec3f(*vertices.min(axis=0)), Gf.Vec3f(*vertices.max(axis=0))])
