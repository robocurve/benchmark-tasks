"""Geometric clearance for an explicitly assumed wrist enclosure around each optical centre."""

import json

import numpy as np
from scipy.spatial.transform import Rotation

from evd.layout import LINKS, NAMES, ROOT

# Assumed installation envelope, including the lens and mount. This is not a measured housing.
RADIUS_M = 0.050
SIDES = ("left", "right")


class WristClearance:
    def __init__(self, scene, offsets):
        # Inspect imports the task before Kit starts. Load its USD bindings only
        # when the live monitor (or an offline geometry check) needs the stage.
        from pxr import Usd, UsdGeom, UsdPhysics

        stage = Usd.Stage.Open(str(scene))
        centers, extents, owners = [], [], []
        self.labels = []
        for i, name in enumerate(NAMES):
            body = stage.GetPrimAtPath("/Trainer/" + name)
            for child in body.GetChildren():
                if not child.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                # Every primitive in this trainer is a scaled unit cube, unit diameter
                # ellipsoid or unit cylinder. The local boxes enclose all three.
                if child.GetTypeName() not in ("Cube", "Sphere", "Cylinder"):
                    raise ValueError(f"Unsupported camera clearance primitive {child.GetPath()}")
                mat = np.asarray(UsdGeom.Xformable(child).GetLocalTransformation()).T
                centers.append(mat[:3, 3])
                extents.append(np.abs(mat[:3, :3]).sum(axis=1) / 2)
                owners.append(i)
                self.labels.append(name + "/" + child.GetName())
        self.centers = np.asarray(centers)
        self.extents = np.asarray(extents)
        self.owners = np.array(owners)
        self.knob = self.labels.index("tap/knob")
        self.offsets = np.array([offsets[side] for side in SIDES])
        bounds = json.loads((ROOT / "assets/link_bounds.json").read_text())["bounds"]
        self.link_boxes = np.asarray([bounds[name] for name in LINKS])
        self.minimum = np.full((2, 3), np.inf)
        self.records = {side: {} for side in SIDES}
        self.samples = 0

    def sample(self, tick, poses, arms):
        grip = np.array([arms[s][LINKS.index("gripper")] for s in SIDES])
        lenses = grip[:, :3] + Rotation.from_quat(grip[:, 3:]).apply(self.offsets)
        bodies = np.array([poses[n] for n in NAMES])
        rot = Rotation.from_quat(bodies[:, 3:]).as_matrix()
        local = np.einsum("sbi,bij->sbj", lenses[:, None] - bodies[None, :, :3], rot)
        gaps = np.maximum(abs(local[:, self.owners] - self.centers) - self.extents, 0)
        distance = 1000 * (np.linalg.norm(gaps, axis=2) - RADIUS_M)
        for i, side in enumerate(SIDES):
            nearest = np.argmin(distance[i])
            for j, (part, key) in enumerate(((nearest, "apparatus"), (self.knob, "knob"))):
                value = distance[i, part]
                if value < self.minimum[i, j]:
                    self.minimum[i, j] = value
                    self.records[side][key] = dict(mm=float(value), tick=int(tick), part=self.labels[part])
            other = SIDES[1 - i]
            links = np.asarray(arms[other])
            local_lens = Rotation.from_quat(links[:, 3:]).inv().apply(lenses[i] - links[:, :3])
            gap = np.maximum(abs(local_lens - self.link_boxes[:, 0]) - self.link_boxes[:, 1], 0)
            link_distances = 1000 * (np.linalg.norm(gap, axis=1) - RADIUS_M)
            link = int(np.argmin(link_distances))
            if link_distances[link] < self.minimum[i, 2]:
                self.minimum[i, 2] = link_distances[link]
                self.records[side]["opposite_arm"] = {
                    "mm": float(link_distances[link]), "tick": int(tick), "part": other + "/" + LINKS[link]
                }
        self.samples += 1

    def report(self):
        return dict(radius_mm=RADIUS_M * 1000, samples=self.samples, closest=self.records)
