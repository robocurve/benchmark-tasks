"""Minimum clearance from the projected clear-lens outline to the tinted outline."""
import numpy as np
from .rounded_panel import vertices


class LensCoverage:
    def __init__(self, spec, actors):
        self.clear_name, self.tint_name = spec["a"], spec["b"]
        rings = []
        for name, index in [(self.clear_name,spec["clear_part"]), (self.tint_name,spec["tint_part"])]:
            part = actors[name]["parts"][index]
            if not np.allclose(part["euler_deg"], [0, 90, 0]):
                raise ValueError("Lens local axes must match the folded frame")
            radius = part["optical_surface"]["corner_m"]
            raw = np.asarray(vertices(part["size"], radius))
            # The optical axis was local X before the fixed 90-degree Y rotation.
            surface = raw[:len(raw)//2]
            ring = np.column_stack((surface[:,2],surface[:,1],-surface[:,0])) + part["offset"]
            rings.append(ring)
        self.clear, tint = rings
        self.tint = tint[:,:2]
        edges = np.roll(self.tint,-1,axis=0)-self.tint
        self.normals = np.column_stack((edges[:,1],-edges[:,0])) / np.linalg.norm(edges,axis=1)[:,None]
        # Choose inward-pointing normals for every side of the convex polygon.
        facing = np.sum((np.mean(self.tint,axis=0)-self.tint)*self.normals,axis=1)
        self.normals *= np.sign(facing)[:,None]

    def measure(self, frames):
        p, rotation = frames[self.clear_name][:2]
        q, tint_rotation = frames[self.tint_name][:2]
        projection = ((self.clear @ rotation.T + p - q) @ tint_rotation)[:,:2]
        separations = projection[:,None,:] - self.tint[None,:,:]
        return float(np.min(np.sum(separations*self.normals,axis=2)))
