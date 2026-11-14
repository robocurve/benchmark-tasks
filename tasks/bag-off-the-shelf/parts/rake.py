"""A small hand rake as one rigid body, written as plain-text USD (no pxr needed).

Rake frame: origin on the supporting surface, under the centre of the head. The head is a bar along y lying on the
surface; the tines stand up from it along +z; the handle lies on the surface along +x. In the hooked scene the rake
lies behind the bag with its tines through the bag's back loop, so pulling the bag towards the robot drags the rake.
"""

from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class RakeSpec:
    head_length: float = 0.09  # along y
    head_size: float = 0.018  # head bar cross-section
    num_tines: int = 3
    tine_spacing: float = 0.035
    tine_height: float = 0.115  # top of the tines above the supporting surface
    tine_radius: float = 0.004
    handle_length: float = 0.09  # short enough for a rake moved left along the shelf to come clear of the strap's +y leg
    handle_radius: float = 0.011
    mass: float = 0.3
    # With the "min" combine mode these are the friction against anything. The head and the handle are smooth steel and
    # varnished wood: the rake slides when the strap drags it (at 0.6 it stays, the strap rides up over the tines and a
    # plain carry takes the tote without it). The tines are rough enough for a pinch to hold the rake's 300 g.
    friction: float = 0.05
    tine_friction: float = 0.6

    def tine_ys(self) -> list[float]:
        """Tine positions along y."""
        n = self.num_tines
        return [(i - (n - 1) / 2) * self.tine_spacing for i in range(n)]


def _capsule(name: str, radius: float, height: float, axis: str, pos: tuple[float, float, float], color: str,
             material: str = "PhysicsMaterial") -> str:
    return f"""
        def Capsule "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = </Rake/{material}>
            uniform token axis = "{axis}"
            double height = {height:.6f}
            double radius = {radius:.6f}
            color3f[] primvars:displayColor = [{color}]
            double3 xformOp:translate = ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}"""


def write_rake_usda(path: str, spec: RakeSpec = RakeSpec()) -> str:
    """Write the rake as a rigid body: a box head, capsule tines and a capsule handle. Returns `path`."""
    s = spec
    metal, wood = "(0.30, 0.31, 0.33)", "(0.55, 0.36, 0.18)"
    tine_len = s.tine_height - s.head_size  # straight part above the head
    parts = []
    for i, y in enumerate(s.tine_ys()):
        cyl = tine_len - s.tine_radius  # capsule caps add one radius at each end
        parts.append(_capsule(f"tine{i}", s.tine_radius, cyl, "Z", (0.0, y, s.head_size + 0.5 * cyl), metal, "TineMaterial"))
    handle_cyl = s.handle_length - 2 * s.handle_radius
    parts.append(_capsule("handle", s.handle_radius, handle_cyl, "X",
                          (0.5 * s.head_size + 0.5 * s.handle_length, 0.0, s.handle_radius), wood))
    body = "".join(parts)
    text = f"""#usda 1.0
(
    defaultPrim = "Rake"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Rake" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{{
    float physics:mass = {s.mass:.4f}

    def Material "PhysicsMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {{
        float physics:staticFriction = {s.friction:.4f}
        float physics:dynamicFriction = {s.friction:.4f}
        float physics:restitution = 0
        uniform token physxMaterial:frictionCombineMode = "min"
    }}

    def Material "TineMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {{
        float physics:staticFriction = {s.tine_friction:.4f}
        float physics:dynamicFriction = {s.tine_friction:.4f}
        float physics:restitution = 0
        uniform token physxMaterial:frictionCombineMode = "min"
    }}

    def Cube "head" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </Rake/PhysicsMaterial>
        double size = 1
        color3f[] primvars:displayColor = [{metal}]
        double3 xformOp:translate = (0, 0, {0.5 * s.head_size:.6f})
        float3 xformOp:scale = ({s.head_size:.6f}, {s.head_length:.6f}, {s.head_size:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}

    def Xform "parts"
    {{{body}
    }}
}}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path
