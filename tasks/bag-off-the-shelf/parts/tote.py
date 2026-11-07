"""The stiff tote: a rigid open box of canvas look with a rigid grab handle on each side wall; its two long shoulder
straps are a separate soft cloth (bag_mesh.make_strap_mesh), sewn to its rim by vertex attachments (cloth.py).

Frame: the bag frame (origin at the centre of the floor's underside, x along the depth, y along the width, z up), as
the cloth bag. The walls' colliders stop TOTE_COLLIDER_GAP below the rim, where the straps are sewn, so the sewn strap
nodes never sit inside a collider; the rendered walls go up to the rim.
"""

from __future__ import annotations

import numpy as np

from parts.items import box, cube_collider, cylinder_along, cylinder_collider, write_rigid_usda

WALL = 0.004  # wall and floor thickness [m]
TOTE_COLLIDER_GAP = 0.01  # the walls' colliders end this far below the rim
HANDLE_RADIUS = 0.0075  # the grab bar
HANDLE_LENGTH = 0.10  # along x, between the standoffs' centres
HANDLE_OFFSET = 0.03  # the bar's axis, out from the side wall's outer face
HANDLE_Z = 0.16  # the bar's axis above the floor's underside
STANDOFF = 0.012  # square section of the two posts that hold the bar off the wall
TOTE_MASS = 0.30

MATERIALS = {
    "canvas": ((0.74, 0.66, 0.52), 0.0, 0.95),
    "hem": ((0.62, 0.54, 0.41), 0.0, 0.9),
    "handle": ((0.42, 0.30, 0.19), 0.0, 0.6),
    "steel": ((0.55, 0.56, 0.58), 1.0, 0.4),
}


def handle_axis_points(spec) -> dict[str, np.ndarray]:
    """The middle of each grab bar's axis, in the bag frame."""
    return {side: np.array([0.0, sgn * (spec.width / 2 + HANDLE_OFFSET), HANDLE_Z]) for side, sgn in (("left", 1.0), ("right", -1.0))}


def tote_parts(spec, soft_bottom: float = 0.0):
    """Visual meshes: floor, walls, a hem round the rim, and the two grab handles. With `soft_bottom` > 0 the floor and
    the walls' lowest `soft_bottom` are left out (they are cloth)."""
    d, w, h = spec.depth, spec.width, spec.height
    z0 = soft_bottom
    hw = h - z0
    parts = [] if soft_bottom else [("floor", box((d, w, WALL), (0.0, 0.0, WALL / 2)), "canvas")]
    for name, size, c in (("front", (WALL, w, hw), (-d / 2 + WALL / 2, 0.0, z0 + hw / 2)), ("back", (WALL, w, hw), (d / 2 - WALL / 2, 0.0, z0 + hw / 2)),
                          ("left", (d, WALL, hw), (0.0, w / 2 - WALL / 2, z0 + hw / 2)), ("right", (d, WALL, hw), (0.0, -w / 2 + WALL / 2, z0 + hw / 2))):
        parts.append((f"wall_{name}", box(size, c), "canvas"))
    hem_h, hem_t = 0.012, WALL + 0.002
    for name, size, c in (("front", (hem_t, w + 0.002, hem_h), (-d / 2 + WALL / 2, 0.0, h - hem_h / 2)),
                          ("back", (hem_t, w + 0.002, hem_h), (d / 2 - WALL / 2, 0.0, h - hem_h / 2)),
                          ("left", (d + 0.002, hem_t, hem_h), (0.0, w / 2 - WALL / 2, h - hem_h / 2)),
                          ("right", (d + 0.002, hem_t, hem_h), (0.0, -w / 2 + WALL / 2, h - hem_h / 2))):
        parts.append((f"hem_{name}", box(size, c), "hem"))
    for side, p in handle_axis_points(spec).items():
        sgn = 1.0 if side == "left" else -1.0
        parts.append((f"bar_{side}", cylinder_along(p - [HANDLE_LENGTH / 2 + STANDOFF / 2, 0, 0], p + [HANDLE_LENGTH / 2 + STANDOFF / 2, 0, 0],
                                                   HANDLE_RADIUS, 20), "handle"))
        for k, x in enumerate((-HANDLE_LENGTH / 2, HANDLE_LENGTH / 2)):
            y0, y1 = sgn * w / 2, p[1]
            parts.append((f"post_{side}_{k}", box((STANDOFF, abs(y1 - y0), STANDOFF), (x, (y0 + y1) / 2, HANDLE_Z)), "handle"))
    return parts


def tote_colliders(spec, root: str, soft_bottom: float = 0.0) -> list[str]:
    """Box colliders for the floor and walls (stopping TOTE_COLLIDER_GAP below the rim, and, with a soft bottom,
    starting 5 mm above its sewn edge), cylinders and boxes for the grab bars."""
    d, w, h = spec.depth, spec.width, spec.height
    z0 = soft_bottom + 0.005 if soft_bottom else 0.0
    hc = h - TOTE_COLLIDER_GAP - z0
    out = [] if soft_bottom else [cube_collider("c_floor", (d, w, WALL), (0.0, 0.0, WALL / 2), root)]
    for name, size, c in (("front", (WALL, w, hc), (-d / 2 + WALL / 2, 0.0, z0 + hc / 2)), ("back", (WALL, w, hc), (d / 2 - WALL / 2, 0.0, z0 + hc / 2)),
                          ("left", (d, WALL, hc), (0.0, w / 2 - WALL / 2, z0 + hc / 2)), ("right", (d, WALL, hc), (0.0, -w / 2 + WALL / 2, z0 + hc / 2))):
        out.append(cube_collider(f"c_{name}", size, c, root))
    for side, p in handle_axis_points(spec).items():
        sgn = 1.0 if side == "left" else -1.0
        out.append(cylinder_collider(f"c_bar_{side}", HANDLE_RADIUS, HANDLE_LENGTH + STANDOFF, "X", p, root))
        for k, x in enumerate((-HANDLE_LENGTH / 2, HANDLE_LENGTH / 2)):
            y0, y1 = sgn * w / 2, p[1]
            out.append(cube_collider(f"c_post_{side}_{k}", (STANDOFF, abs(y1 - y0), STANDOFF), (x, (y0 + y1) / 2, HANDLE_Z), root))
    return out


def write_tote_usda(path: str, spec, friction: tuple[float, float] = (0.8, 0.7), soft_bottom: float = 0.0) -> str:
    """The tote's rigid body; with `soft_bottom` > 0 only the walls above that height (the rest is cloth)."""
    root = "Tote"
    mass = TOTE_MASS * (1.0 - soft_bottom / spec.height) if soft_bottom else TOTE_MASS
    return write_rigid_usda(path, root, mass, tote_colliders(spec, root, soft_bottom), tote_parts(spec, soft_bottom), MATERIALS, friction)



ARCH_TUBE = 0.006  # radius of the arch handles' tube
ARCH_MASS = 0.17  # steel. Hanging from the strap it pushes the rake along hard enough to drag it with the strap almost upright
ARCH_CLEAR = 0.015  # the arch's colliders stop this far (along the arc) from its ends, where the legs are sewn


def arch_centre_line(end_a, end_b, plane: str, n: int = 40, phi0: float = 0.0) -> np.ndarray:
    """Points on a half circle from end_a to end_b: "down", hanging below them in their vertical plane; "out", lying
    flat and bulging along +x. `phi0` trims the ends (radians)."""
    c = (np.asarray(end_a) + np.asarray(end_b)) / 2
    r = np.linalg.norm(np.asarray(end_b) - np.asarray(end_a)) / 2
    phis = np.linspace(phi0, np.pi - phi0, n + 1)
    if plane == "down":
        return np.array([c + [0.0, -r * np.cos(f), -r * np.sin(f)] for f in phis])
    return np.array([c + [r * np.sin(f), -r * np.cos(f), ARCH_TUBE - 0.002] for f in phis])


def _tube(points: np.ndarray, radius: float, sides: int = 12):
    """A tube mesh along a polyline."""
    verts, faces = [], []
    for i, p in enumerate(points):
        t = points[min(i + 1, len(points) - 1)] - points[max(i - 1, 0)]
        t /= np.linalg.norm(t)
        a = np.cross(t, [0.0, 0.0, 1.0]) if abs(t[2]) < 0.9 else np.cross(t, [1.0, 0.0, 0.0])
        a /= np.linalg.norm(a)
        b = np.cross(t, a)
        verts += [p + radius * (np.cos(q) * a + np.sin(q) * b) for q in np.linspace(0, 2 * np.pi, sides, endpoint=False)]
    for i in range(len(points) - 1):
        for j in range(sides):
            a0, a1 = i * sides + j, i * sides + (j + 1) % sides
            faces += [[a0, a1, a1 + sides], [a0, a1 + sides, a0 + sides]]
    return np.array(verts), np.array(faces)


def _capsule_collider(name: str, p0, p1, radius: float, root: str) -> str:
    d = np.asarray(p1) - np.asarray(p0)
    h = float(np.linalg.norm(d))
    d /= h
    axis = np.cross([0.0, 0.0, 1.0], d)
    sa, ca = np.linalg.norm(axis), float(np.dot([0.0, 0.0, 1.0], d))
    if sa < 1e-9:
        q = (1.0, 0.0, 0.0, 0.0) if ca > 0 else (0.0, 1.0, 0.0, 0.0)
    else:
        ang = np.arctan2(sa, ca)
        ax = axis / sa
        q = (np.cos(ang / 2), *(np.sin(ang / 2) * ax))
    m = (np.asarray(p0) + np.asarray(p1)) / 2
    return f"""
    def Capsule "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </{root}/PhysicsMaterial>
        uniform token purpose = "guide"
        uniform token axis = "Z"
        double height = {h:.6f}
        double radius = {radius:.6f}
        double3 xformOp:translate = ({m[0]:.6f}, {m[1]:.6f}, {m[2]:.6f})
        quatf xformOp:orient = ({q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
    }}"""


def write_arch_usda(path: str, root: str, end_a, end_b, plane: str, friction: tuple[float, float] = (0.8, 0.7)) -> str:
    """A rigid arch handle (half ring of ARCH_TUBE tube) between two strap-leg ends, in the bag frame."""
    r = np.linalg.norm(np.asarray(end_b) - np.asarray(end_a)) / 2
    vis = arch_centre_line(end_a, end_b, plane, 40)
    cols = arch_centre_line(end_a, end_b, plane, 8, phi0=ARCH_CLEAR / r)
    colliders = [_capsule_collider(f"c_{i}", cols[i], cols[i + 1], ARCH_TUBE, root) for i in range(len(cols) - 1)]
    return write_rigid_usda(path, root, ARCH_MASS, colliders, [("tube", _tube(vis, ARCH_TUBE), "steel")], MATERIALS, friction)
