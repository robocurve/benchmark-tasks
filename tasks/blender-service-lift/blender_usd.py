"""A corded stick blender (an immersion blender) as one floating articulation, written as plain-text USD (no pxr
needed): the blender's body is the root link, and its power cord is a chain of short capsules ending in a plug.

Blender frame: x along the blender's axis, from the blending bell (x < 0) to the top of the motor housing (x > 0); the
origin is where the housing meets the shaft. The cord leaves the top of the housing along +x, through a short strain
relief boot, at `cord_exit`.

The cord: `n_links` capsules of length `link_length` and radius `cord_radius`, joined end to end by revolute joints whose
axes alternate between the link's local y and z, so the cord bends in any direction over two joints but does not
twist. The chain is authored along a given path, and joint position 0 is that authored shape, so a reset restores it.
Its joints carry no stiffness, only damping (`cord_drive`): the cord keeps no memory of the authored shape.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class BlenderSpec:
    # Colliders: (name, x0, x1, radius) cylinders along x, from the blending foot to the cord's strain relief. The
    # foot is a flared stainless bell wider than the grip; the grip is dark plastic with a light collar and a button.
    parts: tuple = (("foot", -0.165, -0.135, 0.035), ("flare", -0.135, -0.105, 0.020), ("shaft", -0.105, 0.0, 0.011),
                    ("grip", 0.0, 0.165, 0.033), ("boot", 0.165, 0.195, 0.0075))
    mass: float = 0.6
    cord_length: float = 0.40
    link_length: float = 0.02
    cord_radius: float = 0.0035
    link_mass: float = 0.006  # 300 g/m, about twice a real flex: lighter links pinched by the lift blew the solver up
    plug_size: tuple[float, float, float] = (0.035, 0.030, 0.016)  # the body: x along the cord, z up when lying flat
    pin_length: float = 0.017  # two round pins out of the plug's front, a Europlug's
    pin_radius: float = 0.002
    pin_spacing: float = 0.019
    plug_mass: float = 0.04
    joint_limit_deg: float = 150.0
    friction: float = 0.6
    solver_position_iterations: int = 32  # with the door's 8 N push: cord 0.32 mm into the door (12 N: 1.04 mm; 64: 0.75 mm)
    solver_velocity_iterations: int = 8
    # Cap on how fast PhysX pushes a cord link out of an overlap [m/s]. Must stay above the speed anything pushes into
    # the cord (car 0.2 m/s, fingers), or the pusher sinks into it (cloth poke test, 2026-10-29).
    max_depenetration: float | None = 1.0

    @property
    def cord_exit(self) -> np.ndarray:
        return np.array([self.parts[-1][2], 0.0, 0.0])

    @property
    def pin_tips(self) -> np.ndarray:
        """The pins' tips, in the plug's frame."""
        x = 0.5 * self.plug_size[0] + self.pin_length
        return np.array([[x, 0.5 * self.pin_spacing, 0.0], [x, -0.5 * self.pin_spacing, 0.0]])

    @property
    def n_links(self) -> int:
        return int(round(self.cord_length / self.link_length))

    @property
    def lying_height(self) -> float:
        """Height of the blender's axis above a flat surface it lies on."""
        return max(p[3] for p in self.parts)


def cord_drive() -> dict:
    """Implicit drives for the cord's joints (stiffness 0: the cord bends freely)."""
    return {"stiffness": 0.0, "damping": 2.0e-4, "armature": 1.0e-5}


def resample_chain(path: np.ndarray, step: float, n: int) -> np.ndarray:
    """n + 1 points along the polyline `path`, consecutive points exactly `step` apart (Euclidean), starting at path[0].
    Raises if the path is too short."""
    pts = [np.asarray(path[0], dtype=float)]
    seg, u_cur = 0, 0.0  # where the current point lies on the path
    while len(pts) < n + 1:
        cur, found = pts[-1], False
        # Every path point between cur and the next chain point lies within `step` of cur, so the next chain point is
        # where the path leaves the sphere of radius `step` around cur: the larger root on the first segment that has it.
        while seg < len(path) - 1:
            a, d = path[seg], path[seg + 1] - path[seg]
            f = a - cur
            A, B, C = d @ d, 2 * f @ d, f @ f - step * step
            disc = B * B - 4 * A * C
            if A > 0 and disc >= 0:
                u = (-B + np.sqrt(disc)) / (2 * A)
                if u_cur - 1e-12 <= u <= 1.0:
                    pts.append(a + u * d)
                    u_cur, found = u, True
                    break
            seg, u_cur = seg + 1, 0.0
        if not found:
            raise ValueError(f"cord path too short: {len(pts) - 1} of {n} links fit")
    return np.array(pts)


def _rot_between(u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Smallest rotation matrix taking unit vector u to unit vector v."""
    c = float(u @ v)
    axis = np.cross(u, v)
    s = np.linalg.norm(axis)
    if s < 1e-9:
        if c > 0:
            return np.eye(3)
        perp = np.cross(u, [1.0, 0, 0] if abs(u[0]) < 0.9 else [0, 1.0, 0])
        perp /= np.linalg.norm(perp)
        return 2 * np.outer(perp, perp) - np.eye(3)
    k = axis / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    ang = np.arctan2(s, c)
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def mat_to_quat_wxyz(R: np.ndarray) -> np.ndarray:
    """Unit quaternion (w, x, y, z) of a rotation matrix (Shepperd's method: exact signs near 180 degrees too)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = 2 * np.sqrt(tr + 1)
        q = [0.25 * S, (R[2, 1] - R[1, 2]) / S, (R[0, 2] - R[2, 0]) / S, (R[1, 0] - R[0, 1]) / S]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        S = 2 * np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / S, 0.25 * S, (R[0, 1] + R[1, 0]) / S, (R[0, 2] + R[2, 0]) / S]
    elif R[1, 1] > R[2, 2]:
        S = 2 * np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / S, (R[0, 1] + R[1, 0]) / S, 0.25 * S, (R[1, 2] + R[2, 1]) / S]
    else:
        S = 2 * np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / S, (R[0, 2] + R[2, 0]) / S, (R[1, 2] + R[2, 1]) / S, 0.25 * S]
    q = np.array(q)
    return q / np.linalg.norm(q)


def quat_wxyz_to_mat(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def chain_frames(points: np.ndarray) -> list[np.ndarray]:
    """Rotation matrix of each link (columns: local x along the link, y, z), parallel-transported along the chain so
    the links do not roll relative to each other."""
    dirs = np.diff(points, axis=0)
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    x = dirs[0]
    up = np.array([0.0, 0.0, 1.0]) if abs(x[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    frames = [np.column_stack([x, y, np.cross(x, y)])]
    for i in range(1, len(dirs)):
        frames.append(_rot_between(dirs[i - 1], dirs[i]) @ frames[-1])
    return frames


def _fmt(v) -> str:
    return "(" + ", ".join(f"{float(a):.7f}" for a in v) + ")"


def write_blender_usda(path: str, cord_path_local: np.ndarray, spec: BlenderSpec = BlenderSpec(),
                       up: tuple = (0.0, 0.0, 1.0), self_collision: bool = True, root_on_body: bool = True) -> str:
    """Write the blender and its cord. `cord_path_local` is a polyline in the blender frame starting at the cord's exit
    (spec.cord_exit); the chain follows it for spec.cord_length, then the plug continues along the last link. `up`: the
    world's up direction in the blender frame (the plug is authored lying flat). `root_on_body`: the articulation root
    is the blender's body (otherwise PhysX picks a link in the middle of the cord, and Isaac Lab's reset then puts that
    link, not the body, at the asset's initial pose). Returns `path`."""
    s, root = spec, "Blender"
    n, L, r = s.n_links, s.link_length, s.cord_radius
    path_pts = np.asarray(cord_path_local, dtype=float)
    if np.linalg.norm(path_pts[0] - s.cord_exit) > 1e-6:
        raise ValueError("cord path must start at the cord exit")
    pts = resample_chain(path_pts, L, n)
    frames = chain_frames(pts)
    grey, black, white = "(0.82, 0.83, 0.85)", "(0.08, 0.08, 0.09)", "(0.93, 0.93, 0.92)"
    mat = f"/{root}/PhysicsMaterial"

    body_parts = []
    steel, plastic, collar, dark, red = "(0.74, 0.75, 0.77)", "(0.16, 0.16, 0.17)", "(0.82, 0.82, 0.84)", "(0.05, 0.05, 0.06)", "(0.80, 0.10, 0.08)"
    colour = {"foot": steel, "flare": steel, "shaft": steel, "grip": plastic, "boot": black}
    for name, x0, x1, rad in s.parts:
        body_parts.append(f"""
        def Cylinder "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = <{mat}>
            uniform token axis = "X"
            double height = {x1 - x0:.6f}
            double radius = {rad:.6f}
            color3f[] primvars:displayColor = [{colour.get(name, grey)}]
            double3 xformOp:translate = ({0.5 * (x0 + x1):.6f}, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}""")
    # Visual only (no colliders): the foot's flare as a cone, its dark open end and four vent windows, the grip's light
    # collar and domed top, and the power button.
    foot, flare, grip = (next(p for p in s.parts if p[0] == k) for k in ("foot", "flare", "grip"))
    fr = foot[3]
    vis = [f"""
        def Cone "flare_cone"
        {{
            uniform token axis = "X"
            double height = 0.045
            double radius = {fr - 0.001:.6f}
            color3f[] primvars:displayColor = [{steel}]
            double3 xformOp:translate = ({foot[2] + 0.0225:.6f}, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}""", f"""
        def Cylinder "foot_mouth"
        {{
            uniform token axis = "X"
            double height = 0.001
            double radius = {fr - 0.003:.6f}
            color3f[] primvars:displayColor = [{dark}]
            double3 xformOp:translate = ({foot[1] - 0.0004:.6f}, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}""", f"""
        def Cylinder "collar"
        {{
            uniform token axis = "X"
            double height = 0.035
            double radius = {grip[3] + 0.0008:.6f}
            color3f[] primvars:displayColor = [{collar}]
            double3 xformOp:translate = ({grip[1] + 0.0175:.6f}, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}""", f"""
        def Sphere "top_cap"
        {{
            double radius = {grip[3]:.6f}
            color3f[] primvars:displayColor = [{plastic}]
            double3 xformOp:translate = ({grip[2]:.6f}, 0, 0)
            float3 xformOp:scale = (0.45, 1, 1)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}""", f"""
        def Cube "button"
        {{
            double size = 1
            color3f[] primvars:displayColor = [{red}]
            double3 xformOp:translate = ({grip[2] - 0.035:.6f}, 0, {grip[3] + 0.0015:.6f})
            float3 xformOp:scale = (0.022, 0.014, 0.004)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}"""]
    for k, ang in enumerate((45.0, 135.0, 225.0, 315.0)):
        a = np.radians(ang)
        vis.append(f"""
        def Cube "vent_{k}"
        {{
            double size = 1
            color3f[] primvars:displayColor = [{dark}]
            double3 xformOp:translate = ({0.5 * (foot[1] + foot[2]):.6f}, {(fr + 0.0004) * np.cos(a):.6f}, {(fr + 0.0004) * np.sin(a):.6f})
            float3 xformOp:rotateXYZ = ({ang:.1f}, 0, 0)
            float3 xformOp:scale = (0.016, 0.0008, 0.014)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
        }}""")
    body_parts += vis
    art_attrs = f"""
        bool physxArticulation:enabledSelfCollisions = {int(self_collision)}
        int physxArticulation:solverPositionIterationCount = {s.solver_position_iterations}
        int physxArticulation:solverVelocityIterationCount = {s.solver_velocity_iterations}"""
    body_api = ', "PhysicsArticulationRootAPI", "PhysxArticulationAPI"' if root_on_body else ""
    links = [f"""
    def Xform "body" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"{body_api}]
    )
    {{{art_attrs if root_on_body else ""}
        float physics:mass = {s.mass:.4f}
{''.join(body_parts)}
    }}
"""]
    quats = [mat_to_quat_wxyz(R) for R in frames]
    depen_api = ', "PhysxRigidBodyAPI"' if s.max_depenetration is not None else ""
    depen_attr = (f"\n        float physxRigidBody:maxDepenetrationVelocity = {s.max_depenetration:.4f}"
                  if s.max_depenetration is not None else "")
    for R, q in zip(frames, quats):  # the authored link poses must match the joint frames exactly
        assert np.abs(quat_wxyz_to_mat(q) - R).max() < 1e-9
    for i in range(n):
        mid = 0.5 * (pts[i] + pts[i + 1])
        links.append(f"""
    def Xform "link_{i:03d}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"{depen_api}]
    )
    {{
        float physics:mass = {s.link_mass:.5f}{depen_attr}
        double3 xformOp:translate = {_fmt(mid)}
        quatd xformOp:orient = {_fmt(quats[i])}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

        def Capsule "capsule" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = <{mat}>
            uniform token axis = "X"
            double height = {L - 2 * r:.6f}
            double radius = {r:.6f}
            token visibility = "invisible"
        }}

        def Capsule "skin"
        {{
            uniform token axis = "X"
            double height = {L:.6f}
            double radius = {r:.6f}
            color3f[] primvars:displayColor = [{black}]
        }}
    }}
""")
    # The plug lies flat: its x continues the last link, its z is as close to `up` as that allows, and its bottom is
    # level with the cord's bottom, so a plug at the end of a cord lying on a surface rests on it.
    px, py, pz = s.plug_size
    x = frames[-1][:, 0]
    up = np.asarray(up, dtype=float)
    z = up - (up @ x) * x
    z /= np.linalg.norm(z)
    R_plug = np.column_stack([x, np.cross(z, x), z])
    plug_lift = 0.5 * pz - r
    plug_mid = pts[-1] + x * (0.5 * px) + z * plug_lift
    q_plug = mat_to_quat_wxyz(R_plug)
    pins = "".join(f"""
        def Cylinder "pin_{k}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = <{mat}>
            uniform token axis = "X"
            double height = {s.pin_length:.6f}
            double radius = {s.pin_radius:.6f}
            color3f[] primvars:displayColor = [(0.80, 0.72, 0.45)]
            double3 xformOp:translate = ({0.5 * px + 0.5 * s.pin_length:.6f}, {sign * 0.5 * s.pin_spacing:.6f}, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}""" for k, sign in enumerate((1.0, -1.0)))
    links.append(f"""
    def Xform "plug" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    )
    {{
        float physics:mass = {s.plug_mass:.4f}
        double3 xformOp:translate = {_fmt(plug_mid)}
        quatd xformOp:orient = {_fmt(q_plug)}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

        def Cube "shell" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = <{mat}>
            double size = 1
            token visibility = "invisible"
            float3 xformOp:scale = ({px:.6f}, {py:.6f}, {pz:.6f})
            uniform token[] xformOpOrder = ["xformOp:scale"]
        }}

        def Cylinder "body"
        {{
            uniform token axis = "X"
            double height = {0.72 * px:.6f}
            double radius = {0.5 * py:.6f}
            color3f[] primvars:displayColor = [{white}]
            double3 xformOp:translate = ({0.14 * px:.6f}, 0, 0)
            float3 xformOp:scale = (1, 1, {pz / py:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}

        def Sphere "tail"
        {{
            double radius = 1
            color3f[] primvars:displayColor = [{white}]
            double3 xformOp:translate = ({-0.22 * px:.6f}, 0, 0)
            float3 xformOp:scale = ({0.28 * px:.6f}, {0.5 * py:.6f}, {0.5 * pz:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}

        def Cylinder "strain_relief"
        {{
            uniform token axis = "X"
            double height = 0.016
            double radius = {r + 0.002:.6f}
            color3f[] primvars:displayColor = [{black}]
            double3 xformOp:translate = ({-0.5 * px + 0.002:.6f}, 0, {-plug_lift:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}
{pins}
    }}
""")

    def joint(name, body0, body1, pos0, rot0, pos1, axis):
        lim = s.joint_limit_deg
        # Damping only (the actuator sets its value); without a DriveAPI PhysX gives the joint no drive at all.
        return f"""
    def PhysicsRevoluteJoint "{name}" (
        prepend apiSchemas = ["PhysicsDriveAPI:angular"]
    )
    {{
        uniform token drive:angular:physics:type = "force"
        float drive:angular:physics:stiffness = 0
        float drive:angular:physics:damping = 0
        rel physics:body0 = </{root}/{body0}>
        rel physics:body1 = </{root}/{body1}>
        uniform token physics:axis = "{axis}"
        point3f physics:localPos0 = {_fmt(pos0)}
        quatf physics:localRot0 = {_fmt(rot0)}
        point3f physics:localPos1 = {_fmt(pos1)}
        quatf physics:localRot1 = (1, 0, 0, 0)
        float physics:lowerLimit = {-lim:.1f}
        float physics:upperLimit = {lim:.1f}
    }}
"""

    joints = []
    # Joint i joins its parent (the body, or link i-1) to link i at pts[i]; its frame is link i's frame.
    for i in range(n):
        if i == 0:
            parent, R_p, o_p = "body", np.eye(3), np.zeros(3)
        else:
            parent, R_p, o_p = f"link_{i - 1:03d}", frames[i - 1], 0.5 * (pts[i - 1] + pts[i])
        pos0 = R_p.T @ (pts[i] - o_p)
        rot0 = mat_to_quat_wxyz(R_p.T @ frames[i])
        joints.append(joint(f"cj_{i:03d}", parent, f"link_{i:03d}", pos0, rot0, (-0.5 * L, 0.0, 0.0), "Y" if i % 2 == 0 else "Z"))
    joints.append(joint("cj_plug", f"link_{n - 1:03d}", "plug", (0.5 * L, 0.0, 0.0), mat_to_quat_wxyz(frames[-1].T @ R_plug),
                        (-0.5 * px, 0.0, -plug_lift), "Y" if n % 2 == 0 else "Z"))

    root_api = "" if root_on_body else ' (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)'
    text = f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}"{root_api}
{{{"" if root_on_body else art_attrs}

    def Material "PhysicsMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {{
        float physics:staticFriction = {s.friction:.4f}
        float physics:dynamicFriction = {s.friction:.4f}
        float physics:restitution = 0
    }}
{''.join(links)}{''.join(joints)}}}
"""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path


def chain_points(cord_path_local: np.ndarray, spec: BlenderSpec = BlenderSpec()) -> np.ndarray:
    """The cord's joint points as authored (n_links + 1 points in the blender frame)."""
    return resample_chain(np.asarray(cord_path_local, dtype=float), spec.link_length, spec.n_links)
