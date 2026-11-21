"""Use the installed YAM station's frames, arm meshes, joint limits and camera calibration."""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ASSET_DIR, YAM_URDF, station
from yam_arena.urdf import parse_joints, rpy_matrix


def numbers(values):
    return " ".join(f"{v:.10g}" for v in np.atleast_1d(values))


def quat(matrix):
    return numbers(Rotation.from_matrix(matrix).as_quat(scalar_first=True))


def camera(parent, name, spec):
    lens = spec.lens
    # The calibration is expressed for the cropped 640 x 360 image.
    ET.SubElement(
        parent,
        "camera",
        name=name,
        pos=numbers(spec.pos),
        quat=numbers(np.roll(spec.rot_xyzw, 1)),
        resolution="640 360",
        sensorsize=".0064 .0036",
        focalpixel=numbers((lens.fx, lens.fy)),
        principalpixel=numbers((lens.cx - 320, 180 - lens.cy)),
    )


def add_arms(root):
    assets, world, drives = (root.find(k) for k in ("asset", "worldbody", "actuator"))
    joints = parse_joints(YAM_URDF)
    links = {n.get("name"): n for n in ET.parse(YAM_URDF).findall("link")}
    for name, link in links.items():
        path = link.find("visual/geometry/mesh").get("filename")
        ET.SubElement(assets, "mesh", name=name, file=str(ASSET_DIR / path))
    # Preserve the concave wrist and palm geometry where these links fold together.
    precision = ("gripper", "link4")
    for name in precision:
        for mesh in links[name].findall("collision/geometry/mesh"):
            path = ASSET_DIR / mesh.get("filename")
            ET.SubElement(assets, "mesh", name=path.stem, file=str(path))
    for name, parts in station.ARM_PARTS.items():
        for color, path in parts.items():
            ET.SubElement(assets, "mesh", name=f"{name}_{color}", file=str(path))
    camera(world, "top_cam", station.top_camera())
    for side, base in station.BASES.items():
        bodies = {"base": ET.SubElement(world, "body", name=f"{side}_base", pos=numbers(base))}
        for name, j in joints.items():
            child = ET.SubElement(
                bodies[j.parent],
                "body",
                name=f"{side}_{j.child}",
                pos=numbers(j.xyz),
                quat=quat(rpy_matrix(j.rpy)),
                gravcomp="1",
            )
            bodies[j.child] = child
            finger = j.type == "prismatic"
            shoulder = name in ("joint1", "joint2", "joint3")
            joint = side + "_" + name
            ET.SubElement(
                child,
                "joint",
                name=joint,
                type="slide" if finger else "hinge",
                axis=numbers(j.axis),
                range=numbers((j.lower, j.upper)),
                armature=".04" if finger else (".032" if shoulder else ".002"),
            )
            limit = 50 if finger else (27 if shoulder else 7)
            ET.SubElement(
                drives,
                "position",
                name=joint,
                joint=joint,
                kp="5000" if finger else ("110" if shoulder else "28"),
                kv="55" if finger else ("6" if shoulder else "2"),
                forcerange=numbers((-limit, limit)),
                ctrlrange=numbers((j.lower, j.upper)),
            )
            if finger:
                # Fingertip pads and backings in the flange frame at zero aperture.
                sign = -1 if j.child == "tip_left" else 1
                rot = rpy_matrix(j.rpy)
                for label, x, z, size in (
                    ("pad", 0.003, 0.131, (0.003, 0.009, 0.015)),
                    ("back", 0.006, 0.100, (0.005, 0.009, 0.018)),
                ):
                    p = rot.T @ (np.array([sign * x, 0, z]) - j.xyz)
                    ET.SubElement(
                        child,
                        "geom",
                        name=f"{side}_{j.child}_{label}",
                        type="box",
                        size=numbers(size),
                        pos=numbers(p),
                        quat=quat(rot.T),
                        rgba=".10 .11 .12 1",
                        friction="1.3 .008 .0003",
                        condim="4",
                    )
        for name, body in bodies.items():
            link = links[name]
            raw = link.find("inertial")
            origin = raw.find("origin")
            rot = rpy_matrix(np.fromstring(origin.get("rpy"), sep=" "))
            a, b, c, d, e, f = (
                float(raw.find("inertia").get(i)) for i in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            )
            inertia = rot @ np.array([[a, d, e], [d, b, f], [e, f, c]]) @ rot.T
            ET.SubElement(
                body,
                "inertial",
                mass=raw.find("mass").get("value"),
                pos=origin.get("xyz"),
                fullinertia=numbers(inertia[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]),
            )
            v = link.find("visual/origin")
            attrs = dict(
                type="mesh",
                mesh=name,
                name=f"{side}_{name}_mesh",
                pos=v.get("xyz"),
                quat=quat(rpy_matrix(np.fromstring(v.get("rpy"), sep=" "))),
                rgba=".12 .13 .15 1",
            )
            if name in precision:
                for col in link.findall("collision"):
                    o = col.find("origin")
                    path = ASSET_DIR / col.find("geometry/mesh").get("filename")
                    ET.SubElement(
                        body,
                        "geom",
                        name=f"{side}_{path.stem}",
                        type="mesh",
                        mesh=path.stem,
                        pos=o.get("xyz"),
                        quat=quat(rpy_matrix(np.fromstring(o.get("rpy"), sep=" "))),
                        group="3",
                    )
            if name.startswith("tip") or name in precision:
                attrs.update(contype="0", conaffinity="0")
            if name in station.ARM_PARTS:
                attrs["group"] = "3"  # full convex collision hull, separate two-tone visual mesh
                for color in station.ARM_PARTS[name]:
                    ET.SubElement(
                        body,
                        "geom",
                        type="mesh",
                        mesh=f"{name}_{color}",
                        quat=quat(station.ARM_PART_ROT),
                        pos=numbers(station.ARM_PART_POS[name]),
                        rgba=numbers((*getattr(station, "ARM_" + color.upper()), 1)),
                        contype="0",
                        conaffinity="0",
                    )
            ET.SubElement(body, "geom", **attrs)
        camera(bodies["gripper"], side + "_cam", station.wrist_camera(side))
        ET.SubElement(bodies["gripper"], "site", name=side + "_tcp", pos="0 0 .134", size=".001", group="5")
        # The real sliding fingers cross the palm and each other during closing.
        for a, b in (
            ("base", "link1"),
            ("tip_left", "tip_right"),
            ("link5", "tip_left"),
            ("link5", "tip_right"),
        ):
            ET.SubElement(root.find("contact"), "exclude", body1=f"{side}_{a}", body2=f"{side}_{b}")
