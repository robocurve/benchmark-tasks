"""One YAM, an articulated weighted lower leg, and a free foam cushion on a bed.

All contacts, including cushion motion, are solved by MuJoCo. The only actuators
are the arm/fingers and an independent, transient voluntary knee moment.
"""

import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco as mj
import numpy as np
from scipy.spatial.transform import Rotation

ASSETS = Path(__file__).parent / "assets/yam"
READY = np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])
BED_Z = 0.0
HZ = 20
SUBSTEPS = 20
STEPS = 280


def numbers(values):
    return " ".join(f"{v:.10g}" for v in values)


def node(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def quat(rpy):
    q = Rotation.from_euler("xyz", np.fromstring(rpy, sep=" ")).as_quat()
    return numbers(q[[3, 0, 1, 2]])


def arm(world, assets, actuators):
    urdf = ET.parse(ASSETS / "yam.urdf")
    links = {a.get("name"): a for a in urdf.findall("link")}
    parts = {"base": node(world, "body", name="base", pos="-.185 0 .025")}
    for joint in urdf.findall("joint"):
        parent = parts[joint.find("parent").get("link")]
        child = joint.find("child").get("link")
        ori = joint.find("origin")
        body = node(
            parent,
            "body",
            name=child,
            pos=ori.get("xyz"),
            quat=quat(ori.get("rpy")),
            gravcomp="1",
        )
        parts[child] = body
        name = joint.get("name")
        i = int(name[-1])
        limits = joint.find("limit")
        rng = limits.get("lower") + " " + limits.get("upper")
        node(
            body,
            "joint",
            name=name,
            axis=joint.find("axis").get("xyz"),
            range=rng,
            type="hinge" if i <= 6 else "slide",
            damping=0.35 if i <= 6 else 4,
            armature=0.025 if i <= 6 else 0.002,
        )
        effort = 27 if i <= 3 else 7 if i <= 6 else 30
        node(
            actuators,
            "position",
            name=name,
            joint=name,
            kp=160 if i <= 3 else 65 if i <= 6 else 800,
            kv=10 if i <= 3 else 3 if i <= 6 else 8,
            forcerange=f"-{effort} {effort}",
            ctrlrange=rng,
        )
    for name, body in parts.items():
        link = links[name]
        inertial = link.find("inertial")
        inert = inertial.find("inertia")
        tensor = np.array(
            [
                [float(inert.get("i" + a + b) or inert.get("i" + b + a)) for b in "xyz"]
                for a in "xyz"
            ]
        )
        r = Rotation.from_euler(
            "xyz", np.fromstring(inertial.find("origin").get("rpy"), sep=" ")
        ).as_matrix()
        tensor = r @ tensor @ r.T
        node(
            body,
            "inertial",
            pos=inertial.find("origin").get("xyz"),
            mass=inertial.find("mass").get("value"),
            fullinertia=numbers(tensor[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]),
        )
        vis = link.find("visual")
        node(assets, "mesh", name=name, file=vis.find("geometry/mesh").get("filename"))
        node(
            body,
            "geom",
            name="arm_" + name,
            type="mesh",
            mesh=name,
            pos=vis.find("origin").get("xyz"),
            quat=quat(vis.find("origin").get("rpy")),
            rgba=".15 .29 .40 1" if name.startswith("link") else ".25 .36 .40 1",
            contype="2",
            conaffinity="1",
        )
    node(
        parts["gripper"],
        "site",
        name="tcp",
        pos="0 0 .120",
        size=".003",
        rgba="0 0 0 0",
    )


def sleeve_surface(assets, leg):
    """Continuous cloth silhouette above the two instrumented calf regions."""
    # Rows give longitudinal coordinate, lateral radius, vertical centre and vertical radius.
    rings = [
        (-0.295, 0.033, -0.006, 0.030),
        (-0.265, 0.044, -0.007, 0.043),
        (-0.220, 0.050, -0.010, 0.054),
        (-0.186, 0.049, -0.008, 0.051),
        (-0.150, 0.049, -0.004, 0.046),
        (-0.115, 0.052, -0.005, 0.051),
        (-0.080, 0.053, -0.006, 0.051),
        (-0.035, 0.048, -0.004, 0.042),
        (0.005, 0.039, 0.000, 0.035),
    ]
    vertices = []
    faces = []
    sides = 24
    for n, (y, rx, z, rz) in enumerate(rings):
        for j in range(sides):
            angle = 2 * np.pi * j / sides
            vertices.extend((rx * np.cos(angle), y, z + rz * np.sin(angle)))
            if n:
                a = (n - 1) * sides + j
                b = (n - 1) * sides + (j + 1) % sides
                faces.extend((a, a + sides, b, b, a + sides, b + sides))
    for j in range(1, sides - 1):
        faces.extend((0, j, j + 1))
        end = (len(rings) - 1) * sides
        faces.extend((end, end + j + 1, end + j))
    node(
        assets,
        "mesh",
        name="trouser_surface",
        vertex=numbers(vertices),
        face=" ".join(map(str, faces)),
    )
    node(
        leg,
        "geom",
        name="trouser_visual",
        type="mesh",
        mesh="trouser_surface",
        rgba=".19 .39 .54 1",
        contype=0,
        conaffinity=0,
        group=1,
    )
    node(
        leg,
        "geom",
        name="instep_visual",
        type="capsule",
        fromto="0 -.34 -.006 0 -.401 .086",
        size=".033",
        rgba=".76 .55 .43 1",
        contype=0,
        conaffinity=0,
        group=1,
    )


def make_model(
    cushion_y=0.020, stiffness=850.0, leg_mass=3.0, resident_y=0.0, dual=False
):
    root = ET.Element("mujoco", model="heel offloading")
    node(root, "compiler", angle="radian", meshdir=str(ASSETS), autolimits="true")
    node(
        root,
        "option",
        timestep=1 / (HZ * SUBSTEPS),
        integrator="implicitfast",
        iterations=60,
        cone="elliptic",
    )
    visual = node(root, "visual")
    node(visual, "global", offwidth=960, offheight=640)
    node(visual, "headlight", ambient=".65 .65 .65")
    defaults = node(root, "default")
    node(
        defaults,
        "geom",
        friction=".48 .008 .001",
        solref=".012 1",
        solimp=".92 .98 .001",
    )
    assets = node(root, "asset")
    world = node(root, "worldbody")
    act = node(root, "actuator")
    arm(world, assets, act)
    if dual:
        from copy import deepcopy

        # Companion measurement model for the left arm, 610 mm from the working right base.
        extra = deepcopy(world.find("body"))
        extra.set("pos", "-.185 .610 .025")
        for e in extra.iter():
            n = e.get("name")
            if n:
                e.set("name", "other_" + (n[4:] if n.startswith("arm_") else n))
        world.append(extra)
    node(world, "light", pos="-.3 -.5 1.6", dir=".3 .4 -1", diffuse=".64 .63 .60")
    node(
        world,
        "geom",
        name="floor",
        type="plane",
        size="3 3 .1",
        pos="0 0 -.45",
        rgba=".27 .32 .36 1",
        contype=0,
        conaffinity=0,
    )
    node(
        world,
        "geom",
        name="bed",
        type="box",
        size=".41 .91 .035",
        pos=".36 .41 -.035",
        rgba=".75 .83 .84 1",
        friction=".27 .005 .001",
    )
    node(
        world,
        "geom",
        name="plinth",
        type="box",
        size=".085 .105 .052",
        pos="-.185 0 -.03",
        rgba=".22 .29 .33 1",
        contype=0,
        conaffinity=0,
    )
    flesh = ".76 .55 .43 1"
    linen = ".19 .39 .54 1"
    for name, pos, size, color in [
        ("head", ".42 .94 .17", ".095 .125 .10", flesh),
        ("torso", ".42 .70 .145", ".165 .22 .11", linen),
        ("far_leg", ".64 .28 .078", ".065 .40 .065", linen),
        ("far_foot", ".64 -.13 .082", ".053 .09 .064", flesh),
        ("arm_rest", ".25 .69 .115", ".043 .187 .044", flesh),
    ]:
        pos = numbers(np.fromstring(pos, sep=" ") + [0, resident_y, 0])
        node(world, "geom", name=name, type="ellipsoid", pos=pos, size=size, rgba=color)
    for name, pos, size, color in [
        ("hair", [0.42, 1.005, 0.185], [0.09, 0.069, 0.082], ".26 .19 .14 1"),
        ("nose", [0.42, 0.955, 0.258], [0.019, 0.030, 0.017], flesh),
        ("eyeL", [0.455, 0.977, 0.25], [0.009, 0.005, 0.004], ".16 .14 .13 1"),
        ("eyeR", [0.385, 0.977, 0.25], [0.009, 0.005, 0.004], ".16 .14 .13 1"),
        ("blanket", [0.49, 0.52, 0.177], [0.183, 0.216, 0.074], ".42 .53 .61 1"),
        ("hand", [0.25, 0.49, 0.083], [0.043, 0.063, 0.024], flesh),
    ]:
        pos[1] += resident_y
        node(
            world,
            "geom",
            name=name,
            type="ellipsoid",
            pos=numbers(pos),
            size=numbers(size),
            rgba=color,
            contype=0,
            conaffinity=0,
        )
    thigh = node(
        world, "body", name="upper_leg", pos=numbers([0.42, 0.575 + resident_y, 0.14])
    )
    node(
        thigh,
        "joint",
        name="hip",
        type="hinge",
        axis="1 0 0",
        range="-.55 .28",
        damping=4,
    )
    node(
        thigh,
        "geom",
        name="thigh",
        type="ellipsoid",
        pos="0 -.178 -.038",
        size=".076 .215 .096",
        mass=4.8,
        rgba=linen,
    )
    node(
        thigh,
        "geom",
        name="knee_shell",
        type="ellipsoid",
        pos="0 -.375 -.012",
        size=".06 .055 .060",
        mass=0.15,
        rgba=linen,
    )
    leg = node(thigh, "body", name="shin", pos="0 -.375 -.012")
    node(
        leg,
        "joint",
        name="knee",
        type="hinge",
        axis="1 0 0",
        range="-.72 .16",
        damping=1.6,
    )
    node(
        leg,
        "inertial",
        pos="0 -.238 -.006",
        mass=leg_mass,
        diaginertia=".065 .012 .065",
    )
    for name, offset, sz, color in [
        ("calf_prox", "0 -.095 -.008", ".053 .078 .052", linen),
        ("calf_dist", "0 -.211 -.010", ".050 .077 .055", linen),
        ("achilles", "0 -.317 -.014", ".034 .053 .036", flesh),
        ("heel", "0 -.387 -.033", ".043 .052 .049", flesh),
        ("forefoot", "0 -.414 .055", ".046 .046 .098", flesh),
    ]:
        node(leg, "geom", name=name, type="ellipsoid", pos=offset, size=sz, rgba=color)
    # Landmarks are taped to the moving model and have no mass or collision.
    for name, x, y, z, color in [
        ("prox_mark", 0.052, -0.095, -0.003, ".99 .80 .20 1"),
        ("dist_mark", 0.050, -0.211, -0.006, ".99 .80 .20 1"),
        ("heel_mark", 0.043, -0.387, -0.033, ".96 .42 .17 1"),
    ]:
        node(
            leg,
            "geom",
            name=name,
            type="box",
            pos=numbers([x, y, z]),
            size=".001 .006 .005",
            rgba=color,
            contype=0,
            conaffinity=0,
            mass=0,
        )
    sleeve_surface(assets, leg)
    # A free base and six local compressive degrees of freedom approximate a continuous foam cushion.
    cushion = node(
        world, "body", name="cushion", pos=numbers([0.165, cushion_y, 0.014])
    )
    node(cushion, "freejoint", name="free_cushion")
    node(
        cushion,
        "geom",
        name="sole",
        type="box",
        size=".082 .129 .012",
        mass=0.20,
        rgba=".13 .46 .44 1",
        friction=".27 .005 .001",
    )
    for j in range(6):
        cell = node(
            cushion, "body", name=f"foam{j}", pos=numbers([0, (j - 2.5) * 0.043, 0.038])
        )
        node(
            cell,
            "joint",
            name=f"compression{j}",
            type="slide",
            axis="0 0 1",
            range="-.031 .001",
            stiffness=stiffness,
            damping=7,
        )
        node(
            cell,
            "geom",
            name=f"pad{j}",
            type="box",
            size=".081 .0214 .027",
            mass=0.03,
            rgba=".24 .62 .57 1",
            contype=4,
            conaffinity=3,
        )
        for sign, label in ((1, "P"), (-1, "M")):
            node(
                cell,
                "geom",
                name="stripe" + label + str(j),
                type="box",
                pos=numbers([sign * 0.0811, 0, 0]),
                size=".0004 .012 .002",
                rgba=".98 .96 .77 1",
                mass=0,
                contype=0,
                conaffinity=0,
            )
            node(
                cushion,
                "geom",
                name="index" + label + str(j),
                type="box",
                pos=numbers([sign * 0.0822, (j - 2.5) * 0.043, 0.008]),
                size=".0004 .003 .003",
                rgba=".97 .69 .16 1",
                mass=0,
                contype=0,
                conaffinity=0,
            )
    # Adjacent foam nodes may compress independently; exclude base/cell self collisions.
    contact = node(root, "contact")
    for j in range(6):
        node(contact, "exclude", body1="cushion", body2=f"foam{j}")
    return mj.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def reset(model):
    data = mj.MjData(model)
    data.qpos[:6] = READY
    data.qpos[6:8] = -0.018
    data.ctrl[:8] = data.qpos[:8]
    for _ in range(600):
        mj.mj_step(model, data)
    return data
