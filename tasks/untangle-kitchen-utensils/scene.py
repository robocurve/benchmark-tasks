"""Tabletop utensils in metres. Local +x runs along each tool toward its working end."""

import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import station

from station_model import add_arms, numbers, quat

TABLE = station.TABLE_TOP
NAMES = ("whisk", "spoon", "spatula")
# x, y, height above table, heading, and downward pitch of the positive axis.
POSES = {
    "whisk": (0.222, 0.034, 0.029, 0, -7),
    "spoon": (0.310, 0.034, 0.034, 90, 8),
    "spatula": (0.283, -0.020, 0.044, 32, -10),
}
SLOTS = (-0.111, 0.0, 0.111)
TRAY_X = 0.554
TRAY_LENGTH = 0.322
PITCH = 0.111
FLOOR = TABLE + 0.007
RIM = FLOOR + 0.022
WIRE_RADIUS = 0.00145
# Axial and radial distances of one side of a balloon loop from its ferrule.
PROFILE = np.array(
    [
        (0, 0.004),
        (0.024, 0.011),
        (0.047, 0.023),
        (0.072, 0.034),
        (0.094, 0.040),
        (0.112, 0.039),
        (0.127, 0.033),
        (0.139, 0.024),
        (0.146, 0.013),
        (0.149, 0.0),
    ]
)
BEND = 4


def pose(name):
    x, y, h, yaw, pitch = POSES[name]
    return np.array((x, y, TABLE + h)), Rotation.from_euler("ZY", (yaw, pitch), degrees=True).as_matrix()


def element(parent, tag, **attrs):
    return ET.SubElement(
        parent, tag, {k: v if isinstance(v, str) else numbers(v) for k, v in attrs.items()}
    )


def surface(body, name, material, **attrs):
    kind = name.split("_")[0]
    bits = {"whisk": (2, 13), "spoon": (4, 11), "spatula": (8, 7)}
    ct, ca = bits.get(kind, (1, 1))
    return element(body, "geom", name=name, material=material, contype=ct, conaffinity=ca, **attrs)


def make_body(world, name):
    pos, rot = pose(name)
    body = element(world, "body", name=name, pos=pos, quat=quat(rot))
    element(body, "freejoint", name=name)
    return body


def oval_sections(asset, body, name, material, sections, mass):
    """Rounded compound profile. Each station gives x, half width, half height, and z."""
    theta = np.arange(12) * np.pi / 6
    for i, (a, b) in enumerate(zip(sections, sections[1:])):
        vertices = [(x, w * np.cos(t), z + h * np.sin(t)) for x, w, h, z in (a, b) for t in theta]
        part = f"{name}_{i}"
        element(asset, "mesh", name=part, vertex=np.ravel(vertices))
        surface(
            body,
            part,
            material,
            type="mesh",
            mesh=part,
            mass=mass * (b[0] - a[0]) / (sections[-1][0] - sections[0][0]),
        )


def make_whisk(world, equality):
    body = make_body(world, "whisk")
    surface(
        body,
        "whisk_handle",
        "steel",
        type="capsule",
        fromto=(-0.138, 0, 0, -0.011, 0, 0),
        size=0.0105,
        mass=0.059,
    )
    surface(
        body,
        "whisk_ferrule",
        "collar",
        type="cylinder",
        fromto=(-0.015, 0, 0, 0.001, 0, 0),
        size=0.011,
        mass=0.010,
    )
    for loop in range(4):
        ends = []
        for side in range(2):
            angle = np.deg2rad(22.5 + 45 * loop + 180 * side)
            radial = np.array([0.0, np.cos(angle), np.sin(angle)])
            tangent = np.cross([1, 0, 0], radial)
            points = PROFILE[:, :1] * np.array([[1, 0, 0]]) + PROFILE[:, 1:] * radial
            # Stagger the crowns slightly, as nested loops do at the nose.
            points[:, 0] *= 1 - 0.008 * loop
            prefix = f"whisk_{loop}_{side}"
            inner = element(body, "body", name=prefix, pos=points[0])
            element(
                inner,
                "joint",
                name=prefix + "_root",
                axis=tangent,
                range=(-0.31, 0.31),
                stiffness=0.22,
                damping=0.0025,
                armature=0.000012,
            )
            outer = element(inner, "body", name=prefix + "_outer", pos=points[BEND] - points[0])
            element(
                outer,
                "joint",
                name=prefix + "_bend",
                axis=tangent,
                range=(-0.50, 0.50),
                stiffness=0.10,
                damping=0.0015,
                armature=0.000008,
            )
            for i, (a, b) in enumerate(zip(points, points[1:])):
                node, origin = (inner, points[0]) if i < BEND else (outer, points[BEND])
                surface(
                    node,
                    f"{prefix}_wire_{i}",
                    "wire",
                    type="capsule",
                    fromto=np.r_[a - origin, b - origin],
                    size=WIRE_RADIUS,
                    density=7500,
                    friction=".32 .001 .0001",
                )
            for i, point in enumerate(points):
                node, origin = (inner, points[0]) if i < BEND else (outer, points[BEND])
                element(node, "site", name=f"{prefix}_p{i}", pos=point - origin, size=0.0003)
            tip = prefix + "_tip"
            element(outer, "site", name=tip, pos=points[-1] - points[BEND], size=0.001)
            ends.append(tip)
        element(
            equality,
            "connect",
            name=f"crown_{loop}",
            site1=ends[0],
            site2=ends[1],
            solref=".003 1",
            solimp=".97 .999 .0002",
        )


def make_spoon(world, asset):
    body = make_body(world, "spoon")
    oval_sections(
        asset,
        body,
        "spoon_shaft",
        "beech",
        [
            (-0.077, 0.0055, 0.0045, 0.006),
            (-0.071, 0.0105, 0.0065, 0.005),
            (-0.053, 0.0095, 0.0055, 0.001),
            (-0.039, 0.007, 0.005, 0),
            (0.074, 0.0075, 0.0055, 0),
            (0.121, 0.007, 0.005, 0),
            (0.144, 0.009, 0.005, 0),
        ],
        0.032,
    )
    # A dished bowl with a raised rim. Convex strips preserve the hollow face.
    rows = [
        (0.139, 0.008, 0.002),
        (0.153, 0.017, 0),
        (0.177, 0.026, -0.003),
        (0.194, 0.024, -0.002),
        (0.211, 0.014, 0.003),
        (0.216, 0.003, 0.005),
    ]
    for i, (a, b) in enumerate(zip(rows, rows[1:])):
        for k, (u, v) in enumerate(zip((-1, -0.5, 0, 0.5), (-0.5, 0, 0.5, 1))):
            vertices = [
                (x, w * t, z + 0.007 * (w * t / 0.026) ** 2 + h)
                for x, w, z in (a, b)
                for t in (u, v)
                for h in (-0.002, 0.002)
            ]
            part = f"spoon_bowl_{i}_{k}"
            element(asset, "mesh", name=part, vertex=np.ravel(vertices))
            surface(body, part, "beech", type="mesh", mesh=part, mass=0.0011, group=3)
    vertices, faces = [], []
    widths = np.linspace(-1, 1, 13)
    for x, w, z in rows:
        vertices.extend(
            (x, w * t, z + 0.007 * (w * t / 0.026) ** 2 + h) for h in (-0.002, 0.002) for t in widths
        )
    stride = len(widths)

    def patch(a, b, c, d):
        faces.extend(((a, b, c), (a, c, d)))

    for i in range(len(rows) - 1):
        left = i * 2 * stride
        right = left + 2 * stride
        for j in range(stride - 1):
            patch(left + j, left + j + 1, right + j + 1, right + j)
            patch(left + stride + j, right + stride + j, right + stride + j + 1, left + stride + j + 1)
        patch(left, right, right + stride, left + stride)
        patch(left + stride - 1, left + 2 * stride - 1, right + 2 * stride - 1, right + stride - 1)
    for j in range(stride - 1):
        patch(j, stride + j, stride + j + 1, j + 1)
        last = (len(rows) - 1) * 2 * stride
        patch(last + j, last + j + 1, last + stride + j + 1, last + stride + j)
    element(asset, "mesh", name="dish_skin", vertex=np.ravel(vertices), face=np.ravel(faces))
    element(
        body,
        "geom",
        name="dish_skin",
        type="mesh",
        mesh="dish_skin",
        material="beech",
        contype=0,
        conaffinity=0,
        mass=0,
    )


def make_spatula(world, asset):
    body = make_body(world, "spatula")
    oval_sections(
        asset,
        body,
        "spatula_handle",
        "grip",
        [
            (-0.213, 0.007, 0.0055, 0),
            (-0.207, 0.012, 0.0085, 0),
            (-0.154, 0.012, 0.0085, 0),
            (-0.079, 0.010, 0.0075, 0),
            (-0.024, 0.007, 0.006, 0),
            (0.002, 0.006, 0.005, 0),
        ],
        0.057,
    )
    # Silicone molded around a short core, with three elastic bending spans.
    xs, widths, half = (
        (0.0, 0.023, 0.050, 0.075),
        (0.009, 0.023, 0.022, 0.019),
        (0.005, 0.0038, 0.0030, 0.0018),
    )
    for i, (x0, x1) in enumerate(zip(xs, xs[1:])):
        part = f"spatula_blade_{i}"
        limb = element(body, "body", name=part, pos=(0 if i == 0 else xs[i] - xs[i - 1], 0, 0))
        element(
            limb,
            "joint",
            name=part + "_bend",
            axis=(0, 1, 0),
            range=(-0.55, 0.55),
            stiffness=(0.30, 0.12, 0.055)[i],
            damping=0.003,
            armature=0.000018,
        )
        vertices = [
            (x, side * w * fac, z * h)
            for x, w, h in ((0.0, widths[i], half[i]), (x1 - x0, widths[i + 1], half[i + 1]))
            for side in (-1, 1)
            for fac, z in ((1, -0.6), (0.90, -1), (0.90, 1), (1, 0.6))
        ]
        element(asset, "mesh", name=part, vertex=np.ravel(vertices))
        surface(
            limb,
            part,
            "silicone",
            type="mesh",
            mesh=part,
            mass=(0.007, 0.008, 0.005)[i],
            friction=".62 .002 .0001",
        )
        body = limb


def look_at(world, name, eye, target, fovy):
    direction = np.subtract(target, eye)
    direction /= np.linalg.norm(direction)
    right = np.cross(direction, [0, 0, 1])
    right /= np.linalg.norm(right)
    element(
        world, "camera", name=name, pos=eye, xyaxes=np.r_[right, np.cross(right, direction)], fovy=fovy
    )


def build():
    root = ET.fromstring("""<mujoco model="Kitchen utensil drawer">
      <compiler angle="radian" autolimits="true"/>
      <option timestep=".002" integrator="implicitfast" cone="elliptic" iterations="65"/>
      <visual><global offwidth="960" offheight="640"/><quality shadowsize="2048"/>
      <headlight ambient=".49 .49 .49" diffuse=".36 .36 .36" specular=".15 .15 .15"/>
      <map znear=".002"/></visual>
      <default><geom friction=".55 .003 .0001" solref=".005 1" solimp=".96 .995 .0005"/></default>
      <asset>
        <material name="steel" rgba=".48 .54 .57 1" specular=".75" shininess=".70"/>
        <material name="wire" rgba=".48 .54 .60 1" specular=".80" shininess=".72"/>
        <material name="collar" rgba=".11 .15 .17 1" specular=".52"/>
        <material name="beech" rgba=".63 .41 .23 1" specular=".19"/>
        <material name="grip" rgba=".14 .33 .38 1" specular=".3"/>
        <material name="silicone" rgba=".78 .24 .12 1" specular=".2"/>
        <material name="tray" rgba=".43 .60 .56 1"/><material name="rim" rgba=".23 .43 .39 1"/>
        <texture name="sky" type="skybox" builtin="gradient" rgb1=".70 .80 .86" rgb2=".94 .94 .92" width="128" height="768"/>
      </asset><worldbody/><contact/><equality/><actuator/></mujoco>""")
    world, assets = root.find("worldbody"), root.find("asset")
    element(
        world,
        "geom",
        name="table",
        type="box",
        pos=station.TABLE_CENTER,
        size=np.array(station.TABLE_SIZE) / 2,
        rgba=(0.75, 0.68, 0.55, 1),
    )
    element(
        world,
        "geom",
        name="floor",
        type="plane",
        pos=(0, 0, station.FLOOR_Z),
        size=(0, 0, 0.05),
        rgba=(0.65, 0.67, 0.66, 1),
    )
    length = TRAY_LENGTH / 2
    surface(
        world,
        "tray_floor",
        "tray",
        type="box",
        pos=(TRAY_X, 0, FLOOR - 0.0035),
        size=(length, 1.5 * PITCH + 0.003, 0.0035),
    )
    for i in range(4):
        surface(
            world,
            f"tray_rail_{i}",
            "rim",
            type="box",
            pos=(TRAY_X, (i - 1.5) * PITCH, (RIM + FLOOR) / 2),
            size=(length, 0.003, 0.011),
        )
    for i, sign in enumerate((-1, 1)):
        surface(
            world,
            f"tray_end_{i}",
            "rim",
            type="box",
            pos=(TRAY_X + sign * (length - 0.003), 0, (RIM + FLOOR) / 2),
            size=(0.003, 1.5 * PITCH, 0.011),
        )
    make_whisk(world, root.find("equality"))
    make_spoon(world, assets)
    make_spatula(world, assets)
    element(
        world,
        "light",
        pos=(-0.28, -0.7, 2.0),
        dir=(0.35, 0.45, -1),
        directional="true",
        diffuse=(0.49, 0.48, 0.45),
        specular=(0.18, 0.18, 0.17),
        castshadow="false",
    )
    look_at(world, "setup_view", (0.36, -0.42, TABLE + 0.72), (0.397, 0.004, TABLE + 0.014), 39)
    look_at(world, "tangle_view", (0.52, -0.22, TABLE + 0.21), (0.305, 0.027, TABLE + 0.037), 29)
    add_arms(root)
    return ET.tostring(root, encoding="unicode")
