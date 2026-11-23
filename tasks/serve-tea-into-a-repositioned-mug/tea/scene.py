"""Table, two I2RT arms, a free tea server, and a seated adult holding a mug. SI units."""

from pathlib import Path
from xml.etree.ElementTree import Element, SubElement as node, parse, tostring, indent
import numpy as np
from scipy.spatial.transform import Rotation

ASSETS = Path(__file__).resolve().parent / "assets"
TABLE = 0.75
HOME = np.array([0.365, 0.32, TABLE + 0.004])
MUG = np.array([0.428, 0.240, TABLE + 0.002])
READY = [0.0, 1.0, 1.0, -0.6, 0.0, 0.0]
GRIP = np.array([0.0, -0.061, 0.051])
LIP = np.array([0.0, 0.044, 0.089])
DOWN = Rotation.from_euler("y", 45, degrees=True).as_matrix() @ np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
)
SKIN = ".74 .49 .34 1"
BLUE = ".11 .34 .37 1"


def vec(value):
    return " ".join(f"{x:.9g}" for x in value)


def geom(parent, name, kind, group, **kw):
    return node(parent, "geom", name=name, type=kind, group=str(group), **kw)


def tube(parent, name, a, b, radius, group=3, color=SKIN, **kw):
    return geom(
        parent,
        name,
        "capsule",
        group,
        fromto=vec([*a, *b]),
        size=str(radius),
        rgba=color,
        **kw,
    )


def rig(parent, asset, acts, side, base_y):
    urdf = parse(ASSETS / "yam.urdf").getroot()
    links = {link.get("name"): link for link in urdf.findall("link")}
    base = node(parent, "body", name=side + "_base", pos=vec([0, base_y, TABLE]))
    bodies = {"base": base}
    for joint in urdf.findall("joint"):
        link = joint.find("child").get("link")
        origin = joint.find("origin")
        body = node(
            bodies[joint.find("parent").get("link")],
            "body",
            name=side + "_" + link,
            pos=origin.get("xyz"),
            euler=origin.get("rpy"),
            gravcomp="1",
        )
        bodies[link] = body
        idx = int(joint.get("name")[5:])
        limit = joint.find("limit")
        name = side + "_j" + str(idx)
        node(
            body,
            "joint",
            name=name,
            type="hinge" if idx <= 6 else "slide",
            axis=joint.find("axis").get("xyz"),
            range=limit.get("lower") + " " + limit.get("upper"),
            damping=str(0.22 if idx <= 6 else 5),
            armature=str(0.015 if idx <= 6 else 0.002),
        )
        kp, kv, eff = (
            (140, 10, 27) if idx <= 3 else (40, 3, 7) if idx <= 6 else (5000, 45, 50)
        )
        node(
            acts,
            "position",
            name=name,
            joint=name,
            kp=str(kp),
            kv=str(kv),
            forcerange=vec([-eff, eff]),
        )
        if idx > 6:
            # Transform the two closed jaw pad centres from flange to finger-link coordinates.
            off = np.array([-0.0035 if idx == 7 else 0.0035, 0.0, 0.134])
            rotation = Rotation.from_euler(
                "xyz", list(map(float, origin.get("rpy").split()))
            )
            xyz = rotation.inv().apply(off - np.fromstring(origin.get("xyz"), sep=" "))
            geom(
                body,
                name + "_pad",
                "box",
                1,
                pos=vec(xyz),
                size=".009 .0035 .015",
                rgba=".09 .11 .13 1",
                friction="1.7 .013 .003",
                contype="1",
                conaffinity="1",
                mass=".002",
            )
    for link, body in bodies.items():
        spec = links[link]
        vis = spec.find("visual")
        if side == "left":
            node(asset, "mesh", name=link, file=link + ".stl")
        inertial = spec.find("inertial")
        vals = inertial.find("inertia")
        node(
            body,
            "inertial",
            pos=inertial.find("origin").get("xyz"),
            mass=inertial.find("mass").get("value"),
            fullinertia=" ".join(
                vals.get(x) for x in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]
            ),
        )
        orig = vis.find("origin")
        visualonly = link.startswith("tip_") or link == "base"
        geom(
            body,
            side + "_" + link + "_shell",
            "mesh",
            1,
            mesh=link,
            pos=orig.get("xyz"),
            euler=orig.get("rpy"),
            rgba=".78 .80 .82 1"
            if link not in ("base", "gripper")
            else ".15 .20 .24 1",
            contype="0" if visualonly else "1",
            conaffinity="0" if visualonly else "1",
        )
    node(
        bodies["gripper"],
        "site",
        name=side + "_tcp",
        pos="0 0 .134",
        size=".001",
        rgba="0 0 0 0",
    )
    node(
        bodies["gripper"],
        "camera",
        name=side + "_cam",
        pos=".04 -.087 .016",
        xyaxes=(
            ".2549 -.546 .7981 -.4378 -.8011 -.4082"
            if side == "right"
            else "1 0 0 0 -.948 .318"
        ),
        fovy=("39" if side == "right" else "78"),
    )


def build():
    root = Element("mujoco", model="Tea service at a kitchen counter")
    node(
        root,
        "compiler",
        angle="radian",
        eulerseq="XYZ",
        meshdir="assets",
        inertiafromgeom="auto",
    )
    node(
        root,
        "option",
        timestep=str(1 / 240),
        gravity="0 0 -9.81",
        integrator="implicitfast",
        iterations="70",
        cone="elliptic",
        impratio="10",
        noslip_iterations="4",
    )
    v = node(root, "visual")
    node(v, "global", offwidth="960", offheight="540")
    node(v, "map", znear=".008", zfar="6")
    defaults = node(root, "default")
    node(defaults, "geom", friction=".8 .007 .001", solref=".008 1", condim="4")
    asset = node(root, "asset")
    node(
        asset,
        "texture",
        type="skybox",
        builtin="gradient",
        rgb1=".52 .62 .68",
        rgb2=".90 .85 .77",
        width="128",
        height="384",
    )
    world = node(root, "worldbody")
    acts = node(root, "actuator")
    node(
        world,
        "light",
        pos="0 -.5 2.5",
        dir=".1 .2 -1",
        directional="true",
        diffuse=".73 .73 .70",
        castshadow="false",
    )
    node(world, "light", pos="1 1 2", diffuse=".55 .53 .50", castshadow="false")
    # Overview faces the recipient from the robot side; wrists are separately available.
    for name, eye, look, fov in [
        ("top_cam", (-0.22, -0.54, 1.52), (0.46, 0.14, 1.01), 48),
        ("detail", (0.77, 0.56, 1.22), (0.405, 0.211, 0.873), 33),
    ]:
        z = np.array(eye) - look
        z = z / np.linalg.norm(z)
        x = np.cross([0, 0, 1], z)
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        node(
            world,
            "camera",
            name=name,
            pos=vec(eye),
            xyaxes=vec([*x, *y]),
            fovy=str(fov),
        )
    geom(world, "floor", "plane", 4, size="3 3 .05", rgba=".68 .69 .66 1")
    geom(
        world,
        "counter",
        "box",
        4,
        pos=".33 0 .726",
        size=".41 .57 .024",
        rgba=".66 .48 .32 1",
    )
    for i, (x, y) in enumerate(
        [(0.04, -0.47), (0.64, -0.47), (0.04, 0.47), (0.64, 0.47)]
    ):
        geom(
            world,
            "leg" + str(i),
            "box",
            4,
            pos=vec((x, y, 0.35)),
            size=".022 .022 .35",
            rgba=".31 .27 .24 1",
        )
    # Cabinet fronts and backsplash give the reader the scale and setting of the worktop.
    for i, yy in enumerate([-0.39, 0, 0.39]):
        geom(
            world,
            "cabinet" + str(i),
            "box",
            5,
            pos=vec((1.58, yy, 0.45)),
            size=".19 .19 .44",
            rgba=".69 .74 .70 1",
            contype="0",
            conaffinity="0",
        )
        geom(
            world,
            "cabinet_pull" + str(i),
            "box",
            5,
            pos=vec((1.378, yy, 0.78)),
            size=".012 .057 .006",
            rgba=".23 .25 .23 1",
            contype="0",
            conaffinity="0",
        )
    geom(
        world,
        "back_counter",
        "box",
        5,
        pos="1.58 0 .915",
        size=".23 .62 .024",
        rgba=".72 .68 .61 1",
        contype="0",
        conaffinity="0",
    )
    geom(
        world,
        "roomwall",
        "box",
        5,
        pos="1.84 0 1.25",
        size=".035 2 1.25",
        rgba=".88 .84 .73 1",
        contype="0",
        conaffinity="0",
    )
    for side, yy in [("left", 0.3), ("right", -0.3)]:
        rig(world, asset, acts, side, yy)
    pot = node(world, "body", name="pitcher", pos=vec(HOME))
    node(pot, "freejoint", name="pitcher_free")
    node(
        pot,
        "inertial",
        mass=".200",
        pos="-.004 0 .04",
        diaginertia=".00029 .00034 .00024",
    )
    geom(
        pot,
        "pitcher_base",
        "cylinder",
        0,
        pos="0 0 .003",
        size=".035 .003",
        rgba=".57 .76 .78 1",
    )

    # The open shell is built from separate facets to preserve the interior concavity.
    def wall(body, name, rad, height, group, rgba):
        for i in range(24):
            a = 2 * np.pi * i / 24
            geom(
                body,
                name + str(i),
                "box",
                group,
                pos=vec([rad * np.cos(a), rad * np.sin(a), height / 2]),
                size=vec([0.0016, rad * np.tan(np.pi / 24) + 0.0004, height / 2]),
                euler=vec([0, 0, a]),
                rgba=rgba,
            )

    wall(pot, "pitcher_wall", 0.035, 0.088, 0, ".40 .72 .74 .42")
    tube(
        pot,
        "handle_top",
        (0, -0.032, 0.073),
        (0, -0.061, 0.073),
        0.006,
        0,
        ".17 .25 .26 1",
    )
    geom(
        pot,
        "handle_grip",
        "box",
        0,
        pos="0 -.061 .051",
        size=".007 .009 .022",
        rgba=".17 .25 .26 1",
        friction="1.4 .013 .001",
    )
    tube(
        pot,
        "handle_bottom",
        (0, -0.061, 0.030),
        (0, -0.032, 0.026),
        0.006,
        0,
        ".17 .25 .26 1",
    )
    tube(pot, "lip_left", (0.012, 0.033, 0.085), LIP, 0.0028, 0, ".22 .49 .53 1")
    tube(pot, "lip_right", (-0.012, 0.033, 0.085), LIP, 0.0028, 0, ".22 .49 .53 1")
    # Facing the arm, the resident holds the mug from its far side (world +x).
    mug = node(world, "body", name="receiving_hand", mocap="true", pos=vec(MUG))
    wall(mug, "mug_wall", 0.035, 0.089, 2, ".54 .77 .87 .26")
    for i in range(16):
        a = 2 * np.pi * i / 16
        b = 2 * np.pi * (i + 1) / 16
        tube(
            mug,
            "serving_mark" + str(i),
            (0.037 * np.cos(a), 0.037 * np.sin(a), 0.042),
            (0.037 * np.cos(b), 0.037 * np.sin(b), 0.042),
            0.0012,
            5,
            ".04 .36 .43 1",
            contype="0",
            conaffinity="0",
        )
    geom(
        mug,
        "mug_base",
        "cylinder",
        2,
        pos="0 0 .003",
        size=".035 .003",
        rgba=".32 .56 .64 1",
    )
    for i, (a, b) in enumerate(
        [
            ((0.033, 0, 0.067), (0.058, 0, 0.064)),
            ((0.058, 0, 0.064), (0.058, 0, 0.028)),
            ((0.058, 0, 0.028), (0.033, 0, 0.026)),
        ]
    ):
        tube(mug, "cup_handle" + str(i), a, b, 0.0048, 2, ".28 .51 .59 1")
    # Four bent fingers, thumb, and palm, with separate contact surfaces.
    geom(
        mug,
        "skin_palm",
        "ellipsoid",
        3,
        pos=".083 0 .049",
        size=".027 .038 .014",
        rgba=SKIN,
    )
    for i in range(4):
        yy = -0.024 + i * 0.015
        tube(mug, "finger" + str(i), (0.081, yy, 0.049), (0.051, yy, 0.047), 0.006, 3)
        tube(
            mug, "fingertip" + str(i), (0.051, yy, 0.047), (0.050, yy, 0.034), 0.006, 3
        )
    tube(mug, "thumb", (0.084, -0.023, 0.055), (0.052, -0.006, 0.068), 0.008, 3)
    actor = node(world, "body", name="adult", pos=".84 .14 .93")
    geom(
        actor, "torso", "ellipsoid", 3, pos="0 0 .039", size=".113 .166 .226", rgba=BLUE
    )
    tube(actor, "neck", (0, 0, 0.220), (-0.009, 0, 0.29), 0.047)
    geom(
        actor,
        "head",
        "ellipsoid",
        3,
        pos="-.02 0 .366",
        size=".089 .081 .109",
        rgba=SKIN,
    )
    geom(
        actor,
        "hair",
        "ellipsoid",
        3,
        pos="-.010 0 .422",
        size=".089 .083 .063",
        rgba=".16 .13 .11 1",
    )
    geom(
        actor,
        "nose",
        "ellipsoid",
        3,
        pos="-.105 0 .366",
        size=".017 .016 .022",
        rgba=SKIN,
    )
    for i, yy in enumerate([-0.036, 0.036]):
        geom(
            actor,
            "eye" + str(i),
            "sphere",
            3,
            pos=vec([-0.104, yy, 0.391]),
            size=".006",
            rgba=".13 .10 .09 1",
        )
    for name, r, col in [("forearm", 0.032, SKIN), ("upperarm", 0.040, BLUE)]:
        arm = node(world, "body", name=name, mocap="true", pos=".70 .10 .88")
        geom(
            arm,
            "skin_" + name,
            "capsule",
            3,
            size=vec([r, 0.110 if name == "forearm" else 0.135]),
            rgba=col,
        )
    tube(actor, "other_upper", (0, -0.13, 0.16), (0.01, -0.18, -0.09), 0.044, 3, BLUE)
    tube(actor, "other_forearm", (0.01, -0.18, -0.09), (-0.11, -0.15, -0.19), 0.031)
    geom(
        actor,
        "other_hand",
        "ellipsoid",
        3,
        pos="-.12 -.15 -.19",
        size=".052 .030 .019",
        rgba=SKIN,
    )
    for i, yy in enumerate([-0.085, 0.085]):
        tube(
            actor,
            "thigh" + str(i),
            (0, yy, -0.13),
            (-0.21, yy, -0.28),
            0.064,
            3,
            ".20 .25 .29 1",
        )
        tube(
            actor,
            "shin" + str(i),
            (-0.21, yy, -0.28),
            (-0.22, yy, -0.54),
            0.045,
            3,
            ".20 .25 .29 1",
        )
    geom(
        world,
        "chair",
        "box",
        4,
        pos=".92 .14 .71",
        size=".16 .19 .025",
        rgba=".28 .24 .22 1",
    )
    for i, (x, y) in enumerate(
        [(0.82, 0.00), (0.82, 0.28), (1.02, 0.00), (1.02, 0.28)]
    ):
        geom(
            world,
            "chair_leg" + str(i),
            "box",
            5,
            pos=vec((x, y, 0.345)),
            size=".018 .018 .345",
            rgba=".28 .24 .22 1",
            contype="0",
            conaffinity="0",
        )
    for entry in world.iter("geom"):
        if entry.get("group") in ("2", "3"):
            entry.set("contype", "2")
            entry.set("conaffinity", "5")
        if entry.get("name", "").endswith(("tip_left_shell", "tip_right_shell")):
            entry.set("contype", "4")
            entry.set("conaffinity", "2")
    indent(root)
    return tostring(root, encoding="unicode")


if __name__ == "__main__":
    print(build())
