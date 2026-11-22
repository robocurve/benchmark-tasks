"""Convert raw PhysX actor poses, twists, contacts and joint aperture to fixture coordinates."""

import math

import numpy as np
from valve_layout import ORIGIN, PIN, SIDES, TRAY


def rotate(q, v):
    u = np.array(q[:3])
    v = np.asarray(v)
    return v + 2 * np.cross(u, np.cross(u, v) + q[3] * v)


def point(p, v):
    return np.array(p[:3]) + rotate(p[3:], v)


def measure(raw, prev):
    origin = np.asarray(ORIGIN)
    poses = raw["poses"]
    for p in poses.values():
        if (
            len(p) != 7
            or not np.all(np.isfinite(p))
            or abs(np.linalg.norm(p[3:]) - 1) > 0.001
        ):
            raise ValueError("Bad actor transform")
    if np.linalg.norm(np.array(poses["frame"][:3]) - origin) > 0.0005:
        raise ValueError("Fixture frame moved")
    r = poses["rotor"]
    pp = poses["pin"]
    x, y, z = map(float, np.array(pp[:3]) - origin)
    axis = rotate(r[3:], (1, 0, 0))
    angle = 45 + math.degrees(math.atan2(axis[1], axis[0]))
    tilt = math.degrees(math.acos(float(np.clip(rotate(pp[3:], (0, 0, 1))[2], -1, 1))))
    # Both shank ends and the four handle corners must project inside the tray.
    pinpts = [
        point(pp, v) - origin
        for v in [(0, 0, 0), (0, 0, 0.08)]
        + [
            (i * 0.0135, j * 0.009, 0.082 + k * 0.0105)
            for i in (-1, 1)
            for j in (-1, 1)
            for k in (-1, 1)
        ]
    ]
    inside = all(abs(v[0] - TRAY[0]) < 0.048 and abs(v[1]) < 0.055 for v in pinpts)
    forces = {k: 0.0 for k in ("pin", "human", "collision")}
    touching = False
    tray = False
    foreign_support = False
    for a, b, shape_a, shape_b, force in raw["contacts"]:
        if not math.isfinite(force) or force < 0:
            raise ValueError("Invalid contact force")
        if {a, b} == {"pin", "rotor"}:
            forces["pin"] += force
        if {a, b} == {"wrist", "rotor"}:
            forces["human"] += force
        for limb, obj, shape in ((a, b, shape_b), (b, a, shape_a)):
            if limb == "pin" and obj == "frame" and shape.startswith("tray"):
                tray = True
            if (
                limb == "pin"
                and obj not in ("pin", "rotor")
                and "/" not in obj
                and not (
                    obj == "frame"
                    and (
                        shape.startswith(("tray", "guide", "collar")) or shape == "seat"
                    )
                )
            ):
                foreign_support = True
            fields = limb.split("/")
            if len(fields) != 2 or fields[0] not in SIDES:
                continue
            side, link = fields
            if obj.split("/")[0] in SIDES and obj.split("/")[0] != side:
                if side == "left":
                    forces["collision"] += force
            if obj in poses:
                touching = True
                if obj == "wrist" or (
                    obj == "frame"
                    and (
                        shape.startswith(("leg", "boot"))
                        or "Sleeve" in shape
                        or "Glove" in shape
                        or shape
                        in {
                            "vest",
                            "head",
                            "hat",
                            "hatBrim",
                            "otherForearm",
                            "elbow",
                            "coveralls",
                        }
                    )
                ):
                    forces["human"] += force
                intended = (obj == "pin" and shape == "grip") or (
                    obj == "rotor" and shape.startswith("knob")
                )
                if not intended or link not in ("tip_left", "tip_right"):
                    forces["collision"] += force
                if intended and link in ("tip_left", "tip_right") and force > 0.01:
                    prev[side, obj, shape, link] = raw["tick"]

    def held(side, obj, shape):
        # Both pads on one handle in the last 25 ms; disallow open grippers.
        return (
            raw["jaw"][SIDES.index(side)] < 0.034
            and min(
                prev.get((side, obj, shape, t), -99) for t in ("tip_left", "tip_right")
            )
            >= raw["tick"] - 5
        )

    pin_grip = [held(h, "pin", "grip") for h in SIDES]
    off_tray = abs(x - TRAY[0]) > 0.052 or abs(y - TRAY[1]) > 0.0595
    clear = True
    for pose in raw["hands"]:
        # A bounding capsule covers the distal hand and pads including their open span.
        for offset in (0.0, 0.065, 0.135, 0.153):
            a = point(pose, (0, 0, offset)) - origin
            if -0.234 < a[0] < 0.216 and -0.155 < a[1] < 0.155 and -0.04 < a[2] < 0.206:
                clear = False
    v = np.asarray(raw["pin_twist"])
    wrist_mm = 1000 * (-0.027 - (poses["wrist"][1] - origin[1]))
    return dict(
        tick=raw["tick"],
        angle=angle,
        speed=math.degrees(raw["rotor_spin"]),
        lift=z - PIN[2],
        pin_tilt=tilt,
        pin_offset=math.hypot(x - PIN[0], y - PIN[1]),
        pin_force=forces["pin"],
        human_force=forces["human"],
        collision=forces["collision"],
        wrist_mm=wrist_mm,
        memory_mm=1000 * (-0.010 - (poses["memory"][1] - origin[1])),
        pin_speed=float(np.linalg.norm(v[:3])),
        pin_spin=float(np.linalg.norm(v[3:])),
        on_tray=inside and tray,
        pin_lost=(
            z < -0.012
            or (foreign_support and not inside)
            or (
                z < 0.037
                and off_tray
                and not tray
                and not any(pin_grip)
                and math.hypot(x - PIN[0], y - PIN[1]) > 0.022
            )
        ),
        jaw=raw["jaw"],
        touching=touching,
        clear=clear,
        wheel_grip=[
            any(held(h, "rotor", "knob" + str(i)) for i in (0, 1)) for h in SIDES
        ],
        pin_grip=pin_grip,
    )
