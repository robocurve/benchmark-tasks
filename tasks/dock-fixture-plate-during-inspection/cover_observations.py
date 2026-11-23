"""Convert world-frame PhysX transforms and contact reports to optical and sensor readings."""

import math
from cover_geometry import (
    CARRIAGE,
    WORKER_BODIES,
    SHOULDER,
    UPPER_END,
    LOWER_END,
    CUFF_SOCKET,
    GAUGE_TIP,
    ORIGIN,
    LENGTH,
    WIDTH,
    SHEET,
    PICK_X,
    REST_Z,
    BOSS_HEIGHT,
    PIN_X,
    PIN_YS,
    CHAMFER,
    PIN_SHANK,
    PIN_HEIGHT,
    BORE_RADIUS,
    SIDES_COUNT,
    pin_profile,
    PADS,
    PAD_MASS,
    PAD_Z,
    K,
    SLOPE,
    LAND_X,
)


def rotate(q, v):
    x, y, z, w = q
    a, b, c = v
    i, j, k = 2 * (y * c - z * b), 2 * (z * a - x * c), 2 * (x * b - y * a)
    return (
        a + w * i + y * k - z * j,
        b + w * j + z * i - x * k,
        c + w * k + x * j - y * i,
    )


def pt(p, v):
    return tuple(a + b for a, b in zip(p[:3], rotate(p[3:], v)))


# Outward-rounded local AABBs of all convex collision meshes of the stock YAM links.
DISTAL_BOUNDS = {
    "link5": ((0, -0.0286, -0.0357), (0.069, 0.0285, 0.0335)),
    "gripper": ((-0.0511, -0.0325, -0.0105), (0.051, 0.0326, 0.0635)),
    "tip_left": ((-0.0101, -0.0683, -0.0902), (0.058, 0.0033, 0.0037)),
    "tip_right": ((-0.058, -0.0033, -0.0902), (0.0101, 0.0683, 0.0037)),
}
LOCAL_CORNERS = {
    n: [
        (x, y, z)
        for x in (lo[0], hi[0])
        for y in (lo[1], hi[1])
        for z in (lo[2], hi[2])
    ]
    for n, (lo, hi) in DISTAL_BOUNDS.items()
}


def clearance(p, link, low, high):
    corners = [pt(p, c) for c in LOCAL_CORNERS[link]]
    return math.sqrt(
        sum(
            max(lo - max(xs), min(xs) - hi, 0) ** 2
            for xs, lo, hi in zip(zip(*corners), low, high)
        )
    )


# A shank counts as registered only inside all bore faces at both ends of
# the required insertion. This retains the anisotropy of the relieved pin.
BORE_FACE_NORMALS = [
    (
        math.cos((i + 0.5) * 2 * math.pi / SIDES_COUNT),
        math.sin((i + 0.5) * 2 * math.pi / SIDES_COUNT),
    )
    for i in range(SIDES_COUNT)
]
BORE_APOTHEM = BORE_RADIUS * math.cos(math.pi / SIDES_COUNT)
PIN_PROFILES = [pin_profile(i) for i in range(2)]


def registration(cover):
    q = cover[3:]
    inverse = (-q[0], -q[1], -q[2], q[3])
    axis = rotate(q, (0, 0, 1))
    margins, depths = [], []
    for y, outline in zip(PIN_YS, PIN_PROFILES):
        center = pt(cover, (PIN_X, y, CHAMFER))
        px, py = ORIGIN[0] + LAND_X + PIN_X, ORIGIN[1] + y
        if axis[2] < 0.8:
            return [-1.0, -1.0], [-1.0, -1.0]
        plane_z = (
            center[2]
            - ((px - center[0]) * axis[0] + (py - center[1]) * axis[1]) / axis[2]
        )
        peak_z = ORIGIN[2] + REST_Z + PIN_SHANK
        depth = peak_z - plane_z
        depths.append(depth)
        # Conservatively project the full shank profile into the tilted bore.
        vertices = [
            rotate(inverse, (px + dx - cover[0], py + dy - cover[1], z - cover[2]))[:2]
            for dx, dy in outline
            for z in (plane_z, min(plane_z + (BOSS_HEIGHT - CHAMFER) / axis[2], peak_z))
        ]
        margins.append(
            min(
                BORE_APOTHEM
                - max(nx * (vx - PIN_X) + ny * (vy - y) for vx, vy in vertices)
                for nx, ny in BORE_FACE_NORMALS
            )
        )
    return depths, margins


def measure(raw):
    poses = raw["poses"]
    for p in (*poses.values(), *raw["distal"].values()):
        if (
            len(p) != 7
            or not all(map(math.isfinite, p))
            or abs(sum(x * x for x in p[3:]) - 1) > 0.003
        ):
            raise ValueError("Invalid actor transform")
    if math.dist(poses["frame"][:3], ORIGIN) > 0.0006:
        raise ValueError("Station moved")
    cover, car = poses["cover"], poses["carriage"]
    start_x = ORIGIN[0] + CARRIAGE[0]
    travel = (car[0] - start_x) / math.cos(SLOPE)
    deflections = [
        car[2] + PAD_Z - CARRIAGE[2] - poses[n][2] - PAD_MASS * 9.81 / K for n in PADS
    ]
    edge = ORIGIN[0] + LAND_X + LENGTH / 2
    tip = min(
        pt(poses[n], (x, y, z))[0]
        for n in PADS
        for x in (-0.036, 0.036)
        for y in (-0.018, 0.018)
        for z in (-0.0085, 0.0085)
    )
    axis_x = rotate(cover[3:], (1, 0, 0))
    axis_y = rotate(cover[3:], (0, 1, 0))
    tilt = max(
        abs(math.degrees(math.asin(max(-1, min(1, a[2]))))) for a in (axis_x, axis_y)
    )
    if rotate(cover[3:], (0, 0, 1))[2] < 0:
        tilt = 180 - tilt
    rests = [0.0, 0.0]
    stage_loads = [0.0, 0.0]
    skin = {}
    offrest = 0.0
    cover_fingers = [0.0, 0.0]
    handles = {}
    agent_support = False
    foreign_support = False
    for a, b, sa, sb, f in raw["contacts"]:
        if not math.isfinite(f) or f < 0:
            raise ValueError("Invalid contact")
        for obj, other, shape, other_shape in ((a, b, sa, sb), (b, a, sb, sa)):
            if obj == "cover":
                if other in PADS:
                    cover_fingers[PADS.index(other)] += f
                if other == "frame" and other_shape in ("restLeft", "restRight"):
                    rests[int(other_shape == "restRight")] += f
                elif other == "frame" and other_shape in ("stageLeft", "stageRight"):
                    stage_loads[int(other_shape == "stageRight")] += f
                    foreign_support |= f > 0.01
                elif other == "frame" and other_shape.startswith(
                    ("pinShank", "pinNose")
                ):
                    pass  # Intended guide surfaces. Actual receiver loads are checked separately.
                elif other == "environment" or other == "frame":
                    # The starting trough rim is a legitimate bearing during pickup.
                    if other_shape != "rearBearer":
                        offrest += f
                    foreign_support |= f > 0.01
                if "/" in other:
                    agent_support |= f > 0.001
                    if shape.startswith("grip") and f > 0.001:
                        handles[other + "/" + shape] = f
                if other in (*PADS, "carriage", *WORKER_BODIES):
                    foreign_support |= f > 0.01
            if ("/" in obj and other in (*PADS, "carriage", *WORKER_BODIES)) or (
                obj == "cover" and other in ("carriage", *WORKER_BODIES)
            ):
                # Accumulate compressive forces per surface from every robot link;
                # opposite jaws cannot cancel by vector addition.
                if other == "carriage" and other_shape in ("gaugeLeaf", "gaugeGrip"):
                    continue
                surface = other + "/" + other_shape
                skin[surface] = skin.get(surface, 0.0) + f
    ox, oy, oz = ORIGIN
    box_lo = (ox + PICK_X - LENGTH / 2, oy - WIDTH / 2, oz + 0.035)
    box_hi = (ox + LENGTH / 2, oy + WIDTH / 2, oz + 0.16)
    if set(raw["distal"]) != {
        side + "/" + n for side in ("left", "right") for n in DISTAL_BOUNDS
    }:
        raise ValueError("Incomplete distal link transforms")
    clear = min(
        clearance(p, n.split("/")[1], box_lo, box_hi) for n, p in raw["distal"].items()
    )
    v = raw["twist"]
    if len(v) != 6 or not all(map(math.isfinite, v)):
        raise ValueError("Invalid cover velocity")
    delta = [a - b for a, b in zip(poses[PADS[0]][:3], cover[:3])]
    q = cover[3:]
    under = rotate((-q[0], -q[1], -q[2], q[3]), delta)[:2]
    upper, lower = poses["upperArm"], poses["forearmLink"]
    joint_gaps = [
        math.dist(pt(poses["worker"], SHOULDER), upper[:3]),
        math.dist(pt(upper, UPPER_END), lower[:3]),
        math.dist(pt(lower, LOWER_END), pt(car, CUFF_SOCKET)),
    ]
    gauge_gap = pt(car, GAUGE_TIP)[0] - edge
    limb_gap = (
        min(
            tip,
            car[0] - 0.029,
            upper[0] - 0.047,
            lower[0] - 0.047,
            pt(lower, LOWER_END)[0] - 0.036,
        )
        - edge
    )
    depths, margins = registration(cover)
    return dict(
        drive_travel=poses["stageDrive"][0] - start_x,
        pin_depth=depths,
        pin_margin=margins,
        stage_loads=stage_loads,
        limb_gap=limb_gap,
        stage_error=math.dist(cover[:2], (ORIGIN[0] + PICK_X, ORIGIN[1])),
        joint_gap=max(joint_gaps),
        elbow=list(lower[:3]),
        gauge_gap=gauge_gap,
        under=under,
        tick=raw["tick"],
        travel=travel,
        tip_gap=tip - edge,
        stroke=deflections,
        finger_load=cover_fingers,
        skin_N=sum(skin.values()),
        rests=rests,
        contact=agent_support,
        other_support=foreign_support,
        offrest=offrest,
        tilt=tilt,
        clearance=clear,
        bottom=cover[2] - oz,
        xy_error=math.dist(cover[:2], (ORIGIN[0] + LAND_X, ORIGIN[1])),
        speed=math.hypot(*v[:3]),
        spin=math.hypot(*v[3:]),
        handles=handles,
    )
