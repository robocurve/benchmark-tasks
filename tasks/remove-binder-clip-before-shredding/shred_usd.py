"""The shredder, the folded invoices, the binder clip and the clip dish: sizes and USD files.

Lengths are metres. The shredder and the dish are fixed bodies written in station coordinates: their origin is on the
table top under the midpoint between the arm bases (x forward, y left, z up). The invoices' origin is their centre, and the clip's origin the middle of the paper edge it grips. build()
writes the files into build/ (not committed: they are generated each time the task file loads).

The clip is one articulation with a free base: its body (spine, the fixed jaw and the handles) and a moving jaw on a
prismatic joint across the paper, driven toward the fixed jaw with a force limit (the spring). The task opens the
jaw while a gripper pinches the handles, as squeezing a real clip's handles opens it.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import usdkit as U

HERE = Path(__file__).resolve().parent
BUILD = HERE / "build"

# -- shredder (station frame, origin on the table top) -----------------------------------------------------------------
SHRED_X = (0.25, 0.41)
SHRED_Y = (-0.43, -0.13)
SHRED_TOP = 0.12
BIN_TOP = 0.04
SLOT_C = (0.32, -0.28)
SLOT_L, SLOT_W = 0.23, 0.016
SLOT_X = (SLOT_C[0] - SLOT_W / 2, SLOT_C[0] + SLOT_W / 2)
SLOT_Y = (SLOT_C[1] - SLOT_L / 2, SLOT_C[1] + SLOT_L / 2)
LIGHT_R = 0.0075
GREEN_XY = (0.272, -0.154)
RED_XY = (0.272, -0.176)
SWITCH_XY = (0.272, -0.402)

# -- invoices, clip, dish ---------------------------------------------------------------------------------------
STACK = (0.0035, 0.21, 0.099)          # thick (x), long (y), tall (z)
STACK_MASS = 0.015
CLIP_L = 0.041
SPINE_HW = 0.0095                      # half the spine's width across the paper
SPINE_Z = (0.0025, 0.0040)             # above the paper edge
LIP_Z = (-0.018, -0.012)               # the lips grip the paper's faces here
HANDLE_BOX = ((-0.005, 0.005), (-0.0135, 0.0135), (0.0065, 0.026))   # pinch region of the folded-up handles
HANDLE_TOP = 0.027
CLIP_MASS, JAW_MASS = 0.010, 0.002
JAW_CLOSED, JAW_OPEN = 0.004, -0.004   # drive targets (m, + toward the fixed jaw)
DISH_C = (0.18, 0.25)
DISH = (0.09, 0.09, 0.025)
DISH_WALL = 0.005


def _lin(c):
    c = np.asarray(c, float)
    return tuple(float(v) for v in np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4))


LOOKS = {
    "abs_grey": U.Look(color=_lin((0.80, 0.81, 0.80)), roughness=0.45),
    "abs_trim": U.Look(color=_lin((0.62, 0.63, 0.63)), roughness=0.5),
    "abs_black": U.Look(color=_lin((0.06, 0.06, 0.065)), roughness=0.45),
    "throat": U.Look(color=_lin((0.015, 0.015, 0.015)), roughness=0.9),
    "steel_dark": U.Look(color=_lin((0.30, 0.30, 0.31)), metallic=0.9, roughness=0.4),
    "smoke": U.Look(color=_lin((0.10, 0.10, 0.11)), roughness=0.08, opacity=0.45),
    "led_green": U.Look(color=_lin((0.25, 0.95, 0.35)), roughness=0.3, emissive=(0.5, 5.0, 0.8)),
    "led_red_off": U.Look(color=_lin((0.30, 0.06, 0.06)), roughness=0.25),
    "led_red_on": U.Look(color=_lin((1.0, 0.15, 0.10)), roughness=0.3, emissive=(7.0, 0.4, 0.2)),
    "slider": U.Look(color=_lin((0.92, 0.92, 0.90)), roughness=0.5),
    "paper": U.Look(color=_lin((0.95, 0.95, 0.93)), roughness=0.85),
    "invoice": U.Look(color=(1.0, 1.0, 1.0), roughness=0.85, texture="invoice.png", tile=0.21),
    "shreds": U.Look(color=_lin((0.93, 0.93, 0.91)), roughness=0.9),
    "oak": U.Look(color=(1.0, 1.0, 1.0), roughness=0.55, texture="oak.jpg", tile=0.10),
    "clip_black": U.Look(color=_lin((0.035, 0.035, 0.04)), metallic=0.5, roughness=0.3),
    "wire": U.Look(color=_lin((0.80, 0.80, 0.82)), metallic=1.0, roughness=0.25),
    "ceramic": U.Look(color=_lin((0.93, 0.93, 0.91)), roughness=0.15),
}


def _box(lo, hi, bevel=0.0, round_xy=0.0) -> U.Mesh:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return U.box(hi - lo, (lo + hi) / 2, bevel=bevel, round_xy=round_xy)


def _quat_wxyz_y(deg: float):
    a = math.radians(deg) / 2
    return (math.cos(a), 0.0, math.sin(a), 0.0)


def _collider(name, lo, hi, root, material="PhysicsMaterial", indent="    ", offset=0.0003, rot_y_deg=0.0) -> str:
    """A box collider (not rendered), from corner lo to corner hi, optionally turned about y through its centre."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, s = (lo + hi) / 2, hi - lo
    w, x, y, z = _quat_wxyz_y(rot_y_deg)
    return f"""
{indent}def Cube "{name}" (
{indent}    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]
{indent})
{indent}{{
{indent}    rel material:binding:physics = </{root}/{material}>
{indent}    uniform token purpose = "guide"
{indent}    double size = 1
{indent}    float physxCollision:contactOffset = {offset}
{indent}    float physxCollision:restOffset = 0
{indent}    double3 xformOp:translate = ({c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f})
{indent}    quatf xformOp:orient = ({w:.6f}, {x:.6f}, {y:.6f}, {z:.6f})
{indent}    float3 xformOp:scale = ({s[0]:.6f}, {s[1]:.6f}, {s[2]:.6f})
{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
{indent}}}"""


def _meshes(parts, root: str, indent: str) -> str:
    return "".join(U._mesh_usda(p[0], p[1], p[2], root, LOOKS[p[2]], 35.0, indent=indent, hidden=len(p) > 3 and p[3])
                   for p in parts)


def _materials(root: str, names) -> str:
    return "".join(U._material_usda(n, root, LOOKS[n]) for n in sorted(set(names)))


def _header(root: str) -> str:
    return f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)
"""


def _write(name: str, text: str) -> str:
    BUILD.mkdir(exist_ok=True)
    text = text.replace(f"@{(HERE / 'textures').as_posix()}/", "@../textures/")
    (BUILD / name).write_text(text)
    return f"build/{name}"


def _fixed_file(name: str, root: str, parts, colliders, friction=(0.6, 0.5)) -> str:
    """A fixed body: written as a rigid body (the loader needs one); the task spawns it without a mass (kinematic)."""
    text = _header(root) + f"""
def Xform "{root}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
)
{{
    float physics:mass = 5.0{U.physics_material("PhysicsMaterial", friction[0], friction[1])}
{"".join(colliders)}

    def Scope "Looks"
    {{{_materials(root, [p[2] for p in parts])}
    }}

    def Xform "visual"
    {{{_meshes(parts, root, "        ")}
    }}
}}
"""
    return _write(name, text)


# -- shredder -----------------------------------------------------------------------------------------------------------
def shredder_parts():
    x0, x1 = SHRED_X
    y0, y1 = SHRED_Y
    sx0, sx1 = SLOT_X
    sy0, sy1 = SLOT_Y
    t, b = SHRED_TOP, BIN_TOP
    parts = [
        # the head, built round the slot: front, back and the two ends
        ("head_front", _box((x0, y0, b + 0.002), (sx0, y1, t), bevel=0.003, round_xy=0.004), "abs_grey"),
        ("head_back", _box((sx1, y0, b + 0.002), (x1, y1, t), bevel=0.003, round_xy=0.004), "abs_grey"),
        ("head_end_r", _box((sx0 - 0.002, y0, b + 0.002), (sx1 + 0.002, sy0, t - 0.0005), bevel=0.0015), "abs_grey"),
        ("head_end_l", _box((sx0 - 0.002, sy1, b + 0.002), (sx1 + 0.002, y1, t - 0.0005), bevel=0.0015), "abs_grey"),
        ("throat", _box((sx0 + 0.0005, sy0, b + 0.01), (sx1 - 0.0005, sy1, t - 0.0035)), "throat"),
        ("cutters", _box((sx0 + 0.003, sy0 + 0.002, b + 0.03), (sx1 - 0.003, sy1 - 0.002, t - 0.012)), "steel_dark"),
        # a darker trim band round the slot, like the moulded funnel of a real feed opening
        ("trim_front", _box((sx0 - 0.010, sy0 - 0.004, t - 0.0006), (sx0 - 0.0005, sy1 + 0.004, t + 0.0004)), "abs_trim"),
        ("trim_back", _box((sx1 + 0.0005, sy0 - 0.004, t - 0.0006), (sx1 + 0.010, sy1 + 0.004, t + 0.0004)), "abs_trim"),
        # the bin: a black drawer with a smoked window in its front
        ("bin_back", _box((x0 + 0.004, y0 + 0.002, 0.0), (x1 - 0.002, y1 - 0.002, b + 0.002), bevel=0.0015), "abs_black"),
        ("bin_front_low", _box((x0, y0 + 0.002, 0.0), (x0 + 0.004, y1 - 0.002, 0.008), bevel=0.001), "abs_black"),
        ("bin_front_high", _box((x0, y0 + 0.002, 0.034), (x0 + 0.004, y1 - 0.002, b + 0.002), bevel=0.001), "abs_black"),
        ("bin_front_r", _box((x0, y0 + 0.002, 0.008), (x0 + 0.004, -0.33, 0.034)), "abs_black"),
        ("bin_front_l", _box((x0, -0.13, 0.008), (x0 + 0.004, y1 - 0.002, 0.034)), "abs_black"),
        ("window", _box((x0 + 0.001, -0.33, 0.008), (x0 + 0.002, -0.13, 0.034)), "smoke"),
        # controls: a slide switch (set to Auto) and the two lights
        ("switch", _box((SWITCH_XY[0] - 0.009, SWITCH_XY[1] - 0.016, t), (SWITCH_XY[0] + 0.009, SWITCH_XY[1] + 0.016, t + 0.003),
                        bevel=0.001, round_xy=0.003), "abs_black"),
        ("slider", _box((SWITCH_XY[0] - 0.005, SWITCH_XY[1] + 0.004, t + 0.003), (SWITCH_XY[0] + 0.005, SWITCH_XY[1] + 0.013, t + 0.006),
                        bevel=0.0008), "slider"),
        ("ring_green", U.cylinder(LIGHT_R + 0.0015, 0.0012, (*GREEN_XY, t + 0.0006), segments=24), "abs_black"),
        ("ring_red", U.cylinder(LIGHT_R + 0.0015, 0.0012, (*RED_XY, t + 0.0006), segments=24), "abs_black"),
        ("led_green", U.cylinder(LIGHT_R, 0.0024, (*GREEN_XY, t + 0.0012), segments=24, bevel=0.0006), "led_green"),
        ("jam_off", U.cylinder(LIGHT_R, 0.0024, (*RED_XY, t + 0.0012), segments=24, bevel=0.0006), "led_red_off"),
        ("jam_on", U.cylinder(LIGHT_R, 0.0024, (*RED_XY, t + 0.0012), segments=24, bevel=0.0006), "led_red_on", True),
    ]
    rng = np.random.default_rng(3)
    strips = []
    for k in range(70):                                   # shredded strips in the bin, shown once a stack is shredded
        cx, cy = rng.uniform(x0 + 0.012, x0 + 0.05), rng.uniform(-0.325, -0.135)
        cz = 0.004 + rng.uniform(0.0, 0.022)
        s = _box((-0.0025, -0.018, -0.00015), (0.0025, 0.018, 0.00015)).turned(rng.uniform(0, 180), "z")
        s = s.turned(rng.uniform(-35, 35), "x").moved((cx, cy, cz))
        strips.append(s)
    parts.append(("shreds", U.join(*strips), "shreds", True))
    return parts


def shredder_colliders(root: str):
    x0, x1 = SHRED_X
    y0, y1 = SHRED_Y
    sx0, sx1 = SLOT_X
    sy0, sy1 = SLOT_Y
    t = SHRED_TOP
    return [_collider("c_front", (x0, y0, 0.0), (sx0, y1, t), root),
            _collider("c_back", (sx1, y0, 0.0), (x1, y1, t), root),
            _collider("c_end_r", (sx0, y0, 0.0), (sx1, sy0, t), root),
            _collider("c_end_l", (sx0, sy1, 0.0), (sx1, y1, t), root)]


def _shredder() -> str:
    root = "Shredder"
    return _fixed_file("shredder.usda", root, shredder_parts(), shredder_colliders(root))


# -- letter rack --------------------------------------------------------------------------------------------------------
# -- the folded invoices --------------------------------------------------------------------------------------------------
def _invoices() -> str:
    root = "Invoices"
    t, l, h = STACK
    core = _box((-t / 2 + 0.00012, -l / 2, -h / 2), (t / 2 - 0.00012, l / 2, h / 2), bevel=0.0006)
    face_front = _box((-t / 2, -l / 2 + 0.0004, -h / 2 + 0.0004), (-t / 2 + 0.00012, l / 2 - 0.0004, h / 2 - 0.0004))
    face_back = _box((t / 2 - 0.00012, -l / 2 + 0.0004, -h / 2 + 0.0004), (t / 2, l / 2 - 0.0004, h / 2 - 0.0004))
    # the three folded invoices show as layers along the top edge
    layers = [_box((-t / 2 + 0.0001 + k * t / 3, -l / 2 + 0.001, h / 2 - 0.0002), (-t / 2 + (k + 1) * t / 3 - 0.0002, l / 2 - 0.001, h / 2 + 0.0001))
              for k in range(3)]
    parts = [("core", core, "paper"), ("face_front", face_front, "invoice"), ("face_back", face_back, "invoice"),
             ("layers", U.join(*layers), "paper")]
    cols = [_collider("c_stack", (-t / 2, -l / 2, -h / 2), (t / 2, l / 2, h / 2), "Invoices")]
    U.write_usda(str(BUILD / "invoices.usda"), root, parts, LOOKS, cols, mass=STACK_MASS, friction=(0.6, 0.5))
    return "build/invoices.usda"


# -- binder clip ----------------------------------------------------------------------------------------------------------
SLOPE_DEG = math.degrees(math.atan2(SPINE_HW - 0.0028, SPINE_Z[1] - LIP_Z[1] - 0.0005))


def _side(sign: float):
    """A sloped side wall from the spine's edge down to the lip, and the lip's rolled edge, on the side `sign` of x."""
    zt, zb = SPINE_Z[1] - 0.0005, LIP_Z[1]
    xt, xb = sign * (SPINE_HW - 0.0003), sign * 0.0028
    length = math.hypot(xt - xb, zt - zb)
    wall = _box((-0.0003, -CLIP_L / 2, -length / 2), (0.0003, CLIP_L / 2, length / 2)).turned(sign * SLOPE_DEG, "y")
    wall = wall.moved(((xt + xb) / 2, 0.0, (zt + zb) / 2))
    lip = _box((sign * 0.00175 if sign > 0 else -0.00275, -CLIP_L / 2, LIP_Z[0]),
               (0.00275 if sign > 0 else -0.00175, CLIP_L / 2, LIP_Z[1] + 0.0004))
    roll = U.cylinder(0.0011, CLIP_L, (sign * 0.0031, 0.0, LIP_Z[1] - 0.0002), axis="y", segments=12)
    return [wall, lip, roll]


def _handle(sign: float, y_half: float = 0.0125) -> U.Mesh:
    pts = [(sign * 0.0040, -y_half, LIP_Z[1]), (sign * (SPINE_HW + 0.0012), -y_half, SPINE_Z[1] - 0.001),
           (sign * 0.0042, -y_half, HANDLE_TOP - 0.004), (sign * 0.0035, -y_half + 0.004, HANDLE_TOP),
           (sign * 0.0035, y_half - 0.004, HANDLE_TOP), (sign * 0.0042, y_half, HANDLE_TOP - 0.004),
           (sign * (SPINE_HW + 0.0012), y_half, SPINE_Z[1] - 0.001), (sign * 0.0040, y_half, LIP_Z[1])]
    return U.tube(pts, 0.0008, sides=8)


def clip_body_parts(scale: float = 1.0):
    spine = _box((-SPINE_HW, -CLIP_L / 2, SPINE_Z[0] + 0.0009), (SPINE_HW, CLIP_L / 2, SPINE_Z[1]), bevel=0.0003)
    body = U.join(spine, *_side(1.0))
    parts = [("body", body, "clip_black"), ("handle_p", _handle(1.0), "wire"), ("handle_n", _handle(-1.0), "wire")]
    if scale != 1.0:
        parts = [(n, m.scaled(scale), look) for n, m, look in parts]
    return parts


def clip_jaw_parts(scale: float = 1.0):
    parts = [("jaw", U.join(*_side(-1.0)), "clip_black")]
    if scale != 1.0:
        parts = [(n, m.scaled(scale), look) for n, m, look in parts]
    return parts


def _body_block(name, mass, com, inertia, colliders, visuals, extra_api="", extra="") -> str:
    return f"""
    def Xform "{name}" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"{extra_api}]
    )
    {{
        float physics:mass = {mass:.5f}
        point3f physics:centerOfMass = ({com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f})
        float3 physics:diagonalInertia = ({inertia[0]:.3e}, {inertia[1]:.3e}, {inertia[2]:.3e})
        float physxRigidBody:maxDepenetrationVelocity = 1
        float physxRigidBody:sleepThreshold = 0
        float physxRigidBody:stabilizationThreshold = 0{extra}
{colliders}

        def Xform "visual"
        {{{visuals}
        }}
    }}"""


def _clip() -> str:
    root = "BinderClip"
    body, jaw = clip_body_parts(), clip_jaw_parts()
    ind = "            "
    hx, hy, hz = HANDLE_BOX
    body_cols = "".join([
        _collider("c_spine", (-SPINE_HW, -CLIP_L / 2, SPINE_Z[0]), (SPINE_HW, CLIP_L / 2, SPINE_Z[1]), root, indent="        "),
        _collider("c_handles", (hx[0], hy[0], hz[0]), (hx[1], hy[1], hz[1]), root, indent="        "),
        _collider("c_lip", (0.00175, -CLIP_L / 2, LIP_Z[0]), (0.00275, CLIP_L / 2, LIP_Z[1]), root, indent="        "),
        _collider("c_wall", (0.0045, -CLIP_L / 2, LIP_Z[1] + 0.001), (0.0075, CLIP_L / 2, SPINE_Z[0]), root, indent="        "),
    ])
    jaw_cols = "".join([
        _collider("c_lip", (-0.00275, -CLIP_L / 2, LIP_Z[0]), (-0.00175, CLIP_L / 2, LIP_Z[1]), root, indent="        "),
        _collider("c_wall", (-0.0075, -CLIP_L / 2, LIP_Z[1] + 0.001), (-0.0045, CLIP_L / 2, SPINE_Z[0]), root, indent="        "),
    ])
    m = CLIP_MASS
    inertia = (m / 12 * (0.02 ** 2 + 0.045 ** 2), m / 12 * (0.041 ** 2 + 0.045 ** 2), m / 12 * (0.041 ** 2 + 0.02 ** 2))
    blocks = [_body_block("body", m, (0.002, 0.0, 0.0), inertia, body_cols, _meshes(body, root, ind),
                          ', "PhysicsArticulationRootAPI", "PhysxArticulationAPI"', """
        bool physxArticulation:enabledSelfCollisions = 0
        int physxArticulation:solverPositionIterationCount = 32
        int physxArticulation:solverVelocityIterationCount = 16
        float physxArticulation:sleepThreshold = 0
        float physxArticulation:stabilizationThreshold = 0"""),
              _body_block("jaw", JAW_MASS, (-0.004, 0.0, -0.007), (3e-7, 1e-7, 3e-7), jaw_cols, _meshes(jaw, root, ind))]
    joint = f"""
        def PhysicsPrismaticJoint "jaw" (
            prepend apiSchemas = ["PhysicsDriveAPI:linear", "PhysxJointAPI"]
        )
        {{
            uniform token physics:axis = "X"
            rel physics:body0 = </{root}/body>
            rel physics:body1 = </{root}/jaw>
            point3f physics:localPos0 = (0, 0, 0)
            quatf physics:localRot0 = (1, 0, 0, 0)
            point3f physics:localPos1 = (0, 0, 0)
            quatf physics:localRot1 = (1, 0, 0, 0)
            float physics:lowerLimit = -0.006
            float physics:upperLimit = 0.0034
            uniform token drive:linear:physics:type = "force"
            float drive:linear:physics:stiffness = 1000
            float drive:linear:physics:damping = 0.2
            float drive:linear:physics:maxForce = 5
            float physxJoint:armature = 0.00001
        }}"""
    text = _header(root) + f"""
def Xform "{root}"
{{{U.physics_material("PhysicsMaterial", 0.5, 0.45)}

    def Scope "Looks"
    {{{_materials(root, [p[2] for p in body + jaw])}
    }}
{"".join(blocks)}

    def Scope "joints"
    {{{joint}
    }}
}}
"""
    return _write("clip.usda", text)


# -- clip dish --------------------------------------------------------------------------------------------------------------
def _dish() -> str:
    root = "ClipDish"
    cx, cy = DISH_C
    sx, sy, sz = DISH
    w = DISH_WALL
    outer = U.rounded_rect(sx, sy, 0.012, 6) + np.array([cx, cy])
    inner = U.rounded_rect(sx - 2 * w, sy - 2 * w, 0.008, 6) + np.array([cx, cy])
    parts = [("wall", U.ring_prism(outer, inner, 0.0, sz), "ceramic"),
             ("floor", U.prism(U.rounded_rect(sx - 0.002, sy - 0.002, 0.011, 6) + np.array([cx, cy]), 0.0, w), "ceramic")]
    cols = [_collider("c_floor", (cx - sx / 2, cy - sy / 2, 0.0), (cx + sx / 2, cy + sy / 2, w), root),
            _collider("c_wall_f", (cx - sx / 2, cy - sy / 2, w), (cx - sx / 2 + w, cy + sy / 2, sz), root),
            _collider("c_wall_b", (cx + sx / 2 - w, cy - sy / 2, w), (cx + sx / 2, cy + sy / 2, sz), root),
            _collider("c_wall_r", (cx - sx / 2, cy - sy / 2, w), (cx + sx / 2, cy - sy / 2 + w, sz), root),
            _collider("c_wall_l", (cx - sx / 2, cy + sy / 2 - w, w), (cx + sx / 2, cy + sy / 2, sz), root)]
    # three more binder clips lying in the dish: 25, 32 and 41 mm, each on its side
    for k, (scale, (dx, dy), yaw) in enumerate([(25 / 41, (-0.018, 0.020), 30.0), (32 / 41, (0.015, 0.012), -60.0),
                                                 (1.0, (-0.006, -0.018), 95.0)]):
        lie = clip_body_parts(scale) + clip_jaw_parts(scale)
        half = SPINE_HW * scale
        for name, mesh, look in lie:
            m = mesh.turned(90.0, "y").turned(yaw, "z").moved((cx + dx, cy + dy, w + half))
            parts.append((f"clip{k}_{name}", m, look))
        r = math.radians(yaw)
        hx, hy = 0.024 * scale, CLIP_L / 2 * scale
        ex, ey = abs(hx * math.cos(r)) + abs(hy * math.sin(r)), abs(hx * math.sin(r)) + abs(hy * math.cos(r))
        cols.append(_collider(f"c_clip{k}", (cx + dx - ex * 0.7, cy + dy - ey * 0.7, w), (cx + dx + ex * 0.7, cy + dy + ey * 0.7, w + 2 * half * 0.8), root))
    return _fixed_file("dish.usda", root, parts, cols)


def build() -> dict[str, str]:
    return {"shredder": _shredder(), "invoices": _invoices(), "clip": _clip(), "dish": _dish()}


if __name__ == "__main__":
    print(build())
    print("slope", round(SLOPE_DEG, 1))
