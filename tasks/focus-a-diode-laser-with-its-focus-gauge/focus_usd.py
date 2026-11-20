"""The task's objects as plain-text USD: the open-frame 5 W diode laser engraver (Creality CR-Laser Falcon 5W: frame,
gantry, carriage and fixing plate), the 3 mm aluminium plate under the work, the 3 mm plywood sheet, and the laser
module as a fixed-base articulation: a vertical slide (module on its fixing plate), the fixed focus bar on a hinge on
its -y side, and the two hand screws on hinges on the same side.

Visuals are meshes with the real product's proportions; colliders are separate plain boxes and cylinders (purpose
"guide"). Units: metres. Each file's origin is stated with its builder.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

import usdkit as uk
from usdkit import Look

HERE = Path(__file__).resolve().parent
OUT = HERE / "build"

# -- the laser module ------------------------------------------------------------------------------------------------------
# Module frame (articulation file origin): on the module's vertical axis, at the red shield's lower edge, at the start.
BODY = 0.042                 # black body 4.2 x 4.2 cm in plan
SHIELD_X, SHIELD_Y = 0.046, 0.050   # red acrylic shield at the foot: 4.6 deep (x), 5.0 wide (y)
SHIELD_H = 0.042
BODY_H = 0.126               # shield's lower edge to the top of the module
MODULE_MASS = 0.40
BACK = SHIELD_X / 2          # the module's back face (shield), +x
SLIDE_LOW, SLIDE_HIGH = -0.016, 0.027     # slide travel from the start (shield edge 2.3 cm up): 0.7 to 5.0 cm up

# Fixed focus bar: a black T-shaped steel bar on a hinge on the -y side; axis along y.
BAR_PIVOT = np.array([BACK - 0.006, -SHIELD_Y / 2 - 0.004, 0.034])   # 0.6 cm in front of the back face, 3.4 cm up
BAR_L = 0.039                # pivot to the rounded end's tip
BAR_W, BAR_T = 0.007, 0.0025  # leg width (in its swing plane), thickness (along y)
BAR_COL_T = 0.007            # collider thickness along y (thicker than the strip so a fingertip cannot pass)
BAR_HEAD, BAR_HEAD_H = 0.016, 0.006   # the T's head across the pivot
BAR_TOP = BAR_HEAD_H / 2     # the head's edge beyond the pivot
BAR_MASS = 0.005
STOW_DEG = 90.0              # stowed: turned 90 degrees forward to level, its end toward the robot (-x)
HANG_DEG = 0.0               # hanging stop: straight down
BAR_STOP_HI = 2.0            # degrees past the stowed angle
BAR_COM_OFF = -0.002         # the bar's centre of mass lies 2 mm forward of its line (toward -x hanging), so it hangs
                             # back on its stop

# Hand screws: two, on hinges on the -y side; axes along y.
SCREW_X = BACK - 0.018       # 1.8 cm in front of the back face
SCREW_Z = {"screw_l": 0.059, "screw_u": 0.089}   # above the shield's lower edge
HEAD_R, HEAD_T = 0.007, 0.008    # knurled head: 1.4 cm across, 0.8 cm thick
SHANK = 0.005                # the screw stands 0.5 cm off the body's side
HEAD_Y = -BODY / 2 - SHANK - HEAD_T / 2   # head centre (module frame y)
SCREW_MASS = 0.008
SCREW_LOW = -300.0           # degrees: how far a screw can be backed off (snug stop at 0)


def bar_dir(theta_deg: float) -> np.ndarray:
    """Unit vector from the bar's axis to its end at angle theta (0 hanging straight down, + swings the end toward -x)."""
    t = math.radians(theta_deg)
    return np.array([-math.sin(t), 0.0, -math.cos(t)])


def bar_reach() -> float:
    """G: how far the bar's lowest point (its rounded end) lies below the shield's lower edge, hanging on its stop."""
    c = BAR_PIVOT + (BAR_L - BAR_W / 2) * bar_dir(HANG_DEG)
    return -(c[2] - BAR_W / 2)


def screw_axis(name: str) -> np.ndarray:
    return np.array([SCREW_X, HEAD_Y, SCREW_Z[name]])


LOOKS = {
    "frame": Look(color=(0.40, 0.41, 0.43), metallic=0.75, roughness=0.40, texture="brushed.png", tile=0.10),
    "frame_dark": Look(color=(0.16, 0.16, 0.17), metallic=0.6, roughness=0.5),
    "plate": Look(color=(0.55, 0.56, 0.58), metallic=0.8, roughness=0.35, texture="brushed.png", tile=0.08),
    "body": Look(color=(0.035, 0.035, 0.038), metallic=0.5, roughness=0.45, texture="brushed.png", tile=0.05),
    "fins": Look(color=(0.02, 0.02, 0.022), metallic=0.4, roughness=0.55),
    "shield": Look(color=(0.75, 0.02, 0.03), roughness=0.12, opacity=0.78),
    "lens": Look(color=(0.05, 0.05, 0.06), metallic=0.3, roughness=0.2),
    "fan": Look(texture="fan.png", tile_uv=(BODY, BODY), origin=(-BODY / 2, -BODY / 2, 0.0), roughness=0.6),
    "board": Look(color=(0.05, 0.18, 0.08), roughness=0.5),
    "steel": Look(color=(0.70, 0.71, 0.73), metallic=0.95, roughness=0.25),
    "knurl": Look(color=(0.78, 0.79, 0.80), metallic=0.95, roughness=0.3, texture="knurl.png", tile=0.006),
    "black": Look(color=(0.025, 0.025, 0.028), roughness=0.55),
    "magnet": Look(color=(0.12, 0.12, 0.13), metallic=0.6, roughness=0.35),
    "label": Look(color=(0.85, 0.85, 0.83), roughness=0.4),
    "cable": Look(color=(0.02, 0.02, 0.02), roughness=0.7),
    "belt": Look(color=(0.03, 0.03, 0.03), roughness=0.8),
    "wheel": Look(color=(0.04, 0.04, 0.045), roughness=0.6),
    "wood": Look(texture="basswood.png", tile=0.10, roughness=0.75),
    "ply_edge": Look(texture="ply_edge.png", tile_uv=(0.01, 0.003), roughness=0.8),
    "bubble": Look(color=(0.35, 0.85, 0.30), roughness=0.2, opacity=0.8),
}


# -- helpers --------------------------------------------------------------------------------------------------------
def box_col(name, size, center, root, rot=None, offset=0.001, material="PhysicsMaterial", indent="    ") -> str:
    sx, sy, sz = size
    cx, cy, cz = center
    w, qx, qy, qz = rot if rot is not None else (1.0, 0.0, 0.0, 0.0)
    text = f"""
    def Cube "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]
    )
    {{
        rel material:binding:physics = </{root}/{material}>
        uniform token purpose = "guide"
        double size = 1
        float physxCollision:contactOffset = {offset}
        float physxCollision:restOffset = 0
        double3 xformOp:translate = ({cx:.6f}, {cy:.6f}, {cz:.6f})
        quatf xformOp:orient = ({w:.6f}, {qx:.6f}, {qy:.6f}, {qz:.6f})
        float3 xformOp:scale = ({sx:.6f}, {sy:.6f}, {sz:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]
    }}"""
    return text.replace("\n    ", "\n" + indent)


def cyl_col(name, radius, height, center, root, axis="Y", offset=0.001, material="PhysicsMaterial", indent="    ") -> str:
    c = center
    return f"""
{indent}def Cylinder "{name}" (
{indent}    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]
{indent})
{indent}{{
{indent}    rel material:binding:physics = </{root}/{material}>
{indent}    uniform token purpose = "guide"
{indent}    uniform token axis = "{axis}"
{indent}    double radius = {radius:.6f}
{indent}    double height = {height:.6f}
{indent}    float physxCollision:contactOffset = {offset}
{indent}    float physxCollision:restOffset = 0
{indent}    double3 xformOp:translate = ({c[0]:.6f}, {c[1]:.6f}, {c[2]:.6f})
{indent}    uniform token[] xformOpOrder = ["xformOp:translate"]
{indent}}}"""


def _quat_y(deg):
    h = math.radians(deg) / 2
    return (math.cos(h), 0.0, math.sin(h), 0.0)


def _meshes(parts, root, indent="        ") -> str:
    return "".join(uk._mesh_usda(p[0], p[1], p[2], root, LOOKS[p[2]], 35.0, indent=indent) for p in parts)


def _looks(parts, root) -> str:
    return "".join(uk._material_usda(m, root, LOOKS[m]) for m in sorted({p[2] for p in parts}))


def _write(name: str, text: str) -> str:
    OUT.mkdir(exist_ok=True)
    path = OUT / f"{name}.usda"
    rel = Path("..") / "textures"
    text = text.replace(f"@{(HERE / 'textures').as_posix()}/", f"@{rel.as_posix()}/")
    path.write_text(text)
    return str(path.relative_to(HERE))


def _body(name, root, parts, colliders, mass=None, friction=(0.6, 0.5), solver=16, extra_mat="", depen=1.0) -> str:
    kin = mass is None
    m = 1.0 if kin else mass
    head = """ (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
)"""
    body = f"""
    float physics:mass = {m:.6f}
    int physxRigidBody:solverPositionIterationCount = {solver}
    int physxRigidBody:solverVelocityIterationCount = 4
    float physxRigidBody:maxDepenetrationVelocity = {depen}
    float physxRigidBody:sleepThreshold = 0
    float physxRigidBody:stabilizationThreshold = 0{chr(10) + "    bool physics:kinematicEnabled = 1" if kin else ""}
"""
    text = f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}"{head}
{{{body}{uk.physics_material("PhysicsMaterial", *friction)}{extra_mat}
{colliders}

    def Scope "Looks"
    {{{_looks(parts, root)}
    }}

    def Xform "visual"
    {{{_meshes(parts, root)}
    }}
}}
"""
    return _write(name, text)


def box(size, center, bevel=0.0):
    return uk.box(size, center=center, bevel=bevel)


# -- the machine: frame, gantry, carriage and fixing plate (fixed). Origin: table top, below the module's axis. ------------
# Station layout (m, from the arms' midpoint): frame x 0.19..0.758, y -0.316..+0.316; module axis at (0.30, 0.05).
FRAME_X = (0.19, 0.758)
FRAME_Y = (-0.316, 0.316)
RAIL_W, RAIL_H, RAIL_Z0 = 0.040, 0.020, 0.020    # 4 x 2 cm profiles on 2 cm corner feet: tops 4 cm up
MODULE_XY = np.array([0.30, 0.05])
BEAM_X = (0.360, 0.400)                          # gantry beam (station x)
BEAM_Z = (0.165, 0.198)                          # at the module's top, its top the machine's highest point
FIX_X = (MODULE_XY[0] + BACK + 0.0015, MODULE_XY[0] + BACK + 0.0045)     # fixing plate, 1.5 mm behind the module
FIX_Y, FIX_Z = 0.040, (0.030, 0.155)
CAR_X = (0.340, 0.343)                                                     # carriage plate, on its wheels in front of the beam
CAR_Y, CAR_Z = 0.050, (0.100, 0.198)
PLATE = (0.24, 0.54, -0.15, 0.25, 0.003)          # x0, x1, y0, y1, thickness (station)


def machine() -> str:
    """Origin: the table top at the module's axis (station (0.30, 0.05))."""
    root = "Engraver"
    ox, oy = MODULE_XY
    x0, x1 = FRAME_X[0] - ox, FRAME_X[1] - ox
    y0, y1 = FRAME_Y[0] - oy, FRAME_Y[1] - oy
    w, h, z0 = RAIL_W, RAIL_H, RAIL_Z0
    zc = z0 + h / 2
    parts, cols = [], ""
    rails = {
        "rail_front": ((w, y1 - y0, h), (x0 + w / 2, (y0 + y1) / 2, zc)),
        "rail_back": ((w, y1 - y0, h), (x1 - w / 2, (y0 + y1) / 2, zc)),
        "rail_left": ((x1 - x0 - 2 * w, w, h), ((x0 + x1) / 2, y1 - w / 2, zc)),
        "rail_right": ((x1 - x0 - 2 * w, w, h), ((x0 + x1) / 2, y0 + w / 2, zc)),
    }
    for n, (s, c) in rails.items():
        parts.append((n, box(s, c, bevel=0.002), "frame"))
        cols += box_col(f"col_{n}", s, c, root)
    # T-slots along the rails' outer faces, the belts along the side rails' tops, the level bubble on the front rail
    parts.append(("slot_front", box((0.0012, y1 - y0 - 0.02, 0.006), (x0 - 0.0005, (y0 + y1) / 2, zc)), "frame_dark"))
    for side, yy in (("l", y1 - w / 2), ("r", y0 + w / 2)):
        parts.append((f"belt_{side}", box((x1 - x0 - 2 * w - 0.01, 0.010, 0.0015), ((x0 + x1) / 2, yy, z0 + h + 0.0007)), "belt"))
    parts.append(("bubble", uk.cylinder(0.006, 0.002, center=(x0 + w / 2, y0 + 0.08, z0 + h + 0.001), segments=20), "bubble"))
    # corner feet (angled plates in the real machine; drawn as dark blocks)
    for cx in (x0 + w / 2, x1 - w / 2):
        for cy in (y0 + w / 2, y1 - w / 2):
            foot = ((0.036, 0.036, z0), (cx, cy, z0 / 2))
            parts.append((f"foot_{len(parts)}", box(*foot, bevel=0.002), "frame_dark"))
            cols += box_col(f"col_foot_{len(parts)}", *foot, root)
    # control box at the front right foot, outside the frame
    cb = ((0.030, 0.090, 0.060), (x0 - 0.016, y0 + 0.050, 0.032))
    parts.append(("control_box", box(*cb, bevel=0.002), "frame"))
    parts.append(("button", uk.cylinder(0.008, 0.002, center=(x0 - 0.0315, y0 + 0.05, 0.034), axis="x", segments=20), "steel"))
    cols += box_col("col_control_box", *cb, root)
    # gantry beam between the side rails, end plates riding on them, the X motor at the -y end
    bx0, bx1 = BEAM_X[0] - ox, BEAM_X[1] - ox
    bz0, bz1 = BEAM_Z
    beam = ((bx1 - bx0, y1 - y0 - 2 * w, bz1 - bz0), ((bx0 + bx1) / 2, (y0 + y1) / 2, (bz0 + bz1) / 2))
    parts.append(("beam", box(*beam, bevel=0.002), "frame"))
    cols += box_col("col_beam", *beam, root)
    parts.append(("beam_belt", box((0.0015, y1 - y0 - 2 * w - 0.01, 0.008), (bx0 - 0.0008, (y0 + y1) / 2, bz0 + 0.020)), "belt"))
    for side, yy in (("l", y1 - w / 2), ("r", y0 + w / 2)):
        plate = ((0.080, 0.006, bz1 - z0), ((bx0 + bx1) / 2, yy + (0.023 if side == "l" else -0.023), (z0 + bz1) / 2))
        parts.append((f"end_{side}", box(*plate, bevel=0.0015), "frame"))
        cols += box_col(f"col_end_{side}", *plate, root)
    motor = ((0.042, 0.042, 0.042), ((bx0 + bx1) / 2, y0 + w / 2, bz0 - 0.010))
    parts.append(("motor", box(*motor, bevel=0.002), "black"))
    cols += box_col("col_motor", *motor, root)
    # the carriage plate on the beam (four wheels) and the fixing plate down behind the module
    cx0, cx1 = CAR_X[0] - ox, CAR_X[1] - ox
    car = ((cx1 - cx0, CAR_Y, CAR_Z[1] - CAR_Z[0]), ((cx0 + cx1) / 2, 0.0, (CAR_Z[0] + CAR_Z[1]) / 2))
    parts.append(("carriage", box(*car, bevel=0.0008), "frame"))
    cols += box_col("col_carriage", *car, root)
    for wy in (-0.017, 0.017):
        for wz in (bz0 + 0.006, bz1 - 0.006):
            parts.append((f"wheel_{len(parts)}", uk.cylinder(0.0115, 0.010, center=((cx1 + bx0) / 2, wy, wz), axis="x",
                                                             segments=24), "wheel"))
    bridge = ((bx0 - cx1 + 0.001, 0.030, 0.010), ((cx1 + bx0) / 2, 0.0, (bz0 + bz1) / 2))
    parts.append(("bridge", box(*bridge), "frame_dark"))
    fx0, fx1 = FIX_X[0] - ox, FIX_X[1] - ox
    stand = ((cx0 - fx1 + 0.001, 0.030, 0.050), ((fx1 + cx0) / 2, 0.0, 0.150))        # the fixing plate's bracket
    parts.append(("bracket", box(*stand, bevel=0.0008), "frame"))
    cols += box_col("col_bracket", *stand, root)
    fix = ((fx1 - fx0, FIX_Y, FIX_Z[1] - FIX_Z[0]), ((fx0 + fx1) / 2, 0.0, (FIX_Z[0] + FIX_Z[1]) / 2))
    parts.append(("fixing_plate", box(*fix, bevel=0.0006), "frame"))
    cols += box_col("col_fixing_plate", *fix, root)
    return _body("machine", root, parts, cols, mass=None, friction=(0.5, 0.45))


def plate() -> str:
    """The 3 mm aluminium plate under the work. Origin: the table top at the plate's centre."""
    root = "Plate"
    px0, px1, py0, py1, t = PLATE
    sx, sy = px1 - px0, py1 - py0
    parts = [("plate", box((sx, sy, t), (0, 0, t / 2), bevel=0.0004), "plate")]
    cols = box_col("col_plate", (sx, sy, t), (0, 0, t / 2), root, offset=0.002)
    return _body("plate", root, parts, cols, mass=None, friction=(0.5, 0.45))


SHEET = (0.200, 0.300, 0.003)   # x by y by thickness
SHEET_MASS = 0.080


def sheet() -> str:
    """Origin: the sheet's centre on its bottom face."""
    root = "Sheet"
    sx, sy, t = SHEET
    parts = [
        ("face", box((sx, sy, 0.0004), (0, 0, t - 0.0002)), "wood"),
        ("ply", box((sx - 0.0002, sy - 0.0002, t - 0.0004), (0, 0, (t - 0.0004) / 2)), "ply_edge"),
    ]
    cols = box_col("col_sheet", (sx, sy, t), (0, 0, t / 2), root, offset=0.001)
    return _body("sheet", root, parts, cols, mass=SHEET_MASS, friction=(0.5, 0.45), solver=32, depen=0.5)


# -- the laser module articulation --------------------------------------------------------------------------------------
def module_parts():
    """(name, mesh, look) in the module frame, at the start pose (shield's lower edge at z 0)."""
    h = BODY / 2
    parts = []
    # red acrylic shield (open at the bottom) and the lens seen through it
    parts.append(("shield", uk.box((SHIELD_X, SHIELD_Y, SHIELD_H), center=(0, 0, SHIELD_H / 2), bevel=0.0012), "shield"))
    parts.append(("lens", uk.cylinder(0.007, 0.018, center=(0, 0, 0.022), segments=20), "lens"))
    # black finned body, the fan collar and the driver board with its two posts on top
    zb0, zb1 = SHIELD_H, BODY_H - 0.020
    parts.append(("body", uk.box((BODY, BODY, zb1 - zb0), center=(0, 0, (zb0 + zb1) / 2), bevel=0.0015), "body"))
    for k in range(7):
        fx = -BODY / 2 + 0.004 + k * (BODY - 0.008) / 6
        parts.append((f"fin_f{k}", uk.box((0.0012, 0.0016, zb1 - zb0 - 0.034), center=(fx, -h - 0.0006, zb0 + 0.004 + (zb1 - zb0 - 0.034) / 2)), "fins"))
        parts.append((f"fin_b{k}", uk.box((0.0012, 0.0016, zb1 - zb0 - 0.034), center=(fx, h + 0.0006, zb0 + 0.004 + (zb1 - zb0 - 0.034) / 2)), "fins"))
    parts.append(("label", uk.box((0.0006, 0.032, 0.040), center=(-h - 0.0003, 0, zb0 + 0.044)), "label"))
    parts.append(("collar", uk.box((BODY + 0.002, BODY + 0.002, 0.008), center=(0, 0, zb1 - 0.004), bevel=0.001), "black"))
    parts.append(("fan_box", uk.box((BODY, BODY, 0.012), center=(0, 0, zb1 + 0.006), bevel=0.0012), "body"))
    parts.append(("fan", uk.box((BODY - 0.004, BODY - 0.004, 0.0004), center=(0, 0, zb1 + 0.0122)), "fan"))
    parts.append(("board", uk.box((BODY - 0.006, BODY - 0.010, 0.0016), center=(0, 0, BODY_H - 0.0045)), "board"))
    for k, py in enumerate((-0.016, 0.016)):
        parts.append((f"post_{k}", uk.cylinder(0.0035, 0.008, center=(-0.016, py, BODY_H - 0.004), segments=16), "black"))
    # the side strip that carries the hand screws (-y), and the magnet that holds the stowed bar
    parts.append(("strip", uk.box((0.020, 0.0016, 0.060), center=(SCREW_X, -h - 0.0008, 0.074), bevel=0.0004), "black"))
    for name in SCREW_Z:
        a = screw_axis(name)
        parts.append((f"shank_{name}", uk.cylinder(0.0022, SHANK, center=(a[0], -h - SHANK / 2, a[2]), axis="y", segments=16), "steel"))
    top = BAR_PIVOT + (BAR_L - 0.008) * bar_dir(STOW_DEG)
    parts.append(("magnet", uk.cylinder(0.003, 0.0012, center=(top[0], -SHIELD_Y / 2 - 0.0006, top[2]), axis="y", segments=16), "magnet"))
    # the power and signal cable from the board, back over the gantry
    pts = [(0.010, 0.0, BODY_H), (0.010, 0.0, BODY_H + 0.020), (0.030, 0.0, BODY_H + 0.035), (0.060, 0.0, BODY_H + 0.030)]
    parts.append(("cable", uk.tube(pts, 0.003, 8), "cable"))
    return parts


def module_colliders(root, ind):
    cols = box_col("col_body", (BODY, BODY, BODY_H - SHIELD_H), (0, 0, SHIELD_H + (BODY_H - SHIELD_H) / 2), root, indent=ind)
    cols += box_col("col_shield", (SHIELD_X, SHIELD_Y, SHIELD_H), (0, 0, SHIELD_H / 2), root, indent=ind)
    return cols


def bar_parts():
    """The bar in its stowed pose (module frame), drawn hanging and turned. Its free end is rounded."""
    rt = BAR_W / 2
    t = BAR_L - rt
    leg = uk.box((BAR_W, BAR_T, t), center=(0, 0, -t / 2), bevel=0.0006)
    end = uk.cylinder(rt, BAR_T, center=(0, 0, -(BAR_L - rt)), axis="y", segments=20)
    head = uk.box((BAR_HEAD, BAR_T, BAR_HEAD_H), center=(0, 0, 0), bevel=0.0006)
    pin = uk.cylinder(0.0022, BAR_T + 0.002, center=(0, -0.0003, 0), axis="y", segments=12)
    out = []
    for n, m, look in (("leg", leg, "black"), ("end", end, "black"), ("head", head, "black"), ("pin", pin, "steel")):
        out.append((n, m.turned(STOW_DEG, "y").moved(BAR_PIVOT), look))
    return out


def bar_colliders(root, ind):
    """A box along the leg, a box for the head and a cylinder (axis y) at its rounded end: the end rolls on the sheet."""
    rt = BAR_W / 2
    t = BAR_L - rt
    d = bar_dir(STOW_DEG)
    cols = box_col("col_bar", (BAR_W, BAR_COL_T, t), BAR_PIVOT + (t / 2) * d, root, rot=_quat_y(STOW_DEG), offset=0.002,
                   material="BarMaterial", indent=ind)
    cols += box_col("col_bar_head", (BAR_HEAD, BAR_COL_T, BAR_HEAD_H), BAR_PIVOT, root, rot=_quat_y(STOW_DEG), offset=0.002,
                    material="BarMaterial", indent=ind)
    cols += cyl_col("col_bar_end", rt, BAR_COL_T, BAR_PIVOT + (BAR_L - rt) * d, root, offset=0.002, material="BarMaterial",
                    indent=ind)
    return cols


def screw_parts(name):
    """The knurled head (module frame)."""
    a = screw_axis(name)
    head = uk.cylinder(HEAD_R, HEAD_T, center=(0, 0, 0), axis="y", segments=40, bevel=0.0006)
    face = uk.cylinder(HEAD_R - 0.0015, 0.0004, center=(0, -HEAD_T / 2 - 0.0002, 0), axis="y", segments=32)
    dot = uk.box((0.0012, 0.0004, HEAD_R - 0.002), center=(0, -HEAD_T / 2 - 0.0004, (HEAD_R - 0.002) / 2))   # a mark to see it turn
    return [(f"head_{name}", head.moved(a), "knurl"), (f"face_{name}", face.moved(a), "steel"), (f"mark_{name}", dot.moved(a), "black")]


def screw_colliders(name, root, ind):
    return cyl_col(f"col_{name}", HEAD_R, HEAD_T, screw_axis(name), root, offset=0.002, material="ScrewMaterial", indent=ind)


def _inertia_box(m, s):
    a, b, c = s
    return (m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12)


BAR_MAT = """
    def Material "BarMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {
        float physics:staticFriction = 0.15
        float physics:dynamicFriction = 0.12
        float physics:restitution = 0
        uniform token physxMaterial:frictionCombineMode = "min"
    }
"""


def laser() -> str:
    root = "Laser"
    ind = "        "

    def link(name, mass, com, inertia, cols="", parts=(), art=False):
        api = '"PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"'
        extra = ""
        if art:
            api += ', "PhysicsArticulationRootAPI", "PhysxArticulationAPI"'
            extra = """
        bool physxArticulation:enabledSelfCollisions = 0
        int physxArticulation:solverPositionIterationCount = 32
        int physxArticulation:solverVelocityIterationCount = 4
        float physxArticulation:sleepThreshold = 0
        float physxArticulation:stabilizationThreshold = 0"""
        vis = f"""

        def Xform "visual"
        {{{_meshes(parts, root, indent="            ")}
        }}""" if parts else ""
        return f"""
    def Xform "{name}" (
        prepend apiSchemas = [{api}]
    )
    {{
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        float physics:mass = {mass:.6f}
        point3f physics:centerOfMass = ({com[0]:.6f}, {com[1]:.6f}, {com[2]:.6f})
        float3 physics:diagonalInertia = ({inertia[0]:.3e}, {inertia[1]:.3e}, {inertia[2]:.3e})
        float physxRigidBody:maxDepenetrationVelocity = 0.5
        float physxRigidBody:sleepThreshold = 0
        float physxRigidBody:stabilizationThreshold = 0{extra}
{cols}{vis}
    }}"""

    mparts, bparts = module_parts(), bar_parts()
    sparts = {n: screw_parts(n) for n in SCREW_Z}
    t = math.radians(STOW_DEG)
    behind = np.array([math.cos(t), 0.0, -math.sin(t)])          # across the leg, toward +x when hanging
    bar_com = BAR_PIVOT + (BAR_L / 2) * bar_dir(STOW_DEG) + BAR_COM_OFF * behind
    links = link("carriage", 0.05, (0, 0, 0.06), (1e-5, 1e-5, 1e-5), art=True)
    links += link("module", MODULE_MASS, (0, 0, 0.060), _inertia_box(MODULE_MASS, (BODY, BODY, BODY_H)),
                  module_colliders(root, ind), mparts)
    bi = BAR_MASS * BAR_L ** 2 / 3
    links += link("bar", BAR_MASS, bar_com, (bi, bi, BAR_MASS * BAR_W ** 2 / 12 + 1e-8), bar_colliders(root, ind), bparts)
    si = 0.5 * SCREW_MASS * HEAD_R ** 2
    for n in SCREW_Z:
        links += link(n, SCREW_MASS, screw_axis(n), (si, si, si), screw_colliders(n, root, ind), sparts[n])

    def joint(name, kind, b0, b1, pos, lo, hi_, axis, armature=0.00001):
        drive = "linear" if kind == "Prismatic" else "angular"
        return f"""
        def Physics{kind}Joint "{name}" (
            prepend apiSchemas = ["PhysicsDriveAPI:{drive}", "PhysxJointAPI"]
        )
        {{
            uniform token physics:axis = "{axis}"
            rel physics:body0 = </{root}/{b0}>
            rel physics:body1 = </{root}/{b1}>
            point3f physics:localPos0 = ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})
            quatf physics:localRot0 = (1, 0, 0, 0)
            point3f physics:localPos1 = ({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})
            quatf physics:localRot1 = (1, 0, 0, 0)
            float physics:lowerLimit = {lo:.6f}
            float physics:upperLimit = {hi_:.6f}
            float physxJoint:armature = {armature}
            float physxJoint:maxJointVelocity = 1000000
            uniform token drive:{drive}:physics:type = "force"
            float drive:{drive}:physics:targetPosition = 0
            float drive:{drive}:physics:stiffness = 0
            float drive:{drive}:physics:damping = 0
        }}
"""

    joints = joint("slide", "Prismatic", "carriage", "module", (0, 0, 0), SLIDE_LOW, SLIDE_HIGH, "Z")
    joints += joint("bar", "Revolute", "module", "bar", BAR_PIVOT, HANG_DEG - STOW_DEG, BAR_STOP_HI, "Y")
    for n in SCREW_Z:
        joints += joint(n, "Revolute", "module", n, screw_axis(n), SCREW_LOW, 0.0, "Y", armature=0.0001)
    all_parts = mparts + bparts + [p for n in SCREW_Z for p in sparts[n]]
    text = f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}"
{{{uk.physics_material("PhysicsMaterial", 0.8, 0.7)}{BAR_MAT}{uk.physics_material("ScrewMaterial", 0.9, 0.8)}

    def Scope "Looks"
    {{{_looks(all_parts, root)}
    }}
{links}

    def Scope "joints"
    {{{joints}
    }}
}}
"""
    return _write("laser", text)


def build() -> dict:
    return {"machine": machine(), "plate": plate(), "sheet": sheet(), "laser": laser()}


if __name__ == "__main__":
    print(build())
    print("bar reach G (mm):", round(1000 * bar_reach(), 3))
