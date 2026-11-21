"""A clear bowl with one raw egg, a table fork, and a countertop microwave."""

from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np
from yam_arena.station import TABLE_CENTER, TABLE_SIZE
from yam_arena.task_file import TABLE_TOP

sys.path.insert(0, str(Path(__file__).resolve().parent))
import egg_parts as parts
from egg_state import CELL, CELLS, LIQUID_RADIUS, LIQUID_Z, TINE_THICKNESS, Trial, fork_outside_cavity

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = "Start cooking the egg in the microwave."

BOWL_START = (0.282, -0.187, TOP)
BOWL_RADIUS = 0.065
BOWL_HEIGHT = 0.047
FORK_START = (0.282, -0.337, TOP + 0.006)
FORK_TIP = (-0.118, 0.0, -0.004)
MICROWAVE_ORIGIN = (0.414, 0.018, TOP)
CAVITY_FLOOR = 0.029
CAVITY_LOW = np.add(MICROWAVE_ORIGIN, (-0.011, -0.129, CAVITY_FLOOR))
CAVITY_HIGH = np.add(MICROWAVE_ORIGIN, (0.289, 0.195, 0.222))
PLATE_CENTER = (0.111, 0.024)
PLATE_RADIUS = 0.099
PLATE_TOP = CAVITY_FLOOR + 0.006
DOOR_HINGE = (-0.006, 0.202, 0.0)
DOOR_HANDLE = (-0.048, -0.087, 0.138)
BUTTON = (-0.010, -0.166, 0.092)
DOOR_CLOSED = math.radians(3)
BUTTON_DEPTH = 0.0035

STEEL = (0.63, 0.65, 0.67)
GLASS = (0.78, 0.90, 0.94)
HOUSING = (0.75, 0.76, 0.77)
INSIDE = (0.84, 0.86, 0.87)
DARK = (0.13, 0.14, 0.16)
BUTTON_GREEN = (0.28, 0.56, 0.38)
EGG_WHITE = np.array((0.94, 0.92, 0.78))
EGG_YOLK = np.array((1.0, 0.56, 0.10))

_visuals = {}
_trial = Trial()
_status = {}
_measurements = {}


def _look(stage, prim, color, *, opacity=1.0, emission=False):
    from pxr import Gf, Sdf, UsdShade
    from yam_arena.station import linear_rgb

    look = UsdShade.Material.Define(stage, f"{prim.GetPath()}Look")
    shader = UsdShade.Shader.Define(stage, f"{look.GetPath()}/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.28)
    if emission:
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(color)))
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(look)
    return shader


def _egg_visuals(stage, body_path):
    from pxr import Gf, Sdf, UsdGeom, UsdShade
    from yam_arena.station import linear_rgb

    for child in stage.GetPrimAtPath(body_path).GetChildren():
        if child.GetName().startswith(("box_", "cyl_")):
            _look(stage, child, GLASS, opacity=0.22)

    surface = UsdGeom.Mesh.Define(stage, f"{body_path}/EggSurface")
    # Each face maps to a mixing cell. Clip peripheral corners to the circular waterline.
    corners = np.array([[-1, -1], [1, -1], [1, 1], [-1, 1]]) * (CELL / 2)
    xy = (CELLS[:, None] + corners).reshape(-1, 2)
    xy *= (LIQUID_RADIUS / np.maximum(LIQUID_RADIUS, np.linalg.norm(xy, axis=1)))[:, None]
    surface.CreatePointsAttr([Gf.Vec3f(float(x), float(y), LIQUID_Z) for x, y in xy])
    surface.CreateFaceVertexCountsAttr([4] * len(CELLS))
    surface.CreateFaceVertexIndicesAttr(list(range(len(xy))))
    surface.CreateSubdivisionSchemeAttr("none")
    surface.CreateDoubleSidedAttr(True)
    colors = surface.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
    shader = _look(stage, surface.GetPrim(), EGG_WHITE)
    reader = UsdShade.Shader.Define(stage, f"{shader.GetPath()}/Color")
    reader.CreateIdAttr("UsdPrimvarReader_float3")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("displayColor")
    reader.CreateInput("fallback", Sdf.ValueTypeNames.Float3).Set(Gf.Vec3f(*linear_rgb(EGG_WHITE)))
    shader.GetInput("diffuseColor").ConnectToSource(reader.CreateOutput("result", Sdf.ValueTypeNames.Float3))

    yolk = UsdGeom.Sphere.Define(stage, f"{body_path}/IntactYolk")
    yolk.CreateRadiusAttr(1.0)
    yolk.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, LIQUID_Z + 0.001))
    yolk.AddScaleOp().Set(Gf.Vec3f(0.021, 0.021, 0.013))
    _look(stage, yolk.GetPrim(), EGG_YOLK)
    _visuals.update(colors=colors, yolk=yolk)
    _draw_egg()


def _draw_egg():
    if "colors" not in _visuals:
        return
    from pxr import Gf
    from yam_arena.station import linear_rgb

    if _trial.egg.membrane_open:
        _visuals["yolk"].MakeInvisible()
        concentration = _trial.egg.yolk[:, None]
    else:
        _visuals["yolk"].MakeVisible()
        concentration = np.zeros((len(CELLS), 1))
    pigment = EGG_WHITE + concentration * (EGG_YOLK - EGG_WHITE)
    _visuals["colors"].Set([Gf.Vec3f(*linear_rgb(rgb)) for rgb in pigment])


def _microwave_visuals(stage, body_path):
    from pxr import Gf, UsdGeom

    door_path = f"{body_path.rsplit('/', 1)[0]}/door"
    # The translucent pane still has its full collision shape.
    _look(stage, stage.GetPrimAtPath(f"{door_path}/box_0"), DARK, opacity=0.38)
    lamp = UsdGeom.Cube.Define(stage, f"{body_path}/StartIndicator")
    lamp.CreateSizeAttr(1.0)
    lamp.AddTranslateOp().Set(Gf.Vec3d(-0.009, -0.167, 0.183))
    lamp.AddScaleOp().Set(Gf.Vec3f(0.003, 0.038, 0.016))
    _look(stage, lamp.GetPrim(), (0.28, 0.94, 0.41), emission=True)
    lamp.MakeInvisible()
    _visuals["lamp"] = lamp


def _ring_wall():
    n = 20
    radius = BOWL_RADIUS - 0.0015
    width = 2 * BOWL_RADIUS * math.tan(math.pi / n)
    for i in range(n):
        theta = 2 * math.pi * i / n
        yield (
            (radius * math.cos(theta), radius * math.sin(theta), (BOWL_HEIGHT + 0.004) / 2),
            (0.003, width, BOWL_HEIGHT - 0.004), GLASS, ("Z", theta),
        )


BOWL = replace(parts.part(
    "glass_bowl", BOWL_START, boxes=tuple(_ring_wall()), mass=0.225, friction=0.85,
    cylinders=(((0.0, 0.0, 0.0025), BOWL_RADIUS, 0.005, GLASS),),
), meta={"decorate": _egg_visuals})
FORK = parts.part(
    "table_fork", FORK_START, mass=0.032, friction=1.0, boxes=(
        ((0.014, 0.0, 0.001), (0.088, 0.014, 0.004), STEEL),
        ((-0.052, 0.0, 0.000), (0.045, 0.010, 0.003), STEEL),
        ((-0.082, 0.0, -0.002), (0.022, 0.023, 0.003), STEEL),
        *[((-0.105, y, -0.004), (0.027, 0.0035, TINE_THICKNESS), STEEL)
          for y in (-0.009, -0.003, 0.003, 0.009)],
    ),
)
FORK_CENTRES = np.array([box[0] for box in FORK.boxes])
FORK_HALF_SIZES = np.array([box[1] for box in FORK.boxes]) / 2


def _block(lo, hi, color):
    lo, hi = np.array(lo), np.array(hi)
    return (tuple((lo + hi) / 2), tuple(hi - lo), color)


APPLIANCE = replace(parts.part(
    "microwave", MICROWAVE_ORIGIN, boxes=(
        _block((0.0, -0.196, 0.0), (0.308, 0.214, CAVITY_FLOOR), HOUSING),
        _block((0.0, -0.196, 0.222), (0.308, 0.214, 0.239), HOUSING),
        _block((0.289, -0.196, CAVITY_FLOOR), (0.308, 0.214, 0.222), INSIDE),
        _block((0.0, 0.195, CAVITY_FLOOR), (0.308, 0.214, 0.222), INSIDE),
        _block((0.0, -0.196, CAVITY_FLOOR), (0.308, -0.129, 0.222), HOUSING),
        _block((-0.002, -0.186, 0.167), (0.0, -0.145, 0.197), DARK),
        *[_block((x, 0.210, 0.199), (x + 0.008, 0.215, 0.215), DARK)
          for x in (0.198, 0.214, 0.230, 0.246, 0.262)],
    ),
    cylinders=(((PLATE_CENTER[0], PLATE_CENTER[1], CAVITY_FLOOR + 0.003),
                PLATE_RADIUS, 0.006, (0.73, 0.84, 0.89)),),
    hinges=(
        parts.Hinge("door", DOOR_HINGE, lower=-math.radians(103), upper=0.0, mass=0.66,
                    damping=0.25, friction=0.16, boxes=(
            _block((-0.020, -0.105, 0.044), (-0.011, 0.181, 0.202), DARK),
            _block((-0.032, -0.126, 0.026), (0.0, 0.205, 0.045), DARK),
            _block((-0.032, -0.126, 0.201), (0.0, 0.205, 0.225), DARK),
            _block((-0.032, -0.126, 0.043), (0.0, -0.104, 0.204), DARK),
            _block((-0.032, 0.180, 0.043), (0.0, 0.205, 0.204), DARK),
            (DOOR_HANDLE, (0.016, 0.015, 0.124), STEEL),
            ((-0.037, -0.087, 0.191), (0.024, 0.015, 0.015), STEEL),
            ((-0.037, -0.087, 0.085), (0.024, 0.015, 0.015), STEEL),
        )),
        parts.Hinge("start", BUTTON, axis="X", kind="prismatic", lower=0.0, upper=0.006, rest=-0.001,
                    mass=0.013, rate=450.0, damping=2.0, boxes=((BUTTON, (0.010, 0.034, 0.025), BUTTON_GREEN),)),
    ),
), meta={"decorate": _microwave_visuals})

OBJECTS = [BOWL, FORK, APPLIANCE]


def door_handle(angle):
    dx, dy, dz = np.subtract(DOOR_HANDLE, DOOR_HINGE)
    c, s = math.cos(angle), math.sin(angle)
    return np.add(MICROWAVE_ORIGIN, np.add(DOOR_HINGE, (c * dx - s * dy, s * dx + c * dy, dz)))


def _above_table(position, margin=0.0):
    return all(abs(position[i] - TABLE_CENTER[i]) < TABLE_SIZE[i] / 2 - margin for i in (0, 1))


def _seated(bowl):
    relative = bowl.pos - MICROWAVE_ORIGIN
    return (
        np.linalg.norm(relative[:2] - PLATE_CENTER) <= PLATE_RADIUS - BOWL_RADIUS + 0.004
        and abs(relative[2] - PLATE_TOP) <= 0.007
        and bowl.rotation()[2, 2] >= math.cos(math.radians(8))
        and np.linalg.norm(bowl.lin_vel) < 0.035
    )


def _reset(environment):
    global _trial
    _trial = Trial()
    _status.clear()
    _measurements.clear()
    _draw_egg()
    if "lamp" in _visuals:
        _visuals["lamp"].MakeInvisible()
        for _ in range(2):
            environment.unwrapped.sim.render()


def _step(environment):
    from yam_arena.scene import object_states

    states = object_states(environment, (BOWL.name, FORK.name, APPLIANCE.name))
    bowl, fork, appliance = (states[spec.name] for spec in OBJECTS)
    fingers = parts.finger_points(environment)
    valid = all(np.isfinite(state.pos).all() and np.isfinite(state.quat_xyzw).all() for state in states.values())
    if not valid:
        _trial.failed = True
        return
    tip = bowl.to_local(fork.to_world(FORK_TIP))
    hold = min(np.linalg.norm(fork.pos - p) for p in fingers) < 0.030
    # A fork must be held and its tines presented towards the material.
    tines_down = bowl.rotation()[:, 2] @ fork.rotation()[:, 0] > 0.30
    _trial.egg.stir(tip if hold and tines_down else None)
    if _trial.egg.membrane_open and hold:
        _draw_egg()
    upright = bowl.rotation()[2, 2]
    _status.update(
        closed=abs(appliance.joints["door"]) <= DOOR_CLOSED,
        seated=_seated(bowl),
        fork_outside=fork_outside_cavity(fork.pos, fork.rotation(), FORK_CENTRES, FORK_HALF_SIZES,
                                         CAVITY_LOW, CAVITY_HIGH),
        bowl_on_counter=_above_table(bowl.pos, BOWL_RADIUS) and abs(bowl.pos[2] - TOP) < 0.009
        and upright > math.cos(math.radians(8)) and np.linalg.norm(bowl.lin_vel) < 0.035,
    )
    _measurements.update(
        bowl_xyz=bowl.pos.tolist(), fork_xyz=fork.pos.tolist(), tip_in_bowl=tip.tolist(),
        bowl_tilt_deg=math.degrees(math.acos(float(np.clip(upright, -1, 1)))),
        fork_held=bool(hold), door_deg=math.degrees(appliance.joints["door"]),
        start_travel_mm=1000 * appliance.joints["start"],
    )
    spill = (
        upright < math.cos(math.radians(38)) or bowl.pos[2] < TOP - 0.03 or fork.pos[2] < TOP - 0.03
    )
    _trial.update(pressed=appliance.joints["start"] >= BUTTON_DEPTH, spill=spill,
                  **{key: _status[key] for key in ("closed", "seated", "fork_outside")})
    if "lamp" in _visuals and _trial.start_requested:
        _visuals["lamp"].MakeVisible()


def score(_objects):
    return _trial.score(**_status) if _status else 0.0


def success(objects):
    return score(objects) == 1.0


def progress(_objects):
    return {"mixed": _trial.egg.mixed, "membrane_open": _trial.egg.membrane_open, "yolk_spread": _trial.egg.spread,
            "start_requested": _trial.start_requested, "started": bool(_trial.started),
            "unsafe_or_spilled": bool(_trial.failed),
            **{name: bool(value) for name, value in _status.items()}, **_measurements}
