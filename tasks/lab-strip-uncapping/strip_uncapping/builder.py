"""Native tube carrier, connected caps and separate pronged extraction tool."""

from . import config as C
from .usd_scene import Writer

BLUE = (0.055, 0.32, 0.47)
PP = (0.69, 0.79, 0.82)
CAP_COLOR = (0.88, 0.70, 0.22)
TOOL_COLOR = (0.115, 0.13, 0.16)
WATER = (0.042, 0.48, 0.71)


def counterhold_rack(w):
    stage = w.stage("rack")
    body = w.body(stage, "/Asset/Carrier", C.RACK_MASS, ccd=True)
    w.Phy.MassAPI.Get(stage, body).CreateCenterOfMassAttr(w.Gf.Vec3f(*C.RACK_COM))
    size_x, size_y, _ = C.RACK_SIZE
    w.box(stage, body + "/Base", (size_x, size_y, 0.009), (0, 0, 0.0045), BLUE)
    for j, x in enumerate((-0.033, 0.033)):
        w.box(stage, f"{body}/Side{j}", (0.009, size_y, 0.018), (x, 0, 0.014), BLUE)
    for j, y in enumerate((-0.052, 0.052)):
        w.box(stage, f"{body}/End{j}", (0.068, 0.013, 0.026), (0, y, 0.016), BLUE)
    for j, x in enumerate((-0.016, 0.016)):
        w.box(stage, f"{body}/Retainer{j}", (0.022, 0.097, 0.004), (x, 0, 0.022), BLUE)
    for j, y in enumerate(C.WELL_Y):
        cup = f"{body}/Tube{j}"
        parent = w.Geom.Xform.Define(stage, cup)
        parent.AddTranslateOp().Set(w.Gf.Vec3d(0.0, y, 0.0))
        w.cylinder(stage, cup + "/LowerWall", 0.00302, 0.010, (0.0, 0.0, 0.0195), PP)
        segments = 12
        w.ring(
            stage,
            cup + "/OpenBore",
            0.002925,
            0.00046,
            C.RIM_Z - C.WELL_BOTTOM,
            (C.RIM_Z + C.WELL_BOTTOM) / 2.0,
            PP,
            n=segments,
        )
        w.cylinder(
            stage,
            cup + "/Water",
            C.WELL_RADIUS - 0.00032,
            1.0,
            (0.0, 0.0, 0.0),
            WATER,
            collide=False,
        )
        water = w.Geom.Xformable(stage.GetPrimAtPath(cup + "/Water"))
        water.AddScaleOp().Set(
            w.Gf.Vec3f(1.0, 1.0, C.WATER_UL * 1.0e-9 / (3.14159265 * C.WELL_RADIUS**2))
        )
        water.GetOrderedXformOps()[0].Set(
            w.Gf.Vec3d(
                0.0,
                0.0,
                C.WELL_BOTTOM
                + C.WATER_UL * 1.0e-9 / (2 * 3.14159265 * C.WELL_RADIUS**2),
            )
        )
    return stage


def caps(w):
    stage = w.stage("caps")
    body = w.body(stage, "/Asset/Strip", C.CAP_MASS, ccd=True)
    w.Phy.MassAPI.Get(stage, body).CreateCenterOfMassAttr(w.Gf.Vec3f(*C.CAP_COM))
    for j, y in enumerate(C.WELL_Y):
        w.cylinder(
            stage, f"{body}/Flange{j}", 0.00320, 0.00145, (0, y, -0.00148), CAP_COLOR
        )
        w.cylinder(
            stage,
            f"{body}/Plug{j}",
            0.00239,
            C.FIT_DEPTH,
            (0, y, -(0.003 + C.FIT_DEPTH / 2)),
            CAP_COLOR,
        )
        w.cylinder(stage, f"{body}/Top{j}", 0.0030, 0.001, (0, y, -0.00028), CAP_COLOR)
    for j, y in enumerate((C.WELL_Y[:-1] + C.WELL_Y[1:]) / 2):
        w.box(
            stage,
            f"{body}/Web{j}",
            (0.0072, 0.0045, 0.0012),
            (0, y, 0.00035),
            CAP_COLOR,
        )
    for j, y in enumerate((-0.0357, 0.0357)):
        w.box(
            stage,
            f"{body}/EndTab{j}",
            (0.006, 0.006, 0.0012),
            (0, y, -0.0010),
            CAP_COLOR,
        )
    return stage


def tool(w):
    stage = w.stage("tool")
    body = w.body(stage, "/Asset/Tool", C.TOOL_MASS, ccd=True)
    for j, y in enumerate((-0.0158, 0.0158)):
        w.box(stage, f"{body}/Rail{j}", (0.074, 0.0084, 0.012), (0, y, 0), TOOL_COLOR)
    for j, x in enumerate((-0.0315, 0.0275)):
        w.box(
            stage, f"{body}/Crosspiece{j}", (0.011, 0.040, 0.012), (x, 0, 0), TOOL_COLOR
        )
    for j, y in enumerate(C.TOOTH_POSITIONS):
        w.box(
            stage,
            f"{body}/Heel{j}",
            (0.009, 0.003, 0.004),
            (0.035, y, -0.0038),
            TOOL_COLOR,
        )
        w.box(
            stage,
            f"{body}/Prong{j}",
            (0.017, 0.0017, C.TOOTH_T),
            (0.042, y, C.TIP_FROM_TOOL[2]),
            TOOL_COLOR,
        )
    return stage


def dish(w):
    stage = w.stage("dish")
    length, width, height = C.DISH_SIZE
    color = (0.18, 0.24, 0.26)
    w.box(stage, "/Asset/Bottom", (length, width, 0.004), (0, 0, 0.002), color)
    for j, x in enumerate((-length / 2 + 0.002, length / 2 - 0.002)):
        w.box(
            stage, f"/Asset/WallX{j}", (0.004, width, height), (x, 0, height / 2), color
        )
    for j, y in enumerate((-width / 2 + 0.002, width / 2 - 0.002)):
        w.box(
            stage,
            f"/Asset/WallY{j}",
            (length, 0.004, height),
            (0, y, height / 2),
            color,
        )
    return stage


def create_assets(directory):
    w = Writer(directory)
    assets = []
    for name, body_type, factory, position in (
        ("rack", "rigid", counterhold_rack, C.RACK),
        ("caps", "rigid", caps, C.RACK + [0, 0, C.CAP_Z]),
        ("tool", "rigid", tool, C.TOOL),
        ("dish", "base", dish, C.DISH),
    ):
        assets.append(
            {
                "name": name,
                "type": body_type,
                "position": position.tolist(),
                "file": w.save(factory(w)),
            }
        )
    w.inventory()
    return {"assets": assets, "settle_actions": 11}
