"""Two bimanual YAM robots face each other across one table and share a gray supply tray in its middle. The tray holds
two RealSense D405 camera modules (silver cases) and two DYNAMIXEL XL430-W250-T servo motors (black cases). Robot A
(at -x) has an orange output tray in front of it, robot B (at +x) a blue one. A kits the two camera modules into the
orange tray, B the two servo motors into the blue tray; both reach into the shared tray.

score(): 0.25 per part whose centre ends in its assigned tray. report(): collision_free (1 unless any cross-station
contact exceeded 0.5 N), the peak force, and each robot's deliveries. An example of the yam_pair task contract.

Run:   inspect-robots run --embodiment yam_pair -E task=task.py --policy yam_pair_scripted --instruction "$INSTR" \\
           --scorer pair_task_score --max-steps 1800
"""

from __future__ import annotations

from yam_pair.task import TABLE_TOP, Box, Layout

TOP = TABLE_TOP
LAYOUT = Layout("facing", separation=0.8)  # A's arms at x -0.4, B's at x +0.4 (scripted.py assumes this layout)
TABLE = {"size": (1.30, 1.35), "center": (0.0, 0.0)}
PHYSICS_HZ = 240
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

PARTS = {"camera_1": (0.025, 0.12, TOP + 0.006 + 0.0115), "camera_2": (0.025, -0.12, TOP + 0.006 + 0.0115),
         "servo_1": (-0.025, 0.12, TOP + 0.006 + 0.017), "servo_2": (-0.025, -0.12, TOP + 0.006 + 0.017)}
DESTINATIONS = {"a": (-0.20, 0.0), "b": (0.20, 0.0)}
OWNER = {"camera": "a", "servo": "b"}
CONTACT_THRESHOLD_N = 0.5
ORANGE, BLUE, GRAY, RIM = (1.0, 0.55, 0.12), (0.15, 0.42, 0.92), (0.80, 0.80, 0.80), (0.62, 0.62, 0.62)


def _trays() -> list:
    out = [Box("supply_base", pos=(0, 0, TOP + 0.003), size=(0.19, 0.40, 0.006), color=GRAY)]
    for side, x in (("near", -0.098), ("far", 0.098)):
        out.append(Box(f"supply_{side}", pos=(x, 0, TOP + 0.006), size=(0.006, 0.406, 0.012), color=RIM))
    for side, y in (("left", 0.203), ("right", -0.203)):
        out.append(Box(f"supply_{side}", pos=(0, y, TOP + 0.006), size=(0.19, 0.006, 0.012), color=RIM))
    for s, (x, y) in DESTINATIONS.items():
        color = ORANGE if s == "a" else BLUE
        out.append(Box(f"{s}_tray_base", pos=(x, y, TOP + 0.003), size=(0.19, 0.57, 0.006), color=color))
        for side, yy in (("left", 0.288), ("right", -0.288)):
            out.append(Box(f"{s}_tray_{side}", pos=(x, yy, TOP + 0.0125), size=(0.196, 0.006, 0.025), color=color))
        for side, xx in (("near", x - 0.098), ("far", x + 0.098)):
            out.append(Box(f"{s}_tray_{side}", pos=(xx, 0, TOP + 0.0125), size=(0.006, 0.57, 0.025), color=color))
    return out


OBJECTS = _trays()


def isaac_objects(layout):
    """The four parts with the prototype's exact physics (contact offset 1 mm, friction 1.0/0.8, product masses)."""
    import isaaclab.sim as sim
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from isaaclab_physx.sim.schemas import PhysxCollisionPropertiesCfg, PhysxRigidBodyPropertiesCfg

    out = []
    for name, pos in PARTS.items():
        camera = name.startswith("camera")
        common = dict(rigid_props=PhysxRigidBodyPropertiesCfg(),
                      mass_props=sim.MassPropertiesCfg(mass=0.060 if camera else 0.0572),
                      collision_props=PhysxCollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
                      physics_material=sim.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=0.8),
                      visual_material=sim.PreviewSurfaceCfg(
                          diffuse_color=(0.62, 0.64, 0.65) if camera else (0.055, 0.06, 0.065)))
        spawn = sim.CuboidCfg(size=(0.042, 0.042, 0.023) if camera else (0.0285, 0.0465, 0.034), **common)
        out.append(Object(name=name, object_type=ObjectType.RIGID, spawner_cfg=spawn, initial_pose=Pose(position_xyz=pos)))
    return out


def on_build(env) -> None:
    """Recognizable product detail (visual only) riding on each rigid case: the prototype's decorate_hardware()."""
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    stage = omni.usd.get_context().get_stage()

    def material(name, color, metal=0.0):
        path = "/World/HardwareMaterials/" + name
        mat = UsdShade.Material.Define(stage, path)
        s = UsdShade.Shader.Define(stage, path + "/shader")
        s.CreateIdAttr("UsdPreviewSurface")
        s.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        s.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.3)
        s.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        mat.CreateSurfaceOutput().ConnectToSource(s.ConnectableAPI(), "surface")
        return mat

    black, glass, metal = material("black", (0.015, 0.02, 0.025)), material("glass", (0.015, 0.065, 0.095), 0.6), \
        material("metal", (0.62, 0.65, 0.68), 0.8)

    def box(root, name, pos, size, mat):
        o = UsdGeom.Cube.Define(stage, root + "/" + name)
        o.CreateSizeAttr(1.0)
        o.AddTranslateOp().Set(Gf.Vec3d(*pos))
        o.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdShade.MaterialBindingAPI.Apply(o.GetPrim()).Bind(mat)

    def disk(root, name, pos, radius, height, mat):
        o = UsdGeom.Cylinder.Define(stage, root + "/" + name)
        o.CreateRadiusAttr(radius)
        o.CreateHeightAttr(height)
        o.AddTranslateOp().Set(Gf.Vec3d(*pos))
        UsdShade.MaterialBindingAPI.Apply(o.GetPrim()).Bind(mat)

    for name in PARTS:
        root = "/World/envs/env_0/" + name
        if not stage.GetPrimAtPath(root).IsValid():
            raise RuntimeError("Missing hardware body " + root)
        if name.startswith("camera"):
            box(root, "front_plate", (0, 0, 0.0116), (0.037, 0.037, 0.0003), black)
            for i, x in enumerate((-0.010, 0.010)):
                disk(root, f"lens_rim_{i}", (x, 0, 0.01195), 0.0045, 0.0005, metal)
                disk(root, f"stereo_lens_{i}", (x, 0, 0.01225), 0.0037, 0.0002, glass)
            box(root, "side_usb_socket", (0, -0.0211, 0), (0.009, 0.0003, 0.003), black)
        else:
            disk(root, "output_hub", (0, 0.004, 0.0172), 0.0102, 0.0005, metal)
            disk(root, "output_spline", (0, 0.004, 0.01755), 0.004, 0.0003, black)
            for i, (x, y) in enumerate(((-0.010, -0.019), (0.010, -0.019), (-0.010, 0.019), (0.010, 0.019))):
                disk(root, f"case_screw_{i}", (x, y, 0.0173), 0.0015, 0.0006, metal)
            box(root, "connector", (0, -0.0234, 0), (0.011, 0.0004, 0.006), black)


# -- scoring --------------------------------------------------------------------------------------------------------------
def placed(objects) -> dict[str, bool]:
    """Each part: its centre inside its owner's tray (the prototype's bounds)."""
    out = {}
    for name in PARTS:
        x, y, z = objects[name].pos
        dx, dy = DESTINATIONS[OWNER[name.split("_")[0]]]
        out[name] = bool(abs(x - dx) < 0.075 and abs(y - dy) < 0.265 and TOP < z < TOP + 0.08)
    return out


def score(objects, pair) -> float:
    return 0.25 * sum(placed(objects).values())


def success(objects, pair) -> bool:
    return all(placed(objects).values()) and all(objects[n].resting() for n in PARTS)


def report(objects, pair) -> dict:
    done = placed(objects)
    return {
        "collision_free": 0.0 if pair.contact_peak_n > CONTACT_THRESHOLD_N else 1.0,
        "cross_contact_peak_n": pair.contact_peak_n,
        "a_delivered": sum(v for k, v in done.items() if k.startswith("camera")),
        "b_delivered": sum(v for k, v in done.items() if k.startswith("servo")),
        "dropped": sum(objects[n].pos[2] < TOP - 0.05 for n in PARTS),
    }


def demo(layout):
    """Each station's own closed-loop controller (scripted.py), fed only its own 14-D state."""
    from scripted import ScriptedController

    return {"a": ScriptedController("a"), "b": ScriptedController("b")}
