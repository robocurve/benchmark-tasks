"""A counter-height service lift (a dumbwaiter, as in a restaurant), written as plain-text USD (no pxr needed).

Lift frame: origin on the table, at the middle of the opening's width, on the robot-side face of the shaft's front wall;
+x into the shaft, +z up. Two prims stand at that origin:

  lift_walls  static colliders: the front wall (sill, two jambs, lintel), the shaft's side and back walls, and the call
              console beside the opening, with the button's bezel and the status lamp.
  lift        one articulation with a fixed root link and three moving links:
              car     prismatic along +z, a floor `sill_height` above the table, two side walls, a back wall and a roof;
              door    prismatic along +z, a glass pane in a steel frame just in front of the front wall. Joint position 0
                      is fully open and `door_q_closed` is closed, with its bottom 1 mm above the table. Zero must be
                      open: PhysX starts every joint at position 0 whatever the authored pose.
              button  prismatic along -z: a green push button on the console's top, pressed straight down.

The car and the door are counterweighted (no gravity on them). Their drives are force-limited, so a cord or an object in
their way stops them instead of being pushed through. The walls are a separate prim because objects touching colliders
inside the articulation's prim tree that belong to no moving link were thrown out at metres per second in the first step.
"""

from __future__ import annotations

import os
from dataclasses import dataclass


@dataclass(frozen=True)
class LiftSpec:
    opening_width: float = 0.50  # along y; the arms' start pose leaves |y| < 0.28 free at the lift's front
    opening_height: float = 0.40
    sill_height: float = 0.013  # the car floor's top above the table
    wall: float = 0.02  # front wall's thickness
    side_wall: float = 0.012  # thickness of the shaft's side and back walls
    jamb_width: float = 0.02  # front wall beside the opening, each side
    running_gap: float = 0.012  # front wall's inner face to the car floor's front edge
    car_depth: float = 0.30  # inside, along x
    car_height: float = 0.42  # inside
    car_wall: float = 0.010
    car_floor: float = 0.012  # floor thickness, so the floor's underside is 1 mm above the table
    side_gap: float = 0.006  # car's side and back walls to the shaft's walls
    shaft_top: float = 1.25
    car_travel: float = 0.70
    door_thickness: float = 0.012
    door_gap: float = 0.002  # door's back face to the front wall
    door_overlap: float = 0.008  # door wider than the opening on each side: wider, it lands on the fingers of arms at their start pose
    door_bottom: float = 0.001  # door's bottom above the table when closed
    # The call console: a box on the table to the right of the opening, in the lift frame.
    console_center: tuple[float, float] = (0.0, -0.34)
    console_size: tuple[float, float, float] = (0.13, 0.09, 0.06)
    button_offset: float = -0.025  # button's x from the console's centre (towards the robot)
    lamp_offset: float = 0.03  # lamp's x from the console's centre (away from the robot)
    button_radius: float = 0.014
    button_depth: float = 0.010
    button_top: float = 0.019  # cap's face above the console's top; pushed fully in, the cap stays clear of the console
    button_travel: float = 0.008
    button_pressed: float = 0.004  # counts as pressed when pushed in this far
    # The car moves once the door is this close to closed. A 7 mm cord rising over the sill holds the door 1.5 cm up, which
    # must not stop the car; the blender, 6.6 cm across or more, does.
    interlock_gap: float = 0.02
    door_timeout: float = 4.0  # [s] a door not closed this long after the press means the lift is broken
    jam_lag: float = 0.08  # [m] the car this far behind its target means it is jammed
    car_speed: float = 0.2  # [m/s]
    car_mass: float = 1.0  # light car and door: heavier ones pinching light cord links blew the solver up
    door_mass: float = 0.3
    button_mass: float = 0.02
    frame_mass: float = 30.0
    friction: float = 0.5

    @property
    def platform_edge(self) -> float:
        """x of the car floor's front edge (lift frame)."""
        return self.wall + self.running_gap

    @property
    def door_open(self) -> float:
        """Door travel from closed to fully open (open: its bottom 1 cm above the opening's top)."""
        return self.sill_height + self.opening_height + 0.01 - self.door_bottom

    @property
    def door_q_closed(self) -> float:
        return -self.door_open

    @property
    def door_size(self) -> tuple[float, float, float]:
        return (self.door_thickness, self.opening_width + 2 * self.door_overlap,
                self.sill_height + self.opening_height + self.door_overlap - self.door_bottom)

    @property
    def button_xy(self) -> tuple[float, float]:
        return (self.console_center[0] + self.button_offset, self.console_center[1])

    @property
    def button_face(self) -> float:
        """z of the cap's face at rest (lift frame)."""
        return self.console_size[2] + self.button_top


@dataclass(frozen=True)
class Box:
    name: str
    center: tuple[float, float, float]  # in the link's frame
    size: tuple[float, float, float]
    color: tuple[float, float, float]


LAMP_COLORS = {"open": (0.05, 0.85, 0.15), "moving": (1.0, 0.55, 0.0), "broken": (0.95, 0.03, 0.03)}
LAMP_RADIUS = 0.012
STEEL = (0.42, 0.44, 0.47)
CAR = (0.86, 0.86, 0.84)
PLATE = (0.78, 0.79, 0.80)
DARK = (0.12, 0.12, 0.13)


def lamp_state(s: LiftSpec, phase: int, car_q: float, car_target: float, s_since_press: float) -> str:
    """What the status lamp shows. phase: 0 door open and waiting, 1 door closing, 2 car rising."""
    if phase == 0:
        return "open"
    if phase == 1:
        return "broken" if s_since_press > s.door_timeout else "moving"
    if car_target - car_q > s.jam_lag:
        return "broken"
    return "off" if car_q >= s.car_travel - 0.01 else "moving"


def set_lamp(stage, walls_path: str, state: str) -> None:
    """Show the lit lens of `state` under the walls' prim and hide the others (no lit lens: the dark one shows)."""
    from pxr import UsdGeom

    for name in LAMP_COLORS:
        prim = stage.GetPrimAtPath(f"{walls_path}/lamp_{name}")
        if not prim.IsValid():
            continue
        lens = UsdGeom.Imageable(prim)
        lens.MakeVisible() if name == state else lens.MakeInvisible()


def _box(name, x0, x1, y0, y1, z0, z1, color) -> Box:
    return Box(name, (0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * (z0 + z1)), (x1 - x0, y1 - y0, z1 - z0), color)


def link_origins(s: LiftSpec) -> dict[str, tuple[float, float, float]]:
    """Each link's origin in the lift frame at joint position 0 (car down, door open, button up)."""
    bx, by = s.button_xy
    return {"frame": (0.0, 0.0, 0.0), "car": (s.platform_edge, 0.0, s.sill_height),
            "door": (-s.door_gap - 0.5 * s.door_thickness, 0.0, s.door_bottom + s.door_open),
            "button": (bx, by, s.console_size[2])}


def lift_boxes(s: LiftSpec) -> dict[str, list[Box]]:
    """The colliders of each link, as boxes in the link's frame (also used to measure how deep things sink)."""
    W2, H, t, sh = 0.5 * s.opening_width, s.opening_height, s.wall, s.sill_height
    cw, cd = s.car_wall, s.car_depth
    car_out = W2 + cw
    shaft_in = car_out + s.side_gap
    shaft_back = s.platform_edge + cd + cw + s.side_gap
    front_half = W2 + s.jamb_width
    zt = s.shaft_top
    cx, cy = s.console_center
    sx, sy, sz = s.console_size
    frame = [
        _box("sill_wall", 0.0, t, -front_half, front_half, 0.0, sh, STEEL),
        _box("jamb_left", 0.0, t, W2, front_half, sh, zt, STEEL),
        _box("jamb_right", 0.0, t, -front_half, -W2, sh, zt, STEEL),
        _box("lintel", 0.0, t, -W2, W2, sh + H, zt, STEEL),
        _box("side_left", t, shaft_back, shaft_in, shaft_in + s.side_wall, 0.0, zt, STEEL),
        _box("side_right", t, shaft_back, -shaft_in - s.side_wall, -shaft_in, 0.0, zt, STEEL),
        _box("back", shaft_back, shaft_back + s.side_wall, -shaft_in - s.side_wall, shaft_in + s.side_wall, 0.0, zt, STEEL),
        _box("console", cx - 0.5 * sx, cx + 0.5 * sx, cy - 0.5 * sy, cy + 0.5 * sy, 0.0, sz, PLATE),
    ]
    car = [  # car frame: origin at the middle of the floor's front edge, on the floor's top
        _box("floor", 0.0, cd + cw, -car_out, car_out, -s.car_floor, 0.0, CAR),
        _box("wall_left", 0.0, cd, W2, car_out, 0.0, s.car_height, CAR),
        _box("wall_right", 0.0, cd, -car_out, -W2, 0.0, s.car_height, CAR),
        _box("wall_back", cd, cd + cw, -car_out, car_out, 0.0, s.car_height, CAR),
        _box("roof", 0.0, cd + cw, -car_out, car_out, s.car_height, s.car_height + cw, CAR),
    ]
    dx, dy, dz = s.door_size
    door = [Box("shutter", (0.0, 0.0, 0.5 * dz), (dx, dy, dz), (0.22, 0.33, 0.50))]
    return {"frame": frame, "car": car, "door": door}


def lift_drive_limits() -> dict:
    """Implicit PD drives for the lift's joints: stiffness [N/m], damping [N s/m], force limit [N]."""
    return {
        "car_joint": {"stiffness": 4000.0, "damping": 800.0, "effort": 150.0},
        # Pushes down with its force limit wherever it stops (target below the closed limit).
        "door_joint": {"stiffness": 300.0, "damping": 40.0, "effort": 8.0},
        "button_joint": {"stiffness": 300.0, "damping": 3.0, "effort": 20.0},
    }


def _cube(b: Box, material: str) -> str:
    cx, cy, cz = b.center
    sx, sy, sz = b.size
    r, g, bl = b.color
    return f"""
        def Cube "{b.name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = <{material}>
            double size = 1
            color3f[] primvars:displayColor = [({r:.3f}, {g:.3f}, {bl:.3f})]
            double3 xformOp:translate = ({cx:.6f}, {cy:.6f}, {cz:.6f})
            float3 xformOp:scale = ({sx:.6f}, {sy:.6f}, {sz:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}"""


def _car_link(s: LiftSpec, origin, boxes: list[Box], root: str) -> str:
    cubes = "".join(_cube(b, f"/{root}/PhysicsMaterial") for b in boxes)
    return f"""
    def Xform "car" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
    )
    {{
        float physics:mass = {s.car_mass:.4f}
        bool physxRigidBody:disableGravity = 1
        double3 xformOp:translate = ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]
{cubes}
    }}
"""


def _door_link(s: LiftSpec, origin, root: str) -> str:
    """The door: one solid box to collide with (unrendered), and what is seen: a glass pane in a steel frame."""
    dx, dy, dz = s.door_size
    bar = 0.03

    def visual(name, cy, cz, sy, sz):
        return f"""
        def Cube "{name}"
        {{
            double size = 1
            color3f[] primvars:displayColor = [(0.30, 0.32, 0.35)]
            double3 xformOp:translate = (0, {cy:.6f}, {cz:.6f})
            float3 xformOp:scale = ({dx:.6f}, {sy:.6f}, {sz:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}"""

    frame = (visual("frame_bottom", 0.0, 0.5 * bar, dy, bar) + visual("frame_top", 0.0, dz - 0.5 * bar, dy, bar)
             + visual("frame_left", 0.5 * (dy - bar), 0.5 * dz, bar, dz) + visual("frame_right", -0.5 * (dy - bar), 0.5 * dz, bar, dz))
    return f"""
    def Xform "door" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
    )
    {{
        float physics:mass = {s.door_mass:.4f}
        bool physxRigidBody:disableGravity = 1
        double3 xformOp:translate = ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def Cube "shutter" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = </{root}/PhysicsMaterial>
            double size = 1
            token visibility = "invisible"
            double3 xformOp:translate = (0, 0, {0.5 * dz:.6f})
            float3 xformOp:scale = ({dx:.6f}, {dy:.6f}, {dz:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}
{frame}

        def Cube "glass" (
            prepend apiSchemas = ["MaterialBindingAPI"]
        )
        {{
            rel material:binding = </{root}/GlassMaterial>
            double size = 1
            double3 xformOp:translate = (0, 0, {0.5 * dz:.6f})
            float3 xformOp:scale = (0.004, {dy - 2 * bar:.6f}, {dz - 2 * bar:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}
    }}
"""


def _glass_material(root: str) -> str:
    return f"""
    def Material "GlassMaterial"
    {{
        token outputs:mdl:surface.connect = </{root}/GlassMaterial/Shader.outputs:out>
        token outputs:mdl:displacement.connect = </{root}/GlassMaterial/Shader.outputs:out>
        token outputs:mdl:volume.connect = </{root}/GlassMaterial/Shader.outputs:out>

        def Shader "Shader"
        {{
            uniform token info:implementationSource = "sourceAsset"
            uniform asset info:mdl:sourceAsset = @OmniGlass.mdl@
            uniform token info:mdl:sourceAsset:subIdentifier = "OmniGlass"
            color3f inputs:glass_color = (0.90, 0.96, 0.98)
            float inputs:glass_ior = 1.1
            float inputs:frosting_roughness = 0
            bool inputs:thin_walled = 1
            token outputs:out
        }}
    }}
"""


def _button_link(s: LiftSpec, origin, root: str) -> str:
    """The push button: a green cylinder along z (its collider) with a slightly domed face."""
    z0 = s.button_top - s.button_depth
    return f"""
    def Xform "button" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI"]
    )
    {{
        float physics:mass = {s.button_mass:.4f}
        bool physxRigidBody:disableGravity = 1
        double3 xformOp:translate = ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def Cylinder "cap" (
            prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
        )
        {{
            rel material:binding:physics = </{root}/PhysicsMaterial>
            uniform token axis = "Z"
            double height = {s.button_depth:.6f}
            double radius = {s.button_radius:.6f}
            color3f[] primvars:displayColor = [(0.08, 0.62, 0.18)]
            double3 xformOp:translate = (0, 0, {z0 + 0.5 * s.button_depth:.6f})
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}

        def Sphere "dome"
        {{
            double radius = {s.button_radius - 0.0005:.6f}
            color3f[] primvars:displayColor = [(0.12, 0.72, 0.24)]
            double3 xformOp:translate = (0, 0, {s.button_top - 0.001:.6f})
            float3 xformOp:scale = (1, 1, 0.22)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }}
    }}
"""


def _console_top(s: LiftSpec, root: str) -> str:
    """Visual only: the dark bezels round the button and the lamp, and the status lamp's lenses, on the console's top."""
    bx, by = s.button_xy
    lx, top = s.console_center[0] + s.lamp_offset, s.console_size[2]
    bezel = 0.003

    def ring(name, x, r):
        return f"""
    def Cylinder "{name}"
    {{
        uniform token axis = "Z"
        double height = {bezel:.6f}
        double radius = {r:.6f}
        color3f[] primvars:displayColor = [{DARK}]
        double3 xformOp:translate = ({x:.6f}, {by:.6f}, {top + 0.5 * bezel:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}"""

    parts = [ring("button_bezel", bx, s.button_radius + 0.005), ring("lamp_bezel", lx, LAMP_RADIUS + 0.003)]
    z = top + bezel
    for state, color in LAMP_COLORS.items():
        visible = "" if state == "open" else '\n        token visibility = "invisible"'
        emissive = tuple(1.2 * c for c in color)
        parts.append(f"""
    def Material "lamp_{state}_material"
    {{
        token outputs:surface.connect = </{root}/lamp_{state}_material/Shader.outputs:surface>

        def Shader "Shader"
        {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor = ({color[0]}, {color[1]}, {color[2]})
            color3f inputs:emissiveColor = ({emissive[0]}, {emissive[1]}, {emissive[2]})
            float inputs:roughness = 0.3
            token outputs:surface
        }}
    }}

    def Sphere "lamp_{state}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{{visible}
        rel material:binding = </{root}/lamp_{state}_material>
        double radius = {LAMP_RADIUS:.6f}
        double3 xformOp:translate = ({lx:.6f}, {by:.6f}, {z:.6f})
        float3 xformOp:scale = (1, 1, 0.45)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}""")
    parts.append(f"""
    def Sphere "lamp_off"
    {{
        double radius = {LAMP_RADIUS - 0.0005:.6f}
        color3f[] primvars:displayColor = [(0.10, 0.10, 0.11)]
        double3 xformOp:translate = ({lx:.6f}, {by:.6f}, {z:.6f})
        float3 xformOp:scale = (1, 1, 0.42)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}""")
    return "".join(parts)


def _prismatic(root: str, name: str, body1: str, axis: str, pos0, lower: float, upper: float, drive: dict) -> str:
    # A joint without a DriveAPI gets no drive in PhysX, and the actuator's gains then have no effect.
    return f"""
    def PhysicsPrismaticJoint "{name}" (
        prepend apiSchemas = ["PhysicsDriveAPI:linear"]
    )
    {{
        uniform token drive:linear:physics:type = "force"
        float drive:linear:physics:stiffness = {drive["stiffness"]:.4f}
        float drive:linear:physics:damping = {drive["damping"]:.4f}
        float drive:linear:physics:maxForce = {drive["effort"]:.4f}
        float drive:linear:physics:targetPosition = 0
        rel physics:body0 = </{root}/frame>
        rel physics:body1 = </{root}/{body1}>
        uniform token physics:axis = "{axis}"
        point3f physics:localPos0 = ({pos0[0]:.6f}, {pos0[1]:.6f}, {pos0[2]:.6f})
        quatf physics:localRot0 = (1, 0, 0, 0)
        point3f physics:localPos1 = (0, 0, 0)
        quatf physics:localRot1 = (1, 0, 0, 0)
        float physics:lowerLimit = {lower:.6f}
        float physics:upperLimit = {upper:.6f}
    }}
"""


def _material(s: LiftSpec) -> str:
    return f"""
    def Material "PhysicsMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI"]
    )
    {{
        float physics:staticFriction = {s.friction:.4f}
        float physics:dynamicFriction = {s.friction:.4f}
        float physics:restitution = 0
    }}"""


def _write(path: str, text: str) -> str:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)
    return path


def write_lift_usda(path: str, s: LiftSpec = LiftSpec()) -> str:
    """Write the lift articulation (root link fixed to the world, no colliders on it)."""
    root = "Lift"
    boxes, origins = lift_boxes(s), link_origins(s)
    links = (
        f"""
    def Xform "frame" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
    )
    {{
        float physics:mass = {s.frame_mass:.4f}
        float3 physics:diagonalInertia = (1, 1, 1)
    }}
"""
        + _car_link(s, origins["car"], boxes["car"], root)
        + _door_link(s, origins["door"], root)
        + _button_link(s, origins["button"], root)
    )
    drives = lift_drive_limits()
    joints = (
        _prismatic(root, "car_joint", "car", "Z", origins["car"], 0.0, s.car_travel, drives["car_joint"])
        + _prismatic(root, "door_joint", "door", "Z", origins["door"], s.door_q_closed, 0.0, drives["door_joint"])
        + _prismatic(root, "button_joint", "button", "Z", origins["button"], -s.button_travel, 0.0, drives["button_joint"])
    )
    return _write(path, f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}" (
    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]
)
{{
    bool physxArticulation:enabledSelfCollisions = 0
    int physxArticulation:solverPositionIterationCount = 32
    int physxArticulation:solverVelocityIterationCount = 1
{_material(s)}
{_glass_material(root)}
{links}
    def PhysicsFixedJoint "world_joint"
    {{
        rel physics:body1 = </{root}/frame>
    }}
{joints}}}
""")


def write_lift_walls_usda(path: str, s: LiftSpec = LiftSpec()) -> str:
    """Write the shaft's walls and the console as static colliders, with the console's bezels and lamp, as their own prim."""
    root = "LiftWalls"
    walls = "".join(_cube(b, f"/{root}/PhysicsMaterial") for b in lift_boxes(s)["frame"])
    return _write(path, f"""#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}"
{{{_material(s)}
{walls}
{_console_top(s, root)}
}}
""")
