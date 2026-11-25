"""Compile the local physical fixture into a self-contained USDA layer."""
import json
import hashlib
import math
from pathlib import Path
import re


TABLE_TOP = 0.7475


def gripper_command(opening_m, finger_open):
    # YAM's signed finger joint is the displacement of one of TWO mirrored pads.
    span = 2.0 * abs(float(finger_open))
    if not span > 0:
        raise ValueError("Invalid YAM jaw travel")
    return max(0., min(1., float(opening_m)/span))


def load_fixture():
    return json.loads(Path(__file__).with_name("fixture.json").read_text(encoding="utf-8"))


def identifier(raw):
    if not isinstance(raw, str) or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]{0,47}", raw):
        raise ValueError("Invalid fixture identifier")
    return raw


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("Expected a finite number")
    return format(value, ".10g")


def triple(values):
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise ValueError("Expected a 3-vector")
    return "(" + ", ".join(number(value) for value in values) + ")"


def surface(actor):
    name = identifier(actor["name"])
    rgb = triple(actor["rgb"])
    friction = number(actor["friction"])
    return f'''def Material "{name}" ( prepend apiSchemas = ["PhysicsMaterialAPI"] )
    {{
        token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>
        float physics:dynamicFriction = {friction}
        float physics:staticFriction = {friction}
        float physics:restitution = 0
        def Shader "Surface"
        {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor = {rgb}
            float inputs:roughness = 0.65
            token outputs:surface
        }}
    }}'''


def mark_material(color):
    return "paint_"+hashlib.sha256(triple(color).encode("ascii")).hexdigest()[:16]


def ink_surface(color):
    name=mark_material(color)
    return f'''def Material "{name}"
    {{
        token outputs:surface.connect = </Fixture/MarkMaterials/{name}/Surface.outputs:surface>
        def Shader "Surface"
        {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor = {triple(color)}
            float inputs:roughness = 0.75
            token outputs:surface
        }}
    }}'''


def primitive(i, part, name, color, visual=False):
    kind = {"box": "Cube", "cylinder": "Cylinder", "sphere": "Sphere", "capsule": "Capsule"}[part["type"]]
    size = part["size"]
    if min(size) <= 0:
        raise ValueError("Positive collider dimensions required")
    if "corner_m" in part.get("optical_surface", {}):
        from .rounded_panel import geometry as lens_geometry
        kind = "Mesh"
        geometry = lens_geometry(size, part["optical_surface"]["corner_m"])
        scale = [1, 1, 1]
    elif kind == "Cube":
        geometry = "double size = 1"
        scale = size
    elif kind == "Sphere":
        if max(size) - min(size) > 1e-7:
            raise ValueError("Sphere requires equal diameters")
        geometry = f"double radius = {number(size[0]/2)}"
        scale = [1, 1, 1]
    else:
        if abs(size[1] - size[0]) > 1e-7:
            raise ValueError("Circular collider requires equal X/Y diameter")
        height = size[2] - size[0] if kind == "Capsule" else size[2]
        if height <= 0:
            raise ValueError("Invalid cylinder/capsule height")
        geometry = f'double height = {number(height)}\n            double radius = {number(size[0]/2)}\n            uniform token axis = "Z"'
        scale = [1, 1, 1]
    if visual:
        label,schemas="mark_"+str(i),'"MaterialBindingAPI"'
        binding=f'rel material:binding = </Fixture/MarkMaterials/{mark_material(color)}>'
    else:
        label,schemas="part_"+str(i),'"MaterialBindingAPI", "PhysicsCollisionAPI", "PhysxCollisionAPI"'
        if "corner_m" in part.get("optical_surface", {}):
            schemas += ', "PhysicsMeshCollisionAPI"'
        binding=f'''float physxCollision:contactOffset = 0.001
            float physxCollision:restOffset = 0
            rel material:binding = </Fixture/Materials/{name}>
            rel material:binding:physics = </Fixture/Materials/{name}>'''
    if "optical_surface" in part and not visual:
        from .optics import bind_visible_surface
        binding = bind_visible_surface(binding, name, i)
        color = part["optical_surface"]["rgb"]
    return f'''def {kind} "{label}" ( prepend apiSchemas = [{schemas}] )
        {{
            {geometry}
            {binding}
            color3f[] primvars:displayColor = [{triple(color)}]
            double3 xformOp:translate = {triple(part["offset"])}
            float3 xformOp:rotateXYZ = {triple(part["euler_deg"])}
            float3 xformOp:scale = {triple(scale)}
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
        }}'''


def body(actor):
    name = identifier(actor["name"])
    x, y, z = actor["position"]
    solids = "\n".join(primitive(i, p, name, actor["rgb"]) for i, p in enumerate(actor["parts"]))
    marks = "\n".join(primitive(i, p, name, p["rgb"], visual=True) for i,p in enumerate(actor.get("visuals",[])))
    return f'''def Xform "{name}" ( prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI"] )
    {{
        bool physics:rigidBodyEnabled = 1
        bool physics:kinematicEnabled = {int(actor["bolted"])}
        float physics:mass = {number(actor["mass_kg"])}
        float physxContactReport:threshold = 0
        int physxRigidBody:solverPositionIterationCount = 32
        int physxRigidBody:solverVelocityIterationCount = 8
        double3 xformOp:translate = {triple([x,y,TABLE_TOP+z])}
        uniform token[] xformOpOrder = ["xformOp:translate"]
        {solids}
        {marks}
    }}'''


def is_fixed(spec):
    return spec["type"]=="fixed" or (spec["type"] in {"hinge","slider"} and
        all(spec.get(key)==0 for key in ("min","max","rest","stiffness","damping")))


def joint(i, spec):
    if is_fixed(spec):
        return f'''def PhysicsFixedJoint "joint_{i}"
    {{
        rel physics:body0 = </Fixture/Actors/{identifier(spec["a"])}>
        rel physics:body1 = </Fixture/Actors/{identifier(spec["b"])}>
        point3f physics:localPos0 = {triple(spec["anchor_a"])}
        point3f physics:localPos1 = {triple(spec["anchor_b"])}
        quatf physics:localRot0 = (1,0,0,0)
        quatf physics:localRot1 = (1,0,0,0)
        bool physics:collisionEnabled = 0
        uniform bool physics:excludeFromArticulation = 1
    }}'''
    joint_type, drive = {"hinge": ("PhysicsRevoluteJoint", "angular"),
                         "slider": ("PhysicsPrismaticJoint", "linear")}[spec["type"]]
    axis = spec["axis"]
    if axis not in ["X", "Y", "Z"]:
        raise ValueError("Invalid joint axis")
    axes = {"X":(1.,0.,0.),"Y":(0.,1.,0.),"Z":(0.,0.,1.)}
    u = axes[axis]
    target = spec.get("axis_vector", u)
    length = math.hypot(*target)
    if length < 1e-8:
        raise ValueError("Degenerate joint direction")
    v = [x/length for x in target]
    dot = sum(a*b for a,b in zip(u,v))
    if dot < -0.99999999:
        quat = (0.,*(axes["Y"] if axis != "Y" else axes["Z"]))
    else:
        cross = (u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
        raw = (1.+dot,*cross)
        quat = tuple(x/math.hypot(*raw) for x in raw)
    rotation = "(" + ", ".join(number(x) for x in quat) + ")"
    return f'''def {joint_type} "joint_{i}" ( prepend apiSchemas = ["PhysicsDriveAPI:{drive}"] )
    {{
        rel physics:body0 = </Fixture/Actors/{identifier(spec["a"])}>
        rel physics:body1 = </Fixture/Actors/{identifier(spec["b"])}>
        uniform token physics:axis = "{axis}"
        point3f physics:localPos0 = {triple(spec["anchor_a"])}
        point3f physics:localPos1 = {triple(spec["anchor_b"])}
        quatf physics:localRot0 = {rotation}
        quatf physics:localRot1 = {rotation}
        uniform bool physics:excludeFromArticulation = 1
        bool physics:collisionEnabled = 0
        float physics:lowerLimit = {number(spec["min"])}
        float physics:upperLimit = {number(spec["max"])}
        float drive:{drive}:physics:targetPosition = {number(spec["rest"])}
        float drive:{drive}:physics:damping = {number(spec["damping"])}
        float drive:{drive}:physics:stiffness = {number(spec["stiffness"])}
        uniform token drive:{drive}:physics:type = "force"
    }}'''


def compile_layer(fixture):
    actors = fixture["actors"]
    bodies = "\n".join(body(a) for a in actors)
    joints = "\n".join(joint(i, j) for i, j in enumerate(fixture["joints"]))
    materials = "\n".join(surface(a) for a in actors)
    from .optics import emit_materials
    materials += emit_materials(actors)
    paint_colors=sorted({tuple(p["rgb"]) for a in actors for p in a.get("visuals",[])})
    paint="\n".join(ink_surface(rgb) for rgb in paint_colors)
    return f'''#usda 1.0
(
    defaultPrim = "Fixture"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "Fixture"
{{
    def Scope "Actors"
    {{
        {bodies}
    }}
    def Scope "Joints"
    {{
        {joints}
    }}
    def Scope "Materials"
    {{
        {materials}
    }}
    def Scope "MarkMaterials"
    {{
        {paint}
    }}
}}
'''
