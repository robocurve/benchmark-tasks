"""Generate the original USD apparatus and plain text labels; no simulator is needed."""

import json
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

import spec as S

ASSETS = S.DIRECTORY / "assets"
COLORS = {"blue": (0.19, 0.35, 0.52), "steel": (0.65, 0.68, 0.71), "dark": (0.07, 0.08, 0.09),
          "red": (0.65, 0.07, 0.06), "black": (0.055, 0.055, 0.055), "rubber": (0.16, 0.17, 0.19),
          "green": (0.10, 0.48, 0.22), "yellow": (0.85, 0.57, 0.04), "lcd": (0.025, 0.06, 0.045),
          "lit": (0.4, 0.95, 0.6)}


def vector(values):
    return "(" + ", ".join(f"{float(x):.8g}" for x in values) + ")"


def label(filename, text, background):
    im = Image.new("RGB", (768, 160), background)
    draw = ImageDraw.Draw(im)
    font = ImageFont.load_default(size=64)
    draw.text((384, 80), text, fill="white", font=font, anchor="mm")
    im.save(ASSETS / filename)


def box(name, size, pos=(0, 0, 0), color="rubber", angle=0, collision=True):
    return {"name": name, "kind": "Cube", "size": size, "pos": pos, "color": color,
            "angle": angle, "collision": collision}


def cylinder(name, radius, height, pos=(0, 0, 0), color="steel", axis="Y", collision=True):
    return {"name": name, "kind": "Cylinder", "radius": radius, "height": height,
            "axis": axis, "pos": pos, "color": color, "collision": collision}


def sphere(name, radius, pos, color="steel"):
    return {"name": name, "kind": "Sphere", "radius": radius, "pos": pos,
            "color": color, "collision": True}


def texture_plane(name, filename, width, height, pos):
    return f'''
    def Material "{name}_material" {{
        token outputs:surface.connect = </Body/{name}_material/Surface.outputs:surface>
        def Shader "Surface" {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor.connect = </Body/{name}_material/Texture.outputs:rgb>
            float inputs:roughness = 1
            token outputs:surface
        }}
        def Shader "Texture" {{
            uniform token info:id = "UsdUVTexture"
            asset inputs:file = @{filename}@
            float2 inputs:st.connect = </Body/{name}_material/UV.outputs:result>
            float3 outputs:rgb
        }}
        def Shader "UV" {{
            uniform token info:id = "UsdPrimvarReader_float2"
            token inputs:varname = "st"
            float2 outputs:result
        }}
    }}
    def Mesh "{name}" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    ) {{
        point3f[] points = [({-height/2}, {width/2}, 0), ({height/2}, {width/2}, 0), ({height/2}, {-width/2}, 0), ({-height/2}, {-width/2}, 0)]
        int[] faceVertexCounts = [4]
        int[] faceVertexIndices = [0, 1, 2, 3]
        texCoord2f[] primvars:st = [(0, 0), (0, 1), (1, 1), (1, 0)] (interpolation = "vertex")
        uniform token subdivisionScheme = "none"
        bool doubleSided = true
        double3 xformOp:translate = {vector(pos)}
        uniform token[] xformOpOrder = ["xformOp:translate"]
        rel material:binding = </Body/{name}_material>
    }}
'''


def write_usd(filename, shapes, mass=None, extras=""):
    apis = '["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysxContactReportAPI"]'
    root = f'def Xform "Body" (prepend apiSchemas = {apis}) {{' if mass else 'def Xform "Body" {'
    lines = ['#usda 1.0', '(\n    defaultPrim = "Body"\n    metersPerUnit = 1\n    upAxis = "Z"\n)',
             root]
    if mass:
        lines.extend([f'    float physics:mass = {mass}', '    bool physics:rigidBodyEnabled = true',
                      '    float physxRigidBody:linearDamping = 0.1', '    float physxRigidBody:angularDamping = 1',
                      '    float physxRigidBody:maxDepenetrationVelocity = 0.3',
                      '    int physxRigidBody:solverPositionIterationCount = 32',
                      '    int physxRigidBody:solverVelocityIterationCount = 8',
                      '    float physxContactReport:threshold = 0'])
    lines.append('    def Material "PhysicsMaterial" (prepend apiSchemas = ["PhysicsMaterialAPI"]) {\n'
                 '        float physics:staticFriction = 0.9\n        float physics:dynamicFriction = 0.7\n'
                 '        float physics:restitution = 0\n    }')
    for name, color in COLORS.items():
        lines.append(f'''    def Material "{name}" {{
        token outputs:surface.connect = </Body/{name}/Shader.outputs:surface>
        def Shader "Shader" {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor = {vector(color)}
            float inputs:roughness = 0.6
            token outputs:surface
        }}
    }}''')
    for shape in shapes:
        schemas = '["PhysicsCollisionAPI", "PhysxCollisionAPI", "MaterialBindingAPI"]' if shape["collision"] else '["MaterialBindingAPI"]'
        lines.append(f'    def {shape["kind"]} "{shape["name"]}" (prepend apiSchemas = {schemas}) {{')
        if shape["kind"] == "Cube":
            lines.extend(['        double size = 1', f'        double3 xformOp:scale = {vector(shape["size"])}'])
        elif shape["kind"] == "Cylinder":
            lines.extend([f'        double radius = {shape["radius"]}', f'        double height = {shape["height"]}',
                          f'        uniform token axis = "{shape["axis"]}"'])
        else:
            lines.append(f'        double radius = {shape["radius"]}')
        lines.append(f'        double3 xformOp:translate = {vector(shape["pos"])}')
        order = ['"xformOp:translate"']
        if shape.get("angle"):
            lines.append(f'        float xformOp:rotateY = {math.degrees(shape["angle"])}')
            order.append('"xformOp:rotateY"')
        if shape["kind"] == "Cube":
            order.append('"xformOp:scale"')
        lines.append(f'        uniform token[] xformOpOrder = [{", ".join(order)}]')
        lines.append(f'        rel material:binding = </Body/{shape["color"]}>')
        if not shape.get("visible", True):
            lines.append('        token visibility = "invisible"')
        if shape["collision"]:
            lines.extend(['        bool physics:collisionEnabled = true', '        float physxCollision:contactOffset = 0.0003',
                          '        float physxCollision:restOffset = 0', '        rel material:binding:physics = </Body/PhysicsMaterial>'])
        lines.append('    }')
    lines.extend([extras, '}'])
    (ASSETS / filename).write_text("\n".join(lines) + "\n")
    return shapes


def build():
    ASSETS.mkdir(exist_ok=True)
    inventory = {}
    inventory["aa_cell"] = write_usd("aa_cell.usda", [
        cylinder("insulated_can", S.CELL_RADIUS, 0.0493, (0, -0.0004, 0), "blue"),
        cylinder("negative_terminal", S.NEGATIVE_RADIUS, 0.0002, (0, -S.CELL_HALF+0.0001, 0)),
        cylinder("positive_terminal", S.POSITIVE_RADIUS, 0.001, (0, 0.02475, 0)),
        cylinder("positive_shoulder", S.CELL_RADIUS, 0.0003, (0, 0.0241, 0)),
    ], mass=0.023)
    for color in ("red", "black"):
        inventory[color + "_probe"] = write_usd(color + "_probe.usda", [
            cylinder("insulated_handle", 0.008, 0.070, color=color),
            cylinder("finger_guard", 0.012, 0.004, (0, 0.032, 0), color),
            cylinder("insulated_nose", 0.004, 0.050, (0, 0.062, 0), color),
            cylinder("metal_shaft", 0.0014, 0.028, (0, 0.101, 0)),
            sphere("metal_tip", S.TIP_RADIUS, tuple(S.TIP_LOCAL)),
        ], mass=0.028)

    cradles = []
    for i, name in enumerate(S.CELL_NAMES):
        x = S.CELL_START[name][0]
        for end, y in enumerate((-0.0165, 0.0165)):
            cradles.append(box(f"support_{i}_{end}", (0.030, 0.006, 0.040), (x, y, 0.020)))
            for side in (-1, 1):
                cradles.append(box(f"v_{i}_{end}_{'l' if side<0 else 'r'}", (0.014, 0.006, 0.004),
                                   (x + side*0.007, y, 0.0455), angle=-side*math.pi/4))
    inventory["battery_cradles"] = write_usd("battery_cradles.usda", cradles)
    for name, position in S.PROBE_RESTS.items():
        rests = [box(f"rest_{i}", (0.022, 0.006, 0.052), (position[0], position[1] + y, 0.026))
                 for i, y in enumerate((-0.023, 0.023))]
        inventory[name + "_rest"] = write_usd(name + "_rest.usda", rests)

    for name, center in S.BIN_CENTERS.items():
        color = "green" if name == "alive" else "red"
        x, y = center[:2]
        shapes = [box("floor", (0.220, 0.180, 0.006), (x, y, 0.003), color)]
        for side in (-1, 1):
            shapes.extend([box(f"xwall_{side+1}", (0.005, 0.180, 0.070), (x + side*0.1075, y, 0.035), color),
                           box(f"ywall_{side+1}", (0.210, 0.005, 0.070), (x, y + side*0.0875, 0.035), color)])
        label(name + ".png", name.upper(), (20, 95, 45) if name == "alive" else (145, 20, 20))
        shapes.append(box("label_plate", (0.035, 0.165, 0.003), (x+0.121, y, 0.0715), color, collision=False))
        extra = texture_plane("bin_label", name + ".png", 0.16, 0.035, (x+0.121, y, 0.0731))
        inventory[name + "_bin"] = write_usd(name + "_bin.usda", shapes, extras=extra)

    shapes = [box("case", (0.165, 0.250, 0.035), (0, 0, 0.0175), "yellow"),
              box("screen", (0.075, 0.205, 0.002), (-0.030, 0, 0.036), "lcd", collision=False),
              cylinder("function_dial", 0.023, 0.009, (0.047, 0, 0.0395), "dark", axis="Z")]
    label("meter_label.png", f"DC V   CUTOFF {S.THRESHOLD_CV/100:.2f} V", (45, 45, 35))
    extras = texture_plane("meter_label", "meter_label.png", 0.220, 0.027, (0.063, 0, 0.047))
    for side, color in ((1, "red"), (-1, "black")):
        shapes.append(cylinder(f"jack_{color}", 0.006, 0.003, (-0.075, side*0.065, 0.036), color, axis="Z"))
    # Four character positions: optional minus sign, units, tenths, hundredths.
    for i in range(4):
        cx, cy = -0.030, (1.5-i)*0.046
        h, w, t = 0.053, 0.029, 0.004
        for segment, dx, dy, sx, sy in (
            ("a", h/2-t/2, 0, t, w-t), ("b", h/4, -w/2+t/2, h/2-t, t),
            ("c", -h/4, -w/2+t/2, h/2-t, t), ("d", -h/2+t/2, 0, t, w-t),
            ("e", -h/4, w/2-t/2, h/2-t, t), ("f", h/4, w/2-t/2, h/2-t, t),
            ("g", 0, 0, t, w-t),
        ):
            shapes.append(box(f"digit_{i}_{segment}", (sx, sy, 0.0005), (cx+dx, cy+dy, 0.0375), "lit", collision=False))
            shapes[-1]["visible"] = i > 0 and segment == "g"
    shapes.append(box("decimal", (0.004, 0.004, 0.0005), (-0.053, 0.0, 0.0375), "lit", collision=False))
    shapes[-1]["visible"] = False
    inventory["voltage_meter"] = write_usd("voltage_meter.usda", shapes, extras=extras)
    import task
    manifest = {"units": "metres, kilograms, seconds; box sizes are full edge lengths",
                "frame": "x forward from the arm bases, y left, z up",
                "station": {"table_size": [1.1, 1.3, 0.06], "table_center": [0.36, 0, 0.7175],
                            "arm_bases": [[0, 0.305, 0.75], [0, -0.305, 0.75]],
                            "cameras": "unmodified calibrated YAM station overhead and two wrist cameras",
                            "room": "standard YAM station floor, lights and camera stand"},
                "objects": [{"name": o.name, "position": o.pos, "quaternion_xyzw": o.quaternion,
                             "mass_kg": o.mass, "fixed": not o.free, "asset": o.asset_file} for o in task.OBJECTS],
                "asset_geometry": inventory}
    (S.DIRECTORY / "scene_inventory.json").write_text(json.dumps(manifest, indent=2) + "\n")
    metadata_path = S.DIRECTORY/"task.json"
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text())
        count = len(S.CELL_NAMES)
        metadata["instruction"] = S.INSTRUCTION
        positions = ", ".join(f"{p[0]:.2f}" for p in S.CELL_START.values())
        metadata["setup"] = (
            f"{count} identical unbranded AA cells, each 14.5 mm diameter, 50.5 mm overall and 23 g, with steel terminals and blue insulating wrap, "
            f"lie on fixed rubber V supports at x={positions} m, y=0, z=table-top+54 mm, positive ends along +y. "
            "Red and black 152 mm long, 28 g loose handheld probes have insulated handles/noses and bare steel tips, and rest on rubber blocks "
            "at x=0.11 m, y=+/-0.30 m, z=table-top+60.3 mm, pointing inward. A fixed yellow polymer DC voltmeter, 165x250x35 mm, "
            "stands at (0.525,0,table-top), with a live numeric display and preset voltage mode. Its two massless visual leads are 3 mm across. "
            "Fixed polymer ALIVE and DEAD bins, 220x180x70 mm with 210x170 mm inner openings, 6 mm floor, 5 mm walls and raised labels, "
            "stand at x=0.26 m, y=+/-0.30 m. The standard calibrated YAM station has a 1100x1300x60 mm table at z=0.7475 m, "
            "two parallel-gripper arms at (0,+/-0.305,0.75) m and three RGB cameras. README and scene_inventory.json enumerate every shape and pose."
        )
        metadata["scoring"] = (
            f"Both probe tips must contact opposite exposed terminals of the same cell continuously for {S.CONTACT_HOLD:.2f} s. "
            "The simulator requires at least 0.005 N mean native normal contact force on that cell over 50 ms from each probe, "
            "with at most 0.6 mm electrical contact-surface tolerance. Reversed polarity is allowed; use magnitude. "
            f"Values >= {S.THRESHOLD_CV/100:.2f} V belong in ALIVE and lower values in DEAD. "
            f"Each measured cell earns 1/{count} in the current final state when its complete native cell envelope is inside the correct "
            "210x170 mm bin opening, between the floor and rim, with 1 mm contact tolerance on every boundary, "
            "linear speed <=25 mm/s, angular speed <=0.8 rad/s, and every robot collision envelope >=15 mm away. "
            "Unmeasured, misplaced, held, moving or partially contained cells earn zero. "
            f"All {count} must satisfy these conditions for {S.SUCCESS_HOLD:.2f} s for success. Any battery/probe envelope leaving "
            "x=-0.19..0.91 m, y=-0.65..0.65 m, z=table-top-15 mm..1.80 m, or any nonfinite object state, permanently sets the whole trial to zero."
        )
        metadata["run_limit"] = f"{S.MAX_STEPS:,} control steps ({S.MAX_STEPS/S.HZ:g} seconds at {S.HZ} Hz)"
        metadata["scores"] = "Native demonstration: not yet verified after rebuilding; Dream-1: not yet run."
        metadata["media"] = None
        metadata_path.write_text(json.dumps(metadata, indent=2)+"\n")
    print("Built original apparatus assets in", ASSETS)


if __name__ == "__main__":
    build()
