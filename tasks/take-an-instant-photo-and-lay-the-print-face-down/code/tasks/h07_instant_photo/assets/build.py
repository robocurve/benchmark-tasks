#!/usr/bin/env python3
"""Build the models of the instant-photo task: a Polaroid Go Generation 2 instant camera (as modelled) and its print.

    python3 tasks/h07_instant_photo/assets/build.py

Writes, next to this file: ``layout.json`` (every dimension the task file uses) and ``camera.usda`` / ``print.usda``
(models with physics); adds the models' bounds to ``docs/assets/bounds.json``. Plain Python, no textures: the
geometry is drawn at the camera's true outer size with plain colours, no maker's name or logo.

Sources. Polaroid's product page for the Go Generation 2: "105mm x 84mm x 62mm", "Weight: 239 grams (8.43 ounces)
(without film pack)", colours Black, White, Blue, Purple. Polaroid's Go film page: "Format: 66.6mm x 53.9mm", "Image
area: 47mm x 46mm". The Go's manual and Polaroid's support pages: the red shutter button is on the top; "The photo
will eject from the slot at the front of the camera as soon as you let go of the shutter button". Not stated by the
maker, and so our estimates (marked below): which outer size is which (we take 105 mm front to back, 84 mm across,
62 mm high: the 53.9 mm wide print leaves through the front), the button's place, size, travel and force, the slot's
place and size, the film pack's mass, the print's thickness and mass.

Camera frame: metres; x out of the lens, y to the camera's left, z up; the origin on the bench under the camera's
centre. The camera is an articulation: the link ``body`` (everything but the button) and the link ``button`` (the
shutter button) on the prismatic joint ``shutter`` along z, held up by a spring; pressed home it has travelled
``SHUTTER_TRAVEL``. A tunnel runs from the front face 8 cm back, its floor 5 mm over the bench: the print lies in it
and leaves through the slot in the front face.
Print frame: x along its length (the end that leaves the camera first is +x), z out of the picture side (up while the
print is in the camera; the back is black); the origin at the centre of the underside.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

# --- dimensions (the task file reads layout.json) -----------------------------------------------------------------
CAMERA = "Polaroid Go Generation 2"
DEPTH, WIDTH, HEIGHT = 0.105, 0.084, 0.062  # front to back (x), across (y), high (z): Polaroid's 105 x 84 x 62 mm
CORNER = 0.012  # the rounded vertical edges (estimate)
MASS = 0.239 + 0.020  # Polaroid's 239 g without film pack, and the pack (estimate: 20 g)
FLOOR, GAP = 0.005, 0.005  # the tunnel's floor over the bench and its height: the slot (estimate)
TUNNEL = (0.080, 0.062)  # depth from the front face, width (estimate: the 53.9 mm print with room either side)
LENS = (0.0165, 0.010, 0.039)  # radius, length out of the front, axis height (estimate)
SHUTTER = (0.030, -0.024)  # the button's middle on the top: front right, under a right index finger (estimate)
SHUTTER_R, SHUTTER_H = 0.006, 0.003  # 12 mm across, standing 3 mm proud of the top (estimate)
SHUTTER_TRAVEL = 0.002  # m from rest to pressed home (estimate)
SHUTTER_SPRING = 2000.0  # N/m: 4 N pressed home (estimate; a bench reads it with a push gauge)
SHUTTER_DAMPING = 5.0  # N s/m (about critical for the 2 g button)
BUTTON_MASS = 0.002
GRIP = (-0.032, 0.0, HEIGHT - 0.016)  # where a hand from above holds the camera's rear part across its width
FOOT_FRICTION = (0.30, 0.25)  # the plastic underside on the bench (static, dynamic): the camera slides under about 0.8 N

PRINT = (0.0666, 0.0539, 0.0025)  # length, width (Polaroid's 66.6 x 53.9 mm), thickness (2.5 mm for stable contacts)
PRINT_MASS = 0.005  # estimate
PICTURE = (0.047, 0.046)  # Polaroid's image area; the wide border is at the end that leaves the camera last
BORDER_FRONT = 0.005
HIDDEN_X = DEPTH / 2 - TUNNEL[0] + 0.002 + PRINT[0] / 2  # print centre (camera frame) while inside: 2 mm from the tunnel's end
PROTRUDE = 0.045  # how far the print stands out of the slot once ejected (estimate; the rest stays in the rollers)
WHITE, BLACK, GREY, RED = (236, 236, 232), (20, 20, 22), (120, 122, 128), (214, 40, 36)


def layout() -> dict:
    front = DEPTH / 2
    return {
        "camera_model": CAMERA,
        "camera": {"size": (DEPTH, WIDTH, HEIGHT + SHUTTER_H), "top": HEIGHT, "mass": MASS, "front": front, "grip": GRIP,
                   "shutter": (*SHUTTER, HEIGHT + SHUTTER_H), "shutter_radius": SHUTTER_R, "shutter_travel": SHUTTER_TRAVEL,
                   "shutter_spring": SHUTTER_SPRING, "shutter_damping": SHUTTER_DAMPING, "lens": (front + LENS[1], 0.0, LENS[2]),
                   "lens_radius": LENS[0], "foot_friction": FOOT_FRICTION},
        "tunnel": {"floor": FLOOR, "gap": GAP, "depth": TUNNEL[0], "width": TUNNEL[1]},
        "print": {"size": PRINT, "mass": PRINT_MASS, "hidden_x": round(HIDDEN_X, 5), "protrude": PROTRUDE,
                  "ejected_x": round(front + PROTRUDE - PRINT[0] / 2, 5)},
        "assets": {"camera": "h07_camera", "print": "h07_print"},
        "extent": {"camera": DEPTH, "print": PRINT[0]},
    }


# --- USD text -----------------------------------------------------------------------------------------------------
ROOT_NAME = [""]  # the model being written (material paths are absolute inside the file)
INVISIBLE = '        token visibility = "invisible"'


def f(v) -> str:
    return f"{v:.6g}"


def vec(p) -> str:
    return "(" + ", ".join(f(v) for v in p) + ")"


def srgb(r, g, b):
    return tuple((c / 255 / 12.92) if c / 255 <= 0.04045 else ((c / 255 + 0.055) / 1.055) ** 2.4 for c in (r, g, b))


def rounded_rect(w, h, r, n=6):
    """Outline of a w x h rectangle with rounded corners, counter-clockwise, centred: [(a, b)]."""
    pts = []
    for cx, cy, a0 in ((w / 2 - r, h / 2 - r, 0), (-w / 2 + r, h / 2 - r, 90), (-w / 2 + r, -h / 2 + r, 180), (w / 2 - r, -h / 2 + r, 270)):
        for k in range(n + 1):
            a = math.radians(a0 + 90 * k / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def _normal(pts):
    n = [0.0, 0.0, 0.0]
    for i, p in enumerate(pts):
        q = pts[(i + 1) % len(pts)]
        n[0] += (p[1] - q[1]) * (p[2] + q[2])
        n[1] += (p[2] - q[2]) * (p[0] + q[0])
        n[2] += (p[0] - q[0]) * (p[1] + q[1])
    return n


def mesh(name, points, faces, material, st=None, outward_from=None, extra="") -> str:
    """A polygon mesh; ``faces`` index ``points``; ``st`` gives one uv per face corner. With ``outward_from`` each
    face is wound to point away from that centre."""
    faces = [list(fc) for fc in faces]
    st = [list(s) for s in st] if st else None
    if outward_from is not None:
        for k, fc in enumerate(faces):
            pts = [points[i] for i in fc]
            c = [sum(p[a] for p in pts) / len(pts) for a in range(3)]
            n = _normal(pts)
            if sum(n[a] * (c[a] - outward_from[a]) for a in range(3)) < 0:
                faces[k] = fc[::-1]
                if st:
                    st[k] = st[k][::-1]
    lo = [min(p[a] for p in points) for a in range(3)]
    hi = [max(p[a] for p in points) for a in range(3)]
    out = [
        f'    def Mesh "{name}"',
        "    {",
        f"        float3[] extent = [{vec(lo)}, {vec(hi)}]",
        f"        int[] faceVertexCounts = [{', '.join(str(len(fc)) for fc in faces)}]",
        f"        int[] faceVertexIndices = [{', '.join(str(i) for fc in faces for i in fc)}]",
        f"        point3f[] points = [{', '.join(vec(p) for p in points)}]",
        '        uniform token subdivisionScheme = "none"',
        "        uniform bool doubleSided = 1",
        f"        rel material:binding = </{ROOT_NAME[0]}/Looks/{material}>",
    ]
    if st:
        out.append(f'        texCoord2f[] primvars:st = [{", ".join(vec(uv) for s in st for uv in s)}] (interpolation = "faceVarying")')
    if extra:
        out.append(extra)
    out.append("    }")
    return "\n".join(out)


def prism(name, outline, lo, hi, centre_xy, material) -> str:
    """An outline (in x, y about ``centre_xy``) swept from z ``lo`` to ``hi``: sides and two end caps."""
    n = len(outline)
    cx, cy = centre_xy
    pts = [(cx + a, cy + b, lo) for a, b in outline] + [(cx + a, cy + b, hi) for a, b in outline]
    centre = (cx, cy, (lo + hi) / 2)
    faces = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)] + [tuple(range(n)), tuple(range(n, 2 * n))]
    return mesh(name, pts, faces, material, outward_from=centre)


def quad(name, centre, up, right, material, extra="") -> str:
    """A printed rectangle: ``up`` and ``right`` are the image's half-extent vectors from ``centre``."""
    c, u, r = centre, up, right
    p = lambda su, sr: tuple(c[a] + su * u[a] + sr * r[a] for a in range(3))  # noqa: E731
    pts = [p(-1, -1), p(-1, 1), p(1, 1), p(1, -1)]
    return mesh(name, pts, [(0, 1, 2, 3)], material, [[(0, 0), (1, 0), (1, 1), (0, 1)]], extra=extra)


def cylinder(name, radius, height, centre, material, axis="Z", collide=False) -> str:
    api = ' (\n        prepend apiSchemas = ["PhysicsCollisionAPI"]\n    )' if collide else ""
    half = {"X": (height / 2, radius, radius), "Z": (radius, radius, height / 2)}[axis]
    return "\n".join([
        f'    def Cylinder "{name}"{api}',
        "    {",
        f'        uniform token axis = "{axis}"',
        f"        double height = {f(height)}",
        f"        double radius = {f(radius)}",
        f"        float3[] extent = [{vec(tuple(-v for v in half))}, {vec(half)}]",
        f"        double3 xformOp:translate = {vec(centre)}",
        '        uniform token[] xformOpOrder = ["xformOp:translate"]',
        f"        rel material:binding = </{ROOT_NAME[0]}/Looks/{material}>",
        "    }",
    ])


def collider(name, lo, hi, physics_material=None) -> str:
    """An invisible box collider from corner ``lo`` to corner ``hi``."""
    size = tuple(b - a for a, b in zip(lo, hi))
    centre = tuple((a + b) / 2 for a, b in zip(lo, hi))
    out = [
        f'    def Cube "{name}" (',
        '        prepend apiSchemas = ["PhysicsCollisionAPI"]',
        "    )",
        "    {",
        "        double size = 1",
        "        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]",
        f"        double3 xformOp:translate = {vec(centre)}",
        f"        float3 xformOp:scale = {vec(size)}",
        '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]',
        '        token visibility = "invisible"',
    ]
    if physics_material:
        out.append(f"        rel material:binding:physics = </{ROOT_NAME[0]}/Looks/{physics_material}>")
    out.append("    }")
    return "\n".join(out)


def material(name, *, color, roughness=0.5) -> str:
    path = f"/{ROOT_NAME[0]}/Looks/{name}"
    return "\n".join([
        f'        def Material "{name}"', "        {", f"            token outputs:surface.connect = <{path}/Shader.outputs:surface>",
        '            def Shader "Shader"', "            {", '                uniform token info:id = "UsdPreviewSurface"',
        f"                color3f inputs:diffuseColor = {vec(color)}",
        f"                float inputs:roughness = {f(roughness)}", "                float inputs:metallic = 0", "                token outputs:surface", "            }",
        "        }",
    ])


def physics_material(name, static, dynamic) -> str:
    return "\n".join([
        f'        def Material "{name}" (',
        '            prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]',
        "        )",
        "        {",
        f"            float physics:staticFriction = {f(static)}",
        f"            float physics:dynamicFriction = {f(dynamic)}",
        "            float physics:restitution = 0",
        '            uniform token physxMaterial:frictionCombineMode = "min"',
        "        }",
    ])


def model(name, doc, body, looks) -> str:
    return "\n".join([
        "#usda 1.0", "(", f'    defaultPrim = "{name}"', "    metersPerUnit = 1", '    upAxis = "Z"', f'    doc = "{doc}"', ")", "",
        f'def Xform "{name}" (', '    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]', ")", "{",
        *body, '    def Scope "Looks"', "    {", *looks, "    }", "}", "",
    ])


def articulated_model(name, doc, body, body_mass, button, button_mass, button_at, axis, travel, spring, damping, looks) -> str:
    """A free articulation: the link ``body`` (``body_mass``; the articulation's root) and the link ``button``
    (``button_mass``; its parts drawn about its rest centre ``button_at``, body frame) on a prismatic joint
    ``shutter`` along ``axis`` ("X" or "Z" of the body frame): the button travels from 0 (at rest, against its upper
    stop) to -``travel`` (pressed home), held at 0 by a spring of ``spring`` N/m with ``damping`` N s/m (the task's
    actuator carries the same values: Isaac Lab writes its gains over the file's drive). Links do not collide with
    each other, so the button sinks into its recess in the body."""
    return "\n".join([
        "#usda 1.0", "(", f'    defaultPrim = "{name}"', "    metersPerUnit = 1", '    upAxis = "Z"', f'    doc = "{doc}"', ")", "",
        f'def Xform "{name}"', "{",
        '    def Xform "body" (',
        '        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]',
        "    )", "    {", f"        float physics:mass = {f(body_mass)}", "        bool physxArticulation:enabledSelfCollisions = 0",
        *body, "    }",
        '    def Xform "button" (', '        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]', "    )", "    {",
        f"        float physics:mass = {f(button_mass)}", f"        double3 xformOp:translate = {vec(button_at)}",
        '        uniform token[] xformOpOrder = ["xformOp:translate"]', *button, "    }",
        '    def PhysicsPrismaticJoint "shutter" (', '        prepend apiSchemas = ["PhysicsDriveAPI:linear"]', "    )", "    {",
        f"        rel physics:body0 = </{name}/body>", f"        rel physics:body1 = </{name}/button>",
        f'        uniform token physics:axis = "{axis}"', f"        float physics:lowerLimit = {f(-travel)}", "        float physics:upperLimit = 0",
        f"        point3f physics:localPos0 = {vec(button_at)}", "        quatf physics:localRot0 = (1, 0, 0, 0)",
        "        point3f physics:localPos1 = (0, 0, 0)", "        quatf physics:localRot1 = (1, 0, 0, 0)",
        '        uniform token drive:linear:physics:type = "force"', f"        float drive:linear:physics:stiffness = {f(spring)}",
        f"        float drive:linear:physics:damping = {f(damping)}", "        float drive:linear:physics:targetPosition = 0", "    }",
        '    def Scope "Looks"', "    {", *looks, "    }", "}", "",
    ])


def lens_tip() -> float:
    """The front of the lens's glass, past the front face (the model's foremost point)."""
    return DEPTH / 2 + LENS[1] + 0.0012


def box() -> tuple[tuple, tuple]:
    """The camera's local bounds (everything drawn): the lens stands out of the front, the button out of the top."""
    return (-DEPTH / 2 - 0.0004, -WIDTH / 2, 0.0), (lens_tip(), WIDTH / 2, HEIGHT + SHUTTER_H)  # 0.4 mm: the eyepiece drawn on the back


def build_camera() -> str:
    ROOT_NAME[0] = "camera"
    front, half_w = DEPTH / 2, WIDTH / 2
    tw = TUNNEL[1] / 2
    back = front - TUNNEL[0]
    roof = FLOOR + GAP
    eps = 0.0004
    lens_r, lens_len, lens_z = LENS
    body = [
        prism("shell", rounded_rect(DEPTH, WIDTH, CORNER), 0.0, HEIGHT, (0.0, 0.0), "shell"),
        quad("slot", (front + eps, 0.0, FLOOR + GAP / 2), (0, 0, GAP / 2 + 0.001), (0, tw + 0.002, 0), "dark"),
        cylinder("lens_barrel", lens_r, lens_len, (front + lens_len / 2, 0.0, lens_z), "dark", axis="X"),
        cylinder("lens_ring", lens_r - 0.004, 0.001, (front + lens_len + 0.0005, 0.0, lens_z), "ring", axis="X"),
        cylinder("lens_glass", lens_r - 0.007, 0.001, (front + lens_len + 0.0007, 0.0, lens_z), "glass", axis="X"),
        quad("flash", (front + eps, -0.024, HEIGHT - 0.012), (0, 0, 0.005), (0, 0.010, 0), "flash"),  # drawn only (estimate)
        quad("finder_front", (front + eps, 0.026, HEIGHT - 0.012), (0, 0, 0.005), (0, 0.006, 0), "dark"),
        quad("finder_back", (-front - eps, 0.026, HEIGHT - 0.012), (0, 0, 0.005), (0, -0.006, 0), "dark"),
        cylinder("flash_button", 0.004, 0.001, (SHUTTER[0] - 0.020, SHUTTER[1], HEIGHT + 0.0005), "dark"),  # drawn only
        # colliders: under the tunnel a floor, over it the rest of the body; the tunnel's sides and end; the lens
        collider("c_floor", (-front, -half_w, 0.0), (front, half_w, FLOOR), "feet"),
        collider("c_upper", (-front, -half_w, roof), (front, half_w, HEIGHT)),
        collider("c_left", (-front, tw, FLOOR), (front, half_w, roof)),
        collider("c_right", (-front, -half_w, FLOOR), (front, -tw, roof)),
        collider("c_back", (-front, -tw, FLOOR), (back, tw, roof)),
        collider("c_lens", (front, -lens_r, lens_z - lens_r), (front + lens_len, lens_r, lens_z + lens_r)),
    ]
    button = [cylinder("cap", SHUTTER_R, SHUTTER_H, (0.0, 0.0, 0.0), "red", collide=True)]
    looks = [
        material("shell", color=srgb(*WHITE), roughness=0.5),
        material("dark", color=srgb(*BLACK), roughness=0.5),
        material("ring", color=srgb(*GREY), roughness=0.3),
        material("glass", color=srgb(18, 26, 44), roughness=0.05),
        material("flash", color=srgb(206, 210, 214), roughness=0.2),
        material("red", color=srgb(*RED), roughness=0.4),
        physics_material("feet", *FOOT_FRICTION),
    ]
    doc = ("A Polaroid Go Generation 2 instant camera as modelled (outer size from the maker's page, no name or logo): a body "
           "with the lens in front, a tunnel along its floor that ends in the slot in the front face, and the red shutter "
           "button on the top on a sprung prismatic joint")
    return articulated_model("camera", doc, body, MASS - BUTTON_MASS, button, BUTTON_MASS, (*SHUTTER, HEIGHT + SHUTTER_H / 2), "Z",
                             SHUTTER_TRAVEL, SHUTTER_SPRING, SHUTTER_DAMPING, looks)


def build_print() -> str:
    ROOT_NAME[0] = "print"
    length, width, t = PRINT
    pic_x = length / 2 - BORDER_FRONT - PICTURE[0] / 2  # the picture area's middle: the wide border at the -x end
    body = [
        prism("card", rounded_rect(length, width, 0.002, n=3), 0.0, t, (0.0, 0.0), "paper"),
        quad("back", (0.0, 0.0, -0.0002), (length / 2, 0, 0), (0, width / 2, 0), "back"),
        quad("picture", (pic_x, 0.0, t + 0.0002), (PICTURE[0] / 2, 0, 0), (0, -PICTURE[1] / 2, 0), "picture"),
        collider("c_card", (-length / 2, -width / 2, 0.0), (length / 2, width / 2, t)),
    ]
    looks = [
        material("paper", color=srgb(246, 245, 240), roughness=0.6),
        material("back", color=srgb(24, 24, 26), roughness=0.5),
        material("picture", color=srgb(58, 70, 84), roughness=0.3),  # a fresh print's undeveloped picture: dark slate
    ]
    return model("print", "An instant print of Go film size: white card, black back, the picture area (on top) still undeveloped.", body, looks)


def bounds_entry(file, low, high) -> dict:
    size = [round(b - a, 4) for a, b in zip(low, high)]
    return {"extent": max(size[:2]), "file": file, "height": size[2], "high": [round(v, 4) for v in high], "low": [round(v, 4) for v in low], "size": size}


def main() -> None:
    lo, hi = box()
    out = layout()
    out["camera"]["box_centre"] = [round((a + b) / 2, 5) for a, b in zip(lo, hi)]  # where the simulator's centroid sits, camera frame
    out["extent"]["camera"] = round(hi[0] - lo[0], 5)  # the widest horizontal size, lens included: drawn at scale 1
    (HERE / "layout.json").write_text(json.dumps(out, indent=1) + "\n")
    (HERE / "camera.usda").write_text(build_camera())
    (HERE / "print.usda").write_text(build_print())
    bounds_file = ROOT / "docs" / "assets" / "bounds.json"
    bounds = json.loads(bounds_file.read_text())
    bounds["h07_camera"] = bounds_entry("camera.usda", lo, hi)
    bounds["h07_print"] = bounds_entry("print.usda", (-PRINT[0] / 2, -PRINT[1] / 2, 0.0), (PRINT[0] / 2, PRINT[1] / 2, PRINT[2]))
    bounds_file.write_text(json.dumps(dict(sorted(bounds.items())), indent=1) + "\n")
    print("wrote camera.usda (an articulation: body and shutter button), print.usda, layout.json; bounds for 2 models")


if __name__ == "__main__":
    main()
