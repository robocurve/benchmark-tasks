"""Write the task's USD assets from the dimensions in chemcheck.py:

    python make_assets.py            (rewrites the .usda files; test_chemcheck checks they are up to date)
    python make_assets.py --images   (also redraws the label images; needs Pillow and the DejaVu Sans Bold font)

assets/dipper.usda: the PTFE chip dipper, one rigid body (mass set in task.py) whose origin is the centre of the
basket's underside, x from its open front to the back, z up: the basket (base, walls, comb teeth, floor ridges), the
silicon chip standing in its groove, and the handle with a rounded top rising from the floor's centre. Every part
collides.

assets/dish.usda: a bath, a round glass crystallizing dish (fixed): a ring of flat wall segments that collide, and
inside it a thin tinted disc at the fill level that nothing collides with (no liquid is simulated). The origin is the
centre of the dish's footprint on the table. Clear glass, with a blue tint to the liquid; all four baths use it.

assets/label_<TEXT>.usda: the hazard bath's printed label, a thin white card lying flat (it collides) with the label
image on its upper face. The origin is the card's centre; the text reads left to right for a viewer looking along +x
(the robot), its top edge toward +x. The images are assets/labels/<TEXT>.png.
"""

from __future__ import annotations

import sys
from pathlib import Path

import chemcheck as cc

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets" / "dipper.usda"
LABEL_DIR = HERE / "assets" / "labels"
LABEL_TEXTS = cc.HAZARD_LABELS
PTFE = (0.96, 0.96, 0.94)
SILICON = (0.22, 0.25, 0.36)
CARD = (0.97, 0.97, 0.97)


def _f(v: float) -> str:
    return f"{v:.6g}"


def _vec(v) -> str:
    return "(" + ", ".join(_f(c) for c in v) + ")"


def _cube(name, centre, size, color, indent="    ") -> str:
    return f'''
{indent}def Cube "{name}" (
{indent}    prepend apiSchemas = ["PhysicsCollisionAPI"]
{indent})
{indent}{{
{indent}    color3f[] primvars:displayColor = [{_vec(color)}]
{indent}    double size = 1
{indent}    double3 xformOp:translate = {_vec(centre)}
{indent}    float3 xformOp:scale = {_vec(size)}
{indent}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
{indent}}}
'''


def usda() -> str:
    """The dipper."""
    parts = "".join(_cube(name, c, s, SILICON if name == "chip" else PTFE) for name, (c, s) in cc.DIPPER_PARTS.items())
    handle = f'''
    def Cylinder "handle" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        uniform token axis = "Z"
        color3f[] primvars:displayColor = [{_vec(PTFE)}]
        double height = {_f(cc.HANDLE_L)}
        double radius = {_f(cc.HANDLE_R)}
        double3 xformOp:translate = {_vec(cc.HANDLE_CENTRE)}
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
'''
    cap = f'''
    def Sphere "handle_cap" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        color3f[] primvars:displayColor = [{_vec(PTFE)}]
        double radius = {_f(cc.HANDLE_R)}
        double3 xformOp:translate = {_vec(cc.HANDLE_CAP)}
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
'''
    return f'''#usda 1.0
(
    defaultPrim = "Dipper"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Dipper" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{{
    float physics:mass = {_f(cc.DIPPER_KG)}
{parts}{handle}{cap}}}
'''


def label_usda(text: str) -> str:
    """A label card lying flat: a thin white box (collides), and just above it a textured quad facing up."""
    w, h, t = cc.LABEL_W, cc.LABEL_H, cc.LABEL_T
    z = t / 2 + 0.0002  # the printed face, 0.2 mm above the card
    # Looking along +x (the robot's view), +y is on the left: the image's left edge (u 0) goes at +y, its top at +x.
    pts = [(-h / 2, w / 2, z), (-h / 2, -w / 2, z), (h / 2, -w / 2, z), (h / 2, w / 2, z)]
    card = _cube("card", (0.0, 0.0, 0.0), (h, w, t), CARD)
    return f'''#usda 1.0
(
    defaultPrim = "Label"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Label" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
)
{{{card}
    def Mesh "face" (
        prepend apiSchemas = ["MaterialBindingAPI"]
    )
    {{
        int[] faceVertexCounts = [4]
        int[] faceVertexIndices = [0, 1, 2, 3]
        point3f[] points = [{", ".join(_vec(p) for p in pts)}]
        texCoord2f[] primvars:st = [(0, 0), (1, 0), (1, 1), (0, 1)] (
            interpolation = "vertex"
        )
        uniform bool doubleSided = 1
        rel material:binding = </Label/print>
    }}

    def Material "print"
    {{
        token outputs:surface.connect = </Label/print/surface.outputs:surface>

        def Shader "surface"
        {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor.connect = </Label/print/image.outputs:rgb>
            float inputs:roughness = 0.8
            token outputs:surface
        }}

        def Shader "image"
        {{
            uniform token info:id = "UsdUVTexture"
            asset inputs:file = @./labels/{text}.png@
            float2 inputs:st.connect = </Label/print/st.outputs:result>
            token inputs:wrapS = "clamp"
            token inputs:wrapT = "clamp"
            float3 outputs:rgb
        }}

        def Shader "st"
        {{
            uniform token info:id = "UsdPrimvarReader_float2"
            token inputs:varname = "st"
            float2 outputs:result
        }}
    }}
}}
'''


GLASS, GLASS_OPACITY, LIQUID = (0.80, 0.88, 0.92), 0.45, (0.55, 0.80, 0.95)
DISH = HERE / "assets" / "dish.usda"


def dish_usda() -> str:
    import math

    glass, opacity, liquid = GLASS, GLASS_OPACITY, LIQUID
    n, r_in, wall, h = cc.DISH_SEGMENTS, cc.BATH_INNER / 2, cc.BATH_WALL, cc.BATH_HEIGHT
    r_mid, chord = r_in + wall / 2, 2 * (r_in + wall) * math.tan(math.pi / n)
    segs = ""
    for k in range(n):
        ang = 2 * math.pi * k / n
        segs += f'''
    def Cube "wall{k}" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        color3f[] primvars:displayColor = [{_vec(glass)}]
        float[] primvars:displayOpacity = [{_f(opacity)}]
        double size = 1
        double3 xformOp:translate = {_vec((r_mid * math.cos(ang), r_mid * math.sin(ang), h / 2))}
        float xformOp:rotateZ = {_f(math.degrees(ang))}
        float3 xformOp:scale = {_vec((wall, chord, h))}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZ", "xformOp:scale"]
    }}
'''
    return f'''#usda 1.0
(
    defaultPrim = "Dish"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Dish" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI"]
)
{{{segs}
    def Cylinder "liquid"
    {{
        uniform token axis = "Z"
        color3f[] primvars:displayColor = [{_vec(liquid)}]
        double height = 0.002
        double radius = {_f(r_in)}
        double3 xformOp:translate = {_vec((0.0, 0.0, cc.LIQUID_LEVEL - 0.001))}
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }}
}}
'''


def label_path(text: str) -> Path:
    return HERE / "assets" / f"label_{text}.usda"


def draw_label(text: str, path: Path) -> None:
    """A printed label: black (hazard labels: red) bold capitals on white, with a thin border; 640 x 192 px, the
    card's 10 x 3 cm."""
    from PIL import Image, ImageDraw, ImageFont

    w, h = 640, 192
    hazard = text in cc.HAZARD_LABELS
    ink = (190, 20, 20) if hazard else (20, 20, 20)
    img = Image.new("RGB", (w, h), (247, 247, 247))
    d = ImageDraw.Draw(img)
    d.rectangle([6, 6, w - 7, h - 7], outline=ink, width=8)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
    size = 140
    while True:
        font = ImageFont.truetype(font_path, size)
        left, top, right, bottom = d.textbbox((0, 0), text, font=font)
        if right - left <= w - 70 and bottom - top <= h - 60:
            break
        size -= 4
    d.text(((w - (right - left)) / 2 - left, (h - (bottom - top)) / 2 - top), text, font=font, fill=ink)
    img.save(path)


if __name__ == "__main__":
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(usda())
    print(f"wrote {OUT}")
    for text in LABEL_TEXTS:
        label_path(text).write_text(label_usda(text))
        print(f"wrote {label_path(text)}")
    DISH.write_text(dish_usda())
    print(f"wrote {DISH}")
    if "--images" in sys.argv:
        LABEL_DIR.mkdir(parents=True, exist_ok=True)
        for text in LABEL_TEXTS:
            draw_label(text, LABEL_DIR / f"{text}.png")
            print(f"wrote {LABEL_DIR / f'{text}.png'}")
