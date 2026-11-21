"""Original printable recipe pages and their native USD face materials."""

from math import cos, radians, sin

from PIL import Image, ImageDraw, ImageFont

from . import config as C


PAPER = (245, 232, 201)
OLIVE = (36, 74, 44)
INK = (38, 40, 31)
YELLOW = (240, 191, 66)
PAGE_PIXELS = (500, 710)
# All recipe words and headings fit inside this rectangle on both pages.
RECIPE_PIXELS = (36, 25, 472, 365)


def recipe_bounds(side):
    """Return the recipe rectangle as lower and upper (y, z) page coordinates."""
    left, top, right, bottom = RECIPE_PIXELS
    width, height = PAGE_PIXELS
    y_max = C.sign(side) * C.PAGE_CENTER_Y + C.PAGE_WIDTH / 2
    page_height = C.PRINT_TOP - C.PRINT_BOTTOM
    return (
        (y_max - right / width * C.PAGE_WIDTH, C.PRINT_TOP - bottom / height * page_height),
        (y_max - left / width * C.PAGE_WIDTH, C.PRINT_TOP - top / height * page_height),
    )


def art(page_number):
    page = Image.new("RGB", PAGE_PIXELS, PAPER)
    draw = ImageDraw.Draw(page)

    def label(x, y, words, size, color=INK):
        draw.text((x, y), words, font=ImageFont.load_default(size), fill=color)

    def recipe(x, y, words, size, color=INK):
        font = ImageFont.load_default(size)
        bounds = draw.textbbox((x, y), words, font=font)
        left, top, right, bottom = RECIPE_PIXELS
        if not (left <= bounds[0] and top <= bounds[1] and bounds[2] <= right and bounds[3] <= bottom):
            raise ValueError(f"Recipe text exceeds the printable rectangle: {words}")
        draw.text((x, y), words, font=font, fill=color)

    recipe(44, 31, "FROM THE OVEN", 26, OLIVE)
    draw.line((44, 71, 456, 71), fill=OLIVE, width=3)
    recipe(42, 83, "Lemon Potatoes", 57, OLIVE)
    draw.rounded_rectangle((43, 163, 458, 208), radius=10, fill=OLIVE)
    if page_number == 24:
        recipe(62, 168, "INGREDIENTS  /  SERVES 4", 27, PAPER)
        for i, ingredient in enumerate((
            "800 g small yellow potatoes", "3 tbsp olive oil",
            "1 lemon, zest and juice", "2 rosemary sprigs",
            "1 tsp flaky sea salt"
        )):
            recipe(48, 220 + i * 27, ingredient, 24)
    else:
        recipe(62, 168, "METHOD  /  35 MIN", 27, PAPER)
        for i, line in enumerate((
            "Toss the potatoes with the oil,",
            "lemon zest and rosemary.",
            "Roast 35 minutes at 220 °C.",
            "Finish with lemon juice and salt.",
        )):
            recipe(48, 220 + i * 30, line, 24)
        draw.ellipse((130, 368, 372, 612), fill=(209, 179, 135), outline=OLIVE, width=4)
        draw.ellipse((142, 380, 359, 599), fill=(232, 221, 190), outline=(139, 129, 80), width=4)
        for i in range(15):
            a = 2.39996 * i
            r = 74 * (i / 14) ** .5
            x, y = 249 + r * cos(a), 488 + r * sin(a)
            draw.ellipse((x - 24, y - 15, x + 25, y + 15), fill=YELLOW,
                         outline=(145, 82, 31), width=3)
        for i in range(11):
            a = radians(i * 83)
            x, y = 250 + 57 * cos(a), 488 + 56 * sin(a)
            draw.line((x, y, x + 15 * cos(a), y + 15 * sin(a)), fill=OLIVE, width=4)
    draw.line((44, 641, 456, 641), fill=OLIVE, width=2)
    label(225, 660, str(page_number), 24, OLIVE)
    return page


def print_face(w, stage, body, *, center_y, page_number):
    from pxr import Sdf

    name = f"Page{page_number}"
    path = w.directory / (name + ".png")
    art(page_number).save(path)
    geom = w.Geom.Mesh.Define(stage, body + "/Print")
    y0, y1 = center_y + C.PAGE_WIDTH / 2, center_y - C.PAGE_WIDTH / 2
    geom.CreatePointsAttr([(C.PRINT_FRONT, y0, C.PRINT_BOTTOM), (C.PRINT_FRONT, y1, C.PRINT_BOTTOM),
                           (C.PRINT_FRONT, y1, C.PRINT_TOP), (C.PRINT_FRONT, y0, C.PRINT_TOP)])
    geom.CreateFaceVertexCountsAttr([4])
    geom.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    geom.CreateSubdivisionSchemeAttr("none")
    uv = w.Geom.PrimvarsAPI(geom).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                              w.Geom.Tokens.vertex)
    uv.Set([(0., 0.), (1., 0.), (1., 1.), (0., 1.)])
    root = "/Asset/Print" + name
    mat = w.Shade.Material.Define(stage, root)
    st = w.Shade.Shader.Define(stage, root + "/coordinates")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    img = w.Shade.Shader.Define(stage, root + "/page")
    img.CreateIdAttr("UsdUVTexture")
    img.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(path))
    img.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    img.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
    rgb = img.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surf = w.Shade.Shader.Define(stage, root + "/paper")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.8)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)
