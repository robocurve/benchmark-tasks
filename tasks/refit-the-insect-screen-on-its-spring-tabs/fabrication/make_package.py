"""Writes the fabrication package for the insect screen's two squeeze latches: dimensions.json, three dimensioned
drawing sheets (SVG, and PNG when Pillow is installed) and the printable parts as binary STL (when shapely 2.1 or
later is installed). Every number comes from the table below; near_misses.py checks that table against the
simulated model.

    python make_package.py          (run from this folder)

Coordinates of the left-hand latch (the one at the viewer's left, seen from the room), in millimetres:
u from the screen's side edge toward its middle, v up from the screen's bottom edge, w from the frame's room-side
face toward the room. The right-hand latch is the mirror image.
"""

from __future__ import annotations

import json
import math
import pathlib
import struct

HERE = pathlib.Path(__file__).resolve().parent

# -- every dimension (mm) ---------------------------------------------------------------------------------------------
SCREEN_W, SCREEN_H = 406.0, 356.0  # 16 x 14 in outside
FRAME_T, FRAME_FACE = 11.1, 19.05  # 7/16 x 3/4 in roll-formed profile
PLATE_W, PLATE_H, PLATE_T = 45.0, 65.0, 3.0  # housing plate on the room-side face, flush with the corner
TAB_T, TAB_H, TAB_PROUD = 6.0, 28.0, 30.0  # tab and lug: thick (u), tall (v), proud of the plate (w)
TAB_U0 = 16.0  # the tab's outer face at rest
TAB_V0 = 31.0  # bottom of tab and lug
TRAVEL = 11.5
LUG_U0 = TAB_U0 + TAB_T + TRAVEL  # 33.5: the squeezed tab closes onto this face
BOLT_D = 5.56  # 7/32 in music wire
BOLT_V = 45.0  # bolt axis above the screen's bottom edge
BOLT_DEPTH = FRAME_T / 2  # bolt axis behind the room-side face: mid-thickness of the frame
BOLT_OUT = 9.5  # tip proud of the screen's side, tab at rest
BOLT_LEN = 40.0
NOSE = 2.0  # bullet nose ground on the tip
FLAT = (2.5, 8.5, 0.8)  # retention flat: from, to (from the inner end), depth
PLAY = 3.0  # screen side to jamb face, each side, screen centred
HOLE_W, HOLE_H, HOLE_DEPTH = 10.0, 8.0, 12.0  # jamb hole: front to back, tall, deep
LATCH_MIN = 3.0  # a latched bolt's tip is at least this far past the jamb's face
TAB_TO_TIP = TAB_U0 + BOLT_OUT  # 25.5: tab's outer face to the bolt's tip, a fixed length
TAB_TO_JAMB_MAX = TAB_TO_TIP - LATCH_MIN  # 22.5: the real-trial reading of "latched"
TAB_TO_JAMB_REST = TAB_U0 + PLAY  # 19.0: latch fully out, screen centred
TAB_TO_JAMB_SHUT = TAB_TO_JAMB_REST + TRAVEL  # 30.5: tab squeezed onto the lug, screen centred
JAMB_DEPTH = 45.0  # jamb's room-side face to the seated screen's room-side face
SEAT_TOL, SILL_TOL = 2.0, 1.5

SLOT = (19.6, 33.6, 30.7, 59.3)  # u0, u1, v0, v1: slot through the plate for the slide's neck
MOUNT = ((8.0, 26.0), (8.0, 60.0), (37.0, 9.5))  # housing-to-frame screws (u, v)
LUG_SCREW = ((36.5, 37.0), (36.5, 53.0))  # lug screws, from the back of the plate, on the lug's centre line
CLEAR_D, CBORE_D, CBORE_DEEP, PILOT_D = 3.2, 6.2, 1.8, 2.4  # No. 4 screws
POST = (42.5, 45.0, 49.0, 60.0, 10.0)  # u0, u1, v0, v1, depth behind the plate: the spring's fixed seat
SPRING_V = 54.5  # spring axis (same depth as the bolt axis)
PEG = (3.0, 3.4)  # spring locating peg on the post: long, square
NECK_U0 = 19.6
CARR = (19.6, 30.6, 31.0, 62.5, 0.3, 10.3)  # carriage at rest: u0, u1, v0, v1, front and back depth behind the face
BOLT_BORE, SPRING_BORE = 5.7, 6.6
SPRING_SEAT_U = 22.5  # bottom of the spring bore at rest
RETAIN = (25.1, 2.4)  # retention screw pilot: u at rest, square side
TAB_GAP = 0.2  # the tab's underside rides this far above the plate
STILE_HOLE = 6.0  # through both walls of the stile
GAUGE = (43.0, 40.0, 22.5, 1.5)  # long edge, stepped edge, width, thickness

SPRING = {"part": "Lee Spring LC 016C 12 M", "rate_N_per_mm": 0.263, "free_length": 23.83, "solid_height": 3.28,
          "outside_diameter": 6.1, "wire": 0.41}
SPRING["length_at_rest"] = POST[0] - SPRING_SEAT_U
SPRING["length_squeezed"] = SPRING["length_at_rest"] - TRAVEL
SPRING["force_at_rest_N"] = round(SPRING["rate_N_per_mm"] * (SPRING["free_length"] - SPRING["length_at_rest"]), 2)
SPRING["force_squeezed_N"] = round(SPRING["rate_N_per_mm"] * (SPRING["free_length"] - SPRING["length_squeezed"]), 2)

DIMENSIONS = {
    "units": "mm",
    "screen": {"width": SCREEN_W, "height": SCREEN_H, "frame_thickness": FRAME_T, "frame_face": FRAME_FACE},
    "housing_plate": {"width": PLATE_W, "height": PLATE_H, "thickness": PLATE_T},
    "tab_and_lug": {"thickness": TAB_T, "height": TAB_H, "proud_of_plate": TAB_PROUD, "bottom_above_screen_bottom":
                    TAB_V0, "tab_outer_face_from_side_at_rest": TAB_U0, "lug_face_from_side": LUG_U0,
                    "travel": TRAVEL},
    "bolt": {"diameter": BOLT_D, "axis_above_screen_bottom": BOLT_V, "axis_behind_room_face": BOLT_DEPTH,
             "proud_of_side_at_rest": BOLT_OUT, "inside_side_when_squeezed": TRAVEL - BOLT_OUT, "length": BOLT_LEN},
    "window": {"play_each_side": PLAY, "hole_front_to_back": HOLE_W, "hole_tall": HOLE_H, "hole_deep": HOLE_DEPTH,
               "jamb_face_to_seated_screen_face": JAMB_DEPTH},
    "reading": {"tab_outer_face_to_bolt_tip": TAB_TO_TIP, "bolt_past_jamb_face_min": LATCH_MIN,
                "latched_if_tab_outer_face_to_jamb_at_most": TAB_TO_JAMB_MAX,
                "fully_out_centred": TAB_TO_JAMB_REST, "squeezed_shut_centred": TAB_TO_JAMB_SHUT,
                "seated_if_screen_face_behind_jamb_face_at_least": JAMB_DEPTH - SEAT_TOL,
                "on_sill_if_feeler_does_not_enter": SILL_TOL},
    "spring": SPRING,
}


# -- printable parts: stacks of prisms ---------------------------------------------------------------------------------
def build_parts():
    """Each part: (stack axis, [(from, to, shapely region), ...]). Axis 'w': regions in (u, v); axis 'u': in (v, w)."""
    import shapely
    from shapely.geometry import Polygon, box

    def disc(cx, cy, d, n=48):
        return Polygon([(cx + d / 2 * math.cos(2 * math.pi * i / n), cy + d / 2 * math.sin(2 * math.pi * i / n))
                        for i in range(n)])

    def cut(region, *holes):
        for h in holes:
            region = shapely.difference(region, h, grid_size=GRID)
        return region

    plate = cut(box(0, 0, PLATE_W, PLATE_H), box(SLOT[0], SLOT[2], SLOT[1], SLOT[3]))
    post = box(POST[0], POST[2], POST[1], POST[3])
    peg = box(POST[0] - PEG[0], SPRING_V - PEG[1] / 2, POST[0], SPRING_V + PEG[1] / 2)
    m_hole = [disc(u, v, CLEAR_D) for u, v in MOUNT]
    m_bore = [disc(u, v, CBORE_D) for u, v in MOUNT]
    l_hole = [disc(u, v, CLEAR_D) for u, v in LUG_SCREW]
    l_bore = [disc(u, v, CBORE_D) for u, v in LUG_SCREW]
    housing = ("w", [
        (-POST[4], -BOLT_DEPTH - PEG[1] / 2, post),
        (-BOLT_DEPTH - PEG[1] / 2, -BOLT_DEPTH + PEG[1] / 2, shapely.union_all([post, peg], grid_size=GRID)),
        (-BOLT_DEPTH + PEG[1] / 2, 0.0, post),
        (0.0, PLATE_T - CBORE_DEEP, cut(plate, *m_hole, *l_bore)),
        (PLATE_T - CBORE_DEEP, CBORE_DEEP, cut(plate, *m_bore, *l_bore)),
        (CBORE_DEEP, PLATE_T, cut(plate, *m_bore, *l_hole)),
    ])
    lug_r = box(LUG_U0, TAB_V0, LUG_U0 + TAB_T, TAB_V0 + TAB_H)
    lug = ("w", [(PLATE_T, PLATE_T + 9.0, cut(lug_r, *[disc(u, v, PILOT_D) for u, v in LUG_SCREW])),
                 (PLATE_T + 9.0, PLATE_T + TAB_PROUD, lug_r)])
    # the slide, in (v, w), stacked along u
    tab = box(TAB_V0, PLATE_T + TAB_GAP, TAB_V0 + TAB_H, PLATE_T + TAB_PROUD)
    neck = box(TAB_V0, -CARR[4], TAB_V0 + TAB_H, PLATE_T + TAB_GAP)
    carr = box(CARR[2], -CARR[5], CARR[3], -CARR[4])
    bolt = disc(BOLT_V, -BOLT_DEPTH, BOLT_BORE)
    spring = disc(SPRING_V, -BOLT_DEPTH, SPRING_BORE)
    pilot = box(CARR[2], -BOLT_DEPTH - RETAIN[1] / 2, BOLT_V, -BOLT_DEPTH + RETAIN[1] / 2)
    r0, r1 = RETAIN[0] - RETAIN[1] / 2, RETAIN[0] + RETAIN[1] / 2
    slide = ("u", [
        (TAB_U0, NECK_U0, tab),
        (NECK_U0, TAB_U0 + TAB_T, cut(shapely.union_all([tab, neck, carr], grid_size=GRID), bolt)),
        (TAB_U0 + TAB_T, SPRING_SEAT_U, cut(carr, bolt)),
        (SPRING_SEAT_U, r0, cut(carr, bolt, spring)),
        (r0, r1, cut(carr, bolt, spring, pilot)),
        (r1, CARR[1], cut(carr, bolt, spring)),
    ])
    g = Polygon([(0, 0), (GAUGE[0], 0), (GAUGE[0], GAUGE[2] / 2), (GAUGE[1], GAUGE[2] / 2), (GAUGE[1], GAUGE[2]),
                 (0, GAUGE[2])])
    gauge = ("w", [(0.0, GAUGE[3], g)])
    return {"housing": housing, "lug": lug, "slide": slide, "gauge-card": gauge}


GRID = 1e-5  # mm: every outline is snapped to this grid, so layers that share an edge share its points


def _key(p):
    return (round(p[0], 5), round(p[1], 5))


def _polys(g):
    from shapely.geometry.polygon import orient

    if g.is_empty:
        return []
    geoms = g.geoms if hasattr(g, "geoms") else [g]
    return [orient(p, 1.0) for p in geoms if p.geom_type == "Polygon" and p.area > 1e-6]


def _rings(p):
    return [list(r.coords)[:-1] for r in (p.exterior, *p.interiors)]


def _on_segment(a, b, pts):
    """Points of ``pts`` strictly inside segment a-b, sorted from a."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    ll = dx * dx + dy * dy
    out = []
    for p in pts:
        t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / ll
        if 1e-9 < t < 1 - 1e-9 and abs((p[0] - a[0]) * dy - (p[1] - a[1]) * dx) / math.sqrt(ll) < 3 * GRID:
            out.append((t, p))
    return [p for _, p in sorted(out)]


def _densify(ring, pts):
    out = []
    for i, a in enumerate(ring):
        b = ring[(i + 1) % len(ring)]
        out.append(a)
        out.extend(_on_segment(a, b, pts))
    return out


def mesh(axis, stack):
    """Closed triangle mesh of a prism stack, vertices (u, v, w). Faces between layers are only where one layer
    overhangs the other, and every edge is split at the same points on both of its faces."""
    import shapely
    from shapely import constrained_delaunay_triangles
    from shapely.geometry import Polygon

    def to3(p, t):
        return (p[0], p[1], t) if axis == "w" else (t, p[0], p[1])

    def minus(a, b):
        return shapely.difference(a, b, grid_size=GRID)

    stack = [(t0, t1, shapely.set_precision(region, GRID)) for t0, t1, region in stack]
    n = len(stack)
    planes = []  # per interface (0..n): (height, up-facing regions, down-facing regions, vertex set)
    for k in range(n + 1):
        below = stack[k - 1][2] if k > 0 else None
        above = stack[k][2] if k < n else None
        t = stack[k][0] if k < n else stack[k - 1][1]
        up = _polys(below if above is None else minus(below, above)) if below is not None else []
        down = _polys(above if below is None else minus(above, below)) if above is not None else []
        pts = {}
        for g in ([below] if below is not None else []) + ([above] if above is not None else []):
            for p in _polys(g):
                for r in _rings(p):
                    for c in r:
                        pts[_key(c)] = (round(c[0], 6), round(c[1], 6))
        for p in up + down:
            for r in _rings(p):
                for c in r:
                    pts[_key(c)] = (round(c[0], 6), round(c[1], 6))
        planes.append((t, up, down, list(pts.values())))
    tris = []
    for t, up, down, pts in planes:
        for regions, facing_up in ((up, True), (down, False)):
            for p in regions:
                rings = [_densify([(round(c[0], 6), round(c[1], 6)) for c in r], pts) for r in _rings(p)]
                dense = Polygon(rings[0], rings[1:])
                for tri in constrained_delaunay_triangles(dense).geoms:
                    c = [(round(x, 6), round(y, 6)) for x, y in list(tri.exterior.coords)[:3]]
                    area = (c[1][0] - c[0][0]) * (c[2][1] - c[0][1]) - (c[2][0] - c[0][0]) * (c[1][1] - c[0][1])
                    if abs(area) < 1e-9:
                        continue
                    if (area > 0) != facing_up:
                        c = [c[0], c[2], c[1]]
                    tris.append([to3(q, t) for q in c])
    for k, (t0, t1, region) in enumerate(stack):
        lo, hi = planes[k][3], planes[k + 1][3]
        for p in _polys(region):
            for ring in _rings(p):
                ring = [(round(c[0], 6), round(c[1], 6)) for c in ring]
                for i, a in enumerate(ring):
                    b = ring[(i + 1) % len(ring)]
                    bot = [a, *_on_segment(a, b, lo), b]
                    top = [a, *_on_segment(a, b, hi), b]

                    def par(q):
                        return (q[0] - a[0]) * (b[0] - a[0]) + (q[1] - a[1]) * (b[1] - a[1])

                    i_, j_ = 0, 0
                    while i_ < len(bot) - 1 or j_ < len(top) - 1:
                        if i_ < len(bot) - 1 and (j_ == len(top) - 1 or par(bot[i_ + 1]) <= par(top[j_ + 1])):
                            tris.append([to3(bot[i_], t0), to3(bot[i_ + 1], t0), to3(top[j_], t1)])
                            i_ += 1
                        else:
                            tris.append([to3(bot[i_], t0), to3(top[j_ + 1], t1), to3(top[j_], t1)])
                            j_ += 1
    return tris


def check_closed(tris):
    """Every directed edge must be matched by exactly one opposite edge; returns the enclosed volume (mm3)."""
    edges = {}
    for tri in tris:
        k = [tuple(round(x, 5) for x in p) for p in tri]
        for i in range(3):
            e = (k[i], k[(i + 1) % 3])
            edges[e] = edges.get(e, 0) + 1
    bad = [e for e, c in edges.items() if c != 1 or edges.get((e[1], e[0]), 0) != 1]
    if bad:
        raise ValueError(f"{len(bad)} open or doubled edges, e.g. {bad[0]}")
    vol = 0.0
    for a, b, c in tris:
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    return vol


def write_stl(path, tris, mirror=False):
    if mirror:  # the right-hand part: u reversed, facets turned to keep the outside out
        tris = [[(-a[0], a[1], a[2]), (-c[0], c[1], c[2]), (-b[0], b[1], b[2])] for a, b, c in tris]
    lo = [min(p[i] for t in tris for p in t) for i in range(3)]
    with open(path, "wb") as f:
        f.write(path.name.encode().ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            a, b, c = ([p[i] - lo[i] for i in range(3)] for p in (a, b, c))
            ux, uy, uz = (b[i] - a[i] for i in range(3))
            vx, vy, vz = (c[i] - a[i] for i in range(3))
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
            f.write(struct.pack("<12fH", nx / ln, ny / ln, nz / ln, *a, *b, *c, 0))


def make_stl():
    out = HERE / "stl"
    out.mkdir(exist_ok=True)
    report = {}
    for name, (axis, stack) in build_parts().items():
        tris = mesh(axis, stack)
        vol = check_closed(tris)
        want = sum((t1 - t0) * region.area for t0, t1, region in stack)
        if abs(vol - want) > 1e-3 * want:
            raise ValueError(f"{name}: volume {vol:.2f} mm3, expected {want:.2f}")
        handed = name in ("housing", "slide")
        for suffix, mirror in ((("-left", False), ("-right", True)) if handed else (("", False),)):
            write_stl(out / f"{name}{suffix}.stl", tris, mirror)
        report[name] = {"triangles": len(tris), "volume_mm3": round(vol, 1), "closed": True}
    return report


# -- drawings ---------------------------------------------------------------------------------------------------------
INK, THIN, GREY = "#111111", "#444444", "#777777"
WOOD, ALU, PRINT, PRINT_D, STEEL, RED = "#eadfc8", "#d5dade", "#b9b9b9", "#6f6f6f", "#8d9aa8", "#b3261e"


class Sheet:
    def __init__(self, w, h):
        self.w, self.h, self.items = w, h, []

    def line(self, x1, y1, x2, y2, width=1.0, color=INK, dash=None):
        self.items.append(("line", x1, y1, x2, y2, width, color, dash))

    def poly(self, pts, fill=None, stroke=INK, width=1.0, dash=None):
        self.items.append(("poly", list(pts), fill, stroke, width, dash))

    def circle(self, cx, cy, r, fill=None, stroke=INK, width=1.0, dash=None):
        self.items.append(("circle", cx, cy, r, fill, stroke, width, dash))

    def text(self, x, y, s, size=13, anchor="start", bold=False, color=INK):
        self.items.append(("text", x, y, str(s), size, anchor, bold, color))

    def svg(self):
        def da(d):
            return f' stroke-dasharray="{",".join(str(v) for v in d)}"' if d else ""

        o = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
             f'viewBox="0 0 {self.w} {self.h}" font-family="Arial, Helvetica, sans-serif">',
             f'<rect width="{self.w}" height="{self.h}" fill="#ffffff"/>']
        for it in self.items:
            if it[0] == "line":
                _, x1, y1, x2, y2, wd, col, d = it
                o.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="{col}" '
                         f'stroke-width="{wd}"{da(d)}/>')
            elif it[0] == "poly":
                _, pts, fill, stroke, wd, d = it
                p = " ".join(f"{x:.1f},{y:.1f}" for x, y in pts)
                o.append(f'<polygon points="{p}" fill="{fill or "none"}" stroke="{stroke or "none"}" '
                         f'stroke-width="{wd}"{da(d)}/>')
            elif it[0] == "circle":
                _, cx, cy, r, fill, stroke, wd, d = it
                o.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill or "none"}" '
                         f'stroke="{stroke or "none"}" stroke-width="{wd}"{da(d)}/>')
            else:
                _, x, y, s, size, anchor, bold, col = it
                s = s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
                o.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" text-anchor="{anchor}" fill="{col}"'
                         f'{" font-weight=\"bold\"" if bold else ""}>{s}</text>')
        o.append("</svg>")
        return "\n".join(o) + "\n"

    def png(self, path, k=2):
        from PIL import Image, ImageDraw, ImageFont

        img = Image.new("RGB", (self.w * k, self.h * k), "#ffffff")
        dr = ImageDraw.Draw(img)
        fonts = {}

        def font(size, bold):
            if (size, bold) not in fonts:
                try:
                    name = "Arial Bold.ttf" if bold else "Arial.ttf"
                    fonts[(size, bold)] = ImageFont.truetype("/System/Library/Fonts/Supplemental/" + name, size * k)
                except OSError:
                    fonts[(size, bold)] = ImageFont.load_default(size * k)
            return fonts[(size, bold)]

        def dashed(x1, y1, x2, y2, wd, col, d):
            ln = math.hypot(x2 - x1, y2 - y1)
            if not d or ln == 0:
                dr.line([x1 * k, y1 * k, x2 * k, y2 * k], fill=col, width=max(1, round(wd * k)))
                return
            pos, i = 0.0, 0
            while pos < ln:
                seg = d[i % len(d)]
                if i % 2 == 0:
                    a, b = pos / ln, min(ln, pos + seg) / ln
                    dr.line([(x1 + (x2 - x1) * a) * k, (y1 + (y2 - y1) * a) * k, (x1 + (x2 - x1) * b) * k,
                             (y1 + (y2 - y1) * b) * k], fill=col, width=max(1, round(wd * k)))
                pos += seg
                i += 1

        for it in self.items:
            if it[0] == "line":
                dashed(*it[1:])
            elif it[0] == "poly":
                _, pts, fill, stroke, wd, d = it
                if fill:
                    dr.polygon([(x * k, y * k) for x, y in pts], fill=fill)
                if stroke:
                    for i, a in enumerate(pts):
                        b = pts[(i + 1) % len(pts)]
                        dashed(a[0], a[1], b[0], b[1], wd, stroke, d)
            elif it[0] == "circle":
                _, cx, cy, r, fill, stroke, wd, d = it
                if fill:
                    dr.ellipse([(cx - r) * k, (cy - r) * k, (cx + r) * k, (cy + r) * k], fill=fill)
                if stroke:
                    n = 72
                    pts = [(cx + r * math.cos(2 * math.pi * i / n), cy + r * math.sin(2 * math.pi * i / n))
                           for i in range(n + 1)]
                    for i in range(n):
                        if not d or i % 3 != 2:
                            dr.line([pts[i][0] * k, pts[i][1] * k, pts[i + 1][0] * k, pts[i + 1][1] * k],
                                    fill=stroke, width=max(1, round(wd * k)))
            else:
                _, x, y, s, size, anchor, bold, col = it
                f = font(size, bold)
                wd = dr.textlength(s, font=f)
                x0 = x * k - (wd if anchor == "end" else wd / 2 if anchor == "middle" else 0)
                dr.text((x0, y * k), s, font=f, fill=col, anchor="ls")
        img.save(path, optimize=True)


class View:
    """A scaled view on a sheet: millimetres in, y up (or down the sheet with ``down``)."""

    def __init__(self, sheet, ox, oy, k, down=False):
        self.s, self.ox, self.oy, self.k, self.down = sheet, ox, oy, k, down

    def X(self, x):
        return self.ox + x * self.k

    def Y(self, y):
        return self.oy + y * self.k if self.down else self.oy - y * self.k

    def rect(self, x0, y0, x1, y1, fill=None, stroke=INK, width=1.8, dash=None):
        self.s.poly([(self.X(x0), self.Y(y0)), (self.X(x1), self.Y(y0)), (self.X(x1), self.Y(y1)),
                     (self.X(x0), self.Y(y1))], fill, stroke, width, dash)

    def poly(self, pts, fill=None, stroke=INK, width=1.8, dash=None):
        self.s.poly([(self.X(x), self.Y(y)) for x, y in pts], fill, stroke, width, dash)

    def line(self, x0, y0, x1, y1, width=1.0, color=INK, dash=None):
        self.s.line(self.X(x0), self.Y(y0), self.X(x1), self.Y(y1), width, color, dash)

    def circle(self, x, y, d, fill=None, stroke=INK, width=1.4, dash=None):
        self.s.circle(self.X(x), self.Y(y), d / 2 * self.k, fill, stroke, width, dash)

    def text(self, x, y, s, dx=0, dy=0, **kw):
        self.s.text(self.X(x) + dx, self.Y(y) + dy, s, **kw)

    def centre(self, x0, y0, x1, y1):
        self.line(x0, y0, x1, y1, 0.8, GREY, (14, 3, 3, 3))

    def _arrow(self, x, y, dx, dy, color):
        a, h = 8.0, 2.6
        self.s.poly([(x, y), (x - dx * a - dy * h, y - dy * a + dx * h), (x - dx * a + dy * h, y - dy * a - dx * h)],
                    color, None)

    def hdim(self, x1, x2, y, label=None, frm=None, color=THIN, size=13, side=0):
        """Horizontal dimension between x1 and x2 with its line at y; extension lines from ``frm`` (y or (y1, y2))."""
        X1, X2, Yd = self.X(x1), self.X(x2), self.Y(y)
        if frm is not None:
            f1, f2 = frm if isinstance(frm, tuple) else (frm, frm)
            for X, f in ((X1, f1), (X2, f2)):
                Yf = self.Y(f)
                self.s.line(X, Yf + (3 if Yd > Yf else -3), X, Yd + (4 if Yd > Yf else -4), 0.7, color)
        label = f"{abs(x2 - x1):g}" if label is None else label
        tw = 0.56 * size * len(label)
        if X2 - X1 >= 22:
            self.s.line(X1, Yd, X2, Yd, 0.8, color)
            self._arrow(X1, Yd, -1, 0, color)
            self._arrow(X2, Yd, 1, 0, color)
        else:
            self.s.line(X1 - 16, Yd, X2 + 16, Yd, 0.8, color)
            self._arrow(X1, Yd, 1, 0, color)
            self._arrow(X2, Yd, -1, 0, color)
        if side == 0 and X2 - X1 >= tw + 20:
            self.s.text((X1 + X2) / 2, Yd - 4, label, size=size, anchor="middle", color=color)
        elif side < 0:
            self.s.text(X1 - 20, Yd + 4, label, size=size, anchor="end", color=color)
        else:
            self.s.text(X2 + 20, Yd + 4, label, size=size, anchor="start", color=color)

    def vdim(self, y1, y2, x, label=None, frm=None, color=THIN, size=13, side=-1):
        """Vertical dimension between y1 and y2 with its line at x; label left (side -1) or right (+1) of it."""
        Y1, Y2, Xd = self.Y(y1), self.Y(y2), self.X(x)
        if frm is not None:
            f1, f2 = frm if isinstance(frm, tuple) else (frm, frm)
            for Y, f in ((Y1, f1), (Y2, f2)):
                Xf = self.X(f)
                self.s.line(Xf + (3 if Xd > Xf else -3), Y, Xd + (4 if Xd > Xf else -4), Y, 0.7, color)
        label = f"{abs(y2 - y1):g}" if label is None else label
        if abs(Y2 - Y1) >= 22:
            self.s.line(Xd, Y1, Xd, Y2, 0.8, color)
            self._arrow(Xd, Y1, 0, 1 if Y1 > Y2 else -1, color)
            self._arrow(Xd, Y2, 0, -1 if Y1 > Y2 else 1, color)
            ym = (Y1 + Y2) / 2 + 4
        else:
            lo, hi = min(Y1, Y2), max(Y1, Y2)
            self.s.line(Xd, lo - 16, Xd, hi + 16, 0.8, color)
            self._arrow(Xd, lo, 0, 1, color)
            self._arrow(Xd, hi, 0, -1, color)
            ym = lo - 20 if side == 0 else (Y1 + Y2) / 2 + 4
        if side == 0:
            self.s.text(Xd, ym, label, size=size, anchor="middle", color=color)
        else:
            self.s.text(Xd + 6 * side, ym, label, size=size, anchor="end" if side < 0 else "start", color=color)


def _title(s, title, sub):
    s.text(30, 38, title, size=20, bold=True)
    s.text(30, 60, sub, size=13, color=THIN)
    s.line(30, 70, s.w - 30, 70, 1.2)


def _notes(s, x, y, lines, size=13, gap=19):
    for i, ln in enumerate(lines):
        bold = ln.startswith("*")
        s.text(x, y + i * gap, ln.lstrip("*"), size=size, bold=bold)


def sheet_assembly():
    s = Sheet(1640, 1200)
    _title(s, "Sheet 1. Squeeze latch, assembled: left-hand latch seen from the room",
           "Millimetres. The right-hand latch is the mirror image. u: from the screen's side edge toward its middle; "
           "v: up from the screen's bottom edge; w: from the frame's room-side face toward the room.")
    k = 8.0
    # -- front view --------------------------------------------------------------------------------------------------
    a = View(s, 260, 850, k)
    top = 76
    s.text(60, 100, "Front view (from the room), screen seated and centred, tab at rest", size=15, bold=True)
    a.rect(-22, -6, -PLAY, top, WOOD, INK, 1.2)  # jamb
    a.rect(-22, -6, 62, 0, WOOD, INK, 1.2)  # sill
    a.text(-21, 70, "jamb", size=13)
    a.text(52, -4.6, "sill", size=13)
    a.rect(-PLAY - HOLE_DEPTH, BOLT_V - HOLE_H / 2, -PLAY, BOLT_V + HOLE_H / 2, "#ffffff", THIN, 1.0, (5, 4))
    a.rect(0, 0, FRAME_FACE, top, ALU, INK, 1.2)  # stile
    a.rect(0, 0, 62, FRAME_FACE, ALU, INK, 1.2)  # bottom rail
    a.text(1.2, 71, "stile", size=13)
    a.text(52, 2.5, "bottom rail", size=13)
    a.text(40, 71, "mesh", size=13, color=GREY)
    a.rect(0, 0, PLATE_W, PLATE_H, PRINT, INK, 2.2)  # plate
    a.rect(SLOT[0], SLOT[2], SLOT[1], SLOT[3], "#e9e9e9", INK, 1.0)  # slot, backed by the carriage
    a.rect(CARR[0], CARR[2], CARR[1], CARR[3], None, THIN, 1.0, (5, 4))  # carriage (hidden)
    a.rect(POST[0], POST[2], POST[1], POST[3], None, THIN, 1.0, (5, 4))  # post (hidden)
    n = 12  # spring (hidden), drawn as a zigzag
    zz = [(SPRING_SEAT_U + (POST[0] - SPRING_SEAT_U) * i / n, SPRING_V + (2.6 if i % 2 else -2.6))
          for i in range(n + 1)]
    for p, q in zip(zz, zz[1:]):
        a.line(p[0], p[1], q[0], q[1], 1.0, THIN)
    # bolt: hidden inside the frame and the jamb, seen in the gap
    a.rect(-BOLT_OUT, BOLT_V - BOLT_D / 2, CARR[1] - 0.1, BOLT_V + BOLT_D / 2, None, THIN, 1.0, (5, 4))
    a.rect(-PLAY, BOLT_V - BOLT_D / 2, 0, BOLT_V + BOLT_D / 2, STEEL, INK, 1.2)
    a.centre(-17, BOLT_V, 46, BOLT_V)
    a.rect(TAB_U0 + TRAVEL, TAB_V0, TAB_U0 + TRAVEL + TAB_T, TAB_V0 + TAB_H, None, RED, 1.2, (6, 4))  # tab squeezed
    a.rect(TAB_U0, TAB_V0, TAB_U0 + TAB_T, TAB_V0 + TAB_H, PRINT_D, INK, 2.2)  # tab at rest
    a.rect(LUG_U0, TAB_V0, LUG_U0 + TAB_T, TAB_V0 + TAB_H, "#3d3d3d", INK, 2.2)  # lug
    for u, v in MOUNT:
        a.circle(u, v, CBORE_D, "#ffffff", INK, 1.2)
        a.line(u - 2.2, v - 2.2, u + 2.2, v + 2.2, 1.0)
        a.line(u - 2.2, v + 2.2, u + 2.2, v - 2.2, 1.0)
    for num, (x, y), (tx, ty) in ((1, (42, 26), (57, 31)), (2, (19, 58), (12, 71)), (3, (36.5, 58), (31, 71)),
                                  (4, (-1.5, 46.5), (-9, 60)), (5, (41, 55), (57, 56)), (6, (8, 26), (-10, 16)),
                                  (7, (25.1, 30.2), (29, 22.5))):
        a.line(x, y, tx, ty, 0.8, THIN)
        s.circle(a.X(tx), a.Y(ty), 11, "#ffffff", INK, 1.2)
        s.text(a.X(tx), a.Y(ty) + 5, num, size=13, anchor="middle", bold=True)
    a.hdim(0, TAB_U0, -11, frm=(0, TAB_V0))
    a.hdim(TAB_U0, TAB_U0 + TAB_T, -11, frm=TAB_V0, label="6")
    a.hdim(TAB_U0 + TAB_T, LUG_U0, -11, frm=TAB_V0, label="11.5")
    a.hdim(LUG_U0, LUG_U0 + TAB_T, -11, frm=TAB_V0, label="6")
    a.text(TAB_U0, -15.2, "tab 6, travel 11.5, lug 6", size=13, color=THIN)
    a.hdim(0, PLATE_W, -20, frm=0)
    a.vdim(0, PLATE_H, 70, frm=(62, PLATE_W), side=1)
    a.vdim(TAB_V0, TAB_V0 + TAB_H, 50, frm=LUG_U0 + TAB_T, side=1)
    a.vdim(0, TAB_V0, 50, frm=(62, LUG_U0 + TAB_T), side=1, label="31")
    a.vdim(0, BOLT_V, -17.5, frm=(-22, -17), side=-1, label="45")
    a.hdim(-PLAY, 0, 79.5, frm=top, label="3 play", side=1)
    a.hdim(-PLAY, TAB_U0, 84.5, frm=(top, TAB_V0 + TAB_H), color=RED, side=1,
           label="19.0 at rest and centred. LATCHED if 22.5 or less")
    a.hdim(-BOLT_OUT, 0, 36, frm=BOLT_V - BOLT_D / 2, label="9.5", side=-1)
    # -- section through the bolt ------------------------------------------------------------------------------------
    b = View(s, 1040, 400, k, down=True)
    s.text(840, 100, "Section through the bolt axis, seen from above (room at the bottom)", size=15, bold=True)
    back = -FRAME_T
    b.poly([(-22, 40), (-PLAY, 40), (-PLAY, back), (10, back), (10, back - 12), (-22, back - 12)], WOOD, INK, 1.2)
    b.line(-PLAY, back, -22, back, 0.7, GREY, (4, 4))
    b.text(-21, 36, "jamb", size=13)
    b.text(-21, back - 5, "stop", size=13)
    b.rect(-PLAY - HOLE_DEPTH, -BOLT_DEPTH - HOLE_W / 2, -PLAY, -BOLT_DEPTH + HOLE_W / 2, "#ffffff", INK, 1.2)
    b.rect(0, back, FRAME_FACE, 0, ALU, INK, 1.2)
    b.rect(0.8, back + 0.8, FRAME_FACE - 0.8, -0.8, "#ffffff", THIN, 0.8)
    b.line(FRAME_FACE, back, 62, back, 1.0, GREY, (3, 3))
    b.text(47, back - 1.5, "mesh, on the back face", size=13, color=GREY, anchor="middle")
    b.rect(0, 0, PLATE_W, PLATE_T, PRINT, INK, 1.8)
    b.rect(POST[0], -POST[4], POST[1], 0, PRINT, INK, 1.8)
    b.rect(LUG_U0, PLATE_T, LUG_U0 + TAB_T, PLATE_T + TAB_PROUD, "#3d3d3d", INK, 1.8)
    b.rect(CARR[0], -CARR[5], CARR[1], -CARR[4], PRINT_D, INK, 1.8)
    b.rect(NECK_U0, -CARR[4], TAB_U0 + TAB_T, PLATE_T + TAB_GAP, PRINT_D, None)
    b.rect(TAB_U0, PLATE_T + TAB_GAP, TAB_U0 + TAB_T, PLATE_T + TAB_PROUD, PRINT_D, INK, 1.8)
    b.rect(TAB_U0 + TRAVEL, PLATE_T + TAB_GAP, TAB_U0 + TRAVEL + TAB_T, PLATE_T + TAB_PROUD, None, RED, 1.2, (6, 4))
    b.poly([(-BOLT_OUT + NOSE, -BOLT_DEPTH - BOLT_D / 2), (CARR[1] - 0.1, -BOLT_DEPTH - BOLT_D / 2),
            (CARR[1] - 0.1, -BOLT_DEPTH + BOLT_D / 2), (-BOLT_OUT + NOSE, -BOLT_DEPTH + BOLT_D / 2),
            (-BOLT_OUT, -BOLT_DEPTH + 1.5), (-BOLT_OUT, -BOLT_DEPTH - 1.5)], STEEL, INK, 1.4)
    b.centre(-17, -BOLT_DEPTH, 46, -BOLT_DEPTH)
    b.text(23, 20, "tab", size=13)
    b.text(40.5, 20, "lug", size=13)
    b.text(24, 39, "tab squeezed", size=13, color=RED)
    b.hdim(-PLAY, TAB_U0, 37.5, frm=(40, PLATE_T + TAB_PROUD), color=RED, label="19.0", side=-1)
    b.hdim(-BOLT_OUT, TAB_U0, -27, frm=(-BOLT_DEPTH - 1.5, back - 12), label="25.5 tab outer face to bolt tip",
           side=1)
    b.hdim(-BOLT_OUT, -PLAY, -31, frm=(-BOLT_DEPTH - 1.5, back - 12), label="6.5 in the hole (3 or more = latched)",
           side=1, color=RED)
    b.hdim(-PLAY - HOLE_DEPTH, -PLAY, 5, frm=-BOLT_DEPTH + HOLE_W / 2, label="12", side=-1)
    b.vdim(PLATE_T, PLATE_T + TAB_PROUD, 49, frm=(LUG_U0 + TAB_T, PLATE_W), side=1, label="30")
    b.vdim(0, PLATE_T, 49, frm=PLATE_W, side=1, label="3")
    b.vdim(-BOLT_DEPTH, 0, 55, frm=(46, PLATE_W), side=1, label="5.55")
    b.vdim(back, 0, 64, frm=(FRAME_FACE, 55), side=1, label="11.1")
    # -- parts list and the reading ----------------------------------------------------------------------------------
    _notes(s, 840, 800, [
        "*Parts of one latch",
        "1  Housing: plate 45 x 65 x 3 with the spring post behind it (printed, housing-left.stl / -right.stl)",
        "2  Slide: tab, neck and carriage in one piece (printed, slide-left.stl / -right.stl)",
        "3  Lug 6 x 28 x 30 (printed, lug.stl), two screws from the back of the plate",
        "4  Bolt: 7/32 in (5.56) music wire, 40 long, bullet nose, retention flat (sheet 2)",
        "5  Spring: Lee Spring LC 016C 12 M, 20.0 long at rest (1.0 N), 8.5 squeezed (4.0 N)",
        "6  No. 4 x 3/8 in flat-head sheet-metal screws: 3 into the frame, 2 into the lug",
        "7  No. 4 x 1/2 in pan-head sheet-metal screw, from below, onto the bolt's flat",
        "",
        "*Reading the latch in a real trial (from the room, no view into the jamb)",
        "The tab and the bolt are one rigid piece: the bolt's tip is always 25.5 beyond the tab's outer face.",
        "Measure from the tab's outer face to the jamb's face: 22.5 or less means the tip is 3 or more past",
        "the jamb's face, and it can only be there inside the hole. Fully out and centred it reads 19.0;",
        "squeezed onto the lug, 30.5. The 22.5 gauge card (sheet 2) laid flat on the plate between the tab",
        "and the jamb gives the same answer without a ruler: it drops in only when the latch is NOT latched.",
    ])
    s.text(60, 1100, "Hidden parts are dashed: carriage, bolt, spring (zigzag) and post lie behind the plate, inside "
           "the frame's thickness.", size=13, color=THIN)
    s.text(60, 1120, "Red dashed: the tab squeezed 11.5 onto the lug; the bolt's tip is then 2 inside the screen's "
           "side.", size=13, color=RED)
    return s


def sheet_parts():
    s = Sheet(1640, 1200)
    _title(s, "Sheet 2. Printed parts, bolt and gauge card (left-hand parts; mirror for the right-hand latch)",
           "Millimetres, nominal sizes. Print in PETG or nylon, 0.2 layers, 4 walls, 40 % infill or more. Sliding fits "
           "have 0.1 to 0.3 clearance; ease with a file if a printer runs tight.")
    k = 6.0
    # housing, front
    a = View(s, 130, 610, k)
    s.text(60, 100, "1  Housing, from the room", size=15, bold=True)
    a.rect(0, 0, PLATE_W, PLATE_H, PRINT, INK, 2.0)
    a.rect(SLOT[0], SLOT[2], SLOT[1], SLOT[3], "#ffffff", INK, 1.6)
    a.rect(POST[0], POST[2], POST[1], POST[3], None, THIN, 1.0, (5, 4))
    a.rect(POST[0] - PEG[0], SPRING_V - PEG[1] / 2, POST[0], SPRING_V + PEG[1] / 2, None, THIN, 1.0, (5, 4))
    for u, v in MOUNT:
        a.circle(u, v, CBORE_D, "#ffffff", INK, 1.2)
        a.circle(u, v, CLEAR_D, None, INK, 1.2)
    for u, v in LUG_SCREW:
        a.circle(u, v, CBORE_D, None, THIN, 1.0, (4, 3))
        a.circle(u, v, CLEAR_D, "#ffffff", INK, 1.2)
    a.hdim(0, SLOT[0], -8, frm=(0, SLOT[2]), label="19.6", side=-1)
    a.hdim(SLOT[0], SLOT[1], -8, frm=SLOT[2], label="14")
    a.hdim(0, PLATE_W, -15, frm=0)
    a.hdim(0, LUG_SCREW[0][0], 69.5, frm=(PLATE_H, LUG_SCREW[1][1]), label="36.5")
    a.hdim(0, POST[0], 75, frm=(PLATE_H, POST[3]), label="42.5")
    a.vdim(0, PLATE_H, -6, frm=0, side=-1)
    a.vdim(0, SLOT[2], 51, frm=(PLATE_W, SLOT[1]), side=1, label="30.7")
    a.vdim(0, SLOT[3], 60, frm=(PLATE_W, SLOT[1]), side=1, label="59.3")
    _notes(s, 60, 722, [
        "Plate 45 x 65 x 3. Slot 14 x 28.6 through (u 19.6 to 33.6,",
        "v 30.7 to 59.3). Positions below are (u, v) from the corner.",
        "3 mounting holes dia 3.2 through, counterbored dia 6.2 x 1.8 deep",
        "from the FRONT, at (8, 26), (8, 60) and (37, 9.5).",
        "2 lug-screw holes dia 3.2 through, counterbored dia 6.2 x 1.8 deep",
        "from the BACK, at (36.5, 37) and (36.5, 53); these two counterbores",
        "break 0.2 into the slot's inner end, as intended.",
        "Post behind the plate (dashed): 2.5 x 11 x 10 deep, u 42.5 to 45,",
        "v 49 to 60. Square peg 3.4 x 3.4 x 3 long on its outer face, centred",
        "v 54.5 and 5.55 behind the plate's back: it locates the spring.",
        "Print with the front face on the bed (post up); support under",
        "the peg only (a 3 mm overhang). Clean the holes with a 3.2 drill.",
    ], gap=18)
    # housing, edge view
    c = View(s, 640, 610, k)
    s.text(560, 100, "1  Housing, edge view", size=15, bold=True)
    s.text(560, 118, "(from the screen's side)", size=13, color=THIN)
    c.rect(0, 0, PLATE_T, PLATE_H, PRINT, INK, 2.0)
    c.rect(-POST[4], POST[2], 0, POST[3], PRINT, INK, 2.0)
    c.hdim(-POST[4], 0, 66, frm=POST[3], label="10", side=-1)
    c.hdim(0, PLATE_T, 71, frm=PLATE_H, label="3", side=1)
    c.vdim(POST[2], POST[3], 9, frm=(0, 0), side=1, label="post 11 (v 49 to 60), peg 3.4 square")
    c.rect(-BOLT_DEPTH - PEG[1] / 2, SPRING_V - PEG[1] / 2, -BOLT_DEPTH + PEG[1] / 2, SPRING_V + PEG[1] / 2, None,
           INK, 1.2)
    # slide: end view (v, w) drawn with v to the right, w up
    b = View(s, 790, 430, k)
    s.text(830, 100, "2  Slide, end view from the screen's middle", size=15, bold=True)
    b.poly([(CARR[2], -CARR[5]), (CARR[3], -CARR[5]), (CARR[3], -CARR[4]), (TAB_V0 + TAB_H, -CARR[4]),
            (TAB_V0 + TAB_H, PLATE_T + TAB_PROUD), (TAB_V0, PLATE_T + TAB_PROUD)], PRINT_D, INK, 2.0)
    b.line(TAB_V0, PLATE_T + TAB_GAP, TAB_V0 + TAB_H, PLATE_T + TAB_GAP, 0.8, "#ffffff", (4, 3))
    b.line(TAB_V0, -CARR[4], TAB_V0 + TAB_H, -CARR[4], 0.8, "#ffffff", (4, 3))
    b.circle(BOLT_V, -BOLT_DEPTH, BOLT_BORE, "#ffffff", INK, 1.4)
    b.circle(SPRING_V, -BOLT_DEPTH, SPRING_BORE, "#ffffff", INK, 1.4)
    b.rect(CARR[2], -BOLT_DEPTH - RETAIN[1] / 2, BOLT_V - BOLT_BORE / 2, -BOLT_DEPTH + RETAIN[1] / 2, None, "#ffffff",
           1.0, (4, 3))
    b.text(BOLT_V, -BOLT_DEPTH, "A", dx=0, dy=5, size=13, anchor="middle", bold=True)
    b.text(SPRING_V, -BOLT_DEPTH, "B", dx=0, dy=5, size=13, anchor="middle", bold=True)
    b.text(CARR[2] + 5, -BOLT_DEPTH, "C", dx=0, dy=-10, size=13, anchor="middle", bold=True, color="#ffffff")
    b.hdim(TAB_V0, TAB_V0 + TAB_H, 37, frm=PLATE_T + TAB_PROUD, label="28 (v 31 to 59)")
    b.hdim(CARR[2], BOLT_V, -14, frm=(-CARR[5], -BOLT_DEPTH), label="14", side=0)
    b.hdim(BOLT_V, SPRING_V, -14, frm=-BOLT_DEPTH, label="9.5", side=0)
    b.hdim(CARR[2], CARR[3], -20, frm=-CARR[5], label="31.5 (v 31 to 62.5)")
    b.vdim(PLATE_T + TAB_GAP, PLATE_T + TAB_PROUD, 24, frm=TAB_V0, side=-1, label="29.8 tab")
    b.vdim(-CARR[4], PLATE_T + TAB_GAP, 24, frm=TAB_V0, side=-1, label="3.5 neck")
    b.vdim(-CARR[5], -CARR[4], 24, frm=CARR[2], side=-1, label="10 carriage")
    b.vdim(-BOLT_DEPTH, -CARR[4], 68, frm=(SPRING_V, CARR[3]), side=1, label="5.25 to both bore axes")
    # slide: side view (u, w)
    d = View(s, 1290, 300, k, down=True)
    s.text(1330, 100, "2  Slide, from above (room at the bottom)", size=15, bold=True)
    d.poly([(TAB_U0, PLATE_T + TAB_PROUD), (TAB_U0 + TAB_T, PLATE_T + TAB_PROUD), (TAB_U0 + TAB_T, -CARR[4]),
            (CARR[1], -CARR[4]), (CARR[1], -CARR[5]), (CARR[0], -CARR[5]), (NECK_U0, PLATE_T + TAB_GAP),
            (TAB_U0, PLATE_T + TAB_GAP)], PRINT_D, INK, 2.0)
    d.rect(CARR[0], -BOLT_DEPTH - BOLT_BORE / 2, CARR[1], -BOLT_DEPTH + BOLT_BORE / 2, None, "#ffffff", 1.0, (4, 3))
    d.rect(RETAIN[0] - RETAIN[1] / 2, -BOLT_DEPTH - RETAIN[1] / 2, RETAIN[0] + RETAIN[1] / 2,
           -BOLT_DEPTH + RETAIN[1] / 2, None, "#ffffff", 1.0)
    d.hdim(TAB_U0, TAB_U0 + TAB_T, 37, frm=PLATE_T + TAB_PROUD, label="6 (u 16 to 22)", side=1)
    d.hdim(TAB_U0, NECK_U0, -4, frm=PLATE_T + TAB_GAP, label="3.6", side=-1)
    d.hdim(CARR[0], RETAIN[0], -14, frm=(-CARR[5], -BOLT_DEPTH), label="5.5 to pilot C", side=-1)
    d.hdim(CARR[0], CARR[1], -20, frm=-CARR[5], label="11 (u 19.6 to 30.6)", side=1)
    d.vdim(PLATE_T + TAB_GAP, PLATE_T + TAB_PROUD, 38, frm=TAB_U0 + TAB_T, side=1, label="29.8")
    _notes(s, 830, 610, [
        "One piece. The neck (2.4 x 28 x 3.5) passes through the plate's slot; the tab goes in from the back of the "
        "plate.",
        "A: bolt bore dia 5.7, through, 14 above the bottom face.",
        "B: spring bore dia 6.6, open toward the screen's middle, 8.1 deep (its bottom 0.5 beyond the tab's inner "
        "face).",
        "C: pilot 2.4 square, from the bottom face up into the bolt bore, 5.5 from the carriage's outer face, for the "
        "retention screw (part 7).",
        "Print with the bottom face (v 31) on the bed; no support.",
    ], gap=18)
    # lug
    e = View(s, 130, 1150, k)
    s.text(60, 955, "3  Lug", size=15, bold=True)
    e.rect(0, 0, TAB_T, TAB_H, "#3d3d3d", INK, 2.0)
    for _, v in LUG_SCREW:
        e.circle(LUG_SCREW[0][0] - LUG_U0, v - TAB_V0, PILOT_D, "#ffffff", INK, 1.2)
    e.hdim(0, TAB_T, 31.5, frm=TAB_H, label="6", side=1)
    e.vdim(0, TAB_H, -4, frm=0, side=-1, label="28")
    e.vdim(0, LUG_SCREW[0][1] - TAB_V0, 10, frm=(TAB_T, LUG_SCREW[0][0] - LUG_U0), side=1, label="6")
    e.vdim(0, LUG_SCREW[1][1] - TAB_V0, 16, frm=(TAB_T, LUG_SCREW[0][0] - LUG_U0), side=1, label="22")
    _notes(s, 270, 1000, ["6 x 28 x 30 tall.", "2 pilot holes dia 2.4 x 9 deep", "in the base, on its centre line",
                           "(3 from either long face).", "Print on its base."], gap=18)
    # bolt
    f = View(s, 620, 1050, 9.0)
    s.text(560, 955, "4  Bolt: 7/32 in (5.56) music wire", size=15, bold=True)
    f.poly([(0, 1.5), (NOSE, BOLT_D / 2), (BOLT_LEN, BOLT_D / 2), (BOLT_LEN, -BOLT_D / 2), (NOSE, -BOLT_D / 2),
            (0, -1.5)], STEEL, INK, 1.8)
    f.rect(BOLT_LEN - FLAT[1], -BOLT_D / 2, BOLT_LEN - FLAT[0], -BOLT_D / 2 + FLAT[2], "#ffffff", INK, 1.2)
    f.hdim(0, BOLT_LEN, 8, frm=BOLT_D / 2, label="40")
    f.hdim(0, NOSE, -8, frm=-BOLT_D / 2, label="2", side=-1)
    f.hdim(BOLT_LEN - FLAT[1], BOLT_LEN - FLAT[0], -8, frm=-BOLT_D / 2, label="6")
    f.hdim(BOLT_LEN - FLAT[0], BOLT_LEN, -8, frm=-BOLT_D / 2, label="2.5", side=1)
    s.text(560, 1145, "Nose: 2 long, ground down to about dia 3. Flat: 0.8 deep, 6 long, 2.5 from the inner end.",
           size=13, color=THIN)
    s.text(560, 1163, "Cut and grind with an abrasive wheel (the wire is hardened).", size=13, color=THIN)
    s.text(560, 1181, "The flat faces down (toward the screen's bottom edge) when fitted.", size=13, color=THIN)
    # gauge card
    g = View(s, 1180, 1130, 5.0)
    s.text(1120, 955, "Gauge card (printed, gauge-card.stl), 1.5 thick", size=15, bold=True)
    g.poly([(0, 0), (GAUGE[0], 0), (GAUGE[0], GAUGE[2] / 2), (GAUGE[1], GAUGE[2] / 2), (GAUGE[1], GAUGE[2]),
            (0, GAUGE[2])], "#f2c94c", INK, 2.0)
    g.hdim(0, GAUGE[0], -5, frm=0, label="43")
    g.hdim(0, GAUGE[1], 27, frm=GAUGE[2], label="40")
    g.vdim(0, GAUGE[2], -4, frm=0, side=-1, label="22.5")
    g.vdim(0, GAUGE[2] / 2, 47, frm=GAUGE[0], side=1, label="11.25")
    _notes(s, 1465, 990, ["22.5 wide: latch reading.", "43 long: screen's face", "behind the jamb's face.",
                           "40 long: the same, stood", "on the housing plate.", "1.5 thick: sill feeler."], gap=18)
    return s


def sheet_screen():
    s = Sheet(1640, 1080)
    _title(s, "Sheet 3. Screen, latch positions and the jamb holes",
           "Millimetres. Screen: Prime-Line MP8239 frame kit (7/16 x 3/4 in mill aluminium, square-cut plastic "
           "corners, vinyl spline, charcoal fibreglass mesh) cut to 406 x 356 outside.")
    k = 1.7
    a = View(s, 200, 800, k)
    s.text(60, 100, "Screen from the room (latches on this face, mesh and spline on the back face)", size=15,
           bold=True)
    a.rect(0, 0, SCREEN_W, SCREEN_H, "#c9ccd0", INK, 1.8)
    a.rect(FRAME_FACE, FRAME_FACE, SCREEN_W - FRAME_FACE, SCREEN_H - FRAME_FACE, "#eef0f2", INK, 1.2)
    for x0 in (0, SCREEN_W - FRAME_FACE):
        for y0 in (0, SCREEN_H - FRAME_FACE):
            a.rect(x0, y0, x0 + FRAME_FACE, y0 + FRAME_FACE, "#9aa0a6", INK, 1.0)
    for side in (0, 1):
        def ux(u, side=side):
            return u if side == 0 else SCREEN_W - u
        a.rect(ux(0), 0, ux(PLATE_W), PLATE_H, PRINT, INK, 1.6)
        a.rect(ux(TAB_U0), TAB_V0, ux(TAB_U0 + TAB_T), TAB_V0 + TAB_H, PRINT_D, INK, 1.2)
        a.rect(ux(LUG_U0), TAB_V0, ux(LUG_U0 + TAB_T), TAB_V0 + TAB_H, "#3d3d3d", INK, 1.2)
        a.rect(ux(-BOLT_OUT), BOLT_V - BOLT_D / 2, ux(0), BOLT_V + BOLT_D / 2, STEEL, INK, 1.0)
    a.hdim(0, SCREEN_W, -28, frm=0, label="406 (16 in)")
    a.vdim(0, SCREEN_H, -62, frm=0, side=-1, label="356 (14 in)")
    a.vdim(0, BOLT_V, -30, frm=(0, -BOLT_OUT), side=-1, label="45")
    a.hdim(TAB_U0, SCREEN_W - TAB_U0, 110, frm=TAB_V0 + TAB_H, label="374 between the tabs' outer faces, at rest")
    a.hdim(-BOLT_OUT, SCREEN_W + BOLT_OUT, 384, frm=(BOLT_V + BOLT_D / 2, BOLT_V + BOLT_D / 2),
           label="425 over the bolt tips, tabs at rest")
    a.text(SCREEN_W / 2, SCREEN_H / 2, "charcoal fibreglass mesh", size=13, anchor="middle", color=GREY)
    _notes(s, 60, 905, [
        "Opening between the jambs: 412. Over the bolt tips the screen is 425 at rest and 402 with both tabs",
        "squeezed, so it enters only while both tabs are held squeezed.",
        "Frame pieces are cut square and joined by the kit's corner inserts (dark squares).",
        "Drill each stile dia 6.0 through both walls, 45 above the screen's bottom edge, on the middle of the",
        "frame's 11.1 thickness (5.55 from either face). Bolt tips stand 9.5 proud of each side at rest.",
        "Roll the mesh and spline into the groove on the back face last, after both latches are fitted and checked.",
    ], gap=19)
    # jamb hole detail: section from above
    b = View(s, 1250, 335, 7.0, down=True)
    s.text(1010, 100, "Jamb hole, section from above, room at the bottom (left jamb; right mirrored)", size=15,
           bold=True)
    back = -FRAME_T
    b.poly([(-28, 22), (-PLAY, 22), (-PLAY, back), (10, back), (10, back - 12), (-28, back - 12)], WOOD, INK, 1.4)
    b.rect(-PLAY - HOLE_DEPTH, -BOLT_DEPTH - HOLE_W / 2, -PLAY, -BOLT_DEPTH + HOLE_W / 2, "#ffffff", INK, 1.4)
    b.rect(0, back, FRAME_FACE, 0, ALU, INK, 1.2)
    b.text(-27, 19, "jamb", size=13)
    b.text(-27, back - 5, "stop", size=13)
    b.text(2, -3, "screen", size=13)
    b.centre(-20, -BOLT_DEPTH, 24, -BOLT_DEPTH)
    b.hdim(-PLAY - HOLE_DEPTH, -PLAY, 6, frm=-BOLT_DEPTH + HOLE_W / 2, label="12 deep", side=-1)
    b.vdim(-BOLT_DEPTH - HOLE_W / 2, -BOLT_DEPTH + HOLE_W / 2, -20, frm=-PLAY - HOLE_DEPTH, side=-1, label="10")
    b.vdim(back, -BOLT_DEPTH, 28, frm=(10, 24), side=1, label="5.55 (stop's face to hole centre)")
    b.hdim(-PLAY, 0, 26, frm=(22, 0), label="3 play each side", side=1)
    b.hdim(0, 10, back - 16, frm=(back, back - 12), label="stop laps the screen by 10", side=1)
    c = View(s, 1250, 900, 5.0)
    s.text(1010, 575, "Jamb hole, seen from inside the opening", size=15, bold=True)
    c.rect(-30, -5, 30, 0, WOOD, INK, 1.2)
    c.rect(-30, 0, 30, 58, "#f4ecda", INK, 1.2)
    c.rect(-HOLE_W / 2, BOLT_V - HOLE_H / 2, HOLE_W / 2, BOLT_V + HOLE_H / 2, "#ffffff", INK, 1.6)
    c.text(-29, -4, "sill", size=13)
    c.text(-29, 53, "jamb's inner face", size=13)
    c.text(-29, 49.5, "(room to the left)", size=13)
    c.hdim(-HOLE_W / 2, HOLE_W / 2, 54, frm=BOLT_V + HOLE_H / 2, label="10 front to back", side=1)
    c.vdim(BOLT_V - HOLE_H / 2, BOLT_V + HOLE_H / 2, 10, frm=HOLE_W / 2, side=1, label="8 tall")
    c.vdim(0, BOLT_V, -12, frm=(-30, -HOLE_W / 2), side=-1, label="45")
    _notes(s, 1010, 965, [
        "Opening between the jambs 412 (screen 406 + 3 each side); head 366 above the sill;",
        "jamb's room-side face 45 in front of the seated screen's room-side face.",
        "Each hole: 10 front to back, 8 tall, 12 deep, centred 45 above the sill and 5.55 in front",
        "of the stop's face (drill dia 8 twice, 2 apart, and pare the ends square).",
    ], gap=19)
    return s


def main():
    (HERE / "dimensions.json").write_text(json.dumps(DIMENSIONS, indent=2) + "\n")
    wrote = ["dimensions.json"]
    sheets = {"sheet-1-latch-assembly": sheet_assembly(), "sheet-2-printed-parts": sheet_parts(),
              "sheet-3-screen-and-jamb": sheet_screen()}
    for name, sh in sheets.items():
        (HERE / f"{name}.svg").write_text(sh.svg())
        wrote.append(f"{name}.svg")
        try:
            sh.png(HERE / f"{name}.png")
            wrote.append(f"{name}.png")
        except ImportError:
            pass
    try:
        report = make_stl()
        (HERE / "stl" / "mesh-check.json").write_text(json.dumps(report, indent=2) + "\n")
        wrote.append(f"stl/ ({', '.join(report)})")
    except ImportError:
        pass
    print("wrote", "; ".join(wrote))


if __name__ == "__main__":
    main()
