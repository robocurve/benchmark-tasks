"""Writes the build drawings for the real fixture: dimensions.json and four dimensioned sheets (SVG, and PNG when
Pillow is installed): the screen with its two bought corner tension springs and two bought pull tabs, and the window
section with its sill and head channels, plus scoring checks. Every number comes from the table below; near_misses.py checks that table
against the simulated model.

    python make_package.py          (run from this folder)

Screen coordinates, in millimetres: u from the screen's left side edge (seen from the room) toward its right, v up
from its bottom edge, w from the frame's room-side face toward the room. Window section: x from the jambs' room-side
face toward the window, z up from the sill channel's floor.
"""

from __future__ import annotations

import json
import math
import pathlib

HERE = pathlib.Path(__file__).resolve().parent

# -- every dimension (mm) ---------------------------------------------------------------------------------------------
SCREEN_W, SCREEN_H = 406.0, 356.0  # 16 x 14 in outside
FRAME_T, FRAME_FACE = 11.1, 19.05  # 7/16 x 3/4 in roll-formed profile
SPR_LEN, SPR_W = 69.85, 6.35  # Prime-Line PL 7776 corner tension spring, 2-3/4 x 1/4 in
SPR_CROWN = 7.0  # measured free bow of one PL 7776, adopted as model rail-proud crown
SPR_IN = round(SPR_LEN / 2, 2)  # crown centre in from each side edge
SPR_K = 400.0  # N/m, conservative model assumption; measured secant 226 N/m
SPR_PROXY_KG, GRAVITY = 0.005, 9.81  # uncalibrated moving-mass proxy and gravitational acceleration
SPR_PRELOAD_N = 0.0  # no nominal preload; installed value unmeasured
SPR_DRIVE_OFFSET_N = SPR_PRELOAD_N + SPR_PROXY_KG * GRAVITY
PROBE_W, PROBE_H = 16.0, 3.0
SPR_PROXY_OFFSET_N = SPR_PROXY_KG * GRAVITY  # proxy-weight correction, not an installed preload measurement
TAB_PROUD, TAB_TALL, TAB_THK = 27.0, 15.9, 2.38  # PL 14621: 1-1/16 in height, 5/8 in tongue, 3/32 in base
TAB_BASE_TALL, TAB_FOOT_W = 23.8, 6.35  # 15/16 in spline-foot width, 1/4 in foot depth
TAB_V = 140.0  # left pull tab centre above the bottom edge (seen from the room: the robot's left)
TAB_V_R = 50.0  # right pull tab centre: staggered 90, so that a two-handed pinch holds the screen's angle
TAB_IN = 15.0  # pull tab centre in from each side edge, in the stile's spline groove
PLAY = 3.0  # screen side to jamb face, each side, screen centred
DEPTH = 45.0  # jamb's room-side face to the seated screen's room-side face
CLR = 1.5  # sill channel width = frame thickness + 1.5
CHANNEL = FRAME_T + CLR
HEAD_W = 15.9  # head channel 5/8 in wide, room to tip the screen's top in
SILL_LIP_H, SILL_LIP_T = 4.0, 6.0
HEAD_LAP = 6.0  # the head lip covers the seated screen's top 6
GAP_TOP = 6.0  # head channel's ceiling above the seated screen's top edge (springs pressed 1 when seated)
STOP_LAP, STOP_T = 10.0, 12.0
STOOL = 60.0  # stool's projection in front of the jambs' faces
SILL_Z = 70.0  # channel floor (and stool top) above the table
JAMB_W = 40.0  # cropped
REVEAL = 40.0  # interior jambs' inner faces out from the screen's sides
GLASS_X = 24.0  # glass behind the stop's face
SEAT_TOL, SILL_TOL, TOP_IN = CLR, 1.5, 3.0

DIMENSIONS = {
    "units": "mm",
    "screen": {"width": SCREEN_W, "height": SCREEN_H, "frame_thickness": FRAME_T, "frame_face": FRAME_FACE},
    "spring": {"part": "Prime-Line PL 7776 corner-mounted screen tension spring, 2-3/4 x 1/4 in heat-treated steel (6-pack)",
               "length": SPR_LEN, "width": SPR_W, "free_crown_above_top_edge": SPR_CROWN,
               "crown_centre_in_from_side": SPR_IN, "pressed_when_seated": round(SPR_CROWN - GAP_TOP, 2),
               "force_seated_N": round(SPR_K * (SPR_CROWN - GAP_TOP) / 1000, 2),
               "force_flat_N": round(SPR_K * SPR_CROWN / 1000, 2),
               "force_fields_are": "stiffness term kq, not total drive or contact-force measurements",
               "proxy_weight_drive_offset_N": SPR_PROXY_OFFSET_N,
               "assumed_preload_N": SPR_PRELOAD_N,
               "drive_target_m": -SPR_DRIVE_OFFSET_N / SPR_K,
               "net_upright_seated_N": round(SPR_K * (SPR_CROWN - GAP_TOP) / 1000 + SPR_DRIVE_OFFSET_N - SPR_PROXY_KG * GRAVITY, 5),
               "net_upright_flat_N": round(SPR_K * SPR_CROWN / 1000 + SPR_DRIVE_OFFSET_N - SPR_PROXY_KG * GRAVITY, 5),
               "crown_and_forces_are": "one PL 7776 measured: 7 mm free bow, 1.58 N flat; "
                                       f"{SPR_K:g} N/m no nominal preload, 0.04905 N proxy-weight correction; installed curve unmeasured"},
    "pull_tab": {"part": "Prime-Line PL 14621, black vinyl spline channel pull tab", "proud_of_face": TAB_PROUD,
                 "tall": TAB_TALL, "spline_foot_width": TAB_BASE_TALL, "foot_depth": TAB_FOOT_W, "thick_as_modelled": TAB_THK, "centre_above_bottom_edge": TAB_V, "right_centre_above_bottom_edge": TAB_V_R,
                 "in_from_side": TAB_IN},
    "window": {"opening_between_frame_sides": SCREEN_W + 2 * PLAY, "play_each_side": PLAY,
               "reveal_from_screen_side": REVEAL,
               "jamb_face_to_seated_screen_face": DEPTH, "sill_channel_width": CHANNEL, "head_channel_width": HEAD_W,
               "sill_lip_height": SILL_LIP_H,
               "sill_lip_thickness": SILL_LIP_T, "head_lip_covers_top": HEAD_LAP,
               "head_lip_underside_above_floor": SCREEN_H - HEAD_LAP,
               "head_channel_ceiling_above_seated_top": GAP_TOP,
               "head_channel_ceiling_above_floor": SCREEN_H + GAP_TOP, "stop_lap": STOP_LAP,
               "stop_thickness": STOP_T, "stool_projection": STOOL, "sill_above_table": SILL_Z,
               "glass_behind_stop_face": GLASS_X,
               "probe_port_width": PROBE_W, "probe_port_height": PROBE_H,
               "probe_port_centre_from_opening_centre": SCREEN_W / 2 - SPR_LEN / 2,
               "probe_port_bottom_above_floor": SCREEN_H + GAP_TOP - PROBE_H,
               "probe_port_top_above_floor": SCREEN_H + GAP_TOP,
               "probe_port_length": DEPTH + FRAME_T - HEAD_W},
    "reading": {"seated_if_screen_face_behind_jamb_face_at_least": DEPTH - SEAT_TOL,
                "bottom_rail_showing_above_sill_lip_seated": FRAME_FACE - SILL_LIP_H,
                "bottom_down_if_bottom_rail_showing_at_most": FRAME_FACE - SILL_LIP_H + SILL_TOL,
                "top_rail_showing_below_head_lip_seated": FRAME_FACE - HEAD_LAP,
                "top_captured_if_top_rail_showing_below_head_lip_at_most": FRAME_FACE - TOP_IN},
}


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
                weight = ' font-weight="bold"' if bold else ""
                o.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" text-anchor="{anchor}" fill="{col}"'
                         f'{weight}>{s}</text>')
        o.append("</svg>")
        return "\n".join(o) + "\n"

    def png(self, path, k=2):
        from PIL import Image, ImageDraw, ImageFont

        img = Image.new("RGB", (self.w * k, self.h * k), "#ffffff")
        dr = ImageDraw.Draw(img)
        fonts = {}

        def font(size, bold):
            if (size, bold) not in fonts:
                names = (["/System/Library/Fonts/Supplemental/Arial Bold.ttf", "/usr/share/fonts/truetype/ubuntu/Ubuntu-B.ttf"]
                         if bold else
                         ["/System/Library/Fonts/Supplemental/Arial.ttf", "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf"])
                for name in names:
                    try:
                        fonts[(size, bold)] = ImageFont.truetype(name, size * k)
                        break
                    except OSError:
                        continue
                else:
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



def _bow(u0, u1, v, crown, n=24):
    """Points of a bowed leaf from u0 to u1 standing ``crown`` above v at its middle (a sine arch)."""
    return [(u0 + (u1 - u0) * i / n, v + crown * math.sin(math.pi * i / n)) for i in range(n + 1)]


def sheet_screen():
    s = Sheet(1640, 1160)
    _title(s, "Sheet 1. The screen: frame kit, two corner tension springs and two pull tabs (all bought parts)",
           "Millimetres. Frame: Prime-Line MP8239 kit (7/16 x 3/4 in mill aluminium, square-cut plastic corners, "
           "vinyl spline, charcoal fibreglass mesh), cut to 406 x 356 outside.")
    k = 1.75
    a = View(s, 170, 820, k)
    s.text(60, 100, "Screen from the room (spline groove, mesh and pull tabs on this face)", size=15, bold=True)
    a.rect(0, 0, SCREEN_W, SCREEN_H, "#c9ccd0", INK, 1.8)
    a.rect(FRAME_FACE, FRAME_FACE, SCREEN_W - FRAME_FACE, SCREEN_H - FRAME_FACE, "#eef0f2", INK, 1.2)
    for x0 in (0, SCREEN_W - FRAME_FACE):
        for y0 in (0, SCREEN_H - FRAME_FACE):
            a.rect(x0, y0, x0 + FRAME_FACE, y0 + FRAME_FACE, "#9aa0a6", INK, 1.0)
    for u0, u1 in ((0.0, SPR_LEN), (SCREEN_W, SCREEN_W - SPR_LEN)):  # springs bowing up out of the top edge
        pts = _bow(u0, u1, SCREEN_H, SPR_CROWN)
        for p0, p1 in zip(pts, pts[1:]):
            a.line(p0[0], p0[1], p1[0], p1[1], 2.0, STEEL)
    for u, v in ((TAB_IN, TAB_V), (SCREEN_W - TAB_IN, TAB_V_R)):  # pull tabs, edge-on from the room
        a.rect(u - TAB_THK / 2, v - TAB_TALL / 2, u + TAB_THK / 2, v + TAB_TALL / 2, "#222222", INK, 1.0)
    a.hdim(0, SCREEN_W, -28, frm=0, label="406 (16 in)")
    a.vdim(0, SCREEN_H, -40, frm=0, side=-1, label="356 (14 in)")
    a.hdim(0, SPR_LEN, SCREEN_H + 34, frm=SCREEN_H, label="69.85 spring (2-3/4 in)")
    a.hdim(SCREEN_W - SPR_LEN, SCREEN_W, SCREEN_H + 34, frm=SCREEN_H, label="69.85")
    a.vdim(SCREEN_H, SCREEN_H + SPR_CROWN, SCREEN_W + 18, frm=SCREEN_W - SPR_IN, side=1,
           label="7 model proud; bench bow measured")
    a.vdim(0, TAB_V_R, SCREEN_W + 18, frm=SCREEN_W - TAB_IN, side=1, label=f"{TAB_V_R:g}")
    a.vdim(0, TAB_V, TAB_IN + 14, frm=TAB_IN, side=1, label=f"{TAB_V:g}")
    a.hdim(SCREEN_W - TAB_IN, SCREEN_W, TAB_V_R + 30, frm=TAB_V_R + TAB_TALL / 2, label="15", side=1)
    a.text(SCREEN_W / 2, SCREEN_H / 2, "charcoal fibreglass mesh", size=13, anchor="middle", color=GREY)
    a.text(SPR_LEN + 6, SCREEN_H + 4, "corner tension spring", size=12, color=THIN)
    a.text(TAB_IN + 8, TAB_V + 20, "pull tab", size=12, color=THIN)
    # the top-left corner, enlarged: the spring's end slipped into the corner, its leaf bowing along the top edge
    b = View(s, 1010, 330, 6.0, down=False)
    s.text(1000, 100, "Top-left corner from the room, enlarged", size=15, bold=True)
    s.text(1000, 120, "(the spring's straight end slips into the corner, its leaf bows up along the top edge)",
           size=12, color=THIN)
    b.rect(0, -FRAME_FACE - 8, FRAME_FACE, 0, ALU, INK, 1.4)  # stile
    b.rect(FRAME_FACE, -FRAME_FACE, 80, 0, ALU, INK, 1.4)  # top rail
    b.rect(2, -FRAME_FACE + 2, FRAME_FACE - 2, -2, "#e4e4e4", GREY, 0.8, (3, 2))  # corner insert (hidden)
    pts = _bow(0.0, SPR_LEN, 0.0, SPR_CROWN)
    for p0, p1 in zip(pts, pts[1:]):
        b.line(p0[0], p0[1], p1[0], p1[1], 2.4, STEEL)
    b.line(2, -0.6, 14, -0.6, 2.4, STEEL)  # the end held at the corner
    b.vdim(0, SPR_CROWN, SPR_LEN + 6, frm=SPR_IN, side=1, label="model: 7 free, 6 seated")
    b.hdim(0, SPR_IN, SPR_CROWN + 6, frm=SPR_CROWN, label="34.9 to the crown")
    b.text(4, -FRAME_FACE - 4, "corner insert (kit)", size=12, color=THIN)
    _notes(s, 1000, 560, [
        "*Measured on one PL 7776 spring, Nov 23 2026",
        "Calipers/ruler: 0.27 mm strip, 7 mm free bench bow.",
        "Kitchen scale: 161 g (1.58 N) to press center flat.",
        f"Model: {SPR_K:g} N/m, no nominal preload.",
        "Separate 0.04905 N proxy-weight correction.",
        f"Net upright: {SPR_PRELOAD_N + SPR_K*.001:g} N seated (1 mm);",
        f"{SPR_PRELOAD_N + SPR_K*.0065:g} N at 6.5 mm; {SPR_PRELOAD_N + SPR_K*.007:g} N flat.",
        "Adopts bench bow as rail-proud height; check installed.",
        "Above assumed free-part linear secant only.",
        "Installed force/crown/uncertainty unmeasured.",
        "Pull-tab compliance/retention also needs a load check.",
    ], gap=19)
    _notes(s, 1000, 880, [
        "*Parts (bought)",
        "1 Prime-Line MP8239 screen frame kit, cut to 406 x 356",
        "2 Prime-Line PL 7776 corner tension springs (pack of 6)",
        "2 Prime-Line PL 14621 spline channel pull tabs (pack of 25)",
        "*Steps",
        "1 Cut the frame pieces square to 406 x 356 outside with the corners in.",
        "2 Slip a spring into each top corner, leaf on the top edge bowing up",
        "   and running toward the middle (no tools; maker's listing).",
        "3 Roll the mesh and spline into the groove; press a pull tab into the",
        "   groove on each stile: left centred 140, right 50 above the bottom edge.",
        "4 Model target: 7 above rail. Measure installed height and force; not enrolled.",
    ], gap=19)
    s.text(60, 960, "PL 14621 flat tongue (face view along its thickness; rigid model shown)", size=15, bold=True)
    tab = View(s, 210, 1080, 5.0)
    tab.poly([(2.38, -TAB_BASE_TALL / 2), (TAB_PROUD, -TAB_TALL / 2),
              (TAB_PROUD, TAB_TALL / 2), (2.38, TAB_BASE_TALL / 2)], fill="#444444")
    tab.rect(0, -TAB_BASE_TALL / 2, 2.38, TAB_BASE_TALL / 2, fill="#222222")
    tab.hdim(0, TAB_PROUD, 20, frm=(TAB_BASE_TALL / 2, TAB_TALL / 2), label="27 nominal projection")
    tab.vdim(-TAB_TALL / 2, TAB_TALL / 2, TAB_PROUD + 5, frm=TAB_PROUD, side=1, label="15.9 tip")
    _notes(s, 520, 1000, ["Maker line art: 1-1/16 in overall height; 15/16 in foot width;",
                            "5/8 in tongue tip width; 1/4 in foot depth; 3/32 in base.",
                            "Installed on a vertical stile; tongue toward the room.",
                            "Projection and bending are unmeasured. Model is rigid;",
                            "2.38 mm base thickness is used throughout the tongue."], gap=22)
    return s


def sheet_window():
    s = Sheet(1640, 1080)
    _title(s, "Sheet 2. Window section: sill and head channels, jambs, stop",
           "Millimetres. Painted softwood or MDF, glued and screwed; clamped to the table. Every part that touches "
           "the screen is full size; the jambs and head are cropped.")
    # vertical section through the middle of the opening
    k = 1.2
    a = View(s, 280, 900, k)
    s.text(60, 100, "Section through a spring center (room to the left)", size=15, bold=True)
    x_stop = DEPTH + FRAME_T  # stop's face from the jambs' room-side face
    a.rect(-STOOL, -SILL_Z, x_stop + 40, 0, "#e9e5da", INK, 1.2)  # wall below and stool
    a.rect(x_stop - CLR - FRAME_T - SILL_LIP_T, 0, x_stop - CLR - FRAME_T, SILL_LIP_H, WOOD, INK, 1.4)  # sill lip
    a.rect(x_stop, 0, x_stop + STOP_T, STOP_LAP, WOOD, INK, 1.2)  # sill stop
    zl, zc = SCREEN_H - HEAD_LAP, SCREEN_H + GAP_TOP
    a.rect(0, zl, x_stop - HEAD_W, zc + 30, "#f4f1ea", INK, 1.2)  # head, room side
    a.rect(0, zl, x_stop - HEAD_W, zl + 10, WOOD, INK, 1.2)  # head lip
    a.rect(x_stop - HEAD_W, zc, x_stop + 40, zc + 30, "#f4f1ea", INK, 1.2)  # head over the channel
    # Section through a spring center: the port ends at the channel, below its intact ceiling.
    a.rect(0, zc - PROBE_H, x_stop - HEAD_W, zc, "#ffffff", INK, 0.8)
    a.line(-12, zc - .05, x_stop - FRAME_T / 2 - SPR_W / 2, zc - .05, 1.4, RED)
    a.rect(x_stop, SCREEN_H - STOP_LAP, x_stop + STOP_T, zc, WOOD, INK, 1.2)  # head stop
    a.rect(x_stop + GLASS_X, 0, x_stop + GLASS_X + 6, zc, "#cfe0ea", INK, 1.0)  # glass
    a.rect(x_stop - FRAME_T, 0, x_stop, SCREEN_H, None, INK, 1.6, (6, 3))  # the seated screen, outline
    a.rect(x_stop - FRAME_T / 2 - SPR_W / 2, SCREEN_H, x_stop - FRAME_T / 2 + SPR_W / 2, zc, STEEL, INK, 1.0)
    a.text(-STOOL + 4, -SILL_Z / 2, "wall below and stool", size=12)
    a.text(x_stop - FRAME_T - 2, SCREEN_H / 2, "seated screen", size=12, anchor="end", color=THIN)
    a.vdim(0, SCREEN_H, -STOOL - 60, frm=-STOOL, side=-1, label="356 seated screen")
    a.vdim(0, zl, -18, frm=0, side=-1, label="350 head lip")
    a.vdim(zl, zc, x_stop + 70, frm=x_stop + 40, side=1, label="12: head lip to channel ceiling (6 above the seated top)")
    a.vdim(SCREEN_H, zc, x_stop + 22, frm=x_stop, side=1, label="6")
    a.vdim(0, SILL_LIP_H, -18, frm=0, side=-1, label="4 lip")
    a.vdim(-SILL_Z, 0, x_stop + 52, frm=x_stop + 40, side=1, label="70 above the table")
    a.hdim(0, x_stop - FRAME_T, -SILL_Z - 45, frm=(-SILL_Z, -SILL_Z), label="45 to the screen face", side=1)
    a.hdim(-STOOL, 0, -SILL_Z - 20, frm=-SILL_Z, label="60 stool")
    # channel details, enlarged
    d = View(s, 1060, 700, 6.0)
    s.text(860, 100, "Head channel and sill channel, enlarged (room to the left)", size=15, bold=True)
    base = 0.0
    d.rect(-8, base + 0, CHANNEL + 8, base + 2, "#e9e5da", INK, 1.0)
    d.rect(-SILL_LIP_T, base + 2, 0, base + 2 + SILL_LIP_H, WOOD, INK, 1.2)
    d.rect(CHANNEL, base + 2, CHANNEL + 8, base + 2 + STOP_LAP, WOOD, INK, 1.2)
    d.rect(CLR, base + 2, CLR + FRAME_T, base + 2 + 16, None, INK, 1.4, (5, 3))
    d.hdim(0, CHANNEL, base - 6, frm=base, label="12.6 channel (11.1 + 1.5)")
    d.vdim(base + 2, base + 2 + SILL_LIP_H, -SILL_LIP_T - 4, frm=-SILL_LIP_T, side=-1, label="4")
    d.hdim(-SILL_LIP_T, 0, base + 2 + SILL_LIP_H + 6, frm=base + 2 + SILL_LIP_H, label="6", side=-1)
    d.text(CLR + 1, base + 12, "bottom rail", size=12, color=THIN)
    hb = 40.0
    hx = CHANNEL - HEAD_W  # the head lip's back face, 4.8 in front of the seated screen's face
    d.rect(hx - 8, hb, hx, hb + 16, WOOD, INK, 1.2)
    d.rect(hx - 8, hb + HEAD_LAP + GAP_TOP, CHANNEL + 8, hb + HEAD_LAP + GAP_TOP + 3, "#f4f1ea", INK, 1.0)
    d.rect(hx - 8, hb + HEAD_LAP + GAP_TOP - PROBE_H, hx, hb + HEAD_LAP + GAP_TOP,
           "#ffffff", INK, 1.0)
    d.rect(CHANNEL, hb - 4, CHANNEL + 8, hb + HEAD_LAP + GAP_TOP, WOOD, INK, 1.2)
    d.rect(CLR, hb - 8, CLR + FRAME_T, hb + HEAD_LAP, None, INK, 1.4, (5, 3))
    d.rect(CLR + FRAME_T / 2 - SPR_W / 2, hb + HEAD_LAP, CLR + FRAME_T / 2 + SPR_W / 2, hb + HEAD_LAP + GAP_TOP,
           STEEL, INK, 1.0)
    d.hdim(hx, CHANNEL, hb + HEAD_LAP + GAP_TOP + 10, frm=hb + HEAD_LAP + GAP_TOP + 3,
           label="15.9 head channel (5/8 in)", side=-1)
    d.vdim(hb, hb + HEAD_LAP, hx - 12, frm=hx - 8, side=-1, label="6 lip covers the top rail")
    d.vdim(hb + HEAD_LAP, hb + HEAD_LAP + GAP_TOP, CHANNEL + 12, frm=CHANNEL + 8, side=1,
           label="6: springs pressed 1 when seated")
    d.text(CLR + 1, hb - 3, "top rail", size=12, color=THIN)
    _notes(s, 860, 790, [
        "*Window section",
        "Window frame sides 412 apart (406 screen + 3 each side) at the channel depth; interior jambs",
        "   (the reveal, 40 wide) 40 out from the screen's sides, from the room face back to the channels.",
        "Stop 12 thick, lapping the screen by 10 on all four sides; glass 24 behind the stop's face.",
        "Sill channel: floor level with the stool top, 70 above the table; lip 4 tall, 6 thick,",
        "   its back face 12.6 in front of the stop.",
        "Head: lip underside 350 above the floor (6 below the seated screen's top),",
        "   its back face 15.9 in front of the stop (5/8 in channel); ceiling 362 above the floor.",
        "Probe ports: 16 wide x 3 high at spring centers, top flush with ceiling; see sheet 4.",
        "*Fitting check (before trials)",
        "Seated, the screen's face is 45 behind the jambs' faces; 15.05 of the bottom rail shows",
        "above the sill lip and 13.05 of the top rail below the head lip.",
        "Lifting the seated screen 4 or more (springs pressed 5; 6 presses them flat) lets",
        "   its bottom swing out over the lip.",
    ], gap=19)
    return s


def sheet_scoring():
    s = Sheet(1640, 1530)
    _title(s, "Sheet 3. Scoring from the room: ruler and depth-rod positions",
           "Millimetres. Upright model geometry. Full credit also requires D; real probe/video access has not been demonstrated.")
    s.text(65, 118, "A. Depth behind the jamb face (side section; room to the left)", size=17, bold=True)
    a = View(s, 210, 350, 5)
    a.rect(-4, -20, 0, 35, WOOD)
    a.rect(DEPTH, -20, DEPTH + FRAME_T, 35, ALU)
    a.rect(DEPTH + FRAME_T, -20, DEPTH + FRAME_T + STOP_T, 35, WOOD)
    a.text(0, 42, "jamb face", anchor="middle", size=13)
    a.text(DEPTH, 42, "screen face", anchor="middle", size=13)
    a.text(DEPTH + FRAME_T + STOP_T, 42, "stop", anchor="middle", size=13)
    # Bridge rests on the room-side jamb face, with depth rod touching the room-side rail face.
    a.line(-3, -18, -3, 24, width=7, color="#287b9d")
    a.line(-3, 12, DEPTH, 12, width=4, color="#287b9d")
    a.hdim(0, DEPTH, -32, frm=-20, label="45 seated; at least 43.5 for 1.0")
    _notes(s, 65, 540, [
        "Bridge on the jamb face; rod touches the screen frame, never the mesh.",
        "Read the top and bottom of each stile. The stop limits further travel.",
        "1.0: all four back corners within 1.5 of the stop; both bottom corners",
        "within 1.5 of the channel floor; within 2 degrees upright, tabs toward room.",
        "Frame tests only: both crown contacts and the video hold in D also required.",
    ], size=14, gap=23)
    s.text(865, 118, "B. Bottom rail (front view at either end)", size=17, bold=True)
    b = View(s, 1030, 400, 7)
    b.rect(0, 0, 55, FRAME_FACE, ALU)
    b.rect(-5, 0, 60, SILL_LIP_H, WOOD)
    b.line(45, SILL_LIP_H, 45, FRAME_FACE, width=4, color="#287b9d")
    b.vdim(SILL_LIP_H, FRAME_FACE, 65, frm=55, side=1, label="15.05 seated")
    b.text(-5, -5, "sill lip top = ruler zero", size=13)
    _notes(s, 865, 480, [
        "Read the exposed bottom rail above the lip at both ends.",
        "Upright: at most 16.55 showing for the bottom-down test.",
        "Also check depth (A); a rail standing in front of the lip is not seated.",
    ], size=14, gap=23)
    s.text(65, 660, "C. Top rail (front view at either end)", size=17, bold=True)
    c = View(s, 220, 930, 7)
    c.rect(0, -FRAME_FACE, 55, 0, ALU)
    c.rect(-5, -HEAD_LAP, 60, 8, WOOD)
    c.line(45, -HEAD_LAP, 45, -FRAME_FACE, width=4, color="#287b9d")
    c.vdim(-FRAME_FACE, -HEAD_LAP, 65, frm=55, side=1, label="13.05 seated")
    _notes(s, 865, 700, [
        "0.5: all four top corners at least 3 above the head lip underside,",
        "behind its back face; tilt at most 15 degrees, sides inside the jambs.",
        "Upright: at most 16.05 top rail showing below the lip at both ends.",
        "Tilted: use the square-on photo and known rail width/thickness to",
        "project the top corners; the upright 16.05 reading alone is insufficient.",
        "For lean a about the width axis: lowest top rise = 19.05 cos(a)",
        "minus 11.1 max(0, sin(a)) minus the exposed front-rail height.",
        "Check both ends and use the smaller rise (roll also matters).",
        "A depth rod verifies the top rail is behind the lip, not in front.",
        "Video hold and crown checks: see D below (access unverified).",
        "Missing springs, damaged assembly or a rail through wood: invalid setup.",
    ], size=14, gap=24)
    s.line(65, 1125, 1575, 1125, 1.2)
    s.text(65, 1160, "D. Spring-apex contact and visible-frame hold — physical enrolment pending", size=17, bold=True)
    _notes(s, 65, 1190, [
        "Straight 0.10 feeler through two 16 x 3 horizontal head ports (sheet 4); ceiling and stop remain intact.",
        "Nominal leaf edges: 47.375/53.725 deep; use 45/55 probe witnesses to allow frame seating tolerance. Blade <=13 wide, >=80 long.",
        "Known-open 0.30 gap controls must pass on both sides. Refusal before 45 mm is invalid; refusal only between 45 and 55 can establish contact.",
        "1.0 requires each apex line within 0.10 of ceiling at some width point, positive compression and model resistance.",
        f"Model net upright {SPR_PRELOAD_N + SPR_K*.001:g}/{SPR_PRELOAD_N + SPR_K*.007:g} N seated/flat; assumed envelope, not installed measurements.",
        "Hold: all sampled hand points >5 clear; four frame back corners stay <=1 Euclidean displacement for 1 s.",
        "Use four visible front frame/mesh intersections and calibrated room-side views to reconstruct the rigid frame at 20 Hz.",
        "No proxy pad-corner tracking. Validate known 1 mm translations in depth and across the image; reject occlusions.",
        "Drop: origin fall >0.5 m/s between adjacent 0.05 s samples with all hand points >=20 clear gives 0 for trial.",
        "Drawings are not assembled photographs. Actual installed load, grip, access and video accuracy remain unvalidated.",
        "The 0.10 contact and 1 mm frame-motion tolerances are unchanged; physical landmark definitions are corrected.",
    ], size=14, gap=25)
    return s



def sheet_probe():
    s = Sheet(1640, 1250)
    _title(s, "Sheet 4. Straight spring-probe access and visible frame landmarks",
           "Millimetres. Design geometry and protocol, NOT an assembled fit check. Ceiling, stop and 0.10 contact tolerance retained.")
    s.text(65, 115, "A. Room-side head face: ports centered on the two spring apices", size=17, bold=True)
    a = View(s, 180, 300, 2.8)
    a.rect(0, -8, SCREEN_W, 30, WOOD)
    for u in (SPR_LEN / 2, SCREEN_W - SPR_LEN / 2):
        a.rect(u - PROBE_W / 2, 0, u + PROBE_W / 2, PROBE_H, "#ffffff", INK)
        a.centre(u, -7, u, 14)
    a.hdim(0, SPR_LEN / 2, -20, frm=0, label="34.925 center from side")
    a.hdim(SCREEN_W - SPR_LEN / 2, SCREEN_W, -20, frm=0, label="34.925")
    a.text(SCREEN_W / 2, 17, "16 wide x 3 high; top 362, bottom 359 above sill floor", anchor="middle", size=14)
    s.text(65, 420, "B. Section through one spring center; room to the left", size=17, bold=True)
    b = View(s, 200, 650, 12)
    xchannel = DEPTH + FRAME_T - HEAD_W
    xnear = DEPTH + FRAME_T/2 - SPR_W/2
    xfar = DEPTH + FRAME_T/2 + SPR_W/2
    b.rect(0, -12, xchannel, -PROBE_H, WOOD)
    b.rect(0, 0, DEPTH + FRAME_T + 3, 8, WOOD)
    b.rect(DEPTH + FRAME_T, -12, DEPTH + FRAME_T + 3, 0, WOOD)
    b.rect(xnear, -GAP_TOP, xfar, 0, STEEL)
    b.line(-8, -.05, xnear, -.05, 2.0, RED)
    b.vdim(-PROBE_H, 0, -2, frm=0, label="3 high")
    b.hdim(0, xchannel, -16, frm=-12, label="40.2 straight passage")
    b.hdim(0, xnear, -22, frm=-12, label="47.375 nominal near leaf edge")
    b.hdim(0, xfar, -28, frm=-12, label="53.725 nominal far leaf edge")
    b.text(5, 10, "Blade upper face follows intact port roof / channel ceiling", size=14)
    _notes(s, 1010, 470, [
        "*Straight 0.10 feeler; <=13 wide, >=80 long.",
        "Align to apex; mark 45 / 55 probe witnesses.",
        "Empty passage + known 0.30 apex-gap controls",
        "must pass freely to far witness on BOTH sides.",
        "Refusal only at 45-55 deep = <=0.10 local gap.",
        "Refusal before 45 deep = invalid access.",
        "Do not force; reject if frame moves >1.",
        "Record blade uncertainty and close-up views.",
        "No bent tip or hidden 4.8-slot turn is needed.",
        "Actual assembly/access test still required.",
    ], size=14, gap=26)
    s.text(65, 1040, "C. Hold landmarks: four visible front frame / mesh intersections", size=17, bold=True)
    _notes(s, 65, 1080, [
        "Coordinates on front frame: y = +/-(203 - 19.05), z = 19.05 and 356 - 19.05; no spring markers required.",
        "Two fixed calibrated room-side oblique views reconstruct the rigid frame; transform its four back corners at 20 Hz.",
        "Each corner stays <=1 mm Euclidean distance from its hold-start position for 1 s with hands >5 mm clear.",
        "Validate known 1 mm translations in depth and across image, record uncertainty; physical validation remains pending.",
    ], size=14, gap=28)
    return s


def main():
    (HERE / "dimensions.json").write_text(json.dumps(DIMENSIONS, indent=2) + "\n")
    wrote = ["dimensions.json"]
    sheets = {"sheet-1-screen": sheet_screen(), "sheet-2-window-section": sheet_window(),
              "sheet-3-scoring": sheet_scoring(), "sheet-4-probe-access": sheet_probe()}
    for name, sh in sheets.items():
        (HERE / f"{name}.svg").write_text(sh.svg())
        wrote.append(f"{name}.svg")
        try:
            sh.png(HERE / f"{name}.png")
            wrote.append(f"{name}.png")
        except ImportError:
            pass
    print("wrote", "; ".join(wrote))


if __name__ == "__main__":
    main()
