"""Printable placement cards: one A4 page per layout, at actual size, to set the form's start pose in real trials.

    python placement_cards.py          # writes forms/placement_layout0.pdf, _1.pdf, _2.pdf

Each card has two paper pieces, each keyed to physical features of the device: the heel piece fits inside the heel
cup against its back wall, the toe piece lies along the inner face of the medial guide with a hook round the guide's
front end. Each carries the form's top-view outline (1 mm clearance) at that layout's start pose: the heel piece
around the heel, the toe piece around the toes. Lay both flat on the plate, set the form against both outlines, then
lift the pieces straight up.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import footform as F  # noqa: E402
import layout as LY  # noqa: E402
from sizing import DEVICE as D, FORMS  # noqa: E402


def silhouette(spec: dict, fwd: float, side: float, yaw_deg: float):
    """Top-view outline of a form at a start pose, in the device frame (mm), as a shapely polygon."""
    from shapely.geometry import MultiPoint
    from shapely.ops import unary_union

    fo = F.Form(spec["L"], spec["A"], spec["W"], spec["name"])
    psi = math.radians(yaw_deg)
    R = np.array([[math.cos(psi), -math.sin(psi)], [math.sin(psi), math.cos(psi)]])
    pieces = [MultiPoint([tuple(R @ p[:2] + (fwd, side)) for p in h]).convex_hull for h in fo.hulls()]
    return unary_union(pieces).buffer(1.0)  # 1 mm clearance so the card lifts off freely


def card_region():
    """The stencil's outer shape (device frame, mm): inside the heel cup, then a band up to the guide face."""
    from shapely.geometry import Point, box
    from shapely.ops import unary_union

    r = D["cup_r"] - 0.5
    cup = unary_union([Point(D["cup_r"], 0.0).buffer(r, quad_segs=64), box(D["cup_r"], -r, D["cup_side_u"], r)])
    g_end = D["guide_u"][1]
    toe = unary_union([box(238.0, -70.0, g_end, D["guide_v"]),  # along the guide's inner face to its front end
                       box(g_end, 36.0, g_end + 12.0, D["guide_v"] + D["guide_w"] + 4.0)])  # hook round its end
    return cup, toe


def write_card(k: int, out: Path) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import PathPatch
    from matplotlib.path import Path as MPath

    lay = LY.LAYOUTS[k]
    spec = FORMS[lay["form"]]
    sil = silhouette(spec, lay["fwd"], lay["side"], lay["yaw"])
    cup, toe = card_region()
    W, H = 297.0, 210.0  # A4 landscape, mm
    fig = plt.figure(figsize=(W / 25.4, H / 25.4))
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(0, W)
    ax.set_ylim(0, H)
    ax.set_aspect("equal")
    ax.axis("off")

    def draw(shape, ox, oy):
        for poly in getattr(shape, "geoms", [shape]):
            verts, codes = [], []
            for rr in [poly.exterior, *poly.interiors]:
                c = np.asarray(rr.coords)
                verts += list(np.c_[c[:, 0] + ox, c[:, 1] + oy])
                codes += [MPath.MOVETO] + [MPath.LINETO] * (len(c) - 2) + [MPath.CLOSEPOLY]
            ax.add_patch(PathPatch(MPath(verts, codes), facecolor=(0.93, 0.93, 0.88), edgecolor="black", lw=0.8))

    ox, oy = 8.0, 140.0  # heel piece: page = device (u, v) + offset
    draw(cup.difference(sil), ox, oy)
    tx, ty = -60.0, 90.0  # toe piece, laid out separately on the sheet (each piece is keyed on its own)
    from shapely.geometry import Point

    hook = Point(D["guide_u"][1] + 6.0, D["guide_v"] + 4.0)
    keyed = [p for p in getattr(toe.difference(sil), "geoms", [toe.difference(sil)]) if p.buffer(0.1).contains(hook)]
    draw(keyed[0], tx, ty)  # only the part attached to the hook is keyed
    ax.annotate("toe piece: this edge along the guide's inner face,\nthe hook round the guide's front end",
                (tx + 270, ty + D["guide_v"]), (tx + 175, ty + D["guide_v"] + 40), fontsize=7,
                arrowprops=dict(arrowstyle="->", lw=0.6))
    ax.text(ox + 3, oy - 4, "heel piece:\ncurved edge\nagainst the\ncup's back\nwall", fontsize=6, va="center")
    ax.text(90, 110, "cut out both pieces along the solid lines; lay them on the plate;\nset the form against both "
            "outlines; lift the pieces straight up", fontsize=7)
    ax.text(10, 60, f"Placement card, layout {k} (trials k with k mod 3 = {k}): form "
                    f"men's {int(spec['size'])}{' 1/2' if spec['size'] % 1 else ''} {spec['letter']}", fontsize=10, weight="bold")
    ax.text(10, 52, f"Heel {lay['fwd']:.0f} mm forward of the cup's back wall, {abs(lay['side']):.0f} mm toward the "
                    f"outer side, toes out {abs(lay['yaw']):.1f} deg.", fontsize=8)
    ax.text(10, 45, "Print at 100 % (actual size) on A4. Check: the bar below must measure 100 mm.", fontsize=8)
    ax.plot([10, 110], [25, 25], color="black", lw=2)
    for x in (10, 110):
        ax.plot([x, x], [22, 28], color="black", lw=1)
    ax.text(60, 18, "100 mm", ha="center", fontsize=8)
    path = out / f"placement_layout{k}.pdf"
    fig.savefig(path)
    plt.close(fig)
    return path


if __name__ == "__main__":
    out = HERE / "forms"
    out.mkdir(exist_ok=True)
    for k in range(3):
        print(write_card(k, out))
