"""Write the driveshaft tube yoke and the universal-joint cross as USDA text (no pxr needed).

python make_meshes.py <out_dir>

Frames: z up from the table. The yoke frame's origin is on the table under the middle of its cross holes' axis, which
runs along y; the tube runs along +x. The cross frame's origin is on the tip of one journal of the second pair, with
the first pair along y and the second along z, so the cross's centre is HALF_SPAN above the origin.
"""
import json
import math
import sys
from pathlib import Path

N = 24                       # sides of every round hole and journal
# Spicer 1310 series (Spicer K350 catalog, December 2017, "Outside Snap Ring (OSR) Style", kit 5-153X): bearing cap
# diameter 1.062 in, 3.219 in over the caps [READ]. With the caps removed the cross holes are the cap bores.
BORE_R = 0.02697 / 2
OVER_CAPS = 0.0818
EAR_T = 0.011                # [EST] ear thickness; the caps fill the ears, so the ears' outer faces are OVER_CAPS apart
GAP = OVER_CAPS - 2 * EAR_T  # between the ears' inner faces: 59.8 mm
AXIS_H = 0.060               # cross holes' axis above the table [EST, chosen so the hanging journal clears the table]
BOSS_R = 0.022               # each ear is round about its hole, then runs back to the crotch
TANG_X = (0.015, 0.030)
CROTCH_X = (0.030, 0.046)
TUBE_R, TUBE_X = 0.0381, (0.046, 0.120)   # 3 in tube [EST]
STAND = ((0.055, 0.115), (-0.032, 0.032), (0.0, 0.0219))

# Cross [EST]: 15 mm journals, 73 mm journal tip to tip (the 81.8 mm over caps less the two cap ends), 26 mm hub.
# Centred in the yoke each journal reaches 6.6 mm past its ear's inner face, so the cross cannot come down level.
JOURNAL_R = 0.0075
HALF_SPAN = 0.0365
HUB = 0.013                  # half the hub's side


def _fmt(p):
    return "(" + ", ".join(f"{v:.6f}" for v in p) + ")"


def box(name, x, y, z, color):
    sx, sy, sz = x[1] - x[0], y[1] - y[0], z[1] - z[0]
    c = ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2)
    return f'''
    def Cube "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        double size = 1
        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
        color3f[] primvars:displayColor = [{color}]
        double3 xformOp:translate = {_fmt(c)}
        float3 xformOp:scale = {_fmt((sx, sy, sz))}
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}'''


def hull(name, pts, color):
    """A convex collider: the hull of ``pts`` (two rings of n points; its faces are only for display)."""
    n = len(pts) // 2
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    return f'''
    def Mesh "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
    )
    {{
        uniform token physics:approximation = "convexHull"
        int[] faceVertexCounts = [{", ".join(str(len(f)) for f in faces)}]
        int[] faceVertexIndices = [{", ".join(str(i) for f in faces for i in f)}]
        point3f[] points = [{", ".join(_fmt(p) for p in pts)}]
        float3[] extent = [{_fmt(lo)}, {_fmt(hi)}]
        color3f[] primvars:displayColor = [{color}]
        uniform token subdivisionScheme = "none"
    }}'''


def circle(r, n=N):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)) for k in range(n)]


def rod(name, axis, r, span, color, c=(0.0, 0.0)):
    """A round rod along ``axis`` ('x', 'y' or 'z') from span[0] to span[1], centred at c in the other two axes."""
    pts = []
    for s in span:
        for u, v in circle(r):
            u, v = u + c[0], v + c[1]
            pts.append({"x": (s, u, v), "y": (u, s, v), "z": (u, v, s)}[axis])
    return hull(name, pts, color)


def ring_y(name, r_in, r_out, y, z0, color):
    """A ring about an axis along y at height z0, as N convex segments. Inner flats sit at r_in, so the hole is never
    smaller than r_in."""
    a_in = r_in / math.cos(math.pi / N)
    parts = []
    for k in range(N):
        t0, t1 = 2 * math.pi * k / N, 2 * math.pi * (k + 1) / N
        poly = [(a_in * math.cos(t0), a_in * math.sin(t0)), (r_out * math.cos(t0), r_out * math.sin(t0)),
                (r_out * math.cos(t1), r_out * math.sin(t1)), (a_in * math.cos(t1), a_in * math.sin(t1))]
        pts = [(x, y[0], z0 + z) for x, z in poly] + [(x, y[1], z0 + z) for x, z in poly]
        parts.append(hull(f"{name}_{k:02d}", pts, color))
    return parts


def stage(root, parts, kinematic):
    return f'''#usda 1.0
(
    defaultPrim = "{root}"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "{root}" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
)
{{
    bool physics:kinematicEnabled = {"1" if kinematic else "0"}
{"".join(parts)}
}}
'''


YOKE, GREYC, ORANGE, STEEL = "(0.22, 0.30, 0.50)", "(0.45, 0.45, 0.47)", "(0.95, 0.45, 0.05)", "(0.82, 0.82, 0.85)"


def yoke():
    parts = []
    z = (AXIS_H - BOSS_R, AXIS_H + BOSS_R)
    for nm, s in (("left", 1), ("right", -1)):
        y = tuple(sorted((s * GAP / 2, s * (GAP / 2 + EAR_T))))
        parts += ring_y(f"{nm}_ear", BORE_R, BOSS_R, y, AXIS_H, YOKE)
        parts.append(box(f"{nm}_ear_tang", TANG_X, y, z, YOKE))
    parts.append(box("crotch", CROTCH_X, (-OVER_CAPS / 2, OVER_CAPS / 2), z, YOKE))
    parts.append(rod("tube", "x", TUBE_R, TUBE_X, YOKE, c=(0.0, AXIS_H)))
    parts.append(box("stand", *STAND, GREYC))
    return stage("ujoint_yoke", parts, kinematic=True)


def cross():
    c = HALF_SPAN
    parts = [box("hub", (-HUB, HUB), (-HUB, HUB), (c - HUB, c + HUB), ORANGE)]
    for nm, s in (("a_left", 1), ("a_right", -1)):
        parts.append(rod(f"journal_{nm}", "y", JOURNAL_R, tuple(sorted((s * (HUB - 0.001), s * HALF_SPAN))), STEEL,
                         c=(0.0, c)))
    for nm, span in (("b_top", (c + HUB - 0.001, 2 * c)), ("b_bottom", (0.0, c - HUB + 0.001))):
        parts.append(rod(f"journal_{nm}", "z", JOURNAL_R, span, STEEL))
    return stage("ujoint_cross", parts, kinematic=False)


def bounds():
    return {
        "yoke": {"lo": [-BOSS_R, -OVER_CAPS / 2, 0.0], "hi": [TUBE_X[1], OVER_CAPS / 2, AXIS_H + TUBE_R]},
        "cross": {"lo": [-HUB, -HALF_SPAN, 0.0], "hi": [HUB, HALF_SPAN, 2 * HALF_SPAN]},
        "GAP": GAP, "AXIS_H": AXIS_H, "BORE_R": BORE_R,
    }


if __name__ == "__main__":
    out = Path(sys.argv[1])
    (out / "ujoint_yoke.usda").write_text(yoke())
    (out / "ujoint_cross.usda").write_text(cross())
    print(json.dumps(bounds()))
