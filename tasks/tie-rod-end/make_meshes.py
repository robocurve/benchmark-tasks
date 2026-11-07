"""Write the steering knuckle (its steering arm with the tapered hole) as USDA text (no pxr needed).

python make_meshes.py <out_dir>

Frame: z up from the table, origin on the table under the hole's axis. The steering arm runs out along +x from the
boss around the hole to the knuckle's upright.
"""
import json
import math
import sys
from pathlib import Path

N = 24                    # sides of the tapered hole
# The hole matches the stud's taper: 0.71 in across at its large end, 0.58 in at its small end, over 0.76 in
# [READ: NAPA 269-3444 outer tie rod end, "Large Diameter Taper .71", Small .58, Taper Length .76"]
TAPER_LARGE_R = 0.71 * 0.0254 / 2     # 9.017 mm
TAPER_SMALL_R = 0.58 * 0.0254 / 2     # 7.366 mm
TAPER_L = 0.76 * 0.0254               # 19.30 mm: the boss is as thick as the taper is long
# WHY: a fixed asset spawns 1 mm above the table and stays there, so every height here is 1 mm lower than in the world
REST = 0.001
BOSS_TOP = 0.0906 - REST              # boss top above the table (world): 90.6 mm
BOSS_R = 0.022                        # boss outside radius [EST]
ARM_W, ARM_T = 0.024, 0.016           # steering arm section [EST]
ARM_X = (0.012, 0.080)                # from inside the boss to the upright
UPRIGHT = ((0.080, 0.110), (-0.040, 0.040), (0.0, 0.140))  # the knuckle body the arm grows from [EST]


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
    """A convex collider: the hull of ``pts``, four bottom points then four top points (faces only for display)."""
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


def tapered_ring(name, r_bottom, r_top, r_out, z, color):
    """A ring whose hole widens from ``r_bottom`` at z[0] to ``r_top`` at z[1], as N convex segments. The hole's flats
    sit at the given radii, so the hole is never smaller than the cone."""
    k = 1 / math.cos(math.pi / N)
    parts = []
    for i in range(N):
        t0, t1 = 2 * math.pi * i / N, 2 * math.pi * (i + 1) / N
        ring = []
        for zz, r in ((z[0], r_bottom), (z[1], r_top)):
            a = r * k
            ring += [(a * math.cos(t0), a * math.sin(t0), zz), (r_out * math.cos(t0), r_out * math.sin(t0), zz),
                     (r_out * math.cos(t1), r_out * math.sin(t1), zz), (a * math.cos(t1), a * math.sin(t1), zz)]
        parts.append(hull(f"{name}_{i:02d}", ring, color))
    return parts


def stage(root, parts):
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
    bool physics:kinematicEnabled = 1
{"".join(parts)}
}}
'''


IRON = "(0.24, 0.25, 0.27)"


def knuckle():
    z = (BOSS_TOP - TAPER_L, BOSS_TOP)
    parts = tapered_ring("boss", TAPER_SMALL_R, TAPER_LARGE_R, BOSS_R, z, IRON)
    parts.append(box("arm", ARM_X, (-ARM_W / 2, ARM_W / 2), (z[0], z[0] + ARM_T), IRON))
    parts.append(box("upright", *UPRIGHT, IRON))
    return stage("tie_rod_end_knuckle", parts)


def bounds():
    return {"lo": [-BOSS_R, UPRIGHT[1][0], 0.0], "hi": [UPRIGHT[0][1], UPRIGHT[1][1], UPRIGHT[2][1]],
            "boss_top_world": BOSS_TOP + REST, "taper": [TAPER_SMALL_R, TAPER_LARGE_R, TAPER_L]}


if __name__ == "__main__":
    out = Path(sys.argv[1])
    (out / "tie_rod_end_knuckle.usda").write_text(knuckle())
    print(json.dumps(bounds()))
