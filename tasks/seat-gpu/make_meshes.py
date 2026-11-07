"""Write seat_gpu_card.usda and seat_gpu_riser.usda (plain USDA text, no pxr needed).

Frames: x along the card from its bracket end, y across it (PCB plane at y = 0, cooler toward +y), z up.
Card frame z = 0 is the bottom of the edge fingers. Riser frame z = 0 is the table; the seated card's frame is the
riser frame shifted up by FLOOR (the slot floor).
"""
import json
import sys
from pathlib import Path

# card [READ Wikipedia "PCI Express", PCI-SIG table]: x16 edge 11.65 + 71.65 mm, full height 111.15 mm,
# three-quarter length 254 mm, dual slot pitch 39.04 mm. Others are [EST].
L, H = 0.254, 0.11115
FINGER = 0.0085           # edge fingers below the board edge [EST]
PCB = 0.0016              # [EST] 1.57 mm board
Y0, Y1 = -0.002, 0.033    # body across y: 35 mm, cooler toward +y [EST]
A = 0.040                 # fingers start 40 mm from the bracket end [EST]
SHORT, NOTCH, LONG = 0.01165, 0.0020, 0.07165
BR_X = (-0.0040, -0.0030)  # bracket plate, 1 mm thick, 3 mm proud of the body
BR_Y = (-0.004, 0.035)
# WHY: the bracket's tab reaches 2 mm above the bottom of the gold fingers, below the board edge, as on a real card.
# Ending above the board edge, it forced a lip tall enough for the board to land on and pivot over.
BR_Z = (0.002, 0.1200)

# riser
BOARD_TOP = 0.006
HOUSING_TOP = BOARD_TOP + 0.01125
SLOT_HALF = 0.0010        # 2.0 mm slot for a 1.6 mm board
MOUTH_HALF = 0.0025       # lead-in mouth 5 mm wide, over the top 3.5 mm [EST, wider than a real housing]
# WHY: every asset gets the scene's friction of 1.2 (yam_arena.constants.GRIPPY_FRICTION, bound over the USD's own), so a
# corner only slides down a lead-in steeper than atan(1.2) = 50 deg. All lead-ins here are about 66 deg or steeper.
LEAD = 0.0035   # the slot's lead-ins open 1.5 mm over the top 3.5 mm (67 deg)
DEPTH = FINGER            # board edge reaches the housing top as the fingers reach the floor
FLOOR = HOUSING_TOP - DEPTH
END_PLAY = 0.0005
RIB = (A + SHORT + 0.0004, A + SHORT + NOTCH - 0.0004)
# WHY: the key rib's ridge sits 5 mm under the housing top, so the slot's end walls (0.5 mm play) square the card along
# the slot before the notch meets the rib (0.4 mm play); level with the top, a card 1 mm off pivoted on the rib
KEY_TOP = HOUSING_TOP - 0.005
# bracket gap between rear wall and lip: 2.9 mm, centred on the 1 mm bracket (x -4 to -3 mm), 0.95 mm clear each side
GAP = (-0.00495, -0.00205)
# WHY: the gap opens over its top 4 mm, 1.5 mm on the rear-wall side but only 0.8 mm on the lip side, since the lip is
# 1 mm thick (the card's board starts at x = 0)
WALL_LEAD, REAR_FLARE, LIP_FLARE = 0.004, 0.0015, 0.0008
LIP_TOP = 0.016   # under the seated board edge (17.25 mm), over the seated bracket tab's bottom (10.75 mm)
WALL_TOP = 0.035
# far-end support: a channel 0.5 mm wider than the body each side, its top 15 mm flared out 6 mm each side (68 deg)
# [EST, server riser cages flare their card guides]
GUIDE_X = (0.200, 0.250)
GUIDE_TOP = 0.065   # above the riser's base, 59 mm above the board
GUIDE_PLAY = 0.0005
GUIDE_LEAD = 0.015
GUIDE_FLARE = 0.006
BOARD_X = (-0.012, 0.265)


def box(name, x, y, z, color):
    sx, sy, sz = x[1] - x[0], y[1] - y[0], z[1] - z[0]
    cx, cy, cz = (x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2
    return f'''
    def Cube "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
        double size = 1
        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
        color3f[] primvars:displayColor = [{color}]
        double3 xformOp:translate = ({cx:.6f}, {cy:.6f}, {cz:.6f})
        float3 xformOp:scale = ({sx:.6f}, {sy:.6f}, {sz:.6f})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}'''


def prism(name, x, yz, color, along="x"):
    """A convex prism along x (or y) whose cross-section is the polygon yz (or xz)."""
    n = len(yz)
    if along == "x":
        pts = [(x[0], y, z) for y, z in yz] + [(x[1], y, z) for y, z in yz]
    else:
        pts = [(u, x[0], z) for u, z in yz] + [(u, x[1], z) for u, z in yz]
    faces = [list(range(n - 1, -1, -1)), list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    lo = [min(p[k] for p in pts) for k in range(3)]
    hi = [max(p[k] for p in pts) for k in range(3)]
    fmt = lambda p: "(" + ", ".join(f"{v:.6f}" for v in p) + ")"
    return f'''
    def Mesh "{name}" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI"]
    )
    {{
        uniform token physics:approximation = "convexHull"
        int[] faceVertexCounts = [{", ".join(str(len(f)) for f in faces)}]
        int[] faceVertexIndices = [{", ".join(str(i) for f in faces for i in f)}]
        point3f[] points = [{", ".join(fmt(p) for p in pts)}]
        float3[] extent = [{fmt(lo)}, {fmt(hi)}]
        color3f[] primvars:displayColor = [{color}]
        uniform token subdivisionScheme = "none"
    }}'''


def stage(root, parts, kinematic):
    body = "".join(parts)
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
{body}
}}
'''


def card():
    grey, gold, steel = "(0.10, 0.12, 0.14)", "(0.85, 0.65, 0.10)", "(0.75, 0.75, 0.78)"
    fy = (-PCB / 2, PCB / 2)
    return stage("seat_gpu_card", [
        box("body", (0.0, L), (Y0, Y1), (FINGER, H), grey),
        box("fingers_short", (A, A + SHORT), fy, (0.0, FINGER), gold),
        box("fingers_long", (A + SHORT + NOTCH, A + SHORT + NOTCH + LONG), fy, (0.0, FINGER), gold),
        box("bracket", BR_X, BR_Y, BR_Z, steel),
    ], kinematic=False)


def riser():
    green, black, steel = "(0.05, 0.45, 0.15)", "(0.04, 0.04, 0.04)", "(0.75, 0.75, 0.78)"
    x0, x1 = A - END_PLAY, A + SHORT + NOTCH + LONG + END_PLAY   # slot interior along x
    hx = (x0 - 0.0016, x1 + 0.0016)                              # housing, 89 mm overall
    yw = 0.0044
    top_in = HOUSING_TOP - LEAD
    parts = [
        box("board", BOARD_X, (-0.030, 0.050), (0.0, BOARD_TOP), green),
        box("floor", (x0, x1), (-SLOT_HALF, SLOT_HALF), (BOARD_TOP, FLOOR), black),
        box("wall_neg", hx, (-yw, -SLOT_HALF), (BOARD_TOP, top_in), black),
        box("wall_pos", hx, (SLOT_HALF, yw), (BOARD_TOP, top_in), black),
        prism("lead_neg", hx, [(-yw, top_in), (-SLOT_HALF, top_in), (-MOUTH_HALF, HOUSING_TOP), (-yw, HOUSING_TOP)], black),
        prism("lead_pos", hx, [(SLOT_HALF, top_in), (yw, top_in), (yw, HOUSING_TOP), (MOUTH_HALF, HOUSING_TOP)], black),
        box("end_near", (hx[0], x0), (-SLOT_HALF, SLOT_HALF), (FLOOR, top_in), black),
        box("end_far", (x1, hx[1]), (-SLOT_HALF, SLOT_HALF), (FLOOR, top_in), black),
        box("key", RIB, (-SLOT_HALF, SLOT_HALF), (FLOOR, KEY_TOP - 0.0015), black),
        # lead-ins along x over the top 2 mm: the end walls slope outward, the key rib comes to a ridge
        prism("end_near_lead", (-SLOT_HALF, SLOT_HALF),
              [(hx[0], top_in), (x0, top_in), (x0 - (MOUTH_HALF - SLOT_HALF), HOUSING_TOP), (hx[0], HOUSING_TOP)],
              black, along="y"),
        prism("end_far_lead", (-SLOT_HALF, SLOT_HALF),
              [(x1, top_in), (hx[1], top_in), (hx[1], HOUSING_TOP), (x1 + (MOUTH_HALF - SLOT_HALF), HOUSING_TOP)],
              black, along="y"),
        prism("key_lead", (-SLOT_HALF, SLOT_HALF),
              [(RIB[0], KEY_TOP - 0.0015), (RIB[1], KEY_TOP - 0.0015), ((RIB[0] + RIB[1]) / 2, KEY_TOP)], black, along="y"),
        box("rear_wall", (-0.0085, GAP[0]), (-0.008, 0.040), (BOARD_TOP, WALL_TOP - WALL_LEAD), steel),
        box("lip", (GAP[1], -0.0010), (-0.008, 0.040), (BOARD_TOP, LIP_TOP - LEAD), steel),
        prism("rear_wall_lead", (-0.008, 0.040),
              [(-0.0085, WALL_TOP - WALL_LEAD), (GAP[0], WALL_TOP - WALL_LEAD), (GAP[0] - REAR_FLARE, WALL_TOP),
               (-0.0085, WALL_TOP)], steel, along="y"),
        prism("lip_lead", (-0.008, 0.040),
              [(GAP[1], LIP_TOP - LEAD), (-0.0010, LIP_TOP - LEAD), (-0.0010, LIP_TOP),
               (GAP[1] + LIP_FLARE, LIP_TOP)], steel, along="y"),
    ]
    gi, go = Y0 - GUIDE_PLAY, Y1 + GUIDE_PLAY        # inner faces of the far-end channel
    gt = GUIDE_TOP - GUIDE_LEAD
    parts += [
        box("guide_neg", GUIDE_X, (gi - 0.004, gi), (BOARD_TOP, gt), steel),
        box("guide_pos", GUIDE_X, (go, go + 0.004), (BOARD_TOP, gt), steel),
        prism("guide_neg_lead", GUIDE_X, [(gi - GUIDE_FLARE - 0.002, gt), (gi, gt), (gi - GUIDE_FLARE, GUIDE_TOP),
                                          (gi - GUIDE_FLARE - 0.002, GUIDE_TOP)], steel),
        prism("guide_pos_lead", GUIDE_X, [(go, gt), (go + GUIDE_FLARE + 0.002, gt), (go + GUIDE_FLARE + 0.002, GUIDE_TOP),
                                          (go + GUIDE_FLARE, GUIDE_TOP)], steel),
    ]
    return stage("seat_gpu_riser", parts, kinematic=True)


def bounds(kind):
    if kind == "card":
        return {"lo": [BR_X[0], min(Y0, BR_Y[0]), 0.0], "hi": [L, max(Y1, BR_Y[1]), H]}
    return {"lo": [BOARD_X[0], -0.030, 0.0], "hi": [BOARD_X[1], 0.050, GUIDE_TOP]}


if __name__ == "__main__":
    out = Path(sys.argv[1])
    (out / "seat_gpu_card.usda").write_text(card())
    (out / "seat_gpu_riser.usda").write_text(riser())
    print(json.dumps({"card": bounds("card"), "riser": bounds("riser"), "FLOOR": FLOOR, "HOUSING_TOP": HOUSING_TOP}))
