#!/usr/bin/env python3
"""export_grip_cad.py -- dimensioned, printable lever-wedge grip (one grip; print two), ASCII STL in millimetres, v2.
Task numbers (qc_met10.py): HOUSING_A/B/D 49.5/62.2/38.1, jaw width 19.1, gap 0-6.4, lift 6.2, lever 22 mm, RAMP_RUN 16, RAMP_Y 9, slot end 33 mm behind the
jaw face, jaw tips 3 mm out of the front face. Frame: x along the slot (lever moves +x), y along the pull axis (y = 0 housing back face, specimen leaves y = 62.2),
z up from the housing underside. Mapping to the simulation model: jaw plate is 32 mm long (33 - 1 mm clearance) and its front end stands 3 mm out of the housing
front face (y = 65.2); rails are 5 mm wide at yc +- 9; the stem sits at yc = 47.2 (2 mm behind the plate centre, a printing choice, not in the model).
`python3 export_grip_cad.py [outdir]` writes the STLs and assembly.json (solids as polygons extruded along y or boxes, springs and drill list);
`python3 check_assembly.py` runs the mechanical checks on exactly this geometry.
Printed per grip: housing (incl. clamp shank and return arm), jaw (plate + stem), crossbar, carriage (slides on the roof, two sloped wedge rails, handle), (the spring arch is part of the housing).
Bought per grip: 2 jaw compression springs (McMaster 8969T315) and 2 return extension springs (McMaster 1942N5), catalogue values in the constants above;
one M3 x 10 screw (crossbar). For two grips: 4 + 4 springs. The 10 x 10 mm shank goes straight into the stand's own clamp (REAL-TRIAL-PARTS.md)."""
import sys, json, pathlib

D, B, A = 38.1, 62.2, 49.5
JAW_W, JAW_L, JAW_H = 19.1, 32.0, 8.0
GAP_MAX = 6.2
FLOOR = 14.0
SLOT_H = 6.4 + JAW_H + 0.6
ROOF_Z = FLOOR + SLOT_H
Y_SLOT_END = (B + 3.0) - 33.0
YC = (Y_SLOT_END + B) / 2.0
STEM = (6.0, 10.0); HOLE = (6.8, 10.8)
XS = D / 2.0
FOLLOW_UNDER = A + 5.0
CROSS = (6.0, 25.4, 4.0)
RAMP_Y, RAIL_W = 9.0, 5.0
PLATE_T = 4.0
RAMP_RUN, LIFT, LAND = 16.0, 7.2, 12.0
TRAVEL = 22.0
RIDGE_T, RIDGE_H = 2.0, 4.0
X_TOE = XS - 1.28                      # ramp toe in plate coordinates (the crossbar front edge touches the ramp at s = 0.55 mm)
SPRING_Y = 9.5                         # compression springs at yc +- 10 between crossbar top and the cap
# jaw compression spring (catalogue values, one spring of the pair): McMaster-Carr 8969T315, 316 stainless, OD 5.4 mm, wire 0.4 mm, free length 16 mm,
# length at maximum load 7.5 mm, rate 0.07 lbf/mm = 0.311 N/mm (two significant digits in the catalogue; no solid height, no price on the page)
K_JAW, FREE_JAW, MAXLOAD_LEN = 0.311, 16.0, 7.5
SLACK = 0.1                                           # installed length with the jaw closed = free length + 0.1 mm (spring rests on its pins, zero force); matches the model's force within 0.1 N at 3.2 and 6.2 mm
CAP_Z0 = FOLLOW_UNDER + CROSS[2] + FREE_JAW + SLACK    # bridge underside
CAP_Z1 = CAP_Z0 + 4.0
ANCHOR_X = -34.0                       # centre x of the two anchor pins on the arm (hooks bear on their +x faces)
# carriage return: two McMaster-Carr 1942N5 extension springs in parallel per grip (302 stainless, hooked, free length 17.5 mm, OD 3 mm, wire 0.3 mm,
# 0.08 lb (0.356 N) initial tension, rate 0.03 lbf/mm = 0.133 N/mm, 0.84 lb max load at 46.3 mm; the catalogue gives no hook length)
K_RET, F0_RET, FREE_RET, MAXLEN_RET = 2 * 0.1334, 2 * 0.356, 17.5, 46.3
HOOK_DY = 1.75                         # the two springs sit +-1.75 mm either side of the rail line (3.5 mm apart for a 3 mm OD)
SHANK = (10.0, 10.0, 25.0)             # clamp shank on the housing back face: x, z, length along -y (clamped in the printed block)

def box(x0, y0, z0, x1, y1, z1): return {"t": "box", "b": (x0, y0, z0, x1, y1, z1)}
def prism(pts, y0, y1): return {"t": "prism", "pts": pts, "y": (y0, y1)}      # polygon in (x, z), extruded along y

def housing():
    xl, xr = XS - HOLE[0] / 2, XS + HOLE[0] / 2; yl, yr = YC - HOLE[1] / 2, YC + HOLE[1] / 2
    s = [box(0, 0, 0, D, B + 3.0, FLOOR), box(0, 0, FLOOR, D, Y_SLOT_END, A),            # floor runs to the jaw face (y 65.2), back block behind the slot end
         box(0, Y_SLOT_END, ROOF_Z, xl, B, A), box(xr, Y_SLOT_END, ROOF_Z, D, B, A),
         box(xl, Y_SLOT_END, ROOF_Z, xr, yl, A), box(xl, yr, ROOF_Z, xr, B, A),
         box(0, Y_SLOT_END, A, D, Y_SLOT_END + RIDGE_T, A + RIDGE_H), box(0, B - RIDGE_T, A, D, B, A + RIDGE_H),
         box(XS - SHANK[0] / 2, -SHANK[2], A / 2 - SHANK[1] / 2, XS + SHANK[0] / 2, 0, A / 2 + SHANK[1] / 2)]         # clamp shank, 10 x 10 x 25
    ya = YC - RAMP_Y - 2.0
    ya = YC - RAMP_Y - 4.0                                                                                           # 8 mm wide return arm along the rail line
    s.append(box(ANCHOR_X - 3.0, ya, A - 6.0, 0, ya + 8.0, A))                                                       # arm, fused to the roof front face
    for sg in (-1, 1):                                                                                               # two anchor pins 2 x 2 x 8 (hooks hang on them from above)
        yy = YC - RAMP_Y + sg * HOOK_DY
        s.append(box(ANCHOR_X - 1.0, yy - 1.0, A, ANCHOR_X + 1.0, yy + 1.0, A + 8.0))
    s.append(box(XS - 4.0, Y_SLOT_END, A + RIDGE_H, XS + 4.0, Y_SLOT_END + 1.8, CAP_Z1))                              # arch leg, -y (clears the crossbar by 0.3 mm)
    s.append(box(XS - 4.0, B - 1.8, A + RIDGE_H, XS + 4.0, B, CAP_Z1))                                                # arch leg, +y
    s.append(box(XS - 4.0, Y_SLOT_END, CAP_Z0, XS + 4.0, B, CAP_Z1))                                                  # arch bridge, printed with the housing
    for sg in (-1, 1):                                                                                               # spring locating pins under the bridge, 2 x 2 x 3
        s.append(box(XS - 1.0, YC + sg * SPRING_Y - 1.0, CAP_Z0 - 3.0, XS + 1.0, YC + sg * SPRING_Y + 1.0, CAP_Z0))
    return s

def jaw():
    x0, x1 = XS - JAW_W / 2, XS + JAW_W / 2; yf = B + 3.0
    return [box(x0, yf - JAW_L, FLOOR, x1, yf, FLOOR + JAW_H),
            box(XS - STEM[0] / 2, YC - STEM[1] / 2, FLOOR + JAW_H, XS + STEM[0] / 2, YC + STEM[1] / 2, FOLLOW_UNDER + 1.5)]            # stem top at z 56.0, inside the crossbar's underside channel

def crossbar():   # slides in along x over the stem top (channel 10.4 mm wide, 1.5 mm deep, open at both ends), fixed with one M3 x 10 from the top
    y0, y1 = YC - CROSS[1] / 2, YC + CROSS[1] / 2; x0, x1 = XS - CROSS[0] / 2, XS + CROSS[0] / 2
    out = [box(x0, y0, FOLLOW_UNDER + 1.5, x1, y1, FOLLOW_UNDER + CROSS[2]),
           box(x0, y0, FOLLOW_UNDER, x1, YC - 5.2, FOLLOW_UNDER + 1.5), box(x0, YC + 5.2, FOLLOW_UNDER, x1, y1, FOLLOW_UNDER + 1.5)]
    for sg in (-1, 1):
        out.append(box(XS - 1.0, YC + sg * SPRING_Y - 1.0, FOLLOW_UNDER + CROSS[2], XS + 1.0, YC + sg * SPRING_Y + 1.0, FOLLOW_UNDER + CROSS[2] + 3.0))
    return out

def carriage(s=0.0):
    x0 = X_TOE - RAMP_RUN - LAND + s; xb = D + 4.0 + s
    ya, yb = Y_SLOT_END + RIDGE_T + 0.3, B - RIDGE_T - 0.3
    z0, z1 = A, A + PLATE_T
    out = [box(x0, ya, z0, xb, YC - 5.6, z1), box(x0, YC + 5.6, z0, xb, yb, z1), box(XS + 3.4 + s, YC - 5.6, z0, xb, YC + 5.6, z1)]
    for sgn in (-1, 1):
        yr0, yr1 = YC + sgn * RAMP_Y - RAIL_W / 2, YC + sgn * RAMP_Y + RAIL_W / 2
        pts = [(x0, z1), (X_TOE + s, z1), (X_TOE - RAMP_RUN + s, z1 + LIFT), (x0, z1 + LIFT)]   # sloped face from the toe, flat land to the front end
        out.append(prism(pts, yr0, yr1))
    out.append(box(x0 - 8.0, YC - RAMP_Y - 4.0, z0, x0, YC - RAMP_Y + 4.0, z0 + 2.0))   # tab on the plate front, 2 mm thick
    for sg in (-1, 1):                                                                       # two hook pins 2 x 2 x 4 on the tab (hooks bear on their -x faces)
        yy = YC - RAMP_Y + sg * HOOK_DY
        out.append(box(x0 - 5.0, yy - 1.0, z0 + 2.0, x0 - 3.0, yy + 1.0, z0 + 6.0))
    out.append(box(xb, YC - 13.0, A + 1.0, xb + 6.0, YC + 13.0, A + 31.0))               # handle 6 x 26 x 30
    return out

def springs(s=0.0):   # return-spring hook bearing faces taken from the solids: anchor pins' +x face, carriage pins' -x face
    xa = max(sd["b"][3] for sd in housing() if sd["t"] == "box" and abs(sd["b"][3] - sd["b"][0] - 2.0) < 1e-9 and abs(sd["b"][5] - sd["b"][2] - 8.0) < 1e-9)
    xc = min(sd["b"][0] for sd in carriage(s) if sd["t"] == "box" and abs(sd["b"][3] - sd["b"][0] - 2.0) < 1e-9 and abs(sd["b"][5] - sd["b"][2] - 4.0) < 1e-9)
    return {"ret_len": xc - xa, "z": A + 4.0}

def tri_prism(p, y0, y1):
    n = len(p); T = []
    top = [(x, y1, z) for x, z in p]; bot = [(x, y0, z) for x, z in p]
    for i in range(1, n - 1): T += [(bot[0], bot[i + 1], bot[i]), (top[0], top[i], top[i + 1])]
    for i in range(n):
        j = (i + 1) % n; T += [(bot[i], bot[j], top[j]), (bot[i], top[j], top[i])]
    return T

def tri_box(b):
    x0, y0, z0, x1, y1, z1 = b
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    f = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (2, 3, 7), (2, 7, 6), (1, 2, 6), (1, 6, 5), (3, 0, 4), (3, 4, 7)]
    return [(v[a], v[b_], v[c]) for a, b_, c in f]

def write(path, name, solids):
    with open(path, "w") as fh:
        fh.write(f"solid {name}\n")
        for sd in solids:
            for a, b, c in (tri_box(sd["b"]) if sd["t"] == "box" else tri_prism(sd["pts"], *sd["y"])):
                ux, uy, uz = (b[i] - a[i] for i in range(3)); vx, vy, vz = (c[i] - a[i] for i in range(3))
                n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx); m = sum(k * k for k in n) ** .5 or 1.0
                fh.write(f"facet normal {n[0]/m:.6f} {n[1]/m:.6f} {n[2]/m:.6f}\n outer loop\n")
                for p in (a, b, c): fh.write(f"  vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
                fh.write(" endloop\nendfacet\n")
        fh.write(f"endsolid {name}\n")

DRILL = [
    {"part": "grip_block", "what": "two M4 set-screw holes through one side wall into the pocket, centred on y 15, 3.3 mm self-tapping pilot", "x": "side face", "y": "15", "z": "17 and 27 (the pocket spans z 10..30)", "dia": 3.3},  # post-print holes, mm in the frame above (the STLs are solid there)
    {"part": "crossbar", "what": "M3 clearance, vertical, through the 2.5 mm top plate", "x": XS, "y": YC, "z": f"{FOLLOW_UNDER + 1.5}..{FOLLOW_UNDER + CROSS[2]}", "dia": 3.2},
    {"part": "jaw", "what": "M3 x 10 self-tapping pilot in the stem top, depth 8 (screw engages 10 - 2.5 = 7.5 mm)", "x": XS, "y": YC, "z": f"{FOLLOW_UNDER + 1.5} down to {FOLLOW_UNDER - 6.5}", "dia": 2.5},
]
POCKET = SHANK[0] + 0.2               # clamp-block pocket for the 10 mm shank (0.1 mm clearance each side)
def grip_block():   # printed clamp block that holds one grip's shank: 30 x 30 x 30 mm, pocket 10.2 x 10.2 x 20 deep, two M4 set screws drilled on assembly
    W = 30.0; px0 = (W - POCKET) / 2.0; px1 = px0 + POCKET; floor = 10.0
    return [box(0, 0, 0, W, W, floor), box(0, 0, floor, px0, W, W), box(px1, 0, floor, W, W, W),
            box(px0, 0, floor, px1, px0, W), box(px0, px1, floor, px1, W, W)]

PARTS = {"grip_housing": housing, "grip_jaw": jaw, "grip_crossbar": crossbar, "grip_carriage": carriage, "grip_block": grip_block}
if __name__ == "__main__":
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "stl"); out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("grip_*.stl"): old.unlink()
    asm = {}
    for n, f in PARTS.items():
        sd = f(); write(out / f"{n}.stl", n, sd); asm[n] = sd; print("wrote", out / f"{n}.stl", len(sd), "solids")
    asm["drill"] = DRILL
    json.dump(asm, open(out / "assembly.json", "w"), indent=1)
