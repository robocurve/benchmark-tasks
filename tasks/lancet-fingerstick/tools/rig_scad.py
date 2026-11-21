"""rig_scad.py: writes ../rig.scad, the printable real-trial rig, from the task's own numbers (no hand-copied sizes).

Parts: the base (the palm block, palm up and pitched 20 deg, with the index finger and the thumb, on the post the
slide's rail is screwed to), the seat (the spring pins' bosses and the lower stop post), the ring finger (its two
segments and the web behind its knuckle that the slide's block is bolted to, with a foot round the spring pins), printed
shims under the springs, and the lancet stand (a fixed block and post holding the tab cap, and a cup keyed to the
housing that turns on the post, stopped at +-100 deg). Cut and bought: the plywood board under it all (clamped to the
table), a miniature linear guide, M2 and M3 screws and inserts, two 4 x 80 mm dowels, two 4 mm shaft collars, two
compression springs (specs in the file's header). Units mm; z from the table top; x horizontal along the hand toward
the wrist from the ring's apex, y to the thumb side (the model's hand mechanism frame, with z shifted by its HZ).

WHY generated: the rig must be rebuildable from the model; generating it from lancet_fingerstick.py makes the print take
its sizes from the model (test_rig_scad_is_current checks the committed file against a fresh run). Not rendered here:
OpenSCAD is not installed, so nothing checks that the solids union or slice.

    python3 tools/rig_scad.py            # rewrites rig.scad
    python3 tools/rig_scad.py --check    # exits 1 if rig.scad is stale
"""
import math
import pathlib
import sys
import warnings

warnings.simplefilter("ignore")
HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import lancet_fingerstick as F  # noqa: E402

OUT = HERE / "rig.scad"
PIN_D, PIN_L = 4.0, 45.0   # mm: the spring pins, 4 x 45 mm steel dowels [CHOICE: long enough to carry the collars]
BOSS_D = 8.0         # mm: the shims' diameter, round each pin on the pillar's top [CHOICE]
PIN_PRESS = 10.0     # mm: each pin pressed this far into the seat pillar's top [CHOICE]
PILLAR_SCREWS_Y = 8.0   # mm: two M3 screws up through the board into the pillar's bottom, either side of the axis [CHOICE]
M3_INSERT, M3_CLEAR = 4.0, 3.4   # mm: an M3 heat-set insert's hole, an M3 clearance hole [EST: common sizes]
POST_SCREWS_X = (0.130, 0.155)   # m: two M3 screws up through the board into the post's inserts [CHOICE]
CLAMP_X = 0.400      # m: the C-clamps over the board's bar, 20 mm from its end [CHOICE]
SHIM_T, SHIMS = 1.0, 2      # mm: printed shims under each spring on its boss; one out cuts each spring's force by its
#                             rate x 1 mm [CHOICE: the preload's adjustment, README checks 1 and R]
COLLAR_H, COLLAR_OD = 5.0, 10.0   # mm: a 4 mm bore shaft collar, its width and diameter [EST: a common size]
FOOT_HALF_Y = 18.5   # mm: the web's foot across y, round the pins (15 mm out) [CHOICE]
FINGER_N = 48        # the printed fingers' sides (the sim's colliders have 16, round the same circle)
WEB_LOW_HALF_Y = 8.5   # mm: the web where the block is bolted on, the block's half-width [SOURCE: W 17]
STOP_HALF_Y = 3.0    # mm: the lower stop post [CHOICE]
M2_CLEAR, M2_HEAD, M2_INSERT = 2.4, 4.0, 3.2   # mm: an M2 clearance hole, its head's counterbore, a heat-set insert's
#                                                 hole [EST: common sizes]
RAIL_HOLE_E, RAIL_HOLE_P = 5.0, 15.0   # mm: the rail's first hole from its end and the holes' pitch [SOURCE: E 5, P 15]
SPRING_WIRE, SPRING_MEAN = 0.32, 4.8   # mm: the springs' music wire and mean coil diameter [CHOICE]
G_WIRE = 79300.0     # N/mm^2: music wire's shear modulus [EST: a handbook value, not read here]
CUP_OD = 24.0        # mm: the turning cup's outside diameter [CHOICE]
CUP_SINK = 6.0       # mm: how far the cup sits down in its recess in the stand [CHOICE]
POST_D = 10.0        # mm: the fixed post the cup turns on; the tab cap's slot is in its top [CHOICE]
FIT = 0.4            # mm: on each turning diameter (the cup in its recess, the cup on the post)
STOP_PIN_D = 3.0     # mm: the stop pin in the recess's floor that the cup's arc slot meets at +-100 deg [CHOICE]
STOP_R = 9.0         # mm: the stop pin's radius from the post's axis [CHOICE]


def mm(v):
    return round(1000.0 * float(v), 3)


def pt(p, dz=0.0):
    return f"[{mm(p[0])}, {mm(p[1])}, {mm(p[2] + dz)}]"


def spring(rest, comp, bottom, k_each):
    """An order spec for each slide spring (mm, N): music wire SPRING_WIRE on a mean diameter SPRING_MEAN, the active
    coils that give the rate k = G d^4 / (8 D^3 n), closed ends (two dead coils), and its peak stress at the lower stop
    (Wahl-corrected) [ARITH]."""
    d, D = SPRING_WIRE, SPRING_MEAN
    n = G_WIRE * d ** 4 / (8 * D ** 3 * k_each)
    c = D / d
    f_max = k_each * (rest + comp - bottom)
    wahl = (4 * c - 1) / (4 * c - 4) + 0.615 / c
    tau = wahl * 8 * f_max * D / (math.pi * d ** 3)
    return {"wire": d, "od": round(D + d, 2), "id": round(D - d, 2), "active": round(n, 1), "total": round(n + 2, 1),
            "solid": round((n + 2) * d, 2), "index": round(c, 1), "f_max": round(f_max, 2), "tau": round(tau)}


def site_mech():
    """The demo's site in the mechanism frame (m)."""
    v, u0, _, rd, _ = F.FINGERS["ring"]
    t = F.SITE_TURN
    return F.hand_to_mech((u0 + F.SITE_AIM, v + rd * math.sin(t), rd * math.cos(t)))


def numbers():
    """The rig's numbers (mm), each from the model. The tests and the README read these."""
    hz = F.HZ
    z = lambda v: mm(hz + v)  # noqa: E731
    board_t = mm(F.BOARD_T)
    web = (mm(F.WEB_X[0]), mm(F.WEB_X[1]), mm(2 * F.WEB_HALF_Y), z(F.FOOT_Z[1]), z(F.SLIDE_Z))
    foot = (z(F.FOOT_Z[0]), z(F.FOOT_Z[1]))
    block = (z(F.BLOCK_Z[0]), z(F.BLOCK_Z[1]))
    rail = (round(mm(F.POST_X[0]) - mm(F.RAIL["rail_h"]), 3), mm(F.POST_X[0]), z(F.RAIL_Z[0]), z(F.RAIL_Z[1]))
    pillar_top = z(F.PILLAR_TOP)
    seat = round(pillar_top + SHIMS * SHIM_T, 3)
    rest = round(foot[0] - seat, 3)                              # each spring's length at rest, seat to foot
    comp = round(1000 * F.ring_target(), 3)                      # its compression there (preload plus weight)
    k_each = F.K_RING / 2 / 1000                                 # N/mm, two in parallel
    bottom = round(rest - 1000 * F.DROP_STOP, 3)
    block_mid = (F.WEB_X[1] + F.RAIL["height"] / 2, 0.0, (F.BLOCK_Z[0] + F.BLOCK_Z[1]) / 2)
    lever = round(1000 * math.dist(block_mid, site_mech()), 1)   # the press (at the site) to the block's middle
    hold_x = F.HOLD_AT * float(F.AXIS[0])                       # the hold's point on the ring's axis (mechanism x)
    web_w, web_t = 1000 * 2 * F.WEB_HALF_Y, 1000 * (F.WEB_X[1] - F.WEB_X[0])
    web_a, web_z = web_w * web_t, web_w * web_t ** 2 / 6          # mm^2, mm^3: its bending about y
    rp = F.FINGERS["ring"][4]
    end_x = F.RING_END_S * math.cos(F.PITCH) + rp * abs(math.sin(F.PITCH)) / math.cos(math.pi / FINGER_N)   # the ring's flat base end, its far edge
    collar_top = round(foot[1] + COLLAR_H, 3)
    return {
        "web": web,
        "foot": foot,
        "block": block,
        "block_holes_z": (round((block[0] + block[1]) / 2 - mm(F.RAIL["C"]) / 2, 3),
                          round((block[0] + block[1]) / 2 + mm(F.RAIL["C"]) / 2, 3)),
        "rail": rail,
        "rail_holes_z": tuple(round(rail[2] + RAIL_HOLE_E + i * RAIL_HOLE_P, 3) for i in range(3)),
        "post": (mm(F.POST_X[0]), mm(F.POST_X[1]), mm(F.POST_Y[0]), mm(F.POST_Y[1]), board_t,
                 z(F.POST_TOP)),
        "stop_top": z(F.GUIDE_TOP),
        "drop_stop": round(foot[0] - z(F.GUIDE_TOP), 3),
        "pins": (round(mm(F.WEB_X[0] + F.WEB_X[1]) / 2, 3), mm(F.SPRING_Y)),
        "pin": {"bottom": round(pillar_top - PIN_PRESS, 3), "top": round(pillar_top - PIN_PRESS + PIN_L, 3),
                "length": PIN_L},
        "pillar": (round(mm((F.WEB_X[0] + F.WEB_X[1]) / 2 - F.PILLAR_X), 3),
                   round(mm((F.WEB_X[0] + F.WEB_X[1]) / 2 + F.PILLAR_X), 3), mm(F.PILLAR_Y), board_t, pillar_top),
        "collar": (foot[1], collar_top),
        "collar_under_finger": round(z(F.SLIDE_Z) - mm(rp) - collar_top, 1),
        "seat_top": seat,
        "spring": {"rate_each": round(k_each, 3), "rest": rest, "comp": comp, "free": round(rest + comp, 3),
                   "bottom": bottom, "force_rest_each": round(k_each * comp, 3),
                   **spring(rest, comp, bottom, k_each)},
        # the ring's flat base end clear of the rail's face along x (it never meets the rail, at any drop) [ARITH]
        "finger_clear": round(rail[0] - 1000 * end_x, 2),
        "lever": lever,
        "moment_15N": round(15.0 * lever / 1000, 2),
        # the hold's lift (a vertical force at the hold's point on the ring's axis) on the web's narrow 10 by 6 mm
        # section and on the guide's block, per 15 N, check H's bound [ARITH]
        "hold_web_e": round(1000 * ((F.WEB_X[0] + F.WEB_X[1]) / 2 - hold_x), 1),
        "hold_web_MPa_15N": round(15.0 / web_a + 15.0 * 1000 * ((F.WEB_X[0] + F.WEB_X[1]) / 2 - hold_x) / web_z, 1),
        "hold_moment_15N": round(15.0 * (block_mid[0] - hold_x), 2),
        "board": tuple((mm(x0), mm(x1), mm(y0), mm(y1)) for (x0, x1), (y0, y1) in F.BOARD),
        "board_t": board_t,
        "clamp_x": mm(CLAMP_X),
        "apex_z": mm(hz),
        "pitch_deg": round(math.degrees(F.PITCH), 3),
        "stand": (mm(2 * F.STAND_HALF), mm(F.STAND_H)),
        "cup_pocket": (mm(F.L_W + 2 * F.CUP_CLEAR), mm(F.L_T + 2 * F.CUP_CLEAR), mm(F.CUP_D)),
        "tab_stop_deg": round(math.degrees(F.TAB_STOP), 3),
        "stop_half_deg": round(math.degrees(math.asin((STOP_PIN_D / 2 + FIT / 2) / STOP_R)), 3),
        "base_size": base_size(),
    }


def base_size():
    """The base part's bounding box (mm, x by y by z): the post and the fixed parts (index, palm, thumb), rounded up to
    the mm. WHY: the bed it needs (OpenSCAD is not installed, so this is from the model's shapes, not a render)."""
    shapes = list(F.HAND_SHAPES) + [F.support_shapes()[-1]]
    lo = [min(s.bounds()[0][i] for s in shapes) for i in range(3)]
    hi = [max(s.bounds()[1][i] for s in shapes) for i in range(3)]
    return tuple(int(math.ceil(1000 * (b - a) - 1e-6)) for a, b in zip(lo, hi))


def hull_lines(pts, z):
    return "hull() for (p = [" + ", ".join(pt(p, z) for p in pts) + "]) translate(p) cube(0.01, center = true);"


def rod_lines(a, ra, b, rb, z, note):
    """A flat-ended rod (hand frame a to b), the sim's _rod_hull with FINGER_N sides."""
    return [f"  {hull_lines(F._rod_hull(a, ra, b, rb, F.SKIN, n=FINGER_N).points, z)}   // {note}"]


def finger_lines(name, wall):
    v, u0, u1, rd, rp = F.FINGERS[name]
    z = F.HZ
    tip, dip = F.hand_to_mech((u0 + rd, v, 0.0)), F.hand_to_mech((u0 + F.DIP, v, 0.0))
    rr = f"{mm(rd)} - lance_wall" if wall else f"{mm(rd)}"
    end = (F.RING_END_S + F.APEX[0]) if name == "ring" else u1
    out = [f"  // {name}: distal radius {mm(rd)}, proximal {mm(rp)}"
           + (": the distal pad thinner by the Lance finger's wall, which brings it back" if wall else ""),
           f"  capsule({pt(tip, z)}, {pt(dip, z)}, {rr});"]
    out += rod_lines((u0 + F.DIP, v, 0.0), rp, (end, v, 0.0), rp, z,
                     "its proximal segment" + (", on to the web behind the knuckle" if name == "ring" else ""))
    if name != "ring":
        out.append(f"  translate({pt(F.hand_to_mech((u1, v, 0.0)), z)}) sphere({mm(rp)});   // its knuckle")
    return out


def hand_lines():
    z = F.HZ
    out = ["module index() {"] + finger_lines("index", False)
    v, _, u1, _, rp = F.FINGERS["index"]
    out += rod_lines((u1 - 0.004, v, 0.0), rp, (F.PALM_U[0] + 0.006, v, 0.0), rp, z,
                     "the rod from its knuckle into the palm block")
    out.append("}")
    palm = [F.hand_to_mech((u, v, w)) for w in (-F.PALM_W, F.PALM_W) for u, v in
            ((F.PALM_U[0], F.PALM_V[0]), (F.PALM_U[1], F.PALM_V[0]), (F.PALM_U[1], F.PALM_V[1]),
             (F.PALM_U[0], F.PALM_V[1]))]
    out.append("module palm() { " + hull_lines(palm, z) + " }")
    (tv, tu, tr), (bv, bu, br) = F.THUMB
    out.append("module thumb() {")
    out += rod_lines((tu, tv, 0.0), tr, (bu, bv, 0.0), br, z, "tip to base, as the sim's")
    out.append("}")
    out.append("module ring_finger(lance_wall) {")
    out += finger_lines("ring", True)
    out.append("}")
    return out


def lines():
    n = numbers()
    wx0, wx1, ww, wb, wt = n["web"]
    fb, ft = n["foot"]
    bh0, bh1 = n["block_holes_z"]
    pxm, py = n["pins"]
    px0, px1, py0, py1, pb, ptop = n["post"]
    sp, pin = n["spring"], n["pin"]
    bt = n["board_t"]
    cpw, cpt, cpd = n["cup_pocket"]
    sw, sh = n["stand"]
    stop, half = n["tab_stop_deg"], n["stop_half_deg"]
    face = sh + mm(F.TAB_H)                     # the shield's face rests here: the model's stand top plus its tab layer
    rim = face + cpd                              # the cup's rim: the model's socket wall top
    floor = sh - CUP_SINK
    wide = n["block"][1] + 5.0                    # the web is the block's width up to 5 mm over its top, then narrows
    holes = "[" + ", ".join(f"[{y}, {z}]" for y in (-mm(F.RAIL["B"]) / 2, mm(F.RAIL["B"]) / 2) for z in (bh0, bh1)) + "]"
    ymid = round((py0 + py1) / 2, 3)
    sx0, sx1, sy, _, stp = n["pillar"]
    bs = n["base_size"]
    rects = " ".join(f"translate([{x0}, {y0}]) square([{round(x1 - x0, 3)}, {round(y1 - y0, 3)}]);"
                     for x0, x1, y0, y1 in n["board"])
    r = ["// rig.scad: the real-trial rig for lancet-fingerstick, written by tools/rig_scad.py from lancet_fingerstick.py.",
         "// Do not edit by hand; rerun the tool. Units mm, z from the table top, x level along the hand toward the wrist",
         "// from the ring's apex, y to the thumb side. The hand lies palm up, its fingers sloping "
         f"down {abs(n['pitch_deg']):g} deg toward their tips.",
         "// Render each part on its own: part = \"base\", \"seat\", \"ring\", \"stand\", \"cup\", \"shims\" or "
         "\"board\" (a 2D outline",
         "// to cut from plywood); \"all\" views the set assembled, the ring finger on its top stop.",
         f"// The base prints in one piece, {bs[0]} by {bs[1]} by {bs[2]} mm: a bed of at least {bs[0] + 10} by "
         f"{bs[1] + 10} mm with {bs[2] + 5} mm",
         "// of height (from the model's shapes; not rendered).",
         "// Print (CHOICE, not trialled): PETG; base on the post's bottom with tree supports under the palm, fingers and",
         "// thumb; ring on its side with supports under the pad; seat, stand and shims flat; cup rim up.",
         f"// Cut: the board from {bt:g} mm plywood to the outline (part \"board\"), drilled for the post's and the "
         "pillar's screws.",
         f"// Bought: one miniature linear guide {F.RAIL['part']} ({mm(F.RAIL['rail_len']):g} mm rail, block "
         f"{mm(F.RAIL['block_l'])} mm, pitch moment {F.RAIL['MP']} N m);",
         f"// four M2 x 6 socket screws (web to block, heads in {M2_HEAD:g} mm counterbores 2.2 deep, so 2.2 mm enters the "
         "block's 2.5 mm thread);",
         "// three M2 x 6 screws with M2 heat-set inserts (rail to post); four M3 x 16 screws up through the board into "
         "M3 inserts, two in the post's bottom and two in the pillar's;",
         f"// two {PIN_D:g} x {pin['length']:g} mm steel dowel pins pressed {PIN_PRESS:g} mm into the seat pillar's top; "
         f"two {PIN_D:g} mm bore shaft collars",
         f"// ({COLLAR_OD:g} mm OD, {COLLAR_H:g} mm wide) clamped on the pins on the web's foot, the top stop; two "
         "compression springs, one on each pin",
         f"// under the foot, each music wire {sp['wire']} mm, OD {sp['od']} mm, {sp['active']} active coils, "
         f"{sp['total']} total, ends closed and ground,",
         f"// free length {sp['free']} mm, rate {sp['rate_each']} N/mm, solid {sp['solid']} mm (under the "
         f"{sp['bottom']} mm it reaches) [ARITH; README check R measures it];",
         f"// two C-clamps (throat 60 mm or more) over the board's bar at x {n['clamp_x']:g}, holding it to the table.",
         'part = "all";',
         "lance_wall = 1.0;   // the Lance trainer finger's wall, measured with calipers (README check 3b) [EST until measured]",
         "tab_w = 6.0; tab_t = 3.0; tab_h = 4.0;   // the lancet's tab cap, measured with calipers (README check 3) [EST]",
         "$fn = 48;",
         "module capsule(a, b, r) { hull() { translate(a) sphere(r); translate(b) sphere(r); } }",
         ]
    r += hand_lines()
    r += [
        f"// the slide, on the ring: a web x {wx0} to {wx1} fused under the finger behind its knuckle, "
        f"{2 * WEB_LOW_HALF_Y:g} mm wide up to z {wide}",
        f"// and {ww:g} mm above it, the block bolted to its +x face; its foot (z {fb} to {ft}) rides the two pins,",
        f"// pushed up by the springs against the collars, and lands on the stop post after {n['drop_stop']} mm",
        "module web() { difference() { union() {",
        f"  translate([{wx0}, {-WEB_LOW_HALF_Y}, {fb}]) cube([{round(wx1 - wx0, 3)}, {2 * WEB_LOW_HALF_Y:g}, "
        f"{round(wide - fb, 3)}]);",
        f"  translate([{wx0}, {-ww / 2}, {fb}]) cube([{round(wx1 - wx0, 3)}, {ww}, {round(wt - fb, 3)}]);",
        f"  translate([{wx0}, {-FOOT_HALF_Y}, {fb}]) cube([{round(wx1 - wx0, 3)}, {2 * FOOT_HALF_Y:g}, {round(ft - fb, 3)}]); }}",
        f"  for (y = [{-py}, {py}]) translate([{pxm}, y, {fb - 1}]) cylinder(d = {PIN_D + 0.5:g}, h = {round(ft - fb + 2, 3)});"
        "   // the pins run free",
        f"  for (p = {holes}) translate([{wx0 - 0.01}, p[0], p[1]]) rotate([0, 90, 0]) {{",
        f"    cylinder(d = {M2_CLEAR}, h = {round(wx1 - wx0 + 0.02, 3)}); cylinder(d = {M2_HEAD}, h = 2.2); }}"
        "   // the block's four screws",
        "} }",
        "module ring() { union() { ring_finger(lance_wall); web(); } }",
        f"// the post under the palm block: x {px0} to {px1}, y {py0} to {py1}, z {pb} to {ptop}; the rail screwed to its "
        "-x face through three",
        "// heat-set inserts (z " + ", ".join(f"{v:g}" for v in n["rail_holes_z"]) + "), two M3 inserts in its bottom",
        "module post() { difference() {",
        f"  translate([{px0}, {py0}, {pb}]) cube([{round(px1 - px0, 3)}, {round(py1 - py0, 3)}, {round(ptop - pb, 3)}]);",
        f"  for (z = {list(n['rail_holes_z'])}) translate([{px0 - 0.01}, 0, z]) rotate([0, 90, 0]) "
        f"cylinder(d = {M2_INSERT}, h = 5);",
        f"  for (x = {[mm(v) for v in POST_SCREWS_X]}) translate([x, {ymid}, {pb - 0.01}]) cylinder(d = {M3_INSERT:g}, "
        "h = 8); } }",
        f"// the seat on the board: a pillar x {sx0} to {sx1}, y +-{sy:g}, z {bt:g} to {stp:g} (then {SHIMS} shims of "
        f"{SHIM_T:g} mm: the springs sit at z {n['seat_top']}),",
        f"// the pins pressed {PIN_PRESS:g} mm into its top, and the lower stop post on it (top z {n['stop_top']}); two M3 "
        "screws up through the board hold it",
        "module seat() { difference() { union() {",
        f"  translate([{sx0}, {-sy}, {bt}]) cube([{round(sx1 - sx0, 3)}, {round(2 * sy, 3)}, {round(stp - bt, 3)}]);",
        f"  translate([{wx0}, {-STOP_HALF_Y}, {stp - 0.01}]) cube([{round(wx1 - wx0, 3)}, {2 * STOP_HALF_Y:g}, "
        f"{round(n['stop_top'] - stp + 0.01, 3)}]); }}",
        f"  for (y = [{-py}, {py}]) translate([{pxm}, y, {round(stp - PIN_PRESS, 3)}]) cylinder(d = {PIN_D - 0.1:g}, "
        f"h = {PIN_PRESS + 0.01:g});   // the pins pressed in",
        f"  for (y = [{-PILLAR_SCREWS_Y:g}, {PILLAR_SCREWS_Y:g}]) translate([{pxm}, y, {bt - 0.01}]) "
        f"cylinder(d = {M3_INSERT:g}, h = 8);   // its M3 inserts",
        "} }",
        f"module shims() {{ for (i = [0 : {2 * SHIMS - 1}]) translate([12 * i, 0, 0]) difference() {{ "
        f"cylinder(d = {BOSS_D:g}, h = {SHIM_T:g}); translate([0, 0, -0.01]) cylinder(d = {PIN_D + 0.3:g}, h = {SHIM_T + 0.02:g}); }} }}",
        "module base() { union() { post(); index(); palm(); thumb(); } }",
        f"// the board: {bt:g} mm plywood, a plate under the hand and a bar back between the arm bases, clamped at x "
        f"{n['clamp_x']:g}",
        f"module board_outline() {{ difference() {{ union() {{ {rects} }}",
        f"  for (y = [{-PILLAR_SCREWS_Y:g}, {PILLAR_SCREWS_Y:g}]) translate([{pxm}, y]) circle(d = {M3_CLEAR:g});"
        "   // the pillar's screws",
        f"  for (x = {[mm(v) for v in POST_SCREWS_X]}) translate([x, {ymid}]) circle(d = {M3_CLEAR:g}); }} }}"
        "   // the post's screws",
        f"module board() {{ color(\"tan\") linear_extrude({bt:g}) board_outline(); }}",
        f"module stand() {{ difference() {{ translate([{-sw / 2}, {-sw / 2}, 0]) cube([{sw}, {sw}, {sh}]);",
        f"  translate([0, 0, {floor}]) cylinder(d = {CUP_OD + FIT}, h = {CUP_SINK + 1}); }}   // the cup's recess",
        f"  difference() {{ cylinder(d = {POST_D}, h = {face});   // the fixed post, its top at the shield's face",
        f"    translate([-tab_w / 2, -tab_t / 2, {face} - tab_h]) cube([tab_w, tab_t, tab_h + 0.01]); }}"
        "   // the slot that holds the tab cap still",
        f"  translate([{STOP_R}, 0, {floor}]) cylinder(d = {STOP_PIN_D}, h = 3);   // the stop pin in the recess's floor, at 0 deg",
        "}",
        f"// the cup: keyed to the housing ({cpw} by {cpt} mm, {cpd} mm deep, {mm(F.CUP_CLEAR)} mm each side), turning on the",
        f"// post; an arc slot under it, centred on the stop pin at rest, meets the pin at +-{stop:g} deg, the model's "
        "TAB_STOP either way",
        f"// (the slot spans +-{round(stop + half, 3):g} deg: the stop plus the pin's own {half:g} deg half-width)",
        f"module cup() {{ difference() {{ translate([0, 0, {floor}]) cylinder(d = {CUP_OD}, h = {round(rim - floor, 3)});",
        f"  translate([{-cpw / 2}, {-cpt / 2}, {face}]) cube([{cpw}, {cpt}, {cpd + 1}]);",
        f"  translate([0, 0, {floor - 1}]) cylinder(d = {POST_D + FIT}, h = {round(face - floor + 1.01, 3)});"
        "   // turns on the post",
        f"  translate([0, 0, {floor - 0.01}]) rotate([0, 0, {-round(stop + half, 3):g}]) "
        f"rotate_extrude(angle = {round(2 * (stop + half), 3):g}) "
        f"translate([{STOP_R - STOP_PIN_D / 2 - FIT / 2}, 0]) square([{STOP_PIN_D + FIT}, 3.4]);",
        "} }",
        'if (part == "base" || part == "all") color("grey") base();',
        'if (part == "seat" || part == "all") color("grey") seat();',
        'if (part == "ring" || part == "all") ring();',
        'if (part == "board") board_outline();',
        'if (part == "all") board();',
        'if (part == "stand" || part == "all") translate([0, -120, 0]) stand();',
        'if (part == "cup" || part == "all") translate([0, -120, 0]) color("white") cup();',
        'if (part == "shims") shims();',
    ]
    return r


def text():
    return "\n".join(lines()) + "\n"


if __name__ == "__main__":
    if "--check" in sys.argv:
        sys.exit(0 if OUT.exists() and OUT.read_text() == text() else 1)
    OUT.write_text(text())
    print("wrote", OUT, len(text().splitlines()), "lines")
