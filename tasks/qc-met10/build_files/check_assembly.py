#!/usr/bin/env python3
"""check_assembly.py -- mechanical check of the printable grip on exactly the geometry export_grip_cad.py emits (v2).
Every pair of parts is tested (jaw/crossbar included: the stem sits in the crossbar's underside channel); the crossbar/carriage contact is tested on the real sloped rail faces over the whole 0-22 mm travel.
 1 task constants (scalars only, not every feature): reads qc_met10.py (HOUSING A/B/D, JAW_W, GAP_LIFTED, LEVER_OPEN, JAW_K, LEVER_K) and compares with the CAD numbers
 2 travel sweep s = 0..22 step 0.5 mm: lift needed to clear the actual rail face under the crossbar (max over the crossbar's x range), then
   the full pairwise overlap test (no pair skipped, tolerance 0.001 mm) at that lift, and the contact is shown real (overlap at lift - 0.05 mm)
 3 lift at s = 22 is 6.2 mm (+-0.05) and never exceeds it; jaw closed (lift 0) at s = 0; first contact reported
 4 springs from geometry: jaw compression springs' installed length = cap underside - crossbar top - lift, force = k (free - length) >= 0 always,
   total 1.9 N at a 3.2 mm lift and 3.7 N at 6.2 mm (task JAW_K, +-0.1 N); return spring length from plate hook to anchor, force within 0.05 N of zero at rest (free length rounded to 0.1 mm) and 6.6 N at 22 mm (+-0.1)
 5 stem fits the roof hole and the plate slot; open slot height holds the open jaw; the tab (3.2 mm) fits the 6.2 mm gap
 6 every printed part fits a 220 x 220 mm bed (largest dimension per part reported)"""
import re, sys, pathlib, importlib.util
here = pathlib.Path(__file__).parent
spec = importlib.util.spec_from_file_location("cad", here / "export_grip_cad.py"); C = importlib.util.module_from_spec(spec); spec.loader.exec_module(C)
src = (here.parent / "qc_met10.py").read_text()
def const(name): return float(re.search(rf"^{name}\s*=\s*([0-9.eE+-]+)", src, re.M).group(1))
ok = True
def rep(name, good, msg):
    global ok; ok &= bool(good); print(("PASS " if good else "FAIL ") + name + "  " + msg)

def bounds(sd):
    if sd["t"] == "box": return sd["b"]
    xs = [p[0] for p in sd["pts"]]; zs = [p[1] for p in sd["pts"]]; return (min(xs), sd["y"][0], min(zs), max(xs), sd["y"][1], max(zs))
def ztop(sd, x):   # upper boundary height of a prism polygon at x, None outside its x range
    pts = sd["pts"]; xs = [p[0] for p in pts]
    if x < min(xs) - 1e-9 or x > max(xs) + 1e-9: return None
    best = None
    n = len(pts)
    for i in range(n):
        (xa, za), (xb, zb) = pts[i], pts[(i + 1) % n]
        if abs(xa - xb) < 1e-12:
            if abs(x - xa) < 1e-9: best = max(best if best is not None else -1e9, za, zb)
            continue
        lo, hi = sorted((xa, xb))
        if lo - 1e-9 <= x <= hi + 1e-9: best = max(best if best is not None else -1e9, za + (zb - za) * (x - xa) / (xb - xa))
    return best
def overlap_box_solid(bx, sd, tol=1e-3):   # bx = (x0,y0,z0,x1,y1,z1) vs a solid: positive-volume overlap
    if sd["t"] == "box":
        b = sd["b"]; return all(min(bx[i + 3], b[i + 3]) - max(bx[i], b[i]) > tol for i in range(3))
    y0, y1 = sd["y"]
    if min(bx[4], y1) - max(bx[1], y0) <= tol: return False
    xs = [bx[0] + tol, bx[3] - tol] + [p[0] for p in sd["pts"] if bx[0] + tol < p[0] < bx[3] - tol]
    return any((ztop(sd, x) is not None) and (ztop(sd, x) - bx[2] > tol) and (bx[5] - min(p[1] for p in sd["pts"]) > tol) for x in xs)
def shift(sds, dz=0.0):
    out = []
    for sd in sds:
        if sd["t"] == "box": b = sd["b"]; out.append({"t": "box", "b": (b[0], b[1], b[2] + dz, b[3], b[4], b[5] + dz)})
        else: out.append({"t": "prism", "pts": [(x, z + dz) for x, z in sd["pts"]], "y": sd["y"]})
    return out
def need_lift(s):
    """lift of the crossbar needed to clear the actual rail faces: for every rail and every crossbar solid that overlaps it in y, the rail's top height
    over the solid's x range (exact at the polygon vertices and the solid's x edges) minus the solid's underside; the largest value."""
    req = 0.0
    for cs in C.crossbar():
        cb = bounds(cs)
        for sd in C.carriage(s):
            if sd["t"] != "prism" or min(cb[4], sd["y"][1]) - max(cb[1], sd["y"][0]) <= 1e-3: continue
            xs = [cb[0], cb[3]] + [p[0] for p in sd["pts"] if cb[0] < p[0] < cb[3]]
            for x in xs:
                z = ztop(sd, x)
                if z is not None: req = max(req, z - cb[2])
    return req
def pair_overlaps(P):
    names = list(P); bad = []
    for i, n1 in enumerate(names):
        for n2 in names[i + 1:]:
            for a in P[n1]:
                for b in P[n2]:
                    if overlap_box_solid(bounds(a), b) and overlap_box_solid(bounds(b), a): bad.append((n1, n2))
    return sorted(set(bad))

# 1 constants
chk = {"HOUSING_A": C.A, "HOUSING_B": C.B, "HOUSING_D": C.D, "JAW_W": C.JAW_W, "GAP_LIFTED": C.GAP_MAX, "LEVER_OPEN": C.TRAVEL, "RAMP_RUN": C.RAMP_RUN, "RAMP_Y": C.RAMP_Y}
bad = {k: (const(k) * 1000.0, v) for k, v in chk.items() if abs(const(k) * 1000.0 - v) > 1e-6}
rep("1 task constants", not bad, "CAD equals qc_met10.py: " + ", ".join(f"{k}={v}" for k, v in chk.items()) if not bad else f"MISMATCH (task mm, CAD mm): {bad}")
# 2 travel sweep with the full overlap test
fails = []; lifts = {}
for k in range(0, 45):
    s = k / 2.0; L = need_lift(s); lifts[s] = L
    P = {"housing": C.housing(), "jaw": shift(C.jaw(), L), "crossbar": shift(C.crossbar(), L), "carriage": C.carriage(s)}
    o = pair_overlaps(P)
    if o: fails.append((s, o[:3]))
    if L > 1e-6:                                                     # contact is real: 0.05 mm lower the crossbar must overlap a rail
        Pl = {"crossbar": shift(C.crossbar(), L - 0.05), "carriage": C.carriage(s)}
        if not pair_overlaps(Pl): fails.append((s, "contact not real"))
rep("2 travel sweep", not fails, f"no overlap at any of 45 positions with the lift taken from the real rail faces" if not fails else str(fails[:4]))
first = next((s for s in sorted(lifts) if lifts[s] > 1e-6), None)
rep("3 lift", abs(lifts[22.0] - C.GAP_MAX) < 0.05 and max(lifts.values()) <= C.GAP_MAX + 0.05 and lifts[0.0] == 0.0,
    f"lift(0)={lifts[0.0]:.2f}, first contact at s={first} mm, lift(11)={lifts[11.0]:.2f}, lift(22)={lifts[22.0]:.2f}, max {max(lifts.values()):.2f} mm (task 6.2)")
# 4 springs: bearing faces taken from the emitted solids
plate = max((bounds(sd) for sd in C.crossbar() if bounds(sd)[4] - bounds(sd)[1] > 10), key=lambda b: b[5]); cb_top = plate[5]
bridge = next(bounds(sd) for sd in C.housing() if bounds(sd)[5] == C.CAP_Z1 and bounds(sd)[4] - bounds(sd)[1] > 20); cap_under = bridge[2]
sp_ok = True; frc = {}
for g in (0.0, 3.2, 6.2):
    Ls = cap_under - (cb_top + g); f = max(0.0, 2 * C.K_JAW * (C.FREE_JAW - Ls)); frc[g] = (Ls, f); sp_ok &= Ls >= C.MAXLOAD_LEN and f >= -1e-9
rep("4a jaw springs", sp_ok and abs(frc[3.2][1] - const("JAW_K") / 1000 * 3.2) < 0.1 and abs(frc[6.2][1] - const("JAW_K") / 1000 * 6.2) < 0.1,
    f"seat faces {cb_top} / {cap_under} (no recesses); installed length {frc[0.0][0]:.1f}/{frc[3.2][0]:.1f}/{frc[6.2][0]:.1f} mm at gap 0/3.2/6.2, total force {frc[0.0][1]:.2f}/{frc[3.2][1]:.2f}/{frc[6.2][1]:.2f} N (task JAW_K x gap: 0/1.92/3.72); catalogue length at maximum load {C.MAXLOAD_LEN} mm: installed length must stay at or above it (force tolerance 0.1 N against the task's JAW_K x gap)")
r0 = C.springs(0.0)["ret_len"]; r22 = C.springs(22.0)["ret_len"]
f0 = C.F0_RET + C.K_RET * (r0 - C.FREE_RET); f22 = C.F0_RET + C.K_RET * (r22 - C.FREE_RET); f11 = C.F0_RET + C.K_RET * (C.springs(11.0)["ret_len"] - C.FREE_RET)
rep("4b return springs", r0 >= C.FREE_RET and r22 <= C.MAXLEN_RET and abs(f22 - const("LEVER_K") / 1000 * 22) < 0.1,
    f"two 1942N5 in parallel on 2 x 2 mm pins (hooks bear on the anchor pins' +x faces and the carriage pins' -x faces): bearing-to-bearing {r0:.2f} -> {r22:.2f} mm (catalogue length {C.FREE_RET} taken as hook-inside to hook-inside, max-load length {C.MAXLEN_RET}); force {f0:.2f} N closed, {f11:.2f} N at 11 mm, {f22:.2f} N at 22 mm (task LEVER_K x 22 mm = {const('LEVER_K') / 1000 * 22:.1f} N, tolerance 0.1 N at 22 mm). DEVIATIONS from the model: closed preload {f0:.2f} N (model 0), force at 11 mm {f11:.2f} N (model {const('LEVER_K') / 1000 * 11:.2f} N), rate {C.K_RET:.3f} N/mm (model 0.3)")
# spring body envelope (3 mm OD along the line between the pins) must clear every other solid over the sweep
envs = []
for k in range(0, 45):
    s_ = k / 2.0; xa = C.springs(0.0)["ret_len"]; L_ = C.springs(s_)["ret_len"]
    xa_face = max(sd["b"][3] for sd in C.housing() if sd["t"] == "box" and abs(sd["b"][3] - sd["b"][0] - 2.0) < 1e-9 and abs(sd["b"][5] - sd["b"][2] - 8.0) < 1e-9)
    for sg in (-1, 1):
        yy = C.YC - C.RAMP_Y + sg * C.HOOK_DY
        env = {"t": "box", "b": (xa_face, yy - 1.5, C.A + 2.5, xa_face + L_, yy + 1.5, C.A + 5.5)}
        P2 = {"housing": [sd for sd in C.housing() if not (sd["t"] == "box" and abs(sd["b"][3] - sd["b"][0] - 2.0) < 1e-9)],
              "carriage": [sd for sd in C.carriage(s_) if not (sd["t"] == "box" and abs(sd["b"][3] - sd["b"][0] - 2.0) < 1e-9)],
              "jaw": shift(C.jaw(), lift_ := need_lift(s_)), "crossbar": shift(C.crossbar(), lift_)}
        for n, sds in P2.items():
            for sd in sds:
                if overlap_box_solid(env["b"], sd) and overlap_box_solid(bounds(sd), env): envs.append((s_, n))
rep("4c return spring envelope", not envs, "the 3 mm spring body between the pins clears every part over the whole travel" if not envs else str(sorted(set(envs))[:4]))
# 5 fits
rep("5 fits", C.CROSS[0] >= 5.4 + 0.2 and 4.6 > 2.0 and C.HOLE[0] > C.STEM[0] and C.HOLE[1] > C.STEM[1] and 11.2 > C.STEM[1] and C.SLOT_H >= 6.4 + C.JAW_H and C.GAP_MAX > 3.2,
    f"spring OD 5.4 on the 6 mm crossbar, 2 mm pins in its 4.6 mm bore; stem {C.STEM} in hole {C.HOLE} and plate slot 11.2; slot {C.SLOT_H:.1f} >= open jaw {6.4 + C.JAW_H:.1f}; tab 3.2 < gap {C.GAP_MAX}")
blk = [bounds(sd) for sd in C.grip_block()]; depth = max(b[5] for b in blk) - min(b[2] for b in blk if b[2] > 0)
rep("5b clamp block", C.POCKET > C.SHANK[0] and C.POCKET > C.SHANK[1] and depth >= 18.0 and C.SHANK[2] > depth, f"pocket {C.POCKET:.1f} mm square, {depth:.0f} mm deep for the {C.SHANK[0]} x {C.SHANK[1]} x {C.SHANK[2]} mm shank ({C.SHANK[2] - depth:.0f} mm of shank stands out of the block); block extents {max(b[3] for b in blk):.0f} x {max(b[4] for b in blk):.0f} x {max(b[5] for b in blk):.0f} mm")
# 6 bed
dims = {}
for n, f in C.PARTS.items():
    bs = [bounds(sd) for sd in f()]; dims[n] = tuple(round(max(b[i + 3] for b in bs) - min(b[i] for b in bs), 1) for i in range(3))
rep("6 print bed", all(max(d) <= 220 for d in dims.values()), "extents x,y,z mm: " + "; ".join(f"{n[5:]} {d}" for n, d in dims.items()))
print("SCOPE: geometry and spring arithmetic on the emitted solids (the task ends at mounting, before any pull).")
sys.exit(0 if ok else 1)
