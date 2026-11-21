import json, math, os, sys
from pathlib import Path
import numpy as np
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent)); sys.path.insert(0, str(HERE.parent / "tools")); sys.path.insert(0, str(HERE))
import guard_scan as G, ik_scan as IS, build_assets as BA
from rdb_sling_guards import fixed_scene as F, layout as L, scripted as S
os.environ.setdefault("HOOK_DZ", "0")
G.init()
for dy, dz, psi in [(.016, .016, 65), (.020, .016, 65), (.024, .017, 65), (.024, .020, 60), (.028, .020, 65)]:
    seat, Rs = S.edge_point("BL"), S.seat_frame("BL")
    n2, up = Rs[:, 1], np.array((0., 0., 1.))
    a = -(math.cos(math.radians(psi)) * n2 + math.sin(math.radians(psi)) * up)
    R = S.grip_frame(a, Rs[:, 0])
    p = seat + dy * n2 + dz * up
    lp, lR = Rs.T @ (p - seat), Rs.T @ R
    pt, Rt = BA.tray_pose("guard_2"); pt = np.asarray(pt) - (0., 0., F.TOP)
    hold = -(L.GUARD_LEN / 2) - .0005
    out = []
    r, q = G.check("left", pt + Rt @ lp, Rt @ lR, hold); out.append(("tray", r))
    low = np.array((0., 0., -.005))
    for nm, cp in (("out45", seat + .045 * n2 + low), ("out10", seat + .010 * n2 + low), ("under", seat + .002 * n2 + low), ("rel", seat + .002 * n2 - .002 * up)):
        r, q2 = G.check("left", cp + Rs @ lp, R, hold, q); out.append((nm, r)); q = q2 if q2 is not None else q
    print(dy, dz, psi, [(n, r.get("margin_deg"), r.get("fingers_scene_mm"), r.get("body_scene_mm"), r.get("hook_mm")) for n, r in out])
