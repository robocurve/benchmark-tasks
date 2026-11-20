"""Reach scan for the drum pads (kinematics only: yam_arena's inverse kinematics on the YAM's URDF, no Isaac Sim).

For a pad at (x, y, head height z, tilt toward the robot) struck by one arm, a stick held across the gripper's fingers
at its grip point (the TCP), the scan asks:
  1. can the tip sphere rest on the head's centre with the stick pitched nose-down (to the table) by p, for some
     stick heading, in one of the grips (H: gripper pointing down, fingers closing sideways across the stick, the
     stick held nose-down by DELTA in the grip; V: gripper pointing sideways, fingers closing up and down on it)?
  2. from that pose, can the stick pivot about its grip point to lift the tip 15, 30, 60 and 100 mm?
  3. how fast is the stroke joint (H: joint 4, wrist pitch; V: joint 6, wrist roll) at moving the tip, in which
     direction relative to the pad's normal, and how far can that joint alone lift the tip before its limit?
  4. are the hand's collision hulls (link 5, gripper, fingers) and the stick clear of every pad?

Run on the GPU machine (needs numpy and yam_arena):
    python tools/reach_scan.py grid  [--jobs 28] > out/reach/grid.json
    python tools/reach_scan.py layout --name O                    # one layout, both arms, all pads
"""

import argparse
import itertools
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_drum_groove import layout as L  # noqa: E402

# The layouts compared before choosing (pad centre (x, y), height z, tilt toward the robot in degrees). P is the
# package's (layout.PAD_SPECS); S was the prototype's own pick; F is the fallback with the snare between the arms; O is
# an earlier open-handed draft without a tom.
LAYOUTS = {
    "P": dict(L.PAD_SPECS),
    "S": {"hihat": ((.500, .140), .140, 0.), "snare": ((.500, -.180), .120, 5.), "tom": ((.460, -.460), .110, 5.)},
    "F": {"hihat": ((.480, .320), .130, 0.), "snare": ((.450, 0.), .090, 5.), "tom": ((.450, -.290), .080, 5.)},
    "O": {"hihat": ((.500, .190), .135, 0.), "snare": ((.500, -.180), .120, 5.), "tom": ((.420, -.440), .110, 5.)},
}

LIFTS = (.015, .030, .060, .100)
MARGIN_DEG = 3.                  # joints at least this far from their limits
POS_TOL, ROT_TOL = .001, math.radians(1.)
CLEAR_MIN = .010                 # hand hulls at least this far from every pad; stick shaft from the other pads


def _stl_vertices(path):
    data = Path(path).read_bytes()
    count = int.from_bytes(data[80:84], "little")
    record = np.dtype([("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attr", "<u2")])
    triangles = np.frombuffer(data[84:84 + record.itemsize * count], dtype=record)
    return np.unique(triangles["vertices"].reshape(-1, 3).astype(float), axis=0)


class HandHulls:
    """The YAM's collision hulls near the hand (link 5, gripper, fingers) from its URDF, posed by forward kinematics
    (the spark-plug and ukulele packages' dry run)."""

    LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")

    def __init__(self):
        from yam_arena import YAM_URDF
        from yam_arena.urdf import rpy_matrix

        self.urdf = Path(str(YAM_URDF))
        self.hulls = []
        for link in ET.parse(self.urdf).getroot().iter("link"):
            if link.get("name") not in self.LINKS:
                continue
            for collision in link.findall("collision"):
                origin = collision.find("origin")
                xyz = np.array([float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()])
                rpy = np.array([float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()])
                pts = _stl_vertices(self.urdf.parent / collision.find("geometry/mesh").get("filename"))
                self.hulls.append((link.get("name"), pts @ rpy_matrix(rpy).T + xyz))

    def points(self, arm_joints, base, finger=0.):
        from yam_arena.constants import ARM_JOINTS
        from yam_arena.urdf import forward_kinematics

        q = {n: float(v) for n, v in zip(ARM_JOINTS, arm_joints)}
        poses = forward_kinematics(self.urdf, {**q, "joint7": float(finger), "joint8": float(finger)})
        return np.concatenate([v @ poses[k][:3, :3].T + poses[k][:3, 3] for k, v in self.hulls]) + base


def bases():
    from yam_arena import station

    top = float(station.TABLE_TOP)
    return {arm: np.array(station.BASES[arm], dtype=float) - (0., 0., top) for arm in ("left", "right")}


def pad_geometry(kind, centre_xy, z, tilt_deg):
    t = math.radians(tilt_deg)
    n = np.array((-math.sin(t), 0., math.cos(t)))
    c = np.array((centre_xy[0], centre_xy[1], z))
    head_r = {"snare": L.SNARE_HEAD_R, "tom": L.TOM_HEAD_R, "hihat": L.HIHAT_R}[kind]
    outer = head_r + (L.RIM_W if kind != "hihat" else 0.)
    top = L.RIM_H if kind != "hihat" else 0.
    return {"kind": kind, "c": c, "n": n, "outer_r": outer, "top": top, "bottom": -L.SHELL_DEPTH}


def cylinder_gap(points, pad):
    """Signed distance of points to the pad's cylinder (shell + rim), negative inside."""
    d = points - pad["c"]
    a = d @ pad["n"]
    radial = np.linalg.norm(d - np.outer(a, pad["n"]), axis=1)
    dr = radial - pad["outer_r"]
    da = np.maximum(pad["bottom"] - a, a - pad["top"])
    outside = np.sqrt(np.maximum(dr, 0.) ** 2 + np.maximum(da, 0.) ** 2)
    inside = np.maximum(dr, da)
    return np.where((dr < 0) & (da < 0), inside, outside)


def grip_rotation(s, gamma, delta, flip=False):
    """Gripper rotation (columns x_g closing, y_g, z_g approach) holding the stick along ``s``: fingers closing along
    c = cos(gamma) h + sin(gamma) u (h horizontal across the stick, u up across it), stick tilted by ``delta`` in the
    pad plane toward the approach axis."""
    s = s / np.linalg.norm(s)
    h = np.cross((0., 0., 1.), s)
    h /= np.linalg.norm(h)
    u = np.cross(s, h)
    c = math.cos(gamma) * h + math.sin(gamma) * u
    zs = np.cross(c, s)
    yg = math.cos(delta) * s - math.sin(delta) * zs
    zg = math.sin(delta) * s + math.cos(delta) * zs
    R = np.column_stack((c, yg, zg))
    # flipped: the same grip turned 180 degrees about the approach axis (the stick along -y_g), which the fingers'
    # symmetry allows; joint 6 can reach one or the other (yam_arena/demo.py, DOWN_GRIP)
    return R @ np.diag((-1., -1., 1.)) if flip else R


def stick_dir(heading, pitch):
    return np.array((math.cos(pitch) * math.cos(heading), math.cos(pitch) * math.sin(heading), -math.sin(pitch)))


def rot_axis(axis, angle):
    a = axis / np.linalg.norm(axis)
    K = np.array(((0., -a[2], a[1]), (a[2], 0., -a[0]), (-a[1], a[0], 0.)))
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


SEEDS = [np.array(s, dtype=float) for s in (
    (0, 1, 1, -.6, 0, 0), (0, 1.6, 1.3, -1.2, 0, 0), (0, 2.1, 1.0, -.4, 1.5, -.1), (0, 2.1, 1.0, -.4, -1.5, .1),
    (.35, 2.2, .8, 1.4, -1.2, 0), (-.35, 2.2, .8, 1.4, 1.2, -1.57), (.4, 2.5, 1.4, -1.6, 1.1, 0),
    (0, 1.2, 1.6, -1.0, 0, 1.57), (0, 1.2, 1.6, -1.0, 0, -1.57), (0, 2.4, 2.0, -1.4, 0, 0))]


def solve_pose(point, R, base, seeds, iters=200):
    from yam_arena import ik

    quat = ik.matrix_to_quat_xyzw(R)
    best = None
    for seed in seeds[:7]:
        q, err, rerr = ik.solve(point, quat, seed, base, iters=iters)
        margin = math.degrees(float(np.minimum(q - ik.LOWER, ik.UPPER - q).min()))
        ok = err < POS_TOL and rerr < ROT_TOL and margin >= MARGIN_DEG
        score = (ok, margin if ok else -err)
        if best is None or score > best[0]:
            best = (score, q, err, rerr, margin)
        if ok and margin > 15:
            break
    (_, q, err, rerr, margin) = best
    return q, err, rerr, margin


def tip_from(q, base, R_offset_s):
    """The tip sphere's centre for joints q (stick along s_g in the gripper frame)."""
    from yam_arena import ik

    p, R = ik.tcp_pose(q, base)
    return p + R @ R_offset_s * (L.TIP_OUT - L.TIP_R)


def strike(pad, arm, base, grip, heading, pitch, *, hulls=None, others=(), lifts=LIFTS):
    """One candidate strike pose and its lifts. ``grip`` = (name, gamma, delta)."""
    from yam_arena import ik

    name, gamma, delta, flip = grip
    s = stick_dir(heading, pitch)
    R = grip_rotation(s, gamma, delta, flip)
    s_g = R.T @ s                                      # the stick in the gripper frame (fixed by the grip)
    q_tip = pad["c"] + L.TIP_R * pad["n"]               # the tip sphere's centre touching the head's centre
    tcp = q_tip - (L.TIP_OUT - L.TIP_R) * s
    q, err, rerr, margin = solve_pose(tcp, R, base, SEEDS)
    out = {"grip": name, "heading_deg": round(math.degrees(heading), 1), "pitch_deg": round(math.degrees(pitch), 1),
           "angle_to_head_deg": round(math.degrees(math.asin(max(-1., min(1., -float(s @ pad["n"]))))), 1),
           "tcp_mm": [round(1000 * v, 1) for v in tcp], "ok": False, "err_mm": round(1000 * err, 3),
           "margin_deg": round(margin, 1), "q": [round(float(v), 4) for v in q]}
    if not (err < POS_TOL and rerr < ROT_TOL and margin >= MARGIN_DEG):
        return out
    # the stroke joint: tip velocity per rad of joint 4 (H) or joint 6 (V), its direction against the normal
    j = 3 if name.startswith("H") else 5
    eps = 1e-4
    dq = q.copy()
    dq[j] += eps
    v = (tip_from(dq, base, s_g) - tip_from(q, base, s_g)) / eps
    lever = float(np.linalg.norm(v))
    up = 1. if v @ pad["n"] > 0 else -1.                  # the sense of joint j that lifts the tip
    room = (ik.UPPER[j] - q[j]) if up > 0 else (q[j] - ik.LOWER[j])
    out.update(ok=True, stroke_joint=j + 1, stroke_lever_mm=round(1000 * lever, 1),
               stroke_off_normal_deg=round(math.degrees(math.acos(min(1., abs(v @ pad["n"]) / lever))), 1),
               stroke_joint_room_deg=round(math.degrees(room), 1),
               stroke_joint_lift_mm=round(1000 * lever * min(room, math.radians(40.)), 1))
    # lifts by pivoting the stick about its grip point (the tip goes up along an arc)
    h = np.cross((0., 0., 1.), s)
    h /= np.linalg.norm(h)
    lift_rows = {}
    seeds = [q] + SEEDS
    for dz in lifts:
        lo, hi = 0., math.radians(70.)
        for _ in range(40):                              # pivot angle whose tip gain is dz
            mid = (lo + hi) / 2
            s2 = rot_axis(h, -mid) @ s
            gain = ((tcp + (L.TIP_OUT - L.TIP_R) * s2) - q_tip) @ pad["n"]
            lo, hi = (mid, hi) if gain < dz else (lo, mid)
        phi = (lo + hi) / 2
        R2 = rot_axis(h, -phi) @ R
        q2, e2, r2, m2 = solve_pose(tcp, R2, base, seeds)
        lift_rows[f"{round(1000 * dz)}mm"] = {"ok": bool(e2 < POS_TOL and r2 < ROT_TOL and m2 >= MARGIN_DEG),
                                             "pivot_deg": round(math.degrees(phi), 1), "margin_deg": round(m2, 1),
                                             "dq_deg": [round(math.degrees(float(a - b)), 1) for a, b in zip(q2, q)]}
    out["lifts"] = lift_rows
    out["lifts_ok"] = all(r["ok"] for r in lift_rows.values())
    if hulls is not None:
        pts = hulls.points(q, base)
        out["hand_clear_mm"] = {p["kind"]: round(1000 * float(cylinder_gap(pts, p).min()), 1) for p in (pad, *others)}
        shaft = np.array([tcp + t * s for t in np.linspace(-L.GRIP_FROM_BUTT, L.TIP_OUT - 2 * L.TIP_R, 60)])
        out["stick_clear_mm"] = {p["kind"]: round(1000 * (float(cylinder_gap(shaft, p).min()) - L.STICK_R), 1)
                                 for p in others}
        # the stick shaft over the struck pad's own rim (the last 25 mm before the tip may be over the head)
        near = np.array([tcp + t * s for t in np.linspace(-L.GRIP_FROM_BUTT, L.TIP_OUT - .140, 40)])
        out["stick_over_own_rim_mm"] = round(1000 * (float(cylinder_gap(near, pad).min()) - L.STICK_R), 1)
    return out


GRIPS = {}
for _f, _tag in ((False, ""), (True, "f")):
    GRIPS.update({"H0" + _tag: (0., 0., _f), "H10" + _tag: (0., math.radians(10.), _f),
                  "V+" + _tag: (math.pi / 2, 0., _f), "V-" + _tag: (-math.pi / 2, 0., _f),
                  "D45+" + _tag: (math.pi / 4, 0., _f), "D45-" + _tag: (-math.pi / 4, 0., _f)})


_HULLS = None


def _strike_job(job):
    global _HULLS
    pad, arm, base, grip, heading, pitch, with_hulls, others, need_lifts = job
    if with_hulls and _HULLS is None:
        _HULLS = HandHulls()
    return strike(pad, arm, base, grip, heading, pitch, hulls=_HULLS if with_hulls else None, others=others,
                  lifts=LIFTS if need_lifts else ())


POOL = None


def best_strike(pad, arm, base, *, grips=GRIPS, pitches=(8., 12., 18., 24.), headings=None, hulls=None, others=(),
                need_lifts=True):
    if headings is None:
        natural = math.atan2(pad["c"][1] - base[1], pad["c"][0] - base[0])
        headings = [natural + math.radians(d) for d in (0., -15., 15., -30., 30., -45., 45.)]
        if abs(natural) > math.radians(20):            # also straight ahead
            headings.append(0.)
    jobs = [(pad, arm, base, (g, *grips[g]), h, math.radians(p), hulls is not None, others, need_lifts)
            for g in grips for h, p in itertools.product(headings, pitches)]
    results = POOL.map(_strike_job, jobs) if POOL is not None else [_strike_job(j) for j in jobs]
    rows = []
    for gname in grips:
        best = None
        for r in (r for r in results if r["grip"] == gname):
            key = (r["ok"], r.get("lifts_ok", False),
                   min(r.get("hand_clear_mm", {"x": 99}).values()) >= 1000 * CLEAR_MIN if hulls else True,
                   min(r.get("stick_clear_mm", {"x": 99}).values() or [99]) >= 1000 * CLEAR_MIN if hulls else True,
                   r["margin_deg"] - .3 * r.get("stroke_off_normal_deg", 90))
            if best is None or key > best[0]:
                best = (key, r)
        rows.append(best[1])
    return rows


def grid_point(args):
    kind, arm, x, y, z, tilt = args
    base = bases()[arm]
    pad = pad_geometry(kind, (x, y), z, tilt)
    rows = best_strike(pad, arm, base, grips={k: GRIPS[k] for k in ("H10f", "V+", "V-")}, pitches=(10., 18.))
    return {"kind": kind, "arm": arm, "x": x, "y": y, "z": z,
            "grips": {r["grip"]: {"ok": r["ok"] and r.get("lifts_ok", False), "margin": r["margin_deg"],
                                  "off_normal": r.get("stroke_off_normal_deg"), "lever": r.get("stroke_lever_mm"),
                                  "heading": r["heading_deg"], "pitch": r["pitch_deg"],
                                  "joint_lift_mm": r.get("stroke_joint_lift_mm")} for r in rows}}


def run_grid(jobs):
    from multiprocessing import Pool

    xs = [round(v, 3) for v in np.arange(.30, .5751, .05)]
    work = []
    for kind, arm, ys, zs, tilt in (("hihat", "left", np.arange(.10, .4251, .05), np.arange(.08, .1801, .02), 0.),
                                    ("snare", "right", np.arange(-.10, -.3751, -.05), np.arange(.06, .1601, .02), 5.),
                                    ("tom", "right", np.arange(-.30, -.6251, -.05), np.arange(.06, .1401, .02), 5.)):
        for x, y, z in itertools.product(xs, ys, zs):
            work.append((kind, arm, float(x), round(float(y), 3), round(float(z), 3), tilt))
    with Pool(jobs) as pool:
        return pool.map(grid_point, work, chunksize=4)


def run_layout(name, spec=None, verbose=True):
    spec = spec or LAYOUTS[name]
    base = bases()
    hulls = True
    pads = {k: pad_geometry(k, *spec[k]) for k in ("hihat", "snare", "tom")}
    arms = {"hihat": "left", "snare": "right", "tom": "right"}
    report = {"layout": name, "pads": {k: {"centre_mm": [round(1000 * v, 1) for v in p["c"]], "tilt_deg": spec[k][2]}
                                       for k, p in pads.items()}, "strikes": {}}
    for kind, pad in pads.items():
        others = [p for k, p in pads.items() if k != kind]
        rows = best_strike(pad, arms[kind], base[arms[kind]], hulls=hulls, others=others)
        report["strikes"][kind] = rows
    # plan-view gaps between the pads
    def gap(a, b):
        return float(np.linalg.norm(pads[a]["c"][:2] - pads[b]["c"][:2]) - pads[a]["outer_r"] - pads[b]["outer_r"])
    report["plan_gap_mm"] = {"hihat-snare": round(1000 * gap("hihat", "snare"), 1),
                             "snare-tom": round(1000 * gap("snare", "tom"), 1)}
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["grid", "layout"])
    parser.add_argument("--name", default="P", choices=sorted(LAYOUTS))
    parser.add_argument("--jobs", type=int, default=28)
    parser.add_argument("--spec", help="JSON layout spec overriding --name")
    args = parser.parse_args()
    global POOL
    from multiprocessing import Pool

    if args.mode == "grid":
        print(json.dumps(run_grid(args.jobs)))
        return
    POOL = Pool(args.jobs)
    if False:
        pass
    else:
        spec = json.loads(args.spec) if args.spec else None
        if spec:
            spec = {k: (tuple(v[0]), v[1], v[2]) for k, v in spec.items()}
        print(json.dumps(run_layout(args.name, spec), indent=1))


if __name__ == "__main__":
    main()
