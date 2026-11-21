"""grippers.py [cli=N | seed=N | corners | stems | cover=<mm>]: the arms' clearance along the demo, from the YAM's own
collision meshes, every link.

Each arm follows its plan through the same IK the scripted demo uses (lancet_fingerstick.TiltPlan, yam_arena's IK); at
each check the convex collision meshes of link1 to link5, the gripper and both finger tips (yam_arm.urdf, the URDF
yam_arena loads) are posed by forward kinematics and sampled on their faces every 1.5 mm, and the lancet's housing is
added to the arm that holds it. Checked every 0.1 s. Reported, each against its floor:
- arms: the least distance between the two arms (and the lancet);
- trainer: the least distance from each arm to the trainer: its fixed parts (palm, thumb, index finger, the palm
  block's post, the base board) and the ring finger's moving parts. Not counted, by design: the holding pads on the
  ring finger itself, and the shield's face on the site while the lancet touches it;
- bench: from each arm (and the lancet) to the stand and the sharps container, less the lancet in its cup while the
  right takes it;
- table: from each arm to the table top.
Where contact stops an arm (the press past the site, the hold lifting the finger on its stop), the arm is posed where
it stops (the touch, the grip), not at the commanded target. No sim, no dynamics: the arms' lag on a descent (2 to 10
mm, README) is not in it.

    python3 tools/grippers.py            # --seed 0, 2, 4, the layout corners and the stems
"""
import dataclasses
import functools
import math
import multiprocessing as mp
import pathlib
import sys
import warnings
import xml.etree.ElementTree as ET
import zlib

warnings.simplefilter("ignore")
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
from scipy.spatial import ConvexHull, cKDTree  # noqa: E402
import lancet_fingerstick as F  # noqa: E402
from yam_arena import YAM_URDF, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS  # noqa: E402
from yam_arena.urdf import forward_kinematics, rpy_matrix  # noqa: E402

DT_CHECK = 0.1                   # s between checks [CHOICE; the TCPs move under 10 mm in it, EST]
LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
TIPS = ("tip_left", "tip_right")
STEP = 0.0015
ARM_FLOOR = 0.010                # m: the least arm-to-arm distance this check passes [CHOICE: over the 2 to 10 mm lag]
HAND_FLOOR = 0.003               # m: the least distance from an arm to the trainer [CHOICE]
BENCH_FLOOR = 0.003              # m: to the stand and the container [CHOICE]
TABLE_FLOOR = 0.003              # m: to the table top [CHOICE]
SHELL = 0.003                    # m: the 3 mm polycarbonate shells on the left arm (README Rig protection) [CHOICE]
COVERS = {"left": {k: (SHELL if k in ("gripper", "tip_left", "tip_right", "link5") else 0.0) for k in LINKS},
          "right": {k: 0.0 for k in LINKS}}
#                                  WHY: a cover is solid the meshes do not have, so each of its link's points counts as
#                                  that much nearer. The left, beside the press, is shelled; the right holds the lancet
#                                  face first, ahead of its own parts, and is not (README Rig protection). `cover=<mm>`
#                                  on the command line puts that thickness on every link of both arms instead
HELD = ("grip the housing", "let go")   # the right holds the lancet from the first label up to the second
AT_STAND = {"down to the lancet", "grip the housing", "settle", "twist the tab off", "hold",
            "turn it back in the stand", "lift it out of the stand",
            "press it down in the stand", "back up in the stand"}
ON_SITE = {"tip on the site", "load the press", "settle the press", "press until it fires", "hold the press",
           "back off along the axis"}
PRESSED = {"load the press": "tip on the site", "settle the press": "tip on the site",
           "press until it fires": "tip on the site", "hold the press": "tip on the site",
           "press it down in the stand": "hold"}
LIFTED = {"hold it up on its stop": "grip the ring finger", "keep holding through the press": "grip the ring finger"}
HOLDING = {"grip the ring finger", "hold it up on its stop", "keep holding through the press", "ease off",
           "let go of the ring finger"}


def _stl(path):
    b = path.read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:300]:
        v = [list(map(float, ln.split()[1:4])) for ln in b.decode().splitlines() if ln.strip().startswith("vertex")]
        return np.array(v, float).reshape(-1, 3, 3)
    n = int.from_bytes(b[80:84], "little")
    a = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(float)


def _sample(tri):
    """Points on each triangle's face, about STEP apart."""
    out = [tri.reshape(-1, 3)]
    for t in tri:
        a, b, c = t
        n = int(max(np.linalg.norm(b - a), np.linalg.norm(c - a)) / STEP) + 1
        if n < 2:
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1))
        m = (i + j) <= n
        u, v = i[m] / n, j[m] / n
        out.append(a + u[:, None] * (b - a) + v[:, None] * (c - a))
    return np.vstack(out)


@functools.lru_cache(maxsize=None)
def link_points(link):
    """A link's collision meshes, sampled, in its own frame."""
    root = ET.parse(YAM_URDF).getroot()
    base = pathlib.Path(YAM_URDF).parent
    pts = []
    for ln in root.iter("link"):
        if ln.get("name") != link:
            continue
        for col in ln.iter("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in o.get("xyz").split()])
            rpy = np.array([float(v) for v in o.get("rpy").split()])
            p = _sample(_stl(base / col.find("geometry/mesh").get("filename")))
            pts.append(p @ rpy_matrix(rpy).T + xyz)
    p = np.vstack(pts)
    # WHY: the meshes are dense; one point per 1.5 mm cell keeps the distances to within about 1 mm
    _, keep = np.unique(np.floor(p / STEP).astype(np.int64), axis=0, return_index=True)
    return p[np.sort(keep)]


def arm_points(side, q6, finger, links=LINKS):
    """The arm's sampled collision points in the world for joints q6 and finger opening `finger` (m, <= 0), and each
    point's cover thickness (m)."""
    q = {n: float(v) for n, v in zip(ARM_JOINTS, q6)}
    q["joint7"] = q["joint8"] = float(finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.asarray(station.BASES[side], float)
    pts = np.vstack([link_points(k) @ poses[k][:3, :3].T + poses[k][:3, 3] + base for k in links])
    cov = np.concatenate([np.full(len(link_points(k)), COVERS[side][k]) for k in links])
    return pts, cov


def lancet_points(tcp, rot, skip_face=False):
    """The lancet's housing held in a gripper (its face tcp_over_lancet() past the TCP along the approach): points on
    its 13 x 10 mm sides from 3 mm behind its face to its rear, and on its rear. With skip_face, none within 6 mm of
    the face (the shield and the housing's front, on the skin by design)."""
    a, gx, gy = rot[:, 2], rot[:, 0], rot[:, 1]
    face = np.asarray(tcp, float) + F.tcp_over_lancet() * a
    out = []
    for t in np.arange(F.SHIELD_OUT + (0.003 if skip_face else 0.0), F.SHIELD_OUT + F.L_LEN + 1e-9, STEP):
        for i in np.linspace(-1, 1, 9):
            for j in np.linspace(-1, 1, 7):
                if abs(i) == 1 or abs(j) == 1 or t >= F.SHIELD_OUT + F.L_LEN - 1e-9:
                    out.append(face - t * a + i * F.L_W / 2 * gy + j * F.L_T / 2 * gx)
    return np.array(out), np.zeros(len(out))


def shape_points(shape):
    """Points whose convex hull is the shape (mechanism frame): a box's corners, a capsule's or cylinder's end rings
    (with a capsule's caps), a hull's own points."""
    if shape.kind == "hull":
        return np.asarray(shape.points, float)
    lo, hi = (np.asarray(v, float) for v in shape.bounds())
    if shape.kind == "box":
        return np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    r, length = shape.size
    ax = "XYZ".index(shape.axis)
    c = (lo + hi) / 2
    half = length / 2
    out = []
    for e in (-1, 1):
        for th in np.linspace(0, 2 * math.pi, 24, endpoint=False):
            for k, rr in ((0.0, r),) + (((0.38, r * 0.92), (0.7, r * 0.71), (0.92, r * 0.38), (1.0, 0.0))
                                        if shape.kind == "capsule" else ()):
                p = np.array(c, float)
                p[ax] += e * (half + k * r * (shape.kind == "capsule"))
                o = [i for i in range(3) if i != ax]
                p[o[0]] += rr * math.cos(th)
                p[o[1]] += rr * math.sin(th)
                out.append(p)
    return np.array(out)


def _hulls(shapes, R, o):
    return [ConvexHull(shape_points(s) @ R.T + o) for s in shapes]


def trainer_hulls(seed):
    """The trainer (world), each part a convex hull: (fixed parts, the ring finger itself, the ring's slide parts that
    move with it)."""
    hx, hy, hz, yaw = F.layout(seed)["hand"]
    c, s = math.cos(yaw), math.sin(yaw)
    R, o = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]), np.array([hx, hy, hz])
    fixed = list(F.HAND_SHAPES) + list(F.support_shapes()) + list(F.slide_fixed())
    return _hulls(fixed, R, o), _hulls(F.RING_SHAPES, R, o), _hulls(F.ring_moving(), R, o)


def bench_hulls():
    """The stand (with its cup) and the sharps container's walls and lid, world, each part a hull."""
    base, links, _ = F.bench_parts()
    o = np.array([0.0, 0.0, F.TOP])
    return [ConvexHull(shape_points(s) + o) for s in base.shapes + links[0].shapes]


def _hull_gap(hull, pc, near=0.03):
    """The least signed distance from covered points (points, covers) to a convex hull (negative inside), from its
    face planes, less each point's cover; points more than `near` outside the hull's box are skipped."""
    pts, cov = pc
    lo, hi = hull.min_bound - near, hull.max_bound + near
    k = np.all((pts >= lo) & (pts <= hi), axis=1)
    pts, cov = pts[k], cov[k]
    if len(pts) == 0:
        return near
    d = pts @ hull.equations[:, :3].T + hull.equations[:, 3]
    return float((d.max(axis=1) - cov).min())


def _gap(hulls, pc):
    return min((_hull_gap(h, pc) for h in hulls), default=0.03)


def _arm_gap(ac, bc, near=0.03):
    """The least distance between two covered point sets, less both points' covers; WHY: only the points inside each
    other's box (plus `near` and the thickest cover) can be under `near` apart, so the rest are skipped."""
    (a, ca), (b, cb) = ac, bc
    pad = near + 2 * max(ca.max(initial=0.0), cb.max(initial=0.0))
    ka = np.all((a >= b.min(0) - pad) & (a <= b.max(0) + pad), axis=1)
    a, ca = a[ka], ca[ka]
    if len(a) == 0:
        return near
    kb = np.all((b >= a.min(0) - pad) & (b <= a.max(0) + pad), axis=1)
    b, cb = b[kb], cb[kb]
    if len(b) == 0:
        return near
    # WHY k nearest: with covers, the nearest raw point is not always the nearest covered one
    d, i = cKDTree(a).query(b, k=min(8, len(a)))
    d, i = d.reshape(len(b), -1), i.reshape(len(b), -1)
    return float((d - ca[i] - cb[:, None]).min())


def _join(*pcs):
    return np.vstack([p for p, _ in pcs]), np.concatenate([c for _, c in pcs])


def stopped(ws, stops):
    """The plan with each waypoint labelled in `stops` moved to the pose of the waypoint it names (where contact stops
    the arm)."""
    at = {w.label: w for w in ws}
    return [dataclasses.replace(w, pos=at[stops[w.label]].pos) if w.label in stops else w for w in ws]


class _Worst:
    def __init__(self):
        self.v = {k: (9.0, "") for k in ("arms", "trainer", "bench", "table")}

    def add(self, key, d, what):
        if d < self.v[key][0]:
            self.v[key] = (d, what)


FLOORS = {"arms": ARM_FLOOR, "trainer": HAND_FLOOR, "bench": BENCH_FLOOR, "table": TABLE_FLOOR}


def check(job):
    """One layout (and plan): the least distance for each of arms, trainer, bench and table, and when."""
    seed, name, plans = job
    R, L = F.plan(0.0, seed) if plans is None else plans
    R, L = stopped(R, PRESSED), stopped(L, LIFTED)
    pr, pl = F.TiltPlan(R, station.BASES["right"]), F.TiltPlan(L, station.BASES["left"])
    fixed, ring, moving = trainer_hulls(seed)
    bench = bench_hulls()
    dur = max(pr.duration, pl.duration)
    w, t = _Worst(), 0.0
    labels = [x.label for x in R]
    end = labels.index(HELD[1]) if HELD[1] in labels else len(R)   # WHY: stem k00 never lets go
    held = (sum(x.duration for x in R[:labels.index(HELD[0])]), sum(x.duration for x in R[:end]) + (9e9 if end == len(R) else 0))
    while t <= dur + 1e-9:
        qr, _ = pr.action7(t)
        ql, _ = pl.action7(t)
        tr, tl = pr.target(t), pl.target(t)
        a = arm_points("right", qr[:6], tr[2])
        b = arm_points("left", ql[:6], tl[2])
        if held[0] + 0.6 <= t < held[1]:   # WHY + 0.6: the fingers close on it over the "grip the housing" segment
            pos, rot = F.ik.tcp_pose(qr[:6], station.BASES["right"])
            lan = lancet_points(pos, rot, skip_face=tr[3] in ON_SITE)
        else:
            lan = None
        when = f"t {t:.1f} s ({tr[3]} / {tl[3]})"
        w.add("arms", _arm_gap(_join(a, lan) if lan is not None else a, b), when)
        for side, p, tg in (("right", a, tr), ("left", b, tl)):
            w.add("table", float((p[0][:, 2] - p[1]).min() - F.TOP), f"{side} {when}")
            g = min(_gap(fixed, p), _gap(moving, p))
            if not (side == "left" and tg[3] in HOLDING):
                g = min(g, _gap(ring, p))
            else:   # the pads close on the ring by design; the rest of the left must clear it
                body = arm_points("left", ql[:6], tl[2], links=LINKS[:-2])
                g = min(g, _gap(ring, body))
            w.add("trainer", g, f"{side} {when}")
            w.add("bench", _gap(bench, p), f"{side} {when}")
        if lan is not None:
            w.add("trainer", min(_gap(fixed, lan), _gap(moving, lan), _gap(ring, lan)), f"lancet {when}")
            if tr[3] not in AT_STAND:   # WHY: in the stand the lancet sits in its cup by design
                w.add("bench", _gap(bench, lan), f"lancet {when}")
        t += DT_CHECK
    ok = all(w.v[k][0] >= FLOORS[k] for k in FLOORS)
    line = "; ".join(f"{k} {1000 * w.v[k][0]:.1f} ({w.v[k][1]})" for k in FLOORS)
    return ok, f"{name}: least mm (30.0 means 30 or more): {line} " + ("OK" if ok else "BAD")


def jobs_for(args):
    jobs = []
    for a in args:
        if a == "corners":
            jobs += [(c, f"corner {c[0] * 1000:+.0f}, {c[1] * 1000:+.0f} mm, {math.degrees(c[2]):+.0f} deg", None)
                     for c in F.corners()]
        elif a == "stems":
            sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
            import ikcheck
            seed = zlib.crc32(b"0:None:0")
            jobs += [(seed, f"--seed 0 stem {k}00", ikcheck.stem_plan(k, seed)) for k in ikcheck.STEMS]
        else:
            k, v = a.split("=")
            jobs.append((int(v) if k == "seed" else zlib.crc32(f"{v}:None:0".encode()), a, None))
    return jobs


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("cover=")] or ["cli=0", "cli=2", "cli=4", "corners", "stems"]
    for a in sys.argv[1:]:
        if a.startswith("cover="):
            c = float(a.split("=")[1]) / 1000
            for side in COVERS:
                COVERS[side] = {k: c for k in LINKS}
    print("covers, mm: " + "; ".join(f"{side} " + ", ".join(f"{k} {1000 * v:g}" for k, v in c.items())
                                     for side, c in COVERS.items()))
    print("floors, mm: " + ", ".join(f"{k} {1000 * v:g}" for k, v in FLOORS.items()))
    jobs = jobs_for(args)
    with mp.get_context("fork").Pool(min(len(jobs), max(1, mp.cpu_count() - 1))) as pool:
        res = pool.map(check, jobs)
    for ok, line in res:
        print(line)
    bad = sum(not ok for ok, _ in res)
    print("GRIPPERS OK" if bad == 0 else f"GRIPPERS BAD {bad}")
    return bad


if __name__ == "__main__":
    sys.exit(1 if main() else 0)
