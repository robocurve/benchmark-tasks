"""Kinematic dry run of the scripted demonstration, without Isaac Sim: the arms follow their commands exactly, a guard
or fitting the closed fingers hold moves with the hand, a released guard near its seat snaps onto it (the magnets) and
a released fitting whose apex bar is past its latch drops to its bowl's floor. Reports every inverse-kinematics search,
the joint margins, the arms' clearance from the bar, blocks, trays, table and hook (collision hulls of the URDF,
``probe/ik_scan.py``), and the static grade at the end (no hoist test).

Run on a machine with yam_arena and inspect_robots installed, from the task folder:
``python tools/dryrun_demo.py [variant] > dryrun.json``
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "probe"))

from yam_arena import ik  # noqa: E402

import build_assets as BA  # noqa: E402
from rdb_sling_guards import fixed_scene as F  # noqa: E402
from rdb_sling_guards import grading  # noqa: E402
from rdb_sling_guards import hook as HK  # noqa: E402
from rdb_sling_guards import layout as L  # noqa: E402
from rdb_sling_guards import scripted as S  # noqa: E402

OPEN = S.OPEN


def quat(R):
    return tuple(float(v) for v in ik.matrix_to_quat_xyzw(R))


def start_state():
    guards = {}
    for name in F.GUARD_NAMES:
        p, R = BA.tray_pose(name)
        guards[name] = (np.asarray(p, float) - (0., 0., F.TOP), np.asarray(R, float))
    fits = {}
    for name, p, cols in F.sling_start():
        if name in F.FITS:
            R = np.column_stack(cols)
            sgn = -1. if name == "fitA" else 1.
            fits[name] = (np.asarray(p, float), R, sgn)
    return guards, fits


class Scene:
    def __init__(self):
        self.guards, self.fits = start_state()
        self.held = {arm: None for arm in F.ARMS}
        self.O = np.array(F.HOOK_O, float)
        self.Rh = np.eye(3)
        self.events = []

    def fit_dict(self, name):
        p, R, sgn = self.fits[name]
        u = sgn * R[:, 1]
        return {"base": p.copy(), "apex": p + L.FIT_H * u, "u": u, "bar": R[:, 0].copy()}

    def state(self, tick):
        return {"guards": {n: (tuple(p), quat(R)) for n, (p, R) in self.guards.items()},
                "fits": {n: self.fit_dict(n) for n in self.fits}, "hook_O": self.O.copy(), "hook_R": self.Rh.copy(),
                "latch_deg": {"L": 0., "R": 0.}, "magnet_gap_mm": {}, "grade": None}

    def grasp_points(self):
        out = []
        for n, (p, R) in self.guards.items():
            side, edge, _ = F.tray_slots()[n]
            gp, _ = S.guard_grip(edge, p, R)
            out.append(("guard", n, gp))
        for n in self.fits:
            f = self.fit_dict(n)
            e = np.array((1., 0., 0.))
            nn = np.cross(e, f["u"])
            if nn[2] < 0:
                e, nn = -e, -nn
            gp, _ = S.RiggingReference().fit_grip(f["base"], f["u"], e, nn)
            out.append(("fit", n, gp))
        return out

    def update(self, tick, joints, fingers):
        for i, arm in enumerate(F.ARMS):
            tp, tR = ik.tcp_pose(joints[arm], S.BASES[arm])
            closed = fingers[arm] > OPEN * .5 and -fingers[arm] * 2 < .085      # pads closer than 85 mm
            gap = -2 * fingers[arm]
            if self.held[arm] is None:
                for kind, n, gp in self.grasp_points():
                    size = L.GUARD_LEN if kind == "guard" else 2 * L.FIT_SIDE_R
                    if np.linalg.norm(gp - tp) < .006 and gap < size:
                        obj = self.guards[n] if kind == "guard" else self.fits[n][:2]
                        rel = (tR.T @ (obj[0] - tp), tR.T @ obj[1])
                        self.held[arm] = (kind, n, rel, size)
                        self.events.append({"tick": tick, "event": "attached", "arm": arm, "object": n})
                        break
            else:
                kind, n, (lp, lR), size = self.held[arm]
                if gap > size + .002:
                    self.held[arm] = None
                    self.events.append({"tick": tick, "event": "released", "arm": arm, "object": n})
                    self.settle_released(kind, n, tick)
                    continue
                p, R = tp + tR @ lp, tR @ lR
                if kind == "guard":
                    self.guards[n] = (p, R)
                else:
                    self.fits[n] = (p, R, self.fits[n][2])

    def settle_released(self, kind, n, tick):
        if kind == "guard":
            p, R = self.guards[n]
            for edge in grading.EDGE_NAMES:
                seat = S.edge_point(edge)
                Rs = S.seat_frame(edge)
                if np.linalg.norm(p - seat) < .008 and grading.angle_between(tuple(map(tuple, Rs)), tuple(map(tuple, R))) < 8:
                    self.guards[n] = (seat.copy(), Rs)
                    self.events.append({"tick": tick, "event": "guard snapped", "guard": n, "edge": edge,
                                        "off_mm": round(1000 * float(np.linalg.norm(p - seat)), 2)})
                    return
            self.events.append({"tick": tick, "event": "guard dropped off any seat", "guard": n, "p_mm": [round(1000 * v, 1) for v in p]})
        else:
            f = self.fit_dict(n)
            apex = self.Rh.T @ (f["apex"] - self.O)
            for side in ("L", "R"):
                c = np.array(HK.bowl_centre(side))
                d = apex - c
                if math.hypot(d[1], d[2]) < L.BOWL_R_IN and d[2] < L.LATCH_END[1]:
                    # drops to the bowl's floor and hangs straight down
                    floor = self.O + self.Rh @ (c + np.array((0., 0., -(L.BOWL_R_IN - L.FIT_BAR_R))))
                    p, R, sgn = self.fits[n]
                    u = np.array((0., 0., 1.))
                    y = sgn * u
                    x = R[:, 0]
                    Rn = np.column_stack((x, y, np.cross(x, y)))
                    self.fits[n] = (floor - L.FIT_H * u, Rn, sgn)
                    self.events.append({"tick": tick, "event": "fitting dropped into bowl", "fitting": n, "horn": side,
                                        "apex_mm": [round(1000 * v, 1) for v in apex[1:]]})
                    return
            self.events.append({"tick": tick, "event": "fitting released outside a bowl", "fitting": n,
                                "apex_mm": [round(1000 * v, 1) for v in apex[1:]]})


def clearance_check(hulls, joints, fingers):
    """Arms' hull points against the bar, blocks, trays, table and hook (m)."""
    import ik_scan as IS

    boxes = [((L.BAR_X[0], -L.HALF, L.Z_BOT), (L.BAR_X[1], L.HALF, L.Z_TOP))]
    for x0, x1 in L.BLOCK_X:
        boxes.append(((x0, -L.BLOCK_L / 2, 0.), (x1, L.BLOCK_L / 2, L.BLOCK_H)))
    for side in ("left", "right"):
        (cx, cy, cz), (hx, hy, hz) = F.tray_block(side)
        boxes.append(((cx - hx, cy - hy, cz - hz), (cx + hx, cy + hy, cz + hz)))
        x, y = L.TRAY_XY[side]
        sx, sy, fl = L.TRAY_SIZE
        boxes.append(((x - sx / 2, y - sy / 2, 0.), (x + sx / 2, y + sy / 2, fl)))
    O = np.array(F.HOOK_O, float)
    caps = [(n, O + np.array((0., a[0], a[1])), O + np.array((0., b[0], b[1])), r) for n, a, b, r in HK.hook_capsules()]
    out = {}
    for arm in F.ARMS:
        pts = hulls.points(joints[arm], S.BASES[arm], fingers[arm])
        allp = np.concatenate(list(pts.values()))
        out[arm] = {"scene_mm": round(1000 * min(IS.box_dist(allp, *b) for b in boxes), 1),
                    "table_mm": round(1000 * float(allp[:, 2].min()), 1),
                    "hook_mm": round(1000 * min(IS.capsule_dist(allp, a, b, r) for _, a, b, r in caps), 1)}
    return out


def dry_run(variant="full", clearance_every=4):
    """Run the demonstration kinematically; returns a report (also printed by ``main``)."""
    import ik_scan as IS

    if IS.HULLS is None:
        IS.init()
    hulls = IS.HULLS
    scene = Scene()
    ref = S.RiggingReference(variant)
    seq = ref.sequence()
    joints = {arm: np.asarray(S.READY, float) for arm in F.ARMS}
    fingers = dict.fromkeys(F.ARMS, OPEN)
    worst = {arm: (9., None) for arm in F.ARMS}
    clear = []
    tick = 0
    for tick in range(F.MAX_STEPS):
        measured = np.concatenate([np.r_[joints[a], fingers[a] / OPEN] for a in F.ARMS])
        for arm in F.ARMS:          # the pads stop on a held object
            if scene.held[arm] is not None:
                measured[7 * F.ARMS.index(arm) + 6] = scene.held[arm][3] / (2 * abs(OPEN))
        ref.observe(measured, scene.state(tick))
        chunk = next(seq)
        values = np.asarray(chunk.actions[0].data, float)
        for i, arm in enumerate(F.ARMS):
            joints[arm] = values[7 * i:7 * i + 6]
            fingers[arm] = float(values[7 * i + 6]) * OPEN
            m = float(np.degrees(np.minimum(joints[arm] - ik.LOWER, ik.UPPER - joints[arm]).min()))
            if m < worst[arm][0]:
                worst[arm] = (m, tick)
        scene.update(tick, joints, fingers)
        if clearance_every and tick % clearance_every == 0:
            c = clearance_check(hulls, joints, {a: max(fingers[a], -.047) for a in F.ARMS})
            phase = ref.phases[-1][1] if ref.phases else ""
            clear.append((tick, phase, c))
        if ref.done:
            break
    seated = {}
    for n, (p, R) in scene.guards.items():
        e, det = grading.guard_seat(tuple(p), quat(R))
        seated[n] = (e, det)
    hooked = {}
    for n in scene.fits:
        f = scene.fit_dict(n)
        apex = scene.Rh.T @ (f["apex"] - scene.O)
        hooked[n] = [s for s in ("L", "R") if HK.in_bowl(tuple(apex[1:]), s)]
    worst_clear = {}
    for arm in F.ARMS:
        for key in ("scene_mm", "table_mm", "hook_mm"):
            rows = sorted(clear, key=lambda r: r[2][arm][key])[:3]
            worst_clear[f"{arm}_{key}"] = [(t, ph, r[arm][key]) for t, ph, r in rows]
    stopped = [e for e in ref.log["events"] if e["event"] == "demonstration stopped"]
    return {"variant": variant, "steps": tick + 1, "done": ref.done, "stopped": stopped, "phases": ref.phases,
            "worst_joint_margin_deg": worst, "worst_ik_mm": round(1000 * ref.worst_ik, 3),
            "events": ref.log["events"], "scene_events": scene.events,
            "seated": {n: v[0] for n, v in seated.items()}, "seat_details": {n: v[1] for n, v in seated.items()},
            "hooked": hooked, "worst_clearance": worst_clear}


def main():
    variant = sys.argv[1] if len(sys.argv) > 1 else "full"
    print(json.dumps(dry_run(variant), indent=1, default=float))


if __name__ == "__main__":
    main()
