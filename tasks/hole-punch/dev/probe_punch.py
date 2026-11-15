"""HOLE-PUNCH probe: do pre-scored slugs leave the card only when the card is in the punch's path?

No robot. One copy of the fixture per case, side by side in one PhysX scene. External forces stand in for the arm
(a light hold-down on the card) and for the lever (a straight-down force on the punch head).

Frame per copy (m): z up, bed top at z 0, the card's leading edge at x 0 when home against the back stop, punch
pins at (0.012, +-0.040) (ISO 838: 80 mm apart, 12 mm from the edge).

Bodies:
  bed     kinematic steel bed, x -0.010..0.040, with an 8 mm square die hole under each pin; back stop wall x < 0
  table   kinematic table top 20 mm below the bed (the overhanging card can droop onto it)
  card    A6 card stock 105 x 148 mm, modelled 1.0 mm thick, 4.7 g, with a 6.6 mm square cut-out at each pin
  slugs   two 6 mm discs, 1.0 mm, sitting in the cut-outs, held to the card by fixed joints that break at BREAK_N
  head    punch head: a crossbar with two 3 mm pins, on a vertical slide with a return spring, travel 9 mm

A slug counts as punched when it ends below z -0.004 (fell through the die into the chip tray).

Run (GPU machine): source /data/IsaacLab-Arena/yam.env; python probe_punch.py --out out_<tag>
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

from isaaclab.app import AppLauncher

ap = argparse.ArgumentParser()
ap.add_argument("--out", default="out")
ap.add_argument("--device", default="cuda:0")
ap.add_argument("--break_n", type=float, default=8.0)
ap.add_argument("--tol_mm", type=float, default=1.5)
args = ap.parse_args()
app = AppLauncher(headless=True, device=args.device).app

import numpy as np  # noqa: E402
import torch  # noqa: E402
import warp as wp  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import RigidObject, RigidObjectCfg  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from usd_bodies import Body  # noqa: E402

DT = 1.0 / 240.0
CO = 0.0005
T = 0.0015                      # card thickness in the model (real card stock about 0.35 mm)
CX, CY = 0.105, 0.148           # card depth (into the throat) and width
PIN_X, PIN_Y = 0.012, 0.040
CUT = 0.0066                    # square cut-out in the card
SLUG_R = 0.0030
PIN_R = 0.0015
DIE = 0.008
BED_X1 = 0.040
HEAD_REST = 0.003               # pin tips 3 mm above the bed at rest
TRAVEL = 0.009
CARD_M = 0.0047
SLUG_M = 0.0005

# name, card dx, card dy, hold-down N, head force N
CASES = [
    ("A_home_centred", 0.0, 0.0, 2.0, 30.0),
    ("B_side_1.0mm", 0.0, 0.0010, 2.0, 30.0),
    ("C_side_2.5mm", 0.0, 0.0025, 2.0, 30.0),
    ("D_side_4mm", 0.0, 0.004, 2.0, 30.0),
    ("E_edge_1.0mm_short", 0.0010, 0.0, 2.0, 30.0),
    ("F_edge_2.5mm_short", 0.0025, 0.0, 2.0, 30.0),
    ("G_pulled_back_30mm", 0.030, 0.0, 2.0, 30.0),
    ("H_no_card", None, None, 0.0, 30.0),
    ("I_released_before_press", 0.0, 0.0, 0.0, 30.0),
    ("J_weak_press_6N", 0.0, 0.0, 2.0, 6.0),
]
SPACING = 0.5


def build():
    b = {}
    bed = Body("bed", kinematic=True, friction=(0.4, 0.3), contact_offset=CO)
    grey = (0.35, 0.35, 0.38)
    h = 0.010
    # bed plate x -0.010..BED_X1, y +-0.10, top at z 0, with square die holes at the pins
    xs = [-0.010, PIN_X - DIE / 2, PIN_X + DIE / 2, BED_X1]
    ys = [-0.10, -PIN_Y - DIE / 2, -PIN_Y + DIE / 2, PIN_Y - DIE / 2, PIN_Y + DIE / 2, 0.10]
    for i in range(3):
        for j in range(5):
            if i == 1 and j in (1, 3):
                continue  # die holes
            x0, x1, y0, y1 = xs[i], xs[i + 1], ys[j], ys[j + 1]
            bed.box(((x0 + x1) / 2, (y0 + y1) / 2, -h / 2), (x1 - x0, y1 - y0, h), grey)
    bed.box((-0.015, 0.0, 0.004), (0.010, 0.20, 0.008), (0.2, 0.2, 0.2))  # back stop wall, face at x -0.010
    b["bed"] = bed.save()
    table = Body("table", kinematic=True, friction=(0.5, 0.4), contact_offset=CO)
    table.box((0.10, 0.0, -0.025), (0.40, 0.40, 0.010), (0.6, 0.45, 0.3))  # top at z -0.020
    b["table"] = table.save()
    card = Body("card", mass=CARD_M, com=(0, 0, 0), inertia=(CARD_M * CY**2 / 12, CARD_M * CX**2 / 12,
                CARD_M * (CX**2 + CY**2) / 12), friction=(0.4, 0.3), contact_offset=CO, solver_iters=(64, 16))
    # card body frame at its centre; leading edge at local x -CX/2
    ex = -CX / 2
    cx = [ex, ex + PIN_X - CUT / 2, ex + PIN_X + CUT / 2, CX / 2]
    cy = [-CY / 2, -PIN_Y - CUT / 2, -PIN_Y + CUT / 2, PIN_Y - CUT / 2, PIN_Y + CUT / 2, CY / 2]
    for i in range(3):
        for j in range(5):
            if i == 1 and j in (1, 3):
                continue
            x0, x1, y0, y1 = cx[i], cx[i + 1], cy[j], cy[j + 1]
            card.box(((x0 + x1) / 2, (y0 + y1) / 2, 0.0), (x1 - x0, y1 - y0, T), (0.95, 0.85, 0.2))
    b["card"] = card.save()
    slug = Body("slug", mass=SLUG_M, com=(0, 0, 0), inertia=(SLUG_M * (2 * SLUG_R) ** 2 / 12, SLUG_M * (2 * SLUG_R) ** 2 / 12, SLUG_M * (2 * SLUG_R) ** 2 / 6), friction=(0.4, 0.3),
                contact_offset=CO, solver_iters=(32, 8))
    slug.cylinder((0, 0, 0), SLUG_R, T, (0.95, 0.85, 0.2))
    b["slug"] = slug.save()
    head = Body("head", mass=0.15, com=(0, 0, 0.010), inertia=(1e-4, 1e-5, 1e-4), friction=(0.4, 0.3),
                contact_offset=CO, solver_iters=(32, 8))
    head.box((0.0, 0.0, 0.012), (0.012, 0.10, 0.006), (0.15, 0.25, 0.7))
    for s in (-1, 1):
        head.cylinder((0.0, s * PIN_Y, 0.0045), PIN_R, 0.009, (0.8, 0.8, 0.8))
    b["head"] = head.save()
    return b


def place(stage, path, usd, pos):
    prim = stage.DefinePrim(path, "Xform")
    prim.GetReferences().AddReference(str(usd))
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
    x.AddOrientOp().Set(Gf.Quatf(1.0))


def joint(stage, cls, path, body0, body1, pos0, axis=None):
    j = cls.Define(stage, path)
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0))
    j.CreateCollisionEnabledAttr(False)
    j.CreateExcludeFromArticulationAttr(True)
    if body0:
        j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(body1)])
    if axis:
        j.CreateAxisAttr(axis)
    return j


def main():
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=args.device))
    stage = sim.stage
    usd = build()
    nC = len(CASES)
    for i, (name, dx, dy, *_r) in enumerate(CASES):
        o = np.array([i * SPACING, 0.0, 0.0])
        root = f"/World/C_{i}"
        stage.DefinePrim(root, "Xform")
        place(stage, f"{root}/bed", usd["bed"], o)
        place(stage, f"{root}/table", usd["table"], o)
        if dx is None:  # no card: park it far off the fixture, on the table
            cpos = o + (0.25, 0.0, -0.020 + T / 2 + 0.0002)
        else:
            cpos = o + (CX / 2 + dx, dy, T / 2 + 0.0002)
        place(stage, f"{root}/card", usd["card"], cpos)
        for nm in ("card",):
            PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f"{root}/{nm}")).CreateSleepThresholdAttr(0.0)
        for k, s in enumerate((-1, 1)):
            spos = cpos + (-CX / 2 + PIN_X, s * PIN_Y, 0.0)
            place(stage, f"{root}/slug{k}", usd["slug"], spos)
            PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f"{root}/slug{k}")).CreateSleepThresholdAttr(0.0)
        hpos = o + (PIN_X, 0.0, HEAD_REST)
        place(stage, f"{root}/head", usd["head"], hpos)
        jh = joint(stage, UsdPhysics.PrismaticJoint, f"{root}/j_head", None, f"{root}/head", hpos, axis="Z")
        jh.CreateLowerLimitAttr(-TRAVEL)
        jh.CreateUpperLimitAttr(0.0)
        d = UsdPhysics.DriveAPI.Apply(jh.GetPrim(), "linear")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(1300.0)  # cutting resistance: the pins reach the card only above about 13 N
        d.CreateDampingAttr(1000.0)  # a hand press is slow: 30 N moves the head about 30 mm/s
        d.CreateTargetPositionAttr(0.010)  # preloaded up against the top stop (3 N), as a real return spring

    objs = {n: RigidObject(RigidObjectCfg(prim_path=f"/World/C_.*/{n}", spawn=None))
            for n in ("card", "slug0", "slug1", "head")}
    sim.reset()
    for o in objs.values():
        o.update(DT)
    order = [int(p.split("/")[2].split("_")[1]) for p in objs["card"].root_view.prim_paths]
    for n, o in objs.items():
        assert [int(p.split("/")[2].split("_")[1]) for p in o.root_view.prim_paths] == order, n
    case = [CASES[k] for k in order]
    dev = args.device
    org = torch.tensor(np.array([[k * SPACING, 0, 0] for k in order]), dtype=torch.float32, device=dev)

    def pose(n):
        p = objs[n].data.root_pose_w.torch.clone()
        p[:, :3] -= org
        return p

    def push(n, f, pos=None):
        o = objs[n]
        o.root_view.apply_forces_and_torques_at_position(
            force_data=wp.from_torch(f.contiguous().float(), dtype=wp.float32),
            torque_data=wp.from_torch(torch.zeros_like(f).contiguous().float(), dtype=wp.float32),
            position_data=None if pos is None else wp.from_torch((pos + org).contiguous().float(), dtype=wp.float32),
            indices=o._ALL_INDICES, is_global=True)

    card0 = pose("card")[:, :3].clone()
    T_END, steps = 4.0, int(4.0 / DT)
    released = [[False, False] for _ in range(nC)]
    rel_t = [[None, None] for _ in range(nC)]

    def qrot(q, v):  # q (N,4) xyzw, v (N,3)
        u, w = q[:, :3], q[:, 3:4]
        return v + 2 * torch.cross(u, torch.cross(u, v, dim=1) + w * v, dim=1)

    def carry_slugs(t):
        c = objs["card"].data.root_pose_w.torch.clone()
        hz = pose("head")[:, 2]
        for k, sgn in ((0, -1), (1, 1)):
            o = objs[f"slug{k}"]
            off = torch.tensor([-CX / 2 + PIN_X, sgn * PIN_Y, 0.0], device=dev).repeat(nC, 1)
            want = c.clone()
            want[:, :3] = c[:, :3] + qrot(c[:, 3:7], off)
            # release rule: the pin has come down onto the card's top face (pressed), the slug's centre is
            # within tol of the pin's axis, and the card lies seated on the bed there
            pin_xy = torch.tensor([PIN_X, sgn * PIN_Y], device=dev)
            for i in range(nC):
                if released[i][k]:
                    continue
                d_xy = float(torch.linalg.norm(want[i, :2] - org[i, :2] - pin_xy))
                tip_z = float(hz[i]) - 0.0  # head origin = pin tip
                seated = abs(float(want[i, 2]) - T / 2) < 0.0005
                if tip_z <= T + 0.0002 and d_xy <= args.tol_mm / 1000 and seated:
                    released[i][k] = True
                    rel_t[i][k] = round(t, 3)
            keep = torch.tensor([not released[i][k] for i in range(nC)], device=dev)
            if keep.any():
                ids = torch.nonzero(keep).flatten()
                o.write_root_pose_to_sim(want[ids], env_ids=ids)
                o.write_root_velocity_to_sim(torch.zeros(len(ids), 6, device=dev), env_ids=ids)
    marks = {c[0]: {} for c in case}
    min_head = [1.0] * nC
    timeline = []
    for s in range(steps):
        t = s * DT
        hold = torch.zeros(nC, 3, device=dev)
        press = torch.zeros(nC, 3, device=dev)
        for i, (name, dx, dy, fh, fp) in enumerate(case):
            if fh > 0 and t < 3.0:
                hold[i, 2] = -fh
            if 1.5 <= t < 2.5:
                press[i, 2] = -fp * min(1.0, (t - 1.5) / 0.3)
        c = pose("card")
        # the hold-down presses the card onto the bed 25 mm in from its leading edge (over the bed)
        hpos = torch.stack([c[:, 0] - CX / 2 + 0.025, c[:, 1], c[:, 2]], 1)
        push("card", hold, hpos)
        push("head", press)
        carry_slugs(t)
        sim.step()
        for o in objs.values():
            o.update(DT)
        hz = pose("head")[:, 2]
        if s % 12 == 0:
            c, s0, s1 = pose("card"), pose("slug0"), pose("slug1")
            for i, cc in enumerate(case):
                offs = []
                for sp, sgn in ((s0, -1), (s1, 1)):
                    exp = c[i, :3] + torch.tensor([-CX / 2 + PIN_X, sgn * PIN_Y, 0.0], device=dev)
                    offs.append(round(float(torch.linalg.norm(sp[i, :3] - exp)) * 1000, 2))
                timeline.append({"t": round(t, 3), "case": cc[0], "head_mm": round((float(hz[i]) - HEAD_REST) * 1000, 2),
                                 "card_z_mm": round(float(c[i, 2]) * 1000, 2),
                                 "tilt": round(math.degrees(2 * math.acos(min(1.0, abs(float(c[i, 6]))))), 2),
                                 "slug_off": offs, "press_N": round(float(-press[i, 2]), 1)})
        for i in range(nC):
            min_head[i] = min(min_head[i], float(hz[i]))
        for tm, key in ((1.45, "before_press"), (3.95, "end")):
            if abs(t - tm) < DT / 2:
                c, s0, s1 = pose("card"), pose("slug0"), pose("slug1")
                for i, cc in enumerate(case):
                    rel = []
                    for sp, sgn in ((s0, -1), (s1, 1)):
                        exp = c[i, :3] + torch.tensor([-CX / 2 + PIN_X, sgn * PIN_Y, 0.0], device=dev)
                        rel.append(round(float(torch.linalg.norm(sp[i, :3] - exp)) * 1000, 2))
                    marks[cc[0]][key] = {
                        "card_shift_mm": [round(float(v) * 1000, 2) for v in (c[i, :3] - card0[i])],
                        "card_tilt_deg": round(math.degrees(2 * math.acos(min(1.0, abs(float(c[i, 6]))))), 2),
                        "slug_off_card_mm": rel,
                        "slug_z_mm": [round(float(s0[i, 2]) * 1000, 2), round(float(s1[i, 2]) * 1000, 2)],
                        "slugs_punched": sum(int(r > 3.0 and float(sp[i, 2]) < -0.004) for r, sp in zip(rel, (s0, s1))),
                    }
        if not math.isfinite(float(pose("card")[:, 2].sum())):
            print("NaN at", t, flush=True)
            break
    for i, cc in enumerate(case):
        marks[cc[0]]["head_min_mm"] = round((min_head[i] - HEAD_REST) * 1000, 2)
        marks[cc[0]]["released_at_s"] = rel_t[i]
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "timeline.jsonl"), "w") as f:
        for r in timeline:
            f.write(json.dumps(r) + "\n")
    with open(os.path.join(args.out, "marks.json"), "w") as f:
        json.dump({"break_n": args.break_n, "marks": marks}, f, indent=1)
    print(json.dumps(marks, indent=1), flush=True)
    print("PROBE_DONE", flush=True)


main()
app.close()
