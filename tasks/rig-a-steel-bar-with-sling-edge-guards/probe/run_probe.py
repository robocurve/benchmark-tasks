"""Check 2 (stop-loss): the sling / edge-guard / hook probe in Isaac Sim 6.1 (Isaac Lab 3, PhysX GPU, 240 Hz),
no robot. Physics settings as the YAM runtime's (yam_arena.scene.env_cfg_callback: dt 1/240, default PhysxCfg;
Arena's "/physics/tensors/recursiveLeafPatternMatch" = false).

Protocol: guards seated by the script on the four edges (or parked away: --case noguards), both fittings in the
hook's bowl, settle with the lift held, raise the lift at 30 mm/s until the hook carries the pull (default 50 N),
stop there and hold 2 s. Magnets act at every substep. Logged at 40 Hz: pull, per-edge "web on bare steel within
15 mm of the edge" and "web on guard" contact force (separate contact views per body pair, contact points from
get_contact_data), penetration (contact separations), guard displacement, web speed, hook swing, latch angle.

Run:  python run_probe.py --case guards --out out/guards-1
"""

import argparse
import json
import math
import os
import time
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--case", default="guards", choices=["guards", "noguards", "bottomless"])
ap.add_argument("--out", default="out/probe")
ap.add_argument("--settle", type=float, default=1.5)
ap.add_argument("--hold", type=float, default=2.0)
ap.add_argument("--pull", type=float, default=50.)
ap.add_argument("--speed", type=float, default=.03)
ap.add_argument("--swing", type=float, default=0., help="initial hook swing about x (deg)")
ap.add_argument("--usd", default=str(Path(__file__).with_name("sling_probe.usda")))
ap.add_argument("--mag-off-at-lift", action="store_true", help="diagnostic: magnets off once the lift starts")
ap.add_argument("--mag-f0", type=float, default=None, help="pull per magnet at zero gap (N); default sling_model")
ap.add_argument("--stabilization", action="store_true", help="diagnostic: PhysxCfg(enable_stabilization=True)")
ap.add_argument("--contact-last", action="store_true", help="diagnostic: PhysxCfg(solve_articulation_contact_last=True)")
args = ap.parse_args()
OUT = Path(args.out)
OUT.mkdir(parents=True, exist_ok=True)

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
from isaaclab.app import AppLauncher  # noqa: E402

app = AppLauncher(headless=True, device="cuda:0").app

import carb  # noqa: E402
import torch  # noqa: E402

import isaaclab.sim as sim_utils  # noqa: E402

import sling_model as m  # noqa: E402

if args.mag_f0 is not None:
    m.MAG_F0 = args.mag_f0

carb.settings.get_settings().set_bool("/physics/tensors/recursiveLeafPatternMatch", False)
DT = 1 / 240
from isaaclab_physx.physics import PhysxCfg  # noqa: E402

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device="cuda:0", render_interval=24,
                                                          physics=PhysxCfg(enable_stabilization=args.stabilization,
                                                                           solve_articulation_contact_last=args.contact_last)))
cfg = sim_utils.UsdFileCfg(usd_path=args.usd)
cfg.func("/World/Probe", cfg)
sim.reset()

import omni.physics.tensors as tensors  # noqa: E402

R = "/World/Probe"
view = tensors.create_simulation_view("torch")
view.set_subspace_roots("/")
LINKS = [f"{R}/Sling/link_{i:02d}" for i in range(m.N_LINKS)]
FITS = [f"{R}/Sling/fitA", f"{R}/Sling/fitB"]
GUARDS = list(m.EDGES)
GPATH = {g: f"{R}/Guards/guard_{g}" for g in GUARDS}
BAR = f"{R}/Bar"
HOOK, LATCH = f"{R}/HookRig/hook", f"{R}/HookRig/latch"

sling = view.create_articulation_view(f"{R}/Sling/link_11")
rig = view.create_articulation_view(f"{R}/HookRig")
guards = view.create_rigid_body_view(f"{R}/Guards/*")
gpaths = list(guards.prim_paths)
grow = {g: gpaths.index(GPATH[g]) for g in GUARDS}
web_bar = view.create_rigid_contact_view(LINKS, [[BAR]] * len(LINKS), max_contact_data_count=4096)
web_guard = {g: view.create_rigid_contact_view(LINKS, [[GPATH[g]]] * len(LINKS), max_contact_data_count=4096)
             for g in GUARDS}
fit_hook = view.create_rigid_contact_view(FITS, [[HOOK, LATCH]] * 2, max_contact_data_count=512)
fit_fit = view.create_rigid_contact_view([FITS[0]], [[FITS[1]]], max_contact_data_count=256)
fit_bar = view.create_rigid_contact_view(FITS, [[BAR]] * 2, max_contact_data_count=256)
guard_bar = view.create_rigid_contact_view([GPATH[g] for g in GUARDS], [[BAR]] * 4, max_contact_data_count=512)

dof = list(rig.shared_metatype.dof_names)
link_names = list(sling.shared_metatype.link_names)
DEV = "cuda:0"
one = torch.arange(1, dtype=torch.int32, device=DEV)
gids = torch.arange(len(gpaths), dtype=torch.int32, device=DEV)


def np_(x):
    return (x.torch if hasattr(x, "torch") else x).detach().cpu().numpy().copy()


def quat_R(q):                     # PhysX tensors: (x, y, z, w)
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# -- start state -------------------------------------------------------------------------------------------------
seat = {}
g_poses = np_(guards.get_transforms())
for g in GUARDS:
    seat[g] = g_poses[grow[g]].copy()
if args.case in ("noguards", "bottomless"):
    park = guards.get_transforms().clone()
    drop = GUARDS if args.case == "noguards" else ["BL", "BR"]
    for k, g in enumerate(drop):
        park[grow[g]] = torch.tensor((1.2 + .15 * k, .3, .01, 0., 0., 0., 1.), device=DEV)
    guards.set_transforms(park, gids)
    guards.set_velocities(torch.zeros_like(guards.get_velocities()), gids)
MAGNETS_ON = {g: not (args.case == "noguards" or (args.case == "bottomless" and g in ("BL", "BR"))) for g in GUARDS}
mag_local = {}
for g in GUARDS:                    # magnet points in the guard's frame (seat pose has identity rotation)
    mag_local[g] = [(p - seat[g][:3], n) for p, n in m.guard_magnets(g)]
coms = np_(guards.get_coms())       # local centre of mass (x, y, z, qx, qy, qz, qw)

if args.swing:
    q = rig.get_dof_positions().clone()
    q[0, dof.index("swing_x")] = math.radians(args.swing)
    rig.set_dof_positions(q, one)

# the sling's start shape: PhysX starts a floating articulation with straight hinges, so write the bends
sling_dofs = list(sling.shared_metatype.dof_names)
bend = m.start_hinge_angles()
q0 = sling.get_dof_positions().clone()
for i, a in enumerate(bend):
    q0[0, sling_dofs.index(f"hinge_{i:02d}")] = a
root_pose = {b[0]: (b[1], b[2]) for b in m.start_bodies()[0]}[f"link_{m.N_LINKS // 2:02d}"]
import scipy.spatial.transform as sst  # noqa: E402
rq = sst.Rotation.from_matrix(root_pose[1]).as_quat()        # x, y, z, w
rt = sling.get_root_transforms().clone()
rt[0] = torch.tensor((*root_pose[0], *rq), device=DEV)
sling.set_root_transforms(rt, one)
sling.set_dof_positions(q0, one)
sling.set_dof_velocities(torch.zeros_like(sling.get_dof_velocities()), one)
sling.set_dof_position_targets(torch.zeros_like(q0), one)
sling.set_root_velocities(torch.zeros_like(sling.get_root_velocities()), one)
START_BEND_DEG = [round(math.degrees(a), 1) for a in bend]

targets = torch.zeros_like(rig.get_dof_position_targets())
LIFT = dof.index("lift")


def apply_magnets():
    poses = np_(guards.get_transforms())
    F = np.zeros((len(gpaths), 3), np.float32)
    T = np.zeros_like(F)
    worst = {}
    for g in GUARDS:
        if not MAGNETS_ON[g]:
            continue
        p, q = poses[grow[g], :3], poses[grow[g], 3:]
        Rg = quat_R(q)
        pts = np.array([p + Rg @ lp for lp, _ in mag_local[g]])
        f, gap = m.magnet_force(pts)
        com = p + Rg @ coms[grow[g], :3]
        F[grow[g]] = f.sum(axis=0)
        T[grow[g]] = np.cross(pts - com, f).sum(axis=0)
        worst[g] = float(gap.max())
    guards.apply_forces_and_torques_at_position(torch.as_tensor(F, device=DEV), torch.as_tensor(T, device=DEV), None,
                                                gids, True)
    return worst


EDGE_YZ = {g: np.array(m.EDGES[g][0]) for g in GUARDS}


def edge_split(cview, sensors):
    """Per edge: total normal force (N) of contacts within 15 mm of the edge; min separation (mm); all-contact sum."""
    forces, points, normals, seps, counts, starts = cview.get_contact_data(dt=DT)
    forces, points, seps = np_(forces).reshape(-1), np_(points).reshape(-1, 3), np_(seps).reshape(-1)
    counts, starts = np_(counts).reshape(sensors, -1), np_(starts).reshape(sensors, -1)
    per_edge = {g: 0. for g in GUARDS}
    total, minsep, n = 0., 99., 0
    near_pts = []
    for i in range(sensors):
        c, s = int(counts[i, 0]), int(starts[i, 0])
        for k in range(s, s + c):
            f = float(forces[k])
            total += f
            n += 1
            if f > 0:
                minsep = min(minsep, 1000 * float(seps[k]))
            yz = points[k, 1:]
            for g in GUARDS:
                if np.linalg.norm(yz - EDGE_YZ[g]) <= .015:
                    per_edge[g] += f
                    if f > 0:
                        near_pts.append([round(1000 * float(v), 1) for v in points[k]])
    return per_edge, total, minsep, n, near_pts


def force_sum(cview):
    mat = np_(cview.get_contact_force_matrix(dt=DT))
    return mat.reshape(-1, 3).sum(axis=0), float(np.linalg.norm(mat, axis=-1).sum())


def guard_offsets(poses):
    out = {}
    for g in GUARDS:
        p, q = poses[grow[g], :3], poses[grow[g], 3:]
        d = 1000 * float(np.linalg.norm(p - seat[g][:3]))
        ang = math.degrees(2 * math.acos(min(1., abs(float(q[3])))))
        out[g] = (round(d, 2), round(ang, 2))
    return out


def snapshot(t, phase):
    lt = np_(sling.get_link_transforms())[0]
    lv = np_(sling.get_link_velocities())[0]
    gp = np_(guards.get_transforms())
    rq = np_(rig.get_dof_positions())[0]
    bar_edge, bar_total, bar_sep, bar_n, bar_pts = edge_split(web_bar, len(LINKS))
    gf = {}
    gsep = {}
    for g in GUARDS:
        vec, mag = force_sum(web_guard[g])
        gf[g] = mag
        _, _, sep, _, _ = edge_split(web_guard[g], len(LINKS))
        gsep[g] = sep
    hv, hmag = force_sum(fit_hook)
    _, ff = force_sum(fit_fit)
    _, fb = force_sum(fit_bar)
    _, gb = force_sum(guard_bar)
    web_ids = [link_names.index(n.split("/")[-1]) for n in LINKS]
    speed = np.linalg.norm(lv[web_ids, :3], axis=1)
    # geometric check: web link corners inside the bar box
    depth = 0.
    for i in web_ids:
        Rl = quat_R(lt[i, 3:])
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    c = lt[i, :3] + Rl @ (np.array((sx * m.SLING_W / 2, sy * m.LINK_L / 2, sz * m.WEB_T / 2)))
                    inside = np.minimum(c - m.BAR_LO, m.BAR_HI - c)
                    if (inside > 0).all():
                        depth = max(depth, float(inside.min()))
    return {
        "t": round(t, 4), "phase": phase,
        "lift_mm": round(1000 * float(rq[LIFT]), 2), "lift_target_mm": round(1000 * float(targets[0, LIFT]), 2),
        "pull_z_n": round(float(hv[2]), 2), "pull_abs_n": round(hmag, 2),
        "swing_deg": [round(math.degrees(float(rq[dof.index('swing_x')])), 2), round(math.degrees(float(rq[dof.index('swing_y')])), 2)],
        "latch_deg": round(math.degrees(float(rq[dof.index('latch_hinge')])), 2),
        "web_on_bare_steel_within_15mm_n": {g: round(v, 3) for g, v in bar_edge.items()},
        "web_on_bare_steel_total_n": round(bar_total, 3), "web_bar_contacts": bar_n,
        "web_bar_min_sep_mm": round(bar_sep, 3), "web_bar_points_mm": bar_pts[:6],
        "web_on_guard_n": {g: round(v, 3) for g, v in gf.items()},
        "web_guard_min_sep_mm": {g: round(v, 3) for g, v in gsep.items()},
        "web_corner_in_bar_mm": round(1000 * depth, 3),
        "fit_on_fit_n": round(ff, 2), "fit_on_bar_n": round(fb, 2), "guard_on_bar_n": round(gb, 2),
        "guard_offset_mm_deg": guard_offsets(gp),
        "web_speed_max_mm_s": round(1000 * float(speed.max()), 2), "web_speed_rms_mm_s": round(1000 * float(np.sqrt((speed ** 2).mean())), 2),
        "web_z_mm": [round(1000 * float(lt[i, 2]), 1) for i in web_ids],
        "web_y_mm": [round(1000 * float(lt[i, 1]), 1) for i in web_ids],
    }


# -- run ---------------------------------------------------------------------------------------------------------
log = []
t, phase, steps = 0., "settle", 0
t_phase = 0.
wall = time.time()
reached = None
nonfinite = False
while True:
    if phase == "settle" and t_phase >= args.settle:
        phase, t_phase = "lift", 0.
        if args.mag_off_at_lift:
            MAGNETS_ON = {g: False for g in GUARDS}
        log.append(snapshot(t, "end_settle"))
    if phase == "lift":
        pull = abs(log[-1]["pull_z_n"]) if log else 0.
        if pull >= args.pull:
            phase, t_phase = "hold", 0.
            reached = {"t": round(t, 3), "lift_mm": log[-1]["lift_mm"], "pull_n": pull}
        elif targets[0, LIFT] < .19:
            targets[0, LIFT] += args.speed * DT
        else:
            phase = "failed_no_pull"
    if phase == "hold" and t_phase >= args.hold:
        log.append(snapshot(t, "end_hold"))
        break
    if phase == "failed_no_pull" or t > 30:
        log.append(snapshot(t, "abort"))
        break
    rig.set_dof_position_targets(targets, one)
    apply_magnets()
    sim.step(render=False)
    t += DT
    t_phase += DT
    steps += 1
    if steps % 6 == 0:
        row = snapshot(t, phase)
        log.append(row)
        if not np.isfinite(row["pull_z_n"]) or row["web_speed_max_mm_s"] > 5e4:
            nonfinite = True
            break

# -- summary -----------------------------------------------------------------------------------------------------
hold = [r for r in log if r["phase"] == "hold"]
last1 = [r for r in hold if r["t"] >= hold[-1]["t"] - 1.0] if hold else []
settle_end = next((r for r in log if r["phase"] == "end_settle"), None)


def stat(rows, f):
    v = np.array([f(r) for r in rows]) if rows else np.array([np.nan])
    return {"min": round(float(v.min()), 3), "mean": round(float(v.mean()), 3), "max": round(float(v.max()), 3),
            "std": round(float(v.std()), 3)}


summary = {
    "usd": args.usd, "stabilization": args.stabilization, "contact_last": args.contact_last, "mag_f0_n": m.MAG_F0, "mag_off_at_lift": args.mag_off_at_lift, "lift_speed_m_s": args.speed,
    "start_bend_deg": START_BEND_DEG,
    "case": args.case, "swing_deg": args.swing, "pull_goal_n": args.pull, "steps": steps, "sim_s": round(t, 3),
    "wall_s": round(time.time() - wall, 1), "nonfinite_or_blowup": nonfinite, "reached": reached,
    "final_phase": log[-1]["phase"],
    "guard_offset_end_settle": settle_end["guard_offset_mm_deg"] if settle_end else None,
    "guard_offset_end_hold": log[-1]["guard_offset_mm_deg"],
    "last_1s_of_hold": {
        "pull_z_n": stat(last1, lambda r: abs(r["pull_z_n"])),
        "web_speed_max_mm_s": stat(last1, lambda r: r["web_speed_max_mm_s"]),
        "web_on_bare_steel_within_15mm_n": {g: stat(last1, lambda r, g=g: r["web_on_bare_steel_within_15mm_n"][g]) for g in GUARDS},
        "web_on_bare_steel_total_n": stat(last1, lambda r: r["web_on_bare_steel_total_n"]),
        "web_on_guard_n": {g: stat(last1, lambda r, g=g: r["web_on_guard_n"][g]) for g in GUARDS},
        "web_bar_min_sep_mm": stat(last1, lambda r: r["web_bar_min_sep_mm"]),
        "web_guard_min_sep_mm": {g: stat(last1, lambda r, g=g: r["web_guard_min_sep_mm"][g]) for g in GUARDS},
        "web_corner_in_bar_mm": stat(last1, lambda r: r["web_corner_in_bar_mm"]),
        "fit_on_fit_n": stat(last1, lambda r: r["fit_on_fit_n"]),
        "latch_deg": stat(last1, lambda r: r["latch_deg"]),
        "lift_mm": stat(last1, lambda r: r["lift_mm"]),
    },
    "whole_run": {
        "web_speed_max_mm_s": stat(log, lambda r: r["web_speed_max_mm_s"]),
        "web_guard_min_sep_mm": {g: stat(log, lambda r, g=g: r["web_guard_min_sep_mm"][g]) for g in GUARDS},
        "web_bar_min_sep_mm": stat(log, lambda r: r["web_bar_min_sep_mm"]),
        "web_corner_in_bar_mm": stat(log, lambda r: r["web_corner_in_bar_mm"]),
    },
}
# the criterion (as the scorer would apply it after the tension): every edge bare-steel force exactly 0 over the
# last 1 s of the hold, and web-on-guard force > 0
if last1:
    bare = {g: max(r["web_on_bare_steel_within_15mm_n"][g] for r in last1) for g in GUARDS}
    onguard = {g: min(r["web_on_guard_n"][g] for r in last1) for g in GUARDS}
    summary["criterion"] = {"bare_steel_max_n": bare, "on_guard_min_n": onguard,
                            "edges_protected": {g: bool(bare[g] == 0. and onguard[g] > 0.) for g in GUARDS}}
    summary["criterion"]["all_protected"] = all(summary["criterion"]["edges_protected"].values())
(OUT / "log.json").write_text(json.dumps(log, default=float))
(OUT / "summary.json").write_text(json.dumps(summary, indent=1, default=float))
print(json.dumps(summary, indent=1, default=float))
app.close()
