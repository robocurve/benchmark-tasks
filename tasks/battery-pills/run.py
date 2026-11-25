"""battery-pills trial runner: one seeded scene, one policy, native evidence. Isaac Sim 6.1 only (no CPU fallback).

Policies:
  demo     demo_policy.BatteryDemo: right wrist camera + joint readings only (full plan)
  hover    the same, stopping with the cell held over the jar (baseline: never released, the dish never moved)
  nothing  hold the ready joint targets (baseline: do nothing)
  dream    Dream-1 (dreamlabs/dream-1-beta via inspect_robots_dream), camera and joint observations + instruction only

The seed picks the trial's draw (geometry.draws); the bodies are authored at the drawn poses, so no pose is written
during the trial. Actions are clipped to the URDF limits and rate-limited to the joint and gripper speeds as in
peg-out-a-tshirt/run.py. The scorer (scoring.SimScorer, the card's rule) reads every body's pose after every control
step and the gripper collision hulls every 10 steps and over the last 30 steps (Released); whenever a tablet lies
beyond the dish rim it also reads that tablet's top-view gap to the grippers (r7 spill rule); nothing it reads reaches
a policy.

Outputs in --out: run.json, score.json, trace.jsonl.gz, poses.npz, top_cam.mp4 / left_cam.mp4 / right_cam.mp4,
start-*.png / final-*.png / phase-*.png, task-source/ (frozen copy of this folder).
"""
import argparse
import gzip
import hashlib
import json
import os
import shutil
import signal
import time
from datetime import datetime, timezone
from importlib.metadata import PackageNotFoundError, version
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
CONTROL_HZ = 20
MAX_STEPS = 2400          # card run limit
SETTLE_STEPS = 20         # 1 s at the ready pose after the raw reset, every policy
CAMERAS = ('top_cam', 'left_cam', 'right_cam')
EXPLOSION_SPEED = 5.0     # m/s for any task body: stop the trial as unstable
BODIES = ['tray', 'dish', 'jar', 'cell'] + [f'tablet{i}' for i in range(5)]


def sources():
    out = {}
    for p in sorted(HERE.iterdir()):
        if p.suffix in ('.py', '.json', '.md', '.sh', '.txt'):
            if p.suffix == '.py':
                compile(p.read_bytes(), str(p), 'exec')
            elif p.suffix == '.json':
                json.loads(p.read_text())
            out[p.name] = hashlib.sha256(p.read_bytes()).hexdigest()
    return out


def package_versions():
    v = {}
    for name in ('isaacsim', 'isaaclab', 'isaaclab_arena', 'yam-arena', 'inspect-robots', 'inspect-robots-yam',
                 'inspect-robots-dream', 'dreamlabs-client', 'numpy'):
        try:
            v[name] = version(name)
        except PackageNotFoundError:
            v[name] = None
    return v


def encode(x):
    if isinstance(x, np.ndarray):
        return encode(x.tolist())
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return encode(x.item())
    if isinstance(x, dict):
        return {str(k): encode(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [encode(v) for v in x]
    if isinstance(x, float) and not np.isfinite(x):
        return None
    return x


def audit(stage):
    """Fail closed on attachments, joints outside the robots, or a kinematic / missing body."""
    from pxr import UsdPhysics
    res = dict(attachments=[], joints_outside_robots=[], bodies={}, physics_scenes=[])
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        schemas = [str(s) for s in prim.GetAppliedSchemas()]
        if prim.IsA(UsdPhysics.Scene):
            res['physics_scenes'].append(dict(path=path, gpu=prim.GetAttribute('physxScene:enableGPUDynamics').Get(),
                                              solver=prim.GetAttribute('physxScene:solverType').Get()))
        if 'Attachment' in prim.GetTypeName() or any('Attachment' in s for s in schemas):
            res['attachments'].append(path)
        if prim.IsA(UsdPhysics.Joint) and not any(k in path for k in ('LeftArm', 'RightArm', 'left_robot', 'right_robot')):
            res['joints_outside_robots'].append(path)
        for name in BODIES:
            if path.endswith('/' + name) and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                kin = prim.GetAttribute('physics:kinematicEnabled').Get()
                res['bodies'][name] = dict(path=path, kinematic=bool(kin), colliders=sum(
                    1 for c in prim.GetChildren() if c.HasAPI(UsdPhysics.CollisionAPI)),
                    mass_kg=prim.GetAttribute('physics:mass').Get())
    if res['attachments'] or res['joints_outside_robots']:
        raise RuntimeError(f"attachment or joint in an attachment-free scene: {res['attachments'] + res['joints_outside_robots']}")
    for name in BODIES:
        b = res['bodies'].get(name)
        if not b or b['kinematic']:
            raise RuntimeError(f'{name} is missing or kinematic')
    return res


class HullDistance:
    """Lower bound (separating planes) of the distance between the grippers' collision hulls and a body's convex
    pieces, from the simulator's poses. Exact when the closest features are a face and a vertex; never above the true
    distance, so Released (>= 30 mm) is read conservatively."""

    def __init__(self):
        import trimesh

        import scene as S
        self.pieces = {}
        for name in BODIES:
            out = []
            for _, pts in S.pieces_for(S.kind_of(name)):
                hull = trimesh.convex.convex_hull(np.asarray(pts, float))
                n = np.asarray(hull.face_normals, float)
                d = np.einsum('ij,ij->i', n, hull.vertices[hull.faces[:, 0]])
                v = np.asarray(hull.vertices, float)
                c = v.mean(0)
                out.append((v, n, d, c, float(np.linalg.norm(v - c, axis=1).max())))
            self.pieces[name] = out
        self.grip = None

    def set_grippers(self, hulls):
        self.grip = {}
        self.foot = None
        for side, lst in hulls.items():
            rows = []
            for _, h in lst:
                v = np.asarray(h.vertices, float)
                c = v.mean(0)
                rows.append((v, np.asarray(h.planes, float), c, float(np.linalg.norm(v - c, axis=1).max())))
            self.grip[side] = rows

    def _footprints(self):
        """Top-view footprints of the gripper hulls: the convex hull of each hull's vertices projected on the
        horizontal plane, as ordered polygons (metres). Built once per set_grippers, only when a spill check needs it."""
        if self.foot is None:
            from scipy.spatial import ConvexHull
            self.foot = {}
            for side, rows in self.grip.items():
                polys = []
                for v, _, _, _ in rows:
                    xy = v[:, :2]
                    try:
                        polys.append(xy[ConvexHull(xy).vertices])
                    except Exception:            # degenerate (flat) footprint: keep its points
                        polys.append(xy)
                self.foot[side] = polys
        return self.foot

    def xy_gap(self, pose, radius_m):
        """r7 spill rule: lower bound (mm) of the top-view gap between a tablet (footprint disc of ``radius_m`` about its
        centre) and each side's gripper parts (convex footprints), as (left, right)."""
        q = np.asarray(pose[0], float)[:2]
        out = []
        for side in ('left', 'right'):
            best = 1.0
            for poly in self._footprints()[side]:
                n = len(poly)
                if n >= 3:
                    a, b = poly, np.roll(poly, -1, axis=0)
                    e = b - a
                    cross = e[:, 0] * (q[1] - a[:, 1]) - e[:, 1] * (q[0] - a[:, 0])
                    if (cross >= 0).all() or (cross <= 0).all():
                        best = 0.0                      # centre inside the footprint
                        break
                else:
                    a, b = poly, np.roll(poly, -1, axis=0)
                    e = b - a
                L2 = np.maximum((e ** 2).sum(1), 1e-18)
                t = np.clip(((q - a) * e).sum(1) / L2, 0.0, 1.0)
                d = np.linalg.norm(a + t[:, None] * e - q, axis=1).min()
                best = min(best, d)
            out.append(max(0.0, best - radius_m) * 1000.0)
        return out

    def __call__(self, name, pose, cull=0.04):
        p, r = pose
        out = []
        for side in ('left', 'right'):
            best = 1.0
            for v, n, d, c, rad in self.pieces[name]:
                cw = r @ c + p
                vw = None
                for gv, gp, gc, grad in self.grip[side]:
                    gap = np.linalg.norm(gc - cw) - rad - grad
                    if gap > cull:
                        best = min(best, gap)
                        continue
                    if vw is None:
                        vw = v @ r.T + p
                        nw = n @ r.T
                        dw = d + nw @ p
                    s1 = (vw @ gp[:, :3].T - gp[:, 3]).min(0).max()     # gripper hull planes vs body vertices
                    s2 = (gv @ nw.T - dw).min(0).max()                  # body planes vs gripper vertices
                    best = min(best, max(s1, s2))
            out.append(best * 1000.0)
        return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--policy', choices=('demo', 'hover', 'nothing', 'dream'), default='nothing')
    ap.add_argument('--wall-limit', type=float, default=2150.0, help='stop stepping after this many seconds (still scores)')
    ap.add_argument('--dream-rows-per-request', type=int, choices=(1, 2, 4, 8), default=2)
    ap.add_argument('--steps', type=int, default=None, help=f'control steps (nothing: 60; else up to {MAX_STEPS})')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--physics-hz', type=int, choices=(240, 480), default=240)
    ap.add_argument('--no-video', action='store_true')
    ap.add_argument('--check-only', action='store_true')
    args = ap.parse_args()
    if args.steps is None:
        args.steps = 60 if args.policy == 'nothing' else MAX_STEPS
    if not 1 <= args.steps <= MAX_STEPS:
        ap.error(f'--steps must be 1..{MAX_STEPS}')
    hashes = sources()
    if args.check_only:
        print(json.dumps(dict(status='local_syntax_and_json_passed', source_sha256=hashes), indent=2))
        return
    if args.out is None:
        ap.error('--out is required')
    os.environ['ROBOCURVE_BP_SEED'] = str(args.seed)
    os.environ['ROBOCURVE_BP_PHYSICS_HZ'] = str(args.physics_hz)
    versions = package_versions()
    if not (versions.get('isaacsim') or '').startswith('6.1.'):
        raise RuntimeError('use the pinned Isaac Sim 6.1 environment')
    args.out.mkdir(parents=True, exist_ok=False)
    frozen = args.out / 'task-source'
    frozen.mkdir()
    for name in hashes:
        shutil.copyfile(HERE / name, frozen / name)
    spec = json.loads((HERE / 'task.json').read_text())
    t0 = time.monotonic()
    report = dict(kind=f'battery_pills_{args.policy}_trial', status='initializing', score=None, policy=args.policy,
                  seed=args.seed, started_utc=datetime.now(timezone.utc).isoformat(), source_sha256=hashes,
                  versions=versions, control_hz=CONTROL_HZ, physics_hz=args.physics_hz, requested_steps=args.steps,
                  completed_steps=0, instruction=spec['instruction'])

    def save():
        report['wall_seconds'] = time.monotonic() - t0
        tmp = args.out / 'run.json.tmp'
        tmp.write_text(json.dumps(encode(report), indent=2) + '\n')
        tmp.replace(args.out / 'run.json')

    save()
    app = trace = dream = None
    request_audit, session_audit = [], {}
    videos = {}

    def terminate(signum, frame):
        raise SystemExit('SIGTERM: trial interrupted')

    previous = signal.signal(signal.SIGTERM, terminate)
    try:
        os.environ.setdefault('OMNI_KIT_ACCEPT_EULA', 'YES')
        from isaaclab.app import AppLauncher
        app = AppLauncher(headless=True, enable_cameras=True).app
        import imageio.v2 as imageio
        import omni.usd
        import torch
        from PIL import Image

        from yam_arena import inspect_embodiment as wrapper
        from yam_arena import scene as yscene
        yscene.PHYSICS_DT = 1.0 / args.physics_hz
        np.random.seed(args.seed)
        torch.manual_seed(args.seed)
        wrapper._APP = app
        emb = wrapper.YamArenaEmbodiment(task=str(HERE / 'task.py'), object_jitter=0.0, control_hz=CONTROL_HZ,
                                         cam_width=320, cam_height=176)
        emb._ensure_sim(None)
        base = emb._base
        emb._instruction = spec['instruction']
        report['build_wall_seconds'] = time.monotonic() - t0
        import arena_bp as AB
        import geometry as G
        from scoring import S as SCORING
        from scoring import SimScorer, explain
        SPILL_R = SCORING['spill_tablet_radius_mm'] / 1000.0
        lay = G.layout(args.seed)
        report['draw'] = lay['draw']
        report['layout_mm'] = dict(tray=lay['tray'], dish=lay['dish'], jar=lay['jar'], cell=lay['cell'], tablets=lay['tablets'])
        report['physics_dt_s'] = float(base.sim.get_physics_dt())
        report['spawn'] = dict(AB.TRIAL['audit'])
        stage = omni.usd.get_context().get_stage()
        report['native_scene_audit'] = audit(stage)
        save()

        env = emb._env
        ready = emb._ready.detach().clone()
        hd = HullDistance()

        def measure():
            return {n: AB.body_pose(env, n) for n in BODIES}

        def speeds():
            return {n: float(np.linalg.norm(AB.body_velocity(env, n)[0])) for n in BODIES}

        def card_pos(poses):
            return {n: (G.card(p)).round(3).tolist() for n, (p, r) in poses.items()}

        env.reset()
        AB.TRIAL['tracker'] = None
        p0 = measure()
        report['after_reset'] = dict(pos_mm=card_pos(p0))
        for k in range(SETTLE_STEPS):
            env.step(ready)
            if k in (0, 9, 19):
                m = measure()
                report.setdefault('settle', []).append(dict(step=k + 1, pos_mm=card_pos(m), speed=speeds(),
                                                            tilt_deg={n: round(G.tilt_deg(r), 3) for n, (p, r) in m.items()}))
        for _ in range(4):
            base.sim.render()
        emb._t = 0.0
        save()

        if not args.no_video:
            for cam in CAMERAS:
                videos[cam] = imageio.get_writer(args.out / f'{cam}.mp4', fps=CONTROL_HZ, codec='libx264', pixelformat='yuv420p',
                                                 ffmpeg_params=['-movflags', '+faststart'], macro_block_size=1)
        trace = gzip.open(args.out / 'trace.jsonl.gz', 'wt', compresslevel=1)
        from yam_arena.constants import FINGER_OPEN, GRIPPER, SHOULDER, WRIST
        delta = np.tile(np.array([SHOULDER['velocity'] / CONTROL_HZ] * 3 + [WRIST['velocity'] / CONTROL_HZ] * 3 +
                                 [GRIPPER['velocity'] / abs(FINGER_OPEN) / CONTROL_HZ]), 2)
        policy = None
        queued = []
        session_audit = {'session_count': 0}
        if args.policy in ('demo', 'hover'):
            from demo_policy import make_policy
            policy = make_policy(320, 176, CONTROL_HZ, mode=args.policy)
            report['policy_description'] = 'scripted: right wrist camera RGB, rig calibration, joint readings only'
        elif args.policy == 'dream':
            from inspect_robots import Scene

            from dream_policy import MODEL
            from dream_policy import make_policy as make_dream
            dream = make_dream(request_audit, session_audit, rows_per_request=args.dream_rows_per_request)
            dream.bind(emb.info)
            dream.reset(Scene(id='battery-pills', instruction=spec['instruction']))
            report['dream_model'] = MODEL
            report['policy_description'] = 'Dream-1: ordinary camera and joint observations and the instruction only'
        else:
            report['policy_description'] = 'constant ready joint targets'
        tracker = SimScorer(CONTROL_HZ, G.TABLE_TOP)
        obs = emb._observation()
        for cam in CAMERAS:
            Image.fromarray(np.asarray(obs.images[cam], np.uint8)).save(args.out / f'start-{cam}.png')
        active = ready.detach().cpu().numpy()[0].copy()
        poses_log = {k: [] for k in ['step'] + BODIES}
        report['status'] = 'running'
        save()
        result, step, frames_saved = None, 0, set()
        for step in range(1, args.steps + 1):
            tp = time.monotonic()
            if policy is not None:
                ref = np.asarray(policy.act(obs), float)
            elif dream is not None:
                if not queued:
                    queued.extend(np.asarray(a.data, float) for a in dream.act(obs).actions)
                ref = queued.pop(0)
            else:
                ref = ready.detach().cpu().numpy()[0]
            policy_s = time.monotonic() - tp
            if ref.shape != (14,) or not np.isfinite(ref).all():
                raise ValueError('invalid 14-D joint action')
            active = active + np.clip(np.clip(ref, emb._low, emb._high) - active, -delta, delta)
            env.step(torch.as_tensor(active[None, :], device=base.device, dtype=torch.float32))
            emb._t = step / CONTROL_HZ
            poses = measure()
            ending = ((policy is not None and policy.done) or step >= args.steps - 30 or time.monotonic() - t0 > args.wall_limit)
            read_robot = step % 10 == 0 or ending
            hd.set_grippers(AB.gripper_hulls(env)) if True else None
            result = tracker.update(poses, lambda n: hd(n, poses[n]), read_robot=read_robot,
                                    grip_xy=lambda n: hd.xy_gap(poses[n], SPILL_R))
            obs = emb._observation()
            phase = policy.log[-1].name if policy is not None and policy.log else None
            row = dict(step=step, wall_s=round(time.monotonic() - t0, 3), policy_s=round(policy_s, 4),
                       action=active.tolist(), joints=np.asarray(obs.state['joint_pos'], float).tolist(),
                       score=result['score'], pos_mm=card_pos(poses), grip_mm=result['state']['gripper_min_dist_mm'],
                       phase=phase)
            trace.write(json.dumps(encode(row)) + '\n')
            for cam, w in videos.items():
                w.append_data(np.asarray(obs.images[cam], np.uint8))
            poses_log['step'].append(step)
            for n in BODIES:
                poses_log[n].append(np.r_[poses[n][0], poses[n][1].ravel()])
            if phase and phase not in frames_saved:
                frames_saved.add(phase)
                for cam in ('right_cam', 'left_cam', 'top_cam'):
                    Image.fromarray(np.asarray(obs.images[cam], np.uint8)).save(
                        args.out / f'phase-{len(frames_saved):02d}-{phase}-{cam}.png')
            sp = speeds()
            if step % 10 == 0:
                report.setdefault('diagnostics', []).append(dict(
                    step=step, phase=phase, score=result['score'], explanation=explain(result['conditions'], result['score']),
                    pos_mm={n: v for n, v in card_pos(poses).items() if n in ('dish', 'cell', 'jar', 'tablet0')},
                    dish_tilt_deg=round(G.tilt_deg(poses['dish'][1]), 2), grip_mm=result['state']['gripper_min_dist_mm'],
                    grips=[float(np.asarray(obs.state['joint_pos'])[6]), float(np.asarray(obs.state['joint_pos'])[13])],
                    max_speed=max(sp.values())))
                report['completed_steps'] = step
                save()
            fastest = max(sp.values())
            if not np.isfinite(fastest) or fastest > EXPLOSION_SPEED:
                report.update(status='unstable', unstable_at_step=step, speeds=sp)
                break
            if policy is not None and policy.done:
                report['ended'] = 'policy finished its final hold'
                break
            if time.monotonic() - t0 > args.wall_limit:
                report['ended'] = f'wall-clock limit {args.wall_limit:.0f} s at step {step}'
                break
        report['completed_steps'] = step
        if report['status'] == 'running':
            report['status'] = 'complete'
        for cam in CAMERAS:
            Image.fromarray(np.asarray(obs.images[cam], np.uint8)).save(args.out / f'final-{cam}.png')
        if policy is not None:
            report['demo'] = policy.report()
        final = dict(task='battery-pills', seed=args.seed, policy=args.policy, steps=step,
                     score=result['score'] if result else 0.0,
                     explanation=explain(result['conditions'], result['score']) if result else 'no step',
                     conditions=result['conditions'] if result else None, reading=result['state'] if result else None,
                     draw=report.get('draw'))
        report['score'] = final['score']
        np.savez_compressed(args.out / 'poses.npz', **{k: np.asarray(v) for k, v in poses_log.items()})
        final['poses_sha256'] = hashlib.sha256((args.out / 'poses.npz').read_bytes()).hexdigest()
        (args.out / 'score.json').write_text(json.dumps(encode(final), indent=2) + '\n')
    except BaseException as exc:
        import traceback
        report.update(status='interrupted' if isinstance(exc, SystemExit) else 'failed', error_type=type(exc).__name__,
                      error_message=str(exc), traceback=traceback.format_exc()[-4000:])
        raise
    finally:
        if dream is not None:
            try:
                from dream_policy import public_transcript
                (args.out / 'dream-transcript.json').write_text(json.dumps(encode(public_transcript(dream)), indent=2) + '\n')
            except Exception as error:  # noqa: BLE001
                report['dream_transcript_error'] = repr(error)
            report.update(dream_requests=len(request_audit), dream_session_audit=session_audit)
        save()
        if trace is not None:
            trace.close()
        for w in videos.values():
            w.close()
        save()
        if app is not None:
            app.close()
        signal.signal(signal.SIGTERM, previous)


if __name__ == '__main__':
    main()
