"""Physical YAM monitor lift/tilt/pivot demo; robot joint controls only."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20
GRIP_X = -0.035                 # along the handles from the pivot, where the fingers squeeze (pliers frame, m)
GRIP_Y = 0.00675                 # midway between the open handles' outer faces at the fingers (pliers frame, m)
OPEN_WIDTH = 0.0495              # across the open handles' outer faces at the fingers (m)
HOLD_WIDTH = 0.040              # gripper opening while carrying: just enough squeeze to hold them
SHUT_WIDTH = 0.036             # across the shut handles' outer faces
TUBE_IN_JAWS = np.array([0.032, -0.004])   # tube axis in the pliers frame when the jaws are slid on (open, 2 mm off the pad)
TUBE_LOCKED_Y = 0.002           # tube axis in the pliers frame once squeezed shut (pad pressed 4 mm)
LEAD = 0.045                    # how far short of the tube the jaws start their slide (m)


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0

    def _solve(self, pos, rot):
        from yam_arena import ik

        from scipy.optimize import least_squares
        def residual(q):
            p,R=ik.tcp_pose(q,self.base)
            return np.r_[p-np.asarray(pos), .3*ik._rot_error(rot,R)]
        fit=least_squares(residual,np.clip(self.q,ik.LOWER+1e-7,ik.UPPER-1e-7),
                          bounds=(ik.LOWER,ik.UPPER),max_nfev=80,
                          ftol=1e-8,xtol=1e-8,gtol=1e-8)
        q=fit.x
        pe=float(np.linalg.norm(ik.tcp_pose(q,self.base)[0]-pos))
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        before = self.worst_mm
        self.worst_mm = 0.0
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self._solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield
        if self.worst_mm > 1.0:
            print(f"IK {self.name}: {self.worst_mm:.1f} mm on the move to {np.round(pos, 3).tolist()}", flush=True)
        self.worst_mm = max(before, self.worst_mm)

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip)


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def width(w: float) -> float:
    """Gripper command for a finger opening of ``w`` m (1 = 94 mm open)."""
    return float(np.clip(w / 0.094, 0.0, 1.0))


def _rz(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _grip_yaw(points, base) -> float:
    """0 or pi: the turn of the straight-down grip that IK reaches best at the given TCP points."""
    from yam_arena import ik
    from yam_arena.constants import READY
    from yam_arena.demo import grip_rotation

    worst = {}
    for flip in (0.0, math.pi):
        errs = [ik.solve(p, ik.matrix_to_quat_xyzw(grip_rotation(flip)), np.array(READY, float), base_pos=base,
                         iters=200)[1] for p in points]
        worst[flip] = max(errs)
    return min(worst, key=worst.get)


def script(task, ctx):
    T=task; A=ctx.right
    variant=os.environ.get("DEMO_VARIANT", "")
    if variant == "idle":
        yield from wait(5)
        return
    local=np.array([0., -.160, .160])
    grip_R=np.diag([1.,-1.,-1.])
    state=ctx.states()["monitor"]
    centre,R=T.panel_pose(state)
    p=centre+R@local
    yield from A.move(p + [0,0,.10],grip_R,2.5)
    yield from A.move(p,grip_R,2.0)
    yield from A.set_grip(0.0,1.0)
    yield from wait(.5)
    base=state.pos.copy()
    def pose(lift,tilt,pivot):
        M=T.ry(tilt)@T.rx(pivot)
        c=base+T.PIVOT+[0,0,lift]+M@T.PANEL_CENTRE
        return c+M@local, M@grip_R
    def joint_move(a,b,duration):
        from yam_arena import ik
        for k in range(1,round(duration*CONTROL_HZ)+1):
            s=ik.smoothstep(k/round(duration*CONTROL_HZ))
            values=np.asarray(a)+(np.asarray(b)-a)*s
            p,r=pose(*values)
            A._solve(p,r)
            yield
    lift=.128 if variant!="low_rotate" else 0.
    yield from joint_move((0,0,0),(lift,0,0),3.)
    yield from joint_move((lift,0,0),(lift,math.radians(21),0),2.5)
    pivot=0. if variant=="no_pivot" else math.pi/2
    if pivot and variant != "low_rotate":
        # Turn halfway, release on the passive friction hinge, then regrasp a
        # higher point of the same upper bezel. This keeps the wrist clear of
        # the physical robot pedestal during the second half of the turn.
        yield from joint_move((lift,math.radians(21),0),(lift,math.radians(21),math.pi/4),3.)
        yield from A.set_grip(1.,.8)
        actual=ctx.states()["monitor"]
        centre,R=T.panel_pose(actual)
        retreat=A.pos + R @ np.array([0,0,.07])
        yield from A.move(retreat,R @ grip_R,1.5)
        local=np.array([0.,.080,.160])
        actual=ctx.states()["monitor"]
        centre,R=T.panel_pose(actual)
        p=centre+R@local
        yield from A.move(p+R@np.array([0,0,.07]),R@grip_R,2.)
        yield from A.move(p,R@grip_R,1.5)
        yield from A.set_grip(0.,.8)
        yield from wait(.5)
        actual=ctx.states()["monitor"]
        base=actual.pos.copy()
        begin=(actual.sliders['lift'],actual.hinges['tilt'],actual.hinges['pivot'])
        yield from joint_move(begin,(lift,math.radians(21),pivot),4.)
    else:
        yield from joint_move((lift,math.radians(21),0),(lift,math.radians(21),pivot),5.)
    yield from joint_move((lift,math.radians(21),pivot),(lift,0,pivot),2.5)
    end_lift=T.TARGET_CENTRE-(T.START[2]-T.TOP+T.PIVOT[2]) if variant!="low_rotate" else 0.
    yield from joint_move((lift,0,pivot),(end_lift,0,pivot),2.)
    if variant not in ("no_pivot", "low_rotate"):
        # Visual pose feedback compensates real robot/grasp compliance. This
        # only changes robot joint commands; passive monitor joints stay passive.
        from yam_arena import ik
        good=0
        for _ in range(300):
            obj=ctx.states()["monitor"]
            centre,R=T.panel_pose(obj)
            goal_R=obj.rotation() @ T.rx(math.pi/2)
            goal_c=obj.pos + obj.rotation() @ (T.PIVOT+[0,0,end_lift]) + goal_R@T.PANEL_CENTRE
            hand_p,hand_R=ctx.actual_hand("right")
            delta_R=goal_R@R.T
            desired_hand=goal_c+delta_R@(hand_p-centre)
            dp=.08*(desired_hand-hand_p)
            dp*=min(1.,.001/max(float(np.linalg.norm(dp)),1e-9))
            next_p=A.pos+dp
            next_R=ik.slerp_matrix(np.eye(3),delta_R,.08)@A.rot
            A._solve(next_p,next_R)
            m=T.measure(obj)
            ok=(abs(m['pivot']-90)<1 and abs(m['tilt'])<1 and abs(m['swivel'])<1
                and abs(m['height']-T.TARGET_CENTRE)<.004)
            good=good+1 if ok else 0
            yield
            if good>=10:break
        print("feedback_end", json.dumps(T.measure(ctx.states()["monitor"])),flush=True)
        # Remove command preload before opening, so wrist spring-back cannot
        # drag the already positioned monitor as the fingers release.
        hand_p,hand_R=ctx.actual_hand("right")
        A._solve(hand_p,hand_R)
    if variant=="held":
        yield from wait(4)
        return
    yield from A.set_grip(1.,1.)
    obj=ctx.states()["monitor"]
    centre,R=T.panel_pose(obj)
    hand_p,hand_R=ctx.actual_hand("right")
    yield from A.move(hand_p+R@np.array([0,0,.08]),T.rx(math.radians(60)) @ grip_R,2.)
    yield from wait(4)


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", np.asarray(station.BASES["left"])+[0,0,.35])
        self.right = Arm("right", np.asarray(station.BASES["right"])+[0,0,.35])

    def actual_hand(self,name):
        from partkit import _np,_quat_matrix
        from yam_arena.constants import TCP_OFFSET
        scene=self.emb._env.unwrapped.scene
        robot=scene[name+"_robot"]
        p=_np(robot.data.body_link_pose_w)[0][list(robot.body_names).index("gripper")]
        x,y,z,w=p[3:7]
        R=_quat_matrix((w,x,y,z))
        return p[:3]+R@np.asarray(TCP_OFFSET)-_np(scene.env_origins)[0],R

    def states(self):
        from yam_arena.scene import object_states

        return object_states(self.emb._env, self.emb.task.names)

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        return Action(data=pack(self.left.cmd, self.right.cmd))


def _write_mp4(frames, path: Path) -> None:
    import imageio

    imageio.mimsave(path, [np.asarray(f) for f in frames], fps=CONTROL_HZ, codec="libx264",
                    output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])


def run(task_path: str, out_dir: str, max_steps: int = 1200, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 4.0) -> dict:
    from types import SimpleNamespace

    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from yam_arena.scoring import task_score

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height, device=os.environ.get("DEMO_DEVICE", "cpu"))
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb)
    asset=emb._env.unwrapped.scene["monitor"]
    try:
        props=asset.root_view.get_dof_friction_properties()
        props=getattr(props,"torch",props)
        if hasattr(props,"detach"):props=props.detach().cpu().numpy()
        (out/"joint-friction.json").write_text(json.dumps({"names":list(asset.joint_names),"friction":np.asarray(props).tolist()},indent=2))
    except Exception as error:
        (out/"joint-friction-error.txt").write_text(str(error))
    gen = script(task, ctx)
    frames: dict[str, list] = {}
    trace = (out / "trace.jsonl").open("w", buffering=1)

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended = 0, None
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        snap(res.observation)
        p = ctx.states()["monitor"]
        measure = task.measure(p)
        trace.write(json.dumps(dict(step=steps, score=res.info.get("score"), pos=p.pos.tolist(),
            quat=p.quat_xyzw.tolist(), hinges=p.hinges, sliders=p.sliders,
            measure={k: (v.tolist() if isinstance(v, np.ndarray) else v) for k,v in measure.items()},
            tcp=ctx.right.pos.tolist(), command=ctx.right.cmd.tolist(),
            actual_tips=[p.tolist() for p in task._STATE["tips"]], left_command=ctx.left.cmd.tolist())) + "\n")
        if steps % 50 == 0:
            p = ctx.states()["monitor"]
            print(f"step {steps} score {res.info.get('score')} pliers_mm {np.round(p.pos * 1000, 1).tolist()}",
                  flush=True)
        if res.terminated:
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    trace.close()
    scored = task_score()(SimpleNamespace(steps=[SimpleNamespace(result=res)]), None)
    print(f"task_score: {scored.value:g} ({scored.explanation}) after {steps} steps", flush=True)
    result = {"task_score": scored.value, "explanation": scored.explanation, "steps": steps,
              "variant": os.environ.get("DEMO_VARIANT", ""),
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
    if record:
        import imageio

        cams = sorted(frames, key=lambda c: ("left" not in c, "top" not in c))   # left, top, right
        _write_mp4([np.hstack([frames[c][i] for c in cams]) for i in range(len(frames[cams[0]]))], out / "demo.mp4")
        imageio.imwrite(out / "setup.png", np.hstack([frames[c][0] for c in cams]))
    if os.environ.get("DEMO_RESET_CHECK") == "1":
        reset_obs = emb.reset(Scene(id="reset-check", instruction=task.INSTRUCTION))
        reset_ctx = Ctx(emb)
        fresh = reset_ctx.states()["monitor"]
        reset_record = dict(pos=fresh.pos.tolist(), hinges=fresh.hinges, sliders=fresh.sliders,
                            recent_samples=len(task._STATE["recent"]), step=task._STATE["step"])
        for _ in range(20):
            reset_result = emb.step(reset_ctx.action())
        fresh = reset_ctx.states()["monitor"]
        reset_record.update(after20_pos=fresh.pos.tolist(), after20_hinges=fresh.hinges,
                            score=reset_result.info.get("score"), success=bool(reset_result.terminated))
        reset_record["passed"] = bool(
            np.linalg.norm(fresh.pos - np.asarray(task.START)) < 0.002
            and all(abs(x)<.01 for x in fresh.hinges.values())
            and abs(fresh.sliders["lift"])<.002
            and reset_record["step"] == 0 and reset_record["recent_samples"] == 0
            and reset_record["score"] == 0.0 and not reset_record["success"])
        (out / "reset-check.json").write_text(json.dumps(reset_record, indent=2))
        if not reset_record["passed"]:
            raise RuntimeError("Same-process reset check failed; see reset-check.json")
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
