"""Scripted full-score demo (reads the simulator's state: the handles' joint angles).

Both hands work with their fingers closed, the closing axis vertical: one blunt finger 15 mm thick.
1. Right hand, space 2 (directly above the tagged handle): the fingers come in on the handle's outboard side, 2 mm
   above its centre (so 8.5 mm clear of the tagged handle below) and 5.5 mm short of the dead front, and push it toward
   the panel's middle until it passes midway (read from its joint angle). The arm gives under the handle's spring,
   so the push is commanded well past the handle; the moment it passes midway the command is stopped where the
   fingers are. They back off, come out and the arm goes back to the ready pose round the open door.
2. Left hand, space 3: the same push from its outboard side; back to the ready pose.
Space 4 and its tag are never touched.

    python demo.py task.py demo_out [--max-steps 2000] [--no-record] [--variant NAME] [--close-view S] [--hold S]

Variants (scorer checks), each printing its mode:
- ``catch4``: space 2 pushed with the fingers 16 mm lower, catching the top of the tagged handle and its tag;
- ``between``: space 2 pushed with the fingers 12 mm lower, centred on the gap between space 2 and the tagged handle;
- ``all_on``: the demo, plus space 4 pushed ON;
- ``one``: only space 3;
- ``keep_hold``: the demo, then the right fingers come back and rest by space 2's handle;
- ``furnace_off``: the demo, plus space 1 pushed OFF;
- ``nothing``: the arms stay still.
Writes one MP4 per camera, setup.png (the first top-camera frame), result.json and extra.json; with --close-view S,
S seconds after the arms are at ready the top camera moves to a close view of the handles and the tag, then to the
trial camera's pose, 3 s each (close_view.json gives the steps).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, together, wait  # noqa: E402

TRACE = bool(os.environ.get("BT_TRACE"))
T = g.TOP
TIP = 0.013                         # fingertips past the tool point
HALF = 0.0075                       # the closed fingers' half thickness near the tip
TIP_X = g.FRONT_X - 0.0055          # the fingertip this far in when pushing (5.5 mm short of the dead front)
PUSH_DZ = 0.0                       # the fingers' centre on the handle's centre
PUSH2_DZ = 0.002                    # space 2: 2 mm above it (further from the tagged handle below)
PUSH_SPEED = float(os.environ.get("BT_PUSH_SPEED", "0.0006"))   # m per control step along the push


def R_from(a, c) -> np.ndarray:
    """Gripper rotation with approach ``a`` (its z) and closing axis ``c`` (its x)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def flat(yaw: float = 0.0) -> np.ndarray:
    """Horizontal approach turned ``yaw`` from +x, the closing axis vertical."""
    return R_from([math.cos(yaw), math.sin(yaw), 0.0], [0.0, 0.0, 1.0])


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


class Scene:
    """The measured panel: handle angles (rad) and the tag's pose."""

    def __init__(self, ctx):
        ps = ctx.task.panel_state(ctx.emb._env)
        self.q = ps["q"]
        self.tie = ps["tie"]
        self.hole = np.asarray(ps["tag_p"], float)
        self.R = np.asarray(ps["tag_R"], float)

    def lift(self) -> float:
        d = -self.R[:, 2]
        return math.atan2(-d[0], -d[2])


RIGHT_CLEAR = np.array([0.200, -0.280, T + 0.260])     # the right hand's way round the open door


def to_work(ctx, arm, p, rot, seconds=2.0):
    """From wherever the hand is to ``p``: straight there when the hand is within 0.25 m, else in joint space (the
    right hand first comes round the open door)."""
    here, _ = tcp_actual(arm)
    if arm.side == "right" and here[1] < -0.15:
        if np.linalg.norm(here - RIGHT_CLEAR) > 0.02:
            yield from arm.move(RIGHT_CLEAR, rot, 1.5)
        yield from arm.move(np.array([0.22, p[1], p[2]]), rot, 1.5)
        yield from arm.move(p, rot, 1.0)
    elif np.linalg.norm(here - p) < 0.25 and here[0] < 0.30:
        arm.pos = here
        yield from arm.move(p, rot, max(0.8, seconds * 0.6))
    else:
        yield from arm.move_joints(p, rot, seconds, grip=0.0)


def go_ready(ctx, arm, seconds=2.5):
    """Out from the panel along -x, then to the ready pose (the right hand round the open door)."""
    here, _ = tcp_actual(arm)
    arm.pos = here
    if here[0] > 0.24:
        yield from arm.move(np.array([0.22, here[1], here[2]]), arm.rot, 0.8)
    if arm.side == "right":
        yield from arm.move(RIGHT_CLEAR, arm.rot, 1.5)
    yield from arm.go_home(seconds)


def settle_to(arm, pos, rot, tol=0.0006, rounds=4, seconds=0.6):
    """Move to ``pos`` and correct the command by the measured tool-point error (the arm sags under gravity)."""
    target = np.asarray(pos, float)
    cmd = target + getattr(arm, "corr", np.zeros(3))
    yield from arm.move(cmd, rot, seconds)
    for _ in range(rounds):
        yield from wait(0.2)
        err = target - tcp_actual(arm)[0]
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
        yield from arm.move(cmd, rot, 0.3)
    arm.corr = cmd - target


def track(arm, target, rot, gain=0.25, mask=(1.0, 1.0, 1.0), limit=0.03):
    """One control step toward ``target`` with an integral correction of the measured tool-point error."""
    corr = getattr(arm, "corr", np.zeros(3))
    err = (np.asarray(target, float) - tcp_actual(arm)[0]) * np.asarray(mask)
    base = getattr(arm, "corr0", corr)
    corr = np.clip(corr + gain * err, base - limit, base + limit)
    arm.corr = corr
    arm.aim(np.asarray(target, float) + corr, rot)


def note(ctx, key, value):
    ctx.flags.setdefault(key, []).append(value)
    if TRACE:
        print(f"NOTE {key} {value}", flush=True)


# -- pushing a handle -----------------------------------------------------------------------------------------------------
def push(ctx, arm, n: int, dz: float = PUSH_DZ, yaw: float = 0.0, label: str = "", to_on: bool = True):
    """Push space n's handle over with the closed fingers: in on its far side, then along y until it is past midway
    (``to_on``: toward the middle, ON; else away from it, OFF), read from its joint angle; then back off and out."""
    col, _, yc, zc = g.space(n)
    s = 1.0 if col == "L" else -1.0                     # the column's side; ON is toward -s * y
    u = -s if to_on else s                              # the push direction along y
    q0 = Scene(ctx).q[n]
    tip0 = g.handle_tip(n, q0)
    half_w = 0.013 / max(math.cos(yaw), 0.5)            # the fingers' half width across the push
    y_start = tip0[1] - u * (g.HANDLE_T / 2 + 0.003 + half_w + 0.004)
    rot = flat(u * yaw)
    a = rot[:, 2]
    z = T + zc + dz
    tip_x = TIP_X - (0.013 * math.sin(abs(yaw)) if yaw else 0.0)   # a turned tip edge kept off the dead front
    start = np.array([tip_x, y_start, z]) - TIP * a
    arm.grip = 0.0
    arm.corr = np.zeros(3)
    yield from to_work(ctx, arm, start - 0.06 * a, rot)
    yield from settle_to(arm, start - 0.06 * a, rot, rounds=3)
    yield from settle_to(arm, start, rot, rounds=3)
    arm.corr0 = arm.corr.copy()
    passed = False
    goal = start.copy()
    for k in range(int(os.environ.get("BT_PUSH_STEPS", "200"))):   # the arm gives about 8 mm/N along the push
        q = Scene(ctx).q[n]
        if (to_on and q > 0.0) or (not to_on and q < 0.0):  # past midway: its spring takes it to the end
            passed = True
            break
        goal = goal + np.array([0.0, u * PUSH_SPEED, 0.0])
        track(arm, goal, rot, gain=0.1, mask=(1.0, 0.0, 1.0), limit=0.004)   # height and depth held near the settle
        dz_now = tcp_actual(arm)[0][2] - goal[2]
        if abs(dz_now) > 0.004:                         # riding up or down the handle: stop
            note(ctx, f"push{label or n}_slip", dict(dz_mm=round(dz_now * 1000, 1), k=k))
            break
        yield
    cmd = goal
    p_act, _ = tcp_actual(arm)
    if passed:                                          # stop where the finger is: the arm's stored give is let go
        arm.corr = np.zeros(3)
        arm.aim(p_act + np.array([0.0, u * 0.004, 0.0]), rot)
        yield
    note(ctx, f"push{label or n}", dict(passed=passed, cmd_mm=round(abs(cmd[1] - start[1]) * 1000, 1),
                                        lag_mm=round((cmd[1] - p_act[1]) * u * 1000, 1),
                                        q_deg=round(math.degrees(Scene(ctx).q[n]), 1)))
    yield from wait(0.4)
    back = arm.pos - np.array([0.0, u * 0.012, 0.0])
    arm.corr = np.zeros(3)
    if hasattr(arm, "corr0"):
        del arm.corr0
    yield from arm.move(back, rot, 0.5)
    yield from arm.move(back - 0.05 * a, rot, 0.8)


# -- the scripts ---------------------------------------------------------------------------------------------------------
def script(ctx, variant: str):
    print(f"MODE variant={variant}", flush=True)
    L, Rr = ctx.left, ctx.right
    if variant == "one":                                # space 3 only
        yield from push(ctx, L, 3)
        yield from go_ready(ctx, L)
    elif variant != "nothing":
        # space 2, directly above the tagged handle: the right hand, its fingers 2 mm above the handle's centre
        dz2 = PUSH2_DZ - {"catch4": 0.016, "between": 0.012}.get(variant, 0.0)
        yield from push(ctx, Rr, 2, dz=dz2)
        if variant == "all_on":
            yield from push(ctx, Rr, 4, dz=0.004, label="4")
        yield from go_ready(ctx, Rr)
        # space 3: the left hand
        yield from push(ctx, L, 3)
        if variant == "furnace_off":
            yield from push(ctx, L, 1, to_on=False, label="1_off")
        yield from go_ready(ctx, L)
        if variant == "keep_hold":                      # the right fingers come back and rest by space 2's handle
            col, _, yc, zc = g.space(2)
            tip = g.handle_tip(2, Scene(ctx).q[2])
            rot = flat(0.0)
            p = np.array([TIP_X - 0.010, tip[1] - 0.022, T + zc + PUSH2_DZ]) - TIP * rot[:, 2]
            Rr.grip = 0.0
            yield from to_work(ctx, Rr, p - np.array([0.05, 0.0, 0.0]), rot)
            yield from settle_to(Rr, p, rot, rounds=3)
    ctx.flags["script_end_step"] = ctx.task._STATE["step"]
    print("FLAGS", json.dumps(ctx.flags, default=str), flush=True)
    yield from wait(1.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    on = "".join("1" if S.get("on", {}).get(n) else "0" for n in range(1, 9))
    return (f"on {on} q {S.get('q')} loop {S.get('loop_mm')} tag {S.get('tag')} gap {S.get('gap')} ({S.get('gap_to')}) "
            f"released {int(S.get('released', 0))} went_on4 {int(S.get('went_on4', 0))} tag_off "
            f"{int(S.get('tag_off', 0))} tier {S.get('tier')} safe {S.get('safe')}")


VIEWS = {"close": ((0.17, -0.02, 0.27), (0.322, 0.045, 0.175)), "trial": (g.TRIAL_CAM, g.TRIAL_AIM)}


def close_view(ctx, gen, end_s, view_s=3.0):
    """Recorded runs: ``end_s`` after the script ends (the arms at ready), the top camera moves to a close view of the
    handles and the tag, then to the trial camera's pose, ``view_s`` each, so the end state can be read. Only the
    recording changes."""
    import torch

    base = ctx.emb._env.unwrapped
    cam = base.scene["top_cam"]
    o = base.scene.env_origins[0:1].to(torch.float32)
    for _ in gen:
        yield
    for _ in wait(end_s):
        yield
    steps = {}
    for name, (eye, tgt) in VIEWS.items():
        e = np.array(eye) + [0, 0, T]
        t = np.array(tgt) + [0, 0, T]
        cam.set_world_poses_from_view(torch.tensor(np.array([e]), dtype=torch.float32, device=o.device) + o,
                                      torch.tensor(np.array([t]), dtype=torch.float32, device=o.device) + o)
        steps[name] = ctx.task._STATE["step"] + 1
        print(f"top camera: {name} view from step {steps[name]}", flush=True)
        yield from wait(view_s)
    (pathlib.Path(ctx.flags["out"]) / "close_view.json").write_text(json.dumps(steps))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--close-view", type=float, default=None, metavar="S",
                    help="recorded runs: S seconds after the arms are at ready, move the top camera to close views")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds after the script ends (inf: to the step limit)")
    ap.add_argument("--variant", default="full",
                    choices=["full", "catch4", "between", "all_on", "one", "keep_hold", "furnace_off", "nothing"])
    a = ap.parse_args()
    if a.variant == "nothing":
        def scr(ctx):
            print("MODE variant=nothing", flush=True)
            ctx.flags["out"] = a.out
            if a.close_view is not None and not a.no_record:
                yield from close_view(ctx, wait(5.0), a.close_view)
            else:
                yield from wait(5.0)
    else:
        def scr(ctx):
            ctx.flags["out"] = a.out
            if a.close_view is not None and not a.no_record:
                yield from close_view(ctx, script(ctx, a.variant), a.close_view)
            else:
                yield from script(ctx, a.variant)
    res = run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
              log_every=int(os.environ.get("BT_LOG_EVERY", "20")), hold_s=a.hold)
    S = sys.modules["task"]._STATE if "task" in sys.modules else {}
    extra = {"variant": a.variant, "tier": S.get("tier"), "safety_flag": S.get("safe"), "on": S.get("on"),
             "q_deg": S.get("q"), "went_on4": S.get("went_on4"), "tag_off": S.get("tag_off"),
             "q4_max_deg": round(math.degrees(S.get("q4_max", -9.0)), 2), "loop_max_mm": round(S.get("loop_max", 0) * 1000, 2),
             "tag": S.get("tag"), "gap_mm": S.get("gap"), "gap_to": S.get("gap_to"), "released": S.get("released"),
             "first_full": S.get("first_full"), "events": S.get("events"), "steps": S.get("step")}
    print("EXTRA " + json.dumps(extra, default=str), flush=True)
    (pathlib.Path(a.out) / "extra.json").write_text(json.dumps(extra, indent=2, default=str))
