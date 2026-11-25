"""Scripted full-score demo (reads object poses from the simulator: the paper stack's strips and the printer's joints).

1. Take the paper (right arm): the gripper, its approach pitched 20 degrees down, opens 22 mm, goes over and under the
   stack's near edge where it hangs over the document cover, and closes on it.
2. Over the rear tray (right arm): up, then back over the printer high enough that the hanging stack clears it, and
   lined up from the stack's measured lower edge over the slot, level, its right edge 3 mm from the tray's right
   side (clear of the side's top edge).
3. Load (right arm): lowered to just above the housing, lined up again, then into the slot until its lower edge
   stands on the tray's floor; the held edge is carried back on an arc about the lower edge until the stack leans
   back 12 degrees, then down until the stack is on the floor again and along the slot until it is against the right
   side; let go; it falls back onto the raised paper support.
4. Guide (left arm): the closed fingertips go down beside the guide's raised grip and push it toward the paper until
   it is within 0.2 mm of the stack's left edge, and on until the stack is within 0.2 mm of the tray's right side;
   then lift.
5. OK (left arm): pressed once (until it is pushed in 1.1 mm); both arms go home.

    python demo.py task.py demo_out [--max-steps 2000] [--no-record] [--hold S] [--variant NAME]
                                    [--stop-after pinch|carry|load|guide]

Variants (scorer checks):
- ``guide_wide``: the guide is left at its widest;
- ``no_ok``: OK is not pressed;
- ``nothing``: the arms stay still.
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

TRACE = bool(os.environ.get("PC_TRACE"))
GAP_MM = 94.0                       # pads' gap at grip 1.0
TIP = 0.013                         # fingertips past the tool point
W = g.STRIP_W
PINCH_IN = 0.012                    # the pinch: the tool point this far in from the stack's edge, along it
PINCH_PITCH = math.radians(20.0)
PINCH_OPEN = 0.022
CARRY_H = 0.450                     # the tool point's height (above the table) while the stack hangs over the printer
HOLD_PITCH = math.radians(60.0)     # the hand pitched down over the rear tray
RIGHT_AIM = 0.003                   # the stack lowered with its right edge this far from the right side
FLOOR_Z = g.TOP + g.SLOT_FLOOR_H
SLOT_XC = (g.SLOT_X[0] + g.SLOT_X[1]) / 2
LOAD_X = g.SLOT_X[1] - 0.010                # the stack's lower edge goes down 10 mm in front of the slot's back
LEAN_BACK = math.radians(12.0)              # before letting go, the held edge is carried back on an arc to this lean


def grip_for(width_m: float) -> float:
    return max(0.0, min(1.0, width_m * 1000 / GAP_MM))


def R_from(a, c) -> np.ndarray:
    """Gripper rotation with approach ``a`` (its z) and closing axis ``c`` (its x)."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float)
    c = c - a * (a @ c)
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def down(yaw: float = 0.0, tilt: float = 0.0) -> np.ndarray:
    """Pointing down, tilted ``tilt`` toward +x (the approach leaning back), fingers closing along (cos yaw,
    sin yaw)."""
    a = np.array([math.sin(tilt), 0.0, -math.cos(tilt)])
    return R_from(a, [math.cos(yaw), math.sin(yaw), 0.0])


def pitched(deg_down: float) -> np.ndarray:
    """Approach +x pitched ``deg_down`` (rad) below level, fingers closing in the x-z plane."""
    a = np.array([math.cos(deg_down), 0.0, -math.sin(deg_down)])
    return R_from(a, [math.sin(deg_down), 0.0, math.cos(deg_down)])


def tcp_actual(arm):
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    from scenekit import arm_joints, env

    return ik.tcp_pose(arm_joints(env(), arm.side, ARM_JOINTS), arm.base)


def opening(arm) -> float:
    from scenekit import arm_joints, env

    j = arm_joints(env(), arm.side, ["joint7", "joint8"])
    return float(-(j[0] + j[1]))


def scene(ctx) -> dict:
    from scenekit import env

    return ctx.task.read_scene(env())


def strips(ctx):
    return scene(ctx)["strips"]


def support_deg(ctx) -> float:
    return math.degrees(scene(ctx)["joints"]["support"])


def sheet_pt(ctx, a, b, z=0.0):
    return ctx.task.sheet_point(strips(ctx), a, b, z)


def note(ctx, key, value):
    ctx.flags[key] = value
    if TRACE:
        print(f"NOTE {ctx.task._STATE['step']} {key} {json.dumps(value, default=str)}", flush=True)


def r3(v):
    return np.round(np.asarray(v, float), 4).tolist()


def settle_to(arm, pos, rot, tol=0.0006, rounds=3, seconds=0.6):
    """Move to ``pos`` and correct the command by the measured tool-point error."""
    target = np.asarray(pos, float)
    cmd = target.copy()
    yield from arm.move(cmd, rot, seconds)
    for _ in range(rounds):
        yield from wait(0.15)
        err = target - tcp_actual(arm)[0]
        if np.linalg.norm(err) < tol:
            break
        cmd = cmd + err
        yield from arm.move(cmd, rot, 0.25)


def path(arm, points, rots, seconds):
    """A smooth move through waypoints (each with its rotation), ``seconds`` in all."""
    from yam_arena import ik

    pts = [arm.pos.copy()] + [np.asarray(p, float) for p in points]
    rs = [arm.rot.copy()] + list(rots)
    seg = np.array([np.linalg.norm(pts[k + 1] - pts[k]) + 0.05 * np.linalg.norm(rs[k + 1] - rs[k])
                    for k in range(len(pts) - 1)])
    t = np.r_[0.0, np.cumsum(seg)] / max(seg.sum(), 1e-9)
    n = max(1, round(seconds * g.CONTROL_HZ))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        i = min(len(seg) - 1, int(np.searchsorted(t, s, side="right") - 1))
        u = (s - t[i]) / max(t[i + 1] - t[i], 1e-9)
        arm.aim(pts[i] + u * (pts[i + 1] - pts[i]), ik.slerp_matrix(rs[i], rs[i + 1], u))
        yield




# -- 1. take the paper ---------------------------------------------------------------------------------------------------
def pinch(ctx, arm, b=0.0):
    st = strips(ctx)
    c0, R0 = st[0]
    edge = c0 + R0 @ np.array([-W / 2, b, 0.0])
    d = R0[:, 0]
    p = edge + PINCH_IN * d
    rot = pitched(PINCH_PITCH)
    app = rot[:, 2]
    note(ctx, "pinch_target", dict(edge=r3(edge), p=r3(p)))
    yield from arm.move_joints(p - 0.06 * app + [0.0, 0.0, 0.03], rot, 2.2, grip=grip_for(PINCH_OPEN))
    yield from settle_to(arm, p - 0.02 * app, rot, rounds=2, seconds=0.6)
    yield from settle_to(arm, p, rot, rounds=2, seconds=0.6)
    yield from arm.set_grip(0.0, 0.6)
    yield from wait(0.3)
    note(ctx, "pinch_opening_mm", round(opening(arm) * 1000, 2))


def lower_edge(ctx):
    """The stack's lower short edge (three points: right end, middle, left end) as it hangs."""
    st = strips(ctx)
    pts = np.array([sheet_pt(ctx, g.SHEET_W, b) for b in (-g.SHEET_L / 2, 0.0, g.SHEET_L / 2)])
    return pts


def right_edge_y(ctx):
    P = ctx.task.stack_corners(strips(ctx))
    return float(P[:, 1].min())


def carry(ctx, arm):
    """Up, then back over the printer with the stack hanging clear of it, to over the rear tray."""
    p0 = arm.pos.copy()
    r1 = pitched(PINCH_PITCH)
    r2 = pitched(HOLD_PITCH)
    pts = [p0 + [0.0, 0.0, 0.06], np.array([p0[0] + 0.02, p0[1], g.TOP + CARRY_H]),
           np.array([0.22, p0[1], g.TOP + CARRY_H]), np.array([SLOT_XC - 0.075, p0[1], g.TOP + CARRY_H])]
    yield from path(arm, pts, [r1, r1, r2, r2], 5.0)
    yield from wait(1.0)
    note(ctx, "over_tray", dict(lower=r3(lower_edge(ctx).mean(0)), tcp=r3(tcp_actual(arm)[0])))


def line_up(ctx, arm, rounds, tol_y=0.0008):
    """Move the hanging stack (and turn the hand about x to level it) until its lower edge is over LOAD_X, level,
    with its right edge RIGHT_AIM from the tray's right side."""
    for _ in range(rounds):
        e = lower_edge(ctx)
        dx = LOAD_X - e.mean(0)[0]
        dy = (g.RIGHT_Y + RIGHT_AIM) - right_edge_y(ctx)
        dz = e[2][2] - e[0][2]                      # the edge's left end above its right end
        if abs(dx) < 0.0015 and abs(dy) < tol_y and abs(dz) < 0.0015:
            break
        ang = -math.atan2(dz, e[2][1] - e[0][1])
        roll = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(ang), -math.sin(ang)], [0.0, math.sin(ang), math.cos(ang)]])
        yield from arm.move(arm.pos + [dx, dy, 0.0], roll @ arm.rot, 0.6)
        yield from wait(0.8)


def lower_until(ctx, arm, z_stop, max_step):
    low = lambda: float(ctx.task.stack_corners(strips(ctx))[:, 2].min())   # noqa: E731
    for k in range(400):
        d = low() - z_stop
        if d < 0.0008:
            break
        arm.aim(arm.pos - [0.0, 0.0, min(max_step, max(0.0004, 0.25 * d))], arm.rot)
        yield
        if TRACE and k % 10 == 0:
            e = lower_edge(ctx)
            print(f"LOAD low {(low() - FLOOR_Z) * 1000:.1f} mm lower R {np.round(e[0], 4).tolist()} L "
                  f"{np.round(e[2], 4).tolist()} right_gap {(right_edge_y(ctx) - g.RIGHT_Y) * 1000:.2f}", flush=True)


def load(ctx, arm):
    """Line the hanging stack's lower edge up over the slot, lower it to just above the housing, line it up again,
    then lower it into the slot onto the floor and let go."""
    yield from line_up(ctx, arm, 6)
    yield from lower_until(ctx, arm, g.TOP + g.PR_H + 0.008, 0.0015)
    yield from wait(0.6)
    yield from line_up(ctx, arm, 5, tol_y=0.0005)
    e = lower_edge(ctx)
    note(ctx, "lined_up", dict(lower=r3(e.mean(0)), right_gap_mm=round((right_edge_y(ctx) - g.RIGHT_Y) * 1000, 2),
                               level_mm=round((e[2][2] - e[0][2]) * 1000, 2)))
    yield from lower_until(ctx, arm, FLOOR_Z, 0.0008)
    low = float(ctx.task.stack_corners(strips(ctx))[:, 2].min())
    note(ctx, "on_floor", dict(low_mm=round((low - FLOOR_Z) * 1000, 2), lower=r3(lower_edge(ctx).mean(0)),
                               right_gap_mm=round((right_edge_y(ctx) - g.RIGHT_Y) * 1000, 2)))
    # lean it back about its lower edge, so it falls back onto the support when let go
    from yam_arena import ik

    bottom = lower_edge(ctx).mean(0)
    v = arm.pos - bottom
    r, phi0 = math.hypot(v[0], v[2]), math.atan2(v[0], v[2])
    n = round(1.5 * g.CONTROL_HZ)
    for k in range(1, n + 1):
        phi = phi0 + (LEAN_BACK - phi0) * ik.smoothstep(k / n)
        arm.aim(np.array([bottom[0] + r * math.sin(phi), arm.pos[1], bottom[2] + r * math.cos(phi)]), arm.rot)
        yield
    # back down onto the floor, and along the slot against its right side
    yield from lower_until(ctx, arm, FLOOR_Z, 0.0005)
    for k in range(40):
        gap = right_edge_y(ctx) - g.RIGHT_Y
        if gap < 0.00015:
            break
        arm.aim(arm.pos - [0.0, min(0.0005, max(0.0001, 0.5 * gap)), 0.0], arm.rot)
        yield
    note(ctx, "leaned", dict(from_deg=round(math.degrees(phi0), 1), low_mm=round(
        (float(ctx.task.stack_corners(strips(ctx))[:, 2].min()) - FLOOR_Z) * 1000, 2),
        right_gap_mm=round((right_edge_y(ctx) - g.RIGHT_Y) * 1000, 2)))
    yield from arm.set_grip(grip_for(0.020), 0.4)
    yield from wait(0.6)
    yield from arm.move(arm.pos - 0.06 * arm.rot[:, 2] + [0.0, 0.0, 0.04], arm.rot, 0.8)
    yield from wait(0.6)
    note(ctx, "after_load", ctx.task.load_report(strips(ctx), scene(ctx)["joints"]["guide"], support_deg(ctx)))


# -- 4. guide ------------------------------------------------------------------------------------------------------------
GUIDE_YAW, GUIDE_TILT = math.radians(-20.0), math.radians(25.0)


def set_guide(ctx, arm, tol=0.0002):
    rot = down(GUIDE_YAW, GUIDE_TILT)               # leaning toward the printer, clear of the support behind
    a = rot[:, 2]
    gq = scene(ctx)["joints"]["guide"]
    face = g.GUIDE_OPEN_Y + gq
    tip = np.array([SLOT_XC, face + 0.032, g.TOP + g.GUIDE_H[1] - 0.007])   # beside the raised grip, clear of it
    yield from arm.move_joints(tip - TIP * a + [0.0, 0.0, 0.05], rot, 2.0, grip=0.0)
    yield from settle_to(arm, tip - TIP * a, rot, rounds=2, seconds=0.8)
    last, idle = gq, 0.0
    for k in range(500):
        gq_now = scene(ctx)["joints"]["guide"]
        rep = ctx.task.load_report(strips(ctx), gq_now, support_deg(ctx))
        gap, right = rep["guide_gap_mm"] / 1000, rep["right_gap_mm"] / 1000
        if gap < tol and right < tol:
            break
        if abs(gq_now - last) > 0.0001:
            last, idle = gq_now, 0.0
        elif idle > 0.010:                          # pushed 10 mm on without the guide moving: stop
            note(ctx, "guide_stalled", rep)
            break
        # close the guide; once against the stack, push it on (slowly) to the tray's right side
        step = min(0.0015, max(0.0002, 0.3 * gap)) if gap >= tol else min(0.0002, max(0.00005, 0.3 * right))
        idle += step
        arm.aim(arm.pos - [0.0, step, 0.0], arm.rot)
        yield
    yield from wait(0.3)
    yield from arm.move(arm.pos + [0.0, 0.002, 0.0], arm.rot, 0.3)       # stop pressing
    yield from arm.move(arm.pos + [-0.03, 0.0, 0.06], arm.rot, 0.6)
    yield from wait(0.3)
    note(ctx, "after_guide", ctx.task.load_report(strips(ctx), scene(ctx)["joints"]["guide"], support_deg(ctx)))


# -- 5. OK ----------------------------------------------------------------------------------------------------------------
def press_ok(ctx, arm, depth=0.0011):
    rot = down(0.0)
    top = np.array([g.OK_XY[0], g.OK_XY[1], g.TOP + g.BTN_TOP_H])
    yield from arm.move(top + [0.0, 0.0, TIP + 0.03], rot, 1.8, grip=0.0)
    yield from settle_to(arm, top + [0.0, 0.0, TIP + 0.004], rot, rounds=2, seconds=0.5)
    z = top[2] + TIP
    for _ in range(40):
        d = -scene(ctx)["joints"]["btn_ok"]
        if d >= depth:
            break
        z -= 0.0004
        arm.aim(np.array([top[0], top[1], z]), rot)
        yield
    note(ctx, "ok_depth_mm", round(-scene(ctx)["joints"]["btn_ok"] * 1000, 2))
    yield from arm.move(np.array([top[0], top[1], top[2] + TIP + 0.03]), rot, 0.5)


# -- the script -----------------------------------------------------------------------------------------------------------
def script(ctx, variant: str, stop_after: str | None):
    print(f"MODE variant={variant}", flush=True)
    L, Rr = ctx.left, ctx.right

    def done(name):
        note(ctx, f"phase_{name}_end", ctx.task._STATE["step"])
        return stop_after == name

    yield from pinch(ctx, Rr)
    if done("pinch"):
        return
    yield from carry(ctx, Rr)
    if done("carry"):
        return
    yield from load(ctx, Rr)
    if done("load"):
        return
    if variant != "guide_wide":
        yield from together(set_guide(ctx, L), Rr.go_home(2.5))
    else:
        yield from Rr.go_home(2.5)
    if done("guide"):
        return
    if variant != "no_ok":
        yield from press_ok(ctx, L)
    yield from together(L.go_home(2.0), wait(0.1))
    ctx.flags["script_end_step"] = ctx.task._STATE["step"]
    print("FLAGS", json.dumps(ctx.flags, default=str), flush=True)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    return (f"support {S.get('support_deg')} hinge_max {S.get('max_hinge_deg', 0):.1f} rep {S.get('rep')} "
            f"ok_after {int(S.get('ok_after', 0))} gap {S.get('gap', 1) * 1000:.1f} released {int(S.get('released', 0))} "
            f"tier {S.get('tier')}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--hold", type=float, default=3.0, help="seconds after the script ends")
    ap.add_argument("--stop-after", default=None)
    ap.add_argument("--variant", default="full", choices=["full", "guide_wide", "no_ok", "nothing"])
    a = ap.parse_args()
    if a.variant == "nothing":
        def scr(ctx):
            print("MODE variant=nothing", flush=True)
            yield from wait(5.0)
    else:
        def scr(ctx):
            ctx.flags["out"] = a.out
            yield from script(ctx, a.variant, a.stop_after)
    res = run(a.task, a.out, scr, max_steps=a.max_steps, record=not a.no_record, log=log,
              log_every=int(os.environ.get("PC_LOG_EVERY", "20")), hold_s=a.hold)
    S = sys.modules["task"]._STATE if "task" in sys.modules else {}
    extra = {"variant": a.variant, "tier": S.get("tier"), "rep": S.get("rep"), "ok_after": S.get("ok_after"),
             "folded": S.get("folded"), "max_hinge_deg": S.get("max_hinge_deg"), "released": S.get("released"),
             "first_full": S.get("first_full"), "events": S.get("events"), "steps": S.get("step"),
             "support_deg": S.get("support_deg")}
    print("EXTRA " + json.dumps(extra, default=str), flush=True)
    (pathlib.Path(a.out) / "extra.json").write_text(json.dumps(extra, indent=2, default=str))
