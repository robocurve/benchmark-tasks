"""The scripted demo for pry-board-off-joists, run by demokit.py through task.demo_script.

Closed loop: every stage reads where the bar and the board are now (their poses, as a policy's perception could), never
the scorer's state or the nails' rule. Nothing is teleported or edited.

1. The right arm picks the bar up by its shaft, turns it blade-forward and lays the blade flat on the right joist's
   top, its edge in the V the board's eased bottom edge leaves; the bend (the heel) rests on the joist.
2. It levers: pushing the handle down rocks the bar on its heel and the blade lifts the board's edge. Each stroke is
   commanded a little at a time until the board is seen to rise; between strokes the blade is pushed further under.
   It stops once the board stands 14 mm off the joist at the nail (the nail is 12.7 mm in).
3. It sets the bar down in the middle; the left arm picks it up and does the same at the left joist.
4. With the board loose, the left hand keeps the bar where it is while the right hand grips the board inside the bay,
   lifts it straight up off the joists and the blade, carries it toward the robot and lays it flat on the table, to
   the right of the bar's handle; it lets go. Then the left arm draws the bar out (nothing on it now) and puts it down
   on the table outside the left joist.
"""

from __future__ import annotations

import math

import numpy as np
from yam_arena import ik
from yam_arena.urdf import axis_angle_matrix

import layout as L
import scoring as S
from demokit import DT, _as_grip, down

MM = L.MM
R_TILT = np.array(L.bar_frame_rotation())
PAD = 0.013  # the pad tips are 13 mm below the TCP along the grip's axis
FRONT_X = L.BOARD_X - L.BOARD_W / 2  # the board's front face
HALF_T = {"tip": 0.8 * MM}  # half the blade's thickness at its edge
GAP_DONE = 14.0 * MM  # the board this far off the joist at the nail: the nail (12.7 mm in) is out
PARK = {"right": (0.15, -0.24, L.TOP + 0.20), "left": (0.15, 0.24, L.TOP + 0.20)}
BAR_DOWN_TIP = (0.40, 0.34, L.TOP + 0.8 * MM + 1.0 * MM)  # where the bar's edge is put down at the end
BOARD_DOWN_Y = -0.14  # ...and along y: to the right, clear of the bar's handle at the left joist
BOARD_DOWN_X = 0.115  # where the board is laid on the table: its centre, in front of the joists and the bar's claw
FIRST_LIFT = 4.0 * MM  # the first stroke, with the edge only in the V: lift this much, then set the blade deeper


# -- reading the scene ------------------------------------------------------------------------------------------------
def bar_world(ctx, name: str) -> np.ndarray:
    """A point on the bar's centre line ("tip", "heel", "grip") where the bar is now."""
    s = ctx.state("pry_bar")
    return np.asarray(s.to_world(R_TILT @ np.asarray(L.bar_point(name))), float)


def gap(ctx, side: str) -> float:
    b = ctx.state("board")
    return S.nail_gap(b.rotation(), b.origin, side)


def board_point(ctx, local) -> np.ndarray:
    b = ctx.state("board")
    return S.board_to_world(b.rotation(), b.origin, local)


# -- moves --------------------------------------------------------------------------------------------------------------
def turn_by(arm, deg: float, axis, t: float | None = None, label: str = "turn"):
    if abs(deg) > 0.2:
        yield from arm.turn(deg, axis=axis, t=t, label=label)


def report(ctx, arm, label: str) -> None:
    rot = ik.tcp_pose(arm.q, arm.base)[1]
    print(f"[demo] {label}: tcp cmd {np.round(arm.at(), 3)} seen {np.round(arm.tcp(), 3)} approach "
          f"{np.round(rot[:, 2], 2)} closing {np.round(rot[:, 1], 2)} tip {np.round(bar_world(ctx, 'tip'), 3)} "
          f"heel {np.round(bar_world(ctx, 'heel'), 3)} grip {np.round(bar_world(ctx, 'grip'), 3)}", flush=True)


def level_blade(ctx, arm, nose_down: float = 0.0, heading: float = 0.0):
    """Turn the held bar so its blade points along ``heading`` (degrees from +x: 0 is forward, tip toward the board),
    level or tipped ``nose_down`` degrees."""
    report(ctx, arm, "before levelling")
    h = math.radians(heading)
    d = bar_world(ctx, "heel") - bar_world(ctx, "tip")  # from the tip back toward the hand: wanted along heading+180
    yaw_err = math.degrees(math.atan2(d[1], d[0]))
    yield from turn_by(arm, (heading + 180.0 - yaw_err + 180.0) % 360.0 - 180.0, (0.0, 0.0, 1.0),
                       label="aim the blade")
    d = bar_world(ctx, "heel") - bar_world(ctx, "tip")
    c, s = -(d[0] * math.cos(h) + d[1] * math.sin(h)), d[2]  # back along the heading, and up: angle atan2(s, c)
    yield from turn_by(arm, nose_down - math.degrees(math.atan2(s, c)), (-math.sin(h), math.cos(h), 0.0),
                       label="lay the blade level")
    yield from arm.wait(0.5)
    report(ctx, arm, "levelled")


def bring_tip(ctx, arm, goal, t: float = 1.5, label: str = "tip"):
    """Move the held bar so its tip reaches ``goal``, then close the last millimetres on the tip's measured place."""
    shift = np.asarray(goal, float) - bar_world(ctx, "tip")
    yield from arm.move(arm.at() + shift, t=t, label=label)
    yield from arm.servo(goal, tol=0.001, timeout=1.5, read=lambda: bar_world(ctx, "tip"), label=label)


def lever(ctx, arm, side: str, want: float, max_deg: float = 40.0, rate: float = 5.0):
    """Rock the bar on its heel, pushing the handle down, a little each step, until the board stands ``want`` off the
    joist at the nail (or ``max_deg`` is reached). The arm's joints are soft, so the command runs well past the bar's
    geometric angle; the board's measured rise decides when to stop. Returns the angle commanded."""
    heel = bar_world(ctx, "heel")  # the bend's underside, on the joist
    axis = np.array([0.0, 1.0, 0.0])  # the shaft runs back toward -x from the heel: handle down is -deg about +y
    p0, r0 = ik.tcp_pose(arm.q, arm.base)
    deg, rot = 0.0, r0
    while deg < max_deg:
        deg = min(max_deg, deg + rate * DT)
        r = axis_angle_matrix(axis, -math.radians(deg))
        rot = r @ r0
        arm._solve(heel + r @ (p0 - heel), rot)
        yield
        n = int(round(deg / (rate * DT)))
        if n % 10 == 0:
            print(f"[demo]   lever {deg:5.1f} deg: tcp cmd z {arm.at()[2] - L.TOP:.3f} seen {np.round(arm.tcp() - np.array([0, 0, L.TOP]), 3)}"
                  f" tip {np.round(bar_world(ctx, 'tip') - np.array([0, 0, L.TOP]), 4)} heel {np.round(bar_world(ctx, 'heel') - np.array([0, 0, L.TOP]), 4)}"
                  f" gap {gap(ctx, side) / MM:.1f}", flush=True)
        if gap(ctx, side) >= want:
            break
        if bar_world(ctx, "tip")[0] < FRONT_X - 0.5 * MM:
            print(f"[demo] {arm.side} lever: the edge came out from under the board", flush=True)
            break
    arm.grip = _as_grip(rot)
    print(f"[demo] {arm.side} lever: {deg:.1f} deg commanded, the board {gap(ctx, side) / MM:.1f} mm up at the "
          f"{side} nail", flush=True)
    return deg


def relax(arm, t: float = 0.6):
    """Take the command back to where the hand really is (after a stroke the soft arm lags it by centimetres), so the
    next move starts from there instead of pressing on."""
    yield from arm.move(arm.tcp(), t=t, label="relax")


def withdraw_bar(arm):
    """Draw the blade straight back out from under the board (lifting first would carry a loose board up with it),
    then lift the bar clear."""
    yield from arm.move(arm.at() + np.array([-0.07, 0.0, 0.0]), t=1.5, label="blade out")
    yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label="bar up")


def pry(ctx, arm, side: str, strokes: int = 4, withdraw: bool = True):
    """Lay the blade flat on the joist with its edge in the eased-edge V at the board's front edge, over the nail, and
    lever stroke by stroke, pushing the blade further under between strokes, until the nail is out."""
    y = L.NAILS[side]
    rest_z = L.JOIST_TOP + HALF_T["tip"] + 0.5 * MM  # the edge's centre: the blade flat, 0.5 mm over the joist
    yield from level_blade(ctx, arm, nose_down=0.0)
    yield from bring_tip(ctx, arm, (FRONT_X - 0.010, y, rest_z + 0.020), t=2.0, label="blade over the joist")
    yield from bring_tip(ctx, arm, (FRONT_X - 0.006, y, rest_z), label="blade down onto the joist")
    report(ctx, arm, "blade on the joist")
    tip = bar_world(ctx, "tip")  # into the V (3 mm deep): a plain push, which stops where the board does
    yield from arm.move(arm.at() + np.array([FRONT_X + 0.006 - tip[0], 0.0, 0.0]), t=1.0, label="edge into the V")
    want = FIRST_LIFT
    for k in range(strokes):
        report(ctx, arm, f"stroke {k + 1}")
        yield from lever(ctx, arm, side, want)
        yield from arm.wait(0.4)
        yield from relax(arm)
        if gap(ctx, side) >= GAP_DONE:
            break
        # re-set the blade: up off the work, back out, level, and in under the raised board, deeper
        want = GAP_DONE
        yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.015]), t=0.6, label="bar up a little")
        yield from level_blade(ctx, arm)
        under = min(0.030, 0.012 + gap(ctx, side) * 1.5)
        yield from bring_tip(ctx, arm, (FRONT_X - 0.008, y, rest_z + 0.004), t=1.2, label="blade back to the edge")
        tip = bar_world(ctx, "tip")
        yield from arm.move(arm.at() + np.array([FRONT_X + under - tip[0], 0.0, -0.004]), t=1.2,
                            label=f"blade {under / MM:.0f} mm under the board")
    # ease the handle back up so the board (loose now at this end) settles back on the joist
    yield from arm.arc(bar_world(ctx, "heel"), (0.0, 1.0, 0.0), 12.0, t=1.5, label="handle back up")
    yield from arm.wait(0.5)
    yield from relax(arm)
    if withdraw:
        yield from withdraw_bar(arm)
    print(f"[demo] {side} nail: the board stands {gap(ctx, side) / MM:.1f} mm off the joist", flush=True)


def pick_bar(ctx, arm, grip, width: float = L.BAR_W):
    """Down round the bar's shaft at its grip point, close across its width, lift."""
    g = bar_world(ctx, "grip")
    over = g + np.array([0.0, 0.0, PAD + 0.060])
    at = g + np.array([0.0, 0.0, PAD - 0.004])
    yield from arm.move(over, grip=grip, finger=-0.04695, t=2.5, label="over the bar")
    yield from arm.servo(over, timeout=1.0, label="over the bar")
    yield from arm.move(at, t=1.2, label="down round the shaft")
    yield from arm.servo(lambda _t: bar_world(ctx, "grip") + np.array([0.0, 0.0, PAD - 0.004]), timeout=1.0,
                         label="down round the shaft")
    yield from arm.close(width, force=50.0, label="grip the bar")
    yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.10]), t=2.5, label="lift the bar")


def set_bar_down(ctx, arm, tip_goal, label: str, heading: float = 0.0):
    """Lower the held bar (blade level, pointing along ``heading``) until its edge is just over the table at
    ``tip_goal``, let go, rise."""
    yield from level_blade(ctx, arm, heading=heading)
    yield from bring_tip(ctx, arm, np.asarray(tip_goal) + np.array([0.0, 0.0, 0.04]), t=2.0, label=f"{label}: over")
    yield from bring_tip(ctx, arm, tip_goal, t=1.2, label=f"{label}: down")
    yield from arm.open(label=f"{label}: let go")
    yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.08]), t=1.0, label=f"{label}: up")


def lift_board(ctx, arm, local_y: float = -0.09):
    """The free (right) arm grips the loose board across its 89 mm width at ``local_y`` along it (inside the bay, just
    right of its middle: within reach), lifts it straight up off the joists and the bar's blade, carries it toward the
    robot and lays it flat on the table, clear of the bar's handle, then lets go."""
    rb = ctx.state("board").rotation()
    phi = math.degrees(math.atan2(rb[1, 0], rb[0, 0]))  # the board's width axis, from +x: the fingers close along it
    grip = down(90.0 + phi)
    z = PAD - 0.006  # pads round the board's mid-thickness

    def at(up=0.0):
        b = ctx.state("board")
        return S.board_to_world(b.rotation(), b.origin, (0.0, local_y, 0.0)) + np.array([0.0, 0.0, z + up])

    b = ctx.state("board")
    print(f"[demo] board at {np.round(b.origin, 3)} (width axis {phi:.1f} deg, tilt {math.degrees(b.tilt()):.1f} deg); "
          f"grip at {np.round(at(), 3)}", flush=True)
    yield from arm.move(at(0.02), grip=grip, finger=-0.04695, t=2.5, label="over the board")
    yield from arm.servo(lambda _t: at(0.02), timeout=1.0, label="over the board")
    yield from arm.move(at(), t=1.2, label="down round the board")
    yield from arm.close(L.BOARD_W, force=40.0, label="grip the board")
    yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.025]), t=1.2, label="lift it off")
    goal = np.array([BOARD_DOWN_X, BOARD_DOWN_Y + local_y, L.TOP + L.BOARD_T / 2 + 0.002 + z])
    yield from arm.move(np.r_[goal[:2], arm.at()[2]], t=2.5, label="carry it toward the robot")
    yield from arm.move(goal, t=1.5, label="lay it down")
    yield from arm.open(label="let go")
    yield from arm.move(arm.at() + np.array([0.0, 0.0, 0.05]), t=1.0, label="up")
    yield from arm.move(PARK[arm.side], t=1.5, label="aside")


def demo_script(ctx):
    """TASK_DEMO=full: the whole task; right: the right nail only, then stop (debugging)."""
    r, l = ctx.right, ctx.left
    # the right arm: the bar lies along y, blade to the left; fingers across its width (along x)
    yield from pick_bar(ctx, r, down(90))
    report(ctx, r, "picked")
    if ctx.mode == "aim":
        yield from level_blade(ctx, r)
        return
    yield from pry(ctx, r, "right")
    if ctx.mode == "right":
        return
    # set it down in the middle, blade forward under the board, for the left arm
    yield from set_bar_down(ctx, r, (FRONT_X + 0.02, 0.0, L.TOP + HALF_T["tip"] + 1.0 * MM), "bar to the middle")
    yield from r.move(PARK["right"], t=1.5, label="aside")
    yield from pick_bar(ctx, l, down(0))
    # the last nail: the left hand keeps the bar where it is while the right hand lifts the loose board off, so the bar
    # is never dragged out from under a loose board
    yield from pry(ctx, l, "left", withdraw=False)
    yield from lift_board(ctx, r)
    yield from withdraw_bar(l)
    # the bar down on the table outside the left joist, blade forward, beyond the board's end: never under the loose
    # board, and clear of where the board will be laid
    yield from set_bar_down(ctx, l, BAR_DOWN_TIP, "bar down outside the left joist")
    yield from l.move(PARK["left"], t=1.5, label="aside")
