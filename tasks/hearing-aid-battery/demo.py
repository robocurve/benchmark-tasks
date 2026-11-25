"""The scripted reference demo, closed loop. Each control step it reads the poses of the parts in view (the aid and its
door, the cells, the tab, the magnet) and its own arms' joints; never the scorer, the detent, the magnet's force or the
tab's bond. A policy gets only the cameras and the joints.

The right hand works the battery magnet, tilted so that its gripper stays clear of the left hand; the left hand peels
the new cell's tab, then holds the aid. Modes (TASK_DEMO): full; tab (the cell goes in with its tab on); door (the door is
pushed to about 10 degrees, not clicked); drop (the new cell is slid off the magnet beside the door, onto the mat);
flip (the scene starts with the new cell upside down and its tab already in the cup; it is seated as it lies).
"""

from __future__ import annotations

import math
import os

import numpy as np
from yam_arena import ik

import bodykit as bk
import demokit as dk
import parts as P

MM = 0.001
TILT = float(os.environ.get("HAB_TILT", "4"))  # degrees the magnet leans from vertical at the door's cradle
PHI = 45.0  # its tip toward +x+y, the gripper toward -x-y (away from the left hand)
DOOR_TILT, DOOR_PHI = float(os.environ.get("HAB_DOOR_TILT", "30")), 0.0  # at the door it leans toward +x only, so the
# side of the tip that meets the pull tab is upright and cannot ride over it
HOLD_TILT = float(os.environ.get("HAB_HOLD_TILT", "30"))  # the left hand holds the aid, fingers leaning toward the door
HOLD_Y = float(os.environ.get("HAB_HOLD_Y", "0.008"))  # m from the aid's middle toward its far end
ABORT_MM = 12.0  # dev modes only: stop the demo when the tip is this far off (something is in the way)


class Abort(Exception):
    pass


def log(ctx, msg):
    line = f"[demo] t={ctx.t:.2f} {msg}"
    print(line, flush=True)


def frame(ctx, name):
    s = ctx.state(name)
    return np.asarray(s.pos, float), bk.rotation(s.quat_xyzw)


def door_deg(ctx) -> float:
    _, ra = frame(ctx, "aid")
    _, rd = frame(ctx, "door")
    r = ra.T @ rd
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def door_point(ctx, p_door, deg: float | None = None):
    """A point given in the door's frame (at angle `deg` if given, about its pin, else as it is now), in the world."""
    pd, rd = frame(ctx, "door")
    if deg is None:
        return pd + rd @ np.asarray(p_door, float)
    pa, ra = frame(ctx, "aid")
    a = math.radians(deg)
    rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    return pd + ra @ rz @ np.asarray(p_door, float)


class Stick:
    """The magnet held in the right hand: where its tip is, from its visible pose, and moves that place the tip."""

    def __init__(self, ctx):
        self.ctx, self.arm = ctx, ctx.right
        self.off = None  # the tip in the TCP's frame, measured once held

    def tip(self):
        return np.asarray(self.ctx.pos("magnet"), float)

    def measure(self):
        tcp, rot = ik.tcp_pose(self.ctx.observed("right")[:6], self.arm.base)
        self.off = rot.T @ (self.tip() - tcp)
        log(self.ctx, f"magnet tip at {np.round(self.off * 1000, 1).tolist()} mm in the TCP frame")

    def tcp_for(self, tip, grip):
        return np.asarray(tip, float) - grip.rot @ self.off

    def move(self, tip, grip=None, t=None, label="tip"):
        grip = self.arm.grip if grip is None else grip
        yield from self.arm.move(self.tcp_for(tip, grip), grip=grip, t=t, label=label)

    def servo(self, tip, tol=0.0007, timeout=2.0, gain=0.5, label="tip servo"):
        """Shift the command until the seen tip is within `tol` of `tip`."""
        a = self.arm
        c = ik.tcp_pose(a.q, a.base)[0]
        good = 0
        for _ in range(int(timeout * 20)):
            err = np.asarray(tip, float) - self.tip()
            good = good + 1 if np.linalg.norm(err) < tol else 0
            if good >= 3:
                break
            c = c + np.clip(gain * err, -0.003, 0.003)
            a._solve(c, a.grip.rot)
            yield
        a.trim = c - self.tcp_for(tip, a.grip)
        if good < 3:
            log(self.ctx, f"{label}: tip {np.linalg.norm(err) * 1000:.1f} mm short")
            if np.linalg.norm(err) > ABORT_MM * MM and self.ctx.mode.startswith("dev"):
                raise Abort(label)

    def rise(self, dz, rate=0.008, hold=0.4, label="rise"):
        """Hold the tip where it is for `hold` s, then raise it straight up by dz at `rate` m/s, on a held line."""
        p0 = self.tip()
        n = [0]

        def target():
            n[0] += 1
            return p0 + np.array([0, 0, min(dz, max(0.0, (n[0] / 20.0 - hold) * rate))])

        yield from self.path(target, t=hold + dz / rate + 0.5, speed=0.03, label=label)

    def path(self, target, t=2.0, speed=0.02, until=None, gain=0.0, label="tip path"):
        """Drive the tip along a moving target (read each step) at up to `speed` m/s. The tip's seen error across the
        target's direction of travel is integrated into the command (so it stays on the line while it pushes along it)."""
        a = self.arm
        c = ik.tcp_pose(a.q, a.base)[0]
        prev, fix = None, np.zeros(3)
        for k in range(int(t * 20)):
            goal = np.asarray(target(), float)
            e = goal - self.tip()
            if prev is not None and np.linalg.norm(goal - prev) > 1e-5:
                u = (goal - prev) / np.linalg.norm(goal - prev)
                e = e - float(np.dot(e, u)) * u
            prev = goal
            fix = np.clip(fix + 0.15 * e, -0.008, 0.008)
            want = self.tcp_for(goal, a.grip) + a.trim + fix
            if self.ctx.mode.startswith("dev") and k % 5 == 0:
                log(self.ctx, f"{label}: goal {np.round(goal * 1000, 1).tolist()} tip {np.round(self.tip() * 1000, 1).tolist()} door {door_deg(self.ctx):.1f}")
            if gain:
                want = want + gain * (goal - self.tip())
            d = want - c
            n = float(np.linalg.norm(d))
            c = c + d * min(1.0, speed / 20.0 / max(n, 1e-9))
            a._solve(c, a.grip.rot)
            yield
            if until is not None and until():
                break


def grips():
    return dk.down(90.0), dk.tilted(TILT, PHI)


def take_magnet(ctx, st: Stick):
    r = ctx.right
    down, _ = grips()
    top = P.Z0 + P.HOLDER_FLOOR + P.STICK_L
    xy = ctx.pos("magnet")[:2]
    grip_z = top - 0.024  # the fingertips' bottom 24 mm under the shaft's top
    tcp_z = grip_z + 0.013
    yield from r.move((xy[0], xy[1], tcp_z + 0.06), grip=down, finger=dk.FINGER_OPEN)
    yield from r.servo((xy[0], xy[1], tcp_z + 0.06), timeout=1.0)
    yield from r.move((xy[0], xy[1], tcp_z), t=1.2)
    yield from r.servo((xy[0], xy[1], tcp_z), timeout=1.0)
    yield from r.close(width=2 * P.SHAFT_R, force=38.0)
    yield from r.wait(0.3)
    yield from r.move(r.at() + np.array([0, 0, 0.08]), t=1.2)
    st.measure()


def peel_tab(ctx):
    """Left hand: pinch the tab's upright tail and pull it low across the cell, then drop it in the cup."""
    l = ctx.left
    tab = ctx.pos("tab")
    tail = tab + np.array([0.0, P.CELL_R + 0.4 * MM, 0.0])
    z_tip = tab[2] + 0.004  # the fingertips' bottom 4 mm over the cell's top
    tcp = np.array([tail[0], tail[1], z_tip + 0.013])
    yield from l.move(tcp + np.array([0, 0, 0.06]), grip=dk.down(0.0), finger=dk.FINGER_OPEN)
    yield from l.servo(tcp + np.array([0, 0, 0.06]), timeout=1.0)
    yield from l.move(tcp, t=1.2)
    yield from l.servo(tcp, tol=0.0015, timeout=1.0)
    yield from l.close(width=0.0008, force=4.0)
    yield from l.wait(0.4)
    log(ctx, "tab pinched; pulling")
    yield from l.move(l.at() + np.array([0, -0.016, 0.002]), t=2.5, label="peel")
    yield from l.move(l.at() + np.array([0, -0.004, 0.03]), t=1.0, label="peel lift")
    log(ctx, f"tab at {np.round(ctx.pos('tab') * 1000, 1).tolist()}, cell at {np.round(ctx.pos('new_cell') * 1000, 1).tolist()}")
    over = np.array([P.CUP_XY[0], P.CUP_XY[1], P.Z0 + P.CUP_H + 0.04])
    yield from l.move(over, t=2.0)
    yield from l.open()
    yield from l.wait(0.5)
    yield from l.move(over + np.array([0, 0.03, 0.04]), t=1.0)


def hold_aid(ctx):
    l = ctx.left
    pa, ra = frame(ctx, "aid")
    g = pa + ra @ P.q3([0.0, HOLD_Y, 0.0])  # where the pads are centred along the aid
    grip = dk.tilted(HOLD_TILT, -90.0)  # fingers closing along x, across the aid's depth, leaning toward -y
    axis = grip.rot[:, 2]
    tips = np.array([g[0], g[1], P.Z0 + 0.0025 + 0.0132 * math.sin(math.radians(HOLD_TILT)) + 0.0005])
    tcp = tips - axis * 0.013
    yield from l.move(tcp - axis * 0.06, grip=grip, finger=dk.FINGER_OPEN)
    yield from l.servo(tcp - axis * 0.06, timeout=1.0)
    yield from l.move(tcp, t=1.2)
    yield from l.servo(tcp, timeout=1.0)
    yield from l.close(width=P.AID_D, force=35.0)
    yield from l.wait(0.3)


def release_aid(ctx):
    l = ctx.left
    yield from l.open()
    yield from l.move(l.at() + np.array([0, 0, 0.08]), t=1.0)
    yield from l.move((0.25, 0.25, P.TOP + 0.20), t=1.5)


# door-frame points (the door's own axes, written as in the aid's frame and turned by P.Q)
RIDGE = P.q3([-9.0 * MM, 4.8 * MM, 12.65 * MM])
RIDGE_START = P.q3([-9.0 * MM, 8.5 * MM, 12.65 * MM])  # where the tip comes down: 3.7 mm behind that, on the aid's top  # where the tip stands to push the door open: beside its nail ridge, its
# lowest edge (the tip leans 10 deg) just over the aid's top
PUSH_SHUT = P.q3([-9.0 * MM, -3.2 * MM, 7.0 * MM])  # where it stands to push it shut: on its outer end plate


def open_door(ctx, st: Stick):
    tilt = dk.tilted(DOOR_TILT, DOOR_PHI)
    start = door_point(ctx, RIDGE_START, 0.0)  # on the aid's top, well behind the pull tab
    yield from st.move(start + np.array([0, 0, 0.03]), grip=tilt, t=2.5, label="over the door")
    yield from st.servo(start + np.array([0, 0, 0.03]), tol=0.0008, gain=0.3, timeout=3.0)
    yield from st.move(start + np.array([0, 0, 0.004]), t=1.0, label="down")
    yield from st.servo(start + np.array([0, 0, 0.004]), tol=0.0006, gain=0.3, timeout=3.0)
    yield from st.move(start - np.array([0, 0, 0.0006]), t=1.2, label="onto the aid's top")
    log(ctx, f"on the aid behind the pull tab; door {door_deg(ctx):.1f}")
    lead = [0.0]
    pd0, ra0 = frame(ctx, "door")[0], frame(ctx, "aid")[1]  # the frame at the start: the tip does not chase an aid that moves

    def target():
        lead[0] = min(lead[0] + 1.0, 110.0)
        a = math.radians(min(door_deg(ctx) + 25.0, lead[0]))
        rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1.0]])
        return pd0 + ra0 @ rz @ (RIDGE - np.array([0, 0, 0.0006]))

    yield from st.path(target, t=8.0, speed=0.012, until=lambda: door_deg(ctx) > 93.0, label="open")
    log(ctx, f"door open {door_deg(ctx):.1f}")
    yield from st.move(st.tip() + np.array([0.0, 0.004, 0.025]), t=1.0, label="up from ridge")
    yield from st.move(st.tip() + np.array([0.0, -0.01, 0.01]), grip=grips()[1], t=1.5, label="tilt for the cells")


def rim_drop(ctx) -> float:
    """How far the magnet face's lowest edge hangs below its centre, from the magnet's seen tilt."""
    _, rm = frame(ctx, "magnet")
    return P.MAG_R * math.sqrt(max(0.0, 1.0 - rm[2, 2] ** 2))


def away_from_plate(ctx, mm=1.2):
    """A shift along the open door's own +y: from the cradle away from the door's end plate, which stands beside it."""
    _, rd = frame(ctx, "door")
    return rd @ P.q3([0.0, mm * MM, 0.0])


def lift_cell(ctx, st: Stick, name: str, grip=None, bias=lambda: np.zeros(3)):
    c = ctx.pos(name) + bias()
    top = c + np.array([0, 0, P.CELL_BODY_H / 2])
    yield from st.move(top + np.array([0, 0, 0.02]), grip=grip, t=1.5, label=f"over {name}")
    yield from st.servo(top + np.array([0, 0, 0.02]), tol=0.0006, gain=0.3, timeout=3.0)
    over = ctx.pos(name) + bias() + np.array([0, 0, P.CELL_BODY_H / 2 + rim_drop(ctx) + 0.0004])
    yield from st.move(over + np.array([0, 0, 0.003]), t=1.5, label=f"down to {name}")
    yield from st.servo(over + np.array([0, 0, 0.003]), tol=0.0005, gain=0.3, timeout=3.0)
    here = st.tip()
    down = np.array([here[0], here[1], over[2]])  # straight down to 0.4 mm over the face, where the magnet takes it
    yield from st.move(down, t=1.5, label="near")
    yield from st.servo(down, tol=0.0003, gain=0.3, timeout=2.0, label="near")
    yield from st.rise(0.022, label=f"lift {name}")
    yield from ctx.right.wait(0.3)
    log(ctx, f"{name} at {np.round(ctx.pos(name) * 1000, 1).tolist()}, tip {np.round(st.tip() * 1000, 1).tolist()}")


def scrape_in_cup(ctx, st: Stick, name: str):
    """Hang the cell in the cup's mouth, then slide the tip out over the rim: the cell meets the wall's inside and stays."""
    cup = np.array([P.CUP_XY[0], P.CUP_XY[1], P.Z0])
    low = cup + np.array([0, 0, P.CUP_H + 0.0012])  # the tip's lowest edge (it leans 10 deg) just over the rim; the cell hangs below
    yield from st.move(low + np.array([0, 0, 0.03]), t=3.5, label="over cup")
    yield from st.servo(low + np.array([0, 0, 0.03]), timeout=1.0)
    yield from st.move(low, t=1.2, label="into the mouth")
    yield from st.servo(low, tol=0.0007, timeout=1.0)
    yield from st.move(low + np.array([0, 0.028, 0.0005]), t=2.5, label="out over the rim")
    yield from ctx.right.wait(0.5)
    log(ctx, f"{name} at {np.round(ctx.pos(name) * 1000, 1).tolist()}")
    yield from st.move(st.tip() + np.array([0, -0.005, 0.045]), t=1.2, label="out of cup")


def seat_new(ctx, st: Stick, drop=False):
    cradle = door_point(ctx, P.CRADLE_REL)  # the seated cell's centre
    top = cradle + np.array([0, 0, P.CELL_BODY_H / 2])
    cell = ctx.pos("new_cell")
    hang = st.tip() - cell  # how the cell hangs from the tip
    log(ctx, f"cell hangs {np.round(hang * 1000, 1).tolist()} mm from the tip")
    if drop:  # a scorer check: the cell is let down onto the mat beside the door first (dropped), then seated as usual
        beside = cradle + np.array([-0.014, 0.0, 0.0]) + hang
        yield from st.move(beside + np.array([0, 0, 0.015]), grip=grips()[1], t=3.0, label="beside the door")
        yield from st.move(beside - np.array([0, 0, P.SEAT_Z - P.CELL_BODY_H / 2 - P.CAP_H + 0.0005]), t=2.0, label="onto the mat")
        yield from ctx.right.wait(0.5)
    goal_tip = cradle + hang
    yield from st.move(goal_tip + np.array([0, 0, 0.015]), grip=grips()[1], t=3.5, label="over cradle")
    yield from st.servo(goal_tip + np.array([0, 0, 0.015]), tol=0.0006, timeout=2.0)
    # line the cell itself up over the cradle, from where it is seen
    for _ in range(5):
        err = (cradle + away_from_plate(ctx, 0.3) + np.array([0, 0, 0.015])) - ctx.pos("new_cell")
        err[2] = 0
        if np.linalg.norm(err) < 0.0004:
            break
        yield from st.servo(st.tip() + err, tol=0.0003, gain=0.2, timeout=2.0, label="cell over cradle")
        yield from ctx.right.wait(0.3)
    p0, n = st.tip(), [0]

    def lowering():
        n[0] += 1
        return p0 - np.array([0, 0, min(0.017, n[0] * 0.006 / 20.0)])

    # down at 6 mm/s until the cell is seen at its seat (it stops on the lip), so it is never pressed into the door
    yield from st.path(lowering, t=4.0, speed=0.02, until=lambda: ctx.pos("new_cell")[2] < cradle[2] + 0.0003, label="lower in")
    yield from ctx.right.wait(0.4)
    log(ctx, f"cell at {np.round((ctx.pos('new_cell') - cradle) * 1000, 2).tolist()} mm from the seat")
    # slide off straight away from the door's pin, over the ring's top: pulled that way, the door cannot swing, so the
    # cradle's wall holds the cell while the tip slides off it (the end plate stands on the other side)
    pivot = frame(ctx, "door")[0]
    out = cradle - pivot
    out[2] = 0.0
    out /= np.linalg.norm(out)
    yield from st.move(st.tip() + out * 0.0085 + np.array([0, 0, -0.0002]), t=2.0, label="slide off")
    yield from ctx.right.wait(0.3)
    yield from st.move(st.tip() + out * 0.004 + np.array([0, 0, 0.02]), t=1.0, label="up")
    log(ctx, f"cell now {np.round((ctx.pos('new_cell') - cradle) * 1000, 2).tolist()} mm from the seat")


def close_door(ctx, st: Stick, click=True):
    deg = door_deg(ctx)
    start = door_point(ctx, PUSH_SHUT)
    yield from st.move(start + np.array([0, -0.01, 0.025]), grip=dk.tilted(DOOR_TILT, DOOR_PHI), t=1.5, label="over door end")
    yield from st.move(start, t=1.0, label="down by door end")
    yield from st.servo(start, tol=0.0008, timeout=1.0)
    stop = -3.0 if click else 10.0
    lead = [deg]

    def target():
        lead[0] = max(lead[0] - 1.2, stop)
        return door_point(ctx, PUSH_SHUT, max(min(door_deg(ctx) - 25.0, lead[0]), stop))

    yield from st.path(target, t=10.0, speed=0.012, until=(lambda: door_deg(ctx) < 0.5) if click else (lambda: door_deg(ctx) < 11.0), label="shut")
    yield from ctx.right.wait(0.4)
    log(ctx, f"door at {door_deg(ctx):.1f}")
    yield from st.move(st.tip() + np.array([0, -0.006, 0.0]), t=0.8, label="back off")
    yield from st.move(st.tip() + np.array([0, -0.01, 0.05]), t=1.2, label="away")


def script(ctx, mode: str):
    try:
        for _ in _script(ctx, mode.removeprefix("dev") or "full"):
            if ctx.mode.startswith("dev"):
                for n in ("old_cell", "new_cell"):
                    if ctx.pos(n)[2] < P.TOP - 0.01:
                        raise Abort(f"{n} fell off the table")
            yield
    except Abort as e:
        log(ctx, f"ABORT at {e}")


def _script(ctx, mode: str):
    st = Stick(ctx)
    yield from take_magnet(ctx, st)
    if mode not in ("tab", "flip"):
        yield from peel_tab(ctx)
    yield from hold_aid(ctx)
    if mode != "open":
        yield from open_door(ctx, st)
    if mode == "dooronly":
        yield from ctx.right.wait(1.0)
        return
    yield from lift_cell(ctx, st, "old_cell", grips()[1], bias=lambda: away_from_plate(ctx))
    yield from scrape_in_cup(ctx, st, "old_cell")
    yield from lift_cell(ctx, st, "new_cell", dk.down(90.0))  # upright over the pack, away from the other hand
    yield from seat_new(ctx, st, drop=mode == "drop")
    yield from close_door(ctx, st, click=mode != "door")
    yield from release_aid(ctx)
    yield from ctx.right.move(ctx.right.at() + np.array([-0.03, -0.08, 0.06]), t=1.5)
    yield from ctx.right.wait(3.0)
