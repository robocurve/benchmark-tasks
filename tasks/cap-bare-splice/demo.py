"""The scripted demo for cap-bare-splice, run by demokit.py through task.py's ``demo_script``.

It reads the simulator's state to plan (the nut's pose, the splices' joints, the cover's angle); it never moves an
object or edits the simulation. See README.md for the steps.
"""

from __future__ import annotations

import numpy as np

import capping
import layout as L

T = None  # the task module, set by demo_script

DEMO_NUT = 0  # the upright nut nearest the box
GRIP_X = 20.0 * L.MM  # the jaws hold the nut across two flats of its body, the TCP this far up its axis from the mouth
CAP_DEPTH = 13.0 * L.MM  # the copper's end this far into the nut when it is pushed on
STROKE = 180.0  # degrees turned per grip: the wings come back square to the jaws for the next grip
ROLL0 = -100.0  # the grip's roll about its approach at the start of a stroke (the right wrist's roll near -1.7 rad)
JAW_AJAR = -0.014  # finger joint: the pads 28 mm apart, clear of the tray's walls and the other nuts
TURN_GOAL = 722.0  # degrees: just past the two turns that make it bite; the last few degrees go slowly
NUT_PAST_TIP = {"black": 13.0 * L.MM, "white": 14.5 * L.MM}  # how far a capped end's nut reaches past the copper


def _grip_along(a, roll_deg: float):
    """A grip pointing along ``a``, turned ``roll_deg`` about it."""
    import math
    from demokit import _as_grip, tilted
    from yam_arena.urdf import axis_angle_matrix

    a = np.asarray(a, float) / np.linalg.norm(a)
    phi = math.degrees(math.atan2(a[1], a[0]))
    theta = math.degrees(math.acos(float(np.clip(-a[2], -1.0, 1.0))))
    return _as_grip(axis_angle_matrix(a, math.radians(roll_deg)) @ tilted(theta, phi).rot)


def _cap_black(ctx):
    """The right arm picks the upright nut across two flats of its body, pushes it onto the bare black splice and
    twists it on, in half-turn strokes with a regrip between them."""
    import math
    from demokit import down

    r = ctx.right
    name = T.NUTS[DEMO_NUT].name

    def nut():
        return T.nut_frame(DEMO_NUT, ctx.state(name))

    def tip():
        return T.copper_tip(ctx.state("black"))

    # 1. pick it across two flats of its body (the jaws closing square to the wings)
    R, o = nut()
    print(f"[demo] nut at {np.round(o, 4).tolist()}, axis {np.round(R[:, 0], 2).tolist()}", flush=True)
    w = R[:, 2]
    yaw = math.degrees(math.atan2(-w[0], w[1]))
    yaw = (yaw + 90.0) % 180.0 - 90.0  # the right wrist's comfortable half
    def grip_point():
        Rn, on = nut()
        return on + Rn[:, 0] * GRIP_X

    g = down(yaw)
    up = np.array([0.0, 0.0, 0.08])
    yield from r.move(grip_point() + up, grip=g, finger=JAW_AJAR, label="pick nut: over")
    yield from r.servo(grip_point() + up, timeout=1.0, label="pick nut: over")
    yield from r.move(grip_point(), t=1.0, label="pick nut: down")
    yield from r.servo(grip_point(), timeout=1.0, label="pick nut: down")
    yield from r.close(width=0.0084, label="pick nut: close")
    yield from r.move(r.at() + np.array([0.0, 0.0, 0.10]), t=1.0, label="pick nut: lift")
    R, o = nut()
    print(f"[demo] picked: nut at {np.round(o, 4).tolist()}, axis {np.round(R[:, 0], 2).tolist()}, "
          f"TCP {np.round(r.tcp(), 4).tolist()}", flush=True)
    if ctx.mode == "pick":
        return
    # 2. point it along the copper, in front of it
    end, d, _ = tip()
    a = -d
    g = _grip_along(a, ROLL0)
    yield from r.move(end + d * (GRIP_X - CAP_DEPTH + 0.05) + np.array([0.0, 0.0, 0.03]), grip=g, label="to splice")
    yield from r.move(end + d * (GRIP_X - CAP_DEPTH + 0.03), grip=g, t=1.0, label="line up")

    # 3. push it on: the point CAP_DEPTH into the nut onto the copper's end
    def depth_point():
        Rn, on = nut()
        return on + Rn[:, 0] * CAP_DEPTH

    yield from r.servo(tip()[0], read=depth_point, tol=0.0015, timeout=3.0, label="push on")
    e = capping.engagement(*nut(), tip()[0], tip()[1])
    print(f"[demo] pushed on: copper {e.depth / L.MM:.1f} mm in, {e.radial / L.MM:.1f} mm off axis, "
          f"{math.degrees(e.angle):.1f} deg", flush=True)

    # 4. twist on, in strokes; regrip between them
    turned, last = 0.0, capping.turn_angle(nut()[0], tip()[2])

    def update():
        nonlocal turned, last
        now = capping.turn_angle(nut()[0], tip()[2])
        turned += -math.degrees(capping._wrap(now - last))
        last = now

    strokes = 0
    while turned < TURN_GOAL - 0.5 and strokes < 8:
        done = 0.0
        while done < STROKE - 1e-6 and turned < TURN_GOAL - 0.5:
            step = min(30.0, STROKE - done, TURN_GOAL - turned)
            speed = 40.0 if TURN_GOAL - turned > 40.0 else 12.0  # slow onto the bite
            yield from r.arc(tip()[0], a, step, t=max(0.2, step / speed), label="twist")  # about the copper's axis
            done += step
            update()
        strokes += 1
        e = capping.engagement(*nut(), tip()[0], tip()[1])
        print(f"[demo] stroke {strokes}: turned {turned:.0f} deg, copper {e.depth / L.MM:.1f} mm in, "
              f"{e.radial / L.MM:.1f} mm off axis", flush=True)
        if turned >= TURN_GOAL - 0.5:
            yield from r.wait(0.5)
            break
        yield from r.move(r.at(), finger=JAW_AJAR, t=0.4, label="regrip: open")
        yield from r.turn(-done, axis=a, t=done / 60.0, label="regrip: back")
        Rn, on = nut()
        yield from r.move(on + Rn[:, 0] * GRIP_X, t=0.5, label="regrip: on the nut")
        yield from r.close(width=0.0084, label="regrip: close")
        update()
    yield from r.open(label="let go of the nut")
    end, d, _ = tip()
    yield from r.move(r.at() + d * 0.06, t=1.0, label="back off")


def _wire_q(ctx, name: str) -> np.ndarray:
    st = ctx.state(name)
    return np.array([st.joint(f"w{i}") for i in range(L.LINKS)])


def _held(ctx, name: str, back: float = 0.0):
    """(grip point, direction root to end, rotation) of a splice's fifth link now: its middle, or ``back`` from the
    middle toward the box."""
    st = ctx.state(name)
    root = np.asarray(L.SPLICE_ROOT[name], float)
    mid = (L.link_start(name, 5) + L.link_start(name, 6)) / 2 - back * L.splice_dir(name)
    R = st.link_pose("w5")[0]
    return st.link_to_world("w5", mid - root), R @ L.splice_dir(name), R


def _grip_on_wire(a, u):
    """A grip approaching along ``a``, the jaws closing across the wire direction ``u`` (demokit's grip frame: the
    jaws close along its x axis, the approach is its z axis)."""
    from demokit import _as_grip

    a = np.asarray(a, float) / np.linalg.norm(a)
    n = np.cross(a, u)
    n /= np.linalg.norm(n)
    return _as_grip(np.column_stack([n, np.cross(a, n), a]))


def _reachable(arm, pos, grips):
    """Of these grips, the one the arm's IK holds best at ``pos`` (from where it is now)."""
    from yam_arena import ik

    def err(g):
        _q, pe, re_ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(g.rot), arm.q, base_pos=arm.base, iters=80)
        return pe + 0.02 * re_

    return min(grips, key=err)


def _flipped(g):
    """The same grip turned half a turn about its approach (a parallel gripper holds the same either way)."""
    from demokit import _as_grip

    return _as_grip(g.rot @ np.diag([-1.0, -1.0, 1.0]))


def _tuck(ctx, arm, name: str, a0, stage, final, label: str, back: float = 0.0, into=(1.0, 0.0, 0.0),
          hang=(0.0, 0.0, -1.0)):
    """Fold a splice into the box. Take its fifth link with the hand coming in along about ``a0``, the jaws across the
    wire; carry it to ``stage`` in front of the box while turning it so the wire hangs straight down and the hand points
    into the box; push it straight back to ``final``. The links behind it fold up inside the box, and the wire stays as
    it is left. ``back``: grip that far from the link's middle toward the box (clear of a nut on its end)."""
    from demokit import _as_grip

    c, u, _ = _held(ctx, name, back)
    a0 = np.asarray(a0, float)
    a0 = a0 - (a0 @ u) * u
    a0 /= np.linalg.norm(a0)
    g = _grip_on_wire(a0, u)
    g = _reachable(arm, c - a0 * 0.03, [g, _flipped(g)])
    yield from arm.move(c - a0 * 0.03, grip=g, finger=JAW_AJAR, label=f"{label}: by the wire")
    yield from arm.servo(c - a0 * 0.03, timeout=1.0, label=f"{label}: by the wire")
    yield from arm.move(c, t=0.8, label=f"{label}: onto the wire")
    yield from arm.servo(c, timeout=1.0, label=f"{label}: onto the wire")
    yield from arm.close(width=0.004, force=40.0, label=f"{label}: grip")  # the YAM's full squeeze
    c1, u1, _ = _held(ctx, name, back)
    print(f"[demo] {label}: gripped: link at {np.round(c1, 4).tolist()} (aimed {np.round(c, 4).tolist()}), TCP "
          f"{np.round(arm.tcp(), 4).tolist()}, opening {ctx.observed(arm.side)[6]:.3f}", flush=True)
    # the turn that takes (wire, hand) to (``hang``, ``into``): the hand pointing into the box, the wire square to it
    into = np.asarray(into, float) / np.linalg.norm(into)
    down_ = np.asarray(hang, float) - (np.asarray(hang, float) @ into) * into
    down_ /= np.linalg.norm(down_)
    R = np.column_stack([down_, into, np.cross(down_, into)]) @ np.column_stack([u, a0, np.cross(u, a0)]).T
    g1 = _as_grip(R @ g.rot)
    if name == "black":
        _nut_status(ctx, "gripped, before the carry")
    carry = arm.move(np.asarray(stage, float), grip=g1, t=6.0, label=f"{label}: turn and carry to the box")
    for _ in carry:
        yield
    c, u, _ = _held(ctx, name, back)
    print(f"[demo] {label}: staged: link at {np.round(c, 3).tolist()}, wire {np.round(u, 2).tolist()}", flush=True)
    yield from arm.move(np.asarray(final, float), t=2.5, label=f"{label}: push in")
    c, u, _ = _held(ctx, name, back)
    print(f"[demo] {label}: pushed to {np.round(c, 3).tolist()}, wire {np.round(u, 2).tolist()}", flush=True)
    yield from arm.move(arm.at(), finger=JAW_AJAR, t=0.4, label=f"{label}: let go")
    arm.trim = np.zeros(3)
    yield from arm.move(arm.at() - arm.approach_axis() * 0.08, t=1.0, label=f"{label}: out of the box")


def _outermost(ctx, name: str, wire_only: bool = False) -> np.ndarray:
    """The splice's point nearest the robot (smallest x): along its wire, and the far end of its nut; with
    ``wire_only``, along the insulated wire of its last three links only (to push on the wire, not the nut)."""
    st = ctx.state(name)
    root = np.asarray(L.SPLICE_ROOT[name], float)
    if wire_only:
        pts = [st.link_to_world(f"w{i}", L.link_start(name, i) + f * (L.link_start(name, i + 1) - L.link_start(name, i))
                                - root) for i in range(L.LINKS - 3, L.LINKS) for f in (0.0, 0.25, 0.5, 0.75)]
        pts = np.asarray(pts)
        return pts[int(np.argmin(pts[:, 0]))]
    pts = [st.link_to_world(f"w{i}", L.link_start(name, k) - root) for i in range(L.LINKS) for k in (i, i + 1)]
    pts.append(st.link_to_world(T.TIP, L.splice_tip(name) + L.splice_dir(name) * NUT_PAST_TIP[name] - root))
    pts = np.asarray(pts)
    return pts[int(np.argmin(pts[:, 0]))]


def _push_in(ctx, arm, name: str, depth: float, label: str, wire_only: bool = False):
    """Close the jaws and push the splice's outermost point straight back into the box, ``depth`` behind the rim."""
    from demokit import level

    p = _outermost(ctx, name, wire_only)
    g = level(0.0)
    tips = L.RIM_X + depth
    start = np.array([p[0] - 0.03 - 0.013, p[1], p[2]])
    yield from arm.move(start, grip=g, finger=0.0, label=f"{label}: in front of the end")
    yield from arm.servo(start, timeout=1.0, label=f"{label}: in front of the end")
    yield from arm.move(np.array([tips - 0.013, p[1], p[2]]), t=1.5, label=f"{label}: push the end in")
    yield from arm.move(np.array([p[0] - 0.06, p[1], p[2]]), t=1.0, label=f"{label}: back out")
    arm.trim = np.zeros(3)


def _nut_status(ctx, when: str) -> None:
    """Where the demo nut sits on the black copper (as a policy could see it: the two parts' poses)."""
    end, d, _ = T.copper_tip(ctx.state("black"))
    e = capping.engagement(*T.nut_frame(DEMO_NUT, ctx.state(T.NUTS[DEMO_NUT].name)), end, d)
    print(f"[demo] nut {when}: copper {e.depth / L.MM:.1f} mm in, {e.radial / L.MM:.1f} mm off axis", flush=True)


def _sticking_out(ctx, name: str) -> bool:
    return _outermost(ctx, name)[0] < L.RIM_X + 0.012


def _fold_both(ctx):
    """Both splices into the box: the black (on the right) with the right arm, then the white with the left; then
    any end still near the rim pushed back by the arm on its side."""
    z = L.BOX_Z + 0.0175
    # the black's end (with its nut) swings back toward the robot as it is turned, under the palm: the held push stops
    # short of the rim, and the closed fingertips push the rest in below
    yield from _tuck(ctx, ctx.right, "black", (0.0, 0.0, -1.0), (0.36, -0.028, z + 0.015), (0.40, -0.028, z + 0.015),
                     "black", back=0.008)
    _nut_status(ctx, "after the black fold")
    yield from ctx.right.move((0.25, -0.20, L.BOX_Z + 0.08), t=1.5, label="right: clear")
    yield from _tuck(ctx, ctx.left, "white", (0.0, 0.0, -1.0), (0.37, 0.028, z), (0.435, 0.028, z), "white")
    yield from ctx.left.move((0.25, 0.20, L.BOX_Z + 0.08), t=1.5, label="left: clear")
    for _round in range(3):
        todo = [n for n in ("black", "white") if _sticking_out(ctx, n)]
        if not todo:
            break
        for name in todo:
            arm = ctx.right if _outermost(ctx, name)[1] < L.BOX_Y else ctx.left
            if name == "black":  # push on the wire behind the nut, deeper
                yield from _push_in(ctx, arm, name, 0.04, name, wire_only=True)
                _nut_status(ctx, f"after black push {_round + 1}")
            else:
                yield from _push_in(ctx, arm, name, 0.03, name)
            park = (0.25, -0.20, L.BOX_Z + 0.08) if arm is ctx.right else (0.25, 0.20, L.BOX_Z + 0.08)
            yield from arm.move(park, t=1.0, label=f"{arm.side}: clear")
        print(f"[demo] push round {_round + 1}: black frontmost {1000 * (_outermost(ctx, 'black')[0] - L.RIM_X):.1f} mm "
              f"behind the rim", flush=True)


def _close_cover(ctx, arm):
    """Swing the cover down onto the box: the closed fingertips push its edge 5 cm from the pivot screw round the
    screw, staying just off the edge and outside the box's outline, until it is closed and the screw catches it."""
    import math
    from demokit import level

    P = np.asarray(L.PIVOT, float)
    edge_y = P[1] - (L.COVER_W / 2 - L.SCREW_OFF)  # the cover's edge on the screw's side, as drawn (hanging open)
    tips_x = L.COVER_X + 4.5 * L.MM  # the fingertips reach just behind the cover's plane
    b = np.array([tips_x - 0.013, edge_y - 0.004, P[2] + 0.05])  # the TCP, as drawn: the finger's side on the edge

    def at(q):
        c, s_ = math.cos(q), math.sin(q)
        r = b - P
        return P + np.array([r[0], c * r[1] - s_ * r[2], s_ * r[1] + c * r[2]])

    g = level(0.0)
    q0 = ctx.state("jbox").joint("cover")
    start = at(q0 + 0.30)
    yield from arm.move(start - np.array([0.04, 0.0, 0.0]), grip=g, finger=0.0, label="cover: in front")
    yield from arm.move(start, t=0.8, label="cover: beside its edge")
    yield from arm.servo(start, timeout=1.0, label="cover: beside its edge")
    q_end = L.CLOSED - 0.10  # past closed: the hinge stops at its limit, and the screw catches it on the way
    n = int(abs(q_end - (q0 + 0.30)) / math.radians(1.5))
    for k in range(1, n + 1):
        q = q0 + 0.30 + (q_end - q0 - 0.30) * k / n
        arm._solve(at(q) + arm.trim, g.rot)
        yield
        if k % 15 == 0 or k == n:
            print(f"[demo] cover: pushing at {math.degrees(q):.0f}, cover at "
                  f"{math.degrees(ctx.state('jbox').joint('cover')):.1f} deg", flush=True)
    arm.grip = g
    q = ctx.state("jbox").joint("cover")
    print(f"[demo] cover: at {math.degrees(q):.1f} deg (closed is {math.degrees(L.CLOSED):.0f})", flush=True)
    yield from arm.wait(0.5)
    yield from arm.move(arm.at() - np.array([0.06, 0.0, 0.0]), t=1.0, label="cover: back off")
    arm.trim = np.zeros(3)


def _wire_inside(p) -> bool:
    """A wire point (world) behind the rim plane and within the box's inside walls (the demo's own check)."""
    return bool(p[0] >= L.RIM_X + L.INSIDE_MARGIN and abs(p[1] - L.BOX_Y) <= L.INNER_HALF
                and abs(p[2] - L.BOX_Z) <= L.INNER_HALF)


def _report_wires(ctx) -> None:
    st = ctx.states()
    for name in ("black", "white"):
        s_, root = st[name], np.asarray(L.SPLICE_ROOT[name], float)
        bad = []
        for i in range(L.LINKS):
            for p_ in (L.link_start(name, i), L.link_start(name, i + 1)):
                w = s_.link_to_world(f"w{i}", p_ - root)
                if not _wire_inside(w):
                    bad.append((i, np.round((w - np.array([L.RIM_X, L.BOX_Y, L.BOX_Z])) * 1000).astype(int).tolist()))
        q = np.degrees([s_.joint(f"w{i}") for i in range(L.LINKS)])
        o = _outermost(ctx, name)
        print(f"[demo] {name}: joints {np.round(q).astype(int).tolist()}; outside (mm from rim centre): {bad}; "
              f"frontmost point {1000 * (o[0] - L.RIM_X):.1f} mm behind the rim at y {1000 * o[1]:.0f}, "
              f"z {1000 * (o[2] - L.BOX_Z):.0f} mm", flush=True)


def demo_script(ctx):
    """Run by task.py's demo_script. TASK_DEMO=full: cap the black splice, fold the splices in, close the cover. cap: only the capping. idle: the
    arms stay at their ready pose (the setup screenshot). look: the right wrist camera passes in front of the box and
    over the tray (nothing touched)."""
    global T
    T = ctx.emb.task.module  # the task module the harness loaded (importing task.py again would load a second copy)
    if ctx.mode == "look":
        from demokit import level, down
        r = ctx.right
        yield from r.move((0.24, -0.06, L.BOX_Z + 0.03), grip=level(15), t=2.5)
        for _ in range(30):
            yield
        yield from r.move((0.26, -0.17, L.TOP + 0.20), grip=down(90), t=2.0)
        for _ in range(30):
            yield
        return
    if ctx.mode in ("cap", "full", "pick"):
        yield from _cap_black(ctx)
        if ctx.mode == "pick":
            return
    if ctx.mode in ("full", "fold"):
        yield from _fold_both(ctx)
        _report_wires(ctx)
    if ctx.mode in ("full", "fold", "cover"):
        yield from _close_cover(ctx, ctx.left)
        yield from ctx.together(ctx.left.move((0.20, 0.22, L.BOX_Z + 0.10), t=1.5, label="left: park"),
                                ctx.right.move((0.20, -0.22, L.BOX_Z + 0.10), t=1.5, label="right: park"))
        print(f"[demo] T.reading: {T.reading(ctx.states())}", flush=True)
    if ctx.mode in ("cap", "full", "fold", "cover"):
        for _ in range(80):
            yield
        return
    for _ in range(40):
        yield
