"""Scripted full-score demo for the cap-a-marker task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2000] [--no-record] [--cam 640x352]
    DEMO_CALIBRATE=1 python demo.py task.py cal_out --no-record     # measures and prints CAL (closed loop)

The left arm picks the marker up by its barrel, turns it upright, tip up, and stands it on its back end on the table,
holding it there. The right arm picks the cap up near its closed end, turns it mouth down, brings it over the tip in
line with the barrel and lowers it over the nib and onto the nose until it stops on the snap ridge; it lets go, turns
its closed fingertips down onto the cap's top and presses straight down along the marker (the table takes the push
through the barrel) until the cap clicks home. The left arm then lifts the capped marker, turns it level and lays it
on the table, and both arms draw back.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

CONTROL_HZ = 20


def _rot_axis(axis, angle) -> np.ndarray:
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * K @ K


def _turn_onto(a, b) -> np.ndarray:
    """The smallest rotation taking direction ``a`` onto direction ``b``."""
    a, b = np.asarray(a, float) / np.linalg.norm(a), np.asarray(b, float) / np.linalg.norm(b)
    v = np.cross(a, b)
    s, c = float(np.linalg.norm(v)), float(a @ b)
    if s < 1e-9:
        return np.eye(3)
    return _rot_axis(v, math.atan2(s, c))


class Arm:
    """One arm: smooth Cartesian moves (IK on the URDF), each starting where the last command left it. With
    ``solve=False`` it only records where each move ends (for the offline reach check)."""

    def __init__(self, name: str, base_pos, solve: bool = True):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base, self.solve = name, np.asarray(base_pos, float), solve
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.home = (self.pos.copy(), self.rot.copy())
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.worst_deg = 0.0
        self.ends: list[tuple[np.ndarray, np.ndarray, str]] = []

    def _solve(self, pos, rot):
        from yam_arena import ik

        q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=60)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.worst_deg = max(self.worst_deg, math.degrees(re))
        self.cmd = np.r_[q, self.grip]

    def move(self, pos, rot, duration: float, grip: float | None = None, label: str = ""):
        """Generator: go to ``pos``/``rot`` (world TCP); ``grip`` 1 open .. 0 closed (None keeps it)."""
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        pos = np.asarray(pos, float)
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            p, r = p0 + s * (pos - p0), ik.slerp_matrix(r0, rot, s)
            if self.solve:
                self._solve(p, r)
            else:
                self.pos, self.rot = p, r
            yield
        self.ends.append((pos.copy(), np.asarray(rot, float).copy(), label))

    def set_grip(self, grip: float, duration: float = 0.8):
        yield from self.move(self.pos, self.rot, duration, grip, "grip")


def wait(seconds: float):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    gens = [iter(g) for g in gens]
    while gens:
        for g in list(gens):
            try:
                next(g)
            except StopIteration:
                gens.remove(g)
        if gens:
            yield


# The demo runs open loop: it reads no simulator state. Where the marker stands once it is held upright (its shoulder's
# offset from where it was planned, and its axis's lean), where the cap hangs above it before it goes on (its mouth's
# offset from where it was planned, and its axis's lean), and how far past home the press is commanded (the arms are
# compliant, so a press pushes only as hard as it is commanded past where the cap stops) are measured once with
# DEMO_CALIBRATE=1, which reads those poses from the simulator at those moments, uses them at once and prints them;
# they are then fixed here. The layout is fixed (object_jitter=0) and the simulation deterministic.
CAL = {
    # the marker held tip down for the pull: shoulder (dx, dy, dz) from plan (m); axis (ax, ay)
    "marker_pull": (-0.00057, 0.00063, -0.00208, -0.0116, 0.00706),
    # the marker laid down (dx, dy) and the cap stood on its mouth (dx, dy), from plan (m)
    "laid": (-0.00112, -0.00095, -0.00145, 0.0002),
    # the upright marker: shoulder (dx, dy, dz) from plan (m); axis (ax, ay)
    "marker_up": (-0.00079, -0.00031, 0.00104, -0.00783, -0.00005),
    # the cap above the tip: mouth (dx, dy, dz) from plan (m); axis (ax, ay)
    "cap_over": (0.00042, 0.00056, -0.00122, -0.03068, -0.0073),
    "press_past": 0.010,                      # the press is commanded this far past the cap's top when home (m)
}

CALIBRATE = bool(os.environ.get("DEMO_CALIBRATE"))
MODE = os.environ.get("DEMO_MODE", "")   # "" (the demo), or a control: "nopress", "crooked", "offset"
_MEASURED: dict = {}

TIPS = 0.013               # the TCP sits this far short of the pad tips
GRIP_Z = 0.0155            # picking a part off the table: the TCP this high above the table (pad tips 2.5 mm up)
MARKER_GRIP = -0.015       # the left hand holds the barrel 15 mm behind the shoulder (62 mm up from its back end)
RIGHT_GRIP = -0.020        # the right hand first takes the barrel 20 mm behind the shoulder
STAND = (0.32, -0.03)      # where the marker stands upright (x, y)
PULL = (0.30, 0.03)        # where the right hand holds the marker upright, tip down, for the pull (x, y) ...
PULL_SHOULDER = 0.045      # ... its shoulder this high (the nib's end 15 mm above the table)
PULL_TIPS = 0.010          # taking the posted cap, the left hand's pad tips 10 mm above the cap's mouth
PULL_UP = 0.025            # the cap is pulled 25 mm straight up (the post is 10 mm long)
CAP_SPOT = (0.24, 0.12)    # where the cap is stood on its mouth (x, y)
WAIT = (0.22, -0.10, 0.14) # where the right hand waits with the cap (x, y, height) while the left stands the marker up
CAP_TIPS = 0.010           # taking the standing cap, the right hand's pad tips 10 mm above its mouth
LAY = (0.30, 0.06)         # where the right hand lays the marker down (its shoulder; tip toward -y)
OPEN = 0.27                # fingers open to about 25 mm
# Each hand closes only a little past the part, so it holds it with a light squeeze (the fingers' position control
# pushes about 5 N per mm past contact): a full-force pinch on the light cap squeezes it off-centre and tips it.
CAP_CLOSE = 0.144          # target opening 13.5 mm on the 16.5 mm cap
MARKER_CLOSE = 0.10        # target opening 9.4 mm on the 15 mm barrel (a firmer hold for the press)
REGRASP = 0.008            # the left hand's second, deeper hold on the upright barrel (m)
CAP_LET_GO = 0.005         # the cap is let go of with its mouth this far from the shoulder (m)
LEAN_FIX = 2.0             # the most the cap's lean in the hand is turned out (degrees)


def _local(tcp_pos, tcp_rot, part_pos, part_rot):
    """A part's pose in the TCP frame."""
    return tcp_rot.T @ (np.asarray(part_pos, float) - tcp_pos), tcp_rot.T @ part_rot


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    from markerkit import rotation
    from yam_arena.demo import grip_rotation

    L, R = ctx.left, ctx.right
    T = task
    top = T.TOP
    ey, ez = np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
    down = grip_rotation(math.pi / 2)                           # pointing down, fingers closing along x
    down_l = grip_rotation(-math.pi / 2)                        # the same, the left wrist turned the other way round
    up_left = _rot_axis((1, 0, 0), -math.pi / 2) @ down_l       # pointing along -y: a marker in it stands tip up
    up_right = _rot_axis((1, 0, 0), math.pi / 2) @ down         # pointing along +y: a marker in it hangs tip down
    r_b0 = rotation(T.BARREL_XYZW)
    a_b0 = r_b0[:, 2]                                           # the marker's tip direction as it lies (-y)

    def measure(key, value):
        if CALIBRATE:
            _MEASURED[key] = [round(float(v), 5) for v in value]
            print("CAL so far = " + json.dumps(_MEASURED), flush=True)
        return _MEASURED[key] if CALIBRATE else CAL[key]

    def tcp_for(part_pos, part_axis, p_loc, r_loc, base_rot):
        """The TCP pose that puts a part held at (p_loc, r_loc) at part_pos with its z axis along part_axis."""
        rot = _turn_onto(base_rot @ r_loc[:, 2], part_axis) @ base_rot
        return np.asarray(part_pos, float) - rot @ p_loc, rot

    def pick(arm, at, rot, close, lift=0.06):
        yield from arm.move(at + 0.06 * ez, rot, 3.0, grip=OPEN, label="over it")
        yield from arm.move(at, rot, 1.5, label="fingers round it")
        yield from arm.set_grip(close, 0.8)
        yield from wait(0.3)
        yield from arm.move(at + lift * ez, rot, 1.5, label="lift it")

    # 1. Right: pick the marker up by its barrel, 20 mm behind the shoulder, and turn it upright in the air, tip down,
    #    the posted cap on top.
    b0 = np.array(T.BARREL_START)
    grip_b = b0 + RIGHT_GRIP * a_b0
    grip_b[2] = top + GRIP_Z
    pbr, rbr = _local(grip_b, down, b0, r_b0)
    yield from pick(R, grip_b, down, MARKER_CLOSE)
    if CALIBRATE:
        diag(ctx, "marker picked", ("right", R, "marker"))
    sh_pull = np.array([PULL[0], PULL[1], top + PULL_SHOULDER])
    p, r = tcp_for(sh_pull + 0.03 * ez, -ez, pbr, rbr, up_right)
    yield from R.move(p, r, 3.5, label="marker upright, tip down")
    p, r = tcp_for(sh_pull, -ez, pbr, rbr, up_right)
    yield from R.move(p, r, 1.5, label="hold it for the pull")
    yield from wait(0.4)
    if CALIBRATE:
        s = ctx.states()["marker"]
        a = s.rotation()[:, 2]
        cal = measure("marker_pull", (*(s.pos - sh_pull), a[0], a[1]))
    else:
        cal = CAL["marker_pull"]
    tip_dir = np.array([cal[3], cal[4], -math.sqrt(max(0.0, 1 - cal[3] ** 2 - cal[4] ** 2))])
    step = sh_pull + np.array(cal[:3]) - (T.BODY_L - T.POST_L) * tip_dir   # where the posted cap's mouth sits

    # 2. Left: take the posted cap from above, near its top, and pull it straight up off the back end (about 1.2 N).
    grip_c = step + (PULL_TIPS + TIPS) * ez
    yield from L.move(grip_c + 0.04 * ez, down_l, 3.5, grip=OPEN, label="over the posted cap")
    yield from L.move(grip_c, down_l, 1.5, label="fingers round the cap")
    yield from L.set_grip(CAP_CLOSE, 0.8)
    yield from wait(0.3)
    yield from L.move(grip_c + PULL_UP * ez, down_l, 1.5, label="pull it off")
    if CALIBRATE:
        diag(ctx, "cap pulled off", ("left", L, "cap"))

    # 3. Left: stand the cap on its mouth on the table, let go, and move clear.
    spot = np.array([CAP_SPOT[0], CAP_SPOT[1], top + 0.0005])
    yield from L.move(spot + (PULL_TIPS + TIPS + 0.03) * ez, down_l, 3.0, label="cap over its spot")
    yield from L.move(spot + (PULL_TIPS + TIPS) * ez, down_l, 1.2, label="stand it on its mouth")
    yield from L.set_grip(OPEN, 0.5)
    yield from L.move(L.pos + 0.06 * ez, down_l, 1.2, label="up")
    yield from L.move(*L.home, 2.5, grip=1.0, label="out of the way")

    # 4. Right: lay the marker down on the table, tip to the right, let go and move clear.
    lay0 = np.array([LAY[0], LAY[1], top + T.BODY_R + 0.0005])   # the shoulder
    p, r = tcp_for(lay0 + 0.04 * ez, a_b0, pbr, rbr, down)
    yield from R.move(p, r, 3.5, label="turn it level")
    p, r = tcp_for(lay0, a_b0, pbr, rbr, down)
    yield from R.move(p, r, 1.5, label="lay it down")
    yield from R.set_grip(OPEN, 0.5)
    yield from R.move(R.pos + 0.06 * ez, down, 1.2, label="up")
    if CALIBRATE:
        s = ctx.states()
        m, c = s["marker"], s["cap"]
        cal = measure("laid", (*(m.pos[:2] - lay0[:2]), *(c.pos[:2] - spot[:2])))
    else:
        cal = CAL["laid"]
    lay = lay0 + np.array([cal[0], cal[1], 0.0])
    spot = spot + np.array([cal[2], cal[3], 0.0])

    # 5. Right: take the standing cap from above, near its top (it is now the right way up: mouth down), and wait
    #    with it out of the left hand's way.
    grip_s = spot + (CAP_TIPS + TIPS) * ez
    pc, rc = _local(grip_s, down, spot, np.eye(3))              # the cap stands mouth down: its z axis up
    yield from pick(R, grip_s, down, CAP_CLOSE, lift=0.05)
    yield from R.move(np.array([WAIT[0], WAIT[1], top + WAIT[2]]), down, 2.5, label="wait with the cap")

    # 6. Left: pick the marker up by its barrel, turn it upright, tip up, and stand it on its back end at STAND.
    grip_b = lay + MARKER_GRIP * a_b0
    grip_b[2] = top + GRIP_Z
    pb, rb = _local(grip_b, down_l, lay, r_b0)
    yield from pick(L, grip_b, down_l, MARKER_CLOSE)
    shoulder = np.array([STAND[0], STAND[1], top + T.BODY_L - 0.001])
    p_up, r_up = tcp_for(shoulder + 0.04 * ez, ez, pb, rb, up_left)
    yield from L.move(p_up, r_up, 3.5, label="turn it upright")
    p_st, r_st = tcp_for(shoulder, ez, pb, rb, up_left)
    yield from L.move(p_st, r_st, 2.0, label="stand it on its back end")
    # Picked off the table, the barrel sits at the fingertips. Standing on its end, it is let go of for a moment and
    # taken again 8 mm deeper in the hand, so that it is held firmly for the press.
    yield from L.set_grip(OPEN, 0.5)
    yield from L.move(L.pos + REGRASP * L.rot[:, 2], L.rot, 0.8, label="hand 8 mm further round the barrel")
    yield from L.set_grip(MARKER_CLOSE, 0.6)
    pb = pb - np.array([0.0, 0.0, REGRASP])                     # the barrel now sits 8 mm deeper in the hand
    yield from wait(0.5)
    if CALIBRATE:
        s = ctx.states()["marker"]
        a = s.rotation()[:, 2]
        cal_m = measure("marker_up", (*(s.pos - shoulder), a[0], a[1]))
    else:
        cal_m = CAL["marker_up"]
    shoulder = shoulder + np.array(cal_m[:3])                   # where the shoulder really is
    axis = np.array([cal_m[3], cal_m[4], math.sqrt(max(0.0, 1 - cal_m[3] ** 2 - cal_m[4] ** 2))])

    # 7. Right: bring the cap in line over the tip, 35 mm up (above the nib).
    def cap_tcp(gap, fix):
        mouth = shoulder + gap * axis - np.array(fix[:3])
        if MODE == "offset":           # control: the cap brought down 2 mm to the side of the marker's line
            mouth = mouth + np.array([0.002, 0.0, 0.0])
        lean = np.array([fix[3], fix[4], math.sqrt(max(0.0, 1 - fix[3] ** 2 - fix[4] ** 2))])
        # turn the cap's measured axis toward the marker's, about its mouth, by at most LEAN_FIX degrees
        turn = np.eye(3)
        if fix[3] or fix[4]:
            full = _turn_onto(lean, axis)
            ang = math.acos(max(-1.0, min(1.0, (np.trace(full) - 1) / 2)))
            if ang > 1e-9:
                k = np.array([full[2, 1] - full[1, 2], full[0, 2] - full[2, 0], full[1, 0] - full[0, 1]])
                turn = _rot_axis(k, min(ang, math.radians(LEAN_FIX)))
        if MODE == "crooked":          # control: the cap held 8 degrees off the marker's line
            turn = _rot_axis((0, 1, 0), math.radians(8.0)) @ turn
        p, r = tcp_for(mouth, axis, pc, rc, down)
        r = turn @ r
        p = mouth - turn @ (mouth - p)
        return p, r

    over = cap_tcp(0.035, (0, 0, 0, 0, 0))
    yield from R.move(over[0] + np.array([-0.04, 0.0, 0.0]), over[1], 3.0, label="cap mouth down, beside the tip")
    yield from R.move(*over, 2.0, label="in line over the tip")
    yield from wait(0.5)
    if CALIBRATE:
        diag(ctx, "cap over the tip", ("right", R, "cap"))
        s = ctx.states()["cap"]
        a = s.rotation()[:, 2]
        cal_c = measure("cap_over", (*(s.pos - (shoulder + 0.035 * axis)), a[0], a[1]))
    else:
        cal_c = CAL["cap_over"]

    # 8. Lower it over the nib and onto the nose, its mouth 5 mm short of the shoulder, and let go: it slides down onto
    #    the snap ridge (an unclicked cap stops with its mouth about 3 mm from the shoulder). Draw the fingers up.
    yield from R.move(*cap_tcp(0.035, cal_c), 1.0, label="correct")
    yield from R.move(*cap_tcp(0.020, cal_c), 2.0, label="over the nib")
    yield from R.move(*cap_tcp(CAP_LET_GO, cal_c), 2.0, label="onto the nose")
    yield from wait(0.3)
    yield from R.set_grip(OPEN, 0.6)
    yield from R.move(R.pos + 0.035 * ez, R.rot, 1.2, label="fingers up off the cap")

    if MODE == "nopress":              # control: the cap is left resting on the ridge, unclicked
        yield from R.move(*R.home, 3.0, grip=1.0, label="back")
        yield from L.set_grip(OPEN, 0.6)
        yield from L.move(L.pos - 0.06 * L.rot[:, 2], L.rot, 1.5, label="hand off the marker")
        yield from L.move(*L.home, 3.0, grip=1.0, label="back")
        return

    # 9. Right: closed fingertips down onto the cap's top, then press straight down along the marker until it clicks
    #    home; the table takes the push through the barrel.
    cap_top_home = shoulder + T.CAP_L * axis
    above = cap_top_home + (TIPS + 0.009) * axis              # 6 mm above the top of the cap resting on the ridge
    yield from R.move(above, down, 1.5, grip=0.0, label="over the cap")
    yield from R.move(cap_top_home + (TIPS + 0.005) * axis, down, 1.2, label="fingertips on the cap")
    yield from R.move(cap_top_home + (TIPS - CAL["press_past"]) * axis, down, 2.0, label="press it home")
    yield from wait(0.5)
    yield from R.move(above, down, 1.5, label="off the cap")
    yield from R.move(*R.home, 3.0, grip=1.0, label="back")

    # 10. Left: take the marker again 8 mm less deep (back at the fingertips, as it was picked, so the fingers stay
    #     clear of the table when it is laid down), lift the capped marker, turn it level and lay it on the table, let
    #     go and draw back.
    yield from L.set_grip(OPEN, 0.5)
    yield from L.move(L.pos - REGRASP * L.rot[:, 2], L.rot, 0.8, label="hand back to the fingertips")
    yield from L.set_grip(MARKER_CLOSE, 0.6)
    pb = pb + np.array([0.0, 0.0, REGRASP])
    yield from L.move(L.pos + 0.04 * ez, L.rot, 1.5, label="lift it")
    lay_end = np.array([STAND[0], STAND[1] + 0.03, top + T.CAP_R + 0.0005])   # the shoulder, the tip toward -y
    p_lay, r_lay = tcp_for(lay_end + 0.04 * ez, -ey, pb, rb, down_l)
    yield from L.move(p_lay, r_lay, 3.0, label="turn it level")
    p_lay, r_lay = tcp_for(lay_end, -ey, pb, rb, down_l)
    yield from L.move(p_lay, r_lay, 1.5, label="lay it down")
    yield from L.set_grip(OPEN, 0.6)
    yield from L.move(L.pos + 0.06 * ez, L.rot, 1.2, label="up")
    yield from L.move(*L.home, 3.0, grip=1.0, label="back")


def diag(ctx, when: str, *holds) -> None:
    """Calibration only: each held part's axis in its hand's actual TCP frame, and the hand's TCP error."""
    st = ctx.states()
    for side, arm, part in holds:
        p, r = ctx.tcp(side)
        rp = st[part].rotation()
        ang = math.degrees(math.acos(max(-1.0, min(1.0, float(np.trace(arm.rot.T @ r) - 1) / 2))))
        print(f"DIAG {when}: {side} TCP off {np.round(1000 * (p - arm.pos), 1).tolist()} mm, turned {ang:.1f} deg; "
              f"{part} axis in TCP {np.round(r.T @ rp[:, 2], 3).tolist()}, origin in TCP "
              f"{np.round(1000 * (r.T @ (st[part].pos - p)), 1).tolist()} mm", flush=True)


def offline_plan(task):
    """The demo's moves without the simulator: each arm's list of (pos, rot, label) at the end of every move."""
    from yam_arena import station

    class _Ctx:
        left = Arm("left", station.BASES["left"], solve=False)
        right = Arm("right", station.BASES["right"], solve=False)

    ctx = _Ctx()
    steps = sum(1 for _ in script(task, ctx))
    return {"left": ctx.left.ends, "right": ctx.right.ends, "steps": steps}


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def tcp(self, side: str):
        """The actual TCP pose of one arm (world position, rotation) from the simulator."""
        from markerkit import as_numpy, rotation
        from yam_arena.constants import TCP_OFFSET

        base = self.emb._env.unwrapped
        robot = base.scene[f"{side}_robot"]
        origin = as_numpy(base.scene.env_origins[0])
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        j = [n.rsplit("/", 1)[-1] for n in robot.body_names].index("gripper")
        r = rotation(poses[j][3:7])
        return poses[j][:3] - origin + r @ np.array(TCP_OFFSET), r

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


def run(task_path: str, out_dir: str, max_steps: int = 2000, record: bool = True, cam_width: int = 640,
        cam_height: int = 352, hold_s: float = 6.0, still: bool = False) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction=emb.task.module.INSTRUCTION))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = iter(()) if still else script(task, ctx)   # still: a control run, the arms stay where they start
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info, first_full, best = 0, None, False, {}, None, 0.0
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        sc = float(info.get("score", 0.0) or 0.0)
        best = max(best, sc)
        if first_full is None and sc >= 1.0:
            first_full = steps
        snap(res.observation)
        if steps % 100 == 0:
            print(f"step {steps} score {info.get('score')}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    # The trial's score is what the task file's score() reported on the last step (task_score gives 1.0 on success,
    # which score() also is then); it is never set to 1.0 here just because the episode ended.
    final = float(info.get("score", 0.0) or 0.0)
    result = {"task_score": final, "success": success, "steps": steps, "first_full_score_step": first_full,
              "script_ended_step": ended,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)},
              "worst_ik_deg": {"left": round(ctx.left.worst_deg, 2), "right": round(ctx.right.worst_deg, 2)}}
    if CALIBRATE:
        result["CAL"] = _MEASURED
    if record:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((c for c in frames if "top" in c), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
    (out / "result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--cam", default="640x352", help="camera width x height")
    ap.add_argument("--still", action="store_true", help="control run: the arms do not move")
    a = ap.parse_args()
    w, h = (int(v) for v in a.cam.split("x"))
    run(a.task, a.out, a.max_steps, record=not a.no_record, cam_width=w, cam_height=h, still=a.still)
