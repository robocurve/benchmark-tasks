"""Scripted full-score demo for the reacher-grabber litter task, run in the yam_arena embodiment.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record]

The left gripper holds the tool's handle behind the grip and the right gripper holds the trigger. Both point straight
down with their fingers across the tool, so the tool can swing about the fingers' axis: its pitch is set by holding the
trigger end higher or lower than the handle end. For each piece of litter the arms swing the tool's jaws over it, lower
them around it, pull the trigger (the right gripper slides back along the tool), lift and swing the tool over the
bin with the trigger held, and let the trigger go so the piece drops in.

The script runs open loop: every move is planned from the task's published starting layout (task.py constants), with
offsets calibrated once in this deterministic simulation for how far the compliant arms let the tool droop.
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
TIP = 0.013          # the TCP sits this far above the pad tips


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class Pose:
    """A pose of the tool: its origin (on the shaft's axis, 3 cm in front of the handle's back end), yaw, pitch (positive: jaws down) and
    how far the trigger is pulled (m)."""

    def __init__(self, o, yaw, pitch, pull=0.0):
        self.o, self.yaw, self.pitch, self.pull = np.asarray(o, float), float(yaw), float(pitch), float(pull)

    def R(self):
        return Rz(self.yaw) @ Ry(self.pitch)

    def world(self, p):
        return self.o + self.R() @ np.asarray(p, float)

    def lerp(self, other, s):
        return Pose(self.o + s * (other.o - self.o), self.yaw + s * (other.yaw - self.yaw),
                    self.pitch + s * (other.pitch - self.pitch), self.pull + s * (other.pull - self.pull))


# The handle gripper holds the housing firmly enough that the tool turns with it: it leans back LEAN_L from the tool's
# vertical (45 degrees from upright with the tool resting on the table), so the tool's pitch is set by that wrist. The
# trigger gripper points straight down and lets the trigger pivot between its fingers. Leaning the handle gripper back
# keeps the two wrists apart.
LEAN_L = math.radians(float(os.environ.get("LEAN_L", 53.3)))
LEAN_R = 0.0


def grip_rot(yaw, lean):
    """A gripper pointing down with its fingers across the tool (turned ``yaw``), its top leaned ``lean`` toward the
    tool's -x (positive) about the fingers' closing axis."""
    from yam_arena.demo import grip_rotation

    return Rz(yaw) @ Ry(-lean) @ grip_rotation(0.0)


def grip_targets(T, pose: Pose):
    """World TCP poses (position, rotation) of the left (handle) and right (trigger) grippers holding the tool at
    ``pose``: their pad tips on the given points of the handle and the trigger."""
    out = []
    for tip, lean in (((T.GRIP_X, 0.0, T.GRIP_PAD_Z), LEAN_L - pose.pitch), ((T.TRIGGER_X - pose.pull, 0.0, T.TRIGGER_PAD_Z),
                                                                           -LEAN_R)):
        rot = grip_rot(pose.yaw, lean)
        approach = rot @ np.array([0.0, 0.0, 1.0])
        out.append((pose.world(tip) - TIP * approach, rot))
    return out


def clamp_pose(T, point, yaw, pitch, local=None, pull=0.0):
    """The tool pose that puts the tool-frame point ``local`` (default: the jaws' middle) at the world ``point``."""
    local = T.CLAMP if local is None else np.asarray(local, float)
    R = Rz(yaw) @ Ry(pitch)
    return Pose(np.asarray(point, float) - R @ local, yaw, pitch, pull)


def aim_pose(T, point, pitch, local=None, oy=0.0, pull=0.0):
    """Like clamp_pose, with the yaw chosen so the tool's origin is at y = ``oy`` (between the arms)."""
    local = T.CLAMP if local is None else np.asarray(local, float)
    reach = (Ry(pitch) @ local)[0]
    yaw = math.asin(np.clip((point[1] - oy) / reach, -1, 1))
    return clamp_pose(T, point, yaw, pitch, local, pull)


class Arm:
    """One arm: IK on the URDF, each solve starting from the last."""

    def __init__(self, name: str, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.cmd = np.r_[self.q, self.grip]
        self.worst_mm = 0.0
        self.flip = 0.0

    def solve(self, pos, rot, iters=60):
        from yam_arena import ik

        q, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), self.q, base_pos=self.base, iters=iters)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), rot
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.cmd = np.r_[q, self.grip]
        return pe

    def move(self, pos, rot, duration: float, grip: float | None = None):
        from yam_arena import ik

        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.solve(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, rot, s))
            yield


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


# -- the plan --------------------------------------------------------------------------------------------------------
# Calibrated once in this simulation with DEMO_CALIBRATE=1 (which corrects each move closed loop on the simulator's
# tool pose and prints the offsets): the arms are compliant, and under the tool's weight the trigger hand, which carries
# most of it, sags while the handle hand rises, so the jaws droop. Each move's command is offset by (dx, dy, dz) m on
# the tool's origin and dpitch degrees (positive: jaws further down); unlisted moves are not offset.
# The cup's moves, at the edge of the arms' reach, keep the whole sequence of corrections the calibration stepped
# through (a list: the offset during the move, then one per half-second correction step), replayed as recorded;
# DEMO_CALIBRATE=1 DEMO_CALIBRATE_ONLY=cup recalibrates just those.
CAL = {
    "lift": (-0.0006, -0.0081, 0.0044, -1.58),
    "can high": (-0.0029, -0.0061, 0.0019, -3.02),
    "can above": (-0.0019, -0.0036, 0.0058, -3.66),
    "can pick": (-0.0019, -0.0043, 0.0048, -3.68),
    "can pull": (0.0131, -0.0022, 0.0107, -0.23),
    "can lift": (0.0086, 0.0004, 0.0118, -2.55),
    "can carry": (0.0056, 0.0042, 0.0119, -2.98),
    "can drop": (0.0040, -0.0031, 0.0136, -2.94),
    "can release": (-0.0136, 0.0015, 0.0074, -9.99),
    "bottle high": (-0.0110, -0.0046, -0.0072, -11.88),
    "bottle above": (-0.0082, -0.0063, -0.0006, -10.00),
    "bottle pick": (0.0001, -0.0096, 0.0132, -2.57),
    "bottle pull": (0.0126, 0.0056, 0.0127, -1.76),
    "bottle lift": (0.0101, -0.0013, 0.0096, -2.58),
    "bottle carry": (0.0073, -0.0031, 0.0127, -3.52),
    "bottle drop": (0.0089, 0.0030, 0.0134, -2.21),
    "bottle release": (-0.0121, -0.0020, 0.0074, -6.36),
    "cup high": [   # residual [ 0.2  0.8 -3.5] mm 0.1 deg, ik 0/19 mm
        (0.0, 0.0, 0.0, 0.0),
        (-3.16438407053532e-05, -0.0004213799361988402, 0.04, -4.247006492961304),
        (0.003440839731805934, -0.009778116710382912, 0.028194874217626308, -5.472340253907843),
        (0.005886079991276195, -0.006808083423454471, 0.01792812314867774, -5.875557993383747),
        (0.006736258550082841, -0.0052378957742361685, 0.012983854981480433, -5.948256735984994),
        (0.0069326779130178375, -0.004637513328029598, 0.01018598329183021, -5.8683202441027)],
    "cup above": [   # residual [ -6.3 -13.4   2.7] mm 1.8 deg, ik 0/0 mm
        (0.0, 0.0, 0.0, 0.0),
        (-0.0037217374157422437, -0.003438475743115177, -0.02848495754509548, 3.34591626057277),
        (-0.003218199525559751, -0.010379406986559103, -0.04, 8.21371779811168),
        (-0.0021713406639342738, -0.020719292074151508, -0.04, 12.58774491205407),
        (-0.007258825245329615, -0.03211438189646994, -0.037740777053631776, 13.988566835364711),
        (-0.012262018032232369, -0.04, -0.03560536796688179, 15.0)],
    "cup pick": [   # residual [-0.6 -0.   3.4] mm 0.9 deg, ik 0/0 mm
        (0.0, 0.0, 0.0, 0.0),
        (0.005640986868840159, -0.009699822541286963, -0.021096111685091047, 3.4172638210447426),
        (0.005826091059715922, -0.008614264672658446, -0.018207619749832028, 4.137871894938888),
        (0.005552625236668974, -0.00746103665944955, -0.015271374800435035, 4.853665638439763),
        (0.005050061009906948, -0.006965891609023968, -0.012415188932884914, 5.577311640254856),
        (0.004602445985170523, -0.006999101138005282, -0.009657800312331589, 6.310487175359489)],
    "cup pull": [   # residual [-1.5  4.4  1.2] mm 1.2 deg, ik 0/0 mm
        (0.0, 0.0, 0.0, 0.0),
        (0.009673997221038455, -0.0009150287523266298, -0.012111684698874737, 2.161550805422234),
        (0.013468939436183547, 0.004927909630078786, -0.011182453568397933, 3.132205478992578),
        (0.01296357379587949, 0.010114528432713921, -0.01024678785523543, 4.087485676551126),
        (0.011781902396463283, 0.015495938095208352, -0.009316016726078758, 5.027412117607627),
        (0.010550237467930044, 0.01902712685851127, -0.008382020282245595, 5.954185618637389)],
    "cup lift": [   # residual [-0.1  0.2  0. ] mm -0.1 deg, ik 1/0 mm
        (0.0, 0.0, 0.0, 0.0),
        (0.0015275888914087156, 0.018811583650062105, 0.009810164090368457, 0.022758189352325892),
        (0.0017546659747616823, 0.021447238031784856, 0.012691343089886866, -0.006690337754371212),
        (0.0016524085136898849, 0.022055740744399824, 0.013216798869804782, -0.04467795326759741),
        (0.001570989931565503, 0.022229069368070856, 0.013431991713561955, -0.09166264589896009),
        (0.001485622407518683, 0.02236371908944437, 0.013439116231658945, -0.13825674054361423)],
    "cup carry": [   # residual [ 0.1 -0.2 -1.5] mm -0.1 deg, ik 1/0 mm
        (0.0, 0.0, 0.0, 0.0),
        (0.007487439256357398, -0.0063055676972103795, 0.019835884640702607, -0.7958366842468737),
        (0.0096993099578782, -0.007721007428294824, 0.02023393618678959, -1.1490446291437972),
        (0.010053399573700262, -0.0063921953624963114, 0.018370109320305073, -1.3019910145588014),
        (0.010006734186838529, -0.005732065799251873, 0.01856419276518446, -1.5077497822026653),
        (0.010074018995360668, -0.005853707801767038, 0.01738972186275216, -1.5764149901014355)],
    "cup drop": [   # residual [-0.3 -0.3 -0.7] mm -0.0 deg, ik 0/0 mm
        (0.0, 0.0, 0.0, 0.0),
        (0.0028032023829686285, -0.002913915628295394, 0.01892496834784421, -0.5566964593231413),
        (0.003343532467459554, -0.003692887389571409, 0.019752443846298286, -0.7656708764933049),
        (0.0034524932311812864, -0.003955096499114453, 0.01895187151645779, -0.8634927682052357),
        (0.00349046846128811, -0.003954974489855046, 0.018357060479893498, -0.9196968503491075),
        (0.003284250403936362, -0.004183889971071376, 0.01779795317214195, -0.9275631598579267)],
}

SWING_XY = {"can": (0.20, -0.04), "bottle": (0.20, -0.04), "cup": (0.22, 0.0)}   # the tool's origin while it swings
ITEM_OY = {"can": 0.0, "bottle": 0.0, "cup": 0.04}  # tool origin's y while picking (the cup: nearer the handle arm)
DROP_OY = -0.06                   # the tool's origin this far right of centre while over the bin
PULL_PAST = 0.040                 # command the trigger hand this far past the trigger's stop (the arm gives ~5 mm/N)
PICK_Z = 0.145                    # the tool's origin this high above the table while picking
CARRY_Z = 0.15                    # ... and while carrying
JAW_CLEAR = 0.004                 # the jaws' lowest corner this far above the table when picking
LIFT_Z = 0.25                     # the jaws' middle this high while swinging, and over the bin (its rim is 17.8 cm up)
HIGH_PULLED = ("cup",)            # pieces swung toward with the trigger held (let go on the way down onto them)
APPROACH = {"can": 0.06, "bottle": 0.07, "cup": 0.05}   # come down onto a piece from this far above
BOTTLE_AHEAD = 0.0635             # the bottle's centre this far ahead of the jaws' middle (its base 1 cm off the head)
CUP_AHEAD = 0.010                 # the cup's centre this far ahead of the jaws' middle


def item_point(T, name):
    """Where a piece is (world, its centre) and which tool-frame point should hold it there; and the yaw the tool must
    have (None: any)."""
    if name == "bottle":   # held across its body, base end 1 cm in front of the jaw head, the rest sticking out ahead
        return np.array(T.BOTTLE.pos), T.CLAMP + (BOTTLE_AHEAD, 0.0, 0.0), T.BOTTLE.yaw
    if name == "cup":      # centred a little ahead of the jaws' middle, clear of the jaw head
        return np.array(T.CUP.pos), T.CLAMP + (CUP_AHEAD, 0.0, 0.0), None
    return np.array(T.CAN.pos), T.CLAMP, None


def pose_at(T, point, local, o_z, yaw=None, pull=0.0, oy=0.0) -> Pose:
    """The tool pose with the tool-frame point ``local`` at the world ``point`` and the tool's origin at height
    ``o_z``: the pitch follows from the two heights, the yaw (unless given) puts the tool's origin at y 0."""
    lo, hi = math.radians(-50), math.radians(50)
    for _ in range(60):   # the point's height falls as the pitch grows
        mid = (lo + hi) / 2
        if o_z + (Ry(mid) @ local)[2] > point[2]:
            lo = mid
        else:
            hi = mid
    pitch = (lo + hi) / 2
    if yaw is None:
        return aim_pose(T, point, pitch, local, oy=oy, pull=pull)
    return clamp_pose(T, point, yaw, pitch, local, pull)


def jaw_low_z(T, pose: Pose) -> float:
    """The lowest point of the jaws (their four lower corners)."""
    pts = [pose.world((x, y, T.JAW_FOOT[1])) for x in (0.48, 0.58) for y in (-0.071, 0.071)]
    return min(p[2] for p in pts)


def pick_pose(T, name, pull=0.0) -> Pose:
    """Jaws around the piece, their lowest corner JAW_CLEAR above the table, the tool's origin PICK_Z up."""
    point, local, yaw = item_point(T, name)
    target = point.copy()
    for _ in range(4):
        pose = pose_at(T, target, local, T.TOP + PICK_Z, yaw, pull, ITEM_OY[name])
        target[2] += T.TOP + JAW_CLEAR - jaw_low_z(T, pose)
    return pose_at(T, target, local, T.TOP + PICK_Z, yaw, pull, ITEM_OY[name])


def above_pose(T, name, dz, o_z, pull=0.0) -> Pose:
    point, local, yaw = item_point(T, name)
    return pose_at(T, point + (0.0, 0.0, dz), local, T.TOP + o_z, yaw, pull, ITEM_OY[name])


def drop_pose(T, name, pull) -> Pose:
    _, local, _ = item_point(T, name)
    target = np.array([T.BIN_XY[0], T.BIN_XY[1], T.TOP + LIFT_Z])
    pitch = pose_at(T, target, local, T.TOP + CARRY_Z, None).pitch
    return aim_pose(T, target, pitch, local, oy=DROP_OY, pull=pull)


def swing_pose(T, yaw, clamp_z, pull=0.0, name="can") -> Pose:
    """The tool's origin at SWING_XY[name], CARRY_Z up; the tool turned to ``yaw`` and pitched so the jaws' middle is
    ``clamp_z`` above the table."""
    lo, hi = math.radians(-50), math.radians(50)
    for _ in range(60):
        mid = (lo + hi) / 2
        if CARRY_Z + (Ry(mid) @ T.CLAMP)[2] > clamp_z:
            lo = mid
        else:
            hi = mid
    sx, sy = SWING_XY[name]
    return Pose(np.array([sx, sy, T.TOP + CARRY_Z]), yaw, (lo + hi) / 2, pull)


def rest_pose(T) -> Pose:
    return Pose(np.array(T.TOOL.pos), T.TOOL_YAW, T.REST_PITCH)


def script(task, ctx):
    """The demo as a generator: one yield per control step."""
    T = task
    L, R = ctx.left, ctx.right
    state = {"pose": rest_pose(T), "corr": np.zeros(4)}
    up = np.array([0.0, 0.0, 0.06])
    calibrate = bool(os.environ.get("DEMO_CALIBRATE"))

    def command(pose: Pose, corr) -> Pose:
        return Pose(pose.o + corr[:3], pose.yaw, pose.pitch + math.radians(corr[3]), pose.pull)

    def track(pose: Pose, corr):
        (lt, lr), (rt, rr) = grip_targets(T, command(pose, corr))
        L.solve(lt, lr)
        R.solve(rt, rr)

    def tool_to(pose: Pose, duration: float, key=None):
        """Move the tool to ``pose``, commanding it offset by the calibrated correction for ``key``."""
        from yam_arena import ik

        start, c0 = state["pose"], state["corr"]
        entry = CAL.get(key, (0.0, 0.0, 0.0, 0.0))
        seq = [np.array(c, float) for c in entry] if isinstance(entry, list) else [np.array(entry, float)]
        c1 = seq[0]
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            track(start.lerp(pose, s), c0 + s * (c1 - c0))
            yield
        for c_new in seq[1:]:   # a recorded correction sequence: settle, then step to the next offset, as calibrated
            yield from wait(0.5)
            for k in range(1, 11):
                track(pose, c1 + k / 10 * (c_new - c1))
                yield
            c1 = c_new
        only = os.environ.get("DEMO_CALIBRATE_ONLY", "")   # e.g. "cup": calibrate just those moves
        if calibrate and key and key.startswith(only) and not (only and key in CAL):
            # measure where the tool is and correct, to find the offsets fixed in CAL
            steps = [c1.copy()]
            for _ in range(5):
                yield from wait(0.5)
                t = ctx.states()["picker"]
                r = t.rotation()
                pitch = math.asin(-np.clip(r[2, 0], -1, 1))
                err = np.r_[pose.world(T.CLAMP) - t.to_world(T.CLAMP), math.degrees(pose.pitch - pitch)]
                c_new = np.clip(c1 + 0.8 * err, (-0.04, -0.04, -0.04, -15.0), (0.04, 0.04, 0.04, 15.0))
                for k in range(1, 11):
                    track(pose, c1 + k / 10 * (c_new - c1))
                    yield
                c1 = c_new
                steps.append(c1.copy())
            seq_txt = ", ".join("(" + ", ".join(repr(float(v)) for v in c) + ")" for c in steps)
            print(f"CAL {key!r}: [{seq_txt}],  # residual "
                  f"{np.round(1000 * err[:3], 1)} mm {err[3]:.1f} deg, ik {L.worst_mm:.0f}/{R.worst_mm:.0f} mm", flush=True)
            L.worst_mm = R.worst_mm = 0.0
        state["pose"], state["corr"] = pose, c1

    # 1. Both grippers come down on the resting tool: the left across the handle housing behind the grip, the right
    #    across the trigger, and close.
    rest = rest_pose(T)
    (lt, lr), (rt, rr) = grip_targets(T, rest)
    la = lr @ np.array([0.0, 0.0, 1.0])
    yield from both(L.move(lt - 0.08 * la, lr, 2.0, grip=1.0), R.move(rt + 1.5 * up, rr, 2.0, grip=1.0))
    yield from both(L.move(lt, lr, 1.5), R.move(rt, rr, 1.5))
    yield from both(L.move(lt, lr, 0.8, grip=0.0), R.move(rt, rr, 0.8, grip=0.0))
    yield from wait(0.3)

    # 2. Lift the tool off the table.
    lift = Pose(rest.o + (0.04, 0.0, 0.04), rest.yaw, rest.pitch)
    yield from tool_to(lift, 1.5, "lift")
    pull = T.TRIG_TRAVEL + PULL_PAST   # past the trigger's stop, so the compliant arm still holds it there

    for name in ("can", "bottle", "cup"):
        # 3. Swing the open jaws over the piece (high, clear of everything), come down around it, pull the trigger.
        drop = drop_pose(T, name, pull)
        bearing = above_pose(T, name, APPROACH[name], PICK_Z).yaw
        # jaws up, clear of the bin's rim, turned toward the piece; for the cup, far to the left, the trigger is held
        # (jaws shut, empty) on the way, which keeps the trigger hand within reach, and let go on the way down
        high = swing_pose(T, bearing, LIFT_Z, pull if name in HIGH_PULLED else 0.0, name)
        yield from tool_to(high, 2.0, f"{name} high")
        yield from tool_to(above_pose(T, name, APPROACH[name], PICK_Z), 1.5, f"{name} above")
        yield from tool_to(pick_pose(T, name), 1.5, f"{name} pick")
        yield from wait(0.3)
        yield from tool_to(pick_pose(T, name, pull), 1.0, f"{name} pull")
        yield from wait(0.5)
        # 4. Lift it, swing it over the bin and let the trigger go.
        yield from tool_to(above_pose(T, name, 0.10, PICK_Z, pull), 1.2, f"{name} lift")
        carry = swing_pose(T, bearing, LIFT_Z, pull, name)    # jaws up at the height they cross the bin's rim
        yield from tool_to(carry, 1.2, f"{name} carry")
        yield from tool_to(drop, 2.0, f"{name} drop")
        yield from wait(0.5)
        yield from tool_to(drop_pose(T, name, 0.0), 0.8, f"{name} release")
        yield from wait(1.0)

    # 5. Put the tool down on the table and let go.
    yield from tool_to(lift, 2.5, "lift")
    yield from tool_to(rest, 1.5)
    yield from both(L.move(L.pos, L.rot, 0.8, grip=0.6), R.move(R.pos, R.rot, 0.8, grip=0.6))
    yield from both(L.move(L.pos + up, L.rot, 1.0), R.move(R.pos + up, R.rot, 1.0))


class Ctx:
    def __init__(self, emb):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])

    def states(self):   # for the debug printout only; the script never reads it
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


def _debug(task, ctx, steps, info):
    st = ctx.states()
    t = st["picker"]
    r = t.rotation()
    pitch = math.degrees(math.asin(-np.clip(r[2, 0], -1, 1)))
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    msg = (f"step {steps} score {info.get('score')} | tool o {np.round(t.pos - [0, 0, task.TOP], 3)} yaw {yaw:.1f} "
           f"pitch {pitch:.1f} trig {1000 * t.sliders['trigger']:.1f} jaws {1000 * t.sliders['jaw_l']:.1f}/"
           f"{1000 * t.sliders['jaw_r']:.1f} clamp {np.round(t.to_world(task.CLAMP) - [0, 0, task.TOP], 3)}")
    for n in task.LITTER:
        msg += f" | {n} {np.round(st[n].pos - [0, 0, task.TOP], 3)}"
    msg += f" | touched {task._STATE['touched']} lifted {task._STATE['lifted']}"
    from litterkit import gripper_poses

    g = gripper_poses(ctx.emb._env)
    for side, arm, pt in (("left", ctx.left, (task.GRIP_X, 0, task.GRIP_PAD_Z)),
                          ("right", ctx.right, (task.TRIGGER_X + t.sliders["trigger"], 0, task.TRIGGER_PAD_Z))):
        pos, rot, finger = g[side]
        tcp = pos + rot @ np.array([0.0, 0.0, 0.134])
        tip = tcp + rot @ np.array([0.0, 0.0, 0.013])
        # the gripper's approach axis in the tool frame: its angle in the tool's x-z plane, actual vs commanded
        a_act = t.rotation().T @ (rot @ np.array([0.0, 0.0, 1.0]))
        a_cmd = t.rotation().T @ (arm.rot @ np.array([0.0, 0.0, 1.0]))
        rel = math.degrees(math.atan2(a_act[0], -a_act[2]))
        rel_cmd = math.degrees(math.atan2(a_cmd[0], -a_cmd[2]))
        msg += (f" | {side} tip-vs-target(tool frame) {np.round(1000 * t.to_local(tip) - 1000 * np.array(pt), 0)}mm"
                f" lag {np.round(1000 * (tcp - arm.pos), 0)}mm finger {1000 * finger:.1f}"
                f" grip-vs-tool {rel:.1f} (cmd {rel_cmd:.1f})deg")
    print(msg, flush=True)


def run(task_path: str, out_dir: str, max_steps: int = 2400, record: bool = True, cam_width: int = 320,
        cam_height: int = 176, hold_s: float = 4.0) -> dict:
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    task_path = str(Path(task_path).resolve())
    sys.path.insert(0, str(Path(task_path).parent))
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=task_path, object_jitter=0, control_hz=CONTROL_HZ, cam_width=cam_width,
                               cam_height=cam_height)
    obs = emb.reset(Scene(id="demo", instruction="demo"))
    task = emb.task.module
    ctx = Ctx(emb)
    gen = script(task, ctx)
    frames: dict[str, list] = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, success, info = 0, None, False, {}
    shots = os.environ.get("DEMO_SHOTS")
    while steps < max_steps:
        try:
            next(gen)
        except StopIteration:
            ended = steps if ended is None else ended
        res = emb.step(ctx.action())
        steps += 1
        info = res.info
        snap(res.observation)
        if shots and steps % int(shots) == 0:
            import imageio

            for cam, img in res.observation.images.items():
                imageio.imwrite(out / f"shot_{steps:04d}_{cam}.png", np.asarray(img))
        if steps % 100 == 0 or (os.environ.get("DEMO_DEBUG") and steps % 20 == 0):
            if os.environ.get("DEMO_DEBUG"):
                _debug(task, ctx, steps, info)
            else:
                print(f"step {steps} score {info.get('score')}", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= hold_s * CONTROL_HZ:
            break
    score = 1.0 if success else float(info.get("score", 0.0) or 0.0)
    result = {"task_score": score, "success": success, "steps": steps,
              "worst_ik_mm": {"left": round(ctx.left.worst_mm, 2), "right": round(ctx.right.worst_mm, 2)}}
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
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, a.max_steps, record=not a.no_record)
