"""The scripted demo, closed loop on robot-observable inputs only.

What it reads, every control step: the observation Inspect Robots hands any policy, i.e. the three RGB images
(``top_cam``, ``left_cam``, ``right_cam``) and the 14-D ``joint_pos`` state. Nothing else comes from the simulator:
no part poses, no prop joint angles (thread, spring), no scorer state. Fixed knowledge it uses: the documented setup
(task.json ``setup``: where the channel, its red mark and the parts blocks are; the parts' sizes: channel section,
nut, bolt, fitting; the thread: 13 per inch, it catches within the first eighth of a turn), the station's published
camera calibration and the arm's URDF (``yam_arena.station``, ``yam_arena.urdf``), as a real robot's script would.

Plan (arm A = the left arm holds the nut; arm B = the right arm does the rest):
1. Top camera: the red tape on the channel's near lip gives the mark's position along the channel; once the hand
   holds the kit over the channel, the right wrist camera sees the tape at a finer scale and refines it.
2. Arm A, fingers level and pointing forward, takes the spring nut off its post by its two ends (its body, below its
   top face) and holds it up, top face level, 5.5 cm above the post. Arm B takes the fitting (the bolt dropped
   through its hole) off its block by its left and right edges, lowers it over the nut until the bolt's end lands in
   the nut's hole and the bolt rises in the fitting's hole, and lets go 3 mm above the nut.
3. Arm B takes the bolt's head (fingers down, across two flats), presses lightly and starts it by hand: strokes of a
   third of a turn clockwise (grip, turn while following the thread down, let go, turn back, grip), the bolt held
   square to the nut by the hand, until the wrist's measured roll says it has been turned enough for two and a half
   turns of thread (the first eighth of a turn finds the thread's start; the head's height for each grip follows
   from the turns and 13 threads per inch).
4. Arm B grips the head; arm A lets go of the nut, backs out and parks to its side; arm B lifts the kit (the nut hanging on the bolt,
   the fitting resting on the nut, the spring below) to over the channel 5 cm right of the mark and lowers it: the
   spring and then the nut go edge-on into the slot, the fitting lands on the lips; it pushes the head down until
   the nut's top is 3 mm below the lips (the spring pushes back, about 6 N; the measured hand height shows when it
   is there).
5. It turns the head a third of a turn clockwise: below the lips the nut turns with the bolt until its square corners
   stop on the walls (a quarter turn), then the bolt turns in it. It eases the push off: the spring lifts the nut, its
   grooves close on the lips.
6. Holding the head, it slides the bolt (and with it the fitting and the nut) along the channel to the mark, the
   bolt's axis placed from the arm's measured joints (forward kinematics). It lets go and squares the fitting: fingers
   down beside the channel's walls, closed across the channel on the fitting's two edges (the fitting is as wide as
   the channel), then opened.
7. It runs the bolt down in strokes (before each grip it waits until the measured hand is within 0.6 mm of the
   head's position, so closing the fingers does not push the kit along the channel): grip across two flats, a third
   of a turn clockwise, let go, turn back, grip again. The bolt's turns are counted from the wrist's measured roll
   while the fingers hold the head; the head's height for the next grip follows from the turns and the 13 threads
   per inch. Once the counted turns reach the head's documented travel to the fitting, it gives one more stroke and
   stops when the wrist's measured roll lags its command by WRIST_SNUG_LAG more than in the free-running strokes
   (the thread resisting: hand-snug), never more than MAX_EXTRA strokes past the documented travel.
8. It lets go and moves away; both arms stay clear.

``-P skip_turn=1`` (control) leaves out the quarter turn in step 5: the nut stays edge-on in the slot and the bolt
is run down on it all the same. ``-P tilt_deg=<d>`` (hazard check) holds the bolt tilted d degrees about the
channel's direction (its end kept on the nut's axis) while it starts it, then lets go and stops. ``SNC_DEMO_DEBUG=<n>`` prints estimates every n steps.

A small joint-space integral trim (proprioception only) removes the station arms' steady error of a few hundredths of
a radian under their soft gains.
"""

from __future__ import annotations

import math
import os

import numpy as np

from yam_arena import ik, station
from yam_arena.ik import matrix_to_quat_xyzw

from strutnut_sim import spec as S
from strutnut_sim import vision as V
from strutnut_sim.kit import CLOSED, DOWN, FORWARD, ROOT_OFFSET, ArmTimeline, conj, q_axis, qmul

Q_REL = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
TCP_EXTRA = 0.1347 - 0.134
PAD_TIP = 0.013 + TCP_EXTRA
"""Pad tips below our grasp point."""
RED = ((235, 12), (120, 255), (60, 255))
GRIP_OPEN = 0.55
TRIM_GAIN, TRIM_MAX = 0.05, 0.06
STROKE = 2 * math.pi / 3
"""One stroke: a third of a turn (the hex's flats line up with the fingers again)."""
YAW0 = math.radians(60.0)
"""Hand yaw at the start of a stroke; a stroke turns it to -60 deg (clockwise seen from above)."""
TIP_ABOVE_HEAD_BOTTOM = 0.0012
WRIST_SNUG_LAG = math.radians(1.0)
"""Wrist roll lag at a stroke's end beyond the free-running strokes' lag that says the thread is resisting."""
MAX_EXTRA = 3
PUSH_BELOW = 0.003
"""How far below the lip returns the nut's top is pushed before it is turned."""
TUCK_Q = (0.0, 0.05, 0.05, 0.0, 0.0, 0.0)
READY_Q = (0.0, 1.0, 1.0, -0.6, 0.0, 0.0)
NUT_GRIP_DX, NUT_GRIP_DZ = -0.007, -0.019
"""Arm A's grasp point relative to the nut's top-face centre: 7 mm back (pad tips 6 mm past the nut's centre, so the
wrist camera's bracket, 4.5 cm above the fingers, stays 4.5 cm behind the fitting laid on the nut) and 19 mm below its
top face, so the fingers, which widen behind their tips, stay below the fitting."""
GAP_PER_GRIP = 0.0939
"""Gap between the pads per unit of gripper command (each finger travels 0..46.95 mm)."""
SQUEEZE = 0.003
"""Arm A holds the nut and arm B the fitting with the pads commanded this far inside each face (about 15 N a finger
at the fingers' 5 kN/m), not fully closed: each finger is driven on its own, so a fully closed (force-saturated) grip
lets the part slide along the closing axis; a position-held grip keeps it centred."""
NUT_SQUEEZE = 0.003
"""The nut is held position-controlled (about 15 N a finger); with the hand following the thread down through each
stroke there is no bolt tension, and the thread's running friction does not turn the nut in the fingers."""
NUT_GRIP = (S.NUT_L - 2 * NUT_SQUEEZE) / GAP_PER_GRIP
PLATE_GRIP = (S.FIT_W - 2 * SQUEEZE) / GAP_PER_GRIP
HEAD_GRIP = (S.HEAD_AF - 2 * SQUEEZE) / GAP_PER_GRIP
SQUARE_OPEN = 0.75
"""Opening (70 mm) wide enough to come down round a fitting turned up to 30 degrees."""
"""While starting the bolt the head is held position-controlled too (the flats carry the turn), so it stays on the
nut's axis."""
NUT_LIFT = 0.055
"""Arm A lifts the nut this far off its post (the spring's foot clears the post's pocket)."""
PLATE_TCP_ABOVE_TOP = 0.003
"""Arm B's grasp point above the fitting's top face when it takes it by its left and right edges (pad tips 4.4 mm
below it). Closing along y keeps arm B's gripper body (10 cm along its closing axis) clear of arm A's wrist camera
behind the nut."""
BOLT_PRESS = 0.0010
"""How far below the entry the hand aims the bolt's head, pressing its end lightly into the nut's hole."""
START_TURNS_WANTED = 2.33
"""Turns of the wrist (seven strokes) while starting the bolt: about 2.5 turns of thread engaged."""
START_PRESS = 0.001
"""Each start stroke grips the head 1 mm below where the counted turns put it, so the hand presses lightly through the
stroke (the thread draws the bolt down 0.65 mm a stroke) instead of holding it up against the thread."""
CATCH_TURNS = 1 / 8
"""The thread's start is found within the first eighth of a turn (documented thread rule)."""


def q_mat(q):
    from yam_arena.urdf import quat_xyzw_to_matrix

    return quat_xyzw_to_matrix(np.asarray(q, dtype=float))


def down(yaw):
    return qmul(q_axis((0, 0, 1), yaw), DOWN)


class Demo:
    def __init__(self, control_hz: float = 20.0, speed: float = 0.10, skip_turn: bool = False, log=None,
                 tilt_deg: float = 0.0):
        self.log = log or (lambda *a: print(*a, flush=True))
        self.dt = 1.0 / control_hz
        self.speed = speed
        self.skip_turn = bool(skip_turn)
        self.tilt = math.radians(float(tilt_deg))
        self.debug_every = int(os.environ.get("SNC_DEMO_DEBUG", "0") or 0)
        self.k = 0
        self.obs = None
        self.events: dict[int, list] = {}
        self.A = self.B = None
        self.q_cmd = {}
        self.worst = 0.0
        self.trim = {"left": np.zeros(6), "right": np.zeros(6)}
        self.q_prev = {}
        self.mark_y = S.MARK_Y
        k = S.kit_start()
        self.kit0 = k
        self.head_bottom = k["bolt"][2]
        self.head_xy = np.array(k["bolt"][:2])
        self.e_start = S.ENTRY_DEPTH
        self.start_turns = 0.0
        self.start_strokes = 0
        self.turns = 0.0
        self.trim_frozen = False
        self.extra = 0
        self.cw_sign = None
        self.free_lags = []

    # -- helpers ---------------------------------------------------------------------------------------------------
    def hk(self, s):
        return max(1, round(s / self.dt))

    def q(self, side):
        return V.split_state(self.obs.state["joint_pos"])[side]

    def img(self, name):
        return np.asarray(self.obs.images[name])

    def grasp_point(self, side, q6=None):
        p, r = V.tcp_desk(self.q(side)[0] if q6 is None else q6, side)
        return p + r @ np.array([0.0, 0.0, TCP_EXTRA]), qmul(matrix_to_quat_xyzw(r), conj(Q_REL))

    def at(self, step, fn):
        self.events.setdefault(step, []).append(fn)

    def now(self, fn):
        self.at(max(len(self.A.steps), len(self.B.steps)), fn)

    def sync(self):
        n = max(len(self.A.steps), len(self.B.steps))
        self.A.pad_to(n)
        self.B.pad_to(n)

    def when_settled(self, fn, tries=40, tol=0.0015, axis=None, side="right"):
        """Call fn once an arm's measured grasp point is within tol of its target (or after tries x 0.1 s)."""
        self.sync()
        T = self.B if side == "right" else self.A

        def check():
            err = self.grasp_point(side)[0] - (T.pos + ROOT_OFFSET)
            if axis == "xy":
                lag = float(np.linalg.norm(err[:2]))
            else:
                lag = abs(err[axis]) if axis is not None else np.linalg.norm(err)
            if lag < tol or check.tries <= 0:
                if lag >= tol:
                    self.log(f"SETTLE timeout ({side}): lag {lag * 1000:.1f} mm; target {np.round(T.pos + ROOT_OFFSET, 4)}"
                             f", at {np.round(self.grasp_point(side)[0], 4)}")
                fn()
            else:
                check.tries -= 1
                T.hold(self.hk(0.1))
                self.sync()
                self.now(check)

        check.tries = tries
        self.now(check)

    def solve_side(self, side, pos_world, quat, seed):
        from yam_arena.ik import LOWER, UPPER

        tcp = np.asarray(pos_world, dtype=float) + q_mat(quat) @ np.array([0.0, 0.0, -TCP_EXTRA]) + \
            np.array([0.0, 0.0, V.DESK_Z])
        sols = []
        seed = np.asarray(seed, dtype=float)
        for sd in (seed, np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]), np.array([-0.5, 1.9, 1.3, -0.9, 0.0, 0.0]),
                   np.array([0.5, 1.9, 1.3, -0.9, 0.0, 0.0]), np.array([0.6, 1.6, 1.4, -1.0, 0.0, 1.0]),
                   np.array([0.6, 1.6, 1.4, -1.0, 0.0, -1.0])):
            q, pe, re = ik.solve(tcp, qmul(quat, Q_REL), sd, base_pos=station.BASES[side], iters=200)
            sols.append((q, pe, pe + 0.02 * re))
        good = [t for t in sols if t[2] < 0.0015]
        if good:
            def key(t):
                margin = min(np.min(t[0] - LOWER), np.min(UPPER - t[0]))
                return -min(margin, 0.25) * 4 + np.linalg.norm(t[0] - seed) * 0.2
            q, pe, _ = min(good, key=key)
            return q, pe
        q, pe, _ = min(sols, key=lambda t: t[2])
        return q, pe

    def jmove(self, T, side, pos_world, quat, seconds):
        q0 = getattr(T, "last_q", None)
        if q0 is None:
            q0, _ = self.solve_side(side, T.pos + ROOT_OFFSET, T.quat, self.q_cmd[side])
        q1, pe = self.solve_side(side, pos_world, quat, q0)
        if pe > 0.002:
            self.log(f"JMOVE {side}: target off by {pe * 1000:.1f} mm")
        self._jpath(T, q0, q1, seconds)
        T.pos, T.quat, T.last_q = np.asarray(pos_world, dtype=float) - ROOT_OFFSET, np.asarray(quat, dtype=float), q1
        return q1

    def _jpath(self, T, q0, q1, seconds):
        n = self.hk(seconds)
        for i in range(1, n + 1):
            a = i / n
            a = a * a * (3 - 2 * a)
            T.steps.append((None, q0 + a * (q1 - q0), T.grip))

    def jq(self, T, side, q1, seconds):
        q0 = getattr(T, "last_q", None)
        if q0 is None:
            q0, _ = self.solve_side(side, T.pos + ROOT_OFFSET, T.quat, self.q_cmd[side])
        q1 = np.asarray(q1, dtype=float)
        self._jpath(T, q0, q1, seconds)
        p, r = V.tcp_desk(q1, side)
        T.pos = p + r @ np.array([0.0, 0.0, TCP_EXTRA]) - ROOT_OFFSET
        T.quat, T.last_q = qmul(matrix_to_quat_xyzw(r), conj(Q_REL)), q1

    def grip(self, T, value, seconds):
        T.grip = value
        q = getattr(T, "last_q", None)
        for _ in range(self.hk(seconds)):
            T.steps.append((None, q, T.grip) if q is not None else (T.pos.copy(), T.quat.copy(), T.grip))

    # -- perception ------------------------------------------------------------------------------------------------
    def find_mark(self):
        """Red tape on the channel's near wall and lip -> its centre along the channel (y), or None."""
        im = self.img("top_cam")
        cam = V.Cam.top(im.shape[1], im.shape[0])
        bl = V.blobs(V.mask_range(im, *RED), min_px=2)
        if not bl:
            return None
        b = bl[0]
        vs, us = np.nonzero(b["mask"])
        pts = np.array([cam.on_plane((u + 0.5, v + 0.5), S.CH_TOP) for u, v in zip(us, vs)])
        lip = pts[pts[:, 0] > S.CH_X - S.CH_W / 2 - 0.001]  # the tape on the lip's top (on the plane used)
        use = lip if len(lip) >= 2 else pts
        return float(np.median(use[:, 1])), int(b["area"]), np.round(use.mean(axis=0), 4)

    def wrist_mark(self):
        """Right wrist camera (hand held over the channel): the red tape's pixels on the near lip, back-projected onto
        the lip's top plane, give the mark's centre along the channel at a finer scale than the top camera."""
        im = self.img("right_cam")
        cam = V.Cam.wrist("right", self.q("right")[0], im.shape[1], im.shape[0])
        bl = V.blobs(V.mask_range(im, *RED), min_px=4)
        if not bl:
            return None
        vs, us = np.nonzero(bl[0]["mask"])
        pts = np.array([cam.on_plane((u + 0.5, v + 0.5), S.CH_TOP) for u, v in zip(us, vs)])
        lip = pts[(pts[:, 0] > S.CH_X - S.CH_W / 2 + 0.001) & (pts[:, 0] < S.CH_X - S.SLOT / 2 - 0.001)]
        if len(lip) < 4:
            return None
        return float((lip[:, 1].min() + lip[:, 1].max()) / 2), len(lip)

    def cw_of_joint6(self):
        """+1 if raising joint 6 turns the hand clockwise seen from above (fingers down), else -1."""
        q = self.q("right")[0].copy()
        _, r0 = V.tcp_desk(q, "right")
        q[5] += 0.1
        _, r1 = V.tcp_desk(q, "right")
        y0 = math.atan2(r0[1, 0], r0[0, 0])
        y1 = math.atan2(r1[1, 0], r1[0, 0])
        d = (y1 - y0 + math.pi) % (2 * math.pi) - math.pi
        return 1.0 if d < 0 else -1.0

    # -- plan ------------------------------------------------------------------------------------------------------
    def tcp_z_for_head(self, head_bottom):
        return head_bottom + TIP_ABOVE_HEAD_BOTTOM + PAD_TIP

    def start(self):
        r = self.find_mark()
        if r is None:
            self.log("TOP CAM: red mark not found; using the layout")
        else:
            self.mark_y = r[0]
            self.log(f"TOP CAM: red mark centre y {r[0]:.4f} ({r[1]} px, mean {r[2]})")
        A, B = self.A, self.B
        # Arm A: the nut, by its two ends, fingers level and forward.
        nx, ny, ntop = self.kit0["nut"]
        A.grip = 0.6
        g = np.array([nx + NUT_GRIP_DX, ny, ntop + NUT_GRIP_DZ])
        self.jmove(A, "left", g + np.array([-0.045, 0.0, 0.05]), FORWARD, 3.0)
        A.move(g + np.array([-0.045, 0.0, 0.0]), speed=0.03)
        A.move(g, speed=0.02)
        A.hold(self.hk(0.4))
        # Arm B meanwhile: the fitting (the bolt dropped through it) off its block by its left and right edges
        # (fingers closing along y, the bolt's head between them).
        fx, fy, fz = self.kit0["fit"]
        B.grip = 0.6
        top = fz + S.FIT_T
        self.jmove(B, "right", np.array([fx, fy, top + PLATE_TCP_ABOVE_TOP + 0.05]), down(0.0), 3.0)
        B.move(np.array([fx, fy, top + PLATE_TCP_ABOVE_TOP]), speed=0.02)
        B.hold(self.hk(0.3))
        self.when_settled(self.take_nut, side="left", tol=0.0015)

    def take_nut(self):
        A, B = self.A, self.B
        self.log(f"NUT GRASP: arm A at {np.round(self.grasp_point('left')[0], 4)}")
        A.grip_to(NUT_GRIP, self.hk(0.8))
        B.grip_to(PLATE_GRIP, self.hk(0.8))
        g = A.pos + ROOT_OFFSET
        A.move(g + np.array([0.0, 0.0, NUT_LIFT]), speed=0.02)
        A.hold(self.hk(0.5))
        fx, fy, fz = self.kit0["fit"]
        B.move(np.array([fx, fy, fz + S.FIT_T + PLATE_TCP_ABOVE_TOP + 0.045]), speed=0.03)
        self.when_settled(self.place_plate, side="left", tol=0.0012)

    def nut_top(self):
        """The held nut's top-face centre from arm A's measured joints (forward kinematics)."""
        p, q = self.grasp_point("left")
        off = q_mat(q) @ q_mat(FORWARD).T @ np.array([NUT_GRIP_DX, 0.0, NUT_GRIP_DZ])
        return p - off

    def place_plate(self):
        """Over the held nut, the bolt's end 4 mm above the nut's top; then down: the bolt's end lands in the nut's
        hole and the bolt rises in the fitting's hole as the fitting is lowered to 3 mm above the nut."""
        B = self.B
        n = self.nut_top()
        self.nut_at = n
        self.log(f"NUT HELD: top centre {np.round(n, 4)} (lifted from {np.round(self.kit0['nut'], 4)})")
        z0 = n[2] + (S.BOLT_L - S.FIT_T) + 0.004 + PLATE_TCP_ABOVE_TOP + S.FIT_T
        self.jmove(B, "right", np.array([n[0], n[1], z0 + 0.02]), down(0.0), 2.5)
        B.move(np.array([n[0], n[1], z0]), speed=0.01)
        B.hold(self.hk(0.3))
        self.when_settled(self.lower_plate, tol=0.0010)

    def lower_plate(self):
        B = self.B
        n = self.nut_top()
        g = self.grasp_point("right")[0]
        self.log(f"KIT OVER NUT: fitting at {np.round(g[:2], 4)}, nut at {np.round(n[:2], 4)} "
                 f"({np.linalg.norm(g[:2] - n[:2]) * 1000:.1f} mm)")
        z1 = n[2] + 0.003 + PLATE_TCP_ABOVE_TOP + S.FIT_T
        B.move(np.array([n[0], n[1], z1]), speed=0.006)
        B.hold(self.hk(0.5))
        self.sync()
        self.now(self.release_plate)

    def release_plate(self):
        """Let go of the fitting (it drops 3 mm onto the nut round the bolt) and take the bolt's head."""
        B = self.B
        self.log(f"FITTING DOWN: arm B at {np.round(self.grasp_point('right')[0], 4)}")
        B.grip_to(0.6, self.hk(0.5))
        B.move(B.pos + ROOT_OFFSET + np.array([0.0, 0.0, 0.03]), speed=0.03)
        n = self.nut_at = self.nut_top()
        self.start_head = n[2] + S.BOLT_L - S.ENTRY_DEPTH
        B.grip = GRIP_OPEN
        self.jmove(B, "right", self.bolt_grasp(n, self.start_head) + np.array([0.0, 0.0, 0.012]), self.bolt_quat(), 2.0)
        B.move(self.bolt_grasp(n, self.start_head), speed=0.01)
        B.hold(self.hk(0.3))
        self.when_settled(self.take_bolt, tol=0.0010)

    def take_bolt(self):
        B = self.B
        self.cw_sign = self.cw_of_joint6()
        self.log(f"GRASP head at {np.round(self.grasp_point('right')[0], 4)}; joint 6 + is "
                 f"{'clockwise' if self.cw_sign > 0 else 'anticlockwise'} from above")
        B.grip_to(HEAD_GRIP, self.hk(0.8))
        self.sync()
        self.now(self.insert_bolt)

    def bolt_quat(self, yaw=YAW0):
        return qmul(q_axis((1, 0, 0), self.tilt), down(yaw)) if self.tilt else down(yaw)

    def bolt_grasp(self, n, head_bottom):
        """Grasp point holding the bolt with its head's underside at head_bottom (its end on the nut's axis; tilted
        about x by self.tilt round its end in the hazard check)."""
        lg = head_bottom - n[2] + S.BOLT_L * 0 + TIP_ABOVE_HEAD_BOTTOM + PAD_TIP
        tip_z = head_bottom - S.BOLT_L
        if not self.tilt:
            return np.array([n[0], n[1], self.tcp_z_for_head(head_bottom)])
        lg = S.BOLT_L + TIP_ABOVE_HEAD_BOTTOM + PAD_TIP
        return np.array([n[0], n[1] - lg * math.sin(self.tilt), tip_z + lg * math.cos(self.tilt)])

    def insert_bolt(self):
        B = self.B
        n = self.nut_at
        g = self.grasp_point("right")[0]
        self.log(f"ON NUT: bolt axis at {np.round(g[:2], 4)}, nut at {np.round(n[:2], 4)} "
                 f"({np.linalg.norm(g[:2] - n[:2]) * 1000:.1f} mm)")
        self.trim_frozen = True  # press lightly: the arms' own compliance, no integral push into the nut
        B.move(self.bolt_grasp(n, self.start_head - BOLT_PRESS), speed=0.006)
        B.hold(self.hk(0.5))
        self.sync()
        self.now(self.start_stroke)

    def start_stroke(self):
        """One stroke of starting the bolt: a third of a turn clockwise with the head gripped."""
        B = self.B
        self.stroke_q0 = self.q("right")[0].copy()
        q0 = np.asarray(B.last_q if getattr(B, "last_q", None) is not None else self.q_cmd["right"], dtype=float)
        q1 = q0.copy()
        q1[5] += self.cw_sign * STROKE
        # Follow the thread down through the stroke (a third of a pitch), so the hand never holds the bolt up.
        p0, _ = V.tcp_desk(q0, "right")
        p1, r1 = V.tcp_desk(q1, "right")
        quat1 = qmul(matrix_to_quat_xyzw(r1), conj(Q_REL))
        target = p1 + r1 @ np.array([0.0, 0.0, TCP_EXTRA]) - np.array([0.0, 0.0, S.PITCH * STROKE / (2 * math.pi)])
        q1b, pe = self.solve_side("right", target, quat1, q1)
        if pe < 0.0005 and abs(q1b[5] - q1[5]) < 0.05:
            q1 = q1b
        self.jq(B, "right", q1, 1.6)
        B.hold(self.hk(0.3))
        self.sync()
        self.now(self.start_stroke_end)

    def start_head_bottom(self):
        e = S.ENTRY_DEPTH + S.PITCH * max(0.0, self.start_turns - CATCH_TURNS)
        return self.start_head - (e - S.ENTRY_DEPTH)

    def start_stroke_end(self):
        B = self.B
        turned = self.cw_sign * (self.q("right")[0][5] - self.stroke_q0[5]) / (2 * math.pi)
        self.start_turns += turned
        self.start_strokes += 1
        self.log(f"START STROKE {self.start_strokes}: turned {turned:.3f} rev, {self.start_turns:.2f} rev in all")
        if self.tilt:
            self.log("HAZARD CHECK (tilted start): letting go")
            B.grip_to(GRIP_OPEN, self.hk(0.3))
            B.move(B.pos + ROOT_OFFSET + np.array([0.0, 0.0, 0.06]), speed=0.03)
            self.sync()
            self.now(self.finish)
            return
        B.grip_to(GRIP_OPEN, self.hk(0.3))
        B.move(B.pos + ROOT_OFFSET + np.array([0.0, 0.0, 0.004]), speed=0.03)
        n = self.nut_at
        hb = self.start_head_bottom()
        z = self.tcp_z_for_head(hb)
        self.jmove(B, "right", np.array([n[0], n[1], z + 0.004]), down(YAW0), 0.8)
        last = self.start_turns >= START_TURNS_WANTED - 0.05
        B.move(np.array([n[0], n[1], z - (0.0 if last else START_PRESS)]), speed=0.01)
        B.hold(self.hk(0.2))

        def close():
            B.grip_to(CLOSED, self.hk(0.5))
            self.sync()
            self.now(self.carry if last else self.start_stroke)

        self.when_settled(close, tol=0.0008, axis="xy", tries=15)

    def carry(self):
        A, B = self.A, self.B
        self.e_start = S.ENTRY_DEPTH + S.PITCH * (self.start_turns - CATCH_TURNS)
        self.log(f"STARTED: {self.start_turns:.2f} rev turned, about {self.e_start / S.PITCH:.2f} turns engaged")
        self.trim_frozen = False
        # Arm A lets go and backs straight out, up and to its own side, and stays parked there (a joint-space move
        # back to the ready pose swept the arm across the kit).
        A.grip_to(0.6, self.hk(0.6))
        a = A.pos + ROOT_OFFSET
        A.move(np.array([0.20, a[1], a[2]]), speed=0.03)
        A.move(np.array([0.20, a[1], 0.17]), speed=0.03)
        A.move(np.array([0.20, 0.12, 0.17]), speed=0.05)
        self.sync()
        p = B.pos + ROOT_OFFSET
        B.move(p + np.array([0.0, 0.0, 0.02]), speed=0.02)
        self.sync()
        x, y = S.CH_X, S.INSERT_Y
        self.jmove(B, "right", np.array([x, y, p[2] + 0.02]), down(YAW0), 3.0)
        B.hold(self.hk(0.5))
        self.when_settled(self.lower, tol=0.002)

    def lower(self):
        B = self.B
        r = self.wrist_mark()
        if r is None:
            self.log("WRIST CAM: red mark not seen; keeping the top camera's estimate")
        else:
            self.log(f"WRIST CAM: red mark centre y {r[0]:.4f} ({r[1]} px on the lip; top camera said {self.mark_y:.4f})")
            self.mark_y = r[0]
        x, y = S.CH_X, S.INSERT_Y
        # Head bottom such that the nut's top is PUSH_BELOW below the lip returns (engaged length as started).
        e0 = self.e_start
        self.push_head = S.CH_TOP - S.LIP_RETURN - PUSH_BELOW + (S.BOLT_L - e0)
        B.move(np.array([x, y, self.tcp_z_for_head(self.push_head) + 0.030]), speed=0.03)
        B.move(np.array([x, y, self.tcp_z_for_head(self.push_head)]), speed=0.008)
        B.hold(self.hk(0.5))
        self.when_settled(self.quarter_turn, tol=0.0012, axis=2, tries=60)

    def quarter_turn(self):
        B = self.B
        g = self.grasp_point("right")[0]
        self.log(f"PUSHED: grasp point {np.round(g, 4)} (head bottom about {g[2] - PAD_TIP - TIP_ABOVE_HEAD_BOTTOM:.4f},"
                 f" wanted {self.push_head:.4f})")
        q0 = np.asarray(B.last_q if getattr(B, "last_q", None) is not None else self.q_cmd["right"], dtype=float)
        if not self.skip_turn:
            q1 = q0.copy()
            q1[5] += self.cw_sign * STROKE
            self.jq(B, "right", q1, 2.0)
            B.hold(self.hk(0.5))
            self.turns += (STROKE - math.radians(90.0)) / (2 * math.pi)
        else:
            self.log("CONTROL: no quarter turn")
        # Ease the push off: the spring lifts the nut (and the bolt) until its grooves close on the lips.
        e = self.e_start + self.turns * S.PITCH
        self.seat_head = S.SEATED_TOP + (S.BOLT_L - e)
        p = B.pos + ROOT_OFFSET
        self.jmove(B, "right", np.array([p[0], p[1], self.tcp_z_for_head(self.seat_head) + 0.0015]), B.quat, 1.5)
        B.hold(self.hk(0.6))
        self.when_settled(self.slide, tol=0.002)

    def slide(self):
        B = self.B
        g = self.grasp_point("right")[0]
        self.log(f"SEATED?: grasp point {np.round(g, 4)} (seated head bottom {self.seat_head:.4f})")
        z = B.pos[2] + ROOT_OFFSET[2]
        B.move(np.array([S.CH_X, self.mark_y, z]), speed=0.015)
        B.hold(self.hk(0.8))
        self.when_settled(self.place_check, tol=0.0008, axis=1, tries=50)

    def place_check(self):
        B = self.B
        g = self.grasp_point("right")[0]
        err = g[1] - self.mark_y
        self.log(f"AT MARK: bolt axis y {g[1]:.4f} (mark {self.mark_y:.4f}, {err * 1000:+.1f} mm), x {g[0]:.4f}")
        self.head_xy = np.array([S.CH_X, self.mark_y])
        self.head_bottom = self.seat_head
        # Let go and lift; the strokes start with the hand turned back to YAW0.
        B.grip_to(GRIP_OPEN, self.hk(0.5))
        B.move(B.pos + ROOT_OFFSET + np.array([0.0, 0.0, 0.006]), speed=0.02)
        self.square_fitting()

    def square_fitting(self):
        """Square the fitting on the channel before running the bolt down: fingers down beside the channel's walls,
        closing across the channel on the fitting's two edges (the fitting is as wide as the channel, so the closed
        jaws line its edges up with the walls), then open and go on. The fitting may have turned on the nut while the
        bolt was started; nothing else squares it."""
        B = self.B
        hx, hy = self.head_xy
        z = S.CH_TOP + S.FIT_T + PLATE_TCP_ABOVE_TOP
        B.grip = SQUARE_OPEN
        self.jmove(B, "right", np.array([hx, hy, z + 0.03]), down(math.pi / 2), 1.5)
        B.move(np.array([hx, hy, z]), speed=0.015)
        B.hold(self.hk(0.2))
        B.grip_to(PLATE_GRIP, self.hk(1.0))
        B.hold(self.hk(0.3))
        B.grip_to(SQUARE_OPEN, self.hk(0.5))
        B.move(np.array([hx, hy, z + 0.03]), speed=0.03)
        self.sync()
        self.now(self.next_stroke)

    def next_stroke(self):
        B = self.B
        hx, hy = self.head_xy
        e = self.e_start + self.turns * S.PITCH
        self.head_bottom = self.seat_head - (e - (self.e_start + (STROKE - math.radians(90)) /
                                                 (2 * math.pi) * S.PITCH * (0 if self.skip_turn else 1)))
        head_bottom = max(self.head_bottom, S.CH_TOP + S.FIT_T)
        z = self.tcp_z_for_head(head_bottom)
        self.jmove(B, "right", np.array([hx, hy, z + 0.004]), down(YAW0), 0.8)
        B.move(np.array([hx, hy, z]), speed=0.01)
        B.hold(self.hk(0.2))

        def close():
            B.grip_to(CLOSED, self.hk(0.5))
            self.sync()
            self.now(self.turn_stroke)

        self.when_settled(close, tol=0.0006, axis="xy", tries=15)

    def turn_stroke(self):
        B = self.B
        self.stroke_q0 = self.q("right")[0].copy()
        q0 = np.asarray(B.last_q if getattr(B, "last_q", None) is not None else self.q_cmd["right"], dtype=float)
        q1 = q0.copy()
        q1[5] += self.cw_sign * STROKE
        self.stroke_cmd_end = q1[5]
        self.jq(B, "right", q1, 1.6)
        B.hold(self.hk(0.3))
        self.sync()
        self.now(self.end_stroke)

    def end_stroke(self):
        B = self.B
        q_meas = self.q("right")[0]
        turned = self.cw_sign * (q_meas[5] - self.stroke_q0[5]) / (2 * math.pi)
        lag = self.cw_sign * (self.stroke_cmd_end - q_meas[5])
        self.turns += turned
        need = (self.seat_head - (S.CH_TOP + S.FIT_T)) / S.PITCH
        done = self.turns - (0.0 if self.skip_turn else (STROKE - math.radians(90)) / (2 * math.pi))
        self.log(f"STROKE: turned {turned:.3f} rev, counted {done:.2f} of {need:.2f} rev to the head's seat, "
                 f"wrist lag {math.degrees(lag):.1f} deg (free-running {math.degrees(float(np.median(self.free_lags))) if self.free_lags else 0:.1f})")
        B.grip_to(GRIP_OPEN, self.hk(0.3))
        p = B.pos + ROOT_OFFSET
        B.move(p + np.array([0.0, 0.0, 0.004]), speed=0.03)
        if done < need - 0.6:
            self.free_lags.append(lag)
        base = float(np.median(self.free_lags)) if self.free_lags else 0.0
        snug = done >= need - 0.05 and lag - base > WRIST_SNUG_LAG
        if done >= need - 0.05:
            self.extra += 1
        if snug or self.extra > MAX_EXTRA or done > need + 1.5:
            self.log(f"HAND-SNUG: {'wrist lag says snug' if snug else 'stopped (no lag)'} after {done:.2f} rev")
            self.sync()
            self.now(self.finish)
            return
        self.sync()
        self.now(self.next_stroke)

    def finish(self):
        B = self.B
        p = B.pos + ROOT_OFFSET
        B.move(p + np.array([0.0, 0.0, 0.06]), speed=0.04)
        self.jq(B, "right", TUCK_Q, 2.5)
        if self.tilt:
            self.A.grip_to(0.6, self.hk(0.6))
            self.A.move(self.A.pos + ROOT_OFFSET + np.array([-0.07, 0.0, 0.0]), speed=0.04)
        B.hold(self.hk(10.0))
        self.sync()

    # -- action ------------------------------------------------------------------------------------------------------
    def _ik(self, side, pos_root, quat, grip):
        pos = pos_root + ROOT_OFFSET + np.array([0.0, 0.0, V.DESK_Z])
        tcp = pos + q_mat(quat) @ np.array([0.0, 0.0, -TCP_EXTRA])
        rot = qmul(quat, Q_REL)
        q, pe, _ = ik.solve(tcp, rot, self.q_cmd[side], base_pos=station.BASES[side], iters=60)
        if pe > 0.001:
            for seed in (self.q(side)[0], np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])):
                q2, pe2, _ = ik.solve(tcp, rot, seed, base_pos=station.BASES[side], iters=200)
                if pe2 < pe and np.max(np.abs(q2 - self.q_cmd[side])) < 0.6:
                    q, pe = q2, pe2
        if pe > 0.02:
            q = self.q_cmd[side]
        self.q_cmd[side] = q
        if pe > 0.005 and pe > self.worst:
            self.log(f"IK {side} k={self.k} error {pe * 1000:.1f} mm at {np.round(pos_root + ROOT_OFFSET, 4)}")
            self.worst = max(self.worst, pe)
        return np.r_[q, grip]

    def next_action(self, obs) -> np.ndarray:
        self.obs = obs
        if self.A is None:
            for side in ("left", "right"):
                self.q_cmd[side] = self.q(side)[0].copy()
            ga, qa = self.grasp_point("left")
            gb, qb = self.grasp_point("right")
            self.A = ArmTimeline(ga - ROOT_OFFSET, qa, self.dt, self.speed)
            self.B = ArmTimeline(gb - ROOT_OFFSET, qb, self.dt, self.speed)
            self.A.grip = self.B.grip = 1.0
            self.A.hold(self.hk(0.5))
            self.B.hold(self.hk(0.5))
            self.at(self.hk(0.5), self.start)
        while self.k in self.events:
            for fn in self.events.pop(self.k):
                fn()
        for T in (self.A, self.B):
            if self.k >= len(T.steps):
                T.hold(self.k + 1 - len(T.steps))
        acts = []
        for side, T in (("left", self.A), ("right", self.B)):
            st = T.steps[self.k]
            if st[0] is None:
                if st[1] is not None:
                    self.q_cmd[side] = np.asarray(st[1], dtype=float)
                acts.append(np.r_[self.q_cmd[side], st[2]])
            else:
                acts.append(self._ik(side, *st))
        for i, side in enumerate(("left", "right")):
            q_des = acts[i][:6]
            prev = self.q_prev.get(side)
            if prev is not None and not self.trim_frozen:
                still = np.abs(q_des - prev) < 0.0005
                still[5] = False if side == "right" else still[5]  # never trim the turning wrist roll
                err = q_des - self.q(side)[0]
                self.trim[side] = np.where(still, np.clip(self.trim[side] + TRIM_GAIN * err, -TRIM_MAX, TRIM_MAX),
                                           self.trim[side])
            self.q_prev[side] = q_des.copy()
            acts[i] = np.r_[q_des + self.trim[side], acts[i][6]]
        act = np.concatenate(acts)
        if self.debug_every and self.k % self.debug_every == 0:
            gb = self.grasp_point("right")[0]
            self.log(f"DBG k={self.k} B at {np.round(gb, 4)} target {np.round(self.B.steps[self.k][0] + ROOT_OFFSET, 4) if self.B.steps[self.k][0] is not None else 'joint'} "
                     f"q6 {self.q('right')[0][5]:.3f} grip {self.q('right')[1]:.2f} trim {np.round(self.trim['right'], 3)}")
        self.k += 1
        return act
