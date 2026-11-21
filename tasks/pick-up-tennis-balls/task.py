"""Pick up a tennis ball from the court with a rolling wire-cage ball picker and lift it off the ground.

The robot's arms stand on a short bench at the edge of a hard-court mat. On the mat stand a rolling ball picker (a
Garden Weasel Large Nut Gatherer cage: 30 spring-steel wires between two hubs, turning in a U-shaped bail, on a 60 in
aluminium pole) and, 18 cm in front of its cage, a tennis ball. The pole leans back towards the robot at 55 degrees.

The ball goes in when the cage is pressed onto it and rolled over it, so that two wires spread apart round the ball and
close behind it.

Score (end of the trial): 1.0 if the ball is inside the cage and the cage's lowest point is at least 10 cm above the
court, held for 1 s; 0.5 if the ball is inside the cage but the cage is lower; otherwise 0. A trial in which the robot
touched the ball scores 0.
"""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from scenekit import UsdBody, on_reset, on_step  # noqa: E402
from yam_arena import station as _station  # noqa: E402

SCENE = "station"
TOP = 0.7475  # the bench (the station's table, cut back) that the arms stand on
# yam_arena builds the station's table from these two values when it builds the scene, after loading this file
TABLE_X = (-0.19, 0.12)
_station.TABLE_SIZE = (TABLE_X[1] - TABLE_X[0], _station.TABLE_SIZE[1], _station.TABLE_SIZE[2])
_station.TABLE_CENTER = ((TABLE_X[0] + TABLE_X[1]) / 2, _station.TABLE_CENTER[1], _station.TABLE_CENTER[2])
GROUND = _station.FLOOR_Z

IN = 0.0254
R = 4.0 * IN  # cage radius at its middle (assets/picker.usda)
HALF = 4.05 * IN  # half the distance between the hub faces
R_END = 0.75 * R  # the cage's radius at the hub faces
N_WIRES, D_WIRE = 30, 0.0010
BALL_R, BALL_MASS = 0.0335, 0.0577  # ITF type 2: 6.54-6.86 cm, 56.0-59.4 g
COURT_T, COURT_X = 0.004, 0.95  # the mat, 1.6 x 1.6 m
FLOOR = GROUND + COURT_T  # the court's surface
G = 9.81
E = os.environ.get

# -- the picker's start and the ball --------------------------------------------------------------------------------
HANDLE_TILT = math.radians(55.0)  # the pole's rise above the court, leaning back towards the robot
S_END = 60.0 * IN - R  # axle to the pole's end (60 in overall)
END_X = 0.20  # the pole's end at the start, just in front of the bench
X0 = END_X + S_END * math.cos(HANDLE_TILT)  # the cage's axle at the start
START_Z = FLOOR + R + 0.0008
GAP = 0.18  # axle to the ball's centre at the start, along x
_PHI = math.atan2(-math.cos(HANDLE_TILT), math.sin(HANDLE_TILT))
TILT_XYZW = (0.0, math.sin(_PHI / 2), 0.0, math.cos(_PHI / 2))  # turns the picker's +z (the pole) to its lean

# -- physics of the cage and the ball (the wires' springs: a beam model of a 1 mm spring-steel wire) ----------------
K_T, WIRE_DAMP = 56.94, 0.1  # a wire's sideways spring at its middle (N/m), damping (N s/m)
K_R, RAD_DAMP = 321.0, 0.3  # its inward spring (N/m), damping
AXLE_DAMP = 0.001  # N m s/rad
MU_C = 1.1  # ball cloth on the hard court (Cross, Am. J. Phys. 73, 812 (2005): 1.03-1.25 on court-like roughness)
MU_ROLL = 0.015  # tennis ball rolling friction (Singh et al., arXiv:0809.4823: 0.01-0.025), a torque against its spin

PICKER = UsdBody("picker", usd="assets/picker.usda", pos=(X0, 0.0, START_Z), rot_xyzw=TILT_XYZW, size=(0.21, 0.30, 0.21),
                 articulation=True, material=False, solver_iters=32, joint_damping=0.0)
BALL = UsdBody("ball", usd="assets/ball.usda", pos=(X0 + GAP, 0.0, FLOOR + BALL_R + 0.0005),
               size=(0.067, 0.067, 0.067), mass=BALL_MASS, material=False)
COURT = UsdBody("court", usd="assets/court.usda", pos=(COURT_X, 0.0, GROUND + COURT_T / 2), size=(1.6, 1.6, COURT_T),
                friction=(MU_C, MU_C))  # fixed (no mass)
OBJECTS = [COURT, PICKER, BALL]
SUCCESS_HOLD_S = 1.0
LIFTED = 0.10  # m: the cage's lowest point above the court for full credit
TOUCH = 0.03  # m: a hand (finger pads' centre) or arm link this close to the ball's surface touches it

# -- the scripted demo -----------------------------------------------------------------------------------------------
# Each hand holds the pole at a distance s from the axle, its fingers closing across the pole and its approach at
# right angles to it, turned psi round the pole (holds the arms can keep through each move).
GRIP_R, TUBE_R = 0.0135, 0.9 * IN / 2  # rubber grip, pole
GRIP_FROM = S_END - 0.20  # the grip covers the pole's last 20 cm
HOLD = {"right": (1.34, 210.0, -1), "left": (1.16, 120.0, -1)}  # s (m from the axle), psi (deg), finger sign
REGRASP = {"left": (0.70, 120.0, -1), "right": (1.05, 210.0, -1)}  # lower down, one hand after the other
PRESS = 0.010  # the hands aim this far down the pole past where it rests: they press the cage onto the court
PUSH = 0.28  # walk the picker forward this far: the cage rolls over the ball
PUSH_S = 4.0  # ... in this long (7 cm/s)
UPRIGHT = math.radians(65.0)  # then pull it back, standing the pole up to this ...
ROLL_BACK = 0.40  # ... while the cage, the ball in it, rolls back this far
BACK_OFF = 0.06  # a regrasping hand backs off this far along its approach
LIFT_CMD = 0.25  # the hands' target rise when lifting
SQUEEZE = 0.010  # each finger's target this far inside the pole's surface
OPEN_BY = 0.001  # the fingers' clearance round the pole at the start

_S: dict = {}


def _np(x):
    import numpy as np

    x = getattr(x, "torch", x)
    return np.asarray(x.detach().cpu() if hasattr(x, "detach") else x, dtype=float)


# -- scoring ---------------------------------------------------------------------------------------------------------

def inside_cage(rel, axle_dir=(0.0, 1.0, 0.0)) -> bool:
    """Whether a ball centred at `rel` from the cage's axle centre is inside the cage: between the hub faces, and well
    within the wires' surface there (by half its radius)."""
    import numpy as np

    rel, a = np.asarray(rel, float), np.asarray(axle_dir, float) / np.linalg.norm(axle_dir)
    along = float(rel @ a)
    radial = float(np.linalg.norm(rel - along * a))
    y = abs(along)
    surface = R - (R - R_END) * (min(y, HALF) / HALF) ** 2
    return bool(y < HALF and radial < surface - 0.5 * BALL_R)


def grade(inside: bool, low: float, touched: bool) -> float:
    """The score from the ball inside the cage or not, the cage's lowest point above the court (m), and whether the
    robot touched the ball during the trial."""
    if touched or not inside:
        return 0.0
    return 1.0 if low >= LIFTED else 0.5


def _state(environment):
    """(ball centre from the axle, the axle's direction, the cage's lowest point above the court)."""
    import numpy as np

    sc = environment.unwrapped.scene
    o = _S["origin"]
    picker = sc["picker"]
    axle = _np(picker.data.root_pos_w)[0] - o
    x, y, z, w = _np(picker.data.root_quat_w)[0]  # x, y, z, w
    axle_dir = np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)])  # the picker's y
    ball = _np(sc["ball"].data.root_pos_w)[0] - o
    return ball - axle, axle_dir, float(axle[2] - R - FLOOR)


def _score_now() -> float:
    from scenekit import env

    environment = env()
    if environment is None or "origin" not in _S:
        return 0.0
    rel, axle_dir, low = _state(environment)
    return grade(inside_cage(rel, axle_dir), low, _S.get("touched", False))


def score(objects) -> float:
    return _score_now()


def success(objects) -> bool:
    return _score_now() >= 1.0


# -- the simulation: reset, the ball's rolling friction, the touch watch ---------------------------------------------

@on_reset
def _reset(environment) -> None:
    import torch

    sc = environment.unwrapped.scene
    picker, ball = sc["picker"], sc["ball"]
    o = _np(sc.env_origins[0])
    dev = picker.device
    zero6 = torch.zeros((1, 6), device=dev)
    # yam_arena resets only the arms: put the picker and the ball at their start
    picker.write_root_pose_to_sim_index(root_pose=torch.tensor(
        [[o[0] + PICKER.pos[0], o[1] + PICKER.pos[1], o[2] + PICKER.pos[2], *TILT_XYZW]], dtype=torch.float32, device=dev))
    picker.write_root_velocity_to_sim_index(root_velocity=zero6)
    nj = picker.num_joints
    picker.write_joint_state_to_sim_index(position=torch.zeros((1, nj), device=dev), velocity=torch.zeros((1, nj), device=dev))
    ball.write_root_pose_to_sim_index(root_pose=torch.tensor(
        [[o[0] + BALL.pos[0], o[1] + BALL.pos[1], o[2] + BALL.pos[2], 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32, device=dev))
    ball.write_root_velocity_to_sim_index(root_velocity=zero6)
    # the wires' springs (sideways and inwards) and the axle's damping
    for pattern, k, c in (("wire_.*", K_T, WIRE_DAMP), ("rad_.*", K_R, RAD_DAMP)):
        ids, _ = picker.find_joints(pattern)
        picker.write_joint_stiffness_to_sim_index(stiffness=torch.full((1, len(ids)), k, device=dev), joint_ids=ids)
        picker.write_joint_damping_to_sim_index(damping=torch.full((1, len(ids)), c, device=dev), joint_ids=ids)
    axle, _ = picker.find_joints("axle")
    picker.write_joint_damping_to_sim_index(damping=torch.full((1, 1), AXLE_DAMP, device=dev), joint_ids=axle)
    # the arms' starting pose
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    for side in ("left", "right"):
        robot = sc[f"{side}_robot"]
        arm = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
        fing = robot.find_joints(list(FINGER_JOINTS), preserve_order=True)[0]
        q = robot.data.default_joint_pos.torch[:1].clone()
        q[0, arm] = torch.tensor(start_q(side), dtype=q.dtype, device=q.device)
        q[0, fing] = -(_surface(side) + OPEN_BY)
        robot.write_joint_position_to_sim_index(position=q)
        robot.write_joint_velocity_to_sim_index(velocity=torch.zeros_like(q))
        robot.actuators.target_command.set_position_index(value=q.clone())
        robot.actuators.target_command.set_velocity_index(value=torch.zeros_like(q))
    _S.update(t=0.0, step=0, touched=False, ball=ball, origin=o)
    if not _S.get("wrapped"):  # the ball's rolling friction, before every physics substep
        write = picker.write_data_to_sim

        def write_with_rolling_friction():
            _rolling_friction()
            write()

        picker.write_data_to_sim = write_with_rolling_friction
        _S["wrapped"] = True


def _rolling_friction() -> None:
    """A torque against the ball's spin about the horizontal axes: mu_r m g r (PhysX has no rolling friction)."""
    import numpy as np
    import torch

    _S["t"] += 1.0 / 240.0
    ball = _S["ball"]
    spin = _np(ball.data.root_ang_vel_w)[0] * np.array([1.0, 1.0, 0.0])
    torque = -MU_ROLL * BALL_MASS * G * BALL_R * spin / max(float(np.linalg.norm(spin)), 0.5)
    ball.permanent_wrench_composer.set_forces_and_torques_index(
        forces=torch.zeros((1, 1, 3), device=ball.device),
        torques=torch.tensor([[torque.tolist()]], dtype=torch.float32, device=ball.device), body_ids=[0], is_global=True)


@on_step
def _watch(environment) -> None:
    import numpy as np
    import torch
    from scenekit import arm_joints, robot_links
    from yam_arena import ik
    from yam_arena.constants import ARM_JOINTS

    _S["step"] += 1
    ball = _S["ball"]
    if _S["t"] < 0.5:  # the ball lies still on the court at the start
        ball.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=ball.device))
    ball_p = _np(ball.data.root_pos_w)[0] - _S["origin"]
    points = [p for links in robot_links(environment).values() for p in links.values()]
    points += [ik.tcp_pose(arm_joints(environment, side, ARM_JOINTS), _station.BASES[side])[0] for side in ("left", "right")]
    if any(np.linalg.norm(p - ball_p) < BALL_R + TOUCH for p in points):
        _S["touched"] = True
    if E("BALLS_LOG") and _S["step"] % 2 == 0:
        rel, axle_dir, low = _state(environment)
        print(f"LOG t={_S['t']:.2f} ball_rel=({rel[0]:+.3f},{rel[1]:+.3f},{rel[2]:+.3f}) inside={inside_cage(rel, axle_dir)} "
              f"cage_low={low:.3f} touched={_S['touched']} score={_score_now()}", flush=True)


# -- the scripted demo -----------------------------------------------------------------------------------------------

def _handle_dir(tilt=None):
    tilt = HANDLE_TILT if tilt is None else tilt
    return (-math.cos(tilt), 0.0, math.sin(tilt))


def grip_pose(side, dx=0.0, dz=0.0, press=0.0, tilt=None, hold=None, back=0.0):
    """(TCP position, rotation) holding the pole: the picker moved dx forward and dz up, the hand `press` further down
    the pole, the pole at `tilt` (default: its start), `hold` = (s, psi, sign) instead of HOLD[side], `back`: backed
    off along the approach."""
    import numpy as np
    from yam_arena.demo import DOWN_GRIP
    from yam_arena.urdf import quat_xyzw_to_matrix

    s_hold, psi, sgn = hold or HOLD[side]
    u = np.array(_handle_dir(tilt))
    p = np.array([X0 + dx, 0.0, START_Z + dz]) + (s_hold - press) * u
    v1 = np.cross(u, [0.0, 1.0, 0.0])
    v1 /= np.linalg.norm(v1)
    v2 = np.cross(u, v1)
    a = math.cos(math.radians(psi)) * v1 + math.sin(math.radians(psi)) * v2
    c = sgn * np.cross(a, u)
    down = quat_xyzw_to_matrix(np.asarray(DOWN_GRIP))
    a_loc, c_loc = down.T @ np.array([0.0, 0.0, -1.0]), down.T @ np.array([0.0, 1.0, 0.0])
    m = np.column_stack([a, c, np.cross(a, c)])
    m0 = np.column_stack([a_loc, c_loc, np.cross(a_loc, c_loc)])
    return p - back * a, m @ m0.T


def _surface(side, s_hold=None):
    s_hold = HOLD[side][0] if s_hold is None else s_hold
    return GRIP_R if s_hold >= GRIP_FROM else TUBE_R


_START: dict = {}


def start_q(side):
    """The arm's joints at the start (IK from a few seeds; deterministic)."""
    if side not in _START:
        import numpy as np
        from yam_arena import ik

        pos, rot = grip_pose(side)
        best = None
        for q0 in ([0.0, 1.0, 1.0, -0.6, 0.0, 0.0], [0.0, 1.6, 1.2, 0.2, 0.0, 0.0], [0.0, 2.2, 1.6, 0.6, 0.0, 0.0],
                   [0.4, 1.4, 0.8, -0.2, 0.0, 1.0], [-0.4, 1.4, 0.8, -0.2, 0.0, -1.0], [0.0, 2.6, 2.4, 1.0, 0.0, 0.0]):
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(q0), base_pos=_station.BASES[side], iters=300)
            if best is None or pe + 0.05 * re < best[1] + 0.05 * best[2]:
                best = (q, pe, re)
        _START[side] = best[0]
    return _START[side]


def _install_hands() -> None:
    """In this process only, without changing yam_arena: a Waypoint's yaw may be a 3x3 rotation (a tuple of 9), used
    as the grip's rotation as it is; and an arm's plan starts from the arm's starting pose (start_q)."""
    import numpy as np
    from yam_arena import demo as _demo

    if getattr(_demo.grip_rotation, "_balls", False):
        return
    plain = _demo.grip_rotation

    def grip_rotation(yaw=0.0):
        if isinstance(yaw, tuple) and len(yaw) == 9:
            return np.array(yaw, dtype=float).reshape(3, 3)
        return plain(yaw)

    grip_rotation._balls = True
    _demo.grip_rotation = grip_rotation
    init = _demo.ArmPlan.__init__

    def init_holding(self, waypoints, base_pos, q_start=_demo.READY, finger_start=_demo.FINGER_OPEN):
        for side, b in _station.BASES.items():
            if np.allclose(np.asarray(base_pos, float), b):
                q_start, finger_start = start_q(side), -(_surface(side) + OPEN_BY)
        init(self, waypoints, base_pos, q_start, finger_start)

    _demo.ArmPlan.__init__ = init_holding


_install_hands()


def demo():
    """The hands press the cage onto the court and walk the picker forward so the
    cage rolls over the ball and it pops in; pull the cage back, standing the pole up; take the pole again lower
    down, one hand after the other; and lift the picker with the ball in it."""
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    plan = {}
    for side in ("left", "right"):
        f = -(_surface(side) - SQUEEZE)
        steps = []

        def go(duration, label, finger=None, **kw):
            pos, rot = grip_pose(side, **kw)
            steps.append(W(duration, tuple(float(v) for v in pos), f if finger is None else finger, label,
                           tuple(float(v) for v in rot.reshape(-1))))

        go(0.6, "close on the grip")
        go(0.6, "settle")
        go(1.2, "press the cage onto the court", press=PRESS)
        go(PUSH_S, "roll the cage forward over the ball", dx=PUSH, press=PRESS)
        go(0.5, "stop", dx=PUSH, press=PRESS)
        for k in range(1, 5):  # pull the cage back, the ball in it, standing the pole up
            frac = k / 4
            go(0.9, "pull the cage back", dx=PUSH - frac * ROLL_BACK, tilt=HANDLE_TILT + frac * (UPRIGHT - HANDLE_TILT))
        back = dict(dx=PUSH - ROLL_BACK, tilt=UPRIGHT)
        go(0.5, "steady", **back)
        held = {}
        for mover in ("left", "right"):  # each hand in turn lets go, moves down the pole and takes it again
            if mover == side:
                low = REGRASP[side]
                f_low = -(_surface(side, low[0]) - SQUEEZE)
                go(0.4, "open", finger=O, **back, **held)
                go(0.6, "back off", finger=O, back=BACK_OFF, **back, **held)
                go(1.2, "move down the pole", finger=O, back=BACK_OFF, hold=low, **back)
                go(0.6, "close in on it", finger=O, hold=low, **back)
                steps.append(W(0.6, steps[-1].pos, f_low, "grip it", steps[-1].yaw))
                f, held = f_low, dict(hold=low)
            else:
                go(3.4, "hold the pole while the other hand moves down", **back, **held)
        go(2.5, "lift the picker", dz=LIFT_CMD, **back, **held)
        go(3.0, "hold it up", dz=LIFT_CMD, **back, **held)
        plan[side] = steps
    return plan


# -- offline checks --------------------------------------------------------------------------------------------------

def self_test() -> int:
    """The scoring rules on hand-built states and the sizes against each other (no GPU)."""
    cases = 0

    def check(cond, what):
        nonlocal cases
        assert cond, what
        cases += 1

    check(grade(True, 0.15, False) == 1.0, "in, lifted 15 cm: 1.0")
    check(grade(True, LIFTED, False) == 1.0, "in, lifted exactly 10 cm: 1.0")
    check(grade(True, 0.05, False) == 0.5, "in, 5 cm up: 0.5")
    check(grade(True, 0.0, False) == 0.5, "in, on the court: 0.5")
    check(grade(False, 0.15, False) == 0.0, "lifted without the ball: 0")
    check(grade(False, 0.0, False) == 0.0, "nothing done: 0")
    check(grade(True, 0.15, True) == 0.0, "the robot touched the ball: 0")
    check(grade(False, 0.0, True) == 0.0, "touched, not in: 0")
    check(inside_cage((0.0, 0.0, 0.0)), "ball at the axle: inside")
    check(inside_cage((0.0, 0.0, -(R - BALL_R - 0.002))), "ball resting on the bottom wires, inside: inside")
    check(inside_cage((0.01, 0.06, -0.05)), "ball off the middle, inside: inside")
    check(not inside_cage((GAP, 0.0, BALL_R - R)), "ball at its start, in front of the cage: outside")
    check(not inside_cage((R + BALL_R - 0.01, 0.0, BALL_R - R)), "ball pressed against the front wires: outside")
    check(not inside_cage((0.0, HALF + 0.01, 0.0)), "ball beyond a hub face: outside")
    check(inside_cage((0.0, 0.0, 0.05), axle_dir=(0.3, 0.95, 0.0)), "turned cage, ball at the middle: inside")
    gap = 2 * math.pi * R / N_WIRES - D_WIRE
    check(gap < 2 * BALL_R < R, "the wires' gap is narrower than the ball, which fits in the cage")
    check(PUSH > GAP, "the walk forward takes the cage past the ball's start")
    check(min(h[0] for h in REGRASP.values()) > 0.2 and max(h[0] for h in HOLD.values()) < S_END, "holds on the pole")
    check(REGRASP["right"][0] - REGRASP["left"][0] > 0.12 and HOLD["right"][0] - HOLD["left"][0] > 0.12, "hands apart")
    check(LIFT_CMD > LIFTED, "the lift aims above the bar")
    check(TABLE_X[1] < END_X and COURT_X - 0.8 > TABLE_X[1], "the bench ends before the pole's end and the mat")
    return cases


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules, then each arm's plan through the kinematics at 20 Hz.
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    n = self_test()
    plans = {side: ArmPlan(w, station.BASES[side]) for side, w in demo().items()}
    t, dt = 0.0, 0.05
    while t < max(p.duration for p in plans.values()):
        t += dt
        for p in plans.values():
            p.action7(t)
    worst = {side: (round(p.worst_ik_mm, 2), round(p.worst_ik_deg, 2)) for side, p in plans.items()}
    assert all(mm < 3.0 and deg < 1.0 for mm, deg in worst.values()), worst
    print(f"self-test: {n} cases pass; the demo's IK, worst (mm, deg): {worst}; "
          f"{plans['right'].duration:.1f} s")
