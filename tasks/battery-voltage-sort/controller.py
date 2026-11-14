"""Reference demonstration using measured poses and the meter's observed reading.

The demonstration has privileged pose feedback. It sends ordinary joint/gripper
targets, reads the instrument after contacting each battery, and chooses bins
from those readings. It never assigns object poses or changes physics.
"""

import math

import numpy as np
from yam_arena import ik, runtime as yam_runtime, station
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import grip_rotation

import scoring
import spec as S


def together(*generators):
    pending = [iter(g) for g in generators]
    while pending:
        for generator in list(pending):
            try:
                next(generator)
            except StopIteration:
                pending.remove(generator)
        if pending:
            yield


class Arm:
    def __init__(self, side, env):
        self.side, self.robot = side, env.unwrapped.scene[side+"_robot"]
        self.base = np.asarray(station.BASES[side])
        self.q = np.array(READY, dtype=float)
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.grip = 1.0
        self.command = np.r_[self.q, self.grip]
        self.worst_ik_mm = 0.0

    def aim(self, position, rotation=None, grip=None):
        rotation = self.rot if rotation is None else rotation
        if grip is not None:
            self.grip = float(grip)
        self.q, error, _ = ik.solve(np.asarray(position), ik.matrix_to_quat_xyzw(rotation), self.q,
                                    base_pos=self.base, iters=80)
        self.worst_ik_mm = max(self.worst_ik_mm, error*1000)
        self.pos, self.rot = np.asarray(position).copy(), rotation.copy()
        self.command = np.r_[self.q, self.grip]

    def move(self, position, duration=1.2, yaw=None, opening=None):
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        goal = np.asarray(position)
        r1 = self.rot if yaw is None else self.reachable_rotation(goal, yaw)
        g1 = self.grip if opening is None else opening / (-2*FINGER_OPEN)
        count = max(1, round(duration*S.HZ))
        for k in range(1, count+1):
            amount = ik.smoothstep(k/count)
            self.aim(p0+amount*(goal-p0), ik.slerp_matrix(r0, r1, amount), g0+amount*(g1-g0))
            yield

    def reachable_rotation(self, position, yaw):
        options = []
        for turn in (yaw, yaw+math.pi, yaw-math.pi):
            rotation = grip_rotation(turn)
            _, pe, re = ik.solve(np.asarray(position), ik.matrix_to_quat_xyzw(rotation), self.q,
                                 base_pos=self.base, iters=200)
            options.append((pe+0.05*re, rotation))
        return min(options, key=lambda x: x[0])[1]

    def actual_tcp(self):
        return yam_runtime.tcp_world(self.robot)


class Demonstration:
    def __init__(self, env, state):
        self.env, self.state = env, state
        self.arms = {side: Arm(side, env) for side in ("left", "right")}
        self.readings = {}
        self.phase = "starting"

    @property
    def action(self):
        return np.r_[self.arms["left"].command, self.arms["right"].command]

    def wait(self, seconds):
        for _ in range(max(1, round(seconds*S.HZ))):
            yield

    def phase_start(self, name):
        self.phase = name
        print("DEMO:", name, flush=True)

    def take_probes(self):
        self.phase_start("pick up both loose probes")
        moves = []
        for side, name in zip(("left", "right"), S.PROBE_NAMES, strict=True):
            arm = self.arms[side]
            pos = self.state.bodies[name].pos
            moves.append(arm.move((pos[0], pos[1], S.TOP+0.145), 1.5, yaw=math.pi/2, opening=0.035))
        yield from together(*moves)
        yield from together(*(self.arms[side].move(self.state.bodies[name].pos+[0, 0, 0.000], 1.2)
                              for side, name in zip(("left", "right"), S.PROBE_NAMES, strict=True)))
        yield from together(*(arm.move(arm.pos, 0.8, opening=0.006) for arm in self.arms.values()))
        yield from self.wait(0.4)
        for side, arm in self.arms.items():
            print("GRASP:", side, "tcp", arm.actual_tcp().tolist(), "joints", arm.robot.data.joint_pos.torch[0].cpu().tolist(), flush=True)
        self.phase_start("lift both probes")
        yield from together(*(arm.move((arm.pos[0], arm.pos[1], S.TOP+0.12), 1.0) for arm in self.arms.values()))
        for name in S.PROBE_NAMES:
            if self.state.bodies[name].pos[2] < S.TOP+0.080:
                raise RuntimeError("Probe grasp did not lift " + name)

    def contact_targets(self, cell_name, outside=0.0, integrate=False):
        cell = self.state.bodies[cell_name]
        targets = {}
        for side, probe_name, sign in (("left", "red_probe", 1), ("right", "black_probe", -1)):
            probe = self.state.bodies[probe_name]
            desired = cell.pos + cell.rotation[:, 1] * sign*(S.CELL_HALF+S.TIP_RADIUS+outside)
            origin = self.arms[side].pos if integrate else self.arms[side].actual_tcp()
            error = desired-scoring.tip_position(probe)
            targets[side] = origin + (0.35*error if integrate else error)
        return targets

    def measure(self, name):
        self.phase_start("measure " + name)
        # Move above the ends with both needles safely clear, then descend.
        targets = self.contact_targets(name, outside=0.015)
        yield from together(*(self.arms[side].move((target[0], target[1], S.TOP+0.125), 1.3)
                              for side, target in targets.items()))
        targets = self.contact_targets(name, outside=0.014)
        yield from together(*(self.arms[side].move(target, 1.1) for side, target in targets.items()))
        # Servo the observed tips onto the observed terminal faces, using gentle
        # opposed contacts. Pose feedback is used only by this reference demo.
        for k in range(160):
            outside = max(-0.0002, 0.014-(k+1)*0.0005)
            targets = self.contact_targets(name, outside, integrate=True)
            for side, target in targets.items():
                arm = self.arms[side]
                delta = target-arm.pos
                distance = np.linalg.norm(delta)
                if distance > 0.001:
                    delta *= 0.001/distance
                arm.aim(arm.pos+delta)
            if name in self.state.trial.measured:
                reading = abs(self.state.trial.reading_cv or self.state.trial.measured[name])
                self.readings[name] = reading
                print(f"METER: {name} = {reading/100:.2f} V", flush=True)
                break
            yield
        else:
            diagnostic = {probe: {"position": self.state.bodies[probe].pos.tolist(),
                                  "tip": scoring.tip_position(self.state.bodies[probe]).tolist(),
                                  "forces": self.state.forces[probe]} for probe in S.PROBE_NAMES}
            raise RuntimeError(f"No stable two-probe measurement for {name}: {diagnostic}")
        yield from self.wait(0.2)
        yield from together(*(arm.move((arm.pos[0], arm.pos[1] + (0.018 if side == "left" else -0.018), arm.pos[2]), 0.6)
                              for side, arm in self.arms.items()))
        yield from together(*(arm.move((arm.pos[0], arm.pos[1], S.TOP+0.125), 0.7) for arm in self.arms.values()))

    def park_probes(self):
        self.phase_start("put both probes back in their rests")
        yield from together(*(self.arms[side].move((rest[0], rest[1], S.TOP+0.125), 1.4)
                              for side, rest in zip(("left", "right"), S.PROBE_RESTS.values(), strict=True)))
        moves = []
        for side, name in zip(("left", "right"), S.PROBE_NAMES, strict=True):
            # Account for the actual offset of the grasped probe in the fingers.
            target = self.arms[side].actual_tcp() + S.PROBE_RESTS[name]-self.state.bodies[name].pos
            moves.append(self.arms[side].move(target, 1.2))
        yield from together(*moves)
        yield from together(*(arm.move(arm.pos, 0.8, opening=0.040) for arm in self.arms.values()))
        yield from together(*(arm.move((arm.pos[0], arm.pos[1], S.TOP+0.15), 0.8) for arm in self.arms.values()))

    def sort_cell(self, name, slot):
        target = "alive" if self.readings[name] >= S.THRESHOLD_CV else "dead"
        side = "left" if target == "alive" else "right"
        arm = self.arms[side]
        self.phase_start(f"place {name} in {target} bin")
        cell = self.state.bodies[name]
        yaw = math.atan2(-cell.rotation[0, 1], cell.rotation[1, 1])+math.pi/2
        yield from arm.move((cell.pos[0], cell.pos[1], S.TOP+0.125), 1.2, yaw=yaw, opening=0.034)
        yield from arm.move(self.state.bodies[name].pos+[0, 0, 0.006], 1.1)
        yield from arm.move(arm.pos, 0.7, opening=0.008)
        yield from self.wait(0.3)
        yield from arm.move((arm.pos[0], arm.pos[1], S.TOP+0.14), 1.2)
        destination = S.BIN_CENTERS[target].copy()
        destination[0] += (slot % 3-1)*0.050
        if slot >= 3:
            destination[1] += 0.055 if target == "alive" else -0.055
        destination[2] = S.BIN_FLOOR+S.CELL_RADIUS+0.004
        yield from arm.move((destination[0], destination[1], S.TOP+0.14), 1.5)
        offset = arm.actual_tcp()-self.state.bodies[name].pos
        yield from arm.move(destination+offset, 1.1)
        yield from arm.move(arm.pos, 0.6, opening=0.040)
        yield from arm.move((arm.pos[0], arm.pos[1], S.TOP+0.14), 1.1)
        yield from self.wait(0.4)

    def run(self):
        yield from self.take_probes()
        for name in S.CELL_NAMES:
            yield from self.measure(name)
        yield from self.park_probes()
        slots = {"alive": 0, "dead": 0}
        for name in S.CELL_NAMES:
            target = "alive" if self.readings[name] >= S.THRESHOLD_CV else "dead"
            yield from self.sort_cell(name, slots[target])
            slots[target] += 1
        self.phase_start("release and wait for the final-state score")
        yield from self.wait(2.0)
