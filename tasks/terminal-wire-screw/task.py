"""Panel wiring: "Land the stripped wire in the empty terminal and tighten it."

A control-panel mounting plate laid flat on the bench, with five 35 mm2 screw-clamp terminal blocks
on a DIN rail; four have their wires landed. The incoming conductor comes up out of the cable duct
beside the empty middle block, stripped 18 mm, springing back off its line. The bare end has to go
fully into the block's clamp and be held there while the clamp screw is driven tight with the
screwdriver from the holder, turning it, letting go and taking a fresh grip as a hand would.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np

_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

import simkit  # noqa: E402
import terminal as T  # noqa: E402
from scoring import TrialScorer  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP as PAD_TIP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "terminal-wire-screw"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
MAX_STEPS = 1600
OBJECT_NAMES = ("wire", "carrier", "yoke", "screw", "driver")

DOWN = simkit.patch_demo()


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    specs = (T.plate_object(), T.rail_object(), T.duct_object(), T.carrier_object(), T.yoke_object(), T.wire_object(),
             T.screw_object(), T.holder_object(), T.driver_object())
    objects += [yscene.task_object(s) for s in specs]
    cfg = IsaacLabArenaEnvironment(name="yam_terminal_wire_screw", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


SHARED = {}                                          # measured state the scripted demo steers by


class Dynamics:
    """What rigid bodies cannot do by themselves, from measured poses only (a disclosed stand-in).

    * The conductor is a rigid rod from where it leaves the duct, on a sprung slide (the slack it can
      be drawn out by, about 1 N pulling it back when home) and a sprung swivel (sideways and up and
      down), so it springs back toward its rest line when let go.
    * The clamp screw turns only with the screwdriver's tip seated in its head's flared recess (tip
      within 2 mm of the screw's axis and down on the recess floor, shaft within 12 degrees of
      vertical), by the driver's measured twist about its axis (turned back, it loosens). From 0.3 turn
      the clamp bears on whatever is in the channel: with the bare end 8 mm or more in, the conductor
      is held where it is (its slide and swivel locked stiff); the scorer counts 0.3 turn as clamped
      and 0.6 as tight. No reaction torque is put on the driver.
    """

    def __init__(self, env):
        import omni.usd
        self.base = env.unwrapped
        stage = omni.usd.get_context().get_stage()
        self.slide = self.yaw = self.pitch = self.screw_drive = None
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path.endswith("/carrier/spring_joint"):
                self.slide = prim
            if path.endswith("/yoke/hinge_joint"):
                self.yaw = prim
            if path.endswith("/wire/hinge_joint"):
                self.pitch = prim
            if path.endswith("/screw/hinge_joint"):
                self.screw_drive = prim.GetAttribute("drive:angular:physics:targetPosition")
        if None in (self.slide, self.yaw, self.pitch, self.screw_drive):
            raise RuntimeError("conductor or screw joints not found on the stage")

    def reset(self):
        self.turns = 0.0
        self.engaged = False
        self.twist = None
        self.locked = False
        self.screw_drive.Set(0.0)
        self._lock(None)

    def _lock(self, at):
        """Hold the conductor where it is (``at`` = slide m, yaw deg, pitch deg) or free it to spring back."""
        if at is None:
            self.slide.GetAttribute("drive:linear:physics:stiffness").Set(T.SPRING_K)
            self.slide.GetAttribute("drive:linear:physics:targetPosition").Set(0.0)
            self.yaw.GetAttribute("drive:angular:physics:stiffness").Set(T.YAW_K * math.pi / 180)
            self.yaw.GetAttribute("drive:angular:physics:targetPosition").Set(T.REST_YAW)
            self.pitch.GetAttribute("drive:angular:physics:stiffness").Set(T.PITCH_K * math.pi / 180)
            self.pitch.GetAttribute("drive:angular:physics:targetPosition").Set(-T.REST_PITCH)
            self.locked = False
            return
        s, yaw, pitch = at
        self.slide.GetAttribute("drive:linear:physics:stiffness").Set(T.CLAMPED_K)
        self.slide.GetAttribute("drive:linear:physics:targetPosition").Set(float(s))
        self.yaw.GetAttribute("drive:angular:physics:stiffness").Set(50.0)
        self.yaw.GetAttribute("drive:angular:physics:targetPosition").Set(float(yaw))
        self.pitch.GetAttribute("drive:angular:physics:stiffness").Set(50.0)
        self.pitch.GetAttribute("drive:angular:physics:targetPosition").Set(float(pitch))
        self.locked = True

    def before_step(self, step, action):
        pass

    @staticmethod
    def _twist(driver):
        dx = T.quat_matrix(driver[3:7]) @ np.array([1.0, 0.0, 0.0])
        return math.atan2(float(dx[1]), float(dx[0]))

    def after_step(self, step, objects, root_poses, robot):
        wire = np.asarray(root_poses["wire"], float)
        carrier = np.asarray(root_poses["carrier"], float)
        driver = np.asarray(root_poses["driver"], float)
        depth, gap, lined = T.in_clamp(wire)
        ins_end, tip_w, axis = T.wire_end(wire)
        # -- the clamp screw
        tip, d_axis = T.driver_tip(driver)
        head = T.head_centre()
        rel = tip - head
        radial = float(math.hypot(rel[0], rel[1]))
        ang = math.degrees(math.acos(np.clip(float(d_axis[2]), -1, 1)))
        SHARED["socket_err"] = (head - tip)[:2]
        SHARED["tip"] = tip.copy()
        SHARED["head"] = head.copy()
        SHARED["left_tcp"] = np.asarray(robot["left"]["tcp"], float)[:3]
        engaged = radial < 0.002 and -0.003 < rel[2] < 0.0015 and ang < 12
        SHARED["engaged"] = bool(engaged)
        SHARED["wire_tip"] = tip_w.copy()
        SHARED["grip_pt"] = (driver[:3] + d_axis * (T.SHAFT[1] / 2 + T.HANDLE[1] / 2 - GRIP_LOW)).copy()
        tw = self._twist(driver)
        if engaged and self.engaged and self.twist is not None:
            d = (tw - self.twist + math.pi) % (2 * math.pi) - math.pi
            self.turns = max(-0.25, self.turns - d / (2 * math.pi))     # clockwise from above tightens
        self.engaged, self.twist = engaged, tw
        self.screw_drive.Set(float(-self.turns * 360.0))
        # -- the clamp on the conductor
        if self.turns >= T.TURNS_CLAMPED and not self.locked and lined and depth >= 0.008:
            yaw = math.degrees(math.atan2(-float(axis[0]), float(axis[1])))
            pitch = math.degrees(math.asin(float(np.clip(axis[2], -1, 1))))
            self._lock((float(carrier[1] - T.ROOT[1]), yaw, pitch))
        elif self.locked and self.turns < T.TURNS_CLAMPED:
            self._lock(None)
        yaw_d = math.degrees(math.atan2(-float(axis[0]), float(axis[1])))
        pitch_d = math.degrees(math.asin(float(np.clip(-axis[2], -1, 1))))           # + drooping
        return {"yaw_deg": round(yaw_d, 1), "pitch_deg": round(pitch_d, 1), "bare_in_mm": round(1000 * depth, 1), "insulation_gap_mm": round(1000 * gap, 1), "lined_up": bool(lined),
                "screw_turns": round(self.turns, 2), "clamped": bool(self.locked), "driver_on_screw": bool(engaged),
                "slide_mm": round(1000 * float(carrier[1] - T.ROOT[1]), 1)}


def make_dynamics(env):
    return Dynamics(env)


def make_scorer():
    return TrialScorer()


# -- demo ----------------------------------------------------------------------------------

ACROSS_Y = simkit.rot_z(math.pi / 2) @ DOWN          # jaws closing along x, across a conductor along y
REST = {"left": (0.16, 0.24, T.TOP + 0.16), "right": (0.16, -0.24, T.TOP + 0.16)}
SEEN_PITCH = 6.7                                     # deg: its droop as it actually lies (the spring and its weight)
WIRE_GRIP = 0.130                                    # hold the conductor this far back from the insulation's end
WIRE_FINGER = -(2 * T.INS_R - 0.004) / 2
DRIVER_FINGER = -(2 * T.HANDLE[0] - 0.018) / 2       # squeeze hard: it must not turn in the hand
DRIVER_RELEASE = -(2 * T.HANDLE[0] + 0.016) / 2     # let go by opening the hand only so far: it does not sweep the grip
GRIP_LOW = 0.012                                     # take the grip below its middle


def _tcp(tip, rot, label, seconds, finger):
    tip = np.asarray(tip, float)
    return Waypoint(seconds, tuple(map(float, tip - rot @ np.array([0.0, 0.0, PAD_TIP]))), finger, label, rot)


def _rest_grip_point():
    """Where the grip point on the conductor lies at rest (its springs' rest angles, on the plate)."""
    r = T.wire_rot(T.REST_YAW, SEEN_PITCH)
    p = T.ROOT + r @ np.array([0.0, T.INS_L - WIRE_GRIP, 0.0])
    return p


def _wire_hand(seconds, point, finger, label, lift=0.0, dz=-0.0085):
    """Hold the conductor at ``point``: a round conductor settles at the pads' tips once gripped, so
    the hand reaches lower (``dz``) to take it."""
    x, y, z = point
    return _tcp((x, y, z + dz + lift), ACROSS_Y, label, seconds, finger)


def _driver_hand(seconds, tip_xyz, turn, finger, label):
    """Grip the screwdriver's handle, given where its tip should be; ``turn`` (rad) about the vertical."""
    x, y, z = tip_xyz
    mid = z + T.SHAFT[1] + T.HANDLE[1] / 2 - GRIP_LOW
    return _tcp((x, y, mid), simkit.rot_z(turn) @ DOWN, label, seconds, finger)


def demo():
    g0 = _rest_grip_point()
    g0[2] = max(g0[2], T.PLATE_TOP + 0.010)
    f = WIRE_FINGER
    line = np.array([T.TARGET_X, T.FACE_Y - WIRE_GRIP, T.WINDOW_Z])
    home = line + np.array([0.0, 0.010, 0.0])            # pressed on till the insulation butts on the block face
    right = [
        Waypoint(1.5, REST["right"], FINGER_OPEN, "look", DOWN),
        _wire_hand(2.0, g0, FINGER_OPEN, "over the conductor", lift=0.05, dz=-0.009),
        _wire_hand(1.2, g0, FINGER_OPEN, "down round it", dz=-0.009),
        _wire_hand(0.8, g0, f, "take it", dz=-0.009),
        _wire_hand(1.5, line - np.array([0.0, 0.026, 0.0]), f, "lift it in line with the clamp"),
        _wire_hand(1.0, line - np.array([0.0, 0.020, 0.0]), f, "bare end at the opening"),
        _wire_hand(2.0, home, f, "push it in to the insulation"),
    ]
    t_home = sum(w.duration for w in right)
    tip0 = T.driver_tip_rest()
    head = T.head_centre()
    sx, sy = head[0], head[1]
    g = DRIVER_FINGER
    a = math.radians(55.0)                            # wrist turned back (its last joint stops at 120 degrees)
    b = math.radians(110.0)                           # ... and turned on this far
    left = [
        Waypoint(2.0, REST["left"], FINGER_OPEN, "look", DOWN),
        _driver_hand(2.0, (tip0[0], tip0[1], tip0[2] + 0.05), 0.0, FINGER_OPEN, "over the screwdriver"),
        _driver_hand(1.2, tip0, 0.0, FINGER_OPEN, "round its grip"),
        _driver_hand(0.8, tip0, 0.0, g, "take it"),
        _driver_hand(1.5, (tip0[0], tip0[1], tip0[2] + 0.06), 0.0, g, "lift it out of the holder"),
        _driver_hand(max(0.5, t_home - 5.0), (sx, sy + 0.09, head[2] + 0.025), a, g, "wait, holding it"),
        _driver_hand(1.5, (sx, sy, head[2] + 0.018), a, g, "over the clamp screw"),
        _driver_hand(1.5, (sx, sy, head[2] + 0.018), a, g, "over the clamp screw"),
        _driver_hand(1.5, (sx, sy, head[2] + 0.004), a, g, "over the clamp screw"),
        _driver_hand(1.5, (sx, sy, head[2] + SEAT), a, g, "tip into the screw head"),
    ]
    z = head[2] + SEAT
    for k in range(STROKES):
        left += [_driver_hand(1.0, (sx, sy, z), 0.0, g, "turn it clockwise"),
                 _driver_hand(1.0, (sx, sy, z), -b, g, "turn it clockwise"),
                 _driver_hand(0.4, (sx, sy, z), -b, g, "ease off: hold still where it is"),
                 _driver_hand(0.8, (sx, sy, z), -b, DRIVER_RELEASE, "let go of the grip"),
                 _driver_hand(0.4, (sx, sy, z + 0.010), -b, FINGER_OPEN, "lift the open hand a little"),
                 _driver_hand(0.5, (sx, sy, z + 0.010), 0.0, FINGER_OPEN, "turn the open hand back round it"),
                 _driver_hand(0.5, (sx, sy, z + 0.010), a, FINGER_OPEN, "turn the open hand back round it"),
                 _driver_hand(0.4, (sx, sy, z), a, FINGER_OPEN, "turn the open hand back round it"),
                 _driver_hand(0.6, (sx, sy, z), a, FINGER_OPEN + 1e-6, "turn the open hand back round it"),
                 _driver_hand(0.5, (sx, sy, z), a, g, "grip it again")]
    t_tight = sum(w.duration for w in left)
    left += [_driver_hand(1.0, (sx, sy, head[2] + 0.018), a, g, "lift the screwdriver off"),
             _driver_hand(2.0, (tip0[0], tip0[1], tip0[2] + 0.06), 0.0, g, "back over the holder"),
             _driver_hand(1.5, (tip0[0], tip0[1], tip0[2] + 0.002), 0.0, g, "into the holder"),
             _driver_hand(0.6, (tip0[0], tip0[1], tip0[2] + 0.002), 0.0, FINGER_OPEN, "let go"),
             Waypoint(1.5, REST["left"], FINGER_OPEN, "stand clear", DOWN),
             Waypoint(1.5, REST["left"], FINGER_OPEN, "done", DOWN)]
    right += [_wire_hand(max(0.5, t_tight - t_home), home, f + 1e-6, "hold it in while the screw goes down"),
              _wire_hand(0.6, home, FINGER_OPEN, "let go"),
              _wire_hand(0.8, home, FINGER_OPEN, "lift off", lift=0.05),
              Waypoint(1.5, REST["right"], FINGER_OPEN, "stand clear", DOWN)]
    t_r = sum(w.duration for w in right)
    t_l = sum(w.duration for w in left)
    if t_l > t_r:
        right.append(Waypoint(t_l - t_r + 0.5, REST["right"], FINGER_OPEN, "done", DOWN))
    return {"left": left, "right": right}


DRIVER_DZ = T.SHAFT[1] + T.HANDLE[1] / 2 - GRIP_LOW + PAD_TIP   # planned TCP height above the tip
REGRIP_LABELS = ("lift the open hand a little", "turn the open hand back round it", "grip it again")
WIRE_LABELS = ("lift it in line with the clamp", "bare end at the opening", "push it in to the insulation",
               "hold it in while the screw goes down")
Z_LABELS = ("over the clamp screw", "tip into the screw head", "turn it clockwise")
SEAT = 0.0002                                         # tip this far above the head's top when on it (it rests on the head)
STROKES = 2
ALIGN_LABELS = ("over the clamp screw", "tip into the screw head", "turn it clockwise", "let go of the grip",
                "lift the open hand a little", "turn the open hand back round it", "grip it again")
GRASP_LABELS = ("tip into the screw head", "grip it again")
HOLD_LABELS = ("ease off: hold still where it is", "let go of the grip")


class DemoController:
    """The scripted plans, with the left hand's position nudged while driving by the measured offset
    between the screwdriver's tip and the screw head (a scripted demo may steer by sim state)."""

    def __init__(self, plans):
        self.plans = plans
        self.corr = np.zeros(2)
        self.hold = None
        self.off = None
        self.last_xy = None
        self.wcorr = np.zeros(3)                        # right hand: steering the bare end onto the opening
        self.dz = None                                  # the grip's height on the screwdriver, as measured
        self.first_release = False

    def action(self, t):
        from yam_arena import ik
        FC, FO = FINGER_CLOSED, FINGER_OPEN
        out = []
        for side in ("left", "right"):
            plan = self.plans[side]
            pos, rot, finger, label = plan.target(t)
            pos = np.array(pos, float)
            if side == "left" and label in HOLD_LABELS and "left_tcp" in SHARED:
                if self.hold is None:
                    self.hold = np.array(SHARED["left_tcp"], float)
                pos = self.hold.copy()
            elif side == "left":
                self.hold = None
            if side == "left" and label != "turn it clockwise":
                self.last_xy = None
            if side == "left" and label == "let go of the grip":
                self.first_release = True
            if side == "left" and label in Z_LABELS and not self.first_release and "tip" in SHARED:
                # the screwdriver sits in the hand where it was taken from the holder: put its tip, not
                # the planned grip, where the plan wants it
                if label == "over the clamp screw" or self.dz is None:
                    self.dz = float(SHARED["left_tcp"][2] - SHARED["tip"][2]) - DRIVER_DZ
                pos[2] += self.dz
            if side == "left" and label in GRASP_LABELS and "tip" in SHARED:
                d = np.asarray(SHARED["tip"], float) - np.asarray(SHARED["left_tcp"], float)
                d[2] = 0.0
                self.off = rot.T @ d
            if side == "left" and label == "turn it clockwise" and self.off is not None and SHARED.get("engaged"):
                want = np.asarray(SHARED["head"], float)[:2] - (rot @ self.off)[:2] - self.corr
                prev = self.last_xy if self.last_xy is not None else pos[:2]
                pos[:2] = prev + np.clip(want - prev, -0.001, 0.001)
                self.last_xy = pos[:2].copy()
            if side == "left" and label in ALIGN_LABELS and label not in HOLD_LABELS and "socket_err" in SHARED:
                err = np.asarray(SHARED["socket_err"], float)
                if (label in ("over the clamp screw", "tip into the screw head") and float(np.linalg.norm(err)) < 0.025) or \
                        (label == "turn it clockwise" and float(np.linalg.norm(err)) < 0.003):
                    self.corr = np.clip(self.corr + 0.05 * err, -0.02, 0.02)
                pos[:2] += self.corr
            if side == "left" and label in REGRIP_LABELS and self.first_release and "grip_pt" in SHARED:
                # the open hand goes back round the screwdriver where it actually stands
                # centred over the tip, which stays down the guide hole: closing the hand there stands
                # a leaning screwdriver back up
                pos[:2] = np.asarray(SHARED["tip"], float)[:2]
            if side == "right" and label in WIRE_LABELS and "wire_tip" in SHARED:
                tip = np.asarray(SHARED["wire_tip"], float)
                err = np.array([T.TARGET_X - tip[0], 0.0, T.WINDOW_Z - tip[2]])
                if label != "hold it in while the screw goes down":
                    self.wcorr = np.clip(self.wcorr + 0.08 * err, -0.02, 0.02)
                pos += self.wcorr
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), plan.q, base_pos=plan.base_pos, iters=60)
            plan.q = q
            grip = float(np.clip(finger / FO, 0.0, 1.0)) if FO != FC else 1.0
            out.append(np.r_[q, grip])
        return np.concatenate(out)


def make_demo_controller(env, plans):
    return DemoController(plans)
