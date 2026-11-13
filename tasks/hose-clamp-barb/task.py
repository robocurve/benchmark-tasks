"""Pump maintenance: "Connect the hose to the pump's outlet and clamp it."

A coolant pump with a barbed outlet, a rubber hose with a worm-drive clamp run down loosely near its
end, and an 8 mm nut driver in a holder. The hose is pushed over the barb's bead right up to the
outlet boss, which brings the clamp's band onto the barb's shank, and the clamp's screw is driven
tight with the nut driver, turning it, letting go and taking a fresh grip as a hand would.
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
import hose as H  # noqa: E402
from scoring import TrialScorer  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP as PAD_TIP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "hose-clamp-barb"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
MAX_STEPS = 1600
OBJECT_NAMES = ("hose", "clamp", "screw", "driver")

DOWN = simkit.patch_demo()


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    specs = (H.pump_object(), H.barb_trim_object(), H.hose_object(), H.clamp_object(), H.screw_object(), H.holder_object(), H.driver_object())
    objects += [yscene.task_object(s) for s in specs]
    cfg = IsaacLabArenaEnvironment(name="yam_hose_clamp_barb", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


SHARED = {}                                          # measured state the scripted demo steers by


class Dynamics:
    """What rigid bodies cannot do by themselves, from measured poses only (a disclosed stand-in).

    * Rubber on the barb: the hose is a rigid tube, so its grip on the barb is a slide joint along the
      barb's axis, switched on once the mouth is lined up with the barb (centre within 3 mm, axis within
      12 degrees) and pushed 6 mm on (the bead is drawn only: the barb's collider is a plain pin and
      cone, so this switch-on and the drag below stand in for forcing the bore over it; once on, the
      joint also holds the hose on the barb's axis). Its damping drive resists the
      push (about 3 N at 12 mm/s); pushed within 4 mm of home, a spring also holds it against being drawn
      back off (3 kN/m); clamped tight, it is held where it is. Drawn back to 4 mm on, it lets go.
    * The clamp, run down by hand before the hose went on, grips the hose: a stiff slide joint holds it.
    * The worm screw turns only with the socket seated over its head (socket centred within 2 mm, its
      rim 2 to 10 mm down the head, shaft within 12 degrees of the screw's axis), with the driver's
      twist about its own axis measured against the clamp body (turned back, it loosens). From 0.25 turn
      the band is drawn pulled down on the hose, from 0.5 it holds the hose on the barb firmly; the scorer
      counts 0.4 turn as snug and 0.8 as tight. No reaction
      torque is put on the driver (the wrench below is always zero; kept so the recorded run is unchanged).
    """

    def __init__(self, env):
        import omni.usd
        self.base = env.unwrapped
        stage = omni.usd.get_context().get_stage()
        self.slide = self.screw_drive = self.grip = None
        self.band = [None] * (2 * H.BAND_PIECES)
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            for k in range(2 * H.BAND_PIECES):
                if path.endswith(f"/clamp/piece{k}"):
                    self.band[k] = prim
            if path.endswith("/hose/spring_joint"):
                self.grip = prim
            if path.endswith("/clamp/spring_joint"):
                self.slide = prim.GetAttribute("drive:linear:physics:targetPosition")
            if path.endswith("/screw/hinge_joint"):
                self.screw_drive = prim.GetAttribute("drive:angular:physics:targetPosition")
        if self.slide is None or self.screw_drive is None or self.grip is None:
            raise RuntimeError("clamp slide or screw joint not found on the stage")
        import torch
        self.torch = torch
        hose0 = np.asarray(H.hose_object().pos)
        self.rel0 = H.clamp_object().pos[1] - hose0[1]

    def _wrench(self, name, force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)):
        t = self.torch
        dev = self.base.device
        obj = self.base.scene[name]
        obj.permanent_wrench_composer.set_forces_and_torques_index(
            forces=t.tensor([[list(map(float, force))]], dtype=t.float32, device=dev),
            torques=t.tensor([[list(map(float, torque))]], dtype=t.float32, device=dev), is_global=True)

    def reset(self):
        self.turns = 0.0
        self.snug = False
        self.engaged = False
        self.twist = None
        self.max_depth = 0.0
        self.gripping = self.locked = self.seated = False
        self.seat_at = 0.0
        self.opening = {}
        self.hose_hands = ()
        self.driver_hands = ()
        self.slide.Set(0.0)
        self.screw_drive.Set(0.0)
        self._grip(False)
        self._band(False)
        self._wrench("hose")
        self._wrench("driver")

    def before_step(self, step, action):
        """A hand told to open lets go of what it drives: drop the forces at once, not a step late."""
        a = np.asarray(action, float).ravel()
        self.opening = {"left": float(a[6]) > 0.4, "right": float(a[13]) > 0.4} if a.size >= 14 else {}
        if any(self.opening.get(s) for s in self.hose_hands):
            self._wrench("hose")
            self.hose_hands = ()
        if any(self.opening.get(s) for s in self.driver_hands):
            self._wrench("driver")
            self.driver_hands = ()

    def _grip(self, on, lock_at=None):
        """Switch the hose's grip on the barb on or off; ``lock_at`` (m of depth) clamps it there."""
        self.grip.GetAttribute("physics:jointEnabled").Set(bool(on))
        self.grip.GetAttribute("drive:linear:physics:stiffness").Set(H.CLAMPED_K if lock_at is not None else 0.0)
        self.grip.GetAttribute("drive:linear:physics:targetPosition").Set(float(lock_at or 0.0))
        self.gripping = bool(on)
        self.locked = lock_at is not None
        if not on:
            self.seated = False
            self.seat_at = 0.0

    def _band(self, tight):
        """Draw the band standing off the hose, or pulled down tight on it."""
        from pxr import UsdGeom
        for k, prim in enumerate(self.band):
            img = UsdGeom.Imageable(prim)
            (img.MakeVisible() if (k >= H.BAND_PIECES) == tight else img.MakeInvisible())

    def _clamp_slide(self, hose, clamp):
        r = H.quat_matrix(hose[3:7])
        rel = r.T @ (clamp[:3] - hose[:3])
        return float(rel[1] - self.rel0)

    def _twist(self, driver, ref):
        """Driver's turn about the screw's axis (rad): angle of its x axis in the plane of ``ref``'s x, y."""
        rs = H.quat_matrix(ref[3:7])
        dx = H.quat_matrix(driver[3:7]) @ np.array([1.0, 0.0, 0.0])
        return math.atan2(float(np.dot(dx, rs[:, 1])), float(np.dot(dx, rs[:, 0])))

    def after_step(self, step, objects, root_poses, robot):
        hose = np.asarray(root_poses["hose"], float)
        clamp = np.asarray(root_poses["clamp"], float)
        screw = np.asarray(root_poses["screw"], float)
        driver = np.asarray(root_poses["driver"], float)
        # -- rubber on the barb
        depth, lined = H.on_barb(hose)
        self.max_depth = max(self.max_depth, depth)
        mouth, axis = H.hose_axis(hose)
        raw = float(mouth[1] - H.TIP_Y)
        if not self.gripping and lined and depth > H.BEAD_ENGAGE:
            self._grip(True)
        elif self.gripping and raw < H.PULL_OFF_DEPTH:
            self._grip(False)
        tight = self.turns >= H.BAND_GRIPS and self.clamp_on_shank(hose, clamp)
        if self.gripping and tight and not self.locked:
            self._grip(True, lock_at=raw)
        elif self.gripping and not self.locked and raw >= H.INSERT - 0.004 and raw > self.seat_at + 0.0003:
            # pushed (nearly) home: the rubber grips the barb's whole length -- it holds against being
            # drawn back off, and the hose can still be pushed further on
            self.seated = True
            self.seat_at = raw
            self.grip.GetAttribute("drive:linear:physics:stiffness").Set(H.SEATED_K)
            self.grip.GetAttribute("drive:linear:physics:targetPosition").Set(float(raw))
        force = np.zeros(3)
        # -- clamp friction on the hose
        # the clamp, run down by hand before the hose went on, grips the hose: its slide joint holds it
        # -- the worm screw
        tip, d_axis = H.driver_tip(driver)
        s_axis = H.quat_matrix(screw[3:7]) @ np.array([0.0, 0.0, 1.0])
        head = screw[:3] + s_axis * H.SCREW_H / 2
        rel = tip - head
        along = float(np.dot(rel, s_axis))
        radial = float(np.linalg.norm(rel - along * s_axis))
        ang = math.degrees(math.acos(np.clip(float(np.dot(d_axis, s_axis)), -1, 1)))
        SHARED["socket_err"] = (head - tip)[:2]          # for the scripted demo's alignment only
        SHARED["left_tcp"] = np.asarray(robot["left"]["tcp"], float)[:3]
        SHARED["tip"] = tip.copy()
        SHARED["head"] = head.copy()
        engaged = radial < 0.002 and -H.SOCKET_BORE[1] - 0.001 < along < -0.002 and ang < 12   # socket over the head
        SHARED["engaged"] = bool(engaged)
        tw = self._twist(driver, clamp)                # measured against the clamp body, which does not turn
        torque = np.zeros(3)
        if engaged and self.engaged and self.twist is not None:
            d = (tw - self.twist + math.pi) % (2 * math.pi) - math.pi
            self.turns = max(-0.25, self.turns - d / (2 * math.pi))   # clockwise from above tightens
            if self.turns >= H.BAND_DRAWN_TIGHT:
                if not self.snug:
                    self._band(True)
                self.snug = True
                load = H.SNUG_TORQUE + (H.TIGHT_TORQUE - H.SNUG_TORQUE) * min(1.0, (self.turns - H.TURNS_SNUG)
                                                                             / (H.TURNS_TIGHT - H.TURNS_SNUG))
                self.driver_hands = tuple(s for s in ("left", "right") if not self.opening.get(s)
                                          and float(np.mean(robot[s]["fingers"])) > -0.040
                                          and float(np.linalg.norm(np.asarray(robot[s]["tcp"], float)[:3] - driver[:3])) < 0.08)
                if self.driver_hands:
                    torque = s_axis * load                              # resists the clockwise turn
        self.engaged, self.twist = engaged, tw
        self._wrench("driver", torque=torque)
        self.screw_drive.Set(float(-self.turns * 360.0))
        back = H.clamp_back_from_mouth(hose, clamp)
        return {"depth_mm": round(1000 * depth, 1), "max_depth_mm": round(1000 * self.max_depth, 1),
                "lined_up": bool(lined), "clamp_back_mm": round(1000 * back, 1), "screw_turns": round(self.turns, 2),
                "snug": bool(self.snug), "socket_on_screw": bool(engaged)}

    @staticmethod
    def clamp_on_shank(hose, clamp):
        return H.clamp_on_shank(H.clamp_back_from_mouth(hose, clamp))


def make_dynamics(env):
    return Dynamics(env)


def make_scorer():
    return TrialScorer()


# -- demo ----------------------------------------------------------------------------------

REST = {"left": (0.16, 0.24, H.TOP + 0.16), "right": (0.16, -0.24, H.TOP + 0.16)}
ACROSS_Y = simkit.rot_z(math.pi / 2) @ DOWN          # jaws closing along x, across a hose lying along y
HOSE_GRIP = 0.100                                    # grip this far back from the mouth, behind the clamp
HOSE_FINGER = -(2 * H.HOSE_RO - 0.010) / 2
DRIVER_FINGER = -(2 * H.DRIVER_HANDLE[0] - 0.018) / 2       # squeeze hard: it must not turn in the hand


def _tcp(tip, rot, label, seconds, finger):
    tip = np.asarray(tip, float)
    return Waypoint(seconds, tuple(map(float, tip - rot @ np.array([0.0, 0.0, PAD_TIP]))), finger, label, rot)


def _hose_hand(seconds, mouth_y, z, finger, label, lift=0.0):
    return _tcp((H.AXIS_X, mouth_y - HOSE_GRIP, z - 0.002 + lift), ACROSS_Y, label, seconds, finger)


def _driver_hand(seconds, tip_xyz, turn, finger, label):
    """Grip the driver's handle at its middle, given where its socket's bottom should be; ``turn`` (rad)
    turns the wrist about the vertical."""
    x, y, z = tip_xyz
    mid = z + H.SOCKET[1] + H.DRIVER_SHAFT[1] + H.DRIVER_HANDLE[1] / 2 - GRIP_LOW
    return _tcp((x, y, mid), simkit.rot_z(turn) @ DOWN, label, seconds, finger)


SEAT = -0.0085                                        # socket rim below the head's top when seated


def demo():
    lie_z = H.TOP + H.HOSE_RO
    m0 = H.HOSE_REST[1]
    home = H.BOSS_Y + 0.004
    f = HOSE_FINGER
    right = [
        Waypoint(1.5, REST["right"], FINGER_OPEN, "look", DOWN),
        _hose_hand(2.0, m0, lie_z, FINGER_OPEN, "over the hose, behind its clamp", lift=0.06),
        _hose_hand(1.2, m0, lie_z, FINGER_OPEN, "down round it"),
        _hose_hand(0.8, m0, lie_z, f, "take it"),
        _hose_hand(1.5, m0, H.AXIS_Z, f, "lift it to the outlet's height"),
        _hose_hand(1.5, H.TIP_Y - 0.008, H.AXIS_Z, f, "line it up with the barb"),
        _hose_hand(1.0, H.TIP_Y + 0.004, H.AXIS_Z, f, "onto the barb's tip"),
        _hose_hand(2.5, home, H.AXIS_Z, f, "push it on over the bead up to the boss"),
        _hose_hand(0.6, home, H.AXIS_Z, FINGER_OPEN, "let go"),
        _hose_hand(0.8, home, H.AXIS_Z, FINGER_OPEN, "lift off", lift=0.05),
    ]
    right.append(Waypoint(1.5, REST["right"], FINGER_OPEN, "stand clear", DOWN))
    hx = H.AXIS_X + H.housing_offset()[0]
    clamp_y1 = H.BOSS_Y - H.CLAMP_AT          # the band, with the mouth home at the boss
    t_right = sum(w.duration for w in right)
    # the left arm takes the nut driver and drives the screw, regripping between strokes
    tip0 = H.driver_tip_rest()
    sx, sy = hx, clamp_y1
    head_z = H.AXIS_Z + H.HOUSING[2] / 2 + H.SCREW_H
    g = DRIVER_FINGER
    a = math.radians(90.0)
    left = [
        Waypoint(2.0, REST["left"], FINGER_OPEN, "look", DOWN),
        _driver_hand(2.0, (tip0[0], tip0[1], tip0[2] + 0.05), 0.0, FINGER_OPEN, "over the nut driver"),
        _driver_hand(1.2, tip0, 0.0, FINGER_OPEN, "round its handle"),
        _driver_hand(0.8, tip0, 0.0, g, "take it"),
        _driver_hand(1.5, (tip0[0], tip0[1], tip0[2] + 0.055), 0.0, g, "lift it out of the holder"),
        _driver_hand(max(0.5, t_right - 7.5), (sx - 0.05, sy - 0.03, head_z + 0.035), a, g, "wait, holding it"),
        _driver_hand(1.5, (sx, sy, head_z + 0.030), a, g, "over the clamp's screw"),
        _driver_hand(1.5, (sx, sy, head_z + 0.030), a, g, "over the clamp's screw"),
        _driver_hand(1.5, (sx, sy, head_z + 0.004), a, g, "over the clamp's screw"),
        _driver_hand(1.5, (sx, sy, head_z + SEAT), a, g, "seat the socket on it"),
    ]
    z = head_z + SEAT
    for k in range(STROKES):
        left += [_driver_hand(1.0, (sx, sy, z), 0.0, g, "turn it clockwise"),
                 _driver_hand(1.0, (sx, sy, z), -a, g, "turn it clockwise"),
                 _driver_hand(0.4, (sx, sy, z), -a, g, "ease off: hold still where it is"),
                 _driver_hand(0.8, (sx, sy, z), -a, FINGER_OPEN, "let go of the handle"),
                 _driver_hand(0.4, (sx, sy, z + 0.010), -a, FINGER_OPEN, "lift the open hand a little"),
                 _driver_hand(0.5, (sx, sy, z + 0.010), 0.0, FINGER_OPEN, "turn the open hand back round it"),
                 _driver_hand(0.5, (sx, sy, z + 0.010), a, FINGER_OPEN, "turn the open hand back round it"),
                 _driver_hand(0.4, (sx, sy, z), a, FINGER_OPEN, "turn the open hand back round it"),
                 _driver_hand(0.5, (sx, sy, z), a, g, "grip it again")]
    left += [_driver_hand(1.0, (sx, sy, head_z + 0.012), a, g, "lift the driver off"),
             _driver_hand(2.0, (tip0[0], tip0[1], tip0[2] + 0.055), 0.0, g, "back over the holder"),
             _driver_hand(1.5, (tip0[0], tip0[1], tip0[2] + 0.002), 0.0, g, "into the holder"),
             _driver_hand(0.6, (tip0[0], tip0[1], tip0[2] + 0.002), 0.0, FINGER_OPEN, "let go"),
             Waypoint(1.5, REST["left"], FINGER_OPEN, "stand clear", DOWN),
             Waypoint(1.5, REST["left"], FINGER_OPEN, "done", DOWN)]
    right.append(Waypoint(sum(w.duration for w in left) - t_right + 0.5, REST["right"], FINGER_OPEN, "done", DOWN))
    return {"left": left, "right": right}


ALIGN_LABELS = ("over the clamp's screw", "seat the socket on it", "turn it clockwise", "let go of the handle",
                "lift the open hand a little", "turn the open hand back round it", "grip it again")
STROKES = 3
GRASP_LABELS = ("seat the socket on it", "grip it again")
GRIP_LOW = 0.010                                      # take the handle below its middle: the palm stays off its top
HOLD_LABELS = ("ease off: hold still where it is", "let go of the handle")


class DemoController:
    """The scripted plans, with the left hand's position nudged during the driving by the measured
    offset between the driver's socket and the screw head (a scripted demo may steer by sim state)."""

    def __init__(self, plans):
        self.plans = plans
        self.corr = np.zeros(2)
        self.hold = None
        self.off = None
        self.last_xy = None

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
                    self.hold = np.array(SHARED["left_tcp"], float)   # stay where the hand actually is: no push
                pos = self.hold.copy()
            elif side == "left":
                self.hold = None
            if side == "left" and label != "turn it clockwise":
                self.last_xy = None
            if side == "left" and label in GRASP_LABELS and SHARED.get("engaged"):
                # where the driver's axis sits in the hand, in the hand's own frame
                d = np.asarray(SHARED["tip"], float) - np.asarray(SHARED["left_tcp"], float)
                d[2] = 0.0
                self.off = rot.T @ d
            if side == "left" and label == "turn it clockwise" and self.off is not None and SHARED.get("engaged"):
                # turn the hand about the driver's axis, not about its own centre
                want = np.asarray(SHARED["head"], float)[:2] - (rot @ self.off)[:2] - self.corr
                prev = self.last_xy if self.last_xy is not None else pos[:2]
                pos[:2] = prev + np.clip(want - prev, -0.001, 0.001)     # ease the hand over, 1 mm a step at most
                self.last_xy = pos[:2].copy()
            if side == "left" and label in ALIGN_LABELS and label not in HOLD_LABELS and "socket_err" in SHARED:
                err = np.asarray(SHARED["socket_err"], float)
                if label in ("over the clamp's screw", "seat the socket on it") or \
                        (label == "turn it clockwise" and float(np.linalg.norm(err)) < 0.004):
                    self.corr = np.clip(self.corr + 0.05 * err, -0.02, 0.02)
                pos[:2] += self.corr
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), plan.q, base_pos=plan.base_pos, iters=60)
            plan.q = q
            grip = float(np.clip(finger / FO, 0.0, 1.0)) if FO != FC else 1.0
            out.append(np.r_[q, grip])
        return np.concatenate(out)


def make_demo_controller(env, plans):
    return DemoController(plans)
