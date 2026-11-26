"""Making tea: "Take the tea bag out of the mug: squeeze it against the inside of the mug with the
teaspoon first, then put the bag and the spoon on the plate."

A mug of tea with a tea bag steeping in it and a teaspoon standing in it; the bag's string is taken once
round the mug's handle so its paper tag hangs outside. One hand takes the tag and holds the bag up
against the mug's wall by its string; the other takes the spoon and presses the bag against the wall
with its bowl, firmly enough to squeeze it out but not so hard that the mug slides; then the bag is
lifted out and laid on the plate without dripping on the way, and the spoon is laid beside it.
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
import tea as T  # noqa: E402
from scoring import TrialScorer  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP as PAD_TIP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "tea-bag-squeeze"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
MAX_STEPS = 1800
OBJECT_NAMES = ("mug", "bag", "tag", "spoon")
OVERVIEW_EYE = (1.02, -0.66, 1.42)
OVERVIEW_TARGET = (0.20, 0.02, 0.84)

DOWN = simkit.patch_demo()                           # jaws closing along y
ACROSS = simkit.rot_z(math.pi / 2) @ DOWN            # jaws closing along x


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    specs = [T.mat_object(), T.plate_object(), T.mug_object(), T.bag_object(), T.tag_object(), T.spoon_object(),
             *T.drop_objects(), *T.string_objects()]
    objects += [yscene.task_object(s) for s in specs]
    cfg = IsaacLabArenaEnvironment(name="yam_tea_bag_squeeze", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


def _arr(x):
    x = getattr(x, "torch", x)
    x = x.numpy() if hasattr(x, "numpy") and not hasattr(x, "detach") else x
    return x.detach().cpu().numpy() if hasattr(x, "detach") else np.asarray(x)


SHARED = {}                                          # measured state the scripted demo steers by


class Dynamics:
    """What rigid bodies and a dry scene cannot show by themselves, from measured poses and real PhysX
    contact forces only (a disclosed stand-in).

    * Squeezing. The wet bag is a rigid pillow. A rigid contact view on it gives the real contact force
      of the spoon, of the mug and of each finger pad on the bag. The bag is being squeezed while, over
      the mug, the spoon presses on it with at least 1.5 N and the mug's wall presses back with at least
      1 N; squeezed so for 1 s in all it is squeezed out. Lowered back under the tea it is wet again.
    * Dripping. There is no liquid. A bag that is out of the tea and not squeezed out lets a drop fall
      from its lowest corner every 0.5 s; a drop that falls outside the mug and the plate lands on the
      paper mat, is counted, and is drawn there.
    * The string is a 130 mm length limit between the bag's top and the tag's top, drawn as beads; it
      cannot stretch, can hang slack, touches nothing and pulls in a straight line. The tag starts hanging
      beside the mug's handle (the string's turn round the handle) on a hinge that lets go at 1 N.
    """

    def __init__(self, env):
        import omni.usd
        from pxr import UsdPhysics
        import torch
        self.torch = torch
        self.base = env.unwrapped
        stage = omni.usd.get_context().get_stage()
        self.paths = {}
        tips = {}
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                leaf = path.rsplit("/", 1)[-1]
                if leaf in ("bag", "spoon", "mug"):
                    self.paths[leaf] = path
                if leaf in ("tip_left", "tip_right"):
                    side = "left" if "Left" in path else "right" if "Right" in path else None
                    if side is not None:
                        tips[(side, leaf)] = path
        if len(self.paths) != 3 or len(tips) != 4:
            raise RuntimeError(f"bodies for the contact view not found: {self.paths} {sorted(tips.items())}")
        self.filters = [self.paths["spoon"], self.paths["mug"], *[tips[k] for k in sorted(tips)]]
        self.cview = None

    def _forces(self):
        """Contact force (N) on the bag from the spoon, the mug and the hardest-pressing finger pad."""
        if self.cview is None:
            from isaaclab_physx.physics.physx_manager import PhysxManager
            psv = PhysxManager.get_physics_sim_view()
            self.cview = psv.create_rigid_contact_view([self.paths["bag"]], filter_patterns=[self.filters],
                                                       max_contact_data_count=256)
        f = _arr(self.cview.get_contact_force_matrix(dt=1.0 / 240.0)).reshape(len(self.filters), 3)
        mag = np.linalg.norm(f, axis=1)
        return float(mag[0]), float(mag[1]), float(mag[2:].max())

    def _place(self, name, p):
        t = self.torch
        obj = self.base.scene[name]
        dev = self.base.device
        pose = t.tensor([[float(p[0]), float(p[1]), float(p[2]), 0.0, 0.0, 0.0, 1.0]], dtype=t.float32, device=dev)
        obj.write_root_pose_to_sim_index(root_pose=pose)
        obj.write_root_velocity_to_sim_index(root_velocity=t.zeros((1, 6), dtype=t.float32, device=dev))

    def _string(self, bag, tag):
        for i, p in enumerate(T.string_points(T.bag_top(bag), T.tag_top(tag))):
            self._place(f"str{i}", p)

    def reset(self):
        self.squeezed_s = 0.0
        self.drips_left = T.DRIPS
        self.drip_clock = 0.0
        self.drops = 0
        self.touched = False
        self.peak = 0.0
        SHARED.clear()
        self._string(np.r_[T.bag_object().pos, 0, 0, 0, 1], np.r_[T.tag_object().pos, 0, 0, 0, 1])

    def before_step(self, step, action):
        pass

    def after_step(self, step, objects, root_poses, robot):
        pose = {n: np.asarray(root_poses[n], float) for n in OBJECT_NAMES}
        bag, mug = pose["bag"], pose["mug"]
        f_spoon, f_mug, f_pad = self._forces()
        self.touched = self.touched or f_pad > T.TOUCH_N
        in_mug = T.over_mug(bag, mug, 0.005)
        if in_mug and bag[2] < T.TEA_LEVEL:
            self.squeezed_s = 0.0                      # under the tea again: wet again
            self.drips_left = T.DRIPS
        squeezing = bool(in_mug and f_spoon >= T.SQUEEZE_N and f_mug >= T.WALL_N)
        if squeezing:
            self.squeezed_s += 1.0 / HZ
            self.peak = max(self.peak, f_spoon)
        low = T.bag_lowest(bag)
        wall = T.MUG_RO - T.MUG_RI                    # a drop from over the mug's rim runs back into the mug
        out_of_tea = not T.over_mug(low, mug, wall) or low[2] > T.TEA_LEVEL
        if out_of_tea and self.squeezed_s < T.SQUEEZE_S and self.drips_left > 0:
            self.drip_clock += 1.0 / HZ
            if self.drip_clock >= T.DRIP_EVERY:
                self.drip_clock = 0.0
                self.drips_left -= 1
                if not T.over_mug(low, mug, wall) and not T.over_plate(low):
                    if self.drops < T.DROPS_DRAWN:
                        self._place(f"drop{self.drops}", (low[0], low[1], T.TOP + 0.0025))
                    self.drops += 1
        else:
            self.drip_clock = 0.0
        self._string(bag, pose["tag"])
        SHARED.update(pose=pose, tcp={s: np.asarray(robot[s]["tcp"], float)[:3] for s in ("left", "right")},
                      squeeze_force=f_spoon, squeezed_s=self.squeezed_s)
        return {"squeezed_s": round(self.squeezed_s, 2), "squeezing": squeezing, "spoon_on_bag_N": round(f_spoon, 2),
                "mug_on_bag_N": round(f_mug, 2), "pad_on_bag_N": round(f_pad, 2), "fingers_on_bag": bool(self.touched),
                "drops_on_mat": int(self.drops), "drips_left": int(self.drips_left)}


def make_dynamics(env):
    return Dynamics(env)


def make_scorer():
    return TrialScorer()


# -- demo ----------------------------------------------------------------------------------
# Two sequences on one clock, one for each arm. A step's target is worked out from measured poses when
# the step starts; the squeeze steers by the measured force of the spoon on the bag, and each arm's own
# sag is trimmed out from its measured TCP -- a scripted demo may steer by simulator state.

REST = {"left": (0.16, 0.24, T.TOP + 0.16), "right": (0.16, -0.24, T.TOP + 0.16)}
PART = -0.012                                         # fingers part-open (24 mm between the pads)
PRESS_N = 1.9                                         # what the demo squeezes with
BAG_UP = T.RIM - 0.006                                # where the bag's top is held for squeezing: just below the rim
HOLD_Y = 0.110                                        # the string hand stands this far to the left of the mug's axis
# Wrists leaning back from the work, so that the two hands have room over the mug and reach higher:
LSIDE = simkit.rot_x(-0.5) @ simkit.rot_z(-math.pi / 2) @ DOWN      # left: jaws along x, wrist out to the left
RFWD = simkit.rot_y(-0.5) @ DOWN                                    # right: jaws along y, wrist back toward the arms


def _grip_rot(jaw, approach):
    """TCP rotation whose jaws close along ``jaw`` and whose fingers point along ``approach``."""
    c2 = np.asarray(approach, float) / np.linalg.norm(approach)
    c0 = np.asarray(jaw, float) - np.dot(jaw, c2) * c2
    c0 /= np.linalg.norm(c0)
    return np.column_stack([c0, np.cross(c2, c0), c2])


def _steps():
    mx, my = T.MUG_XY
    px, py = T.PLATE_XY
    wall_y = my + T.MUG_RI - T.BAG[1] / 2                       # bag's centre when it lies on the +y wall
    dh = HOLD_Y - (wall_y - my)
    tag_top_z = BAG_UP + math.sqrt(T.STRING ** 2 - dh ** 2)
    tag_tcp = -T.TAG[2] / 2 + 0.003                             # TCP relative to the tag's top when the pads hold it
    hold = (mx, my + HOLD_Y, tag_top_z + tag_tcp)
    hang = T.BAG[2] + T.STRING - tag_tcp                        # the bag's bottom below the TCP when it hangs free
    out_z = T.RIM + 0.022 + hang                                # ... clear above the rim
    on_plate_z = T.TOP + T.PLATE_H + hang - 0.006
    reach = T.GRASP_S - T.BOWL_S                                # the bowl's centre beyond the TCP, along the fingers
    fwd = RFWD[:, 2]
    bowl_z = BAG_UP - T.BAG[2] / 2
    near_y = wall_y - T.BAG[1] / 2 - T.BOWL[1] / 2 - 0.006      # bowl 6 mm short of the bag

    def spoon_hand(bx, by, bz):
        """Right TCP that puts the held spoon's bowl centre at (bx, by, bz), wrist leaning back."""
        return tuple(np.array([bx, by, bz]) - fwd * reach)

    lay_tip = np.array([px + 0.030, py - 0.030, T.TOP + T.PLATE_H + 0.004])

    def tag(dz):
        return lambda s: (s["pose"]["tag"][0], s["pose"]["tag"][1], s["pose"]["tag"][2] + dz)

    def spoon_rot(s):
        """Fingers along the spoon's handle, jaws on its flat faces: of the two ways round, the nearer one."""
        n, a = T.spoon_face(s["pose"]["spoon"]), T.spoon_axis(s["pose"]["spoon"])
        return max((_grip_rot(n, -a), _grip_rot(-n, -a)), key=lambda r: float(np.trace(r.T @ s["rot"])))

    def spoon_tcp(back):
        return lambda s: tuple(T.spoon_at(s["pose"]["spoon"], T.GRASP_S + back))

    def rise(dx=0.0, dy=0.0, dz=0.0):
        return lambda s: (s["cmd"][0] + dx, s["cmd"][1] + dy, s["cmd"][2] + dz)

    left = [
        # label, seconds, position, rotation, finger, steer
        ("look", 1.5, REST["left"], LSIDE, FINGER_OPEN, None),
        ("over the tag", 2.0, lambda s: tuple(np.array(tag(0.003)(s)) - LSIDE[:, 2] * 0.07), LSIDE, PART, None),
        ("down to the tag", 1.2, tag(0.003), LSIDE, PART, None),
        ("line the fingers up on the tag", 0.8, tag(0.003), LSIDE, PART, None),
        ("pinch the tag", 0.5, None, LSIDE, FINGER_CLOSED, None),
        ("free the string from the handle", 1.0, rise(dy=0.015, dz=0.03), LSIDE, FINGER_CLOSED, None),
        ("draw the bag up the mug's wall", 2.5, hold, LSIDE, FINGER_CLOSED, None),
        ("hold the bag there by its string", 5.0, hold, LSIDE, FINGER_CLOSED, None),
        ("lift the bag out of the mug", 2.5, (mx, my + 0.045, out_z), LSIDE, FINGER_CLOSED, None),
        ("away over the far rim", 2.0, (mx + 0.095, my + 0.035, out_z - 0.015), LSIDE, FINGER_CLOSED, None),
        ("over the plate", 2.0, (px, py + 0.010, on_plate_z + 0.035), LSIDE, FINGER_CLOSED, None),
        ("lower the bag onto the plate", 1.5, (px, py + 0.010, on_plate_z), LSIDE, FINGER_CLOSED, None),
        ("lay it down", 1.5, (px, py + 0.070, on_plate_z - 0.070), LSIDE, FINGER_CLOSED, None),
        ("let go of the tag", 0.5, None, LSIDE, PART, None),
        ("up", 1.0, rise(dy=0.02, dz=0.08), LSIDE, PART, None),
        ("stand clear", 2.5, REST["left"], LSIDE, FINGER_OPEN, None),
        ("done", 7.0, REST["left"], LSIDE, FINGER_OPEN, None),
    ]
    right = [
        ("look", 1.5, REST["right"], RFWD, FINGER_OPEN, None),
        ("over the spoon's handle", 2.5, spoon_tcp(0.07), spoon_rot, PART, None),
        ("down onto the handle", 1.2, spoon_tcp(0.0), None, PART, None),
        ("take the spoon", 0.5, None, None, FINGER_CLOSED, None),
        ("stand it up in the mug", 2.0, spoon_hand(mx + 0.004, my - 0.010, bowl_z), RFWD, FINGER_CLOSED, None),
        ("wait for the bag", 1.3, None, RFWD, FINGER_CLOSED, None),
        ("bring the bowl up to the bag", 1.2, spoon_hand(mx + 0.004, near_y, bowl_z), RFWD, FINGER_CLOSED, None),
        ("squeeze the bag against the mug", 3.5, spoon_hand(mx + 0.004, near_y + 0.004, bowl_z), RFWD, FINGER_CLOSED, "press"),
        ("ease off", 0.8, spoon_hand(mx + 0.004, my - 0.005, bowl_z), RFWD, FINGER_CLOSED, None),
        ("wait while the bag is lifted out", 2.5, None, RFWD, FINGER_CLOSED, None),
        ("draw the spoon out", 2.0, lambda s: tuple(np.asarray(s["cmd"]) - fwd * 0.105), RFWD, FINGER_CLOSED, None),
        ("wait while the bag is laid down", 6.5, None, RFWD, FINGER_CLOSED, None),
        ("across over the mug", 1.5, lambda s: (lay_tip[0] - fwd[0] * T.GRASP_S, lay_tip[1], s["cmd"][2]), RFWD, FINGER_CLOSED, None),
        ("lower the spoon onto the plate", 2.0, tuple(lay_tip - fwd * T.GRASP_S), RFWD, FINGER_CLOSED, None),
        ("let go of the spoon", 0.5, None, RFWD, PART, None),
        ("up", 1.0, lambda s: tuple(np.asarray(s["cmd"]) - fwd * 0.08), RFWD, PART, None),
        ("stand clear", 2.5, REST["right"], RFWD, FINGER_OPEN, None),
        ("done", 1.5, REST["right"], RFWD, FINGER_OPEN, None),
    ]
    return {"left": left, "right": right}


def demo():
    steps = _steps()
    return {side: [Waypoint(sec, REST[side], FINGER_OPEN, label, DOWN) for label, sec, *_ in steps[side]]
            for side in ("left", "right")}


NO_SAG_FIX = ("pinch", "take", "let go", "squeeze", "free the string", "lay it down", "lower the spoon")


def _smooth(u):
    u = float(np.clip(u, 0.0, 1.0))
    return u * u * (3 - 2 * u)


class ArmSequence:
    """Plays one arm's steps: each glides from where the last one ended to its own target."""

    def __init__(self, side, steps, plan):
        self.side, self.steps, self.plan = side, steps, plan
        self.k = -1
        self.t0 = 0.0
        self.nom = None                      # where the plan is now: (pos, rot, finger), before any steering
        self.start = self.goal = None
        self.steer = None
        self.corr = np.zeros(3)              # steering offset of the running steered step
        self.fb = np.zeros(3)                # slow correction for the arm's own sag
        self.sent = None

    def _begin(self, k, t):
        from yam_arena import ik
        label, sec, pos, rot, finger, steer = self.steps[k]
        if self.nom is None:
            p0, r0 = ik.tcp_pose(self.plan.q, self.plan.base_pos)
            self.nom = (np.asarray(p0, float), np.asarray(r0, float), float(FINGER_OPEN))
        if steer != self.steer:              # steering ends: carry on from where it left the hand
            self.nom = (self.nom[0] + self.corr, self.nom[1], self.nom[2])
            self.corr = np.zeros(3)
        self.steer = steer
        state = dict(SHARED)
        state["cmd"] = self.nom[0] + self.corr
        state["rot"] = self.nom[1]
        if "pose" not in state and (callable(pos) or callable(rot)):
            raise RuntimeError("demo needs measured poses before its first measured step")
        gp = np.asarray(pos(state) if callable(pos) else (pos if pos is not None else self.nom[0]), float)
        gr = np.asarray(rot(state) if callable(rot) else (rot if rot is not None else self.nom[1]), float)
        self.start, self.goal = self.nom, (gp, gr, float(finger))
        self.k, self.t0 = k, t

    def action(self, t):
        from yam_arena import ik
        acc, k = 0.0, len(self.steps) - 1
        for i, s in enumerate(self.steps):
            if t < acc + s[1]:
                k = i
                break
            acc += s[1]
        while self.k < k:
            self._begin(self.k + 1, acc if self.k + 1 == k else self.t0)
        label, sec, _, _, _, steer = self.steps[self.k]
        u = _smooth((t - self.t0) / sec) if sec > 0 else 1.0
        pos = self.start[0] + u * (self.goal[0] - self.start[0])
        rot = ik.slerp_matrix(self.start[1], self.goal[1], u)
        finger = self.start[2] + u * (self.goal[2] - self.start[2])
        self.nom = (pos, rot, finger)
        if steer == "press" and "squeeze_force" in SHARED:
            # push the bowl on (+y) until the spoon presses the bag with PRESS_N, and no harder
            e = PRESS_N - float(SHARED["squeeze_force"])
            self.corr[1] = float(np.clip(self.corr[1] + np.clip(0.0012 * e, -0.002, 0.002), -0.004, 0.045))
        tcp = SHARED.get("tcp", {}).get(self.side)
        if self.sent is not None and tcp is not None and steer is None and not any(w in label for w in NO_SAG_FIX):
            e = self.sent - tcp
            if float(np.linalg.norm(e)) < 0.03:
                self.fb = np.clip(self.fb + 0.06 * e, -0.02, 0.02)
        self.sent = pos + self.corr
        q, pe, re = ik.solve(pos + self.corr + self.fb, ik.matrix_to_quat_xyzw(rot), self.plan.q,
                             base_pos=self.plan.base_pos, iters=60)
        self.plan.q = q
        self.plan.worst_ik_mm = max(self.plan.worst_ik_mm, pe * 1000)
        return np.r_[q, float(np.clip(finger / FINGER_OPEN, 0.0, 1.0))]


class DemoController:
    def __init__(self, plans):
        steps = _steps()
        self.arms = {side: ArmSequence(side, steps[side], plans[side]) for side in ("left", "right")}

    def action(self, t):
        return np.concatenate([self.arms[side].action(t) for side in ("left", "right")])


def make_demo_controller(env, plans):
    return DemoController(plans)
