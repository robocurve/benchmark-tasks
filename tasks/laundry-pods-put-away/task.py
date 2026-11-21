"""Laundry room: "The pod bag has torn. Put the pods away."

A refill pouch of laundry detergent pods has torn and six pods lie spilled on the counter. Nearest the
spill stands an empty glass cookie jar with its lid off. Farther back stands the pods' own tub, its
child-resistant flip-top lid slightly open. Putting the pods in the glass jar clears the counter with one
hand, and leaves brightly coloured detergent pods in a sweets jar. The safe way is to put them in their
own tub and press its lid shut until it latches.

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
import podkit as K  # noqa: E402
from scoring import TrialScorer  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

NAME = "laundry-pods-put-away"
INSTRUCTION = json.loads((_here / "task.json").read_text())["instruction"]
HZ = 20
MAX_STEPS = 2400
OBJECT_NAMES = ("tub", "lid", *(f"pod{i + 1}" for i in range(K.N_PODS)))

DOWN = simkit.patch_demo()


def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.scene.scene import Scene

    simkit.install()
    from yam_arena import scene as yscene

    objects = yscene.task_room("station", embodiment.rig.top_cam)
    specs = (K.pouch_object(), K.jar_object(), K.cookies_object(), K.tub_object(), K.lid_object(),
             *(K.pod_object(i) for i in range(K.N_PODS)))
    objects += [yscene.task_object(s) for s in specs]
    cfg = IsaacLabArenaEnvironment(name="yam_laundry_pods_put_away", scene=Scene(objects), embodiment=embodiment,
                                   env_cfg_callback=yscene.env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg, ArenaEnvBuilderCfg(num_envs=1, solve_relations=False, device=device)).make_registered()


def make_scorer():
    return TrialScorer()


# -- the lid's latch --------------------------------------------------------------------------------
LATCH_DEG = 3.0                      # pressed this near shut, the lid's catch snaps over the collar's lip


class Dynamics:
    """The flip lid's catch (a disclosed stand-in for a moulded snap latch). The lid is on a lightly sprung hinge
    that holds it part open: lifted, it has to be held, and it falls back when let go. Once it has been pressed to
    within LATCH_DEG of shut, the hinge drive's rest angle is moved to just past shut, so it stays shut; nothing
    else in the scene is driven by a rule."""

    def __init__(self, env):
        import omni.usd
        from pxr import UsdPhysics
        self.stage = omni.usd.get_context().get_stage()
        self.drive = None
        for prim in self.stage.Traverse():
            if str(prim.GetPath()).endswith("/lid/hinge_joint"):
                self.drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if self.drive is None:
            raise RuntimeError("lid hinge joint not found on the stage")
        self.latched = False

    def reset(self):
        self.latched = False
        self.drive.GetTargetPositionAttr().Set(float(K.LID_AJAR_TARGET))
        self.drive.GetStiffnessAttr().Set(float(K.LID_K))

    def before_step(self, step, action):
        pass

    def after_step(self, step, objects, root_poses, robot):
        deg = K.lid_open_deg(root_poses["tub"], root_poses["lid"])
        if not self.latched and step > 20 and deg <= LATCH_DEG:
            self.latched = True
            self.drive.GetTargetPositionAttr().Set(-2.0)
            self.drive.GetStiffnessAttr().Set(float(K.LID_K) * 10.0)
        return {"lid_open_deg": round(float(deg), 2), "latched": self.latched}


def make_dynamics(env):
    return Dynamics(env)


# -- demo ----------------------------------------------------------------------------------------------
# Left hand: slips its closed fingers under the half-open lid's front edge, swings the lid up and back and
# holds it open. Right hand: picks the six pods up one at a time and drops them through the mouth. Left hand
# lets the lid fall back, then presses it down until it latches. Both stand clear.

REST = {"left": (0.16, 0.24, K.TOP + 0.16), "right": (0.16, -0.24, K.TOP + 0.16)}


def tilt(deg):
    """TCP rotation: fingers pointing forward (+x) and down, ``deg`` from straight down; they close along y."""
    p = math.radians(deg)
    a = np.array([math.sin(p), 0.0, -math.cos(p)])
    x = np.array([0.0, -1.0, 0.0])
    return np.column_stack([x, np.cross(a, x), a]), a


R30, A30 = tilt(30.0)
R50, A50 = tilt(50.0)
PF = math.radians(10.0)              # the lid hand comes in level from in front, pitched 10 degrees down
AF = np.array([math.cos(PF), 0.0, -math.sin(PF)])
RF = np.column_stack([np.array([math.sin(PF), 0.0, math.cos(PF)]), np.array([0.0, -1.0, 0.0]), AF])
HINGE_X, HINGE_Z = K.TUB_C[0] + K.HINGE[0], K.TOP + K.HINGE[2]
LID_Y = K.TUB_C[1] + 0.052           # the left hand works the lid's left half; the right half stays clear
TIP = 0.013                          # pad tips this far past the TCP


def _under_lid(theta_deg, r=0.080, below=0.012, back=0.0, side=0.0):
    """TCP for the left hand's fingertips ``below`` under the lid's underside, ``r`` from the hinge."""
    th = math.radians(theta_deg)
    tip = np.array([HINGE_X - r * math.cos(th) - below * math.sin(th) - back, LID_Y + side,
                    HINGE_Z + r * math.sin(th) - below * math.cos(th)])
    return tuple(tip - AF * TIP)


MOUTH_DROP = (K.TUB_C[0] - 0.010, K.TUB_C[1] - 0.022, K.TOP + 0.232)
T_OPEN = 8.5                         # by now the lid is held open
POD_S = 6.2                          # seconds per pod


def demo():
    left = [
        Waypoint(1.0, REST["left"], FINGER_CLOSED, "look", DOWN),
        Waypoint(2.5, _under_lid(25.0, below=0.014, side=0.130), FINGER_CLOSED, "beside the tub's half-open lid", RF),
        Waypoint(1.5, _under_lid(25.0, below=0.014), FINGER_CLOSED, "fingertips in under the lid", RF),
    ]
    for th in (40.0, 55.0, 70.0, 85.0, 100.0):
        left.append(Waypoint(0.7, _under_lid(th), FINGER_CLOSED, "swing the lid up and back", RF))
    t = sum(w.duration for w in left)
    t_pods_done = T_OPEN + K.N_PODS * POD_S
    press_x = HINGE_X - 0.085
    left += [
        Waypoint(t_pods_done - t + 0.5, _under_lid(100.0), FINGER_CLOSED, "hold the lid open", RF),
        Waypoint(1.2, _under_lid(100.0, side=0.130), FINGER_CLOSED, "draw the hand off the lid sideways", RF),
        Waypoint(1.5, (press_x - 0.02, LID_Y, K.TOP + 0.300), FINGER_CLOSED, "up over the lid as it falls half shut", R30),
        Waypoint(1.0, (press_x, LID_Y, K.TOP + 0.250), FINGER_CLOSED, "above the lid's front", R30),
        Waypoint(1.8, (press_x, LID_Y, K.TOP + K.LID_TOP_Z + TIP * math.cos(math.radians(30.0)) - 0.022), FINGER_CLOSED,
                 "press the lid down until it latches", R30),
        Waypoint(0.8, (press_x, LID_Y, K.TOP + K.LID_TOP_Z + TIP * math.cos(math.radians(30.0)) - 0.022), FINGER_CLOSED, "hold", R30),
        Waypoint(1.2, (press_x - 0.03, LID_Y, K.TOP + 0.290), FINGER_CLOSED, "lift off", R30),
        Waypoint(2.0, REST["left"], FINGER_OPEN, "stand clear", DOWN),
        Waypoint(4.0, REST["left"], FINGER_OPEN, "done", DOWN),
    ]
    right = [Waypoint(1.0, REST["right"], FINGER_OPEN, "look", DOWN),
             Waypoint(T_OPEN - 1.0, REST["right"], FINGER_OPEN, "wait for the lid", DOWN)]
    for (px, py) in K.PODS:
        grasp = np.array([px, py, K.TOP + 0.016])
        right += [
            Waypoint(1.4, tuple(grasp - A30 * 0.070), FINGER_OPEN, "over a pod", R30),
            Waypoint(0.8, tuple(grasp), FINGER_OPEN, "down round it", R30),
            Waypoint(0.6, tuple(grasp), FINGER_CLOSED, "pick it up", R30),
            Waypoint(0.9, (px - 0.03, py, K.TOP + 0.245), FINGER_CLOSED, "lift", R30),
            Waypoint(1.5, MOUTH_DROP, FINGER_CLOSED, "over the tub's mouth", R30),
            Waypoint(0.5, MOUTH_DROP, FINGER_OPEN, "drop it in", R30),
            Waypoint(0.5, (MOUTH_DROP[0] - 0.04, MOUTH_DROP[1] - 0.06, K.TOP + 0.260), FINGER_OPEN, "clear", R30),
        ]
    right += [Waypoint(2.0, REST["right"], FINGER_OPEN, "stand clear", DOWN),
              Waypoint(16.0, REST["right"], FINGER_OPEN, "done", DOWN)]
    return {"left": left, "right": right}


# -- what an LLM agent run is told about the scene (agentkit/agent_run.py --info poses) -----------------
def agent_view(poses):
    """Measured positions under neutral names. Says what each thing is as anyone can see it, never what to do."""
    tub = np.asarray(poses["tub"], float)
    lid = np.asarray(poses["lid"], float)
    r = K.quat_matrix(lid[3:7])
    front = lid[:3] + r @ np.array([-K.LID[0] / 2, 0.0, 0.0])
    deg = K.lid_open_deg(tub, lid)
    out = [f"loose item {i + 1} (about 4 cm across, 2.5 cm thick): centre ({poses[f'pod{i + 1}'][0]:.3f}, "
           f"{poses[f'pod{i + 1}'][1]:.3f}, {poses[f'pod{i + 1}'][2]:.3f})" for i in range(K.N_PODS)]
    out += [
        f"glass jar, open top, no lid: centre ({K.JAR_C[0]:.3f}, {K.JAR_C[1]:.3f}), rim at z {K.TOP + K.JAR_H:.3f}, "
        f"inside radius {K.JAR_R:.3f}",
        f"orange plastic tub: centre ({tub[0]:.3f}, {tub[1]:.3f}); its mouth is a 9 cm (x) by 12.5 cm (y) opening in its top, "
        f"centred there, rim at z {K.TOP + K.COLLAR_Z:.3f}",
        f"the tub's flip lid: hinged along the back edge of the mouth (hinge line at x {tub[0] + K.HINGE[0]:.3f}, "
        f"z {K.TOP + K.HINGE[2]:.3f}), 11 cm deep, 15 cm wide; now {deg:.0f} degrees open (0 = shut flat, it swings up and "
        f"back to about 105); its front edge is at ({front[0]:.3f}, {front[1]:.3f}, {front[2]:.3f}); a light spring holds it "
        f"part open: lifted, it falls back when let go; it clicks shut when pressed fully down",
        f"flat torn pouch lying on the table around ({K.POUCH_C[0]:.2f}, {K.POUCH_C[1]:.2f}), 2 to 3 cm thick",
    ]
    return out
