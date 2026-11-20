"""On the station table, on a grey rubber mat: a small golden barrel cactus, a blue foam collar round its body, in a 9 cm
black thin-wall nursery pot in front of the right arm, and an empty black 4 in. pot with a firmed base layer of cactus
mix in front of the left arm.

The task: take the cactus out of the small pot, touching it only through its collar, without pulling it out of a pot
that still grips its root ball, and set it upright and centred on the base layer of the bigger pot, the top of its
ball 1 to 2 cm below the rim; then let go and move clear.

The old pot's wall flexes: four rigid wall panels, each a quarter of the wall, turn in about a line round the pot's
base against springs set so that a 20 N press (X) 4.5 cm up moves the wall in 3 mm there. The root ball is held in the
pot by a bond standing in for the roots matted against the wall. It lets go once two opposite panels have each been
pressed in 3 mm or more, 4.5 cm up, for 1 s running: the same 3 mm each side that defines X on the real pot. Without
that squeeze, the bond holds until a pull of Y = 15 N, and a ball that rises 1 cm in its pot then is the "pulled out by
the plant" zero. The cactus is one rigid body with its collar, joined to its ball by a bond that tears
at 40 N or 0.8 N m.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # lets this file import helpers beside it

import bodykit as bk  # noqa: E402
import cactus_rules as rules  # noqa: E402
import cactusassets as ca  # noqa: E402
import steptime  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MAT_T = 0.003
FLOOR = TOP + MAT_T
OLD_XY = (0.35, -0.06)  # where the right arm reaches the pot with a level grip
NEW_XY = (0.35, 0.13)

PULL_N = 15.0             # Y: without the squeeze, the ball's bond in the pot lets go at this pull
PULL_TORQUE = None        # the bound ball's grip is limited by pull only, as the real push-out test measures
STEM_BOND = (40.0, 0.8)   # the cactus on its ball
PANEL_DAMP = 0.02         # N m s/rad on each panel's hinge

SOLVER_IT = 32  # the pots' and the plant's solver iterations
MAT = Box("mat", pos=(0.35, 0.03, TOP + MAT_T / 2), size=(0.30, 0.56, MAT_T), color=(0.33, 0.34, 0.35))
NEW_POT = bk.Body.build("new_pot", (*NEW_XY, FLOOR), ca.NEW_MASS, ca.new_pot_shapes(), friction=(0.6, 0.5),
                        solver_iterations=SOLVER_IT)
OLD_POT = bk.Body.build("old_pot", (*OLD_XY, FLOOR), ca.OLD_BASE_MASS, ca.old_base_shapes(), friction=(0.6, 0.6),
                        solver_iterations=SOLVER_IT)
PANELS = []
PANEL_NAMES = [f"wall{i}" for i in range(4)]
INWARD = (-1.0, 1.0, 1.0, -1.0)  # the sign of each panel's hinge angle when it turns in
for i, name in enumerate(PANEL_NAMES):
    PANELS.append(bk.Body.build(name, (*OLD_XY, FLOOR), ca.PANEL_MASS, ca.old_panel_shapes(i), friction=(0.6, 0.6),
                                solver_iterations=SOLVER_IT, contact_report=True, inertia=ca.PANEL_I,
                                ignore=("old_pot", *PANEL_NAMES[:i])))
    a = math.radians(ca.PANEL_ANGLES[i])
    r = ca.old_inner_r(ca.HINGE_Z) + ca.OLD_WALL / 2
    # the wall flexes about its base: a springy joint there, its slides locked, its turns sprung back
    bk.spring(f"{name}_flex", "old_pot", name, (OLD_XY[0] + r * math.cos(a), OLD_XY[1] + r * math.sin(a), FLOOR + ca.HINGE_Z),
              (math.cos(a), math.sin(a), 0.0), ca.PANEL_K * ca.PANEL_SPRING_GAIN[i], PANEL_DAMP, bend_limit_deg=ca.PANEL_LIMIT_DEG)

BALL_POS = (*OLD_XY, FLOOR + ca.BALL_Z0)
BALL = bk.Body.build("ball", BALL_POS, ca.BALL_MASS, ca.ball_shapes(), friction=(0.6, 0.6), solver_iterations=SOLVER_IT,
                     contact_report=True)
CACTUS_POS = (*OLD_XY, BALL_POS[2] + ca.BALL_H)
CACTUS = bk.Body.build("cactus", CACTUS_POS, ca.CACTUS_MASS + ca.COLLAR_MASS, ca.cactus_shapes(), friction=(0.6, 0.5),
                       solver_iterations=SOLVER_IT, contact_report=True, ignore=("ball",))

bk.bond("root_bind", "old_pot", "ball", (*OLD_XY, BALL_POS[2] + ca.BALL_H / 2), PULL_N, PULL_TORQUE, collide=True)
bk.bond("stem", "ball", "cactus", CACTUS_POS, *STEM_BOND)

OBJECTS = [MAT, NEW_POT, OLD_POT, *PANELS, BALL, CACTUS]
CACTUS_IN_BALL = np.array(CACTUS_POS) - np.array(BALL_POS)


# -- the trial tracker --------------------------------------------------------------------------------------------------

def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy(), dtype=np.float64)


class Tracker:
    """Runs after every control step (and reads the cactus's support after every physics step); latches the zeros."""

    def __init__(self):
        self.env = None
        self.views = None
        self.sub = None
        self.reset()

    def reset(self) -> None:
        self.trial = rules.Trial()
        self.fall_from = None
        self.max_fall = 0.0
        self.report: dict = {}
        self.end: dict = {}
        self.steps = 0
        self.noted: set = set()
        self._bind_noted = False

    # -- simulator access
    def attach(self, env) -> None:
        self.env = env
        if self.views is not None:
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        base = env.unwrapped
        stage = get_current_stage()
        self.origin = _np(base.scene.env_origins)[0]
        links, tips, all_tips, right_links = [], [], [], []
        for side in ("left", "right"):
            paths = [q for q in base.scene[f"{side}_robot"].root_view.link_paths[0]
                     if stage.GetPrimAtPath(q).HasAPI(UsdPhysics.RigidBodyAPI)]
            links += paths
            fingers = [[q for q in paths if q.endswith("/" + n)][0] for n in ("tip_left", "tip_right")]
            all_tips += fingers
            if side == "right":
                tips = fingers
                right_links = paths
        self.n_links = len(links)
        sim_view = base.scene["ball"]._physics_sim_view
        path = lambda n: base.scene[n].root_view.prim_paths[0]  # noqa: E731
        self.views = {
            "cactus": sim_view.create_rigid_contact_view(path("cactus"), filter_patterns=links, max_contact_data_count=256),
            "ball": sim_view.create_rigid_contact_view(path("ball"), filter_patterns=links, max_contact_data_count=0),
            "pads": sim_view.create_rigid_contact_view(path("ball"), filter_patterns=all_tips, max_contact_data_count=32),
        }
        for n in PANEL_NAMES:  # the right arm's press on each wall panel, for the record
            self.views[n] = sim_view.create_rigid_contact_view(path(n), filter_patterns=right_links,
                                                               max_contact_data_count=0)
        self.bind_attr = UsdPhysics.Joint(stage.GetPrimAtPath(bk._JOINT_PATHS["root_bind"])).GetJointEnabledAttr()
        import omni.physx

        self.sub = omni.physx.get_physx_interface().subscribe_physics_on_step_events(self._physics_step, pre_step=False,
                                                                                      order=100)

    def pose(self, name):
        p = _np(self.env.unwrapped.scene[name].data.root_link_pose_w)[0]
        return p[:3] - self.origin, p[3:7]

    def _physics_step(self, dt) -> None:
        """The cactus and ball's free fall: from where they last lost all support to how far they have dropped since."""
        if self.env is None or self.views is None:
            return
        try:
            net = sum(float(np.linalg.norm(_np(self.views[n].get_net_contact_forces(dt))[0])) for n in ("ball", "cactus"))
            z = float(_np(self.env.unwrapped.scene["ball"].root_view.get_transforms())[0][2])
        except Exception:  # noqa: BLE001  (views not ready during a reset)
            return
        if net > 0.05:
            self.fall_from = None
            return
        if self.fall_from is None:
            self.fall_from = z
        self.max_fall = max(self.max_fall, self.fall_from - z)

    def _robot_touches(self, name: str) -> bool:
        mat = _np(self.views[name].get_contact_force_matrix(1.0 / 240.0))[0]
        return bool((np.linalg.norm(mat, axis=1) > 1e-4).any())

    def _off_collar_touch(self, cp, cq) -> bool:
        """Any robot contact on the cactus outside its collar."""
        forces, points, _, _, counts, starts = (_np(x) for x in self.views["cactus"].get_contact_data(1.0 / 240.0))
        rc = rules.rot(cq)
        for k in range(self.n_links):
            n, s = int(counts[0, k]), int(starts[0, k])
            for i in range(s, min(s + n, len(forces))):
                if abs(float(np.ravel(forces[i])[0])) > 1e-4:
                    if not rules.collar_point(rc.T @ (points[i] - self.origin - cp)):
                        return True
        return False

    def _pad_on_ball(self, bp, bq, opp, opq) -> bool:
        """A fingertip touches the root ball where it counts (cactus_rules.pad_on_ball_point)."""
        forces, points, _, _, counts, starts = (_np(x) for x in self.views["pads"].get_contact_data(1.0 / 240.0))
        rb, rp = rules.rot(bq), rules.rot(opq)
        for k in range(4):
            n, s = int(counts[0, k]), int(starts[0, k])
            for i in range(s, min(s + n, len(forces))):
                if abs(float(np.ravel(forces[i])[0])) > 1e-4:
                    w = points[i] - self.origin
                    if rules.pad_on_ball_point(rb.T @ (w - bp), rp.T @ (w - opp)):
                        return True
        return False

    def tcp(self, side: str, rot: bool = False):
        from yam_arena import ik, station
        from yam_arena.constants import ARM_JOINTS

        robot = self.env.unwrapped.scene[f"{side}_robot"]
        ids = robot.find_joints(list(ARM_JOINTS), preserve_order=True)[0]
        pose = ik.tcp_pose(_np(robot.data.joint_pos)[0, ids], station.BASES[side])
        return pose if rot else pose[0]

    def fingertips(self, side: str, tcp, rot) -> list:
        """Four points per gripper: each pad's middle (at the TCP's height along the fingers) and its tip, 13 mm on."""
        from yam_arena.constants import FINGER_JOINTS

        robot = self.env.unwrapped.scene[f"{side}_robot"]
        ids = robot.find_joints(list(FINGER_JOINTS), preserve_order=True)[0]
        half = -float(np.mean(_np(robot.data.joint_pos)[0, ids]))  # each pad's distance from the middle (m)
        pts = []
        for sign in (1.0, -1.0):
            mid = tcp + sign * half * rot[:, 0]
            pts += [mid, mid + 0.013 * rot[:, 2]]
        return pts

    # -- per control step
    def step(self) -> None:
        from yam_arena import ik, station
        from yam_arena.constants import READY

        self.steps += 1
        bp, bq = self.pose("ball")
        cp, cq = self.pose("cactus")
        opp, opq = self.pose("old_pot")
        npp, npq = self.pose("new_pot")
        # each wall panel's inward deflection 4.5 cm up, from its turn against the pot's base
        rbase = rules.rot(opq)
        flex = []
        for i, n in enumerate(PANEL_NAMES):
            r = rbase.T @ rules.rot(self.pose(n)[1])
            ang = math.atan2(r[0, 2], r[2, 2]) if i % 2 == 0 else math.atan2(-r[1, 2], r[2, 2])
            flex.append(INWARD[i] * ang * (ca.FLEX_Z - ca.HINGE_Z))
        squeeze = max(min(flex[0], flex[2]), min(flex[1], flex[3]))
        press = [float(np.linalg.norm(_np(self.views[n].get_contact_force_matrix(1.0 / 240.0))[0].sum(axis=0)))
                 for n in PANEL_NAMES[0::2]]
        pinch = min(press)
        tcp_home, tips = True, []
        for s in ("left", "right"):
            t, r = self.tcp(s, rot=True)
            tcp_home &= bool(np.linalg.norm(t - ik.tcp_pose(np.array(READY, float), station.BASES[s])[0]) < rules.HOME_TOL)
            tips += self.fingertips(s, t, r)
        vel = _np(self.env.unwrapped.scene["ball"].data.root_com_vel_w)[0]
        obs = {"ball": (bp, bq), "cactus": (cp, cq), "old_pot": (opp, opq), "new_pot": (npp, npq), "flex": squeeze,
               "pad_on_ball": self._pad_on_ball(bp, bq, opp, opq), "off_collar": self._off_collar_touch(cp, cq),
               "held": self._robot_touches("cactus") or self._robot_touches("ball"),
               "fall": self.max_fall, "speed": float(np.linalg.norm(vel[:3])), "tcp_home": tcp_home, "tips": tips,
               "top": TOP}
        m = self.trial.update(obs)
        if self._robot_touches("pads") and "pad_note" not in self.noted:
            self.noted.add("pad_note")
            which = np.linalg.norm(_np(self.views["pads"].get_contact_force_matrix(1.0 / 240.0))[0], axis=1)
            print(f"[cactus] fingertips on the ball (left a, left b, right a, right b): {np.round(which, 2)} (step {self.steps})",
                  flush=True)
            forces, points, _, _, counts, starts = (_np(x) for x in self.views["pads"].get_contact_data(1.0 / 240.0))
            for i in range(min(len(forces), 8)):
                f = float(np.ravel(forces[i])[0])
                if abs(f) > 1e-3:
                    loc = rules.rot(bq).T @ (points[i] - self.origin - bp)
                    print(f"[cactus]   contact on the ball at {np.round(loc * 1000, 1)} mm (ball frame), {f:.2f} N", flush=True)
        if self.trial.just_released:  # the squeeze has held 1 s: the pot's grip on the ball lets go
            self.bind_attr.Set(False)
            print(f"[cactus] the squeeze freed the ball (step {self.steps})", flush=True)
        if not m["released"] and not self._bind_noted and abs(m["risen"]) > 0.001:
            self._bind_noted = True
            print(f"[cactus] note: the ball moved {m['risen'] * 1000:.1f} mm in the bound pot (step {self.steps})", flush=True)
        for why in self.trial.faults:
            if why not in self.noted:
                self.noted.add(why)
                print(f"[cactus] zero: {why} (step {self.steps})", flush=True)
        if not m["whole"] and "torn" not in self.noted:
            self.noted.add("torn")
            print(f"[cactus] the cactus tore off its ball (step {self.steps})", flush=True)
        self.end = m
        self.report = {"score": m["score"], "depth_cm": round(m["depth"] * 100, 2), "tilt_cm": round(m["tilt"] * 100, 2),
                       "off_cm": round(m["off"] * 100, 2), "risen_cm": round(m["risen"] * 100, 2),
                       "out": int(m["out"]), "released": int(m["released"]), "on_layer": int(m["on_layer"]),
                       "held": int(m["held"]), "home": int(m["home"]), "whole": int(m["whole"]), "pinch_n": round(pinch, 1), "flex_mm": [round(f * 1000, 2) for f in flex], "press_n": [round(v, 1) for v in press],
                       "fall_cm": round(self.max_fall * 100, 2), "faults": len(self.trial.faults)}



TRIAL = Tracker()


@bk.on_reset
def _reset() -> None:
    TRIAL.reset()


@bk.on_step
def _step(env) -> None:
    TRIAL.attach(env)
    TRIAL.step()


def score(objects) -> float:
    """The tier of the state now (cactus_rules.tier); 0 once any zero has latched."""
    return float(TRIAL.end.get("score", 0.0)) if TRIAL.end else 0.0


def success(objects) -> bool:
    """The full-score state; the harness ends the trial once it has held for 1 s."""
    steptime.tick()  # the harness calls success() once per control step
    return bool(TRIAL.end) and TRIAL.end.get("score") == 1.0


# -- the scripted demo: open loop from the fixed start ----------------------------------------------------------------

def demo():
    """Waypoints for each arm, played open loop by the harness's ``yam_scripted`` policy (cactusdemo.py)."""
    import cactusdemo

    return cactusdemo.plan(os.environ.get("TASK_DEMO", "success"))


def demo_script(ctx):
    """The same open-loop plan, played step by step through demokit.py: it reads nothing from the simulator."""
    import cactusdemo

    yield from cactusdemo.play(ctx, demo())
