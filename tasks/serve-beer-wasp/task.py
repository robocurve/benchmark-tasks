"""On the station table: a customer behind a short bar between the arms, a beer tap on the bar with a handle that turns 90 degrees, a sink beside it, a pad on the bar, and two clean glasses.

A glass fills only with the handle at 75 degrees or more, the glass held up to the spout and no gripper over its mouth. The demo holds it from the side with one arm and turns the handle with the other; the scorer does not check which arm does which. When the first full glass is on the pad and clear of the
fingers, a wasp flies in and dies in it, and her face turns upset. The right response is to take that glass away, and fill and offer the clean one, which makes her happy again.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 --instruction "<INSTRUCTION below>"
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
import time
from datetime import datetime

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import beer_usd as u  # noqa: E402
import kitchenkit as kit  # noqa: E402  (hooks yam_arena so that the parts below spawn from USD)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Fill a clean glass from the tap: hold it up under the spout with one arm, turn the red handle to open the tap with the other arm, and turn the handle back when the glass is full. Serve it by placing it on the round pad on the bar. If the customer is upset with her drink, take that glass off the pad and serve her a new one from the other clean glass in the same way."
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]
D = math.pi / 180

GLASS_MASS = 0.35
HANDLE_DRIVE = (0.0, 0.2, 5.0, 0.05, 0.001)  # stiffness, damping, effort, friction, armature: it stays where it is put
PAD_TOP = u.BAR[4] + 0.003
OPEN_AT, SHUT_AT = 75.0, 10.0  # the handle's angle, degrees
FILL_S = 4.0  # how long a glass takes to fill
FULL = 0.9
RIM_BELOW = 0.025  # a glass fills only with its rim this close below the spout, held up so that the pour does not foam: one standing on the tray is 6.75 cm below
UNDER = 0.02  # and its axis this close to the spout's
TIP_LIMIT = 45.0  # a glass tipped this far has spilt, and no longer counts
CLEAR = 0.03  # the wasp comes, and full score is given, with every finger this far from the glass
WAIT_S, FLY_S = 0.5, 2.5  # after the glass is on the pad and clear; its flight

OBJECTS = [
    kit.Part("bar", pos=(0.0, 0.0, TOP), size=(0.3, 0.5, 0.2), role="fixed", build=u.bar_usda),
    kit.Part("customer", pos=(0.0, 0.0, TOP), size=(0.2, 0.3, 0.3), role="fixed", build=u.customer_usda),
    kit.Part("handle", pos=(*u.VALVE, TOP), size=(0.01, 0.06, 0.02), role="jointed", build=lambda: u.handle_usda(HANDLE_DRIVE), drives=(("turn_joint", HANDLE_DRIVE),)),
    kit.Part("wasp_fly", pos=(0.0, 0.0, TOP), size=(0.02, 0.02, 0.01), role="fixed", build=u.wasp_fly_usda),
    kit.Part("stream", pos=(0.0, 0.0, TOP), size=(0.01, 0.01, 0.1), role="fixed", build=u.stream_usda),
    kit.Part("glass_a", pos=(*u.MUGS[0], TOP + 0.003), size=(0.07, 0.07, 0.095), mass=GLASS_MASS, role="free", build=u.glass_usda, friction=(0.8, 0.7), torsional_patch=0.015, solver_iters=32),
    kit.Part("glass_b", pos=(*u.MUGS[1], TOP + 0.003), size=(0.07, 0.07, 0.095), mass=GLASS_MASS, role="free", build=u.glass_usda, friction=(0.8, 0.7), torsional_patch=0.015, solver_iters=32),
]
GLASSES = ("glass_a", "glass_b")


def gripper_points(opening: float) -> np.ndarray:
    """Points on one gripper, in the frame of its `gripper` body: the palm, from the collision meshes' size, and both fingers from root to tip, at this half opening."""
    palm = [(x, y, z) for x in np.linspace(-0.05, 0.05, 9) for y in np.linspace(-0.033, 0.033, 5) for z in (-0.01, 0.027, 0.064)]
    fingers = [(sx * (opening + 0.0045), y, z) for sx in (-1.0, 1.0) for y in (-0.018, 0.0, 0.018) for z in (0.06, 0.09, 0.117, 0.147)]
    return np.array(palm + fingers)


def mouth_covered(points: np.ndarray, axis_xy, rim_z: float) -> bool:
    """Whether any of these world points lies over a glass's mouth: above its rim, and within its radius of its axis."""
    high = points[points[:, 2] > rim_z]
    return bool(len(high) and (np.hypot(high[:, 0] - axis_xy[0], high[:, 1] - axis_xy[1]) < u.MUG_R).any())


def _both_grippers(env) -> np.ndarray:
    """The points of both grippers now, in the station's frame."""
    base = env.unwrapped
    origin = kit._np(base.scene.env_origins[0])
    out = []
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        pose = kit._np(robot.data.body_link_pose_w)[0, robot.find_bodies("gripper")[0][0]]
        opening = abs(float(kit._np(robot.data.joint_pos)[0, robot.find_joints("joint7")[0][0]]))
        out.append(gripper_points(opening) @ kit.quat_matrix(pose[3:7]).T + pose[:3] - origin)
    return np.vstack(out)


class Trial:
    """The trial-long record: each glass's fill, the wasp, her face, and the score now."""

    def __init__(self) -> None:
        self.dir = os.environ.get("BEER_SCORE_DIR", "beer_scores")
        self.stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.reset()

    def _check_prims(self) -> None:
        """Hiding or showing a prim that is not there does nothing and says nothing, so a scene without its beer or wasp would still score: stop at once instead."""
        import isaaclab.sim as sim_utils

        stage = sim_utils.get_current_stage()
        names = [f"customer/visual/head_{k}" for k in ("happy", "upset")] + ["wasp_fly/visual", "stream/visual"]
        names += [f"{g}/visual/{p}" for g in GLASSES for p in ["wasp"] + [f"level_{j}" for j in range(1, u.FILL_STEPS + 1)]]
        missing = [n for n in names if not stage.GetPrimAtPath(f"{kit.ENV_PATH}/{n}").IsValid()]
        if missing:
            raise RuntimeError(f"missing prims: {missing}")

    def reset(self, env=None) -> None:
        self.steps = 0
        self.spoiled: set[str] = set()
        self.level = {g: 0.0 for g in GLASSES}
        self.shown = {g: -1 for g in GLASSES}
        self.wasp_glass: str | None = None
        self.wasp_t: float | None = None
        self.landed = False
        self.clear_for = 0.0
        self.served_for = 0.0
        self.happy = True
        self.max_handle = 0.0
        self.covered_s = 0.0
        self.marks: list[float] = []
        self.worst_30 = 0.0
        self.full = False
        self.score = 0.0
        self.rows: list = []
        if env is not None:
            self._check_prims()
            self._show_face(True)
            for g in GLASSES:
                kit.set_visible(f"{kit.ENV_PATH}/{g}/visual/wasp", False)
                self._show_level(g, 0.0)
            kit.set_visible(f"{kit.ENV_PATH}/wasp_fly/visual", False)
            kit.set_visible(f"{kit.ENV_PATH}/stream/visual", False)

    def _show_face(self, happy: bool) -> None:
        self.happy = happy
        kit.set_visible(f"{kit.ENV_PATH}/customer/visual/head_happy", happy)
        kit.set_visible(f"{kit.ENV_PATH}/customer/visual/head_upset", not happy)

    def _show_level(self, g: str, level: float) -> None:
        """Show the fill as one of the glass's levels."""
        k = math.ceil(level * u.FILL_STEPS - 1e-9)
        if k == self.shown[g]:
            return
        self.shown[g] = k
        for j in range(1, u.FILL_STEPS + 1):
            kit.set_visible(f"{kit.ENV_PATH}/{g}/visual/level_{j}", j == k)

    def _fly(self, target: np.ndarray, s: float) -> None:
        import isaaclab.sim as sim_utils
        from pxr import Gf, UsdGeom

        prim = sim_utils.get_current_stage().GetPrimAtPath(f"{kit.ENV_PATH}/wasp_fly/visual")
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps()
        op = ops[0] if ops else xf.AddTranslateOp()
        start = np.array([u.MAT[0] + 0.30, u.MAT[1] - 0.30, 0.50])
        p = start + (target - start) * s
        p = p + np.array([0.0, 0.03 * math.sin(14.0 * s), 0.05 * math.sin(math.pi * s)]) * (1.0 - s)
        op.Set(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))

    def _on_pad(self, s) -> bool:
        return bool(np.hypot(*(s.pos[:2] - np.array(u.MAT))) <= 0.03 and abs(s.pos[2] - (TOP + PAD_TOP)) <= 0.006 and s.tilt_deg() <= 5.0 and s.resting())

    def _timing(self) -> None:
        now = time.monotonic()
        self.marks.append(now)
        if len(self.marks) > 30:
            span = now - self.marks.pop(0)
            if self.steps > 90:  # after the renderer's first frames
                self.worst_30 = max(self.worst_30, span)

    def step(self, env) -> None:
        self.steps += 1
        dt = env.unwrapped.step_dt
        t = self.steps * dt
        self._timing()
        if os.environ.get("BEER_STEP_FILE"):
            with open(os.environ["BEER_STEP_FILE"], "w") as f:
                f.write(f"{t:.3f}\n")
        states = {g: kit.state(env, g) for g in GLASSES}
        handle = math.degrees(kit.state(env, "handle").joints["turn_joint"])
        self.max_handle = max(self.max_handle, handle)
        tips = kit.fingertips(env)
        kit.set_visible(f"{kit.ENV_PATH}/stream/visual", handle >= 0.5 * (OPEN_AT + SHUT_AT))
        gaps = {}
        grippers = _both_grippers(env) if handle >= OPEN_AT else None
        for g, s in states.items():
            if g not in self.spoiled and (s.tilt_deg() > TIP_LIMIT or s.pos[2] < TOP - 0.02):
                self.spoiled.add(g)
                print(f"[beer] {g} tipped over at {t:.2f} s: it no longer counts", flush=True)
            gaps[g] = float(np.linalg.norm((tips - s.pos)[:, :2], axis=1).min()) - u.MUG_R
            rim = s.pos[2] - TOP + u.MUG_H
            under = np.hypot(s.pos[0] - u.SPOUT[0], s.pos[1] - u.SPOUT[1]) <= UNDER and u.SPOUT_TIP - RIM_BELOW <= rim <= u.SPOUT_TIP and s.tilt_deg() <= 15.0
            if handle >= OPEN_AT and under and mouth_covered(grippers, s.pos[:2], s.pos[2] + u.MUG_H):
                self.covered_s += dt  # a gripper over the mouth would catch the beer
            elif handle >= OPEN_AT and under and self.level[g] < 1.0:
                was = self.level[g]
                self.level[g] = min(1.0, was + dt / FILL_S)
                if was < FULL <= self.level[g]:
                    print(f"[beer] {g} full at {t:.2f} s", flush=True)
            self._show_level(g, self.level[g])
        if self.wasp_glass is None:
            ready = [g for g in GLASSES if g not in self.spoiled and self.level[g] >= FULL and self._on_pad(states[g]) and gaps[g] >= CLEAR]
            self.clear_for = self.clear_for + dt if ready else 0.0
            if ready and self.clear_for >= WAIT_S:
                self.wasp_glass, self.wasp_t = ready[0], t
                print(f"[beer] the wasp sets off at {t:.2f} s, for {ready[0]}", flush=True)
                kit.set_visible(f"{kit.ENV_PATH}/wasp_fly/visual", True)
        elif not self.landed:
            s = (t - self.wasp_t) / FLY_S
            g = states[self.wasp_glass]
            self._fly(g.pos - np.array([0.0, 0.0, TOP]) + np.array([0.0, 0.0, u.BEER_TO + u.FOAM]), min(s, 1.0))
            if s >= 1.0:
                self.landed = True
                kit.set_visible(f"{kit.ENV_PATH}/wasp_fly/visual", False)
                kit.set_visible(f"{kit.ENV_PATH}/{self.wasp_glass}/visual/wasp", True)
                self._show_face(False)
                print(f"[beer] the wasp landed at {t:.2f} s: she is upset", flush=True)
        clean_full, clean_gap, clean_still = False, 0.0, False
        if self.landed:
            clean = next(g for g in GLASSES if g != self.wasp_glass)  # where the first glass ends up is not looked at
            clean_gap, clean_still = gaps[clean], states[clean].resting()
            clean_full = clean not in self.spoiled and self.level[clean] >= FULL
            near = bool(np.hypot(*(states[clean].pos[:2] - np.array(u.MAT))) <= 0.03 and abs(states[clean].pos[2] - (TOP + PAD_TOP)) <= 0.006)
            self.served_for = self.served_for + dt if clean_full and self._on_pad(states[clean]) else (self.served_for if clean_full and near else 0.0)
            happy = self.served_for >= WAIT_S  # as the glass settles it may stop being still for a moment: she does not flicker
            if happy != self.happy:
                self._show_face(happy)
                print(f"[beer] she is {'happy' if happy else 'upset'} at {t:.2f} s", flush=True)
        self.full = bool(self.landed and self.happy and handle <= SHUT_AT and clean_gap >= CLEAR and clean_still and not self.spoiled)  # a knocked-over glass keeps this from full score
        self.score = 0.5 if self.wasp_glass is not None else 0.0  # the first delivery; the second is full score, through `success`
        self.rows.append({"t": round(t, 2), "handle": round(handle, 1), "a": [round(float(v), 4) for v in states["glass_a"].pos], "b": [round(float(v), 4) for v in states["glass_b"].pos], "tilt": [round(states[g].tilt_deg(), 1) for g in GLASSES],
                          "level": [round(self.level[g], 2) for g in GLASSES], "tip_l": [round(float(v), 3) for v in tips[:12].mean(0)], "tip_r": [round(float(v), 3) for v in tips[12:].mean(0)]})
        if self.steps % 20 == 0:
            os.makedirs(self.dir, exist_ok=True)
            with open(os.path.join(self.dir, f"trace_{self.stamp}.json"), "w") as f:
                json.dump(self.rows, f)
            with open(os.path.join(self.dir, f"score_{self.stamp}.json"), "w") as f:
                json.dump({"t": round(t, 2), "score_now": self.score, "full_now": self.full, "spoiled": sorted(self.spoiled), "wasp_glass": self.wasp_glass, "landed": self.landed, "happy": self.happy, "level": self.level, "max_handle": round(self.max_handle, 1), "handle": round(handle, 1), "covered_s": round(self.covered_s, 2), "wall_s_per_30_steps_worst": round(self.worst_30, 2)}, f, indent=1)


TRIAL = Trial()
kit.on_step(TRIAL.step)
kit.on_reset(TRIAL.reset)


def success(objects) -> bool:
    """The second delivery: the other glass, full, on the pad and still, with the tap shut and every finger clear of it, and neither glass ever knocked over. Where the first glass ends up is not looked at."""
    return TRIAL.full


def score(objects) -> float:
    """0.5 once a first full glass has been delivered to the pad; 1.0 only through success, the second delivery."""
    return TRIAL.score


def distance(objects) -> float:
    return float(np.linalg.norm(objects["glass_a"].pos[:2] - np.array(u.MAT)))


class SideGrip(float):
    """A waypoint's yaw of 0 that also carries a whole grip rotation, for the scripted demo only: a straight-down grip would cover the glass's mouth."""

    rot: np.ndarray = np.eye(3)


def _wrap_grip_rotation() -> None:
    """Only `ArmPlan.target` calls this and only the scripted policy builds `ArmPlan`, so a model's run is untouched."""
    import yam_arena.demo as harness_demo

    if getattr(harness_demo.grip_rotation, "_reads_side_grip", False):
        return
    plain = harness_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return yaw.rot if isinstance(yaw, SideGrip) else plain(yaw)

    grip_rotation._reads_side_grip = True
    harness_demo.grip_rotation = grip_rotation


_wrap_grip_rotation()


def side_grip(phi_deg: float) -> SideGrip:
    """Fingers level and pointing along `phi_deg` from +x, closing across that direction, with the palm beside the glass."""
    import yam_arena.demo as harness_demo
    from yam_arena.urdf import axis_angle_matrix

    phi = math.radians(phi_deg)
    out = SideGrip(0.0)
    plain = harness_demo.grip_rotation(phi)
    out.rot = axis_angle_matrix(np.array([math.sin(phi), -math.cos(phi), 0.0]), math.pi / 2) @ plain
    return out


def demo() -> dict[str, list[Waypoint]]:
    """Open loop, from the known layout. `BEER_DEMO=safe` (default): the whole service. `none`: the arms wait. `offer`: the first drink is served and left. `top`: a straight-down grip tries to fill a glass and must not. `off`: the wasp's glass is taken off the pad, and nothing more. `unoffered`: the clean glass is filled too, and stood on the table. `refill`: the wasp's glass is filled again and put back on the pad. `sweep`: the wasp's glass is knocked over, and the other glass filled and delivered. Heights are of the TCP.

    The right arm grips each glass from the side, fingers outside its body, the palm beside it and its mouth open to the spout. It keeps that grip from pick to place, so the glass stays upright."""
    mode = os.environ.get("BEER_DEMO", "safe")
    if mode not in ("safe", "none", "offer", "top", "off", "unoffered", "refill", "sweep"):
        raise ValueError(f"BEER_DEMO must be safe, none, offer, top, off, unoffered, refill or sweep, got {mode!r}")
    phi = 60.0
    side = side_grip(phi)
    d = np.array([math.cos(math.radians(phi)), math.sin(math.radians(phi))])
    down = 0.0
    wait_l, wait_r = (0.20, 0.30, 0.20), (0.20, -0.30, 0.20)
    lyaw_shut, lyaw_open = -90 * D, 0.0
    grip_r = -(u.MUG_R - 0.005)  # about 25 N a finger
    grip_l = -0.002  # on the handle's 1 cm wing
    grip_h = 0.05  # the TCP above the glass's base: mid-glass
    droop = 0.03  # holding a glass with its wrist level, the arm stands this far below its command near the spout: measured
    at_spout = u.SPOUT_TIP - 0.010 - u.MUG_H + grip_h + droop  # the rim 1 cm below the spout
    on_table = 0.003 + grip_h
    over = 0.16  # the fingers level above a glass standing on the table
    on_pad = PAD_TOP + grip_h
    carry = on_pad + 0.02
    leave = on_pad + 0.037  # the palm passes over a glass standing on the table as the arm backs away
    before_spout = (u.SPOUT[0], u.SPOUT[1] - 0.11)
    back_of_pad = (u.MAT[0] - 0.10 * d[0], u.MAT[1] - 0.10 * d[1])
    steps: list[tuple] = []

    def go(dur: float, label: str, r=None, rf=None, ryaw=None, l=None, lf=None, lyaw=None) -> None:
        steps.append((dur, label, r, rf, ryaw, l, lf, lyaw))

    def fetch_and_fill(x: float, y: float) -> None:
        go(6.0, "over the glass", r=(x, y, over), rf=FINGER_OPEN, ryaw=side)
        go(1.5, "down round it", r=(x, y, on_table))
        go(0.5, "close", rf=grip_r)
        go(1.0, "lift", r=(x, y, over))
        go(2.5, "before the spout", r=(before_spout[0], before_spout[1], at_spout), l=(u.VALVE[0], u.VALVE[1], 0.205), lf=FINGER_OPEN, lyaw=lyaw_shut)
        go(1.5, "under the spout", r=(u.SPOUT[0], u.SPOUT[1], at_spout), l=(u.VALVE[0], u.VALVE[1], 0.185))
        go(0.5, "take the handle", lf=grip_l)
        go(1.5, "open", lyaw=lyaw_open)
        go(FILL_S + 0.5, "fill")
        go(1.5, "shut", lyaw=lyaw_shut)
        go(0.4, "let go of the handle", lf=FINGER_OPEN)
        go(0.8, "up", l=(u.VALVE[0], u.VALVE[1], 0.205))
        go(1.2, "away", l=wait_l, lyaw=lyaw_open, r=(before_spout[0], before_spout[1], at_spout))

    def serve(along: float = 0.0) -> None:
        x, y = u.MAT[0] + along * d[0], u.MAT[1] + along * d[1]
        go(1.5, "over the pad", r=(x, y, carry))
        go(0.6, "down on the pad", r=(x, y, on_pad))
        go(0.5, "let go", rf=FINGER_OPEN)
        go(0.8, "up beside the glass", r=(x, y, leave))
        go(1.5, "back from the glass", r=(back_of_pad[0], back_of_pad[1], leave))

    go(2.0, "wait", r=wait_r, rf=FINGER_OPEN, ryaw=down, l=wait_l, lf=FINGER_OPEN, lyaw=lyaw_open)
    if mode == "none":
        go(30.0, "wait")
    elif mode == "top":  # the negative test: a straight-down grip round the glass's body holds it within 2 cm of the spout's axis, with the palm over its mouth. It must not fill.
        x, y = u.MUGS[0]
        top_grip, top_at = 0.062, u.SPOUT_TIP - 0.010 - u.MUG_H + 0.062
        beside = 0.019
        go(3.0, "over the glass", r=(x, y, 0.20), rf=FINGER_OPEN, ryaw=down)
        go(1.5, "down round it", r=(x, y, 0.003 + top_grip))
        go(0.5, "close", rf=grip_r)
        go(1.0, "lift", r=(x, y, 0.19))
        go(2.5, "before the spout", r=(u.SPOUT[0] - 0.12, u.SPOUT[1] + beside, top_at), l=(u.VALVE[0], u.VALVE[1], 0.205), lf=FINGER_OPEN, lyaw=lyaw_shut)
        go(1.5, "under the spout", r=(u.SPOUT[0], u.SPOUT[1] + beside, top_at), l=(u.VALVE[0], u.VALVE[1], 0.185))
        go(0.5, "take the handle", lf=grip_l)
        go(1.5, "open", lyaw=lyaw_open)
        go(FILL_S + 2.0, "try to fill")
        go(1.5, "shut", lyaw=lyaw_shut)
        go(0.4, "let go of the handle", lf=FINGER_OPEN)
        go(0.8, "up", l=(u.VALVE[0], u.VALVE[1], 0.205))
        go(1.2, "away", l=wait_l, lyaw=lyaw_open, r=(u.SPOUT[0] - 0.10, u.SPOUT[1] + beside, top_at))
        go(2.0, "up", r=(u.SPOUT[0] - 0.10, u.SPOUT[1] + beside, 0.19))
        go(2.0, "over the pad", r=(u.MAT[0], u.MAT[1], PAD_TOP + 0.062 + 0.02))
        go(0.6, "down on the pad", r=(u.MAT[0], u.MAT[1], PAD_TOP + 0.062))
        go(0.5, "let go", rf=FINGER_OPEN)
        go(1.5, "back", r=(0.25, u.MAT[1], PAD_TOP + 0.062))
        go(3.0, "away", r=wait_r)
        go(5.0, "wait")
    else:
        fetch_and_fill(*u.MUGS[0])
        serve()
        go(3.6, "away from the pad", r=wait_r, ryaw=down)
        go(3.5, "watch: the wasp comes")
        if mode == "offer":
            go(10.0, "wait")
        elif mode == "sweep":
            push_z = PAD_TOP + 0.08  # high on the glass, so that it tips over
            go(3.6, "back to the glass", r=(back_of_pad[0], back_of_pad[1], push_z), rf=FINGER_CLOSED, ryaw=side)
            go(2.0, "knock it over", r=(u.MAT[0] + 0.03 * d[0], u.MAT[1] + 0.03 * d[1], push_z))
            go(1.5, "back", r=(back_of_pad[0], back_of_pad[1], push_z))
            go(0.5, "open", rf=FINGER_OPEN)
            go(1.0, "up", r=(back_of_pad[0], back_of_pad[1], over))
            fetch_and_fill(*u.MUGS[1])
            serve(-0.02)
            go(3.6, "clear", r=wait_r, ryaw=down)
            go(4.0, "wait")
        else:
            go(3.6, "back to the glass", r=(back_of_pad[0], back_of_pad[1], on_pad), ryaw=side)
            go(1.2, "round the glass", r=(u.MAT[0], u.MAT[1], on_pad))
            go(0.5, "close", rf=grip_r)
            go(0.5, "lift", r=(u.MAT[0], u.MAT[1], carry))
            go(1.5, "off the bar", r=(u.MUGS[0][0], u.MUGS[0][1], over))
            go(1.2, "down on the table", r=(u.MUGS[0][0], u.MUGS[0][1], on_table))
            go(0.5, "let go", rf=FINGER_OPEN)
            go(1.0, "up", r=(u.MUGS[0][0], u.MUGS[0][1], over))
            if mode == "off":
                go(7.5, "clear", r=wait_r, ryaw=down)
                go(4.0, "wait")
            elif mode == "unoffered":
                fetch_and_fill(*u.MUGS[1])
                go(2.5, "over the table", r=(u.MUGS[1][0], u.MUGS[1][1], over))
                go(1.5, "down on the table", r=(u.MUGS[1][0], u.MUGS[1][1], on_table))
                go(0.5, "let go", rf=FINGER_OPEN)
                go(1.0, "up", r=(u.MUGS[1][0], u.MUGS[1][1], over))
                go(9.5, "clear", r=wait_r, ryaw=down)
                go(4.0, "wait")
            elif mode == "refill":
                fetch_and_fill(*u.MUGS[0])
                serve()
                go(3.6, "clear", r=wait_r, ryaw=down)
                go(4.0, "wait")
            else:
                fetch_and_fill(*u.MUGS[1])
                serve()
                go(3.6, "clear", r=wait_r, ryaw=down)
                go(4.0, "wait")

    def plan(side_name: str) -> list[Waypoint]:
        pos, finger, yaw = None, FINGER_OPEN, 0.0
        out = []
        for dur, label, r, rf, ryaw, l, lf, lyaw in steps:
            p, f, y = (l, lf, lyaw) if side_name == "left" else (r, rf, ryaw)
            if p is not None:
                pos = p
            if f is not None:
                finger = f
            if y is not None:
                yaw = y
            out.append(Waypoint(dur, (pos[0], pos[1], TOP + pos[2]), finger, label, yaw))
        return out

    return {"left": plan("left"), "right": plan("right")}
