"""Scripted policies for the spring-plunger panel on yam_arena: the full-score demo and three controls.

    python demo.py --out runs/demo                                        # the demo: video, setup.png, result.json
    python demo.py --policy hold pull_only early_release --out runs/controls --no-video

``demo``: both hands go to their grasps, the right hand on the pull bar and the left on the red knob (approaching from
the left), and close. The left hand pulls the knob back 13.3 mm, allowing for the arm's give and trimming on the
measured plunger travel. Once the pin is back 12.2 mm, the right hand swings the panel open along the bar's arc to 41
degrees in 5 s. When the panel passes 0.12 rad (7 degrees, the strike clear of the pin), the left hand lets go and
backs away; the spring returns the pin and the right hand finishes opening.

The controls show that the panel can't be opened by force or with bad timing:

- ``hold``: both arms stay in the ready pose.
- ``pull_only``: the right hand pulls the latched panel along the same arc; the plunger is never touched.
- ``early_release``: the left hand pulls the pin fully back, then lets go while the panel is still shut; 0.6 s later
  the right hand runs the same opening.

Like ``yam_scripted``, the policies read the simulator's state (here the panel's angle and the plunger's travel) and
plan with IK on the arm's URDF; they only ever send the 14 joint and gripper targets of yam_arena's action.
Each policy runs one episode of the task (Isaac Sim starts once for all of them) and writes ``<policy>.json``, and
with video, ``<policy>.mp4``: the station's top camera at its full 640 x 360, one frame at the start and one per
control step, recorded as the Dream-1 run is (``TOP_CAM_VIDEO``, see ``mechanism.py``), and ``setup.png``, its first
frame.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ
MAX_STEPS = 600
POLICIES = ("demo", "hold", "pull_only", "early_release")

APPROACH_S = 2.0      # ready pose to the pregrasps, in joint space (the right hand by way of a waypoint)
MOVE_IN_S = 1.5       # pregrasps to the grasps
CLOSE_S = 1.1         # grippers closing
PULL_AT = APPROACH_S + MOVE_IN_S + CLOSE_S
PULL_S = 1.3
PULL_M = 0.0133       # the knob is pulled back 13.3 mm; 12 mm frees the strike, 14 mm is the plunger's stop
GIVE = 0.25           # the arm gives about this much per metre of pull against the spring (its wrist is soft)
OPEN_FROM_M = 0.0122  # the right hand starts opening once the pin is back this far
OPEN_AT = PULL_AT + PULL_S
OPEN_S = 5.0
OPEN_TO = math.radians(41.0)
RELEASE_AT = 0.12     # rad: the left hand lets go once the panel is this far open (the strike is clear of the pin)
RELEASE_S = 1.0       # the left gripper opens over this long: within inspect-robots' default limit of 5% per step
LATE_RELEASE_DELAY_S = 0.6
HOLD_AFTER_S = 1.5


def _frame(x_axis, z_axis) -> np.ndarray:
    """A TCP rotation from its finger-closing axis (x) and approach axis (z), world frame."""
    x, z = np.asarray(x_axis, float), np.asarray(z_axis, float)
    return np.column_stack([x, np.cross(z, x), z])


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# The right hand reaches the pull bar from the front (approach +x), fingers closing across it; the left hand reaches
# the knob from the left (approach -y), fingers closing on its rim from above and below.
HANDLE_GRIP = _frame((0.0, -1.0, 0.0), (1.0, 0.0, 0.0))
KNOB_GRIP = _frame((0.0, 0.0, 1.0), (0.0, -1.0, 0.0))


def _smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


def _ramp(s: float) -> float:
    return min(max(s, 0.0), 1.0)


class Arm:
    """One arm's joint targets: IK on the URDF, warm-started from the last solution."""

    def __init__(self, side: str):
        from yam_arena import station
        from yam_arena.constants import READY

        self.side = side
        self.base = np.asarray(station.BASES[side], float)
        self.ready = np.array(READY, float)
        self.q = self.ready.copy()
        self.grip = 1.0  # 1 open .. 0 closed
        self.worst_mm = 0.0

    def solve(self, pos, rot, q_init=None, iters: int = 60) -> np.ndarray:
        from yam_arena import ik

        q, pos_err, _ = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot),
                                 self.q if q_init is None else q_init, base_pos=self.base, iters=iters)
        self.worst_mm = max(self.worst_mm, pos_err * 1000)
        return q

    def to(self, pos, rot) -> None:
        self.q = self.solve(pos, rot)

    def command(self) -> np.ndarray:
        return np.r_[self.q, self.grip]


class Scripted:
    """The demo and its controls as one closed-loop script; ``variant`` picks which."""

    def __init__(self, task, variant: str):
        self.task, self.variant = task, variant
        self.left, self.right = Arm("left"), Arm("right")
        self.pregrasp = {"right": task.HANDLE + [-0.075, 0.0, 0.0], "left": task.KNOB + [0.0, 0.05, 0.0]}
        # At the ready pose the right hand is above the cabinet's front; it first goes to this point, 15 cm in front
        # of the panel and 30 cm above the table, then down to its pregrasp, so it never brushes the panel.
        self.waypoint = {"right": np.array([0.2475, -0.20, task.TOP + 0.30])}
        self.grips = {"right": HANDLE_GRIP, "left": KNOB_GRIP}
        self.active = {"demo": ("left", "right"), "early_release": ("left", "right"), "pull_only": ("right",),
                       "hold": ()}[variant]
        self.paths = {}
        for side in self.active:
            arm = getattr(self, side)
            path, q = [arm.ready.copy()], arm.ready.copy()
            for point in ([self.waypoint[side]] if side in self.waypoint else []) + [self.pregrasp[side]]:
                q = arm.solve(point, self.grips[side], q_init=q, iters=400)
                path.append(q)
            self.paths[side] = path
        self.open_at = self.released_at = None
        self.end = None

    def _approach(self, side: str, t: float) -> None:
        arm, path = getattr(self, side), self.paths[side]
        legs = len(path) - 1
        u = min(t / APPROACH_S, 1.0) * legs
        leg = min(int(u), legs - 1)
        arm.q = path[leg] + _smooth(u - leg) * (path[leg + 1] - path[leg])

    def step(self, t: float, angle: float, travel: float) -> np.ndarray:
        """The 14 targets for the control step that ends at time ``t`` (s), given the panel's angle (rad) and the
        plunger's travel (m) now."""
        T, L, R = self.task, self.left, self.right
        if t < APPROACH_S:
            for side in self.active:
                self._approach(side, t)
            return np.r_[L.command(), R.command()]
        tau = t - APPROACH_S
        closing = _ramp((tau - MOVE_IN_S) / CLOSE_S)

        # Events: when the opening starts, when the left hand lets go.
        if self.variant == "demo":
            if self.open_at is None and t >= OPEN_AT and travel >= OPEN_FROM_M:
                self.open_at = t
            if self.released_at is None and self.open_at is not None and angle >= RELEASE_AT:
                self.released_at = t
        elif self.variant == "early_release":
            if self.released_at is None and t >= OPEN_AT and travel >= OPEN_FROM_M:
                self.released_at = t
            if self.open_at is None and self.released_at is not None and t >= self.released_at + LATE_RELEASE_DELAY_S:
                self.open_at = t
        elif self.variant == "pull_only" and self.open_at is None and t >= OPEN_AT:
            self.open_at = t

        if "right" in self.active:
            opened = OPEN_TO * _smooth((t - self.open_at) / OPEN_S) if self.open_at is not None else 0.0
            target = T.handle_at(opened)
            if tau < MOVE_IN_S:
                target = self.pregrasp["right"] + _smooth(tau / MOVE_IN_S) * (target - self.pregrasp["right"])
            R.to(target, _rz(opened) @ HANDLE_GRIP)
            R.grip = 1.0 - closing
            if self.open_at is not None and self.end is None and t >= self.open_at + OPEN_S:
                self.end = t + HOLD_AFTER_S

        if "left" in self.active:
            wanted = PULL_M * _smooth((t - PULL_AT) / PULL_S)
            # Allow for the arm's give against the spring, and trim on the measured travel, without driving the
            # knob into its 14 mm stop.
            correction = GIVE * wanted + float(np.clip(wanted - travel, 0.0, 0.002)) if t >= PULL_AT else 0.0
            knob = T.KNOB + [0.0, wanted + correction, 0.0]
            grip = 1.0 - closing
            if self.released_at is not None:
                age = t - self.released_at
                grip = _ramp(age / RELEASE_S)
                # Back off along the approach, then up and away from the panel's path.
                knob = knob + [0.0, 0.06 * _smooth((age - RELEASE_S) / 0.8),
                               0.04 * _smooth((age - RELEASE_S - 0.8) / 0.8)]
            if tau < MOVE_IN_S:
                knob = self.pregrasp["left"] + _smooth(tau / MOVE_IN_S) * (knob - self.pregrasp["left"])
            L.to(knob, KNOB_GRIP)
            L.grip = grip
        return np.r_[L.command(), R.command()]

    def done(self, t: float) -> bool:
        """The demo stops HOLD_AFTER_S after the opening ends (if success hasn't ended it already); the controls run
        the whole trial, so their score is the one a full run would get."""
        return self.variant == "demo" and self.end is not None and t >= self.end


def run_episode(emb, policy: str, out: Path, max_steps: int, video: bool, write_setup: bool) -> dict:
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack

    import mechanism  # the module task.py loaded (task.py puts this folder on sys.path)

    task = emb.task.module
    started = time.time()
    if video:  # the top camera, recorded from the reset on (mechanism.py)
        os.environ[mechanism.TOP_CAM_VIDEO] = str(out / f"{policy}.mp4")
    else:
        os.environ.pop(mechanism.TOP_CAM_VIDEO, None)
    emb.reset(Scene(id=f"spring-panel-{policy}", instruction=task.INSTRUCTION))
    from yam_arena.scene import object_states  # importable once the simulator runs

    if video and write_setup:
        shutil.copyfile(out / f"{policy}.png", out / "setup.png")
    script = Scripted(task, policy)
    steps, success, info, first_full, max_travel = 0, False, {}, None, 0.0
    trace = []  # per step: time (s), panel angle (deg), plunger travel (mm), score, left and right grip commands
    try:
        while steps < max_steps:
            joints = object_states(emb._env, ["cabinet"])["cabinet"].joints
            angle, travel = joints["panel_hinge"], joints["plunger_slide"]
            max_travel = max(max_travel, travel)
            t = (steps + 1) * DT
            action = script.step(t, angle, travel)
            result = emb.step(Action(data=pack(action[:7], action[7:])))
            steps += 1
            info = result.info
            trace.append([round(t, 2), round(math.degrees(angle), 3), round(1000 * travel, 3),
                          float(info.get("score", 0.0) or 0.0), round(float(action[6]), 3), round(float(action[13]), 3)])
            if first_full is None and float(info.get("score", 0.0) or 0.0) >= 1.0:
                first_full = steps
            if result.terminated:
                success = result.termination_reason == "success"
                break
            if script.done(t):
                break
    finally:
        mechanism._RECORDER.close()
    joints = object_states(emb._env, ["cabinet"])["cabinet"].joints
    record = task.episode()
    out_json = {
        "policy": policy,
        "task_score": 1.0 if success else float(info.get("score", 0.0) or 0.0),
        "success": success,
        "steps": steps,
        "first_full_score_step": first_full,
        "final_angle_deg": round(math.degrees(joints["panel_hinge"]), 3),
        "max_angle_deg": round(record["max_angle_deg"], 3),
        "max_plunger_travel_mm": round(1000 * max(max_travel, joints["plunger_slide"]), 2),
        "worst_pin_overlap_mm": round(record["worst_overlap_mm"], 3),
        "forced": record["forced"],
        "worst_ik_mm": {"left": round(script.left.worst_mm, 2), "right": round(script.right.worst_mm, 2)},
        "wall_s": round(time.time() - started, 1),
    }
    (out / f"{policy}.json").write_text(json.dumps(out_json, indent=2) + "\n")
    (out / f"{policy}_trace.json").write_text(json.dumps(
        {"columns": ["t_s", "angle_deg", "travel_mm", "score", "left_grip", "right_grip"], "rows": trace}) + "\n")
    print(json.dumps(out_json), flush=True)
    return out_json


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--policy", nargs="+", choices=POLICIES, default=["demo"])
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--max-steps", type=int, default=MAX_STEPS)
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--cam-width", type=int, default=640)
    ap.add_argument("--cam-height", type=int, default=360)
    ap.add_argument("--window", action="store_true", help="open the Isaac Sim window instead of running headless")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    # Kit reads its settings from the command line: render on the simulation's GPU only, as Isaac Lab does for
    # multi_gpu=False. On a machine with several GPUs, choose one with CUDA_VISIBLE_DEVICES.
    sys.argv = [sys.argv[0], "--/renderer/multiGpu/activeCudaGpus=0,"]

    from yam_arena.inspect_embodiment import yam_arena_embodiment

    emb = yam_arena_embodiment(task=str(TASK), object_jitter=0, control_hz=CONTROL_HZ, cam_width=args.cam_width,
                               cam_height=args.cam_height, headless=not args.window)
    results = [run_episode(emb, policy, args.out, args.max_steps, not args.no_video, write_setup=(i == 0))
               for i, policy in enumerate(args.policy)]
    (args.out / "result.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
