"""Closed-loop scripted runner for charger task1 on the yam_arena embodiment (Isaac Lab-Arena, PhysX).

Robot-only: every action is a 14-D joint-position target sent through the embodiment (same path as a policy).
It READS object and link poses to plan (allowed for scripted demos); it never writes poses, joint states or scores.
Real YAM actuator gains (SDK) plus an integral term on the measured joints against PD sag (pattern of earlier yam_arena demos).

Stages (--stage):
  demo            strategy (b): black into -, red into +, one at a time, tips kept apart; park; 5 s released hold
  one-lead        black only, then park and hold (expected 0.5)
  tip-touch       live red plug, held level, lowered so its pin lands across the top of the black pin lying on the mat
                  (expected veto, live short)
  case-touch      live red pin nose driven onto the aluminium output wall next to the + jack (expected veto)
  reverse-live    live red plug pushed into the - jack (expected veto)
  xt60-unplugged  evaluator reset variant TASK1_VARIANT=xt60-open (XT60 starts unmated, leads dead); both plugs seated,
                  XT60 left unplugged at the end (expected 0.75: strategy-(a) end state without the re-mate)
  pull-ac         right arm pulls the preset C5 plug 25 mm out of the inlet (expected veto "preset AC cord unseated")
  smoke           do nothing (expected 0)
  reset           two resets in one process with 3 s passive hold each; compares the reset states (reset-result.json)

Usage (in the Isaac container): python demo.py OUT_ROOT --stages demo,one-lead,... [--no-record]
(one Kit process for all listed stages; each stage starts from a fresh env reset)
"""
import argparse
import hashlib
import json
import math
import os
import shutil
import sys
import time
import traceback
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
HZ = 20
GRASP_S, GRASP_H = 0.022, 0.0098  # grasp centre 22 mm behind the insulator face, TCP 9.8 mm above the handle axis
FINGER_OPEN = 0.04695
OPEN_G = 0.012 / FINGER_OPEN  # 12 mm per finger: clear of the 8 mm handle
CLOSE_G = 0.0035 / FINGER_OPEN  # first contact, just inside the handle (no far over-command: two independent finger drives)
SQUEEZE = float(os.environ.get("TASK1_SQUEEZE", "0.0030"))  # m per finger once both pads touch
RUN_LIMIT = 2400
PIN_R_M = 0.002
STAGES = ("demo", "one-lead", "tip-touch", "case-touch", "reverse-live", "xt60-unplugged", "pull-ac", "smoke", "reset")


class Stop(Exception):
    pass


class Arm:
    """IK per control step plus an integral term on the measured joints."""

    def __init__(self, side):
        from yam_arena import station
        from yam_arena.constants import READY
        self.side, self.base = side, station.BASES[side]
        self.q = np.array(READY, float)
        self.meas = self.q.copy()
        self.integ = np.zeros(6)
        self.grip = 1.0
        self.cmd = np.r_[self.q, 1.0]

    def solve(self, p, r):
        from yam_arena import ik
        q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), q_init=self.q, base_pos=self.base, iters=200)
        if pe > 5e-4 or re > 5e-3:
            q2, pe2, re2 = ik.solve(p, ik.matrix_to_quat_xyzw(r), q_init=self.q, base_pos=self.base, iters=600)
            if pe2 + re2 < pe + re:
                q, pe, re = q2, pe2, re2
        return q, pe, re

    def command(self, q, grip, ki=0.15):
        self.q = np.asarray(q, float)
        self.integ = np.clip(self.integ + ki * (self.q - self.meas), -0.08, 0.08)
        self.grip = float(grip)
        self.cmd = np.r_[self.q + self.integ, self.grip]

    def tcp(self):
        from yam_arena import ik
        return ik.tcp_pose(self.q, self.base)


def turn_onto(r, axis):
    """Rotation taking r's X axis onto ``axis`` with the smallest turn (roll kept)."""
    from scipy.spatial.transform import Rotation as R
    v = np.cross(r[:, 0], axis)
    c = float(np.clip(r[:, 0] @ axis, -1, 1))
    if np.linalg.norm(v) < 1e-9:
        return r
    return R.from_rotvec(v / np.linalg.norm(v) * math.acos(c)).as_matrix() @ r


class Runner:
    """One Kit process; ``start(out, stage)`` resets the embodiment and records one run into ``out``."""

    def __init__(self, record, variant=None):
        self.record = record
        if variant:
            os.environ["TASK1_VARIANT"] = variant
        sys.path.insert(0, str(HERE))
        from inspect_robots import Scene
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        import livekit
        self.lk = livekit
        t0 = time.time()
        self.emb = yam_arena_embodiment(task=str(HERE / "task.py"), object_jitter=0, control_hz=HZ, cam_width=640,
                                        cam_height=352)
        print(f"embodiment created in {time.time() - t0:.0f}s", flush=True)
        self.Scene = Scene
        self.runs = 0

    def start(self, out, stage):
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.stage = stage
        self.wall0 = time.time()
        self.events = []
        self.frames = {}
        self.actions = []  # the exact 14-D action sent each control step (replayed by official_replay.py)
        self.ik_max = [0.0, 0.0]
        self.snapshot_source()
        self.reset()
        self.events.append(dict(event="process_run_index", index=self.runs))
        self.runs += 1

    # ------------------------------------------------------------------ plumbing
    def snapshot_source(self):
        src = self.out / "source"
        src.mkdir(exist_ok=True)
        hashes = {}
        for f in sorted(HERE.glob("*.py")):
            shutil.copy2(f, src / f.name)
            hashes[f.name] = hashlib.sha256(f.read_bytes()).hexdigest()
        self.source_hash = hashlib.sha256(json.dumps(hashes, sort_keys=True).encode()).hexdigest()
        (self.out / "source-sha256.json").write_text(json.dumps(dict(files=hashes, combined=self.source_hash), indent=2))

    def reset(self):
        t0 = time.time()
        obs = self.emb.reset(self.Scene(id=f"task1-{self.stage}", instruction=self.instruction()))
        self.events.append(dict(event="reset", wall_s=round(time.time() - t0, 1)))
        print(f"reset done in {time.time() - t0:.0f}s physics_dt={self.emb._base.physics_dt}", flush=True)
        self.env = self.emb._env
        self.left, self.right = Arm("left"), Arm("right")
        self.steps = 0
        self.obs = obs
        self.snap(obs)

    def instruction(self):
        import task
        return task.INSTRUCTION

    def poses(self):
        return self.lk.STATE["view"].poses()

    def plug(self, name):
        return self.poses().plug(name)

    def frames_now(self):
        return self.lk.socket_frames(self.poses())

    def snap(self, obs):
        if not self.record:
            return
        for cam, img in obs.images.items():
            self.frames.setdefault(cam, []).append(np.asarray(img))

    def step(self, label):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import STATE_KEY, pack
        if self.steps >= RUN_LIMIT:
            raise Stop(f"run limit {RUN_LIMIT} reached")
        data = pack(self.left.cmd, self.right.cmd)
        self.actions.append(np.asarray(data, float).ravel().tolist())
        res = self.emb.step(Action(data=data))
        self.obs = res.observation
        jp = np.asarray(self.obs.state[STATE_KEY], float).ravel()
        if not np.all(np.isfinite(jp)):
            raise Stop(f"non-finite joint state at step {self.steps} ({label})")
        self.left.meas, self.right.meas = jp[:6], jp[7:13]
        self.steps += 1
        self.label = label
        self.snap(self.obs)
        self.last_info = res.info

    def wait(self, label, seconds):
        for _ in range(max(1, round(seconds * HZ))):
            self.step(label)

    def go(self, arm, label, duration, p1, r1, grip1=None, feedback=None):
        """Cartesian TCP move of one arm (the other holds its command)."""
        from yam_arena import ik
        p0, r0 = arm.tcp()
        g0 = arm.grip
        g1 = g0 if grip1 is None else grip1
        n = max(1, round(duration * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            p = p0 + s * (np.asarray(p1) - p0)
            r = ik.slerp_matrix(r0, r1, s)
            if feedback is not None:
                p = p + feedback()
            q, pe, re = arm.solve(p, r)
            if pe > 0.002 or re > math.radians(3):
                raise Stop(f"{label}: unreachable target ({pe * 1000:.1f} mm, {math.degrees(re):.1f} deg)")
            self.ik_max = [max(self.ik_max[0], pe), max(self.ik_max[1], re)]
            arm.command(q, g0 + s * (g1 - g0))
            self.step(label)
        self.log(label)

    def joint_go(self, arm, label, duration, q1, grip1):
        q0, g0 = arm.q.copy(), arm.grip
        n = max(1, round(duration * HZ))
        from yam_arena import ik
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            arm.command(q0 + s * (np.asarray(q1) - q0), g0 + s * (grip1 - g0))
            self.step(label)
        self.log(label)

    def log(self, label, **extra):
        tr = self.lk.STATE["trace"][-1] if self.lk.STATE["trace"] else {}
        row = dict(step=self.steps, label=label, score=self.lk.STATE["score"], veto=self.lk.STATE["scorer"].veto,
                   live=tr.get("live"), pos=tr.get("positive"), neg=tr.get("negative"), **extra)
        self.events.append(row)
        print(json.dumps(row, default=float), flush=True)

    # ------------------------------------------------------------------ skills
    def grasp_rotation(self, arm, lead, target, jack=None):
        """Top-down grasp across the handle; of the two wrist yaws keep the one with the most joint margin at the grasp
        and (if given) at the pre-insertion pose in front of ``jack``."""
        from yam_arena import ik
        from yam_arena.demo import grip_rotation
        pp, pr = self.plug(lead)
        h = math.atan2(pr[1, 0], pr[0, 0])
        best = None
        for yaw in (h, h + math.pi):
            r = grip_rotation(yaw)
            q, pe, _ = arm.solve(target, r)
            margin = min(np.min(q - ik.LOWER), np.min(ik.UPPER - q))
            if jack is not None:
                sp, sr = self.frames_now()[jack]
                turn = turn_onto(pr, sr[:, 0]) @ pr.T
                pre = sp + sr[:, 0] * (0.030 + GRASP_S) + [0, 0, GRASP_H]
                saved = arm.q
                arm.q = q
                q2, pe2, _ = arm.solve(pre, turn @ r)
                arm.q = saved
                pe = max(pe, pe2)
                margin = min(margin, np.min(q2 - ik.LOWER), np.min(ik.UPPER - q2))
            if pe < 1e-3 and (best is None or margin > best[0]):
                best = (margin, r)
        if best is None:
            raise Stop(f"{lead}: no reachable top-down grasp")
        return best[1]

    def pick(self, lead, arm=None, jack=None):
        arm = arm or self.left
        pp, pr = self.plug(lead)
        target = pp + pr[:, 0] * GRASP_S + [0, 0, GRASP_H]
        rot = self.grasp_rotation(arm, lead, target, jack)
        self.go(arm, f"{lead}_hover", 2.5, target + [0, 0, 0.06], rot, OPEN_G)
        pp, pr = self.plug(lead)  # re-measure before descending
        target = pp + pr[:, 0] * GRASP_S + [0, 0, GRASP_H]
        self.go(arm, f"{lead}_descend", 2.0, target, rot)
        self.go(arm, f"{lead}_close", 1.5, target, rot, CLOSE_G)
        self.go(arm, f"{lead}_squeeze", 2.0, target, rot, SQUEEZE / FINGER_OPEN)
        z0 = self.plug(lead)[0][2]
        self.go(arm, f"{lead}_lift", 1.5, target + [0, 0, 0.05], rot)
        dz = self.plug(lead)[0][2] - z0
        self.events.append(dict(event=f"{lead}_lifted", dz_m=float(dz)))
        if dz < 0.030:
            raise Stop(f"{lead}: plug did not lift 30 mm with the hand ({dz * 1000:.1f} mm)")

    def move_plug(self, arm, lead, label, duration, target_p, target_r, align_frame=None, check=True):
        """Move the GRASPED plug's frame to a target (joint targets only): the hand is planned through the grasp
        measured at the start of the segment; optional bounded lateral feedback on the measured plug pose."""
        from yam_arena import ik
        p0, r0 = self.plug(lead)
        hp0, hr0 = arm.tcp()
        local_p, local_r = hr0.T @ (p0 - hp0), hr0.T @ r0
        corr = np.zeros(3)
        n = max(1, round(duration * HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            refp = p0 + s * (np.asarray(target_p) - p0)
            refr = ik.slerp_matrix(r0, target_r, s)
            hr = refr @ local_r.T
            hp = refp - hr @ local_p
            if align_frame is not None:
                err = align_frame.T @ (refp - self.plug(lead)[0])
                err[0] = 0.0  # no feedback along the insertion axis
                corr = np.clip(corr + 0.25 * err, -0.0015, 0.0015)
                hp = hp + align_frame @ corr
            q, pe, re = arm.solve(hp, hr)
            if pe > 0.002 or re > math.radians(3):
                raise Stop(f"{label}: unreachable target ({pe * 1000:.1f} mm, {math.degrees(re):.1f} deg)")
            if np.max(np.abs(q - arm.q)) > 0.35:
                raise Stop(f"{label}: IK branch jump")
            self.ik_max = [max(self.ik_max[0], pe), max(self.ik_max[1], re)]
            arm.command(q, arm.grip)
            self.step(label)
            pp, pr = self.plug(lead)
            hp_c, hr_c = arm.tcp()
            slip_d = float(np.linalg.norm(hr_c.T @ (pp - hp_c) - local_p))
            slip_a = math.degrees(math.acos(float(np.clip((hr_c @ local_r)[:, 0] @ pr[:, 0], -1, 1))))
            if slip_d > 0.006 or slip_a > 12:
                raise Stop(f"{label}: grasp slipped {slip_d * 1000:.1f} mm / {slip_a:.1f} deg")
        pp, pr = self.plug(lead)
        perr = float(np.linalg.norm(pp - target_p))
        aerr = math.degrees(math.acos(float(np.clip(target_r[:, 0] @ pr[:, 0], -1, 1))))
        self.log(label, plug_err_mm=round(perr * 1000, 2), axis_err_deg=round(aerr, 2))
        if check and (perr > 0.003 or aerr > 5):
            raise Stop(f"{label}: plug waypoint missed {perr * 1000:.2f} mm / {aerr:.1f} deg")

    def insert(self, lead, jack, arm=None, depth=-0.0015, require_seated=True):
        arm = arm or self.left
        sp, sr = self.frames_now()[jack]
        axis = sr[:, 0]
        pp, pr = self.plug(lead)
        desired = turn_onto(pr, axis)
        pre = sp + axis * 0.030
        high = np.array([pre[0], pre[1], max(pp[2], pre[2]) + 0.030])
        self.move_plug(arm, lead, f"{lead}_carry", 3.0, high, desired, check=False)
        self.move_plug(arm, lead, f"{lead}_preinsert", 2.0, pre, desired, align_frame=sr)
        self.move_plug(arm, lead, f"{lead}_align", 1.5, sp + axis * 0.010, desired, align_frame=sr)
        # push 1.5 mm past the jack face (the handle shoulder stops on the face), then hold the push 0.5 s
        self.move_plug(arm, lead, f"{lead}_insert", 3.0, sp + axis * depth, desired, align_frame=sr, check=False)
        self.wait(f"{lead}_settle", 0.5)
        tr = self.lk.STATE["trace"][-1]
        seated = tr[lead] if jack == lead else None
        if require_seated and jack == lead and not seated:
            raise Stop(f"{lead}: not seated after insertion: rel_mm={np.round(np.array(tr[lead + '_rel']) * 1000, 2).tolist()}"
                       f" angle={tr[lead + '_angle']:.1f}")

    def release(self, lead, arm=None, retreat=(0, 0.05, 0.05)):
        arm = arm or self.left
        hp, hr = arm.tcp()
        self.go(arm, f"{lead}_open", 1.0, hp, hr, OPEN_G)
        self.go(arm, f"{lead}_retreat", 1.5, hp + [0, 0, 0.04], hr)
        self.go(arm, f"{lead}_clear", 1.5, hp + np.array(retreat) + [0, 0, 0.04], hr)

    def park(self, arm=None):
        from yam_arena.constants import READY
        arm = arm or self.left
        self.joint_go(arm, "park", 2.5, np.array(READY, float), 1.0)

    def touch_tip(self, lead, point, approach, label, push=0.0006):
        """Negative stimulus: drive the grasped plug's pin nose onto ``point`` along ``approach`` (pin direction)."""
        approach = np.asarray(approach, float) / np.linalg.norm(approach)
        pp, pr = self.plug(lead)
        desired = turn_onto(pr, -approach)  # plug +X (handle) opposite to the pin direction
        tip = np.array([-0.012, 0, 0])

        def origin(gap):
            return point - desired @ tip - approach * gap

        pre = origin(0.025)
        self.move_plug(self.left, lead, f"{label}_carry", 3.0, np.array([pre[0], pre[1], max(pp[2], pre[2]) + 0.03]),
                       desired, check=False)
        side = np.cross([0, 0, 1], approach) if abs(approach[2]) < 0.9 else np.array([1.0, 0, 0])
        side /= np.linalg.norm(side)
        frame = np.column_stack([approach, side, np.cross(approach, side)])
        # Negative stages only: lateral feedback from the pre leg on and no 3 mm waypoint self-check (controller guard,
        # not physics); the level carry lags 2.5-3.5 mm in the grasp.
        self.move_plug(self.left, lead, f"{label}_pre", 2.0, pre, desired, align_frame=frame, check=False)
        self.move_plug(self.left, lead, f"{label}_touch", 2.0, origin(-push), desired, align_frame=frame, check=False)
        self.wait(f"{label}_dwell", 0.5)

    def touch_from_above(self, lead, point, label, push=0.0006):
        """Negative stimulus: lower the grasped, level plug so the underside of its pin (6 mm behind the insulator)
        lands on ``point`` (top surface of the other pin) and presses ``push`` past contact."""
        pp, pr = self.plug(lead)
        desired = turn_onto(pr, np.array([0.0, 1.0, 0.0]))  # handle +Y, pin pointing world -Y (level)

        def origin(gap):
            return point - desired[:, 0] * -0.006 + np.array([0, 0, PIN_R_M + gap])

        pre = origin(0.015)
        self.move_plug(self.left, lead, f"{label}_carry", 3.0, np.array([pre[0], pre[1], max(pp[2], pre[2]) + 0.03]),
                       desired, check=False)
        frame = np.column_stack([[0.0, 0, -1], [1.0, 0, 0], [0, -1.0, 0]])
        # Negative stage only: no 3 mm waypoint self-check here (a controller guard, not physics). The level carry lags
        # 2.5-3.5 mm (seen: 2.9 mm sideways + 2.3 mm sag at the pre pose), so lateral feedback runs from the pre leg
        # on; the black pin offers 12 mm of length to land on and the sag only brings the contact earlier.
        self.move_plug(self.left, lead, f"{label}_pre", 2.0, pre, desired, align_frame=frame, check=False)
        self.move_plug(self.left, lead, f"{label}_touch", 2.0, origin(-push), desired, align_frame=frame, check=False)
        self.wait(f"{label}_dwell", 0.5)

    def pull_ac(self):
        """Negative stimulus: the right arm grips the preset C5 plug by its rectangular strain-relief block (18 x 16 mm,
        16 mm outside the inlet face) and commands a 25 mm pull straight out of the inlet. (Gripping the round 5 mm cord
        where it lies on the mat lost the cord as soon as the hand lifted - no stimulus.)"""
        from yam_arena import ik
        from yam_arena.demo import grip_rotation
        arm = self.right
        pp, pr = self.plug("power")
        out = pr[:, 0] / np.linalg.norm(pr[:, 0])  # inlet axis, pointing out of the charger
        block = pp + out * 0.016
        grip = block + np.array([0, 0, 0.005])  # pad tips (13 mm past the TCP) reach the block's bottom edge
        yaw = math.atan2(out[1], out[0])
        best = None
        for y in (yaw, yaw + math.pi):
            r = grip_rotation(y)
            q, pe, _ = arm.solve(grip, r)
            margin = min(np.min(q - ik.LOWER), np.min(ik.UPPER - q))
            if pe < 1e-3 and (best is None or margin > best[0]):
                best = (margin, r)
        if best is None:
            raise Stop("pull_ac: C5 block not reachable top-down")
        rot = best[1]
        self.go(arm, "ac_hover", 2.5, grip + [0, 0, 0.06], rot, 0.015 / FINGER_OPEN)
        self.go(arm, "ac_descend", 2.0, grip, rot)
        self.go(arm, "ac_close", 1.5, grip, rot, 0.0095 / FINGER_OPEN)
        self.go(arm, "ac_squeeze", 1.5, grip, rot, 0.0078 / FINGER_OPEN)
        self.go(arm, "ac_pull", 3.0, grip + out * 0.025, rot)

    # ------------------------------------------------------------------ stages
    def run(self):
        st = self.stage
        from task_scoring import HOLD_S as hold
        self.wait("settle", 0.5)
        if st == "smoke":
            self.wait("idle", 1.0)
            return
        if st in ("demo", "xt60-unplugged", "one-lead"):
            self.pick("negative", jack="negative")
            self.insert("negative", "negative")
            self.release("negative")
            if st != "one-lead":
                self.pick("positive", jack="positive")
                self.insert("positive", "positive")
                self.release("positive", retreat=(-0.04, 0.06, 0.05))
            self.park()
            self.wait("released_hold", hold + 1.0)
            return
        if st == "tip-touch":
            self.pick("positive")
            pn, rn = self.plug("negative")
            # The black plug lies on the mat with its pin along world -X. The red plug, held level with its pin along
            # world -Y (the carry pose that holds in PhysX), is brought 15 mm above the black pin and lowered so the
            # middle of the red pin lands across the top of the black pin 8 mm ahead of the black insulator.
            # (earlier attempt: 45 deg nose-down approach -> axial slip in the pads; then a level side-on approach
            # at the black pin's axis height put the red handle 0.5 mm above the mat, it dragged on the mat and the
            # hand moved 6 mm relative to the plug before any touch.)
            # press 3 mm past nominal contact: the grasped plug lags ~1.5 mm behind the hand command at this pose
            # (a 0.6 mm press stopped 0.77 mm short; a 2.6 mm press brought the pins to a -0.004 mm gap)
            self.touch_from_above("positive", pn - rn[:, 0] * 0.008 + [0, 0, PIN_R_M], "tip_to_tip", push=0.003)
            self.wait("after_touch", 1.0)
            return
        if st == "case-touch":
            self.pick("positive")
            cp, cr = self.poses().charger()
            self.touch_tip("positive", cp + cr @ np.array([0.072, -0.006, -0.004]), -cr[:, 0], "tip_to_case")
            self.wait("after_touch", 1.0)
            return
        if st == "reverse-live":
            self.pick("positive", jack="negative")
            self.insert("positive", "negative", require_seated=False)
            self.wait("after_reverse", 1.0)
            return
        if st == "pull-ac":
            self.pull_ac()
            self.wait("after_pull", 1.0)
            return
        raise SystemExit(f"unknown stage {st}")

    def reset_check(self):
        """Two resets in the same process, 3 s passive hold at READY each; compare the measured reset states."""
        rows = []
        for k in range(2):
            if k:
                self.reset()
            p0 = self.poses()
            self.wait(f"hold{k}", 3.0)
            p1 = self.poses()
            tr = self.lk.STATE["trace"]
            rows.append(dict(
                reset=k, initial=self.lk.STATE["initial"], initial_retention=self.lk.STATE["initial_retention"],
                scores=sorted(set(t["score"] for t in tr)), vetoes=sorted(set(str(t["veto"]) for t in tr)),
                poses_after_hold={n: dict(pos=p.tolist(), rot=r.tolist()) for n, (p, r) in p1.items.items()},
                drift_during_hold_m={n: float(np.linalg.norm(p1.items[n][0] - p0.items[n][0])) for n in p0.items}))
        a, b = rows
        diff = max(float(np.linalg.norm(np.array(a["poses_after_hold"][n]["pos"]) - np.array(b["poses_after_hold"][n]["pos"])))
                   for n in a["poses_after_hold"])
        ok = all(r["scores"] == [0.0] and r["vetoes"] == ["None"] for r in rows) and all(
            r["initial"][k]["connected"] for r in rows for k in ("power",)) and all(
            r["initial"]["balance"]["seated"] and r["initial"]["xt60"]["seated"] and r["initial"]["xt60"]["live"]
            and not r["initial"]["positive"]["connected"] and not r["initial"]["negative"]["connected"]
            and not r["initial"]["metal_contacts"] for r in rows)
        res = dict(passed=bool(ok), same_process_reset_max_pose_difference_m=diff, resets=rows,
                   note="GPU PhysX; bit-identity is not claimed, the difference is reported")
        (self.out / "reset-result.json").write_text(json.dumps(res, indent=2, default=float))
        print("RESET", json.dumps(dict(passed=ok, max_diff_m=diff)), flush=True)

    def finish(self, err=None):
        lk = self.lk
        summ = lk.summary()
        from task_scoring import LiveLeadScorer
        s0 = LiveLeadScorer()
        if lk.STATE.get("initial") is not None:
            s0.step(1.0 / HZ, lk.STATE["initial"], released=True)
        result = dict(task="charger-task1-live-banana-leads", engine="Isaac Lab-Arena / yam_arena (PhysX)",
                      stage=self.stage, reset_variant=lk.VARIANT, score=summ["score"], veto=summ["veto"],
                      steps=self.steps, run_limit_steps=RUN_LIMIT, within_run_limit=self.steps <= RUN_LIMIT,
                      first_full_score_step=summ["first_full_step"], initial_score=s0.score,
                      min_live_pin_gap_m=summ["min_live_pin_gap_m"], physics_substeps_monitored=summ["substeps"],
                      live_substeps=summ["live_substeps"], physics_dt=summ["physics_dt"], control_hz=HZ,
                      squeeze_m_per_finger=SQUEEZE, ik_error_max=self.ik_max,
                      robot_controls_only=True, source_sha256=self.source_hash, wall_s=round(time.time() - self.wall0, 1),
                      controller_completed=err is None, controller_stop=None if err is None else str(err),
                      final=lk.STATE["trace"][-1] if lk.STATE["trace"] else None, veto_event=summ["veto_event"],
                      initial=summ["initial"], initial_retention=summ["initial_retention"], events=self.events,
                      evidence_scope="Isaac Lab-Arena (yam_arena station) scripted run; not a model run")
        (self.out / "result.json").write_text(json.dumps(result, indent=2, default=float))
        with open(self.out / "trace.jsonl", "w") as f:
            for row in lk.STATE["trace"]:
                f.write(json.dumps(row, default=float) + "\n")
        if self.actions:
            (self.out / "actions.json").write_text(json.dumps(dict(
                control_hz=HZ, packing="inspect_robots_yam.packing.pack(left7, right7)", actions=self.actions)))
        if summ["veto_event"]:
            (self.out / "veto-event.json").write_text(json.dumps(summ["veto_event"], indent=2, default=float))
        if self.frames:
            import imageio
            for cam, fs in self.frames.items():
                imageio.mimsave(self.out / f"{cam}.mp4", fs, fps=HZ, codec="libx264",
                                output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
            top = next((c for c in self.frames if "top" in c), None)
            if top:
                imageio.imwrite(self.out / "setup.png", self.frames[top][0])
                imageio.imwrite(self.out / "final.png", self.frames[top][-1])
        print("RESULT", json.dumps(dict(score=result["score"], veto=result["veto"], steps=self.steps,
                                        first_full=result["first_full_score_step"], initial_score=s0.score,
                                        min_live_gap=result["min_live_pin_gap_m"], stop=result["controller_stop"])),
              flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out", help="output root; each stage writes OUT/<stage>[-<k>]")
    ap.add_argument("--stages", default="demo", help="comma list, run in this order in ONE Kit process")
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    stages = a.stages.split(",")
    bad = [s for s in stages if s not in STAGES]
    if bad:
        raise SystemExit(f"unknown stages {bad}")
    if "xt60-unplugged" in stages and len(set(stages)) > 1:
        raise SystemExit("xt60-unplugged uses the evaluator reset variant (scene built at import): run it alone")
    r = Runner(not a.no_record, "xt60-open" if "xt60-unplugged" in stages else None)
    failures = 0
    seen = {}
    for st in stages:
        seen[st] = seen.get(st, 0) + 1
        out = Path(a.out) / (st if stages.count(st) == 1 else f"{st}-{seen[st]}")
        err = None
        try:
            r.start(out, st)
            if st == "reset":
                r.reset_check()
            else:
                r.run()
        except Exception as e:  # noqa: BLE001  (recorded; the run's score is still the measured one)
            err = e
            failures += 1
            out.mkdir(parents=True, exist_ok=True)
            (out / "exception.txt").write_text(traceback.format_exc())
            traceback.print_exc()
        finally:
            if hasattr(r, "out") and r.out == out:
                r.finish(err)
    os._exit(1 if failures else 0)  # Kit shutdown can hang; results are already written


if __name__ == "__main__":
    main()
