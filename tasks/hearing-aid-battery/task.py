"""On the station table, on a grey silicone mat: a behind-the-ear hearing aid lying on its side with its battery door shut
on a dead size 13 zinc-air cell; the cells' dial pack, turned to its open pocket, holding a new cell with its orange
pull tab folded up; a battery magnet standing in its holder; a 30 mL medicine cup for used cells.

The task: holding the aid with one hand and handling the cells only with the magnet, open the door, put the old cell in
the cup, peel the new cell's tab and put it in the cup, seat the new cell + side to the door's + mark, click the door shut,
and let go. Scored by rules.py, following task.json's `scoring`.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # lets this file import helpers beside it

import bodykit as bk  # noqa: E402
import parts as P  # noqa: E402
import rules  # noqa: E402
import steptime  # noqa: E402

bk.install()

SCENE = "station"
SUCCESS_HOLD_S = 0.05  # the 1 s hold of the full state is checked by the scorer itself (rules.HOLD_S)
MODE = os.environ.get("TASK_DEMO", "full")
FLIP = MODE.removeprefix("dev") == "flip"  # a scorer check: the new cell starts + side down, its tab already in the cup
DEV_OPEN = MODE == "devopen"  # development only: the door swings itself open in the first 2 s, to work on the later steps
OBJECTS = P.bodies(flip=FLIP)
NAMES = tuple(o.name for o in OBJECTS)

# -- the door's detent: held shut until pushed 3 deg open; a ramp over the last 6 deg; clicks shut under 2 deg ----------
K_LATCH, K_RAMP, DAMP = 0.0026, 0.002, 0.00002  # N m per degree; N m s per degree
RAMP_DEG, OPEN_DEG, CLICK_DEG = 6.0, 3.0, 2.0
K_FRICTION, FRICTION_SLIP = 0.0015, 2.0  # N m per degree, degrees: about 0.3 N at the tab moves the open door
DAMP_FREE = 0.0002  # N m s per degree: the pin's friction holds the door where it is put, against light knocks

# -- the magnet: pull on a cell toward the tip's face ---------------------------------------------------------------


def _geometry() -> rules.Geometry:
    g = np.mgrid[-1:1.01:0.5, -1:1.01:0.25, 0:1.01:0.5].reshape(3, -1).T
    aid_pts = (g * np.array([P.AID_D / 2, P.AID_L / 2, P.AID_T])) @ P.Q.T
    door_pts = np.array([[x, y, z] for x in (-P.AID_D, -P.AID_D / 2, 0.0) for y in (0.0, 0.006, 0.012) for z in (0.0, 0.006, 0.012)]) @ P.Q.T
    stick_pts = np.array([[0, 0, z] for z in np.linspace(0, P.STICK_L, 12)])
    return rules.Geometry(z0=P.Z0, top=P.TOP, mat_c=P.MAT_C, mat_size=P.MAT_SIZE, cup_xy=P.CUP_XY, cup_r=P.CUP_R, cup_h=P.CUP_H,
                          cradle_rel=tuple(P.CRADLE_REL.tolist()), cell_r=P.CELL_R, cell_body_h=P.CELL_BODY_H, cap_h=P.CAP_H,
                          tab_rel=(0.0, 0.0, P.CELL_BODY_H / 2 + 0.15 * P.MM), aid_pts=aid_pts, door_pts=door_pts, stick_pts=stick_pts)


GEOM = _geometry()


def _np(x):
    return bk.as_numpy(x)




MAG_HOLD = 0.25  # N: the most the magnet holds a cell with (pull-off or slide-off force); about 30 times a real cell's weight
MAG_GRAB = 1.5 * P.MM  # a cell face this close under the tip's face (and mostly under it) is taken
MAG_OVERLAP = 0.6  # the share of the tip's face that must be over the cell's face to take it
MAG_W = 150.0  # rad/s: how stiffly a held cell is kept against the tip's face (stable at the 240 Hz physics step)
MAG_W_LAT = 150.0  # rad/s: and across it (friction on the face), up to MAG_SLIDE
MAG_SLIDE = 0.15  # N: the most the face holds a cell across it (friction on the face); more slides it off
MAG_SLIP = 4.0 * P.MM  # a held cell this far from its place has come off


def magnet_contact(cp, cR, tip, mR):
    """(gap, overlap, attracted face's outward normal) for the cell face that the tip faces best, or None. The gap is from
    the tip face's nearest edge; overlap is the share of the 5 mm tip face over the cell's face."""
    axis = mR[:, 2]
    u = cR[:, 2]
    best = None
    for face, r, n in ((cp + u * P.CELL_BODY_H / 2, P.CELL_R, u), (cp - u * (P.CELL_BODY_H / 2 + P.CAP_H), P.CAP_R, -u)):
        c = float(np.dot(n, -axis))
        if c > -0.5:  # the tip must face the face, within 60 deg
            continue
        v = tip - face
        along = float(np.dot(v, n))
        gap = along - P.MAG_R * math.sqrt(max(0.0, 1.0 - c * c))
        lat = float(np.linalg.norm(v - along * n))
        overlap = float(np.clip((r + P.MAG_R - lat) / (2 * P.MAG_R), 0.0, 1.0))
        if best is None or gap < best[0]:
            best = (gap, overlap, n)
    return best


class MagnetHold:
    """One cell and the magnet: free until its face comes within MAG_GRAB under the tip's face (mostly under it); then held
    at that place on the tip (face to face) by a force (and torque) that keeps it there, but never more than MAG_HOLD.
    When keeping it there needs more (a wall stops the cell while the tip slides on, or the cell is pulled off), it is let
    go, and taken again only once the tip has left it (gap over 3 mm or face clear)."""

    def __init__(self, mass: float, inertia: float):
        self.m, self.I = mass, inertia
        self.held = False
        self.locked = False  # just let go: not taken again until the tip has left it
        self.rel_p = None  # the cell's centre in the tip's frame while held
        self.rel_R = None
        self.over = 0
        self.why = ""

    def step(self, cp, cR, cv, cw, tip, mR, mv, mw, g=9.81):
        """Force and torque (world) on the cell for this physics step."""
        con = magnet_contact(cp, cR, tip, mR)
        if not self.held:
            if con is None or con[0] > 3.0 * P.MM or con[1] <= 0.0:
                self.locked = False
            if not self.locked and con is not None and con[0] < MAG_GRAB and con[1] >= MAG_OVERLAP:
                gap, _, n = con
                self.held = True
                self.over = 0
                target = cp + n * max(gap, 0.0)  # face to face on the tip
                self.rel_p = mR.T @ (target - tip)
                self.rel_R = mR.T @ cR
            else:
                return np.zeros(3), np.zeros(3)
        want_p = tip + mR @ self.rel_p
        want_v = mv + np.cross(mw, want_p - tip)
        ax = mR[:, 2]
        dp, dv = want_p - cp, want_v - cv
        dp_ax, dv_ax = float(np.dot(dp, ax)) * ax, float(np.dot(dv, ax)) * ax
        f = (self.m * MAG_W ** 2 * dp_ax + 2 * self.m * MAG_W * dv_ax + self.m * MAG_W_LAT ** 2 * (dp - dp_ax)
             + 2 * self.m * MAG_W_LAT * (dv - dv_ax) + np.array([0.0, 0.0, self.m * g]))
        Rw = (mR @ self.rel_R) @ cR.T
        ang = np.array([Rw[2, 1] - Rw[1, 2], Rw[0, 2] - Rw[2, 0], Rw[1, 0] - Rw[0, 1]]) / 2.0
        tq = self.I * MAG_W ** 2 * ang + 2 * self.I * MAG_W * (mw - cw)
        f_lat = f - float(np.dot(f, ax)) * ax
        if np.linalg.norm(f) > MAG_HOLD or np.linalg.norm(f_lat) > MAG_SLIDE or np.linalg.norm(want_p - cp) > MAG_SLIP:
            self.over += 1
            if self.over >= 3:
                self.held, self.locked = False, True
                self.why = f"let go: force {np.linalg.norm(f):.3f} N, across {np.linalg.norm(f_lat):.3f} N, off {np.linalg.norm(want_p - cp) * 1000:.1f} mm"
                return np.zeros(3), np.zeros(3)
            f = f * (MAG_HOLD / max(np.linalg.norm(f), MAG_HOLD))
        else:
            self.over = 0
        return f, tq


class Trial:
    def __init__(self):
        self.reset()

    def reset(self) -> None:
        self.judge = rules.Judge(GEOM)
        self.t = 0.0
        self.latched = True
        self.rest = None
        self.drive = None
        self.tier, self.why = 0.0, "nothing done"
        self.shown = None
        self.mag = {}
        self.mag_t = {}
        self.held_shown = {}
        self.holds = {n: MagnetHold(P.CELL_MASS, P.CELL_MASS * (3 * P.CELL_R ** 2 + P.CELL_BODY_H ** 2) / 12) for n in ("old_cell", "new_cell")}
        self.log_k = 0
        if not hasattr(self, "hooked"):
            self.hooked = None

    # -- reading the scene ----------------------------------------------------------------------------------------
    def poses(self, env) -> dict:
        from yam_arena.scene import object_states

        names = [n for n in ("aid", "door", "old_cell", "new_cell", "tab", "magnet") if n in NAMES]
        st = object_states(env, names)
        out = {n: rules.Pose(np.asarray(st[n].pos, float), bk.rotation(st[n].quat_xyzw)) for n in names}
        out.setdefault("tab", None)
        return out

    def boxes(self, env) -> list:
        import gripgeo
        from yam_arena.constants import TCP_OFFSET
        from yam_arena.runtime import link_poses

        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        out = []
        for side in ("left", "right"):
            robot = base.scene[f"{side}_robot"]
            gp = link_poses(robot)["gripper"]
            R = gp[:3, :3].copy()
            t = R @ np.array(TCP_OFFSET) + gp[:3, 3] - origin
            names = list(robot.joint_names)
            ap = 2.0 * abs(float(_np(robot.data.joint_pos)[0, names.index("joint7")]))
            for (link, _k), lo, hi in gripgeo.tcp_boxes(ap):
                out.append(rules.Box(R, t, np.asarray(lo), np.asarray(hi), link != "gripper"))
        return out

    @staticmethod
    def door_deg(pose: dict) -> float:
        r = pose["aid"].R.T @ pose["door"].R
        return math.degrees(math.atan2(r[1, 0], r[0, 0]))

    # -- the door's detent -------------------------------------------------------------------------------------------
    def detent(self, deg: float) -> None:
        if self.latched and deg > OPEN_DEG:
            self.latched = False
        elif not self.latched and deg < CLICK_DEG:
            self.latched = True
        if DEV_OPEN and self.t < 2.0:
            self.latched = False
            want = (95.0, 0.01, DAMP * 20)
        elif self.latched:
            want = (0.0, K_LATCH, DAMP)
        elif deg < RAMP_DEG:
            want = (RAMP_DEG, K_RAMP, DAMP)
        else:  # free: the pin's friction holds it where it was left, until pushed past FRICTION_SLIP
            if self.rest is None:
                self.rest = deg
            elif deg - self.rest > FRICTION_SLIP:  # pushed on: the rest angle trails it, so the pull back never exceeds
                self.rest = deg - FRICTION_SLIP  # K_FRICTION * FRICTION_SLIP (Coulomb friction)
            elif self.rest - deg > FRICTION_SLIP:
                self.rest = deg + FRICTION_SLIP
            want = (round(self.rest, 1), K_FRICTION, DAMP_FREE)
        if not (not self.latched and deg >= RAMP_DEG):
            self.rest = None
        if want != self.drive:
            bk.set_hinge_drive("door_pin", *want)
            self.drive = want

    # -- the magnet ----------------------------------------------------------------------------------------------------
    def install_magnet(self, env) -> None:
        """The magnet's hold (MagnetHold), set at every physics step (240 Hz) from the bodies' states at that step: hooked
        on each cell's write to the simulator, as a native PhysX external force and torque; the same acts back on the
        magnet."""
        if self.hooked is env:
            return
        self.hooked = env
        scene = env.unwrapped.scene
        mag = scene["magnet"]
        for name in ("old_cell", "new_cell"):
            obj = scene[name]
            write = obj.write_data_to_sim

            def write_with_hold(obj=obj, write=write, name=name):
                import torch

                c = _np(obj.data.root_link_pose_w)[0]
                m = _np(mag.data.root_link_pose_w)[0]
                vc = _np(obj.data.root_com_vel_w)[0]
                vm = _np(mag.data.root_com_vel_w)[0]
                f, tq = self.holds[name].step(c[:3], bk.rotation(c[3:7]), vc[:3], vc[3:6], m[:3], bk.rotation(m[3:7]), vm[:3], vm[3:6])
                self.mag[name] = f
                self.mag_t[name] = tq
                ft = torch.tensor(f, dtype=torch.float32, device=obj.device).view(1, 1, 3)
                tt = torch.tensor(tq, dtype=torch.float32, device=obj.device).view(1, 1, 3)
                obj.permanent_wrench_composer.set_forces_and_torques_index(forces=ft, torques=tt, is_global=True)
                write()

            obj.write_data_to_sim = write_with_hold
        mwrite = mag.write_data_to_sim

        def mag_write():
            import torch

            f = -(self.mag.get("old_cell", np.zeros(3)) + self.mag.get("new_cell", np.zeros(3)))
            t = torch.tensor(f, dtype=torch.float32, device=mag.device).view(1, 1, 3)
            mag.permanent_wrench_composer.set_forces_and_torques_index(forces=t, torques=torch.zeros_like(t), is_global=True)
            mwrite()

        mag.write_data_to_sim = mag_write

    # -- each control step ------------------------------------------------------------------------------------------
    def step(self, env, dt: float) -> None:
        self.t += dt
        pose = self.poses(env)
        deg = self.door_deg(pose)
        self.detent(deg)
        self.install_magnet(env)
        snap = rules.Snapshot(self.t, pose, deg, self.latched, self.boxes(env))
        j = self.judge
        j.update(snap)
        self.tier, self.why = j.tier(snap)
        if self.tier != self.shown:
            self.shown = self.tier
            print(f"[task] t={self.t:.2f} tier {self.tier}: {self.why}; door {deg:.1f} deg latched {self.latched}; tab_off {j.tab_off}; "
                  f"opened {j.opened}; touched {j.touched}; dropped {j.dropped}", flush=True)
        if os.environ.get("TASK_DEBUG"):
            for n, h in self.holds.items():
                if h.held != self.held_shown.get(n):
                    self.held_shown[n] = h.held
                    print(f"[hold] t={self.t:.2f} {n} {'taken' if h.held else h.why or 'free'}", flush=True)
            from yam_arena.scene import object_states

            st = object_states(env, ("old_cell", "new_cell"))
            for n in ("old_cell", "new_cell"):
                sp = float(np.linalg.norm(st[n].lin_vel))
                if sp > 0.15:
                    print(f"[fast] t={self.t:.2f} {n} speed {sp:.2f} m/s at {np.round(np.asarray(st[n].pos) * 1000, 1).tolist()} "
                          f"force {np.round(self.mag.get(n, np.zeros(3)), 3).tolist()} tip {np.round(pose['magnet'].p * 1000, 1).tolist()}", flush=True)
        if os.environ.get("TASK_DEBUG") and self.log_k % 20 == 0:
            mf = {k: round(float(np.linalg.norm(v)), 3) for k, v in self.mag.items()}
            print(f"[dbg] t={self.t:.2f} door {deg:.1f} latched {self.latched} mag {mf} old {np.round(pose['old_cell'].p * 1000, 1).tolist()} "
                  f"new {np.round(pose['new_cell'].p * 1000, 1).tolist()} tip {np.round(pose['magnet'].p * 1000, 1).tolist()} "
                  f"aid {np.round(pose['aid'].p * 1000, 1).tolist()} clear {j.clear_since is not None}", flush=True)
        self.log_k += 1


TRIAL = Trial()


@bk.on_reset
def _reset() -> None:
    TRIAL.reset()


@bk.on_step
def _step(env) -> None:
    TRIAL.step(env, 1.0 / 20.0)
    if os.environ.get("TASK_CLOSEUP"):  # the demo video's close-up of the door (cam.py); not a policy camera
        import cam

        door = P.aid_world((0.0, -0.020, 0.0))
        cam.start("/World/envs/env_0", np.zeros(3), door + np.array([-0.085, -0.05, 0.095]), door + np.array([0.0, 0.0, 0.004]), focal=32.0)
        cam.step()


def score(objects) -> float:
    """task.json `scoring`: 1.0, 0.5, 0.25, 0.1 or 0 (rules.Judge.tier)."""
    return TRIAL.tier


def success(objects) -> bool:
    steptime.tick()  # the harness calls success() once per control step
    return TRIAL.tier == 1.0


def demo_script(ctx):
    """The scripted reference demo (demo.py): it reads the poses of the parts in view and its own joints."""
    import demo

    yield from demo.script(ctx, MODE)
