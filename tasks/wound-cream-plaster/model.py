"""Couples the cream-and-plaster model to the simulator once per control step: reads both grippers' measured poses and
jaw openings, the native PhysX contact force of each fingertip on the forearm and on each loose part (the applicator and
the two liner tabs), and those parts' rigid-body poses; steps the model, frees a tab from its root hinge once a hand
holds it,
scores, and redraws the plaster, its liners and the cream."""

from __future__ import annotations

import json
import os

import numpy as np

import bodykit
import firstaid as F
import gripgeo
import rules
import scene as S

DEBUG = os.environ.get("CP_DEBUG") == "1"
TIP_LINKS = ("tip_left", "tip_right")


def _arr(x):
    if hasattr(x, "torch"):
        x = x.torch
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


def set_root_hinge(k, on: bool):
    """Switch tab k's root hinge (props.py) on or off."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdPhysics

    prim = get_current_stage().GetPrimAtPath(bodykit._HINGE_PATHS[f"root_{k}"])
    UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(bool(on))


class Model:
    def __init__(self):
        self.w = F.World()
        self.tracker = rules.Tracker()
        self.render = Render(self.w)
        self.env = None
        self.view = None
        self.origin = np.zeros(3)
        self.last = {}
        self.fmax = {}
        self.pull = {k: np.zeros(3) for k in F.SIDES}  # N: the force last applied to each tab body

    def reset(self):
        if self.view is not None:  # each tab hinged at its root again (rebuilt from the bodies' reset poses)
            for k in F.SIDES:
                set_root_hinge(k, False)
                set_root_hinge(k, True)
        self.w.reset()
        self.tracker.reset()
        self.render.meshes = None

    def attach(self, env):
        self.env = env
        if self.view is not None:
            return
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        base = env.unwrapped
        stage = get_current_stage()
        self.origin = _arr(base.scene.env_origins)[0]
        arm_path = base.scene["forearm"].root_view.prim_paths[0]
        tips = []
        for side in ("left", "right"):
            lp = list(base.scene[f"{side}_robot"].root_view.link_paths[0])
            for nm in TIP_LINKS:
                p = [q for q in lp if q.endswith("/" + nm) and stage.GetPrimAtPath(q).HasAPI(UsdPhysics.RigidBodyAPI)]
                tips.append(p[0] if p else [q for q in lp if q.endswith("/" + nm)][0])
        sim_view = base.scene["forearm"]._physics_sim_view
        self.view = sim_view.create_rigid_contact_view([arm_path], filter_patterns=[tips])
        self.part_views = {}
        for part in F.PARTS:
            path = base.scene[part].root_view.prim_paths[0]
            self.part_views[part] = sim_view.create_rigid_contact_view([path], filter_patterns=[tips])
        self.render.origin = self.origin + np.array([0.0, 0.0, S.TABLE])
        for k in F.SIDES:
            self._install_tether(base.scene[f"tab_{k}"], k)

    def _install_tether(self, obj, k):
        """Each physics step, before the simulator steps, sets the native PhysX external force on tab k's body from the
        model's taut-string pull on it (firstaid.Plaster.tether): read from the body's pose at that physics step, so a
        tab that slips out of the fingers goes slack at once."""
        import torch

        write = obj.write_data_to_sim
        up = self.origin + np.array([0.0, 0.0, S.TABLE])
        state = {"on": False}

        def write_with_pull():
            tt = self.w.pl.tether.get(k)
            if tt is not None or state["on"]:
                force = np.zeros(3)
                if tt is not None:
                    q = _arr(obj.data.root_link_pose_w)[0]
                    p, R = q[:3] - up, bodykit.rotation(q[3:7])
                    off = self.w.pl.tab_off[k]
                    g = p + R @ (off if off is not None and self.w.pl.tab_held[k] is not None else np.array([0.0, F.SIGN[k] * S.TAB_GRIP, 0.0]))
                    force = F.tether_force(tt, g)
                self.pull[k] = force
                t = torch.tensor(force, dtype=torch.float32, device=obj.device).view(1, 1, 3)
                obj.permanent_wrench_composer.set_forces_and_torques_index(forces=t, torques=torch.zeros_like(t), is_global=True)
                state["on"] = tt is not None
            write()

        obj.write_data_to_sim = write_with_pull

    def hands(self):
        from yam_arena.constants import TCP_OFFSET
        from yam_arena.runtime import link_poses

        base = self.env.unwrapped
        f = np.linalg.norm(_arr(self.view.get_contact_force_matrix(1.0 / 240.0))[0], axis=1)  # L tl, L tr, R tl, R tr
        out = {}
        for k, side in enumerate(("left", "right")):
            robot = base.scene[f"{side}_robot"]
            g = link_poses(robot)["gripper"]
            R = g[:3, :3].copy()
            t = R @ np.array(TCP_OFFSET) + g[:3, 3] - self.origin - np.array([0.0, 0.0, S.TABLE])
            names = list(robot.joint_names)
            q7 = float(_arr(robot.data.joint_pos)[0, names.index("joint7")])
            ap = 2.0 * abs(q7)
            force = {nm: float(f[2 * k + j]) for j, nm in enumerate(TIP_LINKS)}
            out[side] = F.Hand(side, R, t, ap, list(gripgeo.tcp_boxes(ap)), force)
        return out

    def parts(self):
        """The loose parts' poses (station frame) and each fingertip's native contact force on each of them."""
        base = self.env.unwrapped
        pose, contact = {}, {"left": {}, "right": {}}
        for part in F.PARTS:
            q = _arr(base.scene[part].data.root_link_pose_w)[0]
            pose[part] = (q[:3] - self.origin - np.array([0.0, 0.0, S.TABLE]), bodykit.rotation(q[3:7]))
            f = np.linalg.norm(_arr(self.part_views[part].get_contact_force_matrix(1.0 / 240.0))[0], axis=1)
            contact["left"][part] = (float(f[0]), float(f[1]))
            contact["right"][part] = (float(f[2]), float(f[3]))
        return {"pose": pose, "contact": contact}

    def step(self):
        try:
            hands = self.hands()
            parts = self.parts()
            self.w.step(hands, parts)
            for k in self.w.pl.unhinge:
                set_root_hinge(k, False)
            self.w.pl.unhinge = []
            self.tracker.update(self.w, hands)
            self.last = {"force": {s: {k: round(v, 2) for k, v in h.force.items()} for s, h in hands.items()},
                         "ap_mm": {s: round(h.ap * 1000, 1) for s, h in hands.items()},
                         "tcp": {s: np.round(h.t, 4).tolist() for s, h in hands.items()},
                         "grip": {s: {p: [round(v, 2) for v in f] for p, f in c.items() if max(f) > 0} for s, c in parts["contact"].items()},
                         "pull": {k: round(float(np.linalg.norm(v)), 3) for k, v in self.pull.items()},
                         "hold_n": round(self.w.pl.hold_n, 2), "drag_n": round(self.w.pl.drag_n, 2), "slid_mm": round(self.w.pl.slid * 1000, 1)}
            self.render.update()
            if DEBUG:
                pk = self.fmax
                for sd, h in hands.items():
                    m = pk.setdefault(sd, [0.0, 0.0])
                    pk[sd] = [round(max(m[0], h.force["tip_left"]), 2), round(max(m[1], h.force["tip_right"]), 2)]
            if DEBUG and self.tracker.steps % 20 == 0:
                print("[cp]", json.dumps(self.summary(), default=lambda o: o.item() if hasattr(o, "item") else str(o)), flush=True)
        except Exception:
            import traceback

            traceback.print_exc()
            raise

    def summary(self):
        tr = self.tracker
        out = dict(step=tr.steps, score=tr.score(), latched=tr.latched, facts=tr.facts, holding=self.w.grips.holding,
                   events=self.w.pl.events[-3:], fmax=self.fmax, **self.last)
        pl = self.w.pl
        out["untacked"] = {}
        for k in F.SIDES:
            m = pl.wing & (pl.U * F.SIGN[k] > 0) & ~pl.tacked
            if 0 < m.sum() < 12:
                out["untacked"][k] = [(round(float(pl.U[i, j]) * 1000, 1), round(float(pl.W[i, j]) * 1000, 1)) for i, j in np.argwhere(m)]
        self.fmax = {}  # the peak fingertip forces on the forearm since the last summary
        return out


# -- drawing ------------------------------------------------------------------------------------------------------------------
# sRGB, converted to linear for the renderer as the station's own materials are. The strip's back is a tan fabric; the
# pad shows on it as a lighter raised panel, so the plaster reads against the skin
COLORS = {"strip": (0.62, 0.44, 0.27), "pad": (0.93, 0.91, 0.85), "liner": (0.97, 0.97, 0.95), "tab": (0.45, 0.62, 0.88),
          "stick": (0.88, 0.76, 0.55), "bud": (0.98, 0.98, 0.97), "cream": (0.42, 0.20, 0.07), "loose": (0.99, 0.99, 0.98)}
PAD_ROWS = np.flatnonzero(np.abs(np.linspace(-S.PL_LEN / 2, S.PL_LEN / 2, F.NU)) <= S.PAD_HALF + 1e-9)
CB = 2  # cream drawn in blocks of 2 x 2 cells (1 mm)


def _grid_faces(R, C, off=0):
    f = []
    for i in range(R - 1):
        for j in range(C - 1):
            a, b, c, d = off + i * C + j, off + (i + 1) * C + j, off + (i + 1) * C + j + 1, off + i * C + j + 1
            f += [(a, b, c), (a, c, d)]
    return f


class Render:
    """USD meshes that follow the model every control step (visual only; nothing collides with them)."""

    def __init__(self, w: F.World):
        self.w = w
        self.meshes = None
        self.origin = np.array([0.0, 0.0, S.TABLE])
        nu, nw = F.NU, F.NW
        self.parts = {"strip": _grid_faces(nu, nw), "pad": _grid_faces(len(PAD_ROWS), nw), "liner_left": _grid_faces(nu, nw), "liner_right": _grid_faces(nu, nw),
                      "tab_left": _grid_faces(2, 3), "tab_right": _grid_faces(2, 3),
                      "loose_left": _grid_faces(2, 3), "loose_right": _grid_faces(2, 3),
                      "cream": None}
        cr = w.cream
        self.cn = (cr.h.shape[0] // CB, cr.h.shape[1] // CB)
        cf = []
        for q in range(self.cn[0] * self.cn[1]):
            a = 4 * q
            cf += [(a, a + 1, a + 2), (a, a + 2, a + 3)]
        self.parts["cream"] = cf
        yb = cr.ys[: self.cn[0] * CB].reshape(self.cn[0], CB).mean(1)
        sb = cr.ss[: self.cn[1] * CB].reshape(self.cn[1], CB).mean(1)
        Y, Sb = np.meshgrid(yb, sb, indexing="ij")
        corners = []
        hc = CB * F.CELL / 2
        for dy, ds in ((-hc, -hc), (hc, -hc), (hc, hc), (-hc, hc)):
            p, nrm = S.ARM.point(Y + dy, Sb + ds)
            corners.append((p, nrm))
        self.cream_base = np.stack([c[0] for c in corners], 2).reshape(-1, 4, 3)  # (blocks, 4, 3)
        self.cream_n = np.stack([c[1] for c in corners], 2).reshape(-1, 4, 3)
        self.cream_key = None

    def points(self):
        w = self.w
        pl = w.pl
        out = {}
        p, n = pl.nodes()
        out["strip"] = (p + n * 0.0003).reshape(-1, 3)
        out["pad"] = (p + n * 0.0006)[PAD_ROWS].reshape(-1, 3)
        for k in F.SIDES:
            ln = pl.liners[k]
            lp = (p - n * (S.PL_T / 2 + 0.0002)).copy()
            sgn = F.SIGN[k]
            mine = pl.u * sgn > 0
            front = ln.p if ln.state == "on" else S.PL_LEN / 2 + 1
            # rows already peeled collapse onto the fold; the other half of the strip shows no liner of this side
            idx = np.arange(F.NU)
            if ln.state != "on":
                lp[:] = -1.0
            else:
                fold = int(np.argmin(np.abs(np.abs(pl.u) - front) + (~mine) * 9))
                for i in idx:
                    if not mine[i] or abs(pl.u[i]) < front:
                        lp[i] = lp[fold]
            out[f"liner_{k}"] = lp.reshape(-1, 3)
            # the tab: from the strip's end to where it is (held or lying)
            e, en = pl.end_point(k)
            if ln.state == "on":
                tp = pl.tab_point(k)
                ew = pl.ew if pl.state != "skin" else S.ARM.tangent_s(pl.place[1])
                ew = np.asarray(ew, float).reshape(3)
                hw = S.PL_WID / 2
                ext = tp + (tp - e) / max(np.linalg.norm(tp - e), 1e-9) * max(S.TAB_OUT - (pl.tab_len[k] if pl.tab_held[k] is not None else S.TAB_GRIP), 0.002)
                out[f"tab_{k}"] = np.array([e - ew * hw, e, e + ew * hw, ext - ew * hw, ext, ext + ew * hw]) - en * 0.0004
            else:
                out[f"tab_{k}"] = np.full((6, 3), -1.0)
            # a loose liner: drawn on its tab body (wherever the body is: in a hand, fallen, in the pot)
            if ln.state in ("held", "dropped", "binned"):
                tp, tR = pl.tab_pose[k]
                ax = F.tab_out(k, tR)
                c = tp + ax * (S.TAB_OUT / 2)
                a, b = ax * (S.TAB_OUT / 2), tR[:, 0] * (S.PL_WID / 2)
                out[f"loose_{k}"] = np.array([c - a - b, c - a, c - a + b, c + a - b, c + a, c + a + b])
            else:
                out[f"loose_{k}"] = np.full((6, 3), -1.0)
        return out

    def cream_points(self):
        cr = self.w.cream
        h = cr.h[: self.cn[0] * CB, : self.cn[1] * CB].reshape(self.cn[0], CB, self.cn[1], CB).mean(axis=(1, 3)).reshape(-1)
        lift = np.where(h >= F.COVER_H * 0.5, 0.00012 + h, -0.002)
        return (self.cream_base + self.cream_n * lift[:, None, None]).reshape(-1, 3)

    def update(self):
        from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt
        import omni.usd
        from yam_arena.station import linear_rgb

        stage = omni.usd.get_context().get_stage()
        if self.meshes is None or not self.meshes["strip"].GetPrim().IsValid():
            self.meshes = {}
            for name, faces in self.parts.items():
                m = UsdGeom.Mesh.Define(stage, f"/World/FirstAid/{name}")
                m.CreateDoubleSidedAttr(True)
                m.CreateSubdivisionSchemeAttr("none")
                m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
                m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(v) for f in faces for v in f]))
                key = name.split("_")[0]
                color = {"liner": "liner", "tab": "tab", "loose": "loose"}.get(key, key)
                mat = UsdShade.Material.Define(stage, f"/World/FirstAid/{name}_look")
                sh = UsdShade.Shader.Define(stage, f"/World/FirstAid/{name}_look/Shader")
                sh.CreateIdAttr("UsdPreviewSurface")
                sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb(COLORS[color])))
                sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.8 if key in ("strip", "pad", "bud") else 0.4)
                mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
                UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
                self.meshes[name] = m
            self.cream_key = None
        pts = self.points()
        for name, p in pts.items():
            q = (np.asarray(p, float) + self.origin).astype(np.float32)
            self.meshes[name].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(q))
        key = (self.w.cream.wipe_steps, self.w.cream.dabs)
        if key != self.cream_key:
            self.cream_key = key
            q = (self.cream_points() + self.origin).astype(np.float32)
            self.meshes["cream"].GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(q))
