"""Embodiment ``yam_inspection_mirror``: the stock bimanual YAM station with a hidden-leak inspection.

Robot, cameras, 14-D joint action and observations are the stock adapter's (``yam_arena``), unchanged. This module
adds the fixed housing with its wall and cover strip, the leak trace on the hidden face (placed at reset at one of
five places, drawn from the episode seed), the Ullman HTC-2LT lighted inspection mirror and the paint marker (free tools), and
up to four paint dots. No joints. Measured each step, from the bodies' poses and the robot's link poses:

- contact with the hidden face (the whole bounded face, exact convex-solid distances): the mirror, the marker or any
  collision piece of the robot touching it
  smears the trace;
- the look: the mirror head in the gap and held still (a pause), and whether a wrist camera sees the trace's middle in
  the mirror's glass, unblocked (reflection geometry from the wrist cameras' mounts and lenses);
- the marker tip: where it touches the cover strip, a paint mark stays (a dot is drawn there);
- put down: each tool still, no part of the robot within 5 mm of it, not on the housing, both grippers open;
- the score (``scoring.LeakMarkScore``).

``-E leak=<0..4>`` fixes the leak place (for checks); by default it comes from the episode seed.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from . import design as d
from .mechanism import MechanismEmbodiment
from .scoring import LeakMarkScore

TASK_FILE = Path(__file__).resolve().parents[1] / "inspection_mirror_task.py"
MARK_GAP = 0.003  # a touch more than 3 mm from every earlier mark makes a new mark (a dragged tip leaves a streak)
CONTACT_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")  # can reach the gap or the tools
VIEW_LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")  # can stand between a wrist camera and the mirror


def robot_geometry():
    """Per link of the YAM arm USD: its collision pieces' hull points and boxes (centre, half size), link frame."""
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from yam_arena import robot

    try:
        from scipy.spatial import ConvexHull
    except Exception:  # noqa: BLE001 - all points then (slower, same answer)
        ConvexHull = None
    stage = Usd.Stage.Open(str(robot.require_usd()))
    xc = UsdGeom.XformCache()
    out = {}
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.GetName() not in CONTACT_LINKS:
            continue
        to_link = xc.GetLocalToWorldTransform(prim).GetInverse()
        pts, boxes = [], []
        it = iter(Usd.PrimRange(prim, Usd.TraverseInstanceProxies()))
        for p in it:
            if p != prim and p.HasAPI(UsdPhysics.RigidBodyAPI):
                it.PruneChildren()
                continue
            if not p.HasAPI(UsdPhysics.CollisionAPI):
                continue
            m = xc.GetLocalToWorldTransform(p) * to_link
            if p.IsA(UsdGeom.Mesh):
                v = np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(), float)
            else:
                lo, hi = (np.array(x, float) for x in UsdGeom.Boundable(p).ComputeExtent(Usd.TimeCode.Default()))
                v = np.array([[a, b, c] for a in (lo[0], hi[0]) for b in (lo[1], hi[1]) for c in (lo[2], hi[2])])
            v = np.array([list(m.Transform(Gf.Vec3d(*x))) for x in v])
            if ConvexHull is not None and len(v) > 8:
                try:
                    v = v[ConvexHull(v).vertices]
                except Exception:  # noqa: BLE001 - a flat piece: keep its points
                    pass
            pts.append(v)
            lo, hi = v.min(0), v.max(0)
            boxes.append(((lo + hi) / 2, (hi - lo) / 2))
        if pts:
            out[prim.GetName()] = (np.unique(np.concatenate(pts).round(5), axis=0), boxes)
    return out


class InspectionMirrorEmbodiment(MechanismEmbodiment):
    ENV_NAME = "yam_inspection_mirror"
    TASK_FILE = TASK_FILE

    def __init__(self, leak: int = -1, **kwargs):
        super().__init__(**kwargs)
        self.fixed_leak = int(leak)
        self.scorer = None
        self.leak_i = d.leak_index(None)
        self._geom = None

    def builders(self):
        from .assets import BUILDERS

        return BUILDERS

    def layout(self):
        return d.start_poses()

    def make_joints(self, js, origin):
        return None  # no joints: the housing is fixed and the mirror and marker are free tools

    def reset(self, scene, *, seed=None):
        self.leak_i = self.fixed_leak if 0 <= self.fixed_leak < len(d.LEAK_X) else d.leak_index(seed)
        self.start = d.start_poses()
        self.start["leak"] = d.leak_pose(self.leak_i)
        print(f"[inspect] reset seed={seed} leak place {self.leak_i} at x={d.LEAK_X[self.leak_i]:.3f}", flush=True)
        return super().reset(scene, seed=seed)

    def on_reset(self):
        self.scorer = LeakMarkScore(self.info.control_hz, d.LEAK_X[self.leak_i])
        self.marks: list[np.ndarray] = []
        self.smeared = False
        self.smeared_by = None
        self.min_clear = 1.0  # the mirror's
        self.min_clear_any = 1.0  # mirror, marker or robot
        self.prev = None
        self.state_cached = {"marks": [], "smeared": False}

    def _place_dot(self, i: int, p) -> None:
        torch, base = self._torch, self._base
        pose = torch.tensor([[float(p[0]), float(p[1]), d.TOP + d.CAP_TOP, 0.0, 0.0, 0.0, 1.0]],
                            dtype=torch.float32, device=base.device)
        pose[:, :3] += base.scene.env_origins
        base.scene[f"dot_{i}"].write_root_pose_to_sim_index(root_pose=pose)

    # -- the robot: link poses, hull points, view-blocking boxes, wrist cameras --------------------------------------
    def _robot(self):
        from yam_arena.inspect_embodiment import ARMS

        if self._geom is None:
            geom = robot_geometry()
            self._geom = {"links": geom, "ids": {}}
            for name in ARMS:
                art = self._base.scene[name]
                self._geom["ids"][name] = {ln: art.find_bodies(ln)[0][0] for ln in (*geom, "gripper")}
        o = self.origin()
        solids, boxes, cams = [], [], []
        for name, side in zip(ARMS, ("left", "right")):
            data = self._base.scene[name].data.body_link_pose_w.torch[0].cpu().numpy().astype(float)
            ids = self._geom["ids"][name]
            for ln, (p_link, link_boxes) in self._geom["links"].items():
                pose = data[ids[ln]].copy()
                pose[:3] -= o
                r = d.rot(pose[3:])
                solids += [(pose[:3] + r @ c, r, h) for c, h in link_boxes]  # every collision piece, whole
                if ln in VIEW_LINKS:
                    boxes += [(pose[:3] + r @ c, r, h) for c, h in link_boxes]
            g = data[ids["gripper"]].copy()
            g[:3] -= o
            cam = self.rig.wrist_cams[side]
            pos, rc = d.camera_pose(g, cam.pos, cam.rot_xyzw, cam.convention)
            lens = cam.lens
            cams.append((pos, rc, (lens.fx, lens.fy, lens.cx, lens.cy, lens.width, lens.height)))
        return solids, boxes, cams

    def measure(self):
        p = self.poses()
        mirror, marker = p["mirror"], p["marker"]
        leak_x = d.LEAK_X[self.leak_i]
        dt = 1.0 / self.info.control_hz
        robot_solids, robot_boxes, cams = self._robot()

        # contact with the hidden face: the mirror, the marker or any part of the robot
        clear = {"mirror": d.mirror_face_clearance(mirror), "marker": d.marker_face_clearance(marker),
                 "robot": d.robot_face_clearance(robot_solids)}
        self.min_clear = min(self.min_clear, clear["mirror"])
        self.min_clear_any = min(self.min_clear_any, *clear.values())
        touch = [k for k, v in clear.items() if v <= d.CONTACT]
        if touch and not self.smeared:
            self.smeared, self.smeared_by = True, touch

        # paint: only where the tip touches the strip
        tip = d.marker_tip(marker)
        if d.on_strip(tip) and all(np.hypot(*(tip[:2] - m[:2])) > MARK_GAP for m in self.marks):
            if len(self.marks) < d.MARK_DOTS:
                self._place_dot(len(self.marks), tip)
            self.marks.append(tip.copy())

        # motion (from the poses one step apart)
        prev = self.prev or p
        head_v, head_w = d.motion(prev["mirror"], mirror, dt, d.HEAD_POINT)
        tool_v = {n: d.motion(prev[n], p[n], dt) for n in ("mirror", "marker")}
        self.prev = p

        # the look
        in_gap = d.head_in_gap(mirror)
        paused = in_gap and head_v < d.REST_SPEED and head_w < d.REST_TURN
        seen = in_gap and d.trace_seen(mirror, leak_x, cams, d.static_occluders() + d.tool_occluders(mirror, marker),
                                       robot_boxes)

        # put down
        near = {"mirror": d.mirror_robot_distance(mirror, robot_solids), "marker": d.marker_robot_distance(marker, robot_solids)}
        opening, _ = self.grippers()
        down = {n: d.tool_down(*tool_v[n], near[n], p[n]) for n in ("mirror", "marker")}
        put_down = all(down.values()) and min(opening) >= 0.8

        marks_x = [float(m[0]) for m in self.marks]
        score = self.scorer.update(marks_x, self.smeared, put_down, paused, seen)
        head = d.head_center(mirror)
        self.state_cached = {"marks": marks_x, "smeared": self.smeared}
        rc = cams[1]
        m_px = d.reflected_point(mirror, rc[0], d.trace_points(leak_x)[0])
        info = {"leak_place": self.leak_i, "leak_x": leak_x, "marks_x": marks_x, "smeared": self.smeared,
                "smeared_by": self.smeared_by, "head": head, "mirror_pose": mirror, "marker_pose": marker,
                "clear_mm": {k: v * 1000.0 for k, v in clear.items()},
                "min_face_clear_mm": self.min_clear * 1000.0, "min_any_clear_mm": self.min_clear_any * 1000.0,
                "head_speed_mm_s": head_v * 1000.0, "head_turn": head_w, "paused": paused, "seen": seen,
                "right_cam": rc[0], "right_cam_R": rc[1], "left_cam": cams[0][0], "left_cam_R": cams[0][1],
                "trace_px_right": None if m_px is None else d.project(rc, m_px),
                "robot_to_tool_mm": {k: v * 1000.0 for k, v in near.items()}, "down": down, "put_down": put_down,
                "marker_tip": tip, "grip_open": opening, "score": score}
        if round(self._t * self.info.control_hz) % 40 == 0:  # every 2 s: the sensor's own pose, to check the mount
            try:
                cam = self._base.scene["right_cam"].data
                info["right_cam_sensor"] = (cam.pos_w[0].cpu().numpy() - self.origin()).tolist()
            except Exception as e:  # noqa: BLE001 - diagnostics only
                info["right_cam_sensor"] = repr(e)
        info.update(self.scorer.info())
        if round(self._t * self.info.control_hz) % round(self.info.control_hz / 2) == 0:
            print(f"[inspect] t={self._t:6.2f} head={np.round(head - [0, 0, d.TOP], 3).tolist()} "
                  f"clear m/k/r={clear['mirror'] * 1000:5.1f}/{clear['marker'] * 1000:5.1f}/{clear['robot'] * 1000:5.1f}mm "
                  f"paused={int(paused)} seen={int(seen)} tip={np.round(tip - [0, 0, d.TOP], 4).tolist()} "
                  f"marks={[round(m, 4) for m in marks_x]} smeared={int(self.smeared)} down={int(put_down)} "
                  f"open={opening[0]:.2f}/{opening[1]:.2f} score={score:.3f} {self.scorer.reason}", flush=True)
        return info, self.scorer.success
