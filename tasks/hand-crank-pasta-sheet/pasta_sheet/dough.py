"""The thin strip joints express plastic rolling of mechanically loaded stock."""

from __future__ import annotations

from collections import deque

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class DoughSheet:
    def __init__(self, asset, sim_view, torch, wp, device, from_sim):
        from yam_arena.robot import asset_link_paths

        self.asset = asset
        self.view = asset.root_view
        self.array = from_sim
        self.lower = [asset.body_names.index(f"Lower{k:02}") for k in range(C.SECTIONS)]
        self.upper = [asset.body_names.index(f"Upper{k:02}") for k in range(C.SECTIONS)]
        self.thickness_joints = [
            asset.joint_names.index(f"Thickness{k:02}") for k in range(C.SECTIONS)
        ]
        self.length_joints = [
            asset.joint_names.index(f"Stretch{k:02}") for k in range(1, C.SECTIONS)
        ]
        self.joint_targets = torch.zeros(
            (1, asset.num_joints), dtype=torch.float32, device=device
        )
        self.joint_targets_wp = wp.from_torch(self.joint_targets)
        prefix = "/World/envs/env_0"
        faces = [
            f"{prefix}/dough/{side}{k:02}"
            for k in range(C.SECTIONS)
            for side in ("Lower", "Upper")
        ]
        self.contact = [
            sim_view.create_rigid_contact_view(
                f"{prefix}/machine/{side}Shaft",
                filter_patterns=faces,
                max_contact_data_count=384,
            )
            for side in ("Driven", "Opposed")
        ]
        link_paths = asset_link_paths()
        hands = [
            f"{prefix}/{arm}{link_paths[link]}"
            for arm in ("LeftArm", "RightArm")
            for link in ("gripper", "tip_left", "tip_right")
        ]
        self.hand_contacts = sim_view.create_rigid_contact_view(
            hands, filter_patterns=[faces] * len(hands), max_contact_data_count=512
        )
        if (
            self.hand_contacts.sensor_count != len(hands)
            or self.hand_contacts.filter_count != len(faces)
        ):
            raise RuntimeError("Both grippers must be available for release scoring")
        self.reset()

    def reset(self):
        self.joint_targets.zero_()
        self.plastic = np.zeros(C.SECTIONS)
        self.peak_pair = np.zeros(C.SECTIONS)
        self.peak_rolls = np.zeros((2, C.SECTIONS))
        self.loaded_draw = np.zeros(C.SECTIONS)
        self.released_ticks = deque(maxlen=8)
        self.hand_force_peak = 0.0
        self.contact_force = np.zeros((2, C.SECTIONS))
        self.contact_average = np.zeros((2, C.SECTIONS))
        self.net_force = np.zeros(2)

    def state(self):
        links = self.array(self.view.get_link_transforms())[0, self.lower]
        j = self.array(self.view.get_dof_positions())[0]
        return (
            links,
            C.STOCK_THICKNESS + j[self.thickness_joints],
            j[self.length_joints],
        )

    def measure_contacts(self, dt):
        hand_forces = self.array(self.hand_contacts.get_contact_force_matrix(dt=dt))
        hand_peak = float(np.linalg.norm(hand_forces, axis=-1).max())
        self.hand_force_peak = max(self.hand_force_peak, hand_peak)
        links, _, _ = self.state()
        if not (abs(links[:, 2] - C.NIP[2]) < 0.029).any():
            self.contact_force.fill(0.0)
            self.contact_average *= np.exp(-dt / 0.024)
            return
        for i, view in enumerate(self.contact):
            patches = self.array(view.get_contact_force_matrix(dt=dt)).reshape(
                C.SECTIONS, 2, 3
            )
            self.contact_force[i] = np.linalg.norm(patches.sum(axis=1), axis=1)
            total = self.array(view.get_net_contact_forces(dt=dt)).reshape(-1, 3)
            self.net_force[i] = max(
                self.net_force[i], np.linalg.norm(total, axis=1).max()
            )
        self.peak_rolls = np.maximum(self.peak_rolls, self.contact_force)
        keep = np.exp(-dt / 0.024)
        self.contact_average = (
            keep * self.contact_average + (1.0 - keep) * self.contact_force
        )
        self.peak_pair = np.maximum(self.peak_pair, self.contact_average.min(axis=0))

    def deform(self, surface_speed, dt):
        """Resolve yield only under simultaneous actual contacts with both rollers."""
        if surface_speed < 0.002 or self.contact_average.max() < 0.35:
            return
        links, thickness, _ = self.state()
        in_nip = (abs(links[:, 2] - C.NIP[2]) < 0.019) & (
            abs(links[:, 0] - C.NIP[0]) < 0.014
        )
        opposed_force = self.contact_average.min(axis=0)
        loaded = (
            in_nip
            & (surface_speed > 0.002)
            & (opposed_force >= 0.35)
            & (thickness < C.STOCK_THICKNESS - C.DOUGH_YIELD)
        )
        if not loaded.any():
            return
        self.loaded_draw[loaded] += dt * surface_speed
        # Unloading leaves only a small elastic recovery. Adjacent sections
        # lengthen in inverse proportion to thickness at fixed width.
        permanent = np.clip(
            C.STOCK_THICKNESS - thickness - 0.00007,
            0.0,
            C.STOCK_THICKNESS - C.GAP + 0.00002,
        )
        desired = np.maximum(self.plastic, permanent)
        self.plastic[loaded] += np.minimum(
            desired[loaded] - self.plastic[loaded], 0.018 * dt
        )
        local_stretch = C.SECTION_PITCH * (
            C.STOCK_THICKNESS / (C.STOCK_THICKNESS - self.plastic) - 1.0
        )
        for i, j in enumerate(self.thickness_joints):
            self.joint_targets[0, j] = -float(self.plastic[i])
        for i, j in enumerate(self.length_joints):
            self.joint_targets[0, j] = float(
                (local_stretch[i] + local_stretch[i + 1]) * 0.5
            )
        self.view.set_dof_position_targets(
            self.joint_targets_wp, self.asset._ALL_INDICES
        )

    def cross_sections(self, links, thickness):
        axes = np.array([quat_xyzw_to_matrix(q) for q in links[:, 3:]])
        center = (
            links[:, :3]
            + (thickness - C.STOCK_THICKNESS)[:, None] * 0.5 * axes[:, :, 2]
        )
        sections = []
        for i in range(C.SECTIONS + 1):
            start, stop = max(0, i - 1), min(C.SECTIONS, i + 1)
            basis = axes[start:stop].mean(axis=0)
            point = center[start:stop].mean(axis=0)
            if i in (0, C.SECTIONS):
                stretch = C.STOCK_THICKNESS / (C.STOCK_THICKNESS - self.plastic[start])
                point += (
                    (-1 if i == 0 else 1)
                    * 0.5
                    * C.SECTION_PITCH
                    * stretch
                    * basis[:, 0]
                )
            wide = 0.5 * C.STOCK_WIDTH * basis[:, 1]
            height = 0.5 * thickness[start:stop].mean() * basis[:, 2]
            sections.append(
                (
                    point - wide - height,
                    point + wide - height,
                    point + wide + height,
                    point - wide + height,
                )
            )
        return np.array(sections)

    def show(self, stage, links, thickness):
        from pxr import Gf, UsdGeom

        name = "/World/envs/env_0/PastaSurface"
        fresh = not stage.GetPrimAtPath(name).IsValid()
        mesh = UsdGeom.Mesh.Define(stage, name)
        if fresh:
            faces = [(3, 2, 1, 0)]
            for band in range(C.SECTIONS):
                a, b = 4 * band, 4 * (band + 1)
                for side in range(4):
                    nxt = (side + 1) % 4
                    faces.append((a + side, a + nxt, b + nxt, b + side))
            faces.append(tuple(4 * C.SECTIONS + k for k in range(4)))
            mesh.CreateFaceVertexCountsAttr([4] * len(faces))
            mesh.CreateFaceVertexIndicesAttr([idx for face in faces for idx in face])
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDisplayColorAttr([Gf.Vec3f(0.96, 0.67, 0.23)])
            mesh.CreateDoubleSidedAttr(True)
        points = self.cross_sections(links, thickness).reshape(-1, 3)
        mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float, row)) for row in points])

    def evaluate(self, dt):
        links, thickness, stretch = self.state()
        corners = self.cross_sections(links, thickness)
        center = corners.mean(axis=1)
        section_ends = np.cumsum(np.linalg.norm(np.diff(center, axis=0), axis=1))
        arc = float(section_ends[-1])
        sample_distances = arc * (np.arange(C.SECTIONS) + 0.5) / C.SECTIONS
        samples = thickness[np.searchsorted(section_ends, sample_distances)]
        valid_thickness = (samples >= 0.00345) & (samples <= 0.00415)
        tray_points = (corners - C.CATCH_TOP) @ C.CATCH_FRAME
        normal = tray_points[:, :, 2]
        flat = (
            float(normal.max() - normal.min()) < 0.013
            and -0.005 < float(normal.mean()) < 0.010
        )
        on_board = (
            np.abs(tray_points[:, :, 0]).max() < 0.5 * C.CATCH_LENGTH + 0.002
            and np.abs(tray_points[:, :, 1]).max() < 0.5 * C.CATCH_WIDTH + 0.002
        )
        straight = np.linalg.norm(center[-1] - center[0]) >= 0.90 * arc
        filled = int(np.count_nonzero(valid_thickness))
        hand_force = self.hand_force_peak
        self.hand_force_peak = 0.0
        released = hand_force < 0.02
        if not (flat and on_board and straight and released and filled):
            self.released_ticks.clear()
        elif dt > 0.0:
            # A short sheet can show thinning progress, but never earns full credit.
            self.released_ticks.append(min(filled, C.SECTIONS - int(arc < 0.107)))
        score = (
            min(self.released_ticks) / C.SECTIONS
            if len(self.released_ticks) == self.released_ticks.maxlen
            else 0.0
        )
        return {
            "score": score,
            "success": score == 1.0,
            "section_positions": links[:, :3].tolist(),
            "thickness_mm": (1000.0 * thickness).tolist(),
            "sampled_thickness_mm": (1000.0 * samples).tolist(),
            "length_mm": 1000.0 * arc,
            "stretch_mm": (1000.0 * stretch).tolist(),
            "flat": bool(flat),
            "on_board": bool(on_board),
            "straight": bool(straight),
            "released": released,
            "peak_gripper_contact_n": hand_force,
            "thickness_checks_in_range": filled,
            "released_hold_s": len(self.released_ticks) * 0.05,
            "peak_opposed_contact_n": self.peak_pair.tolist(),
            "peak_separate_roll_contact_n": self.peak_rolls.tolist(),
            "peak_roll_net_n": self.net_force.tolist(),
            "loaded_surface_travel_mm": (1000.0 * self.loaded_draw).tolist(),
            "permanent_reduction_mm": (1000.0 * self.plastic).tolist(),
        }
