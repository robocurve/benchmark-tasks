"""Compliant sheet release and contact-driven lint transfer."""

import math

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

PEEL_FORCE = 1.60
STARTER_FORCE = .58
TAPE_STIFFNESS = 120.
TAPE_DAMPING = .14
FOLD_STIFFNESS = 22.
FOLD_DAMPING = .14
LOAD_REQUIRED = .8
BEARING_DRAG = .00018
NAP_STIFFNESS = 1500.
NAP_HEIGHT = .0014
NAP_FRICTION = .48
NAP_TRACTION = 7.
SHEET = (.91, .80, .56)
DIRT = (.89, .76, .57)


class Tape:
    def __init__(self):
        self.reset()

    def reset(self):
        self.progress = 0.
        self.tension = 0.
        self.peak_tension = 0.
        self.peak_contact = 0.
        self.travel = 0.
        self.previous = None
        self.previous_fold = None
        self.previous_length = None
        self.roller = C.ROLLER.copy()
        self.rotation = np.eye(3)
        self.tab = C.TAB.copy()
        self.tab_rotation = np.eye(3)
        self.front = C.ROLLER + C.SEAM
        self.cloth = C.CLOTH.copy()
        self.cloth_rotation = np.eye(3)
        self.captured = {}
        self.load = 0.

    def radial(self, radians):
        return self.rotation @ np.array([math.sin(radians), 0., math.cos(radians)])

    def advance(self, dt, hub, edge, cloth, angular_velocity):
        self.roller, self.rotation = hub[:3], quat_xyzw_to_matrix(hub[3:])
        self.tab, self.tab_rotation = edge[:3], quat_xyzw_to_matrix(edge[3:])
        self.cloth, self.cloth_rotation = cloth[:3], quat_xyzw_to_matrix(cloth[3:])
        released = self.progress >= C.SHEET_LENGTH
        self.load = min(3., NAP_STIFFNESS * self.nap_compression())
        self.peak_contact = max(self.peak_contact, self.load)
        if released:
            self.tension = 0.
            self.collect(dt, angular_velocity)
            # Gravity acts on the removed flexible sheet through its leading edge.
            return np.array([0., 0., -C.TAB_MASS * 9.81])

        front_angle = C.START_ANGLE - self.progress / C.RADIUS
        self.front = (
            self.roller + self.rotation[:, 1] * C.TAB_Y
            + C.RADIUS * self.radial(front_angle)
        )
        delta = self.tab - self.front
        length = float(np.linalg.norm(delta))
        direction = delta / max(length, 1e-7)
        leader = float(np.linalg.norm(C.TAB_OFFSET - C.SEAM))
        stretch = length - leader - .004 - self.progress
        extension_speed = 0. if self.previous_length is None else (length - self.previous_length) / dt
        self.previous_length = length
        self.tension = 0.
        if stretch > 0.:
            self.tension = float(np.clip(TAPE_STIFFNESS * stretch + TAPE_DAMPING * extension_speed, 0., 3.8))
        self.peak_tension = max(self.tension, self.peak_tension)
        peel_angle = float(direction @ self.radial(front_angle))
        resistance = STARTER_FORCE if self.progress < C.DRY_ZONE else PEEL_FORCE
        if self.tension >= resistance and peel_angle >= -.30:
            speed = min(.30, .24 * (self.tension - resistance))
            self.progress = min(C.SHEET_LENGTH, self.progress + speed * dt)
        fold_offset = self.tab - (self.roller + self.rotation @ C.TAB_OFFSET)
        fold_speed = np.zeros(3) if self.previous_fold is None else (fold_offset - self.previous_fold) / dt
        self.previous_fold = fold_offset
        bend = -FOLD_STIFFNESS * fold_offset - FOLD_DAMPING * fold_speed
        bend *= min(1., .62 / max(np.linalg.norm(bend), .00001))
        fold = max(0., 1. - self.progress / C.DRY_ZONE)
        return -self.tension * direction + fold * bend

    def nap_compression(self):
        normal = self.cloth_rotation[:, 2]
        axis = self.rotation[:, 1]
        if abs(axis @ normal) > .11:
            return 0.
        offset = self.cloth_rotation.T @ (self.roller - self.cloth)
        width = C.CLOTH_SIZE[:2] / 2
        if np.any(abs(offset[:2]) > width):
            return 0.
        return max(0., C.RADIUS + C.CLOTH_SIZE[2] / 2 + NAP_HEIGHT - offset[2])

    def nap_force(self, contact_velocity):
        normal = self.cloth_rotation[:, 2]
        tangent = np.cross(self.rotation[:, 1], normal)
        limit = NAP_FRICTION * self.load
        drag = np.clip(NAP_TRACTION * (contact_velocity @ tangent), -limit, limit)
        return self.load * normal - drag * tangent

    def collect(self, dt, angular_velocity):
        current = self.roller - self.cloth
        previous, self.previous = self.previous, current
        if previous is None or self.load < LOAD_REQUIRED:
            return
        if abs(self.rotation[:, 1] @ self.cloth_rotation[:, 2]) > .11:
            return
        tangent = np.cross(self.rotation[:, 1], self.cloth_rotation[:, 2])
        travel = float((current - previous) @ tangent)
        roll = C.RADIUS * (angular_velocity @ self.rotation[:, 1]) * dt
        if abs(travel) < .00001 or not .43 <= roll / travel <= 1.85:
            return
        self.travel += abs(travel)
        for number, local in enumerate(C.DIRT):
            if number in self.captured:
                continue
            fiber = self.cloth + self.cloth_rotation @ local
            local = self.rotation.T @ (fiber - self.roller)
            if abs(local[1]) < C.WIDTH / 2 - .002 and abs((fiber - self.roller) @ tangent) < .006:
                local[[0, 2]] *= (C.RADIUS + .00115) / max(.0001, np.linalg.norm(local[[0, 2]]))
                self.captured[number] = local

    def snapshot(self):
        return dict(peeled_m=self.progress, tension_n=self.tension,
                    peak_tension_n=self.peak_tension, contact_n=self.load,
                    peak_contact_n=self.peak_contact, rolling_m=self.travel,
                    collected=len(self.captured), total=len(C.DIRT))

    def sheet_panels(self):
        axis = self.rotation[:, 1] * C.WIDTH / 2
        if self.progress < C.SHEET_LENGTH:
            front_angle = C.START_ANGLE - self.progress / C.RADIUS
            angles = np.linspace(front_angle, C.START_ANGLE - math.tau, 33)
            seam = self.roller + (C.RADIUS + .0005) * self.radial(angles[0])
            centre = self.tab if self.progress < C.DRY_ZONE else self.tab - self.rotation[:, 1] * C.TAB_Y
            strip = self.rotation[:, 1] * C.TAB_SIZE[1] / 2 if self.progress < C.DRY_ZONE else axis
            anchor = seam + self.rotation[:, 1] * C.TAB_Y if self.progress < C.DRY_ZONE else seam
            yield centre, anchor, strip
            for a, b in zip(angles, angles[1:]):
                yield (
                    self.roller + (C.RADIUS + .0005) * self.radial(a),
                    self.roller + (C.RADIUS + .0005) * self.radial(b), axis,
                )
            return
        start = self.tab - self.rotation[:, 1] * C.TAB_Y * .45
        across = self.tab_rotation[:, 1] * C.WIDTH / 2
        for i in range(20):
            angle = i * .26
            direction = self.tab_rotation @ np.array([math.cos(angle), 0., -.75 + .45 * math.sin(angle)])
            end = start + direction * C.SHEET_LENGTH / 20
            end[2] = max(C.TABLE + .001, end[2])
            yield start, end, across
            start = end

    def render(self, stage):
        from pxr import Gf, UsdGeom

        mesh = UsdGeom.Mesh.Define(stage, "/World/RollerSheet")
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*SHEET)])
        points = []
        for a, b, axis in self.sheet_panels():
            points.extend([a-axis, a+axis, b+axis, b-axis])
        mesh.CreatePointsAttr().Set([Gf.Vec3f(*p) for p in points])
        mesh.CreateFaceVertexCountsAttr().Set([4] * (len(points) // 4))
        mesh.CreateFaceVertexIndicesAttr().Set(list(range(len(points))))
        for i, local in enumerate(C.DIRT):
            if i in self.captured:
                point = self.roller + self.rotation @ self.captured[i]
            else:
                point = self.cloth + self.cloth_rotation @ local
            speck = UsdGeom.Sphere.Define(stage, f"/World/Lint/f{i:02}")
            speck.CreateRadiusAttr(.0024 if i % 3 else .0031)
            speck.CreateDisplayColorAttr([Gf.Vec3f(*DIRT)])
            attr = speck.GetPrim().GetAttribute("xformOp:translate")
            if not attr:
                attr = speck.AddTranslateOp().GetAttr()
            attr.Set(Gf.Vec3d(*point))
