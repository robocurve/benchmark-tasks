"""Observable outcome measures, independent of the demonstration's actions.

Lengths are metres and masses are kilograms. A physical trial uses an overhead
photo/ruler, a tared spoil tray and video of the two grips.
"""

from dataclasses import dataclass, field

import digplan as d
import numpy as np
from soilkit import _icosahedron, grain_volume

BIN = 0.005
TARGET_U = (-0.020, 0.100)
CROWN_HALF_WIDTH = 0.008
# (score, contiguous exposed length, mass delivered by spade, loaded trips)
LEVELS = ((0.25, 0.020, 0.030, 1), (0.5, 0.040, 0.070, 2), (1.0, 0.080, 0.120, 3))
SPILL_LIMIT = 0.030
_vertices, _faces = _icosahedron()
_tri = _vertices[np.array(_faces)]
_normals = np.cross(_tri[:, 1] - _tri[:, 0], _tri[:, 2] - _tri[:, 0])
_normals /= np.linalg.norm(_normals, axis=1, keepdims=True)
_apothem = float(np.dot(_normals[0], _tri[0, 0]))


def grade(
    exposed, delivered, trips, spilled, failure=None, *, tool_clear=True, blade_load=0.0
):
    if failure is not None or spilled > SPILL_LIMIT:
        return 0.0
    value = 0.0
    for level, length, kg, count in LEVELS:
        if level == 1.0 and (not tool_clear or blade_load > 0.005):
            continue
        if exposed + 1e-9 >= length and delivered >= kg and trips >= count:
            value = level
    return value


def blade_clear_of_bed(spade):
    """Conservative overhead footprint: the blade must be outside the fixture.

    A visible pipe while the blade is still above it is not a finished trial.
    Use a bounding rectangle for the entire cup, including its raised wings.
    """
    corners = np.array(
        [
            d.blade_point(s, w)
            for s in (0.0, 0.025, d.BLADE_LEN)
            for w in (-d.BLADE_W / 2, 0.0, d.BLADE_W / 2)
        ]
    )
    uv = d.to_trench(corners @ spade.rotation().T + spade.pos)
    half = np.array([d.IN_U / 2 + d.WALL, d.IN_V / 2 + d.WALL])
    return bool(((uv.min(axis=0) > half) | (uv.max(axis=0) < -half)).any())


def grain_mass(radii):
    return 2650.0 * grain_volume(radii, "icosahedron")


def exposed_length(pos, radii, orientations=None):
    """Longest unoccluded strip of the pipe's central 16 mm crown.

    Every 5 mm bin is checked at three longitudinal and five transverse rays.
    Rays intersect the actual oriented convex grains, not inflated bounding
    spheres that can mark a visibly clear pipe as covered.
    """
    uv = d.to_trench(pos)
    starts = np.arange(TARGET_U[0], TARGET_U[1] - BIN / 2, BIN)
    uu = starts[:, None, None] + np.array([0.0, BIN / 2, BIN])[None, :, None]
    vv = np.linspace(-CROWN_HALF_WIDTH, CROWN_HALF_WIDTH, 5)[None, None, :]
    points = np.stack(np.broadcast_arrays(uu, vv), axis=-1).reshape(-1, 2)
    crown = d.PIPE_Z + np.sqrt(d.PIPE_R**2 - points[:, 1] ** 2)
    # Only grains near the crown can occlude it; chunk to bound memory.
    near = (pos[:, 2] + radii > d.PIPE_TOP - 0.004) & (
        abs(uv[:, 1]) < CROWN_HALF_WIDTH + radii + 0.001
    )
    near &= (uv[:, 0] + radii >= TARGET_U[0]) & (uv[:, 0] - radii <= TARGET_U[1])
    centres, rr = pos[near], radii[near]
    q = (
        np.tile([0, 0, 0, 1.0], (len(pos), 1))
        if orientations is None
        else np.asarray(orientations)
    )[near]
    x, y, z, w = q.T
    rotations = np.empty((len(q), 3, 3))
    rotations[:, 0, :] = np.stack(
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)), axis=-1
    )
    rotations[:, 1, :] = np.stack(
        (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)), axis=-1
    )
    rotations[:, 2, :] = np.stack(
        (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)), axis=-1
    )
    normals = np.einsum("fj,nkj->nfk", _normals, rotations)
    points_world = d.CENTRE + points[:, :1] * d.U + points[:, 1:] * d.V
    blocked = np.zeros(len(points), bool)
    # Bounding disks select candidates only; exact face planes decide occlusion.
    distance_sq = ((points_world[:, None, :] - centres[None, :, :2]) ** 2).sum(-1)
    ray_ids, grain_ids = np.nonzero(distance_sq <= rr[None, :] ** 2)
    for i in range(0, len(ray_ids), 4096):
        rays, grains = ray_ids[i : i + 4096], grain_ids[i : i + 4096]
        ns = normals[grains]
        delta = points_world[rays] - centres[grains, :2]
        numerator = _apothem * rr[grains, None] - np.einsum(
            "nk,nfk->nf", delta, ns[:, :, :2]
        )
        nz = ns[:, :, 2]
        height = numerator / np.where(abs(nz) > 1e-10, nz, 1.0)
        upper = np.where(nz > 1e-10, height, np.inf).min(axis=-1)
        lower = np.where(nz < -1e-10, height, -np.inf).max(axis=-1)
        outside = ((abs(nz) <= 1e-10) & (numerator < 0)).any(axis=-1)
        hit = ~outside & (upper >= lower) & (upper + centres[grains, 2] >= crown[rays])
        blocked[rays[hit]] = True
    clear = ~blocked.reshape(len(starts), -1).any(1)
    best = run = 0
    for b in clear:
        run = run + 1 if b else 0
        best = max(best, run)
    return best * BIN


def regions(pos, radii):
    uv = d.to_trench(pos)
    in_bed = (abs(uv[:, 0]) < d.IN_U / 2 + d.WALL + radii) & (
        abs(uv[:, 1]) < d.IN_V / 2 + d.WALL + radii
    )
    in_bed &= pos[:, 2] + radii >= d.TOP + d.FLOOR - 0.003
    c, s = np.cos(d.TARP_YAW), np.sin(d.TARP_YAW)
    tarp = (pos[:, :2] - d.TARP_C) @ np.array([[c, -s], [s, c]])
    # Require the entire grain to be on the tarp, rather than merely its centre.
    on_tarp = (abs(tarp[:, 0]) + radii <= d.TARP_SIZE[0] / 2) & (
        abs(tarp[:, 1]) + radii <= d.TARP_SIZE[1] / 2
    )
    on_tarp &= (pos[:, 2] - radii >= d.TOP - 0.002) & (pos[:, 2] < d.TOP + 0.060)
    spilled = ~in_bed & ~on_tarp & (pos[:, 2] < d.TOP + 0.030)
    return in_bed, on_tarp, spilled


@dataclass
class Progress:
    failure: str | None = None
    carried: np.ndarray | None = None
    trip_ids: np.ndarray | None = None
    trip: int = 0
    was_over_bed: bool = False
    grip_loss_s: float = 0.0
    previous_pos: np.ndarray | None = None
    settled_s: np.ndarray | None = None
    metrics: dict = field(default_factory=dict)

    def fail(self, reason):
        if self.failure is None:
            self.failure = reason
        self.metrics.update(score=0.0, failure=self.failure)

    def update(
        self,
        pos,
        radii,
        spade,
        hands,
        *,
        pipe_contact=False,
        hand_soil_contact=False,
        dt=0.05,
        grip_forces=None,
        orientations=None,
    ):
        # Pipe contact is allowed. Its collider still resists the blade; this
        # task scores exposure and spoil handling, not avoidance of all contact.
        if hand_soil_contact:
            self.fail("robot touched soil directly")
        if self.carried is None:
            self.carried = np.zeros(len(pos), bool)
            self.trip_ids = np.zeros(len(pos), int)
            self.settled_s = np.zeros(len(pos), float)
        if (
            len(pos) != len(self.carried)
            or np.shape(pos) != (len(self.carried), 3)
            or np.shape(radii) != (len(pos),)
            or not np.isfinite(pos).all()
            or not np.isfinite(radii).all()
            or (radii <= 0).any()
            or (
                orientations is not None
                and (
                    np.shape(orientations) != (len(pos), 4)
                    or not np.isfinite(orientations).all()
                    or (abs(np.linalg.norm(orientations, axis=1) - 1) > 1e-3).any()
                )
            )
        ):
            self.fail("missing or invalid soil state")
            return 0.0
        r = spade.rotation()
        local = (pos - spade.pos) @ r
        grips = d.grips_local()
        valid_grips = []
        for arm, (target, orient) in grips.items():
            if arm not in hands:
                valid_grips.append(False)
                continue
            p, rotation, finger = hands[arm]
            angle = np.arccos(
                np.clip((np.trace(rotation.T @ r @ orient) - 1) / 2, -1, 1)
            )
            contact = grip_forces is not None and grip_forces.get(arm, 0.0) > 0.05
            valid_grips.append(
                contact
                and np.linalg.norm(p - spade.pos - r @ target) < 0.025
                and angle < np.radians(25)
                and -0.025 < finger < -0.004
            )
        both = all(valid_grips)
        tip = spade.pos + r @ d.tip_local()
        uvtip = d.to_trench(tip)
        over_bed = abs(uvtip[0]) < d.IN_U / 2 and abs(uvtip[1]) < d.IN_V / 2
        if over_bed and not self.was_over_bed:
            self.trip += 1
        self.was_over_bed = over_bed
        blade_s = local @ d.blade_dir()
        blade_h = local @ np.array([-np.sin(d.LIFT), 0, np.cos(d.LIFT)]) - d.BLADE_Z
        widths = np.array(
            [d.half_width(float(np.clip(s, 0, d.BLADE_LEN))) for s in blade_s]
        )
        # The floor, wings and rear ramp are separate planar collision panels.
        top = np.maximum(
            np.maximum(0, abs(local[:, 1]) - d.FLAT_HALF) * np.tan(d.WING),
            d.HEEL_RISE * np.maximum(0, 1 - blade_s / 0.025),
        )
        on_blade = (
            (blade_s > 0)
            & (blade_s < d.BLADE_LEN)
            & (abs(local[:, 1]) < widths)
            & (blade_h > top)
            & (blade_h < top + 0.035)
        )
        lifted = pos[:, 2] - radii > d.TOP + d.WALL_H + 0.005
        working = (over_bed and tip[2] < d.TOP + d.WALL_H) or (on_blade & lifted).any()
        self.grip_loss_s = self.grip_loss_s + dt if working and not both else 0.0
        if self.grip_loss_s > 0.20:
            self.fail("spade used without both grips for over 0.20 s")
        if both:
            acquired = on_blade & lifted & ~self.carried
            self.carried[acquired] = True
            self.trip_ids[acquired] = self.trip
        _, on_tarp, spilled = regions(pos, radii)
        mass = grain_mass(radii)
        speed = (
            np.full(len(pos), np.inf)
            if self.previous_pos is None
            else np.linalg.norm(pos - self.previous_pos, axis=1) / dt
        )
        self.previous_pos = pos.copy()
        resting = on_tarp & ~on_blade & (speed < 0.020)
        self.settled_s = np.where(resting, self.settled_s + dt, 0.0)
        delivered = on_tarp & self.carried & (self.settled_s >= 0.25)
        by_trip = {
            int(t): float(mass[delivered & (self.trip_ids == t)].sum())
            for t in np.unique(self.trip_ids[delivered])
            if t > 0
        }
        trips = sum(m >= 0.020 for m in by_trip.values())
        exposed = exposed_length(pos, radii, orientations)
        delivered_mass = float(mass[delivered].sum())
        spilled_mass = float(mass[spilled].sum())
        blade_load = float(mass[on_blade].sum())
        tool_clear = blade_clear_of_bed(spade)
        value = grade(
            exposed,
            delivered_mass,
            trips,
            spilled_mass,
            self.failure,
            tool_clear=tool_clear,
            blade_load=blade_load,
        )
        self.metrics = {
            "score": value,
            "exposed_m": exposed,
            "delivered_kg": delivered_mass,
            "spilled_kg": spilled_mass,
            "loaded_trips": trips,
            "both_grips": both,
            "blade_grains": int(on_blade.sum()),
            "blade_load_kg": blade_load,
            "tool_clear": tool_clear,
            "failure": self.failure,
            "pipe_contact": bool(pipe_contact),
        }
        return value
