"""Public handoff outcome for the dry two-cartridge service bench."""
import numpy as np
from scipy.spatial.transform import Rotation
from disposition_layout import (USED, HOLD, HOLD_HALF, CAN_HEIGHT, CAN_RADIUS,
                                CUP_HALF_WIDTH, SEAT_XY, INDEX_DEG)

CURRENT = None
def recessed(stage, path):
    points = np.array(stage.GetPrimAtPath(path + "/metal").GetAttribute("points").Get())
    axis = np.linalg.norm(points[:, :2], axis=1) < 1e-6
    return bool(points[:, 2].max() - points[axis, 2].max() > .001)


def contained(pose, center, floor, half_width=CUP_HALF_WIDTH):
    axis = Rotation.from_quat(pose[:, 3:]).as_matrix()[:, :, 2]
    ends = pose[:, :2] + CAN_HEIGHT * axis[:, :2]
    extent = np.maximum(abs(pose[:, :2] - center), abs(ends - center)) + CAN_RADIUS
    return (extent < half_width).all(axis=1) & (floor > 0)


def paint_sector(stage, path, axis_xy, nominal_deg):
    """Angular footprint of an existing painted box, viewed along its mounting axis."""
    mark = stage.GetPrimAtPath(path)
    center = np.asarray(mark.GetAttribute('xformOp:translate').Get())[:2] - axis_xy
    half = np.asarray(mark.GetAttribute('xformOp:scale').Get())[:2] / 2
    corners = center + half * np.array([[-1,-1], [-1,1], [1,-1], [1,1]])
    offsets = (np.rad2deg(np.arctan2(corners[:,1], corners[:,0])) - nominal_deg + 180) % 360 - 180
    return offsets.min(), offsets.max()


class Grader:
    def __init__(self, stage, paths, filters):
        names = [p.rsplit("/", 1)[-1] for p in paths]
        a,b = (names.index(k) for k in ("spare", "reserve"))
        flags = [recessed(stage, paths[i]) for i in [a,b]]
        if flags[0] == flags[1]:
            raise ValueError("The service-return tray needs exactly one intact and one recessed loose face")
        self.good, self.bad = (b,a) if flags[0] else (a,b)
        for i in (a,b):
            if not stage.GetPrimAtPath(paths[i] + "/sensor_ring").IsValid():
                raise ValueError("Both loose training adapters require the same attached sensor ring")
        self.old = names.index("installed")
        self.window = names.index("follower")
        self.part_filters = [filters.index(p) for p in paths]
        self.shelves = [i for i,p in enumerate(filters) if "/bay_shelf_" in p]
        self.other_surfaces = [i for i in range(len(filters)) if i not in self.shelves]
        self.banks = [[i for i in self.shelves if (int(filters[i].rsplit("_",1)[1]) < 16) == half]
                      for half in (True,False)]
        self.table = next(i for i,p in enumerate(filters) if p.endswith("/table/geometry/mesh"))
        self.supply = [i for i,p in enumerate(filters) if p.endswith(("parts_stand","reserve_stand"))]
        self.external_supports = self.supply + [self.table] + [
            i for i, path in enumerate(filters)
            if path.rsplit('/',1)[-1].startswith(('hold_', 'cup_', 'trainer_'))
            or path in (paths[self.old], paths[self.good], paths[self.bad])]
        # Opposing marks are identical. Read their widths from this actual dry fixture.
        root = paths[self.good].rsplit('/',1)[0]
        fixed_lo, fixed_hi = paint_sector(stage, root+'/seat_index_0', SEAT_XY, INDEX_DEG)
        stripe_lo, stripe_hi = paint_sector(stage, paths[self.good]+'/index_a', (0.,0.), 0.)
        self.mark_limits = fixed_lo - stripe_hi, fixed_hi - stripe_lo
        shelf = stage.GetPrimAtPath(filters[self.shelves[0]])
        self.shelf_z = shelf.GetAttribute('xformOp:translate').Get()[2]
        self.outcome = "incomplete"
        self.score = 0.
        self.hold = 0
        self.metrics = {}

    def mounting(self, piece, poses, contacts, forces):
        pose = poses[:,piece]
        axes = Rotation.from_quat(pose[:,3:]).as_matrix()
        twist = np.rad2deg(np.arctan2(axes[:,1,0], axes[:,0,0]))
        error = (twist - INDEX_DEG + 90) % 180 - 90
        lo, hi = self.mark_limits
        marks_overlap = (lo <= error) & (error <= hi)
        # Direct upward reactions distinguish a supporting shelf from a side brush.
        supported = (axes[:,2,2] > 0) & (pose[:,2] < self.shelf_z)
        supported &= ~contacts[:,piece,self.external_supports].any(axis=1)
        for bank in self.banks:
            supported &= contacts[:,piece,bank].any(axis=1)
        # Both lugs engage, and the shelves carry the cylinder. A stop may touch
        # laterally and the shutter may press down; neither is support from below.
        shelf_up = forces[:,piece,self.shelves,2].sum(axis=1)
        other_up = np.maximum(forces[:,piece,self.other_surfaces,2], 0).sum(axis=1)
        supported &= shelf_up > other_up
        return marks_overlap, supported, error

    def update(self, poses, robot, contacts, forces, joints):
        aligned, supported, error = self.mounting(self.good, poses, contacts, forces)
        old_aligned, old_supported, _ = self.mounting(self.old, poses, contacts, forces)
        original = poses[:,self.old]
        collected = contained(original, USED, contacts[:,self.old,self.table])
        isolated = contained(poses[:,self.bad], HOLD, contacts[:,self.bad,self.table], HOLD_HALF)
        released = ~robot[:,:,self.part_filters].any(axis=(1,2))
        lift = poses[:,self.window,2] - .9496
        green = (lift >= .0028) & (lift <= .0046)
        red = lift < .001
        good_upright = Rotation.from_quat(poses[:,self.good,3:]).as_matrix()[:,2,2] > .99
        on_stand = contacts[:,self.good,self.supply].any(axis=1) & good_upright
        ready = aligned & supported & collected & isolated & released & green
        quarantine = isolated & released & red & on_stand & ((old_aligned & old_supported) | collected)
        if ready[-1]:
            outcome, value, samples = 'ready', 1., ready
        elif quarantine[-1]:
            outcome, value, samples = 'isolated_out_of_service', .5, quarantine
        else:
            outcome, value, samples = 'incomplete', 0., None
        # Score the trailing uninterrupted outcome, including a transition inside a control step.
        if samples is None:
            self.hold = 0
        elif samples.all():
            self.hold = (self.hold if self.outcome == outcome else 0) + len(samples)
        else:
            self.hold = len(samples) - 1 - int(np.flatnonzero(~samples)[-1])
        self.outcome = outcome
        self.score = value if self.hold >= 241 else 0.
        self.metrics = dict(score=self.score, outcome=outcome, old_collected=bool(collected[-1]),
                            defective_quarantined=bool(isolated[-1]), index_aligned=bool(aligned[-1]),
                            suspended=bool(supported[-1]), released=bool(released[-1]), green=bool(green[-1]),
                            hold_substeps=self.hold, index_error_deg=float(error[-1]),
                            indicator_lift_mm=float(lift[-1]*1000))
