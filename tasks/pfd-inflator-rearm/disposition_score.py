"""Public handoff outcome for the dry two-cartridge service bench."""
import numpy as np
from scipy.spatial.transform import Rotation
from disposition_layout import (TABLE, USED, HOLD, HOLD_HALF, CAN_HEIGHT, CAN_RADIUS,
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
    return bool((extent < half_width).all() and floor.all())


class Grader:
    def __init__(self, stage, paths, filters):
        names = [p.rsplit("/", 1)[-1] for p in paths]
        a,b = (names.index(k) for k in ("spare", "reserve"))
        flags = [recessed(stage, paths[i]) for i in [a,b]]
        if flags[0] == flags[1]:
            raise ValueError("The service-return tray needs exactly one intact and one recessed loose face")
        self.good, self.bad = (b,a) if flags[0] else (a,b)
        self.old = names.index("installed")
        self.window = names.index("follower")
        self.part_filters = [filters.index(p) for p in paths]
        self.shelves = [i for i,p in enumerate(filters) if "/bay_shelf_" in p]
        self.banks = [[i for i in self.shelves if (int(filters[i].rsplit("_",1)[1]) < 16) == half]
                      for half in (True,False)]
        self.table = next(i for i,p in enumerate(filters) if p.endswith("/table/geometry/mesh"))
        self.floor_supports = [self.table] + [i for i,p in enumerate(filters)
                                                if p.endswith(("parts_stand","reserve_stand"))]
        self.supply = [i for i,p in enumerate(filters) if p.endswith(("parts_stand","reserve_stand"))]
        self.outcome = "incomplete"
        self.score = 0.
        self.hold = 0
        self.reference = None
        self.metrics = {}

    def update(self, poses, robot, contacts, forces, joints):
        new = poses[:, self.good]
        rot = Rotation.from_quat(new[:, 3:])
        lug = new[:, :3] + rot.apply([0.,0.,.130])
        turn = np.rad2deg(np.arctan2(rot.as_matrix()[:,1,0], rot.as_matrix()[:,0,0]))
        error = (turn - INDEX_DEG + 90) % 180 - 90
        aligned = bool((abs(error) <= 6).all() and (rot.as_matrix()[:,2,2] > .99).all())
        supported = bool(all(contacts[:,self.good][:,bank].any(axis=1).all() for bank in self.banks)
                         and not contacts[:,self.good][:,self.floor_supports].any()
                         and (new[:,2] - TABLE > .050).all()
                         and (np.linalg.norm(lug[:,:2]-SEAT_XY,axis=1)<.003).all())
        collected = contained(poses[:,self.old], USED, contacts[:,self.old,self.table])
        isolated = contained(poses[:,self.bad], HOLD, contacts[:,self.bad,self.table], HOLD_HALF)
        released = not robot[:,:,self.part_filters].any()
        lift = poses[:,self.window,2] - .9496
        green = bool(((lift>=.0028)&(lift<=.0046)).all() and
                     contacts[:,self.window,self.part_filters[self.good]].all())
        original = poses[:,self.old]
        old_axis = Rotation.from_quat(original[:,3:])
        old_lug = original[:,:3] + old_axis.apply([0,0,.130])
        old_angle = old_axis.as_euler('xyz',degrees=True)[:,2]
        old_locked = bool((np.linalg.norm(old_lug[:,:2]-SEAT_XY,axis=1)<.003).all()
                          and (abs((old_angle-INDEX_DEG+90)%180-90)<=6).all()
                          and all(contacts[:,self.old][:,bank].any(axis=1).all() for bank in self.banks))
        red = bool((lift < .001).all())
        intact_on_supply = bool(contacts[:,self.good][:,self.supply].any(axis=1).all()
                                and (rot.as_matrix()[:,2,2] > .99).all())
        ready = all((aligned, supported, collected, isolated, released, green))
        quarantine_only = all((isolated, released, red, intact_on_supply, old_locked or collected))
        outcome = "ready" if ready else "isolated_out_of_service" if quarantine_only else "incomplete"
        if outcome != self.outcome:
            self.hold = 0
            self.reference = None
        self.outcome = outcome
        state = np.stack((lug, original[:,:3], poses[:,self.bad,:3]),axis=1)
        if ready or quarantine_only:
            if self.reference is None or (np.linalg.norm(state-self.reference,axis=2)>.001).any():
                self.hold = 0
                self.reference = state[0].copy()
            if (np.linalg.norm(state-self.reference,axis=2)<=.001).all():
                self.hold += len(poses)
            else:
                self.reference = None
        else:
            self.hold = 0
            self.reference = None
        self.score = (1. if ready else .5) if self.hold >= 241 else 0.
        self.metrics = dict(score=self.score, outcome=outcome, old_collected=collected, defective_quarantined=isolated,
                            index_aligned=aligned, suspended=supported, released=released, green=green,
                            hold_substeps=self.hold, index_error_deg=float(error[-1]),
                            indicator_lift_mm=float(lift[-1]*1000))
