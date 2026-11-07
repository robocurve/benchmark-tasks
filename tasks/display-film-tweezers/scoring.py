"""Outcome scoring, independent of Isaac. History is updated once per control step."""
import numpy as np
from geometry import FILM_NAMES, N_BONDED, PITCH, CARRIER_POS, CARRIER_SIZE, SIZES, TABLE_Z, corners, rotation
from tool_geometry import ALL_TOOL_NAMES,STAND_FLOORS,returned

# Film rest is measured from poses over the hold, as two photos a second apart would be. The
# 0.3 g strips of a folded sheet can report several mm/s and rad/s of solver velocity while
# their poses stay fixed, so their instantaneous velocities are not used.
FILM_STILL_M = .002
FILM_STILL_RAD = np.deg2rad(5)


class Trial:
    def __init__(self):
        self.reset()

    def reset(self):
        self.peeled = 0
        self.invalid = False
        self.hands_clear = False
        self.settled_seconds = 0.
        self._rest_poses = None

    def observe(self, objects, *, hands_clear):
        self.hands_clear = bool(hands_clear)
        try:
            for name in (*SIZES,*ALL_TOOL_NAMES,*STAND_FLOORS):
                s = objects[name]
                if not all(np.all(np.isfinite(v)) for v in
                           (s.pos, s.quat_xyzw, s.lin_vel, s.ang_vel)):
                    self.invalid = True
                rotation(s.quat_xyzw)
            p = objects['carrier']
            r = rotation(p.quat_xyzw)
            # Throughout the trial: no lifting, dragging and replacing, or rotating the carrier.
            self.invalid |= bool(np.linalg.norm(np.asarray(p.pos)[:2] - CARRIER_POS[:2]) > .030 + 1e-9)
            self.invalid |= bool(abs(p.pos[2] - CARRIER_POS[2]) > .005 + 1e-9)
            self.invalid |= bool(r[2, 2] < np.cos(np.deg2rad(10)) - 1e-9)
            self.invalid |= bool(abs(np.arctan2(r[1, 0], r[0, 0])) > np.deg2rad(10) + 1e-9)
        except (KeyError, ValueError, IndexError):
            self.invalid = True

    def disposed(self, objects):
        if not self.hands_clear or not returned(objects):
            return False
        plate = objects['carrier']
        r = rotation(plate.quat_xyzw)
        table_contact = False
        for name in FILM_NAMES:
            s = objects[name]
            world = corners(s, SIZES[name])
            local = (world - plate.pos) @ r
            # Conservative whole-piece separation, including a 2 mm margin.
            outside = any(local[:, axis].min() >= CARRIER_SIZE[axis]/2 + .002 or
                          local[:, axis].max() <= -CARRIER_SIZE[axis]/2 - .002
                          for axis in (0, 1))
            # Permit the precreased tab to remain raised after disposal. The sheet
            # must have table support, remain entirely within 30 mm of the surface,
            # be clear of both hands and stay still for the hold (finish_step); an
            # airborne sheet cannot pass.
            table_contact |= bool(TABLE_Z - .002 <= world[:, 2].min() <= TABLE_Z + .003)
            near_table = (world[:, 2].min() >= TABLE_Z - .002
                          and world[:, 2].max() <= TABLE_Z + .030)
            if not (outside and near_table):
                return False
        return table_contact

    def score(self, objects):
        if self.invalid:
            return 0.0
        if self.settled_seconds >= 1. - 1e-9 and self.peeled == N_BONDED and self.disposed(objects):
            return 1.0
        # Consecutive peeled adhesive length: 30 / 60 / 90 mm of 115 mm.
        mm = round(self.peeled * PITCH * 1000, 6)
        return .75 if mm >= 90 else .5 if mm >= 60 else .25 if mm >= 30 else 0.0

    def finish_step(self, objects, dt):
        """Count stable time exactly once per measured control step, even at timeout."""
        stable = not self.invalid and self.peeled == N_BONDED and self.disposed(objects)
        if not stable:
            self._rest_poses, self.settled_seconds = None, 0.
            return
        poses = {n: (np.array(objects[n].pos, dtype=float), rotation(objects[n].quat_xyzw))
                 for n in FILM_NAMES}
        if self._rest_poses is None or not self._still(poses):
            # The hold restarts from the current film poses.
            self._rest_poses, self.settled_seconds = poses, dt
        else:
            self.settled_seconds += dt

    def _still(self, poses):
        for name, (pos, rot) in poses.items():
            ref_pos, ref_rot = self._rest_poses[name]
            turn = np.arccos(np.clip((np.trace(ref_rot.T @ rot) - 1) / 2, -1., 1.))
            if np.linalg.norm(pos - ref_pos) > FILM_STILL_M + 1e-9 or turn > FILM_STILL_RAD + 1e-9:
                return False
        return True
