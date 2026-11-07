"""Observable final-state scoring; no prescribed arm assignment or motion sequence."""
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from .config import FRAME_POSITION, CARTRIDGE_OFFSET, TABLE_TOP, TRAY_POSITION


def rotation(pose):
    return quat_xyzw_to_matrix(np.array(pose[3:]))


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.violations = set()
        self.physics_samples = 0
        self.hold_start = None

    def observe_physics(self, points):
        self.physics_samples += 1
        for p in points.values():
            if not np.isfinite(p).all():
                self.violations.add('invalid_state')
            elif (p[:, 2].min() < TABLE_TOP-.006 or p[:, 0].min() < -.19
                  or p[:, 0].max() > .91 or np.abs(p[:, 1]).max() > .65):
                self.violations.add('part_left_work_area')

    def evaluate(self, s):
        frame = s['bodies']['Frame']['pose']
        frot = rotation(frame)
        used, new = s['used'], s['replacement']
        upos, npos = np.array(used['pose'][:3]), np.array(new['pose'][:3])
        local = frot.T @ (npos-np.array(frame[:3]))
        axis_error = np.arccos(np.clip(abs((rotation(new['pose']).T@frot)[1, 1]), 0, 1))
        # Keep front/rear piston direction as well as axis alignment.
        forward = float(np.dot(rotation(new['pose'])[:, 1], frot[:, 1])) > .98
        ulo, uhi = np.array(s['bounds']['used:Body'])
        inside_tray = bool(ulo[0] >= TRAY_POSITION[0]-.050 and uhi[0] <= TRAY_POSITION[0]+.050
                          and ulo[1] >= TRAY_POSITION[1]-.135 and uhi[1] <= TRAY_POSITION[1]+.135)
        collected = (inside_tray and abs(upos[0]-TRAY_POSITION[0]) < .025
            and abs(upos[1]-TRAY_POSITION[1]) < .025
            and abs(upos[2]-(TABLE_TOP+.029)) < .008
            and abs(rotation(used['pose'])[2, 1]) < .1
            and np.linalg.norm(used['velocity'][:3]) < .02)
        seated = bool(np.max(np.abs(local-CARTRIDGE_OFFSET)) < .006 and axis_error < .10 and forward
            and np.linalg.norm(new['velocity'][:3]) < .02)
        mechanism = s['mechanism']
        turn = float(np.arctan2(np.sin(mechanism['rod_rotation']), np.cos(mechanism['rod_rotation'])))
        piston_gap = float(local[1]-.0985-(.195+mechanism['rod_slide']))
        ready = bool(-.001 <= piston_gap <= .005 and abs(turn) < .15
                     and abs(mechanism['pawl_angle']) < .12)
        parked = (np.linalg.norm(np.array(frame[:3])-FRAME_POSITION) < .020
                  and np.trace(frot) >= 1+2*np.cos(np.deg2rad(10))
                  and np.linalg.norm(s['bodies']['Frame']['velocity'][:3]) < .02)
        clear = bool(s['robot_clearance_m'] >= .03
                     and all(h['opening_m'] >= .070 for h in s['hands'].values()))
        full = bool(collected and seated and ready and parked and clear and not self.violations)
        if full:
            if self.hold_start is None:
                self.hold_start = s['time_s']
        else:
            self.hold_start = None
        success = bool(full and s['time_s']-self.hold_start >= 1.)
        score = (1. if success else .75 if collected and seated and ready else .5 if collected and seated
                 else .25 if collected else 0.) if not self.violations else 0.
        return dict(score=score, success=success, violations=sorted(self.violations),
                    collected=bool(collected), seated=seated, rod_ready=ready, parked=bool(parked),
                    hands_clear=clear, robot_clearance_m=s['robot_clearance_m'], piston_gap_m=piston_gap,
                    mechanism=mechanism, frame_position_m=frame[:3],
                    used_position_m=upos.tolist(), replacement_position_m=npos.tolist(),
                    physics_samples=self.physics_samples)
