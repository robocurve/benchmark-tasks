"""Scripted demo for battery-pills: wrist-camera RGB, joint readings and the published rig calibration only.

No simulator pose, no pose write, no attachment: the policy sees ``observation.images['right_cam']`` and
``observation.state['joint_pos']`` and returns 14-D absolute joint targets like any yam_arena policy. Part sizes and
heights are the card's drawings; the start positions are the card's nominal ones; the draws (dish +-15 mm, layout
rotation, cell / empty slot) are found with the right wrist camera. The tray (+-10 / +-15 mm) and jar (+-10 mm) draws
are not looked for: the dish is set at the tray's nominal centre (its outline allows +-22.5 mm in x, +-110 mm in y
for the 121 mm dish) and the cell is dropped on the jar's nominal axis (35 mm Stowed radius).

Plan
  right arm (the cell)
    survey      camera over the dish's nominal centre, then re-centred over the fitted dish centre: the six discs,
                the darkest = the cell; dish centre = the 7-slot lattice fit; then a closer look over the cell
    pick        straight down; the closing line is the one with the most finger clearance to the tablets (finger
                tips 4.3 mm thick, +-8.6 mm wide at tablet height); jaws 11 mm open; tips 1 mm above the dish floor;
                close to 3.9 mm (about 10 N); the reading must be the cell's 7.9 mm (not a 9.5 mm tablet, not empty),
                checked again 10 mm up; otherwise open in place, look again, retry (3 attempts)
    to_jar      up, over the jar's nominal axis, tips 15 mm above its rim; open; the cell drops in; home
  left arm (the dish)
    rim_grasp   pads straddling the dish's flared wall at its back-left (azimuth 120 deg), the approach axis down the wall (49.5 deg),
                tips 14 mm down the wall from the rim; close (about 7 N on the 3 mm wall)
    carry       up 25 mm slowly, up to the dish bottom 80 mm above the table (over the 51 mm tray wall), across to the
                tray's nominal centre, down to 1.5 mm above the tray floor; open; back out up the wall; home
  final_hold    both arms at ready for 2 s (Released and still)
Mode 'hover' stops after the cell is held over the jar (never released, the dish never moved): a baseline.
"""
import math
from dataclasses import dataclass, field

import numpy as np

import geometry as G
import perception as V

READY = np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0])
TIP_PAST_TCP = 0.0128              # finger tip along the approach axis from the TCP (collision hulls: 146.8 - 134 mm)
SURVEY_DEPTH = 0.140               # camera 14 cm from the dish floor along its view: finger tips ~15 mm over the rim
                                   # (job 4296409 step 2: at 7.5 cm the open fingers struck the dish and threw the cell)
CLOSE_DEPTH = 0.140                # the closer look only re-centres on the cell (same safe height)
PICK_HALF_OPEN = 0.0075            # jaws 15 mm open over the 7.9 mm cell (3.5 mm margin a side; 11 mm let a finger land on
                                   # the cell edge in job 4296409 step 3; tablet clearance still >= 1.4 mm over 59 draws)
PICK_TIP_ABOVE_FLOOR = -0.0003     # finger tips commanded 0.3 mm into the dish floor (they press on it lightly): at 1 mm up
                                   # the closing fingers squeezed the 3.6 mm cell out from under them (job 4296409 step 4)
PICK_TRIM_ABOVE = 0.003            # the TCP trim runs 3 mm above the pick pose (no contact), then straight down
CELL_PINCH = (2 * G.CELL_R - 2 * 1.0) / 94.0   # pads commanded 1.0 mm inside the cell each side (~5 N; 0.5 mm held it 7 mm
                                   # up then dropped it, job 4296409 step 6; the card plans 2 N)
                                   # plan); 2 mm (~10 N) on the 0.53 g cell tipped it up between the pads (job 4296409 steps 4-5)
CELL_LO, CELL_HI = 0.065, 0.078    # job 4296409 step 7: a cell held through the lift read 0.067 at command 0.0627
                                   # (empty jaws read the command); a tablet adds 1.6 mm (~0.084)
#    # reading band for a held cell (7.9 mm = 0.084); a 9.5 mm tablet reads ~0.101; a cell held
                                   # across its 3.6 mm thickness (tipped) reads ~0.04-0.05
JAR_TIP_ABOVE_RIM = 0.025         # finger-tip centre 25 mm above the rim (straight down is out of reach there)
JAR_TILT_DEG = 20.0                # approach axis tilted 20 deg forward (URDF IK: 15-60 deg reach every jar draw corner)
JAR_OPEN = 0.30                    # 28 mm: the cell drops; the lower finger stays ~5 mm above the tip centre's height
WALL_DEG = 49.5                    # mean of the dish's inner (47.7) and outer (51.4) flare angles
RIM_PSI_DEG = 120.0                # left arm takes the rim at azimuth 120 deg (back-left): URDF IK reaches it at every
                                   # dish and tray draw corner, lifted and placed (scan: 0.0 mm at psi 120, 45-50 deg)
RIM_MID_R = 0.059                  # rim mid-line radius (57.5 inner / 60.5 outer)
RIM_TIP_DOWN = 0.014               # finger tips 14 mm down the wall from the rim top
RIM_OPEN = 22.0 / 94.0             # jaws 22 mm open round the 3 mm wall
RIM_PINCH = 0.0                    # close: pads 1.5 mm inside the wall each side (~7 N at 5000 N/m)
RIM_HELD = 0.015                   # reading above this = the wall between the pads (3 mm = 0.032)
CARRY_BOTTOM = 0.080               # dish bottom 80 mm above the table while carried
PLACE_BOTTOM = (G.TRAY_FLOOR + 1.5) * G.MM
SPEED = 0.12
SLOW = 0.02
Q_SEED_R = np.array([0.6, 1.4, 1.2, -0.9, 0.0, 0.0])
Q_SEED_L = np.array([-0.5, 1.2, 1.0, -0.6, 0.0, 0.0])


def r_down(theta):
    """Straight-down gripper, jaws closing along (cos theta, sin theta, 0)."""
    c, s = math.cos(theta), math.sin(theta)
    return np.column_stack([(c, s, 0.0), (s, -c, 0.0), (0.0, 0.0, -1.0)])


def r_tilt_forward(deg):
    """Approach axis tilted ``deg`` from straight down toward +x; jaws close in the x-z plane."""
    b = math.radians(deg)
    z = np.array([math.sin(b), 0.0, -math.cos(b)])
    x = np.array([math.cos(b), 0.0, math.sin(b)])
    return np.column_stack([x, np.cross(z, x), z])


def r_rim(psi_deg=None, deg=None):
    """At the rim point at azimuth ``psi`` round the dish: approach axis down the wall (``deg`` below horizontal,
    pointing inward), jaws closing along the wall normal. Returns (rotation, outward radial unit vector)."""
    psi = math.radians(RIM_PSI_DEG if psi_deg is None else psi_deg)
    a = math.radians(WALL_DEG if deg is None else deg)
    u = np.array([math.cos(psi), math.sin(psi), 0.0])
    z = -math.cos(a) * u + np.array([0.0, 0.0, -math.sin(a)])
    x = math.sin(a) * u + np.array([0.0, 0.0, -math.cos(a)])
    return np.column_stack([x, np.cross(z, x), z]), u


def quat_xyzw(r):
    w, x, y, z = G.matrix_to_quat_wxyz(r)
    return (float(x), float(y), float(z), float(w))


@dataclass
class Phase:
    name: str
    start: int
    end: int = None
    info: dict = field(default_factory=dict)


class Arm:
    """Command state of one arm: joint targets, gripper, a Cartesian offset trimmed from FK of the readings."""

    def __init__(self, side, seed_q):
        self.side = side
        self.model = V.ArmModel(side)
        self.q_cmd = READY.copy()
        self.grip = 1.0
        self.offset = np.zeros(3)
        self.rot = r_down(0.0)
        self.seed_q = seed_q


class BatteryDemo:
    def __init__(self, control_hz=20, mode='demo', cam_width=320, cam_height=176):
        if (cam_width, cam_height) != (V.CAM_W, V.CAM_H):
            raise ValueError('the demo is calibrated for 320 x 176 observations')
        self.hz, self.mode = control_hz, mode
        self.L, self.R = Arm('left', Q_SEED_L), Arm('right', Q_SEED_R)
        self.log, self.events, self.step, self.done = [], [], 0, False
        self.dish_xy = np.array(G.DI['start_centre_mm']) * G.MM
        self.cell_xy = None
        self.tablets = []
        self._obs = None
        self._plan = self._script()

    # -- helpers ----------------------------------------------------------------------------------------------
    def q(self, arm):
        js = np.asarray(self._obs.state['joint_pos'], float)
        return js[:6] if arm.side == 'left' else js[7:13]

    def reading(self, arm):
        js = np.asarray(self._obs.state['joint_pos'], float)
        return float(js[6] if arm.side == 'left' else js[13])

    def _phase(self, name, **info):
        if self.log:
            self.log[-1].end = self.step
        self.log.append(Phase(name, self.step, info=info))

    def _ik(self, arm, tcp_target, rot=None, q_init=None, iters=60):
        from yam_arena.ik import solve
        rot = arm.rot if rot is None else rot
        q, err, rerr = solve(np.asarray(tcp_target) + arm.offset, quat_xyzw(rot),
                             q_init=arm.q_cmd if q_init is None else q_init, base_pos=arm.model.base, iters=iters, tol=2e-5)
        info = self.log[-1].info
        info['worst_ik_mm'] = round(max(info.get('worst_ik_mm', 0.0), err * 1000), 3)
        info['worst_ik_deg'] = round(max(info.get('worst_ik_deg', 0.0), math.degrees(rerr)), 3)
        return q

    def _tcp_cmd(self, arm):
        return arm.model.tcp(arm.q_cmd)[0] - arm.offset

    def _move(self, arm, target, speed=SPEED, settle=6, rot=None):
        """Straight-line TCP move (and a slerp-free orientation change at the start) to ``target``."""
        if rot is not None:
            arm.rot = rot
        start = self._tcp_cmd(arm)
        dist = np.linalg.norm(np.asarray(target) - start)
        n = max(1, int(math.ceil(dist / (speed / self.hz))))
        for k in range(1, n + 1):
            arm.q_cmd = self._ik(arm, start + (np.asarray(target) - start) * k / n)
            yield
        for _ in range(settle):
            yield

    def _joint_move(self, arm, q_target, steps):
        q0 = arm.q_cmd.copy()
        for k in range(1, steps + 1):
            arm.q_cmd = q0 + (np.asarray(q_target) - q0) * k / steps
            yield

    def _clear_ready(self, arm, steps=30):
        """From (or to) the ready pose: the right arm's ready TCP sits 7 mm over the jar's rim (FK: card (323, -305,
        261)), so it first rises 60 mm and backs off 40 mm with the ready orientation (job 4296409: a direct joint move
        to the survey pose swept the jar over)."""
        r_ready = arm.model.tcp(READY)[1]
        wp = arm.model.tcp(READY)[0] + [-0.04, 0.0, 0.06]
        arm.rot = r_ready
        return wp, r_ready

    def _leave_ready(self, arm):
        wp, r_ready = self._clear_ready(arm)
        yield from self._move(arm, wp, speed=0.08, settle=2)

    def _return_ready(self, arm):
        """Back to the ready pose (left arm). The right arm parks at its clear waypoint instead: at ready its fingers are
        9.4 mm from the jar (do-nothing run, job 4296409), inside the card's 30 mm Released clearance."""
        wp, r_ready = self._clear_ready(arm)
        yield from self._goto(arm, wp, r_ready, steps=40, yaw_first=True)
        if arm.side == 'left':
            yield from self._joint_move(arm, READY, 25)

    def _goto(self, arm, target, rot, steps=50, yaw_first=False):
        """Joint-space move to a pose solved from the arm's seed (large reorientations); with ``yaw_first`` the base
        yaw turns first (the arm keeps its height), then the rest."""
        arm.rot = rot
        from yam_arena.ik import solve
        best = None
        for q0 in (arm.q_cmd, arm.seed_q, READY):
            q, err, rerr = solve(np.asarray(target) + arm.offset, quat_xyzw(rot), q_init=q0, base_pos=arm.model.base,
                                 iters=300, tol=2e-5)
            score = err + 0.02 * rerr
            if best is None or score < best[0]:
                best = (score, q, err, rerr)
        self.log[-1].info['goto_ik_mm'] = round(best[2] * 1000, 3)
        if yaw_first:
            mid = arm.q_cmd.copy()
            mid[0] = best[1][0]
            yield from self._joint_move(arm, mid, max(10, steps // 2))
        yield from self._joint_move(arm, best[1], steps)

    def _correct(self, arm, target, tol=3e-4, rounds=4, wait=10, gain=0.6):
        for _ in range(rounds):
            err = np.asarray(target) - arm.model.tcp(self.q(arm))[0]
            self.log[-1].info['tcp_error_mm'] = round(float(np.linalg.norm(err) * 1000), 3)
            if np.linalg.norm(err) < tol:
                return
            arm.offset += gain * err
            arm.q_cmd = self._ik(arm, target)
            for _ in range(wait):
                yield

    def _grip(self, arm, value, steps=12):
        g0 = arm.grip
        for k in range(1, steps + 1):
            arm.grip = g0 + (value - g0) * k / steps
            yield

    def _hold(self, n):
        for _ in range(n):
            yield

    def _survey_tcp(self, arm, point, depth, rot):
        cam_off = rot @ (arm.model.cam_pos_g - arm.model.tcp_offset)
        view = rot @ arm.model.cam_rot_g @ np.array([0.0, 0.0, -1.0])
        return np.asarray(point) - depth * view - cam_off

    def _look(self, what, centre_xy, depth):
        """Survey with the right wrist camera centred on ``centre_xy`` (dish floor height); updates the fits."""
        arm = self.R
        floor = G.TABLE_TOP + G.DISH_FLOOR_TOP * G.MM
        target = self._survey_tcp(arm, (*centre_xy, floor), depth, arm.rot)
        yield from self._move(arm, target, settle=10)
        cam = arm.model.camera(self.q(arm))
        fit = V.find_discs(np.asarray(self._obs.images['right_cam']), cam, G.TABLE_TOP)
        ev = dict(step=self.step, what=what, ok=fit.get('ok'), reason=fit.get('reason'), contrast=fit.get('contrast'),
                  px_per_mm=fit.get('px_per_mm'),
                  blobs=[dict(n=b['n'], value=round(b['value'], 3), sat=round(b['sat'], 3),
                              xy_mm=(np.asarray(b['xy']) / G.MM).round(2).tolist()) for b in fit.get('blobs', [])])
        if fit.get('ok'):
            self.cell_xy = np.asarray(fit['cell'])
            self.tablets = [np.asarray(t) for t in fit['tablets']]
            c, rms = V.lattice_centre([self.cell_xy] + self.tablets, self.dish_xy)
            if np.linalg.norm(c - self.dish_xy) < 0.03:
                self.dish_xy = c
            ev.update(cell_mm=(self.cell_xy / G.MM).round(2).tolist(), dish_mm=(self.dish_xy / G.MM).round(2).tolist(),
                      lattice_rms_mm=round(rms, 3))
        self.events.append(ev)
        return fit

    # -- the script --------------------------------------------------------------------------------------------
    def _script(self):
        L, R = self.L, self.R
        table = G.TABLE_TOP
        floor = table + G.DISH_FLOOR_TOP * G.MM
        self._phase('start')
        yield from self._hold(6)

        # 1. right arm: survey the dish
        self._phase('survey')
        R.rot = r_down(math.pi / 2)            # jaws along world y; the wrist camera on the robot side
        yield from self._grip(R, 0.6, 4)
        yield from self._leave_ready(R)
        R.rot = r_down(math.pi / 2)
        first = self._survey_tcp(R, (*self.dish_xy, floor), SURVEY_DEPTH, R.rot)
        yield from self._goto(R, first, R.rot, steps=50, yaw_first=True)
        yield from self._hold(6)
        fits = 0
        for k in range(3):
            fit = yield from self._look(f'survey {k}', self.dish_xy, SURVEY_DEPTH)
            fits += bool(fit.get('ok'))
        if self.cell_xy is None:
            self.log[-1].info['failed'] = 'no cell found'
            yield from self._finish()
            return
        fit = yield from self._look('close look', self.cell_xy, CLOSE_DEPTH)
        self.log[-1].info.update(dish_mm=(self.dish_xy / G.MM).round(2).tolist(),
                                 cell_mm=(self.cell_xy / G.MM).round(2).tolist())

        # 2. pick the cell
        held = False
        for attempt in range(3):
            self._phase('pick' if attempt == 0 else f'repick{attempt}')
            theta, clear = V.best_closing_angle(self.cell_xy, self.tablets, PICK_HALF_OPEN)
            rot = self._reachable_down(R, theta)
            self.log[-1].info.update(theta_deg=round(math.degrees(theta), 1), clearance_mm=round(clear * 1000, 2),
                                     cell_mm=(self.cell_xy / G.MM).round(2).tolist())
            tcp_z = floor + PICK_TIP_ABOVE_FLOOR + TIP_PAST_TCP
            at = np.array([*self.cell_xy, tcp_z])
            above = at + [0.0, 0.0, 0.05]
            yield from self._goto(R, above, rot, steps=30)
            yield from self._grip(R, 2 * PICK_HALF_OPEN / 0.094, 8)
            yield from self._correct(R, above)
            yield from self._move(R, at + [0.0, 0.0, PICK_TRIM_ABOVE], speed=SLOW, settle=8)
            yield from self._correct(R, at + [0.0, 0.0, PICK_TRIM_ABOVE], rounds=3)
            yield from self._move(R, at, speed=0.005, settle=8)
            yield from self._grip(R, CELL_PINCH, 24)
            yield from self._hold(16)
            r0 = self.reading(R)
            yield from self._move(R, at + [0.0, 0.0, 0.010], speed=0.004, settle=8)
            r1 = self.reading(R)
            self.log[-1].info.update(pinch_reading=round(r0, 4), lifted_reading=round(r1, 4))
            if CELL_LO < r1 < CELL_HI:
                held = True
                break
            # empty, or a tablet: put it back where it was, open, look again
            yield from self._move(R, at + [0.0, 0.0, 0.002], speed=0.01, settle=4)
            yield from self._grip(R, 2 * PICK_HALF_OPEN / 0.094, 8)
            yield from self._move(R, above, speed=0.04, settle=4)
            R.rot = r_down(math.pi / 2)
            yield from self._goto(R, self._survey_tcp(R, (*self.dish_xy, floor), SURVEY_DEPTH, R.rot), R.rot, steps=30)
            yield from self._look(f'relook {attempt}', self.dish_xy, SURVEY_DEPTH)
            yield from self._look(f'close relook {attempt}', self.cell_xy, CLOSE_DEPTH)
        self.log[-1].info['held'] = held

        # 3. to the jar
        self._phase('to_jar')
        jar = np.array(G.JA['start_centre_mm']) * G.MM
        rim = table + G.JAR_H * G.MM
        up = self._tcp_cmd(R).copy()
        up[2] = table + 0.20               # straight down is reachable to 200 mm over the dish (URDF IK), not higher
        yield from self._move(R, up, speed=0.06, settle=2)
        rot = r_tilt_forward(JAR_TILT_DEG)
        over = np.array([*jar, rim + JAR_TIP_ABOVE_RIM]) - TIP_PAST_TCP * rot[:, 2]
        # cross at the drop height, 100 mm short of the jar on the robot side, then straight forward over it
        yield from self._goto(R, np.array([over[0] - 0.08, -0.20, over[2]]), rot, steps=50)
        yield from self._move(R, over - [0.08, 0.0, 0.0], speed=0.06, settle=2)
        yield from self._move(R, over, speed=0.04, settle=8)
        yield from self._correct(R, over, rounds=3)
        self.log[-1].info['reading_over_jar'] = round(self.reading(R), 4)
        if self.mode == 'hover':
            self._phase('hover_hold')
            yield from self._hold(80)
            self.done = True
            return
        self._phase('drop')
        yield from self._grip(R, JAR_OPEN, 8)
        yield from self._hold(20)
        yield from self._move(R, over + [-0.06, 0.0, -0.005], speed=0.05, settle=2)   # back toward the base (up is out of reach)
        self._phase('right_home')
        yield from self._return_ready(R)
        yield from self._grip(R, 1.0, 6)

        # 4. left arm: the dish by its +y wall
        self._phase('rim_grasp', dish_mm=(self.dish_xy / G.MM).round(2).tolist())
        rot, u = r_rim()
        d = rot[:, 2]
        rim_top = np.array([self.dish_xy[0] + RIM_MID_R * u[0], self.dish_xy[1] + RIM_MID_R * u[1], table + G.DISH_H * G.MM])
        grasp = rim_top + (RIM_TIP_DOWN - TIP_PAST_TCP) * d
        back = grasp - 0.05 * d
        yield from self._grip(L, RIM_OPEN, 6)
        yield from self._leave_ready(L)
        yield from self._goto(L, back + [0.0, 0.0, 0.03], rot, steps=60, yaw_first=True)
        yield from self._move(L, back, speed=0.05, settle=4)
        yield from self._correct(L, back)
        yield from self._move(L, grasp, speed=SLOW, settle=8)
        yield from self._correct(L, grasp, rounds=3)
        yield from self._grip(L, RIM_PINCH, 14)
        yield from self._hold(14)
        self.log[-1].info['pinch_reading'] = round(self.reading(L), 4)

        # 5. carry over the tray wall and set it on the tray
        self._phase('carry')
        place = np.array(G.TR['start_centre_mm']) * G.MM            # dish centre target: the tray's nominal centre
        delta = np.array([place[0] - self.dish_xy[0], place[1] - self.dish_xy[1], 0.0])
        p0 = self._tcp_cmd(L)
        yield from self._move(L, p0 + [0.0, 0.0, 0.025], speed=0.01, settle=6)
        self.log[-1].info['lifted_reading'] = round(self.reading(L), 4)
        yield from self._move(L, p0 + [0.0, 0.0, CARRY_BOTTOM], speed=0.03, settle=6)
        yield from self._move(L, p0 + delta + [0.0, 0.0, CARRY_BOTTOM], speed=0.04, settle=10)
        yield from self._move(L, p0 + delta + [0.0, 0.0, PLACE_BOTTOM + 0.03], speed=0.03, settle=4)
        yield from self._move(L, p0 + delta + [0.0, 0.0, PLACE_BOTTOM], speed=0.008, settle=12)
        self.log[-1].info['placed_reading'] = round(self.reading(L), 4)
        self._phase('release')
        yield from self._grip(L, RIM_OPEN, 14)
        yield from self._hold(10)
        here = self._tcp_cmd(L)
        yield from self._move(L, here - 0.05 * d, speed=0.02, settle=2)
        yield from self._move(L, here - 0.05 * d + [0.0, 0.0, 0.06], speed=0.05, settle=2)
        yield from self._finish()

    def _finish(self):
        self._phase('home')
        if np.abs(self.L.q_cmd - READY).max() > 1e-6:
            yield from self._return_ready(self.L)
        yield from self._grip(self.L, 1.0, 6)
        if np.abs(self.R.q_cmd - READY).max() > 1e-6:
            yield from self._return_ready(self.R)
        self._phase('final_hold')
        yield from self._hold(44)
        self.done = True

    def _reachable_down(self, arm, theta):
        """Of the two jaw directions on the closing line (theta, theta + pi), the one IK reaches with less wrist roll."""
        from yam_arena.ik import solve
        target = np.array([*self.cell_xy, G.TABLE_TOP + 0.05])
        best = None
        for th in (theta, theta + math.pi):
            rot = r_down(th)
            q, err, rerr = solve(target, quat_xyzw(rot), q_init=arm.seed_q, base_pos=arm.model.base, iters=300, tol=2e-5)
            score = err * 1000 + 5 * rerr + 0.2 * abs(q[5])
            if best is None or score < best[0]:
                best = (score, rot)
        return best[1]

    # -- policy interface -------------------------------------------------------------------------------------
    def act(self, observation):
        self._obs = observation
        if not self.done:
            try:
                next(self._plan)
            except StopIteration:
                self.done = True
        self.step += 1
        return np.r_[self.L.q_cmd, self.L.grip, self.R.q_cmd, self.R.grip]

    def report(self):
        if self.log and self.log[-1].end is None:
            self.log[-1].end = self.step
        return dict(phases=[dict(name=p.name, start=p.start, end=p.end, **_jsonable(p.info)) for p in self.log],
                    events=_jsonable(self.events),
                    offsets_mm=dict(left=(self.L.offset * 1000).round(3).tolist(), right=(self.R.offset * 1000).round(3).tolist()))


def _jsonable(x):
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return x.tolist()
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return x.item()
    return x


def make_policy(cam_width, cam_height, control_hz, mode='demo'):
    return BatteryDemo(control_hz, mode, cam_width, cam_height)
