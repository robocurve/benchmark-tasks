"""A kinematic stand-in for the simulator, for dry runs of the demonstration and of the scorer. No contact physics.

- Each arm follows its joint commands with a first-order lag; fingertip points come from the arm's kinematics.
- A pad goes down to the height of a fingertip that is over it (never further than its travel), is held at the
  underside of the frame's top once any of it is under the frame, and is otherwise back at its rest height at
  once (harsher than the real spring-back, which takes a fraction of a second).
- The drawer moves in only as far as the pushing fingertips have got, less the arm's give under the sliding
  friction, and no further than the point where a pad that stands above the opening meets the frame.

This checks reach, joint limits, control flow and the scorer's rules. It says nothing about contact forces.
"""

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN, READY

from rdb_overfull_drawer import fixed_scene as fixture
from rdb_overfull_drawer import layout as L
from rdb_overfull_drawer.scripted import TIP

TIP_CORNER = .0045            # a pitched fingertip touches with a corner about this far from its middle
FINGER_HALF = .0025           # a fingertip's middle is this far outside its inner face


class World:
    def __init__(self, *, lag_s=.12, arm_stiffness=500., press_offset=(0., 0., 0.), push_offset=0., slack=0.):
        self.blend = 1. - np.exp(-1. / fixture.CONTROL_HZ / lag_s)
        self.arm_stiffness = arm_stiffness
        self.press_offset = np.asarray(press_offset, dtype=float)     # where the pressing hand really is vs its joints
        self.push_offset = push_offset                                # the same for the pushing hand, along x
        self.slack = slack                                            # the pillow's shift inside the drawer
        self.joints = np.concatenate([np.r_[READY, 1.], np.r_[READY, 1.]])
        self.open = L.OPEN_START
        self.rest = np.array([[L.FLOOR_Z + L.REST_TOP[row]] * L.COLS for row in range(L.ROWS)])
        self.tops = self.rest.copy()
        self.pillow_removed = False
        self.touching = False
        self._settle()

    # -- geometry ---------------------------------------------------------------------------
    def front(self):
        return L.drawer_x(self.open)

    def leads(self):
        return np.array([self.front() + L.row_lead(row) + self.slack for row in range(L.ROWS)])

    def hand(self, index):
        point, rotation = ik.tcp_pose(self.joints[7 * index:7 * index + 6], fixture.BASES[fixture.ARMS[index]])
        return point, rotation

    def fingertip_points(self, index):
        """Ends of the two fingertips of one hand."""
        point, rotation = self.hand(index)
        middle = point + TIP * rotation[:, 2]
        half = abs(float(self.joints[7 * index + 6]) * FINGER_OPEN) + FINGER_HALF
        return [middle + sign * half * rotation[:, 0] for sign in (1., -1.)]

    def boxes(self):
        """(low, high) of the drawer's boxes, the pillow's base and its pads where they are now, tabletop frame."""
        from build_scene import drawer_boxes

        front = np.array((self.front(), 0., 0.))
        out = [(np.asarray(low) + front, np.asarray(high) + front) for _, low, high in drawer_boxes()]
        centre = self.front() + L.FRONT_T + L.DRAWER_INNER[0] / 2 + self.slack
        half = np.array(L.BASE_SIZE) / 2
        out.append((np.array((centre - half[0], -half[1], L.FLOOR_Z)), np.array((centre + half[0], half[1], L.FLOOR_Z + L.BASE_SIZE[2]))))
        leads = self.leads()
        for row in range(L.ROWS):
            for col in range(L.COLS):
                top = self.tops[row, col]
                out.append((np.array((leads[row] - L.PAD_SIZE[0], L.PAD_Y[col] - L.PAD_SIZE[1] / 2, top - L.PAD_SIZE[2])),
                            np.array((leads[row], L.PAD_Y[col] + L.PAD_SIZE[1] / 2, top))))
        return out

    def state(self):
        centre = self.front() + L.FRONT_T + L.DRAWER_INNER[0] / 2 + self.slack
        base = (centre, 0., L.FLOOR_Z + .0005) if not self.pillow_removed else (.20, .40, .0005)
        return {"open": self.open, "pad_top": self.tops.copy(), "pad_lead": self.leads(),
                "base": np.array(base)}

    # -- one control step -------------------------------------------------------------------
    def _settle(self, pressers=()):
        leads = self.leads()
        self.touching = False
        for row in range(L.ROWS):
            for col in range(L.COLS):
                top = self.rest[row, col]
                x1, x0 = leads[row], leads[row] - L.PAD_SIZE[0]
                y0, y1 = L.PAD_Y[col] - L.PAD_SIZE[1] / 2, L.PAD_Y[col] + L.PAD_SIZE[1] / 2
                for p in pressers:
                    if x0 <= p[0] <= x1 and y0 <= p[1] <= y1 and p[2] < top:
                        top = p[2]
                        self.touching = True
                top = max(top, self.rest[row, col] - L.PAD_TRAVEL)
                if leads[row] > L.RAIL_X + .0005:
                    top = min(top, L.RAIL_Z)
                self.tops[row, col] = top

    def step(self, command):
        command = np.asarray(command, dtype=float)
        self.joints = self.joints + self.blend * (command - self.joints)
        pressers = []
        for index in range(2):
            for p in self.fingertip_points(index):
                pressers.append(p + self.press_offset - (TIP_CORNER, 0., TIP_CORNER) if index == 0 else p)
        self._settle(pressers)
        # The pushing hand: its fingertips against the drawer front.
        push = [p for p in self.fingertip_points(1)]
        low, high = L.FRONT_Z
        reach = max((p[0] + TIP_CORNER + self.push_offset for p in push
                     if abs(p[1]) <= L.FRONT_WIDTH / 2 and low <= p[2] <= high), default=-np.inf)
        under = sum(L.COLS * L.PAD_STIFFNESS * (L.PAD_PRELOAD_M + self.rest[row, 0] - L.RAIL_Z)
                    for row in range(L.ROWS) if self.leads()[row] > L.RAIL_X and self.rest[row, 0] > L.RAIL_Z)
        friction = L.RUNNER_FRICTION_N + .3 * under
        wanted = reach - friction / self.arm_stiffness            # how far the arm's give lets the front be pushed
        if reach >= self.front() - 1e-4:
            self.touching = True
        while wanted > self.front() + 1e-6 and self.open > 0.:
            advance = min(.001, wanted - self.front(), self.open)
            leads = self.leads()
            for row in range(L.ROWS):            # a pad standing above the opening stops at the frame
                if leads[row] <= L.RAIL_X + 1e-9 and self.tops[row].max() > L.RAIL_Z - .0005:
                    advance = min(advance, L.RAIL_X - leads[row])
            if advance <= 1e-9:
                break
            self.open -= advance
            self._settle(pressers)
        return self.joints.copy()

    def grade_inputs(self):
        state = self.state()
        tcp = [self.hand(index)[0] for index in range(2)]
        return state["open"], state["base"], (0., 0., 1.), self.tops.ravel(), tcp, self.touching
