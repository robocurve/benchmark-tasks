"""A screw cap as a virtual thread, for a cap that is its own rigid body and must come completely off.

A joint cannot do this (a cap on a prismatic joint stays on its rail, in the way of the pour). Instead, until it has
been turned, the cap is held on its seat: before every physics substep (240 Hz) this module reads the cap's pose
relative to its parent (the pouch or cartridge), keeps only its turn about the parent's z axis, clamps that turn to
0..TURN (opening direction: counter-clockwise seen from above), and puts the cap back on its seat at that turn,
raised CAP_RISE * turn / TURN above it, moving with the parent. Whatever a gripper does to the cap besides turning it
is undone, so the cap only turns. When it has turned TURN it is released and is an ordinary free body: lift it away,
set it down. If a released cap is brought back down onto its seat (after having been lifted clear), it is seated
again, fully open (turn = TURN), and turning it back to 0 closes it.

`closed` = seated and turned back to within CLOSED_DEG of shut. `off` = released and at least OFF_DIST from the seat.
"""

from __future__ import annotations

import math

import numpy as np

CLOSED_DEG = 4.0
OFF_DIST = 0.04  # m from the seat for "taken off"
ARM_DIST = 0.015  # a released cap must first get this far above its seat before it can seat again
SEAT_LATERAL = 0.003  # m, to re-seat
SEAT_TILT_DEG = 10.0
HOLD_NEAR, HOLD_SEP = 0.022, (0.060, 0.082)  # m from the cap centre to the pads' centre; pad separation (tip link to tip link)
THREAD_DRAG = 0.9  # per substep: the thread's own friction on the cap's turning speed


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def qinv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qrot(q, v):
    x, y, z, w = q
    r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                  [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                  [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return r @ np.asarray(v, float)


def qz(angle):
    return np.array([0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2)])


class CapSeat:
    def __init__(self, cap, parent, seat_z: float, turn_deg: float, rise: float, robots=(), offset=(0.0, 0.0)):
        """cap, parent: Isaac Lab RigidObjects. seat_z: the cap centre's height above the parent's origin, seated
        and shut."""
        self.cap, self.parent = cap, parent
        self.seat_z, self.turn, self.rise = seat_z, math.radians(turn_deg), rise
        self.ox, self.oy = offset  # the seat's place in the parent's frame (the cap's axis need not be the parent's z axis)
        self.state, self.theta, self.armed = "seated", 0.0, False
        # the two arms' robot assets: while a hand holds the cap (pads closed on it) the cap turns with the hand
        self.hands = []
        for robot in robots:
            ids = robot.find_bodies(["tip_left", "tip_right", "gripper"], preserve_order=True)[0]
            self.hands.append((robot, ids))
        self._hand_yaw = {}
        write = cap.write_data_to_sim

        def write_with_seat():
            self.update()
            write()

        cap.write_data_to_sim = write_with_seat

    def _pose(self, body):
        p = _np(body.data.root_link_pose_w)[0]
        return p[:3], p[3:7]

    def reset(self) -> None:
        self.state, self.theta, self.armed = "seated", 0.0, False

    def seat_world(self, theta=0.0):
        pp, pq = self._pose(self.parent)
        return pp + qrot(pq, [self.ox, self.oy, self.seat_z + self.rise * theta / self.turn])

    def _hand_turn(self, cp) -> float | None:
        """How far (rad, about z) a hand that has the cap between its closed pads turned since the last substep;
        None if no hand holds it. Held: the pads' centre (TCP) within HOLD_NEAR of the cap's centre and the pads
        closed on something 60-82 mm apart (tip link to tip link; fully open is 48, closed on nothing 101)."""
        for k, (robot, ids) in enumerate(self.hands):
            pos = _np(robot.data.body_pos_w)[0][ids]
            quat = _np(robot.data.body_quat_w)[0][ids[2]]
            a, b = pos[0], pos[1]
            tcp = (a + b) / 2 + qrot(quat, [0, 0, 1.0]) * 0.075  # the gripper's +z is its approach axis
            sep = float(np.linalg.norm(b - a))
            held = float(np.linalg.norm(tcp - cp)) < HOLD_NEAR and HOLD_SEP[0] < sep < HOLD_SEP[1]
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            prev = self._hand_yaw.get(k)
            self._hand_yaw[k] = yaw
            if held and prev is not None:
                return (yaw - prev + math.pi) % (2 * math.pi) - math.pi
        return None

    def update(self) -> None:
        pp, pq = self._pose(self.parent)
        cp, cq = self._pose(self.cap)
        if self.state == "seated":
            rel = qmul(qinv(pq), cq)
            yaw = 2 * math.atan2(rel[2], rel[3])
            yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
            turn = self._hand_turn(cp) if self.hands else None
            self.theta = min(max(self.theta + turn if turn is not None else yaw, 0.0), self.turn)
            if self.theta >= self.turn - 1e-3:
                self.state, self.armed = "free", False
                return
            self._place(pp, pq, self.theta)
        else:
            dist = float(np.linalg.norm(cp - self.seat_world(self.turn)))
            if dist > ARM_DIST:
                self.armed = True
            if self.armed:
                loc = qrot(qinv(pq), cp - pp)
                tilt = math.degrees(2 * math.asin(min(1.0, math.hypot(*qmul(qinv(pq), cq)[:2]))))
                if (math.hypot(loc[0] - self.ox, loc[1] - self.oy) < SEAT_LATERAL and self.seat_z - 0.001 < loc[2] < self.seat_z + self.rise
                        + 0.002 and tilt < SEAT_TILT_DEG):
                    self.state, self.theta = "seated", self.turn - 1e-3
                    self._place(pp, pq, self.theta)

    def _place(self, pp, pq, theta) -> None:
        """Put the cap back on its seat at turn theta, moving with the parent. Its turning speed (about the parent's
        z axis) is kept, less a little thread friction, so a gripper's friction can spin it up over several substeps;
        at either end of the turn the speed is dropped."""
        import torch

        dev = self.cap.device
        pos = pp + qrot(pq, [self.ox, self.oy, self.seat_z + self.rise * theta / self.turn])
        quat = qmul(pq, qz(theta))
        pv = _np(self.parent.data.root_link_vel_w)[0]
        cv = _np(self.cap.data.root_com_vel_w)[0]
        zaxis = qrot(pq, [0, 0, 1.0])
        spin = float((cv[3:6] - pv[3:6]) @ zaxis) * THREAD_DRAG
        if (theta <= 0.0 and spin < 0) or (theta >= self.turn and spin > 0):
            spin = 0.0
        lin = pv[:3] + np.cross(pv[3:6], pos - pp)
        self.cap.write_root_pose_to_sim_index(root_pose=torch.tensor([[*pos, *quat]], dtype=torch.float32, device=dev))
        self.cap.write_root_velocity_to_sim_index(
            root_velocity=torch.tensor([[*lin, *(pv[3:6] + spin * zaxis)]], dtype=torch.float32, device=dev))

    # -- readouts for scoring (valid after a step)
    def closed(self) -> bool:
        return self.state == "seated" and math.degrees(self.theta) <= CLOSED_DEG

    def distance(self) -> float:
        cp, _ = self._pose(self.cap)
        return float(np.linalg.norm(cp - self.seat_world(0.0)))

    def off(self) -> bool:
        return self.state == "free" and self.distance() >= OFF_DIST
