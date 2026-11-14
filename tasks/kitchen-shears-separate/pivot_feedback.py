"""Pose-only pivot-centred alignment and gated axial-lift controller."""
import numpy as np


def rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


class PivotFeedback:
    def __init__(self):
        self.reset()

    def reset(self):
        self.tcp = self.rot = None
        self.last = None
        self.good_since = self.lift_at = None
        self.lift = 0.
        self.angle_trim = 0.
        self.translation = np.zeros(3)
        self.offset = self.angle_error = float('inf')

    def update(self, t, positions, rotations, nominal_tcp, nominal_rot):
        p0, p1 = np.asarray(positions[:2], float)
        r0, r1 = np.asarray(rotations[:2], float)
        if self.tcp is None:
            self.tcp = np.asarray(nominal_tcp, float).copy()
            self.rot = np.asarray(nominal_rot, float).copy()
        d = np.r_[(p1-p0)[:2], 0.]
        yaw = lambda r: float(np.arctan2(r[1, 0], r[0, 0]))
        rel = (yaw(r1)-yaw(r0)+np.pi) % (2*np.pi)-np.pi
        self.angle_error = float((-np.pi/2-rel+np.pi) % (2*np.pi)-np.pi)
        self.offset = float(np.linalg.norm(d))
        movement = float('inf') if self.last is None else float(np.linalg.norm(
            np.asarray([p0, p1])-self.last, axis=1).max())
        self.last = np.asarray([p0, p1]).copy()

        # Correct the commanded TCP about the currently SEEN physical pivot.
        # A yaw correction must not translate the pivot about the loop grip.
        if movement <= .0005:
            da = float(np.clip(.15*self.angle_error, -np.radians(.25), np.radians(.25)))
            next_angle = float(np.clip(self.angle_trim+da, -np.radians(15), np.radians(15)))
            delta = rz(next_angle-self.angle_trim)
            self.tcp = p1+delta@(self.tcp-p1)
            self.rot = delta@self.rot
            self.angle_trim = next_angle
            dp = -np.clip(.15*d, -.0002, .0002)
            next_translation = np.clip(self.translation+dp, -.012, .012)
            self.tcp += next_translation-self.translation
            self.translation = next_translation

        aligned = self.offset < .0004 and abs(self.angle_error) <= np.radians(2.)
        stable = movement < .0002
        if self.lift_at is None:
            if aligned and stable:
                if self.good_since is None:
                    self.good_since = float(t)
                if t-self.good_since >= .5-1e-9:
                    self.lift_at = float(t)
            else:
                self.good_since = None
        if self.lift_at is not None:
            # A four-second, 20 mm axial lift. This modifies only a robot target.
            self.lift = .020*float(np.clip((t-self.lift_at)/4., 0., 1.))
        return self.target(nominal_tcp, nominal_rot)

    def target(self, nominal_tcp, nominal_rot):
        if self.tcp is None:
            return np.asarray(nominal_tcp), np.asarray(nominal_rot)
        return self.tcp + np.array([0., 0., self.lift]), self.rot

    def diagnostic(self):
        return dict(offset_mm=self.offset*1000, angle_error_deg=float(np.degrees(self.angle_error)),
                    good_since=self.good_since, lift_at=self.lift_at, lift_mm=self.lift*1000,
                    angle_trim_deg=float(np.degrees(self.angle_trim)),
                    translation_mm=(self.translation*1000).tolist())
