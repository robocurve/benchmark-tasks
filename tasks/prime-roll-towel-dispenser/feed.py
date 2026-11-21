"""Reduced roll-towel leader and one-way feed kinematics, in SI units."""
from dataclasses import dataclass, field
import math
import numpy as np

WIDTH = 0.2032
SAMPLE_Y = np.array([0.075, -0.075])
LEADER_RESET = np.column_stack((np.full(2, 0.354), SAMPLE_Y, np.full(2, 0.982)))
NIP_X = 0.428
NIP_Z = 0.945
LEAD_LENGTH = .040
OUTLET_Z = 0.821
FEED_PER_RAD = 0.084
TRAVEL_LIMIT = 0.300
DIVERGENCE_LIMIT = 0.025
BACKLASH = math.radians(2.)
RETURN_ANGLE = math.radians(1.)


@dataclass
class Feed:
    """Each grip holds one sampling point on the same continuous paper edge."""

    leader: np.ndarray = field(default_factory=lambda: LEADER_RESET.copy())
    grip_to_point: dict = field(default_factory=dict)
    previous_apertures: np.ndarray = field(default_factory=lambda: np.full(2, 0.075))
    swept_apertures: np.ndarray = field(default_factory=lambda: np.full(2, 0.075))
    armed: np.ndarray = field(default_factory=lambda: np.ones(2, dtype=bool))
    seated: bool = False
    travel: float = 0.
    stroke_depth: float = 0.
    damaged: bool = False
    dwell: int = 0

    def update(self, tcp, axes, apertures, lever):
        if not (np.isfinite(tcp).all() and np.isfinite(axes).all()
                and np.isfinite(apertures).all() and math.isfinite(lever)):
            self.damaged = True
            return
        for hand, (point_id, offset) in list(self.grip_to_point.items()):
            if apertures[hand] >= 0.012:
                del self.grip_to_point[hand]
                continue
            self.leader[point_id] = tcp[hand] + axes[hand] @ offset
        if not self.seated:
            # Tension in the continuous web carries a released edge along.
            # Both endpoints stay independent when the hands constrain them.
            if len(self.grip_to_point) == 1:
                held, _ = next(iter(self.grip_to_point.values()))
                free = 1 - held
                relaxed = self.leader[held].copy()
                relaxed[1] = SAMPLE_Y[free] + self.leader[held, 1] - SAMPLE_Y[held]
                self.leader[free] += .25 * (relaxed - self.leader[free])
            for hand in range(2):
                if apertures[hand] >= .020:
                    if apertures[hand] >= self.previous_apertures[hand]:
                        self.swept_apertures[hand] = apertures[hand]
                    self.armed[hand] = True
                if (hand in self.grip_to_point or not self.armed[hand]
                        or not apertures[hand] <= .008 < self.previous_apertures[hand]):
                    continue
                self.armed[hand] = False
                half_gap = self.swept_apertures[hand] / 2
                for point_id in np.argsort(np.linalg.norm(self.leader - tcp[hand], axis=1)):
                    if any(j == point_id for j, _ in self.grip_to_point.values()):
                        continue
                    delta = axes[hand].T @ (self.leader[point_id] - tcp[hand])
                    # The pads sweep this gap on closing. Once held,
                    # the edge lies between their inner faces.
                    if (abs(delta[0]) <= min(half_gap, .012) and abs(delta[1]) <= .011
                            and -.018 <= delta[2] <= .010
                            and abs(axes[hand, 0, 0]) >= .85):
                        delta[0] = np.clip(delta[0], -.001, .001)
                        self.grip_to_point[hand] = (point_id, delta)
                        break

            if len(self.grip_to_point) == 2 and np.linalg.norm(
                    (self.leader[0] - self.leader[1]) - [0, .15, 0]) > DIVERGENCE_LIMIT:
                self.damaged = True
            centred = np.all(np.abs(self.leader[:, 1] - SAMPLE_Y) <= .007)
            in_guide = (np.all(np.abs(self.leader[:, 0] - NIP_X) <= .007)
                        and np.all((self.leader[:, 2] - LEAD_LENGTH >= NIP_Z - .006)
                                   & (self.leader[:, 2] - LEAD_LENGTH <= NIP_Z + .004)))
            square = abs(self.leader[0, 2] - self.leader[1, 2]) <= .008
            if self.grip_to_point and centred and in_guide and square:
                self.seated = True
                self.leader[:, 0] = NIP_X
                self.leader[:, 2] = NIP_Z
                self.grip_to_point.clear()
        else:
            if -lever <= RETURN_ANGLE:
                self.stroke_depth = 0.
            else:
                driven = max(0., -lever - BACKLASH)
                increment = max(0., driven - self.stroke_depth)
                self.stroke_depth = max(self.stroke_depth, driven)
                self.travel = min(TRAVEL_LIMIT, self.travel + FEED_PER_RAD * increment)
            self.leader[:, 2] = NIP_Z - self.travel

        self.previous_apertures = np.asarray(apertures).copy()
        done = self.exposed >= .050 and not self.damaged
        lever_released = abs(lever) < math.radians(3.)
        clear_of_edge = np.linalg.norm(tcp[:, None] - self.leader[None, :], axis=2).min() > .050
        clear_of_lever = np.linalg.norm(tcp - [.290, -.165, .930], axis=1).min() > .050
        self.dwell = self.dwell + 1 if done and lever_released and clear_of_edge and clear_of_lever else 0

    @property
    def exposed(self):
        return max(0., self.travel - (NIP_Z - OUTLET_Z))

    @property
    def score(self):
        if self.damaged:
            return 0.
        if self.dwell >= 10:
            return 1.
        if self.exposed >= .015:
            return .5
        return .25 if self.seated else 0.
