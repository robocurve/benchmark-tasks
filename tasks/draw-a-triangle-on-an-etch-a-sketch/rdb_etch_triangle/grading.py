"""Score the triangle drawn on the Etch A Sketch.

The simulator samples the logged stylus path. The photo tool supplies rectified
centreline samples to the same drawing checks. Trial video supplies the real
toy-motion and final release checks. The module needs only NumPy.

Drawing frame: millimetres from the start dot, X to the right, Y up the screen. The ideal triangle is
A = (0, 0), B = (80, 0), C = (80, 60).
"""

from dataclasses import dataclass, field

import numpy as np

from . import layout

CONTROL_HZ = 20
NEAR_MM = 2.                         # a drawn point "on" a side: within 2 mm of it
SIDES = {"bottom": ((0., 0.), (80., 0.)), "upright": ((80., 0.), (80., 60.)), "slant": ((80., 60.), (0., 0.))}
NEEDED_MM = {"bottom": 70, "upright": 50, "slant": 100}      # of 80, 60, 100: the 1 mm pieces a tier needs covered
SAMPLE_MM = .25                      # the line between two logged points is checked every quarter millimetre
STEP_MM = 1.                         # a step: a straight piece of line longer than 1 mm ...
STEP_DEG = 10.                       # ... within 10 degrees of horizontal or vertical (the slanted side is at 37)
_STEP_TAN = float(np.tan(np.radians(STEP_DEG)))
MIN_RUN_MM = .2                      # a piece is judged once it is this long
CORNER_CLEAR_MM = 5.                 # within 5 mm of A or C the slanted side meets the other sides: not checked
STRAY_CAP = .25                      # any stray line caps the score here
MOVED_M = .010                       # toy pushed out of place: its centre 10 mm from its marks
TURNED_RAD = float(np.radians(5.))   # or turned 5 degrees
LIFTED_M = .002                      # any bottom corner of the toy 2 mm off the table: lifted, tilted or shaken
LET_GO_TICKS = CONTROL_HZ            # no part of either arm touches the toy for the last second
TIERS = (.25, .5, 1.)
_CORNERS = np.array([(sx * layout.BODY[0] / 2, sy * layout.BODY[1] / 2, 0.) for sx in (-1, 1) for sy in (-1, 1)])


def side_distance(point, side):
    """(distance from the point to the side, position of its foot along the side), mm."""
    a, b = (np.asarray(v, dtype=float) for v in SIDES[side])
    length = float(np.linalg.norm(b - a))
    along = float(np.clip((np.asarray(point) - a) @ (b - a) / length, 0., length))
    return float(np.linalg.norm(np.asarray(point) - (a + (b - a) * along / length))), along


def triangle_distance(point):
    return min(side_distance(point, side)[0] for side in SIDES)


@dataclass
class TriangleGrade:
    start_xy: tuple                       # toy centre on its marks (station frame, metres)
    start_yaw: float
    covered: dict = field(default_factory=lambda: {side: np.zeros(int(round(np.linalg.norm(np.subtract(*SIDES[side])))),
                                                                   dtype=bool) for side in SIDES})
    tier: float = 0.                      # from coverage and steps alone, before the stray cap
    stray: bool = False                   # a drawn point more than 2 mm from every side
    stray_at_mm: list | None = None
    longest_step_mm: float = 0.           # longest run along one axis alone on the slanted side
    step_at_mm: list | None = None
    anchor: np.ndarray | None = None      # where the current run on the slanted side started
    void: str | None = None               # toy lifted, tilted or pushed out of place: the score is 0
    let_go_ticks: int = 0
    last: np.ndarray = field(default_factory=lambda: np.zeros(2))
    path_mm: float = 0.

    def sample(self, point):
        """Accumulate one centreline sample in mm, in order along a continuous stroke."""
        if triangle_distance(point) > NEAR_MM:
            if not self.stray:
                self.stray, self.stray_at_mm = True, [round(float(v), 2) for v in point]
            self.anchor = None
            return
        for side, bins in self.covered.items():
            distance, along = side_distance(point, side)
            if distance <= NEAR_MM:
                # the 1 mm piece the point lies in (a point exactly on a piece's start belongs to the piece before)
                bins[min(max(int(np.ceil(along)) - 1, 0), len(bins) - 1)] = True
        distance, along = side_distance(point, "slant")
        if distance > NEAR_MM or not CORNER_CLEAR_MM <= along <= 100. - CORNER_CLEAR_MM:
            self.anchor = None
            return
        if self.anchor is None:
            self.anchor = point.copy()
            return
        dx, dy = np.abs(point - self.anchor)
        if max(dx, dy) >= MIN_RUN_MM and min(dx, dy) > _STEP_TAN * max(dx, dy):
            self.anchor = point.copy()                  # the line is running at a slant here: a new piece starts
        elif max(dx, dy) > self.longest_step_mm:
            self.longest_step_mm = float(max(dx, dy))
            if self.longest_step_mm > STEP_MM and self.step_at_mm is None:
                self.step_at_mm = [round(float(v), 2) for v in point]

    def draw(self, point_mm):
        """The stylus moved from the last logged point to ``point_mm`` during one tick."""
        point = np.asarray(point_mm, dtype=float)
        step = point - self.last
        length = float(np.linalg.norm(step))
        if length < 1e-6:
            return
        count = max(1, int(np.ceil(length / SAMPLE_MM)))
        for k in range(1, count + 1):
            self.sample(self.last + step * k / count)
        self.path_mm += length
        self.last = point

    def counts(self):
        return {side: int(bins.sum()) for side, bins in self.covered.items()}

    def drawing_result(self):
        """Drawing diagnostics before applying the separately observed trial conditions."""
        covered = self.counts()
        tier = 0.
        if covered["bottom"] >= NEEDED_MM["bottom"]:
            tier = .25
            if covered["upright"] >= NEEDED_MM["upright"]:
                tier = .5
                if covered["slant"] >= NEEDED_MM["slant"] and self.longest_step_mm <= STEP_MM:
                    tier = 1.
        self.tier = tier
        drawn = min(tier, STRAY_CAP) if self.stray else tier
        return {"tier": tier, "drawn_score": drawn, "covered_mm": covered, "stray": self.stray,
                "stray_at_mm": self.stray_at_mm, "longest_step_mm": round(self.longest_step_mm, 2),
                "step_at_mm": self.step_at_mm}

    def update(self, stylus_point_m, toy_position, toy_rotation, arm_touching_toy):
        """One 20 Hz tick. stylus_point_m: where the stylus is, metres from the start dot (right, up);
        toy_position, toy_rotation: the toy in the station frame (origin on the table under its centre);
        arm_touching_toy: any part of either arm touches the frame or a knob."""
        self.draw(1000. * np.asarray(stylus_point_m, dtype=float))
        position = np.asarray(toy_position, dtype=float)
        rotation = np.asarray(toy_rotation, dtype=float)
        offset = float(np.linalg.norm(position[:2] - np.asarray(self.start_xy)))
        yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
        turned = abs((yaw - self.start_yaw + np.pi) % (2 * np.pi) - np.pi)
        highest = float((position[2] + _CORNERS @ rotation[2]).max())
        if self.void is None:
            if highest > LIFTED_M:
                self.void = "toy lifted or tilted"
            elif offset > MOVED_M or turned > TURNED_RAD:
                self.void = "toy pushed off its marks"
        drawing = self.drawing_result()
        self.let_go_ticks = 0 if arm_touching_toy else self.let_go_ticks + 1
        let_go = self.let_go_ticks >= LET_GO_TICKS
        score = 0. if self.void is not None or not let_go else drawing["drawn_score"]
        return {"score": score, **drawing, "stylus_mm": [round(float(v), 2) for v in self.last],
                "let_go": let_go, "void": self.void, "toy_offset_mm": round(1000 * offset, 2),
                "toy_turned_deg": round(float(np.degrees(turned)), 2), "toy_highest_corner_mm": round(1000 * highest, 2),
                "line_mm": round(self.path_mm, 1)}

    @property
    def finished(self):
        """Nothing more can change the score."""
        return self.void is not None or (self.tier == 1. and not self.stray and self.let_go_ticks >= LET_GO_TICKS)


@dataclass(frozen=True)
class TriangleScorer:
    name: str = "etch_triangle_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["triangle_grade"]
        return Score(value=grade["score"],
                     explanation=f"Tier {grade['tier']}; covered {grade['covered_mm']}; longest step "
                                 f"{grade['longest_step_mm']} mm; stray {grade['stray']}; let go {grade['let_go']}; "
                                 f"void {grade['void']}")


def make_scorer():
    return TriangleScorer()
