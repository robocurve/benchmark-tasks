"""Episode history and numerical scoring, independent of Isaac Sim."""
from dataclasses import dataclass
import numpy as np


@dataclass
class History:
    tool_contact: bool = False
    extracted: bool = False
    invalid: bool = False
    open_clear: bool = False
    contact_events: int = 0

    def contact(self, a, b):
        names = (a.lower(), b.lower())
        if not any("/part/" in p or p.endswith("/part") for p in names):
            return
        if any("/spatula/" in p or p.endswith("/spatula") for p in names):
            self.tool_contact = True
            self.contact_events += 1
        if not self.extracted and any(any(robot in p for robot in ("left_robot", "right_robot", "leftarm", "rightarm")) for p in names):
            self.invalid = True

    def update(self, objects, top):
        for key in ("part", "spatula"):
            s = objects[key]
            if (not np.all(np.isfinite(s.pos)) or s.low[2] < top - .02
                    or abs(s.pos[1]) > .62 or not -.15 < s.pos[0] < .96):
                self.invalid = True
        p = objects["part"]
        if self.tool_contact and p.high[0] < .305 and p.low[2] >= top - .003:
            self.extracted = True


def part_in_tray(s, top):
    return bool(.185 < s.low[0] and s.high[0] < .285
                and -.225 < s.low[1] and s.high[1] < -.125
                and abs(s.low[2] - (top + .004)) < .004
                and s.resting())


def tool_parked(s, top):
    return bool(np.linalg.norm(s.pos[:2] - (.24, .27)) < .025
                and abs(s.low[2] - (top + .035)) < .006
                and s.high[2] - s.low[2] < .025 and s.resting())


def evaluate(objects, history, top):
    if history.invalid or not history.extracted:
        return 0.0
    if not part_in_tray(objects["part"], top):
        return .25 if objects["part"].high[0] < .305 else 0.0
    if tool_parked(objects["spatula"], top) and history.open_clear:
        return 1.0
    return .75
