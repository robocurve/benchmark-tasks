"""Native conveyor packing with loose cans and an independent outcome scorer."""

from dataclasses import dataclass
import math

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.ik import smoothstep
from yam_arena.task_file import Box, TABLE_TOP

from dynamic_task_fluid_head_subject_tracking.geometry import Solid
from dynamic_task_fluid_head_subject_tracking.mechanisms import conveyor_frame


TOP = TABLE_TOP["station"]
BELT_SPEED = 0.055
LONG_TABLE_LENGTH = 2.320
DISCHARGE_STEP = 0.004
CAN_RADIUS = 0.033
CAN_HEIGHT = 0.122
CAN_GRASP = 0.045
CAN_COLOR = (0.70, 0.16, 0.64)
CRATE_COLOR = (0.12, 0.55, 0.31)
FLOOR_THICKNESS = 0.008
WALL_THICKNESS = 0.008
CRATE_POCKET_OPENING = 0.104
CRATE_POCKET_HEIGHT = 0.025


@dataclass(frozen=True)
class Transfer:
    arm: str
    can: str
    crate: str
    drop_offset: tuple[float, float]


def open_crate(size):
    """An open box with a separate solid floor and four collision walls."""
    x, y, h = size
    w = WALL_THICKNESS
    return (
        Solid("box", (x, y, FLOOR_THICKNESS), (0, 0, (FLOOR_THICKNESS - h) / 2)),
        Solid("box", (w, y, h), ((w - x) / 2, 0, 0)),
        Solid("box", (w, y, h), ((x - w) / 2, 0, 0)),
        Solid("box", (x - 2 * w, w, h), (0, (w - y) / 2, 0)),
        Solid("box", (x - 2 * w, w, h), (0, (y - w) / 2, 0)),
    )


def packing_insert(crate_height):
    """A four-sided slip-in insert that keeps a loaded can near the box center."""
    inner = CRATE_POCKET_OPENING
    wall = 0.006
    side = (inner + wall) / 2
    z = -crate_height / 2 + FLOOR_THICKNESS + CRATE_POCKET_HEIGHT / 2
    gray = (0.53, 0.55, 0.57)
    return (
        Solid("box", (wall, inner + 2 * wall, CRATE_POCKET_HEIGHT), (-side, 0, z), color=gray),
        Solid("box", (wall, inner + 2 * wall, CRATE_POCKET_HEIGHT), (side, 0, z), color=gray),
        Solid("box", (inner, wall, CRATE_POCKET_HEIGHT), (0, -side, z), color=gray),
        Solid("box", (inner, wall, CRATE_POCKET_HEIGHT), (0, side, z), color=gray),
    )


class ConveyorPacking:
    """Scene and trial state shared by the two directions of conveyor transfer."""

    def __init__(self, *, moving_source):
        self.moving_source = moving_source
        return_clearance = 0.015 if moving_source else 0.0
        self.belt = Box("belt", pos=(0.320, -0.145, TOP + 0.015 + return_clearance),
                        size=(0.180, 1.620, 0.030),
                        mass=5.0, color=(0.13, 0.15, 0.17))
        self.deck = TOP + 0.030 + return_clearance
        self.shipping_case = Box("case", pos=(0.102, 0.0, TOP + 0.050), size=(0.240, 0.240, 0.100),
                                 color=CRATE_COLOR)
        arrival_y = (-0.060 if moving_source else 0.040, 0.510, 0.275)
        staging_y = (-0.281, 0.281, -0.414)
        self.cans = {
            f"can_{i}": Box(f"can_{i}", pos=(self.belt.pos[0] if moving_source else 0.139,
                                             arrival_y[i] if moving_source else staging_y[i],
                                             (self.deck if moving_source else TOP) + CAN_HEIGHT / 2),
                            size=(2 * CAN_RADIUS, 2 * CAN_RADIUS, CAN_HEIGHT), mass=0.045, color=CAN_COLOR)
            for i in range(3)
        }
        if moving_source:
            self.crates = {"case": self.shipping_case}
        else:
            self.crates = {
                f"crate_{i}": Box(f"crate_{i}", pos=(self.belt.pos[0], arrival_y[i], self.deck + 0.040),
                                  size=(0.160, 0.160, 0.080), mass=0.090, color=CRATE_COLOR)
                for i in range(3)
            }
        self.transfers = (
            Transfer("right", "can_0", "case" if moving_source else "crate_0", (0.0, -0.074)),
            Transfer("left", "can_1", "case" if moving_source else "crate_1", (0.0, 0.074)),
            Transfer("right", "can_2", "case" if moving_source else "crate_2", (0.042, 0.0)),
        )
        frame, self.parts = conveyor_frame(self.belt, exposed_return=moving_source)
        self.objects = [self.belt, *frame, *self.cans.values(), *self.crates.values()]
        self.parts.update({name: open_crate(crate.size) for name, crate in self.crates.items()})
        self.parts.update({name: (Solid("cylinder", (CAN_RADIUS, CAN_HEIGHT)),) for name in self.cans})
        if moving_source:
            x, y, _ = self.belt.pos
            self.table = Box("table", pos=(x, y, TOP - 0.030), size=(1.10, LONG_TABLE_LENGTH, 0.060))
            end = y - (self.belt.size[1] - self.belt.size[2]) / 2
            land = y - LONG_TABLE_LENGTH / 2
            height = self.deck - DISCHARGE_STEP - TOP
            self.runout = Box("runout_deck", pos=(x, (end + land)/2, TOP + height/2),
                              size=(0.240, end-land, height), color=(0.79, 0.68, 0.51))
            self.objects.append(self.runout)
        else:
            for name, crate in self.crates.items():
                self.parts[name] += packing_insert(crate.size[2])
        self.payloads = tuple(self.cans if moving_source else self.crates)
        self.reset()

    def reset(self):
        self.recent_transport = {}
        self.unsupported = set()
        self.lifted = set()
        self.released = {}
        self.completed = {}
        self.context = {}

    @staticmethod
    def upright(state):
        x, y, _, _ = state.quat_xyzw
        return 1.0 - 2.0 * (x*x + y*y) >= math.cos(math.radians(12))

    def contained(self, can, crate, origin):
        floor = origin[2] - crate.size[2] / 2 + FLOOR_THICKNESS
        half = np.asarray(crate.size[:2]) / 2 - WALL_THICKNESS if self.moving_source else CRATE_POCKET_OPENING / 2
        return (self.upright(can)
                and np.all(can.low[:2] >= origin[:2] - half - 0.002)
                and np.all(can.high[:2] <= origin[:2] + half + 0.002)
                and floor - 0.004 <= can.low[2] <= floor + 0.006)

    def observe(self, states, context, dt):
        self.context = context
        origins = context["body_origins"]
        time = context["time_s"]
        for name in self.cans:
            part = states[name]
            if self.moving_source and name not in self.lifted:
                within_belt = (abs(part.pos[0] - self.belt.pos[0]) < self.belt.size[0] / 2
                               and abs(part.pos[1] - self.belt.pos[1]) < self.belt.size[1] / 2 - CAN_RADIUS)
                if within_belt and part.low[2] < self.deck - 0.006:
                    self.unsupported.add(name)
                if within_belt and abs(part.low[2] - self.deck) < 0.006 and part.lin_vel[1] < -0.033:
                    self.recent_transport[name] = time
            supported_at_pick = (not self.moving_source or
                                 (name not in self.unsupported and time - self.recent_transport.get(name, -10) < 0.85))
            acquisition_height = self.deck if self.moving_source else TOP
            held = any(context["gripper"][side] < 0.88 and np.linalg.norm(tcp - part.pos) < 0.095
                       for side, tcp in context["tcp"].items())
            if supported_at_pick and held and part.low[2] > acquisition_height + 0.020:
                self.lifted.add(name)
            if name not in self.lifted:
                continue
            for crate_name, crate in self.crates.items():
                key = (name, crate_name)
                origin = origins[crate_name]
                contained = self.contained(part, crate, origin)
                carried = not self.moving_source and states[crate_name].lin_vel[1] < -0.035
                if not contained:
                    self.released.pop(key, None)
                    self.completed.pop(key, None)
                    continue
                hand_near = any(math.dist(part.pos, tcp) < 0.11 and context["gripper"][side] > 0.94
                                for side, tcp in context["tcp"].items())
                if not held and hand_near and (self.moving_source or carried):
                    self.released.setdefault(key, (time, float(origin[1])))
                if key not in self.released:
                    continue
                release_time, release_y = self.released[key]
                clear = all(math.dist(part.pos, tcp) > 0.085 for tcp in context["tcp"].values())
                settled = np.linalg.norm(part.lin_vel - states[crate_name].lin_vel) < 0.023
                retained = self.moving_source or origin[1] <= release_y - 0.036
                if clear and settled and retained and time - release_time >= 0.4:
                    self.completed[key] = True

    def count(self, states):
        if not self.context:
            return 0
        origins = self.context["body_origins"]
        valid = [(can_name, crate_name) for can_name, crate_name in self.completed
                 if self.contained(states[can_name], self.crates[crate_name], origins[crate_name])]
        return len({can for can, _ in valid}) if self.moving_source else len({crate for _, crate in valid})

    def success(self, states):
        return self.count(states) == len(self.cans)

    def score(self, states):
        return self.count(states) / len(self.cans)

    def demo(self):
        self.track_points = {}
        plans = {"left": [], "right": []}
        for i, transfer in enumerate(self.transfers):
            plan = plans[transfer.arm]
            travel = TOP + (0.208 if self.moving_source else 0.204)
            if self.moving_source:
                # Share the case serially. The idle wrist stays out of the packing area.
                park = (0.245, 0.305 if transfer.arm == "left" else -0.305, travel)
                delay = (0.0, 5.0, 10.3)[i] - sum(step.duration for step in plan)
                if delay > 0:
                    plan.append(Waypoint(delay, park, FINGER_OPEN, "wait at side"))
            start = sum(step.duration for step in plan)
            can, crate = self.cans[transfer.can], self.crates[transfer.crate]
            pick_xy = np.array(can.pos[:2])
            if self.moving_source:
                pick_xy[1] -= BELT_SPEED * (start + 2.35)
            place_xy = np.array(crate.pos[:2])
            if self.moving_source:
                place_xy += transfer.drop_offset
            else:
                place_xy[1] -= BELT_SPEED * (start + 6.10)
            grasp = CAN_GRASP if self.moving_source else 0.022
            pick_z = can.pos[2] + grasp
            floor = crate.pos[2] - crate.size[2] / 2 + FLOOR_THICKNESS
            release_z = floor + CAN_HEIGHT / 2 + grasp + 0.004
            self.track_points[i] = (start, pick_xy, place_xy)
            for duration, xy, z, finger, phase in (
                (1.2, pick_xy, travel, FINGER_OPEN, "approach"),
                (0.9, pick_xy, pick_z, FINGER_OPEN, "lower_pick"),
                (0.6, pick_xy, pick_z, FINGER_CLOSED, "close"),
                (0.8, pick_xy, travel, FINGER_CLOSED, "lift"),
                (1.35, place_xy, travel, FINGER_CLOSED, "carry"),
                (0.85, place_xy, release_z, FINGER_CLOSED, "lower_place"),
                (0.6, place_xy, release_z, FINGER_OPEN, "release"),
                (0.8, place_xy, travel, FINGER_OPEN, "clear"),
            ):
                yaw = 0.0
                if self.moving_source and phase in {"carry", "lower_place", "release", "clear"}:
                    yaw = (-1 if transfer.arm == "left" else 1) * math.pi / 2
                plan.append(Waypoint(duration, (*xy, z), finger, f"{i}/{phase}", yaw))
            if self.moving_source:
                plan.append(Waypoint(1.0, park, FINGER_OPEN, "return to side"))
        return plans

    def demo_target(self, arm, label, time, position, rotation, finger):
        if "/" not in label or not self.context:
            return position, rotation, finger
        index, phase = label.split("/")
        i = int(index)
        transfer = self.transfers[i]
        start, pick_xy, place_xy = self.track_points[i]
        origins = self.context["body_origins"]
        adjusted = position.copy()
        if self.moving_source and phase in {"approach", "lower_pick", "close"}:
            blend = smoothstep(float(np.clip((time - start) / 1.2, 0.0, 1.0)))
            adjusted[:2] += blend * (origins[transfer.can][:2] - pick_xy)
        if not self.moving_source and phase in {"carry", "lower_place", "release", "clear"}:
            blend = smoothstep(float(np.clip((time - start - 3.5) / 1.35, 0.0, 1.0)))
            adjusted[:2] += blend * (origins[transfer.crate][:2] - place_xy)
            if phase in {"carry", "lower_place"}:
                adjusted[:2] += np.clip(self.context["tcp"][arm][:2] - origins[transfer.can][:2], -0.018, 0.018)
        return adjusted, rotation, finger
