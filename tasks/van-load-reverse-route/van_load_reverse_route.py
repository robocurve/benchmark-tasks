"""On the station table: a small open-top delivery-van cargo bay stands in front of the arms, its open door end
facing them and its three walls 20 mm high. Beside it, a white route card lists three delivery stops top to bottom
as colored squares: stop 1 red, stop 2 green, stop 3 blue. Three parcels (45 mm cubes) colored to match stand on
the table nearer the arms, from left to right: green, blue, red.

Goal: all three parcels inside the cargo bay in a row from the back wall to the door, in reverse route order (blue
for stop 3 deepest, then green, then red for stop 1 nearest the door), so each one can be unloaded at its stop
without moving the others.

Run it:  inspect-robots run --embodiment yam_arena -E task=van_load_reverse_route.py -E object_jitter=0 \\
           --instruction "Load the parcels into the van so they can be unloaded in route order: the parcel for the \\
           last stop on the route card goes in deepest and the one for the first stop goes nearest the door." ...
"""

import math

from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

WALL_T, WALL_H = 0.006, 0.020
BAY_X0, BAY_X1 = 0.27, 0.44  # inner floor from the door (x0, open) to the back wall (x1)
BAY_Y, BAY_W = 0.13, 0.075  # centre and inner width side to side
GREY = (0.82, 0.83, 0.85)
COLORS = {"red": (0.80, 0.15, 0.12), "green": (0.15, 0.60, 0.25), "blue": (0.15, 0.35, 0.80)}
ROUTE = ("red", "green", "blue")  # stop 1, 2, 3
CUBE = 0.045
PARCEL_START = {"green": (0.20, 0.17), "blue": (0.20, 0.10), "red": (0.20, 0.03)}
CARD_X, CARD_Y = 0.40, -0.03

_bay_cx, _bay_len = (BAY_X0 + BAY_X1) / 2, BAY_X1 - BAY_X0
OBJECTS = [
    Box("bay_back", pos=(BAY_X1 + WALL_T / 2, BAY_Y, TOP + WALL_H / 2), size=(WALL_T, BAY_W + 2 * WALL_T, WALL_H), color=GREY),
    Box("bay_left", pos=(_bay_cx, BAY_Y + BAY_W / 2 + WALL_T / 2, TOP + WALL_H / 2), size=(_bay_len, WALL_T, WALL_H), color=GREY),
    Box("bay_right", pos=(_bay_cx, BAY_Y - BAY_W / 2 - WALL_T / 2, TOP + WALL_H / 2), size=(_bay_len, WALL_T, WALL_H), color=GREY),
    Box("route_card", pos=(CARD_X, CARD_Y, TOP + 0.001), size=(0.09, 0.03, 0.002), color=(0.95, 0.95, 0.95)),
    *[Box(f"route_stop{i + 1}", pos=(CARD_X + 0.025 - 0.025 * i, CARD_Y, TOP + 0.0025), size=(0.014, 0.014, 0.001), color=COLORS[c])
      for i, c in enumerate(ROUTE)],
    *[Box(f"{c}_parcel", pos=(*PARCEL_START[c], TOP + CUBE / 2), size=(CUBE, CUBE, CUBE), mass=0.08, color=COLORS[c])
      for c in ("green", "blue", "red")],
]


def _in_bay(objects, c):
    """Upright on the bay floor, its centre inside the bay (between the door line and the back wall)."""
    p = objects[f"{c}_parcel"].pos
    return (abs(p[2] - (TOP + CUBE / 2)) < 0.004 and BAY_X0 < p[0] < BAY_X1
            and abs(p[1] - BAY_Y) < BAY_W / 2)


def score(objects):
    """0.2 per parcel standing on the bay floor; +0.4 once all three are in and ordered blue (back), green, red (door)
    along the bay, each at least 20 mm deeper than the next."""
    inside = {c: _in_bay(objects, c) for c in ROUTE}
    s = 0.2 * sum(inside.values())
    if all(inside.values()):
        x = {c: objects[f"{c}_parcel"].pos[0] for c in ROUTE}
        if x["blue"] > x["green"] + 0.02 and x["green"] > x["red"] + 0.02:
            s += 0.4
    return round(s, 3)


def success(objects):
    return score(objects) >= 0.999 and all(objects[f"{c}_parcel"].resting() for c in ROUTE)


def distance(objects):
    goal = {"blue": BAY_X1 - 0.03, "green": (BAY_X0 + BAY_X1) / 2, "red": BAY_X0 + 0.03}
    return float(sum(math.hypot(objects[f"{c}_parcel"].pos[0] - gx, objects[f"{c}_parcel"].pos[1] - BAY_Y)
                     for c, gx in goal.items()))


# -- scripted demo --------------------------------------------------------------------------------------------
TRAVEL_Z = TOP + 0.08  # higher or deeper, the wrist (joint4) nears its limit
YAW = 0.0  # fingers close along world y: across the parcel, toward the bay's side walls, away from other parcels
GRIP_ABOVE_BOTTOM = PAD_TIP_PAST_TCP + 0.010
RELEASE_GAP = 0.004  # underside this far above the wall tops when let go
SETTLE_S = 0.8
SLOT_X = {"blue": BAY_X1 - 0.032, "green": (BAY_X0 + BAY_X1) / 2, "red": BAY_X0 + 0.032}
OPEN_W = -(CUBE / 2 + 0.012)


def _load(c):
    sx, sy = PARCEL_START[c]
    gx = SLOT_X[c]
    pick_z = TOP + GRIP_ABOVE_BOTTOM
    drop_z = TOP + WALL_H + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    return [
        Waypoint(1.5, (sx, sy, TRAVEL_Z), OPEN_W, f"to the {c} parcel", YAW),
        Waypoint(1.2, (sx, sy, pick_z), OPEN_W, "descend", YAW),
        Waypoint(0.8, (sx, sy, pick_z), 0.0, "grip", YAW),
        Waypoint(1.0, (sx, sy, TRAVEL_Z), 0.0, "lift", YAW),
        Waypoint(1.8, (gx, BAY_Y, TRAVEL_Z), 0.0, "over its slot", YAW),
        Waypoint(1.0, (gx, BAY_Y, drop_z + 0.012), 0.0, "lower", YAW),
        Waypoint(SETTLE_S, (gx, BAY_Y, drop_z + 0.012), 0.0, "settle", YAW),
        Waypoint(0.8, (gx, BAY_Y, drop_z), 0.0, "to the walls", YAW),
        Waypoint(SETTLE_S, (gx, BAY_Y, drop_z), 0.0, "settle", YAW),
        Waypoint(0.6, (gx, BAY_Y, drop_z), OPEN_W, "let go", YAW),
        Waypoint(1.0, (gx, BAY_Y, TRAVEL_Z), OPEN_W, "up", YAW),
    ]


def demo():
    """Left arm, deepest slot first: blue against the back wall, then green in the middle, then red by the door. Each
    parcel is let go just above the wall tops and drops upright onto the bay floor."""
    left = [w for c in ("blue", "green", "red") for w in _load(c)]
    left.append(Waypoint(1.5, (0.20, 0.25, TRAVEL_Z), FINGER_OPEN, "clear", YAW))
    return {"left": left}
