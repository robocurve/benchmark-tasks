"""On the station table, a 1 g bench analogue of troubleshooting ground-support pneumatic panels: some outlets have
lost pressure, and the technician uses the line diagram on the panel face and the single-fault rule to find the
one valve that explains every dead outlet without killing a live one, then hangs a red danger tag on it. Two fixed
white panels, one in front of each arm, carry a blue supply bottle, grey valve blocks, dark blue line strips (the
pipes) and outlet indicators, green for flow and black for none. On the left panel the stuck valve sits mid-tree,
upstream of a junction feeding two dead outlets; on the right panel it is a leaf, its dead outlet sits nearer another
valve, and the valve feeding its junction would also kill a live outlet. One loose red tag lies beside each panel.
The score reads each tag from its centre and quaternion and pays 0.5 per panel whose stuck valve, and no other valve
of that panel, carries a tag.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/stuck-valve-diagnosis/stuck_valve_diagnosis.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Two pneumatic panels each have one grey valve stuck shut and every other valve open. Dark blue lines are pipes from the blue supply. Outlet indicators show green for flow and black for no flow. Set one red tag on top of each panel's stuck valve." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. Arm bases at (0, +-0.305, 0.75).
PANEL = (0.260, 0.240, 0.006)  # the white base board of each panel
VALVE = (0.030, 0.030, 0.018)
OUTLET = (0.030, 0.030, 0.003)  # an outlet indicator, flush on the board
SUPPLY = (0.030, 0.030, 0.024)  # the supply bottle, 6 mm taller than a valve
LINE_W, LINE_H = 0.010, 0.003  # line strips, the pipes
TAG = (0.030, 0.030, 0.024)
TAG_MASS = 0.020
BOARD_TOP = TOP + PANEL[2]
VALVE_TOP = BOARD_TOP + VALVE[2]  # 24 mm above the table

WHITE = (0.92, 0.92, 0.90)
BLUE = (0.15, 0.35, 0.75)
GREY = (0.55, 0.55, 0.57)
BLACK = (0.10, 0.10, 0.10)
GREEN = (0.15, 0.75, 0.25)
PIPE_BLUE = (0.10, 0.20, 0.55)  # the line strips; black is kept for no flow only
RED = (0.85, 0.10, 0.10)

# Each panel: board centre, supply, valves, outlets (centre, flowing?), the pipe graph (node pairs; j1 and j2 are
# junctions, s the supply) and the line strips that draw it, each a centre line between two block faces with the
# graph edges it carries. Every line runs along x or y; branches butt against the side of the line they leave. Both
# panels sit on a 50 mm grid of block centres (x 0.150 to 0.350, four rows in y), so blocks stand 20 mm apart.
PANELS = {
    "left": dict(
        centre=(0.250, 0.305),
        supply=(0.150, 0.380),
        valves=dict(v1=(0.200, 0.380), v2=(0.300, 0.380), v3=(0.250, 0.330), v4=(0.300, 0.280), v5=(0.250, 0.230)),
        outlets=dict(o1=((0.350, 0.380), True), o2=((0.350, 0.280), False), o3=((0.200, 0.230), False)),
        edges=(("s", "v1"), ("v1", "j1"), ("j1", "v2"), ("v2", "o1"), ("j1", "v3"), ("v3", "j2"), ("j2", "v4"), ("v4", "o2"), ("j2", "v5"), ("v5", "o3")),
        lines=(
            ((0.165, 0.380), (0.185, 0.380), (("s", "v1"),)),
            ((0.215, 0.380), (0.285, 0.380), (("v1", "j1"), ("j1", "v2"))),  # j1 at x = 0.250
            ((0.315, 0.380), (0.335, 0.380), (("v2", "o1"),)),
            ((0.250, 0.375), (0.250, 0.345), (("j1", "v3"),)),
            ((0.250, 0.315), (0.250, 0.245), (("v3", "j2"), ("j2", "v5"))),  # j2 at y = 0.280
            ((0.255, 0.280), (0.285, 0.280), (("j2", "v4"),)),
            ((0.315, 0.280), (0.335, 0.280), (("v4", "o2"),)),
            ((0.235, 0.230), (0.215, 0.230), (("v5", "o3"),)),
        ),
        stuck="v3",
    ),
    "right": dict(  # the dead o1 sits 71 mm from v4 (which feeds the green o3) and 100 mm from v3, its own valve
        centre=(0.250, -0.305),
        supply=(0.150, -0.230),
        valves=dict(v1=(0.200, -0.230), v2=(0.350, -0.230), v3=(0.300, -0.280), v4=(0.250, -0.330)),
        outlets=dict(o1=((0.300, -0.380), False), o2=((0.350, -0.330), True), o3=((0.250, -0.380), True)),
        edges=(("s", "v1"), ("v1", "j1"), ("j1", "v4"), ("v4", "o3"), ("j1", "v2"), ("v2", "j2"), ("j2", "v3"), ("v3", "o1"), ("j2", "o2")),
        lines=(
            ((0.165, -0.230), (0.185, -0.230), (("s", "v1"),)),
            ((0.215, -0.230), (0.335, -0.230), (("v1", "j1"), ("j1", "v2"))),  # j1 at x = 0.250
            ((0.250, -0.235), (0.250, -0.315), (("j1", "v4"),)),
            ((0.250, -0.345), (0.250, -0.365), (("v4", "o3"),)),
            ((0.350, -0.245), (0.350, -0.315), (("v2", "j2"), ("j2", "o2"))),  # j2 at y = -0.280, no valve to o2
            ((0.345, -0.280), (0.315, -0.280), (("j2", "v3"),)),
            ((0.300, -0.295), (0.300, -0.365), (("v3", "o1"),)),
        ),
        stuck="v3",
    ),
}
TAG_STARTS = {"left_tag": (0.150, 0.110), "right_tag": (0.150, -0.110)}  # inboard, where no arm at READY hides them


def _line_box(name, p0, p1):
    (x0, y0), (x1, y1) = p0, p1
    size = (abs(x1 - x0), LINE_W, LINE_H) if y0 == y1 else (LINE_W, abs(y1 - y0), LINE_H)
    return Box(name, pos=((x0 + x1) / 2, (y0 + y1) / 2, BOARD_TOP + LINE_H / 2), size=size, color=PIPE_BLUE)


def _panel_fixtures(side, p):
    cx, cy = p["centre"]
    out = [
        Box(f"{side}_panel_board", pos=(cx, cy, TOP + PANEL[2] / 2), size=PANEL, color=WHITE),
        Box(f"{side}_panel_supply", pos=(*p["supply"], BOARD_TOP + SUPPLY[2] / 2), size=SUPPLY, color=BLUE),
    ]
    out += [Box(f"{side}_valve_{k[1:]}", pos=(x, y, BOARD_TOP + VALVE[2] / 2), size=VALVE, color=GREY) for k, (x, y) in p["valves"].items()]
    out += [Box(f"{side}_outlet_{k[1:]}", pos=(x, y, BOARD_TOP + OUTLET[2] / 2), size=OUTLET, color=GREEN if live else BLACK) for k, ((x, y), live) in p["outlets"].items()]
    out += [_line_box(f"{side}_line_{i + 1}", p0, p1) for i, (p0, p1, _) in enumerate(p["lines"])]
    return out


TAGS = [Box(name, pos=(x, y, TOP + TAG[2] / 2), size=TAG, mass=TAG_MASS, color=RED) for name, (x, y) in TAG_STARTS.items()]
FIXTURES = [f for side, p in PANELS.items() for f in _panel_fixtures(side, p)]
OBJECTS = [*TAGS, *FIXTURES]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

SEAT_TOL = 0.004  # the tag's lowest point to the valve top
CENTRE_TOL = 0.012  # the tag's centre to the valve's centre, seen from above
TILT_TOL = math.radians(10.0)  # some face of the tag within 10 degrees of flat
EPS = 1e-6


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _on_valve(state, valve_xy) -> bool:
    """Resting flat on any face on the valve's top: lowest point (from the centre and quaternion) within 4 mm of
    the top, centre within 12 mm of the valve's centre, some body axis within 10 degrees of vertical, at rest."""
    if not state.resting():
        return False
    up = np.abs(_rot(state.quat_xyzw)[2, :])  # each body axis against world up
    if float(up.max()) < math.cos(TILT_TOL):
        return False
    lowest = float(state.pos[2]) - float(up @ (np.asarray(TAG) / 2))
    if abs(lowest - VALVE_TOP) > SEAT_TOL + EPS:
        return False
    return math.hypot(float(state.pos[0]) - valve_xy[0], float(state.pos[1]) - valve_xy[1]) <= CENTRE_TOL + EPS


def _panel_tagged(objects, panel) -> bool:
    """A tag on the panel's stuck valve and no tag on any other valve of that panel. Either tag may serve."""
    tags = [objects[t.name] for t in TAGS]
    on = {k: any(_on_valve(s, xy) for s in tags) for k, xy in panel["valves"].items()}
    return on[panel["stuck"]] and not any(v for k, v in on.items() if k != panel["stuck"])


def success(objects) -> bool:
    """Both panels' stuck valves tagged, no wrong valve tagged, both tags at rest; holds for 1 s."""
    return all(_panel_tagged(objects, p) for p in PANELS.values())


def distance(objects) -> float:
    """Metres still to go: for each panel, the nearest tag's centre to a tag seated on its stuck valve."""
    total = 0.0
    for p in PANELS.values():
        gx, gy = p["valves"][p["stuck"]]
        goal = np.array([gx, gy, VALVE_TOP + TAG[2] / 2])
        total += min(float(np.linalg.norm(np.asarray(objects[t.name].pos, float) - goal)) for t in TAGS)
    return total


def score(objects) -> float:
    """0.5 per panel whose stuck valve carries a tag while no other valve of that panel does: 0, 0.5 or 1.0."""
    return 0.5 * sum(_panel_tagged(objects, p) for p in PANELS.values())


TRAVEL = TOP + 0.145
GRIP_Z = TOP + 0.019  # the tag on the table; pad tips 6 mm above it, 18 mm of pad on the 24 mm tag
PLACE_Z = VALVE_TOP + 0.006 + 0.019  # tag bottom 6 mm above the valve top, clear of the 4 mm seat band while held
ENTRY = 0.010  # the pause above the valve: tag bottom 16 mm above every valve top
YAW = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along x at the pick and the place; mirrored, inside joint 6
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part


def _pick_place(pick, place, *, width, entry, travel_z, yaw=0.0, place_yaw=None):
    """A top-down pick and place paced for the simulated arm, whose soft wrist trails a moving target by several
    millimetres. The arm settles before every close and release, stops ``entry`` above the place with the part
    clear of every fixture, and makes the last descent slowly. The fingers close to 8 mm narrower than the part's
    ``width`` across the pads rather than fully shut: each finger then pushes 4 mm x 5000 N/m = 20 N, under its
    50 N cap, so the grip centres the part, whereas a fully commanded close saturates both fingers and lets the part
    slide onto one of them. The gripper opens and closes over 1.5 s, inside the default 0.05 per step limiter.
    16.2 s."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    grip = -(width - SQUEEZE) / 2  # finger joint target, m
    return [
        Waypoint(1.5, (px, py, travel_z), FINGER_OPEN, "to the object", yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", yaw),
        Waypoint(0.6, (px, py, pz), FINGER_OPEN, "settle", yaw),
        Waypoint(1.5, (px, py, pz), grip, "close", yaw),
        Waypoint(1.2, (px, py, travel_z), grip, "lift", yaw),
        Waypoint(2.0, (qx, qy, travel_z), grip, "carry", end),
        Waypoint(0.6, (qx, qy, travel_z), grip, "settle over the place", end),
        Waypoint(1.2, (qx, qy, qz + entry), grip, "lower to the entry", end),
        Waypoint(0.8, (qx, qy, qz + entry), grip, "settle at the entry", end),
        Waypoint(2.0, (qx, qy, qz), grip, "set in", end),
        Waypoint(0.8, (qx, qy, qz), grip, "settle", end),
        Waypoint(1.5, (qx, qy, qz), FINGER_OPEN, "release", end),
        Waypoint(1.0, (qx, qy, travel_z), FINGER_OPEN, "retreat", end),
    ]


def demo():
    """Both arms at once, each in front of its own base: one top-down pick and place of its panel's tag onto that
    panel's stuck valve, fingers closing along x across the tag, a carry at 145 mm and a release 6 mm above the
    valve top. 16.2 s of plan, then the 3 s hold: 384 steps at 20 Hz."""
    plan = {}
    for side, tag in (("left", "left_tag"), ("right", "right_tag")):
        sx, sy = TAG_STARTS[tag]
        vx, vy = PANELS[side]["valves"][PANELS[side]["stuck"]]
        plan[side] = _pick_place((sx, sy, GRIP_Z), (vx, vy, PLACE_Z), width=TAG[0], entry=ENTRY, travel_z=TRAVEL, yaw=YAW[side])
    return plan


def _dead(panel, closed):
    """Outlets that no flow reaches from the supply when valve ``closed`` is shut and every other valve is open."""
    nbrs = {}
    for a, b in panel["edges"]:
        nbrs.setdefault(a, set()).add(b)
        nbrs.setdefault(b, set()).add(a)
    seen, todo = {"s"}, ["s"]
    while todo:
        for n in nbrs[todo.pop()]:
            if n not in seen and n != closed:
                seen.add(n)
                todo.append(n)
    return {o for o in panel["outlets"] if o not in seen}


def _rect(box):
    """A fixture's footprint seen from above: (x min, x max, y min, y max)."""
    (x, y, _), (sx, sy, _) = box.pos, box.size
    return (x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2)


def _contact(a, b):
    """How two footprints meet: "overlap", "touch" (zero gap) or the gap in metres."""
    dx, dy = max(a[0] - b[1], b[0] - a[1]), max(a[2] - b[3], b[2] - a[3])
    if dx < -EPS and dy < -EPS:
        return "overlap"
    if max(dx, dy) <= EPS:
        return "touch"
    return math.hypot(max(dx, 0.0), max(dy, 0.0))


def _check_drawing(side, p):
    """The strips as built draw exactly the pipe graph: each strip touches, with zero gap, exactly the blocks its
    edges name and, at a junction, the other strips that carry that junction; nothing overlaps; blocks stand at
    least 20 mm apart and at least 15 mm inside the board edge."""
    fx = {f.name: f for f in FIXTURES}
    blocks = {"s": fx[f"{side}_panel_supply"]}
    blocks.update({k: fx[f"{side}_valve_{k[1:]}"] for k in p["valves"]})
    blocks.update({k: fx[f"{side}_outlet_{k[1:]}"] for k in p["outlets"]})
    parts = dict(blocks)
    parts.update({i: fx[f"{side}_line_{i + 1}"] for i in range(len(p["lines"]))})
    for i, (_, _, edges) in enumerate(p["lines"]):
        want = set()
        for n in {n for e in edges for n in e}:
            if n.startswith("j"):
                want |= {k for k, (_, _, es) in enumerate(p["lines"]) if k != i and any(n in e for e in es)}
            else:
                want.add(n)
        meet = {k: _contact(_rect(parts[i]), _rect(b)) for k, b in parts.items() if k != i}
        assert not [k for k, c in meet.items() if c == "overlap"], (side, i)
        assert {k for k, c in meet.items() if c == "touch"} == want, (side, i, want)
    names = list(blocks)
    for a in range(len(names)):
        for b in range(a + 1, len(names)):
            c = _contact(_rect(blocks[names[a]]), _rect(blocks[names[b]]))
            assert not isinstance(c, str) and c >= 0.020 - EPS, (side, names[a], names[b])
    bx = _rect(fx[f"{side}_panel_board"])
    for k, b in blocks.items():
        r = _rect(b)
        assert min(r[0] - bx[0], bx[1] - r[1], r[2] - bx[2], bx[3] - r[3]) >= 0.015 - EPS, (side, k)


def _shortcut_valves(side, p):
    """The valve each spatial shortcut picks on a panel (a tie gives several)."""
    d = lambda a, b: math.hypot(a[0] - b[0], a[1] - b[1])
    dead = [xy for xy, live in p["outlets"].values() if not live]

    def pick(key, far=False):
        k = {v: key(xy) for v, xy in p["valves"].items()}
        top = max(k.values()) if far else min(k.values())
        return {v for v, x in k.items() if abs(x - top) < EPS}

    return {
        "main valve": pick(lambda xy: d(xy, p["supply"])),
        "nearest a dead outlet": pick(lambda xy: min(d(xy, o) for o in dead)),
        "nearest the tag": pick(lambda xy: d(xy, TAG_STARTS[f"{side}_tag"])),
        "farthest from the supply": pick(lambda xy: d(xy, p["supply"]), far=True),
    }


def self_test(ObjectState=None):
    """The drawing and the diagnosis on each panel's pipe graph, then the scoring rules on hand-built states; runs
    with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    for side, p in PANELS.items():  # the drawing carries exactly the graph, and only the stuck valve explains it
        drawn = sorted(tuple(sorted(e)) for _, _, es in p["lines"] for e in es)
        assert drawn == sorted(tuple(sorted(e)) for e in p["edges"]), side
        _check_drawing(side, p)
        dead = {o for o, (_, live) in p["outlets"].items() if not live}
        assert _dead(p, None) == set(), side
        assert [v for v in p["valves"] if _dead(p, v) == dead] == [p["stuck"]], side
        for rule, picks in _shortcut_valves(side, p).items():  # each of these shortcuts misses on both panels
            assert p["stuck"] not in picks, (side, rule, picks)
    for a, b in (("left", "right"), ("right", "left")):  # copying one panel's answer to the other, mirrored or shifted
        x, y = PANELS[a]["valves"]["v3"]
        gx, gy = PANELS[b]["valves"]["v3"]
        for sx, sy in ((x, -y), (x, y - math.copysign(0.610, y))):
            assert math.hypot(sx - gx, sy - gy) > CENTRE_TOL, (a, b)

    def quat(yaw=0.0, tilt=0.0):  # yaw about the vertical, then tilt about the tag's x axis
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        return np.array([cy * st, sy * st, sy * ct, cy * ct])

    def tag(x, y, bottom=TOP, yaw=0.0, tilt=0.0, vel=(0.0, 0.0, 0.0)):
        q = quat(yaw, tilt)
        half = np.abs(_rot(q)) @ (np.asarray(TAG) / 2)
        pos = np.array([x, y, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def on(side, valve, dx=0.0, dy=0.0, bottom=VALVE_TOP, **kw):
        x, y = PANELS[side]["valves"][valve]
        return tag(x + dx, y + dy, bottom=bottom, **kw)

    start = {name: tag(x, y) for name, (x, y) in TAG_STARTS.items()}

    def scene(left=None, right=None):
        return {"left_tag": left or start["left_tag"], "right_tag": right or start["right_tag"]}

    L3, R3 = on("left", "v3"), on("right", "v3")
    cases = {
        "start": (scene(), 0.0, False),
        "done": (scene(L3, R3), 1.0, True),
        "tags swapped between the panels": (scene(R3, L3), 1.0, True),
        "left only": (scene(L3), 0.5, False),
        "right only": (scene(None, R3), 0.5, False),
        "left main valve v1": (scene(on("left", "v1"), R3), 0.5, False),
        "left v2, which feeds the live outlet": (scene(on("left", "v2"), R3), 0.5, False),
        "left v4, one dead outlet only": (scene(on("left", "v4"), R3), 0.5, False),
        "left v5, one dead outlet only": (scene(on("left", "v5"), R3), 0.5, False),
        "right main valve v1": (scene(L3, on("right", "v1")), 0.5, False),
        "right v2, which also feeds the live o2": (scene(L3, on("right", "v2")), 0.5, False),
        "right v4, the valve nearest the black o1": (scene(L3, on("right", "v4")), 0.5, False),
        "both tags on the left panel, v3 and v4": (scene(L3, on("left", "v4")), 0.0, False),
        "a second tag stacked on the left v3 tag": (scene(L3, on("left", "v3", bottom=VALVE_TOP + TAG[2])), 0.5, False),
        "11.9 mm off the valve centre still counts": (scene(on("left", "v3", dx=0.0119), R3), 1.0, True),
        "an exact 12 mm off counts": (scene(on("left", "v3", dy=-0.012), R3), 1.0, True),
        "13 mm off does not": (scene(on("left", "v3", dx=0.013), R3), 0.5, False),
        "8 mm off along both axes still counts": (scene(L3, on("right", "v3", dx=0.008, dy=-0.008)), 1.0, True),
        "tilted 8 degrees counts": (scene(on("left", "v3", tilt=math.radians(8)), R3), 1.0, True),
        "tilted 12 degrees does not": (scene(on("left", "v3", tilt=math.radians(12)), R3), 0.5, False),
        "lying on its side counts": (scene(on("left", "v3", tilt=math.pi / 2), R3), 1.0, True),
        "upside down and yawed 45 degrees counts": (scene(on("left", "v3", tilt=math.pi, yaw=math.pi / 4), R3), 1.0, True),
        "held still 6 mm above the valve top": (scene(on("left", "v3", bottom=VALVE_TOP + 0.006), R3), 0.5, False),
        "exactly 4 mm above counts": (scene(on("left", "v3", bottom=VALVE_TOP + 0.004), R3), 1.0, True),
        "on the valve but still moving": (scene(on("left", "v3", vel=(0.0, 0.0, 0.02)), R3), 0.5, False),
        "on the bare board beside the stuck valve": (scene(tag(0.195, 0.315, bottom=BOARD_TOP), R3), 0.5, False),
        "on the left dead outlet o3": (scene(tag(0.200, 0.230, bottom=BOARD_TOP + OUTLET[2]), R3), 0.5, False),
        "on the left supply bottle": (scene(tag(0.150, 0.380, bottom=BOARD_TOP + SUPPLY[2]), R3), 0.5, False),
        "on the right live outlet o2": (scene(L3, tag(0.350, -0.330, bottom=BOARD_TOP + OUTLET[2])), 0.5, False),
        "on the right dead outlet o1": (scene(L3, tag(0.300, -0.380, bottom=BOARD_TOP + OUTLET[2])), 0.5, False),
        "on the right pipe from v3 to o1": (scene(L3, tag(0.300, -0.330, bottom=BOARD_TOP + LINE_H)), 0.5, False),
        "on the table in front of the right panel": (scene(L3, tag(0.420, -0.280)), 0.5, False),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert distance(scene(L3, R3)) < 1e-9 and distance(scene()) > 0.4
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the diagnosis and the scoring rules on hand-built states, then both arms' plans
    # replayed through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for arm, waypoints in task.demo().items():
        plan = ArmPlan(list(waypoints), station.BASES[arm])
        q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (arm, plan.worst_ik_mm, plan.worst_ik_deg)
        assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), arm
        print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
