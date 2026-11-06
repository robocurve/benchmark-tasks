"""On the station table, in the left arm's lane, a 1 g bench analogue of static balancing before a spin test: a
gold instrument is glued off-centre on a white payload deck, and two grey ballast blocks of one density, a 30 mm
cube and a 60 mm bar of twice its mass, must go onto the deck so that the combined centre of mass of the
instrument and both blocks sits over the black centre mark. The masses are stated in the instruction; the
instrument's offset must be read from the scene. Many placements balance; putting both blocks at the mark, or
mirroring the instrument with them, does not, because the ballast is lighter than the instrument. The score is the
horizontal distance from the combined centre of mass to the mark, computed from each block that rests flat on the
deck, from the block centres and quaternions.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/payload-deck-balance/payload_deck_balance.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \\
           --instruction "Place both grey ballast blocks flat on the white deck so that the combined centre of mass of the gold instrument and the two blocks lies over the deck's centre mark, within 5 mm horizontally. The instrument is 300 g, the large block 120 g and the small block 60 g." \\
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Frame: x forward from the arm bases, y left, z up, metres. The left arm's base is at (0, +0.305, 0.75).
DECK_C = (0.250, 0.260)  # the deck's centre and its centre mark
DECK = (0.260, 0.260, 0.012)
DECK_TOP = TOP + DECK[2]
MARK_BAR = (0.040, 0.010, 0.001)  # two 10 mm wide bars make the cross; their tops stand 0.5 mm proud of the deck
INSTR_OFF = (0.048, 0.036)  # the instrument's centre from the mark: 60 mm, toward forward and outboard
INSTR = (0.070, 0.050, 0.045)
INSTR_MASS = 0.300  # stated; the instrument is glued, so the simulator holds it fixed
LARGE = (0.060, 0.030, 0.030)  # long axis along x at the start
SMALL = (0.030, 0.030, 0.030)
LARGE_MASS = 0.120  # one density: the bar has twice the cube's volume and twice its mass
SMALL_MASS = 0.060
LARGE_START = (0.180, 0.470)
SMALL_START = (0.280, 0.470)

WHITE = (0.92, 0.92, 0.90)
BLACK = (0.05, 0.05, 0.05)
GOLD = (0.85, 0.65, 0.15)
STEEL = (0.40, 0.42, 0.45)

DECK_PLATE = Box("deck", pos=(DECK_C[0], DECK_C[1], TOP + DECK[2] / 2), size=DECK, color=WHITE)
MARK = [
    Box("deck_mark_x", pos=(DECK_C[0], DECK_C[1], DECK_TOP), size=MARK_BAR, color=BLACK),
    Box("deck_mark_y", pos=(DECK_C[0], DECK_C[1], DECK_TOP), size=MARK_BAR, color=BLACK, yaw=math.pi / 2),
]
INSTRUMENT = Box("instrument", pos=(DECK_C[0] + INSTR_OFF[0], DECK_C[1] + INSTR_OFF[1], DECK_TOP + INSTR[2] / 2), size=INSTR, color=GOLD)
BALLAST_LARGE = Box("ballast_large", pos=(*LARGE_START, TOP + LARGE[2] / 2), size=LARGE, mass=LARGE_MASS, color=STEEL)
BALLAST_SMALL = Box("ballast_small", pos=(*SMALL_START, TOP + SMALL[2] / 2), size=SMALL, mass=SMALL_MASS, color=STEEL)
BALLAST = ((BALLAST_LARGE.name, LARGE, LARGE_MASS), (BALLAST_SMALL.name, SMALL, SMALL_MASS))
OBJECTS = [BALLAST_LARGE, BALLAST_SMALL, DECK_PLATE, *MARK, INSTRUMENT]

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

FLAT_TOL = math.radians(5.0)  # one face down: some body axis within 5 degrees of vertical
BOTTOM_TOL = 0.004  # the block's lowest point to the deck top
BALANCED = 0.005  # combined centre of mass to the mark: full score; 20 mm of bar error or 40 mm of cube error
NEAR = 0.010  # half
ROUGH = 0.0125  # quarter; mirroring the instrument at 60 mm (15 mm) earns nothing
ONE_BLOCK_CAP = 0.25  # the most a deck with only one block can earn
EPS = 1e-4  # 0.1 mm, so a value exactly on a limit passes


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _on_deck(state, size) -> bool:
    """At rest, flat on one of its faces (any face, any yaw), its lowest point within 4 mm of the deck top, and its
    centre over the deck. The bottom is computed from the centre and the quaternion; a block on the table, on the
    instrument, on the other block, tilted against something or still held above the deck fails."""
    if not state.resting():
        return False
    rot = _rot(state.quat_xyzw)
    if max(abs(float(rot[2, i])) for i in range(3)) < math.cos(FLAT_TOL):
        return False
    bottom = float(state.pos[2]) - sum(abs(float(rot[2, i])) * size[i] / 2 for i in range(3))
    if abs(bottom - DECK_TOP) > BOTTOM_TOL:
        return False
    dx, dy = float(state.pos[0]) - DECK_C[0], float(state.pos[1]) - DECK_C[1]
    return abs(dx) <= DECK[0] / 2 + EPS and abs(dy) <= DECK[1] / 2 + EPS


def _com_error(objects, names) -> float:
    """Metres from the mark to the horizontal centre of mass of the instrument and the named blocks."""
    mass = {name: m for name, _, m in BALLAST}
    total = INSTR_MASS + sum(mass[n] for n in names)
    mx = INSTR_MASS * INSTR_OFF[0] + sum(mass[n] * (float(objects[n].pos[0]) - DECK_C[0]) for n in names)
    my = INSTR_MASS * INSTR_OFF[1] + sum(mass[n] * (float(objects[n].pos[1]) - DECK_C[1]) for n in names)
    return math.hypot(mx / total, my / total)


def _level(objects) -> float:
    """1.0, 0.5, 0.25 or 0 from the combined centre of mass of the instrument and the blocks on the deck; a block
    not on the deck counts as missing, and a deck with one block earns at most 0.25."""
    on = [name for name, size, _ in BALLAST if _on_deck(objects[name], size)]
    if not on:
        return 0.0
    err = _com_error(objects, on)
    if err <= BALANCED + EPS:
        level = 1.0
    elif err <= NEAR + EPS:
        level = 0.5
    elif err <= ROUGH + EPS:
        level = 0.25
    else:
        level = 0.0
    return level if len(on) == len(BALLAST) else min(level, ONE_BLOCK_CAP)


def success(objects) -> bool:
    """Both blocks flat on the deck and at rest, the combined centre of mass within 5 mm of the mark; holds 1 s."""
    return _level(objects) == 1.0


def distance(objects) -> float:
    """Metres from the mark to the combined centre of mass of the instrument and both blocks, wherever they are."""
    return _com_error(objects, [name for name, _, _ in BALLAST])


def score(objects) -> float:
    """From the blocks that rest flat on the deck (any face down, tilt under 5 degrees, lowest point within 4 mm of
    the deck top, centre over the deck, at rest) and the instrument, the horizontal centre of mass to the mark:
    1.0 within 5 mm, 0.5 within 10 mm, 0.25 within 12.5 mm, else 0. With one block on the deck the score is
    capped at 0.25; with none it is 0."""
    return _level(objects)


TRAVEL = TOP + 0.140
GRIP_Z = TOP + 0.022  # pad tips 9 mm up the 30 mm blocks
PLACE_Z = DECK_TOP + 0.005 + 0.022  # block bottom 5 mm above the deck; pad tips 14 mm above it
ENTRY = 0.010  # the pause above the place: block bottom 15 mm above the deck
SQUEEZE = 0.008  # each close is commanded 8 mm narrower than the part
LARGE_PLACE = (DECK_C[0] - 0.080, DECK_C[1] - 0.040)  # 89 mm from the mark, behind and inboard
SMALL_PLACE = (DECK_C[0] - 0.080, DECK_C[1] - 0.100)  # 128 mm from the mark; 120 x + 60 x = -300 x for both axes
SMALL_YAW = math.radians(-75.0)  # the cube, taken square at yaw 0, turns in the carry so the open fingers clear the bar; -90 runs joint 6 into its limit


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
    """Left arm only. The bar first, taken across its 30 mm width at yaw 0 and set down along x at 89 mm from the
    mark; then the cube, taken square at yaw 0, turned -75 degrees in the carry so the open fingers point nearly
    along x, clear of the bar, and set down 60 mm inboard of the bar. Their moments, 120 g x (-80, -40) mm
    plus 60 g x (-80, -100) mm, cancel the instrument's 300 g x (48, 36) mm exactly. Each move carries at 14 cm and
    releases 5 mm above the deck. 32.4 s of plan, then the 3 s hold: 708 steps at 20 Hz."""
    left = _pick_place((*LARGE_START, GRIP_Z), (*LARGE_PLACE, PLACE_Z), width=LARGE[1], entry=ENTRY, travel_z=TRAVEL, yaw=0.0)
    left += _pick_place((*SMALL_START, GRIP_Z), (*SMALL_PLACE, PLACE_Z), width=SMALL[0], entry=ENTRY, travel_z=TRAVEL, yaw=0.0, place_yaw=SMALL_YAW)
    return {"left": left}


def self_test(ObjectState=None):
    """The scoring rules on hand-built states; runs with the real ObjectState or any stand-in with the same fields."""
    if ObjectState is None:
        from yam_arena.task_file import ObjectState

    def quat(yaw=0.0, tilt=0.0, end_up=False):  # yaw about the vertical, then tilt about the body x axis
        cy, sy, ct, st = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(tilt / 2), math.sin(tilt / 2)
        q = np.array([cy * st, sy * st, sy * ct, cy * ct])
        if end_up:  # stood on an end: then 90 degrees about the body y axis, so the long x axis is vertical
            s = math.sqrt(0.5)
            x1, y1, z1, w1 = q
            x2, y2, z2, w2 = 0.0, s, 0.0, s
            q = np.array(
                [
                    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                ]
            )
        return q

    def block(size, dx, dy, bottom=DECK_TOP, vel=(0.0, 0.0, 0.0), **kw):  # dx, dy from the mark, metres
        q = quat(**kw)
        half = np.abs(_rot(q)) @ (np.asarray(size) / 2)
        pos = np.array([DECK_C[0] + dx, DECK_C[1] + dy, bottom + half[2]])
        return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), pos - half, pos + half)

    def table(size, xy):
        return block(size, xy[0] - DECK_C[0], xy[1] - DECK_C[1], bottom=TOP)

    def scene(large=None, small=None):
        return {
            "ballast_large": large if large is not None else table(LARGE, LARGE_START),
            "ballast_small": small if small is not None else table(SMALL, SMALL_START),
        }

    L_DONE = (LARGE_PLACE[0] - DECK_C[0], LARGE_PLACE[1] - DECK_C[1])
    S_DONE = (SMALL_PLACE[0] - DECK_C[0], SMALL_PLACE[1] - DECK_C[1])
    done_l, done_s = block(LARGE, *L_DONE), block(SMALL, *S_DONE)
    ux, uy, vx, vy = 0.8, 0.6, -0.6, 0.8  # the instrument's direction from the mark, and its perpendicular

    def opposite(d):  # both blocks d metres from the mark on the side away from the instrument, side by side
        return scene(block(LARGE, -d * ux + 0.025 * vx, -d * uy + 0.025 * vy), block(SMALL, -d * ux - 0.050 * vx, -d * uy - 0.050 * vy))

    BAR_SOLO = (-0.150 * ux, -0.150 * uy)  # where the bar alone balances the instrument: 120 g x 150 mm = 18,000 g mm
    bar_solo = block(LARGE, *BAR_SOLO)
    cases = {
        "start": (scene(), 0.0, False),
        "the demo's placement balances exactly": (scene(done_l, done_s), 1.0, True),
        "both 100 mm opposite the instrument balance exactly": (opposite(0.100), 1.0, True),
        "both 120 mm opposite, overshoot 7.5 mm, half": (opposite(0.120), 0.5, False),
        "both mirrored at 60 mm, 15 mm off, earns nothing": (opposite(0.060), 0.0, False),
        "both blocks beside the mark, 37.5 mm off": (scene(block(LARGE, 0.0, -0.030), block(SMALL, 0.0, 0.060)), 0.0, False),
        "bar mirrored, cube at the mark, 22.5 mm off": (scene(block(LARGE, -0.048, -0.036), block(SMALL, 0.0, 0.0, bottom=DECK_TOP + 0.0005)), 0.0, False),
        "bar alone at the demo's spot, 18.3 mm off": (scene(done_l), 0.0, False),
        "bar alone on the deck, balancing exactly, capped": (scene(bar_solo), 0.25, False),
        "cube alone 160 mm opposite, 23.3 mm off": (scene(None, block(SMALL, -0.160 * ux, -0.160 * uy)), 0.0, False),
        "both on the table beside the deck": (scene(block(LARGE, -0.080, -0.160, bottom=TOP), block(SMALL, 0.020, -0.160, bottom=TOP)), 0.0, False),
        "cube stacked on the balancing bar is missing, capped": (scene(bar_solo, block(SMALL, *BAR_SOLO, bottom=DECK_TOP + LARGE[2])), 0.25, False),
        "cube on the instrument is missing, capped": (scene(bar_solo, block(SMALL, *INSTR_OFF, bottom=DECK_TOP + INSTR[2])), 0.25, False),
        "cube on the table beside the deck is missing, capped": (scene(bar_solo, block(SMALL, -0.080, -0.150, bottom=TOP)), 0.25, False),
        "cube held 10 mm above its spot is missing": (scene(done_l, block(SMALL, *S_DONE, bottom=DECK_TOP + 0.010)), 0.0, False),
        "cube leaning 8 degrees is missing": (scene(done_l, block(SMALL, *S_DONE, tilt=math.radians(8))), 0.0, False),
        "cube still moving is missing": (scene(done_l, block(SMALL, *S_DONE, vel=(0.02, 0.0, 0.0))), 0.0, False),
        "both still moving": (scene(block(LARGE, *L_DONE, vel=(0.0, 0.0, -0.02)), block(SMALL, *S_DONE, vel=(0.02, 0.0, 0.0))), 0.0, False),
        "bar stood on its end still counts": (scene(block(LARGE, *L_DONE, end_up=True), done_s), 1.0, True),
        "bar yawed 30 degrees and cube upside down still count": (scene(block(LARGE, *L_DONE, yaw=math.radians(30)), block(SMALL, *S_DONE, tilt=math.pi)), 1.0, True),
        "4 degrees of tilt still counts": (scene(block(LARGE, *L_DONE, tilt=math.radians(4)), done_s), 1.0, True),
        "cube on a mark bar, 0.5 mm up, still counts": (scene(done_l, block(SMALL, *S_DONE, bottom=DECK_TOP + 0.0005)), 1.0, True),
        "exactly 5.0 mm off is balanced": (scene(block(LARGE, L_DONE[0] + 0.020, L_DONE[1]), done_s), 1.0, True),
        "5.4 mm off is half": (scene(block(LARGE, L_DONE[0] + 0.0216, L_DONE[1]), done_s), 0.5, False),
        "exactly 10.0 mm off is half": (scene(block(LARGE, L_DONE[0], L_DONE[1] + 0.040), done_s), 0.5, False),
        "10.4 mm off is a quarter": (scene(block(LARGE, L_DONE[0], L_DONE[1] + 0.0416), done_s), 0.25, False),
        "exactly 12.5 mm off is a quarter": (scene(block(LARGE, L_DONE[0] + 0.050, L_DONE[1]), done_s), 0.25, False),
        "12.9 mm off earns nothing": (scene(block(LARGE, L_DONE[0] + 0.0516, L_DONE[1]), done_s), 0.0, False),
        "bar overhanging the deck edge, centre 5 mm inside, counts": (scene(block(LARGE, -0.125, -0.040), block(SMALL, 0.010, -0.100)), 1.0, True),
    }
    for name, (st, want, want_success) in cases.items():
        got, got_success = score(st), success(st)
        assert abs(got - want) < 1e-9 and got_success == want_success, (name, got, got_success, want, want_success)
    assert _com_error(scene(done_l, done_s), ["ballast_large", "ballast_small"]) < 1e-9
    assert abs(distance(scene()) - _com_error(scene(), ["ballast_large", "ballast_small"])) < 1e-12
    return len(cases)


if __name__ == "__main__":
    # Offline checks, no GPU: the scoring rules on hand-built states, then the left arm's plan replayed through the
    # kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm.
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
