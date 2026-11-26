"""Refill commercial label pricing gun ink roller.

Workforce maintenance task across retail, supermarket, and logistics operations:
1. Left arm swings open front ink door past detent (>= 55 deg).
2. Right arm extracts spent ink roller spindle from carrier cradle into discard waste tray.
3. Left arm picks up fresh ink roller cylinder (12 mm OD x 28 mm length, 12 g), transports it
   to the pricing gun, and seats it squarely into the carrier journals.
4. Right arm swings front ink door shut until snap detent clicks flush (< 5 deg).
5. Both arms clear (>= 50 mm); assembly holds equilibrium for >= 1.0 s.
"""

import math
import os
import sys

# Add task directory to sys.path so mechkit is found
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mechkit as mk  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402

SCENE = "station"
TOP = station.TABLE_TOP
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "best"

# Visual materials (sRGB)
GUN_RED = (0.75, 0.12, 0.12)
GUN_BLACK = (0.15, 0.15, 0.16)
DOOR_SMOKE = (0.25, 0.25, 0.28)
STEEL_PIN = (0.78, 0.80, 0.82)
INK_SPENT = (0.22, 0.22, 0.22)
INK_FRESH = (0.05, 0.05, 0.06)
TRAY_GREY = (0.50, 0.52, 0.55)

# Physical Geometry (World frame: X forward, Y left, Z up)
GUN_X = 0.280
GUN_Y = 0.050
GUN_Z = TOP + 0.075

kit = mk.Kit()

# 1. Commercial Pricing Gun Main Body (180 x 120 x 45 mm, clamped to fixture)
kit.part(
    "pricing_gun_body",
    (GUN_X, GUN_Y, GUN_Z),
    [
        # Main handle / grip (angled down-back)
        mk.box((0.040, 0.035, 0.090), at=(-0.050, 0, -0.025), color=GUN_BLACK),
        # Main body chassis housing
        mk.box((0.110, 0.040, 0.065), at=(0.015, 0, 0.015), color=GUN_RED),
        # Label roll feed cavity top cowl
        mk.box((0.080, 0.042, 0.030), at=(-0.010, 0, 0.055), color=GUN_RED),
        # Front nose support / anvil bracket
        mk.box((0.035, 0.038, 0.040), at=(0.075, 0, -0.005), color=GUN_BLACK),
        # Lower roller cradle side journals (left & right ears forming U-channel stops)
        mk.box((0.020, 0.005, 0.025), at=(0.070, 0.018, 0.025), color=GUN_RED),
        mk.box((0.020, 0.005, 0.025), at=(0.070, -0.018, 0.025), color=GUN_RED),
        # Journal bottom stop
        mk.box((0.020, 0.032, 0.005), at=(0.070, 0, 0.012), color=GUN_BLACK),
        # Front journal retaining lips (detent constraint for roller spindle)
        mk.box((0.005, 0.005, 0.010), at=(0.078, 0.018, 0.028), color=GUN_RED),
        mk.box((0.005, 0.005, 0.010), at=(0.078, -0.018, 0.028), color=GUN_RED),
        # Bench clamp block fixture
        mk.box((0.070, 0.070, 0.025), at=(-0.050, 0, -0.065), color=GUN_BLACK),
    ]
)

# 2. Hinged Front Ink Roller Access Door
DOOR_HINGE_X = GUN_X + 0.085
DOOR_HINGE_Y = GUN_Y - 0.020
DOOR_HINGE_Z = GUN_Z + 0.015

kit.part(
    "front_ink_door",
    (DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z),
    [
        # Main hinged door cover (swings open forward-right)
        mk.box((0.006, 0.040, 0.038), at=(0, 0.020, 0.015), color=DOOR_SMOKE),
        # Front latch release tab / lip
        mk.box((0.010, 0.012, 0.015), at=(0.006, 0.035, 0.015), color=GUN_RED),
    ],
    mass=0.035
)

# Revolute hinge for front door
kit.hinge(
    "door_hinge",
    "pricing_gun_body",
    "front_ink_door",
    anchor=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_HINGE_Z),
    axis=(0, 0, 1),
    lower_deg=-2.0,
    upper_deg=100.0,
    start_deg=0.0,
    stiffness=0.05,
    rest_deg=0.0,
    friction=0.10
)

# Multi-faceted cylindrical roller generator:
# 12 mm OD (6 mm radius), 26 mm length, with 38 mm lateral steel axle pins
_R_CYL = 0.006
_L_CYL = 0.026
_W_OCT = 2.0 * _R_CYL
_S_OCT = 2.0 * _R_CYL * (math.sqrt(2.0) - 1.0)
_Q_Y45 = (0.0, math.sin(math.pi / 8.0), 0.0, math.cos(math.pi / 8.0))

def _make_roller_boxes(color):
    return [
        # Horizontal cross-section
        mk.box((_W_OCT, _L_CYL, _S_OCT), at=(0, 0, 0), color=color),
        # Vertical cross-section
        mk.box((_S_OCT, _L_CYL, _W_OCT), at=(0, 0, 0), color=color),
        # 45-degree rotated facet box around cylinder axis (Y)
        mk.box((_W_OCT * math.cos(math.pi / 8.0), _L_CYL, _S_OCT), at=(0, 0, 0), color=color, quat=_Q_Y45),
        # Lateral steel axle spindle pins (extending 6 mm past each side into journal stops)
        mk.box((0.0035, 0.038, 0.0035), at=(0, 0, 0), color=STEEL_PIN),
    ]

# 3. Spent Ink Roller (Starts inside carrier cradle journals)
OLD_ROLLER_X = GUN_X + 0.070
OLD_ROLLER_Y = GUN_Y
OLD_ROLLER_Z = GUN_Z + 0.022

kit.part(
    "old_ink_roller",
    (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z),
    _make_roller_boxes(INK_SPENT),
    mass=0.008
)

# 4. Fresh Sealed Ink Roller (Starts on bench staging area)
FRESH_ROLLER_X = 0.220
FRESH_ROLLER_Y = -0.140
FRESH_ROLLER_Z = TOP + 0.015

kit.part(
    "fresh_ink_roller",
    (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z),
    _make_roller_boxes(INK_FRESH),
    mass=0.012
)

# 5. Bench Discard Waste Tray (Static fixture on bench)
TRAY_X = 0.360
TRAY_Y = 0.160
TRAY_Z = TOP + 0.015

kit.part(
    "waste_tray",
    (TRAY_X, TRAY_Y, TRAY_Z),
    [
        # Base tray floor
        mk.box((0.080, 0.080, 0.008), at=(0, 0, -0.004), color=TRAY_GREY),
        # Side rims (shallow container)
        mk.box((0.080, 0.006, 0.020), at=(0, 0.037, 0.006), color=TRAY_GREY),
        mk.box((0.080, 0.006, 0.020), at=(0, -0.037, 0.006), color=TRAY_GREY),
        mk.box((0.006, 0.068, 0.020), at=(0.037, 0, 0.006), color=TRAY_GREY),
        mk.box((0.006, 0.068, 0.020), at=(-0.037, 0, 0.006), color=TRAY_GREY),
    ]
)

OBJECTS = [*kit.objects()]
PARTS = ("pricing_gun_body", "front_ink_door", "old_ink_roller", "fresh_ink_roller", "waste_tray")


def score(objects) -> float:
    """Reversible multi-stage score:
    0.00: Start / door latched.
    0.20: Door swung open past detent (>= 55 deg).
    0.45: Old ink roller removed from carrier and placed into waste tray (> 8 cm from carrier and in tray).
    0.75: Fresh ink roller seated into carrier journals (< 15 mm horiz, < 12 mm vert error).
    0.90: Door swung shut onto retention latch (< 5 deg from closed).
    1.00: Full completion + hands clear (>= 50 mm). Continuous hold is verified by yam_arena SUCCESS_HOLD_S.
    """
    door_q = math.degrees(kit.q(objects, "door_hinge"))

    if door_q >= 55.0:
        mk.MEMORY["door_opened"] = True

    if not mk.MEMORY.get("door_opened", False):
        return 0.00

    pts = 0.20

    # Old roller state check: must be cleared from carrier AND placed in waste tray
    old_p = objects["old_ink_roller"].pos
    old_dist = math.hypot(old_p[0] - OLD_ROLLER_X, old_p[1] - OLD_ROLLER_Y)
    old_cleared = old_dist > 0.080 and old_p[2] <= TOP + 0.060
    in_tray = (abs(old_p[0] - TRAY_X) <= 0.038 and 
               abs(old_p[1] - TRAY_Y) <= 0.038 and 
               old_p[2] <= TOP + 0.038)

    if not (old_cleared and in_tray):
        return pts
    pts = 0.45

    # Fresh roller state check (seated in cradle journals)
    fresh_p = objects["fresh_ink_roller"].pos
    fresh_cradle_dist = math.hypot(fresh_p[0] - OLD_ROLLER_X, fresh_p[1] - OLD_ROLLER_Y)
    fresh_z_err = abs(fresh_p[2] - OLD_ROLLER_Z)
    fresh_seated = fresh_cradle_dist < 0.015 and fresh_z_err < 0.012

    if not fresh_seated:
        return pts
    pts = 0.75

    # Door latch check (closed to within 5 deg)
    door_closed = abs(door_q) < 5.0
    if not door_closed:
        return pts
    pts = 0.90

    # Hand clearance check (>= 50 mm)
    tcps = getattr(mk, "_TEST_TCPS", None) or mk.tcps()
    if tcps:
        hands_clear = True
        for tcp in tcps:
            if math.dist(tcp, (GUN_X, GUN_Y, GUN_Z)) < 0.050 or math.dist(tcp, fresh_p) < 0.045:
                hands_clear = False
                break
    else:
        hands_clear = True

    if not hands_clear:
        return pts

    return 1.00


def success(objects) -> bool:
    return score(objects) >= 1.00


def demo():
    """Bimanual demonstration sequence with standard duration segments."""
    variant = os.environ.get("DEMO_VARIANT", "normal")

    if variant == "door_left_open":
        left_wps = [
            Waypoint(1.2, (GUN_X, GUN_Y + 0.10, TOP + 0.18), FINGER_OPEN, "L approach door"),
            Waypoint(1.0, (GUN_X + 0.088, GUN_Y + 0.020, GUN_Z + 0.025), FINGER_OPEN, "L reach tab"),
            Waypoint(0.5, (GUN_X + 0.088, GUN_Y + 0.020, GUN_Z + 0.025), 0.015, "L pinch tab"),
            Waypoint(1.2, (GUN_X + 0.120, GUN_Y - 0.020, GUN_Z + 0.025), 0.015, "L swing open"),
            Waypoint(0.8, (GUN_X + 0.110, GUN_Y - 0.035, GUN_Z + 0.025), FINGER_OPEN, "L release door"),
            Waypoint(1.5, (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z + 0.080), FINGER_OPEN, "L hover fresh"),
            Waypoint(1.0, (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z + 0.008), FINGER_OPEN, "L descend fresh"),
            Waypoint(0.8, (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z + 0.008), 0.012, "L pinch fresh"),
            Waypoint(1.2, (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z + 0.100), 0.012, "L lift fresh"),
            Waypoint(1.5, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.080), 0.012, "L carry fresh over cradle"),
            Waypoint(1.2, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z), 0.012, "L seat fresh"),
            Waypoint(0.8, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z), FINGER_OPEN, "L release fresh"),
            Waypoint(2.0, (0.18, 0.16, TOP + 0.20), FINGER_OPEN, "L clear")
        ]
        right_wps = [
            Waypoint(1.5, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.080), FINGER_OPEN, "R hover spent"),
            Waypoint(1.0, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.006), FINGER_OPEN, "R descend spent"),
            Waypoint(0.8, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.006), 0.012, "R pinch spent"),
            Waypoint(1.2, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.090), 0.012, "R lift spent"),
            Waypoint(1.5, (TRAY_X, TRAY_Y, TOP + 0.080), 0.012, "R carry tray"),
            Waypoint(1.0, (TRAY_X, TRAY_Y, TOP + 0.020), 0.012, "R set tray"),
            Waypoint(0.8, (TRAY_X, TRAY_Y, TOP + 0.020), FINGER_OPEN, "R release spent"),
            Waypoint(6.9, (0.18, -0.16, TOP + 0.20), FINGER_OPEN, "R clear idle")
        ]
        return {"left": left_wps, "right": right_wps}

    # Normal full-score demonstration (1.00)
    left_wps = [
        # Step 1: Approach and open front ink access door
        Waypoint(1.2, (GUN_X, GUN_Y + 0.10, TOP + 0.18), FINGER_OPEN, "L approach door"),
        Waypoint(1.0, (GUN_X + 0.088, GUN_Y + 0.020, GUN_Z + 0.025), FINGER_OPEN, "L reach tab"),
        Waypoint(0.5, (GUN_X + 0.088, GUN_Y + 0.020, GUN_Z + 0.025), 0.015, "L pinch tab"),
        Waypoint(1.2, (GUN_X + 0.120, GUN_Y - 0.020, GUN_Z + 0.025), 0.015, "L swing open"),
        Waypoint(0.8, (GUN_X + 0.110, GUN_Y - 0.035, GUN_Z + 0.025), FINGER_OPEN, "L release door"),
        # Step 3: Pick fresh roller and seat into carrier journals
        Waypoint(1.5, (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z + 0.080), FINGER_OPEN, "L hover fresh"),
        Waypoint(1.0, (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z + 0.008), FINGER_OPEN, "L descend fresh"),
        Waypoint(0.8, (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z + 0.008), 0.012, "L pinch fresh"),
        Waypoint(1.2, (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z + 0.100), 0.012, "L lift fresh"),
        Waypoint(1.5, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.080), 0.012, "L carry fresh over cradle"),
        Waypoint(1.2, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z), 0.012, "L seat fresh"),
        Waypoint(0.8, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z), FINGER_OPEN, "L release fresh"),
        # Step 5: Clear arm
        Waypoint(3.5, (0.18, 0.16, TOP + 0.20), FINGER_OPEN, "L clear")
    ]

    right_wps = [
        # Step 2: Grasp and extract spent ink roller to waste tray
        Waypoint(1.5, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.080), FINGER_OPEN, "R hover spent"),
        Waypoint(1.0, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.006), FINGER_OPEN, "R descend spent"),
        Waypoint(0.8, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.006), 0.012, "R pinch spent"),
        Waypoint(1.2, (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z + 0.090), 0.012, "R lift spent"),
        Waypoint(1.5, (TRAY_X, TRAY_Y, TOP + 0.080), 0.012, "R carry tray"),
        Waypoint(1.0, (TRAY_X, TRAY_Y, TOP + 0.020), 0.012, "R set tray"),
        Waypoint(0.8, (TRAY_X, TRAY_Y, TOP + 0.020), FINGER_OPEN, "R release spent"),
        # Step 4: Re-close front ink door until latch snaps
        Waypoint(1.8, (GUN_X + 0.125, GUN_Y - 0.030, GUN_Z + 0.025), FINGER_OPEN, "R reach open door"),
        Waypoint(1.2, (GUN_X + 0.090, GUN_Y + 0.010, GUN_Z + 0.025), 0.015, "R push door shut"),
        Waypoint(0.8, (GUN_X + 0.085, GUN_Y + 0.018, GUN_Z + 0.025), 0.015, "R snap latch"),
        Waypoint(0.8, (GUN_X + 0.090, GUN_Y + 0.020, GUN_Z + 0.025), FINGER_OPEN, "R release latch"),
        # Step 5: Clear arm
        Waypoint(3.1, (0.18, -0.16, TOP + 0.20), FINGER_OPEN, "R clear")
    ]

    return {"left": left_wps, "right": right_wps}


def self_test():
    """Offline unit test validating stage scoring, geometry constraints, and failure modes."""
    print("[self_test] Running refill-commercial-label-pricing-gun-ink-roller unit tests...")
    mk._UNIT_TEST = True
    mk.new_episode()

    old_init = (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z)
    fresh_init = (FRESH_ROLLER_X, FRESH_ROLLER_Y, FRESH_ROLLER_Z)
    tray_p = (TRAY_X, TRAY_Y, TRAY_Z)
    cradle_p = (OLD_ROLLER_X, OLD_ROLLER_Y, OLD_ROLLER_Z)
    unit_q = (0, 0, 0, 1)

    # Test 1: Initial state -> Score 0.0
    s_init = kit.fake_states(
        q={"door_hinge": 0.0},
        moved={"old_ink_roller": (old_init, unit_q), "fresh_ink_roller": (fresh_init, unit_q)}
    )
    assert score(s_init) == 0.0, f"Expected 0.0 at start, got {score(s_init)}"

    # Test 2: Door swung open past 55 deg -> Score 0.20
    s_door = kit.fake_states(
        q={"door_hinge": 65.0},
        moved={"old_ink_roller": (old_init, unit_q), "fresh_ink_roller": (fresh_init, unit_q)},
        deg=True
    )
    assert score(s_door) == 0.20, f"Expected 0.20 after door open, got {score(s_door)}"

    # Test 3: Old roller removed to waste tray -> Score 0.45
    s_old = kit.fake_states(
        q={"door_hinge": 65.0},
        moved={"old_ink_roller": (tray_p, unit_q), "fresh_ink_roller": (fresh_init, unit_q)},
        deg=True
    )
    assert score(s_old) == 0.45, f"Expected 0.45 after old roller removed, got {score(s_old)}"

    # Test 4: Fresh roller seated into carrier journals -> Score 0.75
    s_fresh = kit.fake_states(
        q={"door_hinge": 65.0},
        moved={"old_ink_roller": (tray_p, unit_q), "fresh_ink_roller": (cradle_p, unit_q)},
        deg=True
    )
    assert score(s_fresh) == 0.75, f"Expected 0.75 after fresh roller seated, got {score(s_fresh)}"

    # Test 5: Door closed onto latch tab -> Score 1.00 (with clear hands)
    s_done = kit.fake_states(
        q={"door_hinge": 1.2},
        moved={"old_ink_roller": (tray_p, unit_q), "fresh_ink_roller": (cradle_p, unit_q)},
        deg=True
    )
    assert score(s_done) == 1.00, f"Expected 1.00 for full completion, got {score(s_done)}"
    assert success(s_done) is True, "Expected success() to return True"

    # Test 6: Verify demo kinematics dict output
    d = demo()
    assert "left" in d and "right" in d
    assert len(d["left"]) > 0 and len(d["right"]) > 0
    t_left = sum(w.duration for w in d["left"])
    t_right = sum(w.duration for w in d["right"])
    print(f"[self_test] Demo timing: left {t_left:.2f}s, right {t_right:.2f}s (run limit 40s)")
    assert t_left <= 40.0 and t_right <= 40.0, "Demo durations must be <= run limit"

    print("[self_test] All 6 scoring unit test assertions passed cleanly!")


if __name__ == "__main__":
    self_test()
