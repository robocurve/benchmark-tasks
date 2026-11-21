"""Scoring and scrape-rule tests on made-up end states (no simulator)."""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import gum  # noqa: E402
try:
    import scrape_gum_off_shoe_sole as T  # noqa: E402  (the frozen task folder)
except ImportError:
    import gum_off_shoe as T  # noqa: E402  (the library scene)
from yam_arena.task_file import ObjectState  # noqa: E402

Z3 = np.zeros(3)


def yaw_quat(yaw, pitch=0.0):
    """Yaw about z, after a pitch about the object's own y (nose down for positive pitch)."""
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return np.array([-sy * sp, cy * sp, sy * cp, cy * cp])


def state(pos, size, quat=None, vel=Z3):
    pos = np.asarray(pos, dtype=float)
    half = np.asarray(size) / 2
    return ObjectState(pos=pos, quat_xyzw=yaw_quat(0) if quat is None else quat, lin_vel=np.asarray(vel, dtype=float),
                       ang_vel=Z3, low=pos - half, high=pos + half)


def start():
    s = {o.name: state(o.pos, o.size, yaw_quat(o.yaw)) for o in T.OBJECTS}
    gum.reset()
    return s


def sole_world(local_xy, above=0.0):
    return np.array(T._shoe_to_world(local_xy, T.SOLE_LOCAL_Z + above))


def taken(*names):
    """Mark strips as already scraped loose and carried off by the blade, as a real stroke would leave them, so a test
    can teleport them away without fouling."""
    for name in names or [p.name for p in T.GUM_PIECES]:
        gum._STATE["unlocked"].add(name)
        gum._STATE["taken"].add(name)


def moved_along_sole(name, dx):
    """A strip's state at its start place, slid ``dx`` m along the sole."""
    x, y, z = T.GUM_LOCAL[name]
    return sole_world((x + dx, y), z - T.SOLE_LOCAL_Z)


def knife_at(edge_mid_world, heading, pitch_deg):
    """The knife with the middle of its edge at a world point, its blade pointing along ``heading`` (rad, world),
    tipped nose-down by ``pitch_deg``."""
    q = yaw_quat(heading, math.radians(pitch_deg))
    mid = (np.array(T.GUM["edge"][0]) + np.array(T.GUM["edge"][1])) / 2
    return state(np.asarray(edge_mid_world) - gum.rot(q) @ mid, T.KNIFE_SIZE, q)


def test_start_scores_nothing():
    s = start()
    assert T.score(s) == 0.0 and not T.success(s) and T.removed_count(s) == 0 and not gum.unlocked()


def test_the_wad_is_eight_hinged_strips_round_domed_and_just_off_the_smooth_disc():
    assert len(T.GUM_PIECES) == 8 and len(T.GUM["neighbours"]) == 7
    assert all(a != b and {a, b} <= set(T.GUM_LOCAL) for a, b in T.GUM["neighbours"])
    centre = np.array(T.PIVOT_LOCAL)
    offs = np.array([T.GUM_LOCAL[p.name][:2] for p in T.GUM_PIECES]) - centre
    assert np.allclose(offs[:, 1], 0.0)                                           # every strip is centred on the wad
    assert np.isclose(offs[:, 0].max() - offs[:, 0].min() + T.GUM["piece_size"], 0.040)   # 4 cm along the shoe
    widths = [p.size[1] for p in T.GUM_PIECES]
    assert max(widths) < 0.040 and widths == widths[::-1] and widths[0] < widths[3]       # round: widest in the middle
    for p, off in zip(T.GUM_PIECES, offs):                                        # every corner short of the tread
        assert math.hypot(abs(off[0]) + T.GUM["piece_size"] / 2, p.size[1] / 2) < 0.0225
    thick = [p.size[2] for p in T.GUM_PIECES]
    assert max(thick) == thick[3] == thick[4] > 0.0034 and min(thick) == thick[0] == thick[7] < 0.0021
    for p in T.GUM_PIECES:                                                        # each stands the gap off the sole
        assert abs(T.GUM_LOCAL[p.name][2] - T.SOLE_LOCAL_Z - T.GUM["gap"] - p.size[2] / 2) < 1e-12
    assert T.GUM["gap"] > 0.0008                                                  # room for the blade's 0.8 mm edge
    assert T.GUM["gap"] < T.GUM["under"] < T.GUM["gap"] + min(thick)             # under the wad, never over it


def test_each_strip_off_scores_its_share_and_all_off_with_things_at_rest_is_success():
    s = start()
    taken()
    n = len(T.GUM_PIECES)
    seen = []
    for k, p in enumerate(T.GUM_PIECES, 1):
        s[p.name] = state((0.5, -0.3, T.TOP + 0.001), p.size)          # on the table, away from the shoe
        assert T.score(s) == math.floor(9 * k / n) / 10
        seen.append(T.score(s))
        assert not T.success(s) or k == n
    assert seen == [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.9]
    assert T.success(s)


def test_gum_pushed_along_the_sole_is_not_removed():
    s = start()
    taken()                                                           # so a 0 here is on_sole's doing, not a foul
    p = T.GUM_PIECES[0]
    s[p.name] = state(sole_world((-0.08, 0.0), 0.001), p.size)       # smeared toward the heel
    assert T.score(s) == 0.0


def test_gum_lifted_off_on_the_blade_counts():
    s = start()
    taken()
    for k, p in enumerate(T.GUM_PIECES[:2], 1):
        s[p.name] = state(sole_world(T.PIVOT_LOCAL, 0.03), p.size)   # 3 cm above the sole
        assert T.removed_count(s) == k
    assert T.score(s) == 0.2                                          # two of the eight strips


def test_no_success_while_the_knife_is_held_up_or_moving_or_the_shoe_is_off_the_table():
    s = start()
    taken()
    for p in T.GUM_PIECES:
        s[p.name] = state((0.5, -0.3, T.TOP + 0.001), p.size)
    held = dict(s, knife=state((0.3, -0.2, T.TOP + 0.15), T.KNIFE_SIZE))
    moving = dict(s, knife=state(T.KNIFE.pos, T.KNIFE_SIZE, vel=(0.05, 0, 0)))
    fallen = dict(s, shoe=state((0.36, 0.9, T.TOP - 0.5), T.SHOE_SIZE, yaw_quat(T.SHOE_YAW)))
    assert not T.success(held) and T.score(held) == 0.9
    assert not T.success(moving)
    assert not T.success(fallen) and T.score(fallen) == 0.0


def test_fingers_never_unlock_gum():
    s = start()                                                       # knife still on the table, far from the shoe
    for _ in range(50):
        T.score(s)
    assert not gum.unlocked()


def test_a_low_blade_on_the_sole_unlocks_the_strips_it_gets_under():
    s = start()
    centre = sole_world(T.PIVOT_LOCAL, 0.001)                         # the edge 1 mm above the sole: under the wad
    # scraping toward +y (world), edge just short of the wad's first strip
    s["knife"] = knife_at(centre + np.array([0, -0.022, 0]), math.pi / 2, 20)
    T.score(s)
    assert gum.unlocked() == {"gum_7"}                                # the near strip only
    s["knife"] = knife_at(centre + np.array([0, 0.012, 0]), math.pi / 2, 20)
    T.score(s)
    assert len(gum.unlocked()) >= 3 and "gum_7" in gum.unlocked()     # and the strips the edge has now reached


def test_steep_blade_high_blade_and_backward_blade_do_not_unlock():
    centre = sole_world(T.PIVOT_LOCAL, 0.001)
    for knife in (knife_at(centre + np.array([0, -0.022, 0]), math.pi / 2, 60),          # too steep
                  knife_at(centre + np.array([0, -0.022, 0.01]), math.pi / 2, 20),       # 1 cm above the sole
                  knife_at(centre + np.array([0, -0.026, 0]), -math.pi / 2, 20)):        # edge pointing away
        s = start()
        s["knife"] = knife
        T.score(s)
        assert not gum.unlocked()


def test_a_blade_riding_over_the_gum_unlocks_nothing():
    """The edge has to be down on the sole, under the wad: 4 mm up it is on top of the gum, at the rim and in the
    middle alike, and nothing lets go."""
    for along in (-0.022, -0.012, 0.0, 0.012):
        s = start()
        s["knife"] = knife_at(sole_world(T.PIVOT_LOCAL, 0.004) + np.array([0, along, 0]), math.pi / 2, 20)
        T.score(s)
        assert not gum.unlocked()
    s = start()
    s["knife"] = knife_at(sole_world(T.PIVOT_LOCAL, T.GUM["under"] - 0.0002) + np.array([0, -0.022, 0]), math.pi / 2, 20)
    T.score(s)
    assert gum.unlocked() == {"gum_7"}                                # just inside the limit, it does


def test_reset_locks_everything_again():
    s = start()
    s["knife"] = knife_at(sole_world(T.PIVOT_LOCAL, 0.001), math.pi / 2, 20)
    T.score(s)
    assert gum.unlocked()
    gum.reset()
    assert not gum.unlocked()


def test_gum_moved_without_the_blade_fouls_the_trial():
    s = start()                                                       # knife still on the table: nothing unlocked
    p = T.GUM_PIECES[2]
    s[p.name] = state(moved_along_sole(p.name, 0.005), p.size)
    assert T.score(s) == 0.0 and p.name in gum.fouled()
    for q in T.GUM_PIECES:                                            # every strip then put on the table: still 0
        s[q.name] = state((0.5, -0.3, T.TOP + 0.001), q.size)
    assert T.score(s) == 0.0 and not T.success(s) and gum.fouled()


def test_gum_the_blade_lifts_off_never_fouls_wherever_it_goes_after():
    s = start()
    centre = sole_world(T.PIVOT_LOCAL, 0.001)
    s["knife"] = knife_at(centre + np.array([0, -0.022, 0]), math.pi / 2, 20)
    T.score(s)
    s["knife"] = knife_at(centre + np.array([0, 0.012, 0]), math.pi / 2, 20)
    T.score(s)
    loose = sorted(gum.unlocked())
    assert len(loose) >= 3
    for name in loose:                                                # up 3 cm on the blade, the knife still there
        s[name] = state(sole_world(T.GUM_LOCAL[name][:2], 0.03), T.GUM_PIECES[0].size)
    assert T.score(s) == math.floor(9 * len(loose) / len(T.GUM_PIECES)) / 10 and not gum.fouled()
    for name in loose:                                                # and then far off, on the table
        s[name] = state((0.5, -0.3, T.TOP + 0.001), T.GUM_PIECES[0].size)
    assert T.score(s) > 0 and not gum.fouled()


def test_an_unlocked_strip_moved_once_the_knife_is_away_fouls():
    s = start()
    s["knife"] = knife_at(sole_world(T.PIVOT_LOCAL, 0.001) + np.array([0, -0.022, 0]), math.pi / 2, 20)
    T.score(s)
    assert gum.unlocked() == {"gum_7"} and not gum.fouled()
    s["knife"] = state(T.KNIFE.pos, T.KNIFE_SIZE, yaw_quat(T.KNIFE.yaw))   # put back on the table, far away
    T.score(s)
    assert not gum.fouled()
    s["gum_7"] = state(moved_along_sole("gum_7", 0.005), T.GUM_PIECES[7].size)
    assert T.score(s) == 0.0 and "gum_7" in gum.fouled()


def test_a_millimetre_of_give_is_not_a_foul():
    s = start()
    p = T.GUM_PIECES[4]
    s[p.name] = state(moved_along_sole(p.name, 0.001), p.size)
    T.score(s)
    assert not gum.fouled()


def test_reset_clears_a_foul():
    s = start()
    p = T.GUM_PIECES[0]
    s[p.name] = state(moved_along_sole(p.name, 0.005), p.size)
    T.score(s)
    assert gum.fouled()
    gum.reset()
    assert gum.fouled() == "" and not gum._STATE["taken"]


def test_the_demo_reaches_and_stays_in_the_joint_limits():
    import types
    if "labkit" not in sys.modules:                                   # the library scene has no labkit beside it
        try:
            import labkit  # noqa: F401
        except ImportError:
            from yam_arena.demo import Waypoint
            from dataclasses import dataclass

            @dataclass
            class Pose(Waypoint):
                rot: np.ndarray | None = None
            sys.modules["labkit"] = types.SimpleNamespace(Pose=Pose)
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    plan = T.demo()
    assert set(plan) == {"left", "right"}
    for arm, waypoints in plan.items():
        ap = ArmPlan(waypoints, station.BASES[arm])
        acts = np.array([ap.action7((k + 1) / 20)[0] for k in range(round(ap.duration * 20))])
        assert np.all(acts[:, :6] >= ik.LOWER) and np.all(acts[:, :6] <= ik.UPPER)
        assert ap.duration < 60                                       # well inside the 120 s run limit


def test_an_unlocked_strip_lets_go_only_once_drawn_far_enough_from_its_place():
    s = start()
    centre = sole_world(T.PIVOT_LOCAL, 0.001)
    s["knife"] = knife_at(centre + np.array([0, -0.022, 0]), math.pi / 2, 20)
    T.score(s)
    assert gum.unlocked() == {"gum_7"} and not gum.released()
    p = next(q for q in T.GUM_PIECES if q.name == "gum_7")
    s[p.name] = state(np.array(p.pos) + np.array([0, 0, T.GUM["let_go"] - 0.0005]), p.size, yaw_quat(p.yaw))
    T.score(s)
    assert not gum.released()                                         # stretched, still held
    s[p.name] = state(np.array(p.pos) + np.array([0, 0, T.GUM["let_go"] + 0.0005]), p.size, yaw_quat(p.yaw))
    T.score(s)
    assert gum.released() == {"gum_7"} and not gum.fouled()
    gum.reset()
    assert not gum.released()
