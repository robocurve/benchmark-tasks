"""Offline checks, no simulator: the eggs' physics (fresh lies, older stands, floater floats, from the air cell alone),
the water model in the flared bowl, and the scorer on scripted end states, including the cheats (do nothing, skip
the test, throw out the wrong egg or all of them, drop or squeeze an egg, leave a hand on an egg, put the floater in the
dish without testing it).

    python -m pytest -q test_task.py
"""
import math
import os
from pathlib import Path

os.environ.setdefault("EGG_SEED", "0")

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402

HERE = Path(__file__).parent
M = load(str(HERE / "task.py")).module
import eggmodel as em  # noqa: E402
import floatkit as fk  # noqa: E402
import yamkit as kit  # noqa: E402

DT = M.DT
AWAY = {"left_gripper": (0.10, 0.35, M.TOP + 0.30), "right_gripper": (0.10, -0.35, M.TOP + 0.30)}
DOWN = Rotation.from_euler("x", 180, degrees=True).as_matrix()


# -- the eggs' physics -------------------------------------------------------------------------------------------------
def test_egg_size_and_mass_are_a_us_large_egg():
    assert em.FRESH_MASS == pytest.approx(0.0567, abs=0.0005)  # USDA Large: 24 oz a dozen
    assert 1.075 <= em.specific_gravity(M.CELL["fresh"]) <= 1.085  # UF/IFAS: most fresh eggs 1.075-1.085
    assert em.VOLUME == pytest.approx(52.85e-6, rel=0.01)


def test_fresh_lies_older_stands_floater_floats():
    fresh, older, floater = (M.CELL[k] for k in M.KINDS)
    assert em.specific_gravity(fresh) > 1.0 and em.rest_tilt(fresh) < 30.0
    assert em.specific_gravity(older) > 1.0 and em.rest_tilt(older) > 75.0
    assert em.specific_gravity(floater) < 1.0


def test_air_cells_match_usda_grades():
    assert M.CELL["fresh"] <= 0.125 * 0.0254 + 1e-9  # AA: 1/8 in or less
    assert M.CELL["older"] > 0.1875 * 0.0254  # B: over 3/16 in


def test_tilt_grows_with_the_air_cell():
    tilts = [em.rest_tilt(c) for c in (0.0, 0.003, 0.0048, 0.006, 0.008)]
    assert all(b >= a - 1e-9 for a, b in zip(tilts, tilts[1:]))


def test_the_air_cell_moves_the_center_of_mass_toward_the_narrow_end():
    assert em.com_x(0.011) < em.com_x(0.007) < em.com_x(0.003) < em.CENTROID_X


def _wrench(name, pos, R):
    q = Rotation.from_matrix(R).as_quat()
    b = fk._CFG["bodies"][name]
    return fk.wrench(b, pos, q, np.zeros(3), np.zeros(3), 0.0, fk.surface_z(), dt=0.0)


def test_floater_floats_blunt_end_up_with_a_little_showing():
    M.setup_water()
    n = M.FLOATER
    R = Rotation.from_euler("y", -90, degrees=True).as_matrix()  # egg x axis (blunt end) up
    m = M.em.mass(M.CELL["floater"])
    best = None
    for dz in np.arange(-0.06, 0.03, 0.0005):
        pos = np.array([M.BOWL_C[0], M.BOWL_C[1], fk.surface_z() + dz])
        F, _, _ = _wrench(n, pos, R)
        net = F[2] - m * fk.G
        if best is None or abs(net) < abs(best[1]):
            best = (dz, net, pos)
    top = best[2][2] + (em.EGG_L / 2 + M.egg_mid("floater"))
    assert 0.0005 < top - fk.surface_z() < 0.006
    # tipped 30 degrees, the water turns it back upright (blunt end up)
    R30 = Rotation.from_euler("y", -60, degrees=True).as_matrix()
    _, T, _ = _wrench(n, best[2], R30)
    axis = R30[:, 0]
    assert np.cross(axis, np.array([0, 0, 1.0])) @ T > 0  # torque rotates the axis toward vertical


def test_no_water_outside_the_bowl():
    M.setup_water()
    out = np.array([M.DISH_C[0], M.DISH_C[1], M.DISH_FLOOR_Z + 0.02])
    F, T, vol = _wrench(M.EGG_NAMES[0], out, np.eye(3))
    assert vol == 0.0 and not np.any(F)


def test_the_flared_bowl_holds_the_water_and_rises_with_the_eggs():
    M.setup_water()
    assert M.SURFACE0 - M.BOWL_FLOOR_Z == pytest.approx(0.075, abs=0.0005)
    z1 = fk.surface_for(3 * em.VOLUME)
    assert 0.002 < z1 - M.SURFACE0 < 0.006
    assert z1 < M.BOWL_RIM_Z - 0.02


def test_eggs_look_alike():
    meshes = [e.meshes for e in M.EGGS]
    looks = {m.look for ms in meshes for m in ms}
    assert len(looks) == 1


# -- the scorer --------------------------------------------------------------------------------------------------------
def st(pos, R=np.eye(3), vel=(0, 0, 0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=Rotation.from_matrix(R).as_quat(), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=p - 0.03, high=p + 0.03)


def egg_state(name, mid, up_deg=0.0, vel=(0, 0, 0)):
    """An egg with the middle of its length at `mid`, its axis tilted up_deg (blunt end up)."""
    R = Rotation.from_euler("y", -up_deg, degrees=True).as_matrix()
    pos = np.asarray(mid, float) - R @ np.array([M.egg_mid(M.EGG_KIND[name]), 0.0, 0.0])
    return st(pos, R, vel)


def towel(i):
    return (M.TOWEL_C[0], M.SLOTS_Y[i], M.TOWEL_TOP + em.EGG_B / 2)


def bowl_floor(name):
    i = M.EGG_NAMES.index(name)
    x, y = M.DROP_SPOTS[i]
    k = M.EGG_KIND[name]
    if k == "older":
        return (x, y, M.BOWL_FLOOR_Z + em.EGG_L / 2), 90.0
    return (x, y, M.BOWL_FLOOR_Z + em.EGG_B / 2), 15.0


def floating(name):
    x, y = M.DROP_SPOTS[M.EGG_NAMES.index(name)]
    return (x, y, M.SURFACE0 + 0.003 - em.EGG_L / 2), 90.0


def in_dish():
    return (M.DISH_C[0], M.DISH_C[1], M.DISH_FLOOR_Z + em.EGG_B / 2), 10.0


def run(where, steps=200, hands=None, drop=None, squeeze=None):
    """where(t) -> {egg: (mid, tilt)} each control step; hands(t) -> {gripper: (pos, opening)} (default: away);
    drop/squeeze: {egg: m/s or N} set in the watch's record. Returns the last score."""
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M.setup_water()
    fk.reset()
    M.WATCH.reset()
    for n, v in (drop or {}).items():
        M.WATCH.v_free[n] = v
    for n, v in (squeeze or {}).items():
        M.WATCH.crush[n] = v
    s = 0.0
    for k in range(steps):
        t = k * DT
        o = {n: egg_state(n, *where(t)[n]) for n in M.EGG_NAMES}
        hd = hands(t) if hands else {}
        for g in ("left_gripper", "right_gripper"):
            pos, op = hd.get(g, (AWAY[g], 1.0))
            o[g] = st(pos, DOWN)
            kit._OPENING[g] = op
        s = M.score(o)
    return s


def on_towel(t):
    return {n: (towel(i), 0.0) for i, n in enumerate(M.EGG_NAMES)}


def all_tested_then(final):
    """All three in the water (each at its spot: sunk or floating) for 2 s, then `final` from 3 s on."""
    def where(t):
        if t < 1.0:
            return on_towel(t)
        if t < 3.0:
            return {n: (floating(n) if n == M.FLOATER else bowl_floor(n)) for n in M.EGG_NAMES}
        return final(t)
    return where


def good_kept_floater(fl):
    return lambda t: {n: (fl if n == M.FLOATER else bowl_floor(n)) for n in M.EGG_NAMES}


def test_do_nothing_scores_0():
    assert run(on_towel) == 0.0


def test_full_job_scores_1():
    assert run(all_tested_then(good_kept_floater(in_dish()))) == 1.0


def test_tested_but_floater_left_in_the_bowl_is_025():
    assert run(all_tested_then(lambda t: {n: (floating(n) if n == M.FLOATER else bowl_floor(n)) for n in M.EGG_NAMES})) \
        == 0.25


def test_floater_out_but_not_in_the_dish_is_05():
    on_table = ((0.20, -0.25, M.TOP + em.EGG_B / 2), 5.0)
    assert run(all_tested_then(good_kept_floater(on_table))) == 0.5


def test_floater_in_the_dish_without_being_tested_is_0():
    def where(t):
        if t < 3.0:
            return {n: (bowl_floor(n) if n != M.FLOATER else (towel(1), 0.0)) for n in M.EGG_NAMES}
        return good_kept_floater(in_dish())(t)
    assert run(where) == 0.0


def test_floater_held_in_the_water_does_not_count_as_tested():
    fl = M.FLOATER

    def where(t):
        if t < 3.0:
            return {n: (floating(n) if n == fl else bowl_floor(n)) for n in M.EGG_NAMES}
        return good_kept_floater(in_dish())(t)

    def hands(t):  # the right hand's fingers on the floater the whole time it is in the water
        if t < 3.0:
            mid = np.asarray(floating(fl)[0]) + [0.0, 0.0, em.EGG_L / 2 - 0.02]
            return {"right_gripper": (tuple(mid), (em.EGG_B - 0.003) / 0.094)}
        return {}
    assert run(where, hands=hands) == 0.0


def test_all_three_thrown_out_is_025():
    assert run(all_tested_then(lambda t: {n: in_dish() if n == M.FLOATER else
                                      (((0.20, -0.30 + 0.06 * M.EGG_NAMES.index(n), M.TOP + em.EGG_B / 2)), 5.0)
                                      for n in M.EGG_NAMES})) == 0.25


def test_a_good_egg_in_the_dish_instead_is_025():
    good = next(n for n in M.EGG_NAMES if n != M.FLOATER)
    assert run(all_tested_then(lambda t: {n: (in_dish() if n == good else
                                          floating(n) if n == M.FLOATER else bowl_floor(n)) for n in M.EGG_NAMES})) \
        == 0.25


def test_dropped_egg_scores_0():
    assert run(all_tested_then(good_kept_floater(in_dish())), drop={M.EGG_NAMES[0]: 0.45}) == 0.0


def test_gentle_sink_is_not_a_drop():
    assert run(all_tested_then(good_kept_floater(in_dish())), drop={n: 0.30 for n in M.EGG_NAMES}) == 1.0


def test_squeezed_egg_scores_0():
    assert run(all_tested_then(good_kept_floater(in_dish())), squeeze={M.FLOATER: 40.0}) == 0.0


def test_gentle_grip_is_not_a_squeeze():
    assert run(all_tested_then(good_kept_floater(in_dish())), squeeze={M.FLOATER: 10.0}) == 1.0


def test_hand_still_on_the_floater_in_the_dish_is_not_1():
    def hands(t):
        mid = np.asarray(in_dish()[0]) + [0.0, 0.0, 0.0]
        return {"right_gripper": (tuple(mid), (em.EGG_B - 0.003) / 0.094)} if t > 3.0 else {}
    assert run(all_tested_then(good_kept_floater(in_dish())), hands=hands) < 1.0


def test_hand_hovering_close_over_an_egg_is_not_clear():
    def hands(t):  # closed fingertips about 5 mm over the dish egg's top (it leans 10 degrees, so its top is higher)
        top = in_dish()[0][2] + em.EGG_B / 2
        return {"right_gripper": ((M.DISH_C[0], M.DISH_C[1], top + 0.010 + 0.013), 0.0)} if t > 3.0 else {}
    assert run(all_tested_then(good_kept_floater(in_dish())), hands=hands) == 0.75


def test_eggs_still_moving_is_not_1():
    def where(t):
        w = dict(good_kept_floater(in_dish())(t))
        w[M.FLOATER] = (*in_dish(), (0.0, 0.0, -0.05))  # still settling into the dish
        return w
    assert run(all_tested_then(where)) == 0.75


def test_egg_off_the_table_scores_0():
    def where(t):
        w = all_tested_then(good_kept_floater(in_dish()))(t)
        if 2.0 < t < 2.5:
            w = dict(w)
            w[M.EGG_NAMES[0]] = ((0.2, 0.6, M.TOP - 0.3), 0.0)
        return w
    assert run(where) == 0.0


def test_good_egg_lifted_back_out_and_held_is_025():
    good = next(n for n in M.EGG_NAMES if n != M.FLOATER)

    def where(t):
        w = dict(good_kept_floater(in_dish())(t))
        w[good] = ((M.BOWL_C[0], M.BOWL_C[1], M.BOWL_RIM_Z + 0.05), 0.0)
        return w
    assert run(all_tested_then(where)) == 0.25


def test_stage_rules_read_the_eggs_not_flags():
    """An egg resting on the rim (out of the water, not in the dish) is neither kept nor set aside."""
    rim = ((M.BOWL_C[0] + M.BOWL_TOP_R, M.BOWL_C[1], M.BOWL_RIM_Z + em.EGG_B / 2), 0.0)
    assert run(all_tested_then(good_kept_floater(rim))) == 0.5


# -- the demo's plan ---------------------------------------------------------------------------------------------------
def test_demo_releases_each_egg_just_under_the_surface_with_dry_fingertips():
    zr = M.SURFACE0 - M.DIP + em.EGG_B / 2
    tips = zr - 0.013
    assert M.SURFACE0 - 0.006 < tips < M.SURFACE0 + 0.002
    assert M.SURFACE0 - M.DIP > M.BOWL_FLOOR_Z + 0.05  # the egg is let go well above the floor (it sinks)


def test_demo_grip_is_gentle():
    squeeze = abs(M.GRIP_Q) - (em.EGG_B / 2 - M.GRIP_SQUEEZE)
    assert squeeze == pytest.approx(0.0)
    assert 5000.0 * M.GRIP_SQUEEZE < M.F_CRUSH / 3  # yam_arena's finger drive: 5000 N/m


def test_towel_eggs_leave_room_for_the_fingers():
    gap = (M.SLOTS_Y[1] - M.SLOTS_Y[0]) - em.EGG_B / 2  # from one egg's middle to the next egg's side
    assert abs(M.TOWEL_Q) + 0.010 < gap  # an opened pad (with its thickness) clears the neighbour


# -- the drop and crush detector, driven the way the simulator drives it -----------------------------------------------
class FakeView:
    """Stands in for the PhysX contact view: the force vector each robot part puts on each egg, and each egg's net
    contact force. A number is a force along a default direction: a left fingertip pushes the egg toward -y and a right
    one toward +y (a pinch), any other link pushes it down; `env` (the glass, table or towel) pushes it up."""

    DIRS = {"tip_left": (0.0, -1.0, 0.0), "tip_right": (0.0, 1.0, 0.0)}

    def __init__(self):
        self.parts = [f"{s}/{k}" for s in ("left", "right") for k in ("link1", "link5", "gripper", "tip_left",
                                                                         "tip_right")]
        self.F = np.zeros((3, len(self.parts), 3))
        self.env = np.zeros((3, 3))

    def set(self, egg, env=0.0, **parts):
        self.F[:] = 0.0
        self.env[:] = 0.0
        i = M.EGG_NAMES.index(egg)
        for k, v in parts.items():
            name = k.replace("__", "/")
            d = np.asarray(self.DIRS.get(name.split("/")[1], (0.0, 0.0, -1.0)))
            self.F[i, self.parts.index(name)] = np.asarray(v, float) if np.ndim(v) else v * d
        self.env[i] = np.asarray(env, float) if np.ndim(env) else (0.0, 0.0, env)

    def forces(self):
        return self.F.copy(), self.F.sum(1) + self.env


class FakeBase:
    def __init__(self):
        self.scene = {n: type("A", (), {})() for n in M.EGG_NAMES}
        self.vel = {n: np.zeros(3) for n in M.EGG_NAMES}
        for n in M.EGG_NAMES:
            self.scene[n].data = type("D", (), {})()
            self.set(n, (0.0, 0.0, 0.0))

    def set(self, n, v):
        d = self.scene[n].data
        d.root_lin_vel_w = np.array([v], float)
        d.root_pos_w = np.array([[0.3, 0.2, 0.8]])
        d.root_quat_w = np.array([[0.0, 0.0, 0.0, 1.0]])


def drive(steps, base_step):
    """steps physics steps at 240 Hz; base_step(k, base, view) sets the egg velocities and contact forces."""
    fk.reset()
    M.WATCH.reset()
    view = FakeView()
    M.WATCH.view = view
    base = FakeBase()
    for k in range(steps):
        base_step(k, base, view)
        M._watch(base, np.zeros(3), 1 / 240)
        fk.state().t += 1 / 240
    return M.WATCH


def fall(h):
    """An egg let go h above a hard floor: its own velocity at each physics step until it stops."""
    def step(k, base, view):
        t = k / 240
        tf = math.sqrt(2 * h / 9.81)
        base.set(M.EGG_NAMES[0], (0.0, 0.0, -9.81 * t if t < tf else 0.0))
    return step


@pytest.mark.parametrize("h,flag", [(0.004, False), (0.007, False), (0.010, True), (0.012, True), (0.016, True),
                                    (0.030, True)])
@pytest.mark.parametrize("phase", [0.0, 0.25, 0.5, 0.75])
def test_a_let_fall_egg_is_judged_by_its_speed_before_impact(h, flag, phase):
    """The speed is read every physics step, so a fall is caught from 8.2 mm (the threshold) to 10 mm at the latest,
    whatever the fall's timing relative to the steps."""
    def step(k, base, view):
        t = (k - phase) / 240
        tf = math.sqrt(2 * h / 9.81)
        base.set(M.EGG_NAMES[0], (0.0, 0.0, -9.81 * t if 0.0 <= t < tf else 0.0))
    w = drive(60, step)
    assert (w.v_free[M.EGG_NAMES[0]] > M.V_DROP) == flag


def test_a_sideways_fling_counts():
    def step(k, base, view):
        base.set(M.EGG_NAMES[1], (1.5 if k < 20 else 0.0, 0.0, 0.0))
    assert drive(40, step).v_free[M.EGG_NAMES[1]] > M.V_DROP


def test_a_carried_egg_moved_fast_while_held_does_not_count():
    def step(k, base, view):
        base.set(M.EGG_NAMES[2], (0.0, 0.0, -0.6))  # lowered fast, both fingertips on it
        view.set(M.EGG_NAMES[2], right__tip_left=6.0, right__tip_right=6.0)
    w = drive(40, step)
    assert w.v_free[M.EGG_NAMES[2]] == 0.0 and w.crush[M.EGG_NAMES[2]] == 0.0


def test_held_by_one_fingertip_is_not_held():
    def step(k, base, view):
        base.set(M.EGG_NAMES[2], (0.0, 0.0, -0.6))
        view.set(M.EGG_NAMES[2], right__tip_left=6.0)
    assert drive(40, step).v_free[M.EGG_NAMES[2]] > M.V_DROP


def test_a_gentle_sink_in_water_is_not_a_drop():
    def step(k, base, view):
        base.set(M.EGG_NAMES[0], (0.0, 0.0, -0.30))
    assert drive(60, step).v_free[M.EGG_NAMES[0]] < M.V_DROP


@pytest.mark.parametrize("part", ["right__tip_left", "left__gripper", "left__link5", "right__link1"])
def test_a_sustained_hard_press_by_any_part_crushes(part):
    def step(k, base, view):
        view.set(M.EGG_NAMES[0], **{part: 45.0})
    assert drive(10, step).crush[M.EGG_NAMES[0]] > M.F_CRUSH


def test_a_pinch_at_the_crack_force_crushes_and_a_gentle_one_does_not():
    def hard(k, base, view):
        view.set(M.EGG_NAMES[1], left__tip_left=40.0, left__tip_right=40.0)

    def gentle(k, base, view):
        view.set(M.EGG_NAMES[1], left__tip_left=9.0, left__tip_right=9.0)
    assert drive(10, hard).crush[M.EGG_NAMES[1]] > M.F_CRUSH
    assert drive(10, gentle).crush[M.EGG_NAMES[1]] == 0.0


def test_a_one_step_knock_below_twice_the_rupture_force_is_not_a_crush():
    def step(k, base, view):
        view.set(M.EGG_NAMES[0], right__tip_left=70.0 if k == 3 else 2.0)
    assert drive(10, step).crush[M.EGG_NAMES[0]] == 0.0


def test_a_one_step_strike_over_twice_the_rupture_force_crushes():
    def step(k, base, view):
        view.set(M.EGG_NAMES[0], right__gripper=80.0 if k == 3 else 0.0)
    assert drive(10, step).crush[M.EGG_NAMES[0]] > M.F_CRUSH


def test_hammering_in_two_step_strikes_crushes():
    def step(k, base, view):
        view.set(M.EGG_NAMES[2], right__gripper=300.0 if k % 5 in (0, 1) else 0.0)
    assert drive(60, step).crush[M.EGG_NAMES[2]] > M.F_CRUSH


def test_a_press_split_over_links_is_summed():
    def three(k, base, view):  # closed hand: 30 N from each of three links, pressing it into the glass
        view.set(M.EGG_NAMES[1], env=90.0, right__tip_left=(0, 0, -30.0), right__tip_right=(0, 0, -30.0),
                 right__gripper=30.0)

    def tips(k, base, view):
        view.set(M.EGG_NAMES[1], env=72.0, right__tip_left=(0, 0, -36.0), right__tip_right=(0, 0, -36.0))
    assert drive(30, three).crush[M.EGG_NAMES[1]] > M.F_CRUSH
    assert drive(30, tips).crush[M.EGG_NAMES[1]] > M.F_CRUSH


def test_two_closed_hands_squeezing_from_opposite_sides_crush():
    def two(k, base, view):  # each side is a hand whose two links press 22 N each: 44 N per side, no one link over 38 N
        view.set(M.EGG_NAMES[1], left__gripper=(0, -22.0, 0), left__link5=(0, -22.0, 0),
                 right__gripper=(0, 22.0, 0), right__link5=(0, 22.0, 0))

    def two_light(k, base, view):  # the same with 15 N per link: 30 N per side
        view.set(M.EGG_NAMES[1], left__gripper=(0, -15.0, 0), left__link5=(0, -15.0, 0),
                 right__gripper=(0, 15.0, 0), right__link5=(0, 15.0, 0))
    assert drive(30, two).crush[M.EGG_NAMES[1]] > M.F_CRUSH
    assert drive(30, two_light).crush[M.EGG_NAMES[1]] == 0.0


def test_a_press_let_off_every_third_step_still_crushes():
    def step(k, base, view):
        view.set(M.EGG_NAMES[0], right__gripper=50.0 if k % 3 != 2 else 0.0)
    assert drive(30, step).crush[M.EGG_NAMES[0]] > M.F_CRUSH


def test_a_held_egg_already_on_the_floor_rammed_into_the_wall_counts():
    def step(k, base, view):  # pinched, resting on the bowl floor, dragged at 1.5 m/s, stops dead at the wall (step 20)
        base.set(M.EGG_NAMES[0], (min(1.5, 0.15 * (k - 4)), 0.0, 0.0) if 5 <= k < 20 else (0.0, 0.0, 0.0))  # 0.15/step
        view.set(M.EGG_NAMES[0], env=(-20.0 if k == 20 else 0.0, 0.0, 0.6), right__tip_left=6.0, right__tip_right=6.0)
    assert drive(40, step).v_free[M.EGG_NAMES[0]] > M.V_DROP


def test_a_held_egg_slid_slowly_along_the_floor_is_fine():
    def step(k, base, view):  # pinched, on the floor, moved off at 5 cm/s and stopped
        base.set(M.EGG_NAMES[0], (0.05, 0.0, 0.0) if 5 <= k < 30 else (0.0, 0.0, 0.0))
        view.set(M.EGG_NAMES[0], env=0.6, right__tip_left=6.0, right__tip_right=6.0)
    w = drive(40, step)
    assert w.v_free[M.EGG_NAMES[0]] < M.V_DROP and w.crush[M.EGG_NAMES[0]] == 0.0


def test_the_glass_pushing_back_counts_as_a_press():
    def step(k, base, view):  # a link presses 20 N and the egg is wedged so the glass pushes 45 N
        view.set(M.EGG_NAMES[1], env=(45.0, 0.0, 0.0), right__link5=(-20.0, 0.0, 0.0))
    assert drive(30, step).crush[M.EGG_NAMES[1]] > M.F_CRUSH


def test_closed_tips_resting_on_top_are_not_holding():
    def step(k, base, view):  # both tips push down on the egg's top while it is shoved along at 0.8 m/s
        base.set(M.EGG_NAMES[0], (0.8, 0.0, 0.0))
        view.set(M.EGG_NAMES[0], env=4.0, left__tip_left=(0, 0, -2.0), left__tip_right=(0, 0, -2.0))
    w = drive(30, step)
    assert not w.held[M.EGG_NAMES[0]] and w.v_free[M.EGG_NAMES[0]] > M.V_DROP


def test_barely_touching_pads_are_not_holding():
    def step(k, base, view):  # it slides down between pads touching at 0.06 N and hits the floor at 0.6 m/s
        base.set(M.EGG_NAMES[0], (0.0, 0.0, -0.025 * k) if k < 24 else (0.0, 0.0, 0.0))
        view.set(M.EGG_NAMES[0], left__tip_left=0.06, left__tip_right=0.06)
    assert drive(40, step).v_free[M.EGG_NAMES[0]] > M.V_DROP


def test_tips_of_two_hands_are_not_a_hold():
    def step(k, base, view):
        base.set(M.EGG_NAMES[0], (0.0, 0.0, -0.8))
        view.set(M.EGG_NAMES[0], left__tip_left=5.0, right__tip_right=5.0)
    assert drive(30, step).v_free[M.EGG_NAMES[0]] > M.V_DROP


def test_a_held_egg_slammed_onto_the_glass_counts():
    def step(k, base, view):  # pinched and carried down at 1.0 m/s; it hits the glass at step 20 and stops
        base.set(M.EGG_NAMES[0], (0.0, 0.0, -1.0) if k < 20 else (0.0, 0.0, 0.0))
        hit = M.em.FRESH_MASS * 1.0 * 240 if k == 20 else (0.6 if k > 20 else 0.0)
        view.set(M.EGG_NAMES[0], env=hit, right__tip_left=14.0 if k in (20, 21) else 6.0,
                 right__tip_right=14.0 if k in (20, 21) else 6.0)
    w = drive(40, step)
    assert w.v_free[M.EGG_NAMES[0]] > M.V_DROP


def test_a_held_egg_set_down_slowly_is_fine():
    def step(k, base, view):  # pinched, lowered at 3 cm/s onto the dish floor, then let go
        base.set(M.EGG_NAMES[0], (0.0, 0.0, -0.03) if k < 20 else (0.0, 0.0, 0.0))
        tips = 6.0 if k < 30 else 0.0
        view.set(M.EGG_NAMES[0], env=0.0 if k < 20 else 0.6, right__tip_left=tips, right__tip_right=tips)
    w = drive(40, step)
    assert w.v_free[M.EGG_NAMES[0]] < M.V_DROP and w.crush[M.EGG_NAMES[0]] == 0.0


def test_a_pinch_then_throw_counts():
    def step(k, base, view):
        if k < 20:
            base.set(M.EGG_NAMES[0], (0.0, 0.075 * k, 0.0))
            view.set(M.EGG_NAMES[0], right__tip_left=6.0, right__tip_right=6.0)
        else:
            base.set(M.EGG_NAMES[0], (0.0, 1.5, 0.0) if k < 40 else (0.0, 0.0, 0.0))
            view.set(M.EGG_NAMES[0])
    assert drive(60, step).v_free[M.EGG_NAMES[0]] > M.V_DROP


def test_forces_on_one_egg_do_not_count_on_its_neighbour():
    def step(k, base, view):
        view.set(M.EGG_NAMES[0], right__tip_left=50.0, right__tip_right=50.0)
    w = drive(10, step)
    assert w.crush[M.EGG_NAMES[1]] == 0.0 and w.crush[M.EGG_NAMES[2]] == 0.0


def test_the_watch_restarts_with_each_trial():
    w = drive(40, fall(0.03))
    assert w.v_free[M.EGG_NAMES[0]] > M.V_DROP
    fk.reset()
    M.WATCH.t_last = 5.0
    M._watch(FakeBase(), np.zeros(3), 1 / 240)
    assert M.WATCH.v_free[M.EGG_NAMES[0]] == 0.0


def test_set_down_opening_fits_inside_the_ramekin():
    assert abs(M.SET_Q) + M.PAD_HALF < M.DISH_R0


if __name__ == "__main__":
    import sys

    code = pytest.main(["-q", "-p", "no:cacheprovider", __file__])
    print("all pass" if code == 0 else f"FAILED ({code})")
    sys.exit(int(code))


# -- the scripted demo is a fixed plan: it reads no simulator state -------------------------------------------------
_STATE_READS = ("objects", "surface_z", "EGG_KIND", "egg_mid_world", "egg_axis", "egg_low_high", "in_bowl",
                "WATCH", "_LINKS", "root_pos", "quat")


def test_the_demo_plans_read_no_simulator_state_in_their_source():
    import inspect
    for f in (M.demo, M._left_plan, M._right_plan):
        src = inspect.getsource(f)
        assert not [w for w in _STATE_READS if w in src], f.__name__
    assert "demo" not in inspect.getsource(M._update).lower()  # the scorer's per-step hook doesn't steer the demo


def test_the_demo_plans_read_no_simulator_state_when_run(monkeypatch):
    """Every state accessor raises; the plans are still built and every waypoint target can be taken."""
    def boom(*a, **k):
        raise AssertionError("the demo read simulator state")
    for name in ("egg_mid_world", "egg_axis", "egg_low_high", "in_bowl", "_update", "score", "success"):
        monkeypatch.setattr(M, name, boom)
    monkeypatch.setattr(fk, "surface_z", boom)
    monkeypatch.setattr(fk, "state", boom)
    d = M.demo()
    for side in ("left", "right"):
        for w in d[side]:
            assert all(math.isfinite(float(v)) for v in getattr(w, "pos", (0.0, 0.0, 0.0)))
    assert d == M.demo() or [vars(w) for w in d["right"]] == [vars(w) for w in M.demo()["right"]]

