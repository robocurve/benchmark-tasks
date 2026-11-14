"""Offline tests for layout.py, fitgeom.py, scoring.py and task.py: synthetic part states (mechkit AssemblyStates from the
task's own specs) and synthetic arm snapshots (rigstate.Arm), no simulator.
    python3 -m unittest discover -s tests -v
Every number here is a design value or the offline model's prediction; nothing was measured on the simulator."""

import json
import math
import os
import pathlib
import re
import sys
import unittest

sys.dont_write_bytecode = True

import numpy as np  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))


def _offline_dir():
    for p in HERE.parents:
        d = p / "tools" / "offline"
        if (d / "yam_stub" / "yam_arena").is_dir():
            return d
    return None


OFFLINE = _offline_dir()
if OFFLINE is not None:
    for _p in (OFFLINE, OFFLINE / "yam_stub"):
        if str(_p) not in sys.path:
            sys.path.insert(0, str(_p))

try:
    import fitgeom as F  # noqa: E402
    import layout as L  # noqa: E402
    import mechkit as mk  # noqa: E402
    import rigstate  # noqa: E402
    import scoring as S  # noqa: E402
    import task as T  # noqa: E402
    from yam_arena import demo as D  # noqa: E402
except ModuleNotFoundError as e:
    if e.name != "yam_arena" and not str(e.name).startswith("yam_arena."):
        raise
    raise unittest.SkipTest("yam_arena is not importable: run inside its environment or next to tools/offline")

HZ = 20.0
WIN = int(round(L.STILL_WINDOW_S * HZ))
KIT_DIR = HERE.parents[1] / "tools" / "kit"
A_Y, B_Y = L.HOLES["A"][1], L.HOLES["B"][1]
MM = 0.001
LOC = 0.0005  # the located pose: the bracket's centre 0.5 mm (half the 1.0 mm spacing error) toward the other hole


def yaw_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def pin_state(name, xy, z=L.PIN_SEAT_Z, yaw=0.0, tilt=0.0):
    q = mk.quat_from_ypr(yaw, 0.0, tilt)
    return mk.state_from_poses(T.PIN_SPECS[name], (xy[0], xy[1], L.TOP + z, *q), source="test")


def bracket_state(dx=0.0, dy=0.0, z=L.PAD_TOP, yaw=0.0, tilt=0.0, roll=0.0):
    q = mk.quat_from_ypr(math.pi / 2 + yaw if False else yaw, tilt, roll)
    return mk.state_from_poses(T.BRACKET_PART, (L.FIX_X + dx, L.FIX_Y + dy, L.TOP + z, *q), source="test")


def parts(round_hole="B", diamond_hole="A", diamond_yaw=0.0, round_name="round_pin_1", bracket=None, second_round=None):
    """A located set at rest (round pin in ``round_hole``, diamond in ``diamond_hole``) and the bracket seated over it
    unless ``bracket`` is given. The second round pin stays in its rack."""
    out = {n: pin_state(n, L.ROUND_START[n], z=L.RACK_H) for n in L.ROUND}
    out[L.DIAMOND] = pin_state(L.DIAMOND, L.DIAMOND_START, z=L.RACK_H, yaw=L.DIAMOND_START_YAW)
    if round_hole:
        out[round_name] = pin_state(round_name, L.HOLES[round_hole], yaw=0.3)
    if diamond_hole:
        out[L.DIAMOND] = pin_state(L.DIAMOND, L.HOLES[diamond_hole], yaw=diamond_yaw)
    if second_round:
        n2 = [n for n in L.ROUND if n != round_name][0]
        out[n2] = pin_state(n2, L.HOLES[second_round])
    sign = 1.0 if round_hole == "B" else -1.0  # the bracket's centre sits SPACING_ERROR/2 toward the other hole
    out[S.BRACKET] = bracket if bracket is not None else bracket_state(dy=sign * L.SPACING_ERROR / 2)
    return out


FAR = {"left": rigstate.Arm(np.array([0.1, 0.4, 0.95]), np.array([0.1, 0.4, 0.94]), 1.0),
       "right": rigstate.Arm(np.array([0.1, -0.4, 0.95]), np.array([0.1, -0.4, 0.94]), 1.0)}


def run(states, arms=FAR, steps=WIN + 3, tracker=None):
    tr = tracker or S.Tracker()
    res = None
    for _ in range(steps):
        res = tr.update(states, arms)
    return res


class Geometry(unittest.TestCase):
    A = L.HOLE_SPACING / 2

    def test_dimensions(self):
        self.assertAlmostEqual(L.HOLES["A"][1] - L.HOLES["B"][1], 0.100, places=9)
        self.assertAlmostEqual(L.BR_HOLE_SPACING, 0.1010, places=9)
        self.assertAlmostEqual(L.HOLE_D, 0.0124)
        self.assertAlmostEqual(L.BR_HOLE_D, 0.0164)
        self.assertAlmostEqual(2 * L.HEAD_R, 0.016)
        self.assertAlmostEqual(2 * L.SHANK_R, 0.012)
        self.assertAlmostEqual(2 * L.FLANGE_R, 0.022)
        self.assertAlmostEqual(L.SHANK_L, 0.018)
        self.assertAlmostEqual(L.HEAD_H, 0.012)
        self.assertAlmostEqual(sum(h for h, _ in L.HEAD_STEPS), L.HEAD_H)
        self.assertAlmostEqual(L.HEAD_STEPS[0][1], L.HEAD_R)
        self.assertAlmostEqual(L.FIX_LEN_X, 0.120)
        self.assertAlmostEqual(L.FIX_LEN_Y, 0.200)
        self.assertAlmostEqual(L.FIX_H, 0.020)
        self.assertAlmostEqual(L.HOLE_DEPTH - L.SHANK_L, 0.001)  # the shank's tip hangs 1 mm clear of the floor
        self.assertAlmostEqual(L.BR_T, 0.008)
        self.assertGreater(L.PAD_H, L.FLANGE_T)  # the bracket clears the flanges
        self.assertAlmostEqual(L.ROUND_CLEAR, 0.0002)

    def test_masses(self):
        self.assertAlmostEqual(L.pin_mass(False), 0.0439, places=3)  # solid steel, 7.85 g/cm3
        self.assertAlmostEqual(L.pin_mass(True), 0.0341, places=3)
        self.assertAlmostEqual(L.BR_MASS, 0.185, places=3)
        self.assertLessEqual(L.BR_MASS, 0.4)
        self.assertLess(L.pin_mass(True), L.pin_mass(False))

    def test_specs_valid(self):
        for o in T.OBJECTS:
            o._check()
        names = [o.name for o in T.OBJECTS]
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(T.PIN_SPECS[L.DIAMOND].mass, L.pin_mass(True))
        self.assertIsNone(T.FIXTURE.mass)
        self.assertEqual(T.BRACKET_PART.mass, L.BR_MASS)

    def test_ring_free_region_matches_the_polygon(self):
        """Points just inside the 16 flats are clear of every stave; points just outside the flats are in material."""
        shapes = L.fixture_shapes()
        st = mk.state_from_poses(T.FIXTURE, (0, 0, 0, 0, 0, 0, 1), source="test")
        normals, r_in = L.hole_polygon(L.HOLE_R)
        zs = [0.005, 0.010, 0.015]
        for z in zs:
            for k, n in enumerate(normals):
                t = np.array([-n[1], n[0]])
                for s in (-0.0012, 0.0, 0.0012):  # along the flat
                    c = n * (r_in - 0.00005) + t * s
                    inside_clear = st.distance(np.array([[c[0], c[1] + L.HOLE_SPACING / 2, z]]))[0]
                    # the plate's own staves sit at r_in; a point 0.05 mm inside the flat is 0.05 mm from it
                    self.assertAlmostEqual(inside_clear, 0.00005, delta=2e-5)
                    o = n * (r_in + 0.0003) + t * s
                    self.assertEqual(st.distance(np.array([[o[0], o[1] + L.HOLE_SPACING / 2, z]]))[0], 0.0)

    def test_no_protrusion_into_the_circle(self):
        """The true circle of the hole (radius 6.2 mm, and 8.2 mm in the bracket) lies entirely in the free region: no
        shape of the plate or the bracket reaches inside it."""
        fx = mk.state_from_poses(T.FIXTURE, (0, 0, 0, 0, 0, 0, 1), source="test")
        br = mk.state_from_poses(T.BRACKET_PART, (0, 0, 0, 0, 0, 0, 1), source="test")
        a = np.linspace(0, 2 * math.pi, 720, endpoint=False)
        for z in np.linspace(0.0015, 0.0175, 7):
            ring = np.stack([L.HOLE_R * np.cos(a) * 0.999, L.HOLE_R * np.sin(a) * 0.999 + L.HOLE_SPACING / 2,
                             np.full_like(a, z)], axis=1)
            self.assertGreater(fx.distance(ring).min(), -1e-12)
            self.assertGreaterEqual(fx.distance(ring).min(), 0.0)
            self.assertTrue(np.all(fx.distance(ring) > 1e-6), f"fixture shape inside the hole's circle at z {z}")
        for z in np.linspace(0.0016, 0.0078, 6):
            ring = np.stack([L.BR_HOLE_R * np.cos(a) * 0.999, L.BR_HOLE_R * np.sin(a) * 0.999 + L.BR_HOLE_Y,
                             np.full_like(a, z)], axis=1)
            self.assertTrue(np.all(br.distance(ring) > 1e-6), f"bracket shape inside the hole's circle at z {z}")

    def test_funnels_are_clear_cones(self):
        fx = mk.state_from_poses(T.FIXTURE, (0, 0, 0, 0, 0, 0, 1), source="test")
        a = np.linspace(0, 2 * math.pi, 360, endpoint=False)
        for frac in (0.25, 0.5, 0.75):
            z = L.FIX_TOP - L.FUNNEL * (1 - frac)
            r = L.HOLE_R + L.FUNNEL * frac - 0.00005  # just inside the cone's surface
            pts = np.stack([r * np.cos(a), r * np.sin(a) + L.HOLE_SPACING / 2, np.full_like(a, z)], axis=1)
            self.assertTrue(np.all(fx.distance(pts) > 1e-7), "material inside the funnel")
            r2 = L.HOLE_R + L.FUNNEL * frac + 0.0006  # beyond the surface, but not at a polygon's corner gap
        self.assertEqual(len([s for s in L.fixture_shapes() if s.quat is not None]), 2 * 2 * L.SIDES - 2)  # all but the two unturned staves

    def test_home_pins_clear_the_plate(self):
        """A pin home in a hole, at any turn, touches nothing of the plate: its shapes are at least 0.19 mm from every
        plate shape (0.2 mm all round at the flats), and the flange rests on the plate."""
        fx = mk.state_from_poses(T.FIXTURE, (0, 0, 0, 0, 0, 0, 1), source="test")
        a = np.linspace(0, 2 * math.pi, 360, endpoint=False)
        for yaw in (0.0, 0.3, 1.0):
            for hole in ("A", "B"):
                cy = L.HOLES[hole][1] - L.FIX_Y
                for z, r in ((0.0021, L.SHANK_R - 0.0005), (0.0100, L.SHANK_R), (0.0179, L.SHANK_R)):
                    pts = np.stack([r * np.cos(a), r * np.sin(a) + cy, np.full_like(a, z)], axis=1)
                    self.assertGreaterEqual(fx.distance(pts).min(), 0.00019 - 1e-9)

    def test_two_round_pins_bind(self):
        v = F.best_pose([("round", self.A, 0.0), ("round", -self.A, 0.0)], span=0.0015)
        self.assertAlmostEqual(v[0], 0.3 * MM, delta=1e-6)  # 0.3 mm overlap at best, per hole

    def test_round_and_diamond_fit_in_a_window(self):
        for pins, sign in (([("round", self.A, 0.0), ("diamond", -self.A, 0.0)], -1), ([("diamond", self.A, 0.0), ("round", -self.A, 0.0)], 1)):
            w = F.feasible_window(pins, span=0.0015, turn_deg=0.8)
            self.assertIsNotNone(w)
            self.assertAlmostEqual(w["dx_mm"][1], 0.16, delta=0.03)
            self.assertAlmostEqual(w["turn_deg"][1], 0.18, delta=0.04)
            lo, hi = sorted((sign * 0.3, sign * 0.7))
            self.assertAlmostEqual(w["dy_mm"][0], lo, delta=0.03)
            self.assertAlmostEqual(w["dy_mm"][1], hi, delta=0.03)
            self.assertAlmostEqual(sum(w["dy_mm"]) / 2, sign * 0.5, delta=0.03)  # centred on the located pose

    def test_diamond_turned_a_quarter_binds(self):
        v = F.best_pose([("round", self.A, 0.0), ("diamond", -self.A, math.pi / 2)], span=0.0015)[0]
        self.assertGreater(v, 0.2 * MM)  # binds by about 0.28 mm, like two round pins (0.3)

    def test_diamond_fits_at_any_turn_the_scorer_allows(self):
        for deg in (-15, -10, 0, 10, 15, 30, 45):
            v = F.best_pose([("round", self.A, 0.0), ("diamond", -self.A, math.radians(deg))], span=0.0015)[0]
            self.assertLessEqual(v, 0.0, f"diamond turned {deg} deg")
        v80 = F.best_pose([("round", self.A, 0.0), ("diamond", -self.A, math.radians(80))], span=0.0015)[0]
        self.assertGreater(v80, 0.0)

    def test_one_round_pin_fits(self):
        self.assertLess(F.best_pose([("round", self.A, 0.0)])[0], 0.0)

    def test_shapes_agree_with_fitgeom(self):
        """The same answers from the actual shapes: a bracket state's distance to points on the pins' heads."""
        a = np.linspace(0, 2 * math.pi, 360, endpoint=False)

        def head_points(kind, xy, yaw):
            pts = []
            for z in np.linspace(L.PAD_TOP + 0.0005, L.PIN_SEAT_Z + L.FLANGE_T + L.HEAD_H - 0.0016, 5):
                zz = z
                if kind == "round":
                    for t in a:
                        pts.append([xy[0] + L.HEAD_R * math.cos(t), xy[1] + L.HEAD_R * math.sin(t), zz])
                else:
                    c, s = math.cos(yaw), math.sin(yaw)
                    corners = F.DIAMOND_V
                    for lx, ly in corners:
                        pts.append([xy[0] + c * lx - s * ly, xy[1] + s * lx + c * ly, zz])
            return np.array(pts)

        # world frame with the table top at 0: bracket at the located pose over the pair, seated
        def dmin(kinds_yaws, dy):
            br = mk.state_from_poses(T.BRACKET_PART, (L.FIX_X, L.FIX_Y + dy, L.PAD_TOP, 0, 0, 0, 1), source="test")
            pts = []
            for (kind, hole, yaw) in kinds_yaws:
                xy = (L.HOLES[hole][0], L.HOLES[hole][1])
                pts.append(head_points(kind, xy, yaw))
            return br.distance(np.vstack(pts)).min()

        good = dmin([("round", "B", 0.0), ("diamond", "A", 0.0)], LOC)
        self.assertGreater(good, 0.00008)  # about 0.1 mm to the nearest land, none touching
        # two round pins at the best pose (bracket centred): some head point lies inside a shape
        both = dmin([("round", "B", 0.0), ("round", "A", 0.0)], 0.0)
        self.assertEqual(both, 0.0)
        turned = dmin([("round", "B", 0.0), ("diamond", "A", math.pi / 2)], LOC)
        self.assertEqual(turned, 0.0)


class Tiers(unittest.TestCase):
    def test_full_set(self):
        r = run(parts())
        self.assertEqual(r.score, 1.0)
        self.assertTrue(r.success)
        self.assertEqual(r.details["located"], {"round": "round_pin_1", "round_hole": "B", "diamond_hole": "A"})

    def test_mirror_and_either_round_pin(self):
        for rh, dh, name in (("A", "B", "round_pin_1"), ("B", "A", "round_pin_2"), ("A", "B", "round_pin_2")):
            self.assertEqual(run(parts(round_hole=rh, diamond_hole=dh, round_name=name)).score, 1.0, (rh, dh, name))

    def test_located_set_without_the_bracket_is_half(self):
        p = parts(bracket=bracket_state(z=0.060))
        self.assertEqual(run(p).score, 0.5)
        p = parts(bracket=mk.state_from_poses(T.BRACKET_PART, (*L.BR_START, L.TOP + L.RAIL_H, 0, 0, 0, 1), source="test"))
        self.assertEqual(run(p).score, 0.5)

    def test_bracket_resting_on_the_heads_is_half(self):
        self.assertEqual(run(parts(bracket=bracket_state(dy=LOC, z=0.0336))).score, 0.5)

    def test_one_pin_is_a_quarter(self):
        self.assertEqual(run(parts(diamond_hole=None, bracket=bracket_state(z=0.060))).score, 0.25)
        self.assertEqual(run(parts(round_hole=None, bracket=bracket_state(z=0.060))).score, 0.25)
        # the bracket seated over one pin earns only the pin's quarter
        self.assertEqual(run(parts(diamond_hole=None)).score, 0.25)

    def test_two_round_pins_are_a_quarter(self):
        p = parts(round_hole="B", diamond_hole=None, second_round="A", bracket=bracket_state(z=0.0336))
        self.assertEqual(run(p).score, 0.25)
        self.assertEqual(run(parts(round_hole="B", diamond_hole=None, second_round="A")).score, 0.25)

    def test_diamond_turned_wrong_is_a_quarter(self):
        self.assertEqual(run(parts(diamond_yaw=math.pi / 2)).score, 0.25)
        self.assertEqual(run(parts(diamond_yaw=math.radians(45))).score, 0.25)

    def test_turn_tolerance_edge(self):
        self.assertEqual(run(parts(diamond_yaw=math.radians(14.9))).score, 1.0)
        self.assertEqual(run(parts(diamond_yaw=math.radians(-14.9))).score, 1.0)
        self.assertEqual(run(parts(diamond_yaw=math.radians(15.2))).score, 0.25)
        self.assertEqual(run(parts(diamond_yaw=math.pi)).score, 1.0)  # lands either way round: square to the line

    def test_nothing_home_is_zero(self):
        self.assertEqual(run(parts(round_hole=None, diamond_hole=None, bracket=bracket_state(z=0.060))).score, 0.0)

    def test_pins_in_the_racks_do_not_count(self):
        p = parts(round_hole=None, diamond_hole=None)
        self.assertEqual(run(p).score, 0.0)


class HomeEdges(unittest.TestCase):
    def check(self, **kw):
        p = parts(bracket=bracket_state(z=0.060), diamond_hole=None)
        p["round_pin_1"] = pin_state("round_pin_1", kw.pop("xy", L.HOLES["B"]), **kw)
        return run(p).score

    def test_flange_height(self):
        self.assertEqual(self.check(z=L.PIN_SEAT_Z + 0.0004), 0.25)
        self.assertEqual(self.check(z=L.PIN_SEAT_Z + 0.0006), 0.0)
        self.assertEqual(self.check(z=L.PIN_SEAT_Z + L.SHANK_L), 0.0)  # sitting on the plate's face, shank not in

    def test_tilt(self):
        self.assertEqual(self.check(tilt=math.radians(2.9)), 0.25)
        self.assertEqual(self.check(tilt=math.radians(3.2)), 0.0)

    def test_position_in_the_hole(self):
        xy = (L.HOLES["B"][0] + 0.0009, L.HOLES["B"][1])
        self.assertEqual(self.check(xy=xy), 0.25)
        xy = (L.HOLES["B"][0] + 0.0012, L.HOLES["B"][1])
        self.assertEqual(self.check(xy=xy), 0.0)

    def test_upside_down_is_not_home(self):
        self.assertEqual(self.check(tilt=math.pi), 0.0)


class SeatEdges(unittest.TestCase):
    def tier(self, **kw):
        base = dict(dy=L.SPACING_ERROR / 2)
        base.update(kw)
        return run(parts(bracket=bracket_state(**base))).score

    def test_position(self):
        self.assertEqual(self.tier(dy=LOC + 0.00029), 1.0)
        self.assertEqual(self.tier(dy=LOC + 0.00032), 0.5)
        self.assertEqual(self.tier(dy=LOC - 0.00032), 0.5)
        self.assertEqual(self.tier(dx=0.00029), 1.0)
        self.assertEqual(self.tier(dx=0.00032), 0.5)
        self.assertEqual(self.tier(dx=-0.00032), 0.5)

    def test_height(self):
        self.assertEqual(self.tier(z=L.PAD_TOP + 0.00029), 1.0)
        self.assertEqual(self.tier(z=L.PAD_TOP + 0.00032), 0.5)
        self.assertEqual(self.tier(z=L.PAD_TOP - 0.00032), 0.5)

    def test_turn_and_tilt(self):
        self.assertEqual(self.tier(yaw=math.radians(0.29)), 1.0 if 50.5e-3 * math.radians(0.29) < 0.0003 else 0.5)
        self.assertEqual(self.tier(yaw=math.radians(0.35)), 0.5)
        self.assertEqual(self.tier(yaw=math.pi), 1.0)  # turned half way round: the bracket is symmetric
        self.assertEqual(self.tier(tilt=math.radians(0.25)), 1.0)
        self.assertEqual(self.tier(tilt=math.radians(0.35)), 0.5)
        self.assertEqual(self.tier(roll=math.radians(0.35)), 0.5)
        self.assertEqual(self.tier(roll=math.pi), 0.5)  # upside down

    def test_the_wrong_hole_over_the_round_pin(self):
        """The bracket's centre at the fixture's centre (not at the located pose) is 0.5 mm off the round pin: past
        the 0.3 mm tolerance, so not seated."""
        self.assertEqual(self.tier(dy=0.0), 0.5)
        self.assertEqual(self.tier(dy=-0.0001), 0.5)
        self.assertEqual(self.tier(dy=LOC - 0.00029), 1.0)


class OffTable(unittest.TestCase):
    def test_each_part(self):
        for name in L.PIN_NAMES:
            p = parts()
            p[name] = pin_state(name, (0.30, 0.9), z=0.0)
            self.assertEqual(run(p).score, 0.0, name)
        p = parts(bracket=bracket_state(dx=0.0, dy=0.9, z=0.0))
        self.assertEqual(run(p).score, 0.0)
        p = parts(bracket=bracket_state(dx=-0.1, dy=0.3, z=-0.02))
        self.assertEqual(run(p).score, 0.0)
        p = parts()
        p["round_pin_2"] = pin_state("round_pin_2", (0.2, 0.2), z=-0.012)  # below the table by 12 mm
        self.assertEqual(run(p).score, 0.0)

    def test_pin_on_the_floor_is_fine(self):
        p = parts(bracket=bracket_state(z=0.060))
        p["round_pin_2"] = pin_state("round_pin_2", (0.2, 0.2), z=0.012, tilt=math.pi / 2)
        self.assertEqual(run(p).score, 0.5)


class ReleaseAndStill(unittest.TestCase):
    def test_gripper_near_the_bracket(self):
        b = np.array([L.FIX_X, L.FIX_Y, L.TOP + 0.045])
        near = {"left": rigstate.Arm(b + [0, 0.2, 0.05], b + [0, 0.2, 0.04], 1.0), "right": FAR["right"]}
        self.assertEqual(run(parts(), arms=near).score, 1.0)
        t = np.array([L.FIX_X, L.FIX_Y + 0.0, L.TOP + L.PAD_TOP + 0.003 + L.PAD_TIP if hasattr(L, "PAD_TIP") else L.TOP + 0.043])
        held = {"left": rigstate.Arm(t, t - [0, 0, 0.013], 0.3), "right": FAR["right"]}
        self.assertEqual(run(parts(), arms=held).score, 0.5)

    def test_gripper_beside_a_pin(self):
        hx, hy = L.HOLES["A"]
        t = np.array([hx, hy, L.TOP + 0.058])  # a hand just let go of the diamond, its pads round the head
        held = {"left": rigstate.Arm(t, t - [0, 0, 0.013], 1.0), "right": FAR["right"]}
        self.assertEqual(run(parts(), arms=held).score, 0.5)

    def test_unreadable_arms_never_release(self):
        self.assertEqual(run(parts(), arms=None).score, 0.5)
        bad = {"left": FAR["left"], "right": rigstate.Arm(np.array([np.nan, 0, 0]), np.array([0, 0, 0.0]), 1.0)}
        self.assertEqual(run(parts(), arms=bad).score, 0.5)

    def test_release_must_hold_the_whole_second(self):
        tr = S.Tracker()
        near = np.array([L.FIX_X, L.FIX_Y, L.TOP + 0.05])
        held = {"left": rigstate.Arm(near, near - [0, 0, 0.013], 0.2), "right": FAR["right"]}
        for _ in range(WIN):
            tr.update(parts(), held)
        for k in range(WIN - 1):
            self.assertEqual(tr.update(parts(), FAR).score, 0.5, k)
        self.assertEqual(tr.update(parts(), FAR).score, 1.0)

    def test_still_window(self):
        tr = S.Tracker()
        res = [tr.update(parts(), FAR).score for _ in range(WIN)]
        self.assertEqual(res[-1], 0.5)  # 20 readings are not yet a full second of history
        self.assertEqual(tr.update(parts(), FAR).score, 1.0)

    def test_moving_bracket(self):
        tr = S.Tracker()
        for _ in range(WIN + 3):
            tr.update(parts(), FAR)
        p = parts(bracket=bracket_state(dy=L.SPACING_ERROR / 2 + 0.0002))
        self.assertEqual(tr.update(p, FAR).score, 1.0)  # moved 0.2 mm: inside both the 0.3 mm stillness and the seat
        tr = S.Tracker()
        for _ in range(WIN):
            tr.update(parts(bracket=bracket_state(dy=LOC - 0.00025)), FAR)
        # seated at both ends (0.25 mm either side of the located pose) but 0.5 mm apart: not still
        self.assertEqual(tr.update(parts(bracket=bracket_state(dy=LOC + 0.00025)), FAR).score, 0.5)
        # drift 0.1 mm per step: moved more than 0.3 mm inside the second
        tr = S.Tracker()
        res = None
        for k in range(WIN + 3):
            res = tr.update(parts(bracket=bracket_state(dy=L.SPACING_ERROR / 2 + 0.0001 * (k % 3))), FAR)
        self.assertEqual(res.score, 1.0)
        tr = S.Tracker()
        for k in range(WIN + 3):
            res = tr.update(parts(bracket=bracket_state(dy=L.SPACING_ERROR / 2 + 0.00003 * k)), FAR)
        self.assertEqual(res.score, 0.5)

    def test_turning_bracket(self):
        tr = S.Tracker()
        res = None
        for k in range(WIN + 3):
            res = tr.update(parts(bracket=bracket_state(dy=L.SPACING_ERROR / 2, yaw=math.radians(0.02 * (k % 3)))), FAR)
        self.assertEqual(res.score, 1.0)


class Robustness(unittest.TestCase):
    def test_bad_states_score_zero_and_restart_the_windows(self):
        tr = S.Tracker()
        for _ in range(WIN + 3):
            tr.update(parts(), FAR)
        self.assertEqual(tr.update(parts(), FAR).score, 1.0)
        for bad in (None, {}, {"bracket": None}):
            self.assertEqual(tr.update(bad, FAR).score, 0.0)
        self.assertEqual(tr.update(parts(), FAR).score, 0.5)  # a fresh 1 s window is needed

    def test_nan_state(self):
        p = parts()
        b = p[S.BRACKET]
        origin = np.array([np.nan, 0.0, 0.8])
        p[S.BRACKET] = mk.AssemblyState(pos=b.pos, quat_xyzw=b.quat_xyzw, lin_vel=b.lin_vel, ang_vel=b.ang_vel, low=b.low,
                                        high=b.high, origin=origin, joints={}, joint_vel={}, links={}, source="test",
                                        spec=b.spec)
        self.assertEqual(S.Tracker().update(p, FAR).score, 0.0)

    def test_teleport_back_to_the_start_resets(self):
        tr = S.Tracker()
        for _ in range(WIN + 3):
            tr.update(parts(), FAR)
        start = {n: pin_state(n, L.ROUND_START[n], z=L.RACK_H) for n in L.ROUND}
        start[L.DIAMOND] = pin_state(L.DIAMOND, L.DIAMOND_START, z=L.RACK_H, yaw=L.DIAMOND_START_YAW)
        start[S.BRACKET] = mk.state_from_poses(T.BRACKET_PART, (*L.BR_START, L.TOP + L.RAIL_H, 0, 0, 0, 1), source="test")
        r = tr.update(start, FAR)
        self.assertEqual(tr.resets, 1)
        self.assertEqual(r.score, 0.0)

    def test_random_states_never_raise_and_stay_in_the_tiers(self):
        rng = np.random.default_rng(1)
        for _ in range(300):
            p = {}
            for n in L.PIN_NAMES:
                p[n] = pin_state(n, rng.uniform(0.1, 0.4, 2) * [1, 1] + [0, -0.2], z=rng.uniform(0, 0.05),
                                 yaw=rng.uniform(-3, 3), tilt=rng.uniform(0, 0.2))
            p[S.BRACKET] = bracket_state(dx=rng.uniform(-0.1, 0.1), dy=rng.uniform(-0.1, 0.1), z=rng.uniform(0.0, 0.06))
            self.assertIn(S.Tracker().update(p, FAR).score, (0.0, 0.25, 0.5, 1.0))


class TaskFile(unittest.TestCase):
    def test_objects(self):
        names = {o.name for o in T.OBJECTS}
        self.assertTrue({"fixture", "right_rack", "left_rack", "rail_1", "rail_2", *L.PIN_NAMES, S.BRACKET} <= names)
        self.assertEqual(len(T.OBJECTS), 3 + 2 + 3 + 1)

    def test_start_layout_scores_zero_and_is_resting(self):
        start = {n: T.PIN_SPECS[n].rest_state() for n in L.PIN_NAMES}
        start[S.BRACKET] = T.BRACKET_PART.rest_state()
        self.assertEqual(S.Tracker().update(start, FAR).score, 0.0)
        for n in L.ROUND:
            self.assertAlmostEqual(start[n].origin[2], L.TOP + L.RACK_H)
        self.assertAlmostEqual(start[L.DIAMOND].rotation()[1, 0], 1.0, places=6)  # wide axis along y: turned wrong

    def test_pins_hang_in_their_racks(self):
        """A rack's hole is a 14 mm square: the shank hangs in it with 1 mm each side, the flange rests on the top."""
        for rack, pin_xy in ((T.RIGHT_RACK, L.ROUND_START["round_pin_1"]), (T.RIGHT_RACK, L.ROUND_START["round_pin_2"]),
                             (T.LEFT_RACK, L.DIAMOND_START)):
            st = rack.rest_state()
            a = np.linspace(0, 2 * math.pi, 90, endpoint=False)
            for z in (0.006, 0.015, 0.021):
                pts = np.stack([pin_xy[0] + L.SHANK_R * np.cos(a), pin_xy[1] + L.SHANK_R * np.sin(a),
                                np.full_like(a, L.TOP + z)], axis=1)
                if np.linalg.norm(np.array(pin_xy) - np.array([st.origin[0], st.origin[1]])) < 0.04:
                    self.assertGreaterEqual(st.distance(pts).min(), 0.00095)
            self.assertAlmostEqual(L.RACK_H, 0.022)

    def test_weld_snap(self):
        pos, rot = T._snap(np.array([L.HOLES["A"][0] + 0.0004, L.HOLES["A"][1] - 0.0006, L.TOP + 0.0204]),
                           mk.quat_to_matrix(mk.quat_from_ypr(0.7, 0.01, -0.01)))
        self.assertAlmostEqual(pos[0], L.HOLES["A"][0])
        self.assertAlmostEqual(pos[1], L.HOLES["A"][1])
        self.assertAlmostEqual(pos[2], L.TOP + L.PIN_SEAT_Z)
        self.assertAlmostEqual(math.atan2(rot[1, 0], rot[0, 0]), 0.7, places=6)
        self.assertAlmostEqual(rot[2, 2], 1.0)
        self.assertEqual(set(T._LOCKS), set(L.PIN_NAMES))
        for n, w in T._LOCKS.items():
            self.assertTrue(w.when(parts(round_hole="B", diamond_hole="A", round_name=n)) in (True, False))
        r_home = parts()
        self.assertTrue(T._LOCKS["round_pin_1"].when(r_home))
        self.assertTrue(T._LOCKS[L.DIAMOND].when(r_home))
        self.assertFalse(T._LOCKS["round_pin_2"].when(r_home))

    def test_instruction_is_short_and_neutral(self):
        self.assertLessEqual(len(T.INSTRUCTION), 100)
        low = T.INSTRUCTION.lower()
        for word in ("diamond", "round", "relieved", "turn", "hole a", "hole b", "flat"):
            self.assertNotIn(word, low)

    def test_demo_plans(self):
        for v in ("",) + T.VARIANTS:
            os.environ["DEMO_VARIANT"] = v
            try:
                plan = T.demo()
            finally:
                os.environ.pop("DEMO_VARIANT", None)
            self.assertTrue(plan, v)
            for arm, wps in plan.items():
                for w in wps:
                    self.assertTrue(all(math.isfinite(c) for c in w.pos))
                    self.assertGreater(w.duration, 0.0)
                    self.assertGreater(w.pos[2], L.TOP + 0.02, (v, arm, w.label))  # pad tips stay off the table
                    self.assertLessEqual(w.finger, 0.0)
                    self.assertGreaterEqual(w.finger, -0.04695 - 1e-9)
            self.assertLessEqual(max(sum(w.duration for w in wps) for wps in plan.values()) * HZ + 60, 1200 + 60)
        with self.assertRaises(ValueError):
            T.plans("nonsense")

    def test_demo_one_arm_and_hold(self):
        os.environ["DEMO_IDLE_ARM"] = "left"
        try:
            self.assertEqual(set(T.demo()), {"right"})
        finally:
            os.environ.pop("DEMO_IDLE_ARM")
        os.environ["DEMO_HOLD_ONLY"] = "1"
        try:
            self.assertEqual(T.demo(), {})
        finally:
            os.environ.pop("DEMO_HOLD_ONLY")

    def test_demo_turns_the_diamond_a_quarter(self):
        left = T.plans("")["left"]
        yaws = [w.yaw for w in left if "carry it over hole A" in w.label]
        self.assertAlmostEqual(yaws[0] - T.YAW_DIAMOND_PICK, -math.pi / 2, delta=math.radians(1.0))  # a quarter turn less the left hand's measured 0.82 degree over-turn
        wrong = T.plans("diamond_turned")["left"]
        self.assertEqual([w.yaw for w in wrong if "carry it over hole A" in w.label], [T.YAW_DIAMOND_PICK])

    def test_demo_arms_take_turns_at_the_fixture(self):
        plan = T.plans("")
        def at_fixture(wps):
            t, spans, inside, start = 0.0, [], False, 0.0
            for w in wps:
                t += w.duration
                near = abs(w.pos[0] - L.FIX_X) < 0.02 and abs(w.pos[1]) < 0.07
                if near and not inside:
                    start, inside = t - w.duration, True
                if not near and inside:
                    spans.append((start, t - w.duration)); inside = False
            return spans
        left, right = at_fixture(plan["left"]), at_fixture(plan["right"])
        for a in left:
            for b in right:
                self.assertTrue(a[1] <= b[0] + 1e-9 or b[1] <= a[0] + 1e-9, (a, b))

    def test_grips(self):
        self.assertAlmostEqual(-2 * T.GRIP_PIN, 0.008)  # commanded 8 mm across a 16 mm head
        self.assertAlmostEqual(-2 * T.GRIP_BR, 0.052)
        self.assertGreaterEqual(-2 * T.GRIP_PIN, 0.008 - 1e-9)  # the gripper closes to 8 mm at least
        self.assertLessEqual(L.BR_LEN_X, 0.080)
        self.assertLessEqual(L.BR_MASS, 0.4)

    def test_release_heights(self):
        self.assertAlmostEqual(T.PIN_RELEASE_ORIGIN - L.PIN_SEAT_Z - L.SHANK_L, 0.003)  # the tip 3 mm over the mouth
        self.assertAlmostEqual(T.PIN_LET_GO_ORIGIN - L.PIN_SEAT_Z, 0.008)  # let go with 10 mm of the shank in the hole
        self.assertAlmostEqual(T.BR_RELEASE_UNDER, 0.036)  # the underside 1 mm over the pin heads
        self.assertGreater(L.TOP + T.BR_RELEASE_UNDER + 0.003 - L.TOP, L.PAD_TOP)  # the pad tips stay over the rest pads

    def test_bracket_aim_is_the_located_pose(self):
        self.assertAlmostEqual(T.ALIGN_DY["B"], LOC)
        self.assertAlmostEqual(T.ALIGN_DY["A"], -LOC)


class OfflineModel(unittest.TestCase):
    def setUp(self):  # the demo's bracket trim stands for the GPU arm's offset; these tests use exact poses
        self._trim, self._yaw = T.TRIM["bracket"], T.ARM_YAW["bracket"]
        T.TRIM["bracket"], T.ARM_YAW["bracket"] = (0.0, 0.0), 0.0

    def tearDown(self):
        T.TRIM["bracket"], T.ARM_YAW["bracket"] = self._trim, self._yaw

    def fits(self, dy, **kw):
        p = parts(bracket=bracket_state(dy=dy, z=0.030, **kw))
        return T.bracket_fits(S.read(p), {n: S.home_hole(S.read(p), n) for n in L.PIN_NAMES if S.home_hole(S.read(p), n)})

    def test_located_pose_fits(self):
        self.assertTrue(self.fits(LOC))
        self.assertTrue(self.fits(LOC + 0.00015))
        self.assertFalse(self.fits(LOC + 0.0005))
        self.assertFalse(self.fits(LOC, dx=0.0004))
        self.assertFalse(self.fits(LOC, yaw=math.radians(0.5)))

    def test_two_round_pins_do_not_fit(self):
        p = parts(round_hole="B", diamond_hole=None, second_round="A", bracket=bracket_state(z=0.030))
        r = S.read(p)
        homes = {n: S.home_hole(r, n) for n in L.ROUND}
        self.assertFalse(T.bracket_fits(r, homes))

    def test_offline_states_binds_and_seats(self):
        p = parts(round_hole="B", diamond_hole=None, second_round="A", bracket=bracket_state(z=0.030))
        out = T.offline_states(p)
        self.assertAlmostEqual(out[S.BRACKET].origin[2] - L.TOP, T.BIND_Z)
        p = parts(bracket=bracket_state(dy=LOC, z=0.030))
        self.assertAlmostEqual(T.offline_states(p)[S.BRACKET].origin[2] - L.TOP, L.PAD_TOP)
        p = parts(bracket=bracket_state(dy=LOC, z=0.080))  # still carried above the pins: untouched
        self.assertAlmostEqual(T.offline_states(p)[S.BRACKET].origin[2] - L.TOP, 0.080)


class Files(unittest.TestCase):
    def test_kit_files_are_byte_identical(self):
        for f in ("mechkit.py", "rigstate.py"):
            if (KIT_DIR / f).is_file():
                self.assertEqual((HERE / f).read_bytes(), (KIT_DIR / f).read_bytes(), f)

    def test_task_json_matches_layout(self):
        d = json.loads((HERE / "task.json").read_text())
        setup, scoring = d["setup"], d["scoring"]
        for token in ("12.4 mm", "100.0 mm", "16.4 mm", "101.0 mm", "120 mm", "200 mm", "20 mm thick", "185 g", "44 g",
                      "34 g", "12 mm shank 18 mm", "22 mm flange 3 mm", "16 mm head 12 mm", "x 30.0 cm", "y +5.0 cm",
                      "y -5.0 cm", "x 22.0 cm, y -23.5 cm and y -29.5 cm", "x 23.5 cm, y +25.0 cm", "x 13.5 cm, y +11.0 cm"):
            self.assertIn(token, setup)
        for token in ("0.3 mm", "0.3 degrees", "15 degrees", "25 mm", "0.5 mm", "25.0 mm", "1 s", "0.25", "0.5", "1.0"):
            self.assertIn(token, scoring)
        self.assertEqual(d["instruction"], T.INSTRUCTION)
        self.assertEqual(L.HOLES["A"], (0.300, 0.050))
        self.assertAlmostEqual(L.PAD_TOP, 0.025)
        self.assertAlmostEqual(d["fidelity"].count("0.02 mm"), 1)

    def test_text_has_no_identity_no_dashes_no_attribution(self):
        bad = re.compile(r"\u2014|co-authored|claude|anthropic|token|password|api[_ ]key|\w+@\w+\.", re.IGNORECASE)
        for f in ("task.json", "README.md", "task.py", "layout.py", "scoring.py", "fitgeom.py"):
            text = (HERE / f).read_text()
            for m in bad.finditer(text):
                line = text[max(0, m.start() - 40):m.end() + 40].replace("\n", " ")
                if m.group(0).lower() == "team":
                    continue
                self.fail(f"{f}: {m.group(0)!r} in ...{line}...")


if __name__ == "__main__":
    unittest.main()
