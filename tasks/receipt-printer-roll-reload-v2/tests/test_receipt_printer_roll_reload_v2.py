import json
import math
import pathlib
import sys
import types
import unittest

HERE = pathlib.Path(__file__).resolve()
TASK = HERE.parents[1]
sys.path.insert(0, str(TASK))
# The demo plan imports yam_arena (numpy only).
_ROOT = HERE.parents[3].parent
_KNOWN = _ROOT / "robocurve-windows-transfer-2026-11-05" / "project" / "robocurve" / "tmp" / "yam-upstream"
for cand in [_KNOWN] if (_KNOWN / "yam_arena").is_dir() else _ROOT.rglob("yam-upstream"):
    if (cand / "yam_arena").is_dir():
        sys.path.insert(0, str(cand))
        break

import numpy as np  # noqa: E402

import geometry as g  # noqa: E402
from logic import (  # noqa: E402
    axis_off_y_deg,
    bay_open,
    cover_closed,
    evaluate_state,
    feeds,
    lamp_state,
    lid_open_deg,
    on_stand,
    roll_centre,
    seated,
    tail_points_back,
    tail_tip,
    tilt_deg,
)

try:
    import demo_plan  # noqa: E402
    import scoring  # noqa: E402

    HAVE_SIM_API = True
except Exception:  # pragma: no cover
    HAVE_SIM_API = False


def quat_z(deg):
    h = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(h), math.cos(h))


def quat_y(deg):
    h = math.radians(deg) / 2
    return (0.0, math.sin(h), 0.0, math.cos(h))


def quat_x(deg):
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


FLIP = quat_z(180.0)           # free end toward -x: correct
WRONG = quat_z(0.0)            # free end toward +x: wrong way round
SEAT_POS = (g.SEAT_XY[0], g.SEAT_XY[1], g.TABLE_TOP + g.SEAT_ORIGIN_Z)
START_POS = (g.ROLL_START_XY[0], g.ROLL_START_XY[1], g.TABLE_TOP + g.ROLL_START_ORIGIN_Z)
START_QUAT = quat_z(-90.0)
SPEED = 0.01


def full(lid_joint_val=0.0, open_run=25, lift=0.2, fixture=False, released=True, still=True, pos=SEAT_POS, quat=FLIP):
    return evaluate_state(pos, quat, lid_joint_val, open_run, lift, fixture, released, still)[0]


class Logic(unittest.TestCase):
    def test_full(self):
        self.assertEqual(full(), 1.0)

    def test_not_released_or_not_still(self):
        self.assertEqual(full(released=False), 0.75)
        self.assertEqual(full(still=False), 0.75)

    def test_cover_not_closed_is_half(self):
        # 10 degrees open: not closed -> 0.5
        open_10deg = math.radians(10.0)
        self.assertEqual(full(lid_joint_val=open_10deg), 0.5)
        # 4 degrees open: outside 3 degree tolerance -> 0.5
        open_4deg = math.radians(4.0)
        self.assertEqual(full(lid_joint_val=open_4deg), 0.5)
        # 2.5 degrees open: within 3 degree tolerance -> 1.0
        open_2_5deg = math.radians(2.5)
        self.assertEqual(full(lid_joint_val=open_2_5deg), 1.0)

    def test_cover_closed_and_bay_open_thresholds(self):
        self.assertTrue(cover_closed(0.0))
        self.assertTrue(cover_closed(math.radians(2.9)))
        self.assertFalse(cover_closed(math.radians(3.1)))
        self.assertAlmostEqual(lid_open_deg(math.radians(70.0)), 70.0, places=4)
        self.assertTrue(bay_open(math.radians(50.0)))
        self.assertTrue(bay_open(math.radians(70.0)))
        self.assertFalse(bay_open(math.radians(49.9)))

    def test_wrong_orientation_is_zero(self):
        # seated in position but free end runs toward rear wall: zero even with everything else perfect
        self.assertEqual(full(quat=WRONG), 0.0)
        self.assertEqual(full(quat=WRONG, lid_joint_val=-math.radians(10.0)), 0.0)
        self.assertTrue(tail_points_back(WRONG))
        self.assertFalse(tail_points_back(FLIP))

    def test_hovering_or_tilted_roll_is_not_seated(self):
        up = (SEAT_POS[0], SEAT_POS[1], SEAT_POS[2] + 0.005)
        down = (SEAT_POS[0], SEAT_POS[1], SEAT_POS[2] - 0.005)
        self.assertFalse(seated(up, FLIP))
        self.assertFalse(seated(down, FLIP))
        self.assertEqual(full(pos=up, lift=0.0, open_run=0), 0.0)
        self.assertFalse(seated(SEAT_POS, quat_x(7.0)))
        self.assertGreaterEqual(tilt_deg(quat_x(7.0)), 6.9)

    def test_seat_tolerances(self):
        def at(dx=0.0, dy=0.0, dz=0.0):
            return (SEAT_POS[0] + dx, SEAT_POS[1] + dy, SEAT_POS[2] + dz)

        self.assertTrue(seated(at(0.011), FLIP))
        self.assertFalse(seated(at(0.013), FLIP))
        self.assertTrue(seated(at(dy=0.007), FLIP))
        self.assertFalse(seated(at(dy=0.009), FLIP))
        self.assertTrue(seated(at(dz=0.0025), FLIP))
        self.assertFalse(seated(at(dz=0.0035), FLIP))
        self.assertTrue(seated(SEAT_POS, quat_z(180.0 + 4.0)))
        self.assertFalse(seated(SEAT_POS, quat_z(180.0 + 6.0)))
        self.assertFalse(seated(SEAT_POS, quat_z(90.0)))       # axis along x: across wrong way

    def test_feeds_needs_the_free_end_through_the_exit(self):
        self.assertTrue(feeds(SEAT_POS, FLIP))
        tip = tail_tip(SEAT_POS, FLIP)
        self.assertAlmostEqual(float(tip[0]), g.SEAT_XY[0] - g.TAIL_LEN_M, places=6)
        self.assertLess(float(tip[0]), g.FRONT_FACE_X - 0.02)
        # tail that does not reach 2 cm outside front face
        shifted = (SEAT_POS[0] + 0.050, SEAT_POS[1], SEAT_POS[2])
        self.assertFalse(feeds(shifted, FLIP))
        # tail lifted off chute
        self.assertFalse(feeds(SEAT_POS, quat_y(30.0)))
        # tail off to side
        self.assertFalse(feeds((SEAT_POS[0], SEAT_POS[1] + 0.03, SEAT_POS[2]), FLIP))

    def test_quarter_turn_the_wrong_way_ends_toward_the_rear(self):
        for deg, back in ((90.0, True), (-90.0, False)):
            q = quat_z(-90.0 + deg)
            self.assertEqual(tail_points_back(q), back)

    def test_start_pose_is_on_the_stand_and_scores_zero(self):
        self.assertTrue(on_stand(START_POS, START_QUAT))
        c = roll_centre(START_POS, START_QUAT)
        self.assertAlmostEqual(float(c[2]) - g.TABLE_TOP, g.ROLL_START_CENTRE_Z, places=6)
        self.assertEqual(evaluate_state(START_POS, START_QUAT, 0.0, 0, 0.0, False, True, True)[0], 0.0)
        self.assertAlmostEqual(axis_off_y_deg(START_QUAT), 90.0, places=4)

    def test_no_credit_for_lifting_and_putting_back(self):
        # cover held open for 1 s and the roll lifted 3 cm, then put back on the stand: nothing was loaded, so 0.0
        self.assertEqual(evaluate_state(START_POS, START_QUAT, 0.0, 20, 0.05, False, True, True)[0], 0.0)

    def test_fixture_moved_is_zero(self):
        self.assertEqual(full(fixture=True), 0.0)

    def test_lamp(self):
        self.assertEqual(lamp_state(START_POS, START_QUAT, 0.0), "red")
        self.assertEqual(lamp_state(SEAT_POS, FLIP, 0.0), "green")
        self.assertEqual(lamp_state(SEAT_POS, FLIP, -math.radians(10.0)), "red")  # cover open
        self.assertEqual(lamp_state(SEAT_POS, WRONG, 0.0), "amber")              # seated, wrong feed


class Geometry(unittest.TestCase):
    def test_roll_is_a_real_thermal_roll(self):
        self.assertEqual(g.ROLL_WIDTH_M, 0.058)
        self.assertEqual(2 * g.ROLL_RADIUS_M, 0.060)
        self.assertEqual(g.PAPER_ROLL_MASS_KG, 0.09)
        self.assertEqual(2 * g.ROLL_CORE_RADIUS_M, 0.025)
        self.assertGreater(0.094 - g.ROLL_WIDTH_M, 0.035)

    def test_printer_dimensions_and_bay(self):
        self.assertEqual(g.PRINTER_XY[0], 0.215)
        self.assertAlmostEqual(g.FRONT_FACE_X, 0.155)
        self.assertEqual(g.SEAT_XY[0], 0.210)
        self.assertAlmostEqual(g.BAY_X_RANGE[0] + (g.BAY_X_RANGE[1] - g.BAY_X_RANGE[0]) / 2, g.SEAT_XY[0], places=3)

    def test_bay_and_chocks_match_the_roll(self):
        self.assertAlmostEqual(2 * g.CRADLE_HALF_WIDTH_M, 0.068)
        self.assertAlmostEqual(g.CRADLE_HALF_WIDTH_M - g.ROLL_WIDTH_M / 2, 0.005)
        chock_inner = g.STAND_CHOCK.at[1] - g.STAND_CHOCK.size[1] / 2
        self.assertAlmostEqual(chock_inner - g.ROLL_RADIUS_M, 0.0085)
        self.assertLess(g.STAND_CHOCK.size[0], g.ROLL_WIDTH_M)
        self.assertGreater(2 * g.NOTCH_HALF_WIDTH_M - g.ROLL_WIDTH_M, 0.03)
        self.assertAlmostEqual((g.BAY_X_RANGE[1] - g.BAY_X_RANGE[0] - 2 * g.ROLL_RADIUS_M) / 2, 0.0075, places=3)

    def test_cover_hinge_and_spring(self):
        self.assertLessEqual(g.COVER_SPRING_N_M_PER_RAD, 10.0)                 # arm-held spring limit
        self.assertEqual(g.HINGE_AT, (0.06, 0.0, 0.07))                       # back top edge
        self.assertEqual(g.COVER_LOW_RAD, 0.0)
        self.assertGreater(g.COVER_HIGH_RAD, math.radians(80.0))

    def test_bay_open_threshold_clears_roll(self):
        self.assertEqual(g.BAY_OPEN_DEG, 50.0)
        self.assertEqual(g.COVER_CLOSED_TOL_DEG, 3.0)

    def test_seated_roll_fits_under_closed_cover(self):
        roll_top = g.SEAT_CENTRE_Z + g.ROLL_RADIUS_M
        self.assertAlmostEqual(g.WALL_TOP_Z - roll_top, 0.002, places=4)

    def test_all_mech_shapes_are_valid(self):
        for name, shapes in (("printer", g.PRINTER_SHAPES), ("cover", g.COVER_SHAPES), ("stand", g.STAND_SHAPES), ("roll", g.ROLL_SHAPES)):
            for s in shapes:
                self.assertTrue(all(v > 0 for v in s.size), (name, s))
                self.assertTrue(all(0.0 <= v <= 1.0 for v in s.color), (name, s))

    def test_stand_and_printer_do_not_overlap(self):
        stand_x1 = g.STAND_XY[0] + g.STAND_PLATE.size[0] / 2
        self.assertLess(stand_x1, g.FRONT_FACE_X - 0.003)


@unittest.skipUnless(HAVE_SIM_API, "yam_arena not importable")
class Demo(unittest.TestCase):
    TOP = g.TABLE_TOP

    @staticmethod
    def _track(plan, ready, dt=0.05):
        out, t, prev, prev_yaw = [], 0.0, np.array(ready, dtype=float), 0.0
        for w in plan:
            goal = np.array(w.pos, dtype=float)
            n = max(1, int(round(w.duration / dt)))
            for i in range(1, n + 1):
                u = i / n
                out.append((t + w.duration * u, *(prev + (goal - prev) * u), prev_yaw + (w.yaw - prev_yaw) * u))
            t += w.duration
            prev, prev_yaw = goal, w.yaw
        return out

    def _plans(self):
        plan = demo_plan.make_demo_plan()
        return plan, self._track(plan["left"], demo_plan.L_READY), self._track(plan["right"], demo_plan.R_READY)

    def test_every_waypoint_is_inside_the_reach_limits(self):
        plan, _, _ = self._plans()
        for side in ("left", "right"):
            for w in plan[side]:
                x, y, z = w.pos
                self.assertLessEqual(x, 0.30, (side, w.label))
                self.assertLessEqual(abs(y), 0.22, (side, w.label))
                self.assertTrue(math.isfinite(w.yaw))

    def test_lid_lift_waypoints_reach_at_least_60_degrees(self):
        lip_60 = demo_plan._lip_world(60.0)
        lip_70 = demo_plan._lip_world(70.0)
        self.assertLessEqual(lip_60[0], 0.30)
        self.assertLessEqual(lip_70[0], 0.30)
        self.assertAlmostEqual(lip_70[1], 0.13, places=3)

    def test_hands_stay_at_least_120_mm_apart_in_y(self):
        _, left, right = self._plans()
        n = min(len(left), len(right))
        worst = 9.0
        for i in range(n):
            ysep = left[i][2] - right[i][2]
            xmin = min(left[i][1], right[i][1])
            need = 0.19 if xmin <= 0.12 else 0.12
            self.assertGreaterEqual(ysep, need - 1e-9, (left[i][0], ysep))
            worst = min(worst, ysep)
        self.assertGreaterEqual(worst, 0.125)

    def test_roll_grip_and_carry_heights(self):
        self.assertAlmostEqual(demo_plan.GRASP[2] - self.TOP, g.ROLL_START_CENTRE_Z + 0.008, places=6)
        centre = demo_plan.GRASP[2] - self.TOP - 0.008
        self.assertGreaterEqual((demo_plan.GRASP[2] - self.TOP + 0.030) - (centre + g.ROLL_RADIUS_M), 0.0079)
        self.assertAlmostEqual(demo_plan.FINGER_GRIP, -0.027, places=6)

    def test_quarter_turn_direction(self):
        plan, _, _ = self._plans()
        yaws = [w.yaw for w in plan["right"]]
        self.assertAlmostEqual(max(yaws), 1.5708, places=4)
        self.assertAlmostEqual(yaws[-1], 0.0)

    def test_left_holds_lid_until_right_lifted_clear(self):
        plan = demo_plan.make_demo_plan()
        t = 0.0
        t_release_left = None
        for w in plan["left"]:
            if "release" in w.label or "let" in w.label:
                t_release_left = t
            t += w.duration
        t, t_lift_right = 0.0, None
        for w in plan["right"]:
            t += w.duration
            if "lift straight up" in w.label:
                t_lift_right = t
        self.assertIsNotNone(t_release_left)
        self.assertGreater(t_release_left, t_lift_right)

    def test_demo_fits_the_run_limit(self):
        plan = demo_plan.make_demo_plan()
        steps = max(sum(w.duration for w in v) for v in plan.values()) * 20
        self.assertLess(steps, 1500 * 0.5)

    def test_ablation_plans_leave_one_arm_at_ready(self):
        left_idle = demo_plan.make_demo_plan(ablation="left")
        right_idle = demo_plan.make_demo_plan(ablation="right")
        self.assertEqual(len(left_idle["left"]), 1)
        self.assertEqual(tuple(left_idle["left"][0].pos), demo_plan.L_READY)
        self.assertEqual(len(right_idle["right"]), 1)
        self.assertGreater(len(left_idle["right"]), 5)
        self.assertGreater(len(right_idle["left"]), 5)


class Ablation(unittest.TestCase):
    def test_left_idle_lid_blocks_the_bay(self):
        # cover never opens: roll cannot be seated
        self.assertEqual(evaluate_state(START_POS, START_QUAT, 0.0, 0, 0.0, False, True, True)[0], 0.0)

    def test_right_idle_nothing_loaded(self):
        # cover held open at 70 deg and nothing loaded: roll never leaves stand
        open_70 = -math.radians(70.0)
        self.assertEqual(evaluate_state(START_POS, START_QUAT, open_70, 400, 0.0, False, False, True)[0], 0.0)

    def test_neither_arm_alone_reaches_one(self):
        for open_run, lift in ((0, 0.2), (400, 0.0)):
            self.assertLess(evaluate_state(START_POS, START_QUAT, 0.0, open_run, lift, False, True, True)[0], 1.0)


@unittest.skipUnless(HAVE_SIM_API, "yam_arena not importable")
class Bookkeeping(unittest.TestCase):
    @staticmethod
    def _state(pos, joint_val=0.0):
        return types.SimpleNamespace(pos=np.array(pos, dtype=float), joints={g.COVER_NAME: joint_val})

    def setUp(self):
        scoring._reset_run()
        self.printer = (g.PRINTER_XY[0], g.PRINTER_XY[1], g.TABLE_TOP)
        self.stand = (g.STAND_XY[0], g.STAND_XY[1], g.TABLE_TOP)

    def _step(self, joint_val=0.0, roll=START_POS, printer=None, stand=None):
        scoring.advance(self._state(printer or self.printer, joint_val), self._state(roll), self._state(stand or self.stand))

    def test_open_run_counts_consecutive_steps(self):
        open_70 = math.radians(70.0)
        for _ in range(19):
            self._step(joint_val=open_70)
        self._step(joint_val=math.radians(30.0))   # broken run (under 50 deg)
        for _ in range(19):
            self._step(joint_val=open_70)
        self.assertEqual(scoring._RUN["open_run_max"], 19)
        self._step(joint_val=open_70)
        self.assertEqual(scoring._RUN["open_run_max"], 20)

    def test_lift_and_fixture_latches(self):
        self._step(roll=(START_POS[0], START_POS[1], START_POS[2] + 0.04))
        self._step()
        self.assertAlmostEqual(scoring._RUN["max_lift"], 0.04, places=6)
        self.assertFalse(scoring._RUN["fixture_moved"])
        self._step(printer=(self.printer[0] + 0.006, self.printer[1], self.printer[2]))
        self.assertTrue(scoring._RUN["fixture_moved"])
        self._step()
        self.assertTrue(scoring._RUN["fixture_moved"])

    def test_stillness_window(self):
        for i in range(25):
            self._step(joint_val=0.0, roll=SEAT_POS)
        self.assertTrue(scoring.window_still())
        for i in range(25):
            self._step(joint_val=0.002 * i, roll=SEAT_POS)
        self.assertFalse(scoring.window_still())

    def test_release_distance_rule(self):
        roll_pos = np.array(SEAT_POS)
        far = np.array([0.10, -0.14, g.TABLE_TOP + 0.12])
        near_roll = np.array([g.SEAT_XY[0], g.SEAT_XY[1], g.TABLE_TOP + 0.063 + 0.040])
        near_printer = np.array([g.FRONT_FACE_X - 0.02, 0.0, g.TABLE_TOP + 0.02])
        self.assertTrue(scoring.hands_clear([far, far * np.array([1, -1, 1])], roll_pos, FLIP, 0.0))
        self.assertFalse(scoring.hands_clear([far, near_roll], roll_pos, FLIP, 0.0))
        self.assertFalse(scoring.hands_clear([far, near_printer], roll_pos, FLIP, 0.0))


class Text(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tj = json.loads((TASK / "task.json").read_text(encoding="ascii"))
        cls.readme = (TASK / "README.md").read_text(encoding="ascii")

    def test_limits(self):
        t = self.tj
        self.assertLessEqual(len(t["title"]), 120)
        self.assertTrue(20 <= len(t["instruction"]) <= 540)
        self.assertLessEqual(len(t["context"]), 900)
        self.assertLessEqual(len(t["fidelity"]), 500)
        self.assertEqual(t["category"], "Societal relatability")
        self.assertIn("Epson TM-T20II", t["context"])
        self.assertIn("Epson TM-T20II", t["context"])
        self.assertIn("Possible scores:", t["scoring"])

    def test_readme_paragraphs_equal_task_json(self):
        paras = [p.strip() for p in self.readme.split("\n\n")]
        self.assertEqual(paras[0], "# " + self.tj["title"])
        for key in ("instruction", "context", "setup", "reset_notes", "scoring", "fidelity"):
            self.assertIn(self.tj[key].strip(), paras, key)

    def test_numbers_in_the_text_match_the_code(self):
        s = self.tj["setup"] + " " + self.tj["scoring"]
        for needle in ("(21.5, 0)", "58 mm wide, 60 mm across, 25 mm core", "90 g", "0.08 N m/rad", "50 degrees", "3 degrees", "6.8 cm", "(10, -14)",
                       "within 12 mm in x, 8 mm in y and 3 mm in z", "x <= 13.5 cm", "(21.0, 0, z 3.8)", "5 mm"):
            self.assertIn(needle, s, needle)
        self.assertAlmostEqual(g.PRINTER_XY[0], 0.215)
        self.assertAlmostEqual(g.SEAT_TOL_X_M, 0.012)
        self.assertAlmostEqual(g.SEAT_TOL_Y_M, 0.008)
        self.assertAlmostEqual(g.SEAT_TOL_Z_M, 0.003)
        self.assertAlmostEqual(g.FEED_TIP_MAX_X, 0.135)
        self.assertAlmostEqual(g.BAY_OPEN_DEG, 50.0)
        self.assertAlmostEqual(g.COVER_CLOSED_TOL_DEG, 3.0)
        self.assertAlmostEqual(g.FIXTURE_MOVE_M, 0.005)

    def test_ascii_and_run_sh_instruction(self):
        raw = (TASK / "task.json").read_bytes()
        raw.decode("ascii")
        run = (TASK / "run.sh").read_bytes()
        self.assertNotIn(b"\r\n", run)
        self.assertIn(self.tj["instruction"], run.decode("ascii"))
        for mode in ("settle", "demo", "ablation", "dream"):
            self.assertIn(mode + ")", run.decode("ascii"))


@unittest.skipUnless(HAVE_SIM_API, "yam_arena not importable")
class MechValidation(unittest.TestCase):
    def test_mech_objects_validate(self):
        import task

        self.assertEqual([o.name for o in task.OBJECTS], [g.PRINTER_NAME, g.ROLL_NAME, g.STAND_NAME])
        for o in task.OBJECTS:
            self.assertGreaterEqual(len(o.shapes), 1)
        printer = task.OBJECTS[0]
        self.assertEqual([j.name for j in printer.joints], [g.COVER_NAME])
        self.assertEqual(printer.joints[0].kind, "hinge")
        self.assertEqual(printer.joints[0].axis, "y")
        self.assertEqual(printer.joints[0].spring, 0.08)
        self.assertEqual(task.OBJECTS[1].mass, 0.09)

    def test_markers_are_created_once_and_only_moved(self):
        text = (TASK / "display.py").read_text(encoding="ascii")
        code = "\n".join(line for line in text.splitlines() if not line.strip().startswith("#"))
        self.assertNotIn(".set_visibility(", code)
        self.assertIn("-10.0", code)
        self.assertIn(".visualize(", code)


if __name__ == "__main__":
    unittest.main()
