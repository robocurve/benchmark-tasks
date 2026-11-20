"""task.json, README, TECHNICAL.md, validation.json and the code say the same thing. Standard library only."""

import ast
import hashlib
import json
import math
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_ukulele_tuning import fixed_scene as fixture  # noqa: E402
from rdb_ukulele_tuning import grading, tuning  # noqa: E402
from rdb_ukulele_tuning import layout as L  # noqa: E402

TASK = json.loads((ROOT / "task.json").read_text())
REFERENCE = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text()
TECHNICAL = (ROOT / "TECHNICAL.md").read_text()
VALIDATION = json.loads((ROOT / "validation.json").read_text())
PACKAGE = ROOT / "rdb_ukulele_tuning"


def mm(value):
    """Metres to whole millimetres, halves rounded up (as written in task.json)."""
    return str(math.floor(round(1000 * value, 6) + .5))


def station_mm(point):
    return "(" + ", ".join(mm(v) for v in fixture.to_station(point)) + ")"


def hashed_files():
    """run.py's HASHED table, read without importing run.py (it needs Inspect Robots)."""
    tree = ast.parse((PACKAGE / "run.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "HASHED" for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError("run.py has no HASHED table")


class Card(unittest.TestCase):
    def test_fields_and_board_limits(self):
        for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                    "reset_notes", "fidelity", "scores"):
            self.assertTrue(TASK[key], key)
        self.assertEqual(TASK["category"], "Societal relatability")
        for key, limit in (("title", 120), ("setup", 4000), ("context", 1000), ("fidelity", 500), ("scores", 300)):
            self.assertLessEqual(len(TASK[key]), limit, key)
        self.assertEqual(TASK["title"], "Tune a ukulele's G string to pitch with a clip-on tuner")
        self.assertEqual(TASK["keywords"], ["bimanual", "ukulele", "tuning", "clip-on tuner", "geared tuning button",
                                            "closed-loop rotation"])

    def test_scores_field_is_the_recorded_runs(self):
        scripted = VALIDATION["runs"]["scripted"]
        dream = VALIDATION["runs"]["dream"]
        self.assertTrue(TASK["scores"].startswith(
            f"Dream-1 run: {dream['score']:.1f} ({dream['executed_steps']:,} steps; "), TASK["scores"])
        self.assertIn(f"demo: {scripted['score']:.1f} ({scripted['executed_steps']:,} steps", TASK["scores"])

    def test_reference_spec_is_the_same_card(self):
        self.assertEqual({k: v for k, v in TASK.items() if k != "media"},  # the board's upload tool adds a media ID to task.json
                         {k: v for k, v in REFERENCE.items() if k != "media"})

    def test_instruction_and_run_limit_match_the_code(self):
        self.assertEqual(TASK["instruction"], fixture.INSTRUCTION)
        self.assertEqual(fixture.MAX_STEPS, 2400)
        self.assertTrue(TASK["run_limit"].startswith(f"{fixture.MAX_STEPS:,} steps ({fixture.MAX_STEPS // 20} s at 20 Hz)"))
        self.assertEqual(fixture.CONTROL_HZ, grading.CONTROL_HZ)
        self.assertEqual(fixture.CONTROL_HZ, tuning.CONTROL_HZ)
        self.assertIn(f"{fixture.MAX_STEPS:,} actions at 20 Hz", README)

    def test_setup_numbers_match_the_code(self):
        setup = TASK["setup"]
        y0 = fixture.NUT_Y
        self.assertIn(f"bases at y = +{mm(-fixture.BASES['right'][1])} and y = -{mm(-fixture.BASES['right'][1])}", setup)
        self.assertIn(f"(satin mahogany, open-gear tuners with pearloid buttons, {mm(L.SCALE)} mm scale, {mm(L.LENGTH)} mm "
                      f"long, lower bout {mm(L.LOWER_BOUT)} mm, body {mm(L.BODY_DEPTH)} mm deep), Aquila Super Nylgut strings",
                      setup)
        self.assertIn(f"soundboard at x = {mm(fixture.SOUNDBOARD_X)} (back at x = {mm(fixture.SOUNDBOARD_X + L.BODY_DEPTH)}), "
                      f"the neck's centre line {mm(fixture.CENTRE_Z)} mm above the table, nut at y = {mm(y0)}, headstock "
                      f"tip at y = {mm(y0 + L.TIP_Y)}, tail at y = {mm(y0 + L.TAIL_Y)}", setup)
        self.assertIn(f"the G string is the top string, {mm(L.string_point('g', L.PLUCK_Y)[2] - L.TOP_Z)} mm in front "
                      "of the soundboard", setup)
        self.assertIn("is the one of the two buttons on that edge nearer the nut: a flat button "
                      f"{mm(L.BUTTON_WIDE)} mm across, {mm(L.BUTTON_LEN)} mm long and {mm(L.BUTTON_THICK)} mm "
                      f"thick, centred at {station_mm(L.button_centre('g'))}", setup)
        self.assertIn(f"The C string's button is beside it, centred at {station_mm(L.button_centre('c'))}", setup)
        self.assertIn(f"The soundhole, {mm(2 * L.SOUNDHOLE_R)} mm across, is centred at "
                      f"{station_mm((0., L.SOUNDHOLE_Y, L.TOP_Z))}; the G string crosses it at x = "
                      f"{mm(fixture.to_station(L.pluck_point())[0])}, {mm(fixture.to_station(L.pluck_point())[2])} mm above "
                      f"the table, and the body's upper edge is {mm(fixture.CENTRE_Z + L.body_half_width(L.PLUCK_Y))} mm "
                      "above the table there", setup)
        self.assertIn(f"{mm(L.REST_LEN)} mm long", setup)
        # each block's top is where the edge it holds rests (the simulated blocks stop 0.5 mm short of it)
        rib = fixture.CENTRE_Z - L.body_half_width(L.BODY_REST_Y)
        edge = fixture.CENTRE_Z - L.neck_width(L.NECK_REST_Y) / 2
        self.assertIn(f"widest point (y = {mm(y0 + L.BODY_REST_Y)}, top {mm(rib)} mm above the table), with "
                      f"{mm(L.REST_CHEEK[0])} mm cheeks rising {mm(L.REST_CHEEK[1])} mm up the soundboard and the back", setup)
        self.assertIn(f"the 3rd fret (y = {mm(y0 + L.NECK_REST_Y)}, top {mm(edge)} mm above the table)", setup)
        self.assertAlmostEqual(L.NECK_REST_Y, L.fret(3))
        self.assertAlmostEqual(L.body_half_width(L.BODY_REST_Y), L.LOWER_BOUT / 2)      # the lower bout's widest point
        self.assertIn(f"behind the bridge (y = {mm(y0 + L.STRAP_Y[0])} to {mm(y0 + L.STRAP_Y[1])})", setup)
        self.assertIn(f"One {mm(L.STRAP_W)} mm hook-and-loop strap", setup)
        self.assertIn(f"The G string is {-L.START_CENTS['g']:.0f} cents flat (F#4", setup)
        self.assertTrue(all(L.START_CENTS[s] == 0. for s in L.OTHERS))
        self.assertIn(f"tilted {L.TUNER_ELEV_DEG:.0f} degrees from vertical, its middle at {station_mm(L.on_screen(0., 0.))}",
                      setup)
        self.assertIn("in simulation it has lamps for F#, G and G# only", setup)
        self.assertEqual(L.TUNER_NOTES, ("F#", "G", "G#"))
        self.assertIn(f"needle swings {L.NEEDLE_MAX_DEG:.0f} degrees left or right for {L.NEEDLE_RANGE_CENTS:.0f} cents", setup)
        self.assertIn(f"green means within {L.IN_TUNE_CENTS:.0f} cents", setup)
        self.assertIn("When no string sounds the screen is blank (in simulation every lamp is off and the needle rests "
                      "at its left end)", setup)
        blank = tuning.blank()
        self.assertFalse(any(blank["lamps"].values()))
        self.assertEqual(blank["needle_deg"], -L.NEEDLE_MAX_DEG)
        self.assertIn("headstock to the robot's right", setup)
        self.assertIn("soundboard vertical and facing the robot", setup)
        self.assertIn("sticks straight up from the headstock's upper edge", setup)
        self.assertEqual(L.TUNED, "g")
        self.assertLess(abs(L.POST_Y["g"]), abs(L.POST_Y["c"]))         # the G button is the one nearer the nut
        self.assertEqual((L.SIDE["g"], L.SIDE["c"]), (1, 1))             # both on the upper edge

    def test_scoring_says_what_the_scorer_does(self):
        scoring = TASK["scoring"]
        full, clear = f"{grading.FULL_CENTS:.0f}", mm(grading.CLEAR_M)
        self.assertIn(f"Within {full} cents of G4: 1.0 if the trial video shows the robot plucking the G string after its "
                      f"last turn of the G button and, in the video's last frame, both grippers' centre points are at "
                      f"least {clear} mm from the ukulele and tuner; otherwise {grading.BANDS[0][1]}.", scoring)
        bands = "; ".join(("Over " + full + " and up to " if k == 0 else "up to ") + f"{limit:.0f}"
                          + (" cents off" if k == 0 else "") + f": {score:g}" for k, (limit, score) in enumerate(grading.BANDS))
        self.assertIn(bands + f"; over {grading.BANDS[-1][0]:.0f} either way: 0.", scoring)
        self.assertEqual(grading.BANDS, ((15., .75), (30., .5), (50., .25)))
        self.assertIn(f"the C, E or A string reads more than {grading.MOVED_CENTS:.0f} cents from its start", scoring)
        self.assertEqual(set(grading.OTHERS), {"c", "e", "a"})
        self.assertIn("In simulation the same bands are applied to each string's computed pitch at the last step", scoring)
        self.assertIn(f"the G button counts as turned after a pluck once it is more than the gear's "
                      f"{grading.CHECK_TURN_DEG:.0f}-degree free play from its angle at that pluck", scoring)
        self.assertEqual(grading.CHECK_TURN_DEG, L.FREE_PLAY_DEG)
        self.assertIn("the grippers' distance is measured to boxes around the ukulele's parts and the tuner", scoring)
        self.assertIn(f"In simulation the trial ends at the run limit or once 1.0 has held for "
                      f"{grading.END_TICKS // grading.CONTROL_HZ} s; on the bench it runs to the run limit",
                      scoring)
        self.assertIn("Possible scores: " + ", ".join(f"{s:g}" if s not in (0., 1.) else ("0" if s == 0 else "1.0")
                                                      for s in grading.SCORES), scoring)
        # the bands do not depend on the start: 0.25 either way within 50 cents
        for cents, score in ((-50., .25), (50., .25), (-50.01, 0.), (50.01, 0.), (-5., 1.), (5.01, .75)):
            self.assertEqual(grading.band(cents, True, True), score, cents)
        self.assertEqual(grading.band(0., False, True), .75)
        self.assertEqual(grading.band(0., True, False), .75)
        self.assertNotIn("sharper", scoring)
        self.assertNotIn("arms parked", scoring)

    def test_fidelity_numbers_match_the_code(self):
        fidelity = TASK["fidelity"]
        self.assertIn(f"an estimated {L.CENTS_PER_DEG} cents per degree", fidelity)
        self.assertIn(f"about {L.FREE_PLAY_DEG:.0f} degrees of gear free play", fidelity)
        self.assertIn(f"A pluck counts when the G string is pushed at least {mm(L.PLUCK_OUT)} mm across the strings over "
                      "the soundhole and springs back untouched", fidelity)
        self.assertIn(f"The tuner is a stand-in (F#, G, G# and green lamps, a needle) showing the computed pitch for "
                      f"{L.TUNER_ACTIVE_S:.0f} s after a pluck, else unlit with its needle at the left end", fidelity)
        self.assertEqual(L.NEEDLE_REST_DEG, -L.NEEDLE_MAX_DEG)
        self.assertIn("No sound is simulated", fidelity)
        self.assertNotIn("fingertip", fidelity)

    def test_context_matches_the_rules(self):
        context = TASK["context"]
        self.assertIn("(at the start F# reads centred and green)", context)
        start = tuning.reading(L.START_CENTS["g"])
        self.assertEqual((start["note"], start["needle_deg"], start["lamps"][L.IN_TUNE_LAMP[0]]), ("F#", 0., True))
        self.assertIn("the button that winds the G string, not the C string's beside it", context)
        self.assertEqual(L.SIDE["c"], L.SIDE["g"])
        self.assertNotIn("regrip", context)
        self.assertNotIn("more than a turn", context)

    def test_reset_notes(self):
        reset = TASK["reset_notes"]
        self.assertIn("approach it from below", reset)
        self.assertIn("Nothing is consumed", reset)
        self.assertIn(f"Bring the G string to F#4 ({-L.START_CENTS['g']:.0f} cents flat", reset)
        self.assertIn("Retune C, E and A", reset)

    def test_no_text_left_from_earlier_versions(self):
        texts = [("task.json", json.dumps(TASK)), ("README.md", README), ("TECHNICAL.md", TECHNICAL),
                 ("validation.json", json.dumps(VALIDATION))]
        for stale in ("A string has gone flat", "Tune it to A4", "the A button", "A string's tuning button",
                      "G#4 (100", "lying face up on", "lies on its back", "A string is the string nearest",
                      "a-string", "a_string", "die-cast", "Aquila Nylgut", "then let go of the ukulele",
                      "sharper than the start", "review rules", "edge farther from the nut"):
            for name, text in texts:
                self.assertFalse(stale in text, f"{name}: {stale}")
        # nothing a reviewer reads names another package
        readable = texts + [(path.name, path.read_text()) for path in sorted(PACKAGE.glob("*.py"))]
        readable += [(path.name, path.read_text()) for path in sorted((ROOT / "tools").glob("*.py"))]
        readable += [(path.name, path.read_text()) for path in sorted((ROOT / "tests").glob("*.py")) if path.name != "test_package.py"]
        for name, text in readable:
            self.assertFalse("Etch" in text, name)
            self.assertFalse("template" in text.lower(), name)
            self.assertEqual(re.findall(r"rdb_(?!ukulele_tuning)\w+", text), [], name)
        self.assertEqual(json.loads((ROOT / "task.json").read_text())["title"],
                         "Tune a ukulele's G string to pitch with a clip-on tuner")


class Documents(unittest.TestCase):
    def test_readme_has_the_run_commands_and_what_they_write(self):
        for text in ("uv pip install --no-deps .", "python -m rdb_ukulele_tuning.run scripted --out out/scripted",
                     "python -m rdb_ukulele_tuning.run dream --out out/dream", "seed 0", "zero object jitter",
                     "RESULT.json", "media/setup.png", "media/top_cam.mp4", "media/side_cam.mp4",
                     "media/grader-trace.jsonl", "Runtime per trial", "ukulele_tuning_score", "demo.mp4"):
            self.assertIn(text, README, text)

    def test_validation_records_the_scripted_and_dream_runs(self):
        scripted, dream = VALIDATION["runs"]["scripted"], VALIDATION["runs"]["dream"]
        self.assertEqual(scripted["score"], 1.)
        self.assertLess(scripted["executed_steps"], fixture.MAX_STEPS)
        self.assertEqual(scripted["step_limit"], fixture.MAX_STEPS)
        self.assertEqual(scripted["termination_reason"], "success")
        self.assertEqual(dream["status"], "run")
        self.assertEqual(dream["score"], 0.)
        self.assertEqual(dream["model"], "dreamlabs/dream-1-beta")
        self.assertEqual((dream["executed_steps"], dream["step_limit"]), (fixture.MAX_STEPS, fixture.MAX_STEPS))
        self.assertTrue(dream["code_hashes_match_recorded"])
        self.assertIn("Dream-1 (dreamlabs/dream-1-beta) run natively once", VALIDATION["status"])
        self.assertEqual(VALIDATION["max_steps"], fixture.MAX_STEPS)
        self.assertEqual((VALIDATION["seed"], VALIDATION["object_jitter"]), (0, 0))
        self.assertEqual((VALIDATION["control_hz"], VALIDATION["physics_hz"]), (fixture.CONTROL_HZ, fixture.PHYSICS_HZ))
        self.assertIn(f"{scripted['executed_steps']:,} steps", README)
        self.assertIn("Dream-1 (`dreamlabs/dream-1-beta`) scores 0.0", README)
        self.assertIn("Dream-1 has been run once and scores 0.0", TECHNICAL)

    def test_recorded_hashes_are_the_files_here(self):
        recorded = VALIDATION["recorded_hashes"]
        for key, name in hashed_files().items():
            self.assertEqual(recorded[key], hashlib.sha256((PACKAGE / name).read_bytes()).hexdigest(), name)
        self.assertEqual(recorded["asset_sha256"], hashlib.sha256(fixture.USD.read_bytes()).hexdigest())

    def test_every_estimate_says_how_to_check_it(self):
        self.assertGreaterEqual(TECHNICAL.count("estimate, measure on a real Kala KA-15S"), 10)
        lines = (PACKAGE / "layout.py").read_text().splitlines()

        def line(name):
            return next(l for l in lines if l.startswith(name + " =") or l.startswith(name + ","))

        for name in ("CENTS_PER_DEG", "FREE_PLAY_DEG", "BUTTON_DRAG", "TIGHTEN", "STRING_TENSION", "POST_X", "HEAD_T"):
            self.assertIn("[E]", line(name), name)
        for name in ("SCALE", "LENGTH", "BODY_LENGTH", "LOWER_BOUT", "UPPER_BOUT", "WAIST", "BODY_DEPTH", "NUT_W",
                     "NECK_W12"):                                         # Kala's published figures
            self.assertIn("[K]", line(name), name)
        self.assertIn("[P]", line("POST_Y"))                              # Kala's photos: which post is whose

    def test_pyproject_entry_points_and_pins(self):
        text = (ROOT / "pyproject.toml").read_text()
        self.assertIn('requires-python = ">=3.12,<3.13"', text)
        self.assertIn('dependencies = ["inspect-robots==1.0.0", "inspect-robots-yam==0.38.0", "yam-arena==0.1.0", '
                      '"numpy>=1.26", "scipy>=1.11", "pillow>=10", "imageio[ffmpeg]"]', text)
        for line in ('yam_ukulele_tuning = "rdb_ukulele_tuning.adapter:make_embodiment"',
                     'ukulele_tuning_scripted = "rdb_ukulele_tuning.adapter:make_scripted"',
                     'ukulele_tuning_score = "rdb_ukulele_tuning.grading:make_scorer"',
                     'rdb_ukulele_tuning = ["ukulele.usda"]'):
            self.assertIn(line, text)
        self.assertTrue((ROOT / "LICENSE").read_text().startswith("MIT License"))

    def test_text_does_not_identify_anyone(self):
        for text in (json.dumps(TASK), README, TECHNICAL, json.dumps(VALIDATION)):
            self.assertIsNone(re.search(r"/U" r"sers/|(?<!/en)/h" r"ome/|@[a-z]+\.(edu|com)\b", text))
            for word in ("JH" "U", "Hop" "kins", "T-" "75", "gary" "23ai", "lam" "bda-yam"):
                self.assertNotIn(word, text)


if __name__ == "__main__":
    unittest.main()
