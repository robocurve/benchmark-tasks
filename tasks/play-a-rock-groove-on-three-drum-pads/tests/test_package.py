"""task.json, README, TECHNICAL.md, validation.json and the code say the same thing. Standard library only."""

import ast
import hashlib
import json
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from rdb_drum_groove import fixed_scene as F  # noqa: E402
from rdb_drum_groove import layout as L  # noqa: E402

TASK = json.loads((ROOT / "task.json").read_text())
REFERENCE = json.loads((ROOT / "REFERENCE-SPEC.json").read_text())
README = (ROOT / "README.md").read_text()
TECHNICAL = (ROOT / "TECHNICAL.md").read_text()
LICENSE = (ROOT / "LICENSE").read_text()
VALIDATION = json.loads((ROOT / "validation.json").read_text())
PACKAGE = ROOT / "rdb_drum_groove"
TEXT_SUFFIXES = {".py", ".md", ".txt", ".json", ".jsonl", ".toml", ".usda", ".csv", ".sh"}


def mm(v):
    return str(int(round(1000 * v)))


def signed(v):
    return ("+" if v > 0 else "-" if v < 0 else "") + mm(abs(v))


def hashed_files():
    """run.py's HASHED table, read without importing run.py (it needs Inspect Robots)."""
    tree = ast.parse((PACKAGE / "run.py").read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "HASHED" for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError("run.py has no HASHED table")


def package_files():
    for p in sorted(ROOT.rglob("*")):
        rel = p.relative_to(ROOT).parts
        if p.is_file() and rel[0] not in ("prototype",) and "__pycache__" not in rel and p.suffix in TEXT_SUFFIXES:
            yield p


class Card(unittest.TestCase):
    def test_fields_and_board_limits(self):
        for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                    "reset_notes", "fidelity", "scores"):
            self.assertTrue(TASK[key], key)
        self.assertEqual(TASK["category"], "Societal relatability")
        self.assertEqual(TASK["keywords"], ["drum kit", "electronic drums", "drumsticks", "groove", "accents", "timing",
                                            "bimanual", "impact"])
        for key, limit in (("title", 120), ("setup", 4000), ("context", 1000), ("fidelity", 500), ("scores", 300)):
            self.assertLessEqual(len(TASK[key]), limit, key)
        self.assertEqual(TASK["title"], "Play a slow 4/4 rock-ballad groove with two sticks on an electronic drum kit: "
                                        "accented hi-hat, snare backbeat, tom fill")
        self.assertTrue(all(v.isascii() for v in TASK.values() if isinstance(v, str)))

    def test_reference_spec_is_the_same_card(self):
        # the board's upload tool adds a media ID to task.json only
        self.assertEqual({k: v for k, v in TASK.items() if k != "media"}, {k: v for k, v in REFERENCE.items() if k != "media"})

    def test_instruction_and_run_limit_match_the_code(self):
        self.assertEqual(TASK["instruction"], F.INSTRUCTION)
        self.assertIn("then only the tom, on 4 and on 4 &", TASK["instruction"])
        self.assertIn("Let each stroke come straight off the pad", TASK["instruction"])
        self.assertTrue(TASK["run_limit"].startswith(f"{F.MAX_STEPS:,} steps ({F.MAX_STEPS // 20} s at 20 Hz)"))
        self.assertIn(f"{F.MAX_STEPS:,} actions at 20 Hz", README)

    def test_setup_numbers_match_the_code(self):
        s = TASK["setup"]
        for name, label in (("hihat", "centre"), ("snare", "head centre"), ("tom", "head centre")):
            (x, y), z, tilt = L.PAD_SPECS[name]
            self.assertIn(f"{label} ({mm(x)}, {signed(y)}), ", s)
            self.assertIn(f"{mm(z)} above the table", s)
        self.assertIn("tilted 5 degrees toward the robot", s)
        self.assertIn(f"its rim reaches y = {signed(F.pad_frame('tom')[0][1] - F.pad_outer_r('tom'))}", s)
        self.assertIn("hi-hat edge to snare rim 54, snare rim to tom rim 27", s)
        self.assertIn(f"({mm(L.STICK_LEN)} long, {L.STICK_R * 2000:.1f} across, about {L.STICK_MASS * 1000:.0f} g; "
                      "weigh it)", s)
        self.assertIn(f"a tape mark {mm(L.GRIP_FROM_BUTT)} mm from the butt", s)
        self.assertIn("finger pads closed on the stick's upper and lower sides", s)
        self.assertIn(f"{mm(F.START_TIP_LIFT)} above its pad", s)
        self.assertIn("y = -600 and y = +600", s)
        self.assertIn("Real trial only, not simulated: a phone on a stand", s)
        self.assertIn("a stick pivoting loosely at its tape mark, its tip let fall from 10 mm", s)
        self.assertIn("2 kg weight (for example a bag of sand) on its footboard", s)

    def test_scoring_numbers_match_the_code(self):
        s = TASK["scoring"]
        for text in ("at least 0.003 N s", "at 0.5 m/s or more", "(240 Hz)", "(2 mm, 2 steps)", "less than 40 ms apart",
                     "(60 to 90 bpm)", "within 50 ms of the note's fitted time", "within 40 ms of each other",
                     "8 pairs", "all 41 notes", "at most 25 ms", "71 to 79 bpm", "at least 25 MIDI velocity",
                     "at least 37 notes are played, 7 pairs together, the RMS error is at most 35 ms",
                     "at least 31 notes and 5 pairs", "at least 16 notes", "caps the score at 0.5",
                     "caps it at 0.75", "any contact over 2 N", "a held stick", "up to four hi-hat hits",
                     "Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0."):
            self.assertIn(text, s)
        self.assertEqual((L.HIT_IMPULSE, L.HIT_SPEED, L.MASK_S, L.WINDOW_S, L.PAIR_S, L.RMS_FULL_S, L.RMS_075_S),
                         (.003, .5, .04, .05, .04, .025, .035))
        self.assertEqual((L.FULL_BPM, L.BPM_RANGE, L.ACCENT_MARGIN, L.ARM_CONTACT_N, L.REST_S),
                         ((71., 79.), (60., 90.), 25., 2., .5))
        self.assertEqual(L.LEVELS, ((.75, 37, 7, .035, 3), (.5, 31, 5, None, 3), (.25, 16, 0, None, 3)))
        self.assertEqual((L.CAP_ARM_CONTACT, L.CAP_REST), (.5, .75))
        self.assertEqual(len(L.NOTES), 41)

    def test_fidelity_numbers_match_the_code(self):
        f = TASK["fidelity"]
        self.assertIn(f"restitution {L.PAD_RESTITUTION}", f)
        self.assertIn(f"friction only ({L.STICK_MU})", f)
        self.assertIn(f"{L.STICK_MASS * 1000:.0f} g", f)
        self.assertIn("40 ms mask", f)
        self.assertIn("Pads are fixed and rigid", f)

    def test_scores_field_is_the_recorded_runs(self):
        dream = VALIDATION["runs"]["dream"]
        self.assertTrue(TASK["scores"].startswith(
            f"Dream-1 run: {dream['score']:.1f} ({dream['executed_steps']:,} steps; "), TASK["scores"])
        what = TASK["scores"].split("steps; ", 1)[1].split("); demo: ", 1)[0]
        self.assertLessEqual(len(what), 70, what)
        self.assertNotIn("<", TASK["scores"])
        self.assertNotIn("not yet run", TASK["scores"])
        demo = VALIDATION["runs"]["scripted"]
        self.assertIn(f"demo: 1.0 ({demo['executed_steps']} steps; 41/41 notes", TASK["scores"])


class Package(unittest.TestCase):
    def test_no_words_for_the_judges_no_local_paths(self):
        # built from pieces so that this file does not contain what it looks for
        word = "re" + "view" + "ers?"
        bad = re.compile(word + r"\b|/" + "Users/" + r"\w|/" + "home/" + r"\w", re.IGNORECASE)
        for path in package_files():
            text = path.read_text(errors="replace")
            self.assertIsNone(bad.search(text), f"{path.relative_to(ROOT)}: {bad.search(text)}")

    def test_license_names_the_font_and_the_sounds(self):
        self.assertIn("MIT License", LICENSE)
        self.assertIn("DejaVu", LICENSE)
        self.assertIn("synthesized", LICENSE)
        self.assertIn("DejaVu Sans", README)

    def test_readme_states_the_start_pose(self):
        for arm in F.ARMS:
            self.assertIn(f"| {arm} | " + ", ".join(f"{v:.5f}" for v in F.START_Q[arm]) + " |", README)

    def test_readme_score_against_hits(self):
        self.assertIn("Score against hits: for each run, a plot of every logged hit against the written groove, with "
                      "the same numbers in scored-hits.csv.", README)
        runs = [VALIDATION["runs"]["scripted"], *VALIDATION["controls"].values()]
        for run in runs:
            self.assertIn(run["score_vs_hits"]["path"], README)


class Validation(unittest.TestCase):
    def test_scripted_full_score_repeated(self):
        demo = VALIDATION["runs"]["scripted"]
        self.assertEqual(demo["score"], 1.)
        self.assertEqual(demo["score_from_files"], 1.)
        self.assertLessEqual(demo["executed_steps"], demo["step_limit"])
        self.assertEqual(demo["step_limit"], F.MAX_STEPS)
        self.assertTrue(demo["repeat_trace_identical"])
        self.assertTrue(demo["repeat_hits_mid_identical"])
        self.assertEqual(demo["repeat_score"], 1.)

    def test_dream_run_recorded(self):
        dream = VALIDATION["runs"]["dream"]
        self.assertEqual(dream["status"], "run")
        self.assertEqual(dream["model"], "dreamlabs/dream-1-beta")
        self.assertEqual(dream["score"], 0.)
        self.assertEqual((dream["executed_steps"], dream["step_limit"]), (F.MAX_STEPS, F.MAX_STEPS))
        self.assertEqual(dream["termination_reason"], "max_steps")
        self.assertTrue(dream["code_hashes_match_recorded"])
        self.assertLessEqual(dream["action_requests"], dream["request_cap"])
        self.assertIn("Dream-1 (dreamlabs/dream-1-beta) run natively once on the final code", VALIDATION["status"])
        self.assertIn("Dream-1\n> (`dreamlabs/dream-1-beta`) scores 0.0", README)
        self.assertIn("Dream-1 has been run once on the final code and scores 0.0", TECHNICAL)
        self.assertNotIn("has not been run", README + TECHNICAL)

    def test_dream_run_files_match_where_present(self):
        dream = VALIDATION["runs"]["dream"]
        if not (ROOT / dream["result"]).is_file():
            self.skipTest("out/dream is not in this copy of the package")
        result = json.loads((ROOT / dream["result"]).read_text())
        self.assertEqual((result["score"], result["score_from_files"], result["executed_steps"]),
                         (dream["score"], dream["score_from_files"], dream["executed_steps"]))
        self.assertEqual(result["dream_action_requests"], dream["action_requests"])
        for key, value in VALIDATION["recorded_hashes"].items():
            self.assertEqual(result[key], value, key)
        for path, digest in ((dream["score_vs_hits"]["path"], dream["score_vs_hits"]["sha256"]),
                             (dream["videos"]["combined_with_sound"], dream["videos"]["combined_sha256"]),
                             (dream["hits_mid"], dream["hits_mid_sha256"])):
            self.assertEqual(hashlib.sha256((ROOT / path).read_bytes()).hexdigest(), digest, path)

    def test_recorded_runs_exist_and_match(self):
        if not (ROOT / VALIDATION["runs"]["scripted"]["result"]).is_file():
            self.skipTest("out/ is not in this copy of the package")
        runs = {"scripted": VALIDATION["runs"]["scripted"], **VALIDATION["controls"]}
        for name, run in runs.items():
            result = json.loads((ROOT / run["result"]).read_text())
            self.assertEqual(result["score"], run["score"], name)
            self.assertEqual(result["score"], result["score_from_files"], name)
            png = ROOT / run["score_vs_hits"]["path"]
            self.assertEqual(hashlib.sha256(png.read_bytes()).hexdigest(), run["score_vs_hits"]["sha256"], name)
            title = run["judged"]["title"]
            self.assertIn(f"| score {run['score']:.2f} |", title, name)
            self.assertIn(f"notes {result['judged']['H']}/41", title, name)
            self.assertEqual(hashlib.sha256((ROOT / run["hits_mid"]).read_bytes()).hexdigest(), run["hits_mid_sha256"])

    def test_controls_score_as_expected(self):
        for name, run in VALIDATION["controls"].items():
            if run["expected"] is not None:
                self.assertEqual(run["score"], run["expected"], name)
        variants = {run["variant"] for run in VALIDATION["controls"].values()}
        self.assertTrue({"flat", "one_arm", "air", "press", "flail"} <= variants)

    def test_hashes_are_these_files(self):
        recorded = VALIDATION["recorded_hashes"]
        for key, name in hashed_files().items():
            self.assertEqual(hashlib.sha256((PACKAGE / name).read_bytes()).hexdigest(), recorded[key], name)
        self.assertEqual(hashlib.sha256(F.USD.read_bytes()).hexdigest(), recorded["asset_sha256"])

    def test_sound_in_sync(self):
        sync = VALIDATION["runs"]["scripted"]["sound_sync"]
        self.assertTrue(sync["in_sync"])
        self.assertLessEqual(abs(sync["xcorr_offset_ms"]), 2.)


if __name__ == "__main__":
    unittest.main()
