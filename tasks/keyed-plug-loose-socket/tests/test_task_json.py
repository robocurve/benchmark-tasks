"""Checks task.json against the board's rules. ``test_ready_to_submit`` fails until every draft marker is gone and
the measured scores and the media ID are in: it is the last gate before the pull request."""

import json
import os
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent
REQUIRED = ["title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores", "media"]
CATEGORIES = {"Impact on jobs", "Impact on AI timelines", "Safety and alignment", "Societal relevance", "Other"}
# Words that would identify the team, read from the environment so they stay out of the repository:
#   TEAM_WORDS="word1,word2" python -m unittest ...
IDENTIFYING = [w.strip().lower() for w in os.environ.get("TEAM_WORDS", "").split(",") if w.strip()]


def load():
    return json.loads((HERE / "task.json").read_text())


class TaskJson(unittest.TestCase):
    def test_valid_json_and_category(self):
        spec = load()
        self.assertIn(spec["category"], CATEGORIES)
        self.assertEqual(spec["category"], "Impact on AI timelines")

    def test_nothing_identifies_the_team(self):
        if not IDENTIFYING:
            self.skipTest("set TEAM_WORDS to a comma-separated list of words that would identify the team")
        text = json.dumps(load()).lower()
        for word in IDENTIFYING:
            self.assertNotIn(word, text, f"task.json mentions {word!r}: review is blind")

    def test_instruction_is_the_same_everywhere(self):
        instruction = load()["instruction"]
        self.assertIn(instruction, (HERE / "run.sh").read_text())
        self.assertIn(instruction, (HERE / "task.py").read_text())

    def test_ready_to_submit(self):
        spec = load()
        missing = [k for k in REQUIRED if not spec.get(k)]
        self.assertFalse(missing, f"required fields missing (media is filled in by tools/upload_media.py): {missing}")
        text = json.dumps(spec)
        for marker in ("DRAFT", "TBD", "TODO"):
            self.assertNotIn(marker, text, f"task.json still contains {marker}")


if __name__ == "__main__":
    unittest.main()
