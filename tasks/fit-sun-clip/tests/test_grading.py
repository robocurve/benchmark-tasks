import json
from pathlib import Path
import pytest

from task_impl.grading import Grade
from task_impl.scene import load_fixture


CASES = json.loads(Path(__file__).with_name("cases.json").read_text(encoding="utf-8"))


@pytest.mark.parametrize("case", CASES, ids=[case["label"] for case in CASES])
def test_outcome(case):
    fixture = load_fixture()
    metrics = {m["name"] for m in fixture["metrics"]}
    grade = Grade(fixture)
    for phase in case["phases"]:
        assert set(phase["measurements"]) == metrics
        for _ in range(phase["ticks"]):
            final = grade.update(phase["measurements"])
    assert case["min"] <= final["score"] <= case["max"], final
