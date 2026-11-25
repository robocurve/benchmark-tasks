"""Regrade the seated roller and supported crossing from the tested source."""
from importlib.util import module_from_spec, spec_from_file_location
import json
from pathlib import Path
import sys


def verify(folder, limit, *, full=False):
    rows = [json.loads(row) for row in (folder / "trace.jsonl").read_text().splitlines()]
    log, = (folder / "rollout").glob("adhoc_*.json")
    run = json.loads(log.read_text())
    controls = len(rows)
    if not 0 < controls <= limit or run["stats"]["total_steps"] != controls or run["status"] != "success":
        raise ValueError("Inspect rollout and full trace must agree")
    source = folder / "scorer.py"
    if not source.is_file():
        source = folder.parent / "scorer.py"
    spec = spec_from_file_location("dishwasher_trial_scorer", source)
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
        scorer = module.Scorer()
        field_names = module.Reading.__dataclass_fields__
        first = {}
        for step, row in enumerate(rows, 1):
            value = scorer.update(module.Reading(**{field: row[field] for field in field_names}))
            if value != row["score"]:
                raise ValueError(f"The wheel score changed at control {step}")
            first.setdefault(str(value), step)
    finally:
        del sys.modules[spec.name]
    score = run["results"]["metrics"]["task_score"]
    if scorer.score != score:
        raise ValueError("Replay and Inspect scored different endpoints")
    if full:
        reset = json.loads((folder / "reset-audit.json").read_text())
        if score != 1. or not all(reset["checks"].values()):
            raise ValueError("Full supported crossing and repeatable reset are required")
    return dict(controls=controls, score=score, first_scores=first,
                crossing_m=scorer.crossing_dx, rim_arc_m=scorer.crossing_arc_m,
                released_controls=scorer.finish_ticks)
