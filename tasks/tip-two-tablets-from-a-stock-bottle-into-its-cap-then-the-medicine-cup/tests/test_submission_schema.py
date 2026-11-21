import json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def test_submission_card_matches_recorded_task():
    card=json.loads((ROOT/'task.json').read_text())
    reference=json.loads((ROOT/'REFERENCE-SPEC.json').read_text())
    for k in ('instruction','setup','scoring','run_limit','reset_notes','category'):
        assert card[k]==reference[k]
    for k,limit in [('title',120),('setup',4000),('fidelity',500),('context',1000),('scores',300)]:
        assert 0<len(card[k])<=limit
    if 'media' in card: assert re.fullmatch('M-[A-Z0-9]+',card['media'])
    assert 'REFERENCE-SPEC.json' in card['fidelity']

    if "pending" in card["scores"]:
        assert card["scores"] == reference["scores"] and "media" not in card
        return
    for field in ("Dream-1 run", "demo"):
        pattern=re.escape(field)+r":\s*([0-9.]+)"
        assert re.search(pattern,card["scores"])[1]==re.search(pattern,reference["scores"])[1]
