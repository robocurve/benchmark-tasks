"""Board field limits and identity of the recorded instruction/scoring."""
import json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def test_submission_card_fields_and_limits():
    card=json.loads((ROOT/'task.json').read_text())
    for field in ('title','category','instruction','setup','scoring','run_limit','fidelity','scores'):
        assert isinstance(card[field],str) and card[field].strip()
    assert len(card['title'])<=120
    assert len(card['setup'])<=4000
    assert len(card['fidelity'])<=500
    if 'media' in card:assert re.fullmatch(r'M-[A-Z0-9]+',card['media'])
def test_recorded_instruction_score_and_reset_are_preserved():
    card=json.loads((ROOT/'task.json').read_text())
    full=json.loads((ROOT/'REFERENCE-SPEC.json').read_text())
    for key in ('instruction','scoring','run_limit','reset_notes','scores','context'):
        assert card[key]==full[key]
    assert 'REFERENCE-SPEC.json' in card['fidelity']
