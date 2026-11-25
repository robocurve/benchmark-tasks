import json,re
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def test_submission_card_matches_recorded_task():
    card=json.loads((ROOT/'task.json').read_text())
    reference=json.loads((ROOT/'REFERENCE-SPEC.json').read_text())
    for k in ('instruction','setup','scoring','run_limit','reset_notes','scores','category'):
        assert card[k]==reference[k]
    assert card['instruction'].strip()
    assert len(card['instruction']) <= 1000
    for k,limit in [('title',120),('setup',4000),('fidelity',500),('context',1000)]:
        assert 0<len(card[k])<=limit
    if 'media' in card: assert re.fullmatch('M-[A-Z0-9]+',card['media'])
    assert 'REFERENCE-SPEC.json' in card['fidelity']

    from rdb_filter_clamp.fixed_scene import INSTRUCTION
    from rdb_filter_clamp.grading import CONTROL_HZ, HOLD_TICKS, RELEASE_TICKS

    hold_s = HOLD_TICKS / CONTROL_HZ
    release_s = RELEASE_TICKS / CONTROL_HZ
    assert f'for {hold_s:g} s continuously' in card['scoring']
    assert f'For {release_s:g} s, keep' in card['scoring']
    assert f'for {hold_s:g} continuous second' in INSTRUCTION
    assert f'clear for {release_s:g} s.' in INSTRUCTION
    readme = (ROOT / 'README.md').read_text(encoding='utf-8')
    assert f'> {INSTRUCTION}\n' in readme
