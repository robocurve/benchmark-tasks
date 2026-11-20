"""Stimulus + reaction check of finished demo / negative runs (reads JSON only; any Python 3 with numpy not needed).

    python negcheck.py <out.json> <run_dir> [<run_dir> ...]

For each trial directory (a run dir, or each trial*/ inside a multi-trial run) it reads result.json and trace.jsonl
and checks that the stimulus the stage is meant to produce actually happened (from the measured bracket pose) and
that the final score is the expected level.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

EXPECT = {
    'demo': 1.0, 'neg-partial': .25, 'neg-unregistered': .5, 'neg-offset': .5, 'neg-held': .75, 'neg-jammed': 0., 'neg-beside': 0.,
    'idle': 0.,
}


def load_trace(d):
    return [json.loads(line) for line in open(d / 'trace.jsonl')]


def stimulus(stage, rows, res):
    """(done, evidence) from the measured trace, independent of the score."""
    m_end = rows[-1]['detail'].get('measure', {}) if rows else {}
    ever = lambda key: any(r['detail'].get('measure', {}).get(key) for r in rows)  # noqa: E731
    if stage == 'demo':
        return res['success'], dict(final=m_end, success=res['success'])
    if stage == 'neg-partial':
        return (m_end.get('on') and not m_end.get('home')), dict(px_mm=m_end.get('px_mm'), on=m_end.get('on'))
    if stage in ('neg-unregistered', 'neg-offset'):
        return (m_end.get('home') and m_end.get('hole_offset_mm', 0) > .15), dict(
            home=m_end.get('home'), register_deg=m_end.get('register_deg'), hole_offset_mm=m_end.get('hole_offset_mm'))
    if stage == 'neg-held':
        clear = rows[-1]['detail'].get('tip_clearance_mm')
        return (m_end.get('registered') and clear is not None and clear < 20.), dict(
            registered=m_end.get('registered'), tip_clearance_mm=clear)
    if stage == 'neg-jammed':
        # pushed toward the servo with the cheeks more than 1 mm off the horns' height when the cheek ends reach the
        # horn rims (px -22.2 mm), and never on both horns (it ends caught / cocked)
        meas = [r['detail'].get('measure', {}) for r in rows]
        at_rim = next((b for a, b in zip(meas, meas[1:]) if -40 < a.get('px_mm', -99) <= -22.2 < b.get('px_mm', -99) < 0),
                      None)
        tilt_end = m_end.get('tilt_deg')
        done = at_rim is not None and abs(at_rim.get('pz_mm', 0)) > 1.0 and not ever('on')
        return done, dict(pz_at_rim_mm=at_rim and at_rim.get('pz_mm'), ever_on=ever('on'), final_px_mm=m_end.get('px_mm'),
                          final_tilt_deg=tilt_end)
    if stage == 'neg-beside':
        return not ever('on'), dict(ever_on=ever('on'), final=m_end)
    if stage == 'idle':
        moved = max(abs(r['detail'].get('bracket_pos_mm', [0, 0, 0])[0] - rows[0]['detail'].get(
            'bracket_pos_mm', [0, 0, 0])[0]) for r in rows) if rows else 0
        return moved < .5, dict(bracket_moved_mm=round(moved, 3))
    return False, {}


def check(d):
    res = json.loads((d / 'result.json').read_text())
    stage = res['stage']
    rows = load_trace(d)
    done, ev = stimulus(stage, rows, res)
    want = EXPECT.get(stage)
    ok = bool(done) and want is not None and abs(res['task_score'] - want) < 1e-9 and not res.get('nonfinite')
    return dict(dir=str(d), stage=stage, task_score=res['task_score'], expected=want, stimulus_done=bool(done),
                evidence=ev, steps=res['steps'], stop=res.get('stop'), nonfinite=res.get('nonfinite'), passed=ok)


def main(out, dirs):
    rows = []
    for x in dirs:
        p = Path(x)
        trials = sorted(p.glob('trial*_*')) if not (p / 'result.json').exists() else [p]
        rows += [check(t) for t in trials]
    summary = dict(passed=all(r['passed'] for r in rows), n=len(rows), rows=rows)
    Path(out).write_text(json.dumps(summary, indent=1, default=float))
    for r in rows:
        print(('PASS' if r['passed'] else 'FAIL'), r['stage'], r['task_score'], r['expected'], r['evidence'])
    return summary['passed']


if __name__ == '__main__':
    sys.exit(0 if main(sys.argv[1], sys.argv[2:]) else 1)
