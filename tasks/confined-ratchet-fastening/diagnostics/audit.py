"""Independently replay saved scores and challenge final seating with counterfactuals.

These are trace/scorer checks, not additional physical robot trials.
"""
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from ratchet_task.config import AXIS, TABLE, INITIAL_GAP
from ratchet_task.scoring import State, History
from ratchet_task.seating import seat_geometry


def state(row):
    f=row['feedback'];h=f['hardware']
    return State(captured=h['captured'],valid=h['valid'] and abs(h.get('pitch_error_m',0))<=.0002,
        turns=h['turns'],gap=h['seat_gap_m'],interface_torque=h['interface_torque'],
        socket_engaged=h['socket_engaged'],ratchet_cycles=h['ratchet_cycles'],
        fixture_displacement=float(np.linalg.norm(np.array(f['poses']['fixture'])-[AXIS[0],AXIS[1],TABLE])),
        seat_contact=h['seat_contact'],seat_contact_fraction=h['seat_contact_fraction'],
        seat_force_n=h['seat_force_mean_n'],seat_gap_min_m=h['seat_gap_min_m'],seat_gap_max_m=h['seat_gap_max_m'])


def audit(rows):
    history=History();errors=[];geometry_error=0.;first_full=None
    for i,row in enumerate(rows):
        score=history.update(state(row),.05)
        if score!=row['history']['score']:errors.append(i)
        if score==1. and first_full is None:first_full=i
        f=row['feedback'];h=f['hardware']
        geo=seat_geometry(np.array(f['poses']['bolt']),np.array(f['rotations']['bolt']),
            np.array(f['poses']['fixture']),np.array(f['rotations']['fixture']))
        geometry_error=max(geometry_error,max(abs(geo[k]-h[k]) for k in geo))
    failures={}
    cases={
        'old_0.196mm_gap':dict(gap=.0001960964,seat_gap_min_m=.0001960964,seat_gap_max_m=.0001960964),
        'missing_contact':dict(seat_contact=False),
        'intermittent_contact':dict(seat_contact_fraction=.5),
        'zero_force':dict(seat_force_n=0.),
        'deep_penetration':dict(gap=-.0003,seat_gap_min_m=-.0003),
        'unthreaded_push':dict(captured=False),
        'thread_failure':dict(valid=False),
        'fixture_movement':dict(fixture_displacement=.0021)}
    template=State(captured=True,turns=2.,gap=0.,socket_engaged=True,seat_contact=True,
        seat_contact_fraction=1.,seat_force_n=.14,seat_gap_min_m=0.,seat_gap_max_m=0.)
    for name,changes in cases.items():
        s=State(**{**vars(template),**changes});h=History()
        for _ in range(20):h.update(s,.05)
        failures[name]=dict(score=h.score,invalid=h.invalid,reason=h.reason)
        assert h.score==(.4 if name in ['old_0.196mm_gap','missing_contact','intermittent_contact','zero_force'] else 0.)
    diagnostic=[]
    for torque,cycles in [(-10.,0),(0.,0),(10.,100)]:
        h=History();s=State(**{**vars(template),'interface_torque':torque,'ratchet_cycles':cycles})
        for _ in range(10):h.update(s,.05)
        diagnostic.append(h.score)
    assert diagnostic==[1.,1.,1.]
    template.seat_contact=False
    assert h.update(template,.05)==.4 and not h.tightened
    assert not errors and geometry_error<.000002
    return dict(passed=True,steps=len(rows),final_score=history.score,first_full_step=first_full,
        score_mismatches=errors,max_reconstructed_geometry_error_m=geometry_error,
        failure_cases=failures,diagnostic_only_scores=diagnostic,loss_of_seating_score=h.score,
        initial_gap_m=INITIAL_GAP,final_gap_m=rows[-1]['feedback']['hardware']['seat_gap_m'],
        maximum_pitch_error_m=max(abs(r['feedback']['hardware']['pitch_error_m']) for r in rows),
        maximum_mean_socket_torque_nm=max(abs(r['feedback']['hardware']['socket_torque_nm']) for r in rows),
        final_cycles=rows[-1]['feedback']['hardware']['ratchet_cycles'])


if __name__=='__main__':
    p=Path(sys.argv[1]);rows=([json.loads(line) for line in p.read_text().splitlines()] if p.suffix=='.jsonl' else json.loads(p.read_text()))
    print(json.dumps(audit(rows),indent=2))
