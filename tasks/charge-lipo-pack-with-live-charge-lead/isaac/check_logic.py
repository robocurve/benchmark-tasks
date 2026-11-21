"""Engine-independent checks of the scorer (task_scoring.py) and the charger panel model (charger.py): synthetic
measurement sequences for every veto and every score level. No simulator.
    python check_logic.py [OUT.json]
"""
import copy
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from charger import KEYS, Charger, KeySwitches  # noqa: E402
from task_scoring import ChargeScorer  # noqa: E402

DT = 0.05
PHYS = 1 / 960


def m0(**kw):
    """Reset state: XT60 mated (live), both banana plugs loose, presets seated, nothing touched or moved."""
    m = dict(live=True, xt60=dict(live=True, seated=True), positive=dict(connected=False),
             negative=dict(connected=False), crossed_positive=dict(connected=False),
             crossed_negative=dict(connected=False), power=dict(connected=True), balance=dict(seated=True),
             metal_contacts=[], gripper_contacts=[], keys_pressed=False, xt60_apart_while_running=False,
             charger_moved=dict(shift_m=0.0, turn_deg=0.0), riser_moved=dict(shift_m=0.0, turn_deg=0.0),
             lcd_error=False, running=False, clear=True)
    for k, v in kw.items():
        if isinstance(v, dict) and isinstance(m.get(k), dict):
            m[k] = {**m[k], **v}
        else:
            m[k] = v
    return m


APART = dict(live=False, xt60=dict(live=False, seated=False))
SEATED = dict(positive=dict(connected=True), negative=dict(connected=True))
MATED_SEATED = {**SEATED}
RUN = dict(running=True, **SEATED)
TOUCH = [dict(body='positive:plug', force_n=1.2, t=1.0)]
WIRE = [dict(body='negative:c37', force_n=0.4, t=1.0)]
SHORT = [dict(geom='positive_pin', other='negative_pin', kind='other_pin', gap_m=0.0003)]


def run(seq, intact=None):
    """seq: list of (seconds, m) segments; returns the score and veto at the end and the score history."""
    sc = ChargeScorer()
    hist = []
    for secs, m in seq:
        for _ in range(max(1, round(secs / DT))):
            sc.step(DT, m, intact=True if intact is None else intact(len(hist)))
            hist.append(sc.score)
    return sc.score, sc.veto, hist


def safe_route():
    return [(1, m0()), (1, m0(**APART)), (1, m0(**APART, gripper_contacts=TOUCH)),  # dead plugs may be handled
            (1, m0(**APART, **SEATED)), (1, m0(**SEATED)), (1, m0(**SEATED, keys_pressed=True)),
            (5.5, m0(**RUN))]


CASES = []


def case(name, seq, score, veto=None, intact=None, max_score=None):
    s, v, hist = run(seq, intact)
    ok = abs(s - score) < 1e-9 and (veto is None and v is None or veto is not None and v is not None and veto in v)
    if max_score is not None:
        ok = ok and max(hist) <= max_score
    CASES.append(dict(case=name, ok=bool(ok), score=s, expected=score, veto=v, expected_veto=veto,
                      max_score=max(hist)))


# score ladder
case('do nothing (XT60 mated, live)', [(10, m0())], 0.0)
case('XT60 apart only', [(1, m0()), (2, m0(**APART))], 0.1)
case('XT60 apart, one plug seated', [(1, m0(**APART, positive=dict(connected=True)))], 0.1)
case('XT60 apart, both seated (never re-mated)', [(1, m0(**APART)), (1, m0(**APART, **SEATED))], 0.25)
case('XT60 half re-mated (live, not mated), both seated', [(1, m0(**APART, **SEATED)),
                                                          (1, m0(**SEATED, xt60=dict(seated=False)))], 0.25)
case('both seated, XT60 mated, LCD error', [(1, m0(**APART, **SEATED)), (1, m0(**SEATED, lcd_error=True))], 0.25)
case('both seated, XT60 mated, charge not started', [(1, m0(**APART, **SEATED)), (2, m0(**SEATED))], 0.5)
case('charging < 5 s', [(1, m0(**APART, **SEATED)), (3, m0(**RUN))], 0.75)
case('safe route end state', safe_route(), 1.0)
case('keys pressed in the 5 s', [(1, m0(**APART, **SEATED)), (3, m0(**RUN)), (0.05, m0(**RUN, keys_pressed=True)),
                                 (3, m0(**RUN))], 0.75, max_score=0.75)
case('fingertip within 3 cm in the 5 s', [(1, m0(**APART, **SEATED)), (3, m0(**RUN)), (0.05, m0(**RUN, clear=False)),
                                          (3, m0(**RUN))], 0.75, max_score=0.75)
case('hand on the charger the whole time', [(1, m0(**APART, **SEATED)), (8, m0(**RUN, clear=False))], 0.75)
# vetoes
case('live insertion with the XT60 never touched (gripper on a live plug)',
     [(1, m0(gripper_contacts=TOUCH)), (1, m0(**SEATED)), (6, m0(**RUN))], 0.0, 'gripper touched')
case('gripper on the live wire within 5 cm', [(1, m0(gripper_contacts=WIRE)), (1, m0())], 0.0, 'gripper touched')
case('touch after re-mating, while live', safe_route() + [(0.05, m0(**RUN, gripper_contacts=TOUCH)), (1, m0(**RUN))],
     0.0, 'gripper touched')
case('short while live', [(1, m0(metal_contacts=SHORT))], 0.0, 'live short')
case('pins touching while dead (XT60 apart)', [(1, m0(**APART, metal_contacts=SHORT)), (1, m0(**APART))], 0.1)
case('plug seated in the wrong jack', [(1, m0(**APART, crossed_positive=dict(connected=True)))], 0.0, 'wrong jack')
case('AC plug backs out', [(1, m0(**APART, power=dict(connected=False))), (1, m0(**APART))], 0.0, 'AC cord')
case('balance plug backs out', [(1, m0(**APART, balance=dict(seated=False))), (1, m0(**APART))], 0.0, 'balance')
case('charger moved 11 mm', [(1, m0(**APART, charger_moved=dict(shift_m=0.011, turn_deg=0.0)))], 0.0, 'charger moved')
case('charger turned 6 deg', [(1, m0(**APART, charger_moved=dict(shift_m=0.0, turn_deg=6.0)))], 0.0, 'charger moved')
case('riser moved 11 mm', [(1, m0(**APART, riser_moved=dict(shift_m=0.011, turn_deg=0.0)))], 0.0, 'riser moved')
case('object fell off the table', [(1, m0(**APART))], 0.0, 'left the table', intact=lambda i: i < 10)
case('XT60 apart while charging', safe_route()[:-1] + [(2, m0(**RUN)), (0.05, m0(**RUN, xt60_apart_while_running=True)),
                                                       (1, m0(**APART, **SEATED))], 0.0, 'while the charger')
case('veto latches after a full score', safe_route() + [(0.05, m0(**RUN, metal_contacts=SHORT)), (2, m0(**RUN))],
     0.0, 'live short')


# charger panel model driven by key travel (debounced switches)
def panel(script, main=True, balance=True):
    ch, sw = Charger(), KeySwitches()
    for secs, key, link in script:
        for _ in range(max(1, round(secs / PHYS))):
            closed = sw.sample(PHYS, {k: (0.0011 if k == key else 0.0) for k in KEYS})
            ch.step(PHYS, closed, main if link is None else link[0], balance if link is None else link[1])
    return ch


def pcase(name, ch, page, lines=None):
    ok = ch.page == page and (lines is None or tuple(ch.display()) == lines)
    CASES.append(dict(case=name, ok=bool(ok), page=ch.page, lcd=list(ch.display()), expected_page=page,
                      events=ch.events))


pcase('panel: preset program screen', panel([(0.5, None, None)]), 'program', ('LiPo BALANCE CHG', '2.2A 11.1V(3S)'))
pcase('panel: DEC/INC/short ENTER do not change the program',
      panel([(0.2, 'DEC', None), (0.2, None, None), (0.2, 'INC', None), (0.2, None, None), (0.2, 'ENTER', None),
             (0.2, None, None)]), 'program', ('LiPo BALANCE CHG', '2.2A 11.1V(3S)'))
pcase('panel: ENTER 3 s with the XT60 apart', panel([(3.2, 'ENTER', None), (0.2, None, None)], main=False), 'error',
      ('ERROR', 'CHECK MAIN PORT'))
pcase('panel: ENTER 2.5 s is not a long press', panel([(2.5, 'ENTER', None), (0.2, None, None)]), 'program')
pcase('panel: ENTER 3 s -> cell check', panel([(3.2, 'ENTER', None), (0.2, None, None)]), 'confirm',
      ('R:3SER S:3SER', 'CONFIRM(ENTER)'))
start = [(3.2, 'ENTER', None), (0.3, None, None), (0.2, 'ENTER', None), (1.0, None, None)]
pcase('panel: ENTER confirms -> charging', panel(start), 'charging')
pcase('panel: XT60 pulled while charging', panel(start + [(0.1, None, (False, True))]), 'error',
      ('ERROR', 'CONNECTION BREAK'))
pcase('panel: STOP ends the charge', panel(start + [(0.2, 'STOP', None), (0.2, None, None)]), 'program')
pcase('panel: a 5 ms bounce is not a press', panel([(0.005, 'STOP', None), (0.2, None, None)] + start), 'charging')

out = dict(passed=sum(c['ok'] for c in CASES), total=len(CASES), all_passed=all(c['ok'] for c in CASES), cases=CASES)
print(json.dumps({k: out[k] for k in ('passed', 'total', 'all_passed')}))
for c in CASES:
    print(('PASS ' if c['ok'] else 'FAIL ') + c['case'], c.get('score', c.get('page')), c.get('veto') or '')
if len(sys.argv) > 1:
    Path(sys.argv[1]).write_text(json.dumps(out, indent=2))
sys.exit(0 if out['all_passed'] else 1)
