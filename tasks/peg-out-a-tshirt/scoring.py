"""Evidence-bound human judging rubric; no ground-truth pose controller.

Values come from released hold video, close-up photographs and a ruler. The
checker does not infer a clasp from proximity or from a hidden attachment flag.
It refuses to score missing evidence or unknown integrity. Provisional limits
must be fixed before collecting final demo and Dream evidence.
"""
import math


def score(j):
    required=('evidence_sha256','integrity_checked','intact','unsupported_hold_s',
              'minimum_clearance_mm','retained_pegs','peg_separation_mm',
              'supported_only_by_line_and_pegs')
    if any(k not in j for k in required):
        raise ValueError('incomplete judging record')
    evidence=j['evidence_sha256']
    if not isinstance(evidence,dict) or not {'released_video','left_jaw_photo','right_jaw_photo'} <= set(evidence):
        raise ValueError('released video and both jaw photos are required')
    if any(not isinstance(v,str) or len(v)!=64 or any(c not in '0123456789abcdef' for c in v)
           for v in evidence.values()):
        raise ValueError('invalid evidence digest')
    if j['integrity_checked'] is not True:
        raise ValueError('fabric, peg and contact integrity needs review')
    for field in ('unsupported_hold_s','minimum_clearance_mm','peg_separation_mm'):
        value=j[field]
        if type(value) not in (int,float) or not math.isfinite(value) or (value < 0 and field != 'minimum_clearance_mm'):
            raise ValueError('invalid measurement: '+field)
    if type(j['intact']) is not bool:
        raise ValueError('integrity outcome must be known')
    if type(j['supported_only_by_line_and_pegs']) is not bool:
        raise ValueError('support sources require a visible assessment')
    if not j['supported_only_by_line_and_pegs']:
        return 0.
    if j['intact'] is not True:
        return 0.
    if j['unsupported_hold_s'] < 5. or j['minimum_clearance_mm'] < 20.:
        return 0.
    pegs=j['retained_pegs']
    if not isinstance(pegs,list) or len(pegs)!=2 or any(type(x) is not bool for x in pegs):
        raise ValueError('two independent visible peg-capture decisions required')
    count=sum(pegs)
    if count==2 and j['peg_separation_mm'] >= 100.:
        return 1.
    return .5 if count else 0.
