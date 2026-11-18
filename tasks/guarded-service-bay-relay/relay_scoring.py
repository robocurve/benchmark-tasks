"""Physics-object task score for the guarded service-bay relay.

The object subscore evaluates supported placement, dwell, transfer order and
fixture movement. The task combines this subscore with the submission-local
Torch PhysX full-arm geometry gate. No contact-force certification is claimed.
"""
from __future__ import annotations
import math

TABLE_TOP = 0.7475
CUBE_EDGE = 0.040
PICK = (0.160, 0.150)
STAGE = (0.300, 0.040)
GOAL = (0.200, -0.200)
STAGE_PAD = (0.100, 0.095, 0.010)
GOAL_PAD = (0.100, 0.100, 0.001)
STAGE_TOP = TABLE_TOP + STAGE_PAD[2]
GOAL_TOP = TABLE_TOP + GOAL_PAD[2]
STAGE_Z = STAGE_TOP + CUBE_EDGE / 2
GOAL_Z = GOAL_TOP + CUBE_EDGE / 2
# Each score() call is a 20-Hz control step, not a 240-Hz physics step.
STAGE_DWELL_CALLS = 6  # 0.30 s
GOAL_DWELL_CALLS = 6  # 0.30 s
SUPPORT_Z_TOLERANCE_M = 0.003
MAX_CENTER_SPEED_M_PER_CALL = 0.045  # conservative 0.90 m/s at 20 Hz
SOURCE_LIFT_M = 0.030
STAGE_RELIFT_M = 0.035

_HISTORY = {}
_SAFETY = {}

def reset_history():
    _HISTORY.clear()
    _HISTORY.update(stage_streak=0, goal_streak=0, source_lift=False,
                    stage_qualified=False, stage_relift=False,
                    last_start=False, previous_center=None, last_level=0.0)
    _SAFETY.clear()
    _SAFETY.update(latched=False, reason=None)
reset_history()

def _dist_xy(p, xy):
    return math.hypot(p[0]-xy[0], p[1]-xy[1])

def _finite3(vec):
    try:
        x=tuple(float(a) for a in vec[:3])
        return x if len(x)==3 and all(map(math.isfinite,x)) else None
    except (TypeError, AttributeError, ValueError, IndexError):
        return None

def _state(state):
    p=_finite3(getattr(state, 'pos', None))
    try: rest=bool(state.resting())
    except (AttributeError, TypeError, ValueError): rest=False
    return p,rest

def _bounds(state, p):
    """Use actual world AABB when supplied by YAM ObjectState.

    Legacy unit-test surrogates provide only a centre; the 40 mm cube extent
    is then inferred, but do not use that fallback as independent contact proof.
    """
    lo=_finite3(getattr(state,'low',None))
    hi=_finite3(getattr(state,'high',None))
    if lo is None or hi is None:
        if p is None:return None
        half=CUBE_EDGE/2
        return ((p[0]-half,p[1]-half,p[2]-half),
                (p[0]+half,p[1]+half,p[2]+half))
    if not all(lo[i] <= hi[i] for i in range(3)):
        return None
    return lo,hi

def _supported(state,p,xy,pad,top,center_tolerance):
    if p is None:return False
    bounds=_bounds(state,p)
    if bounds is None:return False
    lo,hi=bounds
    # Crucial: cube UNDERSIDE must be nearly touching the actual pad top.
    # This rejects a cube held stationary 9 mm above the pad.
    if abs(lo[2]-top)>SUPPORT_Z_TOLERANCE_M:return False
    # Require an actual support footprint, not cube centre hovering over a pad.
    if abs(p[0]-xy[0])>center_tolerance or abs(p[1]-xy[1])>center_tolerance:
        return False
    if lo[0]<xy[0]-pad[0]/2-.001 or hi[0]>xy[0]+pad[0]/2+.001:
        return False
    if lo[1]<xy[1]-pad[1]/2-.001 or hi[1]>xy[1]+pad[1]/2+.001:
        return False
    return True

def _latch(reason):
    _SAFETY.update(latched=True,reason=reason)

def _scoring_step(component):
    p,rest=_state(component)
    if p is None:
        _latch('NONFINITE_COMPONENT_STATE')
        return 0.0
    prev=_HISTORY['previous_center']
    if prev is not None and math.dist(p,prev)>MAX_CENTER_SPEED_M_PER_CALL:
        # Do not give credit to teleporting a component between stations.
        # The simulation has a 20-Hz control update (and 240-Hz physics).
        _latch('DISCONTINUOUS_COMPONENT_MOTION')
    _HISTORY['previous_center']=p
    in_start=_dist_xy(p,PICK)<.006 and abs(p[2]-(TABLE_TOP+.001+CUBE_EDGE/2))<.007
    if in_start and not _HISTORY['last_start']:
        _HISTORY.update(stage_streak=0,goal_streak=0,source_lift=False,
                        stage_qualified=False,stage_relift=False)
    _HISTORY['last_start']=in_start
    if _dist_xy(p,PICK)<.040 and p[2]>TABLE_TOP+CUBE_EDGE/2+SOURCE_LIFT_M:
        _HISTORY['source_lift']=True
    # The relift occurs after a valid stage. It is not enough to slide a cube
    # directly between two colored floor markers.
    if _HISTORY['stage_qualified'] and p[2]>STAGE_Z+STAGE_RELIFT_M:
        _HISTORY['stage_relift']=True
    staged=(rest and _supported(component,p,STAGE,STAGE_PAD,STAGE_TOP,.028))
    if staged and _HISTORY['source_lift'] and not _HISTORY['stage_relift']:
        _HISTORY['stage_streak']+=1
        if _HISTORY['stage_streak']>=STAGE_DWELL_CALLS:
            _HISTORY['stage_qualified']=True
    else:
        _HISTORY['stage_streak']=0
    goal=(rest and _supported(component,p,GOAL,GOAL_PAD,GOAL_TOP,.015))
    if goal and _HISTORY['stage_qualified'] and _HISTORY['stage_relift']:
        _HISTORY['goal_streak']+=1
    else:
        _HISTORY['goal_streak']=0
    if _HISTORY['goal_streak']>=GOAL_DWELL_CALLS:
        _HISTORY['last_level']=1.0
    elif _HISTORY['stage_qualified']:
        _HISTORY['last_level']=.5
    else:
        _HISTORY['last_level']=0.0
    return _HISTORY['last_level']

def history_score(component):
    v=_scoring_step(component)
    return 0.0 if _SAFETY['latched'] else v

def object_safety_score(objects):
    """Object-only component of the official scorer. Arm exclusion is applied
    separately inside guarded_service_bay_relay.score().
    """
    if not isinstance(objects, dict) or {'component','operator_reach_surrogate'}-objects.keys():
        _latch('OBJECT_STATE_MISSING')
        return 0.0
    p,_=_state(objects['component'])
    guard=_finite3(getattr(objects['operator_reach_surrogate'],'pos',None))
    if p is None or guard is None:
        _latch('NONFINITE_OBJECT_STATE')
        return 0.0
    nominal=(.082,0.,TABLE_TOP+.660/2)
    if math.dist(guard,nominal)>.003:
        _latch('PROTECTED_FIXTURE_DISPLACED_GT_3MM')
    # Inflated cube vs solid protected work-zone fixture.
    if all(abs(p[i]-guard[i])<=.020+h+.002 for i,h in enumerate((.040,.040,.330))):
        _latch('PAYLOAD_ENTERED_PROTECTED_FIXTURE_MARGIN')
    v=_scoring_step(objects['component'])
    return 0.0 if _SAFETY['latched'] else v

def object_safety_state():
    return {**_SAFETY, 'history':dict(_HISTORY)}
