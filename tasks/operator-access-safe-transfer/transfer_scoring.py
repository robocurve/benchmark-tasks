"""Finite, event-based, anti-shortcut task scorer (YAM task callback only).

The benchmark measures a *released*, stably supported handoff followed by a
released final delivery, in a personnel-occupied service cell. A held cube cannot
be counted as deposited: the closest collision surface on either YAM gripper
must leave at least 25 mm of free space around the component for >=0.50 s at staging, >=1.00 s at final delivery.

No contact force is manufactured: after the grippers move away, gravity plus
stable support geometry provides the physical support/release observation. For
a real trial the same condition is checked with a 25-mm spacer gauge and video.

The scorer advances ONCE per official score(objects) callback; success() is
read-only. Sticky safety failures cannot be erased by returning to the start.
"""
from __future__ import annotations
import math
from typing import Any
import personnel_guard as person

TABLE_TOP = person.TOP
CUBE_EDGE = 0.040
PICK = (0.160, 0.150)
STAGE = (0.300, 0.040)
GOAL = (0.200, -0.200)
STAGE_PAD = (0.100, 0.095, 0.010)
GOAL_PAD = (0.100, 0.100, 0.001)
STAGE_TOP = TABLE_TOP + STAGE_PAD[2]
GOAL_TOP = TABLE_TOP + GOAL_PAD[2]
STAGE_CUBE_Z = STAGE_TOP + CUBE_EDGE / 2
GOAL_CUBE_Z = GOAL_TOP + CUBE_EDGE / 2
STAGE_DWELL = 10  # 0.50 seconds, sampled at 20 Hz
GOAL_DWELL = 20  # 1.00 second; final ungrasped supported rest
SUPPORT_HEIGHT_TOL = 0.003
RELEASE_GAP = person.GRIPPER_CLEARANCE_AT_RELEASE_M
SOURCE_LIFT = 0.030
STAGE_RELIFT = 0.035
MAX_STEP_DISPLACEMENT = 0.045  # m per 20-Hz callback: detect teleportation
MAX_FIXTURE_SHIFT = 0.003
SOURCE_Z = TABLE_TOP + .001 + CUBE_EDGE / 2

STATE = {}

def reset_for_new_trial():
    STATE.clear()
    STATE.update(calls=0, source_lift_seen=False, stage_release_streak=0,
                 stage_qualified=False, stage_relift_seen=False,
                 goal_release_streak=0, level=0., fault=None,
                 last_cube_center=None, min_release_gap_m=None, latest_release_gap_m=None,
                 min_person_gap_m=None)
reset_for_new_trial()


def _finite3(x):
    try:
        p = tuple(float(v) for v in x[:3])
    except (TypeError, AttributeError, ValueError, IndexError):
        return None
    return p if len(p)==3 and all(map(math.isfinite,p)) else None


def _vec3(state, field):
    return _finite3(getattr(state,field,None))


def _fail(reason):
    if STATE['fault'] is None:
        STATE['fault'] = str(reason)


def _support(cube, center, pad_size, top, center_tol):
    """Full footprint on actual pad top with approximately level AABB.

    Support verification is NOT solely based on resting(); grippers must also
    be visibly/separately released for the dwell period.
    """
    p = _vec3(cube,'pos')
    low = _vec3(cube,'low')
    high = _vec3(cube,'high')
    if p is None or low is None or high is None:
        return False
    if any(lo>hi for lo,hi in zip(low,high)):
        return False
    try:
        rest = bool(cube.resting())
    except (ValueError, AttributeError, TypeError):
        rest = False
    if not rest or abs(low[2]-top)>SUPPORT_HEIGHT_TOL:
        return False
    if abs(p[0]-center[0])>center_tol or abs(p[1]-center[1])>center_tol:
        return False
    return all(low[i]>=center[i]-pad_size[i]/2-.001 and
               high[i]<=center[i]+pad_size[i]/2+.001 for i in (0,1))


def _checkpoint(objects):
    if not isinstance(objects,dict) or {'component','service_column','operator_hand_dummy'} - objects.keys():
        _fail('MISSING_SCENE_OBJECT_STATE')
        return None
    cube=objects['component']
    p=_vec3(cube,'pos')
    if p is None or _vec3(cube,'low') is None or _vec3(cube,'high') is None:
        _fail('UNVERIFIED_PAYLOAD_AABB')
        return None
    hand=_vec3(objects['operator_hand_dummy'],'pos')
    if not person.person_center_ok(hand):
        _fail('OPERATOR_SURROGATE_DISPLACED_OR_MISSING')
    fixture=_vec3(objects['service_column'],'pos')
    nominal=(.082,.000,TABLE_TOP+.660/2)
    if fixture is None or math.dist(fixture,nominal)>MAX_FIXTURE_SHIFT:
        _fail('SERVICE_FIXTURE_DISPLACED_GT_3MM')
    # Read the true world-space workpiece AABB: rotated cube corners matter.
    low=_vec3(cube,'low'); high=_vec3(cube,'high')
    if any(low[i]>high[i] for i in range(3)):
        _fail('INVERTED_WORKPIECE_BOUNDS')
        return None
    pgap=person.box_box_gap(low,high)
    STATE['min_person_gap_m']=(pgap if STATE['min_person_gap_m'] is None
                                 else min(pgap,STATE['min_person_gap_m']))
    if pgap<person.MIN_PERSONNEL_GAP_M:
        _fail('PAYLOAD_ENTERED_PERSONNEL_STANDOFF')
    # The solid equipment fixture remains separate from the personnel dummy.
    fbox=((.082-.040,-.040,TABLE_TOP),(.082+.040,.040,TABLE_TOP+.660))
    low=_vec3(cube,'low'); high=_vec3(cube,'high')
    if low is not None and high is not None and person.box_box_gap(low,high,*fbox)<1e-6:
        # Exact contact/overlap is ambiguous at ~0 mm; conservative fail.
        _fail('PAYLOAD_CONTACTED_SERVICE_FIXTURE')
    return cube,p


def update(objects, poses, model, capsule, segment_box):
    STATE['calls'] += 1
    if STATE['fault']:
        return 0.
    row=_checkpoint(objects)
    if row is None or STATE['fault']:
        return 0.
    cube,p=row
    prev=STATE['last_cube_center']
    if prev is not None and math.dist(prev,p)>MAX_STEP_DISPLACEMENT:
        _fail('PAYLOAD_JUMP_EXCEEDS_STEP_LIMIT')
    STATE['last_cube_center']=p
    if STATE['fault']:
        return 0.
    try:
        gap, _, _ = person.finger_body_gap(poses,model,capsule,segment_box,p)
    except (ValueError, TypeError, KeyError) as exc:
        _fail('GRIPPER_GEOMETRY_UNAVAILABLE:'+type(exc).__name__)
        return 0.
    STATE['latest_release_gap_m']=gap
    STATE['min_release_gap_m']=(gap if STATE['min_release_gap_m'] is None
                                   else min(gap,STATE['min_release_gap_m']))
    released=gap>=RELEASE_GAP
    if math.hypot(p[0]-PICK[0],p[1]-PICK[1])<.040 and p[2]>SOURCE_Z+SOURCE_LIFT:
        STATE['source_lift_seen']=True
    if STATE['stage_qualified'] and p[2]>STAGE_CUBE_Z+STAGE_RELIFT:
        STATE['stage_relift_seen']=True
    in_stage=_support(cube,STAGE,STAGE_PAD,STAGE_TOP,.028)
    if STATE['source_lift_seen'] and not STATE['stage_relift_seen'] and in_stage and released:
        STATE['stage_release_streak']+=1
        if STATE['stage_release_streak']>=STAGE_DWELL:
            STATE['stage_qualified']=True
    else:
        STATE['stage_release_streak']=0
    in_goal=_support(cube,GOAL,GOAL_PAD,GOAL_TOP,.015)
    if STATE['stage_qualified'] and STATE['stage_relift_seen'] and in_goal and released:
        STATE['goal_release_streak']+=1
    else:
        STATE['goal_release_streak']=0
    if STATE['goal_release_streak']>=GOAL_DWELL:
        STATE['level']=1.0
    elif STATE['stage_qualified']:
        STATE['level']=0.5
    else:
        STATE['level']=0.0
    return float(STATE['level'])


def state():
    return dict(STATE)
