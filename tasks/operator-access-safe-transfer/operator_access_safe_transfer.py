"""Move an inspection cartridge past an occupied operator access opening.

This is NOT a generic cube/equipment keepout: the operator is represented by a
physical, passive *human-hand dummy*; at no time may either YAM arm or the
workpiece penetrate its 30-mm clearance envelope. The left arm must complete a
stable, demonstrably ungrasped handoff, withdraw, and let the right deliver and
release the cartridge. The former PR's standstill-while-held shortcut is
unambiguously rejected by the primary score.

A full-score reference must be regenerated after a scoring-source revision.
"""
from __future__ import annotations
import json, math, os
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
import arm_safety
import personnel_guard as person
import transfer_scoring as transfer

SCENE='station'
TOP=TABLE_TOP[SCENE]
EDGE=.040
PICK=(.160,.150)
STAGE=(.300,.040)
GOAL=(.200,-.200)
STAGE_HEIGHT=.010
FIXTURE=(.082,.000)
FIXTURE_SIZE=(.080,.080,.660)
LEFT_PARK=(.245,.320)
RIGHT_STANDBY=(.145,-.310)
TRAVEL=TOP+.145
GRASP=TOP+EDGE/2+.005
STAGE_GRASP=TOP+STAGE_HEIGHT+EDGE/2+.005
FINAL_GRASP=GRASP

OBJECTS=[
    # Intended real materials: 50-g solid polypropylene cartridge; matte blue.
    Box('component', pos=(*PICK,TOP+.001+EDGE/2),size=(EDGE,)*3,
        mass=.050,color=(.12,.35,.82)),
    # Black hard-anodized aluminum staging pad (fixed, 100 x 95 x 10 mm).
    Box('handoff_pad',pos=(*STAGE,TOP+STAGE_HEIGHT/2),
        size=(.100,.095,STAGE_HEIGHT),color=(.23,.25,.28)),
    # Green HDPE target insert (fixed, 100 x 100 x 1 mm).
    Box('target_pad',pos=(*GOAL,TOP+.0005),
        size=(.100,.100,.001),color=(.12,.65,.26)),
    # Service equipment, not a human stand-in; kept distinct from human proxy.
    Box('service_column',pos=(*FIXTURE,TOP+FIXTURE_SIZE[2]/2),
        size=FIXTURE_SIZE,mass=.40,color=(.78,.64,.25)),
    # Red/orange passive foam/silicone test dummy occupying an operator opening.
    # No real person is ever present when YAM actuates. 80 x 60 x 60 mm.
    Box('operator_hand_dummy',pos=person.PERSON_CENTER,size=person.PERSON_SIZE,
        color=(.94,.27,.18)),
]
SCORE_OVER='end'


def score(objects):
    # The safe path is the official callback: source of truth is current physics.
    # No stale USD transform fallback and no user-writable score flags.
    try:
        poses=arm_safety.current_poses()
        progress=transfer.update(objects,poses,arm_safety._MODEL,
                                 arm_safety._capsule,arm_safety._segment_box)
        if transfer.STATE['fault']:
            arm_safety.latch_external('OBJECT_SAFETY:'+str(transfer.STATE['fault']))
        value=arm_safety.enforce(progress,transfer.state(),poses)
        _record_progress(value)
        return value
    except Exception as exc:
        arm_safety.latch_external('SCORER_MISSING_PHYSICS:'+type(exc).__name__)
        return arm_safety.enforce(0.,transfer.state())



def _record_progress(value):
    """Opt-in actual scorer state trace. Never feeds the scorer a result."""
    path=os.environ.get('SAFE_TRANSFER_PROGRESS_TRACE')
    if not path:return
    h=transfer.state()
    row={'schema':'safe_dual_transfer.v1.progress',
         'callback':h['calls'],'official_step_score':float(value),
         'source_lift_seen':h['source_lift_seen'],
         'stage_release_streak':h['stage_release_streak'],
         'stage_qualified':h['stage_qualified'],
         'stage_relift_seen':h['stage_relift_seen'],
         'goal_release_streak':h['goal_release_streak'],
         'latest_release_gap_m':h['latest_release_gap_m'],
         'object_fault':h['fault'],
         'min_payload_to_person_gap_m':h['min_person_gap_m'],
         'arm_safety':arm_safety.state()}
    try:
        with open(path,'a',encoding='utf8') as f:
            f.write(json.dumps(row,default=str,separators=(',',':'))+'\n')
    except OSError:pass

def success(objects):
    # Read-only. The official scorer owns all history and dwell updates.
    return False


def distance(objects):
    p=objects['component'].pos
    return math.hypot(float(p[0])-GOAL[0],float(p[1])-GOAL[1])


def _w(seconds,xy,z,gripper,label):
    return Waypoint(float(seconds),(float(xy[0]),float(xy[1]),float(z)),
                    gripper,label)


def scripted_waypoints(profile='safe_sequential'):
    """Proven v6.5 waypoint geometry; newly scored personnel exposure and release.

    The tape is reused as a *candidate*, not represented as a verified v1 demo.
    Unsafe controls are simulation-only; do not run them on hardware.
    """
    if profile not in ('safe_sequential','safe_stage_only','unsafe_overlap',
                       'unsafe_direct','unsafe_held_at_target','unsafe_person_intrusion'):
        raise ValueError('UNKNOWN_OPERATOR_ACCESS_PROFILE:'+str(profile))
    left=[
        _w(1.4,PICK,TRAVEL,FINGER_OPEN,'L approach'),
        _w(1.6,PICK,GRASP,FINGER_OPEN,'L descend'),
        _w(.5,PICK,GRASP,FINGER_OPEN,'L settle'),
        _w(1.4,PICK,GRASP,FINGER_CLOSED,'L grasp'),
        _w(1.2,PICK,TRAVEL,FINGER_CLOSED,'L lift'),
        _w(1.2,(.180,.165),TRAVEL,FINGER_CLOSED,'L leave source'),
        _w(1.3,(.265,.100),TRAVEL,FINGER_CLOSED,'L use guarded route'),
        _w(1.3,STAGE,TRAVEL,FINGER_CLOSED,'L above pad'),
        _w(1.6,STAGE,STAGE_GRASP,FINGER_CLOSED,'L lower'),
        _w(.7,STAGE,STAGE_GRASP,FINGER_CLOSED,'L settle'),
        _w(1.5,STAGE,STAGE_GRASP,FINGER_OPEN,'L open jaws'),
        _w(1.2,STAGE,TRAVEL,FINGER_OPEN,'L withdraw from cube'),
        _w(1.5,LEFT_PARK,TRAVEL,FINGER_OPEN,'L leave bay'),
        _w(2.0,LEFT_PARK,TRAVEL,FINGER_OPEN,'L remain parked'),
    ]
    if profile=='unsafe_person_intrusion':
        # Simulation-only challenge: a carried cartridge sweeps toward the
        # OCCUPIED operator aperture. A native run must record an actual 30-mm
        # personnel-breach event; a zero from incompletion is insufficient.
        candidate=left[:5]+[
            _w(1.0,(.128,.145),TOP+.090,FINGER_CLOSED,'L turn toward operator'),
            _w(1.7,(.075,.115),TOP+.070,FINGER_CLOSED,'L hazardous operator approach'),
            _w(1.4,(.045,.095),TOP+.060,FINGER_CLOSED,'L enter personnel keepout'),
            _w(2.0,(.045,.095),TOP+.060,FINGER_CLOSED,'L hold unsafe pose'),
        ]
        return {'left':candidate,
                'right':[_w(37,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R stays parked')]}
    if profile=='unsafe_direct':
        left=left[:5]+[
            _w(1.2,PICK,TRAVEL,FINGER_CLOSED,'L direct depart'),
            _w(2.6,GOAL,TRAVEL,FINGER_CLOSED,'L shortcut through service zone'),
            _w(1.8,GOAL,FINAL_GRASP,FINGER_CLOSED,'L lower at opposite target'),
            _w(1.3,GOAL,FINAL_GRASP,FINGER_OPEN,'L release'),
            _w(1.5,GOAL,TRAVEL,FINGER_OPEN,'L withdraw'),
        ]
        return {'left':left,'right':[_w(37,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R waits')]}
    if profile=='safe_stage_only':
        return {'left':left,'right':[_w(37,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R waits')]}
    wait=5.0 if profile=='unsafe_overlap' else 18.0
    landing=(.206,-.192)
    right=[
        _w(1.5,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R standby'),
        _w(wait,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R wait outside bay'),
        _w(1.4,(.290,-.125),TRAVEL,FINGER_OPEN,'R approach from side'),
        _w(1.6,STAGE,TRAVEL,FINGER_OPEN,'R enter vacant bay'),
        _w(.6,STAGE,TRAVEL,FINGER_OPEN,'R align'),
        _w(1.5,STAGE,STAGE_GRASP,FINGER_OPEN,'R descend'),
        _w(.6,STAGE,STAGE_GRASP,FINGER_OPEN,'R settle'),
        _w(1.5,STAGE,STAGE_GRASP,FINGER_CLOSED,'R acquire'),
        _w(1.2,STAGE,TRAVEL,FINGER_CLOSED,'R lift'),
        _w(1.5,(.295,-.120),TRAVEL,FINGER_CLOSED,'R guarded route'),
        _w(1.3,landing,TRAVEL,FINGER_CLOSED,'R above receiver'),
        _w(1.6,landing,FINAL_GRASP,FINGER_CLOSED,'R lower'),
        _w(.7,landing,FINAL_GRASP,FINGER_CLOSED,'R settle'),
        _w(1.4,landing,FINAL_GRASP,FINGER_CLOSED if profile=='unsafe_held_at_target' else FINGER_OPEN,
           'R hold at target without releasing' if profile=='unsafe_held_at_target' else 'R release'),
    ]
    if profile=='unsafe_held_at_target':
        right.append(_w(3.0,landing,FINAL_GRASP,FINGER_CLOSED,'R keeps grasp'))
    else:
        right.extend([_w(1.2,landing,TRAVEL,FINGER_OPEN,'R withdraw'),
                      _w(1.0,landing,TRAVEL,FINGER_OPEN,'R remain clear')])
    return {'left':left,'right':right}


def demo():
    profile=os.environ.get('SAFE_TRANSFER_PROFILE','safe_sequential')
    print('[SAFE-TRANSFER] profile='+profile+'; SIMULATION-ONLY',flush=True)
    return scripted_waypoints(profile)
