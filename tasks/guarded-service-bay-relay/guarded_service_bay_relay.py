"""Guarded service bay v6.5 — centered tall blocker exclusive cube relay.

DIFFERENT from strut-handoff-keepout: no strut, housing/shove score, tight saddle
seating, or radiator wall. Core hazard: competing YAM wrists entering an exclusive
shared transfer bay, violating the safe set-down/wait/release/acquire protocol.

IMPORTANT: Python demo() is a functional reference, not physical safety certification. Task score(objects) receives object state only; the submission-local safety module
reads moving robot-link poses directly from GPU PhysX tensors inside the scorer.
This is geometric exclusion scoring, not contact-force certification.
"""
from __future__ import annotations
import math, os
from yam_arena.task_file import TABLE_TOP, Box
from yam_arena.constants import FINGER_OPEN, FINGER_CLOSED
from yam_arena.demo import Waypoint
from relay_scoring import object_safety_score, reset_history, object_safety_state
from arm_safety import enforce as enforce_arm_safety

SCENE='station'
TOP=TABLE_TOP[SCENE]
EDGE=.040
PICK=(.160,.150)
STAGE=(.300,.040)
GOAL=(.200,-.200)
STAGE_HEIGHT=.010
GUARD=(.082,.000)
GUARD_SIZE=(.080,.080,.660)
LEFT_PARK=(.245,.320)
RIGHT_STANDBY=(.145,-.310)
TRAVEL=TOP+.145
GRASP=TOP+EDGE/2+.005
STAGE_GRASP=TOP+STAGE_HEIGHT+EDGE/2+.005
FINAL_GRASP=GRASP

OBJECTS=[
    Box('component',pos=(*PICK,TOP+.001+EDGE/2),size=(EDGE,)*3,mass=.050,color=(.12,.35,.82)),
    Box('handoff_pad',pos=(*STAGE,TOP+STAGE_HEIGHT/2),size=(.100,.095,STAGE_HEIGHT),color=(.23,.25,.28)),
    Box('target_pad',pos=(*GOAL,TOP+.0005),size=(.100,.100,.001),color=(.12,.65,.26)),
    # Solid 660-mm operator work-zone keepout surrogate (NOT a human).
    # Rendering alpha is applied separately by ops/native_probe/workzone_visual.py;
    # opacity never changes Box mass, collision geometry, or contacts.
    Box('operator_reach_surrogate',pos=(*GUARD,TOP+GUARD_SIZE[2]/2),
        size=GUARD_SIZE,mass=.40,color=(.90,.33,.05)),
]
SCORE_OVER='end'


def score(objects):
    progress = object_safety_score(objects)
    return enforce_arm_safety(progress, object_safety_state()["history"])


def success(objects):
    # Do not terminate early.
    # Only score() advances staging/placement dwell counters.
    return False


def distance(objects):
    pos=objects['component'].pos
    return math.hypot(float(pos[0])-GOAL[0],float(pos[1])-GOAL[1])


def _w(seconds,xy,z,gripper,label):
    return Waypoint(seconds,(float(xy[0]),float(xy[1]),float(z)),gripper,label)


def scripted_waypoints(profile='sequential_relay'):
    """Separate time-synchronized left and right waypoint tapes.

    No direct gripper-to-gripper contact. Cube is deposited on a support, left
    retreats entirely, then right retrieves. The unsafe_overlap profile moves
    right into the shared bay too early, for a negative safety control.
    """
    valid=('sequential_relay','sequential_relay_tuned','left_stage_only','unsafe_overlap','right_only','unsafe_left_direct_attempt','sequential_relay_fast_review')
    if profile not in valid:raise ValueError('UNKNOWN_V6_PROFILE:'+str(profile))
    left=[
        _w(1.4,PICK,TRAVEL,FINGER_OPEN,'L approach over source'),
        _w(1.6,PICK,GRASP,FINGER_OPEN,'L descend onto cube'),
        _w(.5,PICK,GRASP,FINGER_OPEN,'L settle'),
        _w(1.4,PICK,GRASP,FINGER_CLOSED,'L close on cube'),
        _w(1.2,PICK,TRAVEL,FINGER_CLOSED,'L lift cube'),
        _w(1.2,(.180,.165),TRAVEL,FINGER_CLOSED,'L depart source safely'),
        _w(1.3,(.265,.100),TRAVEL,FINGER_CLOSED,'L arc around +X side of protected witness'),
        _w(1.3,STAGE,TRAVEL,FINGER_CLOSED,'L align above handoff support'),
        _w(1.6,STAGE,STAGE_GRASP,FINGER_CLOSED,'L lower onto support'),
        _w(.7,STAGE,STAGE_GRASP,FINGER_CLOSED,'L settle on support'),
        _w(1.5,STAGE,STAGE_GRASP,FINGER_OPEN,'L release cube'),
        _w(1.2,STAGE,TRAVEL,FINGER_OPEN,'L clear shared bay vertically'),
        _w(1.5,LEFT_PARK,TRAVEL,FINGER_OPEN,'L park outside exclusive bay'),
        _w(2.0,LEFT_PARK,TRAVEL,FINGER_OPEN,'L remain parked'),
    ]
    if profile=='unsafe_left_direct_attempt':
        # Deliberately hazardous simulation-only negative control. The straight
        # pickup -> receiver route crosses the inflated fixture and the left-
        # arm-restricted receiver sector. No intermediate set-down, no right
        # arm participation. This must NEVER qualify as a safe full relay.
        direct=[
            _w(1.2,PICK,TRAVEL,FINGER_CLOSED,'L unsafe direct depart'),
            _w(2.6,GOAL,TRAVEL,FINGER_CLOSED,'L unsafe direct shortcut through protected corridor'),
            _w(1.8,GOAL,FINAL_GRASP,FINGER_CLOSED,'L unsafe direct lower at right target'),
            _w(1.3,GOAL,FINAL_GRASP,FINGER_OPEN,'L unsafe direct release'),
            _w(1.5,GOAL,TRAVEL,FINGER_OPEN,'L unsafe direct retreat'),
        ]
        left=left[:5]+direct
        return {'left':left,'right':[
            _w(1.5,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R standby'),
            _w(35.,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R stays out')
        ]}
    if profile=='right_only':
        left=[_w(35.,LEFT_PARK,TRAVEL,FINGER_OPEN,'L never delivers cube')]
    if profile=='left_stage_only':
        return {'left':left,'right':[
            _w(1.5,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R standby'),
            _w(36.,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R stays out')
        ]}
    # Deliberate staging interval: the right wrist must not enter before left
    # has released the cube AND withdrawn its wrist from the shared bay.
    wait=16.0 if profile=='sequential_relay_fast_review' else (18.0 if profile in ('sequential_relay','sequential_relay_tuned') else 5.0)
    delivery_xy=(.206,-.192) if profile in ('sequential_relay_tuned','sequential_relay_fast_review') else GOAL
    # The tuned profile compensates for the archived v6 terminal position bias
    # (~-4 mm X, -11 mm Y). It is only a candidate until Isaac validates it.
    right=[
        _w(1.5,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R standby outside bay'),
        _w(wait,RIGHT_STANDBY,TRAVEL,FINGER_OPEN,'R wait for exclusive ownership'),
        _w(1.4,(.290,-.125),TRAVEL,FINGER_OPEN,'R approach along -Y'),
        _w(1.6,STAGE,TRAVEL,FINGER_OPEN,'R enter free handoff bay'),
        _w(.6,STAGE,TRAVEL,FINGER_OPEN,'R settle overhead'),
        _w(1.5,STAGE,STAGE_GRASP,FINGER_OPEN,'R descend for second grasp'),
        _w(.6,STAGE,STAGE_GRASP,FINGER_OPEN,'R settle at grasp'),
        _w(1.5,STAGE,STAGE_GRASP,FINGER_CLOSED,'R acquire cube'),
        _w(1.2,STAGE,TRAVEL,FINGER_CLOSED,'R lift from handoff pad'),
        _w(1.5,(.295,-.120),TRAVEL,FINGER_CLOSED,'R arc behind protected witness on +X side'),
        _w(1.3,delivery_xy,TRAVEL,FINGER_CLOSED,'R approach goal from receiver side'),
        _w(1.6,delivery_xy,FINAL_GRASP,FINGER_CLOSED,'R place cube on goal'),
        _w(.7,delivery_xy,FINAL_GRASP,FINGER_CLOSED,'R settle'),
        _w(1.4,delivery_xy,FINAL_GRASP,FINGER_OPEN,'R release'),
        _w(1.2,delivery_xy,TRAVEL,FINGER_OPEN,'R retreat'),
        _w(1.0,delivery_xy,TRAVEL,FINGER_OPEN,'R final hold')
    ]
    return {'left':left,'right':right}


def demo():
    profile=os.environ.get('SERVICE_BAY_WAYPOINT_PROFILE','sequential_relay')
    print('[SERVICE-BAY] scripted two-arm relay profile='+profile+'; simulation only',flush=True)
    return scripted_waypoints(profile)
