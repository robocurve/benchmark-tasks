"""Bounce a full-size table-tennis ball into an empty practice cup.
The trajectory comes entirely from the robot's release and native rigid-body contacts.
A laminated practice board supplies a reproducible measured rebound; no ball state is written.
"""
from __future__ import annotations
from collections import deque
import math
import pathlib
import sys
import numpy as np
sys.path.insert(0, str(pathlib.Path(__file__).parent))
import meshkit as mk
from partkit import finger_points
from yam_arena.task_file import TABLE_TOP
SCENE = "station"
TOP = TABLE_TOP[SCENE]
INSTRUCTION = "Bounce the white table-tennis ball once on the board into the red cup, leaving the ball inside the cup and both hands clear."
RADIUS = .020
BOARD_TOP = TOP + .006
CUP_XY = (.378, -.18034)
CUP_H = .115
CUP_R = .0475
CUP_BASE_R = .030
BALL_START = (.20, -.15, BOARD_TOP + RADIUS + .001)
# Full-size 40 mm sphere, native sphere collider, not a faceted convex proxy.
a = np.linspace(-math.pi/2, math.pi/2, 33)
visual = mk.revolve([(max(0,float(RADIUS*math.cos(t))),float(RADIUS*math.sin(t))) for t in a], segments=48)
BALL = mk.mesh_part("ball", pos=BALL_START, mass=.0027,
    shapes=[mk.Shape(visual, color=(.95,.95,.93), collider="none")], friction=.2,
    contact_offset=.0002, angular_damping=.02, linear_damping=0,
    meta={"sphere_radius": RADIUS, "thin_shell_sphere": True, "restitution": 0.0, "contact_report": True})
BOARD = mk.mesh_part("board", pos=(.29,-.15,TOP),
    shapes=[mk.Shape(mk.box((.36,.26,.006)).moved((0,0,.003)),color=(.24,.43,.55),collider="hull")],
    friction=.2, meta={"restitution": .90})
# Thin tapered polypropylene party cup; tape-fixed at its base for a repeatable practice target.
CUP = mk.mesh_part("cup", pos=(*CUP_XY, BOARD_TOP),
    shapes=[mk.Shape(mk.revolve([(0,0),(.030,0),(.0475,.115),(.0465,.115),(.029,.001),(0,.001)],segments=64),
        color=(.8,.035,.04),collider="exact")], friction=.3)
OBJECTS = [BOARD, CUP, BALL]
_STATE = {}

def reset_state():
    _STATE.clear()
    _STATE.update(prev=None, bounces=0, invalid=False, released=False, handled=False, release_step=None, separated=False, history=deque(maxlen=20), steps=0, contacts=deque(), contact_log=[], board_touch=False, sensor_valid=False, sensor_steps=0, sensor_created=False, sensor_time=-1.0, sensor_samples=0)
reset_state()

_SENSOR = None

@mk.on_reset
def _reset(environment):
    global _SENSOR
    reset_state()
    from isaacsim.sensors.experimental.physics import ContactSensor
    class BallContactSensor(ContactSensor):
        def on_physics_step(self, step_dt):
            _STATE['sensor_steps'] += 1
            super().on_physics_step(step_dt)
            _STATE['sensor_created'] = bool(self._sensor_created)
            reading = self.get_sensor_reading()
            _STATE['sensor_valid'] = bool(reading.is_valid)
            if reading.is_valid and float(reading.time) != _STATE['sensor_time']:
                _STATE['sensor_time'] = float(reading.time)
                _STATE['sensor_samples'] += 1
                consume_contacts(self.get_raw_data())
    _SENSOR = BallContactSensor(mk._CONTACT_SENSOR_PATHS['ball'])
    # IsaacLab may step the SimulationContext without dispatching the optional
    # SimulationManager sensor bus. Observe every actual native sim.step instead.
    sim = environment.unwrapped.sim
    if not getattr(sim, '_ball_contact_observer', False):
        original_step = sim.step
        def observed_step(*args, **kwargs):
            result = original_step(*args, **kwargs)
            _SENSOR.on_physics_step(float(sim.get_physics_dt()))
            return result
        sim.step = observed_step
        sim._ball_contact_observer = True


def consume_contacts(records):
    """Read native sensor data after every physics substep, debounce contact episodes."""
    from pxr import PhysicsSchemaTools
    board = False
    for record in records:
        a = str(PhysicsSchemaTools.intToSdfPath(int(record['body0'])))
        b = str(PhysicsSchemaTools.intToSdfPath(int(record['body1'])))
        position = record['position']
        normal = record['normal']
        p = [position[k] for k in ('x','y','z')]
        n = [normal[k] for k in ('x','y','z')]
        _STATE['contact_log'].append((record['time'],a,b,p,n,record.get('impulse')))
        if len(_STATE['contact_log']) > 30:
            del _STATE['contact_log'][:-30]
        if 'ball' not in a.split('/') and 'ball' not in b.split('/'):
            continue
        other = b if 'ball' in a.split('/') else a
        if 'board' in other.split('/') and board_top_contact(p,n):
            board = True
        elif 'LeftArm' in other.split('/') or 'RightArm' in other.split('/'):
            _STATE['contacts'].append('hand')
    queue_board_episode(board)


def queue_board_episode(touching):
    if touching and not _STATE['board_touch']:
        _STATE['contacts'].append('board')
    _STATE['board_touch'] = bool(touching)


def board_top_contact(position, normal):
    p, n = np.asarray(position,float), np.asarray(normal,float)
    # The solid cup base covers this disk. Its 1 mm floor lies inside PhysX's
    # 2 mm contact margin, so the lower board can report a non-exposed contact
    # while the ball rests on the cup floor. This is not a board bounce.
    exposed = np.linalg.norm(p[:2]-np.asarray(CUP_XY)) > CUP_BASE_R
    return bool(exposed and abs(p[0]-.29) <= .18 and abs(p[1]+.15) <= .13
                and abs(p[2]-BOARD_TOP) <= .002 and abs(n[2]) >= .95)


def clear_of_ball(pos):
    env = mk.env()
    if env is None:
        return True
    return all(np.linalg.norm(np.asarray(p)-pos) > .055 for p in finger_points(env))


def inside(pos):
    z = pos[2] - BOARD_TOP
    # Exact inner conical surface: (.029,.001) to (.0465,.115).
    # A sphere tangent to this wall has normal distance R, so its radial
    # clearance is R*sqrt(1+slope²), rather than an arbitrary inward margin.
    slope = (.0465-.029)/(.115-.001)
    wall_r = .029+slope*(z-.001)
    centre_limit = wall_r-RADIUS*math.sqrt(1+slope*slope)
    return bool(.0205 <= z <= .095 and np.linalg.norm(np.asarray(pos[:2])-CUP_XY) <= centre_limit)



def update(pos, vel, clear=True, contacts=()):
    """20 Hz release/separation protocol; native contacts forbid recontact."""
    p, v = np.asarray(pos,float), np.asarray(vel,float)
    S = _STATE
    S['steps'] += 1
    if p[2] < TOP-.01:
        S['invalid'] = True
    # A complete 50 ms control interval without robot contact confirms release.
    # The 1 mm board gap excludes supported handling, rather than waiting for an
    # arbitrary launch height or requiring impossible clearance at release.
    had_hand = 'hand' in contacts
    S['handled'] = S['handled'] or had_hand
    if (not S['released'] and S['handled'] and not had_hand
            and p[2]-RADIUS > BOARD_TOP+.001 and not inside(p)):
        S['released'] = True
        S['release_step'] = S['steps']-1  # beginning of confirming interval
    if S['released']:
        if had_hand:
            S['invalid'] = True
        if S['separated'] and not clear:
            S['invalid'] = True
        if clear:
            S['separated'] = True
        # Four 50 ms intervals, including the confirming interval. Recontact is
        # forbidden throughout; this allowance is only for opening/withdrawal.
        if S['steps']-S['release_step'] >= 4 and not S['separated']:
            S['invalid'] = True
    for contact in contacts:
        if S['released']:
            if contact == 'board':
                S['bounces'] += 1
            elif contact == 'hand':
                S['invalid'] = True
    if S['bounces'] > 1:
        S['invalid'] = True
    S['prev'] = (p.copy(),v.copy())
    if inside(p) and clear and S['bounces']==1 and not S['invalid']:
        S['history'].append(p.copy())
    else:
        S['history'].clear()

@mk.on_step
def _step(environment):
    from yam_arena.scene import object_states
    b = object_states(environment,['ball'])['ball']
    contacts = list(_STATE['contacts'])
    _STATE['contacts'].clear()
    update(b.pos,b.lin_vel,clear_of_ball(b.pos),contacts)

def still():
    h = _STATE['history']
    return len(h)==20 and max(np.linalg.norm(p-h[0]) for p in h) <= .001

def success(objects):
    return bool(not _STATE['invalid'] and _STATE['bounces']==1 and inside(objects['ball'].pos) and clear_of_ball(objects['ball'].pos) and still())

def score(objects):
    if _STATE['invalid']:
        return 0.0
    if success(objects) and still():
        return 1.0
    if inside(objects['ball'].pos) and clear_of_ball(objects['ball'].pos) and _STATE['bounces']==1:
        return .75
    return .25 if _STATE['bounces']==1 else 0.0

def distance(objects):
    return float(np.linalg.norm(objects['ball'].pos-np.array([*CUP_XY,BOARD_TOP+.021])))

def demo():
    return {}
