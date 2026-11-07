"""Separate the nested empty trays and park both upright in their outlined bays.

The blue outer tray starts beneath the orange inner tray. Grasp the raised tabs.
Lift the inner tray clear of the rim before moving sideways. Release both trays.
"""
from pathlib import Path
import json
from scoring import TrialScorer
import numpy as np
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.constants import FINGER_OPEN as OPEN, FINGER_CLOSED as CLOSED

NAME = 'denest-trays'
INSTRUCTION = json.loads(Path(__file__).with_name('task.json').read_text())['instruction']
TOP = 0.7475
HZ = 20
MAX_STEPS = 800
SUCCESS_HOLD_S = 2.0
START = {'outer': (0.25, 0.0, TOP + .002), 'inner': (0.25, 0.0, TOP + .009)}
GOALS = {'outer': (0.25, .23), 'inner': (0.25, -.23)}
# Each rigid tray is a base, four walls, a bridge and a raised grasp tab.
PARTS = {
    'outer': [((0,0,.0025),(.16,.18,.005)),
              ((-.0775,0,.025),(.005,.18,.045)),((.0775,0,.025),(.005,.18,.045)),
              ((0,-.0875,.025),(.15,.005,.045)),((0,.0875,.025),(.15,.005,.045)),
              ((0,.11,.045),(.032,.05,.008)),((0,.135,.06),(.032,.032,.04))],
    'inner': [((0,0,.002),(.13,.14,.004)),
              ((-.063,0,.019),(.004,.14,.034)),((.063,0,.019),(.004,.14,.034)),
              ((0,-.068,.019),(.122,.004,.034)),((0,.068,.019),(.122,.004,.034)),
              ((0,-.09,.055),(.032,.048,.008)),((0,-.11,.075),(.032,.032,.04))],
}
MASS = {'outer': .16, 'inner': .10}
COLORS = {'outer': (.12,.38,.68), 'inner': (.95,.40,.10)}

def bounds(name):
    lo = np.min([np.array(p)-np.array(s)/2 for p,s in PARTS[name]],axis=0)
    hi = np.max([np.array(p)+np.array(s)/2 for p,s in PARTS[name]],axis=0)
    return lo,hi

def make_environment(embodiment, device, control_hz=HZ):
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.scene.scene import Scene
    from isaaclab_arena.environments.isaaclab_arena_environment import IsaacLabArenaEnvironment
    from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
    from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.scene import task_room, task_object, env_cfg_callback
    from yam_arena.task_file import Box
    objects = task_room('station', embodiment.rig.top_cam)
    for name in PARTS:
        objects.append(Object(name=name, usd_path=str(Path(__file__).parent/'assets'/f'{name}.usda'),
                              object_type=ObjectType.RIGID, initial_pose=Pose(position_xyz=START[name])))
        x,y=GOALS[name]
        # Thin raised lines delineate a bay without an invisible trigger volume or a fitted socket.
        for axis,delta,size in [('x',-.095,(.003,.21,.001)),('x',.095,(.003,.21,.001)),
                                ('y',-.105,(.19,.003,.001)),('y',.105,(.19,.003,.001))]:
            pos=(x+delta,y,TOP+.0005) if axis=='x' else (x,y+delta,TOP+.0005)
            objects.append(task_object(Box(f'{name}_{axis}_{"p" if delta>0 else "n"}',pos=pos,size=size,color=COLORS[name])))
    cfg=IsaacLabArenaEnvironment(name='yam_denest_trays',scene=Scene(objects),embodiment=embodiment,
                                env_cfg_callback=env_cfg_callback(control_hz))
    return ArenaEnvBuilder(cfg,ArenaEnvBuilderCfg(num_envs=1,solve_relations=False,device=device)).make_registered()

def _state_metrics(objects, root_poses, *, clear, dropped):
    details={}
    for name,s in objects.items():
        center=np.asarray(root_poses[name],dtype=float)[:3]
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError('Finite world root poses are required for scoring')
        q=s.quat_xyzw
        upright=bool(q[0]**2+q[1]**2 < np.sin(np.deg2rad(8)/2)**2)
        yaw=np.arctan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]**2+q[2]**2))
        yaw_ok=bool(abs(yaw) < np.deg2rad(10))
        gap=float(s.low[2]-TOP)
        safe=bool(gap>=-.008 and s.high[2]<TOP+.35 and upright and yaw_ok)
        rest=bool(s.resting(max_speed=.01,max_spin=.1))
        grounded=bool(-.005<=gap<=.008)
        error=float(np.linalg.norm(center[:2]-GOALS[name]))
        details[name]={'center_xy':center[:2].tolist(),'bay_error_m':error,'bottom_gap_m':gap,
                       'safe':safe,'resting':rest,'grounded':grounded,
                       'parked':bool(safe and rest and grounded and error<=.02)}
    a,b=objects['outer'],objects['inner']
    separated=bool(a.high[1]+.01<b.low[1] or b.high[1]+.01<a.low[1] or
                   a.high[0]+.01<b.low[0] or b.high[0]+.01<a.low[0])
    valid=all(d['safe'] and d['resting'] and d['grounded'] for d in details.values())
    score=0.0
    if valid and separated and clear and not dropped:
        score=.5
        if all(d['parked'] for d in details.values()): score=1.0
    return {'objects':details,'separated':separated,'score':score,'success':score==1.0}

def make_scorer():
    return TrialScorer(_state_metrics, TOP, HZ, SUCCESS_HOLD_S, OPEN)

def demo():
    ox,oy,oz=START['outer']; ix,iy,iz=START['inner']
    op=(ox,oy+.135,oz+.06); ip=(ix,iy-.11,iz+.075)
    high=TOP+.16
    # First clamp the outer tray by its tab. Hold it while the inner tray clears the rim.
    left=[Waypoint(1.5,(op[0],op[1],high),OPEN,'approach outer'),
          Waypoint(1.5,op,OPEN,'lower to outer tab'),Waypoint(1,op,CLOSED,'hold outer'),
          Waypoint(13,op,CLOSED,'stabilize outer')]
    dest=(GOALS['outer'][0],GOALS['outer'][1]+.135,TOP+.06+.003)
    left += [Waypoint(1.5,(op[0],op[1],high),CLOSED,'lift outer'),
             Waypoint(2,(dest[0],dest[1],high),CLOSED,'park outer'),
             Waypoint(1.5,dest,CLOSED,'lower outer'),Waypoint(1,dest,OPEN,'release outer'),
             Waypoint(1.5,(dest[0],dest[1],high),OPEN,'withdraw left')]
    dest=(GOALS['inner'][0],GOALS['inner'][1]-.11,TOP+.075+.003)
    right=[Waypoint(1.5,(ip[0],ip[1],high),OPEN,'approach inner'),
           Waypoint(3,(ip[0],ip[1],high),OPEN,'wait for outer support')]
    right += pick_place_waypoints(ip,dest,travel_z=high,speed=.8)
    return {'left':left,'right':right}
