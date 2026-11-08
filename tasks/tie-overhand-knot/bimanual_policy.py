"""Loop-only development pilot: support the standing strand near the bight.

Every change is an ordinary arm command. Measured geometry selects a route and
can stop the attempt; it never changes the cord, contacts, joints or forces.
This controller is an unvalidated feasibility probe until a complete physical
recording passes the independent scorer and visual review.
"""
from __future__ import annotations
import math
import numpy as np
from config import CONFIG
import telemetry

READY_STAGES = frozenset({'loop'})
PHASE_STATUS = ('Terminal-grasp pilots formed a crossing but no threadable arch. '
                'An interior standing-strand grasp is under test; threading, '
                'tightening and release remain unvalidated.')


class _StopAttempt(Exception):
    pass


def _event(label, **values):
    telemetry.demo_events.append(dict(label=label, time_s=float(telemetry.latest_sample['time']),
                                      privileged_demo_only=True, **values))


def _stop(reason, **values):
    _event('bimanual attempt stopped', reason=reason, **values)
    raise _StopAttempt(reason)


def _sample():
    sample = telemetry.latest_sample
    if sample['metrics'].get('invalid'):
        _stop('recorded cord geometry became invalid', invalid=sample['metrics']['invalid'])
    return sample


def _drive(motion):
    for _ in motion:
        yield
        _sample()  # Check every captured interval, including during long moves.


def _point(index):
    return np.asarray(_sample()['cord']['points'][index], float)


def _tcp(side):
    return np.asarray(_sample()['hands'][side]['tcp'], float)


def _held(side, index, *, require_loaded=False):
    sample = _sample(); hand = sample['hands'][side]
    local = np.asarray(hand['tcp_rotation'], float).T @ (_point(index)-_tcp(side))
    half_gap = float(hand['opening'])*.04695
    radius = float(CONFIG['radius'])
    # These are pad-overlap witnesses, not a force or grasp measurement.
    valid = (radius*.85 < half_gap < radius+.0015
             and abs(local[0]) < max(.0005,half_gap-radius+.001)
             and abs(local[1]) < .011
             and -.010 < local[2] < .016 and np.linalg.norm(local) < .026)
    if require_loaded:
        valid = valid and half_gap > radius*.9+.0001
    _event('bimanual grasp witness', side=side, material_index=index,
           local_point_m=local.tolist(), measured_half_gap_m=half_gap, passed=bool(valid))
    if not valid:
        _stop('selected material is not retained in the measured pad region', side=side, material_index=index)


def _move(arm, side, point, rotation, seconds, label, *, opening=None, held_index=None):
    goal = np.asarray(point, float)
    yield from _drive(arm.move_pose(goal, rotation, seconds, opening=opening, label=label))
    error = float(np.linalg.norm(_tcp(side)-goal))
    if error > .012:
        _stop('actual TCP did not reach the intended waypoint', motion_label=label, tracking_error_m=error)
    if held_index is not None:
        _held(side, held_index)


def _pick(arm, side, index, down, open_value, pinch):
    p = _point(index)
    yield from _move(arm, side, p+[0,0,.11], down, 2.5, 'approach selected strand', opening=open_value)
    p = _point(index)
    yield from _move(arm, side, p+[0,0,.009], down, 2., 'descend to selected strand')
    yield from _drive(arm.grip(pinch, 1.))
    _held(side, index, require_loaded=True)


def _crossing(reference=None, *, material_start=3, working_over_only=False):
    """Find the large projected bight, allowing its crossing to migrate."""
    p = np.asarray(_sample()['cord']['polyline'], float)
    found = []
    # Material from the initial bend can become part of either strand. Search
    # all interior material, excluding local neighbours and the short tips.
    for i in range(max(3,material_start), len(p)-8):
        for j in range(i+4, len(p)-4):
            u, v = p[i+1]-p[i], p[j+1]-p[j]
            matrix = np.column_stack((u[:2], -v[:2]))
            if abs(np.linalg.det(matrix)) < 1e-9:
                continue
            a,b=np.linalg.solve(matrix,p[j,:2]-p[i,:2])
            if 0 <= a <= 1 and 0 <= b <= 1:
                first,second=p[i]+a*u,p[j]+b*v
                found.append(dict(position=((first+second)/2).tolist(),
                                  standing_index=i, working_index=j,
                                  standing_material=float(i+a), working_material=float(j+b),
                                  standing_point=first.tolist(), working_point=second.tolist(),
                                  working_above_m=float(second[2]-first[2])))
    if working_over_only:
        found = [c for c in found if c['working_above_m'] > 0]
    if reference is not None:
        found=[c for c in found
               if abs(c['standing_material']-reference['standing_material']) <= 6
               and abs(c['working_material']-reference['working_material']) <= 6
               and c['working_above_m'] > 0]
    if not found:
        _stop('no projected crossing found' if reference is None else
              'no over-crossing found near the reference material')
    if reference is not None:
        return min(found,key=lambda c:abs(c['standing_material']-reference['standing_material'])
                   +abs(c['working_material']-reference['working_material']))
    # The intended bight encloses more cord than incidental terminal curls.
    return max(found,key=lambda c:c['working_material']-c['standing_material'])


def _inside_distance(point, polygon):
    """Even-odd inside test and distance to the actual projected loop boundary."""
    point=np.asarray(point,float);a=polygon[:-1];b=polygon[1:]
    edge=b-a;den=np.einsum('ij,ij->i',edge,edge)
    t=np.clip(np.einsum('ij,ij->i',point-a,edge)/np.maximum(den,1e-16),0,1)
    distance=float(np.linalg.norm(a+t[:,None]*edge-point,axis=1).min())
    hits=(a[:,1]>point[1]) != (b[:,1]>point[1])
    crossing_x=a[hits,0]+(point[1]-a[hits,1])*(b[hits,0]-a[hits,0])/(b[hits,1]-a[hits,1])
    return bool(np.count_nonzero(crossing_x>point[0])%2),distance


def _arch_measurement(crossing, feed_x, lift_x):
    """Report two candidate corridors; this is not a validated thread route."""
    p=np.asarray(_sample()['cord']['polyline'],float)
    s=int(crossing['standing_index']);w=int(crossing['working_index'])
    if not 3 <= s < w < len(p)-3:
        _stop('crossing material order is inconsistent')
    # The bight runs from standing material s to working material w. Feed on
    # its far-x side, not under the exterior lead adjacent to the yellow tip.
    boundary=np.vstack((crossing['standing_point'],p[s+1:w+1],crossing['working_point']))[:,:2]
    # Entry under the standing arc and a later whole-hand lift need different
    # positions. The palm may still be outside during entry; it must be inside
    # before lifting through the loop. These are measurements, not commands.
    entry_inside,entry_distance=_inside_distance([feed_x,0.],boundary)
    checks=[_inside_distance([x,y],boundary) for x in np.linspace(lift_x-.145,lift_x+.025,7)
            for y in np.linspace(-.055,.055,5)]
    interior=all(inside and distance >= CONFIG['radius']+.005 for inside,distance in checks)
    # Only the standing arc on the bight side may provide the tunnel ceiling.
    # The exterior strand running back to the left gripper is excluded.
    midpoint=int((crossing['standing_material']+crossing['working_material'])/2)
    standing=np.vstack((crossing['standing_point'],p[s+1:midpoint+1]))
    nearby=standing[(standing[:,0] >= feed_x-.145)&(standing[:,0] <= feed_x+.025)]
    tunnel=(float(nearby[:,2].min()-CONFIG['radius']-CONFIG['table_top'])
            if len(nearby)>=3 else None)
    width=float(p[s:w+2,1].max()-p[s:w+2,1].min()-2*CONFIG['radius'])
    _event('bimanual arch measurement', crossing=crossing, feed_x_m=feed_x, lift_x_m=lift_x,
           standing_to_table_height_m=tunnel, loop_y_extent_m=width,
           entry_centre_inside_projected_bight=entry_inside,
           entry_projected_boundary_distance_m=entry_distance,
           complete_hand_lift_inside_projected_bight=interior,
           minimum_lift_projected_boundary_distance_m=min(distance for _,distance in checks),
           measured_entry_height_at_least_120mm=(tunnel is not None and tunnel >= .12),
           route_validated=False)


def _sequence(arms, stage):
    from yam_arena.constants import FINGER_OPEN
    from demo_policy import wait
    left,right=arms; n=len(_sample()['cord']['points']); li,ri=30,n-3
    radius=float(CONFIG['radius']);pinch=-radius*.9
    # Support the bight directly instead of lifting a long external lead.
    # The left pickup is on the U's bend, so align the fingers across its
    # measured tangent. The right still grasps the straight terminal strand.
    down_right=np.array([[0.,-1.,0.],[-1.,0.,0.],[0.,0.,-1.]])
    nodes=np.asarray(_sample()['cord']['polyline'],float)
    tangent=nodes[li+1]-nodes[li]
    yaw=math.atan2(tangent[1],tangent[0]); cy,sy=math.cos(yaw),math.sin(yaw)
    down_left=np.array([[cy,-sy,0.],[sy,cy,0.],[0.,0.,1.]]) @ down_right
    s,c=math.sin(math.radians(20)),math.cos(math.radians(20))
    forward=np.array([[0.,-s,c],[-1.,0.,0.],[0.,-c,-s]])
    _event('bimanual candidate started', stage=stage, left_material_index=li, right_material_index=ri)
    yield from _pick(left,0,li,down_left,FINGER_OPEN,pinch)
    yield from _pick(right,1,ri,down_right,FINGER_OPEN,pinch)
    yield from _move(left,0,[.20,.24,.90],down_right,3.,'lift standing strand',held_index=li)
    yield from _move(left,0,[.20,.24,.90],forward,2.,'orient standing hand horizontally',held_index=li)
    yield from _move(left,0,[.17,.33,1.04],forward,3.,'raise standing strand into an arch',held_index=li)
    yield from _move(right,1,[.25,-.14,.90],forward,4.,'lift and orient working end',held_index=ri)
    yield from _move(right,1,[.38,-.14,.90],forward,3.,'approach overpass',held_index=ri)
    yield from _move(right,1,[.38,.22,.90],forward,4.,'move working end beside standing branch',held_index=ri)
    # Lift before translating: the diagonal shortcut narrowly passes the
    # opposite forearm. This probe tests whether the table-level crossing
    # rises with the standing branch while both selected grasps remain held.
    yield from _move(left,0,[.17,.33,1.12],forward,2.,
                     'lift standing hand above opposite forearm',held_index=li)
    yield from _move(left,0,[.30,.33,1.12],forward,3.,
                     'move standing hand forward to raise the arch',held_index=li)
    # The interior grip moves the standing branch away from the old crossing
    # route. First reposition it, then cross at a reachable working-hand X.
    yield from _move(right,1,[.26,.22,.90],forward,3.,
                     'cross the relocated standing branch',held_index=ri)
    _held(0,li)
    crossing=_crossing(material_start=li+1)
    if abs(crossing['working_above_m']) < 2*radius:
        _stop('projected crossing has less than one diameter vertical separation',crossing=crossing)
    _event('bimanual first crossing',crossing=crossing,
           working_above=bool(crossing['working_above_m']>0))
    # Measure again after settling. A real crossing can slide along the cord:
    # total material drift alone must not be reported as a lost crossing.
    # This fresh largest-bight measurement does not certify continuity.
    yield from _drive(wait(1.))
    settled=_crossing(material_start=li+1)
    _event('bimanual settled crossing',crossing=settled,
           material_drift=[settled[k]-crossing[k]
                           for k in ('standing_material','working_material')],
           continuity_certified=False)
    entry_x=max(.35,float(settled['position'][0])+.06)
    lift_x=max(.53,float(settled['position'][0])+.18)
    _arch_measurement(settled,entry_x,lift_x)
    _event('bimanual arch probe finished',claimed_score=None,
           threading_validated=False, tightening_validated=False)



def tie_overhand(arms, stage='demo'):
    """Run the loop-only development pilot; later phases are unavailable."""
    if stage not in ('demo','loop','thread'):
        raise ValueError('bimanual stage must be demo, loop or thread')
    if stage not in READY_STAGES:
        raise NotImplementedError('Bimanual threading and tightening are not validated; only mode=loop is available.')
    try:
        yield from _sequence(arms,stage)
    except _StopAttempt:
        return
