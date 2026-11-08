"""Table-supported knot development through ordinary robot joint commands.

Measured cord positions guide the privileged demonstration, never the physics.
Each stage records what happened; completion must be established by replay.
"""
from __future__ import annotations
import numpy as np
from config import CONFIG
from bimanual_policy import (_StopAttempt, _sample, _event, _stop, _drive,
                             _point, _tcp, _held, _move, _pick, _crossing,
                             _inside_distance)

IMPLEMENTED_STAGES = frozenset({'loop', 'thread', 'demo'})
DOWN = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
_ANGLE = np.deg2rad(20.)
FORWARD = np.array([[-1., 0., 0.], [0., np.sin(_ANGLE), -np.cos(_ANGLE)],
                    [0., -np.cos(_ANGLE), -np.sin(_ANGLE)]])


def _loop_boundary(crossing):
    p = np.asarray(_sample()['cord']['polyline'], float)
    s, w = crossing['standing_index'], crossing['working_index']
    return np.vstack((crossing['standing_point'], p[s+1:w+1],
                      crossing['working_point'], crossing['standing_point']))


def _measure_arch(crossing, held_index):
    p = np.asarray(_sample()['cord']['points'], float)
    boundary = _loop_boundary(crossing)
    _event('table local arch measurement', crossing=crossing,
           held_index=held_index,
           cord_surface_height_at_grasp_m=float(p[held_index, 2]-CONFIG['radius']-CONFIG['table_top']),
           projected_loop_extent_m=np.ptp(boundary[:, :2], axis=0).tolist(),
           fraction_of_cord_within_15mm_of_table=float(np.mean(p[:, 2]-CONFIG['radius']-CONFIG['table_top'] < .015)),
           actual_end_tip_m=_sample()['cord']['end_tip'],
           threading_validated=False)


def _tip_location(label):
    """Observe the actual terminal relative to the original over-crossing.

    This is a controller diagnostic, not a topology certificate. In particular,
    a tip inside the top-view loop may have arrived over the rim instead of under.
    """
    crossing = _crossing(working_over_only=True)
    boundary = _loop_boundary(crossing)
    tip = np.asarray(_sample()['cord']['end_tip'], float)
    inside, distance = _inside_distance(tip[:2], boundary[:, :2])
    axis = np.asarray(_sample()['cord']['endpoints'], float)[-1, :, :2]
    axis_inside, axis_distance = _axis_loop_clearance(axis, boundary[:, :2])
    result = dict(tip_m=tip.tolist(), inside=inside, boundary_distance_m=distance,
                  axis_inside=axis_inside, axis_boundary_distance_m=axis_distance,
                  crossing=crossing)
    _event(label, **result)
    return result


def _axis_loop_clearance(axis, boundary):
    """Check the whole projected terminal, including concave loop boundaries."""
    from topology import _pair_distances
    inside = all(_inside_distance(p, boundary)[0] for p in axis)
    if np.linalg.norm(axis[1]-axis[0]) < 1e-8:
        return inside, _inside_distance(axis[0], boundary)[1]
    a, b = boundary[:-1], boundary[1:]
    keep = np.linalg.norm(b-a, axis=1) > 1e-8
    a, b = a[keep], b[keep]
    distance = float(_pair_distances(np.repeat(axis[:1], len(a), axis=0),
        np.repeat(axis[1:], len(a), axis=0), a, b).min())
    return bool(inside and distance > 1e-9), distance


def _form_table_loop(arms):
    from yam_arena.constants import FINGER_OPEN
    from demo_policy import wait
    left, right = arms
    pinch = -CONFIG['radius']*.9
    ri, li = len(_sample()['cord']['points'])-3, 23
    _event('table method started', local_support_material=li, crossing_grasp_material=ri)
    yield from _move(left, 0, [.16, .40, .90], DOWN, 3.,
                     'park empty left hand clear of crossing route', opening=FINGER_OPEN)
    yield from _pick(right, 1, ri, DOWN, FINGER_OPEN, pinch)
    p = _point(ri)
    yield from _move(right, 1, p+[0, 0, .079], DOWN, 2.,
                     'lift working end clear of table', held_index=ri)
    yield from _move(right, 1, [.20, .16, .7915], DOWN, 4.,
                     'cross working end over standing strand', held_index=ri)
    yield from _move(right, 1, [.20, .16, .7735], DOWN, 2.,
                     'lay crossing on table', held_index=ri)
    yield from _drive(right.grip(FINGER_OPEN, 1.))
    yield from _move(right, 1, [.20, .16, .7915], DOWN, 2., 'withdraw from laid crossing')
    yield from _move(right, 1, [.25, -.23, .86], DOWN, 3., 'park working hand')
    crossing = _crossing()
    _event('table laid crossing', crossing=crossing)
    return (yield from _raise_right_arch(right, li, crossing))


def _raise_right_arch(right, index, crossing):
    from yam_arena.constants import FINGER_OPEN
    from demo_policy import wait
    point = _point(index)
    if np.linalg.norm(point[:2]-[.35338, .05294]) > .025:
        _stop('standing strand moved outside the local pickup region', point_m=point.tolist())
    if not crossing['standing_material'] < index < crossing['working_material']:
        _stop('support grasp is not on the bight side of the crossing', crossing=crossing)
    yield from _move(right, 1, point+[0, 0, .0215], DOWN, 4.,
                     'low approach to local arch support', opening=FINGER_OPEN)
    point = _point(index)
    yield from _move(right, 1, point+[0, 0, .009], DOWN, 2., 'grasp standing strand near crossing')
    yield from _drive(right.grip(-CONFIG['radius']*.9, 1.))
    _held(1, index, require_loaded=True)
    point = _point(index)
    low_axis = point+[0, 0, .02]
    yield from _move(right, 1, low_axis+[0, 0, .009], DOWN, 2.,
                     'lift support clear of table', held_index=index)
    angle = np.deg2rad(20.)
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[1., 0., 0.], [0., c, -s], [0., s, c]])@DOWN
    yield from _move(right, 1, low_axis-rotation[:, 2]*.009, rotation, 2.,
                     'tilt supporting wrist clear of feeding arm', held_index=index)
    yield from _move(right, 1, point+[0, 0, .15]-rotation[:, 2]*.009, rotation, 4.,
                     'open a local table-supported arch', held_index=index)
    yield from _drive(wait(1.))
    crossing = _crossing(working_over_only=True)
    _measure_arch(crossing, index)
    return dict(crossing=crossing, standing_index=index)


def _thread_held_terminal(arms):
    """Carry the actual red terminal under the arch with the left hand."""
    from yam_arena.constants import FINGER_OPEN
    from demo_policy import wait
    left, right = arms
    index = len(_sample()['cord']['points'])-1
    point = _point(index)
    if np.linalg.norm(point-[.17271, .16314, .7525]) > .025:
        _stop('red terminal moved outside the audited pickup region', point_m=point.tolist())
    yield from _move(left, 0, point+[0, 0, .11], DOWN, 3.,
                     'approach actual red terminal', opening=FINGER_OPEN)
    point = _point(index)
    yield from _move(left, 0, point+[0, 0, .009], DOWN, 2., 'descend to actual red terminal')
    yield from _drive(left.grip(-CONFIG['radius']*.9, 1.))
    _held(0, index, require_loaded=True)
    yield from _move(left, 0, _point(index)+[0, 0, .069], DOWN, 3.,
                     'lift held terminal clear of table', held_index=index)
    yield from _move(left, 0, [.37, .20, .83], DOWN, 4.,
                     'carry terminal outside the arch', held_index=index)
    forward = FORWARD
    yield from _move(left, 0, [.37, .20, .83], forward, 3.,
                     'face the held terminal toward the opening', held_index=index)
    yield from _move(left, 0, [.37, .20, .765], forward, 2.,
                     'lower held terminal outside arch', held_index=index)
    _tip_location('table held terminal before underpass')
    for y, seconds in ((.14, 2.), (.10, 1.5), (.05, 2.), (.01, 2.)):
        yield from _move(left, 0, [.37, y, .765], forward, seconds,
                         'carry actual terminal under arch', held_index=index)
        _held(1, 23)
        _tip_location('table held terminal during underpass')
    result = _tip_location('table held terminal inside loop')
    if not result['inside'] or result['boundary_distance_m'] < .025:
        _stop('held terminal has not reached a clear loop interior', tip=result)
    yield from _drive(wait(1.))
    yield from _drive(left.grip(-.012, 1.))
    yield from _drive(wait(1.))
    result = _tip_location('table terminal after opening feeding hand')
    # Two cord radii plus 2 mm of surface clearance is a routing margin,
    # not a knot or physics-validity certificate.
    if not result['axis_inside'] or result['axis_boundary_distance_m'] < 2*CONFIG['radius']+.002:
        _stop('released terminal lacks loop clearance for withdrawal', tip=result)
    yield from _move(left, 0, [.37, .20, .765], forward, 4.,
                     'withdraw empty feeding hand along the entry path')
    yield from _move(left, 0, [.37, .20, .86], forward, 2.,
                     'lift empty hand outside the loop')
    yield from _move(left, 0, [.37, .20, .86], DOWN, 2.,
                     'turn empty feeding hand above the table', opening=FINGER_OPEN)
    yield from _move(left, 0, [.16, .40, .90], DOWN, 3., 'park empty feeding hand')
    _held(1, 23)
    support_rotation = np.asarray(_sample()['hands'][1]['tcp_rotation'], float)
    lowering_goal = _tcp(1)-[0, 0, .14]
    yield from _drive(right.move_pose(lowering_goal, support_rotation, 3.,
                                     label='lower the supported arch toward the table'))
    _check_lowered_support(lowering_goal)
    yield from _drive(right.grip(-.012, 1.))
    release_rotation = np.asarray(_sample()['hands'][1]['tcp_rotation'], float)
    yield from _move(right, 1, _tcp(1)+[0, 0, .03], release_rotation, 1.5,
                     'withdraw the open supporting hand vertically')
    yield from _drive(right.grip(FINGER_OPEN, 1.))
    yield from _move(right, 1, [.25, -.23, .94], DOWN, 4., 'clear the supporting hand')
    yield from _drive(right.grip(FINGER_OPEN, .5))
    _event('table terminal after empty-hand withdrawal',
           actual_end_tip_m=_sample()['cord']['end_tip'],
           yellow_point_m=_point(2).tolist(), metrics=_sample()['metrics'])
    yield from _recover_dropped_terminal(arms, index)
    _event('table held-terminal passage measured', material_index=index,
           held_side=1, metrics=_sample()['metrics'], completed_knot_claimed=False)
    return index


def _check_lowered_support(goal):
    """Opening needs a low, retained strand, not an exact vertical TCP target."""
    _held(1, 23)
    horizontal_error = float(np.linalg.norm((_tcp(1)-goal)[:2]))
    heights = (np.asarray(_sample()['cord']['endpoints'][23], float)[:, 2]
               -CONFIG['radius']-CONFIG['table_top'])
    lowest, highest = float(heights.min()), float(heights.max())
    ready = horizontal_error <= .012 and 0 <= lowest <= highest <= .025
    _event('table support lowering measured', horizontal_error_m=horizontal_error,
           support_surface_height_range_m=[lowest, highest], ready_to_open=bool(ready))
    if not ready:
        _stop('support strand is not positioned for a low release',
              horizontal_error_m=horizontal_error, support_surface_height_range_m=[lowest, highest])


def _terminal_down(index, preferred_yaw):
    """Align the pad slot with the measured terminal capsule, after settling."""
    ends = np.asarray(_sample()['cord']['endpoints'], float)[index]
    axis = ends[1]-ends[0]
    if np.linalg.norm(axis[:2]) < .003:
        _stop('terminal is too vertical for the planned top-down grasp', material_index=index)
    yaw = np.arctan2(axis[1], axis[0])-np.pi
    yaw += np.round((preferred_yaw-yaw)/np.pi)*np.pi
    z = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                  [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    return z @ DOWN


def _recover_dropped_terminal(arms, index):
    point = _point(index)
    if not (.33 <= point[0] <= .41 and -.10 <= point[1] <= .005 and .752 <= point[2] <= .79):
        _stop('dropped red terminal outside the regrasp region', point_m=point.tolist())
    rotation = _terminal_down(index, np.pi/2)
    # A wide approach sweeps the neighbouring loop strand while closing.
    yield from _pick(arms[1], 1, index, rotation, -.012, -CONFIG['radius']*.9)


def _pull_and_release(arms, working_index):
    """Capture both ends before moving either toward the pulling position."""
    from yam_arena.constants import FINGER_OPEN
    from demo_policy import both, wait
    left, right = arms
    _held(1, working_index)
    yellow = _point(2)
    if not (.18 <= yellow[0] <= .29 and -.09 <= yellow[1] <= .06 and .752 <= yellow[2] <= .79):
        _stop('yellow terminal outside the regrasp region', point_m=yellow.tolist())
    rotation = _terminal_down(2, 0.)
    yield from _move(left, 0, yellow+[0, .12, .020], rotation, 3.,
                     'approach yellow end from the clear side', opening=-.012)
    yield from _move(left, 0, _point(2)+[0, 0, .020], rotation, 2.,
                     'low approach to yellow end beside the held red end')
    yield from _move(left, 0, _point(2)+[0, 0, .009], rotation, 2.,
                     'descend to yellow end')
    yield from _drive(left.grip(-CONFIG['radius']*.9, 1.))
    _held(0, 2, require_loaded=True)
    _held(1, working_index)
    held_rotation = np.asarray(_sample()['hands'][1]['tcp_rotation'], float)
    yield from _move(right, 1, _tcp(1)+[0, 0, .065], held_rotation, 2.,
                     'lift captured red tip through the loop', held_index=working_index)
    _held(0, 2)
    outward = _tcp(1).copy()
    outward[0] = .39
    yield from _move(right, 1, outward, held_rotation, 2.,
                     'clear the other arm before moving the red end south', held_index=working_index)
    _held(0, 2)
    # Increase normal jaw pressure gradually as both ends are lifted. Keep the
    # earlier threading and red-end clearance motions at their original pinch.
    yield from both(
        _move(left, 0, [.24, .18, .94], DOWN, 6.,
              'raise and separate the yellow end', opening=0., held_index=2),
        _move(right, 1, [.24, -.18, .94], DOWN, 6.,
              'raise and separate the red end', opening=0., held_index=working_index))
    _held(0, 2)
    _held(1, working_index)
    angle = np.deg2rad(25.)
    z = np.array([[np.cos(angle), -np.sin(angle), 0.],
                  [np.sin(angle), np.cos(angle), 0.], [0., 0., 1.]])
    rotations = (z @ np.diag([-1., 1., -1.]),
                 z.T @ np.diag([1., -1., -1.]))
    yield from _drive(both(*[arm.move_pose([.24, sign*.18, .94], rotation, 3.,
        label='orient terminal grasps for simultaneous pull')
        for arm, sign, rotation in zip(arms, (1., -1.), rotations)]))
    for half_span in np.linspace(.20, .56, 13):
        yield from _drive(both(*[arm.move_pose([.24, sign*half_span, .94], rotation, 1.,
            label='pull both ends apart together')
            for arm, sign, rotation in zip(arms, (1., -1.), rotations)]))
        _held(0, 2)
        _held(1, working_index)
    if working_index == len(_sample()['cord']['points'])-1:
        for half_span in (.57, .58, .59):
            if _sample()['metrics']['span_condition']:
                break
            yield from _drive(both(*[arm.move_pose([.24, sign*half_span, .94], rotation, 1.,
                label='finish simultaneous terminal pull')
                for arm, sign, rotation in zip(arms, (1., -1.), rotations)]))
            _held(0, 2)
            _held(1, working_index)
    yield from _drive(wait(1.5))
    _event('table simultaneous pull measured', metrics=_sample()['metrics'])
    yield from _drive(both(left.grip(FINGER_OPEN, 1.), right.grip(FINGER_OPEN, 1.)))
    yield from _drive(wait(6.))
    _event('table released outcome', metrics=_sample()['metrics'])


def tie_overhand(arms, stage='demo'):
    if stage not in IMPLEMENTED_STAGES:
        raise NotImplementedError('Unknown table controller stage: '+stage)
    try:
        yield from _form_table_loop(arms)
        if stage in ('thread', 'demo'):
            index = yield from _thread_held_terminal(arms)
            if stage == 'demo':
                yield from _pull_and_release(arms, index)
    except _StopAttempt:
        return
