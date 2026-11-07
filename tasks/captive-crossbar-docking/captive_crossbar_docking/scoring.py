# SPDX-License-Identifier: MIT
"""CPU-only geometry grading from measured link poses, never controller labels.

Scores describe the current terminal outcome. Invalid measurements, a moved dock,
broken linkage, or a dropped assembly permanently invalidate the episode.
"""
from __future__ import annotations

from itertools import product
import math
import numpy as np

from . import contract as C

_EPS = 1e-9
_CONFIG = dict(centering_tolerance=.004, linkage_position_tolerance=.002,
               linkage_angle_tolerance=math.radians(2), upright_tolerance=math.radians(5),
               support_tolerance=.004, geometry_tolerance=.0005,
               max_linear_speed=.01, max_angular_speed=.1, max_joint_speed=.01,
               release_opening=.8, release_clearance=.08, release_hold_seconds=1.,
               max_hold_sample_gap=.075, drop_depth=.02,
               initial_position_tolerance=.003, initial_joint_tolerance=.003)


def config_dict():
    """Serializable thresholds plus geometry frozen in the scene contract."""
    return dict(_CONFIG, table_top=C.TABLE_TOP, dock_mouth=C.DOCK_MOUTH,
                dock_back=C.DOCK_BACK, dock_half_width=C.DOCK_HALF_WIDTH,
                dock_height=C.DOCK_HEIGHT, slide_limits=list(C.SLIDE_LIMITS),
                carrier_parts=C.CARRIER_PARTS, crossbar_parts=C.CROSSBAR_PARTS,
                payload_carrier_parts=C.PAYLOAD_CARRIER_PART_NAMES)


def _array(value, shape):
    a = np.asarray(value, dtype=float)
    if a.shape != shape or not np.isfinite(a).all():
        raise ValueError('missing, nonfinite or malformed measurement')
    return a


def _pose(value):
    if isinstance(value, dict):
        a = _array(list(value['position']) + list(value['quaternion_xyzw']), (7,))
    else:
        a = _array(value, (7,))
    p, q = a[:3], a[3:]
    if abs(np.linalg.norm(q)-1) > .001:
        raise ValueError('non-unit measured quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    r = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return p, r


def _angle(a, b):
    return float(np.arccos(np.clip((np.trace(a.T @ b)-1)/2, -1, 1)))


def part_corners(position, rotation, size, centre):
    """Eight world corners of a contract box, useful for independent CPU audits."""
    offsets = np.array(list(product((-1., 1.), repeat=3))) * np.array(size)/2
    return (offsets + np.array(centre)) @ rotation.T + position


def _boxes(pose, parts):
    p, r = pose
    return [(name, part_corners(p, r, size, centre), size, np.array(centre))
            for name, size, centre, _ in parts]


def _distance(point, pose, parts):
    p, r = pose
    local = r.T @ (point-p)
    return min(float(np.linalg.norm(np.maximum(np.abs(local-np.array(c))-np.array(s)/2, 0)))
               for _, s, c, _ in parts)


class DockingScorer:
    def __init__(self):
        self.invalid_reason = None
        self.previous = None
        self.hold_since = None

    def _invalid(self, reason):
        self.invalid_reason = reason
        self.hold_since = None
        return dict(score=0., success=False, invalid=True, invalid_reason=reason,
                    release_hold_seconds=0.)

    def update(self, record):
        if self.invalid_reason:
            return self._invalid(self.invalid_reason)
        try:
            t = float(record['t'])
            raw_step = record['step']
            step = int(raw_step)
            if not math.isfinite(t) or step != raw_step or isinstance(raw_step, bool):
                raise ValueError('invalid time or step')
            poses = {name: _pose(record['poses'][name]) for name in ('carrier', 'crossbar', 'dock')}
            velocities = {}
            for name in ('carrier', 'crossbar'):
                v = record['velocities'][name]
                if isinstance(v, dict):
                    v = list(v['linear']) + list(v['angular'])
                velocities[name] = _array(v, (6,))
            q = float(np.asarray(record['joint_position']).reshape(()))
            qv = float(np.asarray(record['joint_velocity']).reshape(()))
            if not math.isfinite(q) or not math.isfinite(qv):
                raise ValueError('nonfinite measured joint')
            tcp = {hand: _pose(record['tcp'][hand]) for hand in ('left', 'right')}
            grippers = _array([record['grippers'][h] for h in ('left', 'right')], (2,))
            if np.any(grippers < 0) or np.any(grippers > 1):
                raise ValueError('gripper opening outside [0,1]')
            if 'joints' in record:
                _array(record['joints'], (14,))
        except (KeyError, TypeError, ValueError, OverflowError):
            return self._invalid('malformed_or_nonfinite_measurement')
        cp, cr = poses['carrier']
        bp, br = poses['crossbar']
        dp, dr = poses['dock']
        if self.previous is None:
            if (step != 0 or abs(t) > _EPS
                    or np.linalg.norm(cp-np.array(C.CARRIER_INITIAL_POS)) > _CONFIG['initial_position_tolerance']+_EPS
                    or abs(q-C.CARRIER_INITIAL_Q) > _CONFIG['initial_joint_tolerance']+_EPS
                    or _angle(cr, np.eye(3)) > _CONFIG['linkage_angle_tolerance']+_EPS):
                return self._invalid('incorrect_initial_layout')
            dt = None
        else:
            old_t, old_step, old_poses, old_q = self.previous
            dt = t-old_t
            if dt <= 0 or step <= old_step:
                return self._invalid('nonmonotonic_time_or_step')
        if (np.linalg.norm(dp-np.array(C.DOCK_POS)) > _CONFIG['linkage_position_tolerance']+_EPS
                or _angle(dr, np.eye(3)) > _CONFIG['linkage_angle_tolerance']+_EPS):
            return self._invalid('fixed_dock_moved')
        residual = float(np.linalg.norm(bp-(cp+cr @ (np.array(C.CROSSBAR_ORIGIN_LOCAL)+np.array([0,q,0])))))
        if (residual > _CONFIG['linkage_position_tolerance']+_EPS
                or _angle(cr, br) > _CONFIG['linkage_angle_tolerance']+_EPS
                or q < C.SLIDE_LIMITS[0]-_CONFIG['geometry_tolerance']
                or q > C.SLIDE_LIMITS[1]+_CONFIG['geometry_tolerance']):
            return self._invalid('broken_or_inconsistent_linkage')
        carrier_boxes = _boxes(poses['carrier'], C.CARRIER_PARTS)
        bar_boxes = _boxes(poses['crossbar'], C.CROSSBAR_PARTS)
        all_points = np.concatenate([b[1] for b in carrier_boxes+bar_boxes])
        if all_points[:, 2].min() < C.TABLE_TOP-_CONFIG['drop_depth']-_EPS:
            return self._invalid('assembly_dropped_below_table')
        speed = max(float(np.linalg.norm(v[:3])) for v in velocities.values())
        spin = max(float(np.linalg.norm(v[3:])) for v in velocities.values())
        joint_speed = abs(qv)
        if dt is not None:
            speed = max(speed, *(float(np.linalg.norm(poses[n][0]-old_poses[n][0]))/dt for n in ('carrier','crossbar')))
            spin = max(spin, *(_angle(poses[n][1],old_poses[n][1])/dt for n in ('carrier','crossbar')))
            joint_speed = max(joint_speed, abs(q-old_q)/dt)
        self.previous = (t, step, poses, q)
        stable = (speed <= _CONFIG['max_linear_speed']+_EPS and spin <= _CONFIG['max_angular_speed']+_EPS
                  and joint_speed <= _CONFIG['max_joint_speed']+_EPS)
        upright = math.acos(float(np.clip(cr[2,2],-1,1))) <= _CONFIG['upright_tolerance']+_EPS
        body = carrier_boxes[0][1]
        supported = abs(float(body[:,2].min())-C.TABLE_TOP) <= _CONFIG['support_tolerance']+_EPS
        compact = abs(q) <= _CONFIG['centering_tolerance']+_EPS
        payload = np.concatenate([b[1] for b in carrier_boxes if b[0] in C.PAYLOAD_CARRIER_PART_NAMES]+[b[1] for b in bar_boxes])
        # Score against the physical fixed opening, not a target root pose or q alone.
        tol = _CONFIG['geometry_tolerance']+_EPS
        envelope = (np.max(np.abs(payload[:,1])) <= C.DOCK_HALF_WIDTH+tol
                    and payload[:,2].min() >= C.TABLE_TOP-tol
                    and payload[:,2].max() <= C.TABLE_TOP+C.DOCK_HEIGHT+tol
                    and payload[:,0].max() <= C.DOCK_BACK+tol)
        # Exempt rear grip must remain on the approach side. Without this,
        # a 180-degree reversed symmetric payload could hide its tall handle
        # beyond/through the back wall while passing the payload-only test.
        handle = next(b[1] for b in carrier_boxes if b[0] == 'Handle')
        handle_outside = handle[:,0].max() <= C.DOCK_MOUTH+tol
        entering = all_points[all_points[:,0] >= C.DOCK_MOUTH-tol]
        rear_clear = (len(entering) == 0 or
                      (np.max(np.abs(entering[:,1])) <= C.DOCK_HALF_WIDTH+tol
                       and entering[:,2].min() >= C.TABLE_TOP-tol
                       and entering[:,2].max() <= C.TABLE_TOP+C.DOCK_HEIGHT+tol
                       and entering[:,0].max() <= C.DOCK_BACK+tol))
        half = (envelope and handle_outside and rear_clear
                and float((body[:,0].min()+body[:,0].max())/2) >= C.DOCK_MOUTH-_EPS)
        full = envelope and handle_outside and rear_clear and payload[:,0].min() >= C.DOCK_MOUTH-tol
        clearances = [min(_distance(tcp[h][0],poses['carrier'],C.CARRIER_PARTS),
                          _distance(tcp[h][0],poses['crossbar'],C.CROSSBAR_PARTS)) for h in ('left','right')]
        released = bool(np.min(grippers) >= _CONFIG['release_opening']-_EPS
                        and min(clearances) >= _CONFIG['release_clearance']-_EPS)
        base = compact and stable and upright and supported
        goal = base and full and released
        if goal:
            if self.hold_since is None or (dt is not None and dt > _CONFIG['max_hold_sample_gap']+_EPS):
                self.hold_since = t
        else:
            self.hold_since = None
        hold = 0. if self.hold_since is None else t-self.hold_since
        success = goal and hold >= _CONFIG['release_hold_seconds']-_EPS
        score = 1. if success else .5 if base and half else .25 if base else 0.
        return dict(score=score, success=bool(success), invalid=False, invalid_reason=None,
                    compact=bool(compact), upright=bool(upright), supported=bool(supported),
                    stable=bool(stable), half_inserted=bool(half), full_inserted=bool(full),
                    released=released, release_hold_seconds=float(hold),
                    linkage_residual_m=residual, linear_speed_m_s=speed,
                    angular_speed_rad_s=spin, joint_speed_m_s=joint_speed,
                    tcp_clearance_m=clearances, joint_position=q)
