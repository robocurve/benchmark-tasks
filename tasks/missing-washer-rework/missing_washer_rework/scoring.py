# SPDX-License-Identifier: MIT
"""Measured-pose grading; no controller labels, actions, or saved success flags.

Poses are actor-root world XYZ + XYZW, SI units. Washer root is its
geometric centre; cap root is its bottom. See config_dict for tolerances.
"""
from __future__ import annotations

import math
import numpy as np
from . import contract as C

CONFIG = dict(fixture_displacement=.020, fixture_tilt_deg=5.,
              reference_displacement=.005, reference_rotation_deg=3.,
              radial_tolerance=.004, seating_height_tolerance=.003,
              part_tilt_deg=5., linear_speed=.010, angular_speed=.10,
              stable_hold_seconds=1.5, max_sample_gap=.075,
              gripper_open=.8, hand_clearance=.10,
              park_hold_seconds=.30, park_linear_speed=.020,
              park_angular_speed=.20, park_post_clearance=.060,
              removed_bottom_clearance=.005, aperture_collision_tolerance=.0005,
              reset_position_tolerance=.010, park_support_tolerance=.003,
              drop_below_table=.010)
NAMES = C.DYNAMIC_NAMES + C.REFERENCE_NAMES


def config_dict():
    return dict(CONFIG)


def _rotation(q):
    q = np.asarray(q, dtype=float)
    n = float(np.linalg.norm(q))
    if abs(n - 1.) > .02:
        raise ValueError('quaternion must be unit XYZW')
    x, y, z, w = q / n
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _angle(a, b):
    return math.acos(float(np.clip((np.trace(a.T @ b)-1.) / 2., -1., 1.)))


def _tilt(r):
    return math.acos(float(np.clip(r[2, 2], -1., 1.)))


def _vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f'expected {size} finite measurements')
    return a


def _parse(record):
    t = float(record['t'])
    step = record['step']
    if not math.isfinite(t) or t < 0 or isinstance(step, bool) or int(step) != step or step < 0:
        raise ValueError('invalid timestamp or control step')
    poses = {n: _vector(record['poses'][n], 7) for n in NAMES}
    rotations = {n: _rotation(p[3:]) for n, p in poses.items()}
    velocities = {n: _vector(record['velocities'][n], 6) for n in C.DYNAMIC_NAMES}
    tcp = {n: _vector(record['tcp'][n], 7) for n in ('left', 'right')}
    for p in tcp.values():
        _rotation(p[3:])
    grippers = {n: float(record['grippers'][n]) for n in tcp}
    if any(not math.isfinite(g) or not 0 <= g <= 1 for g in grippers.values()):
        raise ValueError('grippers must be finite normalized opening in [0,1]')
    _vector(record['joints'], 14)
    return t, int(step), poses, rotations, velocities, tcp, grippers


def _post_fits_aperture(fixture_pos, fixture_rotation, part_pose, part_rotation,
                        aperture_width, slab_low, slab_high):
    """Clip the actual post prism against the aperture's local vertical slab.

    Square holes need corner containment, including relative yaw and tilt;
    matching centre heights alone cannot establish that the post is enclosed.
    """
    points = []
    for z in (-1, 1):
        for y in (-1, 1):
            for x in (-1, 1):
                local = np.array(C.POST_CENTER_LOCAL) + np.array(C.POST_SIZE)*[x,y,z]/2
                world = fixture_pos + fixture_rotation @ local
                points.append(part_rotation.T @ (world-part_pose[:3]))
    clipped = [p for p in points if slab_low <= p[2] <= slab_high]
    for i, a in enumerate(points):
        for bit in (1,2,4):
            j = i ^ bit
            if j <= i:
                continue
            b = points[j]
            for z in (slab_low, slab_high):
                if (a[2]-z)*(b[2]-z) < 0:
                    clipped.append(a+(b-a)*(z-a[2])/(b[2]-a[2]))
    return bool(clipped) and all(np.max(np.abs(p[:2])) <= aperture_width/2+CONFIG['aperture_collision_tolerance'] for p in clipped)


def _collider_boxes(name):
    """Exact local (size, centre) boxes matching generated collision geometry."""
    if name == 'fixture':
        return [(C.FIXTURE_SIZE, (0.,0.,0.)),
                (C.FIXTURE_EXTENSION_SIZE, C.FIXTURE_EXTENSION_LOCAL),
                (C.POST_SIZE, C.POST_CENTER_LOCAL),
                ((.024,.020,.018), (0.,C.FIXTURE_GRIP_LOCAL[1],.017)),
                (C.FIXTURE_GRIP_SIZE, C.FIXTURE_GRIP_LOCAL)]
    if name == 'washer':
        outer, hole, height = C.WASHER_SIZE[0], C.WASHER_HOLE, C.WASHER_SIZE[2]
        wall, offset = (outer-hole)/2, (outer+hole)/4
        return [((wall,outer,height),(-offset,0.,0.)),
                ((wall,outer,height),(offset,0.,0.)),
                ((hole,wall,height),(0.,-offset,0.)),
                ((hole,wall,height),(0.,offset,0.)),
                (C.WASHER_GRIP_SIZE,C.WASHER_GRIP_LOCAL)]
    if name == 'cap':
        outer, hole, height, cavity = C.CAP_SIZE[0], C.CAP_HOLE, C.CAP_SIZE[2], C.CAP_CAVITY_HEIGHT
        wall, offset = (outer-hole)/2, (outer+hole)/4
        return [((wall,outer,cavity),(-offset,0.,cavity/2)),
                ((wall,outer,cavity),(offset,0.,cavity/2)),
                ((hole,wall,cavity),(0.,-offset,cavity/2)),
                ((hole,wall,cavity),(0.,offset,cavity/2)),
                ((outer,outer,height-cavity),(0.,0.,(height+cavity)/2)),
                (C.CAP_GRIP_SIZE,C.CAP_GRIP_LOCAL)]
    raise ValueError(f'unknown dynamic collider: {name}')


def _lowest_collider_z(name, pose, rotation):
    # A rotated box's exact minimum world z equals its transformed centre z
    # minus the vertical projection of its three half-extents.
    return min(float(pose[2]+rotation[2]@np.asarray(centre)-
                     np.abs(rotation[2])@(np.asarray(size)/2))
               for size, centre in _collider_boxes(name))


class ReworkScorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.anchors = None
        self.removed = False
        self.parked = False
        self.washer_after_park = False
        self.previous_washer_seated = False
        self.park_start = None
        self.hold_start = None
        self.fault = None
        self.metrics = dict(score=0., success=False, valid=True,
                            cap_removed=False, cap_parked=False,
                            washer_seated=False, assembly_ready=False,
                            stable_seconds=0., fixture_displacement=0.)

    def _fail(self, reason):
        self.fault = reason
        self.hold_start = self.park_start = None
        self.metrics.update(score=0., success=False, valid=False, error=reason,
                            assembly_ready=False, stable_seconds=0.)
        return dict(self.metrics)

    def _initial_state(self, poses, rotations):
        # Absolute authored datums catch COM/root and local/world-frame mistakes.
        expected = dict(fixture=C.FIXTURE_POS, cap=C.CAP_INITIAL, washer=C.WASHER_INITIAL,
                        reference_fixture=C.REFERENCE_POS,
                        reference_washer=np.array(C.REFERENCE_POS)+C.WASHER_SEATED_LOCAL,
                        reference_cap=np.array(C.REFERENCE_POS)+[0, 0, C.BASE_TOP_LOCAL+C.WASHER_SIZE[2]])
        for name, position in expected.items():
            if np.linalg.norm(poses[name][:3]-position) > CONFIG['reset_position_tolerance']:
                raise ValueError(f'reset pose mismatch: {name}')
            if _tilt(rotations[name]) > math.radians(5):
                raise ValueError(f'reset orientation mismatch: {name}')
        self.anchors = {n: (p[:3].copy(), rotations[n].copy()) for n, p in poses.items()}

    def update(self, record):
        if self.fault:
            return dict(self.metrics)
        try:
            t, step, poses, rotations, velocities, tcp, grippers = _parse(record)
            if self.anchors is None:
                self._initial_state(poses, rotations)
            dt = None
            contiguous = False
            if self.previous is not None:
                pt, ps, pp, pr = self.previous
                dt = t-pt
                if dt <= 0 or step <= ps:
                    raise ValueError('trace time and step must increase strictly')
                contiguous = dt <= CONFIG['max_sample_gap']+1e-8 and step == ps+1
            for name in C.DYNAMIC_NAMES:
                if _lowest_collider_z(name, poses[name], rotations[name]) < C.TABLE_TOP-CONFIG['drop_below_table']-1e-9:
                    raise ValueError(f'part dropped below table by more than 10 mm: {name}')
            fixture_disp = float(np.linalg.norm(poses['fixture'][:3]-self.anchors['fixture'][0]))
            if fixture_disp > CONFIG['fixture_displacement']+1e-9:
                raise ValueError('fixture moved more than 20 mm from reset')
            for name in C.REFERENCE_NAMES:
                anchor, rotation = self.anchors[name]
                if (np.linalg.norm(poses[name][:3]-anchor) > CONFIG['reference_displacement']+1e-9 or
                    _angle(rotation, rotations[name]) > math.radians(CONFIG['reference_rotation_deg'])):
                    raise ValueError(f'reference disturbed: {name}')
        except (KeyError, ValueError, TypeError, OverflowError) as exc:
            return self._fail(str(exc))

        def slow(name, linear, angular):
            v = velocities[name]
            if np.linalg.norm(v[:3]) > linear+1e-9 or np.linalg.norm(v[3:]) > angular+1e-9:
                return False
            if not contiguous:
                return False
            return (np.linalg.norm(poses[name][:3]-pp[name][:3])/dt <= linear+1e-9 and
                    _angle(pr[name], rotations[name])/dt <= angular+1e-9)

        rf = rotations['fixture']
        f = poses['fixture'][:3]
        local = {n: rf.T @ (poses[n][:3]-f) for n in ('cap', 'washer')}
        upright_fixture = (_tilt(rf) <= math.radians(CONFIG['fixture_tilt_deg']) and
                           abs(f[2]-C.FIXTURE_POS[2]) <= CONFIG['seating_height_tolerance'])
        upright = {n: _tilt(rf.T @ rotations[n]) <= math.radians(CONFIG['part_tilt_deg'])
                   for n in ('cap', 'washer')}
        if local['cap'][2] > C.POST_CENTER_LOCAL[2]+C.POST_SIZE[2]/2+CONFIG['removed_bottom_clearance']:
            self.removed = True

        cap = poses['cap'][:3]
        cap_released = all(grippers[h] >= CONFIG['gripper_open'] or
                           np.linalg.norm(tcp[h][:3]-cap) >= CONFIG['hand_clearance'] for h in tcp)
        park_size = np.asarray(getattr(C, 'PARK_SIZE', (.09, .09, .006)))
        cap_half_extent = np.abs(rotations['cap'][:2, :2]) @ (np.array(C.CAP_SIZE[:2])/2)
        on_pad = (np.all(np.abs(cap[:2]-np.array(C.PARK_POS[:2]))+cap_half_extent <= park_size[:2]/2) and
                  abs(cap[2]-(C.PARK_POS[2]+park_size[2]/2)) <= CONFIG['park_support_tolerance'])
        on_table = abs(cap[2]-C.TABLE_TOP) <= CONFIG['park_support_tolerance']
        park_now = (self.removed and
                    np.linalg.norm(cap[:2]-f[:2]) >= CONFIG['park_post_clearance'] and
                    (on_table or on_pad) and
                    _tilt(rotations['cap']) <= math.radians(CONFIG['part_tilt_deg']) and cap_released and
                    slow('cap', CONFIG['park_linear_speed'], CONFIG['park_angular_speed']))
        if not park_now or not contiguous:
            self.park_start = None
        elif self.park_start is None:
            self.park_start = t
        was_parked = self.parked
        if self.park_start is not None and t-self.park_start >= CONFIG['park_hold_seconds']-1e-8:
            self.parked = True

        washer_seated = (upright_fixture and upright['washer'] and
                         _post_fits_aperture(f, rf, poses['washer'], rotations['washer'],
                                             C.WASHER_HOLE, -C.WASHER_SIZE[2]/2, C.WASHER_SIZE[2]/2) and
                         np.linalg.norm(local['washer'][:2]) <= CONFIG['radial_tolerance']+1e-9 and
                         abs(local['washer'][2]-C.WASHER_SEATED_LOCAL[2]) <= CONFIG['seating_height_tolerance']+1e-9)
        # A seating transition must occur after measured parking. A washer
        # seated prematurely must be removed and reseated to earn credit.
        if washer_seated and not self.previous_washer_seated and was_parked:
            self.washer_after_park = True
        if not washer_seated:
            self.washer_after_park = False
        self.previous_washer_seated = bool(washer_seated)
        cap_seated = (upright['cap'] and
                      _post_fits_aperture(f, rf, poses['cap'], rotations['cap'],
                                          C.CAP_HOLE, 0., C.CAP_CAVITY_HEIGHT) and
                      np.linalg.norm(local['cap'][:2]-local['washer'][:2]) <= CONFIG['radial_tolerance']+1e-9 and
                      np.linalg.norm(local['cap'][:2]) <= CONFIG['radial_tolerance']+1e-9 and
                      abs(local['cap'][2]-(local['washer'][2]+C.WASHER_SIZE[2]/2)) <= CONFIG['seating_height_tolerance']+1e-9)
        hands_clear = (all(grippers[h] >= CONFIG['gripper_open'] for h in tcp) and
                       all(np.linalg.norm(tcp[h][:3]-poses[n][:3]) >= CONFIG['hand_clearance']
                           for h in tcp for n in ('fixture', 'washer', 'cap')))
        stable = all(slow(n, CONFIG['linear_speed'], CONFIG['angular_speed']) for n in C.DYNAMIC_NAMES)
        ready = bool(self.washer_after_park and washer_seated and cap_seated and hands_clear and stable)
        if not ready or not contiguous:
            self.hold_start = None
        elif self.hold_start is None:
            self.hold_start = t
        held = 0. if self.hold_start is None else max(0., t-self.hold_start)
        success = ready and held >= CONFIG['stable_hold_seconds']-1e-8
        score = 1. if success else .5 if self.washer_after_park and washer_seated else .25 if self.parked else 0.
        self.previous = (t, step, poses, rotations)
        self.metrics = dict(score=score, success=bool(success), valid=True,
                            cap_removed=self.removed, cap_parked=self.parked,
                            washer_seated=bool(washer_seated), assembly_ready=ready,
                            stable_seconds=held, fixture_displacement=fixture_disp)
        return dict(self.metrics)
