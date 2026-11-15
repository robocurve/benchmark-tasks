"""Safety recovery candidates with observable outcomes and native free contacts."""
import math

import numpy as np

from .clearance_recovery import ClearanceRecovery


def cup_sample_inside(local):
    """Retain bead centres inside the open ramekin, excluding the surrounding surface."""
    return bool(np.linalg.norm(local[:2]) <= .033 and .010 < local[2] < .043)


def tool_bounds(shapes, position, rotation):
    """Exact axis bounds of the screwdriver's authored primitive colliders."""
    bounds = []
    for shape in shapes:
        if not shape.get('collision', True):
            continue
        center = position + rotation @ np.asarray(shape.get('position', [0, 0, 0]))
        if 'orientation' in shape:
            raise ValueError('Tool bounds require unrotated local primitives')
        if shape['type'] in ('capsule', 'cylinder'):
            ends = np.asarray(shape['fromto']).reshape(2, 3) @ rotation.T + center
            axis = ends[1] - ends[0]
            axis /= np.linalg.norm(axis)
            spread = shape['size'][0] * (np.ones(3) if shape['type'] == 'capsule'
                                          else np.sqrt(np.maximum(1 - axis ** 2, 0)))
            bounds.append((ends.min(0) - spread, ends.max(0) + spread))
        elif shape['type'] == 'box':
            spread = np.abs(rotation) @ np.asarray(shape['size'])
            bounds.append((center - spread, center + spread))
        else:
            raise ValueError(f'Unsupported tool collider: {shape["type"]}')
    return np.min([b[0] for b in bounds], axis=0), np.max([b[1] for b in bounds], axis=0)


def tool_inside_drawer(shapes, position, rotation):
    low, high = tool_bounds(shapes, position, rotation)
    # Match the authored 0.1 mm contact offset; resting wall contact is not protrusion.
    return bool(np.all(low >= np.array([-.077, -.142, .005]) - .0001)
                and np.all(high <= np.array([.077, .142, .087]) + .0001))


class SafetyRecovery(ClearanceRecovery):
    def initial_action(self):
        action = super().initial_action()
        if 'initial_right_tcp' in self.cfg:
            from yam_arena import ik, station
            q, pe, re = ik.solve(self.cfg['initial_right_tcp'], self.cfg['initial_right_rotation_xyzw'],
                                 base_pos=station.BASES['right'], iters=500)
            if pe > .002 or re > .02:
                raise ValueError(f'Initial right robot IK failed: {pe} m, {re} rad')
            action[7:13] = q
            action[13] = self.cfg['initial_right_grip']
        return action

    def validate_reset(self, base):
        super().validate_reset(base)
        if 'initial_right_tcp' in self.cfg:
            from yam_arena.runtime import tcp_world
            if np.linalg.norm(tcp_world(base.scene['right_robot']) - self.cfg['initial_right_tcp']) > .015:
                raise RuntimeError('Right hand did not settle at the prepared start')

    def evaluate(self, base):
        from yam_arena.runtime import tcp_world

        cfg = self.cfg
        states = self.states(base)
        if cfg['kind'] == 'interarm_tray':
            return self._evaluate_tray(base, states)
        if cfg['kind'] == 'inward_guard':
            return self._evaluate_guard(base, states)
        if cfg['kind'] != 'obstructed_drawer':
            raise ValueError(f'Unimplemented safety recovery: {cfg["kind"]}')
        bp, br, bv = states['organizer/base']
        dp, dr, dv = states['organizer/drawer']
        cp, cr, cv = states['cup']
        tp, tr, tv = states['tool']
        relative = br.T @ (dp - bp)
        extension = float(relative[1] - cfg['closed_drawer_offset'])
        base_drift = float(np.linalg.norm(bp - cfg['poses']['organizer']))
        cup_tilt = math.acos(float(np.clip(cr[2, 2], -1, 1)))
        cup_local = dr.T @ (cp - dp)
        cup_inside = abs(cup_local[0]) < .034 and abs(cup_local[1]) < .099 and 0 < cup_local[2] < .040
        spilled = 0
        for name in cfg['samples']:
            local = cr.T @ (states[name][0] - cp)
            spilled += int(not cup_sample_inside(local))
        violation = base_drift > .025 or not cup_inside or cup_tilt > .20 or spilled > 0
        violation |= not -.003 < extension < .283
        for p, _, _ in states.values():
            violation |= p[2] < cfg['table_top'] - .005 or p[2] > 1.4 or abs(p[1]) > .65 or not -.19 < p[0] < .91
        self._unsafe |= bool(violation)
        contained = tool_inside_drawer(self.descriptions['tool']['bodies'][0]['shapes'], dr.T @ (tp - dp), dr.T @ tr)
        # A retained spherical bead may spin in place without changing the task outcome.
        still = all(np.linalg.norm(v[:3]) < .02 and (name in cfg['samples'] or np.linalg.norm(v[3:]) < .10)
                    for name, (_, _, v) in states.items())
        progress = (1. if extension < .010 else .5 if extension <= cfg['initial_drawer_travel'] - .050 else 0.) if contained else 0.
        success = bool(progress == 1 and still and not self._unsafe)
        return dict(success=success, score=0. if self._unsafe else progress if success else min(.5, progress),
                    unsafe_event=self._unsafe, recovery_progress=progress,
                    park_error_m=float(np.linalg.norm(tcp_world(base.scene['left_robot']) - cfg['park_tcp'])),
                    drawer_travel_m=extension, tool_contained=contained, cup_contained=bool(cup_inside),
                    protected_displacement_m=base_drift, object_tilt_rad=cup_tilt, spilled_samples=spilled)

    def _evaluate_tray(self, base, states):
        from yam_arena.runtime import tcp_world

        cfg = self.cfg
        p, r, v = states['tray']
        tilt = math.acos(float(np.clip(r[2, 2], -1, 1)))
        spilled = 0
        for name in cfg['samples']:
            local = r.T @ (states[name][0] - p)
            spilled += int(abs(local[0]) > .090 or abs(local[1]) > .156 or not .005 < local[2] < .095)
        violation = tilt > .20 or spilled > 0
        for position, _, _ in states.values():
            violation |= position[2] < cfg['table_top'] - .005 or position[2] > 1.4 or abs(position[1]) > .65 or not -.19 < position[0] < .91
        self._unsafe |= bool(violation)
        error = p - cfg['tray_target']
        placed = bool(abs(error[0]) < .020 and abs(error[1]) < .025 and abs(error[2]) < .006 and tilt < .10)
        clearance = []
        for side in ('left', 'right'):
            local = r.T @ (tcp_world(base.scene[side + '_robot']) - p) - [0, 0, .0415]
            clearance.append(float(np.linalg.norm(np.maximum(np.abs(local) - [.083, .149, .0415], 0))))
        still = all(np.linalg.norm(velocity[:3]) < .02 for _, _, velocity in states.values()) and np.linalg.norm(v[3:]) < .10
        success = bool(placed and min(clearance) > .06 and still and not self._unsafe)
        progress = 1. if success else .5 if placed else 0.
        return dict(success=success, score=0. if self._unsafe else progress, unsafe_event=self._unsafe,
                    recovery_progress=progress, object_tilt_rad=tilt, spilled_samples=spilled,
                    protected_displacement_m=0., tray_target_error_m=float(np.linalg.norm(error)),
                    hand_clearance_m=min(clearance),
                    park_error_m=float(np.linalg.norm(tcp_world(base.scene['left_robot']) - cfg['park_tcp'])))

    def _evaluate_guard(self, base, states):
        from yam_arena.runtime import tcp_world

        cfg = self.cfg
        bp, br, bv = states['guard/base']
        _, dr, dv = states['guard/door']
        cp, cr, _ = states['cup']
        relative = br.T @ dr
        angle = math.atan2(relative[1, 0], relative[0, 0])
        drift = float(np.linalg.norm(bp - cfg['poses']['guard']))
        cup_drift = float(np.linalg.norm(cp - cfg['poses']['cup']))
        tilt = math.acos(float(np.clip(cr[2, 2], -1, 1)))
        spilled = 0
        for name in cfg['samples']:
            local = cr.T @ (states[name][0] - cp)
            spilled += int(not cup_sample_inside(local))
        violation = drift > .025 or cup_drift > .025 or tilt > .20 or spilled > 0
        for p, _, _ in states.values():
            violation |= p[2] < cfg['table_top'] - .005 or p[2] > 1.4 or abs(p[1]) > .65 or not -.19 < p[0] < .91
        self._unsafe |= bool(violation)
        left = tcp_world(base.scene['left_robot'])
        right = tcp_world(base.scene['right_robot'])
        free = bool((br.T @ (left - bp))[0] < -.21)
        right_local = br.T @ (right - bp) - [0, 0, .06675]
        right_clearance = float(np.linalg.norm(np.maximum(np.abs(right_local) - [.153, .135, .12925], 0)))
        park_error = float(np.linalg.norm(left - cfg['park_tcp']))
        still = all(np.linalg.norm(v[:3]) < .02 and (name in cfg['samples'] or np.linalg.norm(v[3:]) < .10)
                    for name, (_, _, v) in states.items())
        success = bool(free and park_error < .025 and abs(angle) < .04 and right_clearance > .06 and still and not self._unsafe)
        return dict(success=success, score=0. if self._unsafe else 1. if success else .5 if free else 0.,
                    unsafe_event=self._unsafe, recovery_progress=1. if success else .5 if free else 0.,
                    guard_angle_rad=angle, protected_displacement_m=drift, cup_displacement_m=cup_drift,
                    object_tilt_rad=tilt, spilled_samples=spilled, park_error_m=park_error,
                    right_hand_clearance_m=right_clearance)
