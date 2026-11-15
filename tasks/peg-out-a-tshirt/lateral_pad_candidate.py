"""Unvalidated local candidate after the measured lateral-pickup miss.

This pure planner re-observes the native garment at action100 and aims at the
continuous finger-pad region instead of the central notch. The runner exposes
it only through an explicit diagnostic option. No simulation state is written.
"""
import copy
import hashlib
import numpy as np

# At local (y=-8 mm, z=-10 mm), the actual collision hulls retain an
# 0.8302 mm gap even at zero command. A normalized 0.018 command therefore
# leaves 2.5204 mm, wider than the two 0.6 mm fabric layers. This candidate
# commands a 1.0650 mm aperture through ordinary finger drives. Retention and
# native contact integrity still have to be demonstrated.
PINCH_OPENING = .0025


def reobserve(reference, measured_action, rest, nodes, *, lift_turn=False,edge_patch=False,wide_insertion=False,no_turn_lift=False):
    from yam_arena import ik, station
    from yam_arena.constants import READY, SHOULDER, WRIST, GRIPPER, FINGER_OPEN
    from fixed_motion import ROTATIONS
    rest = np.asarray(rest, float)
    nodes = np.asarray(nodes, float)
    observed = np.asarray(measured_action, float)
    if rest.shape != nodes.shape or observed.shape != (14,):
        raise ValueError('Expected native garment nodes and fourteen measured joints')
    if not np.isfinite(nodes).all() or not np.isfinite(observed).all():
        raise ValueError('Nonfinite measured candidate input')
    arms = {}
    patches = {}
    audits = []
    left_patch=reference.get('edge_grasp_material_x',{}).get('left',-.13) if edge_patch else -.12
    for index, (side, hx, sign) in enumerate((('left', left_patch, 1.), ('right', .12, -1.))):
        ids = (abs(rest[:, 0] - hx) < .011) & (rest[:, 1] < .011)
        front = ids & (rest[:, 2] > 0)
        uv = rest[front, :2]
        points = nodes[front]
        fit = np.linalg.lstsq(np.c_[uv - uv.mean(0), np.ones(len(uv))],
                              points - points.mean(0), rcond=None)[0]
        width = fit[0] / np.linalg.norm(fit[0])
        normal = np.cross(width, fit[1])
        normal /= np.linalg.norm(normal)
        approach, closing = sign * width, sign * normal
        rotation = np.column_stack((closing, np.cross(approach, closing), approach))
        angle = np.deg2rad(reference.get('edge_grasp_angles_degrees',{}).get(side,-10.)
                          if edge_patch else (-10. if side == 'right' else 0.))
        rotation = rotation @ np.array(((1., 0., 0.),
            (0., np.cos(angle), -np.sin(angle)), (0., np.sin(angle), np.cos(angle))))
        if edge_patch and side=='left':
            tilt=np.deg2rad(reference.get('edge_grasp_left_closing_tilt',0.))
            rotation=rotation@np.array(((np.cos(tilt),0.,np.sin(tilt)),(0.,1.,0.),(-np.sin(tilt),0.,np.cos(tilt))))
        if edge_patch and side=='left' and reference.get('edge_grasp_flip_left',False):
            rotation=rotation@np.diag((-1.,-1.,1.))
        center = nodes[ids].mean(0)
        # Native hulls have no pad at local(y=0,z=-10mm). Both opposed pads
        # have continuous surfaces around(y=-8mm,z=-10mm), measured in125.
        desired_local = np.array((0., -.008, -.010))
        target = center - rotation @ desired_local
        q = observed[7 * index:7 * index + 6].copy()
        opening = float(observed[7 * index + 6])
        frames = []
        schedule = [
            (2., target, 1. if wide_insertion else .2, rotation, 're-observed insertion at continuous pads'),
            (1., target, PINCH_OPENING, rotation, 'ordinary finite pinch at continuous pads'),
            (1., np.array((.1494, target[1], target[2])), PINCH_OPENING, rotation,
             'pull forward clear of support'),
            (1., np.array((.1494, sign * .115, .898 if side == 'left' else .902)),
             PINCH_OPENING, rotation if lift_turn else ROTATIONS[side],
             'carry held hem clear before lifting' if lift_turn else 'turn held hem in clear space')]
        if no_turn_lift:
            schedule.append((5.,np.array((.1494,sign*.115,1.20)),PINCH_OPENING,rotation,
                             'lift both hems without rotating the fingers'))
            schedule.append((2.,np.array((.1494,sign*.115,1.20)),PINCH_OPENING if side=='left' else .6,rotation,
                             'release right hem'))
            schedule.append((2.,np.array((.1494,sign*.115,1.20)),PINCH_OPENING if side=='left' else .6,rotation,
                             'hold left hem after right release'))
        elif lift_turn:
            goal=np.asarray(reference['frames'][259]['action14'])[7*index:7*index+6]
            goal_p,goal_R=ik.tcp_pose(goal,station.BASES[side])
            schedule.append((3.,goal_p,PINCH_OPENING,goal_R,
                             'lift and slowly turn the held hem'))
        for seconds, position, grip, orientation, label in schedule:
            p0, r0 = ik.tcp_pose(q, station.BASES[side])
            opening0 = opening
            maximum_position_error = maximum_rotation_error = 0.
            for k in range(1, round(seconds * 20) + 1):
                fraction = ik.smoothstep(k / (seconds * 20))
                q, pe, re = ik.solve(p0 + fraction * (position - p0),
                    ik.matrix_to_quat_xyzw(ik.slerp_matrix(r0, orientation, fraction)),
                    q, base_pos=station.BASES[side], iters=120)
                maximum_position_error = max(maximum_position_error, float(pe))
                maximum_rotation_error = max(maximum_rotation_error, float(re))
                opening = opening0 + fraction * (grip - opening0)
                frames.append(dict(action=np.r_[q, opening].tolist(), stage=label))
            audits.append(dict(side=side, stage=label,
                max_position_error_m=maximum_position_error,
                max_rotation_error_rad=maximum_rotation_error))
            if maximum_position_error > .004 or maximum_rotation_error > .04:
                raise ValueError('Candidate pose unreachable: ' + repr(audits[-1]))
        arms[side] = frames
        patches[side] = dict(material_node_ids=np.flatnonzero(ids).tolist(),
            observed_center_m=center.tolist(), target_tcp_m=target.tolist(),
            rotation=rotation.tolist(), desired_patch_local_m=desired_local.tolist())
    result = copy.deepcopy(reference)
    replaced_through=380 if no_turn_lift else 260 if lift_turn else 200
    result['frames'][100:replaced_through] = [dict(action14=a['action'] + b['action'], stage=a['stage'])
                               for a, b in zip(arms['left'], arms['right'])]
    # Preserve the new pinch during the lift and preserve the original smooth
    # release timing. Merely changing the closure phase would reopen the jaws.
    for frame_index,frame in enumerate(result['frames'][replaced_through:],replaced_through):
        for index in (6, 13):
            # Later right-hand openings operate the spring peg, not the cloth.
            if index==13 and frame_index>=320:
                continue
            original = float(frame['action14'][index])
            fraction = np.clip((original - .018) / (.6 - .018), 0., 1.)
            frame['action14'][index] = float(PINCH_OPENING + fraction * (.6 - PINCH_OPENING))
    actions = np.array([np.r_[READY, 1., READY, 1.]] +
                       [f['action14'] for f in result['frames']])
    limits = np.tile([SHOULDER['velocity'] / 20] * 3 + [WRIST['velocity'] / 20] * 3 +
                     [GRIPPER['velocity'] / abs(FINGER_OPEN) / 20], 2)
    if np.any(abs(np.diff(actions, axis=0)) > limits * 1.001):
        ratio=abs(np.diff(actions,axis=0))/limits
        frame,joint=np.unravel_index(np.argmax(ratio),ratio.shape)
        raise ValueError(f'Candidate exceeds ordinary action speeds at action{frame+1}, joint{joint}: {ratio[frame,joint]:.3f} times limit')
    result['candidate_status'] = 'diagnostic; native retention not yet established'
    result['reobservation_at_action'] = 100
    result['lift_turn_diagnostic'] = bool(lift_turn)
    result['wide_insertion_diagnostic'] = bool(wide_insertion)
    result['pad_candidate_calibration'] = dict(patches=patches, kinematics=audits,
        normalized_pinch_opening=PINCH_OPENING,
        nominal_pad_aperture_m=.0008302080055186407 + .0939 * PINCH_OPENING,
        input_nodes_sha256=hashlib.sha256(np.ascontiguousarray(nodes).tobytes()).hexdigest(),
        state_writes=False)
    return result
