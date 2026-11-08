"""Unvalidated, contact-only three-peg knotting experiment.

For jig-candidate-v1's untied layout only. Standing strand runs under the low
peg, up the RIGHT peg, across the top to LEFT, and back under the low peg.
The working end must pass from -Y to +Y through this loop. All manipulation
below is via ordinary ArmMotion joint targets. No rope poses, forces,
attachments, contacts, or scoring state are changed.

Integration: import jig_thread and `yield from jig_thread(arms)` from script.
ArmMotion needs its existing move_pose/grip methods. Runtime ~100 simulated s;
allow at least 2400 control actions for this development trial. A completed
motion program is NOT evidence of a knot, retained load, or full task score.
"""
import math
import numpy as np


def jig_thread(arms):
    import telemetry
    from config import CONFIG
    from demo_policy import wait, both
    from yam_arena.constants import FINGER_OPEN
    from yam_arena import ik

    left, right = arms
    top = float(CONFIG['table_top'])
    s, c = math.sin(math.radians(20)), math.cos(math.radians(20))
    rr = np.array([[1., 0., 0.], [0., -s, c], [0., -c, -s]])
    rl = np.array([[-1., 0., 0.], [0., s, -c], [0., -c, -s]])
    receive_r = np.array([[-1., 0., 0.], [0., 0., -1.], [0., -1., 0.]])
    rb = rr @ np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    pinch, receive_open = -.0035, -.012
    right_material, left_material = -11, -2

    def event(label, **fields):
        telemetry.demo_events.append(dict(label=label, development_only=True,
            privileged_demo_only=True, **fields))

    def read():
        sample = telemetry.latest_sample
        return sample, np.asarray(sample['cord']['points'], float)

    def tcp(side):
        return np.asarray(telemetry.latest_sample['hands'][side]['tcp'], float)

    def held(side, material, maximum=.032):
        sample, points = read()
        distance = float(np.linalg.norm(points[material] - tcp(side)))
        event('jig material proximity check', side=side, index=material,
              distance_m=distance, maximum_m=maximum)
        return distance < maximum

    def abort(reason, **fields):
        event('jig sequence stopped', reason=reason, **fields)

    sample, points = read()
    if len(points) != 118:
        abort('candidate requires the 118-link untied jig layout')
        return
    event('jig trial begins', route='working end through loop from -Y to +Y',
          initial_working_index=right_material, receiving_index=left_material,
          warning='motion candidate, not validated demo')

    # Tail is along world Y; yaw pi/2 closes ACROSS it (world X).
    p = points[right_material].copy()
    yield from right.move((p[0], p[1], top+.0925), 2., opening=FINGER_OPEN,
                          yaw=math.pi/2, label='jig approach table tail')
    p = read()[1][right_material].copy()
    yield from right.move(p + [0., 0., .009], 2., label='jig descend to tail')
    yield from right.grip(pinch, 1.)
    yield from right.move((p[0], p[1], top+.0825), 2.5,
                          label='jig first measured tail lift')
    if not held(1, right_material):
        abort('tail lost during first lift')
        return
    yield from right.move_pose(tcp(1), rr, 2.5, label='jig turn side-on behind fixture')
    # Travel behind the posts before advancing centrally. The flexible strand
    # may touch a post; this first experiment must be inspected for snagging.
    yield from right.move_pose((.30, -.135, top+.0825), rr, 3.,
                               label='jig central near-side staging')
    yield from right.move_pose((.30, -.040, top+.0825), rr, 3.,
                               label='jig advance below aperture')
    if not held(1, right_material):
        abort('tail lost or snagged during staging')
        return
    yield from right.move_pose((.30, -.040, .990), rr, 4.,
                               label='jig suspend free tip inside aperture height')
    yield from wait(.6)
    if not held(1, right_material):
        abort('tail lost while raising above bridge')
        return

    # The right HAND passes above the top bridge; only the free dangling end
    # passes through the aperture. This prevents palm obstruction of the loop.
    sample, points = read()
    bridge = points[48:64]
    plane_y = float(np.median(bridge[:, 1]))
    bridge_z = float(np.min(bridge[:, 2]))
    target_z = min(.890, bridge_z-.027)
    if not .865 < target_z < .905:
        abort('loop changed outside validated aperture height', bridge_z=bridge_z)
        return
    event('jig aperture reference', plane_y=plane_y, target_tip_z=target_z,
          bridge_min_z=bridge_z)
    for k, desired_y in enumerate(np.linspace(plane_y-.035, plane_y+.065, 11)):
        sample, points = read()
        if not held(1, right_material):
            abort('tail lost during threading', feed_step=k)
            return
        tip = np.asarray(sample['cord']['end_tip'], float)
        current = tcp(1)
        goal = np.array([.30, desired_y, target_z]) - (tip-current)
        if not (.27 < goal[0] < .33 and -.10 < goal[1] < .15 and .965 < goal[2] < 1.025):
            abort('tip feedback leaves tested approach corridor', target=goal.tolist())
            return
        yield from right.move_pose(goal, rr, 2. if k == 0 else .8,
                                   label=f'jig feed tail through aperture {k}')
    yield from wait(.5)
    sample, points = read()
    tip = np.asarray(sample['cord']['end_tip'])
    if tip[1] < plane_y+.035:
        abort('tip did not remain beyond the loop plane', tip=tip.tolist())
        return
    event('jig tip reached nominal handover region', tip=tip.tolist(),
          initial_bridge_plane_y=plane_y, current_bridge_max_y=float(points[48:64, 1].max()),
          note='The loop can move; final knot topology is independently judged after removal.')

    # Receive a different material point, ~9 cm from the right grasp, from
    # the far side. Proximity is only a guard, never a scoring assertion.
    receive = points[left_material].copy()
    if not (.27 < receive[0] < .33 and .87 < receive[2] < .93):
        abort('receiving point outside tested handover region', point=receive.tolist())
        return
    receive_tcp = receive - receive_r[:, 2]*.003
    yield from left.move_pose(receive_tcp-receive_r[:, 2]*.080, receive_r, 3.,
                              opening=receive_open, label='jig far-side receiver approach')
    # Track the dangling material as open fingers approach; a stale target
    # can leave the cord beside the pads after it moves on first contact.
    for k in range(32):
        receive = read()[1][left_material].copy()
        current = tcp(0)
        delta = receive-receive_r[:, 2]*.003-current
        distance = float(np.linalg.norm(delta))
        if distance < .0015:
            break
        goal = current+delta*min(1., .004/max(distance, 1e-9))
        yield from left.move_pose(goal, receive_r, .20,
                                  label=f'jig track receiving material {k}')
    # Close in stages while the right hand continues holding the cord. A
    # local pad footprint and loaded jaw gap replace the weak radial guard.
    for opening in (-.008, -.005, pinch):
        for k in range(3):
            receive = read()[1][left_material].copy()
            current = tcp(0)
            delta = receive-receive_r[:, 2]*.003-current
            distance = float(np.linalg.norm(delta))
            goal = current+delta*min(1., .003/max(distance, 1e-9))
            yield from left.move_pose(goal, receive_r, .25, opening=opening,
                                      label='jig recenter and close receiver')
    sample, points = read()
    measured_r = np.asarray(sample['hands'][0]['tcp_rotation'], float)
    pad_point = measured_r.T @ (points[left_material]-tcp(0))
    half_gap = float(sample['hands'][0]['opening'])*.04695
    event('jig receiver pad check', point_tcp_m=pad_point.tolist(),
          measured_half_gap_m=half_gap, commanded_half_gap_m=abs(pinch))
    if not (abs(pad_point[0]) < .0025 and abs(pad_point[1]) < .006
            and -.002 < pad_point[2] < .008 and half_gap > abs(pinch)+.0002):
        abort('receiving material not seated between loaded pads')
        return
    yield from right.grip(receive_open, 1.)
    yield from right.move_pose(tcp(1)-rr[:, 2]*.10, rr, 3.,
                               opening=receive_open, label='jig withdraw right above entry side')
    yield from left.move_pose(tcp(0), rl, 1.5,
                              label='jig turn received end for withdrawal')
    before = read()[1][left_material].copy()
    yield from left.move_pose(tcp(0)+[0., .015, 0.], rl, 1.5,
                              label='jig verify received end follows left')
    displacement = float(np.linalg.norm(read()[1][left_material]-before))
    if not held(0, left_material) or displacement < .006:
        abort('receiving grasp not retained', material_displacement_m=displacement)
        return
    # y=.20 is reachable with the current left orientation; farther +Y would
    # require a wrist roll. Keep a separate height from the bridge regrasp.
    yield from left.move_pose((.30, .195, .885), rl, 3.,
                              label='jig park received end beyond peg tips')
    if not held(0, left_material):
        abort('left end slipped while parking')
        return

    # Sideways bridge pickup: local X (closing axis) is nearly vertical and
    # the bridge runs along world X. Straight-down pickup here exceeds IK.
    yield from right.move_pose((.30, -.12, .940), rb, 3.,
                               opening=receive_open, label='jig orient bridge gripper')
    sample, points = read()
    candidates = np.arange(48, 64)
    bridge_index = int(candidates[np.argmin(np.abs(points[candidates, 0]-.30))])
    bridge_point = points[bridge_index].copy()
    if bridge_point[2] < .88:
        abort('bridge collapsed before removal', point=bridge_point.tolist())
        return
    bridge_tcp = bridge_point-rr[:, 2]*.006
    yield from right.move_pose(bridge_tcp, rb, 3., label='jig capture upper bridge')
    yield from right.grip(pinch, 1.)
    if not held(1, bridge_index):
        abort('bridge not captured')
        return

    # Pull the bridge toward the open ends while retaining the working end.
    # Then carry that end over the loop's outer edge to the near side. Keeping
    # it on the far side would leave a strand draped over the lower keeper.
    for k in range(12):
        if not held(1, bridge_index) or not held(0, left_material):
            abort('grasp slipped during peg removal', removal_step=k)
            return
        current = tcp(1)
        if current[1] <= -.178:
            break
        dest = current.copy()
        dest[1] = max(-.180, current[1]-.020)
        yield from right.move_pose(dest, rb, 1.4,
                                   label='jig slide bridge toward free peg ends')
    for target, label in [((.30, .10, .98), 'raise working end above loop edge'),
                          ((.30, -.10, .98), 'carry working end to near side')]:
        yield from left.move_pose(target, rl, 3., label='jig '+label)
        if not held(1, bridge_index) or not held(0, left_material):
            abort('grasp slipped while clearing working end')
            return
    yield from wait(.5)
    sample, points = read()
    clearance = sample['metrics'].get('minimum_jig_clearance_m')
    # This is an intermediate contact guard, not the final scoring threshold.
    # The subsequent pull increases clearance before the robot lets go.
    event('jig whole cord removal check', minimum_fixture_clearance_m=clearance,
          intermediate_surface_clearance_m=.005, final_scoring_clearance_m=.010)
    if clearance is None or clearance < .005:
        abort('cord is not safely off the fixture before final pull')
        return
    yield from right.grip(receive_open, 1.)
    yield from right.move_pose((.20, -.16, .970), rb, 3.,
                               label='jig release bridge and withdraw behind')

    # Tighten by moving the held free end away from the declared standing-end
    # clamp. No endpoint forces are used by this policy. Separate scorer and
    # retention assay determine whether a genuine overhand was tied.
    # Put the free end on the near side of the clamp so the later pull draws
    # the cord away from the jig, rather than toward its lower keeper.
    yield from left.move_pose((.38, -.16, .98), rl, 3.,
                              label='jig draw working end to tighten')
    if not held(0, left_material):
        abort('left end slipped before final release')
        return
    yield from left.grip(receive_open, 1.)
    yield from left.move_pose(tcp(0)+[0., 0., .06], rl, 2.,
                              label='jig release knot and withdraw')
    yield from wait(3.)
    event('jig candidate motion complete', claimed_score=None,
          requires='topology, whole-cord fixture clearance, and retention review')
