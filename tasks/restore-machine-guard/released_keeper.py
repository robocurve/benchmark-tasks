"""Independent reference evidence for the post-release latch on the shipped rigid fixture."""
from math import cos, sin, radians, isfinite

# Metres, in the hinge frame. Verified against the frozen USD by the release tests.
BOLT_X = -.014
BOLT_HOME_Y = -.218
SLOT_Y = (-.184, -.174)
SLOT_X = (-.019, -.0095)
WALL_THICKNESS = .006
SHAFT_Y = (-.0145, .0325)
GROOVE_Y = (-.011, .003)
OPEN_LIMIT = -radians(2.5)
RELEASE_PHASES = {'release_right', 'withdraw_right', 'retention_check'}


def crossing(angle, local_y):
    """Door-local X and world Y where the shaft axis crosses a door-local Y plane."""
    return ((BOLT_X + local_y*sin(angle))/cos(angle),
            (local_y + BOLT_X*sin(angle))/cos(angle))


def capture(angle, travel):
    """Prove axial overlap and a geometric obstruction to reopening at fixed extension."""
    if not all(isfinite(v) for v in (angle, travel)) or abs(angle) > abs(OPEN_LIMIT):
        return False
    root = BOLT_HOME_Y + travel
    for face in SLOT_Y:
        x, y = crossing(angle, face)
        if not (SLOT_X[0] < x < SLOT_X[1] and SHAFT_Y[0] < y-root < SHAFT_Y[1]):
            return False
    mid = sum(SLOT_Y)/2
    _, groove_y = crossing(angle, mid)
    wall_x, shaft_y = crossing(OPEN_LIMIT, mid)
    return (GROOVE_Y[0] < groove_y-root < GROOVE_Y[1]
            and SLOT_X[1] < wall_x < SLOT_X[1]+WALL_THICKNESS
            and SHAFT_Y[0] < shaft_y-root < SHAFT_Y[1])


def release_evidence(physics, controller):
    """Track uninterrupted mechanical capture after physical release and independently check clearance."""
    if len(physics) != len(controller) or len(physics) < 21:
        raise ValueError('Incomplete synchronized release evidence')
    last_robot = None
    candidates = []
    consecutive = 0
    for i, (row, ctrl) in enumerate(zip(physics, controller), 1):
        if row['tick'] != i or ctrl['tick'] != i or abs(row['time_s']-i/20) > 1.e-5:
            raise ValueError('Discontinuous release evidence')
        c = row['contact']
        events = c['events']
        if 'robot_fixture' in events:
            last_robot = i
        q = row['joint_positions']
        free = ('robot_fixture' not in events and not c['pairs']
                and not any(c['bilateral_substeps'].values()))
        valid = (free and row['protected_volume_clear'] is True and row['failed'] is None
                 and ctrl['phase'] in RELEASE_PHASES and q['bolt_slide'] >= .039
                 and q['switch_slide'] >= .0045 and capture(q['hinge'], q['bolt_slide'])
                 and c.get('polled_steps') == c.get('physics_steps') == 12)
        if not valid:
            candidates.clear()
        bearing = valid and 'keeper_bearing' in events
        consecutive = consecutive + 1 if bearing else 0
        if bearing:
            candidates.append((i, consecutive))

    tail = physics[-21:]
    if not all(row['arm_clear'] is True for row in tail):
        raise ValueError('Missing continuous final arm clearance')
    in_dwell = [i for i, _ in candidates if i >= tail[0]['tick']]
    if last_robot is None or not (in_dwell or any(n >= 3 for _, n in candidates)):
        raise ValueError('Missing measured keeper bearing after physical release with capture retained to the end')
    return dict(last_robot_contact_tick=last_robot,
                first_keeper_tick=candidates[0][0], last_keeper_tick=candidates[-1][0],
                keeper_ticks=len(candidates), longest_consecutive_keeper_ticks=max(n for _,n in candidates),
                final_clear_start=tail[0]['tick'], final_tick=tail[-1]['tick'],
                keeper_ticks_in_final_dwell=len(in_dwell), minimum_final_bolt_mm=min(
                    row['joint_positions']['bolt_slide']*1000 for row in tail))



if __name__ == '__main__':
    import argparse
    import json
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path)
    folder = parser.parse_args().reference
    def read_trace(name):
        return [json.loads(line) for line in (folder/name).read_bytes().splitlines()]
    physics = read_trace('guard_trace.jsonl')
    controller = read_trace('reference_controller.jsonl')
    if any(a['joint_positions'] != b['measured_joint_positions'] or a['contact'] != b['contact']
           for a,b in zip(physics, controller, strict=True)):
        raise ValueError('Controller and native physics receipts disagree')
    print(json.dumps(release_evidence(physics, controller), indent=2))
