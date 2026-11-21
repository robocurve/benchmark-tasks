"""Passive suspension control using the canonical cord lengths, assets and joint anchors."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import brentq
from scipy.spatial.transform import Rotation

from . import fixed_scene as s


def rest_profile():
    vertices = np.array(s.REST_STATIONS)
    pieces = [np.linspace(a, b, count + 1)[:-1] for a, b, count in zip(vertices, vertices[1:], (1, 9, 5, 3))]
    return np.vstack([*pieces, vertices[-1]])


def seated_profile(rest):
    """Wrap the identical polygonal chain around a flared pot with a flat base.

    Walk outward from the bottom knot. Each of the 18 spheres of one segment's
    length intersects the contour at the next hinge. Adjust the depth of the
    knot until the final link reaches the exact fixed hook attachment.
    """
    lengths = np.linalg.norm(np.diff(rest, axis=0), axis=1)[::-1]

    def hinges(depth):
        contour = np.array([[s.EYE_RADIUS, -depth], [.0465, -depth],
                            [.0635, -depth + .102], [.009, 0.], [.009, .06]])
        edges = np.diff(contour, axis=0)
        stations = np.r_[0., np.cumsum(np.linalg.norm(edges, axis=1))]

        def position(distance):
            return np.array([np.interp(distance, stations, coord) for coord in contour.T])

        previous_distance = 0.
        points = [contour[0]]
        for length in lengths:
            previous = points[-1]
            distance = brentq(lambda value: np.linalg.norm(position(value) - previous) - length,
                              previous_distance, stations[-1], xtol=1.e-12)
            points.append(position(distance))
            previous_distance = distance
        return np.array(points[::-1])

    depth = brentq(lambda value: hinges(value)[0, 1], .35, .405)
    result = hinges(depth)
    if np.max(np.abs(np.linalg.norm(np.diff(result, axis=0), axis=1) - lengths[::-1])) > 1.e-7:
        raise ValueError('A control strand changed length.')
    if np.linalg.norm(result[0] - rest[0]) > 1.e-6:
        raise ValueError('The control missed its hook attachment.')
    return result


def radial_points(profile, direction):
    return s.HOOK + profile[:, :1] * direction + profile[:, 1:] * np.array([0., 0., 1.])


def initialize_control(rig, offset):
    import torch
    rest = rest_profile()
    loaded = seated_profile(rest)
    x = rig.parts.get_transforms().clone()
    for i in range(4):
        angle = (i + .5) * np.pi / 2
        direction = np.array([np.cos(angle), np.sin(angle), 0.])
        before, after = [radial_points(profile, direction) for profile in (rest, loaded)]
        for j in range(len(before) - 1):
            a = before[j + 1] - before[j]
            b = after[j + 1] - after[j]
            axis = np.cross(a, b)
            rotation_angle = np.arctan2(np.linalg.norm(axis), a @ b)
            orientation = Rotation.from_rotvec(axis * rotation_angle / max(np.linalg.norm(axis), 1.e-14))
            pose = np.r_[(after[j] + after[j + 1]) / 2, orientation.as_quat()]
            x[rig.rows[f'cord_{i}_{j:02d}']] = x.new_tensor(pose)
    from .lacing import routes
    for i, empty_lace, loaded_lace in routes():
        for j,(a,b) in enumerate(zip(np.diff(empty_lace,axis=0),np.diff(loaded_lace,axis=0))):
            axis=np.cross(a,b);angle=np.arctan2(np.linalg.norm(axis),a@b)
            quat=Rotation.from_rotvec(angle*axis/max(1.e-14,np.linalg.norm(axis))).as_quat()
            x[rig.rows[f'lace_{i}_{j:02d}']]=x.new_tensor(np.r_[(loaded_lace[j]+loaded_lace[j+1])/2,quat])
    knot = s.HOOK + [0., 0., loaded[-1, 1]-s.EYE_HEIGHT]
    x[rig.rows['bottom_knot']] = x.new_tensor(np.r_[knot, 0., 0., 0., 1.])
    x[rig.rows['pot']] = x.new_tensor(np.r_[knot + [offset, 0., max(s.KNOT_HALF_HEIGHT,s.EYE_HEIGHT+s.CORD_RADIUS)+.00025], 0., 0., 0., 1.])
    ids = torch.arange(len(x), device=x.device, dtype=torch.int32)
    rig.parts.set_transforms(x, ids)
    rig.parts.set_velocities(torch.zeros_like(rig.parts.get_velocities()), ids)


def main():
    from inspect_robots import Action, Scene
    from .arena import make_embodiment
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--condition', choices=('centered', 'outside', 'upstream_5mm', 'settled_centered'), required=True)
    args = parser.parse_args()
    args.out.mkdir(exist_ok=True, parents=True)
    rig = make_embodiment(args.out)
    try:
        obs = rig.reset(Scene(id='passive-control', instruction=s.INSTRUCTION), seed=0)
        offset = {'centered': 0., 'outside': .145, 'upstream_5mm': -.005, 'settled_centered': 0.}[args.condition]
        initialize_control(rig, offset)
        settle_seconds = 3. if args.condition == 'settled_centered' else 0.
        if settle_seconds:
            # Prior to testing the unchanged release/pulse criterion, allow only
            # gravity and the strand/pot contact solver to seat this passive fixture.
            sim = rig._base.sim
            frames = round(settle_seconds / float(sim.get_physics_dt()))
            for unused in range(frames):
                sim.step(render=False)
            rig.read()
        stationary = Action(data=np.asarray(obs.state['joint_pos']).copy(), meta={})
        rows = []
        for tick in range(160):
            step = rig.step(stationary)
            rows.append(dict(tick=tick + 1, **step.info['fit']))
        success = any(row['score'] == 1. for row in rows)
        data = dict(condition='passive_' + args.condition, full_robot_reference=False,
                    initial_offset_m=offset, passive_settling_s=settle_seconds,
                    passed=success if args.condition != 'outside' else not success,
                    full_score_observed=success, trace=rows)
        (args.out / 'CONTROL.json').write_text(json.dumps(data, indent=1))
    finally:
        rig.close()


if __name__ == '__main__':
    main()
