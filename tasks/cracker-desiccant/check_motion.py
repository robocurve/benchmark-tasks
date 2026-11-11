"""Check recorded expert motion independently of the task score.

This is a trajectory screening tool, not a proof of collision-free execution.
Use measured poses and video as well as reported solver velocities: constrained
PhysX velocities need not match finite differences of the projected poses.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def summarize(folder, replay_trace=None):
    folder = Path(folder)
    result = json.loads((folder / 'result.json').read_text())
    trace = folder / 'trajectory.jsonl'
    rows = [json.loads(line) for line in trace.read_text().splitlines() if line.strip()]
    action_match = True
    if replay_trace is not None:
        reference = rows
        trace = Path(replay_trace)
        measured = [json.loads(line) for line in trace.read_text().splitlines() if line.strip()]
        rows = []
        for i, row in enumerate(measured):
            expected = reference[min(i, len(reference)-1)]
            action_match &= bool(np.allclose(row['action'], expected['action'], atol=1e-5, rtol=0))
            rows.append(dict(row, phase=expected['phase'] if i < len(reference) else 'replay_hold'))
        result = dict(result, passed=bool(rows and rows[-1].get('success') and action_match),
                      score=rows[-1].get('score') if rows else None)
    checks = []
    limits = dict(path_error_p95_m=.015, path_error_peak_m=.030,
                  tcp_acceleration_p95_m_s2=1.5, tcp_acceleration_peak_m_s2=4.,
                  tcp_speed_peak_m_s=.4, joint_target_step_peak_rad=.08,
                  joint_target_second_difference_p95_rad=.012)
    for phase in result['phases']:
        name = phase.get('object')
        if name is None:
            continue
        indices = [i for i, row in enumerate(rows) if row['phase'] == phase['label']]
        if len(indices) < 4:
            continue
        sample = [rows[i] for i in indices]
        side = phase['arm']
        before = rows[max(0, indices[0] - 1)]
        start = np.array(before['measured']['bodies'][name]['pose'][:3])
        end = np.array(phase['target'])
        u = np.arange(1, len(sample) + 1) / len(sample)
        reference = start + (u * u * (3 - 2 * u))[:, None] * (end - start)
        poses = np.array([r['measured']['bodies'][name]['pose'] for r in sample])
        tcp = np.array([r['tcp'][side] for r in sample])
        q_slice = slice(0, 6) if side == 'left' else slice(7, 13)
        commands = np.array([r['action'][q_slice] for r in sample])
        velocities = np.array([r['measured']['bodies'][name]['velocity'] for r in sample])
        entry = dict(
            phase=phase['label'], object=name, samples=len(sample),
            path_error_p95_m=float(np.percentile(np.linalg.norm(poses[:, :3] - reference, axis=1), 95)),
            path_error_peak_m=float(np.linalg.norm(poses[:, :3] - reference, axis=1).max()),
            tcp_acceleration_p95_m_s2=float(np.percentile(np.linalg.norm(np.diff(tcp, n=2, axis=0), axis=1) * 20**2, 95)),
            tcp_acceleration_peak_m_s2=float(np.linalg.norm(np.diff(tcp, n=2, axis=0), axis=1).max() * 20**2),
            tcp_speed_peak_m_s=float(np.linalg.norm(np.diff(tcp, axis=0), axis=1).max() * 20),
            joint_target_step_peak_rad=float(np.abs(np.diff(commands, axis=0)).max()),
            joint_target_second_difference_p95_rad=float(np.percentile(np.abs(np.diff(commands, n=2, axis=0)), 95)),
            reported_angular_speed_p95_rad_s=float(np.percentile(np.linalg.norm(velocities[:, 3:], axis=1), 95)),
        )
        entry['passed'] = all(entry[k] <= maximum for k, maximum in limits.items())
        checks.append(entry)
    joints=np.array([r['action'] for r in rows])[:,[0,1,2,3,4,5,7,8,9,10,11,12]]
    whole=dict(joint_target_step_peak_rad=float(np.abs(np.diff(joints,axis=0)).max()))
    whole['tcp_speed_peak_m_s']=max(float(np.linalg.norm(np.diff(np.array([r['tcp'][s] for r in rows]),axis=0),axis=1).max()*20) for s in ['left','right'])
    whole['tcp_acceleration_peak_m_s2']=max(float(np.linalg.norm(np.diff(np.array([r['tcp'][s] for r in rows]),n=2,axis=0),axis=1).max()*400) for s in ['left','right'])
    whole_limits={k:limits[k] for k in whole}
    whole['passed']=all(whole[k]<=v for k,v in whole_limits.items())
    return dict(
        passed=bool(checks) and result.get('passed', False) and all(c['passed'] for c in checks) and whole['passed'],
        task_score=result.get('score'), carry_segments=len(checks), limits=limits,
        whole_motion=whole,whole_motion_limits=whole_limits,
        phases=checks, trajectory_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        recorded_actions_sha256=hashlib.sha256((folder / 'trajectory.npz').read_bytes()).hexdigest(),
        checker_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        source='fresh replay' if replay_trace is not None else 'synthetic demonstration',
        replay_actions_match_reference=action_match if replay_trace is not None else None,
        limitations='Screening at 20 Hz; inspect videos and separate physics-substep holds. Reported angular speed is descriptive, not the sole acceptance test.',
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial', type=Path)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--replay-trace', type=Path)
    args = parser.parse_args()
    report = summarize(args.trial, args.replay_trace)
    args.out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(passed=report['passed'], task_score=report['task_score'],
                         carry_segments=report['carry_segments'],
                         failures=[r for r in report['phases'] if not r['passed']]), indent=2))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
