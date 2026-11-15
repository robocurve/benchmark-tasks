"""Run the fixed seam setup with the demo or Dream policy."""
import argparse
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', choices=('demo', 'dream'), required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    command = [sys.executable, str(HERE / 'preview_yam.py'), '--out', str(args.out),
        '--steps', '1447' if args.policy == 'demo' else '1600',
        '--judging-views', '--physics-hz', '480', '--coarse-thread',
        '--sewing-clamp', '--passive-clamp', '--supported-clamp', '--backed-clamp',
        '--facet-cloth', '--box-panels', '--geometric-panel-audit',
        '--panel-passage-diagnostic', '--cpu-ccd', '--clear-thread-sampling',
        '--thread-long-side-links', '4', '--thread-inertia-scale', '10',
        '--passive-thread-bending', '--sharp-edge-cut',
        '--rigid-panel-strain-telemetry', '--boundary-contact-envelope',
        '--external-forces-once', '--audit-junctions', '--practical-material',
        '--loose-tacking', '--functional-tacks']
    if args.policy == 'demo':
        command += ['--plan', str(HERE / 'seam-aligned-plan.json'), '--pose-assisted-entry']
    else:
        command += ['--dream', '--dream-rows-per-request', '4']
    subprocess.run(command, check=True)
    subprocess.run([sys.executable, str(HERE / 'assess_trace.py'), str(args.out),
                    '--out', str(args.out / 'score.json')], check=True)


if __name__ == '__main__':
    main()
