"""Run the initial-state single-screw demonstration or scene preview."""
import argparse
from pathlib import Path
import os
from single_screw_demo import run
if __name__=='__main__':
    if os.environ.get('REPAIR_CHECKPOINT'):
        raise RuntimeError('The complete demo entry point does not accept diagnostic checkpoints.')
    parser=argparse.ArgumentParser()
    parser.add_argument('output',type=Path)
    parser.add_argument('--preview-only',action='store_true')
    parser.add_argument('--assembly-only',action='store_true')
    args=parser.parse_args()
    run(args.output,preview_only=args.preview_only,assembly_only=args.assembly_only)
