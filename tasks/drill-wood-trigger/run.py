#!/usr/bin/env python3
"""Run this independent task locally, with the task's physical trigger extension."""

import argparse
from datetime import datetime
from hashlib import sha256
import json
import os
from pathlib import Path
import sys

from inspect_robots.registry import embodiment, policy
from inspect_robots.cli import main
from runtime import WoodDrillEmbodiment
from demo import WoodDrillDemo

HERE = Path(__file__).resolve().parent


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['demo','dream'])
    parser.add_argument('--log-dir',type=Path)
    parser.add_argument('--headless',choices=['true','false'],default='true')
    parser.add_argument('--max-steps',type=int,default=2400)
    args = parser.parse_args()
    output = args.log_dir or Path(os.environ.get('YAM_LOG_ROOT',str(Path.home()/'yam-eval'))) / 'drill-wood-trigger' / (args.mode+'-'+datetime.now().strftime('%Y%m%d-%H%M%S'))
    output.mkdir(parents=True,exist_ok=True)
    manifest={p.name:sha256(p.read_bytes()).hexdigest() for p in HERE.iterdir() if p.suffix in ('.py','.json')}
    (output/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

    @embodiment('wood_drill')
    def make(**kwargs):
        instance=WoodDrillEmbodiment(**kwargs)
        instance._diagnostics=output/'measurements.jsonl'
        return instance

    policy('wood_drill_demo')(WoodDrillDemo)
    instruction=json.loads((HERE/'task.json').read_text())['instruction']
    argv=['run','--embodiment','wood_drill','--policy','wood_drill_demo' if args.mode=='demo' else 'dream',
          '-E','task='+str(HERE/'task.py'),'-E','object_jitter=0','-E','control_hz=20',
          '-E','cam_width=320','-E','cam_height=176','-E','headless='+args.headless,
          '--instruction',instruction,'--scorer','task_score','--max-steps',str(args.max_steps),
          '--seed','0','--no-prompt','--store-frames','--no-rerun','--no-rerun-save','--log-dir',str(output)]
    return main(argv)


if __name__=='__main__':
    sys.exit(run())
