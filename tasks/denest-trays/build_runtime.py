"""Build the pinned simulator runtime; task sources stay in a read-only mount."""
from pathlib import Path
import argparse
import subprocess
import sys
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--image',default='denest-trays-runtime:6.1')
a=p.parse_args()
subprocess.run([sys.executable,str(ROOT/'runtime/prepare.py')],check=True)
subprocess.run(['docker','build','-f',str(ROOT/'runtime/Dockerfile'),'-t',a.image,
    '--build-context',f'arena={ROOT/".runtime/IsaacLab-Arena"}',
    '--build-context',f'yamstack={ROOT/".runtime/yam-arena"}',str(ROOT)],check=True)
