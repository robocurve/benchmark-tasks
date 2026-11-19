"""Build one independently runnable task folder; never publish or upload it."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess


def build(output):
    task=Path(__file__).resolve().parent
    root=task.parents[1]
    output=output.resolve()
    if output==task or task in output.parents:
        raise ValueError('Choose an output outside the source task folder')
    if (task/'resist_isaac/runtime.py').is_file() or not (root/'resist_isaac/runtime.py').is_file():
        raise RuntimeError('Build from the development checkout; the generated folder is already standalone')
    shutil.copytree(task,output,ignore=shutil.ignore_patterns('__pycache__','._*','.DS_Store'))
    (output/'resist_isaac').mkdir()
    for name in ('__init__.py','runtime.py','state.py','usd_assets.py'):
        shutil.copy2(root/'resist_isaac'/name,output/'resist_isaac'/name)
    (output/'tools').mkdir()
    shutil.copy2(root/'tools/run_recovery_pilot.py',output/'tools/run_recovery_pilot.py')
    shutil.copy2(root/'pyproject.toml',output/'pyproject.toml')
    shutil.copy2(root/'LICENSE',output/'LICENSE.runtime')
    provenance={'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
                'dirty':bool(subprocess.check_output(['git','status','--porcelain'],cwd=root,text=True))}
    (output/'SOURCE_REVISION.json').write_text(json.dumps(provenance,indent=2)+'\n')
    hashes={p.relative_to(output).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(output.rglob('*')) if p.is_file()}
    (output/'SHA256SUMS.json').write_text(json.dumps(hashes,indent=2)+'\n')
    print(output)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    build(parser.parse_args().output)
