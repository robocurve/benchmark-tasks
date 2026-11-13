"""Create an offline source bundle without altering any other task or environment."""
import hashlib
import json
from pathlib import Path
import py_compile
import tempfile
import zipfile

ROOT=Path(__file__).resolve().parent


def build():
    files=[p for p in ROOT.rglob('*') if p.is_file()
           and not any(part in ('__pycache__','dist','runs') for part in p.relative_to(ROOT).parts)
           and p.suffix in ('.py','.json','.toml','.sh','.md','.txt')]
    with tempfile.TemporaryDirectory() as temp:
        for i,p in enumerate(files):
            if p.suffix=='.py': py_compile.compile(str(p),cfile=str(Path(temp)/f'{i}.pyc'),doraise=True)
    dist=ROOT/'dist';dist.mkdir(exist_ok=True)
    manifest={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
    bundle=dist/'remove-chip-from-open-packet-source.zip'
    with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(files): z.write(p,ROOT.name+'/'+p.relative_to(ROOT).as_posix())
    report=dict(files=manifest,bundle_sha256=hashlib.sha256(bundle.read_bytes()).hexdigest(),
                isaac_trial_completed=False,submission_ready=False)
    (dist/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(bundle=str(bundle),files=len(files),syntax='passed',submission_ready=False),indent=2))


if __name__=='__main__': build()

