"""Verify hashes, then use the scorer frozen with a native run (CPU only)."""
import argparse,hashlib,json,subprocess,sys
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--frozen-worker',action='store_true');a=p.parse_args();run=a.run.resolve();r=json.loads((run/'run.json').read_text())
 assert not r.get('cpu_preview',True),'A preview is not native evidence'
 for name,digest in r['source_sha256'].items():
  assert Path(name).name==name,'Unsafe frozen filename'
  assert hashlib.sha256((run/'source'/name).read_bytes()).hexdigest()==digest,'Frozen source changed: '+name
 if not a.frozen_worker:
  return subprocess.run([sys.executable,str(run/'source/rescore.py'),str(run),'--frozen-worker'],check=True).returncode
 assert Path(__file__).resolve().parent==run/'source','Scorer is not running from the verified frozen source'
 from trace_audit import audit
 with (run/'trace.jsonl').open() as f:out=audit((json.loads(x) for x in f),r)
 out['frozen_source_verified']=True
 (run/'rescore.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
if __name__=='__main__':raise SystemExit(main())
