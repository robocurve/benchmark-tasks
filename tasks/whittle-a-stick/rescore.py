"""Verify frozen sources and audit native state history on CPU."""
import argparse,hashlib,json,sys
from pathlib import Path

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args();run=a.run.resolve();r=json.loads((run/'run.json').read_text())
 assert not r.get('cpu_preview',True),'A preview is not native evidence'
 for name,digest in r['source_sha256'].items():
  assert Path(name).name==name,'Unsafe frozen filename'
  assert hashlib.sha256((run/'source'/name).read_bytes()).hexdigest()==digest,'Frozen source changed: '+name
 # Import the recorded scorer, never the current development model. The reader
 # converts NumPy scalars only when writing JSON; no scoring tolerance changes.
 sys.path.insert(0,str(run/'source'))
 from trace_audit import audit
 with (run/'trace.jsonl').open() as f:out=audit((json.loads(x) for x in f),r)
 out['frozen_source_verified']=True
 def scalar(value):
  import numpy as np
  if isinstance(value,np.generic):return value.item()
  raise TypeError(type(value).__name__)
 encoded=json.dumps(out,indent=2,default=scalar)
 (run/'rescore.json').write_text(encoded+'\n');print(encoded)
if __name__=='__main__':raise SystemExit(main())
