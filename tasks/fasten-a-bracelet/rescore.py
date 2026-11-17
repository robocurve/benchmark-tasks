"""Verify a complete native run and independently recompute its geometric score."""
import argparse,hashlib,json
from pathlib import Path
from trace_audit import audit
p=argparse.ArgumentParser();p.add_argument('run',type=Path);a=p.parse_args()
r=json.loads((a.run/'run.json').read_text());assert r['status']=='complete' and not r['cpu_preview']
for name,digest in r['source_sha256'].items():
 assert hashlib.sha256((a.run/'source'/name).read_bytes()).hexdigest()==digest,name
with (a.run/'trace.jsonl').open() as f:out=audit((json.loads(x) for x in f),r)
out['frozen_source_verified']=True;out['visual_review_still_required']=True
(a.run/'rescore.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out,indent=2))
