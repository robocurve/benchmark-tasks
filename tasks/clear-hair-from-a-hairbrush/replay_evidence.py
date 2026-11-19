"""Audit the published native state trace on CPU; no GPU or API key required."""
from pathlib import Path
import argparse,gzip,hashlib,json
from trace_audit import audit
p=argparse.ArgumentParser();p.add_argument('mode',choices=['demo','dream']);a=p.parse_args()
root=Path(__file__).resolve().parent
report=json.loads((root/'evidence'/(a.mode+'-run.json')).read_text())
for name in ['model.py','native_model.py','common.py','geometry.py','geometry_helpers.py','trace_audit.py']:
 assert hashlib.sha256((root/name).read_bytes()).hexdigest()==report['source_sha256'][name],name
with gzip.open(root/'evidence'/(a.mode+'-trace.jsonl.gz'),'rt') as f:
 result=audit((json.loads(line) for line in f),report)
print(json.dumps(result,indent=2))
