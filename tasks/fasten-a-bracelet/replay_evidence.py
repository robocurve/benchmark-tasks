"""Check published native trajectories without renting a GPU."""
import argparse,gzip,hashlib,json
from pathlib import Path
from trace_audit import audit
p=argparse.ArgumentParser();p.add_argument('mode',choices=['demo','dream']);a=p.parse_args();root=Path(__file__).resolve().parent
r=json.loads((root/'evidence'/f'{a.mode}-run.json').read_text())
for name in ['model.py','common.py','bracelet_physics.py','bracelet_geometry.py']:
 assert hashlib.sha256((root/name).read_bytes()).hexdigest()==r['source_sha256'][name],name
with gzip.open(root/'evidence'/f'{a.mode}-trace.jsonl.gz','rt') as f:print(json.dumps(audit((json.loads(x) for x in f),r),indent=2))
