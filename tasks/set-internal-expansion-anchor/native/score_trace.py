"""Recomputes the outcome score from a recorded observations.jsonl, without physics."""
import argparse,json,hashlib
from pathlib import Path
from primary_score import PrimaryScore
p=argparse.ArgumentParser();p.add_argument('observations',type=Path);p.add_argument('--geometry',type=Path,required=True);a=p.parse_args()
g=json.load(open(a.geometry));sha=hashlib.sha256(a.geometry.read_bytes()).hexdigest();s=PrimaryScore();n=0
for line in a.observations.open():
 r=json.loads(line)
 if r['geometry_sha256']!=sha:raise ValueError('Sourcegeometryhash mismatch')
 s.observe(r,g);n+=1
print(json.dumps(dict(actual_rows=n,score=s.value,latest=s.latest,geometry_sha256=sha,formal_score=None,limit='Score recomputed from the recorded observations with the same scorer.'),indent=2))
