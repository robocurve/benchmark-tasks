"""Independent entrypoint: replay raw JSONL observations through primary scorer."""
import json,sys
from scoring import Monitor
m=Monitor();count=0
with open(sys.argv[1])as f:
 for line in f:
  if line.strip():m.update(json.loads(line));count+=1
print(json.dumps(dict(observations=count,result=m.last),indent=2))
