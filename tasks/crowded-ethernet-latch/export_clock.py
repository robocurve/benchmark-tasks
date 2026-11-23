"""Freeze actual issued actions; successful fresh replay still required."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np


def export(capture,out,diagnostic=False):
 capture=Path(capture);out=Path(out);result=json.loads((capture/'result.json').read_text())
 if not diagnostic and result.get('task_score')!=1.:raise ValueError('teacher did not complete; only explicitly diagnostic export allowed')
 raw=(capture/'trajectory.jsonl').read_bytes();rows=[json.loads(s) for s in raw.splitlines()]
 if not rows or len(rows)>2400:raise ValueError('invalid number of actual control rows')
 actions=np.asarray([r['action_14d'] for r in rows],dtype=np.float32).astype(float)
 if actions.shape!=(len(rows),14) or not np.isfinite(actions).all():raise ValueError('actual finite14D actions required')
 for i,r in enumerate(rows):
  if abs(r['time']-(i+1)/20)>1e-6:raise ValueError('missing or nonzero-start control interval')
 from yam_arena import ik
 low=np.r_[ik.LOWER,0.,ik.LOWER,0.];high=np.r_[ik.UPPER,1.,ik.UPPER,1.]
 if np.any(actions<low-1e-6) or np.any(actions>high+1e-6):raise ValueError('actual action outside mechanical bounds')
 out.mkdir(parents=True,exist_ok=True);data=json.dumps(actions.tolist(),separators=(',',':')).encode()+b'\n'
 meta=dict(action_count=len(rows),control_hz=20,total_steps=min(2400,len(rows)+20),action_lower=low.tolist(),action_upper=high.tolist(),teacher_trace_sha256=hashlib.sha256(raw).hexdigest(),teacher_measured_score=result.get('task_score'),diagnostic_only=bool(diagnostic),fresh_clock_replay_validated=False,derivation='actual issued14D float32 targets from disclosed privileged teacher; clock policy reads only state_time')
 meta_raw=json.dumps(meta,indent=2).encode()+b'\n';(out/'clock_actions.json').write_bytes(data);(out/'clock_metadata.json').write_bytes(meta_raw)
 (out/'clock_manifest.py').write_text('DATA_SHA256='+repr(hashlib.sha256(data).hexdigest())+'\nMETADATA_SHA256='+repr(hashlib.sha256(meta_raw).hexdigest())+'\n')
 return meta

if __name__=='__main__':
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture',type=Path);p.add_argument('out',type=Path);p.add_argument('--diagnostic-prefix',action='store_true');a=p.parse_args();print(json.dumps(export(a.capture,a.out,a.diagnostic_prefix),indent=2))
