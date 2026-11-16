"""Losslessly retain hand/fluid rows; pack native collider bounds as float32.

Native PhysX poses are float32. This storage conversion is separately checked
against every original head-envelope decision, and its error is reported.
"""
from pathlib import Path
import gzip,hashlib,json
import numpy as np

def pack(source, target, mode):
    source=Path(source);target=Path(target);target.mkdir(exist_ok=True)
    names=None;bounds=[];max_error=0.;rows=0
    low=np.array([.390,0.,.8975])+[-.027,-.080,-.080]
    high=np.array([.390,0.,.8975])+[.120,.080,.135]
    with source.open() as inp,gzip.open(target/(mode+'-trace.jsonl.gz'),'wt') as out:
        for line in inp:
            row=json.loads(line);original=row['hands']['left'].pop('native_body_bounds')
            if names is None:names=list(original)
            assert list(original)==names,'Collider inventory changed'
            a=np.asarray([original[n] for n in names]);b=a.astype(np.float32)
            max_error=max(max_error,float(np.max(abs(a-b))))
            hit=lambda x:np.all(x[:,1]>low,axis=1)&np.all(x[:,0]<high,axis=1)
            assert np.array_equal(hit(a),hit(b)),'Packing changed a face-contact decision'
            bounds.append(b);rows+=1
            out.write(json.dumps(row,separators=(',',':'))+'\n')
    np.savez_compressed(target/(mode+'-bounds.npz'),names=np.asarray(names),bounds=np.asarray(bounds))
    report=dict(rows=rows,colliders=len(names),maximum_error_m=max_error,
        every_head_envelope_decision_unchanged=True,bounds_dtype='float32',
        other_fields_unchanged=True,original_trace_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    (target/(mode+'-packing.json')).write_text(json.dumps(report,indent=2)+'\n')
    return report
