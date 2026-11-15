"""Reproduce a reviewed physical score and verify its photograph/video hashes.

The numeric and visual decisions are made with the ruler/photo/video rubric in
JUDGING.md. This script checks the evidence binding and applies that rubric; it
never infers cloth capture or a knot from a hidden simulator flag.
"""
import argparse
import hashlib
import json
from pathlib import Path
from scoring import score


def evaluate(record, evidence_dir):
    evidence_dir=Path(evidence_dir).resolve()
    hashes=record.get('evidence_sha256')
    files=record.get('evidence_files')
    if not isinstance(hashes,dict) or not isinstance(files,dict) or set(hashes)!=set(files):
        raise ValueError('Every evidence hash requires its corresponding evidence_files path')
    checked={}
    for label,name in files.items():
        if not isinstance(name,str) or not name:
            raise ValueError('Invalid evidence path for '+str(label))
        path=(evidence_dir/name).resolve()
        if not path.is_relative_to(evidence_dir) or not path.is_file():
            raise ValueError('Evidence must be a real file inside the evidence directory: '+str(label))
        actual=hashlib.sha256(path.read_bytes()).hexdigest()
        if actual!=hashes[label]:
            raise ValueError('Evidence digest mismatch: '+str(label))
        checked[label]=dict(file=name,sha256=actual,bytes=path.stat().st_size)
    value=score(record)
    return dict(score=value,method='reviewed video/photos/ruler; deterministic rubric',
                evidence=checked,integrity_checked=record['integrity_checked'])


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('record',type=Path)
    p.add_argument('--evidence-dir',type=Path)
    p.add_argument('--out',type=Path)
    args=p.parse_args()
    record=json.loads(args.record.read_text())
    result=evaluate(record,args.evidence_dir or args.record.parent)
    result['judging_record_sha256']=hashlib.sha256(args.record.read_bytes()).hexdigest()
    result['scoring_source_sha256']=hashlib.sha256(Path(__file__).with_name('scoring.py').read_bytes()).hexdigest()
    text=json.dumps(result,indent=2)+'\n'
    if args.out:
        args.out.write_text(text)
    print(text,end='')


if __name__=='__main__':
    main()
