"""CPU-only independent rescore of raw measurements, without importing simulator code."""
import argparse
import json
import os
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    args=p.parse_args()
    os.environ['FURNITURE_SLIDERS_SPEC']=str((args.run/'spec-used.json').resolve())
    from scoring import Scorer
    scorer=Scorer()
    count=0
    for line in (args.run/'measurements.jsonl').read_text().splitlines():
        record=json.loads(line)
        scorer.update(record['measurement']); count+=1
    result={'observations':count,'scoring':scorer.detail}
    print(json.dumps(result,indent=2))
    return result


if __name__=='__main__': main()
