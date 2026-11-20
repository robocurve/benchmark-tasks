"""Audit a frozen recording using the native float32 surface coordinates.

JSON has no array dtype. The original restore() promotes native float32 nodes
to float64, which slightly changes the SVD axes of an almost circular band.
This CPU-only wrapper restores the recorded precision, then runs the unchanged
frozen audit, including its original numerical tolerances and score checks.
It does not change the trace, frozen source, physics, or outcome rules.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    run = parser.parse_args().run.resolve()
    report = json.loads((run / 'run.json').read_text())
    assert not report.get('cpu_preview', True)
    for name, digest in report['source_sha256'].items():
        assert Path(name).name == name
        assert hashlib.sha256((run / 'source' / name).read_bytes()).hexdigest() == digest
    sys.path.insert(0, str(run / 'source'))
    from model import Model
    from trace_audit import audit

    restore = Model.restore

    def native_restore(self, state):
        restore(self, state)
        self.nodes = self.nodes.astype(np.float32)
        self.node_velocities = self.node_velocities.astype(np.float32)

    Model.restore = native_restore
    with (run / 'trace.jsonl').open() as stream:
        result = audit((json.loads(line) for line in stream), report)
    result.update(
        frozen_source_verified=True,
        precision_adapter='Native surface nodes restored as float32; unchanged frozen audit and tolerances.',
        audit_wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    )
    (run / 'rescore.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
