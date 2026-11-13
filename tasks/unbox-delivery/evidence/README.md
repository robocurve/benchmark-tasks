# Validation evidence

The handle-free, wrapped-tape scene has two sealed-start scripted demonstrations
scoring 1.0. `full-004-audit.json` and `full-005-audit.json` record independent
replay, source hashes,
action checks, final state and complete three-camera frame counts.
`full-demo.jsonl.gz` contains only anonymous measured scorer inputs from that
run; it excludes stored score claims and controller text.

Replay from the task directory without Isaac:

```bash
gzip -dc evidence/full-demo.jsonl.gz > /tmp/unbox-full-demo.jsonl
python3 replay.py /tmp/unbox-full-demo.jsonl
python3 -m unittest -v test_scoring
```

`VALIDATION.json` distinguishes original, curated and compressed trace hashes.
Full videos, source snapshots and raw trial evidence are kept in ignored local
artifacts; submitted media is referenced by task.json. The pre-cut opening diagnostic earns
zero task credit and is not this sealed-start demonstration.

Dream-1 completed 3,600 actions with score 0.0. `dream-001-audit.json` and
`dream-run.jsonl.gz` preserve the measured result and anonymous replay inputs.
