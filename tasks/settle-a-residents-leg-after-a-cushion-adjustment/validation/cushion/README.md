# Does the controller require direct contact with the rigid cushion backing?

A pairwise USD collision filter excluded the only collider on the cushion backing from each of the 18 rigid bodies on the two YAM arms. Foam-to-arm, cushion-to-bed and limb-to-cushion interactions used the original scene. A second native contact binding recorded actual robot contacts with the sole and each of the six compressible rows during all 3,360 physics steps. Each pair used the same recorded action values in its stock and filtered runs, verified by SHA-256.

| Control | Robot–sole contact steps, stock → filtered | Robot–foam contact steps, stock → filtered | Cushion lateral travel (mm), stock → filtered | Heel load (N), stock → filtered | Score, stock → filtered |
| --- | ---: | ---: | ---: | ---: | ---: |
| Reference | 26 → 0 | 420 → 447 | 280.46 → 280.46 | 0.00 → 0.00 | 1.0 → 1.0 |
| Heel on foam | 45 → 0 | 410 → 455 | 281.55 → 281.56 | 18.05 → 18.11 | 0.0 → 0.0 |
| Idle | 0 → 0 | 0 → 0 | 0.00 → 0.00 | 16.92 → 16.92 | 0.0 → 0.0 |

The filtered reference reached 34.76 mm final heel clearance. Under this controller the robot can push the cushion by engaging its compressible rows. The backing still transmits loads, constrains row geometry and slides on the bed. An ordinary pillow and a physical care setup would require separate tests.

These six development runs used standalone CPU PhysX with the task's gravity-compensation approximation. The original Isaac/YAM evaluation and hosted results retain their recorded provenance.

## Reproduction

From the public task directory, activate a Python 3.12 environment with the task geometry dependencies in `runtime/requirements.txt`, the station's `yam_arena` and `inspect_robots` packages, and standalone CPU PhysX (`ovphysx` version `0.8.0.43443703471` in this run). Then run the experiment.

```bash
bash validation/cushion/run.sh
```

The script creates a new directory under `results/`, runs all six trials and compares their controls, contacts and grades. Per-condition logs, sampled poses, raw actions and contact summaries are retained there. The attached [comparison.json](comparison.json) includes fixture, evaluator and action digests. The fixture hashes identify the original USD assets. The filtered body pairs are recorded separately in the temporary composed scene.
