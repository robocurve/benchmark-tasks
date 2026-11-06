# Join two 1×1 blocks

Press the red block onto the blue block, connecting one stud. Lift the pair by the red block alone so the blue block is at least 2 cm above the table for one second. Set the joined pair down upright and let go for one second.

Category: **Societal relevance**. Joining toy bricks is a step in assisted play
and model building.

The small bricks must stay joined by a single stud.
Seated body centres line up horizontally, covering every blue stud. At full insertion, red's body centre is 19.2 mm above blue's.
The studs sit inside the sockets, adding no gap between the bodies.

**Verified simulation results:** demo 1; Dream-1 0. Replayed scores, source hashes and trial metadata are in [VALIDATION.json](VALIDATION.json). The task’s media record is listed in `task.json`.

## Setup

Two custom press-fit bricks, designed for solid printed PLA: blue 1×1, 15.8 × 15.8 mm footprint, 3.907 g model mass, body centre (0.26, 0.1, 0.7571) m; red 1×1, 15.8 × 15.8 mm footprint, 3.907 g model mass, body centre (0.2, 0.2, 0.7571) m. Both have 19.2 mm bodies and 4.8 mm-high studs, 9.6 mm in diameter on a 16 mm grid. Studs face up; edges align with X/Y. Oak-coloured 1.10 × 1.30 m tabletop, centre X=0.36 m/Y=0, top Z=0.7475 m. Two YAM arms start in the station ready pose. Fixed layout, seed 0, no jitter. Printable CAD includes socket dimensions.

World X points away from the arm bases, Y left and Z up. Brick positions refer
to rectangular body centres, excluding studs. Red is uppermost when joined.
`config.py` fixes the layout; `cad/` contains printable geometry. Mounts,
cameras, camera stand and lighting use the installed YAM station. Review rulers
in `visuals.py` are visual only and cannot support objects.

These custom Duplo-scale parts are not official LEGO. Model masses come from
CAD volume at an assumed PLA density of 1.24 g/cm³: 3.907 g for 1×1 and 12.110 g
for 2×2. They are not measured prints. Fit, friction and compliance still need
physical calibration.

## Scoring

0.25: hold red’s lowest point ≥20 mm above the table for 0.20 s. 0.5: after pickup, seat exactly 1 stud for 0.20 s: each stud/socket centre within 1 mm laterally and 0.75 mm of full insertion, relative tilt ≤3°. 1: also lift blue’s lowest point ≥20 mm for 1 s, holding red only, with all robot surfaces ≥2 mm from blue; then release for 1 s with blue’s bottom within 3 mm of the table, upright within 3°, both arms ≥5 mm clear of both bricks, and each brick stable within 2 mm/5°. Use the final score; separation or wrong alignment removes connection credit. 0: no qualifying pickup, or any brick part leaves X[−0.17,0.89] m/Y[−0.63,0.63] m (20 mm inside the table edges) or falls >15 mm below the tabletop.

See [JUDGING.md](JUDGING.md) for manual checks. Full credit requires unsupported
retention, followed by release. The scorer checks measured poses and collision
bounds for every link of both arms. Separation cancels connection credit and
the lift proof; a retry can still succeed. Report the final score.

## Reproduce a trial

Use the competition's Brev launchable with the YAM/Isaac environment and versions
listed below. Keep Dream credentials outside Git. From the repository root:

```bash
source ~/IsaacLab-Arena/yam.env
python -m unittest discover -s tasks/join-two-1x1-blocks -p 'test_*.py' -v
python tasks/join-two-1x1-blocks/demo_policy.py --check
python tasks/join-two-1x1-blocks/run.py --policy demo --seed 0 --max-steps 1200 --log-dir artifacts/join-two-1x1-blocks-demo-001
python tasks/join-two-1x1-blocks/run.py --policy dream --seed 0 --max-steps 1200 --log-dir artifacts/join-two-1x1-blocks-dream-001
python tasks/join-two-1x1-blocks/replay.py artifacts/join-two-1x1-blocks-demo-001/trajectory.jsonl
```

Rescore the included evidence with Python and NumPy; no GPU or API key is needed:

```bash
python tasks/join-two-1x1-blocks/replay.py tasks/join-two-1x1-blocks/evidence/demo.jsonl.gz
python tasks/join-two-1x1-blocks/replay.py tasks/join-two-1x1-blocks/evidence/dream.jsonl.gz
```

Use a new output directory each time and run through `run.py`. It builds the
hollow contact geometry and records robot state; the stock adapter alone
does not perform these steps.

Both policies share the recorded instruction, fixed scene, scorer and action
limits: 20 Hz control, 240 Hz physics, at most 1,200 actions (60 simulated seconds).
The scripted demo commands only joints and grippers. Once after pickup, it uses
measured brick and tool poses to correct placement for grasp offset, recorded
in `demo-corrections.json`. This is privileged demo feedback; Dream receives
only standard camera and robot observations. The demo cannot move bricks
directly or create attachments. Startup, rendering and API calls add wall time.

### Recorded simulation evidence

Wall time includes simulator startup and trial execution.

| Measurement | Demo | Dream |
|---|---|---|
| Run ID | brick-1x1-demo-002 | brick-1x1-dream-001 |
| Final score | 1 | 0 |
| Replay score | 1 | 0 |
| Seed | 0 | 0 |
| Executed actions | 400 | 1200 |
| Simulated seconds | 20 | 60 |
| Measured wall seconds | 106.703 | 241.736 |
| Control Hz | 20 | 20 |
| Physics Hz | 240 | 240 |

Physics frequency comes from the installed adapter configuration, not per-run timing.

Dream request history: 25 requests, 26 attempts, 1 retry, 2 sessions; 1 recovered request.
Retries reuse an observation window before the next action. Recovery can reset client
action smoothing; server continuity and performance effects are uncertain.
The counts alone do not identify each recovery’s cause.

Scorer version: brick-join-v1.

Both runs used the environment below.

| Component | Version |
|---|---|
| Python | 3.12.16 |
| Platform | Linux-6.8.0-1066-aws-x86_64-with-glibc2.35 |
| GPU | NVIDIA L40S |
| Package: dreamlabs-client | 1.0.14 |
| Package: inspect-robots | 0.60.0 |
| Package: inspect-robots-dream | 0.1.0 |
| Package: inspect-robots-yam | 0.37.0 |
| Package: isaaclab | 17.0.2 |
| Package: isaacsim | 6.1.0.0 |
| Package: numpy | 2.3.1 |
| Package: yam-arena | 0.1.0 |
| Source revision: IsaacLab | 37ddf626871758333d6ed89cf64ad702aef127d0 |
| Source revision: IsaacLab-Arena | c7edf3a9d5d8276565208f72422f8f8a0806d6cf |
| Source revision: yam-isaac-lab-arena | 0059d02a1a91bc6834c1d0846022891df0c78730 |

“Not recorded” means the metadata was unavailable.

## Model and evidence

Rigid shapes form the hollow bodies, roofs and studs. Compliant socket rings
hold inserted studs by friction: radial interference 0.1 mm, stiffness
500 N/m, damping 1 N·s/m, static friction 0.6 and dynamic friction 0.5.
These are simulation assumptions. There is no glue, proximity latch or attachment
joint. These parameters were used in the recorded robot trials. The separate [material probe](evidence/material-probe.json) checked aligned insertion, 2 mm misalignment and non-contact hover controls. Physical fit and force calibration remain unproven.

Runs save measured brick and robot state in `trajectory.jsonl`, the final score
in `score-summary.json`, and source snapshots and runtime in `runtime.json`,
alongside setup images and footage. The three policy views are 320×176.
The separate 1600×1280 review view and visual rulers are not Dream inputs.

For final media, export the three recorded policy views with
`inspect-robots video EVAL.json --out RUN/media --fps 20`, then run
`python tasks/join-two-1x1-blocks/render_review.py RUN --output RUN/complete.mp4`.
The exporter preserves every recorded frame and verifies the frame count.
