# Restore a missing spacer washer beneath a protective cap

A capped fixture is missing a spacer washer. Compare it with the complete reference,
remove and park its cap, insert the washer over the white post, replace the cap, and
release both hands. The fixed simulated YAM setup has three full-score scripted
passes and a genuine Dream-1 evaluation scoring 0.0. The recorded media is uploaded as `M-YPMRWZ`.

## Recorded results

All runs use the same frozen executable revision:
`6b17a58f0ed90d227564f634f48946859a507f523387c7ee7de6f3fc72c4e67f`.

| Evaluation | Task score | Control steps | Simulated duration | Wall time |
| --- | ---: | ---: | ---: | ---: |
| Selected scripted demonstration | 1.0 | 1,396 | 69.80 s | 355.0 s |
| Consecutive reset demonstration, epoch 0 | 1.0 | 1,396 | 69.80 s | Combined reset-run timing retained in its receipt |
| Consecutive reset demonstration, epoch 1 | 1.0 | 1,439 | 71.95 s | Combined reset-run timing retained in its receipt |
| Dream-1 beta | 0.0 | 2,400 | 120.00 s | 488.9 s |

The selected demo and both reset epochs independently regraded to 1.0 from raw
measurements. The second epoch restored the missing-washer starting arrangement;
its result is not a continuation of the preceding completed assembly. The fixed-layout
results establish demonstrated simulated feasibility and three successful passes,
not broad reliability, blind diagnosis or hardware performance.

Dream used `dreamlabs/dream-1-beta`, a 300 s per-request timeout, and only the three
calibrated 320 x 176 camera images and joint state. Its recorded
`demo_observations=false` setting excludes privileged object poses. The run completed
without framework errors; its valid task score was 0.0. It lifted the cap but did not
complete parking. No cause for this outcome is established. It used 56 guarded SDK
calls: about $0.0747 at the quoted provider rate, versus a $0.56 conservative guard
reservation. These are estimates/reservations, not an invoice.

46 CPU tests passed, including scoring adversaries, temporal/reset state, independent
measurement replay and actual USD collider parity. Physics trials provide separate
evidence beyond those software checks.

## Scene and reset

World axes: x forward, y left, z up; dimensions are SI units unless marked mm.
Tabletop z is 0.7475 m. The YAM bases are at (0, +0.305, 0.75) and
(0, -0.305, 0.75). The blue fixture, washer and cap are loose rigid plastic parts;
the complete reference and pads are fixed. The cap is open underneath and
gravity-seated, without threads or an elastic lock.

| Object | Position/root datum and geometry |
| --- | --- |
| Blue target fixture | Base centre (0.29, 0, 0.7555); 80 x 280 x 16 mm base, 300 g. Ivory-white square 18 x 18 x 40 mm post starts at the base top. A 30 x 100 x 16 mm lateral extension, centre local (0, 190, 0) mm, carries the stabilization handle. Handle centre local (0, 220, 38) mm, size 26 x 20 x 24 mm; its supporting stem is 24 x 20 x 18 mm, centre local (0, 220, 17) mm. |
| Grey target cap | Root is bottom centre, initially (0.29, 0, 0.7635); outer shell 42 x 42 x 50 mm, 45 g. Open 22 mm square cavity, 44 mm high. Raised 22 x 20 x 32 mm tab centred local (0, 0, 66) mm. |
| Orange loose spacer washer | Root is geometric centre (0.23, -0.22, 0.7545); 68 x 68 x 14 mm, 35 g, 22 mm square through-hole. Raised 9 x 20 x 32 mm tab centred local (-29.5, 0, 23) mm, clear of the cap shell. |
| Complete reference | Grey fixed fixture base centre (0.43, 0.19, 0.7555), otherwise matching geometry. Its fixed orange washer centre is 15 mm above the base centre; its fixed cap bottom is 22 mm above it. |
| Green parking pad | Fixed 90 x 90 x 6 mm, centre (0.17, -0.12, 0.7505). Other supported tabletop parking is allowed. |
| Washer pad | Fixed 90 x 90 x 4 mm, centre (0.23, -0.22, 0.7455), top flush with the table. |

Each loose assembly part has one rigid body and multiple box colliders. The real
washer opening and cap cavity remain open. Their 22 mm openings around the 18 mm
post provide **2 mm side clearance** at aligned orientation. Ordinary contact supplies
mechanical guidance and centering; there is no snapping, attachment, hidden locator
or episode-time body-pose write. The printed-part geometry supplies alignment support;
this does not establish visual precision at millimetre scale.

Reset restores authored poses, zero object velocities, READY arms, scorer state and
controller state. Use zero object jitter for the documented layout. The reference
cannot supply replacement parts or be moved. Do not drop any part off the table.
`contract.py` contains shared dimensions and trace datums; `generate_assets.py`
regenerates the included original USD assets.

## Observations and demonstration

Image-only review of actual final-revision 320 x 176 policy frames confirmed that the
white post and orange aperture edges are simultaneously visible before insertion.
The near-rim tab keeps the gripper below the relevant opening in that wrist view.
Later views show a coherent insertion, cap replacement and grey-over-orange
arrangement. Exact seating, hidden contact and numerical dwell requirements are
established by measured physics traces, rather than inferred from image pixels.
These observations do not establish camera-only model solvability.

The disclosed demo receives privileged measured object poses and commands the same
14 joint/gripper targets as Dream. It stabilizes the existing handle by downward
pressure with a closed left gripper, then lifts before opening. Placement converges
within 1 mm with a stable hold; release retains prior commanded arm/jaw targets and
ramps jaw opening at 0.2 normalized opening per second. Free carries are faster than
contact approaches. There is no arbitrary two-arm contact gate in the scorer.

## Reproduction

Use the prepared Linux environment from the public
[Robocurve simulation walkthrough](https://tasks.robocurve.org/walkthrough).
The validated runtime used Python 3.12.16, an NVIDIA L40S, Isaac Sim 6.1.0.0,
Inspect Robots 0.60.0, inspect-robots-yam 0.37.0, inspect-robots-agent 0.28.0,
inspect-robots-dream 0.1.0 and dreamlabs-client 1.0.14.

| Dependency | Validated revision |
| --- | --- |
| yam-isaac-lab-arena | `0059d02a1a91bc6834c1d0846022891df0c78730` |
| IsaacLab-Arena | `c7edf3a9d5d8276565208f72422f8f8a0806d6cf` |
| Isaac Lab submodule | `91a0906f93802eb859192dcb0bca879f891e92d6` |

The task verifies PhysX PGS global minimum and per-body position/velocity iterations
64/16 at startup and records effective values. All task fixtures are local included
USD assets. If the walkthrough's supplied station smoke instead fails on a staging
asset 404, inspect Arena's `assets/nucleus.py`: the validated baseline uses
`ARENA_NUCLEUS_DIR = ISAACLAB_NUCLEUS_DIR` for the production asset root. Apply that
correction only when the installed baseline needs it.

```bash
source /data/IsaacLab-Arena/yam.env
"$HOME/.local/bin/uv" pip install --no-deps -e /path/to/missing-washer-rework
TASK_TEXT=$(python -c 'from missing_washer_rework.contract import INSTRUCTION; print(INSTRUCTION)')
inspect-robots list embodiments
inspect-robots run --embodiment missing_washer_rework --policy missing_washer_rework_demo \
  -E demo_observations=true -E terminate_on_success=true \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E output_dir=outputs/demo/traces --instruction "$TASK_TEXT" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --grader none --no-prompt --no-rerun --no-rerun-save --log-dir outputs/demo/logs
```

For the consecutive-reset check, use `--epochs 2` and a new output/log directory;
keep `--seed 0` and zero jitter. This checks consecutive resets in one process.
For Dream, configure its credential privately as documented by the walkthrough,
apply a persistent request budget, and run:

```bash
inspect-robots run --embodiment missing_washer_rework --policy dream \
  -P model=dreamlabs/dream-1-beta -P timeout_s=300 \
  -E demo_observations=false -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  -E output_dir=outputs/dream/traces --instruction "$TASK_TEXT" \
  --scorer task_score --max-steps 2400 --epochs 1 --seed 0 --store-frames \
  --grader none --no-prompt --no-rerun --no-rerun-save --log-dir outputs/dream/logs
```

The station's calibrated overhead and two wrist cameras are the default; no crop,
extra viewpoint or privileged state is added for Dream. Choose fresh output paths
for each run. Both policies have a 2,400-action limit at 20 Hz, or 120 simulated seconds.

## Numeric scoring and independent replay

| Score | Measured conditions |
| --- | --- |
| 0.25 | Cap bottom clears the actual post top by at least 5 mm, then the cap is parked at least 60 mm from the post, released and supported on the table or pad for 0.3 s. |
| 0.5 | Subsequently seat the washer: actual post corners fit inside its aperture, upright within 5 degrees, centre within 4 mm of the post and height within 3 mm of its seated datum. |
| 1.0 | Also replace the cap with actual post containment, cap centre within 4 mm of both post and washer, and bottom within 3 mm of the washer top. Both grippers must be at least 0.8 open and both TCPs at least 100 mm from each assembly part. All parts remain stable for 1.5 s: reported and pose-derived speeds at most 10 mm/s and 0.1 rad/s. |

The fixture must remain within 20 mm of reset, upright within 5 degrees and supported
at its original table height within 3 mm. Reference motion beyond 5 mm or 3 degrees,
invalid measurement history, or any dynamic collider corner falling over 10 mm below
the tabletop permanently invalidates the episode. Losing current washer seating
removes its partial credit. See `scoring.py` for exact boundary and continuity tests.

```bash
python -m missing_washer_rework.regrade outputs/demo/traces/trial-0000/measurements.jsonl
inspect-robots video outputs/demo/logs/RUN.json
python -m pytest -p no:cacheprovider
```

CPU tests require NumPy, pytest and usd-core (`pxr`); no GPU is needed for those tests
or replay. The regrader reconstructs the outcome from raw measured poses, velocities,
TCPs, gripper openings and time, ignoring saved grades and policy phase labels.
Exports include `measurements.jsonl`, `metrics.json`, `run_config.json`, source hashes
and native setup/result camera PNGs. Preserve the exact executable source snapshot
with every trace; source and configuration checks accompany final evidence packaging.

Code and original task assets are MIT licensed; external dependencies retain their
own licenses. Rigid contact approximates lightweight printed parts without material
compliance, wear or threaded fastening. The three fixed-layout successes and one
Dream failure do not establish hardware validation, broad reliability or a clinical,
industrial or certified repair claim.
