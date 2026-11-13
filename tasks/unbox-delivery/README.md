# Unbox a taped delivery parcel

Use the orange guarded cutter to sever the parcel's top seam and both taped end
folds, return the tool,
open both flaps, and place the blue item on the green pad. This targets receiving
and unpacking deliveries with tool use followed by access and retrieval.

## Status

The current handle-free, wrapped-tape scene has a complete sealed-start scripted
demonstration and unchanged repeat scoring **1.0**, independently replayed and
audited. Each uses 2,045 actions / 102.25 simulated seconds (289.90 / 292.59 wall
seconds on the reference VM). All 23 scorer/controller tests pass. These are two
fixed-seed repeats, not a broad reliability estimate. Dream-1 completed the full
3,600-action budget with score 0.0 and no tape cuts completed.

See `VALIDATION.json` for source hashes, measured milestones and audit results.

A related carton-cutting task, [PR #1856](https://github.com/robocurve/benchmark-tasks/pull/1856),
evaluates safe seam cutting and opening without product damage. This task additionally requires both wrapped end cuts, bimanual opening of plain flaps, and retrieval/placement of the contents. The objectives overlap; reviewers determine whether the difference is sufficient.

## Setup

Fixed seed 0, no jitter, standard bimanual YAM station and three policy cameras.
World x points away from the robot bases, y left, z up; tabletop z=0.7475 m.
`config.py` and `task.py` specify every initial pose, size, colour and mass.

- Plain cardboard parcel, 220 × 180 × 75 mm, 3 mm walls/bottom, fixed in a
  packing fixture. The five static panels form an actual hollow enclosure.
- Two plain 18 g top flaps, each 213.5 × 89 × 3 mm, hinged along the long outer
  edges, with a 2 mm centre seam and 0.25 mm end-corner clearance. There are no
  handles, pull tabs or extra gripping colliders. Each hinge permits 170° outward
  folding, so an opened flap can rest clear of the opposite wrist. The end walls extend to
  the lid; there are no separate inner minor flaps in this simplified carton.
- A 48 mm tape strip spans the full 220 mm lid and wraps 40 mm down both end
  walls. All three load-bearing cuts are required: the long seam and a transverse
  cut at each end fold. The wall tails remain in place after those cuts.
- Orange 55 g guarded package cutter: 110 × 25 × 25 mm handle, 25 × 30 × 20 mm
  guard, and a 14 × 0.6 × 8 mm metal cutting tooth. Initial root (0.13, 0.19, 0.7695).
- Blue item, 40 mm cube, 60 g, inside the parcel at (0.27, 0, 0.7705).
- Orange cutter pad, 170 × 70 × 2 mm, with an 80 × 40 × 7.5 mm
  handle cradle; green receiving pad, 100 × 100 × 2 mm.

The carton fixture is part of this task's setup. It makes no claim to evaluate
holding an unconstrained delivery box with the other arm.

## Run

In the installed Arena virtual environment, from this folder:

```bash
python run.py --policy demo --max-steps 3600 --log-dir ~/yam-eval/unbox-demo-001
python run.py --policy dream --max-steps 3600 --log-dir ~/yam-eval/unbox-dream-001
```

Use a **new directory for each attempt**. `run.py` is required to install the
compound cutter, physical flap joints, tape mechanics and measured-state audit.
The stock one-file command deliberately fails rather than run an unsealed box.
The prototype uses 20 Hz control and standard action-range clamping. Audits
independently check command rates against the configured actuator velocities. It
accepts 1–3600 steps; 3600 steps is 180 simulated seconds. Full-score runtime for
the revised scene is 102.25 simulated seconds in the validated demonstration.

Export each finalized Inspect log with `inspect-robots video RUN/adhoc_ID.json`.
The runner also writes `setup.png`, `trajectory.jsonl`, `score-summary.json`,
source snapshots, controller events and SHA-256 hashes. Reset is a fresh process with intact tape,
closed flaps, and every object restored to its fixed starting pose.

## Handle-free opening controller

The scripted demonstration uses privileged measured poses and ordinary joint/
gripper commands. Its strategy depresses one ordinary flap to expose
the cut edge of the other. The opposite arm then pinches that cardboard edge;
the supporting arm clears before lifting. Grip preload is preserved during
motion, and loss/stall detection bounds retries. The first flap falls outward
after being lifted past vertical; the opposite arm then pinches the second
ordinary cut edge. Both released flaps rest at about 170°.

The flaps have a physical inward range as well as the outward opening range;
box walls and the payload provide contact constraints. The rectangular panels
fit between the end walls so their corners do not intersect the wall colliders.

An explicitly pre-cut opening diagnostic earns zero task credit:

```bash
python run.py --policy demo --stage opening --max-steps 1800 --log-dir ~/yam-eval/unbox-plain-open-001
```

Every diagnostic row is marked and its task-success callback is disabled. Only
complete sealed-start runs will count toward revised full-score feasibility.

## Scoring and mechanics

Final-state scoring with ordered milestones:

- **0:** any of the three required cuts is incomplete, or an invalid outcome.
- **0.25:** all 316 one-millimetre cells in 30 tape segments, across all three
  H-shaped cut paths, cut by measured held-cutter sweeps.
- **0.5:** after cutting, both flaps have reached at least 80° open.
- **1:** after opening, both flaps remain at least 70° open, the item is placed
  within 25 mm in x and y of the receiving-pad centre, its centre is within 8 mm
  of its resting height, and its speed is below 10 mm/s for one second. Both
  TCPs must be over 65 mm from the item. The cutter must be within 45 mm of
  its pad centre and 25 mm of its initial height, move slower than 20 mm/s,
  and no longer be held.

Tape cuts require the measured blade tip within 6 mm laterally and 4 mm
vertically of each cut path, blade-down component above 0.9, blade-forward
component along that path above 0.85 in magnitude, and cutter handle
within 35 mm of a measured TCP with 8–38% jaw opening. Successive valid samples
must be at most 100 ms apart, blade speed 2–250 mm/s, and along-path motion at
least 85% of total tip motion. Short passes accumulate
coverage; lifting or leaving the seam cannot bridge an uncut region.

Forcing a flap more than 12° while tape remains, dropping the item centre more
than 20 mm below the table, dropping the cutter centre more than 30 mm below
the table, or nonfinite telemetry invalidates all credit, permanently.

The tape film is visual geometry without rigid colliders. Its fracture latch
physically locks both revolute hinges until all segments are cut.
Completing all three cuts releases their angular limits; robot contacts must then move the flaps.
The runner does not teleport the cutter, item or flaps or attach them to a hand.
Darkened tape segments visualize the measured cuts.

## Fidelity and limitations

This is rigid cardboard with ideal hinges and an ideal fixture. Tape is a
geometric blade-sweep fracture approximation releasing a hinge latch, not a
calibrated adhesive, fracture-force or flexible-strip simulation. Grip proximity
and jaw opening are a grasp proxy, not contact-force confirmation. Cutting depth,
cardboard tearing, tool/item damage and real material friction are not validated.
The cutter is a small guarded package-opening tool, not an unrestricted knife.
Real cardboard and tape need calibration before claiming physical fidelity.
Fixed-layout full-score feasibility is demonstrated, not broad reliability.
The support-hand retreat has a maximum planned Cartesian IK residual of 24.6 mm;
the measured hand nevertheless clears and parks before the flap lift. Do not
interpret planned IK residuals as physical tracking precision. Material
calibration and novelty resolution remain open.

## Tests and evidence

```bash
python3 -m unittest -v test_scoring
# In the installed Arena environment:
python -m unittest -v test_scoring test_controller
```

The scorer rejects seam-only cuts, a missing end-fold cut, wrong blade alignment,
air jumps, forced opening before all cuts, and invalid final placement.
`geometry-audit.json` verifies exactly one cardboard collider per flap.

Current complete-run evidence is listed in `VALIDATION.json`.
