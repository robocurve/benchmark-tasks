# Seal a drink on a manual cup sealer

A drink-shop counter task: set an open drink into a lever cup sealer's holder, pull the lever down and hold the
heated plate on the rim for 3 s, then let the lever back up, never leaving a gripper in the holder while the head
comes down.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `cup_sealer_seal.py` | `yam_arena` task file: scene, the per-step monitor, `success()`, `score()`, `distance()`, the scripted `demo()` and an offline self-test |
| `hingekit.py` | In-memory additions to `yam_arena`: parts built from several boxes and cylinders, and a spring-return hinge (a USD Physics revolute joint with a drive), per-shape friction, plus per-step and reset hooks |

## Offline check (no GPU)

From the repository root, with `yam_arena` importable:

```bash
python tasks/cup-sealer-seal/cup_sealer_seal.py
```

It runs the scoring rules on hand-built states (13 cases, including a cup tilted 0.6 degrees, a partial seal) and replays both arms' demo plans through the IK at 20 Hz.

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/cup-sealer-seal/cup_sealer_seal.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "Seal the drink." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sealer-dream

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/cup-sealer-seal/cup_sealer_seal.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "Seal the drink." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/sealer-demo
```

Runtime per trial: up to 2 minutes of robot time; the scripted demo finishes after about 34 s.

## Scoring

`monitor()` runs every control step (through `hingekit`'s step hook) and latches a spill or a gripper in the pinch
zone while the lever is below 15 degrees. It checks the plate against 36 points round the cup's rim (`rim_contact()`)
and grades the seal: full when all 36 stay within 1 mm of the plate for 3 s, partial for 27-35. It prints the grade
once reached, e.g. `[cup_sealer] seal grade: full (36/36 rim points within 1 mm of the plate for 3 s, at t=28.0 s)`. `score()` combines those with the cup's
and lever's current poses as `task.json` describes. The lever angle is read from the lever's pose.

## Scripted demo

The left arm grips the cup across its top band just under the rim (120 mm up, 4 cm above the drink's centre of mass, so it hangs upright) with its fingers pointing 50 degrees down (steep, so the pads hold it from swinging about the jaws), lifts it 15 mm over the
base, carries it under the raised head, lowers it into the holder, lets go and pulls straight back out of the
machine. Then the right arm turns its grip so the jaws close across the round handle bar, pulls the lever down along
its arc (slower near the rim; its targets run 16 degrees below the rim stop, since the arm's servos give some 8 degrees
under the spring), holds the plate on the rim for 4 s, guides the lever back up and lets go. The demo
extends `yam_arena.demo.ArmPlan.target` so a waypoint can carry a full gripper rotation (`rot`); stock waypoints are
unchanged.
