# Hoist a kettlebell hand over hand with a rope over a ratchet block

**Instruction:**
> Raise the kettlebell with the rope over the pulley until its base is above the red line on the board, and hold it
> there. Don't touch the kettlebell or let it run back down.

A 4 kg cast-iron kettlebell stands on a rubber mat under a 75 mm ratchet block hung from an aluminium-extrusion frame.
A 12 mm rope is tied to its handle with a bowline, runs up over the block, and comes back down onto the table in front
of the arms. A white board between the frame's uprights has tape lines every 10 cm above the mat; the 40 cm line is red.

The ratchet block's sheave turns freely when the rope is hauled in and locks when the load pulls back. The rope must
then slip round the locked, grippy sheave, so a hand needs only a small part of the kettlebell's weight to hold it.
One arm can hold the kettlebell but cannot haul it up. To raise it, the two arms must haul together, then take turns:
one hand holds while the other lets go and takes a new grip higher up the rope. A hand that lets go at the wrong
moment, or both hands opening at once, lets the kettlebell run back down. This is ordinary hand-over-hand hauling, as
done by construction laborers raising materials to a scaffold, riggers, arborists raising gear and deckhands.

## Setup (real world)

Every item is a standard, buyable part at full size. Positions are from the midpoint between the two YAM arm bases:
x forward, y left, heights above the table top.

| Item | Specification | Placement |
|---|---|---|
| Frame | 40 x 40 mm aluminium T-slot extrusion: two base rails 27 cm long (x 0.62-0.89 m), two uprights 90 cm tall at x 0.80 m, y +-0.22 m, a crossbar on top, and a 23 cm jib running forward from the crossbar's middle to x 0.57 m. Corner brackets; each base rail held to the table with two C-clamps | Fixed |
| Ratchet block | 75 mm (3 in) single sailing ratchet block for 10-12 mm line, ratchet switched ON, hung from the jib on a shackle and swivel so its sheave axle runs along y | Sheave axle at x 0.60 m, height 0.76 m |
| Rope | 12 mm double-braid polyester, 1.75 m | Tied to the kettlebell handle with a bowline, over the sheave from the far side, straight down its near side, then along the table toward the arms (y = 0), its free end 0.20 m in front of the arm bases |
| Kettlebell | 4 kg cast-iron kettlebell | Base centred at x 0.66 m, y 0, on the mat |
| Mat | 10 mm rubber mat, 20 x 20 cm | Centred at x 0.68 m, y 0 |
| Board | 12 mm white melamine board, 40 cm wide x 78 cm tall, fixed between the uprights flush with their front faces | Tape lines 12 mm wide with their **lower edges** 10, 20, 30 and 40 cm above the mat; the 40 cm line red, the others black; a 1 cm tick ladder up the board |

## Scoring

Measure the height of the kettlebell's base above the mat against the board's lines, from a side video.

| Outcome | Score |
|---|---|
| The base above the red 40 cm line continuously for 2.0 s | **1.0** |
| Otherwise, the highest line the base has reached: 30 / 20 / 10 cm | **0.75 / 0.5 / 0.25** |
| Below the 10 cm line | 0 |
| At any time: a gripper touches the kettlebell, or takes hold of the rope between the kettlebell and the block, or the base falls more than 10 cm below the highest it has reached (the load ran back) | **0** for the run |

The run ends at full score or at the run limit. In simulation the base height is the lowest point of the kettlebell's
base and underside, touching means the gripper's centre within 2 cm of the kettlebell's surface, and taking the rope
means the gripper's centre within 3 cm of a rope link between the kettlebell and the block.

## Simulation model

- **Rope:** 80 rigid capsule links (22 mm long, 12 mm across) in one reduced-coordinate articulation with the
  kettlebell, so the rope can't stretch. Each link weighs 20 g, heavier than real braid, to keep the solver stable. A
  braided skin is redrawn over the links every step.
- **Ratchet block:** the sheave is a free wheel on a drivable axle. The pawl is a stiff spring (400 N m/rad). It
  engages the moment the sheave turns back and lifts once the sheave has been pulled the hauling way for two control
  steps. Rope-on-sheave friction is 1.5, so that with the coarse link contacts the locked sheave carries most of the
  load. Checked behaviours:
  - one hand holds the kettlebell;
  - one arm alone cannot raise it (`HOIST_MODE=single_left` stalls at 0 cm);
  - with both hands off, the rope slips round the locked sheave and the kettlebell runs down
    (`HOIST_MODE=release_test`).
- **Grip:** finger contact on a rigid-link rope slips or jams under load in PhysX. So while a gripper's fingers are
  closed on the rope (gap 5–15.5 mm, a rope link within 25 mm of the gripper centre), that link is held to the gripper
  by a spring-damper (2,000 N/m, about 2 cm of give at 40 N). It lets go when the fingers open. The rule is the same
  for every policy.

## Run and score

From the repository root, with Isaac Lab-Arena, `yam-arena` and Inspect Robots installed:

```bash
python tasks/hoist-heavy-load-with-pulley/run_demo.py --out ~/yam-eval/pulley-demo        # scripted demo + videos
python tasks/hoist-heavy-load-with-pulley/run_demo.py --policy dream --out ~/yam-eval/pulley-dream
```

or directly:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/hoist-heavy-load-with-pulley/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Raise the kettlebell with the rope over the pulley until its base is above the red line on the board, and hold it there. Don't touch the kettlebell or let it run back down." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pulley-dream
```

`run_demo.py` also records three fixed review views to `videos/review/` (`side_left`, `front_right`, `wide_back`:
the block, rope, board and kettlebell from the side and from further back). They are for reviewers only: their images
never reach the policy and don't change the physics. `--no-review-cams` turns them off.

Offline checks (no simulator): `python -m pytest -q tasks/hoist-heavy-load-with-pulley/test_task.py` (scorer) and
`python tasks/hoist-heavy-load-with-pulley/check_plan.py` (the demo's reach).

Run limit: 2,400 steps (2 minutes at 20 Hz). One fixed start; no seeds or randomisation. A trial takes about 15 min
of wall time on one RTX 5090 (the 80-link rope steps at 240 Hz); the scripted demo succeeds at step 992.

## Results

| Policy | Score |
|---|---|
| Scripted demo (`run_demo.py`) | **1.0**: five two-hand hauls with four hand changes; the base reaches 45.8 cm and stays above the red line for 2 s (step 992) |
| Dream-1 (2,400 steps) | 0.0: the kettlebell never leaves the mat |

## Reset

Lower the kettlebell back onto the mat (switch the ratchet off or lift the rope off the sheave to let it down), stand
it at its mark, lay the rope back along the table to its start, and switch the ratchet on. Nothing is consumed.

## Files

| File | Purpose |
|---|---|
| `task.py` | Task file: per-step state, scorer, the ratchet's pawl, rope rendering, scripted demo and check modes |
| `parts.py` | Scene: mat, frame and board, ratchet block, kettlebell and rope |
| `hoistkit.py` | The ratchet block (a fixed-base articulation with a drivable axle) and the kettlebell + rope articulation |
| `kit.py`, `meshes.py`, `ropeskin.py` | Rigid bodies, procedural meshes and the rendered rope skin |
| `run_demo.py` | One trial with videos, a per-step `trace.jsonl` and `summary.json` |
| `review_cams.py` | Three extra fixed cameras (side and wide views) recorded for review only, never a policy input |
| `check_plan.py`, `test_task.py` | Offline reach check of the demo; scorer tests |

Check modes for development (never set in scored runs): `HOIST_MODE=single_left|single_right` (one arm hauls alone),
`hold_test` (one hand holds alone), `release_test` (both hands let go); `HOIST_RATCHET=0` turns the pawl off.
