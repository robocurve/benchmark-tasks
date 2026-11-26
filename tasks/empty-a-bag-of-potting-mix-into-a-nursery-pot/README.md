# Empty an opened bag of potting mix into a nursery pot

![Starting layout](setup.png)

Two arms empty an opened, floppy plastic bag of damp potting mix into a 1-gallon nursery pot, then put the empty
bag down on the bench beside the pot. This is the everyday potting-bench move of home gardening and of nursery or
greenhouse work.

One hand holds the bag by a front corner near its mouth, and the other by the opposite front corner low on the
filled part. The hands carry the mouth over the pot, turn the bag over, and turn the filled end further over and
shake it until the mix that hangs up on the film falls out.

## Why a sagging bag is a different problem from a rigid pot

Pouring from a rigid two-handled pot (as in `pour-a-pot-with-both-hands`) means moving one rigid body along one
path. Both grips are fixed to the same object, and tipping it tips everything.

A bag is not one body. In this task it is simulated as real film, a PhysX cloth-like surface deformable.

- **Only the part full of mix keeps its shape.** The empty film above it folds wherever it is not held, and the bag
  narrows and folds as it empties.
  - Lift the bag by the mouth alone and the filled bottom swings down below it.
  - Lift it by the bottom alone and the mouth flops over and pours wherever it hangs, not into the pot.
  - Each hand controls a different part of the bag. The bag's shape between them is whatever the two hands make
    it.
- **The hands have different jobs, and the bottom hand can go further than a rigid bag would let it.**
  - The mouth hand can only turn the mouth so far over: its wrist roll stops at 120 degrees.
  - Because the film is limp, the bottom hand turns the filled end further over and higher, and the mix runs down
    through the folded film toward the mouth.
  - As the bag pivots, its mouth wanders, so both hands have to keep it over the pot.
  - A rigid pot can never be bent into this shape.
- **Turning it over is not enough.** On the shallow lower wall of the turned-over bag, the mix lies at a slope below
  its angle of sliding friction on the film, and stays there. The bottom hand has to shake it free.
- **The film limits how the hands may move.**
  - If the hands move apart, the film between them goes taut and pulls the seal out of a hand.
  - The filled bottom hangs about 15 cm below the hands and snags the pot's rim if it is carried low.
- **The bag must end up let go and clear of the pot.** Dropping the bag into the pot, or leaving it hanging over
  it, does not count.

## Run the demo

From the repository root, in the Isaac Lab / YAM environment (`conda activate robocurve`):

```bash
python tasks/empty-a-bag-of-potting-mix-into-a-nursery-pot/run_demo.py \
    --out "$HOME/yam-eval/potting-mix-$(date +%Y%m%d-%H%M%S)"
```

The runner records one trial with the fixed layout (seed 0) from three cameras. It exports MP4s into
`<out>/videos/` and writes `trace.jsonl` and `summary.json`.

- **Success:** the demo has succeeded only if `summary.json` reports a final `score` of 1.0. A finished video on its
  own does not show success.
- **Runtime:** a trial took about 20 minutes on one RTX 5090 shared with another trial. The film and 2,300 grains
  run on the GPU and need about 8 GB of its memory.
- **Disk:** raw frames take about 2 GB; delete `<out>/frames` after exporting the videos.

The policy (`bimanual_bag_demo` in `run_demo.py`) plays the joint targets that `bagplan.plan()` computes. It
corrects for the arms' sag under load using the observed joint positions, and limits every command to 0.045 rad per
step. It sends only ordinary joint targets: no welds, pose writes or added forces. `task.py`'s `demo()` gives the
same plan as TCP waypoints for `yam_scripted`.

To evaluate a policy on the task file directly:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/empty-a-bag-of-potting-mix-into-a-nursery-pot/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json;print(json.load(open("tasks/empty-a-bag-of-potting-mix-into-a-nursery-pot/task.json"))["instruction"])')" \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/potting-mix-dream
```

Offline checks of the scorer (NumPy only):

```bash
python -m pytest -q tasks/empty-a-bag-of-potting-mix-into-a-nursery-pot/test_scoring.py
```

The starting fill is made once, and never during an evaluation:

```bash
python tasks/empty-a-bag-of-potting-mix-into-a-nursery-pot/prepare_bag.py
```

It rewrites `bag_start.npz` and `mix_start.npz`.

## Results

| Run | Mix in pot | Left in bag | Spilled | Score |
|---|---|---|---|---|
| Scripted demo (`run_demo.py`, seed 0) | 788 g (98.3 %) | 0 g | 13 g (1.7 %) | **1.0** (succeeded at step 1,045 of 1,200) |
| The same plan without the shakes (`BAG_SHAKES=0`) | 636 g (79.4 %) | 102 g (12.7 %) | 63 g (7.9 %) | 0.5 |
| Dream-1 (`dreamlabs/dream-1-beta`, README command) | 0 g | 801 g | 0 g | 0.0 (did not grasp the bag; ran all 1,200 steps) |

Both hands let go of the bag and it ended clear of the pot in both scripted runs; the pot was never moved. The
inspect-robots evaluator reported `task_score` 1.0 for the demo.

GPU contact simulation varies from run to run. Two earlier recordings of the same plan (only the grain material
setting differed) scored 1.0 with 9 g and 5 g spilled, against the 24 g allowed. Check `summary.json` after each
recording.

## Files

| File | What it is |
|---|---|
| `task.py` | The task file: objects, mix, scoring hook, `demo()`. |
| `clothbag.py` | The bag's film mesh (its rest shape), the corner tabs, and the inside-the-bag test. NumPy only. |
| `bagkit.py` | Authors the film as a PhysX surface deformable attached to its tabs, puts it back at every reset, and reads it back. Also the pot. |
| `bagplan.py` | Layout and the scripted bimanual plan (joint targets at 20 Hz). |
| `scoring.py`, `test_scoring.py` | Where the mix ends up, by mass; the score levels; offline tests. |
| `soilkit.py` | The mix as a few thousand angular rigid grains, put back at every reset. |
| `prepare_bag.py`, `bag_start.npz`, `mix_start.npz` | How the starting fill is made, and the saved fill: film shape, 2,300 grains, 0.80 kg. |
| `run_demo.py` | Runs and records the demo. |
| `bodykit.py`, `fastik.py` | Shared helpers: rigid bodies from shapes; fast YAM kinematics. |
| `DESIGN.md` | Design notes, the real-world protocol in full, and the simulation choices. |

## Software

Conda `robocurve`: Python 3.12, `inspect-robots` 1.0.0, `inspect-robots-yam` 0.38.0, Isaac Sim 6.1, NumPy 2.3.
`run_demo.py` copies the task's sources into each run folder with their SHA-256 hashes.
