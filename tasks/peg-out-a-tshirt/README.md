# Peg a T-shirt onto a clothesline

Lift the flexible shirt, operate two spring pegs and leave it hanging clear of
the table. The pegs must capture actual fabric and line, at least 100 mm apart.

## Run

Use Python 3.12 and the Isaac/YAM environment in `ENVIRONMENT.json`, including
its public asset-root patch. Configure Dream credentials through the SDK.
From the repository root, choose a new output directory:

```sh
python tasks/peg-out-a-tshirt/run_task.py --policy demo --out DEMO
python tasks/peg-out-a-tshirt/run_task.py --policy dream --out DREAM
```

The fixed reset has no randomization. Both policies receive the same five-second
gravity settling, flexible mesh, pegs and contact model. Control is 20 Hz; physics
is 240 Hz with 128 solver iterations. The planned demo uses 2,160 actions, with up to 240 extra alignment actions;
the total limit is 2,400
(120 seconds), including the final five-second released hold. Dream sees only
the standard cameras and joint observations. The demo measures fabric and peg
poses to produce ordinary bounded joint commands. Judging cameras are excluded
from policy input. Run one Dream trial at a time per API key. The baseline requests two actions
at a time to limit idle gaps. Official SDK session recovery is enabled and
logged; more than 12 expiries stops the trial. Recoveries never reset the scene.

The policy receives the three ordinary camera observations at 224 × 224;
the Dream adapter resizes them to 320 × 176. The recorded 960 × 720 judging
views are separate and are never policy inputs.

## Setup and approximation

`task.json` gives the garment, rack, support and peg dimensions and their fixed
positions. `scene.py`, `line_pegs.py` and `arena_props.py` generate those objects.
The dry child-sized shirt is 180 g/m² cotton, 33.9 g, with two sewn layers. Its
2.5 mm surface-FEM grid bends, stretches, falls and self-collides; it is not rigid.
The laundry board is 100 × 440 × 155 mm and the empty pegs start at Y=0 and +60 mm.
Effective cloth stiffness is approximate and can retain pronounced folds.
Sub-millimetre contact error is treated as numerical compliance; visible cloth
passing through itself or a fixture still invalidates the trial.

A contact-driven law approximates spring-peg operation. Fingers must enclose
the handles in the correct compression direction. Squeezing opens the jaws;
releasing retains only native cloth vertices actually inside the mouth with
the line. A missed line or fabric capture can fail. Opening releases the cloth.
A native static-grip constraint also approximates dry finger contact; opening
the hand releases it. Grip slip and spring forces are not calibrated. Free peg
fall is simplified. No demo phase, intended hem location or score selects a grip.

The rest of the garment remains native flexible cloth with gravity and
self-contact. `JUDGING.md` requires a released five-second hold, clear table
separation, actual jaw capture and intact cloth. An attachment flag alone does
not establish a successful fastening. The historical runner name and diagnostic
report label do not assign a task score; final media and reviewed measurements
are bound separately in `VALIDATION.json`.

Fabric tearing is not modelled; visible material passage or other damage still
invalidates a trial. The model tests handling and fastening, not fabric strength.

## Score and evidence

The judging record contains reviewed video/photo decisions and measured
clearances. `scoring.score(record)` applies the physical rubric; `judge.py`
also checks the evidence files against their hashes.

```sh
python -m unittest discover -s tasks/peg-out-a-tshirt -p 'test_*.py'
```

The demo scores **1.0** in 2,160 actions (108 simulated seconds); Dream-1
scores **0.0** after 2,400 actions. On a shared A40, the recorded wall times
were 3,715 and 4,450 seconds. Shared GPU load affects runtime.

The native final hold retains actual fabric in both pegs, 114 mm apart, with
at least 56 mm table clearance and both hands clear. The garment brushes a
vertical post but rests on neither its top nor foot. Its pronounced folds
reflect approximate stiffness. `VALIDATION.json` records the reviewed evidence.

Reproduce both scores without a GPU:

```sh
python tasks/peg-out-a-tshirt/judge.py tasks/peg-out-a-tshirt/evidence/demo-judge.json
python tasks/peg-out-a-tshirt/judge.py tasks/peg-out-a-tshirt/evidence/dream-judge.json
```
