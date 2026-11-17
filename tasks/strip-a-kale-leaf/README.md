# Strip a kale leaf from its stalk

Strip the edible leaf from a tough kale stalk, keeping the leaf on the board. One hand holds the bare stalk while the other strips the leaf. The score measures how much usable leaf is recovered, with zero credit for breaking the stalk.

## Setup and judging

Use one fresh Tuscan kale leaf, 250–290 mm long and about 80 mm wide, with a 5–6 mm stalk and 45 mm of bare stalk. Lay it flat on a 320 × 340 × 20 mm wooden board, bare stalk toward the left arm. Use washable food-safe gripper covers. The simulated board is centred at `(0.34, 0, 0.7575)` metres; its upper surface is 20 mm above the standard YAM table. `geometry.py` specifies the fixed layout.

Use a fresh leaf from the same batch each trial, washed and patted dry. Photograph it on a centimetre grid before restoring the starting layout. After a trial, spread the detached pieces on the grid and compare their area with the original leaf:

- **1.0:** at least 90% stripped and retained on the board, intact stalk at least 20 mm clear of the leaf, and both hands released for three seconds.
- **0.5:** at least 75% stripped and retained.
- **0.25:** at least 50% stripped and retained.
- **0:** all other outcomes, including any broken stalk.

Detached leaf outside the board earns no credit. Leaf fragments can count as recovered food; a torn stalk cannot. Photos and a ruler are sufficient for real judging.

## Simulation

The stalk and leaf are connected, colliding bodies with passive bending joints. Weaker leaf–stalk bonds break under load. Leaf fragments remain physical objects and can fall, slip or be discarded. The score weights patches by their original area. Material stiffness and tear loads are engineering approximations, not measurements of a particular leaf.

Both arms use ordinary YAM joint commands and contact grasps. Nothing attaches a leaf or stalk to a hand, and the controller never moves objects directly. The scripted demo and Dream trial use the same geometry, material model and scorer. Dream receives only the standard overhead and wrist RGB views and joint observations; extra cameras record judging views.

## Run and score

Use the pinned Python 3.12 / Isaac Sim 6.1 / YAM environment in `ENVIRONMENT.json`:

```sh
python tasks/strip-a-kale-leaf/run.py --mode demo --out /tmp/kale-demo
python tasks/strip-a-kale-leaf/run.py --mode dream --out /tmp/kale-dream
python tasks/strip-a-kale-leaf/rescore.py /tmp/kale-demo
python tasks/strip-a-kale-leaf/rescore.py /tmp/kale-dream
```

Configure Dream through its normal credential store or the documented loopback bridge in `dream_policy.py`. Do not put credentials in this folder.

The fixed demo plan is included; `make_plan.py` regenerates it. The limit is 800 actions at 20 Hz (40 seconds); the demo uses 440 (22 seconds). There is no layout randomization. Each run saves native videos, initial/final images, body poses, break events and a frozen copy of its source. Rescoring verifies source hashes, continuous bounded joint actions, break history and the final result. Inspect the video as well as the numerical score.

Validated on 17 November 2026: demo **1.0 / 440 actions**; Dream-1 **0.0 / 800 actions**, 16 requests. Native runtime was about 5.3 minutes for the demo and 13.1 minutes for Dream on a shared RTX PRO6000 validation batch. Both frozen traces independently rescore to the reported scores. `VALIDATION.json` records measurements and source hashes.
