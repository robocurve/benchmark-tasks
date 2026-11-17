# Remove an avocado stone

Remove the stone from an avocado half without crushing its flesh, then put the stone in the waste dish. The robot must grip the protruding stone while supporting the fruit gently.

## Setup and reset

Use a firm-ripe Hass avocado half, about 100 × 65 × 35 mm, with its 35–40 mm stone attached. Place it cut-side up on a 300 × 240 × 18 mm wooden board, narrow end toward the left arm. A 110 mm diameter, 24 mm deep waste dish sits 180 mm toward the right arm. Use clean, removable food-safe pad covers.

The simulated board is centred at `(0.34, 0, 0.7565)` metres and the dish at `(0.34, -0.18, 0.7495)`. `geometry.py` specifies the remaining fixed coordinates. Restore the photographed layout with both hands open.

Use a fresh half at 18–22 °C for each trial, from the same grocery batch. Record its size and ripeness. Reject split, overripe or already loose-stoned halves. Clean the board, dish and covers between trials.

## Judging

- **1.0:** the stone is fully removed and rests in the dish, the intact avocado remains on the board, and both hands have released for three seconds.
- **0.5:** the intact stone is fully clear of the fruit but has not reached the released full-score outcome.
- **0:** all other outcomes. A torn skin or a crushed/torn flesh patch larger than 1 cm² invalidates any credit.

Use the final video, a photograph and a ruler. The simulated partial-score clearance is 45 mm between fruit and stone centres. Each damageable flesh section exceeds 1 cm², so any broken flesh bond invalidates the run.

## Simulation

The skin, flesh sections and stone are colliding bodies. A passive bond resists extraction; separate flesh bonds allow damaging failure under excessive loading. The hollow socket has physical clearance around the stone. Non-spherical parts use convex meshes so their collision dimensions match their visible dimensions.

This is a coarse tissue model. The 7 N extraction and 12 N flesh-bond limits are uncalibrated engineering estimates; they do not establish a force threshold for every avocado. The real task is judged by the removed stone and intact fruit. The robot uses ordinary joint commands and native contact grasps, with no hand attachment or object-pose edits. Dream and the scripted demo share the same scene and material model.

## Run and score

Use the pinned Python 3.12 / Isaac Sim 6.1 / YAM environment in `ENVIRONMENT.json`:

```sh
python tasks/remove-an-avocado-stone/run.py --mode demo --out /tmp/avocado-demo
python tasks/remove-an-avocado-stone/run.py --mode dream --out /tmp/avocado-dream
python tasks/remove-an-avocado-stone/rescore.py /tmp/avocado-demo
python tasks/remove-an-avocado-stone/rescore.py /tmp/avocado-dream
```

Configure Dream through its normal credential store or the loopback bridge supported by `dream_policy.py`. Never put credentials in this folder. `make_plan.py` regenerates the included fixed joint plan.

The limit is 800 actions at 20 Hz (40 seconds); the demo uses 420 (21 seconds). There is no layout randomization. Runs save native videos, setup/final images, continuous body-state traces, material break events and frozen source files. Rescoring verifies source hashes, joint limits, event history and the outcome. The extra judging cameras are not policy inputs.

Validated on 17 November 2026: demo **1.0 / 420 actions**; Dream-1 **0.0 / 800 actions**, 16 requests. Native wall times were 5.9 and 14.6 minutes respectively on a shared RTX PRO6000 validation batch. Both complete traces independently rescore to these results. `VALIDATION.json` records measurements and source hashes.
