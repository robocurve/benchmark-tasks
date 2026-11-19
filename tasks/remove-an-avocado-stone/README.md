# Remove an avocado stone

Remove the stone from an avocado half without crushing its flesh, then put the stone in the waste dish. The robot must grip the protruding stone while supporting the fruit gently.

## Setup and reset

Use a firm-ripe Hass avocado half, about 100 × 65 × 35 mm, with its 35–40 mm stone attached. Place it cut-side up on a 300 × 240 × 18 mm wooden board, narrow end toward the left arm. A 110 mm diameter, 24 mm deep waste dish sits 180 mm toward the right arm. Use clean, removable food-safe pad covers.

The simulated board is centred at `(0.34, 0, 0.7565)` metres and the dish at `(0.34, -0.18, 0.7495)`. `geometry.py` specifies the remaining fixed coordinates. Restore the photographed layout with both hands open.

Use a fresh screened half at **18–22 °C**, within **10 minutes of cutting**. Accept fruit halves 95–105 mm long, 60–70 mm wide and 30–40 mm high, with a 35–40 mm stone. Cut around the equator without scoring the stone; separate the halves gently and retain the half in which the stone stays attached. Do not loosen and reseat it. Reject split skin, brown/mushy flesh, a visibly detached stone or a gap around the stone exceeding 1 mm.

### Quick mechanical screening

The evaluator prepares fruit before the robot trial. Use a 0–20 N digital push/pull gauge (0.1 N resolution), a flat-ended 8 mm diameter probe, ruler, a padded cradle and a small silicone-lined stone clamp. These are preparation tools, removed before frame zero. Check the gauge with a 100 g weight (approximately 0.98 N).

1. **Flesh, on the discarded companion half:** support its skin in the cradle. At two locations halfway between skin and empty stone socket, press the probe 3 mm into exposed flesh over 3 seconds. A 3 mm collar on the probe limits depth. Accept peak force **2–5 N at both locations**, with readings differing by at most **1.5 N**. The probed half is discarded; the trial half is not indented. Record both readings.
2. **Attachment, on three sacrificial fruit per batch:** apply the same flesh and size screen, support the skin without squeezing the flesh, grip only the protruding stone and pull straight out over about 5 seconds. Accept the batch only if all three peak extraction forces are **5–9 N**, no flesh patch over 1 cm² tears away, and the maximum/minimum force ratio is at most **1.5**. Discard these fruit; never reuse their loosened stones. Repeat after two hours or any change of supplier batch, storage temperature or firmness range.
3. **Each trial stone, without extracting it:** support the fruit in the cradle and clamp only the exposed stone. Ramp an outward **1.0 ± 0.1 N** load over 2 seconds, hold for 1 second, then unload. Watch a close side view with a ruler. Reject a stone that moves over **1 mm**, separates visibly, or does not return to its initial position within **1 mm**. Release the clamp without twisting. Do not probe beneath the stone or cycle this load repeatedly. A rejected half is not reseated or used for a trial.

Steps 1 and 3 take less than a minute per prepared fruit; qualified spare halves can be queued within the ten-minute cut-to-trial window. Keep the uncut batch at the same temperature. Record batch ID, temperature, size, the two firmness readings, the three batch extraction forces and the per-stone proof result. Wash the board, dish, preparation tools and removable food-safe pad covers between fruit.

These numeric windows define a reproducible selection protocol, not universal ripeness constants. **A nondestructive proof load establishes a lower bound, not the exact release force of that stone.** The 5–9 N range is checked on sacrificial batch samples; the companion-half check reduces within-batch variation without touching trial flesh. Real-fruit qualification of these chosen windows has not been measured by the authors. Do not claim that passing the screen proves an individual stone will release at 7 N. If a batch cannot meet the windows, reject it rather than adjusting limits during a comparison.

Testing exposed flesh with an 8 mm probe follows ordinary avocado firmness measurement practice ([World Avocado Congress study](https://www.avocadosource.com/wac3/wac3_p331.pdf)); our shallower depth, force windows and paired-half protocol are task specifications, not results from that study. The study also cautions that even nominally nondestructive compression can soften ripe fruit, which is why the firmness test is confined to the discarded half.

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
