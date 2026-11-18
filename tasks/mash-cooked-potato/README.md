# Mash cooked potato

Mash the cooked potato in the bowl until no large lumps remain. Keep the potato in the bowl.

The native demo scores 1.0 in 723 actions (36.15 seconds). Dream-1 scores 0.0 after the full 2,400-action limit. Both runs pass independent frozen-source audits and their native camera videos were inspected.

## Setup and reset

About 60 g of peeled, cooked potato in twelve roughly 22 mm cubes, arranged in a 4 × 3 grid in a stainless-steel bowl (200 mm inside diameter, 75 mm deep). A 140 g wire masher has a 75 × 60 mm head and a 180 mm handle. The bowl is centred between the arms; the masher stands on its head 170 mm toward the right arm. Both hands start open.

The native bowl is 300 g. Its centre is 340 mm forward of the station origin; the food grid has 24 mm spacing. The fixed layout, part geometry and masses are in `geometry.py`. The simulated portion is 58.32 g. The tool may be released in the bowl; returning it to the bench is not required.

Use peeled floury potato from one batch, cut into roughly 22 mm cubes, boiled until a fork passes through easily, drained and cooled to 18–22 °C. Weigh 60 ± 5 g per trial; add no milk, butter or oil. Use fresh pieces and clean the bowl, masher, work surface and food-safe gripper covers between trials. Place the twelve pieces as photographed.

## Scoring

First weigh the retained potato and the lumps whose largest dimension exceeds 9 mm. Losing over 5% of the starting mass scores 0. Otherwise: 1.0 if at least 95% of retained mass is in lumps no larger than 9 mm and both hands have released for 3 seconds; 0.5 for at least 75%; 0.25 for at least 50%; otherwise 0. A qualifying result still held scores 0.5. Use a 0.1 g kitchen scale and a 9 mm reference grid.

For physical judging, collect and weigh the retained food without further mashing. Gently separate the pieces larger than the grid threshold and weigh them. The fine fraction is `(retained mass − oversized-lump mass) / retained mass`. Judge the largest dimension of each lump, not its orientation on the grid. The native equivalent measures connected components and their largest particle-to-particle extent plus particle diameter. All particles have equal mass.

## Material approximation

Each of twelve chunks contains 27 cohesive 7 mm particles. Native compression during masher contact reduces cohesion. Eight overlapping local cells per chunk measure compressive strain; full local failure removes that chunk's remaining cohesion, and otherwise the bond breaking force decreases. The rule never writes particle poses. Native contacts, gravity and friction handle the fragments after separation.

The parameters are an uncalibrated cooked-tissue approximation, not a measurement of potato rheology. Released particles represent small soft lumps, not a smooth paste. Contact-gated damage, damping, finite cohesion and the 2 m/s particle speed cap are identical for the scripted and Dream runs. The demo uses observed food and tool poses to aim ordinary bounded joint actions and stops pressing when the outcome is reached; no hand attachment or success-triggered object motion is used.

SPARTA evaluates food mashing: https://arxiv.org/html/2509.24129v2 . Its overlap is an unresolved novelty risk. Tool pickup, retention scoring or changing the food name do not by themselves establish novelty, and this submission makes no contrary claim.

## Run and rescore

Use the pinned Python 3.12 / Isaac 6.1 / YAM installation in `ENVIRONMENT.json`, with the adapter installed. From the repository root:

```sh
python tasks/mash-cooked-potato/make_plan.py
OMNI_KIT_ACCEPT_EULA=YES python tasks/mash-cooked-potato/run.py --mode demo --out /tmp/potato-demo
OMNI_KIT_ACCEPT_EULA=YES python tasks/mash-cooked-potato/run.py --mode dream --out /tmp/potato-dream
python tasks/mash-cooked-potato/rescore.py /tmp/potato-demo
python tasks/mash-cooked-potato/rescore.py /tmp/potato-dream
```

The control limit is 2,400 actions (120 seconds at 20 Hz). The layout is fixed and uses no random seed. The scripted plan has a 2,260-action upper bound and can finish earlier once the measured outcome is released. Dream receives the full limit and standard three camera views plus joint observations.

The CPU rescorer verifies frozen source hashes, trace continuity, joint/rate bounds, damage progression, native break events and the outcome. It is an evidence audit, not a physics replay. Native camera videos must also be reviewed. `VALIDATION.json` records the audit and measurements. On an RTX PRO 6000 Blackwell, the demo took 390 seconds and Dream took 1627 seconds including 50 policy requests. Runtime varies with API latency.
