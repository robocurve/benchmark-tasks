# Mash cooked potato

Mash the cooked potato in the bowl until no large lumps remain. Keep the potato in the bowl.

The corrected 17 mm setup reaches 1.0 in the scripted demo (519 actions). Dream-1 scores 0.0 over 2,400 actions. Both native traces passed independent frozen-source rescoring and three-view visual review.

## Setup and reset

About 60 g of peeled, cooked potato in twelve 16–18 mm cubes (55–65 g total), arranged in a 4 × 3 grid in a stainless-steel bowl (200 mm inside diameter, 75 mm deep). A 140 g wire masher has a 75 × 60 mm head and 180 mm overall height and a 90 mm long, 20 mm diameter handle. The bowl is centred between the arms; the masher stands on its head 170 mm toward the right arm. Both hands start open.

The simulated portion is **58.32 g**: twelve clusters of 27 particles, each 0.18 g. Each cluster has a 17.0 mm axial envelope; the coarse particles approximate a solid cooked cube. Twelve ideal 17 mm cubes occupy 58.956 mL, giving a nominal density of 0.989 g/mL. The bowl is 300 g, centred 340 mm forward; cube centres are 20 mm apart. The tool may be released in the bowl.

Use peeled floury potato from one batch. Cut 19 mm raw cubes, simmer for 10–12 minutes until a fork passes through, drain for 2 minutes and cool to 18–22 °C. Select twelve intact cooked cubes with each edge 16–18 mm and combined mass 55–65 g. Reject pieces or the batch if both limits cannot be met. Add no milk, butter or oil. Place them in the photographed 4 × 3 grid. Use fresh pieces and clean the equipment and food-safe gripper covers each trial.

## Scoring

Losing over 5% of the starting potato mass scores 0. Otherwise weigh retained potato and lumps larger than 9 mm. The fine-mass fraction earns 0.25 at 50%, 0.5 at 75%, and 1.0 at 95% with both hands released for 3 seconds. At 95% while still held, score 0.5; holding never increases a lower score. All other outcomes score 0. Use a 0.1 g kitchen scale and 9 mm reference grid; measure each lump’s largest dimension.

For physical judging, collect and weigh the retained food without further mashing. Gently separate the pieces larger than the grid threshold and weigh them. The fine fraction is `(retained mass − oversized-lump mass) / retained mass`. Judge the largest dimension of each lump, not its orientation on the grid. The native equivalent measures connected components and their largest particle-to-particle extent plus particle diameter. All particles have equal mass.

## Material approximation

Each of twelve chunks contains 27 cohesive 5.56 mm particles. Native compression during masher contact reduces cohesion. Eight overlapping local cells per chunk measure compressive strain; full local failure removes that chunk's remaining cohesion, and otherwise the bond breaking force decreases. The rule never writes particle poses. Native contacts, gravity and friction handle the fragments after separation.

The parameters are an uncalibrated cooked-tissue approximation, not a measurement of potato rheology. Released particles represent small soft lumps, not a smooth paste. Contact-gated damage, 20/s translational damping, finite cohesion and the 0.5 m/s particle speed cap are identical for the scripted and Dream runs. The demo uses observed food and tool poses to aim ordinary bounded joint actions and stops pressing when the outcome is reached. A contact-gated compliant native joint approximates unresolved handle friction: 30N, 0.60Nm, 6000N/m linear stiffness and a physical pad-contact check at the handle. Its anchor follows the actual hand; no task-body pose is assigned. The grip approximation does not apply reaction forces to the robot.

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

The control limit is 2,400 actions (120 seconds at 20 Hz). The layout is fixed and uses no random seed. The scripted plan has a 2,360-action upper bound and can finish earlier once the measured outcome is released. Dream receives the full limit and standard three camera views plus joint observations.

The CPU rescorer verifies frozen source hashes, trace continuity, joint/rate bounds, damage progression, native break events and the outcome. It is an evidence audit, not a physics replay. Native camera videos must also be reviewed. `VALIDATION.json` records the audit and measurements. On the RTX PRO6000 Blackwell Server GPU, the revised demo took 301 seconds and Dream took 1,169 seconds. Dream used 99 API attempts across 50 SDK sessions, including 49 reported session-expiry recoveries. These recoveries continue the same physical episode; no scene reset or successful-action selection occurred.

The coarse cooked-tissue law accumulates irreversible damage only while a masher bar contacts a locally compressed cell. Normalized excess compression contributes damage per second, in addition to immediate peak-strain damage; unloaded waiting cannot soften food. This is an uncalibrated approximation of repeated crushing, independently reconstructed from every recorded pose. No food position is assigned by this law.

The 90 mm handle can slide within the hand while still touching both finite pads. A segment/box intersection checks the actual handle against both pad faces and their working area; grip ends when that contact is lost. The earlier 8 mm movement test incorrectly treated a chosen midpoint leaving its capture position as the whole handle leaving the fingers. Native 30 N and 0.60 Nm break limits remain unchanged.

The finite grip uses 8 N·m/rad angular compliance and a 0.60 N·m torque cap, an uncalibrated approximation of friction along the real handle contact. Its 30 N force cap, finite pad-face contact and opening/slip release remain active.
