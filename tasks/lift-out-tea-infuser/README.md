# Lift a steeped infuser out of a glass teapot by its thin handle bar, shake off drips, set it on a saucer, lid back on

A glass teapot stands on the table with its lid on and a stainless steel infuser basket of damp, steeped tea leaves
standing in it. The robot takes the lid off, pinches the thin handle bar across the top of the basket and lifts the
basket clear of the pot, shakes it up and down three times over the pot so the drips fall back into it, sets the basket
on the saucer with the leaves still in it, and puts the lid back on the pot.

Tea is drunk by over 159 million Americans on a given day (Tea Association of the USA, via the US Census Bureau,
census.gov/newsroom/stories/hot-tea-month.html). With loose leaf in a glass teapot, lifting the infuser out ends the
steep, and a few quick shakes over the pot keep its drips off the table.

## What is distinctive

**The task.** An everyday, instantly recognisable routine whose difficulty is physical and concentrated in one step:
getting a basket out of a pot through a tight opening by a handle that is barely there. The pot (full of water) must not
be disturbed, the leaves must stay in the basket, and the lid has to go back on.

**The skills it needs, none of which is a plain pick-and-place:**
1. *A precision pinch on a recessed, thin feature.* The only handle is a 2.4 mm flat bar, 11 mm tall, whose top sits 4 mm
   below the rim of a 9.6 cm mouth. The fingers have to go down the mouth to reach it, centred on the bar.
2. *Extraction through a tight opening.* The 8.8 cm basket clears the 9.6 cm mouth by under 4 mm a side, so it has to
   come up straight, without catching the rim or tipping the leaves.
3. *Reach with a tilted hand.* Straight down, the arm cannot lift the basket clear; the demo pinches with the hand tilted
   60 degrees forward and closing along the bar's thin axis.
4. *Control of a suspended load.* Hanging from a pinch on a thin bar, the basket swings like a pendulum. Three quick
   2.5 cm strokes have to be made without the basket tipping, the grip slipping or the leaves leaving it.
5. *Setting down and re-seating a loose lid by its 27 mm knob*, to within 6 mm of the pot's axis, with two arms working
   round the same pot without disturbing it.

The shake is what the real step is for: the leaves and the basket are damp and drip when lifted, and shaking sheds the
drips back into the pot instead of onto the saucer and the table.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, scoring (`score_status`, `status`, the stroke counter `Shake`), `success()`, `self_test()` and the scripted `demo()` |
| `scenekit.py`, `soilkit.py`, `tiltkit.py` | USD bodies and grains for yam_arena task files (hooks yam_arena without changing it); the tilted grip used by the demo (shared helpers) |
| `assets/teapot.usdc` | Teapot body, spout and handle, one free rigid body (body collider: a disc floor and 360 boxes following its inner profile; spout and handle as convex hulls) |
| `assets/lid.usdc` | The lid and knob, one free rigid body (convex hulls) |
| `assets/infuser.usdc` | The infuser basket, one free rigid body (collider: a disc floor, 36 wall boxes, 36 rim boxes, a bar box) |
| `assets/saucer.usdc` | The saucer, one free rigid body (convex decomposition) |
| `assets/leaves_start.npz` | The 200 leaves' start layout (centres, radii) in the basket |
| `scripts/make_assets.py`, `scripts/make_leaves.py`, `scripts/naive_tea.py` | Build the four asset files from the two USDCraft-10K models and the leaf layout; the wrong-strategy demo |

Third-party assets: NVIDIA USDCraft-10K (huggingface.co/datasets/nvidia/USDCraft-10K), licence Apache-2.0, objects
`ds_R089_03` (glass teapot with infuser basket) and `ds_R086_06` (glass tea cup and saucer). `scripts/make_assets.py`
splits the models into the four bodies above and replaces their colliders; `scripts/make_leaves.py` writes the leaf layout.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the table top.

| Object | Where | What |
|---|---|---|
| Teapot | foot centred (36.0, 0.0), handle towards the robot's left, spout to the right | Clear borosilicate glass, round belly 15 cm across, 15 cm tall to the rim, mouth 9.6 cm across (inside), 24.2 cm long with spout and handle, 0.40 kg empty, holding 500 g of room-temperature water (its ballast; it must not be disturbed) |
| Lid | on the rim, 18.0 cm above the table at the knob | Clear glass, 10.5 cm across, 2.7 cm round knob, 0.08 kg |
| Infuser | standing on the pot's floor, axis on the pot's | Stainless steel mesh basket 8.8 cm across at the rim, 13 cm deep, 0.139 kg; a flat handle bar across its top, 8.7 cm long, 2.4 mm thick, 1.1 cm tall, its top 4 mm below the pot's rim, running forward-back (x) |
| Leaves | in the bottom 2 cm of the basket | About 23 g of damp steeped black tea leaves (8 g dry, 3 minutes); they and the wet basket drip when lifted |
| Saucer | centred (30.0, 23.0) | Clear glass, 14.2 cm across, 1.4 cm tall |
| Ring mark | on the table round the pot's foot | 4 cm radius, to show whether the pot shifted |
| Drip mat (real trials) | under the pot and the saucer | Waterproof, 60 x 40 cm |

Sources: model sizes, USDCraft-10K `ds_R089_03` and `ds_R086_06` (above); a 1 L glass teapot with a stainless infuser is
an ordinary retail item (retailer listings give about 22 x 13.5 x 14.5 cm for a 1 L glass pot).

## Testing in simulation

Run a model trial (Dream-1 or any policy) from this folder, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the steeped infuser out of the teapot: lid off, basket out by its handle bar, shaken up and down three times over the pot to shed the drips, set on the saucer, then the lid back on the pot." \
  --scorer task_score --max-steps 2000 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/lift-out-tea-infuser-dream
```

`task_score` prints the score (1.0 on success, otherwise `score()` at the end of the episode). Seed 0; the scene has no
random placement (`-E object_jitter=0`). About 4 minutes per trial including start-up on one GPU. `TEA_LOG=1` prints, every
0.5 s, the basket's height and position, the stroke count, whether it has been shaken, whether it is out, on the saucer, the
share of leaves kept, whether the lid is on, and the score.

In simulation the score is computed from the bodies' poses at every control step: the pot's shift and tilt (latched), the
basket's position over or away from the pot and saucer, the share of the 200 leaf pieces still inside it, the basket's
height trace (the stroke counter `Shake` counts falls and rises of at least 2 cm while the basket is over the pot, clear of
the rim), and the lid's offset and tilt on the rim.

Offline checks (no GPU): `python task.py` runs 19 scoring cases on hand-made states (including slow, shallow, two-stroke and
off-pot shakes) and replays the demo's plan through both arms' kinematics.

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Take the steeped infuser out of the teapot: lid off, basket out by its handle bar, shaken up and down three times over the pot to shed the drips, set on the saucer, then the lid back on the pot." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/lift-out-tea-infuser-demo
```

The right arm pinches the lid's knob (hand tilted 30 degrees forward), lifts the lid clear of the rim and the spout, lowers
it on the table to the right and lets go. The left arm goes down the mouth with its hand tilted 60 degrees forward,
closing along y, pinches the handle bar, lifts the basket clear (22 cm), shakes it three times over the pot (2.5 cm down and
2.5 cm up, a stroke every 0.7 s), carries it to the saucer and sets it down. The right arm then picks the lid up again and
lowers it onto the pot. The run scores 1.0 at 51.4 s (1,028 steps); the run limit is 2,000 steps.

Wrong strategy, scored 0: shoving the whole pot sideways towards the saucer with the right hand (`scripts/naive_tea.py`;
the pot moves 17.8 cm). Taking the basket out and leaving it anywhere else scores at most 0.25.

## Testing in real life

Materials, all ordinary items: a 1 L borosilicate glass teapot with a lift-out stainless mesh infuser basket that has a
handle bar across its top (choose one whose bar sits 2 to 6 mm below the rim and whose basket clears the mouth by 3 to 5 mm a
side; the sizes above can also be made from the asset files), a clear glass saucer, 8 g of dry black tea per trial, a
waterproof drip mat, a marked ring on the table, a kitchen scale (0.1 g), folded paper towel, and a phone filming at 60 frames
per second from the side (a 2 cm marker in view) and from above.

Each trial: reset as below, start the video, and let the robot work to the run limit. Score it from the video and the marks:
the ring shows whether the pot shifted, the ruler gives the basket's and lid's positions, the scale weighs any leaves outside
the basket, and the side video shows the strokes (count the falls and rises of at least 2 cm against the marker, and the time
from the end of the first to the end of the third).

Also report, without scoring them, the water on the saucer after the carry (the weight gain of a folded paper towel laid on
it) and the same trial with a still 3-second hold in place of the shake as a control; the difference shows what the shake sheds.
The 2 cm, three-stroke and 2-second thresholds can be adjusted from these readings without changing the task.

Safety and the rig: water at 40 C or cooler for trials on the robot; the pads touch only the dry bar and knob, and the drip mat
and wiping keep water off the table and the fingertips. The loads are small (lid 0.08 kg, basket with leaves about 0.2 kg),
well inside the fingers' roughly 40 N.

## Scoring

The score at the end of the episode counts, after both grippers are clear. The same rule applies in simulation and in real
trials.

| Outcome | Score |
|---|---|
| The pot shifted more than 4 cm (foot leaves the marked ring) or tipped more than 15 degrees at any time | 0 |
| Basket out of the pot and resting clear of its mouth (axis more than 7.5 cm from the pot's) | 0.25 |
| Basket resting on the saucer (axis within 3 cm of the saucer's centre, tilted under 15 degrees) | 0.5 |
| Also leaves outside the basket under 2 g (at least 90 % still in it), and the basket shaken over the pot: three down-up strokes of at least 2 cm each, the third ending within 2 s of the first, all with its underside at least 1 cm over the pot's rim, its axis within 4 cm of the pot's and tilted under 25 degrees | 0.75 |
| Also the lid back on the pot (centre within 6 mm of the pot's axis, resting on the rim within 4 mm of the seated height, tilted under 5 degrees) | 1.0 |

## Reset

Put the pot back on its marked ring with the lid on and 500 g of room-temperature water in it. Tip the old leaves out of the
basket, put in 8 g of fresh dry black tea and steep the basket in the pot for 3 minutes (water at 40 C or cooler for trials on
the robot); leave the basket standing in the pot with its bar running forward-back and put the lid on. Wipe the saucer, the drip
mat and the robot's fingertips dry and lay a fresh folded paper towel on the saucer. A new trial can start as soon as the pot is
back on its ring (about 5 minutes with the steep).

## Fidelity

Teapot, lid, basket and saucer are rigid bodies; their colliders are boxes and convex hulls rather than the models' own
meshes. The basket's mesh walls are solid in the simulation, so leaves cannot pass them. The damp leaves are 200 rigid
spheres (radius 2.5-3.5 mm, 23 g in all) with no stickiness. Water is not simulated: the pot's 500 g is added to its mass,
and dripping is represented by a rule (three quick strokes of the basket over the pot), which in a real trial is what sheds the
drips. The lid is a flat-bottomed hull that rests on the rim wherever it is set; the score asks for it within 6 mm of the pot's
axis.
