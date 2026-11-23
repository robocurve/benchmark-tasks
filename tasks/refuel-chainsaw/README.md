# Top up a chainsaw's fuel tank from a premix bottle without spilling

A vintage-style chainsaw lies on its flat base on the table, its fuel tank open. A 1 L premix bottle stands upright
beside it. The robot must pour the fuel from the bottle into the saw's tank without spilling it.

Topping up a chainsaw from a premix bottle is a routine job for anyone who cuts firewood or clears trees: the filler opening is small, the bottle must be turned past horizontal while it stays aimed, and fuel spilled on the housing or the ground is a fire and pollution hazard. The difficulty is physical: the 4 cm opening is only a little wider than the 3 cm spout, and the bottle has to be rolled past 120 degrees with its spout over it.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `scenekit.py`, `soilkit.py`, `tiltkit.py`, `capkit.py`, `usdkit.py` | Helpers: USD bodies for task files, the bead fuel, gripping at any angle and joint-space moves, quaternion maths |
| `assets/chainsaw.usdc` | The chainsaw: one kinematic body, USDCraft-10K ds_R951_08 meshes with our own colliders and tank (see below) |
| `assets/can.usda` | The premix bottle: one rigid body, 120 g, hollow shell (4 mm walls) with a 60 mm pour spout |
| `assets/ATTRIBUTION.txt` | Credit for the chainsaw meshes |

The chainsaw's meshes come from the USDCraft-10K dataset ("Vintage 1970s chainsaw", `ds_R951_08`,
https://huggingface.co/datasets/nvidia/USDCraft-10K, Apache-2.0). Changes: its joints, articulation and original collision meshes
are removed (the saw is one fixed body); its fuel cap and neck meshes are removed and replaced by our own opening; box colliders
replace the housing's solid mesh, which is hollowed into a tank. The bottle, the opening's collar and the beads are ours.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the table top. (1) Chainsaw, fixed (clamped to the table): a vintage-style 1970s chainsaw with a red painted cast-aluminium housing, a 50 cm steel bar and a top carry handle, 90 x 28 x 34 cm, lying on its flat base with the bar pointing to the robot's right (-y) along x = 45.0; its housing spans x 30.8 to 59.2, y -49.4 to 41.0. Its fuel tank is open: a round filler opening 4.0 cm across, with a dull-metal collar, centred at (40.0, 20.0) in the housing's top, 20.0 cm above the table, over a tank of about 0.9 L; the fuel cap is not part of the task. (2) Premix bottle: a 1 L molded-plastic fuel-mixing bottle, orange, 8.5 cm along x by 7.0 cm along y and 20.0 cm tall, about 120 g empty, standing upright with the centre of its base at (35.0, 55.0); its pour spout, 3.0 cm across, 2.4 cm bore and 6.0 cm tall, is on the top at (37.0, 56.9), 1.6 cm in from its +y wall. It holds about 0.1 L of fuel (1,170 rigid beads, 70 g, in simulation; see fidelity), a layer 4.5 cm deep. Both arms start at the ready pose; the right arm has nothing to do.

Sources: tank volumes of saws of this class are 0.49 L (McCulloch 610), 0.6 L (Husqvarna 55), 1.18 L (McCulloch Super 250) and 1.24 L
(Homelite 650) (fireandsaw.com model pages; Husqvarna 55 operator's manual, p. 37), so a 0.9 L tank is typical. Chainsaw fuel caps are
about 40-46 mm across (retailer listings for Stihl and Husqvarna replacement caps), so a 4 cm filler opening is typical. 1 L fuel-mixing bottles measure about
21.5 x 12 x 8 cm to 21 x 10 x 9 cm (retailer listings for the Sealey JMIX01 and Proplus bottles).

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Top up the chainsaw's fuel tank from the premix bottle: pour the fuel into the tank without spilling." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/refuel-chainsaw-dream
```

`task_score` prints the trial's score. A trial takes about 12 minutes on one L40S-class GPU, scene start-up and filming included (the
scripted demo itself covers 33 s of simulated time). Seed 0; the scene has no random placement (`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules on 12 hand-built states and prints the bead count and the opening's position.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Top up the chainsaw's fuel tank from the premix bottle: pour the fuel into the tank without spilling." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/refuel-chainsaw-demo
```

The left arm goes round behind the bottle with its hand pointing forward, closes its fingers across the bottle's 7 cm width at
mid-height (its centre of mass), lifts it, rolls it about the hand's axis to 60 degrees while it carries it over the saw, lowers the spout
to the opening, then rolls it on to 123 degrees so the fuel runs into the tank, holds it there for 14 s, and brings it back.

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`).

Read at the end of the run (real trial: the video plus a graduated cylinder and a scale). Fuel in the tank means below the filler opening, inside the tank. Fuel outside means neither in the bottle nor in the tank (on the saw, the table or the floor). 0 if more than 10% of the fuel (by mass) is outside, at the end of the run. Otherwise 0.25 with at least 25% of the fuel in the tank, 0.5 with at least 50%, 0.75 with at least 75%, and 1.0 with at least 80%; less than 25% in the tank scores 0.

In simulation, a bead counts as in the tank when its centre is inside the tank's cavity or the opening's column above it (in the saw's
frame), and as in the bottle when it is inside the bottle's shell or its spout bore (in the bottle's frame); every other bead is outside.

| Outcome | Score |
|---|---|
| More than 10% of the fuel outside the bottle and the tank | 0 |
| Less than 25% in the tank | 0 |
| At least 25% in the tank | 0.25 |
| At least 50% in the tank | 0.5 |
| At least 75% in the tank | 0.75 |
| At least 80% in the tank | 1.0 |

## Reset

Real trial: use water with a drop of food colouring, no fuel. Before each trial empty the saw's tank into a graduated cylinder, refill the bottle with
100 mL, stand it at its mark, and wipe the saw's housing and the table. Keep the chain's bar guard on and the saw unpowered. Read the tank's volume with the graduated
cylinder and the bottle's remainder with a scale; the rest is outside.

## Real trials

The heaviest piece is the saw, about 5.6 kg, standing on its base; the bottle with its fuel is about 0.2 kg. The scripted demo's grip is within the arms'
strength. Nothing is sharp (bar guard on) or powered, and only water is used. Wipe the pads, the table and the saw afterwards.
