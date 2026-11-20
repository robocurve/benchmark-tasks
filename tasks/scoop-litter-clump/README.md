# Scoop a urine clump out of a cat's litter box, sift the loose litter back and tip the clump into the bin

A cat's litter pan stands on the table, filled 2 in deep with tofu clumping litter. One urine clump sits
half-buried near its far side, and a slotted plastic litter scoop rests in the pan with its handle over the rim. A
small round wastebasket stands to the left. The robot must scoop the clump out with the scoop, let the loose litter
run back into the pan, and tip the clump into the bin, without touching the clump.

Scooping the litter box is a routine chore in the 53 million US households with a cat (APPA 2026 Cat Report). The
difficulty is physical. The clump sits in loose litter that gives way and flows: the scoop's lip has to go in beside
the clump and under it, deeper than the clump reaches, and the scoop then has to come up with the clump on its
slotted floor while the litter on and around it runs off. The scoop is light and is held by its thin handle, the
litter pushes back on it, and the clump has to stay on the floor until the scoop is turned over the bin.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, the clump-touch and clump-lifted watch, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `soilkit.py` | The litter: tens of thousands of small rigid spheres authored straight into USD, read back through one PhysX view every control step, put back at every reset |
| `partkit.py` | Rigid parts built from boxes and cylinders (the scoop, its handle, the clump), with per-part materials and welds |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `tiltkit.py` | Any gripper orientation and joint-space moves for the scripted demo |
| `assets/litter_start.npz` (1.7 MB) | The litter's starting layout: 53,920 grain centres and radii, settled round the clump's place and levelled 2 in deep |

`litter_start.npz` comes from `work/scoop-litter-clump/prepare_litter.sh` (a loose lattice of grains settles in the
pan with a fixed block holding the clump's place, grains above the level are struck off, the rest settles again). No
third-party assets.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the
table top.

| Object | Where | What |
|---|---|---|
| Litter pan | centred (30.0, 4.0), long side across the table, on a rubber mat; pan and mat taped to the table | Van Ness small cat pan, 14.4 x 10.75 x 4 in, blue plastic |
| Litter | in the pan, levelled 2 in (5.1 cm) deep | Happy & Polly Clumping Natural Tofu Cat Litter (pea fibre and corn starch, 2% baking soda): strips 2 mm across and 1-2 cm long, about 1.7 kg |
| Clump | centred (33.5, 10.0), its top 0.6 cm above the litter | One fresh urine clump of that litter, about 4.4 cm across, 3.4 cm tall, about 35 g |
| Scoop | basket on the litter centred (30.0, -3.0), pointing to the robot's left, handle resting on the pan's right-hand rim | Van Ness plastic litter scoop, regular: 11 x 4.88 x 2.25 in, five 1/2 in slots about 4 in long in its floor |
| Bin | centred (30.0, 32.5), taped to the table | mDesign 1.25 gal round plastic wastebasket, 7.74 in across the top, 9.12 in tall, empty |

Sources: pan, Van Ness small cat pan (PetValu, Mudbay listings); scoop, Van Ness litter scoop (Aquarium Depot listing:
11.00 x 4.88 x 2.25 in; Chewy questions and answers: 1/2 in slots about 4 in long, 5 in basket); bin, mDesign 1.25 gal
round wastebasket (Target listing); litter, tofu litter strips 2 mm across (Cature, Michu, Happy & Polly listings),
1-2 cm long (Fortress CL102), Happy & Polly 2 mm (Walmart listing), bulk density about 0.42 g/cm3 (Weelitter, 6 L for 2.5 kg), 2-3 in deep (Chewy); clump
size, "about 1 to 2 inches across" (Litter-Robot); one urination about 23 ml (18-28 ml/kg/day for a 4 kg cat, three
to four times a day); PP friction 0.26 (Plastics International, polypropylene homopolymer); cat households, APPA
2026 Cat Report.

## Reset between trials

Each trial uses a fresh clump: pour 23 ml of water onto the levelled litter at the clump's place and let it set 15 minutes (tofu litter clumps within seconds but stays sticky for about 15 minutes), check that it lifts out whole and weigh it (about 35 g), then set it back in its place. Between trials tip the bin out, take out any clump, return loose litter to the pan, level it 2 in deep and lay the scoop back.

Sources: tofu litter clumps in 3 s (Michu, Tractor Supply listing; Happy & Polly's own site) and stays sticky for
about 15 minutes (cats.com, Tuft + Paw litter review).

## Scoring

`score()` at the end of the episode:

| Score | When |
|---|---|
| 0 | a finger touched the clump at any time, the pan was lifted, tipped or shifted, or the clump was never scooped |
| 1.0 | scooped, and the clump in the bin with at most 15 g of loose litter in the bin and at most 15 g of litter anywhere but the pan and the bin |
| 0.5 | scooped, and the clump in the bin with more litter in the bin or spilled |
| 0.25 | scooped, but the clump not in the bin |

Scooped: at some moment the clump rests on the scoop's floor with its bottom 1 cm or more above the litter's level.
In the simulation the pan is fixed (on a real trial it is taped down) and "on the scoop" means the clump's centre is
over the basket's floor, between its walls.

`success()` ends the trial once the score is 1.0 and the clump has come to rest (held 1 s). On a real trial: watch
the video for finger contact, for the pan moving and for the clump scooped clear of the litter, weigh the bin's contents less the clump on a kitchen scale, and sweep
up and weigh any litter on the table, the mat or the scoop.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Scoop the clump out of the litter box with the scoop, sift the loose litter back into the box, and tip the clump into the bin." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/scoop-litter-clump-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. A trial takes
about 25 minutes on one RTX 4080 (scene start-up included; the 53,920 grains run at about a sixth of real time).
Seed 0; the scene has no random placement (`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules on 16 hand-built states (two of them never scooped, as when the pan is tipped into the bin), checks the layout (the
scoop clear of the clump and its grip outside the rim) and that the scoop's lip stays clear of the pan's floor, and
replays the demo's plan through both arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Scoop the clump out of the litter box with the scoop, sift the loose litter back into the box, and tip the clump into the bin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/scoop-litter-clump-demo
```

The right arm takes the handle near its end from the side, the fingers closing on its top and underside, and lifts
the scoop off the rim; the left arm then takes the handle too, near the basket, so the scoop is held at two points
100 mm apart. Together they turn the scoop steep, push its lip into the litter along its floor just behind the clump
to below the clump's bottom, swing it flatter so the lip goes forward under the clump, and lift it straight out with
a jigging motion, levelling it as it rises so the litter runs off through the slots while the clump stays on the
floor. They tip it back, shake it over the pan, carry it over the bin, tip it nose-down and jog it: the clump drops
into the bin. The trial reaches 1.0 at about 67 s (1,350 steps), with no loose litter in the bin and none spilled.
