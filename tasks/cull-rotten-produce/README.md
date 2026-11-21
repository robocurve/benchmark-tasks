# Take the rotten vegetables out of a packed produce container and put them in the waste bin

A produce container stands on the table with two red bell peppers, a zucchini and two tomatoes packed in it, and a
waste bin stands beside it. The two tomatoes are rotten (each has a standardized dark patch beside the stem); the peppers and
the zucchini are fresh. The robot must take the rotten vegetables out of the container and put them in the waste bin,
leaving the fresh produce where it is. The instruction does not say which pieces are rotten.

Clearing out the produce drawer or a crisper box is a routine household chore: a spoiled piece left among fresh ones
spreads mold to its neighbours and spoils them too, and USDA's Food Safety and Inspection Service advises discarding moldy
soft produce (fsis.usda.gov, "Molds on Food"). The instruction already asks for the spoiled pieces to be removed. The hard
part is physical. Each tomato is 8.6-8.8 cm across and the gripper opens 9.4 cm, so the fingers have about 3 mm either side
of it; and the tomato stands 1 cm from the peppers beside it along the row, which keeps the fingers out of the row, so they
have to go down in front of and behind the tomato and close across the row.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: scene, `success()`, `score()`, `self_test()` and the scripted `demo()` |
| `scenekit.py` | USD bodies and fixed shapes for yam_arena task files (hooks yam_arena without changing it) |
| `tiltkit.py` | Lets the scripted demo grip at any angle and make joint-space moves (used for the tilted carry) |
| `assets/rotten_tomato_a.usdc` (675 KB) | The tomato: one rigid body, 220 g, 88.4 x 86.0 x 58.7 mm, convex-hull collider |
| `assets/red_pepper.usdc` (789 KB) | The red bell pepper: one rigid body, 170 g, 84.9 x 101.6 x 101.8 mm, convex-hull collider on a flat base |
| `assets/zucchini.usdc` (1,014 KB) | The zucchini: one rigid body, 297 g, 197.2 x 53.6 x 52.0 mm, convex-hull collider |
| `assets/ATTRIBUTION.txt` | Credit for the three produce models |

The three produce models come from the marso SimReady assets `rotten_tomato`, `red_bell_pepper` and `zucchini` by
M-XR Limited ("marso by M-XR", https://huggingface.co/datasets/M-XR/marso-simready, CC BY 4.0). Changes: the albedo
texture baked into vertex colours (the tomato then recoloured: plain red with one flat-edged dark disc 4.0 cm across), the pepper's mesh thinned, the origin moved to the centre of the bounds, the collider
replaced by a convex hull. The containers are fixed boxes in `task.py`.

## Setup

Positions in cm from the midpoint between the two arm bases (x forward, y to the robot's left); heights above the table
top. All pieces rest on the container's floor.

| Object | Where | What |
|---|---|---|
| Produce container | centred (36.0, -3.0), long side along y, taped to the table | Rubbermaid FreshWorks 3-gallon produce saver, clear plastic, 45.7 x 30.5 cm and 16.0 cm tall (18 x 12 x 6 5/16 in), lid and insert removed, walls about 3 mm |
| Waste bin | centred (40.0, -49.0), taped to the table | OXO Good Grips Easy-Clean 1.75 gallon compost bin, charcoal plastic, 25.8 cm along x by 21.1 cm along y, 30.5 cm tall (10.15 x 8.3 x 12 in), lid open |
| Rotten tomato A | (29.5, -20.1), stem end up, its patch towards the front of the table (the robots) | Firm large red tomato, 8.8 x 8.6 x 5.9 cm, 220 g, a dark patch 4.0 cm across on its upper shoulder beside the stem (prepared as under Reset) |
| Red bell pepper A | (29.5, -9.6), stem up | Red bell pepper (fresh produce), 8.5 x 10.2 x 10.2 cm, 170 g |
| Rotten tomato B | (29.5, 0.9), stem end up, its patch towards the front of the table | As tomato A |
| Red bell pepper B | (29.5, 11.4), stem up | As pepper A |
| Zucchini | (42.5, -14.7), its length along y | Zucchini (fresh produce), dark green, 19.7 x 5.4 x 5.2 cm, 297 g |

Each tomato is about 1 cm clear of the pieces beside it along the row. The left arm has nothing to do.

Sources: the container and the bin, retailer listings (Rubbermaid FreshWorks 3 gal 18 x 12 x 6 5/16 in, WebstaurantStore
item 6902052935; OXO Good Grips Easy-Clean 1.75 gal compost bin 8.3 x 10.15 x 12 in, Walmart and Target listings); the
produce, the sizes and estimated masses of the marso assets (their `USAGE.txt`); the food-waste figure, USDA.

## Run a trial

From this folder, in a shell with the yam_arena environment active (`source <ARENA_DIR>/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the rotten vegetables out of the produce container and put them in the waste bin." \
  --scorer task_score --max-steps 1200 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/cull-rotten-produce-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. A trial takes
about 9 minutes on one L40S-class GPU, scene start-up and filming included (the scripted demo itself covers 24 s of
simulated time). Seed 0; the scene has no random placement (`-E object_jitter=0`).

Offline checks (no GPU): `python task.py` runs the scoring rules on 12 hand-built states, checks the packing (every piece
inside the container's walls, no two overlapping, at least 9 mm beside each tomato) and replays the demo's plan through
the arms' kinematics.

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Take the rotten vegetables out of the produce container and put them in the waste bin." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/cull-rotten-produce-demo
```

For each tomato the right arm comes down with its fingers open, in front of and behind the tomato (the fingers close along
x, across the row), closes on it (finger target 36 mm either side of centre, a gap of 72 mm), lifts it straight up to
clear the peppers, tilts the hand 45 degrees forward in a joint-space move, carries the tomato over the bin, opens, and
returns to its start pose. The trial reaches 1.0 at about 24 s (480 steps), once both tomatoes are in the bin and the
gripper has let go. Three runs gave the same result, with the peppers and the zucchini never moved.

A wrong strategy, taking a fresh pepper out first and then both tomatoes, scores 0: the pepper ends in the waste bin, so
the fresh produce is not kept (the run was filmed to the end of the plan, 40.8 s).

## Scoring

The score at the end of the episode counts (`SCORE_OVER = "end"`). Tools for a real trial: a phone camera, a ruler and
the trial video. Before each trial, tape marks outline each good piece's starting place on the container floor.

- **Fresh produce kept:** both peppers and the zucchini lie inside the produce container's walls, each with its centre
  within 2 cm of its starting centre (ruler against the marks). In simulation: each piece's centre is within the
  container's inner walls and below its rim, and within 20 mm of its starting centre seen from above.
- **In the bin:** a tomato rests inside the bin's walls, below its rim. In simulation: its centre is within the bin's
  inner walls and below its rim.
- **Released:** both grippers are open and at least 7 cm from both tomatoes, and both tomatoes are still, held for 1 s
  in the video. In simulation: no fingertip (88 mm below the finger links' origins, midway between them) within 70 mm of
  a tomato's centre, and each tomato's speed under 0.05 m/s.

| Outcome | Score |
|---|---|
| A pepper or the zucchini is outside the container or more than 2 cm from its start, or no tomato is in the bin | 0 |
| One tomato in the bin, the fresh produce kept | 0.5 |
| Both tomatoes in the bin, but a gripper still within 7 cm of a tomato or a tomato still moving | 0.75 |
| Both tomatoes in the bin, released, held for 1 s | 1.0 |

## Reset

The standardized dark patch is what marks a tomato as rotten, so every trial's tomatoes look alike. Preparing a pair takes under 10 minutes, including drying: choose two firm large tomatoes 8.5-9 cm across and 5.5-6.5 cm tall, 200-240 g on a kitchen scale, with no cracks or soft spots, and spares. Cut a 4.0 cm round hole in a piece of card. Hold the card against the upper shoulder of each tomato, with the hole's centre 2.5 cm from the stem scar, on the side that will face the robots, and dab black edible food-colouring gel through the hole with a foam dauber, twice, to an even opaque dark patch 4.0 cm across; lift the card straight off and let the patch dry 5 minutes until it is not tacky. Each trial use a pair with intact, dry patches, stem end up with the patch towards the robots, at the listed centres; stand the peppers and lay the zucchini back at theirs, checking the 1 cm gaps beside each tomato with a ruler and re-taping the outline marks. Handle the tomatoes by their sides so the patch is not rubbed, and repaint a patch or swap in a spare whenever one is smeared, chipped or the skin is split. Wipe the gripper pads, table and containers afterwards and empty the bin.

## Real trials

The heaviest piece is 297 g. The scripted demo's squeeze is within the arms' strength (about 40 N per finger). Nothing is sharp, hot
or powered; the containers are taped down so a push against a wall cannot tip them. Wipe the gripper pads, table and
containers afterwards and empty the bin.
