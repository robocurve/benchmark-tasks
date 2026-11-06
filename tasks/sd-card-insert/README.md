# Insert the SD card into the reader, label up, until it clicks

Loading a memory card is part of setting up almost any device on an electronics bench: a camera, a single-board
computer, a 3D printer, a data logger. A full-size SD card is 2.1 mm thick and goes into its slot only one way round,
label up and contacts first, guided by its cut corner. It has to be pushed in until the push-push latch clicks,
then it stays in. Here the card lies on a bench shelf with its back end over the shelf's edge, the only place a
gripper can get a pad under it. The reader is loose and light, so one hand holds it while the other pushes the card
in.

| File | What it is |
|---|---|
| `sd_card_insert.py` | The `yam_arena` task file: scene, scoring (`success`, `score`), the latch's step hook and the scripted `demo()` |
| `parts.py` | `Compound`: one rigid body made of several box colliders (the card with its cut corner, the reader with its slot) |
| `latch.py` | The click: a prismatic joint between the card and the reader, switched on once the card is pushed in far enough |
| `bonds.py` | `Sliding`: a box on a sprung prismatic joint (the eject plate inside the slot) |
| `simready.py` | Loads the NVIDIA SimReady monitor and keyboard (pinned revision) |
| `wrist.py` | Demo waypoints with a tilted and rolled grip (a forward grip with one pad above the card and one below) |
| `looks.py` | Puts a detailed mesh on a box at spawn and hides the plain box, which still collides |
| `looks/` | The meshes (`card.usdc`, `reader.usdc`, `shelf.usdc`) and `make_looks.py`, the Blender script that models and exports them |
| `test_sd_card_insert.py` | Offline checks: the slot only takes the card one way round, every scoring outcome, the latch's memory, the demo's depths, arm spacing and reach (no GPU) |
| `task.json` | The task's details for the board |

## Assets

- **Monitor and keyboard:** NVIDIA SimReady `computer_monitor_office_a01` and `computer_keyboard_a01`, dataset
  `nvidia/simready-assets` at revision `9d4556bfe80868c1d27a7a7ba1c2774e63e8cedd` (CC-BY 4.0), downloaded on first use
  into `~/.cache/simready`. Both are fixed (kinematic) and only set the scene.
- **SD card, card reader and shelf:** modelled in Blender by `looks/make_looks.py` and drawn on the task file's boxes.
  The card's and the reader's meshes fit inside their boxes (a test checks it). The card is red so that nothing else
  on the table looks like it.

## The mechanism

- **Card:** 32 x 24 x 2.1 mm, 2 g, one rigid body of three box colliders: its front right corner (label up, contacts
  forward) is cut 5 mm at 45 degrees.
- **Reader:** a card reader and USB hub, 140 x 45 x 14 mm, 60 g, loose on the shelf, one rigid body of box colliders around its SD slot. The slot
  is 25 mm wide, 3 mm high and 24 mm deep, 5 mm above the reader's bottom. Its mouth flares 2 mm out on each side and
  1 mm up and down over its first 4 mm. A stop fills the slot's back right corner (the corner within 4 mm, measured
  along the slot plus across it). The card's cut corner clears it. Upside down, or contacts toward the robot, the
  card's square corner meets it and the card stops 21 mm in or less.
- **Eject plate:** a 2 mm plate across the slot (clear of the stop), 18 mm in, on a sprung prismatic joint
  (400 N/m, so 1.6 N at the click, overdamped), between 18 mm and the back wall.
- **Latch:** once the card is 22 mm or more in the right way round, a step hook switches on a prismatic joint between
  the card and the reader (`latch.py`). It holds the card centred and level in the slot and lets it go deeper, to the
  back wall, but not back out past 22 mm. While it is on, the card and the reader don't collide, so the joint and the
  slot's walls don't fight over the card. The plate pushes the card back against it when let go. A card let go before
  the click is pushed back out to 18 mm by the plate. Pushing a latched card a second time does not eject it.

## Run it

On the YAM Brev launchable, from this folder, with the Arena environment active
(`source /data/IsaacLab-Arena/yam.env`):

```bash
# Dream-1 (needs DREAM_API_KEY), 1,200 steps
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=sd_card_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the black card reader down with the left hand. With the right hand, pick up the red SD card and push it into the reader's SD slot, label up and contacts first, until it clicks." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/sd-card-insert-dream

# Scripted demo (both arms), 600 steps
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=sd_card_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the black card reader down with the left hand. With the right hand, pick up the red SD card and push it into the reader's SD slot, label up and contacts first, until it clicks." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/sd-card-insert-demo
```

- The run prints `task_score` at the end. With `-E object_jitter=0` the layout is fixed and no seed is needed. Keep it
  at 0: jitter would move the eject plate off its joint inside the reader.
- The layout keeps each arm on its own side: the card and the SD slot are on the right (y = -0.18 and -0.085), and the
  left arm holds the reader's left end at y = 0.015. The grippers' fingertips stay 9.5 cm or more apart.
- The card lies with 13 mm of its back end over the shelf's edge: that is where the lower pad of a gripper closing
  vertically gets under it. The shelf stands 13 cm high so that a forward grip, fingers level, can reach the slot.
- The demo: the left arm comes down at 45 degrees, fingers closed, and presses the reader's left end onto the shelf.
  The right arm, fingers pointing forward and rolled a quarter turn (one pad above the card, one below), slides the
  lower pad under the card's overhanging back end, pinches its last 8 mm, lifts it 5.5 mm to the slot's height,
  draws it back off the shelf, carries it 9.5 cm left to the slot, slides it 17 mm in and lets go. It backs off,
  closes its fingers and pushes the card's back edge with the fingertips to the back wall (24 mm): the latch clicks
  at 22 mm. It backs off, the eject plate pushes the card back onto the latch, at 21.7 mm. It reaches 1.0 and ends on
  success at step 273 (13.7 s at 20 Hz). A trial takes about 85 s on an L40S, including startup.
- The same demo with the left arm staying at its ready pose instead of holding the reader (`HOLD = False`) scores
  0.25: the push slides the loose reader 5.6 mm back along the shelf, the card goes only 9 mm in, droops when let go,
  and the fingertips' push then passes over its back edge. It never clicks.
- Videos: `inspect-robots video ~/yam-eval/sd-card-insert-demo/<log>.json` writes one MP4 per camera next to the
  frames.
- Offline checks: `python -m pytest -q test_sd_card_insert.py`.

## Scoring

All positions are centres, in metres, in the station frame (x forward, y to the robot's left, from the midpoint of
the arm bases), and heights are above the table top. The card's depth is measured in the reader's own frame (the
reader may have moved): how far the card's deepest corner is past the slot's mouth, along the slot.

- **In the slot:** the middle of the card's leading edge is within the slot's 25 mm width and within 2 mm of its
  centre plane, and the card's depth is 3 mm or more.
- **Right way round:** label up and contacts first: the card's long axis within 15 degrees of the slot's, its label's
  normal within 15 degrees of the reader's up.
- **Latched:** the latch has clicked (the card was pushed 22 mm or more in, the right way round) and the card is still
  21.5 mm or more in.
- **0** if the card has fallen off the table (centre more than 2 cm below the top), the reader has fallen off the
  shelf (centre 1 cm or more below its start) or the state is not finite.
- **1.0 (success, held 1 s):** latched, the right way round.
- **0.5:** in the slot the right way round, 15 mm or more in, not latched.
- **0.25:** in the slot the right way round, 3 mm or more in.
- **0** otherwise, including a card in the slot the wrong way round (upside down or reversed) at any depth.

In a real trial, a ruler against the card's back end gives how far it is in (32 mm minus what sticks out), the click
is heard and the card stays put when let go, and a photo shows which way round it went in.
