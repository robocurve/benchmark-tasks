# Free a kitchen drawer jammed by a potato masher

The arms open a kitchen drawer jammed by a potato masher. Put away with the drawer part open, the masher ended up at
the back, under the counter, on its side with its fin head standing on one end: 102 mm tall, against 85 mm under the
cabinet's top rail. Pulled, the drawer runs out about 21.5 cm and stops hard where the masher's forked neck catches
the back of the rail. The arms are asked only:

> **"Open the kitchen drawer all the way, leaving everything in it."**

Pulling harder only levers the masher up against the rail. The fix is to reach into the open part of the drawer, roll
the masher onto its fins by the grip (66 mm tall), then pull the drawer out. The jam is a loose utensil whose pose has
to be read from the scene.

`task.py` holds the scene, scorer and scripted demo; `yamkit.py`, `gripkit.py` and `yamgeom.py` are helpers. The
robot model and its collision hulls load at run time and are not redistributed.

## Requirements

- The yam_arena environment (Isaac Sim, the YAM two-arm embodiment, `inspect-robots`).

## Run and score a trial

The scorer (`task_score`) reads the drawer's slide position, the masher's pose and every arm link's collision hulls
each control step and scores the end of the trial. "Flat": the masher's highest point at most 83 mm above the
drawer's floor, 2 mm under the rail.

- **1.0:** the drawer out at least 295 mm (stop at 300 mm), the masher flat and wholly inside the box, every arm link
  at least 5 mm from the masher and the drawer's front and pull, all still (under 1 cm/s) for 1 s.
- **0.5:** the masher flat and the drawer out at least 280 mm, but not fully out, still held, or still moving.
- **0.25:** the masher flat, the drawer short of 280 mm.
- **0:** anything else. Latched at 0 for the rest of the trial: the grip more than 30 degrees off level with the
  drawer out 205 mm or more (a yank at the snag); the part of the head and fork that the rail catches higher than the
  rail's underside while under or in front of the rail, or any other point of the masher (the grip, the fork's front)
  higher than it while under the rail itself (forced through); the drawer past 280 mm with the masher not flat; the
  masher lifted more than 40 mm off the floor or taken out of the box.

Dream-1, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the kitchen drawer all the way, leaving everything in it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir masher-dream
```

Scripted demo (open loop, fixed waypoints, no simulator state read):

1. The left hand pinches the bar pull, draws the drawer out to the snag (about 215 mm), eases it back to 185 mm and
   lets go.
2. The right hand takes the masher's grip from above, rolls it 40 degrees toward the drawer's middle (the head tips
   over past about 33 degrees), lets go and slides its fingers off along the grip. The head falls onto its fins.
3. The left takes the pull again and draws the drawer out to its stop. Both arms go home.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the kitchen drawer all the way, leaving everything in it." \
  --scorer task_score --max-steps 1100 --store-frames --log-dir masher-demo
```

Video: pending.

## Failure modes

Demo variants (`MASHER_VARIANT=`) and offline tests:

| Way to game it | What happens | Score |
|---|---|---|
| Do nothing (`nothing`) | The start meets no tier | 0 |
| Pull to the snag and stop | Masher not flat | 0 |
| Yank to the stop (`yank`) | Levered past 30 degrees or forced under the rail | 0 (latched) |
| Lift the masher out (`lift`) | Lifted over 40 mm or out of the box | 0 (latched) |
| Open, then shut again (`close`) | End state: flat, drawer short of 280 mm | 0.25 |
| Keep holding the pull (`hold`) | Hands not clear | 0.5 |
| Leave it tilted 11 degrees or more | Not flat | 0 |

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). The scripted demo succeeds at step 997 (50 s); the trial ends
  once the 1 s hold is met.
- No randomness: the layout is fixed (`-E object_jitter=0`).

## Real-world build

A real OXO Nylon Potato Masher and a plywood mock of the cabinet section at the sizes in `task.json`, the drawer on
12 in full-extension slides.

## Sources and license

- OXO Good Grips Nylon Potato Masher, Target TCIN 80221526 (10.2 in long, 3.9 in wide, 0.3 lb, nylon):
  https://www.target.com/p/oxo-nylon-potato-masher/-/A-80221526. The head's depth (64 mm) and length, the fork and
  the grip's thickness are estimated from the listing's photos.
- Friction: Engineering ToolBox, https://www.engineeringtoolbox.com/friction-coefficients-d_778.html (wood on wood
  0.25-0.5; nylon on nylon 0.15-0.25, on steel 0.4; rubber on dry concrete 0.6-0.85, for the soft grip). The model
  uses wood 0.40 / 0.30, nylon 0.35 / 0.30, soft grip 0.8 / 0.6.
- Task statement: O*NET OnLine, Food Preparation Workers 35-2021.00, https://www.onetonline.org/link/summary/35-2021.00
- All code here was written for this task; no third-party meshes, images or data files are included.
