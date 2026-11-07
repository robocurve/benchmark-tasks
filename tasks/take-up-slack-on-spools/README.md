# Take up patch-cord slack on spools, then plug in

A blue 6 mm patch cord (84 cm) runs from a fixed device block across the whole table. It is far longer than the
direct path from the device to a nearby socket. The robot must take up the excess by winding the cord once around
two fixed slack spools (60 mm hubs, 12 cm apart), so that the cord lies against both hubs. It then pushes the plug
fully into the socket's port and lets go, keeping the whole cord inside a yellow tape rectangle. In racks and cable
managers, excess patch-cord length is commonly stored on slack spools, which keep the cord tidy and above its minimum
bend radius (TIA-568: 4x the cord diameter for patch cords, 24 mm here, below the 30 mm hub radius). Both arms are needed. Only the right arm reaches the plug at the start, and only the left arm reaches
round the spools and the port, so the plug must be handed over.

## Requirements

- yam-isaac-lab-arena with Isaac Lab-Arena 0.3.1 installed (the YAM bimanual station).
- `cablekit.py` (cable simulation) and `cabledemo.py` (scripted demo runner) are in this folder.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 2400 \
  --instruction "The blue patch cord is too long. Take up its extra length by winding it once around the two grey slack spools, so the cord lies against both spools, then push its plug fully into the port of the dark socket. Keep the whole cord inside the yellow tape rectangle, then let go."
```

## Scripted demo

```bash
python cabledemo.py task.py demo_out
```

The right arm picks up the plug on the far right and carries it up the right side of the right spool to a hand-over
spot. The slack trails behind it in a U that hooks round that spool. The left arm then carries the plug up the far
side of both spools, round the left spool and back along the near side, so the cord makes one full turn round the
pair. It sets the plug down in front of the port and re-pinches it near its cord end, with its wrist camera facing
the port. It squares the plug up, pushes it in (which draws the turn snug against both hubs) and lets go. The demo
reaches task_score 1.0 in about 790 steps.

Shortcut variants for testing (environment variable `SLACK_CHEAT`) all score exactly 0:
`nothing`; `direct` (plug pushed in, slack left loose); `pile` (slack bunched inside the rectangle, then plugged
in without winding); `noplug` (wound, but the plug is left lying in front of the port); `hold` (wound and plugged
in, but the gripper keeps holding the plug).

## Scoring

`score()` is evaluated at the end of the episode:

- It is 0 if either gripper's centre is within 3 cm of the cord or plug (still holding).
- It is 0 if under 95 % of the cord's length lies inside the tape's inner edges.
- It is 0 if the cord lies against under 150 deg of either hub's circumference. Lying against means the cord surface is
  within 15 mm of the hub surface and below the flange, judged in top view and counted as the union of angles round the hub axis.
  One snug turn round both spools covers about 180 deg of each hub. Loose slack, a pile or a route that only passes
  behind the spools covers much less.
- Otherwise the score is 1.0 if the plug tip is at least 15 mm inside the port, 0.5 if it is 5-15 mm inside, and 0
  if it is not in the port.

Success (score 1.0, cord at rest) ends the episode after it has held for 1 s. `progress()` reports these stages:
plug grasped, coverage of each spool, plug entered, plug seated, cord in rectangle and released.

## Runtime and seeds

One trial takes up to 2,400 steps (2 minutes of robot time at 20 Hz), and the demo takes about 790 steps. On one
GPU a trial takes about 5-10 minutes of wall-clock time. The layout is fixed with `object_jitter=0`, and the cable
is not jittered. `VARIATION` in task.py lists the reset offsets that a seeded runner may apply.
