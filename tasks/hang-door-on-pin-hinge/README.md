# Hang a door on a pin hinge

A fixed steel hinge post stands between the arms: a leaf with three square knuckles (18 mm holes, 12 mm tall, 18 mm
apart on a vertical axis). In front of the right arm, a small solid PVC door (12 x 8 cm, 1.2 cm thick, 0.2 kg) stands on edge in a
low holder, two knuckles on its hinge edge. In front of the left arm, a hollow steel hinge pin (10 mm shaft with a rounded nose,
8.5 cm long, 22 mm head) stands in a socket. The robot must hold the door with its knuckles between the post's, lined
up on the axis, and drop the pin down through all five knuckles so the door hangs on it.

Cabinet makers, installers and maintenance workers hang doors, gates and access panels on lift-off pin hinges
(O*NET 51-7011.00, 49-9071.00). The task needs both arms for a physical reason: nothing supports the door in line with
the post until the pin is in, so one arm must hold it there while the other places the pin. It tests holding a loose
part at a precise pose (the knuckles fit the gaps with 6 mm to spare vertically, and the pin has 4 mm of play in the
holes) while the other arm works on the same axis.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |
| `rigidparts.py` | Rigid parts built from several boxes, cylinders and spheres (the post, the door with its knuckles, the pin) |

`rigidparts.py` wraps `yam_arena.scene.task_object` and `object_states` when they load, without modifying yam_arena,
so the task runs with the standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the door on the hinge post: hold its knuckles between the post's knuckles and drop the pin down through all five." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/hinge-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hang the door on the hinge post: hold its knuckles between the post's knuckles and drop the pin down through all five." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/hinge-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

1. The right arm grips the door 4 cm down its height and 8.5 cm out from the hinge edge, lifts it out of its holder,
   carries it beside the post, lowers it to knuckle height and slides its knuckles in between the post's, then holds.
2. Meanwhile the left arm takes the pin out of its socket by the shaft and waits above the hinge axis, high enough that
   the door passes underneath.
3. The left arm lowers the pin until its nose is 3 mm above the top knuckle and opens: the pin drops through all five.
4. The right arm lets go; the door settles onto the knuckle below and hangs on the pin.

The demo reaches 1.0 (success) in about 250 steps (12 s): success is reached once the pin is through all five
knuckles, just before the right arm lets go. It scored 1.0 with `-E object_jitter=0` on NVIDIA L40S and RTX A5000 GPUs. Dream-1 (`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0.

Closing the fingers shifts each part by a few millimetres, the same way every run with `-E object_jitter=0`; the demo
carries each part so that the part, not the fingers, lands on target (`DOOR_GRIP_SHIFT` and `PIN_GRIP_SHIFT` in
`task.py`, measured in a traced run).

## Scoring

Evaluated every control step; the score at the end of the episode counts.

- 1.0 (success, held for 1 s): the pin, within 10 degrees of upright, passes through all three of the post's knuckles
  and both of the door's: at each knuckle's height, its axis is inside the knuckle's hole (within 6 mm of the centre
  along each side; a pin outside a knuckle is at least 14 mm off). The door's lowest point is at least 4 mm above the
  table; door and pin are still.
- If the pin passes through at least one of the post's knuckles: 0.4 for each of the door's two knuckles it passes
  through (0.8 at most without success).
- Otherwise 0. Dropping the pin into the post's knuckles alone scores nothing, so the door has to be held in place;
  pinning the door's own knuckles away from the post scores nothing either.

In a real trial: the door hangs on the post without touching the table, and the pin's head rests on the top knuckle.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 10 minutes on one RTX-class GPU, mostly waiting on the model (about 1.0 s per reply), plus about 1 minute for Isaac Sim to start (longer on a fresh instance). The scripted demo takes under a minute.
The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The demo is open-loop and tuned to this layout, so it may not reach 1.0 with object jitter.

## Fidelity

All parts are rigid, built from boxes, cylinders and a sphere (the pin's nose); the post and the holders are fixed. The
knuckles are square rings rather than round barrels, which leaves the pin the same 4 mm of play each way as a loose
real hinge; hung, the door sags about 3 degrees on the pin and clears the table by about 7 mm. The door and pin use the
station's grippy contact material. Nothing is deformable.

## Real-world build

A steel lift-off hinge leaf with three knuckles, screwed upright to a block clamped to the table; a 12 x 8 cm, 12 mm
solid PVC door with the mating two-knuckle leaf; the hinge's own pin, or a 10 mm steel tube with a rounded end and a
collar. The holders are two blocks taped either side of the door's lower edge and a drilled block for the pin.
