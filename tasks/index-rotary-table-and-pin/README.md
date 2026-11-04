# Index an off-balance rotary table 90 degrees and lock it with a pin

A small rotary table sits on a base tilted 7 degrees down to the left. Its plate turns on a central post, carrying a heavy
off-centre fixture that swings it back to home unless it is held, and a peg to turn it by. The robot must turn the plate 90
degrees clockwise by the peg and lock it there, by dropping a steel pin through the plate's hole into the base hole at the
white mark.

Machinists index rotary tables and lock them with a pin before cutting (O*NET 51-4041.00). The task needs both arms for a
physical reason: the plate swings back the moment it is let go, so one arm holds it on station while the other drops the
pin. It tests rotating a part by pushing it through an arc, a sustained hold against a load, and a precise insertion into a
part that moves if it is nudged.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `success()`, `score()` and the two-arm `demo()` |
| `rigidparts.py` | Rigid parts built from several boxes, cylinders and spheres, tilted onto the slope (the base, plate and pin) |

yam_arena's objects are single primitives turned only about the vertical, so `rigidparts.py` builds these parts. It wraps
`yam_arena.scene.task_object` and `object_states` when they load, without modifying yam_arena, so the task runs with the
standard `yam_arena` embodiment, `yam_scripted` policy and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the rotary table 90 degrees clockwise by its peg and lock it there: drop the pin through the plate's hole into the base hole at the white mark." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/rotary-dream
```

The scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Turn the rotary table 90 degrees clockwise by its peg and lock it there: drop the pin through the plate's hole into the base hole at the white mark." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/rotary-demo
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode.

## Scripted demo

1. The right arm puts its closed fingers behind the peg and pushes it round in steps to 90 degrees, then holds it there.
2. The left arm takes the pin, waits to one side while the fixture swings through the space over the hole, moves over the
   hole, pauses for the pin to settle, and drops it through.
3. The right arm backs away. The plate leans on the pin about 6 degrees short of 90 and stays there.

The demo reaches 1.0 (success) in about 280 steps (14 s) with `-E object_jitter=0`, on NVIDIA L40S, RTX A6000 and RTX A5000
GPUs. Dream-1 (`dreamlabs/dream-1-beta`, 2,400 steps) scored 0.0. The right arm alone scored 0, the plate swinging back when it
let go; letting go 2 s before the pin arrived also scored 0.

The pin sits a little off where the fingers hold it, and the plate stops short of where the hand goes, the same way every
run with `-E object_jitter=0`; the demo allows for both (`PIN_GRIP_SHIFT` and `PUSH_LAG` in `task.py`).

## Scoring

Evaluated every control step; the score at the end of the episode counts. The plate's angle is how far it has turned
clockwise from home about the post. The pin is in a hole when it is within 12 degrees of upright, its axis within 5 mm of the
hole's centre at the base's top, and its foot below the base's top.

- 1.0 (success, held for 1 s): the pin in the 90-degree hole, the plate within 8 degrees of 90, the plate still and the pin
  not moving. A pin spinning in its hole doesn't count against it.
- 0.5: the plate ends within 8 degrees of 90 without the pin in that hole.
- 0.25: the pin in the 45-degree hole with the plate within 8 degrees of 45.
- Otherwise 0.

In a real trial: a protractor printed on the base gives the angle, and a photo from above shows which hole holds the pin.

## Runtime and seeds

A Dream-1 trial of 2,400 steps took about 10 minutes on one RTX-class GPU, mostly waiting on the model, plus about 1 minute
for Isaac Sim to start (longer on a fresh instance). The scripted demo takes under a minute.
The layout is fixed (`-E object_jitter=0`); there are no seeds to choose. The demo is open-loop and tuned to this layout,
so it may not reach 1.0 with object jitter.

## Fidelity

All parts are rigid. The plate rides on three low-friction balls around a plain post, and its off-centre weight comes from
one piece of uniform density, in place of a real table's bearing and a separate heavy fixture. The holes are square where a
real index plate's are round, and the wide plate hole acts as a lead-in, leaving a pinned plate about 6 degrees off 90.
Nothing is deformable.

## Real-world build

A 25 x 25 cm steel base on four legs, tilted 7 degrees, with a 16 mm post, three ball transfers and two 16 mm holes; a 20 cm
HDPE plate of 1 cm with a 10 x 6 x 7 cm block and a 2 x 2 x 4 cm peg fixed to it, 0.8 kg in all; a hollow steel pin in a drilled
block; small white and yellow marks on the table.
