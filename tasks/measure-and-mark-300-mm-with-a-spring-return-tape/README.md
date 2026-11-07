# Hold a tape measure pulled out along a board with one hand while the other marks the board at 300 mm

A pine board lies on the table. A tape measure's hook is caught over a nail at the board's right end, its case lying
on the board against it with the tape wound in. A marker pen stands in a holder beside the board. One hand pulls the
case out along the board, well past 300 mm, and holds it there; the other takes the pen and dabs a mark on the board
right beside the tape's 300 mm line (a red line across the tape), then lifts the pen. The task is one task file,
[`measure_and_mark_300_mm_with_a_spring_return_tape.py`](measure_and_mark_300_mm_with_a_spring_return_tape.py), for the
`yam_arena` embodiment (scene, scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from
the same folder.

What it tests: pulling against a spring and holding a position with one hand while the other places a tool to within
4 mm at a reading on the tape.

Why two hands: the tape's spring winds it straight back in as soon as the case is let go, and a mark counts only if the
tape is out past it when the mark is made. One hand holds the tape out while the other marks. With one hand (the right
hand taking the pen and marking as in the demo, the left never moving) the demo scored 0.0: the mark landed 1.5 mm from
300 mm, but with the tape wound in it was not measured.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The board (with the tape's hook over its nail) and the
pen's holder are kinematic, fixed where they are put. The tape's case and the pen are loose rigid bodies made of shapes
from `bodykit.py`.

- The tape: the case slides along the board on the tape's line on a sprung slider (a `Slider` from `bodykit.py`)
  whose spring pulls it back toward the hook with about 2 N wound in and 3.9 N fully out. The tape itself is 17
  pieces of 3 cm, each with its graduations, which the task file lays on the board after every control step once the
  case's mouth has passed their start, and takes away as the tape winds back in (the rest of the last piece is inside
  the case).
- The ink: 6 dots wait hidden under the table. Each time the pen's tip comes down within 1.5 mm of the board (after
  being at least 4 mm up), the task file lays a dot there and notes how far the tape is out.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the pen and the case still) has held for 1 s, otherwise `score()` at the end of the
episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every object exactly where the task file
puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos; Dream-1's
client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the policy the
same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=measure_and_mark_300_mm_with_a_spring_return_tape.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pull the tape measure's case out along the board, well past the red 300 mm line, and hold it there with one hand; with the other take the pen and dab one mark on the board right beside the tape's 300 mm line, then lift the pen." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tape-dream
```

The scripted demo, which reaches full score: the left hand comes down on the case's grip ridge, leaning 20 degrees
toward its own side to reach across, fingers closing across the ridge, pulls the case straight along the board to
40 cm out and holds it there. The right hand comes straight down on the pen's barrel, takes it out of its holder,
brings it, leaning 20 degrees, over the board just beyond the tape's far edge at the 300 mm line, dabs its tip down on
the board and lifts it. (The pen tips over a little in the fingers as it comes out of its holder; the demo aims against
that.) Then the left lets the case go and the tape winds back in:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=measure_and_mark_300_mm_with_a_spring_return_tape.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pull the tape measure's case out along the board, well past the red 300 mm line, and hold it there with one hand; with the other take the pen and dab one mark on the board right beside the tape's 300 mm line, then lift the pen." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/tape-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/tape-dream/*.json; do inspect-robots video "$log"; done
```

Set `TAPE_DEBUG=1` to print what the score is built from twice a second: how far the tape is out (and the most so
far), where the pen's tip is, how far along the board and how high, every mark so far (how far from the board's end,
how far forward, and whether the tape was out past it), and how many marks are good, near and stray.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (156 s on the clock, with Dream answering in 0.77 s on average), and about 1 minute for the scripted demo (390 steps, 19.5 s of simulated time, 64 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
