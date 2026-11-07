# Press a ruler flat with one hand while the other draws a straight line along its edge with a marker

A sheet of paper lies on the table with a thick wooden ruler across it; a felt-tip marker stands in a holder beside it.
One hand presses the ruler down on the paper; the other takes the marker and draws a straight line along the ruler's
near edge, the tip riding against the edge, at least 18 cm, then lifts the marker off the paper and puts it down.
Then both hands slide the ruler away, showing the line, and both arms go back to where they started. The ink shows up
as the line is drawn. The task is one task file,
[`draw_a_line_along_a_held_ruler.py`](draw_a_line_along_a_held_ruler.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

What it tests: using a straightedge as a guide, pressing the pen against it just enough, while the other hand keeps
the guide from moving.

Why two hands: the ruler is light (25 g) and loose on the paper, and the tip has to ride against its edge (the score
asks for it), which pushes it. Not held, it slides away ahead of the tip: with the left hand kept clear, the scripted
demo's stroke pushes the ruler off the line and the tip touches its edge for only 10% of the line (0.5). One hand
presses the ruler down while the other draws.

## How it is modelled

`yam_arena` task files place single boxes, cylinders and spheres. The paper and the marker's holder are kinematic,
fixed where they are put; the ruler (with its centimetre marks) and the marker are loose rigid bodies made of shapes
from `bodykit.py`.

- The ink: 40 small dark dots wait hidden under the paper. After every control step, while the marker's tip is within
  1.5 mm of the paper, the task file lays the next dot where the tip is whenever it has moved 3.5 mm since the last one;
  the dots overlap into a line. With each dot it notes whether the tip is touching the ruler's near edge, as the ruler
  lies then.

Importing `bodykit.py` hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, gives `score()`
each body's own pose, and lets the task file act on the simulation after every control step.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, which includes the marker laid down and both arms back
where they started, with the marker and the ruler still) has held for 1 s,
otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0` keeps every
object exactly where the task file puts it, so every trial starts from the same layout. The cameras render at
1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=draw_a_line_along_a_held_ruler.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Press the ruler down on the paper with one hand; with the other take the marker from its holder and draw a straight line along the ruler's near edge, at least 18 cm, with the marker's tip against the edge; then lay the marker down beside the paper, slide the ruler away from the line with both hands so the line shows, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/ruler-dream
```

The scripted demo, which reaches full score: the left hand comes straight down on the ruler's left end, fingers
closed, and presses it onto the paper. The right hand comes straight down on the marker, fingers closing across its
barrel, lifts it out of its holder and carries it to the ruler's right end; it sets the tip on the paper by the
ruler's near edge and draws along the edge to the left, 20 cm, pressing the tip lightly against the edge; then it
lifts the marker off the paper and sets it down on the table beside the paper. Both hands, fingers closed, press on
the ruler and slide it away from the line, showing it, and both arms go back to where they started:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=draw_a_line_along_a_held_ruler.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Press the ruler down on the paper with one hand; with the other take the marker from its holder and draw a straight line along the ruler's near edge, at least 18 cm, with the marker's tip against the edge; then lay the marker down beside the paper, slide the ruler away from the line with both hands so the line shows, and take both arms back to where they started." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/ruler-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/ruler-dream/*.json; do inspect-robots video "$log"; done
```

Set `PEN_DEBUG=1` to print what the score is built from (where the marker's tip and the ruler are, how many dots have
been laid and how long the line is, how far its worst bit is from the line along the edge, the share of it drawn with
the tip riding the edge, and whether the marker is lifted) twice a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (159 s on the clock, with Dream answering in 0.76 s on average), and about 3 minutes for the scripted demo (609 steps, 30.4 s of simulated time, 190 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed, and the scripted demo follows fixed waypoints.
