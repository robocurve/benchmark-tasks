# Cap a cocktail shaker and shake it with both hands without spilling

A steel shaker tin holding 12 beads (the drink and ice) stands on the table; its cap rests on a post nearby. One arm
puts the cap on the tin and keeps pressing it on, the other holds the tin, and together they tip the shaker onto its
side, shake it back and forth along its axis and stand it back on the table, without losing a bead. The cap is a loose
fit, so it stays on only while it is held. The task is one task file,
[`shake_a_cocktail_with_both_hands.py`](shake_a_cocktail_with_both_hands.py), for the `yam_arena` embodiment (scene,
scoring and the scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The tin (floor and wall panels), the cap (disc, knob
and skirt panels) and each bead are loose rigid bodies built from several shapes, so `bodykit.py` adds such bodies:
importing it hooks `yam_arena.scene` when the simulator starts, without changing yam_arena, and gives `score()` each
body's own pose.

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the tin, the cap and every bead still) has held for
1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). "Shaken" counts changes of
direction of the tin's middle along the tin's own axis while it is tipped past 60 degrees, each after a leg of 2 cm or
more; tipping the tin over and back, or rocking it, makes none. `-E object_jitter=0` keeps every object exactly where
the task file puts it, so every trial starts from the same layout. The cameras render at 1280 x 704 for sharp videos;
Dream-1's client scales every frame to 320 x 176 before sending it, so `-E cam_width=320 -E cam_height=176` gives the
policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=shake_a_cocktail_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the cap on the shaker tin and keep holding it on with one hand, hold the tin with the other, shake the shaker back and forth on its side, then stand it back on the table without losing any beads." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/shaker-dream
```

The scripted demo, which reaches full score: the right arm takes the cap by its knob, lifts it off the post and presses
it onto the tin; the left arm grips the tin's body. Together they lift the shaker 3 cm, tip it forward onto its side
(100 degrees), shake it 4 times, 2.5 cm each way straight forward and back, stand it up and set it down. The YAM
fingers' collision shapes are not quite parallel, so a gripped cap or tin leans a few degrees in the hand; the demo
turns each wrist to keep them level, and while the shaker is off the table it aims each hand 1.2 cm toward the other,
so the soft arms keep the cap pressed on:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=shake_a_cocktail_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Put the cap on the shaker tin and keep holding it on with one hand, hold the tin with the other, shake the shaker back and forth on its side, then stand it back on the table without losing any beads." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/shaker-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/shaker-dream/*.json; do inspect-robots video "$log"; done
```

Set `SHAKE_DEBUG=1` to print what the score is built from (beads kept, how far the tin is tipped, the changes of
direction so far, where the cap is against the rim) once a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (174 s on the clock, with Dream answering in 0.96 s on average), and about 2 minutes for the scripted demo (607 steps, 30.4 s of simulated time, 96 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the tin, the cap and the beads) shift.
