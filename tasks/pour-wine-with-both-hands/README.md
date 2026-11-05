# Pour a glass of wine with both hands, one on the bottle's neck and one on its body

Both arms pick up an upright wine bottle, one gripper on the neck and the other on the body, tip it together and pour
the wine (10 beads) into a stemmed glass without spilling or touching the glass. While the bottle is tipped past 45
degrees both grippers must be holding it, so it cannot be poured one-handed. The task is one task file,
[`pour_wine_with_both_hands.py`](pour_wine_with_both_hands.py), for the `yam_arena` embodiment (scene, scoring and the
scripted demo), plus [`bodykit.py`](bodykit.py), which it imports from the same folder.

## Requirements

[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) (tested at commit `bd62771`) on
Isaac Lab-Arena 0.3.1 / Isaac Sim 6.1, on Linux with an NVIDIA RTX GPU. Its `setup.sh`, or the program's Brev
launchable, installs everything, including the Dream-1 policy. The task needs no models from Arena's object library.

`yam_arena` task files place single boxes, cylinders and spheres. The hollow bottle (base, body, sloping shoulder and
neck built from wall panels), the stemmed glass and each bead are loose rigid bodies built from several shapes, so
`bodykit.py` adds such bodies: importing it hooks `yam_arena.scene` when the simulator starts, without changing
yam_arena, and gives `score()` each body's own pose. It also hooks the embodiment's step, so the task can check after
every control step whether both grippers are holding the bottle, and lets the scripted demo give a grip as a full
rotation (pouring needs level grips that roll).

## Run and score a trial

The scorer is `task_score`: 1.0 when `success()` (full score, with the bottle, the glass and every bead still) has
held for 1 s, otherwise `score()` at the end of the episode (0, 0.25, 0.5 or 1.0; see `task.json`). `-E object_jitter=0`
keeps every object exactly where the task file puts it, so every trial starts from the same layout. The cameras render
at 1280 x 704 for sharp videos; Dream-1's client scales every frame to 320 x 176 before sending it, so
`-E cam_width=320 -E cam_height=176` gives the policy the same input.

Dream-1 (needs a key in `~/.dreamlabs/credentials`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pour_wine_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pick up the wine bottle with both hands, one on its neck and one on its body, and pour the wine into the glass without spilling any or knocking the glass over." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/wine-dream
```

The scripted demo, which reaches full score: both arms reach the bottle level from their own sides (fingers closing
front and back across it), the right on the neck and the left on the body; they lift it together, tip it toward the
glass, bring the mouth over the glass's middle with the bottle tipped 95 degrees (nothing runs out yet), roll both
wrists on to 110 degrees so the beads roll out into the glass, then stand the bottle back on its mark:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pour_wine_with_both_hands.py -E object_jitter=0 -E control_hz=20 -E cam_width=1280 -E cam_height=704 \
  --instruction "Pick up the wine bottle with both hands, one on its neck and one on its body, and pour the wine into the glass without spilling any or knocking the glass over." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/wine-demo
```

Videos, one MP4 per camera next to each run's frames:

```bash
for log in ~/yam-eval/wine-dream/*.json; do inspect-robots video "$log"; done
```

Set `WINE_DEBUG=1` to print what the score is built from (beads in the glass, in the bottle and spilled; the bottle's
tip; where the mouth is over the glass) once a second.

## Runtime per trial

On one RTX 5090: about 3 minutes for a Dream-1 trial of 1,200 steps (193 s on the clock, with Dream answering in 0.80 s on average), and about 2 minutes for the scripted demo (586 steps, 29.3 s of simulated time, 97 s on the clock). Isaac Sim's first start on a new machine adds a few minutes while it compiles its shaders.

## Seeds

None needed: with `-E object_jitter=0` the layout is fixed and the scripted demo is deterministic. With a nonzero
`object_jitter`, the trial's seed sets how far the free objects (the bottle, the beads and the glass) shift.
