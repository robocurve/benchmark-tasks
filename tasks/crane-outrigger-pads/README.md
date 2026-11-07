# Size and place crane outrigger pads for the ground under each

A 1:20 crane will put its outriggers down on two landing marks, one on soft clay and one on concrete. Each outrigger
presses 120 kN. Clay takes 100 kPa, so it needs a pad of at least 1.2 m2: only the large pad (1.2 m square, 1.44 m2)
works there. Concrete takes 400 kPa and needs 0.3 m2, so the small pad (0.9 m square) is enough. Each pad starts on
the opposite side of the crane from where it must go, so both have to cross over.

## Files

| File | What |
|---|---|
| `task.json` | Task details for review |
| `crane_outrigger_pads.py` | `yam_arena` task file: scene, `success()`, `score()`, `distance()` and the scripted `demo()` |

## Run a trial

Station scene, fixed layout, no randomization (`object_jitter=0`), no seeds. Run from the repository root.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/crane-outrigger-pads/crane_outrigger_pads.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put one outrigger pad flat on each dark landing mark, centred on it, so that neither pad overloads its ground. Each outrigger presses 120 kN on its pad. The brown clay can take 100 kPa and the grey concrete 400 kPa. At full size the large pad is 1.2 m square and the small pad 0.9 m square." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/crane-dream

# Scripted demo (reaches 1.0)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/crane-outrigger-pads/crane_outrigger_pads.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put one outrigger pad flat on each dark landing mark, centred on it, so that neither pad overloads its ground. Each outrigger presses 120 kN on its pad. The brown clay can take 100 kPa and the grey concrete 400 kPa. At full size the large pad is 1.2 m square and the small pad 0.9 m square." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/crane-demo
```

Runtime per trial: up to 2 minutes of robot time (2,400 steps at 20 Hz); the scripted demo finishes its motions
after about 47 s.

## Scoring

`score()` implements the rubric in `task.json`; `adequate()` does the pressure check (120 kN / full-size pad area
against the ground's capacity). Putting each pad on the nearer mark, or swapping them, scores 0.5; one pad on a mark
scores 0.25.

## Scripted demo

Four moves, one arm at a time: the right arm sets the large pad on a handoff spot in front of the crane and the left
arm takes it to the clay mark; then the left arm sets the small pad on the handoff spot and the right arm takes it to
the concrete mark.
