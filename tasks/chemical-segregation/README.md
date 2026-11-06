# Store acid and base reagent bottles in separate containment trays

Four reagent bottles: two red acids and one blue base stand in the middle of the table, and a third red acid has
been left in the base tray. The robot stores every bottle upright in its own secondary-containment tray, which
means first noticing and removing the misplaced acid.

## Run a trial

In the Robocurve YAM launchable (Isaac Lab-Arena venv), from this folder:

```bash
INSTR="Store the reagent bottles: every red acid bottle must end up upright in the red-rimmed acid tray and every blue base bottle upright in the blue-rimmed base tray. Never leave an acid and a base in the same tray."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/chem-dream

# Scripted demo (reaches full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1300 --store-frames --log-dir ~/yam-eval/chem-demo
```

The trial's score is `task_score` in the run's log (see `task.json` for the rule). If a run reaches its step
limit, `inspect-robots` asks for a manual grade: answer `skip` to keep the automatic score.

## Runtime and seeds

At most 2 minutes of simulated time per Dream-1 trial (2,400 steps at 20 Hz); the scripted demo ends
after about 53 s. Use `-E object_jitter=0` for the fixed layout in the setup screenshot; the default seeded
jitter shifts each bottle by up to 3 cm.
