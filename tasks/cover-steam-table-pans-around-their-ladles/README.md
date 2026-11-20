# Cover three steam-table pans, two with ladles in them

A stainless steam-table section holds three 1/3-size hotel pans (32.5 x 17.6 cm, 10 cm deep) side by side. The left
and right pans each have a ladle resting in the food with its handle leaning on the far rim; the middle pan has none.
Three stainless covers lie beside the section: one solid, two with a notch in one short end. The robot is asked only to
cover every pan to keep the food hot. A ladle stays in its pan during service, so a ladle pan takes a notched cover with
the handle out through the notch; a solid cover sits proud on the handle.

| File | What |
|---|---|
| `steamtable.py` | Task file: objects, scoring (`score`), scripted demo (`demo`) |
| `mech.py` | Helper for compound rigid bodies (covers, ladles, the steam-table section) |
| `test_steamtable.py` | Offline scorer tests (`python test_steamtable.py`) |
| `task.json` | Submission details |

## Run a trial

```bash
INSTR="Cover every pan on the steam table to keep the food hot until the next rush."
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=steamtable.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=steamtable.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score --max-steps 1000 --store-frames --log-dir ~/yam-eval
```

## Runtime and seeds

Run limit 1,800 control steps (90 s at 20 Hz); the scripted demo takes about 52 s. Use `-E control_hz=20` and
`-E object_jitter=0` (fixed layout, no seed).

## Scoring

Each pan is worth one third. A pan scores when a cover rests flat on its rim (underside within 3 mm of the rim top,
centre within 1.5 cm of the pan's, within 3 degrees of level and 5 degrees of square). A pan with a ladle scores 1/3
for any cover that rests flat, provided that ladle has never moved 3 cm or turned 10 degrees since the start. The
empty pan scores 1/3 for a solid cover and 1/6 for a notched one. A ladle taken out or pushed into the food zeroes its
pan for the rest of the trial. Counted only at the end with covers and ladles still for 1 s (under 3 mm) and no
gripper finger or fingertip midpoint within 3 cm of any cover's plate, posts or strap (a cover held or touched anywhere,
including its edge, is not released); otherwise 0. A cover's centre is its full plate's centre, taken from a marker
welded at the plate centre (a notched cover's body origin is 3 cm off it). Doing nothing scores 0.

Covers: the sim plate is 3 mm (thicker than a real ~0.7 mm sheet, for stable contact) at the real mass, 0.35 kg solid / 0.33 kg notched.

Real trials: unheated pans with dry rice, dry beans and lentils; stainless 1/3-size pans, 1/3-size flat covers (two
with a 6 x 5 cm notch cut in a short end, e.g. standard slotted covers) and 2 oz-class ladles; all commercial parts.

## License

MIT.
