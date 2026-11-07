# Pass open vials between the arms over a drip tray

Two open vials stand at the left end of a grey drip tray and two white pads mark its right end. Each arm reaches only
its own half of the tray, so every vial has to be passed from the left arm to the right arm near the middle. The vials
are open, so no part of either may be outside the tray's edges at any moment, in the air or during the hand-off.
A full score needs both vials upright and still on the pads, each having been lifted clear of the tray on both halves.
A vial that leaves the tray's outline at any height, even once, scores 0 for the whole trial.

## Files

- `open_vials_cross_the_drip_tray.py`: the task file for the `yam_arena` embodiment (scene, `score()`, `success()`,
  `demo()`).
- `belimits.py`: small geometry helpers (outline tests, a reset-aware latch) the task file imports.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Move both open vials from the left end of the grey drip tray onto the white pads at its right end."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=open_vials_cross_the_drip_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/vials-dream

# Scripted demo (full score): the left arm lifts each vial and sets it in the shared strip at x 24, y 0; the right
# arm takes it from there to a pad. Only one arm is ever low in the strip.
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=open_vials_cross_the_drip_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 800 --store-frames --log-dir ~/yam-eval/vials-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: it keeps the vials exactly where the task file puts them, 2 cm
  inside the tray's edges, and the scorer finds the start of a trial from those spots.
- **Runtime per trial:** the demo takes about 35 s of simulated time (about 700 steps); a Dream-1 trial runs up to 2,400
  steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 120 s for the demo and about 350 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 6 s (both vials placed and still, long enough to see the grippers open and the arms clear), otherwise `score()`
  at the end of the run.

## Scoring

Checked every step, and at the end of a real trial with the grippers open:

1. **Score 0** if at any moment any part of either vial's outline, seen from above, is more than 2 mm outside the
   plate's edges (x 17 to 31 cm, y -20 to +20 cm), at any height, or a vial falls below the table top. The simulator
   latches this over the whole run, so a vial that goes out and comes back still scores 0.
2. **Otherwise 0.5 per vial** that has been lifted 3 cm or more clear of the plate on the left half (y +6 cm or more)
   and on the right half (y -6 cm or less), and now stands on a pad: tilt 15 degrees or less, speed under 1 cm/s,
   centre within 2 cm of the pad's centre, underside within 1 cm of the pad top (0.6 cm above the table).
   A vial still held in the air, stacked, tipped over, or slid across the plate without being lifted earns 0.

Doing nothing scores 0 (the vials stay at the left end). One vial delivered is 0.5. In simulation a held vial is never
scored higher than a released one: a vial more than 1 cm above every support counts as not placed. The arms' paths are
not scored; only where the vials are, step by step. On the real rig the per-step check is read from overhead video.

## Real-world reset

Stand the two vials back on the plate at x 21 and 27, y +16, and check the pads at x 21 and 27, y -16. The lip and pads
stay down. Use empty or sand-ballasted jars; no liquid is needed.
