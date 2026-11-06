# Lift a skimmer basket out of its well by its folded handle

A skimmer basket full of leaves sits in a close-fitting well in a pool deck, its rim 30 mm below the deck surface.
The robot has to take it out and set it on a tray on the deck. There is 4 mm of clearance between the basket and the
well wall, and the gripper's fingers are 10 mm thick, so the basket's side cannot be pinched; the leaves fill it to
the rim, so it cannot be gripped from inside. The only thing to hold is the wire handle, which lies folded flat over
the leaves. A pull on the folded handle only swings it up about its hinge, so it has to be raised to upright before
it can lift the basket.

Files: `task.py` (scene, scoring, scripted demo), `parts.py` (the parts kit, used for the basket's hinged handle; an
unmodified copy), `near_misses.py` (scored near-successes and unintended strategies), `task.json`.

## Run a trial

Simulator: yam-isaac-lab-arena (the program's YAM embodiment for Isaac Lab-Arena), station scene. Submissions run
with `object_jitter=0`; this task needs it, because jitter of up to 3 cm would push the basket into the well wall.

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the skimmer basket out and set it on the deck tray." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/dream
# Scripted demo (fixed waypoints only)
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Take the skimmer basket out and set it on the deck tray." \
  --scorer task_score --max-steps 600 --store-frames --log-dir logs/demo
```

Run limit: 1,200 steps (60 s at 20 Hz). The demo reaches success at about 22 s. One trial takes about 1-2 minutes on
an L40S. The scene has no random elements: there are no seeds, and with `object_jitter=0` every reset is the same.

## Scoring

`--scorer task_score` reports 1.0 once `success()` has held for 1 s; otherwise it reports `score()` at the end of the
run. The task is pass/fail: `score()` is 1.0 exactly when `success()` holds.

- Success: the basket's base centre is within 1 mm of the tray floor (78 mm above the table), the basket is upright
  within 5 degrees, its centre is within 15 mm of the tray centre along x and along y (so the whole 100 mm basket is
  inside the 130 mm tray), and it is still (under 1 cm/s and 0.5 rad/s). The handle's angle does not matter.
- Whole-trial record: if the basket ever tips more than 45 degrees from upright, the trial scores 0 even if the basket
  ends on the tray. The record is cleared only by the scene-reset callback (`parts.on_reset`), which the robot cannot
  trigger.
- `score()` reads only object poses and cannot see the grippers. A basket held still with its base on the tray floor
  would count. The demo opens its fingers with the base 7 mm above the floor, and the GPU trace shows the basket
  dropping to the floor after the fingers open, before success starts to hold.

Ruled out (see `near_misses.py`): the basket still in the well with the handle up or folded; lifted and held over the
tray (3 mm and 7 mm up); propped on the tray rim (tilted 6 degrees) or 16 mm off centre; on the deck or the table;
still moving; and tipped past 45 degrees on the way, then set right.

## Demo

Fixed waypoints only; the left arm does everything, fingers closing along y.
1. Straddle the folded grip bar with the fingers 20 mm apart, the pad tips 2 mm below the bar's axis (just above the
   leaves), and pinch it lightly (pads commanded 7 mm apart on the 10 mm bar) so it can turn between the pads.
2. Move along the handle's arc to 120 degrees (the handle lags by about 15 degrees), pushing it over onto its stop at
   95 degrees, and let go.
3. Come down over the standing bar, pad tips 9 mm below its axis, close fully and lift straight up out of the well
   (base about 7 mm above the tray rim), carry 16 cm to the right over the tray, wait for the swing to die, lower to
   7 mm above the tray floor and let go.

Measured on the GPU: the hanging basket rides about 6 mm below the commanded height. Lifting by the bar with the pad
tips only 2 mm below its axis failed in two traces (the bar slipped out of the pads), which is why the demo re-grips
lower once the handle stands on its stop.
