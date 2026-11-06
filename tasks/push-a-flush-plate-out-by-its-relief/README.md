# Push a flush plate out of its nest, then take it by the overhang

A thin yellow plate sits flush in a nest on a short bench, level with the rails along its long edges, so there is no
edge to pinch. A 4 cm slot runs under the plate, open at its right end only; a small steel block closes its left
end, so no finger fits under that end and the plate slides over the block. The plate has to be pushed along the slot until
at least 4 cm of it hangs past the bench's left end (y +8 cm), without pushing it far enough to tip off; it is then
taken by that overhang and laid flat in the grey tray. A full score needs the plate lying flat on the tray
floor after that push. Dropping it on the table, laying it in the tray without ever having pushed it to an overhang, or
leaving it held up in the air scores 0.

## Files

- `push_a_flush_plate_out_by_its_relief.py`: the task file for the `yam_arena` embodiment (scene, `score()`,
  `success()`, `demo()`).
- `belimits.py`: shared geometry helpers (outline overlap, height above a support, tilt, latched peaks) the task file
  imports.
- `task.json`: the task's details.

## Run a trial

On an instance set up by the simulation walkthrough, from this folder:

```bash
INSTR="Push the yellow plate along its slot until it overhangs the bench's left end, take it by the overhang, and lay it flat in the grey tray."

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=push_a_flush_plate_out_by_its_relief.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 2400 --store-frames --log-dir ~/yam-eval/pfp-dream

# Scripted demo (full score): the right arm pushes the plate 7.5 cm along its slot; the left arm takes the
# overhang, draws the plate clear, carries it over the tray, sets it down and moves clear so the bare plate shows
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=push_a_flush_plate_out_by_its_relief.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$INSTR" --scorer task_score \
  --max-steps 1240 --store-frames --log-dir ~/yam-eval/pfp-demo
```

- **Seed and layout:** `-E object_jitter=0` is required: the plate sits 2 mm clear of each rail, so any shift
  would jam it or let it fall.
- **Runtime per trial:** the demo runs about 54 s of simulated time (about 1,080 steps: the arms finish at 52 s and the 10 s hold ends the run); a Dream-1 trial runs up to
  2,400 steps (2 minutes simulated). Wall-clock time on one L40S, including Isaac Sim start-up: about 180 s for the demo and about 370 s for a full Dream-1 trial.
- **Score:** `task_score` reports 1.0 when `success()` holds for 10 s (plate in the tray, flat and still after a
  push to an overhang), otherwise `score()` at the end of the run. The hold is 10 s so the run outlasts the hand's
  exit: the plate settles at about 44 s and the hand is clear at 52 s, so the run ends at about 54 s with about 2 s of
  bare plate in the top view, before `yam_scripted` stops at 55 s. `score()` still reports 1.0 at the end of a
  time-limited run, so Dream-1 loses nothing.

## Scoring

Read from the end state, and from whether the plate ever touched the table outside the tray (a latch over the run,
cleared only by a teleport: a plate position change of more than 5 cm between two scoring calls, which only a
simulator reset produces):

1. **Score 0** if the plate ever had a point within 0.5 cm of the table top outside the tray's outer outline (or on
   the floor), or if at the last step its bottom is more than 1 cm above every surface under it (held, or in flight).
   A plate pushed off the bench end, or slid along the table toward the tray, scores 0.
2. **1.0** if the plate, having lain flat, moving or still, on the bench with at least 4 cm of it past the left end (y +8 cm; its rear
   end at or past y -4), lies wholly inside the tray's inner outline (x 7 to 21, y 0 to 20 cm), bottom within 3 mm of
   the tray floor, flat (tilt at most 5 degrees, one end at most 1.4 cm higher than the other; either face up), and at
   rest. `success()` needs it to be still for 10 s; the end-of-run `score()` tests the instantaneous speed
   (under 1 cm/s).
3. **0.6:** at rest but not flat on the floor, with its centre over the tray outline (perched or leaning on a wall,
   tilted past 5 degrees, or on edge), after that push.
4. **0.3** if it is still lying flat on the bench (bottom within 6 mm of the strips' top, tilt at most 5 degrees,
   centre not past y +8) with at least 4 cm past the left end.
5. Otherwise **0**, including doing nothing.

The window is set by the plate itself: under 4 cm past the end leaves nothing to take, and past 8 cm (its centre
beyond the end) it tips off and falls. A plate drawn along the bench past 4 cm counts too: the window is set by how far it reaches, not by whether it stopped there. In simulation the outline tests use the plate's actual pose.

The simulator scores the final step without opening the grippers. A plate still held more than 1 cm above every
surface under it counts as not placed, so holding it over the tray never scores better than letting go. A real trial
opens the grippers before measuring.

## Real-world reset

Set the plate back in its nest between the rails, level with them, with its ends at y -8 and +8 cm. Check a plate
that fell for bends before reuse. The strips, rails and tray stay in place.
