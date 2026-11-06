# Put each lid back on the tin it fits

Three round tins (4, 6 and 8 cm across) stand in a row, taped down. Each carries a flat lid of the wrong size, in
a three-way cycle: the 8 cm lid on the 4 cm tin, the 4 cm lid on the 6 cm tin, the 6 cm lid on the 8 cm tin. All
three lids are the same colour and thickness, so only their diameters say which tin each belongs to. No tin is
free, so the robot must set one lid down on the table (or hold it) before the others can move. A lid scores
only on the tin of its own diameter, lying flat and centred, with nothing stacked on it.

| File | What |
|---|---|
| `task.py` | the task file: objects, `success`, `score`, `distance`, scripted `demo` |
| `task.json` | the submission fields (`media` is added by the upload script) |
| `test_task.py` | offline checks (NumPy and pytest, no Isaac Sim) |

## Run a trial

From a checkout of the arena (yam-isaac-lab-arena, tested at `bd62771`) on the GPU machine, with this folder at
`tasks/re-lid-the-tins/`. The layout is fixed (`-E object_jitter=0`), so no seed changes the scene; the
scripted demo is open loop and needs the fixed layout.

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/re-lid-the-tins/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The lids are on the wrong tins. Put each lid on the tin it fits, lying flat and centred. One lid per tin." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir ~/yam-eval/re-lid-the-tins-dream
```

Scripted demo (47.9 s of planned motion, about 960 steps plus the 60-70 steps of overhead seen in sim; the last
lid is let go at 43.5 s, so `success` should end the trial at about 45 s):

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/re-lid-the-tins/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The lids are on the wrong tins. Put each lid on the tin it fits, lying flat and centred. One lid per tin." \
  --scorer task_score --max-steps 1100 --store-frames --log-dir ~/yam-eval/re-lid-the-tins-demo
```

Videos, one MP4 per camera, in each log's `frames/` folder:

```bash
for log in ~/yam-eval/re-lid-the-tins-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end: 1.0 when the trial ended in success, otherwise `score()` at the last step
(0, 1/3, 2/3). Put that number in `task.json` `scores`, e.g. `"Dream-1 run: 0.33; demo: 1.0"`.

Runtime per trial: the run limit is 3,600 steps (3 minutes of simulated time at 20 Hz) for Dream-1 and 1,100 steps
for the demo. Wall clock on an RTX 5090, Isaac start-up included: 98 s for the demo, 304 s
for the Dream-1 run.

Offline tests:

```bash
YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest tasks/re-lid-the-tins/test_task.py -q
```

## What the demo does

Four lid moves, straight-down grips at yaw 0 (a disc needs no turn). The arms' wrists (link 4) bulge sideways:
the offline sweep finds the arms overlapping when one works at the middle tin and the other at its own end tin,
so only one arm at a time works the middle of the table while the other waits at its rest point (x 20, y ±24).

1. Both arms at once, each at the end tin in front of it: the left lifts the 8 cm lid off the 4 cm tin, the right
   lifts the 6 cm lid off the 8 cm tin. The right takes its lid to its rest point and holds it there.
2. Left arm: sets the 8 cm lid down on the table at x 17, y 0 (between the arms), moves the 4 cm lid from the
   middle tin onto the 4 cm tin, and goes to its rest point.
3. Right arm: sets the 6 cm lid on the middle tin, then takes the 8 cm lid from the table to the 8 cm tin, and
   goes to its rest point. Both arms hold still for 1.5 s at the end.

## Design numbers

- **Gripper.** At `FINGER_OPEN` (-0.04695 m per finger) the pads are 93.9 mm apart (`yam_arena/constants.py`;
  arena `tests/test_constants.py`). The largest lid is 80 mm, 13.9 mm under that. The pads are flat on their inner
  face for their lowest 33 mm and 6.4 to 10 mm thick there (URDF collision hulls).
- **Grip height and lid thickness.** The TCP grips 20 mm above a lid's underside, which puts the pad tips 7 mm
  above it and gives 18 mm of pad on the lid's side. On a tin of its own size the tin wall is directly under the
  pads, so the tips must never reach the lid's underside; 7 mm leaves room for the 5 mm slip in the pads seen in
  sim. That is why the lids are 2.5 cm thick.
- **Grip offset and the centre tolerance.** Closing pads do not centre an object: one finger leads and the other
  stays near its open stop, so the object ends up off the TCP by up to the gap that pad started with (12-20 mm in
  earlier tasks, where the fingers opened fully on narrower objects; always toward -y). The demo opens only 7 mm
  past each side of a lid (fully open, 6.95 mm, for the 80 mm lid), so the offset is at most 7 mm, and aims every
  place 3.5 mm to +y (`GRIP_SHIFT_Y`), so a lid lands within 3.5 mm of the target whatever the offset in that
  range. With 5 mm of TCP error on top, the worst case is 8.5 mm, inside the scoring tolerance of 10 mm. The
  offline tests replay the releases with offsets of 0, -3.5 and -7 mm and 5 mm of TCP error in each direction,
  and every case scores 1.
- **Why 10 mm.** 5 mm would be below the grip offset. 10 mm keeps the construct ("the lid visibly
  covers its tin"): it is under the ~2 cm the top camera resolves, so a lid within it looks square on its tin; the
  lid's edge overhangs the tin by at most 1 cm anywhere, which a ruler checks; it is half the smallest tin's
  radius, so the lid rests flat with its centre well inside the rim; and at least 68% of the 4 cm tin's top (84%
  of the 8 cm tin's) is under its lid. Wrong lids fail whatever their centring: they score on no tin.
- **Sizes.** Diameter steps of 2 cm (the smallest size step the top camera shows, at about 2 px); in the start
  layout every mismatch is also visible as an overhang or a bare steel ring. Lid names (`lid_a`, `lid_b`,
  `lid_c`) do not follow size order.
- **Tins fixed.** The tins are taped down: a loose 4 cm tin carrying an 8 cm lid is easy to knock over, and
  knocking over a tin is not what the task measures. Their real counterpart is any closed-top straight-sided steel
  tin of these sizes; open tins would not work, because a smaller lid would fall inside.
- **Reach.** Tins at x = 0.30 m, y = +0.13, 0, -0.13; the parking spot at (0.17, 0.0). Carry height 13 cm above
  the table, which keeps a carried lid's underside 3.5 cm above the tallest thing on the table. Worst IK error
  0.10 mm, 0.013 deg; joint 4 keeps 0.16 rad from its limit.
- **Clearances (offline sweep).** The two arms stay more than 3 cm apart at every step; no link comes within 4 mm
  of a tin or of a lid it is not handling; every place lands at least 3 cm from any other lid. With both arms at
  their own end tins the arms are more than 5 cm apart; with one at the middle tin and the other at its end tin
  they overlap by up to 18 mm, which is what the schedule avoids.

## Assets and licences

None. Every object is a primitive cylinder defined in `task.py`; the room and robot come from the arena.

## Verification

Run on an RTX 5090 with `yam-isaac-lab-arena`, using the commands above (`object_jitter=0`):

- Scripted demo (`demo()`): `task_score` 1.0, ended by success; 98 s wall clock including Isaac start-up.
- Dream-1: `task_score` 0.0 after the full 3,600-step run limit; 304 s wall clock.
- Offline tests: 55 passed.
