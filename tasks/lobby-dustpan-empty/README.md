# Empty an upright lobby dustpan into a wastebasket and park it on its mark

Two arms empty mixed sweepings (a crumpled paper ball, a bottle cap, a folded card and a tape core) from a full-size upright lobby dustpan (its hopper hangs on a free pivot) into a wastebasket, then stand the empty dustpan back on its floor mark with the handle upright and let go.

## Run a trial
Scripted demo (the scene, demo and scorer are all in `task.py`; two helper modules implement the scripted motion):
```
LD_MODE=integrated LD_REPINCH_HOLD=0.365,-0.230,0.650 \
inspect-robots run --embodiment yam_arena --policy yam_scripted -P hold_s=1.0 \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Empty the sweepings (paper ball, bottle cap, folded card, tape core) from the upright lobby dustpan into the wastebasket, then stand the dustpan back on its floor mark with its handle upright." \
  --scorer task_score --max-steps 1280 --store-frames --log-dir logs/demo
```
Dream-1 command:
```
inspect-robots run --embodiment yam_arena --policy dream   -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352   --instruction "Empty the sweepings (paper ball, bottle cap, folded card, tape core) from the upright lobby dustpan into the wastebasket, then stand the dustpan back on its floor mark with its handle upright."   --scorer task_score --max-steps 1280 --store-frames --log-dir logs/dream1
```
The recorded scripted demo also set `LD_MODE=integrated`, `LD_REPINCH_HOLD=0.365,-0.230,0.650` and `LD_LOG=1`. The first two only select the scripted demo's plan, and `LD_LOG` only prints telemetry lines; none changes the scene, the policy input or the scorer.

- Runtime per trial: 292 s wall time on one RTX 5090 for the scripted demo (1,261 control steps; the episode ends on the scorer's success hold) and 312 s for the Dream-1 run (all 1,280 steps), with frames stored at 640x352.
- Seeds: none. `object_jitter=0`, one fixed starting layout.
- Score: the `task_score` scorer reads the task's own `score()`; `[score]` lines print every 10 steps. Possible scores 0, 0.5 and 1.0.

## Scoring
The scorer checks each item's forward passage through the hopper mouth during a 15-degree-or-steeper over-bin tip, followed within 1 s by downward passage through the bin opening. The logic uses the pictured lip and each item's centre, sampled at 20 Hz, with interpolation between adjacent samples. PhysX reports robot/item contacts at physics substeps. The final pose, release and stillness gates apply to both positive tiers. See task.json `scoring`. The thresholds are the scorer constants: `RELEASE` (80 mm hand clearance), `STILL_N`/`STILL_TOL` (21 samples at 20 Hz, 5 mm), and in `score()` the tape rule (`TAPE_OUT_HX`/`TAPE_OUT_HY`: all four hopper base corners remain inside the tape's outer edge throughout the final 21 samples, 1.0 only), 15 mm, 10 deg and 15 deg limits; an item counts in the wastebasket or the hopper only if its whole shape (core and cap cylinders, card box, paper-ball hull) lies inside the inner walls (orientation-aware extents). 0.5 needs at least two of the four items tipped in.

## Real trial
The yellow tape is 14 mm wide. Place its centre lines 400 x 360 mm apart, making the outer rectangle 414 x 374 mm. Verify all four base corners against that outer rectangle throughout the final second of the overhead video.

Stock items only: the dustpan (FG253100), the wastebasket (FG295600), one 3 in kraft tape core, one sheet of A4 80 g/m2 copy paper, a soda-bottle cap, a paperboard card, floor tape and a 100 mm scale bar. Reset per task.json `reset_notes`; read the result with a ruler and phone level, and use synchronized video for the 80 mm release and the 5 mm stillness over the last second.

## Files
- `task.py`: scene, scripted demo and scorer.
- Two helper modules implement the scripted motion; `task.py` imports them.
- `labkit.py`, `arena_assets_fix.py`: scene helpers.
- `transfer_evidence.py`, `core_contacts.py`: per-item transfer gates and named PhysX contact reports (all four items are contact sensors).
