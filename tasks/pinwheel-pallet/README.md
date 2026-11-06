# Palletize a pinwheel layer

Four identical cartons (100 x 50 x 40 mm) lie two in front of each arm; a fixed blue pallet (170 x 170 mm) stands on
the midline between them. One carton has to end up in each corner of the pallet, flat, square, with its two outer sides
flush with the pallet's edges. Two cartons along one edge need 200 mm of a 170 mm edge, so four filled corners are only
possible as a pinwheel: two cartons turned 90 degrees from the other two. In the demo each arm fills its own half of
the pallet, so each turns one of its two cartons. The instruction doesn't mention the pinwheel.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pinwheel_pallet.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put one carton in each corner of the pallet, lying flat on its largest face and square to the pallet, with its two outer sides flush with the pallet's edges." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pinwheel-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pinwheel_pallet.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put one carton in each corner of the pallet, lying flat on its largest face and square to the pallet, with its two outer sides flush with the pallet's edges." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pinwheel-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/pinwheel-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
A carton fills a pallet corner when all of these hold:
- it lies flat on its largest (100 x 50 mm) face, resting on the pallet itself, not on another carton or held above it:
  its 40 mm side within 5 degrees of vertical, and in the sim its bottom within 1 mm of the pallet's top;
- its sides are within 5 degrees of square to the pallet's edges;
- each of its two outer sides is between 3 mm past and 15 mm inside the pallet's edge at that corner.

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | all four corners filled and every carton still for 1 s (ends the trial), or four filled at the last step | a ruler at each corner, both edges |
| 0.75, 0.5, 0.25 | three, two or one corners filled | |
| 0 | no corner filled, including cartons standing on end or on their 100 x 40 mm sides | |

Two cartons along one edge overlap by at least 30 mm, less the 2 x 3 mm they may stand past it, so the only way to
fill all four corners is a pinwheel, either way round. One carton can't fill two corners and two can't share one, so the
score counts cartons too. `score()` gives 1.0 for four filled corners even while a carton moves; `success()` also
needs every carton still. The hooks don't see the grippers, so a carton held above the pallet doesn't count (its bottom
must be within 1 mm of the top; one set down reads 0.0 mm in the sim), but one held still while touching the pallet
does: the hooks can't tell it from one set down. The flush check uses each carton's bounding box, so a tilted
carton's raised corner counts.

## The demo's aim offsets
In the sim a gripped carton hangs about 22 mm off the TCP along the closing axis and centres again when it touches
down. `AIM` and `LAST_GRIP_SHIFT` in `pinwheel_pallet.py` cancel that drift so the scripted demo lands; they are
not part of the scoring.

## Runtime and seeds
- The scripted demo fills all four corners and the trial ends with success at 39.8 s of simulated time (796 steps).
  On an L40S shared with one other simulation, the whole demo run, including starting Isaac Sim, took about 2.5
  minutes; a full 2,400-step trial with Dream-1 took about 7 minutes. Isaac Sim's first start on a machine takes
  about 6 minutes more.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
