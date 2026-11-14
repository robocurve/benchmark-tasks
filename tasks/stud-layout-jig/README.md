# Seat studs in a framing layout jig

A plywood layout jig lies between the arms, with a 2x4 end fence across its far side and four slots, 12 inches on
centre, each formed by a pair of short cleats standing at the fence. Four loose 2x4 studs (38 x 89 x 368 mm) stand on
edge nearby: two on the plywood near the middle, one on the table beside each side of the jig. Every stud has to end up
on edge in its own slot, flat on the plywood and tight against the fence. The slots leave 3 mm per side, so a stud
has to arrive nearly square and on centre; the cleat tops are rounded over toward the slot, so a stud set down slightly
off centre slides in.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=stud_layout_jig.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat the four 2x4 studs on edge in the layout jig, tight against the end fence." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/stud-layout-jig-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=stud_layout_jig.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Seat the four 2x4 studs on edge in the layout jig, tight against the end fence." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/stud-layout-jig-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/stud-layout-jig-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

The scripted demo is open loop and uses only fixed constants. Each arm handles the two slots on its side: it picks a
stud by its top edge, carries it over the slot, lowers it to just above the cleats, slides it forward until its end
presses on the fence, and lets go. The stud drops into the slot (the rounded cleat tops steer it in), and the arm then
pushes the stud's near end with closed fingers to close the last few millimetres to the fence.

The scoring and the demo can be checked without the simulator:

```bash
python -m pytest -q test_stud_layout_jig.py
```

## Scoring
Each stud scores on its own, and each slot counts for at most one stud.

| Score per stud | Condition | Check on a real table |
|---|---|---|
| 0.25 | seated (below) and its end within 2 mm of the fence face | a 2 mm feeler gauge doesn't enter between the stud's end and the fence |
| 0.15 | seated, but its end more than 2 mm from the fence | |
| 0 | anything else: lying flat, resting on the cleat tops, beside or behind a slot, tilted, turned or on the table | |

A stud is **seated** in a slot when it stands on edge (its 89 mm face upright within 3 degrees), its length is square
to the fence within 3 degrees, its bottom is within 3 mm of the plywood, its centre line where it passes the cleats is
within 4 mm of the slot's centre line (it is between that slot's cleats), at least 76 mm of its length (half the cleat
length) is between the cleats, and it is still. A real check: the stud sits flat on the plywood between the cleats, a
square against the fence reads within 3 degrees, and a ruler shows at least 76 mm of the stud inside the cleats.

The trial's score is the sum over the four studs, up to 1.0; 1.0 requires all four seated and within 2 mm of the
fence for 1 s. In simulation the trial is voided (scores 0) if a stud is missing, a pose is not finite, a rotation is
not unit length, or a stud's reported outline doesn't match a 38 x 89 x 368 mm box.

## Runtime and seeds
- The scripted demo seats all four studs and the trial ends with success at about 37 s of simulated time: three
  trials back to back took 742, 741 and 745 steps, all scoring 1.0. The three trials, including starting Isaac Sim,
  took 7 minutes on one GPU. A full 2,400-step trial with Dream-1, including start-up, took about 7 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
