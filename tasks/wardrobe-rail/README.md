# Refit a wardrobe rail between a closed and an open end support

A wardrobe section stands on the table: a base board and two white side panels 60 cm apart. The left panel carries a
closed end support (a cup), the right panel an open U support. The rail, an oval steel wardrobe tube 59.2 cm long,
lies on two foam blocks in front of the arms. The robot must fit the rail into both supports and let go. The scene,
scorer and scripted demo are in `wardrobe_rail.py`.

The rail is a few millimetres shorter than the gap between the panels, as real rails are cut, but it is longer than
the gap minus the closed cup's depth. It therefore cannot be put in level: pushed in level, its far end hits the right
panel before the near end reaches the cup. The way people do it is to hold the rail tilted with the right end high,
feed the left end into the closed cup, then swing the rail down level into the U while the left end slides home.
That needs both arms: they hold the rail at two points and move together so the rail turns in both grips while the
left end stays in line with the bore (2.1 x 3.8 cm bore for a 1.5 x 3.0 cm rail).

## Files

| File | What |
|---|---|
| `wardrobe_rail.py` | The yam_arena task file: objects, `score()` and the scripted demo |
| `partkit.py` | Rigid parts built from boxes (from the accepted `din-rail-mount` task) |
| `task.json` | The submission details |

## Run a trial

In the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`), from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/wardrobe-rail/wardrobe_rail.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the wardrobe rail into its two end supports." \
  --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream
```

Scripted demo: the same flags with `--policy yam_scripted --max-steps 640`. The score is printed as `task_score`.
Wrong attempts for the shortcut checks: set `RAIL_VARIANT` to `level`, `on_top`, `hold` or `nothing` before the scripted
run. `RAIL_DEBUG=1` prints the rail end positions every 20 steps.

## Scoring

Computed from the final state. Credit is given only if both grippers are open (fingers at least 90% open) with both
pad tips at least 3 cm from the rail's axis, and the rail has moved less than 1 mm, for the last 1 s.

- 0.4: the left end is in the closed support: its centre at least 0.8 cm into the bore and inside the bore's outline.
- 0.4: the right end rests in the U support: its centre at least 0.5 cm over the floor and within 0.3 cm of the
  support axis height (so it lies on the floor, not on the cheeks).
- 0.2: both ends in and the rail level within 1 degree.

A real trial is scored by eye and with a ruler: the rail's left end is visibly inside the cup, the right end lies in
the U, and a spirit level on the rail reads level.

## Shortcut and robustness checks (scripted)

| Variant | What it does | Score |
|---|---|---|
| Demo | Tilts, feeds the left end, swings level, lets go, parks | 1.0 (also 1.0 over two trials with a reset between) |
| `level` | Carries the rail in level and pushes it toward the closed cup | 0 (the far end meets the right panel; the rail ends on the base board) |
| `on_top` | Lays the rail across the tops of both supports | 0 (it slides off onto the base board) |
| `hold` | Fits it correctly but never lets go | 0 |
| `nothing` | Both arms park | 0 |
| Dream-1 | One run | 0.0 (see Results) |

## Results

Dream-1: 0.0. Over about 2,000 steps (100 s) both arms move toward the rail and the panels, but the rail is never
lifted off the foam blocks. The Dream-1 session expired at about step 2,000 and the run was stopped there; the video
covers those steps. Demo: 1.0.

## Note for reviewers

This is not the "seat a bar across two supports" family: with two open supports a bar is simply lowered in. Here the
left support is a closed cup and the rail is longer than the gap minus the cup's depth, so it cannot be put in level
(tested: the `level` attempt scores 0). The left end has to be fed in along the bore with the rail tilted about 12
degrees and the right end high, then the rail swung down into the U while the left end slides home, which is how
wardrobe and closet rails are fitted.

## Runtime and seeds

- Scripted demo: about 560 steps (28 s simulated).
- Dream-1: 2,400 steps (2 minutes simulated).
- No randomness: run with `-E object_jitter=0`. No seeds.

## Fidelity

Rigid bodies built from boxes. The oval tube is modelled as a 3.0 x 1.5 cm rectangular bar and the supports as
rectangular bores, with 0.3 cm clearance at the sides and 0.4 cm at the top and bottom. The wardrobe section is one
fixed 25 kg part. Friction 1.2 everywhere. No screws or fixings. The scripted demo aims the left grip 4 mm toward +x
because the left wrist turns a few degrees short of the commanded angle in this pose.

## Real-world build

A wardrobe section from two 16 cm offcuts of 18 mm melamine board screwed to a base board, 60.0 cm apart inside,
with a standard closed oval end support and an open U support for 30 x 15 mm oval wardrobe tube, screwed on at the
same height. A length of oval tube cut to 59.2 cm. Two foam blocks.

## Licence

The task is released under the MIT licence. It uses no library or third-party assets: every object is built from
plain boxes and released under the task licence. `partkit.py` is reused from the accepted `din-rail-mount` task in
this repository (same licence as the repository).
