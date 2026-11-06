# Pack a lunch box with the yogurt and the sandwich against the ice pack

Five lunch items go into a roomy lunch box: an ice pack, a yogurt cup, a sandwich, an apple and a box of crackers.
Food-safety guidance says the perishables (the yogurt and the sandwich) go directly against the frozen pack, and the
shelf-stable snacks can go anywhere. The box has plenty of space, so the perishables only end up against the pack if
the robot puts them there on purpose. The right arm can reach the ice pack and the left arm the perishables. The
left arm has to set its items against faces of the pack that the right arm left free, so a full score takes both
arms, one after the other.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=lunch_perishables_touching_ice_pack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack the lunch into the navy box: lay the blue ice pack flat on the box floor, then stand the pink yogurt cup upright and lay the yellow sandwich flat on the box floor, each directly against the ice pack so they stay cold. The red apple and the brown crackers (lying flat) can go anywhere on the box floor." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/lunch-pack-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=lunch_perishables_touching_ice_pack.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack the lunch into the navy box: lay the blue ice pack flat on the box floor, then stand the pink yogurt cup upright and lay the yellow sandwich flat on the box floor, each directly against the ice pack so they stay cold. The red apple and the brown crackers (lying flat) can go anywhere on the box floor." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/lunch-pack-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/lunch-pack-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once all five items are packed, with the yogurt and the
sandwich against the flat ice pack and everything still, for 2.5 s. Otherwise it scores the partial credit below at
the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_lunch_perishables_touching_ice_pack.py` in
an environment with `yam_arena` installed.

## Scoring

An item is **in the box** when its centre is inside the box's inner outline (24 x 34 cm) and its lowest point is
within 3 mm of the box floor (for the round apple, its centre height minus its 32.5 mm radius, whichever way it has
rolled). **Lying flat** means the item's thinnest dimension is within 10 degrees of vertical
(either face up). The yogurt is **upright** when its axis is within 10 degrees of vertical, opening up.

| Credit | For |
|---|---|
| 0.2 | the ice pack in the box, lying flat |
| 0.3 | the yogurt in the box, upright, its side at most 5 mm from the ice pack's outline (the distance from the point where the cup's axis crosses the pack's mid-height to the pack's outline, minus 27.5 mm), with the ice pack earning its 0.2 |
| 0.3 | the sandwich in the box, lying flat, at most 5 mm from the ice pack's outline (the shortest distance between the two outlines at the pack's mid-height), with the ice pack earning its 0.2 |
| 0.1 | the apple in the box |
| 0.1 | the crackers in the box, lying flat |
| 0 overall | the yogurt or the sandwich has its centre over the box's outer outline (walls included, at any height) without earning its 0.3 |

Gaps are measured horizontally at the height of the ice pack's centre (its mid-height, 1.5 cm above the box floor
when it lies flat). The ice pack's outline is its 9 x 6 cm rectangle at that height. A yogurt leaning within its
10 degrees is measured where its axis crosses that height, not at its own centre. The sandwich's outline there is its
10 x 6 cm rectangle, shifted along its thickness to that height. The zero rule covers
the failure the task is about: a perishable packed more than 5 mm from the ice pack. It also covers a perishable
stacked on the pack or under it, tipped over, packed when the ice pack is missing or standing on edge, or held above
the box. Leaving a perishable out of the box scores less than packing it against the pack, but more than packing it
where it would warm up. Items stacked on other items aren't on the floor, so they earn nothing. Items pushed off the
table, knocked over or left in the air earn nothing.

On a real table, a feeler gauge or a ruler checks the 5 mm gaps and a phone level app checks the tilts. A photo
from above shows that every centre is inside the box.

## The scripted demo

The right arm picks up the ice pack from above, gripping it across its 6 cm width with the fingers closing front to
back. It lays the pack flat in the box, 26 cm in front of the arm bases and 2 cm right of centre, long side running
side to side. It then puts the crackers in the near right corner of the box (nearer the arm bases) the same way and parks above its mat. The left
arm waits above its own mat until then.

The left arm then picks up the sandwich, also gripped across its 6 cm width, so that no finger is on its end. It
sets the sandwich down against the pack's left end, its end 2 mm from the pack. It picks up the yogurt with the
fingers closing side to side, so its far side is free, and stands it against the pack's near side, also 2 mm away.
It then puts the apple in the near left corner of the box and parks above its mat. Every item is let go about 6 mm above
where it lands, once it has settled onto its mat from its start 1 mm above it.

## Runtime and seeds

- The demo's plan takes about 59 s of simulated time; it is packed and still by about 56 s, and the scripted policy
  stops 3 s after the plan, at about 62 s (about 1,240 steps), near half of the 2,400-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
