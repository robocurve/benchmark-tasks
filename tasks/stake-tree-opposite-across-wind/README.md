# Stake a new tree: two stakes opposite each other, outside the root ball, across the wind

A newly planted tree gets two stakes. They must stand on opposite sides of the trunk, outside the root ball, on a
line at right angles to the wind, which a blue strip on the table marks. No spot is marked: the robot has to work
the two positions out from the trunk, the root-ball disc and the strip. The correct line puts one stake in front of
and to the right of the trunk, the other nearer the arms and to its left, 9 cm left of the midline, which is easier
for the left arm to reach. Both stakes start in a tray in front of the right arm; the scripted demo hands one over
from the right arm to the left through a set-down spot on the midline. The left spot is probably within the right
arm's reach too, so the hand-off is the demo's choice, not a requirement: the score depends only on where the stakes
end up.

## Run a trial

On a machine with the official simulator (`yam-isaac-lab-arena`'s `setup.sh`, or the Brev launchable), with a Dream
API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=stake_tree_opposite_across_wind.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stake the tree: stand the two orange stakes upright on the table on opposite sides of the brown trunk, just outside the tan root-ball disc, so that the line between the two stakes is at right angles to the blue wind strip." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/stake-tree-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=stake_tree_opposite_across_wind.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stake the tree: stand the two orange stakes upright on the table on opposite sides of the brown trunk, just outside the tan root-ball disc, so that the line between the two stakes is at right angles to the blue wind strip." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/stake-tree-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/stake-tree-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once both stakes are placed, opposite each other and
across the wind, and still for 2.5 s; otherwise it scores the partial credit below at the last step.

Offline checks of the scorer and the demo's reach (no GPU): `pytest test_stake_tree_opposite_across_wind.py` in an
environment with `yam_arena` installed.

## Scoring

| Credit | For |
|---|---|
| 0.2 | each stake placed: tilted at most 10 degrees from vertical (either end up), its bottom within 3 mm of the table top (not on the disc, the tray or the other stake), its centre 11 to 16 cm from the trunk's axis in plan |
| 0.2 | both placed and opposite each other: the directions from the trunk's axis to the two centres 168 to 192 degrees apart |
| 0.4 | opposite, and the line through the two centres at 78 to 102 degrees to the strip's long side (the strip's axis only) |
| 0 overall | any part of either stake, seen from above, less than 9 cm from the trunk's axis (over the 18 cm disc) while the stake's lowest point is at most 1.5 cm above the table top: it stands or lies on or against the root ball. This covers an upright stake with its centre under 11 cm from the trunk's axis, and a stake lying with one end across the disc wherever its centre is. A stake held higher over the tree does not trigger this |

Some outcomes: correct, 1.0; opposite but in line with the wind, 0.6; both in the ring on the same side of the trunk,
0.4 by design (the right distance, but no support); one placed and the other left in the tray or at the hand-off spot,
0.2; untouched (both stakes in the tray, about 31 cm from the trunk), 0.

On a real table, a tape measure from the trunk's centre gives each stake's distance (to its centre, and to its
nearest edge for the root-ball rule); a photo from above, with a
protractor laid on it, gives the angle between the two stakes around the trunk and the angle between their line and
the strip; a phone level app gives each stake's tilt.

## The scripted demo

The right arm picks stake B from the tray, gripping its top 3.3 cm from above with the fingers closing side to side,
and stands it at a hand-off spot on the midline 10 cm in front of the arm bases (17 cm from the trunk, outside the
scored ring), letting go 6 mm above the table. It then stands stake A 13 cm from the trunk's axis, in front of and to
the right of it, and parks above the tray. The left arm waits 11 s on its own side until the right arm has left the
hand-off spot, picks stake B up there and stands it 13 cm from the trunk on the other side, nearer the arms and to
the left, then parks in front of its own base. The two stakes end on a line at 45 degrees to the forward direction,
at right angles to the strip.

## Runtime and seeds

- The demo's plan takes 22.1 s of simulated time (the left arm's plan is the longer). Replayed offline, it reaches
  success at 22.7 s including the 2.5 s hold (about 455 steps at 20 Hz), well within the 1,800-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
