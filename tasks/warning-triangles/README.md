# Set out a stopped truck's warning triangles

A box truck has stopped on the shoulder of a two-way road, shown at 1:100. The robot sets out its three warning
triangles where US federal rules put them (49 CFR 392.22(b)(1)): one 10 ft behind on the traffic side, one 100 ft
behind and one 100 ft ahead, the last two in the centre of the shoulder. The instruction names the rule's source, not
the layout. Traffic in the US keeps right and the truck is on the far shoulder, so its lane's traffic comes from the
right; the cab at the left end agrees. The yellow centre line makes it a two-way road, so the divided-road layout (10,
100 and 200 ft, all behind) is wrong. The road is straight and rural, so the exceptions for towns, curves and hills
((b)(2)(iii) and (iv)) don't apply.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=warning_triangles.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The truck has stopped on the shoulder of this straight, two-way rural highway. Set out its three warning triangles where US federal rules require them. The scene is at 1:100 scale." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/warning-triangles-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=warning_triangles.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The truck has stopped on the shoulder of this straight, two-way rural highway. Set out its three warning triangles where US federal rules require them. The scene is at 1:100 scale." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/warning-triangles-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/warning-triangles-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
A case counts when it lies on any face on the road: its bottom within 2 mm of the road, within 10 degrees of flat. Distances
are at scale (1 ft = 3.05 mm), from the truck's rear or front to the case's centre. "Behind" is toward approaching
traffic: the truck faces the way its lane's traffic goes, so behind is its end away from the cab.

| Spot | Along the road | Across the road |
|---|---|---|
| near | 5 to 15 ft behind the rear | beside the truck's traffic side: its centre from one case width out into the far lane to the truck's side line |
| far behind | 75 to 125 ft behind the rear | within 10 mm of the shoulder's centre line |
| far ahead | 75 to 125 ft ahead of the front | within 10 mm of the shoulder's centre line |

| Score | Condition | Check on a real table |
|---|---|---|
| 1.0 | all three spots hold a case and every case is still for 1 s (ends the trial) | a ruler from the truck's ends |
| 0.67, 0.33 | two or one spots filled | |
| 0 | none | |

The cases are identical, so any case can fill any spot, but one case fills only one. The rule says "approximately"
10 and 100 ft, so the bands are half of 10 ft and a quarter of 100 ft either way. All three behind the truck (the
divided-road layout) fills two spots, 0.67; the layout mirrored, as if traffic came from the cab's end, also fills two:
only the near spot depends on the traffic's direction, so getting the direction wrong costs a third.
`score()` counts a case while it moves; `success()` also needs every case still. The hooks don't see the grippers, so
a case held flat on the road counts the same as one let go.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
