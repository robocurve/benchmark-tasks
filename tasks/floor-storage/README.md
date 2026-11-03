# Store materials on a floor under construction

One bay of a floor in a building under construction, at 1:50. Three bundles of material lie where they shouldn't:
two beside a hoistway (the shaft of a material hoist), one beside a low parapet at the building's edge. The robot
stores them where US federal construction rules allow (29 CFR 1926.250(b)(1)): not within 6 ft of any hoistway, nor
within 10 ft of an outside wall that does not extend above the top of the material. The instruction names the rule's
source, not the distances. The rule has a condition: the 61 mm wall (a full storey) rises above an 18 mm bundle, so a
bundle may stand right against it, while the 12 mm parapet does not, so bundles stay 10 ft back from it. Nothing on
the table marks where storage is allowed; it follows from the rule and the geometry.

The rule's text, "within 6 feet of any hoistway or inside floor openings", can be read as putting the 6 ft on floor
openings too, or only on hoistways. The scene uses a hoistway, which the 6 ft covers under either reading.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=floor_storage.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This is one bay of a floor in a building under construction, at 1:50 scale. Store the three bundles of material in this bay where US federal construction rules allow." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/floor-storage-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=floor_storage.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "This is one bay of a floor in a building under construction, at 1:50 scale. Store the three bundles of material in this bay where US federal construction rules allow." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/floor-storage-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/floor-storage-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
Distances are at scale (1 ft = 6.1 mm), measured from a bundle's outline seen from above (the hull of its corners, so
a turned bundle is measured by its nearest corner).

A bundle is **stored** when all of these hold:
| Condition | Number | Check on a real table |
|---|---|---|
| lies on any face, on the slab or on another bundle | within 10 degrees of flat, bottom within 2 mm | a square and a ruler |
| its centre over the slab, inside the outside walls | | |
| away from the hoistway's edge | at least 6 ft (36.6 mm) | a ruler from the hoistway's edge |
| away from each outside wall whose top is no higher than the top of the material stored with it | at least 10 ft (61 mm) | a ruler from the parapet |

| Score | Condition |
|---|---|
| 1.0 | all three bundles stored and still for 1 s (ends the trial) |
| 0.67, 0.33 | two or one stored |
| 0 | none |

"The top of the material stored" is the highest top among the bundles stacked or touching with this one. The parapet
(12 mm) is lower than any bundle (18 mm), so the 10 ft limit always applies to it. The full wall (61 mm) applies only
to a stack that reaches its top: three bundles on their 30 mm sides make 90 mm, so a stack like that may not stand
against it. The 10 ft from the parapet is measured to
its nearest point, so a bundle against the full wall but within 10 ft of where the parapet begins is not stored. A
bundle in the hoistway, off the slab or outside the walls scores nothing. `score()` counts a bundle while it moves;
`success()` also needs every bundle still. The hooks don't see the grippers, so a bundle held on the slab counts the
same as one let go.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream-1 gets 1,200 control steps (1 minute at 20 Hz); the scripted demo finishes in about 30 s.
- `python -m pytest -q test_floor_storage.py` checks the scoring and the demo's reach without the simulator.
