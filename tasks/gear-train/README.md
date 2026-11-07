# Mesh a three-gear train, finding each gear's tooth phase by contact

A gearbox assembly step at full size. Three spur gears (62, 42 and 21.8 mm across) lie on the table beside a fixed plate
with three shafts. The robot slides each gear onto its shaft and all the way down, so the medium gear meshes with
both others. The source is a dealer-service step for a rotary cutter's spindle gearbox (a crown gear and its pinion there,
spur gears here): "align gear (1) and gear (5)
so that gear teeth are a match" ([Woods DS8.50 Operator's Manual, page 31, step 11](https://www.manualslib.com/manual/1174793/Woods-Ds8-50.html?page=31)).

The hard part is physical. The bore leaves under 1 mm of play on the shaft, and a gear goes down only when its teeth
line up with the gaps of the gear it meshes with. Out of phase it stops on the other gear's teeth, 7 to 10 mm above
its seat, where it looks almost right from a camera above. The gears have 60, 40 and 20 teeth (module 1 mm), so the
large gear's teeth repeat every 6 degrees, and in a sweep it went down only within about 1 degree of the right angle.
The robot has to find the phase: turn the gear a few degrees and try again, or turn it slowly under light contact. Nothing on the table marks the right angle; it depends on how the
first gear came to rest. On a real bench, forcing a gear that is out of phase chips or burrs its teeth.

## The scripted demo
`demo()` seats the large gear, then the medium and the small ones, with the right arm. It does not rely on knowing
angles or positions to the millimetre: each gear rests on its shaft's tip while the wrist spirals in from 3 mm until
the bore drops over the tip. The medium and small gears then press lightly on the teeth they mesh with and turn
through more than two tooth pitches until they drop into mesh. Each gear is then pushed home.

## Run a trial
On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), with a Dream API key in `~/.dreamlabs/credentials`:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=gear_train.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the gear train on the grey plate: put the large gear on the nearest shaft, the medium gear on the middle shaft and the small gear on the farthest shaft, each pushed all the way down so its teeth mesh with the gear next to it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/gear-train-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=gear_train.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Assemble the gear train on the grey plate: put the large gear on the nearest shaft, the medium gear on the middle shaft and the small gear on the farthest shaft, each pushed all the way down so its teeth mesh with the gear next to it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/gear-train-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/gear-train-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends with 1.0 when `success()` has held for 1 s; otherwise it scores
`score()` at the last step.

## Scoring
| Condition | Number | Check on a real table |
|---|---|---|
| seated | centre within 2 mm of its own shaft's axis, bottom within 1 mm of 6 mm above the table, axis within 3 degrees of vertical | a depth gauge on the gear's top face |
| meshed pair | large and medium, or medium and small, both seated | the two turn together |

| Score | Condition |
|---|---|
| 1.0 | all three gears seated and still for 1 s (ends the trial) |
| 0.5 | one meshed pair: 0.1 for each seated gear plus 0.3 for the pair |
| 0.2 | the large and small gears seated without the medium: no mesh |
| 0.1 | one gear seated |
| 0 | nothing seated: gears stopped on teeth, on the wrong shaft, on a shaft's tip, or lying elsewhere |

Only the medium gear meshes with the others: the large and small shafts are 81 mm apart and those gears' tips reach
42 mm, so seating those two first matches no phase and scores 0.2. Two seated neighbours cannot overlap at seat
height, so a seated pair is meshed. The hooks don't see the grippers, so a gear held seated counts the same as one let go.

## Runtime and seeds
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
- Dream-1 gets 2,400 control steps (2 minutes at 20 Hz); the scripted demo reaches success at about 94 s.
- `python -m pytest -q test_gear_train.py` checks the scoring and the demo's reach without the simulator.

## Fidelity
The plate and gears are the Isaac Lab Factory gear assets at full size, collided by their meshes. The plate is fixed.
The gears weigh 60, 30 and 10 g, lighter than solid plastic parts of this size (solid acetal: about 106, 49 and 13 g).
Every object has rubber-like friction (1.2 static, 1.0 dynamic), so teeth grip each other more than greased metal gears
would.
