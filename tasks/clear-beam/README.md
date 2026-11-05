# Clear a photo-eye's beam

A photo-eye and its reflector stand on posts on a diagonal across the table, and four cartons stand near the invisible
line between them. Two of the cartons are in the beam, one only clipping it by 1 cm near the photo-eye; the other two
only look close, 2.0 and 3.8 cm clear of it, and the near miss is the carton nearest the beam's middle. The robot clears
the beam by moving only the two that block it, each at most 10 cm, leaving the others where they stood. Nothing marks
the beam, so the skill is judging a line between two points by eye, and doing only what is needed. It is the step a
packaging line operator takes when a line stops because something broke a photo-eye's beam.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=clear_beam.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clear the beam between the black photo-eye and the red reflector. Move only the cartons that block it, each no more than 10 cm, and leave every carton standing." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clear-beam-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=clear_beam.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clear the beam between the black photo-eye and the red reflector. Move only the cartons that block it, each no more than 10 cm, and leave every carton standing." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/clear-beam-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/clear-beam-dream/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. The trial ends at 1.0 once the beam has been clear, with every carton standing
and still, for 1 s; otherwise it scores the partial credit below at the last step.

## Scoring
The beam is the straight line between the two posts, 4 cm above the table. A carton blocks it if any part of it comes
within 1 cm of that line.

| Score | When, at the end |
|---|---|
| 1.0 | No carton blocks the beam; the two blocking cartons were each moved at most 10 cm; the other two are where they stood (within 5 mm, turned at most 5 degrees); every carton stands upright and still |
| 0.5 | The beam is clear and the other two are where they stood, but a blocking carton was moved more than 10 cm |
| 0.25 | One of the two blocking cartons was moved clear (at most 10 cm) and is still, and the other two are where they stood |
| 0 | A carton that didn't block the beam was moved, any carton is knocked over or off the table, or nothing above holds |

Standing means the 8 cm axis within 10 degrees of vertical, either way up, with the lowest point within 3 mm of the
table; still means moving under 1 cm/s and turning under 0.5 rad/s; off the table means the lowest point more than
1 cm below the table top.

On a real table, a string stretched between the posts shows the beam; a ruler measures each carton's distance from it
and how far each carton moved from its mark.

## The scripted demo
The left arm moves the two blocking cartons, one at a time, straight across the beam to 3 cm clear of it (4.0 and
5.7 cm), each to the side away from its nearer bystander. It grips each carton across its sides 3.5 cm below its top,
with the fingers aimed 52 mm apart so the grip centres it, lifts it 2.5 cm above every top, sets it down slowly and
lets go gently. The other two cartons are never touched.

## Runtime and seeds
- The scripted demo's plan takes 26 s of simulated time (524 steps at 20 Hz), within the 2,400-step limit.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start.
